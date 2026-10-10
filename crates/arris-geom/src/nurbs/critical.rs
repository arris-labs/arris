//! Critical points of a scalar field over a NURBS surface: every point at
//! which the height above a plane, or the squared distance from a point,
//! is stationary — not the nearest one alone, as
//! [`NurbsSurface::project`] is.
//!
//! On one Bézier patch `S = X / W`, `W > 0`, the gradient of the field is
//! `W²` or `W³` times a pair of polynomials in the patch's own `(u, v)` in
//! Bernstein form, and the critical points are their common zeros — the
//! system [`common_zeros_with`] isolates and certifies (Krawczyk), as the
//! line against a NURBS surface does (`line.rs`):
//!
//! - the height `h = (S − o)·n`: with `H = n·X − (o·n)·W`, the numerators
//!   `A_u = H_u W − H W_u` and `A_v = H_v W − H W_v`;
//! - the squared distance from `p`: with `R = X − p·W`, the numerators
//!   `A_u = Σ R_k (X_k,u W − X_k W_u)` and `A_v` likewise (`W³` times
//!   `(S − p)·S_u` and `(S − p)·S_v`).
//!
//! A zero the isolation certifies is polished on the patch; a box it
//! cannot certify (a zero that is not simple, the field's critical point on
//! a pole's collapsed row or at a fold of its gradient) is descended by
//! Newton's iteration on the surface itself, and kept where the offset
//! has no component along either tangent beyond `tol.linear`. Where the
//! zeros are not isolated — a flat patch, a surface of revolution about the
//! point's axis — the isolation gives up with a front of boxes; each is
//! descended the same way, and stationary points of one value, three or
//! more, found that way on a patch are one *continuum* with the lowest on
//! it. The rounding floor of each system is a multiple of the largest
//! magnitudes that enter it.

use arris_math::{Meter, Point2, Point3, Tolerance, Vec3};

use super::NurbsSurface;
use super::project::Patch;
use crate::bernstein::Binomials;
use crate::bernstein2::{Halving, Poly2, Zero2, common_zeros_with};
use crate::halt::Halt;
use crate::implicit::BERNSTEIN_ROUNDING;
use crate::{GeomError, SurfaceEval};

/// The field whose critical points are asked for.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Field {
    /// `(S − origin)·normal`, `normal` a unit vector.
    Height { origin: Point3, normal: Vec3 },
    /// `|S − point|`.
    Distance { point: Point3 },
}

/// The critical points of a field over a surface, in its own `(u, v)`.
#[derive(Clone, Debug, Default)]
pub(crate) struct Critical {
    /// The isolated critical points, ascending by `u`, then `v`.
    pub(crate) isolated: Vec<Point2>,
    /// One point on each family of critical points of one value, ascending
    /// by value.
    pub(crate) continua: Vec<Point2>,
}

/// How many Newton steps a descent takes. Newton converges quadratically
/// near a simple critical point, so the count is only reached by an
/// iteration that crawls along a fold.
const DESCENT_STEPS: usize = 48;

/// How many stationary points of one value found on one patch after the
/// isolation gave up make a continuum rather than a few isolated points.
/// A structural bound: a field has at most a handful of critical points of
/// one value on a patch, and a family has as many as the front has boxes.
const FAMILY: usize = 3;

/// How many samples along each side of a box tell a flat region: the box
/// the isolation calls flat, whose polynomial is within its floors of zero,
/// is a continuum where the offset is within the tolerance at every one.
const GRID: usize = 4;

impl Field {
    /// The value, the gradient `[f_u, f_v]` and the Hessian
    /// `[[f_uu, f_uv], [f_uv, f_vv]]` of the field at an evaluation, the
    /// distance's as half the square.
    fn jet(&self, e: &SurfaceEval) -> (f64, [f64; 2], [[f64; 2]; 2]) {
        match *self {
            Field::Height { origin, normal } => (
                (e.point - origin).dot(&normal),
                [e.du.dot(&normal), e.dv.dot(&normal)],
                [
                    [e.duu.dot(&normal), e.duv.dot(&normal)],
                    [e.duv.dot(&normal), e.dvv.dot(&normal)],
                ],
            ),
            Field::Distance { point } => {
                let r = e.point - point;
                (
                    0.5 * r.norm_squared(),
                    [r.dot(&e.du), r.dot(&e.dv)],
                    [
                        [
                            e.du.norm_squared() + r.dot(&e.duu),
                            e.du.dot(&e.dv) + r.dot(&e.duv),
                        ],
                        [
                            e.du.dot(&e.dv) + r.dot(&e.duv),
                            e.dv.norm_squared() + r.dot(&e.dvv),
                        ],
                    ],
                )
            }
        }
    }

    /// The offset the field's gradient is, per unit length of each
    /// tangent: zero at a critical point. `None` where a tangent vanishes
    /// (a pole), which says nothing along it.
    fn offsets(&self, e: &SurfaceEval) -> [Option<f64>; 2] {
        let (_, g, _) = self.jet(e);
        [
            (e.du.norm() > 0.0).then(|| g[0] / e.du.norm()),
            (e.dv.norm() > 0.0).then(|| g[1] / e.dv.norm()),
        ]
    }
}

impl NurbsSurface {
    /// Every critical point of `field` over the surface (module docs).
    ///
    /// Errors: [`GeomError::Interrupted`] by the meter, ticked once per
    /// box of the isolation.
    pub(crate) fn critical_points(
        &self,
        field: &Field,
        tol: Tolerance,
        meter: &mut Meter,
    ) -> Result<Critical, GeomError> {
        let mut found: Vec<(Point2, bool)> = Vec::new();
        for patch in self.patches() {
            self.patch_critical(&patch, field, tol, meter, &mut found)?;
        }
        // Points that are one point of space are one: the same critical
        // point found from the two patches on either side of a knot, or at
        // a seam.
        found.sort_by(|a, b| a.0.x.total_cmp(&b.0.x).then(a.0.y.total_cmp(&b.0.y)));
        let mut isolated: Vec<Point2> = Vec::new();
        let mut continua: Vec<Point2> = Vec::new();
        let same = |a: Point2, b: Point2| {
            (self.eval(a.x, a.y).point - self.eval(b.x, b.y).point).norm() <= tol.linear
        };
        for (uv, family) in found {
            let list = if family { &mut continua } else { &mut isolated };
            if !list.iter().any(|&seen| same(seen, uv)) {
                list.push(uv);
            }
        }
        let value = |uv: Point2| field.jet(&self.eval(uv.x, uv.y)).0;
        continua.sort_by(|a, b| value(*a).total_cmp(&value(*b)));
        continua.dedup_by(|a, b| {
            let (x, y) = (
                field_distance(field, self, *a),
                field_distance(field, self, *b),
            );
            (x - y).abs() <= tol.linear
        });
        Ok(Critical { isolated, continua })
    }

    fn patch_critical(
        &self,
        patch: &Patch,
        field: &Field,
        tol: Tolerance,
        meter: &mut Meter,
        out: &mut Vec<(Point2, bool)>,
    ) -> Result<(), GeomError> {
        let system = patch_system(patch, field);
        let global = |s: [f64; 2]| {
            [
                patch.u[0] + s[0] * (patch.u[1] - patch.u[0]),
                patch.v[0] + s[1] * (patch.v[1] - patch.v[0]),
            ]
        };
        let (zeros, gave_up) = match common_zeros_with(
            [&system.poly[0], &system.poly[1]],
            system.floors,
            &[],
            Halving::LookAhead,
            meter,
        ) {
            Ok(isolation) => (isolation.zeros, false),
            Err(Halt::Stopped(stop)) => return Err(GeomError::Interrupted(stop)),
            Err(Halt::Fault(continuum)) => {
                let mut all = continuum.found;
                all.extend(continuum.front);
                (all, true)
            }
        };
        let mut stationary: Vec<Point2> = Vec::new();
        for z in &zeros {
            if let Some(uv) = self.settle_zero(z, patch, field, tol, &global) {
                stationary.push(uv);
            }
        }
        // A flat box (the whole patch of a plane parallel to the field's,
        // a sphere about the point) is a continuum where the offset is
        // within the tolerance everywhere on it.
        let mut family: Vec<Point2> = Vec::new();
        for z in &zeros {
            let corners = (0..=GRID).flat_map(|i| (0..=GRID).map(move |j| (i, j)));
            let flat = corners.clone().all(|(i, j)| {
                let s = [
                    z.lo[0] + (z.hi[0] - z.lo[0]) * i as f64 / GRID as f64,
                    z.lo[1] + (z.hi[1] - z.lo[1]) * j as f64 / GRID as f64,
                ];
                let g = global(s);
                let e = self.eval(g[0], g[1]);
                field
                    .offsets(&e)
                    .iter()
                    .all(|o| o.is_none_or(|o| o.abs() <= tol.linear))
            });
            if flat {
                let g = global(z.at);
                family.push(Point2::new(g[0], g[1]));
            }
        }
        if !family.is_empty() {
            out.extend(family.iter().map(|&uv| (uv, true)));
            return Ok(());
        }
        // Stationary points of one value found after the isolation gave up
        // are a family.
        if gave_up && stationary.len() >= FAMILY {
            let value = |uv: Point2| field_distance(field, self, uv);
            let mut grouped = vec![false; stationary.len()];
            for i in 0..stationary.len() {
                let near: Vec<usize> = (0..stationary.len())
                    .filter(|&j| (value(stationary[j]) - value(stationary[i])).abs() <= tol.linear)
                    .collect();
                if near.len() >= FAMILY && !grouped[i] {
                    out.push((stationary[near[0]], true));
                    for j in near {
                        grouped[j] = true;
                    }
                }
            }
            for (uv, g) in stationary.iter().zip(grouped) {
                if !g {
                    out.push((*uv, false));
                }
            }
            return Ok(());
        }
        out.extend(stationary.into_iter().map(|uv| (uv, false)));
        Ok(())
    }

    /// A zero or an uncertified box as a stationary point of the surface,
    /// in the surface's own parameters, or `None`.
    fn settle_zero(
        &self,
        z: &Zero2,
        patch: &Patch,
        field: &Field,
        tol: Tolerance,
        global: &impl Fn([f64; 2]) -> [f64; 2],
    ) -> Option<Point2> {
        let start = global(z.at);
        let inside = |uv: [f64; 2]| {
            // Within the patch's box, widened by the box's own width: a
            // zero on a knot line is the neighbour's too, found from either.
            let w = [patch.u[1] - patch.u[0], patch.v[1] - patch.v[0]];
            uv[0] >= patch.u[0] - 1e-9 * w[0]
                && uv[0] <= patch.u[1] + 1e-9 * w[0]
                && uv[1] >= patch.v[0] - 1e-9 * w[1]
                && uv[1] <= patch.v[1] + 1e-9 * w[1]
        };
        let [du, dv] = self.domain();
        let clamp = |uv: [f64; 2]| [uv[0].clamp(du.lo(), du.hi()), uv[1].clamp(dv.lo(), dv.hi())];
        let mut uv = clamp(start);
        for _ in 0..DESCENT_STEPS {
            let e = self.eval(uv[0], uv[1]);
            let (_, g, h) = field.jet(&e);
            let det = h[0][0] * h[1][1] - h[0][1] * h[1][0];
            if det == 0.0 || !det.is_finite() {
                break;
            }
            let step = [
                (g[0] * h[1][1] - g[1] * h[0][1]) / det,
                (h[0][0] * g[1] - h[1][0] * g[0]) / det,
            ];
            if !(step[0].is_finite() && step[1].is_finite()) {
                break;
            }
            let next = clamp([uv[0] - step[0], uv[1] - step[1]]);
            let moved = (next[0] - uv[0]).abs().max((next[1] - uv[1]).abs());
            uv = next;
            if moved <= f64::EPSILON * (1.0 + uv[0].abs().max(uv[1].abs())) {
                break;
            }
        }
        let e = self.eval(uv[0], uv[1]);
        let stationary = field
            .offsets(&e)
            .iter()
            .all(|o| o.is_none_or(|o| o.abs() <= tol.linear));
        (stationary && inside(uv)).then(|| Point2::new(uv[0], uv[1]))
    }
}

/// The field's value at `uv`: the distance itself for `Distance`, so that
/// continua of one distance are compared as the tolerance reads them.
fn field_distance(field: &Field, surface: &NurbsSurface, uv: Point2) -> f64 {
    let e = surface.eval(uv.x, uv.y);
    match field {
        Field::Height { .. } => field.jet(&e).0.abs(),
        Field::Distance { point } => (e.point - point).norm(),
    }
}

/// The gradient numerators of a patch.
struct System {
    poly: [Poly2; 2],
    floors: [f64; 2],
}

/// The field's gradient on a patch as two polynomials (module docs).
fn patch_system(patch: &Patch, field: &Field) -> System {
    let [p, q] = patch.degree;
    let b = Binomials::new(3 * (p + q) + 4);
    let coord = |k: usize| -> Poly2 {
        Poly2::new(
            [p, q],
            patch
                .net
                .iter()
                .map(|h| [h.x, h.y, h.z][k])
                .collect::<Vec<_>>(),
        )
    };
    let x = [coord(0), coord(1), coord(2)];
    let w = Poly2::new([p, q], patch.net.iter().map(|h| h.w).collect());
    let (mx, mw) = patch.net.iter().fold((0.0f64, 0.0f64), |(mx, mw), h| {
        (mx.max(h.x.abs()).max(h.y.abs()).max(h.z.abs()), mw.max(h.w))
    });
    let d = p.max(q) as f64;
    // `U_k = X_k,u W − X_k W_u` and its `v` twin; `N = Σ c_k U_k` over a
    // combination of the coordinates.
    let numerators = |value: &Poly2| -> [Poly2; 2] {
        let across = |f: &Poly2, g: &Poly2, df: &Poly2, dg: &Poly2| {
            Poly2::combine(&[(1.0, &df.mul(g, &b)), (-1.0, &f.mul(dg, &b))])
        };
        [
            across(value, &w, &value.du(), &w.du()),
            across(value, &w, &value.dv(), &w.dv()),
        ]
    };
    match *field {
        Field::Height { origin, normal } => {
            let shift = origin.coords.dot(&normal);
            let h = Poly2::combine(&[
                (normal.x, &x[0]),
                (normal.y, &x[1]),
                (normal.z, &x[2]),
                (-shift, &w),
            ]);
            let [au, av] = numerators(&h);
            let magnitude = 3.0 * mx + shift.abs() * mw;
            let floor = BERNSTEIN_ROUNDING * 4.0 * d * magnitude * mw;
            System {
                poly: [au, av],
                floors: [floor; 2],
            }
        }
        Field::Distance { point } => {
            let r: [Poly2; 3] = [0, 1, 2].map(|k| Poly2::combine(&[(1.0, &x[k]), (-point[k], &w)]));
            let n: [[Poly2; 2]; 3] = [0, 1, 2].map(|k| numerators(&x[k]));
            let sum = |dir: usize| {
                let products: Vec<Poly2> = (0..3).map(|k| r[k].mul(&n[k][dir], &b)).collect();
                Poly2::combine(&[
                    (1.0, &products[0]),
                    (1.0, &products[1]),
                    (1.0, &products[2]),
                ])
            };
            let reach = mx + point.coords.norm() * mw;
            let floor = BERNSTEIN_ROUNDING * 3.0 * reach * 4.0 * d * mx * mw;
            System {
                poly: [sum(0), sum(1)],
                floors: [floor; 2],
            }
        }
    }
}
