//! The section of a torus or a NURBS surface with an analytic surface,
//! looked for in the walked surface's own `(u, v)`: the other surface's
//! implicit polynomial over its parametrisation, and the points of it that
//! carry the section's topology.
//!
//! A torus has no rulings to walk, so the other surface's `F` is put
//! into the torus instead: `F(P(u, v)) = 0` is the section, a curve in
//! the parameter plane. The torus is cut into sixteen patches, a quarter
//! turn in `u` by a quarter turn in `v`, and on each a quarter turn is
//! the rational quadratic arc with weights `(1, cos π/4, 1)` — the
//! half-angle parametrisation about the quarter's middle,
//! `tan((θ − θ_mid) / 2) = tan(π/8)·(2s − 1)` — so the patch has
//! homogeneous coordinates of bidegree (2, 2) with a weight `w` between
//! `cos² π/8` and one. [`Implicit::along`] over them is a tensor
//! Bernstein polynomial `f = wᵈ·F(P)` (`crate::bernstein2`): bidegree
//! (2, 2) against a plane, (4, 4) against a quadric, (8, 8) against
//! another torus.
//!
//! A NURBS surface is cut into its Bézier patches instead, one per pair
//! of knot spans (ADR-0061). Each is already a rational patch in
//! homogeneous coordinates `(w·P, w)`, of the surface's bidegree `(p, q)`
//! and linear in its span's parameters, so the same substitution gives
//! `f` of bidegree `(d·p, d·q)` with the net's own weight `w`. A
//! [`Chart`] says how a patch's `(s, t)` stand for the walk's `(u, v)` and
//! which directions close; a NURBS surface's parameters are mapped onto
//! the walk's by a [`Rescale`], each direction's domain onto `[0, 2π]`.
//!
//! Two sets of points are isolated on every patch, and merged across
//! the patches' shared edges and the two seams:
//!
//! - the **turning points**, `f = f_t = 0`, where the section is
//!   parallel to `v` and the number of `v` that solve `f(u, ·) = 0`
//!   changes. On `f = 0` the weight drops out of `f_t`, so they are the
//!   turning points of `F(P(u, v))` whatever the patch. Every component
//!   of the section either has one, winds round `u`, or ends on an edge
//!   of a NURBS surface's domain.
//! - the **critical points** of `F(P(u, v))` near the section: the zeros
//!   of `w·f_s − d·w_s·f = w^(d+1)·∂F/∂s` and of the same in `t`, which
//!   are free of the weight *off* the section as well, so that a near
//!   miss is the same point from either side of a patch edge. `f` only
//!   gates them, and not at its rounding but at what a distance of
//!   `tol.linear` makes of it ([`Implicit::steepness`]): whether one is
//!   a singular point of the section is not decided on `f`, which
//!   carries a factor the size of the other surface to the power `d`,
//!   but in length, on [`Implicit::distance`] at the point.
//!
//! A tube circle of the torus that lies on the other surface is a line
//! `s = s₀` of a column on which `f` vanishes: neither set is isolated
//! along it. It is looked for first ([`PatchedSection::tube_circle_candidates`])
//! and divided out ([`PatchedSection::deflate`]), and both sets are then
//! those of the quotient, the rest of the section. A NURBS surface's
//! isoline `u = u₀` on the other surface is the same line of a patch's
//! column, divided out of the patches whose span holds it, and the walk
//! evaluates the quotient on those patches ([`Quotient`]).

use core::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, TAU};

use arris_math::{Meter, Point3, Tolerance, Vec3};

use crate::arc::{QUARTER_TAN, quarter_angle, quarter_arc, quarter_param, quarter_weight};
use crate::bernstein::{self, Binomials, sign_change_candidates};
use crate::bernstein2::{
    Continuum, Gate, Isolation, Poly2, Wrap, Zero2, common_zeros, merge_wrapped,
};
use crate::halt::Halt;
use crate::implicit::{BERNSTEIN_ROUNDING, Implicit};
use crate::{NurbsSurface, Surface};

/// What the angles of two patches' shared edge may differ by, each
/// rounded on its own way through [`quarter_angle`]: a few `ε` of a turn.
/// A statement about `f64`, never a tolerance.
const ANGLE_ROUNDING: f64 = 4.0 * f64::EPSILON * TAU;

/// Where a line of the torus at `t` of a quarter turn in `v` is looked
/// along for the tube circles that lie on the other surface: every such
/// circle crosses it. A place with nothing special about it, not a
/// tolerance.
const CIRCLE_ROW: f64 = 0.37;

/// How far outside a column's `[0, 1]` a tube circle's parameter may lie
/// for the column still to be divided by it: a circle that near the
/// column's edge leaves the polynomial too small beside it to keep a sign
/// above its rounding. A ratio of the column's width.
const DIVIDED_WITHIN: f64 = 0.5;

/// The quarter turns a range of angles lies over, each with the part of
/// its `[0, 1]` the range covers and whether it is an odd number of whole
/// turns from `[0, 2π)`.
fn quarters([lo, hi]: [f64; 2]) -> Vec<(usize, [f64; 2], bool)> {
    let first = (lo / FRAC_PI_2).floor();
    let last = (hi / FRAC_PI_2).floor().max(first);
    let count = (last - first) as usize + 1;
    (0..count.min(5))
        .map(|k| {
            let start = (first + k as f64) * FRAC_PI_2;
            let from = if k == 0 {
                quarter_param(lo - start)
            } else {
                0.0
            };
            let to = if k + 1 == count {
                quarter_param(hi - start)
            } else {
                1.0
            };
            let quarter = (first + k as f64).rem_euclid(4.0) as usize;
            let odd = ((first + k as f64) / 4.0).floor().rem_euclid(2.0) == 1.0;
            (quarter, [from, to.max(from)], odd)
        })
        .collect()
}

/// How far outside a NURBS patch's `[0, 1]` a zero its isolation found
/// may lie and still be the patch's, put on its edge: a zero on a knot
/// line, polished from either side, lands a few `ε` past it, and the
/// patch's polynomial a billionth of its span past a knot is the next
/// patch's to a billionth of its slope's jump. A ratio of the span, not
/// a tolerance.
const PATCH_SLACK: f64 = 1e-9;

/// How deep a NURBS patch is looked at past an edge of the domain for a
/// critical point the tolerance may not tell from one on the edge
/// ([`PatchedSection::critical_points_past`]), as a fraction of its span:
/// the margin [`crate::bernstein2`] grows a box by to certify a zero,
/// within which a polynomial's extrapolated rounding grows by no more than
/// a few times. A ratio, not a tolerance.
const EDGE_SLAB: f64 = 0.125;

/// What a control point moved into the other surface's frame may be off
/// by, as a share of its terms before they cancel: a rotation's three
/// products and two sums, and the shift added. A statement about `f64`,
/// never a tolerance.
const MOVE_ROUNDING: f64 = 4.0 * f64::EPSILON;

/// How the patches stand for the walked surface's parameters.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Chart {
    /// A torus: sixteen quarter turns, patch `4·qu + qv`, each direction's
    /// `s` an angle through [`quarter_angle`], both closing over `2π`.
    Quarters,
    /// A NURBS surface: a patch per pair of knot spans, row-major over
    /// `u` with `columns` spans of `v` to a row, `s` linear over the
    /// patch's span, in the walk's parameters ([`Rescale`]). A direction
    /// closes over `2π` where the surface's [`NurbsSurface::closure`]
    /// says it closes, and is bounded by `[0, 2π]` where it does not.
    Knots {
        /// The spans of `v`, how many patches a row of `u` holds.
        columns: usize,
        /// Whether each direction closes.
        closed: [bool; 2],
        /// The map from the surface's parameters to the walk's.
        rescale: Rescale,
    },
}

/// A NURBS surface's parameters as the walk takes them: each direction's
/// domain mapped linearly onto `[0, 2π]`, so that a direction that closes
/// closes over `2π` as a torus's angles do, and every size the torus walk
/// measures in radians — a cell, a step, a slack — is the same share of
/// any surface's domain whatever its knots' units. The torus's own angles
/// are taken as they are.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Rescale {
    /// Where each direction's domain starts and ends.
    lo: [f64; 2],
    hi: [f64; 2],
    /// What `2π` of the walk stands for: the closure, or the domain's
    /// length.
    len: [f64; 2],
}

impl Rescale {
    /// The map of `nurbs`'s domain.
    pub(crate) fn of(nurbs: &NurbsSurface) -> Rescale {
        let domain = nurbs.domain();
        let closure = nurbs.closure();
        Rescale {
            lo: domain.map(|d| d.lo()),
            hi: domain.map(|d| d.hi()),
            len: [0, 1].map(|k| closure[k].unwrap_or(domain[k].length())),
        }
    }

    /// The walk's parameter of direction `k` at the surface's `x`; the
    /// domain's ends go to `0` and `2π` exactly.
    pub(crate) fn to_walk(self, k: usize, x: f64) -> f64 {
        if x == self.hi[k] && self.hi[k] - self.lo[k] == self.len[k] {
            TAU
        } else {
            (x - self.lo[k]) / self.len[k] * TAU
        }
    }

    /// The surface's parameter of direction `k` at the walk's `w`; `0` and
    /// `2π` go to the domain's ends exactly, and a closed direction's `w`
    /// past them to the surface's parameter as far past.
    pub(crate) fn to_surface(self, k: usize, w: f64) -> f64 {
        if w == TAU && self.hi[k] - self.lo[k] == self.len[k] {
            self.hi[k]
        } else {
            self.lo[k] + w / TAU * self.len[k]
        }
    }

    /// What a derivative in the surface's parameter of direction `k` is
    /// multiplied by to be one in the walk's.
    pub(crate) fn factor(self, k: usize) -> f64 {
        self.len[k] / TAU
    }
}

/// What [`PatchedSection::sign_over`] looks at over a box.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Probe {
    /// The section's polynomial `f`: no point of the section in the box.
    F,
    /// `∂f/∂s`: at most one point of the section on every line of
    /// constant `v` through the box.
    Fs,
    /// `∂f/∂t`: at most one on every line of constant `u`.
    Ft,
    /// `∂²F/∂v²` of the other surface's polynomial over the torus: no
    /// more than two on a line of constant `u`, either side of the one
    /// zero of `∂F/∂v`.
    Fvv,
}

/// One patch: the other surface's polynomial over it, and its weight.
struct Patch {
    /// The quarter turns of a torus's patch; the spans' indices, `u`'s and
    /// `v`'s, of a NURBS surface's.
    cell: [usize; 2],
    /// The box of the walk's parameters the patch covers: a torus's
    /// angles, a NURBS surface's through its [`Rescale`].
    span: [[f64; 2]; 2],
    f: Poly2,
    weight: Poly2,
    /// The `u₀` an isoline of a NURBS surface was divided out of the
    /// patch at, a closure past the isoline's own where the patch holds
    /// it there ([`PatchedSection::deflate`]).
    divided: Option<f64>,
    /// The rounding of a coefficient of `f`: the section's, until a
    /// division raises it on the patches it divides.
    floor: f64,
}

/// The other surface's polynomial over the patches of a torus or a NURBS
/// surface.
pub(crate) struct PatchedSection<'a> {
    /// The other surface.
    pub(crate) implicit: Implicit<'a>,
    chart: Chart,
    patches: Vec<Patch>,
    /// The degree the binomial tables the patches' products take reach
    /// to.
    binomial_degree: usize,
    /// The rounding of a coefficient of `f`, the largest of the patches'
    /// (`Patch::floor`).
    floor: f64,
    /// What `|f|` is no larger than within `tol.linear` of the other
    /// surface, rounding included.
    near: f64,
    /// `near` without the rounding: what a distance of `tol.linear` makes
    /// of `f`.
    steep: f64,
    /// The polynomial changes sign with every whole turn of `u`: a tube
    /// circle it crossed along has been divided out of it
    /// ([`Self::deflate`]).
    odd_turn: bool,
}

/// The section's polynomial over a NURBS surface with one isoline `u = u₀`
/// divided out of it ([`PatchedSection::deflate`]), owned by the walk: at a
/// point of the walk's parameters, `F / gᵐ` in the sign the patches give
/// it, `F` the other surface's polynomial and `g` the isoline's factor,
/// no larger than one. Evaluated on the patches the isolation certified,
/// a divided patch's quotient over `s − s₀` times the smooth ratio of
/// that factor to `g` — never a small number over a small number — so
/// that the walk and the isolation have the same zeros to rounding.
#[derive(Debug, Clone)]
pub(crate) struct Quotient {
    u0: f64,
    order: usize,
    /// Whether each direction closes.
    closed: [bool; 2],
    /// The spans of `u` and of `v`, ascending.
    rows: Vec<[f64; 2]>,
    columns: Vec<[f64; 2]>,
    /// The other surface's degree.
    degree: i32,
    /// Row-major over `u`: each patch's polynomial, weight, and where the
    /// isoline was divided out of it.
    patches: Vec<(Poly2, Poly2, Option<f64>)>,
}

impl Quotient {
    /// `F / gᵐ` at the walk's `(u, v)`, `u` as it comes — unwrapped, so
    /// that for an odd order the value changes sign with a whole turn
    /// where `u` closes.
    pub(crate) fn at(&self, u: f64, v: f64) -> f64 {
        let (base, turns) = if self.closed[0] {
            let k = (u / TAU).floor();
            (u - k * TAU, k)
        } else {
            (u, 0.0)
        };
        let v = if self.closed[1] { v.rem_euclid(TAU) } else { v };
        let find = |spans: &[[f64; 2]], x: f64| {
            spans
                .partition_point(|s| s[1] < x)
                .min(spans.len().saturating_sub(1))
        };
        let (i, j) = (find(&self.rows, base), find(&self.columns, v));
        let (Some((f, w, home)), Some(&[lo, hi]), Some(&[c, d])) = (
            self.patches.get(i * self.columns.len() + j),
            self.rows.get(i),
            self.columns.get(j),
        ) else {
            return f64::NAN;
        };
        let (s, t) = ((base - lo) / (hi - lo), (v - c) / (d - c));
        let m = self.order as i32;
        let scale = match home {
            // `|s − s₀| / |g|`, smooth through the isoline.
            Some(r) => {
                let half = 0.5 * (base - r);
                let ratio = if !self.closed[0] {
                    TAU
                } else if half == 0.0 {
                    2.0
                } else {
                    2.0 * half / half.sin()
                };
                (ratio.abs() / (hi - lo)).powi(m)
            }
            None => {
                let g = if self.closed[0] {
                    (0.5 * (base - self.u0)).sin()
                } else {
                    (base - self.u0) / TAU
                };
                g.abs().recip().powi(m)
            }
        };
        let flips = self.order % 2 == 1 && turns.rem_euclid(2.0) == 1.0;
        let sign = if flips { -1.0 } else { 1.0 };
        sign * f.eval(s, t) * scale / w.eval(s, t).powi(self.degree)
    }
}

/// The points isolated over the whole walked surface, in its parameters.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Census {
    /// `at` is `(u, v)` in the walk's parameters, ascending: angles in
    /// `[0, 2π)²` on a torus, and on a NURBS surface its own through its
    /// [`Rescale`], in `[0, 2π]` and a closed direction's in `[0, 2π)`. A
    /// box is in the same parameters, around its `at`, and may reach past
    /// a seam.
    pub(crate) zeros: Vec<Zero2>,
    /// The deepest level any patch was subdivided to.
    pub(crate) depth: usize,
    /// How many boxes were looked at, all patches together.
    pub(crate) boxes: usize,
}

impl<'a> PatchedSection<'a> {
    /// `None` unless `torus` is a torus and `other` has an implicit
    /// form, which a NURBS surface has not.
    pub(crate) fn new(torus: &Surface, other: &'a Surface, tol: Tolerance) -> Option<Self> {
        let Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        } = torus
        else {
            return None;
        };
        let implicit = Implicit::of(other)?;
        let (major, minor) = (*major_radius, *minor_radius);
        // The torus's frame in the other surface's: its axes as columns
        // and its origin.
        let axes = [frame.x(), frame.y(), frame.z()]
            .map(|axis| implicit.frame.vec_to_local(axis.into_inner()));
        let origin = implicit.frame.to_local(frame.origin()).coords;
        let degree = implicit.degree();
        let binomials = Binomials::new(4 * degree);

        let mut patches = Vec::with_capacity(16);
        for qu in 0..4 {
            let [cos_u, sin_u, w_u] = quarter_arc(qu);
            for qv in 0..4 {
                let [cos_v, sin_v, w_v] = quarter_arc(qv);
                // w·(R + r cos v), w·r sin v and w itself along v.
                let across: Vec<f64> = (0..3).map(|j| major * w_v[j] + minor * cos_v[j]).collect();
                let up: Vec<f64> = sin_v.iter().map(|s| minor * s).collect();
                let in_torus = [
                    Poly2::outer(&cos_u, &across),
                    Poly2::outer(&sin_u, &across),
                    Poly2::outer(&w_u, &up),
                ];
                let weight = Poly2::outer(&w_u, &w_v);
                let local = [0, 1, 2].map(|k| {
                    Poly2::combine(&[
                        (axes[0][k], &in_torus[0]),
                        (axes[1][k], &in_torus[1]),
                        (axes[2][k], &in_torus[2]),
                        (origin[k], &weight),
                    ])
                });
                let f = implicit.along([&local[0], &local[1], &local[2], &weight], &binomials);
                let quarter = |q: usize| [q as f64 * FRAC_PI_2, (q + 1) as f64 * FRAC_PI_2];
                patches.push(Patch {
                    cell: [qu, qv],
                    span: [quarter(qu), quarter(qv)],
                    f,
                    weight,
                    divided: None,
                    floor: 0.0,
                });
            }
        }
        // No point of the torus is farther from the other frame's origin,
        // and no weight is above one.
        let reach = major + minor + origin.norm();
        let floor = BERNSTEIN_ROUNDING * implicit.magnitude(reach, 1.0);
        let steep = implicit.steepness(reach + tol.linear) * tol.linear;
        for patch in &mut patches {
            patch.floor = floor;
        }
        Some(PatchedSection {
            implicit,
            chart: Chart::Quarters,
            patches,
            binomial_degree: 4 * degree + 4,
            floor,
            near: floor + steep,
            steep,
            odd_turn: false,
        })
    }

    /// The section of `nurbs` with `other` over the Bézier patches of
    /// `nurbs` (ADR-0061); `None` where `other` has no implicit form,
    /// which a NURBS surface has not.
    ///
    /// Each patch's homogeneous net is moved into the other surface's
    /// frame and put into its polynomial. The rounding floor is the
    /// polynomial's magnitude at the largest homogeneous coordinate and
    /// weight of any patch, and what the move's own rounding — a few `ε`
    /// of the terms before they cancelled — adds to that magnitude; what
    /// a distance of `tol.linear` makes of `f` is the other surface's
    /// steepness at the farthest control point times the largest weight
    /// to the degree: a patch lies inside its control points' hull, and
    /// `f = wᵈ·F`. The patches' spans are in the walk's parameters
    /// ([`Rescale`]).
    pub(crate) fn on_nurbs(
        nurbs: &NurbsSurface,
        other: &'a Surface,
        tol: Tolerance,
    ) -> Option<Self> {
        let implicit = Implicit::of(other)?;
        let degree = implicit.degree();
        let [p, q] = nurbs.degree();
        let binomials = Binomials::new(degree * p.max(q));
        // The other frame's origin seen from it: a homogeneous point
        // `(X, w)` is `R·X + w·shift` there.
        let shift = implicit.frame.to_local(Point3::origin()).coords;
        let bezier = nurbs.patches();
        let rescale = Rescale::of(nurbs);
        let mut columns_seen: Vec<f64> = Vec::new();
        let mut rows_seen: Vec<f64> = Vec::new();
        let (mut reach, mut heaviest, mut farthest) = (0.0f64, 0.0f64, 0.0f64);
        let mut before = 0.0f64;
        let mut patches = Vec::with_capacity(bezier.len());
        for patch in &bezier {
            let [pu, pv] = patch.degree;
            let mut coords: [Vec<f64>; 3] = [Vec::new(), Vec::new(), Vec::new()];
            let mut weights = Vec::with_capacity(patch.net.len());
            for h in &patch.net {
                let turned = implicit.frame.vec_to_local(Vec3::new(h.x, h.y, h.z));
                let moved = turned + h.w * shift;
                for (k, c) in coords.iter_mut().enumerate() {
                    c.push(moved[k]);
                }
                weights.push(h.w);
                reach = reach.max(moved.norm());
                // The terms before they cancel: a net a kilometre out,
                // moved into a frame beside it, keeps a rounding of a
                // kilometre's `ε` in every control point.
                before = before.max(turned.norm() + h.w * shift.norm());
                heaviest = heaviest.max(h.w);
                farthest = farthest.max(moved.norm() / h.w);
            }
            let [x, y, z] = coords.map(|c| Poly2::new([pu, pv], c));
            let weight = Poly2::new([pu, pv], weights);
            let f = implicit.along([&x, &y, &z, &weight], &binomials);
            let index = |seen: &mut Vec<f64>, lo: f64| match seen.iter().position(|&k| k == lo) {
                Some(i) => i,
                None => {
                    seen.push(lo);
                    seen.len() - 1
                }
            };
            patches.push(Patch {
                cell: [
                    index(&mut rows_seen, patch.u[0]),
                    index(&mut columns_seen, patch.v[0]),
                ],
                span: [0, 1].map(|k| {
                    let [a, b] = [patch.u, patch.v][k];
                    [rescale.to_walk(k, a), rescale.to_walk(k, b)]
                }),
                f,
                weight,
                divided: None,
                floor: 0.0,
            });
        }
        // Every coefficient is a sum of terms each bounded by a monomial
        // of the coordinates' bound, so a coordinate off by `moved` moves
        // it by no more than the magnitude grows by.
        let moved = MOVE_ROUNDING * before;
        let magnitude = implicit.magnitude(reach, heaviest);
        let floor = BERNSTEIN_ROUNDING * magnitude
            + (implicit.magnitude(reach + moved, heaviest) - magnitude);
        let steep =
            implicit.steepness(farthest + tol.linear) * tol.linear * heaviest.powi(degree as i32);
        for patch in &mut patches {
            patch.floor = floor;
        }
        Some(PatchedSection {
            implicit,
            chart: Chart::Knots {
                columns: columns_seen.len(),
                closed: nurbs.closure().map(|c| c.is_some()),
                rescale,
            },
            patches,
            // `∂²F/∂v²`'s numerator, [`Probe::Fvv`], is the largest
            // product: `f` times the weight twice.
            binomial_degree: (degree + 2) * p.max(q) + 2,
            floor,
            near: floor + steep,
            steep,
            odd_turn: false,
        })
    }

    /// The pole of a NURBS surface at `v0` — an edge of its domain in `v`,
    /// where the row is collapsed onto one point that the other surface
    /// holds — divided out of the polynomial (ADR-0061 §3, amended for a
    /// walked blend's pole), and the order it was divided out at: one where
    /// the other surface crosses the point and two where it is tangent
    /// there. Along the row the polynomial is the point's value times a
    /// positive weight, so it vanishes on the whole edge, though the
    /// section has one point there and no curve. The order is how many of
    /// the leading rows of coefficients at the edge are within what
    /// `tol.linear` makes of `f` on every patch along it, at most two: what
    /// the division drops is within that over the patch, as for an isoline
    /// ([`Self::deflate`]). The patches along the edge are divided by their
    /// own linear factor at it, `t` or `1 − t`; the others keep their
    /// polynomial, which the factor, positive inside the domain, does not
    /// change the sign of.
    pub(crate) fn deflate_pole(&mut self, v0: f64) -> usize {
        let binomials = Binomials::new(self.binomial_degree);
        // The patches along the edge, each transposed so that `v` runs
        // first, with the end of its `[0, 1]` the edge is at.
        let along: Vec<(usize, f64)> = (self.patches.iter().enumerate())
            .filter_map(|(i, patch)| {
                let [lo, hi] = patch.span[1];
                if lo == v0 {
                    Some((i, 0.0))
                } else if hi == v0 {
                    Some((i, 1.0))
                } else {
                    None
                }
            })
            .collect();
        let near = self.near;
        let vanishing = |f: &Poly2, root: f64| {
            let [m, n] = f.degree();
            let row = |i: usize| {
                let i = if root == 0.0 { i } else { m - i };
                (0..=n).all(|j| f.coefficients()[i * (n + 1) + j].abs() <= near)
            };
            (0..m.min(2)).take_while(|&i| row(i)).count()
        };
        let order = (along.iter())
            .map(|&(i, root)| vanishing(&self.patches[i].f.transposed(), root))
            .min()
            .unwrap_or(0);
        let mut worst = self.floor;
        for (i, root) in along {
            let patch = &mut self.patches[i];
            let mut f = patch.f.transposed();
            let mut floor = patch.floor;
            for _ in 0..order {
                let (quotient, carried) = f.over_linear(root, floor, &binomials);
                f = quotient;
                floor = carried;
            }
            patch.f = f.transposed();
            patch.floor = floor;
            worst = worst.max(floor);
        }
        self.floor = worst;
        self.near = worst + self.steep;
        order
    }

    /// The parameter of direction `k` at `s` of `patch`.
    fn param(&self, patch: &Patch, k: usize, s: f64) -> f64 {
        match self.chart {
            Chart::Quarters => quarter_angle(patch.cell[k], s),
            Chart::Knots { .. } => {
                let [lo, hi] = patch.span[k];
                lo * (1.0 - s) + hi * s
            }
        }
    }

    /// The patch at `cell`.
    fn patch_at(&self, [cu, cv]: [usize; 2]) -> Option<&Patch> {
        let columns = match self.chart {
            Chart::Quarters => 4,
            Chart::Knots { columns, .. } => columns,
        };
        self.patches.get(cu * columns + cv)
    }

    /// The cells of direction `k` a range of its parameter lies over, each
    /// with the part of its `[0, 1]` the range covers and whether it is an
    /// odd number of whole turns from the torus's `[0, 2π)`. On a NURBS
    /// surface a closed direction's range is taken a closure either way as
    /// well, a closure from `[0, 2π)` being one turn, and the part of a
    /// range outside the domain has no cell.
    fn cells(&self, k: usize, range: [f64; 2]) -> Vec<(usize, [f64; 2], bool)> {
        let Chart::Knots { closed, .. } = self.chart else {
            return quarters(range);
        };
        let mut spans: Vec<[f64; 2]> = Vec::new();
        for patch in &self.patches {
            let cell = patch.cell[k];
            if spans.len() <= cell {
                spans.resize(cell + 1, [f64::NAN; 2]);
            }
            spans[cell] = patch.span[k];
        }
        let turns: Vec<f64> = if closed[k] {
            vec![-TAU, 0.0, TAU]
        } else {
            vec![0.0]
        };
        let mut out = Vec::new();
        for turn in turns {
            let [lo, hi] = [range[0] - turn, range[1] - turn];
            for (cell, &[a, b]) in spans.iter().enumerate() {
                if hi < a || b < lo {
                    continue;
                }
                let local = |x: f64| ((x - a) / (b - a)).clamp(0.0, 1.0);
                out.push((cell, [local(lo), local(hi)], turn != 0.0));
            }
        }
        out
    }

    /// Candidates for the `u` of a tube circle of the torus lying on the
    /// other surface, ascending in `[0, 2π)`: such a circle crosses every
    /// line of constant `v`, where `f` along the line vanishes and
    /// changes sign, or touches zero and its derivative does. The
    /// columns' edges are candidates as they are, since a change of sign
    /// exactly at the end of a polynomial's `[0, 1]` is no variation of
    /// its coefficients. A candidate is a place to look and no more.
    ///
    /// On a NURBS surface the same for an isoline `u = u₀` on the other
    /// surface (ADR-0061 §3): the knot lines of `u`, the domain's edges
    /// among them, and the sign changes along a line of constant `v`
    /// through every patch, ascending in the walk's `[0, 2π]`.
    pub(crate) fn tube_circle_candidates(&self) -> Vec<f64> {
        if self.chart != Chart::Quarters {
            let mut found = self.knot_lines();
            for patch in &self.patches {
                let line = patch.f.along(CIRCLE_ROW);
                let m = line.len().saturating_sub(1) as f64;
                let slope = bernstein::derivative(&line);
                let roots = sign_change_candidates(&line, patch.floor)
                    .into_iter()
                    .chain(sign_change_candidates(&slope, 2.0 * m * patch.floor));
                found.extend(roots.map(|s| self.param(patch, 0, s)));
            }
            found.sort_by(f64::total_cmp);
            found.dedup();
            return found;
        }
        let mut found: Vec<f64> = (0..4).map(|q| q as f64 * FRAC_PI_2).collect();
        for patch in self.patches.iter().filter(|p| p.cell[1] % 2 == 0) {
            let line = patch.f.along(CIRCLE_ROW);
            let m = line.len().saturating_sub(1) as f64;
            let slope = bernstein::derivative(&line);
            let roots = sign_change_candidates(&line, patch.floor)
                .into_iter()
                .chain(sign_change_candidates(&slope, 2.0 * m * patch.floor));
            found.extend(roots.map(|s| quarter_angle(patch.cell[0], s).rem_euclid(TAU)));
        }
        found.sort_by(f64::total_cmp);
        found
    }

    /// The rounding of a coefficient of the polynomial, and of the other
    /// surface's polynomial at a point of the torus.
    pub(crate) fn floor(&self) -> f64 {
        self.floor
    }

    /// No zero of the polynomial anywhere on the torus: every patch's
    /// coefficients keep one sign beyond their rounding.
    pub(crate) fn is_empty(&self) -> bool {
        (self.patches.iter()).all(|patch| patch.f.sign(patch.floor).is_some())
    }

    /// The tube circle at `u0`, which lies on the other surface, divided
    /// out of the polynomial: `order` is one where the surfaces cross
    /// along it and two where they are tangent along it. What is left is
    /// `F̃(P(u, v)) / sinᵐ((u − u₀) / 2)` times a positive function, its
    /// sign that of the quotient with `u − u₀` taken in `(−2π, 2π)` — so
    /// that for an odd order it changes sign with a whole turn of `u`,
    /// which [`Self::sign_over`] knows.
    ///
    /// With `corrected`, `F̃` is `F` less what keeps the division from
    /// being exact when the circle is on the other surface only within
    /// the tolerance: `F(P(u₀, v))`, and for a tangency `sin(u − u₀)·
    /// ∂F/∂u(u₀, v)` as well, both polynomials on the patches. It is the
    /// function `crate::torus_walk` evaluates in closed form, so the two
    /// have the same zeros to rounding. That needs the patches as
    /// [`Self::new`] made them: a second circle is divided out as it is,
    /// and what the division is off by is dropped.
    ///
    /// A column is divided by its own linear factor `s − s₀` — the chart
    /// of a quarter turn makes `sin((u − u₀) / 2)` that, over a positive
    /// function — when the circle lies in it or within
    /// [`DIVIDED_WITHIN`] of it; the others keep their polynomial, which
    /// the factor does not make small.
    ///
    /// On a NURBS surface `u0` is an isoline's, divided out as
    /// [`Self::deflate_isoline`] says, and `corrected` is not looked at.
    pub(crate) fn deflate(&mut self, u0: f64, order: usize, corrected: bool) {
        if self.chart != Chart::Quarters {
            self.deflate_isoline(u0, order);
            return;
        }
        let d = self.implicit.degree();
        let binomials = Binomials::new(4 * d + 2);
        let home = ((u0 / FRAC_PI_2).floor().max(0.0) as usize).min(3);
        let s_home = quarter_param(u0 - home as f64 * FRAC_PI_2);
        let w_home = quarter_weight(s_home);
        let weight = vec![1.0, FRAC_1_SQRT_2, 1.0];
        let power =
            |n: usize| (0..n).fold(vec![1.0], |p, _| bernstein::mul(&p, &weight, &binomials));
        // The half angle from a column's middle to the circle, and the
        // circle's parameter in the column's chart.
        let chart = |qu: usize| {
            let middle = qu as f64 * FRAC_PI_2 + FRAC_PI_4;
            let alpha = 0.5 * ((u0 - middle + TAU / 2.0).rem_euclid(TAU) - TAU / 2.0);
            let root = if qu == home {
                s_home
            } else {
                0.5 * (1.0 + alpha.tan() / QUARTER_TAN)
            };
            (middle, alpha, root)
        };
        let mut rounding = self.floor;
        if corrected {
            for step in 0..order {
                for qv in 0..4 {
                    let Some(at_home) = self.patches.get(4 * home + qv) else {
                        continue;
                    };
                    let (left, scale) = if step == 0 {
                        (at_home.f.across(s_home), 1.0 / w_home.powi(d as i32))
                    } else {
                        let scale = w_home / (2.0 * QUARTER_TAN * w_home.powi(d as i32));
                        (at_home.f.du().across(s_home), scale)
                    };
                    for qu in 0..4 {
                        let (_, alpha, _) = chart(qu);
                        let (sin, cos) = alpha.sin_cos();
                        let t = QUARTER_TAN;
                        let along = if step == 0 {
                            power(d)
                        } else {
                            // `w·sin(u − u₀)` over `2 cos² π/8`.
                            let first = [-t * cos - sin, t * cos - sin];
                            let second = [cos - t * sin, cos + t * sin];
                            let turn = bernstein::mul(&first, &second, &binomials);
                            bernstein::mul(&power(d - 1), &turn, &binomials)
                        };
                        if let Some(patch) = self.patches.get_mut(4 * qu + qv) {
                            let less = Poly2::outer(&along, &left);
                            patch.f = Poly2::combine(&[(1.0, &patch.f), (-scale, &less)]);
                        }
                    }
                }
                rounding += self.floor;
            }
        }
        let mut worst = rounding;
        for patch in &mut self.patches {
            let (middle, _, root) = chart(patch.cell[0]);
            let odd = order % 2 == 1;
            let divided = (-DIVIDED_WITHIN..=1.0 + DIVIDED_WITHIN).contains(&root);
            let mut floor = rounding;
            if divided {
                for _ in 0..order {
                    let (quotient, carried) = patch.f.over_linear(root, floor, &binomials);
                    patch.f = quotient;
                    floor = carried;
                }
            }
            // `s − s₀` has the sign of `u − u₀` taken in `(−π, π)`, and
            // the sign wanted is that of `u − u₀` as it is.
            let flipped = if divided {
                (middle - u0).abs() > TAU / 2.0
            } else {
                middle < u0
            };
            if odd && flipped {
                patch.f = Poly2::combine(&[(-1.0, &patch.f)]);
            }
            patch.floor = floor;
            worst = worst.max(floor);
        }
        self.floor = worst;
        self.near = worst + self.steep;
        self.odd_turn ^= order % 2 == 1;
    }

    /// The isoline `u = u₀` of a NURBS surface, which lies on the other
    /// surface, divided out of the polynomial (ADR-0061 §3): `order` is
    /// one where the surfaces cross along it and two where they are
    /// tangent along it. What is left is `F / gᵐ` times a positive
    /// function, `g` being `(u − u₀) / 2π` where `u` does not close and
    /// `sin((u − u₀) / 2)` where it does — so that for an odd order it
    /// changes sign with a whole turn, which [`Self::sign_over`] knows —
    /// and never more than one in size, so the quotient over `|∇F|` is
    /// never less than the distance ([`Quotient`]).
    ///
    /// A patch whose span of `u` holds the isoline — both patches, where
    /// it is on a knot, and where `u` closes the patch that holds it a
    /// closure on — is divided by its own linear factor `s − s₀`, the
    /// chart being linear. The others keep their polynomial, which is a
    /// patch's own piece of the surface and does not vanish on the line:
    /// they are only given the sign the quotient has there. No correction
    /// is made for an isoline only within the tolerance of the other
    /// surface: what the division is off by is dropped, and it is within
    /// what `tol.linear` makes of `f`, or the isoline is not accepted.
    fn deflate_isoline(&mut self, u0: f64, order: usize) {
        let Chart::Knots { closed, .. } = self.chart else {
            return;
        };
        let binomials = Binomials::new(self.binomial_degree);
        let odd = order % 2 == 1;
        let homes: &[f64] = if closed[0] { &[u0, u0 + TAU] } else { &[u0] };
        let mut worst = self.floor;
        for patch in &mut self.patches {
            let [lo, hi] = patch.span[0];
            let width = hi - lo;
            let slack = PATCH_SLACK * width;
            let home = (homes.iter().copied()).find(|&r| r >= lo - slack && r <= hi + slack);
            let mut floor = patch.floor;
            if let Some(r) = home {
                let root = ((r - lo) / width).clamp(0.0, 1.0);
                for _ in 0..order {
                    let (quotient, carried) = patch.f.over_linear(root, floor, &binomials);
                    patch.f = quotient;
                    floor = carried;
                }
                patch.divided = Some(r);
            }
            // `s − s₀` has the sign of `u` less the isoline's place it
            // was divided at, and the sign wanted is that of `g`.
            let flipped = match home {
                Some(r) => r != u0,
                None => 0.5 * (lo + hi) < u0,
            };
            if odd && flipped {
                patch.f = Poly2::combine(&[(-1.0, &patch.f)]);
            }
            patch.floor = floor;
            worst = worst.max(floor);
        }
        self.floor = worst;
        self.near = worst + self.steep;
        self.odd_turn ^= odd;
    }

    /// The patches' polynomials as they are now, with the isoline `u =
    /// u₀` of `order` divided out of them, owned: what the walk evaluates
    /// the section on beside the isoline. `None` on a torus.
    pub(crate) fn quotient(&self, u0: f64, order: usize) -> Option<Quotient> {
        let Chart::Knots {
            columns, closed, ..
        } = self.chart
        else {
            return None;
        };
        Some(Quotient {
            u0,
            order,
            closed,
            rows: (self.patches.iter().step_by(columns.max(1)))
                .map(|p| p.span[0])
                .collect(),
            columns: self
                .patches
                .iter()
                .take(columns)
                .map(|p| p.span[1])
                .collect(),
            degree: self.implicit.degree() as i32,
            patches: (self.patches.iter())
                .map(|p| (p.f.clone(), p.weight.clone(), p.divided))
                .collect(),
        })
    }

    /// The knot lines of `u` in the walk's parameter, ascending, the
    /// domain's edges among them; none on a torus.
    pub(crate) fn knot_lines(&self) -> Vec<f64> {
        if self.chart == Chart::Quarters {
            return Vec::new();
        }
        let mut lines: Vec<f64> = (self.patches.iter()).flat_map(|p| p.span[0]).collect();
        lines.sort_by(f64::total_cmp);
        lines.dedup();
        lines
    }

    /// The box of the walk's parameters each patch covers.
    pub(crate) fn spans(&self) -> Vec<[[f64; 2]; 2]> {
        self.patches.iter().map(|p| p.span).collect()
    }

    /// The turning points of the section in `u`. A certified zero is a
    /// simple one: the section turns there and is smooth. An uncertified
    /// box holds a singular point of the section, a turning point that
    /// is an inflection as well, or turning points `f64` does not tell
    /// apart. [`Continuum`] is a whole arc of them: a tube circle of the
    /// torus on the other surface, or the two surfaces the same.
    pub(crate) fn turning_points(&self, meter: &mut Meter) -> Result<Census, Halt<Continuum>> {
        self.census(meter, |patch, meter| {
            let [_, n] = patch.f.degree();
            let f_t = patch.f.dv();
            // A derivative's coefficient is the degree times a difference
            // of two of `f`'s.
            let floors = [patch.floor, 2.0 * n as f64 * patch.floor];
            common_zeros([&patch.f, &f_t], floors, &[], meter)
        })
    }

    /// The critical points of `F(P(u, v))` where the torus is within
    /// `tol.linear` of the other surface, and possibly others near it:
    /// the singular points of the section are among them, and
    /// [`Implicit::distance`] at each says which. A certified zero is a
    /// critical point with a regular Hessian — a crossing of two
    /// branches or an isolated point, at the tolerance.
    pub(crate) fn critical_points(&self, meter: &mut Meter) -> Result<Census, Halt<Continuum>> {
        self.census(meter, |patch, meter| {
            self.critical_isolation(&patch.f, &patch.weight, patch.floor, meter)
        })
    }

    /// The common zeros of the gradient of `F = f / wᵈ` over the unit
    /// square of the polynomial `f` and the weight `w`, where `|f|` is
    /// within `near`; `floor` is the rounding of a coefficient of `f`.
    fn critical_isolation(
        &self,
        f: &Poly2,
        w: &Poly2,
        floor: f64,
        meter: &mut Meter,
    ) -> Result<Isolation, Halt<Continuum>> {
        let degree = self.implicit.degree();
        let binomials = Binomials::new(self.binomial_degree);
        let d = degree as f64;
        let [m, n] = f.degree();
        let g_s = Poly2::combine(&[
            (1.0, &w.mul(&f.du(), &binomials)),
            (-d, &w.du().mul(f, &binomials)),
        ]);
        let g_t = Poly2::combine(&[
            (1.0, &w.mul(&f.dv(), &binomials)),
            (-d, &w.dv().mul(f, &binomials)),
        ]);
        // `w ≤ 1` times a derivative's rounding, and `|w_s| < 1`
        // times `d` of `f`'s own.
        let floors = [(2.0 * m as f64 + d) * floor, (2.0 * n as f64 + d) * floor];
        let gate = Gate {
            poly: f,
            floor: self.near,
            one_sided: false,
        };
        common_zeros([&g_s, &g_t], floors, &[gate], meter)
    }

    /// The critical points of a NURBS surface's section in the slab past
    /// the edge of its domain where direction `k`'s parameter is `edge`,
    /// [`EDGE_SLAB`] of each patch's span along the edge deep, in the
    /// walk's parameters where they are, past the edge (ADR-0061 §3,
    /// amended for a walked blend's junction). The patches' polynomials go
    /// on past the domain, carried on by de Casteljau, and the walk decides
    /// which zeros are close enough to the edge to be put on it. None on a
    /// torus.
    pub(crate) fn critical_points_past(
        &self,
        k: usize,
        edge: f64,
        meter: &mut Meter,
    ) -> Result<Vec<Zero2>, Halt<Continuum>> {
        if self.chart == Chart::Quarters {
            return Ok(Vec::new());
        }
        let d = EDGE_SLAB;
        let mut zeros = Vec::new();
        for patch in &self.patches {
            let [lo, hi] = patch.span[k];
            // The slab on its own `[0, 1]` in direction `k`, and the
            // patch's own parameter at `x` of it: `−d·x` past its start,
            // `1 + d·x` past its end.
            let past_start = if lo == edge {
                true
            } else if hi == edge {
                false
            } else {
                continue;
            };
            let cut = |p: &Poly2| {
                if past_start {
                    p.split_u(-d).0
                } else {
                    p.split_u(1.0 + d).0.split_u(1.0 / (1.0 + d)).1
                }
            };
            let slab = |p: &Poly2| {
                if k == 0 {
                    cut(p)
                } else {
                    cut(&p.transposed()).transposed()
                }
            };
            let own = |x: f64| if past_start { -d * x } else { 1.0 + d * x };
            let found =
                self.critical_isolation(&slab(&patch.f), &slab(&patch.weight), patch.floor, meter)?;
            let angles = |p: [f64; 2]| {
                [0, 1].map(|j| {
                    let s = if j == k { own(p[j]) } else { p[j] };
                    self.param(patch, j, s)
                })
            };
            for z in found.zeros {
                let (a, b) = (angles(z.lo), angles(z.hi));
                zeros.push(Zero2 {
                    at: angles(z.at),
                    lo: [0, 1].map(|j| a[j].min(b[j])),
                    hi: [0, 1].map(|j| a[j].max(b[j])),
                    certified: z.certified,
                });
            }
        }
        Ok(zeros)
    }

    /// The sign `probe` keeps over the box `u × v` of angles — unwrapped,
    /// either side up to a whole turn — beyond its rounding floor:
    /// `Some(true)` for positive, `None` where the coefficients do not
    /// say. The box is cut at the patches' edges and every part has to
    /// agree. `f` is continuous across those edges, so parts on which
    /// `f_s` (or `f_t`) keeps one sign make `f` monotone in `u` (in `v`)
    /// across the whole box; [`Probe::Fvv`] is free of the patch's
    /// weight altogether.
    pub(crate) fn sign_over(&self, probe: Probe, u: [f64; 2], v: [f64; 2]) -> Option<bool> {
        let d = self.implicit.degree() as f64;
        let mut sign = None;
        for (cu, s, turned) in self.cells(0, u) {
            for (cv, t, _) in self.cells(1, v) {
                let patch = self.patch_at([cu, cv])?;
                let [m, n] = patch.f.degree();
                let (m, n) = (m as f64, n as f64);
                let part = match probe {
                    Probe::F => patch.f.restricted(s, t).sign(patch.floor),
                    Probe::Fs => patch.f.du().restricted(s, t).sign(2.0 * m * patch.floor),
                    Probe::Ft => patch.f.dv().restricted(s, t).sign(2.0 * n * patch.floor),
                    Probe::Fvv => {
                        let binomials = Binomials::new(self.binomial_degree);
                        let (f, w) = (&patch.f, &patch.weight);
                        // `w·X_t − d·w_t·X`, twice: `w^(d+1)·F_t` and then
                        // a positive multiple of `∂²F/∂v²`, since a
                        // quarter's `dv/dt` is a constant over its weight.
                        let weighed = |x: &Poly2| {
                            Poly2::combine(&[
                                (1.0, &w.mul(&x.dv(), &binomials)),
                                (-d, &w.dv().mul(x, &binomials)),
                            ])
                        };
                        let g = weighed(f);
                        let floor_g = (2.0 * n + d) * patch.floor;
                        let floor_h = (2.0 * (n + 1.0) + d) * floor_g;
                        weighed(&g).restricted(s, t).sign(floor_h)
                    }
                }?;
                let part = part != (turned && self.odd_turn);
                if sign.is_some_and(|s| s != part) {
                    return None;
                }
                sign = Some(part);
            }
        }
        sign
    }

    /// Candidates for the roots of `f` along the first row of `u` — `u =
    /// 0` on a torus and on a NURBS surface — as values of `v` ascending:
    /// every sign change of the section's polynomial there has one within
    /// rounding of it, and a candidate need be no root
    /// (`crate::bernstein::sign_change_candidates`).
    pub(crate) fn seam_candidates(&self) -> Vec<f64> {
        self.edge_candidates(0, false)
    }

    /// The walked surface's own parameters at `at`, the walk's: a torus's
    /// angles as they are, a NURBS surface's through its [`Rescale`].
    #[cfg(test)]
    pub(crate) fn surface_uv(&self, at: [f64; 2]) -> [f64; 2] {
        match self.chart {
            Chart::Quarters => at,
            Chart::Knots { rescale, .. } => [0, 1].map(|k| rescale.to_surface(k, at[k])),
        }
    }

    /// Whether the section turns back in `u` at a kink of a NURBS
    /// surface: at a root of `f` on a knot line of `v` shared by two
    /// patches — the seam too, where `v` closes — `∂f/∂t` beyond its
    /// rounding of one sign on the patch below and of the other above.
    /// Across a knot line where the surface is only continuous the
    /// section's tangent jumps, and it may come back the way it came with
    /// no turning point to find (ADR-0061). Never on a torus, whose
    /// quarter turns join smoothly.
    pub(crate) fn turns_at_a_kink(&self) -> bool {
        let Chart::Knots {
            columns, closed, ..
        } = self.chart
        else {
            return false;
        };
        for below in &self.patches {
            let [cu, cv] = below.cell;
            let next = if cv + 1 < columns {
                cv + 1
            } else if closed[1] {
                0
            } else {
                continue;
            };
            let Some(above) = self.patch_at([cu, next]) else {
                continue;
            };
            let [_, n] = below.f.degree();
            let [_, n_above] = above.f.degree();
            let (slope_below, slope_above) = (below.f.dv(), above.f.dv());
            let line = below.f.along(1.0);
            for s in sign_change_candidates(&line, below.floor) {
                let lo = slope_below.eval(s, 1.0);
                let hi = slope_above.eval(s, 0.0);
                let clear = lo.abs() > 2.0 * n as f64 * below.floor
                    && hi.abs() > 2.0 * n_above as f64 * above.floor;
                if clear && (lo > 0.0) != (hi > 0.0) {
                    return true;
                }
            }
        }
        false
    }

    /// Whether each direction closes: both on a torus.
    pub(crate) fn closed(&self) -> [bool; 2] {
        match self.chart {
            Chart::Quarters => [true; 2],
            Chart::Knots { closed, .. } => closed,
        }
    }

    /// Candidates for the roots of `f` along the line where direction
    /// `k`'s parameter is at the start of its domain, `0`, or with `high`
    /// at its end, `2π`: values of the other direction's parameter,
    /// ascending, with the same guarantee as [`Self::seam_candidates`].
    /// On a NURBS surface each direction that does not close is bounded
    /// by two such lines, the edges of its domain (ADR-0061).
    pub(crate) fn edge_candidates(&self, k: usize, high: bool) -> Vec<f64> {
        let last = (self.patches.iter()).map(|p| p.cell[k]).max().unwrap_or(0);
        let edge = if high { last } else { 0 };
        let mut found: Vec<f64> = Vec::new();
        for patch in self.patches.iter().filter(|p| p.cell[k] == edge) {
            let [m, n] = patch.f.degree();
            let stride = n + 1;
            let all = patch.f.coefficients();
            let line: Vec<f64> = match (k, high) {
                (0, false) => all[..stride].to_vec(),
                (0, true) => all[m * stride..].to_vec(),
                (_, false) => (0..=m).map(|i| all[i * stride]).collect(),
                (_, true) => (0..=m).map(|i| all[i * stride + n]).collect(),
            };
            found.extend(
                sign_change_candidates(&line, patch.floor)
                    .into_iter()
                    .map(|t| self.param(patch, 1 - k, t)),
            );
        }
        found.sort_by(f64::total_cmp);
        found
    }

    /// A zero a patch's isolation found, in the patch's `[0, 1]²`, if it
    /// is the walked surface's. A NURBS patch's polynomial goes on past
    /// the patch's edges, where the surface is the next patch's or none,
    /// and its isolation certifies zeros there with boxes that reach
    /// across: a certified zero counts within [`PATCH_SLACK`] of the
    /// patch, put on it, and one that is not where its box reaches in,
    /// each box cut to the patch and that slack; the next patch finds its
    /// own. A
    /// torus's quarter turns are one surface, and every zero counts.
    fn own(&self, z: &Zero2) -> Option<Zero2> {
        if self.chart == Chart::Quarters {
            return Some(*z);
        }
        let inside = |x: f64| (-PATCH_SLACK..=1.0 + PATCH_SLACK).contains(&x);
        let kept = if z.certified {
            z.at.iter().all(|&x| inside(x))
        } else {
            (0..2).all(|k| z.lo[k] <= 1.0 && z.hi[k] >= 0.0)
        };
        // The box no farther past the patch than the slack: a zero on a
        // knot line found from either side is one, its two boxes each
        // holding the other's point, and a box a span past the patch
        // would hold the next patch's own zeros.
        kept.then(|| Zero2 {
            at: z.at.map(|x| x.clamp(0.0, 1.0)),
            lo: z.lo.map(|x| x.max(-PATCH_SLACK)),
            hi: z.hi.map(|x| x.min(1.0 + PATCH_SLACK)),
            certified: z.certified,
        })
    }

    /// One isolation per patch, its zeros and boxes taken to the walked
    /// surface's parameters and those found from two patches made one.
    fn census(
        &self,
        meter: &mut Meter,
        isolate: impl Fn(&Patch, &mut Meter) -> Result<Isolation, Halt<Continuum>>,
    ) -> Result<Census, Halt<Continuum>> {
        let mut zeros = Vec::new();
        let (mut depth, mut boxes) = (0, 0);
        for patch in &self.patches {
            let found = isolate(patch, meter)?;
            depth = depth.max(found.depth);
            boxes += found.boxes;
            let angles = |p: [f64; 2]| [0, 1].map(|k| self.param(patch, k, p[k]));
            zeros.extend(
                (found.zeros.iter())
                    .filter_map(|z| self.own(z))
                    .map(|z| Zero2 {
                        at: angles(z.at),
                        lo: angles(z.lo),
                        hi: angles(z.hi),
                        certified: z.certified,
                    }),
            );
        }
        // A shared edge is one knot through one map on either side, the
        // same to the bit; the rest is the isolation's own rounding.
        let wrap = Wrap {
            period: self.closed().map(|c| c.then_some(TAU)),
            origin: [0.0; 2],
            touch: ANGLE_ROUNDING,
        };
        Ok(Census {
            zeros: merge_wrapped(zeros, &wrap),
            depth,
            boxes,
        })
    }
}

#[cfg(test)]
mod tests {
    use arris_math::nalgebra::UnitQuaternion;
    use arris_math::{Frame, Interval, Isometry, Point3, Precision, Vec3};

    use core::f64::consts::PI;

    use super::*;

    const TOL: Tolerance = Precision::DEFAULT.tolerance();

    fn frame(origin: Point3, z: Vec3) -> Frame {
        Frame::from_z(origin, z).unwrap()
    }

    fn torus(major: f64, minor: f64) -> Surface {
        Surface::Torus {
            frame: Frame::world(),
            major_radius: major,
            minor_radius: minor,
        }
    }

    /// The six partners of a torus at rest at the origin, each cutting
    /// through its tube near `u = 0.7` at an oblique angle, and each
    /// again at the torus's own size, through the hole and both sides of
    /// the ring.
    fn partners(major: f64, minor: f64) -> Vec<(&'static str, Surface)> {
        let (r, big) = (minor, major);
        let on_ring = Point3::new(major * 0.7f64.cos(), major * 0.7f64.sin(), 0.0);
        let oblique = Vec3::new(0.4, -0.3, 0.85);
        let steep = Vec3::new(0.2, 0.1, 0.97);
        let centre = Point3::new(0.13 * r, -0.07 * r, 0.21 * r);
        let ring_of = |major: f64, minor: f64| (major > minor).then_some((major, minor));
        let mut all = vec![
            (
                "plane through the tube",
                Surface::Plane {
                    frame: frame(on_ring + 0.3 * r * Vec3::x(), oblique),
                },
            ),
            (
                "plane through the ring",
                Surface::Plane {
                    frame: frame(centre, steep),
                },
            ),
            (
                "cylinder through the tube",
                Surface::Cylinder {
                    frame: frame(on_ring + 0.5 * r * Vec3::z(), oblique),
                    radius: 0.6 * r,
                },
            ),
            (
                "cylinder through the ring",
                Surface::Cylinder {
                    frame: frame(centre, Vec3::new(0.9, 0.3, 0.2)),
                    radius: 0.7 * big,
                },
            ),
            (
                "elliptic cylinder through the tube",
                Surface::EllipticCylinder {
                    frame: frame(on_ring - 0.2 * r * Vec3::y(), oblique),
                    major_radius: 0.9 * r,
                    minor_radius: 0.5 * r,
                },
            ),
            (
                "elliptic cylinder through the ring",
                Surface::EllipticCylinder {
                    frame: frame(centre, Vec3::new(0.3, 0.9, -0.2)),
                    major_radius: 0.8 * big,
                    minor_radius: 0.3 * big,
                },
            ),
            (
                "cone through the tube",
                Surface::Cone {
                    frame: frame(on_ring + 0.2 * r * Vec3::x(), oblique),
                    radius: 0.5 * r,
                    half_angle: 0.4,
                },
            ),
            (
                "cone through the ring",
                Surface::Cone {
                    frame: frame(centre, steep),
                    radius: 0.4 * big,
                    half_angle: 0.6,
                },
            ),
            (
                "sphere in the tube",
                Surface::Sphere {
                    frame: frame(on_ring + 0.7 * r * oblique.normalize(), Vec3::z()),
                    radius: 0.8 * r,
                },
            ),
            (
                "sphere through the ring",
                Surface::Sphere {
                    frame: frame(
                        Point3::new(0.6 * big, 0.2 * big, 0.1 * r),
                        Vec3::new(0.1, 0.2, 0.9),
                    ),
                    radius: big,
                },
            ),
        ];
        if let Some((major, minor)) = ring_of(0.9 * big, 0.8 * r) {
            all.push((
                "torus through the tube",
                Surface::Torus {
                    frame: frame(on_ring + major * Vec3::x(), Vec3::new(0.1, 0.9, 0.3)),
                    major_radius: major,
                    minor_radius: minor,
                },
            ));
        }
        all.push((
            "torus, a chain link",
            Surface::Torus {
                frame: frame(
                    Point3::new(big, 0.1 * r, 0.05 * r),
                    Vec3::new(0.05, 1.0, 0.1),
                ),
                major_radius: big,
                minor_radius: 0.9 * r,
            },
        ));
        all
    }

    /// What the measured cost of the method is held to, with room to
    /// spare. The depth, in halvings, of an isolation whose zeros are all
    /// simple: 25 at the most over the posed pairs, on a torus a hundred
    /// tubes across. And the boxes of any isolation that ends on points:
    /// 740 at the most over the posed pairs, 1508 where a cylinder
    /// touches a saddle of the torus exactly and the turning points' zero
    /// that is not simple is followed down to where `f` is flat.
    const MEASURED_DEPTH: usize = 32;
    const MEASURED_BOXES: usize = 2000;

    /// The torus to walk and the surface to put it into, by the tracer's
    /// rule (`crate::torus_walk`): of two tori the smaller one over all
    /// is walked. A torus's polynomial is a quartic, and at points a
    /// hundred of its own sizes away its terms cancel to `100⁴·ε` of
    /// themselves, so a small torus is never the implicit one for a large
    /// one's points. The flag says the operands were exchanged.
    fn operands<'s>(torus: &'s Surface, other: &'s Surface) -> (&'s Surface, &'s Surface, bool) {
        let size = |s: &Surface| match s {
            Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } => Some(major_radius + minor_radius),
            _ => None,
        };
        match size(torus).zip(size(other)) {
            Some((a, b)) if b < a => (other, torus, true),
            _ => (torus, other, false),
        }
    }

    /// Metre-scale tori: radii from 0.01 to 10, `R/r` from 1.1 to 100.
    const TORI: [(f64, f64); 6] = [
        (0.011, 0.01),
        (1.0, 0.01),
        (10.0, 9.0),
        (10.0, 0.1),
        (0.05, 0.02),
        (2.0, 0.5),
    ];

    /// At rest, turned about the origin, and turned a kilometre away.
    fn poses() -> [Isometry; 3] {
        let turn = UnitQuaternion::from_euler_angles(0.3, -1.1, 2.0);
        [
            Isometry::identity(),
            Isometry::from_rotation(turn),
            Isometry::new(turn, Vec3::new(1e3, -2e3, 5e2)),
        ]
    }

    /// The signed distance of the torus's point to the other surface and
    /// its derivative in `v`.
    fn distance_and_slope(torus: &Surface, implicit: &Implicit<'_>, u: f64, v: f64) -> (f64, f64) {
        let e = torus.eval(u, v);
        let slope = implicit
            .gradient(e.point)
            .dot(&implicit.frame.vec_to_local(e.dv));
        (implicit.distance(e.point), slope)
    }

    /// How many of the scan's finest cells a flagged one may be from a
    /// reported zero: the two zero sets cross at an angle, and cells
    /// along the narrow wedge between them are flagged as well.
    const NEAR_CELLS: f64 = 8.0;

    fn apart(a: [f64; 2], b: [f64; 2]) -> f64 {
        let turn = |x: f64| (x.rem_euclid(TAU)).min(TAU - x.rem_euclid(TAU));
        turn(a[0] - b[0]).max(turn(a[1] - b[1]))
    }

    /// Cells of a grid over `[lo, lo + width]²` where the distance and
    /// its slope in `v` both change sign, looked into on a finer grid
    /// until `depth` runs out: what a turning point cannot hide from.
    fn scan(
        torus: &Surface,
        implicit: &Implicit<'_>,
        lo: [f64; 2],
        width: f64,
        cells: usize,
        depth: usize,
        out: &mut Vec<([f64; 2], f64)>,
    ) {
        let step = width / cells as f64;
        let at = |i: usize, j: usize| [lo[0] + step * i as f64, lo[1] + step * j as f64];
        let grid: Vec<Vec<(f64, f64)>> = (0..=cells)
            .map(|i| {
                (0..=cells)
                    .map(|j| distance_and_slope(torus, implicit, at(i, j)[0], at(i, j)[1]))
                    .collect()
            })
            .collect();
        for i in 0..cells {
            for j in 0..cells {
                let corners = [
                    grid[i][j],
                    grid[i + 1][j],
                    grid[i][j + 1],
                    grid[i + 1][j + 1],
                ];
                let changes = |pick: fn(&(f64, f64)) -> f64| {
                    corners.iter().any(|c| pick(c) <= 0.0) && corners.iter().any(|c| pick(c) >= 0.0)
                };
                if !(changes(|c| c.0) && changes(|c| c.1)) {
                    continue;
                }
                if depth == 0 {
                    out.push((at(i, j), step));
                } else {
                    scan(torus, implicit, at(i, j), step, 8, depth - 1, out);
                }
            }
        }
    }

    #[test]
    fn a_patch_is_the_implicit_polynomial_over_the_torus() {
        let binomials = Binomials::new(0);
        for (major, minor) in TORI {
            for (name, other) in partners(major, minor) {
                let pose = &poses()[1];
                let (torus, other) = (
                    torus(major, minor).transformed(pose),
                    other.transformed(pose),
                );
                let section = PatchedSection::new(&torus, &other, TOL).unwrap();
                for patch in &section.patches {
                    for (s, t) in [(0.0, 0.0), (0.3, 0.8), (1.0, 0.45), (0.5, 0.5), (1.0, 1.0)] {
                        let [u, v] = [0, 1].map(|k| quarter_angle(patch.cell[k], [s, t][k]));
                        let p = section.implicit.frame.to_local(torus.point(u, v));
                        let w = patch.weight.eval(s, t);
                        let coords = [p.x, p.y, p.z, 1.0].map(|c| vec![c]);
                        let [x, y, z, one] = &coords;
                        let direct = section.implicit.along([x, y, z, one], &binomials)[0]
                            * w.powi(section.implicit.degree() as i32);
                        let value = patch.f.eval(s, t);
                        assert!(
                            (value - direct).abs() <= 4.0 * section.floor,
                            "{name} R={major} r={minor} {:?}: {value} vs {direct}, floor {}",
                            patch.cell,
                            section.floor
                        );
                    }
                }
            }
        }
    }

    /// Every pair at every pose: the certified turning points are on the
    /// section and turn; a dense scan finds no turning point the
    /// isolation did not report; the depth and the boxes stay under
    /// their bounds.
    #[test]
    fn no_turning_point_is_missed_on_metre_scale_tori() {
        for (major, minor) in TORI {
            for (name, other) in partners(major, minor) {
                for (k, pose) in poses().iter().enumerate() {
                    let label = format!("{name}, R={major} r={minor}, pose {k}");
                    let torus = torus(major, minor).transformed(pose);
                    let other = other.transformed(pose);
                    let (torus, other, _) = operands(&torus, &other);
                    let section = PatchedSection::new(torus, other, TOL).unwrap();
                    let found = section
                        .turning_points(&mut Meter::default())
                        .unwrap_or_else(|e| panic!("{label}: {e:?}"));
                    let certified = found.zeros.iter().filter(|z| z.certified).count();
                    assert_eq!(certified, found.zeros.len(), "{label}: {:?}", found.zeros);
                    assert_eq!(certified % 2, 0, "{label}: {:?}", found.zeros);
                    for z in &found.zeros {
                        let (d, slope) =
                            distance_and_slope(torus, &section.implicit, z.at[0], z.at[1]);
                        // Rounding of a point a kilometre out, and of a
                        // slope of a length per radian.
                        let scale = major + minor + pose.translation().norm();
                        assert!(d.abs() <= 1e-12 * scale, "{label}: {d} at {:?}", z.at);
                        assert!(
                            slope.abs() <= 1e-7 * minor,
                            "{label}: {slope} at {:?}",
                            z.at
                        );
                    }
                    let mut flagged = Vec::new();
                    scan(
                        torus,
                        &section.implicit,
                        [0.0, 0.0],
                        TAU,
                        128,
                        2,
                        &mut flagged,
                    );
                    for (cell, step) in flagged {
                        let mid = [cell[0] + 0.5 * step, cell[1] + 0.5 * step];
                        assert!(
                            found
                                .zeros
                                .iter()
                                .any(|z| apart(z.at, mid) <= NEAR_CELLS * step),
                            "{label}: a turning point near {mid:?} not among {:?}",
                            found.zeros
                        );
                    }
                    assert!(
                        found.depth <= MEASURED_DEPTH,
                        "{label}: depth {}",
                        found.depth
                    );
                    assert!(
                        found.boxes <= MEASURED_BOXES,
                        "{label}: {} boxes",
                        found.boxes
                    );
                }
            }
        }
    }

    /// `other` moved so that its point at `(uc, vc)` touches the torus at
    /// rest at `(u, v)` from outside — from inside the tube for `inside`
    /// — with a gap of `gap` along the torus's normal, negative for an
    /// overlap, and turned about that normal by a fixed angle.
    fn touching(
        torus: &Surface,
        [u, v]: [f64; 2],
        other: &Surface,
        [uc, vc]: [f64; 2],
        inside: bool,
        gap: f64,
    ) -> Surface {
        let (p, n) = (torus.point(u, v), torus.normal(u, v).unwrap().into_inner());
        let (q, m) = (
            other.point(uc, vc),
            other.normal(uc, vc).unwrap().into_inner(),
        );
        let facing = if inside { n } else { -n };
        let turn = UnitQuaternion::from_scaled_axis(0.7 * n)
            * UnitQuaternion::rotation_between(&m, &facing).unwrap();
        let target = if inside { p - gap * n } else { p + gap * n };
        let motion = Isometry::new(turn, target.coords - turn * q.coords);
        other.transformed(&motion)
    }

    /// The partners of [`touching`], a fraction of the tube across, with
    /// the parameters of the point each touches at and whether it is
    /// inside the tube.
    fn touchers(minor: f64) -> Vec<(&'static str, Surface, [f64; 2], bool)> {
        let r = minor;
        let world = Frame::world();
        vec![
            ("plane", Surface::Plane { frame: world }, [0.0, 0.0], false),
            (
                "cylinder",
                Surface::Cylinder {
                    frame: world,
                    radius: 0.4 * r,
                },
                [0.9, 0.3 * r],
                false,
            ),
            (
                "elliptic cylinder",
                Surface::EllipticCylinder {
                    frame: world,
                    major_radius: 0.6 * r,
                    minor_radius: 0.3 * r,
                },
                [0.9, 0.3 * r],
                false,
            ),
            (
                "cone",
                Surface::Cone {
                    frame: world,
                    radius: 0.4 * r,
                    half_angle: 0.5,
                },
                [2.1, 0.2 * r],
                false,
            ),
            (
                "sphere",
                Surface::Sphere {
                    frame: world,
                    radius: 0.4 * r,
                },
                [0.4, 0.5],
                false,
            ),
            (
                "sphere in the tube",
                Surface::Sphere {
                    frame: world,
                    radius: 0.5 * r,
                },
                [0.4, 0.5],
                true,
            ),
            (
                "torus",
                Surface::Torus {
                    frame: world,
                    major_radius: 1.5 * r,
                    minor_radius: 0.4 * r,
                },
                [1.3, 0.4],
                false,
            ),
        ]
    }

    /// On the outside of the ring, where the torus is convex, and on the
    /// inside, where it is a saddle.
    const TOUCHED_AT: [[f64; 2]; 2] = [[0.9, 0.5], [4.0, 2.6]];

    fn within_tolerance<'z>(
        torus: &Surface,
        section: &PatchedSection<'_>,
        critical: &'z Census,
    ) -> Vec<&'z Zero2> {
        critical
            .zeros
            .iter()
            .filter(|z| {
                let p = torus.point(z.at[0], z.at[1]);
                z.certified && section.implicit.distance(p).abs() <= TOL.linear
            })
            .collect()
    }

    /// The gate itself: surfaces tangent at a point, a near miss and a
    /// small overlap within the tolerance all show as a certified
    /// critical point within the tolerance of the other surface; a
    /// hundred tolerances apart or into each other nothing within the
    /// tolerance is left, and every turning point is certified — the two
    /// either side of a saddle included, √(tol·r) apart.
    #[test]
    fn a_tangency_is_a_critical_point_within_the_tolerance() {
        for (major, minor) in TORI {
            let torus = torus(major, minor);
            for at in TOUCHED_AT {
                for (name, other, on_other, inside) in touchers(minor) {
                    for gap in [0.0, 0.5, -0.5, 100.0, -100.0] {
                        let label = format!("{name} at {at:?}, gap {gap} tol, R={major} r={minor}");
                        let other =
                            touching(&torus, at, &other, on_other, inside, gap * TOL.linear);
                        let (walked, other, exchanged) = operands(&torus, &other);
                        let at = if exchanged { on_other } else { at };
                        let section = PatchedSection::new(walked, other, TOL).unwrap();
                        let critical = section
                            .critical_points(&mut Meter::default())
                            .unwrap_or_else(|e| panic!("{label}: {e:?}"));
                        let within = within_tolerance(walked, &section, &critical);
                        let turning = section
                            .turning_points(&mut Meter::default())
                            .unwrap_or_else(|e| panic!("{label}: {e:?}"));
                        if gap.abs() < 1.0 {
                            assert_eq!(within.len(), 1, "{label}: {:?}", critical.zeros);
                            // The point of tangency, as near as a gap of
                            // half a tolerance leaves it defined.
                            let reach = (TOL.linear / minor).sqrt();
                            assert!(apart(within[0].at, at) <= reach, "{label}: {within:?}");
                        } else {
                            assert!(within.is_empty(), "{label}: {within:?}");
                            let open = turning.zeros.iter().filter(|z| !z.certified).count();
                            assert_eq!(open, 0, "{label}: {:?}", turning.zeros);
                            assert_eq!(turning.zeros.len() % 2, 0, "{label}");
                        }
                        for census in [&critical, &turning] {
                            // No bound on the depth: a zero that is
                            // not simple is halved until the polynomial
                            // is flat, as deep as that is.
                            assert!(
                                census.boxes <= MEASURED_BOXES,
                                "{label}: {} boxes",
                                census.boxes
                            );
                        }
                    }
                }
            }
        }
    }

    /// A tube circle of the torus on the other surface is a whole line
    /// `u = u₀` of turning points, which nothing isolates: it comes back
    /// as one uncertified box, a sliver round `u₀` and a whole turn in
    /// `v` — or as a [`Continuum`] — and whatever else the pair meets in
    /// keeps its certified turning points. The pipe elbow's cylinder is
    /// tangent along the circle, a larger sphere centred on the tangent
    /// to the centre circle crosses the torus along it. And the torus
    /// against itself is zero everywhere: one box, the torus.
    #[test]
    fn a_tube_circle_on_the_other_surface_is_a_box_a_turn_long() {
        let (major, minor) = (2.0, 0.5);
        let torus = torus(major, minor);
        let on_ring = Point3::new(major * 0.7f64.cos(), major * 0.7f64.sin(), 0.0);
        let tangent = Vec3::new(-(0.7f64.sin()), 0.7f64.cos(), 0.0);
        let elbow = Surface::Cylinder {
            frame: frame(on_ring, tangent),
            radius: minor,
        };
        let ball = Surface::Sphere {
            frame: frame(on_ring + 1.2 * minor * tangent, Vec3::z()),
            radius: minor * 1.2f64.hypot(1.0),
        };
        // The ball is its own mirror image in the plane through the axis
        // and its centre, and so is what it shares with the torus: the
        // tube circle at 0.7 and the one as far beyond its centre.
        let mirrored = 0.7 + 2.0 * (1.2 * minor / major).atan();
        for (name, other, circles) in [
            ("elbow", elbow, vec![0.7]),
            ("ball", ball, vec![0.7, mirrored]),
        ] {
            let section = PatchedSection::new(&torus, &other, TOL).unwrap();
            let Ok(found) = section.turning_points(&mut Meter::default()) else {
                continue;
            };
            let open: Vec<&Zero2> = found.zeros.iter().filter(|z| !z.certified).collect();
            assert_eq!(open.len(), circles.len(), "{name}: {:?}", found.zeros);
            for (z, u) in open.iter().zip(circles) {
                // A sliver beside a patch, which is a quarter turn wide:
                // how thin is the rounding's to say, not the test's.
                assert!(z.lo[0] <= u && u <= z.hi[0], "{name}: {u} in {z:?}");
                assert!(z.hi[0] - z.lo[0] < 1e-2, "{name}: {z:?}");
                assert!(z.hi[1] - z.lo[1] >= TAU - ANGLE_ROUNDING, "{name}: {z:?}");
            }
            // A line of zeros is followed all its length to where the
            // polynomials are flat: some 500 boxes for the elbow, tangent
            // along its circle, some 7000 for the ball, which crosses.
            assert!(
                found.boxes <= 5 * MEASURED_BOXES,
                "{name}: {} boxes",
                found.boxes
            );
        }
        let itself = PatchedSection::new(&torus, &torus, TOL).unwrap();
        let found = itself.turning_points(&mut Meter::default()).unwrap();
        assert_eq!(found.zeros.len(), 1, "{:?}", found.zeros);
        let all = &found.zeros[0];
        assert!(!all.certified);
        assert!((0..2).all(|k| all.hi[k] - all.lo[k] >= TAU - ANGLE_ROUNDING));
    }

    /// A tube circle divided out of the polynomial leaves the rest of the
    /// section: the quotient times `sinᵐ((u − u₀) / 2)` has the sign of
    /// the polynomial it came from everywhere, its turning points are all
    /// certified — the elbow's loop across the circle, which turns twice
    /// — and its candidates on the circle are where the loop crosses it.
    /// Two circles crossed along, a plane through the axis, leave nothing.
    #[test]
    fn a_tube_circle_divided_out_leaves_the_rest_of_the_section() {
        let (major, minor) = (2.0, 0.5);
        let torus = torus(major, minor);
        let plane = |u0: f64| Surface::Plane {
            frame: frame(Point3::origin(), Vec3::new(-(u0.sin()), u0.cos(), 0.0)),
        };
        for u0 in [0.0, 0.7, FRAC_PI_2, 4.0, TAU - 1e-3] {
            let on_ring = Point3::new(major * u0.cos(), major * u0.sin(), 0.0);
            let tangent = Vec3::new(-(u0.sin()), u0.cos(), 0.0);
            let elbow = Surface::Cylinder {
                frame: frame(on_ring, tangent),
                radius: minor,
            };
            for (other, order) in [(elbow, 2), (plane(u0), 1)] {
                let whole = PatchedSection::new(&torus, &other, TOL).unwrap();
                let mut rest = PatchedSection::new(&torus, &other, TOL).unwrap();
                let candidates = whole.tube_circle_candidates();
                assert!(
                    candidates
                        .iter()
                        .any(|c| apart([*c, 0.0], [u0, 0.0]) < 1e-9),
                    "{u0} not among {candidates:?}"
                );
                rest.deflate(u0, order, true);
                assert_eq!(rest.odd_turn, order == 1);
                for (f, h) in whole.patches.iter().zip(&rest.patches) {
                    for (s, t) in [(0.03, 0.2), (0.41, 0.9), (0.77, 0.5), (0.98, 0.1)] {
                        let u = quarter_angle(f.cell[0], s);
                        let factor = (0.5 * (u - u0)).sin().powi(order as i32);
                        let (was, is) = (f.f.eval(s, t), h.f.eval(s, t) * factor);
                        if was.abs() > 1e3 * whole.floor {
                            assert_eq!(was > 0.0, is > 0.0, "{u0}, order {order}, {:?}", f.cell);
                        }
                    }
                }
                if order == 2 {
                    let found = rest.turning_points(&mut Meter::default()).unwrap();
                    assert_eq!(found.zeros.len(), 2, "{u0}: {:?}", found.zeros);
                    assert!(found.zeros.iter().all(|z| z.certified), "{u0}");
                    assert!(found.boxes <= MEASURED_BOXES, "{u0}: {} boxes", found.boxes);
                    assert!(rest.floor <= 1e3 * whole.floor, "{u0}: {}", rest.floor);
                } else {
                    // The quotient changes sign with a whole turn, and a
                    // box is told so.
                    let here = rest.sign_over(Probe::F, [u0 + 0.1, u0 + 0.2], [1.0, 1.1]);
                    let turned =
                        rest.sign_over(Probe::F, [u0 + 0.1 + TAU, u0 + 0.2 + TAU], [1.0, 1.1]);
                    assert!(here.is_some() && here == turned.map(|s| !s), "{u0}");
                    assert!(!rest.is_empty(), "{u0}");
                    rest.deflate((u0 + TAU / 2.0).rem_euclid(TAU), 1, false);
                    assert!(rest.is_empty() && !rest.odd_turn, "{u0}");
                }
            }
        }
    }

    /// A plane parallel to the axis, from through the hole to clear of
    /// the ring: the spiric sections, whose turning points are known.
    #[test]
    fn the_spiric_sections_turn_where_they_should() {
        let (major, minor) = (2.0, 0.5);
        let torus = torus(major, minor);
        let census = |d: f64| {
            let plane = Surface::Plane {
                frame: frame(Point3::new(d, 0.0, 0.0), Vec3::x()),
            };
            let section = PatchedSection::new(&torus, &plane, TOL).unwrap();
            let critical = section.critical_points(&mut Meter::default()).unwrap();
            let singular: Vec<[f64; 2]> = within_tolerance(&torus, &section, &critical)
                .iter()
                .map(|z| z.at)
                .collect();
            (
                section.turning_points(&mut Meter::default()).unwrap().zeros,
                singular,
            )
        };
        let expect = |found: &[Zero2], expected: &[[f64; 2]]| {
            let certified: Vec<[f64; 2]> =
                found.iter().filter(|z| z.certified).map(|z| z.at).collect();
            assert_eq!(certified.len(), expected.len(), "{found:?}");
            for e in expected {
                assert!(
                    certified.iter().any(|c| apart(*c, *e) < 1e-12),
                    "{e:?} not in {found:?}"
                );
            }
        };
        // Two ovals round the tube: each turns on the outer equator and
        // on the inner one.
        let (turning, singular) = census(1.0);
        let (outer, inner) = ((1.0f64 / 2.5).acos(), (1.0f64 / 1.5).acos());
        expect(
            &turning,
            &[
                [outer, 0.0],
                [TAU - outer, 0.0],
                [inner, PI],
                [TAU - inner, PI],
            ],
        );
        assert!(turning.iter().all(|z| z.certified) && singular.is_empty());
        // One oval: the outer equator only.
        let (turning, singular) = census(2.0);
        let outer = (2.0f64 / 2.5).acos();
        expect(&turning, &[[outer, 0.0], [TAU - outer, 0.0]]);
        assert!(turning.iter().all(|z| z.certified) && singular.is_empty());
        // The figure eight: its crossing is a singular point, and as a
        // turning point a box that holds it.
        let (turning, singular) = census(1.5);
        let outer = (1.5f64 / 2.5).acos();
        expect(&turning, &[[outer, 0.0], [TAU - outer, 0.0]]);
        assert_eq!(singular.len(), 1, "{singular:?}");
        assert!(apart(singular[0], [0.0, PI]) < 1e-12, "{singular:?}");
        let open: Vec<&Zero2> = turning.iter().filter(|z| !z.certified).collect();
        assert_eq!(open.len(), 1, "{turning:?}");
        assert!(apart(open[0].at, [0.0, PI]) < 1e-6, "{turning:?}");
        // A touch on the outer equator, and clear of the ring.
        let (turning, singular) = census(2.5);
        assert_eq!(singular.len(), 1, "{singular:?}");
        assert!(apart(singular[0], [0.0, 0.0]) < 1e-12, "{singular:?}");
        assert!(
            turning
                .iter()
                .all(|z| !z.certified && apart(z.at, [0.0, 0.0]) < 1e-3)
        );
        let (turning, singular) = census(2.5 + 1e-3);
        assert!(turning.is_empty() && singular.is_empty());
    }

    /// The exact NURBS twin of `surface` over `bounds`.
    fn twin(surface: &Surface, bounds: [Interval; 2]) -> NurbsSurface {
        surface.to_nurbs(bounds).unwrap()
    }

    /// The bicubic saddle `z = u² − v²` over `[−1, 1]²` as a B-spline of
    /// one interior knot each way, so that its saddle is the corner four
    /// patches share. A cubic's polar form of `x²` is `(ab + bc + ca)/3`
    /// and of `x` the mean, at each control point's three knots.
    fn saddle() -> NurbsSurface {
        let knots = vec![-1.0, -1.0, -1.0, -1.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let triples = [
            [-1.0, -1.0, -1.0],
            [-1.0, -1.0, 0.0],
            [-1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ];
        let mean = |[a, b, c]: [f64; 3]| (a + b + c) / 3.0;
        let square = |[a, b, c]: [f64; 3]| (a * b + b * c + c * a) / 3.0;
        let mut points = Vec::new();
        for i in triples {
            for j in triples {
                points.push(Point3::new(mean(i), mean(j), square(i) - square(j)));
            }
        }
        NurbsSurface::new([3, 3], [knots.clone(), knots], points, vec![1.0; 25]).unwrap()
    }

    /// The point of `nurbs` at a census zero.
    fn on(nurbs: &NurbsSurface, z: &Zero2) -> Point3 {
        let [u, v] = [0, 1].map(|k| Rescale::of(nurbs).to_surface(k, z.at[k]));
        nurbs.eval(u, v).point
    }

    /// The certified zeros of `census` match `expected` as points, one to
    /// one, within `within`.
    fn matches(nurbs: &NurbsSurface, census: &[&Zero2], expected: &[Point3], within: f64) {
        assert_eq!(census.len(), expected.len(), "{census:?} vs {expected:?}");
        for e in expected {
            assert!(
                census.iter().any(|z| (on(nurbs, z) - e).norm() <= within),
                "{e:?} not among {:?}",
                census.iter().map(|z| on(nurbs, z)).collect::<Vec<_>>()
            );
        }
    }

    fn certified(census: &Census) -> Vec<&Zero2> {
        census.zeros.iter().filter(|z| z.certified).collect()
    }

    /// The critical points within the tolerance of the other surface.
    fn singular<'z>(
        nurbs: &NurbsSurface,
        section: &PatchedSection<'_>,
        critical: &'z Census,
    ) -> Vec<&'z Zero2> {
        (critical.zeros.iter())
            .filter(|z| z.certified && section.implicit.distance(on(nurbs, z)).abs() <= TOL.linear)
            .collect()
    }

    /// Every patch of a NURBS surface's section is the other surface's
    /// polynomial at the surface's point times the patch's weight to the
    /// degree: the twins of a cylinder, a sphere and a torus, and the
    /// saddle, against a plane, a quadric and a torus, turned and moved a
    /// kilometre away.
    #[test]
    fn a_nurbs_patch_is_the_implicit_polynomial_over_it() {
        let binomials = Binomials::new(0);
        let pose = &poses()[2];
        let surfaces = [
            (
                "cylinder",
                twin(
                    &Surface::Cylinder {
                        frame: Frame::world(),
                        radius: 1.0,
                    },
                    [Interval::TURN, Interval::new(-2.0, 2.0).unwrap()],
                ),
            ),
            (
                "sphere",
                twin(
                    &Surface::Sphere {
                        frame: Frame::world(),
                        radius: 2.0,
                    },
                    Surface::Sphere {
                        frame: Frame::world(),
                        radius: 2.0,
                    }
                    .domain(),
                ),
            ),
            (
                "torus",
                twin(&torus(2.0, 0.5), [Interval::TURN, Interval::TURN]),
            ),
            ("saddle", saddle()),
        ];
        let others = [
            Surface::Plane {
                frame: frame(Point3::new(0.3, -0.2, 0.1), Vec3::new(0.4, -0.3, 0.85)),
            },
            Surface::Cone {
                frame: frame(Point3::new(0.2, 0.1, -0.4), Vec3::new(0.2, 0.1, 0.97)),
                radius: 0.6,
                half_angle: 0.4,
            },
            Surface::Torus {
                frame: frame(Point3::new(1.0, 0.1, 0.05), Vec3::new(0.05, 1.0, 0.1)),
                major_radius: 1.5,
                minor_radius: 0.4,
            },
        ];
        for (name, nurbs) in surfaces {
            let nurbs = nurbs.transformed(pose);
            for other in &others {
                let other = other.transformed(pose);
                let section = PatchedSection::on_nurbs(&nurbs, &other, TOL).unwrap();
                let d = section.implicit.degree() as i32;
                for patch in &section.patches {
                    for (s, t) in [(0.0, 0.0), (0.3, 0.8), (1.0, 0.45), (0.5, 0.5), (1.0, 1.0)] {
                        let at = [0, 1].map(|k| section.param(patch, k, [s, t][k]));
                        let [u, v] = section.surface_uv(at);
                        let p = section.implicit.frame.to_local(nurbs.eval(u, v).point);
                        let w = patch.weight.eval(s, t);
                        let coords = [p.x, p.y, p.z, 1.0].map(|c| vec![c]);
                        let [x, y, z, one] = &coords;
                        let direct =
                            section.implicit.along([x, y, z, one], &binomials)[0] * w.powi(d);
                        let value = patch.f.eval(s, t);
                        assert!(
                            (value - direct).abs() <= 4.0 * section.floor,
                            "{name} {:?}: {value} vs {direct}, floor {}",
                            patch.cell,
                            section.floor
                        );
                    }
                }
            }
        }
    }

    /// The twin of a torus turns where the torus does: the spiric
    /// sections' turning points and singular points are the closed form's
    /// points, found across the twin's seams — the figure eight's crossing
    /// and the touch on the outer equator are at `u = 0`, which a full
    /// turn's twin closes over without being periodic.
    #[test]
    fn a_torus_twin_turns_where_the_torus_does() {
        let (major, minor) = (2.0, 0.5);
        let torus = torus(major, minor);
        let nurbs = twin(&torus, [Interval::TURN, Interval::TURN]);
        assert!(nurbs.closure().iter().all(Option::is_some));
        let census = |d: f64| {
            let plane = Surface::Plane {
                frame: frame(Point3::new(d, 0.0, 0.0), Vec3::x()),
            };
            let section = PatchedSection::on_nurbs(&nurbs, &plane, TOL).unwrap();
            let turning = section.turning_points(&mut Meter::default()).unwrap();
            let critical = section.critical_points(&mut Meter::default()).unwrap();
            let singular: Vec<Zero2> = singular(&nurbs, &section, &critical)
                .into_iter()
                .copied()
                .collect();
            (turning, singular)
        };
        let at = |u: f64, v: f64| torus.point(u, v);
        let (turning, singular) = census(1.0);
        let (outer, inner) = ((1.0f64 / 2.5).acos(), (1.0f64 / 1.5).acos());
        matches(
            &nurbs,
            &certified(&turning),
            &[
                at(outer, 0.0),
                at(-outer, 0.0),
                at(inner, PI),
                at(-inner, PI),
            ],
            1e-12,
        );
        assert_eq!(turning.zeros.len(), 4, "{:?}", turning.zeros);
        assert!(singular.is_empty(), "{singular:?}");
        // The figure eight: its crossing, on the seam, is one singular
        // point, and one uncertified turning box.
        let (turning, singular) = census(1.5);
        let outer = (1.5f64 / 2.5).acos();
        matches(
            &nurbs,
            &certified(&turning),
            &[at(outer, 0.0), at(-outer, 0.0)],
            1e-12,
        );
        matches(
            &nurbs,
            &singular.iter().collect::<Vec<_>>(),
            &[at(0.0, PI)],
            1e-12,
        );
        let open: Vec<&Zero2> = turning.zeros.iter().filter(|z| !z.certified).collect();
        assert_eq!(open.len(), 1, "{:?}", turning.zeros);
        assert!(
            (on(&nurbs, open[0]) - at(0.0, PI)).norm() < 1e-5,
            "{open:?}"
        );
        // The touch on the outer equator, also on the seam.
        let (_, singular) = census(2.5);
        matches(
            &nurbs,
            &singular.iter().collect::<Vec<_>>(),
            &[at(0.0, 0.0)],
            1e-12,
        );
        let (turning, singular) = census(2.5 + 1e-3);
        assert!(turning.zeros.is_empty() && singular.is_empty());
    }

    /// The twins of a cylinder and a sphere against a quadric and a
    /// plane, each turning point and singular point where the closed form
    /// puts it.
    #[test]
    fn a_cylinder_and_a_sphere_twin_turn_where_the_closed_form_says() {
        // A sphere centred `e` off the axis of a unit cylinder: `F` over
        // `(θ, z)` is `1 + e² − 2e cos θ + z² − R²`, which turns at `z = 0`
        // where `cos θ = (1 + e² − R²) / 2e`, and touches the wall from
        // inside at `θ = π` where `R = 1 + e`.
        let wall = Surface::Cylinder {
            frame: Frame::world(),
            radius: 1.0,
        };
        let nurbs = twin(&wall, [Interval::TURN, Interval::new(-2.0, 2.0).unwrap()]);
        let e = 0.8;
        for pose in poses() {
            let nurbs = nurbs.transformed(&pose);
            let ball = |radius: f64| {
                Surface::Sphere {
                    frame: frame(Point3::new(e, 0.0, 0.0), Vec3::new(0.1, 0.2, 0.9)),
                    radius,
                }
                .transformed(&pose)
            };
            let moved = |p: Point3| pose.apply(p);
            let cross = ball(1.2);
            let section = PatchedSection::on_nurbs(&nurbs, &cross, TOL).unwrap();
            let turning = section.turning_points(&mut Meter::default()).unwrap();
            let critical = section.critical_points(&mut Meter::default()).unwrap();
            let theta = ((1.0 + e * e - 1.44) / (2.0 * e)).acos();
            let scale = 1e-12 * (1.0 + pose.translation().norm());
            matches(
                &nurbs,
                &certified(&turning),
                &[
                    moved(Point3::new(theta.cos(), theta.sin(), 0.0)),
                    moved(Point3::new(theta.cos(), -theta.sin(), 0.0)),
                ],
                scale,
            );
            assert_eq!(turning.zeros.len(), 2, "{:?}", turning.zeros);
            assert!(singular(&nurbs, &section, &critical).is_empty());
            let touch = ball(1.0 + e);
            let section = PatchedSection::on_nurbs(&nurbs, &touch, TOL).unwrap();
            let critical = section.critical_points(&mut Meter::default()).unwrap();
            matches(
                &nurbs,
                &singular(&nurbs, &section, &critical),
                &[moved(Point3::new(-1.0, 0.0, 0.0))],
                1e-6 * (1.0 + pose.translation().norm()),
            );
        }

        // A sphere of radius 2: the plane `x = d` turns on the equator at
        // `(d, ±√(4 − d²), 0)`, and the cylinder of radius 1.5 about the
        // vertical through `(1, 0, 0)` at the longitudes where
        // `cos u = (4 + 1 − 2.25) / 4`. Neither passes a pole.
        let globe = Surface::Sphere {
            frame: Frame::world(),
            radius: 2.0,
        };
        let nurbs = twin(&globe, globe.domain());
        let d = 0.7;
        let plane = Surface::Plane {
            frame: frame(Point3::new(d, 0.0, 0.0), Vec3::x()),
        };
        let pipe = Surface::Cylinder {
            frame: frame(Point3::new(1.0, 0.0, 0.0), Vec3::z()),
            radius: 1.5,
        };
        let y = (4.0 - d * d).sqrt();
        let u = (2.75f64 / 4.0).acos();
        for (other, expected) in [
            (plane, [Point3::new(d, y, 0.0), Point3::new(d, -y, 0.0)]),
            (
                pipe,
                [
                    Point3::new(2.0 * u.cos(), 2.0 * u.sin(), 0.0),
                    Point3::new(2.0 * u.cos(), -2.0 * u.sin(), 0.0),
                ],
            ),
        ] {
            let section = PatchedSection::on_nurbs(&nurbs, &other, TOL).unwrap();
            let turning = section.turning_points(&mut Meter::default()).unwrap();
            let critical = section.critical_points(&mut Meter::default()).unwrap();
            matches(&nurbs, &certified(&turning), &expected, 1e-12);
            assert_eq!(turning.zeros.len(), 2, "{:?}", turning.zeros);
            assert!(singular(&nurbs, &section, &critical).is_empty());
        }
    }

    /// The bicubic saddle against the plane at its saddle's height: one
    /// certified singular point at the corner four patches share, found
    /// once, and the turning box there uncertified. Above the saddle the
    /// hyperbola turns at `u = ±√h`.
    #[test]
    fn a_bicubic_saddle_is_one_certified_singular_point() {
        let nurbs = saddle();
        let level = |h: f64| Surface::Plane {
            frame: frame(Point3::new(0.0, 0.0, h), Vec3::z()),
        };
        let flat = level(0.0);
        let section = PatchedSection::on_nurbs(&nurbs, &flat, TOL).unwrap();
        let critical = section.critical_points(&mut Meter::default()).unwrap();
        let found = singular(&nurbs, &section, &critical);
        matches(&nurbs, &found, &[Point3::origin()], 1e-14);
        let at = section.surface_uv(found[0].at);
        assert!(at.iter().all(|c| c.abs() < 1e-14), "{found:?}");
        let turning = section.turning_points(&mut Meter::default()).unwrap();
        assert!(certified(&turning).is_empty(), "{:?}", turning.zeros);
        assert!(
            turning.zeros.iter().all(|z| {
                let (lo, hi) = (section.surface_uv(z.lo), section.surface_uv(z.hi));
                (0..2).all(|k| lo[k] <= 1e-6 && -1e-6 <= hi[k])
            }),
            "{:?}",
            turning.zeros
        );
        let raised = level(0.25);
        let section = PatchedSection::on_nurbs(&nurbs, &raised, TOL).unwrap();
        let turning = section.turning_points(&mut Meter::default()).unwrap();
        matches(
            &nurbs,
            &certified(&turning),
            &[Point3::new(0.5, 0.0, 0.25), Point3::new(-0.5, 0.0, 0.25)],
            1e-12,
        );
        assert_eq!(turning.zeros.len(), 2);
        let critical = section.critical_points(&mut Meter::default()).unwrap();
        assert!(singular(&nurbs, &section, &critical).is_empty());
        // Far from the section the polynomial keeps its sign over a box,
        // and the box is cut at the knot it straddles. The saddle's
        // `[−1, 1]` is the walk's `[0, 2π]`.
        let walk = |[a, b]: [f64; 2]| [a, b].map(|x| (x + 1.0) * PI);
        let sign = |u, v| section.sign_over(Probe::F, walk(u), walk(v));
        assert_eq!(sign([-1.0, -0.9], [-0.1, 0.1]), Some(true));
        assert_eq!(sign([-0.1, 0.1], [-0.6, -0.5]), Some(false));
        assert_eq!(sign([0.3, 0.7], [-0.1, 0.1]), None);
    }

    /// The two bicubic faces of CTC-02 the census found reached, each with
    /// its tool's surface (`testdata/ctc02-bicubic-faces.txt`).
    fn ctc02_faces() -> Vec<(&'static str, NurbsSurface, Surface)> {
        let text = include_str!("../testdata/ctc02-bicubic-faces.txt");
        let mut lines = text.lines().filter(|l| !l.starts_with('#'));
        let numbers = |l: &str| -> Vec<f64> {
            l.split_whitespace()
                .filter_map(|w| w.parse::<f64>().ok())
                .collect()
        };
        let mut out = Vec::new();
        for name in ["f544", "f520"] {
            assert_eq!(lines.next(), Some(format!("face {name}").as_str()));
            let head = numbers(lines.next().unwrap());
            let [p, q, n, m] = [0, 1, 2, 3].map(|k| head[k] as usize);
            let ku = numbers(lines.next().unwrap());
            let kv = numbers(lines.next().unwrap());
            let (mut points, mut weights) = (Vec::new(), Vec::new());
            for _ in 0..n * m {
                let row = numbers(lines.next().unwrap());
                points.push(Point3::new(row[0], row[1], row[2]));
                weights.push(row[3]);
            }
            let nurbs = NurbsSurface::new([p, q], [ku, kv], points, weights).unwrap();
            let tool = numbers(lines.next().unwrap());
            let origin = Point3::new(tool[0], tool[1], tool[2]);
            let axis = Vec3::new(tool[3], tool[4], tool[5]);
            let other = match tool.len() {
                6 => Surface::Plane {
                    frame: frame(origin, axis),
                },
                _ => Surface::Cylinder {
                    frame: frame(origin, axis),
                    radius: tool[6],
                },
            };
            out.push((name, nurbs, other));
        }
        out
    }

    /// The census's largest faces traced against the plane and the drill
    /// that meet them (`crate::torus_walk::trace_nurbs`). The face is
    /// mirror-symmetric, and the plane cuts f544 in two mirrored arcs
    /// from its upper edge of `v` to its lower, each turning once just
    /// short of the lower edge, the arm past the turn leaving the face
    /// inside the turning point's cell. The drill passes f520 by, 42 mm
    /// from it at the nearest. Each branch is on both surfaces, each end
    /// on an edge of the face's domain, and a scan of the domain finds
    /// every change of the plane's or the drill's side beside a branch.
    #[test]
    fn the_census_largest_faces_trace_from_edge_to_edge() {
        use crate::BranchEnd;
        for ((name, nurbs, tool), count) in ctc02_faces().into_iter().zip([2, 0]) {
            let face = Surface::Nurbs(nurbs.clone());
            let trace = crate::torus_walk::trace_nurbs(&face, &tool, TOL, &mut Meter::default())
                .unwrap_or_else(|e| panic!("{name}: {e}"));
            let implicit = Implicit::of(&tool).unwrap();
            assert_eq!(trace.branches().len(), count, "{name}");
            let [du, dv] = nurbs.domain();
            let mut samples = Vec::new();
            for branch in trace.branches() {
                assert_eq!(branch.ends(), Some([BranchEnd::Clipped; 2]), "{name}");
                for k in 0..=1024 {
                    let t = branch.domain().lerp(k as f64 / 1024.0);
                    let p = branch.point(t);
                    let uv = branch.uv(t).unwrap();
                    assert!(
                        (nurbs.eval(uv.x, uv.y).point - p).norm() <= 1e-12,
                        "{name} at {t}"
                    );
                    assert!(implicit.distance(p).abs() <= 1e-11, "{name} at {t}");
                    samples.push([du.clamp(uv.x), dv.clamp(uv.y)]);
                }
                for t in [branch.domain().lo(), branch.domain().hi()] {
                    let uv = branch.uv(t).unwrap();
                    let edge =
                        [du.lo(), du.hi()].contains(&uv.x) || [dv.lo(), dv.hi()].contains(&uv.y);
                    assert!(edge, "{name}: {uv:?} is on no edge of {du:?} × {dv:?}");
                }
            }
            // The domain in cells 1/128 of it a side: a cell the other
            // surface's side changes across holds a sample of a branch
            // within a cell of it.
            let scan = 128;
            let at = |i: usize, j: usize| {
                [
                    du.lerp(i as f64 / scan as f64),
                    dv.lerp(j as f64 / scan as f64),
                ]
            };
            let side = |[u, v]: [f64; 2]| implicit.distance(nurbs.eval(u, v).point) > 0.0;
            let cell = [du.length(), dv.length()].map(|l| l / scan as f64);
            for i in 0..scan {
                for j in 0..scan {
                    let corners = [at(i, j), at(i + 1, j), at(i, j + 1), at(i + 1, j + 1)];
                    if corners.iter().all(|&c| side(c) == side(corners[0])) {
                        continue;
                    }
                    let near = samples
                        .iter()
                        .any(|s| (0..2).all(|k| (s[k] - corners[0][k]).abs() <= 2.0 * cell[k]));
                    assert!(
                        near,
                        "{name}: the section through cell ({i}, {j}) is not traced"
                    );
                }
            }
        }
    }

    /// What the isolation costs on the largest faces the census reached
    /// (ADR-0061): CTC-02's two bicubic faces of 16 × 4 spans, a part
    /// a metre long, against the plane and the drill that meet them, and
    /// against a torus through each, whose bidegree (12, 12) is the most
    /// a bicubic patch is put to: 64 patches each. Every turning point is
    /// certified and on the other surface, and the boxes stay within the
    /// torus tracer's measured bound for its sixteen patches together —
    /// 292 at the most, to a depth of 12, some 5 ms in a test build.
    #[test]
    fn the_census_largest_faces_isolate_within_the_torus_bound() {
        // The plane crosses f544 in two arcs that turn once each, the
        // drill passes f520 by, and the torus meets each in two loops
        // that turn twice each.
        let turns = [(2, 4), (0, 4)];
        for ((name, nurbs, tool), (by_tool, by_torus)) in ctc02_faces().into_iter().zip(turns) {
            let [du, dv] = nurbs.domain();
            // A torus about the face's normal at its middle, its tube
            // through the face in two loops: the ring a fifth of the
            // face's narrower width across.
            let at = |u: f64, v: f64| nurbs.eval(du.lerp(u), dv.lerp(v)).point;
            let middle = nurbs.eval(du.lerp(0.5), dv.lerp(0.5));
            let normal = middle.du.cross(&middle.dv).normalize();
            let width = (at(0.0, 0.5) - at(1.0, 0.5))
                .norm()
                .min((at(0.5, 0.0) - at(0.5, 1.0)).norm());
            let ring = Surface::Torus {
                frame: frame(middle.point, normal),
                major_radius: 0.2 * width,
                minor_radius: 0.05 * width,
            };
            for (what, other, expected) in [("tool", tool, by_tool), ("torus", ring, by_torus)] {
                let label = format!("{name} against its {what}");
                let section = PatchedSection::on_nurbs(&nurbs, &other, TOL).unwrap();
                let turning = section.turning_points(&mut Meter::default()).unwrap();
                let critical = section.critical_points(&mut Meter::default()).unwrap();
                assert_eq!(
                    certified(&turning).len(),
                    expected,
                    "{label}: {:?}",
                    turning.zeros
                );
                assert_eq!(
                    turning.zeros.len(),
                    expected,
                    "{label}: {:?}",
                    turning.zeros
                );
                for z in &turning.zeros {
                    // The rounding of a point most of a metre out.
                    let p = on(&nurbs, z);
                    let d = section.implicit.distance(p);
                    let scale = p.coords.norm();
                    assert!(d.abs() <= 1e-12 * scale, "{label}: {d} at {:?}", z.at);
                }
                assert!(singular(&nurbs, &section, &critical).is_empty(), "{label}");
                for census in [&turning, &critical] {
                    assert!(
                        census.depth <= MEASURED_DEPTH,
                        "{label}: depth {}",
                        census.depth
                    );
                    assert!(
                        census.boxes <= MEASURED_BOXES,
                        "{label}: {} boxes",
                        census.boxes
                    );
                }
            }
        }
    }
}
