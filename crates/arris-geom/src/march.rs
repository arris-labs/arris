//! The marched section of two NURBS surfaces (ADR-0068), as far as it is
//! built: the system of a pair of their Bézier patches, and the loop
//! condition whose zeros seed the section's closed loops.
//!
//! On a patch `P = X/w` over its own `(s, t)` in `[0, 1]²`, `X` and `w`
//! polynomials of degree `(p, q)` in Bernstein form, the section of two
//! patches is the common zeros of the three components of
//! `w₂·X₁ − w₁·X₂` in `(s₁, t₁, s₂, t₂)`: tensor polynomials of degree
//! `(p₁, q₁, p₂, q₂)`, each a sum of products of a polynomial in the
//! first patch's parameters and one in the second's. Three equations in
//! four unknowns leave curves; a fourth picks the points a walk starts
//! from.
//!
//! The **loop condition** is that fourth equation (ADR-0068). The
//! section's tangent is `N₁ × N₂`, the normals' cross product; it is
//! along `∂P₁/∂s₁` exactly where `N₂·∂P₁/∂s₁ = 0`, the section turning in
//! `t₁` there. A closed loop in the first surface's `(u, v)` turns in `v`
//! at least twice, at its highest and its lowest, so solving the section
//! with that equation puts a seed on every loop, and on every open branch
//! that turns. Where the normals are parallel, a singular point of the
//! section, the equation vanishes as well and the box comes back
//! uncertified. A closed loop needs parallel normals inside it (Sederberg
//! and Meyers, *Loop detection in surface patch intersections*, CAGD 5,
//! 1988), so where no two patches' normal cones meet there is none and
//! no pair is isolated; otherwise a pair is pruned where its patches'
//! hulls are apart or the cone of `∂P₁/∂s₁` is nowhere square to that of
//! `N₂`. The alternative ADR-0068 measured against it, the pairs of
//! points whose normals are one line, is its tests' survey.
//!
//! Every derived polynomial is a positive multiple of the quantity it
//! stands for: `w²·∂P/∂s = w·X_s − w_s·X` for a tangent, and
//! `w³·(∂P/∂s × ∂P/∂t) = w·(X_s × X_t) + w_t·(X × X_s) + w_s·(X_t × X)`
//! for the normal; a patch whose weights are one constant drops the
//! weights' terms. The nets are moved to an origin between the two
//! surfaces first, and every rounding floor charges the move: the
//! coordinates' size before it, as ADR-0061 charges a NURBS net moved
//! into an analytic surface's frame.

use arris_math::{Interrupted, Meter, Point3, Vec3, real};

use crate::NurbsSurface;
use crate::bernstein::Binomials;
use crate::bernstein_n::{PolyN, Wrap, Zero, common_zeros, merge};
use crate::halt::Halt;
use crate::implicit::BERNSTEIN_ROUNDING;
use crate::nurbs::Patch;

/// How far past its own patch, as a share of the patch, a certified
/// zero still counts: a zero on a knot line found from both sides is
/// one, and a zero the isolation certifies in the next patch's span
/// (where the polynomial is not the surface) is not this patch's. As
/// `trace_torus`'s.
pub(crate) const PATCH_SLACK: f64 = 1e-9;

/// What [`loop_seeds`] found and what it took.
#[derive(Clone, Debug, Default)]
pub(crate) struct LoopSeeds {
    /// The zeros and the uncertified boxes, in both surfaces' parameters
    /// `(u₁, v₁, u₂, v₂)`, merged across knot lines and seams.
    pub(crate) seeds: Vec<Zero<4>>,
    /// The fronts of the patch pairs whose zeros are not isolated, in
    /// the same parameters.
    pub(crate) continua: Vec<Zero<4>>,
    /// The patch pairs isolated.
    pub(crate) pairs: usize,
    /// The patch pairs pruned before any isolation.
    pub(crate) pruned: usize,
    /// The boxes the isolations looked at, all pairs together.
    pub(crate) boxes: usize,
    /// The deepest halving of any pair.
    pub(crate) depth: usize,
}

/// One Bézier patch as polynomials in its own `(s, t)`, its net moved
/// by `−origin`; a NURBS curve against a NURBS surface reads it too
/// (`crate::spline_nurbs`).
pub(crate) struct Net {
    /// The knot spans the patch is over, `[[u₀, u₁], [v₀, v₁]]`.
    pub(crate) span: [[f64; 2]; 2],
    pub(crate) x: [PolyN<2>; 3],
    pub(crate) w: PolyN<2>,
    /// Some weight differs from the others.
    rational: bool,
    /// The largest `|Xₖ| + |origin|·w` over the net: the size the moved
    /// coordinates' rounding is a share of.
    pub(crate) reach: f64,
    /// The heaviest weight.
    pub(crate) weight: f64,
    /// The higher of the two degrees.
    pub(crate) degree: f64,
    /// The box of the Cartesian control points, moved: it holds the
    /// patch.
    pub(crate) hull: [[f64; 3]; 2],
}

/// The cross product of two vectors of polynomials of one degree.
fn cross(a: &[PolyN<2>; 3], b: &[PolyN<2>; 3], bin: &Binomials) -> [PolyN<2>; 3] {
    let term = |i: usize, j: usize| {
        PolyN::combine(&[(1.0, &a[i].mul(&b[j], bin)), (-1.0, &a[j].mul(&b[i], bin))])
    };
    [term(1, 2), term(2, 0), term(0, 1)]
}

/// `Σ kᵢ·pᵢ` over polynomials of any degrees, each raised to the
/// highest along every axis first.
pub(crate) fn sum<const N: usize>(terms: &[(f64, PolyN<N>)]) -> PolyN<N> {
    let degree: [usize; N] =
        core::array::from_fn(|k| terms.iter().map(|(_, p)| p.degree()[k]).max().unwrap_or(0));
    let raised: Vec<(f64, PolyN<N>)> = terms
        .iter()
        .map(|(k, p)| (*k, p.elevated(degree)))
        .collect();
    let refs: Vec<(f64, &PolyN<N>)> = raised.iter().map(|(k, p)| (*k, p)).collect();
    PolyN::combine(&refs)
}

impl Net {
    pub(crate) fn of(patch: &Patch, origin: Point3) -> Net {
        let [p, q] = patch.degree;
        let o = origin.coords;
        let coord = |k: usize| {
            PolyN::new(
                [p, q],
                patch
                    .net
                    .iter()
                    .map(|h| [h.x, h.y, h.z][k] - o[k] * h.w)
                    .collect(),
            )
        };
        let w0 = patch.net.first().map_or(1.0, |h| h.w);
        let mut hull = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        let (mut reach, mut weight) = (0.0f64, 0.0f64);
        for h in &patch.net {
            let at = [h.x, h.y, h.z];
            for k in 0..3 {
                let c = at[k] / h.w - o[k];
                hull[0][k] = hull[0][k].min(c);
                hull[1][k] = hull[1][k].max(c);
                reach = reach.max(at[k].abs() + o[k].abs() * h.w);
            }
            weight = weight.max(h.w);
        }
        Net {
            span: [patch.u, patch.v],
            x: [coord(0), coord(1), coord(2)],
            w: PolyN::new([p, q], patch.net.iter().map(|h| h.w).collect()),
            rational: patch.net.iter().any(|h| h.w != w0),
            reach,
            weight,
            degree: p.max(q) as f64,
            hull,
        }
    }

    /// A positive multiple of `∂P/∂s` (`along` 0) or `∂P/∂t` (1).
    pub(crate) fn tangent(&self, along: usize, bin: &Binomials) -> Vector {
        let d = |c: &PolyN<2>| c.partial(along);
        let xd = self.x.each_ref().map(d);
        // `Xₛ`'s coefficients are differences of the moved net's, each
        // `2p` of the net's rounding at the most.
        let error = 2.0 * self.degree * self.reach;
        if !self.rational {
            return Vector::of(xd, error);
        }
        let dw = d(&self.w);
        let poly = core::array::from_fn(|k| {
            PolyN::combine(&[
                (1.0, &self.w.mul(&xd[k], bin)),
                (-1.0, &self.x[k].mul(&dw, bin)),
            ])
        });
        Vector::of(poly, 2.0 * self.weight * error)
    }

    /// A positive multiple of `∂P/∂s × ∂P/∂t`.
    fn normal(&self, bin: &Binomials) -> Vector {
        let xs = self.x.each_ref().map(|c| c.partial(0));
        let xt = self.x.each_ref().map(|c| c.partial(1));
        let (ms, mt) = (magnitude(&xs), magnitude(&xt));
        let n = cross(&xs, &xt, bin);
        // A product's rounding is each factor's times the other's size.
        let error = 2.0 * self.degree * self.reach;
        if !self.rational {
            return Vector::of(n, 2.0 * error * (ms + mt));
        }
        let (ws, wt) = (self.w.partial(0), self.w.partial(1));
        let a = cross(&self.x, &xs, bin);
        let b = cross(&xt, &self.x, bin);
        let poly = core::array::from_fn(|k| {
            PolyN::combine(&[
                (1.0, &self.w.mul(&n[k], bin)),
                (1.0, &wt.mul(&a[k], bin)),
                (1.0, &ws.mul(&b[k], bin)),
            ])
        });
        let mx = magnitude(&self.x);
        Vector::of(
            poly,
            6.0 * self.weight * error * (ms + mt + 2.0 * self.degree * mx),
        )
    }
}

/// The largest coefficient of a vector polynomial.
pub(crate) fn magnitude<const N: usize>(v: &[PolyN<N>; 3]) -> f64 {
    v.iter()
        .flat_map(|p| p.coefficients())
        .fold(0.0, |m, c| m.max(c.abs()))
}

/// A vector polynomial in a patch's parameters, with the size of its
/// coefficients and what rounding they carry: `error` times
/// [`BERNSTEIN_ROUNDING`] at the most, charged at the coordinates' size
/// before the move.
pub(crate) struct Vector {
    pub(crate) poly: [PolyN<2>; 3],
    pub(crate) size: f64,
    pub(crate) error: f64,
}

impl Vector {
    fn of(poly: [PolyN<2>; 3], error: f64) -> Vector {
        Vector {
            size: magnitude(&poly),
            poly,
            error,
        }
    }

    /// The floor of `Σᵢ aᵢ·bᵢ`: each factor's rounding times the other's
    /// size, over three terms.
    fn dot_floor(&self, other: &Vector) -> f64 {
        3.0 * BERNSTEIN_ROUNDING * (self.size * other.error + self.error * other.size)
    }

    fn cone(&self) -> Option<Cone> {
        Cone::of(&self.poly, BERNSTEIN_ROUNDING * self.error)
    }
}

/// The cone a vector polynomial's values are in over its patch: every
/// value is a positive combination of its coefficients, so within
/// `half` of `axis`. `None` where it spans a half-space or more, or a
/// coefficient is zero to rounding, at a collapsed row: a value there
/// may be zero, in no cone.
#[derive(Clone, Copy, Debug)]
struct Cone {
    axis: Vec3,
    half: f64,
}

/// The angle between two vectors, `[0, π]`.
fn angle(a: Vec3, b: Vec3) -> f64 {
    real::atan2(a.cross(&b).norm(), a.dot(&b))
}

impl Cone {
    fn of(v: &[PolyN<2>; 3], floor: f64) -> Option<Cone> {
        let c = v.each_ref().map(|p| p.coefficients());
        let vectors: Vec<Vec3> = (0..c[0].len())
            .map(|j| Vec3::new(c[0][j], c[1][j], c[2][j]))
            .collect();
        if vectors.iter().any(|x| x.norm() <= floor) {
            return None;
        }
        let axis = vectors.iter().map(|x| x / x.norm()).sum::<Vec3>();
        if axis.norm() <= floor {
            return None;
        }
        let half = vectors.iter().map(|x| angle(axis, *x)).fold(0.0, f64::max);
        (half < core::f64::consts::FRAC_PI_2).then_some(Cone { axis, half })
    }

    /// Some vector of one cone is parallel to one of the other, either
    /// way round.
    fn meets_either_way(a: Option<Cone>, b: Option<Cone>) -> bool {
        let (Some(a), Some(b)) = (a, b) else {
            return true;
        };
        let theta = angle(a.axis, b.axis);
        theta.min(core::f64::consts::PI - theta) <= a.half + b.half
    }

    /// Some vector of one cone is square to one of the other.
    fn meets_square(a: Option<Cone>, b: Option<Cone>) -> bool {
        let (Some(a), Some(b)) = (a, b) else {
            return true;
        };
        (angle(a.axis, b.axis) - core::f64::consts::FRAC_PI_2).abs() <= a.half + b.half
    }
}

/// Whether the hulls of two moved patches meet, within `slack`.
fn hulls_meet(a: &Net, b: &Net, slack: f64) -> bool {
    (0..3).all(|k| a.hull[0][k] <= b.hull[1][k] + slack && b.hull[0][k] <= a.hull[1][k] + slack)
}

/// `Σᵢ aᵢ ⊗ bᵢ`, the dot product of a vector in the first patch's
/// parameters and one in the second's.
fn dot(a: &[PolyN<2>; 3], b: &[PolyN<2>; 3]) -> PolyN<4> {
    sum(&[0, 1, 2].map(|i| (1.0, PolyN::tensor(&a[i], &b[i]))))
}

/// The section of a patch pair: the components of `w₂·X₁ − w₁·X₂`, and
/// their floor.
fn section(a: &Net, b: &Net) -> ([PolyN<4>; 3], f64) {
    let f = core::array::from_fn(|i| {
        sum(&[
            (1.0, PolyN::tensor(&a.x[i], &b.w)),
            (-1.0, PolyN::tensor(&a.w, &b.x[i])),
        ])
    });
    let floor = BERNSTEIN_ROUNDING * (a.reach * b.weight + a.weight * b.reach);
    (f, floor)
}

/// One patch pair, with what a fourth equation is built from.
struct Pair<'a> {
    a: &'a Net,
    b: &'a Net,
    /// The first patch's tangents along `s₁` and `t₁`.
    tangents: &'a [Vector; 2],
    /// The second patch's normal.
    normal: &'a Vector,
    /// The cones of the first patch's normal and of the second's.
    cones: [Option<Cone>; 2],
    bin: &'a Binomials,
}

/// Four equations in a patch pair's `(s₁, t₁, s₂, t₂)` and their floors.
type System = ([PolyN<4>; 4], [f64; 4]);

/// The section of a pair closed by the loop condition, `None` where the
/// pair is pruned (module docs).
fn turning(pair: &Pair) -> Option<System> {
    let (a, b) = (pair.a, pair.b);
    let ds = &pair.tangents[0];
    let slack = BERNSTEIN_ROUNDING * (a.reach + b.reach);
    if !hulls_meet(a, b, slack) || !Cone::meets_square(ds.cone(), pair.cones[1]) {
        return None;
    }
    let ([f0, f1, f2], floor) = section(a, b);
    let g = dot(&ds.poly, &pair.normal.poly);
    Some((
        [f0, f1, f2, g],
        [floor, floor, floor, ds.dot_floor(pair.normal)],
    ))
}

/// The seeds the loop condition puts on the section of `one` with `two`,
/// over every pair of their Bézier patches the pruning leaves: certified
/// turning points in `v₁`, and uncertified boxes, where a singular point
/// may be.
pub(crate) fn loop_seeds(
    one: &NurbsSurface,
    two: &NurbsSurface,
    meter: &mut Meter,
) -> Result<LoopSeeds, Interrupted> {
    seeds_by(one, two, turning, meter)
}

/// The zeros `condition` gives each patch pair, in the surfaces'
/// parameters and merged; none where no two patches' normal cones meet.
fn seeds_by(
    one: &NurbsSurface,
    two: &NurbsSurface,
    condition: impl Fn(&Pair) -> Option<System>,
    meter: &mut Meter,
) -> Result<LoopSeeds, Interrupted> {
    let origin = middle(one, two);
    let nets =
        |s: &NurbsSurface| -> Vec<Net> { s.patches().iter().map(|p| Net::of(p, origin)).collect() };
    let (first, second) = (nets(one), nets(two));
    let top = first
        .iter()
        .chain(&second)
        .map(|n| n.degree as usize)
        .max()
        .unwrap_or(1);
    let bin = Binomials::new(4 * top + 4);
    let tangents: Vec<[Vector; 2]> = first
        .iter()
        .map(|n| [n.tangent(0, &bin), n.tangent(1, &bin)])
        .collect();
    let cones: Vec<Option<Cone>> = first.iter().map(|n| n.normal(&bin).cone()).collect();
    let normals: Vec<Vector> = second.iter().map(|n| n.normal(&bin)).collect();
    let their_cones: Vec<Option<Cone>> = normals.iter().map(Vector::cone).collect();
    let mut out = LoopSeeds::default();
    let loops = cones
        .iter()
        .any(|a| their_cones.iter().any(|b| Cone::meets_either_way(*a, *b)));
    if !loops {
        out.pruned = first.len() * second.len();
        return Ok(out);
    }
    let mut found = Vec::new();
    for (i, a) in first.iter().enumerate() {
        for (j, b) in second.iter().enumerate() {
            let pair = Pair {
                a,
                b,
                tangents: &tangents[i],
                normal: &normals[j],
                cones: [cones[i], their_cones[j]],
                bin: &bin,
            };
            let Some((system, floors)) = condition(&pair) else {
                out.pruned += 1;
                continue;
            };
            out.pairs += 1;
            let spans = [a.span[0], a.span[1], b.span[0], b.span[1]];
            match common_zeros(system.each_ref(), floors, meter) {
                Ok(isolation) => {
                    out.boxes += isolation.boxes;
                    out.depth = out.depth.max(isolation.depth);
                    found.extend(isolation.zeros.into_iter().filter_map(|z| own(z, spans)));
                }
                Err(Halt::Fault(continuum)) => {
                    out.depth = out.depth.max(continuum.depth);
                    found.extend(continuum.found.into_iter().filter_map(|z| own(z, spans)));
                    out.continua
                        .extend(continuum.front.into_iter().filter_map(|z| own(z, spans)));
                }
                Err(Halt::Stopped(stop)) => return Err(stop),
            }
        }
    }
    let [c1, c2] = [one.closure(), two.closure()];
    let [d1, d2] = [one.domain(), two.domain()];
    let width = [d1[0], d1[1], d2[0], d2[1]]
        .iter()
        .map(|d| d.length())
        .fold(0.0, f64::max);
    let wrap = Wrap {
        period: [c1[0], c1[1], c2[0], c2[1]],
        origin: [d1[0].lo(), d1[1].lo(), d2[0].lo(), d2[1].lo()],
        touch: PATCH_SLACK * width,
    };
    out.seeds = merge(found, &wrap);
    Ok(out)
}

/// The middle of the box of both surfaces' control points: the origin
/// the nets are moved to.
fn middle(one: &NurbsSurface, two: &NurbsSurface) -> Point3 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in one.control_points().iter().chain(two.control_points()) {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    Point3::new(
        0.5 * (lo[0] + hi[0]),
        0.5 * (lo[1] + hi[1]),
        0.5 * (lo[2] + hi[2]),
    )
}

/// A zero of a patch pair's isolation in the surfaces' parameters, if it
/// is the pair's own: a certified one within [`PATCH_SLACK`] of the
/// pair's box, an uncertified one whose box reaches into it, cut to it.
fn own(z: Zero<4>, spans: [[f64; 2]; 4]) -> Option<Zero<4>> {
    let inside = |x: f64| (-PATCH_SLACK..=1.0 + PATCH_SLACK).contains(&x);
    let keep = if z.certified {
        z.at.iter().all(|&x| inside(x))
    } else {
        (0..4).all(|k| z.lo[k] <= 1.0 + PATCH_SLACK && -PATCH_SLACK <= z.hi[k])
    };
    if !keep {
        return None;
    }
    let to = |k: usize, x: f64| spans[k][0] + (spans[k][1] - spans[k][0]) * x;
    let cut = |x: f64| x.clamp(-PATCH_SLACK, 1.0 + PATCH_SLACK);
    Some(Zero {
        at: core::array::from_fn(|k| to(k, z.at[k])),
        lo: core::array::from_fn(|k| to(k, cut(z.lo[k]))),
        hi: core::array::from_fn(|k| to(k, cut(z.hi[k]))),
        certified: z.certified,
    })
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use arris_math::{Frame, Interval, Isometry, nalgebra::UnitQuaternion};
    use core::f64::consts::{FRAC_PI_2, PI};

    use super::*;
    use crate::Surface;

    /// What the measured cost of one surface pair's loop seeds is held
    /// to, with room to spare: 596 boxes at the most and a depth of 27
    /// on the twins, 172 and 21 on the census faces (ADR-0068).
    const MEASURED_BOXES: usize = 1500;
    const MEASURED_DEPTH: usize = 40;

    fn twin(surface: Surface, u: [f64; 2], v: [f64; 2]) -> NurbsSurface {
        let range = |[lo, hi]: [f64; 2]| Interval::new(lo, hi).unwrap();
        surface.to_nurbs([range(u), range(v)]).unwrap()
    }

    /// The alternative ADR-0068 measured: the pairs of points whose
    /// normals are one line, `(P₁ − P₂)·∂P₁/∂s₁ = (P₁ − P₂)·∂P₁/∂t₁ = 0`
    /// and `N₂·∂P₁/∂s₁ = N₂·∂P₁/∂t₁ = 0`. A closed loop that bounds a
    /// disc on both surfaces holds such a pair inside it (Kriezis,
    /// Patrikalakis and Prinz, *Topological and differential-equation
    /// methods for surface intersections*, CAD 24, 1992), off the
    /// section. A pair is pruned where its patches' normal cones do not
    /// meet; its points need not be near each other, so not by hulls.
    fn collinear(pair: &Pair) -> Option<System> {
        let (a, b, bin) = (pair.a, pair.b, pair.bin);
        if !Cone::meets_either_way(pair.cones[0], pair.cones[1]) {
            return None;
        }
        let (f, floor) = section(a, b);
        let size = f
            .iter()
            .flat_map(|p| p.coefficients())
            .fold(0.0, |m: f64, c| m.max(c.abs()));
        // `(w₂·X₁ − w₁·X₂)·D₁`, a positive multiple of `(P₁ − P₂)·D₁`.
        let offset_along = |d: &Vector| {
            let terms: Vec<(f64, PolyN<4>)> = (0..3)
                .flat_map(|i| {
                    [
                        (1.0, PolyN::tensor(&a.x[i].mul(&d.poly[i], bin), &b.w)),
                        (-1.0, PolyN::tensor(&a.w.mul(&d.poly[i], bin), &b.x[i])),
                    ]
                })
                .collect();
            let floor = 3.0 * (size * BERNSTEIN_ROUNDING * d.error + floor * d.size);
            (sum(&terms), floor)
        };
        let [ds, dt] = pair.tangents;
        let ((e0, f0), (e1, f1)) = (offset_along(ds), offset_along(dt));
        let n = pair.normal;
        Some((
            [e0, e1, dot(&ds.poly, &n.poly), dot(&dt.poly, &n.poly)],
            [f0, f1, ds.dot_floor(n), dt.dot_floor(n)],
        ))
    }

    /// The section's tangent along `∂P₁/∂u₁` at a point of both surfaces.
    fn turns(one: &NurbsSurface, two: &NurbsSurface, at: [f64; 4]) -> bool {
        let (a, b) = (one.eval(at[0], at[1]), two.eval(at[2], at[3]));
        let tangent = a.du.cross(&a.dv).cross(&b.du.cross(&b.dv));
        let sine = tangent.cross(&a.du).norm() / (tangent.norm() * a.du.norm());
        let scale = a.point.coords.norm().max(1.0);
        (a.point - b.point).norm() <= 1e-9 * scale && sine <= 1e-6
    }

    /// The normals of both points on the line through them.
    fn collinear_at(one: &NurbsSurface, two: &NurbsSurface, at: [f64; 4]) -> bool {
        let (a, b) = (one.eval(at[0], at[1]), two.eval(at[2], at[3]));
        let (n1, n2) = (a.du.cross(&a.dv), b.du.cross(&b.dv));
        let sine = |x: Vec3, y: Vec3| x.cross(&y).norm() / (x.norm() * y.norm());
        let offset = b.point - a.point;
        let scale = a.point.coords.norm().max(1.0);
        sine(n1, n2) <= 1e-6 && (offset.norm() <= 1e-9 * scale || sine(n1, offset) <= 1e-6)
    }

    /// Twins whose sections are loops inside both domains, away from
    /// every seam and pole, with the turning points in `v₁` the closed
    /// form gives each.
    fn looped_twins() -> Vec<(&'static str, NurbsSurface, NurbsSurface, usize)> {
        let turn = [0.0, 2.0 * PI];
        let pole = [-FRAC_PI_2, FRAC_PI_2];
        // Two spheres meeting in a circle 35° across centred at
        // u = π + 0.3, v = 0.2 on the first, at u = π, v = 0 on the
        // second: it turns in `v` at its top and its bottom.
        let (cu, cv) = (PI + 0.3, 0.2);
        let c = Vec3::new(
            real::cos(cv) * real::cos(cu),
            real::cos(cv) * real::sin(cu),
            real::sin(cv),
        );
        let up = (Vec3::z() - c * c.z).normalize();
        let ball = twin(
            Surface::Sphere {
                frame: Frame::world(),
                radius: 1.0,
            },
            turn,
            pole,
        );
        let other = twin(
            Surface::Sphere {
                frame: Frame::new(Point3::from(c), up, c).unwrap(),
                radius: 0.6,
            },
            turn,
            pole,
        );
        // A cylinder of radius 0.6 along x through one of radius 1 along
        // z, 0.15 off its axis: two loops on the wider, each turning in
        // its `v`, the height, at its top and bottom, and in the
        // narrower's `v`, along x, four times: where y is extreme and
        // where y = 0, the loop farthest out. The wider's seam faces y.
        let wide = twin(
            Surface::Cylinder {
                frame: Frame::new(Point3::origin(), Vec3::z(), Vec3::y()).unwrap(),
                radius: 1.0,
            },
            turn,
            [-2.0, 2.0],
        );
        let narrow = twin(
            Surface::Cylinder {
                frame: Frame::new(Point3::new(0.0, 0.15, 0.0), Vec3::x(), Vec3::z()).unwrap(),
                radius: 0.6,
            },
            turn,
            [-2.0, 2.0],
        );
        // The plane y = 0.85 through a torus of radii 1 and 0.4: one
        // spiric loop around the outer equator at u = π/2, turning in v
        // where cos v = −0.375. The tube's seam is its inner equator.
        let ring = twin(
            Surface::Torus {
                frame: Frame::world(),
                major_radius: 1.0,
                minor_radius: 0.4,
            },
            turn,
            [-PI, PI],
        );
        let wall = twin(
            Surface::Plane {
                frame: Frame::new(Point3::new(0.0, 0.85, 0.0), Vec3::y(), Vec3::x()).unwrap(),
            },
            [-2.0, 2.0],
            [-2.0, 2.0],
        );
        vec![
            ("sphere × sphere", ball.clone(), other.clone(), 2),
            ("sphere × sphere, swapped", other, ball, 2),
            ("crossing cylinders", wide.clone(), narrow.clone(), 4),
            ("crossing cylinders, swapped", narrow, wide, 8),
            ("spiric torus × plane", ring, wall, 2),
        ]
    }

    /// The two bicubic faces of CTC-02 (`testdata/ctc02-bicubic-faces.txt`).
    fn ctc02_faces() -> Vec<(&'static str, NurbsSurface)> {
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
            out.push((
                name,
                NurbsSurface::new([p, q], [ku, kv], points, weights).unwrap(),
            ));
            // The tool's surface.
            lines.next();
        }
        out
    }

    /// Each census face against two copies of itself, with the turning
    /// points in `v₁` of their sections. The face is a long narrow
    /// saddle: along `u` it falls away from its normal by 71 at either
    /// end, 105 out, and across `v` it rises by 2.6, 6.8 out. The first
    /// copy is turned by 0.2 about a tangent oblique to both and crosses
    /// the face through its middle, from edge to edge without turning.
    /// The second is turned a quarter about the normal and over about
    /// `u`, so that the two differ by a definite form near the middle,
    /// tipped by 0.02 about `v` and lifted by 0.45 along the normal: they
    /// meet in one loop some 6 across around the middle, inside both
    /// domains, which turns twice.
    fn moved_copies() -> Vec<(String, NurbsSurface, NurbsSurface, usize)> {
        let mut out = Vec::new();
        for (name, face) in ctc02_faces() {
            let [du, dv] = face.domain();
            let middle = face.eval(du.lerp(0.5), dv.lerp(0.5));
            let (eu, ev) = (middle.du.normalize(), middle.dv.normalize());
            let normal = eu.cross(&ev).normalize();
            let turn =
                |axis: Vec3, by: f64| UnitQuaternion::from_scaled_axis(axis.normalize() * by);
            let about = |turn: UnitQuaternion<f64>, lift: f64| {
                let moved = middle.point.coords + normal * lift;
                Isometry::new(turn, moved - turn * middle.point.coords)
            };
            let crossed = about(turn(eu + ev, 0.2), 0.0);
            let flipped = about(
                turn(ev, 0.02) * turn(eu, PI) * turn(normal, FRAC_PI_2),
                0.45,
            );
            for (what, motion, turns) in [("crossed", crossed, 0), ("flipped", flipped, 2)] {
                let copy = face.transformed(&motion);
                out.push((format!("{name} {what}"), face.clone(), copy, turns));
            }
        }
        out
    }

    /// The cells of `one`'s parameters `[u, v]` across which the side of
    /// `two` changes, on a grid of `cells` over `window`: the section's
    /// footprint, found without the isolation.
    fn crossed_cells(
        one: &NurbsSurface,
        two: &NurbsSurface,
        window: [[f64; 2]; 2],
        cells: usize,
    ) -> Vec<[usize; 2]> {
        let at = |k: usize, i: usize| {
            window[k][0] + (window[k][1] - window[k][0]) * i as f64 / cells as f64
        };
        let side = |i: usize, j: usize| {
            let p = one.eval(at(0, i), at(1, j)).point;
            let near = two.project(p).unwrap();
            let e = two.eval(near.uv.x, near.uv.y);
            (p - near.point).dot(&e.du.cross(&e.dv)) > 0.0
        };
        let sides: Vec<Vec<bool>> = (0..=cells)
            .map(|i| (0..=cells).map(|j| side(i, j)).collect())
            .collect();
        let mut out = Vec::new();
        for i in 0..cells {
            for j in 0..cells {
                let corners = [
                    sides[i][j],
                    sides[i + 1][j],
                    sides[i][j + 1],
                    sides[i + 1][j + 1],
                ];
                if corners.iter().any(|&c| c != corners[0]) {
                    out.push([i, j]);
                }
            }
        }
        out
    }

    /// Every loop of the twins carries the closed form's turning points
    /// in `v₁`, each certified and on both surfaces, in either order of
    /// the surfaces.
    #[test]
    fn march_every_twin_loop_carries_its_turning_points() {
        for (label, one, two, turning_points) in looped_twins() {
            let found = loop_seeds(&one, &two, &mut Meter::default()).unwrap();
            assert!(found.continua.is_empty(), "{label}");
            assert!(
                found.seeds.iter().all(|z| z.certified),
                "{label}: {:?}",
                found.seeds
            );
            assert_eq!(
                found.seeds.len(),
                turning_points,
                "{label}: {:?}",
                found.seeds
            );
            for z in &found.seeds {
                assert!(turns(&one, &two, z.at), "{label}: {z:?}");
            }
            assert!(
                found.boxes <= MEASURED_BOXES,
                "{label}: {} boxes",
                found.boxes
            );
            assert!(
                found.depth <= MEASURED_DEPTH,
                "{label}: depth {}",
                found.depth
            );
        }
    }

    /// The census faces' loop carries its two turning points, the extremes
    /// in `v₁` of the cells a scan finds the section crossing, and the
    /// crossing branch none.
    #[test]
    fn march_the_census_loops_carry_their_turning_points() {
        for (label, one, two, turning_points) in moved_copies() {
            let found = loop_seeds(&one, &two, &mut Meter::default()).unwrap();
            assert!(found.continua.is_empty(), "{label}");
            assert!(
                found.seeds.iter().all(|z| z.certified),
                "{label}: {:?}",
                found.seeds
            );
            assert_eq!(
                found.seeds.len(),
                turning_points,
                "{label}: {:?}",
                found.seeds
            );
            for z in &found.seeds {
                assert!(turns(&one, &two, z.at), "{label}: {z:?}");
            }
            assert!(
                found.boxes <= MEASURED_BOXES,
                "{label}: {} boxes",
                found.boxes
            );
            assert!(
                found.depth <= MEASURED_DEPTH,
                "{label}: depth {}",
                found.depth
            );
            if turning_points == 0 {
                continue;
            }
            // The loop is some 0.01 of `u` across and 0.4 of `v` around
            // the middle: a window twice that, in cells of 1/64 of it.
            let window = [[0.485, 0.515], [0.2, 0.8]];
            let cells = 64;
            let crossed = crossed_cells(&one, &two, window, cells);
            assert!(
                crossed
                    .iter()
                    .all(|c| c.iter().all(|&i| 0 < i && i + 1 < cells)),
                "{label}: the loop reaches the window's edge"
            );
            let v =
                |j: usize| window[1][0] + (window[1][1] - window[1][0]) * j as f64 / cells as f64;
            let cell = (window[1][1] - window[1][0]) / cells as f64;
            let low = crossed.iter().map(|c| c[1]).min().unwrap();
            let high = crossed.iter().map(|c| c[1]).max().unwrap();
            let (lo, hi) = (
                found.seeds[0].at[1].min(found.seeds[1].at[1]),
                found.seeds[0].at[1].max(found.seeds[1].at[1]),
            );
            assert!(
                v(low) - cell <= lo && lo <= v(low + 1),
                "{label}: {lo} against cell {low}"
            );
            assert!(
                v(high) <= hi && hi <= v(high + 1) + cell,
                "{label}: {hi} against cell {high}"
            );
        }
    }

    /// ADR-0068's survey: both loop conditions over the twins and the
    /// census faces, each seed held to its condition, the counts and the
    /// cost printed.
    #[test]
    #[ignore = "the survey ADR-0068 records, both loop conditions with their timings, asserts only what each seed is: run by hand"]
    fn march_the_loop_conditions_compared() {
        let twins = looped_twins()
            .into_iter()
            .map(|(l, a, b, _)| (l.to_string(), a, b));
        let copies = moved_copies().into_iter().map(|(l, a, b, _)| (l, a, b));
        for (label, one, two) in twins.chain(copies) {
            for (name, by_turning) in [("turning", true), ("collinear", false)] {
                let start = Instant::now();
                let found = if by_turning {
                    seeds_by(&one, &two, turning, &mut Meter::default())
                } else {
                    seeds_by(&one, &two, collinear, &mut Meter::default())
                }
                .unwrap();
                let seconds = start.elapsed().as_secs_f64();
                let certified: Vec<_> = found.seeds.iter().filter(|z| z.certified).collect();
                for z in &certified {
                    let holds = if by_turning {
                        turns(&one, &two, z.at)
                    } else {
                        collinear_at(&one, &two, z.at)
                    };
                    assert!(holds, "{label} {name}: {z:?}");
                }
                eprintln!(
                    "{label}, {name}: {} certified, {} open, {} continua; {} pairs isolated, {} pruned; {} boxes, depth {}; {:.3} s, {:.2} ms a pair",
                    certified.len(),
                    found.seeds.len() - certified.len(),
                    found.continua.len(),
                    found.pairs,
                    found.pruned,
                    found.boxes,
                    found.depth,
                    seconds,
                    1e3 * seconds / found.pairs.max(1) as f64
                );
            }
        }
    }
}
