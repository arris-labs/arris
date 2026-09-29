//! The section of a torus with an analytic surface, looked for in the
//! torus's own `(u, v)`: the other surface's implicit polynomial over the
//! torus's parametrisation, and the points of it that carry the
//! section's topology.
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
//! Two sets of points are isolated on every patch, and merged across
//! the patches' shared edges and the two seams:
//!
//! - the **turning points**, `f = f_t = 0`, where the section is
//!   parallel to `v` and the number of `v` that solve `f(u, ·) = 0`
//!   changes. On `f = 0` the weight drops out of `f_t`, so they are the
//!   turning points of `F(P(u, v))` whatever the patch. Every component
//!   of the section either has one or winds round `u`.
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
//! those of the quotient, the rest of the section.

use core::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, TAU};

use arris_math::{Meter, Tolerance};

use crate::Surface;
use crate::arc::{QUARTER_TAN, quarter_angle, quarter_arc, quarter_param, quarter_weight};
use crate::bernstein::{self, Binomials, sign_change_candidates};
use crate::bernstein2::{Continuum, Gate, Isolation, Poly2, Zero2, common_zeros, merge};
use crate::halt::Halt;
use crate::implicit::{BERNSTEIN_ROUNDING, Implicit};

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
    quarter: [usize; 2],
    f: Poly2,
    weight: Poly2,
}

/// The other surface's polynomial over the sixteen patches of a torus.
pub(crate) struct PatchedSection<'a> {
    /// The other surface.
    pub(crate) implicit: Implicit<'a>,
    patches: Vec<Patch>,
    /// The rounding of a coefficient of `f`.
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

/// The points isolated over the whole torus, as angles.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Census {
    /// `at` is `(u, v)` in `[0, 2π)²`, ascending; a box is in the same
    /// angles, around its `at`, and may reach past either seam.
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
                patches.push(Patch {
                    quarter: [qu, qv],
                    f,
                    weight,
                });
            }
        }
        // No point of the torus is farther from the other frame's origin,
        // and no weight is above one.
        let reach = major + minor + origin.norm();
        let floor = BERNSTEIN_ROUNDING * implicit.magnitude(reach, 1.0);
        let steep = implicit.steepness(reach + tol.linear) * tol.linear;
        Some(PatchedSection {
            implicit,
            patches,
            floor,
            near: floor + steep,
            steep,
            odd_turn: false,
        })
    }

    /// Candidates for the `u` of a tube circle of the torus lying on the
    /// other surface, ascending in `[0, 2π)`: such a circle crosses every
    /// line of constant `v`, where `f` along the line vanishes and
    /// changes sign, or touches zero and its derivative does. The
    /// columns' edges are candidates as they are, since a change of sign
    /// exactly at the end of a polynomial's `[0, 1]` is no variation of
    /// its coefficients. A candidate is a place to look and no more.
    pub(crate) fn tube_circle_candidates(&self) -> Vec<f64> {
        let mut found: Vec<f64> = (0..4).map(|q| q as f64 * FRAC_PI_2).collect();
        for patch in self.patches.iter().filter(|p| p.quarter[1] % 2 == 0) {
            let line = patch.f.along(CIRCLE_ROW);
            let m = line.len().saturating_sub(1) as f64;
            let slope = bernstein::derivative(&line);
            let roots = sign_change_candidates(&line, self.floor)
                .into_iter()
                .chain(sign_change_candidates(&slope, 2.0 * m * self.floor));
            found.extend(roots.map(|s| quarter_angle(patch.quarter[0], s).rem_euclid(TAU)));
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
        (self.patches.iter()).all(|patch| patch.f.sign(self.floor).is_some())
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
    pub(crate) fn deflate(&mut self, u0: f64, order: usize, corrected: bool) {
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
            let (middle, _, root) = chart(patch.quarter[0]);
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
            worst = worst.max(floor);
        }
        self.floor = worst;
        self.near = worst + self.steep;
        self.odd_turn ^= order % 2 == 1;
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
            let floors = [self.floor, 2.0 * n as f64 * self.floor];
            common_zeros([&patch.f, &f_t], floors, None, meter)
        })
    }

    /// The critical points of `F(P(u, v))` where the torus is within
    /// `tol.linear` of the other surface, and possibly others near it:
    /// the singular points of the section are among them, and
    /// [`Implicit::distance`] at each says which. A certified zero is a
    /// critical point with a regular Hessian — a crossing of two
    /// branches or an isolated point, at the tolerance.
    pub(crate) fn critical_points(&self, meter: &mut Meter) -> Result<Census, Halt<Continuum>> {
        let degree = self.implicit.degree();
        let binomials = Binomials::new(4 * degree + 2);
        let d = degree as f64;
        self.census(meter, |patch, meter| {
            let [m, n] = patch.f.degree();
            let (f, w) = (&patch.f, &patch.weight);
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
            let floors = [
                (2.0 * m as f64 + d) * self.floor,
                (2.0 * n as f64 + d) * self.floor,
            ];
            let gate = Gate {
                poly: f,
                floor: self.near,
            };
            common_zeros([&g_s, &g_t], floors, Some(gate), meter)
        })
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
        for (qu, s, turned) in quarters(u) {
            for (qv, t, _) in quarters(v) {
                let patch = self.patches.get(4 * qu + qv)?;
                let [m, n] = patch.f.degree();
                let (m, n) = (m as f64, n as f64);
                let part = match probe {
                    Probe::F => patch.f.restricted(s, t).sign(self.floor),
                    Probe::Fs => patch.f.du().restricted(s, t).sign(2.0 * m * self.floor),
                    Probe::Ft => patch.f.dv().restricted(s, t).sign(2.0 * n * self.floor),
                    Probe::Fvv => {
                        let binomials = Binomials::new(4 * self.implicit.degree() + 4);
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
                        let floor_g = (2.0 * n + d) * self.floor;
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

    /// Candidates for the roots of `f(0, ·)`, as angles ascending: every
    /// sign change of the section's polynomial along `u = 0` has one
    /// within rounding of it, and a candidate need be no root
    /// (`crate::bernstein::sign_change_candidates`).
    pub(crate) fn seam_candidates(&self) -> Vec<f64> {
        let mut found: Vec<f64> = Vec::new();
        for patch in self.patches.iter().filter(|p| p.quarter[0] == 0) {
            let stride = patch.f.degree()[1] + 1;
            let row = &patch.f.coefficients()[..stride];
            found.extend(
                sign_change_candidates(row, self.floor)
                    .into_iter()
                    .map(|t| quarter_angle(patch.quarter[1], t)),
            );
        }
        found.sort_by(f64::total_cmp);
        found
    }

    /// One isolation per patch, its zeros and boxes taken to angles and
    /// those found from two patches made one.
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
            let angles = |p: [f64; 2]| [0, 1].map(|k| quarter_angle(patch.quarter[k], p[k]));
            zeros.extend(found.zeros.iter().map(|z| Zero2 {
                at: angles(z.at),
                lo: angles(z.lo),
                hi: angles(z.hi),
                certified: z.certified,
            }));
        }
        Ok(Census {
            zeros: merge(zeros, Some((TAU, ANGLE_ROUNDING))),
            depth,
            boxes,
        })
    }
}

#[cfg(test)]
mod tests {
    use arris_math::nalgebra::UnitQuaternion;
    use arris_math::{Frame, Isometry, Point3, Precision, Vec3};

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
                        let [u, v] = [0, 1].map(|k| quarter_angle(patch.quarter[k], [s, t][k]));
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
                            patch.quarter,
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
                        let u = quarter_angle(f.quarter[0], s);
                        let factor = (0.5 * (u - u0)).sin().powi(order as i32);
                        let (was, is) = (f.f.eval(s, t), h.f.eval(s, t) * factor);
                        if was.abs() > 1e3 * whole.floor {
                            assert_eq!(was > 0.0, is > 0.0, "{u0}, order {order}, {:?}", f.quarter);
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
}
