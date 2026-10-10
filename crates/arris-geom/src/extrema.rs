//! Extrema of the distance: every pair of parameters at which the distance
//! between two curves, or from a point to a curve, is stationary — the
//! critical points of `|A(ta) − B(tb)|²` — on the *untrimmed* curves
//! (`docs/DATA-MODEL.md` §Curves, §Extrema). The minimum distance between
//! two trimmed entities is the least of these that lie inside both trims
//! and the same question asked of their boundaries; that recursion is the
//! topology's, not this module's.
//!
//! **What is solved.** Each pair comes down to a polynomial in one
//! unknown, or a system in two, in Bernstein form over one Bézier piece of
//! each bounded curve (a circle or an ellipse is written as its exact
//! rational quadratic arcs, a NURBS curve as its spans):
//!
//! - A point `p` or a line against a bounded curve `C = X / W` is one
//!   unknown. For a line through `o` along `d` the line's parameter is
//!   eliminated, `s = d·(C − o)`, and what is left is the distance from
//!   `C` to the line. With `Q = X − W·o` (or `X − W·p`), `Q⊥` its part
//!   across `d` (all of it for a point) and `N = X′W − XW′` the tangent's
//!   numerator, the critical points are the zeros of `h = Q⊥ · N`, which
//!   is `W³` times `(C − o)⊥ · C′`. Every sign change of `h` is a
//!   candidate ([`crate::bernstein::sign_change_candidates`]); a zero of
//!   even order changes no sign and is no extremum, so no minimum is lost.
//!   A piece's two ends are candidates too: a root there changes no sign
//!   inside it, and a symmetric pose puts one on a conic arc's end.
//! - A bounded curve against a circle is one unknown too: the circle's
//!   closed-form projection eliminates its parameter, and the stationary
//!   pairs at its near and far feet are the roots of one polynomial
//!   ([`against_circle`]). Pins in holes are circles nearly coaxial with
//!   others, which this route takes in microseconds where two unknowns
//!   stall. A point of the curve on the circle's axis is a double zero of
//!   that polynomial, which the subdivision places only to its depth: the
//!   zeros of the curve's components across the axis, and of their
//!   derivatives, are bisected as candidates too.
//! - Two other bounded curves are two unknowns. With
//!   `R = W_B·X_A − W_A·X_B = W_A W_B (A − B)`, the critical points are the
//!   common zeros of `R · N_A` and `R · N_B`, which
//!   [`crate::bernstein2::common_zeros_with`] isolates and certifies
//!   (Krawczyk), as the line against a NURBS surface does (ADR-0053 §7).
//! - Two lines are the closed form.
//!
//! Every candidate is polished by Newton on the curves themselves and kept
//! where the offset between its points has no component along either
//! tangent beyond `tol.linear`. A pair is solved in one canonical order of
//! its two curves and swapped back, so the answer swaps exactly with them.
//!
//! **Continua.** Where the critical points are not isolated, a whole
//! curve of them, the distance is constant along it: a gradient that
//! vanishes along a curve leaves the function no way to change there. That
//! is reported as a *continuum* with its distance and one deterministic
//! pair on it, never as an error. The closed forms come first — two
//! parallel lines, a line or a point on a circle's axis, two coaxial
//! circles (a continuum at the near and one at the far distance), each
//! read as the tolerance reads it, the distance varying along the family by
//! at most `tol.linear`. A piece whose polynomial is below its own rounding
//! everywhere is a continuum where the exact distance stays within
//! `tol.linear` and the exact residual within `tol.linear` of zero at every
//! sample, and otherwise has the residual's sign changes bracketed. In two
//! unknowns the isolation's give-up is the backstop: one circle on the
//! torus about another is a curve of pairs with no common axis. The boxes
//! it had alive, and any it could not certify, are grouped into connected
//! regions and each region searched in one variable along its longer side
//! ([`System::settle`]).
//!
//! **Floors.** A coefficient's rounding is a multiple of the sum of the
//! magnitudes of the products it sums, so every polynomial is built twice:
//! once as it is and once from the absolute values of its inputs, and the
//! second's largest coefficient times [`BERNSTEIN_ROUNDING`] is the first's
//! floor. Both curves are translated to the middle of their control points
//! first, so a pair far from the origin has the floors of one at it.

use arris_math::{Meter, Point3, Tolerance, Vec3, wrap_angle};

use crate::bernstein::{self, Binomials, sign_change_candidates};
use crate::bernstein2::{Halving, Poly2, Zero2, common_zeros_with};
use crate::halt::Halt;
use crate::implicit::BERNSTEIN_ROUNDING;
use crate::{Curve, CurveProjection, GeomError, NurbsCurve};

mod surface;
mod surfaces;

pub use surface::{CurveSurfaceExtrema, CurveSurfacePair, extrema_curve_surface};
pub use surfaces::{SurfaceExtrema, SurfacePair, extrema_surface_surface};

/// A region of uncertified boxes no wider than this, in a piece's own
/// parameters, holds one point: a zero that is not simple, which the
/// isolation halves to the bottom ([`crate::bernstein::MAX_DEPTH`]) and
/// which is a few ulps across. A ratio, not a tolerance: any region along a
/// curve of stationary pairs is the boxes alive at the depth the front
/// outgrew its bound, wider by orders of magnitude.
const POINT_REGION: f64 = 1e-9;

/// How far outside a piece's `[0, 1]`, in its own parameter, a zero is
/// still the piece's: rounding of a zero on the edge the two pieces
/// share, which both find. A ratio, not a tolerance.
const INSIDE: f64 = 1e-9;

/// Newton steps of a polish on the curves' own parameters. Newton
/// converges quadratically near a simple critical point, so the count is
/// only reached by an iteration that does not.
const POLISH_STEPS: usize = 16;

/// One stationary pair of the distance between two curves.
///
/// `a == A(ta)` and `b == B(tb)` to rounding, `distance == |a − b|`, and
/// `a − b` is perpendicular to both tangents within the tolerance. A
/// periodic parameter is in `[0, 2π)`, a NURBS one in its domain.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ExtremumPair {
    /// The first curve's parameter.
    pub ta: f64,
    /// The second curve's parameter.
    pub tb: f64,
    /// `A(ta)`.
    pub a: Point3,
    /// `B(tb)`.
    pub b: Point3,
    /// `|a − b|`.
    pub distance: f64,
}

/// Every stationary pair of the distance between two curves.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Extrema {
    /// The isolated stationary pairs — minima, maxima and saddles alike —
    /// ascending by `ta`, then `tb`.
    pub isolated: Vec<ExtremumPair>,
    /// One pair on each curve of stationary pairs, along which the
    /// distance is constant within `tol.linear`, ascending by distance.
    /// Two curves of one distance are one entry.
    pub continua: Vec<ExtremumPair>,
}

impl Extrema {
    /// The pair of least distance, isolated or on a continuum; the lower
    /// `ta`, then `tb`, of two at one distance. `None` when there is no
    /// stationary pair at all.
    pub fn nearest(&self) -> Option<ExtremumPair> {
        self.isolated
            .iter()
            .chain(&self.continua)
            .copied()
            .min_by(|x, y| {
                x.distance
                    .total_cmp(&y.distance)
                    .then(x.ta.total_cmp(&y.ta))
                    .then(x.tb.total_cmp(&y.tb))
            })
    }

    fn swapped(self) -> Extrema {
        let swap = |p: ExtremumPair| ExtremumPair {
            ta: p.tb,
            tb: p.ta,
            a: p.b,
            b: p.a,
            distance: p.distance,
        };
        Extrema {
            isolated: self.isolated.into_iter().map(swap).collect(),
            continua: self.continua.into_iter().map(swap).collect(),
        }
        .sorted()
    }

    fn sorted(mut self) -> Extrema {
        self.isolated
            .sort_by(|x, y| x.ta.total_cmp(&y.ta).then(x.tb.total_cmp(&y.tb)));
        self.continua.sort_by(|x, y| {
            x.distance
                .total_cmp(&y.distance)
                .then(x.ta.total_cmp(&y.ta))
                .then(x.tb.total_cmp(&y.tb))
        });
        self
    }
}

/// Every foot of a point on a curve or a surface: the points at which the
/// distance from it is stationary.
#[derive(Debug, Clone, PartialEq)]
pub struct Feet<P> {
    /// The isolated feet, ascending by parameter.
    pub isolated: Vec<P>,
    /// One foot on each curve of feet along which the distance is
    /// constant — the circle a point on its axis sees — ascending by
    /// distance.
    pub continua: Vec<P>,
}

impl Curve {
    /// Every foot of `p` on the curve: each parameter at which the
    /// distance from `p` is stationary, nearest and farthest alike, on the
    /// whole curve (a line's both ways, a NURBS curve's whole domain).
    /// Never `Ambiguous`: a point on a circle's axis has the whole circle
    /// for feet, which is one continuum at `t = 0`.
    ///
    /// A line has one foot; a circle two, at `t` and `t + π`; an ellipse
    /// two to four; a NURBS curve those of its spans' polynomials (module
    /// docs), each polished to rounding, plus no end that is not one. Each
    /// foot's `point` is `self.point(t)` and `distance` its distance from
    /// `p`.
    ///
    /// Errors: [`GeomError::InvalidTolerance`], [`GeomError::Interrupted`]
    /// by the meter, ticked once per span.
    ///
    /// ```
    /// use arris_geom::Curve;
    /// use arris_math::{Frame, Meter, Point3, Precision};
    ///
    /// let c = Curve::Circle { frame: Frame::world(), radius: 2.0 };
    /// let tol = Precision::DEFAULT.tolerance();
    /// let feet = c.feet(Point3::new(5.0, 0.0, 0.0), tol, &mut Meter::default()).unwrap();
    /// assert_eq!(feet.isolated.len(), 2);
    /// assert!((feet.isolated[0].distance - 3.0).abs() < 1e-15);
    /// assert!((feet.isolated[1].distance - 7.0).abs() < 1e-15);
    ///
    /// // On the axis every point of the circle is as far.
    /// let axis = c.feet(Point3::new(0.0, 0.0, 1.5), tol, &mut Meter::default()).unwrap();
    /// assert!(axis.isolated.is_empty());
    /// assert!((axis.continua[0].distance - 2.5).abs() < 1e-15);
    /// ```
    pub fn feet(
        &self,
        p: Point3,
        tol: Tolerance,
        meter: &mut Meter,
    ) -> Result<Feet<CurveProjection>, GeomError> {
        if !tol.is_consistent() {
            return Err(GeomError::InvalidTolerance(tol));
        }
        let foot = |t: f64| {
            let point = self.point(t);
            CurveProjection {
                t,
                point,
                distance: (p - point).norm(),
            }
        };
        match self {
            &Curve::Line { origin, direction } => {
                let t = (p - origin).dot(&direction);
                Ok(Feet {
                    isolated: vec![foot(t)],
                    continua: vec![],
                })
            }
            &Curve::Circle { frame, radius } => {
                let q = frame.to_local(p);
                let (rho, r) = (q.x.hypot(q.y), radius);
                // The distance runs from the near foot's to the far one's
                // round the circle; within the tolerance of each other the
                // whole circle is one foot.
                if (r + rho).hypot(q.z) - (r - rho).hypot(q.z) <= tol.linear {
                    return Ok(Feet {
                        isolated: vec![],
                        continua: vec![foot(0.0)],
                    });
                }
                let t = wrap_angle(q.y.atan2(q.x));
                let mut isolated = vec![foot(t), foot(wrap_angle(t + core::f64::consts::PI))];
                isolated.sort_by(|x, y| x.t.total_cmp(&y.t));
                Ok(Feet {
                    isolated,
                    continua: vec![],
                })
            }
            Curve::Ellipse { .. } | Curve::Nurbs(_) => {
                let target = Target::Point(p);
                let mut found = Found::default();
                for piece in pieces(self, target.anchor())? {
                    meter.tick().map_err(GeomError::Interrupted)?;
                    piece.against(&target, tol, &mut found);
                }
                let mut isolated: Vec<CurveProjection> =
                    found.isolated.into_iter().map(|(t, _)| foot(t)).collect();
                isolated.sort_by(|x, y| x.t.total_cmp(&y.t));
                let isolated = dedup(isolated, |x, y| (x.point - y.point).norm() <= tol.linear);
                let mut continua: Vec<CurveProjection> =
                    found.continua.into_iter().map(|(t, _)| foot(t)).collect();
                continua
                    .sort_by(|x, y| x.distance.total_cmp(&y.distance).then(x.t.total_cmp(&y.t)));
                continua.dedup_by(|x, y| (x.distance - y.distance).abs() <= tol.linear);
                Ok(Feet { isolated, continua })
            }
        }
    }
}

/// Every stationary pair of the distance between the curves `a` and `b`
/// on the whole of each (module docs): `a`'s parameter is `ta`.
///
/// Guarantees: every isolated stationary pair is reported, each to
/// rounding, polished on the curves' own parameters, and no two within
/// `tol.linear` on both curves; a curve of stationary pairs is one
/// [`Extrema::continua`] entry; the answer is deterministic, and swapping
/// the operands swaps it. Two lines parallel within `tol.angular` are a
/// continuum at `ta = 0`.
///
/// Errors: [`GeomError::InvalidTolerance`], [`GeomError::Degenerate`] for
/// a conic radius that is not finite and positive, and
/// [`GeomError::Interrupted`] by the meter, ticked once per pair of
/// pieces.
///
/// ```
/// use arris_geom::{Curve, extrema_curve_curve};
/// use arris_math::{Frame, Meter, Point3, Precision, UnitVec3, Vec3};
///
/// let ring = Curve::Circle { frame: Frame::world(), radius: 2.0 };
/// let rod = Curve::Line {
///     origin: Point3::new(0.0, 0.0, 3.0),
///     direction: UnitVec3::new_normalize(Vec3::new(1.0, 0.0, 0.0)),
/// };
/// let tol = Precision::DEFAULT.tolerance();
/// let e = extrema_curve_curve(&ring, &rod, tol, &mut Meter::default()).unwrap();
/// let near = e.nearest().unwrap();
/// assert!((near.distance - 3.0).abs() < 1e-12);
/// ```
pub fn extrema_curve_curve(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Extrema, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    // The pair is solved in one order whichever way it is asked, so the
    // answer swaps with the operands exactly, even where a stationary pair
    // lies in a valley flat to within the tolerance and either order's
    // polish could settle anywhere along it.
    if order(b, a).is_lt() {
        return Ok(extrema_curve_curve(b, a, tol, meter)?.swapped());
    }
    match (a, b) {
        (
            &Curve::Line {
                origin: oa,
                direction: da,
            },
            &Curve::Line {
                origin: ob,
                direction: db,
            },
        ) => Ok(line_line(
            a,
            b,
            oa,
            da.into_inner(),
            ob,
            db.into_inner(),
            tol,
        )),
        (
            &Curve::Line { origin, direction },
            Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_),
        ) => line_bounded(a, origin, direction.into_inner(), b, tol, meter),
        (
            Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_),
            &Curve::Line { origin, direction },
        ) => Ok(line_bounded(b, origin, direction.into_inner(), a, tol, meter)?.swapped()),
        (
            Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_),
            Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_),
        ) => bounded_bounded(a, b, tol, meter),
    }
}

/// A total order on curves, by kind and then by the numbers that define
/// them, compared exactly: what makes [`extrema_curve_curve`] solve a pair
/// one way round.
fn order(a: &Curve, b: &Curve) -> core::cmp::Ordering {
    fn key(c: &Curve) -> (u8, Vec<f64>) {
        let frame = |f: &arris_math::Frame| {
            let (o, z, x) = (f.origin(), f.z().into_inner(), f.x().into_inner());
            vec![o.x, o.y, o.z, z.x, z.y, z.z, x.x, x.y, x.z]
        };
        match c {
            Curve::Line { origin, direction } => (
                0,
                vec![
                    origin.x,
                    origin.y,
                    origin.z,
                    direction.x,
                    direction.y,
                    direction.z,
                ],
            ),
            Curve::Circle { frame: f, radius } => (1, [frame(f), vec![*radius]].concat()),
            Curve::Ellipse {
                frame: f,
                major_radius,
                minor_radius,
            } => (2, [frame(f), vec![*major_radius, *minor_radius]].concat()),
            Curve::Nurbs(n) => {
                let mut v = vec![n.degree() as f64];
                v.extend(n.knots());
                v.extend(n.control_points().iter().flat_map(|p| [p.x, p.y, p.z]));
                v.extend(n.weights());
                (3, v)
            }
        }
    }
    let ((ka, va), (kb, vb)) = (key(a), key(b));
    ka.cmp(&kb).then_with(|| {
        va.iter()
            .zip(&vb)
            .map(|(x, y)| x.total_cmp(y))
            .find(|o| o.is_ne())
            .unwrap_or(va.len().cmp(&vb.len()))
    })
}

// --- the closed forms ---------------------------------------------------------

fn pair(a: &Curve, b: &Curve, ta: f64, tb: f64) -> ExtremumPair {
    let (pa, pb) = (a.point(ta), b.point(tb));
    ExtremumPair {
        ta,
        tb,
        a: pa,
        b: pb,
        distance: (pa - pb).norm(),
    }
}

fn line_line(
    a: &Curve,
    b: &Curve,
    oa: Point3,
    da: Vec3,
    ob: Point3,
    db: Vec3,
    tol: Tolerance,
) -> Extrema {
    let cross = da.cross(&db);
    let offset = ob - oa;
    if cross.norm().atan2(da.dot(&db).abs()) <= tol.angular {
        // Parallel: every point of `a` is as far from `b`.
        let tb = (oa - ob).dot(&db);
        return Extrema {
            isolated: vec![],
            continua: vec![pair(a, b, 0.0, tb)],
        };
    }
    let n2 = cross.norm_squared();
    let ta = offset.cross(&db).dot(&cross) / n2;
    let tb = offset.cross(&da).dot(&cross) / n2;
    Extrema {
        isolated: vec![pair(a, b, ta, tb)],
        continua: vec![],
    }
}

/// A line along a circle's axis, as the tolerance reads it: the
/// distance from the line varies round the circle by at most
/// `tol.linear`. With the circle's centre `δ` off the line and its axis at
/// `θ` to it, every point of the circle is within `δ + R sin θ` of where
/// it would be about the line itself, at the distance `R`, so twice that
/// bounds the variation.
fn on_axis(circle: &Curve, origin: Point3, direction: Vec3, tol: Tolerance) -> bool {
    let &Curve::Circle { frame, radius } = circle else {
        return false;
    };
    let z = frame.z().into_inner();
    let off = frame.origin() - origin;
    let delta = (off - off.dot(&direction) * direction).norm();
    2.0 * (delta + radius * direction.cross(&z).norm()) <= tol.linear
}

/// Two circles about one axis as the tolerance reads it — the distance
/// varies along each family of stationary pairs by at most `tol.linear`,
/// bounded as [`on_axis`] bounds a line's — are a continuum at the near
/// distance and one at the far.
fn coaxial(a: &Curve, b: &Curve, tol: Tolerance) -> Option<Extrema> {
    let (
        &Curve::Circle {
            frame: fa,
            radius: ra,
        },
        &Curve::Circle {
            frame: fb,
            radius: rb,
        },
    ) = (a, b)
    else {
        return None;
    };
    let (za, zb) = (fa.z().into_inner(), fb.z().into_inner());
    let off = fb.origin() - fa.origin();
    let delta = (off - off.dot(&za) * za)
        .norm()
        .max((off - off.dot(&zb) * zb).norm());
    if 2.0 * (delta + ra.max(rb) * za.cross(&zb).norm()) > tol.linear {
        return None;
    }
    // `a`'s point at 0 and the point of `b` at its own angle, and the one
    // opposite.
    let q = fb.to_local(a.point(0.0));
    let tb = wrap_angle(q.y.atan2(q.x));
    let mut continua = vec![
        pair(a, b, 0.0, tb),
        pair(a, b, 0.0, wrap_angle(tb + core::f64::consts::PI)),
    ];
    continua.sort_by(|x, y| x.distance.total_cmp(&y.distance));
    continua.dedup_by(|x, y| (x.distance - y.distance).abs() <= tol.linear);
    Some(Extrema {
        isolated: vec![],
        continua,
    })
}

// --- the pieces ---------------------------------------------------------------

/// What a bounded curve's piece is measured against in one unknown.
enum Target {
    Point(Point3),
    Line {
        origin: Point3,
        direction: Vec3,
    },
    /// A plane, measured by the height above it: its critical points are
    /// those of the height, and a curve's crossings are not among them.
    Plane {
        origin: Point3,
        normal: Vec3,
    },
}

impl Target {
    fn anchor(&self) -> Option<Point3> {
        match self {
            Target::Point(p) => Some(*p),
            Target::Line { .. } | Target::Plane { .. } => None,
        }
    }

    /// The point of the target nearest `c`.
    fn nearest(&self, c: Point3) -> Point3 {
        match *self {
            Target::Point(p) => p,
            Target::Line { origin, direction } => origin + (c - origin).dot(&direction) * direction,
            Target::Plane { origin, normal } => c - (c - origin).dot(&normal) * normal,
        }
    }

    /// `(C − X)⊥ · C′` and its derivative in `t`: the half-gradient of the
    /// squared distance from the curve to the target.
    fn residual(&self, curve: &Curve, t: f64) -> (f64, f64) {
        let e = curve.eval(t);
        match *self {
            Target::Point(p) => {
                let r = e.point - p;
                (r.dot(&e.d1), e.d1.norm_squared() + r.dot(&e.d2))
            }
            Target::Line { origin, direction } => {
                let across = |v: Vec3| v - v.dot(&direction) * direction;
                let r = across(e.point - origin);
                let d1 = across(e.d1);
                (r.dot(&e.d1), d1.norm_squared() + r.dot(&e.d2))
            }
            Target::Plane { normal, .. } => (e.d1.dot(&normal), e.d2.dot(&normal)),
        }
    }
}

/// A polynomial in one variable with the bound on its coefficients'
/// magnitudes that its rounding is a multiple of (module docs).
#[derive(Clone)]
struct T1 {
    v: Vec<f64>,
    m: Vec<f64>,
}

impl T1 {
    fn of(v: Vec<f64>) -> T1 {
        let m = v.iter().map(|x| x.abs()).collect();
        T1 { v, m }
    }

    fn mul(&self, o: &T1, b: &Binomials) -> T1 {
        T1 {
            v: bernstein::mul(&self.v, &o.v, b),
            m: bernstein::mul(&self.m, &o.m, b),
        }
    }

    fn derivative(&self) -> T1 {
        let n = self.m.len().saturating_sub(1) as f64;
        T1 {
            v: bernstein::derivative(&self.v),
            m: if self.m.len() < 2 {
                vec![0.0]
            } else {
                self.m.windows(2).map(|w| n * (w[0] + w[1])).collect()
            },
        }
    }

    fn sum(terms: &[(f64, &T1)]) -> T1 {
        let v: Vec<(f64, &[f64])> = terms.iter().map(|(k, p)| (*k, p.v.as_slice())).collect();
        let m: Vec<(f64, &[f64])> = terms
            .iter()
            .map(|(k, p)| (k.abs(), p.m.as_slice()))
            .collect();
        T1 {
            v: bernstein::combine(&v),
            m: bernstein::combine(&m),
        }
    }

    fn floor(&self) -> f64 {
        BERNSTEIN_ROUNDING * self.m.iter().fold(0.0f64, |a, &x| a.max(x))
    }
}

/// [`T1`] in two variables.
struct T2 {
    v: Poly2,
    m: Poly2,
}

impl T2 {
    fn outer(a: &T1, b: &T1) -> T2 {
        T2 {
            v: Poly2::outer(&a.v, &b.v),
            m: Poly2::outer(&a.m, &b.m),
        }
    }

    fn mul(&self, o: &T2, b: &Binomials) -> T2 {
        T2 {
            v: self.v.mul(&o.v, b),
            m: self.m.mul(&o.m, b),
        }
    }

    fn sum(terms: &[(f64, &T2)]) -> T2 {
        let v: Vec<(f64, &Poly2)> = terms.iter().map(|(k, p)| (*k, &p.v)).collect();
        let m: Vec<(f64, &Poly2)> = terms.iter().map(|(k, p)| (k.abs(), &p.m)).collect();
        T2 {
            v: Poly2::combine(&v),
            m: Poly2::combine(&m),
        }
    }

    fn floor(&self) -> f64 {
        BERNSTEIN_ROUNDING * self.m.coefficients().iter().fold(0.0f64, |a, &x| a.max(x))
    }
}

/// One Bézier piece of a bounded curve over its own `[0, 1]`, in
/// homogeneous form translated by `centre`: `x = W·(P − centre)`.
struct Piece<'c> {
    curve: &'c Curve,
    /// The curve's parameters at the piece's ends.
    range: [f64; 2],
    x: [T1; 3],
    w: T1,
    degree: usize,
    centre: Point3,
}

/// The Bézier pieces of a bounded curve, translated to the middle of its
/// control points — or to `anchor`'s side of it, `anchor` being a point it
/// is measured against.
fn pieces(curve: &Curve, anchor: Option<Point3>) -> Result<Vec<Piece<'_>>, GeomError> {
    let nurbs = match curve {
        Curve::Line { .. } => return Ok(vec![]),
        &Curve::Circle { frame, radius } => {
            NurbsCurve::circle(&frame, radius, arris_math::Interval::TURN)?
        }
        &Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => NurbsCurve::ellipse(
            &frame,
            major_radius,
            minor_radius,
            arris_math::Interval::TURN,
        )?,
        Curve::Nurbs(c) => c.clone(),
    };
    let spans = nurbs.bezier_spans();
    let centre = anchor.unwrap_or_else(|| middle(nurbs.control_points()));
    Ok(spans
        .into_iter()
        .map(|span| piece_of(curve, &span, centre))
        .collect())
}

fn middle(points: &[Point3]) -> Point3 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in points {
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

fn piece_of<'c>(curve: &'c Curve, span: &crate::nurbs::BezierSpan<3>, centre: Point3) -> Piece<'c> {
    let mut x: [Vec<f64>; 3] = Default::default();
    let mut w = Vec::new();
    for (a, wk) in &span.control {
        for k in 0..3 {
            x[k].push(a[k] - wk * centre[k]);
        }
        w.push(*wk);
    }
    Piece {
        curve,
        range: [span.lo, span.hi],
        degree: span.control.len().saturating_sub(1),
        x: x.map(T1::of),
        w: T1::of(w),
        centre,
    }
}

/// What the searches found: parameters on the first curve (and the
/// second's, in two unknowns), isolated and on continua.
#[derive(Default)]
struct Found {
    isolated: Vec<(f64, f64)>,
    continua: Vec<(f64, f64)>,
}

impl Piece<'_> {
    /// The curve's parameter at the piece's own `s`: the span's for a
    /// NURBS curve, the angle of the point for a conic, whose rational
    /// arcs are not parametrised by it.
    fn param(&self, s: f64) -> f64 {
        let t = self.range[0] + s * (self.range[1] - self.range[0]);
        match self.curve {
            Curve::Line { .. } | Curve::Nurbs(_) => t,
            &Curve::Circle { frame, .. } => {
                let q = frame.to_local(self.point(s));
                wrap_angle(q.y.atan2(q.x))
            }
            &Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let q = frame.to_local(self.point(s));
                wrap_angle((q.y / minor_radius).atan2(q.x / major_radius))
            }
        }
    }

    /// The piece's point at its own `s`, by de Casteljau.
    fn point(&self, s: f64) -> Point3 {
        let w = eval(&self.w.v, s);
        let c = [0, 1, 2].map(|k| eval(&self.x[k].v, s) / w);
        Point3::new(c[0], c[1], c[2]) + self.centre.coords
    }

    /// `N = X′W − XW′`, the numerator of the tangent, per coordinate.
    fn tangent(&self, b: &Binomials) -> [T1; 3] {
        let dw = self.w.derivative();
        [0, 1, 2].map(|k| {
            let dx = self.x[k].derivative();
            T1::sum(&[(1.0, &dx.mul(&self.w, b)), (-1.0, &self.x[k].mul(&dw, b))])
        })
    }

    /// The critical points of the distance from the piece to `target`, in
    /// one unknown (module docs), added to `found` as the curve's
    /// parameters.
    fn against(&self, target: &Target, tol: Tolerance, found: &mut Found) {
        let b = Binomials::new(3 * self.degree + 1);
        let n = self.tangent(&b);
        // `Q = X − W·(o − centre)`, then its part across the line.
        let (o, d) = match *target {
            Target::Point(p) => (p, None),
            Target::Line { origin, direction } => (origin, Some(direction)),
            Target::Plane { normal, .. } => {
                // The height's derivative is `N · n`; there is no `Q`.
                return self.against_height(target, normal, &n, tol, found);
            }
        };
        let o = o - self.centre;
        let q: [T1; 3] = [0, 1, 2].map(|k| T1::sum(&[(1.0, &self.x[k]), (-o[k], &self.w)]));
        let across: [T1; 3] = match d {
            None => q,
            Some(d) => [0, 1, 2].map(|k| {
                let terms: Vec<(f64, &T1)> = (0..3)
                    .map(|j| (if j == k { 1.0 } else { 0.0 } - d[k] * d[j], &q[j]))
                    .collect();
                T1::sum(&terms)
            }),
        };
        let products: Vec<T1> = (0..3).map(|k| across[k].mul(&n[k], &b)).collect();
        let h = T1::sum(&[
            (1.0, &products[0]),
            (1.0, &products[1]),
            (1.0, &products[2]),
        ]);
        let floor = h.floor();
        let starts = if h.v.iter().all(|c| c.abs() <= floor) {
            // Below its rounding: the distance is constant along the piece
            // as the tolerance reads it, or its stationary points are
            // bracketed on the exact residual instead.
            let measure = |s: f64| {
                let e = self.curve.eval(self.param(s));
                let r = e.point - target.nearest(e.point);
                (r.norm(), slope(r, e.d1))
            };
            if continuum_within(measure, tol) {
                found.continua.push((self.param(0.0), 0.0));
                return;
            }
            sampled_roots(|s| target.residual(self.curve, self.param(s)).0)
        } else {
            with_ends(sign_change_candidates(&h.v, floor))
        };
        for s in starts {
            let t = polish1(self.curve, target, self.param(s), self.range);
            if is_critical1(self.curve, target, t, tol) {
                found.isolated.push((t, 0.0));
            }
        }
    }
}

impl Piece<'_> {
    /// The critical points of the height above a plane of normal `normal`,
    /// in one unknown: the sign changes of `N · n`, the numerator of the
    /// height's derivative, `n` unit.
    fn against_height(
        &self,
        target: &Target,
        normal: Vec3,
        n: &[T1; 3],
        tol: Tolerance,
        found: &mut Found,
    ) {
        let h = T1::sum(&[(normal.x, &n[0]), (normal.y, &n[1]), (normal.z, &n[2])]);
        let floor = h.floor();
        let starts = if h.v.iter().all(|c| c.abs() <= floor) {
            // Constant height as far as rounding says: a continuum where
            // the exact height varies by at most the tolerance, else the
            // exact derivative's sign changes.
            let measure = |s: f64| {
                let e = self.curve.eval(self.param(s));
                let r = e.point - target.nearest(e.point);
                (r.norm(), slope(r, e.d1))
            };
            if continuum_within(measure, tol) {
                found.continua.push((self.param(0.0), 0.0));
                return;
            }
            sampled_roots(|s| target.residual(self.curve, self.param(s)).0)
        } else {
            with_ends(sign_change_candidates(&h.v, floor))
        };
        for s in starts {
            let t = polish1(self.curve, target, self.param(s), self.range);
            if is_critical1(self.curve, target, t, tol) {
                found.isolated.push((t, 0.0));
            }
        }
    }
}

fn eval(c: &[f64], s: f64) -> f64 {
    let mut row = c.to_vec();
    for level in 1..row.len() {
        for i in 0..row.len() - level {
            row[i] = (1.0 - s) * row[i] + s * row[i + 1];
        }
    }
    row.first().copied().unwrap_or(0.0)
}

/// Newton on the curve's own parameter from `t`, each step kept if it
/// lowers the residual and stays within `range` widened by its width (a
/// periodic curve's anywhere).
fn polish1(curve: &Curve, target: &Target, start: f64, range: [f64; 2]) -> f64 {
    newton1(curve, start, range, |t| target.residual(curve, t))
}

/// [`polish1`] on any residual and its derivative in the curve's own
/// parameter.
fn newton1(
    curve: &Curve,
    start: f64,
    range: [f64; 2],
    residual: impl Fn(f64) -> (f64, f64),
) -> f64 {
    let width = (range[1] - range[0]).abs();
    let (lo, hi) = (
        range[0].min(range[1]) - width,
        range[0].max(range[1]) + width,
    );
    let mut t = start;
    let mut best = residual(t).0.abs();
    for _ in 0..POLISH_STEPS {
        let (f, df) = residual(t);
        if f == 0.0 || df == 0.0 || !df.is_finite() {
            break;
        }
        let next = t - f / df;
        let periodic = curve.period().is_some();
        if !(next.is_finite() && (periodic || (lo..=hi).contains(&next))) {
            break;
        }
        let r = residual(next).0.abs();
        if r >= best {
            break;
        }
        (t, best) = (next, r);
    }
    wrap(curve, t)
}

/// A parameter of a periodic curve in its fundamental interval.
fn wrap(curve: &Curve, t: f64) -> f64 {
    match (curve, curve.period()) {
        (Curve::Circle { .. } | Curve::Ellipse { .. }, _) => wrap_angle(t),
        (_, Some(p)) => {
            let lo = curve.domain().lo();
            lo + (t - lo).rem_euclid(p)
        }
        (_, None) => t,
    }
}

/// `t` is a foot on the target within the tolerance: the offset from the
/// target has no component along the tangent beyond `tol.linear`.
fn is_critical1(curve: &Curve, target: &Target, t: f64, tol: Tolerance) -> bool {
    let e = curve.eval(t);
    let along = e.d1.norm();
    if along == 0.0 {
        return true;
    }
    let r = e.point - target.nearest(e.point);
    (r.dot(&e.d1) / along).abs() <= tol.linear
}

// --- one unknown --------------------------------------------------------------

fn line_bounded(
    line: &Curve,
    origin: Point3,
    direction: Vec3,
    other: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Extrema, GeomError> {
    let foot = |t: f64| (other.point(t) - origin).dot(&direction);
    if on_axis(other, origin, direction, tol) {
        return Ok(Extrema {
            isolated: vec![],
            continua: vec![pair(line, other, foot(0.0), 0.0)],
        });
    }
    let target = Target::Line { origin, direction };
    let mut found = Found::default();
    for piece in pieces(other, None)? {
        meter.tick().map_err(GeomError::Interrupted)?;
        piece.against(&target, tol, &mut found);
    }
    let isolated = found
        .isolated
        .into_iter()
        .map(|(t, _)| pair(line, other, foot(t), t))
        .collect();
    let continua = found
        .continua
        .into_iter()
        .map(|(t, _)| pair(line, other, foot(t), t))
        .collect();
    Ok(finish(Extrema { isolated, continua }, tol))
}

/// Sorted, and pairs within `tol.linear` on both curves one, the first in
/// order kept; continua of one distance one.
fn finish(e: Extrema, tol: Tolerance) -> Extrema {
    let mut e = e.sorted();
    e.isolated = dedup(e.isolated, |x, y| {
        (x.a - y.a).norm() <= tol.linear && (x.b - y.b).norm() <= tol.linear
    });
    e.continua
        .dedup_by(|x, y| (x.distance - y.distance).abs() <= tol.linear);
    e
}

fn dedup<T: Copy>(items: Vec<T>, same: impl Fn(&T, &T) -> bool) -> Vec<T> {
    let mut kept: Vec<T> = Vec::with_capacity(items.len());
    for item in items {
        if !kept.iter().any(|k| same(k, &item)) {
            kept.push(item);
        }
    }
    kept
}

// --- against a circle ---------------------------------------------------------

/// A bounded curve `a` against the circle `b` of `frame` and radius
/// `rb`, in one unknown: the circle's own projection eliminates its
/// parameter. With `P′ = A − c` from the circle's centre, `Q` its part
/// across the axis `n` and `ρ = |Q|`, the squared distance from `A` to the
/// circle at the near foot is `|P′|² − 2 rb ρ + rb²` and at the far one
/// `|P′|² + 2 rb ρ + rb²`; either is stationary where
/// `ρ (P′·A′) = ± rb (Q·Q′)`, and the square of that,
/// `|Q|² (P′·A′)² = rb² (Q·Q′)²`, multiplied out by `W⁸` on a piece, is a
/// polynomial whose sign changes hold every stationary pair at either
/// foot. Each is then paired with the foot it is stationary at and
/// polished on both curves. A point of `a` on the circle's axis has every
/// point of the circle for a foot; the pairs stationary there are where
/// `(A − B)·A′ = 0`, solved for `B` in closed form.
fn against_circle(
    a: &Curve,
    b: &Curve,
    frame: arris_math::Frame,
    rb: f64,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Extrema, GeomError> {
    let centre = frame.origin();
    let n = frame.z().into_inner();
    let mut found = Found::default();
    // The distances of the continua of points of `a` on the axis.
    let mut on_axis: Vec<f64> = Vec::new();
    for piece in pieces(a, Some(centre))? {
        meter.tick().map_err(GeomError::Interrupted)?;
        let bin = Binomials::new(8 * piece.degree + 1);
        let tangent = piece.tangent(&bin);
        let across = |v: &[T1; 3]| -> [T1; 3] {
            [0, 1, 2].map(|k| {
                let terms: Vec<(f64, &T1)> = (0..3)
                    .map(|j| (if j == k { 1.0 } else { 0.0 } - n[k] * n[j], &v[j]))
                    .collect();
                T1::sum(&terms)
            })
        };
        let (q, nq) = (across(&piece.x), across(&tangent));
        let dot = |u: &[T1; 3], v: &[T1; 3]| {
            let p: Vec<T1> = (0..3).map(|k| u[k].mul(&v[k], &bin)).collect();
            T1::sum(&[(1.0, &p[0]), (1.0, &p[1]), (1.0, &p[2])])
        };
        let (xn, qq, qn) = (dot(&piece.x, &tangent), dot(&q, &q), dot(&q, &nq));
        let ww = piece.w.mul(&piece.w, &bin);
        let lhs = qq.mul(&xn.mul(&xn, &bin), &bin);
        let rhs = ww.mul(&qn.mul(&qn, &bin), &bin);
        let s = T1::sum(&[(1.0, &lhs), (-rb * rb, &rhs)]);
        let floor = s.floor();
        let starts = if s.v.iter().all(|c| c.abs() <= floor) {
            // Below its rounding, as for one curve against a point or a
            // line: a continuum, or roots of the exact residual at either
            // foot.
            let mut starts = Vec::new();
            for side in [1.0, -1.0] {
                let across = |ta: f64| {
                    let tb = circle_foot(b, a.point(ta), tol).unwrap_or(0.0);
                    if side < 0.0 {
                        wrap_angle(tb + core::f64::consts::PI)
                    } else {
                        tb
                    }
                };
                let measure = |st: f64| {
                    let ta = piece.param(st);
                    let e = a.eval(ta);
                    let r = e.point - b.point(across(ta));
                    (circle_distance(e.point, frame, rb, side), slope(r, e.d1))
                };
                if continuum_within(measure, tol) {
                    let ta = piece.param(0.0);
                    found.continua.push((ta, across(ta)));
                    continue;
                }
                starts.extend(sampled_roots(|st| {
                    let ta = piece.param(st);
                    let e = a.eval(ta);
                    (e.point - b.point(across(ta))).dot(&e.d1)
                }));
            }
            starts
        } else {
            // A zero of even order is a stationary pair too where it is one
            // of each of the two feet's factors: the derivative's sign
            // changes hold it.
            let mut starts = sign_change_candidates(&s.v, floor);
            let ds = s.derivative();
            starts.extend(sign_change_candidates(&ds.v, ds.floor()));
            with_ends(starts)
        };
        // Where the curve touches the circle's axis, the squared condition
        // has a double zero that the subdivision places only to its depth,
        // and a start that far off the axis finds the circle's feet, not
        // the whole circle. The axis crossings are the zeros of the curve's
        // two components across the axis, a simple one where it crosses and
        // a double one, the zero of the component's derivative, where it
        // only touches (tangent to the axis, which the whole circle is a
        // foot of): each bracketed on samples and bisected to rounding.
        let mut starts: Vec<f64> = starts.into_iter().map(|st| piece.param(st)).collect();
        for k in 0..2 {
            let axis = if k == 0 { frame.x() } else { frame.y() }.into_inner();
            let component = |s: f64| axis.dot(&(a.point(piece.param(s)) - centre));
            let slope = |s: f64| axis.dot(&a.eval(piece.param(s)).d1);
            for s in sampled_roots(component)
                .into_iter()
                .chain(sampled_roots(slope))
            {
                let t = piece.param(s);
                if across_axis(a.point(t) - centre, n).norm() <= 0.5 * tol.linear {
                    starts.push(t);
                }
            }
        }
        for ta in starts {
            match circle_partners(a, b, ta, tol, meter)? {
                Partners::All => {
                    found.continua.push((wrap(a, ta), 0.0));
                    on_axis.push(pair(a, b, wrap(a, ta), 0.0).distance);
                }
                Partners::Some(partners) => {
                    for tb in partners {
                        let (x, y) = polish2(a, b, ta, tb);
                        if is_critical2(a, b, x, y, tol) {
                            found.isolated.push((x, y));
                        }
                    }
                }
            }
        }
    }
    let continua: Vec<ExtremumPair> = found
        .continua
        .into_iter()
        .map(|(ta, tb)| pair(a, b, ta, tb))
        .collect();
    // An isolated pair at the distance of a point on the axis is a point of
    // that family, which the search left where it settled in the valley.
    let isolated = found
        .isolated
        .into_iter()
        .map(|(ta, tb)| pair(a, b, ta, tb))
        .filter(|x| !on_axis.iter().any(|d| (d - x.distance).abs() <= tol.linear))
        .collect();
    Ok(finish(Extrema { isolated, continua }, tol))
}

/// The part of `v` across the unit axis `n`.
fn across_axis(v: Vec3, n: Vec3) -> Vec3 {
    v - v.dot(&n) * n
}

/// The distance from `p` to the circle of `frame` and `radius` at its
/// near foot (`side` 1) or its far one (−1).
fn circle_distance(p: Point3, frame: arris_math::Frame, radius: f64, side: f64) -> f64 {
    let q = frame.to_local(p);
    (q.x.hypot(q.y) - side * radius).hypot(q.z)
}

/// How many points of a piece a function is sampled at where its
/// polynomial is below its rounding. A structural bound: such a piece is
/// within rounding of a curve of stationary points, along which the
/// residual is a low-order trigonometric or polynomial function of the
/// piece's own parameter, with at most a handful of sign changes.
const SAMPLES: usize = 64;

/// The component of the offset `r` along the tangent `d`, per unit
/// length: zero at a stationary point.
fn slope(r: Vec3, d: Vec3) -> f64 {
    let n = d.norm();
    if n == 0.0 { 0.0 } else { r.dot(&d) / n }
}

/// A piece is a continuum as the tolerance reads it: at every sample of
/// its `[0, 1]`, `measure` gives a distance within `tol.linear` of the
/// others' and a slope — the offset along the tangent — within
/// `tol.linear` of zero, so every sampled pair is stationary as
/// [`is_critical1`] and [`is_critical2`] read it.
fn continuum_within(measure: impl Fn(f64) -> (f64, f64), tol: Tolerance) -> bool {
    let mut range = (f64::INFINITY, f64::NEG_INFINITY);
    for i in 0..=SAMPLES {
        let (d, g) = measure(i as f64 / SAMPLES as f64);
        if g.abs() > tol.linear {
            return false;
        }
        range = (range.0.min(d), range.1.max(d));
    }
    range.1 - range.0 <= tol.linear
}

/// `starts` and the piece's two ends: a root at an end of `[0, 1]` changes
/// no sign inside it, and the ends of a conic's arcs are its vertices and
/// quadrant points, where a symmetric pose puts its stationary pairs.
fn with_ends(mut starts: Vec<f64>) -> Vec<f64> {
    starts.extend([0.0, 1.0]);
    starts
}

/// The sign changes of `f` between consecutive samples of `[0, 1]`, each
/// bisected to rounding.
fn sampled_roots(f: impl Fn(f64) -> f64) -> Vec<f64> {
    let at: Vec<(f64, f64)> = (0..=SAMPLES)
        .map(|i| {
            let s = i as f64 / SAMPLES as f64;
            (s, f(s))
        })
        .collect();
    at.windows(2)
        .filter(|w| w[0].1 == 0.0 || w[0].1.signum() != w[1].1.signum())
        .filter_map(|w| {
            let bracket = arris_math::Interval::new(w[0].0, w[1].0).ok()?;
            arris_math::roots::newton_in_interval(&f, |_| 0.0, bracket, 0.0).ok()
        })
        .collect()
}

/// The near foot of `p` on the circle `b`, `None` on its axis.
fn circle_foot(b: &Curve, p: Point3, tol: Tolerance) -> Option<f64> {
    let &Curve::Circle { frame, .. } = b else {
        return None;
    };
    let q = frame.to_local(p);
    (q.x.hypot(q.y) > tol.linear * f64::EPSILON).then(|| wrap_angle(q.y.atan2(q.x)))
}

/// What `a`'s point at `ta` may be stationary with on the circle `b`.
enum Partners {
    /// These parameters of the circle: its near and far feet, or, on its
    /// axis, the points `B` with `(A − B)·A′ = 0`.
    Some(Vec<f64>),
    /// Every point of the circle: `A` on its axis with a tangent along it.
    All,
}

fn circle_partners(
    a: &Curve,
    b: &Curve,
    ta: f64,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Partners, GeomError> {
    let &Curve::Circle { frame, radius } = b else {
        return Ok(Partners::Some(vec![]));
    };
    let e = a.eval(ta);
    let feet = b.feet(e.point, tol, meter)?;
    if feet.continua.is_empty() {
        return Ok(Partners::Some(feet.isolated.iter().map(|f| f.t).collect()));
    }
    // `rb (cos t·X + sin t·Y)·A′ = (A − c)·A′`, per unit length of `A′`.
    let n = e.d1.norm();
    if n == 0.0 {
        return Ok(Partners::All);
    }
    let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
    let (alpha, beta) = (radius * x.dot(&e.d1) / n, radius * y.dot(&e.d1) / n);
    let gamma = (e.point - frame.origin()).dot(&e.d1) / n;
    let amplitude = alpha.hypot(beta);
    if amplitude <= tol.linear {
        return Ok(Partners::All);
    }
    let phase = beta.atan2(alpha);
    let c = (gamma / amplitude).clamp(-1.0, 1.0).acos();
    Ok(Partners::Some(vec![
        wrap_angle(phase + c),
        wrap_angle(phase - c),
    ]))
}

// --- two unknowns -------------------------------------------------------------

fn bounded_bounded(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Extrema, GeomError> {
    if let Some(e) = coaxial(a, b, tol) {
        return Ok(e);
    }
    if let &Curve::Circle { frame, radius } = b {
        return against_circle(a, b, frame, radius, tol, meter);
    }
    if let &Curve::Circle { frame, radius } = a {
        return Ok(against_circle(b, a, frame, radius, tol, meter)?.swapped());
    }
    let pa = pieces(a, None)?;
    // One centre for both, so `R`'s difference is of translated points.
    let centre = middle(
        &pa.iter()
            .chain(&pieces(b, None)?)
            .map(|p| p.centre)
            .collect::<Vec<_>>(),
    );
    let pa = pieces(a, Some(centre))?;
    let pb = pieces(b, Some(centre))?;
    let mut found = Found::default();
    for x in &pa {
        for y in &pb {
            meter.tick().map_err(GeomError::Interrupted)?;
            System::of(x, y).solve(tol, &mut found, meter)?;
        }
    }
    let isolated = found
        .isolated
        .into_iter()
        .map(|(ta, tb)| pair(a, b, ta, tb))
        .collect();
    let continua = found
        .continua
        .into_iter()
        .map(|(ta, tb)| pair(a, b, ta, tb))
        .collect();
    Ok(finish(Extrema { isolated, continua }, tol))
}

/// The critical system of two pieces: `R · N_A` and `R · N_B`.
struct System<'p, 'c> {
    a: &'p Piece<'c>,
    b: &'p Piece<'c>,
    e: [Poly2; 2],
    floors: [f64; 2],
}

impl<'p, 'c> System<'p, 'c> {
    fn of(a: &'p Piece<'c>, b: &'p Piece<'c>) -> Self {
        let bin = Binomials::new(3 * a.degree.max(b.degree) + 1);
        let one = T1::of(vec![1.0]);
        let r: [T2; 3] = [0, 1, 2].map(|k| {
            T2::sum(&[
                (1.0, &T2::outer(&a.x[k], &b.w)),
                (-1.0, &T2::outer(&a.w, &b.x[k])),
            ])
        });
        let (na, nb) = (a.tangent(&bin), b.tangent(&bin));
        let e = |n: &[T1; 3], along_s: bool| {
            let terms: Vec<T2> = (0..3)
                .map(|k| {
                    let n2 = if along_s {
                        T2::outer(&n[k], &one)
                    } else {
                        T2::outer(&one, &n[k])
                    };
                    r[k].mul(&n2, &bin)
                })
                .collect();
            T2::sum(&[(1.0, &terms[0]), (1.0, &terms[1]), (1.0, &terms[2])])
        };
        let (ea, eb) = (e(&na, true), e(&nb, false));
        let floors = [ea.floor(), eb.floor()];
        System {
            a,
            b,
            e: [ea.v, eb.v],
            floors,
        }
    }

    /// The curves' parameters at the pieces' own `(s, t)`.
    fn params(&self, st: [f64; 2]) -> (f64, f64) {
        (self.a.param(st[0]), self.b.param(st[1]))
    }

    fn solve(&self, tol: Tolerance, found: &mut Found, meter: &mut Meter) -> Result<(), GeomError> {
        if self
            .e
            .iter()
            .zip(self.floors)
            .all(|(p, f)| p.coefficients().iter().all(|c| c.abs() <= f))
        {
            // Every pair of the two pieces is stationary: one continuum.
            let (ta, tb) = self.params([0.0, 0.0]);
            found
                .continua
                .push(polish_foot(self.a.curve, self.b.curve, ta, tb));
            return Ok(());
        }
        let mut zeros = Vec::new();
        let mut fronts = Vec::new();
        self.isolate(&mut zeros, &mut fronts, meter)?;
        let mut loose = Vec::new();
        for z in zeros {
            // The isolation looks past the unit square by its margin, where
            // the polynomial is no longer the curve's: what is there is the
            // neighbouring piece's to find.
            if !z.at.iter().all(|&x| (-INSIDE..=1.0 + INSIDE).contains(&x)) {
                continue;
            }
            if z.certified {
                let (ta, tb) = self.params(z.at);
                found
                    .isolated
                    .push(polish2(self.a.curve, self.b.curve, ta, tb));
            } else {
                loose.push(z);
            }
        }
        fronts.push(loose);
        for front in fronts {
            for region in regions(&front) {
                self.settle(&region, tol, found);
            }
        }
        Ok(())
    }

    /// The system's common zeros, certified or not, and the boxes alive
    /// where the isolation gave up on a curve of them.
    fn isolate(
        &self,
        zeros: &mut Vec<Zero2>,
        fronts: &mut Vec<Vec<Zero2>>,
        meter: &mut Meter,
    ) -> Result<(), GeomError> {
        match common_zeros_with(
            [&self.e[0], &self.e[1]],
            self.floors,
            &[],
            Halving::LookAhead,
            meter,
        ) {
            Ok(isolation) => {
                zeros.extend(isolation.zeros);
                Ok(())
            }
            Err(Halt::Stopped(stop)) => Err(GeomError::Interrupted(stop)),
            Err(Halt::Fault(continuum)) => {
                zeros.extend(continuum.found);
                fronts.push(continuum.front);
                Ok(())
            }
        }
    }

    /// One region of boxes the isolation did not certify: a box that is
    /// not a simple zero, or boxes alive when it gave up, touching.
    ///
    /// A region no wider than [`POINT_REGION`] holds one point, a zero
    /// that is not simple: Newton from its middle. A wider one lies along
    /// a curve of stationary pairs, or of pairs so nearly stationary that
    /// the certificate could not tell: it is searched in one variable, the
    /// parameter it is longer along. At each of [`SAMPLES`] points across
    /// it the other curve's parameter is moved to its foot there, which
    /// leaves the derivative of the distance along the sampled curve,
    /// `(A − B)·A′` with `B` at the foot, as a function of one parameter.
    /// The distance staying within `tol.linear` along the region is a
    /// continuum; otherwise the sign changes of that derivative are the
    /// region's stationary pairs, bisected and polished on both curves.
    fn settle(&self, region: &[Zero2], tol: Tolerance, found: &mut Found) {
        let (ca, cb) = (self.a.curve, self.b.curve);
        let lo = [0, 1].map(|k| {
            region
                .iter()
                .map(|z| z.lo[k])
                .fold(f64::INFINITY, f64::min)
                .clamp(0.0, 1.0)
        });
        let hi = [0, 1].map(|k| {
            region
                .iter()
                .map(|z| z.hi[k])
                .fold(f64::NEG_INFINITY, f64::max)
                .clamp(0.0, 1.0)
        });
        let extent = [hi[0] - lo[0], hi[1] - lo[1]];
        if extent[0].max(extent[1]) <= POINT_REGION {
            let (ta, tb) = self.params([0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])]);
            let (x, y) = polish2(ca, cb, ta, tb);
            if is_critical2(ca, cb, x, y, tol) {
                found.isolated.push((x, y));
            }
            return;
        }
        // Sampled along `k`; the other parameter starts from the middle of
        // the box nearest each sample along `k`.
        let k = usize::from(extent[1] > extent[0]);
        let middle = |z: &Zero2, j: usize| 0.5 * (z.lo[j] + z.hi[j]);
        let at = |u: f64, start: Option<f64>| -> (f64, f64) {
            let other = start.unwrap_or_else(|| {
                region
                    .iter()
                    .min_by(|x, y| {
                        (middle(x, k) - u)
                            .abs()
                            .total_cmp(&(middle(y, k) - u).abs())
                    })
                    .map_or(0.5, |z| middle(z, 1 - k))
            });
            if k == 0 {
                let (ta, tb) = (
                    self.a.param(u),
                    if start.is_some() {
                        other
                    } else {
                        self.b.param(other)
                    },
                );
                polish_foot(ca, cb, ta, tb)
            } else {
                let (ta, tb) = (
                    if start.is_some() {
                        other
                    } else {
                        self.a.param(other)
                    },
                    self.b.param(u),
                );
                polish_foot_back(ca, cb, ta, tb)
            }
        };
        // The derivative along the sampled curve, per unit length, and the
        // distance, at a pair with the other parameter at its foot.
        let along = |(ta, tb): (f64, f64)| {
            let (ea, eb) = (ca.eval(ta), cb.eval(tb));
            let r = ea.point - eb.point;
            (slope(r, if k == 0 { ea.d1 } else { -eb.d1 }), r.norm())
        };
        let samples: Vec<(f64, (f64, f64), f64, f64)> = (0..=SAMPLES)
            .map(|i| {
                let u = lo[k] + extent[k] * i as f64 / SAMPLES as f64;
                let pair = at(u, None);
                let (g, d) = along(pair);
                (u, pair, g, d)
            })
            .collect();
        let (dl, dh) = samples
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), x| {
                (l.min(x.3), h.max(x.3))
            });
        if dh - dl <= tol.linear && samples.iter().all(|x| x.2.abs() <= tol.linear) {
            let nearest = samples
                .iter()
                .min_by(|x, y| x.3.total_cmp(&y.3))
                .map_or((0.0, 0.0), |x| x.1);
            found.continua.push(nearest);
            return;
        }
        for w in samples.windows(2) {
            let ((u0, p0, g0, _), (u1, _, g1, _)) = (w[0], w[1]);
            if g0 != 0.0 && g0.signum() == g1.signum() {
                continue;
            }
            let held = if k == 0 { p0.1 } else { p0.0 };
            let Ok(bracket) = arris_math::Interval::new(u0, u1) else {
                continue;
            };
            let root = arris_math::roots::newton_in_interval(
                |u| along(at(u, Some(held))).0,
                |_| 0.0,
                bracket,
                0.0,
            );
            let Ok(u) = root else {
                continue;
            };
            let (ta, tb) = at(u, Some(held));
            let (x, y) = polish2(ca, cb, ta, tb);
            if is_critical2(ca, cb, x, y, tol) {
                found.isolated.push((x, y));
            } else if is_critical2(ca, cb, ta, tb, tol) {
                found.isolated.push((ta, tb));
            }
        }
    }
}

/// Boxes grouped into connected regions, touching ones together, each
/// region in the order of its first box.
fn regions(boxes: &[Zero2]) -> Vec<Vec<Zero2>> {
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut parent: Vec<usize> = (0..boxes.len()).collect();
    for i in 0..boxes.len() {
        for j in i + 1..boxes.len() {
            let touch = (0..2)
                .all(|k| boxes[i].lo[k] <= boxes[j].hi[k] && boxes[j].lo[k] <= boxes[i].hi[k]);
            if touch {
                let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
                if ri != rj {
                    parent[ri.max(rj)] = ri.min(rj);
                }
            }
        }
    }
    let mut out: std::collections::BTreeMap<usize, Vec<Zero2>> = Default::default();
    for (i, z) in boxes.iter().enumerate() {
        let r = root(&mut parent, i);
        out.entry(r).or_default().push(*z);
    }
    out.into_values().collect()
}

/// `(A − B)·A′` and `(A − B)·B′` with their Jacobian, at `(ta, tb)`.
fn residual2(a: &Curve, b: &Curve, ta: f64, tb: f64) -> ([f64; 2], [[f64; 2]; 2]) {
    let (ea, eb) = (a.eval(ta), b.eval(tb));
    let r = ea.point - eb.point;
    let f = [r.dot(&ea.d1), r.dot(&eb.d1)];
    let j = [
        [ea.d1.norm_squared() + r.dot(&ea.d2), -ea.d1.dot(&eb.d1)],
        [ea.d1.dot(&eb.d1), -eb.d1.norm_squared() + r.dot(&eb.d2)],
    ];
    (f, j)
}

/// Newton on both curves' own parameters from `(ta, tb)`: the pair of
/// least residual it passes through. Every step is taken, not only one
/// that lowers the residual: near a curve of stationary pairs the system
/// is nearly singular and Newton's path rises before it converges.
fn polish2(a: &Curve, b: &Curve, ta: f64, tb: f64) -> (f64, f64) {
    let size = |f: [f64; 2]| f[0].hypot(f[1]);
    let (mut x, mut y) = (ta, tb);
    let mut best = (size(residual2(a, b, x, y).0), x, y);
    for _ in 0..POLISH_STEPS {
        let (f, j) = residual2(a, b, x, y);
        let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        if size(f) == 0.0 || det == 0.0 || !det.is_finite() {
            break;
        }
        let dx = (f[0] * j[1][1] - f[1] * j[0][1]) / det;
        let dy = (j[0][0] * f[1] - j[1][0] * f[0]) / det;
        if !(dx.is_finite() && dy.is_finite()) {
            break;
        }
        (x, y) = (x - dx, y - dy);
        let r = size(residual2(a, b, x, y).0);
        if r < best.0 {
            best = (r, x, y);
        }
        if dx.abs() <= f64::EPSILON * x.abs().max(1.0)
            && dy.abs() <= f64::EPSILON * y.abs().max(1.0)
        {
            break;
        }
    }
    (wrap(a, best.1), wrap(b, best.2))
}

/// `(ta, tb)` with `tb` moved to the foot of `A(ta)` on `B` nearest it,
/// within `B`'s domain: a point of a continuum, whose Jacobian is
/// singular, is settled one curve at a time.
fn polish_foot(a: &Curve, b: &Curve, ta: f64, tb: f64) -> (f64, f64) {
    let target = Target::Point(a.point(ta));
    let d = b.domain();
    (wrap(a, ta), polish1(b, &target, tb, [d.lo(), d.hi()]))
}

/// `(ta, tb)` with `ta` moved to the foot of `B(tb)` on `A` nearest it.
fn polish_foot_back(a: &Curve, b: &Curve, ta: f64, tb: f64) -> (f64, f64) {
    let (tb, ta) = polish_foot(b, a, tb, ta);
    (ta, tb)
}

/// `(ta, tb)` is stationary within the tolerance: the offset between the
/// points has no component along either tangent beyond `tol.linear`.
fn is_critical2(a: &Curve, b: &Curve, ta: f64, tb: f64, tol: Tolerance) -> bool {
    let (ea, eb) = (a.eval(ta), b.eval(tb));
    let r = ea.point - eb.point;
    let along = |d: Vec3| {
        let n = d.norm();
        n == 0.0 || (r.dot(&d) / n).abs() <= tol.linear
    };
    along(ea.d1) && along(eb.d1)
}
