//! The fuzz targets' shared half (ADR-0024 §5): geometry decoded from
//! bytes, the encoder that writes a fixture's operands back as those
//! bytes for the seed corpus, and the three properties every target
//! asserts of an intersection — it does not panic, every hit lies on both
//! operands within the tolerance, and the same input answers the same way
//! twice.
//!
//! The byte layout is this crate's own, read through
//! [`arbitrary::Unstructured`] so the fuzzer's mutations land on it
//! directly: a tag byte per surface or curve, then each `f64` as eight
//! little-endian bytes. A number outside its range is folded into it, so
//! most mutations still decode to valid geometry, and a number already
//! in range decodes as itself, so a seed written by [`Encoder`] decodes to
//! the operands it came from. A non-finite number, or geometry the kernel
//! rejects as a value (a frame whose axes collapse, a knot vector that is
//! not one), decodes to `None`: the input is skipped, not a finding.

use core::f64::consts::FRAC_PI_2;

use arbitrary::Unstructured;
use arris_geom::{
    Curve, CurveIntersection, CurveSurfaceIntersection, MAX_DEGREE, NurbsCurve, Surface,
    SurfaceIntersection, intersect_surfaces,
};
use arris_math::{Aabb, Frame, Point3, Precision, Tolerance, UnitVec3, Vec3};

/// The largest coordinate a decoded point has: the corpus's parts are
/// within a few hundred of the origin, and past this the question is
/// rounding, not the intersector.
pub const REACH: f64 = 1e3;

/// The smallest radius, and the smallest half-extent of a section's
/// region, a decoded operand has: four orders above the linear
/// tolerance, so a radius is never a tolerance question.
pub const SMALLEST: f64 = 1e-3;

/// The most control points a decoded NURBS curve has, past its
/// `degree + 1`: enough spans for every knot pattern, few enough that
/// one input stays fast.
pub const MAX_EXTRA_POINTS: usize = 12;

/// How many points of a returned curve are checked against both
/// operands, before those outside a surface pair's region are dropped.
pub const SAMPLES: usize = 64;

/// The tolerance every target intersects at: the kernel's default.
pub fn tolerance() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// The number of surface kinds a tag byte chooses among: every analytic
/// kind, no NURBS surface, which every intersector refuses by type.
const SURFACE_KINDS: u8 = 6;

/// The number of curve kinds a tag byte chooses among: the three conics
/// and lines, a NURBS curve given by its control points, and a section
/// of two decoded surfaces — the fitted curve a traced section makes.
const CURVE_KINDS: u8 = 5;

/// `x` in `[lo, hi]`: itself when it is already there, folded into the
/// range when it is finite, `None` when it is not.
fn fold(x: f64, lo: f64, hi: f64) -> Option<f64> {
    if !x.is_finite() {
        return None;
    }
    if (lo..=hi).contains(&x) {
        return Some(x);
    }
    let folded = lo + (x - lo).rem_euclid(hi - lo);
    folded.is_finite().then_some(folded.clamp(lo, hi))
}

/// Reads operands from fuzzer bytes. Every method returns `None` when the
/// bytes run out or decode to nothing the kernel takes as a value.
pub struct Decoder<'a> {
    u: Unstructured<'a>,
}

impl<'a> Decoder<'a> {
    /// A decoder over `data`.
    pub fn new(data: &'a [u8]) -> Self {
        Self {
            u: Unstructured::new(data),
        }
    }

    fn byte(&mut self) -> Option<u8> {
        let b = self.u.bytes(1).ok()?;
        Some(b[0])
    }

    fn raw(&mut self) -> Option<f64> {
        let b = self.u.bytes(8).ok()?;
        Some(f64::from_le_bytes(b.try_into().ok()?))
    }

    fn number(&mut self, lo: f64, hi: f64) -> Option<f64> {
        fold(self.raw()?, lo, hi)
    }

    fn coordinate(&mut self) -> Option<f64> {
        self.number(-REACH, REACH)
    }

    fn radius(&mut self) -> Option<f64> {
        self.number(SMALLEST, REACH)
    }

    fn point(&mut self) -> Option<Point3> {
        Some(Point3::new(
            self.coordinate()?,
            self.coordinate()?,
            self.coordinate()?,
        ))
    }

    fn direction(&mut self) -> Option<Vec3> {
        Some(Vec3::new(
            self.number(-1.0, 1.0)?,
            self.number(-1.0, 1.0)?,
            self.number(-1.0, 1.0)?,
        ))
    }

    fn frame(&mut self) -> Option<Frame> {
        let origin = self.point()?;
        let z = self.direction()?;
        let x = self.direction()?;
        Frame::new(origin, z, x).ok()
    }

    /// Two radii with the first the larger, as an ellipse and an elliptic
    /// cylinder hold them.
    fn radii(&mut self) -> Option<(f64, f64)> {
        let (a, b) = (self.radius()?, self.radius()?);
        Some((a.max(b), a.min(b)))
    }

    /// An analytic surface in a pose.
    pub fn surface(&mut self) -> Option<Surface> {
        Some(match self.byte()? % SURFACE_KINDS {
            0 => Surface::Plane {
                frame: self.frame()?,
            },
            1 => Surface::Cylinder {
                frame: self.frame()?,
                radius: self.radius()?,
            },
            2 => {
                let frame = self.frame()?;
                let (major_radius, minor_radius) = self.radii()?;
                Surface::EllipticCylinder {
                    frame,
                    major_radius,
                    minor_radius,
                }
            }
            3 => Surface::Cone {
                frame: self.frame()?,
                radius: self.radius()?,
                half_angle: self.number(SMALLEST, FRAC_PI_2 - SMALLEST)?,
            },
            4 => Surface::Sphere {
                frame: self.frame()?,
                radius: self.radius()?,
            },
            _ => {
                let frame = self.frame()?;
                let (major_radius, minor_radius) = (self.radius()?, self.radius()?);
                // The tube inside its centre circle (`Surface::Torus`).
                if minor_radius >= major_radius {
                    return None;
                }
                Surface::Torus {
                    frame,
                    major_radius,
                    minor_radius,
                }
            }
        })
    }

    /// The region a surface pair's traced section is clipped to: a cube
    /// about the origin.
    pub fn region(&mut self) -> Option<Aabb> {
        let h = self.number(1.0, REACH)?;
        Some(Aabb {
            min: [-h; 3],
            max: [h; 3],
        })
    }

    /// A curve: a line, a circle, an ellipse, a NURBS curve by its
    /// control points, or one curve of a section of two surfaces.
    pub fn curve(&mut self) -> Option<Curve> {
        Some(match self.byte()? % CURVE_KINDS {
            0 => {
                let origin = self.point()?;
                // Scaled first, as `Frame::new` scales an axis: normalised
                // as drawn, a direction near `1e-154` is not unit, and the
                // line would be the decoder's fault.
                let d = self.direction()?;
                let largest = d.amax();
                if largest == 0.0 {
                    return None;
                }
                Curve::Line {
                    origin,
                    direction: UnitVec3::try_new(d / largest, 0.0)?,
                }
            }
            1 => Curve::Circle {
                frame: self.frame()?,
                radius: self.radius()?,
            },
            2 => {
                let frame = self.frame()?;
                let (major_radius, minor_radius) = self.radii()?;
                Curve::Ellipse {
                    frame,
                    major_radius,
                    minor_radius,
                }
            }
            3 => {
                let degree = 1 + usize::from(self.byte()?) % MAX_DEGREE.min(5);
                let n = degree + 1 + usize::from(self.byte()?) % (MAX_EXTRA_POINTS + 1);
                let mut knots = (0..n + degree + 1)
                    .map(|_| self.coordinate())
                    .collect::<Option<Vec<f64>>>()?;
                knots.sort_by(f64::total_cmp);
                let points = (0..n)
                    .map(|_| self.point())
                    .collect::<Option<Vec<Point3>>>()?;
                let weights = (0..n)
                    .map(|_| self.number(SMALLEST, REACH))
                    .collect::<Option<Vec<f64>>>()?;
                Curve::Nurbs(NurbsCurve::new(degree, knots, points, weights).ok()?)
            }
            _ => {
                let a = self.surface()?;
                let b = self.surface()?;
                let within = self.region()?;
                let pick = usize::from(self.byte()?);
                let hit = intersect_surfaces(
                    &a,
                    &b,
                    &within,
                    tolerance(),
                    &mut arris_math::Meter::default(),
                )
                .ok()?;
                let curves = hit.curves();
                if curves.is_empty() {
                    return None;
                }
                curves[pick % curves.len()].curve.clone()
            }
        })
    }
}

/// Writes operands as the bytes [`Decoder`] reads them back from: the
/// seed corpus's writer.
#[derive(Debug, Default)]
pub struct Encoder {
    bytes: Vec<u8>,
}

impl Encoder {
    /// An empty input.
    pub fn new() -> Self {
        Self::default()
    }

    /// The input so far.
    pub fn finish(self) -> Vec<u8> {
        self.bytes
    }

    fn byte(&mut self, b: u8) {
        self.bytes.push(b);
    }

    fn number(&mut self, x: f64) {
        self.bytes.extend_from_slice(&x.to_le_bytes());
    }

    fn point(&mut self, p: Point3) {
        self.number(p.x);
        self.number(p.y);
        self.number(p.z);
    }

    fn direction(&mut self, v: UnitVec3) {
        self.number(v.x);
        self.number(v.y);
        self.number(v.z);
    }

    fn frame(&mut self, f: &Frame) {
        self.point(f.origin());
        self.direction(f.z());
        self.direction(f.x());
    }

    /// An analytic surface; `false`, writing nothing, for a NURBS one,
    /// which has no layout.
    pub fn surface(&mut self, s: &Surface) -> bool {
        match *s {
            Surface::Plane { ref frame } => {
                self.byte(0);
                self.frame(frame);
            }
            Surface::Cylinder { ref frame, radius } => {
                self.byte(1);
                self.frame(frame);
                self.number(radius);
            }
            Surface::EllipticCylinder {
                ref frame,
                major_radius,
                minor_radius,
            } => {
                self.byte(2);
                self.frame(frame);
                self.number(major_radius);
                self.number(minor_radius);
            }
            Surface::Cone {
                ref frame,
                radius,
                half_angle,
            } => {
                self.byte(3);
                self.frame(frame);
                self.number(radius);
                self.number(half_angle);
            }
            Surface::Sphere { ref frame, radius } => {
                self.byte(4);
                self.frame(frame);
                self.number(radius);
            }
            Surface::Torus {
                ref frame,
                major_radius,
                minor_radius,
            } => {
                self.byte(5);
                self.frame(frame);
                self.number(major_radius);
                self.number(minor_radius);
            }
            Surface::Nurbs(_) => return false,
        }
        true
    }

    /// A region as [`Decoder::region`] reads it: the half-extent of the
    /// cube about the origin.
    pub fn region(&mut self, half_extent: f64) {
        self.number(half_extent);
    }

    /// A curve given by its own values; a NURBS curve by its control
    /// points. `false`, writing nothing, for one past the decoder's
    /// degree or point count.
    pub fn curve(&mut self, c: &Curve) -> bool {
        match *c {
            Curve::Line { origin, direction } => {
                self.byte(0);
                self.point(origin);
                self.direction(direction);
            }
            Curve::Circle { ref frame, radius } => {
                self.byte(1);
                self.frame(frame);
                self.number(radius);
            }
            Curve::Ellipse {
                ref frame,
                major_radius,
                minor_radius,
            } => {
                self.byte(2);
                self.frame(frame);
                self.number(major_radius);
                self.number(minor_radius);
            }
            Curve::Nurbs(ref n) => {
                let degree = n.degree();
                let extra = n.control_points().len().checked_sub(degree + 1);
                let (Some(extra), true) = (extra, (1..=MAX_DEGREE.min(5)).contains(&degree)) else {
                    return false;
                };
                if extra > MAX_EXTRA_POINTS {
                    return false;
                }
                self.byte(3);
                self.byte((degree - 1) as u8);
                self.byte(extra as u8);
                for &k in n.knots() {
                    self.number(k);
                }
                for &p in n.control_points() {
                    self.point(p);
                }
                for &w in n.weights() {
                    self.number(w);
                }
            }
        }
        true
    }

    /// A section curve: the `pick`th curve of `a` against `b` in the
    /// cube of `half_extent` about the origin.
    pub fn section(&mut self, a: &Surface, b: &Surface, half_extent: f64, pick: u8) -> bool {
        let mark = self.bytes.len();
        self.byte(4);
        if !(self.surface(a) && self.surface(b)) {
            self.bytes.truncate(mark);
            return false;
        }
        self.region(half_extent);
        self.byte(pick);
        true
    }
}

/// How far `p` may be from an operand that holds it: the tolerance, and
/// the rounding of coordinates of the size in play.
fn allowance(p: Point3, size: f64, tol: Tolerance) -> f64 {
    tol.linear + 64.0 * f64::EPSILON * (p.coords.norm() + size + 1.0)
}

/// The largest length an operand is given by: its origin's distance and
/// its radii, or its farthest control point.
pub fn surface_size(s: &Surface) -> f64 {
    match *s {
        Surface::Plane { ref frame } => frame.origin().coords.norm(),
        Surface::Cylinder { ref frame, radius } | Surface::Sphere { ref frame, radius } => {
            frame.origin().coords.norm() + radius
        }
        Surface::Cone {
            ref frame, radius, ..
        } => frame.origin().coords.norm() + radius,
        Surface::EllipticCylinder {
            ref frame,
            major_radius,
            ..
        }
        | Surface::Torus {
            ref frame,
            major_radius,
            ..
        } => frame.origin().coords.norm() + 2.0 * major_radius,
        Surface::Nurbs(_) => 0.0,
    }
}

/// As [`surface_size`], for a curve.
pub fn curve_size(c: &Curve) -> f64 {
    match *c {
        Curve::Line { origin, .. } => origin.coords.norm(),
        Curve::Circle { ref frame, radius } => frame.origin().coords.norm() + radius,
        Curve::Ellipse {
            ref frame,
            major_radius,
            ..
        } => frame.origin().coords.norm() + major_radius,
        Curve::Nurbs(ref n) => n
            .control_points()
            .iter()
            .map(|p| p.coords.norm())
            .fold(0.0, f64::max),
    }
}

/// How far `p` is from `s`; `None` where the nearest point is not unique
/// (a point on an axis), which says nothing about whether `p` is on it.
fn off_surface(s: &Surface, p: Point3) -> Option<f64> {
    s.project(p).ok().map(|proj| proj.distance)
}

/// How far `p` is from `c`, when its projection is exact — a line or a
/// conic; a NURBS curve's projection is a local search, and a miss there
/// would be the search's, not the intersector's.
fn off_curve(c: &Curve, p: Point3) -> Option<f64> {
    match c {
        Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. } => {
            c.project(p).ok().map(|proj| proj.distance)
        }
        Curve::Nurbs(_) => None,
    }
}

/// `SAMPLES` parameters across a curve: its domain when bounded, a
/// line's stretch of `reach` either side of its point nearest the origin.
fn samples(c: &Curve, reach: f64) -> Vec<f64> {
    let (lo, hi) = match c {
        Curve::Line { origin, direction } => {
            let mid = -origin.coords.dot(direction);
            (mid - reach, mid + reach)
        }
        Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => {
            let d = c.domain();
            (d.lo(), d.hi())
        }
    };
    (0..SAMPLES)
        .map(|i| lo + (hi - lo) * i as f64 / (SAMPLES - 1) as f64)
        .collect()
}

/// Panics, naming `what`, when `d` is past `allowed`: the finding a
/// fuzz target reports.
fn hold(what: &str, d: Option<f64>, allowed: f64) {
    if let Some(d) = d {
        assert!(d <= allowed, "{what}: {d:e} off, allowed {allowed:e}");
    }
}

/// Whether `p` is in `region`.
fn inside(region: &Aabb, p: Point3) -> bool {
    (0..3).all(|i| (region.min[i]..=region.max[i]).contains(&p[i]))
}

/// The second property of a surface pair: every curve and point of a
/// `Meets` lies on both surfaces within the tolerance, where the section
/// was asked for — a curve's samples inside `within` only. The closed
/// forms return their curves unbounded, and a boolean asks about the
/// region of its faces: two planes a hair from parallel meet in a line
/// 1e10 away, and a ruling there is as far off as rounding at 1e10
/// allows, which is no question the region asked.
pub fn check_surfaces(a: &Surface, b: &Surface, within: &Aabb, hit: &SurfaceIntersection) {
    let tol = tolerance();
    let size = surface_size(a).max(surface_size(b)) + within.diagonal();
    for (i, meet) in hit.curves().iter().enumerate() {
        for t in samples(&meet.curve, 0.5 * within.diagonal()) {
            let p = meet.curve.point(t);
            if !inside(within, p) {
                continue;
            }
            let allowed = allowance(p, size, tol);
            hold(
                &format!("curve {i} at {t} on a"),
                off_surface(a, p),
                allowed,
            );
            hold(
                &format!("curve {i} at {t} on b"),
                off_surface(b, p),
                allowed,
            );
        }
    }
    for (i, m) in hit.points().iter().enumerate() {
        let allowed = allowance(m.point, size, tol);
        hold(&format!("point {i} on a"), off_surface(a, m.point), allowed);
        hold(&format!("point {i} on b"), off_surface(b, m.point), allowed);
    }
}

/// The second property of a curve against a surface: every hit is the
/// curve's point at its `t` and lies on the surface; a bounded curve
/// found `Coincident` lies on the surface all along.
pub fn check_curve_surface(c: &Curve, s: &Surface, hit: &CurveSurfaceIntersection) {
    let tol = tolerance();
    let size = curve_size(c).max(surface_size(s));
    match hit {
        CurveSurfaceIntersection::Points(hits) => {
            for (i, h) in hits.iter().enumerate() {
                let allowed = allowance(h.point, size, tol);
                hold(
                    &format!("hit {i} at t = {} on the curve", h.t),
                    Some((c.point(h.t) - h.point).norm()),
                    allowed,
                );
                hold(
                    &format!("hit {i} at t = {} on the surface", h.t),
                    off_surface(s, h.point),
                    allowed,
                );
            }
        }
        CurveSurfaceIntersection::Coincident => {
            // A line is coincident within the angular tolerance, which
            // leaves it after a length; only a bounded curve is held.
            if matches!(c, Curve::Line { .. }) {
                return;
            }
            for t in samples(c, 0.0) {
                let p = c.point(t);
                hold(
                    &format!("coincident curve at {t}"),
                    off_surface(s, p),
                    allowance(p, size, tol),
                );
            }
        }
    }
}

/// The second property of two curves: every hit is `a`'s point at its
/// `ta` and lies on `b` at its `tb`; two bounded curves found
/// `Coincident` lie on each other where a projection is exact.
pub fn check_curves(a: &Curve, b: &Curve, hit: &CurveIntersection) {
    let tol = tolerance();
    let size = curve_size(a).max(curve_size(b));
    match hit {
        CurveIntersection::Points(hits) => {
            for (i, h) in hits.iter().enumerate() {
                let allowed = allowance(h.point, size, tol);
                hold(
                    &format!("hit {i} at ta = {} on a", h.ta),
                    Some((a.point(h.ta) - h.point).norm()),
                    allowed,
                );
                hold(
                    &format!("hit {i} at tb = {} on b", h.tb),
                    Some((b.point(h.tb) - h.point).norm()),
                    allowed,
                );
            }
        }
        CurveIntersection::Coincident => {
            for (on, other, name) in [(a, b, "a on b"), (b, a, "b on a")] {
                if matches!(on, Curve::Line { .. }) {
                    continue;
                }
                for t in samples(on, 0.0) {
                    let p = on.point(t);
                    hold(
                        &format!("coincident {name} at {t}"),
                        off_curve(other, p),
                        allowance(p, size, tol),
                    );
                }
            }
        }
    }
}

/// The third property: the same call answers the same way twice, bit
/// for bit — compared through `Debug`, which prints a `NaN` as itself
/// where `PartialEq` would not match it.
pub fn check_deterministic<T: core::fmt::Debug>(first: &T, second: &T) {
    let (first, second) = (format!("{first:?}"), format!("{second:?}"));
    assert!(
        first == second,
        "two calls on one input differ:\n{first}\n{second}"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_number_in_range_decodes_as_itself_and_one_outside_folds_in() {
        assert_eq!(fold(3.5, -REACH, REACH), Some(3.5));
        let folded = fold(2500.0, -REACH, REACH).unwrap();
        assert!((-REACH..=REACH).contains(&folded));
        assert_eq!(fold(f64::NAN, 0.0, 1.0), None);
        assert_eq!(fold(f64::INFINITY, 0.0, 1.0), None);
    }

    #[test]
    fn an_encoded_surface_and_curve_decode_to_themselves_to_rounding() {
        let frame = Frame::new(
            Point3::new(1.0, -2.0, 0.5),
            Vec3::new(0.0, 0.6, 0.8),
            Vec3::x(),
        )
        .unwrap();
        let surfaces = [
            Surface::Cone {
                frame,
                radius: 1.5,
                half_angle: 0.4,
            },
            Surface::Torus {
                frame,
                major_radius: 3.0,
                minor_radius: 1.0,
            },
        ];
        let curves = [
            Curve::Ellipse {
                frame,
                major_radius: 2.0,
                minor_radius: 1.0,
            },
            Curve::Nurbs(
                NurbsCurve::new(
                    2,
                    vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                    vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.0, 1.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                    ],
                    vec![1.0, 0.5, 1.0],
                )
                .unwrap(),
            ),
        ];
        let mut e = Encoder::new();
        for s in &surfaces {
            assert!(e.surface(s));
        }
        for c in &curves {
            assert!(e.curve(c));
        }
        let bytes = e.finish();
        let mut d = Decoder::new(&bytes);
        // `Frame::new` rebuilds the axes, which may move each by an ulp;
        // twelve decimals are the same values to rounding.
        let same = |a: &dyn core::fmt::Debug, b: &dyn core::fmt::Debug| {
            assert_eq!(format!("{a:.12?}"), format!("{b:.12?}"));
        };
        for s in &surfaces {
            same(&d.surface(), &Some(s));
        }
        for c in &curves {
            same(&d.curve(), &Some(c));
        }
    }

    #[test]
    fn a_section_decodes_to_the_curve_the_intersector_returns() {
        let wall = Surface::Cylinder {
            frame: Frame::world(),
            radius: 2.0,
        };
        let branch = Surface::Cylinder {
            frame: Frame::from_z(Point3::origin(), Vec3::x()).unwrap(),
            radius: 1.0,
        };
        let mut e = Encoder::new();
        assert!(e.section(&wall, &branch, 10.0, 1));
        let bytes = e.finish();
        let curve = Decoder::new(&bytes).curve().unwrap();
        assert!(matches!(curve, Curve::Nurbs(_)));
        let within = Aabb {
            min: [-10.0; 3],
            max: [10.0; 3],
        };
        let hit = intersect_surfaces(
            &wall,
            &branch,
            &within,
            tolerance(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(curve, hit.curves()[1].curve);
        check_surfaces(&wall, &branch, &within, &hit);
    }
}
