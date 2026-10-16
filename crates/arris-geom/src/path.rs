//! A path in space and the frames that carry a profile along it
//! (ADR-0063): the value `ops::sweep` takes beside a
//! [`Profile`](crate::profile::Profile).
//!
//! A [`Path`] is a G1 chain of 3D lines, circular arcs, helix arcs and
//! NURBS curves. [`Path::frames`] validates it against the model's two
//! tolerances and returns a [`FrameField`]: a right-handed frame
//! `(tangent, normal, binormal)` at every arc length, under one of two
//! laws.
//!
//! - **Rotation-minimising**: the normal turns no more than the tangent
//!   makes it. Closed forms on a line, an arc and a helix; double
//!   reflection (Wang, Jüttler, Zheng, Liu 2008) on a NURBS segment, at a
//!   step chosen by step doubling so the frame's error stays under the
//!   angular tolerance.
//! - **Fixed binormal**: the binormal is the given direction projected
//!   off the tangent. A pointwise formula, so no stepping, and a screw
//!   motion where the path is a helix about that direction.

use arris_math::nalgebra::UnitQuaternion;
use arris_math::sweep_steps::{FRAME_ERROR_FLOOR, FRAME_MAX_TURN, FRAME_MIN_STEP};
use arris_math::{
    Axis, Frame, Interrupted, Interval, Isometry, Meter, Point3, Tolerance, UnitVec3, Vec3,
};

use crate::NurbsCurve;
use crate::integrate::{GAUSS_ORDER, gauss_legendre};

/// One segment of a [`Path`], from the end of the one before.
///
/// ```
/// use arris_geom::PathSegment;
/// use arris_math::Point3;
///
/// let s = PathSegment::LineTo(Point3::new(0.0, 0.0, 5.0));
/// assert!(matches!(s, PathSegment::LineTo(_)));
/// ```
#[derive(Debug, Clone, PartialEq)]
pub enum PathSegment {
    /// A straight line to a point.
    LineTo(Point3),
    /// A circular arc through `via` to `to`, the way the three points
    /// order round the circle: the arc may be longer than half a turn.
    ArcTo {
        /// A point on the arc between the ends.
        via: Point3,
        /// The end point.
        to: Point3,
    },
    /// A helix arc from the current point. Its radius is that point's
    /// distance from `axis`. It turns by `angle` radians about the axis,
    /// right-handed about `axis.direction` when `angle` is positive, and
    /// advances `pitch · angle / 2π` along it: the sign of `angle` is the
    /// sense of travel.
    Helix {
        /// The axis the helix winds about.
        axis: Axis,
        /// The advance along the axis per full turn, signed.
        pitch: f64,
        /// The angle turned, in radians, signed.
        angle: f64,
    },
    /// A NURBS curve, which must start at the current point.
    Nurbs(NurbsCurve),
}

/// A G1 chain of 3D segments: the spine a profile is carried along.
///
/// ```
/// use arris_geom::{Path, PathSegment};
/// use arris_math::Point3;
///
/// let path = Path {
///     start: Point3::origin(),
///     segments: vec![PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0))],
/// };
/// assert_eq!(path.segments.len(), 1);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    /// Where the chain starts.
    pub start: Point3,
    /// The segments, each from the end of the one before.
    pub segments: Vec<PathSegment>,
}

/// How a profile turns along a path.
///
/// ```
/// use arris_geom::FrameLaw;
/// use arris_math::Vec3;
///
/// assert_eq!(FrameLaw::default(), FrameLaw::RotationMinimizing);
/// let _screw = FrameLaw::FixedBinormal(Vec3::z_axis());
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum FrameLaw {
    /// The frame turns no more about the tangent than the path's own
    /// bending forces: parallel transport of the normal plane. The
    /// default, and Open CASCADE's corrected Frenet to its tolerance.
    #[default]
    RotationMinimizing,
    /// The binormal is this direction projected perpendicular to the
    /// tangent: a thread or a spring keeps its profile in the axis'
    /// plane. Refused where the direction runs along the tangent.
    FixedBinormal(UnitVec3),
}

/// Why a [`PathSegment`] has no usable geometry.
///
/// ```
/// use arris_geom::PathDegeneracy;
///
/// assert_eq!(PathDegeneracy::Short, PathDegeneracy::Short);
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PathDegeneracy {
    /// The segment is no longer than the linear tolerance.
    Short,
    /// An arc's three points lie on a line.
    CollinearArc,
    /// A helix starts on its axis, so it has no radius.
    StartOnAxis,
    /// A helix's angle is zero, or its pitch or angle is not finite.
    NoTurn,
    /// A NURBS segment's derivative vanishes or is not finite.
    ZeroSpeed,
}

/// Why a [`Path`] is not one, or has no frames. Every variant names the
/// segment, counted from 0 in [`Path::segments`].
///
/// ```
/// use arris_geom::{Path, PathError};
/// use arris_math::Precision;
///
/// let empty = Path { start: arris_math::Point3::origin(), segments: vec![] };
/// let mut meter = arris_math::Meter::default();
/// let err = empty.frames(Default::default(), Precision::DEFAULT.tolerance(), &mut meter);
/// assert_eq!(err.unwrap_err(), PathError::Empty);
/// ```
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PathError {
    /// The path has no segments.
    #[error("the path has no segments")]
    Empty,
    /// A point, radius, pitch or angle of the segment is not finite.
    #[error("segment {segment} has a number that is not finite")]
    NotFinite {
        /// The segment.
        segment: usize,
    },
    /// The segment has no usable geometry.
    #[error("segment {segment} is degenerate: {reason:?}")]
    Degenerate {
        /// The segment.
        segment: usize,
        /// Why.
        reason: PathDegeneracy,
    },
    /// A NURBS segment starts away from the end of the one before.
    #[error("segment {segment} starts {distance} from where the path has reached")]
    Gap {
        /// The segment.
        segment: usize,
        /// How far.
        distance: f64,
    },
    /// The path is not G1 at `at` (arc length from its start): the
    /// tangent turns by `angle` there. Segment 0 names the closing joint
    /// of a path that returns to its start.
    #[error("the path has a corner of {angle} rad at arc length {at}, entering segment {segment}")]
    Corner {
        /// The segment the corner enters.
        segment: usize,
        /// The arc length from the path's start.
        at: f64,
        /// The angle between the two tangents.
        angle: f64,
    },
    /// The fixed binormal runs along the tangent of the segment at `at`.
    #[error("the binormal runs along the tangent of segment {segment} at arc length {at}")]
    BinormalAlongTangent {
        /// The segment.
        segment: usize,
        /// The arc length from the path's start.
        at: f64,
    },
    /// The frame table of a NURBS segment needed a step under
    /// [`FRAME_MIN_STEP`]: a cusp or a jump in the curve.
    #[error("segment {segment}: the frame needs a step too small to take; the curve has a cusp")]
    FrameStep {
        /// The segment.
        segment: usize,
    },
    /// The caller stopped the work.
    #[error(transparent)]
    Interrupted(#[from] Interrupted),
}

/// The frame of a path at one arc length: a point and a right-handed
/// orthonormal triple with `tangent × normal = binormal`.
///
/// ```
/// use arris_geom::{FrameLaw, Path, PathSegment};
/// use arris_math::{Meter, Point3, Precision};
///
/// let path = Path {
///     start: Point3::origin(),
///     segments: vec![PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0))],
/// };
/// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
/// let f = field.at(4.0);
/// assert!((f.origin - Point3::new(0.0, 0.0, 4.0)).norm() < 1e-15);
/// assert!((f.tangent.cross(&f.normal) - f.binormal.into_inner()).norm() < 1e-15);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathFrame {
    /// The path's point.
    pub origin: Point3,
    /// The unit tangent, the direction of travel.
    pub tangent: UnitVec3,
    /// The unit normal.
    pub normal: UnitVec3,
    /// The unit binormal, `tangent × normal`.
    pub binormal: UnitVec3,
}

impl PathFrame {
    /// The frame as a [`Frame`]: `x` the normal, `y` the binormal, `z`
    /// the tangent, so `x × y = z`. A profile drawn in its own
    /// plane with `x` along the normal and `y` along the binormal is
    /// carried by the motion from the field's first frame to this one.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path {
    ///     start: Point3::origin(),
    ///     segments: vec![PathSegment::LineTo(Point3::new(3.0, 0.0, 0.0))],
    /// };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// let frame = field.at(1.0).as_frame();
    /// assert!((frame.z().into_inner() - arris_math::Vec3::x()).norm() < 1e-15);
    /// ```
    pub fn as_frame(&self) -> Frame {
        let basis = [
            self.normal.into_inner(),
            self.binormal.into_inner(),
            self.tangent.into_inner(),
        ];
        Frame::from_rotation(self.origin, &UnitQuaternion::from_basis_unchecked(&basis))
    }
}

/// The frames of a validated path: [`Path::frames`] builds it.
///
/// Positions are arc lengths from the path's start, in `[0, length]`;
/// [`FrameField::at`] evaluates by arc length on every kind of segment,
/// a NURBS segment included (ADR-0063 §3), so a fitted face's `v` is
/// the path's arc length whatever the segment kind.
///
/// ```
/// use arris_geom::{FrameLaw, Path, PathSegment};
/// use arris_math::{Meter, Point3, Precision};
///
/// let path = Path {
///     start: Point3::origin(),
///     segments: vec![
///         PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0)),
///         PathSegment::LineTo(Point3::new(0.0, 0.0, 15.0)),
///     ],
/// };
/// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
/// assert_eq!(field.length(), 15.0);
/// assert_eq!(field.span_range(1), (10.0, 15.0));
/// assert!(!field.is_closed());
/// ```
#[derive(Debug, Clone)]
pub struct FrameField {
    law: FrameLaw,
    spans: Vec<Span>,
    length: f64,
    closed: bool,
    holonomy: f64,
    quad: [(f64, f64); GAUSS_ORDER],
}

#[derive(Debug, Clone)]
struct Span {
    shape: Shape,
    /// Arc length at the span's start.
    start: f64,
    len: f64,
    /// The rotation-minimising normal at the start (carried from the
    /// span before).
    u0: Vec3,
}

#[derive(Debug, Clone)]
enum Shape {
    Line {
        origin: Point3,
        dir: Vec3,
    },
    /// Counter-clockwise about `axis`, from `x`.
    Arc {
        center: Point3,
        axis: Vec3,
        x: Vec3,
        radius: f64,
    },
    Helix {
        origin: Point3,
        axis: Vec3,
        x: Vec3,
        radius: f64,
        /// Advance along the axis per radian of turn, signed.
        rise: f64,
        /// +1 or −1: the sense of travel in the turn angle.
        sense: f64,
    },
    Nurbs {
        curve: NurbsCurve,
        table: Vec<Knot>,
    },
}

#[derive(Debug, Clone, Copy)]
struct Knot {
    t: f64,
    s: f64,
    u: Vec3,
}

fn angle_between(a: &Vec3, b: &Vec3) -> f64 {
    a.cross(b).norm().atan2(a.dot(b))
}

fn unit(v: Vec3) -> Vec3 {
    let n = v.norm();
    if n > 0.0 { v / n } else { v }
}

/// `u` made perpendicular to the unit `t` and unit.
fn perpendicular(u: Vec3, t: &Vec3) -> Vec3 {
    unit(u - t * u.dot(t))
}

/// A unit vector perpendicular to the unit `t`, the same for the same
/// `t` on every platform: the coordinate axis least along it, taken off
/// it.
fn any_perpendicular(t: &Vec3) -> Vec3 {
    let axis = if t.x.abs() <= t.y.abs() && t.x.abs() <= t.z.abs() {
        Vec3::x()
    } else if t.y.abs() <= t.z.abs() {
        Vec3::y()
    } else {
        Vec3::z()
    };
    perpendicular(axis, t)
}

/// `v` rotated about the unit `axis` by `angle` (Rodrigues).
fn rotate(v: &Vec3, axis: &Vec3, angle: f64) -> Vec3 {
    let (s, c) = angle.sin_cos();
    v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
}

/// The double reflection step (Wang et al. 2008): the normal `u0` at
/// `(p0, t0)` carried to `(p1, t1)`.
fn reflect_step(p0: &Point3, t0: &Vec3, u0: &Vec3, p1: &Point3, t1: &Vec3) -> Vec3 {
    let v1 = p1 - p0;
    let c1 = v1.dot(&v1);
    if c1 == 0.0 {
        return perpendicular(*u0, t1);
    }
    let u_l = u0 - v1 * (2.0 / c1 * v1.dot(u0));
    let t_l = t0 - v1 * (2.0 / c1 * v1.dot(t0));
    let v2 = t1 - t_l;
    let c2 = v2.dot(&v2);
    let u1 = if c2 == 0.0 {
        u_l
    } else {
        u_l - v2 * (2.0 / c2 * v2.dot(&u_l))
    };
    perpendicular(u1, t1)
}

impl Shape {
    /// The point and unit tangent at arc length `l` from the span's start.
    fn point_tangent(&self, l: f64, quad: &[(f64, f64); GAUSS_ORDER], len: f64) -> (Point3, Vec3) {
        match self {
            Shape::Line { origin, dir } => (origin + dir * l, *dir),
            Shape::Arc {
                center,
                axis,
                x,
                radius,
            } => {
                let y = axis.cross(x);
                let th = l / radius;
                let (s, c) = th.sin_cos();
                (center + (x * c + y * s) * *radius, -x * s + y * c)
            }
            Shape::Helix {
                origin,
                axis,
                x,
                radius,
                rise,
                sense,
            } => {
                let y = axis.cross(x);
                let rho = radius.hypot(*rise);
                let phi = sense * l / rho;
                let (s, c) = phi.sin_cos();
                let p = origin + (x * c + y * s) * *radius + axis * (rise * phi);
                let t = ((-x * s + y * c) * *radius + axis * *rise) * (sense / rho);
                (p, t)
            }
            Shape::Nurbs { curve, table } => {
                // Before the table exists (the start of the segment, for
                // the joint test) the start is the domain's.
                let t = if table.is_empty() {
                    curve.domain().lo()
                } else {
                    nurbs_parameter(curve, table, l, quad, len).0
                };
                let e = curve.eval(t);
                (e.point, unit(e.d1))
            }
        }
    }

    /// The point, tangent and rotation-minimising normal at `l`.
    fn rotation_minimizing(
        &self,
        l: f64,
        u0: &Vec3,
        quad: &[(f64, f64); GAUSS_ORDER],
        len: f64,
    ) -> (Point3, Vec3, Vec3) {
        match self {
            Shape::Line { .. } => {
                let (p, t) = self.point_tangent(l, quad, len);
                (p, t, *u0)
            }
            Shape::Arc { axis, radius, .. } => {
                let (p, t) = self.point_tangent(l, quad, len);
                (p, t, unit(rotate(u0, axis, l / radius)))
            }
            Shape::Helix {
                axis,
                x,
                radius,
                rise,
                sense,
                ..
            } => {
                let (p, t) = self.point_tangent(l, quad, len);
                let rho = radius.hypot(*rise);
                let tau = rise / (rho * rho);
                let y = axis.cross(x);
                let frenet_n = |phi: f64| {
                    let (s, c) = phi.sin_cos();
                    -(x * c + y * s)
                };
                let start_t = {
                    let (_, t0) = self.point_tangent(0.0, quad, len);
                    t0
                };
                let n0 = frenet_n(0.0);
                let b0 = start_t.cross(&n0);
                let psi0 = u0.dot(&b0).atan2(u0.dot(&n0));
                let psi = psi0 - tau * l;
                let n = frenet_n(sense * l / rho);
                let b = t.cross(&n);
                let (s, c) = psi.sin_cos();
                (p, t, unit(n * c + b * s))
            }
            Shape::Nurbs { curve, table } => {
                let (t, j) = nurbs_parameter(curve, table, l, quad, len);
                let e = curve.eval(t);
                let tangent = unit(e.d1);
                let k = table[j];
                if t == k.t {
                    return (e.point, tangent, k.u);
                }
                let ek = curve.eval(k.t);
                let u = reflect_step(&ek.point, &unit(ek.d1), &k.u, &e.point, &tangent);
                (e.point, tangent, u)
            }
        }
    }
}

/// `∫ |C'|` over `[a, b]` by one Gauss–Legendre interval.
fn speed_integral(curve: &NurbsCurve, a: f64, b: f64, quad: &[(f64, f64); GAUSS_ORDER]) -> f64 {
    let half = (b - a) / 2.0;
    let mid = (a + b) / 2.0;
    quad.iter()
        .map(|&(x, w)| w * curve.eval(mid + half * x).d1.norm())
        .sum::<f64>()
        * half
}

/// The curve parameter at arc length `l` from the span's start, and the
/// index of the table entry it follows. Newton on the arc length from
/// that entry, bracketed by its neighbour.
fn nurbs_parameter(
    curve: &NurbsCurve,
    table: &[Knot],
    l: f64,
    quad: &[(f64, f64); GAUSS_ORDER],
    len: f64,
) -> (f64, usize) {
    let l = l.clamp(0.0, len);
    let last = table.len() - 1;
    let j = table
        .partition_point(|k| k.s <= l)
        .saturating_sub(1)
        .min(last - 1);
    let (lo, hi) = (table[j], table[j + 1]);
    if l <= lo.s {
        return (lo.t, j);
    }
    if l >= hi.s {
        return (hi.t, j);
    }
    let want = l - lo.s;
    let (mut a, mut b) = (lo.t, hi.t);
    let mut t = lo.t + (hi.t - lo.t) * (want / (hi.s - lo.s));
    for _ in 0..64 {
        let f = speed_integral(curve, lo.t, t, quad) - want;
        if f.abs() <= f64::EPSILON * len.max(1.0) {
            break;
        }
        if f > 0.0 {
            b = t;
        } else {
            a = t;
        }
        let speed = curve.eval(t).d1.norm();
        let next = t - f / speed;
        t = if next > a && next < b {
            next
        } else {
            0.5 * (a + b)
        };
        if b - a <= f64::EPSILON * hi.t.abs().max(1.0) {
            break;
        }
    }
    (t, j)
}

impl Path {
    /// Validates the path and builds its [`FrameField`] under `law`.
    ///
    /// The path must be G1 within `tol.angular` at every joint, a NURBS
    /// segment must start within `tol.linear` of where the path has
    /// reached, and no segment may be shorter than `tol.linear`. A path
    /// that ends within `tol.linear` of its start is closed, and must be
    /// G1 at the closing joint as well. Errors name the segment:
    /// [`PathError::Gap`], [`PathError::Corner`], [`PathError::Degenerate`],
    /// [`PathError::BinormalAlongTangent`] and, from a cusp in a NURBS
    /// segment, [`PathError::FrameStep`]. `meter` ticks once per step of
    /// a NURBS segment's frame table.
    ///
    /// The frame error of a NURBS segment under the rotation-minimising
    /// law is held under `tol.angular` (floored at [`FRAME_ERROR_FLOOR`])
    /// by step doubling on the double reflection, which is of fourth
    /// order.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// // A quarter arc of radius 10 in the xz plane, then a line on.
    /// let path = Path {
    ///     start: Point3::origin(),
    ///     segments: vec![
    ///         PathSegment::ArcTo {
    ///             via: Point3::new(10.0 - 10.0 * 0.5f64.sqrt(), 0.0, 10.0 * 0.5f64.sqrt()),
    ///             to: Point3::new(10.0, 0.0, 10.0),
    ///         },
    ///         PathSegment::LineTo(Point3::new(20.0, 0.0, 10.0)),
    ///     ],
    /// };
    /// let tol = Precision::DEFAULT.tolerance();
    /// let field = path.frames(FrameLaw::default(), tol, &mut Meter::default()).unwrap();
    /// assert!((field.length() - (5.0 * std::f64::consts::PI + 10.0)).abs() < 1e-12);
    ///
    /// // A corner is refused, naming the segment.
    /// let bent = Path {
    ///     start: Point3::origin(),
    ///     segments: vec![
    ///         PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0)),
    ///         PathSegment::LineTo(Point3::new(10.0, 0.0, 10.0)),
    ///     ],
    /// };
    /// let err = bent.frames(FrameLaw::default(), tol, &mut Meter::default()).unwrap_err();
    /// assert!(matches!(err, arris_geom::PathError::Corner { segment: 1, .. }));
    /// ```
    pub fn frames(
        &self,
        law: FrameLaw,
        tol: Tolerance,
        meter: &mut Meter,
    ) -> Result<FrameField, PathError> {
        if self.segments.is_empty() {
            return Err(PathError::Empty);
        }
        if !self.start.coords.iter().all(|c| c.is_finite()) {
            return Err(PathError::NotFinite { segment: 0 });
        }
        let quad = gauss_legendre();
        let step_tol = tol.angular.max(FRAME_ERROR_FLOOR);
        let mut spans: Vec<Span> = Vec::with_capacity(self.segments.len());
        let mut here = self.start;
        let mut at = 0.0_f64;
        let mut carried: Option<(Vec3, Vec3)> = None; // end tangent, end normal
        for (k, segment) in self.segments.iter().enumerate() {
            let (shape, len, start_gap) = build_shape(segment, k, here, tol)?;
            if let Some(distance) = start_gap {
                return Err(PathError::Gap {
                    segment: k,
                    distance,
                });
            }
            // The shape's start tangent, and a fixed-binormal screen.
            let (_, t_start) = shape.point_tangent(0.0, &quad, len);
            let u_in = match carried {
                Some((t_end, u_end)) => {
                    let angle = angle_between(&t_end, &t_start);
                    if angle > tol.angular {
                        return Err(PathError::Corner {
                            segment: k,
                            at,
                            angle,
                        });
                    }
                    perpendicular(u_end, &t_start)
                }
                None => any_perpendicular(&t_start),
            };
            let mut shape = shape;
            let u_end = if let Shape::Nurbs { curve, table } = &mut shape {
                let built = build_table(curve, k, &u_in, law, step_tol, tol, &quad, meter, at)?;
                len_check(built.len, tol, k)?;
                *table = built.table;
                // The table's total length is the span's.
                built.u_end
            } else {
                let (_, _, u) = shape.rotation_minimizing(len, &u_in, &quad, len);
                u
            };
            let len = match &shape {
                Shape::Nurbs { table, .. } => table[table.len() - 1].s,
                _ => len,
            };
            if let FrameLaw::FixedBinormal(b) = law {
                check_binormal(&shape, len, &b, k, at, tol, &quad)?;
            }
            let (end_point, end_t) = shape.point_tangent(len, &quad, len);
            spans.push(Span {
                shape,
                start: at,
                len,
                u0: u_in,
            });
            at += len;
            here = end_point;
            carried = Some((end_t, u_end));
        }
        let closed = (here - self.start).norm() <= tol.linear;
        let mut holonomy = 0.0;
        if closed {
            let first = &spans[0];
            let (_, t0) = first.shape.point_tangent(0.0, &quad, first.len);
            if let Some((t_end, u_end)) = carried {
                let angle = angle_between(&t_end, &t0);
                if angle > tol.angular {
                    return Err(PathError::Corner {
                        segment: 0,
                        at,
                        angle,
                    });
                }
                if matches!(law, FrameLaw::RotationMinimizing) {
                    let u_end = perpendicular(u_end, &t0);
                    let u_start = first.u0;
                    let side = t0.cross(&u_start);
                    holonomy = u_end.dot(&side).atan2(u_end.dot(&u_start));
                }
            }
        }
        Ok(FrameField {
            law,
            spans,
            length: at,
            closed,
            holonomy,
            quad,
        })
    }
}

fn len_check(len: f64, tol: Tolerance, segment: usize) -> Result<(), PathError> {
    if len > tol.linear {
        Ok(())
    } else {
        Err(PathError::Degenerate {
            segment,
            reason: PathDegeneracy::Short,
        })
    }
}

/// The geometry of one segment from `here`, its length (a NURBS
/// segment's is set by its table) and the distance it starts from `here`
/// if over the tolerance.
fn build_shape(
    segment: &PathSegment,
    k: usize,
    here: Point3,
    tol: Tolerance,
) -> Result<(Shape, f64, Option<f64>), PathError> {
    let degenerate = |reason| PathError::Degenerate { segment: k, reason };
    match segment {
        PathSegment::LineTo(to) => {
            if !to.coords.iter().all(|c| c.is_finite()) {
                return Err(PathError::NotFinite { segment: k });
            }
            let d = to - here;
            let len = d.norm();
            if len <= tol.linear {
                return Err(degenerate(PathDegeneracy::Short));
            }
            Ok((
                Shape::Line {
                    origin: here,
                    dir: d / len,
                },
                len,
                None,
            ))
        }
        PathSegment::ArcTo { via, to } => {
            if !(via.coords.iter().all(|c| c.is_finite())
                && to.coords.iter().all(|c| c.is_finite()))
            {
                return Err(PathError::NotFinite { segment: k });
            }
            let ab = via - here;
            let ac = to - here;
            if ab.norm() <= tol.linear || ac.norm() <= tol.linear || (to - via).norm() <= tol.linear
            {
                return Err(degenerate(PathDegeneracy::Short));
            }
            let n = ab.cross(&ac);
            let n2 = n.norm_squared();
            // The via point's distance from the chord's line.
            if n.norm() / ac.norm() <= tol.linear {
                return Err(degenerate(PathDegeneracy::CollinearArc));
            }
            let center_rel =
                (n.cross(&ab) * ac.norm_squared() + ac.cross(&n) * ab.norm_squared()) / (2.0 * n2);
            let radius = center_rel.norm();
            let center = here + center_rel;
            let axis = n / n2.sqrt();
            let x = (here - center) / radius;
            let y = axis.cross(&x);
            let to_c = to - center;
            let tau = std::f64::consts::TAU;
            let angle = to_c.dot(&y).atan2(to_c.dot(&x)).rem_euclid(tau);
            Ok((
                Shape::Arc {
                    center,
                    axis,
                    x,
                    radius,
                },
                radius * angle,
                None,
            ))
        }
        PathSegment::Helix { axis, pitch, angle } => {
            if !(pitch.is_finite() && angle.is_finite()) {
                return Err(PathError::NotFinite { segment: k });
            }
            if *angle == 0.0 {
                return Err(degenerate(PathDegeneracy::NoTurn));
            }
            let dir = axis.direction.into_inner();
            let rel = here - axis.origin;
            let along = rel.dot(&dir);
            let off = rel - dir * along;
            let radius = off.norm();
            if radius <= tol.linear {
                return Err(degenerate(PathDegeneracy::StartOnAxis));
            }
            let rise = pitch / std::f64::consts::TAU;
            let rho = radius.hypot(rise);
            let len = angle.abs() * rho;
            if len <= tol.linear {
                return Err(degenerate(PathDegeneracy::Short));
            }
            Ok((
                Shape::Helix {
                    origin: here - off,
                    axis: dir,
                    x: off / radius,
                    radius,
                    rise,
                    sense: angle.signum(),
                },
                len,
                None,
            ))
        }
        PathSegment::Nurbs(curve) => {
            let domain = curve.domain();
            let start = curve.eval(domain.lo()).point;
            if !start.coords.iter().all(|c| c.is_finite()) {
                return Err(PathError::NotFinite { segment: k });
            }
            let distance = (start - here).norm();
            let gap = (distance > tol.linear).then_some(distance);
            Ok((
                Shape::Nurbs {
                    curve: curve.clone(),
                    table: Vec::new(),
                },
                0.0,
                gap,
            ))
        }
    }
}

struct Built {
    table: Vec<Knot>,
    len: f64,
    u_end: Vec3,
}

/// The frame table of a NURBS segment: entries at accepted steps of the
/// double reflection, each carrying the arc length and the normal.
#[allow(clippy::too_many_arguments)]
fn build_table(
    curve: &NurbsCurve,
    segment: usize,
    u_in: &Vec3,
    law: FrameLaw,
    step_tol: f64,
    tol: Tolerance,
    quad: &[(f64, f64); GAUSS_ORDER],
    meter: &mut Meter,
    base: f64,
) -> Result<Built, PathError> {
    let domain = curve.domain();
    let (t0, t1) = (domain.lo(), domain.hi());
    // The breaks: the domain's ends and the distinct interior knots.
    let mut breaks = vec![t0];
    breaks.extend(curve.breaks_within(domain));
    breaks.push(t1);
    breaks.dedup();
    let rm = matches!(law, FrameLaw::RotationMinimizing);
    let tangent_at = |t: f64| -> Result<(Point3, Vec3), PathError> {
        let e = curve.eval(t);
        let speed = e.d1.norm();
        if !speed.is_finite() || speed <= 0.0 || !e.point.coords.iter().all(|c| c.is_finite()) {
            return Err(PathError::Degenerate {
                segment,
                reason: PathDegeneracy::ZeroSpeed,
            });
        }
        Ok((e.point, e.d1 / speed))
    };
    let (mut p, mut tan) = tangent_at(t0)?;
    let mut u = perpendicular(*u_in, &tan);
    let mut s = 0.0_f64;
    let mut table = vec![Knot {
        t: t0,
        s,
        u: if rm { u } else { fixed_normal(law, &tan, &u) },
    }];
    let degree = curve.degree();
    let knots = curve.knots();
    // A C0 joint inside the curve is a corner unless its tangents agree.
    // Tested before the stepping, which evaluates the tangent at a joint
    // from the right and so must be able to rely on G1.
    let mut reached = base;
    for i in 1..breaks.len() - 1 {
        reached += speed_integral(curve, breaks[i - 1], breaks[i], quad);
        let a = breaks[i];
        if knots.iter().filter(|&&q| q == a).count() < degree {
            continue;
        }
        let pieces = Interval::new(breaks[i - 1], a)
            .ok()
            .and_then(|iv| curve.segment(iv).ok())
            .zip(
                Interval::new(a, breaks[i + 1])
                    .ok()
                    .and_then(|iv| curve.segment(iv).ok()),
            );
        if let Some((left, right)) = pieces {
            let angle = angle_between(&unit(left.eval(a).d1), &unit(right.eval(a).d1));
            if angle > tol.angular {
                return Err(PathError::Corner {
                    segment,
                    at: reached,
                    angle,
                });
            }
        }
    }
    for i in 0..breaks.len() - 1 {
        let (a, b) = (breaks[i], breaks[i + 1]);
        let min_h = FRAME_MIN_STEP * (b - a);
        let mut h = (b - a) / 8.0;
        let mut t = a;
        while t < b {
            let mut step = h.min(b - t);
            if b - (t + step) < 0.5 * step {
                step = b - t;
            }
            loop {
                meter.tick()?;
                let end = if t + step >= b { b } else { t + step };
                let (p1, t1v) = tangent_at(end)?;
                let turn = angle_between(&tan, &t1v);
                let (u1, err) = if rm {
                    let big = reflect_step(&p, &tan, &u, &p1, &t1v);
                    let mid = 0.5 * (t + end);
                    let (pm, tm) = tangent_at(mid)?;
                    let half = reflect_step(&p, &tan, &u, &pm, &tm);
                    let fine = reflect_step(&pm, &tm, &half, &p1, &t1v);
                    (fine, angle_between(&big, &fine))
                } else {
                    (u, 0.0)
                };
                if turn <= FRAME_MAX_TURN && err <= step_tol {
                    s += speed_integral(curve, t, end, quad);
                    p = p1;
                    tan = t1v;
                    u = u1;
                    table.push(Knot {
                        t: end,
                        s,
                        u: if rm { u } else { fixed_normal(law, &tan, &u) },
                    });
                    t = end;
                    // Fourth-order step control, grown at most twofold.
                    let grow = if err > 0.0 {
                        (0.9 * (step_tol / err).powf(0.2)).clamp(0.5, 2.0)
                    } else {
                        2.0
                    };
                    let by_turn = if turn > 0.0 {
                        (FRAME_MAX_TURN / turn).clamp(0.5, 2.0)
                    } else {
                        2.0
                    };
                    h = step * grow.min(by_turn);
                    break;
                }
                step *= 0.5;
                if step < min_h {
                    return Err(PathError::FrameStep { segment });
                }
            }
        }
    }
    Ok(Built {
        len: s,
        u_end: u,
        table,
    })
}

/// The fixed-binormal normal at a tangent, for a table entry; the
/// carried normal where the binormal is not usable yet (the check that
/// refuses that comes after).
fn fixed_normal(law: FrameLaw, t: &Vec3, fallback: &Vec3) -> Vec3 {
    match law {
        FrameLaw::FixedBinormal(b) => fixed_binormal_frame(t, &b.into_inner())
            .map(|(u, _)| u)
            .unwrap_or(*fallback),
        FrameLaw::RotationMinimizing => *fallback,
    }
}

/// `(normal, binormal)` of the fixed-binormal frame at the unit tangent
/// `t`, or `None` when the binormal runs along it exactly.
fn fixed_binormal_frame(t: &Vec3, b: &Vec3) -> Option<(Vec3, Vec3)> {
    let perp = b - t * b.dot(t);
    let n = perp.norm();
    if n > 0.0 {
        let binormal = perp / n;
        Some((binormal.cross(t), binormal))
    } else {
        None
    }
}

/// Refuses a span where the fixed binormal comes within the angular
/// tolerance of the tangent. Exact on a line, an arc and a helix, where
/// `b · T` is a sinusoid of the turn angle plus a constant, so its
/// extremes are at the ends and at the sinusoid's own; on a NURBS segment
/// at every table entry, which are at most [`FRAME_MAX_TURN`] apart.
fn check_binormal(
    shape: &Shape,
    len: f64,
    b: &UnitVec3,
    segment: usize,
    base: f64,
    tol: Tolerance,
    quad: &[(f64, f64); GAUSS_ORDER],
) -> Result<(), PathError> {
    let b = b.into_inner();
    let along = |l: f64| -> f64 {
        let (_, t) = shape.point_tangent(l, quad, len);
        b.cross(&t).norm()
    };
    let mut candidates: Vec<f64> = vec![0.0, len];
    match shape {
        Shape::Line { .. } => {}
        Shape::Arc {
            axis, x, radius, ..
        } => {
            let y = axis.cross(x);
            // b·T = −bx sin θ + by cos θ, extreme where its slope vanishes.
            let (bx, by) = (b.dot(x), b.dot(&y));
            let th1 = bx.atan2(by); // by cos θ − bx sin θ = R cos(θ + th1)
            sinusoid_extremes(-th1, 0.0, len / radius, &mut |th| {
                candidates.push(th * radius)
            });
        }
        Shape::Helix {
            axis,
            x,
            radius,
            rise,
            sense,
            ..
        } => {
            let y = axis.cross(x);
            let (bx, by) = (b.dot(x), b.dot(&y));
            let th1 = bx.atan2(by);
            let rho = radius.hypot(*rise);
            let phi_end = sense * len / rho;
            sinusoid_extremes(-th1, phi_end.min(0.0), phi_end.max(0.0), &mut |phi| {
                candidates.push((phi * sense) * rho)
            });
        }
        Shape::Nurbs { table, .. } => candidates.extend(table.iter().map(|k| k.s)),
    }
    let mut worst: Option<(f64, f64)> = None;
    for l in candidates {
        let l = l.clamp(0.0, len);
        let c = along(l);
        if worst.is_none_or(|(w, _)| c < w) {
            worst = Some((c, l));
        }
    }
    match worst {
        Some((c, l)) if c.min(1.0).asin() <= tol.angular => Err(PathError::BinormalAlongTangent {
            segment,
            at: base + l,
        }),
        _ => Ok(()),
    }
}

/// Calls `visit` at every extreme `θ1 + kπ` of `cos(θ − θ1)` inside
/// `[lo, hi]`.
fn sinusoid_extremes(theta1: f64, lo: f64, hi: f64, visit: &mut impl FnMut(f64)) {
    let pi = std::f64::consts::PI;
    let mut k = ((lo - theta1) / pi).ceil();
    loop {
        let th = theta1 + k * pi;
        if th > hi {
            break;
        }
        visit(th);
        k += 1.0;
    }
}

impl FrameField {
    /// The path's length.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path { start: Point3::origin(), segments: vec![PathSegment::LineTo(Point3::new(3.0, 4.0, 0.0))] };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// assert_eq!(field.length(), 5.0);
    /// ```
    pub fn length(&self) -> f64 {
        self.length
    }

    /// The frame law the field was built under.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path { start: Point3::origin(), segments: vec![PathSegment::LineTo(Point3::new(1.0, 0.0, 0.0))] };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// assert_eq!(field.law(), FrameLaw::RotationMinimizing);
    /// ```
    pub fn law(&self) -> FrameLaw {
        self.law
    }

    /// The number of spans: one per segment of the path.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path { start: Point3::origin(), segments: vec![PathSegment::LineTo(Point3::new(1.0, 0.0, 0.0))] };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// assert_eq!(field.span_count(), 1);
    /// ```
    pub fn span_count(&self) -> usize {
        self.spans.len()
    }

    /// The arc lengths where span `k` starts and ends. Panics if `k` is
    /// not a span of the field (a caller's index, not geometry).
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path {
    ///     start: Point3::origin(),
    ///     segments: vec![
    ///         PathSegment::LineTo(Point3::new(0.0, 0.0, 2.0)),
    ///         PathSegment::LineTo(Point3::new(0.0, 0.0, 7.0)),
    ///     ],
    /// };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// assert_eq!(field.span_range(0), (0.0, 2.0));
    /// assert_eq!(field.span_range(1), (2.0, 7.0));
    /// ```
    pub fn span_range(&self, k: usize) -> (f64, f64) {
        let s = &self.spans[k];
        (s.start, s.start + s.len)
    }

    /// `true` when the path ends where it starts: the frame closes up,
    /// and a body swept along it has no caps.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path { start: Point3::origin(), segments: vec![PathSegment::LineTo(Point3::new(1.0, 0.0, 0.0))] };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// assert!(!field.is_closed());
    /// ```
    pub fn is_closed(&self) -> bool {
        self.closed
    }

    /// For a closed path under the rotation-minimising law, the angle
    /// the carried frame has turned about the tangent when it returns to
    /// the start, in `(−π, π]` and right-handed about the tangent. It is
    /// the solid angle the tangent indicatrix encloses, modulo 2π; zero
    /// for a planar path. Zero for an open path or a fixed binormal.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let r = 10.0;
    /// let h = r * 0.5f64.sqrt();
    /// let ring = Path {
    ///     start: Point3::new(r, 0.0, 0.0),
    ///     segments: vec![
    ///         PathSegment::ArcTo { via: Point3::new(h, h, 0.0), to: Point3::new(0.0, r, 0.0) },
    ///         PathSegment::ArcTo { via: Point3::new(-h, h, 0.0), to: Point3::new(-r, 0.0, 0.0) },
    ///         PathSegment::ArcTo { via: Point3::new(-h, -h, 0.0), to: Point3::new(0.0, -r, 0.0) },
    ///         PathSegment::ArcTo { via: Point3::new(h, -h, 0.0), to: Point3::new(r, 0.0, 0.0) },
    ///     ],
    /// };
    /// let field = ring.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// assert!(field.is_closed());
    /// assert!(field.holonomy().abs() < 1e-12);
    /// ```
    pub fn holonomy(&self) -> f64 {
        self.holonomy
    }

    /// The frame at arc length `s`, clamped into `[0, length]`.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path { start: Point3::origin(), segments: vec![PathSegment::LineTo(Point3::new(0.0, 0.0, 6.0))] };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// // Past the end the frame is the last one.
    /// assert_eq!(field.at(99.0).origin, Point3::new(0.0, 0.0, 6.0));
    /// ```
    pub fn at(&self, s: f64) -> PathFrame {
        let s = s.clamp(0.0, self.length);
        let k = self
            .spans
            .partition_point(|sp| sp.start <= s)
            .saturating_sub(1)
            .min(self.spans.len() - 1);
        let span = &self.spans[k];
        let l = (s - span.start).clamp(0.0, span.len);
        let (p, t, u) = match self.law {
            FrameLaw::RotationMinimizing => span
                .shape
                .rotation_minimizing(l, &span.u0, &self.quad, span.len),
            FrameLaw::FixedBinormal(b) => {
                let (p, t) = span.shape.point_tangent(l, &self.quad, span.len);
                let u = fixed_binormal_frame(&t, &b.into_inner())
                    .map(|(u, _)| u)
                    .unwrap_or_else(|| any_perpendicular(&t));
                (p, t, u)
            }
        };
        PathFrame {
            origin: p,
            tangent: UnitVec3::new_unchecked(t),
            normal: UnitVec3::new_unchecked(u),
            binormal: UnitVec3::new_unchecked(t.cross(&u)),
        }
    }

    /// The rigid motion that carries a profile placed on the first frame
    /// to the frame at `s`: first frame to origin, then origin to the
    /// frame at `s`. A line's is a translation and an arc's a rotation
    /// about its axis, whatever the first normal was.
    ///
    /// ```
    /// use arris_geom::{FrameLaw, Path, PathSegment};
    /// use arris_math::{Meter, Point3, Precision};
    ///
    /// let path = Path { start: Point3::origin(), segments: vec![PathSegment::LineTo(Point3::new(0.0, 0.0, 8.0))] };
    /// let field = path.frames(FrameLaw::default(), Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
    /// let m = field.motion_at(3.0);
    /// assert!((m.apply(Point3::new(1.0, 2.0, 0.0)) - Point3::new(1.0, 2.0, 3.0)).norm() < 1e-15);
    /// ```
    pub fn motion_at(&self, s: f64) -> Isometry {
        let first = self.at(0.0).as_frame().as_isometry();
        first.inverse().then(&self.at(s).as_frame().as_isometry())
    }
}

#[cfg(test)]
mod tests;
