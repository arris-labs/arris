//! Coaxial surfaces of revolution meet through their meridians
//! (ADR-0008): one arm for every pair with a cone, a sphere or a torus in
//! it, in the positions where the two share an axis.
//!
//! In the half-plane through the shared axis, with `ρ` signed across the
//! axis and `z` along it, each surface is a set of lines and circles —
//! its *meridian sections* — symmetric under `ρ ↦ −ρ`: a plane
//! perpendicular to the axis is the line `z = h`, a cylinder the lines
//! `ρ = ±R`, a cone the two lines through its apex at its half-angle, a
//! sphere a circle centred on the axis, a torus the two circles at
//! `ρ = ±R`. Two surfaces meet where their sections meet, by the closed
//! forms of two lines, a line and a circle or two circles; each 2D
//! meeting off the axis sweeps a circle about it, and one on the axis is
//! a point. The mirror pairs of sections meet in the mirror points, bit
//! for bit, so only the half-plane `ρ ≥ 0` is read. `tol.linear` carries
//! over unchanged: a distance in the plane through the axis is the 3D
//! distance between points at one angle, and a point's distance to a
//! surface of revolution is its distance to the full symmetric section.
//!
//! Read in the reference tree: Open CASCADE's `IntAna_QuadQuadGeo`, whose
//! coaxial branches (`Perform` on the cylinder–cone, cylinder–sphere,
//! cone–cone, sphere–cone, sphere–sphere, plane–torus, cylinder–torus,
//! cone–torus and sphere–torus pairs) are one closed form per pair; here
//! the sections replace the table.

use core::f64::consts::FRAC_PI_2;

use arris_math::{Aabb, Frame, Meter, Point3, Tolerance, UnitVec3, Vec3};

use crate::intersect::line_angle;
use crate::{
    Curve, GeomError, GeomKind, MeetCurve, MeetKind, MeetPoint, Surface, SurfaceIntersection,
    SurfaceKind,
};

/// The intersection of two surfaces at least one of which is a cone, a
/// sphere or a torus, when they share an axis: circles about the axis
/// ascending along it and points on the axis ascending along it, each a
/// crossing or a touch and together in one `Meets` when both occur
/// (ADR-0018), `Coincident` or `Empty`; and a plane through a cone's or a
/// torus's axis, which cuts the meridian itself — two crossing rulings
/// or tube circles ([`through_the_axis`]). A pair that shares no axis is
/// in general position, decided by [`crate::intersect::off_axis`] inside
/// `within`; a coincident section beside a meeting off it is
/// `Unsupported` naming the kinds.
pub(crate) fn intersect_coaxial(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SurfaceIntersection, GeomError> {
    let unsupported = || GeomError::Unsupported {
        a: GeomKind::Surface(a.kind()),
        b: GeomKind::Surface(b.kind()),
    };
    let axis = match shared_axis(a, b, tol)? {
        Shared::Axis(frame) => frame,
        Shared::Concentric { same } => {
            return Ok(if same {
                SurfaceIntersection::Coincident
            } else {
                SurfaceIntersection::Empty
            });
        }
        Shared::InPlane { normal } => {
            let carrier = if a.kind() == SurfaceKind::Plane { b } else { a };
            return through_the_axis(carrier, normal).ok_or_else(unsupported);
        }
        Shared::None => return crate::intersect::off_axis(a, b, within, tol, meter),
    };
    let (sa, sb) = (sections(a, &axis), sections(b, &axis));
    let mut coincident: Vec<Section> = Vec::new();
    let mut meetings: Vec<(Kind, [f64; 2])> = Vec::new();
    for x in &sa {
        for y in &sb {
            match meet(*x, *y, tol) {
                Meeting::Coincident => coincident.push(*x),
                Meeting::Points(kind, points) => {
                    meetings.extend(points.into_iter().map(|p| (kind, p)));
                }
            }
        }
    }
    if !coincident.is_empty() {
        // Every other meeting must lie on a section the two share, as the
        // apex of two equal coaxial cones does; anything else mixes kinds.
        let on_shared = |p: &[f64; 2]| coincident.iter().any(|s| s.distance(*p) <= tol.linear);
        return if meetings.iter().all(|(_, p)| on_shared(p)) {
            Ok(SurfaceIntersection::Coincident)
        } else {
            Err(unsupported())
        };
    }
    // The half-plane `ρ ≥ 0`: a meeting within `tol.linear` of the axis is
    // a point on it, one beyond is a circle of radius `ρ`, and one on the
    // far side is the mirror of a meeting already kept.
    let mut points: Vec<(MeetKind, f64)> = Vec::new();
    let mut circles: Vec<(MeetKind, [f64; 2])> = Vec::new();
    for (kind, [rho, z]) in meetings {
        let kind = match kind {
            Kind::Cross => MeetKind::Crossing,
            Kind::Touch => MeetKind::Touch,
        };
        if rho.abs() <= tol.linear {
            // A point two sections cross at is a crossing, whatever another
            // pair of sections does there (`Crossing` orders first).
            match points.iter_mut().find(|(_, q)| (q - z).abs() <= tol.linear) {
                Some(point) => point.0 = point.0.min(kind),
                None => points.push((kind, z)),
            }
        } else if rho > 0.0
            && !circles
                .iter()
                .any(|(_, q)| (q[0] - rho).abs() <= tol.linear && (q[1] - z).abs() <= tol.linear)
        {
            circles.push((kind, [rho, z]));
        }
    }
    if circles.is_empty() && points.is_empty() {
        return Ok(SurfaceIntersection::Empty);
    }
    let z: Vec3 = axis.z().into_inner();
    let at = |height: f64| axis.origin() + height * z;
    points.sort_by(|p, q| p.1.total_cmp(&q.1));
    circles.sort_by(|p, q| p.1[1].total_cmp(&q.1[1]).then(p.1[0].total_cmp(&q.1[0])));
    Ok(SurfaceIntersection::Meets {
        curves: circles
            .into_iter()
            .map(|(kind, [rho, height])| MeetCurve {
                curve: Curve::Circle {
                    frame: axis.with_origin(at(height)),
                    radius: rho,
                },
                kind,
            })
            .collect(),
        points: points
            .into_iter()
            .map(|(kind, height)| MeetPoint {
                point: at(height),
                kind,
            })
            .collect(),
    })
}

/// What two operands share.
enum Shared {
    /// An axis: the frame every circle takes, its origin on the axis and
    /// its `Z` along it.
    Axis(Frame),
    /// Two spheres about one centre, the same surface or not.
    Concentric { same: bool },
    /// A plane the carrier's axis lies in, with the plane's normal: the
    /// plane cuts the carrier in its meridian, not in a parallel.
    InPlane { normal: UnitVec3 },
    /// No axis: a general position.
    None,
}

/// The shared axis of the pair, as the frame the circles take: coaxial
/// means the axes parallel within `tol.angular` and the second operand's
/// origin within `tol.linear` of the first's axis; a sphere is coaxial
/// with anything whose axis passes within `tol.linear` of its centre; a
/// plane is coaxial with an axis its normal is parallel to, and holds an
/// axis it is parallel to within `tol.angular` and passes within
/// `tol.linear` of, whose carrier it then cuts through. The frame is
/// the first operand's whose frame carries the axis — a cylinder, a cone
/// or a torus always, a sphere when its own `Z` is along the axis —
/// else, for a plane against a sphere, the plane's frame moved to the
/// sphere's centre, and for two spheres the line from the first centre to
/// the second with `X` the first sphere's `X` or `Y`, whichever has the
/// larger component across the line (`docs/DATA-MODEL.md` §Curves). A
/// pair this arm is never asked about — two planes, a plane and a
/// cylinder, two cylinders, anything with a NURBS — shares nothing here.
fn shared_axis(a: &Surface, b: &Surface, tol: Tolerance) -> Result<Shared, GeomError> {
    let on_axis = |frame: &Frame, p: Point3| {
        let d = p - frame.origin();
        (d - d.dot(&frame.z()) * frame.z().into_inner()).norm() <= tol.linear
    };
    let parallel = |x: &UnitVec3, y: &UnitVec3| line_angle(x, y) <= tol.angular;
    // A sphere against a carrier of the axis: the carrier's frame, or the
    // sphere's own when it comes first and its `Z` is along the axis.
    let sphere_and_carrier = |sphere: &Frame, carrier: &Frame, sphere_first: bool| {
        if !on_axis(carrier, sphere.origin()) {
            Shared::None
        } else if sphere_first && parallel(&sphere.z(), &carrier.z()) {
            Shared::Axis(*sphere)
        } else {
            Shared::Axis(*carrier)
        }
    };
    Ok(match (a, b) {
        (
            Surface::Cylinder { frame: fa, .. }
            | Surface::Cone { frame: fa, .. }
            | Surface::Torus { frame: fa, .. },
            Surface::Cylinder { frame: fb, .. }
            | Surface::Cone { frame: fb, .. }
            | Surface::Torus { frame: fb, .. },
        ) => {
            if parallel(&fa.z(), &fb.z()) && on_axis(fa, fb.origin()) {
                Shared::Axis(*fa)
            } else {
                Shared::None
            }
        }
        (
            Surface::Cylinder { frame: carrier, .. }
            | Surface::Cone { frame: carrier, .. }
            | Surface::Torus { frame: carrier, .. },
            Surface::Plane { frame: plane },
        )
        | (
            Surface::Plane { frame: plane },
            Surface::Cylinder { frame: carrier, .. }
            | Surface::Cone { frame: carrier, .. }
            | Surface::Torus { frame: carrier, .. },
        ) => {
            let across = FRAC_PI_2 - line_angle(&plane.z(), &carrier.z()) <= tol.angular;
            if parallel(&plane.z(), &carrier.z()) {
                Shared::Axis(*carrier)
            } else if across
                && plane.z().dot(&(carrier.origin() - plane.origin())).abs() <= tol.linear
            {
                Shared::InPlane { normal: plane.z() }
            } else {
                Shared::None
            }
        }
        (
            Surface::Cylinder { frame: carrier, .. }
            | Surface::Cone { frame: carrier, .. }
            | Surface::Torus { frame: carrier, .. },
            Surface::Sphere { frame: sphere, .. },
        ) => sphere_and_carrier(sphere, carrier, false),
        (
            Surface::Sphere { frame: sphere, .. },
            Surface::Cylinder { frame: carrier, .. }
            | Surface::Cone { frame: carrier, .. }
            | Surface::Torus { frame: carrier, .. },
        ) => sphere_and_carrier(sphere, carrier, true),
        (Surface::Plane { frame: plane }, Surface::Sphere { frame: sphere, .. })
        | (Surface::Sphere { frame: sphere, .. }, Surface::Plane { frame: plane }) => {
            Shared::Axis(if parallel(&sphere.z(), &plane.z()) {
                *sphere
            } else {
                plane.with_origin(sphere.origin())
            })
        }
        (
            Surface::Sphere {
                frame: fa,
                radius: ra,
            },
            Surface::Sphere {
                frame: fb,
                radius: rb,
            },
        ) => {
            let d = fb.origin() - fa.origin();
            let Some(z) = UnitVec3::try_new(d, tol.linear) else {
                return Ok(Shared::Concentric {
                    same: (ra - rb).abs() <= tol.linear,
                });
            };
            if parallel(&fa.z(), &z) {
                return Ok(Shared::Axis(*fa));
            }
            if parallel(&fb.z(), &z) {
                return Ok(Shared::Axis(*fb));
            }
            let across = |v: &UnitVec3| (v.into_inner() - v.dot(&z) * z.into_inner()).norm();
            let hint = if across(&fa.x()) >= across(&fa.y()) {
                fa.x()
            } else {
                fa.y()
            };
            return Frame::new(fa.origin(), z.into_inner(), hint.into_inner())
                .map(Shared::Axis)
                .map_err(|_| GeomError::Degenerate {
                    kind: GeomKind::Surface(SurfaceKind::Sphere),
                    reason: "non-finite frame".to_owned(),
                });
        }
        // An elliptic cylinder is no surface of revolution and never
        // reaches this arm (`intersect_surfaces` decides its pairs); it
        // is listed so the match stays exhaustive without a wildcard.
        (Surface::Plane { .. }, Surface::Plane { .. })
        | (
            Surface::Nurbs(_) | Surface::EllipticCylinder { .. },
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_),
        )
        | (
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. },
            Surface::Nurbs(_) | Surface::EllipticCylinder { .. },
        ) => Shared::None,
    })
}

/// A plane through `carrier`'s axis, whose normal is `n`: the carrier's
/// meridian, both halves of it, each crossing — the normals across
/// the cut are the plane's and one in the plane. With `w = Z × n` over
/// the carrier's own `Z`, a cone gives the two rulings through its apex
/// on the sides `+w` then `−w`, each from the apex along the cone's
/// `∂P/∂v` there, so a ruling's `t` is the cone's `v` less the apex's; a
/// torus gives its two tube circles about `O ± R·w`, in that order, each
/// with `X` the side and `Y` the torus's `Z`, so a circle's `t` is the
/// torus's `v`. A cylinder is `plane_cylinder`'s, and a plane, a sphere
/// or a NURBS carries no axis: `None`, which the caller turns into the
/// refusal naming the pair.
fn through_the_axis(carrier: &Surface, n: UnitVec3) -> Option<SurfaceIntersection> {
    let side = |frame: &Frame| UnitVec3::try_new(frame.z().cross(&n), 0.0);
    match *carrier {
        Surface::Cone {
            ref frame,
            radius,
            half_angle,
        } => {
            let w = side(frame)?.into_inner();
            let (sa, ca) = half_angle.sin_cos();
            let z = frame.z().into_inner();
            let apex = frame.origin() - (radius * ca / sa) * z;
            let ruling = |w: Vec3| Curve::Line {
                origin: apex,
                direction: UnitVec3::new_normalize(sa * w + ca * z),
            };
            Some(SurfaceIntersection::curves_of(
                MeetKind::Crossing,
                vec![ruling(w), ruling(-w)],
            ))
        }
        Surface::Torus {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            let w = side(frame)?.into_inner();
            let z = frame.z().into_inner();
            let tube = |w: Vec3| {
                Frame::new(frame.origin() + major_radius * w, w.cross(&z), w)
                    .ok()
                    .map(|frame| Curve::Circle {
                        frame,
                        radius: minor_radius,
                    })
            };
            Some(SurfaceIntersection::curves_of(
                MeetKind::Crossing,
                vec![tube(w)?, tube(-w)?],
            ))
        }
        Surface::Plane { .. }
        | Surface::Cylinder { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Sphere { .. }
        | Surface::Nurbs(_) => None,
    }
}

/// A meridian section in the plane through the axis: `(ρ, z)`.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Section {
    /// `n · (ρ, z) = c`, `n` unit.
    Line { n: [f64; 2], c: f64 },
    /// Centre and radius.
    Circle { centre: [f64; 2], radius: f64 },
}

impl Section {
    /// The distance from `p` to the section.
    fn distance(&self, p: [f64; 2]) -> f64 {
        match *self {
            Section::Line { n, c } => (n[0] * p[0] + n[1] * p[1] - c).abs(),
            Section::Circle { centre, radius } => {
                ((p[0] - centre[0]).hypot(p[1] - centre[1]) - radius).abs()
            }
        }
    }
}

/// The meridian sections of `s` in `axis`'s half-plane, both mirrors of
/// each mirror pair.
fn sections(s: &Surface, axis: &Frame) -> Vec<Section> {
    let z = axis.z();
    let height = |p: Point3| (p - axis.origin()).dot(&z);
    match *s {
        Surface::Plane { ref frame } => vec![Section::Line {
            n: [0.0, 1.0],
            c: height(frame.origin()),
        }],
        Surface::Cylinder { radius, .. } => vec![
            Section::Line {
                n: [1.0, 0.0],
                c: radius,
            },
            Section::Line {
                n: [1.0, 0.0],
                c: -radius,
            },
        ],
        Surface::Cone {
            ref frame,
            radius,
            half_angle,
        } => {
            // The apex is at `v = −R / sin α`, `R cot α` behind the origin
            // along the cone's own `Z`; the two lines through it at the
            // half-angle are `ρ cos α ∓ z sin α = ∓ z_apex sin α`.
            let (sa, ca) = half_angle.sin_cos();
            let apex = frame.origin() - (radius * ca / sa) * frame.z().into_inner();
            let za = height(apex);
            vec![
                Section::Line {
                    n: [ca, -sa],
                    c: -za * sa,
                },
                Section::Line {
                    n: [ca, sa],
                    c: za * sa,
                },
            ]
        }
        Surface::Sphere { ref frame, radius } => vec![Section::Circle {
            centre: [0.0, height(frame.origin())],
            radius,
        }],
        Surface::Torus {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            let zc = height(frame.origin());
            vec![
                Section::Circle {
                    centre: [major_radius, zc],
                    radius: minor_radius,
                },
                Section::Circle {
                    centre: [-major_radius, zc],
                    radius: minor_radius,
                },
            ]
        }
        // Not a surface of revolution: no meridian section.
        Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => Vec::new(),
    }
}

/// How a 2D meeting was made: a crossing, or a touch within `tol.linear`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Cross,
    Touch,
}

/// What two sections have in common.
enum Meeting {
    /// The same line or circle within the tolerance.
    Coincident,
    /// Up to two points, all of one kind.
    Points(Kind, Vec<[f64; 2]>),
}

/// Two sections, by the closed form of each pair: two lines cross at one
/// point or are parallel within `tol.angular` and then coincident or
/// apart within `tol.linear`; a line and a circle touch when the centre's
/// distance to the line is within `tol.linear` of the radius, at the
/// foot of the centre, and cross at two points when it is less; two
/// circles about one centre within `tol.linear` are coincident or nested
/// by their radii, touch when the centres are within `tol.linear` of the
/// sum or the difference of the radii, at the point on the line of
/// centres, and cross at the two points of their radical line between.
fn meet(a: Section, b: Section, tol: Tolerance) -> Meeting {
    let none = || Meeting::Points(Kind::Cross, Vec::new());
    match (a, b) {
        (Section::Line { n: n1, c: c1 }, Section::Line { n: n2, c: c2 }) => {
            let cross = n1[0] * n2[1] - n1[1] * n2[0];
            let dot = n1[0] * n2[0] + n1[1] * n2[1];
            if cross.abs().atan2(dot.abs()) <= tol.angular {
                let same_side = if dot >= 0.0 { c2 } else { -c2 };
                return if (c1 - same_side).abs() <= tol.linear {
                    Meeting::Coincident
                } else {
                    none()
                };
            }
            let rho = (c1 * n2[1] - c2 * n1[1]) / cross;
            let z = (n1[0] * c2 - n2[0] * c1) / cross;
            Meeting::Points(Kind::Cross, vec![[rho, z]])
        }
        (Section::Line { n, c }, Section::Circle { centre, radius })
        | (Section::Circle { centre, radius }, Section::Line { n, c }) => {
            let d = n[0] * centre[0] + n[1] * centre[1] - c;
            let foot = [centre[0] - d * n[0], centre[1] - d * n[1]];
            if (d.abs() - radius).abs() <= tol.linear {
                return Meeting::Points(Kind::Touch, vec![foot]);
            }
            if d.abs() >= radius {
                return none();
            }
            let half = (radius * radius - d * d).sqrt();
            let along = [-n[1], n[0]];
            Meeting::Points(
                Kind::Cross,
                vec![
                    [foot[0] - half * along[0], foot[1] - half * along[1]],
                    [foot[0] + half * along[0], foot[1] + half * along[1]],
                ],
            )
        }
        (
            Section::Circle {
                centre: c1,
                radius: r1,
            },
            Section::Circle {
                centre: c2,
                radius: r2,
            },
        ) => {
            let (dr, dz) = (c2[0] - c1[0], c2[1] - c1[1]);
            let d = dr.hypot(dz);
            if d <= tol.linear {
                return if (r1 - r2).abs() <= tol.linear {
                    Meeting::Coincident
                } else {
                    none()
                };
            }
            let u = [dr / d, dz / d];
            let outside = (d - (r1 + r2)).abs() <= tol.linear;
            let inside = (d - (r1 - r2).abs()).abs() <= tol.linear;
            if outside || inside {
                // `r1` along the line of centres toward the second for an
                // outside touch and for an inside one around the smaller
                // second circle, away from it for an inside one within the
                // larger second.
                let along = if outside { r1 } else { r1.copysign(r1 - r2) };
                return Meeting::Points(
                    Kind::Touch,
                    vec![[c1[0] + along * u[0], c1[1] + along * u[1]]],
                );
            }
            if d > r1 + r2 || d < (r1 - r2).abs() {
                return none();
            }
            let along = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
            let half = (r1 * r1 - along * along).max(0.0).sqrt();
            let foot = [c1[0] + along * u[0], c1[1] + along * u[1]];
            let side = [-u[1], u[0]];
            Meeting::Points(
                Kind::Cross,
                vec![
                    [foot[0] - half * side[0], foot[1] - half * side[1]],
                    [foot[0] + half * side[0], foot[1] + half * side[1]],
                ],
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Precision;

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    /// The coaxial pairs here ignore the region.
    fn within() -> Aabb {
        Aabb {
            min: [-100.0; 3],
            max: [100.0; 3],
        }
    }

    /// The curves of a `Meets` with no points, every one of `kind`.
    fn only(r: &SurfaceIntersection, kind: MeetKind) -> Vec<Curve> {
        assert!(r.points().is_empty(), "{r:?}");
        r.curves()
            .iter()
            .map(|m| {
                assert_eq!(m.kind, kind, "{r:?}");
                m.curve.clone()
            })
            .collect()
    }

    /// A `Meets` of these points alone.
    fn points(kind: MeetKind, points: &[Point3]) -> SurfaceIntersection {
        SurfaceIntersection::Meets {
            curves: Vec::new(),
            points: points
                .iter()
                .map(|&point| MeetPoint { point, kind })
                .collect(),
        }
    }

    #[test]
    fn a_plane_through_the_apex_is_a_point_and_beside_it_a_circle() {
        let cone = Surface::Cone {
            frame: Frame::world(),
            radius: 2.0,
            half_angle: core::f64::consts::FRAC_PI_4,
        };
        // The apex is at `z = −2`.
        let through = Surface::Plane {
            frame: Frame::from_z(Point3::new(5.0, 1.0, -2.0), Vec3::z()).unwrap(),
        };
        assert_eq!(
            intersect_coaxial(
                &through,
                &cone,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            points(MeetKind::Crossing, &[Point3::new(0.0, 0.0, -2.0)])
        );
        let beside = Surface::Plane {
            frame: Frame::from_z(Point3::new(5.0, 1.0, 1.0), -Vec3::z()).unwrap(),
        };
        let c = only(
            &intersect_coaxial(
                &cone,
                &beside,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        let [Curve::Circle { frame, radius }] = c.as_slice() else {
            panic!("{c:?}")
        };
        assert_eq!(frame.origin(), Point3::new(0.0, 0.0, 1.0));
        assert!((radius - 3.0).abs() <= 1e-15);
        assert_eq!(frame.x(), Frame::world().x());
    }

    #[test]
    fn a_sphere_on_a_cylinder_of_its_radius_touches_along_the_equator() {
        let wall = Surface::Cylinder {
            frame: Frame::world(),
            radius: 2.0,
        };
        let ball = Surface::Sphere {
            frame: Frame::from_z(Point3::new(0.0, 0.0, 3.0), Vec3::x()).unwrap(),
            radius: 2.0,
        };
        let c = only(
            &intersect_coaxial(
                &ball,
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Touch,
        );
        let [Curve::Circle { frame, radius }] = c.as_slice() else {
            panic!("{c:?}")
        };
        assert_eq!((frame.origin(), *radius), (Point3::new(0.0, 0.0, 3.0), 2.0));
        // The cylinder carries the axis; the sphere's own Z does not.
        assert_eq!(frame.z(), Frame::world().z());
    }

    #[test]
    fn two_spheres_touch_at_a_point_and_cross_in_a_circle() {
        let a = Surface::Sphere {
            frame: Frame::world(),
            radius: 2.0,
        };
        let touch = Surface::Sphere {
            frame: Frame::world().with_origin(Point3::new(3.0, 0.0, 0.0)),
            radius: 1.0,
        };
        assert_eq!(
            intersect_coaxial(
                &a,
                &touch,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            points(MeetKind::Touch, &[Point3::new(2.0, 0.0, 0.0)])
        );
        let cross = Surface::Sphere {
            frame: Frame::world().with_origin(Point3::new(0.0, 2.0, 0.0)),
            radius: 2.0,
        };
        let c = only(
            &intersect_coaxial(
                &a,
                &cross,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        let [Curve::Circle { frame, radius }] = c.as_slice() else {
            panic!("{c:?}")
        };
        assert!((frame.origin() - Point3::new(0.0, 1.0, 0.0)).norm() <= 1e-15);
        assert!((radius - 3f64.sqrt()).abs() <= 1e-15);
        assert!((frame.z().into_inner() - Vec3::y()).norm() <= 1e-15);
        // Two spheres about one centre: the same, or nested.
        let same = Surface::Sphere {
            frame: Frame::from_z(Point3::origin(), Vec3::y()).unwrap(),
            radius: 2.0,
        };
        assert_eq!(
            intersect_coaxial(
                &a,
                &same,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::Coincident
        );
        assert_eq!(
            intersect_coaxial(
                &a,
                &touch,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            intersect_coaxial(
                &touch,
                &a,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap()
        );
    }

    /// A sphere about a point of a cone's axis through its apex: the apex
    /// on the axis and a circle off it, in one `Meets` (ADR-0018). Apex at
    /// `z = −2`, centre at `z = 1`, radius 3: the nappe `ρ = z + 2` meets
    /// `ρ² + (z − 1)² = 9` again at `z = 1`, `ρ = 3`.
    #[test]
    fn a_sphere_through_the_apex_meets_in_a_circle_and_a_point() {
        let cone = Surface::Cone {
            frame: Frame::world(),
            radius: 2.0,
            half_angle: core::f64::consts::FRAC_PI_4,
        };
        let ball = Surface::Sphere {
            frame: Frame::world().with_origin(Point3::new(0.0, 0.0, 1.0)),
            radius: 3.0,
        };
        let r = intersect_coaxial(
            &cone,
            &ball,
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let [point] = r.points() else { panic!("{r:?}") };
        assert_eq!(point.kind, MeetKind::Crossing);
        assert!((point.point - Point3::new(0.0, 0.0, -2.0)).norm() <= 1e-12);
        let [circle] = r.curves() else {
            panic!("{r:?}")
        };
        assert_eq!(circle.kind, MeetKind::Crossing);
        let Curve::Circle { frame, radius } = &circle.curve else {
            panic!("{r:?}")
        };
        assert!((frame.origin() - Point3::new(0.0, 0.0, 1.0)).norm() <= 1e-12);
        assert!((radius - 3.0).abs() <= 1e-12);
        assert_eq!(
            intersect_coaxial(
                &ball,
                &cone,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            r
        );
    }
}
