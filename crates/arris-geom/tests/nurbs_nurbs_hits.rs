//! A NURBS curve against a NURBS surface (ADR-0068's arm of one unknown
//! fewer): the exact twins of a line, a circle or an ellipse and of an
//! analytic surface meet where the analytic pair meets, at the same
//! points; a twin circle through a twin cylinder in the four places it
//! was built to cross; one lying on it `Coincident`; one a half tolerance
//! off a touch and two tolerances off a miss; a line through a sphere
//! twin's pole crossing it there, one in the pole's tangent plane
//! touching it; a segment ending on the surface a hit at its end, not a
//! touch. `tests/oracle.rs` holds fitted pairs to Open CASCADE's.

use arris_debug::testing::tol;
use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::{RADIUS_RANGE, surface};
use arris_debug::prop::{check, finite_f64, unit_vec3};
use arris_geom::{
    Curve, CurveSurfaceHit, CurveSurfaceIntersection, NurbsCurve, NurbsSurface, Surface,
    intersect_curve_surface,
};
use arris_math::{Frame, Interval, Meter, Point3, Vec3};
use proptest::prelude::*;

fn meet(c: &Curve, s: &Surface) -> CurveSurfaceIntersection {
    intersect_curve_surface(c, s, tol(), &mut Meter::default())
        .unwrap_or_else(|e| panic!("{c:?} vs {s:?}: {e}"))
}

fn hits(r: &CurveSurfaceIntersection) -> &[CurveSurfaceHit] {
    match r {
        CurveSurfaceIntersection::Points(h) => h,
        CurveSurfaceIntersection::Coincident => &[],
    }
}

/// How far along a line its twin reaches either way.
const REACH: f64 = 40.0;

/// The twin of `surface` over the whole of it a curve of the strategies
/// can reach, as `tests/conic_nurbs.rs` takes it.
fn twin(surface: &Surface) -> Option<NurbsSurface> {
    let long = Interval::new(-300.0, 300.0).unwrap();
    let bounds = match surface {
        Surface::Plane { .. } => [long, long],
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
            [Interval::TURN, long]
        }
        Surface::Sphere { .. } => [Interval::TURN, Interval::new(-FRAC_PI_2, FRAC_PI_2).ok()?],
        Surface::Torus { .. } => [Interval::TURN, Interval::TURN],
        Surface::Nurbs(_) => return None,
    };
    surface.to_nurbs(bounds).ok()
}

/// The twin of a line over `[−REACH, REACH]` of it, of a circle or an
/// ellipse over its turn.
fn curve_twin(curve: &Curve) -> Curve {
    let spline = match curve {
        Curve::Line { origin, direction } => {
            let d = direction.into_inner();
            NurbsCurve::new(
                1,
                vec![0.0, 0.0, 1.0, 1.0],
                vec![origin - REACH * d, origin + REACH * d],
                vec![1.0, 1.0],
            )
        }
        Curve::Circle { frame, radius } => NurbsCurve::circle(frame, *radius, Interval::TURN),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => NurbsCurve::ellipse(frame, *major_radius, *minor_radius, Interval::TURN),
        Curve::Nurbs(c) => Ok(c.clone()),
    };
    Curve::Nurbs(spline.unwrap())
}

/// The analytic pair's hits as points, each with its touch, matched one
/// for one by the twins' within `1e-6`.
fn same_points(
    analytic: &[CurveSurfaceHit],
    nurbs: &[CurveSurfaceHit],
) -> Result<(), TestCaseError> {
    prop_assert_eq!(analytic.len(), nurbs.len(), "{:?} vs {:?}", analytic, nurbs);
    for a in analytic {
        let found = nurbs
            .iter()
            .any(|n| (a.point - n.point).norm() <= 1e-6 && a.tangent == n.tangent);
        prop_assert!(found, "{a:?} is not among {nurbs:?}");
    }
    Ok(())
}

fn twin_property(curve: Curve, surface: Surface) -> Result<(), TestCaseError> {
    let Some(nurbs) = twin(&surface) else {
        return Ok(());
    };
    let Ok(analytic) = intersect_curve_surface(&curve, &surface, tol(), &mut Meter::default())
    else {
        return Ok(());
    };
    // A touch or a coincidence is decided by the tolerance, which the
    // twins' rounding may tip either way: only crossings are held.
    let CurveSurfaceIntersection::Points(expected) = &analytic else {
        return Ok(());
    };
    if expected.iter().any(|h| h.tangent) {
        return Ok(());
    }
    // A line's twin is a stretch of it: hits beyond, or near enough its
    // ends to be decided by them, are not the twin's.
    let expected: Vec<CurveSurfaceHit> = match curve {
        Curve::Line { .. } => {
            if expected.iter().any(|h| (h.t.abs() - REACH).abs() < 1e-3) {
                return Ok(());
            }
            expected
                .iter()
                .filter(|h| h.t.abs() < REACH)
                .copied()
                .collect()
        }
        _ => expected.clone(),
    };
    let twin_curve = curve_twin(&curve);
    let got = intersect_curve_surface(
        &twin_curve,
        &Surface::Nurbs(nurbs),
        tol(),
        &mut Meter::default(),
    );
    match got {
        Ok(r) => same_points(&expected, hits(&r)),
        Err(e) => Err(TestCaseError::fail(format!(
            "the twin of {curve:?} vs the twin of {surface:?}: {e}"
        ))),
    }
}

/// A line, a circle or an ellipse through a point of the surface in a
/// random direction or plane: it crosses the surface there, and wherever
/// else it reaches it.
fn through_the_surface() -> impl Strategy<Value = (Curve, Surface)> {
    (
        surface(),
        finite_f64(0.0..=TAU),
        finite_f64(-1.2..=1.2),
        unit_vec3(),
        finite_f64(RADIUS_RANGE),
        finite_f64(0.0..=*RADIUS_RANGE.end()),
        0..3usize,
    )
        .prop_map(|(surface, u, v, z, minor, extra, kind)| {
            let at = surface.point(u, v);
            let frame = Frame::from_z(at, z.into_inner()).unwrap();
            let curve = match kind {
                0 => Curve::Line {
                    origin: at,
                    direction: z,
                },
                1 => Curve::Circle {
                    frame,
                    radius: minor,
                },
                _ => Curve::Ellipse {
                    frame,
                    major_radius: minor + extra,
                    minor_radius: minor,
                },
            };
            (curve, surface)
        })
}

#[test]
fn a_twin_curve_meets_a_twin_surface_where_the_analytic_pair_meets() {
    check(through_the_surface(), |(c, s)| twin_property(c, s));
}

fn wall() -> (Surface, Surface) {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    };
    let nurbs = pipe
        .to_nurbs([Interval::TURN, Interval::new(-2.0, 2.0).unwrap()])
        .unwrap();
    (pipe, Surface::Nurbs(nurbs))
}

fn circle(origin: [f64; 3], z: [f64; 3], radius: f64) -> Curve {
    Curve::Circle {
        frame: Frame::from_z(Point3::from(origin), Vec3::from(z)).unwrap(),
        radius,
    }
}

#[test]
fn a_twin_circle_through_a_twin_cylinder_crosses_it_four_times() {
    let (pipe, wall) = wall();
    let ring = circle([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.5);
    let ours = meet(&curve_twin(&ring), &wall);
    let theirs = meet(&ring, &pipe);
    assert_eq!(hits(&ours).len(), 4, "{ours:?}");
    same_points(hits(&theirs), hits(&ours)).unwrap();
}

#[test]
fn a_twin_circle_lying_on_a_twin_cylinder_is_coincident() {
    let (_, wall) = wall();
    let ring = circle([0.0, 0.0, 0.5], [0.0, 0.0, 1.0], 1.0);
    assert_eq!(
        meet(&curve_twin(&ring), &wall),
        CurveSurfaceIntersection::Coincident
    );
}

#[test]
fn a_twin_circle_within_the_tolerance_of_a_twin_touches_it_and_beyond_it_misses() {
    let (_, wall) = wall();
    // A circle of radius 1 in the plane x = 1 + d, centred on the wall's
    // outermost line: it grazes the wall at the two ends of its diameter
    // along z, `d` clear of it.
    let at = |d: f64| curve_twin(&circle([1.0 + d, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0));
    for d in [0.0, 0.5 * tol().linear] {
        let touch = meet(&at(d), &wall);
        assert_eq!(hits(&touch).len(), 2, "{d}: {touch:?}");
        assert!(hits(&touch).iter().all(|h| h.tangent), "{d}: {touch:?}");
    }
    let far = meet(&at(2.0 * tol().linear), &wall);
    assert!(hits(&far).is_empty(), "{far:?}");
}

fn ball() -> Surface {
    let sphere = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    Surface::Nurbs(
        sphere
            .to_nurbs([
                Interval::TURN,
                Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap(),
            ])
            .unwrap(),
    )
}

fn segment(from: [f64; 3], to: [f64; 3]) -> Curve {
    Curve::Nurbs(
        NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![Point3::from(from), Point3::from(to)],
            vec![1.0, 1.0],
        )
        .unwrap(),
    )
}

#[test]
fn a_twin_line_through_a_sphere_twins_pole_crosses_it_there() {
    // Down the axis, through both poles.
    let r = meet(&segment([0.0, 0.0, -3.0], [0.0, 0.0, 3.0]), &ball());
    let h = hits(&r);
    assert_eq!(h.len(), 2, "{r:?}");
    assert!(h.iter().all(|h| !h.tangent), "{r:?}");
    assert!(
        (h[0].point - Point3::new(0.0, 0.0, -2.0)).norm() < 1e-9,
        "{r:?}"
    );
    assert!(
        (h[1].point - Point3::new(0.0, 0.0, 2.0)).norm() < 1e-9,
        "{r:?}"
    );
    // Slanted through the north pole.
    let r = meet(&segment([-1.0, 0.5, 3.5], [0.5, -0.25, 1.25]), &ball());
    let h = hits(&r);
    assert_eq!(h.len(), 1, "{r:?}");
    assert!(!h[0].tangent && (h[0].point - Point3::new(0.0, 0.0, 2.0)).norm() < 1e-9);
}

#[test]
fn a_twin_line_in_a_poles_tangent_plane_touches_it_there() {
    let r = meet(&segment([-3.0, 1.0, 2.0], [3.0, -1.0, 2.0]), &ball());
    let h = hits(&r);
    assert_eq!(h.len(), 1, "{r:?}");
    assert!(h[0].tangent && (h[0].point - Point3::new(0.0, 0.0, 2.0)).norm() < 1e-6);
}

#[test]
fn a_segment_ending_on_a_twin_meets_it_at_its_end_without_touching() {
    // From inside the ball out to its surface at (2, 0, 0).
    let r = meet(&segment([0.5, 0.0, 0.0], [2.0, 0.0, 0.0]), &ball());
    let h = hits(&r);
    assert_eq!(h.len(), 1, "{r:?}");
    assert!(!h[0].tangent && (h[0].t - 1.0).abs() < 1e-12, "{r:?}");
}
