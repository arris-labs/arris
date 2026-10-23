//! A circle or an ellipse against a NURBS surface (ADR-0061): the exact
//! twin of an analytic surface met where the analytic surface is, in the
//! same places along the curve; a circle through a twin cylinder in the
//! four places it was built to cross; a circle lying on a twin
//! `Coincident`; one a half tolerance off a twin a touch; one two
//! tolerances off a miss. `tests/oracle.rs` holds the same hits to Open
//! CASCADE's.

use arris_debug::testing::tol;
use core::f64::consts::TAU;

use arris_debug::prop::geom::{RADIUS_RANGE, surface};
use arris_debug::prop::{check, finite_f64, unit_vec3};
use arris_geom::{
    Curve, CurveSurfaceHit, CurveSurfaceIntersection, GeomError, NurbsSurface, Surface,
    intersect_curve_surface,
};
use arris_math::{Frame, Interval, Meter, Point3, Vec3};
use proptest::prelude::*;

fn frame(origin: [f64; 3], z: [f64; 3]) -> Frame {
    Frame::from_z(Point3::from(origin), Vec3::from(z)).unwrap()
}

fn meet(c: &Curve, s: &Surface) -> Result<CurveSurfaceIntersection, GeomError> {
    intersect_curve_surface(c, s, tol(), &mut Meter::default())
}

fn hits(r: &CurveSurfaceIntersection) -> &[CurveSurfaceHit] {
    match r {
        CurveSurfaceIntersection::Points(h) => h,
        CurveSurfaceIntersection::Coincident => &[],
    }
}

/// The twin of `surface` over the whole of it a section can reach: a full
/// turn, and a long stretch of its axis or its plane.
fn twin(surface: &Surface) -> Option<NurbsSurface> {
    use core::f64::consts::FRAC_PI_2;
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

fn unit_circle_at(origin: [f64; 3], z: [f64; 3], radius: f64) -> Curve {
    Curve::Circle {
        frame: frame(origin, z),
        radius,
    }
}

/// The distance in the circle's parameter, a turn apart being none.
fn apart(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(TAU);
    d.min(TAU - d)
}

fn same_hits(analytic: &[CurveSurfaceHit], nurbs: &[CurveSurfaceHit]) -> Result<(), TestCaseError> {
    prop_assert_eq!(analytic.len(), nurbs.len(), "{:?} vs {:?}", analytic, nurbs);
    for a in analytic {
        let found = nurbs.iter().any(|n| {
            apart(a.t, n.t) <= 1e-6 && (a.point - n.point).norm() <= 1e-6 && a.tangent == n.tangent
        });
        prop_assert!(found, "{a:?} is not among {nurbs:?}");
    }
    Ok(())
}

fn twin_property(conic: Curve, surface: Surface) -> Result<(), TestCaseError> {
    let Some(nurbs) = twin(&surface) else {
        return Ok(());
    };
    let Ok(analytic) = meet(&conic, &surface) else {
        return Ok(());
    };
    // A tangent or coincident case is decided by the tolerance, which the
    // twin's rounding may tip either way: only the crossings are held.
    let CurveSurfaceIntersection::Points(expected) = &analytic else {
        return Ok(());
    };
    if expected.iter().any(|h| h.tangent) {
        return Ok(());
    }
    let got = meet(&conic, &Surface::Nurbs(nurbs));
    match got {
        Ok(r) => same_hits(expected, hits(&r)),
        Err(e) => Err(TestCaseError::fail(format!(
            "{conic:?} vs the twin of {surface:?}: {e}"
        ))),
    }
}

/// A conic centred on a point of the surface, in a random plane, with
/// the radii of a circle or an ellipse: it cuts the surface wherever the
/// surface's section by that plane reaches it.
fn on_the_surface() -> impl Strategy<Value = (Curve, Surface)> {
    (
        surface(),
        finite_f64(0.0..=TAU),
        finite_f64(-1.2..=1.2),
        unit_vec3(),
        finite_f64(RADIUS_RANGE),
        finite_f64(0.0..=*RADIUS_RANGE.end()),
        any::<bool>(),
    )
        .prop_map(|(surface, u, v, z, minor, extra, round)| {
            let origin = surface.point(u, v);
            let frame = Frame::from_z(origin, z.into_inner()).unwrap();
            let conic = if round {
                Curve::Circle {
                    frame,
                    radius: minor,
                }
            } else {
                Curve::Ellipse {
                    frame,
                    major_radius: minor + extra,
                    minor_radius: minor,
                }
            };
            (conic, surface)
        })
}

#[test]
fn a_conic_meets_the_twin_of_an_analytic_surface_where_it_meets_the_surface() {
    check(on_the_surface(), |(c, s)| twin_property(c, s));
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

#[test]
fn a_circle_through_a_cylinder_twin_crosses_it_four_times() {
    let (pipe, wall) = wall();
    // A circle of radius 1.5 about an axis along x through the pipe's
    // axis: it enters and leaves the wall twice each side.
    let ring = unit_circle_at([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.5);
    let ours = meet(&ring, &wall).unwrap();
    let theirs = meet(&ring, &pipe).unwrap();
    assert_eq!(hits(&ours).len(), 4, "{ours:?}");
    same_hits(hits(&theirs), hits(&ours)).unwrap();
}

#[test]
fn a_circle_lying_on_a_twin_is_coincident() {
    let (_, wall) = wall();
    let ring = unit_circle_at([0.0, 0.0, 0.5], [0.0, 0.0, 1.0], 1.0);
    assert_eq!(
        meet(&ring, &wall).unwrap(),
        CurveSurfaceIntersection::Coincident
    );
}

#[test]
fn a_circle_within_the_tolerance_of_a_twin_touches_it_and_beyond_it_misses() {
    let (_, wall) = wall();
    // A circle of radius 1 in the plane x = 1 + d, centred on the wall's
    // outermost line: it grazes the wall at the two ends of its diameter
    // along z, `d` clear of it.
    let at = |d: f64| unit_circle_at([1.0 + d, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0);
    let touch = meet(&at(0.0), &wall).unwrap();
    assert_eq!(hits(&touch).len(), 2, "{touch:?}");
    assert!(hits(&touch).iter().all(|h| h.tangent), "{touch:?}");
    let near = meet(&at(0.5 * tol().linear), &wall).unwrap();
    assert_eq!(hits(&near).len(), 2, "{near:?}");
    assert!(hits(&near).iter().all(|h| h.tangent), "{near:?}");
    let far = meet(&at(2.0 * tol().linear), &wall).unwrap();
    assert!(hits(&far).is_empty(), "{far:?}");
}
