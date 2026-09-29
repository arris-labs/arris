//! The quadric tracer (`docs/DATA-MODEL.md` §Curves): sections
//! of known topology, the singular points decided in the tolerance, a
//! closed form to hold a traced curve to, and at random poses of every
//! pair — each sample on both surfaces, the argument order not reaching
//! the result, and a brute-force sweep of rulings finding no hit the
//! branches missed.

use core::f64::consts::TAU;

use arris_debug::prop::{self, check};
use arris_geom::{
    BranchEnd, Curve, CurveSurfaceIntersection, GeomError, SectionBranch, SectionFault,
    SectionTrace, Surface, intersect_curve_surface, trace_quadrics,
};
use arris_math::{Aabb, Frame, Point3, Precision, Tolerance, UnitVec3, Vec3};
use proptest::prelude::*;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn cube(half: f64) -> Aabb {
    Aabb {
        min: [-half; 3],
        max: [half; 3],
    }
}

fn cylinder(origin: [f64; 3], axis: [f64; 3], radius: f64) -> Surface {
    let frame = Frame::from_z(Point3::from(origin), Vec3::from(axis)).unwrap();
    Surface::Cylinder { frame, radius }
}

fn distance(surface: &Surface, p: Point3) -> f64 {
    surface.project(p).map_or(f64::INFINITY, |on| on.distance)
}

/// `n` points along a branch, both ends included on an open one and the
/// start once on a closed one.
fn samples(branch: &SectionBranch, n: usize) -> Vec<Point3> {
    let domain = branch.domain();
    let steps = if branch.is_closed() { n } else { n - 1 };
    (0..n)
        .map(|i| branch.point(domain.lerp(i as f64 / steps as f64)))
        .collect()
}

/// The largest distance of any sample of any branch from either surface.
fn worst_off(trace: &SectionTrace, a: &Surface, b: &Surface) -> f64 {
    trace
        .branches()
        .iter()
        .flat_map(|branch| samples(branch, 65))
        .chain(trace.points().iter().map(|p| p.point))
        .map(|p| distance(a, p).max(distance(b, p)))
        .fold(0.0, f64::max)
}

fn segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let t = if ab.norm_squared() > 0.0 {
        ((p - a).dot(&ab) / ab.norm_squared()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).norm()
}

/// How far `p` is from the trace: its branches as polylines of `n`
/// samples, and its points.
fn distance_to_trace(polylines: &[Vec<Point3>], trace: &SectionTrace, p: Point3) -> f64 {
    polylines
        .iter()
        .flat_map(|line| {
            line.windows(2)
                .map(|w| segment_distance(p, w[0], w[1]))
                .chain(
                    line.first()
                        .zip(line.last())
                        .map(|(a, b)| segment_distance(p, *b, *a)),
                )
        })
        .chain(trace.points().iter().map(|q| (q.point - p).norm()))
        .fold(f64::INFINITY, f64::min)
}

/// A pipe of radius 1 along x at height `y` against a pipe of radius 2
/// along z.
fn pipes(y: f64) -> (Surface, Surface) {
    (
        cylinder([0.0; 3], [0.0, 0.0, 1.0], 2.0),
        cylinder([0.0, y, 0.0], [1.0, 0.0, 0.0], 1.0),
    )
}

#[test]
fn crossing_axes_of_unequal_radii_meet_in_two_loops() {
    let (main, branch) = pipes(0.0);
    let trace = trace_quadrics(
        &main,
        &branch,
        &cube(5.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(trace.branches().len(), 2);
    assert!(trace.points().is_empty());
    assert!(trace.branches().iter().all(SectionBranch::is_closed));
    assert!(worst_off(&trace, &main, &branch) < 1e-12);
    // One loop on each side of the main pipe.
    let sides: Vec<f64> = trace
        .branches()
        .iter()
        .map(|b| b.point(0.0).x.signum())
        .collect();
    assert_eq!(sides.iter().sum::<f64>(), 0.0);
    for b in trace.branches() {
        let side = b.point(0.0).x.signum();
        assert!(samples(b, 64).iter().all(|p| p.x.signum() == side));
        // Periodic over its own domain.
        let t = 0.37 * b.domain().length();
        assert!((b.point(t) - b.point(t + b.domain().length())).norm() < 1e-12);
    }
}

#[test]
fn skew_axes_meet_in_one_loop_when_the_smaller_pipe_breaks_out() {
    let (main, branch) = pipes(1.5);
    let trace = trace_quadrics(
        &main,
        &branch,
        &cube(5.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(trace.branches().len(), 1);
    assert!(trace.points().is_empty());
    assert!(trace.branches()[0].is_closed());
    assert!(worst_off(&trace, &main, &branch) < 1e-12);
    // The loop visits both sides of the main pipe.
    let xs: Vec<f64> = samples(&trace.branches()[0], 64)
        .iter()
        .map(|p| p.x)
        .collect();
    assert!(xs.iter().any(|&x| x > 1.0) && xs.iter().any(|&x| x < -1.0));
}

#[test]
fn skew_axes_meet_in_two_loops_when_the_smaller_pipe_stays_inside() {
    let (main, branch) = pipes(0.5);
    let trace = trace_quadrics(
        &main,
        &branch,
        &cube(5.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(trace.branches().len(), 2);
    assert!(trace.branches().iter().all(SectionBranch::is_closed));
    assert!(worst_off(&trace, &main, &branch) < 1e-12);
}

/// `geom/c2-cylinder-pairs`' skew pose, where a turning point of the loop
/// lies on the walked cylinder's base circle to rounding: `b`, `√D` and
/// `c` all vanish there, and the root has to be taken in the form whose
/// rounding does not divide by them. The loop starts at that turn, so its
/// seam is where it showed: half a unit off the larger cylinder.
#[test]
fn a_turning_point_on_the_walked_base_circle_is_on_both_surfaces() {
    let main = Surface::Cylinder {
        frame: Frame::new(
            Point3::new(-2.5, 1.75, 0.5),
            Vec3::new(2.0, 3.0, 6.0),
            Vec3::new(3.0, -6.0, 2.0),
        )
        .unwrap(),
        radius: 2.0,
    };
    let drill = Surface::Cylinder {
        frame: Frame::new(
            Point3::new(-1.3732672280129359, 3.4725413007549086, 1.0131517589601917),
            Vec3::new(
                0.8376945264427496,
                -0.5461041060954285,
                -0.006179455766535667,
            ),
            Vec3::new(2.0, 3.0, 6.0),
        )
        .unwrap(),
        radius: 1.2,
    };
    let trace = trace_quadrics(
        &main,
        &drill,
        &cube(20.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(trace.branches().len(), 1);
    let b = &trace.branches()[0];
    assert!(b.is_closed());
    assert!(worst_off(&trace, &main, &drill) < 1e-12);
    let step = 1e-9 * b.domain().length();
    for t in [0.0, b.domain().hi()] {
        assert!((b.point(t) - b.point(t + step)).norm() < 1e-6, "at {t}");
    }
}

#[test]
fn pipes_apart_do_not_meet() {
    let (main, branch) = pipes(4.0);
    let trace = trace_quadrics(
        &main,
        &branch,
        &cube(8.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert!(trace.branches().is_empty() && trace.points().is_empty());
}

/// The figure eight of an inner tangency, exact and a fraction of the
/// tolerance either way: one singular point, and two lobes that start and
/// end at it exactly.
#[test]
fn an_inner_tangency_within_the_tolerance_is_a_figure_eight() {
    for nudge in [0.0, 4e-8, -4e-8] {
        let (main, branch) = pipes(1.0 + nudge);
        let trace = trace_quadrics(
            &main,
            &branch,
            &cube(5.0),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(trace.points().len(), 1, "nudge {nudge}");
        let node = trace.points()[0];
        assert!(!node.isolated);
        assert!(
            (node.point - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-6,
            "{node:?}"
        );
        assert_eq!(trace.branches().len(), 2, "nudge {nudge}");
        for b in trace.branches() {
            assert_eq!(b.ends(), Some([BranchEnd::Singular(0); 2]));
            let domain = b.domain();
            assert_eq!(b.point(domain.lo()), node.point);
            assert_eq!(b.point(domain.hi()), node.point);
        }
        assert!(
            worst_off(&trace, &main, &branch) <= tol().linear,
            "nudge {nudge}"
        );
    }
}

/// Outside the tolerance the same poses are what they are exactly: two
/// loops a hair apart, or one loop with a waist.
#[test]
fn an_inner_tangency_outside_the_tolerance_is_not_singular() {
    for (nudge, loops) in [(-1e-5, 2), (1e-5, 1)] {
        let (main, branch) = pipes(1.0 + nudge);
        let trace = trace_quadrics(
            &main,
            &branch,
            &cube(5.0),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(trace.points().is_empty(), "nudge {nudge}");
        assert_eq!(trace.branches().len(), loops, "nudge {nudge}");
        assert!(trace.branches().iter().all(SectionBranch::is_closed));
        assert!(worst_off(&trace, &main, &branch) < 1e-12);
    }
}

/// Touching from outside, exactly and within the tolerance either way —
/// a hair apart, or overlapping in a loop smaller than the tolerance can
/// tell from a point.
#[test]
fn an_outer_tangency_within_the_tolerance_is_one_isolated_point() {
    for nudge in [0.0, 4e-8, -4e-8] {
        let (main, branch) = pipes(3.0 + nudge);
        let trace = trace_quadrics(
            &main,
            &branch,
            &cube(5.0),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(trace.branches().is_empty(), "nudge {nudge}");
        assert_eq!(trace.points().len(), 1, "nudge {nudge}");
        let touch = trace.points()[0];
        assert!(touch.isolated);
        assert!((touch.point - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-6);
        assert!(worst_off(&trace, &main, &branch) <= tol().linear);
    }
    let (main, branch) = pipes(3.0 - 1e-5);
    let trace = trace_quadrics(
        &main,
        &branch,
        &cube(5.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!((trace.branches().len(), trace.points().len()), (1, 0));
}

/// Viviani's curve, a sphere of radius 2 and a cylinder of radius 1
/// through its centre and its equator: `(1 + cos φ, sin φ, 2 sin(φ/2))`.
#[test]
fn vivianis_curve_matches_its_closed_form() {
    let sphere = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let pipe = cylinder([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], 1.0);
    let trace = trace_quadrics(
        &sphere,
        &pipe,
        &cube(5.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(trace.points().len(), 1);
    assert!((trace.points()[0].point - Point3::new(2.0, 0.0, 0.0)).norm() < 1e-7);
    assert_eq!(trace.branches().len(), 2);
    // Every traced point satisfies the closed form…
    for p in trace.branches().iter().flat_map(|b| samples(b, 257)) {
        let phi = p.y.atan2(p.x - 1.0);
        let want = Point3::new(
            1.0 + phi.cos(),
            phi.sin(),
            p.z.signum() * 2.0 * (0.5 * phi).sin().abs(),
        );
        assert!((p - want).norm() < 1e-7, "{p} against {want}");
    }
    // …and every point of the closed form is on a branch.
    let polylines: Vec<_> = trace.branches().iter().map(|b| samples(b, 2049)).collect();
    for i in 0..400 {
        let phi = 2.0 * TAU * i as f64 / 400.0;
        let p = Point3::new(1.0 + phi.cos(), phi.sin(), 2.0 * (0.5 * phi).sin());
        assert!(distance_to_trace(&polylines, &trace, p) < 1e-5, "φ = {phi}");
    }
}

/// Two cones on crossing axes meet in branches that run to infinity on
/// both nappes: each is clipped to the region, ends `Clipped`, and the
/// whole section inside the region is there.
#[test]
fn unbounded_branches_of_two_cones_are_clipped_to_the_region() {
    let a = Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: 0.5,
    };
    let b = Surface::Cone {
        frame: Frame::from_z(Point3::new(0.5, 0.0, 1.0), Vec3::new(0.3, 0.1, 1.0)).unwrap(),
        radius: 0.7,
        half_angle: 0.6,
    };
    let within = cube(20.0);
    let trace = trace_quadrics(&a, &b, &within, tol(), &mut arris_math::Meter::default()).unwrap();
    assert!(!trace.branches().is_empty());
    assert!(
        trace
            .branches()
            .iter()
            .any(|br| br.ends().is_some_and(|e| e.contains(&BranchEnd::Clipped)))
    );
    assert!(worst_off(&trace, &a, &b) < 1e-9);
    assert_complete(&trace, &a, &b, &within);
}

#[test]
fn a_pair_with_no_ruled_quadric_in_it_is_unsupported() {
    let sphere = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    for (a, b) in [(&sphere, &sphere), (&plane, &sphere)] {
        assert!(matches!(
            trace_quadrics(a, b, &cube(5.0), tol(), &mut arris_math::Meter::default()),
            Err(GeomError::Unsupported { .. })
        ));
    }
    let pipe = cylinder([0.0; 3], [0.0, 0.0, 1.0], 1.0);
    assert!(matches!(
        trace_quadrics(
            &pipe,
            &plane,
            &cube(5.0),
            tol(),
            &mut arris_math::Meter::default()
        ),
        Err(GeomError::Unsupported { .. })
    ));
}

/// The poses the closed forms own are refused by name, never answered
/// wrongly: a sphere in a cylinder of its radius touches along a circle,
/// and two parallel cylinders share rulings.
#[test]
fn the_closed_forms_poses_are_refused_by_name() {
    let pipe = cylinder([0.0; 3], [0.0, 0.0, 1.0], 2.0);
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    assert!(matches!(
        trace_quadrics(
            &pipe,
            &ball,
            &cube(5.0),
            tol(),
            &mut arris_math::Meter::default()
        ),
        Err(GeomError::DegenerateSection {
            fault: SectionFault::TangentAlongCurve,
            ..
        })
    ));
    let beside = cylinder([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], 2.0);
    assert!(matches!(
        trace_quadrics(
            &pipe,
            &beside,
            &cube(5.0),
            tol(),
            &mut arris_math::Meter::default()
        ),
        Err(GeomError::DegenerateSection { .. })
    ));
}

/// Every hit of a dense sweep of both surfaces' rulings inside `within`
/// is on the trace, to what a polyline of the branches resolves.
fn assert_complete(trace: &SectionTrace, a: &Surface, b: &Surface, within: &Aabb) {
    let polylines: Vec<_> = trace
        .branches()
        .iter()
        .map(|br| samples(br, 4097))
        .collect();
    let inside = |p: Point3| (0..3).all(|i| (within.min[i]..=within.max[i]).contains(&p[i]));
    let size = (0..3)
        .map(|i| within.max[i] - within.min[i])
        .fold(0.0, f64::max);
    for (ruled, other) in [(a, b), (b, a)] {
        for k in 0..97 {
            let u = TAU * (k as f64 + 0.5) / 97.0;
            let Some(line) = ruling(ruled, u) else {
                continue;
            };
            let Ok(CurveSurfaceIntersection::Points(hits)) =
                intersect_curve_surface(&line, other, tol(), &mut arris_math::Meter::default())
            else {
                continue;
            };
            for hit in hits.iter().filter(|h| inside(h.point)) {
                let d = distance_to_trace(&polylines, trace, hit.point);
                assert!(
                    d <= 1e-3 * size,
                    "a hit of the ruling at u = {u} at {} is {d} from the trace",
                    hit.point
                );
            }
        }
    }
}

/// The ruling of a ruled quadric at `u`, as a line.
fn ruling(surface: &Surface, u: f64) -> Option<Curve> {
    let (origin, along) = match surface {
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
            let e = surface.eval(u, 0.0);
            (e.point, e.dv)
        }
        Surface::Plane { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => {
            return None;
        }
    };
    Some(Curve::Line {
        origin,
        direction: UnitVec3::new_normalize(along),
    })
}

/// `surface` with its frame's origin moved to `origin`.
fn placed(surface: &Surface, origin: Point3) -> Surface {
    let mut out = surface.clone();
    match &mut out {
        Surface::Plane { frame }
        | Surface::Cylinder { frame, .. }
        | Surface::EllipticCylinder { frame, .. }
        | Surface::Cone { frame, .. }
        | Surface::Sphere { frame, .. }
        | Surface::Torus { frame, .. } => *frame = frame.with_origin(origin),
        Surface::Nurbs(_) => {}
    }
    out
}

/// A rough size of a quadric: how far from a point of it another surface
/// can sit and still be likely to meet it.
fn size(surface: &Surface) -> f64 {
    match *surface {
        Surface::Cylinder { radius, .. }
        | Surface::Sphere { radius, .. }
        | Surface::Cone { radius, .. } => radius,
        Surface::EllipticCylinder { minor_radius, .. } => minor_radius,
        Surface::Plane { .. } | Surface::Torus { .. } | Surface::Nurbs(_) => 1.0,
    }
}

/// The second surface of a pair placed where it meets the first: its
/// origin within its own size of a point of the first.
fn meeting(
    first: impl Strategy<Value = Surface>,
    second: impl Strategy<Value = Surface>,
) -> impl Strategy<Value = (Surface, Surface, Aabb)> {
    (
        first,
        second,
        prop::finite_f64(0.0..=TAU),
        prop::finite_f64(-5.0..=5.0),
        prop::unit_vec3(),
        prop::finite_f64(0.0..=0.9),
    )
        .prop_map(|(a, b, u, v, direction, reach)| {
            let v = if matches!(a, Surface::Sphere { .. }) {
                v * 0.3
            } else {
                v
            };
            let at = a.point(u, v) + direction.into_inner() * (reach * size(&b));
            let within = Aabb::of_point(at).inflated(30.0);
            (a.clone(), placed(&b, at), within)
        })
}

/// What every trace owes: each branch and point on both surfaces, the
/// argument order not reaching the result, and nothing of the section
/// inside the region missed.
fn holds((a, b, within): (Surface, Surface, Aabb)) -> Result<(), TestCaseError> {
    let trace = match trace_quadrics(&a, &b, &within, tol(), &mut arris_math::Meter::default()) {
        Ok(trace) => trace,
        Err(e) => return Err(TestCaseError::fail(format!("refused: {e}"))),
    };
    // A point's own rounding grows with its distance from the frames.
    let bound = |p: Point3| {
        let scale = 1.0 + p.coords.norm();
        let singular = if trace.points().is_empty() {
            0.0
        } else {
            tol().linear
        };
        singular + 1e-11 * scale * scale
    };
    for p in trace.branches().iter().flat_map(|br| samples(br, 33)) {
        prop_assert!(p.coords.iter().all(|x| x.is_finite()));
        let off = distance(&a, p).max(distance(&b, p));
        prop_assert!(off <= bound(p), "{p} is {off} off a surface");
    }
    let swapped = trace_quadrics(&b, &a, &within, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    prop_assert_eq!(trace.points(), swapped.points());
    prop_assert_eq!(trace.branches().len(), swapped.branches().len());
    for (x, y) in trace.branches().iter().zip(swapped.branches()) {
        prop_assert_eq!(samples(x, 9), samples(y, 9));
    }
    assert_complete(&trace, &a, &b, &within);
    Ok(())
}

#[test]
fn random_cylinder_pairs_trace() {
    check(
        meeting(prop::geom::cylinder(), prop::geom::cylinder()),
        holds,
    );
}

#[test]
fn random_cylinder_and_cone_pairs_trace() {
    check(meeting(prop::geom::cylinder(), prop::geom::cone()), holds);
}

#[test]
fn random_cone_pairs_trace() {
    check(meeting(prop::geom::cone(), prop::geom::cone()), holds);
}

#[test]
fn random_sphere_and_cylinder_pairs_trace() {
    check(meeting(prop::geom::sphere(), prop::geom::cylinder()), holds);
}

#[test]
fn random_sphere_and_cone_pairs_trace() {
    check(meeting(prop::geom::sphere(), prop::geom::cone()), holds);
}

#[test]
fn random_elliptic_cylinder_pairs_trace() {
    check(
        meeting(
            prop::geom::elliptic_cylinder(),
            prop_oneof![
                prop::geom::cylinder(),
                prop::geom::elliptic_cylinder(),
                prop::geom::cone(),
                prop::geom::sphere(),
            ],
        ),
        holds,
    );
}

/// The branch parameter is smooth through a turning point: the chord
/// over a small step keeps its length on both sides of every joint.
#[test]
fn a_branch_runs_at_a_steady_speed_through_its_turning_points() {
    let (main, branch) = pipes(1.5);
    let trace = trace_quadrics(
        &main,
        &branch,
        &cube(5.0),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let b = &trace.branches()[0];
    let n = 2000;
    let step = b.domain().length() / n as f64;
    let speeds: Vec<f64> = (0..n)
        .map(|i| (b.point((i + 1) as f64 * step) - b.point(i as f64 * step)).norm() / step)
        .collect();
    for w in speeds.windows(2) {
        assert!(w[0] > 0.0 && (w[1] / w[0] - 1.0).abs() < 0.02, "{w:?}");
    }
}

/// A cylinder or a cone, a point of it on its first nappe, and a sphere
/// tangent to it there from inside to within `gap`: its radius `factor`
/// times the radius of curvature across the rulings, so above one the
/// section crosses itself at the point and below one the sphere touches
/// and stays inside.
fn touching(
    factor: core::ops::RangeInclusive<f64>,
    gap: core::ops::RangeInclusive<f64>,
) -> impl Strategy<Value = (Surface, Surface, Point3, Aabb)> {
    (
        prop_oneof![prop::geom::cylinder(), prop::geom::cone()],
        prop::finite_f64(0.0..=TAU),
        prop::finite_f64(0.0..=5.0),
        prop::finite_f64(factor),
        prop::finite_f64(gap),
    )
        .prop_map(|(ruled, u, v, factor, gap)| {
            let across = match ruled {
                Surface::Cone {
                    radius, half_angle, ..
                } => (radius + v * half_angle.sin()) / half_angle.cos(),
                _ => size(&ruled),
            };
            let at = ruled.point(u, v);
            let normal = ruled.normal(u, v).unwrap().into_inner();
            let radius = factor * across;
            let ball = Surface::Sphere {
                frame: Frame::world().with_origin(at - normal * (radius + gap)),
                radius,
            };
            (
                ruled,
                ball,
                at,
                Aabb::of_point(at).inflated(60.0 + 4.0 * radius),
            )
        })
}

/// A crowded singularity is a refusal the tracer is entitled to, and a
/// pose of measure zero; every other error fails the case.
fn traced(a: &Surface, b: &Surface, within: &Aabb) -> Result<Option<SectionTrace>, TestCaseError> {
    match trace_quadrics(a, b, within, tol(), &mut arris_math::Meter::default()) {
        Ok(trace) => Ok(Some(trace)),
        Err(GeomError::DegenerateSection {
            fault: SectionFault::CrowdedSingularity,
            ..
        }) => Ok(None),
        Err(e) => Err(TestCaseError::fail(format!("refused: {e}"))),
    }
}

#[test]
fn a_larger_sphere_tangent_within_the_tolerance_crosses_at_one_singular_point() {
    let half = 0.5 * tol().linear;
    check(
        touching(1.3..=3.0, -half..=half),
        |(ruled, ball, at, within)| {
            let Some(trace) = traced(&ruled, &ball, &within)? else {
                return Ok(());
            };
            prop_assert_eq!(trace.points().len(), 1);
            let node = trace.points()[0];
            prop_assert!(!node.isolated);
            // A tangency's place is conditioned as the square root of the gap.
            let reach = (2.0 * tol().linear * size(&ball)).sqrt();
            prop_assert!(
                (node.point - at).norm() <= 4.0 * reach,
                "{} from {at}",
                node.point
            );
            let mut ends = 0;
            for b in trace.branches() {
                for (k, end) in b.ends().into_iter().flatten().enumerate() {
                    if end == BranchEnd::Singular(0) {
                        ends += 1;
                        let t = if k == 0 {
                            b.domain().lo()
                        } else {
                            b.domain().hi()
                        };
                        prop_assert_eq!(b.point(t), node.point);
                        // …and the branch runs into its end, it does not jump.
                        let near = b.domain().lerp(if k == 0 { 1e-9 } else { 1.0 - 1e-9 });
                        prop_assert!((b.point(near) - node.point).norm() < 1e-6);
                    }
                }
            }
            prop_assert_eq!(ends, 4);
            let scale = 1.0 + at.coords.norm();
            let off = worst_off(&trace, &ruled, &ball);
            prop_assert!(off <= tol().linear + 1e-11 * scale * scale, "{off}");
            Ok(())
        },
    );
}

#[test]
fn a_smaller_sphere_tangent_within_the_tolerance_touches_at_one_isolated_point() {
    let half = 0.5 * tol().linear;
    check(
        touching(0.3..=0.8, -half..=half),
        |(ruled, ball, at, within)| {
            let Some(trace) = traced(&ruled, &ball, &within)? else {
                return Ok(());
            };
            prop_assert!(trace.branches().is_empty());
            prop_assert_eq!(trace.points().len(), 1);
            prop_assert!(trace.points()[0].isolated);
            let reach = (2.0 * tol().linear * size(&ball)).sqrt();
            prop_assert!((trace.points()[0].point - at).norm() <= 4.0 * reach);
            Ok(())
        },
    );
}

/// Twenty tolerances or more either way the same poses are decided exactly:
/// nothing singular, the larger sphere's section in two or three loops
/// and the smaller sphere's empty or one small loop.
#[test]
fn a_sphere_tangent_outside_the_tolerance_is_not_singular() {
    let far = 20.0 * tol().linear;
    for gap in [-2.0 * far..=-far, far..=2.0 * far] {
        check(touching(0.3..=3.0, gap), |(ruled, ball, _, within)| {
            let Some(trace) = traced(&ruled, &ball, &within)? else {
                return Ok(());
            };
            prop_assert!(trace.points().is_empty());
            prop_assert!(trace.branches().iter().all(SectionBranch::is_closed));
            Ok(())
        });
    }
}

/// How far each of `n` points along `branch` moves at the next float but
/// a few, and how far that is outside its stretch
/// (`SectionBranch::distance`): the largest of each.
fn noise_and_stretch(branch: &SectionBranch, n: usize, dt: f64) -> (f64, f64) {
    let domain = branch.domain();
    (1..n).fold((0.0f64, 0.0f64), |(raw, beyond), i| {
        let t = domain.lerp(i as f64 / n as f64);
        let (p, q) = (branch.point(t), branch.point(t + dt));
        (
            raw.max((q - p).norm()),
            beyond.max(branch.distance(t + dt, p)),
        )
    })
}

/// Two rods tangent along a ruling, the second turned a quarter of the
/// tolerance about the contact's middle: the section runs along the
/// rulings, and the root on each ruling moves along it by `10⁻⁷` between
/// neighbouring floats of `s`, though it stays on both surfaces — the
/// ruling is `10⁻⁹` of a radian from tangent to the other rod. The fit is
/// held to the stretch of the ruling `f64` does not decide
/// (`SectionBranch::distance`, ADR-0022), within which neighbouring points
/// agree to rounding, and not to the point: held to the point, it ran out
/// of spans.
#[test]
fn rods_a_hair_off_parallel_are_known_along_their_rulings_to_rounding() {
    let (ra, rb, length) = (1.338253251879201, 1.594912540100717, 16.10031287625863);
    let turn = -2.0 * 2.5e-8 / length;
    let pivot = Vec3::new(0.0, ra, 0.5 * length);
    let (sin, cos) = turn.sin_cos();
    let spin = |v: Vec3| Vec3::new(v.x, v.y * cos - v.z * sin, v.y * sin + v.z * cos);
    let origin = pivot + spin(Vec3::new(0.0, ra + rb, -6.49554150205149) - pivot);
    let a = cylinder([0.0; 3], [0.0, 0.0, 1.0], ra);
    let b = cylinder(origin.into(), spin(Vec3::z()).into(), rb);
    let within = Aabb {
        min: [-3.0, -3.0, -1.0],
        max: [5.0, 5.0, 17.0],
    };
    let trace = trace_quadrics(&a, &b, &within, tol(), &mut arris_math::Meter::default()).unwrap();
    assert_eq!(trace.branches().len(), 1);
    let branch = &trace.branches()[0];
    let (raw, beyond) = noise_and_stretch(branch, 200_000, 2e-15);
    assert!(raw > 1e-8, "the root moves along the ruling by {raw}");
    assert!(beyond < 1e-12, "a point {beyond} outside its stretch");
    let Ok(arris_geom::SurfaceIntersection::Meets { curves, .. }) =
        arris_geom::intersect_surfaces(&a, &b, &within, tol(), &mut arris_math::Meter::default())
    else {
        panic!("the section is fitted");
    };
    let [fit] = curves.as_slice() else {
        panic!("one curve");
    };
    let Curve::Nurbs(fit) = &fit.curve else {
        panic!("a fitted curve");
    };
    for i in 0..=2000 {
        let t = branch.domain().lerp(i as f64 / 2000.0);
        let off = branch.distance(t, fit.eval(t).point);
        assert!(
            off <= arris_geom::SECTION_FIT_FRACTION * tol().linear,
            "the fit is {off} from its branch at {t}"
        );
    }
}
