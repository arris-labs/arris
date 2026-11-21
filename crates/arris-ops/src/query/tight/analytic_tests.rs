use core::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, TAU};

use arris_check::{Level, check, domain::FaceDomain};
use arris_debug::sample;
use arris_geom::{Profile, ProfileLoop, ProfileSegment, SurfaceKind};
use arris_math::{Aabb, Axis, Frame, Point2, Point3, Vec3};
use arris_topo::{Body, Model};

use super::analytic_face_extrema;
use crate::{Control, cut, revolve};

fn face_domain(m: &Model, body: Body, kind: SurfaceKind) -> FaceDomain<'_> {
    let report = check(m, body, Level::Full);
    assert!(report.is_ok(), "{report:?}");
    let faces = m.faces(body).unwrap();
    let mut matching = faces.into_iter().filter(|f| {
        let face = m.face(f.id).unwrap();
        m.surface(face.surface()).unwrap().kind() == kind
    });
    let face = matching.next().expect("the analytic face");
    assert!(matching.next().is_none());
    FaceDomain::of(m, face.id, m.precision().parametric_tolerance).unwrap()
}

fn boundary_box(m: &Model, domain: &FaceDomain<'_>) -> Aabb {
    domain
        .edges()
        .iter()
        .map(|&id| {
            let edge = m.edge(id).unwrap();
            let ends = Aabb::of_point(m.vertex(edge.start()).unwrap().point())
                .union(Aabb::of_point(m.vertex(edge.end()).unwrap().point()));
            match edge.curve() {
                Some((curve, range)) => ends.union(m.curve(curve).unwrap().bounds(range).unwrap()),
                None => ends,
            }
        })
        .reduce(Aabb::union)
        .expect("a closed face boundary")
}

fn assert_box(got: Aabb, min: [f64; 3], max: [f64; 3], eps: f64) {
    for (actual, expected) in got
        .min
        .into_iter()
        .chain(got.max)
        .zip(min.into_iter().chain(max))
    {
        assert!(
            (actual - expected).abs() <= eps,
            "{got:?}, expected {min:?}..{max:?}"
        );
    }
}

#[test]
fn analytic_extrema_keep_only_the_uncut_ball_pole() {
    let mut m = Model::default();
    let sphere = sample::sphere(&mut m, Point3::origin(), 2.0).unwrap();
    let tool = sample::cuboid(
        &mut m,
        Point3::new(-3.0, -3.0, 1.0),
        Point3::new(3.0, 3.0, 3.0),
    )
    .unwrap();
    let (body, _) = cut(&mut m, sphere, tool, &Control::NONE).unwrap();
    let domain = face_domain(&m, body, SurfaceKind::Sphere);
    let interior = analytic_face_extrema(&domain).expect("the surviving pole and equator");
    let eps = m.precision().default_tolerance;
    assert_box(interior, [-2.0; 3], [2.0, 2.0, 0.0], eps);
    assert_box(
        boundary_box(&m, &domain).union(interior),
        [-2.0; 3],
        [2.0, 2.0, 1.0],
        eps,
    );
}

#[test]
fn analytic_extrema_find_quarter_torus_crests_and_outer_equator_about_z_and_x() {
    let r = FRAC_1_SQRT_2;
    // The turn covers −π/4 to π/4 about its middle radial direction.
    // The outer equator at that middle direction extends beyond both end caps.
    for (axis, start, min, max) in [
        (
            Vec3::z(),
            Vec3::new(r, -r, 0.0),
            [2.0 * r, -4.0 * r, -1.0],
            [4.0, 4.0 * r, 1.0],
        ),
        (
            Vec3::x(),
            Vec3::new(0.0, r, -r),
            [-1.0, 2.0 * r, -4.0 * r],
            [1.0, 4.0, 4.0 * r],
        ),
    ] {
        let mut m = Model::default();
        let origin = Point3::origin();
        let profile = Profile {
            plane: Frame::from_orthonormal(origin, start, axis, start.cross(&axis)).unwrap(),
            outer: ProfileLoop::Circle {
                center: Point2::new(3.0, 0.0),
                radius: 1.0,
            },
            holes: vec![],
        };
        let (body, _) = revolve(
            &mut m,
            &profile,
            Axis::new(origin, axis).unwrap(),
            FRAC_PI_2,
            &Control::NONE,
        )
        .unwrap();
        let domain = face_domain(&m, body, SurfaceKind::Torus);
        let interior = analytic_face_extrema(&domain).expect("the crests and mid-turn extrema");
        let boundary = boundary_box(&m, &domain);
        let eps = m.precision().default_tolerance;
        let (axial, radial) = if axis == Vec3::z() { (2, 0) } else { (0, 1) };
        assert!((interior.min[axial] + 1.0).abs() <= eps);
        assert!((interior.max[axial] - 1.0).abs() <= eps);
        assert!((interior.min[radial] - 2.0).abs() <= eps);
        assert!((interior.max[radial] - 4.0).abs() <= eps);
        assert_box(boundary.union(interior), min, max, eps);
    }
}

#[test]
fn analytic_extrema_leave_the_cone_apex_to_the_boundary() {
    let mut m = Model::default();
    let origin = Point3::origin();
    let profile = Profile {
        plane: Frame::from_orthonormal(origin, Vec3::x(), Vec3::z(), -Vec3::y()).unwrap(),
        outer: ProfileLoop::Path {
            start: Point2::origin(),
            segments: vec![
                ProfileSegment::LineTo(Point2::new(2.0, 0.0)),
                ProfileSegment::LineTo(Point2::new(0.0, 4.0)),
                ProfileSegment::LineTo(Point2::origin()),
            ],
        },
        holes: vec![],
    };
    let (body, _) = revolve(&mut m, &profile, Axis::z_at(origin), TAU, &Control::NONE).unwrap();
    let domain = face_domain(&m, body, SurfaceKind::Cone);
    assert!(analytic_face_extrema(&domain).is_none());
    assert!(domain.edges().iter().any(|&id| {
        let edge = m.edge(id).unwrap();
        edge.is_degenerate()
            && m.vertex(edge.start()).unwrap().point() == Point3::new(0.0, 0.0, 4.0)
    }));
    assert_box(
        boundary_box(&m, &domain),
        [-2.0, -2.0, 0.0],
        [2.0, 2.0, 4.0],
        m.precision().default_tolerance,
    );
}
