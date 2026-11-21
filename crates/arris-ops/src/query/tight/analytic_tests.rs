use core::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, TAU};

use arris_check::{Level, check};
use arris_debug::sample;
use arris_geom::{Profile, ProfileLoop, ProfileSegment, Surface, SurfaceKind};
use arris_math::{Aabb, Axis, Frame, Interval, Point2, Point3, Reflection, Vec2, Vec3, real};
use arris_topo::entity::{Body as BodyEntity, BodyKind, Coedge, Face, Loop, Shell};
use arris_topo::{Body, Face as FaceHandle, FaceId, Model, Shell as ShellHandle};

use super::tight_bounds;
use crate::{Control, cut, mirror, revolve};

fn face_of(m: &Model, body: Body, kind: SurfaceKind) -> FaceId {
    let report = check(m, body, Level::Full);
    assert!(report.is_ok(), "{report:?}");
    let faces = m.faces(body).unwrap();
    let mut matching = faces.into_iter().filter(|f| {
        let face = m.face(f.id).unwrap();
        m.surface(face.surface()).unwrap().kind() == kind
    });
    let face = matching.next().expect("the analytic face");
    assert!(matching.next().is_none());
    face.id
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
fn tight_bounds_of_sphere_patch_in_translated_charts() {
    let mut m = Model::default();
    let eps = m.precision().default_tolerance;
    let half_angle = 0.3;
    let (s, c) = real::sin_cos(half_angle);
    let v = Interval::new(-half_angle, half_angle).unwrap();
    for shift in [0.0, 2.0 * TAU, -3.0 * TAU] {
        let u = Interval::new(shift - half_angle, shift + half_angle).unwrap();
        let surface = Surface::Sphere {
            frame: Frame::world(),
            radius: 1.0,
        };
        let body = sample::patch(&mut m, surface, u, v).unwrap();
        let report = check(&m, body, Level::Full);
        assert!(report.is_ok(), "{report:?}");
        let got = tight_bounds(&m, body.id.into(), &Control::NONE)
            .unwrap()
            .unwrap();
        assert_box(got, [c * c, -s, -s], [1.0, s, s], eps);
    }
}

#[test]
fn tight_bounds_follow_whole_period_joins_between_pcurves() {
    let mut m = Model::default();
    let eps = m.precision().default_tolerance;
    let source = sample::patch(
        &mut m,
        Surface::Sphere {
            frame: Frame::world(),
            radius: 1.0,
        },
        Interval::new(0.25, 0.5).unwrap(),
        Interval::new(-0.3, 0.3).unwrap(),
    )
    .unwrap();
    let face = m.face(m.faces(source).unwrap()[0].id).unwrap();
    let surface = face.surface();
    let mut coedges = face.loops()[0].coedges().to_vec();
    // Write the second pcurve one turn from its neighbors, with the same
    // shared edge. Both joins represent the same UV points on the sphere.
    let use_ = coedges[1];
    let shifted = m
        .curve2(use_.pcurve())
        .unwrap()
        .translated(Vec2::new(TAU, 0.0));
    let pcurve = m.add_curve2(shifted);
    coedges[1] = Coedge::new(use_.edge(), use_.orientation(), pcurve);
    let mut raw = m.raw();
    let face = raw.add_face(Face::new(surface, vec![Loop::new(coedges)], eps));
    let shell = raw.add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let body = Body::forward(raw.add_body(BodyEntity::new(
        BodyKind::Sheet,
        vec![ShellHandle::forward(shell)],
        vec![],
        vec![],
    )));
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}");

    let got = tight_bounds(&m, body.id.into(), &Control::NONE)
        .unwrap()
        .unwrap();
    let (s, c) = real::sin_cos(0.3);
    assert_box(
        got,
        [real::cos(0.5) * c, real::sin(0.25) * c, -s],
        [real::cos(0.25), real::sin(0.5), s],
        eps,
    );
}

#[test]
fn tight_bounds_of_torus_patch_in_translated_and_reflected_charts() {
    let mut m = Model::default();
    let eps = m.precision().default_tolerance;
    let reflection = Reflection::new(Point3::origin(), Vec3::x()).unwrap();
    let half_angle = 0.3;
    let (s, c) = real::sin_cos(half_angle);
    let x_min = (3.0 + c) * c;
    for (du, dv) in [(0.0, 0.0), (2.0 * TAU, 0.0), (0.0, -3.0 * TAU)] {
        let u = Interval::new(du - half_angle, du + half_angle).unwrap();
        let v = Interval::new(dv - half_angle, dv + half_angle).unwrap();
        let surface = Surface::Torus {
            frame: Frame::world(),
            major_radius: 3.0,
            minor_radius: 1.0,
        };
        let body = sample::patch(&mut m, surface, u, v).unwrap();
        let (image, _) = mirror(&mut m, body, &reflection, &Control::NONE).unwrap();
        for (patch, min_x, max_x) in [(body, x_min, 4.0), (image, -4.0, -x_min)] {
            let report = check(&m, patch, Level::Full);
            assert!(report.is_ok(), "{report:?}");
            let got = tight_bounds(&m, patch.id.into(), &Control::NONE)
                .unwrap()
                .unwrap();
            assert_box(got, [min_x, -4.0 * s, -s], [max_x, 4.0 * s, s], eps);
        }
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
    let face = face_of(&m, body, SurfaceKind::Sphere);
    let b = tight_bounds(&m, face.into(), &Control::NONE)
        .unwrap()
        .unwrap();
    assert_box(
        b,
        [-2.0; 3],
        [2.0, 2.0, 1.0],
        m.precision().default_tolerance,
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
        let face = face_of(&m, body, SurfaceKind::Torus);
        let b = tight_bounds(&m, face.into(), &Control::NONE)
            .unwrap()
            .unwrap();
        assert_box(b, min, max, m.precision().default_tolerance);
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
    let face = face_of(&m, body, SurfaceKind::Cone);
    assert!(
        m.face(face)
            .unwrap()
            .loops()
            .iter()
            .flat_map(|l| l.coedges())
            .any(|c| {
                let edge = m.edge(c.edge()).unwrap();
                edge.is_degenerate()
                    && m.vertex(edge.start()).unwrap().point() == Point3::new(0.0, 0.0, 4.0)
            })
    );
    let b = tight_bounds(&m, face.into(), &Control::NONE)
        .unwrap()
        .unwrap();
    assert_box(
        b,
        [-2.0, -2.0, 0.0],
        [2.0, 2.0, 4.0],
        m.precision().default_tolerance,
    );
}
