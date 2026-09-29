//! `measure::mass_properties` (`docs/ARCHITECTURE.md` §Operations):
//! the closed forms of the box, the cylinder and the sphere, the hand-built
//! frame against the oracle's reading of `boolean/frame-cut`, the
//! parallel-axis theorem against a direct integral, and the typed errors
//! for a body that is not a solid and one that fails the checker.

use arris_debug::unmetered::{mass_properties, primitive_box, primitive_cylinder};
use core::f64::consts::PI;

use arris_debug::testing::close_to;
use arris_debug::{fixtures, prop, sample};
use arris_ops::arris_check::arris_topo::arris_geom::Surface;
use arris_ops::arris_check::arris_topo::arris_math::{
    Axis, Frame, Interval, Matrix3, Point2, Point3, Vec3,
};

use arris_ops::arris_check::arris_topo::entity::Body as BodyEntity;
use arris_ops::arris_check::arris_topo::{Body, Model, Shell as ShellHandle, ShellId};
use arris_ops::measure::MassProperties;
use arris_ops::{OpError, Reason};
use proptest::prelude::*;

/// [`arris_debug::testing::close_to`] at a floor of `1.0`.
fn close(a: f64, b: f64, rel: f64) -> bool {
    close_to(a, b, 1.0, rel)
}

fn assert_matrix(found: Matrix3, expected: Matrix3, rel: f64, what: &str) {
    let scale = expected.abs().max().max(1.0);
    for i in 0..3 {
        for j in 0..3 {
            let (f, e) = (found[(i, j)], expected[(i, j)]);
            assert!(
                (f - e).abs() <= rel * scale,
                "{what}: [{i}, {j}] is {f}, not {e}\n{found}{expected}"
            );
        }
    }
}

/// The inertia tensor of a box `min`–`max` about the origin, in closed
/// form and without the parallel-axis theorem: `∫ y² dV` over a box is
/// `(x₁ − x₀)(z₁ − z₀)(y₁³ − y₀³) / 3` and `∫ x y dV` is
/// `(z₁ − z₀)(x₁² − x₀²)(y₁² − y₀²) / 4`.
fn box_inertia_about_origin(min: Point3, max: Point3) -> Matrix3 {
    let d = max - min;
    let volume = d.x * d.y * d.z;
    let second = Vec3::from_iterator(
        (0..3).map(|i| volume / d[i] * (max[i].powi(3) - min[i].powi(3)) / 3.0),
    );
    let product = Vec3::from_iterator((0..3).map(|i| {
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;
        d[k] * (max[i].powi(2) - min[i].powi(2)) * (max[j].powi(2) - min[j].powi(2)) / 4.0
    }));
    Matrix3::new(
        second.y + second.z,
        -product.x,
        -product.z,
        -product.x,
        second.x + second.z,
        -product.y,
        -product.z,
        -product.y,
        second.x + second.y,
    )
}

/// The closed forms of an axis-aligned box: volume, area, centroid and
/// the diagonal inertia `m (b² + c²) / 12` about the centroid.
fn box_properties(min: Point3, max: Point3) -> MassProperties {
    let d = max - min;
    let volume = d.x * d.y * d.z;
    let area = 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
    let diagonal = Vec3::from_iterator((0..3).map(|i| {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        volume * (d[j] * d[j] + d[k] * d[k]) / 12.0
    }));
    MassProperties {
        volume,
        area,
        centroid: Point3::from((min.coords + max.coords) / 2.0),
        inertia: Matrix3::from_diagonal(&diagonal),
    }
}

/// The closed forms of a cylinder of `radius` and `height` on `axis`: a
/// symmetric top, `m r² / 2` about its own axis and `m (3 r² + h²) / 12`
/// across it, so the tensor is `I⊥ (I₃ − d dᵀ) + I‖ d dᵀ`.
fn cylinder_properties(axis: &Axis, radius: f64, height: f64) -> MassProperties {
    let volume = PI * radius * radius * height;
    let d = axis.direction.into_inner();
    let along = volume * radius * radius / 2.0;
    let across = volume * (3.0 * radius * radius + height * height) / 12.0;
    MassProperties {
        volume,
        area: 2.0 * PI * radius * (radius + height),
        centroid: axis.origin + d * (height / 2.0),
        inertia: across * (Matrix3::identity() - d * d.transpose()) + along * d * d.transpose(),
    }
}

fn assert_properties(found: &MassProperties, expected: &MassProperties, rel: f64, what: &str) {
    assert!(
        close(found.volume, expected.volume, rel),
        "{what}: volume {}, not {}",
        found.volume,
        expected.volume
    );
    assert!(
        close(found.area, expected.area, rel),
        "{what}: area {}, not {}",
        found.area,
        expected.area
    );
    let scale = expected.centroid.coords.abs().max().max(1.0);
    assert!(
        (found.centroid - expected.centroid).norm() <= rel * scale,
        "{what}: centroid {}, not {}",
        found.centroid,
        expected.centroid
    );
    assert_matrix(found.inertia, expected.inertia, rel, what);
}

#[test]
fn the_box_measures_its_closed_forms() {
    let mut m = Model::default();
    let (min, max) = (
        Point3::new(-20.0, -15.0, -5.0),
        Point3::new(20.0, 15.0, 5.0),
    );
    let (body, _) = primitive_box(&mut m, min, max).unwrap();
    let p = mass_properties(&m, body).unwrap();
    assert_properties(&p, &box_properties(min, max), 1e-12, "the centred box");
    // Symmetric about all three planes: every product of inertia is zero,
    // and the integration leaves nothing but rounding of the terms it
    // cancelled.
    let noise = 1e-12 * p.inertia.abs().max();
    for (i, j) in [(0, 1), (1, 2), (0, 2)] {
        assert!(
            p.inertia[(i, j)].abs() <= noise,
            "[{i}, {j}]\n{}",
            p.inertia
        );
        assert_eq!(p.inertia[(i, j)], p.inertia[(j, i)], "not symmetric");
    }
}

#[test]
fn an_offset_box_carries_its_centroid_and_its_tensor() {
    let mut m = Model::default();
    let (min, max) = (Point3::new(0.0, 0.0, 0.0), Point3::new(40.0, 30.0, 10.0));
    let (body, _) = primitive_box(&mut m, min, max).unwrap();
    let p = mass_properties(&m, body).unwrap();
    assert_properties(&p, &box_properties(min, max), 1e-12, "the offset box");
    // `inertia_about` against the closed form of the tensor about the
    // origin, integrated directly rather than carried by the theorem.
    assert_matrix(
        p.inertia_about(Point3::origin()),
        box_inertia_about_origin(min, max),
        1e-12,
        "the box about the origin",
    );
    // And back to the centroid.
    assert_matrix(
        p.inertia_about(p.centroid),
        p.inertia,
        1e-12,
        "the box about its centroid",
    );
}

#[test]
fn the_cylinder_measures_its_closed_forms() {
    let mut m = Model::default();
    let axis = Axis::z_at(Point3::origin());
    let (body, _) = primitive_cylinder(&mut m, axis, 4.0, 12.0).unwrap();
    let p = mass_properties(&m, body).unwrap();
    assert_properties(
        &p,
        &cylinder_properties(&axis, 4.0, 12.0),
        1e-12,
        "the cylinder",
    );
}

#[test]
fn the_sphere_measures_its_closed_forms() {
    let mut m = Model::default();
    let (centre, radius) = (Point3::new(3.0, -2.0, 1.0), 2.5);
    let body = sample::sphere(&mut m, centre, radius).unwrap();
    let p = mass_properties(&m, body).unwrap();
    let volume = 4.0 * PI * radius.powi(3) / 3.0;
    let expected = MassProperties {
        volume,
        area: 4.0 * PI * radius * radius,
        centroid: centre,
        inertia: Matrix3::identity() * (2.0 * volume * radius * radius / 5.0),
    };
    assert_properties(&p, &expected, 1e-12, "the sphere");
}

#[test]
fn the_frame_measures_what_the_oracle_read() {
    let mut m = Model::default();
    let body = sample::frame(
        &mut m,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap();
    let p = mass_properties(&m, body).unwrap();
    let fixture = fixtures::load(&fixtures::corpus_root().join("boolean/frame-cut")).unwrap();
    let expected = &fixture.expected.results["default"];
    assert!(close(p.volume, expected.volume.unwrap(), 1e-9), "{p:?}");
    assert!(close(p.area, expected.area.unwrap(), 1e-9), "{p:?}");
    let centroid = Point3::from(Vec3::from_row_slice(&expected.centroid.unwrap()));
    assert!((p.centroid - centroid).norm() <= 1e-9 * centroid.coords.abs().max());
    // The frame is the plate less its window: the difference of two boxes
    // about the shared centroid.
    let plate = box_properties(Point3::origin(), Point3::new(40.0, 30.0, 10.0));
    let window = box_properties(Point3::new(10.0, 10.0, 0.0), Point3::new(30.0, 20.0, 10.0));
    assert_matrix(
        p.inertia,
        plate.inertia - window.inertia,
        1e-12,
        "the frame's tensor",
    );
}

#[test]
fn a_body_that_is_not_a_solid_is_degenerate() {
    let mut m = Model::default();
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    let sheet = sample::patch(
        &mut m,
        plane,
        Interval::new(0.0, 2.0).unwrap(),
        Interval::new(0.0, 3.0).unwrap(),
    )
    .unwrap();
    let err = mass_properties(&m, sheet).unwrap_err();
    assert!(
        matches!(
            err,
            OpError::Degenerate {
                reason: Reason::NotSolid,
                ..
            }
        ),
        "{err}"
    );
    assert!(err.to_string().contains("the body is not a solid"), "{err}");
}

#[test]
fn a_body_that_does_not_resolve_is_not_found() {
    let m = Model::default();
    let body = Body::forward(arris_ops::arris_check::arris_topo::BodyId::new(3, 0));
    assert!(matches!(
        mass_properties(&m, body),
        Err(OpError::NotFound(_))
    ));
}

#[cfg(debug_assertions)]
#[test]
fn a_broken_body_is_invalid_input_in_a_debug_build() {
    let mut m = Model::default();
    // A solid whose one shell does not exist: the checker's M1.
    let body = m
        .raw()
        .add_body(BodyEntity::solid(vec![ShellHandle::forward(ShellId::new(
            9, 0,
        ))]));
    let err = mass_properties(&m, Body::forward(body)).unwrap_err();
    assert!(matches!(err, OpError::InvalidInput { .. }), "{err}");
    assert!(err.to_string().contains("fails the checker"));
}

#[test]
fn random_boxes_measure_their_closed_forms() {
    let strategy = (
        prop::point_in_box(prop::DEFAULT_SCALE),
        prop::radius(0.1..=20.0),
        prop::radius(0.1..=20.0),
        prop::radius(0.1..=20.0),
    );
    prop::check(strategy, |(min, dx, dy, dz)| {
        let max = min + Vec3::new(dx, dy, dz);
        let mut m = Model::default();
        let (body, _) = primitive_box(&mut m, min, max).unwrap();
        let p = mass_properties(&m, body).unwrap();
        let expected = box_properties(min, max);
        prop_assert!(close(p.volume, expected.volume, 1e-12), "{p:?}");
        prop_assert!(close(p.area, expected.area, 1e-12), "{p:?}");
        prop_assert!(
            (p.centroid - expected.centroid).norm()
                <= 1e-12 * expected.centroid.coords.abs().max().max(1.0)
        );
        let scale = expected.inertia.abs().max();
        prop_assert!(
            (p.inertia - expected.inertia).abs().max() <= 1e-12 * scale,
            "{}\n{}",
            p.inertia,
            expected.inertia
        );
        Ok(())
    });
}

#[test]
fn random_cylinders_measure_their_closed_forms_in_any_pose() {
    let strategy = (
        prop::point_in_box(prop::DEFAULT_SCALE),
        prop::unit_vec3(),
        prop::radius(0.1..=20.0),
        prop::radius(0.1..=20.0),
    );
    prop::check(strategy, |(origin, direction, radius, height)| {
        let axis = Axis::new(origin, direction.into_inner()).unwrap();
        let mut m = Model::default();
        let (body, _) = primitive_cylinder(&mut m, axis, radius, height).unwrap();
        let p = mass_properties(&m, body).unwrap();
        let expected = cylinder_properties(&axis, radius, height);
        prop_assert!(close(p.volume, expected.volume, 1e-12), "{p:?}");
        prop_assert!(close(p.area, expected.area, 1e-12), "{p:?}");
        prop_assert!(
            (p.centroid - expected.centroid).norm()
                <= 1e-12 * expected.centroid.coords.abs().max().max(1.0)
        );
        let scale = expected.inertia.abs().max();
        prop_assert!(
            (p.inertia - expected.inertia).abs().max() <= 1e-12 * scale,
            "{}\n{}",
            p.inertia,
            expected.inertia
        );
        Ok(())
    });
}
