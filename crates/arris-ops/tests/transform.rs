//! `ops::transform` (`docs/ARCHITECTURE.md` §Operations): clean at `Full`,
//! one `Modified` per entity and nothing else, the identity motion gives
//! new ids over the same shape, a motion then its inverse returns every
//! vertex, mass properties are covariant, and two runs are identical.

use arris_check::{Level, check};
use arris_debug::testing::entities_of;
use arris_debug::unmetered::{mass_properties, primitive_box, primitive_cylinder, transform};
use arris_debug::{dump_text, prop};
use arris_math::{Axis, Isometry, Point3};
use arris_topo::{Body, Model};
use proptest::prelude::*;

fn the_cylinder(m: &mut Model) -> Body {
    primitive_cylinder(m, Axis::z_at(Point3::origin()), 4.0, 12.0)
        .unwrap()
        .0
}

fn a_pose() -> Isometry {
    Isometry::new(
        arris_math::nalgebra::UnitQuaternion::from_axis_angle(
            &arris_math::UnitVec3::new_normalize(arris_math::Vec3::new(1.0, 1.0, 0.0)),
            core::f64::consts::FRAC_PI_6,
        ),
        arris_math::Vec3::new(10.0, -5.0, 3.0),
    )
}

/// A body of two shells — two boxes apart, assembled from their own faces
/// — moves shell by shell: the two shells in the stored order, each
/// `Modified` from the one it moved and holding as many faces, every
/// entity `Modified` exactly once, and the volume both boxes enclose.
#[test]
fn a_body_of_two_shells_moves_shell_by_shell() {
    use arris_topo::builder::{Assembly, Builder, FaceSpec};
    use arris_topo::entity::BodyKind;
    let mut m = Model::default();
    let (a, _) = primitive_box(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let (b, _) = primitive_box(
        &mut m,
        Point3::new(5.0, 0.0, 0.0),
        Point3::new(6.0, 2.0, 1.0),
    )
    .unwrap();
    let shells = [a, b]
        .iter()
        .map(|&body| {
            m.faces(body)
                .unwrap()
                .into_iter()
                .map(FaceSpec::Keep)
                .collect()
        })
        .collect();
    let assembly = Assembly {
        shells,
        ..Assembly::default()
    };
    let body = Builder::assemble(&m, m.precision().default_tolerance, assembly)
        .unwrap()
        .0
        .finish(&mut m, BodyKind::Solid)
        .unwrap()
        .body;

    let (moved, provenance) = transform(&mut m, body, &a_pose()).unwrap();
    assert!(check(&m, moved, Level::Fast).is_ok());
    let (before, after) = (m.shells(body).unwrap(), m.shells(moved).unwrap());
    assert_eq!(after.len(), 2);
    for (old, new) in before.iter().zip(&after) {
        assert_eq!(provenance.modified_from(old.shape()), [new.shape()]);
        assert_eq!(
            m.shell(old.id).unwrap().faces().len(),
            m.shell(new.id).unwrap().faces().len()
        );
    }
    let (inputs, outputs) = (entities_of(&m, body), entities_of(&m, moved));
    assert_eq!(inputs.len(), outputs.len());
    for e in inputs {
        assert_eq!(provenance.modified_from(e).len(), 1, "{e}");
    }
    let volume = mass_properties(&m, moved).unwrap().volume;
    assert!((volume - 3.0).abs() < 1e-12, "{volume}");
}

#[test]
fn a_transformed_cylinder_is_clean_at_full_and_one_to_one_modified() {
    let mut m = Model::default();
    let body = the_cylinder(&mut m);
    let motion = a_pose();
    let (moved, p) = transform(&mut m, body, &motion).unwrap();
    let report = check(&m, moved, Level::Full);
    assert!(
        report.is_ok(),
        "{report}\n{}",
        dump_text(&m, moved).unwrap()
    );
    assert!(report.unchecked().is_empty());

    let old = entities_of(&m, body);
    let new = entities_of(&m, moved);
    assert_eq!(old.len(), new.len());
    assert_eq!(p.deleted().count(), 0, "{p}");
    assert_eq!(p.origins_recorded().count(), old.len(), "{p}");
    assert_eq!(p.outputs(), new, "{p}");
    for &e in &old {
        let images = p.modified_from(e);
        assert_eq!(images.len(), 1, "{e}: {images:?}\n{p}");
        assert!(new.contains(&images[0]), "{e} -> {}", images[0]);
        assert!(p.generated_from(e).is_empty(), "{e}");
    }
    for &e in &new {
        let origins = p.origins(e);
        assert_eq!(origins.len(), 1, "{e}: {origins:?}\n{p}");
        assert_eq!(origins[0].0, arris_topo::Relation::Modified, "{e}");
    }
}

#[test]
fn the_identity_motion_gives_new_ids_over_the_same_shape() {
    let mut m = Model::default();
    let body = the_cylinder(&mut m);
    let (moved, _) = transform(&mut m, body, &Isometry::identity()).unwrap();
    assert_ne!(body, moved);
    let strip = |t: String| -> String {
        t.lines()
            .filter(|l| !l.starts_with("body"))
            .map(|l| {
                l.chars()
                    .filter(|c| !c.is_ascii_digit())
                    .collect::<String>()
            })
            .collect::<Vec<_>>()
            .join("\n")
    };
    assert_eq!(
        strip(dump_text(&m, body).unwrap()),
        strip(dump_text(&m, moved).unwrap())
    );
}

#[test]
fn a_motion_then_its_inverse_returns_every_vertex() {
    let mut m = Model::default();
    let body = the_cylinder(&mut m);
    let motion = a_pose();
    let (moved, _) = transform(&mut m, body, &motion).unwrap();
    let (back, _) = transform(&mut m, moved, &motion.inverse()).unwrap();
    let scale = 12.0;
    for (a, b) in m
        .vertices(body)
        .unwrap()
        .iter()
        .zip(m.vertices(back).unwrap().iter())
    {
        let (pa, pb) = (
            m.vertex(a.id).unwrap().point(),
            m.vertex(b.id).unwrap().point(),
        );
        assert!((pa - pb).norm() <= 1e-12 * scale, "{pa} vs {pb}");
    }
}

#[test]
fn mass_properties_are_covariant_at_random_poses() {
    let strategy = (
        prop::point_in_box(prop::DEFAULT_SCALE),
        prop::unit_vec3(),
        prop::radius(0.1..=20.0),
        prop::radius(0.1..=20.0),
        prop::pose(),
    );
    prop::check(strategy, |(origin, direction, radius, height, motion)| {
        let axis = Axis::new(origin, direction.into_inner()).unwrap();
        let mut m = Model::default();
        let (body, _) = primitive_cylinder(&mut m, axis, radius, height).unwrap();
        let (moved, _) = transform(&mut m, body, &motion).unwrap();
        let before = mass_properties(&m, body).unwrap();
        let after = mass_properties(&m, moved).unwrap();
        let scale = before.volume.abs().max(1.0);
        prop_assert!((after.volume - before.volume).abs() <= 1e-9 * scale);
        prop_assert!((after.area - before.area).abs() <= 1e-9 * scale);
        let centroid = motion.apply(before.centroid);
        prop_assert!(
            (after.centroid - centroid).norm() <= 1e-9 * centroid.coords.abs().max().max(1.0)
        );
        let r = motion.rotation().to_rotation_matrix().into_inner();
        let expected_inertia = r * before.inertia * r.transpose();
        let inertia_scale = before.inertia.abs().max().max(1.0);
        for i in 0..3 {
            for j in 0..3 {
                prop_assert!(
                    (after.inertia[(i, j)] - expected_inertia[(i, j)]).abs()
                        <= 1e-9 * inertia_scale,
                    "[{i}, {j}]: {} vs {}",
                    after.inertia[(i, j)],
                    expected_inertia[(i, j)]
                );
            }
        }
        Ok(())
    });
}

#[test]
fn two_runs_dump_identically() {
    let build = |m: &mut Model| {
        let body = the_cylinder(m);
        transform(m, body, &a_pose()).unwrap()
    };
    let mut m = Model::default();
    let (b1, p1) = build(&mut m);
    let mut n = Model::default();
    let (b2, p2) = build(&mut n);
    assert_eq!(dump_text(&m, b1).unwrap(), dump_text(&n, b2).unwrap());
    assert_eq!(p1, p2, "the same relations on every run");
    assert_eq!(b1, b2, "the same ids");
}

#[test]
fn a_body_that_does_not_resolve_is_not_found() {
    let mut m = Model::default();
    let body = Body::forward(arris_topo::BodyId::new(3, 0));
    assert!(matches!(
        transform(&mut m, body, &Isometry::identity()),
        Err(arris_ops::OpError::NotFound(_))
    ));
}

#[cfg(debug_assertions)]
#[test]
fn a_broken_body_is_invalid_input_in_a_debug_build() {
    let mut m = Model::default();
    let body = m.raw().add_body(arris_topo::entity::Body::solid(vec![
        arris_topo::Shell::forward(arris_topo::ShellId::new(9, 0)),
    ]));
    let err = transform(&mut m, Body::forward(body), &Isometry::identity()).unwrap_err();
    assert!(
        matches!(err, arris_ops::OpError::InvalidInput { .. }),
        "{err}"
    );
}
