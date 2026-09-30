//! `ops::mirror` (ADR-0031): clean at `Full` with every orientation right
//! (the volume is the original's, positive, and the centroid is the
//! reflected one), one `Modified` per entity and nothing else, a mirror
//! of a mirror is the original, and a budget stops it and leaves the model
//! as it was.

use arris_debug::testing::entities_of;
use arris_debug::unmetered::{mass_properties, mirror, primitive_box, primitive_cylinder};
use arris_debug::{corpus, dump_text, fixtures};
use arris_io::native;
use arris_ops::arris_check::arris_topo::arris_math::{Axis, Point3, Reflection, Vec3};
use arris_ops::arris_check::arris_topo::{Body, Model, Relation};
use arris_ops::arris_check::{Level, check};
use arris_ops::{Control, OpError, Stop};

/// The planes each body is mirrored in: one beside it (axis-aligned),
/// one oblique through it, one through its middle.
fn planes() -> Vec<Reflection> {
    let p = |o: [f64; 3], n: [f64; 3]| {
        Reflection::new(Point3::new(o[0], o[1], o[2]), Vec3::new(n[0], n[1], n[2])).unwrap()
    };
    vec![
        p([30.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        p([1.0, 2.0, 3.0], [1.0, 2.0, -0.5]),
        p([12.0, -5.0, 8.0], [0.0, 1.0, 0.0]),
    ]
}

/// The bodies mirrored: the two `transform/` fixtures' results and a box
/// and a cylinder standing on the origin.
fn bodies() -> Vec<(String, Model, Body)> {
    let mut out = Vec::new();
    for name in ["posed-cylinder", "moved-hollow-ring"] {
        let dir = fixtures::corpus_root().join("transform").join(name);
        let inputs = corpus::inputs(&dir, "default").unwrap();
        let mut m = inputs.model.clone();
        let made = inputs.run_result(&mut m, &Control::NONE).unwrap();
        out.push((name.to_string(), m, made.body));
    }
    let mut m = Model::default();
    let (b, _) = primitive_box(
        &mut m,
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(4.0, 7.0, 5.0),
    )
    .unwrap();
    out.push(("box".into(), m, b));
    let mut m = Model::default();
    let (c, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 4.0, 12.0).unwrap();
    out.push(("cylinder".into(), m, c));
    out
}

#[test]
fn a_mirrored_body_is_clean_reflected_and_one_to_one_modified() {
    for (name, mut m, body) in bodies() {
        let before = mass_properties(&m, body).unwrap();
        let counts = m.faces(body).unwrap().len();
        for (i, plane) in planes().iter().enumerate() {
            let label = format!("{name} in plane {i}");
            let (image, p) = mirror(&mut m, body, plane).unwrap();
            let report = check(&m, image, Level::Full);
            assert!(
                report.is_ok(),
                "{label}: {report}\n{}",
                dump_text(&m, image).unwrap()
            );
            assert!(report.unchecked().is_empty(), "{label}");

            let after = mass_properties(&m, image).unwrap();
            let scale = before.volume.abs().max(1.0);
            assert!(after.volume > 0.0, "{label}: inside out, {}", after.volume);
            assert!(
                (after.volume - before.volume).abs() <= 1e-9 * scale,
                "{label}"
            );
            assert!((after.area - before.area).abs() <= 1e-9 * scale, "{label}");
            let centroid = plane.apply(before.centroid);
            assert!(
                (after.centroid - centroid).norm() <= 1e-9 * scale.max(centroid.coords.abs().max()),
                "{label}"
            );
            assert_eq!(m.faces(image).unwrap().len(), counts, "{label}");

            let (old, new) = (entities_of(&m, body), entities_of(&m, image));
            assert_eq!(old.len(), new.len(), "{label}");
            assert_eq!(p.deleted().count(), 0, "{label}");
            assert_eq!(p.origins_recorded().count(), old.len(), "{label}");
            assert_eq!(p.outputs(), new, "{label}");
            for &e in &old {
                let images = p.modified_from(e);
                assert_eq!(images.len(), 1, "{label}: {e}");
                assert!(new.contains(&images[0]), "{label}: {e}");
                assert!(p.generated_from(e).is_empty(), "{label}: {e}");
            }
            for &e in &new {
                let origins = p.origins(e);
                assert_eq!(origins.len(), 1, "{label}: {e}");
                assert_eq!(origins[0].0, Relation::Modified, "{label}: {e}");
            }
        }
    }
}

#[test]
fn a_mirror_of_a_mirror_in_the_same_plane_is_the_original() {
    for (name, mut m, body) in bodies() {
        let before = mass_properties(&m, body).unwrap();
        for (i, plane) in planes().iter().enumerate() {
            let label = format!("{name} in plane {i}");
            let (image, _) = mirror(&mut m, body, plane).unwrap();
            let (back, _) = mirror(&mut m, image, plane).unwrap();
            let report = check(&m, back, Level::Full);
            assert!(report.is_ok(), "{label}: {report}");
            let after = mass_properties(&m, back).unwrap();
            let scale = before.volume.abs().max(1.0);
            assert!(
                (after.volume - before.volume).abs() <= 1e-9 * scale,
                "{label}"
            );
            assert!(
                (after.centroid - before.centroid).norm()
                    <= 1e-9 * scale.max(before.centroid.coords.abs().max()),
                "{label}"
            );
            let (a, b) = (m.vertices(body).unwrap(), m.vertices(back).unwrap());
            assert_eq!(a.len(), b.len(), "{label}");
            for (x, y) in a.iter().zip(&b) {
                let (px, py) = (
                    m.vertex(x.id).unwrap().point(),
                    m.vertex(y.id).unwrap().point(),
                );
                assert!(
                    (px - py).norm() <= 1e-9 * scale.max(12.0),
                    "{label}: {px} vs {py}"
                );
            }
            // The structure (faces, edges, uses, orientations) is the
            // original's: the dump, ids and numbers aside.
            let shape = |t: String| -> Vec<String> {
                t.lines()
                    .filter(|l| !l.starts_with("body"))
                    .map(|l| {
                        l.split_whitespace()
                            .take(2)
                            .flat_map(|w| w.chars().filter(|c| !c.is_ascii_digit()))
                            .collect::<String>()
                    })
                    .collect()
            };
            assert_eq!(
                shape(dump_text(&m, body).unwrap()),
                shape(dump_text(&m, back).unwrap()),
                "{label}"
            );
        }
    }
}

#[test]
fn a_budget_stops_a_mirror_and_leaves_the_model_as_it_was() {
    use std::sync::atomic::{AtomicU64, Ordering};
    for (name, m, body) in bodies() {
        let plane = &planes()[1];
        let asked = AtomicU64::new(0);
        let poll = || {
            asked.fetch_add(1, Ordering::Relaxed);
            false
        };
        let mut whole_model = m.clone();
        arris_ops::mirror(&mut whole_model, body, plane, &Control::poll(&poll)).unwrap();
        let n = asked.load(Ordering::Relaxed);
        assert!(n >= 3, "{name}: {n} steps");
        let expected = {
            let mut m = m.clone();
            let (image, _) = mirror(&mut m, body, plane).unwrap();
            dump_text(&m, image).unwrap()
        };
        let before = native::to_bytes(&m).unwrap();
        for k in [0, 1, n / 2, n - 1] {
            let mut m = m.clone();
            let stop = match arris_ops::mirror(&mut m, body, plane, &Control::budget(k)) {
                Err(OpError::Interrupted(stop)) => stop,
                other => panic!("{name}: budget {k} of {n}: {other:?}"),
            };
            assert_eq!((stop.by, stop.steps), (Stop::Budget, k), "{name}");
            assert_eq!(
                native::to_bytes(&m).unwrap(),
                before,
                "{name}: budget {k} of {n}"
            );
        }
        let mut m = m.clone();
        let (image, _) = arris_ops::mirror(&mut m, body, plane, &Control::budget(n)).unwrap();
        assert_eq!(
            dump_text(&m, image).unwrap(),
            expected,
            "{name}: a budget of {n}"
        );
    }
}

#[test]
fn a_body_that_does_not_resolve_is_not_found() {
    let mut m = Model::default();
    let body = Body::forward(arris_ops::arris_check::arris_topo::BodyId::new(3, 0));
    let plane = Reflection::new(Point3::origin(), Vec3::z()).unwrap();
    assert!(matches!(
        mirror(&mut m, body, &plane),
        Err(OpError::NotFound(_))
    ));
}
