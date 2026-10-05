//! `ops::mirror` (ADR-0031): clean at `Full` with every orientation right
//! (the volume is the original's, positive, and the centroid is the
//! reflected one), one `Modified` per entity and nothing else, a mirror
//! of a mirror is the original, and a budget stops it and leaves the model
//! as it was.

use arris_check::{Level, check};
use arris_debug::testing::entities_of;
use arris_debug::unmetered::{mass_properties, mirror, primitive_box, primitive_cylinder};
use arris_debug::{corpus, dump_text, fixtures};
use arris_io::native;
use arris_math::{Axis, Point3, Reflection, Vec3};
use arris_ops::{Control, OpError, Stop};
use arris_topo::{Body, Model, Relation};

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

/// The bodies mirrored: the results of fixtures that between them hold
/// every quadric (a torus, a cone, a sphere, an elliptic cylinder, a
/// cylinder wall and a blend), a two-shell body and fitted curves, and a
/// box and a cylinder standing on the origin.
fn bodies() -> Vec<(String, Model, Body)> {
    let mut out = Vec::new();
    for (area, name) in [
        ("transform", "posed-cylinder"),
        ("transform", "moved-hollow-ring"),
        ("sweep", "revolve-ring"),
        ("sweep", "revolve-frustum"),
        ("sweep", "extrude-ellipse"),
        ("boolean", "ball-corner-cut"),
        ("blend", "box-edge-fillet"),
    ] {
        let dir = fixtures::corpus_root().join(area).join(name);
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
            // The same vertices, in whatever order the walk over the
            // mirrored loops listed them.
            let points = |vs: &[arris_topo::Vertex]| -> Vec<Point3> {
                vs.iter().map(|v| m.vertex(v.id).unwrap().point()).collect()
            };
            let (pa, mut pb) = (points(&a), points(&b));
            for p in pa {
                let at = pb
                    .iter()
                    .position(|q| (p - q).norm() <= 1e-9 * scale.max(12.0))
                    .unwrap_or_else(|| panic!("{label}: no vertex comes back to {p}"));
                pb.swap_remove(at);
            }
            // The structure (faces, edges, uses, orientations) is the
            // original's: the dump, ids and numbers aside, and the order
            // its entities are listed in, which follows a walk over loops
            // that a mirror turns around.
            let shape = |t: String| -> Vec<String> {
                let mut lines: Vec<String> = t
                    .lines()
                    .filter(|l| !l.starts_with("body"))
                    .map(|l| {
                        l.split_whitespace()
                            .take(2)
                            .flat_map(|w| w.chars().filter(|c| !c.is_ascii_digit()))
                            .collect::<String>()
                    })
                    .collect();
                lines.sort();
                lines
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
        assert!(n >= 1, "{name}: {n} steps");
        let expected = {
            let mut m = m.clone();
            let (image, _) = mirror(&mut m, body, plane).unwrap();
            dump_text(&m, image).unwrap()
        };
        let before = native::to_bytes(&m).unwrap();
        for k in [0, n / 2, n - 1]
            .into_iter()
            .collect::<std::collections::BTreeSet<_>>()
        {
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
    let body = Body::forward(arris_topo::BodyId::new(3, 0));
    let plane = Reflection::new(Point3::origin(), Vec3::z()).unwrap();
    assert!(matches!(
        mirror(&mut m, body, &plane),
        Err(OpError::NotFound(_))
    ));
}

/// A body of NURBS faces (Open CASCADE's box converted to B-splines,
/// read from STEP, `boolean/nurbs-box-cavity-cut`'s part) mirrored: every
/// face keeps its parameters and its control net is reflected, its use
/// toggled. The checker decides all but the face pairs a NURBS face is in
/// (which it leaves unchecked by design), and the measures are the
/// oracle's for the box — volume 12000, area 3800 — at the reflected
/// centroid.
#[test]
fn a_body_of_nurbs_faces_mirrors_to_the_same_measures() {
    use arris_check::Unchecked;
    let path = fixtures::corpus_root().join("boolean/nurbs-box-cavity-cut/nurbs-box.step");
    let text = String::from_utf8_lossy(&std::fs::read(path).unwrap()).into_owned();
    let mut m = Model::default();
    let read = arris_io::step::read(
        &mut m,
        &text,
        &arris_io::step::ReadOptions::default(),
        &Control::NONE,
    )
    .unwrap();
    let body = read
        .solids
        .into_iter()
        .find(|s| s.entity.id == 15)
        .unwrap()
        .result
        .unwrap()
        .body;
    let before = mass_properties(&m, body).unwrap();
    assert!(
        (before.volume - 12000.0).abs() < 1e-6 * 12000.0,
        "{}",
        before.volume
    );
    assert!(
        (before.area - 3800.0).abs() < 1e-6 * 3800.0,
        "{}",
        before.area
    );
    let centre = before.centroid;
    let planes = [
        Reflection::new(centre + Vec3::new(30.0, 0.0, 0.0), Vec3::x()).unwrap(),
        Reflection::new(centre, Vec3::new(1.0, 2.0, -0.5)).unwrap(),
        Reflection::new(centre + Vec3::new(2.0, 0.0, 0.0), Vec3::new(1.0, 0.3, 0.2)).unwrap(),
    ];
    for (i, plane) in planes.iter().enumerate() {
        let (image, _) = mirror(&mut m, body, plane).unwrap();
        let report = check(&m, image, Level::Full);
        let nurbs = |k: arris_geom::SurfaceKind| k == arris_geom::SurfaceKind::Nurbs;
        assert!(report.is_ok(), "plane {i}: {report}");
        assert!(
            report.unchecked().iter().all(|u| matches!(
                u,
                Unchecked::FacePair { kinds, .. } | Unchecked::ShellFacePair { kinds, .. }
                    if nurbs(kinds.0) || nurbs(kinds.1)
            )),
            "plane {i}: {:?}",
            report.unchecked()
        );
        let after = mass_properties(&m, image).unwrap();
        assert!(
            (after.volume - 12000.0).abs() < 1e-6 * 12000.0,
            "plane {i}: {}",
            after.volume
        );
        assert!(
            (after.area - 3800.0).abs() < 1e-6 * 3800.0,
            "plane {i}: {}",
            after.area
        );
        assert!(
            (after.centroid - plane.apply(centre)).norm() < 1e-6,
            "plane {i}"
        );
    }
}

/// A mirrored body is a body like any other: it tessellates closed with
/// its volume outward, and leaves through STEP and body bytes as itself
/// (checker green at `Full`, the same measures).
#[test]
fn a_mirrored_body_tessellates_and_round_trips_through_step_and_bytes() {
    for (name, mut m, body) in bodies() {
        for (i, plane) in planes().iter().enumerate() {
            let label = format!("{name} in plane {i}");
            let (image, p) = mirror(&mut m, body, plane).unwrap();
            let want = mass_properties(&m, image).unwrap();
            let scale = want.volume.abs().max(1.0);

            let mesh = arris_debug::mesh_of(&m, image).unwrap();
            assert!(mesh.is_closed(), "{label}: the mesh is open");
            let mesh_volume = mesh.signed_volume().unwrap();
            assert!(mesh_volume > 0.0, "{label}: inside out, {mesh_volume}");
            // The original's mesh is the yardstick: a chord's polygon
            // error is the same for both, and a wrongly turned face is not.
            let original = arris_debug::mesh_of(&m, body)
                .unwrap()
                .signed_volume()
                .unwrap();
            assert!(
                (mesh_volume - original).abs() <= 1e-2 * scale,
                "{label}: mesh {mesh_volume} for the original's {original}"
            );

            let text = arris_io::step::write(&m, &[image]).unwrap();
            let mut read_model = Model::default();
            let read = arris_io::step::read(
                &mut read_model,
                &text,
                &arris_io::step::ReadOptions::default(),
                &Control::NONE,
            )
            .unwrap();
            assert_eq!(read.solids.len(), 1, "{label}");
            let back = read
                .solids
                .into_iter()
                .next()
                .unwrap()
                .result
                .unwrap_or_else(|e| panic!("{label}: STEP read back refused: {e:?}"))
                .body;
            let report = check(&read_model, back, Level::Full);
            assert!(report.is_ok(), "{label}: STEP: {report}");
            let got = mass_properties(&read_model, back).unwrap();
            assert!(
                (got.volume - want.volume).abs() <= 1e-6 * scale
                    && (got.area - want.area).abs() <= 1e-6 * scale
                    && (got.centroid - want.centroid).norm() <= 1e-6 * scale.max(10.0),
                "{label}: STEP {got:?} for {want:?}"
            );

            let bytes = arris_io::body::write(&m, image, &p).unwrap();
            let mut bytes_model = Model::default();
            let imported = arris_io::body::read(&mut bytes_model, &bytes, &Control::NONE).unwrap();
            let report = check(&bytes_model, imported.body, Level::Full);
            assert!(report.is_ok(), "{label}: bytes: {report}");
            let got = mass_properties(&bytes_model, imported.body).unwrap();
            assert_eq!(
                (got.volume, got.area, got.centroid),
                (want.volume, want.area, want.centroid),
                "{label}: bytes"
            );
            assert_eq!(
                entities_of(&bytes_model, imported.body).len(),
                entities_of(&m, image).len(),
                "{label}: bytes"
            );
        }
    }
}
