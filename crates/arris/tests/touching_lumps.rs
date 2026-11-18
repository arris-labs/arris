//! Downstream operations on the two touching lumps built by a fuse (ADR-0088).

use std::collections::BTreeSet;
use std::f64::consts::PI;

use arris::check::classify::Classification::{Inside, On, Outside};
use arris::check::{Level, check, lumps};
use arris::io::{native, step};
use arris::math::{Axis, Control, Frame, Point3, Vec3};
use arris::mesh::{TriMesh, tessellate};
use arris::ops::measure::mass_properties;
use arris::ops::query::classify_points;
use arris::ops::{
    BooleanReason, OpError, Reason, common, cut, fuse, primitive_box, primitive_cylinder,
};
use arris::topo::entity::BodyKind;
use arris::topo::{Body, Model, Shape};
use arris_debug::{corpus, dump_text, fixtures, oracle};

const FIXTURE: &str = "boolean/tangent-cylinders-fuse";
const NONE: &Control<'_> = &Control::NONE;

fn touching_cylinders() -> (Model, Body) {
    let dir = fixtures::corpus_root().join(FIXTURE);
    let chain = corpus::chain(&dir, "default").unwrap();
    let body = chain.result().unwrap();
    full_two_lumps(&chain.model, body);
    (chain.model, body)
}

fn cylinders_touching_at_a_seam() -> (Model, Body) {
    let mut m = Model::default();
    let frame = Frame::new(Point3::origin(), Vec3::z(), Vec3::y()).unwrap();
    let a = arris_debug::sample::cylinder_in(&mut m, frame, 1.0, 2.0).unwrap();
    let b = primitive_cylinder(
        &mut m,
        Axis::z_at(Point3::new(0.0, 2.0, -1.0)),
        1.0,
        4.0,
        NONE,
    )
    .unwrap()
    .0;
    let (body, _) = fuse(&mut m, a, b, NONE).unwrap();
    full_two_lumps(&m, body);
    (m, body)
}

fn full_two_lumps(m: &Model, body: Body) {
    assert_eq!(m.body(body.id).unwrap().kind(), BodyKind::Solid);
    let report = check(m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let found = lumps(m, body).unwrap();
    assert_eq!(found.len(), 2);
    assert!(found.iter().all(|lump| lump.voids.is_empty()));
}

fn assert_measure(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() <= 1e-9 * expected.abs(),
        "measured {actual}, expected {expected}"
    );
}

#[test]
fn touching_lumps_can_be_measured_and_classified() {
    let (m, body) = touching_cylinders();
    let props = mass_properties(&m, body, NONE).unwrap();
    assert_measure(props.volume, 6.0 * PI);
    assert_measure(props.area, 16.0 * PI);
    assert_measure(props.centroid.y, 4.0 / 3.0);
    assert_measure(props.centroid.z, 1.0);

    let probes = [
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(0.0, 2.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.0, 4.0, 1.0),
    ];
    let codes = classify_points(&m, body, &probes, NONE).unwrap();
    assert!(
        matches!(
            codes.as_slice(),
            [Some(Inside), Some(Inside), Some(On(_)), Some(Outside)]
        ),
        "{codes:?}"
    );
}

#[test]
fn the_mesh_of_touching_lumps_is_watertight_per_lump() {
    let (m, body) = touching_cylinders();
    let mesh = tessellate(&m, body, 0.01, NONE).unwrap();
    let mut used = BTreeSet::new();
    let mut triangle_count = 0;
    for lump in lumps(&m, body).unwrap() {
        let faces = m.shell(lump.outer.id).unwrap().faces();
        let triangles: Vec<_> = faces
            .iter()
            .flat_map(|face| mesh.face_triangles(face.id).unwrap())
            .copied()
            .collect();
        let indices: BTreeSet<_> = triangles.iter().flatten().copied().collect();
        assert!(used.is_disjoint(&indices), "the lumps share mesh indices");
        used.extend(indices);
        triangle_count += triangles.len();
        let part =
            TriMesh::from_parts(mesh.positions().to_vec(), triangles, vec![], vec![], vec![])
                .unwrap();
        assert!(part.is_closed(), "a lump has an open mesh");
        assert!(part.signed_volume().unwrap() > 0.0);
    }
    assert_eq!(triangle_count, mesh.triangles().len());
}

#[test]
fn touching_lumps_write_two_step_solids_and_round_trip_in_the_native_format() {
    let (m, body) = touching_cylinders();
    let text = step::write(&m, &[body]).unwrap();
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 2);
    oracle::compare(FIXTURE, &text, None, "touching-lumps-downstream").unwrap();

    let bytes = native::to_bytes(&m).unwrap();
    let back = native::from_bytes(&bytes).unwrap();
    full_two_lumps(&back, body);
    assert_eq!(
        dump_text(&back, body).unwrap(),
        dump_text(&m, body).unwrap()
    );
    assert_eq!(native::to_bytes(&back).unwrap(), bytes);
}

#[test]
fn a_through_hole_can_be_cut_in_one_touching_lump() {
    let (mut m, body) = touching_cylinders();
    let hole_radius = 0.25;
    let drill = primitive_cylinder(
        &mut m,
        Axis::z_at(Point3::new(0.0, 0.0, -1.0)),
        hole_radius,
        4.0,
        NONE,
    )
    .unwrap()
    .0;
    let (result, _) = cut(&mut m, body, drill, NONE).unwrap();
    full_two_lumps(&m, result);
    assert_measure(
        mass_properties(&m, result, NONE).unwrap().volume,
        (6.0 - 2.0 * hole_radius * hole_radius) * PI,
    );
}

#[test]
fn a_plate_joining_both_lumps_leaves_a_tangent_contact_in_one_lump() {
    for (mut m, body) in [touching_cylinders(), cylinders_touching_at_a_seam()] {
        let plate = primitive_box(&mut m, [-0.25, 0.5, -2.0], [0.25, 1.5, 0.75], NONE)
            .unwrap()
            .0;
        let before = native::to_bytes(&m).unwrap();
        let error = fuse(&mut m, body, plate, NONE)
            .map(|(result, _)| result)
            .unwrap_err();
        let OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::TangentContact),
            entities,
        } = error
        else {
            panic!("{error:?}");
        };
        let walls: Vec<_> = m
            .faces(body)
            .unwrap()
            .into_iter()
            .filter(|face| {
                let surface = m.face(face.id).unwrap().surface();
                matches!(
                    m.surface(surface).unwrap(),
                    arris::geom::Surface::Cylinder { .. }
                )
            })
            .map(Shape::from)
            .collect();
        assert_eq!(entities, walls);
        assert_eq!(native::to_bytes(&m).unwrap(), before);
    }
}

#[test]
fn a_plate_covering_the_entire_contact_can_join_the_lumps() {
    let (mut m, body) = touching_cylinders();
    let plate = primitive_box(&mut m, [-0.25, 0.5, -2.0], [0.25, 1.5, 4.0], NONE)
        .unwrap()
        .0;
    let (result, _) = fuse(&mut m, body, plate, NONE).unwrap();
    let report = check(&m, result, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(lumps(&m, result).unwrap().len(), 1);
}

#[test]
fn a_bridge_leaving_the_point_contact_exposed_names_both_source_faces() {
    let dir = fixtures::corpus_root().join("boolean/ball-on-disc-fuse");
    let mut chain = corpus::chain(&dir, "default").unwrap();
    let body = chain.result().unwrap();
    let ball = chain.steps["ball"].body;
    let stem = chain.steps["stem"].body;
    let m = &mut chain.model;
    let bridge = primitive_box(m, [0.5, -0.25, -2.25], [0.8, 0.25, -1.5], NONE)
        .unwrap()
        .0;
    let poles = [Point3::new(0.0, 0.0, -2.0)];
    let disc = classify_points(m, stem, &poles, NONE).unwrap();
    let [Some(On(disc))] = disc.as_slice() else {
        panic!("{disc:?}");
    };
    let sphere = m.faces(ball).unwrap()[0];
    let expected = BTreeSet::from([*disc, Shape::from(sphere)]);
    let before = native::to_bytes(m).unwrap();
    let error = fuse(m, body, bridge, NONE).unwrap_err();
    let OpError::Degenerate {
        reason: Reason::Boolean(BooleanReason::TangentContact),
        entities,
    } = error
    else {
        panic!("{error:?}");
    };
    assert_eq!(entities.len(), 2);
    assert_eq!(entities.into_iter().collect::<BTreeSet<_>>(), expected);
    assert_eq!(native::to_bytes(m).unwrap(), before);
}

#[test]
fn a_box_over_the_contact_can_clip_one_touching_lump() {
    let (mut m, body) = touching_cylinders();
    // The box contains the shared ruling and removes half a unit from the
    // bottom of the longer cylinder.
    let clip = primitive_box(&mut m, [-2.0, -2.0, -0.5], [2.0, 4.0, 4.0], NONE)
        .unwrap()
        .0;
    let (result, _) = common(&mut m, body, clip, NONE).unwrap();
    full_two_lumps(&m, result);
    assert_measure(mass_properties(&m, result, NONE).unwrap().volume, 5.5 * PI);
}
