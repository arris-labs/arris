//! `cut_many` and `fuse_many` (ADR-0050): one decomposition over a body and
//! its tools, held to the chain of two-operand booleans it replaces, with a
//! provenance that names each tool, and the refusals it owes.

use arris_debug::unmetered::{
    cut, cut_many, fuse, fuse_many, interferences_many, mass_properties, primitive_box,
    primitive_cylinder,
};
use arris_math::{Axis, Point3, Vec3};
use arris_ops::{BooleanReason, OpError, Reason};
use arris_topo::{Body, Model, Orientation, Origin, Shape};

fn shape(id: impl Into<arris_topo::EntityId>) -> Shape {
    Shape::new(id, Orientation::Forward)
}

/// A plate [0,100]×[0,100]×[0,10] and `n` through holes of radius 3 along a
/// row at y = 50, 15 apart from x = 15.
fn plate_and_holes(m: &mut Model, n: usize) -> (Body, Vec<Body>) {
    let (plate, _) = primitive_box(m, Point3::origin(), Point3::new(100.0, 100.0, 10.0)).unwrap();
    let holes = (0..n)
        .map(|i| {
            let axis = Axis::z_at(Point3::new(15.0 + 15.0 * i as f64, 50.0, -1.0));
            primitive_cylinder(m, axis, 3.0, 12.0).unwrap().0
        })
        .collect();
    (plate, holes)
}

/// Counts of the entities of `body`: shells, faces, edges, vertices.
fn counts(m: &Model, body: Body) -> [usize; 4] {
    let c = m.closure(body).unwrap();
    [
        c.shells.len(),
        c.faces.len(),
        c.edges.len(),
        c.vertices.len(),
    ]
}

/// Volume, area and the entity counts agree to the tolerance a result is
/// measured at.
fn assert_same_solid(m: &Model, many: Body, chained: Body, what: &str) {
    let (a, b) = (
        mass_properties(m, many).unwrap(),
        mass_properties(m, chained).unwrap(),
    );
    assert!(
        (a.volume - b.volume).abs() <= 1e-9 * b.volume.abs(),
        "{what}: volume {} against {}",
        a.volume,
        b.volume
    );
    assert!(
        (a.area - b.area).abs() <= 1e-9 * b.area.abs(),
        "{what}: area {} against {}",
        a.area,
        b.area
    );
    assert_eq!(counts(m, many), counts(m, chained), "{what}: counts");
}

#[test]
fn a_row_of_holes_cut_in_one_call_is_the_chain_of_cuts() {
    let mut m = Model::default();
    let (plate, holes) = plate_and_holes(&mut m, 6);
    let (many, _) = cut_many(&mut m, plate, &holes).unwrap();
    let mut chained = plate;
    for &h in &holes {
        chained = cut(&mut m, chained, h).unwrap().0;
    }
    assert_same_solid(&m, many, chained, "six holes");
    let volume = mass_properties(&m, many).unwrap().volume;
    let want = 100.0 * 100.0 * 10.0 - 6.0 * core::f64::consts::PI * 9.0 * 10.0;
    assert!(
        (volume - want).abs() <= 1e-9 * want,
        "{volume} against {want}"
    );
}

#[test]
fn the_geometry_does_not_depend_on_the_order_of_the_tools() {
    let mut m = Model::default();
    let (plate, holes) = plate_and_holes(&mut m, 5);
    let (forward, _) = cut_many(&mut m, plate, &holes).unwrap();
    let mut reversed = holes.clone();
    reversed.reverse();
    let (backward, _) = cut_many(&mut m, plate, &reversed).unwrap();
    assert_same_solid(&m, forward, backward, "five holes, either order");
}

#[test]
fn round_holes_through_a_curved_wall_are_the_chain_of_cuts() {
    // Cylinders along x blind in a drum's wall: saddle curves, the
    // quadric pair a tool of this kind meets the target's wall in.
    let mut m = Model::default();
    let (drum, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 20.0, 40.0).unwrap();
    let tools: Vec<Body> = [12.0, 28.0]
        .into_iter()
        .map(|z| {
            let axis = Axis::new(Point3::new(10.0, 0.0, z), Vec3::x()).unwrap();
            primitive_cylinder(&mut m, axis, 3.0, 20.0).unwrap().0
        })
        .collect();
    let (many, _) = cut_many(&mut m, drum, &tools).unwrap();
    let mut chained = drum;
    for &t in &tools {
        chained = cut(&mut m, chained, t).unwrap().0;
    }
    assert_same_solid(&m, many, chained, "radial holes");
}

#[test]
fn bosses_fused_in_one_call_are_the_chain_of_fuses() {
    let mut m = Model::default();
    let (plate, _) =
        primitive_box(&mut m, Point3::origin(), Point3::new(100.0, 30.0, 10.0)).unwrap();
    let mut bodies = vec![plate];
    for i in 0..3 {
        let axis = Axis::z_at(Point3::new(20.0 + 30.0 * i as f64, 15.0, 5.0));
        bodies.push(primitive_cylinder(&mut m, axis, 4.0, 15.0).unwrap().0);
    }
    let (many, _) = fuse_many(&mut m, &bodies).unwrap();
    let mut chained = plate;
    for &b in &bodies[1..] {
        chained = fuse(&mut m, chained, b).unwrap().0;
    }
    assert_same_solid(&m, many, chained, "three bosses");
}

#[test]
fn the_record_names_each_tool_and_keeps_what_no_tool_touched() {
    let mut m = Model::default();
    let (plate, holes) = plate_and_holes(&mut m, 3);
    let (body, p) = cut_many(&mut m, plate, &holes).unwrap();

    // The result is the target's body, modified; every tool is deleted.
    assert_eq!(p.modified_from(Origin::Entity(shape(plate.id))).len(), 1);
    for &h in &holes {
        assert!(p.is_deleted(shape(h.id)), "tool {h} not deleted\n{p}");
        for f in m.faces(h).unwrap() {
            assert!(p.is_deleted(shape(f.id)), "{} not deleted\n{p}", f.id);
        }
    }
    // Each tool's wall survives as the hole's wall, generated from that
    // tool's wall alone.
    for &h in &holes {
        let wall = m
            .faces(h)
            .unwrap()
            .into_iter()
            .find(|f| {
                matches!(
                    m.surface(m.face(f.id).unwrap().surface()).unwrap(),
                    arris_geom::Surface::Cylinder { .. }
                )
            })
            .unwrap();
        let faces = p
            .generated_from(shape(wall.id))
            .iter()
            .filter(|s| matches!(s.id, arris_topo::EntityId::Face(_)))
            .count();
        assert_eq!(faces, 1, "{p}");
    }
    // The four side faces, which no tool reaches, keep their ids.
    let sides: Vec<_> = m
        .faces(plate)
        .unwrap()
        .into_iter()
        .filter(|f| p.is_kept(shape(f.id), &m, body))
        .collect();
    assert_eq!(sides.len(), 4, "{p}");
}

#[test]
fn a_tool_that_misses_the_target_is_deleted_whole_and_changes_nothing() {
    let mut m = Model::default();
    let (plate, mut holes) = plate_and_holes(&mut m, 2);
    let far = primitive_cylinder(
        &mut m,
        Axis::z_at(Point3::new(300.0, 50.0, -1.0)),
        3.0,
        12.0,
    )
    .unwrap()
    .0;
    holes.push(far);
    let (body, p) = cut_many(&mut m, plate, &holes).unwrap();
    assert!(p.is_deleted(shape(far.id)), "{p}");
    for f in m.faces(far).unwrap() {
        assert!(p.is_deleted(shape(f.id)) && p.generated_from(shape(f.id)).is_empty());
    }
    // Two holes, as without the far tool.
    let (two, _) = cut_many(&mut m, plate, &holes[..2]).unwrap();
    assert_same_solid(&m, body, two, "a tool that misses");

    // Alone, it leaves every entity of the target as it was.
    let (same, p) = cut_many(&mut m, plate, &[far]).unwrap();
    for f in m.faces(plate).unwrap() {
        assert!(p.is_kept(shape(f.id), &m, same), "{} changed\n{p}", f.id);
    }
    assert!(p.is_deleted(shape(far.id)));
}

#[test]
fn a_cut_of_one_tool_is_a_cut() {
    let mut m = Model::default();
    let (plate, holes) = plate_and_holes(&mut m, 1);
    let mut twin = m.clone();
    let (one, p) = cut(&mut m, plate, holes[0]).unwrap();
    let (many, q) = cut_many(&mut twin, plate, &holes).unwrap();
    assert_eq!(p, q);
    assert_eq!(
        arris_debug::dump::dump_text(&m, one).unwrap(),
        arris_debug::dump::dump_text(&twin, many).unwrap()
    );
}

fn refused(e: OpError) -> (Vec<Shape>, BooleanReason) {
    let OpError::Degenerate {
        entities,
        reason: Reason::Boolean(reason),
    } = e
    else {
        panic!("not a boolean refusal: {e}");
    };
    (entities, reason)
}

#[test]
fn no_tool_and_a_body_twice_are_refused_by_name() {
    let mut m = Model::default();
    let (plate, holes) = plate_and_holes(&mut m, 2);
    let before = arris_io::native::to_bytes(&m).unwrap();

    let (entities, reason) = refused(cut_many(&mut m, plate, &[]).unwrap_err());
    assert_eq!(
        (entities, reason),
        (vec![shape(plate.id)], BooleanReason::NoTools)
    );
    let (_, reason) = refused(fuse_many(&mut m, &[plate]).unwrap_err());
    assert_eq!(reason, BooleanReason::NoTools);
    let (_, reason) = refused(fuse_many(&mut m, &[]).unwrap_err());
    assert_eq!(reason, BooleanReason::NoTools);

    let (entities, reason) =
        refused(cut_many(&mut m, plate, &[holes[0], holes[1], holes[0]]).unwrap_err());
    assert_eq!(
        (entities, reason),
        (vec![shape(holes[0].id)], BooleanReason::RepeatedOperand)
    );
    let (entities, reason) = refused(cut_many(&mut m, plate, &[holes[0], plate]).unwrap_err());
    assert_eq!(
        (entities, reason),
        (vec![shape(plate.id)], BooleanReason::RepeatedOperand)
    );
    let (_, reason) = refused(fuse_many(&mut m, &[plate, holes[0], plate]).unwrap_err());
    assert_eq!(reason, BooleanReason::RepeatedOperand);
    assert_eq!(
        arris_io::native::to_bytes(&m).unwrap(),
        before,
        "the model moved"
    );
}

#[test]
fn tools_that_meet_each_other_are_refused_until_they_are_decomposed() {
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let mut overlapping = Vec::new();
    for x in [40.0, 44.0] {
        let axis = Axis::z_at(Point3::new(x, 50.0, -1.0));
        overlapping.push(primitive_cylinder(&mut m, axis, 3.0, 12.0).unwrap().0);
    }
    let before = arris_io::native::to_bytes(&m).unwrap();
    let err = cut_many(&mut m, plate, &overlapping).unwrap_err();
    assert!(matches!(err, OpError::Unsupported { .. }), "{err}");
    assert_eq!(
        arris_io::native::to_bytes(&m).unwrap(),
        before,
        "the model moved"
    );
}

#[test]
fn the_decomposition_over_three_operands_pairs_only_the_ones_that_meet() {
    let mut m = Model::default();
    let (plate, holes) = plate_and_holes(&mut m, 2);
    let mut operands = vec![plate];
    operands.extend(&holes);
    let i = interferences_many(&m, &operands).unwrap();
    assert_eq!(i.operands, operands);
    assert!(!i.pairs.is_empty());
    // The holes are 15 apart with radius 3: no pair between them.
    assert!(i.pairs.iter().all(|p| p.operands[0] == 0), "{i}");
    let used: std::collections::BTreeSet<usize> = i.pairs.iter().map(|p| p.operands[1]).collect();
    assert_eq!(used, [1, 2].into_iter().collect());
}
