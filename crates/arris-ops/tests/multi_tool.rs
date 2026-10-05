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

/// Three cylinders of radius 5 along x, y and z through `centre`, each
/// `length` long and centred on it.
fn three_orthogonal(m: &mut Model, centre: Point3, length: f64) -> Vec<Body> {
    [Vec3::x(), Vec3::y(), Vec3::z()]
        .into_iter()
        .map(|d| {
            let axis = Axis::new(centre - d * (0.5 * length), d).unwrap();
            primitive_cylinder(m, axis, 5.0, length).unwrap().0
        })
        .collect()
}

#[test]
fn overlapping_holes_cut_in_one_call_are_the_chain_of_cuts() {
    // Three holes 4 apart with radius 3: a slot, each two neighbours'
    // circles crossing on both of the plate's faces off every edge.
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let holes: Vec<Body> = [40.0, 44.0, 48.0]
        .into_iter()
        .map(|x| {
            let axis = Axis::z_at(Point3::new(x, 50.0, -1.0));
            primitive_cylinder(&mut m, axis, 3.0, 12.0).unwrap().0
        })
        .collect();
    let mut operands = vec![plate];
    operands.extend(&holes);
    let i = interferences_many(&m, &operands).unwrap();
    // Two neighbouring pairs, two crossings each, on the top and the
    // bottom: eight triple points, each its own section vertex.
    assert_eq!(i.triple_points.len(), 8, "{i}");
    assert!(
        i.triple_points.iter().all(|q| {
            q.vertex.is_some_and(|v| {
                i.vertices[v].source == arris_ops::boolean::VertexSource::TriplePoint
            })
        }),
        "{i}"
    );
    let (many, _) = cut_many(&mut m, plate, &holes).unwrap();
    let mut chained = plate;
    for &h in &holes {
        chained = cut(&mut m, chained, h).unwrap().0;
    }
    assert_same_solid(&m, many, chained, "a slot");
}

#[test]
fn three_bores_through_one_point_are_the_chain_of_cuts() {
    let mut m = Model::default();
    let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 40.0, 40.0)).unwrap();
    let bores = three_orthogonal(&mut m, Point3::new(20.0, 20.0, 20.0), 60.0);
    let (many, _) = cut_many(&mut m, cube, &bores).unwrap();
    let mut chained = cube;
    for &b in &bores {
        chained = cut(&mut m, chained, b).unwrap().0;
    }
    assert_same_solid(&m, many, chained, "three bores");
}

#[test]
fn a_counterbore_in_one_call_is_the_chain_of_cuts() {
    // A tool's wall inside another tool: the hole's wall above the bore's
    // floor is dropped, the floor is cut by the hole.
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let hole = primitive_cylinder(&mut m, Axis::z_at(Point3::new(50.0, 50.0, -1.0)), 3.0, 12.0)
        .unwrap()
        .0;
    let bore = primitive_cylinder(&mut m, Axis::z_at(Point3::new(50.0, 50.0, 6.0)), 6.0, 6.0)
        .unwrap()
        .0;
    let (many, _) = cut_many(&mut m, plate, &[hole, bore]).unwrap();
    let chained = cut(&mut m, plate, hole).unwrap().0;
    let chained = cut(&mut m, chained, bore).unwrap().0;
    assert_same_solid(&m, many, chained, "a counterbore");
}

#[test]
fn a_tripod_fused_in_one_call_is_the_chain_of_fuses() {
    let mut m = Model::default();
    let legs = three_orthogonal(&mut m, Point3::origin(), 40.0);
    let (many, _) = fuse_many(&mut m, &legs).unwrap();
    let chained = fuse(&mut m, legs[0], legs[1]).unwrap().0;
    let chained = fuse(&mut m, chained, legs[2]).unwrap().0;
    assert_same_solid(&m, many, chained, "a tripod");
}

#[test]
fn a_triple_point_is_generated_from_its_three_faces() {
    // Two crossing pockets: on the plate's top their walls' lines cross
    // at (40, 40, 10), where no edge of any operand passes.
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let (p1, _) = primitive_box(
        &mut m,
        Point3::new(20.0, 40.0, 5.0),
        Point3::new(80.0, 60.0, 12.0),
    )
    .unwrap();
    let (p2, _) = primitive_box(
        &mut m,
        Point3::new(40.0, 20.0, 5.0),
        Point3::new(60.0, 80.0, 12.0),
    )
    .unwrap();
    let i = interferences_many(&m, &[plate, p1, p2]).unwrap();
    let corner = Point3::new(40.0, 40.0, 10.0);
    let q = i
        .triple_points
        .iter()
        .find(|q| (q.point - corner).norm() < 1e-9)
        .unwrap_or_else(|| panic!("no triple point at the corner\n{i}"));
    let pair = &i.pairs[q.pair];
    let faces = [pair.a, pair.b, q.face];

    let (body, p) = cut_many(&mut m, plate, &[p1, p2]).unwrap();
    let vertex = m
        .vertices(body)
        .unwrap()
        .into_iter()
        .find(|v| (m.vertex(v.id).unwrap().point() - corner).norm() < 1e-9)
        .expect("the corner is a vertex of the result");
    for f in faces {
        assert!(
            p.generated_from(shape(f)).contains(&shape(vertex.id)),
            "{} not generated from {f}\n{p}",
            vertex.id
        );
    }
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

#[test]
fn a_triple_point_on_a_rim_is_the_rim_s_hit() {
    // The z bore blind, its cap's rim at the height of four of the eight
    // triple points: each is where the rim pierces the other two bores'
    // walls as well, and the tolerance components make the two one
    // section vertex, with no rule of its own (ADR-0050 §5).
    let mut m = Model::default();
    let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 40.0, 40.0)).unwrap();
    let mut bores = three_orthogonal(&mut m, Point3::new(20.0, 20.0, 20.0), 60.0);
    let rim = 20.0 + 5.0 / 2f64.sqrt();
    bores[2] = primitive_cylinder(
        &mut m,
        Axis::z_at(Point3::new(20.0, 20.0, -10.0)),
        5.0,
        rim + 10.0,
    )
    .unwrap()
    .0;
    let mut operands = vec![cube];
    operands.extend(&bores);
    let i = interferences_many(&m, &operands).unwrap();
    let on_rim: Vec<_> = i
        .vertices
        .iter()
        .filter(|v| !v.triple_points.is_empty() && (v.point.z - rim).abs() < 1e-9)
        .collect();
    assert_eq!(on_rim.len(), 4, "{i}");
    assert!(on_rim.iter().all(|v| !v.hits.is_empty()), "{i}");

    let (many, _) = cut_many(&mut m, cube, &bores).unwrap();
    let mut chained = cube;
    for &b in &bores {
        chained = cut(&mut m, chained, b).unwrap().0;
    }
    assert_same_solid(&m, many, chained, "a bore ending at the triple points");
}

#[test]
fn a_box_repeated_by_value_is_one_pocket() {
    // Every face of the second tool coincident with the first's: the
    // walls cut the plate's top along four lines each pair of tools'
    // sections share, held once (ADR-0050, landed with step 5).
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let tools: Vec<Body> = (0..2)
        .map(|_| {
            primitive_box(
                &mut m,
                Point3::new(20.0, 20.0, 5.0),
                Point3::new(40.0, 40.0, 12.0),
            )
            .unwrap()
            .0
        })
        .collect();
    let (many, _) = cut_many(&mut m, plate, &tools).unwrap();
    let chained = cut(&mut m, plate, tools[0]).unwrap().0;
    assert_same_solid(&m, many, chained, "a box twice");
}

#[test]
fn a_section_two_tools_share_is_generated_from_all_three_faces() {
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let holes: Vec<Body> = (0..2)
        .map(|_| {
            let axis = Axis::z_at(Point3::new(50.0, 50.0, -1.0));
            primitive_cylinder(&mut m, axis, 3.0, 12.0).unwrap().0
        })
        .collect();
    let mut operands = vec![plate];
    operands.extend(&holes);
    let i = interferences_many(&m, &operands).unwrap();
    // The circle on the top and on the bottom, each one edge with a
    // shared use on the second tool's wall.
    assert_eq!(i.sections.len(), 2, "{i}");
    let walls: Vec<_> = holes
        .iter()
        .map(|&h| {
            m.faces(h)
                .unwrap()
                .into_iter()
                .find(|f| {
                    matches!(
                        m.surface(m.face(f.id).unwrap().surface()).unwrap(),
                        arris_geom::Surface::Cylinder { .. }
                    )
                })
                .unwrap()
                .id
        })
        .collect();
    for s in &i.sections {
        let shared: Vec<_> = s.shared.iter().map(|u| u.face).collect();
        assert_eq!(shared, vec![walls[1]], "{i}");
    }

    let (body, p) = cut_many(&mut m, plate, &holes).unwrap();
    let top = m
        .faces(plate)
        .unwrap()
        .into_iter()
        .find(|f| {
            let mid = Point3::new(10.0, 10.0, 10.0);
            m.surface(m.face(f.id).unwrap().surface())
                .unwrap()
                .project(mid)
                .is_ok_and(|q| q.distance < 1e-9)
        })
        .unwrap()
        .id;
    let rim = m
        .edges(body)
        .unwrap()
        .into_iter()
        .find(|e| p.generated_from(shape(top)).contains(&shape(e.id)))
        .expect("the hole's rim on the top is generated from the top");
    for w in &walls {
        assert!(
            p.generated_from(shape(*w)).contains(&shape(rim.id)),
            "{} not generated from {w}\n{p}",
            rim.id
        );
    }
}

#[test]
fn tools_touching_inside_the_target_are_a_tangent_contact() {
    // Two blind bores whose walls touch along a ruling that runs up out
    // of the plate's top: inside the plate both walls survive, above it
    // neither, and the contact is read along its length, not at its
    // midpoint, which lies on the top.
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let bores: Vec<Body> = [40.0, 50.0]
        .into_iter()
        .map(|y| {
            let axis = Axis::z_at(Point3::new(50.0, y, 4.0));
            primitive_cylinder(&mut m, axis, 5.0, 12.0).unwrap().0
        })
        .collect();
    let (_, reason) = refused(cut_many(&mut m, plate, &bores).unwrap_err());
    assert_eq!(reason, BooleanReason::TangentContact);
}

#[test]
fn tools_touching_along_a_seam_are_refused_as_non_manifold() {
    // The touch is the first hole's seam: an image of the seam on the
    // second wall, not a contact, and the two plate quadrants beside it
    // meet in an edge of four faces.
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let holes: Vec<Body> = [40.0, 50.0]
        .into_iter()
        .map(|x| {
            let axis = Axis::z_at(Point3::new(x, 50.0, -1.0));
            primitive_cylinder(&mut m, axis, 5.0, 12.0).unwrap().0
        })
        .collect();
    let e = cut_many(&mut m, plate, &holes).unwrap_err();
    assert!(
        matches!(
            e,
            OpError::Degenerate {
                reason: Reason::Input(arris_ops::InputReason::NonManifold),
                ..
            }
        ),
        "{e}"
    );
}

#[test]
fn pockets_flush_with_the_top_and_each_other_are_the_chain_of_cuts() {
    // Over the overlap the plate's top lies on both tools' tops at once:
    // the piece is decided on the face's two sides, void on both.
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let pocket = |m: &mut Model, x: f64| {
        primitive_box(
            m,
            Point3::new(x, 20.0, 5.0),
            Point3::new(x + 20.0, 40.0, 10.0),
        )
        .unwrap()
        .0
    };
    for (what, second) in [("overlapping", 30.0), ("repeated", 20.0)] {
        let tools = [pocket(&mut m, 20.0), pocket(&mut m, second)];
        let (many, _) = cut_many(&mut m, plate, &tools).unwrap();
        let chained = cut(&mut m, plate, tools[0]).unwrap().0;
        let chained = cut(&mut m, chained, tools[1]).unwrap().0;
        assert_same_solid(&m, many, chained, what);
    }
}

#[test]
fn a_piece_flush_with_two_tools_stands_for_both() {
    // Two overlapping boxes resting on the plate's top: the top under
    // both is kept from the plate, the lowest of the three, and stands
    // for both tools' bottoms.
    let mut m = Model::default();
    let (plate, _) = plate_and_holes(&mut m, 0);
    let tools: Vec<Body> = [20.0, 30.0]
        .into_iter()
        .map(|x| {
            primitive_box(
                &mut m,
                Point3::new(x, 20.0, 10.0),
                Point3::new(x + 20.0, 40.0, 15.0),
            )
            .unwrap()
            .0
        })
        .collect();
    let (body, p) = cut_many(&mut m, plate, &tools).unwrap();
    let volume = mass_properties(&m, body).unwrap().volume;
    assert!((volume - 1e5).abs() <= 1e-9 * 1e5, "{volume}");
    let bottom = |t: Body| {
        m.faces(t)
            .unwrap()
            .into_iter()
            .find(|f| {
                m.surface(m.face(f.id).unwrap().surface())
                    .unwrap()
                    .project(Point3::new(35.0, 30.0, 10.0))
                    .is_ok_and(|q| q.distance < 1e-9)
            })
            .unwrap()
            .id
    };
    let under = |g| -> Vec<Shape> {
        p.generated_from(shape(g))
            .iter()
            .copied()
            .filter(|s| matches!(s.id, arris_topo::EntityId::Face(_)))
            .collect()
    };
    let (first, second) = (under(bottom(tools[0])), under(bottom(tools[1])));
    let both: Vec<_> = first.iter().filter(|s| second.contains(s)).collect();
    assert_eq!(both.len(), 1, "{p}");
}
