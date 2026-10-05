//! `ops::offset_faces` (ADR-0048), the planar core: a box's face pushed
//! and pulled and the whole box out and in, at their closed forms, clean
//! at `Full` and audited with nothing generated or deleted; an L's inner
//! face, whose concave edges are recomputed; the input refusals, and the
//! refusals that keep topology, each leaving the model as it was.

use arris_check::classify::{Classification, classify_point};
use arris_check::{Level, check};
use arris_debug::dump_text;
use arris_debug::unmetered::{extrude, mass_properties, offset_faces, primitive_box};
use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_math::{Frame, Point2, Point3, Vec3};
use arris_ops::{InputReason, OffsetReason, OpError, Reason};
use arris_topo::provenance::{Relation, audit};
use arris_topo::{Body, EntityId, Face, Model};

/// The face of `body` that `at` lies inside.
fn face_at(m: &Model, body: Body, at: Point3) -> Face {
    match classify_point(m, body, at).unwrap() {
        Classification::On(s) => match s.id {
            EntityId::Face(id) => Face::forward(id),
            other => panic!("{at} is on {other}, not inside a face"),
        },
        other => panic!("{at} is {other:?}, on no face"),
    }
}

fn cube(m: &mut Model) -> Body {
    primitive_box(m, Point3::origin(), Point3::new(10.0, 10.0, 10.0))
        .unwrap()
        .0
}

/// The result is clean at `Full` with nothing unchecked, its record
/// audited, and nothing in it generated or deleted: topology is kept.
fn assert_clean(m: &Model, input: Body, out: Body, p: &arris_topo::Provenance) {
    let report = check(m, out, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    audit(m, &[input], out, p).unwrap();
    assert_eq!(p.deleted().count(), 0, "{p:?}");
    for s in p.outputs() {
        for (relation, _) in p.origins(s) {
            assert_eq!(relation, Relation::Modified, "{s}: {p:?}");
        }
    }
}

fn counts(m: &Model, body: Body) -> [usize; 3] {
    [
        m.vertices(body).unwrap().len(),
        m.edges(body).unwrap().len(),
        m.faces(body).unwrap().len(),
    ]
}

/// A box's top pushed and pulled: the walls extended or trimmed, the
/// bottom and its edges kept by id, the top `Modified` into a face on the
/// plane `d` above.
#[test]
fn a_box_top_moves_by_the_distance() {
    for d in [2.0, -2.0, 9.5] {
        let mut m = Model::default();
        let b = cube(&mut m);
        let top = face_at(&m, b, Point3::new(5.0, 5.0, 10.0));
        let bottom = face_at(&m, b, Point3::new(5.0, 5.0, 0.0));
        let (out, p) = offset_faces(&mut m, b, &[top], d).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        assert!(
            (mp.volume - 100.0 * (10.0 + d)).abs() < 1e-9,
            "{d}: {}",
            mp.volume
        );
        assert!((mp.centroid.z - (10.0 + d) / 2.0).abs() < 1e-9);
        assert_eq!(counts(&m, out), [8, 12, 6]);
        let new_top = face_at(&m, out, Point3::new(5.0, 5.0, 10.0 + d));
        assert_eq!(p.modified_from(top.shape()), vec![new_top.shape()]);
        assert!(
            m.faces(out).unwrap().contains(&bottom),
            "the bottom kept by id"
        );
    }
}

/// Every face of a box moved: an outward offset meets at sharp edges, a
/// 12-cube; inward, an 8-cube.
#[test]
fn the_whole_box_offsets_with_sharp_joins() {
    for (d, side) in [(1.0, 12.0), (-1.0, 8.0)] {
        let mut m = Model::default();
        let b = cube(&mut m);
        let faces = m.faces(b).unwrap();
        let (out, p) = offset_faces(&mut m, b, &faces, d).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        assert!(
            (mp.volume - side * side * side).abs() < 1e-9,
            "{}",
            mp.volume
        );
        assert!((mp.area - 6.0 * side * side).abs() < 1e-9);
        assert!((mp.centroid - Point3::new(5.0, 5.0, 5.0)).norm() < 1e-9);
    }
}

/// An extruded L, 20 × 20 less its 10 × 10 corner and 10 tall, its inner
/// face at x = 10 pushed and pulled by 3: the short arm's top trimmed or
/// extended, the area unchanged but for the caps.
#[test]
fn an_inner_face_moves_its_concave_edges() {
    for d in [3.0, -3.0] {
        let mut m = Model::default();
        let b = l_prism(&mut m);
        let inner = face_at(&m, b, Point3::new(10.0, 15.0, 5.0));
        let (out, p) = offset_faces(&mut m, b, &[inner], d).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        assert!((mp.volume - (300.0 + 10.0 * d) * 10.0).abs() < 1e-9);
        assert!((mp.area - (1400.0 + 20.0 * d)).abs() < 1e-9);
        assert_eq!(counts(&m, out), [12, 18, 8]);
        face_at(&m, out, Point3::new(10.0 + d, 15.0, 5.0));
    }
}

fn l_prism(m: &mut Model) -> Body {
    let p = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: [
                p(20.0, 0.0),
                p(20.0, 10.0),
                p(10.0, 10.0),
                p(10.0, 20.0),
                p(0.0, 20.0),
                p(0.0, 0.0),
            ]
            .map(ProfileSegment::LineTo)
            .to_vec(),
        },
        holes: vec![],
    };
    extrude(m, &profile, Vec3::z(), 10.0).unwrap().0
}

/// The refusal `reason` naming `entity`, and the model as it was.
fn assert_refused(
    m: &mut Model,
    b: Body,
    faces: &[Face],
    d: f64,
    reason: Reason,
    entity: impl Into<EntityId>,
) {
    let before = dump_text(m, b).unwrap();
    match offset_faces(m, b, faces, d) {
        Err(OpError::Degenerate {
            entities,
            reason: r,
        }) => {
            assert_eq!(r, reason);
            assert_eq!(entities[0].id, entity.into(), "{entities:?}");
        }
        other => panic!("expected {reason}, got {other:?}"),
    }
    assert_eq!(dump_text(m, b).unwrap(), before);
}

#[test]
fn the_inputs_are_refused_by_name() {
    let mut m = Model::default();
    let b = cube(&mut m);
    let top = face_at(&m, b, Point3::new(5.0, 5.0, 10.0));
    let nan = Reason::Input(InputReason::NonFinite { what: "distance" });
    assert_refused(&mut m, b, &[top], f64::NAN, nan, b.id);
    let zero = Reason::Input(InputReason::NotPositive {
        what: "|distance|",
        value: 0.0,
    });
    assert_refused(&mut m, b, &[top], 0.0, zero, b.id);
    let none = Reason::Offset(OffsetReason::NoFaces);
    assert_refused(&mut m, b, &[], 1.0, none, b.id);
    let twice = Reason::Offset(OffsetReason::RepeatedFace);
    assert_refused(&mut m, b, &[top, top], 1.0, twice, top.id);
    let other = cube(&mut m);
    let foreign = face_at(&m, other, Point3::new(5.0, 5.0, 10.0));
    let not_in = Reason::Offset(OffsetReason::FaceNotInBody);
    assert_refused(&mut m, b, &[foreign], 1.0, not_in, foreign.id);
}

/// A box's top pulled through its bottom: the walls' vertical edges would
/// end before they start.
#[test]
fn a_face_pulled_through_its_opposite_vanishes() {
    let mut m = Model::default();
    let b = cube(&mut m);
    let top = face_at(&m, b, Point3::new(5.0, 5.0, 10.0));
    let vanishes = Reason::Offset(OffsetReason::Vanishes);
    let OpError::Degenerate { reason, entities } =
        offset_faces(&mut m, b, &[top], -10.0).unwrap_err()
    else {
        panic!()
    };
    assert_eq!(reason, vanishes);
    assert!(matches!(entities[0].id, EntityId::Edge(_)), "{entities:?}");
}

/// A square pyramid with one side pushed: the apex's four faces no
/// longer meet in one point.
#[test]
fn a_pyramid_apex_splits() {
    let mut m = Model::default();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 0.0),
        Point3::new(0.0, 10.0, 0.0),
        Point3::new(5.0, 5.0, 10.0),
    ];
    let faces = vec![
        vec![vec![0, 3, 2, 1]],
        vec![vec![0, 1, 4]],
        vec![vec![1, 2, 4]],
        vec![vec![2, 3, 4]],
        vec![vec![3, 0, 4]],
    ];
    let (builder, keys) = arris_debug::polyhedron::polyhedron(&mut m, &points, &faces, 1).unwrap();
    let b = arris_ops::build(&mut m, builder, &keys, &arris_ops::Control::NONE)
        .unwrap()
        .0;
    let side = face_at(&m, b, Point3::new(5.0, 5.0 / 3.0, 10.0 / 3.0));
    let apex = m
        .vertices(b)
        .unwrap()
        .into_iter()
        .find(|v| (m.vertex(v.id).unwrap().point() - points[4]).norm() < 1e-9)
        .unwrap()
        .id;
    let splits = Reason::Offset(OffsetReason::VertexSplits);
    assert_refused(&mut m, b, &[side], 1.0, splits, apex);
    // Every side moved together keeps them meeting: the apex rises.
    let sides: Vec<Face> = m
        .faces(b)
        .unwrap()
        .into_iter()
        .filter(|f| *f != face_at(&m, b, Point3::new(5.0, 5.0, 0.0)))
        .collect();
    let (out, p) = offset_faces(&mut m, b, &sides, 1.0).unwrap();
    assert_clean(&m, b, out, &p);
}
