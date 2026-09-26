//! `ops::build` (ADR-0028): a consumer's
//! own topology — a tetrahedron by the Euler operators, a square frame
//! (genus 1) by `Builder::assemble` — finished into a checker-green solid
//! whose every entity is `Generated` from exactly the consumer's key for
//! its slot; and every refusal typed, with the model as it was.

use std::collections::BTreeMap;

use arris_debug::polyhedron::{edge_key, polyhedron};
use arris_ops::arris_check::arris_topo::arris_geom::{Curve, Curve2, Surface};
use arris_ops::arris_check::arris_topo::arris_math::{
    Frame, Interval, Point2, Point3, UnitVec2, UnitVec3, Vec2, Vec3,
};
use arris_ops::arris_check::arris_topo::builder::{
    Assembly, Builder, FaceRef, FaceSpec, Position, Seed, Split, Strut,
};
use arris_ops::arris_check::arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_ops::arris_check::arris_topo::provenance::{ConsumerKey, Origin, Relation, audit};
use arris_ops::arris_check::arris_topo::{
    Body, EntityId, Model, Orientation, Provenance, Role, Shape,
};
use arris_ops::arris_check::{Level, check};
use arris_ops::{BuildKeys, BuildSlot, OpError, Rejection, build, primitive_box};

const NS: u32 = 7;

/// The tetrahedron over the origin and the three unit points, by the
/// Euler operators: `mvfs` at `a`, struts to `b` and `c`, a lid closed
/// over `c → a`, a strut from `a` up to `d`, and two `mef`s cutting the
/// sides `bcd` and `acd` out of the lid, which is left as `abd`. With
/// `flip` the face `bcd`'s plane faces into the solid.
fn tetrahedron(m: &mut Model, flip: bool) -> (Builder, BuildKeys) {
    let tol = m.precision().default_tolerance;
    let [a, b, c, d] = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
    ];
    let bcd = Vec3::new(1.0, 1.0, 1.0) * if flip { -1.0 } else { 1.0 };
    let frames = [
        Frame::new(a, -Vec3::z(), Vec3::x()).unwrap(),
        Frame::new(a, -Vec3::y(), Vec3::x()).unwrap(),
        Frame::new(b, bcd, c - b).unwrap(),
        Frame::new(a, -Vec3::x(), Vec3::y()).unwrap(),
    ];
    let [s_abc, s_abd, s_bcd, s_acd] = frames.map(|frame| m.add_surface(Surface::Plane { frame }));
    let line = |m: &mut Model, p: Point3, q: Point3| {
        let curve = m.add_curve(Curve::Line {
            origin: p,
            direction: UnitVec3::new_normalize(q - p),
        });
        EdgeGeometry::Curve {
            curve,
            range: Interval::new(0.0, (q - p).norm()).unwrap(),
        }
    };
    let strut = |m: &mut Model, p, q| Strut {
        point: q,
        geometry: line(m, p, q),
        pcurves: [None, None],
    };
    let split = |m: &mut Model, p, q, surface| Split {
        geometry: line(m, p, q),
        surface,
        orientation: Orientation::Forward,
        pcurves: [None, None],
    };
    let mut bd = Builder::new(tol);
    let (va, f_abc) = bd
        .mvfs(Seed {
            point: a,
            surface: s_abc,
            orientation: Orientation::Forward,
        })
        .unwrap();
    let at = bd.find_position(f_abc, 0, va).unwrap();
    let (vb, _) = bd.mev(at, strut(m, a, b)).unwrap();
    let at = bd.find_position(f_abc, 0, vb).unwrap();
    let (vc, _) = bd.mev(at, strut(m, b, c)).unwrap();
    let from = bd.find_position(f_abc, 0, vc).unwrap();
    let to = bd.find_position(f_abc, 0, va).unwrap();
    let (_, lid) = bd.mef(from, to, split(m, c, a, s_abd)).unwrap();
    let at = bd.find_position(lid, 0, va).unwrap();
    let (vd, _) = bd.mev(at, strut(m, a, d)).unwrap();
    let from = bd.find_position(lid, 0, vd).unwrap();
    let to = bd.find_position(lid, 0, vb).unwrap();
    let (_, rest) = bd.mef(from, to, split(m, d, b, s_bcd)).unwrap();
    let from = bd.find_position(rest, 0, vd).unwrap();
    let to = bd.find_position(rest, 0, vc).unwrap();
    bd.mef(from, to, split(m, d, c, s_acd)).unwrap();
    // Every pcurve: the edge's line in its face's plane.
    let frame_of = |s| {
        frames[[s_abc, s_abd, s_bcd, s_acd]
            .iter()
            .position(|&x| x == s)
            .unwrap()]
    };
    let mut wanted = Vec::new();
    for (f, face) in bd.faces() {
        let frame = frame_of(face.surface());
        for (li, lp) in face.loops().iter().enumerate() {
            for (ci, u) in lp.uses().iter().enumerate() {
                let e = bd.edge(u.edge).unwrap();
                let p = frame.to_local(bd.vertex(e.start()).unwrap().point());
                let q = frame.to_local(bd.vertex(e.end()).unwrap().point());
                wanted.push((Position::new(f, li, ci), p, q));
            }
        }
    }
    for (at, p, q) in wanted {
        let pcurve = m.add_curve2(Curve2::Line {
            origin: Point2::new(p.x, p.y),
            direction: UnitVec2::new_normalize(Vec2::new(q.x - p.x, q.y - p.y)),
        });
        bd.set_pcurve(at, pcurve).unwrap();
    }
    // Keys: vertices 0..4 in a, b, c, d order, edges by their slot's
    // position plus 100, faces by the surface's position plus 200.
    let vertex_key = |v| [va, vb, vc, vd].iter().position(|&x| x == v).unwrap() as u64;
    let keys = BuildKeys {
        namespace: NS,
        vertices: bd.vertices().map(|(v, _)| (v, vertex_key(v))).collect(),
        edges: bd
            .edges()
            .enumerate()
            .map(|(i, (e, _))| (e, 100 + i as u64))
            .collect(),
        faces: bd
            .faces()
            .map(|(f, face)| {
                let i = [s_abc, s_abd, s_bcd, s_acd]
                    .iter()
                    .position(|&x| x == face.surface())
                    .unwrap();
                (f, 200 + i as u64)
            })
            .collect(),
        shells: vec![300],
        body: 400,
    };
    (bd, keys)
}

/// The square prism `[0, 3]² × [0, 1]` with the through-hole `[1, 2]²`,
/// as a polyhedron: ten faces, the top and bottom each with a hole loop.
fn frame_faces() -> (Vec<Point3>, Vec<Vec<Vec<usize>>>) {
    let square = |lo: f64, hi: f64, z: f64| {
        [
            Point3::new(lo, lo, z),
            Point3::new(hi, lo, z),
            Point3::new(hi, hi, z),
            Point3::new(lo, hi, z),
        ]
    };
    let mut points = Vec::new();
    points.extend(square(0.0, 3.0, 0.0)); // 0..4 outer bottom
    points.extend(square(0.0, 3.0, 1.0)); // 4..8 outer top
    points.extend(square(1.0, 2.0, 0.0)); // 8..12 hole bottom
    points.extend(square(1.0, 2.0, 1.0)); // 12..16 hole top
    let mut faces = vec![
        vec![vec![0, 3, 2, 1], vec![8, 9, 10, 11]],
        vec![vec![4, 5, 6, 7], vec![12, 15, 14, 13]],
    ];
    for i in 0..4 {
        let j = (i + 1) % 4;
        faces.push(vec![vec![i, j, 4 + j, 4 + i]]);
        faces.push(vec![vec![8 + j, 8 + i, 12 + i, 12 + j]]);
    }
    (points, faces)
}

/// Every entity of `body` is `Generated` from exactly one role, the
/// consumer's key its slot was given; and the record passes `audit`.
fn assert_keyed(m: &Model, body: Body, p: &Provenance) {
    assert!(check(m, body, Level::Full).is_ok());
    audit(m, &[], body, p).unwrap();
    let c = m.closure(body).unwrap();
    let mut outputs: Vec<Shape> = Vec::new();
    outputs.extend(
        c.vertices
            .iter()
            .map(|&v| Shape::new(v, Orientation::Forward)),
    );
    outputs.extend(c.edges.iter().map(|&e| Shape::new(e, Orientation::Forward)));
    outputs.extend(c.faces.iter().map(|&f| Shape::new(f, Orientation::Forward)));
    outputs.extend(
        c.shells
            .iter()
            .map(|&s| Shape::new(s, Orientation::Forward)),
    );
    outputs.push(Shape::new(body.id, Orientation::Forward));
    assert_eq!(p.outputs().len(), outputs.len());
    for s in outputs {
        match p.origins(s).as_slice() {
            [(Relation::Generated, Origin::Role(Role::Consumer(k)))] => {
                assert_eq!(k.namespace, NS, "{s}");
            }
            other => panic!("{s} has origins {other:?}"),
        }
    }
}

fn key(k: u64) -> Role {
    Role::Consumer(ConsumerKey {
        namespace: NS,
        key: k,
    })
}

#[test]
fn a_tetrahedron_by_euler_operators_is_generated_from_its_keys() {
    let mut m = Model::default();
    let (bd, keys) = tetrahedron(&mut m, false);
    let (vertex_of, face_of): (BTreeMap<_, _>, BTreeMap<_, _>) =
        (keys.vertices.clone(), keys.faces.clone());
    let (body, p) = build(&mut m, bd, &keys).unwrap();
    assert_keyed(&m, body, &p);
    assert_eq!(m.faces(body).unwrap().len(), 4);
    assert_eq!(m.edges(body).unwrap().len(), 6);
    assert_eq!(p.generated_from(key(400)), [Shape::from(body)]);
    assert_eq!(p.generated_from(key(300)).len(), 1);
    for k in 0..4 {
        assert_eq!(p.generated_from(key(k)).len(), 1, "vertex {k}");
        assert_eq!(p.generated_from(key(200 + k)).len(), 1, "face {k}");
    }
    for k in 100..106 {
        assert_eq!(p.generated_from(key(k)).len(), 1, "edge {k}");
    }
    // The vertex keyed `3` is `d`.
    let d = p.generated_from(key(3))[0];
    let EntityId::Vertex(d) = d.id else {
        panic!("{d} is not a vertex")
    };
    let point = m.vertex(d).unwrap().point();
    assert_eq!(point, Point3::new(0.0, 0.0, 1.0));
    assert_eq!((vertex_of.len(), face_of.len()), (4, 4));
}

#[test]
fn a_frame_by_assemble_is_generated_from_its_keys() {
    let mut m = Model::default();
    let (points, faces) = frame_faces();
    let (bd, keys) = polyhedron(&mut m, &points, &faces, NS).unwrap();
    let (body, p) = build(&mut m, bd, &keys).unwrap();
    assert_keyed(&m, body, &p);
    assert!(
        arris_debug::euler_line(&m, body)
            .unwrap()
            .ends_with("g1 = 0"),
        "genus 1"
    );
    assert_eq!(m.faces(body).unwrap().len(), 10);
    // Each key's outputs are its slots, however many share it: the body
    // and its shell are `0` with point `0` and face `0`, and the edge
    // `{0, 1}` is `0 << 32 | 1`, which is point `1`'s and face `1`'s too.
    let mut slots: BTreeMap<u64, usize> = BTreeMap::new();
    for k in keys
        .vertices
        .values()
        .chain(keys.edges.values())
        .chain(keys.faces.values())
        .chain(&keys.shells)
        .chain([&keys.body])
    {
        *slots.entry(*k).or_default() += 1;
    }
    for (&k, &n) in &slots {
        assert_eq!(p.generated_from(key(k)).len(), n, "key {k}");
    }
    assert_eq!(slots[&0], 4);
    assert_eq!(slots[&1], 3);
    assert_eq!(p.generated_from(key(edge_key(13, 12))).len(), 1);
}

/// A key missing for a live slot is `Unkeyed` naming it; the model is as
/// it was.
#[test]
fn a_missing_key_is_unkeyed_and_the_model_untouched() {
    let mut m = Model::default();
    let (bd, mut keys) = tetrahedron(&mut m, false);
    let (&f, _) = keys.faces.iter().nth(2).unwrap();
    keys.faces.remove(&f);
    let before = format!("{m:?}");
    match build(&mut m, bd, &keys) {
        Err(OpError::Unkeyed { slot }) => assert_eq!(slot, BuildSlot::Face(f)),
        other => panic!("{other:?}"),
    }
    assert_eq!(format!("{m:?}"), before);

    let (bd, mut keys) = tetrahedron(&mut m, false);
    keys.shells.clear();
    let before = format!("{m:?}");
    assert!(matches!(
        build(&mut m, bd, &keys),
        Err(OpError::Unkeyed {
            slot: BuildSlot::Shell(0)
        })
    ));
    assert_eq!(format!("{m:?}"), before);
}

/// A face whose plane faces into the solid is refused by the checker —
/// in every build profile, since the body is the consumer's input — and
/// the model is as it was.
#[test]
fn a_flipped_face_is_rejected_and_the_model_untouched() {
    let mut m = Model::default();
    let (bd, keys) = tetrahedron(&mut m, true);
    let before = format!("{m:?}");
    match build(&mut m, bd, &keys) {
        Err(OpError::Rejected(Rejection::Checker(report))) => assert!(!report.is_ok()),
        other => panic!("{other:?}"),
    }
    assert_eq!(format!("{m:?}"), before);
}

/// `finish`'s refusal is the consumer's: `Rejected`, not a kernel bug.
#[test]
fn a_builder_refusal_is_rejected_and_the_model_untouched() {
    let mut m = Model::default();
    let (points, faces) = frame_faces();
    let (mut bd, keys) = polyhedron(&mut m, &points, &faces, NS).unwrap();
    // Drop a pcurve's worth of the builder's work: an extra `mev` into a
    // face leaves a strut with no pcurves, which `finish` refuses.
    let (&f, _) = keys.faces.iter().next().unwrap();
    let (&v, _) = keys.vertices.iter().next().unwrap();
    let at = bd.find_position(f, 0, v).unwrap();
    let tip = Point3::new(0.5, 0.5, 0.0);
    let curve = m.add_curve(Curve::Line {
        origin: points[0],
        direction: UnitVec3::new_normalize(tip - points[0]),
    });
    let geometry = EdgeGeometry::Curve {
        curve,
        range: Interval::new(0.0, (tip - points[0]).norm()).unwrap(),
    };
    let (w, e) = bd
        .mev(
            at,
            Strut {
                point: tip,
                geometry,
                pcurves: [None, None],
            },
        )
        .unwrap();
    let mut keys = keys;
    keys.vertices.insert(w, 99);
    keys.edges.insert(e, 99);
    let before = format!("{m:?}");
    match build(&mut m, bd, &keys) {
        Err(OpError::Rejected(Rejection::Builder(_))) => {}
        other => panic!("{other:?}"),
    }
    assert_eq!(format!("{m:?}"), before);
}

/// A slot `assemble` kept from the model is another body's entity:
/// `build` makes a body from nothing, and refuses it.
#[test]
fn a_kept_slot_is_rejected() {
    let mut m = Model::default();
    let (boxed, _) = primitive_box(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let faces: Vec<FaceSpec> = m
        .faces(boxed)
        .unwrap()
        .into_iter()
        .map(FaceSpec::Keep)
        .collect();
    let tol = m.precision().default_tolerance;
    let (bd, slots) = Builder::assemble(
        &m,
        tol,
        Assembly {
            shells: vec![faces],
            ..Assembly::default()
        },
    )
    .unwrap();
    let keys = BuildKeys {
        namespace: NS,
        vertices: bd.vertices().map(|(v, _)| (v, 0)).collect(),
        edges: bd.edges().map(|(e, _)| (e, 0)).collect(),
        faces: slots.faces[0].iter().map(|&f: &FaceRef| (f, 0)).collect(),
        shells: vec![0],
        body: 0,
    };
    let before = format!("{m:?}");
    match build(&mut m, bd, &keys) {
        Err(OpError::Rejected(Rejection::Kept(BuildSlot::Vertex(_)))) => {}
        other => panic!("{other:?}"),
    }
    assert_eq!(format!("{m:?}"), before);
}

/// What `Level::Fast` passes and `build` still refuses, which is why it
/// checks at `Level::Full` (ADR-0028 §3): a hole whose walls cross the
/// outer ones, and a frame turned inside out — every loop reversed, so
/// every face faces into the material. Both are well-formed topology.
#[test]
fn what_the_fast_level_misses_is_rejected() {
    let mut m = Model::default();
    let (mut crossing, faces) = frame_faces();
    for i in [10, 11, 14, 15] {
        crossing[i].y = 4.0;
    }
    let (points, mut inside_out) = frame_faces();
    for l in inside_out.iter_mut().flatten() {
        l.reverse();
    }
    for (points, faces) in [(crossing, faces), (points, inside_out)] {
        let (bd, keys) = polyhedron(&mut m, &points, &faces, NS).unwrap();
        let mut scratch = m.clone();
        let unchecked = bd.clone().finish(&mut scratch, BodyKind::Solid).unwrap();
        assert!(check(&scratch, unchecked.body, Level::Fast).is_ok());
        let before = format!("{m:?}");
        match build(&mut m, bd, &keys) {
            Err(OpError::Rejected(Rejection::Checker(report))) => assert!(!report.is_ok()),
            other => panic!("{other:?}"),
        }
        assert_eq!(format!("{m:?}"), before);
    }
}
