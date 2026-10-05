//! `ops::shell` (ADR-0049), the planar core: a box hollowed inward and
//! outward, open on top or closed into a void, at their closed forms,
//! clean at `Full` and audited — the walls kept and `Modified` into
//! themselves, their skin copies `Generated` from them, the opening
//! `Modified` into its rim; the input refusals, each leaving the model as
//! it was.

use arris_check::classify::{Classification, classify_point};
use arris_check::{Level, check};
use arris_debug::dump_text;
use arris_debug::unmetered::{mass_properties, primitive_box, primitive_cylinder, shell};
use arris_math::{Axis, Point3};
use arris_ops::{InputReason, OffsetReason, OpError, Reason, ShellReason, ShellSide};
use arris_topo::provenance::audit;
use arris_topo::{Body, EntityId, Face, Model, Orientation, Provenance, Shape};

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

/// The result is clean at `Full` with nothing unchecked and its record
/// audited.
fn assert_clean(m: &Model, input: Body, out: Body, p: &Provenance) {
    let report = check(m, out, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    audit(m, &[input], out, p).unwrap();
}

/// Open on top, both sides: the closed forms, the five walls kept by id,
/// each `Modified` into itself and one skin face `Generated` from it, the
/// top `Modified` into its rim, every vertex and edge of the body kept
/// with one copy each, nothing deleted.
#[test]
fn a_box_open_on_top_hollows_to_both_sides() {
    for (side, volume) in [
        (ShellSide::Inward, 1000.0 - 6.0 * 6.0 * 8.0),
        (ShellSide::Outward, 14.0 * 14.0 * 12.0 - 1000.0),
    ] {
        let mut m = Model::default();
        let b = cube(&mut m);
        let top = face_at(&m, b, Point3::new(5.0, 5.0, 10.0));
        let walls: Vec<Face> = m
            .faces(b)
            .unwrap()
            .into_iter()
            .filter(|f| f.id != top.id)
            .collect();
        let (out, p) = shell(&mut m, b, &[top], 2.0, side).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        assert!((mp.volume - volume).abs() < 1e-9, "{side:?}: {}", mp.volume);
        assert_eq!(m.faces(out).unwrap().len(), 11);
        assert_eq!(m.shells(out).unwrap().len(), 1);
        let faces: Vec<_> = m.faces(out).unwrap().into_iter().map(|f| f.id).collect();
        for wall in &walls {
            assert!(faces.contains(&wall.id), "{side:?}: {} kept by id", wall.id);
            assert_eq!(p.modified_from(wall.shape()), vec![wall.shape()]);
            assert_eq!(p.generated_from(wall.shape()).len(), 1, "{side:?}");
        }
        let on_rim = match side {
            ShellSide::Inward => Point3::new(1.0, 5.0, 10.0),
            ShellSide::Outward => Point3::new(-1.0, 5.0, 10.0),
        };
        let rim = face_at(&m, out, on_rim);
        assert_eq!(p.modified_from(top.shape()), vec![rim.shape()]);
        for v in m.vertices(b).unwrap() {
            let v = Shape::new(v.id, Orientation::Forward);
            assert_eq!(p.generated_from(v).len(), 1, "{side:?}: {v}");
        }
        assert_eq!(p.deleted().count(), 0, "{p:?}");
    }
}

/// No opening: a closed void, the second shell reversed inside the
/// first and `Generated` from the body.
#[test]
fn a_closed_box_hollows_to_a_void() {
    for (side, volume) in [
        (ShellSide::Inward, 1000.0 - 216.0),
        (ShellSide::Outward, 2744.0 - 1000.0),
    ] {
        let mut m = Model::default();
        let b = cube(&mut m);
        let (out, p) = shell(&mut m, b, &[], 2.0, side).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        assert!((mp.volume - volume).abs() < 1e-9, "{side:?}: {}", mp.volume);
        assert!((mp.centroid - Point3::new(5.0, 5.0, 5.0)).norm() < 1e-9);
        let shells = m.shells(out).unwrap();
        assert_eq!(shells.len(), 2);
        let from_body = p.generated_from(b.shape());
        assert_eq!(from_body.len(), 1, "{p:?}");
        assert!(shells.iter().any(|s| s.shape() == from_body[0]));
        let void = classify_point(&m, out, Point3::new(5.0, 5.0, 5.0)).unwrap();
        assert_eq!(void, Classification::Outside);
    }
}

/// The refusal `reason` naming `entity`, and the model as it was.
fn assert_refused(
    m: &mut Model,
    b: Body,
    openings: &[Face],
    thickness: f64,
    reason: Reason,
    entity: impl Into<EntityId>,
) {
    let before = dump_text(m, b).unwrap();
    match shell(m, b, openings, thickness, ShellSide::Inward) {
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
    let nan = Reason::Input(InputReason::NonFinite { what: "thickness" });
    assert_refused(&mut m, b, &[top], f64::NAN, nan, b.id);
    for t in [0.0, -1.0] {
        let not_positive = Reason::Input(InputReason::NotPositive {
            what: "thickness",
            value: t,
        });
        assert_refused(&mut m, b, &[top], t, not_positive, b.id);
    }
    let twice = Reason::Shell(ShellReason::RepeatedOpening);
    assert_refused(&mut m, b, &[top, top], 1.0, twice, top.id);
    let other = cube(&mut m);
    let foreign = face_at(&m, other, Point3::new(5.0, 5.0, 10.0));
    let not_in = Reason::Shell(ShellReason::OpeningNotInBody);
    assert_refused(&mut m, b, &[foreign], 1.0, not_in, foreign.id);
    let all = m.faces(b).unwrap();
    let no_walls = Reason::Shell(ShellReason::NoWalls);
    assert_refused(&mut m, b, &all, 1.0, no_walls, b.id);
}

/// A wall as thick as half the box: the skin's opposite faces meet, and
/// the offset refuses it by its own reason.
#[test]
fn a_wall_past_half_the_box_is_the_offsets_refusal() {
    let mut m = Model::default();
    let b = cube(&mut m);
    let top = face_at(&m, b, Point3::new(5.0, 5.0, 10.0));
    let before = dump_text(&m, b).unwrap();
    let err = shell(&mut m, b, &[top], 6.0, ShellSide::Inward).unwrap_err();
    assert!(
        matches!(
            err,
            OpError::Degenerate {
                reason: Reason::Offset(OffsetReason::Vanishes),
                ..
            }
        ),
        "{err:?}"
    );
    assert_eq!(dump_text(&m, b).unwrap(), before);
}

/// The entity of `body` that `at` lies on: a face, an edge or a vertex.
fn on(m: &Model, body: Body, at: Point3) -> Shape {
    match classify_point(m, body, at).unwrap() {
        Classification::On(s) => s,
        other => panic!("{at} is {other:?}, on nothing"),
    }
}

/// Open on top and front, which share an edge, both sides: each rim face
/// one loop, the shared edge gone and its two end pieces left between the
/// rims — inward on the body's edge, `Modified` from it; outward beyond
/// it on the skin's copy, `Generated` from it, the edge `Deleted`.
#[test]
fn two_openings_sharing_an_edge_merge_their_rims() {
    for (side, volume) in [
        (ShellSide::Inward, 1000.0 - 6.0 * 8.0 * 8.0),
        (ShellSide::Outward, 14.0 * 12.0 * 12.0 - 1000.0),
    ] {
        let mut m = Model::default();
        let b = cube(&mut m);
        let top = face_at(&m, b, Point3::new(5.0, 5.0, 10.0));
        let front = face_at(&m, b, Point3::new(5.0, 0.0, 5.0));
        let shared = on(&m, b, Point3::new(5.0, 0.0, 10.0));
        let (out, p) = shell(&mut m, b, &[top, front], 2.0, side).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        assert!((mp.volume - volume).abs() < 1e-9, "{side:?}: {}", mp.volume);
        assert_eq!(m.faces(out).unwrap().len(), 10);
        for opening in [top, front] {
            let rims = p.modified_from(opening.shape());
            assert_eq!(rims.len(), 1, "{side:?}");
            let EntityId::Face(rim) = rims[0].id else {
                panic!("{:?} is no face", rims[0]);
            };
            assert_eq!(m.face(rim).unwrap().loops().len(), 1, "{side:?}");
        }
        let edges: Vec<_> = m.edges(out).unwrap().into_iter().map(|e| e.id).collect();
        assert!(!edges.iter().any(|&e| EntityId::from(e) == shared.id));
        let (pieces, deleted) = match side {
            ShellSide::Inward => (p.modified_from(shared), false),
            ShellSide::Outward => (p.generated_from(shared), true),
        };
        assert_eq!(pieces.len(), 2, "{side:?}: {p:?}");
        assert_eq!(p.is_deleted(shared), deleted, "{side:?}");
        let mouth = Point3::new(5.0, 0.0, 10.0);
        assert_eq!(
            classify_point(&m, out, mouth).unwrap(),
            Classification::Outside
        );
    }
}

/// Open on three faces meeting at a corner: the corner is in no face, and
/// is `Deleted`.
#[test]
fn three_openings_at_a_corner_drop_it() {
    for (side, volume) in [
        (ShellSide::Inward, 1000.0 - 512.0),
        (ShellSide::Outward, 1728.0 - 1000.0),
    ] {
        let mut m = Model::default();
        let b = cube(&mut m);
        let openings = [
            face_at(&m, b, Point3::new(5.0, 5.0, 10.0)),
            face_at(&m, b, Point3::new(5.0, 0.0, 5.0)),
            face_at(&m, b, Point3::new(0.0, 5.0, 5.0)),
        ];
        let corner = on(&m, b, Point3::new(0.0, 0.0, 10.0));
        let (out, p) = shell(&mut m, b, &openings, 2.0, side).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        assert!((mp.volume - volume).abs() < 1e-9, "{side:?}: {}", mp.volume);
        assert_eq!(m.faces(out).unwrap().len(), 9);
        assert!(p.is_deleted(corner), "{side:?}: {p:?}");
    }
}

/// A cylinder open on its side and top: the side's rim meets itself
/// across the seam, the seam's piece left between the two sides of the
/// band, and the top's rim cancels whole — the skin's loop on it is the
/// body's — so the top is `Deleted`, and what is left is a disc as thick
/// as the wall.
#[test]
fn an_opening_meeting_itself_across_its_seam_merges() {
    for side in [ShellSide::Inward, ShellSide::Outward] {
        let mut m = Model::default();
        let (b, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 5.0, 10.0).unwrap();
        let wall = face_at(&m, b, Point3::new(0.0, 5.0, 5.0));
        let top = face_at(&m, b, Point3::new(1.0, 1.0, 10.0));
        let (out, p) = shell(&mut m, b, &[wall, top], 2.0, side).unwrap();
        assert_clean(&m, b, out, &p);
        let mp = mass_properties(&m, out).unwrap();
        let disc = std::f64::consts::PI * 25.0 * 2.0;
        assert!((mp.volume - disc).abs() < 1e-6, "{side:?}: {}", mp.volume);
        assert_eq!(m.faces(out).unwrap().len(), 3, "{side:?}");
        assert!(p.is_deleted(top.shape()), "{side:?}: {p:?}");
        assert!(p.modified_from(top.shape()).is_empty());
        assert_eq!(p.modified_from(wall.shape()).len(), 1, "{side:?}");
    }
}
