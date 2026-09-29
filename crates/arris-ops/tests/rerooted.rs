//! `Provenance::rerooted` (ADR-0028): a
//! record re-rooted at a consumer's keys is the same record under other
//! names — the identity changes nothing, an injective map is undone by
//! its inverse, a map that merges roles concatenates their outputs in
//! the old roles' order — and it stays a record of the operation:
//! `audit` holds on a re-rooted primitive's, sweep's and read solid's
//! record, and re-rooting commutes with `then`.

use arris_debug::unmetered::cut;
use arris_debug::unmetered::{extrude, primitive_box, primitive_cylinder};
use arris_io::step::{self, ReadOptions};
use arris_ops::arris_check::arris_topo::arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_ops::arris_check::arris_topo::arris_math::{Axis, Frame, Point2, Point3, Vec3};
use arris_ops::arris_check::arris_topo::provenance::{
    BoxPart, ConsumerKey, Coord, Origin, Side, audit,
};

use arris_ops::arris_check::arris_topo::{Body, EntityId, Model, Provenance, Role};
/// Every role the record has an origin at, ascending.
fn roles_of(p: &Provenance) -> Vec<Role> {
    p.origins_recorded()
        .filter_map(|o| match o {
            Origin::Role(r) => Some(r),
            Origin::Entity(_) => None,
        })
        .collect()
}

/// The consumer's map for its feature `feature`: each of the
/// operation's roles, by its position among `roles`, prefixed with the
/// feature — `feature << 32 | index` in namespace 9. Injective over
/// `roles`; any other role is left as it is.
fn prefixed(feature: u64, roles: &[Role]) -> impl Fn(Role) -> Role + '_ {
    move |r| match roles.iter().position(|&x| x == r) {
        Some(i) => Role::Consumer(ConsumerKey {
            namespace: 9,
            key: feature << 32 | i as u64,
        }),
        None => r,
    }
}

fn boxed(m: &mut Model) -> (Body, Provenance) {
    primitive_box(m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap()
}

#[test]
fn the_identity_changes_nothing() {
    let mut m = Model::default();
    let (_, p) = boxed(&mut m);
    assert_eq!(p.rerooted(|r| r), p);
}

#[test]
fn an_injective_map_is_undone_by_its_inverse() {
    let mut m = Model::default();
    let (_, p) = boxed(&mut m);
    let roles = roles_of(&p);
    assert_eq!(
        roles.len(),
        8 + 12 + 6 + 2,
        "every part, the shell and the body"
    );
    let there = p.rerooted(prefixed(5, &roles));
    assert!(
        roles_of(&there)
            .iter()
            .all(|r| matches!(r, Role::Consumer(_))),
        "every root is the consumer's"
    );
    assert_eq!(there.outputs(), p.outputs());
    let back = there.rerooted(|r| match r {
        Role::Consumer(k) if k.key >> 32 == 5 => roles[(k.key & 0xffff_ffff) as usize],
        other => other,
    });
    assert_eq!(back, p);
}

/// Every face role of the box merged into one key: its outputs are the
/// six faces, in the order of the roles they came from, each once.
#[test]
fn a_map_that_merges_roles_concatenates_in_the_old_roles_order() {
    let mut m = Model::default();
    let (_, p) = boxed(&mut m);
    let faces = Role::Consumer(ConsumerKey {
        namespace: 9,
        key: 0,
    });
    let merged = p.rerooted(|r| match r {
        Role::Box(BoxPart::Face(..)) => faces,
        other => other,
    });
    let expected: Vec<_> = roles_of(&p)
        .into_iter()
        .filter(|r| matches!(r, Role::Box(BoxPart::Face(..))))
        .flat_map(|r| p.generated_from(r).to_vec())
        .collect();
    assert_eq!(expected.len(), 6);
    assert_eq!(merged.generated_from(faces), expected);
    // A role already there, merged into: the body's key taking the shell
    // too keeps the body first, the shell after, as their roles sort.
    let one = Role::Box(BoxPart::Body);
    let both = merged.rerooted(|r| match r {
        Role::Box(BoxPart::Shell) => one,
        other => other,
    });
    assert_eq!(
        both.generated_from(one),
        [
            p.generated_from(one)[0],
            p.generated_from(Role::Box(BoxPart::Shell))[0]
        ]
    );
}

/// A re-rooted record accounts for the operation as the original did,
/// for every kind of creating operation: a primitive, a sweep, a file.
#[test]
fn audit_holds_on_re_rooted_records() {
    let mut m = Model::default();
    let (b, p) = boxed(&mut m);
    audit(&m, &[], b, &p.rerooted(prefixed(1, &roles_of(&p)))).unwrap();

    let (c, p) =
        primitive_cylinder(&mut m, Axis::z_at(Point3::new(9.0, 0.0, 0.0)), 1.0, 2.0).unwrap();
    audit(&m, &[], c, &p.rerooted(prefixed(2, &roles_of(&p)))).unwrap();

    let at = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: at(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(at(3.0, 0.0)),
                ProfileSegment::LineTo(at(3.0, 2.0)),
                ProfileSegment::LineTo(at(0.0, 2.0)),
                ProfileSegment::LineTo(at(0.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let (e, p) = extrude(&mut m, &profile, Vec3::z(), 1.5).unwrap();
    audit(&m, &[], e, &p.rerooted(prefixed(3, &roles_of(&p)))).unwrap();

    let text = step::write(&m, &[b]).unwrap();
    let mut read_into = Model::default();
    let read = step::read(&mut read_into, &text, &ReadOptions::default()).unwrap();
    let [solid] = read.solids.as_slice() else {
        panic!("one solid written, {} read", read.solids.len())
    };
    let body = solid.result.as_ref().unwrap();
    let rerooted = body
        .provenance
        .rerooted(prefixed(4, &roles_of(&body.provenance)));
    audit(&read_into, &[], body.body, &rerooted).unwrap();
    assert!(
        roles_of(&rerooted)
            .iter()
            .all(|r| matches!(r, Role::Consumer(_)))
    );
}

/// Re-root then compose is compose then re-root: a box's record re-rooted
/// and followed by a cut through its top is the box's record followed by
/// the cut, re-rooted — so a consumer can re-root when it records the
/// primitive or when it reads the chain, and gets one answer.
#[test]
fn rerooted_commutes_with_then() {
    let mut m = Model::default();
    let (b, p) = boxed(&mut m);
    let (tool, _) =
        primitive_cylinder(&mut m, Axis::z_at(Point3::new(2.0, 1.5, -1.0)), 0.5, 4.0).unwrap();
    let (out, q) = cut(&mut m, b, tool).unwrap();
    let roles = roles_of(&p);
    let f = prefixed(7, &roles);
    let early = p.rerooted(&f).then(&q);
    let late = p.then(&q).rerooted(&f);
    assert_eq!(early, late);
    // And the top, holed, ends at the consumer's key for the top, one
    // face (a piece of a generated entity is generated) beside the rim
    // the cut made on it.
    let top = Role::Box(BoxPart::Face(Coord::Z, Side::Max));
    let faces = late
        .generated_from(f(top))
        .iter()
        .filter(|s| matches!(s.id, EntityId::Face(_)))
        .count();
    assert_eq!(faces, 1);
    assert!(late.generated_from(top).is_empty());
    audit(&m, &[tool], out, &late).unwrap();
}
