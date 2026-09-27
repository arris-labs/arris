//! `Role::Consumer` (ADR-0028): the key
//! round-trips through serde JSON and postcard, the variants before it
//! encode as they did before it was appended, it prints as
//! `consumer:{namespace}/{key}`, and it orders by `(namespace, key)`.

use arris_topo::provenance::{
    BoxPart, ConsumerKey, Coord, CylinderPart, FileEntity, Provenance, Role, Side, SweepPart,
};
use arris_topo::{FaceId, Orientation, Shape};

/// One role of each variant that existed before `Consumer`, with its
/// encodings written out by hand from the serde data model: postcard is
/// the variant index as a varint, then the fields; JSON is externally
/// tagged. A change to either is a break of the native format.
fn old_variants() -> Vec<(Role, &'static [u8], &'static str)> {
    vec![
        (
            Role::Box(BoxPart::Face(Coord::Z, Side::Max)),
            &[0, 2, 2, 1],
            r#"{"Box":{"Face":["Z","Max"]}}"#,
        ),
        (
            Role::Cylinder(CylinderPart::Wall),
            &[1, 2],
            r#"{"Cylinder":"Wall"}"#,
        ),
        (
            Role::Extrude(SweepPart::Side {
                loop_index: 1,
                segment: 2,
            }),
            &[2, 5, 1, 2],
            r#"{"Extrude":{"Side":{"loop_index":1,"segment":2}}}"#,
        ),
        (
            Role::Revolve(SweepPart::StartCap),
            &[3, 3],
            r#"{"Revolve":"StartCap"}"#,
        ),
        (
            Role::File(FileEntity {
                id: 300,
                instance: 1,
            }),
            &[4, 0xac, 0x02, 1],
            r#"{"File":{"id":300,"instance":1}}"#,
        ),
    ]
}

#[test]
fn the_variants_before_consumer_encode_as_they_did() {
    for (role, bytes, json) in old_variants() {
        assert_eq!(postcard::to_allocvec(&role).unwrap(), bytes, "{role}");
        assert_eq!(serde_json::to_string(&role).unwrap(), json, "{role}");
        assert_eq!(postcard::from_bytes::<Role>(bytes).unwrap(), role);
        assert_eq!(serde_json::from_str::<Role>(json).unwrap(), role);
    }
}

#[test]
fn a_consumer_key_round_trips() {
    let role = Role::Consumer(ConsumerKey {
        namespace: 7,
        key: u64::MAX,
    });
    let bytes = postcard::to_allocvec(&role).unwrap();
    assert_eq!(bytes[0], 5, "appended last: the next variant index");
    assert_eq!(postcard::from_bytes::<Role>(&bytes).unwrap(), role);
    let json = serde_json::to_string(&role).unwrap();
    assert_eq!(
        json,
        format!(r#"{{"Consumer":{{"namespace":7,"key":{}}}}}"#, u64::MAX)
    );
    assert_eq!(serde_json::from_str::<Role>(&json).unwrap(), role);

    // And inside a record, as an origin, in the native format's bytes.
    let mut p = Provenance::new();
    p.add_generated(role, Shape::new(FaceId::new(3, 0), Orientation::Forward));
    let back: Provenance = postcard::from_bytes(&postcard::to_allocvec(&p).unwrap()).unwrap();
    assert_eq!(back, p);
}

#[test]
fn a_consumer_key_prints_its_namespace_and_key() {
    let role = Role::Consumer(ConsumerKey {
        namespace: 3,
        key: 1 << 40,
    });
    assert_eq!(role.to_string(), "consumer:3/1099511627776");
}

#[test]
fn consumer_keys_order_by_namespace_then_key() {
    let k = |namespace, key| Role::Consumer(ConsumerKey { namespace, key });
    let mut roles = vec![k(2, 0), k(1, 9), k(1, 3), k(0, u64::MAX)];
    roles.sort();
    assert_eq!(roles, [k(0, u64::MAX), k(1, 3), k(1, 9), k(2, 0)]);
    // After every operation's own roles, as appended.
    for (old, _, _) in old_variants() {
        assert!(old < k(0, 0), "{old}");
    }
}
