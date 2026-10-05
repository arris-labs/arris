//! The native format (`docs/DATA-MODEL.md` §Native format): the box, the
//! cylinder and the frame round-trip through both encodings to a
//! byte-identical dump with the same ids; two writes are byte-identical;
//! a bumped version and a truncated stream are typed errors; a value that
//! fails its type's validation is refused on the way in.

use arris_check::{Level, check};
use arris_debug::{dump_text, sample};
use arris_io::native::{self, NATIVE_VERSION, NativeError};
use arris_math::{Point2, Point3};
use arris_topo::{Body, Model};

/// A model holding a box, a cylinder and a frame, with the NURBS probe
/// box for the B-spline arms.
fn three_bodies() -> (Model, Vec<Body>) {
    let mut m = Model::default();
    let bodies = vec![
        sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap(),
        sample::cylinder(&mut m, 4.0, 12.0).unwrap(),
        sample::frame(
            &mut m,
            Point3::origin(),
            Point3::new(40.0, 30.0, 10.0),
            Point2::new(10.0, 10.0),
            Point2::new(30.0, 20.0),
        )
        .unwrap(),
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(2.0, 3.0, 4.0)).unwrap(),
    ];
    (m, bodies)
}

fn dumps(m: &Model, bodies: &[Body]) -> Vec<String> {
    bodies.iter().map(|&b| dump_text(m, b).unwrap()).collect()
}

#[test]
fn every_body_round_trips_through_json_and_bytes_to_the_same_dump_and_ids() {
    let (m, bodies) = three_bodies();
    let before = dumps(&m, &bodies);
    let text = native::to_json(&m).unwrap();
    let bytes = native::to_bytes(&m).unwrap();
    for back in [
        native::from_json(&text).unwrap(),
        native::from_bytes(&bytes).unwrap(),
    ] {
        assert_eq!(back.precision(), m.precision());
        assert_eq!(
            dumps(&back, &bodies),
            before,
            "the same bodies under the same ids"
        );
        for &b in &bodies {
            let report = check(&back, b, Level::Full);
            assert!(report.is_ok(), "{report}");
        }
        // The indices were rebuilt: the seam has two uses in the wall.
        let c = back.closure(bodies[1]).unwrap();
        let seam = c.edges[1];
        assert_eq!(back.edge_uses(seam).unwrap().len(), 2);
        assert_eq!(back.edge_uses(seam).unwrap(), m.edge_uses(seam).unwrap());
        assert_eq!(
            back.vertex_edges(c.vertices[0]).unwrap(),
            m.vertex_edges(c.vertices[0]).unwrap()
        );
        assert_eq!(
            back.face_shells(c.faces[0]).unwrap(),
            m.face_shells(c.faces[0]).unwrap()
        );
    }
    // And again: a second round trip changes nothing at the byte level.
    let back = native::from_bytes(&bytes).unwrap();
    assert_eq!(native::to_bytes(&back).unwrap(), bytes);
    assert_eq!(native::to_json(&back).unwrap(), text);
}

#[test]
fn two_writes_are_byte_identical_and_the_next_id_is_preserved() {
    let (m, _) = three_bodies();
    assert_eq!(native::to_json(&m).unwrap(), native::to_json(&m).unwrap());
    assert_eq!(native::to_bytes(&m).unwrap(), native::to_bytes(&m).unwrap());
    let (n, _) = three_bodies();
    assert_eq!(
        native::to_bytes(&m).unwrap(),
        native::to_bytes(&n).unwrap(),
        "two builds"
    );
    let mut back = native::from_bytes(&native::to_bytes(&m).unwrap()).unwrap();
    let mut again = m.clone();
    let a = sample::cylinder(&mut back, 1.0, 1.0).unwrap();
    let b = sample::cylinder(&mut again, 1.0, 1.0).unwrap();
    assert_eq!(a, b, "the next id is the same as in the original");
    assert_eq!(dump_text(&back, a).unwrap(), dump_text(&again, b).unwrap());
}

#[test]
fn a_bumped_version_and_a_truncated_stream_are_typed_errors() {
    let (m, _) = three_bodies();
    let text = native::to_json(&m).unwrap();
    let bumped = text.replacen(
        &format!("\"version\":{NATIVE_VERSION},"),
        &format!("\"version\":{},", NATIVE_VERSION + 1),
        1,
    );
    assert_ne!(bumped, text);
    assert_eq!(
        native::from_json(&bumped).map(|_| ()),
        Err(NativeError::Version {
            found: NATIVE_VERSION + 1,
            supported: NATIVE_VERSION
        })
    );
    let bytes = native::to_bytes(&m).unwrap();
    let mut bumped = bytes.clone();
    bumped[0] = NATIVE_VERSION as u8 + 1;
    assert_eq!(
        native::from_bytes(&bumped).map(|_| ()),
        Err(NativeError::Version {
            found: NATIVE_VERSION + 1,
            supported: NATIVE_VERSION
        })
    );
    for cut in [1, bytes.len() / 2, bytes.len() - 1] {
        assert!(
            matches!(
                native::from_bytes(&bytes[..cut]),
                Err(NativeError::Decode(_))
            ),
            "cut at {cut}"
        );
    }
    assert!(matches!(
        native::from_bytes(&[]),
        Err(NativeError::Decode(_))
    ));
    assert!(matches!(
        native::from_json(&text[..text.len() / 2]),
        Err(NativeError::Decode(_))
    ));
    assert!(matches!(
        native::from_json("{}"),
        Err(NativeError::Decode(_))
    ));
}

#[test]
fn a_value_that_fails_its_validation_is_refused_on_the_way_in() {
    let (m, _) = three_bodies();
    let text = native::to_json(&m).unwrap();
    // An inconsistent precision.
    let bad = text.replacen("\"min_tolerance\":1e-12", "\"min_tolerance\":1.0", 1);
    assert_ne!(bad, text);
    let err = native::from_json(&bad).unwrap_err();
    assert!(
        matches!(err, NativeError::Decode(ref s) if s.contains("precision")),
        "{err}"
    );
    // A frame that is not orthonormal: the world frame's x turned into y.
    let bad = text.replacen(
        "\"x\":[1.0,0.0,0.0],\"y\":[0.0,1.0,0.0],\"z\":[0.0,0.0,1.0]",
        "\"x\":[0.0,1.0,0.0],\"y\":[0.0,1.0,0.0],\"z\":[0.0,0.0,1.0]",
        1,
    );
    assert_ne!(bad, text, "the cylinder's world frame is in the text");
    let err = native::from_json(&bad).unwrap_err();
    assert!(
        matches!(err, NativeError::Decode(ref s) if s.contains("orthonormal")),
        "{err}"
    );
    // A NURBS whose knots are too few for its degree.
    let bad = text.replacen(
        "\"degree\":1,\"knots\":[0.0,0.0,",
        "\"degree\":3,\"knots\":[0.0,0.0,",
        1,
    );
    assert_ne!(bad, text, "the NURBS probe box is in the text");
    assert!(matches!(
        native::from_json(&bad),
        Err(NativeError::Decode(_))
    ));
}

#[test]
fn a_retained_model_round_trips_with_its_freed_slots_and_next_ids() {
    let (mut m, bodies) = three_bodies();
    m.retain(&[bodies[0], bodies[2]]).unwrap();
    let bytes = native::to_bytes(&m).unwrap();
    let mut back = native::from_bytes(&bytes).unwrap();
    assert_eq!(
        dumps(&back, &[bodies[0], bodies[2]]),
        dumps(&m, &[bodies[0], bodies[2]])
    );
    assert!(
        back.body(bodies[1].id).is_err(),
        "the freed body stays freed"
    );
    let a = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let b = sample::cylinder(&mut back, 4.0, 12.0).unwrap();
    assert_eq!(a, b, "the freed slots are reused alike");
    assert_eq!(a.id.generation(), 1);
    assert_eq!(dump_text(&m, a).unwrap(), dump_text(&back, b).unwrap());
    assert_eq!(
        native::to_bytes(&m).unwrap(),
        native::to_bytes(&back).unwrap()
    );
}
