//! Body bytes (ADR-0029, `docs/plans/body-bytes.md`): a body and its
//! record written and read back through both encodings, into a fresh
//! model and into one that already holds bodies; the same body gives the
//! same bytes from any model that holds it; what is not body bytes is
//! refused at the magic.

use arris_debug::{dump_text, sample};
use arris_io::arris_check::arris_topo::arris_math::{Axis, Point2, Point3};
use arris_io::arris_check::arris_topo::provenance::ConsumerKey;
use arris_io::arris_check::arris_topo::{Body, Model, Origin, Provenance, Role};
use arris_io::arris_check::{Level, check};
use arris_io::body::{self, BODY_VERSION, BodyError, Imported};
use arris_io::native;
use arris_ops::measure::mass_properties;
use arris_ops::{primitive_box, primitive_cylinder};

/// A body and the record it was written with.
struct Case {
    name: &'static str,
    model: Model,
    body: Body,
    record: Provenance,
}

/// A box and a cylinder with their primitives' records, the second
/// re-rooted at a consumer's key; the frame and the NURBS box with none.
fn cases() -> Vec<Case> {
    let mut out = Vec::new();

    let mut m = Model::default();
    let (b, record) = primitive_box(&mut m, [0.0, 0.0, 0.0], [40.0, 30.0, 10.0]).unwrap();
    out.push(Case {
        name: "box",
        model: m,
        body: b,
        record,
    });

    let mut m = Model::default();
    let (b, record) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 4.0, 12.0).unwrap();
    // The consumer's key for each role: its place in the record's order.
    let roles: Vec<Origin> = record.origins_recorded().collect();
    let key = |r: Role| {
        let place = roles.iter().position(|&o| o == Origin::Role(r)).unwrap();
        Role::Consumer(ConsumerKey {
            namespace: 3,
            key: place as u64,
        })
    };
    out.push(Case {
        name: "cylinder",
        model: m,
        body: b,
        record: record.rerooted(key),
    });

    let mut m = Model::default();
    let b = sample::frame(
        &mut m,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap();
    out.push(Case {
        name: "frame",
        model: m,
        body: b,
        record: Provenance::default(),
    });

    let mut m = Model::default();
    let b = sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(2.0, 3.0, 4.0)).unwrap();
    out.push(Case {
        name: "nurbs box",
        model: m,
        body: b,
        record: Provenance::default(),
    });
    out
}

/// The body alone in a fresh model and its record at the copy's ids:
/// what reading its bytes into a fresh model has to give.
fn dense(c: &Case) -> (Model, Body, Provenance) {
    let mut fresh = Model::new(c.model.precision()).unwrap();
    let (copy, map) = fresh.import(&c.model, c.body).unwrap();
    (fresh, copy, c.record.mapped(&map))
}

/// Both encodings of a case read into `target`.
fn both_read(c: &Case, target: &Model) -> [(Model, Imported); 2] {
    let bytes = body::write(&c.model, c.body, &c.record).unwrap();
    let text = body::to_json(&c.model, c.body, &c.record).unwrap();
    let mut a = target.clone();
    let from_bytes = body::read(&mut a, &bytes).unwrap();
    let mut b = target.clone();
    let from_text = body::from_json(&mut b, &text).unwrap();
    [(a, from_bytes), (b, from_text)]
}

#[test]
fn every_case_reads_into_a_fresh_model_as_its_dense_copy() {
    for c in cases() {
        let (fresh, copy, record) = dense(&c);
        let dump = dump_text(&fresh, copy).unwrap();
        let measures = mass_properties(&c.model, c.body).unwrap();
        for (m, read) in both_read(&c, &Model::default()) {
            assert_eq!(read.version, BODY_VERSION);
            assert_eq!(read.body, copy, "{}: the same ids as a dense copy", c.name);
            assert_eq!(dump_text(&m, read.body).unwrap(), dump, "{}", c.name);
            assert_eq!(read.provenance, record, "{}", c.name);
            assert_eq!(
                mass_properties(&m, read.body).unwrap(),
                measures,
                "{}: bit for bit",
                c.name
            );
            let report = check(&m, read.body, Level::Full);
            assert!(report.is_ok(), "{}: {report}", c.name);
        }
    }
}

#[test]
fn every_case_reads_into_a_populated_model_up_to_the_returned_map() {
    let mut populated = Model::default();
    let held = [
        sample::unit_box(&mut populated).unwrap(),
        sample::cylinder(&mut populated, 2.0, 3.0).unwrap(),
    ];
    for c in cases() {
        let (fresh, copy, record) = dense(&c);
        let dump = dump_text(&fresh, copy).unwrap();
        for (m, read) in both_read(&c, &populated) {
            assert_ne!(read.body, copy, "{}: not at the dense ids", c.name);
            assert_eq!(read.map.bodies[&copy.id], read.body.id);
            assert_eq!(read.provenance, record.mapped(&read.map), "{}", c.name);
            // The same body, once copied out densely again.
            let mut again = Model::default();
            let (back, _) = again.import(&m, read.body).unwrap();
            assert_eq!(dump_text(&again, back).unwrap(), dump, "{}", c.name);
            // What the model held is untouched.
            for &h in &held {
                assert_eq!(dump_text(&m, h).unwrap(), dump_text(&populated, h).unwrap());
            }
        }
    }
}

#[test]
fn the_same_body_gives_the_same_bytes_from_a_model_full_of_holes() {
    let mut holey = Model::default();
    let cube = sample::unit_box(&mut holey).unwrap();
    let (cyl, record) =
        primitive_cylinder(&mut holey, Axis::z_at(Point3::origin()), 4.0, 12.0).unwrap();
    holey.retain(&[cyl]).unwrap();
    let _ = cube;

    let mut dense = Model::default();
    let (copy, map) = dense.import(&holey, cyl).unwrap();
    let dense_record = record.mapped(&map);
    assert_eq!(
        body::write(&holey, cyl, &record).unwrap(),
        body::write(&dense, copy, &dense_record).unwrap()
    );
    assert_eq!(
        body::to_json(&holey, cyl, &record).unwrap(),
        body::to_json(&dense, copy, &dense_record).unwrap()
    );
}

#[test]
fn two_writes_of_one_body_are_identical() {
    for c in cases() {
        assert_eq!(
            body::write(&c.model, c.body, &c.record).unwrap(),
            body::write(&c.model, c.body, &c.record).unwrap(),
            "{}",
            c.name
        );
        assert_eq!(
            body::to_json(&c.model, c.body, &c.record).unwrap(),
            body::to_json(&c.model, c.body, &c.record).unwrap(),
            "{}",
            c.name
        );
    }
}

#[test]
fn a_native_model_is_not_body_bytes() {
    let mut a = Model::default();
    sample::unit_box(&mut a).unwrap();
    let mut b = Model::default();
    let before = native::to_bytes(&b).unwrap();
    assert_eq!(
        body::read(&mut b, &native::to_bytes(&a).unwrap()),
        Err(BodyError::Magic)
    );
    assert_eq!(
        body::from_json(&mut b, &native::to_json(&a).unwrap()),
        Err(BodyError::Magic)
    );
    assert_eq!(body::read(&mut b, b""), Err(BodyError::Magic));
    assert_eq!(body::from_json(&mut b, "not json"), Err(BodyError::Magic));
    assert_eq!(native::to_bytes(&b).unwrap(), before, "nothing appended");
}
