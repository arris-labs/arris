//! Body bytes (ADR-0029, `docs/plans/body-bytes.md`): a body and its
//! record written and read back through both encodings, into a fresh
//! model and into one that already holds bodies; the same body gives the
//! same bytes from any model that holds it; what is not body bytes is
//! refused at the magic; every other refusal is typed and leaves the
//! reading model as it was; a plugin's cut comes back with its record
//! translatable into the ids of the model that sent the operands.

#[path = "body/guard.rs"]
mod guard;

use std::collections::BTreeSet;

use arris_debug::{dump_text, sample};
use arris_io::arris_check::arris_topo::arris_math::{Axis, Point2, Point3, Precision};
use arris_io::arris_check::arris_topo::provenance::ConsumerKey;
use arris_io::arris_check::arris_topo::{
    Body, EntityId, IdMap, Model, Origin, Provenance, Role, TopoError, VertexId,
};
use arris_io::arris_check::{Level, check};
use arris_io::body::{self, BODY_VERSION, BodyError, Imported};
use arris_io::native;
use arris_ops::measure::mass_properties;
use arris_ops::{cut, primitive_box, primitive_cylinder};

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
            assert_eq!(
                read.provenance, c.record,
                "{}: as the writer held it",
                c.name
            );
            assert_eq!(
                read.translated(&IdMap::default()),
                record,
                "{}: at the dense ids",
                c.name
            );
            assert!(read.foreign().is_empty(), "{}", c.name);
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
            assert_eq!(read.map.bodies[&c.body.id], read.body.id);
            assert_eq!(
                read.translated(&IdMap::default()),
                c.record.mapped(&read.map),
                "{}",
                c.name
            );
            let _ = record;
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
fn the_same_body_is_the_same_dense_model_from_a_model_full_of_holes() {
    let mut holey = Model::default();
    let cube = sample::unit_box(&mut holey).unwrap();
    let (cyl, record) =
        primitive_cylinder(&mut holey, Axis::z_at(Point3::origin()), 4.0, 12.0).unwrap();
    holey.retain(&[cyl]).unwrap();
    let _ = cube;

    let mut dense = Model::default();
    let (copy, map) = dense.import(&holey, cyl).unwrap();
    let dense_record = record.mapped(&map);
    // The geometry and topology written are the same; the record and the
    // map are each writer's own ids, so they differ.
    let tree = |text: String| serde_json::from_str::<serde_json::Value>(&text).unwrap();
    let a = tree(body::to_json(&holey, cyl, &record).unwrap());
    let b = tree(body::to_json(&dense, copy, &dense_record).unwrap());
    assert_eq!(a["model"], b["model"]);
    assert_eq!(a["body"], b["body"]);
    assert_ne!(a["map"], b["map"]);
    // And both read to the same body with the same record in the reader's
    // ids.
    let mut ra = Model::default();
    let from_holey = body::read(&mut ra, &body::write(&holey, cyl, &record).unwrap()).unwrap();
    let mut rb = Model::default();
    let from_dense =
        body::read(&mut rb, &body::write(&dense, copy, &dense_record).unwrap()).unwrap();
    assert_eq!(
        dump_text(&ra, from_holey.body).unwrap(),
        dump_text(&rb, from_dense.body).unwrap()
    );
    assert_eq!(
        from_holey.translated(&IdMap::default()),
        from_dense.translated(&IdMap::default())
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
    assert!(matches!(
        body::from_json(&mut b, "not json"),
        Err(BodyError::Decode(_))
    ));
    assert_eq!(native::to_bytes(&b).unwrap(), before, "nothing appended");
}

/// A model that already holds a body, for the refusals to leave alone.
fn target() -> Model {
    let mut m = Model::default();
    sample::cylinder(&mut m, 2.0, 3.0).unwrap();
    m
}

/// The unit box's body JSON as a tree, to be broken on purpose.
fn box_tree() -> serde_json::Value {
    let mut m = Model::default();
    let b = sample::unit_box(&mut m).unwrap();
    serde_json::from_str(&body::to_json(&m, b, &Provenance::default()).unwrap()).unwrap()
}

/// The refusal of `text`, having asserted that it left the model as it
/// was, byte for byte.
fn refusal_of_json(text: &str) -> BodyError {
    let mut m = target();
    let before = native::to_bytes(&m).unwrap();
    let e = body::from_json(&mut m, text).unwrap_err();
    assert_eq!(
        native::to_bytes(&m).unwrap(),
        before,
        "{e}: the model changed"
    );
    e
}

/// The same for bytes.
fn refusal_of_bytes(bytes: &[u8]) -> BodyError {
    let mut m = target();
    let before = native::to_bytes(&m).unwrap();
    let e = body::read(&mut m, bytes).unwrap_err();
    assert_eq!(
        native::to_bytes(&m).unwrap(),
        before,
        "{e}: the model changed"
    );
    e
}

#[test]
fn a_body_turned_inside_out_is_rejected_by_the_checker() {
    let mut t = box_tree();
    t["model"]["bodies"][0]["value"]["shells"][0]["orientation"] = "Reversed".into();
    let e = refusal_of_json(&t.to_string());
    assert!(matches!(e, BodyError::Rejected(_)), "{e}");
}

#[test]
fn a_vertex_moved_off_its_edges_is_rejected_by_the_checker() {
    let mut t = box_tree();
    t["model"]["vertices"][0]["value"]["point"] = serde_json::json!([0.25, 0.25, 0.25]);
    let e = refusal_of_json(&t.to_string());
    let BodyError::Rejected(report) = &e else {
        panic!("{e}")
    };
    assert!(!report.is_ok());
}

#[test]
fn a_reference_to_nothing_is_a_topo_error_naming_it() {
    let mut t = box_tree();
    t["model"]["faces"][0]["value"]["surface"]["index"] = 99.into();
    let e = refusal_of_json(&t.to_string());
    assert!(matches!(e, BodyError::Topo(TopoError::NotFound(_))), "{e}");
    assert!(e.to_string().contains("99"), "{e}");
}

#[test]
fn a_truncated_stream_is_a_decode_error() {
    let mut m = Model::default();
    let b = sample::unit_box(&mut m).unwrap();
    let bytes = body::write(&m, b, &Provenance::default()).unwrap();
    for cut in [9, bytes.len() / 2, bytes.len() - 1] {
        let e = refusal_of_bytes(&bytes[..cut]);
        assert!(matches!(e, BodyError::Decode(_)), "cut at {cut}: {e}");
    }
    let text = body::to_json(&m, b, &Provenance::default()).unwrap();
    let e = refusal_of_json(&text[..text.len() / 2]);
    assert!(matches!(e, BodyError::Decode(_)), "{e}");
}

/// The unit box written under `p` and read into a default model.
fn read_under(p: Precision) -> Result<Imported, BodyError> {
    let mut w = Model::new(p).unwrap();
    let b = sample::unit_box(&mut w).unwrap();
    let bytes = body::write(&w, b, &Provenance::default()).unwrap();
    let mut m = target();
    let before = native::to_bytes(&m).unwrap();
    let r = body::read(&mut m, &bytes);
    if r.is_err() {
        assert_eq!(native::to_bytes(&m).unwrap(), before);
    }
    r
}

#[test]
fn a_tolerance_outside_the_readers_range_is_refused_never_rescaled() {
    let default = Precision::DEFAULT;
    let finer = Precision {
        default_tolerance: default.min_tolerance / 10.0,
        min_tolerance: default.min_tolerance / 100.0,
        ..default
    };
    let e = read_under(finer).unwrap_err();
    assert_eq!(
        e,
        BodyError::Precision {
            entity: EntityId::Vertex(VertexId::new(0, 0)),
            tolerance: finer.default_tolerance,
            min: default.min_tolerance,
            max: default.max_tolerance,
        }
    );
    let coarser = Precision {
        default_tolerance: default.max_tolerance * 10.0,
        max_tolerance: default.max_tolerance * 100.0,
        ..default
    };
    assert!(matches!(
        read_under(coarser),
        Err(BodyError::Precision { tolerance, .. }) if tolerance == coarser.default_tolerance
    ));
    // Another precision whose tolerances the reader can hold reads.
    let other = Precision {
        default_tolerance: 1e-5,
        angular_tolerance: 1e-10,
        ..default
    };
    assert!(read_under(other).is_ok());
}

#[test]
fn a_map_that_is_not_one_to_one_onto_the_body_is_refused() {
    let mut t = box_tree();
    t["map"]["faces"].as_array_mut().unwrap().pop();
    let e = refusal_of_json(&t.to_string());
    assert!(matches!(e, BodyError::Decode(_)), "{e}");
    let mut t = box_tree();
    t["map"]["faces"][0][1] = t["map"]["faces"][1][1].clone();
    let e = refusal_of_json(&t.to_string());
    assert!(matches!(e, BodyError::Decode(_)), "{e}");
}

/// Every entity of `body`, by id.
fn entities(m: &Model, body: Body) -> BTreeSet<EntityId> {
    let c = m.closure(body).unwrap();
    let mut out: BTreeSet<EntityId> = BTreeSet::from([body.id.into()]);
    out.extend(c.vertices.iter().map(|&v| EntityId::from(v)));
    out.extend(c.edges.iter().map(|&e| EntityId::from(e)));
    out.extend(c.faces.iter().map(|&f| EntityId::from(f)));
    out.extend(c.shells.iter().map(|&s| EntityId::from(s)));
    out
}

/// `extra`'s entity pairs added to `map`.
fn union(mut map: IdMap, extra: &IdMap) -> IdMap {
    map.vertices.extend(&extra.vertices);
    map.edges.extend(&extra.edges);
    map.faces.extend(&extra.faces);
    map.shells.extend(&extra.shells);
    map.bodies.extend(&extra.bodies);
    map
}

#[test]
fn a_plugins_cut_comes_back_with_its_record_in_the_senders_ids() {
    // The application's model A sends a frame and a bar to the plugin's
    // model B, which cuts one with the other and writes the result back.
    // A holds a box first, so its ids and B's differ.
    let mut a = Model::default();
    sample::unit_box(&mut a).unwrap();
    let frame = sample::frame(
        &mut a,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap();
    let (bar, _) = primitive_box(&mut a, [18.0, -1.0, -1.0], [22.0, 31.0, 11.0]).unwrap();
    let mut b = Model::default();
    let (b_frame, frame_map) = b.import(&a, frame).unwrap();
    let (b_bar, bar_map) = b.import(&a, bar).unwrap();
    let (b_cut, b_record) = cut(&mut b, b_frame, b_bar).unwrap();
    let bytes = body::write(&b, b_cut, &b_record).unwrap();

    // The same cut in A itself: what the record has to amount to.
    let mut here = a.clone();
    let (a_cut, a_record) = cut(&mut here, frame, bar).unwrap();

    let read = body::read(&mut a, &bytes).unwrap();
    let back = union(frame_map.inverse(), &bar_map.inverse());
    let operands: BTreeSet<EntityId> = entities(&a, frame)
        .union(&entities(&a, bar))
        .copied()
        .collect();
    assert!(!read.foreign().is_empty(), "a cut names its inputs");
    for s in read.foreign() {
        let t = back.map(s).unwrap_or_else(|| panic!("{s} was not sent"));
        assert!(operands.contains(&t.id), "{s} → {t} is not an operand's");
    }

    let mine = read.translated(&back);
    let body_ids = entities(&a, read.body);
    for s in mine.outputs() {
        assert!(body_ids.contains(&s.id), "{s} is not the read body's");
    }
    assert_eq!(
        mine.deleted().collect::<Vec<_>>(),
        a_record.deleted().collect::<Vec<_>>(),
        "the same operand entities deleted, in A's ids"
    );
    let origins = |p: &Provenance| p.origins_recorded().collect::<Vec<_>>();
    assert_eq!(
        origins(&mine),
        origins(&a_record),
        "the same origins, in A's ids"
    );
    for o in origins(&a_record) {
        assert_eq!(
            mine.generated_from(o).len(),
            a_record.generated_from(o).len(),
            "{o}"
        );
        assert_eq!(
            mine.modified_from(o).len(),
            a_record.modified_from(o).len(),
            "{o}"
        );
    }
    // And the body is the one the cut makes in A, once both are dense.
    let dense_dump = |m: &Model, body: Body| {
        let mut d = Model::default();
        let (copy, _) = d.import(m, body).unwrap();
        dump_text(&d, copy).unwrap()
    };
    assert_eq!(dense_dump(&a, read.body), dense_dump(&here, a_cut));
}
