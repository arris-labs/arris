//! Body bytes (ADR-0029): a body and its
//! record written and read back through both encodings, into a fresh
//! model and into one that already holds bodies; the same body gives the
//! same bytes from any model that holds it; what is not body bytes is
//! refused at the magic; every other refusal is typed and leaves the
//! reading model as it was; a plugin's cut comes back with its record
//! translatable into the ids of the model that sent the operands.

#[path = "body/guard.rs"]
mod guard;

use arris_debug::unmetered::{body_from_json, body_read};
use arris_debug::unmetered::{mass_properties, primitive_box, primitive_cylinder};
use std::collections::BTreeSet;

use arris_check::{Level, check};
use arris_debug::unmetered::cut;
use arris_debug::{dump_text, sample};
use arris_io::body::{self, BODY_VERSION, BodyError, Imported};
use arris_io::native;
use arris_math::{Axis, Point2, Point3, Precision};
use arris_topo::provenance::ConsumerKey;
use arris_topo::{Body, EntityId, IdMap, Model, Origin, Provenance, Role, TopoError, VertexId};

use arris_debug::corpus;
use arris_debug::prop::recipe::recipe;
use arris_debug::testing::fail;
use proptest::prelude::*;

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
    let from_bytes = body_read(&mut a, &bytes).unwrap();
    let mut b = target.clone();
    let from_text = body_from_json(&mut b, &text).unwrap();
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
    let from_holey = body_read(&mut ra, &body::write(&holey, cyl, &record).unwrap()).unwrap();
    let mut rb = Model::default();
    let from_dense =
        body_read(&mut rb, &body::write(&dense, copy, &dense_record).unwrap()).unwrap();
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
        body_read(&mut b, &native::to_bytes(&a).unwrap()),
        Err(BodyError::Magic)
    );
    assert_eq!(
        body_from_json(&mut b, &native::to_json(&a).unwrap()),
        Err(BodyError::Magic)
    );
    assert_eq!(body_read(&mut b, b""), Err(BodyError::Magic));
    assert!(matches!(
        body_from_json(&mut b, "not json"),
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
    let e = body_from_json(&mut m, text).unwrap_err();
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
    let e = body_read(&mut m, bytes).unwrap_err();
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
    let r = body_read(&mut m, &bytes);
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

    let read = body_read(&mut a, &bytes).unwrap();
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

#[test]
fn a_newer_version_is_refused_and_an_older_one_no_release_wrote_is_a_decode_error() {
    let mut m = Model::default();
    let b = sample::unit_box(&mut m).unwrap();
    let bytes = body::write(&m, b, &Provenance::default()).unwrap();
    // The version is the varint right after the magic, one byte below 128.
    assert_eq!(u32::from(bytes[8]), BODY_VERSION);
    let mut newer = bytes.clone();
    newer[8] = BODY_VERSION as u8 + 1;
    let found = BODY_VERSION + 1;
    let version = BodyError::Version {
        found,
        newest: BODY_VERSION,
    };
    assert_eq!(refusal_of_bytes(&newer), version);
    let mut older = bytes;
    older[8] = 0;
    assert!(matches!(refusal_of_bytes(&older), BodyError::Decode(_)));

    let mut t = box_tree();
    t["version"] = found.into();
    assert_eq!(refusal_of_json(&t.to_string()), version);
    t["version"] = 0.into();
    let e = refusal_of_json(&t.to_string());
    assert!(e.to_string().contains("no release wrote"), "{e}");
}

/// The part of a body's JSON that is its geometry and topology, not the
/// writer's ids: equal for the same body whatever model it was written
/// from.
fn dense_part(text: &str) -> Result<(serde_json::Value, serde_json::Value), TestCaseError> {
    let tree: serde_json::Value = serde_json::from_str(text).map_err(fail)?;
    Ok((tree["model"].clone(), tree["body"].clone()))
}

arris_debug::prop_shards! {
    /// Bodies drawn by the recipe strategy (booleans of posed boxes,
    /// cylinders, extrusions and blends) and their records, written and
    /// read into a fresh model and a populated one: checker green, the
    /// same counts, the same mass properties bit for bit, the record as
    /// written and equal to it mapped, and re-writing a read body gives
    /// the same geometry and topology, the same bytes from then on.
    every_drawn_body_round_trips_through_its_bytes
        [s0 s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 s14 s15] (r) = recipe() => {
        // A panic or a typed refusal building the draw is the
        // differential's to hold (tests/recipe.rs in arris-debug), not a
        // question about body bytes.
        let Ok(Ok(chain)) = std::panic::catch_unwind(|| corpus::build("generated/body", &r)) else {
            return Ok(());
        };
        let body = chain.result().ok_or_else(|| fail("no result"))?;
        let record = &chain.steps[&chain.result].provenance;
        let m = &chain.model;
        let closure = m.closure(body).map_err(fail)?;
        let measures = mass_properties(m, body).map_err(fail)?;
        let bytes = body::write(m, body, record).map_err(fail)?;
        let text = body::to_json(m, body, record).map_err(fail)?;
        prop_assert_eq!(&bytes, &body::write(m, body, record).map_err(fail)?);

        let mut populated = Model::default();
        sample::unit_box(&mut populated).map_err(fail)?;
        sample::cylinder(&mut populated, 2.0, 3.0).map_err(fail)?;
        for target in [Model::default(), populated] {
            let mut t = target.clone();
            let read = body_read(&mut t, &bytes).map_err(fail)?;
            let report = check(&t, read.body, Level::Full);
            prop_assert!(report.is_ok(), "{}", report);
            let c = t.closure(read.body).map_err(fail)?;
            prop_assert_eq!(
                (c.faces.len(), c.edges.len(), c.vertices.len(), c.shells.len()),
                (closure.faces.len(), closure.edges.len(), closure.vertices.len(), closure.shells.len())
            );
            prop_assert_eq!(mass_properties(&t, read.body).map_err(fail)?, measures);
            prop_assert_eq!(&read.provenance, record);
            prop_assert_eq!(read.translated(&IdMap::default()), record.mapped(&read.map));

            let mut u = target.clone();
            let from_text = body_from_json(&mut u, &text).map_err(fail)?;
            prop_assert_eq!(&from_text, &read);

            // Re-written from where it was read, the same body; its ids
            // are the reader's now, so the record and the map are not
            // the first bytes', and from there on nothing changes.
            let mine = read.translated(&IdMap::default());
            let again = body::to_json(&t, read.body, &mine).map_err(fail)?;
            prop_assert_eq!(dense_part(&again)?, dense_part(&text)?);
            let rewritten = body::write(&t, read.body, &mine).map_err(fail)?;
            let mut fresh = Model::default();
            let back = body_read(&mut fresh, &rewritten).map_err(fail)?;
            let twice = body::write(&fresh, back.body, &back.translated(&IdMap::default()))
                .map_err(fail)?;
            let mut fresh2 = Model::default();
            let back2 = body_read(&mut fresh2, &twice).map_err(fail)?;
            let thrice = body::write(&fresh2, back2.body, &back2.translated(&IdMap::default()))
                .map_err(fail)?;
            prop_assert_eq!(twice, thrice);
        }
        Ok(())
    }
}

/// The nightly of 2026-10-01 (seed `e016178c…`, shard s3 of the property
/// above): a revolved hexagon with a hole, cut and fused, whose result had
/// a hole loop crossing its outer loop, so the bytes were refused as an
/// invalid body. The fixture is `boolean/body-bytes-revolved-hole-loops-intersect`.
#[test]
fn a_fuse_whose_edge_runs_past_its_curves_domain_round_trips_through_its_bytes() {
    let dir = arris_debug::fixtures::corpus_root()
        .join("boolean/body-bytes-revolved-hole-loops-intersect");
    let fixture = arris_debug::fixtures::load(&dir).unwrap();
    let chain = corpus::build("regression/body-bytes", &fixture.recipe).unwrap();
    let body = chain.result().unwrap();
    let record = &chain.steps[&chain.result].provenance;
    let m = &chain.model;
    let report = check(m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    let closure = m.closure(body).unwrap();
    let measures = mass_properties(m, body).unwrap();
    let bytes = body::write(m, body, record).unwrap();
    let mut fresh = Model::default();
    let read = body_read(&mut fresh, &bytes).unwrap();
    let report = check(&fresh, read.body, Level::Full);
    assert!(report.is_ok(), "{report}");
    let c = fresh.closure(read.body).unwrap();
    assert_eq!(
        (
            c.faces.len(),
            c.edges.len(),
            c.vertices.len(),
            c.shells.len()
        ),
        (
            closure.faces.len(),
            closure.edges.len(),
            closure.vertices.len(),
            closure.shells.len()
        )
    );
    assert_eq!(mass_properties(&fresh, read.body).unwrap(), measures);
}

/// Found by the `body_read` fuzz target: the elliptic guard body with a
/// pcurve mutated so far off its face that the checker's loop sweep
/// discretises a ring of segments that all meet, and collects every pair
/// of them (4 GiB) only to ask whether there is one.
#[test]
#[ignore = "arris-check's L5 collects every crossing pair of a ring to test for one: an allocation of 4 GiB (backlog)"]
fn a_ring_whose_segments_all_meet_is_refused_without_exhausting_memory() {
    let bytes = include_bytes!("body/regression/loop-sweep-every-pair.bin");
    let e = refusal_of_bytes(bytes);
    assert!(matches!(e, BodyError::Rejected(_)), "{e}");
}
