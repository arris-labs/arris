//! Cancellation of the readers (ADR-0030). On the real-part fixtures the
//! steps an unbudgeted `step::read` takes are counted with a poll that
//! never stops it; a budget below that count stops the whole call with
//! `Interrupted` at exactly the budget and leaves the model's bytes as
//! they were, the solids already read included; a budget at the count
//! reads the same refusal table and the same dumps as the unbudgeted
//! read. `body::read` and `body::from_json` are held to the same.

use std::sync::atomic::{AtomicU64, Ordering};

use arris_debug::{dump, fixtures, sample};
use arris_io::step::{self, Read, ReadError, ReadOptions};
use arris_io::{body, native};
use arris_math::{Control, Interrupted, Stop};
use arris_topo::{Model, Provenance};

/// The steps `f` takes under a poll that never stops it.
fn steps_of<T>(f: impl FnOnce(&Control<'_>) -> T) -> (T, u64) {
    let asked = AtomicU64::new(0);
    let poll = || {
        asked.fetch_add(1, Ordering::Relaxed);
        false
    };
    let out = f(&Control::poll(&poll));
    (out, asked.load(Ordering::Relaxed))
}

fn below(n: u64) -> Vec<u64> {
    let mut ks: Vec<u64> = [0, 1, n / 4, n / 2, 3 * n / 4, n.saturating_sub(1)]
        .into_iter()
        .filter(|&k| k < n)
        .collect();
    ks.sort_unstable();
    ks.dedup();
    ks
}

/// The refusal table and the dump of every body of a read.
fn summary(m: &Model, read: &Read) -> String {
    let mut out = String::new();
    for s in &read.solids {
        out.push_str(&format!("{:?}: ", s.entity));
        match &s.result {
            Ok(b) => out.push_str(&dump::dump_text(m, b.body).unwrap()),
            Err(refusal) => out.push_str(&format!("refused: {refusal}")),
        }
        out.push('\n');
    }
    out
}

fn stopped(e: ReadError) -> Interrupted {
    match e {
        ReadError::Interrupted(stop) => stop,
        other => panic!("expected Interrupted, got {other}"),
    }
}

#[test]
fn a_budget_stops_step_read_at_the_same_step_and_leaves_the_model_as_it_was() {
    let root = fixtures::corpus_root().join("real");
    let mut files: Vec<_> = std::fs::read_dir(&root)
        .expect("the real-part fixtures")
        .flatten()
        .map(|e| e.path())
        .flat_map(|dir| std::fs::read_dir(dir).into_iter().flatten().flatten())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "stp"))
        .collect();
    // The smaller half: each file is read a handful of times, and the
    // larger parts take minutes in the test profile.
    files.sort_by_key(|p| (std::fs::metadata(p).map_or(0, |m| m.len()), p.clone()));
    files.truncate(10);
    assert!(files.len() >= 10, "{} files", files.len());
    let options = ReadOptions::default();
    let mut texts: Vec<(String, String)> = files
        .iter()
        .map(|f| (f.display().to_string(), std::fs::read_to_string(f).unwrap()))
        .collect();
    // A solid the reader refuses, beside the ones it reads: a cylinder
    // whose first oriented edge is turned, its loop no longer closing.
    let mut m = Model::default();
    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = step::write(&m, &[cylinder]).unwrap();
    let at = text.find("ORIENTED_EDGE('',*,*,").unwrap();
    let flag = at + text[at..].find(".T.").unwrap();
    texts.push((
        "a turned edge".into(),
        format!("{}.F.{}", &text[..flag], &text[flag + 3..]),
    ));
    let mut refused = 0;
    for (name, text) in texts {
        let mut whole_model = Model::default();
        let (whole, n) = steps_of(|c| step::read(&mut whole_model, &text, &options, c));
        let whole = whole.unwrap();
        refused += whole.solids.iter().filter(|s| s.result.is_err()).count();
        let expected = summary(&whole_model, &whole);
        assert!(n >= 1, "{name}: {n} steps");

        let start = Model::default();
        let before = native::to_bytes(&start).unwrap();
        for k in below(n) {
            let mut m = start.clone();
            let stop =
                stopped(step::read(&mut m, &text, &options, &Control::budget(k)).unwrap_err());
            assert_eq!(
                (stop.by, stop.steps),
                (Stop::Budget, k),
                "{name}: {k} of {n}"
            );
            assert_eq!(native::to_bytes(&m).unwrap(), before, "{name}: {k} of {n}");
        }
        let mut m = Model::default();
        let again = step::read(&mut m, &text, &options, &Control::budget(n))
            .unwrap_or_else(|e| panic!("{name}: a budget of {n} steps, all it takes: {e}"));
        assert_eq!(summary(&m, &again), expected, "{name}: a budget of {n}");
    }
    assert!(
        refused > 0,
        "no refusal among the files: the table is untested"
    );
}

#[test]
fn a_poll_stops_step_read_and_rolls_the_model_back() {
    let path = fixtures::corpus_root().join("real/nist-ctc-01");
    let file = std::fs::read_dir(&path)
        .unwrap()
        .flatten()
        .map(|e| e.path())
        .find(|p| p.extension().is_some_and(|x| x == "stp"))
        .expect("a STEP file");
    let text = std::fs::read_to_string(file).unwrap();
    let options = ReadOptions::default();
    let (whole, n) = steps_of(|c| step::read(&mut Model::default(), &text, &options, c));
    whole.unwrap();
    let before = native::to_bytes(&Model::default()).unwrap();
    for k in below(n) {
        let mut m = Model::default();
        let asked = AtomicU64::new(0);
        let poll = || asked.fetch_add(1, Ordering::Relaxed) >= k;
        let stop = stopped(step::read(&mut m, &text, &options, &Control::poll(&poll)).unwrap_err());
        assert_eq!((stop.by, stop.steps), (Stop::Poll, k), "poll at {k} of {n}");
        assert_eq!(native::to_bytes(&m).unwrap(), before, "poll at {k} of {n}");
    }
}

#[test]
fn a_budget_stops_the_body_readers() {
    let mut a = Model::default();
    let body_of = sample::cylinder(&mut a, 4.0, 12.0).unwrap();
    let bytes = body::write(&a, body_of, &Provenance::default()).unwrap();
    let text = body::to_json(&a, body_of, &Provenance::default()).unwrap();
    let base = {
        let mut m = Model::default();
        sample::unit_box(&mut m).unwrap();
        m
    };
    let before = native::to_bytes(&base).unwrap();

    let (whole, n) = steps_of(|c| body::read(&mut base.clone(), &bytes, c));
    let whole = whole.unwrap();
    let (whole_json, n_json) = steps_of(|c| body::from_json(&mut base.clone(), &text, c));
    assert_eq!(n, n_json, "one pipeline, two encodings");
    assert!(n >= 4, "{n} steps");
    let unbudgeted = whole.map.clone();
    let _ = whole_json.unwrap();
    for k in below(n) {
        for json in [false, true] {
            let mut m = base.clone();
            let control = Control::budget(k);
            let stopped = if json {
                body::from_json(&mut m, &text, &control)
            } else {
                body::read(&mut m, &bytes, &control)
            };
            match stopped {
                Err(body::BodyError::Interrupted(stop)) => {
                    assert_eq!((stop.by, stop.steps), (Stop::Budget, k), "{k} of {n}");
                }
                other => panic!("budget {k} of {n}: {other:?}"),
            }
            assert_eq!(native::to_bytes(&m).unwrap(), before, "{k} of {n}");
        }
    }
    let mut m = base.clone();
    let again = body::read(&mut m, &bytes, &Control::budget(n)).unwrap();
    assert_eq!(again.map, unbudgeted, "a budget of {n} changed the import");
}

/// `write_products` ticks per occurrence and per body: a budget below the
/// steps it takes stops the call with `Interrupted` at exactly the budget,
/// a budget at the count writes the same text, and a poll that says stop
/// stops it too. The model is read only, so there is nothing to roll back.
#[test]
fn a_budget_stops_write_products_and_a_read_of_its_tree_ticks_per_occurrence() {
    use arris_io::step::{Occurrence, ProductTree, StepError};
    use arris_math::Isometry;

    let mut m = Model::default();
    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let part = |x: f64| Occurrence {
        product: Some(1),
        name: "pin".into(),
        placement: Ok(Isometry::from_translation(arris_math::Vec3::new(
            x, 0.0, 0.0,
        ))),
        colour: None,
        solids: vec![0],
        children: vec![],
    };
    let tree = ProductTree {
        roots: vec![Occurrence {
            product: Some(2),
            name: "assembly".into(),
            placement: Ok(Isometry::identity()),
            colour: None,
            solids: vec![],
            children: vec![part(10.0), part(-10.0)],
        }],
        faces: vec![],
    };
    let (whole, n) = steps_of(|c| step::write_products(&m, &[cylinder], &tree, c));
    let text = whole.unwrap();
    // The root, its two placements of one product, and the body once.
    assert_eq!(n, 4, "{n} steps");
    for k in below(n) {
        match step::write_products(&m, &[cylinder], &tree, &Control::budget(k)) {
            Err(StepError::Interrupted(stop)) => {
                assert_eq!((stop.by, stop.steps), (Stop::Budget, k), "{k} of {n}");
            }
            other => panic!("budget {k} of {n}: {other:?}"),
        }
    }
    assert_eq!(
        step::write_products(&m, &[cylinder], &tree, &Control::budget(n)).unwrap(),
        text
    );
    let stop = step::write_products(&m, &[cylinder], &tree, &Control::poll(&|| true));
    assert!(matches!(stop, Err(StepError::Interrupted(s)) if s.by == Stop::Poll));

    // The tree walk of a read is one step per occurrence, beside the
    // solids' and the file's own.
    let (read, steps) =
        steps_of(|c| step::read(&mut Model::default(), &text, &ReadOptions::default(), c));
    let read = read.unwrap();
    assert_eq!(read.products.occurrences().count(), 3);
    assert!(steps >= 3 + read.solids.len() as u64, "{steps} steps");
}
