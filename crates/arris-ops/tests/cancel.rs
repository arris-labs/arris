//! Cancellation of the booleans (ADR-0030): on every `boolean/` fixture
//! the steps an unbudgeted run takes are the same with `parallel` on and
//! off (one record, `tests/cancel_counts.txt`, both builds are held to),
//! a budget below that count stops with `Interrupted` at exactly the
//! budget and leaves the model's bytes as they were, and a budget at the
//! count gives the unbudgeted body.

use std::sync::atomic::{AtomicU64, Ordering};

use arris_debug::fixtures::{self, Step};
use arris_debug::{corpus, dump};
use arris_io::native;
use arris_ops::arris_check::arris_topo::arris_math::{Control, Interrupted, Stop};
use arris_ops::arris_check::arris_topo::{Body, Model};
use arris_ops::{OpError, boolean, common, cut, fuse};

#[derive(Clone, Copy)]
enum Kind {
    Fuse,
    Common,
    Cut,
}

fn run(
    m: &mut Model,
    kind: Kind,
    (a, b): (Body, Body),
    control: &Control<'_>,
) -> Result<Body, OpError> {
    match kind {
        Kind::Fuse => fuse(m, a, b, control),
        Kind::Common => common(m, a, b, control),
        Kind::Cut => cut(m, a, b, control),
    }
    .map(|(body, _)| body)
}

fn stopped(e: OpError) -> Interrupted {
    match e {
        OpError::Interrupted(stop) => stop,
        other => panic!("expected Interrupted, got {other}"),
    }
}

/// The steps `f` takes under a poll that never stops it: the number of
/// times it was asked.
fn steps_of<T>(f: impl FnOnce(&Control<'_>) -> T) -> (T, u64) {
    let asked = AtomicU64::new(0);
    let poll = || {
        asked.fetch_add(1, Ordering::Relaxed);
        false
    };
    let out = f(&Control::poll(&poll));
    (out, asked.load(Ordering::Relaxed))
}

/// The budgets to try below `n`: the ends, the middle and a spread.
fn below(n: u64) -> Vec<u64> {
    let mut ks: Vec<u64> = [0, 1, n / 4, n / 2, 3 * n / 4, n.saturating_sub(1)]
        .into_iter()
        .filter(|&k| k < n)
        .collect();
    ks.sort_unstable();
    ks.dedup();
    ks
}

fn record_path() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/cancel_counts.txt")
}

#[test]
fn a_budget_stops_the_boolean_at_the_same_step_and_leaves_the_model_as_it_was() {
    let root = fixtures::corpus_root().join("boolean");
    let mut counts = String::new();
    let mut ran = 0;
    let mut dirs: Vec<_> = std::fs::read_dir(&root)
        .expect("the boolean fixtures")
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.join("fixture.json").is_file())
        .collect();
    dirs.sort();
    for dir in dirs {
        let name = dir.file_name().unwrap().to_string_lossy().into_owned();
        let Ok(inputs) = corpus::inputs(&dir, "default") else {
            continue;
        };
        let Some(operands) = inputs.operands() else {
            continue;
        };
        let kind = match inputs.result {
            Step::Fuse { .. } => Kind::Fuse,
            Step::Common { .. } => Kind::Common,
            Step::Cut { .. } => Kind::Cut,
            _ => continue,
        };
        let mut m = inputs.model.clone();
        let (whole, n) = steps_of(|c| run(&mut m, kind, operands, c));
        // A refusal by design is a fixture of its own; an interrupt of it
        // is the same claim, but its count is not a body's.
        let Ok(whole) = whole else { continue };
        let expected = dump::dump_text(&m, whole).unwrap();
        counts.push_str(&format!("{name}: {n}\n"));
        ran += 1;

        let before = native::to_bytes(&inputs.model).unwrap();
        for k in below(n) {
            let mut m = inputs.model.clone();
            let stop = stopped(run(&mut m, kind, operands, &Control::budget(k)).unwrap_err());
            assert_eq!(
                (stop.by, stop.steps),
                (Stop::Budget, k),
                "{name}: budget {k} of {n}"
            );
            assert_eq!(
                native::to_bytes(&m).unwrap(),
                before,
                "{name}: budget {k} of {n} left the model changed"
            );
        }
        let mut m = inputs.model.clone();
        let body = run(&mut m, kind, operands, &Control::budget(n))
            .unwrap_or_else(|e| panic!("{name}: a budget of {n} steps, all it takes: {e}"));
        assert_eq!(
            dump::dump_text(&m, body).unwrap(),
            expected,
            "{name}: a budget of {n} changed the result"
        );
    }
    assert!(ran > 50, "only {ran} boolean fixtures ran");
    if corpus::blessing() {
        std::fs::write(record_path(), &counts).expect("the record is written");
        return;
    }
    let record = std::fs::read_to_string(record_path()).expect("the record is committed");
    assert_eq!(
        counts,
        record,
        "the boolean step counts moved (the same with `parallel` on and off); bless with {}=1",
        corpus::BLESS_VAR
    );
}

#[test]
fn interferences_stops_at_its_budget() {
    let root = fixtures::corpus_root().join("boolean");
    for name in ["boss", "through-hole", "coaxial-cut"] {
        let inputs = corpus::inputs(&root.join(name), "default").unwrap();
        let (a, b) = inputs.operands().unwrap();
        let m = &inputs.model;
        let (whole, n) = steps_of(|c| boolean::interferences(m, a, b, c));
        let whole = whole.unwrap();
        assert!(n >= 2, "{name}: {n} steps");
        let stop = stopped(boolean::interferences(m, a, b, &Control::budget(n / 2)).unwrap_err());
        assert_eq!((stop.by, stop.steps), (Stop::Budget, n / 2), "{name}");
        let again = boolean::interferences(m, a, b, &Control::budget(n)).unwrap();
        assert_eq!(
            again, whole,
            "{name}: a budget of {n} changed the pave model"
        );
    }
}

#[test]
fn a_poll_stops_the_boolean_and_rolls_the_model_back() {
    let root = fixtures::corpus_root().join("boolean");
    let inputs = corpus::inputs(&root.join("boss"), "default").unwrap();
    let operands = inputs.operands().unwrap();
    let Step::Fuse { .. } = inputs.result else {
        // `boss` is a fuse; anything else here is a fixture edit to read.
        panic!("boss is not a fuse any more");
    };
    let before = native::to_bytes(&inputs.model).unwrap();
    let mut m = inputs.model.clone();
    let (whole, n) = steps_of(|c| run(&mut m, Kind::Fuse, operands, c));
    whole.unwrap();
    for k in below(n) {
        let mut m = inputs.model.clone();
        let asked = AtomicU64::new(0);
        let poll = || asked.fetch_add(1, Ordering::Relaxed) >= k;
        let stop = stopped(run(&mut m, Kind::Fuse, operands, &Control::poll(&poll)).unwrap_err());
        assert_eq!(stop.by, Stop::Poll);
        // Where a poll lands is the schedule's under `parallel`; the
        // sequential build asks one question per step.
        if !cfg!(feature = "parallel") {
            assert_eq!(stop.steps, k, "poll at question {}", k + 1);
        }
        assert_eq!(native::to_bytes(&m).unwrap(), before, "poll at {k} of {n}");
    }
}
