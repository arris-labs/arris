//! Cancellation of the operations (ADR-0030): on every fixture of the
//! areas that end in an operation the steps an unbudgeted run takes are
//! the same with `parallel` on and off (one record,
//! `tests/cancel_counts.txt`, both builds are held to), a budget below
//! that count stops with `Interrupted` at exactly the budget and leaves
//! the model's bytes as they were, and a budget at the count gives the
//! unbudgeted body.

use std::sync::atomic::{AtomicU64, Ordering};

use arris_debug::fixtures;
use arris_debug::{corpus, dump};
use arris_io::native;
use arris_ops::measure::mass_properties;
use arris_ops::{Control, Interrupted, OpError, Stop, boolean, primitive_box};

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

/// The areas whose fixtures end in an operation on a model, each
/// fixture's result step run under a budget.
const AREAS: [&str; 7] = [
    "boolean",
    "sweep",
    "blend",
    "build",
    "provenance",
    "transform",
    "primitive",
];

#[test]
fn a_budget_stops_every_operation_at_the_same_step_and_leaves_the_model_as_it_was() {
    let mut counts = String::new();
    let mut ran = 0;
    for area in AREAS {
        let root = fixtures::corpus_root().join(area);
        let mut dirs: Vec<_> = std::fs::read_dir(&root)
            .expect("the fixture area")
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.join("fixture.json").is_file())
            .collect();
        dirs.sort();
        let mut in_area = 0;
        for dir in dirs {
            let name = format!("{area}/{}", dir.file_name().unwrap().to_string_lossy());
            let Ok(inputs) = corpus::inputs(&dir, "default") else {
                continue;
            };
            let mut m = inputs.model.clone();
            let (whole, n) = steps_of(|c| inputs.run_result(&mut m, c));
            // A refusal by design is a fixture of its own.
            let Ok(whole) = whole else { continue };
            let expected = dump::dump_text(&m, whole.body).unwrap();
            counts.push_str(&format!("{name}: {n}\n"));
            in_area += 1;

            let before = native::to_bytes(&inputs.model).unwrap();
            for k in below(n) {
                let mut m = inputs.model.clone();
                let stop = match inputs.run_result(&mut m, &Control::budget(k)) {
                    Err(corpus::CorpusError::Op { source, .. }) => stopped(source),
                    Err(other) => panic!("{name}: budget {k} of {n}: {other}"),
                    Ok(_) => panic!("{name}: budget {k} of {n} ran to the end"),
                };
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
            let made = inputs
                .run_result(&mut m, &Control::budget(n))
                .unwrap_or_else(|e| panic!("{name}: a budget of {n} steps, all it takes: {e}"));
            assert_eq!(
                dump::dump_text(&m, made.body).unwrap(),
                expected,
                "{name}: a budget of {n} changed the result"
            );
        }
        assert!(in_area >= 2, "{area}: only {in_area} fixtures ran");
        ran += in_area;
    }
    assert!(ran > 100, "only {ran} fixtures ran");
    if corpus::blessing() {
        std::fs::write(record_path(), &counts).expect("the record is written");
        return;
    }
    let record = std::fs::read_to_string(record_path()).expect("the record is committed");
    assert_eq!(
        counts,
        record,
        "the step counts moved (they are the same with `parallel` on and off); bless with {}=1",
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
fn a_poll_stops_an_operation_and_rolls_the_model_back() {
    let root = fixtures::corpus_root().join("boolean");
    let inputs = corpus::inputs(&root.join("boss"), "default").unwrap();
    let before = native::to_bytes(&inputs.model).unwrap();
    let mut m = inputs.model.clone();
    let (whole, n) = steps_of(|c| inputs.run_result(&mut m, c));
    whole.unwrap();
    for k in below(n) {
        let mut m = inputs.model.clone();
        let asked = AtomicU64::new(0);
        let poll = || asked.fetch_add(1, Ordering::Relaxed) >= k;
        let Err(corpus::CorpusError::Op { source, .. }) =
            inputs.run_result(&mut m, &Control::poll(&poll))
        else {
            panic!("poll at {k} of {n}: not an interrupt");
        };
        let stop = stopped(source);
        assert_eq!(stop.by, Stop::Poll);
        // Where a poll lands is the schedule's under `parallel`; the
        // sequential build asks one question per step.
        if !cfg!(feature = "parallel") {
            assert_eq!(stop.steps, k, "poll at question {}", k + 1);
        }
        assert_eq!(native::to_bytes(&m).unwrap(), before, "poll at {k} of {n}");
    }
}

#[test]
fn mass_properties_stops_at_its_budget() {
    let mut m = arris_ops::arris_check::arris_topo::Model::default();
    let (body, _) = primitive_box(&mut m, [0.0; 3], [3.0, 2.0, 1.0], &Control::NONE).unwrap();
    let (whole, n) = steps_of(|c| mass_properties(&m, body, c));
    let whole = whole.unwrap();
    assert!(n >= 6, "{n} steps for the faces of a box");
    for k in below(n) {
        let stop = stopped(mass_properties(&m, body, &Control::budget(k)).unwrap_err());
        assert_eq!((stop.by, stop.steps), (Stop::Budget, k));
    }
    let again = mass_properties(&m, body, &Control::budget(n)).unwrap();
    assert_eq!(again.volume, whole.volume);
}
