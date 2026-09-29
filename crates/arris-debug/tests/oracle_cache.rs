//! The oracle cache from the outside (ADR-0024 §1): a second corpus run
//! of a fixture starts no Python, and a mismatch is asked of the oracle
//! every time. Each test points the cache at a fresh directory of its own,
//! whatever `ARRIS_ORACLE_CACHE` says, so the first call always runs the
//! oracle and the count of `uv` starts is exact.

use arris_debug::unmetered::primitive_box;
use std::sync::Mutex;

use arris_debug::corpus;
use arris_debug::oracle::{self, OracleError, cache};
use arris_io::arris_check::arris_topo::Model;
use arris_io::step;

/// The cache setting and the spawn count are the process's, so the tests
/// here take turns under `cargo test`'s threads.
static TURN: Mutex<()> = Mutex::new(());

fn fresh_cache(name: &str) {
    let dir = oracle::scratch_dir().join(name);
    let _ = std::fs::remove_dir_all(&dir);
    cache::set(Some(cache::Setting::At(dir)));
}

#[test]
fn a_second_corpus_run_starts_no_python() {
    let _turn = TURN.lock().unwrap_or_else(|e| e.into_inner());
    fresh_cache("oracle-cache-corpus");
    let dir = oracle::workspace_root().join("tests/fixtures/primitive/box");
    let before = oracle::spawns();
    corpus::run(&dir, "default").unwrap();
    let first = oracle::spawns();
    assert!(first > before, "a fresh cache ran no oracle");
    corpus::run(&dir, "default").unwrap();
    assert_eq!(oracle::spawns(), first, "the second run started `uv`");
    cache::set(None);
}

#[test]
fn a_mismatch_is_asked_again() {
    let _turn = TURN.lock().unwrap_or_else(|e| e.into_inner());
    fresh_cache("oracle-cache-mismatch");
    let mut m = Model::default();
    let (body, _) = primitive_box(&mut m, [0.0, 0.0, 0.0], [1.0, 2.0, 3.0]).unwrap();
    let text = step::write(&m, &[body]).unwrap();
    for round in 1..=2 {
        let before = oracle::spawns();
        let err = oracle::compare("primitive/box", &text, None, "small-box-as-box").unwrap_err();
        assert!(matches!(err, OracleError::Mismatch { .. }), "{err}");
        assert_eq!(oracle::spawns(), before + 1, "round {round}");
    }
    cache::set(None);
}

#[test]
fn off_runs_the_oracle_every_time() {
    let _turn = TURN.lock().unwrap_or_else(|e| e.into_inner());
    cache::set(Some(cache::Setting::Off));
    let dir = oracle::workspace_root().join("tests/fixtures/primitive/box");
    let before = oracle::spawns();
    corpus::run(&dir, "default").unwrap();
    corpus::run(&dir, "default").unwrap();
    // Each run asks `compare.py` of Arris's STEP and `occt_step.py` for
    // Open CASCADE's, plain and converted to B-splines.
    assert_eq!(oracle::spawns(), before + 6);
    cache::set(None);
}
