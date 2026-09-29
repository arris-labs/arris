//! Cancellation of the tessellation (ADR-0030): on a curved body the
//! steps an unbudgeted run takes are the same with `parallel` on and off
//! (one record, `tests/cancel_counts.txt`, both builds are held to), a
//! budget below that count stops with `Interrupted` at exactly the budget,
//! and a budget at the count gives the unbudgeted mesh. The model is only
//! read, so "left as it was" is that its dump is the same after.

use std::sync::atomic::{AtomicU64, Ordering};

use arris_debug::{corpus, dump, sample};
use arris_mesh::{MeshError, MeshRequest, TriMesh, tessellate_with};
use arris_topo::arris_math::{Control, Interrupted, Point3, Stop};
use arris_topo::{Body, Model};

fn stopped(e: MeshError) -> Interrupted {
    match e {
        MeshError::Interrupted(stop) => stop,
        other => panic!("expected Interrupted, got {other}"),
    }
}

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

fn record_path() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/cancel_counts.txt")
}

type Case = (&'static str, fn(&mut Model) -> Body, f64, bool);

fn cylinder(m: &mut Model) -> Body {
    sample::cylinder(m, 4.0, 12.0).unwrap()
}
fn ball(m: &mut Model) -> Body {
    sample::sphere(m, Point3::origin(), 3.0).unwrap()
}
fn cube(m: &mut Model) -> Body {
    sample::unit_box(m).unwrap()
}
fn torus(m: &mut Model) -> Body {
    sample::torus(m, Point3::origin(), 5.0, 1.5).unwrap()
}

/// Bodies (a flat one, curved ones), each at a chord and with and
/// without the render buffer.
const CASES: [Case; 5] = [
    ("cube", cube, 1e-2, false),
    ("cylinder", cylinder, 1e-3, false),
    ("ball", ball, 1e-2, true),
    ("torus", torus, 1e-2, false),
    ("torus-corners", torus, 1e-1, true),
];

fn mesh_of(
    m: &Model,
    body: Body,
    request: &MeshRequest,
    c: &Control<'_>,
) -> Result<TriMesh, MeshError> {
    tessellate_with(m, body, request, c)
}

#[test]
fn a_budget_stops_tessellation_at_the_same_step_and_leaves_the_model_as_it_was() {
    let mut counts = String::new();
    for (name, make, chord, corners) in CASES {
        let mut m = Model::default();
        let body = make(&mut m);
        let request = if corners {
            MeshRequest::new(chord).with_corners()
        } else {
            MeshRequest::new(chord)
        };
        let before = dump::dump_text(&m, body).unwrap();
        let (whole, n) = steps_of(|c| mesh_of(&m, body, &request, c));
        let whole = whole.unwrap();
        counts.push_str(&format!("{name}: {n}\n"));
        assert!(n >= 6, "{name}: {n} steps");
        for k in below(n) {
            let stop = stopped(mesh_of(&m, body, &request, &Control::budget(k)).unwrap_err());
            assert_eq!(
                (stop.by, stop.steps),
                (Stop::Budget, k),
                "{name}: {k} of {n}"
            );
            assert_eq!(
                dump::dump_text(&m, body).unwrap(),
                before,
                "{name}: {k} of {n}"
            );
        }
        let again = mesh_of(&m, body, &request, &Control::budget(n))
            .unwrap_or_else(|e| panic!("{name}: a budget of {n} steps, all it takes: {e}"));
        assert_eq!(again, whole, "{name}: a budget of {n} changed the mesh");
    }
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
fn a_poll_stops_tessellation() {
    let mut m = Model::default();
    let body = ball(&mut m);
    let request = MeshRequest::new(1e-2);
    let (whole, n) = steps_of(|c| mesh_of(&m, body, &request, c));
    whole.unwrap();
    for k in below(n) {
        let asked = AtomicU64::new(0);
        let poll = || asked.fetch_add(1, Ordering::Relaxed) >= k;
        let stop = stopped(mesh_of(&m, body, &request, &Control::poll(&poll)).unwrap_err());
        assert_eq!(stop.by, Stop::Poll);
        // Where a poll lands is the schedule's under `parallel`.
        if !cfg!(feature = "parallel") {
            assert_eq!(stop.steps, k, "poll at question {}", k + 1);
        }
    }
}
