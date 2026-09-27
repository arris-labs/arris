//! `body::read` on any bytes, and `body::from_json` on any that are
//! UTF-8, into a model that already holds a body: no panic; the same
//! answer twice; a body read passes the checker at `Level::Full`, as
//! `read` guarantees for bytes from outside the kernel (ADR-0029); and a
//! refusal leaves the model as it was, byte for byte. Refusals are
//! answers, not findings.
#![no_main]

use std::sync::OnceLock;

use arris_io::arris_check::arris_topo::Model;
use arris_io::arris_check::{Level, check};
use arris_io::body::{self, BodyError, Imported};
use arris_io::native;
use libfuzzer_sys::fuzz_target;

/// The model every input is read into: the guard's tetrahedron, so a
/// refusal has something to leave alone and a body read lands on ids
/// that are not dense.
fn target() -> &'static (Model, Vec<u8>) {
    static TARGET: OnceLock<(Model, Vec<u8>)> = OnceLock::new();
    TARGET.get_or_init(|| {
        let mut m = Model::default();
        body::read(
            &mut m,
            include_bytes!("../../crates/arris-io/tests/body/v1/tetrahedron.bin"),
        )
        .expect("the guard's tetrahedron reads");
        let bytes = native::to_bytes(&m).expect("a model the kernel read encodes");
        (m, bytes)
    })
}

/// One read into a copy of the target, holding it to `read`'s guarantees.
fn once(read: impl Fn(&mut Model) -> Result<Imported, BodyError>) -> Result<Imported, BodyError> {
    let (model, before) = target();
    let mut m = model.clone();
    let answer = read(&mut m);
    match &answer {
        Ok(imported) => {
            let report = check(&m, imported.body, Level::Full);
            assert!(report.is_ok(), "a body read the checker rejects:\n{report}");
        }
        Err(e) => {
            let after = native::to_bytes(&m).expect("the model encodes");
            assert!(&after == before, "{e}: the refusal changed the model");
        }
    }
    answer
}

/// Two answers are the same answer: compared as text, since a refusal's
/// report may hold a NaN (a volume the checker could not measure), which
/// is never equal to itself.
fn same(a: &Result<Imported, BodyError>, b: &Result<Imported, BodyError>) -> bool {
    format!("{a:?}") == format!("{b:?}")
}

fuzz_target!(|data: &[u8]| {
    let first = once(|m| body::read(m, data));
    let second = once(|m| body::read(m, data));
    assert!(same(&first, &second), "two reads of one input differ");
    if let Ok(text) = std::str::from_utf8(data) {
        let first = once(|m| body::from_json(m, text));
        let second = once(|m| body::from_json(m, text));
        assert!(same(&first, &second), "two reads of one text differ");
    }
});
