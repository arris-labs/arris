//! `step::read` on any text: no panic, the same answer twice — the same
//! solids, results and ids in two fresh models — and, where the text
//! does not parse, an error that names a place inside it. A solid the
//! reader returns passes the checker at `Level::Fast` by its own
//! guarantee (ADR-0025 §5); refusals are answers, not findings.
#![no_main]

use arris_topo::Model;
use arris_math::Control;
use arris_io::step::{self, ReadError, ReadOptions};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    // Part 21 is ASCII; a byte that is not UTF-8 becomes U+FFFD, which
    // the parser must refuse as a token like any other stray character.
    let text = String::from_utf8_lossy(data);
    let read = |text: &str| {
        let mut model = Model::default();
        step::read(&mut model, text, &ReadOptions::default(), &Control::NONE)
    };
    let first = read(&text);
    let second = read(&text);
    assert_eq!(first, second, "two reads of one text differ");
    if let Err(ReadError::Parse(e)) = &first {
        // Lines and columns count from 1; the last line may be the one
        // after a final newline, where an unexpected end is found.
        let lines = text.split('\n').count();
        assert!(e.line >= 1 && e.column >= 1, "{e}: not counted from 1");
        assert!(e.line as usize <= lines, "{e}: past the text's {lines} lines");
        if let Some(line) = text.split('\n').nth(e.line as usize - 1) {
            let width = line.chars().count() + 1;
            assert!(e.column as usize <= width, "{e}: past the line's {width} columns");
        }
    }
});
