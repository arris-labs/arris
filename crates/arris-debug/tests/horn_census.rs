//! The run-over census after the horn and spindle band (ADR-0080,
//! `docs/plans/horn-torus`): the four fetched parts whose `BlendTooLarge`
//! edges ADR-0038 counted (277 edges, 98 of them the ring's contact at the
//! axis) are blended edge by edge at the battery's radius again, and what
//! still refuses is held by count and by cause. No oracle is asked, so the
//! test needs no Python; the oracle's verdicts on these edges are the
//! roadmap's record (`real_parts --run-over`). The files are the fetched
//! tier's (`tools/real-parts.sh`, gitignored): a clone without them skips.

use std::collections::BTreeMap;
use std::path::PathBuf;

use arris_debug::census::{BUILT, PANICKED, SolidCensus, census_file};
use arris_debug::oracle::workspace_root;

const DIR: &str = "target/real-parts/files/pmi/NIST-PMI-STEP-Files";

/// A part, its file, and its `BlendTooLarge` edges now by cause (the
/// census's labels), summed over its solids.
/// A cause label and its edge count.
type Causes = &'static [(&'static str, usize)];

const PARTS: [(&str, &str, Causes); 4] = [
    (
        "CTC-01",
        "nist_ctc_01_asme1_ap242-e1.stp",
        &[
            (
                "a corner edge shorter than the trim; edge cylinder × cylinder",
                4,
            ),
            (
                "a corner edge shorter than the trim; edge cylinder × plane",
                4,
            ),
            ("a corner edge shorter than the trim; edge plane × plane", 4),
        ],
    ),
    ("CTC-03", "nist_ctc_03_asme1_ap242-e2.stp", &[]),
    (
        "FTC-08",
        "nist_ftc_08_asme1_ap242-e2.stp",
        &[
            (
                "the contact leaves a cylinder face; edge cylinder × plane",
                4,
            ),
            ("the contact leaves a cylinder face; edge plane × plane", 4),
            ("the contact leaves a plane face; edge cylinder × plane", 9),
            ("the contact leaves a plane face; edge plane × plane", 9),
        ],
    ),
    (
        "STC-06",
        "nist_stc_06_asme1_ap242-e3.stp",
        &[
            ("a corner edge shorter than the trim; edge plane × plane", 1),
            ("the contact leaves a plane face; edge cylinder × plane", 8),
            ("the contact leaves a plane face; edge plane × plane", 7),
        ],
    ),
];

fn by_cause(solids: &[SolidCensus]) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    for s in solids {
        for e in &s.too_large {
            *out.entry(e.cause.clone()).or_default() += 1;
        }
    }
    out
}

#[test]
fn the_horn_and_spindle_edges_of_the_fetched_parts_build_or_are_refused_by_name() {
    let root: PathBuf = workspace_root().join(DIR);
    let mut total = 0;
    let mut built = 0;
    for (part, file, want) in PARTS {
        let path = root.join(file);
        if !path.exists() {
            eprintln!("{part}: {} is not fetched; skipped", path.display());
            continue;
        }
        let solids = census_file(&path, &|_| true).unwrap();
        let got = by_cause(&solids);
        let want: BTreeMap<String, usize> = want.iter().map(|(k, n)| (k.to_string(), *n)).collect();
        let b: usize = solids.iter().filter_map(|s| s.alone.get(BUILT)).sum();
        let refused: usize = got.values().sum();
        eprintln!("{part}: {b} edges build alone, {refused} BlendTooLarge: {got:?}");
        for s in &solids {
            assert!(
                !s.alone.contains_key(PANICKED),
                "{part} {}: an edge panics",
                s.solid
            );
        }
        assert_eq!(got, want, "{part}");
        total += refused;
        built += b;
    }
    eprintln!("census: {built} edges build alone, {total} BlendTooLarge");
}
