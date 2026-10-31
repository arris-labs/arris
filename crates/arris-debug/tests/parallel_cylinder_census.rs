//! The parallel-cylinder row (the census before step 3, the fillet, held its
//! refusals; step 7 recounts) (ADR-0036 §5,
//! plans/parallel-cylinder-blend): every edge of the fetched tier that two
//! cylinders with parallel axes share along a ruling is blended alone at
//! the battery's radius, and what happens to it is held by part, by
//! convexity, by what lies across at its ends and by outcome. No oracle is
//! asked, so the test needs no Python. The files are the fetched tier's
//! (`tools/real-parts.sh`, gitignored): a clone without them skips.

use std::collections::BTreeMap;
use std::path::PathBuf;

use arris_debug::census::{PANICKED, ParallelCylinderEdge, parallel_cylinder_census_file};
use arris_debug::oracle::workspace_root;

const DIR: &str = "target/real-parts/files/pmi/NIST-PMI-STEP-Files";

/// The ends across the stripe are tori, which no blend end meets yet.
const TORI: &str = "Unsupported pair: torus surface × torus surface";

/// A part's file, and the rows it holds after step 3's fillet: convex or concave, the ends,
/// the outcome, the edge count. A part not listed holds none. The count was
/// 29 edges in 5 parts at C6 (ADR-0036 §5), over every cylinder pair the
/// battery met; the ruling edges between parallel axes that remain are
/// these 6 in 3 parts (plans/parallel-cylinder-blend step 1).
type Row = (&'static str, &'static str, &'static str, usize);

const PARTS: [(&str, [Row; 1]); 3] = [
    (
        "nist_ftc_08_asme1_ap242-e2.stp",
        [("convex", "plane / plane", "built", 2)],
    ),
    (
        "nist_stc_08_asme1_ap242-e3.stp",
        [("convex", "plane / plane", "built", 2)],
    ),
    (
        "nist_stc_10_asme1_ap242-e2.stp",
        [("concave", "torus+torus / torus+torus", TORI, 2)],
    ),
];

fn rows(edges: &[ParallelCylinderEdge]) -> BTreeMap<(String, String, String), usize> {
    let mut out = BTreeMap::new();
    for e in edges {
        let shape = if e.convex { "convex" } else { "concave" };
        *out.entry((shape.to_string(), e.ends.clone(), e.outcome.clone()))
            .or_default() += 1;
    }
    out
}

#[test]
fn the_parallel_cylinder_edges_of_the_fetched_parts_are_counted_by_outcome() {
    let root: PathBuf = workspace_root().join(DIR);
    let mut files: Vec<_> = match std::fs::read_dir(&root) {
        Ok(d) => d
            .filter_map(Result::ok)
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "stp"))
            .collect(),
        Err(_) => {
            eprintln!("{} is not fetched; skipped", root.display());
            return;
        }
    };
    files.sort();
    let mut total = 0;
    for path in files {
        let part = path.file_name().unwrap().to_string_lossy().to_string();
        let edges = parallel_cylinder_census_file(&path, &|_| true).unwrap();
        assert!(
            edges.iter().all(|e| e.outcome != PANICKED),
            "{part}: an edge panics"
        );
        let got = rows(&edges);
        let want: BTreeMap<_, _> = PARTS
            .iter()
            .filter(|(f, _)| *f == part)
            .flat_map(|(_, r)| r.iter())
            .map(|(c, e, o, n)| ((c.to_string(), e.to_string(), o.to_string()), *n))
            .collect();
        assert_eq!(got, want, "{part}");
        total += edges.len();
    }
    eprintln!("census: {total} parallel-cylinder ruling edges");
}
