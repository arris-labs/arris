//! Prints the script-corpus refusal histogram (ADR-0067, plans/script-corpus step 7).
//!
//! Reads every `crates/arris-py/corpus/*/outcome.json` and its `oracle.json`,
//! maps every recorded refusal to its cycle, and writes `histogram.md` into
//! `<out-dir>` (or prints to stdout).

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use arris_debug::histogram::{Histogram, ScriptOutcome};

type Error = Box<dyn std::error::Error>;

fn script_corpus_dir() -> PathBuf {
    if let Ok(dir) = std::env::var("ARRIS_SCRIPT_CORPUS") {
        return PathBuf::from(dir);
    }
    let local = Path::new("crates/arris-py/corpus");
    if local.is_dir() {
        return local.to_path_buf();
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../crates/arris-py/corpus")
}

fn main() -> Result<(), Error> {
    let out_dir = std::env::args().nth(1).map(PathBuf::from);
    let corpus_dir = script_corpus_dir();

    let mut histogram = Histogram::new();
    let mut parts: Vec<String> = Vec::new();
    for entry in std::fs::read_dir(&corpus_dir)? {
        let entry = entry?;
        if entry.file_type()?.is_dir() {
            let slug = entry.file_name().to_string_lossy().to_string();
            if entry.path().join("outcome.json").is_file() {
                parts.push(slug);
            }
        }
    }
    parts.sort();

    for slug in parts {
        let dir = corpus_dir.join(&slug);
        let oracle_text = std::fs::read_to_string(dir.join("oracle.json"))?;
        let oracle: serde_json::Value = serde_json::from_str(&oracle_text)?;
        let mut ops: BTreeMap<String, String> = BTreeMap::new();
        if let Some(steps) = oracle.get("steps").and_then(|s| s.as_array()) {
            for step in steps {
                if let (Some(name), Some(op)) = (
                    step.get("name").and_then(|n| n.as_str()),
                    step.get("op").and_then(|o| o.as_str()),
                ) {
                    ops.insert(name.to_string(), op.to_string());
                }
            }
        }

        let outcome_text = std::fs::read_to_string(dir.join("outcome.json"))?;
        let outcomes: BTreeMap<String, ScriptOutcome> = serde_json::from_str(&outcome_text)?;

        for (stage, outcome) in &outcomes {
            let stage_op = ops.get(stage).map(String::as_str).unwrap_or(stage.as_str());
            histogram.add_script(&slug, stage_op, outcome);
        }
    }

    let markdown = histogram.script_markdown();
    if let Some(out) = out_dir {
        std::fs::create_dir_all(&out)?;
        std::fs::write(out.join("histogram.md"), &markdown)?;
    }
    print!("{markdown}");
    Ok(())
}
