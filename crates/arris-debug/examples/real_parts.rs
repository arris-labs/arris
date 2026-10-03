//! The refusal histogram over the real-part corpus (ADR-0026 §5), printed
//! as markdown (`arris_debug::histogram`).
//!
//! - `real_parts --committed` counts the committed tier from its
//!   fixtures: each refusal under the cycle the fixture records for it,
//!   which the part runner holds to the table on every `cargo test`.
//! - `real_parts --part <file.stp> <work-dir> <report.json> [source]`
//!   surveys one file of the fetched tier (`arris_debug::survey`) and
//!   writes its report. `tools/real-parts.sh` runs one process per file,
//!   under a timeout.
//! - `real_parts --census-committed` prints the committed tier's fillet
//!   column by part (`arris_debug::census`): each solid whose fillet stage
//!   Arris refuses, its first refusal at the battery's radius and what
//!   each blendable edge alone meets. `real_parts --census <file.stp>...`
//!   counts every solid of a file, no oracle asked. The fetched tier's is
//!   `fillet-by-part.md`, written by `--summary` from the survey.
//! - `real_parts --run-over <file.stp>...` and `--run-over-committed` take
//!   every `BlendTooLarge` edge of those solids apart by the site that
//!   refused it and put a sample of each cause to Open CASCADE alone at
//!   the same radius (`census::run_over_markdown`; a part's census is the
//!   minutes' work, the oracle's answers are cached).
//! - `real_parts --vertex-blend <file.stp>...` and `--vertex-blend-committed`
//!   do the same for `VertexBlend` (`census::vertex_blend_markdown`): each
//!   edge alone by the site that refused it, and the battery's sample
//!   together, each put to Open CASCADE. `tools/real-parts.sh` leaves the
//!   table at `target/real-parts/vertex-blend.md` beside the others.
//! - `real_parts --summary <manifest> <reports-dir> <waits> <out-dir>`
//!   reads every report the manifest's files should have, writes
//!   `histogram.md` and `failures.md` into `<out-dir>`, and `both.md`,
//!   the histogram over the committed tier and these together, and exits
//!   1 if a
//!   failure is not excluded by a `waits` line naming a fixture still
//!   under `regression/`.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use arris_debug::census;
use arris_debug::histogram::{COMMITTED_TIER, Histogram};
use arris_debug::survey::{self, Report};
use arris_debug::{fixtures, part};

type Error = Box<dyn std::error::Error>;

fn usage() -> ! {
    eprintln!(
        "usage: real_parts --committed\n       real_parts --part <file.stp> <work-dir> <report.json> [source]\n       real_parts --census <file.stp>...\n       real_parts --census-committed\n       real_parts --run-over <file.stp>...\n       real_parts --run-over-committed\n       real_parts --vertex-blend <file.stp>...\n       real_parts --vertex-blend-committed\n       real_parts --summary <manifest> <reports-dir> <waits> <out-dir>"
    );
    std::process::exit(2);
}

fn main() -> Result<(), Error> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("--committed") => committed(),
        Some("--part") if args.len() >= 4 => {
            let file = Path::new(&args[1]);
            let name = file
                .file_stem()
                .and_then(|s| s.to_str())
                .ok_or("no file stem")?;
            let source = args.get(4).map_or("", String::as_str);
            let report = survey::survey(file, name, Path::new(&args[2]), source)?;
            std::fs::write(&args[3], serde_json::to_string_pretty(&report)? + "\n")?;
            println!(
                "{name}: {:.1} s, {} read in {} occurrences, {} refusals, {} failures",
                report.read_seconds,
                report.read,
                report.occurrences,
                report.refusals.len(),
                report.failures.len()
            );
            Ok(())
        }
        Some("--summary") if args.len() == 5 => summary(
            Path::new(&args[1]),
            Path::new(&args[2]),
            Path::new(&args[3]),
            Path::new(&args[4]),
        ),
        Some("--census") if args.len() >= 2 => {
            let mut parts = Vec::new();
            for file in &args[1..] {
                let path = Path::new(file);
                let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or(file);
                parts.push((name.to_string(), census::census_file(path, &|_| true)?));
            }
            print!("{}", census::markdown(&parts));
            Ok(())
        }
        Some("--run-over") if args.len() >= 2 => {
            let mut parts = Vec::new();
            for file in &args[1..] {
                let path = Path::new(file);
                let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or(file);
                parts.push((name.to_string(), census::run_over_file(path, &|_| true)?));
            }
            print!("{}", census::run_over_markdown(&parts));
            Ok(())
        }
        Some("--vertex-blend") if args.len() >= 2 => {
            let mut parts = Vec::new();
            for file in &args[1..] {
                let path = Path::new(file);
                let name = path.file_stem().and_then(|s| s.to_str()).unwrap_or(file);
                parts.push((
                    name.to_string(),
                    census::vertex_blend_file(path, &|_| true)?,
                ));
            }
            print!("{}", census::vertex_blend_markdown(&parts));
            Ok(())
        }
        Some("--vertex-blend-committed") => {
            print!(
                "{}",
                census::vertex_blend_markdown(&census::vertex_blend_committed()?)
            );
            Ok(())
        }
        Some("--run-over-committed") => {
            print!(
                "{}",
                census::run_over_markdown(&census::run_over_committed()?)
            );
            Ok(())
        }
        Some("--census-committed") => {
            print!("{}", census::markdown(&census::committed()?));
            Ok(())
        }
        _ => usage(),
    }
}

fn committed() -> Result<(), Error> {
    print!("{}", committed_tier()?.markdown());
    Ok(())
}

/// The committed tier, counted from its fixtures.
fn committed_tier() -> Result<Histogram, Error> {
    let mut histogram = Histogram::new();
    for name in COMMITTED_TIER {
        let fixture = part::load(&fixtures::corpus_root().join(name))?;
        histogram.add_part(&fixture)?;
    }
    Ok(histogram)
}

/// The part names a manifest lists: each `<sha256>  <path>` line's file
/// stem, in the manifest's order.
fn manifest_parts(manifest: &Path) -> Result<Vec<String>, Error> {
    let text = std::fs::read_to_string(manifest)?;
    let mut parts = Vec::new();
    for line in text
        .lines()
        .filter(|l| !l.trim().is_empty() && !l.starts_with('#'))
    {
        let path = line
            .split_once("  ")
            .ok_or("a manifest line is `<sha256>  <path>`")?
            .1;
        let stem = Path::new(path)
            .file_stem()
            .and_then(|s| s.to_str())
            .ok_or("no file stem")?;
        parts.push(stem.to_string());
    }
    Ok(parts)
}

/// The waits: `<part> <regression/slug>` per line, `#` comments.
fn waits(path: &Path) -> Result<BTreeMap<String, Vec<String>>, Error> {
    let text = std::fs::read_to_string(path)?;
    let mut out: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let (part, slug) = line
            .split_once(char::is_whitespace)
            .ok_or("a waits line is `<part> <regression/slug>`")?;
        out.entry(part.to_string())
            .or_default()
            .push(slug.trim().to_string());
    }
    Ok(out)
}

fn summary(manifest: &Path, reports: &Path, waits_file: &Path, out: &Path) -> Result<(), Error> {
    let parts = manifest_parts(manifest)?;
    let waits = waits(waits_file)?;
    let root = fixtures::corpus_root();
    let waiting = |slug: &str| {
        slug.starts_with(&format!("{}/", fixtures::REGRESSION_AREA))
            && root.join(slug).join("fixture.json").is_file()
    };
    let mut histogram = Histogram::new();
    let mut both = committed_tier()?;
    let mut failures = String::from("# Failures\n\n");
    let mut open = 0;
    let mut fillet_column: Vec<(String, Vec<arris_debug::census::SolidCensus>)> = Vec::new();
    for name in &parts {
        let path: PathBuf = reports.join(format!("{name}.json"));
        let (report, mut found) = match std::fs::read_to_string(&path) {
            Ok(text) => {
                let report: Report = serde_json::from_str(&text)?;
                let found = report.failures.clone();
                (Some(report), found)
            }
            Err(_) => (
                None,
                vec!["no report: the survey did not finish within the script's timeout".into()],
            ),
        };
        if let Some(report) = &report {
            histogram.add_report(report);
            both.add_report(report);
            if !report.fillet_census.is_empty() {
                fillet_column.push((name.clone(), report.fillet_census.clone()));
            }
        }
        let excluded_by = waits.get(name).cloned().unwrap_or_default();
        for slug in &excluded_by {
            if !waiting(slug) {
                found.push(format!(
                    "waits on {slug}, which is not a fixture under regression/ any more: lift the wait"
                ));
            }
        }
        if found.is_empty() {
            if !excluded_by.is_empty() {
                open += 1;
                failures.push_str(&format!(
                    "## {name}\n\n- passes, yet waits on {}: lift the wait\n\n",
                    excluded_by.join(", ")
                ));
            }
            continue;
        }
        let excluded = !excluded_by.is_empty() && excluded_by.iter().all(|s| waiting(s));
        if !excluded {
            open += 1;
        }
        failures.push_str(&format!(
            "## {name}{}\n\n",
            if excluded {
                format!(" (excluded: waits on {})", excluded_by.join(", "))
            } else {
                String::new()
            }
        ));
        for f in found {
            failures.push_str(&format!("- {}\n", f.replace('\n', " / ")));
        }
        failures.push('\n');
    }
    std::fs::create_dir_all(out)?;
    std::fs::write(out.join("histogram.md"), histogram.markdown())?;
    std::fs::write(out.join("both.md"), both.markdown())?;
    std::fs::write(out.join("failures.md"), &failures)?;
    std::fs::write(
        out.join("fillet-by-part.md"),
        census::markdown(&fillet_column),
    )?;
    print!("{}", histogram.markdown());
    println!(
        "\n{open} failing part(s) not excluded; {}",
        out.join("failures.md").display()
    );
    if open > 0 {
        std::process::exit(1);
    }
    Ok(())
}
