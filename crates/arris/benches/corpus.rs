//! The corpus benchmark (ADR-0024 §4): for every fixture under
//! `boolean/`, `sweep/` and `blend/` whose recipe builds, the recipe's
//! build and the result's tessellation at the fixture's `mesh_chord`,
//! timed apart, so a change that costs 10× in either shows as a ratio.
//! For every part under `real/` that does not wait, the STEP reader's
//! read of its file, and the checker at `Fast` alone on every solid the
//! read returns (ADR-0025 §5): the reader runs that check on each solid in
//! every build, so the read less the check is what the reader costs
//! without it (ADR-0025 §Consequences). A part whose file another
//! part already timed is skipped by name.
//!
//! ```sh
//! cargo bench -p arris --bench corpus                          # print the timings
//! cargo bench -p arris --bench corpus -- --save target/bench.json
//! cargo bench -p arris --bench corpus -- --compare target/bench.json
//! cargo bench -p arris --bench corpus -- --compare target/bench.json --table target/table.md
//! cargo bench -p arris --bench corpus -- --filter boolean/     # a subset
//! ```
//!
//! A relative path is taken from the workspace root. `--compare` prints
//! each case's ratio against the saved report and
//! flags those past `bench::RATIO_FLAG`; it never fails on a time.
//! `--table` also writes that comparison, as markdown, to a file — what
//! `tools/bench-compare.sh` hands the nightly's job summary. A
//! fixture that refuses by design (`expected.degenerate`) has nothing to
//! time and is skipped by name.

use std::path::PathBuf;
use std::process::ExitCode;

use arris::check::{Level, check};
use arris::io::step::{self, ReadOptions};
use arris::math::Control;
use arris::mesh::{MeshRequest, tessellate_with};
use arris::topo::Model;
use arris_debug::bench::{self, Config, Report, Wall};
use arris_debug::{corpus, fixtures, part};

/// The corpus areas whose fixtures build a solid from operations.
const AREAS: [&str; 3] = ["boolean/", "sweep/", "blend/"];

struct Args {
    save: Option<PathBuf>,
    compare: Option<PathBuf>,
    table: Option<PathBuf>,
    filter: Option<String>,
}

fn args() -> Result<Args, String> {
    let mut out = Args {
        save: None,
        compare: None,
        table: None,
        filter: None,
    };
    let mut it = std::env::args().skip(1);
    while let Some(a) = it.next() {
        let mut value = || it.next().ok_or(format!("{a} needs a value"));
        match a.as_str() {
            "--save" => out.save = Some(at_root(value()?)),
            "--compare" => out.compare = Some(at_root(value()?)),
            "--table" => out.table = Some(at_root(value()?)),
            "--filter" => out.filter = Some(value()?),
            // What `cargo bench` passes every bench target.
            "--bench" => {}
            other => return Err(format!("unknown argument {other:?}")),
        }
    }
    Ok(out)
}

/// `path` against the workspace root when relative: `cargo bench` runs
/// this binary in the crate's directory, and the examples above name
/// paths from the root.
fn at_root(path: String) -> PathBuf {
    let path = PathBuf::from(path);
    if path.is_absolute() {
        return path;
    }
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .join(path)
}

fn main() -> ExitCode {
    let args = match args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("corpus bench: {e}");
            return ExitCode::from(2);
        }
    };
    if args.table.is_some() && args.compare.is_none() {
        eprintln!("corpus bench: --table needs --compare");
        return ExitCode::from(2);
    }
    let config = Config::default();
    let mut clock = Wall::new();
    let mut report = Report::default();
    for dir in fixtures::corpus() {
        let name = fixtures::name_of(&dir);
        if !AREAS.iter().any(|a| name.starts_with(a)) {
            continue;
        }
        if args.filter.as_ref().is_some_and(|f| !name.contains(f)) {
            continue;
        }
        let fixture = match fixtures::load(&dir) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("corpus bench: {name}: {e}");
                return ExitCode::FAILURE;
            }
        };
        let chain = match corpus::chain(&dir, "default") {
            Ok(c) => c,
            Err(e) => {
                eprintln!("skipped {name}: {e}");
                continue;
            }
        };
        let Some(body) = chain.result() else {
            eprintln!("skipped {name}: no result body");
            continue;
        };
        let build = bench::time(&format!("{name} build"), config, &mut clock, || {
            std::hint::black_box(corpus::chain(&dir, "default").ok());
        });
        let request = MeshRequest::new(fixture.recipe.tolerances.mesh_chord);
        let mesh = bench::time(&format!("{name} mesh"), config, &mut clock, || {
            std::hint::black_box(
                tessellate_with(&chain.model, body, &request, &Control::NONE).ok(),
            );
        });
        for case in [build, mesh] {
            println!(
                "{:<64} {:>10.4} s ± {:.4}",
                case.name, case.median, case.mad
            );
            report.cases.push(case);
        }
    }
    // A pattern of holes in one call against the chain of cuts that
    // makes the same solid (ADR-0050 §Cost), timed apart.
    for dir in fixtures::corpus() {
        let name = fixtures::name_of(&dir);
        if !name.contains("-holes-cut-many")
            || args.filter.as_ref().is_some_and(|f| !name.contains(f))
        {
            continue;
        }
        let Ok(inputs) = corpus::inputs(&dir, "default") else {
            continue;
        };
        let Some((target, tools)) = inputs.cut_operands() else {
            continue;
        };
        // A refusal by design has nothing to time.
        if arris::ops::cut_many(&mut inputs.model.clone(), target, &tools, &Control::NONE).is_err()
        {
            eprintln!("skipped {name}: it refuses");
            continue;
        }
        let many = bench::time(&format!("{name} cut_many"), config, &mut clock, || {
            let mut m = inputs.model.clone();
            std::hint::black_box(arris::ops::cut_many(&mut m, target, &tools, &Control::NONE).ok());
        });
        let chained = bench::time(&format!("{name} chained cuts"), config, &mut clock, || {
            let mut m = inputs.model.clone();
            let mut body = target;
            for &tool in &tools {
                match arris::ops::cut(&mut m, body, tool, &Control::NONE) {
                    Ok((next, _)) => body = next,
                    Err(_) => break,
                }
            }
            std::hint::black_box(body);
        });
        println!(
            "{name}: cut_many {:.4} s, {} chained cuts {:.4} s, ratio {:.2}",
            many.median,
            tools.len(),
            chained.median,
            chained.median / many.median
        );
        report.cases.push(many);
        report.cases.push(chained);
    }
    let mut solids_read = 0;
    let mut timed: Vec<String> = Vec::new();
    for dir in fixtures::corpus() {
        let name = fixtures::name_of(&dir);
        if !name.starts_with("real/") {
            continue;
        }
        if args.filter.as_ref().is_some_and(|f| !name.contains(f)) {
            continue;
        }
        let fixture = match part::load(&dir) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("corpus bench: {name}: {e}");
                return ExitCode::FAILURE;
            }
        };
        if !fixture.part.waits_on.is_empty() {
            eprintln!("skipped {name}: it waits");
            continue;
        }
        // A part shrunk to its whole file is the same read twice.
        if timed.contains(&fixture.part.sha256) {
            eprintln!("skipped {name}: its file is timed already");
            continue;
        }
        timed.push(fixture.part.sha256.clone());
        let text = match std::fs::read(dir.join(&fixture.part.file)) {
            Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
            Err(e) => {
                eprintln!("corpus bench: {name}: {e}");
                return ExitCode::FAILURE;
            }
        };
        let precision = fixture.part.precision.precision();
        let read = || {
            let mut m = Model::new(precision).ok()?;
            let read = step::read(&mut m, &text, &ReadOptions::default(), &Control::NONE).ok()?;
            Some((m, read))
        };
        let Some((m, once)) = read() else {
            eprintln!("skipped {name}: the file does not read");
            continue;
        };
        let bodies: Vec<_> = (once.solids.iter())
            .filter_map(|s| s.result.as_ref().ok().map(|b| b.body))
            .collect();
        solids_read += bodies.len();
        let read_case = bench::time(&format!("{name} read"), config, &mut clock, || {
            std::hint::black_box(read());
        });
        let check_case = bench::time(&format!("{name} check"), config, &mut clock, || {
            for &body in &bodies {
                std::hint::black_box(check(&m, body, Level::Fast));
            }
        });
        for case in [read_case, check_case] {
            println!(
                "{:<64} {:>10.4} s ± {:.4}",
                case.name, case.median, case.mad
            );
            report.cases.push(case);
        }
    }
    let sum = |suffix: &str| -> f64 {
        (report.cases.iter())
            .filter(|c| c.name.ends_with(suffix))
            .map(|c| c.median)
            .sum::<f64>()
            + 0.0
    };
    println!(
        "\n{} cases: build {:.3} s, mesh {:.3} s, read {:.3} s of which check {:.3} s over {solids_read} solids read, total {:.3} s",
        report.cases.len(),
        sum(" build"),
        sum(" mesh"),
        sum(" read"),
        sum(" check"),
        report.total()
    );
    if let Some(path) = &args.compare {
        match Report::load(path) {
            Ok(before) => {
                let table = bench::comparison_table(&bench::compare(&before, &report));
                println!("\nAgainst {}:\n\n{table}", path.display());
                if let Some(out) = &args.table {
                    if let Err(e) = std::fs::write(out, &table) {
                        eprintln!("corpus bench: {}: {e}", out.display());
                        return ExitCode::FAILURE;
                    }
                }
            }
            Err(e) => {
                eprintln!("corpus bench: {}: {e}", path.display());
                return ExitCode::FAILURE;
            }
        }
    }
    if let Some(path) = &args.save {
        if let Err(e) = report.save(path) {
            eprintln!("corpus bench: {}: {e}", path.display());
            return ExitCode::FAILURE;
        }
        println!("saved {}", path.display());
    }
    ExitCode::SUCCESS
}
