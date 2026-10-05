//! A part the corpus does not hold, surveyed (ADR-0026 §2 and its
//! amendment of step 9): the fetched tier's unit of work, run by
//! `tools/real-parts.sh` through the `real_parts` example, one process per
//! file so that a read that never ends is the script's timeout and not the
//! run's.
//!
//! [`survey`] holds a STEP file as the part runner holds a committed part
//! (`crate::part::run`), but counts instead of stopping: it writes a
//! scratch part fixture beside the file, has Open CASCADE read it through
//! the oracle's cache, holds every solid Arris reads to the oracle's
//! reading, derives and runs the battery on the first placement of each
//! solid read, and returns a [`Report`]. A refusal is counted under the
//! cycle it blocks; anything that is a kernel bug — a panic, a checker
//! violation, a solid outside the oracle's measures or one the file does
//! not have, a battery stage the differential calls a bug, a read past
//! ADR-0026 §6's budget — is a failure, named. A file whose every solid
//! Arris refuses is not read by the oracle: there is nothing to hold to
//! it, and its measures of a large spline-bounded part can outlast the
//! survey. A file the reader returns nothing of is a failure.

use crate::unmetered::step_read;
use std::collections::BTreeMap;
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::Path;

use arris_io::step::ReadOptions;
use arris_topo::Model;
use serde::{Deserialize, Serialize};

use crate::battery::{self, Class};
use crate::differential::{Outcome, panicked};
use crate::fixtures::{PrecisionSpec, ReadRefused, Tolerances};
use crate::histogram::{Stage, blocks_parse};
use crate::oracle;
use crate::part::{self, Outcome as ReadOutcome, Part, PartSolid};

/// How long a read may take in a release build (ADR-0026 §6): one that
/// does not finish within it is a reader bug.
pub const READ_BUDGET_SECONDS: f64 = 60.0;

/// One refusal a survey counted.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Counted {
    /// The solid, `#id[instance]`, or the file for a parse error.
    pub at: String,
    /// The stage, as [`Stage::name`] prints it.
    pub stage: String,
    /// The refusal's name: the reader's kind, or the differential's name
    /// of the operation's error.
    pub name: String,
    /// The cycle it blocks, as `Cycle` prints it.
    pub cycle: String,
}

/// What a survey of one file found.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct Report {
    /// The part's name: the file's stem.
    pub part: String,
    /// How long Arris's read took, in seconds.
    pub read_seconds: f64,
    /// Solid instances read.
    pub read: usize,
    /// Occurrences in the file's product tree (ADR-0033): printed, never
    /// asserted beyond "the tree was built".
    #[serde(default)]
    pub occurrences: usize,
    /// Every refusal, the reader's and the battery's Arris refusals.
    pub refusals: Vec<Counted>,
    /// Each battery stage's class, `(solid, stage, class)`: `agree`,
    /// `both-refuse`, `oracle-refuses` or `arris-refuses`.
    pub classes: Vec<(String, String, String)>,
    /// Every kernel bug met, in words.
    pub failures: Vec<String>,
    /// The fillet column of each solid whose fillet stage Arris refuses
    /// and Open CASCADE builds, by part (`crate::census`).
    #[serde(default)]
    pub fillet_census: Vec<crate::census::SolidCensus>,
}

/// The class's name, as a fixture records it.
fn class_name(class: &Class) -> &'static str {
    match class {
        Class::Agree => "agree",
        Class::BothRefuse => "both-refuse",
        Class::OracleRefuses => "oracle-refuses",
        Class::ArrisRefuses(_) => "arris-refuses",
        Class::WaitsOn(_) => "waits-on",
    }
}

/// Surveys the STEP file `file` as the part `name`, in the scratch
/// directory `work` (created; the file is copied into it, and a
/// `fixture.json` and the oracle's `expected.json` written beside it).
/// Deterministic but for `read_seconds`. Errors: the file cannot be read
/// or the scratch fixture written, or the oracle has no environment to
/// run in — never a refusal or a bug of the kernel's, which the report
/// holds.
pub fn survey(file: &Path, name: &str, work: &Path, source: &str) -> Result<Report, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let file_name = file
        .file_name()
        .and_then(|n| n.to_str())
        .ok_or_else(|| format!("{}: no file name", file.display()))?
        .to_string();
    std::fs::create_dir_all(work).map_err(|e| format!("{}: {e}", work.display()))?;
    std::fs::write(work.join(&file_name), &bytes)
        .map_err(|e| format!("{}: {e}", work.display()))?;
    let mut report = Report {
        part: name.to_string(),
        ..Report::default()
    };
    let precision = PrecisionSpec::default();
    let mut m = Model::new(precision.precision()).map_err(|e| e.to_string())?;
    let text = String::from_utf8_lossy(&bytes);
    let started = std::time::Instant::now();
    let read = match catch_unwind(AssertUnwindSafe(|| {
        step_read(&mut m, &text, &ReadOptions::default())
    })) {
        Ok(Ok(read)) => read,
        Ok(Err(e)) => {
            report.read_seconds = started.elapsed().as_secs_f64();
            report.refusals.push(Counted {
                at: file_name,
                stage: Stage::Read.name().into(),
                name: "unparsed".into(),
                cycle: blocks_parse(&e).to_string(),
            });
            return Ok(report);
        }
        Err(payload) => {
            report
                .failures
                .push(format!("the read panics: {}", panicked(&*payload)));
            return Ok(report);
        }
    };
    report.read_seconds = started.elapsed().as_secs_f64();
    report.occurrences = read.products.occurrences().count();
    if report.read_seconds > READ_BUDGET_SECONDS {
        report.failures.push(format!(
            "the read takes {:.1} s, past ADR-0026 §6's {READ_BUDGET_SECONDS} s",
            report.read_seconds
        ));
    }

    // The scratch fixture: the solids as Arris reads them.
    let solids: Vec<PartSolid> = (read.solids.iter())
        .map(|s| PartSolid {
            id: s.entity.id,
            instance: s.entity.instance,
            outcome: match &s.result {
                Ok(_) => ReadOutcome::Read,
                Err(r) => ReadOutcome::Refused(ReadRefused {
                    kind: r.kind().to_string(),
                    why: r.to_string(),
                }),
            },
            battery: BTreeMap::new(),
            blocks: BTreeMap::new(),
        })
        .collect();
    let mut spec = Part {
        description: format!("the fetched tier's {name}"),
        file: file_name,
        source: source.to_string(),
        licence: String::new(),
        sha256: crate::part::sha256_hex(&bytes),
        solids,
        precision,
        tolerances: Tolerances::default(),
        waits_on: Vec::new(),
        read_seconds: None,
        battery: BTreeMap::new(),
    };
    if read.solids.is_empty() {
        report
            .failures
            .push("the reader returns no solid and no refusal: a solid it does not see".into());
        return Ok(report);
    }
    if read.solids.iter().all(|s| s.result.is_err()) {
        // Nothing to hold to the oracle, and its measures of a large
        // spline-bounded part can take longer than the survey may: every
        // refusal counts, and no oracle is asked.
        for (solid, spec) in read.solids.iter().zip(&spec.solids) {
            if let Err(r) = &solid.result {
                report.refusals.push(Counted {
                    at: battery::key(spec.id, spec.instance),
                    stage: Stage::Read.name().into(),
                    name: r.kind().to_string(),
                    cycle: crate::histogram::blocks_refusal(r).to_string(),
                });
            }
        }
        return Ok(report);
    }
    part::save(work, &spec).map_err(|e| format!("{}: {e}", work.display()))?;
    let answered = |what: &str| -> Result<Result<(), String>, String> {
        let mut answers =
            oracle::expected_batch(&[work.to_path_buf()]).map_err(|e| e.to_string())?;
        Ok(answers
            .pop()
            .unwrap_or_else(|| Err("no answer".into()))
            .map_err(|why| format!("Open CASCADE refuses the {what}: {why}")))
    };
    let oracle_reads = answered("file")?;

    if let Err(why) = oracle_reads {
        // Nothing to hold a read to: every read is unverified, and every
        // refusal still counts.
        for (solid, spec) in read.solids.iter().zip(&spec.solids) {
            match &solid.result {
                Ok(_) => report.read += 1,
                Err(r) => report.refusals.push(Counted {
                    at: battery::key(spec.id, spec.instance),
                    stage: Stage::Read.name().into(),
                    name: r.kind().to_string(),
                    cycle: crate::histogram::blocks_refusal(r).to_string(),
                }),
            }
        }
        if report.read > 0 {
            report.failures.push(why);
        }
        return Ok(report);
    }

    // Each solid: a refusal counted, a read held to the oracle.
    let fixture = match part::load(work) {
        Ok(f) => f,
        Err(e) => {
            report.failures.push(format!("the scratch fixture: {e}"));
            return Ok(report);
        }
    };
    let oracle = &fixture.expected.solids;
    let mut taken = vec![false; oracle.len()];
    let mut refused: BTreeMap<u64, usize> = BTreeMap::new();
    let mut held = vec![false; read.solids.len()];
    for (k, (solid, spec)) in read.solids.iter().zip(&fixture.part.solids).enumerate() {
        let at = battery::key(spec.id, spec.instance);
        match &solid.result {
            Err(r) => {
                *refused.entry(spec.id).or_default() += 1;
                report.refusals.push(Counted {
                    at,
                    stage: Stage::Read.name().into(),
                    name: r.kind().to_string(),
                    cycle: crate::histogram::blocks_refusal(r).to_string(),
                });
            }
            Ok(back) => {
                report.read += 1;
                if fixture.expected.solids.is_empty() {
                    continue;
                }
                let stages = catch_unwind(AssertUnwindSafe(|| {
                    part::read_stages(&fixture, &m, back.body, spec, &mut taken, false)
                }));
                match stages {
                    Ok(Ok(())) => held[k] = true,
                    Ok(Err(e)) => report.failures.push(format!("{at}: {e}")),
                    Err(payload) => report
                        .failures
                        .push(format!("{at}: {}", panicked(&*payload))),
                }
            }
        }
    }
    let mut left: BTreeMap<u64, usize> = BTreeMap::new();
    for (solid, _) in oracle.iter().zip(&taken).filter(|(_, t)| !**t) {
        *left.entry(solid.id).or_default() += 1;
    }
    for (id, n) in left {
        let refusals = refused.get(&id).copied().unwrap_or(0);
        let failed = (read.solids.iter().zip(&held))
            .filter(|(s, h)| s.entity.id == id && s.result.is_ok() && !**h)
            .count();
        if n > refusals + failed {
            report.failures.push(format!(
                "Open CASCADE reads {n} solid(s) of #{id} that Arris neither reads nor refuses"
            ));
        }
    }

    // The battery, on the first placement of each solid held.
    let mut first: Vec<u64> = Vec::new();
    for (s, (solid, h)) in spec.solids.iter_mut().zip(read.solids.iter().zip(&held)) {
        let run = *h && !first.contains(&solid.entity.id);
        if run {
            first.push(solid.entity.id);
        } else if s.outcome == ReadOutcome::Read {
            // Kept out of the battery: a later placement, or a read that
            // failed its stages.
            s.outcome = ReadOutcome::Refused(ReadRefused {
                kind: "not in the battery".into(),
                why: String::new(),
            });
        }
    }
    if first.is_empty() {
        return Ok(report);
    }
    let mut with_battery = fixture.clone();
    with_battery.part.solids = spec.solids.clone();
    spec.battery = match battery::derive_from(&with_battery, &m, &read) {
        Ok(b) => b,
        Err(e) => {
            report.failures.push(format!("the battery's operands: {e}"));
            return Ok(report);
        }
    };
    part::save(work, &spec).map_err(|e| format!("{}: {e}", work.display()))?;
    if let Err(why) = answered("battery")? {
        report.failures.push(why);
        return Ok(report);
    }
    let fixture = match part::load(work) {
        Ok(f) => f,
        Err(e) => {
            report.failures.push(format!("the scratch fixture: {e}"));
            return Ok(report);
        }
    };
    for (solid, s) in read.solids.iter().zip(&fixture.part.solids) {
        let key = battery::key(s.id, s.instance);
        let (Some(cases), Ok(back)) = (fixture.part.battery.get(&key), &solid.result) else {
            continue;
        };
        let hold = battery::holds_counts(&fixture, s.id);
        let mut stages = vec![(
            "write_read".to_string(),
            battery::write_read(
                &format!("{name} {key} write_read"),
                &m,
                back.body,
                &fixture.part.tolerances,
            ),
        )];
        for (stage, case) in cases {
            let judged = match battery::oracle_case(&fixture, &key, stage) {
                Ok(oracle) => battery::judge(
                    &fixture,
                    &format!("{name} {key} {stage}"),
                    case,
                    oracle,
                    hold,
                    Some((&m, back)),
                ),
                Err(e) => {
                    report.failures.push(e);
                    continue;
                }
            };
            stages.push((stage.clone(), judged));
        }
        for (stage, judged) in stages {
            let Some(class) = Class::of(&judged.outcome) else {
                report
                    .failures
                    .push(format!("{key} {stage}: {}", judged.outcome));
                continue;
            };
            report
                .classes
                .push((key.clone(), stage.clone(), class_name(&class).into()));
            if stage == "fillet" && matches!(class, Class::ArrisRefuses(_)) {
                match catch_unwind(AssertUnwindSafe(|| {
                    crate::census::blend_census(&m, back.body, &key)
                })) {
                    Ok(Ok(census)) => report.fillet_census.push(census),
                    Ok(Err(e)) => report.failures.push(format!("{key} census: {e}")),
                    Err(payload) => report
                        .failures
                        .push(format!("{key} census: {}", panicked(&*payload))),
                }
            }
            let (Outcome::ArrisRefuses(why), Some(refused)) = (&judged.outcome, &judged.refused)
            else {
                continue;
            };
            let Some(at) = Stage::of_name(&stage) else {
                continue;
            };
            match refused.blocks_in(&m, at) {
                Some(cycle) => report.refusals.push(Counted {
                    at: key.clone(),
                    stage,
                    name: why.clone(),
                    cycle: cycle.to_string(),
                }),
                None => report.failures.push(format!("{key} {stage}: {why}")),
            }
        }
    }
    Ok(report)
}
