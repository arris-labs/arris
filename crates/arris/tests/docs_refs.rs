//! The docs-reference lint: a plan is deleted on retirement
//! (`.agents/rules/docs-lifecycle.md`), so a citation of
//! `docs/plans/<slug>` or `plans/<slug>` left behind in source, tooling or
//! `Cargo.toml` after that outlives the file it points at. This proves the
//! lint catches that the way `corpus_lint.rs` proves its own rules: against
//! a scratch tree with a citation deliberately left dangling, not just by
//! running clean against the real one. Beside it, the refusal histogram
//! `docs/ROADMAP.md` records for the committed tier is held to the one
//! its fixtures print. And `CHANGELOG.md` is held to the shape `/release`
//! and `tools/semver-gate.sh` read (ADR-0027): `## Unreleased` first, then
//! released versions newest first, and a copy in every published crate.

use std::path::{Path, PathBuf};

/// Every `plans/<slug>` substring in `text`, deduplicated. `docs/plans/x`
/// and the commit-message form `(plans/x step N)` both end in `plans/x`,
/// so one scan catches both.
fn plan_refs(text: &str) -> Vec<String> {
    let mut refs = Vec::new();
    let mut from = 0;
    while let Some(pos) = text[from..].find("plans/") {
        let slug_start = from + pos + "plans/".len();
        let slug_end = text[slug_start..]
            .find(|c: char| !(c.is_ascii_alphanumeric() || c == '-'))
            .map_or(text.len(), |o| slug_start + o);
        let slug = &text[slug_start..slug_end];
        if !slug.is_empty() {
            refs.push(slug.to_string());
        }
        from = slug_end.max(slug_start);
    }
    refs.sort();
    refs.dedup();
    refs
}

fn walk(dir: &Path, visit: &mut impl FnMut(&Path)) {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.file_name().is_some_and(|n| n == "target") {
            continue;
        }
        if path.is_dir() {
            walk(&path, visit);
        } else {
            visit(&path);
        }
    }
}

fn check_file(root: &Path, path: &Path, problems: &mut Vec<String>) {
    // This lint's own source is full of example citations, not real ones.
    if path.file_name().is_some_and(|n| n == "docs_refs.rs") {
        return;
    }
    let Ok(text) = std::fs::read_to_string(path) else {
        return;
    };
    for slug in plan_refs(&text) {
        let plan = root.join("docs/plans").join(format!("{slug}.md"));
        if !plan.exists() {
            problems.push(format!(
                "{}: cites plan '{slug}' but {} does not exist",
                path.display(),
                plan.display()
            ));
        }
    }
}

/// Every `docs/plans/<slug>` or `plans/<slug>` citation under `crates/`,
/// `tools/`, `.githooks/` and the root `Cargo.toml`, naming a plan that is
/// not there.
fn stale_plan_refs(root: &Path) -> Vec<String> {
    let mut problems = Vec::new();
    for dir in ["crates", "tools", ".githooks"] {
        walk(&root.join(dir), &mut |path| {
            check_file(root, path, &mut problems)
        });
    }
    check_file(root, &root.join("Cargo.toml"), &mut problems);
    problems.sort();
    problems
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .unwrap()
}

#[test]
fn the_repo_cites_only_plans_that_still_exist() {
    let problems = stale_plan_refs(&repo_root());
    assert!(
        problems.is_empty(),
        "stale plan citations:\n{}",
        problems.join("\n")
    );
}

/// A scratch tree with a citation of a plan that was never written, next to
/// one that names a plan that is: the lint names the dangling one and only
/// that one.
#[test]
fn a_citation_of_a_retired_plan_is_caught() {
    let dir = std::env::temp_dir().join(format!("arris-docs-refs-lint-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(dir.join("docs/plans")).unwrap();
    std::fs::write(dir.join("docs/plans/still-open.md"), "# Plan: still-open\n").unwrap();
    std::fs::create_dir_all(dir.join("crates/fake/src")).unwrap();
    std::fs::write(
        dir.join("crates/fake/src/lib.rs"),
        "//! see docs/plans/still-open.md for the design\n\
         //! and (plans/retired-plan step 3) for why this exists\n",
    )
    .unwrap();

    let problems = stale_plan_refs(&dir);
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(problems[0].contains("retired-plan"), "{problems:?}");
    assert!(!problems[0].contains("still-open"), "{problems:?}");

    std::fs::remove_dir_all(&dir).unwrap();
}

/// The committed tier's refusal histogram in `docs/ROADMAP.md` §C4 is the
/// one its fixtures print today (ADR-0026 §5): a fixture whose outcome or
/// cycle moves fails here until the roadmap's table is printed again.
#[test]
fn the_roadmap_holds_the_committed_tier_histogram() {
    use arris_debug::histogram::{COMMITTED_TIER, Histogram};
    let mut histogram = Histogram::new();
    for name in COMMITTED_TIER {
        let fixture = arris_debug::part::load(&arris_debug::fixtures::corpus_root().join(name))
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        histogram
            .add_part(&fixture)
            .unwrap_or_else(|e| panic!("{e}"));
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let roadmap = std::fs::read_to_string(root.join("docs/ROADMAP.md")).unwrap();
    let open = "<!-- histogram: committed -->\n";
    let start = roadmap
        .find(open)
        .expect("the committed tier's histogram block")
        + open.len();
    let end = start
        + roadmap[start..]
            .find("<!-- /histogram -->")
            .expect("the block's end");
    assert_eq!(
        &roadmap[start..end],
        histogram.markdown(),
        "print it again: cargo run -p arris-debug --example real_parts -- --committed"
    );
}

/// A released heading's `(major, minor, patch)` and date, from
/// `## X.Y.Z — YYYY-MM-DD`.
fn release_heading(heading: &str) -> Option<((u64, u64, u64), &str)> {
    let (version, date) = heading.split_once(" — ")?;
    let mut parts = version.split('.').map(|p| p.parse::<u64>().ok());
    let v = (parts.next()??, parts.next()??, parts.next()??);
    let is_date = date.len() == 10
        && date.char_indices().all(|(i, c)| {
            if i == 4 || i == 7 {
                c == '-'
            } else {
                c.is_ascii_digit()
            }
        });
    (parts.next().is_none() && is_date).then_some((v, date))
}

/// What is wrong with a changelog's `##` headings: the first must be
/// `Unreleased`, every other a release heading, versions strictly and dates
/// weakly descending.
fn changelog_problems(text: &str) -> Vec<String> {
    let headings: Vec<&str> = text.lines().filter_map(|l| l.strip_prefix("## ")).collect();
    let mut problems = Vec::new();
    if headings.first() != Some(&"Unreleased") {
        problems.push("the first `##` heading is not `Unreleased`".to_string());
    }
    let mut previous: Option<((u64, u64, u64), &str)> = None;
    for heading in headings.iter().skip(1) {
        let Some((v, date)) = release_heading(heading) else {
            problems.push(format!("`## {heading}` is not `## X.Y.Z — YYYY-MM-DD`"));
            continue;
        };
        if let Some((pv, pdate)) = previous
            && (v >= pv || date > pdate)
        {
            problems.push(format!(
                "`## {heading}` is not older than the section above it"
            ));
        }
        previous = Some((v, date));
    }
    problems
}

#[test]
fn the_changelog_opens_on_unreleased_and_runs_newest_first() {
    let text = std::fs::read_to_string(repo_root().join("CHANGELOG.md")).unwrap();
    let problems = changelog_problems(&text);
    assert!(
        problems.is_empty(),
        "CHANGELOG.md:\n{}",
        problems.join("\n")
    );
}

/// Each malformation is named, and a well-formed changelog passes.
#[test]
fn a_malformed_changelog_is_caught() {
    let good = "# Changelog\n\n## Unreleased\n\n### Breaking\n\n## 0.2.0 — 2026-09-24\n\n## 0.1.1 — 2026-09-18\n";
    assert!(changelog_problems(good).is_empty());
    let no_unreleased = "## 0.2.0 — 2026-09-24\n";
    assert_eq!(changelog_problems(no_unreleased).len(), 1);
    let bad_heading = "## Unreleased\n## 0.2.0 (2026-09-24)\n";
    assert_eq!(changelog_problems(bad_heading).len(), 1);
    let out_of_order = "## Unreleased\n## 0.1.1 — 2026-09-18\n## 0.2.0 — 2026-09-24\n";
    assert_eq!(changelog_problems(out_of_order).len(), 1);
}

/// Every crate that is published ships the changelog: a `CHANGELOG.md` in
/// its directory that reads as the root one (a symlink to it, as the
/// licences are), so `cargo package` puts it in the tarball.
#[test]
fn every_published_crate_ships_the_changelog() {
    let root = repo_root();
    let changelog = std::fs::read_to_string(root.join("CHANGELOG.md")).unwrap();
    let mut problems = Vec::new();
    let mut published = 0;
    for entry in std::fs::read_dir(root.join("crates")).unwrap().flatten() {
        let dir = entry.path();
        let Ok(manifest) = std::fs::read_to_string(dir.join("Cargo.toml")) else {
            continue;
        };
        if manifest.lines().any(|l| l.trim() == "publish = false") {
            continue;
        }
        published += 1;
        match std::fs::read_to_string(dir.join("CHANGELOG.md")) {
            Ok(copy) if copy == changelog => {}
            Ok(_) => problems.push(format!(
                "{}: CHANGELOG.md differs from the root one",
                dir.display()
            )),
            Err(_) => problems.push(format!("{}: no CHANGELOG.md", dir.display())),
        }
    }
    assert_eq!(published, 8, "the eight published crates");
    assert!(problems.is_empty(), "{}", problems.join("\n"));
}
