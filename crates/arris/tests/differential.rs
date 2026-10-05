//! The differential (ADR-0024 §2): `ARRIS_DIFF_CASES` recipes (default
//! 32) drawn from the property seed, built by Arris and by the Open
//! CASCADE oracle, every outcome classed. A disagreement, a checker
//! violation, a panic or an internal fault fails the run, shrunk to a `fixture.json` for
//! `tests/fixtures/regression/`; refusals are counted, per reason. The
//! histogram prints either way.
//!
//! A failure a named exclusion covers is counted under its name
//! (`differential::EXCLUSIONS`), each citing the regression fixtures that
//! pin it. CI runs 1000 recipes, and the hook 32.

use arris_debug::{differential, prop};

#[test]
fn arris_and_the_oracle_agree_on_drawn_recipes() {
    let run = differential::run(&prop::seed(), differential::cases())
        .unwrap_or_else(|e| panic!("the differential could not run: {e}"));
    println!("{}", run.report());
    assert!(
        run.failures.is_empty(),
        "{}\n{} failing recipes:\n\n{}",
        run.report(),
        run.failures.len(),
        run.failures_text()
    );
}

/// The turned parts alone (ADR-0036): a coned shoulder, a dome or a
/// toroidal bead with a pick of its circular corners blended in one call,
/// each against Open CASCADE. The recipes the meridian row builds are
/// counted among the agreeing — Open CASCADE walks its torus and its
/// cone-against-cylinder blends, so the mass properties are held to what
/// both shapes' own tolerances support — and none may disagree or be
/// refused by Arris alone.
#[test]
fn arris_and_the_oracle_agree_on_turned_parts() {
    let run = differential::run_over(
        "turned",
        arris_debug::prop::recipe::turned_recipe(),
        &prop::seed(),
        differential::cases(),
    )
    .unwrap_or_else(|e| panic!("the differential could not run: {e}"));
    println!("{}", run.report());
    assert!(
        run.failures.is_empty(),
        "{}\n{} failing recipes:\n\n{}",
        run.report(),
        run.failures.len(),
        run.failures_text()
    );
    let histogram = run.histogram();
    let count = |class: &str| histogram.get(class).copied().unwrap_or(0);
    assert_eq!(
        count("ArrisRefuses"),
        0,
        "Arris refuses a turned part Open CASCADE builds\n{}",
        run.report()
    );
    assert!(
        count("Agree") > 0,
        "no turned part compared\n{}",
        run.report()
    );
}

/// The multi-tool booleans (ADR-0050): one `cut` with two to four tools
/// or one `fuse` of three to five bodies, in one call, against Open
/// CASCADE's multi-tool `BRepAlgoAPI_Cut` and `Fuse`. Judged as the
/// general draw is: an agreement, a named exclusion or a refusal, never a
/// disagreement.
#[test]
fn arris_and_the_oracle_agree_on_multi_tool_booleans() {
    let run = differential::run_over(
        "multi-tool",
        arris_debug::prop::recipe::multi_tool_recipe(),
        &prop::seed(),
        differential::cases(),
    )
    .unwrap_or_else(|e| panic!("the differential could not run: {e}"));
    println!("{}", run.report());
    assert!(
        run.failures.is_empty(),
        "{}\n{} failing recipes:\n\n{}",
        run.report(),
        run.failures.len(),
        run.failures_text()
    );
}

/// Where Open CASCADE's boolean records no solid of operands it built, or
/// a volume they cannot bound (a fuse smaller than one operand), and
/// Arris's result is a valid body that adds up (`V(A ∪ B) + V(A ∩ B) =
/// V(A) + V(B)`), the case is an oracle refusal, counted, not a
/// disagreement (ADR-0046; the cases, with the oracle's answers as it gave
/// them, are under `tests/oracle-wrong/`: nightly 2026-10-01's 335, 856
/// and 696). A result that does not add up stays a disagreement.
#[test]
fn an_oracle_with_no_solid_holds_arris_to_additivity() {
    use arris_debug::{differential::Outcome, fixtures};
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/oracle-wrong");
    for slug in [
        "revolve-pair-fuse-oracle-no-solid",
        "revolve-triple-fuse-common-oracle-no-solid",
        "extrude-mirrored-revolve-fuse-oracle-drops-an-operand",
    ] {
        let fixture = fixtures::load(&root.join(slug)).unwrap();
        let outcome = differential::judge(slug, &fixture.recipe, Ok(&fixture));
        assert!(
            matches!(&outcome, Outcome::OracleRefuses(why) if why.contains("additive")),
            "{slug}: {outcome}"
        );
    }
}
