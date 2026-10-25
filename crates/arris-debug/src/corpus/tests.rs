use super::*;
use crate::unmetered::primitive_box;

/// The recipe's `precision` is the model's, and an inconsistent one
/// is the fixture's own error rather than a panic.
#[test]
fn the_recipe_names_the_model_s_precision() {
    let metres = crate::fixtures::corpus_root().join("boolean/probe-through-hole-m");
    let metric = chain(&metres, "default").expect("the consumer's units");
    assert_eq!(metric.model.precision().default_tolerance, 1e-6);

    let millimetres = crate::fixtures::corpus_root().join("boolean/through-hole");
    let default = chain(&millimetres, "default").expect("the default units");
    assert_eq!(default.model.precision(), arris_math::Precision::DEFAULT);

    // A floor above the default tolerance is no precision at all, and
    // the runner says which fixture rather than unwrapping.
    let scratch =
        std::env::temp_dir().join(format!("arris-corpus-precision-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&scratch);
    std::fs::create_dir_all(&scratch).expect("a scratch directory");
    for file in ["fixture.json", "expected.json"] {
        std::fs::copy(millimetres.join(file), scratch.join(file)).expect("a copy");
    }
    let text = std::fs::read_to_string(scratch.join("fixture.json")).expect("the recipe");
    std::fs::write(
        scratch.join("fixture.json"),
        text.replace(
            "\"steps\"",
            "\"precision\": {\"min_tolerance\": 1.0}, \"steps\"",
        ),
    )
    .expect("the edited recipe");
    assert!(matches!(
        chain(&scratch, "default"),
        Err(CorpusError::Precision { .. })
    ));
    let _ = std::fs::remove_dir_all(&scratch);
}

/// A `step` operand reads only the file it hashes, names a solid the
/// file has, and carries the reader's refusal as its own.
#[test]
fn a_step_operand_is_the_solid_of_the_file_it_hashes() {
    let dir = crate::fixtures::corpus_root().join("boolean/step-operand-cut");
    let file = "operand.step";
    let bytes = std::fs::read(dir.join(file)).expect("the committed operand");
    let digest: String = Sha256::digest(&bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    let probe = Tolerances::default().probe;
    let mut m = Model::default();
    let read = read_solid(&mut m, &dir, file, &digest, 22, None, probe)
        .unwrap_or_else(|_| panic!("#22 of {file}"));
    assert!(check(&m, read.body, Level::Full).is_ok());

    let fault = |r: Result<io_step::ReadBody, ReadFault>| match r {
        Ok(_) => panic!("read a solid it should not have"),
        Err(ReadFault::File(what)) => what,
        Err(ReadFault::Refused(r)) => format!("refused: {r}"),
    };
    let mut m = Model::default();
    let other = "0".repeat(64);
    let what = fault(read_solid(&mut m, &dir, file, &other, 22, None, probe));
    assert!(what.contains("not the recipe's"), "{what}");
    let what = fault(read_solid(&mut m, &dir, file, &digest, 23, None, probe));
    assert!(what.contains("has no solid #23"), "{what}");
    let what = fault(read_solid(
        &mut m,
        &dir,
        "absent.step",
        &digest,
        22,
        None,
        probe,
    ));
    assert!(what.contains("absent.step"), "{what}");

    // The same file with its context's length unit taken away: the
    // reader refuses the solid, and the operand is that refusal.
    let scratch =
        std::env::temp_dir().join(format!("arris-corpus-step-operand-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&scratch);
    std::fs::create_dir_all(&scratch).expect("a scratch directory");
    let text = String::from_utf8(bytes).expect("Arris writes ASCII");
    let unitless = text.replace(
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((#10,",
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((",
    );
    assert_ne!(unitless, text, "the operand declares a length unit");
    std::fs::write(scratch.join(file), &unitless).expect("the edited operand");
    let digest: String = Sha256::digest(unitless.as_bytes())
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    let mut m = Model::default();
    let what = fault(read_solid(&mut m, &scratch, file, &digest, 22, None, probe));
    assert!(what.starts_with("refused: "), "{what}");
    let _ = std::fs::remove_dir_all(&scratch);
}

#[test]
fn one_recipe_in_two_directories_names_two_step_files() {
    // The corpus's own copy keeps the name the docs quote.
    let canonical = crate::fixtures::corpus_root().join("primitive/cylinder");
    assert_eq!(
        step_tag("primitive/cylinder", "default", &canonical),
        "primitive-cylinder-default"
    );
    // A scratch copy of the same recipe — what this module's own tests
    // build — does not, so the two never write one file while they run
    // at the same time.
    let scratch = Path::new("/tmp/arris-corpus-dump-diff-1");
    let other = Path::new("/tmp/arris-corpus-dump-diff-2");
    let a = step_tag("primitive/cylinder", "default", scratch);
    let b = step_tag("primitive/cylinder", "default", other);
    assert!(a.starts_with("primitive-cylinder-default-"), "{a}");
    assert_ne!(a, "primitive-cylinder-default");
    assert_ne!(a, b, "two scratch copies name two files");
    assert_eq!(a, step_tag("primitive/cylinder", "default", scratch));
}

#[test]
fn diff_names_the_lines_that_differ() {
    let d = diff("a\nb\nc\n", "a\nx\nc\nd\n");
    assert_eq!(
        d,
        "-    2 b\n+    2 x\n+    4 d\n(3 lines committed, 4 in this build)\n"
    );
    assert_eq!(diff("same\n", "same\n"), "");
}

#[test]
fn the_measure_stage_holds_the_box_to_the_oracle_and_names_what_differs() {
    let dir = fixtures::corpus_root().join("primitive/box");
    let fixture = fixtures::load(&dir).unwrap();
    let expected = fixture.expected.results["default"].clone();
    let tolerances = fixture.recipe.tolerances;
    let mut m = Model::default();
    let (body, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0))
        .expect("the box of the recipe");
    compare_mass(&m, body, &expected, "the oracle's", &tolerances).expect("the oracle's numbers");
    // Every quantity is actually compared: move each one just past
    // its tolerance and the stage says which.
    let mut wrong = expected.clone();
    wrong.volume = Some(expected.volume.unwrap() * (1.0 + 1e-6));
    assert!(
        compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
            .unwrap_err()
            .starts_with("volume ")
    );
    let mut wrong = expected.clone();
    wrong.area = Some(expected.area.unwrap() * (1.0 + 1e-6));
    assert!(
        compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
            .unwrap_err()
            .starts_with("area ")
    );
    let mut wrong = expected.clone();
    let mut centroid = expected.centroid.unwrap();
    centroid[1] += 1e-3;
    wrong.centroid = Some(centroid);
    assert!(
        compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
            .unwrap_err()
            .starts_with("centroid ")
    );
    let mut wrong = expected.clone();
    let mut inertia = expected.inertia.unwrap();
    inertia[0][1] += inertia[2][2] * 1e-6;
    wrong.inertia = Some(inertia);
    assert!(
        compare_mass(&m, body, &wrong, "the oracle's", &tolerances)
            .unwrap_err()
            .starts_with("inertia[0][1] ")
    );
}

/// Under `measure_differs` the stage is held to the recipe's closed
/// forms, every one of which it needs (ADR-0015); without it, to the
/// oracle's measurements.
#[test]
fn measure_differs_holds_the_stage_to_the_closed_forms() {
    let dir = fixtures::corpus_root().join("boolean/cross-cylinders-fuse");
    let mut fixture = fixtures::load(&dir).unwrap();
    let params = fixture.recipe.params_of("default").unwrap();
    let expected = fixture.expected.results["default"].clone();
    let (target, by) = measure_target(&fixture, &params, &expected).unwrap();
    assert_eq!((target, by), (expected.clone(), "the oracle's"));

    fixture.recipe.analytic.measure_differs = Some("a test".into());
    let Err(CorpusError::Measure { what, .. }) = measure_target(&fixture, &params, &expected)
    else {
        panic!("a closed form missing is an error");
    };
    assert_eq!(what, "analytic.measure_differs needs analytic.inertia");

    let row = |r: [&str; 3]| r.map(|x| Num::Expr(x.into()));
    fixture.recipe.analytic.inertia = Some([
        row(["2", "0", "0"]),
        row(["0", "3", "0"]),
        row(["0", "0", "R + 3"]),
    ]);
    let (target, by) = measure_target(&fixture, &params, &expected).unwrap();
    assert_eq!(by, "the closed form's");
    let volume = 2.0 * std::f64::consts::PI * 6.0 - 16.0 / 3.0;
    assert!((target.volume.unwrap() - volume).abs() < 1e-12);
    assert_eq!(target.centroid, Some([0.0; 3]));
    assert_eq!(target.inertia.unwrap()[2][2], 4.0);
    // What the oracle says of everything else is kept.
    assert_eq!(target.counts, expected.counts);
    assert_eq!(target.probes, expected.probes);
}

#[test]
fn dump_paths_and_blessing() {
    let dir = Path::new("x");
    assert_eq!(dump_path(dir, "default"), Path::new("x/dump.txt"));
    assert_eq!(dump_path(dir, "tighter"), Path::new("x/dump.tighter.txt"));
}
