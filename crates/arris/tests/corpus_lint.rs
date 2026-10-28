//! The corpus lint (`docs/ROADMAP.md` §Fixtures): every fixture directory
//! has a recipe and the oracle's answer, the answer is not stale, the Euler
//! line is zero, the closed forms agree with the oracle, and every
//! fixture the runner compares in the corpus's areas carries its blessed
//! dump — zero ignored fixtures there as a test, not a grep — and every
//! exclusion of the differential waits on a fixture under `regression/`.

use std::path::Path;

use arris_debug::differential;
use arris_debug::fixtures::{Kind, corpus, corpus_root, kind_of, lint, load};
use arris_debug::oracle::workspace_root;

#[test]
fn every_fixture_directory_is_clean() {
    let dirs = corpus();
    assert!(
        dirs.len() >= 18,
        "expected the C1 corpus and the two geometry fixtures, found {} directories",
        dirs.len()
    );
    let problems: Vec<String> = dirs.iter().flat_map(|d| lint(d)).collect();
    assert!(problems.is_empty(), "corpus lint:\n{}", problems.join("\n"));
}

/// Every named exclusion of the differential cites fixtures still
/// waiting under `regression/`; the fix that moves one out lifts it
/// (ADR-0024 §2).
#[test]
fn every_differential_exclusion_waits_on_a_regression_fixture() {
    let problems = differential::exclusion_problems(&corpus_root(), differential::EXCLUSIONS);
    assert!(problems.is_empty(), "{}", problems.join("\n"));
}

#[test]
fn every_row_of_the_roadmap_table_has_a_fixture() {
    let names: Vec<String> = corpus()
        .iter()
        .filter(|d| kind_of(d).unwrap() == Kind::Solid)
        .map(|d| load(d).unwrap().name)
        .collect();
    for expected in [
        "primitive/box",
        "primitive/cylinder",
        "boolean/through-hole",
        "boolean/blind-hole",
        "boolean/bolt-pattern-8",
        "boolean/flush-union",
        "boolean/corner-union",
        "boolean/corner-common",
        "boolean/corner-cut",
        "boolean/flush-common",
        "boolean/disjoint-cut",
        "boolean/frame-cut",
        "boolean/boss-flush",
        "boolean/coaxial-fuse",
        "sweep/extrude-plate-with-hole",
        "sweep/extrude-slot",
        "sweep/extrude-downward",
        "sweep/revolve-tube",
        "sweep/revolve-quarter",
        "sweep/revolve-l-profile",
        "sweep/revolve-frustum",
        "sweep/revolve-barrel",
        "sweep/revolve-ring",
        "provenance/bolt-pattern-rebuild",
    ] {
        assert!(
            names.iter().any(|n| n == expected),
            "missing fixture {expected}; have {names:?}"
        );
    }
}

/// A scratch copy of a fixture with one deliberately wrong closed form
/// fails the lint, and a stale recipe hash does too.
#[test]
fn a_wrong_analytic_value_fails_the_lint() {
    let scratch = tempdir("wrong-analytic");
    copy_fixture("primitive/box", &scratch);
    let path = scratch.join("fixture.json");
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains("\"volume\": 12000"));
    std::fs::write(
        &path,
        text.replace("\"volume\": 12000", "\"volume\": 12001"),
    )
    .unwrap();
    let problems = lint(&scratch);
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(
        problems[0].contains("analytic volume 12001 vs oracle 12000"),
        "{problems:?}"
    );

    let scratch = tempdir("stale-hash");
    copy_fixture("primitive/box", &scratch);
    let path = scratch.join("fixture.json");
    let text = std::fs::read_to_string(&path).unwrap();
    std::fs::write(
        &path,
        text.replace("\"max\": [40, 30, 10]", "\"max\": [40, 30, 11]"),
    )
    .unwrap();
    let problems = lint(&scratch);
    assert!(problems.iter().any(|p| p.contains("stale")), "{problems:?}");

    let scratch = tempdir("wrong-genus");
    copy_fixture("boolean/through-hole", &scratch);
    let path = scratch.join("fixture.json");
    let text = std::fs::read_to_string(&path).unwrap();
    std::fs::write(&path, text.replace("\"genus\": 1", "\"genus\": 0")).unwrap();
    let problems = lint(&scratch);
    assert!(
        problems.iter().any(|p| p.contains("Euler line is -2")),
        "{problems:?}"
    );

    assert!(lint(Path::new("/nonexistent/fixture")).len() == 1);
}

/// `analytic.inertia` is a closed form like the others, and
/// `measure_differs` (ADR-0015) needs every closed form and a
/// measurement that actually differs from the oracle's: a scratch copy of
/// the crossing-cylinder fuse, at a turn where Open CASCADE is right,
/// claims it with one form missing, then with all of them agreeing.
#[test]
fn measure_differs_needs_every_closed_form_and_a_difference() {
    let edit = |tag: &str, f: &dyn Fn(&mut serde_json::Value)| {
        let scratch = tempdir(tag);
        copy_fixture("boolean/cross-cylinders-fuse", &scratch);
        let path = scratch.join("fixture.json");
        let mut recipe: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        f(&mut recipe["analytic"]);
        std::fs::write(&path, recipe.to_string()).unwrap();
        lint(&scratch)
    };
    let transverse = "pi * R^2 * L * (3 * R^2 + L^2) / 12";
    let inertia = serde_json::json!([
        [
            format!("{transverse} + pi * R^4 * L / 2 - 112 * R^5 / 45"),
            0,
            0
        ],
        [0, format!("2 * {transverse} - 128 * R^5 / 45"), 0],
        [
            0,
            0,
            format!("{transverse} + pi * R^4 * L / 2 - 112 * R^5 / 45")
        ],
    ]);

    // The Steinmetz union's inertia is the oracle's; one component off is
    // a problem like any closed form.
    assert!(
        edit("inertia", &|a| a["inertia"] = inertia.clone()).is_empty(),
        "the closed-form inertia is the oracle's"
    );
    let problems = edit("wrong-inertia", &|a| {
        a["inertia"] = inertia.clone();
        a["inertia"][1][1] = serde_json::json!(119.7);
    });
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(
        problems[0].contains("analytic inertia[1][1] 119.7 vs oracle"),
        "{problems:?}"
    );

    let problems = edit("measure-differs-missing", &|a| {
        a["measure_differs"] = "a claim".into();
    });
    assert!(
        problems
            .iter()
            .any(|p| p.contains("analytic.measure_differs needs the closed forms it is held to: analytic.inertia missing")),
        "{problems:?}"
    );

    let problems = edit("measure-differs-agrees", &|a| {
        a["measure_differs"] = "a claim".into();
        a["inertia"] = inertia.clone();
    });
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(
        problems[0].contains("[default] analytic.measure_differs is set but every closed form is the oracle's measurement"),
        "{problems:?}"
    );

    // Claimed and borne out: the volume differs, and is not reported as
    // a problem.
    let problems = edit("measure-differs-differs", &|a| {
        a["measure_differs"] = "a claim".into();
        a["inertia"] = inertia.clone();
        a["volume"] = "2 * pi * R^2 * L".into();
    });
    assert!(problems.is_empty(), "{problems:?}");
}

/// A second oracle's record (ADR-0073) stands in for the closed forms
/// under `measure_differs`, and is held to the same claim: a scratch copy
/// of the crossing-cylinder fuse, its closed forms of the measurements
/// dropped, carries a record that is stale, unclaimed, beside closed
/// forms, or the oracle's own numbers, each a problem, and one whose
/// volume differs, which is clean.
#[test]
fn a_second_oracle_record_stands_in_for_the_closed_forms() {
    let edit = |tag: &str, f: &dyn Fn(&mut serde_json::Value, &mut serde_json::Value)| {
        let scratch = tempdir(tag);
        copy_fixture("boolean/cross-cylinders-fuse", &scratch);
        let path = scratch.join("fixture.json");
        let mut recipe: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
        let analytic = recipe["analytic"].as_object_mut().unwrap();
        for key in ["volume", "area", "centroid", "inertia"] {
            analytic.remove(key);
        }
        let expected: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(scratch.join("expected.json")).unwrap())
                .unwrap();
        let m = &expected["results"]["default"];
        let mut record = serde_json::json!({
            "recipe_sha256": expected["recipe_sha256"],
            "results": {"default": {
                "source": "1.0", "run": "abc1234", "accuracy": 1.0, "scale": 1.0,
                "volume": m["volume"], "area": m["area"],
                "centroid": m["centroid"], "inertia": m["inertia"],
            }},
        });
        f(&mut recipe["analytic"], &mut record);
        std::fs::write(&path, recipe.to_string()).unwrap();
        std::fs::write(scratch.join("second_oracle.json"), record.to_string()).unwrap();
        lint(&scratch)
    };
    let claim = |a: &mut serde_json::Value| a["measure_differs"] = "a claim".into();
    let off = |r: &mut serde_json::Value| {
        let v = r["results"]["default"]["volume"].as_f64().unwrap();
        r["results"]["default"]["volume"] = (v * 1.001).into();
    };
    let only = |problems: Vec<String>, text: &str| {
        assert_eq!(problems.len(), 1, "{problems:?}");
        assert!(problems[0].contains(text), "{problems:?}");
    };

    // Claimed and borne out: clean.
    let problems = edit("second-differs", &|a, r| {
        claim(a);
        off(r);
    });
    assert!(problems.is_empty(), "{problems:?}");
    only(
        edit("second-agrees", &|a, _| claim(a)),
        "[default] analytic.measure_differs is set but the second oracle's record is the oracle's measurement",
    );
    only(
        edit("second-unclaimed", &|_, r| off(r)),
        "second_oracle.json holds nothing without analytic.measure_differs",
    );
    only(
        edit("second-stale", &|a, r| {
            claim(a);
            off(r);
            r["recipe_sha256"] = "0".into();
        }),
        "second_oracle.json is stale",
    );
    only(
        edit("second-beside-closed-forms", &|a, r| {
            claim(a);
            off(r);
            a["volume"] = "2 * pi * R^2 * L".into();
        }),
        "so analytic.volume would be a second source",
    );
    only(
        edit("second-no-variant", &|a, r| {
            claim(a);
            r["results"] = serde_json::json!({});
        }),
        "[default] second_oracle.json has no result",
    );
}

/// `step_differs` (ADR-0023) excuses only a result Arris builds, whose
/// STEP the corpus reads back: on a recipe that expects a refusal, there
/// is no such STEP, so the claim is a lint problem.
#[test]
fn step_differs_needs_a_result_arris_builds() {
    let scratch = tempdir("step-differs-refused");
    copy_fixture("boolean/ball-in-bore-cut", &scratch);
    let path = scratch.join("fixture.json");
    let mut fixture: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    fixture["analytic"]["step_differs"] = "a claim".into();
    std::fs::write(&path, serde_json::to_string_pretty(&fixture).unwrap()).unwrap();
    let problems = lint(&scratch);
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(
        problems[0].contains("analytic.step_differs needs a result Arris builds"),
        "{problems:?}"
    );
}

/// A fillet or chamfer Open CASCADE left an asked edge out of builds a body
/// missing a blend (ADR-0070): the oracle records the point as
/// `dropped_edges`, and a recipe that compares Arris against it is a lint
/// problem. One that expects Arris's refusal is not.
#[test]
fn a_dropped_edge_needs_a_recipe_that_expects_a_refusal() {
    let scratch = tempdir("dropped-edge");
    copy_fixture("blend/box-edge-fillet", &scratch);
    let path = scratch.join("expected.json");
    let mut expected: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    expected["results"]["default"]["dropped_edges"] = serde_json::json!([[2.0, 2.0, 1.0]]);
    std::fs::write(&path, serde_json::to_string_pretty(&expected).unwrap()).unwrap();
    let problems = lint(&scratch);
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(problems[0].contains("dropped 1 asked edge"), "{problems:?}");
    assert!(
        lint(
            &arris_debug::fixtures::corpus_root().join("blend/tangent-edge-beside-a-crease-fillet")
        )
        .is_empty()
    );
    let _ = std::fs::remove_dir_all(&scratch);
}

/// A comparable solid fixture in one of the corpus's areas without its
/// committed dump fails the lint, one problem per variant missing one;
/// a result the oracle built no solid for, a recipe expecting a refusal,
/// and a copy outside the areas need none.
#[test]
fn a_comparable_fixture_without_its_dump_fails_the_lint() {
    let root = tempdir("missing-dump");
    let scratch = root.join("sweep/revolve-tube");
    std::fs::create_dir_all(&scratch).unwrap();
    copy_fixture("sweep/revolve-tube", &scratch);
    let problems = lint(&scratch);
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(
        problems[0].contains("[default] dump.txt is not committed"),
        "{problems:?}"
    );
    std::fs::copy(
        corpus_root().join("sweep/revolve-tube/dump.txt"),
        scratch.join("dump.txt"),
    )
    .unwrap();
    assert!(lint(&scratch).is_empty(), "{:?}", lint(&scratch));

    let scratch = root.join("provenance/bolt-pattern-rebuild");
    std::fs::create_dir_all(&scratch).unwrap();
    copy_fixture("provenance/bolt-pattern-rebuild", &scratch);
    std::fs::copy(
        corpus_root().join("provenance/bolt-pattern-rebuild/dump.txt"),
        scratch.join("dump.txt"),
    )
    .unwrap();
    let problems = lint(&scratch);
    assert_eq!(problems.len(), 2, "{problems:?}");
    for variant in ["thicker-wider", "tighter"] {
        let file = format!("[{variant}] dump.{variant}.txt is not committed");
        assert!(problems.iter().any(|p| p.contains(&file)), "{problems:?}");
    }

    for name in ["boolean/flush-common", "boolean/tangent-hole"] {
        let scratch = root.join(name);
        std::fs::create_dir_all(&scratch).unwrap();
        copy_fixture(name, &scratch);
        assert!(lint(&scratch).is_empty(), "{name}: {:?}", lint(&scratch));
    }

    // A failure waiting under `regression/` needs no dump, and one that
    // has a dump passes and has to move.
    let scratch = root.join("regression/revolve-tube");
    std::fs::create_dir_all(&scratch).unwrap();
    copy_fixture("sweep/revolve-tube", &scratch);
    assert!(lint(&scratch).is_empty(), "{:?}", lint(&scratch));
    std::fs::copy(
        corpus_root().join("sweep/revolve-tube/dump.txt"),
        scratch.join("dump.txt"),
    )
    .unwrap();
    let problems = lint(&scratch);
    assert_eq!(problems.len(), 1, "{problems:?}");
    assert!(
        problems[0].contains("dump.txt is committed under regression/"),
        "{problems:?}"
    );

    let outside = tempdir("outside-the-areas");
    copy_fixture("sweep/revolve-tube", &outside);
    assert!(lint(&outside).is_empty(), "{:?}", lint(&outside));
}

/// The geometry kind is linted for presence, hash and shape: a stale
/// recipe and a result count that does not match the recipe both fail.
#[test]
fn a_geometry_fixture_is_linted_for_presence_hash_and_shape() {
    let geometry: Vec<String> = corpus()
        .iter()
        .filter(|d| kind_of(d).unwrap() == Kind::Geometry)
        .map(|d| arris_debug::fixtures::name_of(d))
        .collect();
    for expected in [
        "geom/analytic-eval",
        "geom/c1-intersections",
        "geom/c2-cylinder-pairs",
        "geom/c2-quadric-pairs",
    ] {
        assert!(geometry.iter().any(|n| n == expected), "missing {expected}");
    }

    let scratch = tempdir("geom-stale");
    copy_fixture("geom/c1-intersections", &scratch);
    let path = scratch.join("fixture.json");
    let text = std::fs::read_to_string(&path).unwrap();
    assert!(text.contains("\"radius\": 2.0"));
    std::fs::write(
        &path,
        text.replacen("\"radius\": 2.0", "\"radius\": 2.5", 1),
    )
    .unwrap();
    let problems = lint(&scratch);
    assert!(problems.iter().any(|p| p.contains("stale")), "{problems:?}");

    let scratch = tempdir("geom-shape");
    copy_fixture("geom/c1-intersections", &scratch);
    let path = scratch.join("expected.json");
    let mut expected: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap();
    expected["pairs"].as_array_mut().unwrap().pop();
    std::fs::write(&path, expected.to_string()).unwrap();
    let problems = lint(&scratch);
    assert!(
        problems.iter().any(|p| p.contains("pairs in the recipe")),
        "{problems:?}"
    );
}

/// The parts of the real-part corpus — the committed tier's fixtures and
/// the fetched tier's `tools/real-parts.waits` — wait on exactly the part
/// fixtures open under `regression/`: every exclusion names one, and
/// every one is named (ADR-0026 §4). A fix moves its fixture out of `regression/`, and
/// the part's lint then fails until the exclusion is lifted.
#[test]
fn every_part_exclusion_is_an_open_regression_part() {
    let mut waited = std::collections::BTreeSet::new();
    let mut open = std::collections::BTreeSet::new();
    for dir in corpus() {
        if kind_of(&dir).unwrap() != Kind::Part {
            continue;
        }
        let part = arris_debug::part::load(&dir).unwrap();
        if part.name.starts_with("regression/") {
            open.insert(part.name.clone());
        } else {
            waited.extend(part.part.waits_on.iter().cloned());
        }
    }
    // The fetched tier's parts are not in the corpus: they wait by line.
    let waits = std::fs::read_to_string(workspace_root().join("tools/real-parts.waits")).unwrap();
    for line in waits.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if let Some((_, slug)) = line.split_once(char::is_whitespace) {
            waited.insert(slug.trim().to_string());
        }
    }
    assert_eq!(
        waited, open,
        "the parts' exclusions, and the part fixtures open under regression/"
    );
}

/// A part fixture (ADR-0026) is linted for its file's hash, the dump of
/// every solid it reads and a reason for every refusal it records.
#[test]
fn a_part_is_linted_for_its_file_its_dumps_and_its_reasons() {
    let scratch = tempdir("part");
    let from = corpus_root().join("real/nist-ftc-09");
    let files = [
        "fixture.json",
        "expected.json",
        "nist_ftc_09_asme1_rd.stp",
        "dump.5384.0.txt",
    ];
    for file in files {
        std::fs::copy(from.join(file), scratch.join(file)).unwrap();
    }
    assert!(lint(&scratch).is_empty(), "{:?}", lint(&scratch));

    let dump = scratch.join("dump.5384.0.txt");
    std::fs::remove_file(&dump).unwrap();
    let problems = lint(&scratch);
    assert!(
        problems
            .iter()
            .any(|p| p.contains("reads but dump.5384.0.txt is not committed")),
        "{problems:?}"
    );
    std::fs::copy(from.join("dump.5384.0.txt"), &dump).unwrap();

    let step = scratch.join("nist_ftc_09_asme1_rd.stp");
    let mut bytes = std::fs::read(&step).unwrap();
    bytes.push(b'\n');
    std::fs::write(&step, &bytes).unwrap();
    let problems = lint(&scratch);
    assert!(
        problems.iter().any(|p| p.contains("not the fixture's")),
        "{problems:?}"
    );
    std::fs::copy(from.join("nist_ftc_09_asme1_rd.stp"), &step).unwrap();

    let recipe = scratch.join("fixture.json");
    let text = std::fs::read_to_string(&recipe).unwrap();
    let reasonless = text.replace(
        r#""outcome": "read""#,
        r#""outcome": {"refused": {"kind": "offset", "why": " "}}"#,
    );
    assert_ne!(reasonless, text);
    std::fs::write(&recipe, reasonless).unwrap();
    let problems = lint(&scratch);
    assert!(
        problems
            .iter()
            .any(|p| p.contains("with no reason why the refusal is right")),
        "{problems:?}"
    );
    let _ = std::fs::remove_dir_all(&scratch);
}

fn tempdir(tag: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("arris-corpus-lint-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

fn copy_fixture(name: &str, to: &Path) {
    let from = arris_debug::fixtures::corpus_root().join(name);
    for file in ["fixture.json", "expected.json"] {
        std::fs::copy(from.join(file), to.join(file)).unwrap();
    }
}
