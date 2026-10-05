//! The STEP writer against the Open CASCADE oracle (`docs/ARCHITECTURE.md`
//! §Formats and tools): the sample bodies, written by Arris and read by
//! the oracle, match the `primitive/*` fixtures' numbers; the file is
//! deterministic; what the writer cannot hold is a typed error.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

use arris_check::{Level, LumpError, check};
use arris_debug::fixtures::{self, Num, Recipe, Step};
use arris_debug::{oracle, sample};
use arris_io::step::{self, StepError, Unsupported};
use arris_math::Point3;
use arris_topo::builder::{Assembly, Builder, FaceSpec};
use arris_topo::entity::{Body as BodyEntity, BodyKind};
use arris_topo::{Body, Model};

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../..")
        .canonicalize()
        .expect("the workspace root exists")
}

fn scratch(tag: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("arris-io-step-{tag}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Writes `body` to a file and runs `compare.py` on it against `fixture`.
/// A missing oracle environment is a loud failure with the command that
/// creates it, never a skip (`tools/oracle/README.md`).
fn compare(m: &Model, body: Body, fixture: &str, tag: &str) {
    let text = step::write(m, &[body]).unwrap();
    let file = scratch(tag).join(format!("{tag}.step"));
    std::fs::write(&file, &text).unwrap();
    let root = repo_root();
    let output = Command::new("uv")
        .current_dir(&root)
        .args([
            "run",
            "--project",
            "tools/oracle",
            "tools/oracle/compare.py",
        ])
        .arg(root.join("tests/fixtures").join(fixture))
        .arg(&file)
        .output()
        .unwrap_or_else(|e| {
            panic!("could not run `uv` ({e}); install uv and run `uv sync --project tools/oracle`")
        });
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        output.status.success(),
        "compare.py {fixture} on {}:\n{stdout}\n{stderr}",
        file.display()
    );
    assert!(stdout.contains("MATCH"), "{stdout}");
}

#[test]
fn the_sample_box_matches_the_primitive_box_fixture() {
    let mut m = Model::default();
    let body = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    compare(&m, body, "primitive/box", "box");
}

#[test]
fn the_sample_cylinder_matches_the_primitive_cylinder_fixture() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    compare(&m, body, "primitive/cylinder", "cylinder");
}

/// The B-spline arms: the box with one NURBS edge and one NURBS face reads
/// back with the same numbers as the analytic one.
#[test]
fn the_nurbs_box_matches_the_primitive_box_fixture() {
    let mut m = Model::default();
    let body =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let text = step::write(&m, &[body]).unwrap();
    assert_eq!(text.matches("B_SPLINE_CURVE_WITH_KNOTS(").count(), 1);
    assert_eq!(text.matches("B_SPLINE_SURFACE_WITH_KNOTS(").count(), 1);
    compare(&m, body, "primitive/box", "nurbs-box");
}

#[test]
fn two_writes_and_two_builds_are_byte_identical() {
    let build = || {
        let mut m = Model::default();
        let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
        (m, body)
    };
    let (a, body_a) = build();
    let (b, body_b) = build();
    let first = step::write(&a, &[body_a]).unwrap();
    assert_eq!(first, step::write(&a, &[body_a]).unwrap());
    assert_eq!(first, step::write(&b, &[body_b]).unwrap());
    assert!(!first.contains("NaN") && !first.contains("inf"));
}

#[test]
fn the_seam_is_one_edge_with_two_pcurves_and_the_wall_one_face() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = step::write(&m, &[body]).unwrap();
    assert_eq!(text.matches("EDGE_CURVE(").count(), 3);
    assert_eq!(text.matches("SEAM_CURVE(").count(), 1);
    assert_eq!(text.matches("SURFACE_CURVE(").count(), 2);
    assert_eq!(
        text.matches("ORIENTED_EDGE(").count(),
        6,
        "the seam twice in the wall's loop"
    );
    assert_eq!(text.matches("CYLINDRICAL_SURFACE(").count(), 1);
    assert_eq!(text.matches("FACE_OUTER_BOUND(").count(), 3);
    assert_eq!(text.matches("VERTEX_POINT(").count(), 2);
    assert_eq!(
        text.matches("ADVANCED_FACE('',(#").count(),
        3,
        "every face has a bound list"
    );
    assert!(text.contains(",.F.);\n"), "the bottom cap is used reversed");
}

/// An extruded ellipse's side is written as the section ellipse
/// extruded along its axis, once, with the seam's two pcurves on it
/// (ADR-0014); the oracle reads it back in the corpus
/// (`sweep/extrude-ellipse`).
#[test]
fn an_elliptic_cylinder_is_a_surface_of_linear_extrusion() {
    use arris_geom::{Profile, ProfileLoop};
    use arris_math::{Frame, Point2, Vec2, Vec3};
    let mut m = Model::default();
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Ellipse {
            center: Point2::new(1.0, 2.0),
            major: Vec2::new(4.8, 3.6),
            minor_radius: 3.0,
        },
        holes: Vec::new(),
    };
    let (body, _) = arris_debug::unmetered::extrude(&mut m, &profile, Vec3::z(), 10.0).unwrap();
    let text = step::write(&m, &[body]).unwrap();
    assert_eq!(text.matches("SURFACE_OF_LINEAR_EXTRUSION(").count(), 1);
    assert!(
        text.contains("ELLIPSE('',#"),
        "the section ellipse is written"
    );
    assert_eq!(text.matches("SEAM_CURVE(").count(), 1, "the seam, once");
    assert_eq!(text.matches("ADVANCED_FACE(").count(), 3);
}

#[test]
fn several_bodies_share_one_shape_representation() {
    let mut m = Model::default();
    let a = sample::cuboid(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let b = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = step::write(&m, &[a, b]).unwrap();
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 2);
    assert_eq!(
        text.matches("ADVANCED_BREP_SHAPE_REPRESENTATION(").count(),
        1
    );
    assert_eq!(text.matches("PRODUCT(").count(), 1);
}

/// A recipe of two boxes combined by `combine` into `result`: the oracle
/// builds from it the shape a hand-assembled body of lumps stands for.
fn boxes_recipe(
    description: &str,
    a: ([f64; 3], [f64; 3]),
    b: ([f64; 3], [f64; 3]),
    combine: Step,
) -> Recipe {
    let n = |v: [f64; 3]| v.map(Num::Literal);
    Recipe {
        description: description.to_string(),
        params: BTreeMap::new(),
        variants: BTreeMap::new(),
        steps: vec![
            Step::Box {
                name: "a".into(),
                min: n(a.0),
                max: n(a.1),
            },
            Step::Box {
                name: "b".into(),
                min: n(b.0),
                max: n(b.1),
            },
            combine,
        ],
        result: "result".into(),
        probes: Vec::new(),
        precision: Default::default(),
        tolerances: Default::default(),
        analytic: Default::default(),
    }
}

/// A solid over the faces of each body in order, one shell per body, the
/// faces of a body marked `true` turned into it — a void.
fn solid_of_shells(m: &mut Model, shells: &[(Body, bool)]) -> Body {
    let shells = shells
        .iter()
        .map(|&(body, turned)| {
            m.faces(body)
                .unwrap()
                .into_iter()
                .map(|f| {
                    FaceSpec::Keep(if turned {
                        arris_topo::Face::new(f.id, f.orientation.flipped())
                    } else {
                        f
                    })
                })
                .collect()
        })
        .collect();
    let assembly = Assembly {
        shells,
        ..Assembly::default()
    };
    Builder::assemble(m, m.precision().default_tolerance, assembly)
        .unwrap()
        .0
        .finish(m, BodyKind::Solid)
        .unwrap()
        .body
}

/// A hollow box is one lump with a void and two boxes apart are two lumps:
/// each is written as that many solid entities, and the oracle reads each
/// file back with the counts and the volume of the closed form — one solid
/// of two shells enclosing 10³ − 4³, two solids of one shell each
/// enclosing 2 · 10³.
#[test]
fn lumps_and_voids_are_read_back_as_solids_and_shells() {
    let mut m = Model::default();
    let outer = sample::cuboid(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0)).unwrap();
    let inner = sample::cuboid(
        &mut m,
        Point3::new(3.0, 3.0, 3.0),
        Point3::new(7.0, 7.0, 7.0),
    )
    .unwrap();
    let hollow = solid_of_shells(&mut m, &[(outer, false), (inner, true)]);
    let report = check(&m, hollow, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let text = step::write(&m, &[hollow]).unwrap();
    assert_eq!(text.matches("BREP_WITH_VOIDS(").count(), 1);
    assert_eq!(text.matches("ORIENTED_CLOSED_SHELL(").count(), 1);
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 0);
    let recipe = boxes_recipe(
        "a box of 10 less a box of 4 in its middle: one solid of two shells",
        ([0.0; 3], [10.0; 3]),
        ([3.0; 3], [7.0; 3]),
        Step::Cut {
            name: "result".into(),
            target: "a".into(),
            tool: Some("b".into()),
            tools: Vec::new(),
        },
    );
    let dir = oracle::scratch_fixture("step-hollow-box", &recipe).unwrap();
    let expected = &fixtures::load(&dir).unwrap().expected.results["default"];
    assert_eq!((expected.counts.solids, expected.counts.shells), (1, 2));
    assert!((expected.volume.unwrap() - (1000.0 - 64.0)).abs() < 1e-9 * 1000.0);
    oracle::compare_dir(&dir, &text, None, "step-hollow-box").unwrap();

    let a = sample::cuboid(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0)).unwrap();
    let b = sample::cuboid(
        &mut m,
        Point3::new(20.0, 0.0, 0.0),
        Point3::new(30.0, 10.0, 10.0),
    )
    .unwrap();
    let apart = solid_of_shells(&mut m, &[(a, false), (b, false)]);
    let report = check(&m, apart, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let text = step::write(&m, &[apart]).unwrap();
    assert_eq!(text.matches("MANIFOLD_SOLID_BREP(").count(), 2);
    assert_eq!(text.matches("BREP_WITH_VOIDS(").count(), 0);
    let recipe = boxes_recipe(
        "two boxes of 10, 10 apart, fused: two solids",
        ([0.0; 3], [10.0; 3]),
        ([20.0, 0.0, 0.0], [30.0, 10.0, 10.0]),
        Step::Fuse {
            name: "result".into(),
            a: Some("a".into()),
            b: Some("b".into()),
            bodies: Vec::new(),
        },
    );
    let dir = oracle::scratch_fixture("step-two-boxes", &recipe).unwrap();
    let expected = &fixtures::load(&dir).unwrap().expected.results["default"];
    assert_eq!((expected.counts.solids, expected.counts.shells), (2, 2));
    assert!((expected.volume.unwrap() - 2000.0).abs() < 1e-9 * 2000.0);
    oracle::compare_dir(&dir, &text, None, "step-two-boxes").unwrap();
}

/// A solid whose shells do not nest has no lumps to write.
#[test]
fn a_solid_of_two_outer_shells_one_inside_the_other_is_a_typed_error() {
    let mut m = Model::default();
    let big = sample::cuboid(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0)).unwrap();
    let small = sample::cuboid(
        &mut m,
        Point3::new(3.0, 3.0, 3.0),
        Point3::new(7.0, 7.0, 7.0),
    )
    .unwrap();
    let nested = solid_of_shells(&mut m, &[(big, false), (small, false)]);
    assert!(matches!(
        step::write(&m, &[nested]),
        Err(StepError::Lumps {
            source: LumpError::Nesting { .. },
            ..
        })
    ));
}

#[test]
fn a_wire_body_and_an_empty_list_are_typed_errors() {
    let mut m = Model::default();
    let wire = m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        Vec::new(),
        Vec::new(),
        Vec::new(),
    ));
    let wire = Body::forward(wire);
    assert_eq!(
        step::write(&m, &[wire]),
        Err(StepError::Unsupported {
            body: wire,
            what: Unsupported::Kind(BodyKind::Wire)
        })
    );
    assert_eq!(step::write(&m, &[]), Err(StepError::NoBodies));
    let missing = Body::forward(arris_topo::BodyId::new(7, 0));
    assert!(matches!(
        step::write(&m, &[missing]),
        Err(StepError::NotFound(_))
    ));
}

/// A shell naming a face that does not resolve: a typed refusal, never a
/// panic and never output that silently leaves the face out. `lumps`
/// (`arris_check`) measures every shell's volume before the writer ever
/// reaches its own per-face lookups, and folds the dangling reference
/// into `LumpError::Unmeasurable` there — the writer's own `NotFound`
/// sites (the edge-use cross-reference, a NURBS control point) guard
/// paths `lumps` does not also cover, and are not reachable from a live
/// `&Model` through this one; this test stands in for them.
#[test]
fn a_face_removed_from_a_shell_is_a_typed_refusal_not_a_panic() {
    let mut m = Model::default();
    let body = sample::cuboid(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0)).unwrap();
    let mut faces = m.faces(body).unwrap();
    let ghost = arris_topo::FaceId::new(9999, 0);
    *faces.last_mut().unwrap() = arris_topo::Face::forward(ghost);
    let shell = m.raw().add_shell(arris_topo::entity::Shell::new(faces));
    let ghost_body = m.raw().add_body(BodyEntity::new(
        BodyKind::Solid,
        vec![arris_topo::Shell::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    let ghost_body = Body::forward(ghost_body);
    assert!(matches!(
        step::write(&m, &[ghost_body]),
        Err(StepError::Lumps {
            source: LumpError::Unmeasurable { .. },
            ..
        })
    ));
}
