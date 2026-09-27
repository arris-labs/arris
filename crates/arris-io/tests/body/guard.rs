//! The guard (ADR-0029): one blessed `.bin` and `.json` per body under
//! `tests/body/v<N>/`, for every version a release wrote, each read in
//! the suite forever to the committed `.dump.txt`. A change to the model's
//! types that alters how an old file decodes fails here instead of
//! shipping. `ARRIS_BLESS=1` writes the files of [`BODY_VERSION`]'s set
//! that are missing and never overwrites one that exists: a version's
//! bytes are what that release wrote, frozen.

use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};

use arris_debug::{corpus, dump_text, fixtures, sample};
use arris_io::arris_check::arris_topo::arris_geom::{Curve2Kind, CurveKind, SurfaceKind};
use arris_io::arris_check::arris_topo::arris_math::Point3;
use arris_io::arris_check::arris_topo::{Body, Model, Origin, Provenance, Role};
use arris_io::arris_check::{Level, check};
use arris_io::body::{self, BODY_VERSION, Imported};

/// A body of the current version's set and the record written with it.
struct Written {
    model: Model,
    body: Body,
    record: Provenance,
}

/// The current version's set, by name: together they hold every curve,
/// surface and pcurve kind ([`coverage`]), a consumer-keyed record and
/// a boolean's record with foreign origins.
const SET: [&str; 6] = [
    "sections",
    "ring-pin",
    "elliptic",
    "nurbs",
    "tetrahedron",
    "frame-cut",
];

/// A corpus fixture's result and its record, built at `default`.
fn recipe(area_slug: &str) -> Written {
    let dir = fixtures::corpus_root().join(area_slug);
    let chain = corpus::chain(&dir, "default").unwrap();
    let body = chain.result().unwrap();
    let record = chain.steps[&chain.result].provenance.clone();
    Written {
        model: chain.model,
        body,
        record,
    }
}

/// The body `name` stands for, as it is built today.
fn build(name: &str) -> Written {
    match name {
        // A revolve fused with an extrusion and cut: every curve and
        // pcurve kind, the cone, sphere and torus, sections both traced
        // and fitted to NURBS.
        "sections" => recipe("boolean/revolve-extrude-fuse-cut-rounding-knots"),
        // A torus cut by a cylinder: the torus walker's traced section.
        "ring-pin" => recipe("boolean/ring-pin-cut"),
        "elliptic" => recipe("boolean/elliptic-operand-cut"),
        "nurbs" => {
            let mut model = Model::default();
            let body =
                sample::cuboid_nurbs(&mut model, Point3::origin(), Point3::new(2.0, 3.0, 4.0))
                    .unwrap();
            Written {
                model,
                body,
                record: Provenance::default(),
            }
        }
        // `ops::build` under a consumer's namespace: every origin a role
        // keyed by the consumer.
        "tetrahedron" => recipe("build/tetrahedron"),
        // A cut whose record names the operands' faces: foreign origins.
        "frame-cut" => recipe("provenance/split-frame-cut"),
        _ => panic!("{name} is not in the set"),
    }
}

/// `tests/body/`, holding one `v<N>/` per version.
fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/body")
}

fn version_dir(version: u32) -> PathBuf {
    root().join(format!("v{version}"))
}

/// Every version directory, ascending by version.
fn versions() -> Vec<(u32, PathBuf)> {
    let mut out: Vec<(u32, PathBuf)> = fs::read_dir(root())
        .unwrap()
        .filter_map(|e| {
            let path = e.unwrap().path();
            let v = path
                .file_name()?
                .to_str()?
                .strip_prefix('v')?
                .parse()
                .ok()?;
            path.is_dir().then_some((v, path))
        })
        .collect();
    out.sort();
    out
}

/// The names of the bodies blessed in `dir`, sorted.
fn names(dir: &Path) -> Vec<String> {
    let mut out: Vec<String> = fs::read_dir(dir)
        .unwrap()
        .filter_map(|e| {
            let path = e.unwrap().path();
            (path.extension()? == "bin").then(|| path.file_stem()?.to_str().map(String::from))?
        })
        .collect();
    out.sort();
    out
}

/// What the dump file holds: the body read into a default model, then
/// the record as written, then the foreign entities.
fn dump(m: &Model, read: &Imported) -> String {
    let foreign: Vec<String> = read.foreign().iter().map(|s| s.to_string()).collect();
    format!(
        "{}provenance\n{}\nforeign [{}]\n",
        dump_text(m, read.body).unwrap(),
        read.provenance,
        foreign.join(" ")
    )
}

/// `name` read from `dir` through both encodings into default models,
/// having asserted that both read, checker-green, at `version`, to the
/// same dump.
fn read_both(dir: &Path, name: &str, version: u32) -> (Model, Imported) {
    let bytes = fs::read(dir.join(format!("{name}.bin"))).unwrap();
    let text = fs::read_to_string(dir.join(format!("{name}.json"))).unwrap();
    let mut a = Model::default();
    let from_bytes =
        body::read(&mut a, &bytes).unwrap_or_else(|e| panic!("v{version}/{name}: {e}"));
    let mut b = Model::default();
    let from_text =
        body::from_json(&mut b, &text).unwrap_or_else(|e| panic!("v{version}/{name}.json: {e}"));
    for (m, read) in [(&a, &from_bytes), (&b, &from_text)] {
        assert_eq!(read.version, version, "{name}");
        let report = check(m, read.body, Level::Full);
        assert!(report.is_ok(), "v{version}/{name}: {report}");
    }
    assert_eq!(dump(&a, &from_bytes), dump(&b, &from_text), "{name}");
    (a, from_bytes)
}

/// Writes whatever of the current version's set is missing: the bytes
/// and JSON from the body as built today when there are none, the dump
/// from the committed bytes when only it is missing. Never overwrites.
fn bless() {
    let dir = version_dir(BODY_VERSION);
    fs::create_dir_all(&dir).unwrap();
    for name in SET {
        let bin = dir.join(format!("{name}.bin"));
        let json = dir.join(format!("{name}.json"));
        if !bin.exists() && !json.exists() {
            let w = build(name);
            fs::write(&bin, body::write(&w.model, w.body, &w.record).unwrap()).unwrap();
            fs::write(&json, body::to_json(&w.model, w.body, &w.record).unwrap()).unwrap();
        }
        let dump_file = dir.join(format!("{name}.dump.txt"));
        if !dump_file.exists() {
            let (m, read) = read_both(&dir, name, BODY_VERSION);
            fs::write(&dump_file, dump(&m, &read)).unwrap();
        }
    }
}

#[test]
fn every_blessed_body_of_every_version_reads_to_its_committed_dump() {
    if corpus::blessing() {
        bless();
    }
    let current = names(&version_dir(BODY_VERSION));
    for name in SET {
        assert!(
            current.iter().any(|n| n == name),
            "v{BODY_VERSION}/{name} is not blessed (ARRIS_BLESS=1)"
        );
    }
    let all = versions();
    assert!(all.iter().any(|&(v, _)| v == BODY_VERSION));
    for (version, dir) in all {
        assert!(version <= BODY_VERSION, "{}", dir.display());
        for name in names(&dir) {
            let (m, read) = read_both(&dir, &name, version);
            let committed = fs::read_to_string(dir.join(format!("{name}.dump.txt")))
                .unwrap_or_else(|e| panic!("v{version}/{name}.dump.txt: {e}"));
            assert_eq!(dump(&m, &read), committed, "v{version}/{name}");
        }
    }
}

#[test]
fn the_set_carries_a_consumers_keys_and_a_booleans_foreign_origins() {
    let dir = version_dir(BODY_VERSION);
    let (_, tetrahedron) = read_both(&dir, "tetrahedron", BODY_VERSION);
    let origins: Vec<Origin> = tetrahedron.provenance.origins_recorded().collect();
    assert!(!origins.is_empty());
    assert!(
        origins
            .iter()
            .all(|o| matches!(o, Origin::Role(Role::Consumer(_)))),
        "{origins:?}"
    );
    let (_, cut) = read_both(&dir, "frame-cut", BODY_VERSION);
    assert!(!cut.foreign().is_empty(), "a cut names its operands");
}

/// The guard body that holds each kind. Exhaustive on purpose: a new
/// kind fails to compile here until the guard has a body with it.
fn curve_witness(k: CurveKind) -> &'static str {
    match k {
        CurveKind::Line | CurveKind::Circle | CurveKind::Ellipse | CurveKind::Nurbs => "sections",
    }
}

fn surface_witness(k: SurfaceKind) -> &'static str {
    match k {
        SurfaceKind::Plane
        | SurfaceKind::Cylinder
        | SurfaceKind::Cone
        | SurfaceKind::Sphere
        | SurfaceKind::Torus => "sections",
        SurfaceKind::EllipticCylinder => "elliptic",
        SurfaceKind::Nurbs => "nurbs",
    }
}

fn curve2_witness(k: Curve2Kind) -> &'static str {
    match k {
        Curve2Kind::Line | Curve2Kind::Circle | Curve2Kind::Ellipse | Curve2Kind::Nurbs => {
            "sections"
        }
    }
}

/// The kinds a body read from the current version's guard holds.
fn kinds(
    name: &str,
) -> (
    BTreeSet<CurveKind>,
    BTreeSet<SurfaceKind>,
    BTreeSet<Curve2Kind>,
) {
    let (m, read) = read_both(&version_dir(BODY_VERSION), name, BODY_VERSION);
    let c = m.closure(read.body).unwrap();
    (
        c.curves
            .iter()
            .map(|&i| m.curve(i).unwrap().kind())
            .collect(),
        c.surfaces
            .iter()
            .map(|&i| m.surface(i).unwrap().kind())
            .collect(),
        c.curve2s
            .iter()
            .map(|&i| m.curve2(i).unwrap().kind())
            .collect(),
    )
}

#[test]
fn coverage() {
    // Every kind, listed beside the witnesses above; a kind added to an
    // enum is added to both.
    for k in [
        CurveKind::Line,
        CurveKind::Circle,
        CurveKind::Ellipse,
        CurveKind::Nurbs,
    ] {
        assert!(kinds(curve_witness(k)).0.contains(&k), "no {k} curve");
    }
    for k in [
        SurfaceKind::Plane,
        SurfaceKind::Cylinder,
        SurfaceKind::EllipticCylinder,
        SurfaceKind::Cone,
        SurfaceKind::Sphere,
        SurfaceKind::Torus,
        SurfaceKind::Nurbs,
    ] {
        assert!(kinds(surface_witness(k)).1.contains(&k), "no {k} surface");
    }
    for k in [
        Curve2Kind::Line,
        Curve2Kind::Circle,
        Curve2Kind::Ellipse,
        Curve2Kind::Nurbs,
    ] {
        assert!(kinds(curve2_witness(k)).2.contains(&k), "no {k} pcurve");
    }
}
