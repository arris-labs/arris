//! Surface models read as sheet bodies (ADR-0057): shell- and face-based,
//! open and closed, oriented, with the refusals of the solid path. The file
//! is `tests/fixtures/real/face-based-surface-model/corner.stp` — the
//! 40 × 30 × 10 box Open CASCADE wrote, three faces of it at the origin
//! corner in a `FACE_BASED_SURFACE_MODEL` — edited into the other shapes a
//! surface model takes.

use arris_check::{Level, check};
use arris_debug::fixtures;
use arris_debug::unmetered::{step_read, surface_properties};
use arris_io::step::{ReadOptions, RefusalKind};
use arris_topo::entity::BodyKind;
use arris_topo::{Model, Orientation};

const FACE_BASED: &str = "#15 = FACE_BASED_SURFACE_MODEL('',(#16));";
const THREE_FACES: &str = "#16 = CONNECTED_FACE_SET('',(#17,#237,#331));";
const SIX_FACES: &str = "#17,#137,#237,#284,#331,#338";

fn corner() -> String {
    let path = fixtures::corpus_root().join("real/face-based-surface-model/corner.stp");
    let text = std::fs::read_to_string(path).unwrap();
    assert!(text.contains(FACE_BASED) && text.contains(THREE_FACES));
    text
}

fn read_one(text: &str) -> (Model, arris_io::step::ReadSolid) {
    let mut m = Model::default();
    let mut read = step_read(&mut m, text, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), 1);
    (m, read.solids.remove(0))
}

#[test]
fn a_face_based_model_is_a_sheet_of_its_faces() {
    let (m, solid) = read_one(&corner());
    let body = solid.result.unwrap();
    let body = body.body;
    assert_eq!(m.body(body.id).unwrap().kind(), BodyKind::Sheet);
    assert_eq!(m.faces(body).unwrap().len(), 3);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let p = surface_properties(&m, body).unwrap();
    assert!((p.area - 1900.0).abs() < 1e-9 * 1900.0, "{}", p.area);
}

#[test]
fn a_shell_based_model_of_a_closed_shell_is_a_closed_sheet() {
    let text = corner()
        .replace(FACE_BASED, "#15 = SHELL_BASED_SURFACE_MODEL('',(#16));")
        .replace(
            THREE_FACES,
            &format!("#16 = CLOSED_SHELL('',({SIX_FACES}));"),
        );
    let (m, solid) = read_one(&text);
    let body = solid.result.unwrap().body;
    assert_eq!(m.body(body.id).unwrap().kind(), BodyKind::Sheet);
    assert_eq!(m.faces(body).unwrap().len(), 6);
    let p = surface_properties(&m, body).unwrap();
    assert!((p.area - 3800.0).abs() < 1e-9 * 3800.0, "{}", p.area);
    assert!(check(&m, body, Level::Full).is_ok());
}

#[test]
fn an_oriented_open_shell_turns_the_front_of_its_faces() {
    let plain = corner().replace(FACE_BASED, "#15 = SHELL_BASED_SURFACE_MODEL('',(#16));");
    let open = plain.replace(THREE_FACES, "#16 = OPEN_SHELL('',(#17,#237,#331));");
    let turned = plain.replace(
        THREE_FACES,
        "#16 = ORIENTED_OPEN_SHELL('',*,#400,.F.);\n#400 = OPEN_SHELL('',(#17,#237,#331));",
    );
    let (ma, a) = read_one(&open);
    let (mb, b) = read_one(&turned);
    let (a, b) = (a.result.unwrap().body, b.result.unwrap().body);
    assert!(check(&mb, b, Level::Full).is_ok());
    let sides = |m: &Model, body| -> Vec<Orientation> {
        m.shells(body)
            .unwrap()
            .into_iter()
            .flat_map(|s| m.shell(s.id).unwrap().faces().to_vec())
            .map(|f| f.orientation)
            .collect()
    };
    let (sa, sb) = (sides(&ma, a), sides(&mb, b));
    assert_eq!(sa.len(), 3);
    for (x, y) in sa.iter().zip(&sb) {
        assert_ne!(x, y, "every face's front is turned");
    }
    let (pa, pb) = (
        surface_properties(&ma, a).unwrap(),
        surface_properties(&mb, b).unwrap(),
    );
    assert!((pa.area - pb.area).abs() < 1e-9 * pa.area);
}

#[test]
fn faces_that_disagree_on_orientation_are_a_topology_refusal() {
    // The face and its bound turned together: a consistent face whose
    // front disagrees with its neighbours'.
    let text = corner()
        .replace(
            "#237 = ADVANCED_FACE('',(#238),#44,.F.);",
            "#237 = ADVANCED_FACE('',(#238),#44,.T.);",
        )
        .replace(
            "#238 = FACE_BOUND('',#239,.F.);",
            "#238 = FACE_BOUND('',#239,.T.);",
        );
    let (_, solid) = read_one(&text);
    let refusal = solid.result.unwrap_err();
    assert_eq!(refusal.kind(), RefusalKind::Topology, "{refusal}");
}

#[test]
fn an_open_shell_is_no_solid() {
    let text = corner()
        .replace(FACE_BASED, "#15 = MANIFOLD_SOLID_BREP('',#16);")
        .replace(THREE_FACES, "#16 = OPEN_SHELL('',(#17,#237,#331));");
    let (_, solid) = read_one(&text);
    let refusal = solid.result.unwrap_err();
    assert_eq!(refusal.kind(), RefusalKind::Unsupported, "{refusal}");
}

// The writer (ADR-0057 §6): a sheet is written as a surface model and
// reads back as the sheet it was.

use arris_io::step::{self, Occurrence, StepError, Unsupported};
use arris_math::{Control, Isometry};
use arris_topo::Body;

/// The surface models of `real/`, each read into its own model.
fn surface_models() -> Vec<(&'static str, Model, Body)> {
    [
        "face-based-surface-model/corner.stp",
        "surface-model-extract/extract.stp",
        "surface-model-fill/fill.stp",
        "surface-model-split/split.stp",
    ]
    .into_iter()
    .map(|path| {
        let text =
            std::fs::read_to_string(fixtures::corpus_root().join("real").join(path)).unwrap();
        let (m, solid) = read_one(&text);
        (path, m, solid.result.unwrap().body)
    })
    .collect()
}

fn assert_same_sheet(a: (&Model, Body), b: (&Model, Body), what: &str) {
    assert_eq!(b.0.body(b.1.id).unwrap().kind(), BodyKind::Sheet, "{what}");
    assert_eq!(
        a.0.faces(a.1).unwrap().len(),
        b.0.faces(b.1).unwrap().len(),
        "{what}"
    );
    let report = check(b.0, b.1, Level::Full);
    assert!(
        report.is_ok() && report.unchecked().is_empty(),
        "{what}: {report}"
    );
    let (p, q) = (
        surface_properties(a.0, a.1).unwrap(),
        surface_properties(b.0, b.1).unwrap(),
    );
    assert!(
        (p.area - q.area).abs() <= 1e-9 * p.area,
        "{what}: area {} vs {}",
        p.area,
        q.area
    );
    assert!((p.centroid - q.centroid).norm() <= 1e-7, "{what}: centroid");
    let scale = p.inertia.norm();
    assert!(
        (p.inertia - q.inertia).norm() <= 1e-8 * scale,
        "{what}: inertia"
    );
}

#[test]
fn every_surface_model_of_the_corpus_writes_and_reads_back_as_a_sheet() {
    for (name, m, sheet) in surface_models() {
        let text = step::write(&m, &[sheet]).unwrap();
        assert!(text.contains("SHELL_BASED_SURFACE_MODEL("), "{name}");
        assert!(
            text.contains("MANIFOLD_SURFACE_SHAPE_REPRESENTATION("),
            "{name}"
        );
        assert!(
            !text.contains("ADVANCED_BREP_SHAPE_REPRESENTATION("),
            "{name}"
        );
        assert_eq!(
            step::write(&m, &[sheet]).unwrap(),
            text,
            "{name}: deterministic"
        );
        let (back, solid) = read_one(&text);
        assert_same_sheet((&m, sheet), (&back, solid.result.unwrap().body), name);
    }
}

#[test]
fn a_closed_sheet_is_a_closed_shell_and_an_open_one_an_open_shell() {
    let text = corner()
        .replace(FACE_BASED, "#15 = SHELL_BASED_SURFACE_MODEL('',(#16));")
        .replace(
            THREE_FACES,
            &format!("#16 = CLOSED_SHELL('',({SIX_FACES}));"),
        );
    let (m, solid) = read_one(&text);
    let closed = step::write(&m, &[solid.result.unwrap().body]).unwrap();
    assert_eq!(
        (
            closed.matches("CLOSED_SHELL(").count(),
            closed.matches("OPEN_SHELL(").count()
        ),
        (1, 0)
    );
    let (_, m, sheet) = surface_models().remove(0);
    let open = step::write(&m, &[sheet]).unwrap();
    assert_eq!(
        (
            open.matches("CLOSED_SHELL(").count(),
            open.matches("OPEN_SHELL(").count()
        ),
        (0, 1)
    );
}

#[test]
fn a_solid_and_a_sheet_share_one_file_and_one_product() {
    let (_, mut m, sheet) = surface_models().remove(0);
    let solid = arris_debug::sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = step::write(&m, &[solid, sheet]).unwrap();
    assert_eq!(text.matches("PRODUCT(").count(), 1);
    assert_eq!(
        text.matches("SHAPE_REPRESENTATION_RELATIONSHIP(").count(),
        1
    );
    let mut back = Model::default();
    let read = step_read(&mut back, &text, &ReadOptions::default()).unwrap();
    let kinds: Vec<BodyKind> = read
        .solids
        .iter()
        .map(|s| {
            back.body(s.result.as_ref().unwrap().body.id)
                .unwrap()
                .kind()
        })
        .collect();
    assert_eq!(
        kinds.iter().filter(|&&k| k == BodyKind::Solid).count(),
        1,
        "{kinds:?}"
    );
    assert_eq!(
        kinds.iter().filter(|&&k| k == BodyKind::Sheet).count(),
        1,
        "{kinds:?}"
    );
}

#[test]
fn a_product_tree_carries_a_sheet_in_its_occurrence() {
    let (_, m, sheet) = surface_models().remove(0);
    let part = Occurrence {
        product: None,
        name: "skin".into(),
        placement: Ok(Isometry::identity()),
        colour: None,
        solids: vec![0],
        children: vec![],
    };
    let tree = step::ProductTree {
        roots: vec![part],
        faces: vec![],
    };
    let text = step::write_products(&m, &[sheet], &tree, &Control::NONE).unwrap();
    let mut back = Model::default();
    let read = step_read(&mut back, &text, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), 1);
    assert_eq!(read.products.roots[0].name, "skin");
    assert_eq!(read.products.roots[0].solids, vec![0]);
    assert_same_sheet(
        (&m, sheet),
        (&back, read.solids[0].result.as_ref().unwrap().body),
        "skin",
    );
}

#[test]
fn a_wire_body_is_still_unsupported() {
    use arris_topo::entity::Body as BodyEntity;
    let mut m = Model::default();
    let wire = Body::forward(m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        Vec::new(),
        Vec::new(),
        Vec::new(),
    )));
    assert_eq!(
        step::write(&m, &[wire]),
        Err(StepError::Unsupported {
            body: wire,
            what: Unsupported::Kind(BodyKind::Wire)
        })
    );
}
