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
