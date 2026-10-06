//! `tessellate_faces` (ADR-0052): a subset of a body's faces meshes to
//! the same edges' polylines and the same faces' triangles, position for
//! position, as the one-call mesh of the whole body, whichever faces
//! were left out, so faces meshed at different times meet bit for bit.

use arris_debug::prop::check;
use arris_debug::prop::recipe::recipe;
use arris_debug::sample;
use arris_debug::testing::fail;
use arris_debug::{corpus, unmetered};
use arris_math::{Control, Point3};
use arris_mesh::{MeshError, MeshRequest, TriMesh, tessellate_faces};
use arris_topo::{Body, FaceId, Model};
use proptest::prelude::*;

const CHORD: f64 = 1e-2;

type Triple = [[f64; 3]; 3];

/// A face's triangles as position triples, in the mesh's order.
fn triples(mesh: &TriMesh, face: FaceId) -> Option<Vec<Triple>> {
    let range = mesh.faces().iter().find(|r| r.face == face)?;
    Some(
        mesh.triangles()[range.triangles.clone()]
            .iter()
            .map(|t| t.map(|i| mesh.positions()[i as usize]))
            .collect(),
    )
}

/// An edge's polyline as positions.
fn polyline(mesh: &TriMesh, edge: arris_topo::EdgeId) -> Option<Vec<[f64; 3]>> {
    Some(
        mesh.edge_polyline(edge)?
            .iter()
            .map(|&i| mesh.positions()[i as usize])
            .collect(),
    )
}

/// Meshes `faces` of `body` and holds every edge's polyline and every
/// face's triangles to the whole body's, bit for bit. A body the whole
/// call refuses is not a question about subsets.
fn holds(m: &Model, body: Body, mask: u64) -> Result<(), TestCaseError> {
    let request = MeshRequest::new(CHORD);
    let Ok(whole) = unmetered::tessellate_with(m, body, &request) else {
        return Ok(());
    };
    let all: Vec<FaceId> = m.faces(body).map_err(fail)?.iter().map(|f| f.id).collect();
    let picked: Vec<FaceId> = all
        .iter()
        .enumerate()
        .filter(|(i, _)| mask >> (i % 64) & 1 == 1)
        .map(|(_, &f)| f)
        .collect();
    let part = tessellate_faces(m, body, &picked, &request, &Control::NONE)
        .map_err(|e| fail(format!("{e}")))?;
    prop_assert_eq!(
        part.faces().iter().map(|r| r.face).collect::<Vec<_>>(),
        picked.clone()
    );
    for &face in &picked {
        prop_assert_eq!(triples(&part, face), triples(&whole, face), "{}", face);
    }
    prop_assert!(!part.edges().is_empty() || picked.is_empty());
    for range in part.edges() {
        prop_assert_eq!(
            polyline(&part, range.edge),
            polyline(&whole, range.edge),
            "{}",
            range.edge
        );
    }
    Ok(())
}

arris_debug::prop_shards! {
    /// Random draws of the recipe strategy — booleans, blends, cones and
    /// all — meshed face subset against whole.
    a_subset_of_a_drawn_bodys_faces_meshes_as_the_whole_does
        [s0 s1 s2 s3 s4 s5 s6 s7] (draw) = (recipe(), any::<u64>()) => {
        let (r, mask) = draw;
        let Ok(Ok(chain)) = std::panic::catch_unwind(|| corpus::build("generated/faces", &r)) else {
            return Ok(());
        };
        let Some(body) = chain.result() else { return Ok(()) };
        holds(&chain.model, body, mask)
    }
}

#[test]
fn every_face_alone_and_random_subsets_of_the_sample_bodies_mesh_as_the_whole_does() {
    let mut m = Model::default();
    let bodies = [
        sample::unit_box(&mut m).unwrap(),
        sample::cylinder(&mut m, 4.0, 12.0).unwrap(),
        sample::sphere(&mut m, Point3::origin(), 3.0).unwrap(),
        sample::torus(&mut m, Point3::origin(), 5.0, 1.5).unwrap(),
    ];
    for body in bodies {
        let n = m.faces(body).unwrap().len();
        for i in 0..n {
            holds(&m, body, 1 << i).unwrap();
        }
        check(any::<u64>(), |mask| holds(&m, body, mask));
    }
}

#[test]
fn the_whole_set_is_the_one_call_mesh_and_duplicates_collapse() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let request = MeshRequest::new(1e-3);
    let whole = unmetered::tessellate_with(&m, body, &request).unwrap();
    let ids: Vec<FaceId> = m.faces(body).unwrap().iter().map(|f| f.id).collect();
    let mut shuffled = vec![ids[2], ids[0], ids[1], ids[0]];
    let all = tessellate_faces(&m, body, &shuffled, &request, &Control::NONE).unwrap();
    assert_eq!(all, whole);
    shuffled.truncate(1);
    let one = tessellate_faces(&m, body, &shuffled, &request, &Control::NONE).unwrap();
    assert_eq!(one.faces().len(), 1);
}

#[test]
fn a_face_of_another_body_is_named() {
    let mut m = Model::default();
    let a = sample::unit_box(&mut m).unwrap();
    let b = sample::cylinder(&mut m, 1.0, 2.0).unwrap();
    let stray = m.faces(b).unwrap()[0].id;
    let err =
        tessellate_faces(&m, a, &[stray], &MeshRequest::new(CHORD), &Control::NONE).unwrap_err();
    assert_eq!(
        err,
        MeshError::NotInBody {
            face: stray,
            body: a
        }
    );
}
