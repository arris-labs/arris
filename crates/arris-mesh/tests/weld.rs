//! `TriMesh::weld` (ADR-0052): a body's faces meshed in random parts and
//! welded in a random order are the one-call mesh, closed, edge for edge
//! and triangle for triangle; an edge meshed at two chords is refused.

use arris_debug::prop::recipe::recipe;
use arris_debug::sample;
use arris_debug::testing::fail;
use arris_debug::{corpus, unmetered};
use arris_math::{Control, Point3};
use arris_mesh::{MeshError, MeshRequest, TriMesh, tessellate_faces};
use arris_topo::{Body, EdgeId, FaceId, Model};
use proptest::prelude::*;

const CHORD: f64 = 1e-2;

type Triple = [[f64; 3]; 3];

fn triples(mesh: &TriMesh, face: FaceId) -> Option<Vec<Triple>> {
    let range = mesh.faces().iter().find(|r| r.face == face)?;
    Some(
        mesh.triangles()[range.triangles.clone()]
            .iter()
            .map(|t| t.map(|i| mesh.positions()[i as usize]))
            .collect(),
    )
}

fn polyline(mesh: &TriMesh, edge: EdgeId) -> Option<Vec<[f64; 3]>> {
    Some(
        mesh.edge_polyline(edge)?
            .iter()
            .map(|&i| mesh.positions()[i as usize])
            .collect(),
    )
}

/// A deterministic stream from a seed (splitmix64).
fn next(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// Meshes a random partition of `body`'s faces part by part, welds the
/// parts in a random order and holds the result to the one-call mesh.
fn holds(m: &Model, body: Body, seed: u64, corners: bool) -> Result<(), TestCaseError> {
    let mut request = MeshRequest::new(CHORD);
    if corners {
        request = request.with_corners();
    }
    let Ok(whole) = unmetered::tessellate_with(m, body, &request) else {
        return Ok(());
    };
    let faces: Vec<FaceId> = m.faces(body).map_err(fail)?.iter().map(|f| f.id).collect();
    let mut state = seed;
    let k = 1 + (next(&mut state) % faces.len().max(1) as u64) as usize;
    let mut groups: Vec<Vec<FaceId>> = vec![Vec::new(); k];
    for &f in &faces {
        groups[(next(&mut state) % k as u64) as usize].push(f);
    }
    groups.retain(|g| !g.is_empty());
    let mut parts = Vec::new();
    for g in &groups {
        parts.push(
            tessellate_faces(m, body, g, &request, &Control::NONE)
                .map_err(|e| fail(format!("{e}")))?,
        );
    }
    for i in (1..parts.len()).rev() {
        parts.swap(i, (next(&mut state) % (i as u64 + 1)) as usize);
    }
    let welded = TriMesh::weld(&parts).map_err(|e| fail(format!("{e}")))?;
    prop_assert_eq!(welded.is_closed(), whole.is_closed());
    prop_assert_eq!(welded.corners().is_some(), corners);
    prop_assert_eq!(welded.triangles().len(), whole.triangles().len());
    for &face in &faces {
        prop_assert_eq!(triples(&welded, face), triples(&whole, face), "{}", face);
    }
    prop_assert_eq!(welded.edges().len(), whole.edges().len());
    for range in whole.edges() {
        prop_assert_eq!(
            polyline(&welded, range.edge),
            polyline(&whole, range.edge),
            "{}",
            range.edge
        );
    }
    if let (Some(a), Some(b)) = (welded.corners(), whole.corners()) {
        for &face in &faces {
            let (fa, fb) = (a.face(face).unwrap(), b.face(face).unwrap());
            prop_assert_eq!(
                &a.normals()[fa.vertices.clone()],
                &b.normals()[fb.vertices.clone()]
            );
            prop_assert_eq!(&a.uvs()[fa.vertices.clone()], &b.uvs()[fb.vertices.clone()]);
            prop_assert_eq!(fa.uv_box, fb.uv_box);
        }
        prop_assert_eq!(a.positions().len(), b.positions().len());
    }
    Ok(())
}

arris_debug::prop_shards! {
    /// Random draws of the recipe strategy, partitioned, meshed and welded.
    a_welded_partition_of_a_drawn_bodys_faces_is_the_one_call_mesh
        [s0 s1 s2 s3 s4 s5 s6 s7] (draw) = (recipe(), any::<u64>(), any::<bool>()) => {
        let (r, seed, corners) = draw;
        let Ok(Ok(chain)) = std::panic::catch_unwind(|| corpus::build("generated/weld", &r)) else {
            return Ok(());
        };
        let Some(body) = chain.result() else { return Ok(()) };
        holds(&chain.model, body, seed, corners)
    }
}

#[test]
fn a_welded_partition_of_the_sample_bodies_is_the_one_call_mesh() {
    let mut m = Model::default();
    let bodies = [
        sample::unit_box(&mut m).unwrap(),
        sample::cylinder(&mut m, 4.0, 12.0).unwrap(),
        sample::sphere(&mut m, Point3::origin(), 3.0).unwrap(),
        sample::torus(&mut m, Point3::origin(), 5.0, 1.5).unwrap(),
    ];
    for body in bodies {
        for seed in 0..40 {
            holds(&m, body, seed, seed % 2 == 0).unwrap();
        }
    }
}

#[test]
fn an_edge_meshed_at_two_chords_is_refused_by_name() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let faces: Vec<FaceId> = m.faces(body).unwrap().iter().map(|f| f.id).collect();
    let mesh = |face: FaceId, chord: f64| {
        tessellate_faces(&m, body, &[face], &MeshRequest::new(chord), &Control::NONE).unwrap()
    };
    let fine = mesh(faces[0], 1e-3);
    let coarse = mesh(faces[1], 1e-1);
    match TriMesh::weld(&[fine.clone(), coarse]) {
        Err(MeshError::WeldMismatch { edge }) => {
            assert!(fine.edge_polyline(edge).is_some());
        }
        other => panic!("expected a mismatch, got {other:?}"),
    }
    // The same chord welds.
    assert!(TriMesh::weld(&[fine, mesh(faces[1], 1e-3)]).is_ok());
}

#[test]
fn no_parts_weld_to_an_empty_mesh_and_one_part_to_itself() {
    assert_eq!(TriMesh::weld(&[]).unwrap(), TriMesh::new());
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let whole = unmetered::tessellate_with(&m, body, &MeshRequest::new(CHORD)).unwrap();
    assert_eq!(TriMesh::weld(std::slice::from_ref(&whole)).unwrap(), whole);
}
