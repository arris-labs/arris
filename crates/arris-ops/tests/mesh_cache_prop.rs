//! **A face's mesh survives an edit** (plans/per-face-tessellation step 6;
//! ADR-0010's stable ids, ADR-0052): mesh every face of a body apart,
//! apply an operation, keep the old mesh of each face the provenance says
//! survived, mesh only the others, and weld. The weld succeeds, the result
//! is the one-call mesh of the new body (closed exactly when that is), and
//! each kept face's old mesh equals its new one bit for bit — which is what
//! lets a consumer cache meshes per `FaceId` across edits.
//!
//! The operands are the box, cylinder and rounded box of `multi_cut`, the
//! cone, ball, ring and elliptic prism of `quadric_pair`, and a box whose
//! faces are NURBS; the operations are `cut`, `fuse`, `cut_many`, `split`
//! (each side), `fillet`, `chamfer`, `offset_faces` and `shell`. A refusal
//! by name is allowed, by the operation or by the mesher; a result is held
//! wherever one comes back.
//!
//! A failure prints the shrunk case and the seed and becomes a fixture
//! under `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).

use arris_debug::prop::body::{MultiCut, QuadricPair, multi_cut, quadric_pair};
use arris_debug::prop_shards;
use arris_debug::sample;
use arris_debug::testing::fail;
use arris_debug::unmetered::{chamfer, cut, cut_many, fillet, fuse, offset_faces, shell, split};
use arris_math::{Control, Frame, Point3};
use arris_mesh::{MeshRequest, TriMesh, tessellate_faces};
use arris_ops::{OpError, ShellSide};
use arris_topo::{Body, EdgeId, FaceId, Model, Orientation, Provenance, Shape};
use proptest::prelude::*;

const CHORD: f64 = 1e-2;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Op {
    Cut,
    Fuse,
    CutMany,
    /// The plane through the operand's centre, turned by the pose.
    Split,
    /// The `n`th edge of the body, modulo their count.
    Fillet(usize),
    Chamfer(usize),
    /// The `n`th face of the body, modulo their count.
    Offset(usize),
    Shell(usize),
}

fn op() -> impl Strategy<Value = Op> {
    let pick = || 0usize..256;
    prop_oneof![
        Just(Op::Cut),
        Just(Op::Fuse),
        Just(Op::CutMany),
        Just(Op::Split),
        pick().prop_map(Op::Fillet),
        pick().prop_map(Op::Chamfer),
        pick().prop_map(Op::Offset),
        pick().prop_map(Op::Shell),
    ]
}

#[derive(Debug, Clone)]
enum Operand {
    Multi {
        cut: MultiCut,
        drilled: usize,
    },
    Quadric(QuadricPair),
    /// A box of NURBS faces from the origin to this corner.
    Nurbs([f64; 3]),
}

fn operand() -> impl Strategy<Value = Operand> {
    prop_oneof![
        (multi_cut(), 0usize..=2).prop_map(|(cut, drilled)| Operand::Multi { cut, drilled }),
        quadric_pair().prop_map(Operand::Quadric),
        (5.0..40.0f64, 5.0..40.0f64, 5.0..40.0f64).prop_map(|(x, y, z)| Operand::Nurbs([x, y, z])),
    ]
}

fn is_refusal(e: &OpError) -> bool {
    !matches!(e, OpError::Internal(_) | OpError::InvalidInput { .. })
}

/// One body's every face meshed alone.
fn per_face(m: &Model, body: Body) -> Option<Vec<(FaceId, TriMesh)>> {
    let request = MeshRequest::new(CHORD);
    m.faces(body)
        .ok()?
        .iter()
        .map(|f| {
            tessellate_faces(m, body, &[f.id], &request, &Control::NONE)
                .ok()
                .map(|mesh| (f.id, mesh))
        })
        .collect()
}

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

/// `result` of an edit of a body whose faces were meshed apart as `old`:
/// the kept faces' old meshes and one mesh of the rest weld to the one-call
/// mesh, and each kept face's old mesh is its new one.
fn holds(
    m: &Model,
    old: &[(FaceId, TriMesh)],
    result: Body,
    provenance: &Provenance,
) -> Result<(), TestCaseError> {
    let request = MeshRequest::new(CHORD);
    let Ok(whole) = arris_debug::unmetered::tessellate_with(m, result, &request) else {
        return Ok(());
    };
    let faces: Vec<FaceId> = m
        .faces(result)
        .map_err(fail)?
        .iter()
        .map(|f| f.id)
        .collect();
    let mut parts = Vec::new();
    let mut rest = Vec::new();
    for &face in &faces {
        let kept = provenance.is_kept(Shape::new(face, Orientation::Forward), m, result);
        match old.iter().find(|(id, _)| *id == face).filter(|_| kept) {
            Some((_, mesh)) => {
                prop_assert_eq!(
                    triples(mesh, face),
                    triples(&whole, face),
                    "kept {} meshes differently after the edit",
                    face
                );
                parts.push(mesh.clone());
            }
            None => rest.push(face),
        }
    }
    if !rest.is_empty() {
        let Ok(mesh) = tessellate_faces(m, result, &rest, &request, &Control::NONE) else {
            return Ok(());
        };
        parts.push(mesh);
    }
    let welded = TriMesh::weld(&parts).map_err(|e| fail(format!("weld: {e}")))?;
    prop_assert_eq!(welded.is_closed(), whole.is_closed());
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
    Ok(())
}

prop_shards! {
    /// Kept faces keep their meshes; the weld of the rest is the whole.
    a_kept_faces_mesh_survives_the_edit
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (draw) = (operand(), op()) => {
            let (operand, op) = draw;
            let mut m = Model::default();
            // The body, the tools it is edited with, and the centre and
            // turn of a splitting plane.
            let (body, tools, plane) = match &operand {
                Operand::Multi { cut, drilled } => {
                    let (target, tools) = cut.build(&mut m).map_err(fail)?;
                    let n = (*drilled).min(tools.len());
                    let body = if n == 0 {
                        target
                    } else {
                        match cut_many(&mut m, target, &tools[..n]) {
                            Ok((b, _)) => b,
                            Err(e) if is_refusal(&e) => return Ok(()),
                            Err(e) => return Err(fail(format!("building the body: {e}"))),
                        }
                    };
                    let origin = cut.pose.apply(Point3::origin());
                    let plane = Frame::from_rotation(origin, &cut.pose.rotation());
                    (body, tools[n..].to_vec(), plane)
                }
                Operand::Quadric(pair) => {
                    let (solid, tool) = pair.build(&mut m).map_err(fail)?;
                    let origin = pair.pose.apply(Point3::origin());
                    (solid, vec![tool], Frame::from_rotation(origin, &pair.pose.rotation()))
                }
                Operand::Nurbs(max) => {
                    let corner = Point3::new(max[0], max[1], max[2]);
                    let body = sample::cuboid_nurbs(&mut m, Point3::origin(), corner).map_err(fail)?;
                    let plane = Frame::from_rotation(
                        Point3::from(corner.coords / 2.0),
                        &Frame::world().rotation(),
                    );
                    (body, Vec::new(), plane)
                }
            };
            let Some(old) = per_face(&m, body) else { return Ok(()) };
            let faces = m.faces(body).map_err(fail)?;
            let edges = m.edges(body).map_err(fail)?;
            if faces.is_empty() || edges.is_empty() {
                return Ok(());
            }
            let made = match op {
                Op::Cut | Op::Fuse | Op::CutMany if tools.is_empty() => return Ok(()),
                Op::Cut => cut(&mut m, body, tools[0]),
                Op::Fuse => fuse(&mut m, body, tools[0]),
                Op::CutMany => cut_many(&mut m, body, &tools),
                Op::Split => match split(&mut m, body, &plane) {
                    Ok(halves) => {
                        for side in [halves.positive, halves.negative] {
                            holds(&m, &old, side, &halves.provenance)
                                .map_err(|e| fail(format!("split: {e}")))?;
                        }
                        return Ok(());
                    }
                    Err(e) => Err(e),
                },
                Op::Fillet(k) => fillet(&mut m, body, &[edges[k % edges.len()]], 0.05),
                Op::Chamfer(k) => chamfer(&mut m, body, &[edges[k % edges.len()]], 0.05),
                Op::Offset(k) => offset_faces(&mut m, body, &[faces[k % faces.len()]], 0.05),
                Op::Shell(k) => shell(&mut m, body, &[faces[k % faces.len()]], 0.05, ShellSide::Inward),
            };
            match made {
                Ok((result, provenance)) => {
                    holds(&m, &old, result, &provenance).map_err(|e| fail(format!("{op:?}: {e}")))?;
                }
                Err(e) if is_refusal(&e) => {}
                Err(OpError::Internal(_)) => {}
                Err(e) => return Err(fail(format!("{op:?}: {e}"))),
            }
            Ok(())
        }
}
