//! **A kept edge keeps its pcurves** (plans/per-face-tessellation step 1;
//! ADR-0010's stable ids): an edge an operation keeps by id, seen from a
//! face whose surface the operation leaves as it was, is parameterised on
//! that surface by the same pcurve — the same `Curve2Id`, or a value equal
//! bit for bit. A face meshed before an edit and a neighbour meshed after
//! it can only meet along the edge if both read the edge the same way.
//!
//! The operands are a box, a cylinder or a rounded box in a random pose,
//! plain or drilled by some of its tools; the operations are `cut`,
//! `fuse`, `cut_many`, `split`, `fillet`, `chamfer`, `offset_faces` and
//! `shell`. A refusal by name is allowed; a result is compared wherever
//! one comes back.
//!
//! A failure prints the shrunk case and the seed and becomes a fixture
//! under `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).

use std::collections::BTreeMap;

use arris_debug::prop::body::{MultiCut, multi_cut};
use arris_debug::prop_shards;
use arris_debug::testing::fail;
use arris_debug::unmetered::{chamfer, cut, cut_many, fillet, fuse, offset_faces, shell, split};
use arris_geom::{Curve2, Surface};
use arris_math::{Frame, Point3};
use arris_ops::{OpError, ShellSide};
use arris_topo::{Body, Curve2Id, EdgeId, Face, Model, Provenance};
use proptest::prelude::*;

/// Which operation a case applies, and where.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Op {
    Cut,
    Fuse,
    CutMany,
    /// The plane through the target's centre, normal along the pose's `z`.
    Split,
    /// The `n`th edge of the body, modulo their count.
    Fillet(usize),
    Chamfer(usize),
    /// The `n`th face of the body, modulo their count.
    Offset(usize),
    Shell(usize),
}

#[derive(Debug, Clone)]
struct Case {
    cut: MultiCut,
    /// How many of the tools drill the operand first; zero leaves it plain.
    drilled: usize,
    op: Op,
}

fn case() -> impl Strategy<Value = Case> {
    let pick = || 0usize..256;
    (
        multi_cut(),
        0usize..=2,
        prop_oneof![
            Just(Op::Cut),
            Just(Op::Fuse),
            Just(Op::CutMany),
            Just(Op::Split),
            pick().prop_map(Op::Fillet),
            pick().prop_map(Op::Chamfer),
            pick().prop_map(Op::Offset),
            pick().prop_map(Op::Shell),
        ],
    )
        .prop_map(|(cut, drilled, op)| Case { cut, drilled, op })
}

/// One use of an edge by a face of the body, as the pcurve and surface it
/// had before the operation.
struct Use {
    surface: Surface,
    pcurve: Curve2Id,
    value: Curve2,
}

/// Every use of every edge of `body`, by the face's surface.
fn uses(m: &Model, body: Body) -> Result<BTreeMap<EdgeId, Vec<Use>>, TestCaseError> {
    let mut out: BTreeMap<EdgeId, Vec<Use>> = BTreeMap::new();
    let faces = m.faces(body).map_err(fail)?;
    for f in faces {
        let face = m.face(f.id).map_err(fail)?;
        let surface = m.surface(face.surface()).map_err(fail)?;
        for l in face.loops() {
            for c in l.coedges() {
                out.entry(c.edge()).or_default().push(Use {
                    surface: surface.clone(),
                    pcurve: c.pcurve(),
                    value: m.curve2(c.pcurve()).map_err(fail)?.clone(),
                });
            }
        }
    }
    Ok(out)
}

/// Every edge of `before` that `provenance` keeps in `output` carries, on
/// each face of `output` whose surface equals the surface of one of its
/// faces in `before`, a pcurve equal to one of that surface's.
fn kept_edges_keep_pcurves(
    m: &Model,
    before: &BTreeMap<EdgeId, Vec<Use>>,
    output: Body,
    provenance: &Provenance,
) -> Result<(), TestCaseError> {
    let after = uses(m, output)?;
    for (&edge, then) in before {
        if !provenance.is_kept(
            arris_topo::Shape::new(edge, arris_topo::Orientation::Forward),
            m,
            output,
        ) {
            continue;
        }
        for now in after.get(&edge).into_iter().flatten() {
            let same_surface: Vec<&Use> =
                then.iter().filter(|u| u.surface == now.surface).collect();
            if same_surface.is_empty() {
                continue;
            }
            let held = same_surface
                .iter()
                .any(|u| u.pcurve == now.pcurve || u.value == now.value);
            prop_assert!(
                held,
                "kept edge {} changed its pcurve on an unchanged {:?}: {:?} became {:?}",
                edge,
                now.surface,
                same_surface.iter().map(|u| &u.value).collect::<Vec<_>>(),
                now.value
            );
        }
    }
    Ok(())
}

/// What an operation returns.
type Made = Result<(Body, Provenance), OpError>;

fn is_refusal(e: &OpError) -> bool {
    !matches!(e, OpError::Internal(_) | OpError::InvalidInput { .. })
}

prop_shards! {
    /// A kept edge keeps its pcurve on every unchanged surface.
    kept_edges_keep_their_pcurves
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = case() => {
            let mut m = Model::default();
            let (target, tools) = case.cut.build(&mut m).map_err(fail)?;
            let n = case.drilled.min(tools.len());
            let body = if n == 0 {
                target
            } else {
                match cut_many(&mut m, target, &tools[..n]) {
                    Ok((b, _)) => b,
                    Err(e) if is_refusal(&e) => return Ok(()),
                    Err(e) => return Err(fail(format!("building the body: {e}"))),
                }
            };
            // A tool that has not drilled the body: re-adding one that has
            // is the coincident case, which is the boolean's own tests'.
            let Some(&tool) = tools.get(n) else {
                return Ok(());
            };
            let before = uses(&m, body)?;
            let faces = m.faces(body).map_err(fail)?;
            let edges = m.edges(body).map_err(fail)?;
            let made: Made = match case.op {
                Op::Cut => cut(&mut m, body, tool),
                Op::Fuse => fuse(&mut m, body, tool),
                Op::CutMany => cut_many(&mut m, body, &tools[n..]),
                Op::Split => {
                    let origin = case.cut.pose.apply(Point3::origin());
                    let plane = Frame::from_rotation(origin, &case.cut.pose.rotation());
                    match split(&mut m, body, &plane) {
                        Ok(halves) => {
                            for (side, name) in [(halves.positive, "positive"), (halves.negative, "negative")] {
                                kept_edges_keep_pcurves(&m, &before, side, &halves.provenance)
                                    .map_err(|e| fail(format!("{name} side of split: {e}")))?;
                            }
                            return Ok(());
                        }
                        Err(e) => Err(e),
                    }
                }
                Op::Fillet(k) => fillet(&mut m, body, &[edges[k % edges.len()]], 0.05),
                Op::Chamfer(k) => chamfer(&mut m, body, &[edges[k % edges.len()]], 0.05),
                Op::Offset(k) => {
                    let f: Face = faces[k % faces.len()];
                    offset_faces(&mut m, body, &[f], 0.05)
                }
                Op::Shell(k) => {
                    let f: Face = faces[k % faces.len()];
                    shell(&mut m, body, &[f], 0.05, ShellSide::Inward)
                }
            };
            match made {
                Ok((result, provenance)) => {
                    kept_edges_keep_pcurves(&m, &before, result, &provenance)
                        .map_err(|e| fail(format!("{:?}: {e}", case.op)))?;
                }
                Err(e) if is_refusal(&e) => {}
                Err(OpError::Internal(_)) => {}
                Err(e) => return Err(fail(format!("{:?}: {e}", case.op))),
            }
            Ok(())
        }
}
