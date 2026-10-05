//! The keep/regenerate assembly and its provenance writer (ADR-0004): a
//! surviving piece is assembled fresh unless it is a whole face
//! untouched under a `Reuse` operand, in which case it is kept by id;
//! every touched entity's provenance is written from the pieces as they
//! are made. `boolean::result` is the first producer of pieces; a blend
//! is the second, through [`rewrite`] — one operand, its faces kept by
//! id unless replaced, the blend faces added (ADR-0007).

use std::collections::{BTreeMap, BTreeSet};

use arris_math::Point2;
use arris_topo::builder::{
    Assembly, Builder, EdgeKey, EdgeSpec, FaceSpec, UseSpec, VertexKey, VertexSpec, effective_uses,
};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::{
    Body, Curve2Id, CurveId, EdgeId, EntityId, Face as FaceHandle, FaceId, Model, Orientation,
    Provenance, Shape, ShellId, SurfaceId, VertexId,
};

use crate::boolean::Interferences;
use crate::boolean::pieces::{ERef, PieceUse, SubEdge, VRef};
use crate::error::{Fault, OpError};

/// A `Forward` handle to `id`, as a [`Shape`].
pub(crate) fn forward(id: impl Into<EntityId>) -> Shape {
    Shape::new(id, Orientation::Forward)
}

/// What happens to an operand's entities that survive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Policy {
    /// An untouched entity keeps its id; a piece is `Modified` from its
    /// parent.
    Reuse,
    /// Every entity is `Deleted`; a surviving piece is a new entity
    /// `Generated` from its parent — the tool of a `cut`.
    Regenerate,
}

/// A surviving piece, before assembly.
pub(crate) struct Kept {
    /// Which operand it came from.
    pub side: usize,
    /// The input face, with the operand's use of it.
    pub face: FaceHandle,
    /// The whole face, untouched and reusable by id.
    pub whole: bool,
    /// The result's use of it.
    pub orientation: Orientation,
    /// In the stored sense.
    pub loops: Vec<Vec<PieceUse>>,
    /// A point strictly inside the piece in the face's own (u, v): the
    /// split order's tiebreak (ADR-0009).
    pub uv: Point2,
    /// The coincident face of the other operand the piece lies on and
    /// stands in for, when it does.
    pub stands_for: Option<FaceId>,
}

/// The assembly and the order its new slots were given in.
pub(crate) struct Plan {
    pub assembly: Assembly,
    /// The kept piece behind each face spec, shell by shell in order.
    pub faces: Vec<usize>,
    /// The `VRef` behind each `VertexSpec::New`, in order.
    pub new_vertices: Vec<VRef>,
    /// The `ERef` behind each `EdgeSpec::New`, in order.
    pub new_edges: Vec<ERef>,
    /// Operand vertices kept by id.
    pub kept_vertices: BTreeSet<VertexId>,
    /// Operand edges kept by id.
    pub kept_edges: BTreeSet<EdgeId>,
}

/// The assembly of the kept `pieces`: a vertex spec for every new vertex
/// they reach, an edge spec for every new edge piece, a face spec per
/// piece — `Keep` for a whole untouched face of a `Reuse` operand —
/// shell by shell in `shells`' order, in a deterministic order.
#[allow(clippy::too_many_arguments)]
pub(crate) fn assembly(
    m: &Model,
    policy: [Policy; 2],
    vertices: &[Vec<VertexId>; 2],
    edges: &[Vec<EdgeId>; 2],
    sub_edges: &BTreeMap<EdgeId, Vec<SubEdge>>,
    touched: &BTreeSet<EdgeId>,
    retolerated: &BTreeMap<VertexId, f64>,
    edge_tolerance: &BTreeMap<ERef, f64>,
    i: &Interferences,
    curve_ids: &[CurveId],
    vref_of: &[VRef],
    pieces: &[Kept],
    shells: &[Vec<usize>],
) -> Result<Plan, OpError> {
    let side_of_vertex = |v: VertexId| usize::from(vertices[0].binary_search(&v).is_err());
    let vertex_new = |v: VRef| match v {
        VRef::Section(_) => true,
        VRef::Existing(id) => {
            retolerated.contains_key(&id) || policy[side_of_vertex(id)] == Policy::Regenerate
        }
    };
    let edge_new =
        |e: EdgeId, side: usize| touched.contains(&e) || policy[side] == Policy::Regenerate;

    let mut used_edges: BTreeSet<ERef> = BTreeSet::new();
    for piece in pieces {
        for u in piece.loops.iter().flatten() {
            used_edges.insert(u.edge);
        }
    }
    let mut used_vertices: BTreeSet<VRef> = BTreeSet::new();
    for &e in &used_edges {
        let (start, end) = match e {
            ERef::Sub { edge, index } => {
                let s = &sub_edges[&edge][index];
                (s.start, s.end)
            }
            ERef::Section(k) => {
                let s = &i.sections[k];
                (vref_of[s.start], vref_of[s.end])
            }
        };
        used_vertices.insert(start);
        used_vertices.insert(end);
    }

    let mut assembly = Assembly::default();
    let mut new_vertices = Vec::new();
    let mut kept_vertices = BTreeSet::new();
    let mut vkey: BTreeMap<VRef, VertexKey> = BTreeMap::new();
    // Section vertices first, ascending; then operand vertices that are
    // appended, ascending by id; the rest kept.
    let mut order: Vec<VRef> = used_vertices.iter().copied().collect();
    order.sort_by_key(|v| match *v {
        VRef::Section(k) => (0, k, None),
        VRef::Existing(id) => (1, 0, Some(id)),
    });
    for v in order {
        if vertex_new(v) {
            let (point, tolerance) = match v {
                VRef::Section(k) => (i.vertices[k].point, i.vertices[k].tolerance),
                VRef::Existing(id) => {
                    let stored = m.vertex(id)?;
                    (
                        stored.point(),
                        retolerated.get(&id).copied().unwrap_or(stored.tolerance()),
                    )
                }
            };
            assembly.vertices.push(VertexSpec::New { point, tolerance });
            vkey.insert(v, VertexKey::New(new_vertices.len()));
            new_vertices.push(v);
        } else if let VRef::Existing(id) = v {
            vkey.insert(v, VertexKey::Kept(id));
            kept_vertices.insert(id);
        }
    }

    let mut new_edges = Vec::new();
    let mut kept_edges = BTreeSet::new();
    let mut ekey: BTreeMap<ERef, EdgeKey> = BTreeMap::new();
    for (side, side_edges) in edges.iter().enumerate() {
        for &e in side_edges {
            let edge = *m.edge(e)?;
            let subs = &sub_edges[&e];
            if !edge_new(e, side) {
                let whole = ERef::Sub { edge: e, index: 0 };
                if used_edges.contains(&whole) {
                    ekey.insert(whole, EdgeKey::Kept(e));
                    kept_edges.insert(e);
                }
                continue;
            }
            for (index, s) in subs.iter().enumerate() {
                let r = ERef::Sub { edge: e, index };
                if !used_edges.contains(&r) {
                    continue;
                }
                let geometry = match edge.geometry() {
                    EdgeGeometry::Curve { curve, .. } => EdgeGeometry::Curve {
                        curve,
                        range: s.range,
                    },
                    EdgeGeometry::Degenerate { .. } => EdgeGeometry::Degenerate { range: s.range },
                };
                // The edge's own tolerance whenever no image or common
                // block raised this piece's — never a literal, since the
                // entity's tolerance is always the floor.
                let tolerance = edge_tolerance
                    .get(&r)
                    .map_or(edge.tolerance(), |t| t.max(edge.tolerance()));
                assembly.edges.push(EdgeSpec::New {
                    geometry,
                    start: vkey[&s.start],
                    end: vkey[&s.end],
                    tolerance,
                });
                ekey.insert(r, EdgeKey::New(new_edges.len()));
                new_edges.push(r);
            }
        }
    }
    for (k, s) in i.sections.iter().enumerate() {
        let r = ERef::Section(k);
        if !used_edges.contains(&r) {
            continue;
        }
        assembly.edges.push(EdgeSpec::New {
            geometry: EdgeGeometry::Curve {
                curve: curve_ids[s.curve],
                range: s.range,
            },
            start: vkey[&vref_of[s.start]],
            end: vkey[&vref_of[s.end]],
            tolerance: s.tolerance,
        });
        ekey.insert(r, EdgeKey::New(new_edges.len()));
        new_edges.push(r);
    }

    let mut face_pieces = Vec::with_capacity(pieces.len());
    for shell in shells {
        let mut faces = Vec::with_capacity(shell.len());
        for &k in shell {
            let Some(piece) = pieces.get(k) else {
                continue;
            };
            face_pieces.push(k);
            if piece.whole {
                faces.push(FaceSpec::Keep(FaceHandle::new(
                    piece.face.id,
                    piece.orientation,
                )));
                continue;
            }
            let entity = m.face(piece.face.id)?;
            let loops = piece
                .loops
                .iter()
                .map(|l| {
                    effective_uses(
                        piece.orientation,
                        l.iter().map(|u| (ekey[&u.edge], u.orientation, u.pcurve)),
                    )
                    .into_iter()
                    .map(|(edge, orientation, pcurve)| UseSpec {
                        edge,
                        orientation,
                        pcurve,
                    })
                    .collect()
                })
                .collect();
            faces.push(FaceSpec::New {
                surface: entity.surface(),
                orientation: piece.orientation,
                loops,
                tolerance: entity.tolerance(),
            });
        }
        assembly.shells.push(faces);
    }
    Ok(Plan {
        assembly,
        faces: face_pieces,
        new_vertices,
        new_edges,
        kept_vertices,
        kept_edges,
    })
}

/// One operand's entities as the provenance writer needs them.
pub(crate) struct OperandWrite<'a> {
    /// The operand body.
    pub body: Body,
    /// Its policy.
    pub policy: Policy,
    /// Its vertices, ascending.
    pub vertices: &'a [VertexId],
    /// Its edges, in iteration order.
    pub edges: &'a [EdgeId],
    /// Its faces with their uses, in iteration order.
    pub faces: &'a [FaceHandle],
    /// Its shells, before the operation.
    pub shells: &'a [ShellId],
    /// The shell each of its faces belongs to.
    pub shell_of: &'a BTreeMap<FaceId, ShellId>,
}

/// The provenance of a keep/regenerate assembly: every operand entity
/// recorded deleted, modified into its images, or generated into them
/// by `operand.policy`, every result shell related to the operand
/// shells its pieces came from, and a piece standing for a coincident
/// face `Generated` from it. Callers with their own reasons for a new
/// vertex or edge — the boolean's section geometry — add those records
/// on top of the one this returns.
#[allow(clippy::too_many_arguments)]
pub(crate) fn write_provenance(
    operands: [OperandWrite<'_>; 2],
    built_body: Body,
    sub_edges: &BTreeMap<EdgeId, Vec<SubEdge>>,
    merged_into: &BTreeMap<VertexId, usize>,
    vref_of: &[VRef],
    vertex_id: impl Fn(VRef) -> Option<VertexId>,
    edge_id: impl Fn(ERef) -> Option<EdgeId>,
    face_pieces: &BTreeMap<FaceId, (bool, Vec<usize>)>,
    out_faces: &BTreeMap<usize, FaceId>,
    kept: &[Kept],
    shells: &[Vec<usize>],
    built_shells: &[ShellId],
) -> Provenance {
    let mut p = Provenance::new();
    for operand in &operands {
        let policy = operand.policy;
        let body = operand.body;
        let record = |p: &mut Provenance, input: Shape, images: Vec<Shape>| match policy {
            Policy::Reuse => {
                if images.is_empty() {
                    p.add_deleted(input);
                } else if images != [input] {
                    for image in images {
                        p.add_modified(input, image);
                    }
                }
            }
            Policy::Regenerate => {
                p.add_deleted(input);
                for image in images {
                    p.add_generated(input, image);
                }
            }
        };
        for &v in operand.vertices {
            let image = match merged_into.get(&v) {
                Some(&k) => vertex_id(vref_of[k]),
                None => vertex_id(VRef::Existing(v)),
            };
            record(&mut p, forward(v), image.into_iter().map(forward).collect());
        }
        // An edge's pieces are recorded in split order (ADR-0009): the
        // sub-edge index ascends along the edge's own curve from its
        // range's start, so `0..n` is that order and the record keeps it.
        for &e in operand.edges {
            let n = sub_edges.get(&e).map_or(0, Vec::len);
            let mut images: Vec<Shape> = Vec::new();
            for index in 0..n {
                if let Some(id) = edge_id(ERef::Sub { edge: e, index }) {
                    let image = forward(id);
                    if !images.contains(&image) {
                        images.push(image);
                    }
                }
            }
            record(&mut p, forward(e), images);
        }
        for f in operand.faces {
            let images: Vec<Shape> = face_pieces
                .get(&f.id)
                .map(|(_, pieces)| {
                    pieces
                        .iter()
                        .filter_map(|k| out_faces.get(k))
                        .map(|&id| forward(id))
                        .collect()
                })
                .unwrap_or_default();
            record(&mut p, forward(f.id), images);
        }
        if policy == Policy::Regenerate {
            for &s in operand.shells {
                p.add_deleted(forward(s));
            }
        }
        match policy {
            Policy::Reuse => p.add_modified(forward(body.id), forward(built_body.id)),
            Policy::Regenerate => p.add_deleted(forward(body.id)),
        }
    }
    // A result shell is `Modified` from every shell of a kept-by-id
    // operand a piece of it came from, and one made of a cut tool's
    // pieces alone is `Generated` from the tool's shell; a shell of a
    // kept-by-id operand no result shell came from is gone.
    let mut reached: BTreeSet<ShellId> = BTreeSet::new();
    for (pieces, &out) in shells.iter().zip(built_shells) {
        let mut from: [BTreeSet<ShellId>; 2] = [BTreeSet::new(), BTreeSet::new()];
        for piece in pieces.iter().filter_map(|&k| kept.get(k)) {
            if let Some(&s) = operands[piece.side].shell_of.get(&piece.face.id) {
                from[piece.side].insert(s);
            }
        }
        let reused: Vec<ShellId> = (0..2)
            .filter(|&side| operands[side].policy == Policy::Reuse)
            .flat_map(|side| from[side].iter().copied())
            .collect();
        if reused.is_empty() {
            for s in from.iter().flatten() {
                p.add_generated(forward(*s), forward(out));
            }
        }
        for s in reused {
            p.add_modified(forward(s), forward(out));
            reached.insert(s);
        }
    }
    for side in (0..2).filter(|&side| operands[side].policy == Policy::Reuse) {
        for &s in operands[side]
            .shells
            .iter()
            .filter(|s| !reached.contains(s))
        {
            p.add_deleted(forward(s));
        }
    }
    // A piece kept once from a coincident pair stands for the other
    // operand's face too.
    for (k, piece) in kept.iter().enumerate() {
        if let (Some(g), Some(&id)) = (piece.stands_for, out_faces.get(&k)) {
            p.add_generated(forward(g), forward(id));
        }
    }
    p
}

/// One use of an edge by a loop of a rewritten or added face, in the
/// *stored* sense — `Forward` walks along the edge's parameter — as a
/// face's loops are written counter-clockwise about its surface's normal
/// (`docs/DATA-MODEL.md` §Orientation); [`rewrite`] turns it into the
/// effective walk the builder takes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct StoredUse {
    /// The edge: kept by id, or one of the rewrite's own.
    pub edge: EdgeKey,
    /// Along or against its parameter.
    pub orientation: Orientation,
    /// The pcurve of this use on the face's surface.
    pub pcurve: Curve2Id,
}

/// A face a rewrite adds beside the operand's own.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct AddedFace {
    /// The operand shell it joins, by index in the body's stored order.
    pub shell: usize,
    /// Its surface.
    pub surface: SurfaceId,
    /// `Forward` when the surface's normal is the outward one.
    pub orientation: Orientation,
    /// Its loops in stored order.
    pub loops: Vec<Vec<StoredUse>>,
    /// Its tolerance.
    pub tolerance: f64,
}

/// A rewrite of one operand (ADR-0007): its faces kept whole by id
/// unless replaced by new loops, its edges and vertices kept by id unless
/// a new edge is a piece of one or a face's new loops no longer reach
/// them, plus the faces the operation adds. What a blend hands
/// [`rewrite`], the second producer of an assembly after the boolean's
/// pieces.
#[derive(Debug, Clone, PartialEq, Default)]
pub(crate) struct Rewrite {
    /// The new vertices, addressed by `VertexKey::New`.
    pub vertices: Vec<VertexSpec>,
    /// The new edges, addressed by `EdgeKey::New`, each with the operand
    /// edge it is a piece of, when it is one — a corner edge shortened
    /// by a trim — and `None` for an edge the operation makes.
    pub edges: Vec<(EdgeSpec, Option<EdgeId>)>,
    /// The operand faces replaced, each by its new loops in stored order;
    /// the shell's use and the tolerance stay the face's, and the surface
    /// too unless [`Rewrite::surfaces`] names another.
    pub faces: BTreeMap<FaceId, Vec<Vec<StoredUse>>>,
    /// The surface a replaced face moves to, where it is not its own — an
    /// offset's moved face. A face named here and not in
    /// [`Rewrite::faces`] is not replaced.
    pub surfaces: BTreeMap<FaceId, SurfaceId>,
    /// The operand vertex a new vertex stands for, by its index in
    /// [`Rewrite::vertices`] — a vertex an offset moves: the operand
    /// vertex is `Modified` into it rather than `Deleted`.
    pub vertex_parents: BTreeMap<usize, VertexId>,
    /// The faces added, each appended to its shell after the operand's
    /// own, in this order.
    pub added: Vec<AddedFace>,
}

/// What [`rewrite`] built: the body, the ids behind the rewrite's new
/// vertices, edges and added faces, and the generic half
/// of the provenance — every operand entity kept, `Modified` into its
/// replacement or piece, or `Deleted`; every shell and the body
/// `Modified` one-to-one. The caller adds the records only it knows,
/// the `Generated` relations of the entities it made.
pub(crate) struct Rewritten {
    /// The body built.
    pub body: Body,
    /// The id of `Rewrite::vertices[i]`, at `i`.
    pub vertices: Vec<VertexId>,
    /// The id of `Rewrite::edges[i]`, at `i`.
    pub edges: Vec<EdgeId>,
    /// The id of `Rewrite::added[i]`, at `i`.
    pub added: Vec<FaceId>,
    /// The generic provenance, above.
    pub provenance: Provenance,
}

/// Assembles `body` with `rewrite` applied — every face kept by id
/// unless replaced, the added faces after each shell's own — finishes
/// it as a solid, and writes the generic provenance (ADR-0004's keep-by-
/// id rule, ADR-0007): an operand vertex, edge or face still in the
/// result is kept and unrecorded; an edge with pieces among the new
/// edges is `Modified` into them, a vertex into the new vertices that
/// name it their parent, a replaced face into its replacement;
/// anything else of the operand that is gone is `Deleted`; each shell is
/// `Modified` into the shell built from it and the body into the result.
/// The output is verified as every operation's is. Errors: the builder's
/// refusal as [`OpError::Internal`], an id that does not resolve.
pub(crate) fn rewrite(m: &mut Model, body: Body, rewrite: Rewrite) -> Result<Rewritten, OpError> {
    let precision = m.precision();
    let closure = m.closure(body)?;
    let shells = m.shells(body)?;
    let mut assembly = Assembly {
        vertices: rewrite.vertices,
        edges: rewrite.edges.iter().map(|(spec, _)| *spec).collect(),
        shells: Vec::with_capacity(shells.len()),
    };
    let stored_to_spec =
        |orientation: Orientation, loops: &[Vec<StoredUse>]| -> Vec<Vec<UseSpec>> {
            loops
                .iter()
                .map(|l| {
                    effective_uses(
                        orientation,
                        l.iter().map(|u| (u.edge, u.orientation, u.pcurve)),
                    )
                    .into_iter()
                    .map(|(edge, orientation, pcurve)| UseSpec {
                        edge,
                        orientation,
                        pcurve,
                    })
                    .collect()
                })
                .collect()
        };
    // Where each replaced and added face sits in the assembly's shells.
    let mut replaced_at: BTreeMap<FaceId, (usize, usize)> = BTreeMap::new();
    let mut added_at: Vec<(usize, usize)> = Vec::with_capacity(rewrite.added.len());
    for (s, shell) in shells.iter().enumerate() {
        let entity = m.shell(shell.id)?;
        let mut faces: Vec<FaceSpec> = Vec::with_capacity(entity.faces().len());
        for face_use in entity.faces() {
            let face = face_use.oriented_by(shell.orientation);
            match rewrite.faces.get(&face.id) {
                Some(loops) => {
                    let old = m.face(face.id)?;
                    replaced_at.insert(face.id, (s, faces.len()));
                    faces.push(FaceSpec::New {
                        surface: rewrite
                            .surfaces
                            .get(&face.id)
                            .copied()
                            .unwrap_or(old.surface()),
                        orientation: face.orientation,
                        loops: stored_to_spec(face.orientation, loops),
                        tolerance: old.tolerance(),
                    });
                }
                None => faces.push(FaceSpec::Keep(face)),
            }
        }
        assembly.shells.push(faces);
    }
    for added in &rewrite.added {
        let Some(shell) = assembly.shells.get_mut(added.shell) else {
            return Err(OpError::Internal(Fault::Invariant {
                what: "the shell an added face joins",
            }));
        };
        added_at.push((added.shell, shell.len()));
        shell.push(FaceSpec::New {
            surface: added.surface,
            orientation: added.orientation,
            loops: stored_to_spec(added.orientation, &added.loops),
            tolerance: added.tolerance,
        });
    }

    let (builder, slots) = Builder::assemble(m, precision.default_tolerance, assembly)?;
    let built = builder.finish(m, BodyKind::Solid)?;
    let id_at = |(s, i): (usize, usize)| -> Result<FaceId, OpError> {
        slots
            .faces
            .get(s)
            .and_then(|shell| shell.get(i))
            .and_then(|slot| built.faces.get(slot))
            .copied()
            .ok_or(OpError::Internal(Fault::Invariant {
                what: "the slot of a face spec",
            }))
    };
    let vertices: Vec<VertexId> = slots
        .vertices
        .iter()
        .map(|slot| built.vertices[slot])
        .collect();
    let edges: Vec<EdgeId> = slots.edges.iter().map(|slot| built.edges[slot]).collect();
    let mut faces: BTreeMap<FaceId, FaceId> = BTreeMap::new();
    for (&old, &at) in &replaced_at {
        faces.insert(old, id_at(at)?);
    }
    let mut added: Vec<FaceId> = Vec::with_capacity(added_at.len());
    for &at in &added_at {
        added.push(id_at(at)?);
    }

    // The generic provenance: kept is unrecorded, a piece or a
    // replacement is `Modified`, the rest of what is gone is `Deleted`.
    let out = m.closure(built.body)?;
    let mut p = Provenance::new();
    let mut images_of: BTreeMap<VertexId, Vec<VertexId>> = BTreeMap::new();
    for (&k, &parent) in &rewrite.vertex_parents {
        let Some(&id) = vertices.get(k) else {
            return Err(OpError::Internal(Fault::Invariant {
                what: "the new vertex a parent names",
            }));
        };
        images_of.entry(parent).or_default().push(id);
    }
    for &v in &closure.vertices {
        if out.vertices.binary_search(&v).is_ok() {
            continue;
        }
        match images_of.get(&v) {
            Some(images) => {
                for &image in images {
                    p.add_modified(forward(v), forward(image));
                }
            }
            None => p.add_deleted(forward(v)),
        }
    }
    let mut pieces_of: BTreeMap<EdgeId, Vec<EdgeId>> = BTreeMap::new();
    for ((_, parent), &id) in rewrite.edges.iter().zip(&edges) {
        if let Some(parent) = parent {
            pieces_of.entry(*parent).or_default().push(id);
        }
    }
    for &e in &closure.edges {
        if out.edges.binary_search(&e).is_ok() {
            continue;
        }
        match pieces_of.get(&e) {
            Some(pieces) => {
                for &piece in pieces {
                    p.add_modified(forward(e), forward(piece));
                }
            }
            None => p.add_deleted(forward(e)),
        }
    }
    for &f in &closure.faces {
        if out.faces.binary_search(&f).is_ok() {
            continue;
        }
        match faces.get(&f) {
            Some(&new) => p.add_modified(forward(f), forward(new)),
            None => p.add_deleted(forward(f)),
        }
    }
    for (old, &new) in shells.iter().zip(&built.shells) {
        p.add_modified(forward(old.id), forward(new));
    }
    p.add_modified(forward(body.id), forward(built.body.id));
    crate::verify(m, built.body)?;
    Ok(Rewritten {
        body: built.body,
        vertices,
        edges,
        added,
        provenance: p,
    })
}
