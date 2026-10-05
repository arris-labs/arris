//! The shell's assembly: the body's faces kept, the offset's moved faces
//! beside them as the second skin, each loop of an opening a rim face
//! between the body's loop and the skin's, the void's skin a shell of its
//! own; then the provenance and the checker at `Level::Full`.

use std::collections::BTreeSet;

use arris_math::Meter;
use arris_topo::builder::{Assembly, Builder, EdgeKey, EdgeSpec, FaceSpec};
use arris_topo::entity::BodyKind;
use arris_topo::{Body, Face as FaceHandle, FaceId, Model, Provenance};

use super::{ShellSide, rim};
use crate::body_view::BodyView;
use crate::error::{Fault, OpError, Reason, ShellReason};
use crate::offset;
use crate::rebuild::{forward, stored_to_spec};

/// Where a face spec came from, for the provenance.
#[derive(Debug, Clone, Copy)]
enum Role {
    /// A wall of the body, kept by id.
    Kept,
    /// The skin's copy of a wall.
    Skin(FaceId),
    /// A rim face of an opening.
    Rim(FaceId),
}

/// The shell of `body` open at `open`, its `walls` offset by `distance`
/// (negative inward), built and checked.
pub(super) fn build(
    m: &mut Model,
    body: Body,
    open: &BTreeSet<FaceId>,
    walls: &BTreeSet<FaceId>,
    distance: f64,
    side: ShellSide,
    meter: &mut Meter<'_>,
) -> Result<(Body, Provenance), OpError> {
    let view = BodyView::of(m, body)?;
    let offset = offset::pieces(m, body, walls, distance, meter)?;
    if let Some(&dragged) = open.iter().find(|f| offset.moved.contains(f)) {
        return Err(OpError::Degenerate {
            entities: vec![forward(dragged)],
            reason: Reason::Shell(ShellReason::OpeningDragged),
        });
    }
    let rw = offset.rewrite;
    let missing = |what| OpError::Internal(Fault::Invariant { what });

    let tol = m.precision().tolerance();
    let mut pieces = rim::Pieces::new(rw.edges.len());
    // The openings whose rim cancels whole: the skin's loop on them is the
    // body's.
    let mut vanished: Vec<FaceId> = Vec::new();
    let mut assembly = Assembly {
        vertices: rw.vertices.clone(),
        edges: Vec::new(),
        shells: Vec::new(),
    };
    // The role of each face spec, shell by shell, and the body shell each
    // result shell keeps the faces of (`None` for a void's skin).
    let mut roles: Vec<Vec<Role>> = Vec::new();
    let mut origin: Vec<Option<usize>> = Vec::new();
    let mut voids: Vec<(Vec<FaceSpec>, Vec<Role>, Option<usize>)> = Vec::new();
    for (s, body_shell) in m.shells(body)?.into_iter().enumerate() {
        let mut kept: Vec<(FaceSpec, Role)> = Vec::new();
        let mut rims: Vec<(FaceSpec, Role)> = Vec::new();
        let mut skin: Vec<(FaceSpec, Role)> = Vec::new();
        let mut has_opening = false;
        for face_use in m.shell(body_shell.id)?.faces() {
            let face = face_use.oriented_by(body_shell.orientation);
            let entity = m.face(face.id)?;
            if open.contains(&face.id) {
                has_opening = true;
                let faces = rim::faces(m, &rw, &view, open, face.id, side, &mut pieces, tol)?;
                if faces.is_empty() {
                    vanished.push(face.id);
                }
                for loops in faces {
                    rims.push((
                        FaceSpec::New {
                            surface: entity.surface(),
                            orientation: face.orientation,
                            loops: stored_to_spec(face.orientation, &loops),
                            tolerance: entity.tolerance(),
                        },
                        Role::Rim(face.id),
                    ));
                }
                continue;
            }
            let new_loops = rw
                .faces
                .get(&face.id)
                .ok_or(missing("a wall's loops on the skin"))?;
            let surface = *rw
                .surfaces
                .get(&face.id)
                .ok_or(missing("a wall's surface on the skin"))?;
            let (wall, inner) = match side {
                ShellSide::Inward => (face.orientation, face.orientation.flipped()),
                ShellSide::Outward => (face.orientation.flipped(), face.orientation),
            };
            kept.push((FaceSpec::Keep(FaceHandle::new(face.id, wall)), Role::Kept));
            skin.push((
                FaceSpec::New {
                    surface,
                    orientation: inner,
                    loops: stored_to_spec(inner, new_loops),
                    tolerance: entity.tolerance(),
                },
                Role::Skin(face.id),
            ));
        }
        if !has_opening {
            // A closed void: the skin is a shell of its own, inside the
            // body's for an inward wall, around it for an outward one.
            let (outer, inner) = match side {
                ShellSide::Inward => (kept, skin),
                ShellSide::Outward => (skin, kept),
            };
            // The body's shell is `Modified` into the one holding its kept
            // faces; the skin's shell is `Generated` from the body.
            let (outer_keeps, void_keeps) = match side {
                ShellSide::Inward => (Some(s), None),
                ShellSide::Outward => (None, Some(s)),
            };
            let (specs, r): (Vec<_>, Vec<_>) = outer.into_iter().unzip();
            assembly.shells.push(specs);
            roles.push(r);
            origin.push(outer_keeps);
            let (specs, r): (Vec<_>, Vec<_>) = inner.into_iter().unzip();
            voids.push((specs, r, void_keeps));
        } else {
            let (specs, r): (Vec<_>, Vec<_>) = kept.into_iter().chain(rims).chain(skin).unzip();
            assembly.shells.push(specs);
            roles.push(r);
            origin.push(Some(s));
        }
    }
    for (specs, r, keeps) in voids {
        assembly.shells.push(specs);
        roles.push(r);
        origin.push(keeps);
    }

    // Every edge a face uses — the skin's copy of an edge two openings
    // share is in none, its pieces stand in for it — addressed afresh.
    let all: Vec<EdgeSpec> = rw
        .edges
        .iter()
        .map(|(spec, _)| *spec)
        .chain(pieces.list.iter().map(|piece| piece.spec))
        .collect();
    let slot_of = compact(&mut assembly, &all);
    let at = |k: usize| slot_of.get(k).copied().flatten();

    let precision = m.precision();
    let body_shells = m.shells(body)?;
    let closure = m.closure(body)?;
    let (builder, slots) = Builder::assemble(m, precision.default_tolerance, assembly)?;
    let built = builder.finish(m, BodyKind::Solid)?;

    let mut p = Provenance::new();
    let out = m.closure(built.body)?;
    for (k, &parent) in &rw.vertex_parents {
        let slot = slots
            .vertices
            .get(*k)
            .ok_or(missing("a skin vertex's slot"))?;
        let id = *built.vertices.get(slot).ok_or(missing("a skin vertex"))?;
        if out.vertices.binary_search(&parent).is_ok() {
            p.add_modified(forward(parent), forward(parent));
        }
        p.add_generated(forward(parent), forward(id));
    }
    for (k, (_, parent)) in rw.edges.iter().enumerate() {
        let Some(parent) = *parent else {
            return Err(missing("the wall edge a skin edge copies"));
        };
        let Some(k) = at(k) else { continue };
        let slot = slots.edges.get(k).ok_or(missing("a skin edge's slot"))?;
        let id = *built.edges.get(slot).ok_or(missing("a skin edge"))?;
        if out.edges.binary_search(&parent).is_ok() {
            p.add_modified(forward(parent), forward(parent));
        }
        p.add_generated(forward(parent), forward(id));
    }
    for (i, piece) in pieces.list.iter().enumerate() {
        let k = at(rw.edges.len() + i).ok_or(missing("a rim piece used"))?;
        let slot = slots.edges.get(k).ok_or(missing("a rim piece's slot"))?;
        let id = *built.edges.get(slot).ok_or(missing("a rim piece"))?;
        if piece.on_parent {
            p.add_modified(forward(piece.parent), forward(id));
        } else {
            p.add_generated(forward(piece.parent), forward(id));
        }
    }
    for (shell_roles, shell_slots) in roles.iter().zip(&slots.faces) {
        for (role, slot) in shell_roles.iter().zip(shell_slots) {
            let id = *built
                .faces
                .get(slot)
                .ok_or(missing("a face of the shell"))?;
            match *role {
                Role::Kept => p.add_modified(forward(id), forward(id)),
                Role::Skin(wall) => p.add_generated(forward(wall), forward(id)),
                Role::Rim(opening) => p.add_modified(forward(opening), forward(id)),
            }
        }
    }
    for &f in &vanished {
        p.add_deleted(forward(f));
    }
    for &v in &closure.vertices {
        if out.vertices.binary_search(&v).is_err() && p.modified_from(forward(v)).is_empty() {
            p.add_deleted(forward(v));
        }
    }
    for &e in &closure.edges {
        if out.edges.binary_search(&e).is_err() && p.modified_from(forward(e)).is_empty() {
            p.add_deleted(forward(e));
        }
    }
    for (k, &built_shell) in built.shells.iter().enumerate() {
        match origin.get(k).copied().flatten() {
            Some(s) => p.add_modified(forward(body_shells[s].id), forward(built_shell)),
            None => p.add_generated(forward(body.id), forward(built_shell)),
        }
    }
    p.add_modified(forward(body.id), forward(built.body.id));
    offset::checked_full(m, built.body)?;
    Ok((built.body, p))
}

/// Puts into `assembly` the edges of `all` its faces use, in order, each
/// use renumbered, and returns the new index of each of `all` — `None`
/// for one no face uses.
fn compact(assembly: &mut Assembly, all: &[EdgeSpec]) -> Vec<Option<usize>> {
    let mut used = vec![false; all.len()];
    for spec in assembly.shells.iter().flatten() {
        if let FaceSpec::New { loops, .. } = spec {
            for u in loops.iter().flatten() {
                if let EdgeKey::New(k) = u.edge
                    && let Some(flag) = used.get_mut(k)
                {
                    *flag = true;
                }
            }
        }
    }
    let mut slot_of = Vec::with_capacity(all.len());
    for (spec, &is_used) in all.iter().zip(&used) {
        if is_used {
            slot_of.push(Some(assembly.edges.len()));
            assembly.edges.push(*spec);
        } else {
            slot_of.push(None);
        }
    }
    for spec in assembly.shells.iter_mut().flatten() {
        if let FaceSpec::New { loops, .. } = spec {
            for u in loops.iter_mut().flatten() {
                if let EdgeKey::New(k) = u.edge
                    && let Some(Some(to)) = slot_of.get(k)
                {
                    u.edge = EdgeKey::New(*to);
                }
            }
        }
    }
    slot_of
}
