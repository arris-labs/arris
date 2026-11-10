//! Add the stripes, rings, crowded edges and shortened corner edges.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::pcurve_on;
use arris_math::{Interval, Meter, Tolerance};
use arris_topo::builder::{EdgeKey, EdgeSpec, VertexKey, VertexSpec};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{Curve2Id, EdgeId, FaceId, Model};

use super::super::ends::{EndKind, Pole};
use super::super::ring::{Ring, RingEnd, RingEnds, placed_uv, vertex_tolerance_of};
use super::super::rolled::{Rolled, RolledMade, add_crowded_edge, add_run_over};
use super::super::{Blend, degenerate, invariant};
use super::{
    BlendCtx, Draft, Drafted, Insertion, Joins, Made, Rederived, Shortened, SplitPiece, StripeMade,
    Tail, cut_once,
};
use crate::body_view::BodyView;
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild::{Rewrite, forward};

/// The stripes: four trim vertices, two contact edges and each end's arcs; at a pole, its vertex kept for both contacts and a degenerate edge for the arc.
pub(super) fn add_stripes(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    blends: &[Blend],
    joins: &Joins,
    drafted: &Drafted,
) -> Result<Vec<StripeMade>, OpError> {
    let (m, view, _, tol, _, meter) = ctx.parts();
    let Draft {
        rw,
        cuts,
        edits,
        crowded,
    } = draft;
    let Joins {
        miters,
        miter_at: _,
        cusps: _,
        corners,
        corner_at: _,
    } = joins;
    let (miter_made, corner_made) = (&drafted.miters, &drafted.corners);
    let mut made: Vec<StripeMade> = Vec::with_capacity(blends.len());
    for blend in blends {
        let mut vertices = [[0usize; 2]; 2];
        let mut crossings: Vec<usize> = Vec::new();
        for (end_index, end) in blend.ends.iter().enumerate() {
            match end {
                EndKind::Face(face_end) => {
                    for (k, slot) in vertices[end_index].iter_mut().enumerate() {
                        *slot = rw.vertex(VertexSpec::New {
                            point: face_end.points[k],
                            tolerance: vertex_tolerance_of(&blend.ends, k),
                        });
                    }
                }
                EndKind::Pole(pole) => {
                    let at = rw.vertex(match pole.remnant {
                        None => VertexSpec::Keep(pole.vertex),
                        Some(r) => VertexSpec::New {
                            point: r.point,
                            tolerance: r.tolerance,
                        },
                    });
                    vertices[end_index] = [at; 2];
                }
                EndKind::Miter { at, side } => {
                    let shared = miters[*at].shared[*side];
                    vertices[end_index][shared] = miter_made[*at].vertices[0];
                    vertices[end_index][1 - shared] = miter_made[*at].far(&miters[*at], *side);
                }
                EndKind::Corner { at, side } => {
                    for (k, slot) in vertices[end_index].iter_mut().enumerate() {
                        *slot = corner_made[*at].vertices[corners[*at].contact_point[*side][k]];
                    }
                }
            }
        }
        // A run-over cuts the contacts at its crossings, and its own edits
        // of the faces take the contacts' place (ADR-0071 §2).
        let rolled = match &blend.rolled {
            Some(rolled) => Some(add_run_over(
                m,
                view,
                rw,
                edits,
                rolled,
                vertices,
                crowded.contains(&rolled.run.edge),
            )?),
            None => None,
        };
        let mut contact_edges = [0usize; 2];
        for (k, contact) in blend.contacts.iter().enumerate() {
            if let Some(rolled) = &rolled {
                contact_edges[k] = rolled.contacts[k][0];
                continue;
            }
            // An edge is no wider than its vertices: a contact pinned to a
            // kink's junction (ADR-0075) widens the vertices it ends at.
            for end in vertices {
                if let VertexSpec::New { tolerance, .. } = &mut rw.vertices[end[k]] {
                    *tolerance = tolerance.max(contact.tolerance);
                }
            }
            contact_edges[k] = rw.edge(
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(contact.line.clone()),
                        range: contact.range,
                    },
                    start: VertexKey::New(vertices[0][k]),
                    end: VertexKey::New(vertices[1][k]),
                    tolerance: contact.tolerance,
                },
                None,
            );
        }
        let mut arcs: [Vec<usize>; 2] = Default::default();
        for (end_index, end) in blend.ends.iter().enumerate() {
            let face_end = match end {
                EndKind::Face(face_end) => face_end,
                EndKind::Pole(_) => {
                    // The collapsed row, across from one contact to the
                    // other at the kept vertex.
                    let at = VertexKey::New(vertices[end_index][0]);
                    arcs[end_index].push(rw.edge(
                        EdgeSpec::New {
                            geometry: EdgeGeometry::Degenerate {
                                range: Interval::UNIT,
                            },
                            start: at,
                            end: at,
                            tolerance: blend.stripe.tolerance,
                        },
                        None,
                    ));
                    continue;
                }
                EndKind::Miter { at, side } => {
                    arcs[end_index] = miter_made[*at].arcs(&miters[*at], *side);
                    continue;
                }
                EndKind::Corner { at, side } => {
                    arcs[end_index].push(corner_made[*at].arcs[*side]);
                    continue;
                }
            };
            // The points of the end in its order: the first trim point, a
            // fan's crossings, the other trim point; each crossing cuts its
            // extra edge (ADR-0043 §5).
            let mut along = vec![vertices[end_index][0]];
            for crossing in &face_end.crossings {
                let v = rw.vertex(VertexSpec::New {
                    point: crossing.point,
                    tolerance: crossing.tolerance,
                });
                cut_once(cuts, &crossing.trim, v)?;
                crossings.push(v);
                along.push(v);
            }
            along.push(vertices[end_index][1]);
            for (k, trim) in face_end.trims.iter().enumerate() {
                cut_once(cuts, trim, vertices[end_index][k])?;
            }
            // A flush fan keeps the second corner edge's part from the
            // vertex to its trim, an edge between the first and the last
            // piece (ADR-0082 §2).
            let mut tail = None;
            if let Some(last) = face_end.flush {
                let first = face_end.pieces[0].face;
                let (for_first, for_last) = flush_tail(
                    m,
                    view,
                    rw,
                    face_end,
                    vertices[end_index][1],
                    [first, last],
                    tol,
                    meter,
                )?;
                edits
                    .entry(last)
                    .or_default()
                    .insert
                    .insert(face_end.vertex, for_last);
                tail = Some(for_first);
            }
            for (i, piece) in face_end.pieces.iter().enumerate() {
                let (from, to) = (along[i], along[i + 1]);
                let (first, second) = if piece.arc.lo_first {
                    (from, to)
                } else {
                    (to, from)
                };
                let arc = rw.edge(
                    EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(piece.arc.curve.clone()),
                            range: piece.arc.range,
                        },
                        start: VertexKey::New(first),
                        end: VertexKey::New(second),
                        tolerance: piece.arc.tolerance,
                    },
                    None,
                );
                arcs[end_index].push(arc);
                // The edge cut at the arc's start: the first corner edge,
                // or the extra edge before this piece.
                let edge_at_lo = match i.checked_sub(1) {
                    None => face_end.trims[0].edge,
                    Some(j) => face_end.crossings[j].trim.edge,
                };
                let pcurve = m.add_curve2(piece.arc.on_face.clone());
                edits.entry(piece.face).or_default().insert.insert(
                    face_end.vertex,
                    Insertion {
                        arc: EdgeKey::New(arc),
                        pcurve,
                        lo_first: piece.arc.lo_first,
                        edge_at_lo,
                        splits: (!face_end.stays.is_empty())
                            .then_some(face_end.trims.map(|trim| trim.edge)),
                        tail,
                    },
                );
            }
        }
        // Where the stripe stops short of a vertex, the edge's remnant runs
        // on to it, and on each face the edge is the remnants and the
        // contact between them, in the edge's direction (ADR-0074).
        let remnants = remnant_edges(m, rw, blend, vertices)?;
        if rolled.is_some() && remnants.iter().any(Option::is_some) {
            return Err(degenerate(
                vec![forward(blend.stripe.edge)],
                Reason::Blend(BlendReason::TangentChain),
            ));
        }
        for (k, contact) in blend.contacts.iter().enumerate() {
            if rolled.is_some() {
                continue;
            }
            let pcurve = m.add_curve2(contact.on_face.clone());
            let edit = edits.entry(contact.face).or_default();
            let contact_use = (EdgeKey::New(contact_edges[k]), pcurve);
            if remnants.iter().all(Option::is_none) {
                edit.replace.insert(blend.stripe.edge, contact_use);
                continue;
            }
            let own = (view.uses.get(&blend.stripe.edge).into_iter().flatten())
                .find(|u| u.face == contact.face)
                .ok_or(invariant("the blended edge's use by its face"))?
                .pcurve;
            let [lo, hi] = remnants.map(|r| r.map(|r| SplitPiece::Use(EdgeKey::New(r), own)));
            let pieces = lo
                .into_iter()
                .chain([SplitPiece::Use(contact_use.0, contact_use.1)]);
            edit.split
                .insert(blend.stripe.edge, pieces.chain(hi).collect());
        }
        made.push(StripeMade {
            vertices,
            crossings,
            contacts: contact_edges,
            arcs,
            added: 0,
            rolled,
        });
    }
    Ok(made)
}

/// A flush fan's kept edge (ADR-0082 §2): the second corner edge's curve from
/// the vertex to its trim point `p2` (the rewrite's vertex), added to `rw`
/// with the corner edge as its parent. Returns the first piece's tail, whose
/// pcurve is derived on its plane, and the last piece's insertion, which is
/// the edge itself at the junction, on the corner edge's own pcurve.
#[allow(clippy::too_many_arguments)]
fn flush_tail(
    m: &mut Model,
    view: &BodyView,
    rw: &mut Rewrite,
    end: &super::super::ends::End,
    p2: usize,
    [first, last]: [FaceId; 2],
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<(Tail, Insertion), OpError> {
    let trim = end.trims[1];
    let entity = *m.edge(trim.edge)?;
    let (curve, range) = entity
        .curve()
        .ok_or(invariant("a flush fan's corner edge curve"))?;
    let kept = if trim.cuts_lo {
        Interval::new(range.lo(), trim.t)
    } else {
        Interval::new(trim.t, range.hi())
    }
    .map_err(|_| invariant("a flush fan's corner edge cut inside its range"))?;
    let (start, finish) = if trim.cuts_lo {
        (VertexKey::Kept(end.vertex), VertexKey::New(p2))
    } else {
        (VertexKey::New(p2), VertexKey::Kept(end.vertex))
    };
    let key = EdgeKey::New(rw.edge(
        EdgeSpec::New {
            geometry: EdgeGeometry::Curve { curve, range: kept },
            start,
            end: finish,
            tolerance: entity.tolerance(),
        },
        Some(trim.edge),
    ));
    let surface = m.surface(m.face(first)?.surface())?.clone();
    let on_first = pcurve_on(
        m.curve(curve)?,
        kept,
        &surface,
        Tolerance::new(entity.tolerance(), tol.angular),
        meter,
    )
    .map_err(fault_of)?;
    let on_first = m.add_curve2(on_first);
    let on_last = view
        .uses
        .get(&trim.edge)
        .and_then(|u| u.iter().find(|u| u.face == last))
        .ok_or(invariant("a flush fan's corner edge on its last piece"))?
        .pcurve;
    let to_vertex = !trim.cuts_lo;
    Ok((
        Tail {
            edge: key,
            pcurve: on_first,
            to_vertex,
        },
        Insertion {
            arc: key,
            pcurve: on_last,
            lo_first: to_vertex,
            edge_at_lo: trim.edge,
            splits: None,
            tail: None,
        },
    ))
}

/// The blended edge's remnant at each end where its stripe stops short of
/// the vertex (ADR-0074), added to `rw`: the edge's curve from the vertex
/// to the cut, kept with the edge's tolerance and recorded against it, the
/// cut being the stripe's end vertex `vertices[end][0]`. Each is `None`
/// where the end is not such a pole.
fn remnant_edges(
    m: &Model,
    rw: &mut Rewrite,
    blend: &Blend,
    vertices: [[usize; 2]; 2],
) -> Result<[Option<usize>; 2], OpError> {
    let edge = blend.stripe.edge;
    let entity = *m.edge(edge)?;
    let mut out = [None; 2];
    for (end_index, end) in blend.ends.iter().enumerate() {
        let EndKind::Pole(Pole {
            remnant: Some(r), ..
        }) = end
        else {
            continue;
        };
        let (curve, old) = entity
            .curve()
            .ok_or(invariant("the blended edge's curve"))?;
        let (lo, hi, start, end) = if end_index == 0 {
            (
                old.lo(),
                r.t,
                VertexKey::Kept(entity.start()),
                VertexKey::New(vertices[0][0]),
            )
        } else {
            (
                r.t,
                old.hi(),
                VertexKey::New(vertices[1][0]),
                VertexKey::Kept(entity.end()),
            )
        };
        let range = Interval::new(lo, hi).map_err(|_| invariant("a remnant inside its edge"))?;
        out[end_index] = Some(rw.edge(
            EdgeSpec::New {
                geometry: EdgeGeometry::Curve { curve, range },
                start,
                end,
                tolerance: entity.tolerance().max(r.tolerance),
            },
            Some(edge),
        ));
    }
    Ok(out)
}

/// The rings: a closed edge's two vertices, two contact circles, the blend's seam and the cylinder's seam cut; an open arc's four vertices, two contact arcs and two ends, each inserted into its face across with the corner edges cut.
pub(super) fn add_rings(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    rings: &[Ring],
    joins: &Joins,
    drafted: &Drafted,
) -> Result<Vec<Made>, OpError> {
    let m = &mut *ctx.m;
    let view = &ctx.view;
    let Draft {
        rw,
        cuts,
        edits,
        crowded,
    } = draft;
    let Joins {
        miters, miter_at, ..
    } = joins;
    let miter_made = &drafted.miters;
    let mut ring_made: Vec<Made> = Vec::with_capacity(rings.len());
    for r in rings {
        let mut vertices = [[0usize; 2]; 2];
        match &r.ends {
            RingEnds::Seam(seam) => {
                for (slot, contact) in vertices[0].iter_mut().zip(&r.contacts) {
                    *slot = rw.vertex(VertexSpec::New {
                        point: contact.points[0],
                        tolerance: seam.vertex_tolerance,
                    });
                }
                vertices[1] = vertices[0];
            }
            RingEnds::Open(ends) => {
                for (j, end) in ends.iter().enumerate() {
                    match end {
                        RingEnd::Face(end) => {
                            for (c, contact) in r.contacts.iter().enumerate() {
                                vertices[j][c] = rw.vertex(VertexSpec::New {
                                    point: contact.points[j],
                                    tolerance: end.vertex_tolerance[c],
                                });
                            }
                        }
                        RingEnd::Junction(vertex) => {
                            let at = *miter_at
                                .get(vertex)
                                .ok_or(invariant("a junction at the ring's end"))?;
                            let side = usize::from(miters[at].edges[0] != r.edge);
                            let shared = miters[at].shared[side];
                            vertices[j][shared] = miter_made[at].vertices[0];
                            vertices[j][1 - shared] = miter_made[at].vertices[1];
                        }
                    }
                }
            }
        }
        // A run-over cuts the contacts into pieces and leaves the torus
        // without a seam edge (ADR-0071 §2).
        let rolled_made = match &r.rolled {
            Some(rolled) => Some(add_run_over(
                m,
                view,
                rw,
                edits,
                rolled,
                vertices,
                crowded.contains(&rolled.run.edge),
            )?),
            None => None,
        };
        let mut contact_edges = [0usize; 2];
        if let Some(rolled) = &rolled_made {
            contact_edges = [rolled.contacts[0][0], rolled.contacts[1][0]];
        } else {
            for (c, contact) in r.contacts.iter().enumerate() {
                contact_edges[c] = rw.edge(
                    EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(contact.curve.clone()),
                            range: contact.range,
                        },
                        start: VertexKey::New(vertices[0][c]),
                        end: VertexKey::New(vertices[1][c]),
                        tolerance: r.tolerance,
                    },
                    None,
                );
                let pcurve = m.add_curve2(contact.on_face.clone());
                edits
                    .entry(contact.face)
                    .or_default()
                    .replace
                    .insert(r.edge, (EdgeKey::New(contact_edges[c]), pcurve));
            }
        }
        let mut arcs = [0usize; 2];
        match &r.ends {
            RingEnds::Seam(seam) => {
                if rolled_made.is_none() {
                    arcs = [rw.edge(
                        EdgeSpec::New {
                            geometry: EdgeGeometry::Curve {
                                curve: m.add_curve(seam.curve.clone()),
                                range: seam.range,
                            },
                            start: VertexKey::New(vertices[0][0]),
                            end: VertexKey::New(vertices[0][1]),
                            tolerance: r.tolerance,
                        },
                        None,
                    ); 2];
                }
                for (trim, by) in &seam.cuts {
                    cut_once(cuts, trim, vertices[0][*by])?;
                }
            }
            RingEnds::Open(ends) => {
                for (j, end) in ends.iter().enumerate() {
                    let end = match end {
                        RingEnd::Face(end) => end,
                        RingEnd::Junction(vertex) => {
                            arcs[j] = miter_made[miter_at[vertex]].edge;
                            continue;
                        }
                    };
                    let (first, second) = if end.lo_first { (0, 1) } else { (1, 0) };
                    arcs[j] = rw.edge(
                        EdgeSpec::New {
                            geometry: EdgeGeometry::Curve {
                                curve: m.add_curve(end.curve.clone()),
                                range: end.range,
                            },
                            start: VertexKey::New(vertices[j][first]),
                            end: VertexKey::New(vertices[j][second]),
                            tolerance: end.tolerance,
                        },
                        None,
                    );
                    for (c, trim) in end.trims.iter().enumerate() {
                        cut_once(cuts, trim, vertices[j][c])?;
                    }
                    let pcurve = m.add_curve2(end.on_face.clone());
                    edits.entry(end.face).or_default().insert.insert(
                        end.vertex,
                        Insertion {
                            arc: EdgeKey::New(arcs[j]),
                            pcurve,
                            lo_first: end.lo_first,
                            edge_at_lo: end.trims[0].edge,
                            splits: None,
                            tail: None,
                        },
                    );
                }
            }
        }
        ring_made.push(Made {
            vertices,
            contacts: contact_edges,
            arcs,
            added: 0,
            rolled: rolled_made,
        });
    }
    Ok(ring_made)
}

/// The edges that two or more run-overs cross.
pub(super) fn crowded_edges(blends: &[Blend], rings: &[Ring]) -> BTreeSet<EdgeId> {
    let mut seen: BTreeSet<EdgeId> = BTreeSet::new();
    let mut crowded: BTreeSet<EdgeId> = BTreeSet::new();
    let runs = (blends.iter().filter_map(|b| b.rolled.as_deref()))
        .chain(rings.iter().filter_map(|r| r.rolled.as_deref()));
    for rolled in runs {
        if !seen.insert(rolled.run.edge) {
            crowded.insert(rolled.run.edge);
        }
    }
    crowded
}

/// Each crowded edge cut at the crossings of all the run-overs over it
/// (ADR-0071 §2), once, in the edge's own order.
pub(super) fn add_crowded_edges(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    blends: &[Blend],
    rings: &[Ring],
    made: &mut Drafted,
) -> Result<(), OpError> {
    if draft.crowded.is_empty() {
        return Ok(());
    }
    let m = &mut *ctx.m;
    let view = &ctx.view;
    let Draft {
        rw, edits, crowded, ..
    } = draft;
    let mut by_edge: BTreeMap<EdgeId, Vec<(&Rolled, &mut RolledMade)>> = BTreeMap::new();
    let stripes = (blends.iter().zip(made.stripes.iter_mut()))
        .filter_map(|(b, mk)| Some((b.rolled.as_deref()?, mk.rolled.as_mut()?)));
    let rings = (rings.iter().zip(made.rings.iter_mut()))
        .filter_map(|(r, mk)| Some((r.rolled.as_deref()?, mk.rolled.as_mut()?)));
    for (rolled, rolled_made) in stripes.chain(rings) {
        if crowded.contains(&rolled.run.edge) {
            by_edge
                .entry(rolled.run.edge)
                .or_default()
                .push((rolled, rolled_made));
        }
    }
    for runs in by_edge.values_mut() {
        add_crowded_edge(m, view, rw, edits, runs)?;
    }
    Ok(())
}

/// The corner edges shortened or lengthened, in id order; a lengthened edge's pcurves derived again over its new range (ADR-0038 §4), by the edge and the pcurve each replaces.
pub(super) fn shorten_edges(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
) -> Result<(Shortened, Rederived), OpError> {
    let (m, view, _, tol, _, meter) = ctx.parts();
    let Draft { rw, cuts, .. } = draft;
    let mut shortened: BTreeMap<EdgeId, EdgeKey> = BTreeMap::new();
    let mut rederived: BTreeMap<(EdgeId, Curve2Id), Curve2Id> = BTreeMap::new();
    for (&edge, cut) in &*cuts {
        let entity = *m.edge(edge)?;
        let Some((curve, old)) = entity.curve() else {
            return Err(invariant("a corner edge's curve"));
        };
        let lo = cut.lo.map_or(old.lo(), |(_, t)| t);
        let hi = cut.hi.map_or(old.hi(), |(_, t)| t);
        let range = Interval::new(lo, hi)
            .map_err(|_| degenerate(vec![forward(edge)], Reason::Blend(BlendReason::TooLarge)))?;
        // Placed where the old pcurve still runs: at the end lengthened.
        let anchor = if lo < old.lo() {
            Some(old.lo())
        } else if hi > old.hi() {
            Some(old.hi())
        } else {
            None
        };
        if let Some(anchor) = anchor {
            let edge_tol = Tolerance::new(entity.tolerance(), tol.angular);
            for u in view.uses.get(&edge).into_iter().flatten() {
                let surface = m.surface(m.face(u.face)?.surface())?;
                let fresh = pcurve_on(m.curve(curve)?, range, surface, edge_tol, meter)
                    .map_err(fault_of)?;
                let fresh = placed_uv(fresh, anchor, m.curve2(u.pcurve)?.point(anchor));
                let id = m.add_curve2(fresh);
                rederived.insert((edge, u.pcurve), id);
            }
        }
        let key = rw.edge(
            EdgeSpec::New {
                geometry: EdgeGeometry::Curve { curve, range },
                start: cut
                    .lo
                    .map_or(VertexKey::Kept(entity.start()), |(v, _)| VertexKey::New(v)),
                end: cut
                    .hi
                    .map_or(VertexKey::Kept(entity.end()), |(v, _)| VertexKey::New(v)),
                tolerance: entity.tolerance(),
            },
            Some(edge),
        );
        shortened.insert(edge, EdgeKey::New(key));
    }
    Ok((shortened, rederived))
}
