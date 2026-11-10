//! Rewrite the operand faces and add the blend, corner and ring faces.

use std::collections::BTreeMap;

use arris_check::domain::bands;
use arris_geom::{Curve2, pcurve_ending_on};
use arris_math::{Interval, Meter, Point2, RELATIVE_ROUNDING, Tolerance};
use arris_topo::builder::{EdgeKey, EdgeSpec};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{CurveId, EdgeId, FaceId, Model, Orientation, VertexId};

use super::super::ends::EndKind;
use super::super::ring::{Ring, RingEnd, RingEnds};
use super::super::ring_rolled::add_ring_rolled_faces;
use super::super::rolled::add_rolled_faces;
use super::super::stripe::Section;
use super::super::{Blend, invariant};
use super::{BlendCtx, Draft, Drafted, Joins, Rederived, Shortened, SplitPiece};
use crate::error::{OpError, fault_of};
use crate::rebuild::{AddedFace, Rewrite, StoredUse, along};

/// Every touched face's loops rewritten, in the body's face order.
pub(super) fn rewrite_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    shortened: &Shortened,
    rederived: &Rederived,
) -> Result<(), OpError> {
    let (m, view, _, tol, samples, meter) = ctx.parts();
    let Draft { rw, edits, .. } = draft;
    for &face in &view.faces {
        let entity = m.face(face)?;
        let edit = edits.get(&face);
        let touched = edit.is_some()
            || entity
                .loops()
                .iter()
                .flat_map(|l| l.coedges())
                .any(|c| shortened.contains_key(&c.edge()));
        if !touched {
            continue;
        }
        let mut loops: Vec<Vec<StoredUse>> = Vec::with_capacity(entity.loops().len());
        // A run-over's gaps, by loop and the place in it (ADR-0071 §2).
        let mut gaps: Vec<(EdgeId, usize, usize)> = Vec::new();
        for l in entity.loops() {
            let mut uses: Vec<StoredUse> = Vec::with_capacity(l.coedges().len() + 1);
            // Where each coedge's use sits in `uses`.
            let mut at: Vec<usize> = Vec::with_capacity(l.coedges().len());
            let mut gapped: Vec<(usize, EdgeId)> = Vec::new();
            for c in l.coedges() {
                let id = c.edge();
                if let Some(pieces) = edit.and_then(|e| e.split.get(&id)) {
                    // The pieces walked the way the coedge walks the edge.
                    at.push(uses.len());
                    let forward = c.orientation() == Orientation::Forward;
                    let ordered: Vec<_> = if forward {
                        pieces.iter().collect()
                    } else {
                        pieces.iter().rev().collect()
                    };
                    for piece in ordered {
                        match *piece {
                            SplitPiece::Use(edge, pcurve) => uses.push(StoredUse {
                                edge,
                                orientation: c.orientation(),
                                pcurve,
                            }),
                            SplitPiece::Gap(by) => gapped.push((uses.len(), by)),
                        }
                    }
                    continue;
                }
                let (edge, pcurve) = match edit.and_then(|e| e.replace.get(&id)) {
                    Some(&(key, pcurve)) => (key, pcurve),
                    None => (
                        shortened.get(&id).copied().unwrap_or(EdgeKey::Kept(id)),
                        (rederived.get(&(id, c.pcurve())).copied()).unwrap_or(c.pcurve()),
                    ),
                };
                at.push(uses.len());
                uses.push(StoredUse {
                    edge,
                    orientation: c.orientation(),
                    pcurve,
                });
                let ce = m.edge(id)?;
                let junction = if c.orientation() == Orientation::Forward {
                    ce.end()
                } else {
                    ce.start()
                };
                if let Some(ins) = edit
                    .and_then(|e| e.insert.get(&junction))
                    .filter(|ins| ins.splits.is_none())
                {
                    // The arc runs from the end of the corner edge just
                    // walked to the start of the next.
                    let arrived_at_lo = id == ins.edge_at_lo;
                    let orientation = along(ins.lo_first == arrived_at_lo);
                    let arc = StoredUse {
                        edge: ins.arc,
                        orientation,
                        pcurve: ins.pcurve,
                    };
                    // A flush fan's first piece goes on to the vertex by
                    // the kept edge, before the arc where it arrives by an
                    // extra (ADR-0082 §2).
                    match ins.tail {
                        None => uses.push(arc),
                        Some(tail) => {
                            let kept = StoredUse {
                                edge: tail.edge,
                                orientation: along(arrived_at_lo == tail.to_vertex),
                                pcurve: tail.pcurve,
                            };
                            if arrived_at_lo {
                                uses.extend([arc, kept]);
                            } else {
                                uses.extend([kept, arc]);
                            }
                        }
                    }
                }
            }
            // The face across met twice: the stretch of the loop from the
            // corner edge leaving the surviving vertex to the one arriving
            // at it, closed by the arc, is a loop of its own; the rest
            // keeps the vertex with the edges that stay (ADR-0043 §4).
            let mut inner: Option<Vec<StoredUse>> = None;
            for (&vertex, ins) in edit.iter().flat_map(|e| &e.insert) {
                let Some(corners) = ins.splits else { continue };
                let ends = |i: usize| -> Result<(VertexId, VertexId), OpError> {
                    let c = l.coedges()[i];
                    let ce = m.edge(c.edge())?;
                    Ok(if c.orientation() == Orientation::Forward {
                        (ce.start(), ce.end())
                    } else {
                        (ce.end(), ce.start())
                    })
                };
                let (mut leaves, mut arrives) = (None, None);
                for i in 0..l.coedges().len() {
                    let (from, to) = ends(i)?;
                    if !corners.contains(&l.coedges()[i].edge()) {
                        continue;
                    }
                    if from == vertex {
                        leaves = Some(i);
                    }
                    if to == vertex {
                        arrives = Some(i);
                    }
                }
                let (Some(leaves), Some(arrives)) = (leaves, arrives) else {
                    continue;
                };
                let (first, last) = (at[leaves], at[arrives]);
                let (mut stretch, rest): (Vec<StoredUse>, Vec<StoredUse>) = if first <= last {
                    let stretch = uses[first..=last].to_vec();
                    let rest = [&uses[..first], &uses[last + 1..]].concat();
                    (stretch, rest)
                } else {
                    let stretch = [&uses[first..], &uses[..=last]].concat();
                    (stretch, uses[last + 1..first].to_vec())
                };
                let arrived_at_lo = l.coedges()[arrives].edge() == ins.edge_at_lo;
                stretch.push(StoredUse {
                    edge: ins.arc,
                    orientation: along(ins.lo_first == arrived_at_lo),
                    pcurve: ins.pcurve,
                });
                uses = rest;
                inner = Some(stretch);
            }
            if !gapped.is_empty() {
                if inner.is_some() || uses.is_empty() {
                    return Err(invariant("a run-over's gap in a loop of its own"));
                }
                // Gaps with no use between them, round the loop too, are one.
                for (g, by) in gapped {
                    let gap = (by, loops.len(), g % uses.len());
                    if !gaps.contains(&gap) {
                        gaps.push(gap);
                    }
                }
            }
            loops.push(uses);
            loops.extend(inner);
        }
        join_at_gaps(&mut loops, &gaps)?;
        end_on_kept_pcurves(m, rw, face, &mut loops, tol, samples, meter)?;
        rw.faces.insert(face, loops);
    }
    Ok(())
}

/// A loop being joined: a use, or the gap of a run-over.
enum Item {
    Use(StoredUse),
    Gap(EdgeId),
}

/// `F`'s loops with each run-over's gap joined into one (ADR-0071 §2):
/// the contact's loop runs to one crossing and stops, the loop that held
/// `E` runs on from it along `E`'s outer pieces to the other crossing, so
/// each loop from its gap round to it, one after the other, is the
/// joined loop. It takes the place of the lower loop. A loop with the gaps
/// of several run-overs, the plate's outer loop with its edges crossed by
/// several, is joined to one contact's loop after another. `gaps` are
/// `(run-over, loop, place in it)`; none leaves the loops as they are.
fn join_at_gaps(
    loops: &mut Vec<Vec<StoredUse>>,
    gaps: &[(EdgeId, usize, usize)],
) -> Result<(), OpError> {
    if gaps.is_empty() {
        return Ok(());
    }
    let mut items: Vec<Vec<Item>> = (loops.drain(..))
        .map(|l| l.into_iter().map(Item::Use).collect())
        .collect();
    for &(by, l, at) in gaps.iter().rev() {
        items
            .get_mut(l)
            .ok_or(invariant("a run-over's gap in a loop of its face"))?
            .insert(at, Item::Gap(by));
    }
    let mut ids: Vec<EdgeId> = Vec::new();
    for g in gaps {
        if !ids.contains(&g.0) {
            ids.push(g.0);
        }
    }
    for id in ids {
        let find = |items: &[Vec<Item>]| -> Vec<(usize, usize)> {
            let mut at = Vec::new();
            for (l, list) in items.iter().enumerate() {
                for (i, item) in list.iter().enumerate() {
                    if matches!(item, Item::Gap(g) if *g == id) {
                        at.push((l, i));
                    }
                }
            }
            at
        };
        let [(a, at_a), (b, at_b)] = find(&items)[..] else {
            return Err(invariant("a run-over's two gaps on its face"));
        };
        if a == b {
            return Err(invariant("a run-over's gaps on two loops of its face"));
        }
        let (a, at_a, b, at_b) = if a < b {
            (a, at_a, b, at_b)
        } else {
            (b, at_b, a, at_a)
        };
        let mut joined: Vec<Item> = Vec::new();
        let (second, first): (Vec<Item>, Vec<Item>) =
            (std::mem::take(&mut items[b]), std::mem::take(&mut items[a]));
        for (list, at) in [(first, at_a), (second, at_b)] {
            let (before, after) = list.split_at(at);
            let after = after.iter().skip(1);
            joined.extend(after.chain(before.iter()).map(|i| match i {
                Item::Use(u) => Item::Use(*u),
                Item::Gap(g) => Item::Gap(*g),
            }));
        }
        items[a] = joined;
        items.remove(b);
    }
    for list in items {
        let mut uses = Vec::with_capacity(list.len());
        for item in list {
            match item {
                Item::Use(u) => uses.push(u),
                Item::Gap(_) => return Err(invariant("a run-over's gap left in a loop")),
            }
        }
        loops.push(uses);
    }
    Ok(())
}

/// The edge a stored use is over, its curve, range and tolerance, and
/// whether it is an operand edge the blend shortened (`Some`, its index in
/// the rewrite) or one it made or kept whole (`None`). `None` overall for
/// an edge with no curve.
type UseEdge = (CurveId, Interval, f64, Option<usize>);

fn use_edge(m: &Model, rw: &Rewrite, key: EdgeKey) -> Result<Option<UseEdge>, OpError> {
    Ok(match key {
        EdgeKey::Kept(id) => {
            let edge = m.edge(id)?;
            edge.curve().map(|(c, r)| (c, r, edge.tolerance(), None))
        }
        EdgeKey::New(k) => match rw.edges.get(k) {
            Some((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve { curve, range },
                    tolerance,
                    ..
                },
                parent,
            )) => Some((*curve, *range, *tolerance, parent.map(|_| k))),
            Some(_) => None,
            None => return Err(invariant("a stored use's edge in the rewrite")),
        },
    })
}

/// The loops of `face` with every shortened operand edge's stored pcurve
/// ended where the blend's new edge meets it at the cut: a read edge's
/// pcurve may stand off its curve by as much as the edge's tolerance,
/// while the blend's contacts and arcs are exact, so at the cut the two
/// would jump (L2). The shortened edge is a new edge already and keeps its
/// tolerance, which its cut vertex holds; the move is checked against it
/// at the model's samples (ADR-0062, step 10b). A junction within half the
/// L2 band is left as it is, and so is one whose two points are further
/// apart in 3D than the edge's tolerance, or a move the tolerance does not
/// hold, for the checker to report.
fn end_on_kept_pcurves(
    m: &mut Model,
    rw: &Rewrite,
    face: FaceId,
    loops: &mut [Vec<StoredUse>],
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<(), OpError> {
    let parametric = m.precision().parametric_tolerance;
    let surface = m.surface(m.face(face)?.surface())?.clone();
    let end_param = |range: Interval, o: Orientation, at_end: bool| {
        if (o == Orientation::Forward) == at_end {
            range.hi()
        } else {
            range.lo()
        }
    };
    for uses in loops.iter_mut() {
        let n = uses.len();
        // The moved ends of each shortened use, by its place in the loop:
        // `[range.lo(), range.hi()]`.
        let mut moves: BTreeMap<usize, [Option<Point2>; 2]> = BTreeMap::new();
        for i in 0..n {
            let j = (i + 1) % n;
            let (Some(a), Some(b)) = (
                use_edge(m, rw, uses[i].edge)?,
                use_edge(m, rw, uses[j].edge)?,
            ) else {
                continue;
            };
            let made = |key: EdgeKey| matches!(key, EdgeKey::New(k) if rw.edges[k].1.is_none());
            // The shortened use and the new one, each with the parameter
            // it meets the junction at: the end of `i`, the start of `j`.
            let ((cut, cut_t, cut_range), (new, new_t)) = match (a.3, b.3) {
                (Some(_), None) if made(uses[j].edge) => (
                    (i, end_param(a.1, uses[i].orientation, true), a.1),
                    (j, end_param(b.1, uses[j].orientation, false)),
                ),
                (None, Some(_)) if made(uses[i].edge) => (
                    (j, end_param(b.1, uses[j].orientation, false), b.1),
                    (i, end_param(a.1, uses[i].orientation, true)),
                ),
                _ => continue,
            };
            let target = m.curve2(uses[new].pcurve)?.point(new_t);
            let at = m.curve2(uses[cut].pcurve)?.point(cut_t);
            let bound = bands(&surface, target, parametric);
            let d = target - at;
            if d.x.abs() <= 0.5 * bound[0] && d.y.abs() <= 0.5 * bound[1] {
                continue;
            }
            // Further apart in 3D than the shortened edge's tolerance —
            // a whole period across a seam, which L2 takes — no move
            // within it explains.
            let apart = (surface.point(target.x, target.y) - surface.point(at.x, at.y)).norm();
            let cut_tolerance = if cut == i { a.2 } else { b.2 };
            if apart > cut_tolerance {
                continue;
            }
            let slot = usize::from(cut_t != cut_range.lo());
            moves.entry(cut).or_default()[slot] = Some(target);
        }
        for (cut, ends) in moves {
            let Some((curve, range, tolerance, _)) = use_edge(m, rw, uses[cut].edge)? else {
                return Err(invariant("a shortened use's edge has a curve"));
            };
            let pc = m.curve2(uses[cut].pcurve)?.clone();
            let moved = pcurve_ending_on(&pc, range, ends, &surface, tol.linear, meter)
                .map_err(fault_of)?;
            let curve = m.curve(curve)?;
            let n = samples.max(2);
            let residual = (0..n)
                .map(|i| range.lerp(i as f64 / (n - 1) as f64))
                .map(|t| {
                    let q = moved.point(t);
                    let (on_surface, on_curve) = (surface.point(q.x, q.y), curve.point(t));
                    (on_surface - on_curve).norm()
                        + RELATIVE_ROUNDING * on_surface.coords.norm().max(on_curve.coords.norm())
                })
                .fold(0.0, f64::max);
            if residual <= tolerance {
                uses[cut].pcurve = m.add_curve2(moved);
            }
        }
    }
    Ok(())
}

/// The blend faces, one per edge, after the shell's own: each end's arc walked from one contact to the other, the miter's with its pcurve on this blend's cylinder.
pub(super) fn add_blend_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    blends: &[Blend],
    joins: &Joins,
    made: &mut Drafted,
) -> Result<(), OpError> {
    let (m, view, ..) = ctx.parts();
    let rw = &mut draft.rw;
    let Joins {
        miters, corners, ..
    } = joins;
    let made = &mut made.stripes;
    for (blend, made) in blends.iter().zip(made.iter_mut()) {
        let contact_edges = made.contacts;
        let [lo, hi] = &blend.contacts;
        let mut end_uses: [Vec<StoredUse>; 2] = Default::default();
        for (end_index, end) in blend.ends.iter().enumerate() {
            let arcs: Vec<(bool, &Curve2)> = match end {
                EndKind::Face(face_end) => face_end
                    .pieces
                    .iter()
                    .map(|p| (p.arc.lo_first, &p.arc.on_blend))
                    .collect(),
                EndKind::Pole(pole) => vec![(true, &pole.on_blend)],
                EndKind::Miter { at, side } => miters[*at].arcs(*side),
                EndKind::Corner { at, side } => {
                    let arc = &corners[*at].arcs[*side];
                    vec![(arc.lo_first, &arc.on_blend)]
                }
            };
            if arcs.len() != made.arcs[end_index].len() {
                return Err(invariant("an edge per arc of the blend's end"));
            }
            // The start's arcs are walked from `u = 0` to `u = β`, the
            // end's back, a fan's pieces in reverse.
            let uses = &mut end_uses[end_index];
            for (&(lo_first, pcurve), &edge) in arcs.iter().zip(&made.arcs[end_index]) {
                uses.push(StoredUse::new(
                    m,
                    EdgeKey::New(edge),
                    lo_first == (end_index == 0),
                    pcurve,
                ));
            }
            if end_index == 1 {
                uses.reverse();
            }
        }
        let [start_arcs, end_arcs] = end_uses;
        let shell = view.shell_of[&lo.face];
        if let (Some(rolled), Some(rolled_made)) = (&blend.rolled, &mut made.rolled) {
            add_rolled_faces(
                m,
                rw,
                shell,
                blend,
                rolled,
                rolled_made,
                &mut made.added,
                start_arcs,
                end_arcs,
            )?;
            continue;
        }
        let surface = m.add_surface(blend.stripe.surface.clone());
        let mut loop_uses = start_arcs;
        loop_uses.push(StoredUse::new(
            m,
            EdgeKey::New(contact_edges[1]),
            true,
            &hi.on_blend,
        ));
        loop_uses.extend(end_arcs);
        loop_uses.push(StoredUse::new(
            m,
            EdgeKey::New(contact_edges[0]),
            false,
            &lo.on_blend,
        ));
        made.added = rw.face(AddedFace {
            shell,
            surface,
            orientation: along(
                blend.stripe.convex
                    != matches!(blend.stripe.section, Section::Walked { flipped: true, .. }),
            ),
            loops: vec![loop_uses],
            tolerance: blend.stripe.tolerance,
        });
    }
    Ok(())
}

/// The corner faces, one per corner, their sides in walking order with a sphere's pole crossed where its meridians meet.
pub(super) fn add_corner_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    joins: &Joins,
    made: &mut Drafted,
) {
    let (m, view, ..) = ctx.parts();
    let rw = &mut draft.rw;
    let corners = &joins.corners;
    let corner_made = &mut made.corners;
    for (c, made) in corners.iter().zip(corner_made.iter_mut()) {
        let surface = m.add_surface(c.surface.clone());
        let mut loop_uses: Vec<StoredUse> = Vec::with_capacity(4);
        for (w, &side) in c.walk.iter().enumerate() {
            let arc = &c.arcs[side];
            loop_uses.push(StoredUse::new(
                m,
                EdgeKey::New(made.arcs[side]),
                arc.along,
                &arc.on_corner,
            ));
            if let (Some(pole), Some(edge)) = (&c.pole, made.pole)
                && pole.after == w
            {
                loop_uses.push(StoredUse::new(m, EdgeKey::New(edge), false, &pole.pcurve));
            }
        }
        made.added = rw.face(AddedFace {
            shell: view.shell_of[&c.faces[0]],
            surface,
            orientation: c.orientation,
            loops: vec![loop_uses],
            tolerance: c.tolerance,
        });
    }
}

/// The ring faces, one per circular edge, in (u, v) counter-clockwise: the lower contact along `u`, up at its far `u` — the seam at `u = 2π` or the end there — the upper contact back, down at its near `u`.
pub(super) fn add_ring_faces(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    rings: &[Ring],
    joins: &Joins,
    made: &mut Drafted,
) -> Result<(), OpError> {
    let (m, view, ..) = ctx.parts();
    let rw = &mut draft.rw;
    let Joins {
        miters, miter_at, ..
    } = joins;
    let ring_made = &mut made.rings;
    for (r, made) in rings.iter().zip(ring_made.iter_mut()) {
        let [lower, upper] = &r.contacts;
        if let Some(rolled) = &r.rolled {
            add_ring_rolled_faces(m, rw, view.shell_of[&lower.face], r, rolled, made)?;
            continue;
        }
        let surface = m.add_surface(r.surface.clone());
        if r.across {
            // A walked ring: `u` across, `v` along the edge, the seam at
            // `v`'s two ends — the loop of an open stripe whose two ends
            // are one arc.
            let RingEnds::Seam(seam) = &r.ends else {
                return Err(invariant("a walked ring's seam"));
            };
            let loop_uses = vec![
                StoredUse::new(m, EdgeKey::New(made.arcs[0]), true, &seam.on_blend[0]),
                StoredUse::new(m, EdgeKey::New(made.contacts[1]), true, &upper.on_blend),
                StoredUse::new(m, EdgeKey::New(made.arcs[0]), false, &seam.on_blend[1]),
                StoredUse::new(m, EdgeKey::New(made.contacts[0]), false, &lower.on_blend),
            ];
            made.added = rw.face(AddedFace {
                shell: view.shell_of[&lower.face],
                surface,
                orientation: r.orientation,
                loops: vec![loop_uses],
                tolerance: r.tolerance,
            });
            continue;
        }
        // Each end's edge, its pcurve, and whether it runs up from the
        // lower contact: a seam's and a face end's do, a junction's when
        // its `q` is on the lower contact.
        let (far, near) = match &r.ends {
            RingEnds::Seam(seam) => (
                (made.arcs[0], seam.on_blend[1].clone(), true),
                (made.arcs[0], seam.on_blend[0].clone(), true),
            ),
            RingEnds::Open(ends) => {
                // The edge's end is at the far `u` when the contacts run
                // with `u`, its start otherwise.
                let (hi, lo) = if lower.along_u { (1, 0) } else { (0, 1) };
                let end = |j: usize| match &ends[j] {
                    RingEnd::Face(end) => (made.arcs[j], end.on_blend.clone(), end.lo_first),
                    RingEnd::Junction(vertex) => {
                        let junction = &miters[miter_at[vertex]];
                        let side = usize::from(junction.edges[0] != r.edge);
                        // Up from the lower contact when `range.lo()`
                        // is on it: at `q` when `q` is.
                        (
                            made.arcs[j],
                            junction.on_blend[side].clone(),
                            junction.q_first == (junction.shared[side] == 0),
                        )
                    }
                };
                (end(hi), end(lo))
            }
        };
        let loop_uses = vec![
            StoredUse::new(
                m,
                EdgeKey::New(made.contacts[0]),
                lower.along_u,
                &lower.on_blend,
            ),
            StoredUse::new(m, EdgeKey::New(far.0), far.2, &far.1),
            StoredUse::new(
                m,
                EdgeKey::New(made.contacts[1]),
                !upper.along_u,
                &upper.on_blend,
            ),
            StoredUse::new(m, EdgeKey::New(near.0), !near.2, &near.1),
        ];
        made.added = rw.face(AddedFace {
            shell: view.shell_of[&lower.face],
            surface,
            orientation: r.orientation,
            loops: vec![loop_uses],
            tolerance: r.tolerance,
        });
    }
    Ok(())
}
