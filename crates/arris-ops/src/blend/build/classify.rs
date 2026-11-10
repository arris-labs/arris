//! Classify chained edges and their vertex joins before building blends.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::Curve;
use arris_math::{Meter, Point2};
use arris_topo::{Body, EdgeId, FaceId, VertexId};

use super::super::chain::{chain, cusp_at, kinked_vertex, tangent_vertex};
use super::super::ends::face_end;
use super::super::mutual::{Cusp, CuspJoin};
use super::super::ring::{CuspEnd, Ring, contact_on, in_row, ring};
use super::super::stripe::{Section, stripe};
use super::super::walked_blend::{walked_end, walked_ring, walked_stripe};
use super::super::{degenerate, invariant};
use super::{BlendCtx, Classified, Env, Meets};
use crate::error::{BlendReason, OpError, Reason};
use crate::rebuild::forward;

/// The chained edges, each into the group that builds it: the closed edges' rings are built here.
pub(super) fn classify(
    ctx: &mut BlendCtx<'_, '_>,
    body: Body,
    edges: &[EdgeId],
) -> Result<Classified, OpError> {
    let (env, meter) = ctx.split();
    let Env {
        m, view, kind, tol, ..
    } = env;
    // The named edges and every edge their chains run on into, in the
    // body's order.
    let mut reached = chain(m, view, edges, kind.size(), tol, meter)?;
    let kinks = through_kinks(&env, meter, &mut reached)?;
    let chained: Vec<EdgeId> = m
        .edges(body)?
        .into_iter()
        .map(|e| e.id)
        .filter(|id| reached.contains(id))
        .collect();
    let no_junctions = BTreeSet::new();
    // A closed edge and a circular arc are rings, the one with no ends and
    // the other trimmed at its two or met at a junction; the rest are
    // stripes.
    let mut open: Vec<EdgeId> = Vec::with_capacity(chained.len());
    let mut arcs: Vec<EdgeId> = Vec::new();
    let mut rings: Vec<Ring> = Vec::new();
    for &e in &chained {
        let entity = *m.edge(e)?;
        // A fillet whose faces or curve the closed forms do not hold is
        // walked (ADR-0062).
        let round = match entity.curve() {
            Some((curve, _)) => {
                entity.start() == entity.end() || matches!(m.curve(curve)?, Curve::Circle { .. })
            }
            None => false,
        };
        let walk = round && !in_row(&env, e)?;
        match entity.curve() {
            Some(_) if entity.start() == entity.end() => {
                rings.push(if walk {
                    walked_ring(&env, meter, e)?
                } else {
                    ring(&env, meter, e, &no_junctions, &BTreeMap::new())?
                });
            }
            Some((curve, _)) if !walk && matches!(m.curve(curve)?, Curve::Circle { .. }) => {
                arcs.push(e);
            }
            Some(_) | None => open.push(e),
        }
    }
    Ok(Classified {
        open,
        arcs,
        rings,
        kinks,
    })
}

/// `reached` run on through every kink a chain end's vertex writes as
/// tangent within its tolerance where the end across it meets nothing
/// (ADR-0075), each continued to where it next ends; the kinks, each a
/// junction of the build's. A kink's end is made as the build would make
/// it, and the chain runs through only where that end is the
/// [`near_tangent_corner`](super::super::ends::near_tangent_corner) refusal naming
/// the same corner edge: a contact that misses the face across. Errors: the meter's, and a fault of the model's.
fn through_kinks(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    reached: &mut BTreeSet<EdgeId>,
) -> Result<BTreeSet<VertexId>, OpError> {
    let Env {
        m, view, kind, tol, ..
    } = *env;
    let mut kinks: BTreeSet<VertexId> = BTreeSet::new();
    let mut todo: Vec<EdgeId> = reached.iter().copied().collect();
    while let Some(edge) = todo.pop() {
        meter.tick()?;
        let entity = *m.edge(edge)?;
        if entity.start() == entity.end() {
            continue;
        }
        for vertex in [entity.start(), entity.end()] {
            if kinks.contains(&vertex) {
                continue;
            }
            let Some((next, w)) = kinked_vertex(m, view, edge, vertex, kind.size(), tol)? else {
                continue;
            };
            if reached.contains(&next) || !end_meets_nothing(env, meter, edge, vertex, w)? {
                continue;
            }
            kinks.insert(vertex);
            for e in chain(m, view, &[next], kind.size(), tol, meter)? {
                if reached.insert(e) {
                    todo.push(e);
                }
            }
        }
    }
    Ok(kinks)
}

/// Whether the end of `edge`'s blend at `vertex` meets nothing across
/// it, beside the corner edge `w`: its end, made as the build makes it,
/// is `TangentChain` naming the edge, `w` and the vertex. Any other end,
/// built or refused, is the build's to make. Errors: the meter's, and a
/// fault of the model's.
fn end_meets_nothing(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    edge: EdgeId,
    vertex: VertexId,
    w: EdgeId,
) -> Result<bool, OpError> {
    let Env {
        m, view, kind, tol, ..
    } = *env;
    let made = match stripe(m, view, edge, kind, tol) {
        Err(OpError::Unsupported { .. }) => walked_stripe(env, meter, edge),
        other => other,
    };
    let s = match made {
        Ok(s) => s,
        Err(e @ (OpError::Interrupted(_) | OpError::Internal(_))) => return Err(e),
        Err(_) => return Ok(false),
    };
    let at_lo = s.start == vertex;
    if matches!(s.section, Section::Walked { poles, .. } if poles[usize::from(!at_lo)]) {
        return Ok(false);
    }
    let end = match s.section {
        Section::Walked { .. } => walked_end(env, meter, &s, at_lo),
        Section::Round { .. } | Section::Flat => face_end(env, meter, &s, at_lo),
    };
    match end {
        Err(OpError::Degenerate {
            entities,
            reason: Reason::Blend(BlendReason::TangentChain),
        }) => Ok(entities == [forward(edge), forward(w), forward(vertex)]),
        Err(e @ (OpError::Interrupted(_) | OpError::Internal(_))) => Err(e),
        Ok(_) | Err(_) => Ok(false),
    }
}

/// The blended edges at each vertex, the vertices that are junctions and
/// the cusps both of whose edges are blended; the arcs' rings are built
/// here.
pub(super) fn vertex_joins(
    ctx: &mut BlendCtx<'_, '_>,
    open: &[EdgeId],
    arcs: &[EdgeId],
    rings: &mut Vec<Ring>,
    kinks: &BTreeSet<VertexId>,
) -> Result<Meets, OpError> {
    let (env, meter) = ctx.split();
    let Env {
        m, view, kind, tol, ..
    } = env;
    let edges = open;
    // The blended edges at each vertex: two at a tangent vertex meet in a
    // junction, two elsewhere in a miter, three in a corner, and more at a
    // vertex the closed forms do not cover; an arc meets no other blend but
    // at a junction.
    let mut at_vertex: BTreeMap<VertexId, Vec<EdgeId>> = BTreeMap::new();
    for &e in edges.iter().chain(arcs) {
        let entity = *m.edge(e)?;
        for v in [entity.start(), entity.end()] {
            at_vertex.entry(v).or_default().push(e);
        }
    }
    let mut junctions: BTreeSet<VertexId> = BTreeSet::new();
    let mut cusps: Vec<CuspJoin> = Vec::new();
    for (&v, es) in &at_vertex {
        // A kink the chain runs through is a junction too (ADR-0075 §2).
        if let [ea, eb] = es[..]
            && (kinks.contains(&v) || tangent_vertex(m, view, ea, v, kind.size(), tol)? == Some(eb))
        {
            junctions.insert(v);
            continue;
        }
        // Both edges of a cusp blended: two arcs of one sense, filleted or
        // chamfered under a plane, are the stripes' mutual trim (ADR-0084 §4′); any
        // other such cusp keeps ADR-0042 §6's refusal.
        if let [ea, eb] = es[..]
            && let Some((next, spine)) = cusp_at(m, view, ea, v, kind.size(), tol)?
            && next == eb
        {
            match cusp_of(&env, [ea, eb], spine, v, (open, arcs))? {
                Some(join) => {
                    cusps.push(join);
                    continue;
                }
                None => {
                    return Err(degenerate(
                        vec![forward(ea), forward(spine), forward(v)],
                        Reason::Blend(BlendReason::TangentChain),
                    ));
                }
            }
        }
        if es.len() > 3 || (es.len() > 1 && es.iter().any(|e| arcs.contains(e))) {
            let entities = es.iter().map(|&e| forward(e)).chain([forward(v)]).collect();
            return Err(degenerate(
                entities,
                Reason::Blend(BlendReason::VertexBlend),
            ));
        }
    }
    let mut ends: BTreeMap<VertexId, CuspEnd> = BTreeMap::new();
    for join in &cusps {
        ends.insert(join.vertex, join.end(&env)?);
    }
    for &e in arcs {
        rings.push(ring(&env, meter, e, &junctions, &ends)?);
    }
    at_vertex.retain(|v, es| junctions.contains(v) || !es.iter().any(|e| arcs.contains(e)));
    Ok(Meets {
        at_vertex,
        junctions,
        cusps,
    })
}

/// The cusp at `vertex` of the blended `edges` and the `spine`, read for
/// the mutual trim (ADR-0084 §1, §4′): the edges blended, of one sense,
/// sharing one face `T`, a plane, and each with a wall square to it, and
/// each an arc among `arcs` (a circle, its wall a cylinder about an axis
/// normal to `T`) or a line among `open` (a closed-form stripe, its
/// wall a plane normal to `T`), not both lines. `None` for any other such
/// cusp, which keeps its refusal.
fn cusp_of(
    env: &Env<'_>,
    edges: [EdgeId; 2],
    spine: EdgeId,
    vertex: VertexId,
    (open, arcs): (&[EdgeId], &[EdgeId]),
) -> Result<Option<CuspJoin>, OpError> {
    let (m, view) = (env.m, env.view);
    let size = env.kind.size();
    let is_arc = edges.map(|e| arcs.contains(&e));
    if is_arc == [false, false] || !edges.iter().all(|e| arcs.contains(e) || open.contains(e)) {
        return Ok(None);
    }
    let [Some(convex), Some(other)] = [view.convex(m, edges[0])?, view.convex(m, edges[1])?] else {
        return Ok(None);
    };
    if convex != other {
        return Ok(None);
    }
    let faces_of = |e: EdgeId| -> BTreeSet<FaceId> {
        (view.uses.get(&e).into_iter().flatten())
            .map(|u| u.face)
            .collect()
    };
    let shared: Vec<FaceId> = faces_of(edges[0])
        .intersection(&faces_of(edges[1]))
        .copied()
        .collect();
    let [top] = shared[..] else {
        return Ok(None);
    };
    let &arris_geom::Surface::Plane { frame } = m.surface(m.face(top)?.surface())? else {
        return Ok(None);
    };
    let normal = frame.z().into_inner();
    let mut tops: Vec<Curve> = Vec::with_capacity(2);
    for (e, is_arc) in edges.into_iter().zip(is_arc) {
        for wall in faces_of(e).into_iter().filter(|&f| f != top) {
            let square = match m.surface(m.face(wall)?.surface())? {
                arris_geom::Surface::Cylinder { frame: axis, .. } if is_arc => {
                    axis.z().cross(&normal).norm() <= env.tol.angular
                }
                arris_geom::Surface::Plane { frame: wall } if !is_arc => {
                    wall.z().dot(&normal).abs() <= env.tol.angular
                }
                _ => false,
            };
            if !square {
                return Ok(None);
            }
        }
        // The edge's contact with `T`, before any end trims it.
        if is_arc {
            tops.push(contact_on(env, e, top)?);
        } else {
            let s = stripe(m, view, e, env.kind, env.tol)?;
            let (Section::Round { .. } | Section::Flat, Some(k)) =
                (s.section, s.faces.iter().position(|&f| f == top))
            else {
                return Ok(None);
            };
            tops.push(s.lines[k].clone());
        }
    }
    let Ok(tops) = <[Curve; 2]>::try_from(tops) else {
        return Err(invariant("a cusp join's two contacts"));
    };
    let point = m.vertex(vertex)?.point();
    let uv = frame.to_local(point);
    let outward = view.outward(m, top, Point2::new(uv.x, uv.y))?;
    let inward = if convex { -outward } else { outward };
    let Some(inward) = arris_math::UnitVec3::try_new(inward, env.tol.linear) else {
        return Err(invariant("a plane's unit normal"));
    };
    let (curve, range) = m
        .edge(edges[0])?
        .curve()
        .ok_or(invariant("a cusp edge's curve"))?;
    Ok(Some(CuspJoin {
        vertex,
        edges,
        spine,
        top,
        tops,
        cusp: Cusp {
            top: frame,
            vertex: point,
            side: m.curve(curve)?.point(range.midpoint()),
            inward,
            size,
        },
    }))
}
