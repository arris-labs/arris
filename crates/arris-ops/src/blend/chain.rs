//! Chains of blended edges: tangent vertices, cusps and the parameters where an edge meets a vertex.

use std::collections::BTreeSet;

use arris_math::{Meter, Tolerance, UnitVec3, Vec3};
use arris_topo::{EdgeId, FaceId, Model, VertexId};

use super::view::{View, convex_edge, tangent_at};
use crate::error::OpError;

/// Whether two faces with outward normals `n1` and `n2` at a point of
/// their edge meet tangentially for a blend of `size` (ADR-0040): the
/// normals parallel within `tol.angular`, or the ball — a chamfer's
/// contacts — that touches one face moved by no more than `tolerance`, the
/// faces' own, to touch the other, `size` times the sine between them.
pub(super) fn tangent_normals(
    n1: Vec3,
    n2: Vec3,
    size: f64,
    tolerance: f64,
    tol: Tolerance,
) -> bool {
    let sine = n1.cross(&n2).norm();
    sine <= tol.angular || size * sine <= tolerance
}

/// The parameter of `edge` at its end `vertex`, its start's on a closed
/// edge; `None` for an edge with no curve.
pub(super) fn parameter_at(
    m: &Model,
    edge: EdgeId,
    vertex: VertexId,
) -> Result<Option<f64>, OpError> {
    let entity = *m.edge(edge)?;
    Ok(entity.curve().map(|(_, range)| {
        if entity.start() == vertex {
            range.lo()
        } else {
            range.hi()
        }
    }))
}

/// The unit tangent of `edge` at its end `vertex`, pointing away from it;
/// `None` for an edge with no curve or no tangent there.
pub(super) fn leaving_vertex(
    m: &Model,
    edge: EdgeId,
    vertex: VertexId,
    tol: Tolerance,
) -> Result<Option<Vec3>, OpError> {
    let entity = *m.edge(edge)?;
    let Some((curve, range)) = entity.curve() else {
        return Ok(None);
    };
    let at_lo = entity.start() == vertex;
    let t = if at_lo { range.lo() } else { range.hi() };
    let d1 = m.curve(curve)?.eval(t).d1;
    let away = if at_lo { d1 } else { -d1 };
    Ok(UnitVec3::try_new(away, tol.linear).map(UnitVec3::into_inner))
}

/// The cusp `edge` ends at, at `vertex` (ADR-0042 §1), whatever the
/// senses of its edges and whatever is blended: `(next, spine)` where the
/// vertex has exactly the three edges `edge`, `next` and `spine`, `edge`
/// and `next` share exactly one face, `spine` joins the other face of each
/// and is tangent at the vertex for a blend of `size`, `next` is open and
/// not tangent there, and `next` leaves the vertex the way `edge` does —
/// the outline doubling back. `None` at any other vertex.
pub(super) fn cusp_at(
    m: &Model,
    view: &View,
    edge: EdgeId,
    vertex: VertexId,
    size: f64,
    tol: Tolerance,
) -> Result<Option<(EdgeId, EdgeId)>, OpError> {
    let Some(at) = view.vertex_edges.get(&vertex) else {
        return Ok(None);
    };
    if at.len() != 3 || !at.contains(&edge) {
        return Ok(None);
    }
    let faces_of = |x: EdgeId| -> BTreeSet<FaceId> {
        view.uses
            .get(&x)
            .map(|u| u.iter().map(|u| u.face).collect())
            .unwrap_or_default()
    };
    let own = faces_of(edge);
    let Some(away) = leaving_vertex(m, edge, vertex, tol)? else {
        return Ok(None);
    };
    let others: Vec<EdgeId> = at.iter().copied().filter(|&x| x != edge).collect();
    let [a, b] = others[..] else {
        return Ok(None);
    };
    for (next, spine) in [(a, b), (b, a)] {
        let (theirs, joins) = (faces_of(next), faces_of(spine));
        let shared: Vec<FaceId> = own.intersection(&theirs).copied().collect();
        let [face] = shared[..] else {
            continue;
        };
        let bridges = own.len() == 2
            && theirs.len() == 2
            && joins.len() == 2
            && !joins.contains(&face)
            && own
                .iter()
                .chain(&theirs)
                .filter(|f| joins.contains(f))
                .count()
                == 2;
        let next_entity = *m.edge(next)?;
        let (Some(t_next), Some(t_spine)) = (
            parameter_at(m, next, vertex)?,
            parameter_at(m, spine, vertex)?,
        ) else {
            continue;
        };
        if !bridges
            || next_entity.start() == next_entity.end()
            || tangent_at(m, view, spine, t_spine, size, tol)? != Some(true)
            || tangent_at(m, view, next, t_next, size, tol)? != Some(false)
        {
            continue;
        }
        if leaving_vertex(m, next, vertex, tol)?.is_some_and(|d| d.dot(&away) > 0.0) {
            return Ok(Some((next, spine)));
        }
    }
    Ok(None)
}

/// The edge a blend of `edge` runs on into at `vertex`, when that is a
/// tangent vertex (ADR-0035 §1): exactly three edges `edge`, `next` and
/// `w`, the two faces of `w` tangent at the vertex, `next` open and not a
/// tangent dihedral at its midpoint, `edge` and `next` sharing exactly one
/// face, both convex or both concave — an outline that turns from one to
/// the other there puts the ball on the far side of the shared face — and
/// the direction leaving the vertex along `next` within a right angle of
/// the one arriving along `edge`. Or, where both of the edge's faces turn
/// there (ADR-0039 §1), exactly four edges `edge`, `next`, `w₀` and `w₁`,
/// `edge` and `next` sharing no face, each `w` tangent at the vertex and
/// between one face of `edge` and one of `next`, a different one of each,
/// and `next` as above. Or, where the edge runs on with nothing turning
/// (ADR-0041 §1), `edge` and `next` sharing both their faces, with no other
/// edge at the vertex or only a seam of one of those faces, used twice by
/// it, or a seam of each when both turn, and `next` as above. `None` at any other vertex, and where more
/// than one of the vertex's other edges would qualify.
pub(super) fn tangent_vertex(
    m: &Model,
    view: &View,
    edge: EdgeId,
    vertex: VertexId,
    size: f64,
    tol: Tolerance,
) -> Result<Option<EdgeId>, OpError> {
    let Some(at) = view.vertex_edges.get(&vertex) else {
        return Ok(None);
    };
    if !at.contains(&edge) || !(2..=4).contains(&at.len()) {
        return Ok(None);
    }
    let others: Vec<EdgeId> = at.iter().copied().filter(|&x| x != edge).collect();
    let faces_of = |x: EdgeId| -> BTreeSet<FaceId> {
        view.uses
            .get(&x)
            .map(|u| u.iter().map(|u| u.face).collect())
            .unwrap_or_default()
    };
    let own = faces_of(edge);
    // A seam of one of the edge's faces, the one face using it twice.
    let is_seam = |w: EdgeId| {
        view.uses
            .get(&w)
            .is_some_and(|u| matches!(u[..], [a, b] if a.face == b.face && own.contains(&a.face)))
    };
    // The faces `edge` and `next` share: both where the edge runs on with
    // nothing turning — a vertex of two edges, or of three whose third is a
    // seam — one at another vertex of three edges, none at a vertex of four.
    let shared = match at.len() {
        2 => 2,
        3 if others.iter().any(|&w| is_seam(w)) => 2,
        3 => 1,
        4 if others.iter().filter(|&&w| is_seam(w)).count() == 2 => 2,
        _ => 0,
    };
    let leaving = |x: EdgeId| leaving_vertex(m, x, vertex, tol);
    let Some(arriving) = leaving(edge)?.map(|d| -d) else {
        return Ok(None);
    };
    if shared == 2 && own.len() != 2 {
        return Ok(None);
    }
    let mut found = None;
    'next: for &next in &others {
        let next_entity = *m.edge(next)?;
        let Some((_, next_range)) = next_entity.curve() else {
            continue;
        };
        let theirs = faces_of(next);
        if next_entity.start() == next_entity.end()
            || tangent_at(m, view, next, next_range.midpoint(), size, tol)? != Some(false)
            || own.intersection(&theirs).count() != shared
            || convex_edge(m, view, next)? != convex_edge(m, view, edge)?
        {
            continue;
        }
        // Each other edge tangent at the vertex; at a vertex of four, each
        // joins a face of `edge` to a face of `next`, no face twice.
        let mut joined: BTreeSet<FaceId> = BTreeSet::new();
        for &w in others.iter().filter(|&&w| w != next) {
            // The two seams of a vertex of four are one face's each.
            if shared == 2 && at.len() == 4 && !faces_of(w).iter().all(|&f| joined.insert(f)) {
                continue 'next;
            }
            let Some(t_w) = parameter_at(m, w, vertex)? else {
                continue 'next;
            };
            if tangent_at(m, view, w, t_w, size, tol)? != Some(true) || (shared == 2 && !is_seam(w))
            {
                continue 'next;
            }
            if shared == 0 {
                let faces_w = faces_of(w);
                let bridges = faces_w.len() == 2
                    && own.intersection(&faces_w).count() == 1
                    && theirs.intersection(&faces_w).count() == 1;
                if !bridges || !faces_w.iter().all(|&f| joined.insert(f)) {
                    continue 'next;
                }
            }
        }
        let Some(away) = leaving(next)? else {
            continue;
        };
        if away.dot(&arriving) > 0.0 && found.replace(next).is_some() {
            return Ok(None);
        }
    }
    Ok(found)
}

/// `named` closed under tangent continuation (ADR-0035 §1): every edge a
/// blend of one of them runs on into through tangent vertices, walked
/// from each end until a vertex that is not one.
pub(super) fn chain(
    m: &Model,
    view: &View,
    named: &[EdgeId],
    size: f64,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<BTreeSet<EdgeId>, OpError> {
    let mut reached: BTreeSet<EdgeId> = named.iter().copied().collect();
    let mut todo: Vec<EdgeId> = named.to_vec();
    while let Some(edge) = todo.pop() {
        meter.tick()?;
        let entity = *m.edge(edge)?;
        if entity.start() == entity.end() {
            continue;
        }
        for vertex in [entity.start(), entity.end()] {
            if let Some(next) = tangent_vertex(m, view, edge, vertex, size, tol)?
                && reached.insert(next)
            {
                todo.push(next);
            }
        }
    }
    Ok(reached)
}
