//! Vertex-blend refusal sites and oracle verdicts.

use super::run_over::{solid_id, stride_per_cause};
use super::*;

/// The battery's sample, blended together and refused `VertexBlend`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VertexSet {
    /// [`vertex_set_cause`]: the miter or corner the edges make.
    pub cause: String,
    /// A point on each blended edge, the oracle's name for it.
    pub points: Vec<[f64; 3]>,
    /// Whether every one of the sampled edges builds alone, so that only
    /// the set is refused.
    pub each_alone_builds: bool,
    /// Open CASCADE's verdict on the set together; `None` where not asked.
    pub occt: Option<String>,
}

/// How parallel two directions must be for the census to call them one
/// line or one tangent plane: the sine of the angle between them. The
/// census names a site, it decides no geometry, so the cut is wide enough
/// for a part's file digits and far from any real dihedral.
const PARALLEL: f64 = 1e-6;

/// The edges at `vertex` next to `edge` in the loops of its faces, the
/// corner edges ADR-0007 names: for each use of `edge`, the neighbouring
/// coedge in that loop that touches `vertex`.
fn corner_edges(m: &Model, edge: EdgeId, vertex: VertexId) -> Vec<EdgeId> {
    let mut out = Vec::new();
    let Ok(uses) = m.edge_uses(edge) else {
        return out;
    };
    for u in uses {
        let Ok(face) = m.face(u.face) else { continue };
        let Some(l) = face.loops().get(u.loop_index) else {
            continue;
        };
        let n = l.coedges().len();
        for j in [(u.coedge_index + n - 1) % n, (u.coedge_index + 1) % n] {
            let c = l.coedges()[j].edge();
            let touches = m
                .edge(c)
                .is_ok_and(|x| x.start() == vertex || x.end() == vertex);
            if c != edge && touches && !out.contains(&c) {
                out.push(c);
            }
        }
    }
    out
}

/// The direction of `edge`'s curve at its end on `vertex`, sign as the
/// curve runs.
pub(super) fn tangent_at_vertex(m: &Model, edge: EdgeId, vertex: VertexId) -> Option<Vec3> {
    let e = m.edge(edge).ok()?;
    let (c, range) = e.curve()?;
    let t = if e.start() == vertex {
        range.lo()
    } else {
        range.hi()
    };
    let d = m.curve(c).ok()?.eval(t).d1;
    (d.norm() > 0.0).then_some(d)
}

fn parallel(a: Vec3, b: Vec3) -> bool {
    a.cross(&b).norm() <= PARALLEL * a.norm() * b.norm()
}

/// Whether the two faces of `edge` are tangent at its midpoint.
fn smooth_edge(m: &Model, edge: EdgeId) -> bool {
    let Ok(uses) = m.edge_uses(edge) else {
        return false;
    };
    let (Some(at), [a, b]) = (edge_midpoint(m, edge), uses) else {
        return false;
    };
    let normal = |f| {
        let surface = m.surface(m.face(f).ok()?.surface()).ok()?;
        let uv = surface.project(at).ok()?.uv;
        Some(surface.normal(uv.x, uv.y)?.into_inner())
    };
    match (normal(a.face), normal(b.face)) {
        (Some(x), Some(y)) => parallel(x, y),
        _ => false,
    }
}

/// The cause [`vertex_blend_cause`] names where `edge`'s run goes on
/// straight through `vertex` on another edge that shares exactly one of its
/// faces: the run's second face jumps there, a patch no closed form holds
/// and Open CASCADE does not build either (ftc-06).
const DIHEDRAL_JUMP: &str = "a dihedral jump on a collinear run";

/// Whether `edge` and another edge at `vertex` are one straight line
/// through it and share exactly one face.
fn collinear_jump(m: &Model, edge: EdgeId, vertex: VertexId) -> bool {
    let Some(t) = tangent_at_vertex(m, edge, vertex) else {
        return false;
    };
    let Ok(at) = m.vertex_edges(vertex) else {
        return false;
    };
    let faces = |e: EdgeId| -> Vec<FaceId> { faces_of(m, e) };
    let own = faces(edge);
    at.iter().any(|&x| {
        x != edge
            && tangent_at_vertex(m, x, vertex).is_some_and(|d| parallel(d, t))
            && faces(x).iter().filter(|f| own.contains(f)).count() == 1
    })
}

/// The cycle a `VertexBlend` cause belongs to: the NURBS cycle's where the
/// blend would be a patch with no closed form, the blend network's
/// otherwise.
pub fn vertex_blend_cycle(cause: &str) -> crate::histogram::Cycle {
    if cause.starts_with(DIHEDRAL_JUMP) {
        crate::histogram::Cycle::Nurbs
    } else {
        crate::histogram::Cycle::BlendNetwork
    }
}

/// What an extra edge at a vertex of more than three edges is; `blended`
/// is the edge whose blend was refused there.
fn extra_kind(
    m: &Model,
    edge: EdgeId,
    vertex: VertexId,
    blended: EdgeId,
    corners: &[EdgeId],
) -> &'static str {
    let seam = m
        .edge_uses(edge)
        .is_ok_and(|u| u.len() == 2 && u[0].face == u[1].face);
    if seam {
        return "a seam";
    }
    let Some(t) = tangent_at_vertex(m, edge, vertex) else {
        return "a degenerate edge";
    };
    let continues = tangent_at_vertex(m, blended, vertex).is_some_and(|b| parallel(b, t));
    let collinear = corners
        .iter()
        .filter_map(|&c| tangent_at_vertex(m, c, vertex))
        .any(|c| parallel(c, t));
    if continues {
        "the blended edge's tangent continuation"
    } else if collinear {
        "an edge continuing a corner edge"
    } else if smooth_edge(m, edge) {
        "a smooth edge"
    } else {
        "a sharp edge between two faces across"
    }
}

/// What the second edge at a vertex of two edges is, `blended` being the
/// edge whose blend was refused there: its continuation on one curve between
/// the same two faces (a split rim's second vertex, the seam being at the
/// other), the same two faces turning, or a different pair of faces.
fn second_kind(m: &Model, other: EdgeId, vertex: VertexId, blended: EdgeId) -> &'static str {
    let faces = |e| {
        let mut f: Vec<_> = m
            .edge_uses(e)
            .map(|u| u.iter().map(|x| x.face).collect())
            .unwrap_or_default();
        f.sort();
        f.dedup();
        f
    };
    if faces(blended) != faces(other) {
        return "an edge between a different pair of faces";
    }
    let runs_on = match (
        tangent_at_vertex(m, blended, vertex),
        tangent_at_vertex(m, other, vertex),
    ) {
        (Some(a), Some(b)) => parallel(a, b),
        _ => false,
    };
    if runs_on {
        "a continuation between the same two faces"
    } else {
        "an edge between the same two faces, turning"
    }
}

/// The faces of `edge`'s uses, sorted and without repeats.
pub(super) fn faces_of(m: &Model, edge: EdgeId) -> Vec<FaceId> {
    let mut f: Vec<FaceId> = m
        .edge_uses(edge)
        .map(|u| u.iter().map(|x| x.face).collect())
        .unwrap_or_default();
    f.sort();
    f.dedup();
    f
}

/// The one edge beside `edge` in `face`'s loop that also touches `vertex`.
fn next_at_vertex(m: &Model, face: FaceId, edge: EdgeId, vertex: VertexId) -> Option<EdgeId> {
    let f = m.face(face).ok()?;
    let mut found = None;
    for l in f.loops() {
        let n = l.coedges().len();
        for i in (0..n).filter(|&i| l.coedges()[i].edge() == edge) {
            for j in [(i + n - 1) % n, (i + 1) % n] {
                let c = l.coedges()[j].edge();
                let touches = m
                    .edge(c)
                    .is_ok_and(|x| x.start() == vertex || x.end() == vertex);
                if c != edge && touches && found.is_some_and(|x| x != c) {
                    return None;
                }
                if c != edge && touches {
                    found = Some(c);
                }
            }
        }
    }
    found
}

/// The faces across a blended edge's end at `vertex`: the one face met by
/// both corner edges (*the face across met twice*), or the faces in the
/// vertex's star from one corner edge to the other, each a piece of the end
/// (*a fan of k faces across*). `None` where the star is not one simple walk
/// between the two corner edges (ADR-0043).
fn pieces_across(
    m: &Model,
    blended: EdgeId,
    vertex: VertexId,
    corners: &[EdgeId],
) -> Option<Vec<FaceId>> {
    let [c1, c2] = corners[..] else { return None };
    let blend_faces = faces_of(m, blended);
    let across = |c| match &faces_of(m, c)
        .into_iter()
        .filter(|f| !blend_faces.contains(f))
        .collect::<Vec<_>>()[..]
    {
        [f] => Some(*f),
        _ => None,
    };
    let (a1, a2) = (across(c1)?, across(c2)?);
    if a1 == a2 {
        return Some(vec![a1]);
    }
    let (mut e, mut pieces) = (c1, vec![a1]);
    for _ in 0..m.vertex_edges(vertex).ok()?.len() {
        let f = pieces[pieces.len() - 1];
        let next = next_at_vertex(m, f, e, vertex)?;
        if next == c2 {
            return (f == a2).then_some(pieces);
        }
        pieces.push(*faces_of(m, next).iter().find(|&&x| x != f)?);
        e = next;
    }
    None
}

/// What the faces across a blended edge's end are at `vertex`, as
/// [`pieces_across`] walks them: *the face across met twice* or *a fan of k
/// faces across*.
fn faces_across(
    m: &Model,
    blended: EdgeId,
    vertex: VertexId,
    corners: &[EdgeId],
) -> Option<String> {
    match pieces_across(m, blended, vertex, corners)?[..] {
        [_] => Some("the face across met twice".to_string()),
        ref pieces => Some(format!("a fan of {} faces across", pieces.len())),
    }
}

/// The number of pieces of a *flush fan* (ADR-0082) at `vertex`: the faces
/// across walked by [`pieces_across`], all planes on one plane, each within
/// its own tolerance of the vertex and with its normal parallel to the
/// first's. `None` for any other star.
fn flush_fan(m: &Model, blended: EdgeId, vertex: VertexId, corners: &[EdgeId]) -> Option<usize> {
    let pieces = pieces_across(m, blended, vertex, corners)?;
    if pieces.len() < 2 {
        return None;
    }
    let at = m.vertex(vertex).ok()?.point();
    let mut first: Option<Vec3> = None;
    for &f in &pieces {
        let face = m.face(f).ok()?;
        let arris_geom::Surface::Plane { frame } = *m.surface(face.surface()).ok()? else {
            return None;
        };
        let n = frame.z().into_inner();
        let n0 = *first.get_or_insert(n);
        if !parallel(n0, n) || (at - frame.origin()).dot(&n).abs() > face.tolerance() {
            return None;
        }
    }
    Some(pieces.len())
}

/// Which site refused a `VertexBlend` of `edge` blended alone, from the
/// entities the error names and the vertex's edges (`None` for any other
/// refusal): *a vertex of more than three edges*, with what the extra edges
/// are (sharp edges between two faces across, named by what they make of the
/// end: a fan of k faces across, or the face across met twice; a smooth edge,
/// or a flush fan of k faces across where the pieces are planes on one plane (ADR-0082); a seam; a
/// degenerate edge; the blended edge's tangent continuation; an edge
/// continuing a corner edge); *a corner edge with
/// no curve*; *corner edges that share no face across*; *a closed edge's
/// vertex*; *a blended edge with no curve*; or *an end that more than one
/// blended edge meets*, by its vertex's edges — the chain's junction at a
/// vertex of four edges where both faces turn is one (ADR-0039). Each is followed by the pair of surface kinds the
/// blended edge separates.
pub fn vertex_blend_cause(m: &Model, edge: EdgeId, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::Blend(BlendReason::VertexBlend),
    } = e
    else {
        return None;
    };
    let pair = pair_of(m, edge).unwrap_or_default();
    let ids: Vec<EntityId> = entities.iter().map(|s| s.id).collect();
    let vertex = ids.iter().find_map(|i| match i {
        EntityId::Vertex(v) => Some(*v),
        EntityId::Edge(_) | EntityId::Face(_) | EntityId::Shell(_) | EntityId::Body(_) => None,
    });
    let blended = ids
        .iter()
        .filter(|i| matches!(i, EntityId::Edge(_)))
        .count();
    let what = match (vertex, blended) {
        (None, _) => "a blended edge with no curve".to_string(),
        (Some(v), n) if n > 1 => {
            let at = m.vertex_edges(v).map_or(0, <[EdgeId]>::len);
            format!("{n} edges of the run meet at a vertex of {at} edges")
        }
        (Some(v), _) => {
            let at = m
                .vertex_edges(v)
                .map(<[EdgeId]>::to_vec)
                .unwrap_or_default();
            let corners = corner_edges(m, edge, v);
            let closed = m.edge(edge).is_ok_and(|x| x.start() == x.end());
            if closed {
                format!("a closed edge's vertex of {} edges", at.len())
            } else if at.len() > 3 && collinear_jump(m, edge, v) {
                DIHEDRAL_JUMP.to_string()
            } else if at.len() > 3 {
                let mut kinds: Vec<&str> = at
                    .iter()
                    .filter(|&&x| x != edge && !corners.contains(&x))
                    .map(|&x| extra_kind(m, x, v, edge, &corners))
                    .collect();
                kinds.sort_unstable();
                kinds.dedup();
                let sharp = kinds == ["a sharp edge between two faces across"];
                // A smooth seam collinear with a corner edge reads as the
                // second in `extra_kind`; the site is the same.
                let smooth = at
                    .iter()
                    .filter(|&&x| x != edge && !corners.contains(&x))
                    .all(|&x| smooth_edge(m, x));
                let flush = smooth.then(|| flush_fan(m, edge, v, &corners)).flatten();
                match sharp.then(|| faces_across(m, edge, v, &corners)).flatten() {
                    Some(across) => format!("a vertex of {} edges, {across}", at.len()),
                    None if flush.is_some() => format!(
                        "a vertex of {} edges, a flush fan of {} faces across",
                        at.len(),
                        flush.unwrap_or_default()
                    ),
                    None => format!(
                        "a vertex of {} edges, the extra: {}",
                        at.len(),
                        kinds.join(", ")
                    ),
                }
            } else if corners
                .iter()
                .any(|&c| m.edge(c).is_ok_and(|x| x.curve().is_none()))
            {
                "a corner edge with no curve".to_string()
            } else if let [other] = at
                .iter()
                .copied()
                .filter(|&x| x != edge)
                .collect::<Vec<_>>()[..]
            {
                format!(
                    "a vertex of 2 edges, the second: {}",
                    second_kind(m, edge, v, other)
                )
            } else {
                format!("corner edges that share no face across, {} edges", at.len())
            }
        }
    };
    Some(format!("{what}; edge {pair}"))
}

/// The miter or corner a `VertexBlend` of a set of edges blended together
/// names (`None` for any other refusal): how many blended edges meet at
/// how many edges, and the kinds of face pair they separate.
pub fn vertex_set_cause(m: &Model, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::Blend(BlendReason::VertexBlend),
    } = e
    else {
        return None;
    };
    let mut edges = Vec::new();
    let mut vertex = None;
    for s in entities {
        match s.id {
            EntityId::Edge(x) => edges.push(x),
            EntityId::Vertex(v) => vertex = Some(v),
            EntityId::Face(_) | EntityId::Shell(_) | EntityId::Body(_) => {}
        }
    }
    let mut pairs: Vec<String> = edges.iter().filter_map(|&x| pair_of(m, x)).collect();
    pairs.sort();
    pairs.dedup();
    let at = vertex
        .and_then(|v| m.vertex_edges(v).ok().map(<[EdgeId]>::len))
        .map_or_else(String::new, |n| format!(" at a vertex of {n} edges"));
    Some(format!(
        "{} blended edges{at}; {}",
        edges.len(),
        pairs.join(" / ")
    ))
}

/// Puts a sample of every cause's `VertexBlend` edges of `census`, a solid
/// of `file`, to Open CASCADE alone at the census's radius, as
/// [`ask_the_oracle`] does for `BlendTooLarge`, and the battery's sample
/// together where that is refused `VertexBlend`.
///
/// # Errors
/// As [`ask_the_oracle`].
pub fn ask_the_oracle_vertex(file: &Path, census: &mut SolidCensus) -> Result<(), String> {
    let id = solid_id(&census.solid)?;
    let asked = stride_per_cause(&census.vertex_blend);
    if !asked.is_empty() {
        let points: Vec<[f64; 3]> = asked.iter().map(|&i| census.vertex_blend[i].at).collect();
        let verdicts = crate::oracle::fillet_edges(file, id, census.radius, ORACLE_PROBE, &points)
            .map_err(|e| e.to_string())?;
        for (&i, v) in asked.iter().zip(verdicts) {
            census.vertex_blend[i].occt = Some(v.verdict);
        }
    }
    let radius = census.radius;
    if let Some(set) = &mut census.vertex_set {
        let verdicts = crate::oracle::fillet_edge_sets(
            file,
            id,
            radius,
            ORACLE_PROBE,
            std::slice::from_ref(&set.points),
        )
        .map_err(|e| e.to_string())?;
        set.occt = verdicts.into_iter().next().map(|v| v.verdict);
    }
    Ok(())
}

/// [`census_file`] of `file`, with each solid's `VertexBlend` edges and
/// sampled set put to Open CASCADE ([`ask_the_oracle_vertex`]).
///
/// # Errors
/// As [`census_file`] and [`ask_the_oracle_vertex`].
pub fn vertex_blend_file(
    file: &Path,
    wanted: &dyn Fn(&str) -> bool,
) -> Result<Vec<SolidCensus>, String> {
    let mut solids = census_file(file, wanted)?;
    for s in &mut solids {
        ask_the_oracle_vertex(file, s)?;
    }
    Ok(solids)
}

/// [`committed`] with each solid's `VertexBlend` edges and sampled set put
/// to Open CASCADE.
///
/// # Errors
/// As [`committed`] and [`ask_the_oracle_vertex`].
pub fn vertex_blend_committed() -> Result<Vec<(String, Vec<SolidCensus>)>, String> {
    over_committed_tier(true, vertex_blend_file)
}

/// The `VertexBlend` edges of the solids, counted by `part → cause`, and
/// each part's sampled set where that is refused: how many edges, how many
/// were put to Open CASCADE and what it said.
pub fn vertex_blend_markdown(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut table = Table::new([
        ("part", Align::Left),
        ("solid", Align::Left),
        ("cause", Align::Left),
        ("cycle", Align::Left),
        ("edges", Align::Right),
        ("asked", Align::Right),
        ("builds", Align::Right),
        ("invalid", Align::Right),
        ("refuses", Align::Right),
        ("no edge", Align::Right),
    ]);
    let mut totals = [0usize; 6];
    for (part, solids) in parts {
        for s in solids {
            let mut by_cause: BTreeMap<&str, Vec<&RunOverEdge>> = BTreeMap::new();
            for e in &s.vertex_blend {
                by_cause.entry(&e.cause).or_default().push(e);
            }
            for (cause, edges) in by_cause {
                let said = |v: &str| {
                    edges
                        .iter()
                        .filter(|e| e.occt.as_deref() == Some(v))
                        .count()
                };
                let asked = edges.iter().filter(|e| e.occt.is_some()).count();
                let row = [
                    edges.len(),
                    asked,
                    said("builds"),
                    said("invalid"),
                    said("refuses"),
                    said("no-edge"),
                ];
                for (t, r) in totals.iter_mut().zip(row) {
                    *t += r;
                }
                table.row([
                    part.clone(),
                    s.solid.to_string(),
                    cause.to_string(),
                    vertex_blend_cycle(cause).to_string(),
                    row[0].to_string(),
                    row[1].to_string(),
                    row[2].to_string(),
                    row[3].to_string(),
                    row[4].to_string(),
                    row[5].to_string(),
                ]);
            }
        }
    }
    let mut out = table.to_string();
    out.push_str("\nThe battery's sampled edges refused `VertexBlend`, each alone:\n\n");
    for (part, solids) in parts {
        for s in solids {
            for e in s.vertex_blend.iter().filter(|e| e.sampled) {
                let _ = writeln!(
                    out,
                    "- {part} {}: {} → Open CASCADE {}",
                    s.solid,
                    e.cause,
                    e.occt.as_deref().unwrap_or("not asked")
                );
            }
        }
    }
    out.push_str("\nThe battery's sample blended together, refused `VertexBlend`:\n\n");
    for (part, solids) in parts {
        for s in solids {
            if let Some(set) = &s.vertex_set {
                let _ = writeln!(
                    out,
                    "- {part} {}: {} ({}) → Open CASCADE {}",
                    s.solid,
                    set.cause,
                    if set.each_alone_builds {
                        "each edge alone builds"
                    } else {
                        "an edge alone is refused too"
                    },
                    set.occt.as_deref().unwrap_or("not asked")
                );
            }
        }
    }
    let _ = writeln!(
        out,
        "\nTotal: {} `VertexBlend` edges, {} put to Open CASCADE: {} build, {} build invalid, {} refused, {} found no edge.",
        totals[0], totals[1], totals[2], totals[3], totals[4], totals[5]
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::unmetered::{fuse, primitive_box};
    use arris_math::Point3 as P3;
    use arris_math::Vec3;

    /// The fixture `blend/five-edge-vertex`: a rise ending at a vertex of
    /// five edges, the two extra being the sharp halves of the top edge.
    #[test]
    fn a_vertex_of_five_edges_is_named_by_what_the_extra_edges_are() {
        use crate::unmetered::transform;
        use arris_math::Isometry;
        use arris_math::nalgebra::UnitQuaternion;
        let mut m = Model::default();
        let (lower, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 2.0, 2.0)).unwrap();
        let (unit, _) = primitive_box(&mut m, P3::origin(), P3::new(1.0, 1.0, 1.0)).unwrap();
        let turn = UnitQuaternion::from_axis_angle(&Vec3::z_axis(), -135f64.to_radians());
        let motion = Isometry::new(turn, Vec3::new(1.0, 2.0, 2.0));
        let (upper, _) = transform(&mut m, unit, &motion).unwrap();
        let (fused, _) = fuse(&mut m, lower, upper).unwrap();
        let rise = m
            .edges(fused)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id).is_some_and(|p| (p - P3::new(1.0, 2.0, 2.5)).norm() < 1e-9)
            })
            .expect("the rise at (1, 2, 2.5)");
        // The fillet builds (ADR-0043); the census names the vertex it
        // would have refused, the rise's foot on the top edge.
        let mut scratch = m.clone();
        assert!(blend(&mut scratch, fused, &[rise], 0.1).is_ok());
        use arris_topo::{Orientation, Shape};
        let foot = [
            m.edge(rise.id).unwrap().start(),
            m.edge(rise.id).unwrap().end(),
        ]
        .into_iter()
        .find(|&v| m.vertex(v).unwrap().point().z < 2.1)
        .expect("the rise's foot");
        let e = OpError::Degenerate {
            entities: vec![
                Shape::new(rise.id, Orientation::Forward),
                Shape::new(foot, Orientation::Forward),
            ],
            reason: Reason::Blend(BlendReason::VertexBlend),
        };
        assert_eq!(
            vertex_blend_cause(&m, rise.id, &e).unwrap(),
            "a vertex of 5 edges, the face across met twice; edge plane × plane"
        );
        assert_eq!(run_over_cause(&m, rise.id, &e), None);
        let set = vertex_set_cause(&m, &e).unwrap();
        assert!(
            set.starts_with("1 blended edges at a vertex of 5 edges"),
            "{set}"
        );
    }

    /// The fixture `regression/flush-l-inner-fillet`: the inner edge of an L
    /// of two fused boxes ends at a vertex of four edges whose fourth is the
    /// smooth seam between the cap's two pieces, one plane (ADR-0082). The
    /// census names the site a flush fan.
    #[test]
    fn a_vertex_of_two_flush_pieces_is_named_a_flush_fan() {
        use crate::unmetered::{fuse, primitive_box};
        use arris_topo::{Orientation, Shape};
        let mut m = Model::default();
        let (a, _) = primitive_box(&mut m, P3::new(0.0, 0.0, 0.0), P3::new(2.0, 1.0, 2.0)).unwrap();
        let (b, _) = primitive_box(&mut m, P3::new(0.0, 1.0, 0.0), P3::new(1.0, 2.0, 2.0)).unwrap();
        let (l, _) = fuse(&mut m, a, b).unwrap();
        // As a file reads: the operands are not in the model to share edges.
        m.retain(&[l]).unwrap();
        let inner = m
            .edges(l)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id).is_some_and(|p| (p - P3::new(1.0, 1.0, 1.0)).norm() < 1e-9)
            })
            .unwrap();
        let end = m
            .vertices(l)
            .unwrap()
            .into_iter()
            .find(|v| (m.vertex(v.id).unwrap().point() - P3::new(1.0, 1.0, 0.0)).norm() < 1e-9)
            .unwrap();
        let e = OpError::Degenerate {
            entities: vec![
                Shape::new(inner.id, Orientation::Forward),
                Shape::new(end.id, Orientation::Forward),
            ],
            reason: Reason::Blend(BlendReason::VertexBlend),
        };
        assert_eq!(
            vertex_blend_cause(&m, inner.id, &e).unwrap(),
            "a vertex of 4 edges, a flush fan of 2 faces across; edge plane × plane"
        );
    }

    /// The fixture `blend/chamfered-stadium-foot-fillet`: the chamfer's
    /// foot runs on into the half cone's at a vertex of four edges where
    /// both faces turn (ADR-0039), and the junction there builds. A
    /// junction refused at such a vertex, naming the run's two edges and
    /// the vertex, is named by them.
    #[test]
    fn a_junction_at_a_vertex_of_four_edges_is_named_by_the_run() {
        use crate::unmetered::{chamfer, extrude};
        use arris_geom::{Profile, ProfileLoop, ProfileSegment};
        use arris_math::{Frame, Point2};
        let mut m = Model::default();
        let p = |u, v| Point2::new(u, v);
        let profile = Profile {
            plane: Frame::world(),
            outer: ProfileLoop::Path {
                start: p(0.0, -1.0),
                segments: vec![
                    ProfileSegment::LineTo(p(2.0, -1.0)),
                    ProfileSegment::ArcTo {
                        to: p(2.0, 1.0),
                        via: p(3.0, 0.0),
                    },
                    ProfileSegment::LineTo(p(0.0, 1.0)),
                    ProfileSegment::ArcTo {
                        to: p(0.0, -1.0),
                        via: p(-1.0, 0.0),
                    },
                ],
            },
            holes: Vec::new(),
        };
        let (stadium, _) = extrude(&mut m, &profile, Vec3::z(), 1.0).unwrap();
        let at = |m: &Model, body, q: P3| {
            m.edges(body)
                .unwrap()
                .into_iter()
                .find(|h| edge_midpoint(m, h.id).is_some_and(|p| (p - q).norm() < 1e-9))
                .unwrap()
        };
        let top = at(&m, stadium, P3::new(1.0, 1.0, 1.0));
        let (chamfered, _) = chamfer(&mut m, stadium, &[top], 0.25).unwrap();
        let foot = at(&m, chamfered, P3::new(1.0, 1.0, 0.75));
        let mut scratch = m.clone();
        assert!(blend(&mut scratch, chamfered, &[foot], 0.1).is_ok());
        // The foot line's end at the corner (0, 1), and the foot arc there.
        let corner = m
            .vertices(chamfered)
            .unwrap()
            .into_iter()
            .find(|v| (m.vertex(v.id).unwrap().point() - P3::new(0.0, 1.0, 0.75)).norm() < 1e-9)
            .unwrap();
        let arc = at(&m, chamfered, P3::new(-1.0, 0.0, 0.75));
        use arris_topo::{Orientation, Shape};
        let e = OpError::Degenerate {
            entities: vec![
                Shape::new(foot.id, Orientation::Forward),
                Shape::new(arc.id, Orientation::Forward),
                Shape::new(corner.id, Orientation::Forward),
            ],
            reason: Reason::Blend(BlendReason::VertexBlend),
        };
        assert_eq!(
            vertex_blend_cause(&m, foot.id, &e).unwrap(),
            "2 edges of the run meet at a vertex of 4 edges; edge plane × plane"
        );
    }

    /// A refusal that is not `VertexBlend` has no site here, and a set's
    /// miter is named by how many edges meet and what they separate.
    #[test]
    fn only_a_vertex_blend_has_a_vertex_site() {
        use arris_topo::{Orientation, Shape};
        let mut m = Model::default();
        let (b, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 3.0, 4.0)).unwrap();
        let handles = m.edges(b).unwrap();
        let other = OpError::Degenerate {
            entities: Vec::new(),
            reason: Reason::Blend(BlendReason::TooLarge),
        };
        assert_eq!(vertex_blend_cause(&m, handles[0].id, &other), None);
        assert_eq!(vertex_set_cause(&m, &other), None);
        let v = m.edge(handles[0].id).unwrap().start();
        let shape = |id: EdgeId| Shape::new(id, Orientation::Forward);
        let miter = OpError::Degenerate {
            entities: vec![
                shape(handles[0].id),
                shape(handles[1].id),
                Shape::new(v, Orientation::Forward),
            ],
            reason: Reason::Blend(BlendReason::VertexBlend),
        };
        assert_eq!(
            vertex_set_cause(&m, &miter).unwrap(),
            "2 blended edges at a vertex of 3 edges; plane × plane"
        );
        let curveless = OpError::Degenerate {
            entities: vec![shape(handles[0].id)],
            reason: Reason::Blend(BlendReason::VertexBlend),
        };
        assert_eq!(
            vertex_blend_cause(&m, handles[0].id, &curveless).unwrap(),
            "a blended edge with no curve; edge plane × plane"
        );
    }

    /// The table keeps the sites apart and totals Open CASCADE's verdicts,
    /// and says what the sample together was.
    #[test]
    fn the_vertex_blend_table_counts_by_site_and_verdict() {
        let edge = |cause: &str, occt: Option<&str>| RunOverEdge {
            edge: "e".into(),
            at: [0.0; 3],
            cause: cause.into(),
            sampled: false,
            occt: occt.map(str::to_string),
        };
        let solid = SolidCensus {
            solid: "#1[0]".into(),
            radius: 1.0,
            sample: None,
            blocks: None,
            sampled: Vec::new(),
            alone: BTreeMap::new(),
            too_large: Vec::new(),
            tangent_chain: Vec::new(),
            vertex_blend: vec![
                edge("a vertex of 5 edges", Some("builds")),
                edge("a vertex of 5 edges", Some("refuses")),
                edge("a vertex of 5 edges", None),
            ],
            vertex_set: Some(VertexSet {
                cause: "2 blended edges".into(),
                points: Vec::new(),
                each_alone_builds: true,
                occt: Some("builds".into()),
            }),
        };
        let table = vertex_blend_markdown(&[("p".into(), vec![solid])]);
        assert!(
            table.contains(
                "| p | #1[0] | a vertex of 5 edges | blend network | 3 | 2 | 1 | 0 | 1 | 0 |"
            ),
            "{table}"
        );
        assert!(
            table.contains(
                "- p #1[0]: 2 blended edges (each edge alone builds) → Open CASCADE builds"
            ),
            "{table}"
        );
        assert!(
            table.contains("Total: 3 `VertexBlend` edges, 2 put to Open CASCADE"),
            "{table}"
        );
    }

    /// The committed `nist-ftc-06`'s holes: a rim circle in two half arcs
    /// between one plane and one cylinder, the cylinder's seam at the first
    /// vertex and nothing but the two arcs at the second. The chain runs on
    /// through both (ADR-0041), so no edge of the part is refused at a
    /// vertex of two edges that continue one another, and the arc the
    /// census once named there blends.
    #[test]
    fn no_split_rims_second_vertex_is_refused() {
        let file = corpus_root().join("real/nist-ftc-06/nist_ftc_06_asme1_rd.stp");
        let bytes = std::fs::read(&file).unwrap();
        let mut m = Model::new(PrecisionSpec::default().precision()).unwrap();
        let read = step_read(
            &mut m,
            &String::from_utf8_lossy(&bytes),
            &ReadOptions::default(),
        )
        .unwrap();
        let body = read
            .solids
            .iter()
            .find_map(|s| s.result.as_ref().ok())
            .unwrap()
            .body;
        let handles = m.edges(body).unwrap();
        let mut arcs = 0;
        for h in &handles {
            let mut scratch = m.clone();
            match blend(&mut scratch, body, &[*h], 0.05) {
                Err(Refused::Op(e)) => {
                    let cause = vertex_blend_cause(&m, h.id, &e).unwrap_or_default();
                    assert!(
                        !cause.contains("a continuation between the same two faces"),
                        "{cause}"
                    );
                }
                Ok(_) => {
                    let entity = m.edge(h.id).unwrap();
                    let at = |v| m.vertex_edges(v).unwrap().len();
                    if entity.start() != entity.end()
                        && (at(entity.start()) == 2 || at(entity.end()) == 2)
                    {
                        arcs += 1;
                    }
                }
                Err(_) => {}
            }
        }
        assert!(
            arcs > 0,
            "a split rim's arc blends through its second vertex"
        );
    }

    /// The committed `nist-ftc-06`'s three `VertexBlend` edges are one
    /// straight run on one face whose second face jumps at a vertex of four
    /// edges: the census names "a dihedral jump on a collinear run" and
    /// attributes it to the NURBS cycle, the patch having no closed form.
    #[test]
    fn ftc_06_run_is_a_dihedral_jump_of_the_nurbs_cycle() {
        let file = corpus_root().join("real/nist-ftc-06/nist_ftc_06_asme1_rd.stp");
        let solids = census_file(&file, &|_| true).unwrap();
        let edges: Vec<&RunOverEdge> = solids.iter().flat_map(|s| &s.vertex_blend).collect();
        // Three edges are refused at the jump. Four more, chained to a
        // walked stripe's neighbour, are named by the refusal they reach.
        const JUMP: &str = "a dihedral jump on a collinear run; edge plane × plane";
        let (jumps, meets): (Vec<_>, Vec<_>) = edges.into_iter().partition(|e| e.cause == JUMP);
        assert_eq!(jumps.len(), 3, "{jumps:?}");
        assert_eq!(meets.len(), 4, "{meets:?}");
        for e in jumps {
            assert_eq!(vertex_blend_cycle(&e.cause), crate::histogram::Cycle::Nurbs);
        }
        // Since ADR-0072 the four are two jumps of a cylinder against a
        // plane and two vertices of four edges with a smooth extra; before,
        // all four were a miter at a vertex of three.
        let (cylinder_jumps, fours): (Vec<_>, Vec<_>) = meets
            .into_iter()
            .partition(|e| e.cause.ends_with("; edge cylinder × plane"));
        assert_eq!(cylinder_jumps.len(), 2, "{cylinder_jumps:?}");
        assert_eq!(fours.len(), 2, "{fours:?}");
        for e in cylinder_jumps {
            assert!(
                e.cause.starts_with("a dihedral jump on a collinear run"),
                "{}",
                e.cause
            );
        }
        for e in fours {
            assert!(
                e.cause
                    .starts_with("a vertex of 4 edges, the extra: a smooth edge"),
                "{}",
                e.cause
            );
        }
    }

    /// The part of the fixture `dir` before its last step (the blend), the
    /// edge nearest `at` blended alone, and the census's cause: of its
    /// refusal, or where it builds of a `VertexBlend` constructed at its
    /// start, naming the edge and that vertex. `true` where it builds.
    fn vertex_cause_of(dir: &str, at: P3, radius: f64) -> (String, bool) {
        let dir = corpus_root().join(dir);
        let mut fixture: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("fixture.json")).unwrap())
                .unwrap();
        let last = fixture["steps"].as_array_mut().unwrap().pop().unwrap();
        fixture["result"] = fixture["steps"].as_array().unwrap().last().unwrap()["name"].clone();
        assert!(last["op"] == "fillet" || last["op"] == "chamfer");
        let recipe: crate::fixtures::Recipe = serde_json::from_str(&fixture.to_string()).unwrap();
        let chain = crate::corpus::build("generated/fan", &recipe).unwrap();
        let body = chain.result().unwrap();
        let mut m = chain.model;
        m.retain(&[body]).unwrap();
        let edge = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| edge_midpoint(&m, h.id).is_some_and(|p| (p - at).norm() < 1e-6))
            .expect("the blended edge");
        let mut scratch = m.clone();
        let (e, built) = match blend(&mut scratch, body, &[edge], radius) {
            Err(Refused::Op(e)) => (e, false),
            Ok(_) => {
                use arris_topo::{Orientation, Shape};
                let start = m.edge(edge.id).unwrap().start();
                let e = OpError::Degenerate {
                    entities: vec![
                        Shape::new(edge.id, Orientation::Forward),
                        Shape::new(start, Orientation::Forward),
                    ],
                    reason: Reason::Blend(BlendReason::VertexBlend),
                };
                (e, true)
            }
            Err(_) => panic!("the edge is refused, but not by the operation"),
        };
        (vertex_blend_cause(&m, edge.id, &e).unwrap(), built)
    }

    /// The hexagonal prism's chamfered top, the foot of a facet blended: at
    /// each end the faces across are two, separated by a sharp edge, a fan
    /// that builds (ADR-0043) and is named as one where a refusal names its
    /// vertex; `blend/five-edge-vertex`'s are one face met twice, which builds
    /// too.
    #[test]
    fn a_fan_is_named_apart_from_the_face_across_met_twice() {
        for dir in [
            "blend/hex-chamfer-foot-fan-fillet",
            "blend/hex-chamfer-foot-fan-chamfer",
        ] {
            assert_eq!(
                vertex_cause_of(dir, P3::new(0.75, 3f64.sqrt() / 4.0, 1.8), 0.1),
                (
                    "a vertex of 4 edges, a fan of 2 faces across; edge plane × plane".to_string(),
                    true
                ),
                "{dir}"
            );
        }
        assert_eq!(
            vertex_cause_of("blend/five-edge-vertex", P3::new(1.0, 2.0, 2.5), 0.1),
            (
                "a vertex of 5 edges, the face across met twice; edge plane × plane".to_string(),
                true
            )
        );
    }
}
