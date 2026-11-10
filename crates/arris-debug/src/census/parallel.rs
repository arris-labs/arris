//! Ruling edges between parallel cylinders.

use super::vertex::faces_of;
use super::*;

/// One edge between two cylinders with parallel axes, along a ruling of
/// both (ADR-0036 §5), blended alone at the battery's radius.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub struct ParallelCylinderEdge {
    /// The solid, `#id[instance]`.
    pub solid: String,
    /// The edge, as `{:?}` prints its id.
    pub edge: String,
    /// Whether the edge is convex (the material is the wedge, not its
    /// complement).
    pub convex: bool,
    /// What lies across at each end, sorted: the surface kinds of the
    /// faces at the end vertex other than the edge's two, joined by `+`,
    /// or `free` where there are none.
    pub ends: String,
    /// [`BUILT`], [`PANICKED`], or the refusal: its [`class_of`], and for a
    /// `BlendTooLarge` or `VertexBlend` the site that refused.
    pub outcome: String,
}

/// Every blendable edge of `body` that two cylinders with parallel axes
/// share along a ruling, blended alone at the battery's radius: the census
/// the parallel-cylinder row (ADR-0036 §5) is counted by. `solid` labels
/// the rows. No oracle is asked.
///
/// # Errors
/// A walk of `body` fails: never a refusal of the operation, which is the
/// census's data.
pub fn parallel_cylinder_census(
    m: &Model,
    body: Body,
    solid: &str,
) -> Result<Vec<ParallelCylinderEdge>, String> {
    let (blendable, sample) = fillet_edges(m, body)?;
    let mut out = Vec::new();
    if sample.is_empty() {
        return Ok(out);
    }
    let radius = fillet_radius(&sample);
    let handles = m.edges(body).map_err(|e| e.to_string())?;
    let mut orientation: BTreeMap<FaceId, arris_topo::Orientation> = BTreeMap::new();
    for shell in m.shells(body).map_err(|e| e.to_string())? {
        for face_use in m.shell(shell.id).map_err(|e| e.to_string())?.faces() {
            let face = face_use.oriented_by(shell.orientation);
            orientation.insert(face.id, face.orientation);
        }
    }
    let angular = m.precision().angular_tolerance;
    let mut scratch = m.clone();
    for id in blendable {
        let Some(edge) = handles.iter().find(|h| h.id == id).copied() else {
            continue;
        };
        let Some(convex) = parallel_cylinder_convex(m, &orientation, id, angular)? else {
            continue;
        };
        let outcome = match blend(&mut scratch, body, &[edge], radius) {
            Ok(()) => BUILT.to_string(),
            Err(Refused::Op(e)) => run_over_cause(m, id, &e)
                .or_else(|| vertex_blend_cause(m, id, &e))
                .map_or_else(
                    || class_of(&e),
                    |cause| format!("{}: {cause}", class_of(&e)),
                ),
            Err(Refused::Panic(_)) => PANICKED.to_string(),
        };
        out.push(ParallelCylinderEdge {
            solid: solid.to_string(),
            edge: format!("{id:?}"),
            convex,
            ends: ends_across(m, id),
            outcome,
        });
    }
    Ok(out)
}

/// `Some(convex)` where `edge` is a line between two cylinders whose axes
/// are parallel to it; `None` otherwise.
fn parallel_cylinder_convex(
    m: &Model,
    orientation: &BTreeMap<FaceId, arris_topo::Orientation>,
    edge: EdgeId,
    angular: f64,
) -> Result<Option<bool>, String> {
    let e = m.edge(edge).map_err(|e| e.to_string())?;
    let Some((c, range)) = e.curve() else {
        return Ok(None);
    };
    let curve = m.curve(c).map_err(|e| e.to_string())?;
    if !matches!(curve, arris_geom::Curve::Line { .. }) {
        return Ok(None);
    }
    let mid = range.midpoint();
    let along = curve.eval(mid).d1;
    let uses = m.edge_uses(edge).map_err(|e| e.to_string())?;
    let [ua, ub] = uses else {
        return Ok(None);
    };
    let mut normals = Vec::new();
    for u in [ua, ub] {
        let face = m.face(u.face).map_err(|e| e.to_string())?;
        let surface = m.surface(face.surface()).map_err(|e| e.to_string())?;
        let arris_geom::Surface::Cylinder { frame, .. } = surface else {
            return Ok(None);
        };
        if frame.z().cross(&along).norm() > angular * along.norm() {
            return Ok(None);
        }
        let coedge = face
            .loops()
            .get(u.loop_index)
            .and_then(|l| l.coedges().get(u.coedge_index))
            .ok_or("an edge use names no coedge")?;
        let uv = m
            .curve2(coedge.pcurve())
            .map_err(|e| e.to_string())?
            .point(mid);
        let Some(n) = surface.normal(uv.x, uv.y) else {
            return Ok(None);
        };
        let sign = orientation.get(&u.face).ok_or("a face of no shell")?.sign();
        normals.push((n.into_inner() * sign, coedge.orientation()));
    }
    let [(n1, o1), (n2, _)] = normals[..] else {
        return Ok(None);
    };
    let t1 = along * orientation[&ua.face].compose(o1).sign();
    Ok(Some(n1.cross(&t1).dot(&n2) < 0.0))
}

/// What lies across at each end of `edge`: [`ParallelCylinderEdge::ends`].
fn ends_across(m: &Model, edge: EdgeId) -> String {
    let own = faces_of(m, edge);
    let Ok(e) = m.edge(edge) else {
        return String::new();
    };
    let mut ends: Vec<String> = [e.start(), e.end()]
        .into_iter()
        .map(|v| {
            let mut kinds: Vec<String> = m
                .vertex_edges(v)
                .map(|es| es.to_vec())
                .unwrap_or_default()
                .into_iter()
                .flat_map(|x| faces_of(m, x))
                .filter(|f| !own.contains(f))
                .collect::<std::collections::BTreeSet<_>>()
                .into_iter()
                .filter_map(|f| face_kind(m, f))
                .collect();
            kinds.sort();
            if kinds.is_empty() {
                "free".to_string()
            } else {
                kinds.join("+")
            }
        })
        .collect();
    ends.sort();
    ends.join(" / ")
}

/// [`parallel_cylinder_census`] over every solid of a STEP file the
/// `wanted` predicate keeps, as [`census_file`] reads it.
///
/// # Errors
/// The file does not read, or a solid's walk fails.
pub fn parallel_cylinder_census_file(
    file: &Path,
    wanted: &dyn Fn(&str) -> bool,
) -> Result<Vec<ParallelCylinderEdge>, String> {
    let bytes = std::fs::read(file).map_err(|e| format!("{}: {e}", file.display()))?;
    let mut m = Model::new(PrecisionSpec::default().precision()).map_err(|e| e.to_string())?;
    let read = step_read(
        &mut m,
        &String::from_utf8_lossy(&bytes),
        &ReadOptions::default(),
    )
    .map_err(|e| format!("{}: {e}", file.display()))?;
    let mut seen = Vec::new();
    let mut out = Vec::new();
    for s in &read.solids {
        let Ok(back) = &s.result else { continue };
        if seen.contains(&s.entity.id) {
            continue;
        }
        seen.push(s.entity.id);
        let key = crate::battery::key(s.entity.id, s.entity.instance);
        if !wanted(&key) {
            continue;
        }
        out.extend(parallel_cylinder_census(&m, back.body, &key)?);
    }
    Ok(out)
}
