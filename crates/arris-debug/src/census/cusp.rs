//! Cusp pairs blended together.

use super::tangent::{leaving_vertex, walls_fold};
use super::*;

/// How far from 1 the cosine of the angle between two edges leaving a
/// vertex may be for the census to call them tangent (a cusp's arcs).
const CUSP_TANGENT: f64 = 1e-6;

/// One pair of edges that meet a tangent edge at a cusp and are blended
/// together (ADR-0084): the join a single-edge request cannot reach.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CuspPair {
    /// The solid, `#id[instance]`.
    pub solid: String,
    /// The two edges, as `{:?}` prints their ids.
    pub edges: [String; 2],
    /// The surface kinds each edge separates (`pair_of`).
    pub kinds: [String; 2],
    /// Whether the cusp's walls fold onto one side of the face the edges
    /// share (`None` where it cannot be read).
    pub walls_one_side: Option<bool>,
    /// The fillet radius of the census.
    pub radius: f64,
    /// [`BUILT`], [`PANICKED`], or the refusal's [`class_of`], of the pair
    /// blended together.
    pub outcome: String,
}

/// Every cusp of `body` whose two edges both blend (two blendable edges that
/// leave a vertex of three edges along one tangent), each pair blended
/// together at the battery's radius. `solid` labels the rows. No oracle is
/// asked.
///
/// # Errors
/// A walk of `body` fails: never a refusal of the operation, which is a row.
pub fn cusp_pairs(m: &Model, body: Body, solid: &str) -> Result<Vec<CuspPair>, String> {
    let (blendable, sample) = fillet_edges(m, body)?;
    if sample.is_empty() {
        return Ok(Vec::new());
    }
    let radius = fillet_radius(&sample);
    let handles = m.edges(body).map_err(|e| e.to_string())?;
    let handle = |id| handles.iter().find(|h| h.id == id).copied();
    let mut scratch = m.clone();
    let mut found: BTreeMap<(EdgeId, EdgeId), Option<bool>> = BTreeMap::new();
    let mut vertices: Vec<VertexId> = Vec::new();
    for &id in &blendable {
        if let Ok(e) = m.edge(id) {
            vertices.extend([e.start(), e.end()]);
        }
    }
    vertices.sort();
    vertices.dedup();
    for v in vertices {
        let Ok(at) = m.vertex_edges(v) else { continue };
        let [x, y, z] = at[..] else { continue };
        // The two edges that leave `v` along one tangent are the cusp's
        // arcs; the third is the section edge `w`.
        for (a, b, w) in [(x, y, z), (x, z, y), (y, z, x)] {
            if a == b || !blendable.contains(&a) || !blendable.contains(&b) {
                continue;
            }
            let (Some(da), Some(db)) = (leaving_vertex(m, a, v), leaving_vertex(m, b, v)) else {
                continue;
            };
            if da.dot(&db) < 1.0 - CUSP_TANGENT {
                continue;
            }
            let key = if a < b { (a, b) } else { (b, a) };
            found.insert(key, walls_fold(m, w, v));
        }
    }
    let mut out = Vec::new();
    for ((a, b), walls) in found {
        let (Some(ha), Some(hb)) = (handle(a), handle(b)) else {
            continue;
        };
        let outcome = match blend(&mut scratch, body, &[ha, hb], radius) {
            Ok(()) => BUILT.to_string(),
            Err(Refused::Op(e)) => class_of(&e),
            Err(Refused::Panic(_)) => PANICKED.to_string(),
        };
        out.push(CuspPair {
            solid: solid.to_string(),
            edges: [format!("{a:?}"), format!("{b:?}")],
            kinds: [
                pair_of(m, a).unwrap_or_default(),
                pair_of(m, b).unwrap_or_default(),
            ],
            walls_one_side: walls,
            radius,
            outcome,
        });
    }
    Ok(out)
}

/// [`cusp_pairs`] of every solid of `file`, one [`CuspPair`] per pair, in
/// the file's solid order.
///
/// # Errors
/// The file cannot be read, or [`cusp_pairs`] fails.
pub fn cusp_pairs_file(file: &Path) -> Result<Vec<CuspPair>, String> {
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
        out.extend(cusp_pairs(&m, back.body, &key)?);
    }
    Ok(out)
}

/// The cusp pairs of the committed tier.
///
/// # Errors
/// A fixture cannot be loaded, or [`cusp_pairs_file`] fails.
pub fn cusp_pairs_committed() -> Result<Vec<(String, Vec<CuspPair>)>, String> {
    over_committed_tier(false, |file, _| cusp_pairs_file(file))
}

/// The census of cusp pairs: one row per part and outcome, with the count.
pub fn cusp_pairs_markdown(parts: &[(String, Vec<CuspPair>)]) -> String {
    let mut by: BTreeMap<(&str, &str, Option<bool>), usize> = BTreeMap::new();
    let mut total = 0;
    for (part, pairs) in parts {
        for p in pairs {
            *by.entry((part, &p.outcome, p.walls_one_side)).or_default() += 1;
            total += 1;
        }
    }
    let mut out = format!("cusp pairs: {total} in {} parts\n\n", parts.len());
    for ((part, outcome, walls), n) in by {
        let side = match walls {
            Some(true) => "walls on one side",
            Some(false) => "walls on either side",
            None => "walls unread",
        };
        let _ = writeln!(out, "- {part}: {n} × {outcome} ({side})");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The crescent prism of `blend/cusp-crescent-both-arcs-fillet` has one
    /// cusp pair, its two top arcs, whose walls fold onto one side of the
    /// top they share.
    #[test]
    fn the_crescent_has_one_cusp_pair() {
        let dir = corpus_root().join("blend/cusp-crescent-both-arcs-fillet");
        let chain = crate::corpus::chain(&dir, "default").unwrap();
        let part = chain.steps["result"].inputs[0];
        let pairs = cusp_pairs(&chain.model, part, "crescent").unwrap();
        assert_eq!(pairs.len(), 1, "{pairs:?}");
        let pair = &pairs[0];
        assert_eq!(pair.solid, "crescent");
        assert_ne!(pair.edges[0], pair.edges[1]);
        assert_eq!(pair.walls_one_side, Some(true), "{pair:?}");
        assert!(pair.radius > 0.0);
        assert!(!pair.outcome.is_empty());
    }
}
