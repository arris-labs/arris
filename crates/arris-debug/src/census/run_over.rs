//! Run-over refusal sites and oracle verdicts.

use super::*;

/// One edge refused `BlendTooLarge` when blended alone.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RunOverEdge {
    /// The edge, as `{:?}` prints its id.
    pub edge: String,
    /// The midpoint of its curve, the oracle's name for it.
    pub at: [f64; 3],
    /// [`run_over_cause`]'s sub-cause.
    pub cause: String,
    /// One of the battery's sampled edges: the four that decide whether the
    /// part leaves the fillet column.
    #[serde(default)]
    pub sampled: bool,
    /// Open CASCADE's verdict on this edge alone at the census's radius
    /// (`builds`, `invalid`, `refuses`, `no-edge`); `None` where it was not
    /// asked.
    pub occt: Option<String>,
}

/// Which site refused a `BlendTooLarge` of `edge` blended alone, from the
/// entities the error names (`None` for any other refusal): *the contact
/// leaves a face* (the edge and the face it runs out of), *the ball finds
/// no place on either face* (the edge and both faces), *a corner edge or a
/// seam is shorter than the trim* (the edge and that edge), *a corner of
/// three edges whose trims run past one another* (three edges), *the
/// blend's closing or its fit* (the edge alone). Each is followed by the
/// pair of surface kinds the blended edge separates, which is what a
/// construction is keyed by.
pub fn run_over_cause(m: &Model, edge: EdgeId, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::Blend(BlendReason::TooLarge),
    } = e
    else {
        return None;
    };
    let pair = pair_of(m, edge).unwrap_or_default();
    let ids: Vec<EntityId> = entities.iter().map(|s| s.id).collect();
    let what = match ids.as_slice() {
        [EntityId::Edge(_), EntityId::Face(f)] => {
            format!(
                "the contact leaves a {} face",
                face_kind(m, *f).unwrap_or_default()
            )
        }
        [EntityId::Edge(_), EntityId::Face(_), EntityId::Face(_)] => {
            "the ball finds no place on either face".to_string()
        }
        [EntityId::Edge(_), EntityId::Edge(other)] => {
            let seam = m
                .edge_uses(*other)
                .is_ok_and(|u| u.len() == 2 && u[0].face == u[1].face);
            if seam {
                "a seam shorter than the trim".to_string()
            } else {
                "a corner edge shorter than the trim".to_string()
            }
        }
        [EntityId::Edge(_), EntityId::Edge(_), EntityId::Edge(_)] => {
            "a corner whose trims run past one another".to_string()
        }
        [EntityId::Edge(_)] => "the closing or the fit".to_string(),
        _ => "another site".to_string(),
    };
    Some(format!("{what}; edge {pair}"))
}

pub(super) fn solid_id(solid: &str) -> Result<u64, String> {
    solid
        .trim_start_matches('#')
        .split('[')
        .next()
        .and_then(|s| s.parse().ok())
        .ok_or_else(|| format!("{solid}: not a solid key"))
}

/// The indices a stride over each cause, in id order, asks the oracle
/// about ([`ASKED_PER_CAUSE`] of each), and every sampled edge besides.
pub(super) fn stride_per_cause(edges: &[RunOverEdge]) -> Vec<usize> {
    let mut by_cause: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for (i, e) in edges.iter().enumerate() {
        by_cause.entry(&e.cause).or_default().push(i);
    }
    let mut asked: Vec<usize> = Vec::new();
    for list in by_cause.values() {
        let stride = list.len().div_ceil(ASKED_PER_CAUSE).max(1);
        asked.extend(list.iter().step_by(stride));
    }
    asked.extend((0..edges.len()).filter(|&i| edges[i].sampled));
    asked.sort_unstable();
    asked.dedup();
    asked
}

/// How many edges of one (solid, cause) [`ask_the_oracle`] puts to Open
/// CASCADE: a stride over them in id order. A cause with fewer is asked
/// whole.
pub const ASKED_PER_CAUSE: usize = 12;

/// How close to an edge a census point must lie for Open CASCADE to name
/// that edge by it: the midpoints agree to the file's own digits, and the
/// edges of a part are far further apart.
pub const ORACLE_PROBE: f64 = 1e-4;

/// Puts a sample of every cause's `BlendTooLarge` edges of `census`, a
/// solid of `file`, to Open CASCADE alone at the census's radius, and
/// records each verdict in `occt`. At most [`ASKED_PER_CAUSE`] per cause,
/// and every edge of the battery's sample besides.
///
/// # Errors
/// The solid's key is not `#id[instance]`, or the oracle could not run.
pub fn ask_the_oracle(file: &Path, census: &mut SolidCensus) -> Result<(), String> {
    let id = solid_id(&census.solid)?;
    let asked = stride_per_cause(&census.too_large);
    if asked.is_empty() {
        return Ok(());
    }
    let points: Vec<[f64; 3]> = asked.iter().map(|&i| census.too_large[i].at).collect();
    let verdicts = crate::oracle::fillet_edges(file, id, census.radius, ORACLE_PROBE, &points)
        .map_err(|e| e.to_string())?;
    for (&i, v) in asked.iter().zip(verdicts) {
        census.too_large[i].occt = Some(v.verdict);
    }
    Ok(())
}

/// The `BlendTooLarge` edges of the solids, counted by `part → cause`:
/// how many edges, how many were put to Open CASCADE and what it said.
pub fn run_over_markdown(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut table = Table::new([
        ("part", Align::Left),
        ("solid", Align::Left),
        ("cause", Align::Left),
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
            for e in &s.too_large {
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
    out.push_str("\nThe battery's sampled edges refused `BlendTooLarge`, each alone:\n\n");
    for (part, solids) in parts {
        for s in solids {
            for e in s.too_large.iter().filter(|e| e.sampled) {
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
    let _ = writeln!(
        out,
        "\nTotal: {} `BlendTooLarge` edges, {} put to Open CASCADE: {} build, {} build invalid, {} refused, {} found no edge.",
        totals[0], totals[1], totals[2], totals[3], totals[4], totals[5]
    );
    out
}

/// [`census_file`] of `file`, and each solid's `BlendTooLarge` edges put to
/// Open CASCADE ([`ask_the_oracle`]).
///
/// # Errors
/// As [`census_file`] and [`ask_the_oracle`].
pub fn run_over_file(
    file: &Path,
    wanted: &dyn Fn(&str) -> bool,
) -> Result<Vec<SolidCensus>, String> {
    let mut solids = census_file(file, wanted)?;
    for s in &mut solids {
        ask_the_oracle(file, s)?;
    }
    Ok(solids)
}

/// [`committed`] with each solid's `BlendTooLarge` edges put to Open CASCADE.
///
/// # Errors
/// As [`committed`] and [`ask_the_oracle`].
pub fn run_over_committed() -> Result<Vec<(String, Vec<SolidCensus>)>, String> {
    over_committed_tier(true, run_over_file)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::unmetered::primitive_box;
    use arris_math::Point3 as P3;

    /// A box fillet at a radius past its faces is refused at its corner
    /// edge, named with the pair its edge separates; a radius that builds
    /// is no refusal and has no cause, and nor is another reason.
    #[test]
    fn a_radius_past_the_faces_meets_the_corner_edge() {
        let mut m = Model::default();
        let (b, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 3.0, 4.0)).unwrap();
        let handles = m.edges(b).unwrap();
        let edge = handles[0];
        let mut scratch = m.clone();
        let refused = match blend(&mut scratch, b, &[edge], 5.0) {
            Err(Refused::Op(e)) => e,
            other => panic!(
                "a radius of 5 on a 2 × 3 × 4 box is refused: {}",
                other.is_ok()
            ),
        };
        let cause = run_over_cause(&m, edge.id, &refused).expect("a BlendTooLarge");
        assert_eq!(
            cause,
            "a corner edge shorter than the trim; edge plane × plane"
        );
        let mut scratch = m.clone();
        assert!(blend(&mut scratch, b, &[edge], 0.1).is_ok());
        let other = OpError::Degenerate {
            entities: Vec::new(),
            reason: Reason::Blend(BlendReason::NoEdges),
        };
        assert_eq!(run_over_cause(&m, edge.id, &other), None);
    }

    /// Each shape of entity list is its own site.
    #[test]
    fn the_entities_a_refusal_names_decide_its_site() {
        use arris_topo::{Orientation, Shape};
        let mut m = Model::default();
        let (b, _) = primitive_box(&mut m, P3::origin(), P3::new(2.0, 3.0, 4.0)).unwrap();
        let handles = m.edges(b).unwrap();
        let edge = handles[0].id;
        let uses = m.edge_uses(edge).unwrap().to_vec();
        let e = |id: EdgeId| Shape::new(id, Orientation::Forward);
        let f = |u: usize| Shape::new(uses[u].face, Orientation::Forward);
        let cause = |entities: Vec<Shape>| {
            let err = OpError::Degenerate {
                entities,
                reason: Reason::Blend(BlendReason::TooLarge),
            };
            run_over_cause(&m, edge, &err).unwrap()
        };
        let pair = "edge plane × plane";
        assert_eq!(
            cause(vec![e(edge), f(0)]),
            format!("the contact leaves a plane face; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge), f(0), f(1)]),
            format!("the ball finds no place on either face; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge), e(handles[1].id)]),
            format!("a corner edge shorter than the trim; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge), e(handles[1].id), e(handles[2].id)]),
            format!("a corner whose trims run past one another; {pair}")
        );
        assert_eq!(
            cause(vec![e(edge)]),
            format!("the closing or the fit; {pair}")
        );
    }

    /// The census's table keeps a cause apart from its pair and totals
    /// what the oracle said.
    #[test]
    fn the_run_over_table_counts_by_cause_and_verdict() {
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
            vertex_blend: Vec::new(),
            vertex_set: None,
            tangent_chain: Vec::new(),
            too_large: vec![
                edge("a; edge plane × plane", Some("builds")),
                edge("a; edge plane × plane", Some("refuses")),
                edge("a; edge plane × plane", None),
                edge("b; edge cone × plane", Some("refuses")),
            ],
        };
        let table = run_over_markdown(&[("p".into(), vec![solid])]);
        assert!(
            table.contains("| p | #1[0] | a; edge plane × plane | 3 | 2 | 1 | 0 | 1 | 0 |"),
            "{table}"
        );
        assert!(
            table.contains("| p | #1[0] | b; edge cone × plane | 1 | 1 | 0 | 0 | 1 | 0 |"),
            "{table}"
        );
        assert!(table.contains("Total: 4 `BlendTooLarge` edges, 3 put to Open CASCADE: 1 build, 0 build invalid, 2 refused"), "{table}");
    }
}
