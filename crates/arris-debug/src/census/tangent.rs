//! Tangent-chain refusal sites and oracle verdicts.

use super::run_over::{solid_id, stride_per_cause};
use super::vertex::tangent_at_vertex;
use super::*;

/// How far from the vertex, as a fraction of the tangent edge's chord, the
/// census probes a face's height above the common tangent plane.
const PROBE_STEP: f64 = 1e-2;

/// A height above the tangent plane under this fraction of the probe step
/// is a flat face's: the census names a site, so the cut is a wide one.
const FLAT: f64 = 1e-9;

/// Which way the faces of the tangent edge `w` curve at `vertex`, each the
/// side of their common tangent plane that the surface lies on a step away
/// along the plane and across `w`: `Some(true)` if opposite sides (an
/// inflection), `Some(false)` if one, `None` if either face is flat or
/// cannot be read.
fn turn_across(m: &Model, w: EdgeId, vertex: VertexId) -> Option<Option<bool>> {
    let uses = m.edge_uses(w).ok()?;
    let [a, b] = uses else { return None };
    let at = m.vertex(vertex).ok()?.point();
    let t = tangent_at_vertex(m, w, vertex)?;
    let entity = m.edge(w).ok()?;
    let chord = (m.vertex(entity.start()).ok()?.point() - m.vertex(entity.end()).ok()?.point())
        .norm()
        .max(t.norm());
    let step = PROBE_STEP * chord;
    let surface = |f| m.surface(m.face(f).ok()?.surface()).ok();
    let (sa, sb) = (surface(a.face)?, surface(b.face)?);
    let uv = sa.project(at).ok()?.uv;
    let n0 = sa.normal(uv.x, uv.y)?.into_inner();
    let across = n0.cross(&t).normalize();
    let height = |s: &arris_geom::Surface| {
        let q = at + across * step;
        let on = s.project(q).ok()?;
        Some((s.point(on.uv.x, on.uv.y) - at).dot(&n0))
    };
    let (ha, hb) = (height(sa)?, height(sb)?);
    if ha.abs() <= FLAT * step || hb.abs() <= FLAT * step {
        return Some(None);
    }
    Some(Some((ha > 0.0) != (hb > 0.0)))
}

/// The unit direction in which `edge` leaves `vertex`.
pub(super) fn leaving_vertex(m: &Model, edge: EdgeId, vertex: VertexId) -> Option<Vec3> {
    let d = tangent_at_vertex(m, edge, vertex)?;
    let away = if m.edge(edge).ok()?.start() == vertex {
        d
    } else {
        -d
    };
    Some(away.normalize())
}

/// Whether, at `vertex` of three edges, `edge` and the third edge beside the
/// tangent `w` leave it the same way: the outline then doubles back on
/// itself there (a cusp) instead of running on through.
fn doubles_back(m: &Model, edge: EdgeId, w: EdgeId, vertex: VertexId) -> Option<bool> {
    let at = m.vertex_edges(vertex).ok()?;
    let [next] = at
        .iter()
        .copied()
        .filter(|&x| x != edge && x != w)
        .collect::<Vec<_>>()[..]
    else {
        return None;
    };
    let (a, b) = (
        leaving_vertex(m, edge, vertex)?,
        leaving_vertex(m, next, vertex)?,
    );
    Some(a.dot(&b) > 0.0)
}

/// Whether the two faces of the tangent edge `w` fold back at `vertex`:
/// their outward normals there opposite, both walls of a cusp on one side
/// of the face the cusp's edges share (a knife-edge sliver, material or
/// void), rather than equal, the walls on either side of it (an overhang
/// tip, ADR-0042). Each normal is its face's in its shell, whose own sense
/// in the body is the same for both and cancels. `None` where a face's
/// normal or sense cannot be read.
pub(super) fn walls_fold(m: &Model, w: EdgeId, vertex: VertexId) -> Option<bool> {
    let uses = m.edge_uses(w).ok()?;
    let [a, b] = uses else { return None };
    let at = m.vertex(vertex).ok()?.point();
    let outward = |face| -> Option<Vec3> {
        let surface = m.surface(m.face(face).ok()?.surface()).ok()?;
        let uv = surface.project(at).ok()?.uv;
        let n = surface.normal(uv.x, uv.y)?.into_inner();
        let shell = *m.face_shells(face).ok()?.first()?;
        let sense = m
            .shell(shell)
            .ok()?
            .faces()
            .iter()
            .find(|f| f.id == face)?
            .orientation
            .sign();
        Some(n * sense)
    };
    Some(outward(a.face)?.dot(&outward(b.face)?) < 0.0)
}

/// Which site refused a `TangentChain` of `edge` blended alone, from the
/// entities the error names (`None` for any other refusal): *the edge
/// itself*, a tangent dihedral (the error names the edge and its two
/// faces), or *an end* at a vertex whose corner edge is tangent (the edge,
/// the corner edge and the vertex: the edge is the run's, not always the
/// one asked), by what is there — *a cusp* (the
/// blended edge and the third edge leave the vertex the same way, the
/// outline doubling back instead of running on), with its walls *on one
/// side* of the face the two edges share, which a blend of one sense is cut
/// at and is refused only where the stripe is not a ring's, or *on either
/// side*, an overhang tip (ADR-0042), else by what turns:
/// *an inflection*
/// (the two faces of the tangent edge curve opposite ways: a floor whose
/// wall is an S-bend), *a turn one way* (they curve the same way), *a flat
/// face against a curved one*, or *a turn the census cannot read*. Each is
/// followed by the pair of surface kinds the blended edge separates.
pub fn tangent_chain_cause(m: &Model, edge: EdgeId, e: &OpError) -> Option<String> {
    let OpError::Degenerate {
        entities,
        reason: Reason::Blend(BlendReason::TangentChain),
    } = e
    else {
        return None;
    };
    let pair = pair_of(m, edge).unwrap_or_default();
    let ids: Vec<EntityId> = entities.iter().map(|s| s.id).collect();
    let what = match ids.as_slice() {
        [EntityId::Edge(_), EntityId::Face(_), EntityId::Face(_)] => {
            "the edge itself, a tangent dihedral".to_string()
        }
        [EntityId::Edge(run), EntityId::Edge(w), EntityId::Vertex(v)] => {
            if doubles_back(m, *run, *w, *v) == Some(true) {
                match walls_fold(m, *w, *v) {
                    Some(true) => "an end at a cusp, walls on one side".to_string(),
                    Some(false) => "an end at a cusp, walls on either side".to_string(),
                    None => "an end at a cusp".to_string(),
                }
            } else {
                match turn_across(m, *w, *v) {
                    Some(Some(true)) => "an end at an inflection".to_string(),
                    Some(Some(false)) => "an end at a turn one way".to_string(),
                    Some(None) => "an end at a flat face against a curved one".to_string(),
                    None => "an end at a turn the census cannot read".to_string(),
                }
            }
        }
        _ => "another site".to_string(),
    };
    Some(format!("{what}; edge {pair}"))
}

/// Puts a sample of every site's `TangentChain` edges of `census`, a solid
/// of `file`, to Open CASCADE alone at the census's radius, as
/// [`ask_the_oracle`] does for `BlendTooLarge`.
///
/// # Errors
/// As [`ask_the_oracle`].
pub fn ask_the_oracle_tangent(file: &Path, census: &mut SolidCensus) -> Result<(), String> {
    let id = solid_id(&census.solid)?;
    let asked = stride_per_cause(&census.tangent_chain);
    if asked.is_empty() {
        return Ok(());
    }
    let points: Vec<[f64; 3]> = asked.iter().map(|&i| census.tangent_chain[i].at).collect();
    let verdicts = crate::oracle::fillet_edges(file, id, census.radius, ORACLE_PROBE, &points)
        .map_err(|e| e.to_string())?;
    for (&i, v) in asked.iter().zip(verdicts) {
        census.tangent_chain[i].occt = Some(v.verdict);
    }
    Ok(())
}

/// [`census_file`] of `file`, with each solid's `TangentChain` edges put to
/// Open CASCADE ([`ask_the_oracle_tangent`]), printed by site and verdict.
///
/// # Errors
/// As [`census_file`] and [`ask_the_oracle_tangent`].
pub fn tangent_chain_file(file: &Path) -> Result<String, String> {
    let mut out = String::new();
    for mut s in census_file(file, &|_| true)? {
        ask_the_oracle_tangent(file, &mut s)?;
        let mut by: BTreeMap<(&str, &str), usize> = BTreeMap::new();
        for e in &s.tangent_chain {
            *by.entry((&e.cause, e.occt.as_deref().unwrap_or("not asked")))
                .or_default() += 1;
        }
        for e in s.tangent_chain.iter().filter(|e| e.sampled) {
            let _ = writeln!(
                out,
                "- sampled {:?} at {:?}: {} → Open CASCADE {}",
                e.edge,
                e.at,
                e.cause,
                e.occt.as_deref().unwrap_or("not asked")
            );
        }
        for ((cause, verdict), n) in by {
            let _ = writeln!(
                out,
                "- {} {}: {n} × {cause} → Open CASCADE {verdict}",
                file.display(),
                s.solid
            );
        }
    }
    Ok(out)
}

/// The `TangentChain` edges of the solids, each alone, counted by
/// `part → site` ([`tangent_chain_cause`]); empty where there are none.
pub(super) fn tangent_chain_section(parts: &[(String, Vec<SolidCensus>)]) -> String {
    let mut out = String::new();
    for (part, solids) in parts {
        for s in solids {
            let mut by_cause: BTreeMap<&str, usize> = BTreeMap::new();
            for e in &s.tangent_chain {
                *by_cause.entry(&e.cause).or_default() += 1;
            }
            for (cause, n) in by_cause {
                let _ = writeln!(out, "- {part} {}: {n} × {cause}", s.solid);
            }
        }
    }
    if out.is_empty() {
        out
    } else {
        format!("\n`TangentChain` edges, each alone, by site:\n\n{out}")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Point3 as P3;

    /// The committed `nist-ftc-06`'s sampled edge, at (−86.40, 52.19,
    /// −234.95), is refused `TangentChain` at an end, and the census names
    /// what is there.
    #[test]
    fn ftc_06_sampled_edge_is_an_end_at_a_cusp() {
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
        let sampled = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id)
                    .is_some_and(|p| (p - P3::new(-86.40, 52.19, -234.95)).norm() < 0.01)
            })
            .expect("the sampled edge");
        let mut scratch = m.clone();
        let Err(Refused::Op(e)) = blend(&mut scratch, body, &[sampled], 0.254) else {
            panic!("the sampled edge is refused");
        };
        assert_eq!(
            tangent_chain_cause(&m, sampled.id, &e).unwrap(),
            "an end at a cusp, walls on either side; edge cylinder × plane"
        );
    }

    /// All 21 requests refused at overhang cusps before ADR-0079, retained
    /// by point so admission cannot silently shrink the census. The 13
    /// convex requests build checker-green (Open CASCADE: 2 build, 11 are
    /// invalid; the verdict carries no measures, so only validity is held,
    /// not volume). The 8 concave requests, which Open CASCADE builds with
    /// fitted caps, stay `TangentChain` and have an ignored corpus
    /// regression with the wanted result (ADR-0079, step 3 amendment).
    #[test]
    fn ftc_06_overhang_census_builds_convex_tips_and_records_concave_caps() {
        #[derive(Deserialize)]
        struct Tip {
            at: [f64; 3],
            convex: bool,
            occt: String,
        }
        let records =
            corpus_root().join("regression/nist-ftc-06-concave-overhang-fillet/census.json");
        let tips: Vec<Tip> = serde_json::from_slice(&std::fs::read(records).unwrap()).unwrap();
        assert_eq!(tips.len(), 21);
        assert_eq!(tips.iter().filter(|t| t.convex).count(), 13);
        let file = corpus_root().join("real/nist-ftc-06/nist_ftc_06_asme1_rd.stp");
        let mut m = Model::default();
        let read = step_read(
            &mut m,
            &std::fs::read_to_string(&file).unwrap(),
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
        let points: Vec<_> = tips.iter().map(|t| t.at).collect();
        let verdicts =
            crate::oracle::fillet_edges(&file, 101, 0.254, ORACLE_PROBE, &points).unwrap();
        assert_eq!(verdicts.len(), tips.len());
        let mut counts = BTreeMap::new();
        for (tip, verdict) in tips.iter().zip(verdicts) {
            assert_eq!(verdict.verdict, tip.occt, "{:?}", tip.at);
            let matches: Vec<_> = handles
                .iter()
                .filter(|h| {
                    edge_midpoint(&m, h.id)
                        .is_some_and(|p| (p - P3::from(tip.at)).norm() < ORACLE_PROBE)
                })
                .collect();
            let [edge] = matches[..] else {
                panic!("one edge at {:?}", tip.at)
            };
            let mut scratch = m.clone();
            let result = fillet(&mut scratch, body, &[*edge], 0.254);
            let class = match &result {
                Ok((built, _)) => {
                    let report = arris_check::check(&scratch, *built, arris_check::Level::Full);
                    assert!(report.is_ok(), "{:?}: {report}", tip.at);
                    BUILT.to_string()
                }
                Err(e) => class_of(e),
            };
            if tip.convex {
                assert!(result.is_ok(), "convex tip {:?}: {result:?}", tip.at);
            } else {
                assert!(
                    matches!(
                        result,
                        Err(OpError::Degenerate {
                            reason: Reason::Blend(BlendReason::TangentChain),
                            ..
                        })
                    ),
                    "concave tip {:?}: {result:?}",
                    tip.at
                );
            }
            eprintln!(
                "{:?}: convex {}, Arris {class}, OCCT {}",
                tip.at, tip.convex, verdict.verdict
            );
            *counts.entry((class, verdict.verdict)).or_insert(0) += 1;
        }
        eprintln!("21 overhang requests: {counts:?}");
        let tally = |class: &str, occt: &str| {
            counts
                .get(&(class.to_string(), occt.to_string()))
                .copied()
                .unwrap_or(0)
        };
        assert_eq!(tally(BUILT, "builds"), 2);
        assert_eq!(tally(BUILT, "invalid"), 11);
        assert_eq!(counts.values().sum::<usize>(), 21);
    }

    /// The committed `nist-ctc-04`'s sphere written as a NURBS circle against
    /// the cylinder of its radius, whose axis runs through its centre, at
    /// (179.47, 377.98, 16.95): the file's normals are `7.5e-7` apart, past
    /// the faces' `1e-7` and within the edge's own `1.5e-5`, so the edge
    /// alone is refused `TangentChain` as a tangent dihedral, not as an
    /// unsupported pair (ADR-0045).
    #[test]
    fn ctc_04_sphere_against_its_cylinder_is_a_tangent_dihedral() {
        let file = corpus_root().join("real/nist-ctc-04/nist_ctc_04_asme1_rd.stp");
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
            .find(|s| s.entity.id == 24)
            .and_then(|s| s.result.as_ref().ok())
            .unwrap()
            .body;
        let edge = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id)
                    .is_some_and(|p| (p - P3::new(179.4726, 377.9816, 16.9539)).norm() < 0.01)
            })
            .expect("the sphere's edge");
        let mut scratch = m.clone();
        let Err(Refused::Op(e)) = blend(&mut scratch, body, &[edge], 0.785) else {
            panic!("the edge is refused");
        };
        assert_eq!(class_of(&e), "Degenerate: TangentChain");
        assert_eq!(
            tangent_chain_cause(&m, edge.id, &e).unwrap(),
            "the edge itself, a tangent dihedral; edge cylinder × sphere"
        );
    }

    /// The edge of `recipe`'s body nearest `at` blended alone at `radius`,
    /// refused `TangentChain`, and the census's cause.
    fn cusp_cause(recipe: &str, at: P3, radius: f64) -> String {
        let recipe: crate::fixtures::Recipe = serde_json::from_str(recipe).unwrap();
        let chain = crate::corpus::build("generated/cusp", &recipe).unwrap();
        let body = chain.result().unwrap();
        // The part alone, as a read part is: no operand shares its edges.
        let mut m = chain.model;
        m.retain(&[body]).unwrap();
        let edge = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| edge_midpoint(&m, h.id).is_some_and(|p| (p - at).norm() < 1e-6))
            .expect("the blended edge");
        let mut scratch = m.clone();
        let Err(Refused::Op(e)) = blend(&mut scratch, body, &[edge], radius) else {
            panic!("the edge is refused");
        };
        tangent_chain_cause(&m, edge.id, &e).unwrap()
    }

    /// An overhang tip, the pillar's concave arc against the lip's, is a
    /// cusp with its walls on either side, still refused (ADR-0079).
    #[test]
    fn an_overhang_tip_is_a_cusp_with_walls_on_either_side() {
        let dir = corpus_root().join("blend/cusp-overhang-tip-pillar-fillet");
        let mut fixture: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(dir.join("fixture.json")).unwrap())
                .unwrap();
        // The part before its fillet.
        fixture["steps"].as_array_mut().unwrap().pop();
        fixture["result"] = "part".into();
        assert_eq!(
            cusp_cause(&fixture.to_string(), P3::new(0.0, -2.0, 1.0), 0.1),
            "an end at a cusp, walls on either side; edge cylinder × plane"
        );
    }

    /// A spandrel prism's spine, where the arc leaves the bottom edge
    /// tangent with material in the sliver between the walls, folds: its
    /// walls are on one side of the top (ADR-0042).
    #[test]
    fn a_spandrel_spine_folds() {
        let recipe = r#"{"steps": [
            {"name": "s", "op": "profile", "plane": {"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 1, 0]},
             "outer": {"start": [0, 0], "segments": [{"line_to": [10, 0]}, {"line_to": [10, 10]},
               {"arc_to": [0, 0], "via": [7.0710678118654755, 2.9289321881345245]}]}},
            {"name": "p", "op": "extrude", "profile": "s", "direction": [0, 0, 1], "length": 2}],
            "result": "p"}"#;
        let recipe: crate::fixtures::Recipe = serde_json::from_str(recipe).unwrap();
        let chain = crate::corpus::build("generated/spandrel", &recipe).unwrap();
        let body = chain.result().unwrap();
        let m = chain.model;
        let spine = m
            .edges(body)
            .unwrap()
            .into_iter()
            .find(|h| {
                edge_midpoint(&m, h.id).is_some_and(|p| (p - P3::new(0.0, 0.0, 1.0)).norm() < 1e-6)
            })
            .expect("the spine");
        let tip = m.edge(spine.id).unwrap().start();
        assert_eq!(walls_fold(&m, spine.id, tip), Some(true));
    }
}
