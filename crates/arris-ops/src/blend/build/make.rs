//! Make the stripes, their joins and the shared miter and corner entities.

use std::collections::{BTreeMap, BTreeSet};

use arris_topo::builder::{EdgeKey, EdgeSpec, VertexKey, VertexSpec};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{EdgeId, VertexId};

use super::super::corner::{Corner, corner};
use super::super::ends::{EndKind, face_end};
use super::super::junction::{Run, junction};
use super::super::miter::{Miter, miter};
use super::super::mutual::cusp_join;
use super::super::ring::Ring;
use super::super::rolled::rolled;
use super::super::stripe::{Section, Stripe, contacts, stripe};
use super::super::walked_blend::{
    walked_contacts, walked_cut, walked_end, walked_pole, walked_stripe,
};
use super::super::{Blend, degenerate, invariant};
use super::{BlendCtx, CornerMade, Draft, Env, Insertion, Joins, Meets, MiterMade, cut_once};
use crate::error::{BlendReason, OpError, Reason};
use crate::rebuild::forward;

/// One stripe per open edge.
pub(super) fn make_stripes(
    ctx: &mut BlendCtx<'_, '_>,
    open: &[EdgeId],
) -> Result<Vec<Stripe>, OpError> {
    let (env, meter) = ctx.split();
    let Env {
        m, view, kind, tol, ..
    } = env;
    let edges = open;
    let mut stripes: Vec<Stripe> = Vec::with_capacity(edges.len());
    for &e in edges {
        meter.tick()?;
        stripes.push(match stripe(m, view, e, kind, tol) {
            // The closed forms hold neither faces nor curve: a fillet is
            // walked (ADR-0062).
            Err(OpError::Unsupported { .. }) => walked_stripe(&env, meter, e)?,
            other => other?,
        });
    }
    Ok(stripes)
}

/// The miters, the junctions and the corners, in vertex order.
pub(super) fn join_blends(
    ctx: &mut BlendCtx<'_, '_>,
    open: &[EdgeId],
    stripes: &[Stripe],
    rings: &[Ring],
    meets: &Meets,
    kinks: &BTreeSet<VertexId>,
) -> Result<Joins, OpError> {
    let Meets {
        at_vertex,
        junctions,
        cusps,
    } = meets;
    let (env, meter) = ctx.split();
    let Env { m, view, tol, .. } = env;
    let edges = open;
    let index_of: BTreeMap<EdgeId, usize> =
        edges.iter().enumerate().map(|(i, &e)| (e, i)).collect();
    let ring_of: BTreeMap<EdgeId, usize> =
        rings.iter().enumerate().map(|(i, r)| (r.edge, i)).collect();
    let run = |e: EdgeId| match (index_of.get(&e), ring_of.get(&e)) {
        (Some(&i), _) => Ok(Run::Line(&stripes[i])),
        (None, Some(&i)) => Ok(Run::Arc(&rings[i])),
        (None, None) => Err(invariant("a blended edge's stripe or ring")),
    };
    // The miters, the junctions and the corners, in vertex order.
    let mut miters: Vec<Miter> = Vec::new();
    let mut miter_at: BTreeMap<VertexId, usize> = BTreeMap::new();
    let mut corners: Vec<Corner> = Vec::new();
    let mut corner_at: BTreeMap<VertexId, usize> = BTreeMap::new();
    for (&v, es) in at_vertex {
        // A walked stripe meets another blend only at a tangent vertex of
        // its chain, on the ball's arc there; a miter or a corner patch
        // has no closed form against it.
        if es.len() > 1
            && !junctions.contains(&v)
            && es.iter().any(|e| {
                index_of
                    .get(e)
                    .is_some_and(|&i| matches!(stripes[i].section, Section::Walked { .. }))
            })
        {
            let entities = es.iter().map(|&e| forward(e)).chain([forward(v)]).collect();
            return Err(degenerate(
                entities,
                Reason::Blend(BlendReason::VertexBlend),
            ));
        }
        match es[..] {
            [ea, eb] if junctions.contains(&v) => {
                miter_at.insert(v, miters.len());
                // Across a kink the runs meet within the vertex's tolerance
                // (ADR-0075 §2).
                let kink = if kinks.contains(&v) {
                    Some(m.vertex(v)?.tolerance())
                } else {
                    None
                };
                miters.push(junction(
                    m,
                    view,
                    [run(ea)?, run(eb)?],
                    v,
                    kink,
                    tol,
                    meter,
                )?);
            }
            [ea, eb] => {
                miter_at.insert(v, miters.len());
                miters.push(miter(
                    &env,
                    meter,
                    &stripes[index_of[&ea]],
                    &stripes[index_of[&eb]],
                    v,
                )?);
            }
            [ea, eb, ec] => {
                corner_at.insert(v, corners.len());
                corners.push(corner(
                    &env,
                    meter,
                    [
                        &stripes[index_of[&ea]],
                        &stripes[index_of[&eb]],
                        &stripes[index_of[&ec]],
                    ],
                    v,
                )?);
            }
            _ => {}
        }
    }
    // The cusps' mutual trims, recorded as miters (ADR-0084 §4′).
    let mut cusp_miters: BTreeSet<usize> = BTreeSet::new();
    for join in cusps {
        let [ra, rb] = join.edges.map(run);
        miter_at.insert(join.vertex, miters.len());
        cusp_miters.insert(miters.len());
        miters.push(cusp_join(&env, meter, [ra?, rb?], join)?);
    }
    Ok(Joins {
        miters,
        miter_at,
        cusps: cusp_miters,
        corners,
        corner_at,
    })
}

/// Each stripe's ends, then its contacts between them.
pub(super) fn make_blends(
    ctx: &mut BlendCtx<'_, '_>,
    stripes: Vec<Stripe>,
    joins: &Joins,
) -> Result<Vec<Blend>, OpError> {
    let (env, meter) = ctx.split();
    let Joins {
        miters,
        miter_at,
        corners,
        corner_at,
        ..
    } = joins;
    // Each stripe's ends, then its contacts between them.
    let mut blends: Vec<Blend> = Vec::with_capacity(stripes.len());
    for mut s in stripes {
        let mut ends: Vec<EndKind> = Vec::with_capacity(2);
        for (at_lo, vertex) in [(true, s.start), (false, s.end)] {
            // A walked stripe narrowed to its vertex meets no other blend
            // there: the vertex is kept, with nothing to share.
            if matches!(s.section, Section::Walked { poles, .. } if poles[usize::from(!at_lo)]) {
                if miter_at.contains_key(&vertex) || corner_at.contains_key(&vertex) {
                    return Err(degenerate(
                        vec![forward(s.edge), forward(vertex)],
                        Reason::Blend(BlendReason::VertexBlend),
                    ));
                }
                ends.push(EndKind::Pole(walked_pole(&s, at_lo)?));
                continue;
            }
            ends.push(match (miter_at.get(&vertex), corner_at.get(&vertex)) {
                (Some(&at), _) => EndKind::Miter {
                    at,
                    side: usize::from(miters[at].edges[0] != s.edge),
                },
                (None, Some(&at)) => EndKind::Corner {
                    at,
                    side: corners[at]
                        .edges
                        .iter()
                        .position(|&e| e == s.edge)
                        .ok_or(invariant("a corner's own blend"))?,
                },
                (None, None) => EndKind::Face(Box::new(match s.section {
                    Section::Walked { .. } => walked_end(&env, meter, &s, at_lo)?,
                    Section::Round { .. } | Section::Flat => face_end(&env, meter, &s, at_lo)?,
                })),
            });
        }
        let ends: [EndKind; 2] = ends
            .try_into()
            .map_err(|_| invariant("two ends of the blend"))?;
        let t = [0, 1].map(|end| [0, 1].map(|k| ends[end].t(miters, corners, k)));
        if matches!(s.section, Section::Walked { .. }) {
            let cut = [0, 1].map(|end| match ends[end] {
                EndKind::Miter { at, .. } if miters[at].leaning => Some(t[end][0]),
                _ => None,
            });
            walked_cut(&env, meter, &mut s, cut)?;
        }
        let (contacts, rolled) = match s.section {
            Section::Walked { .. } => (walked_contacts(&env, meter, &s, t)?, None),
            Section::Round { .. } | Section::Flat => {
                let (contacts, run) = contacts(&env, &s, t, meter)?;
                let rolled = match run {
                    Some((k, run)) => Some(Box::new(rolled(&env, meter, &s, &contacts, k, run)?)),
                    None => None,
                };
                (contacts, rolled)
            }
        };
        blends.push(Blend {
            stripe: s,
            contacts,
            ends,
            rolled,
        });
    }

    Ok(blends)
}

/// The miters first: their two vertices, their edge, the third edge cut.
pub(super) fn add_miters(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    miters: &[Miter],
) -> Result<Vec<MiterMade>, OpError> {
    let m = &mut *ctx.m;
    let Draft {
        rw, cuts, edits, ..
    } = draft;
    let mut miter_made: Vec<MiterMade> = Vec::with_capacity(miters.len());
    for mt in miters {
        let q = rw.vertex(VertexSpec::New {
            point: mt.q,
            tolerance: mt.q_tolerance,
        });
        let p3 = rw.vertex(VertexSpec::New {
            point: mt.p3,
            tolerance: mt.p3_tolerance,
        });
        let (first, second) = if mt.q_first { (q, p3) } else { (p3, q) };
        let edge = rw.edge(
            EdgeSpec::New {
                geometry: EdgeGeometry::Curve {
                    curve: m.add_curve(mt.curve.clone()),
                    range: mt.range,
                },
                start: VertexKey::New(first),
                end: VertexKey::New(second),
                tolerance: mt.tolerance,
            },
            None,
        );
        // A miter of unequal dihedrals: the trim arc from `m` on to the
        // third edge, which the narrower blend's far face takes at the
        // corner vertex (ADR-0044 §4).
        let trim_arc = match &mt.trim_arc {
            None => None,
            Some(arc) => {
                let end = rw.vertex(VertexSpec::New {
                    point: arc.end,
                    tolerance: arc.end_tolerance,
                });
                let (first, second) = if arc.m_first { (p3, end) } else { (end, p3) };
                let edge = rw.edge(
                    EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(arc.curve.clone()),
                            range: arc.range,
                        },
                        start: VertexKey::New(first),
                        end: VertexKey::New(second),
                        tolerance: arc.tolerance,
                    },
                    None,
                );
                let pcurve = m.add_curve2(arc.on_face.clone());
                edits.entry(arc.face).or_default().insert.insert(
                    arc.vertex,
                    Insertion {
                        arc: EdgeKey::New(edge),
                        pcurve,
                        lo_first: arc.m_first,
                        edge_at_lo: arc.narrow_edge,
                        splits: None,
                        tail: None,
                    },
                );
                Some((end, edge))
            }
        };
        if let Some(trim) = &mt.trim {
            cut_once(cuts, trim, trim_arc.map_or(p3, |(end, _)| end))?;
        }
        if let Some(trim) = &mt.q_trim {
            cut_once(cuts, trim, q)?;
        }
        miter_made.push(MiterMade {
            vertices: [q, p3],
            edge,
            trim_arc,
        });
    }
    Ok(miter_made)
}

/// The corners next: their three vertices, their three arcs and a sphere's pole; no corner edge is cut.
pub(super) fn add_corners(
    ctx: &mut BlendCtx<'_, '_>,
    draft: &mut Draft,
    corners: &[Corner],
) -> Vec<CornerMade> {
    let m = &mut *ctx.m;
    let rw = &mut draft.rw;
    let mut corner_made: Vec<CornerMade> = Vec::with_capacity(corners.len());
    for c in corners {
        let mut vertices = [0usize; 3];
        for (slot, &point) in vertices.iter_mut().zip(&c.points) {
            *slot = rw.vertex(VertexSpec::New {
                point,
                tolerance: c.tolerance,
            });
        }
        let mut arcs = [0usize; 3];
        for (slot, arc) in arcs.iter_mut().zip(&c.arcs) {
            *slot = rw.edge(
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(arc.curve.clone()),
                        range: arc.range,
                    },
                    start: VertexKey::New(vertices[arc.ends[0]]),
                    end: VertexKey::New(vertices[arc.ends[1]]),
                    tolerance: c.tolerance,
                },
                None,
            );
        }
        let pole = c.pole.as_ref().map(|pole| {
            let at = VertexKey::New(vertices[pole.point]);
            rw.edge(
                EdgeSpec::New {
                    geometry: EdgeGeometry::Degenerate { range: pole.range },
                    start: at,
                    end: at,
                    tolerance: c.tolerance,
                },
                None,
            )
        });
        corner_made.push(CornerMade {
            vertices,
            arcs,
            pole,
            added: 0,
        });
    }
    corner_made
}
