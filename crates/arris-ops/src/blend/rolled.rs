//! A run-over's rolled face and the entities it adds (ADR-0071 §1, §2).
//! Between the two crossings the ball rolls on `E`. The face it sweeps
//! there is skinned through its arcs along the spine, from its foot on
//! `H` to its foot on `E`, as a walked stripe is (ADR-0062). The
//! closed-form stripe ends at the section arc through each crossing, which
//! the two faces share, so it becomes two faces. `E` is cut at the
//! crossings into three pieces. `F`'s loop runs the contact to each
//! crossing and then along `E`'s outer pieces, the loop that held `E`
//! joined into it.

use std::collections::BTreeMap;

use arris_geom::region2::Side;
use arris_geom::{Curve, Curve2, NurbsCurve, SECTION_FIT_FRACTION, Surface, fit_skin, pcurve_on};
use arris_math::{Frame, Interval, Meter, Point2, Tolerance};
use arris_topo::builder::{EdgeKey, EdgeSpec, VertexKey, VertexSpec};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{CurveId, EdgeId, FaceId, Model, Orientation};

use super::build::{Env, FaceEdit, SplitPiece};
use super::run_over::{Crossing, RunOver, crossed};
use super::stripe::{Contact, Section, Stripe, on_side_of_face, placed};
use super::walked::{Ball, Refusal, SURFACE_DEGREE, Walked, arc, contacts as contact_rows};
use super::{Blend, invariant};
use crate::body_view::BodyView;
use crate::error::{OpError, fault_of};
use crate::rebuild::{AddedFace, Rewrite, StoredUse, along};

/// How far past each crossing the rolled face's surface runs, as a share
/// of the stretch between them: the arc it shares with the stripe at each
/// crossing is then a row inside its domain, where the tracer decides the
/// tangent isoline as it does a walked stripe's carried past its ends
/// (ADR-0061, ADR-0062). A reach, not a tolerance.
const REACH_SHARE: f64 = 0.25;

/// The share of the section fit's budget the rolled face is skinned to: a
/// quarter, so that its slope across the arcs it shares with the stripe is
/// under what the tracer reads as tangent (`1.0e-6` against `4e-7` at the
/// whole budget, on the lip filleted over a hole). A ratio between two
/// fits, not a tolerance.
const ROLLED_FIT_SHARE: f64 = 0.25;

/// The sections past a crossing [`reach_past`] looks at before it takes a
/// reach, and the most times it halves one. A sampling density and a
/// structural bound, not tolerances.
const REACH_SAMPLES: usize = 8;
const REACH_HALVINGS: usize = 16;

/// How far past the crossing at `t` the rolled face's surface runs, the
/// way `side` says: [`REACH_SHARE`] of the stretch between the crossings,
/// halved until every one of [`REACH_SAMPLES`] sections out to it has its
/// circle — `E` crosses only so many of the stripe's planes. `None` where
/// no reach is left after [`REACH_HALVINGS`].
fn reach_past(run: &RunOver, t: f64, side: f64) -> Option<f64> {
    let mut reach = REACH_SHARE * run.off.length();
    for _ in 0..REACH_HALVINGS {
        let held = (1..=REACH_SAMPLES).all(|k| {
            run.section(t + side * reach * k as f64 / REACH_SAMPLES as f64)
                .is_some()
        });
        if held {
            return Some(reach);
        }
        reach *= 0.5;
    }
    None
}

/// The stripe's section arc at a crossing: the ball's arc there, from the
/// stripe's contact at `u = 0` to its contact at `u = β`, which the stripe
/// and the rolled face share.
pub(super) struct SectionArc {
    pub(super) curve: Curve,
    pub(super) range: Interval,
    /// Its pcurve on the stripe's surface, the isocurve at the crossing.
    pub(super) on_stripe: Curve2,
    /// Its pcurve on the rolled face, near the surface's end row there.
    pub(super) on_rolled: Curve2,
}

/// A piece of `E` cut at the crossings, in `E`'s own order.
pub(super) struct EdgePiece {
    pub(super) range: Interval,
    /// The crossing at each end, `None` where the piece ends at `E`'s own
    /// vertex.
    pub(super) ends: [Option<usize>; 2],
    /// Its pcurve on the rolled face where the ball rolls on it; `None`
    /// for an outer piece, which stays on `F`.
    pub(super) on_rolled: Option<Curve2>,
}

/// A stripe's run-over, built: the rolled face and what the stripe, `F`,
/// `G` and `E` are cut into around it.
pub(super) struct Rolled {
    pub(super) run: RunOver,
    /// The blended edge, the blend's faces and its contacts' curves, ranges
    /// and pcurves on them, by contact, as the blend placed them.
    pub(super) blended: EdgeId,
    pub(super) faces: [FaceId; 2],
    pub(super) contacts: [Curve; 2],
    pub(super) ranges: [Interval; 2],
    pub(super) on_face: [Curve2; 2],
    /// The stripe's contact on `F`, the one that leaves it.
    pub(super) on_f: usize,
    /// The rolled face's surface: `u` from its foot on `H` (`0`) to its
    /// foot on `E` (`1`), `v` from the first crossing to the second.
    pub(super) surface: Surface,
    pub(super) orientation: Orientation,
    /// The section arcs, by crossing.
    pub(super) arcs: [SectionArc; 2],
    /// The rolled face's foot on `H`, from the first crossing to the
    /// second: its isocurve `u = 0`, its parameter the surface's `v`.
    pub(super) foot: Curve,
    pub(super) foot_range: Interval,
    pub(super) foot_on_face: Curve2,
    /// `E`'s three pieces.
    pub(super) pieces: [EdgePiece; 3],
    /// The pieces the ball rolls on, from the first crossing to the
    /// second, each `true` where that runs along `E`.
    pub(super) rolled_on: Vec<(usize, bool)>,
    pub(super) tolerance: f64,
}

/// What a run-over reads of the blend it belongs to, a stripe's or a
/// ring's: the blended edge, its faces and contacts by contact (the stripe's
/// `u = 0` then `u = β`, the ring's lower `v` then upper), and the loop of
/// each face the contact lies on.
pub(super) struct Host<'a> {
    pub(super) edge: EdgeId,
    pub(super) tolerance: f64,
    pub(super) convex: bool,
    pub(super) faces: [FaceId; 2],
    pub(super) curves: [&'a Curve; 2],
    pub(super) ranges: [Interval; 2],
    pub(super) on_face: [&'a Curve2; 2],
    pub(super) loops: [usize; 2],
}

/// The section arc at a crossing, as the blend that is cut there has it.
pub(super) type ArcAt<'a> = &'a dyn Fn(&Crossing, &mut Meter<'_>) -> Result<SectionArc, OpError>;

/// The rolled face of the run-over `run` of the stripe `s`, its contact
/// `on_f` on `F` (ADR-0071 §1, §2).
///
/// Errors: as [`rolled_of`].
pub(super) fn rolled(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    s: &Stripe,
    contacts: &[Contact; 2],
    on_f: usize,
    run: RunOver,
) -> Result<Rolled, OpError> {
    let Section::Round { radius, .. } = s.section else {
        return Err(invariant("a fillet's closed-form stripe in a run-over"));
    };
    let tol = Tolerance::new(s.tolerance, env.tol.angular);
    let host = Host {
        edge: s.edge,
        tolerance: s.tolerance,
        convex: s.convex,
        faces: [contacts[0].face, contacts[1].face],
        curves: [&contacts[0].line, &contacts[1].line],
        ranges: [contacts[0].range, contacts[1].range],
        on_face: [&contacts[0].on_face, &contacts[1].on_face],
        loops: [s.uses[0].loop_index, s.uses[1].loop_index],
    };
    let arc_at = |c: &Crossing, meter: &mut Meter<'_>| -> Result<SectionArc, OpError> {
        let start = contacts[0].line.point(c.t);
        let frame = Frame::new(c.centre, s.d, start - c.centre)?;
        let curve = Curve::Circle { frame, radius };
        let range = Interval::new(0.0, s.beta).map_err(|_| invariant("a section arc"))?;
        let on_stripe = pcurve_on(&curve, range, &s.surface, tol, meter).map_err(fault_of)?;
        Ok(SectionArc {
            on_stripe: s.place(on_stripe, range.lo(), 0.0),
            on_rolled: Curve2::along_u(0.0),
            curve,
            range,
        })
    };
    rolled_of(env, meter, &host, &arc_at, on_f, run)
}

/// The rolled face of the run-over `run` of the blend `host`, its contact
/// `on_f` on `F` (ADR-0071 §1, §2).
///
/// Errors: `BlendTooLarge` naming the blended edge and `E` where `E` is
/// not between `F` and another face, lies on the loop of `F` that holds the
/// blended edge, would be cut into a piece no longer than the tolerance,
/// or where the rolled face's foot on `H` leaves `H` or a ball's arc is not
/// decided (ADR-0071 §3); a fault of a fit or a pcurve as itself.
pub(super) fn rolled_of(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    s: &Host<'_>,
    arc_at: ArcAt<'_>,
    on_f: usize,
    run: RunOver,
) -> Result<Rolled, OpError> {
    let m = env.m;
    let refuse = || crossed(s.edge, run.edge);
    let tol = Tolerance::new(s.tolerance, env.tol.angular);
    let on_h = 1 - on_f;
    let (face, h_face) = (s.faces[on_f], s.faces[on_h]);
    let edge = *m.edge(run.edge)?;
    let (curve_id, e_range) = edge.curve().ok_or_else(refuse)?;
    let e_curve = m.curve(curve_id)?;
    // `E` between `F` and `G`, on a loop of `F` other than the contact's:
    // that loop is joined into the contact's (§2).
    let uses = env.view.uses.get(&run.edge).ok_or_else(refuse)?;
    let on_face = match uses[..] {
        [a, b] if a.face == face && b.face != face => a,
        [a, b] if b.face == face && a.face != face => b,
        _ => return Err(refuse()),
    };
    if on_face.loop_index == s.loops[on_f] {
        return Err(refuse());
    }
    let h_surface = m.surface(m.face(h_face)?.surface())?;
    let mut arcs: Vec<SectionArc> = Vec::with_capacity(2);
    for c in &run.crossings {
        arcs.push(arc_at(c, meter)?);
    }
    // The skin's `v` is the contact's parameter, each row the section there
    // (ADR-0071, amended), from the first crossing to the second and on past each as
    // far as `reach_past` finds sections: the arcs it shares with the stripe
    // are rows inside its domain.
    let range = run.off;
    let skinned = Interval::new(
        range.lo() - reach_past(&run, range.lo(), -1.0).ok_or_else(refuse)?,
        range.hi() + reach_past(&run, range.hi(), 1.0).ok_or_else(refuse)?,
    )
    .map_err(|_| invariant("a run-over's stretch of positive length"))?;
    let unsettled = || {
        Walked::Refused(Refusal::NoSpine {
            why: "a run-over's section has no circle",
        })
    };
    let section = |v: f64| -> Result<NurbsCurve, Walked> {
        let at = run.section(v).ok_or_else(unsettled)?;
        arc(&Ball {
            centre: at.centre,
            contacts: [at.foot, at.point],
            uv: [Point2::origin(); 2],
        })
    };
    let named = |w: Walked| match w {
        Walked::Refused(_) => refuse(),
        Walked::Op(e) => e,
    };
    let budget = ROLLED_FIT_SHARE * SECTION_FIT_FRACTION * tol.linear;
    let skin = fit_skin(section, skinned, SURFACE_DEGREE, budget, meter).map_err(named)?;
    // `Forward` where the surface's normal is the outward one: away from
    // the ball's centre on a convex blend, toward it on a concave one, as
    // the stripe's cylinder is read.
    let v_mid = range.midpoint();
    let mid = skin.eval(0.5, v_mid).point;
    let normal = skin
        .normal(0.5, v_mid)
        .ok_or(invariant("a regular rolled face"))?;
    let centre = run.section(v_mid).ok_or_else(refuse)?.centre;
    let away = normal.dot(&(mid - centre)) > 0.0;
    let orientation = along(away == s.convex);
    let [foot, _] = contact_rows(&skin).map_err(named)?;
    let foot = Curve::Nurbs(foot);
    let surface = Surface::Nurbs(skin);
    // The foot on `H`, in the translate of `H`'s loop, inside `H` (§3).
    let foot_on_face = pcurve_on(&foot, range, h_surface, tol, meter).map_err(fault_of)?;
    let target = s.on_face[on_h].point(run.crossings[0].t).x;
    let foot_on_face = placed(foot_on_face, range.lo(), target, h_surface.period()[0]);
    if !on_side_of_face(m, h_face, &foot_on_face, range, Side::Inside, env.samples)? {
        return Err(refuse());
    }
    for arc in &mut arcs {
        arc.on_rolled = pcurve_on(&arc.curve, arc.range, &surface, tol, meter).map_err(fault_of)?;
    }
    // `E` cut at the crossings, its pieces in its own order; the ball rolls
    // on the middle one, or on a closed edge's two either side of its
    // vertex where the stretch it rolls on runs through it.
    let mut by_s = [(run.crossings[0].s, 0), (run.crossings[1].s, 1)];
    by_s.sort_by(|a, b| a.0.total_cmp(&b.0));
    let cuts = [e_range.lo(), by_s[0].0, by_s[1].0, e_range.hi()];
    let ends = [None, Some(by_s[0].1), Some(by_s[1].1), None];
    let wraps = (run.along.lo() - by_s[1].0).abs() < (run.along.lo() - by_s[0].0).abs();
    let e_tol = Tolerance::new(s.tolerance.max(edge.tolerance()), env.tol.angular);
    let mut pieces: Vec<EdgePiece> = Vec::with_capacity(3);
    for i in 0..3 {
        let range = Interval::new(cuts[i], cuts[i + 1]).map_err(|_| refuse())?;
        if (e_curve.point(range.lo()) - e_curve.point(range.hi())).norm() <= e_tol.linear {
            return Err(refuse());
        }
        let middle = if wraps { i != 1 } else { i == 1 };
        let on_rolled = if middle {
            Some(pcurve_on(e_curve, range, &surface, e_tol, meter).map_err(fault_of)?)
        } else {
            None
        };
        pieces.push(EdgePiece {
            range,
            ends: [ends[i], ends[i + 1]],
            on_rolled,
        });
    }
    let pieces: [EdgePiece; 3] = pieces
        .try_into()
        .map_err(|_| invariant("three pieces of the crossed edge"))?;
    // The pieces the ball rolls on in `E`'s order, from the crossing at the
    // stretch's lower end; reversed where that is the second crossing.
    let (order, from) = if wraps {
        (vec![2, 0], by_s[1].1)
    } else {
        (vec![1], by_s[0].1)
    };
    let rolled_on = if from == 0 {
        order.into_iter().map(|i| (i, true)).collect()
    } else {
        order.into_iter().rev().map(|i| (i, false)).collect()
    };
    let arcs: [SectionArc; 2] = arcs
        .try_into()
        .map_err(|_| invariant("a section arc at each crossing"))?;
    Ok(Rolled {
        run,
        blended: s.edge,
        faces: s.faces,
        contacts: s.curves.map(Curve::clone),
        ranges: s.ranges,
        on_face: s.on_face.map(Curve2::clone),
        on_f,
        surface,
        orientation,
        arcs,
        foot,
        foot_range: range,
        foot_on_face,
        pieces,
        rolled_on,
        tolerance: s.tolerance,
    })
}

/// The rewrite's indices of a run-over's entities.
#[derive(Debug, Clone, Copy)]
pub(super) struct RolledMade {
    /// The crossing vertices `[crossing][contact]`: on `E` for the contact
    /// on `F`, on `H` for the other.
    pub(super) vertices: [[usize; 2]; 2],
    /// The stripe's contacts cut at the crossings, `[contact][piece]`:
    /// before the first crossing, then after the second.
    pub(super) contacts: [[usize; 2]; 2],
    /// The rolled face's foot on `H`.
    pub(super) foot: usize,
    /// The section arcs, by crossing.
    pub(super) arcs: [usize; 2],
    /// `E`'s three pieces.
    pub(super) pieces: [usize; 3],
    /// The stripe's face after the second crossing and the rolled face,
    /// in the rewrite's added faces; the stripe's face before the first is
    /// the blend's own.
    pub(super) added: [usize; 2],
}

/// The run-over's vertices and edges in `rw`, and the edits of `F`, `H`
/// and `G` (ADR-0071 §2): `F`'s contact cut at the crossings with a gap
/// between for `E`'s outer pieces, `H`'s in three with the rolled face's
/// foot in the middle, and `E` cut into its pieces on `F` (without the
/// middle) and on `G`. `ends` are the stripe's trim vertices
/// `[end][contact]`.
pub(super) fn add_run_over(
    m: &mut Model,
    view: &BodyView,
    rw: &mut Rewrite,
    edits: &mut BTreeMap<FaceId, FaceEdit>,
    rolled: &Rolled,
    ends: [[usize; 2]; 2],
    crowded: bool,
) -> Result<RolledMade, OpError> {
    let run = &rolled.run;
    let (on_f, on_h) = (rolled.on_f, 1 - rolled.on_f);
    let edge = *m.edge(run.edge)?;
    let (e_curve, _) = edge.curve().ok_or(invariant("the crossed edge's curve"))?;
    let mut vertices = [[0usize; 2]; 2];
    for (i, c) in run.crossings.iter().enumerate() {
        vertices[i][on_f] = rw.vertex(VertexSpec::New {
            point: c.point,
            tolerance: rolled.tolerance.max(edge.tolerance()),
        });
        vertices[i][on_h] = rw.vertex(VertexSpec::New {
            point: rolled.contacts[on_h].point(c.t),
            tolerance: rolled.tolerance,
        });
    }
    let [t0, t1] = run.crossings.map(|c| c.t);
    let mut contacts = [[0usize; 2]; 2];
    for j in 0..2 {
        let (line, range) = (&rolled.contacts[j], rolled.ranges[j]);
        let before = Interval::new(range.lo(), t0)
            .map_err(|_| invariant("a contact before its run-over"))?;
        let after =
            Interval::new(t1, range.hi()).map_err(|_| invariant("a contact after its run-over"))?;
        let new = |k: usize| VertexKey::New(k);
        contacts[j] = [
            push_edge(
                rw,
                m.add_curve(line.clone()),
                before,
                new(ends[0][j]),
                new(vertices[0][j]),
                rolled.tolerance,
                None,
            ),
            push_edge(
                rw,
                m.add_curve(line.clone()),
                after,
                new(vertices[1][j]),
                new(ends[1][j]),
                rolled.tolerance,
                None,
            ),
        ];
    }
    let foot = push_edge(
        rw,
        m.add_curve(rolled.foot.clone()),
        rolled.foot_range,
        VertexKey::New(vertices[0][on_h]),
        VertexKey::New(vertices[1][on_h]),
        rolled.tolerance,
        None,
    );
    let mut arcs = [0usize; 2];
    for (i, arc) in rolled.arcs.iter().enumerate() {
        arcs[i] = push_edge(
            rw,
            m.add_curve(arc.curve.clone()),
            arc.range,
            VertexKey::New(vertices[i][0]),
            VertexKey::New(vertices[i][1]),
            rolled.tolerance,
            None,
        );
    }
    let mut pieces = [0usize; 3];
    if !crowded {
        for (i, piece) in rolled.pieces.iter().enumerate() {
            let at = |end: Option<usize>, kept| match end {
                Some(c) => VertexKey::New(vertices[c][on_f]),
                None => VertexKey::Kept(kept),
            };
            pieces[i] = push_edge(
                rw,
                e_curve,
                piece.range,
                at(piece.ends[0], edge.start()),
                at(piece.ends[1], edge.end()),
                edge.tolerance(),
                Some(run.edge),
            );
        }
    }
    // `F`: the contact up to the first crossing, a gap where `E`'s outer
    // pieces join it, the contact on from the second; `H`: the contact
    // with the rolled face's foot between.
    let f = rolled.faces[on_f];
    let h = rolled.faces[on_h];
    let mut piece_of = |edge: usize, pcurve: &Curve2| {
        SplitPiece::Use(EdgeKey::New(edge), m.add_curve2(pcurve.clone()))
    };
    let on_face = &rolled.on_face[on_f];
    let f_contact = vec![
        piece_of(contacts[on_f][0], on_face),
        SplitPiece::Gap(rolled.blended),
        piece_of(contacts[on_f][1], on_face),
    ];
    let on_face = &rolled.on_face[on_h];
    let h_contact = vec![
        piece_of(contacts[on_h][0], on_face),
        piece_of(foot, &rolled.foot_on_face),
        piece_of(contacts[on_h][1], on_face),
    ];
    let mut e_on = BTreeMap::new();
    for u in view.uses.get(&run.edge).into_iter().flatten() {
        if crowded {
            break;
        }
        let keep_middle = u.face != f;
        let mut list = Vec::with_capacity(3);
        for (piece, &id) in rolled.pieces.iter().zip(&pieces) {
            list.push(if keep_middle || piece.on_rolled.is_none() {
                SplitPiece::Use(EdgeKey::New(id), u.pcurve)
            } else {
                SplitPiece::Gap(rolled.blended)
            });
        }
        e_on.insert(u.face, list);
    }
    edits
        .entry(f)
        .or_default()
        .split
        .insert(rolled.blended, f_contact);
    edits
        .entry(h)
        .or_default()
        .split
        .insert(rolled.blended, h_contact);
    for (face, list) in e_on {
        edits.entry(face).or_default().split.insert(run.edge, list);
    }
    Ok(RolledMade {
        vertices,
        contacts,
        foot,
        arcs,
        pieces,
        added: [0; 2],
    })
}

/// A crowded edge `E`, which each of `runs` crosses twice, cut once at all
/// the crossings (ADR-0071 §2): its pieces in its own order, the ones
/// between a run-over's crossings the ones its ball rolls on, which `F`
/// leaves a gap at and `G` keeps. Each run-over's `pieces` are the one
/// before its stretch, the stretch and the one after.
///
/// Errors: `BlendTooLarge` naming a blended edge and `E` where the
/// run-overs do not all leave the same face, one rolls through `E`'s own
/// vertex, the stretches overlap, or a piece would be no longer than the
/// tolerance.
pub(super) fn add_crowded_edge(
    m: &mut Model,
    view: &BodyView,
    rw: &mut Rewrite,
    edits: &mut BTreeMap<FaceId, FaceEdit>,
    runs: &mut [(&Rolled, &mut RolledMade)],
) -> Result<(), OpError> {
    let Some((first, _)) = runs.first() else {
        return Ok(());
    };
    let (e, f) = (first.run.edge, first.faces[first.on_f]);
    let refuse = |rolled: &Rolled| crossed(rolled.blended, e);
    let edge = *m.edge(e)?;
    let (curve, e_range) = edge.curve().ok_or(invariant("the crossed edge's curve"))?;
    let e_curve = m.curve(curve)?.clone();
    let mut order: Vec<usize> = (0..runs.len()).collect();
    for (rolled, _) in runs.iter() {
        if rolled.faces[rolled.on_f] != f
            || rolled.rolled_on.len() != 1
            || rolled.rolled_on[0].0 != 1
        {
            return Err(refuse(rolled));
        }
    }
    order.sort_by(|&a, &b| {
        let lo = |j: usize| runs[j].0.pieces[1].range.lo();
        lo(a).total_cmp(&lo(b))
    });
    // The cuts from `E`'s start to its end: the parameter and the vertex.
    let mut cuts: Vec<(f64, VertexKey)> = vec![(e_range.lo(), VertexKey::Kept(edge.start()))];
    for &j in &order {
        let (rolled, made) = &runs[j];
        let stretch = &rolled.pieces[1];
        let (Some(c0), Some(c1)) = (stretch.ends[0], stretch.ends[1]) else {
            return Err(refuse(rolled));
        };
        let at = |c: usize| VertexKey::New(made.vertices[c][rolled.on_f]);
        cuts.push((stretch.range.lo(), at(c0)));
        cuts.push((stretch.range.hi(), at(c1)));
    }
    cuts.push((e_range.hi(), VertexKey::Kept(edge.end())));
    let mut ids: Vec<usize> = Vec::with_capacity(cuts.len() - 1);
    for w in cuts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let range = Interval::new(a.0, b.0).map_err(|_| refuse(runs[order[0]].0))?;
        let tol = runs[order[0]].0.tolerance.max(edge.tolerance());
        if (e_curve.point(a.0) - e_curve.point(b.0)).norm() <= tol {
            return Err(refuse(runs[order[0]].0));
        }
        ids.push(push_edge(
            rw,
            m.add_curve(e_curve.clone()),
            range,
            a.1,
            b.1,
            edge.tolerance(),
            Some(e),
        ));
    }
    for (p, &j) in order.iter().enumerate() {
        runs[j].1.pieces = [ids[2 * p], ids[2 * p + 1], ids[2 * p + 2]];
    }
    for u in view.uses.get(&e).into_iter().flatten() {
        let mut list = Vec::with_capacity(ids.len());
        for (k, &id) in ids.iter().enumerate() {
            list.push(if k % 2 == 1 && u.face == f {
                SplitPiece::Gap(runs[order[k / 2]].0.blended)
            } else {
                SplitPiece::Use(EdgeKey::New(id), u.pcurve)
            });
        }
        edits.entry(u.face).or_default().split.insert(e, list);
    }
    Ok(())
}

/// A new edge over `curve` and `range` from `start` to `end` in `rw`, a
/// piece of `parent` where it is one: its index.
#[allow(clippy::too_many_arguments)]
fn push_edge(
    rw: &mut Rewrite,
    curve: CurveId,
    range: Interval,
    start: VertexKey,
    end: VertexKey,
    tolerance: f64,
    parent: Option<EdgeId>,
) -> usize {
    rw.edge(
        EdgeSpec::New {
            geometry: EdgeGeometry::Curve { curve, range },
            start,
            end,
            tolerance,
        },
        parent,
    )
}

/// The stripe's two faces either side of the run-over, and the rolled
/// face, appended to `rw.added`: the first face is the blend's own
/// (`made.added`), from its start's arcs `start` to the first crossing's
/// arc; the second from the second crossing's arc to its end's arcs
/// `end`; the rolled face counter-clockwise in its `(u, v)`, across its
/// first crossing's arc from `H` to `E`, up `E`'s pieces, back across the
/// second's and down its foot on `H`.
#[allow(clippy::too_many_arguments)]
pub(super) fn add_rolled_faces(
    m: &mut Model,
    rw: &mut Rewrite,
    shell: usize,
    blend: &Blend,
    rolled: &Rolled,
    made: &mut RolledMade,
    first: &mut usize,
    start: Vec<StoredUse>,
    end: Vec<StoredUse>,
) -> Result<(), OpError> {
    let [lo, hi] = &blend.contacts;
    let stripe_surface = m.add_surface(blend.stripe.surface.clone());
    let face = |loop_uses: Vec<StoredUse>, surface| AddedFace {
        shell,
        surface,
        orientation: along(blend.stripe.convex),
        loops: vec![loop_uses],
        tolerance: blend.stripe.tolerance,
    };
    let mut before = start;
    before.push(StoredUse::new(
        m,
        EdgeKey::New(made.contacts[1][0]),
        true,
        &hi.on_blend,
    ));
    before.push(StoredUse::new(
        m,
        EdgeKey::New(made.arcs[0]),
        false,
        &rolled.arcs[0].on_stripe,
    ));
    before.push(StoredUse::new(
        m,
        EdgeKey::New(made.contacts[0][0]),
        false,
        &lo.on_blend,
    ));
    let mut after = vec![StoredUse::new(
        m,
        EdgeKey::New(made.arcs[1]),
        true,
        &rolled.arcs[1].on_stripe,
    )];
    after.push(StoredUse::new(
        m,
        EdgeKey::New(made.contacts[1][1]),
        true,
        &hi.on_blend,
    ));
    after.extend(end);
    after.push(StoredUse::new(
        m,
        EdgeKey::New(made.contacts[0][1]),
        false,
        &lo.on_blend,
    ));
    *first = rw.face(face(before, stripe_surface));
    let surface = m.add_surface(blend.stripe.surface.clone());
    made.added[0] = rw.face(face(after, surface));
    push_rolled_face(m, rw, shell, rolled, made)?;
    Ok(())
}

/// The rolled face, appended to `rw.added`, its index `made.added[1]`:
/// counter-clockwise in its `(u, v)`, across its first crossing's arc from
/// `H` to `E`, up `E`'s pieces, back across the second's and down its foot
/// on `H`.
pub(super) fn push_rolled_face(
    m: &mut Model,
    rw: &mut Rewrite,
    shell: usize,
    rolled: &Rolled,
    made: &mut RolledMade,
) -> Result<(), OpError> {
    // The arcs run from the contact at `u = 0` to the one at `β`: from `H`
    // to `E` where the contact on `H` is the first.
    let h_first = rolled.on_f == 1;
    let mut around = vec![StoredUse::new(
        m,
        EdgeKey::New(made.arcs[0]),
        h_first,
        &rolled.arcs[0].on_rolled,
    )];
    for &(i, forward) in &rolled.rolled_on {
        let pcurve = (rolled.pieces[i].on_rolled.as_ref()).ok_or(invariant(
            "a pcurve on the rolled face of a piece it rolls on",
        ))?;
        around.push(StoredUse::new(
            m,
            EdgeKey::New(made.pieces[i]),
            forward,
            pcurve,
        ));
    }
    around.push(StoredUse::new(
        m,
        EdgeKey::New(made.arcs[1]),
        !h_first,
        &rolled.arcs[1].on_rolled,
    ));
    around.push(StoredUse::new(
        m,
        EdgeKey::New(made.foot),
        false,
        &Curve2::along_v(0.0),
    ));
    let surface = m.add_surface(rolled.surface.clone());
    made.added[1] = rw.face(AddedFace {
        shell,
        surface,
        orientation: rolled.orientation,
        loops: vec![around],
        tolerance: rolled.tolerance,
    });
    Ok(())
}
