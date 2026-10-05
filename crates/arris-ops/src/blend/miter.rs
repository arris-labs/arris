//! Two blends meeting at a vertex: the miter and its trim arc.

use core::f64::consts::{PI, TAU};
use std::collections::BTreeSet;

use arris_geom::region2::Side;
use arris_geom::{Curve, Curve2, GeomKind, SurfaceKind, pcurve_on};
use arris_math::{
    Frame, Interval, Meter, Point3, Tolerance, UnitVec3, Vec3, shift_into_range, wrap_angle,
};
use arris_topo::{EdgeId, FaceId, Model, VertexId};

use super::ends::Trim;
use super::stripe::{
    Section, Stripe, arc_between, band_u, chord, line_origin, lines_cross, on_side_of_face,
    section_between,
};
use super::view::{View, convex_edge};
use super::{degenerate, invariant};
use crate::error::{OpError, Reason, fault_of};
use crate::rebuild::forward;

/// Two blends meeting at a vertex whose third edge stays sharp
/// (ADR-0007): the curve they meet in — the ellipse where two
/// equal-radius cylinders cross, in the plane bisecting their axes
/// through the ball's one centre, or the line where two chamfer planes
/// cross — from the point where the two contacts on the shared face cross
/// to the point on the third edge where the other two contacts meet it.
/// Where the dihedrals differ (ADR-0044) the curve stops at `m` on the
/// narrower blend's far contact, and a trim arc on the wider blend runs on
/// from `m` to the third edge.
pub(super) struct Miter {
    /// The two blended edges, in the blends' order.
    pub(super) edges: [EdgeId; 2],
    /// For each of the two stripes, which of its contacts ends at `q`: the
    /// one on the shared face.
    pub(super) shared: [usize; 2],
    /// The contact lines' parameters at the miter, `[side][contact]`.
    pub(super) t: [[f64; 2]; 2],
    /// Where the two contacts on the shared face cross; at a junction
    /// whose runs share no face, where two meet on a tangent edge.
    pub(super) q: Point3,
    /// Where the other two contacts meet the third edge; `m`, on the
    /// narrower blend's far contact, where a trim arc runs on from it.
    pub(super) p3: Point3,
    pub(super) curve: Curve,
    pub(super) range: Interval,
    /// `true` when `range.lo()` is at `q`.
    pub(super) q_first: bool,
    /// For each side, `true` when `range.lo()` is at that stripe's
    /// contact at `u = 0`.
    pub(super) lo_first: [bool; 2],
    /// The ellipse's pcurve on each stripe's cylinder, placed.
    pub(super) on_blend: [Curve2; 2],
    /// The third edge shortened to `p3`, or to the trim arc's end where
    /// there is one; `None` at a vertex of two edges that continue one
    /// another (ADR-0041), which has no third edge.
    pub(super) trim: Option<Trim>,
    /// At a junction whose runs share no face (ADR-0039 §2), the tangent
    /// edge `q` lies on, shortened to it.
    pub(super) q_trim: Option<Trim>,
    /// Where the dihedrals differ, the wider blend's trim arc (ADR-0044).
    pub(super) trim_arc: Option<TrimArc>,
    pub(super) tolerance: f64,
    pub(super) q_tolerance: f64,
    pub(super) p3_tolerance: f64,
}

/// The trim arc of a miter of unequal dihedrals (ADR-0044 §3): the
/// section of the wider blend with the narrower blend's far face, from
/// `m` (the miter's `p3`) to where the wider blend's far contact meets the
/// third edge, which that face takes in its loop at the corner vertex.
pub(super) struct TrimArc {
    /// The wider blend's side in the miter.
    pub(super) wide: usize,
    /// The corner vertex, where the face's loop takes the arc.
    pub(super) vertex: VertexId,
    /// The narrower blend's far face.
    pub(super) face: FaceId,
    /// The narrower blend's edge: the corner edge of `face` at `m`.
    pub(super) narrow_edge: EdgeId,
    /// The arc's end on the third edge.
    pub(super) end: Point3,
    pub(super) end_tolerance: f64,
    pub(super) curve: Curve,
    pub(super) range: Interval,
    /// `true` when `range.lo()` is at `m`.
    pub(super) m_first: bool,
    /// Its pcurve on `face`, exact.
    pub(super) on_face: Curve2,
    /// Its pcurve on the wider blend, placed.
    pub(super) on_blend: Curve2,
    pub(super) tolerance: f64,
}

impl Miter {
    /// The stripe at `side`'s arcs at the miter in its end's order, from
    /// its contact at `u = 0` to the other: whether each runs along its
    /// range in that order, and its pcurve on the blend. The wider blend of
    /// a miter of unequal dihedrals has the curve and the trim arc, the
    /// trim arc at its far contact.
    pub(super) fn arcs(&self, side: usize) -> Vec<(bool, &Curve2)> {
        let curve = (self.lo_first[side], &self.on_blend[side]);
        match &self.trim_arc {
            Some(arc) if arc.wide == side => {
                let at_zero = self.shared[side] == 0;
                let trim = (arc.m_first == at_zero, &arc.on_blend);
                if at_zero {
                    vec![curve, trim]
                } else {
                    vec![trim, curve]
                }
            }
            Some(_) | None => vec![curve],
        }
    }
}

/// The miter of stripes `a` and `b` at `vertex`, a corner of three edges
/// whose third stays sharp (ADR-0007). The two stripes share one face,
/// so two fillets' axes, both `r` off it, cross at the ball's one centre,
/// and the miter is the ellipse of the two cylinders in the plane
/// bisecting their axes from where the two contacts on the shared face
/// cross, its pcurve on each cylinder fitted by the oblique-section rule;
/// two chamfers meet in the line from the same point. Where the far
/// contacts meet the third edge at one point, the curve ends there and the
/// third edge is shortened to it. Where two fillets' dihedrals differ they
/// meet it at two (ADR-0044): the ellipse ends at `m`, where it crosses the
/// narrower blend's far contact, and the wider blend's trim arc runs on
/// from `m` to its own point, where the third edge is cut (`trim_arc`); two
/// chamfers of unequal angles with the third edge do the same in lines, `m`
/// where the narrower chamfer's far contact crosses the wider's plane.
/// Blends not both convex or both concave, or edges that do not share
/// exactly one face are `Reason::VertexBlend`; a third edge shorter than
/// the cut is `Reason::BlendTooLarge`.
#[allow(clippy::too_many_arguments)]
pub(super) fn miter(
    m: &Model,
    view: &View,
    a: &Stripe,
    b: &Stripe,
    vertex: VertexId,
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<Miter, OpError> {
    meter.tick()?;
    let v = forward(vertex);
    let (ea, eb) = (forward(a.edge), forward(b.edge));
    let vertex_blend = || degenerate(vec![ea, eb, v], Reason::VertexBlend);
    // A ruling blend's contact on its cylinder meets no other blend's
    // contact on the third edge: two arcs, a backlog line C6 left.
    if a.ruling || b.ruling {
        return Err(vertex_blend());
    }
    // The shared face, and each stripe's contact on it.
    let mut shared: Option<(usize, usize, FaceId)> = None;
    for (ka, &fa) in a.faces.iter().enumerate() {
        for (kb, &fb) in b.faces.iter().enumerate() {
            if fa == fb && shared.replace((ka, kb, fa)).is_some() {
                return Err(vertex_blend());
            }
        }
    }
    let Some((ka, kb, _)) = shared else {
        return Err(vertex_blend());
    };
    let (face_a, face_b) = (a.faces[1 - ka], b.faces[1 - kb]);
    // The third edge: the vertex's one edge that is not blended, between
    // the two faces the blends do not share.
    let at_vertex = view
        .vertex_edges
        .get(&vertex)
        .ok_or(invariant("the corner vertex's edges"))?;
    let third: Vec<EdgeId> = at_vertex
        .iter()
        .copied()
        .filter(|&e| e != a.edge && e != b.edge)
        .collect();
    let (&[e3], 3) = (third.as_slice(), at_vertex.len()) else {
        return Err(vertex_blend());
    };
    let faces3: BTreeSet<FaceId> = view
        .uses
        .get(&e3)
        .ok_or(invariant("the corner edge's uses"))?
        .iter()
        .map(|u| u.face)
        .collect();
    if faces3 != BTreeSet::from([face_a, face_b]) {
        return Err(vertex_blend());
    }
    // Both convex or both concave: one convex and one concave is a corner
    // the closed forms do not cover.
    if a.convex != b.convex {
        return Err(vertex_blend());
    }
    let tolerance = a.tolerance.max(b.tolerance);
    let crossing = |o1: Point3, d1: Vec3, o2: Point3, d2: Vec3, what: &'static str| {
        let (t1, t2) = lines_cross(o1, d1, o2, d2, tol).ok_or_else(vertex_blend)?;
        let (p1, p2) = (o1 + t1 * d1, o2 + t2 * d2);
        if (p1 - p2).norm() > tolerance {
            return Err(invariant(what));
        }
        Ok((p1 + (p2 - p1) * 0.5, t1, t2))
    };
    // Where the two contacts on the shared face cross.
    let (q, tqa, tqb) = crossing(
        line_origin(&a.lines[ka])?,
        a.d,
        line_origin(&b.lines[kb])?,
        b.d,
        "the contacts on the shared face through one point",
    )?;
    // Where the other two contacts meet the third edge.
    let e3_entity = *m.edge(e3)?;
    let Some((c3_id, range3)) = e3_entity.curve() else {
        return Err(vertex_blend());
    };
    let c3 = m.curve(c3_id)?;
    let &Curve::Line {
        origin: o3,
        direction: d3,
    } = c3
    else {
        return Err(OpError::Unsupported {
            a: (GeomKind::Curve(c3.kind()), forward(e3)),
            b: (GeomKind::Surface(SurfaceKind::Plane), forward(face_a)),
        });
    };
    let d3: Vec3 = d3.into_inner();
    let (pa, tpa, t3a) = crossing(
        line_origin(&a.lines[1 - ka])?,
        a.d,
        o3,
        d3,
        "the first blend's contact through the third edge",
    )?;
    let (pb, tpb, t3b) = crossing(
        line_origin(&b.lines[1 - kb])?,
        b.d,
        o3,
        d3,
        "the second blend's contact through the third edge",
    )?;
    // Unequal dihedrals put the far contacts through two points of the
    // third edge: the wider blend's, the farther from the vertex, is where
    // the edge is cut, and the curve stops at the narrower's far contact
    // (ADR-0044 §2).
    let corner_point = m.vertex(vertex)?.point();
    let wide = match (a.section, (pa - pb).norm() > tolerance) {
        (_, false) => None,
        (Section::Round { .. }, true) if (a.beta - b.beta).abs() <= tol.angular => {
            // Equal dihedrals put two fillets' far contacts through one
            // point.
            return Err(invariant(
                "the two contacts through one point of the third edge",
            ));
        }
        // Two chamfers' meet there only when their edges make equal angles
        // with the third edge; otherwise the wider one is the same
        // distance's farther.
        (Section::Round { .. } | Section::Flat, true) => Some(usize::from(
            (pb - corner_point).norm() > (pa - corner_point).norm(),
        )),
    };
    let (p3, t3) = match wide {
        Some(1) => (pb, t3b),
        Some(_) | None => (pa, t3a),
    };
    // The third edge shortened to that point.
    let too_large = || degenerate(vec![ea, eb, forward(e3)], Reason::BlendTooLarge);
    let Some(tc) = shift_into_range(range3, t3, c3.period()) else {
        return Err(too_large());
    };
    let cuts_lo = e3_entity.start() == vertex;
    let far = if cuts_lo {
        e3_entity.end()
    } else {
        e3_entity.start()
    };
    let far_vertex = m.vertex(far)?;
    let kept_length = if cuts_lo {
        range3.hi() - tc
    } else {
        tc - range3.lo()
    };
    if kept_length <= 0.0 || (p3 - far_vertex.point()).norm() <= far_vertex.tolerance() {
        return Err(too_large());
    }
    let trim = Trim {
        edge: e3,
        t: tc,
        cuts_lo,
    };
    // The curve, and where it ends: `p3`, or `m` on the narrower blend's
    // far contact with that contact's parameter there.
    let (curve, range, q_first, end, t_m) = match (a.section, b.section) {
        (Section::Flat, Section::Flat) => {
            // Where the angles differ, `m`: the narrower chamfer's far
            // contact through the wider chamfer's plane.
            let (end, t_m) = match wide {
                None => (p3, None),
                Some(w) => {
                    let (narrow, wider, k) = if w == 0 { (b, a, kb) } else { (a, b, ka) };
                    let origin = line_origin(&narrow.lines[1 - k])?;
                    let o0 = line_origin(&wider.lines[0])?;
                    let o1 = line_origin(&wider.lines[1])?;
                    let n = wider.d.cross(&(o1 - o0));
                    let along = narrow.d.dot(&n);
                    if along.abs() <= tol.angular * n.norm() {
                        return Err(invariant("the narrower contact across the wider plane"));
                    }
                    let t_m = (o0 - origin).dot(&n) / along;
                    (origin + t_m * narrow.d, Some(t_m))
                }
            };
            let (curve, range) = chord(q, end, tol)?;
            (curve, range, true, end, t_m)
        }
        (Section::Round { .. }, Section::Flat) | (Section::Flat, Section::Round { .. }) => {
            return Err(invariant("one kind of blend in one call"));
        }
        (
            Section::Round {
                axis_origin: origin_a,
                radius,
            },
            Section::Round {
                axis_origin: origin_b,
                ..
            },
        ) => {
            // The ball's centre, where the two axes cross.
            let (centre, _, _) = crossing(
                origin_a,
                a.d,
                origin_b,
                b.d,
                "the two axes through the ball's centre",
            )?;
            // The ellipse: in the plane through the centre bisecting the two
            // axes — its normal the difference of the edges' directions toward
            // the vertex — with its minor axis `r` toward the shared face and its
            // major axis `r / |n · Z|` toward the third edge.
            let toward = |s: &Stripe| if s.end == vertex { s.d } else { -s.d };
            let Some(n) = UnitVec3::try_new(toward(a) - toward(b), tol.linear) else {
                return Err(vertex_blend());
            };
            let n: Vec3 = n.into_inner();
            let cos = n.dot(&a.d).abs();
            if cos <= tol.angular || (cos - n.dot(&b.d).abs()).abs() > tol.angular {
                return Err(invariant("the miter plane at one angle to both axes"));
            }
            let y = q - centre;
            if (y.norm() - radius).abs() > tolerance {
                return Err(invariant(
                    "the ball touching the shared face where the contacts cross",
                ));
            }
            let y = y / y.norm();
            let Some(x) = UnitVec3::try_new(n.cross(&y), tol.linear) else {
                return Err(invariant("the miter's major axis"));
            };
            let mut x: Vec3 = x.into_inner();
            if x.dot(&(p3 - centre)) < 0.0 {
                x = -x;
            }
            let frame = Frame::new(centre, x.cross(&y), x)?;
            let curve = Curve::Ellipse {
                frame,
                major_radius: radius / cos,
                minor_radius: radius,
            };
            let param = |p: Point3| -> Result<f64, OpError> {
                let projection = curve.project(p).map_err(fault_of)?;
                if projection.distance > tolerance {
                    return Err(invariant("the miter's ends on its ellipse"));
                }
                Ok(projection.t)
            };
            // Where the dihedrals differ, `m`: the narrower blend's far
            // contact, a ruling of its cylinder, through the ellipse's plane.
            let (end, t_m) = match wide {
                None => (p3, None),
                Some(w) => {
                    let (narrow, k) = if w == 0 { (b, kb) } else { (a, ka) };
                    let origin = line_origin(&narrow.lines[1 - k])?;
                    let along = narrow.d.dot(&n);
                    if along.abs() <= tol.angular {
                        return Err(invariant("the narrower contact across the miter plane"));
                    }
                    let t_m = (centre - origin).dot(&n) / along;
                    (origin + t_m * narrow.d, Some(t_m))
                }
            };
            let (tq, tp) = (param(q)?, param(end)?);
            // The arc between them: the way round that lies inside both blends,
            // between their contacts in `u`.
            let inside = |t: f64| {
                [a, b].iter().all(|s| {
                    let l = s.frame.to_local(curve.point(t));
                    let u = wrap_angle(l.y.atan2(l.x));
                    u > 0.0 && u < s.beta
                })
            };
            let up = |t: f64, from: f64| if t >= from { t } else { t + TAU };
            let direct = (tq + up(tp, tq)) / 2.0;
            let mid = if inside(direct) {
                direct
            } else if inside(direct + PI) {
                direct + PI
            } else {
                return Err(invariant("a miter arc inside both blends"));
            };
            let (range, q_first) = arc_between(tq, tp, mid)?;
            (curve, range, q_first, end, t_m)
        }
    };
    let arc_tol = Tolerance::new(tolerance, tol.angular);
    let shared = [ka, kb];
    let lo_first = shared.map(|k| q_first == (k == 0));
    let mut on_blend: Vec<Curve2> = Vec::with_capacity(2);
    for (i, s) in [a, b].into_iter().enumerate() {
        let pcurve = pcurve_on(&curve, range, &s.surface, arc_tol, meter).map_err(fault_of)?;
        on_blend.push(s.place(pcurve, range.lo(), if lo_first[i] { 0.0 } else { s.u1 }));
    }
    let on_blend: [Curve2; 2] = on_blend
        .try_into()
        .map_err(|_| invariant("the miter's two pcurves"))?;
    let mut t = [[0.0; 2]; 2];
    t[0][ka] = tqa;
    t[0][1 - ka] = tpa;
    t[1][kb] = tqb;
    t[1][1 - kb] = tpb;
    let end_tolerance = tolerance.max(e3_entity.tolerance());
    let trim_arc = match (wide, t_m) {
        (Some(w), Some(t_m)) => {
            let narrow = 1 - w;
            t[narrow][1 - shared[narrow]] = t_m;
            let (ws, ns) = if w == 0 { (a, b) } else { (b, a) };
            let face = ns.faces[1 - shared[narrow]];
            Some(trim_arc(
                m,
                view,
                [ws, ns],
                w,
                face,
                e3,
                vertex,
                [end, p3],
                end_tolerance,
                tol,
                samples,
                meter,
            )?)
        }
        (Some(_), None) | (None, Some(_)) => {
            return Err(invariant("a trim arc where the dihedrals differ"));
        }
        (None, None) => None,
    };
    Ok(Miter {
        edges: [a.edge, b.edge],
        shared,
        t,
        q,
        p3: end,
        curve,
        range,
        q_first,
        lo_first,
        on_blend,
        trim: Some(trim),
        q_trim: None,
        trim_arc,
        tolerance,
        q_tolerance: tolerance,
        p3_tolerance: if wide.is_some() {
            tolerance
        } else {
            end_tolerance
        },
    })
}

/// The trim arc of a miter of unequal dihedrals (ADR-0044 §3): the
/// section of the wider stripe `stripes[0]`, the miter's side `wide`, with
/// `face`, the narrower stripe's far face, from `points[0]`, `m`, to
/// `points[1]` on the third edge `e3` — `section_between`'s, a conic exact
/// on the plane — with its pcurves on the face and on the wider blend. The
/// arc lies inside the face where the third edge is of the blends' sense;
/// a third edge of the other sense is `Reason::VertexBlend`, and an arc
/// leaving the face `Reason::BlendTooLarge`.
#[allow(clippy::too_many_arguments)]
pub(super) fn trim_arc(
    m: &Model,
    view: &View,
    stripes: [&Stripe; 2],
    wide: usize,
    face: FaceId,
    e3: EdgeId,
    vertex: VertexId,
    points: [Point3; 2],
    end_tolerance: f64,
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<TrimArc, OpError> {
    let [ws, ns] = stripes;
    let side = match convex_edge(m, view, e3)? {
        Some(convex) if convex == ws.convex => Side::Inside,
        Some(_) | None => {
            return Err(degenerate(
                vec![forward(ws.edge), forward(ns.edge), forward(vertex)],
                Reason::VertexBlend,
            ));
        }
    };
    let surface = m.surface(m.face(face)?.surface())?;
    let tolerance = ws
        .tolerance
        .max(ns.tolerance)
        .max(m.face(face)?.tolerance());
    let arc_tol = Tolerance::new(tolerance, tol.angular);
    let refuse = || OpError::Unsupported {
        a: (GeomKind::Surface(ws.surface.kind()), forward(ws.edge)),
        b: (GeomKind::Surface(surface.kind()), forward(face)),
    };
    let section = section_between(ws, surface, points, &refuse, arc_tol, meter)?;
    let (curve, range) = (section.curve, section.range);
    let on_face = pcurve_on(&curve, range, surface, arc_tol, meter).map_err(fault_of)?;
    if !on_side_of_face(m, face, &on_face, range, side, samples)? {
        return Err(degenerate(
            vec![forward(ws.edge), forward(ns.edge), forward(face)],
            Reason::BlendTooLarge,
        ));
    }
    let on_blend = pcurve_on(&curve, range, &ws.surface, arc_tol, meter).map_err(fault_of)?;
    let on_blend = ws.place(on_blend, range.lo(), band_u(ws, curve.point(range.lo()))?);
    Ok(TrimArc {
        wide,
        vertex,
        face,
        narrow_edge: ns.edge,
        end: points[1],
        end_tolerance: end_tolerance.max(section.gaps[1]),
        curve,
        range,
        m_first: section.lo_first,
        on_face,
        on_blend,
        tolerance,
    })
}
