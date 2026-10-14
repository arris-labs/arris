//! Blends meeting at a vertex along a run of edges: the junction.

use std::collections::BTreeSet;

use arris_geom::{Curve, Curve2, Surface, pcurve_on};
use arris_math::{Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec3};
use arris_topo::{EdgeId, FaceId, Model, VertexId};

use super::chain::parameter_at;
use super::ends::{cut_corner, on_corner_edge};
use super::miter::Miter;
use super::ring::{Ring, placed_uv};
use super::stripe::{Section, Stripe, chord, line_origin};
use super::{degenerate, invariant};
use crate::body_view::BodyView;
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild::forward;

/// One side of a junction: a line's stripe or an arc's ring.
#[derive(Clone, Copy)]
pub(super) enum Run<'a> {
    Line(&'a Stripe),
    Arc(&'a Ring),
}

/// A side of a junction read at its vertex.
pub(super) struct RunAt<'a> {
    pub(super) edge: EdgeId,
    /// The faces of the contacts, by contact.
    pub(super) faces: [FaceId; 2],
    /// The contacts' points at the vertex, by contact.
    pub(super) points: [Point3; 2],
    /// A fillet's ball centre there.
    pub(super) centre: Option<Point3>,
    pub(super) convex: bool,
    pub(super) tolerance: f64,
    pub(super) surface: &'a Surface,
    /// The edge's parameter there, which the contacts share; a walked
    /// stripe's surface `v`.
    pub(super) t: f64,
    /// `true` for a walked stripe (ADR-0062), whose ball at the vertex is
    /// the one touching its own faces there, not the closed forms' shared
    /// one: a face that turns tangent along a curve the vertex's normal
    /// plane does not hold puts the two balls apart.
    pub(super) walked: bool,
}

impl<'a> Run<'a> {
    pub(super) fn at(self, m: &Model, vertex: VertexId) -> Result<RunAt<'a>, OpError> {
        match self {
            Run::Line(s) => {
                // A walked stripe's contacts run on the surface's `v`: there,
                // the surface's isocurve is the ball's arc (ADR-0062).
                if let Section::Walked { ends, centres, .. } = s.section {
                    let j = usize::from(s.start != vertex);
                    let t = ends[j];
                    return Ok(RunAt {
                        edge: s.edge,
                        faces: s.faces,
                        points: s.lines.each_ref().map(|line| line.point(t)),
                        centre: centres.map(|c| c[j]),
                        convex: s.convex,
                        tolerance: s.tolerance,
                        surface: &s.surface,
                        t,
                        walked: true,
                    });
                }
                let t = parameter_at(m, s.edge, vertex)?.ok_or(invariant("a line edge's curve"))?;
                let mut points = [Point3::origin(); 2];
                for (p, line) in points.iter_mut().zip(&s.lines) {
                    *p = line_origin(line)? + t * s.d;
                }
                Ok(RunAt {
                    edge: s.edge,
                    faces: s.faces,
                    points,
                    centre: match s.section {
                        Section::Round { axis_origin, .. } => Some(axis_origin + t * s.d),
                        Section::Flat => None,
                        Section::Walked { .. } => {
                            return Err(invariant("a walked stripe read at its `v`"));
                        }
                    },
                    convex: s.convex,
                    tolerance: s.tolerance,
                    surface: &s.surface,
                    t,
                    walked: false,
                })
            }
            Run::Arc(r) => {
                let at_lo = m.edge(r.edge)?.start() == vertex;
                let (j, t) = if at_lo {
                    (0, r.range.lo())
                } else {
                    (1, r.range.hi())
                };
                Ok(RunAt {
                    edge: r.edge,
                    faces: r.contacts.each_ref().map(|c| c.face),
                    points: r.contacts.each_ref().map(|c| c.points[j]),
                    centre: r.centres.as_ref().map(|c| c.point(t)),
                    convex: r.convex,
                    tolerance: r.tolerance,
                    surface: &r.surface,
                    t,
                    walked: false,
                })
            }
        }
    }

    /// `pcurve` on the blend placed in its loop's translate, its point at
    /// `lo` on contact `contact` at the junction `at`: on a stripe at that
    /// contact's `u`, on a ring at the contacts' `u` there and that
    /// contact's `v`.
    pub(super) fn place(self, pcurve: Curve2, lo: f64, contact: usize, at: &RunAt<'_>) -> Curve2 {
        match self {
            Run::Line(s) => s.place(pcurve, lo, if contact == 0 { 0.0 } else { s.u1 }),
            Run::Arc(r) => {
                let u = r.contacts[0].on_blend.point(at.t).x;
                let v = r.contacts[contact].on_blend.point(at.t).y;
                placed_uv(pcurve, lo, Point2::new(u, v))
            }
        }
    }
}

/// The junction of `a` and `b` at the tangent vertex `vertex` (ADR-0035
/// §3), recorded as a miter is: the two runs share one face, their
/// contacts on it meet at `q`, and their other two meet at `p` on the
/// vertex's third edge `w`, which is shortened there. The arc between them
/// is the ball's great circle through `q` and `p` for fillets, square to
/// the edges' common direction, or the chord from `q` to `p` for chamfers;
/// every pcurve of it exact but on a walked stripe, where it is fitted
/// near the surface's isocurve there (ADR-0062). Two closed-form runs whose
/// points or ball centres differ by more than their tolerance, or one
/// convex and one concave, is an internal fault the tangent-vertex test
/// makes unreachable; with a walked run among them the points or centres
/// apart are `BlendReason::VertexBlend`, the balls touching their own
/// faces at the vertex being two (`RunAt::walked`); a `w` shorter
/// than the cut is `BlendReason::TooLarge`. Runs that share no face, met at
/// a vertex of four edges where both faces turn (ADR-0039 §2), meet the
/// same way with `q` on a second tangent edge, cut there too: each contact
/// of `a` meets the contact of `b` across the tangent edge between their
/// faces, `q` being the one from `a`'s contact at `u = 0`.
pub(super) fn junction(
    m: &Model,
    view: &BodyView,
    a: Run<'_>,
    b: Run<'_>,
    vertex: VertexId,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Miter, OpError> {
    meter.tick()?;
    let (ra, rb) = (a.at(m, vertex)?, b.at(m, vertex)?);
    let v = forward(vertex);
    let vertex_blend = || {
        degenerate(
            vec![forward(ra.edge), forward(rb.edge), v],
            Reason::Blend(BlendReason::VertexBlend),
        )
    };
    // The contacts of `a` and `b` on one face, as `(ka, kb)` pairs.
    let mut pairs: Vec<(usize, usize)> = Vec::with_capacity(2);
    for (ka, fa) in ra.faces.iter().enumerate() {
        for (kb, fb) in rb.faces.iter().enumerate() {
            if fa == fb {
                pairs.push((ka, kb));
            }
        }
    }
    // The vertex's other edges, the ones the runs' contacts end on.
    let at_vertex = view
        .vertex_edges
        .get(&vertex)
        .ok_or(invariant("the junction vertex's edges"))?;
    let others: Vec<EdgeId> = at_vertex
        .iter()
        .copied()
        .filter(|&e| e != ra.edge && e != rb.edge)
        .collect();
    let faces_of = |w: EdgeId| -> Result<BTreeSet<FaceId>, OpError> {
        Ok(view
            .uses
            .get(&w)
            .ok_or(invariant("the junction's corner edge's uses"))?
            .iter()
            .map(|u| u.face)
            .collect())
    };
    // `ka` and `kb` the contacts that meet at `q`, `wq` the edge `q` cuts
    // where the runs share no face, `w` the edge `p` cuts, if any.
    let (ka, kb, wq, w) = match (&pairs[..], &others[..]) {
        // A vertex of three edges: `q` on the shared face, `p` on the
        // third edge between the two faces the runs do not share.
        (&[(ka, kb)], &[w]) => {
            if faces_of(w)? != BTreeSet::from([ra.faces[1 - ka], rb.faces[1 - kb]]) {
                return Err(vertex_blend());
            }
            (ka, kb, None, Some(w))
        }
        // Runs that continue one another between the same two faces
        // (ADR-0041 §2): each contact meets the other run's on its own
        // face, `q` on `a`'s contact at `u = 0`, nothing cut.
        (&[(0, kb), (1, jb)], &[]) if jb == 1 - kb => (0, kb, None, None),
        // The same with the seam of one face at the vertex: `p` on the
        // contacts on that face, the seam cut there as a closed edge's is.
        (&[(0, kb), (1, jb)], &[w]) if jb == 1 - kb => {
            let seam = view
                .uses
                .get(&w)
                .ok_or(invariant("the junction's seam's uses"))?;
            match seam[..] {
                [s, t] if s.face == t.face && s.face == ra.faces[1] => (0, kb, None, Some(w)),
                [s, t] if s.face == t.face && s.face == ra.faces[0] => (1, 1 - kb, None, Some(w)),
                _ => return Err(vertex_blend()),
            }
        }
        // The same with a seam of each face at the vertex, both curved:
        // `q` on the contacts on `a`'s first face, its seam cut there, `p`
        // on the second's, that seam cut there.
        (&[(0, kb), (1, jb)], &[w0, w1]) if jb == 1 - kb => {
            let seam_of = |w: EdgeId, face: FaceId| -> Result<bool, OpError> {
                let uses = view
                    .uses
                    .get(&w)
                    .ok_or(invariant("the junction's seam's uses"))?;
                Ok(matches!(uses[..], [s, t] if s.face == t.face && s.face == face))
            };
            if seam_of(w0, ra.faces[0])? && seam_of(w1, ra.faces[1])? {
                (0, kb, Some(w0), Some(w1))
            } else if seam_of(w1, ra.faces[0])? && seam_of(w0, ra.faces[1])? {
                (0, kb, Some(w1), Some(w0))
            } else {
                return Err(vertex_blend());
            }
        }
        // A vertex of four where both faces turn (ADR-0039 §2): each
        // contact of `a` meets the contact of `b` across the tangent edge
        // between their faces, `q` on the one from `a`'s contact at
        // `u = 0`, `p` on the other.
        (&[], &[w0, w1]) => {
            let mut across: [Option<(usize, EdgeId)>; 2] = [None; 2];
            for w in [w0, w1] {
                let faces_w = faces_of(w)?;
                for (k, slot) in across.iter_mut().enumerate() {
                    for (j, &fb) in rb.faces.iter().enumerate() {
                        if faces_w == BTreeSet::from([ra.faces[k], fb])
                            && slot.replace((j, w)).is_some()
                        {
                            return Err(vertex_blend());
                        }
                    }
                }
            }
            let [Some((kb, wq)), Some((jb, w))] = across else {
                return Err(vertex_blend());
            };
            if jb == kb || wq == w {
                return Err(vertex_blend());
            }
            (0, kb, Some(wq), Some(w))
        }
        _ => return Err(vertex_blend()),
    };
    let tolerance = ra.tolerance.max(rb.tolerance);
    if ra.convex != rb.convex {
        return Err(invariant("both runs of a junction convex or both concave"));
    }
    // The closed forms' runs meet within their tolerance wherever the
    // tangent-vertex test holds; a walked run's ball may not (the field's
    // doc), which is a junction no shared arc makes: refused by name.
    let walked = ra.walked || rb.walked;
    let meet = |pa: Point3, pb: Point3, what: &'static str| {
        if (pa - pb).norm() > tolerance {
            return Err(if walked {
                vertex_blend()
            } else {
                invariant(what)
            });
        }
        Ok(pa + (pb - pa) * 0.5)
    };
    let q = meet(
        ra.points[ka],
        rb.points[kb],
        "the contacts on the shared face or the first tangent edge through one point",
    )?;
    let p = meet(
        ra.points[1 - ka],
        rb.points[1 - kb],
        "the other contacts through one point of the third edge",
    )?;
    // A walked run's contact is on a seam of the curved face only where the
    // loop crosses it square; at a slant the seam would be cut where the
    // contact crosses it, beside the junction, which is not built.
    for (edge, at) in [(wq, q), (w, p)] {
        if let Some(edge) = edge
            && walked
            && !on_corner_edge(m, edge, at)?
        {
            return Err(vertex_blend());
        }
    }
    let q_trim = wq
        .map(|wq| cut_corner(m, ra.edge, wq, vertex, q))
        .transpose()?;
    let trim = w
        .map(|w| cut_corner(m, ra.edge, w, vertex, p))
        .transpose()?;
    let (curve, range) = match (ra.centre, rb.centre) {
        (None, None) => chord(q, p, tol)?,
        (Some(ca), Some(cb)) => {
            let centre = meet(ca, cb, "the runs' balls one ball")?;
            let (x, y) = (q - centre, p - centre);
            let radius = x.norm();
            if (y.norm() - radius).abs() > tolerance {
                return Err(if walked {
                    vertex_blend()
                } else {
                    invariant("the ball touching both faces across")
                });
            }
            let Some(normal) = UnitVec3::try_new(x.cross(&y), tol.linear) else {
                return Err(invariant("a junction arc of positive angle"));
            };
            let frame = Frame::new(centre, normal.into_inner(), x)?;
            let angle = x.cross(&y).norm().atan2(x.dot(&y));
            (
                Curve::Circle { frame, radius },
                Interval::new(0.0, angle)
                    .map_err(|_| invariant("a junction arc of positive angle"))?,
            )
        }
        (Some(_), None) | (None, Some(_)) => {
            return Err(invariant("one kind of blend in one call"));
        }
    };
    let arc_tol = Tolerance::new(tolerance, tol.angular);
    let mut on_blend: Vec<Curve2> = Vec::with_capacity(2);
    for (run, at, k) in [(a, &ra, ka), (b, &rb, kb)] {
        let pcurve = pcurve_on(&curve, range, at.surface, arc_tol, meter).map_err(fault_of)?;
        on_blend.push(run.place(pcurve, range.lo(), k, at));
    }
    let on_blend: [Curve2; 2] = on_blend
        .try_into()
        .map_err(|_| invariant("the junction's two pcurves"))?;
    Ok(Miter {
        edges: [ra.edge, rb.edge],
        shared: [ka, kb],
        t: [[ra.t; 2], [rb.t; 2]],
        q,
        p3: p,
        curve,
        range,
        q_first: true,
        lo_first: [ka == 0, kb == 0],
        on_blend,
        trim,
        q_trim,
        trim_arc: None,
        tolerance,
        q_tolerance: match wq {
            Some(wq) => tolerance.max(m.edge(wq)?.tolerance()),
            None => tolerance,
        },
        p3_tolerance: match w {
            Some(w) => tolerance.max(m.edge(w)?.tolerance()),
            None => tolerance,
        },
    })
}
