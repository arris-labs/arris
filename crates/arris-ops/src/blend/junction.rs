//! Blends meeting at a vertex along a run of edges: the junction.

use arris_math::real;
use std::collections::{BTreeMap, BTreeSet};

use arris_geom::{Curve, Curve2, NurbsCurve2, SECTION_FIT_FRACTION, Surface, pcurve_on};
use arris_math::{Axis, Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec3, Vec3};
use arris_topo::{EdgeId, FaceId, Model, VertexId};

use super::Blend;
use super::chain::parameter_at;
use super::ends::{EndKind, cut_corner, on_corner_edge};
use super::miter::Miter;
use super::ring::{Ring, placed_uv};
use super::stripe::{Section, Stripe, chord, line_origin};
use super::{degenerate, invariant};
use crate::body_view::{BodyView, tangent_normals};
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
///
/// At a kink the chain runs through (ADR-0075 §2), `kink` is the vertex's
/// tolerance. The runs' end balls stand `2r·sin(θ/2)` apart along the
/// edges, so the contacts on the shared face end where they cross, the far
/// ones meet on the third edge, and the arc is the mean ball's between the
/// two points, its edge and vertices taking the half gap as tolerance
/// (`pin_kinks` adds the pin's). Contacts that do not cross, far ones apart
/// by more than the runs' tolerance, or balls by more than the vertex's are
/// `BlendReason::TangentChain`, the kink wider than the file says.
pub(super) fn junction(
    m: &Model,
    view: &BodyView,
    [a, b]: [Run<'_>; 2],
    vertex: VertexId,
    kink: Option<f64>,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Miter, OpError> {
    meter.tick()?;
    let (mut ra, mut rb) = (a.at(m, vertex)?, b.at(m, vertex)?);
    let (v, edges) = (forward(vertex), [forward(ra.edge), forward(rb.edge)]);
    let vertex_blend = || {
        degenerate(
            vec![edges[0], edges[1], v],
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
    let runs_tolerance = ra.tolerance.max(rb.tolerance);
    // Across a kink a walked contact's far end may lie past its face, which
    // ends short of the vertex by less than the vertex's tolerance: the
    // contact ends where it leaves the face, on the corner edge.
    let mut exits: [Option<f64>; 2] = [None; 2];
    if let Some(within) = kink {
        for (side, (run, at, far)) in [(a, &mut ra, 1 - ka), (b, &mut rb, 1 - kb)]
            .into_iter()
            .enumerate()
        {
            let Run::Line(s) = run else {
                continue;
            };
            let Section::Walked { .. } = s.section else {
                continue;
            };
            let inward = if s.start == vertex { 1.0 } else { -1.0 };
            let Some(t) = last_on_face(m, s, far, at.faces[far], at.t, inward, within)? else {
                return Err(degenerate(
                    vec![edges[0], edges[1], v],
                    Reason::Blend(BlendReason::TangentChain),
                ));
            };
            if t != at.t {
                at.points[far] = s.lines[far].point(t);
                exits[side] = Some(t);
            }
        }
    }
    // How far apart the runs may meet: their own tolerance, or across a kink
    // the vertex's.
    let tolerance = kink.map_or(runs_tolerance, |within| within.max(runs_tolerance));
    if ra.convex != rb.convex {
        return Err(invariant("both runs of a junction convex or both concave"));
    }
    // The closed forms' runs meet within their tolerance wherever the
    // tangent-vertex test holds; a walked run's ball may not (the field's
    // doc). Where the third edge leans, both runs are read again at the one
    // ball off the vertex (ADR-0072); any other is a junction no shared arc
    // makes: refused by name.
    let walked = ra.walked || rb.walked;
    let apart = |pa: Point3, pb: Point3| (pa - pb).norm() > tolerance;
    let leans = walked
        && kink.is_none()
        && (apart(ra.points[ka], rb.points[kb])
            || apart(ra.points[1 - ka], rb.points[1 - kb])
            || matches!((ra.centre, rb.centre), (Some(ca), Some(cb)) if apart(ca, cb)));
    if leans {
        let w = match (wq, w) {
            (None, Some(w)) => w,
            _ => return Err(vertex_blend()),
        };
        if !leaning(m, view, [a, b], [&mut ra, &mut rb], [ka, kb], w, tol)? {
            return Err(vertex_blend());
        }
    }
    let refused = |what: &'static str| {
        if kink.is_some() {
            degenerate(
                vec![edges[0], edges[1], v],
                Reason::Blend(BlendReason::TangentChain),
            )
        } else if walked {
            vertex_blend()
        } else {
            invariant(what)
        }
    };
    // The half of the widest gap a mean point closes: each run's distance
    // to the junction.
    let mut gap = 0.0_f64;
    // Across a kink the contacts on the shared face cross short of the
    // vertex, the balls' gap lying along the edges: they end there, on that
    // face exactly (ADR-0075 §2).
    // Contacts that do not cross, because the kink is too slight to tell
    // them from one line, end where they are, and meet if the vertex's
    // tolerance holds them.
    let crossed = match kink {
        Some(_) => shared_crossing(
            [a, b],
            [ka, kb],
            [ra.t, rb.t],
            [ra.points[ka], rb.points[kb]],
            tolerance,
            tol,
        ),
        None => None,
    };
    let mut meet = |pa: Point3, pb: Point3, within: f64, what: &'static str| {
        let d = (pa - pb).norm();
        if d > within {
            return Err(refused(what));
        }
        gap = gap.max(0.5 * d);
        Ok(pa + (pb - pa) * 0.5)
    };
    let q = match crossed {
        Some((_, q)) => q,
        None => meet(
            ra.points[ka],
            rb.points[kb],
            tolerance,
            "the contacts on the shared face or the first tangent edge through one point",
        )?,
    };
    // The far contacts meet on the third edge as one ball's would, a kink's
    // too: its faces there close their loops only at one point, unless the
    // kink is too slight for its contacts to cross.
    let p = meet(
        ra.points[1 - ka],
        rb.points[1 - kb],
        if kink.is_some() && crossed.is_some() {
            runs_tolerance
        } else {
            tolerance
        },
        "the other contacts through one point of the third edge",
    )?;
    // A walked run's contact is on a seam of the curved face only where the
    // loop crosses it square; at a slant the junction slides along the spine
    // to the ball whose contact is on the seam (ADR-0083), and the seam is
    // cut there. Any other corner edge the contact misses is refused by name.
    let mut slid = false;
    let mut q = q;
    let mut p = p;
    if walked && kink.is_none() && !leans {
        let off = [(wq, q), (w, p)]
            .into_iter()
            .filter_map(|(edge, at)| edge.map(|edge| (edge, at)))
            .map(|(edge, at)| on_corner_edge(m, edge, at).map(|on| (edge, on)))
            .collect::<Result<Vec<_>, _>>()?;
        let missed: Vec<EdgeId> = off
            .iter()
            .filter(|&&(_, on)| !on)
            .map(|&(edge, _)| edge)
            .collect();
        if !missed.is_empty() {
            if !slide(m, view, [a, b], [&mut ra, &mut rb], vertex, &missed)? {
                return Err(vertex_blend());
            }
            slid = true;
            q = meet(
                ra.points[ka],
                rb.points[kb],
                tolerance,
                "the contacts on the shared face through one point",
            )?;
            p = meet(
                ra.points[1 - ka],
                rb.points[1 - kb],
                tolerance,
                "the other contacts through one point of the third edge",
            )?;
        }
    }
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
            let centre = meet(ca, cb, tolerance, "the runs' balls one ball")?;
            let (x, y) = (q - centre, p - centre);
            let radius = x.norm();
            if (y.norm() - radius).abs() > tolerance {
                return Err(refused("the ball touching both faces across"));
            }
            let Some(normal) = UnitVec3::try_new(x.cross(&y), tol.linear) else {
                return Err(invariant("a junction arc of positive angle"));
            };
            let frame = Frame::new(centre, normal.into_inner(), x)?;
            let angle = real::atan2(x.cross(&y).norm(), x.dot(&y));
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
    // Across a kink the edge and its vertices stand off each run by the
    // gap the mean ball closes, within the vertex's tolerance; elsewhere the
    // runs' own tolerance covers it.
    let tolerance = match kink {
        Some(_) => runs_tolerance.max(gap),
        None => tolerance,
    };
    // Across a kink the arc's ends stand off each blend by up to the gap,
    // where the blend's domain ends and the projection clamps; the fit is
    // held to a quarter of its tolerance (`SECTION_FIT_FRACTION`), so the
    // tolerance is widened to leave the pcurve that gap and no more.
    let arc_tol = Tolerance::new(
        match kink {
            Some(_) => tolerance / SECTION_FIT_FRACTION,
            None => tolerance,
        },
        tol.angular,
    );
    let mut on_blend: Vec<Curve2> = Vec::with_capacity(2);
    for (run, at, k) in [(a, &ra, ka), (b, &rb, kb)] {
        let pcurve = pcurve_on(&curve, range, at.surface, arc_tol, meter).map_err(fault_of)?;
        on_blend.push(run.place(pcurve, range.lo(), k, at));
    }
    let on_blend: [Curve2; 2] = on_blend
        .try_into()
        .map_err(|_| invariant("the junction's two pcurves"))?;
    let mut t = [[ra.t; 2], [rb.t; 2]];
    if let Some(([ta, tb], _)) = crossed {
        (t[0][ka], t[1][kb]) = (ta, tb);
    }
    for (side, far) in [(0, 1 - ka), (1, 1 - kb)] {
        if let Some(exit) = exits[side] {
            t[side][far] = exit;
        }
    }
    Ok(Miter {
        edges: [ra.edge, rb.edge],
        shared: [ka, kb],
        t,
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
        leaning: leans || slid,
        kink,
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

/// Where the walked stripe `s`'s contact `k` on `face`, running inward from
/// `t_end` (`inward` ±1 along `t`), is last on its face when its end lies
/// past it: `t_end` itself if the contact is on the face there, else the
/// nearest parameter within `reach` of it, found by bisection, where the
/// contact is on the face within the pcurve's share of the stripe's
/// tolerance (`SECTION_FIT_FRACTION`); `None` where there
/// is none. A NURBS face ends in its domain, and a contact that crosses the
/// corner edge there runs on its extension, off the face.
fn last_on_face(
    m: &Model,
    s: &Stripe,
    k: usize,
    face: FaceId,
    t_end: f64,
    inward: f64,
    reach: f64,
) -> Result<Option<f64>, OpError> {
    let surface = m.surface(m.face(face)?.surface())?;
    let on = |t: f64| {
        surface
            .project(s.lines[k].point(t))
            .is_ok_and(|p| p.distance <= SECTION_FIT_FRACTION * s.tolerance)
    };
    if on(t_end) {
        return Ok(Some(t_end));
    }
    let at = |i: usize| t_end + inward * reach * (i as f64 / EXIT_SCAN as f64);
    let Some(first) = (1..=EXIT_SCAN).find(|&i| on(at(i))) else {
        return Ok(None);
    };
    let (mut off, mut inside) = (at(first - 1), at(first));
    for _ in 0..EXIT_BISECTIONS {
        let mid = 0.5 * (off + inside);
        if on(mid) {
            inside = mid;
        } else {
            off = mid;
        }
    }
    Ok(Some(inside))
}

/// The samples [`last_on_face`] scans inward for a bracket, and the
/// bisections it then takes: the interval narrows by `2⁻⁴⁰` of the scan's
/// step, past any tolerance.
const EXIT_SCAN: usize = 64;
const EXIT_BISECTIONS: usize = 40;

/// Where the contacts of two stripes on their shared face cross near the
/// kink: each one's parameter there, from `t`, and the point, both
/// contacts within `within` of their ends `ends` and the two within the
/// stripes' tolerance of each other. Gauss–Newton on the distance between
/// the two contacts, square in its two parameters; `None` for a ring or
/// where it does not settle.
fn shared_crossing(
    runs: [Run<'_>; 2],
    shared: [usize; 2],
    t: [f64; 2],
    ends: [Point3; 2],
    within: f64,
    tol: Tolerance,
) -> Option<([f64; 2], Point3)> {
    let [Run::Line(sa), Run::Line(sb)] = runs else {
        return None;
    };
    let at = |s: &Stripe, k: usize, t: f64| -> Option<(Point3, Vec3)> {
        match s.section {
            Section::Walked { .. } => {
                let e = s.lines[k].eval(t);
                Some((e.point, e.d1))
            }
            Section::Round { .. } | Section::Flat => {
                Some((line_origin(&s.lines[k]).ok()? + t * s.d, s.d))
            }
        }
    };
    let [mut ta, mut tb] = t;
    for _ in 0..CROSSING_STEPS {
        let ((pa, da), (pb, db)) = (at(sa, shared[0], ta)?, at(sb, shared[1], tb)?);
        // The normal equations of `da·Δa − db·Δb = pb − pa`.
        let r = pb - pa;
        let (aa, ab, bb) = (da.dot(&da), -da.dot(&db), db.dot(&db));
        let (ra, rb) = (da.dot(&r), -db.dot(&r));
        let det = aa * bb - ab * ab;
        if det.is_nan() || det <= 0.0 {
            return None;
        }
        let (dta, dtb) = ((ra * bb - ab * rb) / det, (aa * rb - ab * ra) / det);
        ta += dta;
        tb += dtb;
        if (dta * da).norm().max((dtb * db).norm()) <= tol.linear {
            let ((pa, _), (pb, _)) = (at(sa, shared[0], ta)?, at(sb, shared[1], tb)?);
            let close = (pa - pb).norm() <= sa.tolerance.max(sb.tolerance)
                && (pa - ends[0]).norm() <= within
                && (pb - ends[1]).norm() <= within;
            return close.then(|| ([ta, tb], pa + (pb - pa) * 0.5));
        }
    }
    None
}

/// The Gauss–Newton steps [`shared_crossing`] takes before giving up: two
/// contacts crossing at a few degrees settle in a handful.
const CROSSING_STEPS: usize = 32;

/// Pins the arc of each kink's junction (ADR-0075 §2) on each blend to its
/// own contacts' ends there, so that the blend's loop closes in (u, v):
/// the mean ball's arc projects onto each run's surface beside them. The
/// arc and the junction's vertices take the distance the pin moves it as
/// tolerance, within the vertex's: past it, or for a pcurve no pin keeps
/// exact, `BlendReason::TangentChain` naming both edges and the vertex. A
/// contact on the shared face that ends short of the junction's `q`, the
/// kink too slight for the contacts to cross, is pinned to `q` as a pcurve
/// too, its edge taking the shift as tolerance.
pub(super) fn pin_kinks(
    m: &Model,
    miters: &mut [Miter],
    blends: &mut [Blend],
) -> Result<(), OpError> {
    // Each kink's two runs, as blends by the miter's side.
    let mut runs: BTreeMap<usize, [Option<usize>; 2]> = BTreeMap::new();
    for (i, blend) in blends.iter().enumerate() {
        for kind in &blend.ends {
            if let EndKind::Miter { at, side } = *kind
                && miters[at].kink.is_some()
            {
                runs.entry(at).or_default()[side] = Some(i);
            }
        }
    }
    for (at, sides) in runs {
        let mt = &mut miters[at];
        let (Some(within), [Some(a), Some(b)]) = (mt.kink, sides) else {
            return Err(invariant("a kink's junction of two runs"));
        };
        let vertex = shared_vertex(m, mt.edges)?;
        let refused = || {
            degenerate(
                vec![forward(mt.edges[0]), forward(mt.edges[1]), forward(vertex)],
                Reason::Blend(BlendReason::TangentChain),
            )
        };
        let mut shift = 0.0_f64;
        for (side, i) in [a, b].into_iter().enumerate() {
            let (k, blend) = (mt.shared[side], &blends[i]);
            let on_blend = |c: usize| blend.contacts[c].on_blend.point(mt.t[side][c]);
            let (lo, hi) = if mt.q_first {
                (on_blend(k), on_blend(1 - k))
            } else {
                (on_blend(1 - k), on_blend(k))
            };
            let (arc, surface) = (&mt.on_blend[side], &blend.stripe.surface);
            shift = shift
                .max(distance_on(surface, arc.point(mt.range.lo()), lo))
                .max(distance_on(surface, arc.point(mt.range.hi()), hi));
            mt.on_blend[side] =
                pinned(arc, [(mt.range.lo(), lo), (mt.range.hi(), hi)]).ok_or_else(refused)?;
            // Contacts that end beside each other on the shared face, too
            // slight a kink to cross, end in the junction's `q` as pcurves:
            // the edge takes the shift as tolerance.
            let contact = &mut blends[i].contacts[k];
            if (contact.line.point(mt.t[side][k]) - mt.q).norm() > contact.tolerance {
                let surface = m.surface(m.face(contact.face)?.surface())?;
                let end = if (mt.t[side][k] - contact.range.hi()).abs()
                    < (mt.t[side][k] - contact.range.lo()).abs()
                {
                    contact.range.hi()
                } else {
                    contact.range.lo()
                };
                let other = if end == contact.range.hi() {
                    contact.range.lo()
                } else {
                    contact.range.hi()
                };
                let (old, kept) = (contact.on_face.point(end), contact.on_face.point(other));
                let target = surface.project(mt.q).map_err(|_| refused())?.uv;
                // In the translate of the face's loop the contact is in.
                let target = match surface.period()[0] {
                    Some(period) => Point2::new(
                        target.x + period * ((old.x - target.x) / period).round(),
                        target.y,
                    ),
                    None => target,
                };
                contact.on_face =
                    pinned(&contact.on_face, [(end, target), (other, kept)]).ok_or_else(refused)?;
                contact.tolerance = contact.tolerance.max(distance_on(surface, old, target));
            }
        }
        mt.tolerance += shift;
        mt.q_tolerance = mt.q_tolerance.max(mt.tolerance);
        mt.p3_tolerance = mt.p3_tolerance.max(mt.tolerance);
        if mt.q_tolerance.max(mt.p3_tolerance) > within {
            return Err(refused());
        }
    }
    Ok(())
}

/// The vertex the two edges of a junction share.
fn shared_vertex(m: &Model, [a, b]: [EdgeId; 2]) -> Result<VertexId, OpError> {
    let (ea, eb) = (m.edge(a)?, m.edge(b)?);
    [ea.start(), ea.end()]
        .into_iter()
        .find(|&v| v == eb.start() || v == eb.end())
        .ok_or(invariant("a junction's two edges at one vertex"))
}

/// How far apart `surface` puts the (u, v) points `a` and `b`.
fn distance_on(surface: &Surface, a: Point2, b: Point2) -> f64 {
    (surface.point(a.x, a.y) - surface.point(b.x, b.y)).norm()
}

/// `pcurve` with its points at the two given parameters moved to the two
/// given points: a line becomes the degree-1 B-spline between them, a
/// B-spline takes each parameter as a knot of full multiplicity, where the
/// curve passes through a control point whatever the weights, and that
/// point is moved. Between them the curve moves no further than its ends
/// do, its rational basis summing to one. `None` for a circle or an
/// ellipse, or a parameter outside the curve's domain.
fn pinned(pcurve: &Curve2, pins: [(f64, Point2); 2]) -> Option<Curve2> {
    match pcurve {
        Curve2::Line { .. } => {
            let [(s0, p0), (s1, p1)] = pins;
            let ((lo, a), (hi, b)) = if s0 < s1 {
                ((s0, p0), (s1, p1))
            } else {
                ((s1, p1), (s0, p0))
            };
            NurbsCurve2::new(1, vec![lo, lo, hi, hi], vec![a, b], vec![1.0, 1.0])
                .ok()
                .map(Curve2::Nurbs)
        }
        Curve2::Nurbs(c) => {
            let degree = c.degree();
            let mut c = c.clone();
            for (s, _) in pins {
                let times = c.knots().iter().filter(|&&k| k == s).count();
                if times < degree {
                    c = c.insert_knot(s, degree - times).ok()?;
                }
            }
            let mut points = c.control_points().to_vec();
            for (s, target) in pins {
                let knots = c.knots();
                let last = knots.iter().rposition(|&k| k == s)?;
                let j = if last + 1 == knots.len() {
                    points.len() - 1
                } else {
                    last.checked_sub(degree)?
                };
                *points.get_mut(j)? = target;
            }
            NurbsCurve2::new(degree, c.knots().to_vec(), points, c.weights().to_vec())
                .ok()
                .map(Curve2::Nurbs)
        }
        Curve2::Circle { .. } | Curve2::Ellipse { .. } => None,
    }
}

/// The shared ball of a junction whose tangent edge leans (ADR-0072 §1):
/// its centre `c*` and its feet on the shared plane and on the edge.
#[derive(Clone, Copy, Debug)]
pub(super) struct Leaning {
    /// `c*`, at the radius from the shared plane `P` and from both faces
    /// tangent along the edge `w`.
    pub(super) centre: Point3,
    /// `c*`'s foot on `P`, where the runs' contacts on it meet.
    pub(super) q: Point3,
    /// `c*`'s foot on `w`, a point of its line, where `w` is cut.
    pub(super) p: Point3,
}

/// `c* = w_r ∩ offset(P, σ r)` (ADR-0072 §1). `plane` is `P`, a point of
/// it and its outward normal; `w` the vertex's third edge, a line; `n` the
/// outward normal its two faces share all along it; `σ` −1 for a convex
/// blend, +1 for a concave one. The arc of the junction is the great circle
/// from `q` to `p` (§2). `None` where `w` is parallel to `P` within
/// `angular`: no ball touches the three there.
pub(super) fn leaning_ball(
    plane: Axis,
    w: Axis,
    n: UnitVec3,
    radius: f64,
    convex: bool,
    angular: f64,
) -> Option<Leaning> {
    let sigma = if convex { -1.0 } else { 1.0 };
    let (n_p, d) = (plane.direction.into_inner(), w.direction.into_inner());
    let across = d.dot(&n_p);
    if across.abs() <= angular {
        return None;
    }
    // `w_r`'s point at `s` lies at `σ r` from `P` along its normal.
    let o = w.origin + sigma * radius * n.into_inner();
    let s = (sigma * radius - (o - plane.origin).dot(&n_p)) / across;
    let centre = o + s * d;
    Some(Leaning {
        centre,
        q: centre - sigma * radius * n_p,
        p: centre - sigma * radius * n.into_inner(),
    })
}

/// `c*` where the shared face `P` is no plane or `w` no line: the point of
/// `w_r(s) = w(s) + σ r n(s)`, `n` the far faces' outward normal at `s`,
/// that stands at `σ r` from `P` along `P`'s normal at its foot (ADR-0072
/// §1, read along the curve). Secant steps on `s` from the parameter of
/// `guess` on `w`, which the runs' own balls bound; `None` where they do
/// not settle, the far faces turn from tangent along `w`, or the foot is
/// ambiguous.
#[allow(clippy::too_many_arguments)]
fn curved_ball(
    m: &Model,
    view: &BodyView,
    face: FaceId,
    w: &Curve,
    range: Interval,
    guess: Point3,
    radius: f64,
    convex: bool,
    far_normal: &dyn Fn(f64) -> Result<Option<UnitVec3>, OpError>,
    tol: Tolerance,
) -> Result<Option<Leaning>, OpError> {
    let sigma = if convex { -1.0 } else { 1.0 };
    let surface = m.surface(m.face(face)?.surface())?;
    // The centre at `s`, its foot on `P` and how far it stands from `σ r`
    // along `P`'s normal there.
    let at = |s: f64| -> Result<Option<(Point3, Point3, f64)>, OpError> {
        let Some(n) = far_normal(s)? else {
            return Ok(None);
        };
        let centre = w.point(s) + sigma * radius * n.into_inner();
        let Ok(foot) = surface.project(centre) else {
            return Ok(None);
        };
        let n_p = view.outward(m, face, foot.uv)?;
        Ok(Some((
            centre,
            foot.point,
            (centre - foot.point).dot(&n_p) - sigma * radius,
        )))
    };
    let Ok(start) = w.project(guess) else {
        return Ok(None);
    };
    // On a closed `w`, in the turn that holds its range.
    let turn = w.period().map_or(0.0, |period| {
        period * ((range.midpoint() - start.t) / period).round()
    });
    let (mut s0, mut s1) = (start.t + turn, start.t + turn + BALL_SEED * radius);
    let (Some((_, _, mut g0)), Some((_, _, mut g1))) = (at(s0)?, at(s1)?) else {
        return Ok(None);
    };
    for _ in 0..BALL_STEPS {
        if g1.abs() <= tol.linear {
            let Some((centre, q, _)) = at(s1)? else {
                return Ok(None);
            };
            return Ok(Some(Leaning {
                centre,
                q,
                p: w.point(s1),
            }));
        }
        if g1 == g0 {
            return Ok(None);
        }
        let s2 = s1 - g1 * (s1 - s0) / (g1 - g0);
        if !(s2.is_finite() && range.lo() - radius <= s2 && s2 <= range.hi() + radius) {
            return Ok(None);
        }
        let Some((_, _, g2)) = at(s2)? else {
            return Ok(None);
        };
        (s0, g0, s1, g1) = (s1, g1, s2, g2);
    }
    Ok(None)
}

/// The second secant sample of [`curved_ball`], a step from the first of
/// this fraction of the radius: a fraction of the ball's size, not a
/// tolerance.
const BALL_SEED: f64 = 1e-3;

/// The secant steps [`curved_ball`] takes before giving up: the ball moves
/// by a fraction of the radius, and the distance is smooth in `s`.
const BALL_STEPS: usize = 40;

/// Reads both runs of a junction again at the shared ball of a leaning
/// third edge `w` (ADR-0072): each run's contacts, its ball's centre and its
/// parameter where the ball is `c*`, `false` where the construction does
/// not hold. It holds for two stripes of a fillet whose shared face is a
/// plane and whose `w` is a line along which the far faces share one
/// normal, with `c*` on each stripe's spine within its tolerance: a table
/// stripe's axis, or a walked stripe's surface, which runs `REACH_RADII`
/// past its edge's ends.
fn leaning(
    m: &Model,
    view: &BodyView,
    runs: [Run<'_>; 2],
    at: [&mut RunAt<'_>; 2],
    shared: [usize; 2],
    w: EdgeId,
    tol: Tolerance,
) -> Result<bool, OpError> {
    let [Run::Line(sa), Run::Line(sb)] = runs else {
        return Ok(false);
    };
    let (ka, face) = (shared[0], at[0].faces[shared[0]]);
    let Some(centre) = at[0].centre else {
        return Ok(false);
    };
    let radius = (at[0].points[ka] - centre).norm();
    let edge = m.edge(w)?;
    let Some((curve, range)) = edge.curve() else {
        return Ok(false);
    };
    let (shared_surface, w_curve) = (m.surface(m.face(face)?.surface())?, m.curve(curve)?);
    // The far faces' outward normal at `s` along `w`, one direction within
    // the tangency the blend allows (ADR-0040).
    let tangency = at[0].tolerance.max(at[1].tolerance).max(edge.tolerance());
    let far_normal = |s: f64| -> Result<Option<UnitVec3>, OpError> {
        let mut normal: Option<UnitVec3> = None;
        for u in view
            .uses
            .get(&w)
            .ok_or(invariant("the junction's corner edge's uses"))?
        {
            let pcurve = m.curve2(u.pcurve)?;
            let n = UnitVec3::try_new(view.outward(m, u.face, pcurve.point(s))?, 0.0)
                .ok_or(invariant("a face's normal"))?;
            match normal {
                None => normal = Some(n),
                Some(first)
                    if first.dot(&n) > 0.0
                        && tangent_normals(
                            first.into_inner(),
                            n.into_inner(),
                            radius,
                            tangency,
                            tol,
                        ) => {}
                Some(_) => return Ok(None),
            }
        }
        Ok(normal)
    };
    let ball = match (shared_surface, w_curve) {
        (Surface::Plane { frame }, &Curve::Line { origin, direction }) => {
            let n_p = UnitVec3::try_new(view.outward(m, face, Point2::origin())?, 0.0)
                .ok_or(invariant("a plane's normal"))?;
            let plane = Axis {
                origin: frame.origin(),
                direction: n_p,
            };
            // A line's two ends hold its one direction.
            let (Some(n), Some(other)) = (far_normal(range.lo())?, far_normal(range.hi())?) else {
                return Ok(false);
            };
            if n.dot(&other) <= 0.0
                || !tangent_normals(n.into_inner(), other.into_inner(), radius, tangency, tol)
            {
                return Ok(false);
            }
            leaning_ball(
                plane,
                Axis { origin, direction },
                n,
                radius,
                at[0].convex,
                tol.angular,
            )
        }
        _ => {
            let Some(guess) = at[0].centre else {
                return Ok(false);
            };
            curved_ball(
                m,
                view,
                face,
                w_curve,
                range,
                guess,
                radius,
                at[0].convex,
                &far_normal,
                tol,
            )?
        }
    };
    let Some(ball) = ball else {
        return Ok(false);
    };
    for ((s, at), k) in [sa, sb].into_iter().zip(at).zip(shared) {
        let (t, centre) = match s.section {
            Section::Round { axis_origin, .. } => {
                let t = (ball.centre - axis_origin).dot(&s.d);
                (t, axis_origin + t * s.d)
            }
            Section::Walked { .. } => {
                (s.lines[k].project(ball.q).map_err(fault_of)?.t, ball.centre)
            }
            Section::Flat => return Ok(false),
        };
        let mut points = [Point3::origin(); 2];
        for (point, line) in points.iter_mut().zip(&s.lines) {
            *point = match s.section {
                Section::Walked { .. } => line.point(t),
                Section::Round { .. } | Section::Flat => line_origin(line)? + t * s.d,
            };
        }
        if (centre - ball.centre).norm() > s.tolerance
            || (points[k] - ball.q).norm() > s.tolerance
            || (points[1 - k] - ball.p).norm() > s.tolerance
        {
            return Ok(false);
        }
        at.t = t;
        at.points = points;
        at.centre = Some(centre);
    }
    Ok(true)
}

/// Slides the walked junction of `runs` along their spines to the
/// ball whose contact on the seamed face is on `seams`, the corner edges the
/// contacts missed at the vertex (ADR-0083 §2, §3): both runs are read again
/// there, their contacts, ball centres and parameters. The root is found on
/// the first run's contact on the seam's face, within its surface's reach,
/// and the second run's parameter is the foot of that contact point on its
/// own. With a seam of each face the two roots must be one ball: their
/// contacts on both faces within the stripes' tolerance.
/// `false` where the construction does not apply: a seam is no seam of a
/// periodic face the runs share, two seams are of one face, a run is no
/// walked stripe, the second run's contact does not meet the first's within
/// the stripes' tolerance, or two seams' roots are not one ball.
///
/// Errors: `BlendReason::TooLarge` naming the first run's edge and the seam,
/// where no root lies within the reach, or the root lies past either run's
/// far end (the surface runs on past it for the reach, the edge does not).
fn slide(
    m: &Model,
    view: &BodyView,
    runs: [Run<'_>; 2],
    at: [&mut RunAt<'_>; 2],
    vertex: VertexId,
    seams: &[EdgeId],
) -> Result<bool, OpError> {
    let [Run::Line(sa), Run::Line(sb)] = runs else {
        return Ok(false);
    };
    let [ra, rb] = at;
    if !matches!(sa.section, Section::Walked { .. })
        || !matches!(sb.section, Section::Walked { .. })
    {
        return Ok(false);
    }
    let tolerance = sa.tolerance.max(sb.tolerance);
    // Each seam's root on the first run's spine, and the face it is on.
    let mut roots: Vec<(f64, usize)> = Vec::with_capacity(2);
    for &seam in seams {
        let uses = view
            .uses
            .get(&seam)
            .ok_or(invariant("the junction's seam's uses"))?;
        let [first, second] = uses[..] else {
            return Ok(false);
        };
        let Some((_, range)) = m.edge(seam)?.curve() else {
            return Ok(false);
        };
        if first.face != second.face {
            return Ok(false);
        }
        let face = first.face;
        let (Some(ka), Some(_)) = (
            ra.faces.iter().position(|&f| f == face),
            rb.faces.iter().position(|&f| f == face),
        ) else {
            return Ok(false);
        };
        if roots.iter().any(|&(_, k)| k == ka) {
            return Ok(false);
        }
        // The seam is the isocurve of the periodic parameter its pcurve holds.
        let surface = m.surface(m.face(face)?.surface())?;
        let pcurve = m.curve2(first.pcurve)?;
        let (lo, hi) = (pcurve.point(range.lo()), pcurve.point(range.hi()));
        let period = surface.period();
        let moved = |axis: usize| {
            if axis == 0 {
                (hi.x - lo.x).abs()
            } else {
                (hi.y - lo.y).abs()
            }
        };
        let Some(axis) = [0, 1]
            .into_iter()
            .filter(|&axis| period[axis].is_some())
            .min_by(|&i, &j| moved(i).total_cmp(&moved(j)))
        else {
            return Ok(false);
        };
        let value = if axis == 0 { lo.x } else { lo.y };
        let on_a = |t: f64| sa.lines[ka].point(t);
        let v = sa.surface.domain()[1];
        let Some(t) = seam_ball(surface, axis, value, &on_a, ra.t, [v.lo(), v.hi()]) else {
            return Err(degenerate(
                vec![forward(sa.edge), forward(seam)],
                Reason::Blend(BlendReason::TooLarge),
            ));
        };
        roots.push((t, ka));
    }
    let Some(&(ta, ka)) = roots.first() else {
        return Ok(false);
    };
    if let Some(&(t1, _)) = roots.get(1) {
        if !one_ball(&|k, t| sa.lines[k].point(t), [ta, t1], tolerance) {
            return Ok(false);
        }
    }
    let on_a = |t: f64| sa.lines[ka].point(t);
    let too_large = || {
        degenerate(
            vec![forward(sa.edge), forward(seams[0])],
            Reason::Blend(BlendReason::TooLarge),
        )
    };
    let kb = rb
        .faces
        .iter()
        .position(|&f| f == ra.faces[ka])
        .ok_or(invariant("the junction's shared face"))?;
    let foot = sb.lines[kb].project(on_a(ta)).map_err(fault_of)?;
    let tb = foot.t;
    let domain_b = sb.surface.domain()[1];
    if tb < domain_b.lo()
        || domain_b.hi() < tb
        || past_far_end(sb, vertex, rb.t, tb)
        || past_far_end(sa, vertex, ra.t, ta)
    {
        return Err(too_large());
    }
    if foot.distance > tolerance {
        return Ok(false);
    }
    for (s, at, t) in [(sa, ra, ta), (sb, rb, tb)] {
        let points = [0, 1].map(|k| s.lines[k].point(t));
        // The ball's centre: a contact's point a radius from its face along
        // the normal, into the material for a convex blend, away from it
        // for a concave one.
        at.centre = match at.centre {
            Some(old) => {
                let radius = (at.points[0] - old).norm();
                let sign = if at.convex { -1.0 } else { 1.0 };
                let mut sum = Vec3::zeros();
                for (k, point) in points.iter().enumerate() {
                    let surface = m.surface(m.face(at.faces[k])?.surface())?;
                    let uv = surface.project(*point).map_err(fault_of)?.uv;
                    let n = UnitVec3::try_new(view.outward(m, at.faces[k], uv)?, 0.0)
                        .ok_or(invariant("a face's normal"))?;
                    sum += point.coords + sign * radius * n.into_inner();
                }
                Some(Point3::from(sum * 0.5))
            }
            None => None,
        };
        at.points = points;
        at.t = t;
    }
    Ok(true)
}

/// Whether the parameter `t` of a walked stripe's spine lies past the far
/// end of its edge from `vertex`, `near` being the parameter at `vertex`.
/// The surface runs on `REACH_RADII` past the edge's ends, so its domain
/// is no bound on a slide. A stripe that is not walked has no such reach.
fn past_far_end(s: &Stripe, vertex: VertexId, near: f64, t: f64) -> bool {
    let Section::Walked { ends, .. } = s.section else {
        return false;
    };
    let far = ends[usize::from(s.start == vertex)];
    past(near, far, t)
}

/// Whether `t` is beyond `far` as seen from `near`.
fn past(near: f64, far: f64, t: f64) -> bool {
    (t - far) * (near - far) < 0.0
}

/// Whether the balls at `ts` on a run's spine are one ball: the contacts
/// `contact(k, t)` on both faces within `tolerance`. Two seams' roots agree
/// this way, else the vertex is a `VertexBlend` (ADR-0083 §2).
fn one_ball(contact: &dyn Fn(usize, f64) -> Point3, ts: [f64; 2], tolerance: f64) -> bool {
    (0..2).all(|k| (contact(k, ts[0]) - contact(k, ts[1])).norm() <= tolerance)
}

/// The samples [`seam_ball`] takes across its span to find the contact
/// crossing the seam, and the halvings that then close the bracket. A
/// sampling density and a structural bound, not tolerances.
const SEAM_SAMPLES: usize = 64;
const SEAM_HALVINGS: usize = 64;

/// The fraction of a period that a bracket's two wrapped distances may sum
/// to for the bracket to hold a root. At the seam's antipode the wrapped
/// distance jumps by a whole period, so a bracket across it sums to about
/// half; a real root's sum is the bracket's width, far under a quarter.
const ANTIPODE_FRACTION: f64 = 0.25;

/// The parameter `t*` of the ball on a walked run's spine whose contact on
/// a seamed face lies on the seam (ADR-0083 §2). `contact` is the run's
/// contact isocurve on that face (its `lines[k]`, the surface's `v` as
/// parameter); the seam is the isocurve of `surface`'s parameter `axis`
/// (0 for `u`, 1 for `v`) at `seam`, which `axis` must make periodic. The
/// contact's signed distance to it is its `axis` parameter less `seam`,
/// wrapped into half a period either side; `t*` is its root within
/// `span = [lo, hi]`, the run's reach on either side of the vertex, nearest
/// `near`, the vertex's own parameter. Found by sampling for a change of
/// sign, then halving the bracket. A square crossing gives `near` back.
/// `None` where `axis` is no seam of `surface`, no root lies in `span`, or
/// the sign changes across the antipode of the seam where the wrapped
/// distance jumps, not at a root.
pub(super) fn seam_ball(
    surface: &Surface,
    axis: usize,
    seam: f64,
    contact: &dyn Fn(f64) -> Point3,
    near: f64,
    span: [f64; 2],
) -> Option<f64> {
    let period = surface.period().get(axis).copied().flatten()?;
    let signed = |t: f64| -> Option<f64> {
        let uv = surface.project(contact(t)).ok()?.uv;
        let d = (if axis == 0 { uv.x } else { uv.y }) - seam;
        Some(d - period * (d / period).round())
    };
    let [lo, hi] = span;
    let step = (hi - lo) / SEAM_SAMPLES as f64;
    let mut samples = Vec::with_capacity(SEAM_SAMPLES + 1);
    for i in 0..=SEAM_SAMPLES {
        let t = lo + i as f64 * step;
        samples.push((t, signed(t)?));
    }
    let mut best: Option<(f64, f64)> = None;
    for pair in samples.windows(2) {
        let [(t0, g0), (t1, g1)] = [pair[0], pair[1]];
        if g0 * g1 > 0.0 {
            continue;
        }
        let (mut a, mut b, mut ga) = (t0, t1, g0);
        let mut gb = g1;
        for _ in 0..SEAM_HALVINGS {
            let mid = 0.5 * (a + b);
            let gm = signed(mid)?;
            if gm * ga <= 0.0 {
                (b, gb) = (mid, gm);
            } else {
                (a, ga) = (mid, gm);
            }
        }
        // At the antipode the wrapped distance jumps by a period, and the
        // bracket keeps its two ends half a period apart.
        if ga.abs() + gb.abs() >= ANTIPODE_FRACTION * period {
            continue;
        }
        let t = 0.5 * (a + b);
        if best.is_none_or(|(_, far)| (t - near).abs() < far) {
            best = Some((t, (t - near).abs()));
        }
    }
    best.map(|(t, _)| t)
}

#[cfg(test)]
mod tests {
    use arris_math::{Vec3, real};

    use super::*;

    const A: f64 = 20.0;
    const R: f64 = 0.2;

    /// `blend/rounded-block-oblique-top-fillet`'s top `P`, through
    /// `(0, 1, 1.5)` tilted `A` about `x`, and its ruling at `(x, 0.5)`
    /// between the x wall and the corner cylinder, outward normal `±x`.
    fn block(x: f64, out: f64) -> (Axis, Axis, UnitVec3) {
        let a = A.to_radians();
        let plane = Axis::new(
            Point3::new(0.0, 1.0, 1.5),
            Vec3::new(0.0, -real::sin(a), real::cos(a)),
        )
        .unwrap();
        let w = Axis::z_at(Point3::new(x, 0.5, 0.0));
        let n = UnitVec3::new_normalize(Vec3::new(out, 0.0, 0.0));
        (plane, w, n)
    }

    /// The hand formula: `c*` at `R` inside the wall, under the top at
    /// `y = 0.5` by `R / cos A`.
    fn hand(x: f64, sigma: f64, out: f64) -> Point3 {
        let a = A.to_radians();
        let z = 1.5 - 0.5 * real::tan(a) + sigma * R / real::cos(a);
        Point3::new(x + sigma * R * out, 0.5, z)
    }

    #[test]
    fn leaning_ball_is_the_oracles_on_the_convex_block() {
        let (plane, w, n) = block(0.0, -1.0);
        let ball = leaning_ball(plane, w, n, R, true, 1e-12).unwrap();
        let want = hand(0.0, -1.0, -1.0);
        assert!((ball.centre - want).norm() < 1e-12, "{ball:?}");
        // Open CASCADE's junction circle there (plan step 1).
        assert!((ball.centre - Point3::new(0.2, 0.5, 1.105179328372)).norm() < 1e-11);
        let a = A.to_radians();
        let n_p = Vec3::new(0.0, -real::sin(a), real::cos(a));
        assert!((ball.q - (want + R * n_p)).norm() < 1e-12);
        assert!((ball.p - Point3::new(0.0, 0.5, want.z)).norm() < 1e-12);
    }

    #[test]
    fn leaning_ball_touches_the_three_faces() {
        for (x, out, centre_x) in [(0.0, -1.0, 0.5), (4.0, 1.0, 3.5)] {
            for convex in [true, false] {
                let (plane, w, n) = block(x, out);
                let c = leaning_ball(plane, w, n, R, convex, 1e-12).unwrap();
                // `p` on `w`'s line.
                let off = c.p - w.origin;
                let along = off.dot(&w.direction);
                assert!((off - along * w.direction.into_inner()).norm() < 1e-12);
                // At `R` from `P`, on its inner side for a convex blend.
                let sigma = if convex { -1.0 } else { 1.0 };
                let to_p = (c.centre - plane.origin).dot(&plane.direction);
                assert!((to_p - sigma * R).abs() < 1e-12, "{to_p}");
                // At `R` from the wall `W`, the plane `x = x`.
                assert!(((c.centre.x - x) * out - sigma * R).abs() < 1e-12);
                // At `R` from the corner cylinder `C`, of radius 0.5 about
                // `(centre_x, 0.5)`.
                let radial = Vec3::new(c.centre.x - centre_x, c.centre.y - 0.5, 0.0).norm();
                assert!((radial - (0.5 + sigma * R)).abs() < 1e-12, "{radial}");
                assert!(((c.centre - c.q).norm() - R).abs() < 1e-12);
                assert!((c.centre - hand(x, sigma, out)).norm() < 1e-12);
            }
        }
    }

    #[test]
    fn leaning_ball_is_none_for_an_edge_along_the_plane() {
        let plane = Axis::z_at(Point3::new(0.0, 0.0, 1.0));
        let w = Axis::new(Point3::new(0.0, 0.0, 1.0), Vec3::x()).unwrap();
        let n = UnitVec3::new_normalize(-Vec3::y());
        assert!(leaning_ball(plane, w, n, R, true, 1e-12).is_none());
    }

    #[test]
    fn leaning_ball_of_a_square_ruling_is_the_normal_planes() {
        // A horizontal top over a vertical ruling: `c*` straight under the
        // vertex `(0, 0, 1)` at `R` in from both faces.
        let plane = Axis::z_at(Point3::new(0.0, 0.0, 1.0));
        let w = Axis::z_at(Point3::origin());
        let n = UnitVec3::new_normalize(-Vec3::y());
        let c = leaning_ball(plane, w, n, R, true, 1e-12).unwrap();
        assert!((c.centre - Point3::new(0.0, R, 1.0 - R)).norm() < 1e-12);
        assert!((c.p - Point3::new(0.0, 0.0, 1.0 - R)).norm() < 1e-12);
        assert!((c.q - Point3::new(0.0, R, 1.0)).norm() < 1e-12);
    }

    /// A contact on `surface` whose `u` is `slope · (t − root)` and `v`
    /// is `lift · t`: it crosses the seam `u = 0` at `t = root`.
    fn slanted(surface: &Surface, slope: f64, root: f64, lift: f64) -> impl Fn(f64) -> Point3 + '_ {
        move |t| surface.point(slope * (t - root), lift * t)
    }

    #[test]
    fn seam_ball_of_a_square_crossing_is_the_vertex() {
        let cylinder = Surface::Cylinder {
            frame: Frame::world(),
            radius: 1.0,
        };
        // A circle of the cylinder at one height, the loop square to the seam.
        let circle = |t: f64| cylinder.point(t, 0.7);
        let t = seam_ball(&cylinder, 0, 0.0, &circle, 0.0, [-0.5, 0.5]).unwrap();
        assert!(t.abs() < 1e-10, "{t}");
    }

    #[test]
    fn seam_ball_puts_the_contact_on_the_seam() {
        let sphere = Surface::Sphere {
            frame: Frame::world(),
            radius: 2.0,
        };
        let cylinder = Surface::Cylinder {
            frame: Frame::world(),
            radius: 0.5,
        };
        for (surface, root) in [(&sphere, 0.3), (&sphere, -0.4), (&cylinder, 0.45)] {
            // Crossing the wrap of `u` between `2π` and `0`.
            let contact = slanted(surface, 0.4, root, 0.1);
            let t = seam_ball(surface, 0, 0.0, &contact, 0.0, [-1.0, 1.0]).unwrap();
            assert!((t - root).abs() < 1e-10, "{t} for {root}");
            let p = contact(t);
            assert!(p.y.abs() < 1e-10 && p.x > 0.0, "{p:?}");
        }
    }

    #[test]
    fn two_seams_roots_are_one_ball_only_within_the_tolerance() {
        let contact = |k: usize, t: f64| Point3::new(t, k as f64, 0.0);
        assert!(one_ball(&contact, [0.5, 0.5 + 1e-9], 1e-7));
        // Seams whose roots are a tenth apart on the spine disagree.
        assert!(!one_ball(&contact, [0.5, 0.6], 1e-7));
    }

    #[test]
    fn a_root_past_a_runs_far_end_is_past_it_from_either_side() {
        // The vertex at 1, the far end at 3 (up) or at -1 (down).
        assert!(!past(1.0, 3.0, 2.0));
        assert!(past(1.0, 3.0, 3.5));
        assert!(!past(1.0, 3.0, -0.5), "behind the vertex is the reach");
        assert!(!past(1.0, -1.0, 0.0));
        assert!(past(1.0, -1.0, -1.5));
        // Exactly at the far end is not past it.
        assert!(!past(1.0, 3.0, 3.0));
    }

    #[test]
    fn seam_ball_is_none_without_a_root_in_reach() {
        let cylinder = Surface::Cylinder {
            frame: Frame::world(),
            radius: 1.0,
        };
        // The crossing lies at `t = 3`, past the reach.
        let contact = slanted(&cylinder, 0.4, 3.0, 0.0);
        assert!(seam_ball(&cylinder, 0, 0.0, &contact, 0.0, [-1.0, 1.0]).is_none());
        // `v` of a cylinder is no seam.
        assert!(seam_ball(&cylinder, 1, 0.0, &contact, 0.0, [-1.0, 4.0]).is_none());
        // Half a turn away the wrapped distance jumps, and is no root.
        let far = |t: f64| cylinder.point(core::f64::consts::PI + 0.01 * t, 0.0);
        assert!(seam_ball(&cylinder, 0, 0.0, &far, 0.0, [-1.0, 1.0]).is_none());
    }
}
