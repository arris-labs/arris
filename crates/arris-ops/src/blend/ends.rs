//! The ends of a stripe: trimmed by the face across the corner, by a fan of faces, at a corner or a cut.

use std::collections::BTreeSet;

use arris_geom::region2::Side;
use arris_geom::{
    Curve, Curve2, CurveIntersection, CurveSurfaceIntersection, GeomKind, MeetKind, Surface,
    SurfaceIntersection, SurfaceKind, intersect_curve_surface, intersect_curves,
    intersect_surfaces, pcurve_on,
};
use arris_math::{Aabb, Interval, Meter, Point2, Point3, Tolerance, Vec3, shift_into_range};
use arris_topo::{EdgeId, FaceId, Model, Orientation, VertexId};

use super::chain::{cusp_at, parameter_at};
use super::corner::Corner;
use super::miter::Miter;
use super::mixed;
use super::ring::placed_uv;
use super::stripe::{
    Section, Stripe, arc_between, band_u, chord, in_band, line_origin, lines_cross,
    on_side_of_face, section_between,
};
use super::{degenerate, invariant};
use crate::body_view::{BodyView, UseAt};
use crate::error::{OpError, Reason, fault_of};
use crate::rebuild::forward;

/// A corner edge cut short by a blend's end: where on its curve, and at
/// which of its ends.
#[derive(Debug, Clone, Copy)]
pub(super) struct Trim {
    pub(super) edge: EdgeId,
    pub(super) t: f64,
    /// `true` when the cut is at the edge's `range.lo()` end.
    pub(super) cuts_lo: bool,
}

/// The arc where the blend meets the face across a corner: a circle when
/// that plane is perpendicular to the edge, an ellipse when oblique.
pub(super) struct Arc {
    pub(super) curve: Curve,
    pub(super) range: Interval,
    /// `true` when `range.lo()` is at the arc's start in the end's order
    /// from the contact at `u = 0` to the other: that contact's trim point
    /// for the first piece across, a crossing for a later one.
    pub(super) lo_first: bool,
    /// Its pcurve on the face across, exact.
    pub(super) on_face: Curve2,
    /// Its pcurve on the blend, placed in the blend loop's translate.
    pub(super) on_blend: Curve2,
    pub(super) tolerance: f64,
}

/// One face across an end and the arc the blend cuts it in.
pub(super) struct Piece {
    pub(super) face: FaceId,
    pub(super) arc: Arc,
}

/// Where a fan's end crosses an extra edge between two pieces (ADR-0043
/// §3): the point, the extra edge cut there, and the vertex's tolerance.
pub(super) struct Crossing {
    pub(super) point: Point3,
    pub(super) trim: Trim,
    pub(super) tolerance: f64,
}

/// One end of a blend: the corner vertex it consumes, the faces across
/// it, the two trim points on the contact lines and the corner edges
/// they shorten, and the arcs between them — one, or one per piece of a
/// fan with a crossing between each two (ADR-0043).
pub(super) struct End {
    pub(super) vertex: VertexId,
    /// The trim points, one per contact (`u = 0` first).
    pub(super) points: [Point3; 2],
    /// The contact lines' parameters there.
    pub(super) t: [f64; 2],
    /// The corner edge each contact's trim point cuts.
    pub(super) trims: [Trim; 2],
    /// From the contact at `u = 0`'s corner edge to the other's.
    pub(super) pieces: Vec<Piece>,
    /// `crossings[i]` between `pieces[i]` and `pieces[i + 1]`.
    pub(super) crossings: Vec<Crossing>,
    /// The tolerance of each trim vertex.
    pub(super) vertex_tolerance: [f64; 2],
    /// The edges that stay at the vertex, which survives with them, where
    /// the face across is met twice (ADR-0043 §4); none otherwise.
    pub(super) stays: Vec<EdgeId>,
}

/// One end of a blend: trimmed by the face across the corner, met by the
/// other blend of a miter, or met by the corner face of three blends.
pub(super) enum EndKind {
    Face(Box<End>),
    /// The miter at `miters[at]`, this stripe being its `side`.
    Miter {
        at: usize,
        side: usize,
    },
    /// The corner at `corners[at]`, this stripe being its `side`.
    Corner {
        at: usize,
        side: usize,
    },
}

impl EndKind {
    /// The contact lines' parameters at this end, by contact.
    pub(super) fn t(&self, miters: &[Miter], corners: &[Corner], k: usize) -> f64 {
        match self {
            EndKind::Face(end) => end.t[k],
            EndKind::Miter { at, side } => miters[*at].t[*side][k],
            EndKind::Corner { at, side } => corners[*at].t[*side][k],
        }
    }

    /// The tolerance a face end's trim vertex carries, `None` at a miter
    /// or a corner.
    pub(super) fn vertex_tolerance(&self, k: usize) -> Option<f64> {
        match self {
            EndKind::Face(end) => Some(end.vertex_tolerance[k]),
            EndKind::Miter { .. } | EndKind::Corner { .. } => None,
        }
    }
}

/// An end's corner as [`corner_of`] reads it.
pub(super) struct CornerAt {
    /// The corner edge of each of the blended edge's faces, by its use.
    pub(super) edges: [EdgeId; 2],
    /// The faces across, in the vertex's star from `edges[0]`'s to
    /// `edges[1]`'s: the one face across, or a fan's pieces (ADR-0043).
    pub(super) pieces: Vec<FaceId>,
    /// A fan's extra edges, `extras[i]` between `pieces[i]` and
    /// `pieces[i + 1]`; none for one face across.
    pub(super) extras: Vec<EdgeId>,
    /// The edges that stay at the vertex where the one face across is met
    /// twice, the end not reaching them (ADR-0043 §4); none otherwise.
    pub(super) stays: Vec<EdgeId>,
    /// The index of the corner edge that is a cusp's spine where the end is
    /// one (ADR-0042).
    pub(super) spine: Option<usize>,
}

/// A lone face across, as a ring's end reads its corner: the corner edges,
/// the face and the spine. A fan is `Reason::VertexBlend` naming `edge` and
/// `vertex`: only a stripe's face end fans (ADR-0043 §6).
pub(super) type LoneCorner = ([EdgeId; 2], FaceId, Option<usize>);

impl CornerAt {
    pub(super) fn lone(self, edge: EdgeId, vertex: VertexId) -> Result<LoneCorner, OpError> {
        match self.pieces[..] {
            [face] if self.stays.is_empty() => Ok((self.edges, face, self.spine)),
            _ => Err(degenerate(
                vec![forward(edge), forward(vertex)],
                Reason::VertexBlend,
            )),
        }
    }
}

/// The one edge beside `edge` in `face`'s loops that also has `vertex` as
/// an end, other than `edge` itself; `None` where there is none or more
/// than one.
pub(super) fn beside_at(
    m: &Model,
    face: FaceId,
    edge: EdgeId,
    vertex: VertexId,
) -> Result<Option<EdgeId>, OpError> {
    let mut found = None;
    for l in m.face(face)?.loops() {
        let n = l.coedges().len();
        for i in (0..n).filter(|&i| l.coedges()[i].edge() == edge) {
            for j in [(i + n - 1) % n, (i + 1) % n] {
                let c = l.coedges()[j].edge();
                let entity = m.edge(c)?;
                if c == edge || (entity.start() != vertex && entity.end() != vertex) {
                    continue;
                }
                if found.is_some_and(|x| x != c) {
                    return Ok(None);
                }
                found = Some(c);
            }
        }
    }
    Ok(found)
}

/// A fan's pieces across and the extra edges between them, in the walk's
/// order (`CornerAt`).
pub(super) type Fan = (Vec<FaceId>, Vec<EdgeId>);

/// The fan at `vertex` (ADR-0043 §1, §2): the pieces across and the extra
/// edges between them, walked through the vertex's star from `corners[0]`
/// in its face other than `faces[0]` to `corners[1]` in its face other than
/// `faces[1]`, on the side away from the blended edge. Every edge at the
/// vertex but the blended one is sharp at it for a blend of `size`, with a
/// curve, between two faces, and of the blend's sense — `convex` — and
/// every extra is met once by the walk, no face twice. `None` at any other
/// star: the face across met twice among them (`twice_at`).
#[allow(clippy::too_many_arguments)]
pub(super) fn fan_at(
    m: &Model,
    view: &BodyView,
    faces: [FaceId; 2],
    corners: [EdgeId; 2],
    extras_at: &BTreeSet<EdgeId>,
    vertex: VertexId,
    size: f64,
    convex: Option<bool>,
    tol: Tolerance,
) -> Result<Option<Fan>, OpError> {
    let two_faces = |x: EdgeId| -> Option<[FaceId; 2]> {
        match view.uses.get(&x).map(Vec::as_slice) {
            Some(&[a, b]) if a.face != b.face => Some([a.face, b.face]),
            _ => None,
        }
    };
    for &x in corners.iter().chain(extras_at) {
        let Some(t) = parameter_at(m, x, vertex)? else {
            return Ok(None);
        };
        if two_faces(x).is_none()
            || view.tangent_at(m, x, t, size, tol)? != Some(false)
            || view.convex(m, x)? != convex
        {
            return Ok(None);
        }
    }
    let other =
        |x: EdgeId, own: FaceId| two_faces(x).and_then(|f| f.into_iter().find(|&g| g != own));
    let (Some(first), Some(last)) = (other(corners[0], faces[0]), other(corners[1], faces[1]))
    else {
        return Ok(None);
    };
    if first == last || faces.contains(&first) || faces.contains(&last) {
        return Ok(None);
    }
    let (mut pieces, mut extras) = (vec![first], Vec::new());
    let mut at = corners[0];
    for _ in 0..=extras_at.len() {
        let face = pieces[pieces.len() - 1];
        let Some(next) = beside_at(m, face, at, vertex)? else {
            return Ok(None);
        };
        if next == corners[1] {
            let whole = face == last && extras.len() == extras_at.len();
            return Ok(whole.then_some((pieces, extras)));
        }
        if !extras_at.contains(&next) || extras.contains(&next) {
            return Ok(None);
        }
        let Some(beyond) = other(next, face) else {
            return Ok(None);
        };
        if pieces.contains(&beyond) || faces.contains(&beyond) {
            return Ok(None);
        }
        extras.push(next);
        pieces.push(beyond);
        at = next;
    }
    Ok(None)
}

/// The face across met twice at `vertex` (ADR-0043 §1, §4): both corner
/// edges lead to one face `A` other than the blended edge's, and the two
/// extra edges at the vertex, each sharp with a curve between two faces,
/// are `A`'s on one side of it and a third face's on the other. Walked
/// through the star from `corners[0]` in `A`: an extra edge, the third
/// face across it, the other extra edge, `A` again, `corners[1]`. `None` at
/// any other star.
#[allow(clippy::too_many_arguments)]
pub(super) fn twice_at(
    m: &Model,
    view: &BodyView,
    faces: [FaceId; 2],
    corners: [EdgeId; 2],
    extras_at: &BTreeSet<EdgeId>,
    vertex: VertexId,
    size: f64,
    tol: Tolerance,
) -> Result<Option<FaceId>, OpError> {
    let two_faces = |x: EdgeId| -> Option<[FaceId; 2]> {
        match view.uses.get(&x).map(Vec::as_slice) {
            Some(&[a, b]) if a.face != b.face => Some([a.face, b.face]),
            _ => None,
        }
    };
    let other =
        |x: EdgeId, own: FaceId| two_faces(x).and_then(|f| f.into_iter().find(|&g| g != own));
    let (Some(across), Some(last)) = (other(corners[0], faces[0]), other(corners[1], faces[1]))
    else {
        return Ok(None);
    };
    if across != last || faces.contains(&across) || extras_at.len() != 2 {
        return Ok(None);
    }
    for &x in corners.iter().chain(extras_at) {
        let Some(t) = parameter_at(m, x, vertex)? else {
            return Ok(None);
        };
        if two_faces(x).is_none() || view.tangent_at(m, x, t, size, tol)? != Some(false) {
            return Ok(None);
        }
    }
    let Some(first) = beside_at(m, across, corners[0], vertex)? else {
        return Ok(None);
    };
    let Some(third) = other(first, across) else {
        return Ok(None);
    };
    if !extras_at.contains(&first) || faces.contains(&third) {
        return Ok(None);
    }
    let Some(second) = beside_at(m, third, first, vertex)? else {
        return Ok(None);
    };
    if second == first
        || !extras_at.contains(&second)
        || other(second, third) != Some(across)
        || beside_at(m, across, second, vertex)? != Some(corners[1])
    {
        return Ok(None);
    }
    Ok(Some(across))
}

/// The corner at `vertex`, the end of the blended `edge` at its start
/// (`at_lo`) or its end, `uses` the edge's uses by its two faces: the
/// corner edge each of those faces' loops runs on to there, in the same
/// order, and the face across, the one face the two corner edges share
/// beyond the edge's own. At a vertex of more edges than these three, the
/// faces across are a fan's pieces where `fan_at` walks one (ADR-0043). Any
/// other vertex of other than these three edges, or
/// corner edges that share no such face, is `Reason::VertexBlend`. A corner
/// edge whose two faces meet tangentially at the vertex is the spine of a
/// cusp whose two edges are of one sense (`cusp_at`, ADR-0042 §1), and its
/// index is returned with the corner: the face across is then the next
/// wall, which cuts the stripe. At any other vertex the chain did not run
/// on through — an overhang tip, the next edge turning back with no cusp,
/// itself a tangent dihedral — it is `Reason::TangentChain` naming the
/// edge, that corner edge and the vertex (ADR-0035 §6).
#[allow(clippy::too_many_arguments)]
pub(super) fn corner_of(
    m: &Model,
    view: &BodyView,
    edge: EdgeId,
    uses: &[UseAt; 2],
    vertex: VertexId,
    at_lo: bool,
    size: f64,
    tol: Tolerance,
) -> Result<CornerAt, OpError> {
    let e = forward(edge);
    let v = forward(vertex);
    let vertex_blend = || degenerate(vec![e, v], Reason::VertexBlend);
    // The neighbour of the blended edge's coedge at this end in each
    // face's loop.
    let mut corner_edges = [edge; 2];
    for (k, u) in uses.iter().enumerate() {
        let l = &m.face(u.face)?.loops()[u.loop_index];
        let n = l.coedges().len();
        let before = (u.orientation == Orientation::Forward) == at_lo;
        let j = if before {
            (u.coedge_index + n - 1) % n
        } else {
            (u.coedge_index + 1) % n
        };
        corner_edges[k] = l.coedges()[j].edge();
    }
    let at_vertex = view
        .vertex_edges
        .get(&vertex)
        .ok_or(invariant("the corner vertex's edges"))?;
    let three: BTreeSet<EdgeId> = [edge, corner_edges[0], corner_edges[1]]
        .into_iter()
        .collect();
    if three.len() != 3 || !three.is_subset(at_vertex) {
        return Err(vertex_blend());
    }
    let faces = uses.map(|u| u.face);
    if at_vertex.len() > 3 {
        let extras_at: BTreeSet<EdgeId> = at_vertex.difference(&three).copied().collect();
        let convex = view.convex(m, edge)?;
        let fan = fan_at(
            m,
            view,
            faces,
            corner_edges,
            &extras_at,
            vertex,
            size,
            convex,
            tol,
        )?;
        if let Some((pieces, extras)) = fan {
            return Ok(CornerAt {
                edges: corner_edges,
                pieces,
                extras,
                stays: Vec::new(),
                spine: None,
            });
        }
        let across = twice_at(m, view, faces, corner_edges, &extras_at, vertex, size, tol)?
            .ok_or_else(vertex_blend)?;
        return Ok(CornerAt {
            edges: corner_edges,
            pieces: vec![across],
            extras: Vec::new(),
            stays: extras_at.into_iter().collect(),
            spine: None,
        });
    }
    let mut spine = None;
    for (k, &corner) in corner_edges.iter().enumerate() {
        let Some(t) = parameter_at(m, corner, vertex)? else {
            return Err(vertex_blend());
        };
        if !view
            .tangent_at(m, corner, t, size, tol)?
            .ok_or(invariant("two uses of the corner edge"))?
        {
            continue;
        }
        // A cusp whose edges are of one sense is cut by the next wall
        // (ADR-0042 §1); an overhang tip, of opposite senses, is not.
        let next = corner_edges[1 - k];
        let one_sense =
            view.convex(m, next)?.is_some() && view.convex(m, next)? == view.convex(m, edge)?;
        if spine.is_some()
            || !one_sense
            || cusp_at(m, view, edge, vertex, size, tol)? != Some((next, corner))
        {
            return Err(degenerate(
                vec![e, forward(corner), v],
                Reason::TangentChain,
            ));
        }
        spine = Some(k);
    }
    let other_face = |corner: EdgeId, own: FaceId| -> Result<FaceId, OpError> {
        let corner_uses = view
            .uses
            .get(&corner)
            .ok_or(invariant("the corner edge's uses"))?;
        let others: Vec<FaceId> = corner_uses
            .iter()
            .map(|u| u.face)
            .filter(|&f| f != own)
            .collect();
        match others.as_slice() {
            [f] if !faces.contains(f) => Ok(*f),
            _ => Err(vertex_blend()),
        }
    };
    let across = other_face(corner_edges[0], faces[0])?;
    if other_face(corner_edges[1], faces[1])? != across {
        return Err(vertex_blend());
    }
    Ok(CornerAt {
        edges: corner_edges,
        pieces: vec![across],
        extras: Vec::new(),
        stays: Vec::new(),
        spine,
    })
}

/// The corner edge `corner` cut at `point`, its end at `vertex` moving
/// there: its parameter and which end it cuts, checked to leave the edge a
/// positive length clear of its far vertex — `Reason::BlendTooLarge`
/// naming the blended `edge` and the corner edge otherwise.
pub(super) fn cut_corner(
    m: &Model,
    edge: EdgeId,
    corner: EdgeId,
    vertex: VertexId,
    point: Point3,
) -> Result<Trim, OpError> {
    let ce = *m.edge(corner)?;
    let Some((cid, crange)) = ce.curve() else {
        return Err(degenerate(
            vec![forward(edge), forward(vertex)],
            Reason::VertexBlend,
        ));
    };
    let ccurve = m.curve(cid)?;
    let projection = ccurve.project(point).map_err(fault_of)?;
    if projection.distance > ce.tolerance() {
        return Err(invariant("the corner edge through the trim point"));
    }
    let too_large = || degenerate(vec![forward(edge), forward(corner)], Reason::BlendTooLarge);
    let Some(tc) = shift_into_range(crange, projection.t, ccurve.period()) else {
        return Err(too_large());
    };
    let cuts_lo = ce.start() == vertex;
    let far = if cuts_lo { ce.end() } else { ce.start() };
    let far_vertex = m.vertex(far)?;
    let kept_length = if cuts_lo {
        crange.hi() - tc
    } else {
        tc - crange.lo()
    };
    if kept_length <= 0.0 || (point - far_vertex.point()).norm() <= far_vertex.tolerance() {
        return Err(too_large());
    }
    Ok(Trim {
        edge: corner,
        t: tc,
        cuts_lo,
    })
}

/// A blend's end at a cusp (ADR-0042 §2, §5): the next edge cut at `P`
/// and the spine at `Q`, by contact as `corner_edges` are, each within its
/// edge (`cut_corner`, `Reason::BlendTooLarge` otherwise), and the cut
/// inside the next wall, whose corner the stripe takes whatever its sense.
pub(super) fn cusp_trims(
    m: &Model,
    edge: EdgeId,
    corner_edges: [EdgeId; 2],
    vertex: VertexId,
    points: [Point3; 2],
) -> Result<mixed::CornerTrims, OpError> {
    let mut trims = [Trim {
        edge,
        t: 0.0,
        cuts_lo: true,
    }; 2];
    for c in 0..2 {
        trims[c] = cut_corner(m, edge, corner_edges[c], vertex, points[c])?;
    }
    Ok(mixed::CornerTrims {
        trims,
        lengthened: None,
        side: Side::Inside,
    })
}

/// The point of the face across's own `(u, v)` at `t` on the corner edge
/// `corner`, which a contact cuts there: the translate an end arc's pcurve
/// on a curved face across is placed in.
pub(super) fn at_cut_corner(
    m: &Model,
    view: &BodyView,
    across: FaceId,
    corner: EdgeId,
    t: f64,
) -> Result<Point2, OpError> {
    let corner_use = view
        .uses
        .get(&corner)
        .and_then(|u| u.iter().find(|u| u.face == across))
        .ok_or(invariant("the corner edge on the face across"))?;
    Ok(m.curve2(corner_use.pcurve)?.point(t))
}

/// The end arc's parameter at contact `c`'s trim point: `range.lo()` is at
/// the first contact's when `lo_first`.
pub(super) fn arc_end(range: Interval, lo_first: bool, c: usize) -> f64 {
    if lo_first == (c == 0) {
        range.lo()
    } else {
        range.hi()
    }
}

/// Where the line through `q` along the unit `d` pierces a cylinder or a
/// cone, the root nearest `near` along it: the line's parameter. `None`
/// where it misses the surface, grazes it — two roots within `tolerance`
/// of each other — or meets the cone's other nappe, and for any other
/// surface.
pub(super) fn pierce(
    q: Point3,
    d: Vec3,
    surface: &Surface,
    near: f64,
    tolerance: f64,
) -> Option<f64> {
    // `A t² + B t + C = 0` in the surface's own frame.
    let (a, b, c, cone) = match *surface {
        Surface::Cylinder { frame, radius } => {
            let (p, v) = (frame.to_local(q), frame.vec_to_local(d));
            (
                v.x * v.x + v.y * v.y,
                2.0 * (p.x * v.x + p.y * v.y),
                p.x * p.x + p.y * p.y - radius * radius,
                None,
            )
        }
        Surface::Cone {
            frame,
            radius,
            half_angle,
        } => {
            let (p, v) = (frame.to_local(q), frame.vec_to_local(d));
            let k = half_angle.tan();
            let at = radius + k * p.z;
            (
                v.x * v.x + v.y * v.y - k * k * v.z * v.z,
                2.0 * (p.x * v.x + p.y * v.y - k * v.z * at),
                p.x * p.x + p.y * p.y - at * at,
                Some((frame, radius, k)),
            )
        }
        Surface::Plane { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => return None,
    };
    let roots: Vec<f64> = if a.abs() <= f64::EPSILON * (b.abs() + c.abs()) {
        // A line along a cone's ruling direction meets it once.
        if b == 0.0 {
            return None;
        }
        vec![-c / b]
    } else {
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return None;
        }
        let root = disc.sqrt();
        // The two roots without cancellation.
        let big = -(b + b.signum() * root) / 2.0;
        let pair = if big == 0.0 {
            [0.0, 0.0]
        } else {
            [big / a, c / big]
        };
        if (pair[0] - pair[1]).abs() <= tolerance {
            return None;
        }
        pair.to_vec()
    };
    let t = roots
        .into_iter()
        .min_by(|x, y| (x - near).abs().total_cmp(&(y - near).abs()))?;
    // On the cone's own nappe, the one its frame's radius widens along.
    if let Some((frame, radius, k)) = cone
        && radius + k * frame.to_local(q + t * d).z < 0.0
    {
        return None;
    }
    Some(t)
}

/// The stretch of one of `curves` from `points[0]` to `points[1]` whose
/// midpoint `band` takes: each point within `tolerance` of the curve, one
/// stretch on an open curve and either way round a closed one. `None`
/// unless exactly one stretch qualifies.
pub(super) fn stretch_between(
    curves: &[Curve],
    points: [Point3; 2],
    band: &dyn Fn(Point3) -> Result<bool, OpError>,
    tolerance: f64,
) -> Result<Option<(Curve, Interval, bool)>, OpError> {
    let mut found = None;
    for curve in curves {
        let (p0, p1) = (
            curve.project(points[0]).map_err(fault_of)?,
            curve.project(points[1]).map_err(fault_of)?,
        );
        if p0.distance > tolerance || p1.distance > tolerance {
            continue;
        }
        let (t0, t1) = (p0.t, p1.t);
        let stretches = match curve.period() {
            None => vec![(t0.min(t1), t0.max(t1), t0 <= t1)],
            Some(p) => {
                let up = |t: f64, from: f64| from + (t - from).rem_euclid(p);
                vec![(t0, up(t1, t0), true), (t1, up(t0, t1), false)]
            }
        };
        for (lo, hi, lo_first) in stretches {
            let Ok(range) = Interval::new(lo, hi) else {
                continue;
            };
            if hi - lo <= 0.0 || !band(curve.point(range.midpoint()))? {
                continue;
            }
            if found.replace((curve.clone(), range, lo_first)).is_some() {
                return Ok(None);
            }
        }
    }
    Ok(found)
}

/// The end of `s` at its start (`at_lo`) or its end vertex, trimmed by
/// the face across the corner: the vertex's other two edges, the face
/// they share, where each contact pierces it, the corner edges cut or, at
/// a mixed corner, one lengthened there (`mixed::corner_trims`, ADR-0038)
/// and the arc between — every one checked against the body. A
/// plane across cuts the stripe in a conic, exact (ADR-0007); a cylinder
/// or a cone across cuts a fillet's cylinder in a quartic, traced and
/// fitted between the trim points (ADR-0037), and a chamfer's plane in a
/// conic the intersector writes exactly. At a cusp the face across is the
/// next wall, tangent to the edge's own along the spine: the contact on
/// that wall is trimmed where it crosses the spine and the other where it
/// pierces the next wall on the edge's side, and both corner edges are
/// cut (`cusp_trims`, ADR-0042). A pierce or a section that does not
/// decide the end is `Unsupported` naming the blend and the face across.
pub(super) fn face_end(
    m: &Model,
    view: &BodyView,
    s: &Stripe,
    at_lo: bool,
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<End, OpError> {
    meter.tick()?;
    let (edge, d) = (s.edge, s.d);
    let e = forward(edge);
    let vertex = if at_lo { s.start } else { s.end };
    let vertex_blend = || degenerate(vec![e, forward(vertex)], Reason::VertexBlend);
    let mut corner = corner_of(m, view, edge, &s.uses, vertex, at_lo, s.size, tol)?;
    if corner.pieces.len() > 1 {
        return fan_end(m, view, s, vertex, &corner, tol, samples, meter);
    }
    let stays = std::mem::take(&mut corner.stays);
    let (corner_edges, face3, spine) = corner.lone(edge, vertex)?;
    // At a cusp (ADR-0042): the spine, which the contact on the wall
    // tangent to the face across meets at `Q`, a line where a line edge's
    // walls are a plane and a cylinder tangent along it.
    let spine_line = match spine {
        None => None,
        Some(k) => {
            let tangent_chain = || {
                degenerate(
                    vec![e, forward(corner_edges[k]), forward(vertex)],
                    Reason::TangentChain,
                )
            };
            let (curve, _) = m
                .edge(corner_edges[k])?
                .curve()
                .ok_or(invariant("the cusp's spine's curve"))?;
            match *m.curve(curve)? {
                Curve::Line { origin, direction } => Some((k, origin, direction.into_inner())),
                Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => {
                    return Err(tangent_chain());
                }
            }
        }
    };
    let surface3 = m.surface(m.face(face3)?.surface())?;
    let face3_tolerance = m.face(face3)?.tolerance();
    let arc_tolerance = s.tolerance.max(face3_tolerance);
    let arc_tol = Tolerance::new(arc_tolerance, tol.angular);
    let refuse = || OpError::Unsupported {
        a: (GeomKind::Surface(s.surface.kind()), e),
        b: (GeomKind::Surface(surface3.kind()), forward(face3)),
    };
    // Where each contact line pierces the face across, and the arc
    // between: a chamfer's segment joining them, or a fillet's section on
    // the blend's side of its axis; how far each trim point is from the
    // arc's end when the arc is fitted.
    let mut points = [Point3::origin(); 2];
    let mut t = [0.0; 2];
    let mut gaps = [0.0; 2];
    let (arc_curve, arc_range, lo_first) = match *surface3 {
        Surface::Plane { frame: plane3 } => {
            // A plane tangent to the edge's wall along the spine is that
            // wall's own plane, which no cusp has.
            if let Some((k, ..)) = spine_line {
                return Err(degenerate(
                    vec![e, forward(corner_edges[k]), forward(vertex)],
                    Reason::TangentChain,
                ));
            }
            let n3: Vec3 = plane3.z().into_inner();
            let dn = d.dot(&n3);
            if dn.abs() <= tol.angular {
                return Err(vertex_blend());
            }
            for k in 0..2 {
                let q = line_origin(&s.lines[k])?;
                t[k] = (plane3.origin() - q).dot(&n3) / dn;
                points[k] = q + t[k] * d;
            }
            match s.section {
                Section::Flat => {
                    let (curve, range) = chord(points[0], points[1], tol)?;
                    (curve, range, true)
                }
                Section::Round {
                    axis_origin,
                    radius,
                } => {
                    // The arc joins the two trim points round the blend's
                    // axis, so it lies within a diameter of either; the
                    // plane's closed form ignores the region anyway.
                    let within = Aabb::of_point(points[0])
                        .union(Aabb::of_point(points[1]))
                        .inflated(2.0 * radius);
                    let cut = intersect_surfaces(&s.surface, surface3, &within, tol, meter)
                        .map_err(fault_of)?;
                    let arc_curve = match cut {
                        SurfaceIntersection::Meets { mut curves, points }
                            if points.is_empty()
                                && curves.len() == 1
                                && curves[0].kind == MeetKind::Crossing =>
                        {
                            curves.swap_remove(0).curve
                        }
                        SurfaceIntersection::Meets { .. }
                        | SurfaceIntersection::Empty
                        | SurfaceIntersection::Coincident => {
                            return Err(invariant("a transversal section of the blend at its end"));
                        }
                    };
                    let (arc_range, lo_first) = match &arc_curve {
                        Curve::Circle { .. } => (
                            Interval::new(0.0, s.u1)
                                .map_err(|_| invariant("a blend turning by a positive angle"))?,
                            true,
                        ),
                        Curve::Ellipse { .. } => {
                            // The point of the arc half way round the blend.
                            let half = s.u1 / 2.0;
                            let ruling = axis_origin
                                + radius
                                    * (half.cos() * s.frame.x().into_inner()
                                        + half.sin() * s.frame.y().into_inner());
                            let mid = ruling + ((plane3.origin() - ruling).dot(&n3) / dn) * d;
                            let param = |p: Point3| -> Result<f64, OpError> {
                                arc_curve.project(p).map(|q| q.t).map_err(fault_of)
                            };
                            arc_between(param(points[0])?, param(points[1])?, param(mid)?)?
                        }
                        Curve::Line { .. } | Curve::Nurbs(_) => {
                            return Err(invariant("a conic section of the blend at its end"));
                        }
                    };
                    (arc_curve, arc_range, lo_first)
                }
            }
        }
        Surface::Cylinder { .. } | Surface::Cone { .. } => {
            // The edge's own line pierces the face at the vertex; each
            // contact, beside it, at the root nearest there. At a cusp the
            // edge's line touches the face across at the vertex, so the
            // root is the one nearest the edge's midpoint, on its side,
            // and the contact on the wall tangent to the face meets the
            // spine instead, at `Q` (ADR-0042 §2).
            let at = match spine_line {
                None => m.vertex(vertex)?.point(),
                Some(_) => {
                    let (curve, range) = m
                        .edge(edge)?
                        .curve()
                        .ok_or(invariant("the blended edge's curve"))?;
                    m.curve(curve)?.point(range.midpoint())
                }
            };
            for k in 0..2 {
                let q = line_origin(&s.lines[k])?;
                t[k] = match spine_line {
                    Some((j, origin, along)) if j == k => {
                        let (tq, ts) = lines_cross(q, d, origin, along, tol).ok_or_else(|| {
                            degenerate(
                                vec![e, forward(corner_edges[k]), forward(vertex)],
                                Reason::TangentChain,
                            )
                        })?;
                        if (q + tq * d - (origin + ts * along)).norm() > arc_tolerance {
                            return Err(degenerate(
                                vec![e, forward(corner_edges[k]), forward(vertex)],
                                Reason::TangentChain,
                            ));
                        }
                        tq
                    }
                    Some(_) | None => pierce(q, d, surface3, (at - q).dot(&d), arc_tolerance)
                        .ok_or_else(refuse)?,
                };
                points[k] = q + t[k] * d;
            }
            let section = section_between(s, surface3, points, &refuse, arc_tol, meter)?;
            gaps = section.gaps;
            (section.curve, section.range, section.lo_first)
        }
        Surface::EllipticCylinder { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => {
            return Err(OpError::Unsupported {
                a: (GeomKind::Surface(SurfaceKind::Cylinder), e),
                b: (GeomKind::Surface(surface3.kind()), forward(face3)),
            });
        }
    };
    // The corner edges cut or lengthened to the trim points, and the side
    // of the face across the arc lies on (ADR-0038).
    let corner = match spine {
        Some(_) => cusp_trims(m, edge, corner_edges, vertex, points)?,
        None => mixed::corner_trims(
            m,
            view,
            edge,
            s.faces,
            corner_edges,
            vertex,
            points,
            s.convex,
            tol,
            samples,
            meter,
        )?,
    };
    let trims = corner.trims;
    let mut vertex_tolerance = [0.0; 2];
    for k in 0..2 {
        vertex_tolerance[k] = s
            .tolerance
            .max(m.edge(corner_edges[k])?.tolerance())
            .max(face3_tolerance)
            .max(gaps[k]);
    }
    let on_face = pcurve_on(&arc_curve, arc_range, surface3, arc_tol, meter).map_err(fault_of)?;
    // On a curved face across, in its loop's translate: at the corner edge
    // a contact cuts, never one lengthened past the range its pcurve was
    // stored for (ADR-0038 §4).
    let on_face = match surface3 {
        Surface::Plane { .. } => on_face,
        Surface::Cylinder { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Cone { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => {
            let c = usize::from(corner.lengthened == Some(0));
            let at = at_cut_corner(m, view, face3, corner_edges[c], trims[c].t)?;
            placed_uv(on_face, arc_end(arc_range, lo_first, c), at)
        }
    };
    // The edges that stay at the vertex are not reached: an arc that
    // crosses one within its range has run past the face (ADR-0043 §4).
    for &x in &stays {
        let (curve_id, range) = m
            .edge(x)?
            .curve()
            .ok_or(invariant("a staying edge's curve"))?;
        let curve = m.curve(curve_id)?;
        let hits = match intersect_curves(&arc_curve, curve, arc_tol, meter).map_err(fault_of)? {
            CurveIntersection::Points(hits) => hits,
            CurveIntersection::Coincident => {
                return Err(degenerate(vec![e, forward(x)], Reason::BlendTooLarge));
            }
        };
        for hit in &hits {
            if shift_into_range(range, hit.tb, curve.period()).is_some()
                && shift_into_range(arc_range, hit.ta, arc_curve.period()).is_some()
            {
                return Err(degenerate(vec![e, forward(x)], Reason::BlendTooLarge));
            }
        }
    }
    if !on_side_of_face(m, face3, &on_face, arc_range, corner.side, samples)? {
        return Err(degenerate(vec![e, forward(face3)], Reason::BlendTooLarge));
    }
    let on_blend =
        pcurve_on(&arc_curve, arc_range, &s.surface, arc_tol, meter).map_err(fault_of)?;
    let on_blend = s.place(on_blend, arc_range.lo(), if lo_first { 0.0 } else { s.u1 });
    Ok(End {
        vertex,
        points,
        t,
        trims,
        pieces: vec![Piece {
            face: face3,
            arc: Arc {
                curve: arc_curve,
                range: arc_range,
                lo_first,
                on_face,
                on_blend,
                tolerance: arc_tolerance,
            },
        }],
        crossings: Vec::new(),
        vertex_tolerance,
        stays,
    })
}

/// The end of `s` at `vertex` across a fan (ADR-0043 §3): each contact
/// trimmed where it pierces its corner edge's piece, as against a lone face
/// across, each extra edge cut where the blend's surface crosses it on the
/// band, and each piece cut in the stripe's section between its two
/// points (`section_between`), inside the piece — every edge at the vertex
/// is of the blend's sense (`fan_at`), so the blend takes the corner from
/// each. A contact parallel to a plane piece is `Reason::VertexBlend`; a
/// piece the contact misses or whose section is not decided is
/// `Unsupported` naming the blend and the piece; a crossing not on its
/// extra edge — none on the band within its range, more than one, or at
/// its far vertex — a corner edge shorter than its trim, and an arc
/// leaving its piece are `Reason::BlendTooLarge`.
#[allow(clippy::too_many_arguments)]
pub(super) fn fan_end(
    m: &Model,
    view: &BodyView,
    s: &Stripe,
    vertex: VertexId,
    corner: &CornerAt,
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<End, OpError> {
    let (edge, d) = (s.edge, s.d);
    let e = forward(edge);
    let k = corner.pieces.len();
    let mut surfaces: Vec<&Surface> = Vec::with_capacity(k);
    let mut tolerances: Vec<f64> = Vec::with_capacity(k);
    for &face in &corner.pieces {
        let entity = m.face(face)?;
        surfaces.push(m.surface(entity.surface())?);
        tolerances.push(s.tolerance.max(entity.tolerance()));
    }
    let unsupported = |i: usize| OpError::Unsupported {
        a: (GeomKind::Surface(s.surface.kind()), e),
        b: (
            GeomKind::Surface(surfaces[i].kind()),
            forward(corner.pieces[i]),
        ),
    };
    // Each contact against its corner edge's piece: the first against the
    // first, the other against the last.
    let at = m.vertex(vertex)?.point();
    let mut points = [Point3::origin(); 2];
    let mut t = [0.0; 2];
    for c in 0..2 {
        let i = if c == 0 { 0 } else { k - 1 };
        let q = line_origin(&s.lines[c])?;
        t[c] = match *surfaces[i] {
            Surface::Plane { frame } => {
                let n: Vec3 = frame.z().into_inner();
                let dn = d.dot(&n);
                if dn.abs() <= tol.angular {
                    return Err(degenerate(vec![e, forward(vertex)], Reason::VertexBlend));
                }
                (frame.origin() - q).dot(&n) / dn
            }
            Surface::Cylinder { .. } | Surface::Cone { .. } => {
                pierce(q, d, surfaces[i], (at - q).dot(&d), tolerances[i])
                    .ok_or_else(|| unsupported(i))?
            }
            Surface::EllipticCylinder { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => return Err(unsupported(i)),
        };
        points[c] = q + t[c] * d;
    }
    let mut trims = [Trim {
        edge,
        t: 0.0,
        cuts_lo: true,
    }; 2];
    for c in 0..2 {
        trims[c] = cut_corner(m, edge, corner.edges[c], vertex, points[c])?;
    }
    // Where the blend's surface crosses each extra edge on its band, within
    // the edge.
    let mut crossings: Vec<Crossing> = Vec::with_capacity(k - 1);
    for (i, &x) in corner.extras.iter().enumerate() {
        meter.tick()?;
        let too_large = || degenerate(vec![e, forward(x)], Reason::BlendTooLarge);
        let entity = *m.edge(x)?;
        let (curve_id, range) = entity.curve().ok_or(invariant("an extra edge's curve"))?;
        let curve = m.curve(curve_id)?;
        let tolerance = tolerances[i].max(tolerances[i + 1]).max(entity.tolerance());
        let x_tol = Tolerance::new(tolerance, tol.angular);
        let hits =
            match intersect_curve_surface(curve, &s.surface, x_tol, meter).map_err(fault_of)? {
                CurveSurfaceIntersection::Points(hits) => hits,
                CurveSurfaceIntersection::Coincident => Vec::new(),
            };
        let mut found = None;
        for hit in hits.iter().filter(|h| !h.tangent) {
            if shift_into_range(range, hit.t, curve.period()).is_none()
                || !in_band(s, hit.point, tolerance)?
            {
                continue;
            }
            if found.replace(hit.point).is_some() {
                return Err(too_large());
            }
        }
        let point = found.ok_or_else(too_large)?;
        let trim = cut_corner(m, edge, x, vertex, point)?;
        crossings.push(Crossing {
            point,
            trim,
            tolerance,
        });
    }
    // Each piece's arc between its two points, in the end's order.
    let mut gaps = [0.0; 2];
    let mut pieces: Vec<Piece> = Vec::with_capacity(k);
    for (i, &face) in corner.pieces.iter().enumerate() {
        meter.tick()?;
        let (from, from_u, from_edge, from_t) = match i.checked_sub(1) {
            None => (points[0], 0.0, corner.edges[0], trims[0].t),
            Some(j) => {
                let x = &crossings[j];
                (x.point, band_u(s, x.point)?, x.trim.edge, x.trim.t)
            }
        };
        let (to, to_u) = match crossings.get(i) {
            Some(x) => (x.point, band_u(s, x.point)?),
            None => (points[1], s.u1),
        };
        let arc_tol = Tolerance::new(tolerances[i], tol.angular);
        let section = section_between(
            s,
            surfaces[i],
            [from, to],
            &|| unsupported(i),
            arc_tol,
            meter,
        )?;
        match i.checked_sub(1) {
            None => gaps[0] = section.gaps[0],
            Some(j) => crossings[j].tolerance = crossings[j].tolerance.max(section.gaps[0]),
        }
        if i + 1 < k {
            crossings[i].tolerance = crossings[i].tolerance.max(section.gaps[1]);
        } else {
            gaps[1] = section.gaps[1];
        }
        let (curve, range, lo_first) = (section.curve, section.range, section.lo_first);
        let on_face = pcurve_on(&curve, range, surfaces[i], arc_tol, meter).map_err(fault_of)?;
        // On a curved piece, in its loop's translate at the edge cut at the
        // arc's start.
        let on_face = match surfaces[i] {
            Surface::Plane { .. } => on_face,
            Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => {
                let at = at_cut_corner(m, view, face, from_edge, from_t)?;
                placed_uv(on_face, arc_end(range, lo_first, 0), at)
            }
        };
        if !on_side_of_face(m, face, &on_face, range, Side::Inside, samples)? {
            return Err(degenerate(vec![e, forward(face)], Reason::BlendTooLarge));
        }
        let on_blend = pcurve_on(&curve, range, &s.surface, arc_tol, meter).map_err(fault_of)?;
        let on_blend = s.place(on_blend, range.lo(), if lo_first { from_u } else { to_u });
        pieces.push(Piece {
            face,
            arc: Arc {
                curve,
                range,
                lo_first,
                on_face,
                on_blend,
                tolerance: tolerances[i],
            },
        });
    }
    let mut vertex_tolerance = [0.0; 2];
    for c in 0..2 {
        let i = if c == 0 { 0 } else { k - 1 };
        vertex_tolerance[c] = tolerances[i]
            .max(m.edge(corner.edges[c])?.tolerance())
            .max(gaps[c]);
    }
    Ok(End {
        vertex,
        points,
        t,
        trims,
        pieces,
        crossings,
        vertex_tolerance,
        stays: Vec::new(),
    })
}
