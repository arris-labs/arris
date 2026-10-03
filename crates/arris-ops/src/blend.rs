//! Blends: `fillet` rolls a ball of constant radius along named edges of
//! a solid, `chamfer` cuts them flat at a distance (ADR-0007). Each blend
//! is built directly from its edge's two faces in closed form — two
//! planes blend to a cylinder on the line where their offset planes meet,
//! and chamfer to the plane through the lines at the distance from the
//! edge on each; a plane and a cylinder along a ruling blend to a cylinder
//! on the line where the plane's offset meets the cylinder's, and a plane
//! and a cylinder or a cone along a coaxial circle to a torus found in the
//! half-plane through the axis or chamfer to a cone — with its
//! contact curves read off the construction, each end of an open edge
//! trimmed by the face across the corner or met by the blends sharing its
//! vertex — two in a miter, three in a sphere or a triangle, or the next
//! blend of a chain at a tangent vertex on the ball's great circle — and the
//! result assembled through `rebuild::rewrite` with every untouched
//! entity kept by id (`docs/ARCHITECTURE.md` §Operations,
//! `docs/DATA-MODEL.md` §Provenance).

use core::f64::consts::{FRAC_PI_2, PI, TAU};
use std::collections::{BTreeMap, BTreeSet};

use arris_check::arris_topo::arris_geom::region2::Side;
use arris_check::arris_topo::arris_geom::{
    Curve, Curve2, GeomKind, MeetKind, Surface, SurfaceIntersection, SurfaceKind,
    intersect_surfaces, pcurve_on,
};
use arris_check::arris_topo::arris_math::{
    Aabb, Control, Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec2, UnitVec3, Vec2,
    Vec3, wrap_angle,
};
use arris_check::arris_topo::builder::{EdgeKey, EdgeSpec, VertexKey, VertexSpec};
use arris_check::arris_topo::entity::EdgeGeometry;
use arris_check::arris_topo::{
    Body, Curve2Id, Edge, EdgeId, FaceId, Model, Orientation, Provenance, Shape, VertexId,
};
use arris_check::domain::FaceDomain;

use crate::error::{Fault, OpError, Reason, fault_of};
use crate::rebuild::{self, AddedFace, Rewrite, StoredUse, forward};

mod mixed;
mod traced;

fn degenerate(entities: Vec<Shape>, reason: Reason) -> OpError {
    OpError::Degenerate { entities, reason }
}

fn invariant(what: &'static str) -> OpError {
    OpError::Internal(Fault::Invariant { what })
}

/// One use of an edge by a loop of one of the body's faces, addressed.
#[derive(Debug, Clone, Copy)]
struct UseAt {
    face: FaceId,
    loop_index: usize,
    coedge_index: usize,
    /// The stored orientation of the coedge.
    orientation: Orientation,
    pcurve: Curve2Id,
}

/// The body read once: each face's effective orientation and the shell
/// it belongs to, every edge's uses and every vertex's edges, all in the
/// body's own order.
struct View {
    orientation: BTreeMap<FaceId, Orientation>,
    shell_of: BTreeMap<FaceId, usize>,
    faces: Vec<FaceId>,
    uses: BTreeMap<EdgeId, Vec<UseAt>>,
    vertex_edges: BTreeMap<VertexId, BTreeSet<EdgeId>>,
}

impl View {
    fn of(m: &Model, body: Body) -> Result<View, OpError> {
        let mut view = View {
            orientation: BTreeMap::new(),
            shell_of: BTreeMap::new(),
            faces: Vec::new(),
            uses: BTreeMap::new(),
            vertex_edges: BTreeMap::new(),
        };
        for (s, shell) in m.shells(body)?.into_iter().enumerate() {
            for face_use in m.shell(shell.id)?.faces() {
                let face = face_use.oriented_by(shell.orientation);
                if view.orientation.insert(face.id, face.orientation).is_some() {
                    return Err(invariant("a face used once"));
                }
                view.shell_of.insert(face.id, s);
                view.faces.push(face.id);
                let entity = m.face(face.id)?;
                for (loop_index, l) in entity.loops().iter().enumerate() {
                    for (coedge_index, c) in l.coedges().iter().enumerate() {
                        view.uses.entry(c.edge()).or_default().push(UseAt {
                            face: face.id,
                            loop_index,
                            coedge_index,
                            orientation: c.orientation(),
                            pcurve: c.pcurve(),
                        });
                        let edge = m.edge(c.edge())?;
                        for v in [edge.start(), edge.end()] {
                            view.vertex_edges.entry(v).or_default().insert(c.edge());
                        }
                    }
                }
            }
        }
        Ok(view)
    }

    /// The face's outward normal at `uv`, the surface's composed with the
    /// body's use of the face.
    fn outward(&self, m: &Model, face: FaceId, uv: Point2) -> Result<Vec3, OpError> {
        let surface = m.surface(m.face(face)?.surface())?;
        let n = surface
            .normal(uv.x, uv.y)
            .ok_or(OpError::Internal(Fault::NoNormal { face }))?;
        Ok(n.into_inner() * self.orientation[&face].sign())
    }
}

/// A corner edge cut short by a blend's end: where on its curve, and at
/// which of its ends.
#[derive(Debug, Clone, Copy)]
struct Trim {
    edge: EdgeId,
    t: f64,
    /// `true` when the cut is at the edge's `range.lo()` end.
    cuts_lo: bool,
}

/// The arc where the blend meets the face across a corner: a circle when
/// that plane is perpendicular to the edge, an ellipse when oblique.
struct Arc {
    curve: Curve,
    range: Interval,
    /// `true` when `range.lo()` is at the contact at `u = 0`.
    lo_first: bool,
    /// Its pcurve on the face across, exact.
    on_face: Curve2,
    /// Its pcurve on the blend, placed in the blend loop's translate.
    on_blend: Curve2,
    tolerance: f64,
}

/// One end of a blend: the corner vertex it consumes, the face across
/// it, the two trim points on the contact lines and the corner edges
/// they shorten, and the arc between them.
struct End {
    vertex: VertexId,
    face: FaceId,
    /// The trim points, one per contact (`u = 0` first).
    points: [Point3; 2],
    /// The contact lines' parameters there.
    t: [f64; 2],
    /// The corner edge each contact's trim point cuts.
    trims: [Trim; 2],
    arc: Arc,
    /// The tolerance of each trim vertex.
    vertex_tolerance: [f64; 2],
}

/// A contact of the blend with one of the edge's faces: the line at
/// distance `r tan(φ/2)` from the edge, its `u` on the blend, and its
/// pcurves.
struct Contact {
    face: FaceId,
    line: Curve,
    range: Interval,
    on_face: Curve2,
    on_blend: Curve2,
    tolerance: f64,
}

/// What a blend is: a rolling ball's fillet or an equal-distance chamfer.
#[derive(Debug, Clone, Copy)]
enum Kind {
    Fillet { radius: f64 },
    Chamfer { distance: f64 },
}

/// A stripe's cross-section, which its end curves are decided from.
#[derive(Debug, Clone, Copy)]
enum Section {
    /// A fillet's cylinder of `radius` about the axis through
    /// `axis_origin`, a point level with the edge's line origin.
    Round { axis_origin: Point3, radius: f64 },
    /// A chamfer's plane, its frame's origin on the contact at `u = 0`
    /// and its `X` across to the other.
    Flat,
}

/// One edge's stripe: the blend surface with its two contact lines and
/// the faces they lie on, nothing about its ends decided yet.
struct Stripe {
    edge: EdgeId,
    start: VertexId,
    end: VertexId,
    /// The edge's direction: the cylinder's `Z`, the plane's `Y`.
    d: Vec3,
    /// `true` when one of the edge's faces is a cylinder the edge is a
    /// ruling of, its contact there a ruling too.
    ruling: bool,
    section: Section,
    frame: Frame,
    surface: Surface,
    /// The angle the dihedral turns a ball through, `π − φ` for normals
    /// `φ` apart.
    beta: f64,
    /// The `u` of the contact at the blend's far side: `β` on a fillet's
    /// cylinder, the chamfer's width on its plane.
    u1: f64,
    convex: bool,
    /// The contact lines, at `u = 0` then at `u = β`.
    lines: [Curve; 2],
    /// The face each contact lies on, in the same order.
    faces: [FaceId; 2],
    /// The edge's use by each of those faces, in the same order.
    uses: [UseAt; 2],
    tolerance: f64,
}

/// One end of a blend: trimmed by the face across the corner, met by the
/// other blend of a miter, or met by the corner face of three blends.
enum EndKind {
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
    fn t(&self, miters: &[Miter], corners: &[Corner], k: usize) -> f64 {
        match self {
            EndKind::Face(end) => end.t[k],
            EndKind::Miter { at, side } => miters[*at].t[*side][k],
            EndKind::Corner { at, side } => corners[*at].t[*side][k],
        }
    }

    /// The tolerance a face end's trim vertex carries, `None` at a miter
    /// or a corner.
    fn vertex_tolerance(&self, k: usize) -> Option<f64> {
        match self {
            EndKind::Face(end) => Some(end.vertex_tolerance[k]),
            EndKind::Miter { .. } | EndKind::Corner { .. } => None,
        }
    }
}

/// Two blends meeting at a vertex whose third edge stays sharp
/// (ADR-0007): the curve they meet in — the ellipse where two
/// equal-radius cylinders cross, in the plane bisecting their axes
/// through the ball's one centre, or the line where two chamfer planes
/// cross — from the point where the two contacts on the shared face cross
/// to the point on the third edge where the other two contacts meet it.
struct Miter {
    /// The two blended edges, in the blends' order.
    edges: [EdgeId; 2],
    /// For each of the two stripes, which of its contacts lies on the
    /// shared face.
    shared: [usize; 2],
    /// The contact lines' parameters at the miter, `[side][contact]`.
    t: [[f64; 2]; 2],
    /// Where the two contacts on the shared face cross.
    q: Point3,
    /// Where the other two contacts meet the third edge.
    p3: Point3,
    curve: Curve,
    range: Interval,
    /// `true` when `range.lo()` is at `q`.
    q_first: bool,
    /// For each side, `true` when `range.lo()` is at that stripe's
    /// contact at `u = 0`.
    lo_first: [bool; 2],
    /// The ellipse's pcurve on each stripe's cylinder, placed.
    on_blend: [Curve2; 2],
    /// The third edge shortened to `p3`.
    trim: Trim,
    tolerance: f64,
    q_tolerance: f64,
    p3_tolerance: f64,
}

/// One side of a corner: where one of its three blends meets the corner
/// face — a great circle of the sphere, or a side of the triangle —
/// between the corner's points on that blend's two faces.
struct CornerArc {
    curve: Curve,
    range: Interval,
    /// The corner's points at `range.lo()`, then at `range.hi()`.
    ends: [usize; 2],
    /// `true` when `range.lo()` is at the blend's contact at `u = 0`.
    lo_first: bool,
    /// Its pcurve on the blend, placed.
    on_blend: Curve2,
    /// Its pcurve on the corner face, placed.
    on_corner: Curve2,
    /// `true` when the corner face's loop walks it along its range.
    along: bool,
}

/// A sphere corner's pole: the degenerate edge where its two meridians
/// meet, `u` running along its pcurve at `v = π/2` and walked back.
struct Pole {
    /// The corner's point it stands at.
    point: usize,
    range: Interval,
    pcurve: Curve2,
    /// The place in the corner's walk after which its loop crosses it.
    after: usize,
}

/// Three blends at a vertex of three planes (ADR-0007). Each face's two
/// contacts cross at one point, the corner's three points. Three fillets'
/// axes meet at the ball's one centre, and the corner is the sphere of the
/// radius about it, tangent to each cylinder along the great circle in the
/// plane through the centre square to its axis, between the points on its
/// two faces: its frame's `Z` toward the point of a face square to the
/// other two, the pole, so that the side between those two is the equator
/// and the other two sides meridians. Three chamfers meet in the triangle
/// of the three points, each side a chord in one chamfer's plane.
struct Corner {
    /// The three blended edges, in the blends' order: the sides.
    edges: [EdgeId; 3],
    /// The corner's three faces, each holding the point at its index.
    faces: [FaceId; 3],
    points: [Point3; 3],
    /// For each side, the point each of its contacts ends at.
    contact_point: [[usize; 2]; 3],
    /// The contact lines' parameters at the corner, `[side][contact]`.
    t: [[f64; 2]; 3],
    /// By side.
    arcs: [CornerArc; 3],
    /// The sides in the order the corner face's loop walks them.
    walk: [usize; 3],
    pole: Option<Pole>,
    surface: Surface,
    orientation: Orientation,
    tolerance: f64,
}

/// One edge's blend, decided and checked, before anything is written.
struct Blend {
    stripe: Stripe,
    /// The contacts, at `u = 0` then at `u = β`.
    contacts: [Contact; 2],
    /// At the edge's `range.lo()` end, then its `range.hi()` end.
    ends: [EndKind; 2],
}

/// The parameters of the closest points of two lines with unit
/// directions, or `None` when the lines are parallel within `tol.angular`.
fn lines_cross(o1: Point3, d1: Vec3, o2: Point3, d2: Vec3, tol: Tolerance) -> Option<(f64, f64)> {
    let c = d1.dot(&d2);
    if d1.cross(&d2).norm() <= tol.angular {
        return None;
    }
    let denom = 1.0 - c * c;
    let w = o2 - o1;
    let (w1, w2) = (w.dot(&d1), w.dot(&d2));
    Some(((w1 - c * w2) / denom, (c * w1 - w2) / denom))
}

/// The origin of a contact line.
fn line_origin(line: &Curve) -> Result<Point3, OpError> {
    match line {
        Curve::Line { origin, .. } => Ok(*origin),
        Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => {
            Err(invariant("a contact line"))
        }
    }
}

/// `t` moved by whole periods into `range`, when it can be; a parameter
/// of a non-periodic curve as it is.
fn into_range(range: Interval, t: f64, period: Option<f64>) -> Option<f64> {
    match period {
        Some(p) => {
            let k = ((range.lo() - t) / p).ceil();
            let shifted = t + k * p;
            (shifted <= range.hi()).then_some(shifted)
        }
        None => range.contains(t).then_some(t),
    }
}

/// `pcurve` translated by whole turns in `u` so that its point at `t`
/// has `u` nearest `target`: the blend loop is written in one translate
/// of the cylinder's domain, and `pcurve_on` reports `u` in `[0, 2π)`.
fn placed(pcurve: Curve2, t: f64, target: f64) -> Curve2 {
    let u = pcurve.point(t).x;
    let k = ((target - u) / TAU).round();
    if k == 0.0 {
        pcurve
    } else {
        pcurve.translated(Vec2::new(k * TAU, 0.0))
    }
}

impl Stripe {
    /// `pcurve` on the blend placed so that its point at `t` has `u`
    /// nearest `target`: moved by whole turns on a fillet's cylinder, as
    /// it is on a chamfer's plane.
    fn place(&self, pcurve: Curve2, t: f64, target: f64) -> Curve2 {
        match self.section {
            Section::Round { .. } => placed(pcurve, t, target),
            Section::Flat => pcurve,
        }
    }
}

/// The segment from `a` to `b`: a line starting at `a` over
/// `[0, |b − a|]`.
fn chord(a: Point3, b: Point3, tol: Tolerance) -> Result<(Curve, Interval), OpError> {
    let v = b - a;
    let direction =
        UnitVec3::try_new(v, tol.linear).ok_or(invariant("a segment of positive length"))?;
    let range =
        Interval::new(0.0, v.norm()).map_err(|_| invariant("a segment of positive length"))?;
    Ok((
        Curve::Line {
            origin: a,
            direction,
        },
        range,
    ))
}

/// `true` when `pcurve` over `range` lies strictly on `side` of `face` at
/// `samples` interior parameters, by the face's own domain at its own
/// tolerance: the test a contact line and an end arc pass before a blend
/// is built (ADR-0007, the `BlendTooLarge` bound). A contact lies inside
/// its face; an end arc inside the face across when the blend removes
/// material and outside it when a concave blend adds the corner to it —
/// either way, a curve that changes side crosses an edge of the face
/// that is not the corner's own.
fn on_side_of_face(
    m: &Model,
    face: FaceId,
    pcurve: &Curve2,
    range: Interval,
    side: Side,
    samples: usize,
) -> Result<bool, OpError> {
    let tolerance = m.face(face)?.tolerance();
    let domain = FaceDomain::of(m, face, tolerance)?;
    let n = samples.max(1);
    for i in 1..=n {
        let t = range.lerp(i as f64 / (n + 1) as f64);
        if domain.side(pcurve.point(t)).0 != side {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The range of the end curve between `t_a` and `t_b` that holds
/// `t_mid`, the two candidates being the two ways round a closed curve:
/// the interval and whether it starts at `t_a`.
fn arc_between(t_a: f64, t_b: f64, t_mid: f64) -> Result<(Interval, bool), OpError> {
    let up = |t: f64, from: f64| if t >= from { t } else { t + TAU };
    let (b, mid) = (up(t_b, t_a), up(t_mid, t_a));
    let (lo, hi, a_first) = if mid <= b {
        (t_a, b, true)
    } else {
        (t_b, up(t_a, t_b), false)
    };
    Interval::new(lo, hi)
        .map(|r| (r, a_first))
        .map_err(|_| invariant("an end arc of positive length"))
}
/// The ball of `radius` rolling along a line edge, through `origin` along
/// `d`, between a plane of outward normal `n_plane` and a cylinder
/// (`axis`, radius `big`) whose ruling the edge is, `n_cylinder` its
/// outward normal at the edge and `s` `−1` on a convex edge, `1` on a
/// concave one: the ball's centre level with `origin`, and the directions
/// from it to its contact with the plane and with the cylinder. The
/// centre is on the plane offset by `r` into the ball's side and on the
/// cylinder coaxial with the face at `R ∓ r` — `R − r` when the ball is
/// inside the cylinder, `R + r` when outside — the one of
/// the two lines where they meet on the edge's side of the axis, the side
/// the edge itself is on and the root follows from `r = 0`. `None` when
/// no ball of `radius` touches both, the plane offset clear of the
/// cylinder or the cylinder's offset at no radius.
#[allow(clippy::too_many_arguments)]
fn ruling_ball(
    origin: Point3,
    d: Vec3,
    n_plane: Vec3,
    n_cylinder: Vec3,
    axis: &Frame,
    big: f64,
    radius: f64,
    s: f64,
    tol: Tolerance,
) -> Option<(Point3, Vec3, Vec3)> {
    // The axis's point level with the edge's origin, and the face's
    // normal against the axis's outward direction there.
    let axis_point = axis.origin() + (origin - axis.origin()).dot(&d) * d;
    let radial = origin - axis_point;
    let sigma = n_cylinder.dot(&radial).signum();
    // The ball is on the side `s` times each face's outward normal points
    // to: its centre at `s r` along the plane's normal from the plane, and
    // `s r` along the cylinder's from the cylinder.
    let rho = big + s * sigma * radius;
    let h = (axis_point - origin).dot(&n_plane) - s * radius;
    let across = rho * rho - h * h;
    if rho <= tol.linear || across <= 0.0 || across.sqrt() <= tol.linear {
        return None;
    }
    let w = d.cross(&n_plane);
    let side = radial.dot(&w).signum();
    let centre = axis_point - h * n_plane + side * across.sqrt() * w;
    let out = (centre - axis_point) / rho;
    Some((centre, -s * n_plane, -s * sigma * out))
}

/// The stripe of one edge: the blend surface — a fillet's cylinder or a
/// chamfer's plane — and its two contact lines decided from the edge's two
/// faces, two planes or a plane and a cylinder along a ruling, its ends
/// not yet.
fn stripe(
    m: &Model,
    view: &View,
    edge: EdgeId,
    kind: Kind,
    tol: Tolerance,
) -> Result<Stripe, OpError> {
    let e = forward(edge);
    let entity = *m.edge(edge)?;
    let Some((curve_id, range)) = entity.curve() else {
        return Err(degenerate(vec![e], Reason::VertexBlend));
    };
    if entity.start() == entity.end() {
        return Err(invariant("an open edge, a closed one being a ring"));
    }
    let uses = view
        .uses
        .get(&edge)
        .filter(|u| u.len() == 2)
        .ok_or(invariant("two uses of the blended edge"))?;
    let (ua, ub) = (uses[0], uses[1]);
    let faces = [ua.face, ub.face];
    let pcurves = [m.curve2(ua.pcurve)?, m.curve2(ub.pcurve)?];
    let mut normals = [Vec3::zeros(); 2];
    for i in 0..2 {
        normals[i] = view.outward(m, faces[i], pcurves[i].point(range.midpoint()))?;
    }
    let [n1, n2] = normals;
    let [f1, f2] = faces;
    // A tangent dihedral has no corner to roll a ball into, whatever the
    // surfaces are.
    if n1.cross(&n2).norm() <= tol.angular {
        return Err(degenerate(
            vec![e, forward(f1), forward(f2)],
            Reason::TangentChain,
        ));
    }
    let surfaces = [
        m.surface(m.face(f1)?.surface())?,
        m.surface(m.face(f2)?.surface())?,
    ];
    let unsupported = || OpError::Unsupported {
        a: (GeomKind::Surface(surfaces[0].kind()), forward(f1)),
        b: (GeomKind::Surface(surfaces[1].kind()), forward(f2)),
    };
    // The table: two planes, or a plane and a cylinder along one of its
    // rulings — the cylinder's index, frame and radius; every other pair
    // is named.
    let cylinder = match (surfaces[0], surfaces[1]) {
        (Surface::Plane { .. }, Surface::Plane { .. }) => None,
        (Surface::Plane { .. }, &Surface::Cylinder { frame, radius }) => Some((1, frame, radius)),
        (&Surface::Cylinder { frame, radius }, Surface::Plane { .. }) => Some((0, frame, radius)),
        (
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_),
        ) => return Err(unsupported()),
    };
    let curve = m.curve(curve_id)?;
    let &Curve::Line { origin, direction } = curve else {
        return Err(OpError::Unsupported {
            a: (GeomKind::Curve(curve.kind()), e),
            b: (GeomKind::Surface(SurfaceKind::Plane), forward(f1)),
        });
    };
    let d: Vec3 = direction.into_inner();
    // Convex or concave is read from the dihedral: the direction into
    // face 1 from the edge against face 2's outward normal.
    let t1 = d * view.orientation[&f1].compose(ua.orientation).sign();
    let convex = n1.cross(&t1).dot(&n2) < 0.0;
    let s = if convex { -1.0 } else { 1.0 };
    let c = n1.dot(&n2);
    // A unit ball's centre against the edge, on both offset planes, and
    // each of its contacts against the edge: two planes' construction.
    let unit_offset: Vec3 = (n1 + n2) * (s / (1.0 + c));
    let unit_contact = [unit_offset - n1 * s, unit_offset - n2 * s];
    // `X` toward one contact, so `u` runs from `0` there to its value at
    // the other, which a ball reaches turning by `β` about the edge's
    // direction — `π − φ` between two planes. `x` is the direction from
    // the ball's centre to each contact.
    let turn = |from: Vec3, to: Vec3| to.dot(&d.cross(&from)).atan2(to.dot(&from));
    let order = |x: [Vec3; 2]| {
        if turn(x[0], x[1]) > 0.0 {
            (0, turn(x[0], x[1]))
        } else {
            (1, turn(x[1], x[0]))
        }
    };
    let (lo, beta, contact_offset, section, frame, surface, u1) = match (kind, cylinder) {
        (Kind::Fillet { radius }, _) => {
            let (axis_origin, x) = match cylinder {
                None => (origin + unit_offset * radius, [-s * n1, -s * n2]),
                Some((k, axis, big)) => {
                    let (centre, on_plane, on_cylinder) = ruling_ball(
                        origin,
                        d,
                        normals[1 - k],
                        normals[k],
                        &axis,
                        big,
                        radius,
                        s,
                        tol,
                    )
                    .ok_or_else(|| {
                        degenerate(vec![e, forward(f1), forward(f2)], Reason::BlendTooLarge)
                    })?;
                    let mut x = [on_plane; 2];
                    x[k] = on_cylinder;
                    (centre, x)
                }
            };
            let (lo, beta) = order(x);
            // The cylinder's `Z` is the edge's direction, so `v` is its
            // parameter.
            let frame = Frame::new(axis_origin, d, x[lo])?;
            (
                lo,
                beta,
                x.map(|w| axis_origin + w * radius - origin),
                Section::Round {
                    axis_origin,
                    radius,
                },
                frame,
                Surface::Cylinder { frame, radius },
                beta,
            )
        }
        (Kind::Chamfer { .. }, Some(_)) => return Err(unsupported()),
        (Kind::Chamfer { distance }, None) => {
            let (lo, beta) = order([-s * n1, -s * n2]);
            let hi = 1 - lo;
            // Each contact at `distance` from the edge along its face, the
            // way the unit ball's contact lies from it.
            let mut contact_offset = [Vec3::zeros(); 2];
            for (offset, w) in contact_offset.iter_mut().zip(unit_contact) {
                let into = UnitVec3::try_new(w, tol.angular)
                    .ok_or(invariant("a contact off the blended edge"))?;
                *offset = into.into_inner() * distance;
            }
            let chord = contact_offset[hi] - contact_offset[lo];
            let width = chord.norm();
            let across = chord / width;
            // `X` across to the far contact and `Y` the edge's direction,
            // so `v` is its parameter.
            let frame = Frame::new(origin + contact_offset[lo], across.cross(&d), across)?;
            (
                lo,
                beta,
                contact_offset,
                Section::Flat,
                frame,
                Surface::Plane { frame },
                width,
            )
        }
    };
    let tolerance = m
        .precision()
        .default_tolerance
        .max(m.face(f1)?.tolerance())
        .max(m.face(f2)?.tolerance());
    let by_u = [lo, 1 - lo];
    Ok(Stripe {
        edge,
        start: entity.start(),
        end: entity.end(),
        d,
        ruling: cylinder.is_some(),
        section,
        frame,
        surface,
        beta,
        u1,
        convex,
        lines: by_u.map(|i| Curve::Line {
            origin: origin + contact_offset[i],
            direction,
        }),
        faces: by_u.map(|i| faces[i]),
        uses: by_u.map(|i| uses[i]),
        tolerance,
    })
}

/// The parameter of `edge` at its end `vertex`, its start's on a closed
/// edge; `None` for an edge with no curve.
fn parameter_at(m: &Model, edge: EdgeId, vertex: VertexId) -> Result<Option<f64>, OpError> {
    let entity = *m.edge(edge)?;
    Ok(entity.curve().map(|(_, range)| {
        if entity.start() == vertex {
            range.lo()
        } else {
            range.hi()
        }
    }))
}

/// Whether the two faces of `edge` are tangent at its parameter `t`, their
/// outward normals parallel within `tol.angular`; `None` for an edge not
/// used by exactly two faces.
fn tangent_at(
    m: &Model,
    view: &View,
    edge: EdgeId,
    t: f64,
    tol: Tolerance,
) -> Result<Option<bool>, OpError> {
    let Some(uses) = view.uses.get(&edge).filter(|u| u.len() == 2) else {
        return Ok(None);
    };
    let mut normals = [Vec3::zeros(); 2];
    for (n, u) in normals.iter_mut().zip(uses) {
        *n = view.outward(m, u.face, m.curve2(u.pcurve)?.point(t))?;
    }
    Ok(Some(normals[0].cross(&normals[1]).norm() <= tol.angular))
}

/// Whether `edge` is convex, read at its midpoint as a stripe or a ring
/// reads it: the direction into its first face against the second face's
/// outward normal. `None` for an edge with no curve or not used by exactly
/// two faces.
fn convex_edge(m: &Model, view: &View, edge: EdgeId) -> Result<Option<bool>, OpError> {
    let Some((curve, range)) = m.edge(edge)?.curve() else {
        return Ok(None);
    };
    let Some(&[ua, ub]) = view.uses.get(&edge).map(Vec::as_slice) else {
        return Ok(None);
    };
    let mid = range.midpoint();
    let n1 = view.outward(m, ua.face, m.curve2(ua.pcurve)?.point(mid))?;
    let n2 = view.outward(m, ub.face, m.curve2(ub.pcurve)?.point(mid))?;
    let t1 =
        m.curve(curve)?.eval(mid).d1 * view.orientation[&ua.face].compose(ua.orientation).sign();
    Ok(Some(n1.cross(&t1).dot(&n2) < 0.0))
}

/// The edge a blend of `edge` runs on into at `vertex`, when that is a
/// tangent vertex (ADR-0035 §1): exactly three edges `edge`, `next` and
/// `w`, the two faces of `w` tangent at the vertex, `next` open and not a
/// tangent dihedral at its midpoint, `edge` and `next` sharing exactly one
/// face, both convex or both concave — an outline that turns from one to
/// the other there puts the ball on the far side of the shared face — and
/// the direction leaving the vertex along `next` within a right angle of
/// the one arriving along `edge`. `None` at any other vertex, and where
/// both of the vertex's other edges would qualify.
fn tangent_vertex(
    m: &Model,
    view: &View,
    edge: EdgeId,
    vertex: VertexId,
    tol: Tolerance,
) -> Result<Option<EdgeId>, OpError> {
    let Some(at) = view.vertex_edges.get(&vertex) else {
        return Ok(None);
    };
    if at.len() != 3 || !at.contains(&edge) {
        return Ok(None);
    }
    let others: Vec<EdgeId> = at.iter().copied().filter(|&x| x != edge).collect();
    let faces_of = |x: EdgeId| -> BTreeSet<FaceId> {
        view.uses
            .get(&x)
            .map(|u| u.iter().map(|u| u.face).collect())
            .unwrap_or_default()
    };
    // The unit tangent of `x` at the vertex, pointing away from it.
    let leaving = |x: EdgeId| -> Result<Option<Vec3>, OpError> {
        let entity = *m.edge(x)?;
        let Some((curve, range)) = entity.curve() else {
            return Ok(None);
        };
        let at_lo = entity.start() == vertex;
        let t = if at_lo { range.lo() } else { range.hi() };
        let d1 = m.curve(curve)?.eval(t).d1;
        let away = if at_lo { d1 } else { -d1 };
        Ok(UnitVec3::try_new(away, tol.linear).map(UnitVec3::into_inner))
    };
    let Some(arriving) = leaving(edge)?.map(|d| -d) else {
        return Ok(None);
    };
    let own = faces_of(edge);
    let mut found = None;
    for (i, &next) in others.iter().enumerate() {
        let w = others[1 - i];
        let next_entity = *m.edge(next)?;
        let (Some(t_w), Some((_, next_range))) = (parameter_at(m, w, vertex)?, next_entity.curve())
        else {
            continue;
        };
        if next_entity.start() == next_entity.end()
            || tangent_at(m, view, w, t_w, tol)? != Some(true)
            || tangent_at(m, view, next, next_range.midpoint(), tol)? != Some(false)
            || own.intersection(&faces_of(next)).count() != 1
            || convex_edge(m, view, next)? != convex_edge(m, view, edge)?
        {
            continue;
        }
        let Some(away) = leaving(next)? else {
            continue;
        };
        if away.dot(&arriving) > 0.0 && found.replace(next).is_some() {
            return Ok(None);
        }
    }
    Ok(found)
}

/// `named` closed under tangent continuation (ADR-0035 §1): every edge a
/// blend of one of them runs on into through tangent vertices, walked
/// from each end until a vertex that is not one.
fn chain(
    m: &Model,
    view: &View,
    named: &[EdgeId],
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<BTreeSet<EdgeId>, OpError> {
    let mut reached: BTreeSet<EdgeId> = named.iter().copied().collect();
    let mut todo: Vec<EdgeId> = named.to_vec();
    while let Some(edge) = todo.pop() {
        meter.tick()?;
        let entity = *m.edge(edge)?;
        if entity.start() == entity.end() {
            continue;
        }
        for vertex in [entity.start(), entity.end()] {
            if let Some(next) = tangent_vertex(m, view, edge, vertex, tol)?
                && reached.insert(next)
            {
                todo.push(next);
            }
        }
    }
    Ok(reached)
}

/// The corner at `vertex`, the end of the blended `edge` at its start
/// (`at_lo`) or its end, `uses` the edge's uses by its two faces: the
/// corner edge each of those faces' loops runs on to there, in the same
/// order, and the face across, the one face the two corner edges share
/// beyond the edge's own. A vertex of other than these three edges, or
/// corner edges that share no such face, is `Reason::VertexBlend`; a corner
/// edge whose two faces meet tangentially at the vertex — the contact line
/// every blend face meets its neighbours along — at a vertex the chain did
/// not run on through, the next edge turning back or itself a tangent
/// dihedral, is `Reason::TangentChain` (ADR-0035 §6).
fn corner_of(
    m: &Model,
    view: &View,
    edge: EdgeId,
    uses: &[UseAt; 2],
    vertex: VertexId,
    at_lo: bool,
    tol: Tolerance,
) -> Result<([EdgeId; 2], FaceId), OpError> {
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
    if three.len() != 3 || *at_vertex != three {
        return Err(vertex_blend());
    }
    for &corner in &corner_edges {
        let Some(t) = parameter_at(m, corner, vertex)? else {
            return Err(vertex_blend());
        };
        if tangent_at(m, view, corner, t, tol)?.ok_or(invariant("two uses of the corner edge"))? {
            return Err(degenerate(
                vec![e, forward(corner), v],
                Reason::TangentChain,
            ));
        }
    }
    let faces = uses.map(|u| u.face);
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
    Ok((corner_edges, across))
}

/// The corner edge `corner` cut at `point`, its end at `vertex` moving
/// there: its parameter and which end it cuts, checked to leave the edge a
/// positive length clear of its far vertex — `Reason::BlendTooLarge`
/// naming the blended `edge` and the corner edge otherwise.
fn cut_corner(
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
    let Some(tc) = into_range(crange, projection.t, ccurve.period()) else {
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

/// The point of the face across's own `(u, v)` at `t` on the corner edge
/// `corner`, which a contact cuts there: the translate an end arc's pcurve
/// on a curved face across is placed in.
fn at_cut_corner(
    m: &Model,
    view: &View,
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
fn arc_end(range: Interval, lo_first: bool, c: usize) -> f64 {
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
fn pierce(q: Point3, d: Vec3, surface: &Surface, near: f64, tolerance: f64) -> Option<f64> {
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
fn stretch_between(
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
/// conic the intersector writes exactly. A pierce or a section that does
/// not decide the end is `Unsupported` naming the blend and the face
/// across.
fn face_end(
    m: &Model,
    view: &View,
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
    let (corner_edges, face3) = corner_of(m, view, edge, &s.uses, vertex, at_lo, tol)?;
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
            // contact, beside it, at the root nearest there.
            let at = m.vertex(vertex)?.point();
            for k in 0..2 {
                let q = line_origin(&s.lines[k])?;
                t[k] =
                    pierce(q, d, surface3, (at - q).dot(&d), arc_tolerance).ok_or_else(refuse)?;
                points[k] = q + t[k] * d;
            }
            // The blend's band between its contacts: `u` in `[0, u₁]`.
            let band = |p: Point3| -> Result<bool, OpError> {
                Ok(match s.section {
                    Section::Round { radius, .. } => {
                        let u = s.surface.project(p).map_err(fault_of)?.uv.x;
                        let slack = arc_tolerance / radius;
                        u <= s.u1 + slack || u >= TAU - slack
                    }
                    Section::Flat => {
                        let u = s.frame.to_local(p).x;
                        u >= -arc_tolerance && u <= s.u1 + arc_tolerance
                    }
                })
            };
            let reach = match s.section {
                Section::Round { radius, .. } => 2.0 * radius,
                Section::Flat => s.u1,
            };
            let within = Aabb::of_point(points[0])
                .union(Aabb::of_point(points[1]))
                .inflated(reach);
            match s.section {
                Section::Round { .. } => {
                    let traced = traced::traced_end(
                        &s.surface, surface3, points, &within, &band, &refuse, arc_tol, meter,
                    )?;
                    gaps = traced.gaps;
                    (traced.curve, traced.range, traced.lo_first)
                }
                Section::Flat => {
                    // A plane meets a cylinder or a cone in a conic, which
                    // the intersector writes exactly.
                    let cut = intersect_surfaces(&s.surface, surface3, &within, arc_tol, meter)
                        .map_err(fault_of)?;
                    let curves: Vec<Curve> = match cut {
                        SurfaceIntersection::Meets { curves, .. } => curves
                            .into_iter()
                            .filter(|c| c.kind == MeetKind::Crossing)
                            .map(|c| c.curve)
                            .collect(),
                        SurfaceIntersection::Empty | SurfaceIntersection::Coincident => Vec::new(),
                    };
                    let found = stretch_between(&curves, points, &band, arc_tolerance)?;
                    let (curve, range, lo_first) = found.ok_or_else(refuse)?;
                    let ends = if lo_first {
                        [range.lo(), range.hi()]
                    } else {
                        [range.hi(), range.lo()]
                    };
                    gaps = [0, 1].map(|k| (points[k] - curve.point(ends[k])).norm());
                    (curve, range, lo_first)
                }
            }
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
    let corner = mixed::corner_trims(
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
    )?;
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
    if !on_side_of_face(m, face3, &on_face, arc_range, corner.side, samples)? {
        return Err(degenerate(vec![e, forward(face3)], Reason::BlendTooLarge));
    }
    let on_blend =
        pcurve_on(&arc_curve, arc_range, &s.surface, arc_tol, meter).map_err(fault_of)?;
    let on_blend = s.place(on_blend, arc_range.lo(), if lo_first { 0.0 } else { s.u1 });
    Ok(End {
        vertex,
        face: face3,
        points,
        t,
        trims,
        arc: Arc {
            curve: arc_curve,
            range: arc_range,
            lo_first,
            on_face,
            on_blend,
            tolerance: arc_tolerance,
        },
        vertex_tolerance,
    })
}

/// The contact lines of `s` between their parameters at its two ends,
/// `t[end][contact]`, each checked to lie inside its face.
fn contacts(
    m: &Model,
    s: &Stripe,
    t: [[f64; 2]; 2],
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<[Contact; 2], OpError> {
    meter.tick()?;
    let e = forward(s.edge);
    let mut contacts: Vec<Contact> = Vec::with_capacity(2);
    let spans = [0, 1].map(|k| (t[0][k], t[1][k]));
    for (k, ((&face, line), &(lo, hi))) in s.faces.iter().zip(&s.lines).zip(&spans).enumerate() {
        let too_large = || degenerate(vec![e, forward(face)], Reason::BlendTooLarge);
        if hi - lo <= s.tolerance {
            return Err(too_large());
        }
        let range = Interval::new(lo, hi).map_err(|_| too_large())?;
        let surface = m.surface(m.face(face)?.surface())?;
        let line_tol = Tolerance::new(s.tolerance, tol.angular);
        let on_face = pcurve_on(line, range, surface, line_tol, meter).map_err(fault_of)?;
        // On a cylinder, in the translate of the face's own loop: by whole
        // turns to the `u` of the blended edge's pcurve there.
        let on_face = match surface {
            Surface::Cylinder { .. } => {
                let edge_u = m.curve2(s.uses[k].pcurve)?.point(range.midpoint()).x;
                placed(on_face, range.lo(), edge_u)
            }
            Surface::Plane { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => on_face,
        };
        if !on_side_of_face(m, face, &on_face, range, Side::Inside, samples)? {
            return Err(too_large());
        }
        let u = if k == 0 { 0.0 } else { s.u1 };
        let on_blend = pcurve_on(line, range, &s.surface, line_tol, meter).map_err(fault_of)?;
        let on_blend = s.place(on_blend, range.lo(), u);
        contacts.push(Contact {
            face,
            line: line.clone(),
            range,
            on_face,
            on_blend,
            tolerance: s.tolerance,
        });
    }
    contacts
        .try_into()
        .map_err(|_| invariant("two contacts of the blend"))
}

/// The miter of stripes `a` and `b` at `vertex`, a corner of three edges
/// whose third stays sharp (ADR-0007). The two stripes share one face,
/// and their contacts on the faces the third edge separates meet it at
/// one point, where the third edge is shortened. Two fillets have equal
/// dihedrals, so their axes cross at the ball's one centre and the miter
/// is the ellipse of the two cylinders in the plane bisecting their axes,
/// from where the two contacts on the shared face cross to that point,
/// its pcurve on each cylinder fitted by the oblique-section rule; two
/// chamfers meet in the line between the same two points. A corner of
/// two fillets whose dihedrals differ or of two chamfers whose far
/// contacts miss each other on the third edge, whose blends are not both
/// convex or both concave, or whose edges do not share exactly one face
/// is `Reason::VertexBlend`.
fn miter(
    m: &Model,
    view: &View,
    a: &Stripe,
    b: &Stripe,
    vertex: VertexId,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Miter, OpError> {
    meter.tick()?;
    let v = forward(vertex);
    let (ea, eb) = (forward(a.edge), forward(b.edge));
    let vertex_blend = || degenerate(vec![ea, eb, v], Reason::VertexBlend);
    // A ruling blend's contact on its cylinder meets no other blend's
    // contact on the third edge: two arcs, the blend-network cycle's.
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
    // Both convex or both concave, and two fillets of equal dihedrals:
    // what puts their axes through one centre and their far contacts
    // through one point of the third edge; anything else is a corner of
    // two arcs, the
    // blend-network cycle's.
    let unequal = match (a.section, b.section) {
        (Section::Round { .. }, Section::Round { .. }) => (a.beta - b.beta).abs() > tol.angular,
        (Section::Flat, Section::Flat) => false,
        (Section::Round { .. }, Section::Flat) | (Section::Flat, Section::Round { .. }) => {
            return Err(invariant("one kind of blend in one call"));
        }
    };
    if unequal || a.convex != b.convex {
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
    let (p3, tpa, t3) = crossing(
        line_origin(&a.lines[1 - ka])?,
        a.d,
        o3,
        d3,
        "the first blend's contact through the third edge",
    )?;
    let (p3b, tpb, _) = crossing(
        line_origin(&b.lines[1 - kb])?,
        b.d,
        o3,
        d3,
        "the second blend's contact through the third edge",
    )?;
    if (p3 - p3b).norm() > tolerance {
        return Err(match a.section {
            // Equal dihedrals put two fillets' far contacts through one
            // point.
            Section::Round { .. } => {
                invariant("the two contacts through one point of the third edge")
            }
            // Two chamfers' meet there only when their edges make equal
            // angles with the third edge.
            Section::Flat => vertex_blend(),
        });
    }
    // The third edge shortened to that point.
    let too_large = || degenerate(vec![ea, eb, forward(e3)], Reason::BlendTooLarge);
    let Some(tc) = into_range(range3, t3, c3.period()) else {
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
    let (curve, range, q_first) = match (a.section, b.section) {
        (Section::Flat, Section::Flat) => {
            let (curve, range) = chord(q, p3, tol)?;
            (curve, range, true)
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
            let (tq, tp) = (param(q)?, param(p3)?);
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
            (curve, range, q_first)
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
    Ok(Miter {
        edges: [a.edge, b.edge],
        shared,
        t,
        q,
        p3,
        curve,
        range,
        q_first,
        lo_first,
        on_blend,
        trim,
        tolerance,
        q_tolerance: tolerance,
        p3_tolerance: tolerance.max(e3_entity.tolerance()),
    })
}

/// One side of a junction: a line's stripe or an arc's ring.
#[derive(Clone, Copy)]
enum Run<'a> {
    Line(&'a Stripe),
    Arc(&'a Ring),
}

/// A side of a junction read at its vertex.
struct RunAt<'a> {
    edge: EdgeId,
    /// The faces of the contacts, by contact.
    faces: [FaceId; 2],
    /// The contacts' points at the vertex, by contact.
    points: [Point3; 2],
    /// A fillet's ball centre there.
    centre: Option<Point3>,
    convex: bool,
    tolerance: f64,
    surface: &'a Surface,
    /// The edge's parameter there, which the contacts share.
    t: f64,
}

impl<'a> Run<'a> {
    fn at(self, m: &Model, vertex: VertexId) -> Result<RunAt<'a>, OpError> {
        match self {
            Run::Line(s) => {
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
                    },
                    convex: s.convex,
                    tolerance: s.tolerance,
                    surface: &s.surface,
                    t,
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
                })
            }
        }
    }

    /// `pcurve` on the blend placed in its loop's translate, its point at
    /// `lo` on contact `contact` at the junction `at`: on a stripe at that
    /// contact's `u`, on a ring at the contacts' `u` there and that
    /// contact's `v`.
    fn place(self, pcurve: Curve2, lo: f64, contact: usize, at: &RunAt<'_>) -> Curve2 {
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
/// every pcurve of it exact. Two runs whose points or ball centres differ
/// by more than their tolerance, or one convex and one concave, is an
/// internal fault the tangent-vertex test makes unreachable; a `w` shorter
/// than the cut is `Reason::BlendTooLarge`.
fn junction(
    m: &Model,
    view: &View,
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
            Reason::VertexBlend,
        )
    };
    let mut shared: Option<(usize, usize)> = None;
    for (ka, fa) in ra.faces.iter().enumerate() {
        for (kb, fb) in rb.faces.iter().enumerate() {
            if fa == fb && shared.replace((ka, kb)).is_some() {
                return Err(vertex_blend());
            }
        }
    }
    let Some((ka, kb)) = shared else {
        return Err(vertex_blend());
    };
    // The third edge, between the two faces the runs do not share.
    let at_vertex = view
        .vertex_edges
        .get(&vertex)
        .ok_or(invariant("the junction vertex's edges"))?;
    let third: Vec<EdgeId> = at_vertex
        .iter()
        .copied()
        .filter(|&e| e != ra.edge && e != rb.edge)
        .collect();
    let [w] = third[..] else {
        return Err(vertex_blend());
    };
    let faces_w: BTreeSet<FaceId> = view
        .uses
        .get(&w)
        .ok_or(invariant("the third edge's uses"))?
        .iter()
        .map(|u| u.face)
        .collect();
    if faces_w != BTreeSet::from([ra.faces[1 - ka], rb.faces[1 - kb]]) {
        return Err(vertex_blend());
    }
    let tolerance = ra.tolerance.max(rb.tolerance);
    if ra.convex != rb.convex {
        return Err(invariant("both runs of a junction convex or both concave"));
    }
    let meet = |pa: Point3, pb: Point3, what: &'static str| {
        if (pa - pb).norm() > tolerance {
            return Err(invariant(what));
        }
        Ok(pa + (pb - pa) * 0.5)
    };
    let q = meet(
        ra.points[ka],
        rb.points[kb],
        "the contacts on the shared face through one point",
    )?;
    let p = meet(
        ra.points[1 - ka],
        rb.points[1 - kb],
        "the other contacts through one point of the third edge",
    )?;
    let trim = cut_corner(m, ra.edge, w, vertex, p)?;
    let (curve, range) = match (ra.centre, rb.centre) {
        (None, None) => chord(q, p, tol)?,
        (Some(ca), Some(cb)) => {
            let centre = meet(ca, cb, "the runs' balls one ball")?;
            let (x, y) = (q - centre, p - centre);
            let radius = x.norm();
            if (y.norm() - radius).abs() > tolerance {
                return Err(invariant("the ball touching both faces across"));
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
        tolerance,
        q_tolerance: tolerance,
        p3_tolerance: tolerance.max(m.edge(w)?.tolerance()),
    })
}

/// The corner of stripes `stripes` at `vertex`, a vertex of these three
/// edges and no other (ADR-0007). Every face there a plane and every blend
/// convex or every one concave is what puts the three fillets' axes
/// through one centre; a fillet corner also needs a face square to the
/// other two, so that its sides are the sphere's equator and two meridians
/// with exact pcurves. A corner that is not all planes, of mixed blends or,
/// for fillets, with no such face is `Reason::VertexBlend`, the
/// blend-network cycle's.
fn corner(
    m: &Model,
    view: &View,
    stripes: [&Stripe; 3],
    vertex: VertexId,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Corner, OpError> {
    meter.tick()?;
    let edges = stripes.map(|s| s.edge);
    let vertex_blend = || {
        let entities = edges
            .iter()
            .map(|&e| forward(e))
            .chain([forward(vertex)])
            .collect();
        degenerate(entities, Reason::VertexBlend)
    };
    let at_vertex = view
        .vertex_edges
        .get(&vertex)
        .ok_or(invariant("the corner vertex's edges"))?;
    if *at_vertex != edges.iter().copied().collect::<BTreeSet<EdgeId>>() {
        return Err(vertex_blend());
    }
    // A blend along a ruling has a cylinder face at the corner.
    if stripes
        .iter()
        .any(|s| s.ruling || s.convex != stripes[0].convex)
    {
        return Err(vertex_blend());
    }
    // The corner's faces, in the blends' order, and the point each
    // blend's contacts end at: the one on that contact's face.
    let mut found: Vec<FaceId> = Vec::with_capacity(3);
    for s in &stripes {
        for &f in &s.faces {
            if !found.contains(&f) {
                found.push(f);
            }
        }
    }
    let faces: [FaceId; 3] = found.try_into().map_err(|_| vertex_blend())?;
    let mut contact_point = [[0usize; 2]; 3];
    for (points, s) in contact_point.iter_mut().zip(&stripes) {
        for (point, face) in points.iter_mut().zip(&s.faces) {
            *point = faces
                .iter()
                .position(|f| f == face)
                .ok_or(invariant("a corner face"))?;
        }
    }
    let tolerance = stripes.iter().map(|s| s.tolerance).fold(0.0, f64::max);
    let crossing = |o1: Point3, d1: Vec3, o2: Point3, d2: Vec3, what: &'static str| {
        let (t1, t2) = lines_cross(o1, d1, o2, d2, tol).ok_or_else(vertex_blend)?;
        let (p1, p2) = (o1 + t1 * d1, o2 + t2 * d2);
        if (p1 - p2).norm() > tolerance {
            return Err(invariant(what));
        }
        Ok((p1 + (p2 - p1) * 0.5, t1, t2))
    };
    // Where the two contacts on each face cross.
    let mut points = [Point3::origin(); 3];
    let mut t = [[0.0; 2]; 3];
    for (p, point) in points.iter_mut().enumerate() {
        let on: Vec<(usize, usize)> = (0..3)
            .flat_map(|side| [(side, 0), (side, 1)])
            .filter(|&(side, k)| contact_point[side][k] == p)
            .collect();
        let [(sa, ka), (sb, kb)] = on[..] else {
            return Err(vertex_blend());
        };
        let (a, b) = (stripes[sa], stripes[sb]);
        let (q, ta, tb) = crossing(
            line_origin(&a.lines[ka])?,
            a.d,
            line_origin(&b.lines[kb])?,
            b.d,
            "the contacts on a corner face through one point",
        )?;
        *point = q;
        t[sa][ka] = ta;
        t[sb][kb] = tb;
    }
    // The side between two of the corner's points.
    let side_of = |a: usize, b: usize| {
        contact_point
            .iter()
            .position(|c| c.contains(&a) && c.contains(&b))
            .ok_or_else(vertex_blend)
    };
    let arc_tol = Tolerance::new(tolerance, tol.angular);
    let geometry = fault_of;
    // The side `side` over `curve`, from point `ends[0]` to `ends[1]`: its
    // pcurve on that blend placed as its contacts are.
    let arc = |side: usize,
               curve: Curve,
               range: Interval,
               ends: [usize; 2],
               on_corner: Curve2,
               along: bool,
               meter: &mut Meter<'_>|
     -> Result<CornerArc, OpError> {
        let s = stripes[side];
        let lo_first = ends[0] == contact_point[side][0];
        let on_blend = pcurve_on(&curve, range, &s.surface, arc_tol, meter).map_err(geometry)?;
        let on_blend = s.place(on_blend, range.lo(), if lo_first { 0.0 } else { s.u1 });
        Ok(CornerArc {
            curve,
            range,
            ends,
            lo_first,
            on_blend,
            on_corner,
            along,
        })
    };
    let (surface, orientation, sides, pole) = match stripes[0].section {
        Section::Round { radius, .. } => {
            let axis = |s: &Stripe| match s.section {
                Section::Round { axis_origin, .. } => Ok(axis_origin),
                Section::Flat => Err(invariant("one kind of blend in one call")),
            };
            let [a, b, c] = stripes;
            let (centre, _, _) = crossing(
                axis(a)?,
                a.d,
                axis(b)?,
                b.d,
                "the three axes through the ball's centre",
            )?;
            let off = centre - axis(c)?;
            if (off - off.dot(&c.d) * c.d).norm() > tolerance {
                return Err(invariant("the three axes through the ball's centre"));
            }
            // From the centre toward each point, where the ball touches
            // that face.
            let mut w = [Vec3::zeros(); 3];
            for (wp, p) in w.iter_mut().zip(points) {
                let r = p - centre;
                if (r.norm() - radius).abs() > tolerance {
                    return Err(invariant("the ball touching each corner face at its point"));
                }
                *wp = r / r.norm();
            }
            // The pole: the point of a face square to the other two, taken
            // off the first blend that has one across it. Square is read
            // off the planes' own normals, which `w` is up to sign: `w`
            // comes through points as far out as the body and a radius as
            // small as the blend, and a body 100 out with a blend of 0.01
            // puts it 2e-12 off a normal — past the angular tolerance.
            let mut normal = [Vec3::zeros(); 3];
            for (n, &f) in normal.iter_mut().zip(&faces) {
                *n = match m.surface(m.face(f)?.surface())? {
                    Surface::Plane { frame } => frame.z().into_inner(),
                    Surface::Cylinder { .. }
                    | Surface::EllipticCylinder { .. }
                    | Surface::Cone { .. }
                    | Surface::Sphere { .. }
                    | Surface::Torus { .. }
                    | Surface::Nurbs(_) => return Err(vertex_blend()),
                };
            }
            let mut pole_at = None;
            for ends in &contact_point {
                let p = (0..3)
                    .find(|q| !ends.contains(q))
                    .ok_or(invariant("a corner face off each blend"))?;
                if ends
                    .iter()
                    .all(|&q| normal[p].dot(&normal[q]).abs() <= tol.angular)
                {
                    pole_at = Some((p, *ends));
                    break;
                }
            }
            let Some((p, [i0, i1])) = pole_at else {
                return Err(vertex_blend());
            };
            // `X` toward one equator point, `Y` toward the other.
            let (i, j) = if w[p].cross(&w[i0]).dot(&w[i1]) > 0.0 {
                (i0, i1)
            } else {
                (i1, i0)
            };
            let frame = Frame::new(centre, w[p], w[i])?;
            let gamma = w[j]
                .dot(&frame.y().into_inner())
                .atan2(w[j].dot(&frame.x().into_inner()));
            let equator_range =
                Interval::new(0.0, gamma).map_err(|_| invariant("a corner of positive angle"))?;
            let quarter =
                Interval::new(0.0, FRAC_PI_2).map_err(|_| invariant("a quarter of a turn"))?;
            let sphere = Surface::Sphere { frame, radius };
            let on_sphere = |curve: &Curve,
                             range: Interval,
                             u: f64,
                             meter: &mut Meter<'_>|
             -> Result<Curve2, OpError> {
                let pcurve = pcurve_on(curve, range, &sphere, arc_tol, meter).map_err(geometry)?;
                Ok(placed(pcurve, range.lo(), u))
            };
            // Each meridian from its equator point up to the pole.
            let meridian = |q: usize| -> Result<Curve, OpError> {
                Ok(Curve::Circle {
                    frame: Frame::new(centre, w[q].cross(&w[p]), w[q])?,
                    radius,
                })
            };
            let equator = Curve::Circle { frame, radius };
            let (to_j, to_i) = (meridian(j)?, meridian(i)?);
            // In (u, v), counter-clockwise: along the equator from `i` to
            // `j`, up the meridian at `u = γ`, back along the pole, down
            // the meridian at `u = 0`.
            let sides = vec![
                (
                    side_of(i, j)?,
                    arc(
                        side_of(i, j)?,
                        equator.clone(),
                        equator_range,
                        [i, j],
                        on_sphere(&equator, equator_range, 0.0, meter)?,
                        true,
                        meter,
                    )?,
                ),
                (
                    side_of(j, p)?,
                    arc(
                        side_of(j, p)?,
                        to_j.clone(),
                        quarter,
                        [j, p],
                        on_sphere(&to_j, quarter, gamma, meter)?,
                        true,
                        meter,
                    )?,
                ),
                (
                    side_of(i, p)?,
                    arc(
                        side_of(i, p)?,
                        to_i.clone(),
                        quarter,
                        [i, p],
                        on_sphere(&to_i, quarter, 0.0, meter)?,
                        false,
                        meter,
                    )?,
                ),
            ];
            let pole = Pole {
                point: p,
                range: equator_range,
                pcurve: Curve2::Line {
                    origin: Point2::new(0.0, FRAC_PI_2),
                    direction: UnitVec2::new_unchecked(Vec2::new(1.0, 0.0)),
                },
                after: 1,
            };
            let orientation = if stripes[0].convex {
                Orientation::Forward
            } else {
                Orientation::Reversed
            };
            (sphere, orientation, sides, Some(pole))
        }
        Section::Flat => {
            // The triangle faces out of the material, which every corner
            // face's outward normal points away from.
            let mut outward = Vec3::zeros();
            for &f in &faces {
                outward += view.outward(m, f, Point2::origin())?;
            }
            let turn = |order: [usize; 3]| {
                (points[order[1]] - points[order[0]]).cross(&(points[order[2]] - points[order[0]]))
            };
            let order = if turn([0, 1, 2]).dot(&outward) >= 0.0 {
                [0, 1, 2]
            } else {
                [0, 2, 1]
            };
            let normal = UnitVec3::try_new(turn(order), 0.0)
                .ok_or(invariant("a corner triangle of positive area"))?;
            let frame = Frame::new(
                points[order[0]],
                normal.into_inner(),
                points[order[1]] - points[order[0]],
            )?;
            let plane = Surface::Plane { frame };
            let mut sides = Vec::with_capacity(3);
            for w in 0..3 {
                let (from, to) = (order[w], order[(w + 1) % 3]);
                let side = side_of(from, to)?;
                let (curve, range) = chord(points[from], points[to], tol)?;
                let on_corner =
                    pcurve_on(&curve, range, &plane, arc_tol, meter).map_err(geometry)?;
                sides.push((
                    side,
                    arc(side, curve, range, [from, to], on_corner, true, meter)?,
                ));
            }
            (plane, Orientation::Forward, sides, None)
        }
    };
    let walk: [usize; 3] = sides
        .iter()
        .map(|(side, _)| *side)
        .collect::<Vec<_>>()
        .try_into()
        .map_err(|_| invariant("three sides of a corner"))?;
    let mut by_side: Vec<(usize, CornerArc)> = sides;
    by_side.sort_by_key(|(side, _)| *side);
    if by_side.iter().enumerate().any(|(i, (side, _))| *side != i) {
        return Err(vertex_blend());
    }
    let arcs: [CornerArc; 3] = by_side
        .into_iter()
        .map(|(_, arc)| arc)
        .collect::<Vec<_>>()
        .try_into()
        .map_err(|_| invariant("three sides of a corner"))?;
    Ok(Corner {
        edges,
        faces,
        points,
        contact_point,
        t,
        arcs,
        walk,
        pole,
        surface,
        orientation,
        tolerance,
    })
}

/// A face of the meridian row (ADR-0036 §1): `Some(None)` for a plane,
/// `Some(Some(frame))` for a cylinder, a cone, a sphere or a torus, whose
/// `Z` is the axis when the face is coaxial with the edge, and `None` for
/// any other surface.
fn row_frame(surface: &Surface) -> Option<Option<Frame>> {
    match *surface {
        Surface::Plane { .. } => Some(None),
        Surface::Cylinder { frame, .. }
        | Surface::Cone { frame, .. }
        | Surface::Sphere { frame, .. }
        | Surface::Torus { frame, .. } => Some(Some(frame)),
        Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => None,
    }
}

/// A contact of a circular edge's blend: a circle about the axis on one of the edge's faces, the edge's own frame moved along the
/// axis and its radius changed, so it runs as the edge ran over the same
/// range.
struct RingContact {
    face: FaceId,
    curve: Curve,
    /// The edge's range, or where an end off the axis trims this contact
    /// apart from the other (ADR-0037).
    range: Interval,
    /// Its points at the range's start and end, one vertex on a closed
    /// edge.
    points: [Point3; 2],
    on_face: Curve2,
    /// Its pcurve on the blend, a line at constant `v` placed so that the
    /// range's start is at `u = 0` or `u = 2π`.
    on_blend: Curve2,
    /// `true` when it runs with the blend's `u`.
    along_u: bool,
}

/// One end of an open arc's blend (ADR-0035), trimmed by a plane through
/// the axis — the meridian of the torus there or the ruling of the cone —
/// or by a face off the axis, a plane parallel to it or a cylinder about
/// a parallel axis, on the section traced and fitted (ADR-0037); between
/// the two contacts' points, inserted into that face's loop between the
/// two corner edges it shortens.
struct ArcEnd {
    vertex: VertexId,
    face: FaceId,
    /// The corner edge each contact's point cuts, by contact.
    trims: [Trim; 2],
    curve: Curve,
    range: Interval,
    /// `true` when `range.lo()` is at the first contact's point.
    lo_first: bool,
    /// Its pcurve on the face across, placed in that face loop's translate.
    on_face: Curve2,
    /// Its pcurve on the blend, placed: a line at constant `u` on the
    /// section through the axis.
    on_blend: Curve2,
    tolerance: f64,
    /// The tolerance of each trim vertex, by contact.
    vertex_tolerance: [f64; 2],
}

/// A closed edge's blend seam, from the first contact's vertex to the
/// second's, with its pcurves at `u = 0` then at `u = 2π`, and each
/// curved face's own seam shortened to its contact.
struct RingSeam {
    curve: Curve,
    range: Interval,
    on_blend: [Curve2; 2],
    /// Each curved face's seam, and the contact whose vertex it is cut at.
    cuts: Vec<(Trim, usize)>,
    vertex_tolerance: f64,
}

/// One end of an open arc's blend: trimmed by the face across, or a
/// junction at a tangent vertex, whose arc and vertices the junction holds
/// (ADR-0035 §3).
enum RingEnd {
    Face(Box<ArcEnd>),
    Junction(VertexId),
}

/// How a circular edge's blend closes: a closed edge's runs a whole turn
/// round to its own seam, an open arc's ends at its two vertices.
enum RingEnds {
    Seam(Box<RingSeam>),
    /// At the edge's start, then its end.
    Open(Box<[RingEnd; 2]>),
}

/// The blend of a circular edge where two faces of revolution about one
/// axis meet along a parallel — a plane square to it, a cylinder, a cone, a
/// sphere centred on it or a torus (ADR-0007, ADR-0035, ADR-0036): a torus
/// coaxial with the curved face, or a cone, over the edge's own range. The
/// blend's frame has the curved face's `Z` and its `X` at the edge's start
/// vertex. A closed edge has no ends, its `u` seam — a tube circle of the
/// torus, a ruling of the cone — running from one contact's vertex to the
/// other's through the half-plane the curved face's own seam lies in,
/// which is shortened to its contact; an open arc is trimmed at each end by the
/// face across, a plane through the axis, on the same tube circle or
/// ruling at the vertex's angle.
struct Ring {
    edge: EdgeId,
    /// The edge's range, which both contacts keep.
    range: Interval,
    surface: Surface,
    orientation: Orientation,
    convex: bool,
    /// A fillet's circle of the ball's centre, its frame the edge's
    /// moved along the axis so that it runs as the edge runs; `None` on a
    /// chamfer.
    centres: Option<Curve>,
    /// The contacts at the blend's lower `v`, then its upper.
    contacts: [RingContact; 2],
    ends: RingEnds,
    tolerance: f64,
}

/// A face of revolution's meridian in the half-plane bounded by the axis,
/// `(ρ, h)`, the distance from the axis and the height along it from the
/// edge's centre: a line for a plane, a cylinder or a cone, a circle for a
/// sphere centred on the axis or a coaxial torus (ADR-0036 §1).
#[derive(Debug, Clone, Copy)]
enum Trace {
    /// The line through `point` along the unit `along`.
    Line { point: Vec2, along: Vec2 },
    /// The circle about `centre` of `radius`.
    Circle { centre: Vec2, radius: f64 },
}

impl Trace {
    /// Where the two traces cross nearest `near`; `None` where they do
    /// not, or run parallel.
    fn crossing(&self, other: &Trace, near: Vec2) -> Option<Vec2> {
        let cross = |a: Vec2, b: Vec2| a.x * b.y - a.y * b.x;
        let nearest = |roots: [Vec2; 2]| {
            if (roots[0] - near).norm() <= (roots[1] - near).norm() {
                roots[0]
            } else {
                roots[1]
            }
        };
        match (*self, *other) {
            (Trace::Line { point: p, along: a }, Trace::Line { point: q, along: b }) => {
                let det = cross(a, b);
                if det == 0.0 {
                    return None;
                }
                Some(p + cross(q - p, b) / det * a)
            }
            (Trace::Line { point, along }, Trace::Circle { centre, radius })
            | (Trace::Circle { centre, radius }, Trace::Line { point, along }) => {
                // `|point + t along − centre|² = radius²`, `along` a unit.
                let w = point - centre;
                let b = along.dot(&w);
                let disc = b * b - (w.norm_squared() - radius * radius);
                if disc < 0.0 {
                    return None;
                }
                let root = disc.sqrt();
                Some(nearest([
                    point + (-b - root) * along,
                    point + (-b + root) * along,
                ]))
            }
            (
                Trace::Circle {
                    centre: c1,
                    radius: r1,
                },
                Trace::Circle {
                    centre: c2,
                    radius: r2,
                },
            ) => {
                let w = c2 - c1;
                let d = w.norm();
                if d == 0.0 {
                    return None;
                }
                let along = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
                let across = r1 * r1 - along * along;
                if across < 0.0 {
                    return None;
                }
                let u = w / d;
                let foot = c1 + along * u;
                let side = across.sqrt() * Vec2::new(-u.y, u.x);
                Some(nearest([foot + side, foot - side]))
            }
        }
    }
}

/// A face of the meridian row read in the section (ADR-0036 §1): its
/// trace, and its outward normal at the edge.
#[derive(Debug, Clone, Copy)]
struct Meridian {
    trace: Trace,
    /// The face's outward normal in the section at the edge, a unit
    /// vector; on a line, its normal everywhere.
    normal: Vec2,
}

impl Meridian {
    /// The trace's unit tangent at the edge, either way along it.
    fn along(&self) -> Vec2 {
        match self.trace {
            Trace::Line { along, .. } => along,
            Trace::Circle { .. } => Vec2::new(-self.normal.y, self.normal.x),
        }
    }

    /// The foot of `p` on the trace; `None` at a circle's centre.
    fn foot(&self, p: Vec2) -> Option<Vec2> {
        match self.trace {
            Trace::Line { point, along } => Some(point + (p - point).dot(&along) * along),
            Trace::Circle { centre, radius } => {
                let w = p - centre;
                let d = w.norm();
                (d > 0.0).then(|| centre + radius / d * w)
            }
        }
    }

    /// The trace moved `offset` along the face's outward normal; `None` for
    /// a circle shrunk to no radius.
    fn offset(&self, edge: Vec2, offset: f64) -> Option<Trace> {
        match self.trace {
            Trace::Line { point, along } => Some(Trace::Line {
                point: point + offset * self.normal,
                along,
            }),
            Trace::Circle { centre, radius } => {
                // `+1` where the outward normal points away from the centre.
                let away = self.normal.dot(&(edge - centre)).signum();
                let radius = radius + away * offset;
                (radius > 0.0).then_some(Trace::Circle { centre, radius })
            }
        }
    }

    /// The trace's direction away from the corner into its own face, where
    /// `other` is the face across the edge: the way `other`'s outward
    /// normal reads `s`, `−1` at a convex corner and `1` at a concave one.
    fn inward(&self, other: &Meridian, s: f64) -> Vec2 {
        let along = self.along();
        along * (s * other.normal.dot(&along)).signum()
    }
}

/// `pcurve` translated by whole turns in `u` and `v` so that its point at
/// `t` is nearest `target`: a periodic surface's pcurve placed in the
/// translate of the blend's loop.
fn placed_uv(pcurve: Curve2, t: f64, target: Point2) -> Curve2 {
    let k = ((target - pcurve.point(t)) / TAU).map(f64::round);
    if k == Vec2::zeros() {
        pcurve
    } else {
        pcurve.translated(k * TAU)
    }
}

/// The face across an open arc's end, read against the arc's axis `z`
/// through `centre`: a plane through the axis, which cuts the blend in its
/// section at the vertex's angle (ADR-0035); a plane parallel to the axis
/// and off it, or a cylinder about an axis parallel to it and off it, which
/// each contact meets at its own angle and the blend in a section traced
/// and fitted (ADR-0037); `None` for any other face.
fn across_of(
    surface: &Surface,
    centre: Point3,
    z: Vec3,
    tol: Tolerance,
    tolerance: f64,
) -> Option<Across> {
    match *surface {
        Surface::Plane { frame } => {
            let n = frame.z().into_inner();
            if n.dot(&z).abs() > tol.angular {
                return None;
            }
            let offset = (frame.origin() - centre).dot(&n);
            Some(if offset.abs() <= tolerance {
                Across::Axis
            } else {
                Across::Wall {
                    point: frame.origin(),
                    normal: n,
                }
            })
        }
        Surface::Cylinder { frame, radius } => {
            let w: Vec3 = frame.z().into_inner();
            let off = frame.origin() - centre;
            let apart = off - off.dot(&z) * z;
            (w.cross(&z).norm() <= tol.angular && apart.norm() > tolerance).then_some(
                Across::Post {
                    point: frame.origin(),
                    radius,
                },
            )
        }
        Surface::EllipticCylinder { .. }
        | Surface::Cone { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => None,
    }
}

/// How the face across an open arc's end meets the arc's axis
/// ([`across_of`]).
#[derive(Debug, Clone, Copy)]
enum Across {
    /// A plane through the axis.
    Axis,
    /// A plane parallel to the axis and off it, through `point` with unit
    /// `normal`.
    Wall { point: Point3, normal: Vec3 },
    /// A cylinder of `radius` about an axis parallel to the arc's through
    /// `point`, off it.
    Post { point: Point3, radius: f64 },
}

impl Across {
    /// Where the circle of `radius` about the axis `z` through `centre`,
    /// square to it, meets the face across off the axis — a line in the
    /// circle's plane for a wall, a circle for a post, whose radical line
    /// with the contact's circle the crossings lie on — the crossing on the
    /// side of the line through `centre` square to that one that `vertex`
    /// is on, where the edge itself meets the face. `None` where the circle
    /// misses or grazes the face within `tolerance`, or the vertex is on
    /// that line, and on a plane through the axis, whose end is the
    /// section's.
    fn meet(
        &self,
        centre: Point3,
        z: Vec3,
        radius: f64,
        vertex: Point3,
        tolerance: f64,
    ) -> Option<Point3> {
        let flat = |v: Vec3| v - v.dot(&z) * z;
        // The line the crossings lie on: its unit normal `n` in the plane
        // and its distance `offset` from `centre` along it.
        let (n, offset) = match *self {
            Across::Axis => return None,
            Across::Wall { point, normal } => {
                let n = flat(normal).normalize();
                (n, (point - centre).dot(&n))
            }
            Across::Post {
                point,
                radius: post,
            } => {
                let apart = flat(point - centre);
                let d = apart.norm();
                (
                    apart / d,
                    (radius * radius - post * post + d * d) / (2.0 * d),
                )
            }
        };
        let half = radius * radius - offset * offset;
        if half <= 0.0 || half.sqrt() <= tolerance {
            return None;
        }
        let w = z.cross(&n);
        let side = (vertex - centre).dot(&w);
        if side.abs() <= tolerance {
            return None;
        }
        Some(centre + offset * n + side.signum() * half.sqrt() * w)
    }
}

/// The blend of a circular edge `edge` where two faces of the meridian row
/// meet, read in the half-plane bounded by the axis, where each face's
/// meridian is a line or a circle (ADR-0036 §1). The ball's centre is
/// where the two meridians offset by `r` into the ball's side cross — of
/// two crossings, the nearer the edge — and a fillet is the torus of the
/// centre's distance from the axis and minor `r`, the arc of its tube
/// between the feet of the centre on the two meridians; a chamfer is the
/// cone through the two circles at `d` from the edge along each line
/// meridian, or at the chord `d` on a circle (§2). Against a cylinder the centre circle is at
/// radius `R + sσr` — `s` `−1` on a convex edge, `σ` the side of the axis
/// the cylinder's outward normal points to — and the chamfer is at 45°.
/// A closed edge's one vertex is on the curved face's seam and on nothing
/// else; an open arc's two vertices are each a corner
/// of three edges whose face across is a plane through the axis, a plane
/// parallel to it or a cylinder about a parallel axis — each contact then
/// trimmed where it meets that face, the end traced and fitted (ADR-0037),
/// a corner edge lengthened past the vertex at a mixed corner (ADR-0038)
/// — or one of the `junctions`, which the junction builds. A torus that is not a ring
/// torus and a contact that reaches the axis or a cone's apex are
/// `Reason::BlendTooLarge`,
/// as is a contact or an end that leaves its face or a seam or a corner
/// edge shorter than the trim.
#[allow(clippy::too_many_arguments)]
fn ring(
    m: &Model,
    view: &View,
    edge: EdgeId,
    kind: Kind,
    junctions: &BTreeSet<VertexId>,
    tol: Tolerance,
    samples: usize,
    meter: &mut Meter<'_>,
) -> Result<Ring, OpError> {
    meter.tick()?;
    let e = forward(edge);
    let entity = *m.edge(edge)?;
    let vertex = entity.start();
    let vertex_blend = || degenerate(vec![e, forward(vertex)], Reason::VertexBlend);
    let Some((curve_id, range)) = entity.curve() else {
        return Err(vertex_blend());
    };
    let uses = view
        .uses
        .get(&edge)
        .filter(|u| u.len() == 2)
        .ok_or(invariant("two uses of the blended edge"))?;
    let (ua, ub) = (uses[0], uses[1]);
    let faces = [ua.face, ub.face];
    let [f1, f2] = faces;
    // The dihedral, read at the edge's midpoint as a line edge's is: a
    // tangent one has no corner to roll a ball into, whatever the surfaces
    // are.
    let mid = range.midpoint();
    let pcurves = [m.curve2(ua.pcurve)?, m.curve2(ub.pcurve)?];
    let n1 = view.outward(m, f1, pcurves[0].point(mid))?;
    let n2 = view.outward(m, f2, pcurves[1].point(mid))?;
    if n1.cross(&n2).norm() <= tol.angular {
        return Err(degenerate(
            vec![e, forward(f1), forward(f2)],
            Reason::TangentChain,
        ));
    }
    let surfaces = [
        m.surface(m.face(f1)?.surface())?,
        m.surface(m.face(f2)?.surface())?,
    ];
    let unsupported = || OpError::Unsupported {
        a: (GeomKind::Surface(surfaces[0].kind()), forward(f1)),
        b: (GeomKind::Surface(surfaces[1].kind()), forward(f2)),
    };
    // The meridian row (ADR-0036 §1): two of a plane square to the axis, a
    // cylinder, a cone, a sphere and a torus, at most one a plane — each
    // curved face's frame, whose `Z` is the axis.
    let frames = [row_frame(surfaces[0]), row_frame(surfaces[1])];
    let [Some(fa), Some(fb)] = frames else {
        return Err(unsupported());
    };
    let Some(axis) = fa.or(fb) else {
        return Err(unsupported());
    };
    let k = if fa.is_some() { 0 } else { 1 };
    let curve = m.curve(curve_id)?;
    let &Curve::Circle {
        frame: rim,
        radius: rim_radius,
    } = curve
    else {
        return Err(OpError::Unsupported {
            a: (GeomKind::Curve(curve.kind()), e),
            b: (GeomKind::Surface(surfaces[k].kind()), forward(faces[k])),
        });
    };
    let tolerance = m
        .precision()
        .default_tolerance
        .max(m.face(f1)?.tolerance())
        .max(m.face(f2)?.tolerance());
    let z: Vec3 = axis.z().into_inner();
    for i in 0..2 {
        let Some(frame) = frames[i].flatten() else {
            continue;
        };
        let centre = frame.to_local(rim.origin());
        if rim.z().cross(&frame.z()).norm() > tol.angular || centre.x.hypot(centre.y) > tolerance {
            // A circle on a cylinder or a cone is one of its parallels; on
            // a sphere or a torus it may be a circle about another axis —
            // a sphere's tilted to its frame, a torus's meridian — which
            // is not this row.
            return Err(match surfaces[i] {
                Surface::Sphere { .. } | Surface::Torus { .. } => unsupported(),
                Surface::Plane { .. }
                | Surface::Cylinder { .. }
                | Surface::Cone { .. }
                | Surface::EllipticCylinder { .. }
                | Surface::Nurbs(_) => {
                    invariant("a circular edge of a face of revolution about its axis")
                }
            });
        }
    }
    let eval = curve.eval(mid);
    let t1 = eval.d1 * view.orientation[&f1].compose(ua.orientation).sign();
    let convex = n1.cross(&t1).dot(&n2) < 0.0;
    let s = if convex { -1.0 } else { 1.0 };
    let normals = [n1, n2];
    let radial_at = |t: f64| -> Result<Vec3, OpError> {
        UnitVec3::try_new(curve.point(t) - rim.origin(), tol.linear)
            .map(UnitVec3::into_inner)
            .ok_or(invariant("a circular edge off its axis"))
    };
    // The section in the half-plane through the axis at the edge's
    // midpoint, `(ρ, h)` from the edge's centre: each face's meridian.
    let radial_mid = radial_at(mid)?;
    let edge_at = Vec2::new(rim_radius, 0.0);
    let height = |p: Point3| (p - rim.origin()).dot(&z);
    let mut meridians: Vec<Meridian> = Vec::with_capacity(2);
    for i in 0..2 {
        // The face's outward normal at the edge, in the section.
        let read = Vec2::new(normals[i].dot(&radial_mid), normals[i].dot(&z));
        // A circle meridian's outward normal at the edge: from its centre,
        // or toward it.
        let round = |centre: Vec2, radius: f64| {
            let away = (edge_at - centre) / radius;
            Meridian {
                trace: Trace::Circle { centre, radius },
                normal: away * read.dot(&away).signum(),
            }
        };
        meridians.push(match *surfaces[i] {
            Surface::Plane { .. } => Meridian {
                trace: Trace::Line {
                    point: Vec2::zeros(),
                    along: Vec2::new(1.0, 0.0),
                },
                normal: Vec2::new(0.0, normals[i].dot(&z).signum()),
            },
            Surface::Cylinder { radius, .. } => Meridian {
                trace: Trace::Line {
                    point: Vec2::new(radius, 0.0),
                    along: Vec2::new(0.0, 1.0),
                },
                normal: Vec2::new(normals[i].dot(&radial_mid).signum(), 0.0),
            },
            Surface::Cone {
                frame,
                radius,
                half_angle,
            } => {
                // The ruling at `u`: `(R + v sin α, h₀ ± v cos α)`, the sign
                // of the cone's `Z` against the axis, its normal read off
                // the face's outward one.
                let (sa, ca) = half_angle.sin_cos();
                let up = frame.z().dot(&axis.z()).signum();
                let normal = Vec2::new(up * ca, -sa);
                Meridian {
                    trace: Trace::Line {
                        point: Vec2::new(radius, height(frame.origin())),
                        along: Vec2::new(sa, up * ca),
                    },
                    normal: normal * read.dot(&normal).signum(),
                }
            }
            // A sphere centred on the axis: the circle of its radius about
            // its centre; a torus: its tube circle in the half-plane.
            Surface::Sphere { frame, radius } => {
                round(Vec2::new(0.0, height(frame.origin())), radius)
            }
            Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => round(
                Vec2::new(major_radius, height(frame.origin())),
                minor_radius,
            ),
            Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => {
                return Err(invariant("a face of revolution in the row"));
            }
        });
    }
    let [mer_a, mer_b]: [Meridian; 2] = meridians
        .try_into()
        .map_err(|_| invariant("two meridians"))?;
    // Two lines cross at the edge; a circle meridian passes through it.
    let corner = match (mer_a.trace, mer_b.trace) {
        (Trace::Line { .. }, Trace::Line { .. }) => mer_a
            .trace
            .crossing(&mer_b.trace, edge_at)
            .ok_or(invariant("two meridians crossing at the edge"))?,
        (Trace::Circle { .. }, _) | (_, Trace::Circle { .. }) => edge_at,
    };
    let lines = matches!(
        (mer_a.trace, mer_b.trace),
        (Trace::Line { .. }, Trace::Line { .. })
    );
    // Along each meridian into its face: the way the other face's outward
    // normal reads `s`.
    let d_a = mer_a.inward(&mer_b, s);
    let d_b = mer_b.inward(&mer_a, s);
    let too_large = |face: FaceId| degenerate(vec![e, forward(face)], Reason::BlendTooLarge);
    // The ball's centre is `r` off both meridians on the ball's side, each
    // fillet contact the foot of it on its meridian; a chamfer's contacts
    // are `d` along each from the corner. A contact at the axis, or past a
    // cone's apex, has no circle.
    let contacts_on = |on_a: Vec2, on_b: Vec2| {
        if on_a.x <= tolerance {
            Err(too_large(faces[0]))
        } else if on_b.x <= tolerance {
            Err(too_large(faces[1]))
        } else {
            Ok((on_a, on_b))
        }
    };
    let x = radial_at(range.lo())?;
    let at_height = |h: f64| rim.origin() + h * z;
    let circle = |at: Vec2| -> Result<Curve, OpError> {
        Ok(Curve::Circle {
            frame: Frame::new(at_height(at.y), rim.z().into_inner(), rim.x().into_inner())?,
            radius: at.x,
        })
    };
    // The blend, the contacts in the section, each contact's `v` on the
    // blend, and the centre's circle.
    let (surface, on_a, on_b, v_a, v_b, centres) = match kind {
        Kind::Fillet { radius } => {
            let c = if lines {
                let along =
                    s * radius * (1.0 - mer_a.normal.dot(&mer_b.normal)) / mer_b.normal.dot(&d_a);
                corner + along * d_a + s * radius * mer_a.normal
            } else {
                // Each meridian offset by `r` to the ball's side, a circle
                // shrunk to nothing or two that miss being no ball; of
                // their two crossings the centre is the one the corner is
                // the limit of as `r` goes to `0`, the nearer.
                let offset = |i: usize, mer: &Meridian| {
                    mer.offset(edge_at, s * radius)
                        .ok_or_else(|| too_large(faces[i]))
                };
                offset(0, &mer_a)?
                    .crossing(&offset(1, &mer_b)?, corner)
                    .ok_or_else(|| too_large(faces[k]))?
            };
            let foot = |i: usize, mer: &Meridian| mer.foot(c).ok_or_else(|| too_large(faces[i]));
            let (on_a, on_b) = contacts_on(foot(0, &mer_a)?, foot(1, &mer_b)?)?;
            if c.x <= radius + tolerance {
                return Err(too_large(faces[k]));
            }
            // Each contact's angle about the tube's centre, the two taken
            // the short way round.
            let angle = |p: Vec2| {
                let v = (p.y - c.y).atan2(p.x - c.x);
                if v < 0.0 { v + TAU } else { v }
            };
            let (mut v_a, mut v_b) = (angle(on_a), angle(on_b));
            if (v_a - v_b).abs() > PI {
                if v_a < v_b {
                    v_a += TAU;
                } else {
                    v_b += TAU;
                }
            }
            (
                Surface::Torus {
                    frame: Frame::new(at_height(c.y), z, x)?,
                    major_radius: c.x,
                    minor_radius: radius,
                },
                on_a,
                on_b,
                v_a,
                v_b,
                Some(circle(c)?),
            )
        }
        Kind::Chamfer { distance } => {
            // `d` from the corner along a line meridian; on a circle, the
            // chord of `d` into the face (ADR-0036 §2).
            let leg = |i: usize, mer: &Meridian, d: Vec2| match mer.trace {
                Trace::Line { .. } => Ok(corner + distance * d),
                Trace::Circle { .. } => Trace::Circle {
                    centre: corner,
                    radius: distance,
                }
                .crossing(&mer.trace, corner + distance * d)
                .ok_or_else(|| too_large(faces[i])),
            };
            let (on_a, on_b) = contacts_on(leg(0, &mer_a, d_a)?, leg(1, &mer_b, d_b)?)?;
            // The cone through the two contacts, its `Z` toward its wider
            // circle, its `v` from the narrower along a ruling. Two line
            // meridians that are not tangent leave the chord oblique to the
            // axis when one is a plane's; a circle meridian can put it
            // square to the axis or along it, a plane's annulus or a
            // cylinder that no fixture holds yet.
            let chord = if lines {
                distance * (d_b - d_a)
            } else {
                on_b - on_a
            };
            let length = chord.norm();
            if chord.x.abs() <= tol.angular * length || chord.y.abs() <= tol.angular * length {
                return Err(if lines {
                    invariant("a chamfer's chord oblique to the axis")
                } else {
                    unsupported()
                });
            }
            let wide_a = on_a.x > on_b.x;
            let (narrow, wide) = if wide_a { (on_b, on_a) } else { (on_a, on_b) };
            let (v_a, v_b) = if wide_a { (length, 0.0) } else { (0.0, length) };
            (
                Surface::Cone {
                    frame: Frame::new(
                        at_height(narrow.y),
                        at_height(wide.y) - at_height(narrow.y),
                        x,
                    )?,
                    radius: narrow.x,
                    half_angle: chord.x.abs().atan2(chord.y.abs()),
                },
                on_a,
                on_b,
                v_a,
                v_b,
                None,
            )
        }
    };
    let on_a = circle(on_a)?;
    let on_b = circle(on_b)?;
    let line_tol = Tolerance::new(tolerance, tol.angular);
    let geometry = fault_of;
    // A curved face's own `(u, v)` at the edge's start, which its
    // contact's pcurve is placed at, in `u` and on a torus in `v` too: the
    // contact is within half a turn of the edge round the tube. A plane has
    // none.
    let mut use_uv = [None; 2];
    for (i, uv) in use_uv.iter_mut().enumerate() {
        if [fa, fb][i].is_some() {
            *uv = Some(m.curve2(uses[i].pcurve)?.point(range.lo()));
        }
    }
    let mut contacts: Vec<RingContact> = Vec::with_capacity(2);
    let mut by_v = [(0, faces[0], on_a, v_a), (1, faces[1], on_b, v_b)];
    let swapped = v_b < v_a;
    if swapped {
        by_v.swap(0, 1);
    }
    // The contact's slot, by the face it is on.
    let slot_of = |i: usize| if swapped { 1 - i } else { i };
    // An open arc's corners, read before its contacts as a stripe's are:
    // an end at a tangent corner edge that is no junction is
    // `Reason::TangentChain`, whatever the contacts do beside it.
    let open = entity.start() != entity.end();
    let mut corners: [Option<([EdgeId; 2], FaceId)>; 2] = [None, None];
    if open {
        // The edge's uses by contact, which the corner is read through.
        let use_of = |face: FaceId| {
            uses.iter()
                .copied()
                .find(|u| u.face == face)
                .ok_or(invariant("a contact on one of the edge's faces"))
        };
        let contact_uses = [use_of(by_v[0].1)?, use_of(by_v[1].1)?];
        for (j, at_lo) in [(0, true), (1, false)] {
            let vertex = if at_lo { entity.start() } else { entity.end() };
            if !junctions.contains(&vertex) {
                corners[j] = Some(corner_of(m, view, edge, &contact_uses, vertex, at_lo, tol)?);
            }
        }
    }
    // Each open end's face across, read against the axis; where it is off
    // the axis each contact's own parameter there, in the edge's, which
    // the contact circles share: the crossing on the vertex's side, moved
    // by whole turns to the edge's own end.
    let mut across: [Option<Across>; 2] = [None, None];
    let mut ends_at = [[range.lo(), range.hi()]; 2];
    for (j, at_lo) in [(0, true), (1, false)] {
        let Some((_, face)) = corners[j] else {
            continue;
        };
        // A face of no family is refused at its end, after the contacts.
        let across_surface = m.surface(m.face(face)?.surface())?;
        across[j] = across_of(across_surface, rim.origin(), z, tol, tolerance);
        let Some(kind) = across[j].filter(|k| !matches!(k, Across::Axis)) else {
            continue;
        };
        let t_end = if at_lo { range.lo() } else { range.hi() };
        let vertex_point = curve.point(t_end);
        for (slot, (_, _, contact, _)) in by_v.iter().enumerate() {
            let &Curve::Circle { frame, radius } = contact else {
                return Err(invariant("a contact circle"));
            };
            let point = kind
                .meet(frame.origin(), z, radius, vertex_point, tolerance)
                .ok_or_else(|| too_large(face))?;
            let t = contact.project(point).map_err(geometry)?.t;
            ends_at[slot][j] = t + TAU * ((t_end - t) / TAU).round();
        }
    }
    for (slot, (i, face, contact, v)) in by_v.into_iter().enumerate() {
        // A contact the end trims to nothing has left its face.
        let [lo, hi] = ends_at[slot];
        let length = match contact {
            Curve::Circle { radius, .. } => (hi - lo) * radius,
            Curve::Line { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => 0.0,
        };
        if length <= tolerance {
            return Err(too_large(face));
        }
        let range = Interval::new(lo, hi).map_err(|_| too_large(face))?;
        let mid = range.midpoint();
        let face_surface = m.surface(m.face(face)?.surface())?;
        let on_face =
            pcurve_on(&contact, range, face_surface, line_tol, meter).map_err(geometry)?;
        let on_face = match (use_uv[i], face_surface) {
            (Some(uv), Surface::Torus { .. }) => placed_uv(on_face, range.lo(), uv),
            (Some(uv), _) => placed(on_face, range.lo(), uv.x),
            (None, _) => on_face,
        };
        if !on_side_of_face(m, face, &on_face, range, Side::Inside, samples)? {
            return Err(too_large(face));
        }
        let on_blend = pcurve_on(&contact, range, &surface, line_tol, meter).map_err(geometry)?;
        let along_u = on_blend.point(mid).x > on_blend.point(range.lo()).x;
        let target = Point2::new(if along_u { 0.0 } else { TAU }, v);
        contacts.push(RingContact {
            face,
            range,
            points: [contact.point(range.lo()), contact.point(range.hi())],
            curve: contact,
            on_face,
            on_blend: placed_uv(on_blend, range.lo(), target),
            along_u,
        });
    }
    let contacts: [RingContact; 2] = contacts
        .try_into()
        .map_err(|_| invariant("two contacts of the blend"))?;
    let (v0, v1) = (v_a.min(v_b), v_a.max(v_b));
    // The blend's section through the axis at the radial direction `w`:
    // the tube circle there, or the ruling between the contacts' points
    // `ends`, from the lower contact to the upper.
    let section = |w: Vec3, ends: [Point3; 2]| -> Result<(Curve, Interval), OpError> {
        match surface {
            Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => Ok((
                Curve::Circle {
                    frame: Frame::new(frame.origin() + major_radius * w, w.cross(&z), w)?,
                    radius: minor_radius,
                },
                Interval::new(v0, v1).map_err(|_| invariant("a quarter of the tube"))?,
            )),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Nurbs(_) => chord(ends[0], ends[1], tol),
        }
    };
    let ends = if !open {
        // The vertex's other edges: each curved face's seam, used twice by
        // it, one seam to a curved face.
        let at_vertex = view
            .vertex_edges
            .get(&vertex)
            .ok_or(invariant("the closed edge's vertex's edges"))?;
        let others: Vec<EdgeId> = at_vertex.iter().copied().filter(|&x| x != edge).collect();
        if others.len() != [fa, fb].iter().flatten().count() {
            return Err(vertex_blend());
        }
        // The blend's seam: the section at `u = 0`, in the plane of `X`
        // and `Z`.
        let (seam_curve_new, seam_range) =
            section(x, [contacts[0].points[0], contacts[1].points[0]])?;
        let seam_on =
            pcurve_on(&seam_curve_new, seam_range, &surface, line_tol, meter).map_err(geometry)?;
        let seam_on_blend =
            [0.0, TAU].map(|u| placed_uv(seam_on.clone(), seam_range.lo(), Point2::new(u, v0)));
        // Each curved face's seam shortened to its contact.
        let mut cuts: Vec<(Trim, usize)> = Vec::with_capacity(2);
        let mut vertex_tolerance = tolerance;
        let mut seen = [false; 2];
        for seam in others {
            let seam_uses = view.uses.get(&seam).ok_or(invariant("the seam's uses"))?;
            let [su, sv] = seam_uses[..] else {
                return Err(vertex_blend());
            };
            let Some(i) = (0..2).find(|&i| [fa, fb][i].is_some() && su.face == faces[i]) else {
                return Err(vertex_blend());
            };
            if su.face != sv.face || std::mem::replace(&mut seen[i], true) {
                return Err(vertex_blend());
            }
            let seam_entity = *m.edge(seam)?;
            let Some((seam_curve_id, seam_range0)) = seam_entity.curve() else {
                return Err(vertex_blend());
            };
            let seam_curve = m.curve(seam_curve_id)?;
            let slot = slot_of(i);
            let cut_at = contacts[slot].points[0];
            let projection = seam_curve.project(cut_at).map_err(geometry)?;
            if projection.distance > seam_entity.tolerance().max(tolerance) {
                return Err(invariant(
                    "the curved face's seam through the contact's vertex",
                ));
            }
            let Some(tc) = into_range(seam_range0, projection.t, seam_curve.period()) else {
                return Err(too_large(faces[i]));
            };
            let cuts_lo = seam_entity.start() == vertex;
            let far = m.vertex(if cuts_lo {
                seam_entity.end()
            } else {
                seam_entity.start()
            })?;
            let kept_length = if cuts_lo {
                seam_range0.hi() - tc
            } else {
                tc - seam_range0.lo()
            };
            if kept_length <= 0.0 || (cut_at - far.point()).norm() <= far.tolerance() {
                return Err(degenerate(vec![e, forward(seam)], Reason::BlendTooLarge));
            }
            vertex_tolerance = vertex_tolerance.max(seam_entity.tolerance());
            cuts.push((
                Trim {
                    edge: seam,
                    t: tc,
                    cuts_lo,
                },
                slot,
            ));
        }
        RingEnds::Seam(Box::new(RingSeam {
            curve: seam_curve_new,
            range: seam_range,
            on_blend: seam_on_blend,
            cuts,
            vertex_tolerance,
        }))
    } else {
        let mut ends: Vec<RingEnd> = Vec::with_capacity(2);
        for (j, at_lo) in [(0, true), (1, false)] {
            let vertex = if at_lo { entity.start() } else { entity.end() };
            let Some((corner_edges, across_face)) = corners[j] else {
                ends.push(RingEnd::Junction(vertex));
                continue;
            };
            let across_surface = m.surface(m.face(across_face)?.surface())?;
            let t = if at_lo { range.lo() } else { range.hi() };
            let points = [contacts[0].points[j], contacts[1].points[j]];
            // The corner edges cut or lengthened to the trim points, and
            // the side of the face across the end lies on (ADR-0038).
            let corner = mixed::corner_trims(
                m,
                view,
                edge,
                [contacts[0].face, contacts[1].face],
                corner_edges,
                vertex,
                points,
                convex,
                tol,
                samples,
                meter,
            )?;
            let trims = corner.trims;
            let across_tolerance = m.face(across_face)?.tolerance();
            let mut vertex_tolerance = [0.0; 2];
            for c in 0..2 {
                vertex_tolerance[c] = tolerance
                    .max(m.edge(corner_edges[c])?.tolerance())
                    .max(across_tolerance);
            }
            let end_tolerance = tolerance.max(across_tolerance);
            let end_tol = Tolerance::new(end_tolerance, tol.angular);
            let end = match across[j] {
                // A plane through the axis is the half-plane at the
                // vertex's angle, which meets the blend in its section
                // there, exact.
                Some(Across::Axis) => {
                    let (curve, range) = section(radial_at(t)?, points)?;
                    let on_face = pcurve_on(&curve, range, across_surface, end_tol, meter)
                        .map_err(geometry)?;
                    let on_blend =
                        pcurve_on(&curve, range, &surface, end_tol, meter).map_err(geometry)?;
                    // In the blend loop's translate: at the contacts' `u`
                    // there.
                    let u = contacts[0].on_blend.point(t).x;
                    let on_blend = placed_uv(on_blend, range.lo(), Point2::new(u, v0));
                    (curve, range, true, on_face, on_blend)
                }
                // Off the axis: the section of the blend's exact surface
                // with the face across, traced, its stretch between the two
                // trim points on the blend's band between its contacts
                // fitted (ADR-0037).
                Some(Across::Wall { .. } | Across::Post { .. }) => {
                    let band = |p: Point3| -> Result<bool, OpError> {
                        let v = surface.project(p).map_err(geometry)?.uv.y;
                        Ok(match surface {
                            Surface::Torus { minor_radius, .. } => {
                                let slack = end_tolerance / minor_radius;
                                let w = v0 + (v - v0).rem_euclid(TAU);
                                w <= v1 + slack || w >= v0 + TAU - slack
                            }
                            Surface::Plane { .. }
                            | Surface::Cylinder { .. }
                            | Surface::EllipticCylinder { .. }
                            | Surface::Cone { .. }
                            | Surface::Sphere { .. }
                            | Surface::Nurbs(_) => {
                                v >= v0 - end_tolerance && v <= v1 + end_tolerance
                            }
                        })
                    };
                    let refuse = || OpError::Unsupported {
                        a: (GeomKind::Surface(surface.kind()), e),
                        b: (
                            GeomKind::Surface(across_surface.kind()),
                            forward(across_face),
                        ),
                    };
                    // The stretch runs round the blend's section between
                    // the two points, within its width of either.
                    let width = match surface {
                        Surface::Torus { minor_radius, .. } => 2.0 * minor_radius,
                        Surface::Plane { .. }
                        | Surface::Cylinder { .. }
                        | Surface::EllipticCylinder { .. }
                        | Surface::Cone { .. }
                        | Surface::Sphere { .. }
                        | Surface::Nurbs(_) => v1 - v0,
                    };
                    let within = Aabb::of_point(points[0])
                        .union(Aabb::of_point(points[1]))
                        .inflated(width);
                    let traced = traced::traced_end(
                        &surface,
                        across_surface,
                        points,
                        &within,
                        &band,
                        &refuse,
                        end_tol,
                        meter,
                    )?;
                    for (held, gap) in vertex_tolerance.iter_mut().zip(traced.gaps) {
                        *held = held.max(gap);
                    }
                    let (range, lo_first) = (traced.range, traced.lo_first);
                    let first = arc_end(range, lo_first, 0);
                    // On the face across in its loop's translate, at the
                    // corner edge a contact cuts, never one lengthened
                    // (ADR-0038 §4); on the blend at the first contact's
                    // end.
                    let c = usize::from(corner.lengthened == Some(0));
                    let at_corner =
                        at_cut_corner(m, view, across_face, corner_edges[c], trims[c].t)?;
                    let on_face = pcurve_on(&traced.curve, range, across_surface, end_tol, meter)
                        .map_err(geometry)?;
                    let on_face = placed_uv(on_face, arc_end(range, lo_first, c), at_corner);
                    let on_blend = pcurve_on(&traced.curve, range, &surface, end_tol, meter)
                        .map_err(geometry)?;
                    let at_contact = contacts[0].on_blend.point(contacts[0].range.lo());
                    let at_contact = if at_lo {
                        at_contact
                    } else {
                        contacts[0].on_blend.point(contacts[0].range.hi())
                    };
                    let on_blend = placed_uv(on_blend, first, at_contact);
                    (traced.curve, range, lo_first, on_face, on_blend)
                }
                // Anything else meets a torus in a quartic no family
                // holds.
                None => {
                    return Err(OpError::Unsupported {
                        a: (GeomKind::Surface(surface.kind()), e),
                        b: (
                            GeomKind::Surface(across_surface.kind()),
                            forward(across_face),
                        ),
                    });
                }
            };
            let (end_curve, end_range, lo_first, on_face, on_blend) = end;
            if !on_side_of_face(m, across_face, &on_face, end_range, corner.side, samples)? {
                return Err(too_large(across_face));
            }
            ends.push(RingEnd::Face(Box::new(ArcEnd {
                vertex,
                face: across_face,
                trims,
                curve: end_curve,
                range: end_range,
                lo_first,
                on_face,
                on_blend,
                tolerance: end_tolerance,
                vertex_tolerance,
            })));
        }
        let ends: [RingEnd; 2] = ends
            .try_into()
            .map_err(|_| invariant("two ends of the blend"))?;
        RingEnds::Open(Box::new(ends))
    };
    // The blend faces out of the material: the corner's outward direction
    // at the start's half-plane against the surface's normal there.
    let normal = surface
        .normal(0.0, (v0 + v1) / 2.0)
        .ok_or(invariant("a regular blend surface"))?;
    let outward = |mer: &Meridian| mer.normal.x * x + mer.normal.y * z;
    let orientation = if normal.dot(&(outward(&mer_a) + outward(&mer_b))) > 0.0 {
        Orientation::Forward
    } else {
        Orientation::Reversed
    };
    Ok(Ring {
        edge,
        range,
        surface,
        orientation,
        convex,
        centres,
        contacts,
        ends,
        tolerance,
    })
}

/// The tolerance a face end's trim vertex carries: the largest of the
/// entities that meet there — the contact, the arc and the corner edge —
/// at either end of the contact.
fn vertex_tolerance_of(ends: &[EndKind; 2], k: usize) -> f64 {
    ends.iter()
        .filter_map(|end| end.vertex_tolerance(k))
        .fold(0.0, f64::max)
}

/// The arc inserted into the face across a corner, at the junction the
/// consumed vertex stood at.
#[derive(Debug, Clone, Copy)]
struct Insertion {
    arc: EdgeKey,
    pcurve: Curve2Id,
    /// `true` when the arc's `range.lo()` is at the contact at `u = 0`.
    lo_first: bool,
    /// The corner edge the contact at `u = 0` cuts.
    edge_at_lo: EdgeId,
}

/// What a face's loops are rewritten with: a blended edge replaced by
/// the contact on this face, and an arc inserted at a consumed vertex.
#[derive(Default)]
struct FaceEdit {
    replace: BTreeMap<EdgeId, (EdgeKey, Curve2Id)>,
    insert: BTreeMap<VertexId, Insertion>,
}

/// Where a corner edge is cut at each of its ends: the new vertex and the
/// parameter.
#[derive(Default, Clone, Copy)]
struct Cuts {
    lo: Option<(usize, f64)>,
    hi: Option<(usize, f64)>,
}

/// The rewrite's indices of one blend's entities: its four trim vertices
/// `[end][contact]`, its two contact edges, its two end arcs and its
/// added face. At a miter or a corner the vertices and the arc are the
/// miter's or the corner's, shared with the other blends.
#[derive(Default, Clone, Copy)]
struct Made {
    vertices: [[usize; 2]; 2],
    contacts: [usize; 2],
    arcs: [usize; 2],
    added: usize,
}

/// The rewrite's indices of one miter's entities: its two vertices, `q`
/// then `p3`, and its edge.
#[derive(Clone, Copy)]
struct MiterMade {
    vertices: [usize; 2],
    edge: usize,
}

/// The rewrite's indices of one corner's entities: its three vertices,
/// its three arcs by side, a sphere's pole and its added face.
#[derive(Clone, Copy)]
struct CornerMade {
    vertices: [usize; 3],
    arcs: [usize; 3],
    pole: Option<usize>,
    added: usize,
}

/// Records `cut` at one end of a corner edge, once.
fn cut_once(cuts: &mut BTreeMap<EdgeId, Cuts>, trim: &Trim, vertex: usize) -> Result<(), OpError> {
    let cut = cuts.entry(trim.edge).or_default();
    let slot = if trim.cuts_lo {
        &mut cut.lo
    } else {
        &mut cut.hi
    };
    if slot.replace((vertex, trim.t)).is_some() {
        return Err(invariant("one cut per end of a corner edge"));
    }
    Ok(())
}

/// Builds the blends of `edges`, in that order, into a rewrite of `body`
/// and returns the result with its provenance.
fn build(
    m: &mut Model,
    body: Body,
    edges: &[EdgeId],
    kind: Kind,
    meter: &mut Meter<'_>,
) -> Result<(Body, Provenance), OpError> {
    let precision = m.precision();
    let tol = precision.tolerance();
    let samples = precision.check_samples;
    let view = View::of(m, body)?;
    // The named edges and every edge their chains run on into, in the
    // body's order.
    let reached = chain(m, &view, edges, tol, meter)?;
    let chained: Vec<EdgeId> = m
        .edges(body)?
        .into_iter()
        .map(|e| e.id)
        .filter(|id| reached.contains(id))
        .collect();
    let no_junctions = BTreeSet::new();
    // A closed edge and a circular arc are rings, the one with no ends and
    // the other trimmed at its two or met at a junction; the rest are
    // stripes.
    let mut open: Vec<EdgeId> = Vec::with_capacity(chained.len());
    let mut arcs: Vec<EdgeId> = Vec::new();
    let mut rings: Vec<Ring> = Vec::new();
    for &e in &chained {
        let entity = *m.edge(e)?;
        match entity.curve() {
            Some(_) if entity.start() == entity.end() => {
                rings.push(ring(m, &view, e, kind, &no_junctions, tol, samples, meter)?);
            }
            Some((curve, _)) if matches!(m.curve(curve)?, Curve::Circle { .. }) => {
                arcs.push(e);
            }
            Some(_) | None => open.push(e),
        }
    }
    let edges = &open[..];
    // The blended edges at each vertex: two at a tangent vertex meet in a
    // junction, two elsewhere in a miter, three in a corner, and more at a
    // vertex the closed forms do not cover; an arc meets no other blend but
    // at a junction.
    let mut at_vertex: BTreeMap<VertexId, Vec<EdgeId>> = BTreeMap::new();
    for &e in edges.iter().chain(&arcs) {
        let entity = *m.edge(e)?;
        for v in [entity.start(), entity.end()] {
            at_vertex.entry(v).or_default().push(e);
        }
    }
    let mut junctions: BTreeSet<VertexId> = BTreeSet::new();
    for (&v, es) in &at_vertex {
        if let [ea, eb] = es[..]
            && tangent_vertex(m, &view, ea, v, tol)? == Some(eb)
        {
            junctions.insert(v);
            continue;
        }
        if es.len() > 3 || (es.len() > 1 && es.iter().any(|e| arcs.contains(e))) {
            let entities = es.iter().map(|&e| forward(e)).chain([forward(v)]).collect();
            return Err(degenerate(entities, Reason::VertexBlend));
        }
    }
    for &e in &arcs {
        rings.push(ring(m, &view, e, kind, &junctions, tol, samples, meter)?);
    }
    at_vertex.retain(|v, es| junctions.contains(v) || !es.iter().any(|e| arcs.contains(e)));
    let mut stripes: Vec<Stripe> = Vec::with_capacity(edges.len());
    for &e in edges {
        meter.tick()?;
        stripes.push(stripe(m, &view, e, kind, tol)?);
    }
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
    for (&v, es) in &at_vertex {
        match es[..] {
            [ea, eb] if junctions.contains(&v) => {
                miter_at.insert(v, miters.len());
                miters.push(junction(m, &view, run(ea)?, run(eb)?, v, tol, meter)?);
            }
            [ea, eb] => {
                miter_at.insert(v, miters.len());
                miters.push(miter(
                    m,
                    &view,
                    &stripes[index_of[&ea]],
                    &stripes[index_of[&eb]],
                    v,
                    tol,
                    meter,
                )?);
            }
            [ea, eb, ec] => {
                corner_at.insert(v, corners.len());
                corners.push(corner(
                    m,
                    &view,
                    [
                        &stripes[index_of[&ea]],
                        &stripes[index_of[&eb]],
                        &stripes[index_of[&ec]],
                    ],
                    v,
                    tol,
                    meter,
                )?);
            }
            _ => {}
        }
    }
    // Each stripe's ends, then its contacts between them.
    let mut blends: Vec<Blend> = Vec::with_capacity(stripes.len());
    for s in stripes {
        let mut ends: Vec<EndKind> = Vec::with_capacity(2);
        for (at_lo, vertex) in [(true, s.start), (false, s.end)] {
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
                (None, None) => EndKind::Face(Box::new(face_end(
                    m, &view, &s, at_lo, tol, samples, meter,
                )?)),
            });
        }
        let ends: [EndKind; 2] = ends
            .try_into()
            .map_err(|_| invariant("two ends of the blend"))?;
        let t = [0, 1].map(|end| [0, 1].map(|k| ends[end].t(&miters, &corners, k)));
        let contacts = contacts(m, &s, t, tol, samples, meter)?;
        blends.push(Blend {
            stripe: s,
            contacts,
            ends,
        });
    }

    let mut rw = Rewrite::default();
    let mut cuts: BTreeMap<EdgeId, Cuts> = BTreeMap::new();
    let mut edits: BTreeMap<FaceId, FaceEdit> = BTreeMap::new();
    // The miters first: their two vertices, their edge, the third edge
    // cut.
    let mut miter_made: Vec<MiterMade> = Vec::with_capacity(miters.len());
    for mt in &miters {
        let q = rw.vertices.len();
        rw.vertices.push(VertexSpec::New {
            point: mt.q,
            tolerance: mt.q_tolerance,
        });
        let p3 = rw.vertices.len();
        rw.vertices.push(VertexSpec::New {
            point: mt.p3,
            tolerance: mt.p3_tolerance,
        });
        let (first, second) = if mt.q_first { (q, p3) } else { (p3, q) };
        let edge = rw.edges.len();
        rw.edges.push((
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
        ));
        cut_once(&mut cuts, &mt.trim, p3)?;
        miter_made.push(MiterMade {
            vertices: [q, p3],
            edge,
        });
    }
    // The corners next: their three vertices, their three arcs and a
    // sphere's pole; no corner edge is cut.
    let mut corner_made: Vec<CornerMade> = Vec::with_capacity(corners.len());
    for c in &corners {
        let mut vertices = [0usize; 3];
        for (slot, &point) in vertices.iter_mut().zip(&c.points) {
            *slot = rw.vertices.len();
            rw.vertices.push(VertexSpec::New {
                point,
                tolerance: c.tolerance,
            });
        }
        let mut arcs = [0usize; 3];
        for (slot, arc) in arcs.iter_mut().zip(&c.arcs) {
            *slot = rw.edges.len();
            rw.edges.push((
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
            ));
        }
        let pole = c.pole.as_ref().map(|pole| {
            let at = VertexKey::New(vertices[pole.point]);
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Degenerate { range: pole.range },
                    start: at,
                    end: at,
                    tolerance: c.tolerance,
                },
                None,
            ));
            rw.edges.len() - 1
        });
        corner_made.push(CornerMade {
            vertices,
            arcs,
            pole,
            added: 0,
        });
    }
    let mut made: Vec<Made> = Vec::with_capacity(blends.len());
    for blend in &blends {
        let mut vertices = [[0usize; 2]; 2];
        for (end_index, end) in blend.ends.iter().enumerate() {
            match end {
                EndKind::Face(face_end) => {
                    for (k, slot) in vertices[end_index].iter_mut().enumerate() {
                        *slot = rw.vertices.len();
                        rw.vertices.push(VertexSpec::New {
                            point: face_end.points[k],
                            tolerance: vertex_tolerance_of(&blend.ends, k),
                        });
                    }
                }
                EndKind::Miter { at, side } => {
                    let shared = miters[*at].shared[*side];
                    vertices[end_index][shared] = miter_made[*at].vertices[0];
                    vertices[end_index][1 - shared] = miter_made[*at].vertices[1];
                }
                EndKind::Corner { at, side } => {
                    for (k, slot) in vertices[end_index].iter_mut().enumerate() {
                        *slot = corner_made[*at].vertices[corners[*at].contact_point[*side][k]];
                    }
                }
            }
        }
        let mut contact_edges = [0usize; 2];
        for (k, contact) in blend.contacts.iter().enumerate() {
            contact_edges[k] = rw.edges.len();
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(contact.line.clone()),
                        range: contact.range,
                    },
                    start: VertexKey::New(vertices[0][k]),
                    end: VertexKey::New(vertices[1][k]),
                    tolerance: contact.tolerance,
                },
                None,
            ));
        }
        let mut arcs = [0usize; 2];
        for (end_index, end) in blend.ends.iter().enumerate() {
            let face_end = match end {
                EndKind::Face(face_end) => face_end,
                EndKind::Miter { at, .. } => {
                    arcs[end_index] = miter_made[*at].edge;
                    continue;
                }
                EndKind::Corner { at, side } => {
                    arcs[end_index] = corner_made[*at].arcs[*side];
                    continue;
                }
            };
            let (first, second) = if face_end.arc.lo_first {
                (0, 1)
            } else {
                (1, 0)
            };
            arcs[end_index] = rw.edges.len();
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(face_end.arc.curve.clone()),
                        range: face_end.arc.range,
                    },
                    start: VertexKey::New(vertices[end_index][first]),
                    end: VertexKey::New(vertices[end_index][second]),
                    tolerance: face_end.arc.tolerance,
                },
                None,
            ));
            for (k, trim) in face_end.trims.iter().enumerate() {
                cut_once(&mut cuts, trim, vertices[end_index][k])?;
            }
            let pcurve = m.add_curve2(face_end.arc.on_face.clone());
            edits.entry(face_end.face).or_default().insert.insert(
                face_end.vertex,
                Insertion {
                    arc: EdgeKey::New(arcs[end_index]),
                    pcurve,
                    lo_first: face_end.arc.lo_first,
                    edge_at_lo: face_end.trims[0].edge,
                },
            );
        }
        for (k, contact) in blend.contacts.iter().enumerate() {
            let pcurve = m.add_curve2(contact.on_face.clone());
            edits
                .entry(contact.face)
                .or_default()
                .replace
                .insert(blend.stripe.edge, (EdgeKey::New(contact_edges[k]), pcurve));
        }
        made.push(Made {
            vertices,
            contacts: contact_edges,
            arcs,
            added: 0,
        });
    }
    // The rings: a closed edge's two vertices, two contact circles, the
    // blend's seam and the cylinder's seam cut; an open arc's four
    // vertices, two contact arcs and two ends, each inserted into its face
    // across with the corner edges cut.
    let mut ring_made: Vec<Made> = Vec::with_capacity(rings.len());
    for r in &rings {
        let mut vertices = [[0usize; 2]; 2];
        match &r.ends {
            RingEnds::Seam(seam) => {
                for (slot, contact) in vertices[0].iter_mut().zip(&r.contacts) {
                    *slot = rw.vertices.len();
                    rw.vertices.push(VertexSpec::New {
                        point: contact.points[0],
                        tolerance: seam.vertex_tolerance,
                    });
                }
                vertices[1] = vertices[0];
            }
            RingEnds::Open(ends) => {
                for (j, end) in ends.iter().enumerate() {
                    match end {
                        RingEnd::Face(end) => {
                            for (c, contact) in r.contacts.iter().enumerate() {
                                vertices[j][c] = rw.vertices.len();
                                rw.vertices.push(VertexSpec::New {
                                    point: contact.points[j],
                                    tolerance: end.vertex_tolerance[c],
                                });
                            }
                        }
                        RingEnd::Junction(vertex) => {
                            let at = *miter_at
                                .get(vertex)
                                .ok_or(invariant("a junction at the ring's end"))?;
                            let side = usize::from(miters[at].edges[0] != r.edge);
                            let shared = miters[at].shared[side];
                            vertices[j][shared] = miter_made[at].vertices[0];
                            vertices[j][1 - shared] = miter_made[at].vertices[1];
                        }
                    }
                }
            }
        }
        let mut contact_edges = [0usize; 2];
        for (c, contact) in r.contacts.iter().enumerate() {
            contact_edges[c] = rw.edges.len();
            rw.edges.push((
                EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(contact.curve.clone()),
                        range: contact.range,
                    },
                    start: VertexKey::New(vertices[0][c]),
                    end: VertexKey::New(vertices[1][c]),
                    tolerance: r.tolerance,
                },
                None,
            ));
            let pcurve = m.add_curve2(contact.on_face.clone());
            edits
                .entry(contact.face)
                .or_default()
                .replace
                .insert(r.edge, (EdgeKey::New(contact_edges[c]), pcurve));
        }
        let mut arcs = [0usize; 2];
        match &r.ends {
            RingEnds::Seam(seam) => {
                arcs = [rw.edges.len(); 2];
                rw.edges.push((
                    EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(seam.curve.clone()),
                            range: seam.range,
                        },
                        start: VertexKey::New(vertices[0][0]),
                        end: VertexKey::New(vertices[0][1]),
                        tolerance: r.tolerance,
                    },
                    None,
                ));
                for (trim, by) in &seam.cuts {
                    cut_once(&mut cuts, trim, vertices[0][*by])?;
                }
            }
            RingEnds::Open(ends) => {
                for (j, end) in ends.iter().enumerate() {
                    let end = match end {
                        RingEnd::Face(end) => end,
                        RingEnd::Junction(vertex) => {
                            arcs[j] = miter_made[miter_at[vertex]].edge;
                            continue;
                        }
                    };
                    let (first, second) = if end.lo_first { (0, 1) } else { (1, 0) };
                    arcs[j] = rw.edges.len();
                    rw.edges.push((
                        EdgeSpec::New {
                            geometry: EdgeGeometry::Curve {
                                curve: m.add_curve(end.curve.clone()),
                                range: end.range,
                            },
                            start: VertexKey::New(vertices[j][first]),
                            end: VertexKey::New(vertices[j][second]),
                            tolerance: end.tolerance,
                        },
                        None,
                    ));
                    for (c, trim) in end.trims.iter().enumerate() {
                        cut_once(&mut cuts, trim, vertices[j][c])?;
                    }
                    let pcurve = m.add_curve2(end.on_face.clone());
                    edits.entry(end.face).or_default().insert.insert(
                        end.vertex,
                        Insertion {
                            arc: EdgeKey::New(arcs[j]),
                            pcurve,
                            lo_first: end.lo_first,
                            edge_at_lo: end.trims[0].edge,
                        },
                    );
                }
            }
        }
        ring_made.push(Made {
            vertices,
            contacts: contact_edges,
            arcs,
            added: 0,
        });
    }
    // The corner edges shortened or lengthened, in id order; a lengthened
    // edge's pcurves derived again over its new range (ADR-0038 §4), by
    // the edge and the pcurve each replaces.
    let mut shortened: BTreeMap<EdgeId, EdgeKey> = BTreeMap::new();
    let mut rederived: BTreeMap<(EdgeId, Curve2Id), Curve2Id> = BTreeMap::new();
    for (&edge, cut) in &cuts {
        let entity = *m.edge(edge)?;
        let Some((curve, old)) = entity.curve() else {
            return Err(invariant("a corner edge's curve"));
        };
        let lo = cut.lo.map_or(old.lo(), |(_, t)| t);
        let hi = cut.hi.map_or(old.hi(), |(_, t)| t);
        let range = Interval::new(lo, hi)
            .map_err(|_| degenerate(vec![forward(edge)], Reason::BlendTooLarge))?;
        // Placed where the old pcurve still runs: at the end lengthened.
        let anchor = if lo < old.lo() {
            Some(old.lo())
        } else if hi > old.hi() {
            Some(old.hi())
        } else {
            None
        };
        if let Some(anchor) = anchor {
            let edge_tol = Tolerance::new(entity.tolerance(), tol.angular);
            for u in view.uses.get(&edge).into_iter().flatten() {
                let surface = m.surface(m.face(u.face)?.surface())?;
                let fresh = pcurve_on(m.curve(curve)?, range, surface, edge_tol, meter)
                    .map_err(fault_of)?;
                let fresh = placed_uv(fresh, anchor, m.curve2(u.pcurve)?.point(anchor));
                let id = m.add_curve2(fresh);
                rederived.insert((edge, u.pcurve), id);
            }
        }
        shortened.insert(edge, EdgeKey::New(rw.edges.len()));
        rw.edges.push((
            EdgeSpec::New {
                geometry: EdgeGeometry::Curve { curve, range },
                start: cut
                    .lo
                    .map_or(VertexKey::Kept(entity.start()), |(v, _)| VertexKey::New(v)),
                end: cut
                    .hi
                    .map_or(VertexKey::Kept(entity.end()), |(v, _)| VertexKey::New(v)),
                tolerance: entity.tolerance(),
            },
            Some(edge),
        ));
    }
    // Every touched face's loops rewritten, in the body's face order.
    for &face in &view.faces {
        let entity = m.face(face)?;
        let edit = edits.get(&face);
        let touched = edit.is_some()
            || entity
                .loops()
                .iter()
                .flat_map(|l| l.coedges())
                .any(|c| shortened.contains_key(&c.edge()));
        if !touched {
            continue;
        }
        let mut loops: Vec<Vec<StoredUse>> = Vec::with_capacity(entity.loops().len());
        for l in entity.loops() {
            let mut uses: Vec<StoredUse> = Vec::with_capacity(l.coedges().len() + 1);
            for c in l.coedges() {
                let id = c.edge();
                let (edge, pcurve) = match edit.and_then(|e| e.replace.get(&id)) {
                    Some(&(key, pcurve)) => (key, pcurve),
                    None => (
                        shortened.get(&id).copied().unwrap_or(EdgeKey::Kept(id)),
                        (rederived.get(&(id, c.pcurve())).copied()).unwrap_or(c.pcurve()),
                    ),
                };
                uses.push(StoredUse {
                    edge,
                    orientation: c.orientation(),
                    pcurve,
                });
                let ce = m.edge(id)?;
                let junction = if c.orientation() == Orientation::Forward {
                    ce.end()
                } else {
                    ce.start()
                };
                if let Some(ins) = edit.and_then(|e| e.insert.get(&junction)) {
                    // The arc runs from the end of the corner edge just
                    // walked to the start of the next.
                    let arrived_at_lo = id == ins.edge_at_lo;
                    let orientation = if ins.lo_first == arrived_at_lo {
                        Orientation::Forward
                    } else {
                        Orientation::Reversed
                    };
                    uses.push(StoredUse {
                        edge: ins.arc,
                        orientation,
                        pcurve: ins.pcurve,
                    });
                }
            }
            loops.push(uses);
        }
        rw.faces.insert(face, loops);
    }
    // The blend faces, one per edge, after the shell's own: each end's
    // arc walked from one contact to the other, the miter's with its
    // pcurve on this blend's cylinder.
    for (blend, made) in blends.iter().zip(made.iter_mut()) {
        let (contact_edges, arcs) = (made.contacts, made.arcs);
        let [lo, hi] = &blend.contacts;
        let surface = m.add_surface(blend.stripe.surface.clone());
        let mut end_uses = [None; 2];
        for (end_index, end) in blend.ends.iter().enumerate() {
            let (lo_first, pcurve) = match end {
                EndKind::Face(face_end) => (face_end.arc.lo_first, &face_end.arc.on_blend),
                EndKind::Miter { at, side } => {
                    (miters[*at].lo_first[*side], &miters[*at].on_blend[*side])
                }
                EndKind::Corner { at, side } => {
                    let arc = &corners[*at].arcs[*side];
                    (arc.lo_first, &arc.on_blend)
                }
            };
            // The start arc is walked from `u = 0` to `u = β`, the end
            // arc back.
            let along = lo_first == (end_index == 0);
            end_uses[end_index] = Some(StoredUse {
                edge: EdgeKey::New(arcs[end_index]),
                orientation: if along {
                    Orientation::Forward
                } else {
                    Orientation::Reversed
                },
                pcurve: m.add_curve2(pcurve.clone()),
            });
        }
        let [Some(start_arc), Some(end_arc)] = end_uses else {
            return Err(invariant("two ends of the blend"));
        };
        let loop_uses = vec![
            start_arc,
            StoredUse {
                edge: EdgeKey::New(contact_edges[1]),
                orientation: Orientation::Forward,
                pcurve: m.add_curve2(hi.on_blend.clone()),
            },
            end_arc,
            StoredUse {
                edge: EdgeKey::New(contact_edges[0]),
                orientation: Orientation::Reversed,
                pcurve: m.add_curve2(lo.on_blend.clone()),
            },
        ];
        made.added = rw.added.len();
        rw.added.push(AddedFace {
            shell: view.shell_of[&lo.face],
            surface,
            orientation: if blend.stripe.convex {
                Orientation::Forward
            } else {
                Orientation::Reversed
            },
            loops: vec![loop_uses],
            tolerance: blend.stripe.tolerance,
        });
    }
    // The corner faces, one per corner, their sides in walking order with
    // a sphere's pole crossed where its meridians meet.
    for (c, made) in corners.iter().zip(corner_made.iter_mut()) {
        let surface = m.add_surface(c.surface.clone());
        let mut loop_uses: Vec<StoredUse> = Vec::with_capacity(4);
        for (w, &side) in c.walk.iter().enumerate() {
            let arc = &c.arcs[side];
            loop_uses.push(StoredUse {
                edge: EdgeKey::New(made.arcs[side]),
                orientation: if arc.along {
                    Orientation::Forward
                } else {
                    Orientation::Reversed
                },
                pcurve: m.add_curve2(arc.on_corner.clone()),
            });
            if let (Some(pole), Some(edge)) = (&c.pole, made.pole)
                && pole.after == w
            {
                loop_uses.push(StoredUse {
                    edge: EdgeKey::New(edge),
                    orientation: Orientation::Reversed,
                    pcurve: m.add_curve2(pole.pcurve.clone()),
                });
            }
        }
        made.added = rw.added.len();
        rw.added.push(AddedFace {
            shell: view.shell_of[&c.faces[0]],
            surface,
            orientation: c.orientation,
            loops: vec![loop_uses],
            tolerance: c.tolerance,
        });
    }
    // The ring faces, one per circular edge, in (u, v) counter-clockwise:
    // the lower contact along `u`, up at its far `u` — the seam at `u = 2π`
    // or the end there — the upper contact back, down at its near `u`.
    for (r, made) in rings.iter().zip(ring_made.iter_mut()) {
        let [lower, upper] = &r.contacts;
        let surface = m.add_surface(r.surface.clone());
        let along = |yes: bool| {
            if yes {
                Orientation::Forward
            } else {
                Orientation::Reversed
            }
        };
        // Each end's edge, its pcurve, and whether it runs up from the
        // lower contact: a seam's and a face end's do, a junction's when
        // its `q` is on the lower contact.
        let (far, near) = match &r.ends {
            RingEnds::Seam(seam) => (
                (made.arcs[0], seam.on_blend[1].clone(), true),
                (made.arcs[0], seam.on_blend[0].clone(), true),
            ),
            RingEnds::Open(ends) => {
                // The edge's end is at the far `u` when the contacts run
                // with `u`, its start otherwise.
                let (hi, lo) = if lower.along_u { (1, 0) } else { (0, 1) };
                let end = |j: usize| match &ends[j] {
                    RingEnd::Face(end) => (made.arcs[j], end.on_blend.clone(), end.lo_first),
                    RingEnd::Junction(vertex) => {
                        let junction = &miters[miter_at[vertex]];
                        let side = usize::from(junction.edges[0] != r.edge);
                        (
                            made.arcs[j],
                            junction.on_blend[side].clone(),
                            junction.shared[side] == 0,
                        )
                    }
                };
                (end(hi), end(lo))
            }
        };
        let loop_uses = vec![
            StoredUse {
                edge: EdgeKey::New(made.contacts[0]),
                orientation: along(lower.along_u),
                pcurve: m.add_curve2(lower.on_blend.clone()),
            },
            StoredUse {
                edge: EdgeKey::New(far.0),
                orientation: along(far.2),
                pcurve: m.add_curve2(far.1),
            },
            StoredUse {
                edge: EdgeKey::New(made.contacts[1]),
                orientation: along(!upper.along_u),
                pcurve: m.add_curve2(upper.on_blend.clone()),
            },
            StoredUse {
                edge: EdgeKey::New(near.0),
                orientation: along(!near.2),
                pcurve: m.add_curve2(near.1),
            },
        ];
        made.added = rw.added.len();
        rw.added.push(AddedFace {
            shell: view.shell_of[&lower.face],
            surface,
            orientation: r.orientation,
            loops: vec![loop_uses],
            tolerance: r.tolerance,
        });
    }

    let out = rebuild::rewrite(m, body, rw)?;
    let mut p = out.provenance;
    // Every record against the blended edge; a miter's edge and vertices
    // are generated from both edges it joins, so each records them.
    for (blend, made) in blends.iter().zip(&made) {
        let origin = forward(blend.stripe.edge);
        p.add_generated(origin, forward(out.added[made.added]));
        for &k in made.contacts.iter().chain(&made.arcs) {
            p.add_generated(origin, forward(out.edges[k]));
        }
        for &v in made.vertices.iter().flatten() {
            p.add_generated(origin, forward(out.vertices[v]));
        }
    }
    // A corner's face and a sphere's pole from each of its three edges;
    // its vertices and sides are its blends' own, recorded above.
    for (c, made) in corners.iter().zip(&corner_made) {
        for &edge in &c.edges {
            let origin = forward(edge);
            p.add_generated(origin, forward(out.added[made.added]));
            if let Some(pole) = made.pole {
                p.add_generated(origin, forward(out.edges[pole]));
            }
        }
    }
    // A ring's face, its two contacts, its seam or its two ends, and its
    // two or four vertices.
    for (r, made) in rings.iter().zip(&ring_made) {
        let origin = forward(r.edge);
        let ends = match r.ends {
            RingEnds::Seam(_) => 1,
            RingEnds::Open(_) => 2,
        };
        p.add_generated(origin, forward(out.added[made.added]));
        for &k in made.contacts.iter().chain(&made.arcs[..ends]) {
            p.add_generated(origin, forward(out.edges[k]));
        }
        for &v in made.vertices[..ends].iter().flatten() {
            p.add_generated(origin, forward(out.vertices[v]));
        }
    }
    Ok((out.body, p))
}

/// Blends `edges` of `body` with a rolling ball of `radius`: each edge's
/// two faces are replaced by the same faces cut back to the ball's
/// contact curves, the edge by a blend face tangent to both along them,
/// and each end of the blend is trimmed by the face across the corner,
/// whose vertex goes and whose two other edges are shortened to the
/// trim's arc (ADR-0007). Two planes blend to a cylinder of `radius` on
/// the line where their offset planes meet — its frame's `X` at one
/// contact ruling and `Z` along the edge, so the contacts sit at `u = 0`
/// and `u = π − φ` for the dihedral's normals `φ` apart and `v` is the
/// edge's own parameter — with the contact lines at distance
/// `r tan(φ/2)` from the edge on each face. A plane and a cylinder of
/// radius `R` along a ruling blend to a cylinder of `radius` on the line
/// where the plane's offset by `radius` meets the cylinder coaxial with
/// the face at `R − radius` or `R + radius` — the ball inside the face's
/// cylinder or outside it — on the edge's side of the axis, the contact
/// on the plane a line and on the cylinder the ruling toward the ball's
/// centre, the contacts at `u = 0` and at the angle between them. The arc
/// across an end is a
/// circle when that face is perpendicular to the edge and an ellipse
/// otherwise, exact on the plane and a `Line` or a fitted `Nurbs` on the
/// cylinder; on a cylinder or a cone across — a rib running into a boss —
/// the quartic of the two, traced and fitted between where each contact
/// pierces that face (ADR-0037), the one fitted curve on an exact surface.
/// At a face across whose two corner edges differ in convexity — a blend
/// running into a step, on a stripe or a ring's open arc — the corner edge of the blend's own convexity is
/// lengthened past the vertex along its curve to the trim, and the face
/// across takes the region the arc bounds (ADR-0038).
/// Two blends meeting at a vertex whose third edge stays sharp
/// meet in a miter: the ellipse of the two cylinders in the plane
/// through the ball's centre bisecting their axes, from where the two
/// contacts on the shared face cross to where the other two meet the
/// third edge, a fitted `Nurbs` pcurve on each cylinder; the third edge
/// is shortened to that point and no face takes an arc. Three blends at a
/// vertex of three planes, all convex or all concave, meet in the sphere of
/// `radius` about the ball's one centre, tangent to each cylinder along the
/// great circle through the centre square to its axis, between the points
/// where each face's two contacts cross: its frame's `Z` toward the point
/// of a face square to the other two, so its sides are its equator and two
/// meridians, exact lines in (u, v), meeting at its pole, a degenerate
/// edge; no corner edge is cut. A plane, a
/// cylinder or a cone against another of them, at most one a plane, along
/// a circle coaxial with both — a closed edge, a hole's rim, a boss's
/// base, a frustum's rim, a turned shoulder — blend with no ends to a
/// torus coaxial with the curved faces, its centre circle where the
/// first face's offset meets the second's in the half-plane through the
/// axis (ADR-0036) and its minor radius `radius`, each contact the foot of
/// that circle on its face, a parallel of it, the torus's `u` seam a tube
/// circle from one contact's vertex to the other's in the half-plane of
/// each curved face's seam, which is shortened to its contact. An open arc
/// of such a
/// circle blends to the same torus over the arc's own range, each end
/// trimmed by the face across: a plane through the axis on
/// the tube circle at the vertex's angle (ADR-0035); a plane parallel to
/// the axis or a cylinder about a parallel axis, off it, on the section of
/// the torus with that face, traced and fitted between where each contact
/// meets it, the contacts then trimmed at their own angles (ADR-0037) —
/// the one fitted curve on an exact surface. The selection
/// follows chains: at a tangent vertex — three edges, the third's two
/// faces tangent there, the next edge sharing one face with this one,
/// of its sense and running on — the blend runs on into the next edge,
/// and on until a vertex that is not one, each edge with its own blend
/// face; two blends meet there on the ball's great circle square to the
/// edges' direction, an exact line in (u, v) on each, from where their
/// contacts on the shared face meet to where the other two meet on the
/// third edge, which is shortened to it. Convex or
/// concave is read from the dihedral. Every surface is exact, every
/// untouched entity keeps its id, and the result's ids are the same for
/// any order of the same edges (the blends are built in the body's
/// iteration order of them).
///
/// Provenance, every record against the blended edge: the blend face, its
/// two contact edges, its two end arcs and the four trim vertices
/// `Generated` from it; each of the edge's faces, each face across an
/// end and each corner edge the trim shortens or lengthens `Modified` into its new
/// self; the edge and the two corner vertices `Deleted`; the shell and
/// the body `Modified`. A miter's edge and two vertices are `Generated`
/// from both edges they join. At a corner of three, each side of the
/// sphere is its blend's end arc and each corner point is `Generated` from
/// the two edges whose contacts cross there; the sphere face and its pole
/// from all three. A closed edge's torus face, its two contact
/// circles, its seam and the seam's two vertices are `Generated` from it,
/// the curved face's seam `Modified` and the edge's vertex `Deleted`; an open
/// arc's torus face, its two contacts, its two end sections and their four
/// vertices are `Generated` from it, its faces across and corner edges
/// `Modified`, the arc and its two vertices `Deleted`. An edge the chain
/// reached is recorded as a named one is; the great circle where two
/// blends of a chain meet and its two vertices are `Generated` from both
/// edges, as a miter's are, the third edge `Modified` and the vertex
/// `Deleted`.
/// `arris_topo::provenance::audit` holds on every result
/// (`docs/DATA-MODEL.md` §Provenance).
///
/// Errors, the model untouched: [`OpError::Degenerate`] with
/// [`Reason::NonFinite`] or [`Reason::NotPositive`] on the radius,
/// [`Reason::NoEdges`] for an empty list, [`Reason::RepeatedEdge`] for
/// an edge listed twice, [`Reason::EdgeNotInBody`] for one that is not
/// the body's, [`Reason::TangentChain`] where the edge's faces meet at a
/// tangent dihedral or an end's corner edge has tangent faces at a vertex
/// the chain does not run on through — the next edge turning back,
/// itself a tangent dihedral, or of the other sense —
/// [`Reason::VertexBlend`] at a corner the closed
/// forms do not cover — a vertex of other than three edges, a miter
/// whose two blends have unequal dihedrals or are not both convex or
/// both concave, a miter with a blend along a ruling, whose contact on
/// the cylinder misses the other's on the third edge, or a corner of three
/// blended edges whose faces are not all planes, whose blends are mixed or
/// none of whose faces is square to the other two — and
/// [`Reason::BlendTooLarge`] where a contact line or an end arc leaves
/// its face through an edge that is not the corner's own or a corner
/// edge is shorter than the trim, a trim past the corner's vertex that is
/// no such lengthening or whose stretch leaves its face, or a closed edge's torus would not be a
/// ring torus, a contact reaches the axis or a cone's apex, or its seam is
/// shorter than the trim, or the third edge at
/// a chain's junction is shorter than the cut; a closed edge whose vertex
/// carries more than the curved faces' seams, or an open arc that meets
/// another blended edge at a vertex that is no tangent vertex, is
/// [`Reason::VertexBlend`];
/// [`OpError::Unsupported`] naming the two faces for a pair outside the
/// table (every pair but two planes, a plane and a cylinder along a
/// ruling, and, along a circle coaxial with both, a plane, a cylinder or a
/// cone against another of them, today), an edge the chain reached included, and
/// naming the face
/// across an end that is none of a plane, a cylinder and a cone, or one of
/// the last two that a contact misses or grazes or the tracer does not
/// decide, or, at an open arc's end, that is
/// none of a plane through the axis, a plane parallel to it and a cylinder
/// about a parallel axis — or one of those two the tracer does not decide,
/// a chamfer's cone on a plane off the axis among them;
/// [`OpError::NotFound`] for an edge id that does not resolve.
///
/// ```
/// use arris_ops::{fillet, primitive_box};
/// use arris_ops::arris_check::arris_topo::Model;
/// use arris_ops::arris_check::arris_topo::arris_math::Point3;
///
/// let mut m = Model::default();
/// let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &arris_ops::Control::NONE).unwrap();
/// // The vertical edge at x = 2, y = 2.
/// let edge = m.edges(cube).unwrap().into_iter().find(|e| {
///     let entity = m.edge(e.id).unwrap();
///     let (curve, range) = entity.curve().unwrap();
///     let mid = m.curve(curve).unwrap().point(range.midpoint());
///     (mid - Point3::new(2.0, 2.0, 1.0)).norm() < 1e-9
/// }).unwrap();
/// let (blended, provenance) = fillet(&mut m, cube, &[edge], 0.2, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(blended).unwrap().len(), 7, "six faces and the blend");
/// let generated = provenance.generated_from(edge.shape());
/// assert_eq!(generated.len(), 9, "the blend face, four edges and four vertices");
/// ```
pub fn fillet(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    radius: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    blend(m, body, edges, Kind::Fillet { radius }, control)
}

/// Cuts `edges` of `body` flat at `distance`: each edge's two faces are
/// replaced by the same faces cut back to the lines at `distance` from
/// the edge along each, the edge by the plane through those two lines,
/// and each end of the chamfer is trimmed by the face across the corner
/// as a [`fillet`]'s is — its vertex goes and its two other edges are
/// shortened to the segment across it (ADR-0007). The plane's frame has
/// its origin on one contact line, `X` across to the other and `Y` along
/// the edge, so the contacts sit at `u = 0` and at the chamfer's width
/// and `v` is the edge's own parameter; every curve is a line, exact on
/// every plane it lies on. Two chamfers meeting at a vertex whose third
/// edge stays sharp meet in the line from where their two contacts on
/// the shared face cross to where the other two meet the third edge,
/// which is shortened to that point; those two meet it at one point
/// exactly when the two edges make equal angles with it (a box corner,
/// any right prism). Three chamfers at a vertex of three planes, all convex
/// or all concave, meet in the triangle of the points where each face's
/// two contacts cross, each side a segment in one chamfer's plane, at any
/// such corner; the triangle and every corner point are recorded as a
/// fillet's sphere and points are. A plane and a cylinder or a cone along
/// a coaxial circle chamfer with no ends to the cone coaxial with it
/// through the two circles at `distance` from the edge along each face —
/// 45° against a cylinder — its `Z` toward the wider, its `u` seam the
/// ruling between the contacts' vertices, the curved face's seam
/// shortened as a fillet's is; an open arc chamfers to the same cone over
/// its range, each end on the cone's ruling in a plane through the axis,
/// or on a cylinder about a parallel axis off it on the cone's traced and
/// fitted section with it, as a [`fillet`]'s torus ends there.
/// Convex or concave is read from the dihedral: a
/// concave chamfer adds a prism. The result's ids are the same for any
/// order of the same edges.
///
/// Provenance, every record against the chamfered edge, as a
/// [`fillet`]'s: the chamfer face, its two contact edges, its two end
/// segments and the four trim vertices `Generated`; the edge's faces,
/// the faces across its ends and the corner edges the trim shortens
/// `Modified`; the edge and its corner vertices `Deleted`. The line where
/// two chamfers meet and its two vertices are `Generated` from both
/// edges. `arris_topo::provenance::audit` holds on every result.
///
/// Errors, the model untouched: a [`fillet`]'s, with
/// [`Reason::NonFinite`] or [`Reason::NotPositive`] naming the distance,
/// [`Reason::VertexBlend`] for two chamfers at a corner whose edges
/// make unequal angles with its third edge, and [`OpError::Unsupported`]
/// for a plane and a cylinder along a ruling, which fillets but has no
/// chamfer in the table.
///
/// ```
/// use arris_ops::measure::mass_properties;
/// use arris_ops::{chamfer, primitive_box};
/// use arris_ops::arris_check::arris_topo::Model;
/// use arris_ops::arris_check::arris_topo::arris_math::Point3;
///
/// let mut m = Model::default();
/// let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &arris_ops::Control::NONE).unwrap();
/// // The vertical edge at x = 2, y = 2.
/// let edge = m.edges(cube).unwrap().into_iter().find(|e| {
///     let entity = m.edge(e.id).unwrap();
///     let (curve, range) = entity.curve().unwrap();
///     let mid = m.curve(curve).unwrap().point(range.midpoint());
///     (mid - Point3::new(2.0, 2.0, 1.0)).norm() < 1e-9
/// }).unwrap();
/// let (chamfered, provenance) = chamfer(&mut m, cube, &[edge], 0.2, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(chamfered).unwrap().len(), 7, "six faces and the chamfer");
/// // A right prism cut away, its triangle's legs 0.2, two long.
/// let volume = mass_properties(&m, chamfered, &arris_ops::Control::NONE).unwrap().volume;
/// assert!((volume - 7.96).abs() < 1e-9);
/// assert_eq!(provenance.generated_from(edge.shape()).len(), 9);
/// ```
pub fn chamfer(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    distance: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    blend(m, body, edges, Kind::Chamfer { distance }, control)
}

/// What [`fillet`] and [`chamfer`] share: the input, the size and the
/// edge list checked before anything is built, then the build over the
/// edges in the body's order inside a transaction.
fn blend(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    kind: Kind,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, body)?;
    let b = body.shape();
    let (what, size) = match kind {
        Kind::Fillet { radius } => ("radius", radius),
        Kind::Chamfer { distance } => ("distance", distance),
    };
    if !size.is_finite() {
        return Err(degenerate(vec![b], Reason::NonFinite { what }));
    }
    if size <= 0.0 {
        return Err(degenerate(
            vec![b],
            Reason::NotPositive { what, value: size },
        ));
    }
    if edges.is_empty() {
        return Err(degenerate(vec![b], Reason::NoEdges));
    }
    let closure = m.closure(body)?;
    let mut selected: BTreeSet<EdgeId> = BTreeSet::new();
    for edge in edges {
        m.edge(edge.id)?;
        if !selected.insert(edge.id) {
            return Err(degenerate(vec![forward(edge.id)], Reason::RepeatedEdge));
        }
        if closure.edges.binary_search(&edge.id).is_err() {
            return Err(degenerate(vec![forward(edge.id), b], Reason::EdgeNotInBody));
        }
    }
    let ordered: Vec<EdgeId> = m
        .edges(body)?
        .into_iter()
        .map(|e| e.id)
        .filter(|id| selected.contains(id))
        .collect();
    let mut meter = Meter::new(control);
    m.transaction(|m| build(m, body, &ordered, kind, &mut meter))
}
