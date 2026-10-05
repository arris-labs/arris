//! The stripe of one blended edge: its section, contact lines and the ball between them.

use core::f64::consts::TAU;

use arris_check::domain::FaceDomain;
use arris_geom::region2::Side;
use arris_geom::{
    Curve, Curve2, GeomKind, MeetKind, Surface, SurfaceIntersection, SurfaceKind,
    intersect_surfaces, pcurve_on,
};
use arris_math::{
    Aabb, Frame, Interval, Meter, Point3, Tolerance, UnitVec3, Vec2, Vec3, shift_nearest,
};
use arris_topo::{EdgeId, FaceId, Model, VertexId};

use super::ends::stretch_between;
use super::traced;
use super::{Kind, degenerate, invariant};
use crate::body_view::tangent_normals;
use crate::body_view::{BodyView, UseAt, faces_tolerance};
use crate::error::{OpError, Reason, fault_of};
use crate::rebuild::forward;

/// A contact of the blend with one of the edge's faces: the line at
/// distance `r tan(φ/2)` from the edge, its `u` on the blend, and its
/// pcurves.
pub(super) struct Contact {
    pub(super) face: FaceId,
    pub(super) line: Curve,
    pub(super) range: Interval,
    pub(super) on_face: Curve2,
    pub(super) on_blend: Curve2,
    pub(super) tolerance: f64,
}

/// A stripe's cross-section, which its end curves are decided from.
#[derive(Debug, Clone, Copy)]
pub(super) enum Section {
    /// A fillet's cylinder of `radius` about the axis through
    /// `axis_origin`, a point level with the edge's line origin.
    Round { axis_origin: Point3, radius: f64 },
    /// A chamfer's plane, its frame's origin on the contact at `u = 0`
    /// and its `X` across to the other.
    Flat,
}

/// One edge's stripe: the blend surface with its two contact lines and
/// the faces they lie on, nothing about its ends decided yet.
pub(super) struct Stripe {
    pub(super) edge: EdgeId,
    pub(super) start: VertexId,
    pub(super) end: VertexId,
    /// The edge's direction: the cylinder's `Z`, the plane's `Y`.
    pub(super) d: Vec3,
    /// `true` when one of the edge's faces is a cylinder the edge is a
    /// ruling of, its contact there a ruling too.
    pub(super) ruling: bool,
    pub(super) section: Section,
    pub(super) frame: Frame,
    pub(super) surface: Surface,
    /// The angle the dihedral turns a ball through, `π − φ` for normals
    /// `φ` apart.
    pub(super) beta: f64,
    /// The `u` of the contact at the blend's far side: `β` on a fillet's
    /// cylinder, the chamfer's width on its plane.
    pub(super) u1: f64,
    pub(super) convex: bool,
    /// The contact lines, at `u = 0` then at `u = β`.
    pub(super) lines: [Curve; 2],
    /// The face each contact lies on, in the same order.
    pub(super) faces: [FaceId; 2],
    /// The edge's use by each of those faces, in the same order.
    pub(super) uses: [UseAt; 2],
    pub(super) tolerance: f64,
}

/// The parameters of the closest points of two lines with unit
/// directions, or `None` when the lines are parallel within `tol.angular`.
pub(super) fn lines_cross(
    o1: Point3,
    d1: Vec3,
    o2: Point3,
    d2: Vec3,
    tol: Tolerance,
) -> Option<(f64, f64)> {
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
pub(super) fn line_origin(line: &Curve) -> Result<Point3, OpError> {
    match line {
        Curve::Line { origin, .. } => Ok(*origin),
        Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Nurbs(_) => {
            Err(invariant("a contact line"))
        }
    }
}

/// `pcurve` translated by whole periods of the surface in `u` so that its
/// point at `t` has `u` nearest `target`: the blend loop is written in one
/// translate of the cylinder's domain, and `pcurve_on` reports `u` in
/// `[0, 2π)`. A surface with no period in `u` leaves it as it is.
pub(super) fn placed(pcurve: Curve2, t: f64, target: f64, period: Option<f64>) -> Curve2 {
    let Some(period) = period else {
        return pcurve;
    };
    let by = shift_nearest(pcurve.point(t).x, target, period);
    if by == 0.0 {
        pcurve
    } else {
        pcurve.translated(Vec2::new(by, 0.0))
    }
}

impl Stripe {
    /// `pcurve` on the blend placed so that its point at `t` has `u`
    /// nearest `target`: moved by whole turns on a fillet's cylinder, as
    /// it is on a chamfer's plane.
    pub(super) fn place(&self, pcurve: Curve2, t: f64, target: f64) -> Curve2 {
        match self.section {
            Section::Round { .. } => placed(pcurve, t, target, self.surface.period()[0]),
            Section::Flat => pcurve,
        }
    }
}

/// The segment from `a` to `b`: a line starting at `a` over
/// `[0, |b − a|]`.
pub(super) fn chord(a: Point3, b: Point3, tol: Tolerance) -> Result<(Curve, Interval), OpError> {
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
pub(super) fn on_side_of_face(
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
pub(super) fn arc_between(t_a: f64, t_b: f64, t_mid: f64) -> Result<(Interval, bool), OpError> {
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

/// The edge, the two faces and the ball [`ruling_ball`] fits: the edge
/// through `origin` along `d`, the plane's and the cylinder's outward
/// normals at it, the cylinder's `axis` and `big` radius, the ball's
/// `radius` and the side `s` it is on.
#[derive(Clone, Copy)]
pub(super) struct RulingBall<'a> {
    pub(super) origin: Point3,
    pub(super) d: Vec3,
    pub(super) n_plane: Vec3,
    pub(super) n_cylinder: Vec3,
    pub(super) axis: &'a Frame,
    pub(super) big: f64,
    pub(super) radius: f64,
    pub(super) s: f64,
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
pub(super) fn ruling_ball(ball: &RulingBall<'_>, tol: Tolerance) -> Option<(Point3, Vec3, Vec3)> {
    let RulingBall {
        origin,
        d,
        n_plane,
        n_cylinder,
        axis,
        big,
        radius,
        s,
    } = *ball;
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
pub(super) fn stripe(
    m: &Model,
    view: &BodyView,
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
    let tolerance = faces_tolerance(m, f1, f2)?;
    // A tangent dihedral has no corner to roll a ball into, whatever the
    // surfaces are. It builds no stripe, so the edge's own tolerance, which
    // is where a loose file puts the gap between its faces, may widen the
    // test (ADR-0045).
    if tangent_normals(n1, n2, kind.size(), tolerance.max(entity.tolerance()), tol) {
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
                        &RulingBall {
                            origin,
                            d,
                            n_plane: normals[1 - k],
                            n_cylinder: normals[k],
                            axis: &axis,
                            big,
                            radius,
                            s,
                        },
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

/// Whether `p` lies on the stripe's band between its contacts: `u` in
/// `[0, u₁]`, widened by `tolerance` — on a fillet's cylinder by the angle
/// it subtends.
pub(super) fn in_band(s: &Stripe, p: Point3, tolerance: f64) -> Result<bool, OpError> {
    Ok(match s.section {
        Section::Round { radius, .. } => {
            let u = s.surface.project(p).map_err(fault_of)?.uv.x;
            let slack = tolerance / radius;
            u <= s.u1 + slack || u >= TAU - slack
        }
        Section::Flat => {
            let u = s.frame.to_local(p).x;
            u >= -tolerance && u <= s.u1 + tolerance
        }
    })
}

/// The `u` of `p`, a point of the stripe's band, in the blend loop's
/// translate: a fillet's just below `0` rather than just below `2π`.
pub(super) fn band_u(s: &Stripe, p: Point3) -> Result<f64, OpError> {
    Ok(match s.section {
        Section::Round { .. } => {
            let u = s.surface.project(p).map_err(fault_of)?.uv.x;
            if u > (s.u1 + TAU) / 2.0 { u - TAU } else { u }
        }
        Section::Flat => s.frame.to_local(p).x,
    })
}

/// The section of the stripe `s` by a face across of `surface` from
/// `points[0]` to `points[1]`, each on both, on the stripe's band: a
/// chamfer's chord on a plane; a fillet's conic on a plane and a chamfer's
/// on a cylinder or a cone, the intersector's, exact; a fillet's quartic on
/// a cylinder or a cone, traced and fitted (ADR-0037). `refuse` — the
/// caller's `Unsupported` naming the blend and the face — where no single
/// stretch holds both points within `tol.linear` on the band, or the
/// surface is none of these.
pub(super) fn section_between(
    s: &Stripe,
    surface: &Surface,
    points: [Point3; 2],
    refuse: &dyn Fn() -> OpError,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<traced::TracedEnd, OpError> {
    let band = |p: Point3| in_band(s, p, tol.linear);
    let reach = match s.section {
        Section::Round { radius, .. } => 2.0 * radius,
        Section::Flat => s.u1,
    };
    let within = Aabb::of_point(points[0])
        .union(Aabb::of_point(points[1]))
        .inflated(reach);
    let exact = |meter: &mut Meter<'_>| -> Result<traced::TracedEnd, OpError> {
        let cut = intersect_surfaces(&s.surface, surface, &within, tol, meter).map_err(fault_of)?;
        let curves: Vec<Curve> = match cut {
            SurfaceIntersection::Meets { curves, .. } => curves
                .into_iter()
                .filter(|c| c.kind == MeetKind::Crossing)
                .map(|c| c.curve)
                .collect(),
            SurfaceIntersection::Empty | SurfaceIntersection::Coincident => Vec::new(),
        };
        let found = stretch_between(&curves, points, &band, tol.linear)?;
        let (curve, range, lo_first) = found.ok_or_else(refuse)?;
        let ends = if lo_first {
            [range.lo(), range.hi()]
        } else {
            [range.hi(), range.lo()]
        };
        let gaps = [0, 1].map(|k| (points[k] - curve.point(ends[k])).norm());
        Ok(traced::TracedEnd {
            curve,
            range,
            lo_first,
            gaps,
        })
    };
    match (surface, s.section) {
        (Surface::Plane { .. }, Section::Flat) => {
            let (curve, range) = chord(points[0], points[1], tol)?;
            Ok(traced::TracedEnd {
                curve,
                range,
                lo_first: true,
                gaps: [0.0; 2],
            })
        }
        // A plane meets a cylinder or a cone in a conic, which the
        // intersector writes exactly.
        (Surface::Plane { .. }, Section::Round { .. })
        | (Surface::Cylinder { .. } | Surface::Cone { .. }, Section::Flat) => exact(meter),
        (Surface::Cylinder { .. } | Surface::Cone { .. }, Section::Round { .. }) => {
            traced::traced_end(
                [&s.surface, surface],
                points,
                &within,
                &band,
                refuse,
                tol,
                meter,
            )
        }
        (
            Surface::EllipticCylinder { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_),
            Section::Round { .. } | Section::Flat,
        ) => Err(refuse()),
    }
}

/// The contact lines of `s` between their parameters at its two ends,
/// `t[end][contact]`, each checked to lie inside its face.
pub(super) fn contacts(
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
                placed(on_face, range.lo(), edge_u, surface.period()[0])
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
