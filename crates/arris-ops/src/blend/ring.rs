//! A blend of a closed edge: a ring, its seam and its ends, on a torus or a cylinder.

use core::f64::consts::{PI, TAU};
use std::collections::BTreeSet;

use arris_geom::region2::Side;
use arris_geom::{Curve, Curve2, GeomKind, Surface, pcurve_on};
use arris_math::{
    Aabb, Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec3, Vec2, Vec3, shift_into_range,
};
use arris_topo::{EdgeId, FaceId, Model, Orientation, VertexId};

use super::ends::{EndKind, LoneCorner, Trim, arc_end, at_cut_corner, corner_of, cusp_trims};
use super::stripe::{chord, on_side_of_face, placed};
use super::{Kind, degenerate, invariant};
use super::{mixed, traced};
use crate::body_view::tangent_normals;
use crate::body_view::{BodyView, faces_tolerance};
use crate::error::{OpError, Reason, fault_of};
use crate::rebuild::forward;

/// A face of the meridian row (ADR-0036 §1): `Some(None)` for a plane,
/// `Some(Some(frame))` for a cylinder, a cone, a sphere or a torus, whose
/// `Z` is the axis when the face is coaxial with the edge, and `None` for
/// any other surface.
pub(super) fn row_frame(surface: &Surface) -> Option<Option<Frame>> {
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
pub(super) struct RingContact {
    pub(super) face: FaceId,
    pub(super) curve: Curve,
    /// The edge's range, or where an end off the axis trims this contact
    /// apart from the other (ADR-0037).
    pub(super) range: Interval,
    /// Its points at the range's start and end, one vertex on a closed
    /// edge.
    pub(super) points: [Point3; 2],
    pub(super) on_face: Curve2,
    /// Its pcurve on the blend, a line at constant `v` placed so that the
    /// range's start is at `u = 0` or `u = 2π`.
    pub(super) on_blend: Curve2,
    /// `true` when it runs with the blend's `u`.
    pub(super) along_u: bool,
}

/// One end of an open arc's blend (ADR-0035), trimmed by a plane through
/// the axis — the meridian of the torus there or the ruling of the cone —
/// or by a face off the axis, a plane parallel to it or a cylinder about
/// a parallel axis, on the section traced and fitted (ADR-0037); between
/// the two contacts' points, inserted into that face's loop between the
/// two corner edges it shortens.
pub(super) struct ArcEnd {
    pub(super) vertex: VertexId,
    pub(super) face: FaceId,
    /// The corner edge each contact's point cuts, by contact.
    pub(super) trims: [Trim; 2],
    pub(super) curve: Curve,
    pub(super) range: Interval,
    /// `true` when `range.lo()` is at the first contact's point.
    pub(super) lo_first: bool,
    /// Its pcurve on the face across, placed in that face loop's translate.
    pub(super) on_face: Curve2,
    /// Its pcurve on the blend, placed: a line at constant `u` on the
    /// section through the axis.
    pub(super) on_blend: Curve2,
    pub(super) tolerance: f64,
    /// The tolerance of each trim vertex, by contact.
    pub(super) vertex_tolerance: [f64; 2],
}

/// A closed edge's blend seam, from the first contact's vertex to the
/// second's, with its pcurves at `u = 0` then at `u = 2π`, and each
/// curved face's own seam shortened to its contact.
pub(super) struct RingSeam {
    pub(super) curve: Curve,
    pub(super) range: Interval,
    pub(super) on_blend: [Curve2; 2],
    /// Each curved face's seam, and the contact whose vertex it is cut at.
    pub(super) cuts: Vec<(Trim, usize)>,
    pub(super) vertex_tolerance: f64,
}

/// One end of an open arc's blend: trimmed by the face across, or a
/// junction at a tangent vertex, whose arc and vertices the junction holds
/// (ADR-0035 §3).
pub(super) enum RingEnd {
    Face(Box<ArcEnd>),
    Junction(VertexId),
}

/// How a circular edge's blend closes: a closed edge's runs a whole turn
/// round to its own seam, an open arc's ends at its two vertices.
pub(super) enum RingEnds {
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
pub(super) struct Ring {
    pub(super) edge: EdgeId,
    /// The edge's range, which both contacts keep.
    pub(super) range: Interval,
    pub(super) surface: Surface,
    pub(super) orientation: Orientation,
    pub(super) convex: bool,
    /// A fillet's circle of the ball's centre, its frame the edge's
    /// moved along the axis so that it runs as the edge runs; `None` on a
    /// chamfer.
    pub(super) centres: Option<Curve>,
    /// The contacts at the blend's lower `v`, then its upper.
    pub(super) contacts: [RingContact; 2],
    pub(super) ends: RingEnds,
    pub(super) tolerance: f64,
}

/// A face of revolution's meridian in the half-plane bounded by the axis,
/// `(ρ, h)`, the distance from the axis and the height along it from the
/// edge's centre: a line for a plane, a cylinder or a cone, a circle for a
/// sphere centred on the axis or a coaxial torus (ADR-0036 §1).
#[derive(Debug, Clone, Copy)]
pub(super) enum Trace {
    /// The line through `point` along the unit `along`.
    Line { point: Vec2, along: Vec2 },
    /// The circle about `centre` of `radius`.
    Circle { centre: Vec2, radius: f64 },
}

impl Trace {
    /// Where the two traces cross nearest `near`; `None` where they do
    /// not, or run parallel.
    pub(super) fn crossing(&self, other: &Trace, near: Vec2) -> Option<Vec2> {
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
pub(super) struct Meridian {
    pub(super) trace: Trace,
    /// The face's outward normal in the section at the edge, a unit
    /// vector; on a line, its normal everywhere.
    pub(super) normal: Vec2,
}

impl Meridian {
    /// The trace's unit tangent at the edge, either way along it.
    pub(super) fn along(&self) -> Vec2 {
        match self.trace {
            Trace::Line { along, .. } => along,
            Trace::Circle { .. } => Vec2::new(-self.normal.y, self.normal.x),
        }
    }

    /// The foot of `p` on the trace; `None` at a circle's centre.
    pub(super) fn foot(&self, p: Vec2) -> Option<Vec2> {
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
    pub(super) fn offset(&self, edge: Vec2, offset: f64) -> Option<Trace> {
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
    pub(super) fn inward(&self, other: &Meridian, s: f64) -> Vec2 {
        let along = self.along();
        along * (s * other.normal.dot(&along)).signum()
    }
}

/// `pcurve` translated by whole turns in `u` and `v` so that its point at
/// `t` is nearest `target`: a periodic surface's pcurve placed in the
/// translate of the blend's loop.
pub(super) fn placed_uv(pcurve: Curve2, t: f64, target: Point2) -> Curve2 {
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
pub(super) fn across_of(
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
pub(super) enum Across {
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
    pub(super) fn meet(
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
/// a corner edge lengthened past the vertex at a mixed corner (ADR-0038),
/// and at a cusp the contact on the wall tangent to that face trimmed on
/// the spine at the vertex's angle and the other on the edge's side of it
/// (ADR-0042) — or one of the `junctions`, which the junction builds. A torus that is not a ring
/// torus and a contact that reaches the axis or a cone's apex are
/// `Reason::BlendTooLarge`,
/// as is a contact or an end that leaves its face or a seam or a corner
/// edge shorter than the trim.
#[allow(clippy::too_many_arguments)]
pub(super) fn ring(
    m: &Model,
    view: &BodyView,
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
    // The edge's own tolerance widens the test as a stripe's does
    // (ADR-0045): a ring builds nothing where it is tangent.
    let tangent_tolerance = faces_tolerance(m, f1, f2)?.max(entity.tolerance());
    if tangent_normals(n1, n2, kind.size(), tangent_tolerance, tol) {
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
    let tolerance = faces_tolerance(m, f1, f2)?;
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
    let mut corners: [Option<LoneCorner>; 2] = [None, None];
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
                corners[j] = Some(
                    corner_of(
                        m,
                        view,
                        edge,
                        &contact_uses,
                        vertex,
                        at_lo,
                        kind.size(),
                        tol,
                    )?
                    .lone(edge, vertex)?,
                );
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
        let Some((_, face, spine)) = corners[j] else {
            continue;
        };
        // A face of no family is refused at its end, after the contacts.
        let across_surface = m.surface(m.face(face)?.surface())?;
        across[j] = across_of(across_surface, rim.origin(), z, tol, tolerance);
        let Some(kind) = across[j].filter(|k| !matches!(k, Across::Axis)) else {
            continue;
        };
        let t_end = if at_lo { range.lo() } else { range.hi() };
        // At a cusp the vertex is on the line through both axes, so the
        // side is the edge's own, read at its midpoint: an open arc is
        // under a turn, so it lies wholly on one side (ADR-0042 §2).
        let side_point = if spine.is_some() {
            curve.point(mid)
        } else {
            curve.point(t_end)
        };
        for (slot, (_, _, contact, _)) in by_v.iter().enumerate() {
            // The contact on the wall tangent to the face across meets it
            // on the spine, at the vertex's own angle: the contact runs
            // on that wall as the edge does, `Q` (ADR-0042 §2).
            if spine == Some(slot) {
                continue;
            }
            let &Curve::Circle { frame, radius } = contact else {
                return Err(invariant("a contact circle"));
            };
            let point = kind
                .meet(frame.origin(), z, radius, side_point, tolerance)
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
            (Some(uv), _) => placed(on_face, range.lo(), uv.x, face_surface.period()[0]),
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
            let Some(tc) = shift_into_range(seam_range0, projection.t, seam_curve.period()) else {
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
            let Some((corner_edges, across_face, spine)) = corners[j] else {
                ends.push(RingEnd::Junction(vertex));
                continue;
            };
            let across_surface = m.surface(m.face(across_face)?.surface())?;
            let t = if at_lo { range.lo() } else { range.hi() };
            let points = [contacts[0].points[j], contacts[1].points[j]];
            // The corner edges cut or lengthened to the trim points, and
            // the side of the face across the end lies on (ADR-0038); at a
            // cusp, `Q` checked on the spine first.
            let corner = match spine {
                Some(k) => {
                    let spine_edge = corner_edges[k];
                    let (curve, _) = m
                        .edge(spine_edge)?
                        .curve()
                        .ok_or(invariant("the cusp's spine's curve"))?;
                    let off = m.curve(curve)?.project(points[k]).map_err(geometry)?;
                    if off.distance > m.edge(spine_edge)?.tolerance().max(tolerance) {
                        return Err(degenerate(
                            vec![e, forward(spine_edge), forward(vertex)],
                            Reason::TangentChain,
                        ));
                    }
                    cusp_trims(m, edge, corner_edges, vertex, points)?
                }
                None => mixed::corner_trims(
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
                )?,
            };
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
pub(super) fn vertex_tolerance_of(ends: &[EndKind; 2], k: usize) -> f64 {
    ends.iter()
        .filter_map(|end| end.vertex_tolerance(k))
        .fold(0.0, f64::max)
}
