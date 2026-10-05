//! A blend's end at a corner whose two edges differ in convexity
//! (ADR-0038): a blend running into a step. The trim on the corner edge of
//! the blend's own convexity lies past the vertex, on that edge's
//! extension, and the edge is lengthened to it along its own curve; the
//! trim on the other is within its edge, which is cut short there as at
//! any corner; the end arc lies outside the face across, which takes the
//! region between the arc and the corner. The extension has to lie inside
//! the face it runs down — the face of the blended edge that edge shares —
//! or the blend has run over (`BlendReason::TooLarge`).

use arris_geom::region2::Side;
use arris_geom::{Curve, pcurve_on};
use arris_math::{Interval, Meter, Point3, Tolerance, shift_into_range};
use arris_topo::{EdgeId, FaceId, Model, VertexId};

use super::build::Env;
use super::ends::{Trim, cut_corner};
use super::ring::placed_uv;
use super::stripe::on_side_of_face;
use super::{degenerate, invariant};
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild::forward;

/// A blend's end against the two edges of its corner: where each is cut
/// or lengthened, which one is lengthened, and the side of the face across
/// the end arc lies on.
#[derive(Debug, Clone, Copy)]
pub(super) struct CornerTrims {
    /// By contact, as the corner edges are: each corner edge moved to its
    /// trim point at its end at the vertex.
    pub(super) trims: [Trim; 2],
    /// The contact whose corner edge is lengthened past the vertex, at a
    /// corner of mixed convexity; `None` where both are cut.
    pub(super) lengthened: Option<usize>,
    /// Inside the face across where both corner edges have the blend's
    /// convexity — the blend takes the corner from it — and outside it
    /// otherwise, the face taking the region the arc bounds (ADR-0007,
    /// ADR-0038).
    pub(super) side: Side,
}

/// The blend end whose corner [`corner_trims`] cuts: the blended `edge`
/// and its two `faces`, the two corner edges at `vertex`.
#[derive(Clone, Copy)]
pub(super) struct CornerAt {
    pub(super) edge: EdgeId,
    pub(super) faces: [FaceId; 2],
    pub(super) corner_edges: [EdgeId; 2],
    pub(super) vertex: VertexId,
}

/// The corner edges `corner_edges` of a blend of `edge` at `vertex` moved
/// to the trim points `points`, by contact: the edge's two faces `faces`
/// carry them in the same order, and `convex` is the blend's.
///
/// Each trim point is located on its corner edge's curve. Within the edge
/// it cuts the edge short, clear of its far vertex (`cut_corner`). Past
/// the vertex it lengthens the edge along its curve, which only a corner
/// of mixed convexity does, and only for its edge of the blend's own
/// convexity; the curve must be analytic — a fitted curve has no
/// extension — and the stretch from the vertex to the trim point must lie
/// inside the face of `faces` the corner edge bounds, at `samples`
/// interior parameters by the face's own domain. Anything else is
/// `BlendReason::TooLarge` naming `edge` and the corner edge: the trim
/// past the far vertex, past the vertex at a corner that is not mixed or
/// on the wrong edge of one, or a stretch that leaves its face.
pub(super) fn corner_trims(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    at: &CornerAt,
    points: [Point3; 2],
    convex: bool,
) -> Result<CornerTrims, OpError> {
    let (m, view) = (env.m, env.view);
    let (tol, samples) = (env.tol, env.samples);
    let CornerAt {
        edge,
        faces,
        corner_edges,
        vertex,
    } = *at;
    let convexity = |c: EdgeId| {
        view.convex(m, c)?
            .ok_or(invariant("a corner edge's convexity"))
    };
    let corners_convex = [convexity(corner_edges[0])?, convexity(corner_edges[1])?];
    let mixed = corners_convex[0] != corners_convex[1];
    let side = if corners_convex == [convex; 2] {
        Side::Inside
    } else {
        Side::Outside
    };
    let mut trims = [Trim {
        edge,
        t: 0.0,
        cuts_lo: true,
    }; 2];
    let mut lengthened = None;
    for k in 0..2 {
        let corner = corner_edges[k];
        let too_large = || {
            degenerate(
                vec![forward(edge), forward(corner)],
                Reason::Blend(BlendReason::TooLarge),
            )
        };
        let past = past_vertex(m, corner, vertex, points[k])?;
        let Some(t) = past else {
            trims[k] = cut_corner(m, edge, corner, vertex, points[k])?;
            continue;
        };
        if !mixed || corners_convex[k] != convex {
            return Err(too_large());
        }
        let entity = *m.edge(corner)?;
        let (curve_id, range) = entity.curve().ok_or(invariant("a corner edge's curve"))?;
        let curve = m.curve(curve_id)?;
        if matches!(curve, Curve::Nurbs(_)) {
            return Err(too_large());
        }
        let cuts_lo = entity.start() == vertex;
        let (at_vertex, stretch) = if cuts_lo {
            (range.lo(), Interval::new(t, range.lo()))
        } else {
            (range.hi(), Interval::new(range.hi(), t))
        };
        let stretch = stretch.map_err(|_| too_large())?;
        // The stretch on the face it runs down, in the translate of the
        // corner edge's own pcurve there.
        let face = faces[k];
        let own = view
            .uses
            .get(&corner)
            .and_then(|u| u.iter().find(|u| u.face == face))
            .ok_or(invariant("the corner edge on the blended edge's face"))?;
        let surface = m.surface(m.face(face)?.surface())?;
        let stretch_tol = Tolerance::new(entity.tolerance(), tol.angular);
        let on_face = pcurve_on(curve, stretch, surface, stretch_tol, meter).map_err(fault_of)?;
        let on_face = placed_uv(on_face, at_vertex, m.curve2(own.pcurve)?.point(at_vertex));
        if !on_side_of_face(m, face, &on_face, stretch, Side::Inside, samples)? {
            return Err(too_large());
        }
        trims[k] = Trim {
            edge: corner,
            t,
            cuts_lo,
        };
        lengthened = Some(k);
    }
    Ok(CornerTrims {
        trims,
        lengthened,
        side,
    })
}

/// The parameter of `point` on the curve of `corner` when it lies past the
/// edge's end at `vertex`, on its extension: before `range.lo()` at its
/// start, after `range.hi()` at its end, and on a closed curve the nearer
/// way round from that end than from the other. `None` where it lies
/// within the edge or past its far end, which `cut_corner` decides; an
/// edge closed on `vertex` has no extension.
fn past_vertex(
    m: &Model,
    corner: EdgeId,
    vertex: VertexId,
    point: Point3,
) -> Result<Option<f64>, OpError> {
    let entity = *m.edge(corner)?;
    let Some((curve_id, range)) = entity.curve() else {
        return Ok(None);
    };
    if entity.start() == entity.end() {
        return Ok(None);
    }
    let curve = m.curve(curve_id)?;
    let projection = curve.project(point).map_err(fault_of)?;
    if projection.distance > entity.tolerance() {
        return Err(invariant("the corner edge's curve through the trim point"));
    }
    let t = projection.t;
    if shift_into_range(range, t, curve.period()).is_some() {
        return Ok(None);
    }
    let cuts_lo = entity.start() == vertex;
    Ok(match curve.period() {
        None => (if cuts_lo {
            t < range.lo()
        } else {
            t > range.hi()
        })
        .then_some(t),
        Some(p) => {
            // How far before the start and after the end, each one way.
            let before = (range.lo() - t).rem_euclid(p);
            let after = (t - range.hi()).rem_euclid(p);
            if cuts_lo && before < after {
                Some(range.lo() - before)
            } else if !cuts_lo && after < before {
                Some(range.hi() + after)
            } else {
                None
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use crate::body_view::BodyView;
    use arris_geom::{Profile, ProfileLoop, ProfileSegment, Surface};
    use arris_math::nalgebra::{Unit, UnitQuaternion};
    use arris_math::{Control, Frame, Point2, Vec3};
    use arris_topo::{Body, Shape};

    use super::super::Kind;
    use super::super::ends::corner_of;
    use super::super::stripe::{Stripe, line_origin, stripe};
    use super::*;
    use crate::extrude;

    /// The poses a step is built in: at the origin, and turned and moved a
    /// hundred of its sizes away.
    fn poses() -> [Frame; 2] {
        let turn =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vec3::new(0.3, -0.5, 0.8)), 1.1);
        [
            Frame::world(),
            Frame::from_rotation(Point3::new(120.0, -75.0, 40.0), &turn),
        ]
    }

    /// An L in the pose's xz plane extruded 1.5 along its y: a low block
    /// x 0..2, z 0..1 beside a tall one x −1..0, z 0..`top`, the step's
    /// face from (0, 1) to (`lean`, `top`) — upright at `lean` 0 — and the
    /// front cap at y = 0 one L-shaped face, with `holes` through it.
    fn step(m: &mut Model, pose: &Frame, lean: f64, top: f64, holes: Vec<ProfileLoop>) -> Body {
        let p = |u, v| Point2::new(u, v);
        let profile = Profile {
            // Normal −y and x along x: the sketch's v is the pose's z.
            plane: Frame::new(
                pose.origin(),
                pose.vec_to_world(-Vec3::y()),
                pose.vec_to_world(Vec3::x()),
            )
            .unwrap(),
            outer: ProfileLoop::Path {
                start: p(-1.0, 0.0),
                segments: vec![
                    ProfileSegment::LineTo(p(2.0, 0.0)),
                    ProfileSegment::LineTo(p(2.0, 1.0)),
                    ProfileSegment::LineTo(p(0.0, 1.0)),
                    ProfileSegment::LineTo(p(lean, top)),
                    ProfileSegment::LineTo(p(-1.0, top)),
                    ProfileSegment::LineTo(p(-1.0, 0.0)),
                ],
            },
            holes,
        };
        let direction = pose.vec_to_world(Vec3::y());
        extrude(m, &profile, direction, 1.5, &Control::NONE)
            .unwrap()
            .0
    }

    /// The edge of `body` whose curve's midpoint is `at`.
    fn edge_at(m: &Model, body: Body, at: Point3) -> EdgeId {
        let found: Vec<EdgeId> = (m.edges(body).unwrap().iter())
            .map(|e| e.id)
            .filter(|&e| {
                let (c, range) = m.edge(e).unwrap().curve().unwrap();
                (m.curve(c).unwrap().point(range.midpoint()) - at).norm() < 1e-9
            })
            .collect();
        assert_eq!(found.len(), 1, "one edge at {at:?}");
        found[0]
    }

    /// Where each contact of `s` pierces the plane of `face` (the corner's
    /// face across), by contact.
    fn pierce_plane(m: &Model, s: &Stripe, face: FaceId) -> [Point3; 2] {
        let Surface::Plane { frame } = *m.surface(m.face(face).unwrap().surface()).unwrap() else {
            panic!("a plane across");
        };
        let n = frame.z().into_inner();
        [0, 1].map(|k| {
            let q = line_origin(&s.lines[k]).unwrap();
            q + ((frame.origin() - q).dot(&n) / s.d.dot(&n)) * s.d
        })
    }

    /// The blend of the low block's top front edge of a step at its end at
    /// the vertex `at`: the stripe, the corner and the trims.
    fn end_at(
        m: &Model,
        body: Body,
        pose: &Frame,
        kind: Kind,
        at: Point3,
    ) -> (
        Stripe,
        [EdgeId; 2],
        [Point3; 2],
        Result<CornerTrims, OpError>,
    ) {
        let tol = m.precision().tolerance();
        let view = BodyView::of(m, body).unwrap();
        let env = Env {
            m,
            view: &view,
            kind,
            tol,
            samples: m.precision().check_samples,
        };
        let edge = edge_at(m, body, pose.to_world(Point3::new(1.0, 0.0, 1.0)));
        let s = stripe(m, &view, edge, kind, tol).unwrap();
        let at = pose.to_world(at);
        let at_lo = (m.vertex(s.start).unwrap().point() - at).norm() < 1e-9;
        let vertex = if at_lo { s.start } else { s.end };
        assert!((m.vertex(vertex).unwrap().point() - at).norm() < 1e-9);
        let corner = corner_of(&env, edge, &s.uses, vertex, at_lo).unwrap();
        let (corner_edges, across) = (corner.edges, corner.pieces[0]);
        let points = pierce_plane(m, &s, across);
        let mut meter = Meter::new(&Control::NONE);
        let trims = corner_trims(
            &env,
            &mut meter,
            &CornerAt {
                edge,
                faces: s.faces,
                corner_edges,
                vertex,
            },
            points,
            s.convex,
        );
        (s, corner_edges, points, trims)
    }

    /// The point of `trim`'s edge at its trim parameter.
    fn at_trim(m: &Model, trim: &Trim) -> Point3 {
        let (c, _) = m.edge(trim.edge).unwrap().curve().unwrap();
        m.curve(c).unwrap().point(trim.t)
    }

    /// A fillet or a chamfer of the low block's top front edge, at the step
    /// (x = 0): the tall block's front vertical edge is convex, as the
    /// blend is, and lengthened `r` down the front cap past the vertex; the
    /// step's foot is concave and cut `r` from it; the arc lies outside the
    /// step's face. At the far end (x = 2), a box corner: both cut, the arc
    /// inside the face across.
    #[test]
    fn a_blend_into_an_upright_step_lengthens_its_convex_corner_edge() {
        let r = 0.3;
        for pose in poses() {
            for kind in [Kind::Fillet { radius: r }, Kind::Chamfer { distance: r }] {
                let mut m = Model::default();
                let body = step(&mut m, &pose, 0.0, 2.0, Vec::new());
                let corner = Point3::new(0.0, 0.0, 1.0);
                let (s, corner_edges, points, trims) = end_at(&m, body, &pose, kind, corner);
                let trims = trims.unwrap();
                let k = trims.lengthened.expect("the step's convex edge lengthened");
                assert_eq!(trims.side, Side::Outside);
                assert!(s.convex);
                let lengthened = pose.to_world(Point3::new(0.0, 0.0, 1.0 - r));
                let cut = pose.to_world(Point3::new(0.0, r, 1.0));
                for (j, want) in [(k, lengthened), (1 - k, cut)] {
                    assert_eq!(trims.trims[j].edge, corner_edges[j]);
                    let got = at_trim(&m, &trims.trims[j]);
                    assert!((got - want).norm() < 1e-9, "trim {j}: {got:?} for {want:?}");
                    assert!((points[j] - want).norm() < 1e-9);
                }
                // The lengthened edge reaches past its vertex end.
                let (_, range) = m.edge(corner_edges[k]).unwrap().curve().unwrap();
                let t = trims.trims[k].t;
                assert!(if trims.trims[k].cuts_lo {
                    t < range.lo()
                } else {
                    t > range.hi()
                });

                let far = Point3::new(2.0, 0.0, 1.0);
                let (_, _, _, trims) = end_at(&m, body, &pose, kind, far);
                let trims = trims.unwrap();
                assert_eq!(trims.lengthened, None);
                assert_eq!(trims.side, Side::Inside);
            }
        }
    }

    /// A step whose face leans back, from (0, 1) to (−0.4, 2): the face
    /// across is oblique to the blended edge, the tall block's front edge
    /// slanted, and the trim lies on that edge's line past the vertex, on
    /// the step's plane.
    #[test]
    fn a_blend_into_a_leaning_step_lengthens_along_the_slanted_edge() {
        let r = 0.3;
        for pose in poses() {
            for lean in [-0.4, 0.3] {
                let mut m = Model::default();
                let body = step(&mut m, &pose, lean, 2.0, Vec::new());
                let corner = Point3::new(0.0, 0.0, 1.0);
                let kind = Kind::Fillet { radius: r };
                let (_, corner_edges, points, trims) = end_at(&m, body, &pose, kind, corner);
                let trims = trims.unwrap();
                let k = trims.lengthened.expect("the slanted edge lengthened");
                let got = pose.to_local(at_trim(&m, &trims.trims[k]));
                assert!((pose.to_world(got) - points[k]).norm() < 1e-9);
                // On the slanted line's extension below z = 1, in the front cap.
                assert!(got.y.abs() < 1e-9 && got.z < 1.0);
                assert!((got.x - lean * (got.z - 1.0)).abs() < 1e-9, "{got:?}");
                assert_eq!(trims.trims[k].edge, corner_edges[k]);
            }
        }
    }

    /// A hole through the front cap just below the step's corner, across
    /// the stretch the convex corner edge would be lengthened down: the
    /// blend has run over, refused naming the blended edge and that corner
    /// edge.
    #[test]
    fn a_lengthening_across_a_hole_is_too_large() {
        let r = 0.3;
        for pose in poses() {
            let mut m = Model::default();
            let hole = ProfileLoop::Circle {
                center: Point2::new(0.0, 0.82),
                radius: 0.08,
            };
            let body = step(&mut m, &pose, 0.0, 2.0, vec![hole]);
            let corner = Point3::new(0.0, 0.0, 1.0);
            let kind = Kind::Fillet { radius: r };
            let (s, corner_edges, _, trims) = end_at(&m, body, &pose, kind, corner);
            let Err(OpError::Degenerate { entities, reason }) = trims else {
                panic!("refused: {trims:?}");
            };
            assert_eq!(reason, Reason::Blend(BlendReason::TooLarge));
            assert_eq!(entities[0], forward(s.edge));
            let named: Vec<Shape> = corner_edges.iter().map(|&c| forward(c)).collect();
            assert!(named.contains(&entities[1]), "{entities:?}");
        }
    }
}
