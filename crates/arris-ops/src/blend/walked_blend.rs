//! A walked stripe assembled into a blend (ADR-0062): the edge's two faces
//! have no closed form, so the ball's spine and the NURBS surface skinned
//! through its arcs (`walked`) stand in for the table's cylinder or torus,
//! and the rest of the assembly takes them as it takes a table stripe.
//!
//! An open edge's stripe ([`walked_stripe`]) is a [`Stripe`] whose section
//! is [`Section::Walked`]: its surface runs `u` across from the contact on
//! the edge's first face to the one on the second, `v` along the edge's own
//! direction, and its two contact isocurves are the stripe's lines. A face
//! across an end cuts it where each contact pierces that face, on the
//! section traced and fitted between the two ([`walked_end`], ADR-0037).
//! A closed edge's ring ([`walked_ring`]) keeps the surface as it is, `u`
//! across and `v` once round the edge, and runs round to its own seam, the
//! ball's arc at the edge's vertex, as the table's ring does.

use arris_geom::region2::Side;
use arris_geom::{
    Curve, Curve2, CurveSurfaceIntersection, GeomError, GeomKind, NurbsCurve, NurbsSurface,
    Surface, SurfaceKind, intersect_curve_surface, pcurve_on,
};
use arris_math::{Aabb, Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec2, Vec2};
use arris_topo::entity::Edge;
use arris_topo::{EdgeId, FaceId, Orientation};

use super::build::Env;
use super::ends::{Arced, End, corner_of, finish_end};
use super::ring::{Ring, RingContact, RingEnds, RingSeam, SeamCut, cut_seams, placed_uv};
use super::stripe::{Contact, Section, Stripe, in_band, on_side_of_face, placed};
use super::traced::traced_end;
use super::walked::{Refusal, Rolling, Spine, Walked, arc, contacts as contact_rows, spine};
use super::{degenerate, invariant};
use crate::body_view::{UseAt, faces_tolerance, tangent_normals};
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild::forward;

/// An edge read for a walked blend: its faces, the ball's spine on them
/// and what the assembly needs of the edge.
struct Rolled<'a> {
    entity: Edge,
    /// The faces and the edge's uses by them, in the surface's order:
    /// `u = 0` is the contact on the first.
    faces: [FaceId; 2],
    uses: [UseAt; 2],
    surfaces: [&'a Surface; 2],
    convex: bool,
    /// The faces' tolerance, the stripe's.
    tolerance: f64,
    /// The tolerance the spine and the surface are fitted to.
    fit: Tolerance,
    spine: Spine,
}

/// The refusal the table's closed forms make for a pair, named for a
/// walked stripe that cannot decide it.
fn unsupported(surfaces: [&Surface; 2], faces: [FaceId; 2]) -> OpError {
    OpError::Unsupported {
        a: (GeomKind::Surface(surfaces[0].kind()), forward(faces[0])),
        b: (GeomKind::Surface(surfaces[1].kind()), forward(faces[1])),
    }
}

/// What a spine or a surface that was not built is, named for the edge:
/// `BlendReason::TooLarge` where the radius reaches a principal radius,
/// `Unsupported` naming the faces where the pose is none the offsets and
/// their trace decide.
fn named(walked: Walked, edge: EdgeId, faces: [FaceId; 2], surfaces: [&Surface; 2]) -> OpError {
    match walked {
        Walked::Op(e) => e,
        Walked::Refused(Refusal::TooLarge) => degenerate(
            vec![forward(edge), forward(faces[0]), forward(faces[1])],
            Reason::Blend(BlendReason::TooLarge),
        ),
        Walked::Refused(Refusal::NoOffset(_) | Refusal::NoSpine { .. }) => {
            unsupported(surfaces, faces)
        }
    }
}

/// Reads `edge` and walks the ball along it: its two faces by the edge's
/// uses, the dihedral's convexity, and the spine at the blend's radius.
/// A fillet only.
///
/// Errors: `BlendReason::VertexBlend` for an edge with no curve,
/// `BlendReason::TangentChain` where the faces are tangent along it,
/// `Unsupported` naming the faces where both are NURBS (that edge's spine
/// is NURBS against NURBS) or the spine is not decided, and
/// `BlendReason::TooLarge` where the radius reaches a face's principal
/// radius on the ball's side.
fn roll<'a>(env: &Env<'a>, meter: &mut Meter<'_>, edge: EdgeId) -> Result<Rolled<'a>, OpError> {
    let (m, view) = (env.m, env.view);
    let (tol, kind) = (env.tol, env.kind);
    let e = forward(edge);
    let entity = *m.edge(edge)?;
    let Some((curve_id, range)) = entity.curve() else {
        return Err(degenerate(vec![e], Reason::Blend(BlendReason::VertexBlend)));
    };
    let uses = view
        .uses
        .get(&edge)
        .filter(|u| u.len() == 2)
        .ok_or(invariant("two uses of the blended edge"))?;
    let (ua, ub) = (uses[0], uses[1]);
    let faces = [ua.face, ub.face];
    let [f1, f2] = faces;
    let mid = range.midpoint();
    let pcurves = [m.curve2(ua.pcurve)?, m.curve2(ub.pcurve)?];
    let n1 = view.outward(m, f1, pcurves[0].point(mid))?;
    let n2 = view.outward(m, f2, pcurves[1].point(mid))?;
    let tolerance = faces_tolerance(m, f1, f2)?;
    if tangent_normals(n1, n2, kind.size(), tolerance.max(entity.tolerance()), tol) {
        return Err(degenerate(
            vec![e, forward(f1), forward(f2)],
            Reason::Blend(BlendReason::TangentChain),
        ));
    }
    let surfaces = [
        m.surface(m.face(f1)?.surface())?,
        m.surface(m.face(f2)?.surface())?,
    ];
    if surfaces.iter().all(|s| matches!(s, Surface::Nurbs(_))) {
        return Err(unsupported(surfaces, faces));
    }
    let convex = view
        .convex(m, edge)?
        .ok_or(invariant("a blended edge with a dihedral"))?;
    let side = if convex { -1.0 } else { 1.0 };
    let radius = kind.size();
    let distances = faces.map(|f| side * radius * view.orientation[&f].sign());
    let fit = Tolerance::new(tolerance, tol.angular);
    let rolling = Rolling {
        surfaces,
        distances,
        edge: m.curve(curve_id)?,
        range,
        closed: entity.start() == entity.end(),
        tangency: tolerance.max(entity.tolerance()),
    };
    let spine = spine(&rolling, fit, meter).map_err(|w| named(w, edge, faces, surfaces))?;
    Ok(Rolled {
        entity,
        faces,
        uses: [ua, ub],
        surfaces,
        convex,
        tolerance,
        fit,
        spine,
    })
}

/// Whether the surface's normal `∂u × ∂v` at `(u, v)` points away from the
/// ball's centre `centre`.
fn faces_away(surface: &NurbsSurface, uv: Point2, centre: Point3) -> Result<bool, OpError> {
    let p = surface.eval(uv.x, uv.y).point;
    let n = surface
        .normal(uv.x, uv.y)
        .ok_or(invariant("a regular blend surface"))?;
    Ok(n.dot(&(p - centre)) > 0.0)
}

/// The open edge `edge`'s walked stripe: the spine, the surface skinned
/// through the ball's arcs along it and its two contact curves, no end
/// decided yet. A fillet's only.
///
/// Errors: as [`roll`]; `BlendReason::TangentChain` naming the vertex
/// where the faces turn tangent within the edge's tolerance, a vanishing
/// end, which the assembly does not take yet.
pub(super) fn walked_stripe(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    edge: EdgeId,
) -> Result<Stripe, OpError> {
    let m = env.m;
    meter.tick()?;
    let rolled = roll(env, meter, edge)?;
    let Rolled {
        entity,
        faces,
        uses,
        surfaces,
        convex,
        tolerance,
        fit,
        ref spine,
        ..
    } = rolled;
    if entity.start() == entity.end() {
        return Err(invariant("an open edge, a closed one being a ring"));
    }
    let e = forward(edge);
    for (vertex, vanishes) in [entity.start(), entity.end()]
        .into_iter()
        .zip(spine.vanishing())
    {
        if vanishes {
            return Err(degenerate(
                vec![e, forward(vertex)],
                Reason::Blend(BlendReason::TangentChain),
            ));
        }
    }
    let skin = spine
        .skin(fit, meter)
        .map_err(|w| named(w, edge, faces, surfaces))?;
    let rows = contact_rows(&skin.surface).map_err(|w| named(w, edge, faces, surfaces))?;
    let [v0, v1] = skin.ends;
    let v_mid = 0.5 * (v0 + v1);
    let ball_at = |v: f64| {
        spine
            .ball(skin.spine_t(v))
            .map_err(|w| named(w, edge, faces, surfaces))
    };
    let ball = ball_at(v_mid)?;
    let centres = [ball_at(v0)?.centre, ball_at(v1)?.centre];
    let flipped = !faces_away(&skin.surface, Point2::new(0.5, v_mid), ball.centre)?;
    let (curve_id, range) = entity
        .curve()
        .ok_or(invariant("the blended edge's curve"))?;
    let curve = m.curve(curve_id)?;
    let d = curve.eval(range.midpoint()).d1;
    let d = d
        .try_normalize(0.0)
        .ok_or(invariant("an edge with a direction"))?;
    let frame = Frame::from_z(curve.point(range.midpoint()), d)?;
    Ok(Stripe {
        edge,
        start: entity.start(),
        end: entity.end(),
        d,
        ruling: false,
        section: Section::Walked {
            ends: skin.ends,
            flipped,
            centres,
        },
        frame,
        surface: Surface::Nurbs(skin.surface),
        beta: 0.0,
        u1: 1.0,
        convex,
        lines: rows.map(Curve::Nurbs),
        faces,
        uses,
        tolerance,
    })
}

/// The end of the walked stripe `s` at its start (`at_lo`) or its end
/// vertex, cut by the face across the corner: each contact curve is met
/// with that face where it crosses it nearest the ball of the edge's end,
/// the corner edges are cut there, and the arc between is the section of
/// the stripe's surface with the face, traced and fitted (ADR-0037,
/// ADR-0061).
///
/// Errors: `BlendReason::VertexBlend` for a fan across, or the vertex's
/// other edges not the corner's; `BlendReason::TangentChain` at a cusp;
/// `Unsupported` naming the stripe and the face across where that is a
/// NURBS face, a contact misses it or the section is not decided; the
/// errors of [`finish_end`].
pub(super) fn walked_end(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    s: &Stripe,
    at_lo: bool,
) -> Result<End, OpError> {
    let Section::Walked { ends, .. } = s.section else {
        return Err(invariant("a walked stripe's end"));
    };
    let (m, tol) = (env.m, env.tol);
    meter.tick()?;
    let (edge, e) = (s.edge, forward(s.edge));
    let vertex = if at_lo { s.start } else { s.end };
    let v_end = if at_lo { ends[0] } else { ends[1] };
    let mut corner = corner_of(env, edge, &s.uses, vertex, at_lo)?;
    if corner.pieces.len() > 1 {
        return Err(degenerate(
            vec![e, forward(vertex)],
            Reason::Blend(BlendReason::VertexBlend),
        ));
    }
    let stays = std::mem::take(&mut corner.stays);
    let (corner_edges, face3, cusp) = corner.lone(edge, vertex)?;
    if let Some(k) = cusp {
        return Err(degenerate(
            vec![e, forward(corner_edges[k]), forward(vertex)],
            Reason::Blend(BlendReason::TangentChain),
        ));
    }
    let surface3 = m.surface(m.face(face3)?.surface())?;
    let arc_tolerance = s.tolerance.max(m.face(face3)?.tolerance());
    let arc_tol = Tolerance::new(arc_tolerance, tol.angular);
    let refuse = || OpError::Unsupported {
        a: (GeomKind::Surface(SurfaceKind::Nurbs), e),
        b: (GeomKind::Surface(surface3.kind()), forward(face3)),
    };
    if matches!(surface3, Surface::Nurbs(_)) {
        return Err(refuse());
    }
    // Where each contact pierces the face across: of its hits, the one
    // nearest the ball of the edge's end.
    let mut points = [Point3::origin(); 2];
    let mut t = [0.0; 2];
    for k in 0..2 {
        let hits = match intersect_curve_surface(&s.lines[k], surface3, arc_tol, meter) {
            Ok(CurveSurfaceIntersection::Points(hits)) => hits,
            Ok(CurveSurfaceIntersection::Coincident)
            | Err(GeomError::Unsupported { .. } | GeomError::DegenerateSection { .. }) => {
                return Err(refuse());
            }
            Err(other) => return Err(fault_of(other)),
        };
        let hit = hits
            .iter()
            .min_by(|a, b| (a.t - v_end).abs().total_cmp(&(b.t - v_end).abs()))
            .ok_or_else(refuse)?;
        t[k] = hit.t;
        points[k] = hit.point;
    }
    let chord = (points[1] - points[0]).norm();
    let within = Aabb::of_point(points[0])
        .union(Aabb::of_point(points[1]))
        .inflated(2.0 * chord);
    let band = |p: Point3| in_band(s, p, arc_tolerance);
    let traced = traced_end(
        [&s.surface, surface3],
        points,
        &within,
        &band,
        &refuse,
        arc_tol,
        meter,
    )?;
    finish_end(
        env,
        meter,
        s,
        Arced {
            vertex,
            corner_edges,
            face3,
            surface3,
            spine: None,
            stays,
            points,
            t,
            gaps: traced.gaps,
            arc_curve: traced.curve,
            arc_range: traced.range,
            lo_first: traced.lo_first,
            arc_tolerance,
        },
    )
}

/// The pcurve of an isocurve of a walked stripe's surface: `u` constant
/// at `u`, `v` its own parameter.
fn along_v(u: f64) -> Curve2 {
    Curve2::Line {
        origin: Point2::new(u, 0.0),
        direction: UnitVec2::new_unchecked(Vec2::y()),
    }
}

/// The pcurve of the surface's isocurve at constant `v`, `u` its
/// own parameter.
fn along_u(v: f64) -> Curve2 {
    Curve2::Line {
        origin: Point2::new(0.0, v),
        direction: UnitVec2::new_unchecked(Vec2::x()),
    }
}

/// The contact curves of the walked stripe `s` between their parameters at
/// its two ends, `t[end][contact]`, each checked to lie inside its face.
/// The blend's pcurve of a contact is exact: the surface's isocurve `u = 0`
/// or `u = 1`.
///
/// Errors: `BlendReason::TooLarge` where a contact is no longer than the
/// tolerance between its ends or leaves its face.
pub(super) fn walked_contacts(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    s: &Stripe,
    t: [[f64; 2]; 2],
) -> Result<[Contact; 2], OpError> {
    let (m, tol, samples) = (env.m, env.tol, env.samples);
    meter.tick()?;
    let e = forward(s.edge);
    let edge_mid = m
        .edge(s.edge)?
        .curve()
        .ok_or(invariant("the blended edge's curve"))?
        .1
        .midpoint();
    let mut contacts: Vec<Contact> = Vec::with_capacity(2);
    for (k, (&face, line)) in s.faces.iter().zip(&s.lines).enumerate() {
        let too_large = || degenerate(vec![e, forward(face)], Reason::Blend(BlendReason::TooLarge));
        let (lo, hi) = (t[0][k], t[1][k]);
        if hi - lo <= s.tolerance {
            return Err(too_large());
        }
        let range = Interval::new(lo, hi).map_err(|_| too_large())?;
        let surface = m.surface(m.face(face)?.surface())?;
        let line_tol = Tolerance::new(s.tolerance, tol.angular);
        let on_face = pcurve_on(line, range, surface, line_tol, meter).map_err(fault_of)?;
        // On a periodic face, in the translate of the face's own loop: by
        // whole turns to the `u` of the blended edge's pcurve at its middle,
        // read at the edge's own parameter, which the contact's `v` is not.
        let on_face = match surface.period()[0] {
            Some(period) => {
                let edge_u = m.curve2(s.uses[k].pcurve)?.point(edge_mid).x;
                placed(on_face, range.midpoint(), edge_u, Some(period))
            }
            None => on_face,
        };
        if !on_side_of_face(m, face, &on_face, range, Side::Inside, samples)? {
            return Err(too_large());
        }
        contacts.push(Contact {
            face,
            line: line.clone(),
            range,
            on_face,
            on_blend: along_v(k as f64),
            tolerance: s.tolerance,
        });
    }
    contacts
        .try_into()
        .map_err(|_| invariant("two contacts of the blend"))
}

/// The surface's isocurve at its lowest `v`, across: the first column of
/// the net, exact since the knots are clamped.
fn first_column(surface: &NurbsSurface) -> Result<NurbsCurve, GeomError> {
    let [n, m] = surface.counts();
    let [p, _] = surface.degree();
    let points = (0..n).map(|i| surface.control_points()[i * m]).collect();
    let weights = (0..n).map(|i| surface.weights()[i * m]).collect();
    NurbsCurve::new(p, surface.knots()[0].to_vec(), points, weights)
}

/// The walked ring of the closed edge `edge`: the surface (transposed, so
/// that `u` runs along the edge as a torus's does and `v` from the contact
/// on the first face to the one on the second), the two contact loops, and
/// the seam, the ball's arc at the edge's vertex, from the first contact's
/// vertex to the second's, with each face's own seam at the vertex cut to
/// its contact.
///
/// Errors: as [`roll`]; `BlendReason::VertexBlend` for a vertex that
/// carries an edge that is no seam of one of the faces, and
/// `BlendReason::TooLarge` for a contact that leaves its face or a seam
/// shorter than the cut.
pub(super) fn walked_ring(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    edge: EdgeId,
) -> Result<Ring, OpError> {
    let (m, view, tol, samples) = (env.m, env.view, env.tol, env.samples);
    meter.tick()?;
    let rolled = roll(env, meter, edge)?;
    let Rolled {
        entity,
        faces,
        uses,
        surfaces,
        convex,
        tolerance,
        fit,
        ref spine,
        ..
    } = rolled;
    let e = forward(edge);
    let vertex = entity.start();
    let vertex_blend = || {
        degenerate(
            vec![e, forward(vertex)],
            Reason::Blend(BlendReason::VertexBlend),
        )
    };
    let too_large =
        |face: FaceId| degenerate(vec![e, forward(face)], Reason::Blend(BlendReason::TooLarge));
    let named = |w: Walked| named(w, edge, faces, surfaces);
    let skin = spine.skin(fit, meter).map_err(named)?;
    let rows = contact_rows(&skin.surface).map_err(named)?;
    let [lo, hi] = skin.ends;
    let range = Interval::new(lo, hi).map_err(|_| invariant("a closed edge's period"))?;
    let ball = spine.ball(skin.spine_t(lo)).map_err(named)?;
    let seam_curve = first_column(&skin.surface).map_err(fault_of)?;
    let seam_range = Interval::new(0.0, 1.0).map_err(|_| invariant("an arc's range"))?;
    // The arc at the vertex, exact, against the surface's own isocurve
    // there: they agree within the fit.
    debug_assert!({
        let exact = arc(&ball).map_err(named)?;
        (exact.eval(0.5).point - seam_curve.eval(0.5).point).norm() <= tolerance
    });
    // The blend faces out of the material: its normal against the ball's
    // radial direction.
    let orientation = {
        let outward = faces_away(&skin.surface, Point2::new(0.5, range.lo()), ball.centre)?;
        if outward == convex {
            Orientation::Forward
        } else {
            Orientation::Reversed
        }
    };
    let line_tol = Tolerance::new(tolerance, tol.angular);
    let mut contacts: Vec<RingContact> = Vec::with_capacity(2);
    for (k, row) in rows.into_iter().enumerate() {
        let face = faces[k];
        let face_surface = surfaces[k];
        let curve = Curve::Nurbs(row);
        let on_face = pcurve_on(&curve, range, face_surface, line_tol, meter).map_err(fault_of)?;
        let use_uv = m
            .curve2(uses[k].pcurve)?
            .point(entity.curve().map_or(0.0, |(_, r)| r.lo()));
        let on_face = match face_surface {
            Surface::Plane { .. } => on_face,
            Surface::Torus { .. } => placed_uv(on_face, range.lo(), use_uv),
            Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Nurbs(_) => placed(on_face, range.lo(), use_uv.x, face_surface.period()[0]),
        };
        if !on_side_of_face(m, face, &on_face, range, Side::Inside, samples)? {
            return Err(too_large(face));
        }
        contacts.push(RingContact {
            face,
            range,
            points: [curve.point(range.lo()), curve.point(range.hi())],
            curve,
            on_face,
            on_blend: along_v(k as f64),
            along_u: true,
        });
    }
    let contacts: [RingContact; 2] = contacts
        .try_into()
        .map_err(|_| invariant("two contacts of the blend"))?;
    // The vertex's other edges are the faces' own seams.
    let at_vertex = view
        .vertex_edges
        .get(&vertex)
        .ok_or(invariant("the closed edge's vertex's edges"))?;
    let others: Vec<EdgeId> = at_vertex.iter().copied().filter(|&x| x != edge).collect();
    if others.len() > 2 {
        return Err(vertex_blend());
    }
    let (cuts, vertex_tolerance) = cut_seams(
        env,
        &SeamCut {
            edge,
            vertex,
            faces,
            seamed: surfaces.map(|s| !matches!(s, Surface::Plane { .. })),
            cut_at: [contacts[0].points[0], contacts[1].points[0]],
            slots: [0, 1],
            tolerance,
        },
        others,
    )?;
    Ok(Ring {
        edge,
        range: entity
            .curve()
            .map(|(_, r)| r)
            .ok_or(invariant("the blended edge's range"))?,
        surface: Surface::Nurbs(skin.surface.clone()),
        orientation,
        convex,
        centres: None,
        contacts,
        ends: RingEnds::Seam(Box::new(RingSeam {
            curve: Curve::Nurbs(seam_curve),
            range: seam_range,
            on_blend: [along_u(range.lo()), along_u(range.hi())],
            cuts,
            vertex_tolerance,
        })),
        tolerance,
        across: true,
    })
}
