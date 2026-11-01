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
//! Where the faces turn tangent at an end and the ball there touches both
//! at the vertex, the stripe narrows to it instead, a pole of its surface
//! ([`walked_pole`], ADR-0062 §5).
//! A closed edge's ring ([`walked_ring`]) keeps the surface as it is, `u`
//! across and `v` once round the edge, and runs round to its own seam, the
//! ball's arc at the edge's vertex, as the table's ring does.

use arris_geom::region2::Side;
use arris_geom::{
    Curve, Curve2, CurveSurfaceIntersection, GeomError, GeomKind, NurbsCurve, NurbsSurface,
    SECTION_FIT_FRACTION, Surface, SurfaceKind, intersect_curve_surface, pcurve_on,
};
use arris_math::{Aabb, Frame, Interval, Meter, Point2, Point3, Tolerance};
use arris_topo::entity::{Edge, Vertex};
use arris_topo::{EdgeId, FaceId, Orientation};

use super::Kind;
use super::build::Env;
use super::ends::{Arced, End, Pole, Remnant, corner_of, finish_end, near_tangent_corner};
use super::ring::{Ring, RingContact, RingEnds, RingSeam, SeamCut, cut_seams, placed_uv};
use super::stripe::{Contact, Section, Stripe, in_band, on_side_of_face, placed};
use super::traced::traced_end;
use super::walked::{Refusal, Rolling, Skin, Spine, Walked, arc, contacts as contact_rows, spine};
use super::walked_chamfer::{Chord, Chording};
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
    path: Path,
}

/// What the stripe's surface is skinned through: the ball's arcs along a
/// fillet's spine, a chamfer's chords along its contact curves.
pub(super) enum Path {
    Ball(Box<Spine>),
    Chord(Box<Chord>),
}

impl Path {
    /// Whether the faces turn tangent at the edge's start and end: only a
    /// fillet's ball narrows to nothing there.
    fn vanishing(&self) -> [bool; 2] {
        match self {
            Path::Ball(spine) => spine.vanishing(),
            Path::Chord(_) => [false; 2],
        }
    }

    fn skin(&self, fit: Tolerance, meter: &mut Meter<'_>) -> Result<Skin, Walked> {
        match self {
            Path::Ball(spine) => spine.skin(fit, meter),
            Path::Chord(chord) => chord.skin(fit, meter),
        }
    }
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

/// Reads `edge` and walks the blend along it: its two faces by the edge's
/// uses, the dihedral's convexity, and a fillet's spine at the blend's
/// radius or a chamfer's contact curves at its distance.
///
/// Errors: `BlendReason::VertexBlend` for an edge with no curve,
/// `BlendReason::TangentChain` where the faces are tangent along it,
/// `Unsupported` naming the faces where the spine or a chamfer's contacts
/// are not decided — two NURBS faces' spine is the marched section of
/// their offset fits (ADR-0068) — and
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
    let convex = view
        .convex(m, edge)?
        .ok_or(invariant("a blended edge with a dihedral"))?;
    let radius = kind.size();
    let fit = Tolerance::new(tolerance, tol.angular);
    let closed = entity.start() == entity.end();
    let path = match kind {
        Kind::Fillet { .. } => {
            let side = if convex { -1.0 } else { 1.0 };
            let distances = faces.map(|f| side * radius * view.orientation[&f].sign());
            let rolling = Rolling {
                surfaces,
                distances,
                edge: m.curve(curve_id)?,
                range,
                closed,
                tangency: tolerance.max(entity.tolerance()),
                ends: [
                    m.vertex(entity.start())?.tolerance(),
                    m.vertex(entity.end())?.tolerance(),
                ],
            };
            Path::Ball(Box::new(
                spine(&rolling, fit, meter).map_err(|w| named(w, edge, faces, surfaces))?,
            ))
        }
        Kind::Chamfer { .. } => {
            // Into each face from the edge: its outward normal against the
            // edge's direction along the face's own use of it.
            let curve = m.curve(curve_id)?;
            let mut inward = [0.0; 2];
            for k in 0..2 {
                let uv = pcurves[k].point(mid);
                let n = surfaces[k]
                    .normal(uv.x, uv.y)
                    .ok_or(invariant("a normal at the edge of a chamfered face"))?;
                let outward = if k == 0 { n1 } else { n2 };
                let along = view.orientation[&faces[k]]
                    .compose(uses[k].orientation)
                    .sign();
                inward[k] = n.dot(&outward).signum() * along;
            }
            let chording = Chording {
                surfaces,
                inward,
                distance: radius,
                edge: curve,
                range,
                closed,
            };
            Path::Chord(Box::new(
                Chord::new(&chording, meter).map_err(|w| named(w, edge, faces, surfaces))?,
            ))
        }
    };
    Ok(Rolled {
        entity,
        faces,
        uses: [ua, ub],
        surfaces,
        convex,
        tolerance,
        fit,
        path,
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

/// Whether the surface's normal `∂u × ∂v` at `(u, v)` points toward the
/// point `p`: a walked chamfer's face looks at the edge it cuts as a
/// fillet's, away from its ball, looks out of the material.
fn faces_toward(surface: &NurbsSurface, uv: Point2, p: Point3) -> Result<bool, OpError> {
    let at = surface.eval(uv.x, uv.y).point;
    let n = surface
        .normal(uv.x, uv.y)
        .ok_or(invariant("a regular blend surface"))?;
    Ok(n.dot(&(p - at)) > 0.0)
}

/// The open edge `edge`'s walked stripe: the spine, the surface skinned
/// through the ball's arcs along it and its two contact curves, no end
/// decided yet; a chamfer's, the contact curves and the ruled surface
/// between them.
///
/// Errors: as [`roll`]; `BlendReason::TangentChain` naming the vertex
/// where the faces turn tangent within the edge's tolerance, a vanishing
/// end, the ball there does not close on that vertex, and the stripe
/// cannot stop short of it either ([`remnant`]).
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
        path,
        ..
    } = rolled;
    if entity.start() == entity.end() {
        return Err(invariant("an open edge, a closed one being a ring"));
    }
    let e = forward(edge);
    let skin = path
        .skin(fit, meter)
        .map_err(|w| named(w, edge, faces, surfaces))?;
    // A vanishing end is a pole where the ball there touches both faces at
    // one point, the vertex within its tolerance: the surface's row there
    // is collapsed to it, and the stripe narrows to the vertex, which it
    // keeps (ADR-0062 §5). Where the faces only run within the edge's
    // tolerance of tangent, the ball's arc there stays apart from the
    // vertex: the stripe stops short of it on that arc, and the edge's
    // remnant runs on to the vertex (ADR-0074).
    let singular = Surface::Nurbs(skin.surface.clone()).singularities();
    let mut poles = [false; 2];
    let mut remnants = [None; 2];
    for (end, (vertex, vanishes)) in [entity.start(), entity.end()]
        .into_iter()
        .zip(path.vanishing())
        .enumerate()
    {
        if !vanishes {
            continue;
        }
        let at = m.vertex(vertex)?;
        let closes = singular.iter().any(|row| {
            row.fixed == 1
                && row.value == skin.ends[end]
                && (row.point - at.point()).norm() <= at.tolerance()
        });
        if !closes {
            let share = SECTION_FIT_FRACTION * fit.linear;
            let Some(r) = remnant(
                env,
                &entity,
                at,
                &skin.surface,
                skin.ends[end],
                share,
                end == 0,
            )?
            else {
                return Err(degenerate(
                    vec![e, forward(vertex)],
                    Reason::Blend(BlendReason::TangentChain),
                ));
            };
            remnants[end] = Some(r);
        }
        poles[end] = true;
    }
    let rows = contact_rows(&skin.surface).map_err(|w| named(w, edge, faces, surfaces))?;
    let [v0, v1] = skin.ends;
    let v_mid = 0.5 * (v0 + v1);
    let (flipped, centres) = match &path {
        Path::Ball(spine) => {
            let ball_at = |v: f64| {
                spine
                    .ball(skin.spine_t(v))
                    .map_err(|w| named(w, edge, faces, surfaces))
            };
            let ball = ball_at(v_mid)?;
            let centres = [ball_at(v0)?.centre, ball_at(v1)?.centre];
            let flipped = !faces_away(&skin.surface, Point2::new(0.5, v_mid), ball.centre)?;
            (flipped, Some(centres))
        }
        Path::Chord(chord) => {
            let flipped = !faces_toward(
                &skin.surface,
                Point2::new(0.5, v_mid),
                chord.edge_point(v_mid),
            )?;
            (flipped, None)
        }
    };
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
            poles,
            remnants,
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
        path: Some((path, fit)),
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
/// NURBS face, a contact misses it or the section is not decided, and
/// `BlendReason::TangentChain` where a contact misses it beside a corner
/// edge tangent within the vertex's tolerance ([`near_tangent_corner`]);
/// the errors of [`finish_end`].
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
    // An end the section does not decide, or a contact that misses the face
    // across, beside a corner edge tangent within the vertex's tolerance is
    // a chain the walk takes (ADR-0075), not an unsupported pair.
    let refuse = || {
        near_tangent_corner(env, s, e, corner_edges, vertex)
            .err()
            .unwrap_or_else(|| OpError::Unsupported {
                a: (GeomKind::Surface(SurfaceKind::Nurbs), e),
                b: (GeomKind::Surface(surface3.kind()), forward(face3)),
            })
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
        // A contact that misses the face across, where that face runs on
        // from the stripe's within the vertex's tolerance of tangent, is a
        // chain the walk does not take, named as one (ADR-0062 step 10b).
        let Some(hit) =
            (hits.iter()).min_by(|a, b| (a.t - v_end).abs().total_cmp(&(b.t - v_end).abs()))
        else {
            return Err(refuse());
        };
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
            extras: Vec::new(),
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

/// Where the walked stripe whose surface ends on the row `v` of `surface`
/// at a vanishing end of `edge`, its start (`at_lo`) or its end, stops
/// short of that end's vertex (ADR-0074): the edge's point nearest the
/// row's middle, strictly inside the edge, and a tolerance there that is
/// the edge's own or the row's farthest point from it with the surface's
/// `share` of the tolerance, whichever is larger. `None` where that point
/// is not inside the edge, or the tolerance is past `max_tolerance`, unless
/// the point lies within the end `vertex`'s own tolerance: there that
/// tolerance is the ceiling, since the read recorded that the faces meet no
/// better than that and the cut adds no uncertainty of its own (ADR-0060 §4).
fn remnant(
    env: &Env<'_>,
    edge: &Edge,
    vertex: &Vertex,
    surface: &NurbsSurface,
    v: f64,
    share: f64,
    at_lo: bool,
) -> Result<Option<Remnant>, OpError> {
    let m = env.m;
    let (curve, range) = edge.curve().ok_or(invariant("the blended edge's curve"))?;
    let curve = m.curve(curve)?;
    let middle = surface.eval(0.5, v).point;
    let t = curve.project(middle).map_err(fault_of)?.t;
    let inside = range.lo() < t && t < range.hi();
    let toward_end = if at_lo {
        t < range.midpoint()
    } else {
        t > range.midpoint()
    };
    if !(inside && toward_end) {
        return Ok(None);
    }
    let point = curve.point(t);
    let n = env.samples.max(2);
    let reach = (0..n)
        .map(|i| (surface.eval(i as f64 / (n - 1) as f64, v).point - point).norm())
        .fold(0.0, f64::max);
    let ceiling = if (point - vertex.point()).norm() <= vertex.tolerance() {
        m.precision().max_tolerance.max(vertex.tolerance())
    } else {
        m.precision().max_tolerance
    };
    // The faces' pcurves of the edge stand off its curve by up to its
    // tolerance, and the remnant's end is moved onto the contact (`end_on_
    // kept_pcurves`): where the arc is off the edge by more than that
    // tolerance, the vertex holds the arc and the edge's own on top of it,
    // as far as the ceiling allows.
    let arc = reach + share;
    let tolerance = if arc > edge.tolerance() {
        arc + edge.tolerance().min((ceiling - arc).max(0.0))
    } else {
        edge.tolerance()
    };
    if tolerance > ceiling {
        return Ok(None);
    }
    Ok(Some(Remnant {
        t,
        point,
        tolerance,
    }))
}

/// The pole the walked stripe `s` narrows to at its start (`at_lo`) or its
/// end vertex, where [`walked_stripe`] found one (ADR-0062 §5), or where it
/// stops short of that vertex (ADR-0074).
pub(super) fn walked_pole(s: &Stripe, at_lo: bool) -> Result<Pole, OpError> {
    let Section::Walked {
        ends,
        poles,
        remnants,
        ..
    } = s.section
    else {
        return Err(invariant("a walked stripe's pole"));
    };
    let end = usize::from(!at_lo);
    if !poles[end] {
        return Err(invariant("a pole at the walked stripe's end"));
    }
    Ok(Pole {
        vertex: if at_lo { s.start } else { s.end },
        t: ends[end],
        on_blend: Curve2::along_u(ends[end]),
        remnant: remnants[end],
    })
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
            on_blend: Curve2::along_v(k as f64),
            tolerance: s.tolerance,
        });
    }
    contacts
        .try_into()
        .map_err(|_| invariant("two contacts of the blend"))
}

/// The walked stripe `s` with its surface ended at `cut`, the `v` of a
/// junction's arc at its start and at its end where the junction's tangent
/// edge leans (ADR-0072): refitted along its spine over the stretch kept,
/// so that each such end row is the shared ball's exact arc. The skin past
/// the shared ball, rolling on along the far face's extension, is no part
/// of the blend, and a face across that runs tangent to it there is not
/// asked to tell that reach from the arc. The surface keeps its `v`, so
/// the curves already placed on it — the junction's arc — keep theirs.
///
/// A walked chamfer is cut the same way, its chords refitted over the
/// stretch kept (ADR-0083), the chord at the cut being the junction's arc.
///
/// Errors: the errors of [`Spine::skin_within`] and [`Chord::skin_within`],
/// named for the edge as the stripe's own were.
pub(super) fn walked_cut(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    s: &mut Stripe,
    cut: [Option<f64>; 2],
) -> Result<(), OpError> {
    if cut == [None, None] {
        return Ok(());
    }
    let m = env.m;
    let Some((path, fit)) = &s.path else {
        return Err(invariant("a walked stripe's path"));
    };
    let Surface::Nurbs(surface) = &s.surface else {
        return Err(invariant("a walked stripe's NURBS surface"));
    };
    let domain = surface.domain()[1];
    let within = Interval::new(
        cut[0].unwrap_or(domain.lo()).max(domain.lo()),
        cut[1].unwrap_or(domain.hi()).min(domain.hi()),
    )
    .map_err(|_| invariant("a walked stripe ended inside its surface"))?;
    let surfaces = [0, 1].map(|k| m.face(s.faces[k]).and_then(|f| m.surface(f.surface())));
    let [Ok(sa), Ok(sb)] = surfaces else {
        return Err(invariant("a walked stripe's faces' surfaces"));
    };
    let named = |w| named(w, s.edge, s.faces, [sa, sb]);
    let ended = match path {
        Path::Ball(spine) => spine.skin_within(within, *fit, meter),
        Path::Chord(chord) => chord.skin_within(within, *fit, meter),
    }
    .map_err(named)?;
    let rows = contact_rows(&ended).map_err(named)?;
    s.surface = Surface::Nurbs(ended);
    s.lines = rows.map(Curve::Nurbs);
    Ok(())
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
        ref path,
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
    let skin = path.skin(fit, meter).map_err(named)?;
    let rows = contact_rows(&skin.surface).map_err(named)?;
    let [lo, hi] = skin.ends;
    let range = Interval::new(lo, hi).map_err(|_| invariant("a closed edge's period"))?;
    let seam_curve = first_column(&skin.surface).map_err(fault_of)?;
    let seam_range = Interval::new(0.0, 1.0).map_err(|_| invariant("an arc's range"))?;
    // The blend faces out of the material: its normal against the ball's
    // radial direction, or a chamfer's toward the edge it cuts.
    let outward = match path {
        Path::Ball(spine) => {
            let ball = spine.ball(skin.spine_t(lo)).map_err(named)?;
            // The arc at the vertex, exact, against the surface's own
            // isocurve there: they agree within the fit.
            debug_assert!({
                let exact = arc(&ball).map_err(named)?;
                (exact.eval(0.5).point - seam_curve.eval(0.5).point).norm() <= tolerance
            });
            faces_away(&skin.surface, Point2::new(0.5, range.lo()), ball.centre)?
        }
        Path::Chord(chord) => faces_toward(
            &skin.surface,
            Point2::new(0.5, range.lo()),
            chord.edge_point(range.lo()),
        )?,
    };
    let orientation = if outward == convex {
        Orientation::Forward
    } else {
        Orientation::Reversed
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
            on_blend: Curve2::along_v(k as f64),
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
            on_blend: [Curve2::along_u(range.lo()), Curve2::along_u(range.hi())],
            cuts,
            vertex_tolerance,
        })),
        tolerance,
        across: true,
        rolled: None,
    })
}
