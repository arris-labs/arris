//! Three blends meeting at a vertex: the sphere or triangle corner.

use core::f64::consts::FRAC_PI_2;
use std::collections::BTreeSet;

use arris_geom::{Curve, Curve2, Surface, pcurve_on};
use arris_math::{
    Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec2, UnitVec3, Vec2, Vec3,
};
use arris_topo::{EdgeId, FaceId, Orientation, VertexId};

use super::build::Env;
use super::miter::cross_lines;
use super::stripe::{Section, Stripe, chord, line_origin, placed};
use super::{degenerate, invariant};
use crate::error::{OpError, Reason, fault_of};
use crate::rebuild::forward;

/// One side of a corner: where one of its three blends meets the corner
/// face — a great circle of the sphere, or a side of the triangle —
/// between the corner's points on that blend's two faces.
pub(super) struct CornerArc {
    pub(super) curve: Curve,
    pub(super) range: Interval,
    /// The corner's points at `range.lo()`, then at `range.hi()`.
    pub(super) ends: [usize; 2],
    /// `true` when `range.lo()` is at the blend's contact at `u = 0`.
    pub(super) lo_first: bool,
    /// Its pcurve on the blend, placed.
    pub(super) on_blend: Curve2,
    /// Its pcurve on the corner face, placed.
    pub(super) on_corner: Curve2,
    /// `true` when the corner face's loop walks it along its range.
    pub(super) along: bool,
}

/// A sphere corner's pole: the degenerate edge where its two meridians
/// meet, `u` running along its pcurve at `v = π/2` and walked back.
pub(super) struct Pole {
    /// The corner's point it stands at.
    pub(super) point: usize,
    pub(super) range: Interval,
    pub(super) pcurve: Curve2,
    /// The place in the corner's walk after which its loop crosses it.
    pub(super) after: usize,
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
pub(super) struct Corner {
    /// The three blended edges, in the blends' order: the sides.
    pub(super) edges: [EdgeId; 3],
    /// The corner's three faces, each holding the point at its index.
    pub(super) faces: [FaceId; 3],
    pub(super) points: [Point3; 3],
    /// For each side, the point each of its contacts ends at.
    pub(super) contact_point: [[usize; 2]; 3],
    /// The contact lines' parameters at the corner, `[side][contact]`.
    pub(super) t: [[f64; 2]; 3],
    /// By side.
    pub(super) arcs: [CornerArc; 3],
    /// The sides in the order the corner face's loop walks them.
    pub(super) walk: [usize; 3],
    pub(super) pole: Option<Pole>,
    pub(super) surface: Surface,
    pub(super) orientation: Orientation,
    pub(super) tolerance: f64,
}

/// The corner of stripes `stripes` at `vertex`, a vertex of these three
/// edges and no other (ADR-0007). Every face there a plane and every blend
/// convex or every one concave is what puts the three fillets' axes
/// through one centre; a fillet corner also needs a face square to the
/// other two, so that its sides are the sphere's equator and two meridians
/// with exact pcurves. A corner that is not all planes, of mixed blends or,
/// for fillets, with no such face is `Reason::VertexBlend` (a backlog
/// line C6 left).
pub(super) fn corner(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    stripes: [&Stripe; 3],
    vertex: VertexId,
) -> Result<Corner, OpError> {
    meter.tick()?;
    let rd = read_corner(env, stripes, vertex)?;
    let (surface, orientation, sides, pole) = match stripes[0].section {
        Section::Round { radius, .. } => corner_sphere(env, meter, &rd, radius)?,
        Section::Flat => corner_triangle(env, meter, &rd)?,
    };
    let CornerRead {
        edges,
        faces,
        contact_point,
        tolerance,
        points,
        t,
        ..
    } = rd;
    let vertex_blend = || rd.vertex_blend();
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

/// What a corner reads of its three stripes: the faces they meet, where
/// their contacts cross and the tolerance that holds.
#[derive(Clone, Copy)]
struct CornerRead<'a> {
    stripes: [&'a Stripe; 3],
    vertex: VertexId,
    edges: [EdgeId; 3],
    faces: [FaceId; 3],
    contact_point: [[usize; 2]; 3],
    tolerance: f64,
    points: [Point3; 3],
    t: [[f64; 2]; 3],
}

/// One side of a corner to build: its blend `side`, the `curve` over
/// `range` between points `ends`, and whether it is walked `along`.
struct ArcOf {
    side: usize,
    curve: Curve,
    range: Interval,
    ends: [usize; 2],
    along: bool,
}

impl CornerRead<'_> {
    /// The refusal for a corner the closed forms do not cover.
    fn vertex_blend(&self) -> OpError {
        let entities = self
            .edges
            .iter()
            .map(|&e| forward(e))
            .chain([forward(self.vertex)])
            .collect();
        degenerate(entities, Reason::VertexBlend)
    }

    /// The side between two of the corner's points.
    fn side_of(&self, a: usize, b: usize) -> Result<usize, OpError> {
        self.contact_point
            .iter()
            .position(|c| c.contains(&a) && c.contains(&b))
            .ok_or_else(|| self.vertex_blend())
    }

    /// The side `arc.side`, its pcurve on that blend placed as its
    /// contacts are.
    fn arc(
        &self,
        tol: Tolerance,
        arc: ArcOf,
        on_corner: Curve2,
        meter: &mut Meter<'_>,
    ) -> Result<CornerArc, OpError> {
        let ArcOf {
            side,
            curve,
            range,
            ends,
            along,
        } = arc;
        let arc_tol = Tolerance::new(self.tolerance, tol.angular);
        let s = self.stripes[side];
        let lo_first = ends[0] == self.contact_point[side][0];
        let on_blend = pcurve_on(&curve, range, &s.surface, arc_tol, meter).map_err(fault_of)?;
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
    }
}

/// The corner's faces and points: where the contacts on each face cross.
fn read_corner<'a>(
    env: &Env<'_>,
    stripes: [&'a Stripe; 3],
    vertex: VertexId,
) -> Result<CornerRead<'a>, OpError> {
    let view = env.view;
    let tol = env.tol;
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
        cross_lines(tol, tolerance, o1, d1, o2, d2, what)?.ok_or_else(vertex_blend)
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
    Ok(CornerRead {
        stripes,
        vertex,
        edges,
        faces,
        contact_point,
        tolerance,
        points,
        t,
    })
}

/// The sphere corner of three fillets: its sides are the equator and two
/// meridians of the sphere about the ball's centre.
fn corner_sphere(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    rd: &CornerRead<'_>,
    radius: f64,
) -> Result<CornerFace, OpError> {
    let m = env.m;
    let tol = env.tol;
    let CornerRead {
        stripes,
        faces,
        contact_point,
        tolerance,
        points,
        ..
    } = *rd;
    let vertex_blend = || rd.vertex_blend();
    let crossing = |o1: Point3, d1: Vec3, o2: Point3, d2: Vec3, what: &'static str| {
        cross_lines(tol, tolerance, o1, d1, o2, d2, what)?.ok_or_else(vertex_blend)
    };
    let side_of = |a: usize, b: usize| rd.side_of(a, b);
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
               meter: &mut Meter<'_>| {
        rd.arc(
            tol,
            ArcOf {
                side,
                curve,
                range,
                ends,
                along,
            },
            on_corner,
            meter,
        )
    };
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
    let quarter = Interval::new(0.0, FRAC_PI_2).map_err(|_| invariant("a quarter of a turn"))?;
    let sphere = Surface::Sphere { frame, radius };
    let on_sphere = |curve: &Curve,
                     range: Interval,
                     u: f64,
                     meter: &mut Meter<'_>|
     -> Result<Curve2, OpError> {
        let pcurve = pcurve_on(curve, range, &sphere, arc_tol, meter).map_err(geometry)?;
        Ok(placed(pcurve, range.lo(), u, sphere.period()[0]))
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
    Ok((sphere, orientation, sides, Some(pole)))
}

/// The triangle corner of three chamfers.
fn corner_triangle(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    rd: &CornerRead<'_>,
) -> Result<CornerFace, OpError> {
    let (m, view) = (env.m, env.view);
    let tol = env.tol;
    let CornerRead {
        faces,
        tolerance,
        points,
        ..
    } = *rd;
    let side_of = |a: usize, b: usize| rd.side_of(a, b);
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
               meter: &mut Meter<'_>| {
        rd.arc(
            tol,
            ArcOf {
                side,
                curve,
                range,
                ends,
                along,
            },
            on_corner,
            meter,
        )
    };
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
        let on_corner = pcurve_on(&curve, range, &plane, arc_tol, meter).map_err(geometry)?;
        sides.push((
            side,
            arc(side, curve, range, [from, to], on_corner, true, meter)?,
        ));
    }
    Ok((plane, Orientation::Forward, sides, None))
}

/// A corner face: its surface, its orientation, its sides in walking
/// order and a sphere's pole.
type CornerFace = (Surface, Orientation, Vec<(usize, CornerArc)>, Option<Pole>);
