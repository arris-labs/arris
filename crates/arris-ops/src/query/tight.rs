//! Tight bounds of the closure and trimmed faces.
//!
//! A NURBS face's (u, v) box is subdivided into patches, each bounded by its own
//! clipped control net, until every side of the box is within `eps` of a
//! point the face reaches or of the face's boundary box.
//!
//! Over a face's region `D` in (u, v), each coordinate `S_k` of the surface
//! is largest (and smallest) either on the boundary of `D`, which the
//! caller's `boundary` box holds, or at a point inside `D` where both its
//! partial derivatives vanish. A patch is set aside for coordinate `k`
//! when its net proves `S_k` strictly monotone along `u` or along `v`
//! across the whole patch: no such point is in it. What is left of a
//! coordinate's patches is split until each one's hull is within `eps` of
//! the running box. The proof, for a net with positive weights `w` and
//! points `Q`, the coordinate's hull over the patch `[a, b]`: `∂u S_k` has
//! the sign of `∂u Σ N w (Q_k − s)` at `s = S_k ∈ [a, b]`, a B-spline whose
//! coefficients are positive multiples of the row-to-row differences
//! `w'(Q'_k − s) − w(Q_k − s)`; linear in `s`, they keep one sign over
//! `[a, b]` when they do at both ends.
//!
//! A patch's low or high side along a monotone coordinate may lie on a
//! knot line where the surface is only C0. A crease can hold an extremum
//! even when the coordinate is monotone on both sides of it.
//!
//! A patch the loops pass within their chord deviation of is *grazed*:
//! it may hold points outside `D`, so its evaluated points count only
//! where they are farther than that from every loop and the loops wind
//! around them. Every other patch the loops wind around lies wholly inside
//! `D`, and one they do not wind around holds no point of it.

use std::collections::{BTreeSet, VecDeque};

use arris_geom::{NurbsSurface, Surface};
use arris_math::{Aabb, Control, Interval, Meter, Point2};
use arris_topo::{EntityId, FaceId, Model, Orientation, Shape};

use super::degenerate;
use crate::{OpError, QueryReason, Reason};

mod analytic;
mod domain;
mod trim;

use domain::{TrimDomain, distance_to_rect, distance_to_segment};

/// The tight box of a vertex, edge, face, shell or body in world coordinates.
/// It contains the entity's geometry, with each side within the model's
/// linear tolerance of a point on the entity, to rounding. Entity tolerances
/// do not grow the box.
///
/// Returns `None` if the entity is empty or unbounded.
/// Errors are [`OpError::NotFound`] for an unresolved id,
/// [`OpError::Degenerate`] with [`QueryReason::BoundsUncertified`] when a
/// finite edge or face cannot be bounded to tolerance, and
/// [`OpError::Interrupted`] when the control stops the query. The meter
/// counts closure entities, preparation of knot pieces, period copies,
/// and edge, trim and face splits.
///
/// ```
/// use arris_math::{Axis, Control, Point3, Vec3};
/// use arris_ops::{cut, primitive_cylinder, query::tight_bounds};
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let control = &Control::NONE;
/// // A shaft collar with an axial bore and a cross hole.
/// let (blank, _) =
///     primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 2.0, 2.0, control)?;
/// let (bore, _) =
///     primitive_cylinder(&mut m, Axis::z_at(Point3::new(0.0, 0.0, -1.0)), 1.0, 4.0, control)?;
/// let (ring, _) = cut(&mut m, blank, bore, control)?;
/// let cross_axis = Axis::new(Point3::new(0.0, -3.0, 1.0), Vec3::y())?;
/// let (cross_hole, _) = primitive_cylinder(&mut m, cross_axis, 0.5, 6.0, control)?;
/// let (shaft_collar, _) = cut(&mut m, ring, cross_hole, control)?;
/// let b = tight_bounds(&m, shaft_collar.id.into(), control)?.unwrap();
/// let eps = m.precision().default_tolerance;
/// for k in 0..3 {
///     assert!((b.min[k] - [-2.0, -2.0, 0.0][k]).abs() <= eps);
///     assert!((b.max[k] - 2.0).abs() <= eps);
/// }
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn tight_bounds(
    m: &Model,
    entity: EntityId,
    control: &Control<'_>,
) -> Result<Option<Aabb>, OpError> {
    let mut meter = Meter::new(control);
    let mut entities = BTreeSet::new();
    collect(m, entity, &mut entities, &mut meter)?;

    let eps = m.precision().default_tolerance * 0.5;
    let mut boundary = None;
    let mut faces = Vec::new();
    for entity in entities {
        let b = match entity {
            EntityId::Vertex(id) => {
                meter.tick()?;
                Aabb::of_point(m.vertex(id)?.point())
            }
            EntityId::Edge(id) => {
                meter.tick()?;
                let Some((curve, range)) = m.edge(id)?.curve() else {
                    continue;
                };
                let curve = m.curve(curve)?;
                if !range.is_bounded() {
                    return Ok(None);
                }
                curve
                    .tight_bounds_with(range, eps, &mut meter)?
                    .ok_or_else(|| bounds_uncertified(id))?
            }
            EntityId::Face(id) => {
                faces.push(id);
                continue;
            }
            EntityId::Shell(_) | EntityId::Body(_) => continue,
        };
        include_box(&mut boundary, b);
    }

    let mut result = boundary;
    // The edge, trim, and interior bounds may build on one another.
    // Their allowances add to the linear tolerance.
    let face_eps = eps * 0.5;
    let parametric_tolerance = m.precision().parametric_tolerance;
    for id in faces {
        meter.tick()?;
        let face = m.face(id)?;
        let surface = m.surface(face.surface())?;
        let loops = face
            .loops()
            .iter()
            .map(|l| m.loop_pieces(l))
            .collect::<Result<Vec<_>, _>>()?;
        if loops.iter().any(|pieces| !trim::bounded_loop(pieces)) {
            return Ok(None);
        }
        let hull_inside_closure = if let Surface::Nurbs(nurbs) = surface
            && let (Some(boundary), Some(hull)) = (boundary, box_of(nurbs))
        {
            boundary.union(hull) == boundary
        } else {
            false
        };
        if hull_inside_closure && trim::within_domain(surface, &loops) {
            continue;
        }
        let interior = match surface {
            Surface::Nurbs(nurbs) => {
                // Seed from the closure and this face's lifted trims. The
                // approximation errors of different faces cannot accumulate.
                let seed = boundary.unwrap_or_else(|| {
                    let [u, v] = nurbs.domain();
                    Aabb::of_point(surface.point(u.lo(), v.lo()))
                });
                let seed = trim::bounds(surface, &loops, seed, face_eps, &mut meter)?
                    .ok_or_else(|| bounds_uncertified(id))?;
                if hull_inside_closure {
                    include_box(&mut result, seed);
                    continue;
                }
                let domain =
                    TrimDomain::of(&loops, surface.period(), parametric_tolerance, &mut meter)?
                        .ok_or_else(|| bounds_uncertified(id))?;
                nurbs_face_box(id, nurbs, &domain, seed, face_eps, &mut meter)?
            }
            Surface::Sphere { .. } | Surface::Torus { .. } => {
                let seed = if let Some(boundary) = boundary {
                    Some(
                        trim::bounds(surface, &loops, boundary, face_eps, &mut meter)?
                            .ok_or_else(|| bounds_uncertified(id))?,
                    )
                } else {
                    None
                };
                analytic::face_box(
                    surface,
                    &loops,
                    seed,
                    parametric_tolerance,
                    face_eps,
                    &mut meter,
                )?
                .ok_or_else(|| bounds_uncertified(id))?
            }
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. } => {
                let Some(seed) = boundary.filter(|_| !loops.is_empty()) else {
                    return Ok(None);
                };
                // The ruling argument only needs the lifted trim.
                trim::bounds(surface, &loops, seed, face_eps, &mut meter)?
                    .ok_or_else(|| bounds_uncertified(id))?
            }
        };
        include_box(&mut result, interior);
    }
    Ok(result)
}

fn include_box(bounds: &mut Option<Aabb>, b: Aabb) {
    *bounds = Some(bounds.map_or(b, |acc| acc.union(b)));
}

fn collect(
    m: &Model,
    entity: EntityId,
    entities: &mut BTreeSet<EntityId>,
    meter: &mut Meter<'_>,
) -> Result<(), OpError> {
    if !entities.insert(entity) {
        return Ok(());
    }
    match entity {
        EntityId::Vertex(id) => {
            m.vertex(id)?;
        }
        EntityId::Edge(id) => {
            let edge = m.edge(id)?;
            for vertex in [edge.start(), edge.end()] {
                collect(m, vertex.into(), entities, meter)?;
            }
        }
        EntityId::Face(id) => {
            for c in m.face(id)?.loops().iter().flat_map(|l| l.coedges()) {
                m.curve2(c.pcurve())?;
                collect(m, c.edge().into(), entities, meter)?;
            }
        }
        EntityId::Shell(id) => {
            meter.tick()?;
            for face in m.shell(id)?.faces() {
                collect(m, face.id.into(), entities, meter)?;
            }
        }
        EntityId::Body(id) => {
            meter.tick()?;
            let body = m.body(id)?;
            for shell in body.shells() {
                collect(m, shell.id.into(), entities, meter)?;
            }
            for edge in body.free_edges() {
                collect(m, edge.id.into(), entities, meter)?;
            }
            for &vertex in body.free_vertices() {
                collect(m, vertex.into(), entities, meter)?;
            }
        }
    }
    Ok(())
}

/// How many halvings a patch takes before its hull is kept as it is: a
/// patch `2⁻³²` of the face's (u, v) box is far below any tolerance a
/// model is read at, so a side that has not settled by then is one
/// rounding or a grazing loop keeps open. Its hull is still outer. A
/// structural bound, not a tolerance: the 1000 property cases split at
/// most 28 times in all.
const MAX_DEPTH: u32 = 32;

/// How many units in the last place a patch's side may be from a C0 knot
/// line and still be read as on it. Erring towards a crease only keeps a
/// patch that a monotone proof would have set aside.
const CREASE_ULPS: f64 = 4.0;

/// A rectangle of the subdivision: its (u, v) ranges, its depth, the sides
/// still open (bit `2k` the low side of axis `k`, `2k + 1` the high one),
/// and the segments that pass within their reach of it.
#[derive(Debug, Clone)]
struct Patch {
    uv: [Interval; 2],
    /// The displacement of each closed, clamped direction from its knot domain.
    shift: [f64; 2],
    depth: u32,
    open: u8,
    near: Vec<usize>,
}

/// The tight box of the face `domain` reads, on the NURBS `surface`, given
/// `boundary`, a box that holds the face's lifted trim:
/// every coordinate's extreme not inside the face is on it.
///
/// The box holds every point of the face, with each side within `eps` of
/// the face or of the corresponding side of `boundary`. A face with no
/// trim is read over the whole surface. If refinement reaches its limit
/// before it proves these bounds, the query names the face it could not
/// certify. The meter ticks once per patch split.
fn nurbs_face_box(
    face: FaceId,
    surface: &NurbsSurface,
    domain: &TrimDomain,
    boundary: Aabb,
    eps: f64,
    meter: &mut Meter,
) -> Result<Aabb, OpError> {
    let whole = domain.uv_box().is_none();
    let uv = domain.uv_box().unwrap_or_else(|| surface.domain());
    let Some(root) = fit_to_surface(surface, uv) else {
        return Ok(boundary);
    };
    let segments = domain.segments_in(root, meter)?;
    let creases = [0, 1].map(|dir| creases(surface, dir));

    let mut inner: Option<Aabb> = None;
    // The sides of the hulls kept, axis by axis.
    let mut kept_lo = [f64::INFINITY; 3];
    let mut kept_hi = [f64::NEG_INFINITY; 3];
    let spline_domain = surface.domain();
    let [us, vs] = [0, 1].map(|dir| {
        if surface.closure()[dir].is_some() && surface.period()[dir].is_none() {
            closed_parts(root[dir], spline_domain[dir])
        } else {
            vec![(root[dir], 0.0)]
        }
    });
    if us.len() > 1 || vs.len() > 1 {
        meter.tick()?;
    }
    let mut queue = VecDeque::new();
    for &(v, dv) in &vs {
        for &(u, du) in &us {
            let uv = [u, v];
            let near = (0..segments.len())
                .filter(|&s| distance_to_rect(&segments[s], uv) <= segments[s].reach)
                .collect();
            queue.push_back(Patch {
                uv,
                shift: [du, dv],
                depth: 0,
                open: 0b11_1111,
                near,
            });
        }
    }
    while let Some(patch) = queue.pop_front() {
        let [u, v] = patch.uv;
        let centre = Point2::new(u.lerp(0.5), v.lerp(0.5));
        let grazed = !patch.near.is_empty();
        if !whole && !grazed && !domain.winds_around(centre, meter)? {
            continue;
        }
        let corners = [
            centre,
            Point2::new(u.lo(), v.lo()),
            Point2::new(u.hi(), v.lo()),
            Point2::new(u.lo(), v.hi()),
            Point2::new(u.hi(), v.hi()),
        ];
        for q in corners {
            let counts = !grazed
                || (patch
                    .near
                    .iter()
                    .all(|&s| distance_to_segment(q, &segments[s]) > segments[s].reach)
                    && domain.winds_around(q, meter)?);
            if counts {
                let b = Aabb::of_point(surface.eval(q.x, q.y).point);
                inner = Some(inner.map_or(b, |acc| acc.union(b)));
            }
        }
        let [u_clip, v_clip] = [0, 1].map(|dir| {
            let r = patch.uv[dir];
            let shift = patch.shift[dir];
            if shift == 0.0 {
                return Ok(r);
            }
            let d = spline_domain[dir];
            Interval::new(d.clamp(r.lo() - shift), d.clamp(r.hi() - shift))
        });
        let (Ok(u_clip), Ok(v_clip)) = (u_clip, v_clip) else {
            return Err(bounds_uncertified(face));
        };
        let clip = [u_clip, v_clip];
        let net = surface.clipped(clip);
        let hull = match &net {
            Some(n) => box_of(n),
            None => Surface::Nurbs(surface.clone()).bounds(clip),
        };
        let Some(hull) = hull else {
            return Err(bounds_uncertified(face));
        };
        let r = inner.map_or(boundary, |i| boundary.union(i));
        let mut open = patch.open;
        for k in 0..3 {
            if open & (0b11 << (2 * k)) == 0 {
                continue;
            }
            if !whole
                && let Some(n) = &net
                && monotone(surface, n, k, [hull.min[k], hull.max[k]], clip, &creases)
            {
                open &= !(0b11 << (2 * k));
                continue;
            }
            if open & (1 << (2 * k)) != 0 && hull.min[k] >= r.min[k] - eps {
                kept_lo[k] = kept_lo[k].min(hull.min[k]);
                open &= !(1 << (2 * k));
            }
            if open & (1 << (2 * k + 1)) != 0 && hull.max[k] <= r.max[k] + eps {
                kept_hi[k] = kept_hi[k].max(hull.max[k]);
                open &= !(1 << (2 * k + 1));
            }
        }
        if open == 0 {
            continue;
        }
        let [us, vs] = patch.uv.map(halve);
        if patch.depth >= MAX_DEPTH || (us.is_none() && vs.is_none()) {
            keep_open_sides(&hull, open, &mut kept_lo, &mut kept_hi);
            continue;
        }
        meter.tick()?;
        let us = us
            .as_ref()
            .map_or(std::slice::from_ref(&u), |r| r.as_slice());
        let vs = vs
            .as_ref()
            .map_or(std::slice::from_ref(&v), |r| r.as_slice());
        for &v in vs {
            for &u in us {
                let child = [u, v];
                let near = patch
                    .near
                    .iter()
                    .copied()
                    .filter(|&s| distance_to_rect(&segments[s], child) <= segments[s].reach)
                    .collect();
                queue.push_back(Patch {
                    uv: child,
                    shift: patch.shift,
                    depth: patch.depth + 1,
                    open,
                    near,
                });
            }
        }
    }

    let witnesses = inner.map_or(boundary, |i| boundary.union(i));
    let mut result = witnesses;
    for k in 0..3 {
        // A stalled patch may be settled by a point evaluated elsewhere
        // later in the queue.
        if kept_lo[k] < witnesses.min[k] - eps || kept_hi[k] > witnesses.max[k] + eps {
            return Err(bounds_uncertified(face));
        }
        result.min[k] = result.min[k].min(kept_lo[k]);
        result.max[k] = result.max[k].max(kept_hi[k]);
    }
    Ok(result)
}

fn bounds_uncertified(entity: impl Into<EntityId>) -> OpError {
    degenerate(
        Shape::new(entity, Orientation::Forward),
        Reason::Query(QueryReason::BoundsUncertified),
    )
}

/// The face's (u, v) box cut to what the surface has: an open direction
/// to its domain, a closed one to one period from the box's low end.
/// `None` when an open direction's box misses the domain.
fn fit_to_surface(surface: &NurbsSurface, uv: [Interval; 2]) -> Option<[Interval; 2]> {
    let domain = surface.domain();
    let periods = surface.closure();
    let mut out = uv;
    for dir in 0..2 {
        let r = uv[dir];
        out[dir] = match periods[dir] {
            Some(p) => Interval::new(r.lo(), r.hi().min(r.lo() + p)).ok()?,
            None => {
                let d = domain[dir];
                Interval::new(r.lo().max(d.lo()), r.hi().min(d.hi())).ok()?
            }
        };
    }
    Some(out)
}

/// The pieces of a range of at most one period in a closed, clamped
/// direction. Each piece stays in the trim's translate for winding tests
/// and carries the displacement of its knot domain for clipping.
fn closed_parts(range: Interval, domain: Interval) -> Vec<(Interval, f64)> {
    if domain.contains(range.lo()) && domain.contains(range.hi()) {
        return vec![(range, 0.0)];
    }
    let period = domain.length();
    let mut shift = ((range.lo() - domain.lo()) / period).floor() * period;
    if range.lo() < domain.lo() + shift {
        shift -= period;
    }
    let seam = domain.hi() + shift;
    let mut parts = Vec::with_capacity(2);
    if let Ok(left) = Interval::new(range.lo(), range.hi().min(seam))
        && left.length() > 0.0
    {
        parts.push((left, shift));
    }
    if let Ok(right) = Interval::new(range.lo().max(seam), range.hi())
        && (right.length() > 0.0 || parts.is_empty())
    {
        parts.push((right, shift + period));
    }
    parts
}

/// The knots of direction `dir` inside the domain at which the surface is
/// at most C0: a multiplicity of at least the degree. For a periodic
/// direction, reduced into the domain.
fn creases(surface: &NurbsSurface, dir: usize) -> Vec<f64> {
    let degree = surface.degree()[dir];
    let knots = surface.knots()[dir];
    let domain = surface.domain()[dir];
    let mut out: Vec<f64> = Vec::new();
    let mut i = 0;
    while i < knots.len() {
        let mut j = i;
        while j < knots.len() && knots[j] == knots[i] {
            j += 1;
        }
        let k = knots[i];
        if j - i >= degree && k > domain.lo() && k < domain.hi() {
            out.push(k);
        }
        i = j;
    }
    out
}

/// `true` when `x` in direction `dir` is on one of that direction's
/// creases, read modulo the period where there is one.
fn on_crease(surface: &NurbsSurface, dir: usize, x: f64, creases: &[f64]) -> bool {
    let domain = surface.domain()[dir];
    let closure = surface.closure()[dir];
    let x = match closure {
        Some(p) => domain.lo() + (x - domain.lo()).rem_euclid(p),
        None => x,
    };
    let at = |k: f64| {
        let ulp = f64::EPSILON * k.abs().max(x.abs()).max(1.0);
        (x - k).abs() <= CREASE_ULPS * ulp
    };
    // A closed, clamped seam only promises that its endpoints meet.
    (closure.is_some() && surface.period()[dir].is_none() && (at(domain.lo()) || at(domain.hi())))
        || creases.iter().copied().any(at)
}

/// `true` when `net`, the patch of `surface` over `uv` whose hull in `k`
/// is `range`, proves coordinate `k` strictly monotone along `u` or along
/// `v` across the patch (the module's proof), with both ends clear of
/// creases of `surface`.
fn monotone(
    surface: &NurbsSurface,
    net: &NurbsSurface,
    k: usize,
    range: [f64; 2],
    uv: [Interval; 2],
    creases: &[Vec<f64>; 2],
) -> bool {
    let [n, m] = net.counts();
    let points = net.control_points();
    let weights = net.weights();
    if weights.iter().any(|&w| w.is_nan() || w <= 0.0) {
        return false;
    }
    let value = |i: usize, j: usize, s: f64| weights[i * m + j] * (points[i * m + j][k] - s);
    for dir in 0..2 {
        let mut rising = true;
        let mut falling = true;
        for i in 0..n {
            for j in 0..m {
                let (i1, j1) = if dir == 0 { (i + 1, j) } else { (i, j + 1) };
                if i1 >= n || j1 >= m {
                    continue;
                }
                for s in range {
                    let step = value(i1, j1, s) - value(i, j, s);
                    rising &= step > 0.0;
                    falling &= step < 0.0;
                }
            }
        }
        if !(rising || falling) {
            continue;
        }
        if !on_crease(surface, dir, uv[dir].lo(), &creases[dir])
            && !on_crease(surface, dir, uv[dir].hi(), &creases[dir])
        {
            return true;
        }
    }
    false
}

/// The box of a net's control points: the hull that holds the patch.
fn box_of(net: &NurbsSurface) -> Option<Aabb> {
    let corners: Vec<[f64; 3]> = net
        .control_points()
        .iter()
        .map(|p| [p.x, p.y, p.z])
        .collect();
    Aabb::of_points(&corners)
}

/// Records the hull's still-open sides as kept.
fn keep_open_sides(hull: &Aabb, open: u8, lo: &mut [f64; 3], hi: &mut [f64; 3]) {
    for k in 0..3 {
        if open & (1 << (2 * k)) != 0 {
            lo[k] = lo[k].min(hull.min[k]);
        }
        if open & (1 << (2 * k + 1)) != 0 {
            hi[k] = hi[k].max(hull.max[k]);
        }
    }
}

/// The two halves of `r`, or `None` when its midpoint rounds to an end.
fn halve(r: Interval) -> Option<[Interval; 2]> {
    let mid = r.lerp(0.5);
    if mid <= r.lo() || mid >= r.hi() {
        return None;
    }
    Some([
        Interval::new(r.lo(), mid).ok()?,
        Interval::new(mid, r.hi()).ok()?,
    ])
}

#[cfg(test)]
mod analytic_tests;
#[cfg(test)]
mod query_tests;
#[cfg(test)]
mod tests;
