//! Interior extrema of trimmed faces (`plans/tight-body-bounds`).
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
//! A patch's "uphill" side, where a monotone coordinate is largest, may
//! lie on a knot line where the surface is only C0: a crease, whose ridge
//! no neighbour's monotone proof rules out. A patch whose uphill side is
//! such a line is not set aside that way.
//!
//! A patch the loops pass within their chord deviation of is *grazed*:
//! it may hold points outside `D`, so its evaluated points count only
//! where they are farther than that from every loop and the loops wind
//! around them. Every other patch the loops wind around lies wholly inside
//! `D`, and one they do not wind around holds no point of it.

use core::f64::consts::{FRAC_PI_2, PI, TAU};
use std::collections::VecDeque;

use arris_check::domain::{FaceDomain, shifts};
use arris_geom::{NurbsSurface, Surface, region2::Side};
use arris_math::{Aabb, Interrupted, Interval, Meter, Point2, Vec3, real};

/// Axis extrema of an analytic surface that belong to the trimmed face.
/// A coordinate of a plane, cylinder, elliptic cylinder or cone is affine
/// along a ruling, so the boundary supplies both extrema. A singular end
/// of a cone or torus is represented by a boundary vertex.
#[cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "ops::query::tight_bounds calls it (plans/tight-body-bounds step 4)"
    )
)]
pub(crate) fn analytic_face_extrema(domain: &FaceDomain<'_>) -> Option<Aabb> {
    let surface = domain.surface();
    let [u_range, _] = domain.uv_box()?;
    let mut bounds: Option<Aabb> = None;
    let mut include = |uv: Point2, point| {
        if domain.side(uv).0 != Side::Outside {
            let b = Aabb::of_point(point);
            bounds = Some(bounds.map_or(b, |acc| acc.union(b)));
        }
    };
    let axes = [Vec3::x(), Vec3::y(), Vec3::z()];
    match surface {
        Surface::Sphere { frame, radius } => {
            for axis in axes {
                for d in [axis, -axis] {
                    let x = d.dot(frame.x().as_ref());
                    let y = d.dot(frame.y().as_ref());
                    let z = d.dot(frame.z().as_ref());
                    let h = real::hypot(x, y);
                    let u = if h == 0.0 {
                        u_range.lerp(0.5)
                    } else {
                        real::atan2(y, x).rem_euclid(TAU)
                    };
                    let v = real::atan2(z, h);
                    include(Point2::new(u, v), frame.origin() + *radius * d);
                }
            }
        }
        Surface::Torus { frame, .. } => {
            for axis in axes {
                let x = axis.dot(frame.x().as_ref());
                let y = axis.dot(frame.y().as_ref());
                let z = axis.dot(frame.z().as_ref());
                let h = real::hypot(x, y);
                if h == 0.0 {
                    // Each crest has constant height along u. If the sample
                    // is outside but some of the crest is in the face, the
                    // crest meets an edge that already supplies that height.
                    let u = u_range.lerp(0.5);
                    for v in [FRAC_PI_2, 3.0 * FRAC_PI_2] {
                        include(Point2::new(u, v), surface.point(u, v));
                    }
                } else {
                    // On the regular band, dP/du vanishes along the axis
                    // being measured at the two radial directions ±h.
                    // In either meridian, the two stationary tube angles
                    // are a half turn apart.
                    let azimuth = real::atan2(y, x);
                    for (u, radial) in [(azimuth, h), (azimuth + PI, -h)] {
                        let u = u.rem_euclid(TAU);
                        let meridian = real::atan2(z, radial);
                        for v in [meridian, meridian + PI] {
                            let v = v.rem_euclid(TAU);
                            include(Point2::new(u, v), surface.point(u, v));
                        }
                    }
                }
            }
        }
        Surface::Plane { .. }
        | Surface::Cylinder { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Cone { .. }
        | Surface::Nurbs(_) => {}
    }
    bounds
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

/// One piece of a loop's polygon in the translate the patches are in, and
/// the polygon's chord deviation: the true loop is within `reach` of it.
#[derive(Debug, Clone, Copy)]
struct Segment {
    from: Point2,
    to: Point2,
    reach: f64,
}

/// A rectangle of the subdivision: its (u, v) ranges, its depth, the sides
/// still open (bit `2k` the low side of axis `k`, `2k + 1` the high one),
/// and the segments that pass within their reach of it.
#[derive(Debug, Clone)]
struct Patch {
    uv: [Interval; 2],
    depth: u32,
    open: u8,
    near: Vec<usize>,
}

/// The tight box of the face `domain` reads, on the NURBS `surface`, given
/// `boundary`, a box that holds the face's boundary (its edges' points):
/// every coordinate's extreme not inside the face is on it.
///
/// Guarantees, where the box is `Some`: it holds every point of the face;
/// each of its sides is within `eps` of a point of the face, or of the
/// same side of `boundary` (each side of which the caller holds to a point
/// of the boundary). A side still open on a patch [`MAX_DEPTH`] halvings
/// down keeps that patch's hull: outer, and within the hull's diagonal and
/// the loops' chord deviation of the face, rather than `eps`. `None` when
/// the domain has no (u, v) box (a face no box holds), or a patch has no
/// hull.
///
/// Errors: [`Interrupted`] by the meter, ticked once per patch split.
#[cfg_attr(
    not(test),
    expect(
        dead_code,
        reason = "ops::query::tight_bounds calls it (plans/tight-body-bounds step 4)"
    )
)]
pub(crate) fn nurbs_face_box(
    surface: &NurbsSurface,
    domain: &FaceDomain<'_>,
    boundary: Aabb,
    eps: f64,
    meter: &mut Meter,
) -> Result<Option<Aabb>, Interrupted> {
    let Some(uv) = domain.uv_box() else {
        return Ok(None);
    };
    let Some(root) = fit_to_surface(surface, uv) else {
        return Ok(Some(boundary));
    };
    let segments = segments(domain, surface, root);
    let creases = [0, 1].map(|dir| creases(surface, dir));

    let mut inner: Option<Aabb> = None;
    // The sides of the hulls kept, axis by axis.
    let mut kept_lo = [f64::INFINITY; 3];
    let mut kept_hi = [f64::NEG_INFINITY; 3];
    let mut queue = VecDeque::from([Patch {
        uv: root,
        depth: 0,
        open: 0b11_1111,
        near: (0..segments.len()).collect(),
    }]);
    while let Some(patch) = queue.pop_front() {
        let [u, v] = patch.uv;
        let centre = Point2::new(u.lerp(0.5), v.lerp(0.5));
        let grazed = !patch.near.is_empty();
        if !grazed && !domain.winds_around(centre) {
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
                    && domain.winds_around(q));
            if counts {
                let b = Aabb::of_point(surface.eval(q.x, q.y).point);
                inner = Some(inner.map_or(b, |acc| acc.union(b)));
            }
        }
        let net = surface.clipped(patch.uv);
        let hull = match &net {
            Some(n) => box_of(n),
            None => domain.surface().bounds(patch.uv),
        };
        // A finite rectangle always has a hull; one that does not leaves
        // nothing outer to keep, so there is no box rather than a wrong one.
        let Some(hull) = hull else {
            return Ok(None);
        };
        let r = inner.map_or(boundary, |i| boundary.union(i));
        let mut open = patch.open;
        for k in 0..3 {
            if open & (0b11 << (2 * k)) == 0 {
                continue;
            }
            if let Some(n) = &net
                && monotone(
                    surface,
                    n,
                    k,
                    [hull.min[k], hull.max[k]],
                    patch.uv,
                    &creases,
                )
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
        let halves = [0, 1].map(|dir| halve(patch.uv[dir]));
        let (Some([u0, u1]), Some([v0, v1])) = (halves[0], halves[1]) else {
            keep_open_sides(&hull, open, &mut kept_lo, &mut kept_hi);
            continue;
        };
        if patch.depth >= MAX_DEPTH {
            keep_open_sides(&hull, open, &mut kept_lo, &mut kept_hi);
            continue;
        }
        meter.tick()?;
        for child in [[u0, v0], [u1, v0], [u0, v1], [u1, v1]] {
            let near = patch
                .near
                .iter()
                .copied()
                .filter(|&s| distance_to_rect(&segments[s], child) <= segments[s].reach)
                .collect();
            queue.push_back(Patch {
                uv: child,
                depth: patch.depth + 1,
                open,
                near,
            });
        }
    }

    let mut result = inner.map_or(boundary, |i| boundary.union(i));
    for k in 0..3 {
        result.min[k] = result.min[k].min(kept_lo[k]);
        result.max[k] = result.max[k].max(kept_hi[k]);
    }
    Ok(Some(result))
}

/// The face's (u, v) box cut to what the surface has: a clamped direction
/// to its domain, a periodic one to one period from its low end, where a
/// point a period further is the same point. `None` when a clamped
/// direction's box misses the domain.
fn fit_to_surface(surface: &NurbsSurface, uv: [Interval; 2]) -> Option<[Interval; 2]> {
    let domain = surface.domain();
    let periods = surface.period();
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

/// The loops' polygon pieces, in every translate by a period that brings
/// them within their reach of `root`.
fn segments(domain: &FaceDomain<'_>, surface: &NurbsSurface, root: [Interval; 2]) -> Vec<Segment> {
    let periods = surface.period();
    let mut out = Vec::new();
    for polygon in domain.polygons() {
        let reach = polygon.chord_deviation();
        let points = polygon.points();
        for (i, &p) in points.iter().enumerate() {
            let q = points[(i + 1) % points.len()];
            for du in shifts(periods[0]) {
                for dv in shifts(periods[1]) {
                    let shift = arris_math::Vec2::new(du, dv);
                    let s = Segment {
                        from: p + shift,
                        to: q + shift,
                        reach,
                    };
                    if distance_to_rect(&s, root) <= reach {
                        out.push(s);
                    }
                }
            }
        }
    }
    out
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
    let x = match surface.period()[dir] {
        Some(p) => domain.lo() + (x - domain.lo()).rem_euclid(p),
        None => x,
    };
    creases.iter().any(|&k| {
        let ulp = f64::EPSILON * k.abs().max(x.abs()).max(1.0);
        (x - k).abs() <= CREASE_ULPS * ulp
    })
}

/// `true` when `net`, the patch of `surface` over `uv` whose hull in `k`
/// is `range`, proves coordinate `k` strictly monotone along `u` or along
/// `v` across the patch (the module's proof), and the side where it is
/// largest is not a crease of `surface`.
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
        let uphill = if rising { uv[dir].hi() } else { uv[dir].lo() };
        if !on_crease(surface, dir, uphill, &creases[dir]) {
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

/// The distance from `p` to a segment.
fn distance_to_segment(p: Point2, s: &Segment) -> f64 {
    let d = s.to - s.from;
    let len2 = d.norm_squared();
    let t = if len2 > 0.0 {
        ((p - s.from).dot(&d) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (s.from + t * d)).norm()
}

/// The distance from a point to a rectangle, zero inside it.
fn distance_to_box(p: Point2, uv: [Interval; 2]) -> f64 {
    let [u, v] = uv;
    let dx = (u.lo() - p.x).max(p.x - u.hi()).max(0.0);
    let dy = (v.lo() - p.y).max(p.y - v.hi()).max(0.0);
    real::hypot(dx, dy)
}

/// The distance between a segment and a rectangle: zero where the segment
/// crosses it, else attained at an end of the segment or a corner of the
/// rectangle, as between any two disjoint convex sets with corners.
fn distance_to_rect(s: &Segment, uv: [Interval; 2]) -> f64 {
    if crosses(s, uv) {
        return 0.0;
    }
    let [u, v] = uv;
    let corners = [
        Point2::new(u.lo(), v.lo()),
        Point2::new(u.hi(), v.lo()),
        Point2::new(u.lo(), v.hi()),
        Point2::new(u.hi(), v.hi()),
    ];
    corners
        .iter()
        .map(|&c| distance_to_segment(c, s))
        .chain([distance_to_box(s.from, uv), distance_to_box(s.to, uv)])
        .fold(f64::INFINITY, f64::min)
}

/// `true` when the segment meets the closed rectangle: the segment's
/// parameter clipped by each slab (Liang–Barsky) keeps a range.
fn crosses(s: &Segment, uv: [Interval; 2]) -> bool {
    let d = s.to - s.from;
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for (axis, range) in uv.iter().enumerate() {
        let (o, dd) = (s.from[axis], d[axis]);
        if dd == 0.0 {
            if o < range.lo() || o > range.hi() {
                return false;
            }
            continue;
        }
        let (a, b) = ((range.lo() - o) / dd, (range.hi() - o) / dd);
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        t0 = t0.max(a);
        t1 = t1.min(b);
        if t0 > t1 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod analytic_tests;
#[cfg(test)]
mod tests;
