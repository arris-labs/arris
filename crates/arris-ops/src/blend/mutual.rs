//! A cusp of two blended edges, the stripes' mutual trim (ADR-0084 §4′):
//! each stripe runs on toward the cusp and is cut by the other along
//! their section, from `T*`, where both touch the shared face `T` at the
//! offsets' meeting, down to `S`, where both touch the walls on the
//! spine. Both trim points are closed forms; the section between them is
//! traced on the two exact stripe surfaces and fitted
//! (`traced::traced_end`, ADR-0042 §3–§4). The two stripes are tangent at
//! both trim points, singular points of the section where its branches
//! end exactly (`trace_torus`), so the stretch runs from one node to the
//! other on one branch.

use core::f64::consts::TAU;

use arris_geom::{Curve, GeomKind, Surface, pcurve_on};
use arris_math::{Aabb, Frame, Meter, Point3, Tolerance, UnitVec3, Vec2, real};
use arris_topo::{EdgeId, FaceId, VertexId};

use super::build::Env;
use super::ends::cut_corner;
use super::junction::Run;
use super::miter::Miter;
use super::ring::{CuspEnd, Ring, Trace, placed_uv};
use super::stripe::{Section, Stripe};
use super::traced::{TracedEnd, traced_end};
use super::{degenerate, invariant};
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild::forward;

/// The samples along the fitted section, ends included, each held to both
/// stripes' bands: a stretch that leaves either band between its trim
/// points cuts that stripe's contact on the way, which the mutual trim
/// does not cover (ADR-0084 §5′). The stretch is under a quarter of a
/// turn of either tube, so 64 samples put several on any excursion wider
/// than a sixtieth of it. A sampling density, not a tolerance.
const BAND_SAMPLES: usize = 64;

/// One stripe at the cusp as its mutual section reads it.
pub(super) struct CuspStripe<'a> {
    /// The stripe's exact surface: a torus about an axis normal to the
    /// shared face, or a cylinder along a line edge in it.
    pub(super) surface: &'a Surface,
    /// Its contact with the shared face `T`: a circle about an axis
    /// normal to `T`, or a line in `T`.
    pub(super) top: &'a Curve,
    /// Whether a point lies on the stripe's band: between its contacts
    /// across, and along its edge on the side of the cusp it runs.
    pub(super) band: &'a dyn Fn(Point3) -> Result<bool, OpError>,
}

/// The cusp the two stripes meet at.
#[derive(Debug, Clone, Copy)]
pub(super) struct Cusp {
    /// The shared face's plane: its origin on `T`, its `Z` normal to it.
    pub(super) top: Frame,
    /// The cusp vertex `v`, on `T`, where the spine leaves it.
    pub(super) vertex: Point3,
    /// A point on the side of the cusp both edges leave it on (an edge's
    /// midpoint): of the two crossings of the contacts on `T`, the one
    /// nearer it is `T*`.
    pub(super) side: Point3,
    /// The unit direction from `T` toward the balls' centres, along the
    /// spine: `−σ` times `T`'s outward normal.
    pub(super) inward: UnitVec3,
    /// The fillet's radius.
    pub(super) size: f64,
}

/// The mutual section of two stripes at a cusp.
pub(super) struct MutualSection {
    /// `T*`, on `T` at the offsets' meeting, then `S`, on the spine.
    pub(super) points: [Point3; 2],
    /// The section from `T*` (`gaps[0]`) to `S` (`gaps[1]`) on both
    /// stripes' bands.
    pub(super) end: TracedEnd,
}

/// The contact `curve` on `T` read in `top`'s plane: a circle about an
/// axis normal to it, or a line in it. `None` for any other curve or
/// pose.
fn trace_on(top: &Frame, curve: &Curve, tol: Tolerance) -> Option<Trace> {
    let n = top.z().into_inner();
    let flat = |p: Point3| {
        let q = top.to_local(p);
        Vec2::new(q.x, q.y)
    };
    match *curve {
        Curve::Circle { frame, radius } => {
            (frame.z().dot(&n).abs() >= 1.0 - tol.angular).then(|| Trace::Circle {
                centre: flat(frame.origin()),
                radius,
            })
        }
        Curve::Line { origin, direction } => {
            let d = top.vec_to_local(direction.into_inner());
            let along = Vec2::new(d.x, d.y);
            (d.z.abs() <= tol.angular).then(|| Trace::Line {
                point: flat(origin),
                along: along / along.norm(),
            })
        }
        Curve::Ellipse { .. } | Curve::Nurbs(_) => None,
    }
}

/// The two trim points of the mutual trim (ADR-0084 §2′, §4′): `T*`,
/// where the stripes' contacts on `T` cross on the edges' side of the
/// cusp — the foot of the ball the walls' offsets meet at, `Q` — and `S`,
/// the spine's point `size` from `T` toward the balls. `None` where the
/// contacts are no circle or line in `T`'s plane, or do not cross.
pub(super) fn trim_points(
    stripes: [&CuspStripe<'_>; 2],
    cusp: &Cusp,
    tol: Tolerance,
) -> Option<[Point3; 2]> {
    let at = top_meeting(stripes.map(|s| s.top), cusp, tol)?;
    let s = cusp.vertex + cusp.size * cusp.inward.into_inner();
    Some([at, s])
}

/// `T*`: where the two contacts `tops` on `T` cross on the edges' side of
/// the cusp ([`trim_points`]' first).
fn top_meeting(tops: [&Curve; 2], cusp: &Cusp, tol: Tolerance) -> Option<Point3> {
    let [a, b] = tops.map(|top| trace_on(&cusp.top, top, tol));
    let side = cusp.top.to_local(cusp.side);
    let q = a?.crossing(&b?, Vec2::new(side.x, side.y))?;
    Some(cusp.top.to_world(Point3::new(q.x, q.y, 0.0)))
}

/// A cusp both of whose edges are blended (ADR-0084 §1), as the build
/// joins it: the vertex, its two blended edges in the body's order, the
/// spine and the shared face `T`, each edge's contact on `T` and the cusp
/// read in `T`'s plane. At least one edge is an arc; the other is an arc
/// or a line.
#[derive(Debug, Clone)]
pub(super) struct CuspJoin {
    pub(super) vertex: VertexId,
    pub(super) edges: [EdgeId; 2],
    pub(super) spine: EdgeId,
    pub(super) top: FaceId,
    /// Each edge's contact with `T`, before any end trims it: a circle
    /// about an axis normal to `T` for an arc, a line for a line edge.
    pub(super) tops: [Curve; 2],
    pub(super) cusp: Cusp,
}

impl CuspJoin {
    /// The refusal of a join too large for the law: `BlendTooLarge`
    /// naming both edges (ADR-0084 §2′).
    fn too_large(&self) -> OpError {
        degenerate(
            self.edges.iter().map(|&e| forward(e)).collect(),
            Reason::Blend(BlendReason::TooLarge),
        )
    }

    /// Each arc's end at the cusp ([`CuspEnd`]): `T*`, where the two
    /// edges' contacts on `T` cross, read off the contacts before any ring
    /// is built. Where they do not cross, `too_large`.
    pub(super) fn end(&self, env: &Env<'_>) -> Result<CuspEnd, OpError> {
        let point = top_meeting([&self.tops[0], &self.tops[1]], &self.cusp, env.tol)
            .ok_or_else(|| self.too_large())?;
        Ok(CuspEnd {
            top: self.top,
            point,
        })
    }
}

/// Whether `p` lies on ring `r`'s band: on its torus between its two
/// contacts round the tube, and round the axis within the contacts'
/// angles, each widened by `tolerance`. A ring on any other surface has no
/// band this law reads (ADR-0084 §4′ is two tori).
fn ring_band(r: &Ring, p: Point3, tolerance: f64) -> Result<bool, OpError> {
    let (frame, tube) = match r.surface {
        Surface::Torus {
            frame,
            minor_radius,
            ..
        } => (frame, Some(minor_radius)),
        Surface::Cone { frame, .. } => (frame, None),
        _ => return Err(invariant("a cusp join's ring on a torus or a cone")),
    };
    let uv = r.surface.project(p).map_err(fault_of)?.uv;
    let v_of = |c: usize| r.contacts[c].on_blend.point(r.contacts[c].range.lo()).y;
    let (v0, v1) = (v_of(0), v_of(1));
    let across = match tube {
        Some(minor_radius) => {
            let slack = tolerance / minor_radius;
            let w = v0 + (uv.y - v0).rem_euclid(TAU);
            w <= v1 + slack || w >= v0 + TAU - slack
        }
        // A cone's `v` runs along a ruling, a length: between its two
        // contacts', each widened by the tolerance.
        None => uv.y >= v0.min(v1) - tolerance && uv.y <= v0.max(v1) + tolerance,
    };
    let us: Vec<f64> = (r.contacts.iter())
        .flat_map(|c| [c.range.lo(), c.range.hi()].map(|t| c.on_blend.point(t).x))
        .collect();
    let lo = us.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = us.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let off = frame.to_local(p);
    let rho = real::hypot(off.x, off.y);
    if rho <= tolerance {
        return Ok(false);
    }
    let slack = tolerance / rho;
    let u = lo + (uv.x - lo).rem_euclid(TAU);
    let along = u <= hi + slack || u >= lo + TAU - slack;
    Ok(across && along)
}

/// Whether `p` lies on the line stripe `s`'s band: on its cylinder between
/// its two contacts round the tube, and along its edge on the side of
/// `vertex` the edge runs (`d` points away from `vertex` where the edge
/// starts there), each widened by `tolerance`.
fn stripe_band(
    s: &Stripe,
    (vertex_id, vertex): (VertexId, Point3),
    p: Point3,
    tolerance: f64,
) -> Result<bool, OpError> {
    let uv = s.surface.project(p).map_err(fault_of)?.uv;
    let across = match s.section {
        Section::Round { radius, .. } => {
            let u = uv.x.rem_euclid(TAU);
            let slack = tolerance / radius;
            u <= s.u1 + slack || u >= TAU - slack
        }
        // A chamfer's plane: `u` across from one contact to the other.
        Section::Flat => uv.x >= -tolerance && uv.x <= s.u1 + tolerance,
        Section::Walked { .. } => return Err(invariant("a cusp join's stripe on a cylinder")),
    };
    // `d` points away from the vertex where the edge starts there.
    let away = if s.start == vertex_id { 1.0 } else { -1.0 };
    let along = away * (p - vertex).dot(&s.d);
    Ok(across && along >= -tolerance)
}

/// The join of the two blends at a cusp both of whose edges are blended
/// (ADR-0084 §4′), recorded as a miter is: the section of the two stripe
/// surfaces from `T*`, where the contacts on the shared face end (`q`), to
/// `S` on the spine, where the other two end (`p3`), the spine shortened to
/// `S`. Its pcurve on each surface placed at its contact's end there. The
/// runs are `join.edges`' in that order: a ring built with the join's
/// [`CuspEnd`], or a line's stripe (a cylinder), whose contacts end at the
/// two trim points. Errors: [`mutual_section`]'s, `too_large` being
/// `BlendTooLarge` naming both edges and `refuse` `Unsupported` naming
/// both surfaces; a contact's end off its trim point, `refuse`; a spine
/// shorter than the cut, `BlendTooLarge` naming it.
pub(super) fn cusp_join(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    runs: [Run<'_>; 2],
    join: &CuspJoin,
) -> Result<Miter, OpError> {
    let m = env.m;
    let at = [runs[0].at(m, join.vertex)?, runs[1].at(m, join.vertex)?];
    let tolerance = at.iter().map(|a| a.tolerance).fold(0.0, f64::max);
    let tol = Tolerance::new(tolerance, env.tol.angular);
    let refuse = || OpError::Unsupported {
        a: (GeomKind::Surface(at[0].surface.kind()), forward(at[0].edge)),
        b: (GeomKind::Surface(at[1].surface.kind()), forward(at[1].edge)),
    };
    let too_large = || join.too_large();
    let vertex = (join.vertex, m.vertex(join.vertex)?.point());
    // Each run's contact on `T`.
    let mut shared = [0usize; 2];
    for (i, a) in at.iter().enumerate() {
        shared[i] = (a.faces.iter())
            .position(|&f| f == join.top)
            .ok_or(invariant("a cusp run's contact on the shared face"))?;
    }
    let bands = [0, 1].map(|i| {
        let run = runs[i];
        move |p: Point3| match run {
            Run::Arc(r) => ring_band(r, p, tolerance),
            Run::Line(s) => stripe_band(s, vertex, p, tolerance),
        }
    });
    let stripes = [0, 1].map(|i| CuspStripe {
        surface: at[i].surface,
        top: &join.tops[i],
        band: &bands[i],
    });
    let section = mutual_section(
        [&stripes[0], &stripes[1]],
        &join.cusp,
        &refuse,
        &too_large,
        tol,
        meter,
    )?;
    let [q, p3] = section.points;
    // Each contact ends at its trim point: the one on `T` at `T*`, the
    // other on the spine at `S`. A ring's ends are its contacts' range
    // ends; a line's are the parameters of the trim points on its contact
    // lines.
    let mut t = [[0.0; 2]; 2];
    for (i, run) in runs.iter().enumerate() {
        for (c, slot) in t[i].iter_mut().enumerate() {
            let to = if c == shared[i] { q } else { p3 };
            match run {
                Run::Arc(r) => {
                    let contact = &r.contacts[c];
                    let j = usize::from(m.edge(r.edge)?.start() != join.vertex);
                    if (contact.points[j] - to).norm() > tolerance {
                        return Err(refuse());
                    }
                    *slot = if j == 0 {
                        contact.range.lo()
                    } else {
                        contact.range.hi()
                    };
                }
                Run::Line(s) => {
                    let on = s.lines[c].project(to).map_err(fault_of)?;
                    if on.distance > tolerance {
                        return Err(refuse());
                    }
                    *slot = on.t;
                }
            }
        }
    }
    let TracedEnd {
        curve,
        range,
        lo_first,
        gaps,
    } = section.end;
    let at_q = if lo_first { range.lo() } else { range.hi() };
    let mut on_blend = Vec::with_capacity(2);
    for (i, run) in runs.iter().enumerate() {
        let pcurve = pcurve_on(&curve, range, at[i].surface, tol, meter).map_err(fault_of)?;
        on_blend.push(match run {
            Run::Arc(r) => {
                let contact = &r.contacts[shared[i]];
                placed_uv(pcurve, at_q, contact.on_blend.point(t[i][shared[i]]))
            }
            Run::Line(s) => s.place(pcurve, at_q, if shared[i] == 0 { 0.0 } else { s.u1 }),
        });
    }
    let on_blend: [_; 2] = on_blend
        .try_into()
        .map_err(|_| invariant("the cusp join's two pcurves"))?;
    let trim = cut_corner(m, join.edges[0], join.spine, join.vertex, p3)?;
    let spine_tolerance = m.edge(join.spine)?.tolerance();
    Ok(Miter {
        edges: join.edges,
        shared,
        t,
        q,
        p3,
        curve,
        range,
        q_first: lo_first,
        lo_first: shared.map(|k| lo_first == (k == 0)),
        on_blend,
        trim: Some(trim),
        q_trim: None,
        trim_arc: None,
        leaning: false,
        kink: None,
        tolerance,
        q_tolerance: tolerance.max(gaps[0]),
        p3_tolerance: tolerance.max(gaps[1]).max(spine_tolerance),
    })
}

/// The section two stripes at a cusp are trimmed by each other along
/// (ADR-0084 §4′), from `T*` to `S` ([`trim_points`]): the stretch of
/// the one branch of their section between the two trim points that lies
/// on both stripes' bands, traced on the exact surfaces and fitted.
///
/// Guarantees: both trim points within `tol.linear` of both surfaces; the
/// curve within `SECTION_FIT_FRACTION · tol.linear` of the exact section
/// at its own parameter, each end within `end.gaps` (at most
/// `tol.linear`) of its trim point, and every one of [`BAND_SAMPLES`]
/// points along it on both bands. `too_large` — the caller's
/// `BlendTooLarge` naming both edges — where the contacts do not cross,
/// where `T*` is off either band, or where the stretch leaves either band
/// between the trim points (ADR-0084 §2′, §5′); `refuse` — the caller's
/// refusal naming the two stripes — where the trim points are off the
/// surfaces (the stripes are not this law's) or the trace does not decide
/// one stretch (`traced_end`).
pub(super) fn mutual_section(
    stripes: [&CuspStripe<'_>; 2],
    cusp: &Cusp,
    refuse: &dyn Fn() -> OpError,
    too_large: &dyn Fn() -> OpError,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<MutualSection, OpError> {
    meter.tick()?;
    let points = trim_points(stripes, cusp, tol).ok_or_else(too_large)?;
    for s in stripes {
        for p in points {
            if s.surface.project(p).map_err(fault_of)?.distance > tol.linear {
                return Err(refuse());
            }
        }
        if !(s.band)(points[0])? {
            return Err(too_large());
        }
    }
    let both =
        |p: Point3| -> Result<bool, OpError> { Ok((stripes[0].band)(p)? && (stripes[1].band)(p)?) };
    let within = Aabb::of_point(points[0])
        .union(Aabb::of_point(points[1]))
        .inflated(2.0 * cusp.size);
    let end = traced_end(
        [stripes[0].surface, stripes[1].surface],
        points,
        &within,
        &both,
        refuse,
        tol,
        meter,
    )?;
    let (lo, hi) = (end.range.lo(), end.range.hi());
    for i in 0..=BAND_SAMPLES {
        let p = end
            .curve
            .point(lo + (hi - lo) * i as f64 / BAND_SAMPLES as f64);
        if !both(p)? {
            return Err(too_large());
        }
    }
    Ok(MutualSection { points, end })
}

#[cfg(test)]
mod tests {
    use arris_geom::SECTION_FIT_FRACTION;
    use arris_math::nalgebra::{Unit, UnitQuaternion};
    use arris_math::{Precision, Vec3, real};

    use super::*;

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    /// The poses a cusp is built in: at the origin, and turned and moved
    /// a hundred of its sizes away.
    fn poses() -> [Frame; 2] {
        let turn =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vec3::new(0.3, -0.5, 0.8)), 1.1);
        [
            Frame::world(),
            Frame::from_rotation(Point3::new(120.0, -75.0, 40.0), &turn),
        ]
    }

    fn refused() -> OpError {
        OpError::Internal(crate::error::Fault::Invariant {
            what: "the test's refusal",
        })
    }

    fn too_large() -> OpError {
        OpError::Internal(crate::error::Fault::Invariant {
            what: "the test's too large",
        })
    }

    fn is(err: &OpError, what: &str) -> bool {
        matches!(err, OpError::Internal(crate::error::Fault::Invariant { what: w }) if *w == what)
    }

    /// The three recipes of `regression/cusp-corner-*-fillet` (ADR-0084
    /// §1): the crescent prism's two arcs filleted convex under its top,
    /// its pocket's two floor arcs concave above the floor, and the
    /// spandrel's line and arc convex under its top. All with the shared
    /// face at height 1.
    #[derive(Debug, Clone, Copy, PartialEq)]
    enum Family {
        Crescent,
        Pocket,
        Spandrel,
    }

    impl Family {
        fn name(self) -> &'static str {
            match self {
                Family::Crescent => "cusp-corner-crescent-fillet",
                Family::Pocket => "cusp-corner-pocket-fillet",
                Family::Spandrel => "cusp-corner-spandrel-fillet",
            }
        }
    }

    /// One family's two stripes at radius `r`, in the recipe's own
    /// coordinates moved by `pose`: the surfaces, their contacts on the
    /// top and the cusp; [`Setting::band`] reads a point's bands. `sigma`
    /// is `+1` where the blend lies under `T`, `−1` over it.
    struct Setting {
        surfaces: [Surface; 2],
        tops: [Curve; 2],
        cusp: Cusp,
        pose: Frame,
        family: Family,
        r: f64,
    }

    fn framed(pose: &Frame, origin: Point3, z: Vec3, x: Vec3) -> Frame {
        Frame::new(
            pose.to_world(origin),
            pose.vec_to_world(z),
            pose.vec_to_world(x),
        )
        .unwrap()
    }

    fn setting(family: Family, r: f64, pose: Frame) -> Setting {
        let h = 1.0;
        let sigma = if family == Family::Pocket { -1.0 } else { 1.0 };
        // The balls' centres' height.
        let c = h - sigma * r;
        let torus = |centre: [f64; 2], major: f64| Surface::Torus {
            frame: framed(
                &pose,
                Point3::new(centre[0], centre[1], c),
                Vec3::z(),
                Vec3::x(),
            ),
            major_radius: major,
            minor_radius: r,
        };
        let circle = |centre: [f64; 2], radius: f64| Curve::Circle {
            frame: framed(
                &pose,
                Point3::new(centre[0], centre[1], h),
                Vec3::z(),
                Vec3::x(),
            ),
            radius,
        };
        let (surfaces, tops, vertex, side) = match family {
            // The big wall `ρ₁ = 2` with the crescent inside, the small
            // `ρ₂ = 1` about `(−1, 0)` with it outside: the balls' centres
            // in the crescent, at `2 − r` and `1 + r`, under the prism's
            // top and over the pocket's floor.
            Family::Crescent | Family::Pocket => {
                let (a1, a2) = (2.0 - r, 1.0 + r);
                (
                    [torus([0.0, 0.0], a1), torus([-1.0, 0.0], a2)],
                    [circle([0.0, 0.0], a1), circle([-1.0, 0.0], a2)],
                    Point3::new(-2.0, 0.0, h),
                    Point3::new(-1.0, -1.0, h),
                )
            }
            // The line wall `y = 0` with the body above it, the arc wall
            // `ρ = 4` about `(0, 4)` with the body outside it.
            Family::Spandrel => (
                [
                    Surface::Cylinder {
                        frame: framed(&pose, Point3::new(0.0, r, c), Vec3::x(), Vec3::y()),
                        radius: r,
                    },
                    torus([0.0, 4.0], 4.0 + r),
                ],
                [
                    Curve::Line {
                        origin: pose.to_world(Point3::new(0.0, r, h)),
                        direction: UnitVec3::new_normalize(pose.vec_to_world(Vec3::x())),
                    },
                    circle([0.0, 4.0], 4.0 + r),
                ],
                Point3::new(0.0, 0.0, h),
                Point3::new(2.0, 0.0, h),
            ),
        };
        Setting {
            surfaces,
            tops,
            cusp: Cusp {
                top: framed(&pose, Point3::new(0.0, 0.0, h), Vec3::z(), Vec3::x()),
                vertex: pose.to_world(vertex),
                side: pose.to_world(side),
                inward: UnitVec3::new_normalize(pose.vec_to_world(-sigma * Vec3::z())),
                size: r,
            },
            pose,
            family,
            r,
        }
    }

    impl Setting {
        /// Stripe `k`'s band at `p`: between its contacts round the
        /// tube, and along its edge on the side of the cusp the recipe's
        /// edge lies (`y ≤ 0` for the arcs of the crescent and pocket,
        /// `x ≥ 0` for the spandrel's), each widened by the tolerance.
        fn band(&self, k: usize, p: Point3) -> bool {
            let q = self.pose.to_local(p);
            let (r, slack) = (self.r, tol().linear);
            let sigma = if self.family == Family::Pocket {
                -1.0
            } else {
                1.0
            };
            // Toward `T` from the balls' centres, and away from the wall
            // on the wall's own normal.
            let up = sigma * (q.z - (1.0 - sigma * r)) >= -slack;
            match (self.family, k) {
                (Family::Crescent | Family::Pocket, 0) => {
                    let rho = real::hypot(q.x, q.y);
                    up && rho >= 2.0 - r - slack && q.y <= slack
                }
                (Family::Crescent | Family::Pocket, _) => {
                    let rho = real::hypot(q.x + 1.0, q.y);
                    up && rho <= 1.0 + r + slack && q.y <= slack
                }
                (Family::Spandrel, 0) => up && q.y <= r + slack && q.x >= -slack,
                (Family::Spandrel, _) => {
                    let rho = real::hypot(q.x, q.y - 4.0);
                    up && rho <= 4.0 + r + slack && q.x >= -slack
                }
            }
        }

        /// The section's own closed form in the shared plane, which the
        /// branch on both bands satisfies and the other branches do not:
        /// the tubes' equal offsets from their centre circles taken with
        /// opposite signs, `ρ₁ + ρ₂ = 3` (an ellipse about the two axes),
        /// and on the spandrel `ρ − y = 4` (the parabola with focus the
        /// arc's centre and vertex the cusp). Zero on it.
        fn law(&self, p: Point3) -> f64 {
            let q = self.pose.to_local(p);
            match self.family {
                Family::Crescent | Family::Pocket => {
                    real::hypot(q.x, q.y) + real::hypot(q.x + 1.0, q.y) - 3.0
                }
                Family::Spandrel => real::hypot(q.x, q.y - 4.0) - q.y - 4.0,
            }
        }

        fn section(&self) -> Result<MutualSection, OpError> {
            let bands =
                [0, 1].map(|k| move |p: Point3| -> Result<bool, OpError> { Ok(self.band(k, p)) });
            let stripes = [0, 1].map(|k| CuspStripe {
                surface: &self.surfaces[k],
                top: &self.tops[k],
                band: &bands[k],
            });
            mutual_section(
                [&stripes[0], &stripes[1]],
                &self.cusp,
                &refused,
                &too_large,
                tol(),
                &mut Meter::default(),
            )
        }
    }

    const FAMILIES: [Family; 3] = [Family::Crescent, Family::Pocket, Family::Spandrel];
    const SIZES: [f64; 3] = [0.02, 0.1, 0.3];

    /// Each family's mutual section at the oracle's three sizes, in two
    /// poses: both trim points on both surfaces and where the law puts
    /// them, the curve on both surfaces within the fit's fraction, its
    /// ends at the trim points, and the branch the one on both bands —
    /// every sample on both, and on the section's closed form.
    #[test]
    fn the_mutual_section_runs_from_the_top_contact_to_the_spine() {
        let mut sections = 0;
        for pose in poses() {
            for family in FAMILIES {
                for r in SIZES {
                    let set = setting(family, r, pose);
                    let at = set
                        .section()
                        .unwrap_or_else(|e| panic!("{family:?} at {r}: {e}"));
                    let [t, s] = at.points.map(|p| set.pose.to_local(p));
                    // `T*` on the top over `Q`, `S` on the spine `r` from it.
                    let q = match family {
                        Family::Crescent | Family::Pocket => {
                            let (a1, a2) = (2.0 - r, 1.0 + r);
                            let x = (a2 * a2 - a1 * a1 - 1.0) / 2.0;
                            [x, -(a1 * a1 - x * x).sqrt(), 1.0]
                        }
                        Family::Spandrel => [4.0 * r.sqrt(), r, 1.0],
                    };
                    let slack = 1e-12 * (1.0 + set.pose.origin().coords.norm());
                    assert!(
                        (t - Point3::from(q)).norm() <= slack,
                        "{family:?} at {r}: T* {t}"
                    );
                    let sigma = if family == Family::Pocket { -1.0 } else { 1.0 };
                    let spine = match family {
                        Family::Crescent | Family::Pocket => [-2.0, 0.0],
                        Family::Spandrel => [0.0, 0.0],
                    };
                    let want = Point3::new(spine[0], spine[1], 1.0 - sigma * r);
                    assert!((s - want).norm() <= slack, "{family:?} at {r}: S {s}");
                    for p in at.points {
                        for surface in &set.surfaces {
                            let d = surface.project(p).unwrap().distance;
                            assert!(d <= tol().linear, "{family:?} at {r}: {d:e} off");
                        }
                    }
                    let end = &at.end;
                    assert!(
                        end.gaps.iter().all(|&g| g <= tol().linear),
                        "{:?}",
                        end.gaps
                    );
                    let (lo, hi) = (end.range.lo(), end.range.hi());
                    let first = end.curve.point(if end.lo_first { lo } else { hi });
                    assert!((first - at.points[0]).norm() <= tol().linear);
                    let samples = 256;
                    for i in 0..=samples {
                        let p = end.curve.point(lo + (hi - lo) * i as f64 / samples as f64);
                        let slack = 1e-11 * (1.0 + p.coords.norm());
                        for surface in &set.surfaces {
                            let d = surface.project(p).unwrap().distance;
                            assert!(
                                d <= SECTION_FIT_FRACTION * tol().linear + slack,
                                "{family:?} at {r}: {d:e} off a {:?} at {i}",
                                surface.kind()
                            );
                        }
                        assert!(
                            set.band(0, p) && set.band(1, p),
                            "{family:?} at {r}: off a band at {i}"
                        );
                        // The other branches through the nodes leave this
                        // curve at a finite angle, so the law's residual
                        // stays at the fit's scale only on this one.
                        let off = set.law(p).abs();
                        assert!(off <= 1e-5, "{family:?} at {r}: {off:e} off the law at {i}");
                    }
                    sections += 1;
                }
            }
        }
        assert_eq!(sections, 18);
    }

    /// Arris's section against the edge Open CASCADE's mutual trim of its
    /// own two single-edge fillets shares between the stripes (`common`,
    /// or `fuse` on the pocket), sampled by
    /// `tools/oracle/cusp_corner_section.py` at the three sizes
    /// (`section.json`): each of the oracle's two vertices within its own
    /// tolerance of the trim point it stands for, and each of its points
    /// within the edge's tolerance of Arris's curve, the end ones within
    /// their vertex's. At the spine the two stripes are tangent and the
    /// oracle's curve stops up to `3.6·10⁻⁵` short of its vertex there, on
    /// both surfaces to `3·10⁻¹⁰` but off the section, which its vertex's
    /// tolerance covers; elsewhere the two lie within `3·10⁻⁸`.
    #[test]
    fn the_mutual_section_is_the_oracles_mutual_trim_edge() {
        const RECORD: &str =
            "../../tests/fixtures/regression/cusp-corner-crescent-fillet/section.json";
        let text = std::fs::read_to_string(RECORD).expect("the committed oracle record");
        let record: serde_json::Value = serde_json::from_str(&text).unwrap();
        let point = |p: &serde_json::Value| {
            let c = |k: usize| p[k].as_f64().unwrap();
            Point3::new(c(0), c(1), c(2))
        };
        let linear = tol().linear;
        let mut runs = 0;
        for run in record["records"].as_array().unwrap() {
            let name = run["fixture"].as_str().unwrap();
            let family = FAMILIES.into_iter().find(|f| f.name() == name).unwrap();
            let r = run["size"].as_f64().unwrap();
            let at = setting(family, r, Frame::world()).section().unwrap();
            let edge = run["tolerance"].as_f64().unwrap();
            let vertices: Vec<(Point3, f64)> = (run["vertices"].as_array().unwrap().iter())
                .map(|v| (point(&v["point"]), v["tolerance"].as_f64().unwrap()))
                .collect();
            for &(v, tolerance) in &vertices {
                let d = (at.points.iter())
                    .map(|&p| (p - v).norm())
                    .fold(f64::INFINITY, f64::min);
                assert!(
                    d <= tolerance + linear,
                    "{name} at {r}: vertex {v} {d:e} off"
                );
            }
            let points = run["points"].as_array().unwrap();
            for (i, p) in points.iter().enumerate() {
                let p = point(p);
                let d = at.end.curve.project(p).unwrap().distance;
                // An end sample is its edge's end, which its vertex's
                // tolerance stands for.
                let near = (vertices.iter())
                    .filter(|&&(v, t)| (p - v).norm() <= t + edge)
                    .map(|&(_, t)| t)
                    .fold(0.0, f64::max);
                let allowed = edge.max(near) + linear;
                assert!(
                    d <= allowed,
                    "{name} at {r}: sample {i} {d:e} off, allowed {allowed:e}"
                );
            }
            runs += 1;
        }
        assert_eq!(runs, 9);
    }

    /// The two named refusals: a stretch that leaves a band between the
    /// trim points — stripe 0's band cut across near `T*` — is
    /// `too_large`; a stripe whose ball is not the other's — its tube moved
    /// off the shared height, so `S` is off it — is `refuse`.
    #[test]
    fn a_section_off_a_band_or_off_the_law_is_refused() {
        let set = setting(Family::Crescent, 0.1, Frame::world());
        let t = set.section().unwrap().points[0];
        let cut = |p: Point3| -> Result<bool, OpError> {
            let y = p.y / t.y;
            Ok(set.band(0, p) && !(0.8..0.9).contains(&y))
        };
        let other = |p: Point3| -> Result<bool, OpError> { Ok(set.band(1, p)) };
        let stripes = [
            (&cut as &dyn Fn(Point3) -> Result<bool, OpError>, 0),
            (&other, 1),
        ]
        .map(|(band, k)| CuspStripe {
            surface: &set.surfaces[k],
            top: &set.tops[k],
            band,
        });
        let err = mutual_section(
            [&stripes[0], &stripes[1]],
            &set.cusp,
            &refused,
            &too_large,
            tol(),
            &mut Meter::default(),
        )
        .err()
        .unwrap();
        assert!(is(&err, "the test's too large"), "{err}");

        let mut set = setting(Family::Crescent, 0.1, Frame::world());
        set.surfaces[1] = Surface::Torus {
            frame: Frame::world().with_origin(Point3::new(-1.0, 0.0, 0.85)),
            major_radius: 1.1,
            minor_radius: 0.15,
        };
        let err = set.section().err().unwrap();
        assert!(is(&err, "the test's refusal"), "{err}");
    }
}
