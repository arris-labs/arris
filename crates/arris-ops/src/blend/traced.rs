//! A blend's end the face across cuts in no closed form (ADR-0037): the
//! section of the blend's exact surface with the face across, traced by
//! the intersector's own tracers (`trace_section`), the stretch of one
//! branch from one contact's trim point to the other's taken on the
//! stripe's side, and that stretch fitted at the branch's parameter
//! (`fit_branch`) — the surface stays exact, the end curve is the one
//! fitted item, held to the section as every traced section is.

use core::f64::consts::{PI, TAU};

use arris_geom::{Curve, FitError, GeomError, SectionBranch, Surface, fit_branch, trace_section};
use arris_math::{Aabb, Interval, Meter, Point3, Tolerance};

use crate::error::{OpError, fault_of};

/// The walked parameter between two samples when a trim point is located
/// on a branch, before the golden-section search narrows it: a branch's
/// parameter is a walked angle (ADR-0018, ADR-0019), and a 128th of a
/// half turn puts several samples on the shortest end a stripe has — a
/// quarter turn of a tube — so the nearest sample brackets the point's
/// own stretch and no other. A sampling density, not a tolerance.
const LOCATE_STEP: f64 = PI / 128.0;

/// The halvings by the golden ratio after the bracket: `0.618⁸⁰` of the
/// bracket is below the rounding of any parameter in it.
const LOCATE_ITERATIONS: usize = 80;

/// A traced end: the curve from one trim point to the other on the
/// blend and the face across, its range, and how far each trim point is
/// from the curve's end there.
#[derive(Debug, Clone)]
pub(super) struct TracedEnd {
    /// A `Curve::Nurbs` fitted to a branch's stretch, or a tube circle of
    /// the blend's torus exactly where the section is one.
    pub(super) curve: Curve,
    pub(super) range: Interval,
    /// `true` when `range.lo()` is at the first trim point.
    pub(super) lo_first: bool,
    /// The distance from each trim point, in the caller's order, to the
    /// curve's end it stands for: the trace's reach of the point plus the
    /// fit's, what the trim vertex's tolerance has to cover.
    pub(super) gaps: [f64; 2],
}

/// What a trim point can lie on in a trace: a branch, or a tube circle
/// the tracer returns exact.
#[derive(Clone, Copy)]
enum Piece<'a> {
    Branch(&'a SectionBranch),
    Circle(&'a Curve),
}

impl Piece<'_> {
    fn point(&self, t: f64) -> Point3 {
        match self {
            Piece::Branch(b) => b.point(t),
            Piece::Circle(c) => c.point(t),
        }
    }

    /// The period of a loop, `None` for an open branch.
    fn period(&self) -> Option<f64> {
        match self {
            Piece::Branch(b) => b.is_closed().then(|| b.domain().hi()),
            Piece::Circle(_) => Some(TAU),
        }
    }

    /// The parameters on an open branch that stand for `q`, located at
    /// `t`: `t` itself, or each end of the branch within `tol.linear` of
    /// `q` where one is — a branch that starts and ends at one node, the
    /// point there, has it at both.
    fn ends_at(&self, t: f64, q: Point3, tol: Tolerance) -> Vec<f64> {
        let Piece::Branch(b) = self else {
            return vec![t];
        };
        let domain = b.domain();
        let ends: Vec<f64> = [domain.lo(), domain.hi()]
            .into_iter()
            .filter(|&end| (b.point(end) - q).norm() <= tol.linear)
            .collect();
        if ends.is_empty() { vec![t] } else { ends }
    }

    /// The parameter of the point of the piece nearest `q`, and how far
    /// it is. On a branch the nearest of samples [`LOCATE_STEP`] apart,
    /// narrowed by a golden-section search over the two steps beside it.
    fn locate(&self, q: Point3) -> Result<(f64, f64), OpError> {
        match self {
            Piece::Circle(c) => {
                let p = c.project(q).map_err(fault_of)?;
                Ok((p.t, p.distance))
            }
            Piece::Branch(b) => {
                let domain = b.domain();
                let span = domain.hi() - domain.lo();
                // Whole samples, at least sixteen; a positive finite count.
                let n = ((span / LOCATE_STEP).ceil().max(16.0)) as usize;
                let h = span / n as f64;
                let at = |i: usize| domain.lo() + i as f64 * h;
                let off = |t: f64| (q - b.point(t)).norm();
                let mut best = (0, f64::INFINITY);
                for i in 0..=n {
                    let d = off(at(i));
                    if d < best.1 {
                        best = (i, d);
                    }
                }
                let (mut lo, mut hi) = (at(best.0) - h, at(best.0) + h);
                if !b.is_closed() {
                    lo = lo.max(domain.lo());
                    hi = hi.min(domain.hi());
                }
                let t = golden(off, lo, hi);
                Ok((t, off(t)))
            }
        }
    }
}

/// The minimum of `f` over `[a, b]` by golden-section search, `f`
/// unimodal there.
fn golden(f: impl Fn(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    let g = (5.0_f64.sqrt() - 1.0) / 2.0;
    let (mut c, mut d) = (b - g * (b - a), a + g * (b - a));
    let (mut fc, mut fd) = (f(c), f(d));
    for _ in 0..LOCATE_ITERATIONS {
        if fc <= fd {
            (b, d, fd) = (d, c, fc);
            c = b - g * (b - a);
            fc = f(c);
        } else {
            (a, c, fc) = (c, d, fd);
            d = a + g * (b - a);
            fd = f(d);
        }
    }
    let t = (a + b) / 2.0;
    // The search keeps the better of its two probes; the midpoint is no
    // worse than either unless the bracket held no minimum at all.
    [t, c, d]
        .into_iter()
        .min_by(|&x, &y| f(x).total_cmp(&f(y)))
        .unwrap_or(t)
}

/// The end of a blend whose `blend` surface the face across, `across`,
/// cuts in no closed form: the section's stretch from `points[0]` to
/// `points[1]`, the trim points where the two contacts pierce `across`,
/// on the side of them the stripe is — the one whose midpoint `inside`
/// takes (the blend's band between its contacts). `within` bounds a
/// ruled pair's trace and is ignored by a torus's (`trace_section`).
///
/// Guarantees: the curve lies within `SECTION_FIT_FRACTION · tol.linear`
/// of the exact section at its own parameter, so within that and the
/// trace's own rounding of both surfaces; its ends are the stretch's,
/// each within `gaps` of its trim point. `refuse` is returned — the
/// caller's `Unsupported` naming the blend and the face across — where
/// the trace does not decide the pose (`SectionFault`), the fit does not
/// get under the fraction, no single stretch of one branch or tube
/// circle holds both points within `tol.linear` on the stripe's side, or
/// the two points are one; a stop is the caller's.
pub(super) fn traced_end(
    surfaces: [&Surface; 2],
    points: [Point3; 2],
    within: &Aabb,
    inside: &dyn Fn(Point3) -> Result<bool, OpError>,
    refuse: &dyn Fn() -> OpError,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<TracedEnd, OpError> {
    meter.tick()?;
    let [blend, across] = surfaces;
    let trace = trace_section(blend, across, within, tol, meter).map_err(|e| match e {
        GeomError::DegenerateSection { .. } | GeomError::Unsupported { .. } => refuse(),
        other => fault_of(other),
    })?;
    let pieces = trace
        .circles()
        .iter()
        .map(|c| Piece::Circle(&c.circle))
        .chain(trace.branches().iter().map(Piece::Branch));
    let mut found: Option<(Piece<'_>, Interval, bool)> = None;
    for piece in pieces {
        let (t0, d0) = piece.locate(points[0])?;
        let (t1, d1) = piece.locate(points[1])?;
        if d0 > tol.linear || d1 > tol.linear {
            continue;
        }
        // The stretches from one point to the other: one on an open
        // branch, either way round a loop. A point at a node the branch
        // starts and ends at is at both its ends, and the stretch may
        // reach it at either (ADR-0042 §4).
        let stretches = match piece.period() {
            None => {
                let [a0, a1] =
                    [(t0, points[0]), (t1, points[1])].map(|(t, q)| piece.ends_at(t, q, tol));
                let mut stretches = Vec::new();
                for &s0 in &a0 {
                    for &s1 in &a1 {
                        stretches.push((s0.min(s1), s0.max(s1), s0 <= s1));
                    }
                }
                stretches
            }
            Some(p) => {
                let up = |t: f64, from: f64| from + (t - from).rem_euclid(p);
                vec![(t0, up(t1, t0), true), (t1, up(t0, t1), false)]
            }
        };
        for (lo, hi, lo_first) in stretches {
            let Ok(range) = Interval::new(lo, hi) else {
                return Err(refuse());
            };
            if hi - lo <= 0.0 || !inside(piece.point(range.midpoint()))? {
                continue;
            }
            if found.replace((piece, range, lo_first)).is_some() {
                return Err(refuse());
            }
        }
    }
    let Some((piece, range, lo_first)) = found else {
        return Err(refuse());
    };
    let curve = match piece {
        Piece::Circle(c) => c.clone(),
        Piece::Branch(b) => fit_branch(b, range, tol, meter).map_err(|e| match e {
            GeomError::Fit(FitError::Diverged { .. }) => refuse(),
            other => fault_of(other),
        })?,
    };
    let ends = if lo_first {
        [range.lo(), range.hi()]
    } else {
        [range.hi(), range.lo()]
    };
    let gaps = [0, 1].map(|k| (points[k] - curve.point(ends[k])).norm());
    Ok(TracedEnd {
        curve,
        range,
        lo_first,
        gaps,
    })
}

#[cfg(test)]
mod tests {
    use arris_geom::SECTION_FIT_FRACTION;
    use arris_math::nalgebra::{Unit, UnitQuaternion};
    use arris_math::{Frame, Precision, Vec3};

    use super::*;

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    /// The poses a configuration is built in: at the origin, and turned
    /// and moved a hundred of its sizes away.
    fn poses() -> [Frame; 2] {
        let turn =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vec3::new(0.3, -0.5, 0.8)), 1.1);
        [
            Frame::world(),
            Frame::from_rotation(Point3::new(120.0, -75.0, 40.0), &turn),
        ]
    }

    /// A frame of `pose`'s coordinates: `origin`, `z` and `x` given in them.
    fn framed(pose: &Frame, origin: Point3, z: Vec3, x: Vec3) -> Frame {
        Frame::new(
            pose.to_world(origin),
            pose.vec_to_world(z),
            pose.vec_to_world(x),
        )
        .unwrap()
    }

    /// What a test asks of a traced end: every one of `samples` points
    /// over its range within the fit's fraction of both surfaces (and
    /// their rounding at its distance from the origin), each end within
    /// the tolerance of its trim point, the midpoint on the stripe's side.
    fn holds(
        end: &TracedEnd,
        surfaces: [&Surface; 2],
        points: [Point3; 2],
        inside: &dyn Fn(Point3) -> Result<bool, OpError>,
    ) {
        let samples = 512;
        let (lo, hi) = (end.range.lo(), end.range.hi());
        for i in 0..=samples {
            let p = end.curve.point(lo + (hi - lo) * i as f64 / samples as f64);
            let slack = 1e-11 * (1.0 + p.coords.norm());
            for s in surfaces {
                let d = s.project(p).unwrap().distance;
                assert!(
                    d <= SECTION_FIT_FRACTION * tol().linear + slack,
                    "{d:e} off a {:?} at sample {i}",
                    s.kind()
                );
            }
        }
        for k in 0..2 {
            assert!(end.gaps[k] <= tol().linear, "gap {k}: {:e}", end.gaps[k]);
        }
        let first = end.curve.point(if end.lo_first { lo } else { hi });
        assert!((first - points[0]).norm() <= tol().linear);
        assert!(inside(end.curve.point(end.range.midpoint())).unwrap());
    }

    fn refused() -> OpError {
        OpError::Internal(crate::error::Fault::Invariant {
            what: "the test's refusal",
        })
    }

    /// The open arc's torus ending on a plane parallel to the axis and
    /// off it (ADR-0036 §6, the spiric section): a boss of radius `R`
    /// standing on a plane, its foot blended concave at `r` — the torus
    /// of major `R + r` and minor `r` about the boss's axis, level with
    /// the ball's centre — and a wall at `x = d` cutting the boss, which
    /// the census's parts have where a boss runs into a side face. Each
    /// contact meets the wall in closed form; the end between is a
    /// quarter of the tube's section, traced. `d = 0` is the plane
    /// through the axis, where the tracer returns the tube circle itself.
    #[test]
    fn a_torus_end_on_a_plane_parallel_to_its_axis_is_traced() {
        let mut ends = 0;
        for pose in poses() {
            for (big, r) in [(3.0, 0.5), (1.0, 0.25), (10.0, 0.1), (0.5, 0.4)] {
                for f in [0.0, 0.05, 0.3, 0.6, 0.9, 0.99, 0.9999] {
                    let d = f * big;
                    let ring = big + r;
                    let blend = Surface::Torus {
                        frame: framed(&pose, Point3::new(0.0, 0.0, r), Vec3::z(), Vec3::x()),
                        major_radius: ring,
                        minor_radius: r,
                    };
                    let across = Surface::Plane {
                        frame: framed(&pose, Point3::new(d, 0.0, 0.0), Vec3::x(), Vec3::y()),
                    };
                    let points = [
                        pose.to_world(Point3::new(d, (ring * ring - d * d).sqrt(), 0.0)),
                        pose.to_world(Point3::new(d, (big * big - d * d).sqrt(), r)),
                    ];
                    // The band between the contacts: the tube's quarter
                    // toward the axis and down, the boss's corner.
                    let inside = |p: Point3| -> Result<bool, OpError> {
                        let q = pose.to_local(p);
                        let slack = tol().linear;
                        Ok(q.x.hypot(q.y) <= ring + slack && q.z <= r + slack)
                    };
                    let within = Aabb::of_point(points[0])
                        .union(Aabb::of_point(points[1]))
                        .inflated(2.0 * r);
                    let end = traced_end(
                        [&blend, &across],
                        points,
                        &within,
                        &inside,
                        &refused,
                        tol(),
                        &mut Meter::default(),
                    )
                    .unwrap_or_else(|e| panic!("R {big}, r {r}, d/R {f}: {e}"));
                    assert_eq!(matches!(end.curve, Curve::Circle { .. }), f == 0.0);
                    holds(&end, [&blend, &across], points, &inside);
                    ends += 1;
                }
            }
        }
        assert_eq!(ends, 56);
    }

    /// The open arc's torus ending on a cylinder across (the census's
    /// third family): the boss of the test above, `R`, its foot blended at
    /// `r`, running into a second boss of radius `R₂` whose axis is
    /// parallel to the first `D` from it — two bosses merged on a plate,
    /// the second's wall cutting the first's foot fillet. Each contact is
    /// a circle about the first axis meeting the second boss's in the
    /// plane square to the axes, in closed form; the end is a torus
    /// against a cylinder off its axis, traced on the torus.
    #[test]
    fn a_torus_end_on_a_cylinder_across_is_traced() {
        let mut ends = 0;
        for pose in poses() {
            for (big, r, second, apart) in [
                (3.0, 0.5, 2.0, 4.0),
                (3.0, 0.5, 2.0, 1.5),
                (1.0, 0.25, 3.0, 3.5),
                (10.0, 0.1, 1.0, 10.5),
                (2.0, 0.4, 0.5, 2.2),
            ] {
                let ring = big + r;
                let blend = Surface::Torus {
                    frame: framed(&pose, Point3::new(0.0, 0.0, r), Vec3::z(), Vec3::x()),
                    major_radius: ring,
                    minor_radius: r,
                };
                let across = Surface::Cylinder {
                    frame: framed(&pose, Point3::new(apart, 0.0, 0.0), Vec3::z(), Vec3::x()),
                    radius: second,
                };
                // The circle of `rho` about the first axis meets the second
                // boss's at `x`, on the `+y` side.
                let meet = |rho: f64, z: f64| {
                    let x = (rho * rho - second * second + apart * apart) / (2.0 * apart);
                    pose.to_world(Point3::new(x, (rho * rho - x * x).sqrt(), z))
                };
                let points = [meet(ring, 0.0), meet(big, r)];
                let inside = |p: Point3| -> Result<bool, OpError> {
                    let q = pose.to_local(p);
                    let slack = tol().linear;
                    Ok(q.x.hypot(q.y) <= ring + slack && q.z <= r + slack)
                };
                let within = Aabb::of_point(points[0])
                    .union(Aabb::of_point(points[1]))
                    .inflated(2.0 * r);
                let end = traced_end(
                    [&blend, &across],
                    points,
                    &within,
                    &inside,
                    &refused,
                    tol(),
                    &mut Meter::default(),
                )
                .unwrap_or_else(|e| panic!("R {big}, r {r}, R2 {second}, D {apart}: {e}"));
                assert!(matches!(end.curve, Curve::Nurbs(_)));
                holds(&end, [&blend, &across], points, &inside);
                ends += 1;
            }
        }
        assert_eq!(ends, 10);
    }

    /// A ruling stripe ending on a cylinder across whose axis is square
    /// to it (ADR-0036 §6, the quartic): a bar along `x` with its edge on
    /// the `x` axis between the faces `y = 0` and `z = 0`, blended convex
    /// at `r` — the cylinder of `r` about the line through `(0, r, r)` —
    /// running into a post of radius `R` whose axis is parallel to `z`,
    /// `c` off the bar's edge across it, which the census's parts have
    /// where a rib meets a boss. Each contact line pierces the post in
    /// closed form; the end between is traced on the rulings. The last
    /// poses put a contact a few hundredths of `R` from grazing the post.
    #[test]
    fn a_ruling_stripe_end_on_a_cylinder_across_is_traced() {
        let mut ends = 0;
        for pose in poses() {
            for (big, r, c) in [
                (5.0, 0.5, 2.0),
                (5.0, 0.5, 0.0),
                (2.0, 0.3, -1.0),
                (1.0, 0.1, 0.5),
                (20.0, 1.0, 3.0),
                (1.0, 0.2, 0.95),
                (1.0, 0.2, -0.75),
            ] {
                let post = 3.0 + big;
                let blend = Surface::Cylinder {
                    frame: framed(&pose, Point3::new(0.0, r, r), Vec3::x(), Vec3::y()),
                    radius: r,
                };
                let across = Surface::Cylinder {
                    frame: framed(&pose, Point3::new(post, c, 0.0), Vec3::z(), Vec3::x()),
                    radius: big,
                };
                // The contacts `(x, r, 0)` on `z = 0` and `(x, 0, r)` on
                // `y = 0`, where they meet the post's near side.
                let pierce = |y: f64| post - (big * big - (y - c) * (y - c)).sqrt();
                let points = [
                    pose.to_world(Point3::new(pierce(r), r, 0.0)),
                    pose.to_world(Point3::new(pierce(0.0), 0.0, r)),
                ];
                // The band: the blend's quarter facing the edge.
                let inside = |p: Point3| -> Result<bool, OpError> {
                    let q = pose.to_local(p);
                    let slack = tol().linear;
                    Ok(q.y <= r + slack && q.z <= r + slack)
                };
                let within = Aabb::of_point(points[0])
                    .union(Aabb::of_point(points[1]))
                    .inflated(2.0 * r);
                let end = traced_end(
                    [&blend, &across],
                    points,
                    &within,
                    &inside,
                    &refused,
                    tol(),
                    &mut Meter::default(),
                )
                .unwrap_or_else(|e| panic!("R {big}, r {r}, c {c}: {e}"));
                assert!(matches!(end.curve, Curve::Nurbs(_)));
                holds(&end, [&blend, &across], points, &inside);
                ends += 1;
            }
        }
        assert_eq!(ends, 14);
    }

    /// Two trim points on no one branch — the second moved off the wall —
    /// are the caller's refusal, never a guessed stretch.
    #[test]
    fn points_off_the_section_are_refused() {
        let blend = Surface::Torus {
            frame: Frame::world().with_origin(Point3::new(0.0, 0.0, 0.5)),
            major_radius: 3.5,
            minor_radius: 0.5,
        };
        let across = Surface::Plane {
            frame: Frame::new(Point3::new(1.0, 0.0, 0.0), Vec3::x(), Vec3::y()).unwrap(),
        };
        let points = [
            Point3::new(1.0, (3.5_f64 * 3.5 - 1.0).sqrt(), 0.0),
            Point3::new(1.2, (9.0_f64 - 1.44).sqrt(), 0.5),
        ];
        let within = Aabb::of_point(points[0]).inflated(2.0);
        let inside = |_: Point3| -> Result<bool, OpError> { Ok(true) };
        let err = traced_end(
            [&blend, &across],
            points,
            &within,
            &inside,
            &refused,
            tol(),
            &mut Meter::default(),
        )
        .unwrap_err();
        assert!(matches!(
            err,
            OpError::Internal(crate::error::Fault::Invariant { what }) if what == "the test's refusal"
        ));
    }
}
