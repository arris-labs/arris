//! A run-over (ADR-0071): a fillet's contact on face `F` that leaves it
//! through `E`, an edge of `F` that is not the corner's own. Between the
//! two crossings the ball rolls on `E`, touching the blend's other face
//! `H` and `E`'s curve, and its centre runs on the section of `H`'s
//! offset by the radius, toward the ball, with the pipe of that radius
//! about `E`: a cylinder about a line, a torus about a circle. That
//! section is traced and fitted as a blend's end is (ADR-0037). This
//! module finds the crossings, pairs them and traces the spine between
//! them; where it cannot, the refusal names `E`.

use arris_check::domain::FaceDomain;
use arris_geom::region2::Side;
use arris_geom::{Curve, CurveIntersection, GeomError, Surface, intersect_curves};
use arris_math::{Aabb, Frame, Interval, Meter, Point3, Tolerance, shift_into_range};
use arris_topo::{EdgeId, FaceId};

use super::build::Env;
use super::traced::{self, TracedEnd};
use super::{Kind, degenerate};
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild::forward;

/// The points of `E`'s stretch between the crossings, ends included,
/// whose box bounds the trace of a ruled pair (`trace_section`): the
/// spine lies within the ball's radius of that stretch. A sampling
/// density, not a tolerance.
const BOX_SAMPLES: usize = 16;

/// Where a contact meets `E`.
#[derive(Debug, Clone, Copy)]
pub(super) struct Crossing {
    /// The contact's parameter, in its range.
    pub(super) t: f64,
    /// `E`'s parameter, in its edge's range.
    pub(super) s: f64,
    /// The point, on the contact and on `E`.
    pub(super) point: Point3,
    /// The ball's centre there, touching `F`, `E` and `H` at once.
    pub(super) centre: Point3,
}

/// A contact's run over `E`: the two crossings, the stretches of the
/// contact and of `E` between them, and the ball's centre there.
#[derive(Debug, Clone)]
pub(super) struct RunOver {
    /// `E`.
    pub(super) edge: EdgeId,
    /// Where the contact leaves `F`, then where it comes back, by its
    /// parameter.
    pub(super) crossings: [Crossing; 2],
    /// The contact's parameters off `F`, from the first crossing's `t` to
    /// the second's, which may lie a period past it on a closed contact.
    pub(super) off: Interval,
    /// `E`'s parameters the ball rolls on, from the lower of the two
    /// crossings' `s` to the higher, which may lie a period past it on a
    /// closed circle.
    pub(super) along: Interval,
    /// The ball's centre from one crossing's centre to the other's: on
    /// `H`'s offset and on the pipe about `E`, its `lo_first` at
    /// `crossings[0]`.
    pub(super) spine: TracedEnd,
}

/// A contact that leaves its face, as the stripe or the ring that placed
/// it reads it.
pub(super) struct Leaving<'a> {
    /// The blended edge.
    pub(super) blended: EdgeId,
    /// `F`, the face the contact lies on.
    pub(super) face: FaceId,
    /// `H`, the blend's other face.
    pub(super) other: FaceId,
    pub(super) contact: &'a Curve,
    pub(super) range: Interval,
    /// The ball's centres: a stripe's axis, a ring's circle. The centre
    /// at a contact's point is the point's projection onto it.
    pub(super) centres: &'a Curve,
    pub(super) tolerance: f64,
}

/// The refusal of a contact that leaves its face: `too_large` where the
/// contact crosses no edge of `F` or the blend is a chamfer, and
/// otherwise `BlendTooLarge` naming the blended edge and `E`, the first
/// edge of `F` it crosses. A run-over [`run_over`] pairs and traces is
/// refused the same way until its rolled face is built (ADR-0071, plan
/// steps 4 and 5). A fault is returned as itself.
pub(super) fn refusal(
    env: &Env<'_>,
    leaving: Option<&Leaving<'_>>,
    too_large: OpError,
    meter: &mut Meter<'_>,
) -> OpError {
    let (Some(leaving), Kind::Fillet { .. }) = (leaving, env.kind) else {
        return too_large;
    };
    match run_over(env, leaving, meter) {
        Ok(None) => too_large,
        Ok(Some(run)) => crossed(leaving.blended, run.edge),
        Err(e) => e,
    }
}

/// `BlendTooLarge` naming the blended edge and the edge its contact
/// crosses.
fn crossed(blended: EdgeId, edge: EdgeId) -> OpError {
    degenerate(
        vec![forward(blended), forward(edge)],
        Reason::Blend(BlendReason::TooLarge),
    )
}

/// The run-over of a fillet's contact that leaves its face (ADR-0071
/// §1, §3): `None` where it crosses no edge of `F` away from its ends.
///
/// Guarantees: on `Some`, the contact crosses one edge `E` of `F` exactly
/// twice, neither a touch, and no other edge; `E` is convex and a line
/// or a circle shorter in radius than the ball is not; one stretch of the
/// contact between the crossings lies outside `F`, and its crossings are
/// in that order; the spine runs between the two crossings' centres
/// within the trace's fit (`traced::traced_end`). Each refusal is
/// `BlendTooLarge` naming the blended edge and `E`.
pub(super) fn run_over(
    env: &Env<'_>,
    leaving: &Leaving<'_>,
    meter: &mut Meter<'_>,
) -> Result<Option<RunOver>, OpError> {
    let m = env.m;
    let Kind::Fillet { radius } = env.kind else {
        return Ok(None);
    };
    let tol = Tolerance::new(leaving.tolerance, env.tol.angular);
    let domain = FaceDomain::of(m, leaving.face, m.face(leaving.face)?.tolerance())?;
    let mut hits: Vec<(EdgeId, Hit)> = Vec::new();
    for &x in domain.edges() {
        if x == leaving.blended {
            continue;
        }
        let Some((curve, range)) = m.edge(x)?.curve() else {
            continue;
        };
        let found = crossings(
            leaving.contact,
            leaving.range,
            m.curve(curve)?,
            range,
            tol,
            meter,
        );
        match found {
            Ok(Some(found)) => hits.extend(found.into_iter().map(|h| (x, h))),
            Ok(None) | Err(GeomError::Unsupported { .. }) => {
                return Err(crossed(leaving.blended, x));
            }
            Err(e) => return Err(fault_of(e)),
        }
    }
    let Some(&(e, _)) = hits.first() else {
        return Ok(None);
    };
    let refuse = || crossed(leaving.blended, e);
    let [a, b] = hits.as_slice() else {
        return Err(refuse());
    };
    if a.0 != b.0 || a.1.tangent || b.1.tangent {
        return Err(refuse());
    }
    let (curve, range) = m.edge(e)?.curve().ok_or_else(refuse)?;
    let edge_curve = m.curve(curve)?;
    let pipe = pipe(edge_curve, radius).ok_or_else(refuse)?;
    if env.view.convex(m, e)? != Some(true) {
        return Err(refuse());
    }
    // The stretch of the contact between the crossings that lies outside
    // `F`: the one between them on an open contact, either way round a
    // closed one.
    let (t0, t1) = if a.1.t <= b.1.t {
        (a.1, b.1)
    } else {
        (b.1, a.1)
    };
    let mut stretches = vec![(t0, t1, t1.t)];
    if let Some(p) = leaving.contact.period()
        && leaving.range.length() >= p - tol.linear
    {
        stretches.push((t1, t0, t0.t + p));
    }
    let face_surface = m.surface(m.face(leaving.face)?.surface())?;
    let mut off = None;
    for (first, second, end) in stretches {
        let mid = leaving.contact.point(0.5 * (first.t + end));
        let uv = face_surface.project(mid).map_err(fault_of)?.uv;
        if domain.side(uv).0 == Side::Outside && off.replace((first, second, end)).is_some() {
            return Err(refuse());
        }
    }
    let (first, second, end) = off.ok_or_else(refuse)?;
    let off = Interval::new(first.t, end).map_err(|_| refuse())?;
    let along = stretch_along(edge_curve, range, [first.s, second.s], {
        leaving.contact.point(off.midpoint())
    })
    .ok_or_else(refuse)?;
    let crossings = [first, second].map(|h| {
        leaving
            .centres
            .project(h.point)
            .map(|c| Crossing {
                t: h.t,
                s: h.s,
                point: h.point,
                centre: c.point,
            })
            .map_err(fault_of)
    });
    let [c0, c1] = crossings;
    let crossings = [c0?, c1?];
    let other = m.surface(m.face(leaving.other)?.surface())?;
    let offset =
        offset_through(other, radius, crossings[0].centre, leaving.tolerance).ok_or_else(refuse)?;
    let spine = rolled_spine(
        [&offset, &pipe],
        &crossings,
        edge_curve,
        along,
        radius,
        &refuse,
        tol,
        meter,
    )?;
    Ok(Some(RunOver {
        edge: e,
        crossings,
        off,
        along,
        spine,
    }))
}

/// A crossing as [`crossings`] finds it, before its centre is placed.
#[derive(Debug, Clone, Copy)]
struct Hit {
    t: f64,
    s: f64,
    point: Point3,
    tangent: bool,
}

/// Where `contact` over `range` meets an edge's `curve` over its own
/// `edge_range`, away from the contact's ends by `tol.linear`: a point
/// there is the corner's own edge meeting the contact by construction.
/// Each parameter is moved by whole periods into its range. `None` where
/// the two curves coincide.
fn crossings(
    contact: &Curve,
    range: Interval,
    curve: &Curve,
    edge_range: Interval,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Option<Vec<Hit>>, GeomError> {
    let CurveIntersection::Points(points) = intersect_curves(contact, curve, tol, meter)? else {
        return Ok(None);
    };
    let ends = [contact.point(range.lo()), contact.point(range.hi())];
    let mut hits = Vec::new();
    for p in points {
        let t = shift_into_range(range, p.ta, contact.period());
        let s = shift_into_range(edge_range, p.tb, curve.period());
        let (Some(t), Some(s)) = (t, s) else {
            continue;
        };
        if ends.iter().any(|&q| (q - p.point).norm() <= tol.linear) {
            continue;
        }
        hits.push(Hit {
            t,
            s,
            point: p.point,
            tangent: p.tangent,
        });
    }
    Ok(Some(hits))
}

/// The pipe of `radius` about an edge's curve: the cylinder about a line,
/// the torus about a circle whose radius is the larger. `None` for any
/// other curve, or a circle no wider than the ball (ADR-0071 §3).
pub(super) fn pipe(curve: &Curve, radius: f64) -> Option<Surface> {
    match *curve {
        Curve::Line { origin, direction } => Some(Surface::Cylinder {
            frame: Frame::from_z(origin, direction.into_inner()).ok()?,
            radius,
        }),
        Curve::Circle {
            frame,
            radius: major,
        } => (radius < major).then_some(Surface::Torus {
            frame,
            major_radius: major,
            minor_radius: radius,
        }),
        Curve::Ellipse { .. } | Curve::Nurbs(_) => None,
    }
}

/// The offset of `face` by `radius` that holds `centre` within
/// `tolerance`, toward it: the ball's side. `None` where neither does, or
/// the face has no exact offset.
fn offset_through(face: &Surface, radius: f64, centre: Point3, tolerance: f64) -> Option<Surface> {
    [radius, -radius].into_iter().find_map(|d| {
        let offset = face.offset(d)?;
        let distance = offset.project(centre).ok()?.distance;
        (distance <= tolerance).then_some(offset)
    })
}

/// The stretch of an edge's `curve` from one crossing's parameter to the
/// other's, `s` in the edge's `range`: between them on a line or an arc, and on a
/// closed circle the arc of the two whose midpoint is nearer `toward`, the
/// contact's point midway off the face. `None` where the two are one.
fn stretch_along(curve: &Curve, range: Interval, s: [f64; 2], toward: Point3) -> Option<Interval> {
    let (lo, hi) = (s[0].min(s[1]), s[0].max(s[1]));
    let Some(p) = curve.period().filter(|&p| range.length() >= p) else {
        return Interval::new(lo, hi).ok();
    };
    let near = |a: f64, b: f64| (curve.point(0.5 * (a + b)) - toward).norm();
    let (lo, hi) = if near(lo, hi) <= near(hi, lo + p) {
        (lo, hi)
    } else {
        (hi, lo + p)
    };
    Interval::new(lo, hi).ok()
}

/// The ball's centre between the two crossings: the stretch of the
/// section of `surfaces`, `H`'s offset and the pipe about `E`, from one
/// crossing's centre to the other's, on the ball's side of `F` — the
/// side the first crossing's centre stands from its point. The trace is
/// bounded by `E`'s stretch `along` and the two centres, widened by the
/// ball's diameter. Refused by `refuse` where `traced::traced_end` is.
#[allow(clippy::too_many_arguments)]
fn rolled_spine(
    surfaces: [&Surface; 2],
    crossings: &[Crossing; 2],
    edge: &Curve,
    along: Interval,
    radius: f64,
    refuse: &dyn Fn() -> OpError,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<TracedEnd, OpError> {
    let mut within = Aabb::of_point(crossings[0].centre).union(Aabb::of_point(crossings[1].centre));
    for i in 0..=BOX_SAMPLES {
        let s = along.lerp(i as f64 / BOX_SAMPLES as f64);
        within = within.union(Aabb::of_point(edge.point(s)));
    }
    let within = within.inflated(2.0 * radius);
    let [p, c] = [crossings[0].point, crossings[0].centre];
    let ball_side = c - p;
    let inside = |q: Point3| Ok((q - p).dot(&ball_side) > 0.0);
    traced::traced_end(
        surfaces,
        [crossings[0].centre, crossings[1].centre],
        &within,
        &inside,
        refuse,
        tol,
        meter,
    )
}

#[cfg(test)]
mod tests {
    use arris_geom::SECTION_FIT_FRACTION;
    use arris_math::{Axis, Control, Vec3, real};
    use arris_topo::{Body, Model, Shape};

    use super::*;
    use crate::body_view::BodyView;
    use crate::{cut, fillet, fuse, primitive_box, primitive_cylinder};

    /// The edge of `body` whose curve passes through `at`.
    fn edge_through(m: &Model, body: Body, at: Point3) -> EdgeId {
        let found: Vec<EdgeId> = (m.edges(body).unwrap().iter())
            .map(|e| e.id)
            .filter(|&e| {
                let (c, range) = m.edge(e).unwrap().curve().unwrap();
                let curve = m.curve(c).unwrap();
                let near = curve.project(at).unwrap();
                near.distance < 1e-9 && shift_into_range(range, near.t, curve.period()).is_some()
            })
            .collect();
        assert_eq!(found.len(), 1, "one edge through {at:?}");
        found[0]
    }

    /// The face of `body` that holds `at` inside its loops.
    fn face_through(m: &Model, body: Body, at: Point3) -> FaceId {
        let found: Vec<FaceId> = (m.faces(body).unwrap().iter())
            .map(|f| f.id)
            .filter(|&f| {
                let face = m.face(f).unwrap();
                let near = m.surface(face.surface()).unwrap().project(at).unwrap();
                let domain = FaceDomain::of(m, f, face.tolerance()).unwrap();
                near.distance < 1e-9 && domain.side(near.uv).0 == Side::Inside
            })
            .collect();
        assert_eq!(found.len(), 1, "one face through {at:?}");
        found[0]
    }

    /// The pi mount plate's standoff, shrunk (`standoff-root-fillet-runs-
    /// off-the-plate`): a 10 × 10 × 2 plate, a boss of radius 2 on its top
    /// whose axis is 2.3 from the front face `y = 0`.
    fn plate(m: &mut Model) -> Body {
        let none = &Control::NONE;
        let (plate, _) = primitive_box(m, [0.0, 0.0, 0.0], [10.0, 10.0, 2.0], none).unwrap();
        let axis = Axis::new(Point3::new(5.0, 2.3, 1.0), Vec3::z()).unwrap();
        let (boss, _) = primitive_cylinder(m, axis, 2.0, 3.0, none).unwrap();
        fuse(m, plate, boss, none).unwrap().0
    }

    /// The phone stand's lip, shrunk (`lip-fillet-runs-into-a-hole`): a
    /// 12 × 20 × 8 slab with a hole of radius 3.5 across it along `x`,
    /// its axis at `y = 10`, `z = 4`.
    fn slab(m: &mut Model) -> Body {
        let none = &Control::NONE;
        let (slab, _) = primitive_box(m, [0.0, 0.0, 0.0], [12.0, 20.0, 8.0], none).unwrap();
        let axis = Axis::new(Point3::new(-1.0, 10.0, 4.0), Vec3::x()).unwrap();
        let (drill, _) = primitive_cylinder(m, axis, 3.5, 14.0, none).unwrap();
        cut(m, slab, drill, none).unwrap().0
    }

    /// The run-over of `leaving` on `body` at `radius`.
    fn run(m: &Model, body: Body, leaving: &Leaving<'_>, radius: f64) -> RunOver {
        let view = BodyView::of(m, body).unwrap();
        let env = Env {
            m,
            view: &view,
            kind: Kind::Fillet { radius },
            tol: m.precision().tolerance(),
            samples: m.precision().check_samples,
        };
        let mut meter = Meter::new(&Control::NONE);
        run_over(&env, leaving, &mut meter).unwrap().unwrap()
    }

    /// What a test asks of a spine: every one of its samples within the
    /// fit's fraction of the tolerance of both `on` distances (and their
    /// rounding), its ends at the crossings' centres, and `through` on it.
    fn holds(run: &RunOver, on: [&dyn Fn(Point3) -> f64; 2], through: Point3, tolerance: f64) {
        let spine = &run.spine;
        let samples = 256;
        for i in 0..=samples {
            let p = spine
                .curve
                .point(spine.range.lerp(i as f64 / samples as f64));
            for (k, off) in on.iter().enumerate() {
                let d = off(p).abs();
                assert!(
                    d <= SECTION_FIT_FRACTION * tolerance + 1e-12,
                    "{d:e} off {k} at {i}"
                );
            }
        }
        let (lo, hi) = (spine.range.lo(), spine.range.hi());
        let first = spine.curve.point(if spine.lo_first { lo } else { hi });
        let last = spine.curve.point(if spine.lo_first { hi } else { lo });
        assert!((first - run.crossings[0].centre).norm() <= tolerance);
        assert!((last - run.crossings[1].centre).norm() <= tolerance);
        let near = spine.curve.project(through).unwrap();
        assert!(
            near.distance <= tolerance,
            "{:e} from {through:?}",
            near.distance
        );
        assert!(spine.range.contains(near.t));
    }

    /// What a test asks of the stretches between the crossings: each
    /// crossing at its parameters on the contact and on `E`, the contact's
    /// stretch off the face from the first to the second through `off_mid`,
    /// and `E`'s stretch the ball rolls on through `along_mid`.
    fn stretches(
        m: &Model,
        run: &RunOver,
        contact: &Curve,
        [off_mid, along_mid]: [Point3; 2],
        tolerance: f64,
    ) {
        let (curve, _) = m.edge(run.edge).unwrap().curve().unwrap();
        let edge = m.curve(curve).unwrap();
        for c in run.crossings {
            assert!((contact.point(c.t) - c.point).norm() <= tolerance);
            assert!((edge.point(c.s) - c.point).norm() <= tolerance);
        }
        // The second crossing's `t`, or a turn past it round a closed contact.
        let turn = contact.period().unwrap_or(0.0);
        assert_eq!(run.off.lo(), run.crossings[0].t);
        let past = run.off.hi() - run.crossings[1].t;
        assert!(
            [0.0, turn].iter().any(|&k| (past - k).abs() <= 1e-12),
            "{past}"
        );
        let at = contact.point(run.off.midpoint());
        assert!((at - off_mid).norm() <= tolerance, "off the face at {at:?}");
        let at = edge.point(run.along.midpoint());
        assert!((at - along_mid).norm() <= tolerance, "rolled on at {at:?}");
    }

    /// The concave ring's contact on the plate's top, a circle of radius
    /// 2.6 about the boss's axis, leaves the top through its front edge
    /// at `x = 5 ± √(2.6² − 2.3²)`. Between, the ball's centre is on the
    /// boss's offset, a cylinder of 2.6, and on the pipe of 0.6 about the
    /// edge's line, through `(5, −0.3, 2 + √0.27)` midway.
    #[test]
    fn a_ring_over_an_outer_edge_rolls_on_it() {
        let mut m = Model::default();
        let body = plate(&mut m);
        let tolerance = m.precision().tolerance().linear;
        let r = 0.6;
        let root = edge_through(&m, body, Point3::new(5.0, 4.3, 2.0));
        let front = edge_through(&m, body, Point3::new(5.0, 0.0, 2.0));
        let top = face_through(&m, body, Point3::new(1.0, 9.0, 2.0));
        let boss = face_through(&m, body, Point3::new(5.0, 4.3, 2.5));
        let circle = |z: f64| Curve::Circle {
            frame: Frame::from_z(Point3::new(5.0, 2.3, z), Vec3::z()).unwrap(),
            radius: 2.6,
        };
        let (contact, centres) = (circle(2.0), circle(2.0 + r));
        let leaving = Leaving {
            blended: root,
            face: top,
            other: boss,
            contact: &contact,
            range: Interval::new(0.0, core::f64::consts::TAU).unwrap(),
            centres: &centres,
            tolerance,
        };
        let run = run(&m, body, &leaving, r);
        assert_eq!(run.edge, front);
        let dx = (2.6f64 * 2.6 - 2.3 * 2.3).sqrt();
        let mut xs = run.crossings.map(|c| c.point.x);
        xs.sort_by(f64::total_cmp);
        for (x, want) in xs.into_iter().zip([5.0 - dx, 5.0 + dx]) {
            assert!(
                (x - want).abs() <= tolerance,
                "a crossing at x {x}, not {want}"
            );
        }
        for c in run.crossings {
            let want = Point3::new(c.point.x, 0.0, 2.0 + r);
            assert!((c.centre - want).norm() <= tolerance, "{:?}", c.centre);
        }
        let to_boss = |p: Point3| real::hypot(p.x - 5.0, p.y - 2.3) - 2.6;
        let to_edge = |p: Point3| real::hypot(p.y, p.z - 2.0) - r;
        let mid = Point3::new(5.0, -0.3, 2.0 + 0.27f64.sqrt());
        holds(&run, [&to_boss, &to_edge], mid, tolerance);
        let off_mid = Point3::new(5.0, -0.3, 2.0);
        let along_mid = Point3::new(5.0, 0.0, 2.0);
        stretches(&m, &run, &contact, [off_mid, along_mid], tolerance);
    }

    /// The convex blend's contact on the slab's end `x = 12`, the line at
    /// `z = 6`, runs into the hole through its rim at
    /// `y = 10 ± √(3.5² − 2²)`. Between, the ball's centre is on the
    /// top's offset `z = 6` and on the torus of 2 about the rim, through
    /// `(12 − √1.75, 10, 6)` midway.
    #[test]
    fn a_stripe_into_a_hole_rolls_on_its_rim() {
        let mut m = Model::default();
        let body = slab(&mut m);
        let tolerance = m.precision().tolerance().linear;
        let r = 2.0;
        let lip = edge_through(&m, body, Point3::new(12.0, 1.0, 8.0));
        let rim = edge_through(&m, body, Point3::new(12.0, 10.0, 0.5));
        let end = face_through(&m, body, Point3::new(12.0, 1.0, 1.0));
        let top = face_through(&m, body, Point3::new(6.0, 1.0, 8.0));
        let line = |x: f64| Curve::Line {
            origin: Point3::new(x, 0.0, 8.0 - r),
            direction: Vec3::y_axis(),
        };
        let (contact, centres) = (line(12.0), line(12.0 - r));
        let leaving = Leaving {
            blended: lip,
            face: end,
            other: top,
            contact: &contact,
            range: Interval::new(0.0, 20.0).unwrap(),
            centres: &centres,
            tolerance,
        };
        let run = run(&m, body, &leaving, r);
        assert_eq!(run.edge, rim);
        let dy = (3.5f64 * 3.5 - 2.0 * 2.0).sqrt();
        let ys = run.crossings.map(|c| c.point.y);
        for (y, want) in ys.into_iter().zip([10.0 - dy, 10.0 + dy]) {
            assert!(
                (y - want).abs() <= tolerance,
                "a crossing at y {y}, not {want}"
            );
        }
        for c in run.crossings {
            let want = Point3::new(12.0 - r, c.point.y, 6.0);
            assert!((c.centre - want).norm() <= tolerance, "{:?}", c.centre);
        }
        let to_top = |p: Point3| p.z - 6.0;
        let to_rim =
            |p: Point3| real::hypot(real::hypot(p.y - 10.0, p.z - 4.0) - 3.5, p.x - 12.0) - r;
        let mid = Point3::new(12.0 - 1.75f64.sqrt(), 10.0, 6.0);
        holds(&run, [&to_top, &to_rim], mid, tolerance);
        let off_mid = Point3::new(12.0, 10.0, 6.0);
        let along_mid = Point3::new(12.0, 10.0, 7.5);
        stretches(&m, &run, &contact, [off_mid, along_mid], tolerance);
    }

    /// Until the rolled face is built, both fillets are refused naming the
    /// blended edge and the edge their contact crosses: the ring's on the
    /// plate's front edge, the stripe's on the hole's rim.
    #[test]
    fn a_run_over_is_refused_naming_the_crossed_edge() {
        type Part = fn(&mut Model) -> Body;
        let cases: [(Part, Point3, Point3, f64); 2] = [
            (
                plate,
                Point3::new(5.0, 4.3, 2.0),
                Point3::new(5.0, 0.0, 2.0),
                0.6,
            ),
            (
                slab,
                Point3::new(12.0, 1.0, 8.0),
                Point3::new(12.0, 10.0, 0.5),
                2.0,
            ),
        ];
        for (part, blended, crossed, r) in cases {
            let mut m = Model::default();
            let body = part(&mut m);
            let (blended, crossed) = (
                edge_through(&m, body, blended),
                edge_through(&m, body, crossed),
            );
            let edge = (m.edges(body).unwrap().into_iter())
                .find(|e| e.id == blended)
                .unwrap();
            let err = fillet(&mut m, body, &[edge], r, &Control::NONE).unwrap_err();
            let OpError::Degenerate { entities, reason } = &err else {
                panic!("{err}");
            };
            assert_eq!(*reason, Reason::Blend(BlendReason::TooLarge), "{err}");
            let want: [Shape; 2] = [forward(blended), forward(crossed)];
            assert_eq!(entities[..], want, "{err}");
        }
    }
}
