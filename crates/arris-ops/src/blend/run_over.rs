//! A run-over (ADR-0071, amended): a fillet's contact on face `F` that
//! leaves it through `E`, an edge of `F` that is not the corner's own.
//! Between the two crossings each section of the blend, in the stripe's
//! own plane square to its centres there, is the circle of the radius
//! tangent to the blend's other face `H` through the point where `E`
//! crosses that plane: its centre on `H`'s offset there, at the radius
//! from `E`'s point. This module finds the crossings, pairs them and
//! finds `E`'s point in every section between; where it cannot, the
//! refusal names `E`.

use arris_check::domain::FaceDomain;
use arris_geom::region2::Side;
use arris_geom::{
    Curve, CurveIntersection, GeomError, SECTION_FIT_FRACTION, Surface, intersect_curves,
};
use arris_math::{Interval, Meter, Point3, Tolerance, Vec3, is_negligible, shift_into_range};
use arris_topo::{EdgeId, FaceId};

use super::build::Env;
use super::walked::{NEWTON_STEPS, STEP_ROUNDING};
use super::{Kind, degenerate};
use crate::error::{BlendReason, OpError, Reason, fault_of};
use crate::rebuild::forward;

/// The sections between the crossings at which `E`'s point is found in
/// order, each from the one before: the table a section anywhere between
/// starts from. A sampling density, not a tolerance.
const SECTION_SAMPLES: usize = 32;

/// Where a contact meets `E`.
#[derive(Debug, Clone, Copy)]
pub(super) struct Crossing {
    /// The contact's parameter, in its range.
    pub(super) t: f64,
    /// `E`'s parameter, in its edge's range.
    pub(super) s: f64,
    /// The point, on the contact and on `E`.
    pub(super) point: Point3,
    /// The section's centre there, the ball touching `F`, `E` and `H` at
    /// once.
    pub(super) centre: Point3,
}

/// A section of the blend between the crossings (ADR-0071, amended): its circle's
/// centre, its foot on `H` and `E`'s point, all in the stripe's plane at
/// the contact's parameter.
#[derive(Debug, Clone, Copy)]
pub(super) struct RolledSection {
    pub(super) centre: Point3,
    pub(super) foot: Point3,
    pub(super) point: Point3,
    /// `E`'s parameter at `point`.
    pub(super) s: f64,
}

/// A contact's run over `E`: the two crossings, the stretches of the
/// contact and of `E` between them, and what a section between them is
/// found from.
#[derive(Debug, Clone)]
pub(super) struct RunOver {
    /// The blended edge.
    pub(super) blended: EdgeId,
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
    /// `E`'s parameter in [`SECTION_SAMPLES`] sections evenly over `off`,
    /// ends included, each found from the one before.
    table: Vec<f64>,
    /// The centres' curve, `H`'s surface, `E`'s curve and the radius.
    centres: Curve,
    other: Surface,
    edge_curve: Curve,
    radius: f64,
    tolerance: f64,
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
/// edge of `F` it crosses. A fault is returned as itself.
pub(super) fn refusal(
    env: &Env<'_>,
    leaving: Option<&Leaving<'_>>,
    too_large: OpError,
    meter: &mut Meter<'_>,
) -> OpError {
    match run(env, leaving, too_large, meter) {
        Ok(run) => crossed(run.blended, run.edge),
        Err(e) => e,
    }
}

/// The run-over of a contact that leaves its face, or its [`refusal`]:
/// `too_large` where the contact crosses no edge of `F` or the blend is a
/// chamfer, `BlendTooLarge` naming the blended edge and `E` where
/// [`run_over`] refuses it.
pub(super) fn run(
    env: &Env<'_>,
    leaving: Option<&Leaving<'_>>,
    too_large: OpError,
    meter: &mut Meter<'_>,
) -> Result<RunOver, OpError> {
    let (Some(leaving), Kind::Fillet { .. }) = (leaving, env.kind) else {
        return Err(too_large);
    };
    run_over(env, leaving, meter)?.ok_or(too_large)
}

/// `BlendTooLarge` naming the blended edge and the edge its contact
/// crosses.
pub(super) fn crossed(blended: EdgeId, edge: EdgeId) -> OpError {
    degenerate(
        vec![forward(blended), forward(edge)],
        Reason::Blend(BlendReason::TooLarge),
    )
}

/// The run-over of a fillet's contact that leaves its face (ADR-0071
/// §3 and its amendment): `None` where it crosses no edge of `F` away
/// from its ends.
///
/// Guarantees: on `Some`, the contact crosses one edge `E` of `F` exactly
/// twice, neither a touch, and no other edge; `E` is convex and a line
/// or a circle; one stretch of the contact between the crossings lies
/// outside `F`, and its crossings are in that order; every one of
/// [`SECTION_SAMPLES`] sections between them has its circle
/// ([`RunOver::section`]), `E`'s point running from the first crossing to
/// the second. Each refusal is `BlendTooLarge` naming the blended edge
/// and `E`.
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
    match edge_curve {
        Curve::Line { .. } | Curve::Circle { .. } => {}
        Curve::Ellipse { .. } | Curve::Nurbs(_) => return Err(refuse()),
    }
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
    let mut run = RunOver {
        blended: leaving.blended,
        edge: e,
        crossings,
        off,
        along,
        table: Vec::with_capacity(SECTION_SAMPLES + 1),
        centres: leaving.centres.clone(),
        other: m.surface(m.face(leaving.other)?.surface())?.clone(),
        edge_curve: edge_curve.clone(),
        radius,
        tolerance: leaving.tolerance,
    };
    // `E`'s point from the first crossing's on, each section's from the
    // one before, to the second crossing's.
    let mut s = crossings[0].s;
    for i in 0..=SECTION_SAMPLES {
        meter.tick()?;
        let t = off.lerp(i as f64 / SECTION_SAMPLES as f64);
        s = run.section_from(t, s).ok_or_else(refuse)?.s;
        run.table.push(s);
    }
    let last = run.edge_curve.point(s);
    if (last - crossings[1].point).norm() > leaving.tolerance {
        return Err(refuse());
    }
    Ok(Some(run))
}

impl RunOver {
    /// The section at the contact's parameter `t` in `off` (ADR-0071, amended),
    /// `E`'s point found from the table's nearest sample before it.
    /// `None` where it is not found, or no circle of the radius tangent to
    /// `H` passes through it.
    pub(super) fn section(&self, t: f64) -> Option<RolledSection> {
        let at = ((t - self.off.lo()) / self.off.length() * SECTION_SAMPLES as f64).floor();
        let i = (at.max(0.0) as usize).min(SECTION_SAMPLES);
        self.section_from(t, *self.table.get(i)?)
    }

    /// The section at `t`, `E`'s point found by Newton's iteration from
    /// its parameter `seed`: where `E` crosses the plane through the
    /// centres' point at `t` square to them. The centre is on `H`'s offset
    /// in that plane, the line through the centres' point square to `H`'s
    /// normal, at the radius from `E`'s point: of the two such points the
    /// nearer the centres' point, which the crossings are at. The circle
    /// is tangent to `H` where its centre is at the radius from it within
    /// the section fit's share of the tolerance.
    fn section_from(&self, t: f64, seed: f64) -> Option<RolledSection> {
        let at = self.centres.eval(t);
        let (a, d) = (at.point, at.d1.try_normalize(0.0)?);
        let mut s = seed;
        let mut residual = f64::INFINITY;
        for _ in 0..NEWTON_STEPS {
            let e = self.edge_curve.eval(s);
            residual = (e.point - a).dot(&d);
            let slope = e.d1.dot(&d);
            if slope == 0.0 {
                return None;
            }
            let step = residual / slope;
            s -= step;
            if is_negligible(
                (step * e.d1).norm(),
                STEP_ROUNDING * (a.coords.norm() + self.radius),
            ) {
                break;
            }
        }
        if !residual.is_finite() {
            return None;
        }
        let point = self.edge_curve.point(s);
        if (point - a).dot(&d).abs() > SECTION_FIT_FRACTION * self.tolerance {
            return None;
        }
        let uv = self.other.project(a).ok()?.uv;
        let normal = self.other.normal(uv.x, uv.y)?.into_inner();
        let w: Vec3 = d.cross(&normal).try_normalize(0.0)?;
        // `|a + λw − point| = r`: `λ² + 2bλ + c = 0`, its smaller root.
        let g = a - point;
        let (b, c) = (w.dot(&g), g.norm_squared() - self.radius * self.radius);
        let disc = b * b - c;
        if disc < 0.0 {
            return None;
        }
        let large = -b - if b < 0.0 { -1.0 } else { 1.0 } * disc.sqrt();
        let lambda = if large == 0.0 { 0.0 } else { c / large };
        let centre = a + lambda * w;
        let foot = self.other.project(centre).ok()?;
        if (foot.distance - self.radius).abs() > SECTION_FIT_FRACTION * self.tolerance {
            return None;
        }
        Some(RolledSection {
            centre,
            foot: foot.point,
            point,
            s,
        })
    }
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

#[cfg(test)]
mod tests {
    use arris_math::Frame;
    use arris_math::{Axis, Control, Vec3, real};
    use arris_topo::{Body, Model};

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

    /// What a test asks of the sections between the crossings (ADR-0071,
    /// amended): at every one of 256 sections over `off`, the centre at
    /// `closed`'s place for the centres' point there, at the radius from
    /// `H` by `to_h` and from `E`'s point, which is on `E` by `to_e` and in
    /// the plane square to the centres; at the crossings, their centres;
    /// and `through` the centre midway.
    fn holds(
        run: &RunOver,
        closed: &dyn Fn(Point3) -> Point3,
        [to_h, to_e]: [&dyn Fn(Point3) -> f64; 2],
        through: Point3,
    ) {
        let near = 1e-9;
        let samples = 256;
        for i in 0..=samples {
            let t = run.off.lerp(i as f64 / samples as f64);
            let at = run
                .section(t)
                .unwrap_or_else(|| panic!("no section at {t}"));
            let a = run.centres.eval(t);
            let want = closed(a.point);
            assert!(
                (at.centre - want).norm() <= near,
                "{i}: {:?} not {want:?}",
                at.centre
            );
            assert!((to_h(at.centre) - run.radius).abs() <= near, "{i}: off H");
            assert!(
                ((at.centre - at.point).norm() - run.radius).abs() <= near,
                "{i}"
            );
            assert!(to_e(at.point).abs() <= near, "{i}: {:?} off E", at.point);
            assert!(
                (at.point - a.point).dot(&a.d1.normalize()).abs() <= near,
                "{i}"
            );
            assert!(
                (at.foot - at.centre).norm() - run.radius <= near,
                "{i}: the foot"
            );
        }
        for (c, t) in run.crossings.iter().zip([run.off.lo(), run.off.hi()]) {
            let at = run.section(t).unwrap();
            assert!((at.centre - c.centre).norm() <= near, "{:?}", at.centre);
            assert!((at.point - c.point).norm() <= near, "{:?}", at.point);
        }
        let mid = run.section(run.off.midpoint()).unwrap().centre;
        assert!((mid - through).norm() <= near, "{mid:?} not {through:?}");
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
    /// at `x = 5 ± √(2.6² − 2.3²)`. Between, each section in its meridian
    /// plane is the circle of 0.6 centred on the boss's offset, a cylinder
    /// of 2.6, through the front edge's point there: through
    /// `(5, −0.3, 2 + √0.27)` midway, where it is the ball touching the
    /// edge.
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
        // In the meridian plane through the centres' point, the front edge's
        // point at `ρ` from the axis, the centre at 2.6 from it and 0.6 from
        // that point.
        let closed = |a: Point3| {
            let (ux, uy) = ((a.x - 5.0) / 2.6, (a.y - 2.3) / 2.6);
            let rho = -2.3 / uy;
            let z = 2.0 + (r * r - (rho - 2.6) * (rho - 2.6)).sqrt();
            Point3::new(5.0 + 2.6 * ux, 2.3 + 2.6 * uy, z)
        };
        let to_boss = |p: Point3| real::hypot(p.x - 5.0, p.y - 2.3) - 2.0;
        let to_edge = |p: Point3| real::hypot(p.y, p.z - 2.0);
        let mid = Point3::new(5.0, -0.3, 2.0 + 0.27f64.sqrt());
        holds(&run, &closed, [&to_boss, &to_edge], mid);
        let off_mid = Point3::new(5.0, -0.3, 2.0);
        let along_mid = Point3::new(5.0, 0.0, 2.0);
        stretches(&m, &run, &contact, [off_mid, along_mid], tolerance);
    }

    /// The convex blend's contact on the slab's end `x = 12`, the line at
    /// `z = 6`, runs into the hole through its rim at
    /// `y = 10 ± √(3.5² − 2²)`. Between, each section in its plane
    /// `y = t` is the circle of 2 centred on the top's offset `z = 6`
    /// through the rim's point there: through `(12 − √1.75, 10, 6)` midway,
    /// where it is the ball touching the rim.
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
        // In the plane `y = t`, the rim's point at `z = 4 + h`, the centre
        // on `z = 6` at 2 from it.
        let closed = |a: Point3| {
            let h = (3.5 * 3.5 - (a.y - 10.0) * (a.y - 10.0)).sqrt();
            Point3::new(12.0 - (r * r - (h - 2.0) * (h - 2.0)).sqrt(), a.y, 6.0)
        };
        let to_top = |p: Point3| 8.0 - p.z;
        let to_rim = |p: Point3| real::hypot(real::hypot(p.y - 10.0, p.z - 4.0) - 3.5, p.x - 12.0);
        let mid = Point3::new(12.0 - 1.75f64.sqrt(), 10.0, 6.0);
        holds(&run, &closed, [&to_top, &to_rim], mid);
        let off_mid = Point3::new(12.0, 10.0, 6.0);
        let along_mid = Point3::new(12.0, 10.0, 7.5);
        stretches(&m, &run, &contact, [off_mid, along_mid], tolerance);
    }

    /// Both run-overs build: the lip's, its stripe cut into two faces
    /// either side of the rolled face (10 faces in all), and the standoff
    /// root's, its torus left in one face open at the rolled face (10).
    #[test]
    fn both_run_overs_build_with_a_rolled_face() {
        let mut m = Model::default();
        let body = slab(&mut m);
        let lip = edge_through(&m, body, Point3::new(12.0, 1.0, 8.0));
        let edge = (m.edges(body).unwrap().into_iter())
            .find(|e| e.id == lip)
            .unwrap();
        let (built, _) = fillet(&mut m, body, &[edge], 2.0, &Control::NONE).unwrap();
        assert_eq!(m.faces(built).unwrap().len(), 10);
        let mut m = Model::default();
        let body = plate(&mut m);
        let root = edge_through(&m, body, Point3::new(5.0, 4.3, 2.0));
        let edge = (m.edges(body).unwrap().into_iter())
            .find(|e| e.id == root)
            .unwrap();
        let (built, _) = fillet(&mut m, body, &[edge], 0.6, &Control::NONE).unwrap();
        assert_eq!(m.faces(built).unwrap().len(), 10);
    }
}
