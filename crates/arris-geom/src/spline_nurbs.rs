//! A NURBS curve against a NURBS surface (`docs/DATA-MODEL.md`
//! §Curve–surface intersection): the arm of the table of one unknown
//! fewer than the section of two NURBS surfaces (ADR-0068).
//!
//! On a Bézier span of the curve, `C = A/a` over its own `t` in
//! `[0, 1]`, and a Bézier patch of the surface, `S = X/W` over its own
//! `(s, u)` in `[0, 1]²`, the three components of `D = W·A − a·X` are
//! tensor polynomials in `(t, s, u)` of degree `(p, p₁, q₁)`, and
//! `D = aW·(C − S)` with `aW > 0`: their common zeros are where the curve
//! meets the patch, and `|D| / aW` is how far apart the two points are.
//! `crate::bernstein_n` isolates them and certifies each simple one, a
//! **crossing**. A zero it cannot certify — a double one, the curve
//! tangent to the surface there, or a cluster — comes back as a box.
//!
//! **Touches.** What is decided is decided on the distance along the
//! curve, as every arm of the table decides it: a stretch where it has
//! an extremum within `τ = tol.linear` is one `tangent` hit at the
//! extremum, absorbing the crossings beside it. The extrema near the
//! surface are the critical points of `|C − S|²` in `(t, s, u)`, the
//! common zeros of `D·(aA′ − a′A)`, `D·(W·X_s − W_s·X)` and
//! `D·(W·X_u − W_u·X)` — positive multiples of `(C − S)·C′`,
//! `(C − S)·S_s` and `(C − S)·S_u` — kept by one gate, the tube
//! `|D|² − τ²(aW)² ≤ 0`: a curve passing within `τ` of the surface
//! without meeting it touches it at one of them, as a line does
//! (`crate::nurbs::line`). A crossing is one of them too; what tells it
//! from an extremum is the **second-order model** of the distance along
//! the curve, as the line arm reads one: where the curve meets the
//! surface at the angle `θ` to its tangent plane, and bends towards the
//! normal by `k` against the surface along the curve's direction, the
//! extremum beside the point is at the depth `sin² θ / (2|k|)`, and a
//! point whose extremum is within `τ` is a touch. Every box the
//! isolations could not certify, and up to [`FRONT_STARTS`] boxes spread
//! along each front of zeros that are not isolated, is searched by a
//! descent on `|C − S|²` from its middle, and the model decides what it
//! finds.
//!
//! The **stops** — crossings, extrema within `τ`, an open curve's ends
//! within `τ`, and the poles of the surface's net the curve passes within
//! `τ` — are then read along the curve. Two consecutive ones are one run
//! where the curve stays within `τ` of the surface between them, at
//! [`JOIN_SAMPLES`] points; a run is one hit: a lone crossing a
//! transversal one; a run holding a pole at the pole's own parameters,
//! `tangent` as the model reads it there; one holding an extremum at the
//! extremum nearest the surface, `tangent`; one holding an end at that
//! end, not `tangent` — a section edge ending on a face, as
//! `crate::by_distance` reads one; and crossings alone at the farthest
//! point between them, `tangent`. A **pole** — a row of the net collapsed
//! to one point — makes the whole row a zero of `D` wherever the curve
//! goes through it: the isolation gives up there, and the hit is the
//! pole's, at its own parameters, the other one at the start of its
//! knots.
//!
//! **Coincident** is a curve within `τ` of the surface at every one of
//! [`SPAN_SAMPLES`] points of every span, looked at where the isolation
//! met a front of zeros that are not isolated or found a stop off a
//! crossing.

use arris_math::nalgebra::Matrix3;
use arris_math::{Meter, Point2, Point3, Tolerance, Vec3};

use crate::bernstein::Binomials;
use crate::bernstein_n::{
    Continuum, Gate, Isolation, PolyN, Wrap, Zero, common_zeros, common_zeros_with, merge,
};
use crate::by_distance::closure;
use crate::halt::Halt;
use crate::implicit::BERNSTEIN_ROUNDING;
use crate::intersect_curve::points;
use crate::march::{Net, PATCH_SLACK, Vector, magnitude, sum};
use crate::nurbs::BezierSpan;
use crate::{
    Curve, CurveSurfaceHit, CurveSurfaceIntersection, GeomError, NurbsCurve, NurbsSurface,
    SurfaceEval,
};

/// How many boxes of one front of zeros that are not isolated are
/// descended from, spread along it: the line arm's count
/// (`crate::nurbs::line`). A front is a curve of zeros or the sliver of
/// near-zeros along a tangency, which a descent from anywhere along it
/// reaches the surface from.
const FRONT_STARTS: usize = 16;

/// Steps of the damped Gauss–Newton descent on `|C − S|²`. Its own step
/// converges quadratically near a zero, so the count is only reached by
/// a descent that crawls along a tangency.
const DESCENT_STEPS: usize = 64;

/// The first damping of a step that does not decrease the distance, as a
/// fraction of `JᵀJ`'s diagonal; each next one is [`DAMPING_GROWTH`]
/// times it. Ratios, scale-free: the line arm's.
pub(crate) const DAMPING: f64 = 1e-6;

/// How much each damping grows over the last.
pub(crate) const DAMPING_GROWTH: f64 = 16.0;

/// How many dampings are tried before a descent stops.
pub(crate) const DAMPINGS: usize = 12;

/// How many points strictly between two stops are measured for the
/// curve staying within the tube between them: the line arm's count.
const JOIN_SAMPLES: usize = 3;

/// How many stretches each span is cut into for the curve lying on the
/// surface: a structural count, as the conic arm's 64 points are. A span
/// that leaves the surface by the tolerance does so over a stretch of its
/// length, and a span is a polynomial of degree at most
/// [`crate::MAX_DEGREE`], which bends at most that many times.
const SPAN_SAMPLES: usize = 16;

/// How far into the patch, as a share of its knot span, the normal at a
/// pole is looked for: the pole's own derivatives are parallel, and the
/// normals [`POLE_SIDES`] ways round it, a nudge away, are summed. Their
/// error is the nudge times the surface's curvature, to first order, and
/// cancels round the pole to second.
const POLE_NUDGE: f64 = 1e-4;

/// How many ways round a pole its normal is looked at from.
const POLE_SIDES: usize = 4;

/// How many golden-section steps find the farthest point between two
/// crossings of one run: a structural count, `0.618³²` of the stretch.
const GOLDEN_STEPS: usize = 32;

/// One Bézier span of the curve as polynomials in its own `t`, its net
/// moved by `−origin`, as [`Net`] holds a patch.
struct Arc {
    span: [f64; 2],
    a: [PolyN<1>; 3],
    w: PolyN<1>,
    rational: bool,
    /// The largest `|Aₖ| + |origin|·w` over the net.
    reach: f64,
    weight: f64,
    degree: f64,
    hull: [[f64; 3]; 2],
}

impl Arc {
    fn of(span: &BezierSpan<3>, origin: Point3) -> Arc {
        let n = span.control.len().saturating_sub(1);
        let o = origin.coords;
        let coord = |k: usize| {
            PolyN::new(
                [n],
                span.control.iter().map(|(a, w)| a[k] - o[k] * w).collect(),
            )
        };
        let w0 = span.control.first().map_or(1.0, |c| c.1);
        let mut hull = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        let (mut reach, mut weight) = (0.0f64, 0.0f64);
        for (a, w) in &span.control {
            for k in 0..3 {
                let c = a[k] / w - o[k];
                hull[0][k] = hull[0][k].min(c);
                hull[1][k] = hull[1][k].max(c);
                reach = reach.max(a[k].abs() + o[k].abs() * w);
            }
            weight = weight.max(*w);
        }
        Arc {
            span: [span.lo, span.hi],
            a: [coord(0), coord(1), coord(2)],
            w: PolyN::new([n], span.control.iter().map(|c| c.1).collect()),
            rational: span.control.iter().any(|c| c.1 != w0),
            reach,
            weight,
            degree: n as f64,
            hull,
        }
    }

    /// A positive multiple of `C′`, `a·A′ − a′·A`, with its rounding as
    /// [`Net::tangent`] charges a patch's.
    fn tangent(&self, bin: &Binomials) -> ([PolyN<1>; 3], f64) {
        let ad = self.a.each_ref().map(|c| c.partial(0));
        let error = 2.0 * self.degree * self.reach;
        if !self.rational {
            return (ad, error);
        }
        let dw = self.w.partial(0);
        let poly = core::array::from_fn(|k| {
            sum(&[
                (1.0, self.w.mul(&ad[k], bin)),
                (-1.0, self.a[k].mul(&dw, bin)),
            ])
        });
        (poly, 2.0 * self.weight * error)
    }
}

/// `p(t)` as a polynomial in `(t, s, u)`.
fn in_t(p: &PolyN<1>) -> PolyN<3> {
    PolyN::tensor3(p, &PolyN::new([0, 0], vec![1.0]))
}

/// `p(s, u)` as a polynomial in `(t, s, u)`.
fn in_su(p: &PolyN<2>) -> PolyN<3> {
    PolyN::tensor3(&PolyN::new([0], vec![1.0]), p)
}

/// `Σₖ dₖ·vₖ`, and its floor from `d`'s size and rounding and `v`'s.
fn dot(d: &[PolyN<3>; 3], v: &[PolyN<3>; 3], bin: &Binomials) -> PolyN<3> {
    sum(&[0, 1, 2].map(|k| (1.0, d[k].mul(&v[k], bin))))
}

/// What one span × patch pair's isolations found, in its own `[0, 1]³`.
#[derive(Default)]
struct Found {
    crossings: Vec<Zero<3>>,
    extrema: Vec<Zero<3>>,
    boxes: Vec<Zero<3>>,
    /// The meeting's zeros are not isolated somewhere: a stretch of the
    /// curve on the surface, or a pole it goes through.
    continuum: bool,
}

/// Up to [`FRONT_STARTS`] boxes spread evenly along `front`.
fn spread(front: Vec<Zero<3>>) -> impl Iterator<Item = Zero<3>> {
    let n = front.len();
    let step = n.div_ceil(FRONT_STARTS).max(1);
    front.into_iter().step_by(step)
}

/// Both isolations of one pair: the meeting `D = 0`, and the critical
/// points of the distance in the tube.
fn isolate(
    arc: &Arc,
    tangent: &([PolyN<1>; 3], f64),
    net: &Net,
    sides: &[Vector; 2],
    tau: f64,
    bin: &Binomials,
    meter: &mut Meter,
) -> Result<Found, GeomError> {
    let d: [PolyN<3>; 3] = core::array::from_fn(|k| {
        sum(&[
            (1.0, PolyN::tensor3(&arc.a[k], &net.w)),
            (-1.0, PolyN::tensor3(&arc.w, &net.x[k])),
        ])
    });
    let error = arc.reach * net.weight + arc.weight * net.reach;
    let floor = BERNSTEIN_ROUNDING * error;
    let mut found = Found::default();
    let take = |result: Result<Isolation<3>, Halt<Continuum<3>>>,
                certified: &mut Vec<Zero<3>>,
                boxes: &mut Vec<Zero<3>>,
                continuum: &mut bool|
     -> Result<(), GeomError> {
        let mut file = |z: Zero<3>| {
            if z.certified {
                certified.push(z);
            } else {
                boxes.push(z);
            }
        };
        match result {
            Ok(isolation) => isolation.zeros.into_iter().for_each(&mut file),
            Err(Halt::Fault(c)) => {
                *continuum = true;
                c.found.into_iter().for_each(&mut file);
                spread(c.front).for_each(&mut file);
            }
            Err(Halt::Stopped(stop)) => return Err(GeomError::Interrupted(stop)),
        }
        Ok(())
    };
    let meeting = common_zeros(d.each_ref(), [floor; 3], meter);
    take(
        meeting,
        &mut found.crossings,
        &mut found.boxes,
        &mut found.continuum,
    )?;

    // The critical points of the distance, in the tube.
    let size = magnitude(&d);
    let (ct, ct_error) = tangent;
    let ct3 = ct.each_ref().map(in_t);
    let ct_size = magnitude(&ct3);
    let along: [([PolyN<3>; 3], f64, f64); 3] = [
        (ct3, ct_size, *ct_error),
        (
            sides[0].poly.each_ref().map(in_su),
            sides[0].size,
            sides[0].error,
        ),
        (
            sides[1].poly.each_ref().map(in_su),
            sides[1].size,
            sides[1].error,
        ),
    ];
    let g: [PolyN<3>; 3] = along.each_ref().map(|(v, _, _)| dot(&d, v, bin));
    let g_floors = along
        .each_ref()
        .map(|(_, v_size, v_error)| 3.0 * BERNSTEIN_ROUNDING * (size * v_error + error * v_size));
    let aw = PolyN::tensor3(&arc.w, &net.w);
    let tube = sum(&[
        (1.0, d[0].mul(&d[0], bin)),
        (1.0, d[1].mul(&d[1], bin)),
        (1.0, d[2].mul(&d[2], bin)),
        (-tau * tau, aw.mul(&aw, bin)),
    ]);
    let aw_size = arc.weight * net.weight;
    let tube_floor =
        BERNSTEIN_ROUNDING * (6.0 * size * error + 2.0 * tau * tau * aw_size * aw_size);
    let gate = Gate {
        poly: &tube,
        floor: tube_floor,
    };
    let critical = common_zeros_with(g.each_ref(), g_floors, &[gate], meter);
    let mut ignored = false;
    take(critical, &mut found.extrema, &mut found.boxes, &mut ignored)?;
    Ok(found)
}

/// A zero of a pair's isolation in the curve's and the surface's
/// parameters `(t, u, v)`, if it is the pair's own: `crate::march`'s
/// rule in three unknowns, a certified box left whole.
fn own(z: Zero<3>, spans: [[f64; 2]; 3]) -> Option<Zero<3>> {
    let inside = |x: f64| (-PATCH_SLACK..=1.0 + PATCH_SLACK).contains(&x);
    let keep = if z.certified {
        z.at.iter().all(|&x| inside(x))
    } else {
        (0..3).all(|k| z.lo[k] <= 1.0 + PATCH_SLACK && -PATCH_SLACK <= z.hi[k])
    };
    if !keep {
        return None;
    }
    let to = |k: usize, x: f64| spans[k][0] + (spans[k][1] - spans[k][0]) * x;
    // A certified box keeps its margin: the same zero found from the
    // pair across a knot line is in it, and the two are merged as one.
    let cut = |x: f64| {
        if z.certified {
            x
        } else {
            x.clamp(-PATCH_SLACK, 1.0 + PATCH_SLACK)
        }
    };
    Some(Zero {
        at: core::array::from_fn(|k| to(k, z.at[k])),
        lo: core::array::from_fn(|k| to(k, cut(z.lo[k]))),
        hi: core::array::from_fn(|k| to(k, cut(z.hi[k]))),
        certified: z.certified,
    })
}

/// What a stop is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mark {
    Crossing,
    Extremum,
    End,
    Pole,
}

/// A parameter of the curve where the distance is looked at.
#[derive(Clone, Copy, Debug)]
struct Stop {
    t: f64,
    uv: [f64; 2],
    distance: f64,
    mark: Mark,
}

/// The middle of the box of both nets' control points: the origin they
/// are moved to, as `crate::march` moves two surfaces'.
fn middle(spline: &NurbsCurve, surface: &NurbsSurface) -> Point3 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in spline
        .control_points()
        .iter()
        .chain(surface.control_points())
    {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    Point3::new(
        0.5 * (lo[0] + hi[0]),
        0.5 * (lo[1] + hi[1]),
        0.5 * (lo[2] + hi[2]),
    )
}

/// The hulls of a span and a patch meet within `slack`.
fn hulls_meet(a: &[[f64; 3]; 2], b: &[[f64; 3]; 2], slack: f64) -> bool {
    (0..3).all(|k| a[0][k] <= b[1][k] + slack && b[0][k] <= a[1][k] + slack)
}

/// A NURBS curve against a NURBS surface (module docs).
pub(crate) fn spline_nurbs(
    curve: &Curve,
    spline: &NurbsCurve,
    surface: &NurbsSurface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let tau = tol.linear;
    let origin = middle(spline, surface);
    let arcs: Vec<Arc> = spline
        .bezier_spans()
        .iter()
        .map(|s| Arc::of(s, origin))
        .collect();
    let nets: Vec<Net> = surface
        .patches()
        .iter()
        .map(|p| Net::of(p, origin))
        .collect();
    let top = arcs
        .iter()
        .map(|a| a.degree as usize)
        .chain(nets.iter().map(|n| n.degree as usize))
        .max()
        .unwrap_or(1);
    let bin = Binomials::new(8 * top + 8);
    let tangents: Vec<_> = arcs.iter().map(|a| a.tangent(&bin)).collect();
    let sides: Vec<[Vector; 2]> = nets
        .iter()
        .map(|n| [n.tangent(0, &bin), n.tangent(1, &bin)])
        .collect();

    let (mut crossings, mut extrema, mut boxes) = (Vec::new(), Vec::new(), Vec::new());
    let mut continuum = false;
    for (arc, tangent) in arcs.iter().zip(&tangents) {
        for (net, sides) in nets.iter().zip(&sides) {
            meter.tick()?;
            let slack = tau + BERNSTEIN_ROUNDING * (arc.reach + net.reach);
            if !hulls_meet(&arc.hull, &net.hull, slack) {
                continue;
            }
            let found = isolate(arc, tangent, net, sides, tau, &bin, meter)?;
            let spans = [arc.span, net.span[0], net.span[1]];
            let mine = |zs: Vec<Zero<3>>| zs.into_iter().filter_map(move |z| own(z, spans));
            crossings.extend(mine(found.crossings));
            extrema.extend(mine(found.extrema));
            boxes.extend(mine(found.boxes));
            continuum |= found.continuum;
        }
    }
    let period = closure(curve);
    let domain = spline.domain();
    let [du, dv] = surface.domain();
    let width = [domain, du, dv]
        .iter()
        .map(|d| d.length())
        .fold(0.0, f64::max);
    let [cu, cv] = surface.closure();
    let wrap = Wrap {
        period: [period, cu, cv],
        origin: [domain.lo(), du.lo(), dv.lo()],
        touch: PATCH_SLACK * width,
    };
    let reader = Reader {
        curve,
        surface,
        tau,
        period,
    };

    let mut stops: Vec<Stop> = Vec::new();
    // The boxes go with the crossings, so that one inside a crossing's
    // certified box is that crossing.
    crossings.extend(boxes);
    for z in merge(crossings, &wrap) {
        if z.certified {
            stops.push(reader.stop(z.at, Mark::Crossing));
            continue;
        }
        meter.tick()?;
        let (t, uv) = reader.descend(z.at, meter)?;
        let mut stop = reader.stop([t, uv[0], uv[1]], Mark::Extremum);
        if stop.distance > tau {
            continue;
        }
        if !reader.touches(stop.t, stop.uv) {
            stop.mark = Mark::Crossing;
        }
        stops.push(stop);
    }
    for z in merge(extrema, &wrap) {
        let stop = reader.stop(z.at, Mark::Extremum);
        if stop.distance <= tau && reader.touches(stop.t, stop.uv) {
            stops.push(stop);
        }
    }
    for (k, value, pole) in surface.collapsed_rows() {
        let t = spline.project_parameter(pole);
        let distance = (curve.point(t) - pole).norm();
        if distance <= tau {
            let uv = if k == 0 {
                [value, dv.lo()]
            } else {
                [du.lo(), value]
            };
            stops.push(Stop {
                t,
                uv,
                distance,
                mark: Mark::Pole,
            });
        }
    }
    if period.is_none() {
        for t in [domain.lo(), domain.hi()] {
            let p = curve.point(t);
            if let Ok(on) = surface.project(p)
                && on.distance <= tau
            {
                stops.push(Stop {
                    t,
                    uv: [on.uv.x, on.uv.y],
                    distance: on.distance,
                    mark: Mark::End,
                });
            }
        }
    }
    if (continuum || stops.iter().any(|s| s.mark != Mark::Crossing))
        && reader.on_surface(spline, meter)?
    {
        return Ok(CurveSurfaceIntersection::Coincident);
    }
    reader.hits(stops, meter)
}

/// What the stops are read with.
struct Reader<'a> {
    curve: &'a Curve,
    surface: &'a NurbsSurface,
    tau: f64,
    /// The length the curve repeats over, periodic or closed.
    period: Option<f64>,
}

impl Reader<'_> {
    /// `t` in the curve's domain: wrapped where the curve repeats,
    /// clamped where it does not.
    fn on_curve(&self, t: f64) -> f64 {
        let d = self.curve.domain();
        match self.period {
            Some(period) => {
                let w = d.lo() + (t - d.lo()).rem_euclid(period);
                if w >= d.hi() { d.lo() } else { w }
            }
            None => t.clamp(d.lo(), d.hi()),
        }
    }

    /// `uv` in the surface's domain: wrapped along a direction it closes
    /// in, clamped along one it does not.
    fn on_surface_uv(&self, uv: [f64; 2]) -> [f64; 2] {
        let domain = self.surface.domain();
        let closure = self.surface.closure();
        core::array::from_fn(|k| {
            let d = domain[k];
            match closure[k] {
                Some(period) => {
                    let w = d.lo() + (uv[k] - d.lo()).rem_euclid(period);
                    if w >= d.hi() { d.lo() } else { w }
                }
                None => uv[k].clamp(d.lo(), d.hi()),
            }
        })
    }

    fn stop(&self, at: [f64; 3], mark: Mark) -> Stop {
        let t = self.on_curve(at[0]);
        let uv = self.on_surface_uv([at[1], at[2]]);
        let p = self.surface.eval(uv[0], uv[1]).point;
        Stop {
            t,
            uv,
            distance: (self.curve.point(t) - p).norm(),
            mark,
        }
    }

    /// How far the curve's point at `t` is from the surface, from the
    /// surface's point at `start`.
    fn gap(&self, t: f64, start: [f64; 2]) -> f64 {
        self.surface
            .project_from(self.curve.point(t), Point2::new(start[0], start[1]))
            .distance
    }

    /// Damped Gauss–Newton on `|C(t) − S(u, v)|²` from `at`, the curve's
    /// parameter held in its domain and the surface's in its own.
    fn descend(&self, at: [f64; 3], meter: &mut Meter) -> Result<(f64, [f64; 2]), GeomError> {
        let mut x = [self.on_curve(at[0]), at[1], at[2]];
        let uv = self.on_surface_uv([x[1], x[2]]);
        x[1] = uv[0];
        x[2] = uv[1];
        let residual = |x: [f64; 3]| -> (Vec3, CurveEvalPair) {
            let c = self.curve.eval(x[0]);
            let s = self.surface.eval(x[1], x[2]);
            (c.point - s.point, (c.d1, s))
        };
        let (mut r, mut e) = residual(x);
        for _ in 0..DESCENT_STEPS {
            meter.tick()?;
            let j = Matrix3::from_columns(&[e.0, -e.1.du, -e.1.dv]);
            let jtj = j.transpose() * j;
            let g = j.transpose() * r;
            // Marquardt's scaling: each unknown damped by its own
            // diagonal, the curve's parameter and the surface's being of
            // unrelated speeds. The undamped Gauss–Newton step first.
            let diagonal = Matrix3::from_diagonal(&jtj.diagonal());
            // A NaN trace stops it as a zero one does.
            if diagonal.trace().partial_cmp(&0.0) != Some(core::cmp::Ordering::Greater) {
                break;
            }
            let mut damping = 0.0;
            let mut moved = false;
            for _ in 0..=DAMPINGS {
                let tried = jtj + diagonal * damping;
                damping = if damping == 0.0 {
                    DAMPING
                } else {
                    damping * DAMPING_GROWTH
                };
                let Some(inverse) = tried.try_inverse() else {
                    continue;
                };
                let step = inverse * g;
                let mut next = [x[0] - step[0], x[1] - step[1], x[2] - step[2]];
                next[0] = self.on_curve(next[0]);
                let uv = self.on_surface_uv([next[1], next[2]]);
                next[1] = uv[0];
                next[2] = uv[1];
                let (rn, en) = residual(next);
                if rn.norm() < r.norm() {
                    (x, r, e) = (next, rn, en);
                    moved = true;
                    break;
                }
            }
            if !moved {
                break;
            }
        }
        Ok((x[0], [x[1], x[2]]))
    }

    /// The surface's derivatives at `uv` and its normal there, the
    /// normal of a pole summed from [`POLE_SIDES`] points a
    /// [`POLE_NUDGE`] into the net round it, the derivatives the first
    /// of them; `None` where no normal is found.
    fn frame(&self, uv: [f64; 2]) -> Option<(SurfaceEval, Vec3)> {
        if let Some(n) = self.surface.normal(uv[0], uv[1]) {
            return Some((self.surface.eval(uv[0], uv[1]), n.into_inner()));
        }
        let domain = self.surface.domain();
        // The row the pole is: the direction whose derivative vanishes
        // runs along it, and the other is nudged in from its end.
        let e = self.surface.eval(uv[0], uv[1]);
        let (round, nudge) = if e.du.norm() <= e.dv.norm() {
            (0, 1)
        } else {
            (1, 0)
        };
        let d = domain[nudge];
        let into = if uv[nudge] - d.lo() <= d.hi() - uv[nudge] {
            1.0
        } else {
            -1.0
        };
        let mut sum = Vec3::zeros();
        let mut first = None;
        for k in 0..POLE_SIDES {
            let mut at = uv;
            at[nudge] = (uv[nudge] + into * POLE_NUDGE * d.length()).clamp(d.lo(), d.hi());
            let r = domain[round];
            at[round] = r.lo() + (k as f64 + 0.5) / POLE_SIDES as f64 * r.length();
            if let Some(n) = self.surface.normal(at[0], at[1]) {
                sum += n.into_inner();
                first.get_or_insert(at);
            }
        }
        let at = first?;
        let n = sum.try_normalize(0.0)?;
        Some((self.surface.eval(at[0], at[1]), n))
    }

    /// The second-order model at the curve's `t` and the surface's `uv`
    /// (module docs): the extremum beside the point within `τ`.
    fn touches(&self, t: f64, uv: [f64; 2]) -> bool {
        let Some((s, n)) = self.frame(uv) else {
            return true;
        };
        let c = self.curve.eval(t);
        let speed = c.d1.norm();
        if speed == 0.0 {
            return true;
        }
        let along = c.d1 / speed;
        let sin = along.dot(&n);
        // The curve's bend towards the normal, per length squared.
        let bend = (c.d2 - c.d2.dot(&along) * along).dot(&n) / (speed * speed);
        // The surface's along the curve's direction in its tangent plane.
        let flat = along - sin * n;
        let [a, b] = [s.du, s.dv];
        let (aa, ab, bb) = (a.dot(&a), a.dot(&b), b.dot(&b));
        let det = aa * bb - ab * ab;
        let sag = if det > 0.0 {
            let (fa, fb) = (flat.dot(&a), flat.dot(&b));
            let (x, y) = ((bb * fa - ab * fb) / det, (aa * fb - ab * fa) / det);
            (x * x * s.duu + 2.0 * x * y * s.duv + y * y * s.dvv).dot(&n)
        } else {
            0.0
        };
        let k = bend - sag;
        sin * sin <= 2.0 * k.abs() * self.tau
    }

    /// The curve stays within the tube from `a` to `b` (`b.t` a period
    /// on where the stretch goes round).
    fn joined(&self, a: &Stop, b_t: f64, b_uv: [f64; 2]) -> bool {
        (1..=JOIN_SAMPLES).all(|i| {
            let f = i as f64 / (JOIN_SAMPLES + 1) as f64;
            let t = a.t + f * (b_t - a.t);
            let start = if f <= 0.5 { a.uv } else { b_uv };
            self.gap(self.on_curve(t), start) <= self.tau
        })
    }

    /// The farthest point of the curve from the surface between two
    /// crossings of one run, by golden section.
    fn farthest(&self, a: &Stop, b_t: f64) -> f64 {
        // Not `roots::golden_section`: this search re-evaluates both probes each step, whose bits differ from the shared form's reused probe.
        let phi = 0.5 * (5.0f64.sqrt() - 1.0);
        let gap = |t: f64| self.gap(self.on_curve(t), a.uv);
        let (mut lo, mut hi) = (a.t, b_t);
        for _ in 0..GOLDEN_STEPS {
            let (x, y) = (hi - phi * (hi - lo), lo + phi * (hi - lo));
            if gap(x) >= gap(y) {
                hi = y;
            } else {
                lo = x;
            }
        }
        self.on_curve(0.5 * (lo + hi))
    }

    /// The curve within `τ` of the surface at [`SPAN_SAMPLES`] points of
    /// every span.
    fn on_surface(&self, spline: &NurbsCurve, meter: &mut Meter) -> Result<bool, GeomError> {
        let mut start: Option<[f64; 2]> = None;
        for span in spline.bezier_spans() {
            for i in 0..=SPAN_SAMPLES {
                meter.tick()?;
                let t = span.lo + (span.hi - span.lo) * i as f64 / SPAN_SAMPLES as f64;
                let p = self.curve.point(t);
                // A point with two nearest points on the surface (on its
                // medial axis) is on it only at a pole; the curve is then
                // not decided to lie on it, which only costs the
                // `Coincident` verdict.
                let global = |p| match self.surface.project(p) {
                    Ok(on) => Ok(Some(on)),
                    Err(GeomError::Ambiguous { .. }) => Ok(None),
                    Err(e) => Err(e),
                };
                let on = match start {
                    Some(uv) => Some(self.surface.project_from(p, Point2::new(uv[0], uv[1]))),
                    None => global(p)?,
                };
                let Some(mut on) = on else {
                    return Ok(false);
                };
                if on.distance > self.tau {
                    // A local projection can stall on the wrong side of
                    // a fold: the global one has the last word.
                    let Some(whole) = global(p)? else {
                        return Ok(false);
                    };
                    on = whole;
                    if on.distance > self.tau {
                        return Ok(false);
                    }
                }
                start = Some([on.uv.x, on.uv.y]);
            }
        }
        Ok(true)
    }

    /// The stops read along the curve as runs, one hit each (module
    /// docs).
    fn hits(
        &self,
        mut stops: Vec<Stop>,
        meter: &mut Meter,
    ) -> Result<CurveSurfaceIntersection, GeomError> {
        stops.sort_by(|a, b| a.t.total_cmp(&b.t).then(a.distance.total_cmp(&b.distance)));
        let mut runs: Vec<Vec<Stop>> = Vec::new();
        for stop in stops {
            meter.tick()?;
            match runs.last_mut() {
                Some(run)
                    if run
                        .last()
                        .is_some_and(|last| self.joined(last, stop.t, stop.uv)) =>
                {
                    run.push(stop);
                }
                _ => runs.push(vec![stop]),
            }
        }
        if let Some(period) = self.period
            && runs.len() > 1
        {
            let first = runs[0][0];
            let joined = runs
                .last()
                .and_then(|run| run.last())
                .is_some_and(|last| self.joined(last, first.t + period, first.uv));
            if joined {
                let head = runs.remove(0);
                if let Some(last) = runs.last_mut() {
                    last.extend(head);
                }
            }
        }
        let mut hits = Vec::with_capacity(runs.len());
        for run in &runs {
            hits.push(self.hit(run)?);
        }
        Ok(points(hits))
    }

    /// The one hit of a run.
    fn hit(&self, run: &[Stop]) -> Result<CurveSurfaceHit, GeomError> {
        let nearest = |mark: Mark| {
            run.iter()
                .filter(|s| s.mark == mark)
                .min_by(|a, b| a.distance.total_cmp(&b.distance))
                .copied()
        };
        let (stop, tangent) = if let Some(pole) = nearest(Mark::Pole) {
            (pole, self.touches(pole.t, pole.uv))
        } else if run.iter().all(|s| s.mark == Mark::Crossing)
            && (self.curve.point(run[0].t) - self.curve.point(run[run.len() - 1].t)).norm()
                <= self.tau
        {
            // One crossing, found more than once.
            (run[0], false)
        } else if let Some(extremum) = nearest(Mark::Extremum) {
            (extremum, true)
        } else if let Some(end) = nearest(Mark::End) {
            (end, false)
        } else {
            // Crossings alone, the curve within the tube between them.
            let (first, last) = (run[0], run[run.len() - 1]);
            let mut last_t = last.t;
            if last_t < first.t
                && let Some(period) = self.period
            {
                last_t += period;
            }
            let t = self.farthest(&first, last_t);
            (
                Stop {
                    t,
                    uv: first.uv,
                    distance: 0.0,
                    mark: Mark::Extremum,
                },
                true,
            )
        };
        let point = self.curve.point(stop.t);
        let uv = if stop.mark == Mark::Pole {
            stop.uv
        } else {
            let on = self
                .surface
                .project_from(point, Point2::new(stop.uv[0], stop.uv[1]));
            self.on_surface_uv([on.uv.x, on.uv.y])
        };
        Ok(CurveSurfaceHit {
            t: stop.t,
            uv: Point2::new(uv[0], uv[1]),
            point,
            tangent,
        })
    }
}

/// The curve's tangent and the surface's derivatives at one iterate.
type CurveEvalPair = (Vec3, SurfaceEval);
