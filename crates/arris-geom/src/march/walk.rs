//! The walk of a marched section (ADR-0068): its seeds, its singular
//! points, and the walk from seed to seed in both surfaces' `(u, v)` at
//! once, before any fit.
//!
//! **Seeds.** A branch that reaches an edge of either domain, or crosses
//! a seam, does so where that line meets the other surface: each edge
//! line and each seam is an exact isocurve, and `crate::spline_nurbs`
//! gives its hits, the **boundary seeds**. A seed on an edge ends a
//! branch; one on a seam is passed. A row collapsed to a pole is no line
//! and is not looked at (step 6 decides poles). The loop condition gives
//! the **turning points** in `v₁` (`super::loop_seeds`), passed as seams
//! are, and its uncertified boxes, where a **singular point** may be.
//!
//! **Singular points.** From each uncertified box a Gauss–Newton descent
//! looks for the point where the surfaces meet and their normals are
//! parallel. One within `tol.linear` of both and `tol.angular` of
//! tangent is a singular point; one the descent does not find is looked
//! for as a turning point the isolation could not certify, and a box
//! that is neither is refused, [`SectionFault::CrowdedSingularity`].
//! Around a singular point the section is not followed: its **reach** is
//! one step of the walk at the surfaces' curvature there, and the points
//! of the section at that distance from it, the zeros of the section
//! with `|P₁ − S|² = ρ²` over the patch pairs near it, are its
//! **departures**, certified by `crate::bernstein_n`. A branch ends at a
//! singular point through one of them, the stretch inside the reach the
//! chord to the point, which is within the walk's sag of the section. A
//! singular point with no departures is isolated: the surfaces touch
//! there, or meet in a loop smaller than its reach. Turning points inside
//! a reach are the singular point's and are dropped; a boundary seed
//! inside one, or two reaches that meet, are refused as crowded.
//!
//! **The walk.** A step predicts along the section's tangent `n₁ × n₂`,
//! carried into each surface's `(u, v)` by its first fundamental form,
//! and corrects onto both surfaces by Newton in four unknowns, the fourth
//! equation the plane square to the tangent through the predicted point.
//! Its length is the chord whose sag at the section's curvature is
//! [`SECTION_FIT_FRACTION`] of `tol.linear`; the curvature is exact at
//! the step's start, from the two surfaces' second fundamental forms
//! along the tangent. A step is accepted only where the corrector
//! converges, the corrected point is near the predicted one, the tangent
//! turns by no more than the curvature at either end allows, and the
//! chord's middle corrected onto the section is within the sag of it:
//! a step that jumps to another branch fails one of those and is halved.
//! The **box chain** the step must stay in is the seeds': a seed within
//! the sag of a step's chord is where the step ends, exactly at the
//! seed, and an end seed ends the branch there. Every seed is visited
//! once or the section is refused, [`SectionFault::UnvisitedSeed`]: the
//! walk is not certified, but it cannot skip a branch's seed without
//! being caught.
//!
//! The walks start from every end seed not yet reached, edges first, into
//! the domain or away from the singular point, then from every passed
//! seed not yet reached, which is on a loop and comes back to itself.
//! The walked points are in `(u₁, v₁, u₂, v₂)`, unwrapped across the
//! seams so a branch is continuous; a loop that winds round a closed
//! direction comes back a whole number of closures from where it started.

use arris_math::nalgebra::{Matrix2, Matrix4, Vector2, Vector4};
use arris_math::{Interval, Meter, Point3, Tolerance, Vec3, real};

use super::{Net, PATCH_SLACK, hulls_meet, loop_seeds, middle, own, section, sum};
use crate::bernstein::Binomials;
use crate::bernstein_n::{PolyN, Wrap, Zero, common_zeros, merge};
use crate::halt::Halt;
use crate::implicit::BERNSTEIN_ROUNDING;
use crate::spline_nurbs::spline_nurbs;
use crate::{
    BranchEnd, Curve, CurveSurfaceIntersection, GeomError, GeomKind, NurbsSurface,
    SECTION_FIT_FRACTION, SectionFault, SurfaceEval, SurfaceKind,
};

/// How far apart a walked point's two surface points may be, as a share
/// of `tol.linear`: the corrector's Newton converges quadratically, so
/// it is met a step after rounding is, and the fit's own share is left
/// whole.
const ON_SHARE: f64 = 1.0 / 1024.0;

/// Newton steps of the corrector before it is given up on and the step
/// halved.
const CORRECTOR_STEPS: usize = 16;

/// The longest step, as a share of the diagonal of the box of both
/// nets: where the section is straight the curvature sets no length.
const STEP_SHARE: f64 = 1.0 / 64.0;

/// How far the corrected point may be from the predicted one, as a share
/// of the step: the prediction is off by `κh²/2`, a small share of `h`
/// at the sag's step, and a corrector that moves farther went to another
/// branch.
const STEP_DRIFT: f64 = 0.25;

/// How much more the tangent may turn over a step, and the chord's
/// middle sag, than the curvature at its ends predicts: a ratio.
const CURVATURE_SLACK: f64 = 4.0;

/// How many times a step is halved before the walk is given up.
const STEP_HALVINGS: usize = 40;

/// The most steps one walk takes before it is given up: a structural
/// count far past any section the sag's step makes, which a walk that
/// never comes back to a seed reaches.
const WALK_STEPS: usize = 1 << 20;

/// Gauss–Newton steps of the descent onto a singular point.
const SINGULAR_STEPS: usize = 64;

/// The first damping of a descent step that does not decrease the
/// residual, as a fraction of `JᵀJ`'s diagonal, and how much each next
/// one grows: `crate::spline_nurbs`'s ratios.
const DAMPING: f64 = 1e-6;
const DAMPING_GROWTH: f64 = 16.0;
const DAMPINGS: usize = 12;

/// A walked section of two NURBS surfaces: its branches and its singular
/// points, every point in both surfaces' parameters.
#[derive(Clone, Debug, Default)]
pub(crate) struct MarchedSection {
    pub(crate) branches: Vec<MarchedBranch>,
    pub(crate) points: Vec<MarchedPoint>,
}

/// One branch: the walked points in `(u₁, v₁, u₂, v₂)`, continuous across
/// the seams. A closed branch does not repeat its first point.
#[derive(Clone, Debug)]
pub(crate) struct MarchedBranch {
    pub(crate) at: Vec<[f64; 4]>,
    /// How it ends at its first point and its last; `None` for a loop.
    pub(crate) ends: Option<[BranchEnd; 2]>,
}

/// A singular point of the section, where the surfaces are tangent
/// within the tolerance.
#[derive(Clone, Debug)]
pub(crate) struct MarchedPoint {
    pub(crate) at: [f64; 4],
    pub(crate) point: Point3,
    /// No branch ends here.
    pub(crate) isolated: bool,
}

/// A point the walk has to reach.
#[derive(Clone, Debug)]
struct Seed {
    at: [f64; 4],
    point: Point3,
    /// The edges of the domains it is on, `[surface, direction, end]`,
    /// `end` 0 at the low one: a branch ends here.
    edges: Vec<[usize; 3]>,
    /// The singular point it is a departure from: a branch ends there.
    departure: Option<usize>,
}

impl Seed {
    fn ends(&self) -> bool {
        !self.edges.is_empty() || self.departure.is_some()
    }
}

/// A singular point and its reach.
struct Singular {
    at: [f64; 4],
    point: Point3,
    reach: f64,
}

/// What a walk ended at.
enum End {
    /// An end seed.
    Seed(usize),
    /// The seed it started from, a loop.
    Closed,
}

fn refused(fault: SectionFault) -> GeomError {
    GeomError::DegenerateSection {
        a: GeomKind::Surface(SurfaceKind::Nurbs),
        b: GeomKind::Surface(SurfaceKind::Nurbs),
        fault,
    }
}

fn stopped(halt: Halt<impl Sized>) -> GeomError {
    match halt {
        Halt::Stopped(stop) => GeomError::Interrupted(stop),
        Halt::Fault(_) => refused(SectionFault::CrowdedSingularity),
    }
}

/// The section of `one` with `two`, walked between its seeds (module
/// docs). An isoline of either on the other is refused here as a tube
/// circle until step 6 of `plans/nurbs-nurbs-section` decides it.
pub(crate) fn march(
    one: &NurbsSurface,
    two: &NurbsSurface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<MarchedSection, GeomError> {
    let marcher = Marcher::new(one, two, tol);
    let mut seeds = marcher.boundary_seeds(meter)?;
    let found = loop_seeds(one, two, meter)?;
    if !found.continua.is_empty() {
        return Err(refused(SectionFault::TubeCircle));
    }
    let mut singular: Vec<Singular> = Vec::new();
    let mut passed: Vec<[f64; 4]> = Vec::new();
    for z in found.seeds {
        if z.certified {
            passed.push(z.at);
            continue;
        }
        meter.tick()?;
        if let Some(at) = marcher.singular(z.at, meter)? {
            let point = marcher.surfaces[0].eval(at[0], at[1]).point;
            if singular
                .iter()
                .all(|s| (s.point - point).norm() > tol.linear)
            {
                let reach = marcher.reach(at);
                singular.push(Singular { at, point, reach });
            }
        } else if let Some(at) = marcher.turning(z.at, meter)? {
            passed.push(at);
        } else {
            return Err(refused(SectionFault::CrowdedSingularity));
        }
    }
    singular.sort_by(|a, b| order(&a.at, &b.at));
    for (i, a) in singular.iter().enumerate() {
        if singular[i + 1..]
            .iter()
            .any(|b| (a.point - b.point).norm() <= a.reach + b.reach)
        {
            return Err(refused(SectionFault::CrowdedSingularity));
        }
    }
    let inside = |p: Point3| singular.iter().any(|s| (p - s.point).norm() < s.reach);
    if seeds.iter().any(|s| inside(s.point)) {
        return Err(refused(SectionFault::CrowdedSingularity));
    }
    for at in passed {
        let point = marcher.surfaces[0].eval(at[0], at[1]).point;
        if !inside(point) {
            add(
                &mut seeds,
                Seed {
                    at,
                    point,
                    edges: Vec::new(),
                    departure: None,
                },
                tol,
            );
        }
    }
    let mut isolated = Vec::with_capacity(singular.len());
    for (k, s) in singular.iter().enumerate() {
        let departures = marcher.departures(s, meter)?;
        isolated.push(departures.is_empty());
        for at in departures {
            let point = marcher.surfaces[0].eval(at[0], at[1]).point;
            seeds.push(Seed {
                at,
                point,
                edges: Vec::new(),
                departure: Some(k),
            });
        }
    }
    // Edges first, then departures, then the seeds a walk passes.
    let rank = |s: &Seed| match (s.edges.is_empty(), s.departure) {
        (false, _) => 0,
        (true, Some(_)) => 1,
        (true, None) => 2,
    };
    seeds.sort_by(|a, b| rank(a).cmp(&rank(b)).then(order(&a.at, &b.at)));

    let mut visited = vec![false; seeds.len()];
    let mut branches = Vec::new();
    for i in 0..seeds.len() {
        if visited[i] {
            continue;
        }
        visited[i] = true;
        let sign = marcher.leaving(&seeds[i], &singular)?;
        let (mut at, end) = marcher.walk(i, sign, &seeds, &singular, &mut visited, meter)?;
        let start = match seeds[i].departure {
            Some(k) => {
                at.insert(0, near(singular[k].at, at[0], &marcher.closure));
                Some(BranchEnd::Singular(k))
            }
            None if seeds[i].ends() => Some(BranchEnd::Clipped),
            None => None,
        };
        let ends = match (start, end) {
            (None, End::Closed) => None,
            (Some(first), End::Seed(j)) => {
                let last = match seeds[j].departure {
                    Some(k) => {
                        let tail = at.last().copied().unwrap_or(seeds[j].at);
                        at.push(near(singular[k].at, tail, &marcher.closure));
                        BranchEnd::Singular(k)
                    }
                    None => BranchEnd::Clipped,
                };
                Some([first, last])
            }
            // A loop's walk met an end, or an end's walk came back to
            // its own start.
            _ => return Err(refused(SectionFault::UnvisitedSeed)),
        };
        branches.push(MarchedBranch { at, ends });
    }
    let points = singular
        .iter()
        .zip(isolated)
        .map(|(s, isolated)| MarchedPoint {
            at: s.at,
            point: s.point,
            isolated,
        })
        .collect();
    Ok(MarchedSection { branches, points })
}

/// Lexicographic order of two parameter points.
fn order(a: &[f64; 4], b: &[f64; 4]) -> core::cmp::Ordering {
    (0..4)
        .map(|k| a[k].total_cmp(&b[k]))
        .fold(core::cmp::Ordering::Equal, core::cmp::Ordering::then)
}

/// `seed` added to `seeds`, or its edges to the one already within the
/// tolerance of it.
fn add(seeds: &mut Vec<Seed>, seed: Seed, tol: Tolerance) {
    match seeds
        .iter_mut()
        .find(|s| (s.point - seed.point).norm() <= tol.linear)
    {
        Some(same) => {
            for edge in seed.edges {
                if !same.edges.contains(&edge) {
                    same.edges.push(edge);
                }
            }
        }
        None => seeds.push(seed),
    }
}

/// `at` moved by whole closures to be nearest `to`.
fn near(mut at: [f64; 4], to: [f64; 4], closure: &[Option<f64>; 4]) -> [f64; 4] {
    for k in 0..4 {
        if let Some(c) = closure[k] {
            at[k] += c * ((to[k] - at[k]) / c).round();
        }
    }
    at
}

/// The section's tangent at a point of both surfaces, carried into the
/// parameters, and its curvature vector.
struct Tangent {
    along: Vec3,
    velocity: [f64; 4],
    curvature: Vec3,
}

struct Marcher<'a> {
    surfaces: [&'a NurbsSurface; 2],
    tol: Tolerance,
    /// The sag a step's chord is allowed.
    sag: f64,
    step_max: f64,
    domain: [Interval; 4],
    closure: [Option<f64>; 4],
}

impl<'a> Marcher<'a> {
    fn new(one: &'a NurbsSurface, two: &'a NurbsSurface, tol: Tolerance) -> Marcher<'a> {
        let [d1, d2] = [one.domain(), two.domain()];
        let [c1, c2] = [one.closure(), two.closure()];
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for p in one.control_points().iter().chain(two.control_points()) {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let diagonal = (0..3)
            .map(|k| (hi[k] - lo[k]) * (hi[k] - lo[k]))
            .sum::<f64>()
            .sqrt();
        Marcher {
            surfaces: [one, two],
            tol,
            sag: SECTION_FIT_FRACTION * tol.linear,
            step_max: STEP_SHARE * diagonal,
            domain: [d1[0], d1[1], d2[0], d2[1]],
            closure: [c1[0], c1[1], c2[0], c2[1]],
        }
    }

    fn eval(&self, x: [f64; 4]) -> [SurfaceEval; 2] {
        [
            self.surfaces[0].eval(x[0], x[1]),
            self.surfaces[1].eval(x[2], x[3]),
        ]
    }

    /// Every edge line and seam of both domains against the other
    /// surface.
    fn boundary_seeds(&self, meter: &mut Meter) -> Result<Vec<Seed>, GeomError> {
        let mut seeds: Vec<Seed> = Vec::new();
        for s in 0..2 {
            let (surface, other) = (self.surfaces[s], self.surfaces[1 - s]);
            let poles = surface.collapsed_rows();
            for dir in 0..2 {
                let axis = 2 * s + dir;
                let d = self.domain[axis];
                let closed = self.closure[axis].is_some();
                let ends: &[(usize, f64)] = if closed {
                    &[(0, d.lo())]
                } else {
                    &[(0, d.lo()), (1, d.hi())]
                };
                for &(end, value) in ends {
                    if poles.iter().any(|&(k, at, _)| k == dir && at == value) {
                        continue;
                    }
                    let line = surface.isocurve(dir, value)?;
                    let curve = Curve::Nurbs(line.clone());
                    let hits = match spline_nurbs(&curve, &line, other, self.tol, meter)? {
                        CurveSurfaceIntersection::Points(hits) => hits,
                        CurveSurfaceIntersection::Coincident => {
                            return Err(refused(SectionFault::TubeCircle));
                        }
                    };
                    for hit in hits {
                        // The section touching an edge, entering the
                        // domain there or not, is ADR-0061's refusal.
                        if hit.tangent && !closed {
                            return Err(refused(SectionFault::UnresolvedTurning));
                        }
                        let mut own = [hit.t; 2];
                        own[dir] = value;
                        let at = if s == 0 {
                            [own[0], own[1], hit.uv.x, hit.uv.y]
                        } else {
                            [hit.uv.x, hit.uv.y, own[0], own[1]]
                        };
                        let edges = if closed {
                            Vec::new()
                        } else {
                            vec![[s, dir, end]]
                        };
                        let point = self.surfaces[0].eval(at[0], at[1]).point;
                        add(
                            &mut seeds,
                            Seed {
                                at,
                                point,
                                edges,
                                departure: None,
                            },
                            self.tol,
                        );
                    }
                }
            }
        }
        Ok(seeds)
    }

    /// The tangent at `e`, `None` where the surfaces are tangent within
    /// `tol.angular` or a surface has no normal.
    fn tangent(&self, e: &[SurfaceEval; 2]) -> Option<Tangent> {
        let n = [
            e[0].du.cross(&e[0].dv).try_normalize(0.0)?,
            e[1].du.cross(&e[1].dv).try_normalize(0.0)?,
        ];
        let along = n[0].cross(&n[1]);
        if along.norm() <= real::sin(self.tol.angular) {
            return None;
        }
        let along = along.normalize();
        let mut velocity = [0.0; 4];
        let mut bends = [0.0; 2];
        for k in 0..2 {
            let (a, b) = (e[k].du, e[k].dv);
            let gram = Matrix2::new(a.dot(&a), a.dot(&b), a.dot(&b), b.dot(&b));
            let uv = gram.try_inverse()? * Vector2::new(a.dot(&along), b.dot(&along));
            velocity[2 * k] = uv.x;
            velocity[2 * k + 1] = uv.y;
            let second = e[k].duu * (uv.x * uv.x)
                + e[k].duv * (2.0 * uv.x * uv.y)
                + e[k].dvv * (uv.y * uv.y);
            bends[k] = second.dot(&n[k]);
        }
        // `κ·n₁ = II₁(T)` and `κ·n₂ = II₂(T)`, `κ` in the plane of the
        // normals.
        let c = n[0].dot(&n[1]);
        let [alpha, beta] = {
            let m = Matrix2::new(1.0, c, c, 1.0).try_inverse()?;
            let ab = m * Vector2::new(bends[0], bends[1]);
            [ab.x, ab.y]
        };
        Some(Tangent {
            along,
            velocity,
            curvature: n[0] * alpha + n[1] * beta,
        })
    }

    /// Newton from `x` onto both surfaces and the plane through `base`
    /// square to `normal`.
    fn correct(&self, mut x: [f64; 4], base: Point3, normal: Vec3) -> Option<[f64; 4]> {
        let on = ON_SHARE * self.tol.linear;
        for _ in 0..CORRECTOR_STEPS {
            let e = self.eval(x);
            let gap = e[0].point - e[1].point;
            let plane = normal.dot(&(e[0].point - base));
            let j = Matrix4::new(
                e[0].du.x,
                e[0].dv.x,
                -e[1].du.x,
                -e[1].dv.x,
                e[0].du.y,
                e[0].dv.y,
                -e[1].du.y,
                -e[1].dv.y,
                e[0].du.z,
                e[0].dv.z,
                -e[1].du.z,
                -e[1].dv.z,
                normal.dot(&e[0].du),
                normal.dot(&e[0].dv),
                0.0,
                0.0,
            );
            let step = j.lu().solve(&Vector4::new(gap.x, gap.y, gap.z, plane))?;
            for k in 0..4 {
                x[k] -= step[k];
            }
            if !x.iter().all(|c| c.is_finite()) {
                return None;
            }
            if gap.norm() <= on && plane.abs() <= on {
                let e = self.eval(x);
                return ((e[0].point - e[1].point).norm() <= on).then_some(x);
            }
        }
        None
    }

    /// One step's length at curvature `kappa`: the chord whose sag is
    /// the walk's.
    fn step(&self, kappa: f64) -> f64 {
        if kappa > 0.0 {
            (8.0 * self.sag / kappa).sqrt().min(self.step_max)
        } else {
            self.step_max
        }
    }

    /// The reach of a singular point at `at`: a step at the larger
    /// principal curvature of either surface there.
    fn reach(&self, at: [f64; 4]) -> f64 {
        let e = self.eval(at);
        let kappa = e.iter().map(principal).fold(0.0, f64::max);
        self.step(kappa)
    }

    /// The sign to walk from `seed`: into the domains from an edge, away
    /// from the singular point from a departure, along `n₁ × n₂` from a
    /// seed that is passed.
    fn leaving(&self, seed: &Seed, singular: &[Singular]) -> Result<f64, GeomError> {
        let Some(t) = self.tangent(&self.eval(seed.at)) else {
            return Err(refused(SectionFault::CrowdedSingularity));
        };
        if let Some(k) = seed.departure {
            return Ok(if t.along.dot(&(seed.point - singular[k].point)) >= 0.0 {
                1.0
            } else {
                -1.0
            });
        }
        let mut sign = None;
        for &[s, dir, end] in &seed.edges {
            let axis = 2 * s + dir;
            let speed = real::hypot(t.velocity[2 * s], t.velocity[2 * s + 1]);
            let across = t.velocity[axis];
            if across.abs() <= self.tol.angular * speed {
                return Err(refused(SectionFault::UnresolvedTurning));
            }
            let inward = if end == 0 { 1.0 } else { -1.0 };
            let this = if across * inward > 0.0 { 1.0 } else { -1.0 };
            if sign.is_some_and(|s| s != this) {
                // A corner the section only touches.
                return Err(refused(SectionFault::UnresolvedTurning));
            }
            sign = Some(this);
        }
        Ok(sign.unwrap_or(1.0))
    }

    /// Walk from seed `i` the way `sign` says until an end seed or back
    /// to `i`, marking the seeds passed.
    fn walk(
        &self,
        i: usize,
        sign: f64,
        seeds: &[Seed],
        singular: &[Singular],
        visited: &mut [bool],
        meter: &mut Meter,
    ) -> Result<(Vec<[f64; 4]>, End), GeomError> {
        let mut x = seeds[i].at;
        let mut points = vec![x];
        let Some(mut t) = self.tangent(&self.eval(x)) else {
            return Err(refused(SectionFault::CrowdedSingularity));
        };
        t.along *= sign;
        t.velocity = t.velocity.map(|c| c * sign);
        for _ in 0..WALK_STEPS {
            meter.tick()?;
            let p = self.surfaces[0].eval(x[0], x[1]).point;
            let (next, after) = self.advance(x, p, &t)?;
            let q = self.surfaces[0].eval(next[0], next[1]).point;
            if let Some(j) = self.arrival(p, q, x, next, seeds) {
                if j == i {
                    return Ok((points, End::Closed));
                }
                let at = near(seeds[j].at, next, &self.closure);
                if seeds[j].ends() {
                    visited[j] = true;
                    points.push(at);
                    return Ok((points, End::Seed(j)));
                }
                if visited[j] {
                    return Err(refused(SectionFault::UnvisitedSeed));
                }
                visited[j] = true;
                points.push(at);
                x = at;
                let Some(mut there) = self.tangent(&self.eval(x)) else {
                    return Err(refused(SectionFault::CrowdedSingularity));
                };
                if there.along.dot(&t.along) < 0.0 {
                    there.along = -there.along;
                    there.velocity = there.velocity.map(|c| -c);
                }
                t = there;
                continue;
            }
            let outside = (0..4).any(|k| {
                let d = self.domain[k];
                let slack = PATCH_SLACK * d.length();
                self.closure[k].is_none() && (next[k] < d.lo() - slack || next[k] > d.hi() + slack)
            });
            if outside || singular.iter().any(|s| (q - s.point).norm() < s.reach) {
                return Err(refused(SectionFault::UnvisitedSeed));
            }
            points.push(next);
            x = next;
            t = after;
        }
        Err(refused(SectionFault::UnvisitedSeed))
    }

    /// One accepted step from `x` (at `p`) along `t`: the point and the
    /// tangent there, oriented as `t` (module docs).
    fn advance(
        &self,
        x: [f64; 4],
        p: Point3,
        t: &Tangent,
    ) -> Result<([f64; 4], Tangent), GeomError> {
        let kappa = t.curvature.norm();
        let mut h = self.step(kappa);
        for _ in 0..STEP_HALVINGS {
            if let Some(found) = self.tried(x, p, t, kappa, h) {
                return Ok(found);
            }
            h *= 0.5;
        }
        Err(refused(SectionFault::UnvisitedSeed))
    }

    /// The step of length `h`, if it is accepted.
    fn tried(
        &self,
        x: [f64; 4],
        p: Point3,
        t: &Tangent,
        kappa: f64,
        h: f64,
    ) -> Option<([f64; 4], Tangent)> {
        let predicted: [f64; 4] = core::array::from_fn(|k| x[k] + h * t.velocity[k]);
        let base = p + t.along * h;
        let next = self.correct(predicted, base, t.along)?;
        let e = self.eval(next);
        let q = e[0].point;
        if (q - base).norm() > STEP_DRIFT * h {
            return None;
        }
        let mut after = self.tangent(&e)?;
        if after.along.dot(&t.along) < 0.0 {
            after.along = -after.along;
            after.velocity = after.velocity.map(|c| -c);
        }
        let bend = kappa.max(after.curvature.norm());
        if (after.along - t.along).norm() > CURVATURE_SLACK * bend * h + self.tol.angular {
            return None;
        }
        let chord = q - p;
        let middle: [f64; 4] = core::array::from_fn(|k| 0.5 * (x[k] + next[k]));
        let half = Point3::from(0.5 * (p.coords + q.coords));
        let on = self.correct(middle, half, chord.normalize())?;
        let m = self.surfaces[0].eval(on[0], on[1]).point;
        let sag = (bend * h * h / 8.0).max(self.sag);
        ((m - half).norm() <= CURVATURE_SLACK * sag).then_some((next, after))
    }

    /// The seed the chord from `p` (at `x`) to `q` (at `next`) reaches:
    /// one within the sag's slack of it, past its start, its parameters
    /// inside the step's box; the nearest along it. The walk's own start
    /// counts once it is ahead again.
    fn arrival(
        &self,
        p: Point3,
        q: Point3,
        x: [f64; 4],
        next: [f64; 4],
        seeds: &[Seed],
    ) -> Option<usize> {
        let chord = q - p;
        let length = chord.norm();
        let unit = chord / length;
        let reach = CURVATURE_SLACK * self.sag;
        let mut best: Option<(usize, f64)> = None;
        for (j, seed) in seeds.iter().enumerate() {
            let offset = seed.point - p;
            let along = offset.dot(&unit);
            // Behind the chord's start: the seed the step starts from, or
            // one a step before already reached.
            if along <= reach || along > length + reach || (offset - unit * along).norm() > reach {
                continue;
            }
            let at = near(seed.at, next, &self.closure);
            // A turning point is past the chord's range in the parameter
            // it turns in, by the sag: the box is grown by the step's
            // largest move. It only tells sheets of one point apart.
            let moved = (0..4).map(|k| (next[k] - x[k]).abs()).fold(0.0, f64::max);
            let inside = (0..4).all(|k| {
                let margin = moved + PATCH_SLACK * self.domain[k].length();
                x[k].min(next[k]) - margin <= at[k] && at[k] <= x[k].max(next[k]) + margin
            });
            if inside && best.is_none_or(|(_, a)| along < a) {
                best = Some((j, along));
            }
        }
        best.map(|(j, _)| j)
    }

    /// Gauss–Newton from `at` onto the point where the surfaces meet with
    /// parallel normals, if it is a singular point within the tolerance.
    fn singular(&self, at: [f64; 4], meter: &mut Meter) -> Result<Option<[f64; 4]>, GeomError> {
        let residual = |x: [f64; 4]| -> Option<([f64; 5], [[f64; 4]; 5])> {
            let e = self.eval(x);
            let normal = e[1].du.cross(&e[1].dv);
            let length = normal.norm();
            if length == 0.0 {
                return None;
            }
            let n = normal / length;
            let turned = |d: Vec3| (d - n * n.dot(&d)) / length;
            let dn = [
                turned(e[1].duu.cross(&e[1].dv) + e[1].du.cross(&e[1].duv)),
                turned(e[1].duv.cross(&e[1].dv) + e[1].du.cross(&e[1].dvv)),
            ];
            let gap = e[0].point - e[1].point;
            let (a, b) = (e[0].du, e[0].dv);
            let r = [gap.x, gap.y, gap.z, n.dot(&a), n.dot(&b)];
            let mut j = [[0.0; 4]; 5];
            for k in 0..3 {
                j[k] = [a[k], b[k], -e[1].du[k], -e[1].dv[k]];
            }
            j[3] = [
                n.dot(&e[0].duu),
                n.dot(&e[0].duv),
                dn[0].dot(&a),
                dn[1].dot(&a),
            ];
            j[4] = [
                n.dot(&e[0].duv),
                n.dot(&e[0].dvv),
                dn[0].dot(&b),
                dn[1].dot(&b),
            ];
            Some((r, j))
        };
        let size = |r: &[f64; 5]| r.iter().map(|c| c * c).sum::<f64>();
        let mut x = at;
        let Some((mut r, mut j)) = residual(x) else {
            return Ok(None);
        };
        for _ in 0..SINGULAR_STEPS {
            meter.tick()?;
            let mut jtj = Matrix4::zeros();
            let mut g = Vector4::zeros();
            for row in 0..5 {
                for a in 0..4 {
                    g[a] += j[row][a] * r[row];
                    for b in 0..4 {
                        jtj[(a, b)] += j[row][a] * j[row][b];
                    }
                }
            }
            let diagonal = Matrix4::from_diagonal(&jtj.diagonal());
            let mut damping = 0.0;
            let mut moved = false;
            for _ in 0..=DAMPINGS {
                let tried = jtj + diagonal * damping;
                damping = if damping == 0.0 {
                    DAMPING
                } else {
                    damping * DAMPING_GROWTH
                };
                let Some(step) = tried.lu().solve(&g) else {
                    continue;
                };
                let next: [f64; 4] = core::array::from_fn(|k| x[k] - step[k]);
                if let Some((rn, jn)) = residual(next)
                    && size(&rn) < size(&r)
                {
                    (x, r, j) = (next, rn, jn);
                    moved = true;
                    break;
                }
            }
            if !moved {
                break;
            }
        }
        let e = self.eval(x);
        let tangent = match (
            e[0].du.cross(&e[0].dv).try_normalize(0.0),
            e[1].du.cross(&e[1].dv).try_normalize(0.0),
        ) {
            (Some(a), Some(b)) => a.cross(&b).norm() <= real::sin(self.tol.angular),
            _ => false,
        };
        let meets = (e[0].point - e[1].point).norm() <= self.tol.linear;
        Ok((tangent && meets).then_some(x))
    }

    /// Newton from `at` onto a turning point in `v₁` of the section, the
    /// loop condition `n₂·∂P₁/∂u₁ = 0`, for a box the isolation could
    /// not certify that is no singular point.
    fn turning(&self, at: [f64; 4], meter: &mut Meter) -> Result<Option<[f64; 4]>, GeomError> {
        let mut x = at;
        for _ in 0..CORRECTOR_STEPS {
            meter.tick()?;
            let e = self.eval(x);
            let Some(n) = e[1].du.cross(&e[1].dv).try_normalize(0.0) else {
                return Ok(None);
            };
            let gap = e[0].point - e[1].point;
            let a = e[0].du;
            // `n₂`'s turn is second order beside the first surface's, as
            // the walk's corrector holds its own plane fixed.
            let j = Matrix4::new(
                a.x,
                e[0].dv.x,
                -e[1].du.x,
                -e[1].dv.x,
                a.y,
                e[0].dv.y,
                -e[1].du.y,
                -e[1].dv.y,
                a.z,
                e[0].dv.z,
                -e[1].du.z,
                -e[1].dv.z,
                n.dot(&e[0].duu),
                n.dot(&e[0].duv),
                0.0,
                0.0,
            );
            let Some(step) = j.lu().solve(&Vector4::new(gap.x, gap.y, gap.z, n.dot(&a))) else {
                return Ok(None);
            };
            for k in 0..4 {
                x[k] -= step[k];
            }
        }
        let e = self.eval(x);
        let on = (e[0].point - e[1].point).norm() <= ON_SHARE * self.tol.linear;
        let inside = (0..4).all(|k| self.closure[k].is_some() || self.domain[k].contains(x[k]));
        Ok((on && inside && self.tangent(&e).is_some()).then_some(x))
    }

    /// The departures of a singular point: the section's points at its
    /// reach from it, certified, over the patch pairs whose hulls meet
    /// its ball.
    fn departures(&self, s: &Singular, meter: &mut Meter) -> Result<Vec<[f64; 4]>, GeomError> {
        let [one, two] = self.surfaces;
        let origin = middle(one, two);
        let centre = s.point - origin;
        let nets = |x: &NurbsSurface| -> Vec<Net> {
            x.patches().iter().map(|p| Net::of(p, origin)).collect()
        };
        let (first, second) = (nets(one), nets(two));
        let top = first
            .iter()
            .chain(&second)
            .map(|n| n.degree as usize)
            .max()
            .unwrap_or(1);
        let bin = Binomials::new(4 * top + 4);
        let rho = s.reach;
        let ball = |n: &Net| {
            (0..3).all(|k| n.hull[0][k] <= centre[k] + rho && centre[k] - rho <= n.hull[1][k])
        };
        let unit = PolyN::new([0, 0], vec![1.0]);
        let mut found = Vec::new();
        for a in first.iter().filter(|n| ball(n)) {
            // `|X₁ − w₁·S|² − ρ²·w₁²`, a positive multiple of `|P₁ − S|² − ρ²`.
            let offset: [PolyN<2>; 3] =
                core::array::from_fn(|k| PolyN::combine(&[(1.0, &a.x[k]), (-centre[k], &a.w)]));
            let sphere = sum(&[
                (1.0, offset[0].mul(&offset[0], &bin)),
                (1.0, offset[1].mul(&offset[1], &bin)),
                (1.0, offset[2].mul(&offset[2], &bin)),
                (-rho * rho, a.w.mul(&a.w, &bin)),
            ]);
            let size = offset
                .iter()
                .flat_map(|p| p.coefficients())
                .fold(0.0, |m: f64, c| m.max(c.abs()));
            let error = a.reach + centre.norm() * a.weight;
            let floor =
                BERNSTEIN_ROUNDING * (6.0 * size * error + 2.0 * rho * rho * a.weight * a.weight);
            let g = PolyN::tensor(&sphere, &unit);
            for b in second.iter().filter(|n| ball(n)) {
                let slack = BERNSTEIN_ROUNDING * (a.reach + b.reach);
                if !hulls_meet(a, b, slack) {
                    continue;
                }
                let ([f0, f1, f2], f) = section(a, b);
                let spans = [a.span[0], a.span[1], b.span[0], b.span[1]];
                let isolation =
                    common_zeros([&f0, &f1, &f2, &g], [f, f, f, floor], meter).map_err(stopped)?;
                for z in isolation.zeros.into_iter().filter_map(|z| own(z, spans)) {
                    if !z.certified {
                        return Err(refused(SectionFault::CrowdedSingularity));
                    }
                    found.push(z);
                }
            }
        }
        let width = self.domain.iter().map(|d| d.length()).fold(0.0, f64::max);
        let wrap = Wrap {
            period: self.closure,
            origin: self.domain.map(|d| d.lo()),
            touch: PATCH_SLACK * width,
        };
        Ok(merge(found, &wrap)
            .into_iter()
            .map(|z: Zero<4>| z.at)
            .collect())
    }
}

/// The larger principal curvature's size at `e`, zero where the surface
/// has no normal.
fn principal(e: &SurfaceEval) -> f64 {
    let Some(n) = e.du.cross(&e.dv).try_normalize(0.0) else {
        return 0.0;
    };
    let (ee, f, g) = (e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv));
    let (l, m, nn) = (e.duu.dot(&n), e.duv.dot(&n), e.dvv.dot(&n));
    let det = ee * g - f * f;
    if det <= 0.0 {
        return 0.0;
    }
    let mean = (ee * nn - 2.0 * f * m + g * l) / (2.0 * det);
    let gauss = (l * nn - m * m) / det;
    mean.abs() + (mean * mean - gauss).max(0.0).sqrt()
}

#[cfg(test)]
mod tests;
