//! The geometry oracle: Arris's evaluations, projections and
//! intersections against Open CASCADE's for every `tests/fixtures/geom/`
//! fixture — evaluations and projected parameters to 1e-9 relative, the
//! intersection types exactly, and the oracle's sampled points on Arris's
//! curves to 1e-9. The oracle is the parametrisation's ground truth
//! (`docs/DATA-MODEL.md` §Conventions); a mismatch here is fixed in Arris.

use std::collections::BTreeMap;

use arris_debug::fixtures::geom::{
    Evaluation, GeomFixture, PairResult, Projection, build_curve, build_surface,
};
use arris_debug::fixtures::{Kind, corpus, kind_of};
use arris_debug::testing::{REL, close, close_param};
use arris_geom::{
    Curve, CurveIntersection, CurveSurfaceIntersection, GeomError, MeetKind, SECTION_FIT_FRACTION,
    Surface, SurfaceIntersection, SurfaceKind, intersect_curve_surface, intersect_curves,
    intersect_surfaces,
};
use arris_math::{Point3, Precision, Tolerance, Vec3};

/// Where a curve touches a surface the crossing's location is
/// conditioned as the square root of the rounding — a touch perturbed
/// by 1e-15 in distance moves its point by 1e-7·√R — so the oracle's
/// points at a touch (it reports none, one or two, having no tolerance
/// there) are held to this against Arris's tangent hit, and so are its
/// one or two rulings where two parallel cylinders touch.
const TOUCH: f64 = 1e-6;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// A region every traced section of these tests lies in; the closed
/// forms ignore it.
fn within() -> arris_math::Aabb {
    arris_math::Aabb {
        min: [-100.0; 3],
        max: [100.0; 3],
    }
}

fn p3(a: &[f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

fn v3(a: &[f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn close3(a: Vec3, b: Vec3) -> bool {
    (a - b).norm() <= REL * a.norm().max(b.norm()).max(1.0)
}

struct Built {
    surfaces: BTreeMap<String, Surface>,
    curves: BTreeMap<String, Curve>,
}

fn build(f: &GeomFixture) -> Built {
    let r = &f.recipe;
    Built {
        surfaces: r
            .surfaces
            .iter()
            .map(|(n, s)| (n.clone(), build_surface(n, s, &r.params).unwrap()))
            .collect(),
        curves: r
            .curves
            .iter()
            .map(|(n, c)| (n.clone(), build_curve(n, c, &r.params).unwrap()))
            .collect(),
    }
}

fn geometry_fixtures() -> Vec<GeomFixture> {
    corpus()
        .iter()
        .filter(|d| kind_of(d).unwrap() == Kind::Geometry)
        .map(|d| arris_debug::fixtures::geom::load(d).unwrap())
        .collect()
}

fn check_surface_evaluation(name: &str, s: &Surface, e: &Evaluation, errors: &mut Vec<String>) {
    let Evaluation::Surface {
        at,
        point,
        du,
        dv,
        duu,
        duv,
        dvv,
    } = e
    else {
        errors.push(format!(
            "{name}: a surface evaluation carries a curve result"
        ));
        return;
    };
    let ours = s.eval(at[0], at[1]);
    for (what, a, b) in [
        ("point", ours.point.coords, v3(point)),
        ("du", ours.du, v3(du)),
        ("dv", ours.dv, v3(dv)),
        ("duu", ours.duu, v3(duu)),
        ("duv", ours.duv, v3(duv)),
        ("dvv", ours.dvv, v3(dvv)),
    ] {
        if !close3(a, b) {
            errors.push(format!("{name} at {at:?}: {what} {a:?} vs oracle {b:?}"));
        }
    }
}

fn check_curve_evaluation(name: &str, c: &Curve, e: &Evaluation, errors: &mut Vec<String>) {
    let Evaluation::Curve { at, point, d1, d2 } = e else {
        errors.push(format!(
            "{name}: a curve evaluation carries a surface result"
        ));
        return;
    };
    let ours = c.eval(*at);
    for (what, a, b) in [
        ("point", ours.point.coords, v3(point)),
        ("d1", ours.d1, v3(d1)),
        ("d2", ours.d2, v3(d2)),
    ] {
        if !close3(a, b) {
            errors.push(format!("{name} at {at}: {what} {a:?} vs oracle {b:?}"));
        }
    }
}

fn check_surface_projection(name: &str, s: &Surface, p: &Projection, errors: &mut Vec<String>) {
    let Some(uv) = p.uv else {
        errors.push(format!("{name}: a surface projection without uv"));
        return;
    };
    let ours = match s.project(p3(&p.point)) {
        Ok(o) => o,
        Err(e) => {
            errors.push(format!("{name} projecting {:?}: {e}", p.point));
            return;
        }
    };
    let [pu, pv] = s.period();
    if !close_param(ours.uv.x, uv[0], pu) || !close_param(ours.uv.y, uv[1], pv) {
        errors.push(format!(
            "{name} projecting {:?}: uv {:?} vs oracle {uv:?}",
            p.point, ours.uv
        ));
    }
    if !close3(ours.point.coords, v3(&p.nearest)) || !close(ours.distance, p.distance) {
        errors.push(format!(
            "{name} projecting {:?}: {:?} at {} vs oracle {:?} at {}",
            p.point, ours.point, ours.distance, p.nearest, p.distance
        ));
    }
}

fn check_curve_projection(name: &str, c: &Curve, p: &Projection, errors: &mut Vec<String>) {
    let Some(t) = p.t else {
        errors.push(format!("{name}: a curve projection without t"));
        return;
    };
    let ours = match c.project(p3(&p.point)) {
        Ok(o) => o,
        Err(e) => {
            errors.push(format!("{name} projecting {:?}: {e}", p.point));
            return;
        }
    };
    if !close_param(ours.t, t, c.period()) {
        errors.push(format!(
            "{name} projecting {:?}: t {} vs oracle {t}",
            p.point, ours.t
        ));
    }
    if !close3(ours.point.coords, v3(&p.nearest)) || !close(ours.distance, p.distance) {
        errors.push(format!(
            "{name} projecting {:?}: {:?} at {} vs oracle {:?} at {}",
            p.point, ours.point, ours.distance, p.nearest, p.distance
        ));
    }
}

/// Arris's type name for a surface pair, in the oracle's vocabulary.
fn surface_type(r: &SurfaceIntersection) -> (String, Vec<&Curve>) {
    match r {
        SurfaceIntersection::Empty => ("empty".into(), Vec::new()),
        SurfaceIntersection::Coincident => ("coincident".into(), Vec::new()),
        SurfaceIntersection::Meets { curves, .. } if curves.is_empty() => {
            ("point".into(), Vec::new())
        }
        SurfaceIntersection::Meets { curves, .. } => {
            let kinds: std::collections::BTreeSet<String> =
                curves.iter().map(|c| c.curve.kind().to_string()).collect();
            (
                kinds.into_iter().collect::<Vec<_>>().join("+"),
                curves.iter().map(|c| &c.curve).collect(),
            )
        }
    }
}

/// Arris's name for the kind of an oracle curve: a parabola or a
/// hyperbola's branch, which no `Curve` variant carries, is an exact
/// rational quadratic NURBS (ADR-0018).
fn ours(kind: &str) -> &str {
    match kind {
        "parabola" | "hyperbola" => "NURBS",
        other => other,
    }
}

/// `true` when the surfaces meet in curves and touch along every one.
fn touching(r: &SurfaceIntersection) -> bool {
    !r.curves().is_empty() && r.curves().iter().all(|c| c.kind == MeetKind::Touch)
}

/// The points of a result that meets in isolated points alone, else none.
fn only_points(r: &SurfaceIntersection) -> Vec<Point3> {
    if r.curves().is_empty() {
        r.points().iter().map(|p| p.point).collect()
    } else {
        Vec::new()
    }
}

/// How far a fitted section curve may be from each surface: the fit's
/// fraction of the tolerance, plus rounding.
fn fitted_bound(p: Point3) -> f64 {
    SECTION_FIT_FRACTION * tol().linear + REL * p.coords.norm().max(1.0)
}

/// Every sampled point of `c` within `REL` of both surfaces by their own
/// projections — a fitted curve within [`fitted_bound`]: what a curve of
/// Arris's is held to where the oracle is silent, and beside the
/// oracle's walked sections.
fn on_both_surfaces(c: &Curve, a: &Surface, b: &Surface) -> bool {
    let domain = c.domain();
    (0..8).all(|i| {
        let t = match c.period() {
            Some(p) => p * i as f64 / 8.0,
            None if domain.is_bounded() => domain.lerp(i as f64 / 7.0),
            None => -3.0 + i as f64,
        };
        let p = c.point(t);
        let bound = match c {
            Curve::Nurbs(_) => fitted_bound(p),
            _ => REL * p.coords.norm().max(1.0),
        };
        [a, b].iter().all(|s| {
            s.project(p)
                .map(|proj| proj.distance <= bound)
                .unwrap_or(false)
        })
    })
}

fn check_surface_pair(a: &Surface, b: &Surface, res: &PairResult, errors: &mut Vec<String>) {
    let label = format!("{} vs {}", res.a, res.b);
    if res.kind == "unsolved" {
        // The oracle found no conic: a pair Arris refuses too, a pair
        // Arris decides empty by its closed form (skew axes further apart
        // than the radii), which the oracle has no case for, a coaxial
        // sphere the oracle's exact test on its own axis lets go, or a
        // section Arris traces and fits. Its curves are held to both
        // surfaces, and to the lines the oracle walks where it walked
        // any. Which one each pair is, is pinned by name below.
        match intersect_surfaces(a, b, &within(), tol(), &mut arris_math::Meter::default()) {
            Err(GeomError::Unsupported { .. }) => {}
            Ok(SurfaceIntersection::Empty) if res.curves.is_empty() => {}
            Ok(r) => {
                let curves = surface_type(&r).1;
                for c in &curves {
                    if !on_both_surfaces(c, a, b) {
                        errors.push(format!("{label}: {c:?} is not on both surfaces"));
                    }
                }
                // The oracle drops the lines it walks along a tangency,
                // having no crossing to polish their samples onto
                // (`dropped`), so a curve Arris touches along — the tube
                // circle a pipe elbow shares with its torus — has no
                // walked line covering it, and the ends of the walked
                // crossings sit on it. It is held to both surfaces above,
                // and exactly: it is no fit.
                let (touched, walked): (Vec<&Curve>, Vec<&Curve>) =
                    if res.dropped.is_some_and(|n| n > 0) {
                        let of = |kind| {
                            (r.curves().iter())
                                .filter(move |m| m.kind == kind)
                                .map(|m| &m.curve)
                                .collect()
                        };
                        (of(MeetKind::Touch), of(MeetKind::Crossing))
                    } else {
                        (Vec::new(), curves)
                    };
                if !res.curves.is_empty() {
                    check_walked(&label, &walked, &touched, res, errors);
                }
            }
            Err(e) => errors.push(format!("{label}: {e} vs oracle unsolved")),
        }
        return;
    }
    let r = match intersect_surfaces(a, b, &within(), tol(), &mut arris_math::Meter::default()) {
        Ok(r) => r,
        Err(e) => {
            errors.push(format!("{label}: {e}"));
            return;
        }
    };
    let (kind, curves) = surface_type(&r);
    let points = only_points(&r);
    if !points.is_empty() {
        // Isolated points: the oracle's `point` has the same count, each
        // of its points within REL of one of ours — the foot of a centre,
        // well conditioned unlike the crossings a touch splits into. The
        // oracle decides a plane on a sphere at machine epsilon and may
        // report the touch as `empty` or as a circle of rounding radius
        // instead, which is accepted within TOUCH.
        match res.kind.as_str() {
            "point" if res.points.len() == points.len() => {
                for p in &res.points {
                    let q = p3(p);
                    if !points
                        .iter()
                        .any(|ours| (ours - q).norm() <= REL * q.coords.norm().max(1.0))
                    {
                        errors.push(format!(
                            "{label}: the oracle's point {p:?} is not one of {points:?}"
                        ));
                    }
                }
            }
            "empty" => {}
            "circle" => {
                for sample in &res.curves {
                    for p in &sample.points {
                        let q = p3(p);
                        if !points.iter().any(|ours| (ours - q).norm() <= TOUCH) {
                            errors.push(format!(
                                "{label}: the oracle's circle through {p:?} is not a touch at {points:?}"
                            ));
                        }
                    }
                }
            }
            _ => errors.push(format!(
                "{label}: {} points vs oracle {} with {} points",
                points.len(),
                res.kind,
                res.points.len()
            )),
        }
        return;
    }
    // At a touch the oracle, deciding it by rounding, may split one ruling
    // into two a square root of the rounding apart: every oracle curve is
    // then held to TOUCH against our one, whatever their count.
    let touch = touching(&r);
    let counts_agree = if touch {
        !res.curves.is_empty()
    } else {
        curves.len() == res.curves.len()
    };
    if kind != ours(&res.kind) || !counts_agree {
        errors.push(format!(
            "{label}: {kind} with {} curves vs oracle {} with {}",
            curves.len(),
            res.kind,
            res.curves.len()
        ));
        return;
    }
    // Each oracle curve is one of ours: every sampled point lies on some
    // Arris curve of the same kind, and each Arris curve carries one
    // oracle curve (all of them, at a touch).
    let mut taken = vec![false; curves.len()];
    for sample in &res.curves {
        let on = |c: &Curve| {
            c.kind().to_string() == ours(&sample.kind)
                && sample.points.iter().all(|p| {
                    let bound = if touch {
                        TOUCH
                    } else {
                        REL * p3(p).coords.norm().max(1.0)
                    };
                    c.project(p3(p))
                        .map(|proj| proj.distance <= bound)
                        .unwrap_or(false)
                })
        };
        match curves
            .iter()
            .enumerate()
            .find(|(i, c)| (touch || !taken[*i]) && on(c))
        {
            Some((i, _)) => taken[i] = true,
            None => errors.push(format!(
                "{label}: the oracle's {} through {:?} is not one of {curves:?}",
                sample.kind, sample.points[0]
            )),
        }
    }
}

/// Arris's curves against the lines the oracle walked for a pair it found
/// no conic for: every polished sample on one of Arris's curves within
/// the fit's fraction of the tolerance ([`fitted_bound`]) — the samples
/// are on both surfaces to rounding, and at these transversal poses a
/// point within the fraction of both is as near the section (the
/// corpus's worst is 1.1e-8 against 2.5e-8) — and every
/// curve of Arris's carrying at least one sample and never farther from
/// the samples than they are spaced, so no curve is spurious or longer
/// than the section. A sample on one of `touched` — a curve Arris
/// returns exactly and the oracle dropped the walk of — is accounted
/// for there and left out: the walked crossings end on it.
fn check_walked(
    label: &str,
    curves: &[&Curve],
    touched: &[&Curve],
    res: &PairResult,
    errors: &mut Vec<String>,
) {
    let on_touched = |p: Point3| {
        touched.iter().any(|c| {
            c.project(p)
                .is_ok_and(|proj| proj.distance <= REL * p.coords.norm().max(1.0))
        })
    };
    let samples: Vec<(Point3, f64)> = res
        .curves
        .iter()
        .flat_map(|walked| walked.points.iter().map(|p| (p3(p), fitted_bound(p3(p)))))
        .filter(|&(p, _)| !on_touched(p))
        .collect();
    let off = |c: &Curve, p: Point3| c.project(p).map_or(f64::INFINITY, |proj| proj.distance);
    let mut carried = vec![false; curves.len()];
    for &(p, bound) in &samples {
        match curves.iter().position(|c| off(c, p) <= bound) {
            Some(i) => carried[i] = true,
            None => errors.push(format!(
                "{label}: the oracle's walked point {p} is on none of {} curves, nearest {}",
                curves.len(),
                curves
                    .iter()
                    .map(|c| off(c, p))
                    .fold(f64::INFINITY, f64::min)
            )),
        }
    }
    // The largest gap between a walked sample and its nearest neighbour
    // bounds how far a point of the section is from every sample.
    let spacing = samples
        .iter()
        .map(|&(p, _)| {
            samples
                .iter()
                .map(|&(q, _)| (p - q).norm())
                .filter(|&d| d > 0.0)
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max);
    for (c, carried) in curves.iter().zip(carried) {
        if !carried {
            errors.push(format!(
                "{label}: {c:?} carries none of the oracle's walked points"
            ));
        }
        let domain = c.domain();
        for i in 0..=64 {
            let p = c.point(domain.lerp(i as f64 / 64.0));
            let near = samples
                .iter()
                .map(|&(q, _)| (p - q).norm())
                .fold(f64::INFINITY, f64::min);
            if near > spacing {
                errors.push(format!(
                    "{label}: {p} of a curve is {near} from every walked point, spaced {spacing}"
                ));
                break;
            }
        }
    }
}

fn check_curve_pair(c: &Curve, s: &Surface, res: &PairResult, errors: &mut Vec<String>) {
    let label = format!("{} vs {}", res.a, res.b);
    let r = match intersect_curve_surface(c, s, tol(), &mut arris_math::Meter::default()) {
        Ok(r) => r,
        Err(e) => {
            errors.push(format!("{label}: {e}"));
            return;
        }
    };
    let hits = match (&r, res.kind.as_str()) {
        (CurveSurfaceIntersection::Coincident, "coincident") => return,
        // A ruling of a cone in a general pose: the oracle's quadratic has
        // rounding for its coefficients where they vanish, and it reports
        // points of the ruling rather than a line in the quadric. Each of
        // them is held to the line, which Arris says lies on the cone.
        (CurveSurfaceIntersection::Coincident, "points") if s.kind() == SurfaceKind::Cone => {
            for h in &res.hits {
                let p = p3(&h.point);
                let off = c.project(p).map_or(f64::INFINITY, |proj| proj.distance);
                if off > REL * p.coords.norm().max(1.0) {
                    errors.push(format!(
                        "{label}: coincident, but the oracle's point {p} is {off} off the line"
                    ));
                }
            }
            return;
        }
        (CurveSurfaceIntersection::Points(h), "points") => h,
        _ => {
            errors.push(format!("{label}: {r:?} vs oracle {}", res.kind));
            return;
        }
    };
    // Every oracle hit is one of ours: a transversal hit to REL, a touch
    // to TOUCH; every transversal hit of ours is an oracle hit. A touch
    // may have no oracle hit at all.
    for h in &res.hits {
        let p = p3(&h.point);
        let scale = p.coords.norm().max(1.0);
        let matched = hits.iter().any(|ours| {
            let bound = if ours.tangent { TOUCH } else { REL * scale };
            (ours.point - p).norm() <= bound
                && (ours.t - h.t).abs().min(turn_diff(ours.t, h.t, c)) <= bound
        });
        if !matched {
            errors.push(format!(
                "{label}: the oracle's hit at t = {} {:?} is not among {hits:?}",
                h.t, h.point
            ));
        }
    }
    for ours in hits.iter().filter(|h| !h.tangent) {
        let scale = ours.point.coords.norm().max(1.0);
        if !res
            .hits
            .iter()
            .any(|h| (ours.point - p3(&h.point)).norm() <= REL * scale)
        {
            errors.push(format!(
                "{label}: our hit at t = {} {:?} is not among the oracle's {:?}",
                ours.t, ours.point, res.hits
            ));
        }
    }
}

/// Two curves against the oracle's extrema as `check_curve_pair` holds a
/// curve against a surface: every oracle hit is one of ours — a crossing
/// to REL in its point and both parameters, a touch to TOUCH — and every
/// crossing of ours is an oracle hit. `intersect_curves` keeps the point
/// it computed for a pair it swapped, which is the second curve's, and a
/// crossing's two points agree to rounding.
fn check_curve_curve_pair(a: &Curve, b: &Curve, res: &PairResult, errors: &mut Vec<String>) {
    let label = format!("{} vs {}", res.a, res.b);
    let r = match intersect_curves(a, b, tol(), &mut arris_math::Meter::default()) {
        Ok(r) => r,
        Err(e) => {
            errors.push(format!("{label}: {e}"));
            return;
        }
    };
    let hits = match (&r, res.kind.as_str()) {
        (CurveIntersection::Points(h), "points") => h,
        _ => {
            errors.push(format!("{label}: {r:?} vs oracle {}", res.kind));
            return;
        }
    };
    for h in &res.hits {
        let p = p3(&h.point);
        let scale = p.coords.norm().max(1.0);
        let Some(tb) = h.tb else {
            errors.push(format!("{label}: the oracle's hit carries no tb"));
            continue;
        };
        let matched = hits.iter().any(|ours| {
            let bound = if ours.tangent { TOUCH } else { REL * scale };
            (ours.point - p).norm() <= bound
                && (ours.ta - h.t).abs().min(turn_diff(ours.ta, h.t, a)) <= bound
                && (ours.tb - tb).abs().min(turn_diff(ours.tb, tb, b)) <= bound
        });
        if !matched {
            errors.push(format!(
                "{label}: the oracle's hit at t = {}, tb = {tb} {:?} is not among {hits:?}",
                h.t, h.point
            ));
        }
    }
    for ours in hits.iter().filter(|h| !h.tangent) {
        let scale = ours.point.coords.norm().max(1.0);
        if !res
            .hits
            .iter()
            .any(|h| (ours.point - p3(&h.point)).norm() <= REL * scale)
        {
            errors.push(format!(
                "{label}: our hit at t = {} {:?} is not among the oracle's {:?}",
                ours.ta, ours.point, res.hits
            ));
        }
    }
}

/// `|a − b|` modulo the curve's period — or its domain's length for a
/// closed curve that is not periodic, whose two ends are one point — or
/// infinity for an open curve so the plain difference wins.
fn turn_diff(a: f64, b: f64, c: &Curve) -> f64 {
    let domain = c.domain();
    let closed = domain.is_bounded()
        && (c.point(domain.lo()) - c.point(domain.hi())).norm()
            <= REL * c.point(domain.lo()).coords.norm().max(1.0);
    match c.period().or(closed.then(|| domain.length())) {
        Some(p) => {
            let d = (a - b).rem_euclid(p);
            d.min(p - d)
        }
        None => f64::INFINITY,
    }
}

#[test]
fn every_geometry_fixture_matches_the_oracle() {
    let fixtures = geometry_fixtures();
    assert!(
        fixtures.len() >= 12,
        "expected geom/analytic-eval, geom/c1-intersections, geom/c2-cylinder-pairs, geom/c2-quadric-pairs, geom/c3-cylinder-pairs, geom/c3-quadric-pairs, geom/c3-torus-pairs, geom/c3-conic-hits, geom/c3-nurbs-hits, geom/c3-nurbs-crossings, geom/c4-nurbs-projections and geom/c4-closed-curve-hits"
    );
    let mut errors = Vec::new();
    for f in &fixtures {
        let built = build(f);
        for (sample, result) in f.recipe.samples.iter().zip(&f.expected.samples) {
            let name = format!("{}: {}", f.name, sample.of);
            if let Some(s) = built.surfaces.get(&sample.of) {
                for e in &result.evaluations {
                    check_surface_evaluation(&name, s, e, &mut errors);
                }
                for p in &result.projections {
                    check_surface_projection(&name, s, p, &mut errors);
                }
            } else if let Some(c) = built.curves.get(&sample.of) {
                for e in &result.evaluations {
                    check_curve_evaluation(&name, c, e, &mut errors);
                }
                for p in &result.projections {
                    check_curve_projection(&name, c, p, &mut errors);
                }
            }
        }
        for result in &f.expected.pairs {
            if let (Some(a), Some(b)) = (built.curves.get(&result.a), built.curves.get(&result.b)) {
                check_curve_curve_pair(a, b, result, &mut errors);
                continue;
            }
            let Some(b) = built.surfaces.get(&result.b) else {
                errors.push(format!("{}: {} is not a surface", f.name, result.b));
                continue;
            };
            if let Some(a) = built.surfaces.get(&result.a) {
                check_surface_pair(a, b, result, &mut errors);
            } else if let Some(c) = built.curves.get(&result.a) {
                check_curve_pair(c, b, result, &mut errors);
            }
        }
    }
    assert!(
        errors.is_empty(),
        "{} mismatches:\n{}",
        errors.len(),
        errors.join("\n")
    );
}

/// What Arris says about every pair of `geom/c1-intersections`, by name:
/// the case each was built as. The oracle comparison above is one-sided
/// at a touch (it may report nothing), so the classification is pinned
/// here, where a tangent case cannot pass vacuously.
#[test]
fn the_c1_intersection_cases_classify_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c1-intersections")
        .expect("geom/c1-intersections");
    let built = build(&f);
    // Surface pairs: the type and the curve count; curve pairs: the
    // transversal and tangent hit counts, or coincident.
    type SurfaceCase = (&'static str, &'static str, &'static str, usize);
    type CurveCase = (&'static str, &'static str, Option<(usize, usize)>);
    let surface_cases: &[SurfaceCase] = &[
        ("cap", "cyl", "circle", 1),
        ("oblique", "cyl", "ellipse", 1),
        ("chordal", "cyl", "line", 2),
        ("touching", "cyl", "tangent line", 1),
        ("clear", "cyl", "empty", 0),
        ("cap", "oblique", "line", 1),
        ("cap", "cap_lifted", "empty", 0),
        ("cap", "cap_flipped", "coincident", 0),
    ];
    let curve_cases: &[CurveCase] = &[
        ("skewer", "cap", Some((1, 0))),
        ("hover", "cap", Some((0, 0))),
        ("flat", "cap", None),
        ("chord", "cyl", Some((2, 0))),
        ("grazing", "cyl", Some((0, 1))),
        ("miss", "cyl", Some((0, 0))),
        ("ruling", "cyl", None),
        ("offside", "cyl", Some((0, 0))),
        ("crossing_ring", "cap", Some((2, 0))),
        ("kissing_ring", "cap", Some((0, 1))),
        ("hovering_ring", "cap", Some((0, 0))),
        ("lying_ring", "cap", None),
        ("floating_ring", "cap", Some((0, 0))),
        ("parallel", "cyl", None),
        ("meridional", "cyl", Some((4, 0))),
        ("two", "cyl", Some((2, 0))),
        ("outside_touch", "cyl", Some((0, 1))),
        ("inside_touch", "cyl", Some((0, 1))),
        ("outside_miss", "cyl", Some((0, 0))),
        ("inside_miss", "cyl", Some((0, 0))),
        ("section", "cyl", None),
        ("section", "oblique", None),
        ("section", "cut_minor", Some((2, 0))),
        ("section", "touch_major", Some((0, 1))),
        ("section", "clear_major", Some((0, 0))),
        ("meridional_ellipse", "cyl", Some((4, 0))),
        ("grazing_ellipse", "cyl", Some((0, 2))),
        ("inner_ellipse", "cyl", Some((0, 0))),
    ];
    assert_eq!(
        surface_cases.len() + curve_cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, kind, count) in surface_cases {
        let r = intersect_surfaces(
            &built.surfaces[*a],
            &built.surfaces[*b],
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let (got, curves) = surface_type(&r);
        let got = if touching(&r) {
            format!("tangent {got}")
        } else {
            got
        };
        assert_eq!(
            (got.as_str(), curves.len()),
            (*kind, *count),
            "{a} vs {b}: {r:?}"
        );
    }
    for (a, b, expected) in curve_cases {
        let r = intersect_curve_surface(
            &built.curves[*a],
            &built.surfaces[*b],
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let got = match &r {
            CurveSurfaceIntersection::Coincident => None,
            CurveSurfaceIntersection::Points(h) => Some((
                h.iter().filter(|h| !h.tangent).count(),
                h.iter().filter(|h| h.tangent).count(),
            )),
        };
        assert_eq!(got, *expected, "{a} vs {b}: {r:?}");
    }
}

/// What Arris says about every pair of `geom/c2-cylinder-pairs`, by name.
/// The oracle's `unsolved` admits a closed-form empty and a traced
/// quartic, and a tangent ruling reads as a line there, so the case is
/// pinned here: crossing axes of unequal radii meet in two fitted loops,
/// the skew pair within the radii in one.
#[test]
fn the_c2_cylinder_pairs_classify_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c2-cylinder-pairs")
        .expect("geom/c2-cylinder-pairs");
    let built = build(&f);
    let cases: &[(&str, &str, &str, usize)] = &[
        ("cyl", "two", "line", 2),
        ("cyl", "outside", "tangent line", 1),
        ("cyl", "inside", "tangent line", 1),
        ("cyl", "apart", "empty", 0),
        ("cyl", "nested", "empty", 0),
        ("cyl", "cross_90", "ellipse", 2),
        ("cyl", "cross_50", "ellipse", 2),
        ("cyl", "cross_unequal", "NURBS", 2),
        ("cyl", "skew_apart", "empty", 0),
        ("cyl", "skew_close", "NURBS", 1),
        ("two", "cyl", "line", 2),
        ("cross_50", "cyl", "ellipse", 2),
    ];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, kind, count) in cases {
        let got = match intersect_surfaces(
            &built.surfaces[*a],
            &built.surfaces[*b],
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        ) {
            Err(GeomError::Unsupported { .. }) => ("unsupported".to_owned(), 0),
            Err(e) => panic!("{a} vs {b}: {e}"),
            Ok(r) => {
                let (got, curves) = surface_type(&r);
                let got = if touching(&r) {
                    format!("tangent {got}")
                } else {
                    got
                };
                (got, curves.len())
            }
        };
        assert_eq!((got.0.as_str(), got.1), (*kind, *count), "{a} vs {b}");
    }
}

/// What Arris says about every pair of `geom/c2-quadric-pairs`, by name.
/// A touch reads as a circle in the oracle, a plane on a sphere's pole as
/// `empty`, a sphere whose own frame is turned across a cylinder's axis
/// as `unsolved`, a cone's ruling as points on it and a line's touch as
/// none, one or two hits, so the case is pinned here: a surface pair by
/// its type and curve count, a line by its transversal and tangent hits.
#[test]
fn the_c2_quadric_pairs_classify_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c2-quadric-pairs")
        .expect("geom/c2-quadric-pairs");
    let built = build(&f);
    let cases: &[(&str, &str, &str, usize)] = &[
        ("cap_cone", "cone", "circle", 1),
        ("cap_sphere", "sphere", "circle", 1),
        ("cap_torus", "torus", "circle", 2),
        ("touch_sphere", "sphere", "point", 1),
        ("touch_torus", "torus", "tangent circle", 1),
        ("apex_plane", "cone", "point", 1),
        ("bore", "cone", "circle", 2),
        ("bore", "sphere", "circle", 2),
        ("bore", "orb", "circle", 2),
        ("bore", "torus", "empty", 0),
        ("sleeve", "torus", "circle", 2),
        ("touch_sleeve", "torus", "tangent circle", 1),
        ("hoop", "sphere", "tangent circle", 1),
        ("hoop", "orb", "tangent circle", 1),
        ("cone", "cone_same", "circle", 1),
        ("cone", "cone_steep", "circle", 2),
        ("ball", "torus", "circle", 2),
        ("oblique", "sphere", "circle", 1),
        ("sphere", "ball", "circle", 1),
        ("sphere", "pebble", "point", 1),
        ("marble", "sphere", "point", 1),
        ("meridian_cone", "cone", "line", 2),
        ("torus", "meridian_torus", "circle", 2),
        ("cone", "cap_cone", "circle", 1),
        ("torus", "sleeve", "circle", 2),
    ];
    type LineCase = (&'static str, &'static str, Option<(usize, usize)>);
    let lines: &[LineCase] = &[
        ("chord_sphere", "sphere", Some((2, 0))),
        ("graze_sphere", "sphere", Some((0, 1))),
        ("pole_line", "orb", Some((2, 0))),
        ("pole_line", "torus", Some((0, 0))),
        ("skewer_cone", "cone", Some((2, 0))),
        ("ruling", "cone", None),
        ("apex_line", "cone", Some((0, 1))),
        ("tangent_cone", "cone", Some((0, 1))),
        ("parallel_ruling", "cone", Some((1, 0))),
        ("through_tube", "torus", Some((4, 0))),
        ("graze_outer", "torus", Some((0, 1))),
        ("graze_inner", "torus", Some((2, 1))),
        ("vertical_tube", "torus", Some((2, 0))),
    ];
    assert_eq!(
        cases.len() + lines.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, expected) in lines {
        let r = intersect_curve_surface(
            &built.curves[*a],
            &built.surfaces[*b],
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
        let got = match &r {
            CurveSurfaceIntersection::Coincident => None,
            CurveSurfaceIntersection::Points(h) => Some((
                h.iter().filter(|h| !h.tangent).count(),
                h.iter().filter(|h| h.tangent).count(),
            )),
        };
        assert_eq!(got, *expected, "{a} vs {b}: {r:?}");
    }
    for (a, b, kind, count) in cases {
        let r = intersect_surfaces(
            &built.surfaces[*a],
            &built.surfaces[*b],
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
        let (got, curves) = surface_type(&r);
        let got = if touching(&r) {
            (format!("tangent {got}"), curves.len())
        } else if curves.is_empty() && !r.points().is_empty() {
            (got, r.points().len())
        } else {
            (got, curves.len())
        };
        assert_eq!(
            (got.0.as_str(), got.1),
            (*kind, *count),
            "{a} vs {b}: {r:?}"
        );
    }
}

/// What Arris says about every pair of `geom/c3-cylinder-pairs`, by name:
/// the loops each quartic was built with, fitted and closed. The oracle
/// walks a loop in one line or in two, so its count says nothing, and
/// the comparison above holds its points to these curves.
#[test]
fn the_c3_cylinder_pairs_meet_in_the_loops_they_were_built_with() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c3-cylinder-pairs")
        .expect("geom/c3-cylinder-pairs");
    let built = build(&f);
    let cases: &[(&str, &str, usize)] = &[
        ("cyl", "cross_90", 2),
        ("cyl", "cross_40", 2),
        ("cyl", "cross_larger", 2),
        ("cyl", "skew_out", 1),
        ("cyl", "skew_in", 2),
        ("cyl", "skew_55", 1),
        ("skew_out", "cyl", 1),
    ];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, loops) in cases {
        let r = intersect_surfaces(
            &built.surfaces[*a],
            &built.surfaces[*b],
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
        assert!(r.points().is_empty(), "{a} vs {b}: {r:?}");
        assert_eq!(r.curves().len(), *loops, "{a} vs {b}");
        for m in r.curves() {
            assert_eq!(m.kind, MeetKind::Crossing, "{a} vs {b}");
            assert!(
                matches!(&m.curve, Curve::Nurbs(c) if c.period().is_some()),
                "{a} vs {b}: not a periodic fit"
            );
        }
    }
}

/// What Arris says about every pair of `geom/c3-quadric-pairs`, by name: a
/// plane off a cone's axis in the conic it was built for — the parabola
/// and the hyperbola's branches exact rational quadratics — or through the
/// apex in its point, two rulings or one touching ruling; the traced pairs
/// in the loops they were built with, fitted and closed.
#[test]
fn the_c3_quadric_pairs_classify_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c3-quadric-pairs")
        .expect("geom/c3-quadric-pairs");
    let built = build(&f);
    let cases: &[(&str, &str, &str, usize)] = &[
        ("steep", "cone", "ellipse", 1),
        ("parabolic", "cone", "NURBS", 1),
        ("parallel", "cone", "NURBS", 2),
        ("apex_steep", "cone", "point", 1),
        ("apex_shallow", "cone", "line", 2),
        ("apex_touch", "cone", "tangent line", 1),
        ("cone", "pipe", "loops", 2),
        ("cone", "ball", "loops", 1),
        ("ball", "post", "loops", 1),
        ("cone", "spike", "loops", 2),
        ("cone", "parallel", "NURBS", 2),
        ("spike", "cone", "loops", 2),
    ];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, kind, count) in cases {
        let r = intersect_surfaces(
            &built.surfaces[*a],
            &built.surfaces[*b],
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
        let (got, curves) = surface_type(&r);
        let loops = curves
            .iter()
            .all(|c| matches!(c, Curve::Nurbs(n) if n.period().is_some()));
        let got = if touching(&r) {
            (format!("tangent {got}"), curves.len())
        } else if curves.is_empty() {
            (got, r.points().len())
        } else if loops {
            ("loops".to_owned(), curves.len())
        } else {
            (got, curves.len())
        };
        assert_eq!(
            (got.0.as_str(), got.1),
            (*kind, *count),
            "{a} vs {b}: {r:?}"
        );
    }
}

/// What Arris says about every pair of `geom/c3-torus-pairs`, by name: the
/// branches each section was built with, fitted — loops, or the arms of a
/// figure eight and of the elbow ending at their singular points — and the
/// elbow's shared tube circle exact and touching beside them (ADR-0019).
/// The oracle walks a loop in one line or in two, and drops the elbow's
/// tangent circle, so its counts say nothing; the comparison above holds
/// its polished points to these curves.
#[test]
fn the_c3_torus_pairs_classify_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c3-torus-pairs")
        .expect("geom/c3-torus-pairs");
    let built = build(&f);
    // (a, b, loops, open arms, exact circles, points)
    let cases: &[(&str, &str, usize, usize, usize, usize)] = &[
        ("ring", "hole", 2, 0, 0, 0),
        ("ring", "eight", 0, 2, 0, 1),
        ("ring", "tube", 1, 0, 0, 0),
        ("ring", "oblique", 2, 0, 0, 0),
        ("ring", "drill", 2, 0, 0, 0),
        ("ring", "elbow", 0, 2, 1, 2),
        ("ring", "cone", 2, 0, 0, 0),
        ("ring", "ball", 2, 0, 0, 0),
        ("ring", "link", 2, 0, 0, 0),
        ("ring", "hoop", 2, 0, 0, 0),
        ("hole", "ring", 2, 0, 0, 0),
        ("link", "ring", 2, 0, 0, 0),
    ];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, loops, arms, circles, points) in cases {
        let r = intersect_surfaces(
            &built.surfaces[*a],
            &built.surfaces[*b],
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
        let mut got = (0, 0, 0, r.points().len());
        for m in r.curves() {
            match (&m.curve, m.kind) {
                (Curve::Nurbs(c), MeetKind::Crossing) if c.period().is_some() => got.0 += 1,
                (Curve::Nurbs(_), MeetKind::Crossing) => got.1 += 1,
                (Curve::Circle { .. }, MeetKind::Touch) => got.2 += 1,
                _ => panic!("{a} vs {b}: {m:?}"),
            }
        }
        assert_eq!(got, (*loops, *arms, *circles, *points), "{a} vs {b}: {r:?}");
    }
}

/// How many times each curve of `geom/c3-nurbs-hits` crosses each surface,
/// by name: what the oracle's general intersector found and what Arris's
/// spans in the surfaces' implicit forms find, every hit a crossing.
#[test]
fn the_c3_nurbs_hits_cross_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c3-nurbs-hits")
        .expect("geom/c3-nurbs-hits");
    let built = build(&f);
    let cases: &[(&str, &str, usize)] = &[
        ("ring", "floor", 2),
        ("ring", "wall", 4),
        ("ring", "cone", 4),
        ("ring", "sphere", 2),
        ("ring", "torus", 2),
        ("wave", "floor", 1),
        ("wave", "wall", 2),
        ("wave", "cone", 2),
        ("wave", "sphere", 2),
        ("wave", "torus", 2),
        ("skein", "floor", 2),
        ("skein", "wall", 4),
        ("skein", "cone", 4),
        ("skein", "sphere", 2),
        ("skein", "torus", 1),
    ];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, count) in cases {
        let r = intersect_curve_surface(
            &built.curves[*a],
            &built.surfaces[*b],
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
        let CurveSurfaceIntersection::Points(hits) = &r else {
            panic!("{a} vs {b}: {r:?}")
        };
        assert_eq!(hits.len(), *count, "{a} vs {b}: {hits:?}");
        assert!(hits.iter().all(|h| !h.tangent), "{a} vs {b}: {hits:?}");
    }
}

/// A closed B-spline that is not periodic, met at its join: one hit there,
/// at the start parameter, as the oracle's general intersector counts it,
/// and the pairs' counts as it counts them (ADR-0025).
#[test]
fn a_closed_curve_meets_a_surface_at_its_join_once() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c4-closed-curve-hits")
        .expect("geom/c4-closed-curve-hits");
    let built = build(&f);
    let lo = built.curves["loop"].domain().lo();
    // (surface, hits, one of them at the join)
    let cases: &[(&str, usize, bool)] =
        &[("sheet", 2, true), ("drum", 2, true), ("shelf", 2, false)];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair is pinned here"
    );
    for ((b, count, at_join), oracle) in cases.iter().zip(&f.expected.pairs) {
        assert_eq!(oracle.hits.len(), *count, "{b}: the oracle's count");
        let r = intersect_curve_surface(
            &built.curves["loop"],
            &built.surfaces[*b],
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("loop vs {b}: {e}"));
        let CurveSurfaceIntersection::Points(hits) = &r else {
            panic!("loop vs {b}: {r:?}")
        };
        assert_eq!(hits.len(), *count, "loop vs {b}: {hits:?}");
        assert!(hits.iter().all(|h| !h.tangent), "loop vs {b}: {hits:?}");
        assert_eq!(
            hits.iter().any(|h| h.t == lo),
            *at_join,
            "loop vs {b}: {hits:?}"
        );
    }
}

/// What Arris says about every pair of `geom/c3-nurbs-crossings`, by name:
/// the crossings and touches each was built with. The oracle comparison
/// is one-sided at a touch, so the ring's tangent is pinned here.
#[test]
fn the_c3_nurbs_crossings_classify_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c3-nurbs-crossings")
        .expect("geom/c3-nurbs-crossings");
    let built = build(&f);
    // (a, b, crossings, touches)
    let cases: &[(&str, &str, usize, usize)] = &[
        ("ring", "pierce", 1, 0),
        ("ring", "chord", 2, 0),
        ("ring", "tangent", 0, 1),
        ("ring", "lifted", 0, 0),
        ("ring", "hoop", 1, 0),
        ("ring", "oval", 1, 0),
        ("ring", "round", 4, 0),
        ("wave", "wave_line", 1, 0),
        ("wave", "wave_miss", 0, 0),
        ("wave", "wave_hoop", 1, 0),
        ("wave", "wave_oval", 1, 0),
        ("skein", "skein_line", 1, 0),
        ("skein", "skein_hoop", 1, 0),
        ("hoop", "ring", 1, 0),
        ("wave_line", "wave", 1, 0),
    ];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, crossings, touches) in cases {
        let r = intersect_curves(
            &built.curves[*a],
            &built.curves[*b],
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
        let CurveIntersection::Points(hits) = r else {
            panic!("{a} vs {b}: {r:?}");
        };
        let tangent = hits.iter().filter(|h| h.tangent).count();
        assert_eq!(
            (hits.len() - tangent, tangent),
            (*crossings, *touches),
            "{a} vs {b}: {hits:?}"
        );
    }
}

/// What Arris says about every pair of `geom/c3-conic-hits`, by name:
/// the crossings and touches each conic was built with. The oracle
/// comparison above is one-sided at a touch — it reports none, one or
/// several points there, having no tolerance of its own — so every
/// tangency is pinned here, where it cannot pass vacuously.
#[test]
fn the_c3_conic_hits_classify_as_built() {
    let f = geometry_fixtures()
        .into_iter()
        .find(|f| f.name == "geom/c3-conic-hits")
        .expect("geom/c3-conic-hits");
    let built = build(&f);
    // (a, b, crossings, touches)
    let cases: &[(&str, &str, usize, usize)] = &[
        ("cone_cross", "cone", 4, 0),
        ("cone_touch", "cone", 0, 2),
        ("ball_cross", "ball", 4, 0),
        ("ball_touch", "ball", 0, 1),
        ("oval_cross", "oval", 4, 0),
        ("oval_touch", "oval", 0, 2),
        ("oval_slice", "oval", 4, 0),
        ("ring_cross", "ring", 8, 0),
        ("ring_touch", "ring", 0, 2),
        ("ring_drill", "ring", 2, 0),
        ("flat_oval", "flat_round", 4, 0),
        ("flat_round", "flat_oval", 4, 0),
        ("flat_oval", "flat_minor", 0, 2),
        ("flat_oval", "flat_turned", 4, 0),
    ];
    assert_eq!(
        cases.len(),
        f.recipe.pairs.len(),
        "every pair of the fixture is pinned here"
    );
    for (a, b, crossings, touches) in cases {
        let (found, tangent) = match built.surfaces.get(*b) {
            Some(s) => {
                let r = intersect_curve_surface(
                    &built.curves[*a],
                    s,
                    tol(),
                    &mut arris_math::Meter::default(),
                )
                .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
                let CurveSurfaceIntersection::Points(hits) = r else {
                    panic!("{a} vs {b}: {r:?}")
                };
                let tangent = hits.iter().filter(|h| h.tangent).count();
                (hits.len(), tangent)
            }
            None => {
                let r = intersect_curves(
                    &built.curves[*a],
                    &built.curves[*b],
                    tol(),
                    &mut arris_math::Meter::default(),
                )
                .unwrap_or_else(|e| panic!("{a} vs {b}: {e}"));
                let CurveIntersection::Points(hits) = r else {
                    panic!("{a} vs {b}: {r:?}")
                };
                let tangent = hits.iter().filter(|h| h.tangent).count();
                (hits.len(), tangent)
            }
        };
        assert_eq!(
            (found - tangent, tangent),
            (*crossings, *touches),
            "{a} vs {b}"
        );
    }
}
