//! The section of a NURBS surface with an analytic one, traced in the
//! NURBS surface's own parameters (ADR-0061): the exact twins of analytic
//! surfaces held to their closed forms, the branches ended on the edges
//! of the domain, and at random poses every sample on both surfaces,
//! every edge end on its edge line, the argument order not reaching the
//! result and a dense scan of the domain finding no point of the other
//! surface away from every branch.

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_debug::prop::{self, check};
use arris_geom::{
    BranchEnd, Curve, GeomError, NurbsSurface, SECTION_FIT_FRACTION, SectionBranch, SectionFault,
    SectionTrace, Surface, SurfaceIntersection, fit_branch, intersect_surfaces, trace_section,
    trace_torus,
};
use arris_math::{Aabb, Frame, Interval, Meter, Point3, Precision, Tolerance, Vec3};
use proptest::prelude::*;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn frame(origin: [f64; 3], z: [f64; 3]) -> Frame {
    Frame::from_z(Point3::from(origin), Vec3::from(z)).unwrap()
}

/// What `trace_section` takes and a NURBS section ignores.
fn anywhere() -> Aabb {
    Aabb {
        min: [-1e3; 3],
        max: [1e3; 3],
    }
}

fn trace(a: &Surface, b: &Surface) -> Result<SectionTrace, GeomError> {
    trace_section(a, b, &anywhere(), tol(), &mut Meter::default())
}

fn distance(surface: &Surface, p: Point3) -> f64 {
    surface.project(p).map_or(f64::INFINITY, |on| on.distance)
}

/// `n` points along a branch, both ends included on an open one and the
/// start once on a closed one.
fn samples(branch: &SectionBranch, n: usize) -> Vec<Point3> {
    let domain = branch.domain();
    let steps = if branch.is_closed() { n } else { n - 1 };
    (0..n)
        .map(|i| branch.point(domain.lerp(i as f64 / steps as f64)))
        .collect()
}

/// The twin of `surface` over `bounds`, as a `Surface`.
fn twin(surface: &Surface, bounds: [Interval; 2]) -> (Surface, NurbsSurface) {
    let nurbs = surface.to_nurbs(bounds).unwrap();
    (Surface::Nurbs(nurbs.clone()), nurbs)
}

fn unit_cylinder() -> Surface {
    Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    }
}

/// The plane `z = slope·x + height`.
fn tilted(slope: f64, height: f64) -> Surface {
    Surface::Plane {
        frame: frame([0.0, 0.0, height], [-slope, 0.0, 1.0]),
    }
}

/// Every branch's end on an edge of `nurbs`'s domain where it is
/// clipped: `uv` there on the edge line to rounding and `point` the
/// surface's point at it. Returns how many ends were clipped.
fn clipped_ends_on_edges(trace: &SectionTrace, nurbs: &NurbsSurface) -> usize {
    let [du, dv] = nurbs.domain();
    let mut clipped = 0;
    for branch in trace.branches() {
        let Some(ends) = branch.ends() else {
            continue;
        };
        let domain = branch.domain();
        for (end, t) in ends.iter().zip([domain.lo(), domain.hi()]) {
            if *end != BranchEnd::Clipped {
                continue;
            }
            clipped += 1;
            let uv = branch.uv(t).unwrap();
            let on_edge = [du.lo(), du.hi()].iter().any(|e| (uv.x - e).abs() <= 1e-12)
                || [dv.lo(), dv.hi()].iter().any(|e| (uv.y - e).abs() <= 1e-12);
            assert!(on_edge, "{uv:?} is on no edge of {du:?} × {dv:?}");
            let p = nurbs.eval(uv.x, uv.y).point;
            assert!(
                (p - branch.point(t)).norm() <= 1e-12,
                "{p} vs {}",
                branch.point(t)
            );
        }
    }
    clipped
}

/// The largest distance of any sample of any branch from either surface.
fn worst_off(trace: &SectionTrace, a: &Surface, b: &Surface) -> f64 {
    (trace.branches().iter())
        .flat_map(|branch| samples(branch, 257))
        .chain(trace.points().iter().map(|p| p.point))
        .map(|p| distance(a, p).max(distance(b, p)))
        .fold(0.0, f64::max)
}

/// `uv` of every branch is the NURBS surface's own parameters of its
/// point.
fn uv_is_the_surfaces(trace: &SectionTrace, nurbs: &NurbsSurface) {
    for branch in trace.branches() {
        for k in 0..=16 {
            let t = branch.domain().lerp(k as f64 / 16.0);
            let uv = branch.uv(t).unwrap();
            let p = nurbs.eval(uv.x, uv.y).point;
            assert!((p - branch.point(t)).norm() <= 1e-12, "at {t}: {p}");
        }
    }
}

/// The closed form's curve: the one curve `intersect_surfaces` gives.
fn closed_form(a: &Surface, b: &Surface) -> Curve {
    let within = anywhere();
    match intersect_surfaces(a, b, &within, tol(), &mut Meter::default()).unwrap() {
        SurfaceIntersection::Meets { curves, .. } if curves.len() == 1 => curves[0].curve.clone(),
        other => panic!("no one curve: {other:?}"),
    }
}

/// Each branch fitted as the intersector fits it is within the fit
/// fraction of the closed form's curve at every sample, and every point
/// of the closed form's curve inside `inside` is within it of a fit.
fn fits_the_closed_form(trace: &SectionTrace, curve: &Curve, inside: impl Fn(Point3) -> bool) {
    let bound = SECTION_FIT_FRACTION * tol().linear;
    let fits: Vec<Curve> = (trace.branches().iter())
        .map(|b| fit_branch(b, b.domain(), tol(), &mut Meter::default()).unwrap())
        .collect();
    for fit in &fits {
        let domain = fit.domain();
        for k in 0..=512 {
            let p = fit.point(domain.lerp(k as f64 / 512.0));
            let off = curve.project(p).unwrap().distance;
            assert!(off <= bound, "the fit is {off} off the closed form at {p}");
        }
    }
    let domain = curve.domain();
    for k in 0..512 {
        let p = curve.point(domain.lerp(k as f64 / 512.0));
        if !inside(p) {
            continue;
        }
        let near = (fits.iter())
            .map(|f| f.project(p).map_or(f64::INFINITY, |on| on.distance))
            .fold(f64::INFINITY, f64::min);
        assert!(
            near <= bound,
            "{p} of the closed form is {near} from every fit"
        );
    }
}

#[test]
fn a_full_turn_of_a_cylinder_twin_is_cut_in_one_closed_ellipse() {
    let wall = unit_cylinder();
    let (nurbs, twin) = twin(&wall, [Interval::TURN, Interval::new(-2.0, 2.0).unwrap()]);
    let plane = tilted(0.3, 0.1);
    let trace = trace(&nurbs, &plane).unwrap();
    assert_eq!(trace.branches().len(), 1);
    assert!(trace.branches()[0].is_closed());
    assert!(trace.points().is_empty());
    assert!(worst_off(&trace, &nurbs, &plane) < 1e-12);
    uv_is_the_surfaces(&trace, &twin);
    fits_the_closed_form(&trace, &closed_form(&wall, &plane), |_| true);
}

#[test]
fn half_a_cylinder_twin_ends_its_ellipse_on_the_edges_of_u() {
    let wall = unit_cylinder();
    let (nurbs, twin) = twin(
        &wall,
        [
            Interval::new(0.0, PI).unwrap(),
            Interval::new(-2.0, 2.0).unwrap(),
        ],
    );
    let plane = tilted(0.3, 0.1);
    let trace = trace(&nurbs, &plane).unwrap();
    assert_eq!(trace.branches().len(), 1, "{trace:?}");
    let branch = &trace.branches()[0];
    assert_eq!(branch.ends(), Some([BranchEnd::Clipped; 2]));
    assert_eq!(clipped_ends_on_edges(&trace, &twin), 2);
    assert!(worst_off(&trace, &nurbs, &plane) < 1e-12);
    uv_is_the_surfaces(&trace, &twin);
    fits_the_closed_form(&trace, &closed_form(&wall, &plane), |p| p.y >= 0.0);
}

#[test]
fn a_steep_plane_leaves_a_short_cylinder_twin_through_its_rims() {
    let wall = unit_cylinder();
    let (nurbs, twin) = twin(&wall, [Interval::TURN, Interval::new(-0.2, 0.3).unwrap()]);
    // `z = x`: the ellipse leaves through the top rim and the bottom rim
    // twice each, in two arcs.
    let plane = tilted(1.0, 0.0);
    let trace = trace(&nurbs, &plane).unwrap();
    assert_eq!(trace.branches().len(), 2, "{trace:?}");
    assert_eq!(clipped_ends_on_edges(&trace, &twin), 4);
    assert!(worst_off(&trace, &nurbs, &plane) < 1e-12);
    uv_is_the_surfaces(&trace, &twin);
    fits_the_closed_form(&trace, &closed_form(&wall, &plane), |p| {
        (-0.2..=0.3).contains(&p.z)
    });
}

#[test]
fn a_hemisphere_twin_is_cut_square_to_its_equator() {
    // On the equator the section of a vertical plane runs along the
    // meridian, parallel to `v`: a turning point on the edge of the
    // domain, where the branch ends.
    let globe = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let (nurbs, twin) = twin(
        &globe,
        [Interval::TURN, Interval::new(0.0, FRAC_PI_2).unwrap()],
    );
    let plane = Surface::Plane {
        frame: frame([0.7, 0.0, 0.0], [1.0, 0.0, 0.0]),
    };
    let trace = trace(&nurbs, &plane).unwrap();
    assert_eq!(trace.branches().len(), 1, "{trace:?}");
    assert_eq!(clipped_ends_on_edges(&trace, &twin), 2);
    assert!(worst_off(&trace, &nurbs, &plane) < 1e-12);
    uv_is_the_surfaces(&trace, &twin);
    let y = (4.0f64 - 0.49).sqrt();
    let ends: Vec<Point3> = (trace.branches().iter())
        .flat_map(|b| [b.point(b.domain().lo()), b.point(b.domain().hi())])
        .collect();
    for e in [Point3::new(0.7, y, 0.0), Point3::new(0.7, -y, 0.0)] {
        assert!(
            ends.iter().any(|p| (p - e).norm() < 1e-12),
            "{e} not in {ends:?}"
        );
    }
}

#[test]
fn a_torus_twin_traces_what_the_torus_does() {
    let ring = Surface::Torus {
        frame: Frame::world(),
        major_radius: 2.0,
        minor_radius: 0.5,
    };
    let (nurbs, twin) = twin(&ring, [Interval::TURN, Interval::TURN]);
    for d in [1.0, 2.0, 0.3] {
        let plane = Surface::Plane {
            frame: frame([d, 0.0, 0.0], [1.0, 0.0, 0.0]),
        };
        let traced = trace(&nurbs, &plane).unwrap();
        let exact = trace_torus(&ring, &plane, tol(), &mut Meter::default()).unwrap();
        assert_eq!(traced.branches().len(), exact.branches().len(), "{d}");
        assert!(traced.branches().iter().all(SectionBranch::is_closed));
        assert!(worst_off(&traced, &nurbs, &plane) < 1e-12);
        uv_is_the_surfaces(&traced, &twin);
        // Every point of the torus's trace is on the twin's, to the sag
        // of a polyline through a thousand of its points.
        let polylines: Vec<Vec<Point3>> = (traced.branches().iter())
            .map(|b| samples(b, 1024))
            .collect();
        for branch in exact.branches() {
            for p in samples(branch, 64) {
                let near = (polylines.iter())
                    .flat_map(|line| {
                        let closing = [line[line.len() - 1], line[0]];
                        line.windows(2)
                            .map(|w| segment_distance(p, w[0], w[1]))
                            .chain([segment_distance(p, closing[0], closing[1])])
                    })
                    .fold(f64::INFINITY, f64::min);
                assert!(near < 1e-4, "{d}: {p} is {near} from the twin's trace");
            }
        }
    }
}

#[test]
fn two_nurbs_surfaces_are_unsupported() {
    let (a, _) = twin(&unit_cylinder(), [Interval::TURN, Interval::UNIT]);
    let (b, _) = twin(&tilted(0.3, 0.1), [Interval::UNIT, Interval::UNIT]);
    assert!(matches!(trace(&a, &b), Err(GeomError::Unsupported { .. })));
}

/// A NURBS surface to walk, in a random pose, and whether it is a twin:
/// the twin of an analytic surface over part of its domain — or over a
/// whole turn of `u`, which closes, or of both of a torus's — or a random
/// rational B-spline.
fn patch() -> impl Strategy<Value = (NurbsSurface, bool)> {
    let twin = (
        prop_oneof![
            prop::geom::cylinder(),
            prop::geom::cone(),
            prop::geom::sphere(),
            prop::geom::torus(),
        ],
        prop::finite_f64(0.0..=TAU),
        prop::finite_f64(0.5..=TAU + 1.0),
        prop::finite_f64(0.1..=0.9),
        prop::finite_f64(0.2..=1.2),
    )
        .prop_filter_map("no twin", |(surface, u0, du, v0, dv)| {
            let [su, sv] = surface.domain();
            let height = |t: f64| 4.0 * (t - 0.5) * size(&surface);
            // Past a whole turn is a whole turn.
            let u = if du >= TAU {
                Interval::TURN
            } else {
                Interval::new(u0, u0 + du.min(su.length())).ok()?
            };
            let v = match (sv.is_bounded(), dv > 1.0) {
                (true, true) => sv,
                (true, false) => {
                    Interval::new(sv.lerp(v0 * (1.0 - dv)), sv.lerp(v0 * (1.0 - dv) + dv)).ok()?
                }
                (false, _) => {
                    let dv = dv.min(1.0);
                    Interval::new(height(v0 * (1.0 - dv)), height(v0 * (1.0 - dv) + dv)).ok()?
                }
            };
            Some((surface.to_nurbs([u, v]).ok()?, true))
        });
    prop_oneof![
        3 => twin,
        1 => (prop::geom::nurbs_surface(), prop::pose())
            .prop_map(|(nurbs, pose)| (nurbs.transformed(&pose), false)),
    ]
}

/// Half the diagonal of the box of `nurbs`'s control points.
fn extent(nurbs: &NurbsSurface) -> f64 {
    let points = nurbs.control_points();
    let (lo, hi) = points.iter().fold(
        (Vec3::repeat(f64::INFINITY), Vec3::repeat(f64::NEG_INFINITY)),
        |(lo, hi), p| (lo.inf(&p.coords), hi.sup(&p.coords)),
    );
    0.5 * (hi - lo).norm()
}

fn size(surface: &Surface) -> f64 {
    match *surface {
        Surface::Cylinder { radius, .. }
        | Surface::Sphere { radius, .. }
        | Surface::Cone { radius, .. } => radius,
        Surface::EllipticCylinder { minor_radius, .. } => minor_radius,
        Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => major_radius + minor_radius,
        Surface::Plane { .. } | Surface::Nurbs(_) => 1.0,
    }
}

/// `surface` with its frame's origin moved to `origin`.
fn placed(surface: &Surface, origin: Point3) -> Surface {
    let mut out = surface.clone();
    match &mut out {
        Surface::Plane { frame }
        | Surface::Cylinder { frame, .. }
        | Surface::EllipticCylinder { frame, .. }
        | Surface::Cone { frame, .. }
        | Surface::Sphere { frame, .. }
        | Surface::Torus { frame, .. } => *frame = frame.with_origin(origin),
        Surface::Nurbs(_) => {}
    }
    out
}

/// How many of the other surface's sizes a NURBS surface may reach
/// across. The other surface's polynomial over the NURBS surface's points
/// rounds with their distance to the power of its degree, and the
/// section's turning points a hundred times beyond `f64` are refused, as
/// a torus's are a hundred tubes across (ADR-0061, the floor): the torus
/// tracer's properties hold `R/r` to a hundred too.
const REACH: f64 = 100.0;

/// A NURBS surface and an analytic surface placed where it meets it:
/// its origin within its own size of a point of the NURBS surface. A
/// NURBS surface more than [`REACH`] of the other's sizes across is
/// scaled down about that point to it; against a plane, which has no
/// size and a polynomial of degree one, it is not.
fn meeting(
    other: impl Strategy<Value = Surface>,
) -> impl Strategy<Value = (NurbsSurface, bool, Surface)> {
    (
        patch(),
        other,
        prop::finite_f64(0.0..=1.0),
        prop::finite_f64(0.0..=1.0),
        prop::unit_vec3(),
        prop::finite_f64(0.0..=0.9),
    )
        .prop_map(|((nurbs, twin), other, s, t, direction, reach)| {
            let [du, dv] = nurbs.domain();
            let on = nurbs.eval(du.lerp(s), dv.lerp(t)).point;
            let ratio = extent(&nurbs) / (REACH * size(&other));
            let nurbs = if ratio > 1.0 && !matches!(other, Surface::Plane { .. }) {
                let points: Vec<Point3> = (nurbs.control_points().iter())
                    .map(|p| on + (p - on) / ratio)
                    .collect();
                let [ku, kv] = nurbs.knots();
                let knots = [ku.to_vec(), kv.to_vec()];
                let weights = nurbs.weights().to_vec();
                NurbsSurface::new(nurbs.degree(), knots, points, weights).unwrap_or(nurbs)
            } else {
                nurbs
            };
            let at = on + direction.into_inner() * (reach * size(&other).min(extent(&nurbs)));
            (nurbs, twin, placed(&other, at))
        })
}

/// How many cells a side the scan of the domain takes.
const SCAN: usize = 96;

/// Every cell of a grid over `nurbs`'s domain in which the other
/// surface's side changes is near the trace.
fn assert_complete(trace: &SectionTrace, nurbs: &NurbsSurface, other: &Surface) {
    let [du, dv] = nurbs.domain();
    let at = |i: usize, j: usize| {
        (nurbs.eval(
            du.lerp(i as f64 / SCAN as f64),
            dv.lerp(j as f64 / SCAN as f64),
        ))
        .point
    };
    let side = |p: Point3| {
        let on = other.project(p).unwrap();
        (p - on.point).dot(
            &other
                .normal(on.uv.x, on.uv.y)
                .map_or(Vec3::zeros(), |n| n.into_inner()),
        )
    };
    let polylines: Vec<Vec<Point3>> = (trace.branches().iter())
        .map(|b| samples(b, 4 * SCAN + 1))
        .collect();
    let to_trace = |p: Point3| {
        (polylines.iter())
            .flat_map(|line| line.windows(2).map(|w| segment_distance(p, w[0], w[1])))
            .chain(trace.points().iter().map(|q| (q.point - p).norm()))
            .fold(f64::INFINITY, f64::min)
    };
    let grid: Vec<Vec<f64>> = (0..=SCAN)
        .map(|i| (0..=SCAN).map(|j| side(at(i, j))).collect())
        .collect();
    for i in 0..SCAN {
        for j in 0..SCAN {
            let corners = [
                grid[i][j],
                grid[i + 1][j],
                grid[i][j + 1],
                grid[i + 1][j + 1],
            ];
            if corners.iter().all(|c| *c > 0.0) || corners.iter().all(|c| *c < 0.0) {
                continue;
            }
            let across =
                (at(i, j) - at(i + 1, j + 1)).norm() + (at(i + 1, j) - at(i, j + 1)).norm();
            let d = to_trace(at(i, j));
            assert!(
                d <= 2.0 * across + tol().linear,
                "the section through cell ({i}, {j}) is {d} from the trace"
            );
        }
    }
}

fn segment_distance(p: Point3, a: Point3, b: Point3) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(&ab) / ab.norm_squared()).clamp(0.0, 1.0);
    if t.is_finite() {
        (p - (a + ab * t)).norm()
    } else {
        (p - a).norm()
    }
}

/// Whether `nurbs` is only continuous across a knot of `v` inside its
/// domain: a knot as many times over as the degree, where the section
/// may turn at the kink and the tracer refuses (ADR-0061).
fn kinked(nurbs: &NurbsSurface) -> bool {
    let q = nurbs.degree()[1];
    let knots = &nurbs.knots()[1];
    let [_, dv] = nurbs.domain();
    knots
        .iter()
        .filter(|&&k| dv.lo() < k && k < dv.hi())
        .any(|&k| knots.iter().filter(|&&x| x == k).count() >= q)
}

/// What every trace owes: each sample on both surfaces, `uv` the NURBS
/// surface's own parameters of the point, every clipped end on an edge
/// line, the argument order not reaching the result, and nothing of the
/// section missed. Every pose of a twin is traced; a random B-spline's
/// may be refused by name, never answered wrong.
fn holds((nurbs, twin, other): (NurbsSurface, bool, Surface)) -> Result<(), TestCaseError> {
    let walked = Surface::Nurbs(nurbs.clone());
    let trace = match trace(&walked, &other) {
        Ok(trace) => trace,
        // A turn at a kink is not followed, and refused by name.
        Err(GeomError::DegenerateSection {
            fault: SectionFault::UnresolvedTurning,
            ..
        }) if kinked(&nurbs) => return Ok(()),
        // A random B-spline may fold, or reach far past the other
        // surface's size: a pose `f64` does not decide is refused by
        // name. A twin is regular, and never refused.
        Err(GeomError::DegenerateSection { .. }) if !twin => return Ok(()),
        Err(e) => return Err(TestCaseError::fail(format!("refused: {e}"))),
    };
    let bound = |p: Point3| {
        let scale = 1.0 + p.coords.norm();
        let singular = if trace.points().is_empty() {
            0.0
        } else {
            tol().linear
        };
        singular + 1e-11 * scale * scale
    };
    // On the other surface by its projection; on the NURBS surface by
    // `uv`, its point there — a B-spline's projection need not find a
    // point of a kinked surface (`tests/nurbs_project.rs`).
    for p in trace.branches().iter().flat_map(|br| samples(br, 65)) {
        prop_assert!(p.coords.iter().all(|x| x.is_finite()));
        let off = distance(&other, p);
        prop_assert!(off <= bound(p), "{p} is {off} off the other surface");
    }
    for branch in trace.branches() {
        for k in 0..=8 {
            let t = branch.domain().lerp(k as f64 / 8.0);
            let uv = branch.uv(t).unwrap();
            let p = branch.point(t);
            let off = (nurbs.eval(uv.x, uv.y).point - p).norm();
            prop_assert!(off <= bound(p), "uv {uv:?} is {off} off {p}");
        }
    }
    clipped_ends_on_edges(&trace, &nurbs);
    let swapped = trace_section(&other, &walked, &anywhere(), tol(), &mut Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    prop_assert_eq!(trace.points(), swapped.points());
    prop_assert_eq!(trace.branches().len(), swapped.branches().len());
    for (x, y) in trace.branches().iter().zip(swapped.branches()) {
        prop_assert_eq!(samples(x, 9), samples(y, 9));
    }
    assert_complete(&trace, &nurbs, &other);
    Ok(())
}

#[test]
fn random_nurbs_and_plane_pairs_trace() {
    check(meeting(prop::geom::plane()), holds);
}

#[test]
fn random_nurbs_and_cylinder_pairs_trace() {
    check(meeting(prop::geom::cylinder()), holds);
}

#[test]
fn random_nurbs_and_cone_pairs_trace() {
    check(meeting(prop::geom::cone()), holds);
}

#[test]
fn random_nurbs_and_sphere_pairs_trace() {
    check(meeting(prop::geom::sphere()), holds);
}

#[test]
fn random_nurbs_and_torus_pairs_trace() {
    check(meeting(prop::geom::torus()), holds);
}
