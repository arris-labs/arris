//! A NURBS surface's fitted offset (`NurbsSurface::offset_fitted`,
//! ADR-0062): the exact `to_nurbs` twins of the analytic kinds offset to
//! within the tolerance of `Surface::offset` of the original, a bump fits
//! short of its least radius and folds past it, and a pole is refused.

use arris_debug::prop::{finite_f64, pose};
use arris_geom::{GeomError, NurbsSurface, Surface};
use arris_math::{Frame, Interval, Isometry, Meter, Point3, Vec3};
use proptest::prelude::*;

/// The tolerance each offset is fitted to.
const TOL: f64 = 1e-6;

fn range(lo: f64, hi: f64) -> Interval {
    Interval::new(lo, hi).unwrap()
}

/// An analytic surface in the world frame, the box of its twin, and the
/// offsets asked of it both ways.
struct Case {
    name: &'static str,
    surface: Surface,
    over: [Interval; 2],
    distances: [f64; 2],
}

fn cases() -> Vec<Case> {
    let world = Frame::world();
    vec![
        Case {
            name: "plane",
            surface: Surface::Plane { frame: world },
            over: [range(-1.0, 2.0), range(0.0, 3.0)],
            distances: [0.7, -0.7],
        },
        Case {
            name: "cylinder",
            surface: Surface::Cylinder {
                frame: world,
                radius: 2.0,
            },
            over: [range(0.0, 2.0), range(0.0, 2.0)],
            distances: [0.5, -0.5],
        },
        Case {
            name: "sphere",
            surface: Surface::Sphere {
                frame: world,
                radius: 3.0,
            },
            over: [range(0.0, 1.5), range(-0.8, 0.8)],
            distances: [1.0, -1.0],
        },
        Case {
            name: "torus",
            surface: Surface::Torus {
                frame: world,
                major_radius: 5.0,
                minor_radius: 1.5,
            },
            over: [range(0.0, 2.0), range(0.0, 2.0)],
            distances: [0.5, -0.5],
        },
    ]
}

/// How far `p`, in the world frame, is from the analytic `exact`.
fn distance_to(exact: &Surface, p: Point3) -> f64 {
    match *exact {
        Surface::Plane { frame } => (p - frame.origin()).dot(&frame.z().into_inner()).abs(),
        Surface::Cylinder { radius, .. } => (p.coords.xy().norm() - radius).abs(),
        Surface::Sphere { radius, .. } => (p.coords.norm() - radius).abs(),
        Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => {
            let rho = p.coords.xy().norm() - major_radius;
            ((rho * rho + p.z * p.z).sqrt() - minor_radius).abs()
        }
        _ => unreachable!("the cases are these four"),
    }
}

/// Fits `twin`'s offset by `d` over its own domain and holds it to
/// `S + d·N` of the twin at the same parameter and to `exact`, the
/// analytic offset in the world frame, at a grid of parameters off the
/// fit's own checks; `back` takes a point of `twin` to the world frame.
/// The worst distance to `exact`.
fn holds(twin: &NurbsSurface, d: f64, exact: &Surface, back: &Isometry) -> Result<f64, String> {
    let over = twin.domain();
    let fit = twin
        .offset_fitted(d, over, TOL, &mut Meter::default())
        .map_err(|e| e.to_string())?;
    if fit.domain() != over {
        return Err(format!("domain {:?} for {:?}", fit.domain(), over));
    }
    let mut worst: f64 = 0.0;
    const N: usize = 37;
    for i in 0..=N {
        for j in 0..=N {
            let (u, v) = (
                over[0].lerp(i as f64 / N as f64),
                over[1].lerp(j as f64 / N as f64),
            );
            let p = fit.eval(u, v).point;
            let e = twin.eval(u, v);
            let target = e.point + d * twin.normal(u, v).unwrap().into_inner();
            let same = (p - target).norm();
            if same > TOL {
                return Err(format!("({u}, {v}): {same} from S + dN"));
            }
            worst = worst.max(distance_to(exact, back.apply(p)));
        }
    }
    if worst > TOL {
        return Err(format!("{worst} from the exact offset"));
    }
    Ok(worst)
}

#[test]
fn the_twins_offset_to_the_analytic_offset_both_ways() {
    for case in cases() {
        let twin = case.surface.to_nurbs(case.over).unwrap();
        for d in case.distances {
            let exact = case.surface.offset(d).unwrap();
            let worst = holds(&twin, d, &exact, &Isometry::identity())
                .unwrap_or_else(|e| panic!("{} by {d}: {e}", case.name));
            assert!(worst <= TOL, "{} by {d}: {worst}", case.name);
        }
    }
}

#[test]
fn a_twin_past_its_domain_offsets_its_end_spans_carried_on() {
    // A quarter of a cylinder's twin, asked a tenth past each end: the
    // rational end spans carried on are the circle still.
    let wall = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let twin = wall.to_nurbs([range(0.0, 1.5), range(0.0, 1.0)]).unwrap();
    let over = [range(-0.15, 1.65), range(-0.1, 1.1)];
    let fit = twin
        .offset_fitted(-0.5, over, TOL, &mut Meter::default())
        .unwrap();
    assert_eq!(fit.domain(), over);
    for (u, v) in [(-0.15, -0.1), (1.65, 1.1), (-0.1, 0.5), (1.6, 0.05)] {
        let p = fit.eval(u, v).point;
        assert!((p.coords.xy().norm() - 1.5).abs() <= TOL, "({u}, {v})");
    }
}

/// A bicubic bump over the unit square: flat at its rim, rising `h` in
/// the middle.
fn bump(h: f64) -> NurbsSurface {
    let mut points = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let inner = (1..=2).contains(&i) && (1..=2).contains(&j);
            points.push(Point3::new(
                i as f64 / 3.0,
                j as f64 / 3.0,
                if inner { h } else { 0.0 },
            ));
        }
    }
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    NurbsSurface::new([3, 3], [knots.clone(), knots], points, vec![1.0; 16]).unwrap()
}

/// The least principal radius of `s` on the side a negative offset moves
/// to (where it bends away from its normal), over a fine grid.
fn least_radius_below(s: &NurbsSurface) -> f64 {
    let mut least = f64::INFINITY;
    for i in 0..=64 {
        for j in 0..=64 {
            let (u, v) = (i as f64 / 64.0, j as f64 / 64.0);
            let e = s.eval(u, v);
            let n = s.normal(u, v).unwrap().into_inner();
            let (ee, ff, gg) = (e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv));
            let (l, m, nn) = (e.duu.dot(&n), e.duv.dot(&n), e.dvv.dot(&n));
            let det = ee * gg - ff * ff;
            let k = (l * nn - m * m) / det;
            let h = (ee * nn - 2.0 * ff * m + gg * l) / (2.0 * det);
            let low = h - (h * h - k).max(0.0).sqrt();
            if low < 0.0 {
                least = least.min(-1.0 / low);
            }
        }
    }
    least
}

#[test]
fn a_bump_fits_short_of_its_least_radius_and_folds_past_it() {
    let s = bump(0.4);
    let least = least_radius_below(&s);
    assert!(least.is_finite() && least > 0.0, "{least}");
    let mut meter = Meter::default();
    let fit = s
        .offset_fitted(-0.5 * least, s.domain(), TOL, &mut meter)
        .unwrap();
    let spans = [0, 1].map(|k| {
        let mut k: Vec<f64> = fit.knots()[k].to_vec();
        k.dedup();
        k.len() - 1
    });
    eprintln!("bump: least radius {least:.4}, offset at half of it fits on {spans:?} spans");
    // Up, the bump's top bends away: no fold there for any distance it
    // is short of, and the flanks' radius on that side is larger.
    assert!(
        s.offset_fitted(0.5 * least, s.domain(), TOL, &mut Meter::default())
            .is_ok()
    );
    let past = s.offset_fitted(-1.5 * least, s.domain(), TOL, &mut Meter::default());
    match past {
        Err(GeomError::OffsetFolds {
            distance, radius, ..
        }) => {
            assert_eq!(distance, -1.5 * least);
            assert!(radius <= 1.5 * least, "{radius}");
        }
        other => panic!("past the least radius: {other:?}"),
    }
}

#[test]
fn a_pole_in_the_box_is_refused() {
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let twin = ball.to_nurbs(ball.domain()).unwrap();
    let e = twin
        .offset_fitted(0.5, twin.domain(), TOL, &mut Meter::default())
        .unwrap_err();
    assert!(
        matches!(&e, GeomError::Degenerate { reason, .. } if reason.contains("collapsed row")),
        "{e}"
    );
    // Off the poles it offsets.
    let band = [Interval::TURN, range(-1.0, 1.0)];
    let twin = ball.to_nurbs(band).unwrap();
    assert!(
        twin.offset_fitted(0.5, band, TOL, &mut Meter::default())
            .is_ok()
    );
}

#[test]
fn bad_requests_are_named() {
    let s = bump(0.2);
    let d = s.domain();
    let mut meter = Meter::default();
    assert!(matches!(
        s.offset_fitted(f64::NAN, d, TOL, &mut meter),
        Err(GeomError::Degenerate { .. })
    ));
    assert!(matches!(
        s.offset_fitted(0.1, d, 0.0, &mut meter),
        Err(GeomError::Fit(arris_geom::FitError::InvalidTolerance(_)))
    ));
    assert!(matches!(
        s.offset_fitted(0.1, [Interval::REAL, d[1]], TOL, &mut meter),
        Err(GeomError::Degenerate { .. })
    ));
}

#[test]
fn the_fit_is_deterministic() {
    let s = bump(0.3);
    let a = s
        .offset_fitted(-0.05, s.domain(), TOL, &mut Meter::default())
        .unwrap();
    let b = s
        .offset_fitted(-0.05, s.domain(), TOL, &mut Meter::default())
        .unwrap();
    assert_eq!(a, b);
}

arris_debug::prop_shards! {
    /// The twins in random poses, offset by a random distance either way
    /// short of the radius it would collapse: within the tolerance of
    /// `S + d·N` and of the analytic offset.
    the_twins_offset_in_any_pose [shard_0 shard_1 shard_2 shard_3]
        (drawn) = (pose(), 0usize..4, finite_f64(-0.9..=0.9)) => {
            let (motion, which, t) = drawn;
            let case = cases().swap_remove(which);
            let reach = case.distances[0].abs();
            let d = t * reach;
            let twin = case.surface.to_nurbs(case.over).unwrap().transformed(&motion);
            let exact = case.surface.offset(d).unwrap();
            holds(&twin, d, &exact, &motion.inverse())
                .map_err(|e| TestCaseError::fail(format!("{} by {d}: {e}", case.name)))?;
            Ok(())
        }
}

#[test]
fn the_offset_moves_along_the_normal_of_the_twin() {
    // The sign convention: positive along ∂P/∂u × ∂P/∂v.
    let plane = Surface::Plane {
        frame: Frame::world(),
    };
    let twin = plane.to_nurbs([range(0.0, 1.0), range(0.0, 1.0)]).unwrap();
    let up = twin
        .offset_fitted(0.25, twin.domain(), TOL, &mut Meter::default())
        .unwrap();
    let n = twin.normal(0.5, 0.5).unwrap().into_inner();
    let moved = up.eval(0.5, 0.5).point - twin.eval(0.5, 0.5).point;
    assert!((moved - 0.25 * n).norm() <= TOL);
    assert!((n - Vec3::z()).norm() <= 1e-15 || (n + Vec3::z()).norm() <= 1e-15);
}
