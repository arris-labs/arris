//! Cancellation in the geometry crate (ADR-0030): the budget stops a query
//! at the same step every time, a poll stops it within a step, a stop
//! lands within a bounded time on the section the backlog calls six
//! minutes long, and a budget at the query's own count changes nothing.

use std::sync::Mutex;
use std::time::{Duration, Instant};

use arris_geom::{
    Curve, FitError, GeomError, Surface, fit_curve, intersect_curve_surface, intersect_surfaces,
    trace_torus,
};
use arris_math::{
    Aabb, Control, Frame, Interrupted, Interval, Meter, Point3, Precision, Stop, Tolerance, Vec3,
};

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// The thin section of the backlog: a torus of major radius 1000 whose
/// tube is nearly as wide (900), against an elliptic cylinder of
/// semi-axes 990 and 0.5 tilted a tenth of a radian off the torus's
/// plane. Its two loops are fitted, 60 s of the fit's work in a release
/// build (measured while writing this test), and the
/// trace that precedes it takes milliseconds.
fn slow_pair() -> (Surface, Surface, Aabb) {
    let torus = Surface::Torus {
        frame: Frame::world(),
        major_radius: 1000.0,
        minor_radius: 900.0,
    };
    let tilt = 0.1f64;
    let cylinder = Surface::EllipticCylinder {
        frame: Frame::from_z(Point3::origin(), Vec3::new(tilt.cos(), 0.0, tilt.sin())).unwrap(),
        major_radius: 990.0,
        minor_radius: 0.5,
    };
    let within = Aabb {
        min: [-2000.0; 3],
        max: [2000.0; 3],
    };
    (torus, cylinder, within)
}

fn stopped(e: GeomError) -> Interrupted {
    match e {
        GeomError::Interrupted(stop) => stop,
        other => panic!("expected Interrupted, got {other:?}"),
    }
}

/// How many steps of `f`'s own work an unbudgeted run takes.
fn steps_of<T, E: core::fmt::Debug>(f: impl Fn(&mut Meter) -> Result<T, E>) -> u64 {
    let mut meter = Meter::default();
    f(&mut meter).unwrap();
    meter.steps()
}

/// The longest wait a poll saw between two of its calls — the poll is
/// asked at every step, so that is the longest stretch between two ticks
/// — and how many times it was asked, when it turned true after `run_for`.
fn longest_stretch(
    run_for: Duration,
    query: impl Fn(&mut Meter) -> Result<(), GeomError>,
) -> (Duration, u64, Interrupted) {
    let start = Instant::now();
    let seen = Mutex::new((start, Duration::ZERO, 0u64));
    let poll = || {
        let mut seen = seen.lock().unwrap();
        let now = Instant::now();
        seen.1 = seen.1.max(now - seen.0);
        seen.0 = now;
        seen.2 += 1;
        now - start >= run_for
    };
    let control = Control::poll(&poll);
    let stop = stopped(query(&mut Meter::new(&control)).unwrap_err());
    let (_, gap, polls) = *seen.lock().unwrap();
    (gap, polls, stop)
}

/// The most steps a poll that turns true at its second question lets a
/// query take: the first step passes, the second is stopped.
const STOP_WITHIN_STEPS: u64 = 1;

/// The longest a step of the slow section may take, in the test profile
/// (`opt-level = 1`) on a busy machine. Measured at 3.6 ms in a release
/// build: a step is one call of the branch's point or distance.
const LATENCY_BOUND: Duration = Duration::from_millis(500);

#[test]
fn a_poll_stops_the_slow_section_within_a_step() {
    let (torus, cylinder, within) = slow_pair();
    let calls = core::sync::atomic::AtomicU64::new(0);
    let poll = || calls.fetch_add(1, core::sync::atomic::Ordering::Relaxed) >= 1;
    let stop = stopped(
        intersect_surfaces(
            &torus,
            &cylinder,
            &within,
            tol(),
            &mut Meter::new(&Control::poll(&poll)),
        )
        .unwrap_err(),
    );
    assert_eq!(stop.by, Stop::Poll);
    assert!(stop.steps <= STOP_WITHIN_STEPS, "{stop:?}");
}

#[test]
fn a_budget_stops_the_slow_section_at_the_same_step_every_time() {
    let (torus, cylinder, within) = slow_pair();
    let run = |budget: u64| {
        stopped(
            intersect_surfaces(
                &torus,
                &cylinder,
                &within,
                tol(),
                &mut Meter::new(&Control::budget(budget)),
            )
            .unwrap_err(),
        )
    };
    // Deep in the fit, so the count is the fit's and not the trace's.
    let first = run(20_000);
    assert_eq!(
        first,
        Interrupted {
            by: Stop::Budget,
            steps: 20_000
        }
    );
    assert_eq!(run(20_000), first);
}

#[test]
fn the_slow_section_stops_within_the_latency_bound() {
    let (torus, cylinder, within) = slow_pair();
    let (gap, polls, stop) = longest_stretch(Duration::from_secs(2), |meter| {
        intersect_surfaces(&torus, &cylinder, &within, tol(), meter).map(drop)
    });
    assert_eq!(stop.by, Stop::Poll);
    // It got into the fit, where the time is.
    assert!(polls > 1000, "only {polls} steps in the time given");
    assert!(gap < LATENCY_BOUND, "{gap:?} between two steps");
}

#[test]
fn a_fit_budget_of_its_count_changes_nothing_and_one_less_stops_it() {
    let f = |t: f64| Point3::new(t.cos(), t.sin(), 0.2 * t);
    let range = Interval::new(0.0, 4.0).unwrap();
    let fit = |meter: &mut Meter| fit_curve(f, range, 3, |t, q| (q - f(t)).norm(), 1e-9, meter);
    let n = steps_of(fit);
    assert!(n > 10);
    let whole = fit(&mut Meter::default()).unwrap();
    assert_eq!(fit(&mut Meter::new(&Control::budget(n))).unwrap(), whole);
    let short = fit(&mut Meter::new(&Control::budget(n - 1))).unwrap_err();
    assert_eq!(
        short,
        FitError::Interrupted(Interrupted {
            by: Stop::Budget,
            steps: n - 1
        })
    );
    // Converted to the geometry error, a stop stays a stop.
    assert_eq!(
        GeomError::from(short),
        GeomError::Interrupted(Interrupted {
            by: Stop::Budget,
            steps: n - 1
        })
    );
}

#[test]
fn a_torus_section_stops_at_every_budget_below_its_count() {
    let torus = Surface::Torus {
        frame: Frame::world(),
        major_radius: 2.0,
        minor_radius: 0.5,
    };
    let plane = Surface::Plane {
        frame: Frame::from_z(Point3::new(1.0, 0.0, 0.0), Vec3::x()).unwrap(),
    };
    let trace = |meter: &mut Meter| trace_torus(&torus, &plane, tol(), meter);
    let n = steps_of(trace);
    assert!(n > 5, "{n}");
    for k in 0..n {
        let stop = stopped(trace(&mut Meter::new(&Control::budget(k))).unwrap_err());
        assert_eq!(
            stop,
            Interrupted {
                by: Stop::Budget,
                steps: k
            }
        );
    }
    assert!(trace(&mut Meter::new(&Control::budget(n))).is_ok());
}

#[test]
fn a_fitted_curve_against_a_surface_ticks_per_span() {
    // A fitted loop: every span of it is a step of the query.
    let (torus, plane) = (
        Surface::Torus {
            frame: Frame::world(),
            major_radius: 2.0,
            minor_radius: 0.5,
        },
        Surface::Plane {
            frame: Frame::from_z(Point3::new(0.3, 0.0, 0.0), Vec3::new(1.0, 0.4, 0.1)).unwrap(),
        },
    );
    let hit = intersect_surfaces(
        &torus,
        &plane,
        &Aabb {
            min: [-5.0; 3],
            max: [5.0; 3],
        },
        tol(),
        &mut Meter::default(),
    )
    .unwrap();
    let curve = hit
        .curves()
        .iter()
        .map(|m| &m.curve)
        .find(|c| matches!(c, Curve::Nurbs(_)))
        .unwrap();
    let wall = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let query = |meter: &mut Meter| intersect_curve_surface(curve, &wall, tol(), meter);
    let n = steps_of(query);
    assert!(n > 1);
    let stop = stopped(query(&mut Meter::new(&Control::budget(n - 1))).unwrap_err());
    assert_eq!(
        stop,
        Interrupted {
            by: Stop::Budget,
            steps: n - 1
        }
    );
    assert!(query(&mut Meter::new(&Control::budget(n))).is_ok());
}
