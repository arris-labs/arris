use std::f64::consts::{PI, TAU};

use arris_math::{
    Axis, Control, Interval, Meter, Point3, Precision, Stop, Tolerance, UnitVec3, Vec3,
};

use crate::{
    FrameLaw, NurbsCurve, Path, PathDegeneracy, PathError, PathSegment, fit_curve,
    fit_curve_periodic,
};

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn frames(path: &Path, law: FrameLaw) -> Result<crate::FrameField, PathError> {
    path.frames(law, tol(), &mut Meter::default())
}

/// A helix of radius `r` and pitch `p` about `+z` from `(r, 0, 0)`.
fn helix(r: f64, pitch: f64, turns: f64) -> Path {
    Path {
        start: Point3::new(r, 0.0, 0.0),
        segments: vec![PathSegment::Helix {
            axis: Axis::z_at(Point3::origin()),
            pitch,
            angle: TAU * turns,
        }],
    }
}

fn quarter_arc(r: f64) -> Path {
    let h = r * 0.5f64.sqrt();
    Path {
        start: Point3::origin(),
        segments: vec![PathSegment::ArcTo {
            via: Point3::new(r - h, 0.0, h),
            to: Point3::new(r, 0.0, r),
        }],
    }
}

fn angle_about(from: &Vec3, to: &Vec3, axis: &Vec3) -> f64 {
    to.dot(&axis.cross(from)).atan2(to.dot(from))
}

/// Parallel transport of `u` along a dense polyline of `f`, by the minimal
/// rotation between successive tangents: an independent integrator.
fn transport(
    f: impl Fn(f64) -> Point3,
    a: f64,
    b: f64,
    n: usize,
    t0: Vec3,
    u0: Vec3,
) -> (Vec3, Vec3) {
    let mut u = u0;
    let mut prev = f(a);
    let mut tprev = t0;
    let mut last_t = Vec3::zeros();
    for i in 1..=n {
        let q = f(a + (b - a) * i as f64 / n as f64);
        let t = (q - prev).normalize();
        let axis = tprev.cross(&t);
        let s = axis.norm();
        if s > 0.0 {
            let angle = s.atan2(tprev.dot(&t));
            let k = axis / s;
            let (sn, c) = angle.sin_cos();
            u = u * c + k.cross(&u) * sn + k * (k.dot(&u) * (1.0 - c));
        }
        tprev = t;
        last_t = t;
        prev = q;
    }
    (u, last_t)
}

#[test]
fn a_line_is_a_translation_under_both_laws() {
    let path = Path {
        start: Point3::new(1.0, 2.0, 3.0),
        segments: vec![PathSegment::LineTo(Point3::new(1.0, 2.0, 13.0))],
    };
    for law in [
        FrameLaw::RotationMinimizing,
        FrameLaw::FixedBinormal(UnitVec3::new_normalize(Vec3::new(1.0, 1.0, 0.0))),
    ] {
        let field = frames(&path, law).unwrap();
        let m = field.motion_at(4.0);
        let q = Point3::new(0.3, -0.2, 0.7);
        assert!(
            (m.apply(q) - (q + Vec3::new(0.0, 0.0, 4.0))).norm() < 1e-14,
            "{law:?}"
        );
    }
}

#[test]
fn on_a_circle_the_rotation_minimising_frame_is_the_rotation_about_its_axis() {
    let r = 10.0;
    let path = quarter_arc(r);
    let field = frames(&path, FrameLaw::RotationMinimizing).unwrap();
    assert!((field.length() - r * PI / 2.0).abs() < 1e-13);
    // The axis is the line along y through (r, 0, 0): a motion of the
    // first frame by angle θ about it.
    let center = Point3::new(r, 0.0, 0.0);
    for k in 0..=8 {
        let s = field.length() * f64::from(k) / 8.0;
        let theta = s / r;
        let m = field.motion_at(s);
        for q in [
            Point3::new(0.4, 0.3, 0.2),
            Point3::new(-1.0, 2.0, 0.5),
            Point3::new(3.0, -4.0, 1.0),
        ] {
            let v = q - center;
            // Rotation about +y by −θ takes the start tangent +z to the end's:
            // the arc turns from +z toward +x.
            let (st, ct) = theta.sin_cos();
            let want = center + Vec3::new(v.x * ct + v.z * st, v.y, -v.x * st + v.z * ct);
            assert!(
                (m.apply(q) - want).norm() < 1e-12,
                "s {s}: {} vs {}",
                m.apply(q),
                want
            );
        }
    }
}

#[test]
fn on_a_helix_the_rotation_minimising_normal_turns_against_frenet_by_tau_s() {
    let (r, pitch, turns) = (10.0, 6.0, 2.5);
    let field = frames(&helix(r, pitch, turns), FrameLaw::RotationMinimizing).unwrap();
    let rise = pitch / TAU;
    let rho = f64::hypot(r, rise);
    let tau = rise / (rho * rho);
    let frenet_n = |s: f64| {
        let phi = s / rho;
        -Vec3::new(phi.cos(), phi.sin(), 0.0)
    };
    let angle = |s: f64| {
        let f = field.at(s);
        angle_about(
            &frenet_n(s),
            &f.normal.into_inner(),
            &f.tangent.into_inner(),
        )
    };
    let a0 = angle(0.0);
    for k in 1..=10 {
        let s = field.length() * f64::from(k) / 10.0;
        let want = -tau * s;
        let got = angle(s) - a0;
        let diff = (got - want).rem_euclid(TAU);
        assert!(diff.min(TAU - diff) < 1e-12, "s {s}: {got} vs {want}");
    }
}

#[test]
fn the_helix_closed_form_matches_an_independent_parallel_transport() {
    let (r, pitch, turns) = (10.0, 6.0, 2.0);
    let field = frames(&helix(r, pitch, turns), FrameLaw::RotationMinimizing).unwrap();
    let rise = pitch / TAU;
    let f = |t: f64| Point3::new(r * (TAU * t).cos(), r * (TAU * t).sin(), rise * TAU * t);
    let start = field.at(0.0);
    let (u, t_end) = transport(
        f,
        0.0,
        turns,
        400_000,
        start.tangent.into_inner(),
        start.normal.into_inner(),
    );
    // The transport ends on the last chord, whose direction is the
    // tangent at its midpoint.
    let end = field.at(field.length() - field.length() / 800_000.0);
    assert!(
        (end.normal.into_inner() - u).norm() < 1e-8,
        "{}",
        (end.normal.into_inner() - u).norm()
    );
    assert!((end.tangent.into_inner() - t_end).norm() < 1e-8);
}

#[test]
fn a_fixed_binormal_on_a_helix_about_it_is_a_screw_motion() {
    let (r, pitch, turns) = (10.0, 6.0, 1.7);
    let axis_dir = Vec3::z();
    let field = frames(
        &helix(r, pitch, turns),
        FrameLaw::FixedBinormal(UnitVec3::new_normalize(axis_dir)),
    )
    .unwrap();
    let rise = pitch / TAU;
    let rho = f64::hypot(r, rise);
    for k in 1..=9 {
        let s = field.length() * f64::from(k) / 9.0;
        let theta = s / rho;
        let m = field.motion_at(s);
        for q in [
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(11.0, 0.5, -0.4),
            Point3::new(-2.0, 3.0, 5.0),
        ] {
            let (st, ct) = theta.sin_cos();
            let want = Point3::new(q.x * ct - q.y * st, q.x * st + q.y * ct, q.z + rise * theta);
            assert!((m.apply(q) - want).norm() < 1e-12, "s {s}");
        }
    }
}

#[test]
fn a_nurbs_twin_of_a_helix_has_the_closed_form_frames() {
    let (r, pitch, turns) = (10.0, 6.0, 1.5);
    let rise = pitch / TAU;
    let f = |t: f64| Point3::new(r * t.cos(), r * t.sin(), rise * t);
    let range = Interval::new(0.0, TAU * turns).unwrap();
    let fit = fit_curve(
        f,
        range,
        3,
        |t, q| (q - f(t)).norm(),
        1e-10,
        &mut Meter::default(),
    )
    .unwrap();
    let exact = helix(r, pitch, turns);
    let twin = Path {
        start: exact.start,
        segments: vec![PathSegment::Nurbs(fit)],
    };
    for law in [
        FrameLaw::RotationMinimizing,
        FrameLaw::FixedBinormal(UnitVec3::new_normalize(Vec3::new(0.3, 0.2, 1.0))),
    ] {
        let a = frames(&exact, law).unwrap();
        let b = frames(&twin, law).unwrap();
        assert!((a.length() - b.length()).abs() < 1e-8, "{law:?}");
        for k in 0..=12 {
            let s = a.length() * f64::from(k) / 12.0;
            let (fa, fb) = (a.at(s), b.at(s));
            assert!((fa.origin - fb.origin).norm() < 1e-7, "{law:?} s {s}");
            assert!((fa.tangent.into_inner() - fb.tangent.into_inner()).norm() < 1e-7);
            assert!(
                (fa.normal.into_inner() - fb.normal.into_inner()).norm() < 1e-6,
                "{law:?} s {s}"
            );
        }
    }
}

#[test]
fn the_first_frames_motion_is_the_identity_and_every_motion_is_rigid() {
    let (r, pitch, turns) = (10.0, 6.0, 1.0);
    let rise = pitch / TAU;
    let f = |t: f64| Point3::new(r * t.cos(), r * t.sin(), rise * t);
    let fit = fit_curve(
        f,
        Interval::new(0.0, TAU).unwrap(),
        3,
        |t, q| (q - f(t)).norm(),
        1e-9,
        &mut Meter::default(),
    )
    .unwrap();
    let path = Path {
        start: f(0.0),
        segments: vec![PathSegment::Nurbs(fit)],
    };
    let _ = turns;
    let field = frames(&path, FrameLaw::RotationMinimizing).unwrap();
    let m0 = field.motion_at(0.0);
    let q = Point3::new(1.0, 2.0, 3.0);
    assert!((m0.apply(q) - q).norm() < 1e-14);
    let s = field.length() * 0.37;
    let m = field.motion_at(s);
    let (a, b) = (Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 2.0, 1.0));
    assert!(((m.apply(a) - m.apply(b)).norm() - (a - b).norm()).abs() < 1e-13);
    // The motion takes the first frame to the frame at s.
    let first = field.at(0.0);
    let at = field.at(s);
    let moved = m.apply_vec(first.normal.into_inner());
    assert!((moved - at.normal.into_inner()).norm() < 1e-12);
    assert!((m.apply(first.origin) - at.origin).norm() < 1e-12);
}

#[test]
fn joints_gaps_and_degenerate_segments_are_refused_by_segment() {
    let line = |to: [f64; 3]| PathSegment::LineTo(Point3::new(to[0], to[1], to[2]));
    let bent = Path {
        start: Point3::origin(),
        segments: vec![line([0.0, 0.0, 10.0]), line([10.0, 0.0, 10.0])],
    };
    match frames(&bent, FrameLaw::default()) {
        Err(PathError::Corner {
            segment: 1,
            at,
            angle,
        }) => {
            assert!((at - 10.0).abs() < 1e-12);
            assert!((angle - PI / 2.0).abs() < 1e-12);
        }
        other => panic!("{other:?}"),
    }
    let short = Path {
        start: Point3::origin(),
        segments: vec![line([0.0, 0.0, 1e-9])],
    };
    assert_eq!(
        frames(&short, FrameLaw::default()).unwrap_err(),
        PathError::Degenerate {
            segment: 0,
            reason: PathDegeneracy::Short
        }
    );
    let collinear = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::ArcTo {
            via: Point3::new(0.0, 0.0, 1.0),
            to: Point3::new(0.0, 0.0, 2.0),
        }],
    };
    assert_eq!(
        frames(&collinear, FrameLaw::default()).unwrap_err(),
        PathError::Degenerate {
            segment: 0,
            reason: PathDegeneracy::CollinearArc
        }
    );
    let on_axis = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::Helix {
            axis: Axis::z_at(Point3::origin()),
            pitch: 1.0,
            angle: 1.0,
        }],
    };
    assert_eq!(
        frames(&on_axis, FrameLaw::default()).unwrap_err(),
        PathError::Degenerate {
            segment: 0,
            reason: PathDegeneracy::StartOnAxis
        }
    );
    let no_turn = Path {
        start: Point3::new(1.0, 0.0, 0.0),
        segments: vec![PathSegment::Helix {
            axis: Axis::z_at(Point3::origin()),
            pitch: 1.0,
            angle: 0.0,
        }],
    };
    assert_eq!(
        frames(&no_turn, FrameLaw::default()).unwrap_err(),
        PathError::Degenerate {
            segment: 0,
            reason: PathDegeneracy::NoTurn
        }
    );
    let empty = Path {
        start: Point3::origin(),
        segments: vec![],
    };
    assert_eq!(
        frames(&empty, FrameLaw::default()).unwrap_err(),
        PathError::Empty
    );
    // A NURBS segment that starts elsewhere.
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::new(5.0, 0.0, 0.0), Point3::new(5.0, 0.0, 5.0)],
        vec![1.0, 1.0],
    )
    .unwrap();
    let gap = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::Nurbs(curve)],
    };
    match frames(&gap, FrameLaw::default()) {
        Err(PathError::Gap {
            segment: 0,
            distance,
        }) => assert!((distance - 5.0).abs() < 1e-12),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_corner_inside_a_nurbs_segment_is_a_corner() {
    let curve = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 2.0, 2.0],
        vec![
            Point3::origin(),
            Point3::new(0.0, 0.0, 5.0),
            Point3::new(5.0, 0.0, 5.0),
        ],
        vec![1.0, 1.0, 1.0],
    )
    .unwrap();
    let path = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::Nurbs(curve)],
    };
    match frames(&path, FrameLaw::default()) {
        Err(PathError::Corner { segment: 0, .. }) => {}
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_binormal_along_the_tangent_is_refused_where_it_happens() {
    let line_up = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::LineTo(Point3::new(0.0, 0.0, 5.0))],
    };
    let z = FrameLaw::FixedBinormal(Vec3::z_axis());
    assert_eq!(
        frames(&line_up, z).unwrap_err(),
        PathError::BinormalAlongTangent {
            segment: 0,
            at: 0.0
        }
    );
    // A quarter arc in the xz plane turns from +z to +x: the binormal +x
    // meets the tangent at the far end, not at the start.
    let arc = quarter_arc(10.0);
    match frames(&arc, FrameLaw::FixedBinormal(Vec3::x_axis())) {
        Err(PathError::BinormalAlongTangent { segment: 0, at }) => {
            assert!((at - 10.0 * PI / 2.0).abs() < 1e-9, "{at}");
        }
        other => panic!("{other:?}"),
    }
    // +y is perpendicular to the plane of the arc: fine everywhere.
    assert!(frames(&arc, FrameLaw::FixedBinormal(Vec3::y_axis())).is_ok());
}

fn ring(r: f64) -> Path {
    let h = r * 0.5f64.sqrt();
    let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    Path {
        start: p(r, 0.0),
        segments: vec![
            PathSegment::ArcTo {
                via: p(h, h),
                to: p(0.0, r),
            },
            PathSegment::ArcTo {
                via: p(-h, h),
                to: p(-r, 0.0),
            },
            PathSegment::ArcTo {
                via: p(-h, -h),
                to: p(0.0, -r),
            },
            PathSegment::ArcTo {
                via: p(h, -h),
                to: p(r, 0.0),
            },
        ],
    }
}

#[test]
fn a_closed_planar_path_has_no_holonomy() {
    let field = frames(&ring(10.0), FrameLaw::RotationMinimizing).unwrap();
    assert!(field.is_closed());
    assert!((field.length() - TAU * 10.0).abs() < 1e-12);
    assert!(field.holonomy().abs() < 1e-12, "{}", field.holonomy());
    let fixed = frames(&ring(10.0), FrameLaw::FixedBinormal(Vec3::z_axis())).unwrap();
    assert!(fixed.is_closed());
    assert_eq!(fixed.holonomy(), 0.0);
}

/// The signed area the closed curve of unit vectors `t(i / n)` encloses on
/// the sphere, from the reference point `r`: the sum of the triangles
/// `(r, tᵢ, tᵢ₊₁)` by the Van Oosterom–Strackee formula. Modulo 4π.
fn enclosed_solid_angle(t: impl Fn(f64) -> Vec3, n: usize, r: Vec3) -> f64 {
    let mut sum = 0.0;
    for i in 0..n {
        let a = t(i as f64 / n as f64);
        let b = t((i + 1) as f64 / n as f64);
        let num = r.dot(&a.cross(&b));
        let den = 1.0 + r.dot(&a) + a.dot(&b) + b.dot(&r);
        sum += 2.0 * num.atan2(den);
    }
    sum
}

#[test]
fn a_closed_non_planar_path_has_the_solid_angle_of_its_tangents_as_holonomy() {
    // A trefoil: its tangent indicatrix crosses itself, so it does not
    // bisect the sphere (Jacobi's theorem is for simple indicatrices) and
    // the writhe, hence the holonomy, is not a whole turn.
    let f = |t: f64| {
        let r = 2.0 + (3.0 * t).cos();
        Point3::new(r * (2.0 * t).cos(), r * (2.0 * t).sin(), (3.0 * t).sin())
    };
    let fit = fit_curve_periodic(
        f,
        Interval::new(0.0, TAU).unwrap(),
        3,
        |t, q| (q - f(t)).norm(),
        1e-9,
        &mut Meter::default(),
    )
    .unwrap();
    let path = Path {
        start: f(0.0),
        segments: vec![PathSegment::Nurbs(fit)],
    };
    let field = frames(&path, FrameLaw::RotationMinimizing).unwrap();
    assert!(field.is_closed());
    let tangent = |u: f64| {
        let t = TAU * u;
        let r = 2.0 + (3.0 * t).cos();
        Vec3::new(
            -3.0 * (3.0 * t).sin() * (2.0 * t).cos() - 2.0 * r * (2.0 * t).sin(),
            -3.0 * (3.0 * t).sin() * (2.0 * t).sin() + 2.0 * r * (2.0 * t).cos(),
            3.0 * (3.0 * t).cos(),
        )
        .normalize()
    };
    let omega = enclosed_solid_angle(tangent, 20_000, Vec3::new(0.31, 0.52, 0.79).normalize());
    let wrapped = (omega + PI).rem_euclid(TAU) - PI;
    let h = field.holonomy();
    let diff = (h - wrapped).rem_euclid(TAU);
    assert!(
        diff.min(TAU - diff) < 1e-5,
        "holonomy {h}, solid angle {omega} (wrapped {wrapped})"
    );
    assert!(
        h.abs() > 0.1,
        "the test curve must have a holonomy to test: {h}"
    );
}

#[test]
fn a_stopped_meter_stops_the_frame_table() {
    let f = |t: f64| Point3::new(t.cos(), t.sin(), 0.2 * t);
    let fit = fit_curve(
        f,
        Interval::new(0.0, 3.0).unwrap(),
        3,
        |t, q| (q - f(t)).norm(),
        1e-9,
        &mut Meter::default(),
    )
    .unwrap();
    let path = Path {
        start: f(0.0),
        segments: vec![PathSegment::Nurbs(fit)],
    };
    let control = Control::budget(3);
    let err = path
        .frames(FrameLaw::default(), tol(), &mut Meter::new(&control))
        .unwrap_err();
    match err {
        PathError::Interrupted(stop) => assert_eq!((stop.by, stop.steps), (Stop::Budget, 3)),
        other => panic!("{other:?}"),
    }
}
