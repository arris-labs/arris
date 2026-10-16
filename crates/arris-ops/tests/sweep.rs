//! `ops::sweep` along a path (ADR-0063): a line span and an arc span about
//! an axis in the profile's plane are exact (a pipe of lines and arcs has
//! cylinders and tori, an oblique circle an elliptic cylinder), and every
//! other span is fitted — a degree-one NURBS line agrees with `extrude` and
//! an exact NURBS arc with `revolve` in volume, area and centroid; a spring's volume is the
//! screw's closed form; two spans meet at the profile carried to their
//! joint, named as the second span's start; a closed path builds no caps,
//! and a hole in its profile closes into a void; and an invalid path is
//! refused by name with the model untouched. The checker runs after
//! every build (debug builds), so each `Ok` here is checker-green.

use arris_math::real;
use std::collections::BTreeSet;

use arris_debug::testing::entities_of;
use arris_debug::unmetered::mass_properties;
use arris_geom::{
    FrameLaw, NurbsCurve, Path, PathError, PathSegment, Profile, ProfileLoop, ProfileSegment,
    SurfaceKind,
};
use arris_math::{Axis, Frame, Interval, Point2, Point3, Vec3};
use arris_ops::{Control, OpError, Reason, SweepReason, extrude, revolve, sweep};
use arris_topo::provenance::{Origin, PathPart, Relation, SweepPart};
use arris_topo::{Body, Model, Provenance, Role};
use core::f64::consts::{FRAC_PI_2, PI, TAU};

/// How near a fitted sweep's measures come to the exact build's: the
/// skin is within a quarter of the default tolerance, 2.5e-8, of the
/// carried profile, so a measure of a body some ten across moves by
/// about that relative to its size.
const FITTED_REL: f64 = 1e-8;

fn p(u: f64, v: f64) -> Point2 {
    Point2::new(u, v)
}

/// `(volume, area, centroid)` of `body`.
fn measures(m: &Model, body: Body) -> (f64, f64, Point3) {
    let mp = mass_properties(m, body).unwrap();
    (mp.volume, mp.area, mp.centroid)
}

/// `a` and `b` agree within `rel` of `b`'s size in volume, area and
/// centroid.
fn assert_agree(a: (f64, f64, Point3), b: (f64, f64, Point3), rel: f64, what: &str) {
    let size = real::cbrt(b.0);
    assert!(
        (a.0 - b.0).abs() <= rel * b.0,
        "{what}: volume {} vs {}",
        a.0,
        b.0
    );
    assert!(
        (a.1 - b.1).abs() <= rel * b.1,
        "{what}: area {} vs {}",
        a.1,
        b.1
    );
    assert!(
        (a.2 - b.2).norm() <= rel * size,
        "{what}: centroid {:?} vs {:?}",
        a.2,
        b.2
    );
}

/// The sweep's parts: every entity of the body `Generated` once from a
/// `Role::Sweep`, nothing modified or deleted, and every part naming one
/// entity.
fn parts(m: &Model, body: Body, provenance: &Provenance) -> BTreeSet<PathPart> {
    let entities = entities_of(m, body);
    let mut found = BTreeSet::new();
    for &e in &entities {
        let origins = provenance.origins(e);
        assert_eq!(origins.len(), 1, "{e}: {origins:?}");
        let (relation, origin) = origins[0];
        assert_eq!(relation, Relation::Generated, "{e} from {origin}");
        let Origin::Role(Role::Sweep(part)) = origin else {
            panic!("{e}: generated from {origin}, not a sweep part");
        };
        assert!(found.insert(part), "{part:?} names two entities");
    }
    assert_eq!(provenance.deleted().count(), 0);
    assert_eq!(provenance.outputs().len(), entities.len());
    found
}

/// A plate 40 × 30 with a hole of radius 4 on the xy plane.
fn plate() -> Profile {
    Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(40.0, 0.0)),
                ProfileSegment::LineTo(p(40.0, 30.0)),
                ProfileSegment::LineTo(p(0.0, 30.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        },
        holes: vec![ProfileLoop::Circle {
            center: p(20.0, 15.0),
            radius: 4.0,
        }],
    }
}

/// A slot x ∈ [8, 10] with a round end at x = 10, z ∈ [−1, 1], in the
/// xz plane: beside the z axis, so a turn about z is a revolve.
fn slot_beside_z() -> Profile {
    Profile {
        plane: Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Path {
            start: p(8.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(10.0, -1.0)),
                ProfileSegment::ArcTo {
                    via: p(11.0, 0.0),
                    to: p(10.0, 1.0),
                },
                ProfileSegment::LineTo(p(8.0, 1.0)),
                ProfileSegment::LineTo(p(8.0, -1.0)),
            ],
        },
        holes: Vec::new(),
    }
}

/// The disc of radius `r` (with a hole of radius `hole`, if any) centred
/// at `(R, 0, 0)` in the xz plane.
fn disc_at(big_r: f64, r: f64, hole: Option<f64>) -> Profile {
    Profile {
        plane: Frame::new(Point3::new(big_r, 0.0, 0.0), -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Circle {
            center: p(0.0, 0.0),
            radius: r,
        },
        holes: hole
            .map(|radius| ProfileLoop::Circle {
                center: p(0.0, 0.0),
                radius,
            })
            .into_iter()
            .collect(),
    }
}

/// The circle of radius `r` about z in the xy plane as four quarter arcs
/// from `(r, 0, 0)`, counter-clockwise: a closed planar path.
fn ring_path(r: f64) -> Path {
    let h = r * 0.5f64.sqrt();
    Path {
        start: Point3::new(r, 0.0, 0.0),
        segments: vec![
            PathSegment::ArcTo {
                via: Point3::new(h, h, 0.0),
                to: Point3::new(0.0, r, 0.0),
            },
            PathSegment::ArcTo {
                via: Point3::new(-h, h, 0.0),
                to: Point3::new(-r, 0.0, 0.0),
            },
            PathSegment::ArcTo {
                via: Point3::new(-h, -h, 0.0),
                to: Point3::new(0.0, -r, 0.0),
            },
            PathSegment::ArcTo {
                via: Point3::new(h, -h, 0.0),
                to: Point3::new(r, 0.0, 0.0),
            },
        ],
    }
}

#[test]
fn a_degree_one_nurbs_line_sweeps_the_extrusion() {
    let line = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::origin(), Point3::new(0.0, 0.0, 10.0)],
        vec![1.0, 1.0],
    )
    .unwrap();
    let path = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::Nurbs(line)],
    };
    let mut m = Model::default();
    let (swept, _) = sweep(
        &mut m,
        &plate(),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (extruded, _) = extrude(&mut m, &plate(), Vec3::z(), 10.0, &Control::NONE).unwrap();
    assert_agree(
        measures(&m, swept),
        measures(&m, extruded),
        FITTED_REL,
        "line",
    );
    assert_eq!(m.faces(swept).unwrap().len(), 7);
    assert_eq!(m.edges(swept).unwrap().len(), 15);
}

#[test]
fn an_exact_nurbs_arc_sweeps_the_revolve() {
    // A quarter turn about z from (9, 0, 0), leaving along +y: the
    // profile's plane holds the axis, so the sweep is a revolve.
    let arc =
        NurbsCurve::circle(&Frame::world(), 9.0, Interval::new(0.0, FRAC_PI_2).unwrap()).unwrap();
    let path = Path {
        start: Point3::new(9.0, 0.0, 0.0),
        segments: vec![PathSegment::Nurbs(arc)],
    };
    let mut m = Model::default();
    let (swept, _) = sweep(
        &mut m,
        &slot_beside_z(),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (revolved, _) = revolve(
        &mut m,
        &slot_beside_z(),
        Axis::z_at(Point3::origin()),
        FRAC_PI_2,
        &Control::NONE,
    )
    .unwrap();
    assert_agree(
        measures(&m, swept),
        measures(&m, revolved),
        FITTED_REL,
        "arc",
    );
}

#[test]
fn a_spring_has_the_screws_volume() {
    // A wire of radius 1 five turns round a helix of radius 10 and pitch
    // 6 about z, the profile in the axis' plane: a screw motion, so the
    // volume is the turn angle times the disc's first moment about the
    // axis, 2π · 5 · π · 10.
    let path = Path {
        start: Point3::new(10.0, 0.0, 0.0),
        segments: vec![PathSegment::Helix {
            axis: Axis::z_at(Point3::origin()),
            pitch: 6.0,
            angle: 5.0 * TAU,
        }],
    };
    let mut m = Model::default();
    let (body, provenance) = sweep(
        &mut m,
        &disc_at(10.0, 1.0, None),
        &path,
        FrameLaw::FixedBinormal(Vec3::z_axis()),
        &Control::NONE,
    )
    .unwrap();
    let (volume, _, centroid) = measures(&m, body);
    let exact = TAU * 5.0 * PI * 10.0;
    assert!(
        (volume - exact).abs() <= 1e-9 * exact,
        "{volume} vs {exact}"
    );
    assert!((centroid - Point3::new(0.0, 0.0, 15.0)).norm() <= 1e-8);
    assert_eq!(
        (
            m.faces(body).unwrap().len(),
            m.edges(body).unwrap().len(),
            m.vertices(body).unwrap().len()
        ),
        (3, 3, 2),
        "two caps, the wire's one face, its seam rail"
    );
    let found = parts(&m, body, &provenance);
    let rise = PathPart {
        span: 0,
        part: SweepPart::Rise {
            loop_index: 0,
            vertex: 0,
        },
    };
    assert!(found.contains(&rise), "{found:?}");
}

#[test]
fn two_spans_meet_at_the_profile_carried_to_their_joint() {
    let square = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(-1.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(1.0, -1.0)),
                ProfileSegment::LineTo(p(1.0, 1.0)),
                ProfileSegment::LineTo(p(-1.0, 1.0)),
                ProfileSegment::LineTo(p(-1.0, -1.0)),
            ],
        },
        holes: Vec::new(),
    };
    let path = Path {
        start: Point3::origin(),
        segments: vec![
            PathSegment::LineTo(Point3::new(0.0, 0.0, 4.0)),
            PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0)),
        ],
    };
    let mut m = Model::default();
    let (body, provenance) = sweep(
        &mut m,
        &square,
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (volume, _, _) = measures(&m, body);
    assert!((volume - 40.0).abs() <= FITTED_REL * 40.0, "{volume}");
    // Three rings of four vertices and four edges, two spans of four
    // rails and four sides, two caps.
    assert_eq!(m.vertices(body).unwrap().len(), 12);
    assert_eq!(m.edges(body).unwrap().len(), 20);
    assert_eq!(m.faces(body).unwrap().len(), 10);
    let found = parts(&m, body, &provenance);
    let at = |span, part| PathPart { span, part };
    let joint = at(
        1,
        SweepPart::StartEdge {
            loop_index: 0,
            segment: 2,
        },
    );
    let end = at(
        1,
        SweepPart::EndEdge {
            loop_index: 0,
            segment: 2,
        },
    );
    for part in [
        joint,
        end,
        at(0, SweepPart::StartCap),
        at(1, SweepPart::EndCap),
        at(
            1,
            SweepPart::Side {
                loop_index: 0,
                segment: 3,
            },
        ),
    ] {
        assert!(found.contains(&part), "{part:?} in {found:?}");
    }
    assert!(!found.contains(&at(0, SweepPart::EndCap)));
    assert!(!found.contains(&at(
        0,
        SweepPart::EndEdge {
            loop_index: 0,
            segment: 2
        }
    )));
}

#[test]
fn a_closed_path_builds_no_caps_and_a_hole_closes_into_a_void() {
    let mut m = Model::default();
    let (body, provenance) = sweep(
        &mut m,
        &disc_at(10.0, 2.0, Some(1.0)),
        &ring_path(10.0),
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (volume, _, _) = measures(&m, body);
    // Pappus: the annulus's area times the circle its centre runs.
    let exact = TAU * 10.0 * PI * (4.0 - 1.0);
    assert!(
        (volume - exact).abs() <= FITTED_REL * exact,
        "{volume} vs {exact}"
    );
    assert_eq!(m.shells(body).unwrap().len(), 2, "the tube and its bore");
    assert_eq!(m.faces(body).unwrap().len(), 8, "four spans of two loops");
    let found = parts(&m, body, &provenance);
    assert!(found.iter().all(|p| !matches!(
        p.part,
        SweepPart::StartCap | SweepPart::EndCap | SweepPart::EndEdge { .. }
    )));
    assert!(found.contains(&PathPart {
        span: 0,
        part: SweepPart::Cavity {
            loop_index: 1,
            segment: 0
        }
    }));
}

#[test]
fn an_invalid_path_is_refused_by_name_with_the_model_untouched() {
    let bent = Path {
        start: Point3::origin(),
        segments: vec![
            PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0)),
            PathSegment::LineTo(Point3::new(10.0, 0.0, 10.0)),
        ],
    };
    let mut m = Model::default();
    let before = arris_io::native::to_bytes(&m).unwrap();
    let err = sweep(
        &mut m,
        &plate(),
        &bent,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap_err();
    assert!(
        matches!(err, OpError::Path(PathError::Corner { segment: 1, .. })),
        "{err}"
    );
    assert_eq!(arris_io::native::to_bytes(&m).unwrap(), before);
    let empty = Path {
        start: Point3::origin(),
        segments: Vec::new(),
    };
    let err = sweep(
        &mut m,
        &plate(),
        &empty,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap_err();
    assert_eq!(err, OpError::Path(PathError::Empty));
}

/// A circle of radius `r` centred `c` along x of the xy plane at the
/// origin: normal to a path that starts along z.
fn disc_beside(c: f64, r: f64) -> Profile {
    Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Circle {
            center: p(c, 0.0),
            radius: r,
        },
        holes: Vec::new(),
    }
}

/// A quarter arc of radius `big_r` from `from`, leaving along z and
/// turning toward +x: its centre is `from + (R, 0, 0)`.
fn quarter_arc(from: Point3, big_r: f64) -> PathSegment {
    let h = big_r * 0.5f64.sqrt();
    PathSegment::ArcTo {
        via: from + Vec3::new(big_r - h, 0.0, h),
        to: from + Vec3::new(big_r, 0.0, big_r),
    }
}

fn refusal(
    profile: &Profile,
    path: &Path,
    law: FrameLaw,
) -> (arris_ops::Reason, Vec<arris_topo::Shape>) {
    let mut m = Model::default();
    let before = arris_io::native::to_bytes(&m).unwrap();
    let err = sweep(&mut m, profile, path, law, &Control::NONE).unwrap_err();
    assert_eq!(arris_io::native::to_bytes(&m).unwrap(), before, "{err}");
    match err {
        OpError::Degenerate { reason, entities } => (reason, entities),
        other => panic!("not a degenerate refusal: {other}"),
    }
}

#[test]
fn a_profile_whose_plane_holds_the_path_is_refused() {
    // The plate lies in the xy plane; the path runs along x.
    let along = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::LineTo(Point3::new(10.0, 0.0, 0.0))],
    };
    let (reason, entities) = refusal(&plate(), &along, FrameLaw::RotationMinimizing);
    assert_eq!(reason, Reason::Sweep(SweepReason::ProfileAlongPath));
    assert!(entities.is_empty());
    // Backwards along x is the same plane.
    let back = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::LineTo(Point3::new(-10.0, 0.0, 0.0))],
    };
    let (reason, _) = refusal(&plate(), &back, FrameLaw::RotationMinimizing);
    assert_eq!(reason, Reason::Sweep(SweepReason::ProfileAlongPath));
}

#[test]
fn a_profile_that_reaches_the_radius_of_curvature_folds() {
    let start = Point3::origin();
    let path = Path {
        start,
        segments: vec![quarter_arc(start, 2.0)],
    };
    let length = PI;
    for r in [1.2, 1.0] {
        let (reason, _) = refusal(&disc_beside(1.0, r), &path, FrameLaw::RotationMinimizing);
        let Reason::Sweep(SweepReason::SweepFolds { span, at }) = reason else {
            panic!("radius {r}: {reason}");
        };
        assert_eq!(span, 0);
        assert!((0.0..=length).contains(&at), "at {at}");
    }
    // The fold is on the later span: a line, then the arc.
    let from = Point3::new(0.0, 0.0, 5.0);
    let later = Path {
        start,
        segments: vec![PathSegment::LineTo(from), quarter_arc(from, 2.0)],
    };
    let (reason, _) = refusal(&disc_beside(1.0, 1.2), &later, FrameLaw::RotationMinimizing);
    let Reason::Sweep(SweepReason::SweepFolds { span, at }) = reason else {
        panic!("{reason}");
    };
    assert_eq!(span, 1);
    assert!((5.0..=5.0 + length).contains(&at), "at {at}");
    // On the helix of a spring the radius of curvature is 10.09, and a
    // circle reaching 10.5 toward the axis folds under either law.
    let spring = Path {
        start: Point3::new(10.0, 0.0, 0.0),
        segments: vec![PathSegment::Helix {
            axis: Axis::z_at(Point3::origin()),
            pitch: 6.0,
            angle: TAU,
        }],
    };
    let wide = Profile {
        plane: Frame::new(Point3::new(10.0, 0.0, 0.0), Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Circle {
            center: p(-5.2, 0.0),
            radius: 5.3,
        },
        holes: Vec::new(),
    };
    for law in [
        FrameLaw::FixedBinormal(Vec3::z_axis()),
        FrameLaw::RotationMinimizing,
    ] {
        let (reason, _) = refusal(&wide, &spring, law);
        assert!(
            matches!(
                reason,
                Reason::Sweep(SweepReason::SweepFolds { span: 0, .. })
            ),
            "{reason}"
        );
    }
}

#[test]
fn a_profile_just_inside_the_radius_of_curvature_builds_to_pappus() {
    let start = Point3::origin();
    let path = Path {
        start,
        segments: vec![quarter_arc(start, 2.0)],
    };
    let mut m = Model::default();
    let (body, _) = sweep(
        &mut m,
        &disc_beside(1.0, 0.95),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (volume, ..) = measures(&m, body);
    let pappus = PI * 0.95 * 0.95 * 1.0 * FRAC_PI_2;
    assert!(
        (volume - pappus).abs() <= 1e-8 * pappus,
        "{volume} vs {pappus}"
    );
}

/// Five circular arcs, tangent at every joint, whose tangents' great-circle
/// polygon on the unit sphere encloses the solid angle that is the
/// rotation-minimising frame's holonomy: 3.128748398822758 radians. Each
/// is `(via, to)`; the path starts at the origin.
const SKEW_LOOP: [([f64; 3], [f64; 3]); 5] = [
    (
        [
            0.025631717343445243,
            -0.22255515917092028,
            -0.04141506080308428,
        ],
        [
            0.16146790659143345,
            -0.2709305646864745,
            -0.21779913293172504,
        ],
    ),
    (
        [0.7530928383404543, -0.1717464891315823, -0.9859580680482324],
        [1.2793867563833885, -0.5589088300802914, -1.7091770693833546],
    ),
    (
        [1.4450262211229823, -0.7902576293868131, -1.7700349739616041],
        [1.562805755088113, -0.9627345026429596, -1.5674464912184247],
    ),
    (
        [
            1.4201306620364025,
            -0.7732777471431375,
            -0.44399845834268314,
        ],
        [0.8089029694006776, 0.12606559920359686, -0.0752603668857903],
    ),
    (
        [0.3580550449872174, 0.32097910047823924, -0.1042264954012142],
        [0.0, 0.0, 0.0],
    ),
];

#[test]
fn a_closed_path_whose_frame_does_not_close_is_refused() {
    let path = Path {
        start: Point3::origin(),
        segments: SKEW_LOOP
            .iter()
            .map(|&(via, to)| PathSegment::ArcTo {
                via: Point3::from(via),
                to: Point3::from(to),
            })
            .collect(),
    };
    // Normal to the path's start tangent, the first arc's chord's
    // direction near enough.
    let plane = Frame::new(
        Point3::origin(),
        Vec3::new(-0.23198369933698954, -0.9366602416187215, 0.262395036181486),
        Vec3::new(-0.9706720660307708, 0.24040744628142227, 0.0),
    )
    .unwrap();
    let profile = Profile {
        plane,
        outer: ProfileLoop::Circle {
            center: p(0.0, 0.0),
            radius: 0.05,
        },
        holes: Vec::new(),
    };
    let (reason, _) = refusal(&profile, &path, FrameLaw::RotationMinimizing);
    let Reason::Sweep(SweepReason::FrameDoesNotClose { angle }) = reason else {
        panic!("{reason}");
    };
    assert!((angle - 3.128748398822758).abs() < 1e-4, "angle {angle}");
    // A fixed binormal closes by construction: whatever it does here, it
    // is not this refusal.
    let mut m = Model::default();
    let fixed = sweep(
        &mut m,
        &profile,
        &path,
        FrameLaw::FixedBinormal(Vec3::z_axis()),
        &Control::NONE,
    );
    assert!(
        !matches!(
            fixed,
            Err(OpError::Degenerate {
                reason: Reason::Sweep(SweepReason::FrameDoesNotClose { .. }),
                ..
            })
        ),
        "{fixed:?}"
    );
}

/// How many faces of `body` lie on a surface of `kind`.
fn faces_on(m: &Model, body: Body, kind: SurfaceKind) -> usize {
    m.faces(body)
        .unwrap()
        .into_iter()
        .filter(|f| m.surface(m.face(f.id).unwrap().surface()).unwrap().kind() == kind)
        .count()
}

/// The perimeter of the ellipse of semi-axes `a` and `b`: the trapezoid
/// rule on a periodic integrand converges faster than any power of the
/// step, so 4096 points are exact to rounding.
fn ellipse_perimeter(a: f64, b: f64) -> f64 {
    let n = 4096;
    let step = TAU / n as f64;
    (0..n)
        .map(|i| {
            let (s, c) = real::sin_cos(i as f64 * step);
            (a * a * s * s + b * b * c * c).sqrt()
        })
        .sum::<f64>()
        * step
}

#[test]
fn a_pipe_of_lines_and_arcs_is_cylinders_and_tori() {
    // A circle of radius 2 along 20 up z, a quarter turn of radius 10
    // toward +x, then 20 along +x: the profile's plane holds each arc's
    // axis, so every span is exact.
    let r = 2.0;
    let big_r = 10.0;
    let h = big_r * 0.5f64.sqrt();
    let corner = Point3::new(0.0, 0.0, 20.0);
    let path = Path {
        start: Point3::origin(),
        segments: vec![
            PathSegment::LineTo(corner),
            PathSegment::ArcTo {
                via: corner + Vec3::new(big_r - h, 0.0, h),
                to: corner + Vec3::new(big_r, 0.0, big_r),
            },
            PathSegment::LineTo(corner + Vec3::new(big_r + 20.0, 0.0, big_r)),
        ],
    };
    let profile = disc_beside(0.0, r);
    let mut m = Model::default();
    let (body, _) = sweep(
        &mut m,
        &profile,
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    assert_eq!(faces_on(&m, body, SurfaceKind::Cylinder), 2);
    assert_eq!(faces_on(&m, body, SurfaceKind::Torus), 1);
    assert_eq!(faces_on(&m, body, SurfaceKind::Plane), 2, "the two caps");
    assert_eq!(faces_on(&m, body, SurfaceKind::Nurbs), 0);
    // Guldin: the disc's area times the length of the path.
    let length = 20.0 + 20.0 + big_r * FRAC_PI_2;
    let (volume, area, _) = measures(&m, body);
    let exact = PI * r * r * length;
    assert!(
        (volume - exact).abs() <= 1e-9 * exact,
        "{volume} vs {exact}"
    );
    let lateral = TAU * r * length + 2.0 * PI * r * r;
    assert!(
        (area - lateral).abs() <= 1e-9 * lateral,
        "{area} vs {lateral}"
    );
}

#[test]
fn an_oblique_circle_or_ellipse_is_the_elliptic_cylinder_whatever_the_tilt() {
    // A circle of radius 3 in planes tilted 30° about axes in the plane at
    // several angles, carried 12 along z: the section across the path is
    // an ellipse of semi-axes 3 and 3 cos 30°, whose major axis is the tilt
    // axis, so the circle's own start is on it only for some of them.
    let (r, length, tilt) = (3.0f64, 12.0, 30.0f64.to_radians());
    for about in [0.0, 0.4, FRAC_PI_2, 2.0, 3.5] {
        let axis = Vec3::new(real::cos(about), real::sin(about), 0.0);
        let across = Vec3::z().cross(&axis);
        let plane = Frame::new(
            Point3::origin(),
            across * (-real::sin(tilt)) + Vec3::z() * real::cos(tilt),
            axis,
        )
        .unwrap();
        let profile = Profile {
            plane,
            outer: ProfileLoop::Circle {
                center: p(0.0, 0.0),
                radius: r,
            },
            holes: Vec::new(),
        };
        let path = Path {
            start: Point3::origin(),
            segments: vec![PathSegment::LineTo(Point3::new(0.0, 0.0, length))],
        };
        let mut m = Model::default();
        let (body, _) = sweep(
            &mut m,
            &profile,
            &path,
            FrameLaw::RotationMinimizing,
            &Control::NONE,
        )
        .unwrap();
        assert_eq!(
            faces_on(&m, body, SurfaceKind::EllipticCylinder),
            1,
            "tilt about {about}"
        );
        assert_eq!(
            faces_on(&m, body, SurfaceKind::Nurbs),
            0,
            "tilt about {about}"
        );
        let (volume, area, centre) = measures(&m, body);
        let exact = PI * r * r * real::cos(tilt) * length;
        assert!(
            (volume - exact).abs() <= 1e-9 * exact,
            "{about}: {volume} vs {exact}"
        );
        let lateral = ellipse_perimeter(r, r * real::cos(tilt)) * length + 2.0 * PI * r * r;
        assert!(
            (area - lateral).abs() <= 1e-9 * lateral,
            "{about}: {area} vs {lateral}"
        );
        assert!(
            (centre - Point3::new(0.0, 0.0, 0.5 * length)).norm() <= 1e-9,
            "{about}: {centre:?}"
        );
    }
}

#[test]
fn an_oblique_ellipse_sweeps_the_elliptic_cylinder_too() {
    // An ellipse of semi-axes 4 and 2 tilted 40° about its minor axis (so
    // its major shortens) and about its major (so its minor does).
    let (length, tilt) = (9.0f64, 40.0f64.to_radians());
    for about_major in [true, false] {
        let axis = if about_major { Vec3::x() } else { Vec3::y() };
        let across = Vec3::z().cross(&axis);
        let plane = Frame::new(
            Point3::origin(),
            across * (-real::sin(tilt)) + Vec3::z() * real::cos(tilt),
            axis,
        )
        .unwrap();
        // The ellipse's major axis along the plane's x.
        let profile = Profile {
            plane,
            outer: ProfileLoop::Ellipse {
                center: p(0.0, 0.0),
                major: arris_math::Vec2::new(4.0, 0.0),
                minor_radius: 2.0,
            },
            holes: Vec::new(),
        };
        let path = Path {
            start: Point3::origin(),
            segments: vec![PathSegment::LineTo(Point3::new(0.0, 0.0, length))],
        };
        let mut m = Model::default();
        let Ok((body, _)) = sweep(
            &mut m,
            &profile,
            &path,
            FrameLaw::RotationMinimizing,
            &Control::NONE,
        ) else {
            panic!("tilt about the major: {about_major}");
        };
        assert_eq!(faces_on(&m, body, SurfaceKind::EllipticCylinder), 1);
        let (volume, _, _) = measures(&m, body);
        let exact = PI * 4.0 * 2.0 * real::cos(tilt) * length;
        assert!(
            (volume - exact).abs() <= 1e-9 * exact,
            "{volume} vs {exact}"
        );
    }
}

#[test]
fn an_exact_span_joins_a_fitted_one_at_the_carried_profile() {
    // A line, then the same quarter arc given as its exact NURBS: the line
    // is a cylinder and the arc is fitted, and they share the ring that is
    // the circle's NURBS.
    let (r, big_r) = (1.5, 8.0);
    let arc = NurbsCurve::circle(
        &Frame::new(Point3::new(big_r, 0.0, 6.0), Vec3::y(), -Vec3::x()).unwrap(),
        big_r,
        Interval::new(0.0, FRAC_PI_2).unwrap(),
    )
    .unwrap();
    let path = Path {
        start: Point3::origin(),
        segments: vec![
            PathSegment::LineTo(Point3::new(0.0, 0.0, 6.0)),
            PathSegment::Nurbs(arc),
        ],
    };
    let mut m = Model::default();
    let (body, provenance) = sweep(
        &mut m,
        &disc_beside(0.0, r),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    assert_eq!(faces_on(&m, body, SurfaceKind::Cylinder), 1);
    assert_eq!(faces_on(&m, body, SurfaceKind::Nurbs), 1);
    let (volume, _, _) = measures(&m, body);
    let exact = PI * r * r * (6.0 + big_r * FRAC_PI_2);
    assert!(
        (volume - exact).abs() <= FITTED_REL * exact,
        "{volume} vs {exact}"
    );
    assert!(parts(&m, body, &provenance).contains(&PathPart {
        span: 1,
        part: SweepPart::StartEdge {
            loop_index: 0,
            segment: 0
        },
    }));
}

#[test]
fn a_binormal_that_is_not_the_arcs_axis_fits_it() {
    // A quarter arc about y: the binormal y keeps the profile's motion a
    // rotation about the arc's axis, any other does not.
    let big_r = 10.0;
    let arc = quarter_arc(Point3::origin(), big_r);
    let path = Path {
        start: Point3::origin(),
        segments: vec![arc],
    };
    let tube = disc_beside(0.0, 1.0);
    let mut m = Model::default();
    let (about, _) = sweep(
        &mut m,
        &tube,
        &path,
        FrameLaw::FixedBinormal(Vec3::y_axis()),
        &Control::NONE,
    )
    .unwrap();
    assert_eq!(faces_on(&m, about, SurfaceKind::Torus), 1);
    let slant = arris_math::UnitVec3::new_normalize(Vec3::new(0.0, 1.0, 0.3));
    let (off, _) = sweep(
        &mut m,
        &tube,
        &path,
        FrameLaw::FixedBinormal(slant),
        &Control::NONE,
    )
    .unwrap();
    assert_eq!(faces_on(&m, off, SurfaceKind::Torus), 0);
    assert_eq!(faces_on(&m, off, SurfaceKind::Nurbs), 1);
}
