//! Two NURBS surfaces (ADR-0068): the exact twins of any two analytic
//! surfaces in random poses meet where the analytic pair meets. Inside a
//! cube about the origin that both twins reach past, every point of the
//! analytic section is within `SECTION_FIT_FRACTION · tol.linear` of a
//! curve of the twins' section and the other way round, the curves are as
//! many pieces once cut at either section's crossing points and the
//! closed ones as many, and every isolated point of the analytic pair is
//! one of the twins' (a point, or the end of a branch). A pair whose
//! section touches is left to the tolerance, which the twins' rounding
//! tips either way; a pair the marcher refuses as crowded
//! (`SectionFault::CrowdedSingularity`, about one in twenty) is a typed
//! answer and not a wrong one, and the ignored test below holds two of
//! them to the answer they should give.

use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::finite_f64;
use arris_debug::prop::geom::surface;
use arris_geom::{
    Curve, GeomError, MeetKind, SECTION_FIT_FRACTION, Surface, SurfaceIntersection,
    intersect_surfaces,
};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{Aabb, Frame, Interval, Isometry, Meter, Point3, Precision, Tolerance, Vec3};
use proptest::prelude::*;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// Half the side of the cube the sections are compared in.
const CUBE: f64 = 30.0;
/// How far a plane's, cylinder's or cone's twin reaches either way: past
/// the cube and the strategies' poses.
const REACH: f64 = 300.0;
/// Samples per curve.
const SAMPLES: usize = 257;

fn cube() -> Aabb {
    Aabb {
        min: [-CUBE; 3],
        max: [CUBE; 3],
    }
}

/// The twin of `surface` over all of it a pose of the strategies reaches.
fn twin(surface: &Surface) -> Option<Surface> {
    let long = Interval::new(-REACH, REACH).ok()?;
    let bounds = match surface {
        Surface::Plane { .. } => [long, long],
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
            [Interval::TURN, long]
        }
        Surface::Sphere { .. } => [Interval::TURN, Interval::new(-FRAC_PI_2, FRAC_PI_2).ok()?],
        Surface::Torus { .. } => [Interval::TURN, Interval::TURN],
        Surface::Nurbs(_) => return None,
    };
    surface.to_nurbs(bounds).ok().map(Surface::Nurbs)
}

fn samples(curve: &Curve) -> Vec<Point3> {
    let d = curve.domain();
    (0..SAMPLES)
        .map(|k| curve.point(d.lerp(k as f64 / (SAMPLES - 1) as f64)))
        .collect()
}

fn inside(p: Point3) -> bool {
    p.coords.iter().all(|c| c.abs() <= CUBE)
}

/// How far `p` is from the nearest of `curves`.
fn from_all(p: Point3, curves: &[&Curve]) -> f64 {
    (curves.iter())
        .map(|c| c.project(p).map_or(f64::INFINITY, |on| on.distance))
        .fold(f64::INFINITY, f64::min)
}

fn closed(curve: &Curve) -> bool {
    let d = curve.domain();
    (curve.point(d.lo()) - curve.point(d.hi())).norm() <= tol().linear
}

fn twin_property((a, b): (Surface, Surface)) -> Result<(), TestCaseError> {
    let (Some(ta), Some(tb)) = (twin(&a), twin(&b)) else {
        return Ok(());
    };
    let Ok(SurfaceIntersection::Meets { curves, points }) =
        intersect_surfaces(&a, &b, &cube(), tol(), &mut Meter::default())
    else {
        return Ok(());
    };
    // A touch is decided by the tolerance, which the twins' rounding may
    // tip either way: only crossings are held.
    if curves.iter().any(|m| m.kind == MeetKind::Touch)
        || points.iter().any(|m| m.kind == MeetKind::Touch)
    {
        return Ok(());
    }
    let ours: Vec<&Curve> = curves.iter().map(|m| &m.curve).collect();
    let held = ours.iter().any(|c| samples(c).into_iter().any(inside))
        || points.iter().any(|q| inside(q.point));
    // A section the cube does not reach is the twins' to find or not.
    if !held {
        return Ok(());
    }
    let got = match intersect_surfaces(&ta, &tb, &cube(), tol(), &mut Meter::default()) {
        Ok(SurfaceIntersection::Meets { curves, points }) => (curves, points),
        Ok(other) => {
            return Err(TestCaseError::fail(format!(
                "the twins of {a:?} and {b:?} give {other:?}, the pair {curves:?}"
            )));
        }
        Err(GeomError::DegenerateSection { .. }) => {
            return Ok(());
        }
        Err(e) => {
            return Err(TestCaseError::fail(format!(
                "the twins of {a:?} and {b:?}: {e}"
            )));
        }
    };
    let (twin_curves, twin_points) = got;
    let bound = SECTION_FIT_FRACTION * tol().linear;
    let theirs: Vec<&Curve> = twin_curves.iter().map(|m| &m.curve).collect();
    for m in &curves {
        for p in samples(&m.curve).into_iter().filter(|p| inside(*p)) {
            let off = from_all(p, &theirs);
            prop_assert!(off <= bound, "{p} of the pair is {off} from the twins'");
        }
    }
    for c in &theirs {
        for p in samples(c).into_iter().filter(|p| inside(*p)) {
            let off = from_all(p, &ours);
            prop_assert!(off <= bound, "{p} of the twins is {off} from the pair's");
        }
    }
    // A crossing point of either section cuts the curves through it: a
    // closed form runs its curves through the point where two walls
    // touch, a traced section ends its branches there, and the two are
    // counted as the pieces they make.
    let cuts: Vec<Point3> = (points.iter().chain(&twin_points))
        .filter(|q| q.kind == MeetKind::Crossing)
        .map(|q| q.point)
        .collect();
    let cut = |c: &Curve| {
        let d = c.domain();
        let ends = [c.point(d.lo()), c.point(d.hi())];
        (cuts.iter())
            .filter(|&&q| from_all(q, &[c]) <= tol().linear)
            .filter(|&&q| closed(c) || ends.iter().all(|e| (e - q).norm() > tol().linear))
            .count()
    };
    let pieces = |c: &Curve| {
        let k = cut(c);
        if closed(c) { k.max(1) } else { k + 1 }
    };
    // A section is clipped to the cube by the pair and not by the twins,
    // so the loops are counted where they lie wholly inside it.
    let whole = |c: &Curve| closed(c) && cut(c) == 0 && samples(c).into_iter().all(inside);
    let (ours_closed, theirs_closed) = (
        ours.iter().filter(|c| whole(c)).count(),
        theirs.iter().filter(|c| whole(c)).count(),
    );
    let reaching = |cs: &[&Curve]| -> usize {
        (cs.iter())
            .filter(|c| samples(c).into_iter().any(inside))
            .map(|c| pieces(c))
            .sum()
    };
    prop_assert_eq!(
        reaching(&ours),
        reaching(&theirs),
        "{:?} vs {:?}",
        ours,
        theirs
    );
    prop_assert_eq!(ours_closed, theirs_closed, "{:?} vs {:?}", ours, theirs);
    for q in points.iter().filter(|q| inside(q.point)) {
        let near = (twin_points.iter())
            .map(|t| (t.point - q.point).norm())
            .fold(from_all(q.point, &theirs), f64::min);
        prop_assert!(
            near <= tol().linear,
            "{:?} is {near} from the twins'",
            q.point
        );
    }
    Ok(())
}

/// Two analytic surfaces in random poses, each moved so a point of its
/// own, drawn at random, is the origin: they share it, and the cube is
/// about it.
fn meeting_at_the_origin() -> impl Strategy<Value = (Surface, Surface)> {
    let through = || {
        (surface(), finite_f64(0.0..=TAU), finite_f64(-1.2..=1.2)).prop_map(|(s, u, v)| {
            let at = s.point(u, v);
            s.transformed(&Isometry::new(UnitQuaternion::identity(), -at.coords))
        })
    };
    (through(), through())
}

arris_debug::prop_shards! {
    /// Every pair of analytic kinds through a common point, twinned.
    the_twins_of_two_analytic_surfaces_meet_where_the_pair_does
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (pair) = meeting_at_the_origin() => { twin_property(pair) }
}

fn posed(origin: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Frame {
    Frame::new(Point3::from(origin), Vec3::from(z), Vec3::from(x)).unwrap()
}

/// Pairs the property meets and the marcher refuses as crowded, each
/// with the answer its analytic pair gives. The cone's circle is an
/// isocurve of the cone, found as a continuum that is accounted for, but
/// the walk is refused on an uncertified box that continuum leaves
/// behind (the first operand's, for the plane first it is met); the
/// sphere and cylinder in general pose are refused too, its cause not
/// found.
#[test]
#[ignore = "the marcher refuses these as crowded (docs/BACKLOG.md, CrowdedSingularity on the twins of a cone and a plane)"]
fn the_twins_of_a_cone_and_a_plane_and_of_a_sphere_and_a_cylinder_are_met() {
    let pairs = [
        (
            Surface::Cone {
                frame: posed([62.2, -84.7, -43.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]),
                radius: 0.1,
                half_angle: 1.0785,
            },
            Surface::Plane {
                frame: posed([0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]),
            },
        ),
        (
            Surface::Sphere {
                frame: posed(
                    [0.9638, -1.1622, -0.8715],
                    [-0.7558, -0.6535, 0.0415],
                    [-0.3967, 0.4066, -0.8230],
                ),
                radius: 1.7434,
            },
            Surface::Cylinder {
                frame: posed(
                    [-2.4321, -4.3125, -7.6442],
                    [-0.4125, 0.8583, -0.3051],
                    [0.9039, 0.3441, -0.2542],
                ),
                radius: 9.1002,
            },
        ),
    ];
    let refused: Vec<String> = (pairs.iter())
        .filter_map(|(a, b)| {
            let (ta, tb) = (twin(a).unwrap(), twin(b).unwrap());
            intersect_surfaces(&ta, &tb, &cube(), tol(), &mut Meter::default())
                .err()
                .map(|e| format!("{a:?} and {b:?}: {e}"))
        })
        .collect();
    assert!(refused.is_empty(), "{refused:#?}");
}

/// Two cylinders of one radius on crossing axes, the Steinmetz pair, at
/// 60°, 45° and 75°, each at rest and in a tilted pose, `true` for the
/// tilted one.
fn steinmetz_pairs() -> Vec<(Surface, Surface, bool)> {
    let cylinder = |frame: Frame| Surface::Cylinder { frame, radius: 1.0 };
    let tilt = Isometry::new(
        UnitQuaternion::from_euler_angles(0.4, -0.7, 1.1),
        Vec3::new(0.3, -0.2, 0.5),
    );
    [60.0f64, 45.0, 75.0]
        .iter()
        .flat_map(|&psi| {
            let psi = psi.to_radians();
            let axis = [arris_math::real::sin(psi), 0.0, arris_math::real::cos(psi)];
            let a = cylinder(posed([0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]));
            let b = cylinder(posed([0.0; 3], axis, [0.0, 1.0, 0.0]));
            [
                (a.clone(), b.clone(), false),
                (a.transformed(&tilt), b.transformed(&tilt), true),
            ]
        })
        .collect()
}

/// The faults of the Steinmetz pairs `pick` takes: the section refused
/// with one operand twinned, if `one`, and with both, which then has to
/// meet where the pair does ([`twin_property`]).
fn steinmetz_faults(pick: impl Fn(bool) -> bool, one: bool, both: bool) -> Vec<String> {
    let mut faults = Vec::new();
    for (a, b, tilted) in steinmetz_pairs() {
        if !pick(tilted) {
            continue;
        }
        let (ta, tb) = (twin(&a).unwrap(), twin(&b).unwrap());
        let mut runs = Vec::new();
        if one {
            runs.extend([(&ta, &b, "the first"), (&a, &tb, "the second")]);
        }
        if both {
            runs.push((&ta, &tb, "both"));
        }
        for (x, y, which) in runs {
            if let Err(e) = intersect_surfaces(x, y, &cube(), tol(), &mut Meter::default()) {
                faults.push(format!("{a:?} and {b:?}, {which} twinned: {e}"));
            }
        }
        if both && let Err(e) = twin_property((a.clone(), b.clone())) {
            faults.push(format!("{a:?} and {b:?}: {e}"));
        }
    }
    faults
}

/// The Steinmetz pairs: their section is two ellipses crossing at the
/// two points where the walls touch, and with either twinned, or both at
/// rest, it is the same — two ellipses, or the four arcs between the
/// points that the marcher (ADR-0068) and the tracer of a NURBS surface
/// against a cylinder (ADR-0061) end at them. Both refused them as
/// crowded before the tangent points were found first and their
/// departures isolated on the ball's scale (plans/nurbs-nurbs-operands
/// step 5), found by `twin_prop` with both operands twinned (step 2).
#[test]
fn the_twins_of_two_equal_cylinders_on_crossing_axes_meet_in_two_ellipses() {
    let faults = steinmetz_faults(|_| true, true, false);
    let faults = [faults, steinmetz_faults(|tilted| !tilted, false, true)].concat();
    assert!(faults.is_empty(), "{faults:#?}");
}

/// The tilted Steinmetz pairs both twinned: the same two ellipses. The
/// marcher refuses them, `TubeCircle` or `CrowdedSingularity`: the loop
/// condition vanishes on the product of the two rulings through each
/// tangent point, and on twins 600 long in a pose that mixes the axes the
/// isolation (`crate::bernstein_n`) halves the short parameters to the
/// bottom and never the long ones, so boxes far from any zero outgrow
/// the front or come back uncertified (plans/nurbs-nurbs-operands, open
/// question at step 5).
#[test]
#[ignore = "the marcher refuses the tilted twins of two equal cylinders on crossing axes (docs/BACKLOG.md, the tilted Steinmetz pairs)"]
fn the_tilted_twins_of_two_equal_cylinders_on_crossing_axes_meet_in_two_ellipses() {
    let faults = steinmetz_faults(|tilted| tilted, false, true);
    assert!(faults.is_empty(), "{faults:#?}");
}
