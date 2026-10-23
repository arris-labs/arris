//! Extrema of the distance between two surfaces: the closed forms and the
//! continua by name; and, for random pairs in random poses, every pair
//! stationary and on both surfaces, every local minimum a dense sample of
//! one surface leads to (the other eliminated by its closed-form
//! projection) among the answer, and the answer swapping with its
//! operands. Where the surfaces meet the answer says nothing at distance
//! zero, and the sampled minima are held only away from it.

use arris_debug::testing::tol;
use arris_math::real;
use core::f64::consts::FRAC_PI_2;

use arris_debug::prop::geom::{cone, cylinder, elliptic_cylinder, plane, sphere, surface, torus};
use arris_debug::prop::{DEFAULT_SCALE, finite_f64};
use arris_geom::{
    GeomError, NurbsSurface, Surface, SurfaceExtrema, SurfaceKind, extrema_surface_surface,
};
use arris_math::{Frame, Meter, Point3, Tolerance, Vec3};
use proptest::prelude::*;

fn p(x: [f64; 3]) -> Point3 {
    Point3::new(x[0], x[1], x[2])
}

fn v(x: [f64; 3]) -> Vec3 {
    Vec3::new(x[0], x[1], x[2])
}

fn frame(o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> Frame {
    Frame::new(p(o), v(z), v(x)).unwrap()
}

fn world(o: [f64; 3]) -> Frame {
    frame(o, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
}

fn extrema(a: &Surface, b: &Surface) -> SurfaceExtrema {
    extrema_surface_surface(a, b, tol(), &mut Meter::default())
        .unwrap_or_else(|e| panic!("{a:?} vs {b:?}: {e}"))
}

fn distances(e: &SurfaceExtrema) -> Vec<f64> {
    let mut d: Vec<f64> = e.isolated.iter().map(|x| x.distance).collect();
    d.sort_by(f64::total_cmp);
    d
}

fn continua(e: &SurfaceExtrema) -> Vec<f64> {
    e.continua.iter().map(|x| x.distance).collect()
}

/// Every pair lies on both surfaces at its parameters and is stationary:
/// the offset is along each surface's normal within the tolerance.
fn well_formed(a: &Surface, b: &Surface, e: &SurfaceExtrema) -> Result<(), String> {
    for x in e.isolated.iter().chain(&e.continua) {
        let scale = 1e-12 * DEFAULT_SCALE.max(x.point_a.coords.norm());
        if (a.point(x.uv_a.x, x.uv_a.y) - x.point_a).norm() > scale
            || (b.point(x.uv_b.x, x.uv_b.y) - x.point_b).norm() > scale
        {
            return Err(format!("{x:?} is off its surfaces"));
        }
        if ((x.point_a - x.point_b).norm() - x.distance).abs() > scale {
            return Err(format!("{x:?}: the distance is not the points'"));
        }
        let r = x.point_a - x.point_b;
        for (s, uv) in [(a, x.uv_a), (b, x.uv_b)] {
            if let Some(n) = s.normal(uv.x, uv.y) {
                if x.distance > tol().linear && r.cross(&n).norm() > tol().linear {
                    return Err(format!(
                        "{x:?} is not along a normal: {}",
                        r.cross(&n).norm()
                    ));
                }
            }
        }
    }
    Ok(())
}

/// The answer with the operands swapped is the answer swapped.
fn swaps(a: &Surface, b: &Surface, e: &SurfaceExtrema) -> Result<(), String> {
    let f = extrema(b, a);
    if f.isolated.len() != e.isolated.len() || f.continua.len() != e.continua.len() {
        return Err(format!("{e:?}\nswapped\n{f:?}"));
    }
    for x in &e.isolated {
        let found = f.isolated.iter().any(|y| {
            (x.point_a - y.point_b).norm() <= tol().linear
                && (x.point_b - y.point_a).norm() <= tol().linear
        });
        if !found {
            return Err(format!("{x:?} is not among the swapped {:?}", f.isolated));
        }
    }
    Ok(())
}

// --- by name --------------------------------------------------------------------

#[test]
fn a_ball_beside_a_pipe_is_a_point_against_it() {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    };
    let ball = Surface::Sphere {
        frame: world([3.0, 0.0, 2.0]),
        radius: 0.5,
    };
    let e = extrema(&ball, &pipe);
    let d = distances(&e);
    // The pipe's near and far rulings are 2 and 4 from the centre: 1.5,
    // 2.5, 3.5, 4.5 once the ball's radius is taken either way.
    assert_eq!(d.len(), 4, "{e:?}");
    let want = [1.5, 2.5, 3.5, 4.5];
    assert!(
        d.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-12),
        "{d:?}"
    );
    well_formed(&ball, &pipe, &e).unwrap();
    swaps(&ball, &pipe, &e).unwrap();
}

#[test]
fn a_ball_on_a_pipes_axis_is_a_continuum_near_and_one_far() {
    let pipe = Surface::Cylinder {
        frame: Frame::world(),
        radius: 3.0,
    };
    let ball = Surface::Sphere {
        frame: world([0.0, 0.0, 5.0]),
        radius: 1.0,
    };
    let e = extrema(&ball, &pipe);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert!(
        (c[0] - 2.0).abs() < 1e-12 && (c[1] - 4.0).abs() < 1e-12,
        "{c:?}"
    );
    well_formed(&ball, &pipe, &e).unwrap();
}

#[test]
fn concentric_spheres_are_a_continuum_near_and_one_far() {
    let (big, small) = (
        Surface::Sphere {
            frame: world([1.0, 2.0, 3.0]),
            radius: 5.0,
        },
        Surface::Sphere {
            frame: world([1.0, 2.0, 3.0]),
            radius: 2.0,
        },
    );
    let e = extrema(&small, &big);
    let c = continua(&e);
    assert!(e.isolated.is_empty(), "{e:?}");
    assert!(
        (c[0] - 3.0).abs() < 1e-12 && (c[1] - 7.0).abs() < 1e-12,
        "{c:?}"
    );
}

#[test]
fn two_pipes_a_pin_in_a_hole_are_a_continuum() {
    // A pin of radius 1 in a hole of radius 1.2, a hair off the hole's
    // axis in the tolerance's reading.
    let hole = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.2,
    };
    let pin = Surface::Cylinder {
        frame: world([0.25 * tol().linear, 0.0, 1.0]),
        radius: 1.0,
    };
    let e = extrema(&pin, &hole);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert!(
        (c[0] - 0.2).abs() < 1e-6 && (c[1] - 2.2).abs() < 1e-6,
        "{c:?}"
    );
    well_formed(&pin, &hole, &e).unwrap();
    // Off the hole's axis by more than the tolerance the pin touches one
    // side: still a family along the ruling.
    let off = Surface::Cylinder {
        frame: world([0.1, 0.0, 1.0]),
        radius: 1.0,
    };
    let e = extrema(&off, &hole);
    assert!(e.isolated.is_empty(), "{e:?}");
    // The hole's rulings are 1.1 and 1.3 from the pin's axis: 0.1 and 0.3
    // from the pin's wall on the near side of it, 2.1 and 2.3 on the far.
    let c = continua(&e);
    let want = [0.1, 0.3, 2.1, 2.3];
    assert!(
        c.len() == 4 && c.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-9),
        "{c:?}"
    );
}

#[test]
fn two_skew_pipes_have_the_common_normal() {
    let a = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    };
    let b = Surface::Cylinder {
        frame: frame([5.0, 0.0, 2.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
        radius: 0.5,
    };
    let e = extrema(&a, &b);
    let d = distances(&e);
    // The axes are 5 apart along x: the pipes are 3.5 apart at the nearest.
    assert!((d[0] - 3.5).abs() < 1e-12, "{e:?}");
    well_formed(&a, &b, &e).unwrap();
    swaps(&a, &b, &e).unwrap();
}

#[test]
fn a_torus_against_a_plane_has_its_top_and_bottom_circles() {
    let ring = Surface::Torus {
        frame: world([0.0, 0.0, 4.0]),
        major_radius: 5.0,
        minor_radius: 1.0,
    };
    let floor = Surface::Plane {
        frame: Frame::world(),
    };
    let e = extrema(&ring, &floor);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert!(
        (c[0] - 3.0).abs() < 1e-12 && (c[1] - 5.0).abs() < 1e-12,
        "{c:?}"
    );
    well_formed(&ring, &floor, &e).unwrap();
    // Tilted, the circles become four isolated pairs.
    let tilted = Surface::Torus {
        frame: frame([0.0, 0.0, 12.0], [0.2, 0.1, 1.0], [1.0, 0.0, 0.0]),
        major_radius: 5.0,
        minor_radius: 1.0,
    };
    let e = extrema(&tilted, &floor);
    assert_eq!(e.isolated.len(), 4, "{e:?}");
    well_formed(&tilted, &floor, &e).unwrap();
    swaps(&tilted, &floor, &e).unwrap();
}

#[test]
fn parallel_planes_are_a_continuum_and_planes_that_meet_are_not() {
    let a = Surface::Plane {
        frame: world([0.0, 0.0, 0.0]),
    };
    let b = Surface::Plane {
        frame: frame([1.0, 1.0, 3.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]),
    };
    let e = extrema(&a, &b);
    assert!(e.isolated.is_empty());
    assert!((continua(&e)[0] - 3.0).abs() < 1e-12, "{e:?}");
    well_formed(&a, &b, &e).unwrap();
    let c = Surface::Plane {
        frame: frame([0.0, 0.0, 3.0], [0.0, 0.1, 1.0], [1.0, 0.0, 0.0]),
    };
    let e = extrema(&a, &c);
    assert!(e.isolated.is_empty() && e.continua.is_empty(), "{e:?}");
}

#[test]
fn a_plane_against_a_cone_is_a_family_along_a_ruling_where_it_is_tangent() {
    // The cone's half-angle is 30°: a plane whose normal makes 60° with
    // the axis (sin 30° = 0.5 of it along the axis) is parallel to a
    // tangent plane.
    let alpha = 30f64.to_radians();
    let cone = Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: alpha,
    };
    let n = [real::cos(alpha), 0.0, -real::sin(alpha)];
    let tangent_plane = Surface::Plane {
        frame: frame([6.0, 0.0, 0.0], n, [0.0, 1.0, 0.0]),
    };
    let e = extrema(&tangent_plane, &cone);
    assert!(e.isolated.is_empty(), "{e:?}");
    assert_eq!(e.continua.len(), 1, "{e:?}");
    // The ruling through (1, 0, 0) is at the distance of that point from
    // the plane through (6, 0, 0).
    let want = ((6.0 - 1.0) * real::cos(alpha)).abs();
    assert!(
        (e.continua[0].distance - want).abs() < 1e-12,
        "{e:?} want {want}"
    );
    well_formed(&tangent_plane, &cone, &e).unwrap();
    // A plane at another angle meets the cone: nothing at a distance.
    let other = Surface::Plane {
        frame: world([0.0, 0.0, 5.0]),
    };
    let e = extrema(&other, &cone);
    assert!(e.isolated.is_empty() && e.continua.is_empty(), "{e:?}");
}

#[test]
fn a_plane_along_an_elliptic_cylinder_has_two_rulings() {
    let tube = Surface::EllipticCylinder {
        frame: Frame::world(),
        major_radius: 3.0,
        minor_radius: 1.0,
    };
    // A plane along the axis, x = 5: the rulings at u = 0 and π are 2 and
    // 8 from it.
    let wall = Surface::Plane {
        frame: frame([5.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    };
    let e = extrema(&wall, &tube);
    assert!(e.isolated.is_empty(), "{e:?}");
    let c = continua(&e);
    assert!(
        (c[0] - 2.0).abs() < 1e-12 && (c[1] - 8.0).abs() < 1e-12,
        "{c:?}"
    );
    well_formed(&wall, &tube, &e).unwrap();
    swaps(&wall, &tube, &e).unwrap();
}

#[test]
fn coaxial_cones_of_one_angle_are_a_family_and_of_two_meet() {
    let alpha = 0.5f64;
    let a = Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: alpha,
    };
    let b = Surface::Cone {
        frame: world([0.0, 0.0, 2.0]),
        radius: 1.0,
        half_angle: alpha,
    };
    let e = extrema(&a, &b);
    assert!(e.isolated.is_empty(), "{e:?}");
    // Two parallel lines 2 apart along the axis: 2 sin α between them.
    assert!(
        (continua(&e)[0] - 2.0 * real::sin(alpha)).abs() < 1e-12,
        "{e:?}"
    );
    well_formed(&a, &b, &e).unwrap();
    let c = Surface::Cone {
        frame: world([0.0, 0.0, 2.0]),
        radius: 1.0,
        half_angle: 0.7,
    };
    let e = extrema(&a, &c);
    assert!(e.isolated.is_empty() && e.continua.is_empty(), "{e:?}");
    // Not coaxial: no reduction.
    let off = Surface::Cone {
        frame: world([1.0, 0.0, 2.0]),
        radius: 1.0,
        half_angle: alpha,
    };
    assert!(matches!(
        extrema_surface_surface(&a, &off, tol(), &mut Meter::default()),
        Err(GeomError::Unsupported { .. })
    ));
}

#[test]
fn elliptic_cylinders_with_parallel_axes_are_the_sections_pairs() {
    let a = Surface::EllipticCylinder {
        frame: Frame::world(),
        major_radius: 3.0,
        minor_radius: 1.0,
    };
    let b = Surface::EllipticCylinder {
        frame: world([9.0, 0.0, 4.0]),
        major_radius: 3.0,
        minor_radius: 2.0,
    };
    let e = extrema(&a, &b);
    // The ellipses face each other at their major vertices: 9 − 3 − 3.
    assert!(
        e.nearest()
            .is_some_and(|x| (x.distance - 3.0).abs() < 1e-12),
        "{e:?}"
    );
    well_formed(&a, &b, &e).unwrap();
    // Not parallel: no reduction.
    let tilt = Surface::EllipticCylinder {
        frame: frame([9.0, 0.0, 4.0], [0.1, 0.0, 1.0], [1.0, 0.0, 0.0]),
        major_radius: 3.0,
        minor_radius: 2.0,
    };
    assert!(matches!(
        extrema_surface_surface(&a, &tilt, tol(), &mut Meter::default()),
        Err(GeomError::Unsupported { .. })
    ));
}

#[test]
fn the_pairs_without_a_reduction_are_unsupported_by_name() {
    let cone = Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: 0.4,
    };
    let tube = Surface::EllipticCylinder {
        frame: Frame::world(),
        major_radius: 3.0,
        minor_radius: 1.0,
    };
    let patch = Surface::Nurbs(
        NurbsSurface::new(
            [1, 1],
            [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
            vec![
                p([0.0, 0.0, 0.0]),
                p([0.0, 1.0, 0.0]),
                p([1.0, 0.0, 0.0]),
                p([1.0, 1.0, 0.0]),
            ],
            vec![1.0; 4],
        )
        .unwrap(),
    );
    for (a, b) in [
        (&cone, &tube),
        (&cone, &patch),
        (&tube, &patch),
        (&patch, &patch),
    ] {
        let e = extrema_surface_surface(a, b, tol(), &mut Meter::default());
        assert!(
            matches!(&e, Err(GeomError::Unsupported { a: ka, b: kb })
                if *ka == a.kind().into() && *kb == b.kind().into()
                    || *ka == b.kind().into() && *kb == a.kind().into()),
            "{:?} vs {:?}: {e:?}",
            a.kind(),
            b.kind()
        );
    }
    let _ = SurfaceKind::Nurbs;
}

// --- a NURBS surface ------------------------------------------------------------

fn bowl() -> Surface {
    // A bicubic patch with a dip in the middle, its control net a lattice
    // with the interior points lowered.
    let mut points = Vec::new();
    for i in 0..4 {
        for j in 0..4 {
            let dip = if (1..=2).contains(&i) && (1..=2).contains(&j) {
                -2.0
            } else {
                0.0
            };
            points.push(p([
                3.0 * i as f64,
                3.0 * j as f64,
                dip + 0.3 * (i as f64 - 1.5) * (j as f64 - 1.5),
            ]));
        }
    }
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    Surface::Nurbs(
        NurbsSurface::new([3, 3], [knots.clone(), knots], points, vec![1.0; 16]).unwrap(),
    )
}

#[test]
fn a_nurbs_surface_against_a_plane_has_its_critical_heights() {
    let bowl = bowl();
    let floor = Surface::Plane {
        frame: world([0.0, 0.0, -6.0]),
    };
    let e = extrema(&floor, &bowl);
    assert!(!e.isolated.is_empty(), "{e:?}");
    well_formed(&floor, &bowl, &e).unwrap();
    // The nearest critical height is the dip's lowest: above the floor by
    // the surface's lowest z plus 6.
    let lowest = (0..=200)
        .flat_map(|i| (0..=200).map(move |j| (i, j)))
        .map(|(i, j)| bowl.point(i as f64 / 200.0, j as f64 / 200.0).z)
        .fold(f64::INFINITY, f64::min);
    let nearest = e.nearest().unwrap();
    assert!(
        (nearest.distance - (lowest + 6.0)).abs() < 1e-3,
        "{nearest:?} {lowest}"
    );
    swaps(&floor, &bowl, &e).unwrap();
}

#[test]
fn a_nurbs_surface_against_a_ball_is_the_critical_feet_of_its_centre() {
    let bowl = bowl();
    let ball = Surface::Sphere {
        frame: world([4.5, 4.5, 7.0]),
        radius: 1.0,
    };
    let e = extrema(&ball, &bowl);
    assert!(!e.isolated.is_empty(), "{e:?}");
    well_formed(&ball, &bowl, &e).unwrap();
    let feet = bowl
        .feet(p([4.5, 4.5, 7.0]), tol(), &mut Meter::default())
        .unwrap();
    let near = feet
        .isolated
        .iter()
        .map(|f| f.distance)
        .fold(f64::INFINITY, f64::min);
    assert!((e.nearest().unwrap().distance - (near - 1.0).abs()).abs() < 1e-9);
}

// --- at random ------------------------------------------------------------------

/// Where the first surface is sampled: its angle round, the other
/// parameter within a few scales.
fn grid(s: &Surface) -> Option<([f64; 2], [f64; 2])> {
    let reach = 1.5 * DEFAULT_SCALE;
    Some(match s {
        Surface::Plane { .. } => ([-reach, reach], [-reach, reach]),
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } => {
            ([0.0, core::f64::consts::TAU], [-reach, reach])
        }
        Surface::Cone { .. } => ([0.0, core::f64::consts::TAU], [-reach, reach]),
        Surface::Sphere { .. } => ([0.0, core::f64::consts::TAU], [-FRAC_PI_2, FRAC_PI_2]),
        Surface::Torus { .. } => ([0.0, core::f64::consts::TAU], [0.0, core::f64::consts::TAU]),
        Surface::Nurbs(_) => return None,
    })
}

const N: usize = 90;

/// The distance from the point of `a` at `uv` to `b`, by the closed-form
/// projection; `None` where it is ambiguous.
fn gap(a: &Surface, b: &Surface, uv: [f64; 2]) -> Option<f64> {
    b.project(a.point(uv[0], uv[1])).ok().map(|x| x.distance)
}

/// Every local minimum of a dense sample of `a`, the other surface
/// eliminated by its projection, that is farther than `floor` from it, is
/// a pair of the answer or on a continuum at its distance.
fn minima_held(a: &Surface, b: &Surface, e: &SurfaceExtrema) -> Result<(), String> {
    let Some((ur, vr)) = grid(a) else {
        return Ok(());
    };
    let periodic_u = (ur[1] - ur[0] - core::f64::consts::TAU).abs() < 1e-12;
    let periodic_v = matches!(a, Surface::Torus { .. });
    let at = |i: usize, j: usize| {
        [
            ur[0] + (ur[1] - ur[0]) * i as f64 / N as f64,
            vr[0] + (vr[1] - vr[0]) * j as f64 / N as f64,
        ]
    };
    let g: Vec<Vec<Option<f64>>> = (0..=N)
        .map(|i| (0..=N).map(|j| gap(a, b, at(i, j))).collect())
        .collect();
    let floor = 1e-3 * DEFAULT_SCALE;
    let idx = |k: i64, periodic: bool| -> Option<usize> {
        if periodic {
            Some(k.rem_euclid(N as i64) as usize)
        } else if (0..=N as i64).contains(&k) {
            Some(k as usize)
        } else {
            None
        }
    };
    for i in 0..=N {
        for j in 0..=N {
            let Some(here) = g[i][j] else { continue };
            if here <= floor {
                continue;
            }
            let mut lowest = true;
            'around: for di in -1i64..=1 {
                for dj in -1i64..=1 {
                    if (di, dj) == (0, 0) {
                        continue;
                    }
                    let (Some(ni), Some(nj)) = (
                        idx(i as i64 + di, periodic_u),
                        idx(j as i64 + dj, periodic_v),
                    ) else {
                        // An edge of the sampled window: not interior.
                        lowest = false;
                        break 'around;
                    };
                    match g[ni][nj] {
                        Some(other) if other >= here => {}
                        _ => {
                            lowest = false;
                            break 'around;
                        }
                    }
                }
            }
            if !lowest {
                continue;
            }
            // Pattern search around the sample for the minimum.
            let mut uv = at(i, j);
            let mut step = [(ur[1] - ur[0]) / N as f64, (vr[1] - vr[0]) / N as f64];
            let f = |uv: [f64; 2]| gap(a, b, uv).unwrap_or(f64::INFINITY);
            let mut best = f(uv);
            for _ in 0..400 {
                let mut moved = false;
                for dir in [
                    [1.0, 0.0],
                    [-1.0, 0.0],
                    [0.0, 1.0],
                    [0.0, -1.0],
                    [1.0, 1.0],
                    [1.0, -1.0],
                    [-1.0, 1.0],
                    [-1.0, -1.0],
                ] {
                    let cand = [uv[0] + dir[0] * step[0], uv[1] + dir[1] * step[1]];
                    let value = f(cand);
                    if value < best {
                        (uv, best, moved) = (cand, value, true);
                    }
                }
                if !moved {
                    step = [0.5 * step[0], 0.5 * step[1]];
                    if step[0] < 1e-12 && step[1] < 1e-12 {
                        break;
                    }
                }
            }
            // Sinking into the surfaces' meeting is not a minimum to hold,
            // and nor is a slope too shallow for the sample to see: the
            // point has to be the lowest of a ring two sample steps round.
            if best <= floor {
                continue;
            }
            let ring = (0..16).all(|k| {
                let angle = core::f64::consts::TAU * k as f64 / 16.0;
                let cand = [
                    uv[0] + 2.0 * (ur[1] - ur[0]) / N as f64 * real::cos(angle),
                    uv[1] + 2.0 * (vr[1] - vr[0]) / N as f64 * real::sin(angle),
                ];
                f(cand) >= best - 1e-9
            });
            if !ring {
                continue;
            }
            let point = a.point(uv[0], uv[1]);
            let held = e.isolated.iter().any(|x| {
                (x.point_a - point).norm() <= 1e-4 * DEFAULT_SCALE
                    || ((x.distance - best).abs() <= tol().linear)
            }) || e
                .continua
                .iter()
                .any(|x| (x.distance - best).abs() <= tol().linear);
            if !held {
                return Err(format!(
                    "the sample's local minimum at {uv:?} ({best}) is not in {e:?}"
                ));
            }
        }
    }
    Ok(())
}

fn analytic_pair() -> impl Strategy<Value = (Surface, Surface)> {
    // Every pair that has a reduction: not cone–cone, cone–elliptic, nor
    // elliptic–elliptic (which needs a parallel axis).
    let any = || surface().boxed();
    (any(), any()).prop_filter("a pair with a reduction", |(a, b)| {
        let free =
            |s: &Surface| matches!(s, Surface::Cone { .. } | Surface::EllipticCylinder { .. });
        !(free(a) && free(b))
    })
}

arris_debug::prop_shards! {
    /// Random pairs of analytic surfaces with a reduction.
    random_pairs [s0 s1 s2 s3] ((a, b)) = analytic_pair() => {
        let e = extrema(&a, &b);
        well_formed(&a, &b, &e).map_err(TestCaseError::fail)?;
        minima_held(&a, &b, &e).map_err(TestCaseError::fail)?;
        swaps(&a, &b, &e).map_err(TestCaseError::fail)?;
        Ok(())
    }
}

arris_debug::prop_shards! {
    /// A sphere or a cylinder against anything, the cases the reduction is
    /// built for, with the sphere's centre on or off the other's axis.
    offsets [o0 o1] ((a, b)) = (prop_oneof![sphere(), cylinder(), torus()], prop_oneof![plane(), cylinder(), cone(), sphere(), torus(), elliptic_cylinder()]) => {
        let e = extrema(&a, &b);
        well_formed(&a, &b, &e).map_err(TestCaseError::fail)?;
        minima_held(&a, &b, &e).map_err(TestCaseError::fail)?;
        Ok(())
    }
}

#[test]
fn the_strategies_are_in_scope() {
    // Keep the imports honest for the named strategies.
    let _ = (finite_f64(0.0..=1.0), cone(), plane());
}

// --- the critical points of a NURBS surface -------------------------------------

/// A sampled field over a NURBS surface's domain: every interior local
/// minimum and maximum of the grid, refined by a pattern search on the
/// field, is among the critical points the surface reports.
fn field_extremes_held(
    n: &NurbsSurface,
    field: &dyn Fn(Point3) -> f64,
    reported: &[Point3],
    families: &[Point3],
    tol: Tolerance,
) -> Result<(), String> {
    const M: usize = 48;
    let [du, dv] = n.domain();
    let at = |i: usize, j: usize| {
        [
            du.lo() + (du.hi() - du.lo()) * i as f64 / M as f64,
            dv.lo() + (dv.hi() - dv.lo()) * j as f64 / M as f64,
        ]
    };
    let val = |uv: [f64; 2]| field(n.eval(uv[0], uv[1]).point);
    let g: Vec<Vec<f64>> = (0..=M)
        .map(|i| (0..=M).map(|j| val(at(i, j))).collect())
        .collect();
    for sign in [1.0, -1.0] {
        for i in 1..M {
            for j in 1..M {
                let here = sign * g[i][j];
                let lowest = (-1i64..=1).all(|di| {
                    (-1i64..=1).all(|dj| {
                        sign * g[(i as i64 + di) as usize][(j as i64 + dj) as usize] >= here
                    })
                });
                if !lowest {
                    continue;
                }
                let mut uv = at(i, j);
                let mut step = [
                    (du.hi() - du.lo()) / M as f64,
                    (dv.hi() - dv.lo()) / M as f64,
                ];
                let mut best = here;
                for _ in 0..400 {
                    let mut moved = false;
                    for dir in [
                        [1.0, 0.0],
                        [-1.0, 0.0],
                        [0.0, 1.0],
                        [0.0, -1.0],
                        [1.0, 1.0],
                        [1.0, -1.0],
                        [-1.0, 1.0],
                        [-1.0, -1.0],
                    ] {
                        let cand = [
                            (uv[0] + dir[0] * step[0]).clamp(du.lo(), du.hi()),
                            (uv[1] + dir[1] * step[1]).clamp(dv.lo(), dv.hi()),
                        ];
                        let value = sign * val(cand);
                        if value < best {
                            (uv, best, moved) = (cand, value, true);
                        }
                    }
                    if !moved {
                        step = [0.5 * step[0], 0.5 * step[1]];
                        if step[0] < 1e-13 && step[1] < 1e-13 {
                            break;
                        }
                    }
                }
                // On the domain's boundary the extremum is not a critical
                // point of the surface.
                let on_edge = (uv[0] - du.lo()).abs() < 1e-9
                    || (uv[0] - du.hi()).abs() < 1e-9
                    || (uv[1] - dv.lo()).abs() < 1e-9
                    || (uv[1] - dv.hi()).abs() < 1e-9;
                if on_edge {
                    continue;
                }
                // A kink along a knot line of a piecewise-linear direction
                // is the topology's, as a curve's knot is.
                let kinked = (0..2).any(|k| {
                    n.degree()[k] == 1
                        && n.knots()[k].iter().any(|&knot| (knot - uv[k]).abs() < 1e-6)
                });
                if kinked {
                    continue;
                }
                let point = n.eval(uv[0], uv[1]).point;
                let on_family = families
                    .iter()
                    .any(|f| (field(*f) - field(point)).abs() <= tol.linear);
                if !on_family
                    && !reported
                        .iter()
                        .any(|r| (r - point).norm() <= 1e-5 * DEFAULT_SCALE.max(1.0))
                {
                    return Err(format!(
                        "the grid's {} at {uv:?} {point:?} is not among {} reported points",
                        if sign > 0.0 { "minimum" } else { "maximum" },
                        reported.len()
                    ));
                }
            }
        }
    }
    Ok(())
}

arris_debug::prop_shards! {
    /// The feet of a point on a random NURBS surface: every one stationary,
    /// and every local minimum and maximum of the sampled distance among
    /// them.
    nurbs_feet [n0 n1 n2] ((s, q)) = (arris_debug::prop::geom::nurbs_surface(), arris_debug::prop::point_in_box(DEFAULT_SCALE)) => {
        let surface = Surface::Nurbs(s.clone());
        let feet = surface.feet(q, tol(), &mut Meter::default()).unwrap();
        for f in feet.isolated.iter().chain(&feet.continua) {
            let e = s.eval(f.uv.x, f.uv.y);
            prop_assert!((e.point - f.point).norm() <= 1e-9 * DEFAULT_SCALE);
            let r = e.point - q;
            for d in [e.du, e.dv] {
                let n = d.norm();
                prop_assert!(n == 0.0 || (r.dot(&d) / n).abs() <= tol().linear, "{f:?} is not stationary");
            }
        }
        let reported: Vec<Point3> = feet.isolated.iter().map(|f| f.point).collect();
        let families: Vec<Point3> = feet.continua.iter().map(|f| f.point).collect();
        field_extremes_held(&s, &|x| (x - q).norm(), &reported, &families, tol()).map_err(TestCaseError::fail)?;
        Ok(())
    }
}

arris_debug::prop_shards! {
    /// A random NURBS surface against a plane: every stationary height
    /// reported, every sampled extreme of the height among them.
    nurbs_heights [h0 h1 h2] ((s, f)) = (arris_debug::prop::geom::nurbs_surface(), arris_debug::prop::frame()) => {
        let plane = Surface::Plane { frame: f };
        let nurbs = Surface::Nurbs(s.clone());
        let e = extrema(&plane, &nurbs);
        well_formed(&plane, &nurbs, &e).map_err(TestCaseError::fail)?;
        let reported: Vec<Point3> = e.isolated.iter().map(|x| x.point_b).collect();
        let families: Vec<Point3> = e.continua.iter().map(|x| x.point_b).collect();
        let n = f.z().into_inner();
        field_extremes_held(&s, &|x| (x - f.origin()).dot(&n), &reported, &families, tol()).map_err(TestCaseError::fail)?;
        swaps(&plane, &nurbs, &e).map_err(TestCaseError::fail)?;
        Ok(())
    }
}
