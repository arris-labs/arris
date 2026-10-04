//! `Surface::project` onto a NURBS surface finds the global nearest point
//! (`docs/DATA-MODEL.md` §Surfaces, ADR-0025 §1).
//!
//! Two oracles. The twins of `Surface::to_nurbs` are the analytic surfaces,
//! whose closed-form `project` is exact: over random poses, bounds and
//! query points — near a seam, at a pole, on the surface, far off it — the
//! twin must find the same point at the same distance. And free-form
//! surfaces have no closed form, so what is checked there is what
//! globality means: no sample of the surface is nearer than the answer,
//! and the answer is a point of the surface that no step of Newton's
//! iteration improves.

mod common;

use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::nurbs_surface;
use arris_debug::prop::{check, finite_f64, frame, point_in_box, radius, unit_vec3};
use arris_geom::{AmbiguousLocus, GeomError, NurbsSurface, Surface};
use arris_math::{Frame, Interval, Point3, Vec3};
use common::*;
use proptest::prelude::*;

/// Where a projected point may differ from the closed form's: a minimum
/// is located to the square root of rounding, its distance to rounding.
const POINT: f64 = 1e-7;

/// A surface of any analytic kind with bounds its twin is built over.
fn case() -> impl Strategy<Value = (Surface, [Interval; 2])> {
    use arris_debug::prop::geom::{cone, cylinder, elliptic_cylinder, plane, sphere, torus};
    prop_oneof![
        (plane(), line_range(), line_range()).prop_map(|(s, u, v)| (s, [u, v])),
        (cylinder(), angle_range(), line_range()).prop_map(|(s, u, v)| (s, [u, v])),
        (elliptic_cylinder(), angle_range(), line_range()).prop_map(|(s, u, v)| (s, [u, v])),
        (cone(), angle_range(), line_range()).prop_map(|(s, u, v)| (s, [u, v])),
        (sphere(), angle_range(), latitude_range()).prop_map(|(s, u, v)| (s, [u, v])),
        (torus(), angle_range(), angle_range()).prop_map(|(s, u, v)| (s, [u, v])),
    ]
}

/// Whether the analytic parameters `uv` are inside `bounds`, up to a
/// margin, wrapping a periodic parameter.
fn inside(surface: &Surface, uv: [f64; 2], bounds: [Interval; 2], margin: f64) -> bool {
    let period = surface.period();
    (0..2).all(|k| match period[k] {
        Some(turn) => {
            let a = (uv[k] - bounds[k].lo()).rem_euclid(turn);
            a <= bounds[k].length() + margin || a >= turn - margin
        }
        None => bounds[k].lo() - margin <= uv[k] && uv[k] <= bounds[k].hi() + margin,
    })
}

#[test]
fn a_twin_projects_where_the_analytic_surface_does() {
    check(
        (
            case(),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            unit_vec3(),
            prop_oneof![
                Just(0.0),
                Just(1e-6),
                finite_f64(0.01..=3.0),
                finite_f64(10.0..=300.0)
            ],
        ),
        |((surface, bounds), s, t, direction, magnitude)| {
            let on = surface.point(bounds[0].lerp(s), bounds[1].lerp(t));
            let query = on + magnitude * direction.into_inner();
            let Ok(closed) = surface.project(query) else {
                // A tie of the closed form: the twin's tie is checked below.
                return Ok(());
            };
            if !inside(&surface, [closed.uv.x, closed.uv.y], bounds, 1e-9) {
                return Ok(());
            }
            let twin = surface.to_nurbs(bounds).unwrap();
            let found = twin.project(query);
            let found = match found {
                Ok(found) => found,
                Err(e) => {
                    return Err(TestCaseError::fail(format!(
                        "the closed form finds {closed:?}, the twin says {e}"
                    )));
                }
            };
            prop_assert!(
                (found.distance - closed.distance).abs() <= EXACT,
                "distance {} against {}",
                found.distance,
                closed.distance
            );
            prop_assert!(
                (found.point - closed.point).norm() <= POINT,
                "point {} from the closed form's",
                (found.point - closed.point).norm()
            );
            // The parameters are the point's, inside the twin's domain.
            let [du, dv] = twin.domain();
            prop_assert!(du.contains(found.uv.x) && dv.contains(found.uv.y));
            prop_assert!((twin.eval(found.uv.x, found.uv.y).point - found.point).norm() <= EXACT);
            prop_assert!((found.point - query).norm() - found.distance <= EXACT);
            Ok(())
        },
    );
}

#[test]
fn a_pole_is_a_row_with_u_at_the_start_of_the_knots() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            angle_range(),
            finite_f64(1.05..=50.0),
            any::<bool>(),
        ),
        |(f, r, u, height, north)| {
            let sphere = Surface::Sphere {
                frame: f,
                radius: r,
            };
            let twin = sphere
                .to_nurbs([u, Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap()])
                .unwrap();
            let sign = if north { 1.0 } else { -1.0 };
            let found = twin
                .project(f.origin() + sign * height * r * f.z().into_inner())
                .unwrap();
            let pole = f.origin() + sign * r * f.z().into_inner();
            prop_assert!((found.point - pole).norm() <= POINT, "not the pole");
            prop_assert!((found.distance - (height - 1.0) * r).abs() <= EXACT);
            let [du, dv] = twin.domain();
            prop_assert_eq!(found.uv.x, du.lo());
            prop_assert_eq!(found.uv.y, if north { dv.hi() } else { dv.lo() });
            Ok(())
        },
    );
}

/// A pole found a hair off its row, far from the origin: the sphere the
/// nightly drew at `ARRIS_PROPTEST_SEED=29a5102c…` is projected onto from a
/// point on its axis, and the minimum lands 1e-13 short of the pole in `v`,
/// where the derivative in `u` is too large for rounding to call it zero.
/// The pole is still one point, reported at `u` at the start of the knots
/// and `v` at the row.
#[test]
fn a_pole_found_a_hair_off_its_row_is_reported_on_it() {
    let f = Frame::from_orthonormal(
        Point3::new(-59.860638027004754, 59.554181674241875, -74.581896414637),
        Vec3::new(0.1733734454526067, 0.8695476117044877, -0.4624160457757859),
        Vec3::new(-0.2607258009695479, 0.49329854358465286, 0.8298666179610119),
        Vec3::new(
            0.9497176975926477,
            -0.023313040905939775,
            0.3122383656808341,
        ),
    )
    .unwrap();
    let r = 5.156992731543929;
    let sphere = Surface::Sphere {
        frame: f,
        radius: r,
    };
    let u = Interval::new(1.1858312516335736, 2.1663736345674307).unwrap();
    let twin = sphere
        .to_nurbs([u, Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap()])
        .unwrap();
    let found = twin
        .project(f.origin() + 29.49995745801499 * r * f.z().into_inner())
        .unwrap();
    let [du, dv] = twin.domain();
    assert_eq!(found.uv.x, du.lo());
    assert_eq!(found.uv.y, dv.hi());
}

#[test]
fn a_point_on_the_seam_is_reported_at_the_start_of_the_knots() {
    check(
        (
            prop_oneof![
                (frame(), radius(0.1..=10.0)).prop_map(|(f, r)| Surface::Cylinder {
                    frame: f,
                    radius: r
                }),
                (frame(), radius(0.1..=10.0)).prop_map(|(f, r)| Surface::Sphere {
                    frame: f,
                    radius: r
                }),
            ],
            finite_f64(-7.0..=7.0),
            finite_f64(-1.0..=1.0),
            prop_oneof![Just(0.0), finite_f64(0.01..=5.0)],
        ),
        |(surface, lo, t, outward)| {
            // A full turn starting at `lo`: its seam is the half-plane at
            // the angle `lo`.
            let bounds = [
                Interval::new(lo, lo + TAU).unwrap(),
                match surface {
                    Surface::Sphere { .. } => Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap(),
                    _ => Interval::new(-2.0, 2.0).unwrap(),
                },
            ];
            let v = match surface {
                Surface::Sphere { .. } => 1.2 * t,
                _ => 1.5 * t,
            };
            let on = surface.point(lo, v);
            let n = surface.normal(lo, v).unwrap().into_inner();
            let query = on + outward * n;
            let twin = surface.to_nurbs(bounds).unwrap();
            let found = twin.project(query).unwrap();
            prop_assert!((found.point - on).norm() <= POINT);
            prop_assert_eq!(found.uv.x, twin.domain()[0].lo());
            Ok(())
        },
    );
}

/// Whether a projection says two distinct points tie.
fn is_medial(r: &Result<arris_geom::SurfaceProjection, GeomError>) -> bool {
    matches!(
        r,
        Err(GeomError::Ambiguous {
            locus: AmbiguousLocus::MedialAxis,
            ..
        })
    )
}

#[test]
fn a_tie_is_ambiguous_and_never_chosen() {
    check(
        (
            frame(),
            radius(0.1..=10.0),
            radius(0.1..=10.0),
            finite_f64(-3.0..=3.0),
        ),
        |(f, r, len, along)| {
            let full = [Interval::TURN, Interval::new(-len, len).unwrap()];
            // Every point of a circle of the cylinder is as near to its axis.
            let cylinder = Surface::Cylinder {
                frame: f,
                radius: r,
            }
            .to_nurbs(full)
            .unwrap();
            let on_axis = f.origin() + (along * len / 3.0) * f.z().into_inner();
            let found = cylinder.project(on_axis);
            prop_assert!(is_medial(&found), "the axis: {found:?}");
            // And every point of a sphere from its centre.
            let ball = Surface::Sphere {
                frame: f,
                radius: r,
            }
            .to_nurbs([
                Interval::TURN,
                Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap(),
            ])
            .unwrap();
            let found = ball.project(f.origin());
            prop_assert!(is_medial(&found), "the centre: {found:?}");
            Ok(())
        },
    );
}

#[test]
fn a_point_that_is_not_finite_is_refused() {
    let patch = Surface::Plane {
        frame: arris_math::Frame::world(),
    }
    .to_nurbs([Interval::UNIT, Interval::UNIT])
    .unwrap();
    let e = patch.project(Point3::new(f64::NAN, 0.0, 0.0)).unwrap_err();
    assert!(e.to_string().contains("not finite"), "{e}");
}

/// The samples of `s` on a `n × n` grid.
fn grid(s: &NurbsSurface, n: usize) -> impl Iterator<Item = Point3> + '_ {
    let [du, dv] = s.domain();
    (0..=n).flat_map(move |i| {
        (0..=n).map(move |j| {
            s.eval(du.lerp(i as f64 / n as f64), dv.lerp(j as f64 / n as f64))
                .point
        })
    })
}

#[test]
fn on_a_free_form_surface_nothing_is_nearer_than_the_answer() {
    check((nurbs_surface(), point_in_box(150.0)), |(s, query)| {
        let found = match s.project(query) {
            Ok(found) => found,
            // A tie between distinct points is the answer to a query
            // that has no nearest: the surface is not what is at fault.
            Err(GeomError::Ambiguous { .. }) => return Ok(()),
            Err(e) => return Err(TestCaseError::fail(format!("{e}"))),
        };
        let nearest_sample = grid(&s, 48)
            .map(|p| (p - query).norm())
            .fold(f64::INFINITY, f64::min);
        prop_assert!(
            found.distance <= nearest_sample + EXACT,
            "a sample is nearer: {nearest_sample} against {}",
            found.distance
        );
        // On the surface, at the parameters it names.
        let at = s.eval(found.uv.x, found.uv.y).point;
        prop_assert!((at - found.point).norm() <= EXACT);
        prop_assert!(((found.point - query).norm() - found.distance).abs() <= EXACT);
        // A local minimum: no point a little way off, in any of eight
        // directions and three step lengths, is nearer. (A kink of a
        // degree-one direction or the end of the domain has a gradient
        // that does not vanish, so the gradient is not what is asked.)
        let [du, dv] = s.domain();
        for h in [1e-4, 1e-3, 1e-2] {
            for (a, b) in [
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ] {
                let u = du.clamp(found.uv.x + f64::from(a) * h * du.length());
                let v = dv.clamp(found.uv.y + f64::from(b) * h * dv.length());
                let d = (s.eval(u, v).point - query).norm();
                prop_assert!(d >= found.distance - EXACT, "({u}, {v}) is nearer: {d}");
            }
        }
        Ok(())
    });
}

#[test]
fn projecting_twice_gives_the_same_answer() {
    check((nurbs_surface(), point_in_box(150.0)), |(s, query)| {
        prop_assert_eq!(s.project(query), s.project(query));
        Ok(())
    });
}

/// A saddle `z = (x² − y²) / 4` as a Bézier patch of degree two over
/// `[−2, 2]²`: a surface with no closed form and two directions of
/// curvature of opposite sign.
fn saddle() -> NurbsSurface {
    // `x² / 4` over `[−2, 2]` has the Bézier coefficients `1, −1, 1`, and
    // so has `y² / 4`: `z` is their difference at each control point.
    let a = [1.0, -1.0, 1.0];
    let mut points = Vec::new();
    for i in 0..3 {
        for j in 0..3 {
            points.push(Point3::new(
                -2.0 + 2.0 * i as f64,
                -2.0 + 2.0 * j as f64,
                a[i] - a[j],
            ));
        }
    }
    let k = vec![-2.0, -2.0, -2.0, 2.0, 2.0, 2.0];
    NurbsSurface::new([2, 2], [k.clone(), k], points, vec![1.0; 9]).unwrap()
}

#[test]
fn a_saddle_has_a_nearest_point_on_each_side_of_it() {
    let s = saddle();
    // Above the middle of the saddle, nearer than its centre of curvature
    // along the rising ridge (at height 2), the nearest point is the middle.
    let above = s.project(Point3::new(0.0, 0.0, 1.0)).unwrap();
    assert!((above.distance - 1.0).abs() < 1e-12);
    assert!(above.uv.x.abs() < 1e-7 && above.uv.y.abs() < 1e-7);
    // Off to the side, high above one ridge, the nearest is on that ridge.
    let ridge = s.project(Point3::new(0.8, 0.0, 2.0)).unwrap();
    let e = s.eval(ridge.uv.x, ridge.uv.y);
    let r = e.point - Point3::new(0.8, 0.0, 2.0);
    assert!(e.du.dot(&r).abs() < 1e-9 && e.dv.dot(&r).abs() < 1e-9);
    // Nothing on a fine grid is nearer than either answer.
    for q in [Point3::new(0.0, 0.0, 1.0), Point3::new(0.8, 0.0, 2.0)] {
        let found = s.project(q).unwrap();
        assert!(grid(&s, 200).all(|p| (p - q).norm() >= found.distance - 1e-12));
    }
}

/// A shrunk nightly failure (2026-10-03): a degree-two surface with a
/// collapsed row and zero-length rows, projected from far off.
#[test]
fn a_collapsed_free_form_surface_projects_to_a_local_minimum() {
    let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
    let s = NurbsSurface::new(
        [2, 2],
        [
            vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0],
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        ],
        vec![
            p(0.0, 98.97075805024689, 0.0),
            p(0.0, 0.0, 0.0),
            p(0.0, 0.0, 0.0),
            p(59.931188404822265, 93.9917037091706, 72.06706461149167),
            p(-79.52709665070022, 0.0, -72.2885915723671),
            p(91.19530701777033, 0.0, 53.086060679742424),
            p(-22.42383534955861, -82.71338886087106, -37.61962428763778),
            p(30.094062294987765, 45.55712961399046, -25.470289364899926),
            p(0.0, 77.78519767693746, -72.92432559197552),
            p(56.95881267330661, 65.6463169587858, 62.65888662330536),
            p(0.0, 0.0, 71.98310152863759),
            p(0.0, 0.0, 47.348846354308),
            p(0.0, 0.0, 0.0),
            p(0.0, 0.0, 0.0),
            p(0.0, 0.0, 0.0),
        ],
        vec![
            0.5,
            0.5,
            0.5,
            1.5639621212169927,
            0.5,
            0.5,
            1.1619052784166612,
            0.7405647503340199,
            1.7561345525533654,
            1.0504395748213629,
            1.044177994850871,
            0.9678746365591391,
            0.5,
            0.5,
            0.5,
        ],
    )
    .unwrap();
    let query = p(-51.32972353272771, -36.67971822965331, -141.03093257268955);
    let found = s.project(query).unwrap();
    // The nearer neighbour the property found, a step toward the wall.
    let at = s.eval(1.5902181832202618, 0.006125297360794047).point;
    assert!((at - query).norm() >= found.distance - EXACT);
}

/// A second shrunk nightly failure: a degree-(2, 1) surface with rows that
/// collapse onto an axis, projected from far off.
#[test]
fn a_ruled_free_form_surface_projects_to_the_nearest_sample() {
    let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
    let s = NurbsSurface::new(
        [2, 1],
        [
            vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0],
            vec![0.0, 0.0, 1.0, 1.0],
        ],
        vec![
            p(0.0, 10.603936181731585, 0.0),
            p(0.0, 0.0, 0.0),
            p(0.0, 69.9222749342298, 0.0),
            p(-63.70506808049978, 0.0, 0.0),
            p(-26.26604145048718, -13.638621631034724, -2.2997382899379804),
            p(98.6556737820459, 0.0, 99.85024829356065),
            p(-0.33967378583339636, -37.56735773616384, 9.980743096301868),
            p(-69.202664930244, 0.0, 0.0),
            p(3.495134750533163, -3.769839106383314, -20.081536074790943),
            p(15.429627890646223, 53.01335508485869, -73.8209604899312),
        ],
        vec![
            0.5,
            0.5,
            0.5,
            0.5,
            1.217191219584153,
            0.5,
            1.0078761245036028,
            1.979827241818139,
            1.3522559861324497,
            0.5,
        ],
    )
    .unwrap();
    let query = p(65.45517322761013, -87.34417497143984, -99.72239786953133);
    let found = s.project(query).unwrap();
    let nearest_sample = grid(&s, 48)
        .map(|p| (p - query).norm())
        .fold(f64::INFINITY, f64::min);
    assert!(found.distance <= nearest_sample + EXACT);
}
