//! The (u, v) toolkit: discretised loops have the areas, winding numbers
//! and intersections their pcurves say, across the seam range too, and
//! the region integral recovers areas and the sample cylinder's volume;
//! `point_side` and `interior_point` answer for a region what a
//! boolean's piece classification asks (ADR-0004).

use arris_debug::testing::EXACT;
use arris_math::real;
use core::f64::consts::{PI, TAU};

use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, radius};
use arris_debug::sample;
use arris_geom::integrate::{self, region_integral};
use arris_geom::region2::{
    Piece, Polygon2, Side, SideIndex, discretise, interior_point, point_side,
};
use arris_geom::{Curve2, Surface};
use arris_math::predicates::{Sign, orient2d};
use arris_math::{Frame2, Handedness, Interval, Point2, Point3, UnitVec2, Vec2};
use arris_topo::{Model, Orientation};
use proptest::prelude::*;

fn line_between(a: Point2, b: Point2) -> (Curve2, Interval) {
    let d = b - a;
    (
        Curve2::Line {
            origin: a,
            direction: UnitVec2::new_normalize(d),
        },
        Interval::new(0.0, d.norm()).unwrap(),
    )
}

/// The polygon of straight pieces through `corners`, closed.
fn polygon_of(corners: &[Point2], chord: f64) -> Polygon2 {
    let sides: Vec<(Curve2, Interval)> = (0..corners.len())
        .map(|i| line_between(corners[i], corners[(i + 1) % corners.len()]))
        .collect();
    let pieces: Vec<Piece<'_>> = sides.iter().map(|(c, r)| Piece::along(c, *r)).collect();
    discretise(&pieces, chord)
}

/// An axis-aligned rectangle from `(u0, v0)` of size `w × h`, counter-clockwise.
fn rectangle(u0: f64, v0: f64, w: f64, h: f64) -> [Point2; 4] {
    [
        Point2::new(u0, v0),
        Point2::new(u0 + w, v0),
        Point2::new(u0 + w, v0 + h),
        Point2::new(u0, v0 + h),
    ]
}

#[test]
fn a_discretised_circle_of_either_handedness_has_its_signed_area_within_the_chord_bound() {
    check(
        (
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
            radius(0.1..=DEFAULT_SCALE),
            any::<bool>(),
            finite_f64(-6.0..=-2.0),
        ),
        |(u, v, angle, r, right, log_tol)| {
            let handedness = if right {
                Handedness::Right
            } else {
                Handedness::Left
            };
            let circle = Curve2::Circle {
                frame: Frame2::new(
                    Point2::new(u, v),
                    Vec2::new(real::cos(angle), real::sin(angle)),
                    handedness,
                )
                .unwrap(),
                radius: r,
            };
            let tol = real::powf(10f64, log_tol) * r;
            let polygon = discretise(&[Piece::along(&circle, Interval::TURN)], tol);
            prop_assert!(polygon.chord_deviation() <= tol);
            let area = polygon.signed_area();
            let expected = if right { PI * r * r } else { -PI * r * r };
            // The polygon is inscribed: the area it misses is at most the
            // perimeter times the chord deviation.
            prop_assert!(
                (area - expected).abs() <= TAU * r * tol + EXACT * r,
                "area {area} vs {expected} at chord {tol}"
            );
            prop_assert!(polygon.self_intersections().is_empty());
            Ok(())
        },
    );
}

#[test]
fn the_winding_number_of_a_point_against_a_convex_polygon_is_the_half_plane_test() {
    check(
        (
            proptest::collection::vec(finite_f64(0.0..=TAU), 3..12),
            radius(1.0..=DEFAULT_SCALE),
            finite_f64(-2.0 * DEFAULT_SCALE..=2.0 * DEFAULT_SCALE),
            finite_f64(-2.0 * DEFAULT_SCALE..=2.0 * DEFAULT_SCALE),
        ),
        |(mut angles, r, px, py)| {
            angles.sort_by(f64::total_cmp);
            angles.dedup();
            prop_assume!(angles.len() >= 3);
            // Points on a circle in angular order: convex, counter-clockwise.
            let corners: Vec<Point2> = angles
                .iter()
                .map(|a| Point2::new(r * real::cos(*a), r * real::sin(*a)))
                .collect();
            let polygon = polygon_of(&corners, f64::INFINITY);
            let p = Point2::new(px, py);
            let n = corners.len();
            let sides: Vec<Sign> = (0..n)
                .map(|i| orient2d(corners[i], corners[(i + 1) % n], p))
                .collect();
            prop_assume!(sides.iter().all(|&s| s != Sign::Zero));
            let inside = sides.iter().all(|&s| s == Sign::Positive);
            let winding = polygon.winding_number(p);
            prop_assert_eq!(winding, i32::from(inside), "{:?} against {:?}", p, corners);
            if !inside {
                prop_assert_eq!(winding, 0);
            }
            prop_assert!(polygon.signed_area() > 0.0);
            Ok(())
        },
    );
}

#[test]
fn a_rectangle_across_the_seam_range_has_its_unwrapped_area() {
    check(
        (
            finite_f64(0.0..=TAU),
            finite_f64(0.1..=TAU),
            finite_f64(0.1..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        ),
        |(u0, w, h, v0)| {
            // u runs past 2π: the pieces are taken as written, never wrapped.
            let polygon = polygon_of(&rectangle(u0, v0, w, h), f64::INFINITY);
            prop_assert!((polygon.signed_area() - w * h).abs() <= EXACT * (w + h));
            prop_assert_eq!(
                polygon.winding_number(Point2::new(u0 + w / 2.0, v0 + h / 2.0)),
                1
            );
            prop_assert_eq!(
                polygon.winding_number(Point2::new(u0 + w / 2.0 - TAU, v0 + h / 2.0)),
                0,
                "a period away is outside: the toolkit does not wrap"
            );
            // Sides built corner to corner meet the next corner to rounding.
            prop_assert!(polygon.gaps().iter().all(|&g| g <= EXACT));
            Ok(())
        },
    );
}

#[test]
fn disjoint_loops_report_no_intersection_and_crossing_ones_report_the_pair() {
    check(
        (
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(1.0..=DEFAULT_SCALE),
            finite_f64(1.0..=DEFAULT_SCALE),
            finite_f64(0.1..=0.9),
            finite_f64(1.1..=3.0),
        ),
        |(u0, v0, w, h, inside_fraction, apart)| {
            let a = polygon_of(&rectangle(u0, v0, w, h), f64::INFINITY);
            // Shifted by a fraction of its width: the two right sides cross
            // the other's top and bottom.
            let shifted = rectangle(u0 + inside_fraction * w, v0 + inside_fraction * h, w, h);
            let b = polygon_of(&shifted, f64::INFINITY);
            // Its right side (1) and top (2) are crossed, whatever rounding
            // does to the closing segment's index in `b`.
            let hits = a.intersections(&b);
            let crossed: std::collections::BTreeSet<usize> = hits.iter().map(|h| h.0).collect();
            prop_assert_eq!(crossed, [1, 2].into_iter().collect(), "{:?}", hits);
            // Moved clear of it: nothing.
            let far = rectangle(u0 + apart * w, v0, w, h);
            let c = polygon_of(&far, f64::INFINITY);
            prop_assert!(a.intersections(&c).is_empty());
            prop_assert!(a.self_intersections().is_empty());
            // A hole strictly inside: no intersection, winding one for its corners.
            let hole = rectangle(
                u0 + 0.25 * w,
                v0 + 0.25 * h,
                0.5 * w * inside_fraction,
                0.5 * h * inside_fraction,
            );
            let d = polygon_of(&hole, f64::INFINITY);
            prop_assert!(a.intersections(&d).is_empty());
            prop_assert!(hole.iter().all(|&p| a.winding_number(p) == 1));
            Ok(())
        },
    );
}

#[test]
fn the_region_integral_of_one_is_the_area_of_a_rectangle_and_of_the_cylinder_wall() {
    check(
        (
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.1..=DEFAULT_SCALE),
            finite_f64(0.1..=DEFAULT_SCALE),
            radius(0.1..=DEFAULT_SCALE),
        ),
        |(u0, v0, w, h, r)| {
            let corners = rectangle(u0, v0, w, h);
            let sides: Vec<(Curve2, Interval)> = (0..4)
                .map(|i| line_between(corners[i], corners[(i + 1) % 4]))
                .collect();
            let pieces: Vec<Piece<'_>> = sides.iter().map(|(c, r)| Piece::along(c, *r)).collect();
            let area = region_integral(&pieces, &integrate::Grid::NONE, |_, _| 1.0);
            prop_assert!(
                (area - w * h).abs() <= EXACT * (w + h).max(1.0),
                "{area} vs {}",
                w * h
            );
            // The cylinder wall's domain [0, 2π] × [0, h] with the surface's
            // own area element: 2π r h.
            let wall = Surface::Cylinder {
                frame: arris_math::Frame::world(),
                radius: r,
            };
            let corners = rectangle(0.0, 0.0, TAU, h);
            let sides: Vec<(Curve2, Interval)> = (0..4)
                .map(|i| line_between(corners[i], corners[(i + 1) % 4]))
                .collect();
            let pieces: Vec<Piece<'_>> = sides.iter().map(|(c, r)| Piece::along(c, *r)).collect();
            let area = region_integral(&pieces, &integrate::surface_grid(&wall), |u, v| {
                let e = wall.eval(u, v);
                e.du.cross(&e.dv).norm()
            });
            let expected = TAU * r * h;
            prop_assert!(
                (area - expected).abs() <= 1e-12 * expected.max(1.0) * DEFAULT_SCALE,
                "{area} vs {expected}"
            );
            Ok(())
        },
    );
}

/// Gauss's volume: `∬ P · N / 3 dA` summed over the faces with their use
/// orientation, each face's region walked as its loop is stored.
fn gauss_volume(m: &Model, body: arris_topo::Body) -> f64 {
    let mut volume = 0.0;
    for face in m.faces(body).unwrap() {
        let entity = m.face(face.id).unwrap();
        let surface = m.surface(entity.surface()).unwrap();
        let sign = face.orientation.sign();
        for l in entity.loops() {
            let pcurves: Vec<(&Curve2, Interval, bool)> = l
                .coedges()
                .iter()
                .map(|c| {
                    let edge = m.edge(c.edge()).unwrap();
                    (
                        m.curve2(c.pcurve()).unwrap(),
                        edge.range(),
                        c.orientation() == Orientation::Reversed,
                    )
                })
                .collect();
            let pieces: Vec<Piece<'_>> = pcurves
                .iter()
                .map(|&(curve, range, reversed)| Piece {
                    curve,
                    range,
                    reversed,
                })
                .collect();
            volume += sign
                * region_integral(&pieces, &integrate::surface_grid(surface), |u, v| {
                    let e = surface.eval(u, v);
                    (e.point - Point3::origin()).dot(&e.du.cross(&e.dv)) / 3.0
                });
        }
    }
    volume
}

#[test]
fn the_gauss_volume_integrand_over_the_sample_cylinder_gives_its_volume() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let volume = gauss_volume(&m, body);
    let expected = PI * 16.0 * 12.0;
    assert!(
        (volume - expected).abs() <= 1e-9 * expected,
        "{volume} vs {expected}"
    );
    let body = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let volume = gauss_volume(&m, body);
    assert!((volume - 12000.0).abs() <= 1e-9 * 12000.0, "{volume}");
    let body =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let volume = gauss_volume(&m, body);
    assert!((volume - 12000.0).abs() <= 1e-9 * 12000.0, "{volume}");
}

// --- point_side and interior_point ----------------------------------------

/// A star-shaped ring around `centre`, counter-clockwise when `ccw`.
fn star(centre: Point2, angles: &[f64], radii: &[f64], ccw: bool) -> Vec<Point2> {
    let mut ring: Vec<Point2> = angles
        .iter()
        .zip(radii)
        .map(|(a, r)| Point2::new(centre.x + r * real::cos(*a), centre.y + r * real::sin(*a)))
        .collect();
    if !ccw {
        ring.reverse();
    }
    ring
}

/// `n` angles around the turn, one per equal sector, each jittered inside
/// the middle of its sector, so a star through them is star-shaped.
fn angles(n: std::ops::RangeInclusive<usize>) -> impl Strategy<Value = Vec<f64>> {
    n.prop_flat_map(|n| proptest::collection::vec(finite_f64(0.1..=0.9), n))
        .prop_map(|jitter| {
            let n = jitter.len();
            jitter
                .iter()
                .enumerate()
                .map(|(i, j)| TAU * (i as f64 + j) / n as f64)
                .collect()
        })
}

/// A star with up to four holes inside the disc it contains: the region
/// `interior_point` has to find a point in.
fn star_with_holes() -> impl Strategy<Value = Vec<Polygon2>> {
    (
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        angles(8..=24),
        proptest::collection::vec(radius(0.8..=1.0), 24),
        radius(1.0..=DEFAULT_SCALE),
        proptest::collection::vec(
            (
                angles(3..=10),
                proptest::collection::vec(radius(0.3..=1.0), 10),
            ),
            0..=4,
        ),
    )
        .prop_map(|(cx, cy, outer_angles, outer_radii, r, holes)| {
            let centre = Point2::new(cx, cy);
            let mut rings = vec![star(
                centre,
                &outer_angles,
                &outer_radii.iter().map(|k| k * r).collect::<Vec<_>>(),
                true,
            )];
            for (j, (hole_angles, hole_radii)) in holes.iter().enumerate() {
                let d = 0.25 * r;
                let (sx, sy) = match j {
                    0 => (1.0, 1.0),
                    1 => (-1.0, 1.0),
                    2 => (-1.0, -1.0),
                    _ => (1.0, -1.0),
                };
                let hole_centre = Point2::new(centre.x + sx * d, centre.y + sy * d);
                let scale = 0.15 * r;
                rings.push(star(
                    hole_centre,
                    hole_angles,
                    &hole_radii.iter().map(|k| k * scale).collect::<Vec<_>>(),
                    false,
                ));
            }
            rings
                .iter()
                .map(|ring| Polygon2::from_points(ring.iter().copied()))
                .collect()
        })
}

#[test]
fn an_interior_point_is_inside_and_clear_of_every_segment() {
    check(
        (star_with_holes(), radius(1e-6..=1e-3)),
        |(polygons, clearance)| {
            let Some(p) = interior_point(&polygons, clearance) else {
                // A region the mid-height line misses has no point by this
                // construction, and says so rather than guessing.
                return Ok(());
            };
            let winding: i32 = polygons.iter().map(|q| q.winding_number(p)).sum();
            prop_assert_ne!(winding, 0, "{:?} is not inside", p);
            prop_assert_eq!(point_side(&polygons, p, clearance), Side::Inside);
            prop_assert_eq!(
                interior_point(&polygons, clearance),
                Some(p),
                "two runs differ"
            );
            Ok(())
        },
    );
}

#[test]
fn every_face_of_every_sample_body_has_an_interior_point() {
    let mut m = Model::default();
    let bodies = [
        sample::unit_box(&mut m).unwrap(),
        sample::cylinder(&mut m, 4.0, 12.0).unwrap(),
        sample::frame(
            &mut m,
            Point3::origin(),
            Point3::new(40.0, 30.0, 10.0),
            Point2::new(10.0, 10.0),
            Point2::new(30.0, 20.0),
        )
        .unwrap(),
        sample::sphere(&mut m, Point3::origin(), 3.0).unwrap(),
        sample::torus(&mut m, Point3::origin(), 5.0, 2.0).unwrap(),
    ];
    for body in bodies {
        for face in m.faces(body).unwrap() {
            let entity = m.face(face.id).unwrap();
            let polygons: Vec<Polygon2> = entity
                .loops()
                .iter()
                .map(|l| discretise(&m.loop_pieces(l).unwrap(), 1e-4))
                .collect();
            let deviation = polygons
                .iter()
                .map(Polygon2::chord_deviation)
                .fold(0.0, f64::max);
            let p = interior_point(&polygons, deviation)
                .unwrap_or_else(|| panic!("{face}: no interior point"));
            assert_eq!(
                point_side(&polygons, p, deviation),
                Side::Inside,
                "{face}: {p:?}"
            );
        }
    }
}

#[test]
fn a_point_on_a_loop_is_on_the_boundary_and_a_hole_is_outside() {
    let q = |x: f64, y: f64| Point2::new(x, y);
    let outer = Polygon2::from_points([q(0.0, 0.0), q(4.0, 0.0), q(4.0, 4.0), q(0.0, 4.0)]);
    let hole = Polygon2::from_points([q(1.0, 1.0), q(1.0, 3.0), q(3.0, 3.0), q(3.0, 1.0)]);
    let region = [outer, hole];
    assert_eq!(point_side(&region, q(0.5, 2.0), 1e-9), Side::Inside);
    assert_eq!(point_side(&region, q(2.0, 2.0), 1e-9), Side::Outside);
    assert_eq!(point_side(&region, q(2.0, 0.0), 1e-9), Side::Boundary);
    assert_eq!(point_side(&region, q(2.0, 1.0), 1e-9), Side::Boundary);
    assert_eq!(point_side(&region, q(2.0, 0.5), 1e-9), Side::Inside);
    // The band is a distance, not a rank: a point a hair off a segment is
    // on the boundary at a loose tolerance and inside at a tight one.
    assert_eq!(point_side(&region, q(2.0, 0.001), 0.01), Side::Boundary);
    assert_eq!(point_side(&region, q(2.0, 0.001), 1e-9), Side::Inside);
    assert_eq!(point_side(&[], q(0.0, 0.0), 1e-9), Side::Outside);
    // The mid-height of this region runs through the hole: the two spans
    // either side of it are equal, and the leftmost wins the tie.
    assert_eq!(interior_point(&region, 0.0), Some(q(0.5, 2.0)));
    assert_eq!(interior_point(&[], 0.0), None);
}

/// `SideIndex` answers `point_side`'s question the same way for every
/// point: across and around a star with holes, and a fraction of the
/// tolerance either side of each of its vertices and segment midpoints,
/// at tolerances from far below a segment's length to far above it
/// (docs/ARCHITECTURE.md §The checker).
#[test]
fn the_side_index_answers_as_the_walk() {
    check(
        (
            star_with_holes(),
            proptest::collection::vec((finite_f64(-1.5..=1.5), finite_f64(-1.5..=1.5)), 32),
            finite_f64(-12.0..=0.0),
            finite_f64(-2.0..=2.0),
        ),
        |(polygons, spots, log_tolerance, offset)| {
            let index = SideIndex::new(&polygons);
            let corners: Vec<Point2> = polygons
                .iter()
                .flat_map(|p| p.points().iter().copied())
                .collect();
            let lo = corners
                .iter()
                .fold(Point2::new(f64::INFINITY, f64::INFINITY), |m, p| {
                    Point2::new(m.x.min(p.x), m.y.min(p.y))
                });
            let hi = corners
                .iter()
                .fold(Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY), |m, p| {
                    Point2::new(m.x.max(p.x), m.y.max(p.y))
                });
            let centre = lo + 0.5 * (hi - lo);
            let half = 0.5 * (hi - lo);
            let tolerance = real::powf(10f64, log_tolerance) * (hi - lo).norm();
            let mut points: Vec<Point2> = spots
                .iter()
                .map(|&(s, t)| centre + Vec2::new(s * half.x, t * half.y))
                .collect();
            for polygon in &polygons {
                for (a, b) in polygon.segments() {
                    let along = (b - a).normalize();
                    let across = Vec2::new(-along.y, along.x);
                    for q in [a, a + 0.5 * (b - a)] {
                        points.push(q);
                        points.push(q + offset * tolerance * across);
                    }
                }
            }
            for p in points {
                prop_assert_eq!(
                    index.side(p, tolerance),
                    point_side(&polygons, p, tolerance),
                    "{:?} at tolerance {}",
                    p,
                    tolerance
                );
            }
            Ok(())
        },
    );
}
