//! The constrained Delaunay triangulation (ADR-0003): on random
//! star-shaped outers with disjoint star-shaped holes and random
//! interior points, every polygon segment is a triangle edge, every
//! triangle is counter-clockwise, the areas add up to the region's,
//! every other edge is shared by two triangles and is locally Delaunay,
//! and no triangle lies outside the region; plus hand-picked shapes.

use core::f64::consts::TAU;
use std::collections::BTreeMap;

use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, radius};
use arris_debug::testing::fail;
use arris_geom::region2::Polygon2;
use arris_math::Point2;
use arris_math::predicates::{Sign, incircle, orient2d};
use arris_mesh::cdt::{CdtError, SegmentRef, Triangulation2, VertexRef, triangulate};
use proptest::prelude::*;

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

/// The shoelace area of a ring.
fn area_of(ring: &[Point2]) -> f64 {
    let n = ring.len();
    (0..n)
        .map(|i| {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            a.x * b.y - b.x * a.y
        })
        .sum::<f64>()
        / 2.0
}

/// A star-shaped ring around `centre`: `angles` sorted and distinct,
/// each with its own radius, counter-clockwise when `ccw`.
fn star(centre: Point2, angles: &[f64], radii: &[f64], ccw: bool) -> Vec<Point2> {
    let mut ring: Vec<Point2> = angles
        .iter()
        .zip(radii)
        .map(|(a, r)| p(centre.x + r * a.cos(), centre.y + r * a.sin()))
        .collect();
    if !ccw {
        ring.reverse();
    }
    ring
}

/// `n` angles around the turn, one per equal sector, each jittered
/// within the middle of its sector so consecutive gaps stay between
/// `0.2` and `1.8` sectors: a star through them has an inscribed disc of
/// at least `cos(0.9π / n)` of its smallest radius.
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

/// What the property asserts of every triangulation: the invariants of
/// `arris_mesh::cdt`'s crate doc, against the input rings and the area
/// they enclose.
fn assert_valid(
    t: &Triangulation2,
    rings: &[Vec<Point2>],
    expected_area: f64,
    scale: f64,
) -> Result<(), TestCaseError> {
    let pts = t.points();
    let mut area = 0.0;
    let mut edge_count: BTreeMap<(usize, usize), usize> = BTreeMap::new();
    let mut apex: BTreeMap<(usize, usize), usize> = BTreeMap::new();
    for &[a, b, c] in t.triangles() {
        prop_assert_eq!(
            orient2d(pts[a], pts[b], pts[c]),
            Sign::Positive,
            "triangle {:?} is not counter-clockwise",
            [a, b, c]
        );
        area += area_of(&[pts[a], pts[b], pts[c]]);
        for (x, y, z) in [(a, b, c), (b, c, a), (c, a, b)] {
            *edge_count.entry((x.min(y), x.max(y))).or_default() += 1;
            apex.insert((x, y), z);
        }
    }
    prop_assert!(
        (area - expected_area).abs() <= 1e-12 * scale * scale,
        "area {area} vs {expected_area}"
    );
    // Every polygon segment is an edge, used by exactly one triangle.
    let mut segments = std::collections::BTreeSet::new();
    for (k, ring) in rings.iter().enumerate() {
        let n = ring.len();
        for i in 0..n {
            let (a, b) = (
                t.polygon_vertex(k, i).unwrap(),
                t.polygon_vertex(k, (i + 1) % n).unwrap(),
            );
            prop_assert_eq!(
                edge_count.get(&(a.min(b), a.max(b))).copied(),
                Some(1),
                "segment {} of polygon {} is not a boundary edge",
                i,
                k
            );
            segments.insert((a.min(b), a.max(b)));
        }
    }
    // Every other edge is interior — two triangles — and locally Delaunay.
    for (&(a, b), &n) in &edge_count {
        if segments.contains(&(a, b)) {
            continue;
        }
        prop_assert_eq!(n, 2, "edge ({}, {}) is used {} times", a, b, n);
        let (c, d) = (apex[&(a, b)], apex[&(b, a)]);
        prop_assert_ne!(
            incircle(pts[a], pts[b], pts[c], pts[d]),
            Sign::Positive,
            "edge ({}, {}) is not locally Delaunay",
            a,
            b
        );
    }
    // No triangle's centroid is outside the region.
    let polygons: Vec<Polygon2> = rings
        .iter()
        .map(|r| Polygon2::from_points(r.iter().copied()))
        .collect();
    for &[a, b, c] in t.triangles() {
        let centroid = p(
            (pts[a].x + pts[b].x + pts[c].x) / 3.0,
            (pts[a].y + pts[b].y + pts[c].y) / 3.0,
        );
        let winding: i32 = polygons.iter().map(|q| q.winding_number(centroid)).sum();
        prop_assert_ne!(winding, 0, "a triangle lies outside the region");
    }
    Ok(())
}

#[test]
fn random_stars_with_holes_and_interior_points_triangulate_validly() {
    check(
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
            proptest::collection::vec((finite_f64(-1.0..=1.0), finite_f64(-1.0..=1.0)), 0..=12),
        ),
        |(cx, cy, outer_angles, outer_radii, r, holes, inner)| {
            let centre = p(cx, cy);
            // The outer star, radii in [0.8 r, r] over at least eight
            // sectors, holds the disc of radius 0.6 r.
            let outer = star(
                centre,
                &outer_angles,
                &outer_radii.iter().map(|k| k * r).collect::<Vec<_>>(),
                true,
            );
            // Holes at the four diagonal positions 0.35 r out, each within
            // 0.15 r of its centre: inside the disc and apart from each
            // other.
            let mut rings = vec![outer];
            for (j, (hole_angles, hole_radii)) in holes.iter().enumerate() {
                let d = 0.25 * r;
                let (sx, sy) = match j {
                    0 => (1.0, 1.0),
                    1 => (-1.0, 1.0),
                    2 => (-1.0, -1.0),
                    _ => (1.0, -1.0),
                };
                let hole_centre = p(centre.x + sx * d, centre.y + sy * d);
                let scale = 0.15 * r;
                rings.push(star(
                    hole_centre,
                    hole_angles,
                    &hole_radii.iter().map(|k| k * scale).collect::<Vec<_>>(),
                    false,
                ));
            }
            let polygons: Vec<Polygon2> = rings
                .iter()
                .map(|ring| Polygon2::from_points(ring.iter().copied()))
                .collect();
            // Interior points anywhere in the outer's box, kept off the
            // segments; those outside the region are discarded by the
            // triangulation and must not change the area.
            let interior: Vec<Point2> = inner
                .iter()
                .map(|&(x, y)| p(centre.x + x * r, centre.y + y * r))
                .filter(|q| !polygons.iter().any(|poly| poly.contains(*q)))
                .collect();
            let t = triangulate(&polygons, &interior).map_err(fail)?;
            let expected: f64 = rings.iter().map(|ring| area_of(ring)).sum();
            assert_valid(&t, &rings, expected, DEFAULT_SCALE)?;
            // Determinism: the same input gives the same lists.
            let again = triangulate(&polygons, &interior).map_err(fail)?;
            prop_assert_eq!(&t, &again);
            Ok(())
        },
    );
}

#[test]
fn a_regular_polygon_at_every_count_has_n_minus_two_triangles() {
    for n in 3..=64usize {
        let ring: Vec<Point2> = (0..n)
            .map(|i| {
                let a = TAU * i as f64 / n as f64;
                p(3.0 * a.cos(), 3.0 * a.sin())
            })
            .collect();
        let t = triangulate(&[Polygon2::from_points(ring.iter().copied())], &[]).unwrap();
        assert_eq!(t.triangles().len(), n - 2, "n = {n}");
        assert_valid(&t, std::slice::from_ref(&ring), area_of(&ring), 3.0).unwrap();
    }
}

#[test]
fn a_square_with_a_square_hole_is_eight_triangles() {
    let outer = vec![p(0.0, 0.0), p(4.0, 0.0), p(4.0, 4.0), p(0.0, 4.0)];
    let hole = vec![p(1.0, 1.0), p(1.0, 3.0), p(3.0, 3.0), p(3.0, 1.0)];
    let polygons = [
        Polygon2::from_points(outer.iter().copied()),
        Polygon2::from_points(hole.iter().copied()),
    ];
    let t = triangulate(&polygons, &[]).unwrap();
    assert_eq!(t.triangles().len(), 8);
    assert_valid(&t, &[outer, hole], 12.0, 4.0).unwrap();
    // With an interior point in the material, two more triangles.
    let t = triangulate(&polygons, &[p(0.5, 2.0)]).unwrap();
    assert_eq!(t.triangles().len(), 10);
    assert_eq!(t.interior_vertex(0), Some(8));
    assert_eq!(t.vertex_ref(8), Some(VertexRef::Interior(0)));
    // An interior point inside the hole adds nothing kept.
    let t = triangulate(&polygons, &[p(2.0, 2.0)]).unwrap();
    assert_eq!(t.triangles().len(), 8);
}

#[test]
fn collinear_runs_on_one_segment_are_split_and_kept() {
    // Ten points along the bottom, five along the top, all collinear
    // with their neighbours.
    let mut ring = Vec::new();
    for i in 0..10 {
        ring.push(p(i as f64, 0.0));
    }
    ring.push(p(10.0, 0.0));
    ring.push(p(10.0, 5.0));
    for i in (0..=4).rev() {
        ring.push(p(2.0 * i as f64, 5.0));
    }
    let t = triangulate(&[Polygon2::from_points(ring.iter().copied())], &[]).unwrap();
    assert_eq!(t.triangles().len(), ring.len() - 2);
    assert_valid(&t, &[ring], 50.0, 10.0).unwrap();
}

#[test]
fn points_one_ulp_apart_make_a_sliver_that_is_still_valid() {
    let nudge = f64::from_bits(0.0f64.to_bits() + 1);
    // A unit square with a fifth vertex one ulp inside its left side.
    let ring = vec![
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(1.0, 1.0),
        p(0.0, 1.0),
        p(nudge, 0.5),
    ];
    let t = triangulate(&[Polygon2::from_points(ring.iter().copied())], &[]).unwrap();
    assert_eq!(t.triangles().len(), 3);
    assert_valid(&t, &[ring], 1.0, 1.0).unwrap();
    // Two consecutive points one ulp apart in y, a sliver on the top.
    let ring = vec![
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(1.0, 1.0),
        p(1.0, 1.0 + f64::EPSILON),
        p(0.0, 1.0),
    ];
    let t = triangulate(&[Polygon2::from_points(ring.iter().copied())], &[]).unwrap();
    assert_eq!(t.triangles().len(), 3);
    assert_valid(&t, std::slice::from_ref(&ring), area_of(&ring), 1.0).unwrap();
}

#[test]
fn a_crossing_pair_is_the_named_error() {
    let outer = Polygon2::from_points([p(0.0, 0.0), p(4.0, 0.0), p(4.0, 4.0), p(0.0, 4.0)]);
    // A "hole" whose right side crosses the outer's right side.
    let poking = Polygon2::from_points([p(2.0, 1.0), p(2.0, 3.0), p(6.0, 3.0), p(6.0, 1.0)]);
    let err = triangulate(&[outer, poking], &[]).unwrap_err();
    assert_eq!(
        err,
        CdtError::Crossing {
            a: SegmentRef {
                polygon: 1,
                segment: 1
            },
            b: SegmentRef {
                polygon: 0,
                segment: 1
            }
        }
    );
    assert_eq!(
        err.to_string(),
        "polygon 1 segment 1 crosses polygon 0 segment 1"
    );
}
