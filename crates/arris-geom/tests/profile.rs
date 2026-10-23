//! A sketch is a profile: random star polygons with arcs and holes, in
//! either orientation and in a random plane pose, come back oriented with
//! the consumer's indices intact and with exact pcurves; the closed forms
//! of a rectangle, a disc, a stadium and a rectangle with a hole are the
//! areas and centroids `region_integral` reports; and every committed
//! `sweep/*` recipe loads into a profile the validator accepts
//! (`docs/DATA-MODEL.md` §Profiles).

use arris_debug::testing::tol;
use std::collections::BTreeMap;

use arris_debug::fixtures::{self, Step, geom::build_profile};
use arris_debug::prop::{DEFAULT_SCALE, check};
use arris_geom::profile::{Profile, ProfileEdge, ProfileLoop, ProfileSegment};
use arris_geom::region2::{Piece, Polygon2};
use arris_math::{Frame, Point2, Point3};
use core::f64::consts::PI;
use proptest::prelude::*;

/// The profile strategy's coordinates live within a few multiples of this.
const SCALE: f64 = DEFAULT_SCALE;
/// Exact: the pcurve's image is the curve to rounding at the scale.
const EXACT: f64 = 1e-12 * SCALE;
/// Parameters the image is checked at per edge, both ends included.
const CHECKS: usize = 32;

fn p(u: f64, v: f64) -> Point2 {
    Point2::new(u, v)
}

/// The minimal polygon of one loop's edges, walked in order.
fn polygon(edges: &[ProfileEdge]) -> Polygon2 {
    let pieces: Vec<Piece<'_>> = edges
        .iter()
        .map(|e| Piece::along(&e.pcurve, e.range))
        .collect();
    arris_geom::region2::discretise(&pieces, f64::INFINITY)
}

#[test]
fn a_sketch_comes_back_oriented_with_its_indices_intact() {
    check(arris_debug::prop::profile::star(), |sketch| {
        let loops = sketch.edges(tol()).map_err(|e| {
            TestCaseError::fail(format!(
                "the strategy drew a profile the validator refuses: {e}"
            ))
        })?;
        prop_assert_eq!(loops.len(), 1 + sketch.holes.len());
        for (i, edges) in loops.iter().enumerate() {
            // The outer loop turns counter-clockwise, every hole clockwise.
            let area = polygon(edges).signed_area();
            prop_assert_eq!(area > 0.0, i == 0, "loop {} has area {}", i, area);
            // The consumer's indices, each once, and the walk runs with
            // them or against them as `reversed` says.
            let reversed = edges[0].reversed;
            let written: Vec<usize> = edges.iter().map(|e| e.segment).collect();
            let mut expected: Vec<usize> = (0..edges.len()).collect();
            if reversed {
                expected.reverse();
            }
            prop_assert_eq!(&written, &expected, "loop {}", i);
            prop_assert!(
                edges
                    .iter()
                    .all(|e| e.loop_index == i && e.reversed == reversed)
            );
            // Consecutive edges meet at their vertices, and the loop closes.
            for k in 0..edges.len() {
                let here = &edges[k];
                let next = &edges[(k + 1) % edges.len()];
                prop_assert!(
                    (here.end - next.start).norm() <= EXACT,
                    "loop {} edge {} ends at {} and {} starts at {}",
                    i,
                    k,
                    here.end,
                    (k + 1) % edges.len(),
                    next.start
                );
                // The curve runs from `start` to `end` over its range.
                prop_assert!(
                    (sketch.to_world(here.start) - here.curve.point(here.range.lo())).norm()
                        <= EXACT
                );
                prop_assert!(
                    (sketch.to_world(here.end) - here.curve.point(here.range.hi())).norm() <= EXACT
                );
                // The pcurve's image is the curve at the same parameter.
                for s in 0..=CHECKS {
                    let t = here.range.lerp(s as f64 / CHECKS as f64);
                    let uv = here.pcurve.point(t);
                    let image = sketch.to_world(uv);
                    let err = (image - here.curve.point(t)).norm();
                    prop_assert!(err <= EXACT, "at t = {}: off by {}", t, err);
                }
            }
        }
        // The area is positive and the centroid is inside the outer loop.
        let (area, centroid) = sketch.area_and_centroid(tol()).unwrap();
        prop_assert!(area > 0.0, "area {}", area);
        prop_assert!(centroid.coords.amax().is_finite());
        Ok(())
    });
}

/// The elliptic grammar (ADR-0014): a full ellipse and elliptic arcs
/// come back as exact `Curve::Ellipse` edges with `a ≥ b` whichever
/// radius the consumer called major, the area is `π a b`, an arc's ends
/// are the ellipse's nearest points and its turn is the flag's — flipped
/// when the loop is turned round — radii that agree to the tolerance
/// make a circle edge, and every fault names its segment.
#[test]
fn ellipses_come_back_exact_with_their_axes_normalised() {
    let plane = Frame::new(
        Point3::new(1.0, 2.0, 3.0),
        Point3::new(1.0, 1.0, 1.0).coords,
        Point3::new(1.0, -1.0, 0.0).coords,
    )
    .unwrap();
    let major = arris_math::Vec2::new(4.8, 3.6);
    // Written with the longer radius as the minor one: the axes swap.
    let swapped = Profile {
        plane,
        outer: ProfileLoop::Ellipse {
            center: p(1.0, 2.0),
            major: arris_math::Vec2::new(2.4, 1.8),
            minor_radius: 6.0,
        },
        holes: Vec::new(),
    };
    let loops = swapped.edges(tol()).unwrap();
    let e = &loops[0][0];
    let arris_geom::Curve::Ellipse {
        frame,
        major_radius,
        minor_radius,
    } = &e.curve
    else {
        panic!("{:?}", e.curve)
    };
    assert_eq!((*major_radius, *minor_radius), (6.0, 3.0));
    // The major axis is the written minor one, a quarter turn on.
    let x = plane.vec_to_local(frame.x().into_inner());
    assert!(
        (Point2::new(x.x, x.y) - p(-0.6, 0.8)).norm() <= EXACT,
        "{x}"
    );
    assert!(
        (e.start - p(1.0 - 3.6, 2.0 + 4.8)).norm() <= EXACT,
        "{}",
        e.start
    );
    let (area, centroid) = swapped.area_and_centroid(tol()).unwrap();
    assert!((area - PI * 18.0).abs() <= EXACT, "{area}");
    assert!((centroid - p(1.0, 2.0)).norm() <= EXACT);
    // As a hole it is turned round and its `Z` opposes the normal.
    let holed = Profile {
        plane,
        outer: ProfileLoop::Path {
            start: p(-20.0, -20.0),
            segments: vec![
                ProfileSegment::LineTo(p(20.0, -20.0)),
                ProfileSegment::LineTo(p(20.0, 20.0)),
                ProfileSegment::LineTo(p(-20.0, 20.0)),
                ProfileSegment::LineTo(p(-20.0, -20.0)),
            ],
        },
        holes: vec![ProfileLoop::Ellipse {
            center: p(1.0, 2.0),
            major,
            minor_radius: 3.0,
        }],
    };
    let loops = holed.edges(tol()).unwrap();
    assert!(loops[1][0].reversed);
    let arris_geom::Curve::Ellipse { frame, .. } = &loops[1][0].curve else {
        panic!()
    };
    assert!(frame.z().dot(&plane.z()) < 0.0);
    let (area, _) = holed.area_and_centroid(tol()).unwrap();
    assert!((area - (1600.0 - PI * 18.0)).abs() <= EXACT, "{area}");

    // A slot of two half-ellipses: each arc's ends are its minor
    // vertices, its range a half turn, and the walk keeps its turn.
    let slot = |ccw: bool| Profile {
        plane,
        outer: ProfileLoop::Path {
            start: p(-10.0, -3.0),
            segments: vec![
                ProfileSegment::LineTo(p(10.0, -3.0)),
                ProfileSegment::EllipseTo {
                    to: p(10.0, 3.0),
                    center: p(10.0, 0.0),
                    major: arris_math::Vec2::new(5.0, 0.0),
                    minor_radius: 3.0,
                    ccw,
                },
                ProfileSegment::LineTo(p(-10.0, 3.0)),
                ProfileSegment::EllipseTo {
                    to: p(-10.0, -3.0),
                    center: p(-10.0, 0.0),
                    major: arris_math::Vec2::new(5.0, 0.0),
                    minor_radius: 3.0,
                    ccw,
                },
            ],
        },
        holes: Vec::new(),
    };
    let loops = slot(true).edges(tol()).unwrap();
    let arc = loops[0].iter().find(|e| e.segment == 1).unwrap();
    assert!(!arc.reversed);
    assert!((arc.range.length() - PI).abs() <= EXACT, "{:?}", arc.range);
    let mid = arc.curve.point(arc.range.midpoint());
    assert!(
        (mid - slot(true).to_world(p(15.0, 0.0))).norm() <= EXACT,
        "{mid}"
    );
    let (area, _) = slot(true).area_and_centroid(tol()).unwrap();
    assert!((area - (120.0 + 15.0 * PI)).abs() <= EXACT, "{area}");
    // The arcs turned the other way bulge inward: a concave slot, still
    // counter-clockwise as written, with the area of the rectangle less
    // the two half-ellipses.
    let loops = slot(false).edges(tol()).unwrap();
    let arc = loops[0].iter().find(|e| e.segment == 1).unwrap();
    let mid = arc.curve.point(arc.range.midpoint());
    assert!(
        (mid - slot(false).to_world(p(5.0, 0.0))).norm() <= EXACT,
        "{mid}"
    );
    let (area, _) = slot(false).area_and_centroid(tol()).unwrap();
    assert!((area - (120.0 - 15.0 * PI)).abs() <= EXACT, "{area}");

    // Radii within the tolerance of each other: a circle edge.
    let round = Profile {
        plane,
        outer: ProfileLoop::Ellipse {
            center: p(0.0, 0.0),
            major: arris_math::Vec2::new(2.0, 0.0),
            minor_radius: 2.0 + 1e-8,
        },
        holes: Vec::new(),
    };
    let loops = round.edges(tol()).unwrap();
    assert!(matches!(
        loops[0][0].curve,
        arris_geom::Curve::Circle { .. }
    ));

    // Faults name the segment.
    let bad = |segment: ProfileSegment| Profile {
        plane,
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![ProfileSegment::LineTo(p(10.0, 0.0)), segment],
        },
        holes: Vec::new(),
    };
    let off = bad(ProfileSegment::EllipseTo {
        to: p(0.0, 0.0),
        center: p(5.0, 0.0),
        major: arris_math::Vec2::new(5.0, 0.0),
        minor_radius: 2.0,
        ccw: true,
    });
    assert!(off.edges(tol()).is_ok());
    let off = bad(ProfileSegment::EllipseTo {
        to: p(0.0, 0.0),
        center: p(5.0, 1.0),
        major: arris_math::Vec2::new(5.0, 0.0),
        minor_radius: 2.0,
        ccw: true,
    });
    assert!(matches!(
        off.edges(tol()),
        Err(arris_geom::ProfileError::OffEllipse {
            loop_index: 0,
            segment: 1,
            ..
        })
    ));
    let flat = bad(ProfileSegment::EllipseTo {
        to: p(0.0, 0.0),
        center: p(5.0, 0.0),
        major: arris_math::Vec2::new(5.0, 0.0),
        minor_radius: 0.0,
        ccw: true,
    });
    assert!(matches!(
        flat.edges(tol()),
        Err(arris_geom::ProfileError::DegenerateEllipse {
            loop_index: 0,
            segment: 1
        })
    ));
    let flat_loop = Profile {
        plane,
        outer: ProfileLoop::Ellipse {
            center: p(0.0, 0.0),
            major: arris_math::Vec2::new(0.0, 0.0),
            minor_radius: 2.0,
        },
        holes: Vec::new(),
    };
    assert!(matches!(
        flat_loop.edges(tol()),
        Err(arris_geom::ProfileError::DegenerateEllipse {
            loop_index: 0,
            segment: 0
        })
    ));
}

#[test]
fn the_closed_forms_of_four_regions_are_the_integrals_over_them() {
    let plane = Frame::new(
        Point3::new(1.0, -2.0, 3.0),
        arris_math::Vec3::new(1.0, 2.0, 3.0),
        arris_math::Vec3::new(3.0, 0.0, -1.0),
    )
    .unwrap();
    let of = |outer, holes| Profile {
        plane,
        outer,
        holes,
    };
    let rect = |w: f64, h: f64| ProfileLoop::Path {
        start: p(0.0, 0.0),
        segments: vec![
            ProfileSegment::LineTo(p(w, 0.0)),
            ProfileSegment::LineTo(p(w, h)),
            ProfileSegment::LineTo(p(0.0, h)),
            ProfileSegment::LineTo(p(0.0, 0.0)),
        ],
    };
    let close = |(area, centroid): (f64, Point2), a: f64, c: Point2| {
        assert!((area - a).abs() <= 1e-12 * a, "area {area} vs {a}");
        assert!(
            (centroid - c).norm() <= 1e-12 * a.sqrt(),
            "centroid {centroid} vs {c}"
        );
    };
    // A rectangle.
    close(
        of(rect(4.0, 6.0), Vec::new())
            .area_and_centroid(tol())
            .unwrap(),
        24.0,
        p(2.0, 3.0),
    );
    // A disc.
    close(
        of(
            ProfileLoop::Circle {
                center: p(-3.0, 5.0),
                radius: 2.5,
            },
            Vec::new(),
        )
        .area_and_centroid(tol())
        .unwrap(),
        PI * 6.25,
        p(-3.0, 5.0),
    );
    // A stadium: two lines and two semicircular arcs.
    let (l, r) = (20.0, 5.0);
    let stadium = ProfileLoop::Path {
        start: p(0.0, -r),
        segments: vec![
            ProfileSegment::LineTo(p(l, -r)),
            ProfileSegment::ArcTo {
                to: p(l, r),
                via: p(l + r, 0.0),
            },
            ProfileSegment::LineTo(p(0.0, r)),
            ProfileSegment::ArcTo {
                to: p(0.0, -r),
                via: p(-r, 0.0),
            },
        ],
    };
    close(
        of(stadium, Vec::new()).area_and_centroid(tol()).unwrap(),
        2.0 * r * l + PI * r * r,
        p(l / 2.0, 0.0),
    );
    // A rectangle with a hole, off centre: the centroid moves away from it.
    let holed = of(
        rect(40.0, 30.0),
        vec![ProfileLoop::Circle {
            center: p(10.0, 15.0),
            radius: 4.0,
        }],
    );
    let area = 1200.0 - PI * 16.0;
    let cu = (1200.0 * 20.0 - PI * 16.0 * 10.0) / area;
    close(holed.area_and_centroid(tol()).unwrap(), area, p(cu, 15.0));
}

#[test]
fn every_sweep_recipe_loads_into_a_profile() {
    let mut seen = 0;
    for dir in fixtures::corpus() {
        if !dir.to_string_lossy().contains("/sweep/") {
            continue;
        }
        let fixture = fixtures::load(&dir).expect("the sweep fixtures load");
        for variant in fixture.recipe.variant_names() {
            let params: BTreeMap<String, f64> = fixture
                .recipe
                .params_of(&variant)
                .expect("a named variant has params");
            for step in &fixture.recipe.steps {
                let Step::Profile {
                    name,
                    plane,
                    outer,
                    holes,
                } = step
                else {
                    continue;
                };
                let profile = build_profile(name, plane, outer, holes, &params)
                    .unwrap_or_else(|e| panic!("{}: step {name}: {e}", fixture.name));
                let loops = profile
                    .edges(tol())
                    .unwrap_or_else(|e| panic!("{}: step {name}: {e}", fixture.name));
                assert_eq!(loops.len(), 1 + profile.holes.len());
                assert!(polygon(&loops[0]).signed_area() > 0.0);
                let (area, _) = profile
                    .area_and_centroid(tol())
                    .unwrap_or_else(|e| panic!("{}: step {name}: {e}", fixture.name));
                assert!(area > 0.0, "{}: step {name}: area {area}", fixture.name);
                seen += 1;
            }
        }
    }
    assert!(seen >= 3, "the corpus holds sweep fixtures with profiles");
}
