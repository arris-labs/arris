//! Loft sections made compatible (ADR-0065): a loop as one NURBS that
//! reproduces the profile exactly, sections cut at arc-length fractions
//! from their start points, one knot vector across sections, and each
//! refusal by name.

use arris_geom::{
    LoftError, LoopNurbs, MatchedLoop, Profile, ProfileLoop, ProfileSegment, match_sections,
};
use arris_math::{Frame, Point2, Point3, Precision, Tolerance, Vec3, real};

const EXACT: f64 = 1e-12;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

fn p(u: f64, v: f64) -> Point2 {
    Point2::new(u, v)
}

fn at(z: f64) -> Frame {
    Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).unwrap()
}

fn polygon(z: f64, points: &[(f64, f64)]) -> Profile {
    let n = points.len();
    Profile {
        plane: at(z),
        outer: ProfileLoop::Path {
            start: p(points[0].0, points[0].1),
            segments: (1..=n)
                .map(|i| ProfileSegment::LineTo(p(points[i % n].0, points[i % n].1)))
                .collect(),
        },
        holes: vec![],
    }
}

fn circle(z: f64, r: f64, start_turned: bool) -> Profile {
    // A circle edge starts on the frame's x axis: turn the frame to move it.
    let plane = if start_turned {
        Frame::new(
            Point3::new(0.0, 0.0, z),
            Vec3::z(),
            Vec3::new(1.0, 1.0, 0.0),
        )
        .unwrap()
    } else {
        at(z)
    };
    Profile {
        plane,
        outer: ProfileLoop::Circle {
            center: p(0.0, 0.0),
            radius: r,
        },
        holes: vec![],
    }
}

/// A square of half-side `h`, counter-clockwise from the corner (h, h).
fn square(z: f64, h: f64) -> Profile {
    polygon(z, &[(h, h), (-h, h), (-h, -h), (h, -h)])
}

/// The points of a matched loop at its segments' ends.
fn vertices(l: &MatchedLoop) -> Vec<Point3> {
    l.nurbs
        .pieces
        .iter()
        .map(|piece| l.nurbs.curve.control_points()[piece.first])
        .collect()
}

#[test]
fn loft_loop_nurbs_is_the_profile() {
    let sq = square(0.0, 2.0);
    let edges = sq.edges(tol()).unwrap();
    let nurbs = LoopNurbs::new(&edges[0]).unwrap();
    assert_eq!(nurbs.segments.len(), 4);
    // Every segment is its edge, to rounding.
    for (edge, seg) in edges[0].iter().zip(&nurbs.segments) {
        for i in 0..=8 {
            let t = edge.range.lo() + (edge.range.hi() - edge.range.lo()) * i as f64 / 8.0;
            let s = seg.domain().lo() + (seg.domain().hi() - seg.domain().lo()) * i as f64 / 8.0;
            assert!((edge.curve.point(t) - seg.eval(s).point).norm() < EXACT);
        }
    }
    let c = &nurbs.curve;
    assert!((c.eval(c.domain().lo()).point - c.eval(c.domain().hi()).point).norm() < EXACT);
}

#[test]
fn loft_circle_is_cut_where_the_square_has_corners() {
    // The circle starts on the square's start corner's direction.
    let sections = [circle(0.0, 1.0, true), square(1.0, 3.0)];
    let m = match_sections(&sections, false, tol()).unwrap();
    for s in &m {
        assert_eq!(s.loops.len(), 1);
        assert_eq!(s.loops[0].nurbs.segments.len(), 4);
    }
    // The circle's vertices are a quarter turn apart, on the radius.
    let v = vertices(&m[0].loops[0]);
    let r = 1.0;
    for (i, q) in v.iter().enumerate() {
        let a = core::f64::consts::FRAC_PI_4 + core::f64::consts::FRAC_PI_2 * i as f64;
        assert!(
            (q - Point3::new(r * real::cos(a), r * real::sin(a), 0.0)).norm() < EXACT,
            "{i}"
        );
    }
    // The square's are its corners.
    let w = vertices(&m[1].loops[0]);
    assert!((w[0] - Point3::new(3.0, 3.0, 1.0)).norm() < EXACT);
    // Both are one knot vector, and the circle's curve stays on the circle.
    assert_eq!(
        m[0].loops[0].nurbs.curve.knots(),
        m[1].loops[0].nurbs.curve.knots()
    );
    let c = &m[0].loops[0].nurbs.curve;
    for i in 0..=64 {
        let t = c.domain().lo() + (c.domain().hi() - c.domain().lo()) * i as f64 / 64.0;
        let q = c.eval(t).point;
        assert!((q.coords.xy().norm() - r).abs() < EXACT && q.z.abs() < EXACT);
    }
}

#[test]
fn loft_split_points_are_arc_length_fractions_of_the_start() {
    // A 12 x 4 rectangle from (6, -2): corners at 0, 1/8, 1/2, 5/8 of its
    // perimeter. The circle is cut at those fractions of its own length,
    // not at the corners' directions (Open CASCADE's rule, atan(1/3)).
    let rect = polygon(1.0, &[(6.0, -2.0), (6.0, 2.0), (-6.0, 2.0), (-6.0, -2.0)]);
    let m = match_sections(&[circle(0.0, 1.0, false), rect], false, tol()).unwrap();
    let v = vertices(&m[0].loops[0]);
    assert_eq!(v.len(), 4);
    for (q, f) in v.iter().zip([0.0, 0.125, 0.5, 0.625]) {
        let a = core::f64::consts::TAU * f;
        assert!(
            (q - Point3::new(real::cos(a), real::sin(a), 0.0)).norm() < EXACT,
            "{f}"
        );
    }
}

#[test]
fn loft_unequal_counts_cut_every_section_at_every_vertex() {
    // A triangle and a square from the same corner: vertices at thirds and
    // quarters, so both are cut to the six fractions of their union.
    let tri = polygon(0.0, &[(2.0, 2.0), (-2.0, 2.0), (0.0, -2.0)]);
    let m = match_sections(&[tri, square(1.0, 2.0)], false, tol()).unwrap();
    for s in &m {
        assert_eq!(s.loops[0].nurbs.segments.len(), 6);
        // Cut pieces name the consumer's segment they came from.
        assert_eq!(s.loops[0].sources.len(), 6);
    }
    // Each triangle segment is cut once or twice, in order.
    let src = &m[0].loops[0].sources;
    assert!(src.windows(2).all(|w| w[0] <= w[1]) && src[0] == 0 && src[5] == 2);
    assert_eq!(
        m[0].loops[0].nurbs.curve.knots(),
        m[1].loops[0].nurbs.curve.knots()
    );
}

#[test]
fn loft_equal_counts_correspond_by_index() {
    // A square and a rounded one with the same eight... a square and a
    // rhombus: four segments each, whatever the fractions.
    let rhombus = polygon(1.0, &[(5.0, 0.0), (0.0, 1.0), (-5.0, 0.0), (0.0, -1.0)]);
    let m = match_sections(&[square(0.0, 2.0), rhombus], false, tol()).unwrap();
    assert_eq!(m[0].loops[0].nurbs.segments.len(), 4);
    let v = vertices(&m[1].loops[0]);
    assert!((v[0] - Point3::new(5.0, 0.0, 1.0)).norm() < EXACT);
    assert_eq!(m[0].loops[0].sources, vec![0, 1, 2, 3]);
}

#[test]
fn loft_holes_match_hole_to_hole() {
    let holed = |z: f64, h: f64, r: f64| Profile {
        holes: vec![ProfileLoop::Circle {
            center: p(0.0, 0.0),
            radius: r,
        }],
        ..square(z, h)
    };
    let m = match_sections(&[holed(0.0, 5.0, 2.0), holed(1.0, 3.0, 1.0)], false, tol()).unwrap();
    for s in &m {
        assert_eq!(s.loops.len(), 2);
        assert_eq!(s.loops[1].nurbs.segments.len(), 1);
    }
}

#[test]
fn loft_refusals_name_the_section() {
    let a = square(0.0, 2.0);
    assert_eq!(
        match_sections(std::slice::from_ref(&a), false, tol()).unwrap_err(),
        LoftError::TooFewSections {
            given: 1,
            needed: 2
        }
    );
    assert_eq!(
        match_sections(&[a.clone(), square(1.0, 1.0)], true, tol()).unwrap_err(),
        LoftError::TooFewSections {
            given: 2,
            needed: 3
        }
    );
    let holed = Profile {
        holes: vec![ProfileLoop::Circle {
            center: p(0.0, 0.0),
            radius: 0.5,
        }],
        ..square(1.0, 2.0)
    };
    assert_eq!(
        match_sections(&[a.clone(), a.clone(), holed], false, tol()).unwrap_err(),
        LoftError::LoopCount {
            section: 2,
            expected: 1,
            found: 2
        }
    );
    assert_eq!(
        match_sections(&[a.clone(), a.clone()], false, tol()).unwrap_err(),
        LoftError::Coincident {
            section: 1,
            previous: 0
        }
    );
    let one = Profile {
        plane: at(1.0),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![ProfileSegment::LineTo(p(0.0, 0.0))],
        },
        holes: vec![],
    };
    assert!(matches!(
        match_sections(&[a, one], false, tol()).unwrap_err(),
        LoftError::Profile { section: 1, .. }
    ));
}
