//! The chord bounds (`docs/ARCHITECTURE.md` §Tessellation):
//! `Curve::chord_segments` agrees with its (u, v) twin, and
//! `Surface::chord_steps` is the second fundamental form per kind — flat
//! or ruled directions unbounded, the cone read at its far bound, the
//! doubly curved kinds sharing the chord between their directions.

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::{cone, cylinder, plane, sphere, torus};
use arris_debug::prop::{check, finite_f64, radius};
use arris_geom::region2::{MAX_SEGMENTS_PER_PIECE, MIN_SEGMENTS_PER_TURN, Piece};
use arris_geom::{Curve, Curve2, NurbsSurface, Surface};
use arris_math::{Frame, Frame2, Interval, Point3, Vec3};
use proptest::prelude::*;

#[test]
fn a_line_is_one_segment_and_a_circle_matches_its_pcurve_twin() {
    let line = Curve::Line {
        origin: Point3::origin(),
        direction: Vec3::x_axis(),
    };
    assert_eq!(
        line.chord_segments(Interval::new(0.0, 100.0).unwrap(), 1e-6),
        1
    );
    assert_eq!(line.chord_segments(Interval::REAL, 1e-6), 1);
    check(
        (
            radius(0.1..=100.0),
            finite_f64(-6.0..=-1.0),
            finite_f64(0.1..=TAU),
        ),
        |(r, log_chord, length)| {
            let chord = real::powf(10f64, log_chord) * r;
            let range = Interval::new(0.0, length).unwrap();
            let circle = Curve::Circle {
                frame: Frame::world(),
                radius: r,
            };
            let twin = Curve2::Circle {
                frame: Frame2::identity(),
                radius: r,
            };
            let n = circle.chord_segments(range, chord);
            prop_assert_eq!(n, Piece::along(&twin, range).segment_count(chord));
            // The sagitta of one segment is within the chord, and one
            // fewer segment would not be (unless the floor applies).
            let theta = length / n as f64;
            prop_assert!(r * (1.0 - real::cos(theta / 2.0)) <= chord * (1.0 + 1e-12));
            if n > MIN_SEGMENTS_PER_TURN {
                let coarser = length / (n - 1) as f64;
                prop_assert!(r * coarser * coarser / 8.0 > chord);
            }
            Ok(())
        },
    );
    let circle = Curve::Circle {
        frame: Frame::world(),
        radius: 1.0,
    };
    assert_eq!(
        circle.chord_segments(Interval::TURN, f64::INFINITY),
        MIN_SEGMENTS_PER_TURN
    );
    assert_eq!(
        circle.chord_segments(Interval::TURN, 0.0),
        MAX_SEGMENTS_PER_PIECE
    );
    assert_eq!(
        circle.chord_segments(Interval::TURN, 1e-300),
        MAX_SEGMENTS_PER_PIECE
    );
}

#[test]
fn a_plane_and_the_ruled_directions_are_unbounded() {
    check(
        (plane(), cylinder(), cone(), finite_f64(-6.0..=-1.0)),
        |(p, cyl, cn, log_chord)| {
            let chord = real::powf(10f64, log_chord);
            prop_assert_eq!(p.chord_steps(chord, p.domain()), [f64::INFINITY; 2]);
            let Surface::Cylinder { radius, .. } = cyl else {
                unreachable!()
            };
            let [hu, hv] = cyl.chord_steps(chord, cyl.domain());
            prop_assert!((hu - (8.0 * chord / radius).sqrt()).abs() <= 1e-15 * hu);
            prop_assert_eq!(hv, f64::INFINITY);
            // A cone over v in [0, 5]: the u step from the wider end.
            let Surface::Cone {
                radius, half_angle, ..
            } = cn
            else {
                unreachable!()
            };
            let v = Interval::new(0.0, 5.0).unwrap();
            let [hu, hv] = cn.chord_steps(chord, [Interval::TURN, v]);
            let rho = (radius + 5.0 * real::sin(half_angle)).max(radius);
            let k = rho * real::cos(half_angle);
            prop_assert!((hu - (8.0 * chord / k).sqrt()).abs() <= 1e-15 * hu);
            prop_assert_eq!(hv, f64::INFINITY);
            // An unbounded v leaves no step at all.
            prop_assert_eq!(cn.chord_steps(chord, cn.domain())[0], 0.0);
            Ok(())
        },
    );
}

#[test]
fn the_doubly_curved_kinds_share_the_chord() {
    check(
        (sphere(), torus(), finite_f64(-6.0..=-1.0)),
        |(s, t, log_chord)| {
            let chord = real::powf(10f64, log_chord);
            let Surface::Sphere { radius, .. } = s else {
                unreachable!()
            };
            let h = (4.0 * chord / radius).sqrt();
            let [hu, hv] = s.chord_steps(chord, s.domain());
            prop_assert!((hu - h).abs() <= 1e-15 * h && (hv - h).abs() <= 1e-15 * h);
            let Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } = t
            else {
                unreachable!()
            };
            let [hu, hv] = t.chord_steps(chord, t.domain());
            let (eu, ev) = (
                (4.0 * chord / (major_radius + minor_radius)).sqrt(),
                (4.0 * chord / minor_radius).sqrt(),
            );
            prop_assert!((hu - eu).abs() <= 1e-15 * eu && (hv - ev).abs() <= 1e-15 * ev);
            Ok(())
        },
    );
}

#[test]
fn a_bilinear_nurbs_plane_is_unbounded_and_a_zero_chord_gives_no_step() {
    let flat = Surface::Nurbs(
        NurbsSurface::new(
            [1, 1],
            [vec![0.0, 0.0, 2.0, 2.0], vec![0.0, 0.0, 3.0, 3.0]],
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 3.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 3.0, 0.0),
            ],
            vec![1.0; 4],
        )
        .unwrap(),
    );
    assert_eq!(flat.chord_steps(1e-3, flat.domain()), [f64::INFINITY; 2]);
    let cyl = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    assert_eq!(cyl.chord_steps(0.0, cyl.domain()), [0.0, f64::INFINITY]);
    // The sphere's v range is a quarter turn each way.
    let s = Surface::Sphere {
        frame: Frame::world(),
        radius: 1.0,
    };
    assert_eq!(s.domain()[1], Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap());
}
