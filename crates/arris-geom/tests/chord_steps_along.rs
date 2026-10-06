//! `Surface::chord_steps_along` (ADR-0052 §1): an edge's chord steps are a
//! function of the surface and the (u, v) band the edge reaches, never of
//! the face's whole box.

use arris_debug::prop::check;
use arris_debug::prop::geom::{nurbs_surface, surface};
use arris_geom::{Surface, SurfaceKind};
use arris_math::Interval;
use proptest::prelude::*;

const CHORD: f64 = 1e-3;

#[test]
fn a_nurbs_surfaces_steps_are_its_whole_domains_whatever_the_band() {
    check((nurbs_surface(), 0.0..0.5f64, 0.5..1.0f64), |(s, a, b)| {
        let surf = Surface::Nurbs(s);
        let d = surf.domain();
        let band = [
            Interval::new(d[0].lerp(a), d[0].lerp(b)).unwrap(),
            Interval::new(d[1].lerp(a), d[1].lerp(b)).unwrap(),
        ];
        let whole = surf.chord_steps(CHORD, d);
        prop_assert_eq!(surf.chord_steps_along(CHORD, band), whole);
        prop_assert_eq!(surf.chord_steps_along(CHORD, d), whole);
        Ok(())
    });
}

#[test]
fn every_other_kind_reads_the_band_as_chord_steps_does() {
    check((surface(), 0.0..0.5f64, 0.5..1.0f64), |(surf, a, b)| {
        prop_assume!(surf.kind() != SurfaceKind::Nurbs);
        let d = surf.domain();
        let pick = |i: usize| {
            let (lo, hi) = (d[i].lerp(a), d[i].lerp(b));
            Interval::new(lo, hi).unwrap_or(d[i])
        };
        let band = [pick(0), pick(1)];
        prop_assert_eq!(
            surf.chord_steps_along(CHORD, band),
            surf.chord_steps(CHORD, band)
        );
        Ok(())
    });
}
