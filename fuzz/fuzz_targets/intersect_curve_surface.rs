//! `intersect_curve_surface` on a line, a conic, a NURBS curve or a
//! fitted section against an analytic surface: no panic, every hit on
//! both within the tolerance, the same answer twice.
#![no_main]

use arris_fuzz::{Decoder, check_curve_surface, check_deterministic, tolerance};
use arris_geom::intersect_curve_surface;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut d = Decoder::new(data);
    let (Some(c), Some(s)) = (d.curve(), d.surface()) else {
        return;
    };
    let first = intersect_curve_surface(&c, &s, tolerance(), &mut arris_math::Meter::default());
    let second = intersect_curve_surface(&c, &s, tolerance(), &mut arris_math::Meter::default());
    check_deterministic(&first, &second);
    if let Ok(hit) = &first {
        check_curve_surface(&c, &s, hit);
    }
});
