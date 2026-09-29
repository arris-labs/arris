//! `intersect_curves` on two lines, conics, NURBS curves or fitted
//! sections: no panic, every hit on both within the tolerance, the same
//! answer twice.
#![no_main]

use arris_fuzz::{Decoder, check_curves, check_deterministic, tolerance};
use arris_geom::intersect_curves;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut d = Decoder::new(data);
    let (Some(a), Some(b)) = (d.curve(), d.curve()) else {
        return;
    };
    let first = intersect_curves(&a, &b, tolerance(), &mut arris_math::Meter::default());
    let second = intersect_curves(&a, &b, tolerance(), &mut arris_math::Meter::default());
    check_deterministic(&first, &second);
    if let Ok(hit) = &first {
        check_curves(&a, &b, hit);
    }
});
