//! `intersect_surfaces` on two analytic surfaces in any pose, in a cube
//! about the origin: no panic, every curve and point of the answer on
//! both surfaces within the tolerance, the same answer twice. A trailing
//! byte twins either surface as a NURBS one, so the marched section of
//! two NURBS surfaces (ADR-0068) is drawn too.
#![no_main]

use arris_fuzz::{Decoder, check_deterministic, check_surfaces, tolerance, twin};
use arris_geom::intersect_surfaces;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut d = Decoder::new(data);
    let (Some(a), Some(b), Some(within)) = (d.surface(), d.surface(), d.region()) else {
        return;
    };
    let [twin_a, twin_b] = d.twins();
    let a = if twin_a { twin(&a).unwrap_or(a) } else { a };
    let b = if twin_b { twin(&b).unwrap_or(b) } else { b };
    let first = intersect_surfaces(
        &a,
        &b,
        &within,
        tolerance(),
        &mut arris_math::Meter::default(),
    );
    let second = intersect_surfaces(
        &a,
        &b,
        &within,
        tolerance(),
        &mut arris_math::Meter::default(),
    );
    check_deterministic(&first, &second);
    if let Ok(hit) = &first {
        check_surfaces(&a, &b, &within, hit);
    }
});
