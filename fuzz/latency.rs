//! Measures the longest stretch between two ticks on every `slow-unit-*`
//! input the intersector targets have kept (ADR-0030): each is run with a
//! poll that turns true after a time budget, and the poll's own question
//! spacing is the stretch between ticks.
//!
//! ```sh
//! cargo run --release --manifest-path fuzz/Cargo.toml --example latency -- 3
//! ```

use std::sync::Mutex;
use std::time::{Duration, Instant};

use arris_fuzz::{Decoder, tolerance};
use arris_geom::{Curve, Surface, intersect_curve_surface, intersect_curves, intersect_surfaces};
use arris_math::{Aabb, Control, Meter};

enum Operands {
    Surfaces(Surface, Surface, Aabb),
    CurveSurface(Curve, Surface),
    Curves(Curve, Curve),
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let secs: f64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(3.0);
    let mut worst = (Duration::ZERO, String::new());
    let mut count = 0;
    for target in [
        "intersect_surfaces",
        "intersect_curve_surface",
        "intersect_curves",
    ] {
        let dir = format!("{}/artifacts/{target}", env!("CARGO_MANIFEST_DIR"));
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        let mut paths: Vec<_> = entries.filter_map(Result::ok).map(|e| e.path()).collect();
        paths.sort();
        for path in paths {
            if !path
                .file_name()
                .is_some_and(|n| n.to_string_lossy().starts_with("slow-unit"))
            {
                continue;
            }
            let data = std::fs::read(&path)?;
            // Decoded first: a decoded curve may be a section the decoder
            // fitted, which is the decoder's time and not the query's.
            let mut d = Decoder::new(&data);
            let operands = match target {
                "intersect_surfaces" => d
                    .surface()
                    .zip(d.surface())
                    .zip(d.region())
                    .map(|((a, b), w)| Operands::Surfaces(a, b, w)),
                "intersect_curve_surface" => d
                    .curve()
                    .zip(d.surface())
                    .map(|(c, s)| Operands::CurveSurface(c, s)),
                _ => d
                    .curve()
                    .zip(d.curve())
                    .map(|(a, b)| Operands::Curves(a, b)),
            };
            let Some(operands) = operands else {
                continue;
            };
            let start = Instant::now();
            let seen = Mutex::new((start, Duration::ZERO));
            let poll = || {
                let mut seen = seen.lock().unwrap();
                let now = Instant::now();
                seen.1 = seen.1.max(now - seen.0);
                seen.0 = now;
                now - start >= Duration::from_secs_f64(secs)
            };
            let control = Control::poll(&poll);
            let meter = &mut Meter::new(&control);
            match operands {
                Operands::Surfaces(a, b, w) => {
                    let _ = intersect_surfaces(&a, &b, &w, tolerance(), meter);
                }
                Operands::CurveSurface(c, s) => {
                    let _ = intersect_curve_surface(&c, &s, tolerance(), meter);
                }
                Operands::Curves(a, b) => {
                    let _ = intersect_curves(&a, &b, tolerance(), meter);
                }
            }
            // The last stretch, from the final question to the return.
            {
                let mut seen = seen.lock().unwrap();
                let end = Instant::now();
                seen.1 = seen.1.max(end - seen.0);
            }
            let (_, gap) = *seen.lock().unwrap();
            let name = path.file_name().unwrap().to_string_lossy().into_owned();
            println!("{target} {name} {:?} max stretch {gap:?}", start.elapsed());
            if gap > worst.0 {
                worst = (gap, format!("{target}/{name}"));
            }
            count += 1;
        }
    }
    println!(
        "{count} inputs; longest stretch {:?} on {}",
        worst.0, worst.1
    );
    Ok(())
}
