//! Prints what a fuzz input decodes to and what the intersector answers:
//! the first look at a crash before it is shrunk into a fixture.
//!
//! ```sh
//! cargo run --manifest-path fuzz/Cargo.toml --example show -- intersect_curves fuzz/artifacts/intersect_curves/crash-…
//! ```

use arris_fuzz::{Decoder, tolerance};
use arris_geom::{intersect_curve_surface, intersect_curves, intersect_surfaces};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let (Some(target), Some(path)) = (args.next(), args.next()) else {
        return Err("usage: show <target> <input>".into());
    };
    let data = std::fs::read(path)?;
    let mut d = Decoder::new(&data);
    match target.as_str() {
        "intersect_surfaces" => {
            let (a, b, within) = (d.surface(), d.surface(), d.region());
            println!("a = {a:#?}\nb = {b:#?}\nwithin = {within:?}");
            if let (Some(a), Some(b), Some(within)) = (a, b, within) {
                println!(
                    "{:#?}",
                    intersect_surfaces(
                        &a,
                        &b,
                        &within,
                        tolerance(),
                        &mut arris_math::Meter::default()
                    )
                );
            }
        }
        "intersect_curve_surface" => {
            let (c, s) = (d.curve(), d.surface());
            println!("curve = {c:#?}\nsurface = {s:#?}");
            if let (Some(c), Some(s)) = (c, s) {
                println!(
                    "{:#?}",
                    intersect_curve_surface(&c, &s, tolerance(), &mut arris_math::Meter::default())
                );
            }
        }
        "intersect_curves" => {
            let (a, b) = (d.curve(), d.curve());
            println!("a = {a:#?}\nb = {b:#?}");
            if let (Some(a), Some(b)) = (a, b) {
                println!(
                    "{:#?}",
                    intersect_curves(&a, &b, tolerance(), &mut arris_math::Meter::default())
                );
            }
        }
        other => return Err(format!("unknown target {other}").into()),
    }
    Ok(())
}
