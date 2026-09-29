//! Writes the fuzz targets' seed corpus from every `tests/fixtures/geom/`
//! pair: each surface pair to `corpus/intersect_surfaces/`, each curve
//! against a surface to `corpus/intersect_curve_surface/`, each curve
//! pair to `corpus/intersect_curves/`, and every curve a surface pair's
//! section returns against both of its surfaces, as a section input, to
//! `corpus/intersect_curve_surface/` — the fitted curves the fixtures
//! only reach through a boolean — and every solid fixture's STEP to
//! `corpus/step_read/`: Arris's own, and Open CASCADE's where the
//! oracle's environment is there — and every file of body bytes'
//! guard, each version's `.bin` and `.json`, to `corpus/body_read/`.
//!
//! ```sh
//! cargo run --manifest-path fuzz/Cargo.toml --example seed
//! ```
//!
//! Each file is named after its fixture and pair, so a second run
//! rewrites the same files. The region is the oracle test's, a cube of
//! 100 about the origin.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use arris_debug::corpus::chain;
use arris_debug::fixtures::geom::{build_curve, build_surface, load};
use arris_debug::fixtures::{self, Kind, corpus, kind_of, name_of};
use arris_debug::oracle;
use arris_fuzz::{Encoder, tolerance};
use arris_geom::intersect_surfaces;
use arris_io::step;
use arris_math::Aabb;

const HALF_EXTENT: f64 = 100.0;

fn write(dir: &Path, name: &str, bytes: Vec<u8>) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    std::fs::write(dir.join(name), bytes)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("corpus");
    let surfaces_dir = root.join("intersect_surfaces");
    let curve_surface_dir = root.join("intersect_curve_surface");
    let curves_dir = root.join("intersect_curves");
    let step_dir = root.join("step_read");
    let within = Aabb {
        min: [-HALF_EXTENT; 3],
        max: [HALF_EXTENT; 3],
    };
    let mut counts = BTreeMap::<&str, usize>::new();
    for dir in corpus() {
        let kind = kind_of(&dir)?;
        // A part's STEP file is a whole real part, not a seed a mutation
        // gets far with; the corpus runner reads every one already.
        if kind == Kind::Part {
            continue;
        }
        if kind == Kind::Solid {
            // A result Arris refuses by design, or one still failing
            // under `regression/`, has no file to seed from.
            let slug = name_of(&dir).replace('/', "-");
            for variant in fixtures::load(&dir)?.expected.results.keys() {
                let Ok(built) = chain(&dir, variant) else {
                    continue;
                };
                let Some(body) = built.result() else {
                    continue;
                };
                let Ok(text) = step::write(&built.model, &[body]) else {
                    continue;
                };
                write(
                    &step_dir,
                    &format!("{slug}-{variant}.step"),
                    text.into_bytes(),
                )?;
                *counts.entry("step_read").or_default() += 1;
                // Open CASCADE's own file of the recipe, where the
                // oracle's environment is there: a seed with another
                // writer's habits. Not its B-spline conversion: those
                // take tens of seconds each with debug assertions on,
                // the checker after every operation, and the corpus
                // runner reads every one of them already.
                let tag = format!("seed-occt-{slug}-{variant}");
                if let Ok(text) = oracle::occt_step(&dir, Some(variant), false, &tag) {
                    write(&step_dir, &format!("{tag}.step"), text.into_bytes())?;
                    *counts.entry("step_read").or_default() += 1;
                }
            }
            continue;
        }
        let fixture = load(&dir)?;
        let r = &fixture.recipe;
        let slug = name_of(&dir).replace('/', "-");
        let surfaces = r
            .surfaces
            .iter()
            .map(|(n, s)| Ok((n.clone(), build_surface(n, s, &r.params)?)))
            .collect::<Result<BTreeMap<_, _>, Box<dyn std::error::Error>>>()?;
        let curves = r
            .curves
            .iter()
            .map(|(n, c)| Ok((n.clone(), build_curve(n, c, &r.params)?)))
            .collect::<Result<BTreeMap<_, _>, Box<dyn std::error::Error>>>()?;
        for pair in &r.pairs {
            let name = format!("{slug}-{}-{}", pair.a, pair.b);
            let mut e = Encoder::new();
            match (
                surfaces.get(&pair.a),
                surfaces.get(&pair.b),
                curves.get(&pair.a),
                curves.get(&pair.b),
            ) {
                (Some(a), Some(b), _, _) => {
                    if e.surface(a) && e.surface(b) {
                        e.region(HALF_EXTENT);
                        write(&surfaces_dir, &name, e.finish())?;
                        *counts.entry("intersect_surfaces").or_default() += 1;
                    }
                    let Ok(hit) = intersect_surfaces(
                        a,
                        b,
                        &within,
                        tolerance(),
                        &mut arris_math::Meter::default(),
                    ) else {
                        continue;
                    };
                    for pick in 0..hit.curves().len().min(usize::from(u8::MAX)) {
                        for (side, s) in [("a", a), ("b", b)] {
                            let mut e = Encoder::new();
                            if e.section(a, b, HALF_EXTENT, pick as u8) && e.surface(s) {
                                write(
                                    &curve_surface_dir,
                                    &format!("{name}-section{pick}-on-{side}"),
                                    e.finish(),
                                )?;
                                *counts.entry("intersect_curve_surface").or_default() += 1;
                            }
                        }
                    }
                }
                (_, Some(s), Some(c), _) => {
                    if e.curve(c) && e.surface(s) {
                        write(&curve_surface_dir, &name, e.finish())?;
                        *counts.entry("intersect_curve_surface").or_default() += 1;
                    }
                }
                (_, _, Some(a), Some(b)) => {
                    if e.curve(a) && e.curve(b) {
                        write(&curves_dir, &name, e.finish())?;
                        *counts.entry("intersect_curves").or_default() += 1;
                    }
                }
                _ => return Err(format!("{name}: a pair of unknown operands").into()),
            }
        }
    }
    // Body bytes' guard, every version: bytes and JSON a release wrote.
    let body_dir = root.join("body_read");
    let guard = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../crates/arris-io/tests/body");
    let mut versions: Vec<PathBuf> = std::fs::read_dir(&guard)?
        .map(|e| e.map(|e| e.path()))
        .collect::<Result<_, _>>()?;
    versions.retain(|p| p.is_dir());
    versions.sort();
    for version in versions {
        let tag = version.file_name().and_then(|n| n.to_str()).unwrap_or("v");
        let mut files: Vec<PathBuf> = std::fs::read_dir(&version)?
            .map(|e| e.map(|e| e.path()))
            .collect::<Result<_, _>>()?;
        files.retain(|p| matches!(p.extension().and_then(|x| x.to_str()), Some("bin" | "json")));
        files.sort();
        for file in files {
            let name = file.file_name().and_then(|n| n.to_str()).unwrap_or("body");
            write(&body_dir, &format!("{tag}-{name}"), std::fs::read(&file)?)?;
            *counts.entry("body_read").or_default() += 1;
        }
    }
    for (target, n) in counts {
        println!("{target}: {n} seeds");
    }
    Ok(())
}
