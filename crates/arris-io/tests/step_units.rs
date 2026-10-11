//! The writer declares the caller's unit (A12, ADR-0025 §5): a body
//! written with `WriteOptions { length_unit }` and read back with
//! `ReadOptions` of the same unit has every vertex bit-identical, read in
//! millimetres it is scaled by the unit's factor, and Open CASCADE, which
//! reads everything into millimetres, sees the metre file a thousand times
//! larger in every length.

use arris_debug::corpus;
use arris_debug::fixtures;
use arris_debug::prop::body::{QuadricSolid, quadric_solid};
use arris_debug::testing::fail;
use arris_debug::unmetered::{step_read, transform};
use arris_debug::{oracle, prop, prop_shards};
use arris_io::step::{self, LengthUnit, ReadOptions, WriteOptions};
use arris_math::{Control, Isometry};
use arris_topo::{Body, Model};
use proptest::prelude::*;

const UNITS: [LengthUnit; 6] = [
    LengthUnit::Micrometre,
    LengthUnit::Millimetre,
    LengthUnit::Centimetre,
    LengthUnit::Metre,
    LengthUnit::Inch,
    LengthUnit::Foot,
];

/// One `unit` in millimetres.
fn millimetres(unit: LengthUnit) -> f64 {
    match unit {
        LengthUnit::Micrometre => 1e-3,
        LengthUnit::Millimetre => 1.0,
        LengthUnit::Centimetre => 10.0,
        LengthUnit::Metre => 1000.0,
        LengthUnit::Inch => 25.4,
        LengthUnit::Foot => 304.8,
    }
}

/// The vertex coordinates of `body`, sorted by bit pattern.
fn vertices(m: &Model, body: Body) -> Result<Vec<[u64; 3]>, TestCaseError> {
    let mut out = Vec::new();
    for v in m.closure(body).map_err(fail)?.vertices {
        let p = m.vertex(v).map_err(fail)?.point();
        out.push([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
    }
    out.sort_unstable();
    Ok(out)
}

fn read_all(text: &str, unit: LengthUnit) -> Result<(Model, Vec<Body>), TestCaseError> {
    let mut back = Model::default();
    let read = step_read(&mut back, text, &ReadOptions { length_unit: unit }).map_err(fail)?;
    let mut bodies = Vec::new();
    for solid in &read.solids {
        let r = solid.result.as_ref().map_err(|r| fail(format!("{r}")))?;
        bodies.push(r.body);
    }
    Ok((back, bodies))
}

fn round_trip(solid: QuadricSolid, pose: Isometry) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let body = solid.build(&mut m).map_err(fail)?;
    let body = transform(&mut m, body, &pose).map_err(fail)?.0;
    let written = vertices(&m, body)?;
    for unit in UNITS {
        let options = WriteOptions { length_unit: unit };
        let text = step::write_with(&m, &[body], &options).map_err(fail)?;
        let (back, bodies) = read_all(&text, unit)?;
        let mut read = Vec::new();
        for &b in &bodies {
            read.extend(vertices(&back, b)?);
        }
        read.sort_unstable();
        prop_assert_eq!(&read, &written, "{:?} read in its own unit", unit);

        let (mm, bodies) = read_all(&text, LengthUnit::Millimetre)?;
        let factor = millimetres(unit);
        let mut scaled = Vec::new();
        for &b in &bodies {
            scaled.extend(vertices(&mm, b)?);
        }
        scaled.sort_unstable();
        prop_assert_eq!(scaled.len(), written.len());
        // The nearest vertex, not the same index: scaling can reorder.
        for p in &written {
            let p = p.map(f64::from_bits);
            let want = p.map(|x| x * factor);
            let near = scaled.iter().any(|q| {
                let q = q.map(f64::from_bits);
                (0..3).all(|k| (q[k] - want[k]).abs() <= 1e-12 * (want[k].abs() + factor))
            });
            prop_assert!(
                near,
                "{unit:?}: {want:?} in millimetres is not among the read"
            );
        }
    }
    Ok(())
}

prop_shards! {
    /// Written in a unit and read in the same, every vertex is the same
    /// bits; read in millimetres, it is the unit's factor larger.
    a_unit_round_trips_bit_exact
        [shard_0 shard_1 shard_2 shard_3]
        ((solid, pose)) = (quadric_solid(), prop::pose()) => {
            round_trip(solid, pose)
        }
}

#[test]
fn the_default_writes_millimetres() {
    let mut m = Model::default();
    let body = arris_debug::sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = step::write(&m, &[body]).unwrap();
    assert_eq!(
        step::write_with(&m, &[body], &WriteOptions::default()).unwrap(),
        text
    );
    assert!(text.contains("SI_UNIT(.MILLI.,.METRE.)"));
    let metres = WriteOptions {
        length_unit: LengthUnit::Metre,
    };
    let in_metres = step::write_with(&m, &[body], &metres).unwrap();
    assert!(in_metres.contains("LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.)"));
    assert!(!in_metres.contains(".MILLI."));
    let inches = WriteOptions {
        length_unit: LengthUnit::Inch,
    };
    let in_inches = step::write_with(&m, &[body], &inches).unwrap();
    assert!(in_inches.contains("CONVERSION_BASED_UNIT('INCH'"));
    // Only the unit entities differ: no coordinate is converted.
    let strip = |t: &str| t.lines().filter(|l| l.contains("CARTESIAN_POINT")).count();
    assert_eq!(strip(&text), strip(&in_metres));
}

/// The products writer declares the unit, and reads back the same.
#[test]
fn the_products_writer_declares_the_unit() {
    use arris_io::step::{Occurrence, ProductTree};
    let mut m = Model::default();
    let body = arris_debug::sample::cylinder(&mut m, 0.004, 0.012).unwrap();
    let part = Occurrence {
        product: None,
        name: "pin".into(),
        placement: Ok(Isometry::identity()),
        colour: None,
        solids: vec![0],
        children: vec![],
    };
    let tree = ProductTree {
        roots: vec![part],
        faces: vec![],
    };
    for unit in UNITS {
        let options = WriteOptions { length_unit: unit };
        let text = step::write_products_with(&m, &[body], &tree, &options, &Control::NONE).unwrap();
        let (back, bodies) = read_all(&text, unit).unwrap();
        assert_eq!(
            vertices(&back, bodies[0]).unwrap(),
            vertices(&m, body).unwrap(),
            "{unit:?}"
        );
    }
}

/// Open CASCADE reads a metre file of the corpus's box into millimetres:
/// the box's numbers as metres, a thousand times longer, 10⁹ × the volume.
#[test]
fn open_cascade_reads_the_metre_file_a_thousand_times_larger() {
    let dir = fixtures::corpus_root().join("primitive/box");
    let chain = corpus::chain(&dir, "default").unwrap();
    let body = chain.result().unwrap();
    let metres = WriteOptions {
        length_unit: LengthUnit::Metre,
    };
    let text = step::write_with(&chain.model, &[body], &metres).unwrap();
    let read = oracle::occt_read_assembly(&text, "units-box-metre").unwrap();
    let volume = read.instances[0].volume;
    assert!(
        (volume - 12000.0e9).abs() <= 1e-9 * 12000.0e9,
        "{volume} vs 1.2e13"
    );
    let mm = step::write(&chain.model, &[body]).unwrap();
    let read = oracle::occt_read_assembly(&mm, "units-box-mm").unwrap();
    assert!((read.instances[0].volume - 12000.0).abs() <= 1e-9 * 12000.0);
}
