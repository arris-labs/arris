//! The product tree the STEP reader returns beside the flattened bodies
//! (ADR-0033): Open CASCADE's XCAF assembly reads to the oracle's names,
//! nesting, placements and colours, and a plain file of one part reads to
//! one root holding it.

use arris_debug::oracle::{self, OracleOccurrence};
use arris_debug::unmetered::step_read;
use arris_debug::{fixtures, sample};
use arris_io::arris_check::arris_topo::Model;
use arris_io::step::{self, Occurrence, ReadOptions};

/// A placement as the oracle prints it, a 3×4 row-major matrix.
fn matrix(placement: &arris_io::arris_check::arris_topo::arris_math::Isometry) -> [f64; 12] {
    let r = placement.rotation().to_rotation_matrix();
    let t = placement.translation();
    let mut out = [0.0; 12];
    for row in 0..3 {
        for col in 0..3 {
            out[4 * row + col] = r[(row, col)];
        }
        out[4 * row + 3] = t[row];
    }
    out
}

/// `found` is `expected`: names, placements, colours, the solids each holds
/// and the nesting, recursively.
fn same(found: &Occurrence, expected: &OracleOccurrence, path: &str) -> Result<(), String> {
    if found.name != expected.name {
        return Err(format!(
            "{path}: name {:?} vs {:?}",
            found.name, expected.name
        ));
    }
    match (&found.placement, expected.placement) {
        (Ok(p), Some(want)) => {
            let got = matrix(p);
            for (i, (g, w)) in got.iter().zip(want).enumerate() {
                if (g - w).abs() > 1e-9 {
                    return Err(format!("{path}: placement {got:?} vs {want:?} at {i}"));
                }
            }
        }
        (Ok(p), None)
            if matrix(p)
                .iter()
                .zip(IDENTITY)
                .all(|(a, b)| (a - b).abs() < 1e-12) => {}
        (other, want) => return Err(format!("{path}: placement {other:?} vs {want:?}")),
    }
    if found.solids != expected.solids {
        return Err(format!(
            "{path}: solids {:?} vs {:?}",
            found.solids, expected.solids
        ));
    }
    if found.children.len() != expected.children.len() {
        return Err(format!(
            "{path}: {} children vs {}",
            found.children.len(),
            expected.children.len()
        ));
    }
    for (i, (f, e)) in found.children.iter().zip(&expected.children).enumerate() {
        same(f, e, &format!("{path}/{i}"))?;
    }
    Ok(())
}

const IDENTITY: [f64; 12] = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0];

/// Open CASCADE's XCAF file — a named assembly of a part placed once and a
/// sub-assembly that holds another placed twice — reads to the oracle's
/// tree, each leaf's solids the bodies whose volume and centroid the
/// oracle measured, the flattened bodies as they were.
#[test]
fn an_xcaf_assembly_reads_to_the_oracles_tree() {
    let root = fixtures::corpus_root();
    let (text, oracle) = oracle::occt_assembly_oracle(
        &root.join("primitive/box"),
        &root.join("boolean/through-hole"),
        "occt-assembly-box-through-hole",
    )
    .unwrap();
    let mut m = Model::default();
    let read = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), oracle.instances.len());
    assert_eq!(read.products.roots.len(), 1, "{:#?}", read.products.roots);
    same(&read.products.roots[0], &oracle.tree, "").unwrap();
    // A leaf's solid is the body the oracle measured for that instance.
    for occurrence in read.products.occurrences() {
        for &i in &occurrence.solids {
            let back = read.solids[i].result.as_ref().unwrap();
            let mass = arris_debug::unmetered::mass_properties(&m, back.body).unwrap();
            let want = &oracle.instances[i];
            assert!((mass.volume - want.volume).abs() <= 1e-9 * want.volume);
        }
    }
}

/// A file of one part — Arris's own write of a cylinder — reads to one
/// root of one occurrence, named for the product, at the identity,
/// holding the one solid.
#[test]
fn a_plain_file_reads_to_one_root_of_one_occurrence() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = step::write(&m, &[body]).unwrap();
    let mut back = Model::default();
    let read = step_read(&mut back, &text, &ReadOptions::default()).unwrap();
    let roots = &read.products.roots;
    assert_eq!(roots.len(), 1, "{roots:#?}");
    let root = &roots[0];
    assert_eq!(root.name, "arris");
    assert!(root.product.is_some());
    assert_eq!(root.solids, vec![0]);
    assert!(root.children.is_empty());
    assert_eq!(matrix(root.placement.as_ref().unwrap()), IDENTITY);
    assert!(read.products.faces.is_empty());
}
