//! `cargo run -p arris-debug --example render_labelled -- <mesh> <labels.json>
//! <out.png> [iso|top|front|right]`: a mesh coloured by a label per
//! triangle, Lambert-shaded and supersampled (`arris_debug::render_labelled`).
//!
//! `<mesh>` is what `crates/arris-py/examples/bracket.py` writes: two
//! little-endian `u32` counts (positions, triangles), the positions as
//! `f64` triples, the triangles as `u32` triples. `<labels.json>` is
//! `{"names": [...], "triangles": [label per triangle]}`. `tools/showcase.sh`
//! runs it.

use arris_debug::{View, render_labelled};
use arris_mesh::TriMesh;

/// One colour per label, in the order of the names; mid-tone so the
/// shading shows on every one.
const PALETTE: [[u8; 3]; 8] = [
    [150, 160, 175],
    [60, 130, 200],
    [225, 130, 40],
    [80, 175, 110],
    [190, 80, 160],
    [225, 190, 50],
    [60, 175, 185],
    [210, 80, 70],
];

fn read_u32(bytes: &[u8], at: usize) -> Result<u32, String> {
    let word = bytes.get(at..at + 4).ok_or("the mesh file ends early")?;
    Ok(u32::from_le_bytes(
        word.try_into().map_err(|_| "short word")?,
    ))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [mesh_path, labels_path, out, rest @ ..] = args.as_slice() else {
        eprintln!("usage: render_labelled <mesh> <labels.json> <out.png> [iso|top|front|right]");
        std::process::exit(2);
    };
    let view = match rest.first().map(String::as_str) {
        None | Some("iso") => View::Iso,
        Some("top") => View::Top,
        Some("front") => View::Front,
        Some("right") => View::Right,
        Some(other) => return Err(format!("no view {other}").into()),
    };
    let bytes = std::fs::read(mesh_path)?;
    let (n_positions, n_triangles) = (read_u32(&bytes, 0)? as usize, read_u32(&bytes, 4)? as usize);
    if bytes.len() != 8 + 24 * n_positions + 12 * n_triangles {
        return Err("the mesh file is not its header's size".into());
    }
    let positions: Vec<[f64; 3]> = (0..n_positions)
        .map(|i| {
            let at = 8 + 24 * i;
            let coord = |k: usize| {
                f64::from_le_bytes(bytes[at + 8 * k..at + 8 * k + 8].try_into().unwrap())
            };
            [coord(0), coord(1), coord(2)]
        })
        .collect();
    let base = 8 + 24 * n_positions;
    let triangles = (0..n_triangles)
        .map(|i| {
            let word = |k: usize| read_u32(&bytes, base + 12 * i + 4 * k);
            Ok([word(0)?, word(1)?, word(2)?])
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mesh = TriMesh::from_parts(positions, triangles, vec![], vec![], vec![])?;
    let document: serde_json::Value = serde_json::from_str(&std::fs::read_to_string(labels_path)?)?;
    let labels: Vec<usize> = document["triangles"]
        .as_array()
        .ok_or("no `triangles` list")?
        .iter()
        .map(|v| {
            v.as_u64()
                .map(|n| n as usize)
                .ok_or("a label is not a number")
        })
        .collect::<Result<_, _>>()?;
    let raster = render_labelled(&mesh, &labels, &PALETTE, view)?;
    raster.save(out.as_ref())?;
    println!("{out}");
    Ok(())
}
