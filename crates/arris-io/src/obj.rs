//! OBJ (`docs/ARCHITECTURE.md` §Formats and tools, ADR-0013): `write` of
//! one or several [`arris_mesh::TriMesh`]es (one per body, as
//! `step::write` takes several bodies of one `Model` — facade-swap step
//! 5), `v`/`f` always and `vt`/`vn` when a mesh carries a
//! [`arris_mesh::Corners`] block.
//!
//! `v` is [`TriMesh::positions`](arris_mesh::TriMesh::positions), shared
//! across faces as the mesh itself shares it. When the mesh has no
//! corner block, `f` names three `v` indices and nothing else — OBJ's
//! bare `f v1 v2 v3` — and there is one `g <face>` per
//! [`arris_mesh::FaceRange`], partitioning the triangles by face in the
//! body's iteration order.
//!
//! When it does, `vt` is [`Corners::uvs`](arris_mesh::Corners::uvs) and
//! `vn` is [`Corners::normals`](arris_mesh::Corners::normals), one of
//! each per face-local vertex, and `f` is `v/vt/vn v/vt/vn v/vt/vn` —
//! the shared position from the mesh's own triangle, the (u, v) and
//! normal from [`Corners::triangles`](arris_mesh::Corners::triangles)'s
//! parallel face-local one, so a corner's `vt` and `vn` are always the
//! same index: OBJ has no way to name one without the other, and a
//! face-local vertex has exactly one of each. That is what a per-corner
//! render buffer is for.
//!
//! Several meshes are written one after another, each with its own `v`
//! (and `vt`/`vn` if it has corners) followed by its own `g`/`f` lines,
//! every index of a mesh past the first offset by the running total of
//! the meshes before it — OBJ's `v`/`vt`/`vn` indices are shared across
//! the whole file, unlike STL's, which has none to offset.
//!
//! Every real is Rust's shortest round-trip decimal, `f64` throughout
//! (ADR-0011), so two writes of the same meshes are byte-identical.

use core::fmt::Write as _;

use arris_mesh::TriMesh;

use crate::MeshWriteError;

/// The OBJ text of `meshes`. Never fails: OBJ has no bound this crate
/// enforces.
///
/// ```
/// use arris_mesh::TriMesh;
/// use arris_io::obj;
///
/// let mut mesh = TriMesh::new();
/// for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
///     mesh.push_position(p).unwrap();
/// }
/// mesh.push_face(
///     arris_topo::FaceId::new(0, 0),
///     [[0, 1, 2]],
/// )
/// .unwrap();
/// let text = obj::write(std::slice::from_ref(&mesh)).unwrap();
/// assert!(text.contains("v 0. 0. 0."));
/// assert!(text.contains("f 1 2 3"));
/// assert!(!text.contains("vt"), "no corner block, no vt");
/// ```
pub fn write(meshes: &[TriMesh]) -> Result<String, MeshWriteError> {
    let mut out = String::new();
    let mut position_offset = 0u32;
    let mut corner_offset = 0u32;
    for mesh in meshes {
        for p in mesh.positions() {
            let _ = writeln!(out, "v {} {} {}", real(p[0]), real(p[1]), real(p[2]));
        }
        if let Some(corners) = mesh.corners() {
            for uv in corners.uvs() {
                let _ = writeln!(out, "vt {} {}", real(uv[0]), real(uv[1]));
            }
            for n in corners.normals() {
                let _ = writeln!(out, "vn {} {} {}", real(n[0]), real(n[1]), real(n[2]));
            }
        }
        for range in mesh.faces() {
            let _ = writeln!(out, "g {}", range.face);
            for i in range.triangles.clone() {
                let v = mesh.triangles()[i];
                if let Some(corners) = mesh.corners() {
                    let c = corners.triangles()[i];
                    let _ = writeln!(
                        out,
                        "f {}/{}/{} {}/{}/{} {}/{}/{}",
                        v[0] + 1 + position_offset,
                        c[0] + 1 + corner_offset,
                        c[0] + 1 + corner_offset,
                        v[1] + 1 + position_offset,
                        c[1] + 1 + corner_offset,
                        c[1] + 1 + corner_offset,
                        v[2] + 1 + position_offset,
                        c[2] + 1 + corner_offset,
                        c[2] + 1 + corner_offset,
                    );
                } else {
                    let _ = writeln!(
                        out,
                        "f {} {} {}",
                        v[0] + 1 + position_offset,
                        v[1] + 1 + position_offset,
                        v[2] + 1 + position_offset,
                    );
                }
            }
        }
        position_offset += mesh.positions().len() as u32;
        if let Some(corners) = mesh.corners() {
            corner_offset += corners.uvs().len() as u32;
        }
    }
    Ok(out)
}

/// A finite real in OBJ's spelling: Rust's shortest round-trip decimal
/// with a decimal point always present, mirroring `step::real` and
/// `stl::real`.
fn real(x: f64) -> String {
    let x = if x == 0.0 { 0.0 } else { x };
    let s = format!("{x}");
    if s.contains('.') { s } else { format!("{s}.") }
}

#[cfg(test)]
mod tests {
    use arris_topo::FaceId;

    use super::*;

    fn triangle() -> TriMesh {
        let mut m = TriMesh::new();
        for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
            m.push_position(p).unwrap();
        }
        m.push_face(FaceId::new(0, 0), [[0, 1, 2]]).unwrap();
        m
    }

    #[test]
    fn a_mesh_without_corners_has_no_vt_or_vn() {
        let text = write(std::slice::from_ref(&triangle())).unwrap();
        assert!(text.contains("v 0. 0. 0."));
        assert!(text.contains("g f0"));
        assert!(text.contains("f 1 2 3"));
        assert!(!text.contains("vt"));
        assert!(!text.contains("vn"));
    }

    #[test]
    fn a_second_mesh_offsets_its_v_indices_by_the_firsts_count() {
        let text = write(&[triangle(), triangle()]).unwrap();
        assert_eq!(text.matches("v 0. 0. 0.").count(), 2);
        assert!(text.contains("f 1 2 3"), "the first mesh's own indices");
        assert!(
            text.contains("f 4 5 6"),
            "the second mesh's indices offset by the first's 3 positions: {text}"
        );
    }

    #[test]
    fn reals_are_shortest_round_trip_with_a_point() {
        assert_eq!(real(40.0), "40.");
        assert_eq!(real(0.5), "0.5");
        assert_eq!(real(-0.0), "0.");
    }
}
