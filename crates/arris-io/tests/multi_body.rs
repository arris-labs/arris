//! Several bodies' meshes in one STL or OBJ file (`docs/ARCHITECTURE.md`
//! §Formats and tools, facade-swap step 5): the triangle count is the sum
//! of the meshes', and each mesh's own positions are unchanged, read back
//! independently of `arris_io` — STL through Open CASCADE's `RWStl`
//! (`tools/oracle/mesh.py`), OBJ through this test's own minimal parser.

use arris_debug::oracle::compare_stl;
use arris_debug::sample;
use arris_debug::unmetered::tessellate;
use arris_io::{obj, stl};
use arris_ops::arris_check::arris_topo::Model;
use arris_ops::arris_check::arris_topo::arris_math::Point3;

/// `v` lines and bare `f v1 v2 v3` lines, in file order. These meshes
/// carry no corner block, so OBJ's `f` is the bare form.
fn parse_obj_bare(text: &str) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for line in text.lines() {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("v") => {
                let c: Vec<f64> = it.map(|s| s.parse().unwrap()).collect();
                positions.push([c[0], c[1], c[2]]);
            }
            Some("f") => {
                let v: Vec<u32> = it.map(|s| s.parse::<u32>().unwrap() - 1).collect();
                triangles.push([v[0], v[1], v[2]]);
            }
            _ => {}
        }
    }
    (positions, triangles)
}

#[test]
fn stl_of_two_bodies_has_the_summed_triangle_count() {
    let mut m = Model::default();
    let a = sample::cuboid(&mut m, Point3::origin(), Point3::new(3.0, 2.0, 1.0)).unwrap();
    let b = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let mesh_a = tessellate(&m, a, 1e-2).unwrap();
    let mesh_b = tessellate(&m, b, 1e-2).unwrap();
    let meshes = [mesh_a.clone(), mesh_b.clone()];
    let total_triangles = mesh_a.triangles().len() + mesh_b.triangles().len();

    for (variant, bytes) in [
        (
            "ascii",
            stl::write_ascii(&meshes, "pair").unwrap().into_bytes(),
        ),
        ("binary", stl::write_binary(&meshes, "pair").unwrap()),
    ] {
        let reading = compare_stl(&bytes, &format!("multi-body-{variant}")).unwrap();
        assert_eq!(
            reading.triangles, total_triangles,
            "{variant}: triangle count"
        );
        let area_rel = (reading.area - (mesh_a.area() + mesh_b.area())).abs()
            / (mesh_a.area() + mesh_b.area());
        assert!(
            area_rel <= 1e-6,
            "{variant}: area {} vs the two meshes' summed {}, {area_rel:e} relative",
            reading.area,
            mesh_a.area() + mesh_b.area()
        );
    }
}

#[test]
fn obj_of_two_bodies_keeps_each_bodys_positions_and_offsets_the_second_bodys_indices() {
    let mut m = Model::default();
    let a = sample::cuboid(&mut m, Point3::origin(), Point3::new(3.0, 2.0, 1.0)).unwrap();
    let b = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let mesh_a = tessellate(&m, a, 1e-2).unwrap();
    let mesh_b = tessellate(&m, b, 1e-2).unwrap();
    let text = obj::write(&[mesh_a.clone(), mesh_b.clone()]).unwrap();
    let (positions, triangles) = parse_obj_bare(&text);

    assert_eq!(
        positions.len(),
        mesh_a.positions().len() + mesh_b.positions().len(),
        "position count is the sum"
    );
    assert_eq!(
        triangles.len(),
        mesh_a.triangles().len() + mesh_b.triangles().len(),
        "triangle count is the sum"
    );
    assert_eq!(
        &positions[..mesh_a.positions().len()],
        mesh_a.positions(),
        "the first body's own positions, unchanged"
    );
    assert_eq!(
        &positions[mesh_a.positions().len()..],
        mesh_b.positions(),
        "the second body's own positions, unchanged"
    );
    let offset = mesh_a.positions().len() as u32;
    for (parsed, own) in triangles[mesh_a.triangles().len()..]
        .iter()
        .zip(mesh_b.triangles())
    {
        assert_eq!(
            *parsed,
            [own[0] + offset, own[1] + offset, own[2] + offset],
            "the second body's triangles reference its own vertices, offset by the first's count"
        );
    }
}
