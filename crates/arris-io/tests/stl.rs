//! STL against the Open CASCADE oracle (`docs/ARCHITECTURE.md` §Formats
//! and tools): `RWStl`'s reading of Arris's ASCII and binary bytes agrees
//! with the mesh's own triangle count and area, and its signed volume is
//! within the mesh's own bound of `measure::mass_properties`'s, over the
//! cube, the cylinder (a seam), the sphere (poles), the torus (genus 1)
//! and a filleted box; two writes of one mesh are byte-identical.

use arris_debug::fixtures::Tolerances;
use arris_debug::oracle::compare_stl;
use arris_debug::sample;
use arris_debug::unmetered::{fillet, primitive_box};
use arris_io::arris_mesh::tessellate;
use arris_io::stl;
use arris_ops::arris_check::arris_topo::arris_math::Point3;
use arris_ops::arris_check::arris_topo::{Body, Edge, Model};
fn filleted_box(m: &mut Model) -> Body {
    let (body, _) = primitive_box(m, Point3::origin(), Point3::new(2.0, 2.0, 2.0)).unwrap();
    let edge: Edge = m
        .edges(body)
        .unwrap()
        .into_iter()
        .find(|e| {
            let entity = m.edge(e.id).unwrap();
            entity.curve().is_some_and(|(curve, range)| {
                (m.curve(curve).unwrap().point(range.midpoint()) - Point3::new(2.0, 2.0, 1.0))
                    .norm()
                    < 1e-9
            })
        })
        .expect("the vertical edge at (2, 2)");
    fillet(m, body, &[edge], 0.3).unwrap().0
}

/// `RWStl`'s reading of `mesh` written both ways agrees with the mesh's
/// own triangle count and area exactly, and its volume with
/// `measure::mass_properties`'s within the corpus's own `mesh_volume_rel`
/// at `mesh_chord` — the mesh's own bound, not a fixture's.
fn assert_stl_matches_the_oracle(m: &Model, body: Body, name: &str) {
    let tolerances = Tolerances::default();
    let mesh = tessellate(m, body, tolerances.mesh_chord).unwrap();
    let mass = arris_debug::unmetered::mass_properties(m, body).unwrap();

    let one = std::slice::from_ref(&mesh);
    for (variant, bytes) in [
        ("ascii", stl::write_ascii(one, name).unwrap().into_bytes()),
        ("binary", stl::write_binary(one, name).unwrap()),
    ] {
        let reading = compare_stl(&bytes, &format!("{name}-{variant}")).unwrap();
        assert_eq!(
            reading.triangles,
            mesh.triangles().len(),
            "{name} {variant}: triangle count"
        );
        let area_rel = (reading.area - mesh.area()).abs() / mesh.area();
        assert!(
            area_rel <= 1e-6,
            "{name} {variant}: area {} vs the mesh's {}, {area_rel:e} relative",
            reading.area,
            mesh.area()
        );
        let volume_rel = (reading.volume - mass.volume).abs() / mass.volume;
        assert!(
            volume_rel <= tolerances.mesh_volume_rel,
            "{name} {variant}: volume {} vs measure's {}, {volume_rel:e} relative, above mesh_volume_rel {:e}",
            reading.volume,
            mass.volume,
            tolerances.mesh_volume_rel
        );
    }
}

#[test]
fn ascii_and_binary_stl_of_every_sample_match_the_oracle() {
    let mut m = Model::default();
    let bodies = [
        (
            "stl-cuboid",
            sample::cuboid(&mut m, Point3::origin(), Point3::new(3.0, 2.0, 1.0)).unwrap(),
        ),
        ("stl-cylinder", sample::cylinder(&mut m, 4.0, 12.0).unwrap()),
        (
            "stl-sphere",
            sample::sphere(&mut m, Point3::new(1.0, -2.0, 0.5), 3.0).unwrap(),
        ),
        (
            "stl-torus",
            sample::torus(&mut m, Point3::origin(), 5.0, 2.0).unwrap(),
        ),
        ("stl-filleted-box", filleted_box(&mut m)),
    ];
    for (name, body) in bodies {
        assert_stl_matches_the_oracle(&m, body, name);
    }
}

#[test]
fn two_writes_are_byte_identical() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let mesh = tessellate(&m, body, 1e-2).unwrap();
    let one = std::slice::from_ref(&mesh);
    assert_eq!(
        stl::write_ascii(one, "cylinder").unwrap(),
        stl::write_ascii(one, "cylinder").unwrap()
    );
    assert_eq!(
        stl::write_binary(one, "cylinder").unwrap(),
        stl::write_binary(one, "cylinder").unwrap()
    );
}
