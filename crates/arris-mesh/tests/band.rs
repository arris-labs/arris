//! A torus face on its regular band (ADR-0080) tessellates like any other
//! face: the horn and spindle blends of the corpus, and the same blends
//! after a cut and a fuse, mesh closed with a positive volume, and every
//! triangle of the torus face lies off its axis and faces outward.

use arris_debug::unmetered::tessellate;
use arris_debug::{corpus, fixtures};
use arris_geom::Surface;
use arris_math::{Point3, Vec3};

const CHORD: f64 = 1e-3;

const BLENDS: &[&str] = &[
    "blend/tangent-chain-horn-torus",
    "blend/cylinder-rim-spindle-fillet",
    "blend/horn-blend-box-cut",
    "blend/spindle-blend-cylinder-fuse",
];

#[test]
fn the_blends_of_a_horn_and_a_spindle_torus_mesh_closed_with_normals_off_the_axis() {
    for name in BLENDS {
        let chain = corpus::chain(&fixtures::corpus_root().join(name), "default").unwrap();
        let body = chain.result().unwrap();
        let m = &chain.model;
        let mesh = tessellate(m, body, CHORD).unwrap();
        assert!(mesh.is_closed(), "{name}: the mesh is not closed");
        assert!(mesh.signed_volume().unwrap() > 0.0, "{name}: no volume");

        let mut tori = 0;
        for range in mesh.faces() {
            let face = m.face(range.face).unwrap();
            let Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } = *m.surface(face.surface()).unwrap()
            else {
                continue;
            };
            tori += 1;
            let axis = frame.z().into_inner();
            let origin = frame.origin();
            let radial = |p: Point3| -> Vec3 {
                let d = p - origin;
                d - axis * d.dot(&axis)
            };
            for i in range.triangles.clone() {
                let t = mesh.triangle_positions(i).unwrap();
                let [a, b, c] = t.map(|p| Point3::new(p[0], p[1], p[2]));
                for p in [a, b, c] {
                    // Off the axis by a fraction of the tube, on the band.
                    assert!(
                        radial(p).norm() > 1e-3 * minor_radius,
                        "{name}: a vertex of the torus face at {p} is on its axis"
                    );
                }
                let centroid = Point3::from((a.coords + b.coords + c.coords) / 3.0);
                let r = radial(centroid);
                let tube_centre =
                    origin + axis * (centroid - origin).dot(&axis) + r.normalize() * major_radius;
                let outward = centroid - tube_centre;
                let winding = (b - a).cross(&(c - a));
                assert!(
                    winding.dot(&outward) > 0.0,
                    "{name}: a triangle of the torus face faces inward"
                );
            }
        }
        assert!(tori >= 1, "{name}: no torus face to hold");
    }
}
