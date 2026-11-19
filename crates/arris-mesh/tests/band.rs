//! A torus face on its regular band (ADR-0080) tessellates like any other
//! face: the horn and spindle blends of the corpus, and the same blends
//! after a cut and a fuse, mesh closed with a positive volume, and every
//! triangle of the torus face lies off its axis and faces outward.
//! Revolve fixtures cover the outer sheet up to a torus pole (ADR-0092).

use arris_debug::unmetered::{tessellate, tessellate_with};
use arris_debug::{corpus, fixtures};
use arris_geom::Surface;
use arris_math::{Point3, Vec3, real};
use arris_mesh::MeshRequest;
use arris_topo::Orientation;
use arris_topo::entity::EdgeGeometry;

const CHORD: f64 = 1e-3;

const BLENDS: &[&str] = &[
    "blend/tangent-chain-horn-torus",
    "blend/cylinder-rim-spindle-fillet",
    "blend/horn-blend-box-cut",
    "blend/spindle-blend-cylinder-fuse",
];

/// How many boundary rows of each revolved torus face reach the axis.
const REVOLVES: &[(&str, usize)] = &[
    ("sweep/revolve-spindle-arc-from-axis", 1),
    ("sweep/revolve-spindle-arc-to-axis", 1),
    ("sweep/revolve-spindle-arc-clear-of-axis", 0),
    ("sweep/revolve-horn-arc-to-axis", 1),
    ("sweep/revolve-horn-torus-half-turn", 2),
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

#[test]
fn the_revolved_torus_fixtures_mesh_closed_to_their_volume_with_pole_normals() {
    for &(name, pole_rows) in REVOLVES {
        let dir = fixtures::corpus_root().join(name);
        let fixture = fixtures::load(&dir).unwrap();
        for variant in fixture.recipe.variant_names() {
            let label = format!("{name}/{variant}");
            let chain = corpus::chain(&dir, &variant).unwrap();
            let (body, _) = corpus::check_stage(&fixture, &chain).unwrap();
            let target =
                corpus::measure_stage(&fixture, &chain, &fixture.expected.results[&variant])
                    .unwrap();
            let tolerances = fixture.recipe.tolerances;
            let m = &chain.model;
            let request = MeshRequest::new(tolerances.mesh_chord).with_corners();
            let mesh = tessellate_with(m, body, &request).unwrap();
            let volume = mesh
                .signed_volume()
                .unwrap_or_else(|| panic!("{label}: the mesh is not closed"));
            let expected = target.measured.volume.unwrap();
            assert!(
                (volume - expected).abs() <= expected * tolerances.mesh_volume_rel,
                "{label}: mesh volume {volume} differs from {} {expected}",
                target.by
            );

            let corners = mesh.corners().unwrap();
            let mut tori = 0;
            let mut poles = 0;
            for face_use in m.faces(body).unwrap() {
                let face = m.face(face_use.id).unwrap();
                let Surface::Torus { frame, .. } = m.surface(face.surface()).unwrap() else {
                    continue;
                };
                tori += 1;
                let range = mesh.faces().iter().find(|r| r.face == face_use.id).unwrap();
                let mut previous_pole = None;
                for coedge in face.loops().iter().flat_map(|l| l.coedges()) {
                    let edge = m.edge(coedge.edge()).unwrap();
                    if !edge.is_degenerate() {
                        continue;
                    }
                    poles += 1;
                    let &[pole] = mesh.edge_polyline(coedge.edge()).unwrap() else {
                        panic!("{label}: the pole must have one shared index");
                    };
                    if pole_rows == 2 {
                        assert_eq!(
                            pole,
                            *previous_pole.get_or_insert(pole),
                            "{label}: the horn rows split the pole"
                        );
                    }
                    let EdgeGeometry::Degenerate { range: edge_range } = edge.geometry() else {
                        unreachable!();
                    };
                    let row = m.curve2(coedge.pcurve()).unwrap().point(edge_range.lo()).y;
                    let fan_corners = corners.triangles()[range.triangles.clone()]
                        .iter()
                        .flatten()
                        .map(|&i| i as usize)
                        .filter(|&i| {
                            corners.positions()[i] == pole
                                && (corners.uvs()[i][1] - row).abs()
                                    <= m.precision().parametric_tolerance
                        });
                    let mut fan_triangles = 0;
                    for i in fan_corners {
                        fan_triangles += 1;
                        let [u, v] = corners.uvs()[i];
                        let (su, cu) = real::sin_cos(u);
                        let (sv, cv) = real::sin_cos(v);
                        let limit = cv
                            * (cu * frame.x().into_inner() + su * frame.y().into_inner())
                            + sv * frame.z().into_inner();
                        let expected = if face_use.orientation == Orientation::Reversed {
                            -limit
                        } else {
                            limit
                        };
                        let normal = Vec3::from(corners.normals()[i]);
                        assert!(
                            (normal - expected).norm() < 1e-12,
                            "{label}: at ({u}, {v}) the pole normal is {normal}, expected {expected}"
                        );
                    }
                    assert!(fan_triangles > 1, "{label}: the pole fan is missing");
                }
            }
            assert_eq!(tori, 1, "{label}: expected one torus face");
            assert_eq!(poles, pole_rows, "{label}: wrong number of pole rows");
        }
    }
}
