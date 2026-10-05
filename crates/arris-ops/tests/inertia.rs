//! `measure::mass_properties` against an independent integrator over
//! `tessellate`'s `TriMesh` (`docs/ARCHITECTURE.md` §Operations): volume,
//! centroid and the full tensor from the tetrahedra each triangle forms
//! with the origin, by the divergence theorem — which holds for any
//! origin, inside the body or not, so a random pose is no special case.
//! This proves the physical convention (`∫(|r|²I − r rᵀ)dV`, negated
//! products) against a second path, not only against the oracle.

use arris_debug::fixtures::Tolerances;
use arris_debug::prop_shards;
use arris_debug::unmetered::tessellate;
use arris_debug::unmetered::{fillet, mass_properties, primitive_box, transform};
use arris_debug::{prop, sample};
use arris_math::{Isometry, Matrix3, Point3, Vec3};
use arris_ops::measure::MassProperties;
use arris_topo::{Body, Edge, Model};
/// A named sample body, built fresh in the model it is asked for.
type Sample = (&'static str, fn(&mut Model) -> Body);

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

/// The volume, and the first and second moments about the origin, of the
/// tetrahedron `(0, a, b, c)`, signed by the triangle's own winding: the
/// standard result of mapping the unit simplex by `(a, b, c)`, `j = a ·
/// (b × c)` its Jacobian. Verified against the standard simplex `a = x̂,
/// b = ŷ, c = ẑ`, whose own moments are textbook (`∫x² = 1/60`, `∫xy =
/// 1/120`, `∫x = 1/24`, over `j = 1`).
fn tetra_moments(a: Vec3, b: Vec3, c: Vec3) -> (f64, Vec3, Matrix3) {
    let j = a.dot(&b.cross(&c));
    let volume = j / 6.0;
    let first = j / 24.0 * (a + b + c);
    let sum = a + b + c;
    let mut second = Matrix3::zeros();
    for i in 0..3 {
        for k in 0..3 {
            let diag = a[i] * a[k] + b[i] * b[k] + c[i] * c[k];
            second[(i, k)] = j / 120.0 * (diag + sum[i] * sum[k]);
        }
    }
    (volume, first, second)
}

/// `mesh`'s volume, centroid and inertia about its own centroid, by
/// summing [`tetra_moments`] over every triangle with the origin and
/// shifting the second moment from about the origin to about the
/// centroid — the parallel-axis theorem run backwards from
/// [`MassProperties::inertia_about`].
fn mesh_properties(mesh: &arris_mesh::TriMesh) -> MassProperties {
    let positions = mesh.positions();
    let (mut volume, mut first, mut second) = (0.0, Vec3::zeros(), Matrix3::zeros());
    for &t in mesh.triangles() {
        let [a, b, c] = t.map(|i| Point3::from(positions[i as usize]).coords);
        let (v, m1, m2) = tetra_moments(a, b, c);
        volume += v;
        first += m1;
        second += m2;
    }
    let centroid = Point3::from(first / volume);
    let about_origin = second.trace() * Matrix3::identity() - second;
    let c = centroid.coords;
    let inertia = about_origin - volume * (c.dot(&c) * Matrix3::identity() - c * c.transpose());
    MassProperties {
        volume,
        area: mesh.area(),
        centroid,
        inertia,
    }
}

/// `body`, moved by `motion`, measured exactly and by [`mesh_properties`]
/// of its tessellation at `tolerances.mesh_chord`: volume and centroid
/// agree within `tolerances.mesh_volume_rel`, the inertia tensor —
/// diagonal and off-diagonal alike — within the same bound of its own
/// scale.
fn assert_mesh_matches_measure(m: &mut Model, body: Body, motion: &Isometry, what: &str) {
    let tolerances = Tolerances::default();
    let (moved, _) = transform(m, body, motion).unwrap();
    let exact = mass_properties(m, moved).unwrap();
    let mesh = tessellate(m, moved, tolerances.mesh_chord).unwrap();
    let from_mesh = mesh_properties(&mesh);

    let rel = tolerances.mesh_volume_rel;
    let volume_scale = exact.volume.abs().max(1.0);
    assert!(
        (from_mesh.volume - exact.volume).abs() <= rel * volume_scale,
        "{what}: volume {} vs measure's {}",
        from_mesh.volume,
        exact.volume
    );
    let centroid_scale = exact.centroid.coords.abs().max().max(1.0);
    assert!(
        (from_mesh.centroid - exact.centroid).norm() <= rel * centroid_scale,
        "{what}: centroid {} vs measure's {}",
        from_mesh.centroid,
        exact.centroid
    );
    let inertia_scale = exact.inertia.abs().max().max(1.0);
    for i in 0..3 {
        for j in 0..3 {
            let (found, expected) = (from_mesh.inertia[(i, j)], exact.inertia[(i, j)]);
            assert!(
                (found - expected).abs() <= rel * inertia_scale,
                "{what}: inertia [{i}, {j}] {found} vs measure's {expected}"
            );
        }
    }
}

prop_shards! {
    /// Sharded (`arris_debug::prop::config`): tessellating five bodies at
    /// the corpus's own `mesh_chord` on every case is too slow for one
    /// nextest binary, so the configured case count runs split over
    /// eight, in parallel.
    mesh_integration_agrees_with_measure_over_the_sample_bodies_and_a_filleted_box
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (motion) = prop::pose() => {
            let bodies: [Sample; 5] = [
                ("cuboid", |m| {
                    sample::cuboid(m, Point3::origin(), Point3::new(3.0, 2.0, 1.0)).unwrap()
                }),
                ("cylinder", |m| sample::cylinder(m, 4.0, 12.0).unwrap()),
                ("sphere", |m| {
                    sample::sphere(m, Point3::new(1.0, -2.0, 0.5), 3.0).unwrap()
                }),
                ("torus", |m| sample::torus(m, Point3::origin(), 5.0, 2.0).unwrap()),
                ("filleted box", filleted_box),
            ];
            for (name, build) in bodies {
                let mut m = Model::default();
                let body = build(&mut m);
                assert_mesh_matches_measure(&mut m, body, &motion, name);
            }
            Ok(())
        }
}
