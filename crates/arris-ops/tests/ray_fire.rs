//! `query::ray_fire` (ADR-0053): the hits of a ray on a body, each naming
//! the most specific entity it lands on, and the refusals, each naming
//! the entity. The oracle's comparison is the `query/` corpus.

use arris_debug::unmetered::{primitive_box, primitive_cylinder};
use arris_math::{Axis, Point3, Vec3};
use arris_ops::query::{Ray, RayHits, ray_fire};
use arris_ops::{InputReason, OpError, QueryReason, Reason};
use arris_topo::{Body, EntityId, Model};

const EXACT: f64 = 1e-12;

/// The box `[0, 4] × [0, 3] × [0, 2]`.
fn cuboid() -> (Model, Body) {
    let mut m = Model::default();
    let (body, _) = primitive_box(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0)).unwrap();
    (m, body)
}

fn ray(origin: [f64; 3], direction: [f64; 3]) -> Ray {
    Ray {
        origin: Point3::new(origin[0], origin[1], origin[2]),
        direction: Vec3::new(direction[0], direction[1], direction[2]),
    }
}

fn refusal(error: OpError) -> (EntityId, QueryReason) {
    match error {
        OpError::Degenerate {
            entities,
            reason: Reason::Query(reason),
        } => (entities[0].id, reason),
        other => panic!("{other}"),
    }
}

#[test]
fn a_ray_through_a_box_enters_and_leaves_by_faces_with_outward_normals() {
    let (m, body) = cuboid();
    let hits = ray_fire(
        &m,
        body,
        &ray([2.0, 1.5, -1.0], [0.0, 0.0, 5.0]),
        RayHits::All,
    )
    .unwrap();
    assert_eq!(hits.len(), 2);
    assert!((hits[0].t - 1.0).abs() < EXACT && (hits[1].t - 3.0).abs() < EXACT);
    assert!(hits.iter().all(|h| matches!(h.shape.id, EntityId::Face(_))));
    assert!((hits[0].normal.z + 1.0).abs() < EXACT);
    assert!((hits[1].normal.z - 1.0).abs() < EXACT);
    assert!((hits[1].point - Point3::new(2.0, 1.5, 2.0)).norm() < EXACT);
    assert!(hits.iter().all(|h| !h.tangent));

    let first = ray_fire(
        &m,
        body,
        &ray([2.0, 1.5, -1.0], [0.0, 0.0, 5.0]),
        RayHits::First,
    )
    .unwrap();
    assert_eq!(first, hits[..1]);
}

#[test]
fn a_ray_is_a_half_line_and_one_from_inside_leaves_by_what_it_meets() {
    let (m, body) = cuboid();
    // Behind the origin there is a face, which is not a hit.
    let away = ray_fire(
        &m,
        body,
        &ray([2.0, 1.5, -1.0], [0.0, 0.0, -1.0]),
        RayHits::All,
    )
    .unwrap();
    assert!(away.is_empty());
    let inside = ray_fire(
        &m,
        body,
        &ray([2.0, 1.5, 1.0], [0.0, 0.0, 1.0]),
        RayHits::All,
    )
    .unwrap();
    assert_eq!(inside.len(), 1);
    assert!((inside[0].t - 1.0).abs() < EXACT && inside[0].normal.z > 0.5);
    let beside = ray_fire(
        &m,
        body,
        &ray([9.0, 1.5, -1.0], [0.0, 0.0, 1.0]),
        RayHits::All,
    )
    .unwrap();
    assert!(beside.is_empty());
}

#[test]
fn a_ray_from_a_point_on_a_face_hits_it_at_zero() {
    let (m, body) = cuboid();
    let hits = ray_fire(
        &m,
        body,
        &ray([2.0, 1.5, 2.0], [0.0, 0.0, 1.0]),
        RayHits::All,
    )
    .unwrap();
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].t, 0.0);
    assert!(matches!(hits[0].shape.id, EntityId::Face(_)));
}

#[test]
fn a_ray_through_an_edge_or_a_vertex_is_one_hit_naming_it() {
    let (m, body) = cuboid();
    // Diagonally through the edge x = 4, y = 3, at height 1.
    let edge = ray_fire(
        &m,
        body,
        &ray([5.0, 4.0, 1.0], [-1.0, -1.0, 0.0]),
        RayHits::All,
    )
    .unwrap();
    assert_eq!(
        edge.len(),
        2,
        "in at the edge, out at the far one: {edge:?}"
    );
    assert!(matches!(edge[0].shape.id, EntityId::Edge(_)));
    assert!(
        (edge[0].t - 2f64.sqrt()).abs() < EXACT,
        "the distance along the unit direction"
    );
    // Through the corner (4, 3, 2).
    let corner = ray_fire(
        &m,
        body,
        &ray([5.0, 4.0, 3.0], [-1.0, -1.0, -1.0]),
        RayHits::First,
    )
    .unwrap();
    assert_eq!(corner.len(), 1);
    assert!(matches!(corner[0].shape.id, EntityId::Vertex(_)));
    assert!((corner[0].point - Point3::new(4.0, 3.0, 2.0)).norm() < EXACT);
    // The same edge hit twice is the same hit, in the same order.
    let again = ray_fire(
        &m,
        body,
        &ray([5.0, 4.0, 1.0], [-1.0, -1.0, 0.0]),
        RayHits::All,
    )
    .unwrap();
    assert_eq!(again, edge);
}

#[test]
fn a_ray_in_a_face_is_refused_naming_the_face_and_one_beside_it_misses() {
    let (m, body) = cuboid();
    // In the plane of the top face, across it.
    let (entity, reason) = refusal(
        ray_fire(
            &m,
            body,
            &ray([-1.0, 1.0, 2.0], [1.0, 0.0, 0.0]),
            RayHits::All,
        )
        .unwrap_err(),
    );
    assert!(matches!(entity, EntityId::Face(_)));
    assert_eq!(reason, QueryReason::RayInFace);
    // In the same plane, past the face.
    let past = ray_fire(
        &m,
        body,
        &ray([-1.0, 5.0, 2.0], [1.0, 0.0, 0.0]),
        RayHits::All,
    )
    .unwrap();
    assert!(past.is_empty());
}

#[test]
fn a_ray_with_no_direction_or_no_finite_number_is_refused_naming_the_body() {
    let (m, body) = cuboid();
    let (entity, reason) =
        refusal(ray_fire(&m, body, &ray([0.0, 0.0, 0.0], [0.0; 3]), RayHits::All).unwrap_err());
    assert_eq!(
        (entity, reason),
        (body.id.into(), QueryReason::ZeroDirection)
    );
    for bad in [
        ray([f64::NAN, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ray([0.0, 0.0, 0.0], [f64::INFINITY, 0.0, 0.0]),
    ] {
        assert!(matches!(
            ray_fire(&m, body, &bad, RayHits::All).unwrap_err(),
            OpError::Degenerate {
                reason: Reason::Input(InputReason::NonFinite { .. }),
                ..
            }
        ));
    }
}

#[test]
fn a_ray_grazing_a_cylinder_is_one_tangent_hit_and_one_along_its_wall_is_in_the_face() {
    let mut m = Model::default();
    let (body, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 2.0, 5.0).unwrap();
    // Tangent to the wall at (0, 2, 2.5), away from the seam.
    let graze = ray_fire(
        &m,
        body,
        &ray([-3.0, 2.0, 2.5], [1.0, 0.0, 0.0]),
        RayHits::All,
    )
    .unwrap();
    assert_eq!(graze.len(), 1, "{graze:?}");
    assert!(graze[0].tangent);
    assert!((graze[0].t - 3.0).abs() < 1e-9);
    assert!((graze[0].normal.y - 1.0).abs() < 1e-9);
    // Across the axis it is a crossing.
    let across = ray_fire(
        &m,
        body,
        &ray([-3.0, 0.0, 2.5], [1.0, 0.0, 0.0]),
        RayHits::All,
    )
    .unwrap();
    assert_eq!(across.len(), 2);
    assert!(across.iter().all(|h| !h.tangent));
    assert!((across[0].normal.x + 1.0).abs() < 1e-9 && (across[1].normal.x - 1.0).abs() < 1e-9);
    // Along a ruling away from the seam.
    let (entity, reason) = refusal(
        ray_fire(
            &m,
            body,
            &ray([0.0, 2.0, -1.0], [0.0, 0.0, 1.0]),
            RayHits::All,
        )
        .unwrap_err(),
    );
    assert!(matches!(entity, EntityId::Face(_)));
    assert_eq!(reason, QueryReason::RayInFace);
}
