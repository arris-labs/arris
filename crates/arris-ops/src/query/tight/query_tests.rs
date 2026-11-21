use core::f64::consts::TAU;

use arris_check::{Level, check, domain::FaceDomain};
use arris_debug::sample;
use arris_geom::{NurbsSurface, Profile, ProfileLoop, ProfileSegment, Surface};
use arris_math::{Aabb, Axis, Frame, Interval, Point2, Point3, Stop, Vec2, Vec3};
use arris_topo::entity::{Body as BodyEntity, BodyKind, Face, Vertex};
use arris_topo::{Body, Model};

use crate::query::tight_bounds;
use crate::{Control, OpError, extrude, primitive_box, primitive_cylinder, revolve};

fn assert_bounds(m: &Model, body: Body, min: [f64; 3], max: [f64; 3]) {
    let report = check(m, body, Level::Full);
    assert!(report.is_ok(), "{report:?}");
    let mut shell_box = None;
    let mut outer = None;
    for shell in m.shells(body).unwrap() {
        let mut faces_box = None;
        for face in m.shell(shell.id).unwrap().faces() {
            let b = tight_bounds(m, face.id.into(), &Control::NONE)
                .unwrap()
                .unwrap();
            let domain = FaceDomain::of(m, face.id, m.precision().parametric_tolerance).unwrap();
            let cheap = domain.bounds().unwrap();
            assert_eq!(cheap.union(b), cheap, "{face:?}");
            super::include_box(&mut faces_box, b);
            super::include_box(&mut outer, cheap);
        }
        let b = tight_bounds(m, shell.id.into(), &Control::NONE).unwrap();
        assert_eq!(b, faces_box);
        super::include_box(&mut shell_box, b.unwrap());
    }
    let b = tight_bounds(m, body.id.into(), &Control::NONE)
        .unwrap()
        .unwrap();
    let cheap = outer.unwrap();
    assert_eq!(cheap.union(b), cheap);
    let eps = m.precision().default_tolerance;
    for k in 0..3 {
        assert!((b.min[k] - min[k]).abs() <= eps, "{b:?}");
        assert!((b.max[k] - max[k]).abs() <= eps, "{b:?}");
        let from_shells = shell_box.unwrap();
        assert!((b.min[k] - from_shells.min[k]).abs() <= eps, "{b:?}");
        assert!((b.max[k] - from_shells.max[k]).abs() <= eps, "{b:?}");
    }
}

#[test]
fn tight_bounds_analytic_primitives_match_their_extents() {
    let mut m = Model::default();
    let origin = Point3::origin();
    let axis = Axis::z_at(origin);
    let control = &Control::NONE;
    let (block, _) = primitive_box(&mut m, origin, Point3::new(1.0, 2.0, 3.0), control).unwrap();
    assert_bounds(&m, block, [0.0; 3], [1.0, 2.0, 3.0]);
    let (cylinder, _) = primitive_cylinder(&mut m, axis, 2.0, 3.0, control).unwrap();
    assert_bounds(&m, cylinder, [-2.0, -2.0, 0.0], [2.0, 2.0, 3.0]);
    let sphere = sample::sphere(&mut m, origin, 2.0).unwrap();
    assert_bounds(&m, sphere, [-2.0; 3], [2.0; 3]);
    let torus = sample::torus(&mut m, origin, 3.0, 1.0).unwrap();
    assert_bounds(&m, torus, [-4.0, -4.0, -1.0], [4.0, 4.0, 1.0]);
    let triangle = Profile {
        plane: Frame::from_orthonormal(origin, Vec3::x(), Vec3::z(), -Vec3::y()).unwrap(),
        outer: ProfileLoop::Path {
            start: Point2::origin(),
            segments: vec![
                ProfileSegment::LineTo(Point2::new(2.0, 0.0)),
                ProfileSegment::LineTo(Point2::new(0.0, 3.0)),
                ProfileSegment::LineTo(Point2::origin()),
            ],
        },
        holes: vec![],
    };
    let (cone, _) = revolve(&mut m, &triangle, axis, TAU, control).unwrap();
    assert_bounds(&m, cone, [-2.0, -2.0, 0.0], [2.0, 2.0, 3.0]);
    let ellipse = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Ellipse {
            center: Point2::origin(),
            major: Vec2::new(2.0, 0.0),
            minor_radius: 1.0,
        },
        holes: vec![],
    };
    let (elliptic, _) = extrude(&mut m, &ellipse, Vec3::z(), 3.0, control).unwrap();
    assert_bounds(&m, elliptic, [-2.0, -1.0, 0.0], [2.0, 1.0, 3.0]);
}

#[test]
fn tight_bounds_nurbs_keeps_an_interior_peak_and_spends_patch_budget() {
    let mut m = Model::default();
    // A quadratic patch with z = 16u(1-u)v(1-v) and z = 0 on its edges.
    let knots = [0., 0., 0., 1., 1., 1.].to_vec();
    let points = (0..3)
        .flat_map(|i| {
            (0..3).map(move |j| {
                let z = if i == 1 && j == 1 { 4.0 } else { 0.0 };
                Point3::new(i as f64 / 2.0, j as f64 / 2.0, z)
            })
        })
        .collect();
    let surface = NurbsSurface::new([2, 2], [knots.clone(), knots], points, vec![1.0; 9]).unwrap();
    let body = sample::patch(
        &mut m,
        Surface::Nurbs(surface),
        Interval::UNIT,
        Interval::UNIT,
    )
    .unwrap();
    assert_bounds(&m, body, [0.0; 3], [1.0; 3]);

    let closure = m.closure(body).unwrap();
    let steps = 1
        + closure.shells.len()
        + closure.faces.len()
        + closure.edges.len()
        + closure.vertices.len();
    assert_eq!(
        tight_bounds(&m, body.id.into(), &Control::budget(steps as u64)),
        Err(OpError::Interrupted(arris_math::Interrupted {
            by: Stop::Budget,
            steps: steps as u64
        }))
    );
}

#[test]
fn tight_bounds_vertex_edge_unbounded_face_and_query_errors() {
    let mut m = Model::default();
    let (body, _) = primitive_box(
        &mut m,
        Point3::origin(),
        Point3::new(1.0, 2.0, 3.0),
        &Control::NONE,
    )
    .unwrap();
    let edge = m.edges(body).unwrap()[0].id;
    let edge_entity = m.edge(edge).unwrap();
    let start = edge_entity.start();
    let a = Aabb::of_point(m.vertex(start).unwrap().point());
    let b = Aabb::of_point(m.vertex(edge_entity.end()).unwrap().point());
    assert_eq!(
        tight_bounds(&m, start.into(), &Control::NONE).unwrap(),
        Some(a)
    );
    assert_eq!(
        tight_bounds(&m, edge.into(), &Control::NONE).unwrap(),
        Some(a.union(b))
    );
    let extra = Aabb::of_point(Point3::new(-3.0, 4.0, 5.0));
    let tol = m.precision().default_tolerance;
    let vertex = m
        .raw()
        .add_vertex(Vertex::new(Point3::from(extra.min), tol));
    let general = m.raw().add_body(BodyEntity::new(
        BodyKind::General,
        vec![],
        vec![arris_topo::Edge::forward(edge)],
        vec![vertex],
    ));
    assert_eq!(
        tight_bounds(&m, general.into(), &Control::NONE).unwrap(),
        Some(a.union(b).union(extra))
    );
    assert_eq!(
        tight_bounds(&Model::default(), edge.into(), &Control::NONE),
        Err(OpError::NotFound(edge.into()))
    );
    assert_eq!(
        tight_bounds(&m, edge.into(), &Control::poll(&|| true)),
        Err(OpError::Interrupted(arris_math::Interrupted {
            by: Stop::Poll,
            steps: 0
        }))
    );

    // A loopless raw plane is unbounded.
    let plane = m.add_surface(Surface::Plane {
        frame: Frame::world(),
    });
    let face = m.raw().add_face(Face::new(plane, vec![], tol));
    assert_eq!(tight_bounds(&m, face.into(), &Control::NONE).unwrap(), None);
}
