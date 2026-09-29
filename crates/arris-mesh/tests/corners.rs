//! The corner block (ADR-0012): the render buffer beside the watertight
//! one. Every face-local vertex of every sample body — and of a filleted
//! box — evaluates back to the shared position it stands on and carries
//! the outward normal there; a seam's two copies differ by exactly one
//! period in `u`; a sphere's pole corners carry the axis and a cone's
//! apex corners the cone's own ring of normals; the watertight buffer is
//! the one `tessellate` gives, corners or not; and a block that does not
//! fit its mesh is a typed refusal.

use arris_debug::unmetered::{fillet, primitive_box, revolve};
use core::f64::consts::{PI, TAU};

use arris_debug::sample;
use arris_debug::unmetered::{tessellate, tessellate_with};
use arris_mesh::{CornerFace, Corners, MeshError, MeshRequest, TriMesh};
use arris_ops::arris_check::arris_topo::arris_geom::{
    Profile, ProfileLoop, ProfileSegment, Surface,
};

use arris_ops::arris_check::arris_topo::arris_math::{Axis, Frame, Interval, Point2, Point3, Vec3};
use arris_ops::arris_check::arris_topo::{Body, Edge, Model, Orientation};
/// Rounding at the scale of a coordinate.
const EXACT: f64 = 1e-12;

fn corners_of(m: &Model, body: Body, chord: f64) -> (TriMesh, Corners) {
    let mesh = tessellate_with(m, body, &MeshRequest::new(chord).with_corners()).unwrap();
    let corners = mesh.corners().expect("corners were asked for").clone();
    (mesh, corners)
}

/// ADR-0012's two invariants on every face-local vertex of a body, and
/// the structure that ties the block to the mesh it stands beside:
///
/// - the face's surface at the corner's own (u, v) is the shared
///   position it names, within the face's tolerance — the pcurves are
///   same-parameter, so a loop sample lands on the surface too;
/// - its normal is unit, perpendicular to both surface tangents, and is
///   the surface's own normal in the face use's sense wherever the
///   parametrisation is not singular;
/// - it points out of the material: it agrees in sense with the winding
///   of every triangle of its own face that uses it.
fn assert_corners(m: &Model, body: Body, mesh: &TriMesh, corners: &Corners) {
    let faces = m.faces(body).unwrap();
    assert_eq!(
        corners.faces().iter().map(|f| f.face).collect::<Vec<_>>(),
        faces.iter().map(|f| f.id).collect::<Vec<_>>(),
        "one CornerFace per face, in iteration order"
    );
    assert_eq!(corners.triangles().len(), mesh.triangles().len());
    let positions = mesh.positions();
    for (used, cf) in faces.iter().zip(corners.faces()) {
        let face = m.face(used.id).unwrap();
        let surface = m.surface(face.surface()).unwrap();
        let tol = face.tolerance();
        let reversed = used.orientation == Orientation::Reversed;
        assert!(
            !cf.vertices.is_empty(),
            "{} has no face-local vertex",
            cf.face
        );
        for i in cf.vertices.clone() {
            let [u, v] = corners.uvs()[i];
            let shared = corners.positions()[i] as usize;
            let at = Point3::from(positions[shared]);
            // The surface at the corner's own (u, v) is the position.
            let on_surface = surface.point(u, v);
            assert!(
                (on_surface - at).norm() <= tol,
                "{}: ({u}, {v}) evaluates to {on_surface}, not the {at} it stands on",
                cf.face
            );
            assert!(cf.uv_box[0].contains(u) && cf.uv_box[1].contains(v));

            // The normal: unit, in the tangent plane, and the surface's
            // own by the face use wherever there is one.
            let n = Vec3::from(corners.normals()[i]);
            assert!(
                (n.norm() - 1.0).abs() <= EXACT,
                "{}: |n| = {}",
                cf.face,
                n.norm()
            );
            let e = surface.eval(u, v);
            for d in [e.du, e.dv] {
                assert!(
                    n.dot(&d).abs() <= EXACT * d.norm().max(1.0),
                    "{}: the normal at ({u}, {v}) leaves the tangent plane",
                    cf.face
                );
            }
            if let Some(own) = surface.normal(u, v) {
                let outward = if reversed {
                    -own.into_inner()
                } else {
                    own.into_inner()
                };
                assert!(
                    (n - outward).norm() <= EXACT,
                    "{}: the normal at ({u}, {v}) is {n}, not the outward {outward}",
                    cf.face
                );
            }
        }
        // Outward: every triangle of the face is counter-clockwise seen
        // from outside, so its winding agrees with its corners' normals.
        let range = mesh
            .faces()
            .iter()
            .find(|f| f.face == cf.face)
            .unwrap()
            .triangles
            .clone();
        for t in range {
            let local = corners.triangles()[t];
            let [a, b, c] = mesh.triangle_positions(t).unwrap().map(Point3::from);
            let winding = (b - a).cross(&(c - a));
            if winding.norm() <= EXACT * (b - a).norm().max((c - a).norm()) {
                continue; // A sliver's plane says nothing about a direction.
            }
            let winding = winding.normalize();
            for i in local {
                let i = i as usize;
                assert!(
                    cf.vertices.contains(&i),
                    "{}: triangle {t} uses a face-local vertex of another face",
                    cf.face
                );
                assert_eq!(
                    corners.positions()[i],
                    mesh.triangles()[t][local.iter().position(|&j| j as usize == i).unwrap()],
                    "{}: triangle {t}'s corner stands on another mesh vertex",
                    cf.face
                );
                let n = Vec3::from(corners.normals()[i]);
                assert!(
                    n.dot(&winding) > 0.0,
                    "{}: triangle {t}'s winding faces {winding}, its corner normal {n}",
                    cf.face
                );
            }
        }
    }
}

/// A cone: a right triangle with one leg on the axis, turned a full
/// turn, so the apex is a degenerate edge (`docs/DATA-MODEL.md` E6).
fn cone(m: &mut Model, radius: f64, height: f64) -> Body {
    let at = |u: f64, v: f64| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::from_orthonormal(Point3::origin(), Vec3::x(), Vec3::z(), -Vec3::y()).unwrap(),
        outer: ProfileLoop::Path {
            start: at(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(at(radius, 0.0)),
                ProfileSegment::LineTo(at(0.0, height)),
                ProfileSegment::LineTo(at(0.0, 0.0)),
            ],
        },
        holes: vec![],
    };
    revolve(m, &profile, Axis::z_at(Point3::origin()), TAU)
        .unwrap()
        .0
}

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

/// Every sample body, a NURBS patch and a filleted box: the block holds
/// ADR-0012's invariants on every one of their face-local vertices, and
/// the watertight buffer is the one `tessellate` gives either way.
#[test]
fn every_face_local_vertex_stands_on_its_surface_and_faces_out() {
    let mut m = Model::default();
    let bodies = [
        (
            "cuboid",
            sample::cuboid(&mut m, Point3::origin(), Point3::new(3.0, 2.0, 1.0)).unwrap(),
        ),
        ("cylinder", sample::cylinder(&mut m, 4.0, 12.0).unwrap()),
        (
            "sphere",
            sample::sphere(&mut m, Point3::new(1.0, -2.0, 0.5), 3.0).unwrap(),
        ),
        (
            "torus",
            sample::torus(&mut m, Point3::origin(), 5.0, 2.0).unwrap(),
        ),
        (
            "patch",
            sample::patch(
                &mut m,
                Surface::Sphere {
                    frame: Frame::world(),
                    radius: 2.0,
                },
                Interval::new(0.2, 2.0).unwrap(),
                Interval::new(-0.5, 0.9).unwrap(),
            )
            .unwrap(),
        ),
        ("cone", cone(&mut m, 1.0, 2.0)),
        ("filleted box", filleted_box(&mut m)),
    ];
    for (name, body) in bodies {
        let chord = 1e-2;
        let (mesh, corners) = corners_of(&m, body, chord);
        assert_corners(&m, body, &mesh, &corners);

        // The watertight buffer is untouched by the request.
        let plain = tessellate(&m, body, chord).unwrap();
        assert!(plain.corners().is_none(), "{name}");
        assert_eq!(mesh.positions(), plain.positions(), "{name}: positions");
        assert_eq!(mesh.triangles(), plain.triangles(), "{name}: triangles");
        assert_eq!(mesh.faces(), plain.faces(), "{name}: face ranges");
        assert_eq!(mesh.edge_indices(), plain.edge_indices(), "{name}: edges");
        assert_eq!(mesh.edges(), plain.edges(), "{name}: edge ranges");
        assert_eq!(mesh.signed_volume(), plain.signed_volume(), "{name}");

        // Deterministic: the same block twice.
        let (_, again) = corners_of(&m, body, chord);
        assert_eq!(corners, again, "{name}: two runs differ");
    }
}

/// A seam is one index run used twice by the same face: the two copies
/// are two face-local vertices on the same shared position whose `u` —
/// or, on a torus's second seam, whose `v` — differs by exactly one
/// period, which is what a welded buffer cannot carry (ADR-0012). A
/// pole's fan shares a position too, but the surface is singular there
/// and its copies differ by no period at all.
#[test]
fn a_seams_two_copies_differ_by_one_period() {
    for (name, build) in [
        (
            "cylinder",
            Box::new(|m: &mut Model| sample::cylinder(m, 4.0, 12.0).unwrap())
                as Box<dyn Fn(&mut Model) -> Body>,
        ),
        (
            "sphere",
            Box::new(|m: &mut Model| sample::sphere(m, Point3::origin(), 3.0).unwrap()),
        ),
        (
            "torus",
            Box::new(|m: &mut Model| sample::torus(m, Point3::origin(), 5.0, 2.0).unwrap()),
        ),
    ] {
        let mut m = Model::default();
        let body = build(&mut m);
        let (_, corners) = corners_of(&m, body, 1e-2);
        let mut seen = 0;
        for cf in corners.faces() {
            let face = m.face(cf.face).unwrap();
            let surface = m.surface(face.surface()).unwrap();
            let periods = surface.period();
            // The face-local vertices of this face, by the shared
            // position they stand on.
            for i in cf.vertices.clone() {
                for j in cf.vertices.clone().filter(|&j| j > i) {
                    if corners.positions()[i] != corners.positions()[j] {
                        continue;
                    }
                    let (a, b) = (corners.uvs()[i], corners.uvs()[j]);
                    // A pole's fan shares a position where the surface
                    // is singular; it is not a seam and owes no period.
                    if surface.normal(a[0], a[1]).is_none() {
                        continue;
                    }
                    let mut apart = 0;
                    for dir in 0..2 {
                        let d = (a[dir] - b[dir]).abs();
                        if d <= EXACT {
                            continue;
                        }
                        let period = periods[dir].unwrap_or_else(|| {
                            panic!("{name}: two copies {d} apart in a parameter with no period")
                        });
                        assert!(
                            (d - period).abs() <= EXACT,
                            "{name}: the seam's copies differ by {d}, not the period {period}"
                        );
                        apart += 1;
                    }
                    assert!(apart > 0, "{name}: two copies at the same (u, v)");
                    seen += 1;
                }
            }
        }
        assert!(seen > 0, "{name}: no seam copy found");
    }
}

/// A sphere's pole is one shared position under a whole fan: every
/// corner of the fan carries the axis, outward, whatever its own `u`
/// (ADR-0012).
#[test]
fn a_spheres_pole_corners_carry_the_axis() {
    let mut m = Model::default();
    let radius = 3.0;
    let body = sample::sphere(&mut m, Point3::origin(), radius).unwrap();
    let (_, corners) = corners_of(&m, body, 1e-2);
    let mut poles = [0usize; 2];
    for cf in corners.faces() {
        for i in cf.vertices.clone() {
            let [_, v] = corners.uvs()[i];
            let n = Vec3::from(corners.normals()[i]);
            let axis = if (v - PI / 2.0).abs() <= EXACT {
                poles[1] += 1;
                Vec3::z()
            } else if (v + PI / 2.0).abs() <= EXACT {
                poles[0] += 1;
                -Vec3::z()
            } else {
                continue;
            };
            assert!(
                (n - axis).norm() <= EXACT,
                "a pole corner at v = {v} carries {n}, not the axis {axis}"
            );
        }
    }
    assert!(
        poles[0] > 1 && poles[1] > 1,
        "each pole is a fan of corners, not one: {poles:?}"
    );
}

/// A cone's apex is one shared position under a fan whose corners each
/// want their own normal: the cone's own, at the corner's own `u`, so
/// the tip shades instead of going black (ADR-0012).
#[test]
fn a_cones_apex_corners_carry_its_ring() {
    let mut m = Model::default();
    let (radius, height) = (1.0, 2.0);
    let body = cone(&mut m, radius, height);
    let (_, corners) = corners_of(&m, body, 1e-2);
    let mut seen = 0;
    let mut directions: Vec<Vec3> = Vec::new();
    for cf in corners.faces() {
        let face = m.face(cf.face).unwrap();
        let Surface::Cone {
            frame,
            radius: r,
            half_angle,
            ..
        } = m.surface(face.surface()).unwrap().clone()
        else {
            continue;
        };
        let apex_v = -r / half_angle.sin();
        for i in cf.vertices.clone() {
            let [u, v] = corners.uvs()[i];
            if (v - apex_v).abs() > EXACT {
                continue;
            }
            seen += 1;
            // The cone's normal does not depend on `v`, so the limit at
            // the apex is the cone's own at this corner's `u`.
            let want = frame.x().into_inner() * (half_angle.cos() * u.cos())
                + frame.y().into_inner() * (half_angle.cos() * u.sin())
                - frame.z().into_inner() * half_angle.sin();
            let n = Vec3::from(corners.normals()[i]);
            assert!(
                (n - want).norm() <= EXACT,
                "an apex corner at u = {u} carries {n}, not the cone's {want}"
            );
            directions.push(n);
        }
    }
    assert!(seen > 2, "the apex is a fan of corners, not one: {seen}");
    // A ring, not one repeated direction: the fan spans the whole turn.
    let spread = directions
        .iter()
        .flat_map(|a| directions.iter().map(move |b| (a - b).norm()))
        .fold(0.0f64, f64::max);
    assert!(
        spread > half_angle_spread(radius, height),
        "the apex normals are not a ring: spread {spread}"
    );
}

/// Two normals a half turn apart on a cone of this shape differ by
/// `2 cos α`; anything above `cos α` is a ring rather than a point.
fn half_angle_spread(radius: f64, height: f64) -> f64 {
    (radius / (radius * radius + height * height).sqrt())
        .atan()
        .cos()
}

/// A corner block that does not fit is a typed refusal, never a panic
/// and never a mesh whose two index spaces disagree.
#[test]
fn a_block_that_does_not_fit_is_a_typed_error() {
    let mut m = Model::default();
    let body = sample::cuboid(&mut m, Point3::origin(), Point3::new(1.0, 1.0, 1.0)).unwrap();
    let (mesh, corners) = corners_of(&m, body, 1e-2);
    let plain = tessellate(&m, body, 1e-2).unwrap();

    // Parallel arrays that are not.
    let short = Corners::from_parts(
        corners.positions().to_vec(),
        corners.normals()[1..].to_vec(),
        corners.uvs().to_vec(),
        corners.triangles().to_vec(),
        corners.faces().to_vec(),
    );
    assert!(matches!(short, Err(MeshError::Corners(_))), "{short:?}");

    // A normal that was never normalised.
    let mut normals = corners.normals().to_vec();
    normals[0] = [0.0, 0.0, 2.0];
    let stretched = Corners::from_parts(
        corners.positions().to_vec(),
        normals,
        corners.uvs().to_vec(),
        corners.triangles().to_vec(),
        corners.faces().to_vec(),
    );
    assert!(
        matches!(stretched, Err(MeshError::Corners(_))),
        "{stretched:?}"
    );

    // A (u, v) outside the face's own box.
    let mut uvs = corners.uvs().to_vec();
    uvs[0] = [1e6, 1e6];
    let outside = Corners::from_parts(
        corners.positions().to_vec(),
        corners.normals().to_vec(),
        uvs,
        corners.triangles().to_vec(),
        corners.faces().to_vec(),
    );
    assert!(matches!(outside, Err(MeshError::Corners(_))), "{outside:?}");

    // Faces that do not partition the vertices.
    let mut faces: Vec<CornerFace> = corners.faces().to_vec();
    faces[0].vertices.end += 1;
    let overlapping = Corners::from_parts(
        corners.positions().to_vec(),
        corners.normals().to_vec(),
        corners.uvs().to_vec(),
        corners.triangles().to_vec(),
        faces,
    );
    assert!(
        matches!(overlapping, Err(MeshError::Corners(_))),
        "{overlapping:?}"
    );

    // A block whose corners stand on other vertices than the mesh's.
    let mut shifted = corners.positions().to_vec();
    shifted.rotate_left(1);
    let block = Corners::from_parts(
        shifted,
        corners.normals().to_vec(),
        corners.uvs().to_vec(),
        corners.triangles().to_vec(),
        corners.faces().to_vec(),
    )
    .unwrap();
    assert!(matches!(
        plain.clone().with_corners(block),
        Err(MeshError::Corners(_))
    ));

    // A block of another mesh's size.
    let other = sample::cylinder(&mut m, 1.0, 1.0).unwrap();
    let (_, elsewhere) = corners_of(&m, other, 1e-2);
    assert!(matches!(
        mesh.with_corners(elsewhere),
        Err(MeshError::Corners(_))
    ));
}
