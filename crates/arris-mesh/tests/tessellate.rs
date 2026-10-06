//! Tessellation (ADR-0003): the sample bodies and both primitives mesh
//! closed, with one range per entity in iteration order, the seam used
//! twice, every position on its geometry, and a volume the closed form
//! of an inscribed prism bounds; a thousand random cylinders in random
//! poses do the same; bad chords and bodies are typed errors; two runs
//! are identical. A patch of every surface kind in a random pose stays
//! within its chord, the sphere and the torus mesh closed through their
//! interior grids, and a ruled surface takes no grid at all — a frustum
//! of any radius ratio up to 100 and a cone down to its apex stay within
//! their chord, a frustum with rings where its radii are far apart. The wall of
//! a hole drilled at an angle (ADR-0005) — a strip oblique to the ruling
//! — meshes column by column, at the fixture's tilt and at random ones.

use arris_debug::unmetered::{mass_properties, primitive_box, primitive_cylinder};
use core::f64::consts::{PI, TAU};
use std::collections::{BTreeMap, BTreeSet};

use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, point_in_box, radius, unit_vec3};
use arris_debug::sample;
use arris_debug::testing::fail;
use arris_debug::unmetered::tessellate;
use arris_geom::region2::MIN_SEGMENTS_PER_TURN;
use arris_geom::{CurveKind, NurbsSurface, Surface, SurfaceKind};
use arris_math::{Axis, Interval, Point2, Point3, Vec3};
use arris_mesh::{MeshError, RING_RATIO, TriMesh};
use arris_topo::entity::{Body as BodyEntity, EdgeGeometry};
use arris_topo::{Body, Model, Shell as ShellHandle, ShellId};
use proptest::prelude::*;

/// Rounding at the scale of a coordinate.
const EXACT: f64 = 1e-12;

fn p3(p: [f64; 3]) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

/// The relative volume error of an inscribed prism at sagitta `chord`
/// on radius `r`: the closed form of ADR-0003.
fn prism_bound(chord: f64, r: f64) -> f64 {
    4.0 * chord / (3.0 * r)
}

/// The structural guarantees every mesh of a solid keeps: closed, ranges
/// per entity in iteration order, every edge polyline from its start
/// vertex to its end vertex, every position on the geometry it came
/// from, every edge segment a triangle edge in both directions.
fn assert_structure(m: &Model, body: Body, mesh: &TriMesh) {
    assert!(mesh.is_closed(), "not closed");
    let tol = m.precision().default_tolerance;
    let faces = m.faces(body).unwrap();
    let edges = m.edges(body).unwrap();
    assert_eq!(
        mesh.faces().iter().map(|f| f.face).collect::<Vec<_>>(),
        faces.iter().map(|f| f.id).collect::<Vec<_>>(),
        "one FaceRange per face, in iteration order"
    );
    assert_eq!(
        mesh.edges().iter().map(|e| e.edge).collect::<Vec<_>>(),
        edges.iter().map(|e| e.id).collect::<Vec<_>>(),
        "one EdgeRange per edge, in iteration order"
    );
    let positions = mesh.positions();
    // Vertices: the polyline ends are the vertices' own points.
    for e in &edges {
        let edge = m.edge(e.id).unwrap();
        let polyline = mesh.edge_polyline(e.id).unwrap();
        let (start, end) = (
            m.vertex(edge.start()).unwrap().point(),
            m.vertex(edge.end()).unwrap().point(),
        );
        assert_eq!(p3(positions[polyline[0] as usize]), start);
        assert_eq!(p3(positions[*polyline.last().unwrap() as usize]), end);
        // Every sample on the curve. Cycle 1 has no closed form for the
        // nearest point of a NURBS *surface*, and none is asked of one
        // here either: the NURBS arms are checked against their own
        // closed form in `a_nurbs_patch_meshes_through_its_grid`.
        if let EdgeGeometry::Curve { curve, range } = edge.geometry() {
            let curve = m.curve(curve).unwrap();
            assert!(polyline.len() >= 2);
            for &i in polyline {
                let p = p3(positions[i as usize]);
                if curve.kind() != CurveKind::Nurbs {
                    let projection = curve.project(p).unwrap();
                    assert!(projection.distance <= tol, "{p} is off its curve");
                    // The end vertex of a straight edge projects back to
                    // the range's end give or take an ulp of the
                    // coordinate, so the range is read at the model's
                    // parametric tolerance.
                    let slack = m.precision().parametric_tolerance;
                    assert!(
                        (projection.t - range.clamp(projection.t)).abs() <= slack
                            || curve.period().is_some(),
                        "{} {:?} t {} range {range:?}",
                        e.id,
                        curve.kind(),
                        projection.t
                    );
                }
            }
        }
    }
    // Faces: every corner on the surface, every triangle non-degenerate.
    let mut directed: BTreeMap<(u32, u32), Vec<usize>> = BTreeMap::new();
    for (k, f) in faces.iter().enumerate() {
        let face = m.face(f.id).unwrap();
        let surface = m.surface(face.surface()).unwrap();
        let triangles = mesh.face_triangles(f.id).unwrap();
        assert!(!triangles.is_empty(), "{} has no triangles", f.id);
        let projectable = surface.kind() != SurfaceKind::Nurbs;
        for t in triangles {
            assert!(t[0] != t[1] && t[1] != t[2] && t[2] != t[0]);
            for &i in t {
                let p = p3(positions[i as usize]);
                if projectable {
                    let projection = surface.project(p).unwrap();
                    assert!(projection.distance <= tol, "{p} is off {}", f.id);
                }
            }
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                directed.entry((a, b)).or_default().push(k);
            }
        }
    }
    // Every segment of every edge is a triangle edge once in each
    // direction, so the loops were honoured and every edge is shared.
    for e in &edges {
        let polyline = mesh.edge_polyline(e.id).unwrap();
        for w in polyline.windows(2) {
            assert_eq!(directed.get(&(w[0], w[1])).map(Vec::len), Some(1));
            assert_eq!(directed.get(&(w[1], w[0])).map(Vec::len), Some(1));
        }
    }
}

/// The faces (by iteration index) whose triangles run along `edge`'s
/// polyline, in either direction.
fn faces_along(m: &Model, body: Body, mesh: &TriMesh, edge: arris_topo::EdgeId) -> BTreeSet<usize> {
    let faces = m.faces(body).unwrap();
    let polyline = mesh.edge_polyline(edge).unwrap();
    let segments: BTreeSet<(u32, u32)> = polyline
        .windows(2)
        .flat_map(|w| [(w[0], w[1]), (w[1], w[0])])
        .collect();
    let mut out = BTreeSet::new();
    for (k, f) in faces.iter().enumerate() {
        for t in mesh.face_triangles(f.id).unwrap() {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                if segments.contains(&(a, b)) {
                    out.insert(k);
                }
            }
        }
    }
    out
}

#[test]
fn the_unit_box_meshes_exactly() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    let mesh = tessellate(&m, body, 1e-3).unwrap();
    assert_structure(&m, body, &mesh);
    assert_eq!(mesh.positions().len(), 8);
    assert_eq!(mesh.triangles().len(), 12);
    assert!((mesh.signed_volume().unwrap() - 1.0).abs() <= EXACT);
    assert!((mesh.area() - 6.0).abs() <= EXACT);
    assert_eq!(mesh, tessellate(&m, body, 1e-3).unwrap(), "two runs");
}

#[test]
fn the_frame_meshes_with_its_window_open() {
    let mut m = Model::default();
    let body = sample::frame(
        &mut m,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap();
    let mesh = tessellate(&m, body, 1e-3).unwrap();
    assert_structure(&m, body, &mesh);
    assert_eq!(mesh.positions().len(), 16);
    let volume = 40.0 * 30.0 * 10.0 - 20.0 * 10.0 * 10.0;
    assert!((mesh.signed_volume().unwrap() - volume).abs() <= EXACT * volume);
    // Every edge lies in exactly two faces; the window's edges in the
    // two-loop face and an inner wall.
    for e in m.edges(body).unwrap() {
        assert_eq!(faces_along(&m, body, &mesh, e.id).len(), 2, "{}", e.id);
    }
    let two_loop: Vec<_> = m
        .faces(body)
        .unwrap()
        .iter()
        .filter(|f| m.face(f.id).unwrap().loops().len() == 2)
        .map(|f| f.id)
        .collect();
    assert_eq!(two_loop.len(), 2);
    for f in two_loop {
        assert_eq!(mesh.face_triangles(f).unwrap().len(), 8);
    }
    assert_eq!(mesh, tessellate(&m, body, 1e-3).unwrap(), "two runs");
}

/// The mesh of a cylinder of radius `r`, height `h` at `chord`: closed,
/// the seam used twice, and its volume the inscribed prism's exactly —
/// `sin θ / θ` of the true volume for `θ = 2π / n` — with the closed
/// form `4δ / (3r)` bounding the error.
fn assert_cylinder(m: &Model, body: Body, mesh: &TriMesh, r: f64, h: f64, chord: f64) {
    assert_structure(m, body, mesh);
    let edges = m.edges(body).unwrap();
    let faces = m.faces(body).unwrap();
    // The seam is the one edge with distinct end vertices; it lies in
    // the wall alone, in both directions.
    let seam = edges
        .iter()
        .find(|e| !m.edge(e.id).unwrap().is_closed())
        .unwrap();
    let wall = faces
        .iter()
        .position(|f| m.face(f.id).unwrap().loops()[0].coedges().len() == 4)
        .unwrap();
    assert_eq!(faces_along(m, body, mesh, seam.id), BTreeSet::from([wall]));
    let rim = edges
        .iter()
        .find(|e| m.edge(e.id).unwrap().is_closed())
        .unwrap();
    let n = mesh.edge_polyline(rim.id).unwrap().len() - 1;
    assert!(n >= MIN_SEGMENTS_PER_TURN);
    let theta = TAU / n as f64;
    let exact = PI * r * r * h;
    let volume = mesh.signed_volume().unwrap();
    let ratio = volume / exact;
    assert!(
        (ratio - theta.sin() / theta).abs() <= 1e-9,
        "ratio {ratio} vs sin θ / θ {}",
        theta.sin() / theta
    );
    let error = 1.0 - ratio;
    assert!(error >= 0.0, "inscribed");
    assert!(
        error <= prism_bound(chord, r),
        "error {error} above 4δ/(3r) = {}",
        prism_bound(chord, r)
    );
    // The sagitta the mesh achieved is within the chord asked for.
    assert!(r * (1.0 - (theta / 2.0).cos()) <= chord * (1.0 + 1e-12));
}

#[test]
fn the_sample_cylinder_meshes_within_the_closed_form_bound() {
    let mut m = Model::default();
    let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    for chord in [1e-1, 1e-2, 1e-3, 1e-4] {
        let mesh = tessellate(&m, body, chord).unwrap();
        assert_cylinder(&m, body, &mesh, 4.0, 12.0, chord);
        assert_eq!(mesh, tessellate(&m, body, chord).unwrap(), "two runs");
    }
}

#[test]
fn both_primitives_mesh() {
    let mut m = Model::default();
    let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let mesh = tessellate(&m, cube, 1e-3).unwrap();
    assert_structure(&m, cube, &mesh);
    assert!((mesh.signed_volume().unwrap() - 12000.0).abs() <= EXACT * 12000.0);
    assert_eq!(mesh, tessellate(&m, cube, 1e-3).unwrap(), "two runs");
    let (cylinder, _) =
        primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 4.0, 12.0).unwrap();
    let mesh = tessellate(&m, cylinder, 1e-2).unwrap();
    assert_cylinder(&m, cylinder, &mesh, 4.0, 12.0, 1e-2);
    assert_eq!(mesh, tessellate(&m, cylinder, 1e-2).unwrap(), "two runs");
}

/// `random_cylinders_in_random_poses_mesh_within_the_bound` at 5000 cases
/// on the fixed seed, shrunk: a cylinder of radius and height 0.1 about 120
/// out. `signed_volume` summed its triple products about the world origin,
/// each of the order of the distance cubed, and lost 9e-9 of the volume to
/// their cancellation; about the mesh's own box it holds to rounding
/// (ADR-0024).
#[test]
fn a_small_cylinder_far_out_keeps_its_mesh_volume() {
    let (r, h, chord) = (0.1, 0.1, 0.1 * 0.1);
    let mut m = Model::default();
    let origin = Point3::new(-64.57238853865147, 68.58539347065553, 76.73988644277067);
    let axis = Axis::new(origin, Vec3::x()).unwrap();
    let (body, _) = primitive_cylinder(&mut m, axis, r, h).unwrap();
    let mesh = tessellate(&m, body, chord).unwrap();
    let n = mesh
        .edge_polyline(m.edges(body).unwrap()[0].id)
        .unwrap()
        .len()
        - 1;
    let theta = TAU / n as f64;
    let ratio = mesh.signed_volume().unwrap() / (PI * r * r * h);
    assert!(
        (ratio - theta.sin() / theta).abs() <= 1e-12,
        "ratio {ratio} vs sin θ / θ {}",
        theta.sin() / theta
    );
}

#[test]
fn random_cylinders_in_random_poses_mesh_within_the_bound() {
    check(
        (
            radius(0.1..=10.0),
            radius(0.1..=DEFAULT_SCALE),
            point_in_box(DEFAULT_SCALE),
            unit_vec3(),
            finite_f64(-5.0..=-1.0),
        ),
        |(r, h, origin, direction, log_chord)| {
            let chord = 10f64.powf(log_chord) * r;
            let mut m = Model::default();
            let axis = Axis::new(origin, direction.into_inner()).unwrap();
            let (body, _) = primitive_cylinder(&mut m, axis, r, h).unwrap();
            let mesh = tessellate(&m, body, chord).map_err(fail)?;
            prop_assert!(mesh.is_closed());
            let n = mesh
                .edge_polyline(m.edges(body).unwrap()[0].id)
                .unwrap()
                .len()
                - 1;
            let theta = TAU / n as f64;
            let exact = PI * r * r * h;
            let volume = mesh.signed_volume().unwrap();
            let ratio = volume / exact;
            prop_assert!(
                (ratio - theta.sin() / theta).abs() <= 1e-9,
                "ratio {ratio} vs {}",
                theta.sin() / theta
            );
            prop_assert!(1.0 - ratio <= prism_bound(chord, r), "above the bound");
            prop_assert!(1.0 - ratio >= -1e-12, "not inscribed");
            prop_assert_eq!(&mesh, &tessellate(&m, body, chord).map_err(fail)?);
            Ok(())
        },
    );
}

#[test]
fn a_bad_chord_is_a_typed_error() {
    let mut m = Model::default();
    let body = sample::unit_box(&mut m).unwrap();
    for chord in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        match tessellate(&m, body, chord) {
            Err(MeshError::Chord(c)) => {
                assert!(c.is_nan() == chord.is_nan() && (c.is_nan() || c == chord))
            }
            other => panic!("{chord}: {other:?}"),
        }
    }
    let err = tessellate(&m, body, -1.0).unwrap_err();
    assert_eq!(
        err.to_string(),
        "chord tolerance -1 is not finite and positive"
    );
}

#[test]
fn a_body_that_does_not_resolve_is_not_found() {
    let m = Model::default();
    let body = Body::forward(arris_topo::BodyId::new(3, 0));
    assert!(matches!(
        tessellate(&m, body, 1e-3),
        Err(MeshError::NotFound(_))
    ));
}

#[cfg(debug_assertions)]
#[test]
fn a_broken_body_is_invalid_input_in_a_debug_build() {
    let mut m = Model::default();
    // A solid whose one shell does not exist: the checker's M1.
    let body = m
        .raw()
        .add_body(BodyEntity::solid(vec![ShellHandle::forward(ShellId::new(
            9, 0,
        ))]));
    let err = tessellate(&m, Body::forward(body), 1e-3).unwrap_err();
    assert!(matches!(err, MeshError::InvalidInput { .. }), "{err}");
    assert!(err.to_string().contains("fails the checker"));
}

// ---------------------------------------------------------------------
// Step 3: the doubly curved kinds and their interior grids.

/// The (u, v) box a patch of `surface` is cut from: the whole domain of a
/// NURBS surface, and for the analytic kinds a box clear of every
/// singularity — a cone's apex, a sphere's poles — and short of a whole
/// period, so the four sides are four distinct edges.
fn patch_box(surface: &Surface) -> [Interval; 2] {
    let i = |lo: f64, hi: f64| Interval::new(lo, hi).unwrap();
    match surface {
        Surface::Plane { .. } => [i(-5.0, 5.0), i(-5.0, 5.0)],
        Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } => {
            [i(0.0, 0.9 * TAU), i(-5.0, 5.0)]
        }
        Surface::Cone { .. } => [i(0.0, 0.9 * TAU), i(0.5, 5.0)],
        Surface::Sphere { .. } => [i(0.0, 0.9 * TAU), i(-1.4, 1.4)],
        Surface::Torus { .. } => [i(0.0, 0.9 * TAU), i(0.0, 0.9 * TAU)],
        Surface::Nurbs(s) => s.domain(),
    }
}

/// A sub-interval of `box` at the two fractions, between a fifth and two
/// fifths of it long, so a patch is never a sliver and never the whole
/// period.
fn sub_interval(bounds: Interval, lo: f64, span: f64) -> Interval {
    let lo_fraction = 0.6 * lo;
    Interval::new(
        bounds.lerp(lo_fraction),
        bounds.lerp(lo_fraction + 0.2 + 0.2 * span),
    )
    .unwrap()
}

/// A chord that puts about six steps in each curved direction of the
/// region, so a random surface's grid stays small however sharply it
/// curves. The step scales as `√chord`, so `chord = (L / 6 h₁)²` for the
/// step `h₁` at chord 1; never below 1e-2, an order below the smallest
/// radius the strategies draw.
fn chord_for(surface: &Surface, region: [Interval; 2]) -> f64 {
    let unit = surface.chord_steps(1.0, region);
    let mut chord: f64 = 1e-2;
    for dir in 0..2 {
        if unit[dir].is_finite() && unit[dir] > 0.0 {
            chord = chord.max((region[dir].length() / (6.0 * unit[dir])).powi(2));
        }
    }
    chord
}

/// The mesh indices that lie on some edge's polyline: everything but the
/// interior grid.
fn boundary_indices(mesh: &TriMesh) -> BTreeSet<u32> {
    mesh.edges()
        .iter()
        .flat_map(|e| &mesh.edge_indices()[e.indices.clone()])
        .copied()
        .collect()
}

/// Every triangle of the mesh deviates from `surface` by at most `chord`,
/// measured where a flat triangle is farthest from a curved surface: its
/// centroid and the midpoints of its three edges.
fn assert_within_chord(mesh: &TriMesh, surface: &Surface, chord: f64) -> Result<(), TestCaseError> {
    for i in 0..mesh.triangles().len() {
        let t = mesh.triangle_positions(i).expect("a triangle of the mesh");
        let [a, b, c] = [p3(t[0]), p3(t[1]), p3(t[2])];
        let centroid = Point3::from((a.coords + b.coords + c.coords) / 3.0);
        let probes = [
            centroid,
            Point3::from((a.coords + b.coords) / 2.0),
            Point3::from((b.coords + c.coords) / 2.0),
            Point3::from((c.coords + a.coords) / 2.0),
        ];
        for p in probes {
            let projection = surface.project(p).map_err(fail)?;
            prop_assert!(
                projection.distance <= chord,
                "{p} is {} off the surface, above the chord {chord}",
                projection.distance
            );
        }
    }
    Ok(())
}

#[test]
fn a_patch_of_every_surface_kind_meshes_onto_its_surface() {
    check(
        (
            arris_debug::prop::geom::surface(),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
        ),
        |(surface, lo_u, span_u, lo_v, span_v)| {
            let full = patch_box(&surface);
            let region = [
                sub_interval(full[0], lo_u, span_u),
                sub_interval(full[1], lo_v, span_v),
            ];
            let chord = chord_for(&surface, region);
            let mut m = Model::default();
            let body =
                sample::patch(&mut m, surface.clone(), region[0], region[1]).map_err(fail)?;
            let mesh = tessellate(&m, body, chord).map_err(fail)?;
            prop_assert!(!mesh.triangles().is_empty());
            assert_within_chord(&mesh, &surface, chord)?;
            // A ruled direction needs no interior point; a plane and a
            // cylinder are ruled in one, so their grids are empty, and a
            // cone takes rings only where its radii are far apart.
            let interior = mesh.positions().len() - boundary_indices(&mesh).len();
            match surface {
                Surface::Plane { .. }
                | Surface::Cylinder { .. }
                | Surface::EllipticCylinder { .. } => {
                    prop_assert_eq!(interior, 0, "a ruled surface takes no interior point");
                }
                Surface::Cone { .. } => {
                    if radius_ratio(&surface, region[1]) <= RING_RATIO {
                        prop_assert_eq!(interior, 0, "a narrow frustum takes no ring");
                    }
                }
                Surface::Sphere { .. } | Surface::Torus { .. } => {
                    // A region narrower than one step in a direction has
                    // no line to put a point on; anything wider does.
                    let steps = surface.chord_steps(chord, region);
                    if (0..2).all(|d| region[d].length() > steps[d]) {
                        prop_assert!(interior > 0, "a doubly curved patch needs interior points");
                    }
                }
                // `prop::geom::surface` draws the analytic kinds only;
                // the NURBS arm is `a_nurbs_patch_meshes_through_its_grid`.
                Surface::Nurbs(_) => prop_assert!(false, "no NURBS in this strategy"),
            }
            prop_assert_eq!(&mesh, &tessellate(&m, body, chord).map_err(fail)?);
            Ok(())
        },
    );
}

/// The ratio of a cone's radii at the two ends of `v`.
fn radius_ratio(cone: &Surface, v: Interval) -> f64 {
    let Surface::Cone {
        radius, half_angle, ..
    } = *cone
    else {
        panic!("not a cone: {cone:?}");
    };
    let (a, b) = (
        (radius + v.lo() * half_angle.sin()).abs(),
        (radius + v.hi() * half_angle.sin()).abs(),
    );
    a.max(b) / a.min(b)
}

/// [`assert_within_chord`] probed on a barycentric grid over each
/// triangle: on a cone, a segment between samples of two radii is
/// farthest from the surface `1 / (1 + √(ρ₂/ρ₁))` of the way from the
/// narrow end, not at its middle (ADR-0052).
fn assert_within_chord_densely(
    mesh: &TriMesh,
    surface: &Surface,
    chord: f64,
) -> Result<(), TestCaseError> {
    const N: usize = 12;
    for i in 0..mesh.triangles().len() {
        let t = mesh.triangle_positions(i).expect("a triangle of the mesh");
        let [a, b, c] = [p3(t[0]), p3(t[1]), p3(t[2])];
        for j in 0..=N {
            for k in 0..=(N - j) {
                // A corner is a sample on the surface by construction,
                // and at an apex has no one nearest point to project to.
                if [(0, 0), (N, 0), (0, N)].contains(&(j, k)) {
                    continue;
                }
                let (x, y) = (j as f64 / N as f64, k as f64 / N as f64);
                let p = Point3::from(a.coords * (1.0 - x - y) + b.coords * x + c.coords * y);
                let projection = surface.project(p).map_err(fail)?;
                prop_assert!(
                    projection.distance <= chord,
                    "{p} is {} off the surface, above the chord {chord}",
                    projection.distance
                );
            }
        }
    }
    Ok(())
}

/// A frustum patch of `cone` whose radii run from `narrow` to
/// `narrow · ratio`, a little over a radian of the turn wide.
fn frustum_patch(m: &mut Model, cone: &Surface, ratio: f64, narrow: f64) -> (Body, Interval) {
    let Surface::Cone {
        radius, half_angle, ..
    } = *cone
    else {
        panic!("not a cone: {cone:?}");
    };
    let v_at = |rho: f64| (rho - radius) / half_angle.sin();
    let v = Interval::new(v_at(narrow), v_at(narrow * ratio)).unwrap();
    let body = sample::patch(m, cone.clone(), Interval::new(0.0, 1.2).unwrap(), v).unwrap();
    (body, v)
}

/// Frusta of radius ratio 2, 3, 10 and 100 mesh within the chord, probed
/// densely; one of ratio 2 takes no ring and every wider one does
/// (ADR-0052).
#[test]
fn a_frustum_meshes_within_its_chord_at_every_radius_ratio() {
    let cone = Surface::Cone {
        frame: arris_math::Frame::world(),
        radius: 1.0,
        half_angle: 0.6,
    };
    for ratio in [2.0, 3.0, 10.0, 100.0] {
        for chord in [1e-2, 1e-3] {
            let mut m = Model::default();
            let (body, v) = frustum_patch(&mut m, &cone, ratio, 0.05);
            assert!((radius_ratio(&cone, v) - ratio).abs() <= 1e-9 * ratio);
            let mesh = tessellate(&m, body, chord).unwrap();
            assert_within_chord_densely(&mesh, &cone, chord).unwrap();
            let interior = mesh.positions().len() - boundary_indices(&mesh).len();
            if ratio <= RING_RATIO {
                assert_eq!(interior, 0, "ratio {ratio} takes no ring");
            } else {
                assert!(interior > 0, "ratio {ratio} takes rings");
            }
        }
    }
}

/// A solid cone, its apex a degenerate edge: within the chord at every
/// probe, closed, its apex one index, and no ring — a face reaching its
/// apex takes none, since a ring would open the apex's fan (ADR-0052).
/// (`assert_structure` projects every corner, and the apex has no one
/// nearest point on the cone.)
#[test]
fn a_cone_with_its_apex_meshes_within_its_chord() {
    use arris_debug::unmetered::revolve;
    use arris_geom::{Profile, ProfileLoop, ProfileSegment};
    let plane = arris_math::Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
    let at = Point2::new;
    let profile = Profile {
        plane,
        outer: ProfileLoop::Path {
            start: at(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(at(4.0, 0.0)),
                ProfileSegment::LineTo(at(0.0, 3.0)),
                ProfileSegment::LineTo(at(0.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let mut m = Model::default();
    let (body, _) = revolve(&mut m, &profile, Axis::z_at(Point3::origin()), TAU).unwrap();
    for chord in [1e-2, 1e-3] {
        let mesh = tessellate(&m, body, chord).unwrap();
        assert!(mesh.is_closed());
        let interior = mesh.positions().len() - boundary_indices(&mesh).len();
        assert_eq!(interior, 0, "a cone reaching its apex takes no ring");
        for e in m.edges(body).unwrap() {
            if m.edge(e.id).unwrap().is_degenerate() {
                assert_eq!(mesh.edge_polyline(e.id).unwrap().len(), 1, "the apex");
            }
        }
        for range in mesh.faces() {
            let surface = m.surface(m.face(range.face).unwrap().surface()).unwrap();
            if !matches!(surface, Surface::Cone { .. }) {
                continue;
            }
            let mut part = TriMesh::new();
            for i in range.triangles.clone() {
                let t = mesh.triangle_positions(i).unwrap();
                let a = part.push_position(t[0]).unwrap();
                let b = part.push_position(t[1]).unwrap();
                let c = part.push_position(t[2]).unwrap();
                part.push_triangle([a, b, c]).unwrap();
            }
            assert_within_chord_densely(&part, surface, chord).unwrap();
        }
    }
}

/// Random cones in random poses, cut to frusta of radius ratio up to
/// 100 at a random narrow radius, mesh within the chord probed densely.
#[test]
fn a_frustum_of_any_ratio_meshes_within_its_chord() {
    check(
        (
            arris_debug::prop::geom::cone(),
            finite_f64(0.0..=2.0),
            finite_f64(0.1..=2.0),
        ),
        |(cone, log_ratio, narrow)| {
            let ratio = 10f64.powf(log_ratio).max(1.01);
            let mut m = Model::default();
            let (body, v) = frustum_patch(&mut m, &cone, ratio, narrow);
            let chord = chord_for(&cone, [Interval::new(0.0, 1.2).unwrap(), v]);
            let mesh = tessellate(&m, body, chord).map_err(fail)?;
            prop_assert!(!mesh.triangles().is_empty());
            assert_within_chord_densely(&mesh, &cone, chord)?;
            Ok(())
        },
    );
}

#[test]
fn the_sample_sphere_meshes_within_the_inscribed_bound() {
    let mut m = Model::default();
    let radius = 3.0;
    let body = sample::sphere(&mut m, Point3::new(1.0, -2.0, 0.5), radius).unwrap();
    let report = arris_check::check(&m, body, arris_check::Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(
        report.unchecked().is_empty(),
        "one face has no pair to skip"
    );
    for chord in [1e-1, 1e-2, 1e-3] {
        let mesh = tessellate(&m, body, chord).unwrap();
        assert_structure(&m, body, &mesh);
        // The two poles are one index each and no triangle survives that
        // collapses onto one.
        for e in m.edges(body).unwrap() {
            if m.edge(e.id).unwrap().is_degenerate() {
                assert_eq!(mesh.edge_polyline(e.id).unwrap().len(), 1, "{}", e.id);
            }
        }
        // Inscribed: every triangle plane is at least `radius − chord`
        // from the centre, so the mesh holds that ball and is held by the
        // sphere.
        let exact = 4.0 / 3.0 * PI * radius.powi(3);
        let volume = mesh.signed_volume().unwrap();
        let inner = 4.0 / 3.0 * PI * (radius - chord).powi(3);
        assert!(
            volume <= exact * (1.0 + EXACT) && volume >= inner,
            "{volume} outside [{inner}, {exact}] at chord {chord}"
        );
        assert_eq!(mesh, tessellate(&m, body, chord).unwrap(), "two runs");
    }
}

/// A torus whose minor radius is far smaller than its major one, at a
/// chord fine enough for the minor circle: the major direction's lattice
/// (which does not shrink with the minor radius) would need billions of
/// points at that chord. `MeshError::GridTooLarge` names the face and the
/// count it would have needed, not an allocation.
#[test]
fn a_torus_with_a_tiny_minor_radius_at_a_fine_chord_is_grid_too_large() {
    let mut m = Model::default();
    let (major, minor) = (5.0, 1e-6);
    let body = sample::torus(&mut m, Point3::origin(), major, minor).unwrap();
    let face = m.faces(body).unwrap()[0].id;
    match tessellate(&m, body, 1e-12).unwrap_err() {
        MeshError::GridTooLarge { face: f, points } => {
            assert_eq!(f, face);
            assert!(points > 1 << 20, "{points}");
        }
        other => panic!("{other:?}"),
    }
}

#[test]
fn the_sample_torus_meshes_closed() {
    let mut m = Model::default();
    let (major, minor) = (5.0, 2.0);
    let body = sample::torus(&mut m, Point3::origin(), major, minor).unwrap();
    let report = arris_check::check(&m, body, arris_check::Level::Full);
    assert!(report.is_ok(), "{report}");
    for chord in [1e-1, 1e-2] {
        let mesh = tessellate(&m, body, chord).unwrap();
        assert_structure(&m, body, &mesh);
        // Both seams are used by the one face twice, in opposite
        // directions; the surface's area bounds the volume a boundary
        // that is everywhere within `chord` of it can be wrong by.
        let exact = 2.0 * PI * PI * major * minor * minor;
        let area = 4.0 * PI * PI * major * minor;
        let volume = mesh.signed_volume().unwrap();
        assert!(
            (volume - exact).abs() <= area * chord,
            "{volume} against {exact} at chord {chord}"
        );
        assert!(
            mesh.positions().len() > boundary_indices(&mesh).len(),
            "grid"
        );
        assert_eq!(mesh, tessellate(&m, body, chord).unwrap(), "two runs");
    }
}

/// A saddle as a bilinear NURBS: `P(u, v) = (u, v, u v)` over `[0, 1]²`,
/// whose closed form stands in for the projection cycle 1 has no NURBS
/// arm for — the surface is the graph of `z = x y`, so the vertical
/// distance from any point to it bounds the true one.
fn saddle() -> NurbsSurface {
    NurbsSurface::new(
        [1, 1],
        [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ],
        vec![1.0; 4],
    )
    .unwrap()
}

#[test]
fn a_nurbs_patch_meshes_through_its_grid() {
    let mut m = Model::default();
    let surface = Surface::Nurbs(saddle());
    let unit = Interval::new(0.0, 1.0).unwrap();
    let body = sample::patch(&mut m, surface, unit, unit).unwrap();
    let chord = 1e-3;
    let mesh = tessellate(&m, body, chord).unwrap();
    // A sheet is not closed, so `assert_structure` does not apply: what a
    // patch owes is one range per entity and triangles on the surface.
    assert_eq!(mesh.faces().len(), 1);
    assert_eq!(mesh.edges().len(), 4);
    for t in mesh.triangles() {
        assert!(t[0] != t[1] && t[1] != t[2] && t[2] != t[0]);
    }
    // A saddle curves in the mixed direction, so its face takes a grid.
    assert!(
        mesh.positions().len() > boundary_indices(&mesh).len(),
        "a curved NURBS patch needs interior points"
    );
    let tol = m.precision().default_tolerance;
    for p in mesh.positions() {
        assert!((p[2] - p[0] * p[1]).abs() <= tol, "{p:?} is off the saddle");
    }
    for i in 0..mesh.triangles().len() {
        let t = mesh.triangle_positions(i).unwrap();
        let [a, b, c] = [p3(t[0]), p3(t[1]), p3(t[2])];
        let centroid = Point3::from((a.coords + b.coords + c.coords) / 3.0);
        assert!(
            (centroid.z - centroid.x * centroid.y).abs() <= chord,
            "{centroid} is above the chord from the saddle"
        );
    }
    assert_eq!(mesh, tessellate(&m, body, chord).unwrap(), "two runs");
}

#[test]
fn the_nurbs_box_meshes_with_its_bilinear_face() {
    let mut m = Model::default();
    let body =
        sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let mesh = tessellate(&m, body, 1e-3).unwrap();
    assert_structure(&m, body, &mesh);
    // A bilinear patch is flat: its face takes no interior point, and
    // the ninth position is the middle of the one NURBS *edge*, which a
    // B-spline piece is sampled at `MIN_SEGMENTS_PER_SPAN` of.
    assert_eq!(mesh.positions().len(), 9);
    assert_eq!(mesh.positions().len(), boundary_indices(&mesh).len());
    let volume = 40.0 * 30.0 * 10.0;
    assert!((mesh.signed_volume().unwrap() - volume).abs() <= EXACT * volume);
}

/// A cylinder face bounded by two oblique sections: the strip its
/// (u, v) region makes runs oblique to the ruling, so Delaunay in a
/// domain scaled to the surface's own lengths joins each boundary point
/// to the *nearest* one on the other chain — offset along the ruling by
/// the shear, and so a large part of a turn away in `u`, where the
/// boundary is sampled every `0.048`. Flattening the ruled direction
/// (ADR-0005) leaves `u` alone to decide, and the wall is the inscribed
/// prism the bound is written for. `tests/fixtures/boolean/oblique-hole`
/// is the same solid through the corpus.
#[test]
fn an_oblique_hole_meshes_within_the_inscribed_bound() {
    let (r, tilt) = (3.0, PI / 6.0);
    let mut m = Model::default();
    let body = oblique_hole(&mut m, r, tilt, 8.0);
    let chord = 1e-3;
    let mesh = tessellate(&m, body, chord).unwrap();
    assert_structure(&m, body, &mesh);
    let exact = 12000.0 - PI * r * r * 10.0 / tilt.cos();
    let volume = mesh.signed_volume().unwrap();
    assert!(
        (volume - exact).abs() <= exact * prism_bound(chord, r),
        "mesh volume {volume} vs {exact}: {} relative, above {}",
        (volume - exact).abs() / exact,
        prism_bound(chord, r)
    );
    // The wall is one quad per step of its boundary: no triangle travels
    // more of the turn than the chord step the bound stands on.
    let step = (8.0 * chord / r).sqrt();
    let travel = wall_travel(&m, body, &mesh);
    assert!(
        travel <= step,
        "a triangle travels {travel} of the turn, past the chord step {step}"
    );
    assert_eq!(mesh, tessellate(&m, body, chord).unwrap(), "two runs");
}

/// The same at random radii, tilts and chords, with the tool made long
/// enough to pierce the plate at every tilt so the closed form holds:
/// the solid measures as the closed form says, the mesh is within the
/// inscribed prism's bound of it, and no triangle of the wall travels
/// past one chord step of the turn however the strip is sheared.
#[test]
fn an_oblique_hole_meshes_column_by_column_at_random_tilts() {
    check(
        (
            radius(0.5..=3.0),
            finite_f64(0.0..=1.0),
            finite_f64(-4.0..=-2.0),
        ),
        |(r, tilt, log_chord)| {
            let chord = 10f64.powf(log_chord) * r;
            let mut m = Model::default();
            let half = (6.0 + r * tilt.sin()) / tilt.cos();
            let body = oblique_hole(&mut m, r, tilt, half);
            let exact = 12000.0 - PI * r * r * 10.0 / tilt.cos();
            let measured = mass_properties(&m, body).map_err(fail)?.volume;
            prop_assert!(
                (measured - exact).abs() <= 1e-9 * exact,
                "the solid is not the closed form's: {measured} vs {exact}"
            );
            let mesh = tessellate(&m, body, chord).map_err(fail)?;
            prop_assert!(mesh.is_closed());
            let volume = mesh.signed_volume().unwrap();
            prop_assert!(
                (volume - exact).abs() <= exact * prism_bound(chord, r),
                "volume {volume} vs {exact}: {} relative",
                (volume - exact).abs() / exact
            );
            let step = (8.0 * chord / r).sqrt();
            let travel = wall_travel(&m, body, &mesh);
            prop_assert!(travel <= step, "travel {travel} past the step {step}");
            Ok(())
        },
    );
}

/// The plate of `boolean/oblique-hole` less a cylinder of radius `r` and
/// length `2 half` through its middle, its axis tilted `tilt` from `z`
/// about `x`: the hole's wall is bounded by two ellipse sections, and
/// its (u, v) region is a strip sheared by `r tan(tilt)` against the
/// ruling.
fn oblique_hole(m: &mut Model, r: f64, tilt: f64, half: f64) -> Body {
    let (plate, _) = primitive_box(m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let axis = Axis::new(
        Point3::new(20.0, 15.0 - half * tilt.sin(), 5.0 - half * tilt.cos()),
        Vec3::new(0.0, tilt.sin(), tilt.cos()),
    )
    .unwrap();
    let (tool, _) = primitive_cylinder(m, axis, r, 2.0 * half).unwrap();
    arris_debug::unmetered::cut(m, plate, tool).unwrap().0
}

/// The largest turn any triangle of `body`'s one cylindrical face
/// travels: the `u` extent of its three corners, taken in the surface's
/// own frame, which is what the chord bound on a cylinder is written in.
fn wall_travel(m: &Model, body: Body, mesh: &TriMesh) -> f64 {
    let mut worst: f64 = 0.0;
    for f in m.faces(body).unwrap() {
        let face = m.face(f.id).unwrap();
        let Ok(Surface::Cylinder { frame, .. }) = m.surface(face.surface()) else {
            continue;
        };
        let range = mesh.faces().iter().find(|x| x.face == f.id).unwrap();
        let u_of = |i: u32| {
            let p = mesh.positions()[i as usize];
            let local = frame.to_local(p3([p[0], p[1], p[2]]));
            local.y.atan2(local.x)
        };
        for t in &mesh.triangles()[range.triangles.clone()] {
            let base = u_of(t[0]);
            let turns: Vec<f64> = t
                .iter()
                .map(|&i| {
                    let mut d = u_of(i) - base;
                    while d > PI {
                        d -= TAU;
                    }
                    while d < -PI {
                        d += TAU;
                    }
                    d
                })
                .collect();
            let lo = turns.iter().copied().fold(f64::INFINITY, f64::min);
            let hi = turns.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            worst = worst.max(hi - lo);
        }
    }
    worst
}
