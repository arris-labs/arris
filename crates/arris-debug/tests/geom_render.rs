//! A surface and curves without a body render through the rasteriser
//! (`docs/ARCHITECTURE.md` §Formats and tools), written to
//! `target/inspect/cylinder-wireframe.png` for the agent to read.

use arris_debug::render::colors;
use arris_debug::{View, polyline_of, render, render_png, wireframe_of};
use arris_geom::{Curve, Surface};
use arris_math::{Frame, Interval, Point3, Vec3};
use arris_mesh::TriMesh;

#[test]
fn cylinder_wireframe_with_a_circle_and_an_ellipse_renders() {
    let axis = Frame::new(Point3::new(0.0, 0.0, 0.0), Vec3::z(), Vec3::x()).unwrap();
    let cyl = Surface::Cylinder {
        frame: axis,
        radius: 1.0,
    };
    let height = Interval::new(-1.5, 1.5).unwrap();
    let mut lines = wireframe_of(&cyl, [Interval::TURN, height], 12);
    // A circle around the axis at v = −1.1 (between two wireframe rings, so
    // it reads as its own curve) and an oblique section at 30°:
    // an ellipse with minor radius R and major radius R / cos 30°.
    let circle = Curve::Circle {
        frame: Frame::new(Point3::new(0.0, 0.0, -1.1), Vec3::z(), Vec3::x()).unwrap(),
        radius: 1.0,
    };
    let tilt = 30f64.to_radians();
    let ellipse = Curve::Ellipse {
        frame: Frame::new(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, -tilt.sin(), tilt.cos()),
            Vec3::new(0.0, tilt.cos(), tilt.sin()),
        )
        .unwrap(),
        major_radius: 1.0 / tilt.cos(),
        minor_radius: 1.0,
    };
    lines.push(polyline_of(&circle, Interval::TURN, 96));
    lines.push(polyline_of(&ellipse, Interval::TURN, 96));
    // Every ellipse point lies on the cylinder.
    for p in &lines.last().unwrap().points {
        let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
        assert!(
            (r - 1.0).abs() < 1e-12,
            "ellipse point off the cylinder: {p:?}"
        );
    }
    let mesh = TriMesh::new();
    let raster = render(&mesh, &lines, View::Iso, None);
    let drawn = raster
        .pixels()
        .iter()
        .filter(|&&px| px == colors::LINE)
        .count();
    assert!(drawn > 1000, "only {drawn} line pixels");
    let path = render_png(&mesh, &lines, View::Iso, None, "cylinder-wireframe").unwrap();
    assert!(path.exists());
}

/// The curves `intersect_surfaces` returns for a cylinder cut by a
/// parallel plane (circle), an oblique plane (ellipse) and a perpendicular
/// plane through the axis (two rulings) render on the cylinder's
/// wireframe: `target/inspect/plane-cylinder-sections.png`.
#[test]
fn plane_cylinder_sections_render() {
    use arris_geom::{MeetKind, intersect_surfaces};
    use arris_math::Precision;

    let tol = Precision::DEFAULT.tolerance();
    let cyl = Surface::Cylinder {
        frame: Frame::world(),
        radius: 1.0,
    };
    let height = Interval::new(-1.5, 1.5).unwrap();
    let mut lines = wireframe_of(&cyl, [Interval::TURN, height], 12);
    let tilt = 30f64.to_radians();
    let planes = [
        Frame::from_z(Point3::new(0.0, 0.0, -1.1), Vec3::z()).unwrap(),
        Frame::from_z(
            Point3::new(0.0, 0.0, 0.5),
            Vec3::new(0.0, -tilt.sin(), tilt.cos()),
        )
        .unwrap(),
        Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vec3::x()).unwrap(),
    ];
    let mut kinds = Vec::new();
    for frame in planes {
        let plane = Surface::Plane { frame };
        let r = intersect_surfaces(
            &plane,
            &cyl,
            &within(),
            tol,
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(
            !r.curves().is_empty() && r.curves().iter().all(|m| m.kind == MeetKind::Crossing),
            "every section here is transversal: {r:?}"
        );
        for c in r.curves().iter().map(|m| m.curve.clone()) {
            kinds.push(c.kind());
            let range = match c {
                Curve::Line { .. } => height,
                _ => Interval::TURN,
            };
            lines.push(polyline_of(&c, range, 96));
        }
    }
    use arris_geom::CurveKind::{Circle, Ellipse, Line};
    assert_eq!(kinds, [Circle, Ellipse, Line, Line]);
    let mesh = TriMesh::new();
    let path = render_png(&mesh, &lines, View::Iso, None, "plane-cylinder-sections").unwrap();
    assert!(path.exists());
    let front = render_png(
        &mesh,
        &lines,
        View::Front,
        None,
        "plane-cylinder-sections-front",
    )
    .unwrap();
    assert!(front.exists());
}

/// The branches `trace_quadrics` finds render over the smaller surface's
/// wireframe: a pipe through a larger one on a skew axis (one loop),
/// tangent to it from inside (a figure eight through the singular point,
/// highlighted) and Viviani's curve — `target/inspect/trace-*.png`.
#[test]
fn traced_quadric_sections_render() {
    use arris_debug::Highlight;
    use arris_geom::trace_quadrics;
    use arris_math::{Aabb, Precision};
    use arris_mesh::Polyline;

    let tol = Precision::DEFAULT.tolerance();
    let within = Aabb {
        min: [-5.0; 3],
        max: [5.0; 3],
    };
    let main = Surface::Cylinder {
        frame: Frame::world(),
        radius: 2.0,
    };
    let pipe = |y: f64| Surface::Cylinder {
        frame: Frame::from_z(Point3::new(0.0, y, 0.0), Vec3::x()).unwrap(),
        radius: 1.0,
    };
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let through = Surface::Cylinder {
        frame: Frame::from_z(Point3::new(1.0, 0.0, 0.0), Vec3::z()).unwrap(),
        radius: 1.0,
    };
    let span = Interval::new(-3.0, 3.0).unwrap();
    let cases = [
        ("trace-skew-pipes", main.clone(), pipe(1.5), 1),
        ("trace-tangent-pipes", main, pipe(1.0), 2),
        ("trace-viviani", ball, through, 2),
    ];
    for (name, a, b, branches) in cases {
        let trace =
            trace_quadrics(&a, &b, &within, tol, &mut arris_math::Meter::default()).unwrap();
        assert_eq!(trace.branches().len(), branches, "{name}");
        let mut lines = wireframe_of(&b, [Interval::TURN, span], 12);
        for branch in trace.branches() {
            let domain = branch.domain();
            let points = (0..=256)
                .map(|i| branch.point(domain.lerp(i as f64 / 256.0)).coords.into())
                .collect();
            lines.push(Polyline::new(points));
        }
        let at = trace
            .points()
            .first()
            .map(|p| Highlight::Point(p.point.coords.into()));
        let path = render_png(&TriMesh::new(), &lines, View::Iso, at, name).unwrap();
        assert!(path.exists());
    }
}

/// A region every traced section of these tests lies in; the closed
/// forms ignore it.
fn within() -> arris_math::Aabb {
    arris_math::Aabb {
        min: [-100.0; 3],
        max: [100.0; 3],
    }
}
