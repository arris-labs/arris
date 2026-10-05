//! Valid bodies built by hand, with explicit pcurves: what the checker's
//! tests start from (they cannot use `arris-ops`) and what the STEP writer
//! is first tried on. The box and the cylinder go through the raw insert
//! API and are the shapes the matching `primitive/*` fixtures describe,
//! built with the conventions the primitives of `arris-ops` follow, so
//! the two dump alike; the [`frame`] goes through the Euler operators and
//! is `boolean/frame-cut`'s twin, the cross-check for the boolean that
//! builds it by recipe. [`sphere`] is the body with a seam and two
//! degenerate pole edges, [`torus`] the genus-1 one with two seams and
//! no pole, and [`patch`] a rectangular sheet of any surface kind over
//! its own exact iso-curves — what tessellation's property tests mesh,
//! since a body is the only way into it.

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_geom::{Curve, Curve2, GeomError, NurbsCurve, NurbsSurface, Surface};
use arris_math::{
    Frame, Frame2, FrameError, Interval, Point2, Point3, UnitVec2, UnitVec3, Vec2, Vec3,
};
use arris_topo::builder::{BuildError, Builder, FaceRef, Position, Seed, Split, Strut};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{Body, Face as FaceHandle, Model, Orientation, Shell as ShellHandle};

/// Why a sample could not be built.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SampleError {
    /// An extent is not finite and positive.
    #[error("sample {name} must be finite and positive, not {value}")]
    Extent {
        /// Which extent.
        name: &'static str,
        /// What was given.
        value: f64,
    },
    /// A placing frame could not be built.
    #[error("sample frame: {0}")]
    Frame(#[from] FrameError),
    /// A NURBS value could not be built.
    #[error("sample geometry: {0}")]
    Geometry(#[from] GeomError),
    /// The builder refused an operator or the finish.
    #[error("sample builder: {0}")]
    Build(#[from] BuildError),
    /// A [`patch`]'s (u, v) region is not one exact iso-curves bound.
    #[error("sample region: {0}")]
    Region(&'static str),
}

/// Which geometry [`cuboid`] and [`cuboid_nurbs`] give their entities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flavour {
    /// Lines and planes.
    Analytic,
    /// The first edge a degree-1 NURBS, the bottom face a bilinear NURBS.
    NurbsProbe,
}

fn positive(name: &'static str, value: f64) -> Result<f64, SampleError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(SampleError::Extent { name, value })
    }
}

fn finite_point(name: &'static str, p: Point3) -> Result<Point3, SampleError> {
    if p.coords.iter().all(|c| c.is_finite()) {
        Ok(p)
    } else {
        Err(SampleError::Extent {
            name,
            value: f64::NAN,
        })
    }
}

/// The pcurve `(t, v)` (`along_u`) or `(u, t)`: the straight side of a
/// rectangle in (u, v), same-parameter with an iso-curve of the surface.
fn uv_line(m: &mut Model, u: f64, v: f64, along_u: bool) -> arris_topo::Curve2Id {
    m.add_curve2(Curve2::Line {
        origin: Point2::new(u, v),
        direction: if along_u {
            Vec2::x_axis()
        } else {
            Vec2::y_axis()
        },
    })
}

/// The unit cube `[0, 1]³`: [`cuboid`] from the origin.
pub fn unit_box(m: &mut Model) -> Result<Body, SampleError> {
    cuboid(m, Point3::origin(), Point3::new(1.0, 1.0, 1.0))
}

/// The axis-aligned box from `min` to `max` as a solid: eight vertices,
/// twelve line edges, six planar faces of one loop each. Every plane's
/// `Z` is the face's outward normal and every face is used `Forward`; a
/// loop walks its corners counter-clockwise in the plane's (u, v), so
/// each edge is used twice in opposite directions. Tolerances are the
/// model's `default_tolerance`. Errors: an extent that is not finite and
/// positive. The model is untouched on error.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let b = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
/// assert_eq!(m.faces(b).unwrap().len(), 6);
/// assert_eq!(m.edges(b).unwrap().len(), 12);
/// ```
pub fn cuboid(m: &mut Model, min: Point3, max: Point3) -> Result<Body, SampleError> {
    cuboid_with(m, min, max, Flavour::Analytic)
}

/// [`cuboid`] with the same topology, pcurves and tolerances, but its
/// first edge's line stored as a degree-1 `Curve::Nurbs` over `[0, dx]`
/// and its bottom face's plane as a bilinear `Surface::Nurbs` over the
/// face's (u, v) rectangle — both exactly the analytic geometry at the
/// same parameter, so every check and comparison that passes on
/// [`cuboid`] must pass here through the B-spline arms. Errors as
/// [`cuboid`].
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let b = sample::cuboid_nurbs(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
/// let text = arris_debug::dump_text(&m, b).unwrap();
/// assert_eq!(text.matches("nurbs degree").count(), 2, "one curve, one surface");
/// ```
pub fn cuboid_nurbs(m: &mut Model, min: Point3, max: Point3) -> Result<Body, SampleError> {
    cuboid_with(m, min, max, Flavour::NurbsProbe)
}

fn cuboid_with(
    m: &mut Model,
    min: Point3,
    max: Point3,
    flavour: Flavour,
) -> Result<Body, SampleError> {
    let dx = positive("x extent", max.x - min.x)?;
    let dy = positive("y extent", max.y - min.y)?;
    let dz = positive("z extent", max.z - min.z)?;
    finite_point("min", min)?;
    let tol = m.precision().default_tolerance;
    // Corner `i` has the bits (x, y, z) of `i`.
    let corner = |i: usize| {
        Point3::new(
            if i & 1 == 0 { min.x } else { min.x + dx },
            if i & 2 == 0 { min.y } else { min.y + dy },
            if i & 4 == 0 { min.z } else { min.z + dz },
        )
    };
    // Twelve edges, x-parallel first, then y, then z, each from the lower
    // corner to the upper.
    let ends: [(usize, usize); 12] = [
        (0, 1),
        (2, 3),
        (4, 5),
        (6, 7),
        (0, 2),
        (1, 3),
        (4, 6),
        (5, 7),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    // Six faces as corner cycles, counter-clockwise seen from outside,
    // with the outward normal: bottom, top, front, back, left, right.
    let cycles: [([usize; 4], Vec3); 6] = [
        ([0, 2, 3, 1], -Vec3::z()),
        ([4, 5, 7, 6], Vec3::z()),
        ([0, 1, 5, 4], -Vec3::y()),
        ([2, 6, 7, 3], Vec3::y()),
        ([0, 4, 6, 2], -Vec3::x()),
        ([1, 3, 7, 5], Vec3::x()),
    ];
    // Every frame first, so a frame error leaves the model untouched.
    let mut frames = Vec::with_capacity(6);
    for (cycle, normal) in &cycles {
        let o = corner(cycle[0]);
        let x_hint = corner(cycle[1]) - o;
        frames.push(Frame::new(o, *normal, x_hint)?);
    }

    // Every geometry value that can fail is built before the first append,
    // so an error leaves the model untouched.
    let mut curves = Vec::with_capacity(12);
    for (i, &(a, b)) in ends.iter().enumerate() {
        let (pa, pb) = (corner(a), corner(b));
        let d = pb - pa;
        let length = d.norm();
        curves.push(if flavour == Flavour::NurbsProbe && i == 0 {
            Curve::Nurbs(NurbsCurve::new(
                1,
                vec![0.0, 0.0, length, length],
                vec![pa, pb],
                vec![1.0, 1.0],
            )?)
        } else {
            Curve::Line {
                origin: pa,
                direction: UnitVec3::new_normalize(d),
            }
        });
    }
    let mut surfaces = Vec::with_capacity(6);
    for (i, ((cycle, _), frame)) in cycles.iter().zip(&frames).enumerate() {
        surfaces.push(if flavour == Flavour::NurbsProbe && i == 0 {
            // The face's rectangle in its own (u, v): u along the first
            // edge of the cycle, v along the last, both from the origin.
            let o = corner(cycle[0]);
            let extent_u = (corner(cycle[1]) - o).norm();
            let extent_v = (corner(cycle[3]) - o).norm();
            let at = |u: f64, v: f64| frame.to_world(Point3::new(u, v, 0.0));
            Surface::Nurbs(NurbsSurface::new(
                [1, 1],
                [
                    vec![0.0, 0.0, extent_u, extent_u],
                    vec![0.0, 0.0, extent_v, extent_v],
                ],
                vec![
                    at(0.0, 0.0),
                    at(0.0, extent_v),
                    at(extent_u, 0.0),
                    at(extent_u, extent_v),
                ],
                vec![1.0; 4],
            )?)
        } else {
            Surface::Plane { frame: *frame }
        });
    }

    let vertices: Vec<_> = (0..8)
        .map(|i| m.raw().add_vertex(Vertex::new(corner(i), tol)))
        .collect();
    let mut edges = Vec::with_capacity(12);
    for (&(a, b), curve) in ends.iter().zip(curves) {
        let (pa, pb) = (corner(a), corner(b));
        let length = (pb - pa).norm();
        let curve = m.add_curve(curve);
        let range = Interval::new(0.0, length).map_err(|_| SampleError::Extent {
            name: "edge length",
            value: length,
        })?;
        edges.push(m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            vertices[a],
            vertices[b],
            tol,
        )));
    }
    let mut faces = Vec::with_capacity(6);
    for (((cycle, _), frame), surface) in cycles.iter().zip(&frames).zip(surfaces) {
        let surface = m.add_surface(surface);
        let mut coedges = Vec::with_capacity(4);
        for k in 0..4 {
            let (a, b) = (cycle[k], cycle[(k + 1) % 4]);
            let ei = ends
                .iter()
                .position(|&(p, q)| (p, q) == (a, b) || (p, q) == (b, a))
                .ok_or(SampleError::Extent {
                    name: "box edge table",
                    value: k as f64,
                })?;
            let (ea, eb) = ends[ei];
            let orientation = if ea == a {
                Orientation::Forward
            } else {
                Orientation::Reversed
            };
            // Same-parameter: the pcurve starts where the edge's own start
            // maps into the plane and runs along the edge's own direction.
            let start = frame.to_local(corner(ea));
            let dir = frame.vec_to_local(corner(eb) - corner(ea));
            let pcurve = m.add_curve2(Curve2::Line {
                origin: arris_math::Point2::new(start.x, start.y),
                direction: UnitVec2::new_normalize(arris_math::Vec2::new(dir.x, dir.y)),
            });
            coedges.push(Coedge::new(edges[ei], orientation, pcurve));
        }
        faces.push(
            m.raw()
                .add_face(Face::new(surface, vec![Loop::new(coedges)], tol)),
        );
    }
    let shell = m.raw().add_shell(Shell::new(
        faces.iter().map(|&f| FaceHandle::forward(f)).collect(),
    ));
    let body = m
        .raw()
        .add_body(BodyEntity::solid(vec![ShellHandle::forward(shell)]));
    Ok(Body::forward(body))
}

/// A cylinder of `radius` and `height` on the `z` axis with its base at
/// the origin, as a solid: two vertices on the seam, a bottom circle, the
/// seam line, a top circle; one wall face on the cylinder surface whose
/// one loop is bottom circle, seam up, top circle, seam down — the two
/// seam pcurves at `u = 2π` and `u = 0` (`docs/DATA-MODEL.md` §Seams);
/// two cap faces on planes whose `Z` is the axis, the bottom cap used
/// `Reversed`, as the reference tree's one-axis primitive builds it, so
/// both caps' circle pcurves are right-handed (the circle's `Z` is along
/// each plane's normal). The seam is where the surface's `X` points.
/// Tolerances are the model's `default_tolerance`. Errors: a radius or
/// height that is not finite and positive. The model is untouched on
/// error.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let b = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// assert_eq!(m.faces(b).unwrap().len(), 3);
/// assert_eq!(m.edges(b).unwrap().len(), 3);
/// assert_eq!(m.vertices(b).unwrap().len(), 2);
/// ```
pub fn cylinder(m: &mut Model, radius: f64, height: f64) -> Result<Body, SampleError> {
    let radius = positive("radius", radius)?;
    let height = positive("height", height)?;
    let tol = m.precision().default_tolerance;
    let base = Frame::world();
    let top = base.with_origin(Point3::new(0.0, 0.0, height));
    let axis = Vec3::z_axis();

    let wall = m.add_surface(Surface::Cylinder {
        frame: base,
        radius,
    });
    let bottom_plane = m.add_surface(Surface::Plane { frame: base });
    let top_plane = m.add_surface(Surface::Plane { frame: top });

    let bottom_circle = m.add_curve(Curve::Circle {
        frame: base,
        radius,
    });
    let seam_line = m.add_curve(Curve::Line {
        origin: Point3::new(radius, 0.0, 0.0),
        direction: axis,
    });
    let top_circle = m.add_curve(Curve::Circle { frame: top, radius });

    let v0 = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(radius, 0.0, 0.0), tol));
    let v1 = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(radius, 0.0, height), tol));

    let turn = Interval::TURN;
    let rise = Interval::new(0.0, height).map_err(|_| SampleError::Extent {
        name: "height",
        value: height,
    })?;
    let e_bottom = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: bottom_circle,
            range: turn,
        },
        v0,
        v0,
        tol,
    ));
    let e_seam = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: seam_line,
            range: rise,
        },
        v0,
        v1,
        tol,
    ));
    let e_top = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: top_circle,
            range: turn,
        },
        v1,
        v1,
        tol,
    ));

    // The wall: (0, 0) → (2π, 0) → (2π, h) → (0, h), counter-clockwise in
    // (u, v) with the outward normal up.
    let p_bottom = uv_line(m, 0.0, 0.0, true);
    let p_seam_up = uv_line(m, TAU, 0.0, false);
    let p_top = uv_line(m, 0.0, height, true);
    let p_seam_down = uv_line(m, 0.0, 0.0, false);
    let wall_face = m.raw().add_face(Face::new(
        wall,
        vec![Loop::new(vec![
            Coedge::new(e_bottom, Orientation::Forward, p_bottom),
            Coedge::new(e_seam, Orientation::Forward, p_seam_up),
            Coedge::new(e_top, Orientation::Reversed, p_top),
            Coedge::new(e_seam, Orientation::Reversed, p_seam_down),
        ])],
        tol,
    ));
    // The caps: each circle is the plane's own (u, v) circle at the same
    // parameter, right-handed since the circle's Z is the plane's normal.
    let cap_circle = |m: &mut Model| {
        m.add_curve2(Curve2::Circle {
            frame: Frame2::identity(),
            radius,
        })
    };
    let p_bottom_cap = cap_circle(m);
    let bottom_face = m.raw().add_face(Face::new(
        bottom_plane,
        vec![Loop::new(vec![Coedge::new(
            e_bottom,
            Orientation::Forward,
            p_bottom_cap,
        )])],
        tol,
    ));
    let p_top_cap = cap_circle(m);
    let top_face = m.raw().add_face(Face::new(
        top_plane,
        vec![Loop::new(vec![Coedge::new(
            e_top,
            Orientation::Forward,
            p_top_cap,
        )])],
        tol,
    ));

    let shell = m.raw().add_shell(Shell::new(vec![
        FaceHandle::forward(wall_face),
        FaceHandle::new(bottom_face, Orientation::Reversed),
        FaceHandle::forward(top_face),
    ]));
    let body = m
        .raw()
        .add_body(BodyEntity::solid(vec![ShellHandle::forward(shell)]));
    Ok(Body::forward(body))
}

/// The rectangular frame of `boolean/frame-cut`: the box from `min` to
/// `max` with the window `[window_min, window_max]` in (x, y) cut through
/// its full height — sixteen vertices, twenty-four line edges, ten planar
/// faces of which the top and the bottom have two loops each, genus 1.
/// Built through the Euler operators (`docs/DATA-MODEL.md` §Euler
/// operators): Mäntylä's box recipe, then a bridge strut into the top, the
/// window's rim as struts closed by `mef` into a plug on the *bottom's*
/// plane, `kemr` on the bridge to make the rim a ring, struts down from
/// the rim, `mef` for each inner wall, and `kfmrh` to open the plug into
/// the bottom; every pcurve is given afterwards from each edge's line in
/// each face's plane. Every plane's `Z` is the face's outward normal and
/// every face is used `Forward`. Tolerances are the model's
/// `default_tolerance`. Errors: an extent that is not finite and positive,
/// a window not strictly inside the box in (x, y). The model is untouched
/// on error.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_math::{Point2, Point3};
///
/// let mut m = Model::default();
/// let b = sample::frame(
///     &mut m,
///     Point3::origin(),
///     Point3::new(40.0, 30.0, 10.0),
///     Point2::new(10.0, 10.0),
///     Point2::new(30.0, 20.0),
/// )
/// .unwrap();
/// assert_eq!(m.faces(b).unwrap().len(), 10);
/// assert_eq!(m.edges(b).unwrap().len(), 24);
/// assert!(arris_debug::euler_line(&m, b).unwrap().ends_with("g1 = 0"));
/// ```
pub fn frame(
    m: &mut Model,
    min: Point3,
    max: Point3,
    window_min: Point2,
    window_max: Point2,
) -> Result<Body, SampleError> {
    positive("x extent", max.x - min.x)?;
    positive("y extent", max.y - min.y)?;
    positive("z extent", max.z - min.z)?;
    positive("window x extent", window_max.x - window_min.x)?;
    positive("window y extent", window_max.y - window_min.y)?;
    for (name, value) in [
        ("window min x", window_min.x - min.x),
        ("window min y", window_min.y - min.y),
        ("window max x", max.x - window_max.x),
        ("window max y", max.y - window_max.y),
    ] {
        positive(name, value)?;
    }
    if !min.coords.iter().all(|c| c.is_finite()) {
        return Err(SampleError::Extent {
            name: "min",
            value: f64::NAN,
        });
    }
    let tol = m.precision().default_tolerance;
    // Outer corners counter-clockwise from above, bottom then top; the
    // window's the same way, top (rim) then bottom (floor).
    let b = [
        Point3::new(min.x, min.y, min.z),
        Point3::new(max.x, min.y, min.z),
        Point3::new(max.x, max.y, min.z),
        Point3::new(min.x, max.y, min.z),
    ];
    let t: [Point3; 4] = core::array::from_fn(|i| Point3::new(b[i].x, b[i].y, max.z));
    let h = [
        Point3::new(window_min.x, window_min.y, max.z),
        Point3::new(window_max.x, window_min.y, max.z),
        Point3::new(window_max.x, window_max.y, max.z),
        Point3::new(window_min.x, window_max.y, max.z),
    ];
    let g: [Point3; 4] = core::array::from_fn(|i| Point3::new(h[i].x, h[i].y, min.z));
    // Planes: bottom, top, the four outer sides, the four window walls;
    // every Z the outward normal.
    let mut frames = Vec::with_capacity(10);
    frames.push(Frame::new(b[0], -Vec3::z(), Vec3::x())?);
    frames.push(Frame::new(t[0], Vec3::z(), Vec3::x())?);
    let side_normals = [-Vec3::y(), Vec3::x(), Vec3::y(), -Vec3::x()];
    for i in 0..4 {
        frames.push(Frame::new(b[i], side_normals[i], b[(i + 1) % 4] - b[i])?);
    }
    let wall_normals = [Vec3::y(), -Vec3::x(), -Vec3::y(), Vec3::x()];
    for i in 0..4 {
        frames.push(Frame::new(g[i], wall_normals[i], g[(i + 1) % 4] - g[i])?);
    }
    m.transaction(|m| {
        let surfaces: Vec<_> = frames
            .iter()
            .map(|&frame| m.add_surface(Surface::Plane { frame }))
            .collect();
        let (s_bottom, s_top) = (surfaces[0], surfaces[1]);
        let line = |m: &mut Model, p: Point3, q: Point3| -> Result<EdgeGeometry, SampleError> {
            let d = q - p;
            let length = d.norm();
            let curve = m.add_curve(Curve::Line {
                origin: p,
                direction: UnitVec3::new_normalize(d),
            });
            let range = Interval::new(0.0, length).map_err(|_| SampleError::Extent {
                name: "edge length",
                value: length,
            })?;
            Ok(EdgeGeometry::Curve { curve, range })
        };
        let strut = |m: &mut Model, p: Point3, q: Point3| -> Result<Strut, SampleError> {
            Ok(Strut {
                point: q,
                geometry: line(m, p, q)?,
                pcurves: [None, None],
            })
        };
        let split = |m: &mut Model,
                     p: Point3,
                     q: Point3,
                     surface,
                     orientation|
         -> Result<Split, SampleError> {
            Ok(Split {
                geometry: line(m, p, q)?,
                surface,
                orientation,
                pcurves: [None, None],
            })
        };
        let mut bd = Builder::new(tol);
        // The bottom's rectangle: three struts and a closing mef whose new
        // face is the lid that becomes the top.
        let (v0, f_bottom) = bd.mvfs(Seed {
            point: b[0],
            surface: s_bottom,
            orientation: Orientation::Forward,
        })?;
        let mut corners = vec![v0];
        for i in 1..4 {
            let at = bd.find_position(f_bottom, 0, corners[i - 1])?;
            let (v, _) = bd.mev(at, strut(m, b[i - 1], b[i])?)?;
            corners.push(v);
        }
        let from = bd.find_position(f_bottom, 0, corners[3])?;
        let to = bd.find_position(f_bottom, 0, corners[0])?;
        let (_, f_top) = bd.mef(from, to, split(m, b[3], b[0], s_top, Orientation::Forward)?)?;
        // Struts up from the lid's corners, then the four sides.
        let mut tops = Vec::with_capacity(4);
        for i in 0..4 {
            let at = bd.find_position(f_top, 0, corners[i])?;
            let (v, _) = bd.mev(at, strut(m, b[i], t[i])?)?;
            tops.push(v);
        }
        for i in 0..4 {
            let from = bd.find_position(f_top, 0, tops[(i + 1) % 4])?;
            let to = bd.find_position(f_top, 0, tops[i])?;
            bd.mef(
                from,
                to,
                split(
                    m,
                    t[(i + 1) % 4],
                    t[i],
                    surfaces[2 + i],
                    Orientation::Forward,
                )?,
            )?;
        }
        // The window: a bridge into the top, the rim as struts, the plug
        // on the bottom's plane facing up, the bridge cut into a ring.
        let at = bd.find_position(f_top, 0, tops[0])?;
        let (h0, bridge) = bd.mev(at, strut(m, t[0], h[0])?)?;
        let mut rim = vec![h0];
        for i in 1..4 {
            let at = bd.find_position(f_top, 0, rim[i - 1])?;
            let (v, _) = bd.mev(at, strut(m, h[i - 1], h[i])?)?;
            rim.push(v);
        }
        let from = bd.find_position(f_top, 0, rim[3])?;
        let to = match bd.find_position(f_top, 0, rim[0]) {
            Err(BuildError::Ambiguous { positions, .. }) => Position::new(f_top, 0, positions[0]),
            Ok(p) => p,
            Err(e) => return Err(e.into()),
        };
        let (_, plug) = bd.mef(
            from,
            to,
            split(m, h[3], h[0], s_bottom, Orientation::Reversed)?,
        )?;
        bd.kemr(bridge)?;
        // Struts down from the rim, the four walls, and the floor opened
        // into the bottom.
        let mut floor = Vec::with_capacity(4);
        for i in 0..4 {
            let at = bd.find_position(plug, 0, rim[i])?;
            let (v, _) = bd.mev(at, strut(m, h[i], g[i])?)?;
            floor.push(v);
        }
        for i in 0..4 {
            let from = bd.find_position(plug, 0, floor[(i + 1) % 4])?;
            let to = bd.find_position(plug, 0, floor[i])?;
            bd.mef(
                from,
                to,
                split(
                    m,
                    g[(i + 1) % 4],
                    g[i],
                    surfaces[6 + i],
                    Orientation::Forward,
                )?,
            )?;
        }
        bd.kfmrh(plug, f_bottom)?;
        // Every pcurve: the edge's line in the face's plane, at the edge's
        // own parameter.
        let plane_of = |surface| frames[surfaces.iter().position(|&s| s == surface).unwrap_or(0)];
        let mut wanted: Vec<(Position, Point3, Point3, Frame)> = Vec::new();
        for (f, face) in bd.faces() {
            let frame = plane_of(face.surface());
            for (li, lp) in face.loops().iter().enumerate() {
                for (ci, u) in lp.uses().iter().enumerate() {
                    let e = bd.edge(u.edge)?;
                    let p = bd.vertex(e.start())?.point();
                    let q = bd.vertex(e.end())?.point();
                    wanted.push((Position::new(f, li, ci), p, q, frame));
                }
            }
        }
        for (at, p, q, frame) in wanted {
            let origin = frame.to_local(p);
            let direction = frame.vec_to_local(q - p);
            let pcurve = m.add_curve2(Curve2::Line {
                origin: Point2::new(origin.x, origin.y),
                direction: UnitVec2::new_normalize(Vec2::new(direction.x, direction.y)),
            });
            bd.set_pcurve(at, pcurve)?;
        }
        let _: FaceRef = f_bottom;
        Ok(bd.finish(m, BodyKind::Solid)?.body)
    })
}

/// A sphere of `radius` centred at `centre` as a solid: one face on the
/// sphere surface whose single loop walks the (u, v) rectangle
/// `[0, 2π] × [−π/2, π/2]` counter-clockwise — the south pole's
/// degenerate edge along `v = −π/2`, the seam meridian up at `u = 2π`,
/// the north pole's degenerate edge along `v = π/2`, the seam meridian
/// down at `u = 0` (`docs/DATA-MODEL.md` §Seams) — two vertices at the
/// poles, one meridian edge used twice and one degenerate edge per pole,
/// the first sample body with E6 edges. The meridian is parametrised by
/// `t = v + π/2`, so it runs from the south pole at `t = 0` to the north
/// at `t = π` and its pcurves are same-parameter with it. Tolerances are
/// the model's `default_tolerance`. Errors: a radius or a centre that is
/// not finite and positive. The model is untouched on error.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let b = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
/// assert_eq!(m.faces(b).unwrap().len(), 1);
/// assert_eq!(m.edges(b).unwrap().len(), 3);
/// assert_eq!(m.vertices(b).unwrap().len(), 2);
/// use arris_check::{Level, check};
/// assert!(check(&m, b, Level::Fast).is_ok());
/// ```
pub fn sphere(m: &mut Model, centre: Point3, radius: f64) -> Result<Body, SampleError> {
    let radius = positive("radius", radius)?;
    finite_point("centre", centre)?;
    let tol = m.precision().default_tolerance;
    let base = Frame::world().with_origin(centre);
    // The seam meridian at `u = 0`: its `X` is the surface's `−Z` and its
    // `Y` the surface's `X`, so `P(t) = C − R cos t·Z + R sin t·X` is the
    // surface at `(0, t − π/2)`.
    let meridian_frame = Frame::from_orthonormal(centre, -Vec3::z(), Vec3::x(), -Vec3::y())?;
    let half_turn = Interval::new(0.0, PI).map_err(|_| SampleError::Extent {
        name: "half turn",
        value: PI,
    })?;

    let surface = m.add_surface(Surface::Sphere {
        frame: base,
        radius,
    });
    let meridian = m.add_curve(Curve::Circle {
        frame: meridian_frame,
        radius,
    });
    let south = m.raw().add_vertex(Vertex::new(
        Point3::new(centre.x, centre.y, centre.z - radius),
        tol,
    ));
    let north = m.raw().add_vertex(Vertex::new(
        Point3::new(centre.x, centre.y, centre.z + radius),
        tol,
    ));
    let e_seam = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: meridian,
            range: half_turn,
        },
        south,
        north,
        tol,
    ));
    let e_south = m.raw().add_edge(Edge::new(
        EdgeGeometry::Degenerate {
            range: Interval::TURN,
        },
        south,
        south,
        tol,
    ));
    let e_north = m.raw().add_edge(Edge::new(
        EdgeGeometry::Degenerate {
            range: Interval::TURN,
        },
        north,
        north,
        tol,
    ));
    let p_south = uv_line(m, 0.0, -FRAC_PI_2, true);
    let p_seam_up = uv_line(m, TAU, -FRAC_PI_2, false);
    let p_north = uv_line(m, 0.0, FRAC_PI_2, true);
    let p_seam_down = uv_line(m, 0.0, -FRAC_PI_2, false);
    let face = m.raw().add_face(Face::new(
        surface,
        vec![Loop::new(vec![
            Coedge::new(e_south, Orientation::Forward, p_south),
            Coedge::new(e_seam, Orientation::Forward, p_seam_up),
            Coedge::new(e_north, Orientation::Reversed, p_north),
            Coedge::new(e_seam, Orientation::Reversed, p_seam_down),
        ])],
        tol,
    ));
    let shell = m
        .raw()
        .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let body = m
        .raw()
        .add_body(BodyEntity::solid(vec![ShellHandle::forward(shell)]));
    Ok(Body::forward(body))
}

/// A torus of `major_radius` and `minor_radius` about the `z` axis at
/// `centre` as a solid: one face on the torus surface whose single loop
/// walks the (u, v) square `[0, 2π]²` counter-clockwise, one vertex at
/// `(u, v) = (0, 0)`, and two seam edges — the `v = 0` circle of radius
/// `R + r` used at `v = 0` and `v = 2π`, the `u = 0` tube circle of
/// radius `r` used at `u = 0` and `u = 2π` — so the body is genus 1 and
/// has no degenerate edge. Both circles are parametrised by the
/// parameter their pcurves run along. Tolerances are the model's
/// `default_tolerance`. Errors: a radius that is not finite and
/// positive, or a minor radius not smaller than the major one. The model
/// is untouched on error.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let b = sample::torus(&mut m, Point3::origin(), 5.0, 2.0).unwrap();
/// assert_eq!(m.faces(b).unwrap().len(), 1);
/// assert_eq!(m.edges(b).unwrap().len(), 2);
/// assert!(arris_debug::euler_line(&m, b).unwrap().contains("g1 = 0"));
/// ```
pub fn torus(
    m: &mut Model,
    centre: Point3,
    major_radius: f64,
    minor_radius: f64,
) -> Result<Body, SampleError> {
    let major_radius = positive("major radius", major_radius)?;
    let minor_radius = positive("minor radius", minor_radius)?;
    finite_point("centre", centre)?;
    if minor_radius >= major_radius {
        return Err(SampleError::Extent {
            name: "minor radius",
            value: minor_radius,
        });
    }
    let tol = m.precision().default_tolerance;
    let base = Frame::world().with_origin(centre);
    // The tube circle at `u = 0`: centred on the centre circle, its `X`
    // the surface's `X` and its `Y` the surface's `Z`, so `P(v)` is the
    // surface at `(0, v)`.
    let tube_frame = Frame::from_orthonormal(
        Point3::new(centre.x + major_radius, centre.y, centre.z),
        Vec3::x(),
        Vec3::z(),
        -Vec3::y(),
    )?;

    let surface = m.add_surface(Surface::Torus {
        frame: base,
        major_radius,
        minor_radius,
    });
    let outer = m.add_curve(Curve::Circle {
        frame: base,
        radius: major_radius + minor_radius,
    });
    let tube = m.add_curve(Curve::Circle {
        frame: tube_frame,
        radius: minor_radius,
    });
    let corner = m.raw().add_vertex(Vertex::new(
        Point3::new(centre.x + major_radius + minor_radius, centre.y, centre.z),
        tol,
    ));
    let e_outer = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: outer,
            range: Interval::TURN,
        },
        corner,
        corner,
        tol,
    ));
    let e_tube = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: tube,
            range: Interval::TURN,
        },
        corner,
        corner,
        tol,
    ));
    let p_outer_low = uv_line(m, 0.0, 0.0, true);
    let p_tube_high = uv_line(m, TAU, 0.0, false);
    let p_outer_high = uv_line(m, 0.0, TAU, true);
    let p_tube_low = uv_line(m, 0.0, 0.0, false);
    let face = m.raw().add_face(Face::new(
        surface,
        vec![Loop::new(vec![
            Coedge::new(e_outer, Orientation::Forward, p_outer_low),
            Coedge::new(e_tube, Orientation::Forward, p_tube_high),
            Coedge::new(e_outer, Orientation::Reversed, p_outer_high),
            Coedge::new(e_tube, Orientation::Reversed, p_tube_low),
        ])],
        tol,
    ));
    let shell = m
        .raw()
        .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let body = m
        .raw()
        .add_body(BodyEntity::solid(vec![ShellHandle::forward(shell)]));
    Ok(Body::forward(body))
}

/// A rectangular patch of `surface` over `u × v` as a `Sheet` body: one
/// face used `Forward`, four vertices at the rectangle's corners, four
/// edges whose 3D curves are the surface's own iso-curves — a line, a
/// circle, or a boundary row of a clamped NURBS net — each parametrised
/// by the (u, v) parameter its pcurve runs along, so every edge is
/// same-parameter by construction, and pcurves the rectangle's four
/// sides walked counter-clockwise. What the tessellation's property
/// tests mesh: one face of every surface kind over a region a caller
/// chooses. Tolerances are the model's `default_tolerance`.
///
/// Errors: a range that is not finite or has no length; a range that
/// spans a whole period, so the two sides would coincide; a region that
/// reaches a singularity — a sphere's pole, a cone's apex; a NURBS
/// region that is not the surface's whole domain, or a NURBS that is not
/// clamped, since only there is a boundary row an exact iso-curve. The
/// model is untouched on error.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_geom::Surface;
/// use arris_math::{Frame, Interval};
///
/// let mut m = Model::default();
/// let sphere = Surface::Sphere { frame: Frame::world(), radius: 2.0 };
/// let u = Interval::new(0.2, 1.4).unwrap();
/// let v = Interval::new(-0.5, 0.9).unwrap();
/// let b = sample::patch(&mut m, sphere, u, v).unwrap();
/// assert_eq!(m.faces(b).unwrap().len(), 1);
/// assert_eq!(m.edges(b).unwrap().len(), 4);
/// use arris_check::{Level, check};
/// assert!(check(&m, b, Level::Fast).is_ok());
/// ```
pub fn patch(
    m: &mut Model,
    surface: Surface,
    u: Interval,
    v: Interval,
) -> Result<Body, SampleError> {
    let periods = surface.period();
    for (name, range, period) in [("u range", u, periods[0]), ("v range", v, periods[1])] {
        if !(range.lo().is_finite() && range.hi().is_finite() && range.length() > 0.0) {
            return Err(SampleError::Extent {
                name,
                value: range.length(),
            });
        }
        if period.is_some_and(|p| range.length() >= p) {
            return Err(SampleError::Region("a patch spans a whole period"));
        }
    }
    let tol = m.precision().default_tolerance;
    // Every curve first, so a singular region leaves the model untouched.
    let curves = [
        iso_u(&surface, v.lo())?,
        iso_v(&surface, u.hi())?,
        iso_u(&surface, v.hi())?,
        iso_v(&surface, u.lo())?,
    ];
    // The corners in walking order: (u₀, v₀), (u₁, v₀), (u₁, v₁), (u₀, v₁).
    let corners = [
        (u.lo(), v.lo()),
        (u.hi(), v.lo()),
        (u.hi(), v.hi()),
        (u.lo(), v.hi()),
    ];
    let points: Vec<Point3> = corners
        .iter()
        .map(|&(a, b)| surface.point(a, b))
        .collect::<Vec<_>>();
    for p in &points {
        finite_point("patch corner", *p)?;
    }

    let surface_id = m.add_surface(surface);
    let vertices: Vec<_> = points
        .iter()
        .map(|&p| m.raw().add_vertex(Vertex::new(p, tol)))
        .collect();
    // Each side from the corner its curve's parameter starts at: the two
    // `u` sides run from `u₀`, the two `v` sides from `v₀`.
    let ends = [(0, 1), (1, 2), (3, 2), (0, 3)];
    let ranges = [u, v, u, v];
    let mut edges = Vec::with_capacity(4);
    for ((curve, (a, b)), range) in curves.into_iter().zip(ends).zip(ranges) {
        let curve = m.add_curve(curve);
        edges.push(m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            vertices[a],
            vertices[b],
            tol,
        )));
    }
    let pcurves = [
        uv_line(m, 0.0, v.lo(), true),
        uv_line(m, u.hi(), 0.0, false),
        uv_line(m, 0.0, v.hi(), true),
        uv_line(m, u.lo(), 0.0, false),
    ];
    let orientations = [
        Orientation::Forward,
        Orientation::Forward,
        Orientation::Reversed,
        Orientation::Reversed,
    ];
    let coedges: Vec<Coedge> = (0..4)
        .map(|k| Coedge::new(edges[k], orientations[k], pcurves[k]))
        .collect();
    let face = m
        .raw()
        .add_face(Face::new(surface_id, vec![Loop::new(coedges)], tol));
    let shell = m
        .raw()
        .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let body = m.raw().add_body(BodyEntity::new(
        BodyKind::Sheet,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    Ok(Body::forward(body))
}

/// The surface's iso-curve along `u` at the fixed `v`, parametrised by
/// `u` itself.
fn iso_u(surface: &Surface, v: f64) -> Result<Curve, SampleError> {
    Ok(match *surface {
        Surface::Plane { frame } => Curve::Line {
            origin: frame.to_world(Point3::new(0.0, v, 0.0)),
            direction: frame.x(),
        },
        Surface::Cylinder { frame, radius } => Curve::Circle {
            frame: frame.with_origin(frame.to_world(Point3::new(0.0, 0.0, v))),
            radius,
        },
        Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        } => Curve::Ellipse {
            frame: frame.with_origin(frame.to_world(Point3::new(0.0, 0.0, v))),
            major_radius,
            minor_radius,
        },
        Surface::Cone {
            frame,
            radius,
            half_angle,
        } => {
            let (sa, ca) = half_angle.sin_cos();
            Curve::Circle {
                frame: frame.with_origin(frame.to_world(Point3::new(0.0, 0.0, v * ca))),
                radius: positive_radius(radius + v * sa, "a cone patch reaches its apex")?,
            }
        }
        Surface::Sphere { frame, radius } => {
            let (sv, cv) = v.sin_cos();
            Curve::Circle {
                frame: frame.with_origin(frame.to_world(Point3::new(0.0, 0.0, radius * sv))),
                radius: positive_radius(radius * cv, "a sphere patch reaches a pole")?,
            }
        }
        Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => {
            let (sv, cv) = v.sin_cos();
            Curve::Circle {
                frame: frame.with_origin(frame.to_world(Point3::new(0.0, 0.0, minor_radius * sv))),
                radius: positive_radius(
                    major_radius + minor_radius * cv,
                    "a torus patch reaches its axis",
                )?,
            }
        }
        Surface::Nurbs(ref s) => nurbs_boundary(s, 1, v)?,
    })
}

/// The surface's iso-curve along `v` at the fixed `u`, parametrised by
/// `v` itself.
fn iso_v(surface: &Surface, u: f64) -> Result<Curve, SampleError> {
    let (su, cu) = u.sin_cos();
    Ok(match *surface {
        Surface::Plane { frame } => Curve::Line {
            origin: frame.to_world(Point3::new(u, 0.0, 0.0)),
            direction: frame.y(),
        },
        Surface::Cylinder { frame, radius } => Curve::Line {
            origin: frame.to_world(Point3::new(radius * cu, radius * su, 0.0)),
            direction: frame.z(),
        },
        Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        } => Curve::Line {
            origin: frame.to_world(Point3::new(major_radius * cu, minor_radius * su, 0.0)),
            direction: frame.z(),
        },
        Surface::Cone {
            frame,
            radius,
            half_angle,
        } => {
            let (sa, ca) = half_angle.sin_cos();
            let radial = frame.vec_to_world(Vec3::new(cu, su, 0.0));
            Curve::Line {
                origin: frame.to_world(Point3::new(radius * cu, radius * su, 0.0)),
                direction: UnitVec3::new_normalize(sa * radial + ca * frame.z().into_inner()),
            }
        }
        Surface::Sphere { frame, radius } => Curve::Circle {
            frame: meridian_frame(&frame, cu, su)?,
            radius,
        },
        Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => {
            let centre = frame.to_world(Point3::new(major_radius * cu, major_radius * su, 0.0));
            Curve::Circle {
                frame: meridian_frame(&frame.with_origin(centre), cu, su)?,
                radius: minor_radius,
            }
        }
        Surface::Nurbs(ref s) => nurbs_boundary(s, 0, u)?,
    })
}

/// The frame of a circle through the surface's axis at the angle whose
/// cosine and sine are `cu` and `su`: `X` radially outward, `Y` the
/// surface's `Z`, so `P(v)` is the surface at `(u, v)` around it.
fn meridian_frame(frame: &Frame, cu: f64, su: f64) -> Result<Frame, SampleError> {
    let radial = frame.vec_to_world(Vec3::new(cu, su, 0.0));
    let axis = frame.z().into_inner();
    Ok(Frame::from_orthonormal(
        frame.origin(),
        radial,
        axis,
        radial.cross(&axis),
    )?)
}

fn positive_radius(value: f64, reason: &'static str) -> Result<f64, SampleError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(SampleError::Region(reason))
    }
}

/// The boundary iso-curve of a clamped NURBS surface: `fixed` is the
/// direction held at `at`, which must be one of that direction's domain
/// bounds, and the curve runs along the other direction over the
/// boundary row or column of the net.
fn nurbs_boundary(s: &NurbsSurface, fixed: usize, at: f64) -> Result<Curve, SampleError> {
    let counts = s.counts();
    let degree = s.degree()[fixed];
    let knots = s.knots()[fixed];
    let clamped = knots[..=degree].iter().all(|&k| k == knots[0])
        && knots[knots.len() - degree - 1..]
            .iter()
            .all(|&k| k == knots[knots.len() - 1]);
    if !clamped {
        return Err(SampleError::Region(
            "a NURBS patch needs a surface clamped in both directions",
        ));
    }
    let domain = s.domain()[fixed];
    let index = if at == domain.lo() {
        0
    } else if at == domain.hi() {
        counts[fixed] - 1
    } else {
        return Err(SampleError::Region(
            "a NURBS patch runs over the surface's whole domain",
        ));
    };
    let along = 1 - fixed;
    let mut points = Vec::with_capacity(counts[along]);
    let mut weights = Vec::with_capacity(counts[along]);
    for k in 0..counts[along] {
        let (i, j) = if fixed == 0 { (index, k) } else { (k, index) };
        points.push(
            s.control_point(i, j)
                .ok_or(SampleError::Region("a NURBS net index is out of range"))?,
        );
        weights.push(s.weights()[i * counts[1] + j]);
    }
    Ok(Curve::Nurbs(NurbsCurve::new(
        s.degree()[along],
        s.knots()[along].to_vec(),
        points,
        weights,
    )?))
}
