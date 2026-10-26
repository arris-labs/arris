//! Builders and report helpers the row files share: the raw-API rebuild of a sample body, the report as `(code, entity)` lines, and the plate.

use arris_check::Report;
use arris_debug::sample;
use arris_geom::{Curve, Curve2, NurbsSurface, Surface};
use arris_math::{Frame, Interval, Point2, Point3, UnitVec2, UnitVec3, Vec3};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Coedge, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, EdgeId, Face as FaceHandle, FaceId, Model, Orientation, Shell as ShellHandle, ShellId,
    SurfaceId, VertexId,
};
use std::collections::BTreeMap;

/// A copy of `body` appended through the raw API in iteration order, each
/// entity passed through its edit with its index in that order; references
/// are remapped to the copies, an id the edit made dangling is kept. The
/// new ids, by index.
pub struct Rebuilt {
    pub body: Body,
    pub vertices: Vec<VertexId>,
    pub edges: Vec<EdgeId>,
    pub faces: Vec<FaceId>,
}

pub fn rebuild(
    m: &mut Model,
    body: Body,
    vertex: &dyn Fn(usize, Vertex) -> Vertex,
    edge: &dyn Fn(usize, Edge) -> Edge,
    face: &dyn Fn(usize, Face) -> Face,
) -> Rebuilt {
    let old = m.body(body.id).unwrap().clone();
    let old_vertices = m.vertices(body).unwrap();
    let old_edges = m.edges(body).unwrap();
    let old_faces = m.faces(body).unwrap();
    let old_shells = m.shells(body).unwrap();

    let mut vmap = BTreeMap::new();
    let mut vertices = Vec::new();
    for (i, v) in old_vertices.iter().enumerate() {
        let v0 = *m.vertex(v.id).unwrap();
        let id = m.raw().add_vertex(vertex(i, v0));
        vmap.insert(v.id, id);
        vertices.push(id);
    }
    let mut emap = BTreeMap::new();
    let mut edges = Vec::new();
    for (i, e) in old_edges.iter().enumerate() {
        let e0 = *m.edge(e.id).unwrap();
        let remapped = Edge::new(
            e0.geometry(),
            vmap.get(&e0.start()).copied().unwrap_or(e0.start()),
            vmap.get(&e0.end()).copied().unwrap_or(e0.end()),
            e0.tolerance(),
        );
        let id = m.raw().add_edge(edge(i, remapped));
        emap.insert(e.id, id);
        edges.push(id);
    }
    let mut fmap = BTreeMap::new();
    let mut faces = Vec::new();
    for (i, f) in old_faces.iter().enumerate() {
        let f0 = m.face(f.id).unwrap().clone();
        let loops = f0
            .loops()
            .iter()
            .map(|l| {
                Loop::new(
                    l.coedges()
                        .iter()
                        .map(|c| {
                            Coedge::new(
                                emap.get(&c.edge()).copied().unwrap_or(c.edge()),
                                c.orientation(),
                                c.pcurve(),
                            )
                        })
                        .collect(),
                )
            })
            .collect();
        let id = m
            .raw()
            .add_face(face(i, Face::new(f0.surface(), loops, f0.tolerance())));
        fmap.insert(f.id, id);
        faces.push(id);
    }
    let mut smap = BTreeMap::new();
    for s in &old_shells {
        let s0 = m.shell(s.id).unwrap().clone();
        let uses = s0
            .faces()
            .iter()
            .map(|f| FaceHandle::new(fmap.get(&f.id).copied().unwrap_or(f.id), f.orientation))
            .collect();
        smap.insert(s.id, m.raw().add_shell(Shell::new(uses)));
    }
    let shells = old
        .shells()
        .iter()
        .map(|s| ShellHandle::new(smap.get(&s.id).copied().unwrap_or(s.id), s.orientation))
        .collect();
    let free_edges = old
        .free_edges()
        .iter()
        .map(|e| arris_topo::Edge::new(emap.get(&e.id).copied().unwrap_or(e.id), e.orientation))
        .collect();
    let free_vertices = old
        .free_vertices()
        .iter()
        .map(|v| vmap.get(v).copied().unwrap_or(*v))
        .collect();
    let id = m.raw().add_body(BodyEntity::new(
        old.kind(),
        shells,
        free_edges,
        free_vertices,
    ));
    Rebuilt {
        body: Body::new(id, body.orientation),
        vertices,
        edges,
        faces,
    }
}

pub fn keep_vertex(_: usize, v: Vertex) -> Vertex {
    v
}

pub fn keep_edge(_: usize, e: Edge) -> Edge {
    e
}

pub fn keep_face(_: usize, f: Face) -> Face {
    f
}

/// `(code, entity)` per line of the report.
pub fn lines(report: &Report) -> Vec<(String, String)> {
    report
        .violations()
        .iter()
        .map(|v| (v.code().to_string(), v.entity().to_string()))
        .collect()
}

/// The report holds exactly `expected` as `(code, entity)` lines, in
/// report order.
pub fn assert_lines(report: &Report, expected: &[(&str, String)]) {
    let want: Vec<(String, String)> = expected
        .iter()
        .map(|(c, e)| (c.to_string(), e.clone()))
        .collect();
    assert_eq!(lines(report), want, "report:\n{report}");
}

/// The two faces of the rebuilt body that use `edge`, ascending.
pub fn faces_of(m: &Model, edge: EdgeId, r: &Rebuilt) -> (String, String) {
    let mut faces: Vec<FaceId> = r
        .faces
        .iter()
        .copied()
        .filter(|&f| {
            m.face(f)
                .unwrap()
                .loops()
                .iter()
                .flat_map(|l| l.coedges())
                .any(|c| c.edge() == edge)
        })
        .collect();
    faces.sort();
    (faces[0].to_string(), faces[1].to_string())
}

pub fn box40(m: &mut Model) -> Body {
    sample::cuboid(m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap()
}

/// The sample cylinder's entities in iteration order: v0 at the base of
/// the seam, v1 at its top; e0 the bottom circle, e1 the seam, e2 the top
/// circle; f0 the wall, f1 the bottom cap, f2 the top cap.
pub fn edit_edge(e: &Edge, geometry: EdgeGeometry) -> Edge {
    Edge::new(geometry, e.start(), e.end(), e.tolerance())
}

pub fn p2(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

/// The corners of an axis-aligned rectangle, counter-clockwise in (u, v)
/// when `ccw`, clockwise otherwise.
pub fn rect(x0: f64, y0: f64, x1: f64, y1: f64, ccw: bool) -> Vec<Point2> {
    let mut corners = vec![p2(x0, y0), p2(x1, y0), p2(x1, y1), p2(x0, y1)];
    if !ccw {
        corners.reverse();
    }
    corners
}

/// The plane `z = 0`, whose own (u, v) is the world's `x` and `y`.
pub fn ground(m: &mut Model) -> SurfaceId {
    m.add_surface(Surface::Plane {
        frame: Frame::world(),
    })
}

/// The bilinear NURBS patch over `[0, 1]²` that is the same plane at the
/// same parameter, extrapolated outside its knot range — a surface with
/// a *bounded* domain, which F1 is the only row to care about.
pub fn ground_patch(m: &mut Model) -> SurfaceId {
    let corner = |u: f64, v: f64| Point3::new(u, v, 0.0);
    m.add_surface(Surface::Nurbs(
        NurbsSurface::new(
            [1, 1],
            [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
            vec![
                corner(0.0, 0.0),
                corner(0.0, 1.0),
                corner(1.0, 0.0),
                corner(1.0, 1.0),
            ],
            vec![1.0; 4],
        )
        .unwrap(),
    ))
}

/// One ring of the ground plane: a vertex per corner, a line edge per
/// side from corner `k` to corner `k + 1`, and a forward coedge per edge
/// whose pcurve is that side in (u, v) at the edge's own parameter.
pub struct Ring {
    pub edges: Vec<EdgeId>,
    pub coedges: Vec<Coedge>,
}

pub fn ring(m: &mut Model, corners: &[Point2]) -> Ring {
    let tol = m.precision().default_tolerance;
    let n = corners.len();
    let at = |p: Point2| Point3::new(p.x, p.y, 0.0);
    let vertices: Vec<_> = corners
        .iter()
        .map(|&c| m.raw().add_vertex(Vertex::new(at(c), tol)))
        .collect();
    let mut edges = Vec::with_capacity(n);
    let mut coedges = Vec::with_capacity(n);
    for k in 0..n {
        let (a, b) = (corners[k], corners[(k + 1) % n]);
        let d = b - a;
        let range = Interval::new(0.0, d.norm()).unwrap();
        let curve = m.add_curve(Curve::Line {
            origin: at(a),
            direction: UnitVec3::new_normalize(Vec3::new(d.x, d.y, 0.0)),
        });
        let edge = m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve { curve, range },
            vertices[k],
            vertices[(k + 1) % n],
            tol,
        ));
        let pcurve = m.add_curve2(Curve2::Line {
            origin: a,
            direction: UnitVec2::new_normalize(d),
        });
        edges.push(edge);
        coedges.push(Coedge::new(edge, Orientation::Forward, pcurve));
    }
    Ring { edges, coedges }
}

/// A body of one shell over `uses`, of `kind`.
pub fn body_of(m: &mut Model, uses: Vec<FaceHandle>, kind: BodyKind) -> (Body, ShellId) {
    let shell = m.raw().add_shell(Shell::new(uses));
    let body = m.raw().add_body(BodyEntity::new(
        kind,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    (Body::forward(body), shell)
}

/// A sheet body of one face on `surface` with `loops`, at `tolerance`.
pub fn one_face(
    m: &mut Model,
    surface: SurfaceId,
    loops: Vec<Loop>,
    tolerance: f64,
) -> (Body, FaceId, ShellId) {
    let face = m.raw().add_face(Face::new(surface, loops, tolerance));
    let (body, shell) = body_of(m, vec![FaceHandle::forward(face)], BodyKind::Sheet);
    (body, face, shell)
}

/// A sheet body of one planar face whose loops are `rings`, in order.
pub fn plate(m: &mut Model, rings: &[Vec<Point2>]) -> (Body, FaceId, Vec<Ring>) {
    let tol = m.precision().default_tolerance;
    let surface = ground(m);
    let rings: Vec<Ring> = rings.iter().map(|c| ring(m, c)).collect();
    let loops = rings.iter().map(|r| Loop::new(r.coedges.clone())).collect();
    let (body, face, _) = one_face(m, surface, loops, tol);
    (body, face, rings)
}
