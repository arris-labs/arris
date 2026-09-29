//! A body to a mesh and a picture, and a face to its (u, v) domain: what
//! `.agents/skills/inspect/SKILL.md` reaches for once a shape has topology,
//! not just geometry (`geom.rs` is for the shape that has none yet).

use std::path::{Path, PathBuf};

use arris_geom::region2::{Polygon2, discretise};
use arris_mesh::cdt::{self, CdtError, VertexRef};
use arris_mesh::{MeshError, Polyline, TriMesh};
use arris_topo::{Body, Face, Model, NotFound};

use crate::render::{Highlight, RenderError, View, render_png};
use crate::unmetered;

/// [`mesh_of`]'s first pass: coarse enough that every curve and surface in
/// a body of ordinary engineering scale hits its minimum segment count
/// (`arris_geom::region2::MIN_SEGMENTS_PER_TURN` and
/// `MIN_SEGMENTS_PER_SPAN`), so the pass is cheap and its mesh's bounding
/// box is what [`RENDER_CHORD_FRACTION`] scales from.
const COARSE_CHORD: f64 = 1e6;

/// The fraction of a scene's bounding-box diagonal [`mesh_of`] and
/// [`render_domain`] use as their chord tolerance: a picture's resolution,
/// not a model tolerance (the checker and the corpus runner use their own).
pub const RENDER_CHORD_FRACTION: f64 = 1.0 / 400.0;

/// [`mesh_of`]'s error: exactly [`arris_mesh::MeshError`], since `mesh_of`
/// is [`unmetered::tessellate`] called twice.
pub type DebugMeshError = MeshError;

/// `body` tessellated at [`RENDER_CHORD_FRACTION`] of its own bounding-box
/// diagonal: a picture's resolution, found by tessellating once at a
/// deliberately coarse chord to measure the box and once more at the
/// chord that gives.
///
/// ```
/// use arris_debug::{body, sample};
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// let mesh = body::mesh_of(&m, cylinder).unwrap();
/// assert!(mesh.is_closed());
/// ```
pub fn mesh_of(m: &Model, body: Body) -> Result<TriMesh, DebugMeshError> {
    let coarse = unmetered::tessellate(m, body, COARSE_CHORD)?;
    let diagonal = coarse.aabb().map_or(0.0, |b| b.diagonal());
    let chord = if diagonal.is_finite() && diagonal > 0.0 {
        diagonal * RENDER_CHORD_FRACTION
    } else {
        COARSE_CHORD
    };
    unmetered::tessellate(m, body, chord)
}

/// Why [`render_body`] could not draw a body.
#[derive(Debug, thiserror::Error)]
pub enum RenderBodyError {
    /// The body could not be meshed.
    #[error(transparent)]
    Mesh(#[from] DebugMeshError),
    /// The mesh could not be rendered.
    #[error(transparent)]
    Render(#[from] RenderError),
}

/// [`mesh_of`]'s mesh of `body`, rendered from `view` with `highlight`, to
/// the PNG at `path` ([`render_png`]'s rules for the path).
///
/// ```no_run
/// use arris_debug::{body, sample};
/// use arris_debug::View;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// let path = body::render_body(&m, cylinder, View::Iso, None, "cylinder-iso").unwrap();
/// assert!(path.exists());
/// ```
pub fn render_body(
    m: &Model,
    body: Body,
    view: View,
    highlight: Option<Highlight>,
    path: impl AsRef<Path>,
) -> Result<PathBuf, RenderBodyError> {
    let mesh = mesh_of(m, body)?;
    Ok(render_png(&mesh, &[], view, highlight, path)?)
}

/// Why [`render_domain`] could not draw a face's domain.
#[derive(Debug, thiserror::Error)]
pub enum DomainError {
    /// The face, or a curve or pcurve it refers to, does not resolve.
    #[error(transparent)]
    NotFound(#[from] NotFound),
    /// The face's loops are not the simple nested polygons a valid face
    /// has — the checker's L4 and L5 promise this never reaches a caller
    /// for a face that passed `Level::Fast`.
    #[error(transparent)]
    Cdt(#[from] CdtError),
    /// A loop's discretisation put a point where the mesh cannot hold it.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// The triangulation could not be rendered.
    #[error(transparent)]
    Render(#[from] RenderError),
}

/// `face`'s loops in (u, v) — the *unscaled* parameters its pcurves are
/// written in, never the surface-lengths scaling [`arris_mesh::tessellate`]
/// triangulates in (ADR-0003) — discretised at [`RENDER_CHORD_FRACTION`]
/// of their own bounding-box diagonal and triangulated by
/// [`arris_mesh::cdt::triangulate`], with no interior points: the picture
/// to read when a face's mesh comes out wrong, since the CDT never sees
/// 3D. Drawn from [`View::Top`] (`u` right, `v` up) with the loops in
/// black over the triangulation's one face colour.
///
/// ```no_run
/// use arris_debug::{body, sample};
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// let wall = m.faces(cylinder).unwrap()[0];
/// let path = body::render_domain(&m, wall, "wall-domain").unwrap();
/// assert!(path.exists());
/// ```
pub fn render_domain(
    m: &Model,
    face: Face,
    path: impl AsRef<Path>,
) -> Result<PathBuf, DomainError> {
    let entity = m.face(face.id)?;
    let mut coarse: Vec<Polygon2> = Vec::with_capacity(entity.loops().len());
    for l in entity.loops() {
        coarse.push(discretise(&m.loop_pieces(l)?, f64::INFINITY));
    }
    let diagonal = polygons_diagonal(&coarse);
    let chord = if diagonal.is_finite() && diagonal > 0.0 {
        diagonal * RENDER_CHORD_FRACTION
    } else {
        f64::INFINITY
    };
    let mut polygons: Vec<Polygon2> = Vec::with_capacity(entity.loops().len());
    for l in entity.loops() {
        polygons.push(discretise(&m.loop_pieces(l)?, chord));
    }
    let triangulation = cdt::triangulate(&polygons, &[])?;

    let mut mesh = TriMesh::new();
    let mut rings: Vec<Vec<u32>> = Vec::with_capacity(polygons.len());
    for polygon in &polygons {
        let mut ring = Vec::with_capacity(polygon.points().len());
        for p in polygon.points() {
            ring.push(mesh.push_position([p.x, p.y, 0.0])?);
        }
        rings.push(ring);
    }
    let index_of = |v: usize| -> Result<u32, DomainError> {
        match triangulation.vertex_ref(v) {
            Some(VertexRef::Polygon { polygon, vertex }) => rings
                .get(polygon)
                .and_then(|r| r.get(vertex))
                .copied()
                .ok_or_else(|| CdtError::Internal("a triangle corner names no loop point").into()),
            _ => Err(CdtError::Internal("a triangle corner is not a polygon point").into()),
        }
    };
    let mut triangles = Vec::with_capacity(triangulation.triangles().len());
    for &[a, b, c] in triangulation.triangles() {
        triangles.push([index_of(a)?, index_of(b)?, index_of(c)?]);
    }
    mesh.push_face(face.id, triangles)?;

    let polylines: Vec<Polyline> = polygons
        .iter()
        .map(|polygon| {
            let mut points: Vec<[f64; 3]> =
                polygon.points().iter().map(|p| [p.x, p.y, 0.0]).collect();
            if let Some(&first) = points.first() {
                points.push(first);
            }
            Polyline::new(points)
        })
        .collect();

    Ok(render_png(&mesh, &polylines, View::Top, None, path)?)
}

/// The diagonal of the box around every point of `polygons`, or `0.0` for
/// no points.
fn polygons_diagonal(polygons: &[Polygon2]) -> f64 {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for polygon in polygons {
        for p in polygon.points() {
            lo = [lo[0].min(p.x), lo[1].min(p.y)];
            hi = [hi[0].max(p.x), hi[1].max(p.y)];
        }
    }
    if lo[0].is_finite() {
        ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2)).sqrt()
    } else {
        0.0
    }
}
