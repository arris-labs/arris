//! A body to a [`TriMesh`]: edges discretised once, faces triangulated in
//! their own (u, v) through the same-parameter pcurves (ADR-0003,
//! `docs/ARCHITECTURE.md` §Tessellation).

use std::collections::BTreeMap;

use arris_check::domain::FaceDomain;
use arris_geom::Surface;
use arris_geom::region2::{MAX_SEGMENTS_PER_PIECE, Polygon2};
use arris_math::{Control, Interval, Meter, Point2, UnitVec3, Vec3};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{Body, EdgeId, FaceId, Model, NotFound, Orientation, VertexId};

use crate::cdt::{self, CdtError, VertexRef};
use crate::corners::{CornerFace, Corners};
use crate::{MeshError, TriMesh};

/// A cap on a face's interior lattice, total points in both directions:
/// an allocation and a CDT input this size is already enormous, and a
/// face that would need more — a torus whose minor radius is orders of
/// magnitude below its major one, at a fine chord — is
/// [`MeshError::GridTooLarge`] instead of attempted.
pub const MAX_INTERIOR_POINTS: usize = 1 << 20;

/// What a caller asks [`tessellate_with`] for: the chord tolerance, and
/// whether the mesh carries the render buffer beside the watertight one
/// (ADR-0012).
///
/// ```
/// use arris_mesh::MeshRequest;
///
/// let plain = MeshRequest::new(1e-3);
/// assert!(!plain.corners);
/// assert!(MeshRequest::new(1e-3).with_corners().corners);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeshRequest {
    /// How far a triangle may lie from the surface it stands on: the
    /// consumer's request, a number like a render's resolution, not a
    /// model tolerance. Finite and positive.
    pub chord: f64,
    /// Whether to build [`crate::Corners`]: face-local vertices with
    /// per-corner outward normals and the surface's own (u, v), beside
    /// the watertight buffer and changing nothing in it.
    pub corners: bool,
}

impl MeshRequest {
    /// A request for the watertight buffer alone, at `chord`.
    pub const fn new(chord: f64) -> Self {
        MeshRequest {
            chord,
            corners: false,
        }
    }

    /// The same request, with the corner block.
    pub const fn with_corners(mut self) -> Self {
        self.corners = true;
        self
    }
}

/// The samples of one edge: `n + 1` parameters over its range and the
/// mesh index of each, the first and last being the end vertices'.
struct EdgeSamples {
    params: Vec<f64>,
    indices: Vec<u32>,
}

/// The triangle mesh of `body` at `chord`: every position within `chord`
/// of the geometry, as ADR-0003 and `docs/ARCHITECTURE.md`
/// §Tessellation guarantee.
///
/// Every topo vertex of the body is one mesh vertex. Every edge is
/// sampled once, at `n` uniform parameters over its range — `n` the
/// largest of its curve's [`arris_geom::Curve::chord_segments`]
/// at `chord` and, per coedge, the count that keeps each step's `u`- and
/// `v`-travel under the face's surface's
/// [`arris_geom::Surface::chord_steps`], so a triangle
/// standing on the edge is within `chord` of the surface — and its
/// samples are one run of positions, its [`crate::EdgeRange`] the
/// polyline from its start vertex to its end vertex along its curve's
/// parameter; a degenerate edge's range is one index. Every face's loops
/// are the *same* parameters through each coedge's pcurve, triangulated
/// by [`cdt::triangulate`] and mapped back to the shared indices, so the
/// mesh of a solid is closed by construction; a seam's run is used twice
/// with opposite directions, triangles that collapse onto one index are
/// dropped, and the triangles of a face used `Reversed` are turned so
/// every triangle is counter-clockwise seen from outside. A face whose
/// surface curves in both directions — a sphere, a torus, a NURBS
/// surface — also carries interior points on a uniform (u, v) lattice at
/// its `chord_steps`, those the loops wind around, so a triangle in the
/// middle of the face is within `chord` of the surface as one standing
/// on an edge is; a plane, a cylinder and a cone are ruled and take
/// none, and on a cylinder or a cone the ruled direction is flattened
/// before the triangulation so no triangle travels more than one chord
/// step in the curved one, however oblique to the ruling the face's
/// region runs (ADR-0005). Ranges are in the body's iteration order (`Model::faces`,
/// `Model::edges`). The output is the same on every platform for the
/// same body and chord.
///
/// The chord is the consumer's request, a number like the render's
/// resolution, not a model tolerance; it must be finite and positive.
/// In debug builds the body passes `arris_check` at `Level::Fast` first.
///
/// Errors: [`MeshError::Chord`]; [`MeshError::InvalidInput`] (debug
/// builds); [`MeshError::NotFound`] for the body or anything it refers
/// to; [`MeshError::Face`] when a face's loops are not the simple nested
/// polygons a valid face has; [`MeshError::GridTooLarge`] when a face's
/// interior lattice at `chord` would need more than
/// [`MAX_INTERIOR_POINTS`] points; [`MeshError::NonFinitePosition`] when
/// the geometry evaluates to a non-finite point; [`MeshError::Interrupted`]
/// when `control`'s poll or budget stops it (a step is an edge, a face, an
/// interior point or a CDT insertion; ADR-0030). The model is only read,
/// so an interrupt leaves nothing to undo.
///
/// ```
/// use arris_debug::sample;
/// use arris_mesh::tessellate;
/// use arris_topo::Model;
/// use arris_math::Control;
/// use core::f64::consts::PI;
///
/// let mut m = Model::default();
/// let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// let mesh = tessellate(&m, cylinder, 1e-3, &Control::NONE).unwrap();
/// assert!(mesh.is_closed());
/// assert_eq!(mesh.faces().len(), 3);
/// assert_eq!(mesh.edges().len(), 3);
/// let volume = mesh.signed_volume().unwrap();
/// let exact = PI * 16.0 * 12.0;
/// // An inscribed prism at sagitta δ on radius r: within 4δ / (3r).
/// assert!(volume > 0.0 && exact - volume <= exact * 4.0 * 1e-3 / (3.0 * 4.0));
/// ```
pub fn tessellate(
    m: &Model,
    body: Body,
    chord: f64,
    control: &Control<'_>,
) -> Result<TriMesh, MeshError> {
    tessellate_with(m, body, &MeshRequest::new(chord), control)
}

/// [`tessellate`] of `request.chord`, with the render buffer beside the
/// watertight one when `request.corners` asks for it (ADR-0012).
///
/// Everything [`tessellate`] guarantees holds unchanged, position for
/// position and triangle for triangle: the corner block is an addition,
/// never a different mesh. Asked for, [`TriMesh::corners`] is `Some` and
/// carries one *face-local vertex* per face per triangulation input
/// point — a loop sample or an interior lattice point — with
///
/// - the shared position index it stands on, so the watertight mesh is
///   one lookup away;
/// - the **outward** unit normal there: the surface's own, flipped where
///   the face is used `Reversed`, never averaged with a neighbouring
///   face's, so a sharp edge stays sharp;
/// - the surface's **own** (u, v), never normalised, so a seam's two
///   copies differ by exactly one period and a consumer that wants
///   `[0, 1]` divides by [`crate::CornerFace::uv_box`].
///
/// Where the parametrisation is singular — a sphere's pole, a cone's
/// apex — the normal is the limit approached along the parameter the
/// surface still moves in, from inside the face's own (u, v) box: `±Z`
/// at a sphere's pole for every corner of the fan, and at a cone's apex
/// one normal per corner's own `u` (ADR-0012).
///
/// Errors: [`tessellate`]'s (`control` stops it the same way), and [`MeshError::Corners`] if the block
/// could not be built — a face no point of which has a normal.
///
/// ```
/// use arris_debug::sample;
/// use arris_mesh::{MeshRequest, tessellate};
/// use arris_topo::Model;
/// use arris_math::{Control, Point3};
///
/// let mut m = Model::default();
/// let ball = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
/// let request = MeshRequest::new(1e-2).with_corners();
/// let mesh = arris_mesh::tessellate_with(&m, ball, &request, &Control::NONE).unwrap();
///
/// // The watertight buffer is the one `tessellate` gives.
/// let plain = tessellate(&m, ball, 1e-2, &Control::NONE).unwrap();
/// assert_eq!(mesh.positions(), plain.positions());
/// assert_eq!(mesh.triangles(), plain.triangles());
/// assert!(plain.corners().is_none());
///
/// let corners = mesh.corners().unwrap();
/// assert_eq!(corners.triangles().len(), mesh.triangles().len());
/// // Every corner normal is the outward radial direction of the ball.
/// for (&position, normal) in corners.positions().iter().zip(corners.normals()) {
///     let p = mesh.positions()[position as usize];
///     let radial = p.map(|c| c / 3.0);
///     assert!((0..3).all(|i| (radial[i] - normal[i]).abs() < 1e-9));
/// }
/// ```
pub fn tessellate_with(
    m: &Model,
    body: Body,
    request: &MeshRequest,
    control: &Control<'_>,
) -> Result<TriMesh, MeshError> {
    let mut meter = Meter::new(control);
    let chord = request.chord;
    if !(chord.is_finite() && chord > 0.0) {
        return Err(MeshError::Chord(chord));
    }
    // A handle that does not resolve is `NotFound` in every build; the
    // debug check below would otherwise report it as an M1 violation.
    m.body(body.id)?;
    #[cfg(debug_assertions)]
    {
        let report = arris_check::check(m, body, arris_check::Level::Fast);
        if !report.is_ok() {
            return Err(MeshError::InvalidInput {
                body,
                report: Box::new(report),
            });
        }
    }
    let faces = m.faces(body)?;
    let edges = m.edges(body)?;
    let vertices = m.vertices(body)?;

    // Pass one: what each face asks of the edges it uses, so a step along
    // an edge never travels farther in (u, v) than the surface allows,
    // and the (u, v) box and step each face's interior grid stands on.
    let mut required: BTreeMap<EdgeId, usize> = BTreeMap::new();
    let mut domains: Vec<([Interval; 2], [f64; 2])> = Vec::with_capacity(faces.len());
    for f in &faces {
        meter.tick()?;
        let face = m.face(f.id)?;
        let surface = m.surface(face.surface())?;
        // The (u, v) box holding the loops' true boundary, read at the
        // requested chord; the whole plane for a face with no loop.
        let bounds = FaceDomain::of(m, f.id, chord)?
            .uv_box()
            .unwrap_or([Interval::REAL; 2]);
        let steps = surface.chord_steps(chord, bounds);
        domains.push((bounds, steps));
        for coedge in face.loops().iter().flat_map(|l| l.coedges()) {
            let range = m.edge(coedge.edge())?.range();
            let speed = m.curve2(coedge.pcurve())?.speed_bounds(range);
            let mut n = 0usize;
            for dir in 0..2 {
                let travel = range.length() * speed[dir];
                if travel <= 0.0 {
                    continue;
                }
                let wanted = (travel / steps[dir]).ceil();
                n = n.max(if wanted.is_finite() {
                    wanted as usize
                } else {
                    MAX_SEGMENTS_PER_PIECE
                });
            }
            let entry = required.entry(coedge.edge()).or_default();
            *entry = (*entry).max(n);
        }
    }

    // Pass two: every vertex a position, every edge a run of samples.
    let mut mesh = TriMesh::new();
    let mut vertex_index: BTreeMap<VertexId, u32> = BTreeMap::new();
    for v in &vertices {
        let p = m.vertex(v.id)?.point();
        let index = mesh.push_position([p.x, p.y, p.z])?;
        vertex_index.insert(v.id, index);
    }
    let mut samples: BTreeMap<EdgeId, EdgeSamples> = BTreeMap::new();
    for e in &edges {
        meter.tick()?;
        let edge = m.edge(e.id)?;
        let range = edge.range();
        let start = *vertex_index
            .get(&edge.start())
            .ok_or(NotFound::new(edge.start()))?;
        let end = *vertex_index
            .get(&edge.end())
            .ok_or(NotFound::new(edge.end()))?;
        let curve = match edge.geometry() {
            EdgeGeometry::Curve { curve, .. } => Some(m.curve(curve)?),
            EdgeGeometry::Degenerate { .. } => None,
        };
        let n = curve
            .map_or(1, |c| c.chord_segments(range, chord))
            .max(required.get(&e.id).copied().unwrap_or(0))
            .clamp(1, MAX_SEGMENTS_PER_PIECE);
        let params: Vec<f64> = (0..=n).map(|i| range.lerp(i as f64 / n as f64)).collect();
        let indices = match curve {
            Some(c) => {
                let mut indices = Vec::with_capacity(n + 1);
                indices.push(start);
                for &t in &params[1..n] {
                    let p = c.point(t);
                    indices.push(mesh.push_position([p.x, p.y, p.z])?);
                }
                indices.push(end);
                indices
            }
            None => vec![start; n + 1],
        };
        if curve.is_some() {
            mesh.push_edge(e.id, &indices)?;
        } else {
            mesh.push_edge(e.id, &[start])?;
        }
        samples.insert(e.id, EdgeSamples { params, indices });
    }

    // Pass three: every face's loops through its pcurves at the edges'
    // parameters, mapped back to the shared indices, positions pushed —
    // all sequential, since the shared position buffer's indices depend
    // on push order. What is left, [`triangulate_face`]'s CDT, touches
    // only its own face's data and runs in parallel behind `parallel`.
    let mut works: Vec<FaceWork> = Vec::with_capacity(faces.len());
    let mut block = request.corners.then(CornerBlock::default);
    for (k, f) in faces.iter().enumerate() {
        meter.tick()?;
        let face = m.face(f.id)?;
        let mut polygons: Vec<Polygon2> = Vec::with_capacity(face.loops().len());
        let mut rings: Vec<Vec<u32>> = Vec::with_capacity(face.loops().len());
        for l in face.loops() {
            let mut points: Vec<Point2> = Vec::new();
            let mut indices: Vec<u32> = Vec::new();
            for coedge in l.coedges() {
                let s = samples
                    .get(&coedge.edge())
                    .ok_or(NotFound::new(coedge.edge()))?;
                let pcurve = m.curve2(coedge.pcurve())?;
                let n = s.params.len() - 1;
                // All but the last sample in walking order: the next
                // coedge's first sample is the junction.
                let order: Vec<usize> = if coedge.orientation() == Orientation::Forward {
                    (0..n).collect()
                } else {
                    (1..=n).rev().collect()
                };
                for i in order {
                    let uv = pcurve.point(s.params[i]);
                    if points.last() != Some(&uv) {
                        points.push(uv);
                        indices.push(s.indices[i]);
                    }
                }
            }
            if points.len() > 1 && points.first() == points.last() {
                points.pop();
                indices.pop();
            }
            let polygon = Polygon2::from_points(points.iter().copied());
            if polygon.points().len() != indices.len() {
                return Err(MeshError::Internal(
                    "a loop's ring and its index ring differ in length",
                ));
            }
            polygons.push(polygon);
            rings.push(indices);
        }
        // The interior points the surface's curvature asks for, each a
        // position of its own on the surface.
        let (bounds, steps) = domains.get(k).copied().ok_or(NotFound::new(f.id))?;
        let surface = m.surface(face.surface())?;
        let interior = interior_grid(f.id, &polygons, bounds, steps)?;
        let mut interior_indices: Vec<u32> = Vec::with_capacity(interior.len());
        for uv in &interior {
            meter.tick()?;
            let p = surface.point(uv.x, uv.y);
            interior_indices.push(mesh.push_position([p.x, p.y, p.z])?);
        }
        // The triangulation is taken in a (u, v) scaled to the
        // surface's own lengths, and flattened along a ruled direction,
        // so Delaunay's criterion measures what the chord bound is made
        // of: the indices it returns are the same either way.
        let scale = uv_scale(surface, bounds, steps);
        let scaled: Vec<Polygon2> = polygons
            .iter()
            .map(|p| Polygon2::from_points(p.points().iter().map(|q| scaled_point(*q, scale))))
            .collect();
        let scaled_interior: Vec<Point2> =
            interior.iter().map(|p| scaled_point(*p, scale)).collect();
        let reversed = f.orientation == Orientation::Reversed;
        // The face-local vertices this face contributes to the corner
        // block: its loop samples in ring order, then its interior
        // points, each with the (u, v) the CDT was given unscaled and
        // the outward normal there (ADR-0012).
        let corners = match block.as_mut() {
            Some(block) => Some(block.push_face(
                f.id,
                surface,
                bounds,
                reversed,
                &polygons,
                &rings,
                &interior,
                &interior_indices,
            )?),
            None => None,
        };
        works.push(FaceWork {
            face: f.id,
            reversed,
            polygons: scaled,
            interior: scaled_interior,
            rings,
            interior_indices,
            corners,
        });
    }

    // Pass four: each face's CDT and the mapping of its triangle corners
    // back to the shared indices — the compute-heavy, purely local step,
    // over `rayon` behind `parallel`, sequential otherwise; either way
    // the results are collected in face order before they reach `mesh`,
    // so the mesh is identical with the feature on or off.
    for (w, triangles) in works.iter().zip(triangulate_faces(&works, &mut meter)?) {
        mesh.push_face(w.face, triangles.shared)?;
        if let Some(block) = block.as_mut() {
            block.triangles.extend(triangles.local);
        }
    }
    match block {
        Some(block) => mesh.with_corners(block.finish()?),
        None => Ok(mesh),
    }
}

/// The corner block under construction: the three vertex arrays, the
/// triangles over them and one [`CornerFace`] per face, all in face
/// iteration order (ADR-0012).
#[derive(Default)]
struct CornerBlock {
    positions: Vec<u32>,
    normals: Vec<[f64; 3]>,
    uvs: Vec<[f64; 2]>,
    triangles: Vec<[u32; 3]>,
    faces: Vec<CornerFace>,
}

impl CornerBlock {
    /// Appends one face's face-local vertices — its loop samples ring by
    /// ring, then its interior points — and returns where they start and
    /// how the CDT's input points map onto them.
    #[allow(clippy::too_many_arguments)]
    fn push_face(
        &mut self,
        face: FaceId,
        surface: &Surface,
        bounds: [Interval; 2],
        reversed: bool,
        polygons: &[Polygon2],
        rings: &[Vec<u32>],
        interior: &[Point2],
        interior_indices: &[u32],
    ) -> Result<CornerWork, MeshError> {
        let base = self.uvs.len();
        let mut ring_starts = Vec::with_capacity(polygons.len());
        let mut uvs: Vec<[f64; 2]> = Vec::new();
        for (polygon, ring) in polygons.iter().zip(rings) {
            ring_starts.push(uvs.len());
            for (uv, &index) in polygon.points().iter().zip(ring) {
                uvs.push([uv.x, uv.y]);
                self.positions.push(index);
            }
        }
        let interior_start = uvs.len();
        for (uv, &index) in interior.iter().zip(interior_indices) {
            uvs.push([uv.x, uv.y]);
            self.positions.push(index);
        }
        let mut normals: Vec<Option<[f64; 3]>> = uvs
            .iter()
            .map(|&[u, v]| corner_normal(surface, Point2::new(u, v), bounds, reversed))
            .collect();
        fill_missing_normals(face, &uvs, &mut normals)?;
        let normals: Vec<[f64; 3]> =
            normals
                .into_iter()
                .collect::<Option<_>>()
                .ok_or(MeshError::Internal(
                    "a face-local vertex is left without a normal",
                ))?;
        let mut uv_box = [[f64::INFINITY, f64::NEG_INFINITY]; 2];
        for uv in &uvs {
            for dir in 0..2 {
                uv_box[dir][0] = uv_box[dir][0].min(uv[dir]);
                uv_box[dir][1] = uv_box[dir][1].max(uv[dir]);
            }
        }
        let uv_box = uv_box.map(|[lo, hi]| Interval::new(lo, hi));
        let [u, v] = uv_box;
        let uv_box = match (u, v) {
            (Ok(u), Ok(v)) => [u, v],
            _ => {
                return Err(MeshError::Corners(format!(
                    "{face}: a face-local vertex has a non-finite (u, v)"
                )));
            }
        };
        self.normals.extend(normals);
        self.uvs.extend(uvs);
        self.faces.push(CornerFace {
            face,
            vertices: base..self.uvs.len(),
            uv_box,
        });
        Ok(CornerWork {
            base,
            ring_starts,
            interior_start,
        })
    }

    /// The finished, validated block.
    fn finish(self) -> Result<Corners, MeshError> {
        Corners::from_parts(
            self.positions,
            self.normals,
            self.uvs,
            self.triangles,
            self.faces,
        )
    }
}

/// The outward unit normal of `surface` at `uv` for a face used forward
/// or `reversed`, with ADR-0012's rule where the parametrisation is
/// singular: the limit of the normal approached along the parameter the
/// surface still moves in, from inside the face's own `bounds`. `None`
/// where even that limit vanishes.
fn corner_normal(
    surface: &Surface,
    uv: Point2,
    bounds: [Interval; 2],
    reversed: bool,
) -> Option<[f64; 3]> {
    let n = match surface.normal(uv.x, uv.y) {
        Some(n) => n.into_inner(),
        None => singular_normal(surface, uv, bounds)?,
    };
    let n = if reversed { -n } else { n };
    Some([n.x, n.y, n.z])
}

/// The limit of `∂P/∂u × ∂P/∂v` at a singular `uv`, to first order.
///
/// Where `∂P/∂u` vanishes — a sphere's pole, a cone's apex — the
/// parameter the surface still moves in is `v`, and
/// `∂P/∂u(u, v + ε) ≈ ε ∂²P/∂u∂v`, so the cross product tends to
/// `ε (∂²P/∂u∂v × ∂P/∂v)`; symmetrically where `∂P/∂v` vanishes. The
/// sign of `ε` is which side of the singular parameter the face's own
/// domain lies on, so the limit is taken from inside the face.
///
/// Whichever of the two candidates is longer is the live one — the other
/// is a cross product with a vector that vanished — so no threshold and
/// no match on the surface kind is needed, and neither is a division.
/// A sphere's pole comes out `±Z` for every `u`, a cone's apex as the
/// cone's own normal at the corner's `u`.
fn singular_normal(surface: &Surface, uv: Point2, bounds: [Interval; 2]) -> Option<Vec3> {
    let e = surface.eval(uv.x, uv.y);
    // Which way the face's domain lies from `t`; outward-unbounded, or
    // exactly at the middle, takes the forward side.
    let inward = |b: Interval, t: f64| -> f64 {
        let mid = b.midpoint();
        if !mid.is_finite() || mid >= t {
            1.0
        } else {
            -1.0
        }
    };
    let along_v = e.duv.cross(&e.dv) * inward(bounds[1], uv.y);
    let along_u = e.du.cross(&e.duv) * inward(bounds[0], uv.x);
    let limit = if along_v.norm() >= along_u.norm() {
        along_v
    } else {
        along_u
    };
    UnitVec3::try_new(limit, 0.0).map(UnitVec3::into_inner)
}

/// ADR-0012's fallback: a corner that even [`singular_normal`] leaves
/// without one — a NURBS whose two derivatives stay parallel to second
/// order — takes the normal of the nearest face-local vertex of the same
/// face that has one, by (u, v) distance and then by index, so the
/// result is deterministic and the block never carries an absent normal.
fn fill_missing_normals(
    face: FaceId,
    uvs: &[[f64; 2]],
    normals: &mut [Option<[f64; 3]>],
) -> Result<(), MeshError> {
    if normals.iter().all(Option::is_some) {
        return Ok(());
    }
    let known: Vec<([f64; 2], [f64; 3])> = uvs
        .iter()
        .zip(normals.iter())
        .filter_map(|(uv, n)| n.map(|n| (*uv, n)))
        .collect();
    if known.is_empty() {
        return Err(MeshError::Corners(format!(
            "{face}: no face-local vertex of the face has a surface normal"
        )));
    }
    for (uv, normal) in uvs.iter().zip(normals.iter_mut()) {
        if normal.is_some() {
            continue;
        }
        let mut best: Option<(f64, [f64; 3])> = None;
        for (at, n) in &known {
            let d = (at[0] - uv[0]).powi(2) + (at[1] - uv[1]).powi(2);
            if best.is_none_or(|(so_far, _)| d < so_far) {
                best = Some((d, *n));
            }
        }
        *normal = best.map(|(_, n)| n);
    }
    Ok(())
}

/// One face's inputs to its CDT, gathered while the shared position
/// buffer is still being pushed to in face order (`tessellate`'s pass
/// three) so that [`triangulate_faces`] can run each face independently.
struct FaceWork {
    face: FaceId,
    reversed: bool,
    /// Loop polygons and interior points, in the scaled (u, v) [`uv_scale`]
    /// defines.
    polygons: Vec<Polygon2>,
    interior: Vec<Point2>,
    /// Loop `i`, vertex `j`'s mesh index, indexed the same as `polygons`.
    rings: Vec<Vec<u32>>,
    /// Interior point `i`'s mesh index, indexed the same as `interior`.
    interior_indices: Vec<u32>,
    /// Where this face's face-local vertices sit in the corner block,
    /// `None` when none was asked for.
    corners: Option<CornerWork>,
}

/// How one face's CDT input points map onto its face-local vertices in
/// the corner block: the block is laid out ring by ring and then the
/// interior points, exactly as [`CornerBlock::push_face`] appended them.
struct CornerWork {
    /// The block index of this face's first face-local vertex.
    base: usize,
    /// Where each loop ring starts, relative to `base`.
    ring_starts: Vec<usize>,
    /// Where the interior points start, relative to `base`.
    interior_start: usize,
}

/// One face's triangles in both index spaces, the same triangles in the
/// same order: `local` is empty when no corner block was asked for.
struct FaceTriangles {
    shared: Vec<[u32; 3]>,
    local: Vec<[u32; 3]>,
}

/// One face's CDT and the triangle corners mapped back to the shared
/// mesh indices — and, where the face carries a [`CornerWork`], to its
/// face-local ones as well — oriented by `reversed`, collapsed triangles
/// dropped. The two lists are the same triangles in the same order.
fn triangulate_face(w: &FaceWork, meter: &mut Meter<'_>) -> Result<FaceTriangles, MeshError> {
    let triangulation = cdt::triangulate_metered(&w.polygons, &w.interior, meter).map_err(
        |source| match source {
            CdtError::Interrupted(stop) => MeshError::Interrupted(stop),
            source => MeshError::Face {
                face: w.face,
                source,
            },
        },
    )?;
    let index_of =
        |v: usize| -> Result<(u32, u32), MeshError> {
            match triangulation.vertex_ref(v) {
                Some(VertexRef::Polygon { polygon, vertex }) => {
                    let shared = w
                        .rings
                        .get(polygon)
                        .and_then(|r| r.get(vertex))
                        .copied()
                        .ok_or(MeshError::Internal("a triangle corner names no loop point"))?;
                    let local =
                        match &w.corners {
                            Some(c) => {
                                c.base
                                    + c.ring_starts.get(polygon).copied().ok_or(
                                        MeshError::Internal("a triangle corner names no loop ring"),
                                    )?
                                    + vertex
                            }
                            None => 0,
                        };
                    Ok((shared, local as u32))
                }
                Some(VertexRef::Interior(i)) => {
                    let shared = w
                        .interior_indices
                        .get(i)
                        .copied()
                        .ok_or(MeshError::Internal(
                            "a triangle corner names no interior point",
                        ))?;
                    let local = match &w.corners {
                        Some(c) => c.base + c.interior_start + i,
                        None => 0,
                    };
                    Ok((shared, local as u32))
                }
                None => Err(MeshError::Internal(
                    "a triangle corner is not an input point",
                )),
            }
        };
    let n = triangulation.triangles().len();
    let mut shared = Vec::with_capacity(n);
    let mut local = Vec::with_capacity(if w.corners.is_some() { n } else { 0 });
    for &[a, b, c] in triangulation.triangles() {
        let (ia, la) = index_of(a)?;
        let (ib, lb) = index_of(b)?;
        let (ic, lc) = index_of(c)?;
        if ia == ib || ib == ic || ic == ia {
            continue;
        }
        let wind = |[a, b, c]: [u32; 3]| if w.reversed { [a, c, b] } else { [a, b, c] };
        shared.push(wind([ia, ib, ic]));
        if w.corners.is_some() {
            local.push(wind([la, lb, lc]));
        }
    }
    Ok(FaceTriangles { shared, local })
}

/// [`triangulate_face`] over every face, in face order in the result
/// regardless of how it was computed: over `rayon` behind `parallel`,
/// a plain loop otherwise. Each face gets a [`Meter::split`] of `meter`
/// and the faces' steps are charged to it in face order, so the first
/// face whose running total crosses the budget stops the call with the
/// same count in both builds (ADR-0030 §4); every thread polls.
fn triangulate_faces(
    works: &[FaceWork],
    meter: &mut Meter<'_>,
) -> Result<Vec<FaceTriangles>, MeshError> {
    /// One face's outcome and steps into `meter`, in face order.
    fn settle(
        meter: &mut Meter<'_>,
        result: Result<FaceTriangles, MeshError>,
        steps: u64,
    ) -> Result<FaceTriangles, MeshError> {
        match result {
            Err(MeshError::Interrupted(stop)) => Err(meter.charge_stop(stop).into()),
            other => {
                meter.charge(steps)?;
                other
            }
        }
    }
    let mut out = Vec::with_capacity(works.len());
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        // Every result first, then the first error in face order: a
        // `Result` collected straight from `rayon` is whichever error a
        // thread met first.
        let base = meter.split();
        let all: Vec<(Result<FaceTriangles, MeshError>, u64)> = works
            .par_iter()
            .map(|w| {
                let mut own = base;
                let result = triangulate_face(w, &mut own);
                (result, own.steps())
            })
            .collect();
        for (result, steps) in all {
            out.push(settle(meter, result, steps)?);
        }
    }
    #[cfg(not(feature = "parallel"))]
    for w in works {
        let mut own = meter.split();
        let result = triangulate_face(w, &mut own);
        out.push(settle(meter, result, own.steps())?);
    }
    Ok(out)
}

/// How much longer the surface is along `u` than along `v` over the
/// region, as the two mean speeds `|∂P/∂u|` and `|∂P/∂v|` sampled over
/// its box.
///
/// A Delaunay triangulation in the raw parameters would call a torus's
/// `u`, along which the surface runs `R + r cos v` units per radian, and
/// its `v`, along which it runs `r`, the same length, and stretch
/// triangles across whichever is short — past the deviation the chord
/// steps promise, since the bound holds for a triangle a step wide, not
/// for one that spans the region. Scaling the domain by these speeds
/// makes it roughly isometric to the surface, which is the shape
/// Delaunay's empty-circle criterion is good at (ADR-0003; Open
/// CASCADE's `BRepMesh` scales its domain the same way). A direction the
/// surface is ruled along is then [`flattened`], since the criterion has
/// nothing to weigh there (ADR-0005).
fn uv_scale(surface: &Surface, bounds: [Interval; 2], steps: [f64; 2]) -> [f64; 2] {
    let mut sums = [0.0f64; 2];
    let mut count = 0.0;
    for i in 0..=SPEED_SAMPLES {
        let u = bounds[0].lerp(i as f64 / SPEED_SAMPLES as f64);
        for j in 0..=SPEED_SAMPLES {
            let e = surface.eval(u, bounds[1].lerp(j as f64 / SPEED_SAMPLES as f64));
            sums[0] += e.du.norm();
            sums[1] += e.dv.norm();
            count += 1.0;
        }
    }
    let mut scale = [sums[0] / count, sums[1] / count];
    let largest = scale[0].max(scale[1]);
    if !(largest.is_finite() && largest > 0.0) {
        return [1.0; 2];
    }
    for s in &mut scale {
        // A direction the surface does not move along at all — no face
        // has one over its whole region — would collapse the domain.
        *s = if s.is_finite() && *s > 0.0 {
            *s / largest
        } else {
            1.0
        };
    }
    flattened(scale, bounds, steps)
}

/// `scale` with a ruled direction flattened: on a surface curved along
/// one parameter and ruled along the other — a cylinder, a cone — the
/// ruled direction is scaled so the region's whole extent along it is
/// [`RULED_RIBBON`] of one chord step in the curved one.
///
/// A step along the ruling costs no deviation, so it must not compete in
/// the empty-circle criterion: in an isometric domain it does, and a
/// region whose boundary chains run oblique to the ruling — the wall of
/// a hole cut at an angle, whose two ends are ellipse sections — is
/// triangulated by joining each boundary point to the one *nearest* on
/// the other chain, which is offset along the ruling by the shear and so
/// travels a large part of a turn in the curved parameter, far past the
/// step the chord bound stands on. Flattening the region to a ribbon
/// leaves the curved parameter alone to decide, which is the mesh a
/// ruled face wants — one quad per step of its boundary — and costs no
/// interior points at all.
///
/// The bound: a Delaunay triangle's circumcircle holds no vertex, and a
/// circle that covers a ribbon of thickness `e` over a span `w` of the
/// curved parameter holds every boundary sample in that span, so
/// `w` is under the boundary's own step `d`; a circle of radius `r`
/// centred in the ribbon covers `2√(r² − e²)`, so `r² < d²/4 + e²` and
/// the triangle, which the circle contains, travels under `√(d² + 4e²)`
/// — within a few hundredths of `d` at `e = d / 8`.
///
/// A plane is ruled both ways and takes no chord step at all, so nothing
/// is flattened there and any triangulation of it is exact; a sphere, a
/// torus and a NURBS surface curve both ways and keep the isometric
/// domain their interior lattice is sized in.
fn flattened(scale: [f64; 2], bounds: [Interval; 2], steps: [f64; 2]) -> [f64; 2] {
    let mut out = scale;
    for curved in 0..2 {
        let ruled = 1 - curved;
        if !steps[curved].is_finite() || steps[ruled].is_finite() {
            continue;
        }
        let extent = bounds[ruled].length() * scale[ruled];
        let ribbon = RULED_RIBBON * steps[curved] * scale[curved];
        if extent.is_finite() && extent > ribbon && ribbon > 0.0 {
            out[ruled] = scale[ruled] * ribbon / extent;
        }
    }
    out
}

/// How many samples per direction [`uv_scale`] takes of the speeds: a
/// mean over the region, not a bound, so a coarse grid is enough.
const SPEED_SAMPLES: usize = 4;

/// How thick [`flattened`] leaves a ruled region, as a share of one
/// chord step in the curved direction: `1 / 8`, which by the bound in
/// [`flattened`] keeps a triangle's travel in the curved parameter under
/// `√(1 + 4 / 64)` of the boundary's own step — three hundredths over,
/// against the square in the deviation, so the inscribed-prism bound
/// holds as it does for a face whose boundary runs along the ruling.
const RULED_RIBBON: f64 = 0.125;

/// A (u, v) point in the scaled domain [`uv_scale`] defines.
fn scaled_point(p: Point2, scale: [f64; 2]) -> Point2 {
    Point2::new(p.x * scale[0], p.y * scale[1])
}

/// The interior points of a face's domain: a uniform (u, v) lattice at
/// most `steps` apart, strictly inside `bounds`, keeping the points the
/// loops wind around that lie on no loop segment.
///
/// A direction the surface is flat or ruled along has an infinite step
/// and so no interior line, which leaves the grid empty on a plane, a
/// cylinder and a cone: there the loops' own samples already bound the
/// chord (ADR-0003). A sphere, a torus and a NURBS surface curve in both
/// directions and get a lattice sized by
/// [`arris_geom::Surface::chord_steps`], never by a
/// per-triangle error estimate.
fn interior_grid(
    face: FaceId,
    polygons: &[Polygon2],
    bounds: [Interval; 2],
    steps: [f64; 2],
) -> Result<Vec<Point2>, MeshError> {
    let mut counts = [1usize; 2];
    for dir in 0..2 {
        let length = bounds[dir].length();
        if !(length.is_finite() && length > 0.0 && steps[dir] > 0.0) {
            return Ok(Vec::new());
        }
        let wanted = (length / steps[dir]).ceil();
        if !wanted.is_finite() {
            return Ok(Vec::new());
        }
        counts[dir] = (wanted as usize).max(1);
    }
    // Capped by the lattice's total, not per direction: two directions
    // each under the old per-direction cap could still multiply into an
    // allocation and a CDT input no chord tolerance should ask for.
    let points_wanted = counts[0].saturating_mul(counts[1]);
    if points_wanted > MAX_INTERIOR_POINTS {
        return Err(MeshError::GridTooLarge {
            face,
            points: points_wanted,
        });
    }
    let mut points = Vec::new();
    for i in 1..counts[0] {
        let u = bounds[0].lerp(i as f64 / counts[0] as f64);
        for j in 1..counts[1] {
            let p = Point2::new(u, bounds[1].lerp(j as f64 / counts[1] as f64));
            let winding: i32 = polygons.iter().map(|q| q.winding_number(p)).sum();
            if winding != 0 && !polygons.iter().any(|q| q.contains(p)) {
                points.push(p);
            }
        }
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Frame;

    fn cylinder(radius: f64) -> Surface {
        Surface::Cylinder {
            frame: Frame::world(),
            radius,
        }
    }

    /// A cylinder's ruled `v` is scaled so the region's height is an
    /// eighth of a chord step in `u`; a region already thinner than that
    /// is left alone, and a plane, which has no chord step at all, keeps
    /// its isometric scale whatever its region.
    #[test]
    fn a_ruled_direction_is_flattened_to_a_ribbon() {
        let chord = 1e-3;
        let radius = 3.0;
        let surface = cylinder(radius);
        let bounds = [
            Interval::new(0.0, core::f64::consts::TAU).unwrap(),
            Interval::new(0.0, 15.0).unwrap(),
        ];
        let steps = surface.chord_steps(chord, bounds);
        assert!(steps[0].is_finite() && !steps[1].is_finite());
        let scale = uv_scale(&surface, bounds, steps);
        let height = bounds[1].length() * scale[1];
        assert!(
            (height - RULED_RIBBON * steps[0] * scale[0]).abs() <= 1e-15,
            "the region is a ribbon an eighth of a step thick"
        );
        // A sliver shorter than the ribbon keeps the isometric scale.
        let short = [bounds[0], Interval::new(0.0, 1e-6).unwrap()];
        assert_eq!(
            uv_scale(&surface, short, surface.chord_steps(chord, short))[1],
            uv_scale(&surface, short, [f64::INFINITY; 2])[1]
        );
        // A plane is ruled both ways and is never flattened.
        let plane = Surface::Plane {
            frame: Frame::world(),
        };
        assert_eq!(
            uv_scale(&plane, bounds, plane.chord_steps(chord, bounds)),
            [1.0; 2]
        );
    }
}
