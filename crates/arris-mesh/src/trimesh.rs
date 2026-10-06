//! The triangle mesh and its measurements.

use core::ops::Range;
use std::collections::BTreeMap;

use arris_check::Report;
use arris_math::Interrupted;
use arris_topo::{Body, EdgeId, FaceId, NotFound};

use crate::Aabb;
use crate::cdt::CdtError;
use crate::corners::{CornerFace, Corners};

/// The triangles of one B-Rep face: a contiguous run of a
/// [`TriMesh`]'s triangle list, in the body's face iteration order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FaceRange {
    /// The face.
    pub face: FaceId,
    /// Indices into [`TriMesh::triangles`].
    pub triangles: Range<usize>,
}

/// The discretisation of one B-Rep edge: a contiguous run of a
/// [`TriMesh`]'s edge index list, forming one polyline through the mesh's
/// vertices, in the body's edge iteration order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EdgeRange {
    /// The edge.
    pub edge: EdgeId,
    /// Indices into [`TriMesh::edge_indices`].
    pub indices: Range<usize>,
}

/// Why a mesh could not be built or extended: by hand through the
/// `push_*` methods, or from a body by [`crate::tessellate`].
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MeshError {
    /// A triangle or edge index does not name a position.
    #[error("index {index} is out of range for {positions} positions")]
    IndexOutOfRange {
        /// The offending index.
        index: u32,
        /// How many positions the mesh has.
        positions: usize,
    },
    /// A face or edge range does not fit in the list it indexes.
    #[error("range {start}..{end} is out of bounds for a list of {len}")]
    RangeOutOfBounds {
        /// Range start.
        start: usize,
        /// Range end.
        end: usize,
        /// Length of the indexed list.
        len: usize,
    },
    /// A position has a non-finite coordinate.
    #[error("position {index} is not finite")]
    NonFinitePosition {
        /// The position's index.
        index: usize,
    },
    /// The body, or an entity or geometry value it refers to, does not
    /// resolve in the model.
    #[error(transparent)]
    NotFound(#[from] NotFound),
    /// A face asked for by [`crate::tessellate_faces`] is not a face of
    /// the body.
    #[error("{face} is not a face of {body}")]
    NotInBody {
        /// The face.
        face: FaceId,
        /// The body.
        body: Body,
    },
    /// Two meshes offered to [`TriMesh::weld`] discretise `edge`
    /// differently: not the same number of samples, or a sample at
    /// another position. They were meshed at different chords, or of
    /// different states of the model. Nothing is snapped (ADR-0003).
    #[error("{edge} is meshed differently in the parts to weld")]
    WeldMismatch {
        /// The edge.
        edge: EdgeId,
    },
    /// The body fails the checker at `Level::Fast` (checked in debug
    /// builds before tessellation starts, as every operation checks its
    /// input).
    #[error("{body} fails the checker:\n{report}")]
    InvalidInput {
        /// The body.
        body: Body,
        /// What it fails.
        report: Box<Report>,
    },
    /// The chord tolerance asked for is not finite and positive.
    #[error("chord tolerance {0} is not finite and positive")]
    Chord(f64),
    /// A face's domain could not be triangulated: its loops, discretised,
    /// are not the simple nested polygons the checker's L4 and L5 rows
    /// promise.
    #[error("{face}: {source}")]
    Face {
        /// The face.
        face: FaceId,
        /// What the triangulation found.
        source: CdtError,
    },
    /// The interior lattice `chord` asks for on `face` would need more
    /// than [`crate::MAX_INTERIOR_POINTS`] points: a surface whose
    /// curvature varies enormously over its domain (a torus with a
    /// minor radius far smaller than its major one, at a fine chord).
    #[error("{face}'s interior lattice would need {points} points")]
    GridTooLarge {
        /// The face.
        face: FaceId,
        /// How many points it would need.
        points: usize,
    },
    /// A corner block does not fit together, or does not fit the mesh it
    /// was offered to: parallel arrays of different lengths, a face list
    /// that is not the mesh's faces in order, a triangle list that is not
    /// parallel to the mesh's, or a face-local vertex that does not stand
    /// on the mesh vertex its triangle does (ADR-0012).
    #[error("corner block: {0}")]
    Corners(String),
    /// Tessellation's own bookkeeping broke on validated input: never a
    /// property of the body or the chord tolerance, and never the CDT's
    /// own fault, so never a [`MeshError::Face`].
    #[error("kernel bug: {0}")]
    Internal(&'static str),
    /// The caller's poll or budget stopped the tessellation (ADR-0030):
    /// nothing was built, and the same call with a larger budget or a poll
    /// that stays `false` can succeed. The model is only read, so it is
    /// untouched.
    #[error("{0}")]
    Interrupted(Interrupted),
}

impl From<Interrupted> for MeshError {
    fn from(stop: Interrupted) -> Self {
        MeshError::Interrupted(stop)
    }
}

/// An indexed triangle mesh with `f64` positions, the output type of
/// tessellation and the input of the rasteriser.
///
/// Triangles are counter-clockwise seen from outside, so a closed mesh's
/// signed volume is positive when its normals point out of the material.
/// The mesh is a buffer, not geometry: positions are plain arrays so a
/// consumer can upload them, and the per-face and per-edge ranges are what
/// let it colour or pick by B-Rep entity.
///
/// Every index is validated when it enters, so the measurements below never
/// panic; a mesh with no triangles is not closed and has no volume.
///
/// **Positions are `f64` and the kernel ships no `f32` accessor**
/// (ADR-0011). The mesh is measured as well as drawn — the fixture corpus
/// holds its signed volume to the oracle at the chord tolerance asked for
/// — so the narrowing belongs at the consumer's own boundary, where it
/// knows its buffer layout and whether to subtract a local origin first.
/// It is one line over [`TriMesh::positions`], which borrows:
///
/// ```
/// # use arris_mesh::TriMesh;
/// # let mut mesh = TriMesh::new();
/// # mesh.push_position([1.0, 2.0, 3.0]).unwrap();
/// let gpu: Vec<[f32; 3]> = mesh
///     .positions()
///     .iter()
///     .map(|p| p.map(|c| c as f32))
///     .collect();
/// assert_eq!(gpu[0], [1.0f32, 2.0, 3.0]);
/// ```
///
/// ```
/// use arris_mesh::TriMesh;
///
/// // A tetrahedron with outward normals.
/// let mut m = TriMesh::new();
/// for p in [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
///     m.push_position(p).unwrap();
/// }
/// for t in [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]] {
///     m.push_triangle(t).unwrap();
/// }
/// assert!(m.is_closed());
/// assert!((m.signed_volume().unwrap() - 1.0 / 6.0).abs() < 1e-15);
/// ```
#[derive(Debug, Clone, PartialEq, Default)]
pub struct TriMesh {
    positions: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
    faces: Vec<FaceRange>,
    edge_indices: Vec<u32>,
    edges: Vec<EdgeRange>,
    corners: Option<Corners>,
}

impl TriMesh {
    /// An empty mesh.
    pub fn new() -> Self {
        Self::default()
    }

    /// A mesh from its parts, validated once: every index names a
    /// position, every range fits its list, every position is finite.
    pub fn from_parts(
        positions: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
        faces: Vec<FaceRange>,
        edge_indices: Vec<u32>,
        edges: Vec<EdgeRange>,
    ) -> Result<Self, MeshError> {
        for (index, p) in positions.iter().enumerate() {
            if !p.iter().all(|c| c.is_finite()) {
                return Err(MeshError::NonFinitePosition { index });
            }
        }
        let n = positions.len();
        for t in &triangles {
            check_indices(t, n)?;
        }
        check_indices(&edge_indices, n)?;
        for f in &faces {
            check_range(&f.triangles, triangles.len())?;
        }
        for e in &edges {
            check_range(&e.indices, edge_indices.len())?;
        }
        Ok(TriMesh {
            positions,
            triangles,
            faces,
            edge_indices,
            edges,
            corners: None,
        })
    }

    /// Adds a position and returns its index.
    pub fn push_position(&mut self, p: [f64; 3]) -> Result<u32, MeshError> {
        if !p.iter().all(|c| c.is_finite()) {
            return Err(MeshError::NonFinitePosition {
                index: self.positions.len(),
            });
        }
        self.positions.push(p);
        Ok((self.positions.len() - 1) as u32)
    }

    /// Adds a triangle and returns its index.
    pub fn push_triangle(&mut self, t: [u32; 3]) -> Result<usize, MeshError> {
        check_indices(&t, self.positions.len())?;
        self.triangles.push(t);
        Ok(self.triangles.len() - 1)
    }

    /// Adds the triangles of one face and records their range under `face`.
    pub fn push_face(
        &mut self,
        face: FaceId,
        triangles: impl IntoIterator<Item = [u32; 3]>,
    ) -> Result<&FaceRange, MeshError> {
        let start = self.triangles.len();
        for t in triangles {
            self.push_triangle(t)?;
        }
        self.faces.push(FaceRange {
            face,
            triangles: start..self.triangles.len(),
        });
        Ok(self.faces.last().expect("just pushed"))
    }

    /// Adds the polyline of one edge, as position indices, and records its
    /// range under `edge`.
    pub fn push_edge(&mut self, edge: EdgeId, polyline: &[u32]) -> Result<&EdgeRange, MeshError> {
        check_indices(polyline, self.positions.len())?;
        let start = self.edge_indices.len();
        self.edge_indices.extend_from_slice(polyline);
        self.edges.push(EdgeRange {
            edge,
            indices: start..self.edge_indices.len(),
        });
        Ok(self.edges.last().expect("just pushed"))
    }

    /// The vertex positions.
    pub fn positions(&self) -> &[[f64; 3]] {
        &self.positions
    }

    /// The triangles as index triples, counter-clockwise from outside.
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }

    /// The per-face ranges, in face iteration order.
    pub fn faces(&self) -> &[FaceRange] {
        &self.faces
    }

    /// The concatenated edge polylines as position indices.
    pub fn edge_indices(&self) -> &[u32] {
        &self.edge_indices
    }

    /// The per-edge ranges, in edge iteration order.
    pub fn edges(&self) -> &[EdgeRange] {
        &self.edges
    }

    /// The render buffer beside this one, or `None` when it was not
    /// asked for (ADR-0012).
    ///
    /// It is present exactly when the mesh came from
    /// [`crate::tessellate_with`] with [`crate::MeshRequest::corners`]
    /// set, or from [`TriMesh::with_corners`]. It adds face-local
    /// vertices with per-corner normals and (u, v) and changes nothing
    /// here: the positions, the triangles and the ranges are the same
    /// either way.
    pub fn corners(&self) -> Option<&Corners> {
        self.corners.as_ref()
    }

    /// This mesh carrying `corners`, validated against it once.
    ///
    /// The block must describe *these* triangles of *these* faces: its
    /// face list is this mesh's faces in the same order, its triangle
    /// list is parallel to this mesh's, and each corner triangle's three
    /// face-local vertices belong to the face whose [`FaceRange`] holds
    /// that triangle and stand on the very positions the mesh's triangle
    /// names, in the same order. So the two index spaces can never drift
    /// apart.
    ///
    /// Errors: [`MeshError::Corners`] naming what does not fit. Any
    /// block already present is replaced.
    pub fn with_corners(mut self, corners: Corners) -> Result<Self, MeshError> {
        if corners.triangles().len() != self.triangles.len() {
            return Err(MeshError::Corners(format!(
                "{} corner triangles for {} mesh triangles",
                corners.triangles().len(),
                self.triangles.len()
            )));
        }
        if corners.faces().len() != self.faces.len() {
            return Err(MeshError::Corners(format!(
                "{} corner faces for {} mesh faces",
                corners.faces().len(),
                self.faces.len()
            )));
        }
        for (index, &shared) in corners.positions().iter().enumerate() {
            if shared as usize >= self.positions.len() {
                return Err(MeshError::Corners(format!(
                    "face-local vertex {index} names position {shared} of {}",
                    self.positions.len()
                )));
            }
        }
        for (cf, fr) in corners.faces().iter().zip(&self.faces) {
            if cf.face != fr.face {
                return Err(MeshError::Corners(format!(
                    "corner face {} where the mesh has {}",
                    cf.face, fr.face
                )));
            }
            for i in fr.triangles.clone() {
                let local = corners.triangles()[i];
                let shared = self.triangles[i];
                for corner in 0..3 {
                    let v = local[corner] as usize;
                    if !cf.vertices.contains(&v) {
                        return Err(MeshError::Corners(format!(
                            "{}: triangle {i}'s face-local vertex {v} is not one of its own",
                            cf.face
                        )));
                    }
                    if corners.positions()[v] != shared[corner] {
                        return Err(MeshError::Corners(format!(
                            "{}: triangle {i}'s face-local vertex {v} stands on position {}, not the {} the mesh triangle names",
                            cf.face,
                            corners.positions()[v],
                            shared[corner]
                        )));
                    }
                }
            }
        }
        self.corners = Some(corners);
        Ok(self)
    }

    /// Joins meshes of disjoint faces of one body, made at one chord, into
    /// the one mesh whose edges they share: the mesh the one-call
    /// [`crate::tessellate`] would give those faces.
    ///
    /// Positions are merged along the edges the parts have in common: the
    /// samples of an [`EdgeId`] in two parts are one position index in the
    /// result, index for index. Faces, edges and positions keep the order
    /// they are first seen in, parts in the order given, so the result
    /// depends on the order of `parts` only through that. A part's
    /// triangles are unchanged but for their indices. A [`Corners`] block
    /// is carried when every part has one, and none otherwise; its
    /// face-local vertices are never shared, so they are re-indexed and
    /// concatenated.
    ///
    /// The parts must hold different faces: a face in two parts is in the
    /// result twice, and the result is then not closed.
    ///
    /// Errors: [`MeshError::WeldMismatch`] naming the first edge, in the
    /// order of the parts, whose samples differ bit for bit between two
    /// parts. Nothing is snapped or averaged: positions are compared
    /// exactly, because an edge's samples are a function of the edge and
    /// the chord alone (ADR-0052), so a difference is a mesh of another
    /// chord or another model, never rounding.
    ///
    /// ```
    /// # use arris_debug::sample;
    /// # use arris_math::Control;
    /// # use arris_mesh::{MeshRequest, TriMesh, tessellate_faces};
    /// # use arris_topo::Model;
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    /// let request = MeshRequest::new(1e-2);
    /// let parts: Vec<TriMesh> = m
    ///     .faces(body)
    ///     .unwrap()
    ///     .iter()
    ///     .map(|f| tessellate_faces(&m, body, &[f.id], &request, &Control::NONE).unwrap())
    ///     .collect();
    /// assert!(parts.iter().all(|p| !p.is_closed()));
    /// let whole = TriMesh::weld(&parts).unwrap();
    /// assert!(whole.is_closed());
    ///
    /// // The same edge at another chord does not fit.
    /// let coarse = tessellate_faces(
    ///     &m, body, &[m.faces(body).unwrap()[0].id], &MeshRequest::new(1.0), &Control::NONE,
    /// ).unwrap();
    /// assert!(TriMesh::weld(&[parts[1].clone(), coarse]).is_err());
    /// ```
    pub fn weld(parts: &[TriMesh]) -> Result<TriMesh, MeshError> {
        let mut base = Vec::with_capacity(parts.len());
        let mut total = 0usize;
        for part in parts {
            base.push(total);
            total += part.positions.len();
        }
        let mut sets = Classes::new(total);
        // The first occurrence of each edge: (part, its run).
        let mut first: BTreeMap<EdgeId, (usize, Range<usize>)> = BTreeMap::new();
        for (p, part) in parts.iter().enumerate() {
            for range in &part.edges {
                let Some((q, seen)) = first.get(&range.edge).cloned() else {
                    first.insert(range.edge, (p, range.indices.clone()));
                    continue;
                };
                let (a, b) = (
                    &parts[q].edge_indices[seen],
                    &part.edge_indices[range.indices.clone()],
                );
                if a.len() != b.len() {
                    return Err(MeshError::WeldMismatch { edge: range.edge });
                }
                for (&i, &j) in a.iter().zip(b) {
                    let same = parts[q].positions[i as usize]
                        .iter()
                        .zip(&part.positions[j as usize])
                        .all(|(x, y)| x.to_bits() == y.to_bits());
                    if !same {
                        return Err(MeshError::WeldMismatch { edge: range.edge });
                    }
                    sets.join(base[q] + i as usize, base[p] + j as usize);
                }
            }
        }
        // Positions in first-seen order: a class is numbered where its
        // lowest member stands.
        let mut number = vec![u32::MAX; total];
        let mut positions = Vec::new();
        let mut index = vec![0u32; total];
        for (p, part) in parts.iter().enumerate() {
            for (i, &position) in part.positions.iter().enumerate() {
                let root = sets.find(base[p] + i);
                if number[root] == u32::MAX {
                    number[root] = positions.len() as u32;
                    positions.push(position);
                }
                index[base[p] + i] = number[root];
            }
        }
        let mut triangles = Vec::new();
        let mut faces = Vec::new();
        let mut edge_indices = Vec::new();
        let mut edges = Vec::new();
        let mut done: BTreeMap<EdgeId, ()> = BTreeMap::new();
        for (p, part) in parts.iter().enumerate() {
            let at = triangles.len();
            triangles.extend(
                part.triangles
                    .iter()
                    .map(|t| t.map(|i| index[base[p] + i as usize])),
            );
            faces.extend(part.faces.iter().map(|f| FaceRange {
                face: f.face,
                triangles: f.triangles.start + at..f.triangles.end + at,
            }));
            for range in &part.edges {
                if done.insert(range.edge, ()).is_some() {
                    continue;
                }
                let start = edge_indices.len();
                edge_indices.extend(
                    part.edge_indices[range.indices.clone()]
                        .iter()
                        .map(|&i| index[base[p] + i as usize]),
                );
                edges.push(EdgeRange {
                    edge: range.edge,
                    indices: start..edge_indices.len(),
                });
            }
        }
        let mesh = TriMesh::from_parts(positions, triangles, faces, edge_indices, edges)?;
        if parts.is_empty() || parts.iter().any(|p| p.corners.is_none()) {
            return Ok(mesh);
        }
        let mut shared = Vec::new();
        let mut normals = Vec::new();
        let mut uvs = Vec::new();
        let mut local = Vec::new();
        let mut runs = Vec::new();
        for (p, part) in parts.iter().enumerate() {
            let Some(c) = part.corners.as_ref() else {
                continue;
            };
            let at = shared.len();
            shared.extend(c.positions().iter().map(|&i| index[base[p] + i as usize]));
            normals.extend_from_slice(c.normals());
            uvs.extend_from_slice(c.uvs());
            local.extend(c.triangles().iter().map(|t| t.map(|i| i + at as u32)));
            runs.extend(c.faces().iter().map(|f| CornerFace {
                face: f.face,
                vertices: f.vertices.start + at..f.vertices.end + at,
                uv_box: f.uv_box,
            }));
        }
        mesh.with_corners(Corners::from_parts(shared, normals, uvs, local, runs)?)
    }

    /// The triangles of `face`, or `None` if the mesh has no range for it.
    pub fn face_triangles(&self, face: FaceId) -> Option<&[[u32; 3]]> {
        let r = self.faces.iter().find(|f| f.face == face)?;
        self.triangles.get(r.triangles.clone())
    }

    /// The polyline of `edge` as position indices, or `None` if the mesh
    /// has no range for it.
    pub fn edge_polyline(&self, edge: EdgeId) -> Option<&[u32]> {
        let r = self.edges.iter().find(|e| e.edge == edge)?;
        self.edge_indices.get(r.indices.clone())
    }

    /// The three corner positions of triangle `i`.
    pub fn triangle_positions(&self, i: usize) -> Option<[[f64; 3]; 3]> {
        let t = self.triangles.get(i)?;
        Some([self.pos(t[0]), self.pos(t[1]), self.pos(t[2])])
    }

    fn pos(&self, i: u32) -> [f64; 3] {
        // Every index was validated on entry.
        self.positions[i as usize]
    }

    /// `true` when the triangles form a closed, consistently oriented
    /// surface: every directed edge `(i, j)` occurs exactly once and so
    /// does its opposite `(j, i)`. A flipped triangle, a missing one, a
    /// degenerate one or an empty mesh all make this `false`.
    pub fn is_closed(&self) -> bool {
        if self.triangles.is_empty() {
            return false;
        }
        let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for t in &self.triangles {
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                if a == b {
                    return false;
                }
                *count.entry((a, b)).or_insert(0) += 1;
            }
        }
        count
            .iter()
            .all(|(&(a, b), &n)| n == 1 && count.get(&(b, a)) == Some(&1))
    }

    /// The enclosed volume by the divergence theorem, `Σ a · (b × c) / 6`
    /// over the triangles: positive for outward normals. `None` when the
    /// mesh is not closed, because the sum then depends on the origin and
    /// means nothing.
    ///
    /// The positions are taken about the centre of the mesh's box, where
    /// a closed mesh's sum is the same: about the world origin each term
    /// is of the order of the distance cubed, and a body small against
    /// its distance lost its volume to their cancellation — a cylinder of
    /// radius 0.1 about 120 out was 9e-9 off, relative.
    pub fn signed_volume(&self) -> Option<f64> {
        self.is_closed().then(|| {
            let centre = self.aabb().map_or([0.0; 3], |b| b.center());
            self.triangles
                .iter()
                .map(|t| {
                    let a = sub(self.pos(t[0]), centre);
                    let b = sub(self.pos(t[1]), centre);
                    let c = sub(self.pos(t[2]), centre);
                    dot(a, cross(b, c))
                })
                .sum::<f64>()
                / 6.0
        })
    }

    /// Sum of the triangle areas.
    pub fn area(&self) -> f64 {
        self.triangles
            .iter()
            .map(|t| {
                let a = self.pos(t[0]);
                let b = self.pos(t[1]);
                let c = self.pos(t[2]);
                norm(cross(sub(b, a), sub(c, a))) / 2.0
            })
            .sum::<f64>()
    }

    /// The bounding box of the positions, or `None` for no positions.
    pub fn aabb(&self) -> Option<Aabb> {
        Aabb::of_points(&self.positions)
    }
}

/// Disjoint sets over `0..n` with the lowest member as the root, so the
/// numbering a weld derives from them is deterministic.
struct Classes(Vec<usize>);

impl Classes {
    fn new(n: usize) -> Self {
        Classes((0..n).collect())
    }

    fn find(&mut self, mut i: usize) -> usize {
        while self.0[i] != i {
            self.0[i] = self.0[self.0[i]];
            i = self.0[i];
        }
        i
    }

    fn join(&mut self, a: usize, b: usize) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a.max(b)] = a.min(b);
        }
    }
}

fn check_indices(indices: &[u32], positions: usize) -> Result<(), MeshError> {
    match indices.iter().find(|&&i| i as usize >= positions) {
        Some(&index) => Err(MeshError::IndexOutOfRange { index, positions }),
        None => Ok(()),
    }
}

fn check_range(r: &Range<usize>, len: usize) -> Result<(), MeshError> {
    if r.start > r.end || r.end > len {
        return Err(MeshError::RangeOutOfBounds {
            start: r.start,
            end: r.end,
            len,
        });
    }
    Ok(())
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The cube [-1, 1]³ as 8 positions and 12 outward triangles, one
    /// `FaceRange` per face in the order -z, +z, -y, +y, -x, +x.
    pub(crate) fn cube() -> TriMesh {
        let mut m = TriMesh::new();
        for z in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for x in [-1.0, 1.0] {
                    m.push_position([x, y, z]).unwrap();
                }
            }
        }
        // Position index = x + 2y + 4z with each coordinate 0 or 1.
        let quads: [[u32; 4]; 6] = [
            [0, 2, 3, 1], // -z, normal down
            [4, 5, 7, 6], // +z
            [0, 1, 5, 4], // -y
            [2, 6, 7, 3], // +y
            [0, 4, 6, 2], // -x
            [1, 3, 7, 5], // +x
        ];
        for (i, q) in quads.iter().enumerate() {
            m.push_face(
                FaceId::new(i as u32, 0),
                [[q[0], q[1], q[2]], [q[0], q[2], q[3]]],
            )
            .unwrap();
        }
        m
    }

    #[test]
    fn cube_measures_eight_twenty_four_closed() {
        let m = cube();
        assert_eq!(m.triangles().len(), 12);
        assert_eq!(m.faces().len(), 6);
        assert!(m.is_closed());
        assert_eq!(m.signed_volume(), Some(8.0));
        assert_eq!(m.area(), 24.0);
        assert_eq!(
            m.aabb(),
            Some(Aabb {
                min: [-1.0; 3],
                max: [1.0; 3]
            })
        );
        assert_eq!(
            m.face_triangles(FaceId::new(1, 0)),
            Some(&[[4, 5, 7], [4, 7, 6]][..])
        );
        assert_eq!(m.face_triangles(FaceId::new(9, 0)), None);
    }

    #[test]
    fn one_flipped_triangle_is_not_closed() {
        let m = cube();
        let mut tris = m.triangles().to_vec();
        tris[5].swap(1, 2);
        let flipped =
            TriMesh::from_parts(m.positions().to_vec(), tris, vec![], vec![], vec![]).unwrap();
        assert!(!flipped.is_closed());
        assert_eq!(
            flipped.signed_volume(),
            None,
            "volume of an inconsistent mesh is unreliable"
        );
        assert_eq!(flipped.area(), 24.0, "area does not care about orientation");
    }

    #[test]
    fn one_missing_triangle_is_open() {
        let m = cube();
        let mut tris = m.triangles().to_vec();
        tris.pop();
        let open =
            TriMesh::from_parts(m.positions().to_vec(), tris, vec![], vec![], vec![]).unwrap();
        assert!(!open.is_closed());
        assert_eq!(
            open.signed_volume(),
            None,
            "volume of an open mesh is unreliable"
        );
        assert_eq!(open.area(), 22.0);
    }

    #[test]
    fn inside_out_cube_is_closed_with_negative_volume() {
        let m = cube();
        let tris: Vec<[u32; 3]> = m.triangles().iter().map(|t| [t[0], t[2], t[1]]).collect();
        let inv =
            TriMesh::from_parts(m.positions().to_vec(), tris, vec![], vec![], vec![]).unwrap();
        assert!(inv.is_closed());
        assert_eq!(inv.signed_volume(), Some(-8.0));
    }

    #[test]
    fn volume_is_translation_invariant_only_when_closed() {
        let m = cube();
        let shifted: Vec<[f64; 3]> = m
            .positions()
            .iter()
            .map(|p| [p[0] + 10.0, p[1] - 3.0, p[2] + 0.5])
            .collect();
        let s =
            TriMesh::from_parts(shifted, m.triangles().to_vec(), vec![], vec![], vec![]).unwrap();
        assert!((s.signed_volume().unwrap() - 8.0).abs() < 1e-12);
    }

    #[test]
    fn empty_and_degenerate_are_not_closed() {
        assert!(!TriMesh::new().is_closed());
        assert_eq!(TriMesh::new().signed_volume(), None);
        assert_eq!(TriMesh::new().aabb(), None);
        let mut m = TriMesh::new();
        m.push_position([0.0; 3]).unwrap();
        m.push_position([1.0, 0.0, 0.0]).unwrap();
        m.push_triangle([0, 1, 1]).unwrap();
        assert!(!m.is_closed());
    }

    #[test]
    fn bad_input_is_a_typed_error_not_a_panic() {
        let mut m = TriMesh::new();
        m.push_position([0.0; 3]).unwrap();
        assert_eq!(
            m.push_triangle([0, 0, 7]),
            Err(MeshError::IndexOutOfRange {
                index: 7,
                positions: 1
            })
        );
        assert_eq!(
            m.push_position([f64::NAN, 0.0, 0.0]),
            Err(MeshError::NonFinitePosition { index: 1 })
        );
        assert_eq!(
            m.push_edge(EdgeId::new(0, 0), &[0, 3]),
            Err(MeshError::IndexOutOfRange {
                index: 3,
                positions: 1
            })
        );
        let bad_range = TriMesh::from_parts(
            vec![[0.0; 3]],
            vec![],
            vec![FaceRange {
                face: FaceId::new(0, 0),
                triangles: 0..1,
            }],
            vec![],
            vec![],
        );
        assert_eq!(
            bad_range,
            Err(MeshError::RangeOutOfBounds {
                start: 0,
                end: 1,
                len: 0
            })
        );
    }

    #[test]
    fn edges_are_polylines_over_mesh_vertices() {
        let mut m = cube();
        m.push_edge(EdgeId::new(0, 0), &[0, 1]).unwrap();
        m.push_edge(EdgeId::new(1, 0), &[1, 3, 7]).unwrap();
        assert_eq!(m.edge_polyline(EdgeId::new(1, 0)), Some(&[1, 3, 7][..]));
        assert_eq!(m.edges()[1].indices, 2..5);
    }
}
