//! The render buffer beside the watertight one: face-local vertices with
//! the surface's own (u, v) and the outward normal (ADR-0012).

use core::ops::Range;

use arris_math::Interval;
use arris_topo::FaceId;

use crate::MeshError;

/// How far a corner normal's length may lie from `1` for
/// [`Corners::from_parts`] to accept it.
///
/// A `f64` direction normalised in any order is within a few ulps of
/// unit, and a consumer that built the block itself may have normalised
/// in another order than the kernel would; anything past this is a
/// direction that was never normalised, not rounding. It is a validation
/// bound on an input array, not a model tolerance: no geometric decision
/// is taken with it.
pub const NORMAL_UNIT_SLACK: f64 = 1e-12;

/// The face-local vertices of one B-Rep face: a contiguous run of a
/// [`Corners`]'s vertex arrays, in the body's face iteration order, so
/// the list of these is parallel to [`crate::TriMesh::faces`].
#[derive(Debug, Clone, PartialEq)]
pub struct CornerFace {
    /// The face.
    pub face: FaceId,
    /// Indices into [`Corners::positions`], [`Corners::normals`] and
    /// [`Corners::uvs`].
    pub vertices: Range<usize>,
    /// The (u, v) box of exactly these vertices — the tightest one, not
    /// the face's domain — so a consumer that wants `[0, 1]` texture
    /// coordinates normalises by it (ADR-0012).
    pub uv_box: [Interval; 2],
}

/// The optional render buffer of a [`crate::TriMesh`]: one vertex per
/// face per triangulation input point, each carrying the shared position
/// it stands on, the outward normal there and the surface's own (u, v)
/// (ADR-0012).
///
/// A *face-local vertex* is one input point of one face's triangulation:
/// a sample of one of its loops, or one of its interior lattice points.
/// Within a face it is shared by every triangle that uses it; across
/// faces nothing is shared at all. That is the granularity the geometry
/// has, and it is what makes the three cases a welded buffer cannot
/// carry fall out rather than need a rule:
///
/// - a **sharp edge** is two faces meeting along one shared index run,
///   and each side's corners carry their own face's normal — never an
///   average, so a box's edges do not shade as fillets;
/// - a **seam** is one index run used twice by the same face, as two
///   runs of face-local vertices whose `u` differs by exactly one period;
/// - a **pole or apex** is one shared position under a whole fan, with
///   one face-local vertex per triangle that touches it, each with its
///   own `u`.
///
/// [`Corners::triangles`] is parallel to [`crate::TriMesh::triangles`]
/// and [`Corners::faces`] to [`crate::TriMesh::faces`], so a renderer
/// uploads the three vertex arrays and the corner index buffer as they
/// stand, and reaches the watertight mesh through
/// [`Corners::positions`] whenever it needs to.
///
/// Every value is `f64` (ADR-0011, ADR-0012): the block is checked
/// against the surface on every fixture of the corpus, not only drawn.
#[derive(Debug, Clone, PartialEq)]
pub struct Corners {
    positions: Vec<u32>,
    normals: Vec<[f64; 3]>,
    uvs: Vec<[f64; 2]>,
    triangles: Vec<[u32; 3]>,
    faces: Vec<CornerFace>,
}

impl Corners {
    /// A corner block from its parts, validated once.
    ///
    /// The three vertex arrays are parallel and indexed by face-local
    /// vertex; `triangles` indexes them; `faces` runs in face iteration
    /// order and its `vertices` ranges partition the arrays in that
    /// order, so every face-local vertex belongs to exactly one face.
    /// Every normal is finite and unit within [`NORMAL_UNIT_SLACK`],
    /// every (u, v) is finite and inside its face's `uv_box`.
    ///
    /// Errors: [`MeshError::Corners`] naming what does not fit. Fitting
    /// the block to a mesh is [`crate::TriMesh::with_corners`]'s.
    pub fn from_parts(
        positions: Vec<u32>,
        normals: Vec<[f64; 3]>,
        uvs: Vec<[f64; 2]>,
        triangles: Vec<[u32; 3]>,
        faces: Vec<CornerFace>,
    ) -> Result<Self, MeshError> {
        let n = positions.len();
        if normals.len() != n || uvs.len() != n {
            return Err(MeshError::Corners(format!(
                "{n} shared-position indices, {} normals and {} (u, v)s are not parallel arrays",
                normals.len(),
                uvs.len()
            )));
        }
        for (i, normal) in normals.iter().enumerate() {
            let length = normal.iter().map(|c| c * c).sum::<f64>().sqrt();
            if !length.is_finite() || (length - 1.0).abs() > NORMAL_UNIT_SLACK {
                return Err(MeshError::Corners(format!(
                    "the normal of face-local vertex {i} has length {length}, not 1"
                )));
            }
        }
        for (i, uv) in uvs.iter().enumerate() {
            if !uv.iter().all(|c| c.is_finite()) {
                return Err(MeshError::Corners(format!(
                    "face-local vertex {i} has a non-finite (u, v)"
                )));
            }
        }
        for t in &triangles {
            if let Some(&index) = t.iter().find(|&&i| i as usize >= n) {
                return Err(MeshError::Corners(format!(
                    "corner triangle index {index} is out of range for {n} face-local vertices"
                )));
            }
        }
        let mut next = 0usize;
        for f in &faces {
            if f.vertices.start != next || f.vertices.end < f.vertices.start {
                return Err(MeshError::Corners(format!(
                    "{}'s vertices {}..{} do not continue the face before it at {next}",
                    f.face, f.vertices.start, f.vertices.end
                )));
            }
            next = f.vertices.end;
            if next > n {
                return Err(MeshError::Corners(format!(
                    "{}'s vertices end at {next}, past the {n} face-local vertices",
                    f.face
                )));
            }
            for uv in &uvs[f.vertices.clone()] {
                if !(f.uv_box[0].contains(uv[0]) && f.uv_box[1].contains(uv[1])) {
                    return Err(MeshError::Corners(format!(
                        "{}: ({}, {}) is outside the face's uv_box",
                        f.face, uv[0], uv[1]
                    )));
                }
            }
        }
        if next != n {
            return Err(MeshError::Corners(format!(
                "the faces cover {next} of the {n} face-local vertices"
            )));
        }
        Ok(Corners {
            positions,
            normals,
            uvs,
            triangles,
            faces,
        })
    }

    /// The shared [`crate::TriMesh::positions`] index of each face-local
    /// vertex: the watertight mesh's vertex it stands on.
    pub fn positions(&self) -> &[u32] {
        &self.positions
    }

    /// The outward unit normal at each face-local vertex, in the face
    /// use's sense — the surface's own normal flipped where the face is
    /// used `Reversed`, so it points out of the material, and never
    /// averaged with a neighbouring face's.
    pub fn normals(&self) -> &[[f64; 3]] {
        &self.normals
    }

    /// The surface's own parameters at each face-local vertex, never
    /// normalised: a cylinder's `u` is an angle in radians, a NURBS
    /// face's are its knot ranges (ADR-0012).
    pub fn uvs(&self) -> &[[f64; 2]] {
        &self.uvs
    }

    /// The triangles over the face-local vertices, parallel to
    /// [`crate::TriMesh::triangles`] and wound the same way: triangle
    /// `i` here has the corners of triangle `i` there, in the same
    /// order.
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }

    /// The per-face vertex runs, in face iteration order and parallel to
    /// [`crate::TriMesh::faces`].
    pub fn faces(&self) -> &[CornerFace] {
        &self.faces
    }

    /// The run of `face`, or `None` if the block has none for it.
    pub fn face(&self, face: FaceId) -> Option<&CornerFace> {
        self.faces.iter().find(|f| f.face == face)
    }
}
