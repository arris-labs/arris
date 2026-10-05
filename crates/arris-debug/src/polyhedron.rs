//! A polyhedron from points and faces of point indices, as the
//! [`Builder`] and [`BuildKeys`] `arris_ops::build` takes: what the
//! corpus's `polyhedron` recipe op builds, and a consumer's own topology
//! in the tests of `build` (`tests/fixtures/README.md`).
//!
//! The keys are this module's convention, not the kernel's: point `i` is
//! key `i`, face `j` is key `j`, the edge between points `a < b` is key
//! `a << 32 | b`, and the one shell and the body are key `0`.

use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, FrameError, Interval, Point2, Point3, UnitVec2, UnitVec3, Vec2, Vec3};
use arris_ops::BuildKeys;
use arris_topo::builder::{
    Assembly, BuildError, Builder, EdgeKey, EdgeSpec, FaceSpec, UseSpec, VertexKey, VertexSpec,
};
use arris_topo::entity::EdgeGeometry;
use arris_topo::{Model, Orientation};
use std::collections::BTreeMap;

/// Why [`polyhedron`] could not describe the solid.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PolyhedronError {
    /// A loop names a point index past the list.
    #[error("face {face} names point {point}, past the {count} points")]
    Index {
        /// The face.
        face: usize,
        /// The index.
        point: usize,
        /// How many points there are.
        count: usize,
    },
    /// A face has no loop, or a loop fewer than three points.
    #[error("face {face} has a loop of fewer than three points, or none")]
    Short {
        /// The face.
        face: usize,
    },
    /// A loop joins two points at one place: an edge of no length.
    #[error("points {a} and {b} coincide")]
    Coincident {
        /// The lower index.
        a: usize,
        /// The higher.
        b: usize,
    },
    /// A face's outer loop spans no plane.
    #[error("face {face} spans no plane: {error}")]
    Plane {
        /// The face.
        face: usize,
        /// The frame's refusal.
        error: FrameError,
    },
    /// [`Builder::assemble`] refused the description.
    #[error("the builder refused the polyhedron: {0}")]
    Build(#[from] BuildError),
}

/// The key of the edge between points `a` and `b`, either way round.
pub const fn edge_key(a: usize, b: usize) -> u64 {
    let (lo, hi) = if a < b { (a, b) } else { (b, a) };
    (lo as u64) << 32 | hi as u64
}

/// The solid bounded by `faces` over `points`, as a builder filled by
/// [`Builder::assemble`] and the keys that name each slot in
/// `namespace`. Each face is a list of loops of point indices; its outer
/// loop — the first — runs counter-clockwise seen from outside the
/// solid, a hole's clockwise. A face lies on the plane through its outer
/// loop's first point with its Newell normal, `X` along the loop's first
/// edge, used `Forward`; each edge is a line from its lower point index
/// to its higher, and every use has the line in its face's plane as its
/// pcurve. Tolerances are the model's `default_tolerance`. The builder
/// proves the topology closes; that the faces are planar and do not
/// cross is the checker's, when `build` finishes it.
///
/// Errors: a bad index, a short loop, coincident points, a face whose
/// outer loop spans no plane, or `assemble`'s refusal. Curves and
/// surfaces added before an error stay in the model, unreferenced.
///
/// ```
/// use arris_debug::polyhedron::polyhedron;
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let points = [
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.0, 1.0, 0.0),
///     Point3::new(0.0, 0.0, 1.0),
/// ];
/// let faces = [vec![vec![0, 2, 1]], vec![vec![0, 1, 3]], vec![vec![0, 3, 2]], vec![vec![1, 2, 3]]];
/// let (builder, keys) = polyhedron(&mut m, &points, &faces, 7).unwrap();
/// let (body, provenance) = arris_ops::build(&mut m, builder, &keys, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(body).unwrap().len(), 4);
/// assert_eq!(keys.edges.len(), 6);
/// # let _ = provenance;
/// ```
pub fn polyhedron(
    m: &mut Model,
    points: &[Point3],
    faces: &[Vec<Vec<usize>>],
    namespace: u32,
) -> Result<(Builder, BuildKeys), PolyhedronError> {
    let tolerance = m.precision().default_tolerance;
    for (face, loops) in faces.iter().enumerate() {
        if loops.is_empty() || loops.iter().any(|l| l.len() < 3) {
            return Err(PolyhedronError::Short { face });
        }
        for &point in loops.iter().flatten() {
            if point >= points.len() {
                return Err(PolyhedronError::Index {
                    face,
                    point,
                    count: points.len(),
                });
            }
        }
    }
    let mut assembly = Assembly {
        vertices: points
            .iter()
            .map(|&point| VertexSpec::New { point, tolerance })
            .collect(),
        ..Assembly::default()
    };
    // Each edge once, by its sorted pair, in the order the faces first
    // name it.
    let mut edge_of: BTreeMap<(usize, usize), usize> = BTreeMap::new();
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    for l in faces.iter().flatten() {
        for (i, &a) in l.iter().enumerate() {
            let b = l[(i + 1) % l.len()];
            let pair = (a.min(b), a.max(b));
            if pair.0 == pair.1 || (points[pair.1] - points[pair.0]).norm() <= tolerance {
                return Err(PolyhedronError::Coincident {
                    a: pair.0,
                    b: pair.1,
                });
            }
            if let std::collections::btree_map::Entry::Vacant(slot) = edge_of.entry(pair) {
                slot.insert(pairs.len());
                pairs.push(pair);
            }
        }
    }
    for &(a, b) in &pairs {
        let (p, q) = (points[a], points[b]);
        let d = q - p;
        let curve = m.add_curve(Curve::Line {
            origin: p,
            direction: UnitVec3::new_normalize(d),
        });
        let range =
            Interval::new(0.0, d.norm()).map_err(|_| PolyhedronError::Coincident { a, b })?;
        assembly.edges.push(EdgeSpec::New {
            geometry: EdgeGeometry::Curve { curve, range },
            start: VertexKey::New(a),
            end: VertexKey::New(b),
            tolerance,
        });
    }
    let mut shell = Vec::with_capacity(faces.len());
    for (face, loops) in faces.iter().enumerate() {
        let outer = &loops[0];
        let mut normal = Vec3::zeros();
        for (i, &a) in outer.iter().enumerate() {
            let (p, q) = (points[a], points[outer[(i + 1) % outer.len()]]);
            normal += Vec3::new(
                (p.y - q.y) * (p.z + q.z),
                (p.z - q.z) * (p.x + q.x),
                (p.x - q.x) * (p.y + q.y),
            );
        }
        let origin = points[outer[0]];
        let frame = Frame::new(origin, normal, points[outer[1]] - origin)
            .map_err(|error| PolyhedronError::Plane { face, error })?;
        let surface = m.add_surface(Surface::Plane { frame });
        let mut uses = Vec::with_capacity(loops.len());
        for l in loops {
            let mut walk = Vec::with_capacity(l.len());
            for (i, &a) in l.iter().enumerate() {
                let b = l[(i + 1) % l.len()];
                let pair = (a.min(b), a.max(b));
                let edge = edge_of[&pair];
                // The pcurve runs the way the edge's line does, from its
                // lower point to its higher, at the line's own parameter.
                let (p, q) = (
                    frame.to_local(points[pair.0]),
                    frame.to_local(points[pair.1]),
                );
                let pcurve = m.add_curve2(Curve2::Line {
                    origin: Point2::new(p.x, p.y),
                    direction: UnitVec2::new_normalize(Vec2::new(q.x - p.x, q.y - p.y)),
                });
                walk.push(UseSpec {
                    edge: EdgeKey::New(edge),
                    orientation: if a < b {
                        Orientation::Forward
                    } else {
                        Orientation::Reversed
                    },
                    pcurve,
                });
            }
            uses.push(walk);
        }
        shell.push(FaceSpec::New {
            surface,
            orientation: Orientation::Forward,
            loops: uses,
            tolerance,
        });
    }
    assembly.shells.push(shell);
    let (builder, slots) = Builder::assemble(m, tolerance, assembly)?;
    let keys = BuildKeys {
        namespace,
        vertices: slots
            .vertices
            .iter()
            .enumerate()
            .map(|(i, &v)| (v, i as u64))
            .collect(),
        edges: slots
            .edges
            .iter()
            .zip(&pairs)
            .map(|(&e, &(a, b))| (e, edge_key(a, b)))
            .collect(),
        faces: slots
            .faces
            .iter()
            .flatten()
            .enumerate()
            .map(|(j, &f)| (f, j as u64))
            .collect(),
        shells: vec![0],
        body: 0,
    };
    Ok((builder, keys))
}
