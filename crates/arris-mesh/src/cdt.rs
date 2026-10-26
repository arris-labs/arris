//! The constrained Delaunay triangulation of a face's domain in (u, v)
//! (`docs/adr/0003-tessellation-by-cdt-through-pcurves.md`): polygons with
//! holes and optional interior points, over the exact predicates of
//! `arris_math::predicates`, with no tolerance anywhere.
//!
//! Guarantees: every input polygon segment is an edge of the result, every
//! triangle is counter-clockwise, every edge that is not a polygon segment
//! is locally Delaunay, the triangles cover exactly the region the
//! polygons wind around (non-zero winding number), and the same input
//! gives the same index lists on every platform. Bad input — a polygon of
//! fewer than three distinct points, two constraints that cross, a point
//! on a constraint it does not belong to, a point given twice — is a typed
//! [`CdtError`] naming the polygons, segments and points involved, never a
//! panic and never a guess.

mod constraint;
mod mesh;
mod winding;

use core::fmt;

use arris_geom::region2::Polygon2;
use arris_math::{Interrupted, Meter, Point2};

use mesh::Mesh;

/// One segment of one input polygon: from the polygon's point `segment`
/// to the next one, the last segment closing onto the first point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SegmentRef {
    /// Index into the `polygons` slice given to [`triangulate`].
    pub polygon: usize,
    /// Index of the segment's first point in that polygon.
    pub segment: usize,
}

impl fmt::Display for SegmentRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "polygon {} segment {}", self.polygon, self.segment)
    }
}

/// One input point of [`triangulate`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum VertexRef {
    /// A point of a polygon.
    Polygon {
        /// Index into the `polygons` slice.
        polygon: usize,
        /// Index into that polygon's points.
        vertex: usize,
    },
    /// A point of the `interior` slice.
    Interior(usize),
}

impl fmt::Display for VertexRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            VertexRef::Polygon { polygon, vertex } => {
                write!(f, "polygon {polygon} point {vertex}")
            }
            VertexRef::Interior(i) => write!(f, "interior point {i}"),
        }
    }
}

/// Why a set of polygons could not be triangulated. Every variant names
/// what it is about; the checker's L4 and L5 rows promise a valid face
/// never produces one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum CdtError {
    /// A coordinate is NaN or infinite, or so large that a bounding
    /// triangle around the input cannot be built in `f64`.
    #[error("{0} is not finite")]
    NonFinite(VertexRef),
    /// A polygon has fewer than three distinct points, so it bounds no
    /// region.
    #[error("polygon {polygon} has {points} distinct points, fewer than three")]
    Polygon {
        /// The polygon.
        polygon: usize,
        /// How many distinct points it has.
        points: usize,
    },
    /// Two input points are the same point.
    #[error("{a} and {b} are the same point")]
    Duplicate {
        /// The earlier point.
        a: VertexRef,
        /// The later one.
        b: VertexRef,
    },
    /// Two polygon segments join the same two points.
    #[error("{a} and {b} are the same segment")]
    Coincident {
        /// The earlier segment.
        a: SegmentRef,
        /// The later one.
        b: SegmentRef,
    },
    /// Two polygon segments cross in their interiors.
    #[error("{a} crosses {b}")]
    Crossing {
        /// The segment being inserted.
        a: SegmentRef,
        /// The segment already there.
        b: SegmentRef,
    },
    /// A point lies exactly on a polygon segment it is not an end of.
    #[error("{vertex} lies on {segment}")]
    OnConstraint {
        /// The point.
        vertex: VertexRef,
        /// The segment.
        segment: SegmentRef,
    },
    /// The triangulation found its own structure inconsistent: a kernel
    /// bug caught, never a fault of the input.
    #[error("kernel bug: {0}")]
    Internal(&'static str),
    /// [`triangulate_metered`] was stopped by its caller's poll or budget
    /// (ADR-0030), not by the input.
    #[error("{0}")]
    Interrupted(Interrupted),
}

/// The triangles of a region in (u, v): what [`triangulate`] returns.
///
/// The points are the input points in input order — every point of
/// `polygons[0]`, then of `polygons[1]`, and so on, then the `interior`
/// points — so a caller maps a triangle corner back to the polygon point
/// or interior point it came from through [`Triangulation2::vertex_ref`],
/// and forward through [`Triangulation2::polygon_vertex`] and
/// [`Triangulation2::interior_vertex`]. Triangles are counter-clockwise
/// in (u, v).
#[derive(Debug, Clone, PartialEq)]
pub struct Triangulation2 {
    points: Vec<Point2>,
    triangles: Vec<[usize; 3]>,
    /// `starts[k]` is the index of polygon `k`'s first point;
    /// `starts[polygons]` that of the first interior point.
    starts: Vec<usize>,
}

impl Triangulation2 {
    /// The points, in input order.
    pub fn points(&self) -> &[Point2] {
        &self.points
    }

    /// The triangles as index triples into [`Triangulation2::points`],
    /// counter-clockwise.
    pub fn triangles(&self) -> &[[usize; 3]] {
        &self.triangles
    }

    /// How many polygons were triangulated.
    pub fn polygon_count(&self) -> usize {
        self.starts.len() - 1
    }

    /// The index of point `vertex` of polygon `polygon`, or `None` when
    /// there is no such point.
    pub fn polygon_vertex(&self, polygon: usize, vertex: usize) -> Option<usize> {
        let start = *self.starts.get(polygon)?;
        let end = *self.starts.get(polygon + 1)?;
        let index = start + vertex;
        (index < end).then_some(index)
    }

    /// The index of interior point `index`, or `None` when there is no
    /// such point.
    pub fn interior_vertex(&self, index: usize) -> Option<usize> {
        let start = *self.starts.last()?;
        let i = start + index;
        (i < self.points.len()).then_some(i)
    }

    /// Which input point the point at `index` is.
    pub fn vertex_ref(&self, index: usize) -> Option<VertexRef> {
        if index >= self.points.len() {
            return None;
        }
        let interior = *self.starts.last()?;
        if index >= interior {
            return Some(VertexRef::Interior(index - interior));
        }
        let polygon = self.starts.partition_point(|&s| s <= index) - 1;
        Some(VertexRef::Polygon {
            polygon,
            vertex: index - self.starts[polygon],
        })
    }
}

/// The constrained Delaunay triangulation of the region `polygons` wind
/// around, with `interior` points as extra vertices.
///
/// The region is where the winding number of the polygons is not zero: a
/// counter-clockwise outer polygon with clockwise holes gives its interior
/// minus the holes, as a face's loops are written (`docs/DATA-MODEL.md`
/// §Entities). Every polygon segment becomes a triangle edge; every other
/// edge is locally Delaunay; the triangles are counter-clockwise. An
/// interior point outside the region is triangulated and its triangles
/// discarded with the rest of the exterior. The algorithm — incremental
/// insertion into a bounding triangle with Lawson's flips over exact
/// `orient2d` and `incircle`, each polygon segment recovered by removing
/// the triangles it crosses and retriangulating the two pseudo-polygons
/// Delaunay-wise, then the exterior and the holes told from the region by
/// counting the constraints crossed from the bounding triangle — is
/// deterministic in the input order, so two calls give identical index
/// lists on every platform.
///
/// Errors: [`CdtError::Polygon`] for a polygon of fewer than three
/// distinct points; [`CdtError::Duplicate`] when a point is given twice
/// (a point equal to its predecessor in a polygon is merged first, by
/// [`Polygon2`]); [`CdtError::Crossing`] and [`CdtError::Coincident`] when
/// two segments cross or coincide; [`CdtError::OnConstraint`] when a
/// point lies on a segment it does not end; [`CdtError::NonFinite`] for a
/// coordinate that is not.
///
/// ```
/// use arris_mesh::cdt::triangulate;
/// use arris_geom::region2::Polygon2;
/// use arris_math::Point2;
///
/// let p = |x, y| Point2::new(x, y);
/// let outer = Polygon2::from_points([p(0.0, 0.0), p(4.0, 0.0), p(4.0, 4.0), p(0.0, 4.0)]);
/// let hole = Polygon2::from_points([p(1.0, 1.0), p(1.0, 3.0), p(3.0, 3.0), p(3.0, 1.0)]);
/// let t = triangulate(&[outer, hole], &[]).unwrap();
/// assert_eq!(t.triangles().len(), 8);
/// let area: f64 = t
///     .triangles()
///     .iter()
///     .map(|&[a, b, c]| {
///         let (a, b, c) = (t.points()[a], t.points()[b], t.points()[c]);
///         ((b - a).x * (c - a).y - (b - a).y * (c - a).x) / 2.0
///     })
///     .sum();
/// assert_eq!(area, 12.0);
/// ```
pub fn triangulate(polygons: &[Polygon2], interior: &[Point2]) -> Result<Triangulation2, CdtError> {
    triangulate_metered(polygons, interior, &mut Meter::default())
}

/// [`triangulate`] that counts a step at each point inserted and each
/// polygon segment recovered, and stops with [`CdtError::Interrupted`]
/// when `meter`'s budget or poll says so (ADR-0030). Everything else
/// [`triangulate`] guarantees holds for a run that is not stopped.
///
/// ```
/// use arris_mesh::cdt::{CdtError, triangulate_metered};
/// use arris_geom::region2::Polygon2;
/// use arris_math::{Control, Meter, Point2};
///
/// let p = |x, y| Point2::new(x, y);
/// let square = Polygon2::from_points([p(0.0, 0.0), p(1.0, 0.0), p(1.0, 1.0), p(0.0, 1.0)]);
/// let mut meter = Meter::new(&Control::budget(2));
/// let stopped = triangulate_metered(&[square], &[], &mut meter);
/// assert!(matches!(stopped, Err(CdtError::Interrupted(_))));
/// ```
pub fn triangulate_metered(
    polygons: &[Polygon2],
    interior: &[Point2],
    meter: &mut Meter<'_>,
) -> Result<Triangulation2, CdtError> {
    // The points in input order, with the numbering the result keeps.
    let mut points: Vec<Point2> = Vec::new();
    let mut starts: Vec<usize> = Vec::with_capacity(polygons.len() + 1);
    for (k, polygon) in polygons.iter().enumerate() {
        starts.push(points.len());
        let ring = polygon.points();
        for (i, &p) in ring.iter().enumerate() {
            if !(p.x.is_finite() && p.y.is_finite()) {
                return Err(CdtError::NonFinite(VertexRef::Polygon {
                    polygon: k,
                    vertex: i,
                }));
            }
            points.push(p);
        }
        let distinct = ring
            .iter()
            .map(|p| (canonical(p.x).to_bits(), canonical(p.y).to_bits()))
            .collect::<std::collections::BTreeSet<_>>()
            .len();
        if distinct < 3 {
            return Err(CdtError::Polygon {
                polygon: k,
                points: distinct,
            });
        }
    }
    starts.push(points.len());
    for (i, &p) in interior.iter().enumerate() {
        if !(p.x.is_finite() && p.y.is_finite()) {
            return Err(CdtError::NonFinite(VertexRef::Interior(i)));
        }
        points.push(p);
    }
    let real = points.len();
    let name = |index: usize| -> VertexRef {
        let interior_start = starts[starts.len() - 1];
        if index >= interior_start {
            VertexRef::Interior(index - interior_start)
        } else {
            let polygon = starts.partition_point(|&s| s <= index) - 1;
            VertexRef::Polygon {
                polygon,
                vertex: index - starts[polygon],
            }
        }
    };

    let mut mesh = Mesh::new(points).map_err(|()| CdtError::NonFinite(name(0)))?;
    // Every polygon point in hierarchical order, then every segment, then
    // the interior points the same way.
    let interior_start = starts[starts.len() - 1];
    for index in bit_reversal(interior_start) {
        meter.tick().map_err(CdtError::Interrupted)?;
        mesh.insert(index, parent(index))
            .map_err(|e| e.named(&name, None))?;
    }
    for k in 0..polygons.len() {
        let (start, end) = (starts[k], starts[k + 1]);
        let n = end - start;
        for i in 0..n {
            let segment = SegmentRef {
                polygon: k,
                segment: i,
            };
            let (a, b) = (start + i, start + (i + 1) % n);
            meter.tick().map_err(CdtError::Interrupted)?;
            mesh.insert_constraint(a, b, segment)
                .map_err(|e| e.named(&name, Some(segment)))?;
        }
    }
    for offset in bit_reversal(real - interior_start) {
        let index = interior_start + offset;
        meter.tick().map_err(CdtError::Interrupted)?;
        mesh.insert(index, parent(offset).map(|p| interior_start + p))
            .map_err(|e| e.named(&name, None))?;
    }
    let winding = mesh.windings()?;
    let triangles: Vec<[usize; 3]> = mesh
        .tris
        .iter()
        .enumerate()
        .filter(|(t, tri)| {
            tri.alive && tri.v.iter().all(|&v| v < real) && winding[*t].is_some_and(|w| w != 0)
        })
        .map(|(_, tri)| {
            // Rotated so the smallest index leads: one spelling per
            // triangle, whatever slot it was made in.
            let v = tri.v;
            // Three candidates, so a minimum exists.
            let k = (0..3).min_by_key(|&i| v[i]).unwrap_or(0);
            [v[k], v[(k + 1) % 3], v[(k + 2) % 3]]
        })
        .collect();
    let mut points = mesh.points;
    points.truncate(real);
    Ok(Triangulation2 {
        points,
        triangles,
        starts,
    })
}

/// `-0.0` as `0.0`, so two points equal as `f64` have equal bits.
fn canonical(x: f64) -> f64 {
    x + 0.0
}

/// The indices `0..n` in bit-reversed order: index `0`, then the middle,
/// then the quarters, and so on, every level doubling the density along
/// the input. Consecutive input points — a loop's samples — are
/// neighbours, so inserting them in input order leaves one vertex at
/// the front with a fan every next point has to flip, quadratic in all;
/// this order keeps every fan local without a random shuffle, so the
/// result is still a function of the input alone.
fn bit_reversal(n: usize) -> Vec<usize> {
    let width = n.next_power_of_two();
    let bits = width.trailing_zeros();
    (0..width)
        .map(|r| {
            if bits == 0 {
                0
            } else {
                r.reverse_bits() >> (usize::BITS - bits)
            }
        })
        .filter(|&i| i < n)
        .collect()
}

/// The index inserted one level before `index` in [`bit_reversal`]
/// order and nearest to it along the input — `index` with its lowest
/// set bit cleared — whose triangle is where the walk to `index`'s point
/// starts. `None` for the first index.
fn parent(index: usize) -> Option<usize> {
    (index > 0).then(|| index & (index - 1))
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::predicates::{Sign, orient2d};

    fn p(x: f64, y: f64) -> Point2 {
        Point2::new(x, y)
    }

    #[test]
    fn a_triangle_is_itself() {
        let tri = Polygon2::from_points([p(0.0, 0.0), p(1.0, 0.0), p(0.0, 1.0)]);
        let t = triangulate(&[tri], &[]).unwrap();
        assert_eq!(t.triangles(), &[[0, 1, 2]]);
        assert_eq!(t.polygon_count(), 1);
        assert_eq!(t.polygon_vertex(0, 2), Some(2));
        assert_eq!(t.polygon_vertex(0, 3), None);
        assert_eq!(t.interior_vertex(0), None);
        assert_eq!(
            t.vertex_ref(1),
            Some(VertexRef::Polygon {
                polygon: 0,
                vertex: 1
            })
        );
    }

    #[test]
    fn a_clockwise_polygon_is_still_triangulated_counter_clockwise() {
        let cw = Polygon2::from_points([p(0.0, 0.0), p(0.0, 1.0), p(1.0, 1.0), p(1.0, 0.0)]);
        let t = triangulate(&[cw], &[]).unwrap();
        assert_eq!(t.triangles().len(), 2);
        for &[a, b, c] in t.triangles() {
            assert_eq!(
                orient2d(t.points()[a], t.points()[b], t.points()[c]),
                Sign::Positive
            );
        }
    }

    #[test]
    fn bad_input_is_named() {
        let two = Polygon2::from_points([p(0.0, 0.0), p(1.0, 0.0), p(1.0, 0.0)]);
        assert_eq!(
            triangulate(&[two], &[]),
            Err(CdtError::Polygon {
                polygon: 0,
                points: 2
            })
        );
        let nan = Polygon2::from_points([p(0.0, 0.0), p(1.0, 0.0), p(f64::NAN, 1.0)]);
        assert_eq!(
            triangulate(&[nan], &[]),
            Err(CdtError::NonFinite(VertexRef::Polygon {
                polygon: 0,
                vertex: 2
            }))
        );
        let square = Polygon2::from_points([p(0.0, 0.0), p(2.0, 0.0), p(2.0, 2.0), p(0.0, 2.0)]);
        assert_eq!(
            triangulate(std::slice::from_ref(&square), &[p(1.0, 0.0)]),
            Err(CdtError::OnConstraint {
                vertex: VertexRef::Interior(0),
                segment: SegmentRef {
                    polygon: 0,
                    segment: 0
                }
            })
        );
        assert_eq!(
            triangulate(std::slice::from_ref(&square), &[p(2.0, 2.0)]),
            Err(CdtError::Duplicate {
                a: VertexRef::Polygon {
                    polygon: 0,
                    vertex: 2
                },
                b: VertexRef::Interior(0)
            })
        );
        let same = Polygon2::from_points([p(0.0, 0.0), p(2.0, 0.0), p(2.0, 2.0), p(0.0, 2.0)]);
        assert_eq!(
            triangulate(&[square.clone(), same], &[]),
            Err(CdtError::Duplicate {
                a: VertexRef::Polygon {
                    polygon: 0,
                    vertex: 0
                },
                b: VertexRef::Polygon {
                    polygon: 1,
                    vertex: 0
                }
            })
        );
        // A hole that pokes out of the square: its top side crosses the
        // square's top side.
        let poking = Polygon2::from_points([p(0.5, 1.0), p(0.5, 3.0), p(1.5, 3.0), p(1.5, 1.0)]);
        assert_eq!(
            triangulate(&[square, poking], &[]),
            Err(CdtError::Crossing {
                a: SegmentRef {
                    polygon: 1,
                    segment: 0
                },
                b: SegmentRef {
                    polygon: 0,
                    segment: 2
                }
            })
        );
    }

    #[test]
    fn a_vertex_on_a_segment_is_named() {
        // The first segment passes through the fourth vertex.
        let bent = Polygon2::from_points([
            p(0.0, 0.0),
            p(4.0, 0.0),
            p(4.0, 2.0),
            p(2.0, 0.0),
            p(0.0, 2.0),
        ]);
        let err = triangulate(&[bent], &[]).unwrap_err();
        assert!(
            matches!(
                err,
                CdtError::OnConstraint {
                    vertex: VertexRef::Polygon {
                        polygon: 0,
                        vertex: 3
                    },
                    ..
                }
            ),
            "{err}"
        );
    }
}
