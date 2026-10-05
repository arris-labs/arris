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

use core::fmt;
use std::collections::{BTreeMap, VecDeque};

use arris_geom::region2::Polygon2;
use arris_math::predicates::{Sign, incircle, orient2d};
use arris_math::{Interrupted, Meter, Point2};

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

/// The unordered key of an edge.
fn key(a: usize, b: usize) -> (usize, usize) {
    (a.min(b), a.max(b))
}

/// A polygon segment recorded as a constrained edge, with the direction
/// the polygon walks it in.
#[derive(Debug, Clone, Copy)]
struct Constraint {
    segment: SegmentRef,
    from: usize,
    to: usize,
}

/// One triangle: vertices counter-clockwise, and across edge `i` — from
/// `v[i]` to `v[(i + 1) % 3]` — the neighbouring triangle, `None` on the
/// bounding triangle's boundary.
#[derive(Debug, Clone, Copy)]
struct Tri {
    v: [usize; 3],
    n: [Option<usize>; 3],
    alive: bool,
}

/// What a walk found at a point.
#[derive(Debug, Clone, Copy)]
enum Location {
    Inside(usize),
    OnEdge(usize, usize),
    OnVertex(usize),
}

/// A failure inside the mesh, before the points have names.
#[derive(Debug, Clone, Copy)]
enum Fail {
    Duplicate { existing: usize, new: usize },
    OnConstraint { vertex: usize, segment: SegmentRef },
    Crossing { existing: SegmentRef },
    Coincident { existing: SegmentRef },
    Internal(&'static str),
}

impl Fail {
    fn named(self, name: &impl Fn(usize) -> VertexRef, inserting: Option<SegmentRef>) -> CdtError {
        match self {
            Fail::Duplicate { existing, new } => CdtError::Duplicate {
                a: name(existing),
                b: name(new),
            },
            Fail::OnConstraint { vertex, segment } => CdtError::OnConstraint {
                vertex: name(vertex),
                segment,
            },
            Fail::Crossing { existing } => CdtError::Crossing {
                a: inserting.unwrap_or(existing),
                b: existing,
            },
            Fail::Coincident { existing } => CdtError::Coincident {
                a: existing,
                b: inserting.unwrap_or(existing),
            },
            Fail::Internal(what) => CdtError::Internal(what),
        }
    }
}

/// The triangulation under construction: the input points followed by
/// the three vertices of the bounding triangle, the triangles in slots
/// that a removed triangle frees for the next one made, one incident
/// triangle per vertex to start a walk from, and the constrained edges.
struct Mesh {
    points: Vec<Point2>,
    tris: Vec<Tri>,
    free: Vec<usize>,
    vertex_tri: Vec<usize>,
    constrained: BTreeMap<(usize, usize), Constraint>,
    last: usize,
}

impl Mesh {
    /// The bounding triangle around `points` and nothing else. `Err` when
    /// the points span so much of `f64` that the triangle's corners are
    /// not finite.
    fn new(mut points: Vec<Point2>) -> Result<Mesh, ()> {
        let (mut lo, mut hi) = (
            Point2::new(f64::INFINITY, f64::INFINITY),
            Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY),
        );
        for p in &points {
            lo = Point2::new(lo.x.min(p.x), lo.y.min(p.y));
            hi = Point2::new(hi.x.max(p.x), hi.y.max(p.y));
        }
        if points.is_empty() {
            (lo, hi) = (Point2::origin(), Point2::origin());
        }
        // A triangle with the box's centre well inside: base six
        // half-extents wide, apex six high, on a half-extent that is at
        // least one so a box of zero size still gets a triangle.
        let r = ((hi.x - lo.x).max(hi.y - lo.y) / 2.0).max(1.0);
        let c = Point2::new((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0);
        let corners = [
            Point2::new(c.x - 6.0 * r, c.y - 3.0 * r),
            Point2::new(c.x + 6.0 * r, c.y - 3.0 * r),
            Point2::new(c.x, c.y + 6.0 * r),
        ];
        if !corners.iter().all(|p| p.x.is_finite() && p.y.is_finite()) {
            return Err(());
        }
        let n = points.len();
        points.extend(corners);
        let tris = vec![Tri {
            v: [n, n + 1, n + 2],
            n: [None; 3],
            alive: true,
        }];
        Ok(Mesh {
            points,
            tris,
            free: Vec::new(),
            vertex_tri: vec![0; n + 3],
            constrained: BTreeMap::new(),
            last: 0,
        })
    }

    fn point(&self, v: usize) -> Point2 {
        self.points[v]
    }

    /// A new counter-clockwise triangle with no neighbours yet.
    fn alloc(&mut self, v: [usize; 3]) -> usize {
        let tri = Tri {
            v,
            n: [None; 3],
            alive: true,
        };
        let t = match self.free.pop() {
            Some(t) => {
                self.tris[t] = tri;
                t
            }
            None => {
                self.tris.push(tri);
                self.tris.len() - 1
            }
        };
        for &x in &v {
            self.vertex_tri[x] = t;
        }
        self.last = t;
        t
    }

    fn kill(&mut self, t: usize) {
        self.tris[t].alive = false;
        self.free.push(t);
    }

    /// The index of the directed edge `a → b` in triangle `t`.
    fn edge_index(&self, t: usize, a: usize, b: usize) -> Result<usize, Fail> {
        let v = self.tris[t].v;
        (0..3)
            .find(|&i| v[i] == a && v[(i + 1) % 3] == b)
            .ok_or(Fail::Internal(
                "an edge is not in the triangle it was looked up in",
            ))
    }

    /// Points `t`'s edge `i` at `other`, and `other`'s reverse of it back
    /// at `t`.
    fn link(&mut self, t: usize, i: usize, other: Option<usize>) -> Result<(), Fail> {
        self.tris[t].n[i] = other;
        if let Some(o) = other {
            let v = self.tris[t].v;
            let j = self.edge_index(o, v[(i + 1) % 3], v[i])?;
            self.tris[o].n[j] = Some(t);
        }
        Ok(())
    }

    /// The vertex of `t` opposite its edge `i`.
    fn apex(&self, t: usize, i: usize) -> usize {
        self.tris[t].v[(i + 2) % 3]
    }

    /// Where `p` lies: a walk from `from` (the last triangle made when
    /// it is dead), crossing an edge the point is on the far side of,
    /// with a scan of every triangle when the walk has taken more steps
    /// than there are triangles (a walk cycles only in a non-Delaunay
    /// triangulation, which the constraints make possible).
    fn locate(&self, p: Point2, from: usize) -> Result<Location, Fail> {
        let mut t = if self.tris.get(from).is_some_and(|t| t.alive) {
            from
        } else {
            self.last
        };
        let bound = 2 * self.tris.len() + 8;
        for _ in 0..bound {
            if !self.tris[t].alive {
                break;
            }
            let signs = self.signs(t, p);
            match signs.iter().position(|&s| s == Sign::Negative) {
                None => return self.classify(t, signs),
                Some(i) => match self.tris[t].n[i] {
                    Some(next) => t = next,
                    None => break,
                },
            }
        }
        self.scan(p)
    }

    /// Every alive triangle in slot order, the first containing `p`.
    fn scan(&self, p: Point2) -> Result<Location, Fail> {
        for (t, tri) in self.tris.iter().enumerate() {
            if !tri.alive {
                continue;
            }
            let signs = self.signs(t, p);
            if signs.iter().all(|&s| s != Sign::Negative) {
                return self.classify(t, signs);
            }
        }
        Err(Fail::Internal(
            "a point is in no triangle of the bounding triangle",
        ))
    }

    /// `orient2d` of `p` against each edge of `t`.
    fn signs(&self, t: usize, p: Point2) -> [Sign; 3] {
        let v = self.tris[t].v;
        [
            orient2d(self.point(v[0]), self.point(v[1]), p),
            orient2d(self.point(v[1]), self.point(v[2]), p),
            orient2d(self.point(v[2]), self.point(v[0]), p),
        ]
    }

    /// A point with no negative sign against `t`: inside, on the one
    /// edge whose sign is zero, or on the vertex shared by the two.
    fn classify(&self, t: usize, signs: [Sign; 3]) -> Result<Location, Fail> {
        let zeros: Vec<usize> = (0..3).filter(|&i| signs[i] == Sign::Zero).collect();
        match zeros.as_slice() {
            [] => Ok(Location::Inside(t)),
            [i] => Ok(Location::OnEdge(t, *i)),
            // Edges i and j meet at the vertex they share.
            [i, j] => {
                let v = self.tris[t].v;
                let shared = if (*i + 1) % 3 == *j { v[*j] } else { v[*i] };
                Ok(Location::OnVertex(shared))
            }
            // All three signs zero: a degenerate triangle, never a
            // property of the input points alone.
            _ => Err(Fail::Internal("a triangle with three zero signs")),
        }
    }

    /// Inserts the point at index `p`: splits the triangle or edge it is
    /// in and restores the (constrained) Delaunay property by flips. The
    /// walk to it starts at the triangle recorded for `near`, an already
    /// inserted point next to it along the input.
    fn insert(&mut self, p: usize, near: Option<usize>) -> Result<(), Fail> {
        let point = self.point(p);
        let from = near.map_or(self.last, |v| self.vertex_tri[v]);
        let mut stack = Vec::new();
        match self.locate(point, from)? {
            Location::OnVertex(existing) => {
                return Err(Fail::Duplicate { existing, new: p });
            }
            Location::Inside(t) => {
                // t = (a, b, c) becomes (a, b, p), (b, c, p), (c, a, p).
                let Tri {
                    v: [a, b, c], n, ..
                } = self.tris[t];
                self.tris[t].v = [a, b, p];
                let t1 = self.alloc([b, c, p]);
                let t2 = self.alloc([c, a, p]);
                self.vertex_tri[a] = t;
                self.vertex_tri[b] = t;
                self.vertex_tri[p] = t;
                self.tris[t].n = [n[0], Some(t1), Some(t2)];
                self.tris[t1].n = [n[1], Some(t2), Some(t)];
                self.tris[t2].n = [n[2], Some(t), Some(t1)];
                self.link(t1, 0, n[1])?;
                self.link(t2, 0, n[2])?;
                stack.extend([(t, 0), (t1, 0), (t2, 0)]);
            }
            Location::OnEdge(t, i) => {
                let Tri { v, n, .. } = self.tris[t];
                let (a, b, c) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
                if let Some(constraint) = self.constrained.get(&key(a, b)) {
                    return Err(Fail::OnConstraint {
                        vertex: p,
                        segment: constraint.segment,
                    });
                }
                let (x, y) = (n[(i + 1) % 3], n[(i + 2) % 3]);
                let Some(t2) = n[i] else {
                    return Err(Fail::Internal(
                        "a point lies on the bounding triangle's boundary",
                    ));
                };
                let j = self.edge_index(t2, b, a)?;
                let d = self.apex(t2, j);
                let (z, w) = (self.tris[t2].n[(j + 1) % 3], self.tris[t2].n[(j + 2) % 3]);
                // (a, b, c) and (b, a, d) become (a, p, c), (p, b, c),
                // (b, p, d), (p, a, d).
                self.tris[t].v = [a, p, c];
                self.tris[t2].v = [b, p, d];
                let t1 = self.alloc([p, b, c]);
                let t3 = self.alloc([p, a, d]);
                for &x in &[a, c, p] {
                    self.vertex_tri[x] = t;
                }
                self.vertex_tri[b] = t2;
                self.vertex_tri[d] = t2;
                self.tris[t].n = [Some(t3), Some(t1), y];
                self.tris[t1].n = [Some(t2), x, Some(t)];
                self.tris[t2].n = [Some(t1), Some(t3), w];
                self.tris[t3].n = [Some(t), z, Some(t2)];
                self.link(t1, 1, x)?;
                self.link(t3, 1, z)?;
                stack.extend([(t, 2), (t1, 1), (t2, 2), (t3, 1)]);
            }
        }
        self.legalize(stack)
    }

    /// Lawson's flips: every edge on the stack that is not constrained
    /// and whose two triangles' fourth vertex lies strictly inside the
    /// circumcircle of the first is flipped, and the four edges around
    /// the flipped pair go on the stack. Strict, so cocircular points
    /// are never flipped back and forth.
    fn legalize(&mut self, mut stack: Vec<(usize, usize)>) -> Result<(), Fail> {
        while let Some((t, i)) = stack.pop() {
            if !self.tris[t].alive {
                continue;
            }
            let Some(t2) = self.tris[t].n[i] else {
                continue;
            };
            let v = self.tris[t].v;
            let (a, b, c) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
            if self.constrained.contains_key(&key(a, b)) {
                continue;
            }
            let j = self.edge_index(t2, b, a)?;
            let d = self.apex(t2, j);
            if incircle(self.point(a), self.point(b), self.point(c), self.point(d))
                == Sign::Positive
            {
                self.flip(t, i)?;
                stack.extend([(t, 0), (t, 1), (t2, 0), (t2, 1)]);
            }
        }
        Ok(())
    }

    /// Replaces edge `i` of `t` — `(a, b)`, with `c` opposite in `t` and
    /// `d` opposite in the neighbour — by the diagonal `(c, d)`: `t`
    /// becomes `(c, a, d)` and the neighbour `(d, b, c)`.
    fn flip(&mut self, t: usize, i: usize) -> Result<(), Fail> {
        let Tri { v, n, .. } = self.tris[t];
        let (a, b, c) = (v[i], v[(i + 1) % 3], v[(i + 2) % 3]);
        let t2 = n[i].ok_or(Fail::Internal("a flip across the boundary"))?;
        let j = self.edge_index(t2, b, a)?;
        let d = self.apex(t2, j);
        let (x, y) = (n[(i + 1) % 3], n[(i + 2) % 3]);
        let (z, w) = (self.tris[t2].n[(j + 1) % 3], self.tris[t2].n[(j + 2) % 3]);
        self.tris[t].v = [c, a, d];
        self.tris[t].n = [y, z, Some(t2)];
        self.tris[t2].v = [d, b, c];
        self.tris[t2].n = [w, x, Some(t)];
        self.link(t, 1, z)?;
        self.link(t2, 1, x)?;
        for &p in &[a, c, d] {
            self.vertex_tri[p] = t;
        }
        self.vertex_tri[b] = t2;
        self.last = t;
        Ok(())
    }

    /// The triangles around vertex `a`, counter-clockwise from the one
    /// recorded for it, each with the index of `a` in it. Every vertex is
    /// strictly inside the bounding triangle, so the fan closes.
    fn fan(&self, a: usize) -> Result<Vec<(usize, usize)>, Fail> {
        let start = self.vertex_tri[a];
        let mut out = Vec::new();
        let mut t = start;
        loop {
            if !self.tris[t].alive || out.len() > self.tris.len() {
                return Err(Fail::Internal("the fan around a vertex does not close"));
            }
            let k = self.tris[t]
                .v
                .iter()
                .position(|&x| x == a)
                .ok_or(Fail::Internal("a vertex's triangle does not contain it"))?;
            out.push((t, k));
            t = self.tris[t].n[k].ok_or(Fail::Internal("the fan around a vertex is open"))?;
            if t == start {
                return Ok(out);
            }
        }
    }

    /// Makes the segment from point `a` to point `b` an edge: finds the
    /// triangles it crosses, removes them, retriangulates the two
    /// pseudo-polygons either side Delaunay-wise, and records the edge
    /// as constrained.
    fn insert_constraint(&mut self, a: usize, b: usize, segment: SegmentRef) -> Result<(), Fail> {
        let k = key(a, b);
        if let Some(existing) = self.constrained.get(&k) {
            return Err(Fail::Coincident {
                existing: existing.segment,
            });
        }
        let constraint = Constraint {
            segment,
            from: a,
            to: b,
        };
        let (pa, pb) = (self.point(a), self.point(b));
        // Already an edge: nothing to recover.
        let fan = self.fan(a)?;
        if fan.iter().any(|&(t, k)| {
            let v = self.tris[t].v;
            v[(k + 1) % 3] == b || v[(k + 2) % 3] == b
        }) {
            self.constrained.insert(k, constraint);
            return Ok(());
        }
        // The triangle the segment leaves `a` through: (a, c, d) with c
        // strictly right of a → b and d strictly left. A vertex on the
        // segment itself is the error the checker's L5 promised away.
        let forward = |x: usize| (self.point(x) - pa).dot(&(pb - pa)) > 0.0;
        let mut first = None;
        for &(t, k) in &fan {
            let v = self.tris[t].v;
            let (c, d) = (v[(k + 1) % 3], v[(k + 2) % 3]);
            let (sc, sd) = (
                orient2d(pa, pb, self.point(c)),
                orient2d(pa, pb, self.point(d)),
            );
            for (x, s) in [(c, sc), (d, sd)] {
                if s == Sign::Zero && forward(x) {
                    return Err(Fail::OnConstraint { vertex: x, segment });
                }
            }
            if sc == Sign::Negative && sd == Sign::Positive {
                first = Some((t, c, d));
            }
        }
        let Some((mut t, mut right, mut left)) = first else {
            return Err(Fail::Internal(
                "a segment leaves its start through no triangle",
            ));
        };
        // Walk to `b`, collecting the crossed triangles and the chain of
        // vertices on each side. The crossing edge is (right, left) in
        // the triangle being left and (left, right) in the one entered.
        let mut crossed = vec![t];
        let mut lefts = vec![a, left];
        let mut rights = vec![a, right];
        loop {
            if let Some(existing) = self.constrained.get(&key(right, left)) {
                return Err(Fail::Crossing {
                    existing: existing.segment,
                });
            }
            let i = self.edge_index(t, right, left)?;
            let next = self.tris[t].n[i].ok_or(Fail::Internal(
                "a segment crosses the bounding triangle's boundary",
            ))?;
            let j = self.edge_index(next, left, right)?;
            let e = self.apex(next, j);
            crossed.push(next);
            t = next;
            if e == b {
                break;
            }
            match orient2d(pa, pb, self.point(e)) {
                Sign::Zero => return Err(Fail::OnConstraint { vertex: e, segment }),
                Sign::Positive => {
                    lefts.push(e);
                    left = e;
                }
                Sign::Negative => {
                    rights.push(e);
                    right = e;
                }
            }
        }
        lefts.push(b);
        rights.push(b);
        // The edges around the crossed strip and what lies beyond each,
        // keyed by the edge as the strip's triangles direct it.
        let crossed_set: std::collections::BTreeSet<usize> = crossed.iter().copied().collect();
        let mut outside: BTreeMap<(usize, usize), Option<usize>> = BTreeMap::new();
        for &t in &crossed {
            let Tri { v, n, .. } = self.tris[t];
            for i in 0..3 {
                if !n[i].is_some_and(|o| crossed_set.contains(&o)) {
                    outside.insert((v[i], v[(i + 1) % 3]), n[i]);
                }
            }
        }
        for &t in &crossed {
            self.kill(t);
        }
        // The pseudo-polygons, counter-clockwise with the segment first:
        // left of a → b is [a, b, lefts reversed]; right of it is
        // [b, a, rights].
        let mut left_polygon = vec![a, b];
        left_polygon.extend(lefts[1..lefts.len() - 1].iter().rev());
        let mut right_polygon = vec![b, a];
        right_polygon.extend(&rights[1..rights.len() - 1]);
        let mut made = Vec::new();
        self.fill(&left_polygon, &mut made)?;
        self.fill(&right_polygon, &mut made)?;
        // Link the new triangles to each other and to what was beyond.
        let mut by_edge: BTreeMap<(usize, usize), (usize, usize)> = BTreeMap::new();
        for &t in &made {
            let v = self.tris[t].v;
            for i in 0..3 {
                by_edge.insert((v[i], v[(i + 1) % 3]), (t, i));
            }
        }
        for &t in &made {
            let v = self.tris[t].v;
            for i in 0..3 {
                let (x, y) = (v[i], v[(i + 1) % 3]);
                if let Some(&(o, _)) = by_edge.get(&(y, x)) {
                    self.tris[t].n[i] = Some(o);
                } else if let Some(&beyond) = outside.get(&(x, y)) {
                    self.link(t, i, beyond)?;
                } else {
                    return Err(Fail::Internal("a retriangulated edge matches nothing"));
                }
            }
        }
        self.constrained.insert(k, constraint);
        let stack: Vec<(usize, usize)> = made
            .iter()
            .flat_map(|&t| (0..3).map(move |i| (t, i)))
            .collect();
        self.legalize(stack)
    }

    /// Triangulates the pseudo-polygon `poly` — counter-clockwise, its
    /// first two points the base edge — by the triangle on the base
    /// whose circumcircle holds no other vertex of the polygon, and the
    /// two sub-polygons it leaves. Every triangle made goes on `made`.
    fn fill(&mut self, poly: &[usize], made: &mut Vec<usize>) -> Result<(), Fail> {
        if poly.len() < 3 {
            return Ok(());
        }
        let (p0, p1) = (poly[0], poly[1]);
        let mut c = 2;
        for i in 3..poly.len() {
            if incircle(
                self.point(p0),
                self.point(p1),
                self.point(poly[c]),
                self.point(poly[i]),
            ) == Sign::Positive
            {
                c = i;
            }
        }
        if orient2d(self.point(p0), self.point(p1), self.point(poly[c])) != Sign::Positive {
            return Err(Fail::Internal(
                "a pseudo-polygon's triangle is not counter-clockwise",
            ));
        }
        made.push(self.alloc([p0, p1, poly[c]]));
        let mut sub = vec![poly[c], p1];
        sub.extend_from_slice(&poly[2..c]);
        self.fill(&sub, made)?;
        let mut sub = vec![p0, poly[c]];
        sub.extend_from_slice(&poly[c + 1..]);
        self.fill(&sub, made)
    }

    /// The winding number of the polygons around every alive triangle,
    /// counted exactly: zero at the bounding triangle's corners, and one
    /// more each time a constraint is crossed from its right to its left
    /// (into the region a counter-clockwise polygon bounds), one less the
    /// other way. Path-independent because every polygon is closed; a
    /// triangle reached with two different counts is a structural fault.
    fn windings(&self) -> Result<Vec<Option<i32>>, CdtError> {
        let mut winding: Vec<Option<i32>> = vec![None; self.tris.len()];
        let start = self.vertex_tri[self.points.len() - 3];
        if !self.tris[start].alive {
            return Err(CdtError::Internal(
                "the bounding triangle's corner has no triangle",
            ));
        }
        winding[start] = Some(0);
        let mut queue = VecDeque::from([start]);
        while let Some(t) = queue.pop_front() {
            let w = winding[t].unwrap_or(0);
            let Tri { v, n, .. } = self.tris[t];
            for i in 0..3 {
                let Some(t2) = n[i] else {
                    continue;
                };
                let (x, y) = (v[i], v[(i + 1) % 3]);
                let delta = match self.constrained.get(&key(x, y)) {
                    None => 0,
                    Some(c) if (c.from, c.to) == (x, y) => -1,
                    Some(_) => 1,
                };
                match winding[t2] {
                    None => {
                        winding[t2] = Some(w + delta);
                        queue.push_back(t2);
                    }
                    Some(seen) if seen != w + delta => {
                        return Err(CdtError::Internal(
                            "the constraints do not close: two winding numbers for one triangle",
                        ));
                    }
                    Some(_) => {}
                }
            }
        }
        Ok(winding)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
