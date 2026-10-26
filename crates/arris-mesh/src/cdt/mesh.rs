//! The mesh under construction: triangles in slots, the walk to a point, insertion, legalisation.

use std::collections::BTreeMap;

use arris_math::Point2;
use arris_math::predicates::{Sign, incircle, orient2d};

use super::*;

/// The unordered key of an edge.
pub(super) fn key(a: usize, b: usize) -> (usize, usize) {
    (a.min(b), a.max(b))
}

/// A polygon segment recorded as a constrained edge, with the direction
/// the polygon walks it in.
#[derive(Debug, Clone, Copy)]
pub(super) struct Constraint {
    pub(super) segment: SegmentRef,
    pub(super) from: usize,
    pub(super) to: usize,
}

/// One triangle: vertices counter-clockwise, and across edge `i` — from
/// `v[i]` to `v[(i + 1) % 3]` — the neighbouring triangle, `None` on the
/// bounding triangle's boundary.
#[derive(Debug, Clone, Copy)]
pub(super) struct Tri {
    pub(super) v: [usize; 3],
    pub(super) n: [Option<usize>; 3],
    pub(super) alive: bool,
}

/// What a walk found at a point.
#[derive(Debug, Clone, Copy)]
pub(super) enum Location {
    Inside(usize),
    OnEdge(usize, usize),
    OnVertex(usize),
}

/// A failure inside the mesh, before the points have names.
#[derive(Debug, Clone, Copy)]
pub(super) enum Fail {
    Duplicate { existing: usize, new: usize },
    OnConstraint { vertex: usize, segment: SegmentRef },
    Crossing { existing: SegmentRef },
    Coincident { existing: SegmentRef },
    Internal(&'static str),
}

impl Fail {
    pub(super) fn named(
        self,
        name: &impl Fn(usize) -> VertexRef,
        inserting: Option<SegmentRef>,
    ) -> CdtError {
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
pub(super) struct Mesh {
    pub(super) points: Vec<Point2>,
    pub(super) tris: Vec<Tri>,
    pub(super) free: Vec<usize>,
    pub(super) vertex_tri: Vec<usize>,
    pub(super) constrained: BTreeMap<(usize, usize), Constraint>,
    pub(super) last: usize,
}

impl Mesh {
    /// The bounding triangle around `points` and nothing else. `Err` when
    /// the points span so much of `f64` that the triangle's corners are
    /// not finite.
    pub(super) fn new(mut points: Vec<Point2>) -> Result<Mesh, ()> {
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

    pub(super) fn point(&self, v: usize) -> Point2 {
        self.points[v]
    }

    /// A new counter-clockwise triangle with no neighbours yet.
    pub(super) fn alloc(&mut self, v: [usize; 3]) -> usize {
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

    pub(super) fn kill(&mut self, t: usize) {
        self.tris[t].alive = false;
        self.free.push(t);
    }

    /// The index of the directed edge `a → b` in triangle `t`.
    pub(super) fn edge_index(&self, t: usize, a: usize, b: usize) -> Result<usize, Fail> {
        let v = self.tris[t].v;
        (0..3)
            .find(|&i| v[i] == a && v[(i + 1) % 3] == b)
            .ok_or(Fail::Internal(
                "an edge is not in the triangle it was looked up in",
            ))
    }

    /// Points `t`'s edge `i` at `other`, and `other`'s reverse of it back
    /// at `t`.
    pub(super) fn link(&mut self, t: usize, i: usize, other: Option<usize>) -> Result<(), Fail> {
        self.tris[t].n[i] = other;
        if let Some(o) = other {
            let v = self.tris[t].v;
            let j = self.edge_index(o, v[(i + 1) % 3], v[i])?;
            self.tris[o].n[j] = Some(t);
        }
        Ok(())
    }

    /// The vertex of `t` opposite its edge `i`.
    pub(super) fn apex(&self, t: usize, i: usize) -> usize {
        self.tris[t].v[(i + 2) % 3]
    }

    /// Where `p` lies: a walk from `from` (the last triangle made when
    /// it is dead), crossing an edge the point is on the far side of,
    /// with a scan of every triangle when the walk has taken more steps
    /// than there are triangles (a walk cycles only in a non-Delaunay
    /// triangulation, which the constraints make possible).
    pub(super) fn locate(&self, p: Point2, from: usize) -> Result<Location, Fail> {
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
    pub(super) fn scan(&self, p: Point2) -> Result<Location, Fail> {
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
    pub(super) fn signs(&self, t: usize, p: Point2) -> [Sign; 3] {
        let v = self.tris[t].v;
        [
            orient2d(self.point(v[0]), self.point(v[1]), p),
            orient2d(self.point(v[1]), self.point(v[2]), p),
            orient2d(self.point(v[2]), self.point(v[0]), p),
        ]
    }

    /// A point with no negative sign against `t`: inside, on the one
    /// edge whose sign is zero, or on the vertex shared by the two.
    pub(super) fn classify(&self, t: usize, signs: [Sign; 3]) -> Result<Location, Fail> {
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
    pub(super) fn insert(&mut self, p: usize, near: Option<usize>) -> Result<(), Fail> {
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
    pub(super) fn legalize(&mut self, mut stack: Vec<(usize, usize)>) -> Result<(), Fail> {
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
    pub(super) fn flip(&mut self, t: usize, i: usize) -> Result<(), Fail> {
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
    pub(super) fn fan(&self, a: usize) -> Result<Vec<(usize, usize)>, Fail> {
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
}
