//! Recovering a polygon segment as a constrained edge.

use std::collections::BTreeMap;

use arris_math::predicates::{Sign, incircle, orient2d};

use super::mesh::*;
use super::*;

impl Mesh {
    /// Makes the segment from point `a` to point `b` an edge: finds the
    /// triangles it crosses, removes them, retriangulates the two
    /// pseudo-polygons either side Delaunay-wise, and records the edge
    /// as constrained.
    pub(super) fn insert_constraint(
        &mut self,
        a: usize,
        b: usize,
        segment: SegmentRef,
    ) -> Result<(), Fail> {
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
    pub(super) fn fill(&mut self, poly: &[usize], made: &mut Vec<usize>) -> Result<(), Fail> {
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
}
