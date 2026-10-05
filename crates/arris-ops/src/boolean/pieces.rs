//! Splitting a face in its own (u, v) (ADR-0004): the arrangement of the
//! pieces of its loops and the section edges on it, walked into regions.
//! Each region is a piece of the face with a point strictly inside it,
//! which the boolean carries to 3D and classifies against the other body.
//!
//! The arrangement is a planar subdivision: nodes at every vertex image
//! (a seam vertex has two), half-edges for every loop piece — one-sided,
//! since the face has no material on the loop's right — and two for every
//! section edge. At each node the half-edges leaving it are ordered by
//! the angle of their pcurve's tangent, ties within the model's angular
//! tolerance by the signed curvature, and a tie of both is
//! [`BooleanReason::TangentContact`]. A region is walked by taking, at the end
//! of each half-edge, the next half-edge clockwise from the direction
//! one came from; a cycle turning once counter-clockwise bounds a region
//! and one turning clockwise is a hole, assigned to the innermost
//! counter-clockwise cycle that winds around it.

use core::f64::consts::{PI, TAU};
use std::collections::{BTreeMap, BTreeSet};

use arris_geom::region2::{Piece as Walk, Polygon2, discretise, interior_point};
use arris_geom::{Curve2, Surface};
use arris_math::{Interval, Point2, Point3, Precision, is_negligible, wrap_angle};
use arris_topo::{Curve2Id, EdgeId, FaceId, Model, Orientation, Shape, VertexId};

use arris_check::domain::chord;

use crate::error::{BooleanReason, Fault, OpError, Reason, SplitFault};

/// A vertex of the result as the split names it before it has an id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum VRef {
    /// A vertex of an operand, kept or re-tolerated.
    Existing(VertexId),
    /// A section vertex, an index into `Interferences::vertices`.
    Section(usize),
}

/// An edge of the result as the split names it before it has an id.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) enum ERef {
    /// The `index`-th piece of an operand edge between consecutive paves;
    /// index 0 is the whole edge when it has none.
    Sub {
        /// The edge.
        edge: EdgeId,
        /// Which piece, ascending along the edge's parameter.
        index: usize,
    },
    /// A section edge, an index into `Interferences::sections`.
    Section(usize),
}

/// A piece of an operand edge between consecutive paves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct SubEdge {
    /// The parameter range on the edge's curve.
    pub range: Interval,
    /// The vertex at `range.lo()`.
    pub start: VRef,
    /// The vertex at `range.hi()`.
    pub end: VRef,
}

/// An edge that splits a face without being part of its loops, as that
/// face sees it: a section edge of one of its pairs, or a piece of an
/// edge of a coincident face lying inside it (its image).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct EdgeOnFace {
    /// The edge piece.
    pub edge: ERef,
    /// Its parameter range on its curve.
    pub range: Interval,
    /// The vertex at `range.lo()`.
    pub start: VRef,
    /// The vertex at `range.hi()`.
    pub end: VRef,
    /// Its pcurve on this face, already in the model.
    pub pcurve: Curve2Id,
    /// The other face of the pair, for an error.
    pub other: FaceId,
}

/// A piece of an operand edge that the result holds as a piece of
/// another edge — a common block of a coincident pair — as one use of
/// it by a loop sees it: what replaces the use.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Alias {
    /// The piece the result holds.
    pub edge: ERef,
    /// Its parameter range on its own curve.
    pub range: Interval,
    /// `true` when the replaced piece runs against it.
    pub reversed: bool,
    /// The vertex at `range.lo()`.
    pub start: VRef,
    /// The vertex at `range.hi()`.
    pub end: VRef,
    /// The pcurve of the held piece on this face, in this use's
    /// translate of the domain, same-parameter with `range`.
    pub pcurve: Curve2Id,
}

/// One use of an edge piece by a piece's loop, in the *stored* sense:
/// `Forward` walks along the edge's parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct PieceUse {
    /// The edge piece.
    pub edge: ERef,
    /// Along or against its parameter.
    pub orientation: Orientation,
    /// The pcurve of the use on this face.
    pub pcurve: Curve2Id,
}

/// A region of the face's arrangement: a piece of the face.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct FacePiece {
    /// The outer loop first, then the holes, each counter-clockwise about
    /// the surface's normal with the piece on its left.
    pub loops: Vec<Vec<PieceUse>>,
    /// A point strictly inside the piece, on the surface.
    pub interior: Point3,
    /// Its (u, v).
    pub uv: Point2,
}

/// A face split into its pieces.
#[derive(Debug, Clone, PartialEq)]
pub(super) struct SplitFace {
    /// `true` when nothing on the face changed — no section edge, no
    /// touched edge — so its one piece is the face itself.
    pub untouched: bool,
    /// The pieces, in the order the walk found them.
    pub pieces: Vec<FacePiece>,
}

/// A node of the arrangement: one image of a vertex in (u, v).
struct Node {
    uv: Point2,
}

/// A direction leaving a node: the tangent's angle in `[0, 2π)` and the
/// signed curvature in the direction of travel.
#[derive(Clone, Copy)]
struct Dir {
    angle: f64,
    curvature: f64,
}

/// A half-edge of the arrangement.
struct Half {
    from: usize,
    to: usize,
    pcurve: Curve2Id,
    range: Interval,
    reversed: bool,
    edge: ERef,
    orientation: Orientation,
    /// The other side, for a section edge; a loop piece has none.
    twin: Option<usize>,
    /// Leaving `from`.
    start: Dir,
    /// Leaving `to` backwards along this half-edge.
    back: Dir,
    /// What an error names for this half-edge.
    named: Shape,
}

/// An entry of a node's angular order: a half-edge leaving the node, or
/// the direction a one-sided half-edge arrives from.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Entry {
    Out(usize),
    Stub(usize),
}

/// The direction a pcurve leaves `t` in when walked along (`reversed`
/// false) or against its parameter, `None` where the tangent vanishes.
fn leaving(c: &Curve2, t: f64, reversed: bool) -> Option<Dir> {
    let e = c.eval(t);
    let d1 = if reversed { -e.d1 } else { e.d1 };
    let speed = d1.norm();
    if !(speed.is_finite() && speed > 0.0) {
        return None;
    }
    Some(Dir {
        angle: wrap_angle(d1.y.atan2(d1.x)),
        curvature: (d1.x * e.d2.y - d1.y * e.d2.x) / (speed * speed * speed),
    })
}

/// The turn from direction `from` to direction `to`, in `[−π, π)`.
fn turn(from: f64, to: f64) -> f64 {
    (to - from + PI).rem_euclid(TAU) - PI
}

/// How far the tangent of the walk `w` rotates from its start to its
/// end, by the sum of the turns between consecutive samples at the
/// walk's own segment count for `chord` — never fewer than the conic
/// minimum, so no two samples are half a turn apart.
fn rotation(w: &Walk<'_>, chord: f64) -> Option<f64> {
    let n = w.segment_count(chord).max(1);
    let mut total = 0.0;
    let mut previous: Option<f64> = None;
    for i in 0..=n {
        let s = i as f64 / n as f64;
        let s = if w.reversed { 1.0 - s } else { s };
        let dir = leaving(w.curve, w.range.lerp(s), w.reversed)?;
        if let Some(p) = previous {
            total += turn(p, dir.angle);
        }
        previous = Some(dir.angle);
    }
    Some(total)
}

/// The arrangement of one face.
struct Arrangement<'m> {
    m: &'m Model,
    face: FaceId,
    surface: &'m Surface,
    chord: f64,
    angular: f64,
    nodes: Vec<Node>,
    halves: Vec<Half>,
    /// The nodes standing for each vertex.
    images: BTreeMap<VRef, Vec<usize>>,
    /// Per node, its entries in counter-clockwise angular order.
    order: Vec<Vec<Entry>>,
}

impl<'m> Arrangement<'m> {
    fn fault(&self, fault: SplitFault) -> OpError {
        OpError::Internal(Fault::Split(fault))
    }

    /// A direction of `pcurve` at `t`, or the arrangement's fault.
    fn dir(&self, pcurve: Curve2Id, t: f64, reversed: bool) -> Result<Dir, OpError> {
        let c = self.m.curve2(pcurve)?;
        leaving(c, t, reversed).ok_or_else(|| self.fault(SplitFault::Turn { face: self.face }))
    }

    /// The (u, v) of `pcurve` at `t`.
    fn uv(&self, pcurve: Curve2Id, t: f64) -> Result<Point2, OpError> {
        Ok(self.m.curve2(pcurve)?.point(t))
    }

    /// A new node, an image of `vertex`.
    fn node(&mut self, uv: Point2, vertex: VRef) -> usize {
        self.nodes.push(Node { uv });
        let k = self.nodes.len() - 1;
        self.images.entry(vertex).or_default().push(k);
        k
    }

    /// The image of `vertex` nearest `uv` — a seam vertex has two, a
    /// period apart — or a new node when the face has none.
    fn image_near(&mut self, vertex: VRef, uv: Point2) -> usize {
        let nearest = self.images.get(&vertex).and_then(|list| {
            list.iter().copied().min_by(|&a, &b| {
                let da = (self.nodes[a].uv - uv).norm();
                let db = (self.nodes[b].uv - uv).norm();
                da.total_cmp(&db).then(a.cmp(&b))
            })
        });
        match nearest {
            Some(k) => k,
            None => self.node(uv, vertex),
        }
    }

    /// A half-edge from `from` to `to` over `pcurve` on `range`.
    #[allow(clippy::too_many_arguments)]
    fn half(
        &mut self,
        from: usize,
        to: usize,
        pcurve: Curve2Id,
        range: Interval,
        reversed: bool,
        edge: ERef,
        orientation: Orientation,
        named: Shape,
    ) -> Result<usize, OpError> {
        let (t0, t1) = if reversed {
            (range.hi(), range.lo())
        } else {
            (range.lo(), range.hi())
        };
        let start = self.dir(pcurve, t0, reversed)?;
        let back = self.dir(pcurve, t1, !reversed)?;
        self.halves.push(Half {
            from,
            to,
            pcurve,
            range,
            reversed,
            edge,
            orientation,
            twin: None,
            start,
            back,
            named,
        });
        Ok(self.halves.len() - 1)
    }

    /// The loops of the face as one-sided half-edges, each coedge cut at
    /// its paves, a piece with an alias replaced by the piece it stands
    /// for; consecutive pieces share a node by construction.
    fn loops(
        &mut self,
        sub_edges: &BTreeMap<EdgeId, Vec<SubEdge>>,
        alias: &BTreeMap<(ERef, Curve2Id), Alias>,
    ) -> Result<(), OpError> {
        let face = self.m.face(self.face)?.clone();
        for l in face.loops() {
            let mut steps: Vec<(VRef, Curve2Id, Interval, bool, ERef, Orientation, Shape)> =
                Vec::new();
            for c in l.coedges() {
                let subs = sub_edges
                    .get(&c.edge())
                    .ok_or(OpError::Internal(Fault::Invariant {
                        what: "an edge's sub-edges",
                    }))?;
                let named = Shape::new(c.edge(), Orientation::Forward);
                let reversed = c.orientation().is_reversed();
                let indices: Vec<usize> = if reversed {
                    (0..subs.len()).rev().collect()
                } else {
                    (0..subs.len()).collect()
                };
                for i in indices {
                    let s = &subs[i];
                    let e = ERef::Sub {
                        edge: c.edge(),
                        index: i,
                    };
                    let step = match alias.get(&(e, c.pcurve())) {
                        None => (
                            if reversed { s.end } else { s.start },
                            c.pcurve(),
                            s.range,
                            reversed,
                            e,
                            c.orientation(),
                            named,
                        ),
                        Some(al) => {
                            let against = reversed != al.reversed;
                            (
                                if against { al.end } else { al.start },
                                al.pcurve,
                                al.range,
                                against,
                                al.edge,
                                if against {
                                    Orientation::Reversed
                                } else {
                                    Orientation::Forward
                                },
                                named,
                            )
                        }
                    };
                    steps.push(step);
                }
            }
            let n = steps.len();
            if n == 0 {
                continue;
            }
            let mut nodes = Vec::with_capacity(n);
            for &(vertex, pcurve, range, reversed, _, _, _) in &steps {
                let t = if reversed { range.hi() } else { range.lo() };
                let uv = self.uv(pcurve, t)?;
                nodes.push(self.node(uv, vertex));
            }
            for (k, &(_, pcurve, range, reversed, edge, orientation, named)) in
                steps.iter().enumerate()
            {
                let from = nodes[k];
                let to = nodes[(k + 1) % n];
                self.half(from, to, pcurve, range, reversed, edge, orientation, named)?;
            }
        }
        Ok(())
    }

    /// The section edges and images as pairs of half-edges, their ends
    /// at the vertex images nearest the pcurve's ends, or at new interior
    /// nodes.
    fn sections(&mut self, sections: &[EdgeOnFace]) -> Result<(), OpError> {
        for s in sections {
            let uv_start = self.uv(s.pcurve, s.range.lo())?;
            let uv_end = self.uv(s.pcurve, s.range.hi())?;
            let start = self.image_near(s.start, uv_start);
            let end = self.image_near(s.end, uv_end);
            let named = Shape::new(s.other, Orientation::Forward);
            let edge = s.edge;
            let along = self.half(
                start,
                end,
                s.pcurve,
                s.range,
                false,
                edge,
                Orientation::Forward,
                named,
            )?;
            let against = self.half(
                end,
                start,
                s.pcurve,
                s.range,
                true,
                edge,
                Orientation::Reversed,
                named,
            )?;
            self.halves[along].twin = Some(against);
            self.halves[against].twin = Some(along);
        }
        Ok(())
    }

    /// The direction of an entry.
    fn entry_dir(&self, e: Entry) -> Dir {
        match e {
            Entry::Out(h) => self.halves[h].start,
            Entry::Stub(h) => self.halves[h].back,
        }
    }

    fn entry_shape(&self, e: Entry) -> Shape {
        match e {
            Entry::Out(h) | Entry::Stub(h) => self.halves[h].named,
        }
    }

    /// The angular order at every node: ascending by angle, a run of
    /// angles within the angular tolerance ordered by curvature, the
    /// list rotated to start after its widest gap so no run straddles
    /// the wrap. A run two of whose members agree in curvature too is
    /// the tangent contact the manifold result cannot hold.
    fn order(&mut self) -> Result<(), OpError> {
        let mut at: Vec<Vec<Entry>> = vec![Vec::new(); self.nodes.len()];
        for (h, half) in self.halves.iter().enumerate() {
            at[half.from].push(Entry::Out(h));
            if half.twin.is_none() {
                at[half.to].push(Entry::Stub(h));
            }
        }
        let mut order = Vec::with_capacity(at.len());
        for list in at {
            order.push(self.sorted(list)?);
        }
        self.order = order;
        Ok(())
    }

    fn sorted(&self, mut list: Vec<Entry>) -> Result<Vec<Entry>, OpError> {
        list.sort_by(|&a, &b| {
            let (da, db) = (self.entry_dir(a), self.entry_dir(b));
            da.angle.total_cmp(&db.angle).then(a_key(a).cmp(&a_key(b)))
        });
        let n = list.len();
        if n < 2 {
            return Ok(list);
        }
        // Rotate to start after the widest gap.
        let mut widest = (0usize, f64::NEG_INFINITY);
        for i in 0..n {
            let a = self.entry_dir(list[i]).angle;
            let b = self.entry_dir(list[(i + 1) % n]).angle;
            let gap = if i + 1 == n { b + TAU - a } else { b - a };
            if gap > widest.1 {
                widest = ((i + 1) % n, gap);
            }
        }
        list.rotate_left(widest.0);
        // Runs of near-equal angles, by curvature; a full tie is a
        // tangent contact.
        let mut i = 0;
        while i < n {
            let mut j = i + 1;
            while j < n && self.near(list[j - 1], list[j]) {
                j += 1;
            }
            if j - i > 1 {
                list[i..j].sort_by(|&a, &b| {
                    let (ka, kb) = (self.entry_dir(a).curvature, self.entry_dir(b).curvature);
                    ka.total_cmp(&kb).then(a_key(a).cmp(&a_key(b)))
                });
                for w in list[i..j].windows(2) {
                    let (ka, kb) = (
                        self.entry_dir(w[0]).curvature,
                        self.entry_dir(w[1]).curvature,
                    );
                    if is_negligible((ka - kb).abs(), ka.abs().max(kb.abs())) {
                        return Err(OpError::Degenerate {
                            entities: vec![
                                Shape::new(self.face, Orientation::Forward),
                                self.entry_shape(w[0]),
                                self.entry_shape(w[1]),
                            ],
                            reason: Reason::Boolean(BooleanReason::TangentContact),
                        });
                    }
                }
            }
            i = j;
        }
        Ok(list)
    }

    /// `true` when two entries' angles are within the angular tolerance.
    fn near(&self, a: Entry, b: Entry) -> bool {
        let d = (self.entry_dir(a).angle - self.entry_dir(b).angle).abs();
        d.min(TAU - d) <= self.angular
    }

    /// The half-edge that continues the region on the left of `h`: the
    /// next clockwise entry at `h`'s end from the direction `h` arrives
    /// from.
    fn next(&self, h: usize) -> Result<usize, OpError> {
        let half = &self.halves[h];
        let list = &self.order[half.to];
        let back = match half.twin {
            Some(t) => Entry::Out(t),
            None => Entry::Stub(h),
        };
        let r = list
            .iter()
            .position(|&e| e == back)
            .ok_or_else(|| self.fault(SplitFault::Turn { face: self.face }))?;
        if list.len() < 2 {
            return Err(self.fault(SplitFault::Dangling { face: self.face }));
        }
        match list[(r + list.len() - 1) % list.len()] {
            Entry::Out(g) => Ok(g),
            Entry::Stub(_) => Err(self.fault(SplitFault::Turn { face: self.face })),
        }
    }

    /// Every cycle of the walk, each a list of half-edges, in the order
    /// of their first half-edge.
    fn cycles(&self) -> Result<Vec<Vec<usize>>, OpError> {
        let mut visited = vec![false; self.halves.len()];
        let mut cycles = Vec::new();
        for h0 in 0..self.halves.len() {
            if visited[h0] {
                continue;
            }
            let mut cycle = Vec::new();
            let mut h = h0;
            loop {
                if visited[h] {
                    return Err(self.fault(SplitFault::Turn { face: self.face }));
                }
                visited[h] = true;
                cycle.push(h);
                h = self.next(h)?;
                if h == h0 {
                    break;
                }
            }
            cycles.push(cycle);
        }
        Ok(cycles)
    }

    /// The walk pieces of a cycle, for its polygon.
    fn walks(&self, cycle: &[usize]) -> Result<Vec<Walk<'m>>, OpError> {
        cycle
            .iter()
            .map(|&h| {
                let half = &self.halves[h];
                let curve = self.m.curve2(half.pcurve)?;
                Ok(Walk {
                    curve,
                    range: half.range,
                    reversed: half.reversed,
                })
            })
            .collect()
    }

    /// How many turns a cycle makes: `+1` counter-clockwise, `−1`
    /// clockwise, anything else the fault.
    fn turns(&self, cycle: &[usize], walks: &[Walk<'_>]) -> Result<i64, OpError> {
        let mut total = 0.0;
        for (k, &h) in cycle.iter().enumerate() {
            total += rotation(&walks[k], self.chord)
                .ok_or_else(|| self.fault(SplitFault::Turn { face: self.face }))?;
            let g = cycle[(k + 1) % cycle.len()];
            total += self.corner(h, g);
        }
        let turns = (total / TAU).round();
        if turns == 1.0 || turns == -1.0 {
            Ok(turns as i64)
        } else {
            Err(self.fault(SplitFault::Turn { face: self.face }))
        }
    }

    /// The turn at the node from half-edge `h` into `g`, in `[−π, π]`.
    /// At a cusp — `g` leaving back the way `h` arrived, within the
    /// angular tolerance, as where a curve leaves a line tangent to it —
    /// the angle alone cannot say which way the walk turned, and rounding
    /// would pick. The node's order has already decided it by curvature
    /// ([`Arrangement::sorted`]): the walk turns left, `+π`, round a
    /// spike between the two when `g` bends to the right of `h` walked
    /// back, and right, `−π`, otherwise.
    fn corner(&self, h: usize, g: usize) -> f64 {
        let (back, start) = (self.halves[h].back, self.halves[g].start);
        let t = turn(wrap_angle(back.angle + PI), start.angle);
        if PI - t.abs() > self.angular {
            t
        } else if start.curvature < back.curvature {
            PI
        } else {
            -PI
        }
    }

    fn uses(&self, cycle: &[usize]) -> Vec<PieceUse> {
        cycle
            .iter()
            .map(|&h| PieceUse {
                edge: self.halves[h].edge,
                orientation: self.halves[h].orientation,
                pcurve: self.halves[h].pcurve,
            })
            .collect()
    }

    /// The regions: every counter-clockwise cycle with the clockwise
    /// cycles it is the innermost to wind around, and a point inside.
    fn regions(&self) -> Result<Vec<FacePiece>, OpError> {
        let cycles = self.cycles()?;
        let mut polygons: Vec<Polygon2> = Vec::with_capacity(cycles.len());
        let mut outer: Vec<(usize, BTreeSet<usize>)> = Vec::new();
        let mut holes: Vec<usize> = Vec::new();
        for (k, cycle) in cycles.iter().enumerate() {
            let walks = self.walks(cycle)?;
            polygons.push(discretise(&walks, self.chord));
            match self.turns(cycle, &walks)? {
                1 => {
                    let nodes = cycle.iter().map(|&h| self.halves[h].from).collect();
                    outer.push((k, nodes));
                }
                _ => holes.push(k),
            }
        }
        let mut assigned: Vec<Vec<usize>> = vec![Vec::new(); outer.len()];
        for &k in &holes {
            let q = self.halves[cycles[k][0]].from;
            let uv = self.nodes[q].uv;
            let container = outer
                .iter()
                .enumerate()
                .filter(|(_, (c, nodes))| {
                    !nodes.contains(&q) && polygons[*c].winding_number(uv) != 0
                })
                .min_by(|(_, (a, _)), (_, (b, _))| {
                    polygons[*a]
                        .signed_area()
                        .abs()
                        .total_cmp(&polygons[*b].signed_area().abs())
                        .then(a.cmp(b))
                })
                .map(|(i, _)| i)
                .ok_or_else(|| self.fault(SplitFault::Hole { face: self.face }))?;
            assigned[container].push(k);
        }
        let mut pieces = Vec::with_capacity(outer.len());
        for (i, (k, _)) in outer.iter().enumerate() {
            let mut loops = vec![self.uses(&cycles[*k])];
            let mut rings = vec![polygons[*k].clone()];
            for &hole in &assigned[i] {
                loops.push(self.uses(&cycles[hole]));
                rings.push(polygons[hole].clone());
            }
            let polygons = rings;
            let clearance = polygons
                .iter()
                .map(Polygon2::chord_deviation)
                .fold(0.0, f64::max);
            let uv = interior_point(&polygons, clearance)
                .ok_or_else(|| self.fault(SplitFault::NoInterior { face: self.face }))?;
            pieces.push(FacePiece {
                loops,
                interior: self.surface.point(uv.x, uv.y),
                uv,
            });
        }
        Ok(pieces)
    }
}

/// A key that orders entries with equal directions deterministically.
fn a_key(e: Entry) -> (usize, usize) {
    match e {
        Entry::Out(h) => (h, 0),
        Entry::Stub(h) => (h, 1),
    }
}

/// The pieces of `face`: the regions of the arrangement of its loops,
/// cut at their paves into `sub_edges` with the pieces in `alias`
/// replaced, and of `sections` — the section edges and images on it. A
/// face with no section edge or image and none of its edges in
/// `touched` is untouched and has one piece, itself, still with its
/// interior point.
///
/// Errors: [`OpError::Degenerate`] with [`BooleanReason::TangentContact`] for a
/// tie at a node; [`OpError::Internal`] with a [`SplitFault`] for an
/// arrangement that is not a subdivision; [`OpError::NotFound`] for an id
/// that does not resolve.
pub(super) fn split_face(
    m: &Model,
    precision: &Precision,
    face: FaceId,
    sub_edges: &BTreeMap<EdgeId, Vec<SubEdge>>,
    touched: &BTreeSet<EdgeId>,
    sections: &[EdgeOnFace],
    alias: &BTreeMap<(ERef, Curve2Id), Alias>,
) -> Result<SplitFace, OpError> {
    let entity = m.face(face)?;
    let surface = m.surface(entity.surface())?;
    let untouched = sections.is_empty()
        && !entity
            .loops()
            .iter()
            .flat_map(|l| l.coedges())
            .any(|c| touched.contains(&c.edge()));
    let mut a = Arrangement {
        m,
        face,
        surface,
        chord: chord(m, entity, surface, entity.tolerance()),
        angular: precision.angular_tolerance,
        nodes: Vec::new(),
        halves: Vec::new(),
        images: BTreeMap::new(),
        order: Vec::new(),
    };
    a.loops(sub_edges, alias)?;
    a.sections(sections)?;
    a.order()?;
    let pieces = a.regions()?;
    Ok(SplitFace { untouched, pieces })
}
