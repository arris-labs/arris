//! The coincident face pairs: the paves every section vertex puts on their edges, each edge piece placed on the other face as an image or matched to a piece of its boundary as a common block.

use std::collections::BTreeSet;

use arris_geom::SurfaceIntersection;
use arris_geom::region2::Side;
use arris_topo::{EdgeId, FaceId};

use super::{Block, Build, End, g_face_of, geometry};
use crate::boolean::faces::{EdgeInfo, FaceInfo};
use crate::boolean::{CommonBlock, EdgeImage, Pave};
use crate::error::{Fault, OpError};

impl<'m, 'c> Build<'m, 'c> {
    /// Every edge of a face of a `Coincident` pair paved by every section
    /// vertex that lies on it within the vertex's tolerance and is not
    /// one of its ends: the vertices the other operand's edges made on
    /// neighbouring faces, which the pieces along the shared surface
    /// have to meet at (ADR-0004). A vertex nothing ends at — two branches
    /// of a traced section crossing on the faces' boundary and leaving
    /// both, no hit, no operand vertex and no section edge there — has
    /// nothing for the pieces to meet, and paves nothing.
    pub(super) fn pave_coincident_edges(&mut self) {
        let m = self.m;
        let mut used = vec![false; self.vertices.len()];
        for (k, v) in self.vertices.iter().enumerate() {
            used[k] = !(v.hits.is_empty() && v.crossings.is_empty() && v.existing.is_empty());
        }
        for s in &self.sections {
            for k in [s.start, s.end] {
                if let Some(u) = used.get_mut(k) {
                    *u = true;
                }
            }
        }
        let mut edges: BTreeSet<(usize, EdgeId)> = BTreeSet::new();
        for (pi, pair) in self.pairs.iter().enumerate() {
            if pair.intersection != SurfaceIntersection::Coincident {
                continue;
            }
            let (fa, fb) = self.pair_infos(pi);
            let [oa, ob] = pair.operands;
            edges.extend(fa.edges().iter().map(|&e| (oa, e)));
            edges.extend(fb.edges().iter().map(|&e| (ob, e)));
        }
        // An edge lying in a face of the other operand is paved the same
        // way, whether or not a face of its own is coincident with it: its
        // pieces inside that face are images there.
        for &(e, _) in &self.coincident {
            if let Some(operand) = self.operand_of_edge(e) {
                edges.insert((operand, e));
            }
        }
        let mut wanted: Vec<(EdgeId, f64, usize)> = Vec::new();
        for (side, id) in edges {
            let Some(e) = self.edge_info(side, id) else {
                continue;
            };
            for (k, v) in self.vertices.iter().enumerate() {
                if !used[k] {
                    continue;
                }
                let point = v.point(m);
                if e.vertex_at(point).is_some() || e.ends.iter().any(|x| v.existing.contains(&x.0))
                {
                    continue;
                }
                let Ok(projection) = e.curve.project(point) else {
                    continue;
                };
                if projection.distance > v.tolerance(m) {
                    continue;
                }
                let Some(t) = e.in_range(projection.t) else {
                    continue;
                };
                if t == e.range.lo() || t == e.range.hi() {
                    continue;
                }
                wanted.push((id, t, k));
            }
        }
        for (edge, t, vertex) in wanted {
            let list = self.paves.entry(edge).or_default();
            if list.iter().all(|p| p.vertex != vertex) {
                list.push(Pave { t, vertex });
                list.sort_by(|x, y| x.t.total_cmp(&y.t));
            }
        }
    }

    /// The piece of an edge of `other` that `block` of `e` is a piece
    /// of: an edge on the same curve (the curve–curve `Coincident`
    /// verdict, never the polygon band) whose piece overlaps it — either
    /// midpoint strictly inside the other's range — and then the same
    /// piece: both midpoints inside, the same ends. `None` when no piece
    /// of `other` overlaps; the fault when one overlaps without being the
    /// same piece, since every vertex the two edges share should have
    /// paved both.
    pub(super) fn matching_block(
        &self,
        side: usize,
        e: &EdgeInfo<'m>,
        block: &Block,
        other: &FaceInfo<'m>,
    ) -> Result<Option<(EdgeId, Block, bool)>, OpError> {
        let fault = || {
            OpError::Internal(Fault::CommonBlock {
                edge: e.id,
                face: other.id,
            })
        };
        let mid = e.curve.point(block.range.midpoint());
        let Some(other_side) = self.operand_of_face(other.id) else {
            return Ok(None);
        };
        for &gid in other.edges() {
            let same = if side < other_side {
                self.same_curve.contains(&(e.id, gid))
            } else {
                self.same_curve.contains(&(gid, e.id))
            };
            if !same {
                continue;
            }
            let Some(g) = self.edge_info(other_side, gid) else {
                continue;
            };
            let Ok(on_g) = g.curve.project(mid) else {
                continue;
            };
            for gb in self.blocks_of(g) {
                let g_mid = g.curve.point(gb.range.midpoint());
                let Ok(on_e) = e.curve.project(g_mid) else {
                    continue;
                };
                let e_in_g = Self::strictly_inside(g, gb.range, on_g.t);
                let g_in_e = Self::strictly_inside(e, block.range, on_e.t);
                if !(e_in_g || g_in_e) {
                    continue;
                }
                if !(e_in_g && g_in_e) {
                    return Err(fault());
                }
                let reversed = e.curve.eval(block.range.midpoint()).d1.dot(
                    &g.curve
                        .eval(g.in_range(on_g.t).unwrap_or(gb.range.midpoint()))
                        .d1,
                ) < 0.0;
                let (es, ee) = (self.canonical(block.start), self.canonical(block.end));
                let (gs, ge) = (self.canonical(gb.start), self.canonical(gb.end));
                let ends_match = if reversed {
                    es == ge && ee == gs
                } else {
                    es == gs && ee == ge
                };
                if !ends_match {
                    return Err(fault());
                }
                return Ok(Some((gid, gb, reversed)));
            }
        }
        Ok(None)
    }

    /// `block` of `e`, an edge of face `f` of operand `side`, as an image
    /// on `other` under pair `pi`: kept when the block lies inside that
    /// face by its polygons — within the band of a loop without being on
    /// any edge of it, the winding number decides — with its pcurve there
    /// and the edge's tolerance raised to the pcurve's residual; `None`
    /// when it lies outside.
    pub(super) fn image(
        &self,
        pi: usize,
        side: usize,
        e: &EdgeInfo<'m>,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        block: &Block,
    ) -> Result<Option<EdgeImage>, OpError> {
        let mid = e.curve.point(block.range.midpoint());
        let projection = other
            .surface
            .project(mid)
            .map_err(|err| geometry(err, e.shape(), other.shape()))?;
        let (s, shift) = other.domain.side(projection.uv);
        let uv_mid = projection.uv + shift;
        let inside = match s {
            Side::Outside => false,
            Side::Inside => true,
            Side::Boundary => other.domain.winds_around(projection.uv),
        };
        if !inside {
            return Ok(None);
        }
        let fit = e.tolerance + f.tolerance.max(other.tolerance);
        let (pcurve, residual) =
            self.pcurve_of(other, f, e.curve, block.range, uv_mid, fit, &[])?;
        let tolerance = e.tolerance.max(residual);
        if tolerance > self.precision.max_tolerance {
            return Err(OpError::Tolerance {
                entity: e.shape(),
                wanted: tolerance,
            });
        }
        let on = self
            .operand_of_face(other.id)
            .ok_or(OpError::Internal(Fault::Invariant {
                what: "the operand of an image's face",
            }))?;
        Ok(Some(EdgeImage {
            pair: pi,
            operand: side,
            on,
            edge: e.id,
            index: block.index,
            range: block.range,
            pcurve,
            tolerance,
        }))
    }

    /// Every piece of every edge of both faces of each `Coincident` pair
    /// decided against the other face: inside it, an image with its
    /// pcurve there; along its boundary, a common block with the piece
    /// of the other face's edge it coincides with, recorded once — from
    /// `b`'s side, under the first pair it is met in — with a pcurve for
    /// each use of `b`'s piece; outside it, nothing.
    pub(super) fn coincident(&mut self) -> Result<(), OpError> {
        let mut images = Vec::new();
        let mut blocks = Vec::new();
        let mut floors: Vec<(End, f64)> = Vec::new();
        for pi in 0..self.pairs.len() {
            if self.pairs[pi].intersection != SurfaceIntersection::Coincident {
                continue;
            }
            let (fa, fb) = self.pair_infos(pi);
            let [oa, ob] = self.pairs[pi].operands;
            for (side, f, other) in [(oa, fa, fb), (ob, fb, fa)] {
                for &eid in f.edges() {
                    let Some(e) = self.edge_info(side, eid) else {
                        continue;
                    };
                    for block in self.blocks_of(e) {
                        // Along the other face's boundary, by the curves: a
                        // common block, recorded once, from `b`'s side and
                        // under the first pair it is met in.
                        if let Some((gid, gb, reversed)) =
                            self.matching_block(side, e, &block, other)?
                        {
                            let seen = blocks
                                .iter()
                                .any(|x: &CommonBlock| x.b == (e.id, block.index));
                            if side == ob && !seen {
                                let (b, raised) =
                                    self.common_block(pi, e, &block, gid, &gb, reversed)?;
                                floors.extend(
                                    [block.start, block.end, gb.start, gb.end]
                                        .into_iter()
                                        .map(|end| (end, raised)),
                                );
                                blocks.push(b);
                            }
                            continue;
                        }
                        if let Some(image) = self.image(pi, side, e, f, other, &block)? {
                            floors.push((block.start, image.tolerance));
                            floors.push((block.end, image.tolerance));
                            images.push(image);
                        }
                    }
                }
            }
        }
        // An edge lying in a face of the other operand that no face of its
        // own is coincident with — a seam on a ruling two parallel walls
        // cross along, whose block of the section curve is the edge and
        // not a section edge — has no coincident neighbour to place it on
        // that face, so its pieces inside it are placed here, under the
        // pair of the first face that uses it.
        for &(eid, gid) in &self.coincident {
            let Some(side) = self.operand_of_edge(eid) else {
                continue;
            };
            let Some((other_side, other)) = (0..self.operands.len())
                .filter(|&o| o != side)
                .find_map(|o| Some((o, self.faces[o].iter().find(|g| g.id == gid)?)))
            else {
                continue;
            };
            let Some(e) = self.edge_info(side, eid) else {
                continue;
            };
            let pair_with = |f: FaceId| {
                self.pairs.iter().position(|p| {
                    if side < other_side {
                        p.a == f && p.b == gid
                    } else {
                        p.a == gid && p.b == f
                    }
                })
            };
            let owners: Vec<(&FaceInfo<'m>, Option<usize>)> = self.faces[side]
                .iter()
                .filter(|f| f.edges().contains(&eid))
                .map(|f| (f, pair_with(f.id)))
                .collect();
            let placed = owners.iter().any(|&(_, pi)| {
                pi.is_some_and(|pi| self.pairs[pi].intersection == SurfaceIntersection::Coincident)
            });
            let Some((f, pi)) = owners.iter().find_map(|&(f, pi)| Some((f, pi?))) else {
                continue;
            };
            if placed {
                continue;
            }
            for block in self.blocks_of(e) {
                // A piece along the other face's boundary is that face's
                // edge, not a curve inside it — a pipe's cap circle on the
                // bend it runs into, the two walls tangent along it — and
                // the coincident caps beside it hold the common block.
                if let Some((gid, gb, _)) = self.matching_block(side, e, &block, other)? {
                    let (ours, theirs) = ((e.id, block.index), (gid, gb.index));
                    let held = blocks.iter().any(|x: &CommonBlock| {
                        (x.a, x.b) == (theirs, ours) || (x.a, x.b) == (ours, theirs)
                    });
                    if !held {
                        return Err(OpError::Internal(Fault::Invariant {
                            what: "an edge along a face's boundary with no common block",
                        }));
                    }
                    continue;
                }
                if let Some(image) = self.image(pi, side, e, f, other, &block)? {
                    floors.push((block.start, image.tolerance));
                    floors.push((block.end, image.tolerance));
                    images.push(image);
                }
            }
        }
        // A piece of an edge that a block of a section curve lies along
        // ([`Self::along_block`]) is that block on the pair's other face:
        // its image there, where the section edge would have been.
        for along in &self.along {
            let (fa, fb) = self.pair_infos(along.pair);
            let [oa, _] = self.pairs[along.pair].operands;
            let (f, other) = if along.operand == oa {
                (fa, fb)
            } else {
                (fb, fa)
            };
            let Some(e) = self.edge_info(along.operand, along.edge) else {
                continue;
            };
            // The piece is placed once on a face, whichever pair placed it
            // first: an edge lying in the face is imaged there already.
            let on = |x: &EdgeImage| {
                let pair = &self.pairs[x.pair];
                if x.on == pair.operands[1] {
                    pair.b
                } else {
                    pair.a
                }
            };
            let held = images.iter().any(|x: &EdgeImage| {
                (x.edge, x.index, on(x)) == (along.edge, along.block.index, other.id)
            });
            if held {
                continue;
            }
            // Outside the other face, the block bounds nothing there.
            let Some(image) = self.image(along.pair, along.operand, e, f, other, &along.block)?
            else {
                continue;
            };
            floors.push((along.block.start, image.tolerance));
            floors.push((along.block.end, image.tolerance));
            images.push(image);
        }
        for (end, tolerance) in floors {
            if let End::Section(k) = self.canonical(end) {
                let v = &mut self.vertices[k];
                v.floor = v.floor.max(tolerance);
            }
        }
        self.images = images;
        self.blocks = blocks;
        Ok(())
    }

    /// A common block: `b`'s piece `block` of `e` is `a`'s piece `gb` of
    /// `gid`, with the pcurve of `a`'s piece for every use of `b`'s by a
    /// face of `b`, and the tolerance the two edges' raised to the
    /// residuals.
    pub(super) fn common_block(
        &self,
        pi: usize,
        e: &EdgeInfo<'m>,
        block: &Block,
        gid: EdgeId,
        gb: &Block,
        reversed: bool,
    ) -> Result<(CommonBlock, f64), OpError> {
        let m = self.m;
        let [oa, ob] = self.pairs[pi].operands;
        let g = self
            .edge_info(oa, gid)
            .ok_or(OpError::Internal(Fault::Invariant {
                what: "a's edge info for a common block",
            }))?;
        let g_mid = g.curve.point(gb.range.midpoint());
        // `b`'s parameter of `a`'s midpoint: where each use's own pcurve
        // gives the (u, v) the fitted one has to be placed at.
        let on_e = e
            .curve
            .project(g_mid)
            .map_err(|err| geometry(err, g.shape(), e.shape()))?;
        let t_e = e.in_range(on_e.t).unwrap_or(block.range.midpoint());
        let mut tolerance = g.tolerance.max(e.tolerance);
        let mut pcurves = Vec::new();
        for fb in &self.faces[ob] {
            if !fb.edges().contains(&e.id) {
                continue;
            }
            let face = m.face(fb.id)?;
            for c in face.loops().iter().flat_map(|l| l.coedges()) {
                if c.edge() != e.id {
                    continue;
                }
                let own = m.curve2(c.pcurve())?;
                let uv_mid = own.point(t_e);
                let fit = g.tolerance.max(e.tolerance) + fb.tolerance;
                let (pc, residual) = self.pcurve_of(
                    fb,
                    g_face_of(self, oa, gid),
                    g.curve,
                    gb.range,
                    uv_mid,
                    fit,
                    &[],
                )?;
                tolerance = tolerance.max(residual);
                pcurves.push((c.pcurve(), pc));
            }
        }
        if tolerance > self.precision.max_tolerance {
            return Err(OpError::Tolerance {
                entity: e.shape(),
                wanted: tolerance,
            });
        }
        Ok((
            CommonBlock {
                pair: pi,
                a: (gid, gb.index),
                b: (e.id, block.index),
                reversed,
                pcurves,
                tolerance,
            },
            tolerance,
        ))
    }
}
