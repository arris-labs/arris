//! The section curves of the face pairs, cut into blocks and kept as section edges with their pcurves, or laid along an operand edge as an image.

use arris_geom::{
    Curve, Curve2, MeetKind, SurfaceIntersection, curves_coincide, pcurve_ending_on, pcurve_on,
};
use arris_math::{
    Interval, Point2, Point3, RELATIVE_ROUNDING, Tolerance, Vec2, period_end, wrap_into,
};
use arris_topo::{EdgeId, FaceId};

use arris_check::domain::bands;

use super::{Along, Block, Build, End, VertexBuild, geometry, samples, tolerance_of};
use crate::boolean::faces::{EdgeInfo, FaceInfo};
use crate::boolean::{Contact, Pave, SectionCurve, SectionEdge, VertexSource, meet_curves};
use crate::error::{Fault, OpError};

impl<'m, 'c> Build<'m, 'c> {
    /// The section curves of every pair, paved and cut into blocks, the
    /// blocks interior to both faces kept as section edges. A pair's
    /// crossing curves are its sections whatever else its `Meets` holds:
    /// a touching curve beside them — a pipe's bend tangent to the
    /// straight run along the tube circle and crossing it in a quartic —
    /// is [`Self::contacts`]'.
    pub(super) fn sections(&mut self) -> Result<(), OpError> {
        for pi in 0..self.pairs.len() {
            let curves: Vec<(usize, Curve)> =
                meet_curves(&self.pairs[pi].intersection, MeetKind::Crossing)
                    .map(|(ci, c)| (ci, c.clone()))
                    .collect();
            for (ci, curve) in &curves {
                self.section_curve(pi, *ci, curve)?;
            }
        }
        Ok(())
    }

    /// The touching curves of every pair, paved by the touches and cut
    /// into blocks, the blocks interior to both faces kept as contacts.
    pub(super) fn contacts(&mut self) -> Result<(), OpError> {
        for pi in 0..self.pairs.len() {
            let curves: Vec<(usize, Curve)> =
                meet_curves(&self.pairs[pi].intersection, MeetKind::Touch)
                    .map(|(ci, c)| (ci, c.clone()))
                    .collect();
            for (ci, curve) in &curves {
                self.contact_curve(pi, *ci, curve)?;
            }
        }
        Ok(())
    }

    /// One tangent curve: its paves are the hits of either face's edges
    /// on the other face that lie on it — where the curve leaves one
    /// face inside the other, a touch, since every curve in a face
    /// tangent to the other surface is tangent to it there — and each
    /// block between consecutive paves whose midpoint is inside both
    /// faces is a contact. A block that is interior to both faces has a
    /// pave at each end: the ends of the overlap of the two faces' spans
    /// along it are each an end of one span inside the other. On a
    /// closed curve — a ball in a bore of its radius, along the circle
    /// they share — the last block wraps round to the first pave, as a
    /// section loop's does, and a curve no edge reaches is one block, a
    /// contact when it is interior to both faces.
    pub(super) fn contact_curve(
        &mut self,
        pi: usize,
        ci: usize,
        curve: &Curve,
    ) -> Result<(), OpError> {
        let [oa, ob] = self.pairs[pi].operands;
        let (ia, ib) = self.pair_faces[pi];
        let (fa, fb) = (&self.faces[oa][ia], &self.faces[ob][ib]);
        let mut paves: Vec<f64> = Vec::new();
        for (k, h) in self.hits.iter().enumerate() {
            let on_pair = (h.face == fb.id && fa.edges().contains(&h.edge))
                || (h.face == fa.id && fb.edges().contains(&h.edge));
            if !on_pair {
                continue;
            }
            let Ok(projection) = curve.project(h.point) else {
                continue;
            };
            if projection.distance <= self.hit_tolerance[k] {
                paves.push(curve.period().map_or(projection.t, |p| {
                    wrap_into(projection.t, curve.domain().lo(), p)
                }));
            }
        }
        paves.sort_by(f64::total_cmp);
        let mut blocks: Vec<(f64, f64)> = paves.windows(2).map(|w| (w[0], w[1])).collect();
        if let Some(period) = curve.period() {
            match (paves.first(), paves.last()) {
                (Some(&first), Some(&last)) => {
                    blocks.push((last, (first + period).min(period_end(last, period))));
                }
                _ => {
                    let lo = curve.domain().lo();
                    blocks.push((lo, period_end(lo, period)));
                }
            }
        }
        for (lo, hi) in blocks {
            let Ok(range) = Interval::new(lo, hi) else {
                continue;
            };
            if range.length() <= 0.0 {
                continue;
            }
            let point = curve.point(range.midpoint());
            let Some(uv) = Self::inside_both(fa, fb, point) else {
                continue;
            };
            self.contacts.push(Contact {
                pair: pi,
                curve: ci,
                range,
                point,
                uv,
            });
        }
        Ok(())
    }

    /// One section curve: its paves from the vertices on it, a seed
    /// vertex for a closed curve with none, then its blocks.
    pub(super) fn section_curve(
        &mut self,
        pi: usize,
        ci: usize,
        curve: &Curve,
    ) -> Result<(), OpError> {
        let m = self.m;
        let [oa, ob] = self.pairs[pi].operands;
        let (ia, ib) = self.pair_faces[pi];
        let (fa, fb) = (&self.faces[oa][ia], &self.faces[ob][ib]);
        let periodic = curve.period().is_some();
        // An open curve that ends on a section vertex is paved at that end
        // — at both, for a traced branch that leaves a singular point and
        // comes back to it, which a projection finds once (ADR-0018).
        let mut ends: Vec<Pave> = Vec::new();
        if !periodic {
            let domain = curve.domain();
            for t in [domain.lo(), domain.hi()] {
                if !t.is_finite() {
                    continue;
                }
                let at = curve.point(t);
                for (k, v) in self.vertices.iter().enumerate() {
                    if (v.point(m) - at).norm() <= v.tolerance(m) {
                        ends.push(Pave { t, vertex: k });
                    }
                }
            }
        }
        let mut paves: Vec<Pave> = Vec::new();
        for (k, v) in self.vertices.iter().enumerate() {
            let point = v.point(m);
            let Ok(projection) = curve.project(point) else {
                continue;
            };
            let at_an_end = ends.iter().any(|e| {
                e.vertex == k && (curve.point(e.t) - projection.point).norm() <= v.tolerance(m)
            });
            if at_an_end {
                continue;
            }
            if projection.distance <= v.tolerance(m) {
                // Where a section crossing of this curve and another, or
                // a triple point found on this curve, is one of the
                // vertex's members, the curve is paved at that member's
                // own parameter, as an edge is at its hit's: the curves
                // then end on the same point whatever the vertex's
                // representative one is.
                let t = v
                    .section_crossings
                    .iter()
                    .map(|&x| &self.section_crossings[x])
                    .filter(|x| x.pair == pi && x.curves[0] != x.curves[1])
                    .find_map(|x| x.curves.iter().position(|&c| c == ci).map(|i| x.t[i]))
                    .or_else(|| {
                        v.triple_points
                            .iter()
                            .map(|&x| &self.triple_points[x])
                            .find(|x| x.pair == pi && x.curve == ci)
                            .map(|x| x.t)
                    })
                    .unwrap_or(projection.t);
                paves.push(Pave {
                    t: curve
                        .period()
                        .map_or(t, |p| wrap_into(t, curve.domain().lo(), p)),
                    vertex: k,
                });
            }
        }
        paves.extend(ends);
        paves.sort_by(|x, y| x.t.total_cmp(&y.t).then(x.vertex.cmp(&y.vertex)));
        let curve_index = self.curves.len();
        if paves.is_empty() && periodic {
            // Nothing cuts it: it is interior to both faces or clear of
            // one, and only in the first case does it become an edge, from
            // its own point at the start of its domain.
            let domain = curve.domain();
            if Self::inside_both(fa, fb, curve.point(domain.midpoint())).is_some() {
                self.vertices.push(VertexBuild {
                    points: vec![curve.point(domain.lo())],
                    on_edge: None,
                    base: fa.tolerance.max(fb.tolerance),
                    floor: 0.0,
                    hits: Vec::new(),
                    crossings: Vec::new(),
                    section_crossings: Vec::new(),
                    triple_points: Vec::new(),
                    existing: Vec::new(),
                    source: VertexSource::CurveStart {
                        pair: pi,
                        curve: ci,
                    },
                });
                paves.push(Pave {
                    t: domain.lo(),
                    vertex: self.vertices.len() - 1,
                });
            }
        }
        // The blocks between consecutive paves; on a periodic curve the
        // last wraps round to the first.
        let mut blocks: Vec<(f64, f64, usize, usize)> = Vec::new();
        for w in paves.windows(2) {
            blocks.push((w[0].t, w[1].t, w[0].vertex, w[1].vertex));
        }
        if periodic {
            if let (Some(first), Some(last)) = (paves.first(), paves.last()) {
                let period = curve.period().ok_or(OpError::Internal(Fault::Invariant {
                    what: "a periodic curve's period",
                }))?;
                // A block that wraps ends one period after the first
                // pave, and never more than one period after the last —
                // with a single pave the two are the same turn, and the
                // sum can round one unit in the last place past it
                // (the checker's E1).
                let hi = (first.t + period).min(period_end(last.t, period));
                blocks.push((last.t, hi, last.vertex, first.vertex));
            }
        }
        // The operand edges of the two faces that lie on this curve: a
        // block that is a piece of one of them is that edge, not a section
        // edge, whatever the polygons' band says of its midpoint — its
        // split of the other face, where one is needed, is the edge's
        // image through the coincident neighbour (ADR-0004).
        let mut along: Vec<&EdgeInfo<'m>> = Vec::new();
        for (side, f, other) in [(oa, fa, fb), (ob, fb, fa)] {
            for &eid in f.edges() {
                let Some(e) = self.edge_info(side, eid) else {
                    continue;
                };
                let tol =
                    tolerance_of(&self.precision, e.tolerance, fa.tolerance.max(fb.tolerance));
                // The surfaces first: an edge whose other face lies on
                // the other face's surface is on this pair's section, and
                // along this branch when it lies on it at all.
                if let Some(on) = self.known_by_surfaces(side, e, f, other, curve, tol) {
                    if on {
                        along.push(e);
                    }
                    continue;
                }
                // Only the verdict: an edge in the plane of a planar
                // section conic — a rim circle beside the ellipse its cap
                // plane cuts from the other wall — has no closed form for
                // where the two meet, and needs none here.
                if self
                    .metered(|mt| curves_coincide(curve, e.curve, tol, mt))
                    .map_err(|err| geometry(err, fa.shape(), e.shape()))?
                {
                    along.push(e);
                }
            }
        }
        let is_an_edge: Vec<bool> = blocks
            .iter()
            .map(|&(lo, hi, _, _)| {
                let mid = curve.point(0.5 * (lo + hi));
                along.iter().any(|e| {
                    e.curve
                        .project(mid)
                        .is_ok_and(|on| Self::strictly_inside(e, e.range, on.t))
                })
            })
            .collect();
        drop(along);
        let mut edges = Vec::new();
        for (k, (lo, hi, start, end)) in blocks.into_iter().enumerate() {
            if is_an_edge[k] {
                continue;
            }
            let Ok(range) = Interval::new(lo, hi) else {
                continue;
            };
            if range.length() <= 0.0 {
                continue;
            }
            // Along an edge of one face, the block is on that face's
            // boundary and its polygons cannot say it is inside: the
            // verdict comes first, and the other face decides the image.
            let along = [(oa, fa), (ob, fb)].into_iter().find_map(|(side, f)| {
                self.along_block(side, f, fa, fb, curve, range, start, end)
                    .map(|(edge, block)| (side, edge, block))
            });
            if let Some((side, edge, block)) = along {
                // Along an edge of the other face as well, wherever that
                // edge is paved, it is both faces' boundary: each edge's
                // own piece there, neither a section edge nor an image of
                // one on the other face.
                let other = if side == oa { (ob, fb) } else { (oa, fa) };
                if !self.along_boundary(other.0, other.1, fa, fb, curve, range) {
                    self.along.push(Along {
                        pair: pi,
                        operand: side,
                        edge,
                        block,
                    });
                }
                continue;
            }
            let Some(uv) = Self::inside_both(fa, fb, curve.point(range.midpoint())) else {
                continue;
            };
            let section = self.section_edge(
                [oa, ob],
                (fa, fb),
                curve,
                range,
                uv,
                curve_index,
                start,
                end,
            )?;
            // Every vertex ≥ its edges: the ends carry at least the
            // section edge's tolerance.
            for k in [start, end] {
                let v = &mut self.vertices[k];
                v.floor = v.floor.max(section.tolerance);
            }
            edges.push(self.sections.len());
            self.sections.push(section);
        }
        self.curves.push(SectionCurve {
            pair: pi,
            index: ci,
            curve: curve.clone(),
            paves,
            edges,
        });
        Ok(())
    }

    /// Whether a face of operand `side` other than `f` that uses `e` lies
    /// on a surface `Coincident` with `other`'s, a face of the other
    /// operand, by the face pair's own verdict.
    pub(super) fn beside_on(&self, side: usize, e: EdgeId, f: FaceId, other: FaceId) -> bool {
        let Some(other_side) = self.operand_of_face(other) else {
            return false;
        };
        self.faces[side]
            .iter()
            .filter(|g| g.id != f && g.edges().contains(&e))
            .any(|g| {
                let (a, b) = if side < other_side {
                    (g.id, other)
                } else {
                    (other, g.id)
                };
                self.pairs.iter().any(|p| {
                    p.a == a && p.b == b && p.intersection == SurfaceIntersection::Coincident
                })
            })
    }

    /// Whether `e`, an edge of face `f` of operand `side`, runs along
    /// `curve`, a section of `f`'s surface and `other`'s, known by the
    /// surfaces rather than by comparing the curves: where a face beside
    /// `f` in `e`'s own operand lies on a surface `Coincident` with
    /// `other`'s, the edge lies on both of the pair's surfaces to its own
    /// tolerance (E4) and so on their section, and along `curve` exactly
    /// when its midpoint lies on it within `tol`. Two fits of one traced
    /// section over different regions are two splines no closed form
    /// compares, and each is held to the exact branch (ADR-0022), so the
    /// midpoint decides. `None` when no such face is beside `f`, and the
    /// curves have to be asked.
    pub(super) fn known_by_surfaces(
        &self,
        side: usize,
        e: &EdgeInfo<'m>,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        curve: &Curve,
        tol: Tolerance,
    ) -> Option<bool> {
        if !self.beside_on(side, e.id, f.id, other.id) {
            return None;
        }
        let mid = e.curve.point(e.range.midpoint());
        Some(curve.project(mid).is_ok_and(|on| on.distance <= tol.linear))
    }

    /// Whether `ea` of `fa` and `eb` of `fb`, two faces on one surface,
    /// are one section by their surfaces: a face beside `fa` using `ea`
    /// and one beside `fb` using `eb` lie on one surface too, so both
    /// edges lie on the section of the two surfaces, and they are the
    /// same curve when `ea`'s midpoint lies on `eb` within `tol`.
    pub(super) fn same_section(
        &self,
        ea: &EdgeInfo<'m>,
        fa: &FaceInfo<'m>,
        eb: &EdgeInfo<'m>,
        fb: &FaceInfo<'m>,
        tol: Tolerance,
    ) -> bool {
        let (Some(oa), Some(ob)) = (self.operand_of_face(fa.id), self.operand_of_face(fb.id))
        else {
            return false;
        };
        let beside = self.faces[ob]
            .iter()
            .filter(|g| g.id != fb.id && g.edges().contains(&eb.id))
            .any(|g| self.beside_on(oa, ea.id, fa.id, g.id));
        beside
            && eb
                .curve
                .project(ea.curve.point(ea.range.midpoint()))
                .is_ok_and(|on| on.distance <= tol.linear)
    }

    /// The piece of an operand edge of `f`, the pair's face of operand
    /// `side`, that the block `range` of `curve` of the pair `fa`, `fb`,
    /// from section vertex `start` to `end`, lies along: a piece between
    /// the same two vertices, in either order,
    /// that every point the model checks the block at lies within the
    /// tolerance of, inside the piece's own range. Two curves of one
    /// surface crossing at a shallow angle twice stay within the
    /// tolerance of each other over the whole stretch between — a small
    /// circle through a sphere's pole 2e-4 of a radian off the seam, back
    /// across it 1.8e-4 on — and the block between the two crossings is
    /// then no curve of its own but the edge's piece: built as a section
    /// edge it would bound a sliver of zero area on the edge's face. The
    /// verdict is the block's alone, beside the whole-curve one
    /// (`curves_coincide`) the curve's blocks are first held to; `None`
    /// when no piece is along it.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn along_block(
        &self,
        side: usize,
        f: &FaceInfo<'m>,
        fa: &FaceInfo<'m>,
        fb: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        start: usize,
        end: usize,
    ) -> Option<(EdgeId, Block)> {
        let (start, end) = (End::Section(start), End::Section(end));
        let points: Vec<Point3> = samples(range, self.precision.check_samples)
            .into_iter()
            .map(|t| curve.point(t))
            .collect();
        for &eid in f.edges() {
            let Some(e) = self.edge_info(side, eid) else {
                continue;
            };
            let within = e.tolerance.max(fa.tolerance.max(fb.tolerance));
            for block in self.blocks_of(e) {
                let ends = (self.canonical(block.start), self.canonical(block.end));
                if ends != (start, end) && ends != (end, start) {
                    continue;
                }
                let lies_along = points.iter().all(|&p| {
                    e.curve.project(p).is_ok_and(|on| {
                        on.distance <= within
                            && e.in_range(on.t)
                                .is_some_and(|t| Self::within_block(e, block.range, t))
                    })
                });
                if lies_along {
                    return Some((eid, block));
                }
            }
        }
        None
    }

    /// Whether every point the model checks the block `range` of `curve`
    /// at lies within the tolerance of an edge of `f`, the face of operand
    /// `side` of the pair `fa`, `fb`, inside that edge's range: the block
    /// runs along `f`'s boundary, whatever vertices pave the edges there.
    /// Two edges crossing at a grazing angle are hit at points scattered
    /// along them by the rounding over the angle, further apart than a
    /// tolerance, so the pieces of the two edges need not end on the same
    /// vertices.
    pub(super) fn along_boundary(
        &self,
        side: usize,
        f: &FaceInfo<'m>,
        fa: &FaceInfo<'m>,
        fb: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
    ) -> bool {
        let edges: Vec<&EdgeInfo<'m>> = f
            .edges()
            .iter()
            .filter_map(|&eid| self.edge_info(side, eid))
            .collect();
        samples(range, self.precision.check_samples)
            .into_iter()
            .all(|t| {
                let p = curve.point(t);
                edges.iter().any(|e| {
                    let within = e.tolerance.max(fa.tolerance.max(fb.tolerance));
                    e.curve
                        .project(p)
                        .is_ok_and(|on| on.distance <= within && e.in_range(on.t).is_some())
                })
            })
    }

    /// `t`, placed in `e`'s range, inside `range` or beyond either end by
    /// no more than the edge's tolerance converted to the parameter there.
    pub(super) fn within_block(e: &EdgeInfo<'m>, range: Interval, t: f64) -> bool {
        let speed = e.curve.eval(t).d1.norm();
        let slack = if speed > 0.0 {
            e.tolerance / speed
        } else {
            0.0
        };
        t >= range.lo() - slack && t <= range.hi() + slack
    }

    /// A kept block as a section edge: the pcurve on each face, placed
    /// in the face's translate of the domain and ending on its vertices'
    /// own (u, v) there ([`Self::ended`]), and the tolerance raised to
    /// the pcurves' residual.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn section_edge(
        &self,
        operands: [usize; 2],
        (fa, fb): (&FaceInfo<'m>, &FaceInfo<'m>),
        curve: &Curve,
        range: Interval,
        uv_mid: [Point2; 2],
        curve_index: usize,
        start: usize,
        end: usize,
    ) -> Result<SectionEdge, OpError> {
        let base = fa.tolerance.max(fb.tolerance);
        let m = self.m;
        let ends = [start, end].map(|k| {
            let v = &self.vertices[k];
            (v.point(m), v.tolerance(m))
        });
        let (pa, ra) = self.pcurve_of(fa, fb, curve, range, uv_mid[0], base, &ends)?;
        let (pb, rb) = self.pcurve_of(fb, fa, curve, range, uv_mid[1], base, &ends)?;
        let (pa, ra) = self.ended(
            operands[0],
            fa,
            fb,
            curve,
            range,
            base,
            [start, end],
            (pa, ra),
        )?;
        let (pb, rb) = self.ended(
            operands[1],
            fb,
            fa,
            curve,
            range,
            base,
            [start, end],
            (pb, rb),
        )?;
        let tolerance = base.max(ra).max(rb);
        if tolerance > self.precision.max_tolerance {
            return Err(OpError::Tolerance {
                entity: fa.shape(),
                wanted: tolerance,
            });
        }
        Ok(SectionEdge {
            curve: curve_index,
            range,
            start,
            end,
            tolerance,
            pcurves: [pa, pb],
        })
    }

    /// The pcurve of a curve block on one face, placed, with the largest
    /// deviation of its image from the curve at the model's check
    /// parameters. `base` is the tolerance the fit is asked for; `ends`
    /// the balls of the vertices the block ends on, as [`Self::place`]
    /// reads them.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn pcurve_of(
        &self,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        uv_mid: Point2,
        base: f64,
        ends: &[(Point3, f64)],
    ) -> Result<(Curve2, f64), OpError> {
        let tol = Tolerance::new(base, self.precision.angular_tolerance);
        let pc = self
            .metered(|mt| pcurve_on(curve, range, f.surface, tol, mt))
            .map_err(|e| geometry(e, other.shape(), f.shape()))?;
        let pc = self.place(f, other, pc, range, uv_mid, base, ends)?;
        let residual = self.residual(f, curve, range, &pc);
        Ok((pc, residual))
    }

    /// The largest deviation of `pc`'s image on `f` from `curve` at the
    /// model's check parameters over `range`, with the rounding of the
    /// coordinates it is made of at each (the checker's own allowance
    /// for E4): a tolerance raised to it holds when the surfaces and the
    /// curve are evaluated again from the stored model, which rounds
    /// every point.
    pub(super) fn residual(
        &self,
        f: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        pc: &Curve2,
    ) -> f64 {
        samples(range, self.precision.check_samples)
            .into_iter()
            .map(|t| {
                let q = pc.point(t);
                let (on_surface, on_curve) = (f.surface.point(q.x, q.y), curve.point(t));
                (on_surface - on_curve).norm()
                    + RELATIVE_ROUNDING * on_surface.coords.norm().max(on_curve.coords.norm())
            })
            .fold(0.0, f64::max)
    }

    /// A section edge's placed pcurve on face `f` of operand `side`, with
    /// its residual, ended on the (u, v) of the section vertices it ends
    /// on ([`Self::vertex_uv`]) wherever it lies further from it than
    /// half the band L2 holds a junction to — so that any two ends there
    /// meet within the band — and its residual measured again with the
    /// move. A section vertex merged from points further apart than a
    /// face's tolerance ([`components`]) has the curve's ends and the
    /// edge it paves apart by that much, and exact curves cannot meet on
    /// it: the section edge's pcurve moves, and its tolerance with it
    /// (`docs/DATA-MODEL.md` §Tolerances), never the operand's.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn ended(
        &self,
        side: usize,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        curve: &Curve,
        range: Interval,
        base: f64,
        vertices: [usize; 2],
        (pc, residual): (Curve2, f64),
    ) -> Result<(Curve2, f64), OpError> {
        let [start, end] = [(range.lo(), vertices[0]), (range.hi(), vertices[1])].map(|(t, k)| {
            let at = pc.point(t);
            let target = self
                .vertex_uv(side, f, k, at)
                .or_else(|| self.singular_arrival(f, k, at))?;
            let bound = bands(f.surface, target, self.precision.parametric_tolerance);
            let d = target - at;
            (d.x.abs() > 0.5 * bound[0] || d.y.abs() > 0.5 * bound[1]).then_some(target)
        });
        if start.is_none() && end.is_none() {
            return Ok((pc, residual));
        }
        let pc = self
            .metered(|mt| pcurve_ending_on(&pc, range, [start, end], f.surface, base, mt))
            .map_err(|e| geometry(e, other.shape(), f.shape()))?;
        let residual = self.residual(f, curve, range, &pc);
        Ok((pc, residual))
    }

    /// Where a pcurve arriving at `at` on section vertex `k`, a singular
    /// vertex of `f`, ends: `at` with each periodic parameter held to the
    /// face's (u, v) box. The vertex is a whole line of (u, v), any `u` on
    /// it one point, and the `u` a curve arrives with is its tangent there;
    /// one that arrives past the box's edge crossed the seam inside the
    /// vertex's ball — a small circle through a pole a hair off the seam's
    /// meridian, crossing it again nearer the pole than the vertex's
    /// tolerance — and ends on the seam's own corner of the box, where the
    /// degenerate edge meets it. `None` for any other vertex, and for an
    /// arrival inside the box.
    pub(super) fn singular_arrival(
        &self,
        f: &FaceInfo<'m>,
        k: usize,
        at: Point2,
    ) -> Option<Point2> {
        let v = &self.vertices[k];
        if !f.singular.iter().any(|s| v.existing.contains(&s.vertex)) {
            return None;
        }
        let mut held = at;
        for (d, period) in f.surface.period().iter().enumerate() {
            if period.is_some() {
                held[d] = at[d].clamp(f.uv_lo[d], f.uv_hi[d]);
            }
        }
        (held != at).then_some(held)
    }

    /// Where section vertex `k` is on face `f` of operand `side`, in the
    /// translate nearest `near`: an operand edge of the face paved there
    /// at its pave, or ending on an operand vertex it holds at that end —
    /// the point every piece of that edge will meet the vertex at — and
    /// otherwise the (u, v) of the vertex's point. Where the face holds
    /// several, the nearest; ties to the first in loop order. `None` at a
    /// singular vertex of the face, a whole line of (u, v) whose pcurves
    /// meet along the degenerate edge (ADR-0021), and where the point
    /// does not project.
    pub(super) fn vertex_uv(
        &self,
        side: usize,
        f: &FaceInfo<'m>,
        k: usize,
        near: Point2,
    ) -> Option<Point2> {
        let v = &self.vertices[k];
        if f.singular.iter().any(|s| v.existing.contains(&s.vertex)) {
            return None;
        }
        let mut candidates: Vec<Point2> = Vec::new();
        for &(eid, pcurve) in &f.uses {
            if let Some(paves) = self.paves.get(&eid) {
                candidates.extend(
                    paves
                        .iter()
                        .filter(|p| p.vertex == k)
                        .map(|p| pcurve.point(p.t)),
                );
            }
            if let Some(e) = self.edge_info(side, eid) {
                for (end, t) in [(e.ends[0].0, e.range.lo()), (e.ends[1].0, e.range.hi())] {
                    if v.existing.contains(&end) {
                        candidates.push(pcurve.point(t));
                    }
                }
            }
        }
        if candidates.is_empty() {
            candidates.push(f.surface.project(v.point(self.m)).ok()?.uv);
        }
        let periods = f.surface.period();
        candidates
            .into_iter()
            .map(|mut c| {
                for (d, period) in periods.iter().enumerate() {
                    if let Some(p) = period {
                        c[d] += ((near[d] - c[d]) / p).round() * p;
                    }
                }
                c
            })
            .min_by(|x, y| (x - near).norm().total_cmp(&(y - near).norm()))
    }

    /// The pcurve translated by whole periods so its point at the block's
    /// midpoint is the face's own (u, v) there, and held to the face's
    /// (u, v) box in every periodic direction within `tolerance` converted
    /// at each point checked, in that direction: a block that still
    /// leaves it crosses a seam inside itself, which the seam's own hit
    /// should have paved (ADR-0004). Converted where the pcurve is and not
    /// once at the midpoint, because on a sphere or a cone a tolerance is
    /// no one step in `u`: a loop round a pole ends on the seam at a
    /// latitude where the fit's rounding in `u` is many times what the
    /// same length allows at the loop's far side. Inside the ball of a
    /// vertex it ends on (`ends`, point and tolerance) the block is that
    /// vertex, and held to its tolerance: a section vertex merged from
    /// points a tolerance apart ([`components`]) has the seam crossing inside
    /// its ball, and the stretch of the block up to it is no crossing the
    /// block makes.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn place(
        &self,
        f: &FaceInfo<'m>,
        other: &FaceInfo<'m>,
        pc: Curve2,
        range: Interval,
        uv_mid: Point2,
        tolerance: f64,
        ends: &[(Point3, f64)],
    ) -> Result<Curve2, OpError> {
        let periods = f.surface.period();
        let at = pc.point(range.midpoint());
        let mut by = Vec2::zeros();
        for (d, period) in periods.iter().enumerate() {
            if let Some(p) = period {
                by[d] = ((uv_mid[d] - at[d]) / p).round() * p;
            }
        }
        let pc = pc.translated(by);
        for (d, period) in periods.iter().enumerate() {
            if period.is_none() {
                continue;
            }
            for t in samples(range, self.precision.check_samples) {
                let at = pc.point(t);
                let outside = |reach: f64| {
                    let near = bands(f.surface, at, reach)[d];
                    at[d] < f.uv_lo[d] - near || at[d] > f.uv_hi[d] + near
                };
                if !outside(tolerance) {
                    continue;
                }
                let point = f.surface.point(at.x, at.y);
                // Within the ball of a singular point of the face, a step
                // of `u` is no length at all ([`Self::singular_arrival`]).
                let singular = |p: Point3, reach: f64| {
                    f.singular.iter().any(|s| (s.point - p).norm() <= reach)
                };
                let within_an_end = ends.iter().any(|&(p, reach)| {
                    (point - p).norm() <= reach && (singular(p, reach) || !outside(reach))
                });
                if !within_an_end {
                    return Err(OpError::Internal(Fault::Seam {
                        face: f.id,
                        other: other.id,
                    }));
                }
            }
        }
        Ok(pc)
    }
}
