//! The loop, face, shell and body rows of `Level::Fast` — L1–L4, F1–F2,
//! S1–S4 and B3 of `docs/DATA-MODEL.md` §Invariants. The Euler–Poincaré
//! line every report carries is `arris_topo::euler::EulerLine`.
//!
//! Every row here reads the face's own loops and the shell's own face
//! uses, never the arena's adjacency indices (M2 is the only row that
//! speaks for those), and the (u, v) rows go through
//! `arris_geom::region2`: a loop is discretised at the minimum segment
//! counts, and finer only where a sign or a containment question is
//! within the coarse polygon's deviation (L4).

use core::f64::consts::{PI, TAU};
use std::collections::{BTreeMap, BTreeSet};

use arris_geom::region2::{Polygon2, discretise};
use arris_geom::{Curve2, Surface, torus_v_reaches_axis};
use arris_math::{DisjointSets, Interval, Point2, Vec2, real, shift_into_range, wrap_signed};
use arris_topo::entity::{BodyKind, Coedge, Face, Loop};
use arris_topo::{EdgeId, FaceId, Model, Orientation, ShellId, VertexId};

use crate::check::{Checker, coedges};
use crate::domain::{bands, bounded_pieces, chord};
use crate::violation::{
    EdgeUseFault, FaceFault, LoopBreak, NestingFault, ToleranceBound, Violation, WireFault,
};

/// One translate of the regular band, with a pole allowed at either end.
/// A span that crosses a root cannot fit in the same translate (ADR-0092).
struct TorusBand {
    root: f64,
    margin: f64,
    angular: f64,
}

impl TorusBand {
    fn holds(&self, span: Interval, poles: [bool; 2]) -> bool {
        let centre = TAU * (span.midpoint() / TAU).round();
        let lo = span.lo() - centre;
        let hi = span.hi() - centre;
        (if poles[0] {
            (lo + self.root).abs() <= self.angular
        } else {
            lo > -self.margin
        }) && (if poles[1] {
            (hi - self.root).abs() <= self.angular
        } else {
            hi < self.margin
        })
    }

    /// The unwrapped `v` and vertex of a degenerate edge at a torus pole.
    fn pole(&self, model: &Model, surface: &Surface, coedge: Coedge) -> Option<(f64, VertexId)> {
        let edge = model.edge(coedge.edge()).ok()?;
        if !edge.is_degenerate() || edge.start() != edge.end() || !edge.range().is_bounded() {
            return None;
        }
        let Curve2::Line { origin, direction } = model.curve2(coedge.pcurve()).ok()? else {
            return None;
        };
        let v = wrap_signed(origin.y, TAU);
        let root = self.root.copysign(v);
        if direction.y != 0.0 || (v - root).abs() > self.angular {
            return None;
        }
        let vertex = model.vertex(edge.start()).ok()?;
        ((vertex.point() - surface.point(origin.x, root)).norm() <= vertex.tolerance())
            .then_some((origin.y, edge.start()))
    }

    /// Only a terminal approach may enter the margin. An interior turn
    /// of `v` in it is a separate approach, even if an endpoint has the
    /// same `v`. Check conic extrema exactly and sample a spline per span.
    fn turns_clear(&self, pcurve: &Curve2, range: Interval, samples: usize) -> bool {
        let clear = |v| wrap_signed(v, TAU).abs() < self.margin;
        let conic = |a, b| {
            let top = real::atan2(b, a);
            let interior = Interval::new(range.lo() + self.angular, range.hi() - self.angular).ok();
            [top, top + PI].into_iter().all(|t| {
                interior
                    .and_then(|span| shift_into_range(span, t, Some(TAU)))
                    .is_none_or(|t| clear(pcurve.point(t).y))
            })
        };
        match pcurve {
            Curve2::Line { .. } => true,
            Curve2::Circle { frame, radius } => conic(radius * frame.x().y, radius * frame.y().y),
            Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => conic(major_radius * frame.x().y, minor_radius * frame.y().y),
            Curve2::Nurbs(spline) => {
                let mut lo = range.lo();
                let mut values = vec![pcurve.point(lo).y];
                for hi in spline.breaks_within(range).into_iter().chain([range.hi()]) {
                    if let Ok(span) = Interval::new(lo, hi) {
                        values.extend(
                            span.samples(samples.max(3))
                                .into_iter()
                                .skip(1)
                                .map(|t| pcurve.point(t).y),
                        );
                    }
                    lo = hi;
                }
                values.windows(3).all(|run| {
                    let (a, v, b) = (run[0], run[1], run[2]);
                    clear(v) || (a < v && v < b) || (a > v && v > b)
                })
            }
        }
    }
}

impl<'m> Checker<'m> {
    /// L1–L4 and F1–F2, face by face in closure order.
    pub(crate) fn face_rows(&mut self) {
        let model = self.model;
        for face_id in self.closure.faces.clone() {
            let Ok(face) = model.face(face_id) else {
                continue;
            };
            self.l1_closed(face_id, face);
            self.l2_junctions(face_id, face);
            self.l3_reuse(face_id, face);
            self.l4_nesting(face_id, face);
            self.f1_malformed(face_id, face);
            self.f2_tolerance(face_id, face);
        }
    }

    /// The effective start and end vertex of a coedge in its face: the
    /// edge's, swapped when the use is `Reversed`. `None` when the edge
    /// does not resolve (M1's).
    fn coedge_ends(&self, edge: EdgeId, orientation: Orientation) -> Option<(VertexId, VertexId)> {
        let e = self.model.edge(edge).ok()?;
        Some(if orientation == Orientation::Forward {
            (e.start(), e.end())
        } else {
            (e.end(), e.start())
        })
    }

    /// The (u, v) point where a coedge's walk starts, and where it ends.
    /// `None` when the edge, its range or the pcurve does not resolve.
    fn coedge_uv(
        &self,
        edge: EdgeId,
        orientation: Orientation,
        pcurve: &Curve2,
    ) -> Option<(Point2, Point2)> {
        let range = self.model.edge(edge).ok()?.range();
        if !(range.lo().is_finite() && range.hi().is_finite()) {
            return None;
        }
        let (first, last) = if orientation == Orientation::Forward {
            (range.lo(), range.hi())
        } else {
            (range.hi(), range.lo())
        };
        Some((pcurve.point(first), pcurve.point(last)))
    }

    /// L1: a loop has a coedge and closes. One line per loop: the first
    /// junction that breaks, since every later one is its consequence.
    fn l1_closed(&mut self, face_id: FaceId, face: &'m Face) {
        let mut faults = Vec::new();
        for (loop_index, l) in face.loops().iter().enumerate() {
            let cs = l.coedges();
            if cs.is_empty() {
                faults.push((loop_index, LoopBreak::Empty));
                continue;
            }
            for i in 0..cs.len() {
                let here = self.coedge_ends(cs[i].edge(), cs[i].orientation());
                let next = cs[(i + 1) % cs.len()];
                let there = self.coedge_ends(next.edge(), next.orientation());
                if let (Some((_, end)), Some((start, _))) = (here, there) {
                    if end != start {
                        faults.push((loop_index, LoopBreak::Between { coedge: i }));
                        break;
                    }
                }
            }
        }
        for (loop_index, fault) in faults {
            self.push(Violation::LoopOpen {
                face: face_id,
                loop_index,
                fault,
            });
        }
    }

    /// L2: the pcurves meet in (u, v) at every junction, or jump by
    /// exactly one period of the surface — across a seam edge, and where
    /// a closed edge's pcurve wraps the periodic parameter once. Every
    /// junction that is too far apart is reported; each is its own
    /// measurement and its own repair.
    fn l2_junctions(&mut self, face_id: FaceId, face: &'m Face) {
        let model = self.model;
        let Ok(surface) = model.surface(face.surface()) else {
            return;
        };
        let mut gaps = Vec::new();
        for (loop_index, l) in face.loops().iter().enumerate() {
            let cs = l.coedges();
            for i in 0..cs.len() {
                let next = cs[(i + 1) % cs.len()];
                let (Ok(p_here), Ok(p_next)) =
                    (model.curve2(cs[i].pcurve()), model.curve2(next.pcurve()))
                else {
                    continue;
                };
                let (Some((_, end)), Some((start, _))) = (
                    self.coedge_uv(cs[i].edge(), cs[i].orientation(), p_here),
                    self.coedge_uv(next.edge(), next.orientation(), p_next),
                ) else {
                    continue;
                };
                let d = start - end;
                if !self.uv_jump_ok(surface, end, d) {
                    gaps.push((loop_index, i, d.norm()));
                }
            }
        }
        for (loop_index, coedge, gap) in gaps {
            self.push(Violation::PcurveGap {
                face: face_id,
                loop_index,
                coedge,
                gap,
            });
        }
    }

    /// `true` when the (u, v) step `d` at `uv` is no jump at all within
    /// the model's parametric tolerance, or is exactly one period of a
    /// periodic parameter with no step in the other.
    fn uv_jump_ok(&self, surface: &Surface, uv: Point2, d: Vec2) -> bool {
        let bound = bands(surface, uv, self.precision.parametric_tolerance);
        if d[0].abs() <= bound[0] && d[1].abs() <= bound[1] {
            return true;
        }
        let periods = surface.period();
        (0..2).any(|dir| {
            periods[dir].is_some_and(|period| {
                (d[dir].abs() - period).abs() <= bound[dir] && d[1 - dir].abs() <= bound[1 - dir]
            })
        })
    }

    /// L3: an edge is used once by a face, or twice by one of its loops
    /// as a seam. A pair in one loop of a *periodic* surface is E7's to
    /// judge; every other repeat — two loops of the face, more than two
    /// uses, a pair on a surface with no period — is this row's.
    fn l3_reuse(&mut self, face_id: FaceId, face: &'m Face) {
        let periodic = self
            .model
            .surface(face.surface())
            .is_ok_and(|s| s.period() != [None, None]);
        let mut loops_of: BTreeMap<EdgeId, BTreeSet<usize>> = BTreeMap::new();
        let mut count: BTreeMap<EdgeId, usize> = BTreeMap::new();
        for (loop_index, _, coedge) in coedges(face) {
            loops_of
                .entry(coedge.edge())
                .or_default()
                .insert(loop_index);
            *count.entry(coedge.edge()).or_default() += 1;
        }
        let reused: Vec<EdgeId> = count
            .iter()
            .filter(|&(edge, &n)| {
                let one_loop = loops_of[edge].len() == 1;
                n > 1 && !(n == 2 && one_loop && periodic)
            })
            .map(|(&edge, _)| edge)
            .collect();
        for edge in reused {
            self.push(Violation::EdgeReusedInFace {
                face: face_id,
                edge,
            });
        }
    }

    /// The polygon of a loop in (u, v) at the minimum segment counts —
    /// all a sign, a winding number and a containment question need.
    /// `None` when the loop is empty, a range is not bounded (E1's) or a
    /// reference does not resolve (M1's).
    fn loop_polygon(&self, l: &'m Loop, chord: f64) -> Option<Polygon2> {
        let pieces = bounded_pieces(self.model, l).ok()??;
        Some(discretise(&pieces, chord))
    }

    /// L4: every loop turns, and the loops nest — one positively wound
    /// outer loop per connected component of the domain, every negatively
    /// wound hole inside one of them.
    ///
    /// Decided first on the loops at the minimum segment counts, and
    /// again at a finer chord ([`L4_COARSENING`], then the fine one) only
    /// where an answer is within the polygons' deviation: the fine
    /// polygons of a real part's
    /// face run to millions of segments, and every operation's debug
    /// check and every `tessellate_faces` call pays for them.
    fn l4_nesting(&mut self, face_id: FaceId, face: &'m Face) {
        let model = self.model;
        let Ok(surface) = model.surface(face.surface()) else {
            return;
        };
        // Fine enough that a hole lying inside an outer loop in the model
        // lies inside its polygon: the loops are discretised as a face's
        // domain is, within the parametric tolerance. A coarser outer
        // polygon, a circle's inscribed octagon, leaves out a hole that
        // runs close to the rim, so each coarser pass only answers with a
        // margin and hands that case on to the next.
        let chord = chord(model, face, surface, self.precision.parametric_tolerance);
        let faults = L4_COARSENING
            .iter()
            .find_map(|&k| self.nesting_faults(face, surface, chord * k, true))
            .or_else(|| self.nesting_faults(face, surface, chord, false))
            .unwrap_or_default();
        for fault in faults {
            self.push(Violation::LoopNesting {
                face: face_id,
                fault,
            });
        }
    }

    /// L4's faults for `face`'s loops discretised at `chord`. With
    /// `certain`, `None` when an answer is not sure to be the loops' own:
    /// a loop's area, of which a polygon's shoelace area is within its
    /// deviation times its perimeter, not clear of zero, or a hole with
    /// no point (on its curve) farther than twice the deviation from the
    /// polygon it is asked to be inside — so the answer is also the one
    /// a finer polygon gives. A loop of zero area is never decided
    /// coarse.
    fn nesting_faults(
        &self,
        face: &'m Face,
        surface: &Surface,
        chord: f64,
        certain: bool,
    ) -> Option<Vec<NestingFault>> {
        let mut faults: Vec<NestingFault> = Vec::new();
        let mut areas: Vec<(usize, Polygon2, f64)> = Vec::new();
        for (loop_index, l) in face.loops().iter().enumerate() {
            let Some(polygon) = self.loop_polygon(l, chord) else {
                continue;
            };
            let area = polygon.signed_area();
            // A loop whose mean width — its area over half its perimeter
            // — is below the model's parametric tolerance in (u, v)
            // encloses nothing.
            let perimeter: f64 = polygon.segments().map(|(a, b)| (b - a).norm()).sum();
            let width = match polygon.points().first() {
                Some(&p) => {
                    let bound = bands(surface, p, self.precision.parametric_tolerance);
                    bound[0].min(bound[1])
                }
                None => return Some(Vec::new()),
            };
            if certain {
                // The fine polygon's perimeter is at most the loop's
                // length; twice the coarse one stands for it.
                let margin = polygon.chord_deviation() * perimeter;
                if !(margin.is_finite() && area.abs() - margin > width * perimeter) {
                    return None;
                }
            }
            if !area.is_finite() || area.abs() <= width * perimeter / 2.0 {
                faults.push(NestingFault::ZeroArea { loop_index });
                continue;
            }
            areas.push((loop_index, polygon, area));
        }
        let holds = |inner: &Polygon2, outer: &Polygon2| -> Option<bool> {
            if certain && !clear_of(inner, outer) {
                return None;
            }
            Some(inside(inner, outer))
        };
        let outer: Vec<&(usize, Polygon2, f64)> =
            areas.iter().filter(|(_, _, a)| *a > 0.0).collect();
        let holes: Vec<&(usize, Polygon2, f64)> =
            areas.iter().filter(|(_, _, a)| *a < 0.0).collect();
        if outer.is_empty() && !areas.is_empty() {
            faults.push(NestingFault::NoOuter);
        }
        // Two outer loops belong to one component exactly when one holds
        // the other; disjoint ones are two components, which the row
        // allows.
        let mut nested: BTreeSet<usize> = BTreeSet::new();
        for (i, a) in outer.iter().enumerate() {
            for b in outer.iter().skip(i + 1) {
                if holds(&a.1, &b.1)? || holds(&b.1, &a.1)? {
                    nested.insert(a.0);
                    nested.insert(b.0);
                }
            }
        }
        if !nested.is_empty() {
            faults.push(NestingFault::MultipleOuter {
                loops: nested.into_iter().collect(),
            });
        }
        // A hole is only outside something: with no outer loop at all,
        // `NoOuter` has said it already.
        for hole in outer.is_empty().then(Vec::new).unwrap_or(holes) {
            let mut held = false;
            for o in &outer {
                if holds(&hole.1, &o.1)? {
                    held = true;
                    break;
                }
            }
            if !held {
                faults.push(NestingFault::HoleOutside { loop_index: hole.0 });
            }
        }
        Some(faults)
    }

    /// F1: the face has a loop, every pcurve lies in the surface's bounded
    /// domain, and a torus face stays on the regular band up to its poles.
    /// One line per face, at the first coedge that leaves the domain.
    fn f1_malformed(&mut self, face_id: FaceId, face: &'m Face) {
        if face.loops().is_empty() || face.loops().iter().all(|l| l.coedges().is_empty()) {
            self.push(Violation::FaceMalformed {
                face: face_id,
                fault: FaceFault::NoLoops,
            });
            return;
        }
        let model = self.model;
        let Ok(surface) = model.surface(face.surface()) else {
            return;
        };
        let fault = if let Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } = surface
        {
            self.f1_torus_band(face, surface, *major_radius, *minor_radius)
        } else {
            self.f1_surface_domain(face, surface)
        };
        if let Some(fault) = fault {
            self.push(Violation::FaceMalformed {
                face: face_id,
                fault,
            });
        }
    }

    fn f1_surface_domain(&self, face: &Face, surface: &Surface) -> Option<FaceFault> {
        let domain = surface.domain();
        let periods = surface.period();
        for (loop_index, coedge_index, coedge) in coedges(face) {
            let (Ok(edge), Ok(pcurve)) = (
                self.model.edge(coedge.edge()),
                self.model.curve2(coedge.pcurve()),
            ) else {
                continue;
            };
            let range = edge.range();
            if !range.is_bounded() {
                continue;
            }
            for t in range.samples(self.precision.check_samples) {
                let uv = pcurve.point(t);
                let bound = bands(surface, uv, self.precision.parametric_tolerance);
                for dir in 0..2 {
                    if periods[dir].is_some() {
                        continue;
                    }
                    let lo = domain[dir].lo() - bound[dir];
                    let hi = domain[dir].hi() + bound[dir];
                    if !(lo <= uv[dir] && uv[dir] <= hi) {
                        return Some(FaceFault::PcurveOutsideDomain {
                            loop_index,
                            coedge: coedge_index,
                        });
                    }
                }
            }
        }
        None
    }

    fn f1_torus_band(
        &self,
        face: &Face,
        surface: &Surface,
        major: f64,
        minor: f64,
    ) -> Option<FaceFault> {
        let off_band = |span| torus_v_reaches_axis(major, minor, face.tolerance(), span);
        let band = (major <= minor && face.tolerance() < major + minor).then(|| TorusBand {
            root: real::acos(-major / minor),
            margin: real::acos(((face.tolerance() - major) / minor).min(1.0)),
            angular: self.precision.angular_tolerance,
        });
        let mut v_span: Option<Interval> = None;
        let mut pole_span: Option<Interval> = None;
        for (loop_index, coedge_index, coedge) in coedges(face) {
            let (Ok(edge), Ok(pcurve)) = (
                self.model.edge(coedge.edge()),
                self.model.curve2(coedge.pcurve()),
            ) else {
                continue;
            };
            let range = edge.range();
            let Some(extent) = pcurve.v_extent(range) else {
                continue;
            };
            v_span = Some(v_span.map_or(extent, |span| span.hull(&extent)));
            if !off_band(extent) {
                continue;
            }
            if let Some(band) = &band {
                if band.pole(self.model, surface, coedge).is_some() {
                    pole_span = Some(pole_span.map_or(extent, |span| span.hull(&extent)));
                    continue;
                }
                if !edge.is_degenerate() && extent.length() > 0.0 {
                    let cs = face.loops()[loop_index].coedges();
                    let before = cs[(coedge_index + cs.len() - 1) % cs.len()];
                    let after = cs[(coedge_index + 1) % cs.len()];
                    let mut ends = [
                        (edge.start(), pcurve.point(range.lo()).y),
                        (edge.end(), pcurve.point(range.hi()).y),
                    ];
                    if coedge.orientation() == Orientation::Reversed {
                        ends.reverse();
                    }
                    let mut poles = [before, after].map(|adj| band.pole(self.model, surface, adj));
                    for (pole, (vertex, v)) in poles.iter_mut().zip(ends) {
                        *pole = pole.filter(|&(pole_v, at)| {
                            at == vertex && wrap_signed(v - pole_v, TAU).abs() <= band.angular
                        });
                    }
                    if ends[0].1 > ends[1].1 {
                        ends.reverse();
                        poles.reverse();
                    }
                    let at_pole = [
                        poles[0].is_some() && (extent.lo() - ends[0].1).abs() <= band.angular,
                        poles[1].is_some() && (extent.hi() - ends[1].1).abs() <= band.angular,
                    ];
                    let ends_clear = ends.iter().zip(poles).all(|((_, v), pole)| {
                        pole.is_some() || wrap_signed(*v, TAU).abs() < band.margin
                    });
                    if ends_clear
                        && band.holds(extent, at_pole)
                        && band.turns_clear(pcurve, range, self.precision.check_samples)
                    {
                        continue;
                    }
                }
            }
            return Some(FaceFault::OffTorusBand {
                loop_index,
                coedge: coedge_index,
            });
        }
        // The loops bound the face in their given (u, v) plane: even loops
        // that each fit on a band can enclose the axis between them.
        let span = v_span?;
        if !off_band(span) {
            return None;
        }
        let held_poles = band.zip(pole_span).is_some_and(|(band, poles)| {
            band.holds(
                span,
                [
                    (poles.lo() - span.lo()).abs() <= band.angular,
                    (poles.hi() - span.hi()).abs() <= band.angular,
                ],
            )
        });
        (!held_poles).then_some(FaceFault::OffTorusBand {
            loop_index: 0,
            coedge: 0,
        })
    }

    /// F2: the face's tolerance is at or above the model's floor and at
    /// or below every incident edge's. The edge's side of the second
    /// bound is E5's line on the edge.
    fn f2_tolerance(&mut self, face_id: FaceId, face: &'m Face) {
        let tolerance = face.tolerance();
        if !tolerance.is_finite() {
            return;
        }
        if tolerance < self.precision.min_tolerance {
            self.push(Violation::FaceTolerance {
                face: face_id,
                tolerance,
                bound: ToleranceBound::BelowMinimum,
            });
        }
        let mut seen: BTreeSet<EdgeId> = BTreeSet::new();
        let mut over = Vec::new();
        for (_, _, coedge) in coedges(face) {
            if !seen.insert(coedge.edge()) {
                continue;
            }
            let Ok(edge) = self.model.edge(coedge.edge()) else {
                continue;
            };
            if tolerance > edge.tolerance() {
                over.push((coedge.edge(), edge.tolerance()));
            }
        }
        for (edge, edge_tolerance) in over {
            self.push(Violation::FaceTolerance {
                face: face_id,
                tolerance,
                bound: ToleranceBound::Neighbour {
                    entity: edge.into(),
                    tolerance: edge_tolerance,
                },
            });
        }
    }

    /// V4 and S1–S4, shell by shell in closure order.
    pub(crate) fn shell_rows(&mut self, distinct_vertices: bool) {
        let kind = self.model.body(self.body.id).map(|b| b.kind()).ok();
        for shell_id in self.closure.shells.clone() {
            let uses = self.shell_edge_uses(shell_id);
            if distinct_vertices {
                self.violations.extend(
                    self.v4_coincident(&uses)
                        .into_iter()
                        .map(|[a, b]| Violation::CoincidentVertices { a, b }),
                );
            }
            self.s1_faces(shell_id);
            self.s2_edge_uses(shell_id, kind, uses);
            self.s3_connected(shell_id);
        }
    }

    /// S1: no face is used twice by one shell.
    fn s1_faces(&mut self, shell_id: ShellId) {
        let Ok(shell) = self.model.shell(shell_id) else {
            return;
        };
        let mut seen: BTreeSet<FaceId> = BTreeSet::new();
        let twice: BTreeSet<FaceId> = shell
            .faces()
            .iter()
            .filter(|f| !seen.insert(f.id))
            .map(|f| f.id)
            .collect();
        for face in twice {
            self.push(Violation::FaceUsedTwice {
                shell: shell_id,
                face,
            });
        }
    }

    /// The effective orientation of every coedge use of every edge of a
    /// shell: the coedge's composed with the face use's, in the shell's
    /// own stored order.
    pub(crate) fn shell_edge_uses(&self, shell_id: ShellId) -> BTreeMap<EdgeId, Vec<Orientation>> {
        let mut uses: BTreeMap<EdgeId, Vec<Orientation>> = BTreeMap::new();
        let Ok(shell) = self.model.shell(shell_id) else {
            return uses;
        };
        for face_use in shell.faces() {
            let Ok(face) = self.model.face(face_use.id) else {
                continue;
            };
            for (_, _, coedge) in coedges(face) {
                uses.entry(coedge.edge())
                    .or_default()
                    .push(face_use.orientation.compose(coedge.orientation()));
            }
        }
        uses
    }

    /// S2 (the coedge count and the orientations that pair up, by body
    /// kind) and S4 (a solid's shell is closed).
    fn s2_edge_uses(
        &mut self,
        shell_id: ShellId,
        kind: Option<BodyKind>,
        uses: BTreeMap<EdgeId, Vec<Orientation>>,
    ) {
        // A wire body has no shell at all (B3); a shell it does carry is
        // not judged by a rule written for faces.
        let Some(kind) = kind.filter(|k| *k != BodyKind::Wire) else {
            return;
        };
        let mut found = Vec::new();
        for (edge, orientations) in uses {
            // A degenerate edge is a singular point of the surface, not a
            // boundary between two faces: a sphere's pole is used once by
            // the one face that closes on it, and counting it would call
            // every sphere open (E6, `docs/DATA-MODEL.md` §Invariants).
            if self.model.edge(edge).is_ok_and(|e| e.is_degenerate()) {
                continue;
            }
            let n = orientations.len();
            let forward = orientations
                .iter()
                .filter(|o| **o == Orientation::Forward)
                .count();
            let count_ok = match kind {
                BodyKind::Solid => n == 2,
                BodyKind::Sheet => n == 1 || n == 2,
                BodyKind::Wire | BodyKind::General => true,
            };
            if !count_ok {
                found.push(Violation::EdgeUses {
                    shell: shell_id,
                    edge,
                    fault: EdgeUseFault::Count { coedges: n },
                });
            }
            // The uses pair up: as many forward as reversed, but for an
            // odd count, where exactly one use is left over.
            if forward.abs_diff(n - forward) > n % 2 {
                found.push(Violation::EdgeUses {
                    shell: shell_id,
                    edge,
                    fault: EdgeUseFault::Orientation,
                });
            }
            // S4: the same edge leaves a solid's shell open.
            if kind == BodyKind::Solid && n == 1 {
                found.push(Violation::ShellOpen {
                    shell: shell_id,
                    edge,
                });
            }
        }
        for v in found {
            self.push(v);
        }
    }

    /// S3: the shell's faces form one component through their shared
    /// edges.
    fn s3_connected(&mut self, shell_id: ShellId) {
        let Ok(shell) = self.model.shell(shell_id) else {
            return;
        };
        let faces: Vec<FaceId> = shell.faces().iter().map(|f| f.id).collect();
        if faces.len() < 2 {
            return;
        }
        // Union–find over the face uses, joined by every shared edge.
        let mut sets = DisjointSets::new(faces.len());
        let mut owner: BTreeMap<EdgeId, usize> = BTreeMap::new();
        for (i, &face_id) in faces.iter().enumerate() {
            let Ok(face) = self.model.face(face_id) else {
                continue;
            };
            for (_, _, coedge) in coedges(face) {
                match owner.get(&coedge.edge()) {
                    Some(&j) => {
                        sets.union(i, j);
                    }
                    None => {
                        owner.insert(coedge.edge(), i);
                    }
                }
            }
        }
        let components: BTreeSet<usize> = (0..faces.len()).map(|i| sets.find(i)).collect();
        if components.len() > 1 {
            self.push(Violation::ShellDisconnected {
                shell: shell_id,
                components: components.len(),
            });
        }
    }

    /// B3: a wire body has no shell, and its free edges form chains.
    pub(crate) fn body_rows(&mut self) {
        let Ok(body) = self.model.body(self.body.id) else {
            return;
        };
        if body.kind() != BodyKind::Wire {
            return;
        }
        let mut faults: Vec<WireFault> = body
            .shells()
            .iter()
            .map(|s| WireFault::HasShell { shell: s.id })
            .collect();
        let mut at: BTreeMap<VertexId, usize> = BTreeMap::new();
        for e in body.free_edges() {
            let Ok(edge) = self.model.edge(e.id) else {
                continue;
            };
            for v in [edge.start(), edge.end()] {
                *at.entry(v).or_default() += 1;
            }
        }
        for (vertex, edges) in at {
            if edges > 2 {
                faults.push(WireFault::VertexOverused { vertex, edges });
            }
        }
        for fault in faults {
            self.push(Violation::WireMalformed {
                body: self.body.id,
                fault,
            });
        }
    }
}

/// The chords L4 tries before the fine one, as multiples of it: the
/// minimum segment counts, then a million and a thousand times the fine
/// chord. Not tolerances: a pass answers only beyond its polygons'
/// deviation, so these set the cost and never the answer. A polygon's
/// segments go as the inverse square root of its chord, each pass about
/// thirty times the one before, and a face is read only as finely as its
/// answer needs.
const L4_COARSENING: [f64; 3] = [f64::INFINITY, 1e6, 1e3];

/// `true` when `inner`'s ring lies inside `outer`'s: its first vertex
/// has a non-zero winding number about `outer`. Two rings that cross are
/// L5's, not this test's.
fn inside(inner: &Polygon2, outer: &Polygon2) -> bool {
    clear_point(inner, outer)
        .or_else(|| inner.points().first().copied())
        .is_some_and(|p| outer.winding_number(p) != 0)
}

/// `true` when a vertex of `inner`, a point of its loop, is farther than
/// twice `outer`'s deviation from every segment of `outer`: then
/// `outer`'s loop and every polygon of it at least as fine wind about it
/// as `outer` does.
fn clear_of(inner: &Polygon2, outer: &Polygon2) -> bool {
    clear_point(inner, outer).is_some()
}

/// The first vertex of `inner` farther than twice `outer`'s deviation
/// from every segment of `outer`, the point a hole is asked to be inside
/// at. Not always the first: a hole may meet its outer loop at a vertex
/// of both — Open CASCADE's crescent, an outer bound and a hole through
/// the point where its arcs touch — and starts there.
fn clear_point(inner: &Polygon2, outer: &Polygon2) -> Option<Point2> {
    let reach = 2.0 * outer.chord_deviation();
    if !reach.is_finite() {
        return None;
    }
    inner.points().iter().copied().find(|&p| {
        outer.segments().all(|(a, b)| {
            let ab = b - a;
            let t = ((p - a).dot(&ab) / ab.norm_squared()).clamp(0.0, 1.0);
            (p - (a + ab * t)).norm() > reach
        })
    })
}
