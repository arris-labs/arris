//! E1–E7.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::Curve;
use arris_math::{Interval, Point3, RELATIVE_ROUNDING};
use arris_topo::entity::{BodyKind, Coedge, Edge, EdgeGeometry};
use arris_topo::{CoedgeRef, EdgeId, FaceId, Orientation, VertexId};

use super::Checker;
use crate::domain::bands;
use crate::violation::{DegenerateFault, EndMismatch, SeamFault, ToleranceBound, Violation};

/// The interior points of an edge's range that E2 looks at for a stray from
/// its start: a curve that returns to it leaves its tolerance on the way.
const E2_SAMPLES: usize = 8;

impl Checker<'_> {
    /// E1–E7.
    pub(crate) fn edge_rows(&mut self) {
        let precision = self.precision;
        let body_kind = self.model.body(self.body.id).map(|b| b.kind()).ok();
        let free_edges: BTreeSet<EdgeId> = self
            .model
            .body(self.body.id)
            .map(|b| b.free_edges().iter().map(|e| e.id).collect())
            .unwrap_or_default();
        let mut pcurve_off: BTreeMap<(EdgeId, FaceId), (f64, f64)> = BTreeMap::new();
        let mut seams: Vec<Violation> = Vec::new();
        for edge_id in self.closure.edges.clone() {
            let Ok(edge) = self.model.edge(edge_id).copied() else {
                continue;
            };
            let uses = self.uses_in_body(edge_id);
            let range = edge.range();
            let range_finite = range.lo().is_finite() && range.hi().is_finite();
            match edge.geometry() {
                EdgeGeometry::Curve { curve, range } => {
                    let Ok(curve) = self.model.curve(curve) else {
                        continue;
                    };
                    if range_finite {
                        self.e1_range(edge_id, curve, range);
                        self.e2_ends(edge_id, &edge, curve, range);
                    }
                }
                EdgeGeometry::Degenerate { .. } => {
                    self.e6_degenerate(edge_id, &edge, &uses);
                }
            }
            // E3
            if uses.is_empty() {
                let free_of_wire = free_edges.contains(&edge_id)
                    && matches!(body_kind, Some(BodyKind::Wire | BodyKind::General));
                if !free_of_wire {
                    self.push(Violation::EdgeUnused { edge: edge_id });
                }
            }
            // E4 and E5 over the uses.
            let mut faces_seen: BTreeSet<FaceId> = BTreeSet::new();
            for (u, coedge) in &uses {
                let Ok(face) = self.model.face(u.face) else {
                    continue;
                };
                if faces_seen.insert(u.face) && face.tolerance() > edge.tolerance() {
                    self.push(Violation::EdgeTolerance {
                        edge: edge_id,
                        tolerance: edge.tolerance(),
                        bound: ToleranceBound::Neighbour {
                            entity: u.face.into(),
                            tolerance: face.tolerance(),
                        },
                    });
                }
                if let (Some((curve_id, range)), Ok(surface), Ok(pcurve)) = (
                    edge.curve(),
                    self.model.surface(face.surface()),
                    self.model.curve2(coedge.pcurve()),
                ) {
                    if let (Ok(curve), true) = (self.model.curve(curve_id), range_finite) {
                        let mut worst: Option<(f64, f64)> = None;
                        for t in range.samples(precision.check_samples) {
                            let uv = pcurve.point(t);
                            let (on_surface, on_curve) =
                                (surface.point(uv.x, uv.y), curve.point(t));
                            let gap = (on_surface - on_curve).norm();
                            // A tolerance raised to a measured gap, or a body
                            // moved rigidly, leaves the gap at the tolerance to
                            // the rounding of the coordinates it is made of.
                            let rounding = RELATIVE_ROUNDING
                                * on_surface.coords.norm().max(on_curve.coords.norm());
                            if gap > edge.tolerance() + rounding
                                && worst.is_none_or(|(_, g)| gap > g)
                            {
                                worst = Some((t, gap));
                            }
                        }
                        if let Some((t, gap)) = worst {
                            let entry = pcurve_off.entry((edge_id, u.face)).or_insert((t, gap));
                            if gap > entry.1 {
                                *entry = (t, gap);
                            }
                        }
                    }
                }
            }
            let mut vertices_seen: BTreeSet<VertexId> = BTreeSet::new();
            for v in [edge.start(), edge.end()] {
                let Ok(vertex) = self.model.vertex(v) else {
                    continue;
                };
                if vertices_seen.insert(v) && vertex.tolerance() < edge.tolerance() {
                    self.push(Violation::EdgeTolerance {
                        edge: edge_id,
                        tolerance: edge.tolerance(),
                        bound: ToleranceBound::Neighbour {
                            entity: v.into(),
                            tolerance: vertex.tolerance(),
                        },
                    });
                }
            }
            // E7: an edge used twice by one loop.
            let mut by_loop: BTreeMap<(FaceId, usize), Vec<Coedge>> = BTreeMap::new();
            for (u, coedge) in &uses {
                by_loop
                    .entry((u.face, u.loop_index))
                    .or_default()
                    .push(*coedge);
            }
            for ((face_id, _), pair) in by_loop {
                let [a, b] = pair.as_slice() else {
                    continue;
                };
                if let Some(fault) = self.seam_fault(face_id, &edge, a, b) {
                    seams.push(Violation::Seam {
                        edge: edge_id,
                        face: face_id,
                        fault,
                    });
                }
            }
        }
        for ((edge, face), (parameter, distance)) in pcurve_off {
            self.push(Violation::PcurveOffCurve {
                edge,
                face,
                parameter,
                distance,
            });
        }
        for v in seams {
            self.push(v);
        }
    }

    /// E1: a non-empty range inside the curve's domain, crossing a period
    /// at most once.
    fn e1_range(&mut self, edge_id: EdgeId, curve: &Curve, range: Interval) {
        let ok = range.lo() < range.hi()
            && match curve.period() {
                Some(period) => range.length() <= period,
                None => {
                    let domain = curve.domain();
                    domain.lo() <= range.lo() && range.hi() <= domain.hi()
                }
            };
        if !ok {
            self.push(Violation::EdgeRange { edge: edge_id });
        }
    }

    /// E2: `start == end` exactly when the curve returns to its start over
    /// the range, to the edge's tolerance. An edge whose whole range lies
    /// within that tolerance of its start does not return to it: it is a
    /// short edge between two vertices, such as the remnant a blend leaves
    /// of an edge where its faces turn tangent (ADR-0074).
    fn e2_ends(&mut self, edge_id: EdgeId, edge: &Edge, curve: &Curve, range: Interval) {
        let start = curve.point(range.lo());
        let gap = (curve.point(range.hi()) - start).norm();
        let closed = gap <= edge.tolerance();
        let returns = || {
            (1..E2_SAMPLES).any(|i| {
                let t = range.lerp(i as f64 / E2_SAMPLES as f64);
                (curve.point(t) - start).norm() > edge.tolerance()
            })
        };
        let fault = match (closed, edge.start() == edge.end()) {
            (true, false) if returns() => Some(EndMismatch::ClosedWithTwoVertices {
                start: edge.start(),
                end: edge.end(),
            }),
            (false, true) => Some(EndMismatch::OpenWithOneVertex {
                vertex: edge.start(),
                gap,
            }),
            (true, false) | (true, true) | (false, false) => None,
        };
        if let Some(fault) = fault {
            self.push(Violation::EdgeEnds {
                edge: edge_id,
                fault,
            });
        }
    }

    /// E6: one vertex, and a surface singular along each pcurve — its image
    /// over the range within the vertex's tolerance of one point.
    fn e6_degenerate(&mut self, edge_id: EdgeId, edge: &Edge, uses: &[(CoedgeRef, Coedge)]) {
        if edge.start() != edge.end() {
            self.push(Violation::DegenerateEdge {
                edge: edge_id,
                fault: DegenerateFault::TwoVertices,
            });
        }
        let Ok(vertex) = self.model.vertex(edge.start()) else {
            return;
        };
        let range = edge.range();
        if !(range.lo().is_finite() && range.hi().is_finite()) {
            return;
        }
        let mut faces_seen: BTreeSet<FaceId> = BTreeSet::new();
        for (u, coedge) in uses {
            if !faces_seen.insert(u.face) {
                continue;
            }
            let (Ok(face), Ok(pcurve)) =
                (self.model.face(u.face), self.model.curve2(coedge.pcurve()))
            else {
                continue;
            };
            let Ok(surface) = self.model.surface(face.surface()) else {
                continue;
            };
            let images: Vec<Point3> = range
                .samples(self.precision.check_samples)
                .into_iter()
                .map(|t| {
                    let uv = pcurve.point(t);
                    surface.point(uv.x, uv.y)
                })
                .collect();
            let mut extent: f64 = 0.0;
            for (i, p) in images.iter().enumerate() {
                for q in &images[i + 1..] {
                    extent = extent.max((p - q).norm());
                }
            }
            if extent > vertex.tolerance() {
                self.push(Violation::DegenerateEdge {
                    edge: edge_id,
                    fault: DegenerateFault::NotSingular {
                        face: u.face,
                        extent,
                    },
                });
            }
        }
    }

    /// E7: the two uses of an edge in one loop run opposite ways and their
    /// pcurves differ by one period in a periodic direction of the surface,
    /// within `parametric_tolerance` scaled to the surface's speed. `None`
    /// when the surface has no periodic direction (L3's case) or the
    /// geometry does not resolve.
    fn seam_fault(
        &self,
        face_id: FaceId,
        edge: &Edge,
        a: &Coedge,
        b: &Coedge,
    ) -> Option<SeamFault> {
        if a.orientation() == b.orientation() {
            return Some(SeamFault::SameOrientation);
        }
        let face = self.model.face(face_id).ok()?;
        let surface = self.model.surface(face.surface()).ok()?;
        let periods = surface.period();
        if periods == [None, None] {
            return None;
        }
        let (first, second) = if a.orientation() == Orientation::Forward {
            (a, b)
        } else {
            (b, a)
        };
        let p1 = self.model.curve2(first.pcurve()).ok()?;
        let p2 = self.model.curve2(second.pcurve()).ok()?;
        let range = edge.range();
        if !(range.lo().is_finite() && range.hi().is_finite()) {
            return None;
        }
        let mut fault = None;
        for t in [range.lo(), range.midpoint(), range.hi()] {
            let (u1, u2) = (p1.point(t), p2.point(t));
            let d = u2 - u1;
            let bound = bands(surface, u1, self.precision.parametric_tolerance);
            let matches = |dir: usize| {
                let Some(period) = periods[dir] else {
                    return false;
                };
                let other = 1 - dir;
                (d[dir].abs() - period).abs() <= bound[dir] && d[other].abs() <= bound[other]
            };
            if !(matches(0) || matches(1)) {
                // Report against the surface's first periodic direction.
                let dir = if periods[0].is_some() { 0 } else { 1 };
                fault = Some(SeamFault::PeriodMismatch {
                    offset: d[dir],
                    period: periods[dir].unwrap_or(0.0),
                });
                break;
            }
        }
        fault
    }
}
