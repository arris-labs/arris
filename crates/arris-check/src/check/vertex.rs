//! V1–V3.

use std::collections::BTreeMap;

use arris_topo::{EdgeId, FaceId, VertexId};

use super::{Checker, coedges};
use crate::violation::{ToleranceBound, Violation};

impl Checker<'_> {
    /// V1–V3.
    pub(crate) fn vertex_rows(&mut self) {
        let precision = self.precision;
        let mut off_edge: Vec<(VertexId, EdgeId, f64)> = Vec::new();
        for id in self.closure.vertices.clone() {
            let Ok(v) = self.model.vertex(id) else {
                continue;
            };
            let tolerance = v.tolerance();
            // V1: the floor only. `max_tolerance` bounds what an
            // operation grows, not what a body holds (ADR-0060).
            if tolerance.is_finite() && tolerance < precision.min_tolerance {
                self.push(Violation::VertexTolerance {
                    vertex: id,
                    tolerance,
                    bound: ToleranceBound::BelowMinimum,
                });
            }
            // V2: each incident edge of the body, at the end that names v.
            for edge_id in self.vertex_edges.get(&id).cloned().unwrap_or_default() {
                let Ok(edge) = self.model.edge(edge_id) else {
                    continue;
                };
                let Some((curve_id, range)) = edge.curve() else {
                    continue;
                };
                let Ok(curve) = self.model.curve(curve_id) else {
                    continue;
                };
                let mut worst: f64 = 0.0;
                for (end, t) in [(edge.start(), range.lo()), (edge.end(), range.hi())] {
                    if end == id && t.is_finite() {
                        worst = worst.max((curve.point(t) - v.point()).norm());
                    }
                }
                if worst > tolerance {
                    off_edge.push((id, edge_id, worst));
                }
            }
        }
        for (vertex, edge, distance) in off_edge {
            self.push(Violation::VertexOffEdge {
                vertex,
                edge,
                distance,
            });
        }
        // V3: the surface at each pcurve's ends against the vertex there.
        let mut off_face: BTreeMap<(VertexId, FaceId), f64> = BTreeMap::new();
        for &face_id in &self.closure.faces {
            let Ok(face) = self.model.face(face_id) else {
                continue;
            };
            let Ok(surface) = self.model.surface(face.surface()) else {
                continue;
            };
            for (_, _, coedge) in coedges(face) {
                let Ok(edge) = self.model.edge(coedge.edge()) else {
                    continue;
                };
                let Ok(pcurve) = self.model.curve2(coedge.pcurve()) else {
                    continue;
                };
                let range = edge.range();
                for (vertex_id, t) in [(edge.start(), range.lo()), (edge.end(), range.hi())] {
                    let Ok(vertex) = self.model.vertex(vertex_id) else {
                        continue;
                    };
                    if !t.is_finite() {
                        continue;
                    }
                    let image = surface.point(pcurve.point(t).x, pcurve.point(t).y);
                    let distance = (image - vertex.point()).norm();
                    if distance > vertex.tolerance() {
                        let worst = off_face.entry((vertex_id, face_id)).or_insert(0.0);
                        *worst = worst.max(distance);
                    }
                }
            }
        }
        for ((vertex, face), distance) in off_face {
            self.push(Violation::VertexOffFace {
                vertex,
                face,
                distance,
            });
        }
    }
}
