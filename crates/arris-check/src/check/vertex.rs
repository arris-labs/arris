//! V1–V4.

use std::collections::{BTreeMap, BTreeSet};

use arris_topo::{EdgeId, FaceId, Orientation, VertexId};

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

    /// V4: distinct vertices of one shell must be farther apart than
    /// both of their tolerances, except at the ends of a short edge.
    pub(crate) fn v4_coincident(
        &self,
        edges: &BTreeMap<EdgeId, Vec<Orientation>>,
    ) -> Vec<[VertexId; 2]> {
        let ids: BTreeSet<_> = edges
            .keys()
            .filter_map(|&id| self.model.edge(id).ok())
            .flat_map(|edge| [edge.start(), edge.end()])
            .collect();
        let mut vertices: Vec<_> = ids
            .into_iter()
            .filter_map(|id| self.model.vertex(id).ok().map(|vertex| (id, vertex)))
            // M3 handles invalid coordinates and tolerances.
            .filter(|(_, vertex)| {
                vertex.tolerance().is_finite()
                    && vertex.point().coords.iter().all(|x| x.is_finite())
            })
            .collect();
        vertices
            .sort_unstable_by(|(a, v), (b, w)| v.point().x.total_cmp(&w.point().x).then(a.cmp(b)));
        let mut pairs = Vec::new();
        for &(id, vertex) in &vertices {
            let point = vertex.point();
            let tolerance = vertex.tolerance();
            let first =
                vertices.partition_point(|(_, other)| point.x - other.point().x > tolerance);
            for &(other_id, other) in vertices
                .iter()
                .skip(first)
                .take_while(|(_, other)| other.point().x - point.x <= tolerance)
            {
                // Only the larger tolerance tests the pair. Break ties by id.
                if (tolerance, id) <= (other.tolerance(), other_id)
                    || (point - other.point()).norm() > tolerance
                {
                    continue;
                }
                // An edge between different points can carry valid material
                // even when its ends are closer than their tolerances (E2).
                let joined = point != other.point()
                    && self.vertex_edges.get(&id).into_iter().flatten().any(|e| {
                        edges.contains_key(e)
                            && self.model.edge(*e).is_ok_and(|edge| {
                                edge.curve().is_some()
                                    && (edge.start() == other_id || edge.end() == other_id)
                            })
                    });
                if !joined {
                    pairs.push([id.min(other_id), id.max(other_id)]);
                }
            }
        }
        pairs
    }
}
