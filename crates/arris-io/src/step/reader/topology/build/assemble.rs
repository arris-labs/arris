//! The assembly phases: splits carried to every use, loops placed and
//! measured, the assembly stated for the builder.

use std::collections::BTreeMap;

use arris_geom::{Curve2, Surface};
use arris_math::Interval;
use arris_topo::Orientation;
use arris_topo::builder::{Assembly, EdgeKey, EdgeSpec, FaceSpec, UseSpec, VertexKey, VertexSpec};
use arris_topo::entity::EdgeGeometry;

use super::{Build, Walked};
use crate::step::reader::Refusal;
use crate::step::reader::topology::{Entity, Gaps, hold_in_domain, place_loops};

/// The vertices' and edges' tolerances the gaps left.
pub(super) struct Tolerances {
    pub(super) vertices: Vec<f64>,
    pub(super) edges: Vec<f64>,
}

impl Build<'_, '_> {
    /// Every other use of a split edge is split with it, and the edge
    /// ends at the new vertex.
    pub(super) fn propagate_splits(&mut self, walked: &mut [Vec<Walked>]) -> Result<(), Refusal> {
        for split in &self.splits {
            for face in walked.iter_mut().flatten() {
                for uses in face.loops.iter_mut() {
                    split.apply(uses);
                }
            }
            let edge = &mut self.edges[split.edge];
            edge.range =
                Interval::new(edge.range.lo(), split.t).map_err(|e| Refusal::Degenerate {
                    entity: self.edge_ids[split.edge],
                    what: e.to_string(),
                })?;
            edge.end = split.vertex;
        }
        Ok(())
    }

    /// The faces placed in their surfaces' periods and held in their
    /// domains, the gaps they leave measured, and the assembly the
    /// builder is given.
    pub(super) fn assemble(
        &mut self,
        walked: Vec<Vec<Walked>>,
    ) -> Result<(Assembly, Tolerances), Refusal> {
        let precision = self.precision;
        let mut gaps = Gaps::new(self.points.len());
        let mut shells = Vec::with_capacity(walked.len());
        // Each face's surface, and each edge's uses on them, for where
        // the faces an edge bounds meet.
        let mut face_surfaces: Vec<Surface> = Vec::new();
        let mut uses_of: BTreeMap<usize, Vec<(usize, Curve2, Interval)>> = BTreeMap::new();
        for faces in walked {
            let mut specs = Vec::with_capacity(faces.len());
            for Walked {
                surface,
                surface_id,
                flipped,
                mut loops,
            } in faces
            {
                place_loops(
                    &mut loops,
                    surface.period(),
                    surface.domain(),
                    precision.check_samples,
                );
                for u in loops.iter_mut().flatten() {
                    hold_in_domain(&mut u.pcurve, u.range, &surface, precision);
                }
                for uses in &loops {
                    gaps.measure_uses(
                        &surface,
                        uses,
                        &self.points,
                        &self.edges,
                        &self.rebuilt,
                        self.model,
                        precision,
                    );
                }
                for u in loops.iter().flatten() {
                    uses_of.entry(self.edge_key(u.edge)).or_default().push((
                        face_surfaces.len(),
                        u.pcurve.clone(),
                        u.range,
                    ));
                }
                face_surfaces.push(surface);
                let loops = loops
                    .into_iter()
                    .map(|uses| {
                        uses.into_iter()
                            .map(|u| UseSpec {
                                edge: EdgeKey::New(self.edge_key(u.edge)),
                                orientation: u.orientation,
                                pcurve: self.model.add_curve2(u.pcurve),
                            })
                            .collect()
                    })
                    .collect();
                specs.push(FaceSpec::New {
                    surface: surface_id,
                    orientation: if flipped {
                        Orientation::Reversed
                    } else {
                        Orientation::Forward
                    },
                    loops,
                    tolerance: self.tolerance,
                });
            }
            shells.push(specs);
        }

        gaps.measure_edges(&self.points, &self.edges, &self.rebuilt, self.model);
        gaps.measure_sections(
            &face_surfaces,
            &uses_of,
            &self.edges,
            &self.rebuilt,
            self.model,
            precision,
        );
        let (vertices, edges) = gaps
            .tolerances(&self.edges, &self.rebuilt, self.tolerance, self.cap)
            .map_err(|(entity, gap)| Refusal::Gap {
                entity: match entity {
                    Entity::Vertex(v) => self.vertex_ids[v],
                    Entity::Edge(k) if k < self.edges.len() => self.edge_ids[k],
                    Entity::Edge(k) => self.rebuilt_faces[k - self.edges.len()],
                },
                gap,
                cap: self.cap,
            })?;
        let assembly = Assembly {
            vertices: (self.points.iter().zip(&vertices))
                .map(|(&point, &tolerance)| VertexSpec::New { point, tolerance })
                .collect(),
            edges: self
                .edges
                .iter()
                .map(|e| {
                    (
                        EdgeGeometry::Curve {
                            curve: e.curve,
                            range: e.range,
                        },
                        e.start,
                        e.end,
                    )
                })
                .chain(self.rebuilt.iter().map(|r| (r.geometry, r.start, r.end)))
                .zip(&edges)
                .map(|((geometry, start, end), &tolerance)| EdgeSpec::New {
                    geometry,
                    start: VertexKey::New(start),
                    end: VertexKey::New(end),
                    tolerance,
                })
                .collect(),
            shells,
        };
        Ok((assembly, Tolerances { vertices, edges }))
    }
}
