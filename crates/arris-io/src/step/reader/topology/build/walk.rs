//! The walk phase: every face's bounds as loops of uses on its surface.

use std::collections::BTreeMap;

use arris_geom::Curve2;
use arris_math::Interval;
use arris_topo::Orientation;

use super::{Build, Walked};
use crate::step::reader::Refusal;
use crate::step::reader::topology::{
    Edge, EdgeRef, Fix, Jump, Split, Step, Use, Walk, band, fitted, junctions, seam_crosses,
    seam_to, stopped_in, turn,
};

impl Build<'_, '_> {
    /// Every shell's faces walked, in the file's order.
    pub(super) fn walk_shells(&mut self) -> Result<Vec<Vec<Walked>>, Refusal> {
        let file = self.file;
        let precision = self.precision;
        let tol = self.tol;
        let (cap, read_cap) = (self.cap, self.read_cap);
        let mut walked = Vec::with_capacity(file.shells.len());
        for (si, shell) in file.shells.iter().enumerate() {
            let mut faces = Vec::with_capacity(shell.faces.len());
            for (fi, face) in shell.faces.iter().enumerate() {
                self.tick()?;
                let (surface, surface_id) = self.surface(face)?;
                // Outward is the file face's normal turned by the shell or
                // an ORIENTED_FACE, the file face's normal is the file
                // surface's turned where `same_sense` is false, and the
                // variant's normal is the file surface's turned by
                // `reversed`.
                let flipped = face.surface.reversed ^ !face.same_sense ^ face.turned;
                let mut pcurves: BTreeMap<usize, Curve2> = BTreeMap::new();
                let singular = surface.singularities();
                let at = junctions(&surface, &singular, &self.points, cap, precision, flipped);
                // A VERTEX_LOOP is joined to the face's one other bound by
                // a seam, which Arris's face needs and the file left out.
                let vertex_bounds: Vec<(u64, usize)> = (face.bounds.iter())
                    .filter_map(|b| match b.walk {
                        Walk::Vertex(v) => Some((b.id, v)),
                        Walk::Edges(_) => None,
                    })
                    .collect();
                let joined = match vertex_bounds[..] {
                    [] => None,
                    [one] if face.bounds.len() == 2 => Some(one),
                    [(bound, _), ..] => {
                        return Err(Refusal::Unsupported {
                            entity: bound,
                            name: "a VERTEX_LOOP beside other than one EDGE_LOOP".into(),
                        });
                    }
                };
                let mut loops = Vec::with_capacity(face.bounds.len());
                for bound in &face.bounds {
                    let Walk::Edges(ref walk) = bound.walk else {
                        continue;
                    };
                    let mut walk = walk.clone();
                    if face.turned != self.turn_back.contains(&(si, fi)) {
                        turn(&mut walk);
                    }
                    // Each file edge's pieces, in the walk's direction.
                    let walk: Vec<Step> = (walk.iter())
                        .flat_map(|s| {
                            let mut run = self.pieces[s.edge].clone();
                            if s.forward != self.edges[s.edge].along {
                                run.reverse();
                            }
                            run.into_iter().map(|edge| Step {
                                edge,
                                forward: s.forward,
                            })
                        })
                        .collect();
                    let mut uses = Vec::with_capacity(walk.len());
                    for s in walk {
                        self.meter.tick().map_err(|_| stopped_in(self.solid))?;
                        let edge = &self.edges[s.edge];
                        let pcurve = match pcurves.get(&s.edge) {
                            Some(p) => p.clone(),
                            None => {
                                let id = self.edge_ids[s.edge];
                                // Under the read cap first, so a curve
                                // that needs no healing is fitted as it
                                // would be without it.
                                let p = match fitted(
                                    &edge.geometry,
                                    edge.range,
                                    &surface,
                                    precision,
                                    read_cap,
                                    self.meter,
                                ) {
                                    Err(fault) if cap > read_cap && fault.heals() => fitted(
                                        &edge.geometry,
                                        edge.range,
                                        &surface,
                                        precision,
                                        cap,
                                        self.meter,
                                    ),
                                    first => first,
                                }
                                .map_err(|fault| fault.refusal(id, face.id, cap))?;
                                pcurves.insert(s.edge, p.clone());
                                p
                            }
                        };
                        let (orientation, vertices) = if s.forward == edge.along {
                            (Orientation::Forward, [edge.start, edge.end])
                        } else {
                            (Orientation::Reversed, [edge.end, edge.start])
                        };
                        uses.push(Use {
                            edge: EdgeRef::File(s.edge),
                            orientation,
                            pcurve,
                            range: edge.range,
                            vertices,
                        });
                    }
                    let seam = match joined {
                        Some((vertex_bound, apex)) => {
                            let unsupported = |name: &str| Refusal::Unsupported {
                                entity: vertex_bound,
                                name: name.into(),
                            };
                            let row = at.singular_at(apex).ok_or_else(|| {
                                unsupported("a VERTEX_LOOP off its surface's singular points")
                            })?;
                            let seam = seam_to(
                                self.model,
                                &surface,
                                row,
                                apex,
                                &uses,
                                &self.points,
                                tol,
                                self.meter,
                            )
                            .map_err(unsupported)?;
                            Some(seam.join(&mut uses, &mut self.rebuilt))
                        }
                        None => None,
                    };
                    let mut uses = at
                        .walk(uses, &mut self.rebuilt)
                        .map_err(|jump| match jump {
                            Jump::Open(vertex) => Refusal::OpenLoop {
                                face: face.id,
                                bound: bound.id,
                                vertex: self.vertex_ids[vertex],
                            },
                            Jump::Gap(vertex, gap) => Refusal::Gap {
                                entity: self.vertex_ids[vertex],
                                gap,
                                cap,
                            },
                        })?;
                    at.meet(&mut uses, self.meter)
                        .map_err(|(edge, e)| Refusal::Pcurve {
                            edge: match edge {
                                EdgeRef::File(i) => self.edge_ids[i],
                                EdgeRef::Rebuilt(_) => face.id,
                            },
                            face: face.id,
                            what: e.to_string(),
                        })?;
                    if let (Some(i), Some((vertex_bound, _))) = (seam, joined) {
                        if seam_crosses(
                            &uses,
                            i,
                            &surface,
                            precision.parametric_tolerance,
                            precision.check_samples,
                        ) {
                            return Err(Refusal::Unsupported {
                                entity: vertex_bound,
                                name: "a VERTEX_LOOP whose seam to its face's bound crosses it"
                                    .into(),
                            });
                        }
                    }
                    loops.push(uses);
                }
                if joined.is_none() {
                    let unsupported = |name: &str| Refusal::Unsupported {
                        entity: face.id,
                        name: name.into(),
                    };
                    let samples = precision.check_samples;
                    let cut = band(
                        &at,
                        &self.edges,
                        self.model,
                        &mut loops,
                        &mut self.rebuilt,
                        tol,
                        samples,
                        self.meter,
                    )
                    .map_err(unsupported)?;
                    if let Some(Fix::Apex { face_loop, row }) = cut {
                        let apex = self.points.len();
                        self.points.push(row.point);
                        self.vertex_ids.push(face.id);
                        let at =
                            junctions(&surface, &singular, &self.points, cap, precision, flipped);
                        let mut uses = std::mem::take(&mut loops[face_loop]);
                        let seam = seam_to(
                            self.model,
                            &surface,
                            row,
                            apex,
                            &uses,
                            &self.points,
                            tol,
                            self.meter,
                        )
                        .map_err(unsupported)?;
                        let i = seam.join(&mut uses, &mut self.rebuilt);
                        let mut uses = at.walk(uses, &mut self.rebuilt).map_err(|_| {
                            unsupported("a face of one wrapping loop whose seam to its apex jumps")
                        })?;
                        at.meet(&mut uses, self.meter).map_err(|_| {
                            unsupported("a face of one wrapping loop whose seam to its apex does not meet it")
                        })?;
                        if seam_crosses(&uses, i, &surface, at.parametric, samples) {
                            return Err(unsupported(
                                "a face of one wrapping loop whose seam to its apex crosses it",
                            ));
                        }
                        loops[face_loop] = uses;
                    } else if let Some(Fix::Cut(cut)) = cut {
                        // The seam's far end is a vertex on the other
                        // loop's edge, which is split there.
                        if cut.edge >= file.edges.len()
                            || self.splits.iter().any(|s| s.edge == cut.edge)
                        {
                            return Err(unsupported(
                                "a band whose seam would split an edge a second time",
                            ));
                        }
                        let old = &self.edges[cut.edge];
                        let split = Split {
                            edge: cut.edge,
                            t: cut.t,
                            vertex: self.points.len(),
                            new: self.edges.len(),
                        };
                        let range = Interval::new(cut.t, old.range.hi())
                            .map_err(|_| unsupported("a band's seam at its edge's end"))?;
                        self.points.push(cut.point);
                        self.vertex_ids.push(self.edge_ids[cut.edge]);
                        self.edges.push(Edge {
                            geometry: old.geometry.clone(),
                            curve: old.curve,
                            range,
                            start: split.vertex,
                            end: old.end,
                            along: old.along,
                        });
                        self.edge_ids.push(self.edge_ids[cut.edge]);
                        for uses in &mut loops {
                            split.apply(uses);
                        }
                        self.splits.push(split);
                        let at =
                            junctions(&surface, &singular, &self.points, cap, precision, flipped);
                        if band(
                            &at,
                            &self.edges,
                            self.model,
                            &mut loops,
                            &mut self.rebuilt,
                            tol,
                            samples,
                            self.meter,
                        )
                        .map_err(unsupported)?
                        .is_some()
                        {
                            return Err(unsupported(
                                "a band whose split edge still leaves no seam",
                            ));
                        }
                    }
                }
                self.rebuilt_faces.resize(self.rebuilt.len(), face.id);
                faces.push(Walked {
                    surface,
                    surface_id,
                    flipped,
                    loops,
                });
            }
            walked.push(faces);
        }
        Ok(walked)
    }
}
