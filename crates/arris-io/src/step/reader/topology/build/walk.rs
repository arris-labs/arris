//! The walk phase: every face's bounds as loops of uses on its surface,
//! face by face and bound by bound, then the seams a face needs and the
//! file left out.

use std::collections::BTreeMap;

use arris_geom::{Curve2, Singularity, Surface};
use arris_math::{Interval, Point3, Precision};
use arris_topo::Orientation;

use super::{Build, Walked};
use crate::step::reader::Refusal;
use crate::step::reader::topology::{
    Bound, Cut, Edge, EdgeRef, FileFace, Fix, Jump, Junctions, Split, Step, Use, Walk, band,
    fitted, junctions, seam_crosses, seam_to, stopped_in, turn,
};

/// What walking one face's bounds shares.
struct FaceWalk<'f> {
    face: &'f FileFace,
    surface: Surface,
    singular: Vec<Singularity>,
    /// The face's effective normal is its surface's turned.
    flipped: bool,
    /// The face's `VERTEX_LOOP` — its bound's id and vertex — to be
    /// joined to its one other bound by a seam, which Arris's face needs
    /// and the file left out.
    joined: Option<(u64, usize)>,
}

impl FaceWalk<'_> {
    /// The judge of this face's junctions among `points`.
    fn junctions<'b>(
        &'b self,
        points: &'b [Point3],
        cap: f64,
        precision: Precision,
    ) -> Junctions<'b> {
        junctions(
            &self.surface,
            &self.singular,
            points,
            cap,
            precision,
            self.flipped,
        )
    }

    /// The refusal of this face for a seam it cannot have.
    fn unsupported(&self, name: &str) -> Refusal {
        Refusal::Unsupported {
            entity: self.face.id,
            name: name.into(),
        }
    }
}

impl Build<'_, '_> {
    /// Every shell's faces walked, in the file's order.
    pub(super) fn walk_shells(&mut self) -> Result<Vec<Vec<Walked>>, Refusal> {
        let file = self.file;
        let mut walked = Vec::with_capacity(file.shells.len());
        for (si, shell) in file.shells.iter().enumerate() {
            let mut faces = Vec::with_capacity(shell.faces.len());
            for (fi, face) in shell.faces.iter().enumerate() {
                faces.push(self.walk_face(face, self.turn_back.contains(&(si, fi)))?);
            }
            walked.push(faces);
        }
        Ok(walked)
    }

    /// One face: its bounds walked, then the seams its loops need.
    /// `turned_back` is the face being one the file walked against every
    /// neighbour.
    fn walk_face(&mut self, face: &FileFace, turned_back: bool) -> Result<Walked, Refusal> {
        self.tick()?;
        let (surface, surface_id) = self.surface(face)?;
        // Outward is the file face's normal turned by the shell or
        // an ORIENTED_FACE, the file face's normal is the file
        // surface's turned where `same_sense` is false, and the
        // variant's normal is the file surface's turned by
        // `reversed`.
        let flipped = face.surface.reversed ^ !face.same_sense ^ face.turned;
        let face_walk = FaceWalk {
            face,
            singular: surface.singularities(),
            surface,
            flipped,
            joined: vertex_join(face)?,
        };
        let mut pcurves: BTreeMap<usize, Curve2> = BTreeMap::new();
        let mut loops = Vec::with_capacity(face.bounds.len());
        for bound in &face.bounds {
            let Walk::Edges(ref walk) = bound.walk else {
                continue;
            };
            let mut walk = walk.clone();
            if face.turned != turned_back {
                turn(&mut walk);
            }
            loops.push(self.walk_bound(&face_walk, bound, walk, &mut pcurves)?);
        }
        if face_walk.joined.is_none() {
            self.close_band(&face_walk, &mut loops)?;
        }
        self.rebuilt_faces.resize(self.rebuilt.len(), face.id);
        Ok(Walked {
            surface: face_walk.surface,
            surface_id,
            flipped,
            loops,
        })
    }

    /// One `EDGE_LOOP` as uses with pcurves: joined by a seam to the
    /// face's `VERTEX_LOOP`, the degenerate edges at its singular points
    /// rebuilt, each use moved to meet the one before it.
    fn walk_bound(
        &mut self,
        w: &FaceWalk<'_>,
        bound: &Bound,
        walk: Vec<Step>,
        pcurves: &mut BTreeMap<usize, Curve2>,
    ) -> Result<Vec<Use>, Refusal> {
        let (cap, tol, precision) = (self.cap, self.tol, self.precision);
        let mut uses = self.bound_uses(w, walk, pcurves)?;
        let at = w.junctions(&self.points, cap, precision);
        let seam = match w.joined {
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
                    &w.surface,
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
                    face: w.face.id,
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
                    EdgeRef::Rebuilt(_) => w.face.id,
                },
                face: w.face.id,
                what: e.to_string(),
            })?;
        if let (Some(i), Some((vertex_bound, _))) = (seam, w.joined) {
            if seam_crosses(
                &uses,
                i,
                &w.surface,
                precision.parametric_tolerance,
                precision.check_samples,
            ) {
                return Err(Refusal::Unsupported {
                    entity: vertex_bound,
                    name: "a VERTEX_LOOP whose seam to its face's bound crosses it".into(),
                });
            }
        }
        Ok(uses)
    }

    /// The uses of a walk over each file edge's pieces, in the walk's
    /// direction, each with its pcurve on the face's surface. A pcurve is
    /// fitted once per edge, in `pcurves`.
    fn bound_uses(
        &mut self,
        w: &FaceWalk<'_>,
        walk: Vec<Step>,
        pcurves: &mut BTreeMap<usize, Curve2>,
    ) -> Result<Vec<Use>, Refusal> {
        let (cap, read_cap, precision) = (self.cap, self.read_cap, self.precision);
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
                    // Under the read cap first, so a curve that needs no
                    // healing is fitted as it would be without it.
                    let p = match fitted(
                        &edge.geometry,
                        edge.range,
                        &w.surface,
                        precision,
                        read_cap,
                        self.meter,
                    ) {
                        Err(fault) if cap > read_cap && fault.heals() => fitted(
                            &edge.geometry,
                            edge.range,
                            &w.surface,
                            precision,
                            cap,
                            self.meter,
                        ),
                        first => first,
                    }
                    .map_err(|fault| fault.refusal(id, w.face.id, cap))?;
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
        Ok(uses)
    }

    /// A face of loops that wrap a period gets the seam its surface
    /// needs: one wrapping loop is joined to the singular point it closes
    /// at, two are joined to each other, the other's edge split where no
    /// vertex faces the first.
    fn close_band(&mut self, w: &FaceWalk<'_>, loops: &mut Vec<Vec<Use>>) -> Result<(), Refusal> {
        let (cap, tol, precision) = (self.cap, self.tol, self.precision);
        let at = w.junctions(&self.points, cap, precision);
        let cut = band(
            &at,
            &self.edges,
            self.model,
            loops,
            &mut self.rebuilt,
            tol,
            precision.check_samples,
            self.meter,
        )
        .map_err(|name| w.unsupported(name))?;
        match cut {
            Some(Fix::Apex { face_loop, row }) => self.join_apex(w, loops, face_loop, row),
            Some(Fix::Cut(cut)) => self.split_for_band(w, loops, cut),
            None => Ok(()),
        }
    }

    /// The one wrapping loop `face_loop` joined to the singular point
    /// `row`, a vertex the file left out.
    fn join_apex(
        &mut self,
        w: &FaceWalk<'_>,
        loops: &mut [Vec<Use>],
        face_loop: usize,
        row: Singularity,
    ) -> Result<(), Refusal> {
        let (cap, tol, precision) = (self.cap, self.tol, self.precision);
        let apex = self.points.len();
        self.points.push(row.point);
        self.vertex_ids.push(w.face.id);
        let at = w.junctions(&self.points, cap, precision);
        let mut uses = std::mem::take(&mut loops[face_loop]);
        let seam = seam_to(
            self.model,
            &w.surface,
            row,
            apex,
            &uses,
            &self.points,
            tol,
            self.meter,
        )
        .map_err(|name| w.unsupported(name))?;
        let i = seam.join(&mut uses, &mut self.rebuilt);
        let mut uses = at.walk(uses, &mut self.rebuilt).map_err(|_| {
            w.unsupported("a face of one wrapping loop whose seam to its apex jumps")
        })?;
        at.meet(&mut uses, self.meter).map_err(|_| {
            w.unsupported("a face of one wrapping loop whose seam to its apex does not meet it")
        })?;
        if seam_crosses(&uses, i, &w.surface, at.parametric, precision.check_samples) {
            return Err(
                w.unsupported("a face of one wrapping loop whose seam to its apex crosses it")
            );
        }
        loops[face_loop] = uses;
        Ok(())
    }

    /// The seam's far end is a vertex on the other loop's edge, which is
    /// split there, on every loop that uses it; the seam is then found.
    fn split_for_band(
        &mut self,
        w: &FaceWalk<'_>,
        loops: &mut Vec<Vec<Use>>,
        cut: Cut,
    ) -> Result<(), Refusal> {
        let (cap, tol, precision) = (self.cap, self.tol, self.precision);
        if cut.edge >= self.file.edges.len() || self.splits.iter().any(|s| s.edge == cut.edge) {
            return Err(w.unsupported("a band whose seam would split an edge a second time"));
        }
        let old = &self.edges[cut.edge];
        let split = Split {
            edge: cut.edge,
            t: cut.t,
            vertex: self.points.len(),
            new: self.edges.len(),
        };
        let range = Interval::new(cut.t, old.range.hi())
            .map_err(|_| w.unsupported("a band's seam at its edge's end"))?;
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
        for uses in loops.iter_mut() {
            split.apply(uses);
        }
        self.splits.push(split);
        let at = w.junctions(&self.points, cap, precision);
        if band(
            &at,
            &self.edges,
            self.model,
            loops,
            &mut self.rebuilt,
            tol,
            precision.check_samples,
            self.meter,
        )
        .map_err(|name| w.unsupported(name))?
        .is_some()
        {
            return Err(w.unsupported("a band whose split edge still leaves no seam"));
        }
        Ok(())
    }
}

/// The face's `VERTEX_LOOP`, if it has one beside exactly one other
/// bound. Errors: a `VERTEX_LOOP` in any other company.
fn vertex_join(face: &FileFace) -> Result<Option<(u64, usize)>, Refusal> {
    let vertex_bounds: Vec<(u64, usize)> = (face.bounds.iter())
        .filter_map(|b| match b.walk {
            Walk::Vertex(v) => Some((b.id, v)),
            Walk::Edges(_) => None,
        })
        .collect();
    match vertex_bounds[..] {
        [] => Ok(None),
        [one] if face.bounds.len() == 2 => Ok(Some(one)),
        [(bound, _), ..] => Err(Refusal::Unsupported {
            entity: bound,
            name: "a VERTEX_LOOP beside other than one EDGE_LOOP".into(),
        }),
    }
}
