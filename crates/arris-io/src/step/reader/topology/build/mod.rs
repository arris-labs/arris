//! The body of a solid read: the file's edges and faces resolved, walked,
//! measured and handed to the builder, one phase to a method of [`Build`],
//! in the order [`Geometry::build`] calls them — the pattern of
//! `arris_ops::blend::build`.
//!
//! 1. [`Build::new`] reads the vertices' points and names the faces the
//!    file walked against every neighbour.
//! 2. [`Build::edges`] resolves each edge's curve, bounds the part and
//!    sets the gap caps.
//! 3. [`Build::split_at_singular_points`] splits an edge through a
//!    singular point of a face it bounds, before any face is walked.
//! 4. [`Build::walk_shells`] walks every face's bounds into loops of uses
//!    with their pcurves, rebuilding the seams and degenerate edges the
//!    file left out.
//! 5. [`Build::propagate_splits`] splits every other use of an edge a
//!    seam split.
//! 6. [`Build::assemble`] places the loops, measures the gaps and states
//!    the assembly.
//! 7. [`Build::finish`] builds the body, names what healing grew, checks
//!    it and records the provenance.

mod assemble;
mod finish;
mod walk;

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::{PCURVE_SINGULAR_BAND, Surface};
use arris_math::{
    Aabb, HEAL_GAP_FRACTION, Interval, Meter, Point3, Precision, READ_GAP_FRACTION, Tolerance,
};
use arris_topo::{CurveId, Model, SurfaceId};

use super::{
    Edge, EdgeRef, FileFace, FileSolid, Rebuilt, Solid, Split, Use, ball_of, edge_range,
    stopped_in, through, turned_against_neighbours,
};
use crate::step::reader::geometry::{Ball, Geometry};
use crate::step::reader::{Heal, Refusal};

/// A face walked: its surface and the loops of uses on it.
struct Walked {
    surface: Surface,
    surface_id: SurfaceId,
    /// The face's effective normal is its surface's turned.
    flipped: bool,
    loops: Vec<Vec<Use>>,
}

/// One solid's read in progress: what the file gives, the model it is
/// read into, and what the phases have made so far.
struct Build<'a, 'm> {
    model: &'a mut Model,
    meter: &'a mut Meter<'m>,
    file: &'a FileSolid,
    solid: u64,
    instance: u32,
    heal: Heal,
    precision: Precision,
    tol: Tolerance,
    /// The model's default tolerance.
    tolerance: f64,
    points: Vec<Point3>,
    /// The ball every unbounded curve and surface is bounded by.
    part: Ball,
    edges: Vec<Edge>,
    /// The read cap; healing, the healing ceiling, which the model's
    /// `max_tolerance` does not bound (ADR-0058 §2).
    read_cap: f64,
    cap: f64,
    surfaces: BTreeMap<u64, (Surface, SurfaceId)>,
    /// The edges the file left out, and the face each is rebuilt on.
    rebuilt: Vec<Rebuilt>,
    rebuilt_faces: Vec<u64>,
    /// The file entity each vertex and edge stands for: a vertex or an
    /// edge a band's seam splits off stands for the edge it splits.
    vertex_ids: Vec<u64>,
    edge_ids: Vec<u64>,
    splits: Vec<Split>,
    /// Each file edge as its pieces, in its curve's order.
    pieces: Vec<Vec<usize>>,
    /// The faces whose walk the file turned against every neighbour,
    /// turned back (ADR-0058 §4).
    turn_back: BTreeSet<(usize, usize)>,
}

impl Geometry<'_> {
    /// The body of `file`, appended to `model`.
    pub(super) fn build(
        &self,
        model: &mut Model,
        solid: u64,
        instance: u32,
        heal: Heal,
        file: &FileSolid,
        meter: &mut Meter<'_>,
    ) -> Result<Solid, Refusal> {
        let mut build = Build::new(self, model, solid, instance, heal, file, meter)?;
        build.edges()?;
        build.split_at_singular_points()?;
        let mut walked = build.walk_shells()?;
        build.propagate_splits(&mut walked)?;
        let (assembly, tolerances) = build.assemble(walked)?;
        build.finish(assembly, tolerances)
    }
}

impl<'a, 'm> Build<'a, 'm> {
    /// The points of the file's vertices, and the faces it turned.
    fn new(
        geometry: &'a Geometry<'a>,
        model: &'a mut Model,
        solid: u64,
        instance: u32,
        heal: Heal,
        file: &'a FileSolid,
        meter: &'a mut Meter<'m>,
    ) -> Result<Self, Refusal> {
        let precision = model.precision();
        let tolerance = precision.default_tolerance;
        let points = file
            .vertices
            .iter()
            .map(|&v| {
                let args = geometry.entities.record(v, v, "VERTEX_POINT")?;
                geometry
                    .units
                    .point(&geometry.entities, v, args.reference(1)?)
            })
            .collect::<Result<Vec<Point3>, Refusal>>()?;
        // An unbounded curve is bounded by the vertices alone; every
        // surface by those and every edge.
        let part = ball_of(points.iter().map(|&p| Aabb::of_point(p)), tolerance);
        let turn_back = match heal {
            Heal::Off => BTreeSet::new(),
            Heal::GapsAndOrientation => turned_against_neighbours(file),
        };
        Ok(Build {
            model,
            meter,
            file,
            solid,
            instance,
            heal,
            precision,
            tol: precision.tolerance(),
            tolerance,
            points,
            part,
            edges: Vec::with_capacity(file.edges.len()),
            read_cap: tolerance,
            cap: tolerance,
            surfaces: BTreeMap::new(),
            rebuilt: Vec::new(),
            rebuilt_faces: Vec::new(),
            vertex_ids: file.vertices.clone(),
            edge_ids: file.edges.iter().map(|e| e.id).collect(),
            splits: Vec::new(),
            pieces: (0..file.edges.len()).map(|i| vec![i]).collect(),
            turn_back,
        })
    }

    /// One step of the meter, the caller's stop as a refusal.
    fn tick(&mut self) -> Result<(), Refusal> {
        self.meter.tick().map_err(|_| stopped_in(self.solid))
    }

    /// The face's surface, resolved and added to the model at its first
    /// use.
    fn surface(&mut self, face: &FileFace) -> Result<(Surface, SurfaceId), Refusal> {
        if let Some(s) = self.surfaces.get(&face.surface.id) {
            return Ok(s.clone());
        }
        let s = face.surface.resolve(&self.part)?;
        let id = self.model.add_surface(s.clone());
        self.surfaces.insert(face.surface.id, (s.clone(), id));
        Ok((s, id))
    }

    /// Every file edge resolved: its curve (added to the model once), the
    /// vertices it runs between along that curve and its range. Then the
    /// part is bounded by the vertices and the edges, and the gap caps
    /// set from it.
    fn edges(&mut self) -> Result<(), Refusal> {
        let file = self.file;
        let mut curves: BTreeMap<u64, (arris_geom::Curve, CurveId)> = BTreeMap::new();
        for e in &file.edges {
            self.tick()?;
            let (curve, curve_id) = match curves.get(&e.curve.id) {
                Some(c) => c.clone(),
                None => {
                    let c = e.curve.resolve(&self.part)?;
                    let id = self.model.add_curve(c.clone());
                    curves.insert(e.curve.id, (c.clone(), id));
                    (c, id)
                }
            };
            // The Arris edge runs along its curve.
            let along = e.same_sense != e.curve.reversed;
            let (start, end) = if along {
                (e.start, e.end)
            } else {
                (e.end, e.start)
            };
            let range = edge_range(
                e.id,
                &curve,
                self.points[start],
                self.points[end],
                start == end,
            )?;
            self.edges.push(Edge {
                geometry: curve,
                curve: curve_id,
                range,
                start,
                end,
                along,
            });
        }
        let boxes = self.edges.iter().filter_map(|e| e.geometry.bounds(e.range));
        self.part = ball_of(
            self.points.iter().map(|&p| Aabb::of_point(p)).chain(boxes),
            self.tolerance,
        );
        self.read_cap = (READ_GAP_FRACTION * 2.0 * self.part.radius)
            .min(self.precision.max_tolerance)
            .max(self.tolerance);
        self.cap = match self.heal {
            Heal::Off => self.read_cap,
            Heal::GapsAndOrientation => {
                (HEAL_GAP_FRACTION * 2.0 * self.part.radius).max(self.read_cap)
            }
        };
        Ok(())
    }

    /// An edge whose curve runs through a singular point of a face it
    /// bounds — a meridian circle through a sphere's pole — is split
    /// there before any face is walked, a vertex on the point: no pcurve
    /// runs through it, and each side of it has one (ADR-0021). Each file
    /// edge then walks as its pieces, in its curve's order.
    fn split_at_singular_points(&mut self) -> Result<(), Refusal> {
        let file = self.file;
        let on_point = PCURVE_SINGULAR_BAND * self.tol.linear;
        for face in file.shells.iter().flat_map(|shell| &shell.faces) {
            self.tick()?;
            let (surface, _) = self.surface(face)?;
            for row in surface.singularities() {
                for bound in &face.bounds {
                    let super::Walk::Edges(ref walk) = bound.walk else {
                        continue;
                    };
                    for step in walk {
                        while let Some((at, p, t)) = (self.pieces[step.edge].iter().enumerate())
                            .find_map(|(at, &p)| {
                                Some((at, p, through(&self.edges[p], row.point, on_point)?))
                            })
                        {
                            let old = &self.edges[p];
                            let (Ok(before), Ok(after)) = (
                                Interval::new(old.range.lo(), t),
                                Interval::new(t, old.range.hi()),
                            ) else {
                                break;
                            };
                            let (vertex, new) = (self.points.len(), self.edges.len());
                            let piece = Edge {
                                geometry: old.geometry.clone(),
                                curve: old.curve,
                                range: after,
                                start: vertex,
                                end: old.end,
                                along: old.along,
                            };
                            self.points.push(row.point);
                            self.vertex_ids.push(self.edge_ids[p]);
                            self.edges[p].range = before;
                            self.edges[p].end = vertex;
                            self.edges.push(piece);
                            self.edge_ids.push(self.edge_ids[p]);
                            self.pieces[step.edge].insert(at + 1, new);
                        }
                    }
                }
            }
        }
        Ok(())
    }

    /// The use an edge stands for, as the key the gap tables are keyed by:
    /// a file edge by its index, a rebuilt one after them.
    fn edge_key(&self, edge: EdgeRef) -> usize {
        match edge {
            EdgeRef::File(i) => i,
            EdgeRef::Rebuilt(i) => self.edges.len() + i,
        }
    }
}
