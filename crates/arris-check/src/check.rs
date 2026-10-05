//! `check`: the closure walk and the rows it evaluates.
//!
//! Rows implemented here: M1–M3, V1–V3 and E1–E7 (`docs/DATA-MODEL.md`
//! §Invariants); the loop, face, shell and body rows are in
//! `crate::topology` and the `Full` rows in `crate::full`, which `level`
//! selects. Every row walks the body's closure in sorted id
//! order and reports through `Report::new`, so the report is the same on
//! every run and platform.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Frame2, Interval, Point3, Precision, RELATIVE_ROUNDING};
use arris_topo::entity::{BodyKind, Coedge, Edge, EdgeGeometry, Face};
use arris_topo::euler::EulerLine;
use arris_topo::{
    Body, Closure, CoedgeRef, EdgeId, EntityId, FaceId, Model, NotFound, Orientation, VertexId,
};

use crate::domain::{FaceDomain, bands};
use crate::report::Report;
use crate::unchecked::Unchecked;
use crate::violation::{
    DegenerateFault, EndMismatch, Level, Quantity, Reference, SeamFault, ToleranceBound, Violation,
};

/// Every violation of `docs/DATA-MODEL.md` §Invariants that `body`
/// exhibits in `model`, at `level` — `Fast` after every operation, `Full`
/// on demand — as a [`Report`] sorted by entity then row. A body handle
/// that does not resolve is one `M1` line naming the body. Never panics
/// and never repairs: a reference that does not resolve is reported once
/// under M1 and skipped by every other row.
///
/// ```
/// use arris_check::{Level, check};
/// use arris_debug::sample;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// assert!(check(&m, body, Level::Fast).is_ok());
/// ```
pub fn check(model: &Model, body: Body, level: Level) -> Report {
    let Ok(mut c) = Checker::new(model, body) else {
        return Report::new(vec![Violation::Unresolved {
            from: body.id.into(),
            to: Reference::Entity(body.id.into()),
        }]);
    };
    c.references();
    c.indices();
    c.finiteness();
    c.vertex_rows();
    c.edge_rows();
    c.face_rows();
    c.shell_rows();
    c.body_rows();
    if level == Level::Full {
        c.full_rows();
    }
    // Linear in the closure, so it is taken at every level.
    let line = EulerLine::of(model, &c.closure);
    let unchecked = core::mem::take(&mut c.unchecked);
    let violations = c
        .violations
        .into_iter()
        .filter(|v| v.level() <= level)
        .collect();
    Report::new(violations)
        .with_euler(line)
        .with_unchecked(unchecked)
}

pub(crate) struct Checker<'m> {
    pub(crate) model: &'m Model,
    pub(crate) precision: Precision,
    pub(crate) body: Body,
    pub(crate) closure: Closure,
    /// Edge → its uses by the body's faces, in face creation order.
    pub(crate) uses: BTreeMap<EdgeId, Vec<(CoedgeRef, Coedge)>>,
    /// Vertex → the body's edges that end at it, each once.
    pub(crate) vertex_edges: BTreeMap<VertexId, Vec<EdgeId>>,
    pub(crate) violations: Vec<Violation>,
    /// Every face's domain at the model's parametric tolerance, built by
    /// the `Full` rows and empty at `Fast`.
    pub(crate) domains: BTreeMap<FaceId, FaceDomain<'m>>,
    /// The `Full` rows this body could not be decided on.
    pub(crate) unchecked: Vec<Unchecked>,
}

impl<'m> Checker<'m> {
    /// A checker over `body`'s closure with no row run yet: what [`check`]
    /// runs the rows on and `crate::lumps` runs B1's nesting on. Errors:
    /// the body handle does not resolve.
    pub(crate) fn new(model: &'m Model, body: Body) -> Result<Self, NotFound> {
        let closure = model.closure(body)?;
        // Adjacency is read off the closure's own entities, not the arena's
        // indices, so every row stands on its own and M2 is the only row
        // that says anything about the indices.
        let mut uses: BTreeMap<EdgeId, Vec<(CoedgeRef, Coedge)>> = BTreeMap::new();
        for &face_id in &closure.faces {
            let Ok(face) = model.face(face_id) else {
                continue;
            };
            for (loop_index, coedge_index, coedge) in coedges(face) {
                uses.entry(coedge.edge()).or_default().push((
                    CoedgeRef {
                        face: face_id,
                        loop_index,
                        coedge_index,
                    },
                    coedge,
                ));
            }
        }
        let mut vertex_edges: BTreeMap<VertexId, Vec<EdgeId>> = BTreeMap::new();
        for &edge_id in &closure.edges {
            let Ok(edge) = model.edge(edge_id) else {
                continue;
            };
            for v in [edge.start(), edge.end()] {
                let list = vertex_edges.entry(v).or_default();
                if list.last() != Some(&edge_id) {
                    list.push(edge_id);
                }
            }
        }
        Ok(Checker {
            model,
            precision: model.precision(),
            body,
            closure,
            uses,
            vertex_edges,
            violations: Vec::new(),
            domains: BTreeMap::new(),
            unchecked: Vec::new(),
        })
    }
}

/// `(face, loop index, coedge index, coedge)` for every coedge of a face.
pub(crate) fn coedges(face: &Face) -> impl Iterator<Item = (usize, usize, Coedge)> + '_ {
    face.loops().iter().enumerate().flat_map(|(li, l)| {
        l.coedges()
            .iter()
            .enumerate()
            .map(move |(ci, &c)| (li, ci, c))
    })
}

/// `n` parameters over `range`, both ends included; the midpoint alone
/// when `n` is one.
pub(crate) fn samples(range: Interval, n: usize) -> Vec<f64> {
    if n <= 1 {
        return vec![range.midpoint()];
    }
    (0..n)
        .map(|i| range.lerp(i as f64 / (n - 1) as f64))
        .collect()
}

fn all_finite(xs: impl IntoIterator<Item = f64>) -> bool {
    xs.into_iter().all(f64::is_finite)
}

fn frame_finite(f: &Frame) -> bool {
    all_finite(
        f.origin()
            .coords
            .iter()
            .chain(f.x().iter())
            .chain(f.y().iter())
            .chain(f.z().iter())
            .copied(),
    )
}

fn frame2_finite(f: &Frame2) -> bool {
    all_finite(
        f.origin()
            .coords
            .iter()
            .chain(f.x().iter())
            .chain(f.y().iter())
            .copied(),
    )
}

/// Which quantities of a curve are non-finite: `(coordinates, parameters)`.
fn curve_non_finite(c: &Curve) -> (bool, bool) {
    match c {
        Curve::Line { origin, direction } => (
            !all_finite(origin.coords.iter().chain(direction.iter()).copied()),
            false,
        ),
        Curve::Circle { frame, radius } => (!frame_finite(frame), !radius.is_finite()),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => (
            !frame_finite(frame),
            !(major_radius.is_finite() && minor_radius.is_finite()),
        ),
        Curve::Nurbs(n) => (
            !all_finite(
                n.control_points()
                    .iter()
                    .flat_map(|p| p.coords.iter().copied()),
            ),
            !all_finite(n.knots().iter().chain(n.weights()).copied()),
        ),
    }
}

fn surface_non_finite(s: &Surface) -> (bool, bool) {
    match s {
        Surface::Plane { frame } => (!frame_finite(frame), false),
        Surface::Cylinder { frame, radius } | Surface::Sphere { frame, radius } => {
            (!frame_finite(frame), !radius.is_finite())
        }
        Surface::Cone {
            frame,
            radius,
            half_angle,
        } => (
            !frame_finite(frame),
            !(radius.is_finite() && half_angle.is_finite()),
        ),
        Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        }
        | Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        } => (
            !frame_finite(frame),
            !(major_radius.is_finite() && minor_radius.is_finite()),
        ),
        Surface::Nurbs(n) => (
            !all_finite(
                n.control_points()
                    .iter()
                    .flat_map(|p| p.coords.iter().copied()),
            ),
            !all_finite(
                n.knots()
                    .iter()
                    .flat_map(|k| k.iter())
                    .chain(n.weights())
                    .copied(),
            ),
        ),
    }
}

/// A pcurve's numbers are all parameters.
fn pcurve_non_finite(c: &Curve2) -> bool {
    match c {
        Curve2::Line { origin, direction } => {
            !all_finite(origin.coords.iter().chain(direction.iter()).copied())
        }
        Curve2::Circle { frame, radius } => !frame2_finite(frame) || !radius.is_finite(),
        Curve2::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => !frame2_finite(frame) || !(major_radius.is_finite() && minor_radius.is_finite()),
        Curve2::Nurbs(n) => !all_finite(
            n.control_points()
                .iter()
                .flat_map(|p| p.coords.iter().copied())
                .chain(n.knots().iter().copied())
                .chain(n.weights().iter().copied()),
        ),
    }
}

impl Checker<'_> {
    pub(crate) fn push(&mut self, v: Violation) {
        self.violations.push(v);
    }

    /// The uses of `edge` by faces of the body, in face creation order.
    fn uses_in_body(&self, edge: EdgeId) -> Vec<(CoedgeRef, Coedge)> {
        self.uses.get(&edge).cloned().unwrap_or_default()
    }

    /// M1: every id an entity of the body references resolves.
    fn references(&mut self) {
        let mut unresolved: Vec<(EntityId, Reference)> = Vec::new();
        let body_id: EntityId = self.body.id.into();
        if let Ok(body) = self.model.body(self.body.id) {
            for s in body.shells() {
                if self.model.shell(s.id).is_err() {
                    unresolved.push((body_id, Reference::Entity(s.id.into())));
                }
            }
            for e in body.free_edges() {
                if self.model.edge(e.id).is_err() {
                    unresolved.push((body_id, Reference::Entity(e.id.into())));
                }
            }
            for &v in body.free_vertices() {
                if self.model.vertex(v).is_err() {
                    unresolved.push((body_id, Reference::Entity(v.into())));
                }
            }
        }
        for &shell_id in &self.closure.shells {
            let Ok(shell) = self.model.shell(shell_id) else {
                continue;
            };
            for f in shell.faces() {
                if self.model.face(f.id).is_err() {
                    unresolved.push((shell_id.into(), Reference::Entity(f.id.into())));
                }
            }
        }
        for &face_id in &self.closure.faces {
            let Ok(face) = self.model.face(face_id) else {
                continue;
            };
            if self.model.surface(face.surface()).is_err() {
                unresolved.push((face_id.into(), Reference::Geometry(face.surface().into())));
            }
            for (_, _, coedge) in coedges(face) {
                if self.model.edge(coedge.edge()).is_err() {
                    unresolved.push((face_id.into(), Reference::Entity(coedge.edge().into())));
                }
                if self.model.curve2(coedge.pcurve()).is_err() {
                    unresolved.push((face_id.into(), Reference::Geometry(coedge.pcurve().into())));
                }
            }
        }
        for &edge_id in &self.closure.edges {
            let Ok(edge) = self.model.edge(edge_id) else {
                continue;
            };
            for v in [edge.start(), edge.end()] {
                if self.model.vertex(v).is_err() {
                    unresolved.push((edge_id.into(), Reference::Entity(v.into())));
                }
            }
            if let Some((curve, _)) = edge.curve() {
                if self.model.curve(curve).is_err() {
                    unresolved.push((edge_id.into(), Reference::Geometry(curve.into())));
                }
            }
        }
        unresolved.sort();
        unresolved.dedup();
        for (from, to) in unresolved {
            self.push(Violation::Unresolved { from, to });
        }
    }

    /// M2: every reference a parent makes is in the adjacency index of the
    /// entity it names.
    fn indices(&mut self) {
        let mut missing: BTreeSet<(EntityId, EntityId)> = BTreeSet::new();
        for &shell_id in &self.closure.shells {
            let Ok(shell) = self.model.shell(shell_id) else {
                continue;
            };
            for f in shell.faces() {
                if let Ok(shells) = self.model.face_shells(f.id) {
                    if !shells.contains(&shell_id) {
                        missing.insert((f.id.into(), shell_id.into()));
                    }
                }
            }
        }
        for &face_id in &self.closure.faces {
            let Ok(face) = self.model.face(face_id) else {
                continue;
            };
            for (li, ci, coedge) in coedges(face) {
                if let Ok(uses) = self.model.edge_uses(coedge.edge()) {
                    let expected = CoedgeRef {
                        face: face_id,
                        loop_index: li,
                        coedge_index: ci,
                    };
                    if !uses.contains(&expected) {
                        missing.insert((coedge.edge().into(), face_id.into()));
                    }
                }
            }
        }
        for &edge_id in &self.closure.edges {
            let Ok(edge) = self.model.edge(edge_id) else {
                continue;
            };
            for v in [edge.start(), edge.end()] {
                if let Ok(edges) = self.model.vertex_edges(v) {
                    if !edges.contains(&edge_id) {
                        missing.insert((v.into(), edge_id.into()));
                    }
                }
            }
        }
        for (entity, referenced_by) in missing {
            self.push(Violation::NotIndexed {
                entity,
                referenced_by,
            });
        }
    }

    /// M3: every coordinate, parameter and tolerance is finite, reported
    /// against the entity holding or referencing the number.
    fn finiteness(&mut self) {
        let mut found: BTreeSet<(EntityId, Quantity)> = BTreeSet::new();
        for &id in &self.closure.vertices {
            let Ok(v) = self.model.vertex(id) else {
                continue;
            };
            if !all_finite(v.point().coords.iter().copied()) {
                found.insert((id.into(), Quantity::Coordinate));
            }
            if !v.tolerance().is_finite() {
                found.insert((id.into(), Quantity::Tolerance));
            }
        }
        for &id in &self.closure.edges {
            let Ok(e) = self.model.edge(id) else {
                continue;
            };
            let range = e.range();
            if !(range.lo().is_finite() && range.hi().is_finite()) {
                found.insert((id.into(), Quantity::Parameter));
            }
            if !e.tolerance().is_finite() {
                found.insert((id.into(), Quantity::Tolerance));
            }
            if let Some(curve) = e.curve().and_then(|(c, _)| self.model.curve(c).ok()) {
                let (coords, params) = curve_non_finite(curve);
                if coords {
                    found.insert((id.into(), Quantity::Coordinate));
                }
                if params {
                    found.insert((id.into(), Quantity::Parameter));
                }
            }
        }
        for &id in &self.closure.faces {
            let Ok(f) = self.model.face(id) else {
                continue;
            };
            if !f.tolerance().is_finite() {
                found.insert((id.into(), Quantity::Tolerance));
            }
            if let Ok(surface) = self.model.surface(f.surface()) {
                let (coords, params) = surface_non_finite(surface);
                if coords {
                    found.insert((id.into(), Quantity::Coordinate));
                }
                if params {
                    found.insert((id.into(), Quantity::Parameter));
                }
            }
            for (_, _, coedge) in coedges(f) {
                if let Ok(pcurve) = self.model.curve2(coedge.pcurve()) {
                    if pcurve_non_finite(pcurve) {
                        found.insert((id.into(), Quantity::Parameter));
                    }
                }
            }
        }
        for (entity, quantity) in found {
            self.push(Violation::NonFinite { entity, quantity });
        }
    }

    /// V1–V3.
    fn vertex_rows(&mut self) {
        let precision = self.precision;
        let mut off_edge: Vec<(VertexId, EdgeId, f64)> = Vec::new();
        for id in self.closure.vertices.clone() {
            let Ok(v) = self.model.vertex(id) else {
                continue;
            };
            let tolerance = v.tolerance();
            // V1
            if tolerance.is_finite() {
                if tolerance < precision.min_tolerance {
                    self.push(Violation::VertexTolerance {
                        vertex: id,
                        tolerance,
                        bound: ToleranceBound::BelowMinimum,
                    });
                } else if tolerance > precision.max_tolerance {
                    self.push(Violation::VertexTolerance {
                        vertex: id,
                        tolerance,
                        bound: ToleranceBound::AboveMaximum,
                    });
                }
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

    /// E1–E7.
    fn edge_rows(&mut self) {
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
                        for t in samples(range, precision.check_samples) {
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
    /// the range, to the edge's tolerance.
    fn e2_ends(&mut self, edge_id: EdgeId, edge: &Edge, curve: &Curve, range: Interval) {
        let gap = (curve.point(range.hi()) - curve.point(range.lo())).norm();
        let closed = gap <= edge.tolerance();
        let fault = match (closed, edge.start() == edge.end()) {
            (true, false) => Some(EndMismatch::ClosedWithTwoVertices {
                start: edge.start(),
                end: edge.end(),
            }),
            (false, true) => Some(EndMismatch::OpenWithOneVertex {
                vertex: edge.start(),
                gap,
            }),
            _ => None,
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
            let images: Vec<Point3> = samples(range, self.precision.check_samples)
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
