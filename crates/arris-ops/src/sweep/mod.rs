//! The sweeps: a planar [`Profile`] becomes a solid by `extrude`,
//! `revolve` or `sweep` along a path, or a one-face sheet by
//! `planar_face`. A sweep's faces are known outright, so each enters the
//! builder through `Builder::assemble` (ADR-0004's entry, as `transform`
//! uses it) and records every entity `Generated` from a [`SweepPart`]
//! naming the part of the sketch it came from (`docs/ARCHITECTURE.md`
//! §Operations, `docs/DATA-MODEL.md` §Provenance).
//!
//! `extrude`, `revolve` and `sweep` (in `along`) have a file each; this
//! one holds what they share: the cap and side face specs, the walk of a
//! loop's uses and the provenance record.

mod along;
mod exact;
mod extrude;
mod loft;
mod refuse;
mod revolve;

pub use along::sweep;
pub use extrude::{extrude, planar_face};
pub use loft::{LoftKind, loft};
pub use revolve::revolve;

use arris_geom::{Curve, Curve2, GeomError, GeomKind, ProfileEdge, Surface};
use arris_math::{Frame, Interval, Point2, Vec2, Vec3};
use arris_topo::builder::{AssemblySlots, Built, EdgeKey, FaceSpec, UseSpec};
use arris_topo::provenance::SweepPart;
use arris_topo::{EntityId, Model, Orientation, Provenance, Role, Shape};

use crate::error::{Fault, OpError, Reason};

fn degenerate(reason: Reason) -> OpError {
    OpError::Degenerate {
        entities: Vec::new(),
        reason,
    }
}

/// An entity the fixed sequence of a sweep should have made for `edge`
/// and did not: a kernel bug, never a property of the sketch.
fn unmade(edge: &ProfileEdge) -> OpError {
    OpError::Internal(Fault::Unmade {
        segment: edge.segment,
    })
}

/// A profile edge whose curve is not one `Profile::edges` makes: a
/// kernel bug, never a property of the sketch.
fn profile_curve_fault(edge: &ProfileEdge) -> OpError {
    OpError::Internal(Fault::ProfileCurve(GeomError::Unsupported {
        a: GeomKind::Curve(edge.curve.kind()),
        b: GeomKind::Curve2(edge.pcurve.kind()),
    }))
}

/// One use of an edge in a side face's loop before its pcurve is placed:
/// the edge's slot, the direction it is walked in, its exact pcurve as
/// `pcurve_on` gave it, and the edge's range.
struct SideUse {
    edge: usize,
    orientation: Orientation,
    pcurve: Curve2,
    range: Interval,
}

/// Every pcurve of a loop translated by whole periods into the copy of
/// the surface's domain the loop is written in: the first use nearest
/// `u = 0`, each later use continuous with the end of the one before it.
/// `pcurve_on` reports a periodic parameter in `[0, 2π)`, so a curve in
/// the profile plane — at `u = 0` by the frame convention — can come back
/// at `2π` less a rounding, and a seam's second use is the first
/// translated by the period (`docs/DATA-MODEL.md` §Seams).
fn place_in_domain(surface: &Surface, uses: &mut [SideUse]) {
    let periods = surface.period();
    let mut previous_end: Option<Point2> = None;
    for u in uses.iter_mut() {
        let (t0, t1) = if u.orientation.is_reversed() {
            (u.range.hi(), u.range.lo())
        } else {
            (u.range.lo(), u.range.hi())
        };
        let start = u.pcurve.point(t0);
        let target = previous_end.unwrap_or(Point2::new(0.0, start.y));
        let mut by = Vec2::zeros();
        for (dir, period) in periods.iter().enumerate() {
            if let Some(period) = period {
                by[dir] = ((target[dir] - start[dir]) / period).round() * period;
            }
        }
        if by != Vec2::zeros() {
            u.pcurve = u.pcurve.translated(by);
        }
        previous_end = Some(u.pcurve.point(t1));
    }
}

/// Whether the swept face's surface normal is the outward one, read at
/// the middle of `range` on `curve`, the segment as it stands on the
/// plane with unit `normal`: outward is the segment's in-plane normal on
/// the right of the walk (the material is on the loop's left), and the
/// face is `Forward` when the surface normal there agrees with it.
/// `pcurve` is the segment's on `surface`, at the curve's own parameter.
fn side_orientation(
    curve: &Curve,
    range: Interval,
    normal: Vec3,
    surface: &Surface,
    pcurve: &Curve2,
) -> Result<Orientation, OpError> {
    let t = range.midpoint();
    let tangent = curve.eval(t).d1;
    let outward = tangent.cross(&normal);
    let uv = pcurve.point(t);
    let surface_normal = surface
        .normal(uv.x, uv.y)
        .ok_or(OpError::Internal(Fault::Invariant {
            what: "the swept surface's normal at the segment's midpoint",
        }))?;
    Ok(if surface_normal.dot(&outward) > 0.0 {
        Orientation::Forward
    } else {
        Orientation::Reversed
    })
}

/// The consumer's index of the vertex an oriented edge starts at: the
/// segment's own when the loop was not turned round, the next segment's
/// when it was, since then the walk runs the segment backwards.
fn vertex_index(edge: &ProfileEdge, segments: usize) -> usize {
    if edge.reversed {
        (edge.segment + 1) % segments
    } else {
        edge.segment
    }
}

/// Each profile edge's own pcurve on the profile's plane, per loop in
/// walking order: what a cap of analytic edges is bounded by.
fn profile_pcurves(loops: &[Vec<ProfileEdge>]) -> Vec<Vec<Curve2>> {
    loops
        .iter()
        .map(|edges| edges.iter().map(|e| e.pcurve.clone()).collect())
        .collect()
}

/// A flat end of a sweep on the plane `frame`: every loop of the profile
/// walked as oriented, each use `Forward` with its pcurve from `pcurves`,
/// and turned round when the face is used `Reversed`, so the material
/// stays on the walk's left seen from outside. A rigid motion carries the
/// parametrisation, so an end moved off the profile's plane has the
/// profile's own pcurves. `slots` holds each edge's slot, `None` for an
/// edge the end does not hold.
fn cap_face(
    m: &mut Model,
    pcurves: &[Vec<Curve2>],
    frame: Frame,
    orientation: Orientation,
    slots: &[Vec<Option<usize>>],
    tolerance: f64,
) -> FaceSpec {
    let surface = m.add_surface(Surface::Plane { frame });
    let mut cap_loops = Vec::with_capacity(pcurves.len());
    for (li, pcurves_of) in pcurves.iter().enumerate() {
        let mut uses: Vec<UseSpec> = pcurves_of
            .iter()
            .enumerate()
            .filter_map(|(j, pcurve)| {
                Some(UseSpec {
                    edge: EdgeKey::New(slots[li][j]?),
                    orientation: Orientation::Forward,
                    pcurve: m.add_curve2(pcurve.clone()),
                })
            })
            .collect();
        if orientation.is_reversed() {
            uses.reverse();
            for u in &mut uses {
                u.orientation = u.orientation.flipped();
            }
        }
        cap_loops.push(uses);
    }
    FaceSpec::New {
        surface,
        orientation,
        loops: cap_loops,
        tolerance,
    }
}

/// A side face of a sweep on `surface`: its cycle of uses placed in the
/// surface's domain and written as one loop — or, with `split`, one loop
/// per use, as an annulus of a full turn keeps its two closed rises.
fn side_face(
    m: &mut Model,
    surface: &Surface,
    orientation: Orientation,
    cycle: Vec<SideUse>,
    split: bool,
    tolerance: f64,
) -> FaceSpec {
    let mut side_loops: Vec<Vec<SideUse>> = if split {
        cycle.into_iter().map(|u| vec![u]).collect()
    } else {
        vec![cycle]
    };
    let surface_id = m.add_surface(surface.clone());
    let mut spec_loops = Vec::with_capacity(side_loops.len());
    for uses in &mut side_loops {
        place_in_domain(surface, uses);
        spec_loops.push(
            uses.iter()
                .map(|u| UseSpec {
                    edge: EdgeKey::New(u.edge),
                    orientation: u.orientation,
                    pcurve: m.add_curve2(u.pcurve.clone()),
                })
                .collect::<Vec<_>>(),
        );
    }
    FaceSpec::New {
        surface: surface_id,
        orientation,
        loops: spec_loops,
        tolerance,
    }
}

/// The roles of an assembled sweep, running parallel to its specs, as one
/// `Generated` per entity. `slots` names the entity behind each spec, in
/// spec order, so the vertex, edge and face role lists zip with `slots`'
/// lists, and the shells zip with `built.shells` directly, one per
/// assembly shell in order.
fn record(
    built: &Built,
    slots: &AssemblySlots,
    vertex_roles: &[Role],
    edge_roles: &[Role],
    face_roles: &[Role],
    shell_roles: &[Role],
    part: impl Fn(SweepPart) -> Role,
) -> Provenance {
    let mut provenance = Provenance::new();
    let forward = |id: EntityId| Shape::new(id, Orientation::Forward);
    for (&slot, &role) in slots.vertices.iter().zip(vertex_roles) {
        provenance.add_generated(role, forward(built.vertices[&slot].into()));
    }
    for (&slot, &role) in slots.edges.iter().zip(edge_roles) {
        provenance.add_generated(role, forward(built.edges[&slot].into()));
    }
    for (&slot, &role) in slots.faces.iter().flatten().zip(face_roles) {
        provenance.add_generated(role, forward(built.faces[&slot].into()));
    }
    for (&shell, &role) in built.shells.iter().zip(shell_roles) {
        provenance.add_generated(role, forward(shell.into()));
    }
    provenance.add_generated(part(SweepPart::Body), built.body);
    provenance
}
