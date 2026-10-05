//! The sweeps: a planar [`Profile`] becomes a solid by `extrude` or
//! `revolve`. A sweep's faces are known
//! outright, so both enter the builder through `Builder::assemble`
//! (ADR-0004's entry, as `transform` uses it) and record every entity
//! `Generated` from a [`SweepPart`] naming the part of the sketch it came
//! from (`docs/ARCHITECTURE.md` §Operations, `docs/DATA-MODEL.md`
//! §Provenance).

use core::f64::consts::TAU;
use std::collections::BTreeMap;

use arris_geom::{Curve, Curve2, GeomError, GeomKind, Profile, ProfileEdge, Surface, pcurve_on};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{
    Axis, Control, Frame, Interval, Isometry, Meter, Point2, Point3, Tolerance, UnitVec2, UnitVec3,
    Vec2, Vec3, wrap_angle,
};
use arris_topo::builder::{
    Assembly, AssemblySlots, Builder, Built, EdgeKey, EdgeSpec, FaceSpec, UseSpec, VertexKey,
    VertexSpec,
};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::provenance::SweepPart;
use arris_topo::{Body, EntityId, Model, Orientation, Provenance, Role, Shape};

use crate::error::{Fault, InputReason, OpError, Reason, SweepReason, fault_of};
use crate::verify;

fn degenerate(reason: Reason) -> OpError {
    OpError::Degenerate {
        entities: Vec::new(),
        reason,
    }
}

/// The revolve axis seen in the profile plane's own (u, v), once it has
/// been found to lie in that plane: a point of it, its direction, and the
/// unit in-plane direction from the axis towards the material — the side
/// of the axis the whole profile lies on.
#[derive(Debug, Clone, Copy)]
struct AxisInPlane {
    origin: Point2,
    along: Vec2,
    radial: Vec2,
}

impl AxisInPlane {
    /// The position along the axis of a (u, v) point.
    fn t(&self, p: Point2) -> f64 {
        (p - self.origin).dot(&self.along)
    }

    /// The signed distance of a (u, v) point from the axis, positive on
    /// the material side.
    fn rho(&self, p: Point2) -> f64 {
        (p - self.origin).dot(&self.radial)
    }
}

/// `true` when the angle `t` of a periodic parameter lies within `range`,
/// whose length is at most one turn.
fn turn_contains(range: Interval, t: f64) -> bool {
    wrap_angle(t - range.lo()) <= range.length()
}

/// The least and the greatest signed distance from the axis over one
/// edge: its ends for a line; its ends and, where the arc passes them,
/// the two points of its circle nearest to and farthest from the axis.
fn rho_range(edge: &ProfileEdge, axis: &AxisInPlane) -> (f64, f64) {
    let mut candidates = vec![edge.range.lo(), edge.range.hi()];
    if let Curve2::Circle { frame, .. } = &edge.pcurve {
        // `ρ(θ) = ρ(c) + r (a cos θ + b sin θ)`: extreme at `atan2(b, a)`
        // and half a turn on.
        let a = axis.radial.dot(&frame.x());
        let b = axis.radial.dot(&frame.y());
        let extreme = b.atan2(a);
        for t in [extreme, extreme + core::f64::consts::PI] {
            if turn_contains(edge.range, t) {
                candidates.push(edge.range.lo() + wrap_angle(t - edge.range.lo()));
            }
        }
    }
    candidates
        .into_iter()
        .map(|t| axis.rho(edge.pcurve.point(t)))
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), r| {
            (lo.min(r), hi.max(r))
        })
}

/// The axis validated against the profile's plane: in it within the
/// tolerances ([`SweepReason::AxisNotInProfilePlane`] otherwise), then
/// projected exactly into it, so every surface of revolution is placed
/// by an axis the profile's curves lie in a plane through. Returns the
/// projected axis and its (u, v) view with `radial` the *left* normal of
/// its direction; which side the material is on is decided by
/// [`orient`].
fn axis_in_plane(
    profile: &Profile,
    axis: Axis,
    tol: Tolerance,
) -> Result<(Axis, AxisInPlane), OpError> {
    if !axis.origin.coords.iter().all(|c| c.is_finite()) {
        return Err(degenerate(Reason::Input(InputReason::NonFinite {
            what: "axis origin",
        })));
    }
    let plane = &profile.plane;
    let n = plane.z().into_inner();
    let d = axis.direction.into_inner();
    let dn = d.dot(&n);
    // The angle between the direction and the plane, and the origin's
    // height above it.
    let off_plane = dn.abs().atan2((1.0 - dn * dn).max(0.0).sqrt());
    let height = (axis.origin - plane.origin()).dot(&n);
    if !(off_plane <= tol.angular && height.abs() <= tol.linear) {
        return Err(degenerate(Reason::Sweep(
            SweepReason::AxisNotInProfilePlane,
        )));
    }
    let origin = axis.origin - height * n;
    let direction = UnitVec3::try_new(d - dn * n, 0.0).ok_or(degenerate(Reason::Sweep(
        SweepReason::AxisNotInProfilePlane,
    )))?;
    let projected = Axis { origin, direction };
    let o = plane.to_local(origin);
    let a = plane.vec_to_local(direction.into_inner());
    let along = Vec2::new(a.x, a.y);
    let along = along / along.norm();
    Ok((
        projected,
        AxisInPlane {
            origin: Point2::new(o.x, o.y),
            along,
            radial: Vec2::new(-along.y, along.x),
        },
    ))
}

/// The first elliptic edge of a validated profile in the consumer's own
/// order — the lowest `(loop_index, segment)`, which the walk may have
/// reversed — or `None` when every edge is a line or a circle. The
/// surface such a segment would sweep has no variant, so a revolve
/// refuses it (ADR-0014); an ellipse whose radii agree within the linear
/// tolerance is already a circle edge here and is not one of these.
fn elliptic(loops: &[Vec<ProfileEdge>]) -> Option<&ProfileEdge> {
    loops
        .iter()
        .flatten()
        .filter(|e| matches!(e.curve, Curve::Ellipse { .. }))
        .min_by_key(|e| (e.loop_index, e.segment))
}

/// The profile held to one side of the axis: `radial` turned to face the
/// side every point of every edge lies on, reaching at worst within
/// `tol.linear` of the axis — a point that near is *on* it —
/// ([`SweepReason::ProfileCrossesAxis`] for points beyond the tolerance on
/// both sides, [`InputReason::ZeroThickness`] for a profile within it
/// everywhere), and no arc's circle crossing the axis off its centre
/// ([`SweepReason::SpindleTorus`]).
fn orient(
    loops: &[Vec<ProfileEdge>],
    mut axis: AxisInPlane,
    tol: Tolerance,
) -> Result<AxisInPlane, OpError> {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for edge in loops.iter().flatten() {
        let (a, b) = rho_range(edge, &axis);
        lo = lo.min(a);
        hi = hi.max(b);
    }
    if !(lo.is_finite() && hi.is_finite()) {
        return Err(degenerate(Reason::Input(InputReason::NonFinite {
            what: "profile",
        })));
    }
    if lo < -tol.linear && hi > tol.linear {
        return Err(degenerate(Reason::Sweep(SweepReason::ProfileCrossesAxis)));
    }
    if hi <= tol.linear {
        axis.radial = -axis.radial;
        hi = -lo;
    }
    if hi <= tol.linear {
        return Err(degenerate(Reason::Input(InputReason::ZeroThickness)));
    }
    for edge in loops.iter().flatten() {
        if let Curve2::Circle { frame, radius } = &edge.pcurve {
            let centre = axis.rho(frame.origin());
            if centre.abs() > tol.linear && centre - radius <= tol.linear {
                return Err(degenerate(Reason::Sweep(SweepReason::SpindleTorus)));
            }
        }
    }
    Ok(axis)
}

/// The surface one edge sweeps about the axis, in the one frame convention
/// of every surface of revolution here: origin on the axis, `X` the unit
/// radial from the axis into the profile's plane (so `u = 0` *is* the
/// profile plane and every seam lies in it), `Z` the axis direction —
/// except a cone whose radius shrinks along the axis, which takes
/// `Z = −axis`, since the data model's cone grows along `+Z`. A line
/// parallel to the axis sweeps a cylinder, perpendicular a plane (an
/// annulus, `annulus` set), oblique a cone with its apex on the axis; an
/// arc centred on the axis a sphere, elsewhere a torus of `R` its centre's
/// distance and `r` its radius.
fn swept_surface(
    edge: &ProfileEdge,
    axis: &AxisInPlane,
    base: &Frame,
    axis_point: impl Fn(f64) -> Point3,
    tol: Tolerance,
) -> Result<(Surface, bool), OpError> {
    match &edge.curve {
        Curve::Line { .. } => {
            let chord = edge.end - edge.start;
            let g = chord / chord.norm();
            let along = g.dot(&axis.along);
            let across = g.dot(&axis.radial);
            let (rho_s, rho_e) = (axis.rho(edge.start), axis.rho(edge.end));
            let (t_s, t_e) = (axis.t(edge.start), axis.t(edge.end));
            if across.abs() <= tol.angular {
                Ok((
                    Surface::Cylinder {
                        frame: *base,
                        radius: rho_s,
                    },
                    false,
                ))
            } else if along.abs() <= tol.angular {
                Ok((
                    Surface::Plane {
                        frame: base.with_origin(axis_point(t_s)),
                    },
                    true,
                ))
            } else {
                let (d_rho, d_t) = (rho_e - rho_s, t_e - t_s);
                let origin = axis_point(t_s);
                let frame = if d_rho * d_t > 0.0 {
                    base.with_origin(origin)
                } else {
                    Frame::from_orthonormal(
                        origin,
                        base.x().into_inner(),
                        -base.y().into_inner(),
                        -base.z().into_inner(),
                    )?
                };
                // A start on the axis is the apex itself.
                Ok((
                    Surface::Cone {
                        frame,
                        radius: if rho_s.abs() <= tol.linear {
                            0.0
                        } else {
                            rho_s
                        },
                        half_angle: d_rho.abs().atan2(d_t.abs()),
                    },
                    false,
                ))
            }
        }
        &Curve::Circle { radius, .. } => {
            // The centre in the profile's (u, v): the arc's pcurve is the
            // same circle there, so its frame's origin is exact.
            let Curve2::Circle {
                frame: in_plane, ..
            } = &edge.pcurve
            else {
                return Err(profile_curve_fault(edge));
            };
            let centre = in_plane.origin();
            let origin = axis_point(axis.t(centre));
            let major = axis.rho(centre);
            if major.abs() <= tol.linear {
                Ok((
                    Surface::Sphere {
                        frame: base.with_origin(origin),
                        radius,
                    },
                    false,
                ))
            } else {
                Ok((
                    Surface::Torus {
                        frame: base.with_origin(origin),
                        major_radius: major,
                        minor_radius: radius,
                    },
                    false,
                ))
            }
        }
        // `Profile::edges` makes lines, circles and ellipses, and
        // `elliptic` refused the ellipses before anything was made.
        Curve::Ellipse { .. } | Curve::Nurbs(_) => Err(profile_curve_fault(edge)),
    }
}

/// A maximal run of one loop's segments off the revolve axis — a loop that
/// never lies along the axis is one — which a full turn closes into one
/// shell.
struct Chain {
    /// The loop, as the consumer wrote it.
    loop_index: usize,
    /// The lowest segment index the consumer wrote among its segments.
    segment: usize,
    /// Where along the axis its first vertex and its last lie; `None` for
    /// a loop that never lies along the axis.
    span: Option<(f64, f64)>,
}

/// The pcurve of the degenerate edge where a side of revolution closes at
/// the axis — a cone's apex, a sphere's pole: the line at the singular `v`
/// over the rise's range, its `u` running as a rise there would, with the
/// axis along the surface's `Z` and against it otherwise.
fn apex_pcurve(surface: &Surface, axis: &Axis, v: f64) -> Curve2 {
    let along = match surface {
        Surface::Plane { frame }
        | Surface::Cylinder { frame, .. }
        | Surface::EllipticCylinder { frame, .. }
        | Surface::Cone { frame, .. }
        | Surface::Sphere { frame, .. }
        | Surface::Torus { frame, .. } => frame.z().dot(&axis.direction),
        Surface::Nurbs(_) => 1.0,
    };
    Curve2::Line {
        origin: Point2::new(0.0, v),
        direction: UnitVec2::new_unchecked(Vec2::new(if along < 0.0 { -1.0 } else { 1.0 }, 0.0)),
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
/// the segment's midpoint: outward is the segment's in-plane normal on
/// the right of the walk (the material is on the loop's left), and the
/// face is `Forward` when the surface normal there agrees with it.
fn side_orientation(
    edge: &ProfileEdge,
    normal: Vec3,
    surface: &Surface,
    pcurve: &Curve2,
) -> Result<Orientation, OpError> {
    let t = edge.range.midpoint();
    let tangent = edge.curve.eval(t).d1;
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

/// A flat end of a sweep on the plane `frame`: every loop of the profile
/// walked as oriented, each use `Forward`, and turned round when the face
/// is used `Reversed`, so the material stays on the walk's left seen from
/// outside. A rigid motion carries the parametrisation, so an end moved
/// off the profile's plane has the profile's own pcurves. `slots` holds
/// each edge's slot, `None` for an edge the end does not hold.
fn cap_face(
    m: &mut Model,
    loops: &[Vec<ProfileEdge>],
    frame: Frame,
    orientation: Orientation,
    slots: &[Vec<Option<usize>>],
    tolerance: f64,
) -> FaceSpec {
    let surface = m.add_surface(Surface::Plane { frame });
    let mut cap_loops = Vec::with_capacity(loops.len());
    for (li, edges_of) in loops.iter().enumerate() {
        let mut uses: Vec<UseSpec> = edges_of
            .iter()
            .enumerate()
            .filter_map(|(j, edge)| {
                Some(UseSpec {
                    edge: EdgeKey::New(slots[li][j]?),
                    orientation: Orientation::Forward,
                    pcurve: m.add_curve2(edge.pcurve.clone()),
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

/// Revolves `profile` about `axis` by `angle` into a solid: a partial
/// turn with two flat ends, or a full turn with seams when `angle` is
/// within `angular_tolerance` of `2π`. Every segment of the profile
/// sweeps one face — a segment parallel to the axis a cylinder,
/// perpendicular a plane (an annulus, or a sector of one), oblique a cone;
/// an arc centred on the axis a sphere, elsewhere a torus — every vertex a
/// circular *rise* about the axis, and every pcurve is exact through
/// `pcurve_on`. A vertex within `default_tolerance` of the axis is *on* it
/// and sweeps no rise — one vertex, shared by both flat ends of a partial
/// turn and not made in a full turn — and a line segment with both ends
/// on it lies *along* it and sweeps no face: in a partial turn it is the
/// one edge both flat ends share, in a full turn nothing. A face closing
/// at a vertex on the axis — a cone's apex, a sphere's pole — holds a
/// degenerate edge there in place of the rise, its pcurve the line at the
/// singular `v`, one per face and each `Generated` from the vertex's
/// `Rise`; a full turn keeps the vertex for it. The surfaces of revolution share one frame: origin on the
/// axis, `X` the radial into the profile's plane, so `u = 0` is the
/// profile plane and every seam lies in it, `Z` the axis (`−axis` for a
/// cone narrowing along it). The flat ends of a partial turn are the
/// profile face, its outward normal against the turn, and its copy
/// rotated by `angle`. Entities are appended in one fixed order —
/// vertices per loop in walking order (the start ring, then the end
/// ring), edges (start, end, rises), faces (start cap, end cap, sides per
/// loop per segment) — so the ids are a function of the profile alone;
/// every tolerance is `default_tolerance`. A full turn closes the profile
/// into one lump (ADR-0006) of a shell per *chain* — a maximal run of a
/// loop's segments off the axis, a loop that never lies along it being
/// one: the chain whose ends span every other's along the axis is the
/// outer shell, stored first, and every other — a hole, a notch cut in
/// from the axis — a void of it, stored by loop and lowest segment; a
/// partial turn's flat ends join everything into one shell. Provenance is
/// one `Generated` per entity from a [`Role::Revolve`] naming the part of
/// the sketch it came from: a segment perpendicular to the axis in a full
/// turn sweeps an annulus of two closed rises and has no `StartEdge`, and
/// a void is [`SweepPart::Cavity`], named by its loop and the lowest
/// segment index the consumer wrote in its chain.
///
/// Errors, the model untouched: [`OpError::Profile`] when
/// `Profile::edges` refuses the sketch; [`OpError::Degenerate`] with
/// [`InputReason::NonFinite`] for a non-finite angle, axis origin or profile,
/// [`InputReason::NotPositive`] for an angle at or below zero,
/// [`SweepReason::AngleAboveTurn`] above `2π`, [`SweepReason::AxisNotInProfilePlane`]
/// when the axis is off the plane by more than the tolerances,
/// [`SweepReason::ProfileCrossesAxis`] when the profile has points on both
/// sides of the axis, [`InputReason::ZeroThickness`] when it lies within
/// `default_tolerance` of the axis everywhere,
/// [`InputReason::NonManifold`] when a full turn's profile touches the axis at
/// a vertex with no segment along it, where the swept surface would touch
/// itself, [`SweepReason::SpindleTorus`] when an arc's circle crosses it off
/// its centre, and [`SweepReason::EllipticRevolve`] naming the first elliptic
/// segment of the sketch, whose swept surface has no variant (ADR-0014).
///
/// ```
/// use arris_ops::revolve;
/// use arris_topo::Model;
/// use arris_geom::{Profile, ProfileLoop, ProfileSegment};
/// use arris_math::{Axis, Frame, Point2, Point3, Vec3};
/// use arris_topo::provenance::{Role, SweepPart};
/// use core::f64::consts::TAU;
///
/// // A rectangle x ∈ [1, 2], z ∈ [−1, 1] in the xz plane, revolved about z: a tube.
/// let plane = Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
/// let p = |u, v| Point2::new(u, v);
/// let profile = Profile {
///     plane,
///     outer: ProfileLoop::Path {
///         start: p(1.0, -1.0),
///         segments: vec![
///             ProfileSegment::LineTo(p(2.0, -1.0)),
///             ProfileSegment::LineTo(p(2.0, 1.0)),
///             ProfileSegment::LineTo(p(1.0, 1.0)),
///             ProfileSegment::LineTo(p(1.0, -1.0)),
///         ],
///     },
///     holes: Vec::new(),
/// };
/// let mut m = Model::default();
/// let (body, provenance) = revolve(&mut m, &profile, Axis::z_at(Point3::origin()), TAU, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(body).unwrap().len(), 4, "two annuli and two walls");
/// assert_eq!(m.edges(body).unwrap().len(), 6, "four rises and two seams");
/// let wall = Role::Revolve(SweepPart::Side { loop_index: 0, segment: 1 });
/// assert_eq!(provenance.generated_from(wall).len(), 1);
/// ```
pub fn revolve(
    m: &mut Model,
    profile: &Profile,
    axis: Axis,
    angle: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    let mut meter = Meter::new(control);
    let meter = &mut meter;
    let precision = m.precision();
    let tol = precision.tolerance();
    if !angle.is_finite() {
        return Err(degenerate(Reason::Input(InputReason::NonFinite {
            what: "angle",
        })));
    }
    if angle <= 0.0 {
        return Err(degenerate(Reason::Input(InputReason::NotPositive {
            what: "angle",
            value: angle,
        })));
    }
    if angle > TAU + tol.angular {
        return Err(degenerate(Reason::Sweep(SweepReason::AngleAboveTurn)));
    }
    let full = (angle - TAU).abs() <= tol.angular;
    let angle = if full { TAU } else { angle };
    let (axis, in_plane) = axis_in_plane(profile, axis, tol)?;
    let loops = profile.edges(tol)?;
    if let Some(edge) = elliptic(&loops) {
        return Err(degenerate(Reason::Sweep(SweepReason::EllipticRevolve {
            loop_index: edge.loop_index,
            segment: edge.segment,
        })));
    }
    let in_plane = orient(&loops, in_plane, tol)?;

    let plane = &profile.plane;
    let normal = plane.z().into_inner();
    let radial = plane.vec_to_world(Vec3::new(in_plane.radial.x, in_plane.radial.y, 0.0));
    let base = Frame::new(axis.origin, axis.direction.into_inner(), radial)?;
    let axis_point = |t: f64| axis.at(t);
    // The material sweeps along `Z × X` at the start; whether that is the
    // profile's normal or its opposite decides the caps' and the sides'
    // walks below.
    let turn_along_normal = base.y().dot(&normal) > 0.0;
    let rotation = UnitQuaternion::from_axis_angle(&axis.direction, angle);
    let about_axis = Isometry::new(
        rotation,
        axis.origin.coords - Isometry::from_rotation(rotation).apply_vec(axis.origin.coords),
    );

    // Which vertices lie on the axis — walk index `j` the vertex edge `j`
    // starts at — and which line segments lie along it, both ends on it.
    let on_axis: Vec<Vec<bool>> = loops
        .iter()
        .map(|edges| {
            edges
                .iter()
                .map(|e| in_plane.rho(e.start).abs() <= tol.linear)
                .collect()
        })
        .collect();
    let along: Vec<Vec<bool>> = loops
        .iter()
        .zip(&on_axis)
        .map(|(edges, on)| {
            let n = edges.len();
            edges
                .iter()
                .enumerate()
                .map(|(j, e)| matches!(e.curve, Curve::Line { .. }) && on[j] && on[(j + 1) % n])
                .collect()
        })
        .collect();

    // Every segment's surface, before anything is written; none for a
    // segment along the axis, which sweeps nothing.
    let mut surfaces: Vec<Vec<Option<(Surface, bool)>>> = Vec::with_capacity(loops.len());
    for (li, edges) in loops.iter().enumerate() {
        let mut row = Vec::with_capacity(edges.len());
        for (j, edge) in edges.iter().enumerate() {
            row.push(if along[li][j] {
                None
            } else {
                Some(swept_surface(edge, &in_plane, &base, axis_point, tol)?)
            });
        }
        surfaces.push(row);
    }
    // Which faces close at a vertex on the axis — a cone's apex, a
    // sphere's pole — each holding a degenerate edge there in place of the
    // rise: the face ending at vertex `v`, then the one starting there.
    let singular = |li: usize, j: usize| {
        matches!(
            surfaces[li][j],
            Some((Surface::Cone { .. } | Surface::Sphere { .. }, _))
        )
    };
    let closes_at = |li: usize, v: usize| -> [bool; 2] {
        let n = loops[li].len();
        let on = on_axis[li][v];
        [on && singular(li, (v + n - 1) % n), on && singular(li, v)]
    };
    // A full turn touching the axis at a vertex with no segment along it
    // sweeps a surface touching itself there — two chains, or one and
    // itself, meeting at a point. A partial turn's flat ends close the
    // vertex's link, so it builds.
    if full
        && loops.iter().enumerate().any(|(li, edges)| {
            let n = edges.len();
            (0..n).any(|j| on_axis[li][j] && !along[li][j] && !along[li][(j + n - 1) % n])
        })
    {
        return Err(degenerate(Reason::Input(InputReason::NonManifold)));
    }
    // A full turn's shells: each loop's chains, `chain[li][j]` the chain of
    // segment `j` and `None` along the axis, walked from just past the
    // loop's first segment along the axis.
    let mut chain: Vec<Vec<Option<usize>>> = Vec::with_capacity(loops.len());
    let mut chains: Vec<Chain> = Vec::new();
    for (li, edges) in loops.iter().enumerate() {
        let n = edges.len();
        let first_along = (0..n).find(|&j| along[li][j]);
        let from = first_along.map_or(0, |j| j + 1);
        let mut row = vec![None; n];
        let mut current: Option<usize> = None;
        for (k, edge) in edges.iter().cycle().skip(from).take(n).enumerate() {
            let j = (from + k) % n;
            if along[li][j] {
                current = None;
                continue;
            }
            if current.is_none() {
                chains.push(Chain {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                    span: first_along.map(|_| (in_plane.t(edge.start), in_plane.t(edge.start))),
                });
                current = Some(chains.len() - 1);
            }
            row[j] = current;
            if let Some(c) = chains.last_mut() {
                c.segment = c.segment.min(edge.segment);
                if let Some(span) = &mut c.span {
                    span.1 = in_plane.t(edge.end);
                }
            }
        }
        chain.push(row);
    }
    // The chain whose ends span every other's along the axis is the lump's
    // outer shell — a loop that never lies along the axis spans everything
    // — and the voids follow by loop and lowest segment.
    let width = |c: &Chain| c.span.map_or(f64::INFINITY, |(a, b)| (b - a).abs());
    let Some(outer) = (0..chains.len())
        .filter(|&c| chains[c].loop_index == 0)
        .max_by(|&a, &b| width(&chains[a]).total_cmp(&width(&chains[b])))
    else {
        return Err(degenerate(Reason::Input(InputReason::ZeroThickness)));
    };
    let mut shell_order: Vec<usize> = (0..chains.len()).filter(|&c| c != outer).collect();
    shell_order.sort_by_key(|&c| (chains[c].loop_index, chains[c].segment));
    shell_order.insert(0, outer);

    let tolerance = precision.default_tolerance;
    let rise_range = if full {
        Interval::TURN
    } else {
        Interval::new(0.0, angle).map_err(|_| {
            degenerate(Reason::Input(InputReason::NotPositive {
                what: "angle",
                value: angle,
            }))
        })?
    };
    let part = |p: SweepPart| Role::Revolve(p);

    m.transaction(|m| {
        let mut vertices: Vec<VertexSpec> = Vec::new();
        let mut vertex_roles: Vec<Role> = Vec::new();
        let mut edges: Vec<EdgeSpec> = Vec::new();
        let mut edge_roles: Vec<Role> = Vec::new();
        let mut faces: Vec<FaceSpec> = Vec::new();
        let mut face_roles: Vec<Role> = Vec::new();

        // The start ring, then the end ring; slot (loop, walk index). A
        // vertex on the axis is one vertex, with no end copy, and a full
        // turn makes it only where a face closes there on a degenerate
        // edge, since no other face keeps it.
        let mut start_vertex: Vec<Vec<Option<usize>>> = Vec::with_capacity(loops.len());
        let mut end_vertex: Vec<Vec<Option<usize>>> = Vec::with_capacity(loops.len());
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut ring = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                if full && on_axis[li][j] && closes_at(li, j) == [false, false] {
                    ring.push(None);
                    continue;
                }
                ring.push(Some(vertices.len()));
                vertices.push(VertexSpec::New {
                    point: edge.curve.point(edge.range.lo()),
                    tolerance,
                });
                vertex_roles.push(part(SweepPart::StartVertex {
                    loop_index: edge.loop_index,
                    vertex: vertex_index(edge, n),
                }));
            }
            start_vertex.push(ring);
        }
        if full {
            end_vertex.clone_from(&start_vertex);
        } else {
            for (li, edges_of) in loops.iter().enumerate() {
                let n = edges_of.len();
                let mut ring = Vec::with_capacity(n);
                for (j, edge) in edges_of.iter().enumerate() {
                    meter.tick()?;
                    if on_axis[li][j] {
                        ring.push(start_vertex[li][j]);
                        continue;
                    }
                    ring.push(Some(vertices.len()));
                    vertices.push(VertexSpec::New {
                        point: about_axis.apply(edge.curve.point(edge.range.lo())),
                        tolerance,
                    });
                    vertex_roles.push(part(SweepPart::EndVertex {
                        loop_index: edge.loop_index,
                        vertex: vertex_index(edge, n),
                    }));
                }
                end_vertex.push(ring);
            }
        }
        let key = |slot: Option<usize>, edge: &ProfileEdge| {
            slot.map(VertexKey::New).ok_or_else(|| unmade(edge))
        };

        // The start edges, the end edges, then the rises.
        let mut start_edge: Vec<Vec<Option<usize>>> = Vec::with_capacity(loops.len());
        let mut end_edge: Vec<Vec<Option<usize>>> = Vec::with_capacity(loops.len());
        let mut rise_edge: Vec<Vec<Option<usize>>> = Vec::with_capacity(loops.len());
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut row = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                // A full turn's annulus keeps only its rises, and a segment
                // along the axis is nothing there; in a partial turn that
                // segment is the one edge both flat ends share.
                let annulus = surfaces[li][j].as_ref().is_some_and(|s| s.1);
                if full && (annulus || along[li][j]) {
                    row.push(None);
                    continue;
                }
                let start = key(start_vertex[li][j], edge)?;
                let end = key(start_vertex[li][(j + 1) % n], edge)?;
                row.push(Some(edges.len()));
                edges.push(EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(edge.curve.clone()),
                        range: edge.range,
                    },
                    start,
                    end,
                    tolerance,
                });
                edge_roles.push(part(SweepPart::StartEdge {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }));
            }
            start_edge.push(row);
        }
        let mut end_curves: Vec<Vec<Option<Curve>>> = Vec::with_capacity(loops.len());
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut row = Vec::with_capacity(n);
            let mut curves = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                if full || along[li][j] {
                    row.push(None);
                    curves.push(None);
                    continue;
                }
                let start = key(end_vertex[li][j], edge)?;
                let end = key(end_vertex[li][(j + 1) % n], edge)?;
                let curve = edge.curve.transformed(&about_axis);
                row.push(Some(edges.len()));
                edges.push(EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(curve.clone()),
                        range: edge.range,
                    },
                    start,
                    end,
                    tolerance,
                });
                edge_roles.push(part(SweepPart::EndEdge {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }));
                curves.push(Some(curve));
            }
            end_edge.push(row);
            end_curves.push(curves);
        }
        let mut rises: Vec<Vec<Option<Curve>>> = Vec::with_capacity(loops.len());
        // The degenerate edges, by (loop, side, whether at the side's start).
        let mut apex: BTreeMap<(usize, usize, bool), usize> = BTreeMap::new();
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut row = Vec::with_capacity(n);
            let mut curves = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                // A vertex on the axis sweeps no rise; each face closing
                // there holds a degenerate edge at it instead.
                if on_axis[li][j] {
                    let before = (j + n - 1) % n;
                    for (side, closes) in [before, j].into_iter().zip(closes_at(li, j)) {
                        if !closes {
                            continue;
                        }
                        let at = key(start_vertex[li][j], edge)?;
                        apex.insert((li, side, side == j), edges.len());
                        edges.push(EdgeSpec::New {
                            geometry: EdgeGeometry::Degenerate { range: rise_range },
                            start: at,
                            end: at,
                            tolerance,
                        });
                        edge_roles.push(part(SweepPart::Rise {
                            loop_index: edge.loop_index,
                            vertex: vertex_index(edge, n),
                        }));
                    }
                    row.push(None);
                    curves.push(None);
                    continue;
                }
                let start = key(start_vertex[li][j], edge)?;
                let end = key(end_vertex[li][j], edge)?;
                let rise = Curve::Circle {
                    frame: base.with_origin(axis_point(in_plane.t(edge.start))),
                    radius: in_plane.rho(edge.start),
                };
                row.push(Some(edges.len()));
                edges.push(EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(rise.clone()),
                        range: rise_range,
                    },
                    start,
                    end,
                    tolerance,
                });
                edge_roles.push(part(SweepPart::Rise {
                    loop_index: edge.loop_index,
                    vertex: vertex_index(edge, n),
                }));
                curves.push(Some(rise));
            }
            rise_edge.push(row);
            rises.push(curves);
        }

        // The two flat ends of a partial turn: the profile face, its
        // outward normal against the turn, and its rotated copy. A rigid
        // motion carries the parametrisation, so the rotated edges have
        // the profile's own pcurves on the rotated plane.
        if !full {
            let (start_use, end_use) = if turn_along_normal {
                (Orientation::Reversed, Orientation::Forward)
            } else {
                (Orientation::Forward, Orientation::Reversed)
            };
            faces.push(cap_face(
                m,
                &loops,
                *plane,
                start_use,
                &start_edge,
                tolerance,
            ));
            face_roles.push(part(SweepPart::StartCap));
            // A segment along the axis is the start cap's edge in the end
            // cap too.
            let end_slots: Vec<Vec<Option<usize>>> = end_edge
                .iter()
                .zip(&start_edge)
                .zip(&along)
                .map(|((ends, starts), on)| {
                    ends.iter()
                        .zip(starts)
                        .zip(on)
                        .map(|((&e, &s), &a)| if a { s } else { e })
                        .collect()
                })
                .collect();
            faces.push(cap_face(
                m,
                &loops,
                plane.transformed(&about_axis),
                end_use,
                &end_slots,
                tolerance,
            ));
            face_roles.push(part(SweepPart::EndCap));
        }

        // The chain of every side, in `faces` order after the caps: a full
        // turn's shells.
        let mut side_chain: Vec<(Option<usize>, &ProfileEdge)> = Vec::new();
        // The sides: start edge, rise up, end edge back, rise down — the
        // end edge the start edge's second use across the seam in a full
        // turn — walked that way when the material sweeps along the
        // profile's normal and the other way otherwise, so the material
        // is on the walk's left seen from outside; an annulus of a full
        // turn keeps only its closed rises, one loop each. A segment along
        // the axis sweeps no side, and a vertex on the axis has no rise to
        // walk, so a side reaching the axis closes there.
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                let Some((surface, annulus)) = &surfaces[li][j] else {
                    continue;
                };
                let next = (j + 1) % n;
                let mut on = |curve: &Curve, range: Interval| -> Result<Curve2, OpError> {
                    pcurve_on(curve, range, surface, tol, meter).map_err(fault_of)
                };
                let start_pcurve = on(&edge.curve, edge.range)?;
                let orientation = side_orientation(edge, normal, surface, &start_pcurve)?;
                // Where the side's own start edge meets the axis, if it
                // does: the singular `v` of a degenerate edge there.
                let (v_start, v_end) = (
                    start_pcurve.point(edge.range.lo()).y,
                    start_pcurve.point(edge.range.hi()).y,
                );
                let mut cycle: Vec<SideUse> = Vec::with_capacity(4);
                if let Some(slot) = start_edge[li][j] {
                    cycle.push(SideUse {
                        edge: slot,
                        orientation: Orientation::Forward,
                        pcurve: start_pcurve.clone(),
                        range: edge.range,
                    });
                }
                if let (Some(slot), Some(rise)) = (rise_edge[li][next], &rises[li][next]) {
                    cycle.push(SideUse {
                        edge: slot,
                        orientation: Orientation::Forward,
                        pcurve: on(rise, rise_range)?,
                        range: rise_range,
                    });
                } else if let Some(&slot) = apex.get(&(li, j, false)) {
                    cycle.push(SideUse {
                        edge: slot,
                        orientation: Orientation::Forward,
                        pcurve: apex_pcurve(surface, &axis, v_end),
                        range: rise_range,
                    });
                }
                match (end_edge[li][j], &end_curves[li][j], start_edge[li][j]) {
                    (Some(slot), Some(curve), _) => cycle.push(SideUse {
                        edge: slot,
                        orientation: Orientation::Reversed,
                        pcurve: on(curve, edge.range)?,
                        range: edge.range,
                    }),
                    (None, _, Some(seam)) => cycle.push(SideUse {
                        edge: seam,
                        orientation: Orientation::Reversed,
                        pcurve: start_pcurve,
                        range: edge.range,
                    }),
                    (None, _, None) => {}
                    (Some(_), None, _) => {
                        return Err(OpError::Internal(Fault::Invariant {
                            what: "an end edge's curve",
                        }));
                    }
                }
                if let (Some(slot), Some(rise)) = (rise_edge[li][j], &rises[li][j]) {
                    cycle.push(SideUse {
                        edge: slot,
                        orientation: Orientation::Reversed,
                        pcurve: on(rise, rise_range)?,
                        range: rise_range,
                    });
                } else if let Some(&slot) = apex.get(&(li, j, true)) {
                    cycle.push(SideUse {
                        edge: slot,
                        orientation: Orientation::Reversed,
                        pcurve: apex_pcurve(surface, &axis, v_start),
                        range: rise_range,
                    });
                }
                if !turn_along_normal {
                    cycle.reverse();
                    for u in &mut cycle {
                        u.orientation = u.orientation.flipped();
                    }
                }
                faces.push(side_face(
                    m,
                    surface,
                    orientation,
                    cycle,
                    full && *annulus,
                    tolerance,
                ));
                face_roles.push(part(SweepPart::Side {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }));
                side_chain.push((chain[li][j], edge));
            }
        }

        // A full turn has no caps, so the sides close across the axis on
        // their own, a shell per chain: the spanning chain's the lump's
        // outer shell, stored first, every other a void of it by loop and
        // lowest segment, each shell's faces in the order they were made. A
        // partial turn's caps join everything into one shell.
        let (shells, shell_roles, face_roles) = if full {
            let mut buckets: Vec<Vec<(FaceSpec, Role)>> =
                chains.iter().map(|_| Vec::new()).collect();
            for ((face, role), (c, edge)) in faces.into_iter().zip(face_roles).zip(side_chain) {
                let Some(bucket) = c.and_then(|c| buckets.get_mut(c)) else {
                    return Err(unmade(edge));
                };
                bucket.push((face, role));
            }
            let mut shells = Vec::with_capacity(shell_order.len());
            let mut shell_roles = Vec::with_capacity(shell_order.len());
            let mut face_roles = Vec::new();
            for &c in &shell_order {
                let (specs, roles): (Vec<FaceSpec>, Vec<Role>) =
                    core::mem::take(&mut buckets[c]).into_iter().unzip();
                shells.push(specs);
                face_roles.extend(roles);
                shell_roles.push(if c == outer {
                    part(SweepPart::Shell)
                } else {
                    part(SweepPart::Cavity {
                        loop_index: chains[c].loop_index,
                        segment: chains[c].segment,
                    })
                });
            }
            (shells, shell_roles, face_roles)
        } else {
            (vec![faces], vec![part(SweepPart::Shell)], face_roles)
        };
        let assembly = Assembly {
            vertices,
            edges,
            shells,
        };
        let (b, slots) = Builder::assemble(m, tolerance, assembly)?;
        let built = b.finish(m, BodyKind::Solid)?;
        verify(m, built.body)?;
        let provenance = record(
            &built,
            &slots,
            &vertex_roles,
            &edge_roles,
            &face_roles,
            &shell_roles,
            part,
        );
        Ok((built.body, provenance))
    })
}

/// Extrudes `profile` along its plane's normal by `length` into a solid:
/// `direction` says which way, and must be the normal or its opposite
/// within `angular_tolerance`. Every segment of the profile sweeps one side
/// face — a line a plane whose `X` is the segment and `Y` the sweep, an arc
/// a cylinder whose frame is the arc's centre, `Z` the sweep and `X` the
/// profile plane's, so a circle loop's seam stands at its vertex's rise,
/// an elliptic arc an elliptic cylinder whose `X` is the arc's major
/// axis, so an ellipse loop's seam stands at its vertex's rise too
/// (ADR-0014) —
/// and every vertex a straight *rise* along the sweep; every pcurve is
/// exact through `pcurve_on`. The profile face keeps its plane's frame
/// whichever way the sweep goes and is the cap whose outward normal opposes
/// it; the other cap is its copy translated by the sweep. Entities are
/// appended in one fixed order — vertices per loop in walking order (the
/// start ring, then the end ring), edges (start, end, rises), faces (start
/// cap, end cap, sides per loop per segment) — so the ids are a function of
/// the profile alone; every tolerance is `default_tolerance`. Provenance is
/// one `Generated` per entity from a [`Role::Extrude`] naming the part of
/// the sketch it came from.
///
/// Errors, the model untouched: [`OpError::Profile`] when
/// `Profile::edges` refuses the sketch; [`OpError::Degenerate`] with
/// [`InputReason::NonFinite`] for a non-finite length or direction,
/// [`InputReason::NotPositive`] for a length at or below zero or a zero
/// direction, [`InputReason::ZeroThickness`] for a length within
/// `default_tolerance` of zero, and [`SweepReason::DirectionNotNormal`] for a
/// direction off the plane's normal — an oblique extrusion of an arc is a
/// cylinder of elliptical section, which no sweep builds yet (ADR-0047).
///
/// ```
/// use arris_ops::extrude;
/// use arris_topo::Model;
/// use arris_geom::{Profile, ProfileLoop, ProfileSegment};
/// use arris_math::{Frame, Point2, Vec3};
/// use arris_topo::provenance::{Role, SweepPart};
///
/// // A plate 40×30 with a hole of radius 4, extruded 10 up the z axis.
/// let p = |u, v| Point2::new(u, v);
/// let profile = Profile {
///     plane: Frame::world(),
///     outer: ProfileLoop::Path {
///         start: p(0.0, 0.0),
///         segments: vec![
///             ProfileSegment::LineTo(p(40.0, 0.0)),
///             ProfileSegment::LineTo(p(40.0, 30.0)),
///             ProfileSegment::LineTo(p(0.0, 30.0)),
///             ProfileSegment::LineTo(p(0.0, 0.0)),
///         ],
///     },
///     holes: vec![ProfileLoop::Circle { center: p(20.0, 15.0), radius: 4.0 }],
/// };
/// let mut m = Model::default();
/// let (body, provenance) = extrude(&mut m, &profile, Vec3::z(), 10.0, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(body).unwrap().len(), 7, "two caps, four walls and the bore");
/// assert_eq!(m.edges(body).unwrap().len(), 15, "the bore's seam once");
/// let bore = Role::Extrude(SweepPart::Side { loop_index: 1, segment: 0 });
/// assert_eq!(provenance.generated_from(bore).len(), 1);
/// ```
pub fn extrude(
    m: &mut Model,
    profile: &Profile,
    direction: Vec3,
    length: f64,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    let mut meter = Meter::new(control);
    let meter = &mut meter;
    let precision = m.precision();
    let tol = precision.tolerance();
    if !length.is_finite() {
        return Err(degenerate(Reason::Input(InputReason::NonFinite {
            what: "length",
        })));
    }
    if length <= 0.0 {
        return Err(degenerate(Reason::Input(InputReason::NotPositive {
            what: "length",
            value: length,
        })));
    }
    if length <= tol.linear {
        return Err(degenerate(Reason::Input(InputReason::ZeroThickness)));
    }
    if !direction.iter().all(|c| c.is_finite()) {
        return Err(degenerate(Reason::Input(InputReason::NonFinite {
            what: "direction",
        })));
    }
    let Some(d) = UnitVec3::try_new(direction, 0.0) else {
        return Err(degenerate(Reason::Input(InputReason::NotPositive {
            what: "direction's length",
            value: direction.norm(),
        })));
    };
    let plane = &profile.plane;
    let normal = plane.z().into_inner();
    let dn = d.dot(&normal);
    let off_normal = d.cross(&normal).norm().atan2(dn.abs());
    if off_normal > tol.angular {
        return Err(degenerate(Reason::Sweep(SweepReason::DirectionNotNormal)));
    }
    let loops = profile.edges(tol)?;
    // The sweep is the exact normal, never the caller's rounding of it.
    let along_normal = dn > 0.0;
    let sweep = if along_normal { plane.z() } else { -plane.z() };
    let shift = Isometry::from_translation(sweep.into_inner() * length);

    // Every segment's surface, before anything is written.
    let mut surfaces: Vec<Vec<Surface>> = Vec::with_capacity(loops.len());
    for edges in &loops {
        let mut row = Vec::with_capacity(edges.len());
        for edge in edges {
            row.push(match &edge.curve {
                Curve::Line { direction, .. } => Surface::Plane {
                    frame: Frame::new(
                        edge.curve.point(edge.range.lo()),
                        direction.cross(&sweep),
                        direction.into_inner(),
                    )?,
                },
                &Curve::Circle { ref frame, radius } => Surface::Cylinder {
                    frame: Frame::new(frame.origin(), sweep.into_inner(), plane.x().into_inner())?,
                    radius,
                },
                // An elliptic cylinder placed as a cylinder is, but with
                // `X` the edge's own major axis, so the cap ellipse is
                // the section at constant `v` (ADR-0014).
                &Curve::Ellipse {
                    ref frame,
                    major_radius,
                    minor_radius,
                } => Surface::EllipticCylinder {
                    frame: Frame::new(frame.origin(), sweep.into_inner(), frame.x().into_inner())?,
                    major_radius,
                    minor_radius,
                },
                // `Profile::edges` makes lines, circles and ellipses and
                // nothing else.
                Curve::Nurbs(_) => return Err(profile_curve_fault(edge)),
            });
        }
        surfaces.push(row);
    }

    let tolerance = precision.default_tolerance;
    let rise_range = Interval::new(0.0, length).map_err(|_| {
        degenerate(Reason::Input(InputReason::NotPositive {
            what: "length",
            value: length,
        }))
    })?;
    let part = |p: SweepPart| Role::Extrude(p);

    m.transaction(|m| {
        let mut vertices: Vec<VertexSpec> = Vec::new();
        let mut vertex_roles: Vec<Role> = Vec::new();
        let mut edges: Vec<EdgeSpec> = Vec::new();
        let mut edge_roles: Vec<Role> = Vec::new();
        let mut faces: Vec<FaceSpec> = Vec::new();
        let mut face_roles: Vec<Role> = Vec::new();

        // The start ring, then the end ring; slot (loop, walk index).
        let mut start_vertex: Vec<Vec<usize>> = Vec::with_capacity(loops.len());
        let mut end_vertex: Vec<Vec<usize>> = Vec::with_capacity(loops.len());
        for edges_of in &loops {
            let n = edges_of.len();
            let mut ring = Vec::with_capacity(n);
            for edge in edges_of {
                ring.push(vertices.len());
                vertices.push(VertexSpec::New {
                    point: edge.curve.point(edge.range.lo()),
                    tolerance,
                });
                vertex_roles.push(part(SweepPart::StartVertex {
                    loop_index: edge.loop_index,
                    vertex: vertex_index(edge, n),
                }));
            }
            start_vertex.push(ring);
        }
        for edges_of in &loops {
            let n = edges_of.len();
            let mut ring = Vec::with_capacity(n);
            for edge in edges_of {
                ring.push(vertices.len());
                vertices.push(VertexSpec::New {
                    point: shift.apply(edge.curve.point(edge.range.lo())),
                    tolerance,
                });
                vertex_roles.push(part(SweepPart::EndVertex {
                    loop_index: edge.loop_index,
                    vertex: vertex_index(edge, n),
                }));
            }
            end_vertex.push(ring);
        }

        // The start edges, the end edges, then the rises.
        let mut start_edge: Vec<Vec<usize>> = Vec::with_capacity(loops.len());
        let mut end_edge: Vec<Vec<usize>> = Vec::with_capacity(loops.len());
        let mut end_curves: Vec<Vec<Curve>> = Vec::with_capacity(loops.len());
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut row = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                row.push(edges.len());
                edges.push(EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(edge.curve.clone()),
                        range: edge.range,
                    },
                    start: VertexKey::New(start_vertex[li][j]),
                    end: VertexKey::New(start_vertex[li][(j + 1) % n]),
                    tolerance,
                });
                edge_roles.push(part(SweepPart::StartEdge {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }));
            }
            start_edge.push(row);
        }
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut row = Vec::with_capacity(n);
            let mut curves = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                let curve = edge.curve.transformed(&shift);
                row.push(edges.len());
                edges.push(EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(curve.clone()),
                        range: edge.range,
                    },
                    start: VertexKey::New(end_vertex[li][j]),
                    end: VertexKey::New(end_vertex[li][(j + 1) % n]),
                    tolerance,
                });
                edge_roles.push(part(SweepPart::EndEdge {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }));
                curves.push(curve);
            }
            end_edge.push(row);
            end_curves.push(curves);
        }
        let mut rise_edge: Vec<Vec<usize>> = Vec::with_capacity(loops.len());
        let mut rises: Vec<Vec<Curve>> = Vec::with_capacity(loops.len());
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut row = Vec::with_capacity(n);
            let mut curves = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                let rise = Curve::Line {
                    origin: edge.curve.point(edge.range.lo()),
                    direction: sweep,
                };
                row.push(edges.len());
                edges.push(EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(rise.clone()),
                        range: rise_range,
                    },
                    start: VertexKey::New(start_vertex[li][j]),
                    end: VertexKey::New(end_vertex[li][j]),
                    tolerance,
                });
                edge_roles.push(part(SweepPart::Rise {
                    loop_index: edge.loop_index,
                    vertex: vertex_index(edge, n),
                }));
                curves.push(rise);
            }
            rise_edge.push(row);
            rises.push(curves);
        }

        // The caps: the profile face on its own plane, its outward normal
        // against the sweep, and its translated copy.
        let slots = |edges: &[Vec<usize>]| -> Vec<Vec<Option<usize>>> {
            edges
                .iter()
                .map(|row| row.iter().copied().map(Some).collect())
                .collect()
        };
        let (start_use, end_use) = if along_normal {
            (Orientation::Reversed, Orientation::Forward)
        } else {
            (Orientation::Forward, Orientation::Reversed)
        };
        faces.push(cap_face(
            m,
            &loops,
            *plane,
            start_use,
            &slots(&start_edge),
            tolerance,
        ));
        face_roles.push(part(SweepPart::StartCap));
        faces.push(cap_face(
            m,
            &loops,
            plane.transformed(&shift),
            end_use,
            &slots(&end_edge),
            tolerance,
        ));
        face_roles.push(part(SweepPart::EndCap));

        // The sides: start edge, rise up, end edge back, rise down —
        // walked that way when the sweep runs along the profile's normal
        // and the other way otherwise, so the material is on the walk's
        // left seen from outside. A circle loop's one rise is its seam,
        // used twice.
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                let surface = &surfaces[li][j];
                let next = (j + 1) % n;
                let mut on = |curve: &Curve, range: Interval| -> Result<Curve2, OpError> {
                    pcurve_on(curve, range, surface, tol, meter).map_err(fault_of)
                };
                let start_pcurve = on(&edge.curve, edge.range)?;
                let orientation = side_orientation(edge, normal, surface, &start_pcurve)?;
                let mut cycle = vec![
                    SideUse {
                        edge: start_edge[li][j],
                        orientation: Orientation::Forward,
                        pcurve: start_pcurve,
                        range: edge.range,
                    },
                    SideUse {
                        edge: rise_edge[li][next],
                        orientation: Orientation::Forward,
                        pcurve: on(&rises[li][next], rise_range)?,
                        range: rise_range,
                    },
                    SideUse {
                        edge: end_edge[li][j],
                        orientation: Orientation::Reversed,
                        pcurve: on(&end_curves[li][j], edge.range)?,
                        range: edge.range,
                    },
                    SideUse {
                        edge: rise_edge[li][j],
                        orientation: Orientation::Reversed,
                        pcurve: on(&rises[li][j], rise_range)?,
                        range: rise_range,
                    },
                ];
                if !along_normal {
                    cycle.reverse();
                    for u in &mut cycle {
                        u.orientation = u.orientation.flipped();
                    }
                }
                faces.push(side_face(m, surface, orientation, cycle, false, tolerance));
                face_roles.push(part(SweepPart::Side {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }));
            }
        }

        let assembly = Assembly {
            vertices,
            edges,
            shells: vec![faces],
        };
        let (b, slots) = Builder::assemble(m, tolerance, assembly)?;
        let built = b.finish(m, BodyKind::Solid)?;
        verify(m, built.body)?;
        let provenance = record(
            &built,
            &slots,
            &vertex_roles,
            &edge_roles,
            &face_roles,
            &[part(SweepPart::Shell)],
            part,
        );
        Ok((built.body, provenance))
    })
}
