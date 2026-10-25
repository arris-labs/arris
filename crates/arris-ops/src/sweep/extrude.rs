//! `extrude` along the profile's normal, and `planar_face`, its cap
//! taken alone.

use arris_geom::{Curve, Curve2, Profile, ProfileEdge, Surface, pcurve_on};
use arris_math::real;
use arris_math::{Control, Frame, Interval, Isometry, Meter, Tolerance, UnitVec3, Vec3};
use arris_topo::builder::{Assembly, Builder, EdgeSpec, VertexKey, VertexSpec};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::provenance::SweepPart;
use arris_topo::{Body, Model, Orientation, Provenance, Role};

use crate::error::{InputReason, OpError, Reason, fault_of};
use crate::verify;

use super::skeleton::{
    Leg, Specs, all, counts, finish_solid, present, rails, ring_edges, ring_vertices, side_cycle,
};
use super::{
    cap_face, degenerate, profile_pcurves, record, side_face, side_orientation, vertex_index,
};

/// Extrudes `profile` along `direction` by `length` into a solid:
/// every segment of the profile sweeps one side face — a line a plane
/// whose `X` is the segment and `Y` the sweep, an arc a cylinder when
/// normal to the profile and an elliptic cylinder when oblique (ADR-0014,
/// ADR-0063), and an ellipse an elliptic cylinder — and every vertex a
/// straight *rise* along the sweep; every pcurve is exact through
/// `pcurve_on`. The sweep snaps to the plane's exact normal when within
/// `angular_tolerance`. The profile face keeps its plane's frame
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
/// direction, and [`InputReason::ZeroThickness`] for a length within
/// `default_tolerance` of zero or a direction whose normal component
/// times `length` is within the linear tolerance.
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
    let plane = &profile.plane;
    let (sweep, along_normal) = checked_sweep(plane, direction, length, tol)?;
    let loops = profile.edges(tol)?;
    let rise_range = Interval::new(0.0, length).map_err(|_| {
        degenerate(Reason::Input(InputReason::NotPositive {
            what: "length",
            value: length,
        }))
    })?;
    let exact =
        super::exact::line_span(&loops, plane, &Isometry::identity(), sweep, rise_range, tol)?
            .ok_or_else(|| degenerate(Reason::Input(InputReason::ZeroThickness)))?;
    let build = Build {
        loops: &loops,
        surfaces: &exact.surfaces,
        shift: Isometry::from_translation(sweep.into_inner() * length),
        sweep,
        rise_range,
        normal: plane.z().into_inner(),
        along_normal,
        tol,
        tolerance: precision.default_tolerance,
    };

    m.transaction(|m| {
        let mut specs = Specs::default();
        let rings = add_rings(m, &mut specs, meter, &build)?;

        // The caps: the profile face on its own plane, its outward normal
        // against the sweep, and its translated copy.
        let (start_use, end_use) = if along_normal {
            (Orientation::Reversed, Orientation::Forward)
        } else {
            (Orientation::Forward, Orientation::Reversed)
        };
        let start_cap = cap_face(
            m,
            &profile_pcurves(&loops),
            *plane,
            start_use,
            &present(&rings.start_edge),
            build.tolerance,
        );
        specs.face(start_cap, part(SweepPart::StartCap));
        let end_cap = cap_face(
            m,
            &profile_pcurves(&loops),
            plane.transformed(&build.shift),
            end_use,
            &present(&rings.end_edge),
            build.tolerance,
        );
        specs.face(end_cap, part(SweepPart::EndCap));

        add_sides(m, &mut specs, meter, &build, &rings)?;
        let shells = specs.one_shell(part(SweepPart::Shell));
        finish_solid(m, build.tolerance, specs, shells, part)
    })
}

fn part(p: SweepPart) -> Role {
    Role::Extrude(p)
}

/// The sweep's unit direction — the plane's exact normal when `direction`
/// is within `angular_tolerance` of it — and whether it runs along that
/// normal, or the refusal of a `length` or `direction` that sweeps
/// nothing.
fn checked_sweep(
    plane: &Frame,
    direction: Vec3,
    length: f64,
    tol: Tolerance,
) -> Result<(UnitVec3, bool), OpError> {
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
    let normal = plane.z().into_inner();
    let dn = d.dot(&normal);
    if dn.abs() * length <= tol.linear {
        return Err(degenerate(Reason::Input(InputReason::ZeroThickness)));
    }
    let along_normal = dn > 0.0;
    let off_normal = real::atan2(d.cross(&normal).norm(), dn.abs());
    let sweep = if off_normal <= tol.angular {
        if along_normal { plane.z() } else { -plane.z() }
    } else {
        d
    };
    Ok((sweep, along_normal))
}

/// What the rings and sides of one extrusion are built from.
struct Build<'a> {
    loops: &'a [Vec<ProfileEdge>],
    /// The exact surface each segment sweeps, per loop.
    surfaces: &'a [Vec<Surface>],
    shift: Isometry,
    sweep: UnitVec3,
    rise_range: Interval,
    normal: Vec3,
    along_normal: bool,
    tol: Tolerance,
    tolerance: f64,
}

/// The edges of an extrusion's rings and rises, by (loop, walk index),
/// with the curves the sides need pcurves of.
struct Rings {
    start_edge: Vec<Vec<usize>>,
    end_edge: Vec<Vec<usize>>,
    rise_edge: Vec<Vec<usize>>,
    end_curves: Vec<Vec<Curve>>,
    rises: Vec<Vec<Curve>>,
}

/// The start ring's vertices, the end ring's, then the start edges, the
/// end edges and the rises, appended in that order.
fn add_rings(
    m: &mut Model,
    specs: &mut Specs,
    meter: &mut Meter,
    b: &Build<'_>,
) -> Result<Rings, OpError> {
    let (loops, tolerance) = (b.loops, b.tolerance);
    let shape = counts(loops);
    let vertex_at = |li: usize, j: usize| -> &ProfileEdge { &loops[li][j] };
    let start_vertex = all(ring_vertices(
        specs,
        &shape,
        tolerance,
        |li, j| {
            Ok(Some(
                vertex_at(li, j).curve.point(vertex_at(li, j).range.lo()),
            ))
        },
        |li, j| {
            let edge = vertex_at(li, j);
            part(SweepPart::StartVertex {
                loop_index: edge.loop_index,
                vertex: vertex_index(edge, shape[li]),
            })
        },
    )?)?;
    let end_vertex = all(ring_vertices(
        specs,
        &shape,
        tolerance,
        |li, j| {
            let edge = vertex_at(li, j);
            Ok(Some(b.shift.apply(edge.curve.point(edge.range.lo()))))
        },
        |li, j| {
            let edge = vertex_at(li, j);
            part(SweepPart::EndVertex {
                loop_index: edge.loop_index,
                vertex: vertex_index(edge, shape[li]),
            })
        },
    )?)?;

    let start_edge = all(ring_edges(
        specs,
        &present(&start_vertex),
        tolerance,
        |li, j| {
            meter.tick()?;
            let edge = &loops[li][j];
            Ok(Some((
                EdgeGeometry::Curve {
                    curve: m.add_curve(edge.curve.clone()),
                    range: edge.range,
                },
                part(SweepPart::StartEdge {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }),
            )))
        },
    )?)?;
    let mut end_curves: Vec<Vec<Curve>> = shape.iter().map(|&n| Vec::with_capacity(n)).collect();
    let end_edge = all(ring_edges(
        specs,
        &present(&end_vertex),
        tolerance,
        |li, j| {
            meter.tick()?;
            let edge = &loops[li][j];
            let curve = edge.curve.transformed(&b.shift);
            let geometry = EdgeGeometry::Curve {
                curve: m.add_curve(curve.clone()),
                range: edge.range,
            };
            end_curves[li].push(curve);
            Ok(Some((
                geometry,
                part(SweepPart::EndEdge {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }),
            )))
        },
    )?)?;
    let mut rises: Vec<Vec<Curve>> = shape.iter().map(|&n| Vec::with_capacity(n)).collect();
    let rise_edge = all(rails(
        specs,
        &present(&start_vertex),
        &present(&end_vertex),
        tolerance,
        |_, li, j| {
            meter.tick()?;
            let edge = &loops[li][j];
            let rise = Curve::Line {
                origin: edge.curve.point(edge.range.lo()),
                direction: b.sweep,
            };
            let geometry = EdgeGeometry::Curve {
                curve: m.add_curve(rise.clone()),
                range: b.rise_range,
            };
            rises[li].push(rise);
            Ok(Some((
                geometry,
                part(SweepPart::Rise {
                    loop_index: edge.loop_index,
                    vertex: vertex_index(edge, shape[li]),
                }),
            )))
        },
    )?)?;
    Ok(Rings {
        start_edge,
        end_edge,
        rise_edge,
        end_curves,
        rises,
    })
}

/// The sides: start edge, rise up, end edge back, rise down — walked that
/// way when the sweep runs along the profile's normal and the other way
/// otherwise, so the material is on the walk's left seen from outside. A
/// circle loop's one rise is its seam, used twice.
fn add_sides(
    m: &mut Model,
    specs: &mut Specs,
    meter: &mut Meter,
    b: &Build<'_>,
    rings: &Rings,
) -> Result<(), OpError> {
    let rise_range = b.rise_range;
    for (li, edges_of) in b.loops.iter().enumerate() {
        let n = edges_of.len();
        for (j, edge) in edges_of.iter().enumerate() {
            meter.tick()?;
            let surface = &b.surfaces[li][j];
            let next = (j + 1) % n;
            let mut on = |curve: &Curve, range: Interval| -> Result<Curve2, OpError> {
                pcurve_on(curve, range, surface, b.tol, meter).map_err(fault_of)
            };
            let start_pcurve = on(&edge.curve, edge.range)?;
            let orientation =
                side_orientation(&edge.curve, edge.range, b.normal, surface, &start_pcurve)?;
            let cycle = side_cycle(
                [
                    Some(Leg {
                        edge: rings.start_edge[li][j],
                        pcurve: start_pcurve,
                        range: edge.range,
                    }),
                    Some(Leg {
                        edge: rings.rise_edge[li][next],
                        pcurve: on(&rings.rises[li][next], rise_range)?,
                        range: rise_range,
                    }),
                    Some(Leg {
                        edge: rings.end_edge[li][j],
                        pcurve: on(&rings.end_curves[li][j], edge.range)?,
                        range: edge.range,
                    }),
                    Some(Leg {
                        edge: rings.rise_edge[li][j],
                        pcurve: on(&rings.rises[li][j], rise_range)?,
                        range: rise_range,
                    }),
                ],
                !b.along_normal,
            );
            let side = side_face(m, surface, orientation, cycle, false, b.tolerance);
            specs.face(
                side,
                part(SweepPart::Side {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }),
            );
        }
    }
    Ok(())
}

/// The sheet body of one planar face: `profile`'s loops on its plane, the
/// face `Forward` on the plane's frame, so its front is the plane's
/// normal. The outer loop runs counter-clockwise about the normal and each
/// hole clockwise, as [`Profile::edges`] walks them, so the material is on
/// the left of every walk. Entities are appended in one fixed order —
/// vertices per loop in walking order, edges, the face — so the ids are a
/// function of the profile alone; every tolerance is `default_tolerance`.
/// Provenance is one `Generated` per entity from a [`Role::PlanarFace`]:
/// `SweepPart::StartVertex`, `StartEdge` (named as the sketch's vertex and
/// segment), `StartCap` for the face, `Shell` and `Body`.
///
/// This is the cap the sweeps close their solids with, taken alone
/// (M5 `⚠ OPEN` 1): `extrude`'s profile-side cap is this face used
/// `Reversed` when the sweep runs along the normal.
///
/// Errors, the model untouched: [`OpError::Profile`] when `Profile::edges`
/// refuses the sketch.
///
/// ```
/// use arris_ops::planar_face;
/// use arris_topo::Model;
/// use arris_topo::entity::BodyKind;
/// use arris_geom::{Profile, ProfileLoop};
/// use arris_math::{Frame, Point2};
///
/// // A disc of radius 3 on the xy plane: one face, one edge, one vertex.
/// let profile = Profile {
///     plane: Frame::world(),
///     outer: ProfileLoop::Circle { center: Point2::new(0.0, 0.0), radius: 3.0 },
///     holes: vec![],
/// };
/// let mut m = Model::default();
/// let (sheet, _provenance) = planar_face(&mut m, &profile, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.body(sheet.id).unwrap().kind(), BodyKind::Sheet);
/// assert_eq!(m.faces(sheet).unwrap().len(), 1);
/// assert_eq!(m.edges(sheet).unwrap().len(), 1);
/// ```
pub fn planar_face(
    m: &mut Model,
    profile: &Profile,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    let mut meter = Meter::new(control);
    let meter = &mut meter;
    let tol = m.precision().tolerance();
    let loops = profile.edges(tol)?;
    let tolerance = m.precision().default_tolerance;
    let part = |p: SweepPart| Role::PlanarFace(p);

    m.transaction(|m| {
        let mut vertices: Vec<VertexSpec> = Vec::new();
        let mut vertex_roles: Vec<Role> = Vec::new();
        let mut edges: Vec<EdgeSpec> = Vec::new();
        let mut edge_roles: Vec<Role> = Vec::new();
        let mut ring_of: Vec<Vec<usize>> = Vec::with_capacity(loops.len());
        for edges_of in &loops {
            let n = edges_of.len();
            let mut ring = Vec::with_capacity(n);
            for edge in edges_of {
                meter.tick()?;
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
            ring_of.push(ring);
        }
        let mut slots: Vec<Vec<Option<usize>>> = Vec::with_capacity(loops.len());
        for (li, edges_of) in loops.iter().enumerate() {
            let n = edges_of.len();
            let mut row = Vec::with_capacity(n);
            for (j, edge) in edges_of.iter().enumerate() {
                meter.tick()?;
                row.push(Some(edges.len()));
                edges.push(EdgeSpec::New {
                    geometry: EdgeGeometry::Curve {
                        curve: m.add_curve(edge.curve.clone()),
                        range: edge.range,
                    },
                    start: VertexKey::New(ring_of[li][j]),
                    end: VertexKey::New(ring_of[li][(j + 1) % n]),
                    tolerance,
                });
                edge_roles.push(part(SweepPart::StartEdge {
                    loop_index: edge.loop_index,
                    segment: edge.segment,
                }));
            }
            slots.push(row);
        }
        let face = cap_face(
            m,
            &profile_pcurves(&loops),
            profile.plane,
            Orientation::Forward,
            &slots,
            tolerance,
        );
        let assembly = Assembly {
            vertices,
            edges,
            shells: vec![vec![face]],
        };
        let (b, assigned) = Builder::assemble_sheet(m, tolerance, assembly)?;
        let built = b.finish(m, BodyKind::Sheet)?;
        verify(m, built.body)?;
        let provenance = record(
            &built,
            &assigned,
            &vertex_roles,
            &edge_roles,
            &[part(SweepPart::StartCap)],
            &[part(SweepPart::Shell)],
            part,
        );
        Ok((built.body, provenance))
    })
}
