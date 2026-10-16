//! `sweep`: a profile carried along a path (ADR-0063). The path's frame
//! field gives the profile's rigid motion at every arc length; each span
//! of the path — one segment — builds its own faces, joined to the next
//! span's at the profile carried exactly to the joint.
//!
//! Every span is fitted here (ADR-0063 §7): each loop of the profile is
//! one rational quadratic NURBS, its segments joined at the vertices by
//! knots of multiplicity two, carried by the motion and skinned by
//! `fit_skin` along the span's arc length. The skin's net splits at those
//! knots exactly into one surface per segment, so two neighbouring side
//! faces end on the same column of control points, and that column, a
//! NURBS curve along the path, is the rail they share: the isoline of
//! both, with no deviation to carry. Across the path the same holds: the
//! skin interpolates its end sections, so the segment's own NURBS carried
//! to a joint is the isoline there of the faces on both sides, and of the
//! cap, where its control points lie in the cap's plane. No pcurve on a
//! fitted span is fitted.

use arris_geom::{
    Curve, Curve2, FrameField, FrameLaw, GeomError, LoopNurbs, LoopPiece, NurbsCurve, NurbsSurface,
    Path, Profile, ProfileEdge, SECTION_FIT_FRACTION, Surface, fit_skin, pcurve_on,
};
use arris_math::{Control, Interval, Isometry, Meter, Point2, UnitVec2, Vec2};
use arris_topo::builder::{Assembly, Builder, EdgeSpec, FaceSpec, VertexKey, VertexSpec};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::provenance::{PathPart, SweepPart};
use arris_topo::{Body, Model, Orientation, Provenance, Role};

use crate::error::{Fault, OpError, fault_of};
use crate::verify;

use super::exact::{ExactSpan, exact_span};
use super::{
    SideUse, cap_face, profile_pcurves, record, refuse, side_face, side_orientation, vertex_index,
};

/// The degree of a fitted span's surface along the path: a quintic, as a
/// fitted section is (`SECTION_FIT_DEGREE`). Every column of the skin
/// follows a point of the profile round the path's turns, a helix about
/// the axis for a spring, and a quintic holds a turn of radius 10 to the
/// fit's share of the default tolerance in about thirty spans where a
/// cubic needs some two hundred. A structural choice, not a tolerance.
const SWEEP_FIT_DEGREE: usize = 5;

/// The loop `edges`, walked as oriented, as one [`LoopNurbs`]; a curve
/// no rational quadratic holds is the profile's fault.
fn composite(edges: &[ProfileEdge]) -> Result<LoopNurbs, OpError> {
    LoopNurbs::new(edges).map_err(|e| match e {
        e @ GeomError::Unsupported { .. } => OpError::Internal(Fault::ProfileCurve(e)),
        other => fault_of(other),
    })
}

/// The surface of the skin over one segment's columns `piece`: exact,
/// since the knot of multiplicity two at each vertex makes the segments'
/// bases independent; its `u` is the segment's own parameter range.
fn piece_surface(skin: &NurbsSurface, piece: &LoopPiece) -> Result<NurbsSurface, GeomError> {
    let [_, m] = skin.counts();
    let rows = piece.first..piece.first + piece.count;
    let at = |i: usize, j: usize| i * m + j;
    let points = rows
        .clone()
        .flat_map(|i| (0..m).map(move |j| skin.control_points()[at(i, j)]))
        .collect();
    let weights = rows
        .flat_map(|i| (0..m).map(move |j| skin.weights()[at(i, j)]))
        .collect();
    NurbsSurface::new(
        skin.degree(),
        [piece.knots.clone(), skin.knots()[1].to_vec()],
        points,
        weights,
    )
}

/// A fitted span with no skin: a kernel bug, never a property of the sweep.
fn unskinned() -> OpError {
    OpError::Internal(Fault::Invariant {
        what: "a skin for every fitted span",
    })
}

/// The skin's isoline at the `u` of control row `i`, along the path: the
/// row's control points and weights over the knots along `v`, the rail a
/// profile vertex at that row runs along.
fn column(skin: &NurbsSurface, i: usize) -> Result<NurbsCurve, GeomError> {
    let [_, m] = skin.counts();
    let [_, q] = skin.degree();
    NurbsCurve::new(
        q,
        skin.knots()[1].to_vec(),
        skin.control_points()[i * m..(i + 1) * m].to_vec(),
        skin.weights()[i * m..(i + 1) * m].to_vec(),
    )
}

/// The pcurve of a rail on a side face: the isoline at `u`, its parameter
/// the path's arc length `v`.
fn along_path(u: f64) -> Curve2 {
    Curve2::Line {
        origin: Point2::new(u, 0.0),
        direction: UnitVec2::new_unchecked(Vec2::y()),
    }
}

/// The pcurve of a joint's edge on a side face: the isoline at the
/// joint's arc length `v`, its parameter the segment's own `u`.
fn across_path(v: f64) -> Curve2 {
    Curve2::Line {
        origin: Point2::new(0.0, v),
        direction: UnitVec2::new_unchecked(Vec2::x()),
    }
}

/// The fitted skin of one loop over one span: the loop carried by the
/// field's motion at each arc length of `range`, skinned within
/// [`SECTION_FIT_FRACTION`] of the tolerance (ADR-0062's budget, ADR-0063
/// §7).
fn skin_span(
    loop_curve: &NurbsCurve,
    field: &FrameField,
    range: Interval,
    tolerance: f64,
    meter: &mut Meter,
) -> Result<NurbsSurface, OpError> {
    let section = |v: f64| -> Result<NurbsCurve, GeomError> {
        Ok(loop_curve.transformed(&field.motion_at(v)))
    };
    fit_skin(
        section,
        range,
        SWEEP_FIT_DEGREE,
        SECTION_FIT_FRACTION * tolerance,
        meter,
    )
    .map_err(fault_of)
}

/// Sweeps `profile` along `path` into a solid, the profile turning under
/// `law` ([`FrameLaw::RotationMinimizing`] by default, or a fixed
/// binormal for a spring or a thread). The profile is taken where it
/// stands — it need not be normal to the path or touch it — and carried
/// by the rigid motion from the path's first frame to its frame at each
/// arc length ([`FrameField::motion_at`], ADR-0063 §4).
///
/// Each span of the path, one segment, builds its own faces: a side face
/// per segment of the profile, its surface a NURBS skinned through the
/// carried profile within a quarter of `default_tolerance` (ADR-0062's
/// budget), `u` the profile segment's own parameter and `v` the path's arc
/// length. Each profile vertex runs along a *rail* over the span, the
/// isoline both its side faces end on; where two spans meet, each segment
/// carried exactly to the joint — as its NURBS, a line raised to degree
/// two, an arc its rational quadratic — is the isoline both spans' faces
/// share; and the ends are planar caps, the start the profile face
/// itself, its outward normal against the path, and the end its copy
/// carried to the path's end. Every pcurve is exact: a line in a side
/// face's (u, v), a NURBS on a cap. A
/// path that ends where it starts closes the frame and builds no caps:
/// its last span ends on the first span's edges, and each hole of the
/// profile closes into a void of the one lump. Entities are appended in
/// one fixed order — vertices per joint ring per loop in walking order,
/// edges (the rings' carried segments, then each span's rails), faces
/// (start cap, end cap, sides per span per loop per segment) — so the ids
/// are a function of the inputs alone; every tolerance is
/// `default_tolerance`. Provenance is one `Generated` per entity from a
/// [`Role::Sweep`] naming the span and the part of the sketch
/// ([`PathPart`]).
///
/// Errors, the model untouched: [`OpError::Profile`] when
/// `Profile::edges` refuses the sketch; [`OpError::Path`] when
/// `Path::frames` refuses the path — a gap, a corner, a degenerate
/// segment, a fixed binormal along the tangent — naming the segment; and
/// [`OpError::Interrupted`] when `control` stops it.
///
/// ```
/// use arris_ops::sweep;
/// use arris_topo::Model;
/// use arris_geom::{FrameLaw, Path, PathSegment, Profile, ProfileLoop};
/// use arris_math::{Axis, Frame, Point2, Point3, Vec3};
/// use arris_topo::provenance::{PathPart, Role, SweepPart};
/// use core::f64::consts::TAU;
///
/// // A wire of radius 0.5 wound one turn round a helix of radius 5 and
/// // pitch 3 about z, the profile in the axis' plane at (5, 0, 0).
/// let plane = Frame::new(Point3::new(5.0, 0.0, 0.0), -Vec3::y(), Vec3::x()).unwrap();
/// let profile = Profile {
///     plane,
///     outer: ProfileLoop::Circle { center: Point2::new(0.0, 0.0), radius: 0.5 },
///     holes: Vec::new(),
/// };
/// let path = Path {
///     start: Point3::new(5.0, 0.0, 0.0),
///     segments: vec![arris_geom::PathSegment::Helix {
///         axis: Axis::z_at(Point3::origin()),
///         pitch: 3.0,
///         angle: TAU,
///     }],
/// };
/// let mut m = Model::default();
/// let law = FrameLaw::FixedBinormal(Vec3::z_axis());
/// let (body, provenance) = sweep(&mut m, &profile, &path, law, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(body).unwrap().len(), 3, "two caps and the wire's skin");
/// let side = Role::Sweep(PathPart { span: 0, part: SweepPart::Side { loop_index: 0, segment: 0 } });
/// assert_eq!(provenance.generated_from(side).len(), 1);
/// ```
pub fn sweep(
    m: &mut Model,
    profile: &Profile,
    path: &Path,
    law: FrameLaw,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    let mut meter = Meter::new(control);
    let meter = &mut meter;
    let precision = m.precision();
    let tol = precision.tolerance();
    let tolerance = precision.default_tolerance;
    let loops = profile.edges(tol)?;
    let field = path.frames(law, tol, meter)?;
    let plane = &profile.plane;
    let normal = plane.z().into_inner();
    refuse::along_path(&field, &normal, tol)?;
    refuse::frame_closes(&field, tol)?;
    refuse::folds(&field, &loops[0], tol)?;
    // The material is carried along the profile's normal when the path
    // leaves it that way; the caps' and the sides' walks follow from it,
    // and a rigid motion keeps it along the whole path.
    let along_normal = field.at(0.0).tangent.dot(&normal) > 0.0;
    let spans = field.span_count();
    let closed = field.is_closed();
    // The joint rings: one at each span's start and, on an open path, one
    // at its end; a closed path's last span ends on ring 0.
    let rings = if closed { spans } else { spans + 1 };
    let at_ring = |r: usize| {
        if r < spans {
            field.span_range(r).0
        } else {
            field.length()
        }
    };
    let motions: Vec<Isometry> = (0..rings).map(|r| field.motion_at(at_ring(r))).collect();
    let end_ring = |k: usize| (k + 1) % rings;
    let span_range = |k: usize| -> Result<Interval, OpError> {
        let (lo, hi) = field.span_range(k);
        Interval::new(lo, hi).map_err(|_| {
            OpError::Internal(Fault::Invariant {
                what: "a span of positive length",
            })
        })
    };

    // Every loop as one NURBS, then its skin over each span, split into
    // one surface per segment.
    let composites = loops
        .iter()
        .map(|edges| composite(edges))
        .collect::<Result<Vec<_>, _>>()?;
    // A span the exact faces hold is built from them; the rest are
    // skinned, and a ring is exact only where every span it joins is.
    let mut exacts: Vec<Option<ExactSpan>> = Vec::with_capacity(spans);
    for (k, segment) in path.segments.iter().enumerate() {
        meter.tick()?;
        let end = field.motion_at(field.span_range(k).1);
        exacts.push(exact_span(
            &field,
            k,
            segment,
            &loops,
            plane,
            (&motions[k], &end),
            tol,
        )?);
    }
    let exact_ring = |r: usize| -> bool {
        let before = if r == 0 {
            closed.then_some(spans - 1)
        } else {
            Some(r - 1)
        };
        let after = (r < spans).then_some(r);
        [before, after]
            .into_iter()
            .flatten()
            .all(|k| exacts[k].is_some())
    };
    let mut skins: Vec<Option<Vec<NurbsSurface>>> = Vec::with_capacity(spans);
    for (k, exact) in exacts.iter().enumerate() {
        if exact.is_some() {
            skins.push(None);
            continue;
        }
        let range = span_range(k)?;
        let mut row = Vec::with_capacity(composites.len());
        for c in &composites {
            row.push(skin_span(&c.curve, &field, range, tolerance, meter)?);
        }
        skins.push(Some(row));
    }

    let part = |span: usize, part: SweepPart| Role::Sweep(PathPart { span, part });
    let last = spans - 1;

    m.transaction(|m| {
        let mut vertices: Vec<VertexSpec> = Vec::new();
        let mut vertex_roles: Vec<Role> = Vec::new();
        let mut edges: Vec<EdgeSpec> = Vec::new();
        let mut edge_roles: Vec<Role> = Vec::new();
        let mut faces: Vec<FaceSpec> = Vec::new();
        let mut face_roles: Vec<Role> = Vec::new();

        // The rings' vertices, per ring per loop in walking order: span
        // `r`'s start, or the last span's end on an open path's last ring.
        let mut ring_vertex: Vec<Vec<Vec<usize>>> = Vec::with_capacity(rings);
        for (r, motion) in motions.iter().enumerate() {
            let mut per_loop = Vec::with_capacity(loops.len());
            for edges_of in &loops {
                let n = edges_of.len();
                let mut ring = Vec::with_capacity(n);
                for edge in edges_of {
                    meter.tick()?;
                    ring.push(vertices.len());
                    vertices.push(VertexSpec::New {
                        point: motion.apply(edge.curve.point(edge.range.lo())),
                        tolerance,
                    });
                    let (loop_index, vertex) = (edge.loop_index, vertex_index(edge, n));
                    vertex_roles.push(if r == spans {
                        part(last, SweepPart::EndVertex { loop_index, vertex })
                    } else {
                        part(r, SweepPart::StartVertex { loop_index, vertex })
                    });
                }
                per_loop.push(ring);
            }
            ring_vertex.push(per_loop);
        }

        // The rings' edges: each segment's own NURBS carried exactly to
        // the ring.
        let mut ring_edge: Vec<Vec<Vec<usize>>> = Vec::with_capacity(rings);
        let mut ring_range: Vec<Vec<Vec<Interval>>> = Vec::with_capacity(rings);
        let mut ring_curve: Vec<Vec<Vec<Curve>>> = Vec::with_capacity(rings);
        for (r, motion) in motions.iter().enumerate() {
            let analytic = exact_ring(r);
            let mut slots = Vec::with_capacity(loops.len());
            let mut ranges = Vec::with_capacity(loops.len());
            let mut curves = Vec::with_capacity(loops.len());
            for (li, edges_of) in loops.iter().enumerate() {
                let n = edges_of.len();
                let mut row = Vec::with_capacity(n);
                let mut row_ranges = Vec::with_capacity(n);
                let mut row_curves = Vec::with_capacity(n);
                for (j, edge) in edges_of.iter().enumerate() {
                    meter.tick()?;
                    // Between exact spans the ring is the profile's own
                    // segment carried; beside a fitted span it is that
                    // segment's NURBS, the isoline the fitted faces end on.
                    let (curve, range) = if analytic {
                        (edge.curve.transformed(motion), edge.range)
                    } else {
                        let segment = composites[li].segments[j].transformed(motion);
                        let range = segment.domain();
                        (Curve::Nurbs(segment), range)
                    };
                    row.push(edges.len());
                    edges.push(EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(curve.clone()),
                            range,
                        },
                        start: VertexKey::New(ring_vertex[r][li][j]),
                        end: VertexKey::New(ring_vertex[r][li][(j + 1) % n]),
                        tolerance,
                    });
                    let (loop_index, segment) = (edge.loop_index, edge.segment);
                    edge_roles.push(if r == spans {
                        part(
                            last,
                            SweepPart::EndEdge {
                                loop_index,
                                segment,
                            },
                        )
                    } else {
                        part(
                            r,
                            SweepPart::StartEdge {
                                loop_index,
                                segment,
                            },
                        )
                    });
                    row_ranges.push(range);
                    row_curves.push(curve);
                }
                slots.push(row);
                ranges.push(row_ranges);
                curves.push(row_curves);
            }
            ring_edge.push(slots);
            ring_range.push(ranges);
            ring_curve.push(curves);
        }

        // Each span's rails: a vertex's straight rise or circle about the
        // axis on an exact span, the skin's column at it on a fitted one.
        let mut rail: Vec<Vec<Vec<usize>>> = Vec::with_capacity(spans);
        let mut rail_curve: Vec<Vec<Vec<Curve>>> = Vec::with_capacity(spans);
        for k in 0..spans {
            let mut per_loop = Vec::with_capacity(loops.len());
            let mut per_curve = Vec::with_capacity(loops.len());
            for (li, edges_of) in loops.iter().enumerate() {
                let n = edges_of.len();
                let mut row = Vec::with_capacity(n);
                let mut row_curves = Vec::with_capacity(n);
                for (j, edge) in edges_of.iter().enumerate() {
                    meter.tick()?;
                    let (curve, range) = match (&exacts[k], &skins[k]) {
                        (Some(exact), _) => (exact.rails[li][j].clone(), exact.rail_range),
                        (None, Some(skins_of)) => {
                            let first = composites[li].pieces[j].first;
                            let column = column(&skins_of[li], first).map_err(fault_of)?;
                            (Curve::Nurbs(column), span_range(k)?)
                        }
                        (None, None) => return Err(unskinned()),
                    };
                    row.push(edges.len());
                    edges.push(EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(curve.clone()),
                            range,
                        },
                        start: VertexKey::New(ring_vertex[k][li][j]),
                        end: VertexKey::New(ring_vertex[end_ring(k)][li][j]),
                        tolerance,
                    });
                    edge_roles.push(part(
                        k,
                        SweepPart::Rise {
                            loop_index: edge.loop_index,
                            vertex: vertex_index(edge, n),
                        },
                    ));
                    row_curves.push(curve);
                }
                per_loop.push(row);
                per_curve.push(row_curves);
            }
            rail.push(per_loop);
            rail_curve.push(per_curve);
        }

        // The caps of an open path: the profile face, its outward normal
        // against the path, and its copy carried to the path's end.
        if !closed {
            let slots = |r: usize| -> Vec<Vec<Option<usize>>> {
                ring_edge[r]
                    .iter()
                    .map(|row| row.iter().copied().map(Some).collect())
                    .collect()
            };
            let (start_use, end_use) = if along_normal {
                (Orientation::Reversed, Orientation::Forward)
            } else {
                (Orientation::Forward, Orientation::Reversed)
            };
            // An exact ring's cap has the profile's own pcurves; a fitted
            // one's has each segment's NURBS in the plane's (u, v), exact,
            // and carried with the plane to the end.
            let on_plane = Surface::Plane { frame: *plane };
            let mut nurbs: Option<Vec<Vec<Curve2>>> = None;
            let mut pcurves_of = |analytic: bool| -> Result<Vec<Vec<Curve2>>, OpError> {
                if analytic {
                    return Ok(profile_pcurves(&loops));
                }
                if let Some(done) = &nurbs {
                    return Ok(done.clone());
                }
                let mut all: Vec<Vec<Curve2>> = Vec::with_capacity(loops.len());
                for c in &composites {
                    let mut row = Vec::with_capacity(c.segments.len());
                    for segment in &c.segments {
                        let curve = Curve::Nurbs(segment.clone());
                        row.push(
                            pcurve_on(&curve, segment.domain(), &on_plane, tol, meter)
                                .map_err(fault_of)?,
                        );
                    }
                    all.push(row);
                }
                nurbs = Some(all.clone());
                Ok(all)
            };
            let (start_pcurves, end_pcurves) =
                (pcurves_of(exact_ring(0))?, pcurves_of(exact_ring(spans))?);
            faces.push(cap_face(
                m,
                &start_pcurves,
                *plane,
                start_use,
                &slots(0),
                tolerance,
            ));
            face_roles.push(part(0, SweepPart::StartCap));
            faces.push(cap_face(
                m,
                &end_pcurves,
                plane.transformed(&motions[spans]),
                end_use,
                &slots(spans),
                tolerance,
            ));
            face_roles.push(part(last, SweepPart::EndCap));
        }

        // The sides: the ring's edge, the rail up at the next vertex, the
        // next ring's edge back, the rail down — walked that way when the
        // path leaves the profile along its normal and the other way
        // otherwise, so the material is on the walk's left seen from
        // outside. Every edge of a side is an isoline of its skin, so every
        // pcurve is a line in (u, v).
        let mut side_loop: Vec<usize> = Vec::new();
        for k in 0..spans {
            let range = span_range(k)?;
            let motion = &motions[k];
            let carried_normal = motion.apply_vec(normal);
            for (li, edges_of) in loops.iter().enumerate() {
                let n = edges_of.len();
                for (j, edge) in edges_of.iter().enumerate() {
                    meter.tick()?;
                    if let Some(exact) = &exacts[k] {
                        let surface = &exact.surfaces[li][j];
                        let mut on = |curve: &Curve, range: Interval| -> Result<Curve2, OpError> {
                            pcurve_on(curve, range, surface, tol, meter).map_err(fault_of)
                        };
                        let start = (&ring_curve[k][li][j], ring_range[k][li][j]);
                        let end = (
                            &ring_curve[end_ring(k)][li][j],
                            ring_range[end_ring(k)][li][j],
                        );
                        let start_pcurve = on(start.0, start.1)?;
                        let orientation = side_orientation(
                            start.0,
                            start.1,
                            carried_normal,
                            surface,
                            &start_pcurve,
                        )?;
                        let next = (j + 1) % n;
                        let mut cycle = vec![
                            SideUse {
                                edge: ring_edge[k][li][j],
                                orientation: Orientation::Forward,
                                pcurve: start_pcurve,
                                range: start.1,
                            },
                            SideUse {
                                edge: rail[k][li][next],
                                orientation: Orientation::Forward,
                                pcurve: on(&rail_curve[k][li][next], exact.rail_range)?,
                                range: exact.rail_range,
                            },
                            SideUse {
                                edge: ring_edge[end_ring(k)][li][j],
                                orientation: Orientation::Reversed,
                                pcurve: on(end.0, end.1)?,
                                range: end.1,
                            },
                            SideUse {
                                edge: rail[k][li][j],
                                orientation: Orientation::Reversed,
                                pcurve: on(&rail_curve[k][li][j], exact.rail_range)?,
                                range: exact.rail_range,
                            },
                        ];
                        if !along_normal {
                            cycle.reverse();
                            for u in &mut cycle {
                                u.orientation = u.orientation.flipped();
                            }
                        }
                        faces.push(side_face(m, surface, orientation, cycle, false, tolerance));
                        face_roles.push(part(
                            k,
                            SweepPart::Side {
                                loop_index: edge.loop_index,
                                segment: edge.segment,
                            },
                        ));
                        side_loop.push(li);
                        continue;
                    }
                    let Some(skins_of) = &skins[k] else {
                        return Err(unskinned());
                    };
                    let sub = piece_surface(&skins_of[li], &composites[li].pieces[j])
                        .map_err(fault_of)?;
                    let surface = Surface::Nurbs(sub.clone());
                    let [du, _] = sub.domain();
                    // Outward is the carried segment's tangent across the
                    // carried normal, as on the profile's own plane.
                    let at = sub.eval(du.midpoint(), range.lo());
                    let outward = at.du.cross(&carried_normal);
                    let surface_normal =
                        sub.normal(du.midpoint(), range.lo())
                            .ok_or(OpError::Internal(Fault::Invariant {
                                what: "the swept surface's normal at the segment's midpoint",
                            }))?;
                    let orientation = if surface_normal.dot(&outward) > 0.0 {
                        Orientation::Forward
                    } else {
                        Orientation::Reversed
                    };
                    let mut cycle = vec![
                        SideUse {
                            edge: ring_edge[k][li][j],
                            orientation: Orientation::Forward,
                            pcurve: across_path(range.lo()),
                            range: ring_range[k][li][j],
                        },
                        SideUse {
                            edge: rail[k][li][(j + 1) % n],
                            orientation: Orientation::Forward,
                            pcurve: along_path(du.hi()),
                            range,
                        },
                        SideUse {
                            edge: ring_edge[end_ring(k)][li][j],
                            orientation: Orientation::Reversed,
                            pcurve: across_path(range.hi()),
                            range: ring_range[end_ring(k)][li][j],
                        },
                        SideUse {
                            edge: rail[k][li][j],
                            orientation: Orientation::Reversed,
                            pcurve: along_path(du.lo()),
                            range,
                        },
                    ];
                    if !along_normal {
                        cycle.reverse();
                        for u in &mut cycle {
                            u.orientation = u.orientation.flipped();
                        }
                    }
                    faces.push(side_face(m, &surface, orientation, cycle, false, tolerance));
                    face_roles.push(part(
                        k,
                        SweepPart::Side {
                            loop_index: edge.loop_index,
                            segment: edge.segment,
                        },
                    ));
                    side_loop.push(li);
                }
            }
        }

        // An open path's caps join everything into one shell; a closed
        // one's sides close a shell per loop, the outer loop's the lump's
        // outer shell and each hole's a void of it.
        let (shells, shell_roles, face_roles) = if closed {
            let mut buckets: Vec<Vec<(FaceSpec, Role)>> =
                loops.iter().map(|_| Vec::new()).collect();
            for ((face, role), li) in faces.into_iter().zip(face_roles).zip(side_loop) {
                buckets[li].push((face, role));
            }
            let mut shells = Vec::with_capacity(loops.len());
            let mut shell_roles = Vec::with_capacity(loops.len());
            let mut face_roles = Vec::new();
            for (li, bucket) in buckets.into_iter().enumerate() {
                let (specs, roles): (Vec<FaceSpec>, Vec<Role>) = bucket.into_iter().unzip();
                shells.push(specs);
                face_roles.extend(roles);
                shell_roles.push(if li == 0 {
                    part(0, SweepPart::Shell)
                } else {
                    part(
                        0,
                        SweepPart::Cavity {
                            loop_index: loops[li].first().map_or(li, |e| e.loop_index),
                            segment: 0,
                        },
                    )
                });
            }
            (shells, shell_roles, face_roles)
        } else {
            (vec![faces], vec![part(0, SweepPart::Shell)], face_roles)
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
            |p| part(0, p),
        );
        Ok((built.body, provenance))
    })
}
