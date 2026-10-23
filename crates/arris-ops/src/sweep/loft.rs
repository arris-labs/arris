//! `loft`: a solid through two or more placed profiles, its loft sections
//! (ADR-0065). The sections are matched into one compatible NURBS per loop
//! (`match_sections`); a **ruled** loft joins consecutive sections by a
//! degree-one surface in the direction between them, one face per matched
//! segment per span; a **smooth** one by one surface per matched segment
//! through every section (`interpolate_skin`), so its only rings are the
//! end sections'.
//!
//! Two neighbouring faces of a span end on the same *rail*, the straight
//! column of control points at their shared vertex, and two spans meet on
//! the section's own matched segment, the isoline both faces are built
//! from: nothing in a loft is fitted, so every pcurve of a side face is an
//! exact line in `(u, v)`, `u` the matched segment's own parameter and `v`
//! the span's, `[0, 1]`.

use arris_geom::{
    Curve, Curve2, GeomError, LoftError, MAX_SMOOTH_SECTIONS, MatchedSection, NurbsCurve,
    NurbsCurve2, NurbsSurface, Profile, Surface, interpolate_skin, match_sections, pcurve_on,
};
use arris_math::{Control, Interval, Meter, Point2, UnitVec3};
use arris_topo::builder::{Assembly, Builder, EdgeSpec, FaceSpec, VertexKey, VertexSpec};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::provenance::{PathPart, SweepPart};
use arris_topo::{Body, Model, Orientation, Provenance, Role};

use crate::error::{Fault, OpError, fault_of};
use crate::verify;

use super::loft_exact::{Kind, analytic_curve, exact_face, kind_of};
use super::{SideUse, cap_face, record, side_face, side_orientation};

/// How a loft joins consecutive sections.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoftKind {
    /// Each span between consecutive sections is straight: a face of the
    /// span is ruled between the two sections' matched segments.
    Ruled,
    /// One surface through every section: per matched segment the
    /// polynomial of degree `n − 1` through the `n` sections' points at
    /// chord-length parameters ([`arris_geom::interpolate_skin`]), a
    /// closed loft counting its first section twice (ADR-0065 §7). Two
    /// sections are the ruled loft.
    Smooth,
}

/// The parameters of the sections along a smooth loft, `0` to `1`: each
/// span's share of the summed chord between consecutive sections' control
/// nets, every control point of every loop counted once, and the last
/// section back to the first when closed.
fn chord_parameters(matched: &[MatchedSection], closed: bool) -> Vec<f64> {
    let chord = |a: &MatchedSection, b: &MatchedSection| -> f64 {
        a.loops
            .iter()
            .zip(&b.loops)
            .flat_map(|(a, b)| {
                a.nurbs
                    .curve
                    .control_points()
                    .iter()
                    .zip(b.nurbs.curve.control_points())
            })
            .map(|(p, q)| (q - p).norm())
            .sum()
    };
    let mut chords: Vec<f64> = matched.windows(2).map(|w| chord(&w[0], &w[1])).collect();
    if let (true, Some(last), Some(first)) = (closed, matched.last(), matched.first()) {
        chords.push(chord(last, first));
    }
    let total: f64 = chords.iter().sum();
    let mut params = vec![0.0];
    let mut run = 0.0;
    for c in &chords {
        run += c;
        params.push(run / total);
    }
    if let Some(end) = params.last_mut() {
        *end = 1.0;
    }
    params
}

/// The ruled surface between two compatible segments, `u` the segments'
/// own parameter and `v` from `a` (`0`) to `b` (`1`).
fn ruled(a: &NurbsCurve, b: &NurbsCurve) -> Result<NurbsSurface, GeomError> {
    let points = a
        .control_points()
        .iter()
        .zip(b.control_points())
        .flat_map(|(p, q)| [*p, *q])
        .collect();
    let weights = a
        .weights()
        .iter()
        .zip(b.weights())
        .flat_map(|(w, x)| [*w, *x])
        .collect();
    NurbsSurface::new(
        [a.degree(), 1],
        [a.knots().to_vec(), vec![0.0, 0.0, 1.0, 1.0]],
        points,
        weights,
    )
}

/// The rail between control point `i` of the two segments: the degree-one
/// isoline of their ruled surface there, over `v` in `[0, 1]`.
fn rail(a: &NurbsCurve, b: &NurbsCurve, i: usize) -> Result<NurbsCurve, GeomError> {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![a.control_points()[i], b.control_points()[i]],
        vec![a.weights()[i], b.weights()[i]],
    )
}

/// How far the loft moves the points of loop 0 between sections `from`
/// and `to`, on average, along `normal`: positive when the loft leaves
/// the section along its normal.
fn flow(a: &[NurbsCurve], b: &[NurbsCurve], normal: &arris_math::Vec3) -> f64 {
    let (mut sum, mut count) = (0.0, 0usize);
    for (a, b) in a.iter().zip(b) {
        for (p, q) in a.control_points().iter().zip(b.control_points()) {
            sum += (q - p).dot(normal);
            count += 1;
        }
    }
    sum / count.max(1) as f64
}

/// A loft through `sections`, one solid.
///
/// The sections are placed profiles taken where they stand; loop `k` of
/// every section goes with loop `k` of the others (the outer with the
/// outer, hole `i` with hole `i`), from each loop's start point, a section
/// with fewer segments cut by arc length ([`arris_geom::match_sections`]).
/// Each span between consecutive sections has a face per matched segment
/// per loop, ruled; a [`LoftKind::Smooth`] loft of three sections or
/// more is one span, a face per matched segment per loop through every
/// section, and has rings only at its ends. The ends are planar caps, the
/// first section's profile face and the last's. With `closed` the last
/// section joins the first through one more span (smooth: the one span
/// returns to the first section) and there are no caps: a hole in the
/// profile closes into a void of the one lump.
///
/// Entities are appended in one fixed order — vertices per section ring
/// per loop in walking order, edges (the rings' matched segments, then
/// each span's rails), faces (start cap, end cap, sides per span per loop
/// per segment) — so the ids are a function of the inputs alone; every
/// tolerance is `default_tolerance`. Provenance is one `Generated` per
/// entity from a [`Role::Loft`] naming the span (from section `k` to
/// `k + 1`) and the part of the sketch it came from: a side face is
/// `Side { loop_index, segment }` with `segment` the matched segment in
/// the loop's walk, a joint between spans `k − 1` and `k` is span `k`'s
/// `StartEdge`, and a `Rise` is the rail one vertex runs along.
///
/// Errors, the model untouched: [`OpError::Loft`] when the sections
/// cannot be matched (too few, unequal loop counts, an invalid profile, a
/// section equal to the one before it), naming the section, a smooth
/// loft has more than [`MAX_SMOOTH_SECTIONS`] (the first counted twice
/// when closed), or its surface through one matched segment cannot be
/// interpolated ([`LoftError::Smooth`]); [`OpError::Interrupted`] when `control` stops it.
///
/// ```
/// use arris_geom::{Profile, ProfileLoop, ProfileSegment};
/// use arris_math::{Frame, Point2, Point3, Vec3};
/// use arris_ops::{Control, LoftKind, loft};
/// use arris_topo::Model;
///
/// let square = |z: f64, h: f64| Profile {
///     plane: Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).unwrap(),
///     outer: ProfileLoop::Path {
///         start: Point2::new(h, h),
///         segments: vec![
///             ProfileSegment::LineTo(Point2::new(-h, h)),
///             ProfileSegment::LineTo(Point2::new(-h, -h)),
///             ProfileSegment::LineTo(Point2::new(h, -h)),
///             ProfileSegment::LineTo(Point2::new(h, h)),
///         ],
///     },
///     holes: vec![],
/// };
/// let mut m = Model::default();
/// let sections = [square(0.0, 2.0), square(3.0, 1.0)];
/// let (body, _) = loft(&mut m, &sections, LoftKind::Ruled, false, &Control::NONE).unwrap();
/// assert_eq!(m.faces(body).unwrap().len(), 6, "four sides and two caps");
/// ```
pub fn loft(
    m: &mut Model,
    sections: &[Profile],
    kind: LoftKind,
    closed: bool,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    let count = sections.len();
    let smooth = match kind {
        LoftKind::Ruled => false,
        LoftKind::Smooth => count > 2,
    };
    let rows = count + usize::from(closed);
    if smooth && rows > MAX_SMOOTH_SECTIONS {
        return Err(LoftError::TooManySections {
            given: rows,
            max: MAX_SMOOTH_SECTIONS,
        }
        .into());
    }
    let mut meter = Meter::new(control);
    let meter = &mut meter;
    let precision = m.precision();
    let tol = precision.tolerance();
    let tolerance = precision.default_tolerance;
    let matched = match_sections(sections, closed, tol)?;
    // The sections that carry a ring of edges: every one of a ruled loft,
    // the ends of a smooth one (only the first when it closes).
    let stations: Vec<usize> = match (smooth, closed) {
        (false, _) => (0..count).collect(),
        (true, false) => vec![0, count - 1],
        (true, true) => vec![0],
    };
    let rings = stations.len();
    let spans = if closed { rings } else { rings - 1 };
    let end_ring = |k: usize| (k + 1) % rings;
    let loop_count = matched[0].loops.len();
    let counts: Vec<usize> = matched[0]
        .loops
        .iter()
        .map(|l| l.nurbs.segments.len())
        .collect();
    let segment = |r: usize, li: usize, j: usize| &matched[stations[r]].loops[li].nurbs.segments[j];
    let normal_of = |r: usize| sections[stations[r]].plane.z().into_inner();

    // Which way the loft leaves each end section: the start cap faces
    // against the flow and the end cap with it.
    let loop_zero = |r: usize| -> &[NurbsCurve] { &matched[r].loops[0].nurbs.segments };
    let leaves_start = flow(loop_zero(0), loop_zero(1), &normal_of(0)) > 0.0;
    let leaves_end = flow(
        loop_zero(count - 2),
        loop_zero(count - 1),
        &sections[count - 1].plane.z().into_inner(),
    ) > 0.0;

    // What each matched segment is, and so which faces are known outright
    // (ADR-0065 §6): `exact[k][li][j]` is span `k`'s face on segment `j`.
    let kinds: Vec<Vec<Vec<Kind>>> = stations
        .iter()
        .map(|&r| {
            matched[r]
                .loops
                .iter()
                .map(|l| l.nurbs.segments.iter().map(|c| kind_of(c, tol)).collect())
                .collect()
        })
        .collect();
    let exact: Vec<Vec<Vec<Option<Surface>>>> = (0..spans)
        .map(|k| {
            counts
                .iter()
                .enumerate()
                .map(|(li, &n)| {
                    (0..n)
                        .map(|j| {
                            if smooth {
                                return None;
                            }
                            exact_face(
                                (segment(k, li, j), &kinds[k][li][j]),
                                (segment(end_ring(k), li, j), &kinds[end_ring(k)][li][j]),
                                tol,
                            )
                        })
                        .collect()
                })
                .collect()
        })
        .collect();
    // A ring edge is the analytic line or circle only where every face it
    // bounds is exact; beside a NURBS face it is the segment's own NURBS.
    let ring_exact = |r: usize, li: usize, j: usize| -> bool {
        let before = if r == 0 {
            closed.then_some(spans - 1)
        } else {
            Some(r - 1)
        };
        let after = (r < spans).then_some(r);
        [before, after]
            .into_iter()
            .flatten()
            .all(|k| exact[k][li][j].is_some())
    };
    // A rail is the straight line wherever a face it bounds is exact.
    let rail_exact = |k: usize, li: usize, j: usize| -> bool {
        let n = counts[li];
        exact[k][li][j].is_some() || exact[k][li][(j + n - 1) % n].is_some()
    };
    let part = |span: usize, part: SweepPart| Role::Loft(PathPart { span, part });
    let last = spans - 1;

    // A smooth loft's one surface per matched segment, through every
    // section at chord-length parameters.
    let skins: Vec<Vec<NurbsSurface>> = if smooth {
        let params = chord_parameters(&matched, closed);
        counts
            .iter()
            .enumerate()
            .map(|(li, &n)| {
                (0..n)
                    .map(|j| {
                        meter.tick()?;
                        let column: Vec<NurbsCurve> = matched
                            .iter()
                            .map(|s| s.loops[li].nurbs.segments[j].clone())
                            .collect();
                        interpolate_skin(&column, &params, rows - 1, closed).map_err(|source| {
                            OpError::Loft(LoftError::Smooth {
                                loop_index: li,
                                segment: j,
                                source,
                            })
                        })
                    })
                    .collect::<Result<Vec<_>, OpError>>()
            })
            .collect::<Result<_, _>>()?
    } else {
        Vec::new()
    };

    m.transaction(|m| {
        let mut vertices: Vec<VertexSpec> = Vec::new();
        let mut vertex_roles: Vec<Role> = Vec::new();
        let mut edges: Vec<EdgeSpec> = Vec::new();
        let mut edge_roles: Vec<Role> = Vec::new();
        let mut faces: Vec<FaceSpec> = Vec::new();
        let mut face_roles: Vec<Role> = Vec::new();

        // The rings' vertices, per section per loop in walking order.
        let mut ring_vertex: Vec<Vec<Vec<usize>>> = Vec::with_capacity(rings);
        for r in 0..rings {
            let mut per_loop = Vec::with_capacity(loop_count);
            for (li, &n) in counts.iter().enumerate() {
                let mut ring = Vec::with_capacity(n);
                for j in 0..n {
                    meter.tick()?;
                    let seg = segment(r, li, j);
                    ring.push(vertices.len());
                    vertices.push(VertexSpec::New {
                        point: seg.eval(seg.domain().lo()).point,
                        tolerance,
                    });
                    vertex_roles.push(if closed || r < spans {
                        part(
                            r,
                            SweepPart::StartVertex {
                                loop_index: li,
                                vertex: j,
                            },
                        )
                    } else {
                        part(
                            last,
                            SweepPart::EndVertex {
                                loop_index: li,
                                vertex: j,
                            },
                        )
                    });
                }
                per_loop.push(ring);
            }
            ring_vertex.push(per_loop);
        }

        // The rings' edges: each section's matched segments, as the line or
        // circle they are where every face on them is exact.
        let mut ring_edge: Vec<Vec<Vec<usize>>> = Vec::with_capacity(rings);
        let mut ring_curve: Vec<Vec<Vec<(Curve, Interval)>>> = Vec::with_capacity(rings);
        for (r, ring_of) in ring_vertex.iter().enumerate() {
            let mut slots = Vec::with_capacity(loop_count);
            let mut curves = Vec::with_capacity(loop_count);
            for (li, &n) in counts.iter().enumerate() {
                let mut row = Vec::with_capacity(n);
                let mut row_curves = Vec::with_capacity(n);
                for j in 0..n {
                    meter.tick()?;
                    let seg = segment(r, li, j);
                    let (curve, range) = ring_exact(r, li, j)
                        .then(|| analytic_curve(&kinds[r][li][j]))
                        .flatten()
                        .unwrap_or_else(|| (Curve::Nurbs(seg.clone()), seg.domain()));
                    row.push(edges.len());
                    row_curves.push((curve.clone(), range));
                    edges.push(EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(curve),
                            range,
                        },
                        start: VertexKey::New(ring_of[li][j]),
                        end: VertexKey::New(ring_of[li][(j + 1) % n]),
                        tolerance,
                    });
                    edge_roles.push(if closed || r < spans {
                        part(
                            r,
                            SweepPart::StartEdge {
                                loop_index: li,
                                segment: j,
                            },
                        )
                    } else {
                        part(
                            last,
                            SweepPart::EndEdge {
                                loop_index: li,
                                segment: j,
                            },
                        )
                    });
                }
                slots.push(row);
                curves.push(row_curves);
            }
            ring_edge.push(slots);
            ring_curve.push(curves);
        }

        // Each span's rails: the straight column at a vertex.
        let mut rail_edge: Vec<Vec<Vec<usize>>> = Vec::with_capacity(spans);
        let mut rail_curve: Vec<Vec<Vec<(Curve, Interval)>>> = Vec::with_capacity(spans);
        for k in 0..spans {
            let mut per_loop = Vec::with_capacity(loop_count);
            let mut per_curve = Vec::with_capacity(loop_count);
            for (li, &n) in counts.iter().enumerate() {
                let mut row = Vec::with_capacity(n);
                let mut row_curves = Vec::with_capacity(n);
                for (j, &start) in ring_vertex[k][li].iter().enumerate() {
                    meter.tick()?;
                    let (a, b) = (segment(k, li, j), segment(end_ring(k), li, j));
                    let (from, to) = (a.eval(a.domain().lo()).point, b.eval(b.domain().lo()).point);
                    let straight = UnitVec3::try_new(to - from, 0.0)
                        .zip(Interval::new(0.0, (to - from).norm()).ok());
                    let (curve, range) = match straight {
                        Some((direction, range)) if rail_exact(k, li, j) => (
                            Curve::Line {
                                origin: from,
                                direction,
                            },
                            range,
                        ),
                        _ if smooth => (
                            Curve::Nurbs(skins[li][j].control_column(0).map_err(fault_of)?),
                            Interval::UNIT,
                        ),
                        _ => (
                            Curve::Nurbs(rail(a, b, 0).map_err(fault_of)?),
                            Interval::UNIT,
                        ),
                    };
                    row.push(edges.len());
                    row_curves.push((curve.clone(), range));
                    edges.push(EdgeSpec::New {
                        geometry: EdgeGeometry::Curve {
                            curve: m.add_curve(curve),
                            range,
                        },
                        start: VertexKey::New(start),
                        end: VertexKey::New(ring_vertex[end_ring(k)][li][j]),
                        tolerance,
                    });
                    edge_roles.push(part(
                        k,
                        SweepPart::Rise {
                            loop_index: li,
                            vertex: j,
                        },
                    ));
                }
                per_loop.push(row);
                per_curve.push(row_curves);
            }
            rail_edge.push(per_loop);
            rail_curve.push(per_curve);
        }

        // The caps of an open loft: each end section's profile face, its
        // matched segments' pcurves on the section's plane.
        if !closed {
            let mut cap = |m: &mut Model,
                           r: usize,
                           orientation: Orientation|
             -> Result<FaceSpec, OpError> {
                let plane = Surface::Plane {
                    frame: sections[stations[r]].plane,
                };
                let mut pcurves = Vec::with_capacity(loop_count);
                for curves in &ring_curve[r] {
                    let mut row = Vec::with_capacity(curves.len());
                    for (curve, range) in curves {
                        row.push(pcurve_on(curve, *range, &plane, tol, meter).map_err(fault_of)?);
                    }
                    pcurves.push(row);
                }
                let slots: Vec<Vec<Option<usize>>> = ring_edge[r]
                    .iter()
                    .map(|row| row.iter().copied().map(Some).collect())
                    .collect();
                Ok(cap_face(
                    m,
                    &pcurves,
                    sections[stations[r]].plane,
                    orientation,
                    &slots,
                    tolerance,
                ))
            };
            let (start_use, end_use) = (
                if leaves_start {
                    Orientation::Reversed
                } else {
                    Orientation::Forward
                },
                if leaves_end {
                    Orientation::Forward
                } else {
                    Orientation::Reversed
                },
            );
            faces.push(cap(m, 0, start_use)?);
            face_roles.push(part(0, SweepPart::StartCap));
            faces.push(cap(m, rings - 1, end_use)?);
            face_roles.push(part(last, SweepPart::EndCap));
        }

        // The sides: the ring's edge, the rail up at the next vertex, the
        // next ring's edge back, the rail down — reversed when the face is,
        // so the material is on the walk's left seen from outside.
        let mut side_loop: Vec<usize> = Vec::new();
        for k in 0..spans {
            let carried_normal = normal_of(k);
            for (li, &n) in counts.iter().enumerate() {
                for j in 0..n {
                    meter.tick()?;
                    let mut cycle;
                    let surface;
                    let orientation;
                    let reversed;
                    if let Some(exact_surface) = &exact[k][li][j] {
                        let mut on = |curve: &Curve, range: Interval| {
                            pcurve_on(curve, range, exact_surface, tol, meter).map_err(fault_of)
                        };
                        let next = (j + 1) % n;
                        let (start, end) = (&ring_curve[k][li][j], &ring_curve[end_ring(k)][li][j]);
                        let (up, down) = (&rail_curve[k][li][next], &rail_curve[k][li][j]);
                        let start_pcurve = on(&start.0, start.1)?;
                        orientation = side_orientation(
                            &start.0,
                            start.1,
                            carried_normal,
                            exact_surface,
                            &start_pcurve,
                        )?;
                        cycle = vec![
                            SideUse {
                                edge: ring_edge[k][li][j],
                                orientation: Orientation::Forward,
                                pcurve: start_pcurve,
                                range: start.1,
                            },
                            SideUse {
                                edge: rail_edge[k][li][next],
                                orientation: Orientation::Forward,
                                pcurve: on(&up.0, up.1)?,
                                range: up.1,
                            },
                            SideUse {
                                edge: ring_edge[end_ring(k)][li][j],
                                orientation: Orientation::Reversed,
                                pcurve: on(&end.0, end.1)?,
                                range: end.1,
                            },
                            SideUse {
                                edge: rail_edge[k][li][j],
                                orientation: Orientation::Reversed,
                                pcurve: on(&down.0, down.1)?,
                                range: down.1,
                            },
                        ];
                        // The walk is counter-clockwise seen from outside
                        // when it goes round the flow's side of the
                        // tangent: whatever the plane's own normal says.
                        let (at, to) = (segment(k, li, j), segment(end_ring(k), li, j));
                        let t = at.domain().midpoint();
                        let (here, there) = (at.eval(t), to.eval(t));
                        let walk = here.d1.cross(&(there.point - here.point));
                        if walk.dot(&here.d1.cross(&carried_normal)) < 0.0 {
                            cycle.reverse();
                            for u in &mut cycle {
                                u.orientation = u.orientation.flipped();
                            }
                        }
                        reversed = false;
                        surface = exact_surface.clone();
                    } else {
                        let (a, b) = (segment(k, li, j), segment(end_ring(k), li, j));
                        let sub = if smooth {
                            skins[li][j].clone()
                        } else {
                            ruled(a, b).map_err(fault_of)?
                        };
                        let [du, dv] = sub.domain();
                        // Outward is the section's tangent across its
                        // normal, as on the profile's own plane.
                        let at = sub.eval(du.midpoint(), dv.lo());
                        let outward = at.du.cross(&carried_normal);
                        let surface_normal = sub.normal(du.midpoint(), dv.lo()).ok_or(
                            OpError::Internal(Fault::Invariant {
                                what: "the ruled surface's normal at the segment's midpoint",
                            }),
                        )?;
                        orientation = if surface_normal.dot(&outward) > 0.0 {
                            Orientation::Forward
                        } else {
                            Orientation::Reversed
                        };
                        // A rail that is a straight line runs by arc length
                        // where the surface runs `v` from 0 to 1.
                        let rail_pcurve = |u: f64, range: Interval| -> Result<Curve2, OpError> {
                            if range == Interval::UNIT {
                                return Ok(Curve2::along_v(u));
                            }
                            NurbsCurve2::new(
                                1,
                                vec![range.lo(), range.lo(), range.hi(), range.hi()],
                                vec![Point2::new(u, 0.0), Point2::new(u, 1.0)],
                                vec![1.0, 1.0],
                            )
                            .map(Curve2::Nurbs)
                            .map_err(fault_of)
                        };
                        let next = (j + 1) % n;
                        let (up, down) = (&rail_curve[k][li][next], &rail_curve[k][li][j]);
                        cycle = vec![
                            SideUse {
                                edge: ring_edge[k][li][j],
                                orientation: Orientation::Forward,
                                pcurve: Curve2::along_u(dv.lo()),
                                range: du,
                            },
                            SideUse {
                                edge: rail_edge[k][li][next],
                                orientation: Orientation::Forward,
                                pcurve: rail_pcurve(du.hi(), up.1)?,
                                range: up.1,
                            },
                            SideUse {
                                edge: ring_edge[end_ring(k)][li][j],
                                orientation: Orientation::Reversed,
                                pcurve: Curve2::along_u(dv.hi()),
                                range: du,
                            },
                            SideUse {
                                edge: rail_edge[k][li][j],
                                orientation: Orientation::Reversed,
                                pcurve: rail_pcurve(du.lo(), down.1)?,
                                range: down.1,
                            },
                        ];
                        reversed = orientation.is_reversed();
                        surface = Surface::Nurbs(sub);
                    }
                    if reversed {
                        cycle.reverse();
                        for u in &mut cycle {
                            u.orientation = u.orientation.flipped();
                        }
                    }
                    faces.push(side_face(m, &surface, orientation, cycle, false, tolerance));
                    face_roles.push(part(
                        k,
                        SweepPart::Side {
                            loop_index: li,
                            segment: j,
                        },
                    ));
                    side_loop.push(li);
                }
            }
        }

        // An open loft's caps join everything into one shell; a closed
        // one's sides close a shell per loop, the outer loop's the lump's
        // outer shell and each hole's a void of it.
        let (shells, shell_roles, face_roles) = if closed {
            let mut buckets: Vec<Vec<(FaceSpec, Role)>> =
                (0..loop_count).map(|_| Vec::new()).collect();
            for ((face, role), li) in faces.into_iter().zip(face_roles).zip(side_loop) {
                buckets[li].push((face, role));
            }
            let mut shells = Vec::with_capacity(loop_count);
            let mut shell_roles = Vec::with_capacity(loop_count);
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
                            loop_index: li,
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
