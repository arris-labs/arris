//! The sweep's properties (ADR-0063): the twin property (plans/sweep
//! step 5). A planar path of lines and arcs in the xz plane, swept by a
//! profile on the xy plane, builds exact faces — planes, cylinders, tori —
//! and the same path given as its exact NURBS twin (each line a degree-one
//! NURBS, each arc its rational quadratic circle) builds fitted ones; the
//! two bodies agree in volume, area and centroid to what a fit may miss,
//! and the exact one is clean at `Full` with no NURBS face.

use std::f64::consts::{PI, TAU};

use arris_check::{Level, Unchecked, check};
use arris_debug::testing::fail;
use arris_debug::unmetered::mass_properties;
use arris_debug::{prop, prop_shards};
use arris_geom::{
    FrameLaw, NurbsCurve, Path, PathSegment, Profile, ProfileLoop, ProfileSegment, SurfaceKind,
};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{Axis, Frame, Interval, Isometry, Meter, Point2, Point3, Vec3};
use arris_ops::{Control, OpError, Reason, extrude, sweep, transform};
use arris_topo::{Body, Model};
use proptest::prelude::*;

/// How near the fitted twin's measures come to the exact build's, as a
/// fraction of the body's size: the skin is within a quarter of the
/// default tolerance of the carried profile, and a measure moves by that
/// relative to the size of what it integrates.
const TWIN_REL: f64 = 1e-7;

/// One segment of a planar path: its length, or its radius and turn.
#[derive(Debug, Clone, Copy)]
enum Piece {
    Line { length: f64 },
    Arc { radius: f64, turn: f64, left: bool },
}

fn piece() -> impl Strategy<Value = Piece> {
    prop_oneof![
        prop::finite_f64(2.0..=15.0).prop_map(|length| Piece::Line { length }),
        (
            prop::finite_f64(6.0..=20.0),
            prop::finite_f64(0.2..=0.9),
            any::<bool>()
        )
            .prop_map(|(radius, turn, left)| Piece::Arc { radius, turn, left }),
    ]
}

/// A profile on the xy plane, normal to a path that leaves along z: a
/// rectangle or a disc off the origin by at most 1.5 and reaching at most
/// 3.5, inside the smallest radius of curvature (6).
#[derive(Debug, Clone, Copy)]
enum Section {
    Rectangle {
        centre: (f64, f64),
        half: (f64, f64),
    },
    Disc {
        centre: (f64, f64),
        radius: f64,
    },
}

fn section() -> impl Strategy<Value = Section> {
    let offset = || prop::finite_f64(-1.5..=1.5);
    prop_oneof![
        (
            (offset(), offset()),
            (prop::finite_f64(0.3..=2.0), prop::finite_f64(0.3..=2.0))
        )
            .prop_map(|(centre, half)| Section::Rectangle { centre, half }),
        ((offset(), offset()), prop::finite_f64(0.3..=2.0))
            .prop_map(|(centre, radius)| Section::Disc { centre, radius }),
    ]
}

fn profile(section: Section) -> Profile {
    let outer = match section {
        Section::Rectangle {
            centre: (cx, cy),
            half: (hx, hy),
        } => {
            let p = |u, v| Point2::new(cx + u * hx, cy + v * hy);
            ProfileLoop::Path {
                start: p(-1.0, -1.0),
                segments: vec![
                    ProfileSegment::LineTo(p(1.0, -1.0)),
                    ProfileSegment::LineTo(p(1.0, 1.0)),
                    ProfileSegment::LineTo(p(-1.0, 1.0)),
                    ProfileSegment::LineTo(p(-1.0, -1.0)),
                ],
            }
        }
        Section::Disc {
            centre: (cx, cy),
            radius,
        } => ProfileLoop::Circle {
            center: Point2::new(cx, cy),
            radius,
        },
    };
    Profile {
        plane: Frame::world(),
        outer,
        holes: Vec::new(),
    }
}

/// The path of `pieces` from the origin along +z in the xz plane, as
/// written and as its NURBS twin.
fn paths(pieces: &[Piece]) -> Result<(Path, Path), TestCaseError> {
    let mut at = Point3::origin();
    let mut heading = Vec3::z();
    let mut exact = Vec::new();
    let mut twin = Vec::new();
    for &piece in pieces {
        match piece {
            Piece::Line { length } => {
                let to = at + heading * length;
                exact.push(PathSegment::LineTo(to));
                twin.push(PathSegment::Nurbs(
                    NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![at, to], vec![1.0, 1.0])
                        .map_err(fail)?,
                ));
                at = to;
            }
            Piece::Arc { radius, turn, left } => {
                let side = if left { 1.0 } else { -1.0 };
                // The unit normal to the left of the heading in the xz plane.
                let left_normal = Vec3::y().cross(&heading);
                let centre = at + left_normal * (side * radius);
                let out = -left_normal * side;
                let position =
                    |theta: f64| centre + (out * theta.cos() + heading * theta.sin()) * radius;
                exact.push(PathSegment::ArcTo {
                    via: position(0.5 * turn),
                    to: position(turn),
                });
                // Counter-clockwise about `out × heading` from `out`.
                let frame = Frame::new(centre, out.cross(&heading), out).map_err(fail)?;
                twin.push(PathSegment::Nurbs(
                    NurbsCurve::circle(&frame, radius, Interval::new(0.0, turn).map_err(fail)?)
                        .map_err(fail)?,
                ));
                at = position(turn);
                heading = left_normal * (side * turn.sin()) + heading * turn.cos();
            }
        }
    }
    Ok((
        Path {
            start: Point3::origin(),
            segments: exact,
        },
        Path {
            start: Point3::origin(),
            segments: twin,
        },
    ))
}

prop_shards! {
    /// A path of lines and arcs sweeps the same body exact and as its
    /// NURBS twin.
    an_exact_path_and_its_nurbs_twin_sweep_one_body
        [shard_0 shard_1 shard_2 shard_3]
        ((pieces, section)) = (
            proptest::collection::vec(piece(), 1..=3).prop_filter(
                "the path turns less than a half turn in all",
                |pieces| pieces.iter().map(|p| match p {
                    Piece::Arc { turn, .. } => *turn,
                    Piece::Line { .. } => 0.0,
                }).sum::<f64>() <= 2.7,
            ),
            section(),
        ) => {
            let (exact, twin) = paths(&pieces)?;
            let profile = profile(section);
            let mut m = Model::default();
            let (a, _) = sweep(&mut m, &profile, &exact, FrameLaw::RotationMinimizing, &Control::NONE)
                .map_err(|e| fail(format!("exact: {e}")))?;
            let (b, _) = sweep(&mut m, &profile, &twin, FrameLaw::RotationMinimizing, &Control::NONE)
                .map_err(|e| fail(format!("twin: {e}")))?;
            let report = check(&m, a, Level::Full);
            prop_assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
            for face in m.faces(a).map_err(fail)? {
                let kind = m.surface(m.face(face.id).map_err(fail)?.surface()).map_err(fail)?.kind();
                prop_assert!(kind != SurfaceKind::Nurbs, "a NURBS face in the exact sweep");
            }
            let (ma, mb) = (
                mass_properties(&m, a).map_err(fail)?,
                mass_properties(&m, b).map_err(fail)?,
            );
            prop_assert!(ma.volume > 0.0 && mb.volume > 0.0, "{} {}", ma.volume, mb.volume);
            let size = ma.volume.cbrt();
            prop_assert!(
                (ma.volume - mb.volume).abs() <= TWIN_REL * ma.volume,
                "volume {} vs {}", ma.volume, mb.volume
            );
            prop_assert!(
                (ma.area - mb.area).abs() <= TWIN_REL * ma.area,
                "area {} vs {}", ma.area, mb.area
            );
            prop_assert!(
                (ma.centroid - mb.centroid).norm() <= TWIN_REL * size,
                "centroid {:?} vs {:?}", ma.centroid, mb.centroid
            );
            Ok(())
        }
}

prop_shards! {
    /// Every sweep over a random G1 path is checker-green or ends on a
    /// named refusal.
    the_checker_is_green_or_the_sweep_is_a_named_refusal
        [shard_0 shard_1 shard_2 shard_3]
        ((pieces, section)) = (
            proptest::collection::vec(g1_piece(), 1..=3),
            safe_section(),
        ) => {
            let path = build_g1_path(&pieces);
            let profile = profile(section);
            let law = FrameLaw::RotationMinimizing;
            let mut m = Model::default();
            match sweep(&mut m, &profile, &path, law, &Control::NONE) {
                Ok((body, _)) => {
                    check_green(&m, body)?;
                    let mass = mass_properties(&m, body).map_err(fail)?;
                    prop_assert!(mass.volume > 0.0 && mass.area > 0.0);
                }
                Err(e) => {
                    prop_assert!(is_named_refusal(&e), "unexpected refusal: {e:?}");
                }
            }
            Ok(())
        }
}

prop_shards! {
    /// Sweeping a profile along a path commutes with rigid motions: move
    /// then sweep equals sweep then move.
    a_sweep_commutes_with_rigid_motions
        [shard_0 shard_1 shard_2 shard_3]
        ((pieces, section, pose)) = (
            proptest::collection::vec(g1_piece(), 1..=3),
            safe_section(),
            prop::pose(),
        ) => {
            let path = build_g1_path(&pieces);
            let profile = profile(section);
            let law = FrameLaw::RotationMinimizing;
            let mut m1 = Model::default();
            let built = sweep(&mut m1, &profile, &path, law, &Control::NONE);
            let (body1, _) = match built {
                Ok(res) => res,
                Err(e) if is_named_refusal(&e) => return Ok(()),
                Err(e) => return Err(fail(format!("sweep: {e}"))),
            };

            let profile_moved = Profile {
                plane: pose.apply_frame(&profile.plane),
                outer: profile.outer.clone(),
                holes: profile.holes.clone(),
            };
            let path_moved = transform_path(&path, &pose);
            let mut m2 = Model::default();
            let (body2, _) = sweep(&mut m2, &profile_moved, &path_moved, law, &Control::NONE)
                .map_err(|e| fail(format!("moved sweep: {e}")))?;
            let (body1_moved, _) = transform(&mut m1, body1, &pose, &Control::NONE)
                .map_err(|e| fail(format!("transform: {e}")))?;

            let m_orig = mass_properties(&m1, body1_moved).map_err(fail)?;
            let m_moved = mass_properties(&m2, body2).map_err(fail)?;
            prop_assert!(
                (m_orig.volume - m_moved.volume).abs() <= 1e-6 * m_orig.volume,
                "volume {} vs {}", m_orig.volume, m_moved.volume
            );
            prop_assert!(
                (m_orig.area - m_moved.area).abs() <= 1e-6 * m_orig.area,
                "area {} vs {}", m_orig.area, m_moved.area
            );
            prop_assert!(
                (m_orig.centroid - m_moved.centroid).norm() <= 1e-6 * m_orig.volume.cbrt().max(1.0),
                "centroid {:?} vs {:?}", m_orig.centroid, m_moved.centroid
            );
            Ok(())
        }
}

prop_shards! {
    /// Splitting a path at an interior point adds the two pieces' volumes
    /// to the whole path's.
    splitting_the_path_at_an_interior_point_adds_volumes
        [shard_0 shard_1 shard_2 shard_3]
        ((pieces, section)) = (
            proptest::collection::vec(g1_piece(), 2..=3),
            safe_section(),
        ) => {
            let path = build_g1_path(&pieces);
            let profile = profile(section);
            let law = FrameLaw::RotationMinimizing;
            let mut m = Model::default();
            let built = sweep(&mut m, &profile, &path, law, &Control::NONE);
            let (body_all, _) = match built {
                Ok(res) => res,
                Err(e) if is_named_refusal(&e) => return Ok(()),
                Err(e) => return Err(fail(format!("sweep failed: {e}"))),
            };
            let v_all = mass_properties(&m, body_all).map_err(fail)?.volume;

            let tol = m.precision().tolerance();
            let mut meter = Meter::default();
            let field = path.frames(law, tol, &mut meter).map_err(fail)?;
            let (_, len1) = field.span_range(0);
            let p_split = field.at(len1).origin;
            let motion1 = field.motion_at(len1);

            let path1 = Path {
                start: path.start,
                segments: vec![path.segments[0].clone()],
            };
            let path2 = Path {
                start: p_split,
                segments: path.segments[1..].to_vec(),
            };
            let profile2 = Profile {
                plane: motion1.apply_frame(&profile.plane),
                outer: profile.outer.clone(),
                holes: profile.holes.clone(),
            };

            let (b1, _) = sweep(&mut m, &profile, &path1, law, &Control::NONE)
                .map_err(|e| fail(format!("first piece: {e}")))?;
            let (b2, _) = sweep(&mut m, &profile2, &path2, law, &Control::NONE)
                .map_err(|e| fail(format!("second piece: {e}")))?;

            let v1 = mass_properties(&m, b1).map_err(fail)?.volume;
            let v2 = mass_properties(&m, b2).map_err(fail)?.volume;
            prop_assert!(
                (v1 + v2 - v_all).abs() <= 1e-6 * v_all,
                "v1 + v2 = {} vs v_all = {}", v1 + v2, v_all
            );
            Ok(())
        }
}

prop_shards! {
    /// A line path agrees with `extrude` in volume, area, centroid and counts.
    a_line_path_agrees_with_extrude
        [shard_0 shard_1 shard_2 shard_3]
        ((section, length, tilt, angle)) = (
            safe_section(),
            prop::finite_f64(2.0..=12.0),
            prop::finite_f64(0.0..=0.5),
            prop::finite_f64(-PI..=PI),
        ) => {
            let profile = profile(section);
            let normal = profile.plane.z().into_inner();
            let x = profile.plane.x().into_inner();
            let y = profile.plane.y().into_inner();
            let (sn, cn) = angle.sin_cos();
            let side = x * cn + y * sn;
            let dir = (normal * tilt.cos() + side * tilt.sin()).normalize();
            let path = Path {
                start: profile.plane.origin(),
                segments: vec![PathSegment::LineTo(profile.plane.origin() + dir * length)],
            };
            let mut m1 = Model::default();
            let mut m2 = Model::default();
            let (b_sw, _) = sweep(&mut m1, &profile, &path, FrameLaw::RotationMinimizing, &Control::NONE)
                .map_err(|e| fail(format!("sweep: {e}")))?;
            let (b_ex, _) = extrude(&mut m2, &profile, dir, length, &Control::NONE)
                .map_err(|e| fail(format!("extrude: {e}")))?;
            let rep1 = check(&m1, b_sw, Level::Full);
            let rep2 = check(&m2, b_ex, Level::Full);
            prop_assert!(rep1.is_ok() && rep1.unchecked().is_empty(), "{rep1}");
            prop_assert!(rep2.is_ok() && rep2.unchecked().is_empty(), "{rep2}");
            let (m_sw, m_ex) = (
                mass_properties(&m1, b_sw).map_err(fail)?,
                mass_properties(&m2, b_ex).map_err(fail)?,
            );
            prop_assert!(
                (m_sw.volume - m_ex.volume).abs() <= 1e-7 * m_ex.volume,
                "volume {} vs {}", m_sw.volume, m_ex.volume
            );
            prop_assert!(
                (m_sw.area - m_ex.area).abs() <= 1e-7 * m_ex.area,
                "area {} vs {}", m_sw.area, m_ex.area
            );
            prop_assert!(
                (m_sw.centroid - m_ex.centroid).norm() <= 1e-7 * m_ex.volume.cbrt(),
                "centroid {:?} vs {:?}", m_sw.centroid, m_ex.centroid
            );
            prop_assert_eq!(counts(&m1, &[b_sw])?, counts(&m2, &[b_ex])?);
            Ok(())
        }
}

prop_shards! {
    /// A swept body round-trips through STEP: passes the checker and
    /// keeps its volume and counts.
    a_swept_body_round_trips_through_step
        [shard_0 shard_1 shard_2 shard_3]
        ((pieces, section)) = (
            proptest::collection::vec(g1_piece(), 1..=3),
            safe_section(),
        ) => {
            let path = build_g1_path(&pieces);
            let profile = profile(section);
            let law = FrameLaw::RotationMinimizing;
            let mut m = Model::default();
            let built = sweep(&mut m, &profile, &path, law, &Control::NONE);
            let (body, _) = match built {
                Ok(res) => res,
                Err(e) if is_named_refusal(&e) => return Ok(()),
                Err(e) => return Err(fail(format!("sweep: {e}"))),
            };

            let text = arris_io::step::write(&m, &[body]).map_err(fail)?;
            let mut back = Model::default();
            let read = arris_io::step::read(
                &mut back,
                &text,
                &arris_io::step::ReadOptions::default(),
                &Control::NONE,
            )
            .map_err(|e| fail(format!("read step: {e}")))?;
            prop_assert_eq!(read.solids.len(), 1);
            let read_body = match &read.solids[0].result {
                Ok(rb) => rb.body,
                Err(e) => return Err(fail(format!("solid refused: {e:?}"))),
            };

            check_green(&back, read_body)?;
            let c_orig = counts(&m, &[body])?;
            let c_back = counts(&back, &[read_body])?;
            prop_assert_eq!(c_orig, c_back);

            let m_orig = mass_properties(&m, body).map_err(fail)?;
            let m_back = mass_properties(&back, read_body).map_err(fail)?;
            prop_assert!(
                (m_orig.volume - m_back.volume).abs() <= 1e-6 * m_orig.volume,
                "volume {} vs {}", m_orig.volume, m_back.volume
            );
            Ok(())
        }
}

/// One piece of a general G1 3D path: a straight line, an arc in any
/// orientation around the tangent, or a helix winding about an axis.
#[derive(Debug, Clone, Copy)]
enum G1Piece {
    Line {
        length: f64,
    },
    Arc {
        radius: f64,
        turn: f64,
        roll: f64,
    },
    Helix {
        radius: f64,
        pitch: f64,
        turns: f64,
        roll: f64,
    },
}

fn g1_piece() -> impl Strategy<Value = G1Piece> {
    prop_oneof![
        prop::finite_f64(2.0..=12.0).prop_map(|length| G1Piece::Line { length }),
        (
            prop::finite_f64(6.0..=20.0),
            prop::finite_f64(0.2..=1.0),
            prop::finite_f64(-PI..=PI),
        )
            .prop_map(|(radius, turn, roll)| G1Piece::Arc { radius, turn, roll }),
        (
            prop::finite_f64(6.0..=15.0),
            prop::finite_f64(2.0..=8.0),
            prop::finite_f64(0.25..=0.75),
            prop::finite_f64(-PI..=PI),
        )
            .prop_map(|(radius, pitch, turns, roll)| G1Piece::Helix {
                radius,
                pitch,
                turns,
                roll,
            }),
    ]
}

fn build_g1_path(pieces: &[G1Piece]) -> Path {
    let mut here = Point3::origin();
    let mut t = Vec3::z();
    let mut n = Vec3::x();
    let mut segments = Vec::new();
    for &piece in pieces {
        match piece {
            G1Piece::Line { length } => {
                let to = here + t * length;
                segments.push(PathSegment::LineTo(to));
                here = to;
            }
            G1Piece::Arc { radius, turn, roll } => {
                let b = t.cross(&n);
                let (s_roll, c_roll) = roll.sin_cos();
                let n_arc = n * c_roll + b * s_roll;
                let center = here + n_arc * radius;
                let point = |theta: f64| {
                    center - n_arc * (radius * theta.cos()) + t * (radius * theta.sin())
                };
                let via = point(turn * 0.5);
                let to = point(turn);
                segments.push(PathSegment::ArcTo { via, to });
                here = to;
                let next_t = (n_arc * turn.sin() + t * turn.cos()).normalize();
                let rot_axis = t.cross(&n_arc);
                let next_n = if rot_axis.norm_squared() > 1e-12 {
                    let k = rot_axis.normalize();
                    let (sn, c) = turn.sin_cos();
                    n * c + k.cross(&n) * sn + k * (k.dot(&n) * (1.0 - c))
                } else {
                    n
                };
                t = next_t;
                n = next_n;
            }
            G1Piece::Helix {
                radius,
                pitch,
                turns,
                roll,
            } => {
                let b = t.cross(&n);
                let (s_roll, c_roll) = roll.sin_cos();
                let u = n * c_roll + b * s_roll;
                let rise = pitch / TAU;
                let alpha = (rise / radius).atan();
                let axis_dir = (t * alpha.sin() + u * alpha.cos()).normalize();
                let radial = t.cross(&u).normalize();
                let axis_origin = here - radial * radius;
                let axis = Axis::new(axis_origin, axis_dir).unwrap();
                let angle = turns * TAU;
                segments.push(PathSegment::Helix { axis, pitch, angle });
                let rot = UnitQuaternion::from_axis_angle(&axis.direction, angle);
                let rel = here - axis.origin;
                let along = rel.dot(&axis.direction.into_inner());
                let perp = rel - axis.direction.into_inner() * along;
                let perp_end = rot * perp;
                here =
                    axis.origin + axis.direction.into_inner() * (along + rise * angle) + perp_end;
                t = rot * t;
                n = rot * n;
            }
        }
    }
    Path {
        start: Point3::origin(),
        segments,
    }
}

fn safe_section() -> impl Strategy<Value = Section> {
    let offset = || prop::finite_f64(-0.8..=0.8);
    prop_oneof![
        (
            (offset(), offset()),
            (prop::finite_f64(0.3..=1.5), prop::finite_f64(0.3..=1.5))
        )
            .prop_map(|(centre, half)| Section::Rectangle { centre, half }),
        ((offset(), offset()), prop::finite_f64(0.3..=1.5))
            .prop_map(|(centre, radius)| Section::Disc { centre, radius }),
    ]
}

fn is_named_refusal(e: &OpError) -> bool {
    matches!(
        e,
        OpError::Degenerate {
            reason: Reason::Sweep(_),
            ..
        } | OpError::Path(_)
            | OpError::Profile(_)
            | OpError::Unsupported { .. }
    )
}

fn transform_path(path: &Path, iso: &Isometry) -> Path {
    let start = iso.apply(path.start);
    let segments = path
        .segments
        .iter()
        .map(|s| match s {
            PathSegment::LineTo(to) => PathSegment::LineTo(iso.apply(*to)),
            PathSegment::ArcTo { via, to } => PathSegment::ArcTo {
                via: iso.apply(*via),
                to: iso.apply(*to),
            },
            PathSegment::Helix { axis, pitch, angle } => PathSegment::Helix {
                axis: Axis {
                    origin: iso.apply(axis.origin),
                    direction: iso.apply_unit(axis.direction),
                },
                pitch: *pitch,
                angle: *angle,
            },
            PathSegment::Nurbs(curve) => PathSegment::Nurbs(curve.transformed(iso)),
        })
        .collect();
    Path { start, segments }
}

fn counts(m: &Model, bodies: &[Body]) -> Result<[usize; 5], TestCaseError> {
    let mut n = [0; 5];
    for &b in bodies {
        let c = m.closure(b).map_err(fail)?;
        let mut loops = 0;
        for &f in &c.faces {
            loops += m.face(f).map_err(fail)?.loops().len();
        }
        for (k, x) in [
            c.vertices.len(),
            c.edges.len(),
            c.faces.len(),
            loops,
            c.shells.len(),
        ]
        .into_iter()
        .enumerate()
        {
            n[k] += x;
        }
    }
    Ok(n)
}

fn check_green(m: &Model, body: Body) -> Result<(), TestCaseError> {
    let report = check(m, body, Level::Full);
    if !report.is_ok() {
        return Err(fail(format!("checker report: {report}")));
    }
    let ok = report
        .unchecked()
        .iter()
        .all(|u| matches!(u, Unchecked::FacePair { .. }));
    if !ok {
        return Err(fail(format!("unexpected unchecked items: {report}")));
    }
    Ok(())
}
