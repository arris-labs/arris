//! The sweep's properties (ADR-0063): the twin property (plans/sweep
//! step 5). A planar path of lines and arcs in the xz plane, swept by a
//! profile on the xy plane, builds exact faces — planes, cylinders, tori —
//! and the same path given as its exact NURBS twin (each line a degree-one
//! NURBS, each arc its rational quadratic circle) builds fitted ones; the
//! two bodies agree in volume, area and centroid to what a fit may miss,
//! and the exact one is clean at `Full` with no NURBS face.

use arris_check::{Level, check};
use arris_debug::testing::fail;
use arris_debug::unmetered::mass_properties;
use arris_debug::{prop, prop_shards};
use arris_geom::{
    FrameLaw, NurbsCurve, Path, PathSegment, Profile, ProfileLoop, ProfileSegment, SurfaceKind,
};
use arris_math::{Frame, Interval, Point2, Point3, Vec3};
use arris_ops::{Control, sweep};
use arris_topo::Model;
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
