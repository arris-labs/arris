//! The loft's properties (ADR-0065): a loft commutes with rigid motions,
//! reversing the order of the sections keeps the volume and the area, two
//! equal parallel sections loft to their extrude, and similar coaxial
//! sections loft to the frustum formula. Sections are rectangles and
//! circles on the xy plane raised along z, so the closed forms are known.

use arris_check::{Level, Unchecked, Violation, check};
use arris_debug::testing::fail;
use arris_debug::unmetered::mass_properties;
use arris_debug::{prop, prop_shards};
use arris_geom::{Profile, ProfileLoop, ProfileSegment, Surface};
use arris_math::{Frame, Point2, Point3, Vec3, real};
use arris_ops::{Control, LoftKind, OpError, extrude, loft, transform};
use arris_topo::{Body, Model};
use core::f64::consts::PI;
use proptest::prelude::*;

/// How near two builds of one body come, as a fraction of its size: the
/// spans are exact or interpolated the same way, so only rounding parts
/// them.
const REL: f64 = 1e-7;

/// How near an extrude and a loft of equal sections come: both are exact.
const EXACT_REL: f64 = 1e-9;

/// One section on its plane: a rectangle of half extents, or a circle.
#[derive(Debug, Clone, Copy)]
enum Section {
    Rectangle(f64, f64),
    Circle(f64),
}

impl Section {
    fn area(self) -> f64 {
        match self {
            Section::Rectangle(x, y) => 4.0 * x * y,
            Section::Circle(r) => PI * r * r,
        }
    }

    fn scaled(self, s: f64) -> Section {
        match self {
            Section::Rectangle(x, y) => Section::Rectangle(x * s, y * s),
            Section::Circle(r) => Section::Circle(r * s),
        }
    }

    fn at(self, z: f64) -> Profile {
        let plane = Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x())
            .unwrap_or_else(|_| Frame::world());
        let outer = match self {
            Section::Rectangle(hx, hy) => {
                let p = |u: f64, v: f64| Point2::new(u * hx, v * hy);
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
            Section::Circle(radius) => ProfileLoop::Circle {
                center: Point2::origin(),
                radius,
            },
        };
        Profile {
            plane,
            outer,
            holes: Vec::new(),
        }
    }
}

fn rectangle() -> impl Strategy<Value = Section> {
    (prop::finite_f64(0.8..=3.0), prop::finite_f64(0.8..=3.0))
        .prop_map(|(x, y)| Section::Rectangle(x, y))
}

fn circle() -> impl Strategy<Value = Section> {
    prop::finite_f64(0.8..=3.0).prop_map(Section::Circle)
}

fn any_section() -> impl Strategy<Value = Section> {
    prop_oneof![rectangle(), circle()]
}

fn heights(count: usize) -> impl Strategy<Value = Vec<f64>> {
    proptest::collection::vec(prop::finite_f64(1.0..=4.0), count)
}

/// Two or three sections raised by cumulative heights, a ruled loft of
/// either kind of section, or a smooth one of one kind only.
fn stack() -> impl Strategy<Value = (LoftKind, Vec<Profile>)> {
    let raise = |sections: Vec<Section>, heights: Vec<f64>| {
        let mut z = 0.0;
        let mut profiles = vec![sections[0].at(0.0)];
        for (s, h) in sections[1..].iter().zip(&heights) {
            z += h;
            profiles.push(s.at(z));
        }
        profiles
    };
    prop_oneof![
        (
            proptest::collection::vec(any_section(), 3),
            heights(2),
            2usize..=3
        )
            .prop_map(move |(mut sections, heights, count)| {
                sections.truncate(count);
                (LoftKind::Ruled, raise(sections, heights))
            }),
        (
            proptest::collection::vec(rectangle(), 3),
            heights(2),
            2usize..=3
        )
            .prop_map(move |(mut sections, heights, count)| {
                sections.truncate(count);
                (LoftKind::Smooth, raise(sections, heights))
            }),
        (
            proptest::collection::vec(circle(), 3),
            heights(2),
            2usize..=3
        )
            .prop_map(move |(mut sections, heights, count)| {
                sections.truncate(count);
                (LoftKind::Smooth, raise(sections, heights))
            }),
    ]
}

/// A loft's body, clean at `Full` but for what two NURBS faces leave
/// undecided and the S5 findings the marched section of two of them
/// makes between neighbours that meet along their shared edge only:
/// held by the ignored
/// `regression/loft-rect-circle-rect-ruled-s5-false-positive`, whose fix
/// (`nurbs-nurbs-operands`) lifts the second arm.
fn check_green(m: &Model, body: Body) -> Result<(), TestCaseError> {
    let report = check(m, body, Level::Full);
    let nurbs = |f| {
        m.face(f)
            .ok()
            .and_then(|f| m.surface(f.surface()).ok())
            .is_some_and(|s| matches!(s, Surface::Nurbs(_)))
    };
    if !report.violations().iter().all(|v| {
        matches!(v, Violation::FacesIntersect { face_a, face_b, .. }
            if nurbs(*face_a) && nurbs(*face_b))
    }) {
        return Err(fail(format!("checker report: {report}")));
    }
    if !report
        .unchecked()
        .iter()
        .all(|u| matches!(u, Unchecked::FacePair { .. }))
    {
        return Err(fail(format!("unexpected unchecked items: {report}")));
    }
    Ok(())
}

fn build(m: &mut Model, kind: LoftKind, sections: &[Profile]) -> Result<Body, TestCaseError> {
    match loft(m, sections, kind, false, &Control::NONE) {
        Ok((body, _)) => Ok(body),
        Err(e @ OpError::Loft(_)) => Err(fail(format!("a drawn stack was refused: {e}"))),
        Err(e) => Err(fail(format!("loft: {e}"))),
    }
}

prop_shards! {
    /// Moving the sections then lofting equals lofting then moving the
    /// body.
    a_loft_commutes_with_rigid_motions
        [shard_0 shard_1 shard_2 shard_3]
        (((kind, sections), pose)) = (stack(), prop::pose()) => {
            let mut m1 = Model::default();
            let body = build(&mut m1, kind, &sections)?;
            check_green(&m1, body)?;
            let (moved_body, _) = transform(&mut m1, body, &pose, &Control::NONE)
                .map_err(|e| fail(format!("transform: {e}")))?;
            let moved: Vec<Profile> = sections
                .iter()
                .map(|p| Profile {
                    plane: pose.apply_frame(&p.plane),
                    outer: p.outer.clone(),
                    holes: p.holes.clone(),
                })
                .collect();
            let mut m2 = Model::default();
            let other = build(&mut m2, kind, &moved)?;
            let a = mass_properties(&m1, moved_body).map_err(fail)?;
            let b = mass_properties(&m2, other).map_err(fail)?;
            let size = real::cbrt(a.volume);
            prop_assert!((a.volume - b.volume).abs() <= REL * a.volume, "volume {} vs {}", a.volume, b.volume);
            prop_assert!((a.area - b.area).abs() <= REL * a.area, "area {} vs {}", a.area, b.area);
            prop_assert!(
                (a.centroid - b.centroid).norm() <= REL * size.max(1.0),
                "centroid {:?} vs {:?}", a.centroid, b.centroid
            );
            Ok(())
        }
}

prop_shards! {
    /// The same sections in the other order loft a body of the same
    /// volume and area.
    reversing_the_sections_keeps_volume_and_area
        [shard_0 shard_1 shard_2 shard_3]
        ((kind, sections)) = stack() => {
            let mut m = Model::default();
            let body = build(&mut m, kind, &sections)?;
            let mut reversed = sections.clone();
            reversed.reverse();
            let other = build(&mut m, kind, &reversed)?;
            check_green(&m, other)?;
            let a = mass_properties(&m, body).map_err(fail)?;
            let b = mass_properties(&m, other).map_err(fail)?;
            prop_assert!((a.volume - b.volume).abs() <= REL * a.volume, "volume {} vs {}", a.volume, b.volume);
            prop_assert!((a.area - b.area).abs() <= REL * a.area, "area {} vs {}", a.area, b.area);
            Ok(())
        }
}

prop_shards! {
    /// A profile and itself raised along its normal loft to the profile
    /// extruded that far: lines and arcs give planes and cylinders, so
    /// the two are exact.
    two_equal_parallel_sections_are_their_extrude
        [shard_0 shard_1 shard_2 shard_3]
        ((profile, height)) = (prop::profile::star(), prop::finite_f64(1.0..=4.0)) => {
            let normal = profile.plane.z().into_inner();
            let raised = Profile {
                plane: Frame::new(
                    profile.plane.origin() + normal * height,
                    profile.plane.z().into_inner(),
                    profile.plane.x().into_inner(),
                )
                .map_err(fail)?,
                outer: profile.outer.clone(),
                holes: profile.holes.clone(),
            };
            let mut m = Model::default();
            let lofted = build(&mut m, LoftKind::Ruled, &[profile.clone(), raised])?;
            check_green(&m, lofted)?;
            let (extruded, _) = extrude(&mut m, &profile, normal, height, &Control::NONE)
                .map_err(|e| fail(format!("extrude: {e}")))?;
            let a = mass_properties(&m, lofted).map_err(fail)?;
            let b = mass_properties(&m, extruded).map_err(fail)?;
            let size = real::cbrt(b.volume);
            prop_assert!((a.volume - b.volume).abs() <= EXACT_REL * b.volume, "volume {} vs {}", a.volume, b.volume);
            prop_assert!((a.area - b.area).abs() <= EXACT_REL * b.area, "area {} vs {}", a.area, b.area);
            prop_assert!(
                (a.centroid - b.centroid).norm() <= EXACT_REL * size.max(1.0),
                "centroid {:?} vs {:?}", a.centroid, b.centroid
            );
            Ok(())
        }
}

prop_shards! {
    /// Sections of one shape scaled evenly up the axis loft to the
    /// frustum between the first and the last, whatever lies between:
    /// `h/3 (A1 + A2 + sqrt(A1 A2))`.
    similar_coaxial_sections_meet_the_frustum_formula
        [shard_0 shard_1 shard_2 shard_3]
        ((section, scale, height, fraction)) = (
            any_section(),
            prop::finite_f64(0.3..=2.5),
            prop::finite_f64(1.0..=6.0),
            prop::finite_f64(0.2..=0.8),
        ) => {
            let middle = 1.0 + (scale - 1.0) * fraction;
            let sections = [
                section.at(0.0),
                section.scaled(middle).at(height * fraction),
                section.scaled(scale).at(height),
            ];
            let mut m = Model::default();
            let body = build(&mut m, LoftKind::Ruled, &sections)?;
            check_green(&m, body)?;
            let (a1, a2) = (section.area(), section.scaled(scale).area());
            let expected = height / 3.0 * (a1 + a2 + (a1 * a2).sqrt());
            let found = mass_properties(&m, body).map_err(fail)?.volume;
            prop_assert!(
                (found - expected).abs() <= EXACT_REL * expected,
                "volume {found} vs the frustum's {expected}"
            );
            Ok(())
        }
}
