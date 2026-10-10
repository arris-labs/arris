//! `query::distance` between random solids (ADR-0054): a box, a cylinder
//! or a quadric solid (a frustum, a ball, a ring, an elliptic prism), each
//! in a random pose, about half of them near enough to overlap.
//!
//! The distance is symmetric and its feet are on their bodies and as far
//! apart as it says; it is zero exactly where the boolean `common` finds
//! material (or the operands touch, which `common` calls `Empty`); it is
//! the closed form for a ball against a ball and against a box; and moving
//! `b` toward `a` along the feet's line by `δ` shortens it by `δ`. The last
//! holds for any bodies: the moved pair of feet bounds it above, and the
//! distance is 1-Lipschitz in a translation, below.
//!
//! A pair the query refuses as `Unsupported` (two off-axis cones, say) is
//! skipped: that is the designed refusal, pruned or not. Any other error is
//! a failure. A failure prints the shrunk case and the seed and becomes a
//! fixture under `tests/fixtures/regression/`
//! (`tests/fixtures/README.md` §Property-test failures).

use arris_check::classify::{Classification, classify_point};
use arris_debug::prop::body::{
    Boxed, Cylindrical, QuadricSolid, box_in, cylinder_in, quadric_solid,
};
use arris_debug::testing::fail;
use arris_debug::unmetered::{common, transform};
use arris_debug::{prop, prop_shards};
use arris_math::{Control, Isometry, Point3, Vec3};
use arris_ops::query::{Distance, Operand, distance};
use arris_ops::{BooleanReason, OpError, Reason};
use arris_topo::{Body, Model, Shape};
use proptest::prelude::*;

/// What the distance is held to: the extrema are certified to far below
/// this, and the classifier's own reach is the model's tolerance.
const ABS: f64 = 1e-6;

/// A solid and the motion that poses it.
#[derive(Debug, Clone, Copy)]
enum Solid {
    Box(Boxed),
    Cylinder(Cylindrical),
    Quadric(QuadricSolid, Isometry),
}

impl Solid {
    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match self {
            Solid::Box(b) => b.build(m),
            Solid::Cylinder(c) => c.build(m),
            Solid::Quadric(q, pose) => {
                let body = q.build(m)?;
                Ok(transform(m, body, pose)?.0)
            }
        }
    }

    /// The same solid after `motion`, as a description to build afresh.
    fn moved(&self, motion: &Isometry) -> Solid {
        match *self {
            Solid::Box(b) => Solid::Box(Boxed {
                pose: b.pose.then(motion),
                ..b
            }),
            Solid::Cylinder(c) => Solid::Cylinder(Cylindrical {
                pose: c.pose.then(motion),
                ..c
            }),
            Solid::Quadric(q, pose) => Solid::Quadric(q, pose.then(motion)),
        }
    }
}

fn whole(b: Body) -> Operand {
    Operand::Shape(Shape::new(b.id, b.orientation))
}

fn solid_at(scale: f64) -> BoxedStrategy<Solid> {
    prop_oneof![
        box_in(scale).prop_map(Solid::Box),
        cylinder_in(scale).prop_map(Solid::Cylinder),
        (quadric_solid(), prop::pose_in(scale)).prop_map(|(q, pose)| Solid::Quadric(q, pose)),
    ]
    .boxed()
}

/// Two solids in poses within one scale: near enough to overlap and far
/// enough to be apart.
fn pair() -> impl Strategy<Value = (Solid, Solid)> {
    prop_oneof![
        2 => (solid_at(10.0), solid_at(10.0)),
        1 => (solid_at(25.0), solid_at(25.0)),
        1 => (solid_at(60.0), solid_at(60.0)),
    ]
}

/// The distance between two bodies, or `None` for the designed refusal.
fn measure(m: &Model, a: Body, b: Body) -> Result<Option<Distance>, TestCaseError> {
    match distance(m, whole(a), whole(b), &Control::NONE) {
        Ok(d) => Ok(Some(d)),
        Err(OpError::Unsupported { .. }) => Ok(None),
        Err(other) => Err(fail(other)),
    }
}

prop_shards! {
    /// Swapping the operands keeps the distance and `contained`, and each
    /// foot is on its body with the feet the distance apart.
    the_distance_is_symmetric_and_its_feet_are_on_the_bodies
        [shard_0 shard_1 shard_2 shard_3]
        (case) = pair() => {
            let (sa, sb) = case;
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let (Some(ab), Some(ba)) = (measure(&m, a, b)?, measure(&m, b, a)?) else {
                return Ok(());
            };
            prop_assert!(
                (ab.distance - ba.distance).abs() <= ABS,
                "a to b {} but b to a {}", ab.distance, ba.distance
            );
            prop_assert_eq!(ab.contained, ba.contained);
            if ab.contained {
                prop_assert_eq!(ab.distance, 0.0);
                return Ok(());
            }
            for (d, from, to) in [(&ab, a, b), (&ba, b, a)] {
                prop_assert!(
                    ((d.a.point - d.b.point).norm() - d.distance).abs() <= ABS,
                    "feet {:?} and {:?} are not {} apart", d.a.point, d.b.point, d.distance
                );
                for (foot, body) in [(&d.a, from), (&d.b, to)] {
                    prop_assert!(foot.shape.is_some(), "a body's foot names its entity");
                    let there = classify_point(&m, body, foot.point).map_err(fail)?;
                    prop_assert!(
                        matches!(there, Classification::On(_)),
                        "foot {:?} classifies as {there:?}", foot.point
                    );
                }
            }
            Ok(())
        }
}

prop_shards! {
    /// Material in common means distance zero, and a positive distance
    /// means `common` is `Empty`; one operand inside the other is
    /// `contained`. Where the two only touch, `common` is `Empty` and the
    /// distance zero, which neither side forbids.
    the_distance_is_zero_where_the_operands_share_material
        [shard_0 shard_1 shard_2 shard_3]
        (case) = pair() => {
            let (sa, sb) = case;
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let Some(d) = measure(&m, a, b)? else { return Ok(()); };
            let tol = m.precision().default_tolerance;
            match common(&mut m, a, b) {
                Ok(_) => prop_assert!(
                    d.distance <= 10.0 * tol,
                    "they share material but are {} apart", d.distance
                ),
                Err(OpError::Degenerate {
                    reason: Reason::Boolean(BooleanReason::Empty), ..
                }) => {
                    prop_assert!(!d.contained, "contained, yet `common` is empty");
                }
                // Any other refusal is the boolean's to answer for.
                Err(_) => return Ok(()),
            }
            if d.distance > 10.0 * tol {
                prop_assert!(!d.contained);
            }
            Ok(())
        }
}

/// A ball of `radius` at `centre` for the closed forms, and the pose that
/// puts it there.
fn ball(radius: f64, centre: Vec3) -> Solid {
    Solid::Quadric(
        QuadricSolid::Ball { radius },
        Isometry::from_translation(centre),
    )
}

prop_shards! {
    /// Ball against ball is the gap between the surfaces, or zero.
    sphere_against_sphere_is_the_gap_between_the_surfaces
        [shard_0 shard_1 shard_2 shard_3]
        (case) = (
            prop::radius(1.0..=15.0), prop::radius(1.0..=15.0),
            prop::point_in_box(40.0), prop::point_in_box(40.0),
        ) => {
            let (r1, r2, c1, c2) = case;
            let mut m = Model::default();
            let a = ball(r1, c1.coords).build(&mut m).map_err(fail)?;
            let b = ball(r2, c2.coords).build(&mut m).map_err(fail)?;
            let d = measure(&m, a, b)?.ok_or_else(|| fail("a ball against a ball is refused"))?;
            let want = ((c1 - c2).norm() - r1 - r2).max(0.0);
            prop_assert!(
                (d.distance - want).abs() <= ABS,
                "balls {r1} at {c1:?} and {r2} at {c2:?}: {} against {want}", d.distance
            );
            Ok(())
        }
}

prop_shards! {
    /// Ball against a posed box is the centre's distance to the box less
    /// the radius, or zero.
    sphere_against_box_is_the_centres_distance_less_the_radius
        [shard_0 shard_1 shard_2 shard_3]
        (case) = (
            prop::radius(1.0..=15.0), prop::point_in_box(40.0), box_in(30.0),
        ) => {
            let (r, c, cuboid) = case;
            let mut m = Model::default();
            let a = ball(r, c.coords).build(&mut m).map_err(fail)?;
            let b = cuboid.build(&mut m).map_err(fail)?;
            let d = measure(&m, a, b)?.ok_or_else(|| fail("a ball against a box is refused"))?;
            // The centre in the box's own frame, clamped into the box.
            let local = cuboid.pose.inverse().apply(c);
            let clamped = Point3::new(
                local.x.clamp(cuboid.min.x, cuboid.max.x),
                local.y.clamp(cuboid.min.y, cuboid.max.y),
                local.z.clamp(cuboid.min.z, cuboid.max.z),
            );
            let want = ((local - clamped).norm() - r).max(0.0);
            prop_assert!(
                (d.distance - want).abs() <= ABS,
                "ball {r} at {c:?} against {cuboid:?}: {} against {want}", d.distance
            );
            Ok(())
        }
}

prop_shards! {
    /// Moving `b` toward `a` along the line of the feet by a part of the
    /// distance shortens it by exactly that.
    moving_toward_along_the_feet_shortens_the_distance_by_the_move
        [shard_0 shard_1 shard_2 shard_3]
        (case) = (pair(), prop::finite_f64(0.1..=0.9)) => {
            let ((sa, sb), part) = case;
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let Some(d) = measure(&m, a, b)? else { return Ok(()); };
            // Touching or overlapping pairs have no line to move along.
            if d.contained || d.distance <= 1e-3 {
                return Ok(());
            }
            let toward = (d.a.point - d.b.point) / d.distance;
            let step = part * d.distance;
            let moved = sb.moved(&Isometry::from_translation(toward * step));
            let b2 = moved.build(&mut m).map_err(fail)?;
            let Some(after) = measure(&m, a, b2)? else { return Ok(()); };
            prop_assert!(
                (after.distance - (d.distance - step)).abs() <= ABS,
                "{} moved by {step} along the feet is {}, not {}",
                d.distance, after.distance, d.distance - step
            );
            Ok(())
        }
}
