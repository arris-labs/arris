//! `query::interference` between random solids (ADR-0055): a box, a
//! cylinder or a quadric solid in a random pose, about half near enough to
//! overlap.
//!
//! The verdict agrees with `distance` (apart exactly beyond the model's
//! linear tolerance); an overlapping pair's volume is `common`'s on a
//! scratch copy, bit for bit, and `vol(a) = volume + vol(cut(a, b))`;
//! swapping the operands swaps the containment flags and keeps the rest;
//! translating both keeps the verdict; and the model is never written.
//!
//! A pair either query refuses (`Unsupported`, a boolean's `Degenerate`
//! such as a tangent contact) is skipped: that is the designed refusal.
//! `Internal` and `InvalidInput` are failures. A failure prints the shrunk
//! case and the seed and becomes a fixture under `tests/fixtures/regression/`
//! (`tests/fixtures/README.md` §Property-test failures).

use arris_debug::dump::dump_text;
use arris_debug::prop::body::{
    Boxed, Cylindrical, QuadricSolid, box_in, cylinder_in, quadric_solid,
};
use arris_debug::testing::{close_to, fail};
use arris_debug::unmetered::{common, cut, mass_properties, transform};
use arris_debug::{prop, prop_shards};
use arris_math::{Control, Isometry, Vec3};
use arris_ops::OpError;
use arris_ops::query::{Contact, Interference, Operand, distance, interference, interference_many};
use arris_topo::{Body, Model, Shape};
use proptest::prelude::*;

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

fn whole(b: Body) -> Operand {
    Operand::Shape(Shape::new(b.id, b.orientation))
}

/// The verdict on `a` and `b`, or `None` for a refusal the kernel owes a
/// random pair.
fn judge(m: &Model, a: Body, b: Body) -> Result<Option<Interference>, TestCaseError> {
    match interference(m, a, b, &Control::NONE) {
        Ok(i) => Ok(Some(i)),
        Err(OpError::Internal(_) | OpError::InvalidInput { .. }) => Err(fail("a fault")),
        Err(_) => Ok(None),
    }
}

/// The kind of verdict, without its payload.
fn kind(i: &Interference) -> u8 {
    match i {
        Interference::Apart(_) => 0,
        Interference::Touching { .. } => 1,
        Interference::Overlapping { .. } => 2,
    }
}

fn contact_of(i: &Interference) -> Option<Contact> {
    match i {
        Interference::Touching { contact, .. } => Some(*contact),
        _ => None,
    }
}

prop_shards! {
    /// Apart exactly when `distance` exceeds the linear tolerance, with
    /// that distance.
    the_verdict_agrees_with_the_distance
        [shard_0 shard_1 shard_2 shard_3]
        (case) = pair() => {
            let (sa, sb) = case;
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let Ok(d) = distance(&m, whole(a), whole(b), &Control::NONE) else {
                return Ok(());
            };
            let Some(verdict) = judge(&m, a, b)? else { return Ok(()); };
            let linear = m.precision().tolerance().linear;
            match verdict {
                Interference::Apart(apart) => {
                    prop_assert!(d.distance > linear, "apart but the distance is {}", d.distance);
                    prop_assert_eq!(apart.distance, d.distance);
                }
                _ => prop_assert!(
                    d.distance <= linear,
                    "not apart but the distance is {}", d.distance
                ),
            }
            Ok(())
        }
}

prop_shards! {
    /// The overlap's volume is `common`'s, measured on a scratch copy bit
    /// for bit, and the operand's volume less the cut's is the same
    /// volume.
    the_volume_is_commons_and_completes_the_cut
        [shard_0 shard_1 shard_2 shard_3]
        (case) = pair() => {
            let (sa, sb) = case;
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let Some(Interference::Overlapping { volume, .. }) = judge(&m, a, b)? else {
                return Ok(());
            };
            let mut scratch = m.clone();
            let Ok((shared, _)) = common(&mut scratch, a, b) else { return Ok(()); };
            let measured = mass_properties(&scratch, shared).map_err(fail)?;
            prop_assert_eq!(volume, measured.volume);
            let whole_a = mass_properties(&m, a).map_err(fail)?.volume;
            let left = match cut(&mut scratch, a, b) {
                Ok((rest, _)) => mass_properties(&scratch, rest).map_err(fail)?.volume,
                // `a` lies wholly inside `b`: nothing is left.
                Err(OpError::Degenerate { .. }) => 0.0,
                Err(_) => return Ok(()),
            };
            prop_assert!(
                close_to(whole_a, volume + left, 1.0, 1e-6),
                "vol(a) {whole_a} but the overlap {volume} and the rest {left}"
            );
            Ok(())
        }
}

prop_shards! {
    /// Swapping the operands swaps the containment flags and keeps the
    /// verdict, the volume and the contact.
    swapping_the_operands_swaps_the_flags
        [shard_0 shard_1 shard_2 shard_3]
        (case) = pair() => {
            let (sa, sb) = case;
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let (Some(ab), Some(ba)) = (judge(&m, a, b)?, judge(&m, b, a)?) else {
                return Ok(());
            };
            prop_assert_eq!(kind(&ab), kind(&ba));
            prop_assert_eq!(contact_of(&ab), contact_of(&ba));
            if let (
                Interference::Overlapping { volume: v1, a_in_b: a1, b_in_a: b1 },
                Interference::Overlapping { volume: v2, a_in_b: a2, b_in_a: b2 },
            ) = (ab, ba)
            {
                prop_assert!(close_to(v1, v2, 1.0, 1e-6), "volume {v1} against {v2}");
                prop_assert_eq!((a1, b1), (b2, a2));
            }
            Ok(())
        }
}

prop_shards! {
    /// Translating both solids keeps the verdict, the contact and the
    /// containment.
    translating_both_keeps_the_verdict
        [shard_0 shard_1 shard_2 shard_3]
        (case) = (pair(), prop::point_in_box(20.0)) => {
            let ((sa, sb), by) = case;
            let motion = Isometry::from_translation(Vec3::new(by.x, by.y, by.z));
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let a2 = sa.moved(&motion).build(&mut m).map_err(fail)?;
            let b2 = sb.moved(&motion).build(&mut m).map_err(fail)?;
            let (Some(first), Some(second)) = (judge(&m, a, b)?, judge(&m, a2, b2)?) else {
                return Ok(());
            };
            prop_assert_eq!(kind(&first), kind(&second));
            prop_assert_eq!(contact_of(&first), contact_of(&second));
            if let (
                Interference::Overlapping { volume: v1, a_in_b: a1, b_in_a: b1 },
                Interference::Overlapping { volume: v2, a_in_b: a2, b_in_a: b2 },
            ) = (first, second)
            {
                prop_assert!(close_to(v1, v2, 1.0, 1e-6), "volume {v1} against {v2}");
                prop_assert_eq!((a1, b1), (a2, b2));
            }
            Ok(())
        }
}

prop_shards! {
    /// The model is not written: both bodies dump the same before and
    /// after.
    the_model_is_never_written
        [shard_0 shard_1 shard_2 shard_3]
        (case) = pair() => {
            let (sa, sb) = case;
            let mut m = Model::default();
            let a = sa.build(&mut m).map_err(fail)?;
            let b = sb.build(&mut m).map_err(fail)?;
            let before = (dump_text(&m, a).map_err(fail)?, dump_text(&m, b).map_err(fail)?);
            let _ = judge(&m, a, b)?;
            let after = (dump_text(&m, a).map_err(fail)?, dump_text(&m, b).map_err(fail)?);
            prop_assert!(before == after, "the dumps changed");
            Ok(())
        }
}

prop_shards! {
    /// The screen of an assembly is the brute-force loop over every pair:
    /// the same verdicts, the same ascending pairs, apart ones left out.
    the_screen_is_the_brute_force_loop
        [shard_0 shard_1 shard_2 shard_3]
        (case) = proptest::collection::vec(solid_at(25.0), 2..=5) => {
            let mut m = Model::default();
            let mut bodies = Vec::new();
            for s in &case {
                bodies.push(s.build(&mut m).map_err(fail)?);
            }
            let mut brute = Vec::new();
            for i in 0..bodies.len() {
                for j in i + 1..bodies.len() {
                    match judge(&m, bodies[i], bodies[j])? {
                        Some(Interference::Apart(_)) => {}
                        Some(verdict) => brute.push(([i, j], verdict)),
                        // A refused pair: the screen may refuse too.
                        None => return Ok(()),
                    }
                }
            }
            let screen = interference_many(&m, &bodies, &Control::NONE).map_err(fail)?;
            let found: Vec<([usize; 2], Interference)> =
                screen.iter().map(|c| (c.operands, c.verdict)).collect();
            prop_assert_eq!(found, brute);
            Ok(())
        }
}
