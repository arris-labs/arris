//! `query::ray_fire` at random (ADR-0053): a box, a cylinder or a quadric
//! solid (a frustum, a ball, a ring, an elliptic prism) in a random pose,
//! fired at from outside and from inside along a random direction through
//! a point of its interior.
//!
//! Each hit lies on the ray (`origin + t · direction`) and on the body
//! (the classifier calls it `On`); hits ascend in `t`; `First` is the
//! first of `All`; a ray from outside a closed solid crosses an even
//! number of times and one from inside an odd number, which agrees with
//! `classify_point`; and a ray fired back from past the last hit returns
//! the same hits reversed.
//!
//! A ray that lands on an edge or a vertex, grazes, lies in a face or goes
//! through a pole is measure zero for a random aim. Its parity is not
//! counted (a ray may clip an edge and leave by it), and a refusal that
//! names the face (`RayInFace`, `Singular`) is allowed; a kernel fault, or
//! any other refusal, never is. A failure prints the shrunk case and the
//! seed and becomes a fixture under `tests/fixtures/regression/`
//! (`tests/fixtures/README.md` §Property-test failures).

use arris_check::classify::{Classification, classify_point};
use arris_debug::prop::body::{
    Boxed, Cylindrical, QuadricSolid, box_in, cylinder_in, quadric_solid,
};
use arris_debug::testing::fail;
use arris_debug::unmetered::transform;
use arris_debug::{prop, prop_shards};
use arris_math::{Isometry, Point3, UnitVec3, Vec3};
use arris_ops::query::{Ray, RayHit, RayHits, ray_fire};
use arris_ops::{OpError, QueryReason, Reason};
use arris_topo::{Body, EntityId, Model};
use proptest::prelude::*;

/// What is fired at.
#[derive(Debug, Clone, Copy)]
enum Solid {
    Box(Boxed),
    Cylinder(Cylindrical),
    /// A quadric solid and the motion that poses it.
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

    /// A point strictly inside the solid, each of `(s, t, w)` in the unit
    /// cube, and the radius of a ball about the solid's `centre` holding
    /// it.
    fn interior(&self, (s, t, w): (f64, f64, f64)) -> (Point3, f64) {
        let turn = core::f64::consts::TAU * t;
        match self {
            Solid::Box(b) => {
                let extent = b.max - b.min;
                let local = b.min + Vec3::new(s, t, w).component_mul(&extent) * 0.8 + extent * 0.1;
                (b.pose.apply(local), extent.norm())
            }
            Solid::Cylinder(c) => {
                let r = 0.8 * s * c.radius;
                let local = Point3::new(
                    r * turn.cos(),
                    r * turn.sin(),
                    c.axis.origin.z + (0.1 + 0.8 * w) * c.height,
                );
                (c.pose.apply(local), c.radius.hypot(c.height) * 2.0)
            }
            Solid::Quadric(q, pose) => (pose.apply(q.interior(s, t, w)), q.reach() * 2.0),
        }
    }

    /// Where the solid's own origin went: the centre of the ball
    /// [`Solid::interior`] bounds it by.
    fn centre(&self) -> Point3 {
        let pose = match self {
            Solid::Box(b) => &b.pose,
            Solid::Cylinder(c) => &c.pose,
            Solid::Quadric(_, pose) => pose,
        };
        pose.apply(Point3::origin())
    }
}

/// A ray through a point of the solid, from outside or from that point.
#[derive(Debug, Clone, Copy)]
struct Case {
    solid: Solid,
    aim: (f64, f64, f64),
    direction: UnitVec3,
    inside: bool,
}

fn case() -> impl Strategy<Value = Case> {
    let solid = prop_oneof![
        box_in(prop::DEFAULT_SCALE).prop_map(Solid::Box),
        cylinder_in(prop::DEFAULT_SCALE).prop_map(Solid::Cylinder),
        (quadric_solid(), prop::pose()).prop_map(|(q, pose)| Solid::Quadric(q, pose)),
    ];
    let unit = || prop::finite_f64(0.0..=1.0);
    (
        solid,
        (unit(), unit(), unit()),
        prop::unit_vec3(),
        any::<bool>(),
    )
        .prop_map(|(solid, aim, direction, inside)| Case {
            solid,
            aim,
            direction,
            inside,
        })
}

/// The hits of `ray`, or `None` when the ray is one the query refuses by
/// design; any other error fails the case.
fn fire(
    m: &Model,
    body: Body,
    ray: &Ray,
    which: RayHits,
) -> Result<Option<Vec<RayHit>>, TestCaseError> {
    match ray_fire(m, body, ray, which) {
        Ok(hits) => Ok(Some(hits)),
        Err(OpError::Degenerate {
            reason: Reason::Query(QueryReason::RayInFace | QueryReason::Singular),
            ..
        }) => Ok(None),
        Err(other) => Err(fail(other)),
    }
}

prop_shards! {
    /// Hits are on the ray and on the body, ascending; `First` is the
    /// first of `All`; the crossings' parity follows the origin's side
    /// and agrees with the classifier.
    hits_are_on_the_ray_and_the_body_and_cross_with_the_right_parity
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let mut m = Model::default();
            let body = case.solid.build(&mut m).map_err(fail)?;
            let tol = m.precision().default_tolerance;
            let (target, reach) = case.solid.interior(case.aim);
            let d = case.direction;
            let origin = if case.inside {
                target
            } else {
                // Past the ball holding the solid, so outside it.
                target - d.into_inner() * (reach + (target - case.solid.centre()).norm() + 1.0)
            };
            let class = classify_point(&m, body, origin).map_err(fail)?;
            if case.inside {
                prop_assert_eq!(class, Classification::Inside, "the aim point is interior");
            } else {
                prop_assert_eq!(class, Classification::Outside, "the origin is past the solid");
            }

            let ray = Ray { origin, direction: d.into_inner() * 3.7 };
            let Some(hits) = fire(&m, body, &ray, RayHits::All)? else { return Ok(()); };

            let mut last = 0.0f64;
            for hit in &hits {
                prop_assert!(hit.t >= last, "hits ascend: {} after {last}", hit.t);
                last = hit.t;
                let along = origin + d.into_inner() * hit.t;
                prop_assert!(
                    (along - hit.point).norm() <= 10.0 * tol,
                    "point {:?} is {:?} on the ray",
                    hit.point,
                    along,
                );
                let there = classify_point(&m, body, hit.point).map_err(fail)?;
                prop_assert!(
                    matches!(there, Classification::On(_)),
                    "hit at t = {} classifies as {there:?}",
                    hit.t,
                );
            }

            let Some(first) = fire(&m, body, &ray, RayHits::First)? else { return Ok(()); };
            prop_assert_eq!(first.len(), hits.len().min(1));
            if let (Some(f), Some(h)) = (first.first(), hits.first()) {
                prop_assert_eq!(f.shape, h.shape);
                prop_assert!((f.t - h.t).abs() <= tol, "First is {} and All's first {}", f.t, h.t);
            }

            // Parity only where every hit is a plain crossing of a face.
            let plain = hits
                .iter()
                .all(|h| !h.tangent && matches!(h.shape.id, EntityId::Face(_)));
            if plain {
                let crossings = hits.len();
                if case.inside {
                    prop_assert!(crossings % 2 == 1, "{crossings} crossings from inside");
                } else {
                    prop_assert!(crossings % 2 == 0, "{crossings} crossings from outside");
                }
            }
            Ok(())
        }
}

prop_shards! {
    /// A ray fired back from past the last hit returns the same entities
    /// at the mirrored distances.
    a_ray_fired_back_returns_the_hits_reversed
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let mut m = Model::default();
            let body = case.solid.build(&mut m).map_err(fail)?;
            let tol = m.precision().default_tolerance;
            let (target, reach) = case.solid.interior(case.aim);
            let d = case.direction.into_inner();
            let origin = target - d * (reach + (target - case.solid.centre()).norm() + 1.0);

            let Some(there) = fire(&m, body, &Ray { origin, direction: d }, RayHits::All)?
            else { return Ok(()); };
            let Some(last) = there.last() else { return Ok(()); };
            if there.iter().any(|h| h.tangent) {
                return Ok(());
            }
            let start = last.t + 1.0;
            let turn = Ray { origin: origin + d * start, direction: -d };
            let Some(back) = fire(&m, body, &turn, RayHits::All)? else { return Ok(()); };

            prop_assert_eq!(back.len(), there.len(), "{:?} there, {:?} back", there, back);
            for (a, b) in there.iter().rev().zip(&back) {
                prop_assert_eq!(a.shape.id, b.shape.id);
                prop_assert!(
                    (start - a.t - b.t).abs() <= 10.0 * tol,
                    "t = {} there is {} back from {start}",
                    a.t,
                    b.t,
                );
            }
            Ok(())
        }
}
