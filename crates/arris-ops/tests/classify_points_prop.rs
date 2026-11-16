//! `query::classify_points` (C12): the batch
//! answers each point as `classify_point` does, in the order given, over
//! random boxes and cylinders in random poses. The probes are interior,
//! exterior, and on faces, edges and vertices of the body (a box's
//! corners and rims, a cylinder's caps and rims), so every answer kind
//! and every boundary entity is reached.
//!
//! A point no ray decides is `None` in the batch and `Undecided` from
//! `classify_point`; that pairing is held too. Any other error is a
//! failure, printed with the seed and shrunk as the other property tests.

use std::f64::consts::TAU;

use arris_check::classify::{ClassifyError, classify_point};
use arris_debug::prop::body::{Boxed, Cylindrical, box_in, cylinder_in};
use arris_debug::prop_shards;
use arris_debug::testing::fail;
use arris_math::{Control, Interrupted, Isometry, Point3, Stop, real};
use arris_ops::query::classify_points;
use arris_ops::{InputReason, OpError, Reason, primitive_box};
use arris_topo::{Body, Model};
use proptest::prelude::*;

/// A solid, described by its numbers and its pose.
#[derive(Debug, Clone, Copy)]
enum Solid {
    Box(Boxed),
    Cylinder(Cylindrical),
}

impl Solid {
    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match self {
            Solid::Box(b) => b.build(m),
            Solid::Cylinder(c) => c.build(m),
        }
    }

    fn pose(&self) -> Isometry {
        match self {
            Solid::Box(b) => b.pose,
            Solid::Cylinder(c) => c.pose,
        }
    }

    /// A probe in the solid's own frame, before its pose.
    fn local(&self, raw: &Raw) -> Point3 {
        match self {
            Solid::Box(b) => {
                let mut p = [0.0; 3];
                for (k, coord) in p.iter_mut().enumerate() {
                    let (lo, hi) = (b.min.coords[k], b.max.coords[k]);
                    *coord = match raw.snap[k] {
                        1 => lo,
                        2 => hi,
                        _ => lo + raw.f[k] * (hi - lo),
                    };
                }
                Point3::new(p[0], p[1], p[2])
            }
            Solid::Cylinder(c) => {
                let (r, h) = (c.radius, c.height);
                // The axis is z, its origin the base cap's centre.
                let s = if raw.snap[0] == 2 { 1.0 } else { raw.f[0] };
                let z = match raw.snap[2] {
                    1 => -h / 2.0,
                    2 => h / 2.0,
                    _ => -h / 2.0 + raw.f[2] * h,
                };
                Point3::new(
                    s * r * real::cos(raw.theta),
                    s * r * real::sin(raw.theta),
                    z,
                )
            }
        }
    }
}

/// One probe before it meets a solid: fractions along each axis, a snap
/// per axis (0 keeps the fraction, 1 is the low side, 2 the high one), and
/// an angle for a cylinder's wall.
#[derive(Debug, Clone, Copy)]
struct Raw {
    f: [f64; 3],
    snap: [u8; 3],
    theta: f64,
}

fn raw() -> impl Strategy<Value = Raw> {
    (
        proptest::array::uniform3(-0.3..1.3f64),
        proptest::array::uniform3(0u8..3),
        0.0..TAU,
    )
        .prop_map(|(f, snap, theta)| Raw { f, snap, theta })
}

fn solid() -> impl Strategy<Value = Solid> {
    prop_oneof![
        box_in(10.0).prop_map(Solid::Box),
        cylinder_in(10.0).prop_map(Solid::Cylinder),
    ]
}

prop_shards! {
    /// The batch's `i`-th answer is `classify_point`'s answer for the
    /// `i`-th point, and an undecided point is `None` in the batch.
    the_batch_answers_each_point_as_classify_point_does
        [shard_0 shard_1 shard_2 shard_3]
        (case) = (solid(), proptest::collection::vec(raw(), 1..=24)) => {
            let (s, raws) = case;
            let mut m = Model::default();
            let body = s.build(&mut m).map_err(fail)?;
            let pose = s.pose();
            let points: Vec<Point3> = raws.iter().map(|r| pose.apply(s.local(r))).collect();
            let batch = classify_points(&m, body, &points, &Control::NONE).map_err(fail)?;
            prop_assert_eq!(batch.len(), points.len());
            for (i, &p) in points.iter().enumerate() {
                match classify_point(&m, body, p) {
                    Ok(one) => prop_assert_eq!(
                        batch[i], Some(one),
                        "point {} at {:?} of {:?}", i, p, s
                    ),
                    Err(ClassifyError::Undecided { .. }) => prop_assert_eq!(
                        batch[i], None,
                        "point {} at {:?} of {:?} is undecided", i, p, s
                    ),
                    Err(other) => return Err(fail(other)),
                }
            }
            Ok(())
        }
}

/// A unit box at the origin, the points of a test below.
fn unit_box(m: &mut Model) -> Body {
    let none = Control::NONE;
    primitive_box(m, Point3::origin(), Point3::new(2.0, 2.0, 2.0), &none)
        .expect("a box builds")
        .0
}

#[test]
fn a_budget_stops_after_that_many_points() {
    let mut m = Model::default();
    let body = unit_box(&mut m);
    let inside = Point3::new(1.0, 1.0, 1.0);
    let points = [inside, inside, inside];
    let err = classify_points(&m, body, &points, &Control::budget(2)).unwrap_err();
    assert!(
        matches!(
            err,
            OpError::Interrupted(Interrupted {
                by: Stop::Budget,
                steps: 2
            })
        ),
        "{err:?}"
    );
    let all = classify_points(&m, body, &points, &Control::budget(3)).expect("3 points, budget 3");
    assert_eq!(all.len(), 3);
}

#[test]
fn a_non_finite_point_fails_the_call() {
    let mut m = Model::default();
    let body = unit_box(&mut m);
    let points = [Point3::new(1.0, 1.0, 1.0), Point3::new(f64::NAN, 0.0, 0.0)];
    let err = classify_points(&m, body, &points, &Control::NONE).unwrap_err();
    assert!(
        matches!(
            err,
            OpError::Degenerate {
                reason: Reason::Input(InputReason::NonFinite { .. }),
                ..
            }
        ),
        "{err:?}"
    );
}

#[test]
fn a_sheet_is_refused_as_not_a_solid() {
    use arris_debug::sample;
    use arris_geom::Surface;
    use arris_math::{Frame, Interval};
    let mut m = Model::default();
    let sphere = Surface::Sphere {
        frame: Frame::world(),
        radius: 2.0,
    };
    let (u, v) = (
        Interval::new(0.2, 1.4).unwrap(),
        Interval::new(-0.5, 0.9).unwrap(),
    );
    let sheet = sample::patch(&mut m, sphere, u, v).expect("a patch builds");
    let err = classify_points(&m, sheet, &[Point3::origin()], &Control::NONE).unwrap_err();
    assert!(
        matches!(
            err,
            OpError::Degenerate {
                reason: Reason::Input(InputReason::NotSolid),
                ..
            }
        ),
        "{err:?}"
    );
}
