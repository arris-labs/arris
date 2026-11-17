//! Solids that share a plane (the retired flat-edge-read-as-tangent-contact plan):
//! three to five boxes and cylinders standing on the plane z = 0, half of
//! them on whole numbers so bases, tops and walls come out flush.
//! `fuse_many` and the chain of `fuse`s agree in volume and area, or both
//! refuse; a refusal is a named one, never a kernel fault. A random box
//! split by a random plane and fused back has `interference` with the whole
//! box equal to its volume, each inside the other.
//!
//! The first property holds the image of an edge shared by two coplanar
//! faces of one operand, placed on the other operand's face once; the
//! second holds a body against the same solid split another way. A
//! failure prints the shrunk case and the seed and becomes a fixture under
//! `tests/fixtures/regression/` (`tests/fixtures/README.md` §Property-test
//! failures).

use arris_check::{Level, check};
use arris_debug::prop;
use arris_debug::testing::{REL, close_to, fail, fitted_rel};
use arris_debug::unmetered::{fuse, fuse_many, mass_properties, primitive_box, split, transform};
use arris_math::{Control, Frame, Isometry, Point3};
use arris_ops::OpError;
use arris_ops::query::{Interference, interference};
use arris_topo::provenance::audit;
use arris_topo::{Body, Model, Provenance};
use proptest::prelude::*;

/// What an operation returns.
type Made = Result<(Body, Provenance), OpError>;

/// A refusal the kernel owes a random input: not a fault, not an input
/// that failed the checker.
fn is_refusal(e: &OpError) -> bool {
    !matches!(e, OpError::Internal(_) | OpError::InvalidInput { .. })
}

/// `result`'s body, clean at `Full` with nothing unchecked and its
/// provenance audited against `inputs`; `None` for a named refusal.
fn settle(
    m: &Model,
    name: &str,
    inputs: &[Body],
    result: Made,
) -> Result<Option<Body>, TestCaseError> {
    let (body, provenance) = match result {
        Ok(made) => made,
        Err(e) if is_refusal(&e) => return Ok(None),
        Err(e) => return Err(fail(format!("{name}: {e}"))),
    };
    let report = check(m, body, Level::Full);
    if !report.is_ok() || !report.unchecked().is_empty() {
        return Err(fail(format!("{name}: not clean at Full\n{report}")));
    }
    audit(m, inputs, body, &provenance).map_err(|e| fail(format!("{name}: provenance: {e}")))?;
    Ok(Some(body))
}

/// The chain of `fuse`s over `bodies`: the first refusal or fault, else the
/// last result.
fn chain(m: &mut Model, bodies: &[Body]) -> Made {
    let mut made = Ok((bodies[0], Provenance::default()));
    let mut body = bodies[0];
    for &next in &bodies[1..] {
        made = fuse(m, body, next);
        match &made {
            Ok((b, _)) => body = *b,
            Err(_) => break,
        }
    }
    made
}

/// `fuse_many` of solids standing on one plane is the chain of `fuse`s: the
/// same volume and area, or both refuse.
///
/// Ignored: at 1000 cases it finds what the fixtures
/// `regression/two-boxes-on-one-plane-fuse-unsupported`,
/// `regression/chained-fuse-three-boxes-and-cylinder-unsupported` and
/// `regression/fuse-many-three-boxes-plane-against-plane-unsupported` hold
/// (it passes at 16). Their fix turns this into `prop_shards!` shards.
#[test]
#[ignore = "finds regression/two-boxes-on-one-plane-fuse-unsupported, chained-fuse-three-boxes-and-cylinder-unsupported and fuse-many-three-boxes-plane-against-plane-unsupported"]
fn the_chained_fuse_on_a_shared_plane_is_fuse_many() {
    prop::check(prop::body::standing(), |case| {
        let mut m = Model::default();
        let bodies = case.build(&mut m).map_err(fail)?;
        let many = fuse_many(&mut m, &bodies);
        let chained = chain(&mut m, &bodies);
        // The chain's record is its last step's, so its result is
        // checked clean and not audited against all the operands.
        let chained = match chained {
            Ok((b, _)) => {
                let report = check(&m, b, Level::Full);
                if !report.is_ok() || !report.unchecked().is_empty() {
                    return Err(fail(format!("the chain: not clean at Full\n{report}")));
                }
                Some(b)
            }
            Err(e) if is_refusal(&e) => None,
            Err(e) => return Err(fail(format!("the chain: {e}"))),
        };
        let many = settle(&m, "fuse_many", &bodies, many)?;
        match (many, chained) {
            (Some(many), Some(chained)) => {
                let (pa, pb) = (
                    mass_properties(&m, many).map_err(fail)?,
                    mass_properties(&m, chained).map_err(fail)?,
                );
                let rel = fitted_rel(&m, &pb).max(REL);
                prop_assert!(
                    close_to(pa.volume, pb.volume, pb.volume, rel),
                    "volume {} against the chain's {}",
                    pa.volume,
                    pb.volume
                );
                prop_assert!(
                    close_to(pa.area, pb.area, pb.area, rel),
                    "area {} against the chain's {}",
                    pa.area,
                    pb.area
                );
            }
            (None, None) => {}
            (Some(_), None) => return Err(fail("fuse_many builds where the chain refuses")),
            (None, Some(_)) => return Err(fail("the chain builds where fuse_many refuses")),
        }
        Ok(())
    });
}

/// A box split by a plane through its interior: the box's extents, the
/// plane's frame as the pose and a point of the box.
#[derive(Debug, Clone)]
struct Halved {
    extent: arris_math::Vec3,
    /// Where the plane's origin is, as fractions of the extents from the
    /// centre.
    at: [f64; 3],
    turn: arris_math::nalgebra::UnitQuaternion<f64>,
    pose: Isometry,
}

fn halved() -> impl Strategy<Value = Halved> {
    let side = || prop::finite_f64(1.0..=20.0);
    let fraction = || prop::finite_f64(-0.3..=0.3);
    (
        (side(), side(), side()),
        [fraction(), fraction(), fraction()],
        prop::rotation(),
        prop::pose(),
    )
        .prop_map(|((x, y, z), at, turn, pose)| Halved {
            extent: arris_math::Vec3::new(x, y, z),
            at,
            turn,
            pose,
        })
}

/// A box split by a plane and its two sides fused back is the same solid:
/// `interference` of the box against the fused body, both ways, answers
/// the box's volume, each inside the other. The fused body keeps the box's
/// untouched faces by id, so the two operands share entities
/// (`query/interference-box-against-its-split-and-fused-sides`). A draw
/// whose split or fuse is refused by name asks no `interference`; at
/// least half the draws must reach it.
#[test]
fn a_box_split_and_fused_back_overlaps_the_box_in_its_volume() {
    use std::cell::Cell;
    let (drawn, asked) = (Cell::new(0_usize), Cell::new(0_usize));
    prop::check(halved(), |case| {
        drawn.set(drawn.get() + 1);
        let mut m = Model::default();
        let e = case.extent;
        let (whole, _) =
            primitive_box(&mut m, Point3::from(-e / 2.0), Point3::from(e / 2.0)).map_err(fail)?;
        let whole = transform(&mut m, whole, &case.pose).map_err(fail)?.0;
        let origin = Point3::new(case.at[0] * e.x, case.at[1] * e.y, case.at[2] * e.z);
        let plane = Frame::from_rotation(origin, &case.turn);
        let plane = plane.transformed(&case.pose);
        let halves = match split(&mut m, whole, &plane) {
            Ok(h) => h,
            Err(e) if is_refusal(&e) => return Ok(()),
            Err(e) => return Err(fail(format!("split: {e}"))),
        };
        let fused = fuse(&mut m, halves.positive, halves.negative);
        let Some(back) = settle(
            &m,
            "fuse of the sides",
            &[halves.positive, halves.negative],
            fused,
        )?
        else {
            return Ok(());
        };
        asked.set(asked.get() + 1);
        let volume = mass_properties(&m, whole).map_err(fail)?.volume;
        for (name, a, b) in [
            ("whole against back", whole, back),
            ("back against whole", back, whole),
        ] {
            match interference(&m, a, b, &Control::NONE) {
                Ok(Interference::Overlapping {
                    volume: v,
                    a_in_b,
                    b_in_a,
                }) => {
                    prop_assert!(
                        close_to(v, volume, volume, REL),
                        "{}: overlap {} against the box's {}",
                        name,
                        v,
                        volume
                    );
                    prop_assert!(a_in_b && b_in_a, "{}: each should lie in the other", name);
                }
                Ok(other) => return Err(fail(format!("{name}: {other:?}"))),
                Err(e) => return Err(fail(format!("{name}: {e}"))),
            }
        }
        Ok(())
    });
    assert!(
        2 * asked.get() >= drawn.get(),
        "{} of {} draws reached interference",
        asked.get(),
        drawn.get()
    );
}

/// The draws build clean bodies, none has a wall tangent to another, and
/// `tangent` names the cases it leaves out: a cylinder whose wall touches a
/// box's face along a line, and two cylinders touching along a ruling.
#[test]
fn the_standing_draws_are_clean_and_leave_tangent_walls_out() {
    use arris_debug::prop::body::{Standing, StandingShape, StandingSolid};
    let solid = |shape, centre| StandingSolid {
        shape,
        centre,
        height: 1.0,
    };
    let slab = StandingShape::Box { half: (1.0, 1.0) };
    let disc = |radius| StandingShape::Cylinder { radius };
    let tangent_to_face = Standing {
        solids: vec![solid(slab, (0.0, 0.0)), solid(disc(1.0), (2.0, 0.0))],
    };
    let tangent_to_wall = Standing {
        solids: vec![solid(disc(1.0), (0.0, 0.0)), solid(disc(2.0), (3.0, 0.0))],
    };
    let apart = Standing {
        solids: vec![solid(slab, (0.0, 0.0)), solid(disc(1.5), (2.0, 0.0))],
    };
    assert!(tangent_to_face.tangent() && tangent_to_wall.tangent() && !apart.tangent());
    prop::check(prop::body::standing(), |case| {
        prop_assert!(!case.tangent());
        let mut m = Model::default();
        for body in case.build(&mut m).map_err(fail)? {
            let report = check(&m, body, Level::Fast);
            prop_assert!(report.is_ok(), "{}", report);
        }
        Ok(())
    });
}
