//! The twin property (plans/nurbs-analytic-section step 8, ADR-0061): a
//! boolean over a body whose faces are replaced by their exact
//! `to_nurbs` twins ([`arris_debug::prop::twin::nurbs_twin`]) gives the
//! analytic boolean's volume, area and counts, or both refuse. The
//! operands are the ones of the C3 identities — a frustum, a ball, a ring
//! or an elliptic prism against a box or a cylinder, a box against a
//! cylinder, two cylinders on crossing axes — so a plane, a cylinder, a
//! cone, a sphere and a torus each meet each other kind as a NURBS
//! surface against an analytic one, with the NURBS operand first or
//! second, in a random pose.
//!
//! A disagreement prints the shrunk pair and the seed and becomes a
//! fixture under `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).

use arris_check::{Level, Unchecked, check};
use arris_debug::dump::euler_line;
use arris_debug::prop::twin::nurbs_twin;
use arris_debug::testing::{close_to, fail, fitted_rel};
use arris_debug::unmetered::{common, cut, fuse, mass_properties};
use arris_debug::{prop, prop_shards};
use arris_ops::measure::MassProperties;
use arris_ops::{BooleanReason, Fault, OpError, Reason};
use arris_topo::{Body, Model, Provenance};
use proptest::prelude::*;

/// A boolean of two bodies: `fuse`, `common` or `cut`.
type Boolean = fn(&mut Model, Body, Body) -> Result<(Body, Provenance), OpError>;

/// What one boolean came to.
enum Outcome {
    /// A body, clean at `Full`, with its measure and counts.
    Made(MassProperties, String),
    /// A refusal, named.
    Refused(String),
}

/// `op(a, b)`, its body clean at `Full`, or its refusal. An analytic
/// result has nothing unchecked; a `twin` result may leave a pair of its
/// faces undecided (S5: two NURBS faces the checker cannot tell apart),
/// and nothing else. A fault the differential names as an exclusion, and the
/// designed `BesideSingularity`, reject the case: neither is the twin's.
fn outcome(
    m: &mut Model,
    name: &str,
    op: Boolean,
    a: Body,
    b: Body,
    twin: bool,
) -> Result<Outcome, TestCaseError> {
    let r = op(m, a, b);
    match r {
        Ok((body, _)) => {
            let report = check(m, body, Level::Full);
            let undecided = report
                .unchecked()
                .iter()
                .any(|u| !(twin && matches!(u, Unchecked::FacePair { .. })));
            if !report.is_ok() || undecided {
                return Err(fail(format!("{name}: not clean at Full\n{report}")));
            }
            let props = mass_properties(m, body).map_err(|e| fail(format!("{name}: {e}")))?;
            let counts = euler_line(m, body).map_err(fail)?;
            Ok(Outcome::Made(props, counts))
        }
        Err(OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::BesideSingularity),
            ..
        }) => Err(TestCaseError::reject("BesideSingularity")),
        // A twin result of several lumps: the shells' nesting is undecided,
        // NURBS against NURBS (B1), so the cut is refused where the
        // analytic one is not. Held by the ignored
        // `regression/nurbs-cylinder-slab-cut-two-lumps`; the fix lifts
        // this arm.
        Err(OpError::Internal(Fault::Lumps(_))) if twin => {
            Err(TestCaseError::reject("nurbs-cylinder-slab-cut-two-lumps"))
        }
        Err(e) => {
            if let Some(x) = arris_debug::differential::exclusion_of_error(&e) {
                return Err(TestCaseError::reject(x.name));
            }
            Ok(Outcome::Refused(e.to_string()))
        }
    }
}

/// Which boolean of a pair is run and which operand is the twin.
type Pick = (usize, bool);

/// The strategy of a [`Pick`]: a twin boolean costs 5 to 50 times an
/// analytic one and its `Full` check as much again, so a case runs one
/// boolean over one twin, and the cases cover the rest.
fn pick() -> impl Strategy<Value = Pick> {
    (0usize..3, any::<bool>())
}

/// The boolean `pick` names over the pair `(a, b)` in `m` and over the
/// pair with `a` or `b` replaced by its twin: the same volume, area and
/// counts, or both refused.
fn same_as_twin(m: &mut Model, a: Body, b: Body, pick: Pick) -> Result<(), TestCaseError> {
    let (op_name, op) = [
        ("fuse", fuse as Boolean),
        ("common", common as Boolean),
        ("cut", cut as Boolean),
    ][pick.0];
    let which = if pick.1 { "a" } else { "b" };
    let twin = nurbs_twin(m, if pick.1 { a } else { b })
        .map_err(|e| fail(format!("twin of {which}: {e}")))?;
    let (x, y) = if pick.1 { (twin, b) } else { (a, twin) };
    let analytic = outcome(m, &format!("{op_name}(a, b)"), op, a, b, false)?;
    let name = format!("{op_name}(a, b) with {which} a twin");
    match (&analytic, outcome(m, &name, op, x, y, true)?) {
        (Outcome::Refused(_), Outcome::Refused(_)) => {}
        (Outcome::Made(p, counts), Outcome::Made(q, twin_counts)) => {
            let rel = fitted_rel(m, p);
            prop_assert!(
                close_to(p.volume, q.volume, p.volume, rel),
                "{}: volume {} against the twin's {}",
                name,
                p.volume,
                q.volume
            );
            prop_assert!(
                close_to(p.area, q.area, p.area, rel),
                "{}: area {} against the twin's {}",
                name,
                p.area,
                q.area
            );
            prop_assert_eq!(counts, &twin_counts, "{}: counts", name);
        }
        (Outcome::Made(..), Outcome::Refused(why)) => {
            return Err(fail(format!("{name}: the twin refuses, {why}")));
        }
        (Outcome::Refused(why), Outcome::Made(..)) => {
            return Err(fail(format!("{name}: the analytic pair refuses, {why}")));
        }
    }
    Ok(())
}

prop_shards! {
    /// A frustum, a ball, a ring or an elliptic prism and a box or a
    /// cylinder through its interior: the cone, the sphere, the torus,
    /// the elliptic cylinder, the cylinder and the plane each against the
    /// others, one operand a NURBS twin.
    quadric_twins_agree_with_the_analytic_boolean
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((pair, pick)) = (prop::body::quadric_pair(), pick()) => {
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            same_as_twin(&mut m, a, b, pick)?;
            Ok(())
        }
}

prop_shards! {
    /// A box and a cylinder through it: planes against a cylinder.
    overlapping_twins_agree_with_the_analytic_boolean
        [shard_0 shard_1 shard_2 shard_3]
        ((pair, pick)) = (prop::body::overlapping_pair(), pick()) => {
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            same_as_twin(&mut m, a, b, pick)?;
            Ok(())
        }
}

prop_shards! {
    /// Two cylinders on crossing axes: a cylinder against a cylinder.
    crossing_twins_agree_with_the_analytic_boolean
        [shard_0 shard_1 shard_2 shard_3]
        ((pair, pick)) = (prop::body::crossing_pair(), pick()) => {
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            same_as_twin(&mut m, a, b, pick)?;
            Ok(())
        }
}
