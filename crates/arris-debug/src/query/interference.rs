//! The `interference` form of the `query` fixture kind (ADR-0055): two
//! whole bodies, held to Open CASCADE's verdict.
//!
//! The runner compares the verdict exactly, the distance of an apart pair
//! within the fixture's `probe`, the volume of an overlapping pair relative
//! to `volume_rel`, the containment flags exactly, and a touching pair's
//! contact dimension exactly unless `analytic.query_differs` says why not.

use std::collections::BTreeMap;

use arris_math::Control;
use arris_ops::query::{Contact, Interference, interference};

use super::{Answer, DistanceSpec, OperandSpec, Query, QueryError, QueryFixture, VariantAnswers};
use crate::corpus::Chain;
use crate::fixtures::Num;

/// The verdicts a fixture may name.
const VERDICTS: [&str; 3] = ["apart", "touching", "overlapping"];
/// The contact dimensions a fixture may name.
const CONTACTS: [&str; 3] = ["point", "curve", "face"];

fn verdict_of(i: &Interference) -> &'static str {
    match i {
        Interference::Apart(_) => "apart",
        Interference::Touching { .. } => "touching",
        Interference::Overlapping { .. } => "overlapping",
    }
}

fn contact_of(c: Contact) -> &'static str {
    match c {
        Contact::Point => "point",
        Contact::Curve => "curve",
        Contact::Face => "face",
    }
}

/// The body an operand names: interference takes whole bodies only.
fn body_of(chain: &Chain, spec: &OperandSpec) -> Result<arris_topo::Body, String> {
    if spec.point.is_some() || spec.face.is_some() || spec.edge.is_some() || spec.vertex.is_some() {
        return Err("an interference operand is a whole `step`".into());
    }
    let name = spec.step.as_ref().ok_or("an operand needs a `step`")?;
    chain
        .steps
        .get(name)
        .map(|s| s.body)
        .ok_or_else(|| format!("no step {name:?}"))
}

/// Runs one interference query of `fixture` under `variant` against `answer`.
pub(super) fn run(
    fixture: &QueryFixture,
    variant: &str,
    chain: &Chain,
    query: &Query,
    spec: &DistanceSpec,
    answer: &Answer,
) -> Result<(), QueryError> {
    let wrong = |what: String| QueryError::Mismatch {
        fixture: fixture.name.clone(),
        variant: variant.to_string(),
        label: query.label.clone(),
        what,
    };
    let Some(oracle) = &answer.interference else {
        return Err(wrong("the oracle has no interference".into()));
    };
    let expression = |source| QueryError::Expression {
        fixture: fixture.name.clone(),
        label: query.label.clone(),
        source,
    };
    let a = body_of(chain, &spec.a).map_err(wrong)?;
    let b = body_of(chain, &spec.b).map_err(wrong)?;
    let ours =
        interference(&chain.model, a, b, &Control::NONE).map_err(|source| QueryError::Refused {
            fixture: fixture.name.clone(),
            variant: variant.to_string(),
            label: query.label.clone(),
            source: Box::new(source),
        })?;
    let tolerances = &fixture.recipe.tolerances;
    let verdict = verdict_of(&ours);
    if let Some(expect) = &query.expect
        && expect != verdict
    {
        return Err(wrong(format!("{verdict} but the recipe expects {expect}")));
    }
    if verdict != oracle.verdict {
        return Err(wrong(format!(
            "{verdict} but the oracle's verdict is {}",
            oracle.verdict
        )));
    }
    let relative = |name: &str, got: f64, want: f64, tolerance: f64| {
        let difference = (got - want).abs() / want.abs().max(got.abs()).max(f64::MIN_POSITIVE);
        if difference.is_nan() || difference > tolerance {
            Err(wrong(format!(
                "{name} {got} but {want} expected: {difference:e} relative, above {tolerance:e}"
            )))
        } else {
            Ok(())
        }
    };
    match ours {
        Interference::Apart(d) => {
            if (d.distance - oracle.distance).abs() > tolerances.probe {
                return Err(wrong(format!(
                    "distance {} but the oracle's is {}",
                    d.distance, oracle.distance
                )));
            }
        }
        Interference::Touching { contact, .. } => {
            let ours = contact_of(contact);
            if let Some(expect) = &query.expect_contact
                && expect != ours
            {
                return Err(wrong(format!(
                    "contact {ours} but the recipe expects {expect}"
                )));
            }
            let differs = fixture.recipe.analytic.query_differs.is_some();
            if !differs && oracle.contact.as_deref() != Some(ours) {
                return Err(wrong(format!(
                    "contact {ours} but the oracle's is {:?}",
                    oracle.contact
                )));
            }
        }
        Interference::Overlapping {
            volume,
            a_in_b,
            b_in_a,
        } => {
            if let Some(expect) = &query.expect_volume {
                let expect = expect.eval(&chain.params).map_err(expression)?;
                relative("volume", volume, expect, tolerances.volume_rel)?;
            }
            let Some(want) = oracle.volume else {
                return Err(wrong("the oracle has no volume".into()));
            };
            relative("volume", volume, want, tolerances.volume_rel)?;
            for (name, got, want) in [
                ("a_in_b", a_in_b, oracle.a_in_b),
                ("b_in_a", b_in_a, oracle.b_in_a),
            ] {
                if want != Some(got) {
                    return Err(wrong(format!("{name} {got} but the oracle's is {want:?}")));
                }
            }
        }
    }
    Ok(())
}

/// The lint of one interference query: its `expect*` names are known and
/// agree with the oracle's answer, which must be an interference.
pub(super) fn lint(
    q: &Query,
    params: &BTreeMap<String, f64>,
    answers: &VariantAnswers,
) -> Vec<String> {
    let mut out = Vec::new();
    let Some(oracle) = answers
        .queries
        .iter()
        .find(|a| a.label == q.label)
        .and_then(|a| a.interference.as_ref())
    else {
        out.push("the oracle has no interference".into());
        return out;
    };
    if !VERDICTS.contains(&oracle.verdict.as_str()) {
        out.push(format!(
            "the oracle's verdict {:?} is unknown",
            oracle.verdict
        ));
    }
    if let Some(expect) = &q.expect {
        if !VERDICTS.contains(&expect.as_str()) {
            out.push(format!(
                "expect {expect:?} is not apart, touching or overlapping"
            ));
        } else if *expect != oracle.verdict {
            out.push(format!(
                "expect {expect} but the oracle says {}",
                oracle.verdict
            ));
        }
    }
    if let Some(expect) = &q.expect_contact {
        if !CONTACTS.contains(&expect.as_str()) {
            out.push(format!(
                "expect_contact {expect:?} is not point, curve or face"
            ));
        } else if oracle.contact.as_ref() != Some(expect) {
            out.push(format!(
                "expect_contact {expect} but the oracle says {:?}",
                oracle.contact
            ));
        }
    }
    if let Some(expect) = &q.expect_volume {
        let num: &Num = expect;
        match (num.eval(params), oracle.volume) {
            (Ok(e), Some(v)) if (e - v).abs() > 1e-9 * v.abs().max(e.abs()) => {
                out.push(format!("expect_volume {e} but the oracle says {v}"));
            }
            (Ok(_), Some(_)) => {}
            (Ok(_), None) => {
                out.push("expect_volume but the oracle's pair does not overlap".into())
            }
            (Err(e), _) => out.push(e.to_string()),
        }
    }
    out
}
