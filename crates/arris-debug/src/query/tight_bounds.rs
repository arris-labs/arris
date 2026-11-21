//! Compare a result's tight box with its closed form and oracle record.

use std::collections::BTreeMap;

use super::{Answer, Query};

type Corners = [[f64; 3]; 2];

pub(super) struct Comparison {
    closed: Corners,
    oracle: Corners,
}

pub(super) fn comparison(
    query: &Query,
    answer: &Answer,
    params: &BTreeMap<String, f64>,
) -> Result<Comparison, String> {
    if query.tight_bounds != Some(true) {
        return Err("`tight_bounds` must be true".into());
    }
    let expected = query.expect_box.as_ref().ok_or("missing `expect_box`")?;
    let mut closed = [[0.0; 3]; 2];
    for (corner, expressions) in closed.iter_mut().zip(expected) {
        for (coordinate, expression) in corner.iter_mut().zip(expressions) {
            *coordinate = expression.eval(params).map_err(|e| e.to_string())?;
        }
    }
    let oracle = answer.tight_bounds.ok_or("the oracle has no tight box")?;
    for (source, [min, max]) in [("expect_box", closed), ("oracle", oracle)] {
        if !min
            .into_iter()
            .zip(max)
            .all(|(lo, hi)| lo.is_finite() && hi.is_finite() && lo <= hi)
        {
            return Err(format!("{source} needs finite and ordered box corners"));
        }
    }
    Ok(Comparison { closed, oracle })
}

impl Comparison {
    pub(super) fn oracle_difference(&self, tolerance: f64) -> Option<String> {
        difference(self.oracle, self.closed, tolerance, "oracle", "closed form")
    }

    pub(super) fn check(
        &self,
        actual: Corners,
        tolerance: f64,
        measure_differs: bool,
    ) -> Result<(), String> {
        if let Some(diff) = difference(actual, self.closed, tolerance, "Arris", "closed form") {
            return Err(diff);
        }
        match self.oracle_difference(tolerance) {
            Some(_) if measure_differs => Ok(()),
            Some(diff) => Err(diff),
            None => {
                difference(actual, self.oracle, tolerance, "Arris", "oracle").map_or(Ok(()), Err)
            }
        }
    }
}

fn difference(
    actual: Corners,
    expected: Corners,
    tolerance: f64,
    actual_name: &str,
    expected_name: &str,
) -> Option<String> {
    for (corner, (got, want)) in ["min", "max"]
        .into_iter()
        .zip(actual.into_iter().zip(expected))
    {
        for (axis, (a, b)) in ["x", "y", "z"].into_iter().zip(got.into_iter().zip(want)) {
            if !a.is_finite() || (a - b).abs() > tolerance {
                return Some(format!(
                    "box {corner}.{axis}: {actual_name} gives {a}, {expected_name} gives {b}, tolerance {tolerance}"
                ));
            }
        }
    }
    None
}
