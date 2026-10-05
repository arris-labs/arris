//! A metered pass over independent items (ADR-0030 §4): run in `rayon`
//! behind `parallel`, a plain loop otherwise, and stopped at the same
//! item with the same count either way.

use arris_math::Meter;

use crate::error::OpError;

/// Takes one item's outcome and the steps it used into `meter`, in the
/// sequential order: an item that finished or refused is charged its
/// steps, one that stopped is charged through
/// [`Meter::charge_stop`].
fn settle<T>(meter: &mut Meter<'_>, result: Result<T, OpError>, steps: u64) -> Result<T, OpError> {
    match result {
        Ok(value) => {
            meter.charge(steps)?;
            Ok(value)
        }
        Err(OpError::Interrupted(stop)) => Err(meter.charge_stop(stop).into()),
        Err(other) => {
            meter.charge(steps)?;
            Err(other)
        }
    }
}

/// `one` over every item, in the items' order in the result however it was
/// computed. Each item gets a [`Meter::split`] of `meter` — the same poll,
/// capped at the budget left when it starts — and the items' steps are
/// charged to `meter` in the sequential order, so the first item whose
/// running total crosses the budget is the one that stops the pass, with
/// the same count in both builds. The parallel build splits every item off
/// the budget left when the pass starts and only wastes the work of the
/// items past the stop; every thread polls, since a poll promises the
/// model as it was and nothing more.
///
/// The first error in item order wins: a `Result` collected straight from
/// `rayon` is whichever error a thread met first, and would make a refusal
/// depend on the schedule.
pub(crate) fn pass<I: Sync, T: Send>(
    items: &[I],
    meter: &mut Meter<'_>,
    one: impl Fn(&I, &mut Meter<'_>) -> Result<T, OpError> + Sync + Send,
) -> Result<Vec<T>, OpError> {
    let mut out = Vec::with_capacity(items.len());
    #[cfg(feature = "parallel")]
    {
        use rayon::prelude::*;
        let base = meter.split();
        let done: Vec<(Result<T, OpError>, u64)> = items
            .par_iter()
            .map(|item| {
                let mut own = base;
                let result = one(item, &mut own);
                (result, own.steps())
            })
            .collect();
        for (result, steps) in done {
            out.push(settle(meter, result, steps)?);
        }
    }
    #[cfg(not(feature = "parallel"))]
    for item in items {
        let mut own = meter.split();
        let result = one(item, &mut own);
        out.push(settle(meter, result, own.steps())?);
    }
    Ok(out)
}
