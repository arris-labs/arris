//! **Cancellation at a random step** (ADR-0030, the roadmap's C5
//! property): on recipes drawn by `prop::recipe`, every body step's
//! unbudgeted step count `N` is measured, then a budget `k < N` stops the
//! step with `Interrupted` at exactly `k` and leaves the model's native
//! bytes as they were, a poll that answers `true` on its `k + 1`-th
//! question stops it the same way (at `k` in the sequential build; where a
//! poll lands under `parallel` is the schedule's), and a budget of `N` or
//! more gives the unbudgeted body. Run with `parallel` on and off; the
//! counts are the same in both (`cancel_counts.txt` holds the corpus's).

use std::sync::atomic::{AtomicU64, Ordering};

use arris_debug::dump::dump_text;
use arris_debug::fixtures::Step;
use arris_debug::testing::fail;
use arris_debug::{corpus, prop, prop_shards};
use arris_io::native;
use arris_ops::{Control, OpError, Stop};
use proptest::prelude::*;

/// One question asked of the poll per step: the steps `f` takes.
fn steps_of<T>(f: impl FnOnce(&Control<'_>) -> T) -> (T, u64) {
    let asked = AtomicU64::new(0);
    let poll = || {
        asked.fetch_add(1, Ordering::Relaxed);
        false
    };
    let out = f(&Control::poll(&poll));
    (out, asked.load(Ordering::Relaxed))
}

/// The interrupt a run ended in, or why it was not one.
fn interrupt<T>(run: Result<T, corpus::CorpusError>) -> Result<arris_ops::Interrupted, String> {
    match run {
        Err(corpus::CorpusError::Op {
            source: OpError::Interrupted(stop),
            ..
        }) => Ok(stop),
        Err(other) => Err(format!("not an interrupt: {other}")),
        Ok(_) => Err("ran to the end".into()),
    }
}

prop_shards! {
    /// Every body step of a drawn recipe: interrupted at a drawn `k < N`
    /// it stops there and changes nothing; at `k >= N` it changes nothing
    /// about its result.
    every_step_stops_at_a_random_step_and_leaves_the_model_as_it_was
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((recipe, draws)) = (
            prop::recipe::recipe(),
            proptest::collection::vec(any::<u64>(), 12),
        ) => {
            for (i, step) in recipe.steps.iter().enumerate() {
                if matches!(step, Step::Profile { .. }) {
                    continue;
                }
                let name = step.name();
                let Ok(inputs) = corpus::inputs_for_step("generated/cancel", &recipe, name)
                else {
                    break;
                };
                let before = native::to_bytes(&inputs.model).map_err(fail)?;
                let mut whole_model = inputs.model.clone();
                let (whole, n) = steps_of(|c| inputs.run_result(&mut whole_model, c));
                // A refusal is the kernel's answer to the recipe, and the
                // steps after it cannot be built.
                let Ok(whole) = whole else { break };
                let expected = dump_text(&whole_model, whole.body).map_err(fail)?;

                let draw = draws[i % draws.len()];
                if n > 0 {
                    let k = draw % n;
                    let mut m = inputs.model.clone();
                    let stop = interrupt(inputs.run_result(&mut m, &Control::budget(k)))
                        .map_err(|e| fail(format!("{name}: budget {k} of {n}: {e}")))?;
                    prop_assert_eq!((stop.by, stop.steps), (Stop::Budget, k), "{}: {} of {}", name, k, n);
                    prop_assert_eq!(
                        native::to_bytes(&m).map_err(fail)?,
                        before.clone(),
                        "{}: budget {} of {} changed the model", name, k, n
                    );

                    let mut m = inputs.model.clone();
                    let asked = AtomicU64::new(0);
                    let poll = || asked.fetch_add(1, Ordering::Relaxed) >= k;
                    let stop = interrupt(inputs.run_result(&mut m, &Control::poll(&poll)))
                        .map_err(|e| fail(format!("{name}: poll at {k} of {n}: {e}")))?;
                    prop_assert_eq!(stop.by, Stop::Poll);
                    if !cfg!(feature = "parallel") {
                        prop_assert_eq!(stop.steps, k, "{}: poll at question {}", name, k + 1);
                    }
                    prop_assert_eq!(
                        native::to_bytes(&m).map_err(fail)?,
                        before.clone(),
                        "{}: poll at {} of {} changed the model", name, k, n
                    );
                }

                let budget = n + draw % 3;
                let mut m = inputs.model.clone();
                let made = inputs
                    .run_result(&mut m, &Control::budget(budget))
                    .map_err(|e| fail(format!("{name}: a budget of {budget}, N = {n}: {e}")))?;
                prop_assert_eq!(
                    dump_text(&m, made.body).map_err(fail)?,
                    expected,
                    "{}: a budget of {} changed the result (N = {})", name, budget, n
                );
            }
            Ok(())
        }
}
