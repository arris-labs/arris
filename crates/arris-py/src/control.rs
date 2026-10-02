//! Stopping an operation: the `Cancel` token, a budget of steps and Ctrl-C.
//!
//! Every operation takes `cancel=` and `budget=` as keyword arguments and
//! releases the GIL around the kernel call. The kernel asks a poll at each
//! step (ADR-0030); the poll built here answers `true` when the token is
//! set, and — on the main thread, every [`SIGNAL_EVERY`] polls — when
//! Python has a signal pending, which an operation reports as
//! `KeyboardInterrupt`.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use arris::Control;
use pyo3::prelude::*;

/// How many polls pass between two looks at Python's signals. Looking
/// needs the GIL back, which the operation gave up so other threads run;
/// the kernel's own poll is a relaxed load, so the interval keeps the
/// look's cost off the steps that matter.
pub const SIGNAL_EVERY: u64 = 256;

/// A flag one thread sets to stop an operation another thread is running.
///
/// Pass it as `cancel=` to any operation. A set token stops the operation
/// at its next step with `Interrupted`, leaving the model as it was, ids
/// included; it stays set until `reset()`, so it also refuses an
/// operation that has not started yet.
///
/// ```python
/// import arris
///
/// token = arris.Cancel()
/// assert not token.is_set()
/// token.set()
/// assert token.is_set()
/// token.reset()
/// assert not token.is_set()
/// ```
#[pyclass(frozen, module = "arris")]
#[derive(Clone, Debug, Default)]
pub struct Cancel {
    flag: Arc<AtomicBool>,
}

#[pymethods]
impl Cancel {
    /// A token that is not set.
    #[new]
    fn new() -> Cancel {
        Cancel::default()
    }

    /// Asks every operation holding this token to stop.
    fn set(&self) {
        self.flag.store(true, Ordering::Release);
    }

    /// Clears the token, so it can be used again.
    fn reset(&self) {
        self.flag.store(false, Ordering::Release);
    }

    /// Whether the token is set.
    fn is_set(&self) -> bool {
        self.flag.load(Ordering::Acquire)
    }

    fn __repr__(&self) -> String {
        format!("Cancel(set={})", self.is_set())
    }
}

/// What a call was told about stopping, taken off the interpreter so the
/// kernel call can run without it.
#[derive(Clone, Debug, Default)]
pub(crate) struct Limits {
    cancel: Option<Arc<AtomicBool>>,
    budget: Option<u64>,
}

impl Limits {
    /// The limits of a call's `cancel=` and `budget=`.
    pub(crate) fn new(cancel: Option<&Cancel>, budget: Option<u64>) -> Limits {
        Limits {
            cancel: cancel.map(|c| Arc::clone(&c.flag)),
            budget,
        }
    }

    /// Runs `call` with a [`Control`] that enforces these limits and
    /// watches for Ctrl-C; the flag is `true` when Python's signal handler
    /// raised, so the caller can say `KeyboardInterrupt` and not
    /// `Interrupted`. The GIL must be released: the poll takes it back.
    pub(crate) fn run<T>(&self, call: impl FnOnce(&Control<'_>) -> T) -> (T, bool) {
        let signalled = AtomicBool::new(false);
        let polls = AtomicU64::new(0);
        let poll = || {
            if self
                .cancel
                .as_ref()
                .is_some_and(|flag| flag.load(Ordering::Acquire))
            {
                return true;
            }
            if polls.fetch_add(1, Ordering::Relaxed) % SIGNAL_EVERY == 0
                && Python::attach(|py| py.check_signals().is_err())
            {
                signalled.store(true, Ordering::Release);
                return true;
            }
            false
        };
        let control = Control::poll(&poll);
        let control = match self.budget {
            Some(steps) => control.with_budget(steps),
            None => control,
        };
        let out = call(&control);
        (out, signalled.load(Ordering::Acquire))
    }
}
