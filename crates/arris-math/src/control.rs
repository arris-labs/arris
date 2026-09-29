//! Cancellation: the argument every operation takes so its caller can stop
//! it, and the counter the kernel's loops tick (ADR-0030).
//!
//! A [`Control`] is what the caller supplies: a poll it answers from
//! whatever its platform has, and a budget of steps. A [`Meter`] is what
//! an algorithm holds: it counts a step at each loop boundary of work
//! that can grow past its input's size and asks the budget and then the
//! poll. A stop is an [`Interrupted`], which the operation returns in its
//! own error type; the transaction it runs in rolls the model back.

use core::fmt;

/// The poll a caller supplies: `true` asks the running operation to stop.
/// `Sync` because the `parallel` passes poll from several threads.
pub type Poll<'a> = &'a (dyn Fn() -> bool + Sync);

/// What stopped an operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Stop {
    /// The caller's poll answered `true`.
    Poll,
    /// The caller's budget of steps ran out.
    Budget,
}

/// An operation that was stopped, by the caller and not by its input. The
/// model is as it was before the call.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Interrupted {
    /// What stopped it.
    pub by: Stop,
    /// The steps taken before the stop. For [`Stop::Budget`] it is the
    /// budget; the same input and budget stop at the same count on every
    /// platform and with `parallel` on or off.
    pub steps: u64,
}

impl fmt::Display for Interrupted {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.by {
            Stop::Poll => write!(
                f,
                "interrupted by the caller's poll after {} steps",
                self.steps
            ),
            Stop::Budget => write!(
                f,
                "interrupted: the budget of {} steps is spent",
                self.steps
            ),
        }
    }
}

impl core::error::Error for Interrupted {}

/// How a caller stops an operation: a poll, a budget of steps, both or
/// neither. Every operation on a model, and every long query beside one,
/// takes one last; [`Control::NONE`] is what a caller that does not care
/// passes.
///
/// A step is one iteration of a loop whose trip count is not bounded by
/// the input's entity count (ADR-0030 §5). Step counts are stable within
/// one release, not across releases: a budget is a cap a caller tunes.
///
/// ```
/// use arris_math::{Control, Meter, Stop};
/// use core::sync::atomic::{AtomicBool, Ordering};
///
/// let stop = AtomicBool::new(false);
/// let poll = || stop.load(Ordering::Relaxed);
/// let control = Control::poll(&poll).with_budget(2);
/// let mut meter = Meter::new(&control);
/// assert!(meter.tick().is_ok() && meter.tick().is_ok());
/// let spent = meter.tick().unwrap_err();
/// assert_eq!((spent.by, spent.steps), (Stop::Budget, 2));
///
/// stop.store(true, Ordering::Relaxed);
/// let stopped = Meter::new(&control).tick().unwrap_err();
/// assert_eq!((stopped.by, stopped.steps), (Stop::Poll, 0));
/// ```
#[derive(Clone, Copy, Default)]
pub struct Control<'a> {
    poll: Option<Poll<'a>>,
    budget: Option<u64>,
}

impl fmt::Debug for Control<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Control")
            .field("poll", &self.poll.map(|_| "…"))
            .field("budget", &self.budget)
            .finish()
    }
}

impl Control<'static> {
    /// No poll and no budget: the operation runs to its end.
    pub const NONE: Control<'static> = Control {
        poll: None,
        budget: None,
    };

    /// A budget of `steps` and no poll: the `steps + 1`-th step is
    /// [`Stop::Budget`].
    pub const fn budget(steps: u64) -> Control<'static> {
        Control {
            poll: None,
            budget: Some(steps),
        }
    }
}

impl<'a> Control<'a> {
    /// A poll and no budget. The poll is asked at every step, and a
    /// `true` stops the operation at that step.
    pub const fn poll(poll: Poll<'a>) -> Control<'a> {
        Control {
            poll: Some(poll),
            budget: None,
        }
    }

    /// This control with a budget of `steps`.
    #[must_use]
    pub const fn with_budget(mut self, steps: u64) -> Self {
        self.budget = Some(steps);
        self
    }

    /// This control with `poll`.
    #[must_use]
    pub const fn with_poll(mut self, poll: Poll<'a>) -> Self {
        self.poll = Some(poll);
        self
    }
}

/// The counter an algorithm ticks: a [`Control`] and the steps taken.
///
/// [`tick`](Meter::tick) is the step: it asks the budget, then the poll,
/// and counts the step only if neither stops it. A parallel pass gives
/// each item a [`split`](Meter::split) and afterwards
/// [`charge`](Meter::charge)s the items' steps in the sequential order, so
/// both builds stop on the same item with the same count (ADR-0030 §4).
#[derive(Clone, Copy)]
pub struct Meter<'a> {
    poll: Option<Poll<'a>>,
    cap: Option<u64>,
    steps: u64,
}

impl fmt::Debug for Meter<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Meter")
            .field("cap", &self.cap)
            .field("steps", &self.steps)
            .finish()
    }
}

impl Default for Meter<'_> {
    /// A meter that never stops: [`Control::NONE`]'s.
    fn default() -> Self {
        Meter::new(&Control::NONE)
    }
}

impl<'a> Meter<'a> {
    /// A meter at zero steps under `control`.
    pub fn new(control: &Control<'a>) -> Self {
        Meter {
            poll: control.poll,
            cap: control.budget,
            steps: 0,
        }
    }

    /// One step. `Err` when the budget is spent (checked first, and the
    /// poll is then not asked) or the poll answers `true`; the step is not
    /// counted, so [`Interrupted::steps`] is the steps completed.
    pub fn tick(&mut self) -> Result<(), Interrupted> {
        if self.cap.is_some_and(|cap| self.steps >= cap) {
            return Err(self.stop(Stop::Budget));
        }
        if self.poll.is_some_and(|poll| poll()) {
            return Err(self.stop(Stop::Poll));
        }
        self.steps += 1;
        Ok(())
    }

    /// The steps taken so far.
    pub fn steps(&self) -> u64 {
        self.steps
    }

    /// The meter for one item of a parallel pass: the same poll, no steps
    /// yet, capped at the budget this meter has left.
    #[must_use]
    pub fn split(&self) -> Meter<'a> {
        Meter {
            poll: self.poll,
            cap: self.cap.map(|cap| cap.saturating_sub(self.steps)),
            steps: 0,
        }
    }

    /// Takes the `steps` of one item of a pass this meter was
    /// [`split`](Meter::split) for. Items are charged in the sequential
    /// order; the first whose running total crosses the budget is
    /// `Err(Interrupted { by: Stop::Budget, steps: budget })`, which is
    /// what the sequential build's own tick reports there.
    pub fn charge(&mut self, steps: u64) -> Result<(), Interrupted> {
        let total = self.steps.saturating_add(steps);
        if let Some(cap) = self.cap
            && total > cap
        {
            self.steps = cap;
            return Err(self.stop(Stop::Budget));
        }
        self.steps = total;
        Ok(())
    }

    /// The [`Interrupted`] of an item that stopped itself, as this meter
    /// reports it: the item's steps added to the ones already charged, so
    /// the count is the sequential build's. An item that finished is
    /// [`charge`](Meter::charge)d; one that stopped is charged here
    /// instead, never both.
    pub fn charge_stop(&mut self, item: Interrupted) -> Interrupted {
        self.steps = self.steps.saturating_add(item.steps);
        self.stop(item.by)
    }

    fn stop(&self, by: Stop) -> Interrupted {
        Interrupted {
            by,
            steps: self.steps,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU64, Ordering};

    #[test]
    fn none_never_stops() {
        let mut m = Meter::new(&Control::NONE);
        for _ in 0..1000 {
            assert!(m.tick().is_ok());
        }
        assert_eq!(m.steps(), 1000);
    }

    #[test]
    fn a_budget_of_n_allows_n_steps() {
        let mut m = Meter::new(&Control::budget(3));
        assert!(m.tick().is_ok() && m.tick().is_ok() && m.tick().is_ok());
        let e = m.tick().unwrap_err();
        assert_eq!(
            e,
            Interrupted {
                by: Stop::Budget,
                steps: 3
            }
        );
        // Stays stopped, and does not count past the budget.
        assert_eq!(m.tick().unwrap_err(), e);
        assert_eq!(Meter::new(&Control::budget(0)).tick().unwrap_err().steps, 0);
    }

    #[test]
    fn a_poll_is_asked_at_every_step_and_stops_at_its_first_true() {
        let calls = AtomicU64::new(0);
        let poll = || calls.fetch_add(1, Ordering::Relaxed) + 1 == 4;
        let mut m = Meter::new(&Control::poll(&poll));
        assert!(m.tick().is_ok() && m.tick().is_ok() && m.tick().is_ok());
        let e = m.tick().unwrap_err();
        assert_eq!(
            e,
            Interrupted {
                by: Stop::Poll,
                steps: 3
            }
        );
    }

    #[test]
    fn the_budget_is_asked_before_the_poll() {
        let calls = AtomicU64::new(0);
        let poll = || {
            calls.fetch_add(1, Ordering::Relaxed);
            false
        };
        let mut m = Meter::new(&Control::poll(&poll).with_budget(1));
        m.tick().unwrap();
        assert_eq!(m.tick().unwrap_err().by, Stop::Budget);
        assert_eq!(calls.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn a_split_is_capped_at_what_is_left() {
        let mut m = Meter::new(&Control::budget(5));
        m.tick().unwrap();
        m.tick().unwrap();
        let mut item = m.split();
        for _ in 0..3 {
            item.tick().unwrap();
        }
        assert_eq!(item.tick().unwrap_err().steps, 3);
        assert_eq!(Meter::new(&Control::NONE).split().cap, None);
    }

    /// The parallel rule against the sequential one: items charged in
    /// order stop at the same item, with the same count, as one meter
    /// ticking through all of them.
    #[test]
    fn charging_in_order_stops_where_sequential_ticks_stop() {
        let items = [2u64, 3, 1, 4];
        for budget in 0..=12u64 {
            let mut sequential = Meter::new(&Control::budget(budget));
            let seq = items
                .iter()
                .enumerate()
                .find_map(|(i, &n)| (0..n).find_map(|_| sequential.tick().err()).map(|e| (i, e)));
            let mut joined = Meter::new(&Control::budget(budget));
            let par = items.iter().enumerate().find_map(|(i, &n)| {
                let mut item = joined.split();
                let done = (0..n).try_for_each(|_| item.tick());
                match done {
                    Ok(()) => joined.charge(item.steps()).err().map(|e| (i, e)),
                    Err(e) => Some((i, joined.charge_stop(e))),
                }
            });
            assert_eq!(par, seq, "budget {budget}");
            assert_eq!(joined.steps(), sequential.steps(), "budget {budget}");
        }
    }
}
