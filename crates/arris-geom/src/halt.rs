//! Why an internal search stopped: its own fault, or the caller's stop.
//! Crate-private: the public entry points turn a [`Halt`] into their own
//! error type (`GeomError`), where a fault keeps its meaning and a stop
//! is [`GeomError::Interrupted`](crate::GeomError::Interrupted).

use arris_math::Interrupted;

/// A search that ended without its answer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Halt<F> {
    /// The search's own refusal.
    Fault(F),
    /// The caller stopped it (ADR-0030).
    Stopped(Interrupted),
}

impl<F> Halt<F> {
    /// The same stop, or the fault turned by `f`.
    pub(crate) fn map_fault<G>(self, f: impl FnOnce(F) -> G) -> Halt<G> {
        match self {
            Halt::Fault(fault) => Halt::Fault(f(fault)),
            Halt::Stopped(stop) => Halt::Stopped(stop),
        }
    }
}

impl<F> From<Interrupted> for Halt<F> {
    fn from(stop: Interrupted) -> Self {
        Halt::Stopped(stop)
    }
}

impl From<crate::SectionFault> for Halt<crate::SectionFault> {
    fn from(fault: crate::SectionFault) -> Self {
        Halt::Fault(fault)
    }
}
