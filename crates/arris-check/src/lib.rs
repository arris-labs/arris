//! The Arris invariant checker: one `Violation` per row of
//! `docs/DATA-MODEL.md` §Invariants, a `Level` that says which rows run,
//! and a `Report` that lists every violation with the entity that violates
//! it.
//!
//! Guarantees: the checker never repairs, never panics, and reports in a
//! deterministic order. Its only workspace dependency is `arris-topo`, so no
//! algorithm crate can bypass it by accident (`docs/ARCHITECTURE.md` §The
//! checker); `arris-topo` is re-exported, with `arris-geom` and `arris-math`
//! under it, so a crate above reaches the whole representation through this
//! one alone.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod check;
pub mod classify;
pub mod domain;
pub mod flux;
mod full;
mod lumps;
mod report;
mod topology;
mod unchecked;
mod violation;

pub use arris_topo::euler::EulerLine;
pub use check::check;
pub use classify::{Classification, Classifier, ClassifyError, classify_point};
pub use lumps::{Lump, LumpError, lumps};
pub use report::Report;
pub use unchecked::Unchecked;
pub use violation::{
    DegenerateFault, EdgeUseFault, EndMismatch, FaceFault, Level, LoopBreak, NestingFault,
    Quantity, Reference, SeamFault, ShellNestingFault, ToleranceBound, Violation, WireFault,
};
