//! Operations of the Arris kernel: primitives, `build` for a consumer's
//! own topology, planar profiles, extrude,
//! revolve, transform, the booleans (and `boolean::interferences`, their
//! decomposition as a value), the blends (`fillet`, `chamfer`), `measure` for
//! mass properties, and `query` for the projection of edges and vertices
//! onto a plane.
//!
//! Guarantees: every operation has the shape `op(&mut Model, inputs…) ->
//! Result<(Body, Provenance), OpError>` (`docs/ARCHITECTURE.md`
//! §Operations); it never mutates its inputs, never panics on geometry,
//! returns provenance for every entity it touched, and leaves the model as
//! it was on `Err`. In debug builds its output passes `arris-check` at
//! `Level::Fast` before it is returned, and a failure there panics with
//! the report: a kernel bug, the one place a panic is allowed. `build`
//! is the exception: its body is the consumer's input, checked in every
//! profile and refused with [`OpError::Rejected`]. The
//! `paranoid` feature runs the same check in release builds and returns
//! [`OpError::Internal`] instead. The `parallel` feature reserves `rayon`
//! inside an operation.
//!
//! Every operation on a model, and `measure::mass_properties` beside
//! them, takes a trailing [`Control`]: a poll the caller answers from
//! whatever its platform has and a budget of steps, [`Control::NONE`]
//! for neither. A stop is [`OpError::Interrupted`] — the model as it was,
//! ids included — and the same input and budget stop at the same step on
//! every platform and with `parallel` on or off (ADR-0030). Depends on
//! `arris-check` and below, re-exported here.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod blend;
mod body_view;
pub mod boolean;
mod build;
mod error;
pub mod measure;
mod mirror;
mod pass;
mod primitive;
pub mod query;
mod rebuild;
mod sweep;
mod transform;

pub use arris_math::{Control, Interrupted, Stop};

pub use blend::{chamfer, fillet};
pub use boolean::{common, cut, fuse};
pub use build::{BuildKeys, BuildSlot, Rejection, build};
pub use error::{Fault, OpError, Reason, SplitFault};
pub use mirror::mirror;
pub use primitive::{primitive_box, primitive_cylinder};
pub use sweep::{extrude, revolve};
pub use transform::transform;

use arris_topo::{Body, Model};

/// The input check every operation and query runs before it reads a
/// body: the handle resolves ([`OpError::NotFound`] otherwise), and in
/// debug builds — or release with the `paranoid` feature — the body
/// passes the checker at `Level::Fast` ([`OpError::InvalidInput`] with
/// the report otherwise).
fn verify_input(m: &Model, body: Body) -> Result<(), OpError> {
    m.body(body.id)?;
    #[cfg(any(debug_assertions, feature = "paranoid"))]
    {
        let report = arris_check::check(m, body, arris_check::Level::Fast);
        if !report.is_ok() {
            return Err(OpError::InvalidInput {
                body,
                report: Box::new(report),
            });
        }
    }
    Ok(())
}

/// The debug-build guard every operation runs on its output before
/// returning `Ok`: `Level::Fast`, a panic with the report on a failure.
/// With the `paranoid` feature a release build runs it too and returns
/// [`OpError::Internal`]. In a plain release build, nothing runs.
#[cfg(any(debug_assertions, feature = "paranoid"))]
fn verify(m: &Model, body: Body) -> Result<(), OpError> {
    let report = arris_check::check(m, body, arris_check::Level::Fast);
    if report.is_ok() {
        return Ok(());
    }
    #[cfg(debug_assertions)]
    panic!("kernel bug: an operation's output fails the checker\n{report}");
    #[cfg(not(debug_assertions))]
    Err(OpError::Internal(Fault::Checker(Box::new(report))))
}

#[cfg(not(any(debug_assertions, feature = "paranoid")))]
fn verify(_: &Model, _: Body) -> Result<(), OpError> {
    Ok(())
}
