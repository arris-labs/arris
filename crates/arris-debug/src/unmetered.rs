//! The operations that take a [`Control`], called with [`Control::NONE`]
//! (ADR-0030): what a test that does not care about cancellation imports
//! in place of `arris_ops`'s, so its calls read as the operation and its
//! operands. A test that does care calls the operation itself.

use arris_ops::arris_check::arris_topo::arris_math::Control;
use arris_ops::arris_check::arris_topo::{Body, Model, Provenance};
use arris_ops::boolean::Interferences;
use arris_ops::{self, OpError};

/// [`arris_ops::cut`] to its end.
pub fn cut(m: &mut Model, target: Body, tool: Body) -> Result<(Body, Provenance), OpError> {
    arris_ops::cut(m, target, tool, &Control::NONE)
}

/// [`arris_ops::fuse`] to its end.
pub fn fuse(m: &mut Model, a: Body, b: Body) -> Result<(Body, Provenance), OpError> {
    arris_ops::fuse(m, a, b, &Control::NONE)
}

/// [`arris_ops::common`] to its end.
pub fn common(m: &mut Model, a: Body, b: Body) -> Result<(Body, Provenance), OpError> {
    arris_ops::common(m, a, b, &Control::NONE)
}

/// [`arris_ops::boolean::interferences`] to its end.
pub fn interferences(m: &Model, a: Body, b: Body) -> Result<Interferences, OpError> {
    arris_ops::boolean::interferences(m, a, b, &Control::NONE)
}
