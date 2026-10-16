//! The numeric bounds of a sweep's frame table and fold test (ADR-0063).
//!
//! None is a tolerance of the model: each bounds an algorithm's step or
//! sampling, and its reason is on the constant.

/// The most the tangent of a NURBS segment may turn between two entries
/// of its frame table, in radians. It bounds the table's step beside the
/// error control, so that a frame between two entries is one reflection
/// pair from the entry before it, and that a fixed binormal is tested
/// against the tangent often enough that a crossing is not missed.
pub const FRAME_MAX_TURN: f64 = 0.05;

/// The smallest step of the frame table, as a fraction of the knot
/// interval it walks. A step that would be smaller is a cusp or a
/// discontinuity the table cannot cross (`PathError::FrameStep`).
pub const FRAME_MIN_STEP: f64 = 1e-9;

/// The floor under the error the frame table is held to, in radians.
/// Round-off in a few hundred reflections is of this order; a tolerance
/// under it would never be met.
pub const FRAME_ERROR_FLOOR: f64 = 1e-13;

/// The offset of a sample from the limit under which the fold test
/// refines: an interval whose ends both sit under half the limit is not
/// cut.
pub const FOLD_NEAR: f64 = 0.5;

/// Differences of the carried reach under which an interval is not cut
/// again, as a fraction of the limit. Below it the reach is flat to the
/// precision the test can read from a finite difference of the tangent.
pub const FOLD_FLAT: f64 = 1e-3;

/// The step of the finite difference of the tangent that gives the
/// curvature, as a fraction of the span's length. The tangent is
/// smooth along a span, so the difference's error is of the order of
/// the step squared, far under the sample's own error.
pub const CURVATURE_STEP: f64 = 1e-5;
