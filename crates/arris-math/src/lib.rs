//! Numeric foundation of the Arris kernel: points, vectors and unit vectors
//! over `nalgebra`, frames, axes and rigid motions, intervals, exact orientation
//! predicates over `robust`, polynomial and interval-guarded root finding,
//! and `Precision`, the model-wide tolerance configuration.
//!
//! Guarantees: `f64` throughout, no allocation in evaluation, no panic on
//! any finite input, and no numeric literal standing in for a tolerance —
//! every tolerance is a `Precision` field, a [`Tolerance`] derived from it,
//! or a named constant with a comment (`.agents/rules/kernel.md`). Depends
//! on nothing in the workspace.
//!
//! The point and vector types are `nalgebra`'s by alias and `nalgebra` is
//! re-exported (`docs/adr/0001-nalgebra-types-by-alias.md`), so a caller
//! reaches every operator and solver `nalgebra` has and a `nalgebra` major
//! bump is an Arris API change.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod aabb;
mod axis;
mod control;
mod frame;
mod interval;
mod isometry;
mod periodic;
mod precision;
pub mod predicates;
mod reflection;
pub mod roots;
pub mod sweep_steps;
mod tolerance;

pub use nalgebra;

pub use aabb::Aabb;
pub use axis::Axis;
pub use control::{Control, Interrupted, Meter, Poll, Stop};
pub use frame::{Frame, Frame2, FrameError, Handedness};
pub use interval::{Interval, IntervalError};
pub use isometry::Isometry;
pub use periodic::{
    period_end, shift_into_range, shift_nearest, shift_nearest_uv, wrap_angle, wrap_into,
    wrap_offset, wrap_signed,
};
pub use precision::Precision;
pub use reflection::{Reflection, ReflectionError};
pub use tolerance::Tolerance;

/// Relative rounding slack: a magnitude at or below this fraction of its
/// natural scale is rounding noise, not a value. Eight ulps — what a
/// handful of multiplications and one trigonometric evaluation leave
/// behind, and orders of magnitude below any model tolerance. It is not a
/// geometric tolerance and never decides whether two things are *the
/// same*; it decides whether a computed quantity is zero *in floating
/// point*: the radius of a sphere's parallel at the pole, `cos(π/2)`
/// evaluated in `f64`, is `6e-17`, not `0`.
pub const RELATIVE_ROUNDING: f64 = 8.0 * f64::EPSILON;

/// The widest gap a file's entities may leave that a read closes by a
/// tolerance, as a fraction of the part's size — the diameter of the ball
/// holding its vertices and edges (ADR-0025 §4). A vertex or an edge whose
/// measured gap is above it, or above `Precision::max_tolerance`, is
/// refused as a gap: closing it would be sewing, which is healing, and a
/// solid grown to cover it would be a wrong one.
///
/// Evidence: an exact writer leaves gaps at the rounding of its decimal
/// digits, around `1e-15` of the part (Arris's own files); a kernel's
/// raised tolerances reach a few `1e-7` on a part of unit size (the
/// corpus's `boolean/seam-a-tolerance-from-crossing-fuse`, `1.2e-7` on a
/// part of size 7). Open CASCADE's own STEP of every corpus fixture
/// (the corpus's read-back stage) measures at most `2.5e-6` of the part's
/// diagonal on 201 of 202 files, and `8.5e-5` on the last — `7.4e-4` on a
/// part `8.7` across, `boolean/seam-beside-crossing-fuse`, whose boolean
/// Open CASCADE itself gets wrong by that much. Converted to B-splines
/// (step 15), its `closed-form-band` variant measures `1.05e-3` on a part
/// whose ball is `9.3` across, `1.13e-4` of it — exactly the tolerance
/// Open CASCADE's own shape carries there, which its checker passes.
/// Healing tools commonly default their sewing tolerance to `1e-6`–`1e-3`
/// of the model's size, so `2e-4` admits every file an exporting kernel
/// called valid by its own tolerances, the largest seen with `1.8×` to
/// spare, and refuses the gaps a healer would have to sew. A tolerance
/// policy, not a rounding slack. Over the real-part corpus it refuses
/// gaps in 7 of NIST's 38 parts, which count for the healing cycle
/// (ADR-0026 §5, `docs/ROADMAP.md` §C4).
pub const READ_GAP_FRACTION: f64 = 2e-4;

/// The healing ceiling: the largest gap, as a fraction of the part's size
/// (the same ball [`READ_GAP_FRACTION`] is of), that a STEP read heals by
/// growing the tolerance of the entity it lies at (ADR-0058). A gap past
/// it is still a refusal.
///
/// Measured on the real-part corpus by the gap census (`real_parts
/// --gaps`): the seven parts the read refuses a gap of hold gaps of
/// `8.6e-6` to `1.5e-4` of their size, the largest FTC-11's `0.0141` on a
/// part `96.5` across, and Open CASCADE's reading of the same parts
/// carries tolerances of up to `2.6e-4` of the part (FTC-07's `0.104` on
/// `407`; CTC-02's `0.108` on `1167` is `9e-5`). `5e-4` is `3.4×` the
/// largest gap Arris measured and `1.9×` the largest Open CASCADE carries: room for a part
/// a little worse than any seen, and under every healing tool's usual
/// sewing tolerance (`1e-3` of the size). A tolerance policy, not a
/// rounding slack.
pub const HEAL_GAP_FRACTION: f64 = 5e-4;

/// The most a read edge's gap is raised for the angle its two faces cross
/// at (ADR-0059): an edge lying `d` off one face and on the other is
/// `d / sin θ` from where faces crossing at `θ` meet, and its tolerance
/// is raised to that where `1 / sin θ` is at most this — faces crossing at
/// `30°` or more. Under it the faces are near tangent, where they meet is
/// ill-conditioned (a G1 blend's faces meet anywhere along a band), and
/// the edge keeps its gap from each surface.
///
/// Evidence: CTC-02's drafted bosses, a cylinder `2°` off the normal of
/// the plane it stands on, `1 / sin θ = 1.0006`
/// (`real/edge-off-a-drafted-cylinder`). A geometric bound, not a
/// rounding slack.
pub const SECTION_GROWTH_LIMIT: f64 = 2.0;

/// `|x| ≤ RELATIVE_ROUNDING · |scale|`: `x` is zero to rounding at
/// `scale`. A zero `scale` makes only an exact zero negligible.
///
/// ```
/// use arris_math::is_negligible;
/// use core::f64::consts::FRAC_PI_2;
///
/// assert!(is_negligible(3.0 * FRAC_PI_2.cos(), 3.0));
/// assert!(!is_negligible(3.0 * (FRAC_PI_2 - 1e-9).cos(), 3.0));
/// ```
pub fn is_negligible(x: f64, scale: f64) -> bool {
    x.abs() <= RELATIVE_ROUNDING * scale.abs()
}

/// A position in 3D. `nalgebra::Point3<f64>` (ADR-0001).
pub type Point3 = nalgebra::Point3<f64>;
/// A displacement or direction in 3D, of any length.
/// `nalgebra::Vector3<f64>` (ADR-0001).
pub type Vec3 = nalgebra::Vector3<f64>;
/// A direction in 3D: a [`Vec3`] of unit length, guaranteed by construction.
/// `nalgebra::Unit<Vector3<f64>>` (ADR-0001).
pub type UnitVec3 = nalgebra::Unit<Vec3>;
/// A position in a surface's (u, v) plane. `nalgebra::Point2<f64>`
/// (ADR-0001).
pub type Point2 = nalgebra::Point2<f64>;
/// A displacement in the (u, v) plane. `nalgebra::Vector2<f64>` (ADR-0001).
pub type Vec2 = nalgebra::Vector2<f64>;
/// A direction in the (u, v) plane: a [`Vec2`] of unit length.
/// `nalgebra::Unit<Vector2<f64>>` (ADR-0001).
pub type UnitVec2 = nalgebra::Unit<Vec2>;
/// A 3x3 matrix, column-major: a rotation, or a tensor such as the
/// inertia of a body. `nalgebra::Matrix3<f64>` (ADR-0001).
pub type Matrix3 = nalgebra::Matrix3<f64>;

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{PI, TAU};

    #[test]
    fn wrap_angle_lands_in_the_half_open_turn() {
        assert_eq!(wrap_angle(0.0), 0.0);
        assert_eq!(wrap_angle(-1e-300), 0.0);
        assert_eq!(wrap_angle(-1.0), TAU - 1.0);
        assert_eq!(wrap_angle(PI), PI);
        assert!(wrap_angle(-f64::EPSILON) < TAU);
        assert_eq!(wrap_angle(3.0 * TAU + 1.0), 1.0);
        assert_eq!(wrap_angle(-3.0 * TAU - 1.0), TAU - 1.0);
        assert!(wrap_angle(f64::NAN).is_nan());
        assert_eq!(wrap_angle(f64::INFINITY), f64::INFINITY);
    }
}
