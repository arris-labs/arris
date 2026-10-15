//! What `sweep` refuses once the path's frames are known (ADR-0063 §6):
//! a profile that lies along its path, a rotation-minimising frame that
//! does not close with a closed path, and a fold, where the carried
//! profile reaches past a curved span's radius of curvature.

use core::f64::consts::TAU;

use arris_geom::{Curve, FrameField, FrameLaw, ProfileEdge};
use arris_math::{Point3, Tolerance, Vec3};

use crate::error::{OpError, Reason, SweepReason};

use super::{degenerate, profile_curve_fault};

/// Intervals each span is first cut into when the fold test samples it.
const FOLD_SAMPLES: usize = 16;

/// How many times an interval near the limit is halved. A span of
/// length `L` is then sampled down to `L / (16 · 2^10)`, finer than the
/// frame field's own table; a structural bound, not a tolerance.
const FOLD_DEPTH: usize = 10;

/// The offset of a sample from the limit under which the test refines:
/// an interval whose ends both sit under half the limit is not cut.
const FOLD_NEAR: f64 = 0.5;

/// Differences of the carried reach under which an interval is not cut
/// again, as a fraction of the limit. Below it the reach is flat to the
/// precision the test can read from a finite difference of the tangent.
const FOLD_FLAT: f64 = 1e-3;

/// The step of the finite difference of the tangent that gives the
/// curvature, as a fraction of the span's length. The tangent is
/// smooth along a span, so the difference's error is of the order of
/// the step squared, far under the sample's own error.
const CURVATURE_STEP: f64 = 1e-5;

/// [`SweepReason::ProfileAlongPath`] when the path's start tangent lies
/// in the profile's plane within the angular tolerance: the profile
/// would be carried along its own plane and sweep no volume.
pub(super) fn along_path(field: &FrameField, normal: &Vec3, tol: Tolerance) -> Result<(), OpError> {
    let t = field.at(0.0).tangent;
    if t.dot(normal).abs().asin() <= tol.angular {
        return Err(degenerate(Reason::Sweep(SweepReason::ProfileAlongPath)));
    }
    Ok(())
}

/// [`SweepReason::FrameDoesNotClose`] when the path closes and the
/// rotation-minimising frame comes back turned by more than the
/// angular tolerance (its holonomy). A fixed binormal closes by
/// construction.
pub(super) fn frame_closes(field: &FrameField, tol: Tolerance) -> Result<(), OpError> {
    if field.is_closed()
        && matches!(field.law(), FrameLaw::RotationMinimizing)
        && field.holonomy().abs() > tol.angular
    {
        return Err(degenerate(Reason::Sweep(SweepReason::FrameDoesNotClose {
            angle: field.holonomy().abs(),
        })));
    }
    Ok(())
}

/// The farthest the `edges` reach from `origin` along `w`: the support
/// function of the loop, exact on each edge — a line at its ends, a
/// circle or an ellipse at its ends and, when the range holds it, at the
/// angle where its derivative along `w` vanishes.
fn support(edges: &[ProfileEdge], origin: &Point3, w: &Vec3) -> Result<f64, OpError> {
    let mut reach = f64::NEG_INFINITY;
    for edge in edges {
        let (lo, hi) = (edge.range.lo(), edge.range.hi());
        for t in [lo, hi] {
            reach = reach.max((edge.curve.point(t) - origin).dot(w));
        }
        let (centre, x, y, a, b) = match &edge.curve {
            Curve::Line { .. } => continue,
            Curve::Circle { frame, radius } => {
                (frame.origin(), frame.x(), frame.y(), *radius, *radius)
            }
            Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => (
                frame.origin(),
                frame.x(),
                frame.y(),
                *major_radius,
                *minor_radius,
            ),
            Curve::Nurbs(_) => return Err(profile_curve_fault(edge)),
        };
        // `c + a cos θ x + b sin θ y` along `w`: `A cos θ + B sin θ` has
        // its maximum `hypot(A, B)` at `atan2(B, A)` and its translates.
        let (big_a, big_b) = (a * x.dot(w), b * y.dot(w));
        let peak = big_b.atan2(big_a);
        let k = ((lo - peak) / TAU).ceil();
        if peak + k * TAU <= hi {
            reach = reach.max((centre - origin).dot(w) + big_a.hypot(big_b));
        }
    }
    Ok(reach)
}

/// How far the carried profile's reach along the curvature normal is
/// from the radius of curvature at arc length `s` of the span
/// `[lo, hi]`: `κ · (reach + tol)`, which is at least `1` where the
/// profile has folded. `0` on a straight run.
fn fold_ratio(
    field: &FrameField,
    edges: &[ProfileEdge],
    origin: &Point3,
    (lo, hi): (f64, f64),
    s: f64,
    tol: Tolerance,
) -> Result<f64, OpError> {
    let h = CURVATURE_STEP * (hi - lo);
    let s = s.clamp(lo + h, hi - h);
    let dt =
        (field.at(s + h).tangent.into_inner() - field.at(s - h).tangent.into_inner()) / (2.0 * h);
    let kappa = dt.norm();
    if kappa == 0.0 {
        return Ok(0.0);
    }
    // The profile is carried rigidly, so its reach along `n` at `s` is
    // its reach at the start along `n` pulled back by the motion.
    let w = field.motion_at(s).inverse().apply_vec(dt / kappa);
    Ok(kappa * (support(edges, origin, &w)? + tol.linear))
}

/// [`SweepReason::SweepFolds`] when along a curved span the carried
/// outer loop reaches the radius of curvature within the linear
/// tolerance: the face the loop sweeps there has a vanishing speed along
/// the path (`1 − κ · reach`), the profile has folded through the
/// path's centre of curvature. The span is sampled and the intervals
/// near the limit halved while the ratio still moves; a fold narrower
/// than the last interval is left to the checker. Revolve's
/// `ProfileCrossesAxis` and `SpindleTorus` are the same test on an arc.
pub(super) fn folds(
    field: &FrameField,
    outer: &[ProfileEdge],
    tol: Tolerance,
) -> Result<(), OpError> {
    let origin = field.at(0.0).origin;
    for k in 0..field.span_count() {
        let range = field.span_range(k);
        let (lo, hi) = range;
        let mut stack: Vec<(f64, f64, f64, f64, usize)> = Vec::new();
        let mut previous: Option<(f64, f64)> = None;
        for i in 0..=FOLD_SAMPLES {
            let s = lo + (hi - lo) * i as f64 / FOLD_SAMPLES as f64;
            let g = fold_ratio(field, outer, &origin, range, s, tol)?;
            if g >= 1.0 {
                return Err(fold(k, s));
            }
            if let Some((s0, g0)) = previous {
                stack.push((s0, g0, s, g, 0));
            }
            previous = Some((s, g));
        }
        while let Some((s0, g0, s1, g1, depth)) = stack.pop() {
            if depth >= FOLD_DEPTH || g0.max(g1) <= FOLD_NEAR || (g0 - g1).abs() <= FOLD_FLAT {
                continue;
            }
            let mid = 0.5 * (s0 + s1);
            let g = fold_ratio(field, outer, &origin, range, mid, tol)?;
            if g >= 1.0 {
                return Err(fold(k, mid));
            }
            stack.push((s0, g0, mid, g, depth + 1));
            stack.push((mid, g, s1, g1, depth + 1));
        }
    }
    Ok(())
}

fn fold(span: usize, at: f64) -> OpError {
    degenerate(Reason::Sweep(SweepReason::SweepFolds { span, at }))
}
