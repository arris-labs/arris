//! Rational B-splines: `NurbsCurve`, `NurbsCurve2` and `NurbsSurface` per
//! `docs/DATA-MODEL.md` §NURBS — degree, knots, Cartesian control
//! points with positive weights kept apart, validating constructors,
//! evaluation with derivatives to second order into stack buffers (so it
//! never allocates), the period unclamped knots imply, and knot insertion
//! for curves, and least-squares fitting of a curve at a given
//! parametrisation — in 2D for the pcurves without an exact form, in 3D,
//! clamped or periodic, for the section curves without one. *The NURBS
//! Book* ch. 2–5 and §9.4 are the reference for the algorithms; the
//! reference tree's `truck-geometry` B-spline module was read for the
//! shape of a Rust implementation and nothing was taken from it: truck
//! stores homogeneous points and builds derivative curves, Arris keeps
//! weights separate and evaluates the basis derivatives in place.

mod basis;
mod curve;
mod exact;
mod fit;
mod line;
mod project;
mod spline;
mod surface;

pub use basis::MAX_DEGREE;
pub use curve::{NurbsCurve, NurbsCurve2};
pub use fit::{FitError, MAX_FIT_SPANS, fit_curve, fit_curve_periodic, fit_curve2};
pub(crate) use line::line_nurbs;
pub(crate) use spline::BezierSpan;
pub use surface::NurbsSurface;
