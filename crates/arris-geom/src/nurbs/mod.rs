//! Rational B-splines: `NurbsCurve`, `NurbsCurve2` and `NurbsSurface` per
//! `docs/DATA-MODEL.md` §NURBS — degree, knots, Cartesian control
//! points with positive weights kept apart, validating constructors,
//! evaluation with derivatives to second order into stack buffers (so it
//! never allocates), the period unclamped knots imply, and knot insertion
//! for curves, and least-squares fitting of a curve at a given
//! parametrisation — in 2D for the pcurves without an exact form, in 3D,
//! clamped or periodic, for the section curves without one — and of a
//! surface's offset over a parameter box and of a surface skinned through
//! compatible sections (ADR-0062). *The NURBS
//! Book* ch. 2–5 and §9.4 are the reference for the algorithms; the
//! reference tree's `truck-geometry` B-spline module was read for the
//! shape of a Rust implementation and nothing was taken from it: truck
//! stores homogeneous points and builds derivative curves, Arris keeps
//! weights separate and evaluates the basis derivatives in place.

mod basis;
mod critical;
mod curve;
mod exact;
mod fit;
mod line;
mod offset;
mod project;
mod skin;
mod spline;
mod surface;

pub use basis::MAX_DEGREE;
pub(crate) use critical::Field;
pub use curve::{NurbsCurve, NurbsCurve2};
pub use fit::{FitError, MAX_FIT_SPANS, fit_curve, fit_curve_periodic, fit_curve2};
pub(crate) use line::{conic_nurbs, line_nurbs};
pub use offset::{MAX_OFFSET_PATCHES, MAX_OFFSET_SPANS, OFFSET_FIT_DEGREE};
pub use skin::{fit_skin, interpolate_skin};
pub(crate) use spline::BezierSpan;
pub use surface::NurbsSurface;
