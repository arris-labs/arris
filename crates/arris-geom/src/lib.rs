//! Geometry of the Arris kernel: analytic and NURBS surfaces and curves with
//! the parametrisations of `docs/DATA-MODEL.md` §Geometry, their
//! evaluation and derivatives, point projection, the curve/surface and
//! surface/surface intersections, the pcurves of curves on the analytic
//! surfaces, the consumer's sketch as a value (`profile`), and the
//! (u, v) toolkit — regions bounded by pcurve pieces (`region2`) and
//! integrals over them (`integrate`).
//!
//! Guarantees: `Surface`, `Curve` and `Curve2` are exhaustive enums, so a
//! new variant fails every dispatch to compile until it is handled; a pair
//! without a closed form is an explicit unsupported arm, never a wildcard.
//! Evaluation never panics and never allocates (a NURBS degree is bounded
//! by [`MAX_DEGREE`] so its buffers live on the stack). Every query that can fail
//! returns a [`GeomError`] naming its operands; a projection with no
//! unique answer is [`GeomError::Ambiguous`], never a guessed parameter.
//! Depends only on `arris-math`.
#![forbid(unsafe_code)]
#![warn(missing_docs)]

mod arc;
mod bernstein;
mod bernstein2;
mod by_distance;
mod cone_section;
mod conic2;
mod curve;
mod curve2;
mod error;
mod halt;
mod implicit;
pub mod integrate;
mod intersect;
mod intersect_curve;
mod intersect_curves;
mod intersect_spline;
mod meridian;
mod nurbs;
mod pcurve;
pub mod profile;
mod project;
pub mod region2;
mod section;
mod surface;
mod torus_walk;
mod trace;
mod trace_torus;

pub use cone_section::HYPERBOLA_HALF_SPAN;
pub use curve::{Curve, CurveEval, CurveKind};
pub use curve2::{Curve2, Curve2Eval, Curve2Kind, Curve2Projection};
pub use error::{AmbiguousLocus, GeomError, GeomKind};
pub use intersect::{MeetCurve, MeetKind, MeetPoint, SurfaceIntersection, intersect_surfaces};
pub use intersect_curve::{CurveSurfaceHit, CurveSurfaceIntersection, intersect_curve_surface};
pub use intersect_curves::{
    CurveCurveHit, CurveIntersection, conic_crossings, curves_coincide, intersect_curves,
};
pub use nurbs::{
    FitError, MAX_DEGREE, MAX_FIT_SPANS, NurbsCurve, NurbsCurve2, NurbsSurface, fit_curve,
    fit_curve_periodic, fit_curve2,
};
pub use pcurve::{
    PCURVE_FIT_DEGREE, PCURVE_SAMPLES, PCURVE_SINGULAR_BAND, pcurve_ending_on, pcurve_on,
    project_to_plane,
};
pub use profile::{Profile, ProfileEdge, ProfileError, ProfileLoop, ProfileSegment};
pub use project::{CurveProjection, SurfaceProjection};
pub use section::{SECTION_FIT_DEGREE, SECTION_FIT_FRACTION};
pub use surface::{ParamMap, Singularity, Surface, SurfaceEval, SurfaceKind};
pub use torus_walk::trace_torus;
pub use trace::{
    BranchEnd, SectionBranch, SectionCircle, SectionFault, SectionPoint, SectionTrace,
    trace_quadrics,
};
