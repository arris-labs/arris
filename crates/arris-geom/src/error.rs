//! The typed errors of the geometry crate.

use core::fmt;

use arris_math::{Interrupted, Point2, Point3, Tolerance};

use crate::{Curve2Kind, CurveKind, FitError, SectionFault, SurfaceKind};

/// The kind of a geometric operand, for errors that name a pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum GeomKind {
    /// A [`crate::Surface`] variant.
    Surface(SurfaceKind),
    /// A [`crate::Curve`] variant.
    Curve(CurveKind),
    /// A `Curve2` variant (`docs/DATA-MODEL.md` §Pcurves).
    Curve2(Curve2Kind),
    /// A point, the first operand of a projection.
    Point,
}

impl fmt::Display for GeomKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeomKind::Surface(k) => write!(f, "{k} surface"),
            GeomKind::Curve(k) => write!(f, "{k} curve"),
            GeomKind::Curve2(k) => write!(f, "{k} pcurve"),
            GeomKind::Point => f.write_str("point"),
        }
    }
}

impl From<SurfaceKind> for GeomKind {
    fn from(k: SurfaceKind) -> Self {
        GeomKind::Surface(k)
    }
}

impl From<CurveKind> for GeomKind {
    fn from(k: CurveKind) -> Self {
        GeomKind::Curve(k)
    }
}

impl From<Curve2Kind> for GeomKind {
    fn from(k: Curve2Kind) -> Self {
        GeomKind::Curve2(k)
    }
}

/// Where a projection has no unique answer: the locus a point was found
/// on. Every locus is a set of measure zero decided to rounding
/// ([`arris_math::is_negligible`]), never a tolerance, and it is reported
/// rather than resolved by a silent choice of parameter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AmbiguousLocus {
    /// The axis of a cylinder, cone or torus, or the axis of a circle
    /// through its centre: every point of a whole parallel is nearest.
    Axis,
    /// The centre of a sphere, or of a circle-like ellipse.
    Centre,
    /// The torus's centre circle, at the middle of the tube: every point
    /// of a whole meridian is nearest.
    CentreCircle,
    /// The plane through a cone's apex perpendicular to its axis: the two
    /// nappes are equally near.
    ApexPlane,
    /// The segment of an ellipse's major axis inside its evolute — or,
    /// on an elliptic cylinder, the strip that segment sweeps along the
    /// axis: two points, mirror images across the axis, are equally near.
    MajorAxis,
    /// The medial axis of a NURBS surface: two or more *distinct* points
    /// of it are as near as each other to rounding, so no parameter is
    /// the nearest without a guess.
    MedialAxis,
}

impl fmt::Display for AmbiguousLocus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            AmbiguousLocus::Axis => "the axis",
            AmbiguousLocus::Centre => "the centre",
            AmbiguousLocus::CentreCircle => "the centre circle",
            AmbiguousLocus::ApexPlane => "the plane through the apex",
            AmbiguousLocus::MajorAxis => "the major axis inside the evolute",
            AmbiguousLocus::MedialAxis => "the surface's medial axis",
        })
    }
}

/// Why a geometric query has no answer. Every variant names the operands
/// involved, so the message says *which* pair or *which* point, never
/// "projection failed" (`.agents/rules/kernel.md`).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum GeomError {
    /// The exhaustive dispatch reached a pair the kernel has no closed
    /// form for yet. Never a fallback: a wildcard arm is forbidden.
    #[error("no closed form for {a} against {b}")]
    Unsupported {
        /// The first operand's kind.
        a: GeomKind,
        /// The second operand's kind.
        b: GeomKind,
    },
    /// An operand has no valid representation for the query: a frame
    /// with a non-finite axis, a knot vector that is not one, a knot
    /// insertion that would break one.
    #[error("degenerate {kind}: {reason}")]
    Degenerate {
        /// The operand.
        kind: GeomKind,
        /// What is wrong with it.
        reason: String,
    },
    /// The tolerance handed to the query is not finite and positive in
    /// both parts, so no decision it makes would mean anything.
    #[error("tolerance {0:?} is not finite and positive")]
    InvalidTolerance(Tolerance),
    /// The point has no unique nearest point on the target, or its
    /// nearest point has no unique parameter that the query could return
    /// without guessing.
    #[error("projection of {point} onto a {kind} is ambiguous: the point is on {locus}")]
    Ambiguous {
        /// What was projected onto.
        kind: GeomKind,
        /// The locus the point lies on.
        locus: AmbiguousLocus,
        /// The point.
        point: Point3,
    },
    /// The (u, v) point has no unique nearest point on the pcurve.
    #[error("projection of {point} onto a {kind} is ambiguous: the point is on {locus}")]
    AmbiguousUv {
        /// What was projected onto.
        kind: GeomKind,
        /// The locus the point lies on.
        locus: AmbiguousLocus,
        /// The point.
        point: Point2,
    },
    /// The curve does not lie on the surface within the linear tolerance,
    /// so it has no pcurve there; `t` is the first sampled parameter that
    /// is off and `distance` how far.
    #[error("the {curve} at t = {t} is {distance} from the {surface}, outside the tolerance")]
    NotOnSurface {
        /// The curve.
        curve: GeomKind,
        /// The surface.
        surface: GeomKind,
        /// Where it was measured.
        t: f64,
        /// The distance there.
        distance: f64,
    },
    /// The curve runs through a singular point of the surface's
    /// parametrisation — a cone's apex, a sphere's pole — inside the range
    /// a pcurve was asked over, where `u` has no single value: `t` is the
    /// parameter of the curve's nearest approach, within
    /// [`crate::PCURVE_SINGULAR_BAND`] of `tol.linear` of the point, and
    /// the caller splits the range there. A range that *ends* on the
    /// point has a pcurve ([`crate::pcurve_on`]).
    #[error(
        "the {curve} runs through a singular point of the {surface} at t = {t}: split the range there"
    )]
    ThroughSingularity {
        /// The curve.
        curve: GeomKind,
        /// The surface whose singular point it is.
        surface: GeomKind,
        /// The curve's parameter nearest the point.
        t: f64,
    },
    /// The section of two quadrics is one the tracer
    /// ([`crate::trace_quadrics`]) does not resolve: a pose of measure
    /// zero that a closed form owns, or one that needs a decision the
    /// tracer will not guess.
    #[error("the section of the {a} and the {b} is degenerate: {fault}")]
    DegenerateSection {
        /// The walked surface.
        a: GeomKind,
        /// The other surface.
        b: GeomKind,
        /// What is degenerate about it.
        fault: SectionFault,
    },
    /// A NURBS fit ([`crate::fit_curve2`], [`crate::fit_curve`],
    /// [`crate::fit_curve_periodic`]) did not produce a curve.
    #[error("fit: {0}")]
    Fit(FitError),
    /// The caller stopped the query ([`arris_math::Control`]): nothing
    /// was computed that outlives the call.
    #[error(transparent)]
    Interrupted(Interrupted),
}

impl From<FitError> for GeomError {
    /// A fit the caller stopped is the query stopped, not a fit that
    /// failed.
    fn from(e: FitError) -> Self {
        match e {
            FitError::Interrupted(stop) => GeomError::Interrupted(stop),
            other => GeomError::Fit(other),
        }
    }
}

impl From<Interrupted> for GeomError {
    fn from(stop: Interrupted) -> Self {
        GeomError::Interrupted(stop)
    }
}
