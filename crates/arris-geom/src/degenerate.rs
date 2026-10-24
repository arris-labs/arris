//! Why an operand is degenerate: the typed reasons behind
//! [`GeomError::Degenerate`](crate::GeomError::Degenerate).
//!
//! One variant per distinct reason, grouped by the family that raises it.
//! Each `Display` is the sentence the free-text reason used to be, so a
//! message a caller or the Python binding shows is unchanged; a caller that
//! needs to act on the cause matches the variant instead of the text.

use arris_math::roots::RootError;
use arris_math::{FrameError, Point3, Vec3};

use crate::{AmbiguousLocus, MAX_DEGREE};

/// A direction of a NURBS net: `u` or `v`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum NetDirection {
    /// The first parameter.
    U,
    /// The second parameter.
    V,
}

impl core::fmt::Display for NetDirection {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            NetDirection::U => "u",
            NetDirection::V => "v",
        })
    }
}

/// What is wrong with an operand of a geometric query.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum DegenerateReason {
    /// A frame or axis the query needs is not one.
    #[error(transparent)]
    Frame(FrameFault),
    /// A root-finding step of an intersection found nothing it could use.
    #[error(transparent)]
    Roots(RootsFault),
    /// A knot vector or weight list is not a spline's.
    #[error(transparent)]
    Spline(SplineFault),
    /// A NURBS surface's net is not one.
    #[error(transparent)]
    Net(NetFault),
    /// The sections of a skin do not make one.
    #[error(transparent)]
    Skin(SkinFault),
    /// A NURBS surface cannot be offset as asked.
    #[error(transparent)]
    Offset(OffsetFault),
    /// A pcurve cannot be built as asked.
    #[error(transparent)]
    Pcurve(PcurveFault),
    /// An exact rational form cannot be built for the given parameters.
    #[error(transparent)]
    Exact(ExactFault),
    /// A branch has no parameters on the operand asked for.
    #[error("a branch has no parameters on operand {surface}")]
    BranchNoParameters {
        /// The operand index.
        surface: usize,
    },
    /// The point to project is not finite.
    #[error("the point {point} to project is not finite")]
    PointNotFinite {
        /// The point.
        point: Point3,
    },
    /// The projection search ended without a candidate.
    #[error("the search found no candidate to project onto")]
    NoProjectionCandidate,
    /// A profile loop has no segment to lift.
    #[error("a profile loop with a segment")]
    ProfileLoopEmpty,
    /// Two sections of a loft are cut into different numbers of pieces.
    #[error("a loop cut into as many pieces as the other sections")]
    LoopPiecesDiffer,
}

impl From<FrameFault> for Box<DegenerateReason> {
    fn from(fault: FrameFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<FrameFault> for DegenerateReason {
    fn from(fault: FrameFault) -> Self {
        DegenerateReason::Frame(fault)
    }
}
impl From<RootsFault> for Box<DegenerateReason> {
    fn from(fault: RootsFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<RootsFault> for DegenerateReason {
    fn from(fault: RootsFault) -> Self {
        DegenerateReason::Roots(fault)
    }
}
impl From<SplineFault> for Box<DegenerateReason> {
    fn from(fault: SplineFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<SplineFault> for DegenerateReason {
    fn from(fault: SplineFault) -> Self {
        DegenerateReason::Spline(fault)
    }
}
impl From<NetFault> for Box<DegenerateReason> {
    fn from(fault: NetFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<NetFault> for DegenerateReason {
    fn from(fault: NetFault) -> Self {
        DegenerateReason::Net(fault)
    }
}
impl From<SkinFault> for Box<DegenerateReason> {
    fn from(fault: SkinFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<SkinFault> for DegenerateReason {
    fn from(fault: SkinFault) -> Self {
        DegenerateReason::Skin(fault)
    }
}
impl From<OffsetFault> for Box<DegenerateReason> {
    fn from(fault: OffsetFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<OffsetFault> for DegenerateReason {
    fn from(fault: OffsetFault) -> Self {
        DegenerateReason::Offset(fault)
    }
}
impl From<PcurveFault> for Box<DegenerateReason> {
    fn from(fault: PcurveFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<PcurveFault> for DegenerateReason {
    fn from(fault: PcurveFault) -> Self {
        DegenerateReason::Pcurve(fault)
    }
}
impl From<ExactFault> for Box<DegenerateReason> {
    fn from(fault: ExactFault) -> Self {
        Box::new(DegenerateReason::from(fault))
    }
}
impl From<ExactFault> for DegenerateReason {
    fn from(fault: ExactFault) -> Self {
        DegenerateReason::Exact(fault)
    }
}

/// A frame, axis or direction the query needs is not one.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum FrameFault {
    /// Only a non-finite frame reaches the constructor.
    #[error("non-finite frame")]
    NonFinite,
    /// A non-finite frame or radius in a quadric's polynomial.
    #[error("a non-finite frame or radius")]
    NonFiniteOrRadius,
    /// A line's direction gives no frame.
    #[error("the line's direction has no frame: {0}")]
    LineDirection(FrameError),
    /// A line's direction is not a direction.
    #[error("a line whose direction is not a direction")]
    LineNotADirection,
    /// A line in a conic's plane whose direction is not a direction.
    #[error("a line in the conic's plane whose direction is not a direction")]
    CoplanarLineNotADirection,
    /// A conic lies in a plane perpendicular to its own.
    #[error("a conic inside a plane perpendicular to its own")]
    ConicInPerpendicularPlane,
    /// A conic's radii are not positive.
    #[error("the conic's radii are not positive")]
    ConicRadiiNotPositive,
    /// A plane perpendicular to a cone's axis, which the meridian owns.
    #[error("a plane perpendicular to the axis")]
    PlanePerpendicularToAxis,
    /// A conic's axis is square to the plane the other lies in.
    #[error("a conic axis square to the plane it lies in")]
    ConicAxisSquareToPlane,
    /// A conic's axis has no part in the cylinder's section.
    #[error("a conic axis has no part in the section")]
    ConicAxisNoPartInSection,
}

/// A root-finding step found nothing usable.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum RootsFault {
    /// The two sections' meeting.
    #[error("the sections' meeting: {0}")]
    SectionsMeeting(RootError),
    /// A conic against a cylinder's section.
    #[error("the conic against the section: {0}")]
    ConicAgainstSection(RootError),
    /// Two conics in one plane.
    #[error("two conics in one plane: {0}")]
    ConicsInPlane(RootError),
    /// The extrema of the distance from a line to a torus.
    #[error("extrema of the distance to a torus: {0}")]
    TorusExtrema(RootError),
    /// The radial extrema of a conic.
    #[error("radial extrema: {0}")]
    RadialExtrema(RootError),
    /// The extrema along a conic.
    #[error("the extrema along the conic: {0}")]
    ConicExtrema(RootError),
    /// The radial extrema along a conic.
    #[error("the radial extrema along the conic: {0}")]
    ConicRadialExtrema(RootError),
    /// The crossings of a cone's apex plane.
    #[error("the crossings of the apex plane: {0}")]
    ConicApexCrossings(RootError),
    /// A crossing bracket that is not an interval.
    #[error("crossing bracket [{lo}, {hi}]")]
    CrossingBracket {
        /// The bracket's low end.
        lo: f64,
        /// The bracket's high end.
        hi: f64,
    },
    /// A crossing the root finder did not converge on.
    #[error("crossing in [{lo}, {hi}]: {error}")]
    Crossing {
        /// The bracket's low end.
        lo: f64,
        /// The bracket's high end.
        hi: f64,
        /// What the root finder said.
        error: RootError,
    },
}

/// A knot vector, point list or weight list is not a spline's.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SplineFault {
    /// The degree is outside `1..=MAX_DEGREE`.
    #[error("degree {degree} is not in 1..={}", MAX_DEGREE)]
    Degree {
        /// The degree given.
        degree: usize,
    },
    /// Fewer than `degree + 1` control points.
    #[error("{points} control points for degree {degree}: at least {} are needed", .degree + 1)]
    TooFewPoints {
        /// The count given.
        points: usize,
        /// The degree.
        degree: usize,
    },
    /// The knot count is not `points + degree + 1`.
    #[error(
        "{knots} knots for degree {degree} and {points} control points: {} are needed",
        .points + .degree + 1
    )]
    KnotCount {
        /// The count given.
        knots: usize,
        /// The degree.
        degree: usize,
        /// The control point count.
        points: usize,
    },
    /// A knot is NaN or infinite.
    #[error("knot {index} is not finite")]
    KnotNotFinite {
        /// The knot's index.
        index: usize,
    },
    /// Two consecutive knots decrease.
    #[error("knots {index} and {} decrease: {first} > {second}", .index + 1)]
    KnotsDecrease {
        /// The first knot's index.
        index: usize,
        /// Its value.
        first: f64,
        /// The next knot's value.
        second: f64,
    },
    /// A non-empty span is subnormal.
    #[error("knots {index} and {} are a subnormal span apart: {first} and {second}", .index + 1)]
    SubnormalSpan {
        /// The first knot's index.
        index: usize,
        /// Its value.
        first: f64,
        /// The next knot's value.
        second: f64,
    },
    /// The domain has no length.
    #[error("the domain [{lo}, {hi}] is empty")]
    DomainEmpty {
        /// The domain's low end.
        lo: f64,
        /// The domain's high end.
        hi: f64,
    },
    /// The last span of the domain is empty.
    #[error("the span ending at the domain's end {hi} is empty")]
    LastSpanEmpty {
        /// The domain's high end.
        hi: f64,
    },
    /// A knot repeats more than `degree + 1` times.
    #[error("knot {value} has multiplicity {run}, above degree + 1 = {}", .degree + 1)]
    Multiplicity {
        /// The knot's value.
        value: f64,
        /// Its multiplicity.
        run: usize,
        /// The degree.
        degree: usize,
    },
    /// An interior knot repeats more than `degree` times.
    #[error("interior knot {value} has multiplicity {run}, above the degree {degree}")]
    InteriorMultiplicity {
        /// The knot's value.
        value: f64,
        /// Its multiplicity.
        run: usize,
        /// The degree.
        degree: usize,
    },
    /// The weight count differs from the control point count.
    #[error("{weights} weights for {points} control points")]
    WeightCount {
        /// The weights given.
        weights: usize,
        /// The control points given.
        points: usize,
    },
    /// A control point is not finite.
    #[error("control point {index} is not finite")]
    PointNotFinite {
        /// The point's index.
        index: usize,
    },
    /// A weight is not finite and positive.
    #[error("weight {index} is {weight}: weights are finite and positive")]
    Weight {
        /// The weight's index.
        index: usize,
        /// Its value.
        weight: f64,
    },
    /// A knot to insert is outside the domain.
    #[error("knot {knot} is outside the domain [{lo}, {hi})")]
    KnotOutsideDomain {
        /// The knot.
        knot: f64,
        /// The domain's low end.
        lo: f64,
        /// The domain's high end.
        hi: f64,
    },
    /// An insertion would raise a knot past the degree.
    #[error(
        "inserting {knot} {times} more times would give it multiplicity {}, above the degree {degree}",
        .already + .times
    )]
    InsertionMultiplicity {
        /// The knot.
        knot: f64,
        /// Insertions asked.
        times: usize,
        /// Its multiplicity now.
        already: usize,
        /// The degree.
        degree: usize,
    },
    /// A segment's ends are not a range.
    #[error("[{lo}, {hi}] is no range")]
    NoRange {
        /// The low end given.
        lo: f64,
        /// The high end given.
        hi: f64,
    },
    /// A segment of a periodic spline is longer than its period.
    #[error("[{lo}, {hi}] is longer than the period {period}")]
    LongerThanPeriod {
        /// The low end given.
        lo: f64,
        /// The high end given.
        hi: f64,
        /// The period.
        period: f64,
    },
    /// A segment lies outside the domain.
    #[error("[{lo}, {hi}] is outside the domain [{domain_lo}, {domain_hi}]")]
    OutsideDomain {
        /// The low end given.
        lo: f64,
        /// The high end given.
        hi: f64,
        /// The domain's low end.
        domain_lo: f64,
        /// The domain's high end.
        domain_hi: f64,
    },
    /// A knot the insertion put in is missing from the result.
    #[error("a knot the insertion put in is missing")]
    InsertedKnotMissing,
}

/// A NURBS surface's net is not one.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum NetFault {
    /// The degree along a direction is outside `1..=MAX_DEGREE`.
    #[error("{direction}: degree {degree} is not in 1..={}", MAX_DEGREE)]
    Degree {
        /// The direction.
        direction: NetDirection,
        /// The degree given.
        degree: usize,
    },
    /// The knot count leaves too few control points.
    #[error(
        "{direction}: {knots} knots for degree {degree} leave fewer than {} control points",
        .degree + 1
    )]
    KnotCount {
        /// The direction.
        direction: NetDirection,
        /// The knots given.
        knots: usize,
        /// The degree.
        degree: usize,
    },
    /// The knot vector along a direction is not a spline's.
    #[error("{direction}: {fault}")]
    Knots {
        /// The direction.
        direction: NetDirection,
        /// What is wrong.
        fault: SplineFault,
    },
    /// The control point count is not `n × m`.
    #[error("{points} control points for a {n} × {m} net")]
    PointCount {
        /// The points given.
        points: usize,
        /// The count along `u`.
        n: usize,
        /// The count along `v`.
        m: usize,
    },
    /// The weight count differs from the control point count.
    #[error("{weights} weights for {points} control points")]
    WeightCount {
        /// The weights given.
        weights: usize,
        /// The control points given.
        points: usize,
    },
    /// A control point is not finite.
    #[error("control point ({u}, {v}) is not finite")]
    PointNotFinite {
        /// The point's row.
        u: usize,
        /// The point's column.
        v: usize,
    },
    /// A weight is not finite and positive.
    #[error("weight ({u}, {v}) is {weight}: weights are finite and positive")]
    Weight {
        /// The weight's row.
        u: usize,
        /// The weight's column.
        v: usize,
        /// Its value.
        weight: f64,
    },
    /// A control row index is past the net.
    #[error("control row {row} of {rows}")]
    ControlRow {
        /// The row asked.
        row: usize,
        /// The rows in the net.
        rows: usize,
    },
}

/// The sections of a skin do not make one.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum SkinFault {
    /// A section sampled at `v` differs from the first in degree or knots.
    #[error(
        "the section at v = {v} is not compatible with the first: degree {degree} \
         and {knots} knots, where the first has degree {first_degree} and {first_knots} knots"
    )]
    SectionAt {
        /// The parameter sampled.
        v: f64,
        /// The section's degree.
        degree: usize,
        /// The section's knot count.
        knots: usize,
        /// The first section's degree.
        first_degree: usize,
        /// The first section's knot count.
        first_knots: usize,
    },
    /// Unifying the weights needs a degree past the maximum.
    #[error(
        "{classes} distinct weight functions need degree {degree}, above {}",
        MAX_DEGREE
    )]
    WeightClasses {
        /// The distinct weight functions.
        classes: usize,
        /// The degree they need.
        degree: usize,
    },
    /// No sections at all.
    #[error("a skin through no sections")]
    NoSections,
    /// Fewer than two sections.
    #[error("a skin through {sections} section, at least two are needed")]
    TooFewSections {
        /// The sections given.
        sections: usize,
    },
    /// The parameter count differs from the section count.
    #[error("{params} parameters for {sections} sections")]
    ParameterCount {
        /// The parameters given.
        params: usize,
        /// The sections, the closing copy included.
        sections: usize,
    },
    /// A parameter is not finite.
    #[error("parameter {index} is not finite")]
    ParameterNotFinite {
        /// The parameter's index.
        index: usize,
    },
    /// Two consecutive parameters do not increase.
    #[error("parameters {index} and {} do not increase: {first} and {second}", .index + 1)]
    ParametersNotIncreasing {
        /// The first parameter's index.
        index: usize,
        /// Its value.
        first: f64,
        /// The next one's value.
        second: f64,
    },
    /// The degree along `v` is outside `1..sections`.
    #[error("degree {degree} is not in 1..{sections} for {sections} sections")]
    Degree {
        /// The degree given.
        degree: usize,
        /// The sections, the closing copy included.
        sections: usize,
    },
    /// A section differs from the first in degree, knots or count.
    #[error("section {index} is not compatible with the first")]
    SectionIncompatible {
        /// The section's index.
        index: usize,
    },
    /// The collocation matrix is singular at a parameter.
    #[error("the interpolation at parameter {parameter} is singular")]
    Singular {
        /// The parameter.
        parameter: f64,
    },
    /// An interpolated weight is not positive.
    #[error("the interpolated weight of control point {point} at row {row} is {weight}")]
    InterpolatedWeight {
        /// The control point's index in a section.
        point: usize,
        /// The row.
        row: usize,
        /// The weight.
        weight: f64,
    },
}

/// A NURBS surface cannot be offset as asked.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum OffsetFault {
    /// The distance is NaN or infinite.
    #[error("the offset distance {distance} is not finite")]
    DistanceNotFinite {
        /// The distance.
        distance: f64,
    },
    /// A range is not bounded or has no length.
    #[error(
        "the offset's {direction} range [{lo}, {hi}] is not a bounded interval of positive length"
    )]
    RangeNotBounded {
        /// The direction.
        direction: NetDirection,
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// A collapsed row lies in the offset's box.
    #[error(
        "a collapsed row at {direction} = {value} lies in the offset's box: the normal is \
         undefined along it"
    )]
    CollapsedRow {
        /// The direction the row is constant in.
        direction: NetDirection,
        /// The parameter value of the row.
        value: f64,
    },
    /// The surface has no normal at a sample.
    #[error("no normal at ({u}, {v})")]
    NoNormal {
        /// The sample's `u`.
        u: f64,
        /// The sample's `v`.
        v: f64,
    },
}

/// A pcurve cannot be built as asked.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum PcurveFault {
    /// The range is not bounded or has no length.
    #[error("a pcurve needs a bounded range of positive length, not [{lo}, {hi}]")]
    Range {
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// A line perpendicular to the plane it should lie in.
    #[error("the line is perpendicular to the plane")]
    LinePerpendicular,
    /// A line perpendicular to the plane it is projected onto.
    #[error("the line is perpendicular to the plane: its projection is a point")]
    LineProjectsToPoint,
    /// A conic's plane is perpendicular to the target plane.
    #[error("the conic's plane is perpendicular to the target: its projection is a segment")]
    ConicProjectsToSegment,
    /// A conic's axes do not make a frame in the plane.
    #[error("conic axes in the plane: {0:?}")]
    ConicAxes(FrameError),
    /// The projected axes of an ellipse do not make a frame.
    #[error("projected axes: {0:?}")]
    ProjectedAxes(FrameError),
    /// A ruling meets the ellipse at an ambiguous place.
    #[error("the ruling's section point is on {0} of the ellipse")]
    RulingAmbiguous(AmbiguousLocus),
    /// A cone's half-angle is outside `(0, π/2)`.
    #[error("a cone's half-angle is in (0, π/2), not {half_angle}")]
    ConeHalfAngle {
        /// The half-angle.
        half_angle: f64,
    },
    /// A ruling has no radial direction.
    #[error("the ruling has no radial direction")]
    RulingNoRadial,
    /// A curve arrives at a singular point with no tangent.
    #[error("arrives at a singular point with no tangent")]
    SingularNoTangent,
    /// The curve winds faster than the sampling resolves.
    #[error(
        "winds {swing} of a turn around the axis between two of {samples} samples halved {depth} times: faster than the pcurve sampling resolves"
    )]
    WindsTooFast {
        /// The swing as a fraction of the turn.
        swing: f64,
        /// The samples taken.
        samples: usize,
        /// The halvings tried.
        depth: usize,
    },
}

/// An exact rational form cannot be built for the given parameters.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ExactFault {
    /// A quantity is not finite and positive.
    #[error("{what} {value} is not finite and positive")]
    NotPositive {
        /// The quantity's name.
        what: &'static str,
        /// Its value.
        value: f64,
    },
    /// The angle range is not finite.
    #[error("the angle range [{lo}, {hi}] is not finite")]
    AngleRangeNotFinite {
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// The angle range is empty.
    #[error("the angle range [{lo}, {hi}] is empty")]
    AngleRangeEmpty {
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// The angle range is longer than a full turn.
    #[error("the angle range [{lo}, {hi}] is longer than a full turn")]
    AngleRangeTooLong {
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// A hyperbola's range overflows its rational form.
    #[error("the range [{lo}, {hi}] is too long for a hyperbola's rational form")]
    HyperbolaRangeTooLong {
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// A range is empty or not finite.
    #[error("the range [{lo}, {hi}] is empty or not finite")]
    RangeEmpty {
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// The extrusion direction is zero or not finite.
    #[error("the extrusion direction {direction:?} is zero or not finite")]
    ExtrusionDirection {
        /// The direction.
        direction: Vec3,
    },
    /// The extrusion range is empty or not finite.
    #[error("the extrusion range [{lo}, {hi}] is empty or not finite")]
    ExtrusionRange {
        /// The range's low end.
        lo: f64,
        /// The range's high end.
        hi: f64,
    },
    /// The parameter bounds of a conversion are not finite.
    #[error("the parameter bounds are not finite")]
    BoundsNotFinite,
    /// The axes of a revolved quadric's meridian do not make a frame.
    #[error("{0}")]
    Frame(FrameError),
    /// A curve to revolve lies on the axis.
    #[error("the curve lies on the axis: nothing to revolve")]
    CurveOnAxis,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn said(reason: impl Into<DegenerateReason>) -> String {
        reason.into().to_string()
    }

    #[test]
    fn every_reason_says_what_the_free_text_did() {
        let frame = FrameError::NonFinite;
        let root = RootError::Zero;
        let cases: Vec<(DegenerateReason, String)> = vec![
            (FrameFault::NonFinite.into(), "non-finite frame".into()),
            (
                FrameFault::NonFiniteOrRadius.into(),
                "a non-finite frame or radius".into(),
            ),
            (
                FrameFault::LineDirection(frame).into(),
                format!("the line's direction has no frame: {frame}"),
            ),
            (
                FrameFault::LineNotADirection.into(),
                "a line whose direction is not a direction".into(),
            ),
            (
                FrameFault::CoplanarLineNotADirection.into(),
                "a line in the conic's plane whose direction is not a direction".into(),
            ),
            (
                FrameFault::ConicInPerpendicularPlane.into(),
                "a conic inside a plane perpendicular to its own".into(),
            ),
            (
                FrameFault::ConicRadiiNotPositive.into(),
                "the conic's radii are not positive".into(),
            ),
            (
                FrameFault::PlanePerpendicularToAxis.into(),
                "a plane perpendicular to the axis".into(),
            ),
            (
                FrameFault::ConicAxisSquareToPlane.into(),
                "a conic axis square to the plane it lies in".into(),
            ),
            (
                FrameFault::ConicAxisNoPartInSection.into(),
                "a conic axis has no part in the section".into(),
            ),
            (
                RootsFault::SectionsMeeting(root).into(),
                format!("the sections' meeting: {root}"),
            ),
            (
                RootsFault::ConicAgainstSection(root).into(),
                format!("the conic against the section: {root}"),
            ),
            (
                RootsFault::TorusExtrema(root).into(),
                format!("extrema of the distance to a torus: {root}"),
            ),
            (
                RootsFault::RadialExtrema(root).into(),
                format!("radial extrema: {root}"),
            ),
            (
                RootsFault::ConicExtrema(root).into(),
                format!("the extrema along the conic: {root}"),
            ),
            (
                RootsFault::ConicRadialExtrema(root).into(),
                format!("the radial extrema along the conic: {root}"),
            ),
            (
                RootsFault::ConicApexCrossings(root).into(),
                format!("the crossings of the apex plane: {root}"),
            ),
            (
                RootsFault::CrossingBracket { lo: 1.0, hi: 0.5 }.into(),
                "crossing bracket [1, 0.5]".into(),
            ),
            (
                RootsFault::Crossing {
                    lo: 1.0,
                    hi: 2.0,
                    error: root,
                }
                .into(),
                format!("crossing in [1, 2]: {root}"),
            ),
            (
                SplineFault::Degree { degree: 0 }.into(),
                format!("degree 0 is not in 1..={MAX_DEGREE}"),
            ),
            (
                SplineFault::TooFewPoints {
                    points: 2,
                    degree: 3,
                }
                .into(),
                "2 control points for degree 3: at least 4 are needed".into(),
            ),
            (
                SplineFault::KnotCount {
                    knots: 5,
                    degree: 2,
                    points: 4,
                }
                .into(),
                "5 knots for degree 2 and 4 control points: 7 are needed".into(),
            ),
            (
                SplineFault::KnotNotFinite { index: 3 }.into(),
                "knot 3 is not finite".into(),
            ),
            (
                SplineFault::KnotsDecrease {
                    index: 2,
                    first: 1.0,
                    second: 0.5,
                }
                .into(),
                "knots 2 and 3 decrease: 1 > 0.5".into(),
            ),
            (
                SplineFault::SubnormalSpan {
                    index: 2,
                    first: 0.0,
                    second: 1e-320,
                }
                .into(),
                format!(
                    "knots 2 and 3 are a subnormal span apart: 0 and {}",
                    1e-320_f64
                ),
            ),
            (
                SplineFault::DomainEmpty { lo: 1.0, hi: 1.0 }.into(),
                "the domain [1, 1] is empty".into(),
            ),
            (
                SplineFault::LastSpanEmpty { hi: 2.0 }.into(),
                "the span ending at the domain's end 2 is empty".into(),
            ),
            (
                SplineFault::Multiplicity {
                    value: 1.0,
                    run: 5,
                    degree: 2,
                }
                .into(),
                "knot 1 has multiplicity 5, above degree + 1 = 3".into(),
            ),
            (
                SplineFault::InteriorMultiplicity {
                    value: 1.0,
                    run: 3,
                    degree: 2,
                }
                .into(),
                "interior knot 1 has multiplicity 3, above the degree 2".into(),
            ),
            (
                SplineFault::WeightCount {
                    weights: 2,
                    points: 3,
                }
                .into(),
                "2 weights for 3 control points".into(),
            ),
            (
                SplineFault::PointNotFinite { index: 1 }.into(),
                "control point 1 is not finite".into(),
            ),
            (
                SplineFault::Weight {
                    index: 1,
                    weight: 0.0,
                }
                .into(),
                "weight 1 is 0: weights are finite and positive".into(),
            ),
            (
                SplineFault::KnotOutsideDomain {
                    knot: 2.0,
                    lo: 0.0,
                    hi: 1.0,
                }
                .into(),
                "knot 2 is outside the domain [0, 1)".into(),
            ),
            (
                SplineFault::InsertionMultiplicity {
                    knot: 1.0,
                    times: 2,
                    already: 1,
                    degree: 2,
                }
                .into(),
                "inserting 1 2 more times would give it multiplicity 3, above the degree 2".into(),
            ),
            (
                SplineFault::NoRange { lo: 1.0, hi: 0.0 }.into(),
                "[1, 0] is no range".into(),
            ),
            (
                SplineFault::LongerThanPeriod {
                    lo: 0.0,
                    hi: 3.0,
                    period: 2.0,
                }
                .into(),
                "[0, 3] is longer than the period 2".into(),
            ),
            (
                SplineFault::OutsideDomain {
                    lo: 0.0,
                    hi: 3.0,
                    domain_lo: 0.5,
                    domain_hi: 2.0,
                }
                .into(),
                "[0, 3] is outside the domain [0.5, 2]".into(),
            ),
            (
                SplineFault::InsertedKnotMissing.into(),
                "a knot the insertion put in is missing".into(),
            ),
            (
                NetFault::Degree {
                    direction: NetDirection::U,
                    degree: 0,
                }
                .into(),
                format!("u: degree 0 is not in 1..={MAX_DEGREE}"),
            ),
            (
                NetFault::KnotCount {
                    direction: NetDirection::V,
                    knots: 3,
                    degree: 2,
                }
                .into(),
                "v: 3 knots for degree 2 leave fewer than 3 control points".into(),
            ),
            (
                NetFault::Knots {
                    direction: NetDirection::U,
                    fault: SplineFault::KnotNotFinite { index: 0 },
                }
                .into(),
                "u: knot 0 is not finite".into(),
            ),
            (
                NetFault::PointCount {
                    points: 5,
                    n: 2,
                    m: 3,
                }
                .into(),
                "5 control points for a 2 × 3 net".into(),
            ),
            (
                NetFault::WeightCount {
                    weights: 5,
                    points: 6,
                }
                .into(),
                "5 weights for 6 control points".into(),
            ),
            (
                NetFault::PointNotFinite { u: 1, v: 2 }.into(),
                "control point (1, 2) is not finite".into(),
            ),
            (
                NetFault::Weight {
                    u: 1,
                    v: 2,
                    weight: -1.0,
                }
                .into(),
                "weight (1, 2) is -1: weights are finite and positive".into(),
            ),
            (
                NetFault::ControlRow { row: 4, rows: 3 }.into(),
                "control row 4 of 3".into(),
            ),
            (
                SkinFault::SectionAt {
                    v: 0.5,
                    degree: 2,
                    knots: 6,
                    first_degree: 3,
                    first_knots: 8,
                }
                .into(),
                "the section at v = 0.5 is not compatible with the first: degree 2 \
                 and 6 knots, where the first has degree 3 and 8 knots"
                    .into(),
            ),
            (
                SkinFault::WeightClasses {
                    classes: 4,
                    degree: 30,
                }
                .into(),
                format!("4 distinct weight functions need degree 30, above {MAX_DEGREE}"),
            ),
            (
                SkinFault::NoSections.into(),
                "a skin through no sections".into(),
            ),
            (
                SkinFault::TooFewSections { sections: 1 }.into(),
                "a skin through 1 section, at least two are needed".into(),
            ),
            (
                SkinFault::ParameterCount {
                    params: 2,
                    sections: 3,
                }
                .into(),
                "2 parameters for 3 sections".into(),
            ),
            (
                SkinFault::ParameterNotFinite { index: 1 }.into(),
                "parameter 1 is not finite".into(),
            ),
            (
                SkinFault::ParametersNotIncreasing {
                    index: 1,
                    first: 2.0,
                    second: 1.0,
                }
                .into(),
                "parameters 1 and 2 do not increase: 2 and 1".into(),
            ),
            (
                SkinFault::Degree {
                    degree: 3,
                    sections: 3,
                }
                .into(),
                "degree 3 is not in 1..3 for 3 sections".into(),
            ),
            (
                SkinFault::SectionIncompatible { index: 2 }.into(),
                "section 2 is not compatible with the first".into(),
            ),
            (
                SkinFault::Singular { parameter: 0.5 }.into(),
                "the interpolation at parameter 0.5 is singular".into(),
            ),
            (
                SkinFault::InterpolatedWeight {
                    point: 1,
                    row: 2,
                    weight: -0.5,
                }
                .into(),
                "the interpolated weight of control point 1 at row 2 is -0.5".into(),
            ),
            (
                OffsetFault::DistanceNotFinite { distance: f64::NAN }.into(),
                "the offset distance NaN is not finite".into(),
            ),
            (
                OffsetFault::RangeNotBounded {
                    direction: NetDirection::V,
                    lo: 0.0,
                    hi: 0.0,
                }
                .into(),
                "the offset's v range [0, 0] is not a bounded interval of positive length".into(),
            ),
            (
                OffsetFault::CollapsedRow {
                    direction: NetDirection::U,
                    value: 0.5,
                }
                .into(),
                "a collapsed row at u = 0.5 lies in the offset's box: the normal is \
                 undefined along it"
                    .into(),
            ),
            (
                OffsetFault::NoNormal { u: 0.5, v: 1.5 }.into(),
                "no normal at (0.5, 1.5)".into(),
            ),
            (
                PcurveFault::Range { lo: 0.0, hi: 0.0 }.into(),
                "a pcurve needs a bounded range of positive length, not [0, 0]".into(),
            ),
            (
                PcurveFault::LinePerpendicular.into(),
                "the line is perpendicular to the plane".into(),
            ),
            (
                PcurveFault::LineProjectsToPoint.into(),
                "the line is perpendicular to the plane: its projection is a point".into(),
            ),
            (
                PcurveFault::ConicProjectsToSegment.into(),
                "the conic's plane is perpendicular to the target: its projection is a segment"
                    .into(),
            ),
            (
                PcurveFault::ConicAxes(frame).into(),
                format!("conic axes in the plane: {frame:?}"),
            ),
            (
                PcurveFault::ProjectedAxes(frame).into(),
                format!("projected axes: {frame:?}"),
            ),
            (
                PcurveFault::RulingAmbiguous(AmbiguousLocus::Axis).into(),
                "the ruling's section point is on the axis of the ellipse".into(),
            ),
            (
                PcurveFault::ConeHalfAngle { half_angle: 2.0 }.into(),
                "a cone's half-angle is in (0, π/2), not 2".into(),
            ),
            (
                PcurveFault::RulingNoRadial.into(),
                "the ruling has no radial direction".into(),
            ),
            (
                PcurveFault::SingularNoTangent.into(),
                "arrives at a singular point with no tangent".into(),
            ),
            (
                PcurveFault::WindsTooFast {
                    swing: 0.5,
                    samples: 9,
                    depth: 32,
                }
                .into(),
                "winds 0.5 of a turn around the axis between two of 9 samples halved 32 times: \
                 faster than the pcurve sampling resolves"
                    .into(),
            ),
            (
                ExactFault::NotPositive {
                    what: "radius",
                    value: -1.0,
                }
                .into(),
                "radius -1 is not finite and positive".into(),
            ),
            (
                ExactFault::AngleRangeNotFinite {
                    lo: 0.0,
                    hi: f64::INFINITY,
                }
                .into(),
                "the angle range [0, inf] is not finite".into(),
            ),
            (
                ExactFault::AngleRangeEmpty { lo: 1.0, hi: 1.0 }.into(),
                "the angle range [1, 1] is empty".into(),
            ),
            (
                ExactFault::AngleRangeTooLong { lo: 0.0, hi: 7.0 }.into(),
                "the angle range [0, 7] is longer than a full turn".into(),
            ),
            (
                ExactFault::HyperbolaRangeTooLong { lo: 0.0, hi: 800.0 }.into(),
                "the range [0, 800] is too long for a hyperbola's rational form".into(),
            ),
            (
                ExactFault::RangeEmpty { lo: 1.0, hi: 1.0 }.into(),
                "the range [1, 1] is empty or not finite".into(),
            ),
            (
                ExactFault::ExtrusionDirection {
                    direction: Vec3::zeros(),
                }
                .into(),
                format!(
                    "the extrusion direction {:?} is zero or not finite",
                    Vec3::zeros()
                ),
            ),
            (
                ExactFault::ExtrusionRange { lo: 1.0, hi: 1.0 }.into(),
                "the extrusion range [1, 1] is empty or not finite".into(),
            ),
            (
                ExactFault::BoundsNotFinite.into(),
                "the parameter bounds are not finite".into(),
            ),
            (ExactFault::Frame(frame).into(), frame.to_string()),
            (
                ExactFault::CurveOnAxis.into(),
                "the curve lies on the axis: nothing to revolve".into(),
            ),
            (
                DegenerateReason::BranchNoParameters { surface: 1 },
                "a branch has no parameters on operand 1".into(),
            ),
            (
                DegenerateReason::PointNotFinite {
                    point: Point3::new(f64::NAN, 0.0, 0.0),
                },
                format!(
                    "the point {} to project is not finite",
                    Point3::new(f64::NAN, 0.0, 0.0)
                ),
            ),
            (
                DegenerateReason::NoProjectionCandidate,
                "the search found no candidate to project onto".into(),
            ),
            (
                DegenerateReason::ProfileLoopEmpty,
                "a profile loop with a segment".into(),
            ),
            (
                DegenerateReason::LoopPiecesDiffer,
                "a loop cut into as many pieces as the other sections".into(),
            ),
        ];
        for (reason, text) in cases {
            assert_eq!(reason.to_string(), text);
        }
        assert_eq!(said(FrameFault::NonFinite), "non-finite frame");
    }
}
