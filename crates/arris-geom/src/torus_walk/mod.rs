//! The section of a torus with an analytic surface, traced exactly in the
//! torus's own `(u, v)`: every branch, its ends and the singular points
//! where branches meet, before any fit (`docs/DATA-MODEL.md` §Curves).
//! The section of a NURBS surface with an analytic one is traced the same
//! way in the NURBS surface's own parameters (ADR-0061), mapped onto the
//! torus's `[0, 2π]` each way (`crate::trace_torus::Rescale`); what a
//! torus never has, the edges of a domain that does not close, is at the
//! end of this note.
//!
//! `crate::trace_torus` isolates the points that carry the section's
//! topology: the turning points, where the section is parallel to `v`,
//! and the critical points of the other surface's distance over the
//! torus, of which those within `tol.linear` of it are the section's
//! **singular points**. Between them the section is a graph `v(u)`, and a
//! branch is such graphs joined through their turning points by ADR-0018's
//! device, `u = u_T ± L(1 − cos θ)`, which takes the square root out of a
//! turn: one smooth callable, closed or open, as a ruled pair's is. Inside
//! a turning point's cell a graph is evaluated along `v` from that square
//! root, and not along `u`, where a rounding in `u` is its square root
//! along the curve (`Fold`).
//!
//! What a ruled pair has in closed form — the root a ruling's quadratic
//! gives — is here the root of the other surface's signed distance along
//! a tube circle, found by a bracketed Newton iteration. The bracket is
//! what makes it the *right* root, and every bracket is proven before it
//! is used. Each graph is covered by **cells**, boxes of the parameter
//! plane marched along it from a turning point or a singular one:
//!
//! - an ordinary cell has `∂f/∂t` of one sign over the box, so a line of
//!   constant `u` meets the section in it at most once, and `f` of one
//!   sign along its lower edge and of the other along its upper, so it
//!   does meet it, at every `u` of the cell;
//! - a turning point's cell has `∂f/∂s` of one sign and no other turning
//!   point in it: the section in it is one graph `u(v)` with its one
//!   extremum at the turning point, so `v_T` itself parts its two arms.
//!   It is as tall as the fold `u − u_T ≈ c·(v − v_T)²` is at its width;
//! - a singular point's cell has `∂²F/∂v²` of one sign and `f` of one
//!   sign along its lower and upper edges: two roots on every line of
//!   constant `u`, parted by the one zero of the distance's slope in `v`.
//!
//! All of it is decided on the Bernstein coefficients of the section's
//! polynomial over the box (`PatchedSection::sign_over`), against their
//! rounding floor. A march ends where it enters another turning point's
//! or singular point's cell, which holds nothing but that point's own
//! arms, so the branch structure is proven and not sampled. Components
//! that never turn wind round `u` and cross `u = 0`: the roots of
//! `f(0, ·)` no marched graph accounts for seed them.
//!
//! A singular point is ADR-0018's carried to two dimensions. The distance
//! `δ` over the torus has a critical point there with a value within the
//! tolerance; it is replaced by `δ − E`, `E` a bump of that value that
//! falls to zero at twice the offset at which `δ`'s quadratic part would
//! have made up for it — measured in the Hessian's own metric, its
//! eigenvalues made positive, so that a ring a hundred tubes across is
//! corrected as far as it has to be each way and no farther. The corrected
//! distance vanishes *at* the critical point and keeps its Hessian's
//! signature. A saddle is then a crossing, four arms ending at the point
//! exactly; an extremum is an isolated point. A gap, a near miss and a
//! loop smaller than the tolerance all become the one point they are at
//! that tolerance, and the turning points the near miss had, `√(tol·r)`
//! apart, lie inside the reach and are the singular point's.
//!
//! A **tube circle of the torus on the other surface** — the pipe elbow
//! against its pipe, a bead in the tube, a cone about the tangent to the
//! centre circle — is a whole line `u = u₀` of turning points, which
//! nothing isolates. It is found first, held to the tolerance all the way
//! round, and returned as the circle it is. Then it is divided out: the
//! other surface's `F` over the torus is `sinᵐ((u − u₀) / 2)` times a
//! quotient, `m` one where the surfaces cross along the circle and two
//! where they are tangent along it, and the quotient's zeros are the rest
//! of the section. On the patches the division is one by a linear factor
//! of a column's chart (`PatchedSection::deflate`); for the walk the
//! quotient is a closed form, `F` expanded along the chord from the
//! circle's point ([`Walker::quotient`]) — never a small number over a
//! small number, so the rest of the section is as exact where it crosses
//! the circle as anywhere. Everything above then runs on the quotient as
//! it does on the distance, and a branch that comes to `u = u₀` is cut
//! there: the crossing is a singular point of the section, the branch
//! ends at it exactly, and the circle runs through it. What is dropped
//! for the division to be exact — `F` along the circle, and along a
//! tangency its slope across it — is within what `tol.linear` makes of
//! `F` anywhere on the torus, or the circle is not accepted, so the rest
//! is within `tol.linear` of the other surface as the circle is.
//!
//! A NURBS surface's direction that does not close ends at the edges of
//! its domain, and every point, cell and march stays inside it. A branch
//! that reaches an edge ends there, at the root of the section along the
//! edge line: each root is found on the edge's own polynomial, proven
//! alone in a bracket and given an **edge cell** ([`EdgeCell`]), one arm
//! from the root into the domain, which a march starts from or ends in
//! as at a turning point's arm. A turning point near an edge of `v` has
//! its cell cut there, and an arm that reaches the edge inside the cell
//! ends on it; one on the edge is where the section crosses it square,
//! and has one arm. The patches' polynomials go on past the edges, and a
//! zero isolated beyond one is dropped. A singular point on an edge of
//! `u` has its cell cut there and only its arms into the domain; with no
//! isoline, one on an edge of `v` is traced with the parameters swapped.
//! The section touching an edge without entering the domain is an
//! [`EdgeTouch`]: a point, or the stretch of the edge rounding cannot
//! sign, within the tolerance. Refused by name: the section touching an
//! edge and entering the domain there, a crossing at a corner, and a turn
//! at a kink, a knot of `v` where the surface is only continuous and the
//! section comes back with no turning point to find.
//!
//! An **isoline of a NURBS surface on the other surface**, a whole line
//! `u = u₀`, is its tube circle (ADR-0061 §3): found the same way, held to
//! the tolerance from edge to edge, returned as the surface's own line
//! and divided out of the patches that hold it
//! (`PatchedSection::deflate`). The quotient the walk evaluates is the
//! patches' own polynomial with it divided out
//! (`crate::trace_torus::Quotient`), not a closed form. A line `v = v₀`
//! is one of `u` with the surface's parameters swapped, and the walk runs
//! on the swapped surface and swaps them back. A patch that lies on the
//! other surface all over is refused before any of it.

mod arcs;
mod assembly;
mod cells;
mod run;
mod singular;
mod walker;

use arris_math::real;
use core::cell::Cell as Latest;
use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};
use std::sync::Arc;

use arris_math::roots::{POLYNOMIAL_ROUNDING, newton_in_interval};
use arris_math::{
    Frame, Interval, Meter, Point2, Point3, Tolerance, is_negligible, wrap_angle, wrap_offset,
};

use crate::bernstein_n::Zero;
use crate::halt::Halt;
use crate::implicit::Implicit;
use crate::trace::{
    Arc1, BranchEnd, REACH, SectionBranch, SectionCircle, SectionFault, SectionPoint, SectionTrace,
    Stretch,
};
use crate::trace_torus::{PatchedSection, Probe, Quotient, Rescale};
use crate::{Curve, GeomError, GeomKind, NurbsSurface, Surface, SurfaceEval, SurfaceKind};

/// The half-width a turning point's or a singular point's cell is first
/// tried at, and halved from until its certificate holds. A size in the
/// parameter plane, not a tolerance: a thirty-second of a half turn is
/// small enough that most cells pass at once and large enough that the
/// march towards one takes a handful of cells to reach it.
const CELL_START: f64 = PI / 32.0;

/// How many times a cell is halved before its point is given up as one
/// `f64` cannot resolve: `2⁻¹⁶` of [`CELL_START`] is a few microradians,
/// below which the coefficients of `f` over the cell are its rounding.
const CELL_HALVINGS: usize = 16;

/// The least half-height of a march's cell, the smallest a point's own
/// cell gets: two arms' edges a rounding apart leave a march a step of
/// `1e-14` between them, and a cell as flat as that holds `f` below its
/// rounding floor all over. A cell taller than it need be is only harder
/// to certify, never wrong.
const HALF_MIN: f64 = CELL_START / (1u64 << CELL_HALVINGS) as f64;

/// The longest step of a march in `u`, a sixteenth of a turn: a cell
/// never lies over more than two patches either way.
const STEP_MAX: f64 = PI / 8.0;

/// How near an edge of a NURBS surface's domain that does not close a
/// turning point is taken to be on it, in the walk's parameter: the
/// least a turning point's cell may be tall ([`Tracer::turning_cell`]), so
/// that one nearer has no cell inside the domain. A statement about
/// `f64`, never a tolerance.
const EDGE_TOUCH: f64 = 64.0 * ANGLE_SLACK;

/// How many times a march halves its step before it gives up on a cell.
/// A structural bound, as `crate::bernstein::MAX_DEPTH` is.
const STEP_HALVINGS: usize = 40;

/// How many cells one graph may take. A march shrinks its cells
/// geometrically towards a turning point and takes a few dozen between
/// two of them; a count, there to end a march that is not getting
/// anywhere.
const MAX_CELLS: usize = 4096;

/// The step of the central differences the Hessian of the distance is
/// taken with, in radians: the gradient is analytic, so the differences
/// are good to the step's square, eight digits. It only sizes a singular
/// point's reach and cell, which no result depends on to more than two.
const HESSIAN_STEP: f64 = 1e-4;

/// Where a turning point's arm leaves its cell through the cell's upper
/// or lower edge, the share of the way there it is followed inside the
/// cell, so that a march handing over to it does so well inside the
/// arm's bracket. A ratio, not a tolerance.
const EXIT_SHARE: f64 = 0.75;

/// What two sums of the same angles may differ by: a march clipped at
/// another cell's edge has to recognise that it is there.
const ANGLE_SLACK: f64 = 16.0 * f64::EPSILON * TAU;

/// How far either side of a candidate a tube circle is looked for, in
/// radians: wide enough that the distance there is far above its
/// rounding, narrow enough that nothing else of the section is likely to
/// be between. A place to look, not a tolerance: the circle found is held
/// to `tol.linear` afterwards, all the way round.
const CIRCLE_BRACKET: f64 = 1e-4;

/// How many tube circles of `v` a candidate is looked at on, and how many
/// points of a tube circle are held to the tolerance. The other surface's
/// polynomial along a tube circle is a trigonometric polynomial of degree
/// four at the most, which thirty-two points bound to within a tenth.
const CIRCLE_SAMPLES: usize = 32;

/// Of the tube circles of `v` a candidate is looked at on, how many it is
/// refined on: those where the distance beside it is largest, away from
/// wherever else the section crosses the circle.
const CIRCLE_REFINED: usize = 5;

/// How many golden sections look for the tube circle nearest the other
/// surface inside [`CIRCLE_BRACKET`]: forty take the bracket to `1e-12`
/// radians, below anything a tolerance tells apart.
/// The points `Tracer::seed` samples along the seam between a
/// root it cannot prove alone and a singular point, to tell whether the
/// run between them is within the tolerance: a sampling count, not a
/// tolerance.
const SEAM_RUN_SAMPLES: usize = 16;

const GOLDEN_STEPS: usize = 40;

/// How far either side of a candidate along an edge line the least
/// distance is looked for as a touch of the edge: a candidate is within
/// rounding of a sign change of the line's polynomial, or of where it
/// comes nearest zero.
const TOUCH_BRACKET: f64 = 1e-4;

/// How many times a stretch of an edge line is halved to read the sign
/// of `f` along it beside a touch: from a cell's width down to a
/// tolerance's, with room to spare.
const TOUCH_HALVINGS: usize = 48;

/// How many steps each way a NURBS patch is looked at on to tell whether
/// it lies on the other surface ([`Tracer::flush`]): a patch is a
/// polynomial of a few degrees each way, and one within the tolerance at
/// nine points across in both directions and not between them is a
/// surface that would have been refused anyway. A sampling count, not a
/// tolerance.
const FLUSH_SAMPLES: usize = 8;

/// A tube circle this near a column's edge, in radians, is put on it: a
/// billionth of a radian is a thousandth of any tolerance a model carries
/// on a torus metres across, the circle is held to the tolerance where it
/// is put, and a branch seeded on `u = 0` then starts on the circle and
/// not a rounding away from it.
const EDGE_SNAP: f64 = 1e-9;

/// How far from a turning point in `v` its arms are walked in `v`
/// ([`Fold`]), in radians. The root along `u` is `√ε` off within about
/// `√ε` of the turn, and at a thousandth of a radian it is good to
/// `ε / 10⁻³` again; the cubic in `√off` meets the arm there in value and
/// slope and not in curvature, a break a fitted section spends control
/// points on, which the nearer the turn it is the less it weighs: over a
/// whole turning point's cell the torus fixtures' loops took up to four
/// times their control points, and at `10⁻²` to `10⁻⁴` the same as the
/// walk along `u`. A reach, not a tolerance.
const FOLD_REACH: f64 = 1e-3;

/// The step of the central differences that give the gradient of the
/// distance with a tube circle divided out of it
/// ([`Walker::quotient`]): the quotient is analytic and good to rounding,
/// so the gradient is to ten digits, and no root depends on it — it
/// steers Newton steps and sizes cells.
const QUOTIENT_STEP: f64 = 1e-5;

/// How many times a point's stretch is halved before it is taken to be
/// the point alone ([`Walker::stretch`]): from its first half-angle, at
/// most `π`, to `2⁻⁶⁰` of it, below any rounding of an angle. A count,
/// not a tolerance.
const STRETCH_HALVINGS: usize = 60;

/// A singular point's correction to the distance: `value` at `at`,
/// falling to zero by a smoothstep of `y = √(xᵀ·M·x) / reach`, `x` the
/// offset in the parameter plane. `M` is the Hessian of the distance
/// there with its eigenvalues made positive, so that the correction is
/// measured in the distance's own quadratic part: a torus a hundred tubes
/// across curves a hundred times faster one way than the other, and a
/// round bump wide enough for the one would swamp the other.
#[derive(Debug, Clone, Copy)]
struct Bump {
    at: [f64; 2],
    value: f64,
    /// `[M_uu, M_uv, M_vv]`.
    metric: [f64; 3],
    reach: f64,
    /// Whether each direction closes, so that an offset is taken the
    /// short way round.
    closed: [bool; 2],
}

/// The difference `x` of the walk's parameter in a direction, the short
/// way round where the direction closes and as it is where it does not.
fn offset(closed: bool, x: f64) -> f64 {
    if closed { wrap_offset(x, TAU) } else { x }
}

/// A singular point's cell: `at ± half`.
#[derive(Debug, Clone, Copy)]
struct SingularCell {
    at: [f64; 2],
    half: [f64; 2],
    /// The slope `dv/du` of the line between the two arms on either side,
    /// from the corrected Hessian: where the distance's slope in `v`
    /// vanishes, to first order.
    slope: f64,
    /// A saddle, which four arms end at; otherwise an extremum, an
    /// isolated point.
    crossing: bool,
    /// Whether the cell reaches below its point in `u` and above it. Both,
    /// but for a saddle on an edge of a NURBS surface's `u` that does not
    /// close, whose cell is cut there and whose arms beyond the edge are
    /// none.
    sides: [bool; 2],
    /// Whether the cell reaches below its point in `v` and above it: both,
    /// but for a saddle on an edge of `v` that does not close, which at a
    /// corner of the domain is cut in `u` as well and keeps the one arm
    /// inside the quadrant.
    rows: [bool; 2],
}

/// A tube circle of the walked torus that lies on the other surface, to
/// be divided out of the distance: what is left of the section is traced
/// on the quotient.
#[derive(Debug, Clone, Copy)]
struct TubeFactor {
    u0: f64,
    /// One where the surfaces cross along the circle, two where they are
    /// tangent along it.
    order: usize,
}

/// The walked surface — a torus, or a NURBS surface — and the other
/// surface with the singular points' corrections: what a branch is
/// evaluated on, in the walk's parameters.
#[derive(Debug)]
pub(crate) struct Walker {
    walked: Surface,
    other: Surface,
    /// A NURBS surface's parameters as the walk takes them; `None` on a
    /// torus, whose angles are walked as they are.
    rescale: Option<Rescale>,
    /// Whether each direction closes over `2π`; one that does not is
    /// bounded by `[0, 2π]`, the walked surface's domain.
    closed: [bool; 2],
    /// What a point of the walked surface is summed from: a torus's
    /// radii, a NURBS surface's farthest control point.
    size: f64,
    factor: Option<TubeFactor>,
    /// A NURBS surface's section with its isoline `factor` divided out,
    /// which the quotient is evaluated on; `None` on a torus, whose
    /// quotient is a closed form.
    divided: Option<Quotient>,
    /// The NURBS surface is walked with its parameters swapped, so that
    /// an isoline of its `v` is one of the walk's `u`: its own `(u, v)`
    /// are the walk's `(v, u)`.
    transposed: bool,
    bumps: Vec<Bump>,
    singular: Vec<SingularCell>,
}

/// Where the root of a cell is looked for.
#[derive(Debug, Clone, Copy)]
enum Bracket {
    /// Between two values of `v`.
    Fixed([f64; 2]),
    /// In a singular point's cell, above or below what parts its arms;
    /// `centre` is the point in the arc's unwrapped angles.
    Beside {
        singular: usize,
        centre: [f64; 2],
        upper: bool,
    },
}

/// A stretch of `u`, ascending, and the bracket that holds the graph's
/// one root at every `u` of it.
#[derive(Debug, Clone, Copy)]
struct Cell {
    u: [f64; 2],
    bracket: Bracket,
}

/// One graph `v(u)` of the section between two of its ends, in unwrapped
/// angles of its own.
#[derive(Debug, Clone)]
pub(crate) struct TorusArc {
    /// Ascending in `u`, end to end.
    cells: Vec<Cell>,
    /// `(u, v)` where the march started and where it ended.
    ends: [[f64; 2]; 2],
    /// How the graph is evaluated beside a turning point it ends at.
    folds: [Option<Fold>; 2],
}

/// An arm of a turning point, as the graph ending there is evaluated
/// inside the point's cell: by `v` and not by `u`.
///
/// At a turn the section is tangent to the line of constant `u`, so the
/// root along it moves by `√ε` for a rounding `ε` in `u`, and two arms
/// each finding the turn's `v` from its `u` end that far apart. Along the
/// line of constant `v` the root is well conditioned all over the cell
/// (its certificate is `∂f/∂u` of one sign), so the arm is walked in `v`:
/// `v − v_T = a₁r + a₂r² + a₃r³` in `r = √off`, `off` the offset from the
/// turn in `u` as the branch's parameter gives it — smooth through the
/// turn, as `Arc1`'s parameter makes `√off` — and `u` the root along that
/// `v`, as far as [`FOLD_REACH`] or the cell's edge. Both arms are `v_T`
/// at the turn itself, one point; `a₁` is the fold's own, `(u − u_T) ≈
/// c·(v − v_T)²`, the same for the two arms, so the branch is smooth
/// through the turn; `a₂` and `a₃` meet the walk along `u` and its slope
/// where it takes over, so the branch is smooth there too.
#[derive(Debug, Clone, Copy)]
struct Fold {
    /// The turning point, in the graph's angles.
    at: [f64; 2],
    /// How far from it in `u` the arm is walked in `v`: to where the
    /// walk along `u` takes over.
    reach: f64,
    /// `[a₁, a₂, a₃]`, signed towards the arm.
    coeffs: [f64; 3],
    /// The turning point's cell in `u`, where the root along `v` is.
    span: [f64; 2],
}

/// The arcs of one branch in the order it runs through them, each with
/// the whole turns that make `(u, v)` continuous from one to the next.
#[derive(Debug, Clone)]
pub(crate) struct TorusWalk {
    walker: Arc<Walker>,
    arcs: Vec<TorusArc>,
    shifts: Vec<[f64; 2]>,
    /// `(u, v)` of the singular points or the domain's edge points the
    /// branch's ends are.
    pins: [Option<[f64; 2]>; 2],
}

/// A turning point's cell, `at ± half`.
#[derive(Debug, Clone, Copy)]
struct TurningCell {
    at: [f64; 2],
    half: [f64; 2],
    /// `+1` where the section lies at larger `u` than the point, `−1` at
    /// smaller.
    side: f64,
    /// How far from the point in `u` the arm below it and the arm above
    /// it are followed inside the cell.
    extent: [f64; 2],
    /// `c` of the fold `|u − u_T| ≈ c·(v − v_T)²`.
    fold: f64,
    /// The arm — `0` below, `1` above — that lies outside a NURBS
    /// surface's domain, the point being on an edge of `v` that does not
    /// close: the section crosses the edge there square to it, and a
    /// branch ends at the point.
    open: Option<usize>,
    /// The arms that reach an edge of a NURBS surface's domain inside the
    /// cell, at their extent: a branch ends there, on the edge line.
    exits: [bool; 2],
}

/// Where the section meets an edge of a NURBS surface's domain that does
/// not close, and its cell: `at` is the root of the section along the
/// edge line, and in the cell, `at ± half` clipped to the domain, the
/// section is one arm from `at` to the cell's side `side` (ADR-0061).
///
/// On an edge of `u` the cell is a march's: `∂f/∂t` of one sign over it,
/// `f` of one sign along its lower edge and of the other along its upper,
/// so every line of constant `u` in it meets the section once. On an edge
/// of `v` the cell lies to the domain's side of it: `∂f/∂t` and `∂f/∂s` of
/// one sign over it, so that a line of constant `u` meets the section in
/// it at most once and the edge line exactly once, at `at`; and `f` of one
/// sign along its inner edge, so the arm leaves through a side, the one
/// where `f` on the edge line has the other sign.
#[derive(Debug, Clone, Copy)]
struct EdgeCell {
    at: [f64; 2],
    half: [f64; 2],
    /// `+1` where the arm lies at larger `u` than `at`, `−1` at smaller.
    side: f64,
    /// How far from `at` in `u` the arm is followed inside the cell.
    extent: f64,
    /// Where the arm's root is on every line of constant `u` in the cell.
    bracket: [f64; 2],
}

/// Where the section touches an edge of a NURBS surface's domain and
/// does not enter it: `f` moves away from zero into the domain all over
/// the cell `at ± half`, and is of the sign it moves to along the edge
/// line but for the stretch `run` of it, and along a line half the
/// tolerance in. Every zero in the cell is then within half the tolerance
/// of the stretch, which lies within the tolerance of the other surface.
/// A table stripe's cylinder against the walked blend it runs into,
/// beside the isoline they share, touches the blend's contact edge so at
/// the junction's vertex, to fourth order. The stretch is an isolated
/// point where it is shorter than the tolerance, and otherwise the edge's
/// own line over it, tangent ([`SectionCircle`]).
#[derive(Debug, Clone, Copy)]
struct EdgeTouch {
    at: [f64; 2],
    half: [f64; 2],
    /// The edge's direction, `0` for an edge of `u`.
    k: usize,
    /// The stretch of the edge line, in the other direction's parameter.
    run: [f64; 2],
}

/// What a candidate root on an edge line of a NURBS surface's domain is.
enum EdgeRoot {
    /// No root.
    None,
    /// A root proven alone on the line, where the section crosses it.
    Crossing(f64),
    /// A touch ([`EdgeTouch`]).
    Touch(EdgeTouch),
}

/// What a march ended at: an arm of a turning point — `0` below it, `1`
/// above — an arm of a singular point, `2·right + upper`, the seed it
/// started from, a turn or several later, the arm of an edge cell, where
/// an arm of a turning point leaves the domain inside its cell, or an
/// edge of `v` it ran aground on ([`Tracer::aground`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum ArcEnd {
    Turning(usize, usize),
    Singular(usize, usize),
    Home,
    Edge(usize),
    Exit(usize, usize),
    Aground,
}

/// A singular point as it is decided, before its cell.
struct Singular {
    at: [f64; 2],
    hessian: [f64; 3],
    /// Its correction; with no reach where the distance there is zero
    /// as it is.
    bump: Bump,
    crossing: bool,
}

struct Tracer<'a> {
    section: PatchedSection<'a>,
    walker: Walker,
    turning: Vec<TurningCell>,
    /// Where the section meets the edges of a NURBS surface's domain.
    edges: Vec<EdgeCell>,
    /// Where it touches them without entering the domain.
    touches: Vec<EdgeTouch>,
    /// The tube circles of the torus on the other surface, ascending.
    circles: Vec<TubeFactor>,
    tol: Tolerance,
}

/// The section of a torus with another analytic surface.
///
/// Operands: a torus against a plane, a cylinder, an elliptic cylinder, a
/// cone, a sphere or another torus, in any pose; every other pair is
/// [`GeomError::Unsupported`]. A torus is compact, so there is no region
/// to clip to and the whole section is returned. A tube circle of the
/// torus that lies on the other surface within `tol.linear` — a pipe
/// elbow against its pipe, a plane through the axis, a sphere or a cone
/// about the tangent to the centre circle — is one of
/// [`SectionTrace::circles`], exact, and what else the pair meets in is
/// traced beside it. The poses the tracer does not resolve are refused by
/// name as [`GeomError::DegenerateSection`]: two such circles with more
/// of the section besides ([`SectionFault::TubeCircle`]), surfaces
/// tangent along any other curve or the same, a singular point crowded by
/// another, by a turning point or by a tube circle, and turning points
/// `f64` does not tell apart — a loop within a ten-thousandth of a tube
/// circle all the way round among them.
///
/// Guarantees: every branch lies on both surfaces, exactly on the walked
/// torus — [`SectionBranch::uv`] is its parameters there — and to rounding
/// on the other, within `tol.linear` of it inside a singular point's
/// reach; two surfaces that come within `tol.linear` of tangency at a
/// point meet there in a [`SectionPoint`], and branches through it end at
/// it exactly; every part of the section is on a branch, because each
/// stretch of a branch is proven alone in its cell on the section's
/// polynomial, and the turning points and the roots along `u = 0` that
/// seed the branches are isolated, not sampled. A branch that reaches a
/// tube circle of [`SectionTrace::circles`] ends on it, at a
/// [`SectionPoint`] that is not isolated, and beside such a circle the
/// singular points are decided on the distance over the circle's factor,
/// which is never less than the distance. Which of two tori is
/// walked is a rule on the two surfaces — the smaller over all, then the
/// smaller tube — so swapping the arguments changes nothing, bit for bit.
///
/// ```
/// use arris_geom::{Surface, trace_torus};
/// use arris_math::{Frame, Point3, Precision, Vec3};
///
/// // A plane parallel to a ring's axis, through its hole: two ovals.
/// let ring = Surface::Torus { frame: Frame::world(), major_radius: 2.0, minor_radius: 0.5 };
/// let across = Frame::from_z(Point3::new(1.0, 0.0, 0.0), Vec3::x()).unwrap();
/// let plane = Surface::Plane { frame: across };
/// let trace = trace_torus(&ring, &plane, Precision::DEFAULT.tolerance(), &mut arris_math::Meter::default()).unwrap();
/// assert_eq!(trace.branches().len(), 2);
/// assert!(trace.branches().iter().all(|b| b.is_closed()));
/// let p = trace.branches()[0].point(0.3);
/// assert!((p.x - 1.0).abs() < 1e-12);
/// let uv = trace.branches()[0].uv(0.3).unwrap();
/// assert!((ring.point(uv.x, uv.y) - p).norm() < 1e-12);
/// ```
pub fn trace_torus(
    a: &Surface,
    b: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SectionTrace, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    let unsupported = || GeomError::Unsupported {
        a: GeomKind::Surface(a.kind()),
        b: GeomKind::Surface(b.kind()),
    };
    let (torus, other) = walked_torus(a, b).ok_or_else(unsupported)?;
    let section = PatchedSection::new(torus, other, tol).ok_or_else(unsupported)?;
    let tracer = Tracer {
        section,
        walker: Walker {
            walked: torus.clone(),
            other: other.clone(),
            rescale: None,
            closed: [true; 2],
            size: 0.0,
            factor: None,
            divided: None,
            transposed: false,
            bumps: Vec::new(),
            singular: Vec::new(),
        },
        turning: Vec::new(),
        edges: Vec::new(),
        touches: Vec::new(),
        circles: Vec::new(),
        tol,
    };
    tracer.run(meter).map_err(|halt| match halt {
        Halt::Fault(fault) => GeomError::DegenerateSection {
            a: GeomKind::Surface(torus.kind()),
            b: GeomKind::Surface(other.kind()),
            fault,
        },
        Halt::Stopped(stop) => GeomError::Interrupted(stop),
    })
}

/// The section of a NURBS surface with an analytic one, traced in the
/// NURBS surface's own `(u, v)` as a torus's is in its own (ADR-0061):
/// the other surface's polynomial over each of its Bézier patches, the
/// turning, critical and singular points certified over them, and the
/// branches marched between those points and the edges of the domain.
///
/// Operands: a `Surface::Nurbs` against a plane, a cylinder, an elliptic
/// cylinder, a cone, a sphere or a torus, in either order and any pose;
/// two NURBS surfaces are [`GeomError::Unsupported`]. The surface's domain
/// bounds the section: a direction that does not close ends at its
/// domain's edges, and a branch that reaches one ends there,
/// [`BranchEnd::Clipped`](crate::BranchEnd::Clipped), at the root of the
/// section along the edge line exactly, the point the edge's own curve
/// meets the other surface at. A direction that closes
/// ([`crate::NurbsSurface::closure`]) is walked across its seam.
///
/// An isoline of the surface — a whole line of constant `u` or of
/// constant `v`, its domain's edges among them — that lies on the other
/// surface within `tol.linear` is one of [`SectionTrace::circles`]: the
/// surface's own line, exact (`Curve::Nurbs`, never fitted), its
/// parameter the surface's other one. What else the pair meets in is
/// traced beside it, and a branch that crosses it ends on it at a
/// [`SectionPoint`] that is not isolated, as at a tube circle. A NURBS
/// surface on the other surface over a whole Bézier patch — flush with
/// it over an area — is [`GeomError::Unsupported`].
///
/// Guarantees: those of [`trace_torus`], with the NURBS surface for the
/// torus — every branch on the NURBS surface exactly, [`SectionBranch::uv`]
/// its own parameters there, and on the other to rounding; every part of
/// the section inside the domain on a branch or an isoline. Refused by
/// name as [`GeomError::DegenerateSection`]: the poses [`trace_torus`]
/// refuses, the section touching an edge of the domain and entering it
/// there or crossing it at a corner, isolines of both directions, and two
/// isolines or more with more of the section besides
/// ([`SectionFault::TubeCircle`]). A touch of an edge that does not enter
/// the domain is an isolated point, or the edge's own line over the
/// stretch rounding cannot sign, among the isolines.
pub(crate) fn trace_nurbs(
    a: &Surface,
    b: &Surface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SectionTrace, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    let unsupported = || GeomError::Unsupported {
        a: GeomKind::Surface(a.kind()),
        b: GeomKind::Surface(b.kind()),
    };
    let (nurbs, other) = match (a, b) {
        (Surface::Nurbs(nurbs), other) | (other, Surface::Nurbs(nurbs)) => (nurbs, other),
        _ => return Err(unsupported()),
    };
    let tracer = |nurbs: &NurbsSurface, transposed: bool| {
        let section = PatchedSection::on_nurbs(nurbs, other, tol)?;
        let size = (nurbs.control_points().iter())
            .map(|p| p.coords.norm())
            .fold(0.0, f64::max);
        Some(Tracer {
            section,
            walker: Walker {
                walked: Surface::Nurbs(nurbs.clone()),
                other: other.clone(),
                rescale: Some(Rescale::of(nurbs)),
                closed: nurbs.closure().map(|c| c.is_some()),
                size,
                factor: None,
                divided: None,
                transposed,
                bumps: Vec::new(),
                singular: Vec::new(),
            },
            turning: Vec::new(),
            edges: Vec::new(),
            touches: Vec::new(),
            circles: Vec::new(),
            tol,
        })
    };
    let along = tracer(nurbs, false).ok_or_else(unsupported)?;
    // Flush over an area: the boolean's coincident arm is the analytic
    // kinds' own.
    if along.flush() {
        return Err(unsupported());
    }
    // An isoline of `v` is one of `u` with the parameters swapped, and the
    // walk divides out isolines of its `u` alone.
    let flipped = nurbs.transposed();
    let across = tracer(&flipped, true).ok_or_else(unsupported)?;
    let (rows, columns) = (along.tube_circles(), across.tube_circles());
    let traced = match (rows.is_empty(), columns.is_empty()) {
        // A saddle whose cell does not fit on the one parameterisation
        // is tried on the other.
        (true, true) => match along.run_with(rows, meter) {
            Err(Halt::Fault(SectionFault::CrowdedSingularity)) => across.run_with(columns, meter),
            traced => traced,
        },
        (_, true) => along.run_with(rows, meter),
        (true, false) => across.run_with(columns, meter),
        // Isolines both ways, where one way's are all poles of the surface
        // — a collapsed row at an edge of the domain, one point of the
        // section, as where a walked blend narrows to its vertex — are
        // traced beside the poles: each is divided out of the other way's
        // section and comes back as its own point (ADR-0061 §3, amended).
        (false, false) => {
            if columns.iter().all(|c| across.is_pole(c.u0)) {
                along.run_beside_poles(rows, &columns, meter)
            } else if rows.iter().all(|c| along.is_pole(c.u0)) {
                across.run_beside_poles(columns, &rows, meter)
            } else {
                Err(SectionFault::TubeCircle.into())
            }
        }
    };
    traced.map_err(|halt| match halt {
        Halt::Fault(fault) => GeomError::DegenerateSection {
            a: GeomKind::Surface(SurfaceKind::Nurbs),
            b: GeomKind::Surface(other.kind()),
            fault,
        },
        Halt::Stopped(stop) => GeomError::Interrupted(stop),
    })
}

/// The torus to walk and the surface to put it into: of two tori the
/// smaller one over all, `R + r` — a torus's quartic at points a hundred
/// of its own sizes away cancels to `100⁴·ε`, so a small torus is never
/// the implicit one for a large one's points, however thin the large
/// one's tube — then the smaller tube, then the frames, coordinate by
/// coordinate. `None` without a torus.
fn walked_torus<'s>(a: &'s Surface, b: &'s Surface) -> Option<(&'s Surface, &'s Surface)> {
    let key = |s: &Surface| match s {
        Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => {
            let (o, z, x) = (frame.origin(), frame.z(), frame.x());
            Some([
                major_radius + minor_radius,
                *minor_radius,
                o.x,
                o.y,
                o.z,
                z.x,
                z.y,
                z.z,
                x.x,
                x.y,
                x.z,
            ])
        }
        _ => None,
    };
    match (key(a), key(b)) {
        (Some(ka), Some(kb)) => {
            let b_first = (kb.iter().zip(&ka))
                .map(|(p, q)| p.total_cmp(q))
                .find(|o| o.is_ne())
                .is_some_and(|o| o.is_lt());
            Some(if b_first { (b, a) } else { (a, b) })
        }
        (Some(_), None) => Some((a, b)),
        (None, Some(_)) => Some((b, a)),
        (None, None) => None,
    }
}

/// `p` inside `at ± half`, the short way round in a direction that
/// closes.
fn in_box(closed: [bool; 2], p: [f64; 2], at: [f64; 2], half: [f64; 2]) -> bool {
    (0..2).all(|k| offset(closed[k], p[k] - at[k]).abs() <= half[k])
}

/// `range` of a direction inside `[0, 2π]`, or the direction closes.
fn fits(closed: bool, [lo, hi]: [f64; 2]) -> bool {
    closed || (lo >= 0.0 && hi <= TAU)
}

/// One marched graph with what its two ends are.
struct Marched {
    arc: TorusArc,
    ends: [ArcEnd; 2],
}

/// Where a branch ends: at a point of the section, an index into the
/// points as they are found, or on an edge of a NURBS surface's domain.
#[derive(Debug, Clone, Copy)]
enum Stop {
    At(usize),
    Edge,
}

/// A stretch of one marched graph as a branch runs through it: all of it,
/// or what lies between two crossings of a tube circle.
struct Piece {
    graph: usize,
    /// `(u, v)` where the branch enters the stretch and where it leaves.
    from: [f64; 2],
    to: [f64; 2],
    /// The section turns back at that end of the stretch.
    turns: [bool; 2],
    /// Where a branch ends there.
    ends: [Option<Stop>; 2],
    /// How the stretch is evaluated beside a turning point at that end.
    fold: [Option<Fold>; 2],
}
