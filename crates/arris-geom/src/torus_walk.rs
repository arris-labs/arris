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

use core::cell::Cell as Latest;
use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};
use std::sync::Arc;

use arris_math::roots::{POLYNOMIAL_ROUNDING, newton_in_interval};
use arris_math::{
    Frame, Interval, Meter, Point2, Point3, Tolerance, is_negligible, wrap_angle, wrap_offset,
};

use crate::bernstein2::Zero2;
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

impl Bump {
    /// `y` at `(u, v)`, and `M·x`.
    fn measure(&self, u: f64, v: f64) -> (f64, [f64; 2]) {
        let x = [
            offset(self.closed[0], u - self.at[0]),
            offset(self.closed[1], v - self.at[1]),
        ];
        let [muu, muv, mvv] = self.metric;
        let mx = [muu * x[0] + muv * x[1], muv * x[0] + mvv * x[1]];
        let y = (x[0] * mx[0] + x[1] * mx[1]).max(0.0).sqrt() / self.reach;
        (y, mx)
    }

    /// How far the correction reaches along `u` and along `v`.
    fn extent(&self) -> [f64; 2] {
        let [muu, muv, mvv] = self.metric;
        let det = muu * mvv - muv * muv;
        [mvv, muu].map(|m| self.reach * (m / det).sqrt())
    }
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

impl Walker {
    /// The walked surface at the walk's `(u, v)`, its derivatives in the
    /// walk's parameters.
    fn eval(&self, u: f64, v: f64) -> SurfaceEval {
        let Some(map) = &self.rescale else {
            return self.walked.eval(u, v);
        };
        let e = self.walked.eval(map.to_surface(0, u), map.to_surface(1, v));
        let [a, b] = [map.factor(0), map.factor(1)];
        SurfaceEval {
            point: e.point,
            du: e.du * a,
            dv: e.dv * b,
            duu: e.duu * (a * a),
            duv: e.duv * (a * b),
            dvv: e.dvv * (b * b),
        }
    }

    /// The walked surface's point at the walk's `(u, v)`.
    fn point(&self, u: f64, v: f64) -> Point3 {
        match &self.rescale {
            None => self.walked.point(u, v),
            Some(map) => (self.walked).point(map.to_surface(0, u), map.to_surface(1, v)),
        }
    }

    /// The walked surface's own parameters at the walk's `at`: a NURBS
    /// surface walked transposed has them swapped back.
    fn surface_uv(&self, at: [f64; 2]) -> [f64; 2] {
        let uv = match &self.rescale {
            None => at,
            Some(map) => [0, 1].map(|k| map.to_surface(k, at[k])),
        };
        if self.transposed { [uv[1], uv[0]] } else { uv }
    }

    /// The difference `x` of direction `k`'s parameter, the short way
    /// round where it closes.
    fn off(&self, k: usize, x: f64) -> f64 {
        offset(self.closed[k], x)
    }

    /// The other surface's signed distance at the walk's `(u, v)` and
    /// its gradient in `(u, v)`, as they are.
    fn distance(&self, u: f64, v: f64) -> (f64, [f64; 2]) {
        let Some(implicit) = Implicit::of(&self.other) else {
            return (f64::NAN, [f64::NAN; 2]);
        };
        let e = self.eval(u, v);
        let (value, normal) = implicit.level(e.point);
        let slope = [e.du, e.dv].map(|d| normal.dot(&implicit.frame.vec_to_local(d)));
        (value, slope)
    }

    /// The distance with the tube circle `factor` divided out of it:
    /// `F̃(P(u, v)) / sinᵐ((u − u₀) / 2)` over `|∇F|`, a length, with `u`
    /// as it comes — unwrapped, so that the quotient is continuous, and
    /// for an odd order changes sign with a whole turn.
    ///
    /// No quotient is taken. The chord from the circle's point `A = P(u₀,
    /// v)` to `P(u, v)` is `λ·τ`, `λ = 2ρ sin((u − u₀) / 2)` along the
    /// unit tangent `τ` of the parallel circle half way, so `F(P)` is
    /// `F(A)` and `λ·(∇F(A)·τ + λ·c₂ + λ²·c₃ + λ³·c₄)` with the `c` of
    /// [`Implicit::bend`], and the bracket is `F̃ / λ` in closed form.
    /// `F(A)` is what is dropped: zero when the circle is on the other
    /// surface, within the tolerance when it is within the tolerance of
    /// it. Where the surfaces are tangent along the circle, `∇F(A)·τ`,
    /// which is `cos(·)·∇F(A)·τ₀ − sin(·)·∇F(A)·e₀`, loses its first term
    /// the same way — the gradient is across the circle's tangent `τ₀` —
    /// and the second carries the other `sin`.
    ///
    /// On a NURBS surface the isoline's quotient is the polynomial the
    /// isolation certified with the isoline divided out
    /// ([`Quotient::at`]), over `|∇F|`.
    fn quotient(&self, factor: &TubeFactor, u: f64, v: f64) -> f64 {
        if let Some(divided) = &self.divided {
            let Some(implicit) = Implicit::of(&self.other) else {
                return f64::NAN;
            };
            let quotient = divided.at(u, v);
            let steep = implicit
                .slope(implicit.frame.to_local(self.point(u, v)))
                .norm();
            return if steep > 0.0 {
                quotient / steep
            } else {
                quotient
            };
        }
        let (
            Some(implicit),
            Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            },
        ) = (Implicit::of(&self.other), &self.walked)
        else {
            return f64::NAN;
        };
        let half = 0.5 * (u - factor.u0);
        let rho = major_radius + minor_radius * v.cos();
        let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
        let (sin, cos) = factor.u0.sin_cos();
        let (sin_mid, cos_mid) = (factor.u0 + half).sin_cos();
        let outward = implicit.frame.vec_to_local(x * cos + y * sin);
        let tau = implicit.frame.vec_to_local(y * cos_mid - x * sin_mid);
        let a = implicit.frame.to_local(self.walked.point(factor.u0, v));
        let p = implicit.frame.to_local(self.walked.point(u, v));
        let gradient = implicit.slope(a);
        let [c2, c3, c4] = implicit.bend(a, tau);
        let lambda = 2.0 * rho * half.sin();
        let higher = c2 + lambda * (c3 + lambda * c4);
        let quotient = if factor.order == 1 {
            2.0 * rho * (gradient.dot(&tau) + lambda * higher)
        } else {
            2.0 * rho * (2.0 * rho * higher - gradient.dot(&outward))
        };
        let steep = implicit.slope(p).norm();
        if steep > 0.0 {
            quotient / steep
        } else {
            quotient
        }
    }

    /// What is decided in length at `(u, v)`: the exact distance to the
    /// other surface, or with a tube circle divided out the quotient,
    /// which is never less.
    fn apart(&self, u: f64, v: f64) -> f64 {
        match (&self.factor, Implicit::of(&self.other)) {
            (Some(factor), _) => self.quotient(factor, u, v),
            (None, Some(other)) => other.distance(self.point(u, v)),
            (None, None) => f64::NAN,
        }
    }

    /// The other surface's signed distance at the torus's `(u, v)` — with
    /// a tube circle divided out of it where one lies on the other
    /// surface — corrected, and its gradient in `(u, v)`.
    fn value(&self, u: f64, v: f64) -> (f64, [f64; 2]) {
        let (mut value, mut slope) = match &self.factor {
            None => self.distance(u, v),
            Some(factor) => {
                let h = QUOTIENT_STEP;
                let at = |u: f64, v: f64| self.quotient(factor, u, v);
                let slope = [
                    (at(u + h, v) - at(u - h, v)) / (2.0 * h),
                    (at(u, v + h) - at(u, v - h)) / (2.0 * h),
                ];
                (at(u, v), slope)
            }
        };
        for bump in &self.bumps {
            let (y, mx) = bump.measure(u, v);
            if y < 1.0 {
                value -= bump.value * (1.0 - y * y * (3.0 - 2.0 * y));
                for k in 0..2 {
                    slope[k] += 6.0 * bump.value * (1.0 - y) * mx[k] / (bump.reach * bump.reach);
                }
            }
        }
        (value, slope)
    }

    fn is_negative(&self, u: f64, v: f64) -> bool {
        self.value(u, v).0 < 0.0
    }

    /// The root of the distance along the tube circle at `u` between
    /// `lo` and `hi`, which a cell's certificate says is the only one.
    /// Where rounding leaves no sign change between the two — at a
    /// turning point itself, where the root is an end — it is the end
    /// nearer the surface.
    fn root_v(&self, u: f64, lo: f64, hi: f64) -> f64 {
        let Ok(bracket) = Interval::new(lo, hi) else {
            return lo;
        };
        // The iteration asks for the slope where it has just asked for
        // the value: one evaluation serves both.
        let last = Latest::new((f64::NAN, 0.0));
        let value = |v: f64| {
            let (value, slope) = self.value(u, v);
            last.set((v, slope[1]));
            value
        };
        let slope = |v: f64| match last.get() {
            (at, slope) if at == v => slope,
            _ => self.value(u, v).1[1],
        };
        let found = newton_in_interval(value, slope, bracket, 0.0);
        found.unwrap_or_else(|_| {
            if self.value(u, lo).0.abs() <= self.value(u, hi).0.abs() {
                lo
            } else {
                hi
            }
        })
    }

    /// The root along the line of constant `v` between `lo` and `hi`.
    fn root_u(&self, v: f64, lo: f64, hi: f64) -> Option<f64> {
        let bracket = Interval::new(lo.min(hi), lo.max(hi)).ok()?;
        newton_in_interval(
            |u| self.value(u, v).0,
            |u| self.value(u, v).1[0],
            bracket,
            0.0,
        )
        .ok()
    }

    /// What parts the two arms of a singular point's cell at `u`: the
    /// zero of the distance's slope in `v` across the cell, and where
    /// rounding hides it — next to the point itself — the line through
    /// the point that it is to first order. `centre` is the point in the
    /// caller's unwrapped angles.
    fn separator(&self, cell: &SingularCell, centre: [f64; 2], u: f64) -> f64 {
        let model = centre[1] + cell.slope * (u - centre[0]);
        let lo = if cell.rows[0] {
            centre[1] - cell.half[1]
        } else {
            centre[1]
        };
        let hi = if cell.rows[1] {
            centre[1] + cell.half[1]
        } else {
            centre[1]
        };
        let Ok(bracket) = Interval::new(lo, hi) else {
            return model;
        };
        let slope = |v: f64| self.value(u, v).1[1];
        let curvature =
            |v: f64| (slope(v + HESSIAN_STEP) - slope(v - HESSIAN_STEP)) / (2.0 * HESSIAN_STEP);
        newton_in_interval(slope, curvature, bracket, 0.0).unwrap_or(model.clamp(lo, hi))
    }

    /// `(u, v)` beside a turning point, `off` from it in `u` along the
    /// arm `fold` is: `v` from `√off`, and `u` the root along that line of
    /// constant `v`, which the turning point's cell holds alone and which
    /// is well conditioned there, where the root along `u` is not.
    fn beside(&self, fold: &Fold, off: f64) -> Option<[f64; 2]> {
        let r = off.sqrt();
        let [a1, a2, a3] = fold.coeffs;
        let v = fold.at[1] + r * (a1 + r * (a2 + r * a3));
        let u = self.root_u(v, fold.span[0], fold.span[1])?;
        Some([u, v])
    }

    /// The stretch through `P(u, v)` along the circle the root was found
    /// on — the tube circle, `along_v`, or the parallel beside a turning
    /// point: the arc, on the torus exactly, over which the other
    /// surface's distance stays within `δ` of its value at the point,
    /// `δ` being `POLYNOMIAL_ROUNDING` of the magnitude the distance is
    /// summed from there — the point's own and the torus's — and what it
    /// changes by over the last digit of the angle held fixed, which
    /// moves the root along the circle by its slope ratio: `2.4·10⁸` on a
    /// torus sliced a hair off a meridian plane, a digit of `u` a tenth
    /// of a tolerance along the tube circle. Its half-angle
    /// is `δ` over the distance's slope along the circle, halved until the
    /// distance at both its ends is within `δ` of the point's, which
    /// holds the curve of the distance along the circle and not only its
    /// slope. The torus walk's counterpart of a ruling's stretch
    /// ([`SectionBranch::distance`]).
    fn stretch(&self, u: f64, v: f64, along_v: bool) -> Stretch {
        let Some(implicit) = Implicit::of(&self.other) else {
            return Stretch::Point;
        };
        let e = self.eval(u, v);
        let summed = e.point.coords.norm() + implicit.frame.to_local(e.point).coords.norm();
        let magnitude = match self.walked {
            Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } => summed + major_radius + minor_radius,
            _ => summed + self.size,
        };
        let normal = implicit.frame.vec_to_world(implicit.gradient(e.point));
        let (tangent, bend, across, fixed) = if along_v {
            (e.dv, e.dvv, e.du, u)
        } else {
            (e.du, e.duu, e.dv, v)
        };
        // The parameter the walk's is mapped from, and its last digit in
        // the walk's.
        let fixed = match &self.rescale {
            None => fixed.abs(),
            Some(map) => {
                let k = usize::from(!along_v);
                map.to_surface(k, fixed).abs() / map.factor(k)
            }
        };
        let slope = normal.dot(&tangent).abs();
        // The angle held fixed is itself a float: the root moves along the
        // circle by as much as the distance changes over its last digit.
        let budget =
            POLYNOMIAL_ROUNDING * magnitude + normal.dot(&across).abs() * fixed * f64::EPSILON;
        // A tube circle's stretch is an arc of it about its centre; a NURBS
        // surface's is a segment of the tangent to its isoparametric line,
        // as short as a rounding is and so as straight.
        let axis = match self.walked {
            Surface::Torus { .. } => match tangent.cross(&bend).try_normalize(0.0) {
                Some(axis) => Some(axis),
                None => return Stretch::Point,
            },
            _ => None,
        };
        let at = |angle: f64| {
            let point = if along_v {
                self.point(u, v + angle)
            } else {
                self.point(u + angle, v)
            };
            implicit.distance(point)
        };
        let here = implicit.distance(e.point);
        let mut half = (budget / slope).min(PI);
        let mut halvings = 0;
        while !(half.is_finite()
            && (at(half) - here).abs() <= budget
            && (at(-half) - here).abs() <= budget)
        {
            half *= 0.5;
            halvings += 1;
            if halvings > STRETCH_HALVINGS {
                return Stretch::Point;
            }
        }
        match axis {
            Some(normal) => Stretch::Arc {
                centre: e.point + bend,
                normal,
                half,
            },
            None => Stretch::Segment(tangent * half),
        }
    }

    /// `v` on `arc` at `u`.
    fn v_on(&self, arc: &TorusArc, u: f64) -> f64 {
        let i = arc
            .cells
            .partition_point(|cell| cell.u[1] < u)
            .min(arc.cells.len().saturating_sub(1));
        let Some(cell) = arc.cells.get(i) else {
            return arc.ends[0][1];
        };
        match cell.bracket {
            Bracket::Fixed([lo, hi]) => self.root_v(u, lo, hi),
            Bracket::Beside {
                singular,
                centre,
                upper,
            } => {
                let Some(at) = self.singular.get(singular) else {
                    return centre[1];
                };
                if u == centre[0] {
                    return centre[1];
                }
                let between = self.separator(at, centre, u);
                if upper {
                    self.root_v(u, between, centre[1] + at.half[1])
                } else {
                    self.root_v(u, centre[1] - at.half[1], between)
                }
            }
        }
    }
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

impl TorusWalk {
    /// `(u, v)` of arc `i` at the walked angle `u`, in that arc's own
    /// angles; `anchor` is the turning point `u` is measured from and
    /// half the offset, as `Arc1::angle` gives them.
    fn local(&self, i: usize, u: f64, anchor: Option<(f64, f64)>) -> Option<[f64; 2]> {
        let arc = self.arcs.get(i)?;
        if let Some((turn, half)) = anchor {
            let off = 2.0 * half.abs();
            let fold = (arc.folds.iter().flatten()).find(|f| f.at[0] == turn && off <= f.reach);
            if let Some(uv) = fold.and_then(|fold| self.walker.beside(fold, off)) {
                return Some(uv);
            }
        }
        Some([u, self.walker.v_on(arc, u)])
    }

    /// The point of arc `i` at `u`, measured from `anchor`, with its
    /// stretch along the circle its root was found on
    /// ([`Walker::stretch`]).
    pub(crate) fn located(
        &self,
        i: usize,
        u: f64,
        anchor: Option<(f64, f64)>,
    ) -> (Point3, Stretch) {
        let (uv, along_v) = match self.arcs.get(i) {
            None => ([0.0; 2], true),
            Some(arc) => {
                let fold = anchor.and_then(|(turn, half)| {
                    let off = 2.0 * half.abs();
                    let fold =
                        (arc.folds.iter().flatten()).find(|f| f.at[0] == turn && off <= f.reach)?;
                    self.walker.beside(fold, off)
                });
                match fold {
                    Some(uv) => (uv, false),
                    None => ([u, self.walker.v_on(arc, u)], true),
                }
            }
        };
        let [u, v] = uv;
        (self.walker.point(u, v), self.walker.stretch(u, v, along_v))
    }

    /// The point of arc `i` at `u`, measured from `anchor`.
    pub(crate) fn point(&self, i: usize, u: f64, anchor: Option<(f64, f64)>) -> Point3 {
        let [u, v] = self.local(i, u, anchor).unwrap_or([0.0; 2]);
        self.walker.point(u, v)
    }

    /// `(u, v)` of arc `i` at `u`, measured from `anchor`, in the
    /// walked surface's own parameters, unwrapped along the branch;
    /// `at_end` says the parameter is the branch's start or its end.
    pub(crate) fn uv(
        &self,
        i: usize,
        u: f64,
        anchor: Option<(f64, f64)>,
        at_end: [bool; 2],
    ) -> Point2 {
        for (pin, at) in self.pins.iter().zip(at_end) {
            if let (Some(pin), true) = (pin, at) {
                let [u, v] = self.walker.surface_uv(*pin);
                return Point2::new(u, v);
            }
        }
        let (Some([u, v]), Some(shift)) = (self.local(i, u, anchor), self.shifts.get(i)) else {
            return Point2::origin();
        };
        let [u, v] = self.walker.surface_uv([u + shift[0], v + shift[1]]);
        Point2::new(u, v)
    }
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
/// started from, a turn or several later, the arm of an edge cell, or
/// where an arm of a turning point leaves the domain inside its cell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum ArcEnd {
    Turning(usize, usize),
    Singular(usize, usize),
    Home,
    Edge(usize),
    Exit(usize, usize),
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

impl Singular {
    /// How far the correction reaches along `u` and along `v`.
    fn extent(&self) -> [f64; 2] {
        if self.bump.reach > 0.0 {
            self.bump.extent()
        } else {
            [0.0; 2]
        }
    }

    /// `p` within `times` the correction's reach.
    fn holds(&self, p: [f64; 2], times: f64) -> bool {
        self.bump.reach > 0.0 && self.bump.measure(p[0], p[1]).0 <= times
    }
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

impl Tracer<'_> {
    fn run(self, meter: &mut Meter) -> Result<SectionTrace, Halt<SectionFault>> {
        let circles = self.tube_circles();
        self.run_with(circles, meter)
    }

    /// The trace with `circles` — the tube circles of a torus or the
    /// isolines of a NURBS surface on the other surface — divided out.
    fn run_with(
        mut self,
        circles: Vec<TubeFactor>,
        meter: &mut Meter,
    ) -> Result<SectionTrace, Halt<SectionFault>> {
        self.circles = circles;
        let torus = self.walker.rescale.is_none();
        // Two surfaces that are not the same share two tube circles at
        // the most: their centre lines touch the centre circle there.
        if torus && self.circles.len() > 2 {
            return Err(SectionFault::TangentAlongCurve.into());
        }
        for (i, circle) in self.circles.clone().iter().enumerate() {
            self.section.deflate(circle.u0, circle.order, i == 0);
        }
        match self.circles[..] {
            [] => {}
            [only] => {
                self.walker.factor = Some(only);
                self.walker.divided = self.section.quotient(only.u0, only.order);
            }
            // Two circles and nothing else is a plane through the axis,
            // or a sphere centred on the tangent to the centre circle.
            // With more of the section besides there is no quotient in
            // closed form to walk it on.
            _ if self.section.is_empty() => {
                let circles = self.section_circles()?;
                return Ok(SectionTrace::new(Vec::new(), Vec::new()).with_circles(circles));
            }
            _ => return Err(SectionFault::TubeCircle.into()),
        }
        // A turn at a kink of a NURBS surface has no turning point to
        // seed a march from.
        if self.section.turns_at_a_kink() {
            return Err(SectionFault::UnresolvedTurning.into());
        }
        let turning = (self.section.turning_points(meter))
            .map_err(|halt| halt.map_fault(|_| SectionFault::TangentAlongCurve))?
            .zeros;
        let critical = (self.section.critical_points(meter))
            .map_err(|halt| halt.map_fault(|_| SectionFault::TangentAlongCurve))?
            .zeros;
        let (turning, mut critical) = (self.on_domain(turning), self.on_domain(critical));
        if !torus {
            let past = self.critical_past_edges(&critical, meter)?;
            critical.extend(past);
        }
        // A box of turning points a whole turn long in `v` is a tube
        // circle the other surface runs along, and not one held to the
        // tolerance, which would have been divided out; a whole turn both
        // ways is the torus itself. On a NURBS surface a box the whole
        // domain both ways is the surface on the other; one the whole
        // domain one way is an isoline the other surface runs along
        // without holding it to the tolerance, which would have been
        // divided out (ADR-0061 §3), and is refused below with the
        // turning points it does not resolve.
        let turn = TAU - ANGLE_SLACK;
        for z in turning.iter().filter(|z| !z.certified) {
            let long = [0, 1].map(|k| z.hi[k] - z.lo[k] >= turn);
            match long {
                [true, true] => return Err(SectionFault::TangentAlongCurve.into()),
                [false, true] if torus => return Err(SectionFault::TubeCircle.into()),
                _ => {}
            }
        }

        let singular = self.singular_points(&critical)?;
        self.walker.bumps = singular
            .iter()
            .filter(|s| s.bump.reach > 0.0)
            .map(|s| s.bump)
            .collect();
        // The turning points of a near miss, of a loop smaller than the
        // tolerance and of the crossing itself are the singular point's.
        let absorbed = |z: &Zero2| {
            singular.iter().any(|s| {
                let held = !z.certified
                    && (0..2).all(|k| {
                        let half = 0.5 * (z.hi[k] - z.lo[k]);
                        let off = s.at[k] - 0.5 * (z.lo[k] + z.hi[k]);
                        // A singular point on an edge of the domain is
                        // on the box's own edge, when the box reaches it.
                        self.walker.off(k, off).abs() <= half + EDGE_TOUCH
                    });
                held || s.holds(z.at, 1.0)
            })
        };
        let turning: Vec<Zero2> = turning.into_iter().filter(|z| !absorbed(z)).collect();
        // A turning point on an edge of `u`, or an uncertified box of them
        // that reaches it, is the section touching the edge: where it does not enter
        // the domain there it is a touch, and otherwise it is refused.
        let mut inside = Vec::with_capacity(turning.len());
        for z in turning {
            let edge = (!self.walker.closed[0])
                .then(|| {
                    // A certified point is on the edge where its cell would
                    // be (`turning_cell`); a box of them, wherever it reaches.
                    [0.0, TAU].into_iter().find(|e| {
                        (z.at[0] - e).abs() <= EDGE_TOUCH
                            || (!z.certified
                                && (z.lo[0] - EDGE_TOUCH..=z.hi[0] + EDGE_TOUCH).contains(e))
                    })
                })
                .flatten();
            match edge {
                None => inside.push(z),
                Some(edge) => {
                    let touch = self.edge_touch(0, edge, z.at[1]);
                    (self.touches).push(touch.ok_or(SectionFault::UnresolvedTurning)?);
                }
            }
        }
        let turning = inside;
        if turning.iter().any(|z| !z.certified) {
            return Err(SectionFault::UnresolvedTurning.into());
        }
        let turning: Vec<[f64; 2]> = turning.iter().map(|z| z.at).collect();

        for s in &singular {
            let cell = self.singular_cell(s, &singular, &turning)?;
            self.walker.singular.push(cell);
        }
        for (i, &at) in turning.iter().enumerate() {
            let cell = self.turning_cell(i, at, &turning)?;
            self.turning.push(cell);
        }
        (self.edges, self.touches) = self.edge_cells()?;
        // What is left of the section crosses the tube circle wherever it
        // reaches it, and a branch is cut there. Where it turns or is
        // singular within the tolerance of the circle there is no telling
        // a crossing from a touch.
        if let Some(factor) = self.walker.factor {
            // How far `tol.linear` reaches across the circle or the
            // isoline in `u`.
            let speed = match &self.walker.walked {
                Surface::Torus {
                    major_radius,
                    minor_radius,
                    ..
                } => major_radius + minor_radius,
                _ => (0..CIRCLE_SAMPLES)
                    .map(|k| {
                        let v = TAU * k as f64 / (CIRCLE_SAMPLES - 1) as f64;
                        self.walker.eval(factor.u0, v).du.norm()
                    })
                    .fold(0.0, f64::max),
            };
            let near = self.tol.linear / speed;
            let off = |u: f64| self.walker.off(0, u - factor.u0).abs();
            let crowded = (self.turning.iter().any(|t| off(t.at[0]) <= near))
                || (self.walker.singular.iter()).any(|s| off(s.at[0]) <= s.half[0] + near);
            if crowded {
                return Err(SectionFault::CrowdedSingularity.into());
            }
        }
        self.assemble(meter)
    }

    /// Whether the walked NURBS surface's isoline at `u0` is a pole: one
    /// point to rounding all along it, a collapsed row.
    fn is_pole(&self, u0: f64) -> bool {
        let walker = &self.walker;
        let at = |k: usize| {
            let v = TAU * k as f64 / (CIRCLE_SAMPLES - 1) as f64;
            walker.eval(u0, v).point
        };
        let first = at(0);
        walker.rescale.is_some()
            && (1..CIRCLE_SAMPLES).all(|k| is_negligible((at(k) - first).norm(), walker.size))
    }

    /// [`Self::run_with`] with the isolines `circles` divided out, beside
    /// `poles`: each an isoline of the other direction that is a pole, an
    /// edge of the domain in this one's `v`, divided out of the section
    /// first at the order its patches show ([`PatchedSection::deflate_pole`])
    /// and returned as its own isoline, one point, tangent where that is
    /// two. A pole whose patches do not vanish there within the tolerance
    /// is refused, as the isolines of both directions were.
    fn run_beside_poles(
        mut self,
        circles: Vec<TubeFactor>,
        poles: &[TubeFactor],
        meter: &mut Meter,
    ) -> Result<SectionTrace, Halt<SectionFault>> {
        let (Surface::Nurbs(nurbs), Some(map)) = (&self.walker.walked, &self.walker.rescale) else {
            return Err(SectionFault::TubeCircle.into());
        };
        let rows = (poles.iter())
            .map(|p| {
                (nurbs.isocurve(1, map.to_surface(1, p.u0))).map_err(|_| SectionFault::TubeCircle)
            })
            .collect::<Result<Vec<_>, SectionFault>>()?;
        let mut points = Vec::with_capacity(poles.len());
        for (p, row) in poles.iter().zip(rows) {
            let order = self.section.deflate_pole(p.u0);
            if order == 0 {
                return Err(SectionFault::TubeCircle.into());
            }
            points.push(SectionCircle {
                circle: Curve::Nurbs(row),
                tangent: order == 2,
            });
        }
        Ok(self.run_with(circles, meter)?.with_more_circles(points))
    }

    /// The critical points of a NURBS surface's section past an open edge
    /// of its domain whose point is within the tolerance of the edge's
    /// beside it, each put on the edge, its box cut there
    /// ([`PatchedSection::critical_points_past`]): the edge's place is not
    /// resolved within the tolerance, and a saddle the fit puts a hair
    /// outside it is the section's singular point there, whose near miss
    /// inside the domain its turning points are. A box of them is put on
    /// the edge where its point nearest the edge is within the tolerance
    /// of it, for [`Self::singular_points`] to refuse where it is near the
    /// other surface. One the census inside the domain has found already,
    /// on the edge, is not found twice.
    fn critical_past_edges(
        &self,
        inside: &[Zero2],
        meter: &mut Meter,
    ) -> Result<Vec<Zero2>, Halt<SectionFault>> {
        let mut out = Vec::new();
        for k in (0..2).filter(|&k| !self.walker.closed[k]) {
            for edge in [0.0, TAU] {
                let found = (self.section)
                    .critical_points_past(k, edge, meter)
                    .map_err(|halt| halt.map_fault(|_| SectionFault::TangentAlongCurve))?;
                for mut z in found {
                    let mut nearest = z.at;
                    nearest[k] = edge.clamp(z.lo[k], z.hi[k]);
                    let mut on_edge = nearest;
                    on_edge[k] = edge;
                    let apart = (self.walker.point(nearest[0], nearest[1]))
                        - self.walker.point(on_edge[0], on_edge[1]);
                    if apart.norm() > self.tol.linear {
                        continue;
                    }
                    (z.at[k], z.lo[k], z.hi[k]) = (edge, edge, edge);
                    let known = inside.iter().any(|c| {
                        (0..2).all(|j| {
                            (c.lo[j] - EDGE_TOUCH..=c.hi[j] + EDGE_TOUCH).contains(&z.at[j])
                        })
                    });
                    if !known {
                        out.push(z);
                    }
                }
            }
        }
        Ok(out)
    }

    /// Whether the walked NURBS surface lies on the other surface over a
    /// whole patch: every point of a grid over it, corners and edges
    /// included, within `tol.linear` of it (ADR-0061 §3). The two
    /// surfaces are then flush over an area, which no isoline or branch
    /// stands for.
    fn flush(&self) -> bool {
        let Some(other) = Implicit::of(&self.walker.other) else {
            return false;
        };
        let at = |[lo, hi]: [f64; 2], i: usize| lo + (hi - lo) * i as f64 / FLUSH_SAMPLES as f64;
        (self.section.spans().into_iter()).any(|[u, v]| {
            (0..=FLUSH_SAMPLES).all(|i| {
                (0..=FLUSH_SAMPLES).all(|j| {
                    let p = self.walker.point(at(u, i), at(v, j));
                    other.distance(p).abs() <= self.tol.linear
                })
            })
        })
    }

    /// The zeros of a census that are points of the walked surface. A
    /// NURBS surface's patch polynomials go on past the edges of its
    /// domain, and a zero certified there by a box that reaches past an
    /// edge is no point of the surface: a certified zero counts where it
    /// is inside the domain, or within [`EDGE_TOUCH`] of it and is put on
    /// the edge; one that is not certified where its box reaches into the
    /// domain, its box cut at the edge. On a torus every zero counts.
    fn on_domain(&self, zeros: Vec<Zero2>) -> Vec<Zero2> {
        let closed = self.walker.closed;
        let range = -EDGE_TOUCH..=TAU + EDGE_TOUCH;
        (zeros.into_iter())
            .filter(|z| {
                (0..2).all(|k| {
                    closed[k]
                        || if z.certified {
                            range.contains(&z.at[k])
                        } else {
                            z.hi[k] >= 0.0 && z.lo[k] <= TAU
                        }
                })
            })
            .map(|mut z| {
                for k in (0..2).filter(|&k| !closed[k]) {
                    z.at[k] = z.at[k].clamp(0.0, TAU);
                    z.lo[k] = z.lo[k].max(0.0);
                    z.hi[k] = z.hi[k].min(TAU);
                }
                z
            })
            .collect()
    }

    /// The tube circles of the torus that lie on the other surface within
    /// the tolerance, ascending by `u`, each with whether the surfaces
    /// cross along it or are tangent along it.
    ///
    /// A candidate ([`PatchedSection::tube_circle_candidates`]) is looked
    /// at on tube circles of `v` all the way round: either side of it the
    /// distance has opposite signs where the surfaces cross along the
    /// circle, and then the circle is where it vanishes; the same sign
    /// where they are tangent, and then it is where its slope in `u`
    /// does. The circle is accepted in length: every point of it within
    /// `tol.linear` of the other surface, and the other surface's
    /// polynomial along it no more than a distance of `tol.linear` makes
    /// of it anywhere on the torus ([`Implicit::firmness`]) — that
    /// polynomial is what the rest of the section is traced without, so
    /// the rest stays within `tol.linear` of the other surface too. It is
    /// a tangency when the polynomial and its slope in `u` are within
    /// that bound together: there both are dropped.
    ///
    /// On a NURBS surface the same finds its isolines `u = u₀` on the other
    /// surface (ADR-0061 §3), held to the tolerance from one edge of `v`
    /// to the other and put on a knot line within [`EDGE_SNAP`] of one;
    /// where `u` does not close, one a rounding past an edge is put on
    /// it, and its sides are looked at on the surface's polynomial pieces
    /// carried past the edge.
    fn tube_circles(&self) -> Vec<TubeFactor> {
        let walker = &self.walker;
        let Some(other) = Implicit::of(&walker.other) else {
            return Vec::new();
        };
        let closed = walker.closed;
        // Round a tube circle; along a NURBS surface's `v` that does not
        // close, from one edge to the other.
        let round = |k: usize| {
            if closed[1] {
                TAU * (k as f64 + 0.5) / CIRCLE_SAMPLES as f64
            } else {
                TAU * k as f64 / (CIRCLE_SAMPLES - 1) as f64
            }
        };
        let tol = self.tol.linear;
        // Every point of the tube circle at `u` within `within` of the
        // other surface.
        let on = |u: f64, within: f64| {
            (0..CIRCLE_SAMPLES).all(|k| other.distance(walker.point(u, round(k))).abs() <= within)
        };
        // What the other surface's polynomial may be along a circle for
        // the section traced without it to stay within the tolerance of
        // the other surface: a distance of `tol.linear` wherever on the
        // torus it is traced, and the polynomial's own rounding.
        let dropped = {
            // Whatever is traced is on the torus, and as far from a point
            // as the torus is.
            let clear_of = |p: Point3| match &walker.walked {
                Surface::Nurbs(nurbs) => {
                    (nurbs.project(other.frame.to_world(p))).map_or(0.0, |on| on.distance)
                }
                walked => (Implicit::of(walked))
                    .map_or(0.0, |torus| torus.distance(other.frame.to_world(p)).abs()),
            };
            tol * other.firmness(clear_of) + self.section.floor()
        };
        // Candidates with the torus within the tolerance of the other
        // surface all the way from one to the next are one circle at that
        // tolerance — the two a sphere a little larger than the tube cuts
        // it in, either side of where it would touch — and are looked at
        // together.
        let near: Vec<f64> = (self.section.tube_circle_candidates().into_iter())
            .filter(|&u| on(u, 2.0 * tol))
            .collect();
        let joined = |a: f64, b: f64| on(a + 0.5 * walker.off(0, b - a), tol);
        // A group is its first candidate and how far the others reach
        // either side of it, the short way round.
        let mut groups: Vec<(f64, [f64; 2])> = Vec::new();
        for (i, &u) in near.iter().enumerate() {
            match groups.last_mut() {
                Some((anchor, reach)) if i > 0 && joined(near[i - 1], u) => {
                    let off = walker.off(0, u - *anchor);
                    *reach = [reach[0].min(off), reach[1].max(off)];
                }
                _ => groups.push((u, [0.0; 2])),
            }
        }
        // The group below `2π` and the group above zero may be one.
        if let (Some(&(anchor, reach)), Some(&last), true) =
            (groups.first(), near.last(), groups.len() > 1 && closed[0])
        {
            if joined(last, anchor) {
                groups.remove(0);
                if let Some((end, span)) = groups.last_mut() {
                    let off = wrap_offset(anchor - *end, TAU);
                    *span = [span[0].min(off + reach[0]), span[1].max(off + reach[1])];
                }
            }
        }

        let torus = walker.rescale.is_none();
        let knots = self.section.knot_lines();
        let mut found: Vec<TubeFactor> = Vec::new();
        for (anchor, reach) in groups {
            let lo = anchor + reach[0] - CIRCLE_BRACKET;
            let hi = anchor + reach[1] + CIRCLE_BRACKET;
            let mut beside: Vec<(f64, f64, f64)> = (0..CIRCLE_SAMPLES)
                .map(|k| {
                    let v = round(k);
                    (walker.distance(lo, v).0, walker.distance(hi, v).0, v)
                })
                .collect();
            let crosses = beside.iter().map(|(a, b, _)| a * b).sum::<f64>() < 0.0;
            beside.sort_by(|p, q| (q.0.abs().min(q.1.abs())).total_cmp(&p.0.abs().min(p.1.abs())));
            let Ok(bracket) = Interval::new(lo, hi) else {
                continue;
            };
            let mut roots: Vec<f64> = (beside.iter().take(CIRCLE_REFINED))
                .filter_map(|&(_, _, v)| {
                    let value = |u: f64| walker.distance(u, v).0;
                    let slope = |u: f64| walker.distance(u, v).1[0];
                    let bend = |u: f64| {
                        (slope(u + HESSIAN_STEP) - slope(u - HESSIAN_STEP)) / (2.0 * HESSIAN_STEP)
                    };
                    if crosses {
                        newton_in_interval(value, slope, bracket, 0.0).ok()
                    } else {
                        newton_in_interval(slope, bend, bracket, 0.0).ok()
                    }
                })
                .collect();
            roots.sort_by(f64::total_cmp);
            let refined = (roots.get(roots.len() / 2).copied()).unwrap_or(0.5 * (lo + hi));
            // The other surface's polynomial along the circle at `u` at
            // its largest, the same with its slope in `u` added, and how
            // far the circle is from the other surface.
            let measure = |u: f64| {
                let u0 = if torus {
                    let edge = (u / FRAC_PI_2).round() * FRAC_PI_2;
                    wrap_angle(if (u - edge).abs() <= EDGE_SNAP {
                        edge
                    } else {
                        u
                    })
                } else {
                    let knot = (knots.iter().copied()).find(|k| (u - k).abs() <= EDGE_SNAP);
                    let u = knot.unwrap_or(u);
                    if closed[0] {
                        wrap_angle(u)
                    } else {
                        u.clamp(0.0, TAU)
                    }
                };
                let mut worst = [0.0f64; 2];
                let mut away = 0.0f64;
                for k in 0..CIRCLE_SAMPLES {
                    let e = walker.eval(u0, round(k));
                    let a = other.frame.to_local(e.point);
                    let off = other.value(a).abs();
                    let slope = other.slope(a).dot(&other.frame.vec_to_local(e.du)).abs();
                    worst = [worst[0].max(off), worst[1].max(off + slope)];
                    away = away.max(other.distance(e.point).abs());
                }
                (u0, worst, away)
            };
            let accepted =
                |&(_, worst, away): &(f64, [f64; 2], f64)| away <= tol && worst[0] <= dropped;
            // Where the circle is on the other surface the roots are one
            // to rounding, and their median is the circle. Where it is
            // only within the tolerance of it they are as far apart as
            // that lets them be, and the circle held best is looked for:
            // the polynomial along the circle is, this near, linear in
            // `u` at every `v`, and the largest of its magnitudes convex.
            let mut best = measure(refined);
            if !accepted(&best) {
                let ratio = 0.5 * (5f64.sqrt() - 1.0);
                let (mut a, mut b) = (lo, hi);
                for _ in 0..GOLDEN_STEPS {
                    let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
                    if measure(c).1[0] < measure(d).1[0] {
                        b = d;
                    } else {
                        a = c;
                    }
                }
                best = measure(0.5 * (a + b));
            }
            if accepted(&best) {
                let (u0, worst, _) = best;
                let order = if worst[1] <= dropped { 2 } else { 1 };
                found.push(TubeFactor { u0, order });
            }
        }
        found.sort_by(|a, b| a.u0.total_cmp(&b.u0));
        if !torus {
            // Two groups that settled on one isoline.
            found.dedup_by(|a, b| a.u0 == b.u0);
        }
        found
    }

    /// The tube circles as curves: on the torus exactly, the parameter
    /// the torus's `v`.
    fn section_circles(&self) -> Result<Vec<SectionCircle>, SectionFault> {
        if self.circles.is_empty() {
            return Ok(Vec::new());
        }
        let Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        } = &self.walker.walked
        else {
            // A NURBS surface's isolines, exact: its own line at `u₀`, as
            // a curve in its other parameter.
            let (Surface::Nurbs(nurbs), Some(map)) = (&self.walker.walked, &self.walker.rescale)
            else {
                return Err(SectionFault::TubeCircle);
            };
            return (self.circles.iter())
                .map(|c| {
                    let line = (nurbs.isocurve(0, map.to_surface(0, c.u0)))
                        .map_err(|_| SectionFault::TubeCircle)?;
                    Ok(SectionCircle {
                        circle: Curve::Nurbs(line),
                        tangent: c.order == 2,
                    })
                })
                .collect();
        };
        (self.circles.iter())
            .map(|c| {
                let (sin, cos) = c.u0.sin_cos();
                let outward = frame.x().into_inner() * cos + frame.y().into_inner() * sin;
                let up = frame.z().into_inner();
                let origin = frame.origin() + outward * *major_radius;
                let around = Frame::from_orthonormal(origin, outward, up, outward.cross(&up))
                    .map_err(|_| SectionFault::TubeCircle)?;
                Ok(SectionCircle {
                    circle: Curve::Circle {
                        frame: around,
                        radius: *minor_radius,
                    },
                    tangent: c.order == 2,
                })
            })
            .collect()
    }

    /// The touches of the domain's edges ([`EdgeTouch`]): those whose
    /// stretch is shorter than the tolerance as points, in the walk's
    /// `(u, v)`, and the others as the edge's own line over their
    /// stretch, tangent.
    fn touch_runs(&self) -> Result<(Vec<[f64; 2]>, Vec<SectionCircle>), SectionFault> {
        let (Surface::Nurbs(nurbs), Some(map)) = (&self.walker.walked, &self.walker.rescale) else {
            return Ok((Vec::new(), Vec::new()));
        };
        let mut points = Vec::new();
        let mut runs = Vec::new();
        for t in &self.touches {
            let (k, j) = (t.k, 1 - t.k);
            let ends = t.run.map(|x| {
                let at = if k == 0 { [t.at[0], x] } else { [x, t.at[1]] };
                self.walker.point(at[0], at[1])
            });
            if (ends[1] - ends[0]).norm() <= self.tol.linear {
                points.push(t.at);
                continue;
            }
            let line = (nurbs.isocurve(k, map.to_surface(k, t.at[k])))
                .map_err(|_| SectionFault::TubeCircle)?;
            let range = Interval::new(map.to_surface(j, t.run[0]), map.to_surface(j, t.run[1]))
                .map_err(|_| SectionFault::TubeCircle)?;
            let run = line.segment(range).map_err(|_| SectionFault::TubeCircle)?;
            runs.push(SectionCircle {
                circle: Curve::Nurbs(run),
                tangent: true,
            });
        }
        Ok((points, runs))
    }

    /// The critical points within the tolerance of the other surface,
    /// each with its reach.
    fn singular_points(&self, critical: &[Zero2]) -> Result<Vec<Singular>, SectionFault> {
        let raw = |u: f64, v: f64| self.walker.value(u, v);
        let mut out: Vec<Singular> = Vec::new();
        // Whether a critical point is singular is decided in length, on
        // the exact distance — over a tube circle's factor where one has
        // been divided out, which is never less.
        let apart = |u: f64, v: f64| self.walker.apart(u, v);
        for z in critical {
            let value = raw(z.at[0], z.at[1]).0;
            if !z.certified {
                // A critical point that is not simple, within the
                // tolerance of the other surface somewhere in its box: a
                // box that is long either way is a curve of them, the
                // surfaces tangent along it; a small one is a point too
                // flat to decide.
                let corners = [[0, 0], [0, 1], [1, 0], [1, 1]]
                    .map(|[i, j]| apart([z.lo[0], z.hi[0]][i], [z.lo[1], z.hi[1]][j]));
                let near = |d: &f64| d.abs() <= self.tol.linear;
                if corners.iter().any(near) || near(&apart(z.at[0], z.at[1])) {
                    let long = (0..2).any(|k| z.hi[k] - z.lo[k] > CELL_START);
                    return Err(if long {
                        SectionFault::TangentAlongCurve
                    } else {
                        SectionFault::CrowdedSingularity
                    });
                }
                continue;
            }
            let off = apart(z.at[0], z.at[1]).abs();
            if off.is_nan() || off > self.tol.linear {
                continue;
            }
            let [u, v] = z.at;
            let h = HESSIAN_STEP;
            let (right, left) = (raw(u + h, v).1, raw(u - h, v).1);
            let (above, below) = (raw(u, v + h).1, raw(u, v - h).1);
            let huu = (right[0] - left[0]) / (2.0 * h);
            let hvv = (above[1] - below[1]) / (2.0 * h);
            let huv = 0.25 * ((right[1] - left[1]) + (above[0] - below[0])) / h;
            let det = huu * hvv - huv * huv;
            let mean = 0.5 * (huu + hvv);
            let spread = (mean * mean - det).max(0.0).sqrt();
            // The Hessian with its eigenvalues `mean ± spread` made
            // positive: `|λ₁|·P₁ + |λ₂|·P₂`, `P` the projectors onto its
            // eigenvectors.
            let (first, second) = ((mean + spread).abs(), (mean - spread).abs());
            let metric = if spread > 0.0 {
                let (sum, difference) = (0.5 * (first + second), 0.5 * (first - second));
                let along = [huu - mean, huv, hvv - mean].map(|h| h / spread);
                [
                    sum + difference * along[0],
                    difference * along[1],
                    sum + difference * along[2],
                ]
            } else {
                [first, 0.0, first]
            };
            // In that metric the distance's quadratic part is `±y²/2`
            // along each eigenvector and the correction adds at most
            // three quarters of it, so the corrected distance keeps the
            // signature: ADR-0018's bound, with one half for its `κ`.
            let reach = if value == 0.0 {
                0.0
            } else {
                REACH * (2.0 * value.abs()).sqrt()
            };
            let found = Singular {
                at: z.at,
                hessian: [huu, huv, hvv],
                bump: Bump {
                    at: z.at,
                    value,
                    metric,
                    reach,
                    closed: self.walker.closed,
                },
                crossing: det < 0.0,
            };
            let [wide, tall] = found.extent();
            if !(2.0 * wide <= CELL_START && 2.0 * tall <= FRAC_PI_4) {
                return Err(SectionFault::CrowdedSingularity);
            }
            out.push(found);
        }
        for (i, s) in out.iter().enumerate() {
            for o in &out[i + 1..] {
                if s.holds(o.at, 2.0) || o.holds(s.at, 2.0) {
                    return Err(SectionFault::CrowdedSingularity);
                }
            }
        }
        Ok(out)
    }

    /// The cell of a singular point: the largest tried whose certificate
    /// holds, never smaller than twice what its correction reaches.
    fn singular_cell(
        &self,
        s: &Singular,
        all: &[Singular],
        turning: &[[f64; 2]],
    ) -> Result<SingularCell, SectionFault> {
        if !s.crossing {
            return Ok(SingularCell {
                at: s.at,
                half: s.extent(),
                slope: 0.0,
                crossing: false,
                sides: [true; 2],
                rows: [true; 2],
            });
        }
        let [huu, huv, hvv] = s.hessian;
        // The arms' slopes are `mid ± spread`, of the distance as it is
        // and as it is corrected; the cell is tall enough for the steeper.
        let lift = if s.bump.reach > 0.0 {
            s.bump.metric.map(|m| 0.75 * s.bump.value.signum() * m)
        } else {
            [0.0; 3]
        };
        let steepness = |huu: f64, huv: f64, hvv: f64| {
            ((huv * huv - huu * hvv).max(0.0).sqrt() + huv.abs()) / hvv.abs()
        };
        let lifted = [huu + lift[0], huv + lift[1], hvv + lift[2]];
        let steepest = steepness(huu, huv, hvv).max(steepness(lifted[0], lifted[1], lifted[2]));
        let slope = -lifted[1] / lifted[2];
        let [wide, tall] = s.extent();
        let walker = &self.walker;
        let closed = walker.closed;
        // A saddle on an edge of `u` that does not close — a plane tangent
        // to a walked blend at the end of its contact, on the extended
        // surface's edge — has its cell cut there: below the edge the
        // certificate holds as in a whole cell, line by line of constant
        // `u`, and beyond it there is no surface and no arm.
        let edge = (!closed[0])
            .then(|| {
                [0.0, TAU]
                    .into_iter()
                    .find(|e| (s.at[0] - e).abs() <= EDGE_TOUCH)
            })
            .flatten();
        // The same on an edge of `v`: at a corner of the domain the cell
        // is cut both ways, and of the four arms the one inside the
        // quadrant is left.
        let ridge = (!closed[1])
            .then(|| {
                [0.0, TAU]
                    .into_iter()
                    .find(|e| (s.at[1] - e).abs() <= EDGE_TOUCH)
            })
            .flatten();
        let at = [edge.unwrap_or(s.at[0]), ridge.unwrap_or(s.at[1])];
        let sides = match edge {
            Some(0.0) => [false, true],
            Some(_) => [true, false],
            None => [true; 2],
        };
        let rows = match ridge {
            Some(0.0) => [false, true],
            Some(_) => [true, false],
            None => [true; 2],
        };
        for k in 0..=CELL_HALVINGS {
            let a = CELL_START / (1u64 << k) as f64;
            if a < 2.0 * wide {
                break;
            }
            let b = (2.0 * a * steepest).max(a).max(2.0 * tall);
            // A cell taller than an eighth of a turn is an arm too steep
            // to follow as a graph of `u`; no number at all is a Hessian
            // with no curvature in `v`.
            if b.is_nan() || b > FRAC_PI_4 {
                continue;
            }
            let cell = SingularCell {
                at,
                half: [a, b],
                slope,
                crossing: true,
                sides,
                rows,
            };
            let crowded = (turning.iter().any(|&t| in_box(closed, t, at, cell.half)))
                || all.iter().any(|o| {
                    let apart = (0..2).any(|k| o.at[k] != s.at[k]);
                    let [wide, tall] = o.extent();
                    apart && in_box(closed, o.at, at, [a + wide, b + tall])
                });
            let u = [
                if sides[0] { at[0] - a } else { at[0] },
                if sides[1] { at[0] + a } else { at[0] },
            ];
            let v = [
                if rows[0] { at[1] - b } else { at[1] },
                if rows[1] { at[1] + b } else { at[1] },
            ];
            // On a NURBS surface the cell lies inside the domain.
            if crowded || !(fits(closed[0], u) && fits(closed[1], v)) {
                continue;
            }
            if self.section.sign_over(Probe::Fvv, u, v).is_none() {
                continue;
            }
            // Outside the arms, on every row the cell reaches; a corner's
            // one row is the side of the arm that is left.
            let reached: Vec<Option<bool>> = [(0, rows[0]), (1, rows[1])]
                .iter()
                .filter(|(_, reached)| *reached)
                .map(|&(k, _)| self.section.sign_over(Probe::F, u, [v[k], v[k]]))
                .collect();
            let Some(&Some(outside)) = reached.first() else {
                continue;
            };
            if reached.iter().any(|r| *r != Some(outside)) {
                continue;
            }
            // Between the arms the distance has the other sign — negative
            // where `f` is positive outside them — all the way out to the
            // cell's sides.
            let parted = [0.25, 0.5, 1.0].iter().all(|share| {
                [(-1.0, sides[0]), (1.0, sides[1])]
                    .iter()
                    .filter(|(_, reached)| *reached)
                    .all(|(side, _)| {
                        let u = at[0] + side * share * a;
                        let between = walker.separator(&cell, at, u);
                        walker.is_negative(u, between) == outside
                    })
            });
            if parted {
                return Ok(cell);
            }
        }
        Err(SectionFault::CrowdedSingularity)
    }

    /// The cell of turning point `index`.
    ///
    /// On a NURBS surface the cell is cut at the edges of the domain that
    /// do not close. An arm that reaches an edge inside the cell — through
    /// the cut side in `u`, or the cut lower or upper edge in `v` — leaves
    /// the surface there, and a branch ends on it. A turning point on an
    /// edge of `v` is where the section crosses the edge square to it:
    /// its arm beyond the edge is none, and a branch ends at the point.
    /// One on an edge of `u` is where the section touches the edge, which
    /// is not followed.
    fn turning_cell(
        &self,
        index: usize,
        at: [f64; 2],
        turning: &[[f64; 2]],
    ) -> Result<TurningCell, SectionFault> {
        let walker = &self.walker;
        let closed = walker.closed;
        let on_edge = |k: usize| {
            (!closed[k])
                .then(|| {
                    [0.0, TAU]
                        .into_iter()
                        .find(|e| (at[k] - e).abs() <= EDGE_TOUCH)
                })
                .flatten()
        };
        if on_edge(0).is_some() {
            return Err(SectionFault::UnresolvedTurning);
        }
        // Below the point on `v = 0`, above it on `v = 2π`.
        let open = on_edge(1).map(|edge| usize::from(edge > 0.0));
        let at = match on_edge(1) {
            Some(edge) => [at[0], edge],
            None => at,
        };
        let clip = |k: usize, [lo, hi]: [f64; 2]| {
            if closed[k] {
                [lo, hi]
            } else {
                [lo.max(0.0), hi.min(TAU)]
            }
        };
        // The fold is `u − u_T = c·(v − v_T)²` to second order, and the
        // cell is as tall as the fold is at the cell's own width, so that
        // its arms are followed about as far in `u` as the cell reaches:
        // on a ring a hundred tubes across a square cell's arms would
        // leave through its upper and lower edges a few nanoradians of
        // `u` from the point, where `f` over a march's box is rounding.
        let h = HESSIAN_STEP;
        let bend =
            (walker.value(at[0], at[1] + h).1[1] - walker.value(at[0], at[1] - h).1[1]) / (2.0 * h);
        let fold = (0.5 * bend / walker.value(at[0], at[1]).1[0]).abs();
        for k in 0..=CELL_HALVINGS {
            let a = CELL_START / (1u64 << k) as f64;
            let tall = (a / fold).sqrt();
            let mut b = if tall.is_finite() {
                tall.clamp(a, STEP_MAX)
            } else {
                a
            };
            // No other turning point in the cell: one beside this one in
            // `u` — the other end of an S-bend, a few nanoradians of `u`
            // away and well apart in `v` — leaves the cell half the way
            // to it.
            for (j, t) in turning.iter().enumerate() {
                let d = [0, 1].map(|c| walker.off(c, t[c] - at[c]).abs());
                if j != index && d[0] <= a {
                    b = b.min(0.5 * d[1]);
                }
            }
            let crowded = b <= 64.0 * ANGLE_SLACK
                || (walker.singular.iter())
                    .any(|s| in_box(closed, s.at, at, [a + s.half[0], b + s.half[1]]));
            if crowded {
                continue;
            }
            let (u, v) = (
                clip(0, [at[0] - a, at[0] + a]),
                clip(1, [at[1] - b, at[1] + b]),
            );
            if self.section.sign_over(Probe::Fs, u, v).is_none() {
                continue;
            }
            // The tube circle through the point touches the section there
            // and nowhere else in the cell: the sign along it is the sign
            // outside the fold — along either edge of the cell but one the
            // point is on.
            let rows: Vec<bool> = (v.iter())
                .filter(|&&row| row != at[1])
                .map(|&row| walker.is_negative(at[0], row))
                .collect();
            let Some(&outside) = rows.first() else {
                continue;
            };
            if rows.iter().any(|&r| r != outside) {
                continue;
            }
            let inside = [u[0], u[1]].map(|u| walker.is_negative(u, at[1]) != outside);
            let side = match inside {
                [false, true] => 1.0,
                [true, false] => -1.0,
                _ => continue,
            };
            // Where the arms are followed to on the side they lie: the
            // cell's side, or the edge of the domain that cuts it.
            let reach = at[0] + side * a;
            let far = if closed[0] {
                reach
            } else {
                reach.clamp(0.0, TAU)
            };
            let cut = [far != reach, v[0] != at[1] - b, v[1] != at[1] + b];
            let extent = [0, 1].map(|arm| {
                let edge = v[arm];
                if open == Some(arm) {
                    // Outside the domain: no arm.
                    Some((0.0, true))
                } else if walker.is_negative(far, edge) == outside {
                    // The arm leaves through the cell's side.
                    let to = if cut[0] { (far - at[0]).abs() } else { a };
                    Some((to, cut[0]))
                } else if cut[1 + arm] {
                    // Through the edge of the domain in `v`.
                    (walker.root_u(edge, at[0], far)).map(|u| ((u - at[0]).abs(), true))
                } else {
                    (walker.root_u(edge, at[0], far))
                        .map(|u| (EXIT_SHARE * (u - at[0]).abs(), false))
                }
            });
            if let [Some(below), Some(above)] = extent {
                let held = [below, above]
                    .iter()
                    .enumerate()
                    .all(|(arm, (e, _))| open == Some(arm) || *e > ANGLE_SLACK);
                if held {
                    return Ok(TurningCell {
                        at,
                        half: [a, b],
                        side,
                        extent: [below.0, above.0],
                        fold,
                        open,
                        exits: [below.1 && open != Some(0), above.1 && open != Some(1)],
                    });
                }
            }
        }
        Err(SectionFault::UnresolvedTurning)
    }

    /// Where the section meets the edges of a NURBS surface's domain that
    /// do not close, each with its cell, the edges of `u` first, each
    /// from its start: every root of `f` along an edge line that no
    /// turning point's or singular point's cell holds already, and after
    /// them the touches ([`EdgeTouch`]). A candidate that is no root is
    /// dropped; one that cannot be proven a root alone or a touch, and is
    /// within the tolerance of the other surface, is refused.
    fn edge_cells(&self) -> Result<(Vec<EdgeCell>, Vec<EdgeTouch>), SectionFault> {
        let closed = self.walker.closed;
        let mut cells: Vec<EdgeCell> = Vec::new();
        let mut touches: Vec<EdgeTouch> = self.touches.clone();
        for k in 0..2 {
            if closed[k] {
                continue;
            }
            for high in [false, true] {
                let edge = if high { TAU } else { 0.0 };
                let on_line = |x: f64| if k == 0 { [edge, x] } else { [x, edge] };
                for candidate in self.section.edge_candidates(k, high) {
                    let held = |at: [f64; 2], cells: &[EdgeCell], touches: &[EdgeTouch]| {
                        (self.turning.iter()).any(|t| in_box(closed, at, t.at, t.half))
                            || (self.walker.singular.iter())
                                .any(|s| in_box(closed, at, s.at, s.half))
                            || cells.iter().any(|c| in_box(closed, at, c.at, c.half))
                            || touches.iter().any(|t| in_box(closed, at, t.at, t.half))
                    };
                    if held(on_line(candidate), &cells, &touches) {
                        continue;
                    }
                    match self.edge_root(k, edge, candidate)? {
                        EdgeRoot::None => {}
                        EdgeRoot::Crossing(root) => {
                            let at = on_line(root);
                            if !held(at, &cells, &touches) {
                                cells.push(self.edge_cell(k, high, at)?);
                            }
                        }
                        EdgeRoot::Touch(touch) => {
                            if !held(touch.at, &cells, &touches) {
                                touches.push(touch);
                            }
                        }
                    }
                }
            }
        }
        Ok((cells, touches))
    }

    /// The root of `f` along the edge line where direction `k`'s parameter
    /// is `edge` that `candidate` stands for, proven the only one in a
    /// bracket about it: the slope along the line of one sign over the
    /// bracket and `f` of opposite signs at its ends. A double root the
    /// section only touches the edge at is [`EdgeRoot::Touch`]
    /// ([`Tracer::edge_touch`]).
    fn edge_root(&self, k: usize, edge: f64, candidate: f64) -> Result<EdgeRoot, SectionFault> {
        let walker = &self.walker;
        let on_line = |x: f64| if k == 0 { [edge, x] } else { [x, edge] };
        let negative = |x: f64| {
            let [u, v] = on_line(x);
            walker.is_negative(u, v)
        };
        let mut half = CELL_START;
        for _ in 0..=STEP_HALVINGS {
            let range = [candidate - half, candidate + half];
            half *= 0.5;
            if !fits(walker.closed[1 - k], range) {
                continue;
            }
            let (probe, u, v) = if k == 0 {
                (Probe::Ft, [edge; 2], range)
            } else {
                (Probe::Fs, range, [edge; 2])
            };
            if self.section.sign_over(probe, u, v).is_some()
                && negative(range[0]) != negative(range[1])
            {
                let root = if k == 0 {
                    Some(walker.root_v(edge, range[0], range[1]))
                } else {
                    walker.root_u(edge, range[0], range[1])
                };
                return root
                    .map(EdgeRoot::Crossing)
                    .ok_or(SectionFault::UnresolvedTurning);
            }
        }
        let [u, v] = on_line(candidate);
        if walker.value(u, v).0.abs() > self.tol.linear {
            Ok(EdgeRoot::None)
        } else {
            (self.edge_touch(k, edge, candidate))
                .map(EdgeRoot::Touch)
                .ok_or(SectionFault::UnresolvedTurning)
        }
    }

    /// The touch ([`EdgeTouch`]) `candidate` stands for on the edge line
    /// where direction `k`'s parameter is `edge`, if it is one: about the
    /// least distance along the line near the candidate, the largest cell
    /// whose certificate holds on the section's polynomial, reaching as
    /// far into the domain as along the line. Its stretch is grown from
    /// a tolerance's length until `f` is signed along the edge beyond it,
    /// and every point of it is within the tolerance of the other surface.
    fn edge_touch(&self, k: usize, edge: f64, candidate: f64) -> Option<EdgeTouch> {
        let walker = &self.walker;
        let j = 1 - k;
        let on_line = |x: f64| if k == 0 { [edge, x] } else { [x, edge] };
        let along = |x: f64| {
            let [u, v] = on_line(x);
            walker.value(u, v).0.abs()
        };
        let (mut a, mut b) = (candidate - TOUCH_BRACKET, candidate + TOUCH_BRACKET);
        if !walker.closed[j] {
            (a, b) = (a.max(0.0), b.min(TAU));
        }
        let ratio = 0.5 * (5f64.sqrt() - 1.0);
        for _ in 0..GOLDEN_STEPS {
            let (c, d) = (b - ratio * (b - a), a + ratio * (b - a));
            if along(c) < along(d) {
                b = d;
            } else {
                a = c;
            }
        }
        let x0 = 0.5 * (a + b);
        let at = on_line(x0);
        if along(x0) > self.tol.linear {
            return None;
        }
        let e = walker.eval(at[0], at[1]);
        let speed = [e.du.norm(), e.dv.norm()];
        // The least stretch along the line, and how far in the line is
        // that `f` must be signed along all the cell's width.
        let middle = 0.5 * self.tol.linear / speed[j];
        let depth = 0.5 * self.tol.linear / speed[k];
        let inward = if edge == 0.0 { 1.0 } else { -1.0 };
        let inner = edge + inward * depth;
        let sign = |probe: Probe, x: [f64; 2], y: [f64; 2]| {
            let y = [y[0].min(y[1]), y[0].max(y[1])];
            let (u, v) = if k == 0 { (y, x) } else { (x, y) };
            self.section.sign_over(probe, u, v)
        };
        // `f` along a stretch of a line of the edge's direction, halved
        // where its coefficients do not say: beside the touch it is as
        // small as the miss, and a cell's coefficients are not all of its
        // sign there.
        let sign_along = |x: [f64; 2], y: f64| {
            let mut pending = vec![(x, 0)];
            let mut found = None;
            while let Some((x, depth)) = pending.pop() {
                match sign(Probe::F, x, [y, y]) {
                    Some(s) if found.is_none_or(|f| f == s) => found = Some(s),
                    Some(_) => return None,
                    None if depth < TOUCH_HALVINGS => {
                        let mid = 0.5 * (x[0] + x[1]);
                        pending.extend([([x[0], mid], depth + 1), ([mid, x[1]], depth + 1)]);
                    }
                    None => return None,
                }
            }
            found
        };
        let across = if k == 0 { Probe::Fs } else { Probe::Ft };
        for i in 0..=STEP_HALVINGS {
            let a = CELL_START / (1u64 << i) as f64;
            if a <= 2.0 * middle.max(depth) {
                break;
            }
            let line = [x0 - a, x0 + a];
            if !fits(walker.closed[j], line) {
                continue;
            }
            let Some(slope) = sign(across, line, [edge, edge + inward * a]) else {
                continue;
            };
            // `f` rises into the domain where its slope across the edge
            // is positive inward, and is then positive along the edge.
            let rising = slope == (inward > 0.0);
            if sign_along(line, inner) != Some(rising) {
                continue;
            }
            // How far the stretch reaches from `x0` that way.
            let reach = |way: f64| {
                let mut m = middle;
                while m < a {
                    let beyond = [x0 + way * m, x0 + way * a];
                    let beyond = [beyond[0].min(beyond[1]), beyond[0].max(beyond[1])];
                    if sign_along(beyond, edge) == Some(rising) {
                        return Some(m);
                    }
                    m *= 2.0;
                }
                None
            };
            let (Some(below), Some(above)) = (reach(-1.0), reach(1.0)) else {
                continue;
            };
            let run = [x0 - below, x0 + above];
            let held = (0..=CIRCLE_SAMPLES).all(|n| {
                let x = run[0] + (run[1] - run[0]) * n as f64 / CIRCLE_SAMPLES as f64;
                let [u, v] = on_line(x);
                walker.distance(u, v).0.abs() <= self.tol.linear
            });
            return held.then_some(EdgeTouch {
                at,
                half: [a; 2],
                k,
                run,
            });
        }
        None
    }

    /// The cell of the root `at` on the edge where direction `k`'s
    /// parameter is `0`, or with `high` `2π`: the largest tried whose
    /// certificate holds ([`EdgeCell`]), as tall as a march's cell for
    /// the section's slope there.
    fn edge_cell(&self, k: usize, high: bool, at: [f64; 2]) -> Result<EdgeCell, SectionFault> {
        let walker = &self.walker;
        let closed = walker.closed;
        let slope = walker.value(at[0], at[1]).1;
        let steep = (slope[0] / slope[1]).abs();
        let inward = if high { -1.0 } else { 1.0 };
        for i in 0..=STEP_HALVINGS {
            let a = CELL_START / (1u64 << i) as f64;
            let b = (2.0 * steep * a).max(0.5 * a).max(HALF_MIN);
            if b.is_nan() || b > FRAC_PI_4 {
                continue;
            }
            let sign = |u: [f64; 2], v: f64| self.section.sign_over(Probe::F, u, [v, v]);
            if k == 0 {
                let far = at[0] + inward * a;
                let u = [at[0].min(far), at[0].max(far)];
                let v = [at[1] - b, at[1] + b];
                let proven = fits(closed[1], v)
                    && self.section.sign_over(Probe::Ft, u, v).is_some()
                    && (sign(u, v[0]).zip(sign(u, v[1]))).is_some_and(|(lo, hi)| lo != hi);
                if proven {
                    return Ok(EdgeCell {
                        at,
                        half: [a, b],
                        side: inward,
                        extent: a,
                        bracket: v,
                    });
                }
            } else {
                let inner = at[1] + inward * b;
                let u = [at[0] - a, at[0] + a];
                let v = [at[1].min(inner), at[1].max(inner)];
                let monotone = fits(closed[0], u)
                    && self.section.sign_over(Probe::Ft, u, v).is_some()
                    && self.section.sign_over(Probe::Fs, u, v).is_some();
                let Some(positive) = sign(u, inner).filter(|_| monotone) else {
                    continue;
                };
                // The arm is where the edge line's sign is not the inner
                // edge's.
                let ends = u.map(|u| walker.is_negative(u, at[1]));
                if ends[0] != ends[1] {
                    return Ok(EdgeCell {
                        at,
                        half: [a, b],
                        side: if ends[1] == positive { 1.0 } else { -1.0 },
                        extent: a,
                        bracket: v,
                    });
                }
            }
        }
        Err(SectionFault::UnresolvedTurning)
    }

    /// The arm a march in direction `dir` has reached at `(u, v)`, if it
    /// stands in a cell whose arms point back at it, as far from the
    /// cell's point as the arm is followed there: the cell holds nothing
    /// but its point's arms. Two turning points of an S-bend may have
    /// cells that overlap, and a march between them starts captured.
    fn captured(&self, [u, v]: [f64; 2], dir: f64) -> Option<(ArcEnd, Cell, [f64; 2])> {
        let walker = &self.walker;
        for (i, t) in self.turning.iter().enumerate() {
            if t.side == dir {
                continue;
            }
            for arm in 0..2 {
                if t.open == Some(arm) || t.exits[arm] {
                    continue;
                }
                let ahead = dir * walker.off(0, t.at[0] - u);
                let dv = walker.off(1, v - t.at[1]);
                let inside = if arm == 1 {
                    (0.0..=t.half[1]).contains(&dv)
                } else {
                    (-t.half[1]..=0.0).contains(&dv)
                };
                if (-ANGLE_SLACK..=t.extent[arm] + ANGLE_SLACK).contains(&ahead) && inside {
                    let centre = [u + dir * ahead, v - dv];
                    let bracket = if arm == 1 {
                        [centre[1], centre[1] + t.half[1]]
                    } else {
                        [centre[1] - t.half[1], centre[1]]
                    };
                    let cell = Cell {
                        u: [u.min(centre[0]), u.max(centre[0])],
                        bracket: Bracket::Fixed(bracket),
                    };
                    return Some((ArcEnd::Turning(i, arm), cell, centre));
                }
            }
        }
        for (i, s) in self.walker.singular.iter().enumerate() {
            let ahead = dir * walker.off(0, s.at[0] - u);
            let dv = walker.off(1, v - s.at[1]);
            let near = ahead > 0.0 && ahead <= s.half[0] + ANGLE_SLACK;
            // A march comes to the side of the cell it is on.
            let reached = s.sides[usize::from(dir < 0.0)];
            if s.crossing && near && reached && dv.abs() <= s.half[1] {
                let centre = [u + dir * ahead, v - dv];
                let upper = v > self.walker.separator(s, centre, u);
                let cell = Cell {
                    u: [u.min(centre[0]), u.max(centre[0])],
                    bracket: Bracket::Beside {
                        singular: i,
                        centre,
                        upper,
                    },
                };
                let right = usize::from(dir < 0.0);
                return Some((
                    ArcEnd::Singular(i, 2 * right + usize::from(upper)),
                    cell,
                    centre,
                ));
            }
        }
        for (i, e) in self.edges.iter().enumerate() {
            if e.side == dir {
                continue;
            }
            let ahead = dir * walker.off(0, e.at[0] - u);
            let dv = walker.off(1, v - e.at[1]);
            let within = (e.bracket[0] - e.at[1]..=e.bracket[1] - e.at[1]).contains(&dv);
            if (-ANGLE_SLACK..=e.extent + ANGLE_SLACK).contains(&ahead) && within {
                let centre = [u + dir * ahead, v - dv];
                let bracket = e.bracket.map(|b| b - e.at[1] + centre[1]);
                let cell = Cell {
                    u: [u.min(centre[0]), u.max(centre[0])],
                    bracket: Bracket::Fixed(bracket),
                };
                return Some((ArcEnd::Edge(i), cell, centre));
            }
        }
        None
    }

    /// How far a march at `u` may go in direction `dir` before the next
    /// edge it could be captured at; a whole turn when there is none, and
    /// on a NURBS surface whose `u` does not close no farther than the
    /// domain's edge.
    fn next_edge(&self, u: f64, dir: f64, seeded: bool) -> f64 {
        let turning = (self.turning.iter().filter(|t| t.side != dir)).flat_map(|t| {
            (0..2)
                .filter(|&arm| t.open != Some(arm) && !t.exits[arm])
                .map(|arm| t.at[0] + t.side * t.extent[arm])
        });
        let singular =
            (self.walker.singular.iter().filter(|s| s.crossing)).map(|s| s.at[0] - dir * s.half[0]);
        let edges =
            (self.edges.iter().filter(|e| e.side != dir)).map(|e| e.at[0] + e.side * e.extent);
        let closed = self.walker.closed[0];
        let limit = match (closed, dir > 0.0) {
            (true, _) => TAU,
            (false, true) => TAU - u,
            (false, false) => u,
        };
        turning
            .chain(singular)
            .chain(edges)
            .chain(seeded.then_some(0.0))
            .map(|edge| {
                let ahead = dir * (edge - u);
                if closed { wrap_angle(ahead) } else { ahead }
            })
            .map(|d| if d <= ANGLE_SLACK { f64::INFINITY } else { d })
            .fold(limit, f64::min)
    }

    /// Cells along the graph from `start`, a point of the section, in
    /// direction `dir`, until an arm captures it — or, from a seed on
    /// `u = 0`, until it is back at the seed. The cells come back
    /// ascending in `u`, with what the march ended at and where.
    fn march(
        &self,
        start: [f64; 2],
        dir: f64,
        seeded: bool,
        meter: &mut Meter,
    ) -> Result<(Vec<Cell>, ArcEnd, [f64; 2]), Halt<SectionFault>> {
        let walker = &self.walker;
        let mut cells: Vec<Cell> = Vec::new();
        let [mut u, mut v] = start;
        let mut step = STEP_MAX;
        let mut home = 0.0;
        let (end, last) = loop {
            meter.tick()?;
            if let Some((end, cell, centre)) = self.captured([u, v], dir) {
                cells.push(cell);
                break (end, centre);
            }
            let back = !cells.is_empty() && walker.off(0, u).abs() <= ANGLE_SLACK;
            if seeded && back && walker.off(1, v - start[1]).abs() <= home {
                break (ArcEnd::Home, [u, v]);
            }
            if cells.len() >= MAX_CELLS {
                return Err(SectionFault::UnresolvedTurning.into());
            }
            let edge = self.next_edge(u, dir, seeded);
            // At the edge of the domain and no cell's: a root on the edge
            // line no edge cell holds.
            if edge <= ANGLE_SLACK {
                return Err(SectionFault::UnresolvedTurning.into());
            }
            let slope = walker.value(u, v).1;
            let steep = (slope[0] / slope[1]).abs();
            let mut du = step.min(edge);
            let mut tries = 0;
            let (next, half) = loop {
                let next = u + dir * du;
                let half = (2.0 * steep * du).max(0.5 * du).max(HALF_MIN);
                // On a NURBS surface the box stops at an edge of `v`: a
                // graph that does not cross it there keeps `f` along it
                // of the other sign from below.
                let vs = [v - half, v + half];
                let vs = if walker.closed[1] {
                    vs
                } else {
                    [vs[0].max(0.0), vs[1].min(TAU)]
                };
                let us = [u.min(next), u.max(next)];
                let proven = half <= FRAC_PI_4
                    && self.section.sign_over(Probe::Ft, us, vs).is_some()
                    && (self.section.sign_over(Probe::F, us, [vs[0], vs[0]]))
                        .zip(self.section.sign_over(Probe::F, us, [vs[1], vs[1]]))
                        .is_some_and(|(below, above)| below != above);
                if proven {
                    break (next, half);
                }
                tries += 1;
                if tries > STEP_HALVINGS {
                    return Err(SectionFault::UnresolvedTurning.into());
                }
                du *= 0.5;
            };
            if cells.is_empty() {
                home = half;
            }
            let vs = [v - half, v + half];
            let vs = if walker.closed[1] {
                vs
            } else {
                [vs[0].max(0.0), vs[1].min(TAU)]
            };
            cells.push(Cell {
                u: [u.min(next), u.max(next)],
                bracket: Bracket::Fixed(vs),
            });
            v = walker.root_v(next, vs[0], vs[1]);
            u = next;
            step = (2.0 * du).min(STEP_MAX);
        };
        if dir < 0.0 {
            cells.reverse();
        }
        Ok((cells, end, last))
    }
}

/// One marched graph with what its two ends are.
struct Marched {
    arc: TorusArc,
    ends: [ArcEnd; 2],
}

impl Tracer<'_> {
    /// Every arm marched once, the winding components seeded, and the
    /// graphs chained into branches: open ones first, from the singular
    /// points in their order, then the loops, in the order of the turning
    /// points — an order and orientations the two surfaces alone decide.
    fn assemble(self, meter: &mut Meter) -> Result<SectionTrace, Halt<SectionFault>> {
        let mut marched: Vec<Marched> = Vec::new();
        let taken =
            |marched: &[Marched], end: ArcEnd| marched.iter().any(|m| m.ends.contains(&end));
        for (i, t) in self.turning.iter().enumerate() {
            for arm in 0..2 {
                if t.open == Some(arm) || taken(&marched, ArcEnd::Turning(i, arm)) {
                    continue;
                }
                if t.exits[arm] {
                    marched.push(self.exit(i, arm));
                    continue;
                }
                let edge = t.at[0] + t.side * t.extent[arm];
                let bracket = if arm == 1 {
                    [t.at[1], t.at[1] + t.half[1]]
                } else {
                    [t.at[1] - t.half[1], t.at[1]]
                };
                let first = Cell {
                    u: [edge.min(t.at[0]), edge.max(t.at[0])],
                    bracket: Bracket::Fixed(bracket),
                };
                let start = [edge, self.walker.root_v(edge, bracket[0], bracket[1])];
                marched.push(self.arc_from(
                    first,
                    t.at,
                    start,
                    t.side,
                    ArcEnd::Turning(i, arm),
                    meter,
                )?);
            }
        }
        for (i, s) in self.walker.singular.iter().enumerate() {
            for arm in 0..4 {
                let beyond = !s.sides[usize::from(arm >= 2)] || !s.rows[arm % 2];
                if !s.crossing || beyond || taken(&marched, ArcEnd::Singular(i, arm)) {
                    continue;
                }
                let (dir, upper) = (if arm >= 2 { 1.0 } else { -1.0 }, arm % 2 == 1);
                let edge = s.at[0] + dir * s.half[0];
                let first = Cell {
                    u: [edge.min(s.at[0]), edge.max(s.at[0])],
                    bracket: Bracket::Beside {
                        singular: i,
                        centre: s.at,
                        upper,
                    },
                };
                let probe = TorusArc {
                    cells: vec![first],
                    ends: [s.at; 2],
                    folds: [None; 2],
                };
                let start = [edge, self.walker.v_on(&probe, edge)];
                marched.push(self.arc_from(
                    first,
                    s.at,
                    start,
                    dir,
                    ArcEnd::Singular(i, arm),
                    meter,
                )?);
            }
        }
        for (i, e) in self.edges.iter().enumerate() {
            if taken(&marched, ArcEnd::Edge(i)) {
                continue;
            }
            let edge = e.at[0] + e.side * e.extent;
            let first = Cell {
                u: [edge.min(e.at[0]), edge.max(e.at[0])],
                bracket: Bracket::Fixed(e.bracket),
            };
            let start = [edge, self.walker.root_v(edge, e.bracket[0], e.bracket[1])];
            marched.push(self.arc_from(first, e.at, start, e.side, ArcEnd::Edge(i), meter)?);
        }
        // Components that wind round a `u` that closes cross `u = 0`; on a
        // NURBS surface whose `u` does not close every component turns,
        // or ends on an edge.
        let seeds = if self.walker.closed[0] {
            self.section.seam_candidates()
        } else {
            Vec::new()
        };
        for seed in seeds {
            if let Some(start) = self.seed(seed, &marched)? {
                let (cells, end, last) = self.march(start, 1.0, true, meter)?;
                // A component with a turning point or a singular one was
                // marched from there, and its roots on `u = 0` are no
                // seeds.
                if end != ArcEnd::Home {
                    return Err(SectionFault::UnresolvedTurning.into());
                }
                marched.push(Marched {
                    arc: TorusArc {
                        cells,
                        ends: [start, last],
                        folds: [None; 2],
                    },
                    ends: [ArcEnd::Home, end],
                });
            }
        }
        Ok(self.chained(marched)?)
    }

    /// The arm `arm` of turning point `i`, which leaves the domain inside
    /// the point's cell: from the point to the edge, through the side the
    /// cell is cut at in `u` — the end the root along that edge line — or
    /// the edge it is cut at in `v`, where the end is on the edge.
    fn exit(&self, i: usize, arm: usize) -> Marched {
        let t = &self.turning[i];
        let end = t.at[0] + t.side * t.extent[arm];
        let bracket = if arm == 1 {
            [t.at[1], t.at[1] + t.half[1]]
        } else {
            [t.at[1] - t.half[1], t.at[1]]
        };
        let bracket = if self.walker.closed[1] {
            bracket
        } else {
            [bracket[0].max(0.0), bracket[1].min(TAU)]
        };
        let v = if !self.walker.closed[0] && (end == 0.0 || end == TAU) {
            self.walker.root_v(end, bracket[0], bracket[1])
        } else {
            bracket[arm]
        };
        Marched {
            arc: TorusArc {
                cells: vec![Cell {
                    u: [t.at[0].min(end), t.at[0].max(end)],
                    bracket: Bracket::Fixed(bracket),
                }],
                ends: [t.at, [end, v]],
                folds: [None; 2],
            },
            ends: [ArcEnd::Turning(i, arm), ArcEnd::Exit(i, arm)],
        }
    }

    /// The graph that starts with `first`, an arm's own cell, and is
    /// marched on from `start` in direction `dir`.
    fn arc_from(
        &self,
        first: Cell,
        from: [f64; 2],
        start: [f64; 2],
        dir: f64,
        end: ArcEnd,
        meter: &mut Meter,
    ) -> Result<Marched, Halt<SectionFault>> {
        let (mut cells, last_end, last) = self.march(start, dir, false, meter)?;
        if dir < 0.0 {
            cells.push(first);
        } else {
            cells.insert(0, first);
        }
        Ok(Marched {
            arc: TorusArc {
                cells,
                ends: [from, last],
                folds: [None; 2],
            },
            ends: [end, last_end],
        })
    }

    /// The root of the distance along `u = 0` that `candidate` stands
    /// for, when it is one no cell and no marched graph accounts for.
    fn seed(&self, candidate: f64, marched: &[Marched]) -> Result<Option<[f64; 2]>, SectionFault> {
        let walker = &self.walker;
        let closed = walker.closed;
        let at = [0.0, candidate];
        let in_cell = (self
            .turning
            .iter()
            .any(|t| in_box(closed, at, t.at, t.half)))
            || (walker.singular.iter()).any(|s| in_box(closed, at, s.at, s.half))
            || (self.edges.iter()).any(|e| in_box(closed, at, e.at, e.half));
        // A graph accounts for every root inside its cells' brackets at
        // each of its crossings of `u = 0`.
        let on_graph = marched.iter().any(|m| {
            m.arc.cells.iter().any(|cell| {
                let turn = (cell.u[0] / TAU).ceil() * TAU;
                let Bracket::Fixed([lo, hi]) = cell.bracket else {
                    return false;
                };
                let above = if closed[1] {
                    wrap_angle(candidate - lo)
                } else {
                    candidate - lo
                };
                turn <= cell.u[1] && (0.0..=hi - lo).contains(&above)
            })
        });
        if in_cell || on_graph {
            return Ok(None);
        }
        let mut half = CELL_START;
        for _ in 0..=STEP_HALVINGS {
            let v = [candidate - half, candidate + half];
            let one =
                fits(closed[1], v) && self.section.sign_over(Probe::Ft, [0.0, 0.0], v).is_some();
            if one && walker.is_negative(0.0, v[0]) != walker.is_negative(0.0, v[1]) {
                return Ok(Some([0.0, walker.root_v(0.0, v[0], v[1])]));
            }
            half *= 0.5;
        }
        // Off the other surface by more than the tolerance, the candidate
        // was no root. Joined along the seam to a singular point on it by
        // a run within the tolerance, it is that point: where the two
        // surfaces are tangent their distance is quadratic, within the
        // tolerance over a run far wider than the point's own cell, and
        // the seam's root search lands anywhere on it (NIST's FTC-07, a
        // plane tangent to a torus's rim on its seam). A root that cannot
        // be proven alone otherwise is a fault.
        let within = |v: f64| walker.value(0.0, v).0.abs() <= self.tol.linear;
        if !within(candidate) {
            return Ok(None);
        }
        let touch = walker.singular.iter().any(|s| {
            let du = wrap_angle(s.at[0]);
            du.min(TAU - du) <= s.half[0]
                && (0..=SEAM_RUN_SAMPLES).all(|i| {
                    let f = i as f64 / SEAM_RUN_SAMPLES as f64;
                    within(candidate + f * (s.at[1] - candidate))
                })
        });
        if touch {
            Ok(None)
        } else {
            Err(SectionFault::UnresolvedTurning)
        }
    }

    /// How graph `m` is evaluated beside the turning point its end `side`
    /// is, if it is one ([`Fold`]).
    fn fold(&self, m: &Marched, side: usize) -> Option<Fold> {
        let ArcEnd::Turning(i, _) = m.ends[side] else {
            return None;
        };
        let t = self.turning.get(i)?;
        let at = *m.arc.ends.get(side)?;
        // The arm lies to the turning point's side in `u`, and its own
        // cell is the graph's first or last as the graph ascends; it is
        // walked in `v` to `FOLD_REACH` from the turn, `c·FOLD_REACH²` in
        // `u`, or to the cell's edge if that is nearer.
        let cell = if t.side > 0.0 {
            m.arc.cells.first()?.u[1]
        } else {
            m.arc.cells.last()?.u[0]
        };
        let reach = (cell - at[0]).abs().min(t.fold * FOLD_REACH * FOLD_REACH);
        let handover = at[0] + t.side * reach;
        if reach <= ANGLE_SLACK {
            return None;
        }
        let v = self.walker.v_on(&m.arc, handover);
        let (w, sign) = ((v - at[1]).abs(), (v - at[1]).signum());
        let slope = self.walker.value(handover, v).1;
        let r = reach.sqrt();
        // The walk along `u`'s `|v − v_T|` against `r` where it takes
        // over, and its slope in `r`: `d|v|/d(off) · 2r`.
        let d = (slope[0] / slope[1]).abs() * 2.0 * r;
        let a1 = t.fold.sqrt().recip();
        let (e1, e2) = (w - a1 * r, d - a1);
        let a3 = (e2 * r - 2.0 * e1) / (r * r * r);
        let a2 = (3.0 * e1 - e2 * r) / (r * r);
        // Rising all the way, or the arm is `v` linear in `r` to its
        // handover — continuous, and no longer smooth there.
        let rise = |x: f64| a1 + x * (2.0 * a2 + 3.0 * a3 * x);
        let vertex = -a2 / (3.0 * a3);
        let rising = [a1, a2, a3].iter().all(|a| a.is_finite())
            && rise(0.0) > 0.0
            && rise(r) > 0.0
            && !(vertex > 0.0 && vertex < r && rise(vertex) <= 0.0);
        let coeffs = if rising {
            [a1, a2, a3]
        } else {
            [w / r, 0.0, 0.0]
        };
        (w > 0.0).then_some(Fold {
            at,
            reach,
            coeffs: coeffs.map(|a| sign * a),
            span: [at[0] - t.half[0], at[0] + t.half[0]],
        })
    }

    fn chained(self, marched: Vec<Marched>) -> Result<SectionTrace, SectionFault> {
        let mut circles = self.section_circles()?;
        let (touched, runs) = self.touch_runs()?;
        circles.extend(runs);
        let folds: Vec<[Option<Fold>; 2]> = (marched.iter())
            .map(|m| [0, 1].map(|side| self.fold(m, side)))
            .collect();
        let tol = self.tol;
        // A turning point on an edge of the domain has one arm, and a
        // branch ends there as at an edge cell.
        let opens: Vec<bool> = self.turning.iter().map(|t| t.open.is_some()).collect();
        let ends_open = |end: &ArcEnd| match *end {
            ArcEnd::Singular(..) | ArcEnd::Edge(_) | ArcEnd::Exit(..) => true,
            ArcEnd::Turning(i, _) => opens.get(i).copied().unwrap_or(false),
            ArcEnd::Home => false,
        };
        let walker = Arc::new(self.walker);
        // The graph that holds the other arm of a turning point.
        let across = |end: ArcEnd| -> Option<(usize, usize)> {
            let ArcEnd::Turning(i, arm) = end else {
                return None;
            };
            let other = ArcEnd::Turning(i, 1 - arm);
            marched.iter().enumerate().find_map(|(k, m)| {
                let side = m.ends.iter().position(|e| *e == other)?;
                Some((k, side))
            })
        };
        let mut visited = vec![false; marched.len()];
        let mut chains: Vec<Vec<(usize, bool)>> = Vec::new();
        let walk = |start: usize, enter: usize, visited: &mut Vec<bool>| {
            let mut steps = Vec::new();
            let (mut k, mut side) = (start, enter);
            loop {
                visited[k] = true;
                steps.push((k, side == 0));
                match across(marched[k].ends[1 - side]) {
                    Some((next, enter)) if !visited[next] => (k, side) = (next, enter),
                    Some(_) => break Ok(steps),
                    None if matches!(marched[k].ends[1 - side], ArcEnd::Turning(..))
                        && !ends_open(&marched[k].ends[1 - side]) =>
                    {
                        break Err(SectionFault::UnresolvedTurning);
                    }
                    None => break Ok(steps),
                }
            }
        };
        let mut open: Vec<(ArcEnd, usize, usize)> = Vec::new();
        for (k, m) in marched.iter().enumerate() {
            for (side, end) in m.ends.iter().enumerate() {
                if ends_open(end) {
                    open.push((*end, k, side));
                }
            }
        }
        open.sort();
        for (_, k, side) in open {
            if !visited[k] {
                chains.push(walk(k, side, &mut visited)?);
            }
        }
        for k in 0..marched.len() {
            if !visited[k] {
                chains.push(walk(k, 0, &mut visited)?);
            }
        }

        // The points branches end at, as `(u, v)` in `[0, 2π)²`: the
        // singular points, and after them the crossings of a tube circle
        // that lies on the other surface, as the chains come to them.
        let mut ends_at: Vec<([f64; 2], bool)> = (walker.singular.iter())
            .map(|s| (s.at, !s.crossing))
            .collect();
        let stop = |end: ArcEnd| match end {
            ArcEnd::Singular(i, _) => Some(Stop::At(i)),
            ArcEnd::Edge(_) | ArcEnd::Exit(..) => Some(Stop::Edge),
            ArcEnd::Turning(..) if ends_open(&end) => Some(Stop::Edge),
            ArcEnd::Turning(..) | ArcEnd::Home => None,
        };
        let turns = |end: ArcEnd| matches!(end, ArcEnd::Turning(..));
        // A point's `v` in `[0, 2π)` where `v` closes, as it is where not.
        let wrap_v = |v: f64| if walker.closed[1] { wrap_angle(v) } else { v };
        let mut cut: Vec<Vec<Piece>> = Vec::new();
        for chain in &chains {
            let mut pieces: Vec<Piece> = Vec::new();
            for &(k, forward) in chain {
                let m = &marched[k];
                let (from, to) = if forward { (0, 1) } else { (1, 0) };
                let (start, finish) = (m.arc.ends[from], m.arc.ends[to]);
                // Where the graph crosses the circle, in the order it
                // runs: a graph seeded on the circle starts and ends on
                // it, at one point.
                let mut stops: Vec<([f64; 2], Option<Stop>)> = Vec::new();
                let seeded = walker
                    .factor
                    .filter(|f| m.ends[from] == ArcEnd::Home && f.u0 == 0.0);
                let home = seeded.map(|f| {
                    ends_at.push(([f.u0, wrap_v(start[1])], false));
                    Stop::At(ends_at.len() - 1)
                });
                stops.push((start, stop(m.ends[from]).or(home)));
                if let Some(factor) = walker.factor {
                    let (lo, hi) = (start[0].min(finish[0]), start[0].max(finish[0]));
                    let first = ((lo - factor.u0) / TAU).floor() + 1.0;
                    let mut inside: Vec<f64> = (0..)
                        .map(|n| factor.u0 + TAU * (first + n as f64))
                        .take_while(|u| *u < hi - 4.0 * ANGLE_SLACK)
                        .filter(|u| *u > lo + 4.0 * ANGLE_SLACK)
                        .collect();
                    if start[0] > finish[0] {
                        inside.reverse();
                    }
                    for u in inside {
                        let v = walker.v_on(&m.arc, u);
                        ends_at.push(([factor.u0, wrap_v(v)], false));
                        stops.push(([u, v], Some(Stop::At(ends_at.len() - 1))));
                    }
                }
                stops.push((finish, stop(m.ends[to]).or(home)));
                let last = stops.len() - 1;
                for (i, w) in stops.windows(2).enumerate() {
                    pieces.push(Piece {
                        graph: k,
                        from: w[0].0,
                        to: w[1].0,
                        turns: [
                            i == 0 && turns(m.ends[from]),
                            i + 1 == last && turns(m.ends[to]),
                        ],
                        ends: [w[0].1, w[1].1],
                        fold: [
                            (i == 0).then(|| folds[k][from]).flatten(),
                            (i + 1 == last).then(|| folds[k][to]).flatten(),
                        ],
                    });
                }
            }
            // A loop that crosses the circle starts at a crossing.
            if pieces.first().is_some_and(|p| p.ends[0].is_none()) {
                if let Some(i) = pieces.iter().position(|p| p.ends[0].is_some()) {
                    pieces.rotate_left(i);
                }
            }
            let mut group: Vec<Piece> = Vec::new();
            for piece in pieces {
                let ended = piece.ends[1].is_some();
                group.push(piece);
                if ended {
                    cut.push(core::mem::take(&mut group));
                }
            }
            if !group.is_empty() {
                cut.push(group);
            }
        }

        // The touches of an edge, isolated, after the crossings.
        let crossings_end = ends_at.len();
        ends_at.extend(touched.into_iter().map(|at| (at, true)));
        // The points in their documented order, ascending by `u` and
        // then `v`, and where each went.
        let mut order: Vec<usize> = (0..ends_at.len()).collect();
        order.sort_by(|&a, &b| {
            let (p, q) = (ends_at[a].0, ends_at[b].0);
            p[0].total_cmp(&q[0]).then(p[1].total_cmp(&q[1]))
        });
        let mut index = vec![0; ends_at.len()];
        for (now, &was) in order.iter().enumerate() {
            index[was] = now;
        }
        let points: Vec<SectionPoint> = (order.iter())
            .map(|&i| SectionPoint {
                point: walker.point(ends_at[i].0[0], ends_at[i].0[1]),
                isolated: ends_at[i].1,
            })
            .collect();
        // Two crossings of the circle within the tolerance of each other
        // are a touch, or a crossing, that the tolerance does not decide.
        let crossings = &order[..];
        for (i, &a) in crossings.iter().enumerate() {
            for &b in &crossings[i + 1..] {
                let crossing = |i: usize| (walker.singular.len()..crossings_end).contains(&i);
                let both = crossing(a) && crossing(b);
                let apart = points[index[a]].point - points[index[b]].point;
                if both && apart.norm() <= 2.0 * tol.linear {
                    return Err(SectionFault::CrowdedSingularity);
                }
            }
        }

        let mut branches = Vec::new();
        for group in cut {
            let mut arcs = Vec::new();
            let mut graphs = Vec::new();
            let mut shifts: Vec<[f64; 2]> = Vec::new();
            let mut reached: Option<[f64; 2]> = None;
            for piece in &group {
                arcs.push(Arc1::new(
                    piece.from[0],
                    piece.to[0],
                    piece.turns[0],
                    piece.turns[1],
                ));
                // Whole turns, so that this graph starts where the last
                // one ended, and the first inside `[0, 2π)`; none in a
                // direction that does not close.
                let shift = [0, 1].map(|c| match reached {
                    _ if !walker.closed[c] => 0.0,
                    Some(at) => TAU * ((at[c] - piece.from[c]) / TAU).round(),
                    None => -TAU * (piece.from[c] / TAU).floor(),
                });
                reached = Some([0, 1].map(|c| piece.to[c] + shift[c]));
                shifts.push(shift);
                graphs.push(TorusArc {
                    cells: marched[piece.graph].arc.cells.clone(),
                    ends: [piece.from, piece.to],
                    folds: piece.fold,
                });
            }
            let (Some(first), Some(last)) = (group.first(), group.last()) else {
                continue;
            };
            let end = |s: Stop| match s {
                Stop::At(i) => BranchEnd::Singular(index[i]),
                Stop::Edge => BranchEnd::Clipped,
            };
            let ends = (first.ends[0].zip(last.ends[1])).map(|(a, b)| [end(a), end(b)]);
            // An edge's end is where the graph's end is, on the edge line.
            let pin = |s: Option<Stop>, at: [f64; 2]| match s? {
                Stop::At(i) => points.get(index[i]).map(|p| p.point),
                Stop::Edge => Some(walker.point(at[0], at[1])),
            };
            let uv_of = |at: [f64; 2], shift: &[f64; 2]| [0, 1].map(|c| at[c] + shift[c]);
            let (pins, pins_uv) = match (ends, shifts.first(), shifts.last()) {
                (Some(_), Some(sa), Some(sb)) => (
                    [pin(first.ends[0], first.from), pin(last.ends[1], last.to)],
                    [Some(uv_of(first.from, sa)), Some(uv_of(last.to, sb))],
                ),
                _ => ([None; 2], [None; 2]),
            };
            let walk = TorusWalk {
                walker: Arc::clone(&walker),
                arcs: graphs,
                shifts,
                pins: pins_uv,
            };
            branches.push(SectionBranch::on_torus(walk, arcs, ends, pins));
        }
        Ok(SectionTrace::new(branches, points).with_circles(circles))
    }
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
