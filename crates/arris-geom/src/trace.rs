//! The section of two quadrics, one of them ruled, traced exactly: every
//! branch, its ends and the singular points where branches meet, before
//! any fit (`docs/DATA-MODEL.md` §Curves).
//!
//! The ruled operand — a cylinder, an elliptic cylinder or a cone — is a
//! family of lines indexed by the angle `s` of its parametrisation: a
//! cylinder's rulings `p(s) + w·Z`, a cone's `apex + w·d(s)`. Substituted
//! into the other quadric's implicit form, a ruling meets it where
//! `a(s)w² + b(s)w + c(s) = 0`. With a cone's rulings taken from its apex
//! the discriminant `D = b² − 4ac` is a trigonometric polynomial of degree
//! two in `s` for every ruled kind, so where the number of hits changes —
//! the turning points of the section, and the points where the surfaces
//! are tangent — is the roots of one quartic in `tan(s/2)`. The topology
//! of the section is therefore algebraic: no loop can fall between two
//! samples, because nothing is sampled.
//!
//! Between turning points the two roots `w±(s)` are two arcs of the
//! section. Arcs are joined where they turn, through `s = s_T ± L(1 −
//! cos θ)`, which removes the square root's singularity, so a branch is
//! one smooth callable, closed or open. A critical point of `D` where the
//! surfaces come within `tol.linear` of tangency is a **singular point**:
//! there `D` is replaced by `D − E`, `E` a bump of `D`'s own value with a
//! reach of twice the angle at which `D` would have vanished, so that the
//! corrected discriminant has an exact double root. Branches end at the
//! singular point exactly, stay on the walked surface to rounding and
//! within `tol.linear` of the other; a gap, a near miss and a loop smaller
//! than the tolerance all become the one point they are at that
//! tolerance.
//!
//! The family is unbounded along its rulings, and so may the section be
//! (two cones, a cylinder along a cone's ruling). Every branch is clipped
//! to the extent of the caller's region along the rulings: a root that
//! leaves it ends its branch there. The clip's ends are again the roots
//! of a quartic.

use arris_math::real;
use core::f64::consts::{FRAC_PI_4, PI, TAU};
use core::fmt;
use std::sync::Arc;

use arris_math::roots::{POLYNOMIAL_ROUNDING, newton_in_interval, quartic};
use arris_math::{
    Aabb, Frame, Interval, Meter, Point2, Point3, RELATIVE_ROUNDING, Tolerance, Vec3, wrap_angle,
};

use arris_math::nalgebra::{Matrix4, Vector4};

use crate::torus_walk::TorusWalk;
use crate::{Curve, GeomError, GeomKind, NurbsSurface, Surface};

/// How many parameters a branch is sampled at to decide that it is no
/// longer than the tolerance, and is its end point rather than a curve:
/// the apex of a walked cone lying on the other surface makes one root
/// of every ruling that apex. A count, not a tolerance; an arc between
/// two breakpoints of a quartic has no more than a few extrema.
const EXTENT_SAMPLES: usize = 17;

/// The reach of a singular point's correction, as a multiple of the
/// angle `√(|D| / κ)` at which the uncorrected discriminant `D ≈ D_e +
/// κ·x²` vanishes or would have: the corrected one is `κx² + D_e·S(x/ρ)`
/// with `S(y) ≤ 3y²`, which keeps its sign for every `ρ² > 3·|D_e|/κ`.
/// Two is the smallest whole multiple that does. A ratio, not a
/// tolerance.
pub(crate) const REACH: f64 = 2.0;

/// Why a section is one its tracer does not resolve. Each is a pose of
/// measure zero that a closed form owns or that needs a decision the
/// tracer will not guess.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SectionFault {
    /// The surfaces are tangent along a whole arc of the section, or
    /// everywhere: every ruling of an interval touches the other surface
    /// within the tolerance. A sphere in a cylinder of its radius, a cone
    /// around a sphere — the shared-axis arms decide those (ADR-0008).
    TangentAlongCurve,
    /// A ruling of the walked surface lies on the other surface, so the
    /// section holds a line: two cones sharing a ruling, a cylinder
    /// through a cone's ruling.
    SharedRuling,
    /// A singular point whose discriminant is flat to rounding, or with a
    /// turning point or another singular point inside its reach: three
    /// or more roots of the discriminant within the tolerance of one
    /// another, a cusp of the section.
    CrowdedSingularity,
    /// Tube circles of the walked torus lie along the other surface and
    /// the tracer does not part the rest of the section from them. One
    /// on the other surface within the tolerance is answered
    /// ([`SectionTrace::circles`]), and so are two with nothing else. This
    /// is two with more of the section besides — an elliptic cylinder
    /// along a chord of the centre circle, a circular section of either
    /// family on a tube circle — and a circle the other surface runs
    /// along without holding it to the tolerance everywhere the rest is
    /// traced: a cone a fraction of the tolerance off the pose, its apex
    /// by the torus. Of a NURBS surface the same for its isolines on the
    /// other surface: two or more with more of the section besides, or
    /// isolines of both its directions.
    TubeCircle,
    /// Turning points of a torus section that `f64` does not tell apart,
    /// away from any singular point: a turning point that is an
    /// inflection as well, or a stretch of the section too close to
    /// another to be followed alone. Of a NURBS surface's section also:
    /// the section touching an edge of the surface's domain and entering
    /// the domain there, or crossing the edge at a corner, and the section
    /// turning back at a kink of the surface, a knot where it is only
    /// continuous.
    UnresolvedTurning,
    /// The walk of the section of two NURBS surfaces (ADR-0068) did not
    /// account for every seed: a seed no walk reached, a walk that came
    /// back to a seed another walk had passed, or one that stalled or
    /// left a domain away from every seed. The section has a stretch the
    /// walk could not follow alone, and nothing of it is returned.
    UnvisitedSeed,
}

impl fmt::Display for SectionFault {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SectionFault::TangentAlongCurve => "the surfaces are tangent along a curve",
            SectionFault::SharedRuling => "a ruling of one surface lies on the other",
            SectionFault::CrowdedSingularity => {
                "three or more turning points within the tolerance of one another"
            }
            SectionFault::TubeCircle => {
                "tube circles of the torus lie along the other surface with more of the section"
            }
            SectionFault::UnresolvedTurning => {
                "turning points of the section that cannot be told apart"
            }
            SectionFault::UnvisitedSeed => "a seed of the marched section that no walk reached",
        })
    }
}

/// A trigonometric polynomial of degree two,
/// `k + c1·cos s + s1·sin s + c2·cos 2s + s2·sin 2s`.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Trig2 {
    k: f64,
    c1: f64,
    s1: f64,
    c2: f64,
    s2: f64,
}

/// A trigonometric polynomial of degree one, `[k, c, s]`.
type Trig1 = [f64; 3];

impl Trig2 {
    const ZERO: Trig2 = Trig2 {
        k: 0.0,
        c1: 0.0,
        s1: 0.0,
        c2: 0.0,
        s2: 0.0,
    };

    fn constant(k: f64) -> Trig2 {
        Trig2 { k, ..Trig2::ZERO }
    }

    fn linear([k, c1, s1]: Trig1) -> Trig2 {
        Trig2 {
            k,
            c1,
            s1,
            ..Trig2::ZERO
        }
    }

    /// `f·g`, by `cos² = (1 + cos 2s)/2`, `sin² = (1 − cos 2s)/2` and
    /// `sin·cos = sin 2s / 2`.
    fn product([f0, fc, fs]: Trig1, [g0, gc, gs]: Trig1) -> Trig2 {
        Trig2 {
            k: f0 * g0 + 0.5 * (fc * gc + fs * gs),
            c1: f0 * gc + fc * g0,
            s1: f0 * gs + fs * g0,
            c2: 0.5 * (fc * gc - fs * gs),
            s2: 0.5 * (fc * gs + fs * gc),
        }
    }

    fn plus(self, o: Trig2) -> Trig2 {
        Trig2 {
            k: self.k + o.k,
            c1: self.c1 + o.c1,
            s1: self.s1 + o.s1,
            c2: self.c2 + o.c2,
            s2: self.s2 + o.s2,
        }
    }

    fn scaled(self, by: f64) -> Trig2 {
        Trig2 {
            k: self.k * by,
            c1: self.c1 * by,
            s1: self.s1 * by,
            c2: self.c2 * by,
            s2: self.s2 * by,
        }
    }

    fn eval(&self, s: f64) -> f64 {
        let (sin, cos) = real::sin_cos(s);
        let (sin2, cos2) = (2.0 * sin * cos, (cos - sin) * (cos + sin));
        self.k + self.c1 * cos + self.s1 * sin + self.c2 * cos2 + self.s2 * sin2
    }

    fn derivative(&self) -> Trig2 {
        Trig2 {
            k: 0.0,
            c1: self.s1,
            s1: -self.c1,
            c2: 2.0 * self.s2,
            s2: -2.0 * self.c2,
        }
    }

    /// The sum of the coefficients' magnitudes: a bound on `|eval|`, and
    /// the scale its rounding is measured against.
    fn sum_abs(&self) -> f64 {
        self.k.abs() + self.c1.abs() + self.s1.abs() + self.c2.abs() + self.s2.abs()
    }

    /// `σ ↦ self(s0 + σ)`.
    fn rotated(&self, s0: f64) -> Trig2 {
        let (sin, cos) = real::sin_cos(s0);
        let (sin2, cos2) = real::sin_cos(2.0 * s0);
        Trig2 {
            k: self.k,
            c1: self.c1 * cos + self.s1 * sin,
            s1: self.s1 * cos - self.c1 * sin,
            c2: self.c2 * cos2 + self.s2 * sin2,
            s2: self.s2 * cos2 - self.c2 * sin2,
        }
    }

    /// `self(anchor + 2h) − self(anchor)` without the cancellation of two
    /// evaluations, by the sum-to-product identities: the difference is
    /// `2 sin h` times a bounded factor, so it keeps its relative accuracy
    /// as `h → 0` — which is where a turning point's square root needs it.
    fn difference(&self, anchor: f64, h: f64) -> f64 {
        let (sin_h, cos_h) = real::sin_cos(h);
        let (sin_m, cos_m) = real::sin_cos(anchor + h);
        let (sin_2m, cos_2m) = (2.0 * sin_m * cos_m, (cos_m - sin_m) * (cos_m + sin_m));
        2.0 * sin_h
            * (self.s1 * cos_m - self.c1 * sin_m
                + 2.0 * cos_h * (self.s2 * cos_2m - self.c2 * sin_2m))
    }

    /// The roots on the circle, ascending in `[0, 2π)`, or `None` when
    /// the polynomial vanishes identically to rounding. `t = tan(σ/2)`
    /// turns it into a quartic whose leading coefficient is the value at
    /// `σ = π`, the substitution's blind spot; the circle is turned first
    /// so that the blind spot is where the polynomial is largest among
    /// eight places, and a polynomial of degree two that is negligible at
    /// all eight is negligible everywhere.
    fn roots(&self) -> Option<Vec<f64>> {
        let mag = self.sum_abs();
        if !(mag.is_finite() && mag > 0.0) {
            return None;
        }
        let mut best = (0usize, 0.0f64);
        for k in 0..8 {
            let v = self.eval(k as f64 * FRAC_PI_4 + PI).abs();
            if v > best.1 {
                best = (k, v);
            }
        }
        if best.1 <= POLYNOMIAL_ROUNDING * mag {
            return None;
        }
        let s0 = best.0 as f64 * FRAC_PI_4;
        let g = self.rotated(s0);
        let found = quartic(
            g.k - g.c1 + g.c2,
            2.0 * g.s1 - 4.0 * g.s2,
            2.0 * g.k - 6.0 * g.c2,
            2.0 * g.s1 + 4.0 * g.s2,
            g.k + g.c1 + g.c2,
        )
        .ok()?;
        let slope = self.derivative();
        let mut out: Vec<f64> = found
            .iter()
            .map(|r| self.polished(&slope, s0 + 2.0 * real::atan(r.value)))
            .map(wrap_angle)
            .collect();
        out.sort_by(f64::total_cmp);
        // Two roots an angle's own rounding apart are one root: a double
        // root whose quartic the turn's `sin π ≠ 0` split.
        let same = RELATIVE_ROUNDING * TAU;
        out.dedup_by(|later, first| *later - *first <= same);
        if let [first, .., last] = out[..] {
            if first + TAU - last <= same {
                out.pop();
            }
        }
        Some(out)
    }

    /// A root of the quartic in `tan(σ/2)` brought back to the circle and
    /// improved by Newton steps on the polynomial itself, each kept only
    /// when it lowers the residual — a double root, where the slope
    /// vanishes, keeps the quartic's own answer.
    fn polished(&self, slope: &Trig2, mut s: f64) -> f64 {
        let mut residual = self.eval(s).abs();
        for _ in 0..3 {
            let d = slope.eval(s);
            if d == 0.0 {
                break;
            }
            let next = s - self.eval(s) / d;
            let r = self.eval(next).abs();
            if !(next.is_finite() && r < residual) {
                break;
            }
            (s, residual) = (next, r);
        }
        s
    }
}

/// A vector-valued trigonometric polynomial of degree one,
/// `k + c·cos s + s·sin s`.
#[derive(Debug, Clone, Copy)]
struct TrigVec {
    k: Vec3,
    c: Vec3,
    s: Vec3,
}

impl TrigVec {
    fn eval(&self, s: f64) -> Vec3 {
        let (sin, cos) = real::sin_cos(s);
        self.k + self.c * cos + self.s * sin
    }

    /// The derivative in `s`.
    fn slope(&self, s: f64) -> Vec3 {
        let (sin, cos) = real::sin_cos(s);
        self.s * cos - self.c * sin
    }

    fn component(&self, i: usize) -> Trig1 {
        [self.k[i], self.c[i], self.s[i]]
    }

    fn abs(&self) -> TrigVec {
        TrigVec {
            k: self.k.abs(),
            c: self.c.abs(),
            s: self.s.abs(),
        }
    }
}

/// A quadric in its own frame, `Σ mᵢxᵢ² + 2 l·x + k`, scaled so that its
/// value is a length squared and its gradient a length: the value over
/// the gradient's norm is then the distance to the surface, to first
/// order.
#[derive(Debug, Clone, Copy)]
struct Quadric {
    m: Vec3,
    l: Vec3,
    k: f64,
}

impl Quadric {
    fn value(&self, x: Vec3) -> f64 {
        self.m.dot(&x.component_mul(&x)) + 2.0 * self.l.dot(&x) + self.k
    }

    fn gradient(&self, x: Vec3) -> Vec3 {
        2.0 * (self.m.component_mul(&x) + self.l)
    }

    fn abs(&self) -> Quadric {
        Quadric {
            m: self.m.abs(),
            l: self.l.abs(),
            k: self.k.abs(),
        }
    }

    /// `a`, `b` and `c` of a ruling family against this quadric as
    /// trigonometric polynomials. `b` is of degree one for every family:
    /// either the direction or the base point is constant.
    fn against(&self, p: &TrigVec, d: &TrigVec) -> (Trig2, Trig1, Trig2) {
        let mut a = Trig2::ZERO;
        let mut b = [0.0; 3];
        let mut c = Trig2::constant(self.k);
        for i in 0..3 {
            let (pi, di) = (p.component(i), d.component(i));
            a = a.plus(Trig2::product(di, di).scaled(self.m[i]));
            c = c
                .plus(Trig2::product(pi, pi).scaled(self.m[i]))
                .plus(Trig2::linear(pi).scaled(2.0 * self.l[i]));
            // One of `pi`, `di` is constant, so the product is of degree
            // one and its second harmonics are exactly zero.
            let pd = Trig2::product(di, pi);
            for (slot, term) in b.iter_mut().zip([pd.k, pd.c1, pd.s1]) {
                *slot += 2.0 * self.m[i] * term;
            }
            for (slot, term) in b.iter_mut().zip(di) {
                *slot += 2.0 * self.l[i] * term;
            }
        }
        (a, b, c)
    }
}

/// Which root of a ruling's quadratic an arc follows.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Label {
    /// `(−b + √D) / 2a`.
    Plus,
    /// `(−b − √D) / 2a`.
    Minus,
}

/// A singular point's correction to the discriminant: `value` at `at`,
/// falling to zero over `reach` by a smoothstep.
#[derive(Debug, Clone, Copy)]
struct Bump {
    at: f64,
    value: f64,
    reach: f64,
}

/// The walked family against the other quadric, in the other's frame.
#[derive(Debug)]
struct Pencil {
    frame: Frame,
    p: TrigVec,
    d: TrigVec,
    quadric: Quadric,
    disc: Trig2,
    bumps: Vec<Bump>,
}

impl Pencil {
    /// `a`, `b`, `c` at `s`, with the ruling's base point and direction.
    fn at(&self, s: f64) -> (f64, f64, f64, Vec3, Vec3) {
        let (p, d) = (self.p.eval(s), self.d.eval(s));
        let q = &self.quadric;
        let a = q.m.dot(&d.component_mul(&d));
        let b = 2.0 * (q.m.dot(&d.component_mul(&p)) + q.l.dot(&d));
        (a, b, q.value(p), p, d)
    }

    fn correction(&self, s: f64) -> f64 {
        self.bumps
            .iter()
            .map(|bump| {
                let x = (wrap_angle(s - bump.at + PI) - PI).abs() / bump.reach;
                if x < 1.0 {
                    bump.value * (1.0 - x * x * (3.0 - 2.0 * x))
                } else {
                    0.0
                }
            })
            .sum()
    }

    /// The corrected discriminant, never negative: at `s`, or — the
    /// accurate form beside a turning point — at `anchor + 2h`, where the
    /// discriminant is zero at the anchor by the anchor's definition.
    fn discriminant(&self, s: f64, anchor: Option<(f64, f64)>) -> f64 {
        let raw = match anchor {
            Some((at, h)) => self.disc.difference(at, h),
            None => self.disc.eval(s),
        };
        (raw - self.correction(s)).max(0.0)
    }

    /// The root `label` names on the ruling at `s`, in whichever of its
    /// two forms rounds less. `(−b ± √D) / 2a` loses the digits of `b`
    /// and `√D` when they cancel, an error of about `ε(|b| + √D) / |a|`;
    /// `2c / (−b ∓ √D)` never cancels but carries `c`'s own rounding,
    /// `ε·C` for `C` the magnitude `c` is summed from, over `|b| + √D`.
    /// The second is taken where it is the smaller — where `a → 0` sends
    /// the other root to infinity — and the first everywhere else: at a
    /// turning point whose root is the ruling's base point, `b`, `√D`
    /// and `c` all vanish and `c`'s rounding over them is anything.
    /// Inside a correction's reach, where `4ac` is no longer `b² − D`, it
    /// is always the midpoint plus or minus the half-width.
    fn root(&self, s: f64, anchor: Option<(f64, f64)>, label: Label) -> (f64, Vec3, Vec3) {
        let (a, b, c, p, d) = self.at(s);
        let r = self.discriminant(s, anchor).sqrt();
        let corrected = self.correction(s) != 0.0;
        let sum = b.abs() + r;
        let scale = self.quadric.abs().value(p.abs());
        let direct = corrected || sum * sum <= 4.0 * a.abs() * scale;
        let w = match label {
            Label::Plus if b <= 0.0 || direct => (-b + r) / (2.0 * a),
            Label::Plus => 2.0 * c / (-b - r),
            Label::Minus if b >= 0.0 || direct => (-b - r) / (2.0 * a),
            Label::Minus => 2.0 * c / (-b + r),
        };
        (w, p, d)
    }

    fn point(&self, s: f64, anchor: Option<(f64, f64)>, label: Label) -> Point3 {
        let (w, p, d) = self.root(s, anchor, label);
        self.frame.to_world(Point3::from(p + d * w))
    }

    /// The point at `s` with its stretch: the segment of the ruling,
    /// centred on the root `label` names, on which the other quadric's
    /// value vanishes in `f64` — `POLYNOMIAL_ROUNDING` of the magnitude it
    /// is summed from at the root, and what it changes by over the last
    /// digit of `s`, together `M`. Every point of it lies on the
    /// walked surface exactly and on the other to its rounding, so `f64`
    /// holds no reason to prefer the root over any of them: where the
    /// ruling runs a hair from tangent to the other surface, the root
    /// moves along it by as much from one `s` to the next, though the
    /// point stays on both surfaces. Its half-length is
    /// `POLYNOMIAL_ROUNDING · M / |f′(w)|` for the ruling's quadratic `f`,
    /// and no more than `√(POLYNOMIAL_ROUNDING · M / |a|)`, where `f′`
    /// vanishes: over it `|f′λ + aλ²|` stays within twice the budget.
    fn located(&self, s: f64, anchor: Option<(f64, f64)>, label: Label) -> (Point3, Stretch) {
        let (w, p, d) = self.root(s, anchor, label);
        let x = p + d * w;
        let point = self.frame.to_world(Point3::from(x));
        let a = self.quadric.m.dot(&d.component_mul(&d));
        let gradient = self.quadric.gradient(x);
        let slope = gradient.dot(&d).abs();
        // `s` is itself a float: the root moves along the ruling by as
        // much as the value changes over its last digit.
        let turn = self.p.slope(s) + self.d.slope(s) * w;
        let budget = POLYNOMIAL_ROUNDING * self.quadric.abs().value(x.abs())
            + gradient.dot(&turn).abs() * s.abs() * f64::EPSILON;
        let half = (budget / slope).min((budget / a.abs()).sqrt());
        let stretch = if half.is_finite() && half > 0.0 {
            Stretch::Segment(self.frame.vec_to_world(d * half))
        } else {
            Stretch::Point
        };
        (point, stretch)
    }
}

/// How far along the section `f64` leaves a branch's point undecided:
/// the stretch of the line or circle its root was found along on which
/// the other surface does not tell one point from another
/// ([`SectionBranch::distance`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Stretch {
    /// The point alone.
    Point,
    /// The point plus or minus this vector, along a ruling.
    Segment(Vec3),
    /// The arc of the circle about `centre`, in the plane normal to the
    /// unit `normal`, through the point and `half` a radian either side
    /// of it.
    Arc {
        centre: Point3,
        normal: Vec3,
        half: f64,
    },
}

impl Stretch {
    /// The distance from `q` to the stretch through `p`.
    pub(crate) fn distance(&self, p: Point3, q: Point3) -> f64 {
        match *self {
            Stretch::Point => (q - p).norm(),
            Stretch::Segment(half) => {
                let r = q - p;
                let along = (r.dot(&half) / half.norm_squared()).clamp(-1.0, 1.0);
                (r - half * along).norm()
            }
            Stretch::Arc {
                centre,
                normal,
                half,
            } => {
                let radial = p - centre;
                let radius = radial.norm();
                if radius == 0.0 {
                    return (q - p).norm();
                }
                let e1 = radial / radius;
                let e2 = normal.cross(&e1);
                let r = q - centre;
                let angle = real::atan2(r.dot(&e2), r.dot(&e1)).clamp(-half, half);
                let (sin, cos) = real::sin_cos(angle);
                (q - (centre + (e1 * cos + e2 * sin) * radius)).norm()
            }
        }
    }
}

/// One arc of a branch: one root over an interval of the walked angle —
/// a ruled family's, or a torus's `u` — from `start` to `finish`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Arc1 {
    start: f64,
    finish: f64,
    /// The section turns back at this end: the square root vanishes here
    /// and the parametrisation removes it.
    turn_start: bool,
    turn_finish: bool,
    /// Where the arc starts in the branch's parameter, and how long it is.
    t0: f64,
    len: f64,
}

impl Arc1 {
    /// An arc yet to be laid into a branch ([`Arc1::lay`]).
    pub(crate) fn new(start: f64, finish: f64, turn_start: bool, turn_finish: bool) -> Arc1 {
        Arc1 {
            start,
            finish,
            turn_start,
            turn_finish,
            t0: 0.0,
            len: 0.0,
        }
    }

    /// The arcs laid end to end in the branch's parameter, and the
    /// branch's length.
    pub(crate) fn lay(arcs: &mut [Arc1]) -> f64 {
        let mut t = 0.0;
        for arc in arcs {
            arc.t0 = t;
            arc.len = Arc1::length(arc.start, arc.finish, arc.turn_start, arc.turn_finish);
            t += arc.len;
        }
        t
    }

    /// The parameter length of an arc: an arc that turns runs at
    /// `θ = t / √L`, so that `s − s_T = t²/2` to leading order on both
    /// sides of a turn whatever the two arcs' lengths, and the branch is
    /// smooth through it.
    fn length(start: f64, finish: f64, turn_start: bool, turn_finish: bool) -> f64 {
        let l = (finish - start).abs();
        match (turn_start, turn_finish) {
            (true, true) => PI * (0.5 * l).sqrt(),
            (true, false) | (false, true) => 0.5 * PI * l.sqrt(),
            (false, false) => l,
        }
    }

    /// The family angle at `x ∈ [0, len]`, and the turning point it is
    /// measured from with half the offset, when there is one.
    pub(crate) fn angle(&self, x: f64) -> (f64, Option<(f64, f64)>) {
        let sign = (self.finish - self.start).signum();
        let l = (self.finish - self.start).abs();
        let from_start = |off: f64| {
            (
                self.start + sign * off,
                Some((self.start, 0.5 * sign * off)),
            )
        };
        let from_finish = |off: f64| {
            (
                self.finish - sign * off,
                Some((self.finish, -0.5 * sign * off)),
            )
        };
        match (self.turn_start, self.turn_finish) {
            (true, true) => {
                let half = 0.5 * l;
                let (sin, cos) = real::sin_cos(0.5 * x / half.sqrt());
                if sin <= cos {
                    from_start(2.0 * half * sin * sin)
                } else {
                    from_finish(2.0 * half * cos * cos)
                }
            }
            (true, false) => {
                let sin = real::sin(0.5 * x / l.sqrt());
                from_start(2.0 * l * sin * sin)
            }
            (false, true) => {
                let sin = real::sin(0.5 * (self.len - x) / l.sqrt());
                from_finish(2.0 * l * sin * sin)
            }
            (false, false) => (self.start + sign * x, None),
        }
    }
}

/// How an open branch ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BranchEnd {
    /// At a singular point of the section, an index into
    /// [`SectionTrace::points`]: the branch's end *is* that point.
    Singular(usize),
    /// Where the branch leaves the region's extent along the walked
    /// rulings, or the domain of a walked NURBS surface: there on the
    /// edge line exactly.
    Clipped,
}

/// What a branch is walked on: the rulings of a quadric
/// ([`trace_quadrics`]), or the parameter plane of a torus
/// ([`crate::trace_torus`]) or of a NURBS surface (ADR-0061).
#[derive(Debug, Clone)]
enum Walk {
    /// Which root of its ruling's quadratic each arc follows.
    Ruled {
        pencil: Arc<Pencil>,
        labels: Vec<Label>,
    },
    Torus(TorusWalk),
    Marched(MarchedWalk),
}

/// How far apart a corrected sample's two surface points may be, as a
/// share of `tol.linear`: `crate::march`'s `ON_SHARE`.
const MARCHED_ON_SHARE: f64 = 1.0 / 1024.0;

/// A walked section of two NURBS surfaces in their parameter planes.
#[derive(Debug, Clone)]
pub(crate) struct MarchedWalk {
    pub(crate) surfaces: [Arc<NurbsSurface>; 2],
    pub(crate) points: Vec<[f64; 4]>,
    pub(crate) cumulative: Vec<f64>,
    pub(crate) length: f64,
    /// How far apart the two surface points of a corrected sample may be.
    on: f64,
}

impl MarchedWalk {
    pub(crate) fn new(
        surfaces: [Arc<NurbsSurface>; 2],
        mut points: Vec<[f64; 4]>,
        closed: bool,
        tol: Tolerance,
    ) -> Self {
        if let (true, Some(&last)) = (closed, points.last()) {
            let closure = [
                surfaces[0].closure()[0],
                surfaces[0].closure()[1],
                surfaces[1].closure()[0],
                surfaces[1].closure()[1],
            ];
            let closing = near_closure(points[0], last, &closure);
            points.push(closing);
        }
        let mut cumulative = Vec::with_capacity(points.len());
        if !points.is_empty() {
            cumulative.push(0.0);
            for k in 1..points.len() {
                let p0 = surfaces[0].eval(points[k - 1][0], points[k - 1][1]).point;
                let p1 = surfaces[0].eval(points[k][0], points[k][1]).point;
                let d = (p1 - p0).norm();
                cumulative.push(cumulative[k - 1] + d);
            }
        } else {
            cumulative.push(0.0);
        }
        let length = *cumulative.last().unwrap_or(&0.0);
        let length = if length <= 0.0 { 1.0 } else { length };
        MarchedWalk {
            surfaces,
            points,
            cumulative,
            length,
            on: tol.linear * MARCHED_ON_SHARE,
        }
    }

    fn sample(&self, t: f64) -> [f64; 4] {
        if self.points.is_empty() {
            return [0.0; 4];
        }
        if self.points.len() == 1 {
            return self.points[0];
        }
        let t = t.clamp(0.0, self.length);
        let i = self
            .cumulative
            .partition_point(|&c| c <= t)
            .saturating_sub(1);
        let i = i.min(self.points.len() - 2);
        let (c0, c1) = (self.cumulative[i], self.cumulative[i + 1]);
        let s = if c1 > c0 { (t - c0) / (c1 - c0) } else { 0.0 }.clamp(0.0, 1.0);
        let (p0, p1) = (self.points[i], self.points[i + 1]);
        if s <= 0.0 {
            return p0;
        }
        if s >= 1.0 {
            return p1;
        }
        let guess = [
            (1.0 - s) * p0[0] + s * p1[0],
            (1.0 - s) * p0[1] + s * p1[1],
            (1.0 - s) * p0[2] + s * p1[2],
            (1.0 - s) * p0[3] + s * p1[3],
        ];
        let p0_pt = self.surfaces[0].eval(p0[0], p0[1]).point;
        let p1_pt = self.surfaces[0].eval(p1[0], p1[1]).point;
        let chord = p1_pt - p0_pt;
        let Some(dir) = chord.try_normalize(0.0) else {
            return guess;
        };
        let p_chord = p0_pt + chord * s;
        self.correct(guess, p_chord, dir).unwrap_or(guess)
    }

    /// The sample Newton-corrected onto both surfaces and the plane
    /// through `base` normal to `normal`; `None` where it does not
    /// converge within `self.on`, and the caller keeps the interpolation.
    fn correct(&self, mut x: [f64; 4], base: Point3, normal: Vec3) -> Option<[f64; 4]> {
        for _ in 0..8 {
            let e0 = self.surfaces[0].eval(x[0], x[1]);
            let e1 = self.surfaces[1].eval(x[2], x[3]);
            let gap = e0.point - e1.point;
            let plane = normal.dot(&(e0.point - base));
            if gap.norm() <= self.on && plane.abs() <= self.on {
                return Some(x);
            }
            let j = Matrix4::new(
                e0.du.x,
                e0.dv.x,
                -e1.du.x,
                -e1.dv.x,
                e0.du.y,
                e0.dv.y,
                -e1.du.y,
                -e1.dv.y,
                e0.du.z,
                e0.dv.z,
                -e1.du.z,
                -e1.dv.z,
                normal.dot(&e0.du),
                normal.dot(&e0.dv),
                0.0,
                0.0,
            );
            let step = j.lu().solve(&Vector4::new(gap.x, gap.y, gap.z, plane))?;
            if !step.iter().all(|c| c.is_finite()) {
                return None;
            }
            for k in 0..4 {
                x[k] -= step[k];
            }
        }
        None
    }

    pub(crate) fn point(&self, t: f64) -> Point3 {
        let x = self.sample(t);
        self.surfaces[0].eval(x[0], x[1]).point
    }

    pub(crate) fn uv(&self, surface: usize, t: f64) -> Point2 {
        let x = self.sample(t);
        if surface == 0 {
            Point2::new(x[0], x[1])
        } else {
            Point2::new(x[2], x[3])
        }
    }
}

fn near_closure(mut at: [f64; 4], to: [f64; 4], closure: &[Option<f64>; 4]) -> [f64; 4] {
    for k in 0..4 {
        if let Some(c) = closure[k] {
            at[k] += c * ((to[k] - at[k]) / c).round();
        }
    }
    at
}

/// One branch of a section: a smooth curve lying on the walked surface to
/// rounding and within the trace's tolerance of the other — to rounding
/// too, away from a singular point's reach.
///
/// Guarantees: `point` is defined, finite and continuous over `domain`,
/// with a non-vanishing derivative inside it; a closed branch is periodic
/// over its domain's length; an open branch ending
/// [`BranchEnd::Singular`] evaluates to that singular point at that end.
#[derive(Debug, Clone)]
pub struct SectionBranch {
    walk: Walk,
    arcs: Vec<Arc1>,
    ends: Option<[BranchEnd; 2]>,
    /// The singular points the ends are, so that an end evaluates to its
    /// point exactly and not to the arc's angle summed back to it.
    pins: [Option<Point3>; 2],
    length: f64,
}

impl SectionBranch {
    /// A branch of a torus section from its arcs, one of `walk`'s to
    /// each; `pins` are the singular points the ends are.
    pub(crate) fn on_torus(
        walk: TorusWalk,
        mut arcs: Vec<Arc1>,
        ends: Option<[BranchEnd; 2]>,
        pins: [Option<Point3>; 2],
    ) -> SectionBranch {
        let length = Arc1::lay(&mut arcs);
        SectionBranch {
            walk: Walk::Torus(walk),
            arcs,
            ends,
            pins,
            length,
        }
    }

    /// A branch of two NURBS surfaces walked between seeds.
    pub(crate) fn on_marched(
        walk: MarchedWalk,
        ends: Option<[BranchEnd; 2]>,
        pins: [Option<Point3>; 2],
    ) -> SectionBranch {
        let length = walk.length;
        SectionBranch {
            walk: Walk::Marched(walk),
            arcs: Vec::new(),
            ends,
            pins,
            length,
        }
    }

    /// `[0, length]`. The parameter is the tracer's own: the walked
    /// angle — a ruled family's, a torus's `u` — along an arc that does
    /// not turn, and a parameter in which the section is smooth through a
    /// turning point elsewhere.
    pub fn domain(&self) -> Interval {
        Interval::new(0.0, self.length).unwrap_or(Interval::UNIT)
    }

    /// `true` for a loop; its domain's length is then its period.
    pub fn is_closed(&self) -> bool {
        self.ends.is_none()
    }

    /// How the branch ends at its domain's start and end; `None` for a
    /// closed branch.
    pub fn ends(&self) -> Option<[BranchEnd; 2]> {
        self.ends
    }

    /// The point at `t`, wrapped into the domain on a closed branch and
    /// clamped to it on an open one.
    pub fn point(&self, t: f64) -> Point3 {
        let t = self.inside(t);
        match self.pins {
            [Some(start), _] if t <= 0.0 => return start,
            [_, Some(end)] if t >= self.length => return end,
            _ => {}
        }
        let (i, s, anchor) = self.located(t);
        match &self.walk {
            Walk::Ruled { pencil, labels } => match labels.get(i) {
                Some(&label) => pencil.point(s, anchor, label),
                None => pencil.frame.origin(),
            },
            Walk::Torus(walk) => walk.point(i, s, anchor),
            Walk::Marched(walk) => walk.point(t),
        }
    }

    /// How far `q` is from the branch at `t`, as precisely as `f64` knows
    /// the branch there: the distance from `q` to the stretch of the
    /// walked ruling through `point(t)` on which the other surface's
    /// implicit value vanishes in `f64` — a segment on the walked surface
    /// and on the other to rounding, of the root's own rounding along
    /// the ruling. It is `|q − point(t)|` less a length at rounding's
    /// scale, except where the ruling runs within a hair of tangent to
    /// the other surface: there the root is known along the ruling only
    /// to that stretch, and the section with it — two rods a quarter of a
    /// tolerance off parallel, whose section runs along the rulings, have
    /// branches whose points step back and forth along it by `10⁻⁷`
    /// between neighbouring parameters. A torus branch's point is a root
    /// along a tube circle, or along a parallel beside a turning point,
    /// and its stretch is the arc of that circle on which the other
    /// surface's distance stays within its rounding of the point's.
    ///
    /// ```
    /// use arris_geom::{Surface, trace_quadrics};
    /// use arris_math::{Aabb, Frame, Point3, Precision, Vec3};
    ///
    /// // Two pipes crossing square: the branch is known to rounding.
    /// let main = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let across = Frame::from_z(Point3::origin(), Vec3::x()).unwrap();
    /// let pipe = Surface::Cylinder { frame: across, radius: 1.0 };
    /// let within = Aabb { min: [-5.0; 3], max: [5.0; 3] };
    /// let tol = Precision::DEFAULT.tolerance();
    /// let trace = trace_quadrics(&main, &pipe, &within, tol, &mut arris_math::Meter::default()).unwrap();
    /// let branch = &trace.branches()[0];
    /// let (t, q) = (1.0, Point3::new(0.0, 0.0, 3.0));
    /// assert_eq!(branch.distance(t, branch.point(t)), 0.0);
    /// assert!((branch.distance(t, q) - (q - branch.point(t)).norm()).abs() < 1e-12);
    /// ```
    pub fn distance(&self, t: f64, q: Point3) -> f64 {
        let t = self.inside(t);
        match self.pins {
            [Some(start), _] if t <= 0.0 => return (q - start).norm(),
            [_, Some(end)] if t >= self.length => return (q - end).norm(),
            _ => {}
        }
        let (i, s, anchor) = self.located(t);
        let (p, stretch) = match &self.walk {
            Walk::Ruled { pencil, labels } => match labels.get(i) {
                Some(&label) => pencil.located(s, anchor, label),
                None => (pencil.frame.origin(), Stretch::Point),
            },
            Walk::Torus(walk) => walk.located(i, s, anchor),
            Walk::Marched(walk) => (walk.point(t), Stretch::Point),
        };
        stretch.distance(p, q)
    }

    /// The walked surface's own `(u, v)` of `point(t)`, exact as the
    /// point is, for a branch of [`crate::trace_torus`] or of a NURBS
    /// surface's section (the NURBS surface's parameters); `None` for a
    /// branch walked on rulings. Unwrapped across the seams of the
    /// directions that close, so it is continuous along the branch and
    /// may leave the domain there; a closed branch that winds round comes
    /// back a whole number of closures from where it started.
    pub fn uv(&self, t: f64) -> Option<Point2> {
        self.uv_on(0, t)
    }

    /// The second surface's own `(u, v)` of `point(t)`, for a branch of
    /// a marched section between two NURBS surfaces; `None` otherwise.
    pub fn uv2(&self, t: f64) -> Option<Point2> {
        self.uv_on(1, t)
    }

    /// The surface's own `(u, v)` of `point(t)` on operand `surface`
    /// (0 or 1), exact as the point is, for a branch of
    /// [`crate::trace_torus`] (surface 0, the torus), of a NURBS section
    /// with an analytic surface (surface 0, the NURBS surface) or a
    /// marched section between two NURBS surfaces (surfaces 0 and 1);
    /// `None` for a branch walked on rulings or outside the operands.
    pub fn uv_on(&self, surface: usize, t: f64) -> Option<Point2> {
        let wrapped = self.inside(t);
        match &self.walk {
            Walk::Torus(walk) if surface == 0 => {
                let t = wrapped;
                let (i, s, anchor) = self.located(t);
                Some(walk.uv(i, s, anchor, [t <= 0.0, t >= self.length]))
            }
            // The end of a loop is its start a closure on: the parameters
            // continue to it rather than wrap back.
            Walk::Marched(walk) if surface < 2 => {
                let end = self.is_closed() && t == self.length;
                Some(walk.uv(surface, if end { t } else { wrapped }))
            }
            _ => None,
        }
    }

    /// The `NurbsSurface` operand `surface` (0 or 1), if the branch was
    /// marched.
    pub(crate) fn surface(&self, surface: usize) -> Option<Arc<NurbsSurface>> {
        match &self.walk {
            Walk::Marched(walk) if surface < 2 => Some(walk.surfaces[surface].clone()),
            _ => None,
        }
    }

    /// `t` wrapped into the domain on a closed branch and clamped to it
    /// on an open one.
    fn inside(&self, t: f64) -> f64 {
        if self.is_closed() {
            t.rem_euclid(self.length)
        } else {
            t.clamp(0.0, self.length)
        }
    }

    /// The arc `t` falls on and the walked angle there, with the turning
    /// point it is measured from.
    fn located(&self, t: f64) -> (usize, f64, Option<(f64, f64)>) {
        let i = self
            .arcs
            .partition_point(|arc| arc.t0 <= t)
            .saturating_sub(1);
        let Some(arc) = self.arcs.get(i) else {
            return (i, 0.0, None);
        };
        let (s, anchor) = arc.angle((t - arc.t0).clamp(0.0, arc.len));
        (i, s, anchor)
    }
}

/// A point of the section where the surfaces are tangent within the
/// tolerance.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SectionPoint {
    /// Where: on the walked surface to rounding, within the tolerance of
    /// the other.
    pub point: Point3,
    /// `true` when no branch ends here: the surfaces touch at the point
    /// and meet nowhere near it. Otherwise branches cross here, and each
    /// ends at it.
    pub isolated: bool,
}

/// A tube circle of the walked torus, or an isoline of a walked NURBS
/// surface, that lies on the other surface within the tolerance: a curve
/// of the section in closed form, exact and never fitted.
#[derive(Debug, Clone, PartialEq)]
pub struct SectionCircle {
    /// Of a torus, a [`Curve::Circle`] on the torus exactly and within
    /// the trace's tolerance of the other surface, its parameter the
    /// torus's `v` and its frame's origin on the torus's centre circle.
    /// Of a NURBS surface, the surface's own line of constant `u` or `v`
    /// as a [`Curve::Nurbs`] — the net summed at that parameter, no fit —
    /// its parameter the surface's other one, within the tolerance of
    /// the other surface: the whole line, or, where the other surface
    /// touches an edge of the domain without entering it and rounding
    /// cannot sign the section along a stretch of that edge, the edge's
    /// line over the stretch, `tangent`.
    pub circle: Curve,
    /// `true` where the surfaces are tangent along the circle and do not
    /// cross there — a pipe elbow against its straight pipe, a sphere of
    /// the tube's radius inside it, a plane along a cylinder's ruling;
    /// `false` where they cross along it.
    pub tangent: bool,
}

/// A traced section: its branches, its singular points, and the conics a
/// tracer answers in closed form.
#[derive(Debug, Clone)]
pub struct SectionTrace {
    branches: Vec<SectionBranch>,
    points: Vec<SectionPoint>,
    circles: Vec<SectionCircle>,
}

impl SectionTrace {
    pub(crate) fn new(branches: Vec<SectionBranch>, points: Vec<SectionPoint>) -> SectionTrace {
        SectionTrace {
            branches,
            points,
            circles: Vec::new(),
        }
    }

    /// The trace with the tube circles of the walked torus that lie on
    /// the other surface.
    pub(crate) fn with_circles(mut self, circles: Vec<SectionCircle>) -> SectionTrace {
        self.circles = circles;
        self
    }

    /// The trace with `more` isolines of a walked NURBS surface after its
    /// own: poles of the other direction, each one point
    /// (`torus_walk::Tracer::run_beside_poles`).
    pub(crate) fn with_more_circles(mut self, more: Vec<SectionCircle>) -> SectionTrace {
        self.circles.extend(more);
        self
    }

    /// The tube circles of the walked torus that lie on the other
    /// surface, ascending by the torus's `u`, or the isolines of a walked
    /// NURBS surface, ascending by the parameter each is constant in;
    /// none from [`trace_quadrics`]. A branch that reaches one ends on
    /// it, at a singular point of [`Self::points`] that is not isolated;
    /// the circle itself runs through that point whole.
    pub fn circles(&self) -> &[SectionCircle] {
        &self.circles
    }

    /// The branches, in an order and with orientations that depend on the
    /// two surfaces alone — never on which was passed first.
    pub fn branches(&self) -> &[SectionBranch] {
        &self.branches
    }

    /// The singular points, ascending by the angle of the walked ruling
    /// through each; of a torus section, by the walked torus's `u` and
    /// then its `v`.
    pub fn points(&self) -> &[SectionPoint] {
        &self.points
    }
}

/// The section of two quadrics, one of them ruled, inside `within`.
///
/// Operands: a cylinder, an elliptic cylinder or a cone against any of
/// those or a sphere, in any pose; every other pair is
/// [`GeomError::Unsupported`]. The poses a closed form owns — parallel
/// cylinders, surfaces of revolution on one axis — are not the tracer's:
/// it answers the ones it can and refuses the rest as
/// [`GeomError::DegenerateSection`].
///
/// Guarantees: every branch lies on both surfaces, to rounding on the
/// walked one and within `tol.linear` of the other; two surfaces that
/// come within `tol.linear` of tangency at a point meet there in a
/// [`SectionPoint`], and branches through it end at it exactly; every
/// part of the section inside `within` is on a branch, because the
/// branch structure comes from the roots of quartics and not from
/// samples; a branch is clipped only where it leaves the extent of
/// `within` along the walked rulings, so a loop inside it is returned
/// closed. Which operand is walked is a rule on the two surfaces — a
/// cylinder before a cone, the smaller before the larger — so swapping
/// the arguments changes nothing, bit for bit.
///
/// ```
/// use arris_geom::{Surface, trace_quadrics};
/// use arris_math::{Aabb, Frame, Point3, Precision, Vec3};
///
/// // A pipe of radius 1 through a pipe of radius 2: two loops.
/// let main = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
/// let across = Frame::from_z(Point3::origin(), Vec3::x()).unwrap();
/// let branch = Surface::Cylinder { frame: across, radius: 1.0 };
/// let within = Aabb { min: [-5.0; 3], max: [5.0; 3] };
/// let trace = trace_quadrics(&main, &branch, &within, Precision::DEFAULT.tolerance(), &mut arris_math::Meter::default()).unwrap();
/// assert_eq!(trace.branches().len(), 2);
/// assert!(trace.branches().iter().all(|b| b.is_closed()));
/// let p = trace.branches()[0].point(1.0);
/// assert!((p.x.hypot(p.y) - 2.0).abs() < 1e-12 && (p.y.hypot(p.z) - 1.0).abs() < 1e-12);
/// ```
pub fn trace_quadrics(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SectionTrace, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    let (walked, other) = walked_first(a, b)?;
    let kinds = (
        GeomKind::Surface(walked.kind()),
        GeomKind::Surface(other.kind()),
    );
    let fault = |fault| GeomError::DegenerateSection {
        a: kinds.0,
        b: kinds.1,
        fault,
    };
    let (Some((p, d)), Some((quadric, frame))) = (rulings(walked), quadric_of(other)) else {
        return Err(GeomError::Unsupported {
            a: kinds.0,
            b: kinds.1,
        });
    };
    // The rulings in the other surface's frame, where its form is diagonal
    // and every magnitude is the pair's own and not its distance from the
    // world's origin.
    let local = |v: Vec3| frame.vec_to_local(v);
    let p = TrigVec {
        k: frame.to_local(Point3::from(p.k)).coords,
        c: local(p.c),
        s: local(p.s),
    };
    let d = TrigVec {
        k: local(d.k),
        c: local(d.c),
        s: local(d.s),
    };
    let (qa, qb, qc) = quadric.against(&p, &d);
    // A family has a constant direction or a constant base point, so one
    // of `a`, `c` is a constant and `4ac` is of degree two.
    let four_ac = if d.c == Vec3::zeros() && d.s == Vec3::zeros() {
        qc.scaled(4.0 * qa.k)
    } else {
        qa.scaled(4.0 * qc.k)
    };
    let disc = Trig2::product(qb, qb).plus(four_ac.scaled(-1.0));
    // The same construction over magnitudes bounds what rounding can do
    // to the discriminant.
    let (ma, mb, mc) = quadric.abs().against(&p.abs(), &d.abs());
    let mb = mb.iter().map(|x| x.abs()).sum::<f64>();
    let noise = POLYNOMIAL_ROUNDING * (mb * mb + 4.0 * ma.sum_abs() * mc.sum_abs());
    if !noise.is_finite() {
        return Err(GeomError::Degenerate {
            kind: kinds.0,
            reason: "a non-finite frame or radius".into(),
        });
    }

    let mut pencil = Pencil {
        frame,
        p,
        d,
        quadric,
        disc,
        bumps: Vec::new(),
    };

    // The critical points of the discriminant: between two of them it is
    // monotone and has at most one root, and one where the surfaces are
    // within the tolerance of tangency is a singular point.
    let slope = disc.derivative();
    let critical = slope.roots().unwrap_or_default();
    if critical.is_empty() && disc.k.abs() <= noise {
        return Err(fault(SectionFault::TangentAlongCurve));
    }
    let curvature = slope.derivative();
    let mut singular: Vec<Singular> = Vec::new();
    let mut is_singular = vec![false; critical.len()];
    for (i, &s) in critical.iter().enumerate() {
        meter.tick()?;
        let value = disc.eval(s);
        let (a, b, c, p, d) = pencil.at(s);
        // A ruling whose quadratic term is rounding — parallel to a
        // generator of the other surface — has its double root at
        // infinity, not at the quotient of two roundings: no point of
        // the clip, and no distance to measure the tangency by.
        let mid = if a.abs() <= POLYNOMIAL_ROUNDING * ma.sum_abs() {
            f64::INFINITY
        } else {
            -b / (2.0 * a)
        };
        let gradient = if mid.is_finite() {
            pencil.quadric.gradient(p + d * mid).norm()
        } else {
            f64::NAN
        };
        // How far the other surface has to move for the ruling to touch
        // it: the discriminant's value as a distance.
        let offset = value.abs() / (4.0 * a.abs() * gradient);
        let within_tol = offset.is_finite() && offset <= tol.linear;
        if !(value.abs() <= noise || within_tol) {
            continue;
        }
        if a.abs() <= POLYNOMIAL_ROUNDING * ma.sum_abs()
            && b.abs() <= POLYNOMIAL_ROUNDING * mb
            && c.abs() <= POLYNOMIAL_ROUNDING * mc.sum_abs()
        {
            return Err(fault(SectionFault::SharedRuling));
        }
        is_singular[i] = true;
        // The value is taken out by a bump, so that the two roots meet at
        // the singular point exactly — also a value that is only
        // rounding, whose bump reaches no further than rounding does.
        let mut reach = 0.0;
        if value != 0.0 {
            let kappa = 0.5 * curvature.eval(s).abs();
            if kappa <= POLYNOMIAL_ROUNDING * disc.sum_abs() {
                return Err(fault(SectionFault::CrowdedSingularity));
            }
            reach = REACH * (value.abs() / kappa).sqrt();
            pencil.bumps.push(Bump {
                at: s,
                value,
                reach,
            });
        }
        singular.push(Singular {
            at: s,
            reach,
            mid,
            point: None,
        });
    }
    let n = critical.len();
    for i in 0..n {
        if n > 1 && is_singular[i] && is_singular[(i + 1) % n] {
            return Err(fault(SectionFault::TangentAlongCurve));
        }
    }

    // The turning points: the one root between two critical points of
    // opposite sign, neither of them singular.
    let mut turning: Vec<f64> = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        if is_singular[i] || is_singular[j] || (n == 1) {
            continue;
        }
        let (lo, hi) = (
            critical[i],
            if j > i {
                critical[j]
            } else {
                critical[j] + TAU
            },
        );
        if disc.eval(lo).signum() == disc.eval(hi).signum() {
            continue;
        }
        let Ok(bracket) = Interval::new(lo, hi) else {
            continue;
        };
        if let Ok(root) = newton_in_interval(|s| disc.eval(s), |s| slope.eval(s), bracket, 0.0) {
            turning.push(wrap_angle(root));
        }
    }
    let apart = |x: f64, y: f64| (wrap_angle(x - y + PI) - PI).abs();
    for (i, s) in singular.iter().enumerate() {
        let crowded = turning.iter().any(|&t| apart(t, s.at) <= s.reach)
            || singular
                .iter()
                .skip(i + 1)
                .any(|o| apart(o.at, s.at) <= s.reach + o.reach);
        if crowded {
            return Err(fault(SectionFault::CrowdedSingularity));
        }
    }

    // The clip: the region's extent along the rulings, and where a root
    // crosses either end of it.
    let (w_lo, w_hi) = extent(&pencil, within);
    let b2 = Trig2::linear(qb);
    let mut breaks: Vec<Break> = Vec::new();
    breaks.extend(turning.iter().map(|&at| Break {
        at,
        kind: BreakKind::Turning,
    }));
    breaks.extend(singular.iter().enumerate().map(|(i, s)| Break {
        at: s.at,
        kind: BreakKind::Singular(i),
    }));
    for w in [w_lo, w_hi] {
        let g = qa.scaled(w * w).plus(b2.scaled(w)).plus(qc);
        breaks.extend(g.roots().unwrap_or_default().into_iter().map(|at| Break {
            at,
            kind: BreakKind::Clip,
        }));
    }
    breaks.sort_by(|x, y| {
        x.at.total_cmp(&y.at)
            .then(x.kind.rank().cmp(&y.kind.rank()))
    });
    breaks.dedup_by(|later, first| later.at == first.at);

    let pencil = Arc::new(pencil);
    let in_range = |w: f64| w.is_finite() && (w_lo..=w_hi).contains(&w);
    for s in &mut singular {
        if in_range(s.mid) {
            s.point = Some(pencil.point(s.at, None, Label::Plus));
        }
    }

    meter.tick()?;
    let chains = chains(&pencil, &breaks, in_range);
    Ok(assemble(&pencil, chains, &singular, tol))
}

/// A critical point of the discriminant decided singular.
#[derive(Debug, Clone, Copy)]
struct Singular {
    at: f64,
    reach: f64,
    /// The root on its ruling, where the two roots meet.
    mid: f64,
    /// The point, when it is inside the clip.
    point: Option<Point3>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum BreakKind {
    Singular(usize),
    Turning,
    Clip,
}

impl BreakKind {
    fn rank(&self) -> usize {
        match self {
            BreakKind::Singular(_) => 0,
            BreakKind::Turning => 1,
            BreakKind::Clip => 2,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Break {
    at: f64,
    kind: BreakKind,
}

/// What an end of a piece — one root over one interval between
/// breakpoints — is joined to.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Link {
    /// The end of another piece, `(piece, side)`; `turning` when the
    /// section turns back there, the other piece being the other root of
    /// the same interval.
    To {
        piece: usize,
        side: usize,
        turning: bool,
    },
    End(EndKind),
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum EndKind {
    Singular(usize),
    Clipped,
}

/// A run of pieces walked end to end: `(piece, forward, turns_after)`.
struct Chain {
    steps: Vec<(usize, bool, bool)>,
    ends: Option<[EndKind; 2]>,
    /// The intervals' bounds, for the arcs.
    bounds: Vec<(f64, f64)>,
}

const LABELS: [Label; 2] = [Label::Plus, Label::Minus];

/// The live pieces linked at the breakpoints and walked into chains, in
/// an order the breakpoints alone decide: open chains from their first
/// end in piece order, then the loops.
fn chains(pencil: &Pencil, breaks: &[Break], in_range: impl Fn(f64) -> bool) -> Vec<Chain> {
    let n = breaks.len();
    let bounds: Vec<(f64, f64)> = if n == 0 {
        vec![(0.0, TAU)]
    } else {
        (0..n)
            .map(|i| {
                let hi = if i + 1 < n {
                    breaks[i + 1].at
                } else {
                    breaks[0].at + TAU
                };
                (breaks[i].at, hi)
            })
            .collect()
    };
    let live: Vec<bool> = bounds
        .iter()
        .flat_map(|&(lo, hi)| {
            let mid = 0.5 * (lo + hi);
            let exists = pencil.disc.eval(mid) - pencil.correction(mid) > 0.0;
            LABELS.map(|label| exists && in_range(pencil.root(mid, None, label).0))
        })
        .collect();
    let count = bounds.len();
    let mut links = vec![[Link::End(EndKind::Clipped); 2]; 2 * count];
    if n == 0 {
        for (piece, link) in links.iter_mut().enumerate() {
            *link = [1, 0].map(|side| Link::To {
                piece,
                side,
                turning: false,
            });
        }
    }
    for (k, at) in breaks.iter().enumerate() {
        // The interval ending at this breakpoint and the one starting.
        let (left, right) = ((k + count - 1) % count, k);
        match at.kind {
            BreakKind::Singular(i) => {
                for l in 0..2 {
                    links[2 * left + l][1] = Link::End(EndKind::Singular(i));
                    links[2 * right + l][0] = Link::End(EndKind::Singular(i));
                }
            }
            BreakKind::Turning => {
                for (interval, side) in [(left, 1), (right, 0)] {
                    if live[2 * interval] && live[2 * interval + 1] {
                        for l in 0..2 {
                            links[2 * interval + l][side] = Link::To {
                                piece: 2 * interval + 1 - l,
                                side,
                                turning: true,
                            };
                        }
                    }
                }
            }
            BreakKind::Clip => {
                for l in 0..2 {
                    if live[2 * left + l] && live[2 * right + l] {
                        links[2 * left + l][1] = Link::To {
                            piece: 2 * right + l,
                            side: 0,
                            turning: false,
                        };
                        links[2 * right + l][0] = Link::To {
                            piece: 2 * left + l,
                            side: 1,
                            turning: false,
                        };
                    }
                }
            }
        }
    }

    let mut visited = vec![false; 2 * count];
    let mut out = Vec::new();
    let walk = |start: usize, enter: usize, visited: &mut Vec<bool>| {
        let mut steps = Vec::new();
        let first = match links[start][enter] {
            Link::End(kind) => Some(kind),
            Link::To { .. } => None,
        };
        let (mut piece, mut side) = (start, enter);
        let last = loop {
            visited[piece] = true;
            match links[piece][1 - side] {
                Link::End(kind) => {
                    steps.push((piece, side == 0, false));
                    break Some(kind);
                }
                Link::To {
                    piece: next,
                    side: next_side,
                    turning,
                } => {
                    steps.push((piece, side == 0, turning));
                    if visited[next] {
                        break None;
                    }
                    (piece, side) = (next, next_side);
                }
            }
        };
        Chain {
            steps,
            ends: first.zip(last).map(|(x, y)| [x, y]),
            bounds: bounds.clone(),
        }
    };
    for piece in 0..2 * count {
        for (side, link) in links[piece].iter().enumerate() {
            if live[piece] && !visited[piece] && matches!(link, Link::End(_)) {
                out.push(walk(piece, side, &mut visited));
            }
        }
    }
    for piece in 0..2 * count {
        if live[piece] && !visited[piece] {
            out.push(walk(piece, 0, &mut visited));
        }
    }
    out
}

/// Chains into branches: pieces of one root merged across the clip's
/// breakpoints into arcs, the arcs laid end to end in the branch's
/// parameter; branches no longer than the tolerance dropped for the
/// point they are; coincident singular points merged.
fn assemble(
    pencil: &Arc<Pencil>,
    chains: Vec<Chain>,
    singular: &[Singular],
    tol: Tolerance,
) -> SectionTrace {
    // Singular points inside the clip, coincident ones merged — a walked
    // cone's apex on the other surface is the singular point of two
    // rulings.
    let mut points: Vec<SectionPoint> = Vec::new();
    let index: Vec<Option<usize>> = singular
        .iter()
        .map(|s| {
            let p = s.point?;
            let found = points
                .iter()
                .position(|q| (q.point - p).norm() <= tol.linear);
            Some(found.unwrap_or_else(|| {
                points.push(SectionPoint {
                    point: p,
                    isolated: true,
                });
                points.len() - 1
            }))
        })
        .collect();
    let end_of = |kind: EndKind| match kind {
        EndKind::Singular(i) => index
            .get(i)
            .copied()
            .flatten()
            .map_or(BranchEnd::Clipped, BranchEnd::Singular),
        EndKind::Clipped => BranchEnd::Clipped,
    };

    let mut branches = Vec::new();
    for chain in chains {
        let mut steps = chain.steps;
        if chain.ends.is_none() {
            // A loop starts just after a turn, so that no arc is split
            // across the branch's start.
            if let Some(k) = steps.iter().position(|step| step.2) {
                steps.rotate_left(k + 1);
            }
        }
        let closed = chain.ends.is_none();
        let turns_at_start = closed && steps.last().is_some_and(|step| step.2);
        let mut arcs: Vec<Arc1> = Vec::new();
        let mut labels: Vec<Label> = Vec::new();
        let mut open: Option<Arc1> = None;
        for (k, &(piece, forward, turns_after)) in steps.iter().enumerate() {
            let (lo, hi) = chain.bounds[piece / 2];
            let arc = open.get_or_insert_with(|| {
                let start = if forward { lo } else { hi };
                let turn_start = if k == 0 {
                    turns_at_start
                } else {
                    steps[k - 1].2
                };
                labels.push(LABELS[piece % 2]);
                Arc1::new(start, start, turn_start, false)
            });
            arc.finish += if forward { hi - lo } else { lo - hi };
            if turns_after || k + 1 == steps.len() {
                arc.turn_finish = turns_after;
                arcs.extend(open.take());
            }
        }
        let t = Arc1::lay(&mut arcs);
        if !(t.is_finite() && t > 0.0) {
            continue;
        }
        let ends = chain.ends.map(|ends| ends.map(end_of));
        let pin = |end: BranchEnd| match end {
            BranchEnd::Singular(i) => points.get(i).map(|p| p.point),
            BranchEnd::Clipped => None,
        };
        let branch = SectionBranch {
            walk: Walk::Ruled {
                pencil: Arc::clone(pencil),
                labels,
            },
            arcs,
            ends,
            pins: ends.map_or([None; 2], |ends| ends.map(pin)),
            length: t,
        };
        let first = branch.point(0.0);
        let extent = (1..EXTENT_SAMPLES)
            .map(|i| (branch.point(t * i as f64 / (EXTENT_SAMPLES - 1) as f64) - first).norm())
            .fold(0.0, f64::max);
        if extent <= tol.linear {
            continue;
        }
        for end in branch.ends.iter().flatten() {
            if let BranchEnd::Singular(i) = *end {
                if let Some(p) = points.get_mut(i) {
                    p.isolated = false;
                }
            }
        }
        branches.push(branch);
    }
    SectionTrace::new(branches, points)
}

/// The extent of the region along the rulings: a parallel family's
/// common direction, measured from the base curve's plane, or for a
/// cone's rulings the farthest corner's distance from the apex on both
/// nappes.
fn extent(pencil: &Pencil, within: &Aabb) -> (f64, f64) {
    let corners = (0..8).map(|i| {
        let pick = |axis: usize| {
            if i >> axis & 1 == 0 {
                within.min[axis]
            } else {
                within.max[axis]
            }
        };
        pencil
            .frame
            .to_local(Point3::new(pick(0), pick(1), pick(2)))
            .coords
    });
    if pencil.d.c == Vec3::zeros() && pencil.d.s == Vec3::zeros() {
        let along = corners.map(|c| (c - pencil.p.k).dot(&pencil.d.k));
        along.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), w| {
            (lo.min(w), hi.max(w))
        })
    } else {
        let reach = corners.map(|c| (c - pencil.p.k).norm()).fold(0.0, f64::max);
        (-reach, reach)
    }
}

/// The rulings of a ruled quadric in the world, `p(s) + w·d(s)` with `d`
/// a unit vector, or `None` for a surface that has none.
fn rulings(surface: &Surface) -> Option<(TrigVec, TrigVec)> {
    let zero = Vec3::zeros();
    match *surface {
        Surface::Cylinder { ref frame, radius } => Some((
            TrigVec {
                k: frame.origin().coords,
                c: frame.x().into_inner() * radius,
                s: frame.y().into_inner() * radius,
            },
            TrigVec {
                k: frame.z().into_inner(),
                c: zero,
                s: zero,
            },
        )),
        Surface::EllipticCylinder {
            ref frame,
            major_radius,
            minor_radius,
        } => Some((
            TrigVec {
                k: frame.origin().coords,
                c: frame.x().into_inner() * major_radius,
                s: frame.y().into_inner() * minor_radius,
            },
            TrigVec {
                k: frame.z().into_inner(),
                c: zero,
                s: zero,
            },
        )),
        Surface::Cone {
            ref frame,
            radius,
            half_angle,
        } => {
            let (sin, cos) = real::sin_cos(half_angle);
            let apex = frame.origin() - frame.z().into_inner() * (radius * cos / sin);
            Some((
                TrigVec {
                    k: apex.coords,
                    c: zero,
                    s: zero,
                },
                TrigVec {
                    k: frame.z().into_inner() * cos,
                    c: frame.x().into_inner() * sin,
                    s: frame.y().into_inner() * sin,
                },
            ))
        }
        Surface::Plane { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. }
        | Surface::Nurbs(_) => None,
    }
}

/// A quadric's implicit form in its own frame, or `None` for a surface
/// that is not one the tracer takes.
fn quadric_of(surface: &Surface) -> Option<(Quadric, Frame)> {
    match *surface {
        Surface::Cylinder { frame, radius } => Some((
            Quadric {
                m: Vec3::new(1.0, 1.0, 0.0),
                l: Vec3::zeros(),
                k: -radius * radius,
            },
            frame,
        )),
        Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        } => Some((
            Quadric {
                m: Vec3::new(
                    minor_radius / major_radius,
                    major_radius / minor_radius,
                    0.0,
                ),
                l: Vec3::zeros(),
                k: -major_radius * minor_radius,
            },
            frame,
        )),
        // `cos²α (x² + y²) − (R cos α + z sin α)²`: the usual form times
        // `cos²α`, so it stays bounded as the cone opens.
        Surface::Cone {
            frame,
            radius,
            half_angle,
        } => {
            let (sin, cos) = real::sin_cos(half_angle);
            Some((
                Quadric {
                    m: Vec3::new(cos * cos, cos * cos, -sin * sin),
                    l: Vec3::new(0.0, 0.0, -radius * cos * sin),
                    k: -radius * radius * cos * cos,
                },
                frame,
            ))
        }
        Surface::Sphere { frame, radius } => Some((
            Quadric {
                m: Vec3::new(1.0, 1.0, 1.0),
                l: Vec3::zeros(),
                k: -radius * radius,
            },
            frame,
        )),
        Surface::Plane { .. } | Surface::Torus { .. } | Surface::Nurbs(_) => None,
    }
}

/// The two operands with the walked one first: a family of parallel
/// rulings before a cone's, which all pass through one point; then the
/// smaller radius or the narrower cone, whose rulings the other is the
/// likelier to meet everywhere; then a circular section before an
/// elliptic one; then the frames, coordinate by coordinate. A rule on
/// the two surfaces, so the argument order never reaches the result.
fn walked_first<'s>(
    a: &'s Surface,
    b: &'s Surface,
) -> Result<(&'s Surface, &'s Surface), GeomError> {
    let key = |s: &Surface| -> Option<(usize, f64, usize, [f64; 9])> {
        let (class, size, kind) = match *s {
            Surface::Cylinder { radius, .. } => (0, radius, 0),
            Surface::EllipticCylinder { minor_radius, .. } => (0, minor_radius, 1),
            Surface::Cone { half_angle, .. } => (1, half_angle, 2),
            Surface::Plane { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => return None,
        };
        let f = s.frame()?;
        let (o, z, x) = (f.origin(), f.z(), f.x());
        Some((
            class,
            size,
            kind,
            [o.x, o.y, o.z, z.x, z.y, z.z, x.x, x.y, x.z],
        ))
    };
    let order = |x: &(usize, f64, usize, [f64; 9]), y: &(usize, f64, usize, [f64; 9])| {
        x.0.cmp(&y.0)
            .then(x.1.total_cmp(&y.1))
            .then(x.2.cmp(&y.2))
            .then_with(|| {
                x.3.iter()
                    .zip(&y.3)
                    .map(|(p, q)| p.total_cmp(q))
                    .find(|o| o.is_ne())
                    .unwrap_or(core::cmp::Ordering::Equal)
            })
    };
    match (key(a), key(b)) {
        (Some(ka), Some(kb)) if order(&kb, &ka).is_lt() => Ok((b, a)),
        (Some(_), _) => Ok((a, b)),
        (None, Some(_)) => Ok((b, a)),
        (None, None) => Err(GeomError::Unsupported {
            a: GeomKind::Surface(a.kind()),
            b: GeomKind::Surface(b.kind()),
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(x: f64, y: f64) -> bool {
        (x - y).abs() <= 1e-12 * (1.0 + x.abs().max(y.abs()))
    }

    #[test]
    fn a_product_of_two_degree_one_polynomials_evaluates_as_the_product() {
        let (f, g) = ([0.3, -1.2, 0.7], [2.0, 0.4, -0.9]);
        let fg = Trig2::product(f, g);
        for k in 0..16 {
            let s = k as f64 * 0.41;
            let want = Trig2::linear(f).eval(s) * Trig2::linear(g).eval(s);
            assert!(close(fg.eval(s), want), "at {s}");
        }
    }

    #[test]
    fn roots_are_found_at_the_blind_spot_and_with_multiplicity() {
        // (1 + cos s): a double root at π, the substitution's blind spot.
        let f = Trig2 {
            k: 1.0,
            c1: 1.0,
            ..Trig2::ZERO
        };
        let roots = f.roots().unwrap();
        assert_eq!(roots.len(), 1);
        assert!((roots[0] - PI).abs() < 1e-7, "{roots:?}");
        // cos 2s: four simple roots.
        let g = Trig2 {
            c2: 1.0,
            ..Trig2::ZERO
        };
        let roots = g.roots().unwrap();
        assert_eq!(roots.len(), 4);
        for (k, r) in roots.iter().enumerate() {
            assert!(
                close(*r, FRAC_PI_4 + k as f64 * 2.0 * FRAC_PI_4),
                "{roots:?}"
            );
        }
        assert!(Trig2::ZERO.roots().is_none());
        assert_eq!(Trig2::constant(3.0).roots(), Some(vec![]));
    }

    #[test]
    fn a_difference_keeps_its_relative_accuracy_at_a_small_offset() {
        let f = Trig2 {
            k: 0.5,
            c1: 1.0,
            s1: -0.3,
            c2: 0.25,
            s2: 0.8,
        };
        let anchor = 1.1;
        for h in [0.4, 1e-3, 1e-9, 1e-14] {
            let want = f.derivative().eval(anchor) * 2.0 * h;
            let got = f.difference(anchor, h);
            if h < 1e-6 {
                assert!((got - want).abs() <= 1e-5 * want.abs(), "h = {h}");
            } else {
                assert!(
                    close(got, f.eval(anchor + 2.0 * h) - f.eval(anchor)),
                    "h = {h}"
                );
            }
        }
    }

    #[test]
    fn a_rotation_is_the_polynomial_at_the_turned_angle() {
        let f = Trig2 {
            k: 0.5,
            c1: 1.0,
            s1: -0.3,
            c2: 0.25,
            s2: 0.8,
        };
        let g = f.rotated(0.7);
        for k in 0..8 {
            let s = k as f64 * 0.9;
            assert!(close(g.eval(s), f.eval(0.7 + s)));
        }
    }
}
