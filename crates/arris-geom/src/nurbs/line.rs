//! A line, a circle or an ellipse against a NURBS surface
//! (`docs/DATA-MODEL.md` §Curve–surface intersection); the text below is
//! the line's, and the conic's differences are at its end.
//!
//! The line is the meet of two planes through it, with unit normals
//! `n₁`, `n₂` perpendicular to its direction `d`. A point of the surface
//! is on the line exactly where both of its offsets `Φₖ = nₖ·(S − o)`
//! vanish, and `|Φ|` is its distance from the line. On one Bézier patch
//! `S = X / W` of the surface, `Hₖ = nₖ·(X − W·o)` are polynomials of the
//! patch's own degrees in Bernstein form, `W > 0`, and `Φ = H / W`: the
//! points where the line meets the patch are the common zeros of two
//! polynomials in `(u, v)`, which [`common_zeros_with`] isolates and
//! certifies — the Krawczyk test proves one simple zero in a box, and
//! Newton's step converges in it. A zero it cannot certify — one that is
//! not simple, the line tangent to the surface there — comes back as a
//! box, and Newton's descent on `Φ` in the box finds it. A patch whose
//! control points all lie beyond the tube of radius `τ = tol.linear`
//! round the line, on one side of one of the planes, is never looked
//! at. Each hit's `t` is `d·(S − o)`, where the surface's point is.
//!
//! **Touches.** What is decided is decided on the distance along the
//! line, as every analytic arm decides it: a stretch where it has an
//! extremum within `τ` is one `tangent` hit at the extremum, absorbing
//! the crossings beside it. Where the line meets the surface at `C` at
//! the angle `θ` to its tangent plane, and the surface's second
//! fundamental form along the line's direction in that plane is `II`,
//! the distance along the line is `s sin θ − II s² / 2` to second order:
//! its extremum is `sin θ / II` along the line from `C`, at the depth
//! `sin² θ / (2 |II|)` — the crossing beyond it is the other side of one
//! fold. A crossing whose extremum the model puts within the tube is
//! that touch where the surface agrees: Newton's iteration on the
//! gradient of `|Φ|²`, from the surface's nearest point to the line there,
//! finds the extremum off the surface within the tube, and the line runs
//! to it from the crossing without leaving the tube; or it converges back
//! onto the crossing with the line tangent there, a double zero. Where the
//! line does not reach the surface at all, the extremum is a critical
//! point of `|Φ|²` off the surface: the common zeros of
//! `Aᵤ = Σ Hₖ (Hₖ,ᵤ W − Hₖ Wᵤ)` and its `v` twin (`W³` times the
//! gradient), kept by two gates — `D = W⁴ d·(Sᵤ × Sᵥ) / W` near zero, the
//! fold where `d` lies in the tangent plane, and `|H|² − τ² W²` at most
//! zero, the tube. Touches whose stretches of the line within the tube
//! overlap are one.
//!
//! A **pole** — a row of the net collapsed to one point, a sphere's or a
//! cone's — that the line passes within the tube makes the whole row a
//! zero, as the tolerance reads it: the offsets and the gradient are
//! divided by the row's linear factor first, carrying what they are off
//! zero by as rounding, and the pole is a hit of its own — at the
//! crossing beside it the line makes, at the pole's own parameters where
//! the line goes through it. It is a touch where the quotient vanishes
//! on the row, the line lying along a meridian's tangent there.
//!
//! **Coincident** is a touch from which the line stays within the tube
//! across the whole surface, parallel within `tol.angular` to a direction
//! the surface does not curve along — a ruling, or any direction of a
//! plane — as an analytic arm reads a line on a ruling.
//!
//! Zeros or critical points that are not isolated — a ruling, or the
//! sliver of near-zeros along a tangency — make the isolation give up;
//! the region is then quartered
//! [`CONTINUUM_SPLITS`] times to keep the isolated points beside it, and
//! what is left is searched by descents spread along each region of boxes
//! the isolation still had alive when it gave up, which between them hold
//! every zero it had not found.
//!
//! What the arm does not look at: a graze of the boundary of the
//! surface's domain, or of a crease between two patches, that is no
//! critical point of either patch — the line's nearest approach to a
//! face's edge is the topology's to decide, not the surface's.
//!
//! **A circle or an ellipse** is the meet of its plane and its cylinder:
//! `Φ₁ = z` in the conic's frame, linear, and `Φ₂ = E / |∇E|` with
//! `E = (x/a)² + (y/b)² − 1`, the distance in the plane from the conic to
//! first order ([`Conic`]). On a patch `Φ₁ = h₁ / W` and, with `E`
//! scaled by a constant for the polynomials, `Φ₂ = h₂ / W²`, of twice the
//! patch's degrees; the isolation, the poles and the tube are the line's.
//! The critical points of `|Φ|²` are the common zeros of
//! `W² h₁ D₁,ₓ + h₂ D₂,ₓ` with `Dₖ,ₓ = hₖ,ₓ W − eₖ hₖ Wₓ`, and the
//! Jacobian's determinant gates them as the fold does. A touch is
//! decided by the second-order model of the line with the conic's own
//! bend added to the surface's sag. A conic that lies on the surface all
//! round is `Coincident`, decided on [`CONIC_SAMPLES`] points.

use arris_math::real;
use arris_math::{DisjointSets, Frame, Interval, Meter, Point2, Point3, Tolerance, Vec3};

use super::NurbsSurface;
use super::project::{LOCATION, Patch};
use crate::bernstein::Binomials;
use crate::bernstein2::{Gate, Halving, Poly2, Zero2, common_zeros_with};
use crate::halt::Halt;
use crate::implicit::BERNSTEIN_ROUNDING;
use crate::{Curve, CurveSurfaceHit, CurveSurfaceIntersection, GeomError, GeomKind, SurfaceEval};

/// How many times a region whose zeros are not isolated is quartered
/// before what is left is searched by descents. A structural bound, not
/// a tolerance: the points isolated beside a curve of zeros are found in
/// the quarters that do not hold it, and two levels leave the curve in at
/// most `2²` of the `4²` quarters along its length.
const CONTINUUM_SPLITS: usize = 2;

/// How many points of a conic are looked at for it lying on the surface
/// all round. A structural count: a circle of a surface's own curvature
/// that leaves it by the tolerance does so over an arc of its length, and
/// a sample every `2π / 64` is within that arc for anything the surface's
/// patches are fine enough to tell from it.
const CONIC_SAMPLES: usize = 64;

/// How many points strictly between a crossing and the extremum beside
/// it are measured for the line staying within the tube between them.
/// The distance along the line is monotone from the one to the other
/// where the second-order model holds, so one would do; three keep an
/// extremum the model put beside the wrong crossing from passing by luck.
const JOIN_SAMPLES: usize = 3;

/// How many of the boxes of one region the isolation gave up on are
/// descended from, spread along it. A region is a curve of zeros or the
/// sliver of near-zeros along a tangency, which holds two zeros or none:
/// a descent from anywhere along it reaches the zero on its own side, and
/// starts spread along the region's length reach both.
const FRONT_STARTS: usize = 16;

/// Steps of the damped Newton iterations on `Φ` and on its gradient.
/// Newton's own step converges quadratically near a simple zero, so the
/// count is only reached by an iteration that crawls along a fold.
const DESCENT_STEPS: usize = 64;

/// The first damping of a Newton step that does not decrease the
/// distance, as a fraction of `JᵀJ`'s trace; each next one is
/// [`DAMPING_GROWTH`] times it. Ratios, scale-free.
const DAMPING: f64 = 1e-6;

/// How much each damping grows over the last.
const DAMPING_GROWTH: f64 = 16.0;

/// How many dampings, or halvings of a step, are tried before an
/// iteration stops.
const DAMPINGS: usize = 12;

/// The line as two plane offsets.
struct Line {
    origin: Point3,
    direction: Vec3,
    normals: [Vec3; 2],
}

impl Line {
    /// `Φ`, the offsets of `p` from the two planes.
    fn offset(&self, p: Point3) -> [f64; 2] {
        let r = p - self.origin;
        self.normals.map(|n| n.dot(&r))
    }

    /// The line's parameter nearest `p`.
    fn t(&self, p: Point3) -> f64 {
        self.direction.dot(&(p - self.origin))
    }
}

/// A circle or an ellipse as two functions of a point whose common zeros
/// are its points. `Φ₁` is the offset from its plane, linear. `Φ₂` is its
/// cylinder's implicit form `E = (x/a)² + (y/b)² − 1` in the conic's
/// frame over `|∇E|`, which is the distance in the plane from the conic to
/// first order; it is the plain `E` over the constant [`Conic::scale`]
/// in the polynomials the isolation reads, where it has to be one.
struct Conic {
    frame: Frame,
    radii: [f64; 2],
    /// `|∇E|` on the conic at the least, `2 / max(a, b)`: inside it the
    /// normalised form is held to this, so it is smooth through the
    /// centre.
    steepness: f64,
}

impl Conic {
    /// `E` and its gradient `(∂E/∂x, ∂E/∂y)` at the local point `l`.
    fn implicit(&self, l: Point3) -> (f64, [f64; 2]) {
        let [a, b] = self.radii;
        (
            l.x * l.x / (a * a) + l.y * l.y / (b * b) - 1.0,
            [2.0 * l.x / (a * a), 2.0 * l.y / (b * b)],
        )
    }

    /// The factor of `E` that makes `Φ₂` a distance in the polynomials:
    /// `1 / |∇E|` at the largest, `min(a, b) / 2`. A point within `δ` of
    /// the conic in the plane then has `|Φ₂| ≤ δ (1 + δ / min(a, b))`
    /// or less, so a tube of `τ` in the distance is a tube of
    /// `τ (1 + τ / min(a, b))` in the polynomial.
    fn scale(&self) -> f64 {
        0.5 * self.radii[0].min(self.radii[1])
    }

    fn offset(&self, p: Point3) -> [f64; 2] {
        let l = self.frame.to_local(p);
        let (e, g) = self.implicit(l);
        [l.z, e / real::hypot(g[0], g[1]).max(self.steepness)]
    }

    /// `∇Φ₁` and `∇Φ₂` at `p`, in world axes.
    fn gradients(&self, p: Point3) -> [Vec3; 2] {
        let l = self.frame.to_local(p);
        let [a, b] = self.radii;
        let (e, g) = self.implicit(l);
        let size = real::hypot(g[0], g[1]);
        let grad = if size > self.steepness {
            // `g = E / G` with `G = |∇E|`, `G ∂G/∂x = Eₓ · 2 / a²`.
            let dg = [g[0] * 2.0 / (a * a) / size, g[1] * 2.0 / (b * b) / size];
            [0, 1].map(|k| g[k] / size - e * dg[k] / (size * size))
        } else {
            g.map(|x| x / self.steepness)
        };
        [
            self.frame.z().into_inner(),
            self.frame.x().into_inner() * grad[0] + self.frame.y().into_inner() * grad[1],
        ]
    }

    /// The curve itself, as the conic's parameter `t` evaluates it.
    fn curve(&self) -> Curve {
        if self.radii[0] == self.radii[1] {
            Curve::Circle {
                frame: self.frame,
                radius: self.radii[0],
            }
        } else {
            Curve::Ellipse {
                frame: self.frame,
                major_radius: self.radii[0],
                minor_radius: self.radii[1],
            }
        }
    }

    /// The parameter of the conic's point nearest `p`; zero at the centre,
    /// where every one is.
    fn position(&self, p: Point3) -> f64 {
        let l = self.frame.to_local(p);
        let t = real::atan2(l.y / self.radii[1], l.x / self.radii[0]);
        if t < 0.0 {
            t + core::f64::consts::TAU
        } else {
            t
        }
    }
}

/// What the surface is cut by: a line, or a circle or an ellipse. Every
/// question the search asks of the curve — its two offsets and their
/// gradients, where along it a point lies, its direction and bend there —
/// has one answer for each.
enum Cut {
    Line(Line),
    Conic(Conic),
}

impl Cut {
    /// `Φ`, the offsets of `p`.
    fn offset(&self, p: Point3) -> [f64; 2] {
        match self {
            Cut::Line(line) => line.offset(p),
            Cut::Conic(conic) => conic.offset(p),
        }
    }

    /// `∇Φ₁` and `∇Φ₂` at `p`.
    fn gradients(&self, p: Point3) -> [Vec3; 2] {
        match self {
            Cut::Line(line) => line.normals,
            Cut::Conic(conic) => conic.gradients(p),
        }
    }

    /// The curve's parameter nearest `p`.
    fn position(&self, p: Point3) -> f64 {
        match self {
            Cut::Line(line) => line.t(p),
            Cut::Conic(conic) => conic.position(p),
        }
    }

    /// The curve's point at the parameter `t`.
    fn point_at(&self, t: f64) -> Point3 {
        match self {
            Cut::Line(line) => line.origin + line.direction * t,
            Cut::Conic(conic) => conic.curve().point(t),
        }
    }

    /// The unit tangent at the point of the curve nearest `p`.
    fn direction(&self, p: Point3) -> Option<Vec3> {
        match self {
            Cut::Line(line) => Some(line.direction),
            Cut::Conic(conic) => {
                let t = conic.position(p);
                conic.curve().eval(t).d1.try_normalize(0.0)
            }
        }
    }

    /// The curvature vector `κN` at the point of the curve nearest `p`:
    /// zero on a line.
    fn bend(&self, p: Point3) -> Vec3 {
        match self {
            Cut::Line(_) => Vec3::zeros(),
            Cut::Conic(conic) => {
                let e = conic.curve().eval(conic.position(p));
                let speed2 = e.d1.norm_squared();
                if speed2 == 0.0 {
                    return Vec3::zeros();
                }
                (e.d2 - e.d1 * (e.d1.dot(&e.d2) / speed2)) / speed2
            }
        }
    }

    /// The speed of the parameter at `t`: length per unit.
    fn speed(&self, t: f64) -> f64 {
        match self {
            Cut::Line(_) => 1.0,
            Cut::Conic(conic) => conic.curve().eval(t).d1.norm(),
        }
    }

    /// The parameter a length `s` along the curve from `t`.
    fn shifted(&self, t: f64, s: f64) -> f64 {
        let speed = self.speed(t);
        if speed > 0.0 { t + s / speed } else { t }
    }

    /// `to − from` as a signed step along the curve: the short way round
    /// a conic.
    fn step(&self, from: f64, to: f64) -> f64 {
        match self {
            Cut::Line(_) => to - from,
            Cut::Conic(_) => {
                let turn = core::f64::consts::TAU;
                let d = (to - from).rem_euclid(turn);
                if d > 0.5 * turn { d - turn } else { d }
            }
        }
    }

    /// The length of curve between two parameters, the short way.
    fn gap(&self, a: f64, b: f64) -> f64 {
        self.step(a, b).abs() * self.speed(a)
    }
}

/// Something found on a patch, at `uv` in its box, in the surface's own
/// parameters.
#[derive(Clone, Copy, Debug)]
struct Found {
    uv: [f64; 2],
    lo: [f64; 2],
    hi: [f64; 2],
}

/// A touch: an extremum of the distance along the line within the tube,
/// at `uv`, and the distance there.
#[derive(Clone, Copy, Debug)]
struct Touch {
    uv: [f64; 2],
    distance: f64,
}

/// A pole the line passes within the tube.
#[derive(Clone, Copy)]
struct Pole {
    uv: [f64; 2],
    point: Point3,
    tangent: bool,
}

/// One Bézier patch as the line sees it.
struct Piece {
    /// The patch's box of the surface's parameters.
    span: [[f64; 2]; 2],
    /// `nₖ·(X − W·o)`.
    h: [Poly2; 2],
    w: Poly2,
    /// The rounding of each of `h`'s coefficients.
    floors: [f64; 2],
    /// Bounds on `|h|`'s and `w`'s coefficients.
    magnitude: [f64; 2],
    /// The rows of the net collapsed to one point within the tube: the
    /// direction whose parameter is fixed along the row and its value,
    /// `0` or `1`.
    poles: Vec<(usize, f64)>,
    /// The Cartesian offsets of the control points.
    offsets: Vec<[f64; 2]>,
    /// How many of the offsets are linear, so that their control points
    /// bound the patch: both of a line's, a conic's plane alone.
    planes: usize,
}

impl Piece {
    fn of(patch: &Patch, cut: &Cut, reach: f64, tau: f64, binomials: &Binomials) -> Piece {
        let [p, q] = patch.degree;
        let mut h = [Vec::new(), Vec::new()];
        let mut w = Vec::new();
        let mut offsets = Vec::new();
        let (mut mh, mut mw) = (0.0f64, 0.0f64);
        match cut {
            Cut::Line(line) => {
                let o = line.origin.coords;
                for x in &patch.net {
                    let xyz = Vec3::new(x.x, x.y, x.z);
                    for (hk, n) in h.iter_mut().zip(line.normals) {
                        hk.push(n.dot(&xyz) - x.w * n.dot(&o));
                    }
                    w.push(x.w);
                    offsets.push(cut.offset(Point3::from(xyz / x.w)));
                    mh = mh.max(xyz.norm() + x.w * o.norm());
                    mw = mw.max(x.w);
                }
            }
            Cut::Conic(conic) => {
                // The three local coordinates of `X − W·c`, and `W`.
                let o = conic.frame.origin().coords;
                let [a, b] = conic.radii;
                let (ex, ey, ez) = (
                    conic.frame.x().into_inner(),
                    conic.frame.y().into_inner(),
                    conic.frame.z().into_inner(),
                );
                let (mut lx, mut ly) = (Vec::new(), Vec::new());
                for x in &patch.net {
                    let xyz = Vec3::new(x.x, x.y, x.z);
                    let r = xyz - o * x.w;
                    lx.push(ex.dot(&r) / a);
                    ly.push(ey.dot(&r) / b);
                    h[0].push(ez.dot(&r));
                    w.push(x.w);
                    offsets.push([cut.offset(Point3::from(xyz / x.w))[0], 0.0]);
                    mh = mh.max(xyz.norm() + x.w * o.norm());
                    mw = mw.max(x.w);
                }
                let square = |c: Vec<f64>| {
                    let poly = Poly2::new([p, q], c);
                    poly.mul(&poly, binomials)
                };
                let (lx2, ly2) = (square(lx), square(ly));
                let w2 = square(w.clone());
                let e = Poly2::combine(&[(1.0, &lx2), (1.0, &ly2), (-1.0, &w2)]);
                h[1] = e.coefficients().iter().map(|c| c * conic.scale()).collect();
            }
        }
        let cartesian = |a: usize, b: usize| {
            let x = patch.net[a * (q + 1) + b];
            Point3::new(x.x / x.w, x.y / x.w, x.z / x.w)
        };
        let rows: [(usize, f64, Vec<Point3>); 4] = [
            (0, 0.0, (0..=q).map(|b| cartesian(0, b)).collect()),
            (0, 1.0, (0..=q).map(|b| cartesian(p, b)).collect()),
            (1, 0.0, (0..=p).map(|a| cartesian(a, 0)).collect()),
            (1, 1.0, (0..=p).map(|a| cartesian(a, q)).collect()),
        ];
        let poles = rows
            .iter()
            .filter(|(_, _, row)| {
                row.iter()
                    .all(|x| arris_math::is_negligible((x - row[0]).norm(), reach))
                    && norm(cut.offset(row[0])) <= tau
            })
            .map(|(k, r, _)| (*k, *r))
            .collect();
        let (degrees, floors, planes) = match cut {
            Cut::Line(_) => ([p, q], [BERNSTEIN_ROUNDING * mh; 2], 2),
            Cut::Conic(conic) => {
                // `h₂` is a sum of three squares of polynomials of the
                // patch's degrees, scaled.
                let [a, b] = conic.radii;
                let size = conic.scale() * (mh * mh / (a * a) + mh * mh / (b * b) + mw * mw);
                let factor = 2.0 * (2 * p + 2 * q + 2) as f64;
                (
                    [2 * p, 2 * q],
                    [BERNSTEIN_ROUNDING * mh, BERNSTEIN_ROUNDING * factor * size],
                    1,
                )
            }
        };
        let [d0, d1] = match cut {
            Cut::Line(_) => [[p, q], [p, q]],
            Cut::Conic(_) => [[p, q], degrees],
        };
        Piece {
            span: [patch.u, patch.v],
            h: [Poly2::new(d0, h[0].clone()), Poly2::new(d1, h[1].clone())],
            w: Poly2::new([p, q], w),
            floors,
            magnitude: [mh, mw],
            poles,
            offsets,
            planes,
        }
    }

    /// The surface's parameters at the patch's own `local` ones.
    fn global(&self, local: [f64; 2]) -> [f64; 2] {
        [0, 1].map(|k| self.span[k][0] + local[k] * (self.span[k][1] - self.span[k][0]))
    }

    fn found(&self, z: &Zero2) -> Found {
        Found {
            uv: self.global(z.at),
            lo: self.global(z.lo),
            hi: self.global(z.hi),
        }
    }

    /// Every control point farther than `reach` from the line on one side
    /// of one of its planes: the patch, inside the hull of its control
    /// points, is too.
    fn clear_of(&self, reach: f64) -> bool {
        (0..self.planes).any(|k| {
            self.offsets.iter().all(|c| c[k] > reach) || self.offsets.iter().all(|c| c[k] < -reach)
        })
    }
}

/// `p` divided by its linear factor across the row `(k, r)`, and the
/// carried rounding. What `p` is off zero by along the row — rounding
/// where the line goes through the pole, up to the tolerance where it
/// passes it within the tube — is dropped by the division, so it is
/// carried as rounding too: the quotient is the polynomial of the
/// surface with its pole moved onto the line, and the floors say so.
fn over_row(p: &Poly2, (k, r): (usize, f64), floor: f64, b: &Binomials) -> (Poly2, f64) {
    let row = if k == 0 { p.across(r) } else { p.along(r) };
    let off = row.iter().fold(floor, |m, c| m.max(c.abs()));
    if k == 0 {
        p.over_linear(r, off, b)
    } else {
        let (q, f) = p.transposed().over_linear(r, off, b);
        (q.transposed(), f)
    }
}

/// The intersection of the line through `origin` along the unit
/// `direction` and a NURBS surface: module docs.
pub(crate) fn line_nurbs(
    curve: &Curve,
    origin: Point3,
    direction: Vec3,
    surface: &NurbsSurface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveSurfaceIntersection, GeomError> {
    let frame = Frame::from_z(origin, direction).map_err(|e| GeomError::Degenerate {
        kind: GeomKind::Curve(curve.kind()),
        reason: format!("the line's direction has no frame: {e}"),
    })?;
    let line = Line {
        origin,
        direction: frame.z().into_inner(),
        normals: [frame.x().into_inner(), frame.y().into_inner()],
    };
    query(surface, Cut::Line(line), origin, tol).run(curve, meter)
}

/// The intersection of a circle or an ellipse and a NURBS surface: the
/// module docs, with the conic's plane and cylinder in the line's two
/// planes' place. `frame` is the conic's, `radii` its semi-axes.
pub(crate) fn conic_nurbs(
    curve: &Curve,
    frame: &Frame,
    radii: [f64; 2],
    surface: &NurbsSurface,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveSurfaceIntersection, GeomError> {
    if !(radii.iter().all(|r| r.is_finite() && *r > 0.0)) {
        return Err(GeomError::Degenerate {
            kind: GeomKind::Curve(curve.kind()),
            reason: "the conic's radii are not positive".to_owned(),
        });
    }
    let conic = Conic {
        frame: *frame,
        radii,
        steepness: 2.0 / radii[0].max(radii[1]),
    };
    query(surface, Cut::Conic(conic), frame.origin(), tol).run(curve, meter)
}

/// The search of `surface` for `cut`, the curve's `anchor` a point of it
/// that enters the offsets.
fn query(surface: &NurbsSurface, cut: Cut, anchor: Point3, tol: Tolerance) -> Query<'_> {
    let reach = surface
        .control_points()
        .iter()
        .map(|p| p.coords.norm())
        .fold(0.0, f64::max)
        + anchor.coords.norm();
    let (lo, hi) = surface.control_points().iter().fold(
        ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]),
        |(lo, hi), p| {
            (
                [0, 1, 2].map(|k| lo[k].min(p[k])),
                [0, 1, 2].map(|k| hi[k].max(p[k])),
            )
        },
    );
    let extent = (0..3)
        .map(|k| real::powi(hi[k] - lo[k], 2))
        .sum::<f64>()
        .sqrt();
    Query {
        surface,
        extent,
        cut,
        tau: tol.linear,
        angular: tol.angular,
        reach,
        domain: surface.domain(),
        closure: surface.closure(),
    }
}

struct Query<'a> {
    surface: &'a NurbsSurface,
    cut: Cut,
    tau: f64,
    angular: f64,
    /// The diagonal of the control net's box.
    extent: f64,
    /// The magnitude of the coordinates that enter an offset.
    reach: f64,
    domain: [Interval; 2],
    closure: [Option<f64>; 2],
}

#[derive(Default)]
struct Collected {
    /// Where the line meets the surface.
    meets: Vec<Found>,
    /// Critical points of the distance off the surface, within the tube.
    folds: Vec<Touch>,
    poles: Vec<Pole>,
}

/// The critical points of `|Φ|²` on a patch as a system: the two
/// polynomials whose common zeros they are, the roundings of their
/// coefficients, the fold gate and the tube gate with theirs.
struct FoldSystem {
    a: [Poly2; 2],
    floors: [f64; 2],
    fold: Poly2,
    floor_fold: f64,
    tube: Poly2,
    floor_tube: f64,
}

/// A polynomial that only excludes, as [`Gate`] takes it.
struct GateSpec<'a> {
    poly: &'a Poly2,
    floor: f64,
    one_sided: bool,
}

/// What an isolation over a patch found, in the patch's own parameters:
/// its zeros and certified boxes, and, where its zeros are not isolated,
/// the boxes alive when the isolation gave up, which hold every zero it
/// had not found.
#[derive(Default)]
struct Isolated {
    zeros: Vec<Zero2>,
    continua: Vec<Zero2>,
}

impl Query<'_> {
    fn run(&self, curve: &Curve, meter: &mut Meter) -> Result<CurveSurfaceIntersection, GeomError> {
        let [p, q] = self.surface.degree();
        let binomials = Binomials::new(match self.cut {
            Cut::Line(_) => 3 * p.max(q) + 1,
            Cut::Conic(_) => 6 * p.max(q) + 2,
        });
        let slack = BERNSTEIN_ROUNDING * self.reach;
        let mut all = Collected::default();
        let pieces: Vec<Piece> = self
            .surface
            .patches()
            .iter()
            .map(|patch| Piece::of(patch, &self.cut, self.reach, self.tau, &binomials))
            .filter(|piece| !piece.clear_of(self.tau + slack))
            .collect();
        for piece in &pieces {
            meter.tick()?;
            self.meets(piece, &binomials, &mut all, meter)?;
        }
        for piece in &pieces {
            meter.tick()?;
            self.folds(piece, &binomials, &mut all, meter)?;
        }
        self.hits(curve, all)
    }

    /// The common zeros of `system` over `region` of the patch, the gates
    /// applied: a region whose zeros are not isolated is quartered
    /// [`CONTINUUM_SPLITS`] times, and what is left of it is returned as
    /// a continuum.
    #[allow(clippy::too_many_arguments)] // the system, its floors and gates, the region and the recursion
    fn isolate(
        &self,
        system: [&Poly2; 2],
        floors: [f64; 2],
        gates: &[GateSpec<'_>],
        region: [[f64; 2]; 2],
        depth: usize,
        out: &mut Isolated,
        meter: &mut Meter,
    ) -> Result<(), GeomError> {
        let whole = region == [[0.0, 1.0], [0.0, 1.0]];
        let part = |p: &Poly2| {
            if whole {
                p.clone()
            } else {
                p.restricted(region[0], region[1])
            }
        };
        let parts = system.map(part);
        let gate_parts: Vec<Poly2> = gates.iter().map(|g| part(g.poly)).collect();
        let gates_here: Vec<Gate<'_>> = gates
            .iter()
            .zip(&gate_parts)
            .map(|(g, poly)| Gate {
                poly,
                floor: g.floor,
                one_sided: g.one_sided,
            })
            .collect();
        let to_patch =
            |s: [f64; 2]| [0, 1].map(|k| region[k][0] + s[k] * (region[k][1] - region[k][0]));
        let in_patch = |z: &Zero2| Zero2 {
            at: to_patch(z.at),
            lo: to_patch(z.lo),
            hi: to_patch(z.hi),
            certified: z.certified,
        };
        match common_zeros_with(
            [&parts[0], &parts[1]],
            floors,
            &gates_here,
            Halving::LookAhead,
            meter,
        ) {
            Ok(isolation) => {
                out.zeros.extend(isolation.zeros.iter().map(in_patch));
                Ok(())
            }
            Err(Halt::Stopped(stop)) => Err(GeomError::Interrupted(stop)),
            Err(Halt::Fault(_)) if depth < CONTINUUM_SPLITS => {
                for i in 0..4 {
                    let quarter = [0, 1].map(|k| {
                        let mid = 0.5 * (region[k][0] + region[k][1]);
                        if (i >> k) & 1 == 0 {
                            [region[k][0], mid]
                        } else {
                            [mid, region[k][1]]
                        }
                    });
                    self.isolate(system, floors, gates, quarter, depth + 1, out, meter)?;
                }
                Ok(())
            }
            Err(Halt::Fault(continuum)) => {
                out.zeros.extend(continuum.found.iter().map(in_patch));
                out.continua.extend(continuum.front.iter().map(in_patch));
                Ok(())
            }
        }
    }

    /// Where the line meets the patch, and the poles it passes within
    /// the tube.
    fn meets(
        &self,
        piece: &Piece,
        binomials: &Binomials,
        all: &mut Collected,
        meter: &mut Meter,
    ) -> Result<(), GeomError> {
        let mut h = piece.h.clone();
        let mut floors = piece.floors;
        for &row in &piece.poles {
            for k in 0..2 {
                (h[k], floors[k]) = over_row(&h[k], row, floors[k], binomials);
            }
        }
        let mut found = Isolated::default();
        let whole = [[0.0, 1.0], [0.0, 1.0]];
        self.isolate([&h[0], &h[1]], floors, &[], whole, 0, &mut found, meter)?;
        let mut tangent_at = vec![false; piece.poles.len()];
        for z in &found.zeros {
            // A zero of the quotient on a pole's row is the line along a
            // meridian's tangent there: the pole is a touch.
            if let Some(i) = piece.poles.iter().position(|&(k, r)| {
                z.lo[k] <= r
                    && r <= z.hi[k]
                    && (!z.certified || arris_math::is_negligible(z.at[k] - r, 1.0))
            }) {
                tangent_at[i] = true;
                continue;
            }
            let f = piece.found(z);
            if z.certified {
                // A zero past the square, by more than rounding, is the
                // neighbour's: this patch's polynomial there is no piece
                // of the surface; and the box holds one zero of it only
                // over the square.
                if self.owns(piece, z.at) {
                    let mut f = f;
                    for k in 0..2 {
                        f.lo[k] = f.lo[k].max(piece.span[k][0]).min(f.uv[k]);
                        f.hi[k] = f.hi[k].min(piece.span[k][1]).max(f.uv[k]);
                    }
                    all.meets.push(f);
                }
            } else {
                self.search(&f, all);
            }
        }
        self.search_front(piece, &found.continua, all);
        for (&(k, r), tangent) in piece.poles.iter().zip(tangent_at) {
            let mut local = [0.5; 2];
            local[k] = r;
            let mut pole = piece.global(local);
            pole[1 - k] = self.domain[1 - k].lo();
            let point = self.surface.eval(pole[0], pole[1]).point;
            // Where the line passes the pole within the tube rather than
            // through it, the crossing beside it is found from it on the
            // surface itself.
            let (beside, distance) = self.descend(pole, piece.span);
            let uv = if tangent || distance > self.rounding() {
                pole
            } else {
                beside
            };
            all.poles.push(Pole { uv, point, tangent });
        }
        Ok(())
    }

    /// The boxes an isolation still had alive when it gave up, searched:
    /// boxes that touch are one region — a curve of zeros, or the sliver
    /// of near-zeros along a tangency — and each region is searched from
    /// up to [`FRONT_STARTS`] of its boxes spread along it, the descent
    /// held in the region.
    fn search_front(&self, piece: &Piece, front: &[Zero2], all: &mut Collected) {
        let mut boxes: Vec<Found> = front.iter().map(|z| piece.found(z)).collect();
        boxes.sort_by(order);
        // Union-find over boxes that touch, in a fixed order.
        let mut sets = DisjointSets::new(boxes.len());
        for i in 0..boxes.len() {
            for j in i + 1..boxes.len() {
                let touch = (0..2)
                    .all(|k| boxes[i].lo[k] <= boxes[j].hi[k] && boxes[j].lo[k] <= boxes[i].hi[k]);
                if touch {
                    sets.union(i, j);
                }
            }
        }
        let mut regions: Vec<Vec<usize>> = Vec::new();
        let mut index = vec![usize::MAX; boxes.len()];
        for i in 0..boxes.len() {
            let r = sets.find(i);
            if index[r] == usize::MAX {
                index[r] = regions.len();
                regions.push(Vec::new());
            }
            regions[index[r]].push(i);
        }
        for members in regions {
            let mut hull = boxes[members[0]];
            for &i in &members {
                for k in 0..2 {
                    hull.lo[k] = hull.lo[k].min(boxes[i].lo[k]);
                    hull.hi[k] = hull.hi[k].max(boxes[i].hi[k]);
                }
            }
            let starts = members.len().min(FRONT_STARTS);
            for s in 0..starts {
                let i = members[s * members.len() / starts];
                self.search(
                    &Found {
                        uv: boxes[i].uv,
                        ..hull
                    },
                    all,
                );
            }
        }
    }

    /// A descent from the middle of a box the isolation could not decide,
    /// held in it: the zero it reaches is where the line meets the
    /// surface; one that only gets within the tube has found a candidate
    /// for a touch, which Newton's iteration on the gradient settles.
    fn search(&self, f: &Found, all: &mut Collected) {
        let b = bounds(f);
        let (uv, distance) = self.descend(f.uv, b);
        if distance <= self.rounding() {
            all.meets.push(Found { uv, ..*f });
        } else if distance <= self.tau {
            let uv = self.critical(uv, self.surface_bounds());
            let distance = self.distance(uv);
            if distance <= self.rounding() {
                // The critical point is a zero: a meeting.
                all.meets.push(Found { uv, ..*f });
            } else if distance <= self.tau && self.is_critical(uv) {
                all.folds.push(Touch { uv, distance });
            }
        }
    }

    /// Whether the patch-local `at` is in the patch's square, or past it
    /// by no more than the rounding of the coordinates on the surface.
    fn owns(&self, piece: &Piece, at: [f64; 2]) -> bool {
        let held = at.map(|x| x.clamp(0.0, 1.0));
        if held == at {
            return true;
        }
        let [a, b] = [piece.global(at), piece.global(held)];
        let off = self.surface.eval(a[0], a[1]).point - self.surface.eval(b[0], b[1]).point;
        arris_math::is_negligible(off.norm(), LOCATION * self.reach)
    }

    /// The rounding of an offset from the line: what a zero of it is
    /// located to on the surface.
    fn rounding(&self) -> f64 {
        LOCATION * BERNSTEIN_ROUNDING * self.reach
    }

    /// The distance of the surface's point at `uv` from the line.
    fn distance(&self, uv: [f64; 2]) -> f64 {
        norm(self.cut.offset(self.surface.eval(uv[0], uv[1]).point))
    }

    /// The patch's critical points of the distance within the tube, off
    /// the surface: the extrema of a curve that does not reach it.
    fn folds(
        &self,
        piece: &Piece,
        binomials: &Binomials,
        all: &mut Collected,
        meter: &mut Meter,
    ) -> Result<(), GeomError> {
        let Some(FoldSystem {
            mut a,
            mut floors,
            fold,
            floor_fold,
            tube,
            floor_tube,
        }) = (match &self.cut {
            Cut::Line(_) => self.line_folds(piece, binomials),
            Cut::Conic(conic) => self.conic_folds(piece, conic, binomials),
        })
        else {
            return Ok(());
        };
        // Through a pole the gradient vanishes along its row, across it
        // once and along it twice.
        for &(k, r) in &piece.poles {
            for (i, ai) in a.iter_mut().enumerate() {
                let times = if i == k { 1 } else { 2 };
                for _ in 0..times {
                    (*ai, floors[i]) = over_row(ai, (k, r), floors[i], binomials);
                }
            }
        }
        let gates = [
            GateSpec {
                poly: &fold,
                floor: floor_fold,
                one_sided: false,
            },
            GateSpec {
                poly: &tube,
                floor: floor_tube,
                one_sided: true,
            },
        ];
        let mut found = Isolated::default();
        let whole = [[0.0, 1.0], [0.0, 1.0]];
        self.isolate([&a[0], &a[1]], floors, &gates, whole, 0, &mut found, meter)?;
        for z in &found.zeros {
            let f = piece.found(z);
            // A box that is not certified is one where the polynomials'
            // rounding hides the gradient: Newton on the gradient itself,
            // evaluated on the surface, finds the critical point in it.
            let uv = if z.certified {
                f.uv
            } else {
                let held = self.critical(f.uv, bounds(&f));
                let free = self.critical(held, self.surface_bounds());
                if !self.is_critical(free) {
                    continue;
                }
                free
            };
            let distance = self.distance(uv);
            if distance <= self.rounding() {
                // A crossing the fold gate did not exclude: a meeting.
                all.meets.push(Found { uv, ..f });
            } else if distance <= self.tau {
                all.folds.push(Touch { uv, distance });
            }
        }
        self.search_front(piece, &found.continua, all);
        Ok(())
    }

    /// The critical-point system of a line on a patch: `W³` times half the
    /// gradient of `|Φ|²`, the fold gate `W⁴ d·(Sᵤ × Sᵥ)` over `W` and the
    /// tube `|H|² ≤ τ² W²`; `None` where the fold keeps one sign over the
    /// patch, so that no point of it is critical but on the line.
    fn line_folds(&self, piece: &Piece, binomials: &Binomials) -> Option<FoldSystem> {
        let [h1, h2] = &piece.h;
        let w = &piece.w;
        let [m, n] = h1.degree().map(|d| d as f64);
        let [mh, mw] = piece.magnitude;
        let mul = |a: &Poly2, b: &Poly2| a.mul(b, binomials);
        let fold = Poly2::combine(&[
            (1.0, &mul(w, &mul(&h1.du(), &h2.dv()))),
            (-1.0, &mul(w, &mul(&h2.du(), &h1.dv()))),
            (1.0, &mul(&w.dv(), &mul(h1, &h2.du()))),
            (-1.0, &mul(&w.dv(), &mul(h2, &h1.du()))),
            (1.0, &mul(&w.du(), &mul(&h1.dv(), h2))),
            (-1.0, &mul(&w.du(), &mul(&h2.dv(), h1))),
        ]);
        let floor_fold = BERNSTEIN_ROUNDING * 24.0 * m * n * mw * mh * mh;
        if fold.sign(floor_fold).is_some() {
            return None;
        }
        let gradient = |dh: [Poly2; 2], dw: Poly2| {
            let terms: Vec<Poly2> = (0..2)
                .map(|k| {
                    let inner =
                        Poly2::combine(&[(1.0, &mul(&dh[k], w)), (-1.0, &mul(&piece.h[k], &dw))]);
                    mul(&piece.h[k], &inner)
                })
                .collect();
            Poly2::combine(&[(1.0, &terms[0]), (1.0, &terms[1])])
        };
        let a = [
            gradient([h1.du(), h2.du()], w.du()),
            gradient([h1.dv(), h2.dv()], w.dv()),
        ];
        let floors = [
            BERNSTEIN_ROUNDING * 8.0 * m * mh * mh * mw,
            BERNSTEIN_ROUNDING * 8.0 * n * mh * mh * mw,
        ];
        let tube = Poly2::combine(&[
            (1.0, &mul(h1, h1)),
            (1.0, &mul(h2, h2)),
            (-self.tau * self.tau, &mul(w, w)),
        ]);
        let floor_tube = BERNSTEIN_ROUNDING * (2.0 * mh * mh + self.tau * self.tau * mw * mw);
        Some(FoldSystem {
            a,
            floors,
            fold,
            floor_fold,
            tube,
            floor_tube,
        })
    }

    /// The same for a conic, whose `Φ₁ = h₁ / W` and `Φ₂ = h₂ / W²` are
    /// homogeneous of different degrees. With `Dₖ,ₓ = hₖ,ₓ W − eₖ hₖ Wₓ`
    /// (`e = (1, 2)`), `∂ₓΦₖ = Dₖ,ₓ / W^{eₖ+1}`: half the gradient of
    /// `|Φ|²` times `W⁵` is `W² h₁ D₁,ₓ + h₂ D₂,ₓ`, the Jacobian's
    /// determinant times `W⁵` is `D₁,ᵤ D₂,ᵥ − D₁,ᵥ D₂,ᵤ`, and the tube
    /// `|Φ|² ≤ τ²` times `W⁴` is `W² h₁² + h₂² − τ² W⁴`, with `τ` grown by
    /// what [`Conic::scale`] says the polynomial `Φ₂` may exceed the
    /// distance by.
    fn conic_folds(
        &self,
        piece: &Piece,
        conic: &Conic,
        binomials: &Binomials,
    ) -> Option<FoldSystem> {
        let [h1, h2] = &piece.h;
        let w = &piece.w;
        let mul = |a: &Poly2, b: &Poly2| a.mul(b, binomials);
        let magnitude = |p: &Poly2| p.coefficients().iter().fold(0.0f64, |m, c| m.max(c.abs()));
        let d = |h: &Poly2, e: f64, partial: &dyn Fn(&Poly2) -> Poly2| {
            Poly2::combine(&[(1.0, &mul(&partial(h), w)), (-e, &mul(h, &partial(w)))])
        };
        let du = |p: &Poly2| p.du();
        let dv = |p: &Poly2| p.dv();
        let (d1u, d1v) = (d(h1, 1.0, &du), d(h1, 1.0, &dv));
        let (d2u, d2v) = (d(h2, 2.0, &du), d(h2, 2.0, &dv));
        let fold = Poly2::combine(&[(1.0, &mul(&d1u, &d2v)), (-1.0, &mul(&d1v, &d2u))]);
        let [m, n] = fold.degree().map(|x| x as f64);
        let growth = 2.0 * (m + n + 2.0);
        let floor_fold = BERNSTEIN_ROUNDING
            * growth
            * (magnitude(&d1u) * magnitude(&d2v) + magnitude(&d1v) * magnitude(&d2u));
        if fold.sign(floor_fold).is_some() {
            return None;
        }
        let w2 = mul(w, w);
        let gradient = |d1: &Poly2, d2: &Poly2| {
            Poly2::combine(&[(1.0, &mul(&mul(&w2, h1), d1)), (1.0, &mul(h2, d2))])
        };
        let a = [gradient(&d1u, &d2u), gradient(&d1v, &d2v)];
        let floors = [0, 1].map(|k| {
            let (d1, d2) = if k == 0 { (&d1u, &d2u) } else { (&d1v, &d2v) };
            let [m, n] = a[k].degree().map(|x| x as f64);
            BERNSTEIN_ROUNDING
                * 2.0
                * (m + n + 2.0)
                * (magnitude(&w2) * magnitude(h1) * magnitude(d1) + magnitude(h2) * magnitude(d2))
        });
        let [lo, hi] = [0, 1].map(|k| conic.radii[k]);
        let reach = self.tau * (1.0 + self.tau / lo.min(hi));
        let w4 = mul(&w2, &w2);
        let tube = Poly2::combine(&[
            (1.0, &mul(&w2, &mul(h1, h1))),
            (1.0, &mul(h2, h2)),
            (-reach * reach, &w4),
        ]);
        let floor_tube = BERNSTEIN_ROUNDING
            * 2.0
            * (tube.degree()[0] + tube.degree()[1] + 2) as f64
            * (magnitude(&w2) * magnitude(h1) * magnitude(h1)
                + magnitude(h2) * magnitude(h2)
                + reach * reach * magnitude(&w4));
        Some(FoldSystem {
            a,
            floors,
            fold,
            floor_fold,
            tube,
            floor_tube,
        })
    }

    /// `Φ`, its Jacobian and the surface's evaluation at `uv`.
    fn offset(&self, uv: [f64; 2]) -> ([f64; 2], [[f64; 2]; 2], SurfaceEval) {
        let e = self.surface.eval(uv[0], uv[1]);
        let f = self.cut.offset(e.point);
        let j = self
            .cut
            .gradients(e.point)
            .map(|n| [n.dot(&e.du), n.dot(&e.dv)]);
        (f, j, e)
    }

    /// The local minimum of the distance from the line that a damped
    /// Newton descent on `Φ` reaches from `start`, held inside `bounds`,
    /// and the distance there. The undamped step is Newton's on the two
    /// offsets, which converges quadratically onto a simple zero; a step
    /// that does not decrease the distance is damped
    /// (Levenberg–Marquardt) until one does, and the descent stops where
    /// none does.
    fn descend(&self, start: [f64; 2], bounds: [[f64; 2]; 2]) -> ([f64; 2], f64) {
        let hold = |x: [f64; 2]| [0, 1].map(|k| x[k].clamp(bounds[k][0], bounds[k][1]));
        let mut x = hold(start);
        let (mut f, mut j, mut e) = self.offset(x);
        let mut r = f[0] * f[0] + f[1] * f[1];
        for _ in 0..DESCENT_STEPS {
            let jtj = gram(j);
            let g = [
                j[0][0] * f[0] + j[1][0] * f[1],
                j[0][1] * f[0] + j[1][1] * f[1],
            ];
            let trace = jtj[0] + jtj[2];
            let mut moved = None;
            for k in 0..=DAMPINGS {
                let mu = if k == 0 {
                    0.0
                } else {
                    trace * DAMPING * real::powi(DAMPING_GROWTH, k as i32 - 1)
                };
                let (a, b, c) = (jtj[0] + mu, jtj[1], jtj[2] + mu);
                let det = a * c - b * b;
                if !(det.is_finite() && det > 0.0) {
                    continue;
                }
                let step = [-(c * g[0] - b * g[1]) / det, -(a * g[1] - b * g[0]) / det];
                let y = hold([x[0] + step[0], x[1] + step[1]]);
                let (fy, jy, ey) = self.offset(y);
                let ry = fy[0] * fy[0] + fy[1] * fy[1];
                if ry < r {
                    moved = Some((y, fy, jy, ey, ry));
                    break;
                }
            }
            let Some((y, fy, jy, ey, ry)) = moved else {
                break;
            };
            let travelled = travel(&e, x, y);
            (x, f, j, e, r) = (y, fy, jy, ey, ry);
            if arris_math::is_negligible(travelled, self.reach) {
                break;
            }
        }
        (x, r.sqrt())
    }

    /// The gradient `JᵀΦ` of `½|Φ|²` at `uv`, its Hessian
    /// `JᵀJ + Σ Φₖ ∇²Φₖ` as `[a, b, c]` for `[[a, b], [b, c]]`, and the
    /// surface's evaluation there.
    fn second_order(&self, uv: [f64; 2]) -> ([f64; 2], [f64; 3], SurfaceEval) {
        let (f, j, e) = self.offset(uv);
        let g = [
            j[0][0] * f[0] + j[1][0] * f[1],
            j[0][1] * f[0] + j[1][1] * f[1],
        ];
        let grads = self.cut.gradients(e.point);
        let second = |v: Vec3| grads.map(|n| n.dot(&v));
        let (fuu, fuv, fvv) = (second(e.duu), second(e.duv), second(e.dvv));
        let jtj = gram(j);
        let h = [
            jtj[0] + f[0] * fuu[0] + f[1] * fuu[1],
            jtj[1] + f[0] * fuv[0] + f[1] * fuv[1],
            jtj[2] + f[0] * fvv[0] + f[1] * fvv[1],
        ];
        (g, h, e)
    }

    /// The critical point of the distance that Newton's iteration on its
    /// gradient reaches from `start`, held inside `bounds`. Near a fold
    /// the Hessian is stiff across the fold and soft along it, and a step
    /// along the fold bends the valley's floor out from under the linear
    /// model by more than the soft gradient it was taken for: so each
    /// step ([`pseudo_newton`], halved until it shrinks the gradient) is
    /// followed by Newton's steps along the stiff direction alone, back
    /// onto the floor, before the gradient is compared.
    fn critical(&self, start: [f64; 2], bounds: [[f64; 2]; 2]) -> [f64; 2] {
        let hold = |x: [f64; 2]| [0, 1].map(|k| x[k].clamp(bounds[k][0], bounds[k][1]));
        // Newton's steps along the eigenvector of the larger eigenvalue.
        let floor = |mut x: [f64; 2]| {
            let (mut g, mut h, _) = self.second_order(x);
            for _ in 0..DAMPINGS {
                let [a, b, c] = h;
                let mean = 0.5 * (a + c);
                let radius = (0.25 * (a - c) * (a - c) + b * b).sqrt();
                let stiff = if mean >= 0.0 {
                    mean + radius
                } else {
                    mean - radius
                };
                let Some(q) = eigenvector(h, stiff).filter(|_| stiff != 0.0) else {
                    break;
                };
                let along = q[0] * g[0] + q[1] * g[1];
                let y = hold([x[0] - along / stiff * q[0], x[1] - along / stiff * q[1]]);
                let (gy, hy, _) = self.second_order(y);
                if (q[0] * gy[0] + q[1] * gy[1]).abs() >= along.abs() {
                    break;
                }
                (x, g, h) = (y, gy, hy);
            }
            (x, g, h)
        };
        let (mut x, mut g, mut h) = floor(hold(start));
        for _ in 0..DESCENT_STEPS {
            let step = pseudo_newton(h, g);
            if !step.iter().all(|s| s.is_finite()) || step == [0.0, 0.0] {
                break;
            }
            let mut alpha = 1.0;
            let mut moved = None;
            for _ in 0..DAMPINGS {
                let next = floor(hold([x[0] + alpha * step[0], x[1] + alpha * step[1]]));
                if norm(next.1) < norm(g) {
                    moved = Some(next);
                    break;
                }
                alpha *= 0.5;
            }
            let Some((y, gy, hy)) = moved else {
                break;
            };
            let e = self.surface.eval(x[0], x[1]);
            let travelled = travel(&e, x, y);
            (x, g, h) = (y, gy, hy);
            if arris_math::is_negligible(travelled, self.reach) {
                break;
            }
        }
        x
    }

    /// Whether the gradient of the distance at `uv` is zero to the
    /// rounding of the offsets it is made of.
    fn is_critical(&self, uv: [f64; 2]) -> bool {
        let (f, j, _) = self.offset(uv);
        let g = [
            j[0][0] * f[0] + j[1][0] * f[1],
            j[0][1] * f[0] + j[1][1] * f[1],
        ];
        let size = real::hypot(real::hypot(j[0][0], j[0][1]), real::hypot(j[1][0], j[1][1]));
        norm(g) <= self.rounding() * size
    }

    /// The bounds a parameter is held in on the whole surface: its
    /// domain, a closure further either way, which `eval` wraps.
    fn surface_bounds(&self) -> [[f64; 2]; 2] {
        [0, 1].map(|k| match self.closure[k] {
            Some(c) => [self.domain[k].lo() - c, self.domain[k].hi() + c],
            None => [self.domain[k].lo(), self.domain[k].hi()],
        })
    }

    /// The extremum of the distance along the line beside the meeting
    /// at `uv`, from the second-order model of the module docs: where it
    /// is, in `(u, v)` and along the line from the meeting, and its depth.
    /// `None` where the line crosses the tangent plane and the surface
    /// has no curvature along it there, or where the surface has no
    /// normal.
    fn extremum(&self, uv: [f64; 2]) -> Option<([f64; 2], f64, f64)> {
        let e = self.surface.eval(uv[0], uv[1]);
        let n = e.du.cross(&e.dv).try_normalize(0.0)?;
        let direction = self.cut.direction(e.point)?;
        let sin = direction.dot(&n);
        // The curve's direction in the tangent plane, in `(u, v)`.
        let along = direction - n * sin;
        let g = [e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv)];
        let rhs = [e.du.dot(&along), e.dv.dot(&along)];
        let det = g[0] * g[2] - g[1] * g[1];
        if !(det.is_finite() && det > 0.0) {
            return None;
        }
        let ab = [
            (g[2] * rhs[0] - g[1] * rhs[1]) / det,
            (g[0] * rhs[1] - g[1] * rhs[0]) / det,
        ];
        // The sag of the surface along the curve less the curve's own
        // bend towards it.
        let second = n.dot(
            &(e.duu * (ab[0] * ab[0]) + e.duv * (2.0 * ab[0] * ab[1]) + e.dvv * (ab[1] * ab[1])),
        ) - self.cut.bend(e.point).dot(&n);
        // A surface not curved along the line, to the rounding of its
        // sag across the control net, has no extremum beside a crossing:
        // the line lies on it, parallel within `tol.angular`, or crosses
        // it once.
        if second.abs() <= self.flat_curvature() * along.norm_squared() {
            return (sin.abs() <= self.angular).then_some((uv, 0.0, 0.0));
        }
        let s = sin / second;
        Some((
            [uv[0] + s * ab[0], uv[1] + s * ab[1]],
            s,
            sin * sin / (2.0 * second.abs()),
        ))
    }

    /// Whether the line is tangent to the surface at the meeting at `uv`,
    /// to the rounding a double zero is located to: off it by `δ` along
    /// the surface, the line meets the tangent plane at the angle
    /// `κ δ` with the surface's largest normal curvature `κ`, and `δ` is
    /// `√(ε / κ)` for the rounding `ε` of the offsets — so the angle's
    /// square over `2κ` is within that rounding. A surface not curved
    /// there is tangent to the line parallel to it within `tol.angular`.
    fn is_double(&self, uv: [f64; 2]) -> bool {
        let e = self.surface.eval(uv[0], uv[1]);
        let Some(n) = e.du.cross(&e.dv).try_normalize(0.0) else {
            return false;
        };
        let Some(direction) = self.cut.direction(e.point) else {
            return false;
        };
        let sin = direction.dot(&n);
        let Some([l, m, k]) = self.form(uv) else {
            return sin.abs() <= self.angular;
        };
        let largest = 0.5 * (l + k).abs() + (0.25 * (l - k) * (l - k) + m * m).sqrt();
        if largest <= self.flat_curvature() {
            return sin.abs() <= self.angular;
        }
        sin * sin / (2.0 * largest) <= self.rounding()
    }

    /// A curvature whose sag across the control net is within the
    /// rounding of the coordinates: none, as far as `f64` can tell.
    fn flat_curvature(&self) -> f64 {
        2.0 * self.rounding() / (self.extent * self.extent)
    }

    /// Whether the line stays within the tube from its point at `from` to
    /// its point at `to`, measured at [`JOIN_SAMPLES`] points strictly
    /// between by the surface's own projection: a point it does not
    /// decide is not within.
    fn joined(&self, from: f64, to: f64) -> bool {
        let span = self.cut.step(from, to);
        (1..=JOIN_SAMPLES).all(|i| {
            let t = from + span * i as f64 / (JOIN_SAMPLES + 1) as f64;
            self.surface
                .project(self.cut.point_at(t))
                .is_ok_and(|p| p.distance <= self.tau)
        })
    }

    /// The touch beside the meeting at `uv` whose model puts the extremum
    /// of the distance `along` the line from it, at `guess` on the
    /// surface, within the tube — or `None` where the surface says the
    /// meeting is a crossing. Newton's iteration on the gradient from the
    /// surface's nearest point to the line's point there finds the
    /// extremum: a touch where it is off the surface within the tube and
    /// the line runs to it from the meeting without leaving the tube; the
    /// meeting itself, a double zero, where it converges back onto it
    /// with the line tangent there. Where it finds neither — a valley so
    /// flat along the line that the iteration cannot place its floor —
    /// the line itself decides: within the tube from the meeting to the
    /// model's extremum, the touch is the surface's nearest point there.
    fn extremum_beside(&self, uv: [f64; 2], guess: [f64; 2], along: f64) -> Option<Touch> {
        let at = self.cut.position(self.surface.eval(uv[0], uv[1]).point);
        let there = self.cut.point_at(self.cut.shifted(at, along));
        let nearest = self.surface.project(there).ok();
        let start = nearest.map_or(guess, |p| [p.uv.x, p.uv.y]);
        let found = self.critical(start, self.surface_bounds());
        let distance = self.distance(found);
        if self.is_critical(found) && self.settle(found).is_some() {
            if distance <= self.rounding() {
                return self.is_double(uv).then(|| Touch {
                    uv,
                    distance: self.distance(uv),
                });
            }
            if distance <= self.tau {
                let to = self
                    .cut
                    .position(self.surface.eval(found[0], found[1]).point);
                return self.joined(at, to).then_some(Touch {
                    uv: found,
                    distance,
                });
            }
            return None;
        }
        let p = nearest?;
        let uv = [p.uv.x, p.uv.y];
        (p.distance <= self.tau
            && self.settle(uv).is_some()
            && self.joined(at, self.cut.shifted(at, along)))
        .then_some(Touch {
            uv,
            distance: p.distance,
        })
    }

    /// The surface's second fundamental form at `uv` in the orthonormal
    /// basis of its tangent plane whose first axis is the line's
    /// direction there: `[L, M, N]` for `[[L, M], [M, N]]`, `L` the normal
    /// curvature along the line. `None` where the surface has no normal
    /// there or the line is along it.
    fn form(&self, uv: [f64; 2]) -> Option<[f64; 3]> {
        let e = self.surface.eval(uv[0], uv[1]);
        let n = e.du.cross(&e.dv).try_normalize(0.0)?;
        let direction = self.cut.direction(e.point)?;
        let along = (direction - n * direction.dot(&n)).try_normalize(0.0)?;
        let across = n.cross(&along);
        let g = [e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv)];
        let det = g[0] * g[2] - g[1] * g[1];
        if !(det.is_finite() && det > 0.0) {
            return None;
        }
        // The `(u, v)` of a tangent vector, and the form on it.
        let coords = |w: Vec3| {
            let r = [e.du.dot(&w), e.dv.dot(&w)];
            [
                (g[2] * r[0] - g[1] * r[1]) / det,
                (g[0] * r[1] - g[1] * r[0]) / det,
            ]
        };
        let form = |a: [f64; 2], b: [f64; 2]| {
            n.dot(
                &(e.duu * (a[0] * b[0])
                    + e.duv * (a[0] * b[1] + a[1] * b[0])
                    + e.dvv * (a[1] * b[1])),
            )
        };
        let (t, c) = (coords(along), coords(across));
        Some([
            form(t, t) - self.cut.bend(e.point).dot(&n),
            form(t, c),
            form(c, c),
        ])
    }

    /// The angle from the line's direction to the nearest direction of
    /// the tangent plane the surface does not curve along — a ruling's,
    /// or an asymptotic one — from the form's eigenvectors, which are
    /// exact to rounding where the eigenvalues are apart: one eigenvalue
    /// zero to rounding against the other is a parabolic point, whose
    /// direction is that eigenvector; two of opposite signs a hyperbolic
    /// one, whose two lie either side of the second eigenvector at
    /// `atan √(−λ₁ / λ₂)`. `None` at an elliptic point, curved every way.
    fn flat_angle(&self, [l, m, n]: [f64; 3]) -> Option<f64> {
        let mean = 0.5 * (l + n);
        let radius = (0.25 * (l - n) * (l - n) + m * m).sqrt();
        let (big, small) = if mean >= 0.0 {
            (mean + radius, mean - radius)
        } else {
            (mean - radius, mean + radius)
        };
        let axis = |lambda: f64| eigenvector([l, m, n], lambda).unwrap_or([1.0, 0.0]);
        let from_line = |v: [f64; 2]| {
            real::atan2(v[1], v[0])
                .abs()
                .min(real::atan2(-v[1], -v[0]).abs())
        };
        let flat = self.flat_curvature();
        if big.abs() <= flat {
            return Some(0.0);
        }
        if small.abs() <= flat {
            return Some(from_line(axis(small)));
        }
        if small * big > 0.0 {
            return None;
        }
        let [x, y] = axis(big);
        let spread = real::atan((-small / big).sqrt());
        let base = real::atan2(y, x);
        let angles = [base + spread, base - spread];
        Some(
            angles
                .iter()
                .map(|a| from_line([real::cos(*a), real::sin(*a)]))
                .fold(f64::INFINITY, f64::min),
        )
    }

    /// Half the length of the stretch of the line about a touch that is
    /// within the tube, to second order: along the line's direction the
    /// distance grows as `κ s² / 2` with the surface's normal curvature
    /// `κ` along it, from the touch's distance `δ`, so `√(2 (τ − δ) / κ)`;
    /// infinite where `κ` is zero.
    fn reach_within(&self, touch: &Touch, form: Option<[f64; 3]>) -> f64 {
        let kappa = form.map_or(f64::INFINITY, |f| f[0].abs());
        (2.0 * (self.tau - touch.distance).max(0.0) / kappa).sqrt()
    }

    /// Whether the line lies on the surface from a touch, as an analytic
    /// arm reads a line on a ruling: parallel, within `tol.angular`, to a
    /// direction the surface does not curve along there, and within the
    /// tube across the extent of the control net along the line.
    fn lies_along(&self, touch: &Touch) -> bool {
        // A conic lies on the surface or it does not: [`Self::on_surface`].
        let Cut::Line(line) = &self.cut else {
            return false;
        };
        let form = self.form(touch.uv);
        if form
            .and_then(|f| self.flat_angle(f))
            .is_none_or(|angle| angle > self.angular)
        {
            return false;
        }
        let (lo, hi) = self
            .surface
            .control_points()
            .iter()
            .map(|p| line.t(*p))
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), t| {
                (lo.min(t), hi.max(t))
            });
        self.reach_within(touch, form) >= hi - lo
    }

    /// `uv` in the surface's domain, as a projection reports it: a closed
    /// direction's parameter in `[lo, hi)`, an open one's held at an end
    /// it is past by no more than the rounding of the coordinates
    /// ([`LOCATION`]), and `None` beyond that.
    fn settle(&self, uv: [f64; 2]) -> Option<[f64; 2]> {
        let e = self.surface.eval(uv[0], uv[1]);
        let speed = [e.du.norm(), e.dv.norm()];
        let noise = LOCATION * self.reach;
        let mut out = uv;
        for k in 0..2 {
            let (lo, hi) = (self.domain[k].lo(), self.domain[k].hi());
            let negligible = |d: f64| arris_math::is_negligible(d * speed[k], noise);
            match self.closure[k] {
                Some(c) => {
                    let mut x = lo + (out[k] - lo).rem_euclid(c);
                    if x >= hi || negligible(hi - x) {
                        x = lo;
                    }
                    out[k] = x;
                }
                None if out[k] < lo => {
                    if !negligible(lo - out[k]) {
                        return None;
                    }
                    out[k] = lo;
                }
                None if out[k] > hi => {
                    if !negligible(out[k] - hi) {
                        return None;
                    }
                    out[k] = hi;
                }
                None => {}
            }
        }
        Some(out)
    }

    /// Whether a conic that meets the surface lies on it all round: every
    /// one of [`CONIC_SAMPLES`] points of it, evenly spread in its
    /// parameter, within the tube by the surface's own projection. A
    /// line lying on a surface is decided by the touches along it
    /// ([`Self::lies_along`]); a conic's curvature makes a sample the
    /// simpler test, and the isolation has met the surface somewhere on it.
    fn on_surface(&self, all: &Collected) -> bool {
        let Cut::Conic(conic) = &self.cut else {
            return false;
        };
        if all.meets.is_empty() && all.folds.is_empty() && all.poles.is_empty() {
            return false;
        }
        let curve = conic.curve();
        (0..CONIC_SAMPLES).all(|i| {
            let t = core::f64::consts::TAU * (i as f64 + 0.5) / CONIC_SAMPLES as f64;
            self.surface
                .project(curve.point(t))
                .is_ok_and(|p| p.distance <= self.tau)
        })
    }

    /// The crossings, the touches and the poles, as hits ascending by
    /// `t`; or `Coincident`.
    fn hits(&self, curve: &Curve, all: Collected) -> Result<CurveSurfaceIntersection, GeomError> {
        let point = |uv: [f64; 2]| self.surface.eval(uv[0], uv[1]).point;
        if self.on_surface(&all) {
            return Ok(CurveSurfaceIntersection::Coincident);
        }
        let same =
            |a: Point3, b: Point3| arris_math::is_negligible((a - b).norm(), LOCATION * self.reach);
        let mut poles: Vec<Pole> = Vec::new();
        for p in all.poles {
            match poles.iter_mut().find(|q| same(q.point, p.point)) {
                Some(q) => q.tangent |= p.tangent,
                None => poles.push(p),
            }
        }
        // Within the tube of a pole is the pole's, to the rounding the
        // pole was put in the tube with.
        let near_pole = |here: Point3| {
            poles
                .iter()
                .any(|p| (p.point - here).norm() <= self.tau + self.rounding())
        };
        // Each meeting's partner across the extremum beside it, twice as
        // far along the line: a descent from there finds it where a
        // region of near-zeros kept the isolation from it.
        let mut meets = all.meets;
        let partners: Vec<Found> = meets
            .iter()
            .filter_map(|m| {
                let (guess, _, _) = self.extremum(m.uv)?;
                let start = [0, 1].map(|k| 2.0 * guess[k] - m.uv[k]);
                let (uv, distance) = self.descend(start, self.surface_bounds());
                (distance <= self.rounding()).then_some(Found { uv, lo: uv, hi: uv })
            })
            .collect();
        meets.extend(partners);
        // A meeting found twice — by two patches on a knot line or a
        // seam, by two descents, which a crossing at a small angle sends
        // apart along the line by its rounding over the angle's sine — is
        // one; two nearer than the tolerance are one to it. The smaller
        // box stands for it.
        meets.sort_by(|a, b| area(a).total_cmp(&area(b)).then(order(a, b)));
        let mut kept: Vec<Found> = Vec::new();
        for m in meets {
            let here = point(m.uv);
            if !near_pole(here) && !kept.iter().any(|k| (point(k.uv) - here).norm() <= self.tau) {
                kept.push(m);
            }
        }
        // A meeting whose extremum of the distance is within the tube is
        // a touch there; the others are crossings.
        let mut touches: Vec<Touch> = all
            .folds
            .into_iter()
            .filter(|t| !near_pole(point(t.uv)))
            .collect();
        let mut crossings = Vec::new();
        for m in kept {
            let Some((guess, along, depth)) = self.extremum(m.uv) else {
                crossings.push(m.uv);
                continue;
            };
            if depth > self.tau {
                crossings.push(m.uv);
                continue;
            }
            match self.extremum_beside(m.uv, guess, along) {
                Some(touch) => touches.push(touch),
                None => crossings.push(m.uv),
            }
        }
        // Touches whose stretches of the line within the tube overlap are
        // one, at the nearest to the line: a line nearly along a ruling
        // has a long flat valley of them, whose place along it is
        // conditioned as the angle.
        touches.sort_by(|a, b| {
            a.distance
                .total_cmp(&b.distance)
                .then(a.uv[0].total_cmp(&b.uv[0]))
                .then(a.uv[1].total_cmp(&b.uv[1]))
        });
        let mut one: Vec<(Touch, f64, f64)> = Vec::new();
        for t in touches {
            let at = self.cut.position(point(t.uv));
            let half = self.reach_within(&t, self.form(t.uv));
            if !one
                .iter()
                .any(|(_, other, reach)| self.cut.gap(at, *other) <= half + reach)
            {
                one.push((t, at, half));
            }
        }
        let one: Vec<Touch> = one.into_iter().map(|(t, _, _)| t).collect();
        if one.iter().any(|t| self.lies_along(t)) {
            return Ok(CurveSurfaceIntersection::Coincident);
        }
        let mut out = Vec::new();
        let mut push = |uv: [f64; 2], tangent: bool| {
            let t = self.cut.position(point(uv));
            out.push(CurveSurfaceHit {
                t,
                uv: Point2::new(uv[0], uv[1]),
                point: curve.point(t),
                tangent,
            });
        };
        for uv in crossings {
            if let Some(uv) = self.settle(uv) {
                push(uv, false);
            }
        }
        for t in &one {
            if let Some(uv) = self.settle(t.uv) {
                push(uv, true);
            }
        }
        for p in &poles {
            push(p.uv, p.tangent);
        }
        out.sort_by(|a, b| {
            a.t.total_cmp(&b.t)
                .then(a.uv.x.total_cmp(&b.uv.x))
                .then(a.uv.y.total_cmp(&b.uv.y))
        });
        Ok(CurveSurfaceIntersection::Points(out))
    }
}

/// A found box as bounds to hold an iteration in.
fn bounds(f: &Found) -> [[f64; 2]; 2] {
    [[f.lo[0], f.hi[0]], [f.lo[1], f.hi[1]]]
}

/// `JᵀJ` as `[a, b, c]` for `[[a, b], [b, c]]`.
fn gram(j: [[f64; 2]; 2]) -> [f64; 3] {
    [
        j[0][0] * j[0][0] + j[1][0] * j[1][0],
        j[0][0] * j[0][1] + j[1][0] * j[1][1],
        j[0][1] * j[0][1] + j[1][1] * j[1][1],
    ]
}

/// How far the surface moved between two parameters, to first order.
fn travel(e: &SurfaceEval, x: [f64; 2], y: [f64; 2]) -> f64 {
    ((y[0] - x[0]) * e.du.norm()).abs() + ((y[1] - x[1]) * e.dv.norm()).abs()
}

/// The unit eigenvector of the symmetric `[[a, b], [b, c]]` for its
/// eigenvalue `lambda`, `None` where the matrix is a multiple of the
/// identity.
fn eigenvector([a, b, c]: [f64; 3], lambda: f64) -> Option<[f64; 2]> {
    let candidates = [[b, lambda - a], [lambda - c, b]];
    let pick = if norm(candidates[0]) >= norm(candidates[1]) {
        candidates[0]
    } else {
        candidates[1]
    };
    let size = norm(pick);
    (size > 0.0 && size.is_finite()).then(|| [pick[0] / size, pick[1] / size])
}

/// `−H⁺ g` for the symmetric `H = [[a, b], [b, c]]`: Newton's step along
/// each eigenvector of `H` whose eigenvalue is not zero to rounding
/// against the larger, and none along one that is — a direction the
/// gradient does not change along, as along a ruling parallel to the
/// line, where every point is as critical as its neighbour.
fn pseudo_newton([a, b, c]: [f64; 3], g: [f64; 2]) -> [f64; 2] {
    let mean = 0.5 * (a + c);
    let radius = (0.25 * (a - c) * (a - c) + b * b).sqrt();
    let values = [mean + radius, mean - radius];
    let top = values[0].abs().max(values[1].abs());
    let mut step = [0.0; 2];
    for (i, &lambda) in values.iter().enumerate() {
        if arris_math::is_negligible(lambda, top) {
            continue;
        }
        // Equal eigenvalues: the matrix is `λI`, and either axis is one.
        let axis = if i == 0 { [1.0, 0.0] } else { [0.0, 1.0] };
        let q = eigenvector([a, b, c], lambda).unwrap_or(axis);
        let along = -(q[0] * g[0] + q[1] * g[1]) / lambda;
        step[0] += along * q[0];
        step[1] += along * q[1];
    }
    step
}

fn norm(x: [f64; 2]) -> f64 {
    real::hypot(x[0], x[1])
}

fn area(f: &Found) -> f64 {
    (f.hi[0] - f.lo[0]) * (f.hi[1] - f.lo[1])
}

fn order(a: &Found, b: &Found) -> core::cmp::Ordering {
    a.uv[0]
        .total_cmp(&b.uv[0])
        .then(a.uv[1].total_cmp(&b.uv[1]))
}
