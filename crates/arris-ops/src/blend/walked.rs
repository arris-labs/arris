//! A walked stripe's spine and contacts (ADR-0062): the ball's centre runs
//! on the section of the two faces' offsets toward it — exact for an
//! analytic face (`Surface::offset`), fitted over the band the ball sweeps
//! for a NURBS one (`NurbsSurface::offset_fitted`) — traced by the
//! intersector's own tracers (`trace_section`). The stretch of the one
//! branch holding the ball's centre at both ends of the edge is the spine,
//! carried on past each end so that a face across can cut it, and every
//! sample of it is snapped by Newton onto the two exact distances from
//! the faces, so the ball touches both true faces wherever it is asked:
//! the offset fit and the section fit decide only which branch is walked
//! and where a sample starts (ADR-0062 §1). Where the faces turn tangent
//! at an end of the edge within its tolerance, the spine stops at the
//! ball the edge's point locates, the narrowest the exact faces allow
//! there, and the edge's own ball is kept apart for the assembly. The
//! stripe's
//! surface is skinned through the ball's exact arcs along the spine
//! (`fit_skin`), each arc two rational quadratic segments from the
//! contact on the first face to the contact on the second, so its
//! isocurves `u = 0` and `u = 1` are the contact curves.

use arris_geom::{
    Curve, GeomError, NurbsCurve, NurbsSurface, SECTION_FIT_FRACTION, Surface, SurfaceIntersection,
    SurfaceKind, fit_skin, intersect_surfaces, trace_section,
};
use arris_math::nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use arris_math::{
    Aabb, Interrupted, Interval, Meter, Point2, Point3, Tolerance, Vec3, is_negligible,
};

use super::traced::Piece;
use crate::body_view::tangent_normals;
use crate::error::{OpError, fault_of};

/// The share of the stripe's tolerance the fitted offset of a NURBS face
/// is held to (ADR-0062 §1): a quarter, the section fit's own share, so
/// that the traced spine starts every snap well inside its basin. A ratio
/// between two fits, not a tolerance.
const OFFSET_FIT_FRACTION: f64 = 0.25;

/// How far past each end of the edge the spine runs, in radii of arc
/// along it: a face across the end cuts the blend where the ball, rolled
/// on past the edge's end, still touches both faces, which is within one
/// radius of the end's centre for a face across square to the edge and
/// within `r·(1 + tan α)` for one turned by `α` from square — two radii
/// take a face across turned by up to 45°. A reach, not a tolerance.
pub(super) const REACH_RADII: f64 = 2.0;

/// The edge's samples at which the ball is found in its normal plane:
/// the two ends, the middle that decides which way round a loop the
/// stretch runs, and the samples between, whose contacts bound the band a
/// NURBS face's offset is fitted over. A sampling density, not a
/// tolerance.
pub(super) const EDGE_SAMPLES: usize = 16;

/// The most Newton steps a snap or a foot takes before it is given up, and
/// the most halvings of one step: the iteration starts within the fits'
/// quarter of the tolerance of its root and converges quadratically, so a
/// handful are taken where it converges at all. A structural bound that
/// makes the iteration terminate.
pub(super) const NEWTON_STEPS: usize = 32;

/// The multiple of [`arris_math::RELATIVE_ROUNDING`] of the coordinates'
/// magnitude at which a Newton step has stopped moving the point: a
/// projection onto a NURBS face carries the rounding of its own Newton
/// iteration into the step, a few ulps of the coordinates. A rounding
/// slack, not a tolerance.
pub(super) const STEP_ROUNDING: f64 = 16.0;

/// The marching steps across a piece's domain that carry the spine past
/// an end of the edge, before a bisection finds where the chord reaches
/// [`REACH_RADII`]. A sampling density, not a tolerance.
const REACH_STEPS: usize = 512;

/// The bisections of the step that crosses the reach: `2⁻⁶⁰` of a step is
/// below the rounding of any parameter in it.
const REACH_BISECTIONS: usize = 60;

/// The degree of a walked stripe's surface along its spine: a cubic, as
/// the fitted offset is (`OFFSET_FIT_DEGREE`), so the surface follows the
/// spine's curvature with a residual of fourth order per span. A
/// structural choice, not a tolerance.
pub(super) const SURFACE_DEGREE: usize = 3;

/// Why a walked stripe has no spine, for the caller to name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Refusal {
    /// The radius reaches a principal radius of a face on the ball's side
    /// — a NURBS face's offset folds, an analytic face's collapses: the
    /// blend's `BlendTooLarge`.
    TooLarge,
    /// Face `k` has no offset to roll on: a pole of a NURBS face inside
    /// the band the ball sweeps, or an elliptic cylinder, whose offset is
    /// no surface the tracers take.
    NoOffset(usize),
    /// The trace holds no single branch through the ball's centre at both
    /// ends of the edge, or a ball does not settle on both faces: the
    /// faces' pair, named by the caller; `why` says which.
    NoSpine { why: &'static str },
}

/// A spine that could not be built: a refusal the caller names, or an
/// error that is the operation's.
#[derive(Debug)]
pub(super) enum Walked {
    Refused(Refusal),
    Op(OpError),
}

impl From<OpError> for Walked {
    fn from(e: OpError) -> Self {
        Walked::Op(e)
    }
}

/// A fit or a construction that failed where the spine said it could
/// not: the operation's, a diverged surface fit among them (ADR-0062 §6).
impl From<GeomError> for Walked {
    fn from(e: GeomError) -> Self {
        Walked::Op(fault_of(e))
    }
}

impl From<Interrupted> for Walked {
    fn from(e: Interrupted) -> Self {
        Walked::Op(e.into())
    }
}

/// A geometric error of the tracer or a projection: the caller's
/// [`Refusal::NoSpine`] where the pose is one the trace does not decide,
/// the operation's own otherwise.
pub(super) fn no_spine_or_fault(e: GeomError) -> Walked {
    match e {
        GeomError::DegenerateSection { .. }
        | GeomError::Unsupported { .. }
        | GeomError::Ambiguous { .. } => Walked::Refused(Refusal::NoSpine {
            why: "the trace or a projection does not decide the pose",
        }),
        other => Walked::Op(fault_of(other)),
    }
}

/// The edge a walked stripe rolls along and the two faces it rolls on.
pub(super) struct Rolling<'a> {
    /// The two faces' surfaces.
    pub(super) surfaces: [&'a Surface; 2],
    /// Each face's offset toward the ball along its surface's own normal:
    /// `±r`, the sign the side of the face the ball is on.
    pub(super) distances: [f64; 2],
    /// The edge's curve, run from `range.lo()` to `range.hi()`.
    pub(super) edge: &'a Curve,
    pub(super) range: Interval,
    /// `true` when the edge is closed, its two ends one vertex.
    pub(super) closed: bool,
    /// What the faces are tangent within at an end (ADR-0045): the faces'
    /// tolerance, or the edge's where that is wider.
    pub(super) tangency: f64,
    /// The tolerance of the vertex at each end of the edge, `range.lo()`'s
    /// first: how far from its point the faces may meet there, so how far
    /// a vanishing end's tangency may lie from it.
    pub(super) ends: [f64; 2],
}

/// The ball at one place of the spine: its centre, and its contact with
/// each face with the contact's `(u, v)` on that face.
#[derive(Debug, Clone, Copy)]
pub(super) struct Ball {
    pub(super) centre: Point3,
    pub(super) contacts: [Point3; 2],
    pub(super) uv: [Point2; 2],
}

/// The nearest point of a face to a centre, with the face's normal there
/// turned toward the ball.
#[derive(Debug, Clone, Copy)]
struct Foot {
    uv: Point2,
    point: Point3,
    toward: Vec3,
}

/// The two faces as a ball meets them: each surface and the sign that
/// turns its normal toward the ball, and the radius.
struct Faces {
    surfaces: [Surface; 2],
    signs: [f64; 2],
    radius: f64,
}

impl Faces {
    /// The foot of `c` on face `k`: by the closed form on an analytic
    /// surface, by Newton from `seed` on a NURBS one — continued past its
    /// domain as its end spans carry on (ADR-0062 §3) — or from the global
    /// nearest point where there is no seed.
    fn foot(&self, k: usize, c: Point3, seed: Option<Point2>) -> Result<Foot, Walked> {
        let surface = &self.surfaces[k];
        let (uv, point) = foot_on(surface, c, seed)?;
        let n = surface
            .normal(uv.x, uv.y)
            .ok_or(Walked::Refused(Refusal::NoSpine {
                why: "no normal at a foot",
            }))?;
        Ok(Foot {
            uv,
            point,
            toward: self.signs[k] * n.into_inner(),
        })
    }

    /// The two distances' residuals at `c` toward the ball, and the
    /// plane's where `plane` is `(p, t)`, with the feet they are read at.
    fn residual(
        &self,
        c: Point3,
        seeds: [Option<Point2>; 2],
        plane: Option<(Point3, Vec3)>,
    ) -> Result<([Foot; 2], Vector3<f64>), Walked> {
        let feet = [self.foot(0, c, seeds[0])?, self.foot(1, c, seeds[1])?];
        let g = feet.map(|f| (c - f.point).dot(&f.toward) - self.radius);
        let h = plane.map_or(0.0, |(p, t)| (c - p).dot(&t));
        Ok((feet, Vector3::new(g[0], g[1], h)))
    }

    /// The ball whose centre is the root nearest `start` of the two exact
    /// distances `dist(c, Fₖ) = r` toward the ball, and of `(c − p)·t = 0`
    /// where `plane` is `(p, t)`: Newton's, the least step that meets the
    /// two distances where no plane holds the third coordinate (ADR-0062
    /// §1), halved until it shortens the residual. Where the faces meet at
    /// a shallow angle the root lies along a long valley across the spine,
    /// and the least step keeps the centre near `start` along it. `seeds`
    /// start the NURBS faces' feet.
    fn snap(
        &self,
        start: Point3,
        seeds: [Option<Point2>; 2],
        plane: Option<(Point3, Vec3)>,
    ) -> Result<Ball, Walked> {
        let unsettled = Walked::Refused(Refusal::NoSpine {
            why: "a ball does not settle on both faces",
        });
        let mut c = start;
        let (mut feet, mut f) = self.residual(c, seeds, plane)?;
        for _ in 0..NEWTON_STEPS {
            let scale = STEP_ROUNDING * (c.coords.norm() + self.radius);
            if is_negligible(f.norm(), scale) {
                break;
            }
            let [a, b] = feet.map(|f| f.toward);
            let step = match plane {
                Some((_, t)) => {
                    let j = Matrix3::from_rows(&[a.transpose(), b.transpose(), t.transpose()]);
                    j.lu().solve(&-f).ok_or(Walked::Refused(Refusal::NoSpine {
                        why: "the normal plane and the faces' normals are dependent",
                    }))?
                }
                None => {
                    // The least-norm step: `−Jᵀ (J Jᵀ)⁻¹ g`.
                    let jj = Matrix2::new(a.dot(&a), a.dot(&b), a.dot(&b), b.dot(&b));
                    let w = jj
                        .lu()
                        .solve(&Vector2::new(-f.x, -f.y))
                        .ok_or(Walked::Refused(Refusal::NoSpine {
                            why: "the faces' normals are parallel at a ball",
                        }))?;
                    a * w.x + b * w.y
                }
            };
            if is_negligible(step.norm(), scale) {
                break;
            }
            let mut alpha = 1.0;
            let mut moved = None;
            for _ in 0..NEWTON_STEPS {
                let next = c + alpha * step;
                let (feet, g) = self.residual(next, feet.map(|f| Some(f.uv)), plane)?;
                if g.norm() < f.norm() {
                    moved = Some((next, feet, g));
                    break;
                }
                alpha *= 0.5;
            }
            let Some(next) = moved else { break };
            (c, feet, f) = next;
        }
        if !is_negligible(f.norm(), STEP_ROUNDING * (c.coords.norm() + self.radius)) {
            return Err(unsettled);
        }
        Ok(Ball {
            centre: c,
            contacts: feet.map(|f| f.point),
            uv: feet.map(|f| f.uv),
        })
    }
}

/// The foot of `c` on `surface`: by the closed form on an analytic
/// surface, by Newton from `seed` on a NURBS one — continued past its
/// domain as its end spans carry on (ADR-0062 §3) — or from the global
/// nearest point where there is no seed.
pub(super) fn foot_on(
    surface: &Surface,
    c: Point3,
    seed: Option<Point2>,
) -> Result<(Point2, Point3), Walked> {
    if let Surface::Nurbs(s) = surface {
        let seed = match seed {
            Some(seed) => seed,
            None => s.project(c).map_err(no_spine_or_fault)?.uv,
        };
        Ok(nurbs_foot(s, c, seed))
    } else {
        let p = surface.project(c).map_err(no_spine_or_fault)?;
        Ok((p.uv, p.point))
    }
}

/// The local nearest point of `s` to `c` Newton's iteration on the
/// squared distance reaches from `seed`, unclamped: past the domain the
/// end spans' polynomials carry on, which is the face the ball rolls on
/// there (ADR-0062 §3). Gauss–Newton where the Hessian is not positive
/// definite.
fn nurbs_foot(s: &NurbsSurface, c: Point3, seed: Point2) -> (Point2, Point3) {
    let mut x = Vector2::new(seed.x, seed.y);
    let mut e = s.eval(x.x, x.y);
    for _ in 0..NEWTON_STEPS {
        let r = e.point - c;
        let g = Vector2::new(e.du.dot(&r), e.dv.dot(&r));
        let gauss = Matrix2::new(
            e.du.dot(&e.du),
            e.du.dot(&e.dv),
            e.du.dot(&e.dv),
            e.dv.dot(&e.dv),
        );
        let full = gauss + Matrix2::new(e.duu.dot(&r), e.duv.dot(&r), e.duv.dot(&r), e.dvv.dot(&r));
        let definite = full[(0, 0)] > 0.0 && full.determinant() > 0.0;
        let h = if definite { full } else { gauss };
        let Some(step) = h.lu().solve(&-g) else {
            break;
        };
        x += step;
        let moved = (step.x * e.du).norm() + (step.y * e.dv).norm();
        e = s.eval(x.x, x.y);
        if is_negligible(moved, STEP_ROUNDING * (c.coords.norm() + r.norm())) {
            break;
        }
    }
    (Point2::new(x.x, x.y), e.point)
}

/// The piece of a trace a spine runs on, owned.
#[derive(Debug, Clone)]
enum Owned {
    Branch(arris_geom::SectionBranch),
    Circle(Curve),
}

impl Owned {
    fn piece(&self) -> Piece<'_> {
        match self {
            Owned::Branch(b) => Piece::Branch(b),
            Owned::Circle(c) => Piece::Circle(c),
        }
    }
}

/// A walked stripe's spine: the stretch of one piece of the offsets'
/// section the ball's centre runs on, from the ball at the edge's start
/// to the ball at its end and on past each by [`REACH_RADII`], and the
/// faces it snaps every sample onto.
pub(super) struct Spine {
    faces: Faces,
    piece: Owned,
    /// The piece's parameters the spine covers, the reach past each end
    /// included.
    range: Interval,
    /// The piece's parameters of the ball at the edge's start and at its
    /// end: the edge runs the way `from` to `to` does, either way along
    /// the piece.
    from: f64,
    to: f64,
    /// The ball at `from` and at `to` where the faces turn tangent there
    /// within the edge's tolerance, on the faces' common normal at the
    /// edge's point. It is no ball of the surface: the exact ball at
    /// `from` or `to` may lie as far as `√(2r·τ)` from it ([`spine`]), its arc
    /// as wide as the faces' angle there (ADR-0062 §5).
    pinned: [Option<Ball>; 2],
    /// The ball at each of the edge's samples, which seed a NURBS face's
    /// feet.
    edge_balls: Vec<Ball>,
    /// `true` for a closed edge's spine, a loop of one period.
    closed: bool,
}

impl Spine {
    /// The piece's parameters the spine covers, the reach past each end
    /// included.
    #[cfg(test)]
    pub(super) fn range(&self) -> Interval {
        self.range
    }

    /// The piece's parameters of the ball at the edge's start and at its
    /// end.
    #[cfg(test)]
    pub(super) fn ends(&self) -> [f64; 2] {
        [self.from, self.to]
    }

    /// Whether the faces turn tangent at the edge's start and at its end
    /// within the edge's tolerance, the spine stopping there.
    pub(super) fn vanishing(&self) -> [bool; 2] {
        self.pinned.map(|b| b.is_some())
    }

    /// The ball on the faces' common normal at the edge's point, at each
    /// vanishing end: where the edge says the stripe narrows to, which the
    /// assembly joins to the exact ball there.
    #[cfg(test)]
    pub(super) fn pinned(&self) -> [Option<Ball>; 2] {
        self.pinned
    }

    /// The piece's period where it is a loop.
    #[cfg(test)]
    pub(super) fn period(&self) -> Option<f64> {
        self.piece.piece().period()
    }

    /// The traced centre at `t`, before it is snapped.
    pub(super) fn traced(&self, t: f64) -> Point3 {
        self.piece.piece().point(t)
    }

    /// The ball at `t` of the piece: its traced centre snapped onto the
    /// two exact distances from the faces, each contact the snapped
    /// centre's foot on its face (ADR-0062 §1), at a vanishing end too.
    /// The feet on a NURBS face start from the nearest of the edge's
    /// balls.
    pub(super) fn ball(&self, t: f64) -> Result<Ball, Walked> {
        let start = self.traced(t);
        let nearest = self.edge_balls.iter().min_by(|a, b| {
            (a.centre - start)
                .norm()
                .total_cmp(&(b.centre - start).norm())
        });
        let seeds = [0, 1].map(|k| nearest.map(|ball| ball.uv[k]));
        self.faces.snap(start, seeds, None)
    }

    /// The stripe's surface over the spine's whole range: skinned through
    /// the ball's arcs ([`arc`]) along the piece's parameter, within
    /// [`SECTION_FIT_FRACTION`] of `tol.linear` of every arc it is
    /// checked against (ADR-0062 §1). `u` runs from the contact on the
    /// first face (`u = 0`) to the one on the second (`u = 1`), `v` over
    /// [`Spine::range`] upward. At a vanishing end the isocurve there is
    /// the exact ball's arc, as narrow as the faces' angle there, not the
    /// edge's point ([`Spine::pinned`]).
    ///
    /// Errors: a ball the spine cannot snap ([`Walked::Refused`]); the
    /// fit diverging, a kernel finding ([`Walked::Op`]).
    #[cfg(test)]
    pub(super) fn surface(
        &self,
        tol: Tolerance,
        meter: &mut Meter<'_>,
    ) -> Result<NurbsSurface, Walked> {
        fit_skin(
            |t| arc(&self.ball(t)?),
            self.range,
            SURFACE_DEGREE,
            SECTION_FIT_FRACTION * tol.linear,
            meter,
        )
    }
}

/// A stripe's surface skinned so that its `v` runs the way the edge does
/// (ADR-0062): the spine's own parameter, or its mirror in the spine's
/// range where the spine runs against the edge.
pub(super) struct Skin {
    pub(super) surface: NurbsSurface,
    /// The surface's `v` at the ball of the edge's start, then of its end:
    /// the first is below the second.
    pub(super) ends: [f64; 2],
    /// `true` when the surface's `v` is the spine's mirrored parameter.
    reflected: bool,
    range: Interval,
}

impl Skin {
    /// A surface already running along the edge's own direction over
    /// `range`, `ends` the `v` of the edge's start and of its end: a
    /// walked chamfer's, which is no spine's mirror.
    pub(super) fn along_edge(surface: NurbsSurface, range: Interval, ends: [f64; 2]) -> Skin {
        Skin {
            surface,
            ends,
            reflected: false,
            range,
        }
    }

    /// The spine's parameter that the surface's `v` stands for.
    pub(super) fn spine_t(&self, v: f64) -> f64 {
        if self.reflected {
            self.range.lo() + self.range.hi() - v
        } else {
            v
        }
    }
}

impl Spine {
    /// The stripe's surface as [`Spine::surface`] fits it, with `v` running
    /// along the edge: its contact isocurves then have the edge's direction,
    /// the way a contact edge's curve must.
    pub(super) fn skin(&self, tol: Tolerance, meter: &mut Meter<'_>) -> Result<Skin, Walked> {
        let range = self.range;
        let reflected = self.from > self.to;
        let at = |v: f64| {
            if reflected {
                range.lo() + range.hi() - v
            } else {
                v
            }
        };
        let budget = SECTION_FIT_FRACTION * tol.linear;
        // The surface's `v` runs along the edge, so its lower end is the
        // edge's start and its upper end the edge's end.
        let [stop_lo, stop_hi] = self.vanishing();
        // A loop's two ends are one ball: found once, so that the
        // surface's first and last rows of control points are the same to
        // the last bit and the face is closed, not only fitted to be. At a
        // vanishing end whose ball touches both faces at one point within
        // the surface's share of the tolerance, the arc is that point, and
        // the surface's row there is collapsed: a pole (ADR-0062 §5).
        let section = |v: f64| {
            let v = if self.closed && v >= range.hi() {
                range.lo()
            } else {
                v
            };
            let ball = self.ball(at(v))?;
            let vanishes = (stop_lo && v == range.lo()) || (stop_hi && v == range.hi());
            if vanishes && (ball.contacts[0] - ball.contacts[1]).norm() <= budget {
                point_arc(ball.contacts[0])
            } else {
                arc(&ball)
            }
        };
        let surface = fit_skin(section, range, SURFACE_DEGREE, budget, meter)?;
        // A vanishing end is where the spine stops, the range's own end:
        // named exactly, so that its isocurve is the surface's end row.
        let ends = [
            if stop_lo { range.lo() } else { at(self.from) },
            if stop_hi { range.hi() } else { at(self.to) },
        ];
        Ok(Skin {
            surface,
            ends,
            reflected,
            range,
        })
    }
}

/// The ball's arc from its contact on the first face to its contact on
/// the second, about its centre, as two rational quadratic segments over
/// `[0, ½]` and `[½, 1]` meeting at the arc's middle: exact, its weights
/// `cos(θ/4)` for the arc's angle `θ`, the same knots for every ball. A
/// ball whose contacts are one point (a vanishing end) gives the arc
/// collapsed to it.
///
/// Errors: [`Refusal::NoSpine`] where the contacts lie opposite each
/// other across the centre, the faces facing each other with the ball
/// between, so no one arc joins them.
pub(super) fn arc(ball: &Ball) -> Result<NurbsCurve, Walked> {
    let c = ball.centre;
    let [a, b] = ball.contacts;
    let (ra, rb) = (a - c, b - c);
    let (la, lb) = (ra.norm(), rb.norm());
    let opposite = || {
        Walked::Refused(Refusal::NoSpine {
            why: "a ball's contacts lie opposite each other",
        })
    };
    let (ua, ub) = (ra / la, rb / lb);
    let middle = (ua + ub).try_normalize(0.0).ok_or_else(opposite)?;
    // Each segment's angle is half the arc's, `φ`; its middle control
    // point is where the tangents at its ends meet, `ρ / cos(φ/2)` out
    // along the bisector, and its weight `cos(φ/2)`.
    let rho = 0.5 * (la + lb);
    let m = c + rho * middle;
    let segment = |u: Vec3, r: f64| {
        let cos = u.dot(&middle).clamp(-1.0, 1.0);
        let corner = c + (u * r + middle * rho) / (1.0 + cos);
        (corner, (0.5 * (1.0 + cos)).sqrt())
    };
    let (pa, wa) = segment(ua, la);
    let (pb, wb) = segment(ub, lb);
    if !(wa > 0.0 && wb > 0.0) {
        return Err(opposite());
    }
    Ok(NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![a, pa, m, pb, b],
        vec![1.0, wa, 1.0, wb, 1.0],
    )?)
}

/// The ball's arc at a vanishing end whose contacts are one point `p`:
/// [`arc`]'s knots, weights and count, every control point `p`, so that it
/// skins compatibly with the arcs beside it and the surface's row there is
/// collapsed to `p`.
fn point_arc(p: Point3) -> Result<NurbsCurve, Walked> {
    Ok(NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0],
        vec![p; 5],
        vec![1.0; 5],
    )?)
}

/// The stripe's two contact curves, the isocurves `u = 0` on the first
/// face and `u = 1` on the second of a surface [`Spine::surface`] built:
/// its first and last rows of control points along `v`, exact, since its
/// `u` knots are clamped.
pub(super) fn contacts(surface: &NurbsSurface) -> Result<[NurbsCurve; 2], Walked> {
    let [n, m] = surface.counts();
    let [_, q] = surface.degree();
    let row = |i: usize| -> Result<NurbsCurve, Walked> {
        let points = surface.control_points()[i * m..(i + 1) * m].to_vec();
        let weights = surface.weights()[i * m..(i + 1) * m].to_vec();
        Ok(NurbsCurve::new(
            q,
            surface.knots()[1].to_vec(),
            points,
            weights,
        )?)
    };
    Ok([row(0)?, row(n - 1)?])
}

/// The spine of the stripe `rolling` asks for, its tolerance `tol.linear`.
///
/// The ball is first found in the edge's normal plane at each of its
/// [`EDGE_SAMPLES`] samples. Each NURBS face is offset over the band its
/// contacts span, widened by the reach, to [`OFFSET_FIT_FRACTION`] of the
/// tolerance; each analytic face exactly. The two offsets' section is
/// traced, and the one piece of it within `tol.linear` of the ball's
/// centre at both ends of the edge is the spine, the stretch between the
/// two that holds the middle ball, carried on past each end by
/// [`REACH_RADII`] where the piece goes on — never past a closed piece's
/// period, never past an open one's ends. At an end where the faces are
/// tangent within `rolling.tangency` (ADR-0045) the spine stops: the
/// ball there is pinned to the edge's point, on the faces' common normal,
/// held only to `√(2r·τ)` of the piece, `τ` the wider of the tangency
/// and the end vertex's tolerance (ADR-0062, step 10a), and located on
/// it; the
/// exact ball at that place is the snap of the piece's point, which the
/// pinned one may miss by as much ([`Spine::pinned`]).
///
/// Errors: [`Walked::Refused`] with the [`Refusal`] the caller names;
/// [`Walked::Op`] for an interruption or a fault, the fit of an offset
/// diverging among them (ADR-0062 §6).
pub(super) fn spine(
    rolling: &Rolling<'_>,
    tol: Tolerance,
    meter: &mut Meter<'_>,
) -> Result<Spine, Walked> {
    let radius = rolling.distances[0].abs();
    let faces = Faces {
        surfaces: rolling.surfaces.map(Surface::clone),
        signs: rolling.distances.map(f64::signum),
        radius,
    };
    let reach = REACH_RADII * radius;
    // The ball at each of the edge's samples, in its normal plane, from
    // the tangent planes' ball and then the previous one.
    let mut edge_balls: Vec<Ball> = Vec::with_capacity(EDGE_SAMPLES + 1);
    let mut vanishing = [false; 2];
    for i in 0..=EDGE_SAMPLES {
        meter.tick()?;
        let e = rolling
            .edge
            .eval(rolling.range.lerp(i as f64 / EDGE_SAMPLES as f64));
        let along =
            e.d1.try_normalize(0.0)
                .ok_or(Walked::Refused(Refusal::NoSpine {
                    why: "the edge's tangent vanishes",
                }))?;
        let seeds = [0, 1].map(|k| edge_balls.last().map(|b| b.uv[k]));
        let feet = [
            faces.foot(0, e.point, seeds[0])?,
            faces.foot(1, e.point, seeds[1])?,
        ];
        let [a, b] = feet.map(|f| f.toward);
        let j = Matrix3::from_rows(&[a.transpose(), b.transpose(), along.transpose()]);
        let start = j
            .lu()
            .solve(&Vector3::new(radius, radius, 0.0))
            .map(|x| e.point + x)
            .ok_or(Walked::Refused(Refusal::NoSpine {
                why: "the tangent planes' ball has no centre",
            }))?;
        // Faces tangent at an end of the edge narrow the stripe to the
        // edge's point there: the ball touches both at it, its centre on
        // their common normal, whether or not the offsets still cross
        // within rounding of it. Tangent inside the edge, the stripe has
        // no corner to roll in there.
        if tangent_normals(a, b, radius, rolling.tangency, tol) {
            let end = match i {
                0 => 0,
                EDGE_SAMPLES => 1,
                _ => EDGE_SAMPLES + 1,
            };
            if rolling.closed || end > 1 {
                return Err(Walked::Refused(Refusal::NoSpine {
                    why: "the faces turn tangent inside the edge",
                }));
            }
            let normal = (a + b).normalize();
            vanishing[end] = true;
            edge_balls.push(Ball {
                centre: e.point + radius * normal,
                contacts: feet.map(|f| f.point),
                uv: feet.map(|f| f.uv),
            });
            continue;
        }
        let seeds = feet.map(|f| Some(f.uv));
        edge_balls.push(faces.snap(start, seeds, Some((e.point, along)))?);
    }
    // Each face's offset toward the ball.
    let mut offsets = Vec::with_capacity(2);
    for k in 0..2 {
        let d = rolling.distances[k];
        let offset = match rolling.surfaces[k] {
            Surface::Nurbs(s) => {
                let over = band(s, k, &edge_balls, reach + radius)
                    .ok_or(Walked::Refused(Refusal::NoOffset(k)))?;
                let fit = s
                    .offset_fitted(d, over, OFFSET_FIT_FRACTION * tol.linear, meter)
                    .map_err(|e| match e {
                        GeomError::OffsetFolds { .. } => Walked::Refused(Refusal::TooLarge),
                        GeomError::Degenerate { .. } => Walked::Refused(Refusal::NoOffset(k)),
                        // A diverged fit is the kernel's finding (ADR-0062 §6).
                        other => Walked::Op(fault_of(other)),
                    })?;
                Surface::Nurbs(fit)
            }
            analytic => analytic.offset(d).ok_or(Walked::Refused(
                if analytic.kind() == SurfaceKind::EllipticCylinder {
                    Refusal::NoOffset(k)
                } else {
                    Refusal::TooLarge
                },
            ))?,
        };
        offsets.push(offset);
    }
    let within = edge_balls
        .iter()
        .fold(Aabb::of_point(edge_balls[0].centre), |b, ball| {
            b.union(Aabb::of_point(ball.centre))
        })
        .inflated(reach + radius);
    // The pieces the offsets' section is made of: the tracers' where the
    // pair has no closed form, the intersector's exact curves where it has
    // (ADR-0062 §1: two analytic faces).
    let pieces: Vec<Owned> = match trace_section(&offsets[0], &offsets[1], &within, tol, meter) {
        Ok(trace) => (trace.circles().iter())
            .map(|c| Owned::Circle(c.circle.clone()))
            .chain(trace.branches().iter().cloned().map(Owned::Branch))
            .collect(),
        Err(GeomError::Unsupported { .. }) => {
            match intersect_surfaces(&offsets[0], &offsets[1], &within, tol, meter)
                .map_err(no_spine_or_fault)?
            {
                SurfaceIntersection::Meets { curves, .. } => {
                    curves.into_iter().map(|c| Owned::Circle(c.curve)).collect()
                }
                SurfaceIntersection::Empty | SurfaceIntersection::Coincident => {
                    return Err(Walked::Refused(Refusal::NoSpine {
                        why: "the offsets do not meet",
                    }));
                }
            }
        }
        Err(other) => return Err(no_spine_or_fault(other)),
    };
    let first = edge_balls[0].centre;
    let last = edge_balls[EDGE_SAMPLES].centre;
    let middle = edge_balls[EDGE_SAMPLES / 2].centre;
    // A vanishing end's ball is where the offsets touch, and touching
    // surfaces cross, if they do, wherever a gap of rounding or of the
    // fits closes: surfaces a gap `δ` apart that curve away from each
    // other by `1/r` meet `√(2rδ)` along from where they would touch. So
    // a vanishing end's ball is held to that reach of the gap the faces
    // may have there: the tangency they were found tangent within, or the
    // vertex's tolerance where the file wrote it wider than the edge's
    // (CTC-05's AP242 v84, 2.1e-2 against 2.4e-4).
    let near = [0, 1].map(|k| {
        if vanishing[k] {
            (2.0 * radius * rolling.tangency.max(rolling.ends[k])).sqrt()
        } else {
            tol.linear
        }
    });
    let piece = holding(pieces, [first, last], near)?;
    let p = piece.piece();
    let (t0, _) = p.locate(first)?;
    let (from, to) = if rolling.closed {
        let Some(period) = p.period() else {
            return Err(Walked::Refused(Refusal::NoSpine {
                why: "a closed edge's spine is no loop",
            }));
        };
        // The way round the loop the edge runs: the piece's tangent at the
        // start against the edge's.
        let along = rolling.edge.eval(rolling.range.lo()).d1;
        let h = period / REACH_STEPS as f64;
        if (p.point(t0 + h) - p.point(t0 - h)).dot(&along) >= 0.0 {
            (t0, t0 + period)
        } else {
            (t0 + period, t0)
        }
    } else {
        let (t1, _) = p.locate(last)?;
        let (tm, dm) = p.locate(middle)?;
        if dm > tol.linear {
            return Err(Walked::Refused(Refusal::NoSpine {
                why: "the middle ball is off the spine",
            }));
        }
        match p.period() {
            Some(period) => {
                let up = |t: f64, from: f64| from + (t - from).rem_euclid(period);
                if up(tm, t0) <= up(t1, t0) {
                    (t0, up(t1, t0))
                } else {
                    (up(t0, t1), t1)
                }
            }
            None => {
                if (tm - t0) * (tm - t1) > 0.0 {
                    return Err(Walked::Refused(Refusal::NoSpine {
                        why: "the middle ball is off the stretch between the ends",
                    }));
                }
                (t0, t1)
            }
        }
    };
    if from == to {
        return Err(Walked::Refused(Refusal::NoSpine {
            why: "the ends' balls are one point of the spine",
        }));
    }
    let range = if rolling.closed {
        Interval::new(from.min(to), from.max(to))
    } else {
        let up = to > from;
        let (lo, hi) = if up { (from, to) } else { (to, from) };
        // Whether the stripe narrows to a point at each, past which it
        // has nowhere to run.
        let (stop_lo, stop_hi) = if up {
            (vanishing[0], vanishing[1])
        } else {
            (vanishing[1], vanishing[0])
        };
        // How far each end may run on: to an open piece's ends, or round a
        // loop no farther than half the rest of its period each way.
        let (room_lo, room_hi) = match p.period() {
            Some(period) => {
                let rest = 0.5 * (period - (hi - lo)).max(0.0);
                (lo - rest, hi + rest)
            }
            None => {
                // A line or an unbounded curve runs on without end: as far
                // as the reach needs.
                let d = p.domain();
                (d.lo().max(lo - 4.0 * reach), d.hi().min(hi + 4.0 * reach))
            }
        };
        let lo = if stop_lo {
            lo
        } else {
            reach_past(&p, lo, room_lo, reach)
        };
        let hi = if stop_hi {
            hi
        } else {
            reach_past(&p, hi, room_hi, reach)
        };
        Interval::new(lo, hi)
    }
    .map_err(|_| {
        Walked::Refused(Refusal::NoSpine {
            why: "the spine's stretch is empty",
        })
    })?;
    let pinned = [0, EDGE_SAMPLES]
        .map(|i| edge_balls[i])
        .map(Some)
        .into_iter()
        .zip(vanishing)
        .map(|(ball, v)| ball.filter(|_| v))
        .collect::<Vec<_>>();
    Ok(Spine {
        faces,
        piece,
        range,
        from,
        to,
        pinned: [pinned[0], pinned[1]],
        edge_balls,
        closed: rolling.closed,
    })
}

/// The parameter box of NURBS face `k` its contacts at the edge's balls
/// span, widened by `pad` of arc each way at the slowest the face's
/// parameters run there; a closed direction it would wrap round is the
/// domain's whole period. `None` where a parameter does not move the
/// face at a contact, a pole the offset is refused at anyway.
fn band(s: &NurbsSurface, k: usize, balls: &[Ball], pad: f64) -> Option<[Interval; 2]> {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    let mut speed = [f64::INFINITY; 2];
    for ball in balls {
        let uv = ball.uv[k];
        let e = s.eval(uv.x, uv.y);
        for (d, (x, v)) in [(uv.x, e.du), (uv.y, e.dv)].into_iter().enumerate() {
            lo[d] = lo[d].min(x);
            hi[d] = hi[d].max(x);
            speed[d] = speed[d].min(v.norm());
        }
    }
    let domain = s.domain();
    let closure = s.closure();
    let mut over = [Interval::UNIT; 2];
    for d in 0..2 {
        if speed[d].is_nan() || speed[d] <= 0.0 {
            return None;
        }
        let widen = pad / speed[d];
        let (a, b) = (lo[d] - widen, hi[d] + widen);
        over[d] = match closure[d] {
            Some(period) if b - a >= period => domain[d],
            _ => Interval::new(a, b).ok()?,
        };
    }
    Some(over)
}

/// The one of `pieces` within `near` of both `points`, the ball's
/// centres at the edge's ends: refused where none is, or more than one.
fn holding(pieces: Vec<Owned>, points: [Point3; 2], near: [f64; 2]) -> Result<Owned, Walked> {
    let mut found = None;
    for piece in pieces {
        let p = piece.piece();
        let holds = |k: usize| -> Result<bool, Walked> { Ok(p.locate(points[k])?.1 <= near[k]) };
        if holds(0)? && holds(1)? && found.replace(piece).is_some() {
            return Err(Walked::Refused(Refusal::NoSpine {
                why: "two pieces hold the ends' balls",
            }));
        }
    }
    found.ok_or(Walked::Refused(Refusal::NoSpine {
        why: "no piece holds the ends' balls",
    }))
}

/// The parameter of `piece` past `end`, toward `room`, at which the chord
/// from `end` first reaches `reach`; `room` itself where it never does.
fn reach_past(piece: &Piece<'_>, end: f64, room: f64, reach: f64) -> f64 {
    let at = piece.point(end);
    let far = |t: f64| (piece.point(t) - at).norm() >= reach;
    let h = (room - end) / REACH_STEPS as f64;
    if h == 0.0 {
        return end;
    }
    let mut near = end;
    for i in 1..=REACH_STEPS {
        let t = end + i as f64 * h;
        if far(t) {
            let mut bracket = (near, t);
            for _ in 0..REACH_BISECTIONS {
                let mid = 0.5 * (bracket.0 + bracket.1);
                if far(mid) {
                    bracket.1 = mid;
                } else {
                    bracket.0 = mid;
                }
            }
            return bracket.1;
        }
        near = t;
    }
    room
}

#[cfg(test)]
mod tests {
    use core::f64::consts::TAU;

    use arris_debug::prop::{finite_f64, frame_in};
    use arris_debug::prop_shards;
    use arris_math::nalgebra::{Unit, UnitQuaternion};
    use arris_math::{Frame, Precision};
    use proptest::prelude::*;

    use super::*;

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    /// The poses a configuration is built in: at the origin, and turned
    /// and moved a hundred of its sizes away.
    fn poses() -> [Frame; 2] {
        let turn =
            UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vec3::new(0.3, -0.5, 0.8)), 1.1);
        [
            Frame::world(),
            Frame::from_rotation(Point3::new(120.0, -75.0, 40.0), &turn),
        ]
    }

    /// A frame of `pose`'s coordinates: `origin`, `z` and `x` given in them.
    fn framed(pose: &Frame, origin: Point3, z: Vec3, x: Vec3) -> Frame {
        Frame::new(
            pose.to_world(origin),
            pose.vec_to_world(z),
            pose.vec_to_world(x),
        )
        .unwrap()
    }

    /// Which face of a pair is replaced by its exact NURBS twin.
    #[derive(Debug, Clone, Copy)]
    enum Twin {
        Plane,
        Cylinder,
    }

    /// A plane and a cylinder meeting square along `edge`, with the ball's
    /// exact centre and contacts in the pose's coordinates as functions of
    /// the edge's parameter: what the table's closed form puts there.
    struct Pair {
        surfaces: [Surface; 2],
        distances: [f64; 2],
        edge: Curve,
        range: Interval,
        closed: bool,
        /// The closed form's ball: centre, contact on the plane, contact
        /// on the cylinder, in world coordinates, at a point of the
        /// spine's locus nearest a given world point.
        exact: Box<dyn Fn(Point3) -> [Point3; 3]>,
    }

    /// The plate `z ≤ 0` and a cylinder of radius `big` lying along `x`
    /// with its axis in the plate's top: a rod on the plate, its foot
    /// filleted concave (`s = 1`), or a channel cut in it, its rim filleted
    /// convex (`s = −1`). The edge is the ruling `y = big`, `z = 0`, run
    /// over `length` from `x = 0`; the spine the line `y = √((R+r)² − r²)`,
    /// `z = s·r`.
    fn ruling(pose: Frame, big: f64, r: f64, s: f64, length: f64, twin: Twin) -> Pair {
        let plane_frame = framed(&pose, Point3::origin(), Vec3::z(), Vec3::x());
        let axis = framed(&pose, Point3::origin(), Vec3::x(), Vec3::y());
        let plane = Surface::Plane { frame: plane_frame };
        let cylinder = Surface::Cylinder {
            frame: axis,
            radius: big,
        };
        let margin = length + 4.0 * (big + r);
        let span = |lo: f64, hi: f64| Interval::new(lo, hi).unwrap();
        let surfaces = match twin {
            Twin::Plane => [
                Surface::Nurbs(
                    plane
                        .to_nurbs([span(-margin, margin), span(-margin, margin)])
                        .unwrap(),
                ),
                cylinder,
            ],
            Twin::Cylinder => [
                plane,
                Surface::Nurbs(
                    cylinder
                        .to_nurbs([span(-1.2, 1.2), span(-margin, margin)])
                        .unwrap(),
                ),
            ],
        };
        let edge = Curve::Line {
            origin: pose.to_world(Point3::new(0.0, big, 0.0)),
            direction: Unit::new_normalize(pose.vec_to_world(Vec3::x())),
        };
        let y = ((big + r) * (big + r) - r * r).sqrt();
        let exact = move |q: Point3| {
            let x = pose.to_local(q).x;
            let centre = Point3::new(x, y, s * r);
            let on_cylinder =
                Point3::new(x, 0.0, 0.0) + (centre - Point3::new(x, 0.0, 0.0)) * (big / (big + r));
            [centre, Point3::new(x, y, 0.0), on_cylinder].map(|p| pose.to_world(p))
        };
        Pair {
            surfaces,
            distances: [s * r, r],
            edge,
            range: span(0.0, length),
            closed: false,
            exact: Box::new(exact),
        }
    }

    /// The plate `z ≤ 0` and a cylinder of radius `big` about `z`: a boss
    /// standing on it, its foot filleted concave (`s = 1`), or a hole
    /// through it, its rim filleted convex (`s = −1`). The edge is the
    /// circle `ρ = big`, `z = 0`, over `arc` of its angle (closed when
    /// that is the whole turn); the spine the circle `ρ = R + r`,
    /// `z = s·r`.
    fn circle(pose: Frame, big: f64, r: f64, s: f64, arc: Interval, twin: Twin) -> Pair {
        let frame = framed(&pose, Point3::origin(), Vec3::z(), Vec3::x());
        let plane = Surface::Plane { frame };
        let cylinder = Surface::Cylinder { frame, radius: big };
        let margin = 2.0 * (big + 2.0 * r);
        let span = |lo: f64, hi: f64| Interval::new(lo, hi).unwrap();
        let surfaces = match twin {
            Twin::Plane => [
                Surface::Nurbs(
                    plane
                        .to_nurbs([span(-margin, margin), span(-margin, margin)])
                        .unwrap(),
                ),
                cylinder,
            ],
            Twin::Cylinder => [
                plane,
                Surface::Nurbs(
                    cylinder
                        .to_nurbs([span(0.0, TAU), span(-4.0 * r, 4.0 * r)])
                        .unwrap(),
                ),
            ],
        };
        let closed = arc.length() >= TAU;
        let exact = move |q: Point3| {
            let local = pose.to_local(q);
            let out = Vec3::new(local.x, local.y, 0.0).normalize();
            let centre = Point3::origin() + out * (big + r) + Vec3::z() * (s * r);
            let on_plane = Point3::origin() + out * (big + r);
            let on_cylinder = Point3::origin() + out * big + Vec3::z() * (s * r);
            [centre, on_plane, on_cylinder].map(|p| pose.to_world(p))
        };
        Pair {
            surfaces,
            distances: [s * r, r],
            edge: Curve::Circle { frame, radius: big },
            range: arc,
            closed,
            exact: Box::new(exact),
        }
    }

    /// How far the snap moved the traced centre at `t` to `ball`, times
    /// the sine of half the angle the faces meet at there: the offset fit
    /// and the section fit each put the trace within a quarter of the
    /// tolerance of its offsets, which meet at that angle, so a snap
    /// inside its basin moves it no more than half the tolerance over that
    /// sine (ADR-0062 §1).
    fn across(spine: &Spine, t: f64, ball: &Ball, r: f64) -> f64 {
        let sine = (ball.contacts[0] - ball.contacts[1]).norm() / (2.0 * r);
        (spine.traced(t) - ball.centre).norm() * sine
    }

    /// What a test asks of a spine: every one of its samples snapped onto
    /// the closed form's locus with the closed form's contacts, to
    /// rounding at its distance from the origin, from a traced centre
    /// within its fits' reach ([`across`]); the balls at the edge's ends on its ends; and
    /// the stretch running on by the reach past each end of an open edge,
    /// a closed edge's loop whole.
    fn holds(pair: &Pair, spine: &Spine, r: f64) {
        let samples = 64;
        let slack = |p: Point3| 1e-11 * (1.0 + p.coords.norm());
        let range = spine.range();
        for i in 0..=samples {
            let t = range.lerp(i as f64 / samples as f64);
            let ball = spine
                .ball(t)
                .unwrap_or_else(|e| panic!("sample {i}: {e:?}"));
            let [centre, on_plane, on_cylinder] = (pair.exact)(ball.centre);
            let c = ball.centre;
            assert!(
                (c - centre).norm() <= slack(c),
                "centre {i}: {:e}",
                (c - centre).norm()
            );
            for (k, want) in [(0, on_plane), (1, on_cylinder)] {
                let got = ball.contacts[k];
                assert!(
                    (got - want).norm() <= slack(c),
                    "contact {k} at {i}: {:e}",
                    (got - want).norm()
                );
            }
            let drift = across(spine, t, &ball, r);
            assert!(drift <= 0.5 * tol().linear, "drift {drift:e} at {i}");
        }
        let [from, to] = spine.ends();
        for (t, at) in [(from, pair.range.lo()), (to, pair.range.hi())] {
            let want = (pair.exact)(pair.edge.point(at))[0];
            assert!((spine.traced(t) - want).norm() <= tol().linear);
        }
        if pair.closed {
            assert!((range.length() - (to - from).abs()).abs() <= 1e-12 * range.length());
            return;
        }
        // Round a loop the reach may run out of room: half the rest of
        // the period each way.
        let room = spine
            .period()
            .is_some_and(|p| (range.length() - p).abs() <= 1e-9 * p);
        let reach = REACH_RADII * r;
        for (end, past) in [(from.min(to), range.lo()), (from.max(to), range.hi())] {
            let chord = (spine.traced(past) - spine.traced(end)).norm();
            let off = chord - reach;
            assert!(off <= 1e-9 * (1.0 + r), "reach {chord} for {reach}");
            assert!(
                room || off >= -1e-9 * (1.0 + r),
                "reach {chord} for {reach}"
            );
        }
    }

    /// What a test asks of a stripe's surface over a twin pair: on a grid
    /// denser than the fit's checks, every point within the surface's
    /// share of the tolerance of the closed form's ball, the cylinder or
    /// torus the table builds (ADR-0062 §1), and each contact curve within
    /// it of the closed form's contact. Returns the surface's spans along
    /// the spine.
    fn skinned(pair: &Pair, spine: &Spine, r: f64) -> usize {
        let surface = spine
            .surface(tol(), &mut Meter::default())
            .unwrap_or_else(|e| panic!("{e:?}"));
        let budget = SECTION_FIT_FRACTION * tol().linear;
        let range = spine.range();
        assert_eq!(surface.domain()[1], range);
        let curves = contacts(&surface).unwrap();
        let (us, vs) = (24, 211);
        for j in 0..=vs {
            let v = range.lerp(j as f64 / vs as f64);
            for i in 0..=us {
                let p = surface.eval(i as f64 / us as f64, v).point;
                let [centre, _, _] = (pair.exact)(p);
                let off = (p - centre).norm() - r;
                assert!(off.abs() <= budget, "radial {off:e} at ({i}, {j})");
            }
            for (k, curve) in curves.iter().enumerate() {
                let q = curve.eval(v).point;
                let want = (pair.exact)(q)[1 + k];
                let off = (q - want).norm();
                assert!(off <= budget, "contact {k} off by {off:e} at {j}");
                let on_surface = surface.eval(k as f64, v).point;
                assert!((q - on_surface).norm() <= 1e-12 * (1.0 + q.coords.norm()));
            }
        }
        surface.knots()[1]
            .windows(2)
            .filter(|w| w[1] > w[0])
            .count()
    }

    fn walk(pair: &Pair) -> Spine {
        let rolling = Rolling {
            surfaces: [&pair.surfaces[0], &pair.surfaces[1]],
            distances: pair.distances,
            edge: &pair.edge,
            range: pair.range,
            closed: pair.closed,
            tangency: tol().linear,
            ends: [tol().linear; 2],
        };
        spine(&rolling, tol(), &mut Meter::default()).unwrap_or_else(|e| panic!("{e:?}"))
    }

    /// The twin of a plane or a cylinder against the other along a ruling,
    /// concave and convex: the spine on the closed form's axis line, the
    /// contacts on its contact lines, the surface on its cylinder.
    #[test]
    fn a_twin_against_a_cylinder_along_a_ruling_walks_and_skins_the_closed_forms_cylinder() {
        for pose in poses() {
            for (big, r) in [(2.0, 0.5), (1.0, 0.1), (5.0, 1.5)] {
                for s in [1.0, -1.0] {
                    for twin in [Twin::Plane, Twin::Cylinder] {
                        let pair = ruling(pose, big, r, s, 3.0, twin);
                        let spine = walk(&pair);
                        holds(&pair, &spine, r);
                        let spans = skinned(&pair, &spine, r);
                        eprintln!("ruling R {big} r {r} s {s} {twin:?}: {spans} spans");
                    }
                }
            }
        }
    }

    /// The twin of a plane or a cylinder against the other along a
    /// circle, a boss's foot and a hole's rim, the whole circle and an
    /// arc of it: the spine on the closed form's circle, the surface on
    /// its torus.
    #[test]
    fn a_twin_against_a_cylinder_along_a_circle_walks_and_skins_the_closed_forms_torus() {
        let whole = Interval::new(0.0, TAU).unwrap();
        let arc = Interval::new(0.3, 2.4).unwrap();
        for pose in poses() {
            for (big, r) in [(2.0, 0.5), (1.0, 0.1)] {
                for s in [1.0, -1.0] {
                    for twin in [Twin::Plane, Twin::Cylinder] {
                        for range in [whole, arc] {
                            let pair = circle(pose, big, r, s, range, twin);
                            let spine = walk(&pair);
                            holds(&pair, &spine, r);
                            let spans = skinned(&pair, &spine, r);
                            eprintln!(
                                "circle R {big} r {r} s {s} {twin:?} over {range:?}: {spans} spans"
                            );
                        }
                    }
                }
            }
        }
    }

    /// A box whose top is a bicubic graph `z = h(x, y)` waving along
    /// `y`, its side `x = 0` meeting the top in an S-shaped edge, filleted
    /// convex: the surface within its share of the tolerance of the ball
    /// at every place of the spine, each contact curve on its face within
    /// it. The spans along the spine are printed for the plan.
    #[test]
    fn an_s_shaped_spine_on_a_bicubic_bump_skins_within_the_budget() {
        let r = 0.3;
        let degree = 3;
        // Clamped uniform cubic knots over `[a, b]` in `spans` spans, and
        // their Greville abscissae, at which the net puts its `x` and `y`
        // so that `x = u` and `y = v` exactly.
        let knots = |a: f64, b: f64, spans: usize| {
            let mut k = vec![a; degree];
            k.extend((0..=spans).map(|i| a + (b - a) * i as f64 / spans as f64));
            k.extend(vec![b; degree]);
            let greville: Vec<f64> = (0..k.len() - degree - 1)
                .map(|i| k[i + 1..=i + degree].iter().sum::<f64>() / degree as f64)
                .collect();
            (k, greville)
        };
        let (ku, xs) = knots(-3.0, 1.0, 6);
        let (kv, ys) = knots(-4.0, 4.0, 12);
        let h = |x: f64, y: f64| 0.5 * (0.8 * y).sin() + 0.1 * x;
        let mut net = Vec::new();
        for &x in &xs {
            for &y in &ys {
                net.push(Point3::new(x, y, h(x, y)));
            }
        }
        let weights = vec![1.0; net.len()];
        let top = NurbsSurface::new([degree, degree], [ku, kv], net, weights).unwrap();
        let side = Surface::Plane {
            frame: Frame::new(Point3::origin(), Vec3::x(), Vec3::y()).unwrap(),
        };
        let bump = Surface::Nurbs(top.clone());
        let range = Interval::new(-2.0, 2.0).unwrap();
        let iso = |t: f64| top.eval(0.0, t).point;
        let edge = Curve::Nurbs(
            arris_geom::fit_curve(
                iso,
                range,
                3,
                |t, q| (q - iso(t)).norm(),
                1e-12,
                &mut Meter::default(),
            )
            .unwrap(),
        );
        // The material is `x ≤ 0` below the top: both faces' normals point
        // out of it, and the ball is inside.
        let rolling = Rolling {
            surfaces: [&side, &bump],
            distances: [-r, -r],
            edge: &edge,
            range,
            closed: false,
            tangency: tol().linear,
            ends: [tol().linear; 2],
        };
        let spine =
            spine(&rolling, tol(), &mut Meter::default()).unwrap_or_else(|e| panic!("{e:?}"));
        let started = std::time::Instant::now();
        let surface = spine
            .surface(tol(), &mut Meter::default())
            .unwrap_or_else(|e| panic!("{e:?}"));
        let fitted = started.elapsed();
        let budget = SECTION_FIT_FRACTION * tol().linear;
        let curves = contacts(&surface).unwrap();
        let samples = 157;
        let mut spine_z = Vec::new();
        for j in 0..=samples {
            let v = spine.range().lerp(j as f64 / samples as f64);
            let ball = spine.ball(v).unwrap();
            spine_z.push(ball.centre.z);
            for i in 0..=16 {
                let p = surface.eval(i as f64 / 16.0, v).point;
                let off = (p - ball.centre).norm() - r;
                assert!(off.abs() <= budget, "radial {off:e} at ({i}, {j})");
            }
            let on_side = curves[0].eval(v).point;
            assert!(
                on_side.x.abs() <= budget,
                "side contact {:e} at {j}",
                on_side.x
            );
            let on_top = curves[1].eval(v).point;
            let off = top.project(on_top).unwrap().distance;
            assert!(off <= budget, "top contact {off:e} at {j}");
        }
        // The spine bends both ways: its height rises and falls back in
        // turn, an S.
        let bends = spine_z
            .windows(3)
            .map(|w| (w[2] - 2.0 * w[1] + w[0]).signum())
            .collect::<Vec<_>>();
        assert!(bends.windows(2).any(|w| w[0] != w[1]), "no inflection");
        let spans = surface.knots()[1]
            .windows(2)
            .filter(|w| w[1] > w[0])
            .count();
        eprintln!("S-shaped spine: {spans} spans along it, fitted in {fitted:?}");
    }

    prop_shards! {
        /// Random poses and sizes of the twin pairs, along a ruling and
        /// along an arc of a circle: spine and surface.
        twin_pairs [shard_0 shard_1 shard_2 shard_3] (
            (frame, big, ratio, convex, plane_twin, along_circle, arc)
        ) = (
            frame_in(100.0),
            finite_f64(0.5..=5.0),
            finite_f64(0.05..=0.6),
            any::<bool>(),
            any::<bool>(),
            any::<bool>(),
            finite_f64(0.5..=5.5),
        ) => {
            let r = ratio * big;
            let s = if convex { -1.0 } else { 1.0 };
            let twin = if plane_twin { Twin::Plane } else { Twin::Cylinder };
            let pair = if along_circle {
                circle(frame, big, r, s, Interval::new(0.2, 0.2 + arc).unwrap(), twin)
            } else {
                ruling(frame, big, r, s, arc, twin)
            };
            let spine = walk(&pair);
            holds(&pair, &spine, r);
            skinned(&pair, &spine, r);
            Ok(())
        }
    }

    /// NIST CTC-05's convex edge between its rational B-spline face f109
    /// and the torus f111 at the battery's radius (the plan's census): the
    /// spine traced over the NURBS face's offset in the ball's band, every
    /// sample snapped onto both faces at the radius, and the surface
    /// skinned within its share of the tolerance of every ball, its contact
    /// curves on their faces. At v84, where the faces are tangent within
    /// the edge's tolerance, the surface ends on the exact ball's arc,
    /// narrower than that tolerance and within `√(2r·τₑ)` of the edge's
    /// point. The times, the spans and that arc are printed for the plan.
    #[test]
    fn real_ctc_05s_nurbs_torus_edge_has_a_spine_and_a_surface() {
        use arris_debug::unmetered::step_read;
        use arris_io::step::ReadOptions;
        use arris_topo::Model;

        use crate::body_view::{BodyView, faces_tolerance};

        const PART: &str = "../../tests/fixtures/real/nist-ctc-05-ap203-torus-elbow-s5/nist_ctc_05_asme1_ap203-249.stp";
        const RADIUS: f64 = 0.8889999999999987;
        let at = Point3::new(150.98396231882393, 31.26068001247087, 56.54476386520281);
        let text = std::fs::read_to_string(PART).expect("the committed NIST part");
        let mut m = Model::default();
        let read = step_read(&mut m, &text, &ReadOptions::default()).expect("the part reads");
        let body = read.solids[0].result.as_ref().expect("#249 reads").body;
        let view = BodyView::of(&m, body).unwrap();
        let (edge, _) = (view.uses.keys())
            .filter_map(|&e| {
                let (curve, range) = m.edge(e).ok()?.curve()?;
                let p = m.curve(curve).ok()?.project(at).ok()?;
                range.contains(p.t).then_some((e, p.distance))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap();
        let entity = *m.edge(edge).unwrap();
        let (curve, range) = entity.curve().unwrap();
        let uses = &view.uses[&edge];
        let faces = [uses[0].face, uses[1].face];
        let surfaces = faces.map(|f| m.surface(m.face(f).unwrap().surface()).unwrap().clone());
        let mut kinds = surfaces.each_ref().map(|s| s.kind());
        kinds.sort();
        assert_eq!(kinds, [SurfaceKind::Torus, SurfaceKind::Nurbs]);
        let convex = view.convex(&m, edge).unwrap().unwrap();
        assert!(convex);
        let s = if convex { -1.0 } else { 1.0 };
        let distances = faces.map(|f| s * RADIUS * view.orientation[&f].sign());
        let linear = faces_tolerance(&m, faces[0], faces[1]).unwrap();
        let tol = Tolerance::new(linear, m.precision().tolerance().angular);
        let rolling = Rolling {
            surfaces: [&surfaces[0], &surfaces[1]],
            distances,
            edge: m.curve(curve).unwrap(),
            range,
            closed: entity.start() == entity.end(),
            tangency: linear.max(entity.tolerance()),
            ends: [entity.start(), entity.end()].map(|v| m.vertex(v).unwrap().tolerance()),
        };
        let started = std::time::Instant::now();
        let spine =
            spine(&rolling, tol, &mut Meter::default()).unwrap_or_else(|e| panic!("{edge}: {e:?}"));
        let traced = started.elapsed();
        let samples = 64;
        // The edge's start is where the two faces turn tangent within the
        // edge's tolerance (their normals 8e-6 apart): the spine stops there.
        assert_eq!(spine.vanishing(), [true, false]);
        let mut drift = 0.0f64;
        for i in 0..=samples {
            let t = spine.range().lerp(i as f64 / samples as f64);
            let ball = spine
                .ball(t)
                .unwrap_or_else(|e| panic!("sample {i}: {e:?}"));
            for k in 0..2 {
                let off = (ball.centre - ball.contacts[k]).norm() - RADIUS;
                assert!(off.abs() <= 1e-9, "face {k} at {i}: {off:e}");
            }
            drift = drift.max(across(&spine, t, &ball, RADIUS));
        }
        assert!(drift <= 0.5 * tol.linear, "drift {drift:e}");
        let started = std::time::Instant::now();
        let surface = spine
            .surface(tol, &mut Meter::default())
            .unwrap_or_else(|e| panic!("{edge}: {e:?}"));
        let fitted = started.elapsed();
        let budget = SECTION_FIT_FRACTION * tol.linear;
        let curves = contacts(&surface).unwrap();
        for j in 0..=97 {
            let v = spine.range().lerp(j as f64 / 97.0);
            let ball = spine.ball(v).unwrap();
            for i in 0..=16 {
                let p = surface.eval(i as f64 / 16.0, v).point;
                let off = (p - ball.centre).norm() - RADIUS;
                assert!(off.abs() <= budget, "radial {off:e} at ({i}, {j})");
            }
            // The exact contact, the foot on the face carried past its
            // domain where the reach runs past it (ADR-0062 §3).
            for (k, curve) in curves.iter().enumerate() {
                let q = curve.eval(v).point;
                let off = (q - ball.contacts[k]).norm();
                assert!(off <= budget, "contact {k} off by {off:e} at {j}");
            }
        }
        // The vanishing end: the exact arc, not the edge's point.
        let start = spine.ends()[0];
        let ends = curves.each_ref().map(|c| c.eval(start).point);
        let width = (ends[0] - ends[1]).norm();
        assert!(width <= rolling.tangency, "end arc {width:e} wide");
        let at_edge = spine.pinned()[0].unwrap().contacts[0];
        let reach = (2.0 * RADIUS * rolling.tangency).sqrt();
        let away = ends.map(|q| (q - at_edge).norm());
        assert!(
            away.iter().all(|&d| d <= reach),
            "end arc {away:?} from the edge's point"
        );
        let spans = surface.knots()[1]
            .windows(2)
            .filter(|w| w[1] > w[0])
            .count();
        eprintln!(
            "CTC-05 {edge}: spine over {:?} (edge at {:?}) traced in {traced:?}, drift {drift:e}; \
             surface of {spans} spans fitted in {fitted:?}, its end arc {width:e} wide, \
             {:e} from the edge's point",
            spine.range(),
            spine.ends(),
            away[0].max(away[1])
        );
    }
}
