//! A walked chamfer's contacts (ADR-0062 §4): a chamfer has no ball, so no
//! spine. At each point of the edge the two contacts lie in the edge's
//! normal plane, each on its own face at chord `d` from the edge's point,
//! on the side of the edge the face lies on — the ring's rule (ADR-0036 §2)
//! and Open CASCADE's symmetric chamfer. Each is the root of two equations
//! on the face's `(u, v)`, the plane's and the sphere's of radius `d` about
//! the edge's point, found by Newton from the face's foot of that point
//! moved along the face by `d`. The stripe's surface is the ruled surface
//! between the two contact curves, skinned (`fit_skin`) from segments of
//! degree one, so its isocurves `u = 0` and `u = 1` are the contact curves
//! and every isocurve across is a chord.

use arris_geom::{Curve, NurbsCurve, NurbsSurface, SECTION_FIT_FRACTION, Surface, fit_skin};
use arris_math::nalgebra::{Matrix2, Vector2};
use arris_math::{Interval, Meter, Point2, Point3, Tolerance, Vec3, is_negligible};

use super::walked::{
    EDGE_SAMPLES, NEWTON_STEPS, REACH_RADII, Refusal, STEP_ROUNDING, SURFACE_DEGREE, Skin, Walked,
    foot_on,
};

/// The edge and the two faces a chamfer's contacts are found on.
pub(super) struct Chording<'a> {
    pub(super) surfaces: [&'a Surface; 2],
    /// Each face's sign turning `∂u × ∂v` into the face's outward
    /// normal, times the sign of the edge's direction along that face's
    /// use of it: `n × T` then points into the face.
    pub(super) inward: [f64; 2],
    pub(super) distance: f64,
    pub(super) edge: &'a Curve,
    pub(super) range: Interval,
    pub(super) closed: bool,
}

/// A walked chamfer's contact curves, evaluated by the edge's parameter
/// over the edge's range carried on past each end.
pub(super) struct Chord {
    surfaces: [Surface; 2],
    inward: [f64; 2],
    distance: f64,
    edge: Curve,
    /// The edge's own range.
    edge_range: Interval,
    /// The range the contacts are skinned over: the edge's, and past each
    /// end by [`REACH_RADII`] chords where the edge goes on, so that a face
    /// across can cut the contacts.
    range: Interval,
    /// The contacts' `(u, v)` on each face at the range's samples, which
    /// seed a NURBS face's Newton iteration.
    seeds: Vec<(f64, [Point2; 2])>,
}

fn unsettled(why: &'static str) -> Walked {
    Walked::Refused(Refusal::NoSpine { why })
}

impl Chord {
    /// The chamfer's contacts along `c`, found at the range's samples first.
    ///
    /// Errors: [`Refusal::NoSpine`] where a contact does not settle on its
    /// face (the faces fold away from the chord, a pole, a speed of zero
    /// along the edge); the operation's error for an interruption.
    pub(super) fn new(c: &Chording<'_>, meter: &mut Meter<'_>) -> Result<Chord, Walked> {
        let speed = |t: f64| c.edge.eval(t).d1.norm();
        let range = if c.closed {
            c.range
        } else {
            let reach = REACH_RADII * c.distance;
            let (s0, s1) = (speed(c.range.lo()), speed(c.range.hi()));
            if !(s0 > 0.0 && s1 > 0.0) {
                return Err(unsettled("an edge of no speed at its end"));
            }
            let (mut before, mut after) = (reach / s0, reach / s1);
            // A circle is not carried on past its period; a NURBS edge is,
            // on its end spans' polynomials as its faces are (ADR-0062 §3).
            if let (Some(period), false) = (c.edge.period(), matches!(c.edge, Curve::Line { .. })) {
                let free = (period - c.range.length()).max(0.0);
                before = before.min(0.49 * free);
                after = after.min(0.49 * free);
            }
            Interval::new(c.range.lo() - before, c.range.hi() + after)
                .map_err(|_| unsettled("an edge's range"))?
        };
        let mut chord = Chord {
            surfaces: c.surfaces.map(Clone::clone),
            inward: c.inward,
            distance: c.distance,
            edge: c.edge.clone(),
            edge_range: c.range,
            range,
            seeds: Vec::with_capacity(EDGE_SAMPLES + 1),
        };
        let mut previous: [Option<Point2>; 2] = [None, None];
        for i in 0..=EDGE_SAMPLES {
            meter.tick()?;
            let t = range.lerp(i as f64 / EDGE_SAMPLES as f64);
            let [a, b] = [0, 1].map(|k| chord.contact(k, t, previous[k]));
            let (uv_a, uv_b) = (a?.1, b?.1);
            previous = [Some(uv_a), Some(uv_b)];
            chord.seeds.push((t, [uv_a, uv_b]));
        }
        Ok(chord)
    }

    /// The edge's point at `t`.
    pub(super) fn edge_point(&self, t: f64) -> Point3 {
        self.edge.point(t)
    }

    /// The chamfer's two contacts at `t` and their `(u, v)` on the faces,
    /// each seeded from the nearest of the samples.
    fn contacts(&self, t: f64) -> Result<[Point3; 2], Walked> {
        let nearest = self
            .seeds
            .iter()
            .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
            .map(|s| s.1);
        let a = self.contact(0, t, nearest.map(|s| s[0]))?;
        let b = self.contact(1, t, nearest.map(|s| s[1]))?;
        Ok([a.0, b.0])
    }

    /// The contact on face `k` at the edge's parameter `t`: the point of the
    /// face in the edge's normal plane at chord `distance` from the edge's
    /// point, on the face's side of the edge.
    fn contact(&self, k: usize, t: f64, seed: Option<Point2>) -> Result<(Point3, Point2), Walked> {
        let e = self.edge.eval(t);
        let tangent =
            e.d1.try_normalize(0.0)
                .ok_or_else(|| unsettled("an edge of no direction"))?;
        let surface = &self.surfaces[k];
        let (uv, foot) = foot_on(surface, e.point, seed)?;
        let n = surface
            .normal(uv.x, uv.y)
            .ok_or_else(|| unsettled("no normal at a chamfer's foot"))?;
        let inward: Vec3 = (self.inward[k] * n.cross(&tangent))
            .try_normalize(0.0)
            .ok_or_else(|| unsettled("a face tangent to its edge"))?;
        // The first guess: the foot moved by the chord along the face's
        // tangent plane, in `(u, v)` by least squares.
        let d = self.distance;
        let at = surface.eval(uv.x, uv.y);
        let want = foot + d * inward - at.point;
        let gram = Matrix2::new(
            at.du.dot(&at.du),
            at.du.dot(&at.dv),
            at.du.dot(&at.dv),
            at.dv.dot(&at.dv),
        );
        let first = gram
            .lu()
            .solve(&Vector2::new(at.du.dot(&want), at.dv.dot(&want)))
            .ok_or_else(|| unsettled("a singular point at a chamfer's foot"))?;
        let mut x = Vector2::new(uv.x, uv.y) + first;
        // The residual in lengths: the plane's, the chord's excess.
        let residual = |x: &Vector2<f64>| {
            let s = surface.eval(x.x, x.y);
            let r = s.point - e.point;
            (
                s,
                r,
                Vector2::new(r.dot(&tangent), 0.5 * (r.norm_squared() - d * d) / d),
            )
        };
        let scale = STEP_ROUNDING * (e.point.coords.norm() + d);
        let (mut s, mut r, mut f) = residual(&x);
        for _ in 0..NEWTON_STEPS {
            if is_negligible(f.norm(), scale) {
                break;
            }
            let j = Matrix2::new(
                tangent.dot(&s.du),
                tangent.dot(&s.dv),
                r.dot(&s.du) / d,
                r.dot(&s.dv) / d,
            );
            let step = j
                .lu()
                .solve(&-f)
                .ok_or_else(|| unsettled("a chamfer's contact is not isolated on its face"))?;
            if is_negligible((step.x * s.du + step.y * s.dv).norm(), scale) {
                x += step;
                (s, r, f) = residual(&x);
                break;
            }
            let mut alpha = 1.0;
            let mut moved = None;
            for _ in 0..NEWTON_STEPS {
                let next = x + alpha * step;
                let (ns, nr, nf) = residual(&next);
                if nf.norm() < f.norm() {
                    moved = Some((next, ns, nr, nf));
                    break;
                }
                alpha *= 0.5;
            }
            let Some(next) = moved else { break };
            (x, s, r, f) = next;
        }
        if !is_negligible(f.norm(), scale) || r.dot(&inward) <= 0.0 {
            return Err(unsettled("a chamfer's contact does not settle on its face"));
        }
        Ok((s.point, Point2::new(x.x, x.y)))
    }

    /// The segment from the first contact to the second at `t`, as a
    /// curve of degree one over `[0, 1]`.
    fn section(&self, t: f64) -> Result<NurbsCurve, Walked> {
        let [a, b] = self.contacts(t)?;
        Ok(NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![a, b],
            vec![1.0, 1.0],
        )?)
    }

    /// The ruled surface between the contact curves over the whole range,
    /// within [`SECTION_FIT_FRACTION`] of `tol.linear` of every chord it is
    /// checked against, `u` from the contact on the first face to the one
    /// on the second and `v` the edge's own parameter.
    pub(super) fn skin(&self, tol: Tolerance, meter: &mut Meter<'_>) -> Result<Skin, Walked> {
        let surface: NurbsSurface = fit_skin(
            |t| self.section(t),
            self.range,
            SURFACE_DEGREE,
            SECTION_FIT_FRACTION * tol.linear,
            meter,
        )?;
        Ok(Skin::along_edge(
            surface,
            self.range,
            [self.edge_range.lo(), self.edge_range.hi()],
        ))
    }
}
