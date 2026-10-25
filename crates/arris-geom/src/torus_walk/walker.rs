//! The walk's evaluations: the bump, the walker over a torus and the
//! section's surface, and what a branch does with them.

use super::*;

impl Bump {
    /// `y` at `(u, v)`, and `M·x`.
    pub(super) fn measure(&self, u: f64, v: f64) -> (f64, [f64; 2]) {
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
    pub(super) fn extent(&self) -> [f64; 2] {
        let [muu, muv, mvv] = self.metric;
        let det = muu * mvv - muv * muv;
        [mvv, muu].map(|m| self.reach * (m / det).sqrt())
    }
}

impl Walker {
    /// The walked surface at the walk's `(u, v)`, its derivatives in the
    /// walk's parameters.
    pub(super) fn eval(&self, u: f64, v: f64) -> SurfaceEval {
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
    pub(super) fn point(&self, u: f64, v: f64) -> Point3 {
        match &self.rescale {
            None => self.walked.point(u, v),
            Some(map) => (self.walked).point(map.to_surface(0, u), map.to_surface(1, v)),
        }
    }

    /// The walked surface's own parameters at the walk's `at`: a NURBS
    /// surface walked transposed has them swapped back.
    pub(super) fn surface_uv(&self, at: [f64; 2]) -> [f64; 2] {
        let uv = match &self.rescale {
            None => at,
            Some(map) => [0, 1].map(|k| map.to_surface(k, at[k])),
        };
        if self.transposed { [uv[1], uv[0]] } else { uv }
    }

    /// The difference `x` of direction `k`'s parameter, the short way
    /// round where it closes.
    pub(super) fn off(&self, k: usize, x: f64) -> f64 {
        offset(self.closed[k], x)
    }

    /// The other surface's signed distance at the walk's `(u, v)` and
    /// its gradient in `(u, v)`, as they are.
    pub(super) fn distance(&self, u: f64, v: f64) -> (f64, [f64; 2]) {
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
    pub(super) fn quotient(&self, factor: &TubeFactor, u: f64, v: f64) -> f64 {
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
        let rho = major_radius + minor_radius * real::cos(v);
        let (x, y) = (frame.x().into_inner(), frame.y().into_inner());
        let (sin, cos) = real::sin_cos(factor.u0);
        let (sin_mid, cos_mid) = real::sin_cos(factor.u0 + half);
        let outward = implicit.frame.vec_to_local(x * cos + y * sin);
        let tau = implicit.frame.vec_to_local(y * cos_mid - x * sin_mid);
        let a = implicit.frame.to_local(self.walked.point(factor.u0, v));
        let p = implicit.frame.to_local(self.walked.point(u, v));
        let gradient = implicit.slope(a);
        let [c2, c3, c4] = implicit.bend(a, tau);
        let lambda = 2.0 * rho * real::sin(half);
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
    pub(super) fn apart(&self, u: f64, v: f64) -> f64 {
        match (&self.factor, Implicit::of(&self.other)) {
            (Some(factor), _) => self.quotient(factor, u, v),
            (None, Some(other)) => other.distance(self.point(u, v)),
            (None, None) => f64::NAN,
        }
    }

    /// The other surface's signed distance at the torus's `(u, v)` — with
    /// a tube circle divided out of it where one lies on the other
    /// surface — corrected, and its gradient in `(u, v)`.
    pub(super) fn value(&self, u: f64, v: f64) -> (f64, [f64; 2]) {
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

    pub(super) fn is_negative(&self, u: f64, v: f64) -> bool {
        self.value(u, v).0 < 0.0
    }

    /// The root of the distance along the tube circle at `u` between
    /// `lo` and `hi`, which a cell's certificate says is the only one.
    /// Where rounding leaves no sign change between the two — at a
    /// turning point itself, where the root is an end — it is the end
    /// nearer the surface.
    pub(super) fn root_v(&self, u: f64, lo: f64, hi: f64) -> f64 {
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
    pub(super) fn root_u(&self, v: f64, lo: f64, hi: f64) -> Option<f64> {
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
    pub(super) fn separator(&self, cell: &SingularCell, centre: [f64; 2], u: f64) -> f64 {
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
    pub(super) fn beside(&self, fold: &Fold, off: f64) -> Option<[f64; 2]> {
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
    pub(super) fn stretch(&self, u: f64, v: f64, along_v: bool) -> Stretch {
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
    pub(super) fn v_on(&self, arc: &TorusArc, u: f64) -> f64 {
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

impl TorusWalk {
    /// `(u, v)` of arc `i` at the walked angle `u`, in that arc's own
    /// angles; `anchor` is the turning point `u` is measured from and
    /// half the offset, as `Arc1::angle` gives them.
    pub(super) fn local(&self, i: usize, u: f64, anchor: Option<(f64, f64)>) -> Option<[f64; 2]> {
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

impl Singular {
    /// How far the correction reaches along `u` and along `v`.
    pub(super) fn extent(&self) -> [f64; 2] {
        if self.bump.reach > 0.0 {
            self.bump.extent()
        } else {
            [0.0; 2]
        }
    }

    /// `p` within `times` the correction's reach.
    pub(super) fn holds(&self, p: [f64; 2], times: f64) -> bool {
        self.bump.reach > 0.0 && self.bump.measure(p[0], p[1]).0 <= times
    }
}
