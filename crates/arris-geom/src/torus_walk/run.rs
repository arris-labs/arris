//! The run: the tracer's phases in order, from the cells found to the
//! marches begun, and the poles and edges that cut them.

use super::*;

impl Tracer<'_> {
    pub(super) fn run(self, meter: &mut Meter) -> Result<SectionTrace, Halt<SectionFault>> {
        let circles = self.tube_circles();
        self.run_with(circles, meter)
    }

    /// The trace with `circles` — the tube circles of a torus or the
    /// isolines of a NURBS surface on the other surface — divided out.
    pub(super) fn run_with(
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
        if self.section.turns_at_a_kink(self.tol) {
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
            // A box of critical points beside the isoline divided out, the
            // strip from it to the isoline held to the tolerance, is the
            // isoline's, as a box of turning points is: the quotient's
            // double zero a hair off a fitted row tangent to an exact arc,
            // which far from the origin the rounding floor leaves
            // uncertified (ADR-0061, amended 2026-10-29).
            critical.retain(|z| z.certified || !self.beside_isoline(z));
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
        // Such a box beside the isoline divided out, or a shorter one, the
        // strip between the two within the tolerance of the other surface
        // all over, is the isoline's: a tangency held to the tolerance a
        // hair off a line of the surface's own, where a fitted row stands
        // for an exact arc.
        let turning: Vec<Zero<2>> = (turning.into_iter())
            .filter(|z| torus || z.certified || !self.beside_isoline(z))
            .collect();

        // A singular point beside the isoline divided out, held to the
        // tolerance out to it, is the isoline's.
        let singular: Vec<Singular> = (self.singular_points(&critical)?.into_iter())
            .filter(|s| torus || !self.singular_beside_isoline(s))
            .collect();
        self.walker.bumps = singular
            .iter()
            .filter(|s| s.bump.reach > 0.0)
            .map(|s| s.bump)
            .collect();
        // The turning points of a near miss, of a loop smaller than the
        // tolerance and of the crossing itself are the singular point's.
        let absorbed = |z: &Zero<2>| {
            singular.iter().any(|s| {
                let held = !z.certified
                    && (0..2).all(|k| {
                        let half = 0.5 * (z.hi[k] - z.lo[k]);
                        let off = s.at[k] - 0.5 * (z.lo[k] + z.hi[k]);
                        // A singular point on an edge of the domain is
                        // on the box's own edge, when the box reaches it.
                        self.walker.off(k, off).abs() <= half + EDGE_TOUCH
                    });
                held || s.holds(z.at, 1.0) || (!s.crossing && self.beside_touch(z, s.at))
            })
        };
        let turning: Vec<Zero<2>> = turning.into_iter().filter(|z| !absorbed(z)).collect();
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
    pub(super) fn is_pole(&self, u0: f64) -> bool {
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
    pub(super) fn run_beside_poles(
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
    pub(super) fn critical_past_edges(
        &self,
        inside: &[Zero<2>],
        meter: &mut Meter,
    ) -> Result<Vec<Zero<2>>, Halt<SectionFault>> {
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
                    // Near the other direction's edge too, it is the
                    // corner's: put there where its point is within the
                    // tolerance of the corner's, and no point of the
                    // surface where it lies past that edge and is not.
                    let j = 1 - k;
                    if !self.walker.closed[j] {
                        let corner = if z.at[j] < PI { 0.0 } else { TAU };
                        let mut at = z.at;
                        at[j] = corner;
                        let off =
                            (self.walker.point(at[0], at[1])) - self.walker.point(z.at[0], z.at[1]);
                        if off.norm() <= self.tol.linear {
                            (z.at[j], z.lo[j], z.hi[j]) = (corner, corner, corner);
                        } else if !(0.0..=TAU).contains(&z.at[j]) {
                            continue;
                        }
                    }
                    let known = inside.iter().any(|c| {
                        (0..2).all(|j| {
                            (c.lo[j] - EDGE_TOUCH..=c.hi[j] + EDGE_TOUCH).contains(&z.at[j])
                        })
                    }) || out.iter().any(|c: &Zero<2>| c.at == z.at);
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
    pub(super) fn flush(&self) -> bool {
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
    pub(super) fn on_domain(&self, zeros: Vec<Zero<2>>) -> Vec<Zero<2>> {
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
}
