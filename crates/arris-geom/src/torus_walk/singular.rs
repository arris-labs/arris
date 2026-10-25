//! The singular points: where branches meet, and the cells that hold them.

use super::*;

impl Tracer<'_> {
    /// The critical points within the tolerance of the other surface,
    /// each with its reach.
    pub(super) fn singular_points(
        &self,
        critical: &[Zero<2>],
    ) -> Result<Vec<Singular>, SectionFault> {
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
    pub(super) fn singular_cell(
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
}
