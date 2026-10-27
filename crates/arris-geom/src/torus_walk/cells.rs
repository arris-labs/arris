//! The cells a march starts from and ends in: tube circles, turning
//! points, the edges of a domain.

use super::*;

impl Tracer<'_> {
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
    pub(super) fn tube_circles(&self) -> Vec<TubeFactor> {
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
                // Not `roots::golden_section`: this search re-evaluates both probes each step, whose bits differ from the shared form's reused probe.
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
            let tangent = |&(_, worst, _): &(f64, [f64; 2], f64)| worst[1] <= dropped;
            // A crossing whose next crossing beside it is past the
            // bracket, the polynomial within the tolerance between the
            // two, is one side of a tangency at that tolerance, as two
            // candidates of one group are: the line the slope across
            // vanishes on, a Newton step on the slope from the crossing,
            // is tried as the isoline of order two. A walked blend's end
            // arc against the table stripe's cylinder it runs on into is
            // such a pair of crossings, the fit `8.4e-4` of `u` apart and
            // within `1.5e-9` between (ADR-0062).
            if accepted(&best) && !tangent(&best) {
                let mut steps: Vec<f64> = (0..CIRCLE_SAMPLES)
                    .filter_map(|k| {
                        let v = round(k);
                        let slope = |u: f64| walker.distance(u, v).1[0];
                        let bend = (slope(best.0 + HESSIAN_STEP) - slope(best.0 - HESSIAN_STEP))
                            / (2.0 * HESSIAN_STEP);
                        let step = -slope(best.0) / bend;
                        step.is_finite().then_some(step)
                    })
                    .collect();
                steps.sort_by(f64::total_cmp);
                if let Some(&step) = steps.get(steps.len() / 2) {
                    let touch = measure(best.0 + step);
                    if accepted(&touch) && tangent(&touch) {
                        best = touch;
                    }
                }
            }
            if accepted(&best) {
                let (u0, _, _) = best;
                let order = if tangent(&best) { 2 } else { 1 };
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
    pub(super) fn section_circles(&self) -> Result<Vec<SectionCircle>, SectionFault> {
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
                let (sin, cos) = real::sin_cos(c.u0);
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
    pub(super) fn touch_runs(&self) -> Result<(Vec<[f64; 2]>, Vec<SectionCircle>), SectionFault> {
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
    pub(super) fn turning_cell(
        &self,
        index: usize,
        at: [f64; 2],
        turning: &[[f64; 2]],
    ) -> Result<TurningCell, SectionFault> {
        let walker = &self.walker;
        let closed = walker.closed;
        // The fold is `u − u_T = c·(v − v_T)²` to second order.
        let h = HESSIAN_STEP;
        let fold_at = |[u, v]: [f64; 2]| {
            let bend = (walker.value(u, v + h).1[1] - walker.value(u, v - h).1[1]) / (2.0 * h);
            (0.5 * bend / walker.value(u, v).1[0]).abs()
        };
        // A point is on an edge within `EDGE_TOUCH` of it, and on an edge
        // of `v` also where the fold between them moves `u` by less than
        // an angle's rounding: there the sign along the edge row is
        // rounding's, and the arm beyond the edge is none to `f64`. A
        // walked blend's contact edge has `∂f/∂v` vanish along it where
        // the face across shares the tangent plane, and the section
        // crosses it at a turning point the isolation puts `1.6e-11` in.
        let c = fold_at(at);
        let on_edge = |k: usize| {
            (!closed[k])
                .then(|| {
                    [0.0, TAU].into_iter().find(|e| {
                        let d = (at[k] - e).abs();
                        d <= EDGE_TOUCH || (k == 1 && c * d * d <= ANGLE_SLACK)
                    })
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
        let fold = fold_at(at);
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
    pub(super) fn edge_cells(&self) -> Result<(Vec<EdgeCell>, Vec<EdgeTouch>), SectionFault> {
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
    pub(super) fn edge_root(
        &self,
        k: usize,
        edge: f64,
        candidate: f64,
    ) -> Result<EdgeRoot, SectionFault> {
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
    /// far into the domain as along the line. `f` is signed along a line
    /// half a tolerance in, or failing that along the first line deeper,
    /// doubling, where rounding signs it; a zone that deep is then held
    /// to the tolerance of the other surface at every point of it. Its
    /// stretch is grown from a tolerance's length until `f` is signed
    /// along the edge beyond it, and every point of it is within the
    /// tolerance of the other surface.
    pub(super) fn edge_touch(&self, k: usize, edge: f64, candidate: f64) -> Option<EdgeTouch> {
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
        // Not `roots::golden_section`: this search re-evaluates both probes each step, whose bits differ from the shared form's reused probe.
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
            // A tolerance in, or deeper where rounding cannot sign `f`
            // that near the edge: where an isoline divided out ends on
            // the edge, the quotient's floor carries the division, and a
            // shallow touch there is unsigned as far in as the floor
            // reaches. The zone between the edge and that line is held
            // to the tolerance below, every point of it.
            let mut inner = depth;
            while inner < a && sign_along(line, edge + inward * inner) != Some(rising) {
                inner *= 2.0;
            }
            if inner >= a {
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
            // The edge line over the stretch, and with a deeper zone
            // every line of it out to its inner one.
            let rows = if inner > depth { CIRCLE_SAMPLES } else { 0 };
            let held = (0..=rows).all(|r| {
                let line = edge + inward * inner * r as f64 / CIRCLE_SAMPLES as f64;
                (0..=CIRCLE_SAMPLES).all(|n| {
                    let x = run[0] + (run[1] - run[0]) * n as f64 / CIRCLE_SAMPLES as f64;
                    let [u, v] = if k == 0 { [line, x] } else { [x, line] };
                    walker.distance(u, v).0.abs() <= self.tol.linear
                })
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
    pub(super) fn edge_cell(
        &self,
        k: usize,
        high: bool,
        at: [f64; 2],
    ) -> Result<EdgeCell, SectionFault> {
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
}
