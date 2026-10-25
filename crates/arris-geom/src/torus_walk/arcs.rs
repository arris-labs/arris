//! The arcs: a march from a cell to where it ends, and how a marched
//! graph is read back as an arc.

use super::*;

impl Tracer<'_> {
    /// The arm a march in direction `dir` has reached at `(u, v)`, if it
    /// stands in a cell whose arms point back at it, as far from the
    /// cell's point as the arm is followed there: the cell holds nothing
    /// but its point's arms. Two turning points of an S-bend may have
    /// cells that overlap, and a march between them starts captured.
    pub(super) fn captured(&self, [u, v]: [f64; 2], dir: f64) -> Option<(ArcEnd, Cell, [f64; 2])> {
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

    /// Where a march stuck at `(u, v)` in direction `dir` ends, on the
    /// edge of a NURBS surface's `v` it has come to within the
    /// tolerance, and no farther along it than `room`: no step from
    /// `(u, v)` is proven because the section there is nearer the edge
    /// than its polynomial's floor can sign. That is where a branch runs
    /// into its foot on the edge alongside it, the edge being within the
    /// tolerance of the other surface over a stretch: two faces tangent
    /// to one face across, the walked blend's contact edge on it and a
    /// table stripe's cylinder beside it (ADR-0062). The branch is not the tracer's to resolve there, as a point
    /// within the tolerance of an edge is put on it (ADR-0061 §3).
    ///
    /// The march runs aground where the line of constant `u` from
    /// `(u, v)` to the edge is within `tol.linear` of the other surface
    /// all the way, and follows the edge line while that holds of it, in
    /// steps doubling from [`HALF_MIN`], to the first of: an edge cell
    /// whose arm points back at it, which captures it as a march is
    /// captured; the foot of the isoline divided out, where the branch
    /// meets it; a root of `f` along the line, found by bisection, where
    /// the branch leaves the domain. Short of all three it ends on the
    /// edge at `u`. `None` where the march is not within the tolerance
    /// of an edge so, or `v` closes.
    pub(super) fn aground(
        &self,
        [u, v]: [f64; 2],
        dir: f64,
        room: f64,
    ) -> Option<(ArcEnd, [f64; 2])> {
        let walker = &self.walker;
        if walker.closed[1] {
            return None;
        }
        let other = Implicit::of(&walker.other)?;
        let edge = if v < PI { 0.0 } else { TAU };
        let held = |x: f64, y: f64| other.distance(walker.point(x, y)).abs() <= self.tol.linear;
        let down = (0..=CIRCLE_SAMPLES).all(|n| {
            let y = v + (edge - v) * n as f64 / CIRCLE_SAMPLES as f64;
            held(u, y)
        });
        if !down {
            return None;
        }
        let negative = |x: f64| walker.is_negative(x, edge);
        let start = negative(u);
        let mut last = u;
        let mut step = HALF_MIN;
        loop {
            let reach = step.min(room);
            let x = u + dir * reach;
            if !held(x, edge) {
                break;
            }
            if let Some((end @ ArcEnd::Edge(_), _, centre)) = self.captured([x, edge], dir) {
                return Some((end, centre));
            }
            let foot = (walker.factor)
                .map(|f| f.u0)
                .filter(|&u0| dir * (u0 - last) > 0.0 && dir * (x - u0) >= 0.0);
            if let Some(u0) = foot {
                return Some((ArcEnd::Aground, [u0, edge]));
            }
            if negative(x) != start {
                let (mut a, mut b) = (last, x);
                for _ in 0..STRETCH_HALVINGS {
                    let mid = 0.5 * (a + b);
                    if negative(mid) == start {
                        a = mid;
                    } else {
                        b = mid;
                    }
                }
                return Some((ArcEnd::Aground, [0.5 * (a + b), edge]));
            }
            if reach >= room {
                break;
            }
            last = x;
            step *= 2.0;
        }
        Some((ArcEnd::Aground, [u, edge]))
    }

    /// How far a march at `u` may go in direction `dir` before the next
    /// edge it could be captured at; a whole turn when there is none, and
    /// on a NURBS surface whose `u` does not close no farther than the
    /// domain's edge.
    pub(super) fn next_edge(&self, u: f64, dir: f64, seeded: bool) -> f64 {
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
    pub(super) fn march(
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
        let (end, last) = 'march: loop {
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
                    if let Some((end, at)) = self.aground([u, v], dir, edge) {
                        cells.push(Cell {
                            u: [u.min(at[0]), u.max(at[0])],
                            bracket: Bracket::Fixed([v.min(at[1]), v.max(at[1])]),
                        });
                        break 'march (end, at);
                    }
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

    /// The arm `arm` of turning point `i`, which leaves the domain inside
    /// the point's cell: from the point to the edge, through the side the
    /// cell is cut at in `u` — the end the root along that edge line — or
    /// the edge it is cut at in `v`, where the end is on the edge.
    pub(super) fn exit(&self, i: usize, arm: usize) -> Marched {
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
    pub(super) fn arc_from(
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

    /// How graph `m` is evaluated beside the turning point its end `side`
    /// is, if it is one ([`Fold`]).
    pub(super) fn fold(&self, m: &Marched, side: usize) -> Option<Fold> {
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
}
