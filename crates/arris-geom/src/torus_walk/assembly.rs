//! The assembly: marched graphs chained into branches and points.

use super::*;

impl Tracer<'_> {
    /// Every arm marched once, the winding components seeded, and the
    /// graphs chained into branches: open ones first, from the singular
    /// points in their order, then the loops, in the order of the turning
    /// points — an order and orientations the two surfaces alone decide.
    pub(super) fn assemble(self, meter: &mut Meter) -> Result<SectionTrace, Halt<SectionFault>> {
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

    /// The root of the distance along `u = 0` that `candidate` stands
    /// for, when it is one no cell and no marched graph accounts for.
    pub(super) fn seed(
        &self,
        candidate: f64,
        marched: &[Marched],
    ) -> Result<Option<[f64; 2]>, SectionFault> {
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

    pub(super) fn chained(self, marched: Vec<Marched>) -> Result<SectionTrace, SectionFault> {
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
            ArcEnd::Singular(..) | ArcEnd::Edge(_) | ArcEnd::Exit(..) | ArcEnd::Aground => true,
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
            ArcEnd::Edge(_) | ArcEnd::Exit(..) | ArcEnd::Aground => Some(Stop::Edge),
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
