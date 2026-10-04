//! The nearest point of a NURBS surface to a point: global, not local.
//!
//! The surface is cut into its Bézier patches (a rational patch with
//! positive weights lies in the convex hull of its Cartesian control
//! points, so the distance to that hull's box bounds the distance to the
//! patch from below). A best-first search keeps the patches whose bound is
//! within the best distance found so far, halves the ones that are still
//! large, and hands each survivor that is small to a projected Newton
//! iteration on the squared distance. What comes out is every local
//! minimum nearer than the best distance plus rounding, so a tie between
//! two *distinct* points is seen and reported rather than resolved by the
//! order the patches were visited in.
//!
//! *The NURBS Book* §6.1 (Bézier extraction by blossoming) and, for
//! bounding a surface by its hull and pruning against the best distance,
//! Piegl & Tiller §6.1's point inversion, made global by the pruning; the
//! reference tree's `truck-geometry` searches by a grid and a Newton
//! iteration and returns whichever local minimum the grid finds, which is
//! the behaviour this replaces.

use core::cmp::Ordering;
use std::collections::BinaryHeap;

use arris_math::nalgebra::Vector4;
use arris_math::{Point2, Point3, RELATIVE_ROUNDING, Vec3, is_negligible};

use super::NurbsSurface;
use crate::{AmbiguousLocus, GeomError, GeomKind, SurfaceKind, SurfaceProjection};

/// How many times the rounding of the coordinates a minimum is located to:
/// a Newton iteration on noisy evaluations stops a few units of it short of
/// an end of the domain or a pole, more the farther the query is.
const LOCATION: f64 = 8.0;

/// A patch is a leaf, handed to Newton's iteration, once the diagonal of
/// its control net's box is at most this fraction of the whole surface's.
/// A ratio that sets how small a patch is before one Newton start inside it
/// converges to the minimum it holds: it costs a few more halvings where
/// it is smaller and a Newton run that wanders to a neighbour where it is
/// larger, and the neighbour's own leaf finds the same point, which
/// merges. It never changes an answer.
const LEAF_FRACTION: f64 = 1.0 / 64.0;

/// How many times a patch may be halved. Each halving is along one
/// direction, so this is about thirty per direction: far below what
/// `f64` parameters resolve, a bound on the search and not a resolution.
const MAX_DEPTH: usize = 60;

/// How many nodes the search may take from its queue, beyond
/// [`NODES_PER_PATCH`] for each Bézier patch the surface starts with. A
/// structural bound: a query with more nodes as near as the best one is a
/// query on a continuum of equally near points — a surface's axis of
/// symmetry, a sphere's centre — and is reported as
/// [`AmbiguousLocus::MedialAxis`]. A query with one nearest point takes a
/// few hundred; a continuum takes as many as its leaves.
const NODE_BUDGET: usize = 4096;

/// The nodes each initial patch adds to [`NODE_BUDGET`], so that a large
/// free-form surface, whose patches all survive a far query's first
/// round, is not taken for a continuum.
const NODES_PER_PATCH: usize = 32;

/// Newton steps per start. Newton's iteration converges quadratically from
/// a leaf, so the count is reached only by a start that cycles on rounding.
const MAX_STEPS: usize = 64;

/// The fraction of the decrease the slope promises that a step must deliver
/// (Armijo's condition, *Numerical Optimization* §3.1): a conventional
/// value that only has to be small, not tuned.
const SUFFICIENT_DECREASE: f64 = 1e-4;

/// Halvings of a Newton step that fails to decrease the distance.
const MAX_BACKTRACKS: usize = 40;

/// One Bézier patch, in homogeneous coordinates `(w·P, w)`, over a box of
/// the surface's parameters.
#[derive(Clone)]
struct Patch {
    u: [f64; 2],
    v: [f64; 2],
    /// The knot span's box the patch is inside: the polynomial piece of the
    /// surface it belongs to.
    span: [[f64; 2]; 2],
    /// `[p, q]`.
    degree: [usize; 2],
    /// Row-major over `u`: `(a, b)` is `net[a * (q + 1) + b]`.
    net: Vec<Vector4<f64>>,
    depth: usize,
}

impl Patch {
    fn cartesian(h: &Vector4<f64>) -> [f64; 3] {
        [h.x / h.w, h.y / h.w, h.z / h.w]
    }

    /// The box of the Cartesian control points, `[min, max]`: the patch is
    /// inside it.
    fn hull_box(&self) -> [[f64; 3]; 2] {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for h in &self.net {
            for (k, c) in Self::cartesian(h).into_iter().enumerate() {
                lo[k] = lo[k].min(c);
                hi[k] = hi[k].max(c);
            }
        }
        [lo, hi]
    }

    /// How far `p` is from the patch's convex hull, from below: no point of
    /// the patch is nearer than this. The larger of the distance to the
    /// hull's axis-aligned box and of two support bounds — for a unit
    /// direction `n`, every point `x` of the hull has `|p − x| ≥ n·(p − x)`,
    /// so the least of `n·(p − Qᵢ)` over the control points is a bound. The
    /// directions are the one from the hull's centre to `p` and the hull's
    /// own normal turned toward `p`: a box is as loose as a tilted, thin
    /// patch is long, and these two are tight for the thin patches a
    /// subdivision ends in, which is what lets a flat valley's neighbours be
    /// pruned instead of visited.
    fn lower_bound(&self, p: Point3) -> f64 {
        let points: Vec<Vec3> = self
            .net
            .iter()
            .map(|h| Vec3::from(Self::cartesian(h)))
            .collect();
        let [lo, hi] = self.hull_box();
        let mut bound = (0..3)
            .map(|k| (lo[k] - p[k]).max(p[k] - hi[k]).max(0.0).powi(2))
            .sum::<f64>()
            .sqrt();
        let centre = points.iter().sum::<Vec3>() / points.len() as f64;
        let toward = p.coords - centre;
        let (rows, cols) = (self.degree[0] + 1, self.degree[1] + 1);
        let corner = |a: usize, b: usize| points[a * cols + b];
        let across_u =
            corner(rows - 1, 0) + corner(rows - 1, cols - 1) - corner(0, 0) - corner(0, cols - 1);
        let across_v =
            corner(0, cols - 1) + corner(rows - 1, cols - 1) - corner(0, 0) - corner(rows - 1, 0);
        for n in [toward, across_u.cross(&across_v)] {
            let Some(n) = n.try_normalize(0.0) else {
                continue;
            };
            let n = if n.dot(&toward) < 0.0 { -n } else { n };
            let support = points
                .iter()
                .map(|q| n.dot(&(p.coords - q)))
                .fold(f64::INFINITY, f64::min);
            bound = bound.max(support);
        }
        bound
    }

    fn diagonal(&self) -> f64 {
        let [lo, hi] = self.hull_box();
        (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt()
    }

    fn middle(&self) -> [f64; 2] {
        [0.5 * (self.u[0] + self.u[1]), 0.5 * (self.v[0] + self.v[1])]
    }

    /// The direction in which the control net is longer: the one to halve.
    fn longer(&self) -> usize {
        let [p, q] = self.degree;
        let at = |a: usize, b: usize| Vec3::from(Self::cartesian(&self.net[a * (q + 1) + b]));
        let along_u = (0..=q)
            .map(|b| {
                (0..p)
                    .map(|a| (at(a + 1, b) - at(a, b)).norm())
                    .sum::<f64>()
            })
            .fold(0.0, f64::max);
        let along_v = (0..=p)
            .map(|a| {
                (0..q)
                    .map(|b| (at(a, b + 1) - at(a, b)).norm())
                    .sum::<f64>()
            })
            .fold(0.0, f64::max);
        usize::from(along_v > along_u)
    }

    /// The two halves along `direction`, by de Casteljau's subdivision at
    /// the middle of the patch's parameter range.
    fn halves(&self, direction: usize) -> [Patch; 2] {
        let [p, q] = self.degree;
        let (rows, cols) = (p + 1, q + 1);
        let split = |line: &[Vector4<f64>]| -> (Vec<Vector4<f64>>, Vec<Vector4<f64>>) {
            let mut work = line.to_vec();
            let n = work.len();
            let (mut first, mut second) = (vec![work[0]], vec![work[n - 1]]);
            for r in 1..n {
                for i in 0..n - r {
                    work[i] = 0.5 * (work[i] + work[i + 1]);
                }
                first.push(work[0]);
                second.push(work[n - 1 - r]);
            }
            second.reverse();
            (first, second)
        };
        let mut nets = [Vec::new(), Vec::new()];
        if direction == 0 {
            let mut halves = [
                vec![Vector4::zeros(); rows * cols],
                vec![Vector4::zeros(); rows * cols],
            ];
            for b in 0..cols {
                let column: Vec<_> = (0..rows).map(|a| self.net[a * cols + b]).collect();
                let (first, second) = split(&column);
                for a in 0..rows {
                    halves[0][a * cols + b] = first[a];
                    halves[1][a * cols + b] = second[a];
                }
            }
            nets = halves;
        } else {
            for a in 0..rows {
                let (first, second) = split(&self.net[a * cols..(a + 1) * cols]);
                nets[0].extend(first);
                nets[1].extend(second);
            }
        }
        let [net_lo, net_hi] = nets;
        let mid = self.middle();
        let (u, v) = if direction == 0 {
            ([[self.u[0], mid[0]], [mid[0], self.u[1]]], [self.v, self.v])
        } else {
            ([self.u, self.u], [[self.v[0], mid[1]], [mid[1], self.v[1]]])
        };
        let make = |net, u, v| Patch {
            u,
            v,
            span: self.span,
            degree: self.degree,
            net,
            depth: self.depth + 1,
        };
        [make(net_lo, u[0], v[0]), make(net_hi, u[1], v[1])]
    }
}

/// The `p + 1` Bézier control values of the span `i` of a knot vector, from
/// the homogeneous control values `active` of the controls `i − p ..= i`
/// that carry it: the blossom of the spline at `lo` and `hi` in every
/// mixture (*The NURBS Book* §5.7).
fn extract(p: usize, knots: &[f64], i: usize, active: &[Vector4<f64>]) -> Vec<Vector4<f64>> {
    let (lo, hi) = (knots[i], knots[i + 1]);
    (0..=p)
        .map(|k| {
            let mut d = active.to_vec();
            for r in 1..=p {
                // The last `k` arguments of the blossom are `hi`.
                let t = if r > p - k { hi } else { lo };
                for j in (r..=p).rev() {
                    let g = i - p + j;
                    let alpha = (t - knots[g]) / (knots[g + p - r + 1] - knots[g]);
                    d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
                }
            }
            d[p]
        })
        .collect()
}

/// The largest `f64` below `x`, for a finite `x`: `f64::next_down`, which
/// is stable from Rust 1.86 and the workspace supports 1.85.
fn below(x: f64) -> f64 {
    if x == 0.0 {
        return -f64::from_bits(1);
    }
    let bits = x.to_bits();
    f64::from_bits(if x > 0.0 { bits - 1 } else { bits + 1 })
}

/// A local minimum of the distance.
struct Candidate {
    uv: [f64; 2],
    point: Point3,
    distance: f64,
}

/// A patch in the queue: nearest lower bound first, insertion order among
/// equals, so the search order — and with it the answer — is the same on
/// every run.
struct Queued {
    bound: f64,
    seq: usize,
    patch: Patch,
}

impl PartialEq for Queued {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Queued {}

impl PartialOrd for Queued {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Queued {
    /// Reversed, for a max-heap that pops the smallest bound.
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .bound
            .total_cmp(&self.bound)
            .then(other.seq.cmp(&self.seq))
    }
}

impl NurbsSurface {
    /// The Bézier patches of the surface: one per pair of non-empty knot
    /// spans, in row-major order over `u`. Unclamped knots are read as they
    /// are — a periodic direction's patches are the ones inside its
    /// domain.
    fn patches(&self) -> Vec<Patch> {
        let [p, q] = self.degree();
        let [n, m] = self.counts();
        let [ku, kv] = self.knots();
        let homogeneous = |i: usize, j: usize| {
            let idx = i * m + j;
            let w = self.weights()[idx];
            let x = self.control_points()[idx];
            Vector4::new(w * x.x, w * x.y, w * x.z, w)
        };
        let spans = |k: &[f64], deg: usize, count: usize| -> Vec<usize> {
            (deg..count).filter(|&i| k[i] < k[i + 1]).collect()
        };
        let mut out = Vec::new();
        for iu in spans(ku, p, n) {
            for iv in spans(kv, q, m) {
                // Along `u` in every active column, then along `v`.
                let columns: Vec<Vec<Vector4<f64>>> = (iv - q..=iv)
                    .map(|j| {
                        let active: Vec<_> = (iu - p..=iu).map(|i| homogeneous(i, j)).collect();
                        extract(p, ku, iu, &active)
                    })
                    .collect();
                let mut net = Vec::with_capacity((p + 1) * (q + 1));
                for a in 0..=p {
                    let active: Vec<_> = columns.iter().map(|c| c[a]).collect();
                    net.extend(extract(q, kv, iv, &active));
                }
                out.push(Patch {
                    u: [ku[iu], ku[iu + 1]],
                    v: [kv[iv], kv[iv + 1]],
                    span: [[ku[iu], ku[iu + 1]], [kv[iv], kv[iv + 1]]],
                    degree: [p, q],
                    net,
                    depth: 0,
                });
            }
        }
        out
    }

    /// The nearest point of the surface to `p`, with its `(u, v)`: the
    /// **global** nearest, over every patch, not the nearest local
    /// minimum from a start. The point is on the surface and `distance` is
    /// its distance from `p`; a parameter is exact to the conditioning of
    /// the surface there (a minimum is only as sharp as the square root of
    /// rounding says, and the point and the distance are second-order
    /// better).
    ///
    /// Where two parameters name one point they are reported as one, at
    /// the lower: a point on the seam of a closed direction is reported at
    /// the start of its knots, and one on a collapsed row — a pole — at the
    /// row's own `v` with `u` at the start of its knots, as the sphere's
    /// closed form does. A periodic parameter is reported inside its
    /// domain, `[knots[p], knots[n])`.
    ///
    /// Errors: [`GeomError::Ambiguous`] with
    /// [`AmbiguousLocus::MedialAxis`] where two or more *distinct* points
    /// of the surface are as near as each other to rounding
    /// ([`arris_math::is_negligible`] against the coordinates' magnitude),
    /// or where more patches than the search allows are — never a silent
    /// choice between them. [`GeomError::Degenerate`] for a `p` that is
    /// not finite.
    ///
    /// ```
    /// use arris_geom::NurbsSurface;
    /// use arris_math::{Point3};
    ///
    /// // The bilinear patch that is the unit square of the xy plane.
    /// let patch = NurbsSurface::new(
    ///     [1, 1],
    ///     [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
    ///     vec![
    ///         Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0),
    ///         Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0),
    ///     ],
    ///     vec![1.0; 4],
    /// ).unwrap();
    /// let near = patch.project(Point3::new(0.25, 0.75, 2.0)).unwrap();
    /// assert!((near.uv.x - 0.25).abs() < 1e-12 && (near.uv.y - 0.75).abs() < 1e-12);
    /// assert!((near.distance - 2.0).abs() < 1e-12);
    /// // Off the edge the nearest point is on it.
    /// let edge = patch.project(Point3::new(3.0, 0.5, 0.0)).unwrap();
    /// assert!((edge.point - Point3::new(1.0, 0.5, 0.0)).norm() < 1e-12);
    /// ```
    pub fn project(&self, p: Point3) -> Result<SurfaceProjection, GeomError> {
        if !p.coords.iter().all(|c| c.is_finite()) {
            return Err(GeomError::Degenerate {
                kind: GeomKind::Point,
                reason: format!("the point {p} to project is not finite"),
            });
        }
        let ambiguous = || GeomError::Ambiguous {
            kind: GeomKind::Surface(SurfaceKind::Nurbs),
            locus: AmbiguousLocus::MedialAxis,
            point: p,
        };
        let patches = self.patches();
        // The surface's size, for the leaf; the magnitude of every
        // coordinate that enters a distance, for what is equal to rounding.
        let (extent, reach) = {
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            let mut reach = p.coords.norm();
            for x in self.control_points() {
                for k in 0..3 {
                    lo[k] = lo[k].min(x[k]);
                    hi[k] = hi[k].max(x[k]);
                }
                reach = reach.max(x.coords.norm());
            }
            (
                (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt(),
                reach,
            )
        };
        let slack = RELATIVE_ROUNDING * reach;

        let mut queue = BinaryHeap::new();
        let mut seq = 0;
        let mut best = f64::INFINITY;
        let mut push = |queue: &mut BinaryHeap<Queued>, best: &mut f64, patch: Patch| {
            let [u, v] = patch.middle();
            *best = best.min((self.eval(u, v).point - p).norm());
            let bound = patch.lower_bound(p);
            if bound <= *best + slack {
                queue.push(Queued { bound, seq, patch });
                seq += 1;
            }
        };
        for patch in patches {
            push(&mut queue, &mut best, patch);
        }

        let mut found: Vec<Candidate> = Vec::new();
        let mut taken = 0;
        let budget = NODE_BUDGET + NODES_PER_PATCH * queue.len();
        while let Some(Queued { bound, patch, .. }) = queue.pop() {
            if bound > best + slack {
                break;
            }
            taken += 1;
            if taken > budget {
                return Err(ambiguous());
            }
            if patch.diagonal() <= LEAF_FRACTION * extent || patch.depth >= MAX_DEPTH {
                let candidate =
                    self.refine(p, patch.middle(), [patch.u, patch.v], patch.span, reach);
                best = best.min(candidate.distance);
                found.push(candidate);
            } else {
                for half in patch.halves(patch.longer()) {
                    push(&mut queue, &mut best, half);
                }
            }
        }
        let Some(nearest) = found.iter().map(|c| c.distance).min_by(f64::total_cmp) else {
            return Err(GeomError::Degenerate {
                kind: GeomKind::Surface(SurfaceKind::Nurbs),
                reason: "the search found no candidate to project onto".into(),
            });
        };

        // Every local minimum as near as the nearest, to rounding, in a
        // fixed order; then the distinct points among them.
        let mut ties: Vec<Candidate> = found
            .into_iter()
            .filter(|c| c.distance <= nearest + slack)
            .collect();
        ties.sort_by(|a, b| {
            a.uv[0]
                .total_cmp(&b.uv[0])
                .then(a.uv[1].total_cmp(&b.uv[1]))
        });
        // A minimum is located to the square root of rounding: two starts
        // that reach one minimum agree to that, not to rounding itself.
        let same = RELATIVE_ROUNDING.sqrt() * extent;
        let first = &ties[0];
        if ties
            .iter()
            .any(|c| (c.point - first.point).norm() > same.max(slack))
        {
            return Err(ambiguous());
        }
        // The lowest parameters of the point.
        let uv = first.uv;
        let point = self.eval(uv[0], uv[1]).point;
        Ok(SurfaceProjection {
            uv: Point2::new(uv[0], uv[1]),
            point,
            distance: (point - p).norm(),
        })
    }

    /// The local minimum of the distance to `p` that Newton's iteration
    /// reaches from `start`, across knot spans and, in a direction the
    /// surface closes in ([`NurbsSurface::closure`]), across its seam
    /// without wrapping the parameter: the answer is continuous with
    /// `start`, which is what a pcurve following a curve along the
    /// surface needs, and it may lie outside the domain there. An open
    /// direction is held inside its domain. Not global: a caller that has
    /// no nearby start, or whose answer is not near enough, asks
    /// [`NurbsSurface::project`].
    pub(crate) fn project_from(&self, p: Point3, start: Point2) -> SurfaceProjection {
        let domain = self.domain();
        let closure = self.closure();
        let reach = self
            .control_points()
            .iter()
            .map(|x| x.coords.norm())
            .fold(p.coords.norm(), f64::max);
        let hold = |k: usize, x: f64| match closure[k] {
            Some(_) => x,
            None => x.clamp(domain[k].lo(), domain[k].hi()),
        };
        let mut x = [hold(0, start.x), hold(1, start.y)];
        let mut e = self.eval(x[0], x[1]);
        let mut f = (e.point - p).norm_squared();
        for _ in 0..MAX_STEPS {
            let r = e.point - p;
            let g = [e.du.dot(&r), e.dv.dot(&r)];
            let held = |k: usize| {
                closure[k].is_none()
                    && ((x[k] <= domain[k].lo() && g[k] > 0.0)
                        || (x[k] >= domain[k].hi() && g[k] < 0.0))
            };
            let free = [!held(0), !held(1)];
            let step = newton_step(&e.du, &e.dv, &e.duu, &e.duv, &e.dvv, &r, g, free);
            let mut alpha = 1.0;
            let mut moved = None;
            for _ in 0..MAX_BACKTRACKS {
                let y = [
                    hold(0, x[0] + alpha * step[0]),
                    hold(1, x[1] + alpha * step[1]),
                ];
                let ey = self.eval(y[0], y[1]);
                let fy = (ey.point - p).norm_squared();
                if fy <= f + RELATIVE_ROUNDING * reach * reach {
                    moved = Some((y, ey, fy));
                    break;
                }
                alpha *= 0.5;
            }
            let Some((y, ey, fy)) = moved else { break };
            let travelled =
                ((y[0] - x[0]) * e.du.norm()).abs() + ((y[1] - x[1]) * e.dv.norm()).abs();
            (x, e, f) = (y, ey, fy);
            if is_negligible(travelled, reach) {
                break;
            }
        }
        SurfaceProjection {
            uv: Point2::new(x[0], x[1]),
            point: e.point,
            distance: f.sqrt(),
        }
    }

    /// The local minimum of the distance to `p` inside the knot span `span`
    /// that a projected Newton iteration reaches from `start`, in canonical
    /// parameters. `reach` is the magnitude of the coordinates that enter
    /// the distance, which sets what is zero to rounding.
    ///
    /// The iteration is Newton's on the gradient of the squared distance,
    /// with the Hessian replaced by its Gauss–Newton part when it is not
    /// positive definite, a backtracking search that never accepts a step
    /// that lengthens the distance by more than rounding, and a parameter
    /// held at an end of the span while the gradient pushes it out. It stays
    /// inside one span, where the surface is one polynomial piece, and
    /// evaluates a wall from the inside: a minimum on a kink between two
    /// spans is a minimum of each, held at their shared wall, where an
    /// iteration across it would only oscillate. The neighbouring span's own
    /// leaf finds the same point, which merges.
    fn refine(
        &self,
        p: Point3,
        start: [f64; 2],
        patch: [[f64; 2]; 2],
        span: [[f64; 2]; 2],
        reach: f64,
    ) -> Candidate {
        let domain = self.domain();
        let closure = self.closure();
        let eval = |x: [f64; 2]| {
            // The span's end belongs to the next span, or wraps to the first
            // of a periodic direction: nudged inside, it is the polynomial
            // piece being minimised over.
            let inside = |k: usize| {
                if x[k] >= span[k][1] {
                    below(span[k][1])
                } else {
                    x[k]
                }
            };
            self.eval(inside(0), inside(1))
        };
        let mut x = [
            start[0].clamp(span[0][0], span[0][1]),
            start[1].clamp(span[1][0], span[1][1]),
        ];
        let mut e = eval(x);
        let mut f = (e.point - p).norm_squared();
        // Twice: held inside the patch's own box first, then released to
        // the span. From a small box the iteration cannot cross a corner
        // into a worse basin than the one the patch holds, and the second
        // pass carries the result on to the minimum proper, which may lie
        // beyond the box's wall.
        for limit in [patch, span] {
            let wall = |k: usize, x: f64| x.clamp(limit[k][0], limit[k][1]);
            for _ in 0..MAX_STEPS {
                let r = e.point - p;
                let g = [e.du.dot(&r), e.dv.dot(&r)];
                let held = |k: usize| {
                    (x[k] <= limit[k][0] && g[k] > 0.0) || (x[k] >= limit[k][1] && g[k] < 0.0)
                };
                let free = [!held(0), !held(1)];
                // The line search along one Newton step: the first halving
                // that decreases the distance enough.
                let search = |step: [f64; 2]| {
                    let mut alpha = 1.0;
                    for _ in 0..MAX_BACKTRACKS {
                        let y = [
                            wall(0, x[0] + alpha * step[0]),
                            wall(1, x[1] + alpha * step[1]),
                        ];
                        let ey = eval(y);
                        let fy = (ey.point - p).norm_squared();
                        // Sufficient decrease against the slope along the step
                        // taken (`f` is the squared distance, `g` is half its
                        // gradient): "no worse" lets a step across a narrow
                        // valley zig-zag between its walls without converging.
                        let slope = 2.0 * (g[0] * (y[0] - x[0]) + g[1] * (y[1] - x[1]));
                        if fy <= f + SUFFICIENT_DECREASE * slope + RELATIVE_ROUNDING * reach * reach
                        {
                            return Some((y, ey, fy, alpha));
                        }
                        alpha *= 0.5;
                    }
                    None
                };
                // The step in both parameters. Where it is taken whole it is
                // Newton's, with its quadratic convergence. Where it needed
                // halving, the surface is nearly degenerate in one direction (a
                // collapsing row), the joint step is dominated by it and
                // crawls along the other: each parameter's own step is tried
                // too, and the lowest wins.
                let joint = newton_step(&e.du, &e.dv, &e.duu, &e.duv, &e.dvv, &r, g, free);
                let mut moved = search(joint);
                if moved.as_ref().is_none_or(|m| m.3 < 1.0) {
                    for mask in [[free[0], false], [false, free[1]]] {
                        if mask == [false, false] {
                            continue;
                        }
                        let step = newton_step(&e.du, &e.dv, &e.duu, &e.duv, &e.dvv, &r, g, mask);
                        if let Some(found) = search(step)
                            && moved.as_ref().is_none_or(|m| found.2 < m.2)
                        {
                            moved = Some(found);
                        }
                    }
                }
                let Some((y, ey, fy, _)) = moved else { break };
                let travelled =
                    ((y[0] - x[0]) * e.du.norm()).abs() + ((y[1] - x[1]) * e.dv.norm()).abs();
                (x, e, f) = (y, ey, fy);
                if is_negligible(travelled, reach) {
                    break;
                }
            }
        }
        // The same point at its lowest parameters. A parameter that is
        // within the rounding of the point's coordinates of an end of its
        // domain is that end: a minimum at a wall is approached, not
        // reached, by a Newton iteration on noisy evaluations. The end of a
        // periodic or closed direction is its start, and a collapsed row is
        // one point whatever its `u` says.
        //
        // The iteration locates a parameter to a few times the rounding of
        // the coordinates it works in, which include the query's, however
        // far it is (`LOCATION`): at a pole the parameter along the row is
        // then free to that much, and the row's own derivative, a hair off
        // it, is no longer zero to rounding itself.
        let noise = LOCATION * reach;
        let derivative = [e.du.norm(), e.dv.norm()];
        for k in 0..2 {
            let [lo, hi] = [domain[k].lo(), domain[k].hi()];
            if is_negligible((x[k] - lo) * derivative[k], noise) {
                x[k] = lo;
            } else if is_negligible((hi - x[k]) * derivative[k], noise) {
                x[k] = if closure[k].is_some() { lo } else { hi };
            }
        }
        let derivative = [
            self.eval(x[0], x[1]).du.norm(),
            self.eval(x[0], x[1]).dv.norm(),
        ];
        for k in 0..2 {
            if is_negligible(derivative[k] * domain[k].length(), noise) {
                x[k] = domain[k].lo();
            }
        }
        let point = self.eval(x[0], x[1]).point;
        Candidate {
            uv: x,
            point,
            distance: (point - p).norm(),
        }
    }
}

/// The Newton step on the two parameters that are free: `−H⁻¹g` for the
/// Hessian `H` of half the squared distance when it is positive definite,
/// else the Gauss–Newton step `−(JᵀJ + μI)⁻¹g` with a shift `μ` a small
/// fraction of its trace, which is a descent direction. Zero along a held
/// parameter.
#[allow(clippy::too_many_arguments)] // the surface's derivatives, the residual and the gradient
fn newton_step(
    du: &Vec3,
    dv: &Vec3,
    duu: &Vec3,
    duv: &Vec3,
    dvv: &Vec3,
    r: &Vec3,
    g: [f64; 2],
    free: [bool; 2],
) -> [f64; 2] {
    let (jj, jk, kk) = (du.dot(du), du.dot(dv), dv.dot(dv));
    let h = [
        [jj + duu.dot(r), jk + duv.dot(r)],
        [jk + duv.dot(r), kk + dvv.dot(r)],
    ];
    // A shift is a fraction of the matrix's own size, so it is scale-free.
    const SHIFT: f64 = 1e-3;
    match free {
        [false, false] => [0.0, 0.0],
        [true, false] => [descent_1d(h[0][0], jj, g[0]), 0.0],
        [false, true] => [0.0, descent_1d(h[1][1], kk, g[1])],
        [true, true] => {
            let det = h[0][0] * h[1][1] - h[0][1] * h[1][0];
            let (a, b, c) = if h[0][0] > 0.0 && det > 0.0 {
                (h[0][0], h[0][1], h[1][1])
            } else {
                let mu = SHIFT * (jj + kk);
                (jj + mu, jk, kk + mu)
            };
            let det = a * c - b * b;
            if det > 0.0 {
                [-(c * g[0] - b * g[1]) / det, -(a * g[1] - b * g[0]) / det]
            } else {
                [0.0, 0.0]
            }
        }
    }
}

/// `−g / h` where the curvature `h` is positive, else along the
/// Gauss–Newton curvature `jj`, else no step.
fn descent_1d(h: f64, jj: f64, g: f64) -> f64 {
    if h > 0.0 {
        -g / h
    } else if jj > 0.0 {
        -g / jj
    } else {
        0.0
    }
}
