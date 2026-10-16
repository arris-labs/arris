//! Integrals over a region of a surface's (u, v) plane bounded by pcurve
//! pieces, by Green's theorem: `∬ f du dv = ∮ G dv` with `G(u, v) =
//! ∫_{u₀}^{u} f(s, v) ds`, the boundary integral taken along each piece
//! and both integrals by Gauss–Legendre quadrature. What the checker's
//! B2 row, `measure` and every mass property integrate with; the caller
//! supplies `f` — `|∂P/∂u × ∂P/∂v|` for an area, `P · (∂P/∂u × ∂P/∂v) / 3`
//! for a volume — over the surface's own parametrisation.

use arris_math::real;
use core::f64::consts::{FRAC_PI_2, PI};

use crate::Surface;
use crate::region2::Piece;

/// Points per Gauss–Legendre interval. Sixteen integrate a polynomial of
/// degree 31 exactly and a full turn of a trigonometric integrand (a
/// conic's `cos t`, `sin t` against a quadric's parametrisation) to below
/// 1e-20 relative, far under any model tolerance. What sixteen points do
/// not resolve is a kink or a knot inside one interval, which is why a
/// conic piece is split at quarter turns and a NURBS piece at its knots
/// before quadrature.
pub const GAUSS_ORDER: usize = 16;

/// Parameters per piece at which the region's least `u` is looked for, to
/// place the inner integral's origin `u₀` at the region's left edge so the
/// strip it integrates over stays inside the region's bounding box.
const ORIGIN_SAMPLES: usize = 8;

/// The most sub-intervals the inner integral is ever split into,
/// whatever step the caller asks for. A quarter-period step over a whole
/// period is four; the cap is there so a step of zero, or one far below
/// the region's own scale, costs bounded time instead of hanging. It is
/// also the most knots of a NURBS surface a direction is split at, so it
/// is as many as a fitted surface has spans along its longest direction
/// ([`crate::MAX_FIT_SPANS`]): a walked blend's surface runs to hundreds
/// (ADR-0062), and one split short of its knots integrates to `1e-5`.
pub const MAX_INNER_INTERVALS: usize = 4096;

/// Values of one parameter where an integral is split: each of `breaks`,
/// repeated by whole periods where there is one, so that a feature of the
/// integrand there is a Gauss–Legendre interval's end and never its
/// middle.
///
/// Guarantees: [`Breaks::NONE`] splits nowhere.
///
/// ```
/// use arris_geom::integrate::Breaks;
/// use core::f64::consts::{FRAC_PI_2, TAU};
///
/// let quarters = Breaks::periodic(vec![0.0, FRAC_PI_2], FRAC_PI_2 * 2.0);
/// assert_eq!(quarters.breaks_in(0.1, 3.5), vec![FRAC_PI_2, 2.0 * FRAC_PI_2]);
/// assert!(Breaks::NONE.breaks_in(0.0, TAU).is_empty());
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Breaks {
    /// The breaks within one period, ascending; all of them without one.
    breaks: Vec<f64>,
    /// The period they repeat by, if any.
    period: Option<f64>,
}

impl Breaks {
    /// No break at all.
    pub const NONE: Breaks = Breaks {
        breaks: Vec::new(),
        period: None,
    };

    /// `breaks`, and nothing else.
    pub fn at(mut breaks: Vec<f64>) -> Breaks {
        breaks.retain(|b| b.is_finite());
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
        Breaks {
            breaks,
            period: None,
        }
    }

    /// `breaks` repeated by whole multiples of `period`.
    pub fn periodic(breaks: Vec<f64>, period: f64) -> Breaks {
        let mut grid = Breaks::at(breaks);
        grid.period = (period.is_finite() && period > 0.0).then_some(period);
        grid
    }

    /// The breaks strictly inside `(lo, hi)`, ascending, at most
    /// [`MAX_INNER_INTERVALS`] of them; `None` past that.
    fn inside(&self, lo: f64, hi: f64) -> Option<Vec<f64>> {
        let mut out = Vec::new();
        match self.period {
            None => out.extend(self.breaks.iter().copied().filter(|&b| lo < b && b < hi)),
            Some(p) => {
                let (first, last) = ((lo / p).floor() - 1.0, (hi / p).ceil() + 1.0);
                if !(first.is_finite() && last.is_finite())
                    || (last - first) * self.breaks.len() as f64 > MAX_INNER_INTERVALS as f64
                {
                    return None;
                }
                let mut k = first;
                while k <= last {
                    out.extend(
                        (self.breaks.iter())
                            .map(|&b| b + k * p)
                            .filter(|&b| lo < b && b < hi),
                    );
                    k += 1.0;
                }
                out.sort_by(f64::total_cmp);
                out.dedup();
            }
        }
        (out.len() < MAX_INNER_INTERVALS).then_some(out)
    }

    /// The breaks strictly inside `(lo, hi)`, ascending.
    pub fn breaks_in(&self, lo: f64, hi: f64) -> Vec<f64> {
        self.inside(lo, hi).unwrap_or_default()
    }

    /// Whether `x` is one of the breaks, exactly: a value the open
    /// intervals of [`Breaks::breaks_in`] on either side of it both leave
    /// out.
    fn hits(&self, x: f64) -> bool {
        match self.period {
            None => self.breaks.contains(&x),
            Some(p) => {
                let k = (x / p).floor();
                [k - 1.0, k, k + 1.0]
                    .iter()
                    .any(|k| self.breaks.iter().any(|&b| b + k * p == x))
            }
        }
    }
}

/// Where [`region_integral`] splits its integrals over a region of a
/// surface: the inner integral in `u` at `inner`, and each boundary piece
/// where its `u` crosses `outer[0]` or its `v` crosses `outer[1]` — a
/// NURBS surface's knots, across which the integrand's derivatives jump,
/// so that the boundary integrand's kink there is an interval's end.
///
/// Guarantees: [`Grid::NONE`] splits nowhere, which is exact for an
/// integrand polynomial in (u, v) of degree below `2 · GAUSS_ORDER`.
///
/// ```
/// use arris_geom::integrate::{Breaks, Grid};
///
/// let grid = Grid { inner: Breaks::at(vec![1.0]), outer: [Breaks::NONE, Breaks::at(vec![2.0])] };
/// assert_eq!(grid.inner.breaks_in(0.0, 3.0), vec![1.0]);
/// assert_eq!(Grid::NONE.inner, Breaks::NONE);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct Grid {
    /// Where the inner integral is split in `u`.
    pub inner: Breaks,
    /// Where a boundary piece is split as its `u` (first) or `v` (second)
    /// crosses a value.
    pub outer: [Breaks; 2],
}

impl Grid {
    /// No split at all: a plane's, whose integrands are polynomials.
    pub const NONE: Grid = Grid {
        inner: Breaks::NONE,
        outer: [Breaks::NONE, Breaks::NONE],
    };
}

/// Where [`region_integral`] splits its integrals on `surface`: the inner
/// integral at every quarter of the surface's period where it turns in
/// `u`, so that
/// no Gauss–Legendre interval spans more than a quarter of an oscillation
/// of the integrand — the same cadence [`region_integral`] already splits
/// a conic boundary piece at. A plane is affine in (u, v) and every
/// integrand this module is used with is a low-degree polynomial along
/// it, which the quadrature is exact for, so it asks for no split at all;
/// a NURBS surface asks for its own knots in `u` (and at least its
/// degree's count of panels), where its polynomial
/// pieces change and, at a knot of multiplicity its degree, its
/// derivatives jump — repeated by its closure where it closes — and for
/// its boundary pieces to be split where they cross its knots in `u` or
/// `v`, where the boundary integrand kinks for the same reason. An
/// elliptic cylinder asks for a sixteenth of a turn: its area element
/// `√(a² sin²u + b² cos²u)` is no trigonometric polynomial, and has
/// complex singularities `atanh(b / a)` off the real axis at `u = 0` and
/// `π`, which the quadrature resolves only when they sit at an interval's
/// end and the interval is short against that distance — at an aspect of
/// eighteen, a quarter turn leaves `1e-7` of the area, a sixteenth
/// `1e-11`.
///
/// ```
/// use arris_geom::Surface;
/// use arris_geom::integrate::{Grid, surface_grid};
/// use arris_math::Frame;
/// use core::f64::consts::{FRAC_PI_2, PI};
///
/// let cylinder = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
/// assert_eq!(surface_grid(&cylinder).inner.breaks_in(0.1, PI), vec![FRAC_PI_2]);
/// assert_eq!(surface_grid(&Surface::Plane { frame: Frame::world() }), Grid::NONE);
/// ```
pub fn surface_grid(surface: &Surface) -> Grid {
    let turn = |parts: u32| Grid {
        inner: Breaks::periodic(
            (0..parts)
                .map(|k| f64::from(k) * 2.0 * PI / f64::from(parts))
                .collect(),
            2.0 * PI,
        ),
        outer: [Breaks::NONE, Breaks::NONE],
    };
    match surface {
        Surface::Plane { .. } => Grid::NONE,
        Surface::Cylinder { .. }
        | Surface::Cone { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. } => turn(4),
        Surface::EllipticCylinder { .. } => turn(16),
        Surface::Nurbs(s) => {
            let closure = s.closure();
            let knots = [0, 1].map(|k| {
                let knots = panels(s.knots()[k], s.degree()[k]);
                match closure[k] {
                    Some(c) => Breaks::periodic(knots, c),
                    None => Breaks::at(knots),
                }
            });
            Grid {
                inner: knots[0].clone(),
                outer: knots,
            }
        }
    }
}

/// A NURBS direction's knots, and where it has fewer spans than its
/// degree, each span cut evenly so the direction has at least `degree`
/// panels: one polynomial piece of degree `p` turns up to `p − 1` times,
/// and a smooth loft's one span round a ring is a whole turn, whose area
/// element one Gauss–Legendre interval leaves `1e-7` of (four leave
/// rounding), as the quarter turn of a torus does.
fn panels(knots: &[f64], degree: usize) -> Vec<f64> {
    let mut ends: Vec<f64> = knots.to_vec();
    ends.dedup();
    let spans = ends.len().saturating_sub(1);
    if spans == 0 || spans >= degree {
        return knots.to_vec();
    }
    let cuts = degree.div_ceil(spans);
    let mut out = knots.to_vec();
    for w in ends.windows(2) {
        for i in 1..cuts {
            out.push(w[0] + (w[1] - w[0]) * i as f64 / cuts as f64);
        }
    }
    out
}

/// The nodes and weights of Gauss–Legendre quadrature of [`GAUSS_ORDER`]
/// on `[-1, 1]`, as `(node, weight)` pairs ascending in the node: the
/// roots of the Legendre polynomial by Newton from the Chebyshev guess,
/// iterated to a fixed point, so every platform computes the same values
/// to rounding.
pub fn gauss_legendre() -> [(f64, f64); GAUSS_ORDER] {
    let n = GAUSS_ORDER;
    let mut table = [(0.0, 0.0); GAUSS_ORDER];
    for (i, slot) in table.iter_mut().enumerate() {
        // The i-th root counted from the top; the guess interleaves them.
        let k = n - i;
        let mut x = real::cos(PI * (k as f64 - 0.25) / (n as f64 + 0.5));
        let mut derivative = 1.0;
        // Newton on P_n(x) with the three-term recurrence; the iteration
        // count bounds the loop, the fixed point ends it.
        for _ in 0..64 {
            let (mut p0, mut p1) = (1.0, x);
            for m in 2..=n {
                let p2 = ((2 * m - 1) as f64 * x * p1 - (m - 1) as f64 * p0) / m as f64;
                p0 = p1;
                p1 = p2;
            }
            derivative = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let step = p1 / derivative;
            let next = x - step;
            if next == x {
                break;
            }
            x = next;
        }
        *slot = (x, 2.0 / ((1.0 - x * x) * derivative * derivative));
    }
    table
}

/// `∫_a^b g(t) dt` by one Gauss–Legendre interval.
fn gauss(a: f64, b: f64, table: &[(f64, f64); GAUSS_ORDER], g: impl Fn(f64) -> f64) -> f64 {
    let half = (b - a) / 2.0;
    let mid = (a + b) / 2.0;
    table
        .iter()
        .map(|&(x, w)| w * g(mid + half * x))
        .sum::<f64>()
        * half
}

/// `∫_a^b g(t) dt` over sub-intervals split at `breaks` (ascending,
/// strictly inside `[a, b]`).
fn gauss_split(
    a: f64,
    b: f64,
    breaks: &[f64],
    table: &[(f64, f64); GAUSS_ORDER],
    g: impl Fn(f64) -> f64,
) -> f64 {
    let mut total = 0.0;
    let mut from = a;
    for &at in breaks {
        total += gauss(from, at, table, &g);
        from = at;
    }
    total + gauss(from, b, table, &g)
}

/// How many sub-intervals each interval between a piece's breaks is
/// sampled at to find where it crosses a grid line: a pcurve piece turns
/// by far less than this many times between its own knots or quarter
/// turns, so each crossing lies between two samples, and bisection places
/// it to rounding.
const CROSSING_SAMPLES: usize = 32;

/// `breaks` of `piece` (ascending, strictly inside its range) with the
/// parameters where its `u` crosses `outer[0]` or its `v` crosses
/// `outer[1]` added, each found by bisection between two samples.
fn crossings(piece: &Piece<'_>, breaks: Vec<f64>, outer: &[Breaks; 2]) -> Vec<f64> {
    if outer.iter().all(|b| *b == Breaks::NONE) {
        return breaks;
    }
    let (lo, hi) = (piece.range.lo(), piece.range.hi());
    let mut knots = vec![lo];
    knots.extend(breaks.iter().copied());
    knots.push(hi);
    let mut samples = Vec::new();
    for w in knots.windows(2) {
        for i in 0..CROSSING_SAMPLES {
            samples.push(w[0] + (w[1] - w[0]) * i as f64 / CROSSING_SAMPLES as f64);
        }
    }
    samples.push(hi);
    let at = |t: f64| piece.curve.point(t);
    let mut out = breaks;
    // A sample exactly on a grid line: the windows on either side of it
    // are open there and both miss it.
    for &t in &samples {
        if lo < t && t < hi && (0..2).any(|k| outer[k].hits(at(t)[k])) {
            out.push(t);
        }
    }
    for w in samples.windows(2) {
        let (a, b) = (at(w[0]), at(w[1]));
        for (k, grid) in outer.iter().enumerate() {
            let (x, y) = (a[k].min(b[k]), a[k].max(b[k]));
            for value in grid.breaks_in(x, y) {
                // The side of `value` the sample at `lo` is on, kept.
                let (mut s, mut e) = (w[0], w[1]);
                let below = a[k] < value;
                for _ in 0..64 {
                    let m = 0.5 * (s + e);
                    if m <= s || m >= e {
                        break;
                    }
                    if (at(m)[k] < value) == below {
                        s = m;
                    } else {
                        e = m;
                    }
                }
                let t = 0.5 * (s + e);
                if lo < t && t < hi {
                    out.push(t);
                }
            }
        }
    }
    out.sort_by(f64::total_cmp);
    out.dedup();
    out
}

/// Where a piece's parameter range is cut before quadrature: a conic at
/// every quarter turn, a NURBS at its interior knots, a line nowhere.
fn breaks_of(piece: &Piece<'_>) -> Vec<f64> {
    match piece.curve {
        crate::Curve2::Line { .. } => Vec::new(),
        crate::Curve2::Circle { .. } | crate::Curve2::Ellipse { .. } => {
            let (lo, hi) = (piece.range.lo(), piece.range.hi());
            let mut breaks = Vec::new();
            let mut k = (lo / FRAC_PI_2).floor() + 1.0;
            while k * FRAC_PI_2 < hi {
                let at = k * FRAC_PI_2;
                if at > lo {
                    breaks.push(at);
                }
                k += 1.0;
            }
            breaks
        }
        crate::Curve2::Nurbs(_) => piece.interior_knots(),
    }
}

/// `∬_R f(u, v) du dv` over the region `R` bounded by `pieces` walked in
/// order — counter-clockwise for a positive result, so the result carries
/// the sign of the loop's turn and a face's hole loops subtract
/// themselves — by Green's theorem along the pieces with Gauss–Legendre
/// quadrature of [`GAUSS_ORDER`] points per interval, each piece split
/// at its quarter turns or knots. `f` is evaluated across the strip
/// between the region's least `u` and each boundary point, so it must be
/// defined over the region's bounding box in `u`. The inner integral is
/// itself split at every break of `grid.inner` it crosses — the surface's
/// own quarter turns or knots ([`surface_grid`]), and each piece where it
/// crosses `grid.outer`, so an integrand's feature
/// there is an interval's end and never its middle — at most
/// [`MAX_INNER_INTERVALS`] intervals, beyond which it falls back to that
/// many equal ones: [`Grid::NONE`] takes it in one, which is exact
/// for an `f` polynomial in `u` of degree below `2 · GAUSS_ORDER`, and
/// [`surface_grid`] of the surface `f` evaluates is what a caller
/// integrating over a curved surface passes, since a strip that spans a
/// whole turn of `cos u` is not one interval's work. An empty `pieces` is
/// zero.
///
/// ```
/// use arris_geom::integrate::{Grid, region_integral};
/// use arris_geom::region2::Piece;
/// use arris_geom::Curve2;
/// use arris_math::{Frame2, Interval};
/// use core::f64::consts::PI;
///
/// let circle = Curve2::Circle { frame: Frame2::identity(), radius: 3.0 };
/// let area = region_integral(
///     &[Piece::along(&circle, Interval::TURN)],
///     &Grid::NONE,
///     |_, _| 1.0,
/// );
/// assert!((area - 9.0 * PI).abs() < 1e-12);
/// ```
pub fn region_integral(pieces: &[Piece<'_>], grid: &Grid, f: impl Fn(f64, f64) -> f64) -> f64 {
    let table = gauss_legendre();
    let u0 = pieces
        .iter()
        .flat_map(|piece| {
            (0..=ORIGIN_SAMPLES).map(move |i| {
                piece
                    .curve
                    .point(piece.range.lerp(i as f64 / ORIGIN_SAMPLES as f64))
                    .x
            })
        })
        .fold(f64::INFINITY, f64::min);
    if !u0.is_finite() {
        return 0.0;
    }
    let inner = |u: f64, v: f64| {
        let g = |s: f64| f(s, v);
        let (lo, hi) = (u0.min(u), u0.max(u));
        let sign = if u >= u0 { 1.0 } else { -1.0 };
        match grid.inner.inside(lo, hi) {
            Some(breaks) => sign * gauss_split(lo, hi, &breaks, &table, g),
            None => {
                let step = (hi - lo) / MAX_INNER_INTERVALS as f64;
                sign * (0..MAX_INNER_INTERVALS)
                    .map(|i| {
                        let a = lo + i as f64 * step;
                        gauss(a, a + step, &table, g)
                    })
                    .sum::<f64>()
            }
        }
    };
    let mut total = 0.0;
    for piece in pieces {
        let breaks = crossings(piece, breaks_of(piece), &grid.outer);
        let along = gauss_split(piece.range.lo(), piece.range.hi(), &breaks, &table, |t| {
            let e = piece.curve.eval(t);
            inner(e.point.x, e.point.y) * e.d1.y
        });
        total += if piece.reversed { -along } else { along };
    }
    total
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Curve2;
    use arris_math::{Frame2, Interval, Point2, UnitVec2, Vec2};
    use core::f64::consts::TAU;

    /// `parts` equal parts of a turn, repeated by it.
    fn turn(parts: u32) -> Grid {
        let step = TAU / f64::from(parts);
        Grid {
            inner: Breaks::periodic((0..parts).map(|k| f64::from(k) * step).collect(), TAU),
            outer: [Breaks::NONE, Breaks::NONE],
        }
    }

    /// A closed periodic NURBS loop walked as two blocks, the second
    /// wrapping past the end of the knots, encloses what the whole loop
    /// does: the wrapped block is split at the knots a period on, not
    /// taken in one interval across them.
    /// A grid line a boundary piece's crossing samples land on exactly
    /// still splits it: `|v − 1/4|` over the unit square, whose kink at
    /// `v = 1/4` is a sample of a side from 0 to 1.
    #[test]
    fn a_grid_line_on_a_crossing_sample_splits_the_piece() {
        let side = |origin: Point2, direction: Vec2| Curve2::Line {
            origin,
            direction: UnitVec2::new_unchecked(direction),
        };
        let (bottom, right) = (
            side(Point2::new(0.0, 0.0), Vec2::x()),
            side(Point2::new(1.0, 0.0), Vec2::y()),
        );
        let (top, left) = (
            side(Point2::new(0.0, 1.0), Vec2::x()),
            side(Point2::new(0.0, 0.0), Vec2::y()),
        );
        let unit = Interval::new(0.0, 1.0).unwrap();
        let square = [
            Piece::along(&bottom, unit),
            Piece::along(&right, unit),
            Piece::against(&top, unit),
            Piece::against(&left, unit),
        ];
        let grid = Grid {
            inner: Breaks::NONE,
            outer: [Breaks::NONE, Breaks::at(vec![0.25])],
        };
        let got = region_integral(&square, &grid, |_, v| (v - 0.25).abs());
        assert!((got - 0.3125).abs() <= 1e-15, "{got}");
    }

    #[test]
    fn a_block_wrapping_a_periodic_pcurve_is_split_at_its_knots() {
        use crate::NurbsCurve2;
        let n = 12;
        let ring: Vec<Point2> = (0..n)
            .map(|i| {
                let a = TAU * i as f64 / n as f64;
                let r = 2.0 + 0.7 * real::cos(3.0 * a);
                Point2::new(r * real::cos(a), r * real::sin(a))
            })
            .collect();
        let degree = 3;
        let points: Vec<Point2> = ring.iter().chain(&ring[..degree]).copied().collect();
        let knots: Vec<f64> = (0..points.len() + degree + 1).map(|i| i as f64).collect();
        let weights = vec![1.0; points.len()];
        let curve = Curve2::Nurbs(NurbsCurve2::new(degree, knots, points, weights).unwrap());
        let period = curve.period().unwrap();
        let lo = curve.domain().lo();
        let whole = region_integral(
            &[Piece::along(
                &curve,
                Interval::new(lo, lo + period).unwrap(),
            )],
            &Grid::NONE,
            |u, v| 1.0 + u * v,
        );
        let (a, b) = (lo + 0.6 * period, lo + 0.9 * period);
        let blocks = region_integral(
            &[
                Piece::along(&curve, Interval::new(a, b).unwrap()),
                Piece::along(&curve, Interval::new(b, a + period).unwrap()),
            ],
            &Grid::NONE,
            |u, v| 1.0 + u * v,
        );
        assert!(whole.abs() > 1.0);
        assert!(
            (blocks - whole).abs() < 1e-12 * whole.abs(),
            "{blocks} {whole}"
        );
    }

    #[test]
    fn nodes_are_symmetric_and_weights_sum_to_two() {
        let table = gauss_legendre();
        let sum: f64 = table.iter().map(|&(_, w)| w).sum();
        assert!((sum - 2.0).abs() < 1e-15);
        for i in 0..GAUSS_ORDER / 2 {
            let (x, w) = table[i];
            let (y, u) = table[GAUSS_ORDER - 1 - i];
            assert!((x + y).abs() < 1e-15 && (w - u).abs() < 1e-15);
        }
        assert!(table.windows(2).all(|p| p[0].0 < p[1].0));
        // Exact for x^30 over [-1, 1]: 2 / 31.
        let table = gauss_legendre();
        let v = gauss(-1.0, 1.0, &table, |x| real::powi(x, 30));
        assert!((v - 2.0 / 31.0).abs() < 1e-15, "{v}");
    }

    #[test]
    fn a_rectangle_walked_either_way_integrates_a_polynomial() {
        let line = |ox: f64, oy: f64, dx: f64, dy: f64| Curve2::Line {
            origin: Point2::new(ox, oy),
            direction: UnitVec2::new_normalize(Vec2::new(dx, dy)),
        };
        let (w, h) = (4.0, 3.0);
        let sides = [
            line(0.0, 0.0, 1.0, 0.0),
            line(w, 0.0, 0.0, 1.0),
            line(w, h, -1.0, 0.0),
            line(0.0, h, 0.0, -1.0),
        ];
        let ranges = [w, h, w, h].map(|l| Interval::new(0.0, l).unwrap());
        let pieces: Vec<Piece<'_>> = sides
            .iter()
            .zip(ranges)
            .map(|(c, r)| Piece::along(c, r))
            .collect();
        // ∬ u² v du dv = w³/3 · h²/2.
        let v = region_integral(&pieces, &Grid::NONE, |u, v| u * u * v);
        assert!(
            (v - real::powi(w, 3) / 3.0 * h * h / 2.0).abs() < 1e-12,
            "{v}"
        );
        let reversed: Vec<Piece<'_>> = sides
            .iter()
            .zip(ranges)
            .rev()
            .map(|(c, r)| Piece::against(c, r))
            .collect();
        let v = region_integral(&reversed, &Grid::NONE, |_, _| 1.0);
        assert!((v + w * h).abs() < 1e-12, "{v}");
    }

    #[test]
    fn the_inner_step_splits_what_one_interval_does_not_resolve() {
        // A whole turn of `cos⁴ u` over [0, 2π] × [0, 1]: `2π · 3/8`. The
        // inner integral runs from `u₀ = 0` to each boundary point, so
        // one Gauss interval has to carry a full turn of the integrand
        // and leaves a thousand times more than rounding; at a quarter
        // turn a step it is exact.
        let (w, h) = (TAU, 1.0);
        let line = |ox: f64, oy: f64, dx: f64, dy: f64| Curve2::Line {
            origin: Point2::new(ox, oy),
            direction: UnitVec2::new_normalize(Vec2::new(dx, dy)),
        };
        let sides = [
            line(0.0, 0.0, 1.0, 0.0),
            line(w, 0.0, 0.0, 1.0),
            line(w, h, -1.0, 0.0),
            line(0.0, h, 0.0, -1.0),
        ];
        let ranges = [w, h, w, h].map(|l| Interval::new(0.0, l).unwrap());
        let pieces: Vec<Piece<'_>> = sides
            .iter()
            .zip(ranges)
            .map(|(c, r)| Piece::along(c, r))
            .collect();
        let exact = TAU * 3.0 / 8.0;
        let f = |u: f64, _: f64| real::powi(real::cos(u), 4);
        let coarse = region_integral(&pieces, &Grid::NONE, f);
        let fine = region_integral(&pieces, &turn(4), f);
        assert!((coarse - exact).abs() > 1e-12, "{coarse}");
        assert!((fine - exact).abs() < 1e-14, "{fine}");
    }

    #[test]
    fn the_inner_split_is_aligned_to_the_grid_and_resolves_an_ellipses_area_element() {
        // The area element of an elliptic cylinder of aspect eighteen over
        // `[0, 2π] × [0, 1]`: the perimeter of the ellipse, against a fine
        // composite Simpson reference. Aligned sixteenth turns hold it to
        // `1e-11`; a whole turn's worth of equal intervals one longer than
        // the grid — what a span a rounding past `2π` used to get — does
        // not.
        let (a, b) = (9.0, 0.5);
        let g = |u: f64| {
            (a * a * real::powi(real::sin(u), 2) + b * b * real::powi(real::cos(u), 2)).sqrt()
        };
        let n = 400_000;
        let h = TAU / n as f64;
        let reference = (1..n)
            .map(|i| if i % 2 == 1 { 4.0 } else { 2.0 } * g(i as f64 * h))
            .sum::<f64>()
            + g(0.0)
            + g(TAU);
        let reference = reference * h / 3.0;
        let line = |ox: f64, oy: f64, dx: f64, dy: f64| Curve2::Line {
            origin: Point2::new(ox, oy),
            direction: UnitVec2::new_normalize(Vec2::new(dx, dy)),
        };
        // The strip's far side a rounding past the turn.
        let w = TAU + 4.0 * f64::EPSILON;
        let sides = [
            line(0.0, 0.0, 1.0, 0.0),
            line(w, 0.0, 0.0, 1.0),
            line(w, 1.0, -1.0, 0.0),
            line(0.0, 1.0, 0.0, -1.0),
        ];
        let ranges = [w, 1.0, w, 1.0].map(|l| Interval::new(0.0, l).unwrap());
        let pieces: Vec<Piece<'_>> = sides
            .iter()
            .zip(ranges)
            .map(|(c, r)| Piece::along(c, r))
            .collect();
        let area = region_integral(&pieces, &turn(16), |u, _| g(u));
        assert!(
            ((area - reference) / reference).abs() < 1e-11,
            "{area} vs {reference}"
        );
        // The grid also leaves the trigonometric cases exact.
        let exact = TAU * 3.0 / 8.0;
        let fine = region_integral(&pieces, &turn(4), |u, _| real::powi(real::cos(u), 4));
        assert!((fine - exact).abs() < 1e-13, "{fine}");
    }

    #[test]
    fn a_circle_walked_clockwise_is_negative_and_quarter_turns_split() {
        let cw = Curve2::Circle {
            frame: Frame2::new(
                Point2::new(1.0, 2.0),
                Vec2::x(),
                arris_math::Handedness::Left,
            )
            .unwrap(),
            radius: 2.0,
        };
        let piece = Piece::along(&cw, Interval::TURN);
        assert_eq!(breaks_of(&piece).len(), 3);
        let v = region_integral(&[piece], &Grid::NONE, |_, _| 1.0);
        assert!((v + 4.0 * PI).abs() < 1e-12, "{v}");
        let arc = Piece::along(&cw, Interval::new(0.3, 5.0).unwrap());
        assert_eq!(breaks_of(&arc), [FRAC_PI_2, PI, 3.0 * FRAC_PI_2]);
    }
}
