//! A trim domain for the bounds query, with a certified chord deviation
//! for each segment. Straight joins close gaps between pcurve endpoints.
//!
//! The straight-line homotopy between each pcurve and its chord stays
//! within that chord's reach. Consequently the polygons have the trim's
//! winding at any point clear of all those neighborhoods. A connected
//! rectangle clear of the same neighborhoods lies wholly on one side.

use arris_geom::region2::{Piece, Polygon2, Side};
use arris_math::{
    Interrupted, Interval, Meter, Point2, RELATIVE_ROUNDING, Vec2, real, shift_nearest_uv,
};

#[derive(Debug, Clone, Copy)]
pub(super) struct Segment {
    from: Point2,
    to: Point2,
    pub(super) reach: f64,
}

impl Segment {
    fn new(from: Point2, to: Point2, reach: f64) -> Self {
        let scale = from.coords.abs().max().max(to.coords.abs().max());
        Self {
            from,
            to,
            reach: reach + RELATIVE_ROUNDING * scale,
        }
    }

    fn shifted(self, shift: Vec2) -> Self {
        let rounded = self.reach + RELATIVE_ROUNDING * shift.abs().max();
        Self::new(self.from + shift, self.to + shift, rounded)
    }
}

struct TrimLoop {
    polygon: Polygon2,
    segments: Vec<Segment>,
    uv: [Interval; 2],
}

pub(super) struct TrimDomain {
    loops: Vec<TrimLoop>,
    uv: Option<[Interval; 2]>,
    periods: [Option<f64>; 2],
}

impl TrimDomain {
    pub(super) fn of(
        loops: &[Vec<Piece<'_>>],
        periods: [Option<f64>; 2],
        chord: f64,
        meter: &mut Meter<'_>,
    ) -> Result<Option<Self>, Interrupted> {
        let mut trim_loops = Vec::with_capacity(loops.len());
        for pieces in loops {
            let Some(l) = TrimLoop::read(pieces, periods, chord, meter)? else {
                return Ok(None);
            };
            trim_loops.push(l);
        }
        let uv = trim_loops
            .iter()
            .map(|l| l.uv)
            .reduce(|a, b| [a[0].hull(&b[0]), a[1].hull(&b[1])]);
        Ok(Some(Self {
            loops: trim_loops,
            uv,
            periods,
        }))
    }

    pub(super) fn uv_box(&self) -> Option<[Interval; 2]> {
        self.uv
    }

    /// Winding at a point already proved clear of the trim in every
    /// translate. Each whole loop is translated together.
    pub(super) fn winds_around(
        &self,
        p: Point2,
        meter: &mut Meter<'_>,
    ) -> Result<bool, Interrupted> {
        Ok(self.winding_side(p, false, meter)? == Side::Inside)
    }

    /// `Boundary` means the chords cannot certify the winding at `p`.
    pub(super) fn side(&self, p: Point2, meter: &mut Meter<'_>) -> Result<Side, Interrupted> {
        self.winding_side(p, true, meter)
    }

    fn winding_side(
        &self,
        p: Point2,
        test_reach: bool,
        meter: &mut Meter<'_>,
    ) -> Result<Side, Interrupted> {
        let mut winding = 0_i64;
        for l in &self.loops {
            for (copy, shift) in translates([p, p], corners(l.uv), self.periods).enumerate() {
                if copy > 0 {
                    meter.tick()?;
                }
                let q = p + shift;
                if test_reach
                    && l.segments
                        .iter()
                        .any(|s| distance_to_segment(q, s) <= s.reach)
                {
                    return Ok(Side::Boundary);
                }
                winding += i64::from(l.polygon.winding_number(q));
            }
        }
        Ok(if self.uv.is_none() || winding != 0 {
            Side::Inside
        } else {
            Side::Outside
        })
    }

    /// Chords in every translate that can approach `root`.
    pub(super) fn segments_in(
        &self,
        root: [Interval; 2],
        meter: &mut Meter<'_>,
    ) -> Result<Vec<Segment>, Interrupted> {
        let mut out = Vec::new();
        for l in &self.loops {
            for (copy, shift) in translates(corners(l.uv), corners(root), self.periods).enumerate()
            {
                if copy > 0 {
                    meter.tick()?;
                }
                for s in &l.segments {
                    let s = if shift == Vec2::zeros() {
                        *s
                    } else {
                        s.shifted(shift)
                    };
                    if distance_to_rect(&s, root) <= s.reach {
                        out.push(s);
                    }
                }
            }
        }
        Ok(out)
    }
}

impl TrimLoop {
    fn read(
        pieces: &[Piece<'_>],
        periods: [Option<f64>; 2],
        chord: f64,
        meter: &mut Meter<'_>,
    ) -> Result<Option<Self>, Interrupted> {
        let Some(first) = pieces.first() else {
            return Ok(None);
        };
        let mut segments = Vec::new();
        let mut end = first.first();
        for piece in pieces {
            let shift = shift_nearest_uv(end - piece.first(), periods);
            let Some(mut chords) = piece.curve.chord_segments(piece.range, chord, meter)? else {
                return Ok(None);
            };
            if piece.reversed {
                chords.reverse();
            }
            for (range, reach) in chords {
                let mut from = piece.curve.point(range.lo());
                let mut to = piece.curve.point(range.hi());
                if piece.reversed {
                    std::mem::swap(&mut from, &mut to);
                }
                let mut segment = Segment::new(from, to, reach);
                if shift != Vec2::zeros() {
                    segment = segment.shifted(shift);
                }
                if !segment
                    .from
                    .coords
                    .iter()
                    .chain(segment.to.coords.iter())
                    .all(|x| x.is_finite())
                {
                    return Ok(None);
                }
                join(&mut segments, end, segment.from);
                segments.push(segment);
                end = segment.to;
            }
        }
        // A closed loop must come back to its original chart. A winding
        // around the surface's period needs a seam use to close the region.
        if shift_nearest_uv(end - first.first(), periods) != Vec2::zeros() {
            return Ok(None);
        }
        join(&mut segments, end, first.first());
        let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
        let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for s in &segments {
            for k in 0..2 {
                lo[k] = lo[k].min(s.from[k].min(s.to[k]) - s.reach);
                hi[k] = hi[k].max(s.from[k].max(s.to[k]) + s.reach);
            }
        }
        let (Ok(u), Ok(v)) = (Interval::new(lo.x, hi.x), Interval::new(lo.y, hi.y)) else {
            return Ok(None);
        };
        if !u.is_bounded() || !v.is_bounded() {
            return Ok(None);
        }
        let polygon = Polygon2::from_points(segments.iter().map(|s| s.from));
        Ok(Some(Self {
            polygon,
            segments,
            uv: [u, v],
        }))
    }
}

fn join(segments: &mut Vec<Segment>, from: Point2, to: Point2) {
    if from != to {
        segments.push(Segment::new(from, to, 0.0));
    }
}

fn corners([u, v]: [Interval; 2]) -> [Point2; 2] {
    [Point2::new(u.lo(), v.lo()), Point2::new(u.hi(), v.hi())]
}

/// Whole-period shifts that can bring the source box to the target box.
fn translates(
    source: [Point2; 2],
    target: [Point2; 2],
    periods: [Option<f64>; 2],
) -> impl Iterator<Item = Vec2> {
    let [u, v] = [0, 1].map(|k| {
        let Some(p) = periods[k] else {
            return (0.0, 0, 1.0);
        };
        let first = real::next_down((target[0][k] - source[1][k]) / p).ceil();
        let last = (-real::next_down((source[0][k] - target[1][k]) / p)).floor();
        (first, (last - first) as i64, p)
    });
    let shifts = |(first, steps, period)| (0..=steps).map(move |n| (first + n as f64) * period);
    shifts(u).flat_map(move |du| shifts(v).map(move |dv| Vec2::new(du, dv)))
}

/// The distance from `p` to a segment.
pub(super) fn distance_to_segment(p: Point2, s: &Segment) -> f64 {
    let d = s.to - s.from;
    let len2 = d.norm_squared();
    let t = if len2 > 0.0 {
        ((p - s.from).dot(&d) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (s.from + t * d)).norm()
}

fn distance_to_box(p: Point2, [u, v]: [Interval; 2]) -> f64 {
    let dx = (u.lo() - p.x).max(p.x - u.hi()).max(0.0);
    let dy = (v.lo() - p.y).max(p.y - v.hi()).max(0.0);
    real::hypot(dx, dy)
}

/// The distance between a segment and a rectangle: zero where the segment
/// crosses it, else attained at an end of the segment or a corner of the
/// rectangle, as between any two disjoint convex sets with corners.
pub(super) fn distance_to_rect(s: &Segment, uv: [Interval; 2]) -> f64 {
    if crosses(s, uv) {
        return 0.0;
    }
    let [u, v] = uv;
    let corners = [
        Point2::new(u.lo(), v.lo()),
        Point2::new(u.hi(), v.lo()),
        Point2::new(u.lo(), v.hi()),
        Point2::new(u.hi(), v.hi()),
    ];
    corners
        .iter()
        .map(|&c| distance_to_segment(c, s))
        .chain([distance_to_box(s.from, uv), distance_to_box(s.to, uv)])
        .fold(f64::INFINITY, f64::min)
}

/// `true` when the segment meets the closed rectangle: the segment's
/// parameter clipped by each slab (Liang–Barsky) keeps a range.
fn crosses(s: &Segment, uv: [Interval; 2]) -> bool {
    let d = s.to - s.from;
    let (mut t0, mut t1) = (0.0_f64, 1.0_f64);
    for (axis, range) in uv.iter().enumerate() {
        let (o, dd) = (s.from[axis], d[axis]);
        if dd == 0.0 {
            if o < range.lo() || o > range.hi() {
                return false;
            }
            continue;
        }
        let (a, b) = ((range.lo() - o) / dd, (range.hi() - o) / dd);
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        t0 = t0.max(a);
        t1 = t1.min(b);
        if t0 > t1 {
            return false;
        }
    }
    true
}
