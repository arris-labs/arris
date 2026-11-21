//! Certified pcurve chords used to bound trims in parameter space.

use arris_math::{Interrupted, Interval, Meter, Point2, Vec2, real};

use crate::Curve2;
use crate::curve::sinusoid_range;

pub(crate) struct Span {
    pub(crate) range: Interval,
    control: Vec<(Vec2, f64)>,
}

pub(crate) struct Chord {
    pub(crate) ends: [Point2; 2],
    pub(crate) uv: [Interval; 2],
    pub(crate) deviation: Vec2,
}

impl Span {
    pub(crate) fn chord(&self, curve: &Curve2) -> Option<Chord> {
        let [a, b] = [curve.point(self.range.lo()), curve.point(self.range.hi())];
        if !a
            .coords
            .iter()
            .chain(b.coords.iter())
            .all(|x| x.is_finite())
        {
            return None;
        }
        let mut uv = intervals(a, b)?;
        let mut deviation = Vec2::zeros();
        match curve {
            Curve2::Line { .. } => {}
            Curve2::Circle { frame, radius } => {
                (uv, deviation) = conic_chord(frame, [*radius; 2], self.range, [a, b])?;
            }
            Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                (uv, deviation) =
                    conic_chord(frame, [*major_radius, *minor_radius], self.range, [a, b])?;
            }
            Curve2::Nurbs(_) => {
                // Each rational point is a convex combination of the
                // Cartesian controls. For each control choose its closest
                // point on the segment. Their convex combinations lie on
                // that same segment, giving a componentwise deviation.
                let length = (b - a).norm();
                if !length.is_finite() {
                    return None;
                }
                let direction = if length > 0.0 {
                    (b - a) / length
                } else {
                    Vec2::zeros()
                };
                let mut low = a.coords.zip_map(&b.coords, f64::min);
                let mut high = a.coords.zip_map(&b.coords, f64::max);
                for &(h, w) in &self.control {
                    if !(w > 0.0 && w.is_finite()) {
                        return None;
                    }
                    let p = h / w;
                    if !p.iter().all(|x| x.is_finite()) {
                        return None;
                    }
                    let delta = p - a.coords;
                    let chord = direction * delta.dot(&direction).clamp(0.0, length);
                    deviation = deviation.zip_map(&(delta - chord).abs(), f64::max);
                    low = low.zip_map(&p, f64::min);
                    high = high.zip_map(&p, f64::max);
                }
                uv = intervals(Point2::from(low), Point2::from(high))?;
            }
        }
        Some(Chord {
            ends: [a, b],
            uv,
            deviation,
        })
    }

    pub(crate) fn split(self) -> Option<[Self; 2]> {
        let mid = self.range.midpoint();
        if !(self.range.lo() < mid && mid < self.range.hi()) {
            return None;
        }
        let a = Interval::new(self.range.lo(), mid).ok()?;
        let b = Interval::new(mid, self.range.hi()).ok()?;
        let mut work = self.control;
        let mut left = Vec::with_capacity(work.len());
        let mut right = Vec::with_capacity(work.len());
        if let (Some(&first), Some(&last)) = (work.first(), work.last()) {
            left.push(first);
            right.push(last);
            let fraction = (mid - self.range.lo()) / self.range.length();
            for level in 1..work.len() {
                let length = work.len() - level;
                for i in 0..length {
                    let (a, b) = (work[i], work[i + 1]);
                    work[i] = (
                        (1.0 - fraction) * a.0 + fraction * b.0,
                        (1.0 - fraction) * a.1 + fraction * b.1,
                    );
                }
                left.push(work[0]);
                right.push(work[length - 1]);
            }
            right.reverse();
        }
        Some([
            Self {
                range: a,
                control: left,
            },
            Self {
                range: b,
                control: right,
            },
        ])
    }
}

impl Curve2 {
    /// Consecutive ranges over which the pcurve is within `chord` of
    /// the closed segment through its endpoints. Each segment carries a
    /// certified maximum deviation in (u, v). The meter ticks per split.
    ///
    /// Returns `None` if the range or chord is invalid, the rational
    /// denominator cannot be bounded away from zero, or rounding
    /// prevents subdivision from reaching the specified chord.
    ///
    /// ```
    /// use arris_geom::Curve2;
    /// use arris_math::{Interval, Meter};
    ///
    /// let line = Curve2::along_u(0.0);
    /// let chords = line.chord_segments(Interval::UNIT, 1e-6, &mut Meter::default()).unwrap();
    /// assert_eq!(chords, Some(vec![(Interval::UNIT, 0.0)]));
    /// ```
    pub fn chord_segments(
        &self,
        range: Interval,
        chord: f64,
        meter: &mut Meter<'_>,
    ) -> Result<Option<Vec<(Interval, f64)>>, Interrupted> {
        if !(chord.is_finite() && chord > 0.0) {
            return Ok(None);
        }
        let Some(spans) = self.chord_spans(range) else {
            return Ok(None);
        };
        let mut result = Vec::new();
        let mut pending: Vec<_> = spans.into_iter().rev().collect();
        while let Some(span) = pending.pop() {
            if let Some(certificate) = span.chord(self) {
                let deviation = certificate.deviation.norm();
                if deviation <= chord {
                    result.push((span.range, deviation));
                    continue;
                }
            }
            let Some([a, b]) = span.split() else {
                return Ok(None);
            };
            meter.tick()?;
            pending.extend([b, a]);
        }
        Ok(Some(result))
    }

    pub(crate) fn chord_spans(&self, range: Interval) -> Option<Vec<Span>> {
        if !(range.is_bounded() && range.length() > 0.0 && range.length().is_finite()) {
            return None;
        }
        match self {
            Curve2::Line { .. } | Curve2::Circle { .. } | Curve2::Ellipse { .. } => {
                Some(vec![Span {
                    range,
                    control: Vec::new(),
                }])
            }
            Curve2::Nurbs(curve) => curve
                .bernstein_on(range)?
                .into_iter()
                .map(|span| {
                    Some(Span {
                        range: Interval::new(span.lo, span.hi).ok()?,
                        control: span.control,
                    })
                })
                .collect(),
        }
    }
}

fn conic_chord(
    frame: &arris_math::Frame2,
    [a, b]: [f64; 2],
    range: Interval,
    [first, last]: [Point2; 2],
) -> Option<([Interval; 2], Vec2)> {
    let [x, y] = [frame.x(), frame.y()];
    let axis = |k| {
        let (u, v) = (a * x[k], b * y[k]);
        let [lo, hi] = sinusoid_range(u, v, range);
        let centre = frame.origin()[k];
        let extent = Interval::new(
            (centre + lo).min(first[k]).min(last[k]),
            (centre + hi).max(first[k]).max(last[k]),
        )
        .ok()?;
        let h = range.length();
        Some((extent, real::hypot(u, v) * (h * h / 8.0)))
    };
    let (u, du) = axis(0)?;
    let (v, dv) = axis(1)?;
    Some(([u, v], Vec2::new(du, dv)))
}

pub(crate) fn intervals(a: Point2, b: Point2) -> Option<[Interval; 2]> {
    Some([
        Interval::new(a.x.min(b.x), a.x.max(b.x)).ok()?,
        Interval::new(a.y.min(b.y), a.y.max(b.y)).ok()?,
    ])
}
