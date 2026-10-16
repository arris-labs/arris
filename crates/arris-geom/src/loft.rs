//! Loft sections: a profile loop as one closed NURBS, and sections made
//! compatible (ADR-0065).
//!
//! [`LoopNurbs`] is a loop of a [`Profile`] as one rational quadratic
//! curve split at its vertices by knots of multiplicity two, so its net is
//! the segments' nets end to end. [`match_sections`] takes the sections of
//! a loft and returns, for every loop, one [`LoopNurbs`] per section with
//! the same number of segments, the same knot vector and the segments in
//! correspondence: what a skin through them needs.

use arris_math::{Interval, Tolerance};

use crate::integrate::gauss_legendre;
use crate::{
    Curve, CurveKind, GeomError, GeomKind, NurbsCurve, Profile, ProfileEdge, ProfileError,
};

/// The degree every segment of a loop is raised to, so the loop is one
/// NURBS: a conic's rational quadratic. A structural fact, not a
/// tolerance.
pub const LOOP_DEGREE: usize = 2;

/// The most sections a smooth loft interpolates, its first counted twice
/// when closed: one polynomial through more oscillates, and Open CASCADE
/// caps the degree at 8 and approximates instead (ADR-0065 §7). A bound
/// on the construction, not a tolerance.
pub const MAX_SMOOTH_SECTIONS: usize = 9;

/// Sub-intervals an arc length is integrated over per edge; the integrand
/// of a line or a circle is constant, an ellipse's is smooth, and sixteen
/// Gauss points on each eighth hold it to rounding.
const LENGTH_PIECES: usize = 8;

/// Two knots of the unit interval closer than this are one: the spread of
/// the same fraction computed twice. A structural constant, not a model
/// tolerance — a knot is a parameter of the matching itself.
const KNOT_COINCIDENCE: f64 = 1e-9;

/// One segment's share of a loop's [`LoopNurbs`]: the first of its control
/// points in the loop's net, how many it has, and its own knots, which run
/// over the segment's own parameter range.
#[derive(Debug, Clone, PartialEq)]
pub struct LoopPiece {
    /// Index of the segment's first control point in the loop's net.
    pub first: usize,
    /// How many control points the segment has.
    pub count: usize,
    /// The segment's own knots.
    pub knots: Vec<f64>,
}

/// A loop of a profile as one closed NURBS on the profile's plane: each
/// segment rational quadratic — a line raised from degree one, a circle
/// or an ellipse as its exact arcs — and consecutive segments sharing the
/// control point at their vertex, with a knot of multiplicity two there,
/// so the net is the segments' nets end to end and splits back into them
/// exactly. `segments` are those pieces as curves, on the loop's own
/// control points, each over its segment's parameter range.
///
/// ```
/// use arris_geom::{LoopNurbs, Profile, ProfileLoop, ProfileSegment};
/// use arris_math::{Frame, Point2, Precision};
///
/// let p = |u, v| Point2::new(u, v);
/// let square = Profile {
///     plane: Frame::world(),
///     outer: ProfileLoop::Path {
///         start: p(0.0, 0.0),
///         segments: vec![
///             ProfileSegment::LineTo(p(2.0, 0.0)),
///             ProfileSegment::LineTo(p(2.0, 2.0)),
///             ProfileSegment::LineTo(p(0.0, 2.0)),
///             ProfileSegment::LineTo(p(0.0, 0.0)),
///         ],
///     },
///     holes: vec![],
/// };
/// let edges = square.edges(Precision::DEFAULT.tolerance()).unwrap();
/// let nurbs = LoopNurbs::new(&edges[0]).unwrap();
/// assert_eq!(nurbs.segments.len(), 4);
/// let c = &nurbs.curve;
/// assert!((c.eval(c.domain().lo()).point - c.eval(c.domain().hi()).point).norm() < 1e-15);
/// ```
#[derive(Debug, Clone)]
pub struct LoopNurbs {
    /// The whole loop.
    pub curve: NurbsCurve,
    /// Where each segment sits in the loop's net.
    pub pieces: Vec<LoopPiece>,
    /// Each segment as its own curve.
    pub segments: Vec<NurbsCurve>,
}

/// The exact rational quadratic of one curve over its range: at the
/// curve's own parameter for a line, and at the arc's angle at its ends
/// for a circle or an ellipse ([`NurbsCurve::circle`]).
fn piece_nurbs(curve: &Curve, range: Interval, pcurve: GeomKind) -> Result<NurbsCurve, GeomError> {
    let (lo, hi) = (range.lo(), range.hi());
    let nurbs = match curve {
        Curve::Line { .. } => {
            let (a, b) = (curve.point(lo), curve.point(hi));
            NurbsCurve::new(
                LOOP_DEGREE,
                vec![lo, lo, lo, hi, hi, hi],
                vec![a, a + (b - a) * 0.5, b],
                vec![1.0; 3],
            )
        }
        &Curve::Circle { ref frame, radius } => NurbsCurve::circle(frame, radius, range),
        &Curve::Ellipse {
            ref frame,
            major_radius,
            minor_radius,
        } => NurbsCurve::ellipse(frame, major_radius, minor_radius, range),
        // `Profile::edges` makes lines, circles and ellipses only.
        Curve::Nurbs(_) => return Err(unsupported(curve, pcurve)),
    }?;
    if nurbs.degree() != LOOP_DEGREE {
        return Err(unsupported(curve, pcurve));
    }
    Ok(nurbs)
}

fn unsupported(curve: &Curve, pcurve: GeomKind) -> GeomError {
    GeomError::Unsupported {
        a: GeomKind::Curve(curve.kind()),
        b: pcurve,
    }
}

/// The segments `pieces`, walked in order, as one [`LoopNurbs`]. Each
/// segment after the first keeps the one before's last control point as
/// its first, its weights scaled to agree there — a rational curve's
/// weights scaled together are the same curve — and the last segment ends
/// on the first's first control point, so the loop closes exactly and the
/// two columns a skin fits for that point are one.
fn assemble(parts: &[NurbsCurve]) -> Result<LoopNurbs, GeomError> {
    let mut knots: Vec<f64> = Vec::new();
    let mut points = Vec::new();
    let mut weights: Vec<f64> = Vec::new();
    let mut pieces = Vec::with_capacity(parts.len());
    for c in parts {
        let (k, p, w) = (c.knots(), c.control_points(), c.weights());
        match (knots.last().copied(), weights.last().copied()) {
            (Some(end), Some(w_end)) => {
                let shift = end - k[0];
                let scale = w_end / w[0];
                pieces.push(LoopPiece {
                    first: points.len() - 1,
                    count: p.len(),
                    knots: k.to_vec(),
                });
                knots.extend(k[LOOP_DEGREE + 1..k.len() - 1].iter().map(|t| t + shift));
                points.extend_from_slice(&p[1..]);
                weights.extend(w[1..].iter().map(|x| x * scale));
            }
            _ => {
                pieces.push(LoopPiece {
                    first: 0,
                    count: p.len(),
                    knots: k.to_vec(),
                });
                knots.extend_from_slice(&k[..k.len() - 1]);
                points.extend_from_slice(p);
                weights.extend_from_slice(w);
            }
        }
    }
    let (Some(&end), Some(&first)) = (knots.last(), points.first()) else {
        return Err(GeomError::Degenerate {
            kind: GeomKind::Curve(CurveKind::Nurbs),
            reason: "a profile loop with a segment".to_string(),
        });
    };
    knots.push(end);
    if let Some(last) = points.last_mut() {
        *last = first;
    }
    let segments = pieces
        .iter()
        .map(|piece| {
            let rows = piece.first..piece.first + piece.count;
            NurbsCurve::new(
                LOOP_DEGREE,
                piece.knots.clone(),
                points[rows.clone()].to_vec(),
                weights[rows].to_vec(),
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let curve = NurbsCurve::new(LOOP_DEGREE, knots, points, weights)?;
    Ok(LoopNurbs {
        curve,
        pieces,
        segments,
    })
}

impl LoopNurbs {
    /// The loop `edges`, walked as oriented, as one closed NURBS.
    ///
    /// Errors: [`GeomError::Unsupported`] for an edge that is not a line,
    /// a circle or an ellipse (`Profile::edges` makes no other), and
    /// [`GeomError::Degenerate`] for no edges.
    pub fn new(edges: &[ProfileEdge]) -> Result<Self, GeomError> {
        let parts = edges
            .iter()
            .map(|e| piece_nurbs(&e.curve, e.range, GeomKind::Curve2(e.pcurve.kind())))
            .collect::<Result<Vec<_>, _>>()?;
        assemble(&parts)
    }
}

/// Why sections cannot be matched into a loft.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum LoftError {
    /// Fewer sections than a loft needs: two, or three when closed.
    #[error("a loft needs at least {needed} sections, found {given}")]
    TooFewSections {
        /// How many were given.
        given: usize,
        /// How many the loft needs.
        needed: usize,
    },
    /// A smooth loft through more sections than one polynomial takes,
    /// [`MAX_SMOOTH_SECTIONS`], the first counted twice when closed.
    #[error(
        "a smooth loft takes at most {max} sections, the first twice when closed; found {given}"
    )]
    TooManySections {
        /// How many were given, the first twice when closed.
        given: usize,
        /// The most it takes.
        max: usize,
    },
    /// A section has a different number of loops from the first.
    #[error("section {section} has {found} loops, the first has {expected}")]
    LoopCount {
        /// The section.
        section: usize,
        /// The first section's loop count.
        expected: usize,
        /// This section's.
        found: usize,
    },
    /// A section is not a profile.
    #[error("section {section}: {source}")]
    Profile {
        /// The section.
        section: usize,
        /// Why.
        source: ProfileError,
    },
    /// A loop of a section has no length to match by, or no curve a
    /// rational quadratic holds.
    #[error("section {section}, loop {loop_index} cannot be matched: {source}")]
    Loop {
        /// The section.
        section: usize,
        /// The loop: `0` the outer, the holes from `1`.
        loop_index: usize,
        /// Why.
        source: GeomError,
    },
    /// A section is the same as the one before it, loop for loop: the span
    /// between them has no extent.
    #[error("section {section} coincides with section {previous}")]
    Coincident {
        /// The later section.
        section: usize,
        /// The one before it.
        previous: usize,
    },
}

/// One loop of one section made compatible with the same loop of every
/// other section.
#[derive(Debug, Clone)]
pub struct MatchedLoop {
    /// The loop, its segments in correspondence across sections.
    pub nurbs: LoopNurbs,
    /// For each matched segment, the consumer's segment of this section
    /// it came from (`ProfileEdge::segment`); a split segment's pieces
    /// share it.
    pub sources: Vec<usize>,
}

/// One section of a loft: its loops, compatible with the other sections'.
#[derive(Debug, Clone)]
pub struct MatchedSection {
    /// The loops: `0` the outer, the holes from `1`.
    pub loops: Vec<MatchedLoop>,
}

/// A stretch of a loop: a curve over a range, from the consumer's segment
/// `source`.
struct Stretch {
    curve: Curve,
    range: Interval,
    pcurve: GeomKind,
    source: usize,
}

/// The arc length of `curve` over `[lo, hi]`.
fn arc_length(curve: &Curve, lo: f64, hi: f64) -> f64 {
    let rule = gauss_legendre();
    let h = (hi - lo) / LENGTH_PIECES as f64;
    let mut total = 0.0;
    for i in 0..LENGTH_PIECES {
        let a = lo + h * i as f64;
        let mid = a + 0.5 * h;
        for &(x, w) in &rule {
            total += 0.5 * h * w * curve.eval(mid + 0.5 * h * x).d1.norm();
        }
    }
    total
}

/// The parameter in `[lo, hi]` at arc length `target` from `lo`, by
/// bisection to the width of the interval's rounding.
fn parameter_at(curve: &Curve, lo: f64, hi: f64, target: f64) -> f64 {
    let (mut a, mut b) = (lo, hi);
    for _ in 0..80 {
        let mid = 0.5 * (a + b);
        if arc_length(curve, lo, mid) < target {
            a = mid;
        } else {
            b = mid;
        }
    }
    0.5 * (a + b)
}

/// The matched sections' loop `loop_index`: every section's stretches cut
/// where any section has a vertex, at the arc-length fractions measured
/// from each loop's start; loops with the same segment count are left as
/// they are, segment for segment.
fn matched_stretches(loops: &[&[ProfileEdge]], tol: Tolerance) -> Vec<Vec<Stretch>> {
    let stretches: Vec<Vec<Stretch>> = loops
        .iter()
        .map(|edges| {
            edges
                .iter()
                .map(|e| Stretch {
                    curve: e.curve.clone(),
                    range: e.range,
                    pcurve: GeomKind::Curve2(e.pcurve.kind()),
                    source: e.segment,
                })
                .collect()
        })
        .collect();
    let count = stretches[0].len();
    if stretches.iter().all(|s| s.len() == count) {
        return stretches;
    }
    // Each loop's vertices as fractions of its length from its start.
    let lengths: Vec<Vec<f64>> = stretches
        .iter()
        .map(|s| {
            s.iter()
                .map(|t| arc_length(&t.curve, t.range.lo(), t.range.hi()))
                .collect()
        })
        .collect();
    let perimeters: Vec<f64> = lengths.iter().map(|l| l.iter().sum()).collect();
    let shortest = perimeters.iter().copied().fold(f64::INFINITY, f64::min);
    let eps = tol.linear / shortest;
    let mut cuts: Vec<f64> = vec![0.0];
    for (l, perimeter) in lengths.iter().zip(&perimeters) {
        let mut run = 0.0;
        for len in l {
            cuts.push(run / perimeter);
            run += len;
        }
    }
    cuts.sort_by(f64::total_cmp);
    let mut union: Vec<f64> = Vec::new();
    for c in cuts {
        if union.last().is_none_or(|&last| c - last > eps) {
            union.push(c);
        }
    }
    stretches
        .into_iter()
        .zip(lengths.iter().zip(&perimeters))
        .map(|(stretch, (l, perimeter))| {
            let mut out = Vec::new();
            let mut run = 0.0;
            for (t, len) in stretch.into_iter().zip(l) {
                let (lo, hi) = (t.range.lo(), t.range.hi());
                let (from, to) = (run / perimeter, (run + len) / perimeter);
                let mut at = lo;
                for &u in union.iter().filter(|&&u| u > from + eps && u < to - eps) {
                    let split = parameter_at(&t.curve, lo, hi, (u * perimeter) - run);
                    if let Ok(range) = Interval::new(at, split) {
                        out.push(Stretch {
                            curve: t.curve.clone(),
                            range,
                            pcurve: t.pcurve,
                            source: t.source,
                        });
                        at = split;
                    }
                }
                if let Ok(range) = Interval::new(at, hi) {
                    out.push(Stretch {
                        curve: t.curve,
                        range,
                        pcurve: t.pcurve,
                        source: t.source,
                    });
                }
                run += len;
            }
            out
        })
        .collect()
}

/// `curve` reparametrised affinely onto `[0, 1]`.
fn unit_range(curve: &NurbsCurve) -> Result<NurbsCurve, GeomError> {
    let (lo, hi) = (curve.domain().lo(), curve.domain().hi());
    let knots = curve.knots().iter().map(|k| (k - lo) / (hi - lo)).collect();
    NurbsCurve::new(
        curve.degree(),
        knots,
        curve.control_points().to_vec(),
        curve.weights().to_vec(),
    )
}

/// The interior knots of every curve of `pieces` (each on `[0, 1]`) as one
/// ascending list, a value as often as the curve that has it most often.
fn knot_union(pieces: &[NurbsCurve]) -> Vec<(f64, usize)> {
    let mut union: Vec<(f64, usize)> = Vec::new();
    for c in pieces {
        let k = c.knots();
        let inner = &k[c.degree() + 1..k.len() - c.degree() - 1];
        let mut i = 0;
        while i < inner.len() {
            let t = inner[i];
            let mut m = 1;
            while i + m < inner.len() && (inner[i + m] - t).abs() <= KNOT_COINCIDENCE {
                m += 1;
            }
            match union
                .iter_mut()
                .find(|(u, _)| (u - t).abs() <= KNOT_COINCIDENCE)
            {
                Some((_, have)) => *have = (*have).max(m),
                None => union.push((t, m)),
            }
            i += m;
        }
    }
    union.sort_by(|a, b| a.0.total_cmp(&b.0));
    union
}

/// `piece`, on `[0, 1]`, with the knots `union` inserted where it lacks
/// them, and every interior knot set to `union`'s own value, so the
/// vectors of the sections agree to the bit.
fn refined(piece: &NurbsCurve, union: &[(f64, usize)]) -> Result<NurbsCurve, GeomError> {
    let mut curve = piece.clone();
    for &(t, want) in union {
        let k = curve.knots();
        let have = k
            .iter()
            .filter(|x| (**x - t).abs() <= KNOT_COINCIDENCE)
            .count();
        if have < want {
            curve = curve.insert_knot(t, want - have)?;
        }
    }
    let d = curve.degree();
    let mut knots = curve.knots().to_vec();
    let n = knots.len();
    for k in &mut knots[d + 1..n - d - 1] {
        if let Some(&(t, _)) = union
            .iter()
            .find(|(t, _)| (*t - *k).abs() <= KNOT_COINCIDENCE)
        {
            *k = t;
        }
    }
    NurbsCurve::new(
        d,
        knots,
        curve.control_points().to_vec(),
        curve.weights().to_vec(),
    )
}

/// The `sections` of a loft made compatible: every loop split and refined
/// so that loop `k` of every section has the same segments in the same
/// order and one knot vector.
///
/// Sections are matched loop by loop (the outer loop with the outer, hole
/// `i` with hole `i`), each loop from its start point — the first edge of
/// [`Profile::edges`] — by arc length. A set of loops with the same
/// number of segments corresponds segment for segment, whatever their
/// shapes; otherwise every loop is cut at the arc-length fractions of
/// every other loop's vertices, so a circle against a square is cut where
/// the square's corners fall. Inside a pair of segments, points correspond
/// by the segments' parameters normalised to `[0, 1]`.
///
/// Errors: [`LoftError`], each naming the section.
///
/// ```
/// use arris_geom::{match_sections, Profile, ProfileLoop, ProfileSegment};
/// use arris_math::{Frame, Point2, Point3, Precision, Vec3};
///
/// let square = |z: f64, h: f64| Profile {
///     plane: Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).unwrap(),
///     outer: ProfileLoop::Path {
///         start: Point2::new(h, h),
///         segments: vec![
///             ProfileSegment::LineTo(Point2::new(-h, h)),
///             ProfileSegment::LineTo(Point2::new(-h, -h)),
///             ProfileSegment::LineTo(Point2::new(h, -h)),
///             ProfileSegment::LineTo(Point2::new(h, h)),
///         ],
///     },
///     holes: vec![],
/// };
/// let circle = |z: f64| Profile {
///     plane: Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).unwrap(),
///     outer: ProfileLoop::Circle { center: Point2::origin(), radius: 1.0 },
///     holes: vec![],
/// };
/// let tol = Precision::DEFAULT.tolerance();
/// let matched = match_sections(&[circle(0.0), square(1.0, 2.0)], false, tol).unwrap();
/// // The circle was cut where the square has corners.
/// assert_eq!(matched[0].loops[0].nurbs.segments.len(), 4);
/// assert_eq!(matched[0].loops[0].nurbs.curve.knots(), matched[1].loops[0].nurbs.curve.knots());
/// ```
pub fn match_sections(
    sections: &[Profile],
    closed: bool,
    tol: Tolerance,
) -> Result<Vec<MatchedSection>, LoftError> {
    let needed = if closed { 3 } else { 2 };
    if sections.len() < needed {
        return Err(LoftError::TooFewSections {
            given: sections.len(),
            needed,
        });
    }
    let edges: Vec<Vec<Vec<ProfileEdge>>> = sections
        .iter()
        .enumerate()
        .map(|(section, p)| {
            p.edges(tol)
                .map_err(|source| LoftError::Profile { section, source })
        })
        .collect::<Result<_, _>>()?;
    let loop_count = edges[0].len();
    for (section, e) in edges.iter().enumerate() {
        if e.len() != loop_count {
            return Err(LoftError::LoopCount {
                section,
                expected: loop_count,
                found: e.len(),
            });
        }
    }
    let mut matched: Vec<MatchedSection> = (0..sections.len())
        .map(|_| MatchedSection {
            loops: Vec::with_capacity(loop_count),
        })
        .collect();
    for loop_index in 0..loop_count {
        let loops: Vec<&[ProfileEdge]> = edges.iter().map(|e| e[loop_index].as_slice()).collect();
        let cut = matched_stretches(&loops, tol);
        // Each segment of each section as a rational quadratic on [0, 1].
        let at = |section: usize| {
            move |source: GeomError| LoftError::Loop {
                section,
                loop_index,
                source,
            }
        };
        let mut units: Vec<Vec<NurbsCurve>> = Vec::with_capacity(cut.len());
        for (section, stretches) in cut.iter().enumerate() {
            let pieces = stretches
                .iter()
                .map(|t| piece_nurbs(&t.curve, t.range, t.pcurve).and_then(|c| unit_range(&c)))
                .collect::<Result<Vec<_>, _>>()
                .map_err(at(section))?;
            units.push(pieces);
        }
        let count = units[0].len();
        let mut refined_units: Vec<Vec<NurbsCurve>> = vec![Vec::with_capacity(count); units.len()];
        for j in 0..count {
            let column: Vec<NurbsCurve> = units.iter().map(|u| u[j].clone()).collect();
            let union = knot_union(&column);
            for (section, piece) in column.iter().enumerate() {
                refined_units[section].push(refined(piece, &union).map_err(at(section))?);
            }
        }
        for (section, parts) in refined_units.iter().enumerate() {
            let nurbs = assemble(parts).map_err(at(section))?;
            let sources = cut[section].iter().map(|t| t.source).collect();
            matched[section].loops.push(MatchedLoop { nurbs, sources });
        }
    }
    for section in 1..matched.len() {
        let same = matched[section]
            .loops
            .iter()
            .zip(&matched[section - 1].loops)
            .all(|(a, b)| {
                a.nurbs
                    .curve
                    .control_points()
                    .iter()
                    .zip(b.nurbs.curve.control_points())
                    .all(|(p, q)| (p - q).norm() <= tol.linear)
            });
        if same {
            return Err(LoftError::Coincident {
                section,
                previous: section - 1,
            });
        }
    }
    Ok(matched)
}
