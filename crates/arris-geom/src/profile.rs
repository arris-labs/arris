//! Profiles: a consumer's sketch as a value — a plane and closed loops of
//! lines, arcs and elliptic arcs in the plane's own (u, v) — validated
//! once and turned into oriented edges carrying exact pcurves
//! (`docs/DATA-MODEL.md` §Profiles, ADR-0014).
//!
//! A profile is written the way it is drawn: an outer loop and holes, each
//! a chain of segments, a full circle or a full ellipse, in any
//! orientation. What comes
//! back from [`Profile::edges`] is the same loops oriented — the outer
//! counter-clockwise about the plane's normal and every hole clockwise —
//! with the consumer's own loop and segment indices still on every edge,
//! so an operation can name the part of the sketch each entity it makes
//! came from.

use arris_math::{
    Frame, Interval, Meter, Point2, Point3, Tolerance, UnitVec3, Vec2, Vec3, is_negligible,
    wrap_angle,
};

use crate::integrate::{Grid, region_integral};
use crate::project::{ellipse_distance, ellipse_nearest};
use crate::region2::{Piece, Polygon2};
use crate::{Curve, Curve2, GeomError, Surface, pcurve_on};

/// A sketch: the plane it is drawn in, an outer loop and its holes, in
/// the plane's own (u, v). Plain data with no orientation convention —
/// [`Profile::edges`] is what validates it and decides the turn of every
/// loop.
///
/// The plane's frame is the profile's coordinate system: `(u, v)` is
/// `origin + u·X + v·Y`, and the plane's `Z` is the normal every
/// orientation here is measured about.
#[derive(Debug, Clone, PartialEq)]
pub struct Profile {
    /// The plane the loops are drawn in.
    pub plane: Frame,
    /// The outer loop.
    pub outer: ProfileLoop,
    /// The holes, each inside the outer loop and outside every other.
    pub holes: Vec<ProfileLoop>,
}

/// One closed loop of a [`Profile`]: a full circle, a full ellipse, or a
/// chain of segments returning to where it started. Written in either
/// orientation.
#[derive(Debug, Clone, PartialEq)]
pub enum ProfileLoop {
    /// A full circle in the profile's plane.
    Circle {
        /// Its centre in (u, v).
        center: Point2,
        /// Its radius.
        radius: f64,
    },
    /// A full ellipse in the profile's plane (ADR-0014). A `minor_radius`
    /// longer than `major` names the same point set, and the edge's axes
    /// are swapped so `a ≥ b`; radii that agree within the linear
    /// tolerance make a circle edge of their mean radius.
    Ellipse {
        /// Its centre in (u, v).
        center: Point2,
        /// From the centre to one end of the major axis: its length is the
        /// major radius and its direction the axis.
        major: Vec2,
        /// The minor radius.
        minor_radius: f64,
    },
    /// A chain of segments: the first starts at `start` and the last ends
    /// there.
    Path {
        /// Where the chain starts, in (u, v).
        start: Point2,
        /// The segments, in the order they are walked.
        segments: Vec<ProfileSegment>,
    },
}

/// One segment of a [`ProfileLoop::Path`], from where the previous
/// segment ended.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ProfileSegment {
    /// A straight segment to a point.
    LineTo(Point2),
    /// A circular arc through `via` to `to`.
    ArcTo {
        /// Where the arc ends.
        to: Point2,
        /// A point on the arc between its ends, which decides the arc's
        /// centre, radius and which way round it goes.
        via: Point2,
    },
    /// An arc of the ellipse of `center`, `major` and `minor_radius` to
    /// `to`, turning counter-clockwise in (u, v) when `ccw` and clockwise
    /// otherwise (ADR-0014). The ellipse's centre and axes are given, so
    /// no `via` is needed: only the turn is left, and the flag states it.
    /// Both ends must lie on the ellipse within the linear tolerance; the
    /// normalisation of [`ProfileLoop::Ellipse`] applies.
    EllipseTo {
        /// Where the arc ends.
        to: Point2,
        /// The ellipse's centre in (u, v).
        center: Point2,
        /// From the centre to one end of the major axis.
        major: Vec2,
        /// The minor radius.
        minor_radius: f64,
        /// Counter-clockwise from the previous end to `to`, about the
        /// plane's normal.
        ccw: bool,
    },
}

impl ProfileSegment {
    /// Where the segment ends, in (u, v).
    pub fn end(&self) -> Point2 {
        match self {
            ProfileSegment::LineTo(p) => *p,
            ProfileSegment::ArcTo { to, .. } | ProfileSegment::EllipseTo { to, .. } => *to,
        }
    }

    /// The arc's `via` point, or `None` for a straight or an elliptic
    /// segment, which has none.
    pub fn via(&self) -> Option<Point2> {
        match self {
            ProfileSegment::LineTo(_) | ProfileSegment::EllipseTo { .. } => None,
            ProfileSegment::ArcTo { via, .. } => Some(*via),
        }
    }
}

/// An ellipse's axes normalised for an edge: `(X, Y, a, b)` with `X` the
/// unit major axis, `Y` its quarter turn in (u, v), and `a ≥ b`; a
/// `minor_radius` longer than `major` swaps the axes and turns the frame
/// a quarter turn. `None` when either radius is not finite or is within
/// `tol.linear` of zero.
fn ellipse_axes(major: Vec2, minor_radius: f64, tol: Tolerance) -> Option<(Vec2, Vec2, f64, f64)> {
    let a = major.norm();
    if !(a.is_finite() && minor_radius.is_finite()) || a <= tol.linear || minor_radius <= tol.linear
    {
        return None;
    }
    let ex = major / a;
    let ey = Vec2::new(-ex.y, ex.x);
    Some(if minor_radius > a {
        (ey, -ex, minor_radius, a)
    } else {
        (ex, ey, a, minor_radius)
    })
}

/// One oriented edge of a validated profile: the 3D curve of one segment,
/// the range walked, the exact pcurve in the profile's plane, the two
/// endpoints in (u, v), and where in the *consumer's* sketch it came from.
///
/// The curve runs along the walk: `curve.point(range.lo())` is [`start`]
/// and `curve.point(range.hi())` is [`end`], whichever way the loop had to
/// be turned — to rounding for a line or an arc, and within the linear
/// tolerance for an elliptic arc, whose ends are the ellipse's nearest
/// points to the consumer's. A circular or elliptic edge's frame carries
/// that direction in its `Z`,
/// which is the plane's normal for a counter-clockwise arc and its
/// opposite for a clockwise one, so the parameter always runs from the
/// segment's start through its `via`, or along its stated turn.
///
/// [`start`]: ProfileEdge::start
/// [`end`]: ProfileEdge::end
#[derive(Debug, Clone, PartialEq)]
pub struct ProfileEdge {
    /// The 3D curve: a [`Curve::Line`] for a straight segment, a
    /// [`Curve::Circle`] for an arc or a circle loop, a [`Curve::Ellipse`]
    /// with `X` along the major axis for an elliptic arc or an ellipse
    /// loop — or a [`Curve::Circle`] for one whose radii agree within the
    /// tolerance.
    pub curve: Curve,
    /// The parameter range walked, along the parameter.
    pub range: Interval,
    /// The exact pcurve of `curve` on the profile's plane, at the same
    /// parameter ([`pcurve_on`]).
    pub pcurve: Curve2,
    /// Where the walk starts, in (u, v).
    pub start: Point2,
    /// Where it ends, in (u, v); a circle loop's single edge starts and
    /// ends at the same point.
    pub end: Point2,
    /// Which loop: `0` is the outer, the holes from `1` in the order the
    /// profile lists them.
    pub loop_index: usize,
    /// Which segment of that loop, as the consumer wrote it. A circle
    /// loop has the one segment `0`.
    pub segment: usize,
    /// Whether the loop had to be turned round to be oriented: `true`
    /// when the consumer's order was the opposite of the one returned.
    pub reversed: bool,
}

/// Why a sketch is not a profile. Every variant names the loop — `0` the
/// outer, the holes from `1` — and where in it the fault is, so a message
/// says *which* segment, never "invalid profile"
/// (`.agents/rules/kernel.md`).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ProfileError {
    /// The last segment of a path loop does not end where the loop
    /// started.
    #[error("loop {loop_index} is not closed: its last segment ends {gap} from its start")]
    NotClosed {
        /// The loop.
        loop_index: usize,
        /// How far the end is from the start.
        gap: f64,
    },
    /// A path loop has fewer than two segments, so it bounds nothing.
    #[error("loop {loop_index} has fewer than two segments")]
    TooFewSegments {
        /// The loop.
        loop_index: usize,
    },
    /// A segment is shorter than the linear tolerance: its length for a
    /// straight segment, the distance between its ends for an arc — so an
    /// arc that returns to its own start is refused here rather than
    /// taken for a full circle, which is a [`ProfileLoop::Circle`].
    #[error("loop {loop_index}, segment {segment} is shorter than the tolerance")]
    ShortSegment {
        /// The loop.
        loop_index: usize,
        /// The segment.
        segment: usize,
    },
    /// An arc's `via` point is on its chord within the linear tolerance,
    /// so the three points name no circle.
    #[error("loop {loop_index}, segment {segment}: the arc's via point is on its chord")]
    DegenerateArc {
        /// The loop.
        loop_index: usize,
        /// The segment.
        segment: usize,
    },
    /// An ellipse's major or minor radius is not finite or is within the
    /// linear tolerance of zero, so it names no ellipse.
    #[error("loop {loop_index}, segment {segment}: the ellipse has a degenerate radius")]
    DegenerateEllipse {
        /// The loop.
        loop_index: usize,
        /// The segment; `0` for an ellipse loop.
        segment: usize,
    },
    /// An elliptic segment's start or end is farther than the linear
    /// tolerance from its ellipse.
    #[error("loop {loop_index}, segment {segment}: an end is {distance} off its ellipse")]
    OffEllipse {
        /// The loop.
        loop_index: usize,
        /// The segment.
        segment: usize,
        /// How far the end is from the ellipse.
        distance: f64,
    },
    /// A loop encloses no area: its mean width — twice its area over its
    /// perimeter, which is the width of a long thin rectangle and the
    /// radius of a disc — is at or below the linear tolerance.
    #[error("loop {loop_index} encloses no area")]
    ZeroArea {
        /// The loop.
        loop_index: usize,
    },
    /// A loop crosses or touches itself.
    #[error("loop {loop_index} crosses itself: segments {} and {}", .segments[0], .segments[1])]
    SelfIntersecting {
        /// The loop.
        loop_index: usize,
        /// The two segments that meet, ascending.
        segments: [usize; 2],
    },
    /// Two loops cross or touch.
    #[error("loops {} and {} cross", .loops[0], .loops[1])]
    Crossing {
        /// The two loops, ascending.
        loops: [usize; 2],
    },
    /// A hole lies outside the outer loop.
    #[error("loop {hole} is a hole outside the outer loop")]
    HoleOutside {
        /// The hole's loop index.
        hole: usize,
    },
    /// One hole lies inside another.
    #[error("hole {} contains hole {}", .holes[0], .holes[1])]
    NestedHoles {
        /// The containing hole's loop index, then the contained one's.
        holes: [usize; 2],
    },
    /// The geometry crate refused a curve the profile built: an
    /// inconsistent tolerance, or a curve that does not lie in the
    /// profile's plane. The second cannot happen for a curve this module
    /// built, and is carried rather than unwrapped.
    #[error("profile geometry: {0}")]
    Geometry(#[from] GeomError),
}

impl Profile {
    /// The 3D point of a (u, v) point of this profile's plane.
    ///
    /// ```
    /// use arris_geom::profile::{Profile, ProfileLoop};
    /// use arris_math::{Frame, Point2, Point3};
    ///
    /// let p = Profile {
    ///     plane: Frame::world(),
    ///     outer: ProfileLoop::Circle { center: Point2::origin(), radius: 1.0 },
    ///     holes: Vec::new(),
    /// };
    /// assert_eq!(p.to_world(Point2::new(2.0, 3.0)), Point3::new(2.0, 3.0, 0.0));
    /// ```
    pub fn to_world(&self, uv: Point2) -> Point3 {
        self.plane.to_world(Point3::new(uv.x, uv.y, 0.0))
    }

    /// The loops in index order: the outer first, then the holes.
    fn all_loops(&self) -> Vec<&ProfileLoop> {
        core::iter::once(&self.outer)
            .chain(self.holes.iter())
            .collect()
    }

    /// The profile's loops validated and oriented: one `Vec<ProfileEdge>`
    /// per loop in walking order, the outer (index `0`) counter-clockwise
    /// about the plane's normal and every hole clockwise, with each edge
    /// carrying the loop and segment index the consumer wrote it as and
    /// whether the loop was turned round to get there.
    ///
    /// Guarantees on the result: every edge's `curve` lies in the
    /// profile's plane and runs from its `start` to its `end` over its
    /// `range`; every edge's `pcurve` is that curve's image in the plane's
    /// (u, v) *at the same parameter*; consecutive edges of a loop meet at
    /// a point, and the last meets the first.
    ///
    /// What is checked, in this order — every loop before the next check,
    /// so the error reported is always the first fault in loop order:
    /// a path loop closes within `tol.linear` and has at least two
    /// segments, no segment is shorter than `tol.linear`, no arc's
    /// `via` is on its chord, every ellipse has radii above `tol.linear`
    /// and every elliptic segment's ends are within `tol.linear` of its
    /// ellipse; each loop, discretised at `region2`'s
    /// minimum segment counts, has a mean width above `tol.linear` and
    /// does not meet itself; no two loops meet; every hole is inside the
    /// outer loop and outside every other hole, by winding number. The
    /// containment tests are made on those minimal polygons, whose arcs
    /// are their chords: a hole whose *arc* pokes out of the outer loop by
    /// less than the sagitta of a minimal chord is not caught here.
    ///
    /// ```
    /// use arris_geom::profile::{Profile, ProfileLoop, ProfileSegment};
    /// use arris_math::{Frame, Point2, Precision};
    ///
    /// let p = |u, v| Point2::new(u, v);
    /// // A square written clockwise: it comes back counter-clockwise.
    /// let square = Profile {
    ///     plane: Frame::world(),
    ///     outer: ProfileLoop::Path {
    ///         start: p(0.0, 0.0),
    ///         segments: vec![
    ///             ProfileSegment::LineTo(p(0.0, 1.0)),
    ///             ProfileSegment::LineTo(p(1.0, 1.0)),
    ///             ProfileSegment::LineTo(p(1.0, 0.0)),
    ///             ProfileSegment::LineTo(p(0.0, 0.0)),
    ///         ],
    ///     },
    ///     holes: Vec::new(),
    /// };
    /// let loops = square.edges(Precision::DEFAULT.tolerance()).unwrap();
    /// assert_eq!(loops[0].len(), 4);
    /// assert!(loops[0].iter().all(|e| e.reversed));
    /// // The walk runs backwards through the consumer's segments.
    /// let order: Vec<usize> = loops[0].iter().map(|e| e.segment).collect();
    /// assert_eq!(order, vec![3, 2, 1, 0]);
    /// assert_eq!(loops[0][0].start, p(0.0, 0.0));
    /// assert_eq!(loops[0][0].end, p(1.0, 0.0));
    /// ```
    pub fn edges(&self, tol: Tolerance) -> Result<Vec<Vec<ProfileEdge>>, ProfileError> {
        if !tol.is_consistent() {
            return Err(GeomError::InvalidTolerance(tol).into());
        }
        let surface = Surface::Plane { frame: self.plane };
        let loops = self.all_loops();
        // The loops as the consumer wrote them, with their curves and
        // pcurves: the structural checks happen here.
        let mut written: Vec<Vec<ProfileEdge>> = Vec::with_capacity(loops.len());
        for (i, lp) in loops.iter().enumerate() {
            written.push(self.loop_edges(&surface, tol, i, lp, false)?);
        }
        // Each loop's minimal polygon, with the consumer's segment index
        // of every polygon segment: area, thinness and self-intersection.
        let mut polygons: Vec<Polygon2> = Vec::with_capacity(written.len());
        for (i, edges) in written.iter().enumerate() {
            let (polygon, owner) = polygon_of(edges);
            let area = polygon.signed_area();
            let perimeter: f64 = polygon.segments().map(|(a, b)| (b - a).norm()).sum();
            let mean_width = 2.0 * area.abs() / perimeter;
            if mean_width.is_nan() || mean_width <= tol.linear {
                return Err(ProfileError::ZeroArea { loop_index: i });
            }
            if let Some(&(a, b)) = polygon.self_intersections().first() {
                let (mut s, mut t) = (owner[a], owner[b]);
                if s > t {
                    core::mem::swap(&mut s, &mut t);
                }
                return Err(ProfileError::SelfIntersecting {
                    loop_index: i,
                    segments: [s, t],
                });
            }
            polygons.push(polygon);
        }
        for i in 0..polygons.len() {
            for j in i + 1..polygons.len() {
                if !polygons[i].intersections(&polygons[j]).is_empty() {
                    return Err(ProfileError::Crossing { loops: [i, j] });
                }
            }
        }
        // No two loops meet, so one point of a loop decides where the
        // whole loop lies with respect to another.
        let inside = |a: usize, b: usize| polygons[a].winding_number(polygons[b].points()[0]) != 0;
        for h in 1..polygons.len() {
            if !inside(0, h) {
                return Err(ProfileError::HoleOutside { hole: h });
            }
        }
        for i in 1..polygons.len() {
            for j in i + 1..polygons.len() {
                if inside(i, j) {
                    return Err(ProfileError::NestedHoles { holes: [i, j] });
                }
                if inside(j, i) {
                    return Err(ProfileError::NestedHoles { holes: [j, i] });
                }
            }
        }
        // Turn round every loop whose written order is the wrong way.
        let mut oriented = Vec::with_capacity(written.len());
        for (i, edges) in written.into_iter().enumerate() {
            let wants_ccw = i == 0;
            if (polygons[i].signed_area() > 0.0) == wants_ccw {
                oriented.push(edges);
            } else {
                oriented.push(self.loop_edges(&surface, tol, i, loops[i], true)?);
            }
        }
        Ok(oriented)
    }

    /// The area the profile's loops enclose in its plane and the (u, v)
    /// centroid of that region, by `integrate::region_integral` over the
    /// oriented edges, so the holes subtract themselves. The area is
    /// positive.
    ///
    /// Errors: as [`Profile::edges`], which validates the profile first.
    ///
    /// ```
    /// use arris_geom::profile::{Profile, ProfileLoop};
    /// use arris_math::{Frame, Point2, Precision};
    /// use core::f64::consts::PI;
    ///
    /// let disc = Profile {
    ///     plane: Frame::world(),
    ///     outer: ProfileLoop::Circle { center: Point2::new(3.0, 1.0), radius: 2.0 },
    ///     holes: Vec::new(),
    /// };
    /// let (area, centroid) = disc.area_and_centroid(Precision::DEFAULT.tolerance()).unwrap();
    /// assert!((area - 4.0 * PI).abs() < 1e-12);
    /// assert!((centroid - Point2::new(3.0, 1.0)).norm() < 1e-12);
    /// ```
    pub fn area_and_centroid(&self, tol: Tolerance) -> Result<(f64, Point2), ProfileError> {
        Ok(area_and_centroid_of(&self.edges(tol)?))
    }

    /// One loop's edges, built in the consumer's order or against it.
    fn loop_edges(
        &self,
        surface: &Surface,
        tol: Tolerance,
        loop_index: usize,
        lp: &ProfileLoop,
        reversed: bool,
    ) -> Result<Vec<ProfileEdge>, ProfileError> {
        match lp {
            &ProfileLoop::Circle { center, radius } => {
                let finite = radius.is_finite() && center.coords.iter().all(|c| c.is_finite());
                if !finite || radius <= 0.0 {
                    return Err(ProfileError::ZeroArea { loop_index });
                }
                let normal = self.plane.z().into_inner();
                let z = if reversed { -normal } else { normal };
                let frame = Frame::new(self.to_world(center), z, self.plane.x().into_inner())
                    .map_err(|_| ProfileError::ZeroArea { loop_index })?;
                let curve = Curve::Circle { frame, radius };
                let range = Interval::TURN;
                let pcurve = pcurve_on(&curve, range, surface, tol, &mut Meter::default())?;
                // Where the oracle's `gp_Circ` on the plane's `Ax2` puts
                // the seam: the circle's own `t = 0`, along the plane's X.
                let vertex = center + Vec2::new(radius, 0.0);
                Ok(vec![ProfileEdge {
                    curve,
                    range,
                    pcurve,
                    start: vertex,
                    end: vertex,
                    loop_index,
                    segment: 0,
                    reversed,
                }])
            }
            &ProfileLoop::Ellipse {
                center,
                major,
                minor_radius,
            } => {
                let degenerate = || ProfileError::DegenerateEllipse {
                    loop_index,
                    segment: 0,
                };
                if !center.coords.iter().all(|c| c.is_finite()) {
                    return Err(degenerate());
                }
                let (ex, _, a, b) =
                    ellipse_axes(major, minor_radius, tol).ok_or_else(degenerate)?;
                let normal = self.plane.z().into_inner();
                let z = if reversed { -normal } else { normal };
                let x = self.plane.vec_to_world(Vec3::new(ex.x, ex.y, 0.0));
                let frame = Frame::new(self.to_world(center), z, x).map_err(|_| degenerate())?;
                // Where the oracle's `gp_Elips` puts the seam: `t = 0`,
                // at the end of the (normalised) major axis.
                let (curve, reach) = if a - b <= tol.linear {
                    let radius = 0.5 * (a + b);
                    (Curve::Circle { frame, radius }, radius)
                } else {
                    (
                        Curve::Ellipse {
                            frame,
                            major_radius: a,
                            minor_radius: b,
                        },
                        a,
                    )
                };
                let range = Interval::TURN;
                let pcurve = pcurve_on(&curve, range, surface, tol, &mut Meter::default())?;
                let vertex = center + reach * ex;
                Ok(vec![ProfileEdge {
                    curve,
                    range,
                    pcurve,
                    start: vertex,
                    end: vertex,
                    loop_index,
                    segment: 0,
                    reversed,
                }])
            }
            ProfileLoop::Path { start, segments } => {
                let n = segments.len();
                if n < 2 {
                    return Err(ProfileError::TooFewSegments { loop_index });
                }
                let gap = (segments[n - 1].end() - start).norm();
                if gap.is_nan() || gap > tol.linear {
                    return Err(ProfileError::NotClosed { loop_index, gap });
                }
                // The walk's points; the last segment's end *is* the start.
                let mut points = Vec::with_capacity(n);
                points.push(*start);
                points.extend(segments[..n - 1].iter().map(|s| s.end()));
                let order: Vec<usize> = if reversed {
                    (0..n).rev().collect()
                } else {
                    (0..n).collect()
                };
                let mut edges = Vec::with_capacity(n);
                for k in order {
                    let (from, to) = (points[k], points[(k + 1) % n]);
                    let (from, to) = if reversed { (to, from) } else { (from, to) };
                    edges.push(self.build_edge(
                        surface,
                        tol,
                        loop_index,
                        k,
                        reversed,
                        from,
                        to,
                        &segments[k],
                    )?);
                }
                Ok(edges)
            }
        }
    }

    /// The circular arc from `from` to `to` about `centre`, of `radius`,
    /// turning counter-clockwise in (u, v) or not: the frame's `X` is the
    /// start's direction from the centre and its `Z` the plane's normal
    /// or its opposite, so the parameter runs from `0` through a positive
    /// sweep. `None` when the sweep is not positive.
    fn arc_about(
        &self,
        centre: Point2,
        radius: f64,
        from: Point2,
        to: Point2,
        counter_clockwise: bool,
    ) -> Option<(Curve, Interval)> {
        let (vs, ve) = (from - centre, to - centre);
        let sense = if counter_clockwise { 1.0 } else { -1.0 };
        let sweep = wrap_angle(sense * (vs.x * ve.y - vs.y * ve.x).atan2(vs.dot(&ve)));
        if sweep.is_nan() || sweep <= 0.0 {
            return None;
        }
        let normal = self.plane.z().into_inner();
        let z = if counter_clockwise { normal } else { -normal };
        let x = self.plane.vec_to_world(Vec3::new(vs.x, vs.y, 0.0));
        let frame = Frame::new(self.to_world(centre), z, x).ok()?;
        Some((
            Curve::Circle { frame, radius },
            Interval::new(0.0, sweep).ok()?,
        ))
    }

    /// One segment's edge, walked from `from` to `to`; `reversed` says
    /// the walk runs the consumer's segment backwards.
    #[expect(
        clippy::too_many_arguments,
        reason = "the segment's place in the sketch is four of them"
    )]
    fn build_edge(
        &self,
        surface: &Surface,
        tol: Tolerance,
        loop_index: usize,
        segment: usize,
        reversed: bool,
        from: Point2,
        to: Point2,
        shape: &ProfileSegment,
    ) -> Result<ProfileEdge, ProfileError> {
        let short = || ProfileError::ShortSegment {
            loop_index,
            segment,
        };
        let chord = to - from;
        let length = chord.norm();
        if length.is_nan() || length <= tol.linear {
            return Err(short());
        }
        let (curve, range) = match *shape {
            ProfileSegment::LineTo(_) => {
                let d = self.plane.vec_to_world(Vec3::new(chord.x, chord.y, 0.0));
                let direction = UnitVec3::try_new(d, 0.0).ok_or_else(short)?;
                let curve = Curve::Line {
                    origin: self.to_world(from),
                    direction,
                };
                (curve, Interval::new(0.0, length).map_err(|_| short())?)
            }
            ProfileSegment::ArcTo { via, .. } => {
                let degenerate = || ProfileError::DegenerateArc {
                    loop_index,
                    segment,
                };
                let b = via - from;
                let turn = b.x * chord.y - b.y * chord.x;
                // The `via` point's distance from the chord.
                let off = turn.abs() / length;
                if off.is_nan() || off <= tol.linear {
                    return Err(degenerate());
                }
                // The circumcentre of the three points, from `from`.
                let (bb, cc) = (b.norm_squared(), chord.norm_squared());
                let d = 2.0 * turn;
                let centre =
                    from + Vec2::new((bb * chord.y - cc * b.y) / d, (cc * b.x - bb * chord.x) / d);
                let radius = (from - centre).norm();
                if !radius.is_finite() || radius <= 0.0 {
                    return Err(degenerate());
                }
                // `turn > 0` is `from → via → to` counter-clockwise in
                // (u, v), so the arc runs about the plane's normal.
                self.arc_about(centre, radius, from, to, turn > 0.0)
                    .ok_or_else(degenerate)?
            }
            ProfileSegment::EllipseTo {
                center,
                major,
                minor_radius,
                ccw,
                ..
            } => {
                let degenerate = || ProfileError::DegenerateEllipse {
                    loop_index,
                    segment,
                };
                if !center.coords.iter().all(|c| c.is_finite()) {
                    return Err(degenerate());
                }
                let (ex, ey, a, b) =
                    ellipse_axes(major, minor_radius, tol).ok_or_else(degenerate)?;
                // Walked backwards, the consumer's turn is the other way.
                let ccw = ccw != reversed;
                // Each end's parameter on the ellipse and distance from
                // it, in the ellipse's own axes.
                let mut ends = [0.0; 2];
                for (p, t) in [from, to].into_iter().zip(&mut ends) {
                    let d = p - center;
                    let (lx, ly) = (d.dot(&ex), d.dot(&ey));
                    let noise = p.coords.norm() + center.coords.norm();
                    // An end with no unique nearest point is deep inside
                    // the ellipse and refused by the distance's bound.
                    let (nearest, distance) = match ellipse_nearest(a, b, lx, ly, noise) {
                        Ok(found) => found,
                        Err(_) => (
                            (ly / b).atan2(lx / a),
                            ellipse_distance(a, b, lx, ly, noise),
                        ),
                    };
                    if distance.is_nan() || distance > tol.linear {
                        return Err(ProfileError::OffEllipse {
                            loop_index,
                            segment,
                            distance,
                        });
                    }
                    *t = nearest;
                }
                if a - b <= tol.linear {
                    // A near-circular section is a circle, through the
                    // same ends (ADR-0014).
                    self.arc_about(center, 0.5 * (a + b), from, to, ccw)
                        .ok_or_else(short)?
                } else {
                    // The frame's `Z` is the normal for a counter-clockwise
                    // turn; against it, `Y` is `−ey` and the parameter
                    // runs the other way.
                    let normal = self.plane.z().into_inner();
                    let (z, sense) = if ccw { (normal, 1.0) } else { (-normal, -1.0) };
                    let lo = wrap_angle(sense * ends[0]);
                    let sweep = wrap_angle(sense * (ends[1] - ends[0]));
                    if sweep.is_nan() || sweep <= 0.0 {
                        return Err(short());
                    }
                    let x = self.plane.vec_to_world(Vec3::new(ex.x, ex.y, 0.0));
                    let frame =
                        Frame::new(self.to_world(center), z, x).map_err(|_| degenerate())?;
                    (
                        Curve::Ellipse {
                            frame,
                            major_radius: a,
                            minor_radius: b,
                        },
                        Interval::new(lo, lo + sweep).map_err(|_| short())?,
                    )
                }
            }
        };
        let pcurve = pcurve_on(&curve, range, surface, tol, &mut Meter::default())?;
        Ok(ProfileEdge {
            curve,
            range,
            pcurve,
            start: from,
            end: to,
            loop_index,
            segment,
            reversed,
        })
    }
}

/// The minimal polygon of one loop's edges and, per polygon segment, the
/// consumer's index of the profile segment it came from.
fn polygon_of(edges: &[ProfileEdge]) -> (Polygon2, Vec<usize>) {
    let mut points: Vec<Point2> = Vec::new();
    let mut owner: Vec<usize> = Vec::new();
    for e in edges {
        let piece = Piece::along(&e.pcurve, e.range);
        // `f64::INFINITY` asks for `region2`'s minimum counts.
        let samples = piece.sample(piece.segment_count(f64::INFINITY));
        // The last sample is the next piece's first point.
        for p in samples.iter().take(samples.len().saturating_sub(1)) {
            if points.last() != Some(p) {
                points.push(*p);
                owner.push(e.segment);
            }
        }
    }
    if points.len() > 1 && points.first() == points.last() {
        points.pop();
        owner.pop();
    }
    (Polygon2::from_points(points), owner)
}

/// The area and (u, v) centroid of the region the oriented loops bound.
/// A plane's (u, v) is affine, so the integrands here are polynomials the
/// quadrature is exact for and the inner integral is taken in one
/// interval — `integrate::surface_grid` of a plane.
fn area_and_centroid_of(loops: &[Vec<ProfileEdge>]) -> (f64, Point2) {
    let pieces: Vec<Piece<'_>> = loops
        .iter()
        .flatten()
        .map(|e| Piece::along(&e.pcurve, e.range))
        .collect();
    let step = &Grid::NONE;
    let area = region_integral(&pieces, step, |_, _| 1.0);
    if is_negligible(area, 1.0) {
        return (area, Point2::origin());
    }
    let u = region_integral(&pieces, step, |u, _| u);
    let v = region_integral(&pieces, step, |_, v| v);
    (area, Point2::new(u / area, v / area))
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Precision;
    use core::f64::consts::{FRAC_PI_2, PI};

    fn tol() -> Tolerance {
        Precision::DEFAULT.tolerance()
    }

    fn p(u: f64, v: f64) -> Point2 {
        Point2::new(u, v)
    }

    /// A rectangle `[0, w] × [0, h]` written counter-clockwise.
    fn rectangle(w: f64, h: f64) -> ProfileLoop {
        ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(w, 0.0)),
                ProfileSegment::LineTo(p(w, h)),
                ProfileSegment::LineTo(p(0.0, h)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        }
    }

    fn profile(outer: ProfileLoop, holes: Vec<ProfileLoop>) -> Profile {
        Profile {
            plane: Frame::world(),
            outer,
            holes,
        }
    }

    #[test]
    fn an_arc_runs_from_its_start_through_its_via() {
        // A semicircle over the chord (0,0) → (2,0) through (1,1): its
        // turn is clockwise, so its frame's Z opposes the plane's normal.
        let sketch = profile(
            ProfileLoop::Path {
                start: p(0.0, 0.0),
                segments: vec![
                    ProfileSegment::ArcTo {
                        to: p(2.0, 0.0),
                        via: p(1.0, 1.0),
                    },
                    ProfileSegment::LineTo(p(0.0, 0.0)),
                ],
            },
            Vec::new(),
        );
        let loops = sketch.edges(tol()).unwrap();
        // Written clockwise (the bulge is above the chord back), so it
        // comes back turned round: the line first, then the arc backwards.
        let arc = loops[0].iter().find(|e| e.segment == 0).unwrap();
        let Curve::Circle { frame, radius } = &arc.curve else {
            panic!("{:?}", arc.curve)
        };
        assert!((radius - 1.0).abs() < 1e-15);
        assert!((frame.origin() - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-15);
        assert!((arc.range.hi() - PI).abs() < 1e-15);
        // Whichever way it was walked, the mid-parameter point is the via.
        let mid = arc.curve.point(arc.range.midpoint());
        assert!((mid - Point3::new(1.0, 1.0, 0.0)).norm() < 1e-14, "{mid}");
        // The pcurve's image is the curve at the same parameter.
        for i in 0..=16 {
            let t = arc.range.lerp(i as f64 / 16.0);
            let uv = arc.pcurve.point(t);
            assert!((Point3::new(uv.x, uv.y, 0.0) - arc.curve.point(t)).norm() < 1e-14);
        }
    }

    #[test]
    fn a_circle_loop_puts_its_vertex_on_the_planes_x() {
        let sketch = profile(
            ProfileLoop::Circle {
                center: p(1.0, 2.0),
                radius: 3.0,
            },
            Vec::new(),
        );
        let loops = sketch.edges(tol()).unwrap();
        assert_eq!(loops[0].len(), 1);
        let e = &loops[0][0];
        assert_eq!(e.start, p(4.0, 2.0));
        assert_eq!(e.end, e.start);
        assert!(!e.reversed);
        assert_eq!(e.range, Interval::TURN);
        // As a hole it is the same circle turned round.
        let holed = profile(
            rectangle(10.0, 10.0),
            vec![ProfileLoop::Circle {
                center: p(5.0, 5.0),
                radius: 1.0,
            }],
        );
        let loops = holed.edges(tol()).unwrap();
        assert!(loops[1][0].reversed);
        let Curve::Circle { frame, .. } = &loops[1][0].curve else {
            panic!()
        };
        assert!(frame.z().z < 0.0);
    }

    #[test]
    fn a_quarter_arc_sweeps_a_quarter_turn() {
        let sketch = profile(
            ProfileLoop::Path {
                start: p(1.0, 0.0),
                segments: vec![
                    ProfileSegment::ArcTo {
                        to: p(0.0, 1.0),
                        via: p(0.5f64.sqrt(), 0.5f64.sqrt()),
                    },
                    ProfileSegment::LineTo(p(0.0, 0.0)),
                    ProfileSegment::LineTo(p(1.0, 0.0)),
                ],
            },
            Vec::new(),
        );
        let loops = sketch.edges(tol()).unwrap();
        let arc = loops[0].iter().find(|e| e.segment == 0).unwrap();
        assert!(!arc.reversed);
        assert!((arc.range.hi() - FRAC_PI_2).abs() < 1e-14, "{arc:?}");
        let (area, _) = sketch.area_and_centroid(tol()).unwrap();
        assert!((area - PI / 4.0).abs() < 1e-12, "{area}");
    }

    #[test]
    fn the_area_of_a_rectangle_with_a_hole_subtracts_the_hole() {
        let sketch = profile(
            rectangle(4.0, 2.0),
            vec![ProfileLoop::Circle {
                center: p(2.0, 1.0),
                radius: 0.5,
            }],
        );
        let (area, centroid) = sketch.area_and_centroid(tol()).unwrap();
        assert!((area - (8.0 - PI * 0.25)).abs() < 1e-12, "{area}");
        assert!((centroid - p(2.0, 1.0)).norm() < 1e-12, "{centroid}");
    }

    #[test]
    fn a_bad_tolerance_is_refused_before_any_geometry() {
        let sketch = profile(rectangle(1.0, 1.0), Vec::new());
        assert!(matches!(
            sketch.edges(Tolerance::new(0.0, 1e-9)),
            Err(ProfileError::Geometry(GeomError::InvalidTolerance(_)))
        ));
    }

    #[test]
    fn a_loop_of_zero_width_is_refused() {
        // A loop that goes out and comes straight back: two segments of
        // length ten enclosing nothing.
        let retraced = ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(10.0, 0.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        };
        let sketch = profile(retraced, Vec::new());
        assert_eq!(
            sketch.edges(tol()),
            Err(ProfileError::ZeroArea { loop_index: 0 })
        );
    }

    #[test]
    fn the_holes_of_a_profile_come_back_clockwise() {
        let sketch = profile(
            rectangle(10.0, 10.0),
            vec![ProfileLoop::Path {
                start: p(2.0, 2.0),
                segments: vec![
                    ProfileSegment::LineTo(p(4.0, 2.0)),
                    ProfileSegment::LineTo(p(4.0, 4.0)),
                    ProfileSegment::LineTo(p(2.0, 4.0)),
                    ProfileSegment::LineTo(p(2.0, 2.0)),
                ],
            }],
        );
        let loops = sketch.edges(tol()).unwrap();
        let (polygon, _) = polygon_of(&loops[0]);
        assert!(polygon.signed_area() > 0.0);
        let (hole, _) = polygon_of(&loops[1]);
        assert!(hole.signed_area() < 0.0);
        assert!(loops[1].iter().all(|e| e.reversed));
        let (area, _) = sketch.area_and_centroid(tol()).unwrap();
        assert!((area - 96.0).abs() < 1e-12, "{area}");
        // The profile's plane need not be the world's.
        let tilted = Profile {
            plane: Frame::new(
                Point3::new(1.0, 2.0, 3.0),
                Vec3::new(1.0, 1.0, 1.0),
                Vec3::new(1.0, -1.0, 0.0),
            )
            .unwrap(),
            ..sketch
        };
        let (area, _) = tilted.area_and_centroid(tol()).unwrap();
        assert!((area - 96.0).abs() < 1e-12, "{area}");
        for loop_edges in tilted.edges(tol()).unwrap() {
            for e in &loop_edges {
                let uv = e.pcurve.point(e.range.lo());
                assert!((tilted.to_world(uv) - e.curve.point(e.range.lo())).norm() < 1e-12);
            }
        }
    }

    #[test]
    fn every_fault_names_its_loop_and_segment() {
        let square = rectangle(10.0, 10.0);
        // A gap.
        let open = ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(10.0, 0.0)),
                ProfileSegment::LineTo(p(10.0, 10.0)),
                ProfileSegment::LineTo(p(0.0, 1.0)),
            ],
        };
        assert!(matches!(
            profile(open, Vec::new()).edges(tol()),
            Err(ProfileError::NotClosed { loop_index: 0, gap }) if (gap - 1.0).abs() < 1e-15
        ));
        // One segment.
        let one = ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![ProfileSegment::LineTo(p(0.0, 0.0))],
        };
        assert_eq!(
            profile(one, Vec::new()).edges(tol()),
            Err(ProfileError::TooFewSegments { loop_index: 0 })
        );
        // A segment below the tolerance.
        let stub = ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(1e-9, 0.0)),
                ProfileSegment::LineTo(p(10.0, 10.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        };
        assert_eq!(
            profile(stub, Vec::new()).edges(tol()),
            Err(ProfileError::ShortSegment {
                loop_index: 0,
                segment: 0
            })
        );
        // A collinear via.
        let flat_arc = ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(10.0, 0.0)),
                ProfileSegment::ArcTo {
                    to: p(10.0, 10.0),
                    via: p(10.0, 5.0),
                },
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        };
        assert_eq!(
            profile(flat_arc, Vec::new()).edges(tol()),
            Err(ProfileError::DegenerateArc {
                loop_index: 0,
                segment: 1
            })
        );
        // A bow-tie.
        let bowtie = ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(10.0, 0.0)),
                ProfileSegment::LineTo(p(2.0, 8.0)),
                ProfileSegment::LineTo(p(8.0, 10.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        };
        assert_eq!(
            profile(bowtie, Vec::new()).edges(tol()),
            Err(ProfileError::SelfIntersecting {
                loop_index: 0,
                segments: [1, 3]
            })
        );
        // Two crossing loops.
        let overlapping = profile(
            square.clone(),
            vec![ProfileLoop::Circle {
                center: p(10.0, 5.0),
                radius: 2.0,
            }],
        );
        assert_eq!(
            overlapping.edges(tol()),
            Err(ProfileError::Crossing { loops: [0, 1] })
        );
        // A hole outside the outer loop.
        let outside = profile(
            square.clone(),
            vec![ProfileLoop::Circle {
                center: p(20.0, 5.0),
                radius: 2.0,
            }],
        );
        assert_eq!(
            outside.edges(tol()),
            Err(ProfileError::HoleOutside { hole: 1 })
        );
        // A hole inside a hole.
        let nested = profile(
            square,
            vec![
                ProfileLoop::Circle {
                    center: p(5.0, 5.0),
                    radius: 3.0,
                },
                ProfileLoop::Circle {
                    center: p(5.0, 5.0),
                    radius: 1.0,
                },
            ],
        );
        assert_eq!(
            nested.edges(tol()),
            Err(ProfileError::NestedHoles { holes: [1, 2] })
        );
    }

    #[test]
    fn a_stadium_has_the_area_of_its_rectangle_and_its_disc() {
        let (l, r) = (20.0, 5.0);
        let stadium = ProfileLoop::Path {
            start: p(0.0, -r),
            segments: vec![
                ProfileSegment::LineTo(p(l, -r)),
                ProfileSegment::ArcTo {
                    to: p(l, r),
                    via: p(l + r, 0.0),
                },
                ProfileSegment::LineTo(p(0.0, r)),
                ProfileSegment::ArcTo {
                    to: p(0.0, -r),
                    via: p(-r, 0.0),
                },
            ],
        };
        let sketch = profile(stadium, Vec::new());
        let (area, centroid) = sketch.area_and_centroid(tol()).unwrap();
        let expected = 2.0 * r * l + PI * r * r;
        assert!((area - expected).abs() < 1e-12 * expected, "{area}");
        assert!((centroid - p(l / 2.0, 0.0)).norm() < 1e-12, "{centroid}");
    }
}
