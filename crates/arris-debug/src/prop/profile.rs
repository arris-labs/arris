//! Strategies for random sketches: a star polygon with arcs and holes in
//! a random plane pose, in either orientation; a rectilinear staircase
//! beside an axis or reaching it, with the sweep parameters that go with
//! it; and a general profile beside an axis or with a side along it, whose
//! segments sweep every surface kind.
//!
//! Every profile a strategy here produces is a *valid* one —
//! `Profile::edges` accepts it — so a property over sweeps never has to
//! discard a case. That is bought with the shape of the generator rather
//! than with a filter: the outer loop's vertices sit at strictly
//! increasing angles about a centre, which makes the polygon star-shaped
//! and so simple, and an arc replaces a chord by a shallow bulge *outward*
//! whose sagitta stays a small fraction of the chord, which keeps it
//! inside the angular sector the chord spans and keeps the region
//! star-shaped. The holes are placed inside the largest disc about the
//! centre that the chords leave free. [`general`] needs deeper arcs, so
//! its polygon is convex (see there).

use arris_math::real;
use core::f64::consts::TAU;
use core::ops::RangeInclusive;

use arris_geom::profile::{Profile, ProfileLoop, ProfileSegment};
use arris_math::{Axis, Frame, Point2, Point3, UnitVec3, Vec2, Vec3};
use proptest::prelude::*;

use super::{finite_f64, frame};

/// The radius the outer loop's vertices are drawn within. Far below
/// [`super::DEFAULT_SCALE`], so a profile in a random pose stays in the box the
/// other strategies place their geometry in.
pub const PROFILE_RADIUS: f64 = 10.0;

/// Vertices of the outer loop: enough for a star, few enough that its
/// notches stay shallow (see [`JITTER`]).
pub const VERTICES: core::ops::RangeInclusive<usize> = 5..=8;

/// How far a vertex's angle may stray from its even share of the turn, as
/// a fraction of that share: at 0.2 two neighbours are never closer than
/// 0.6 of the even spacing, which bounds how sharp a notch between two
/// far vertices can get — and with it how much two outward arcs meeting
/// there may turn towards each other.
pub const JITTER: f64 = 0.2;

/// A vertex's radius, as a fraction of [`PROFILE_RADIUS`]: the notches
/// are deep enough to make the polygon non-convex and shallow enough to
/// keep every interior angle well away from a full turn.
pub const RADIUS_FRACTION: core::ops::RangeInclusive<f64> = 0.6..=1.0;

/// An arc's sagitta as a fraction of its chord. Bounded above so the arc
/// stays inside the sector its chord spans; bounded below so the arc is
/// never within rounding of its chord.
pub const SAGITTA_FRACTION: core::ops::RangeInclusive<f64> = 0.03..=0.12;

/// A hole's radius and the distance of its centre from the profile's
/// centre, as fractions of the largest disc about the centre the outer
/// loop's chords leave free: together below one, so a hole is inside the
/// outer loop, and far enough apart that two holes never meet.
const HOLE_RADIUS_FRACTION: f64 = 0.12;
const HOLE_OFFSET_FRACTION: f64 = 0.45;

/// The raw draw of [`star`], before it is turned into a [`Profile`].
type Draw = (
    arris_math::Frame,
    Vec<(f64, f64)>,
    Vec<Option<f64>>,
    usize,
    Vec<bool>,
);

/// A star polygon in a random plane pose: `VERTICES` vertices at strictly
/// increasing angles, some chords replaced by outward arcs, with zero,
/// one or two holes — a circle and a triangle, so both loop kinds and
/// both `ProfileLoop` variants appear — and every loop written in a random
/// orientation, since a profile carries none.
pub fn star() -> impl Strategy<Value = Profile> {
    let vertices = VERTICES;
    (
        frame(),
        proptest::collection::vec(
            (finite_f64(-JITTER..=JITTER), finite_f64(RADIUS_FRACTION)),
            vertices.clone(),
        ),
        proptest::collection::vec(proptest::option::of(finite_f64(SAGITTA_FRACTION)), vertices),
        0usize..=2,
        proptest::collection::vec(any::<bool>(), 2),
    )
        .prop_map(build)
}

fn build((plane, vertices, arcs, holes, flipped): Draw) -> Profile {
    let n = vertices.len().min(arcs.len());
    let centre = Vec2::new(0.0, 0.0);
    let share = core::f64::consts::TAU / n as f64;
    let points: Vec<Point2> = (0..n)
        .map(|k| {
            let (jitter, fraction) = vertices[k];
            let angle = (k as f64 + jitter) * share;
            let r = fraction * PROFILE_RADIUS;
            Point2::new(
                centre.x + r * real::cos(angle),
                centre.y + r * real::sin(angle),
            )
        })
        .collect();
    // The sagitta of each chord, positive outward from the centre.
    let vias: Vec<Option<Point2>> = (0..n)
        .map(|k| {
            let (a, b) = (points[k], points[(k + 1) % n]);
            let fraction = arcs[k]?;
            let chord = b - a;
            let mid = a + chord / 2.0;
            let outward = mid - Point2::from(centre);
            let outward = outward / outward.norm();
            Some(mid + fraction * chord.norm() * outward)
        })
        .collect();
    // The largest disc about the centre the chords leave free.
    let free = (0..n)
        .map(|k| distance_to_segment(Point2::from(centre), points[k], points[(k + 1) % n]))
        .fold(f64::INFINITY, f64::min);
    Profile {
        plane,
        outer: path_loop(&points, &vias, flipped[0]),
        holes: holes_within(free, holes, flipped[1]),
    }
}

/// Zero, one or two holes inside the disc of radius `free` about the
/// origin: a circle to the right of the centre, then a triangle to the
/// left, written forwards or backwards — so both `ProfileLoop` variants
/// appear, and never within reach of each other.
fn holes_within(free: f64, count: usize, backwards: bool) -> Vec<ProfileLoop> {
    let mut loops = Vec::new();
    if count >= 1 {
        loops.push(ProfileLoop::Circle {
            center: Point2::new(HOLE_OFFSET_FRACTION * free, 0.0),
            radius: HOLE_RADIUS_FRACTION * free,
        });
    }
    if count >= 2 {
        let c = Vec2::new(-HOLE_OFFSET_FRACTION * free, 0.0);
        let r = HOLE_RADIUS_FRACTION * free;
        let corner = |i: usize| {
            let a = TAU * i as f64 / 3.0;
            Point2::from(c + r * Vec2::new(real::cos(a), real::sin(a)))
        };
        let triangle: Vec<Point2> = (0..3).map(corner).collect();
        loops.push(path_loop(&triangle, &[None, None, None], backwards));
    }
    loops
}

/// The loop through `points`, segment `k` running from `points[k]` to the
/// next and bulging through `vias[k]` when there is one, written forwards
/// or backwards.
fn path_loop(points: &[Point2], vias: &[Option<Point2>], backwards: bool) -> ProfileLoop {
    let n = points.len();
    let segment = |to: usize, via: Option<Point2>| match via {
        None => ProfileSegment::LineTo(points[to]),
        Some(via) => ProfileSegment::ArcTo {
            to: points[to],
            via,
        },
    };
    if backwards {
        let start = points[0];
        let segments = (0..n)
            .map(|i| {
                // Backwards, the i-th step goes from point n−i to n−i−1,
                // which is the consumer's segment n−i−1 walked the other
                // way; its via is that segment's.
                let to = (n - i + n - 1) % n;
                segment(to, vias[to])
            })
            .collect();
        ProfileLoop::Path { start, segments }
    } else {
        let segments = (0..n).map(|k| segment((k + 1) % n, vias[k])).collect();
        ProfileLoop::Path {
            start: points[0],
            segments,
        }
    }
}

/// The distance from `p` to the segment `a`–`b`.
fn distance_to_segment(p: Point2, a: Point2, b: Point2) -> f64 {
    let d = b - a;
    let len2 = d.norm_squared();
    if len2 <= 0.0 {
        return (p - a).norm();
    }
    let t = ((p - a).dot(&d) / len2).clamp(0.0, 1.0);
    (p - (a + t * d)).norm()
}

/// The radii of an [`ellipse`] profile: far from zero and from each
/// other, so the section is never a circle to the tolerance, and below
/// [`PROFILE_RADIUS`] together.
pub const ELLIPSE_MINOR: RangeInclusive<f64> = 0.5..=4.0;
/// How much longer than its minor radius an [`ellipse`]'s major one is.
pub const ELLIPSE_EXTRA: RangeInclusive<f64> = 0.2..=5.0;

/// A full ellipse in a random plane pose with an extrude length: its
/// centre within [`PROFILE_RADIUS`] of the plane's origin, its major
/// axis at a random angle with the radii of [`ELLIPSE_MINOR`] and
/// [`ELLIPSE_EXTRA`] — written the consumer's way, so half the time the
/// `minor_radius` is the longer one and the edge's axes are swapped —
/// and a length in `[1, 10]`. What the extrude property of ADR-0014's
/// surface is held on: volume `π a b h`.
pub fn ellipse() -> impl Strategy<Value = (Profile, f64)> {
    (
        frame(),
        (finite_f64(-5.0..=5.0), finite_f64(-5.0..=5.0)),
        finite_f64(0.0..=TAU),
        finite_f64(ELLIPSE_MINOR),
        finite_f64(ELLIPSE_EXTRA),
        any::<bool>(),
        finite_f64(1.0..=10.0),
    )
        .prop_map(|(plane, (cu, cv), angle, minor, extra, swapped, length)| {
            let (along, across) = if swapped {
                (minor, minor + extra)
            } else {
                (minor + extra, minor)
            };
            let profile = Profile {
                plane,
                outer: ProfileLoop::Ellipse {
                    center: Point2::new(cu, cv),
                    major: along * Vec2::new(real::cos(angle), real::sin(angle)),
                    minor_radius: across,
                },
                holes: Vec::new(),
            };
            (profile, length)
        })
}

/// A profile with the parameters of the sweeps it is drawn for: an axis
/// in its plane, clear of every loop or touching the outer one, a revolve
/// angle in `(0, 2π]` and an extrude length. What [`rectilinear`] and
/// [`general`] yield and what a sweep property test builds a body from.
#[derive(Debug, Clone, PartialEq)]
pub struct Sweep {
    /// The sketch.
    pub profile: Profile,
    /// The revolve axis: in the profile's plane, clear of the profile or
    /// touching its outer loop, never crossing it.
    pub axis: Axis,
    /// The revolve angle in radians, `2π` for a full turn.
    pub angle: f64,
    /// The extrude length.
    pub length: f64,
}

/// The number of bars of a [`rectilinear`] staircase.
pub const STAIRCASE_BARS: RangeInclusive<usize> = 2..=5;

/// The distance of a [`rectilinear`] staircase's inner sides from the
/// axis, where they are not on it: at least one, so the smallest radius a
/// cylinder face has is one.
pub const STAIRCASE_CLEARANCE: RangeInclusive<f64> = 1.0..=3.0;

/// How far the bars of a staircase reach beyond its inner side, at most.
const STAIRCASE_REACH: f64 = 6.0;

/// The height of one bar along the axis.
const BAR_HEIGHT: RangeInclusive<f64> = 0.5..=2.0;

/// Where a bar's outer side falls within its share of the reach: away
/// from both ends of the share, so two bars are never within rounding of
/// one radius and no bar is within rounding of the inner side.
const BAR_JITTER: RangeInclusive<f64> = 0.1..=0.9;

/// A revolve angle: a full turn one time in four, otherwise a partial one
/// clear of both ends of the range so the flat ends never coincide and
/// the turn is never within rounding of none.
fn angle() -> impl Strategy<Value = f64> {
    prop_oneof![
        1 => Just(TAU),
        3 => finite_f64(0.1..=TAU - 0.1),
    ]
}

/// The raw draw of [`rectilinear`].
type StaircaseDraw = (
    (Frame, f64, f64, f64, bool),
    (f64, usize, Vec<f64>, Vec<f64>, Vec<f64>, Vec<bool>, bool),
    (bool, bool, f64, f64),
);

/// A staircase polygon beside an axis, both in a random plane pose: a
/// histogram of `STAIRCASE_BARS` bars stacked along the axis, every
/// segment parallel or perpendicular to it, each bar's inner side at
/// `STAIRCASE_CLEARANCE` from the axis — or, half the time, some of them
/// (at least one) on it, so a side lies along the axis and a bar clear of
/// it between two on it is a notch cut in from the axis — and the bars'
/// outer sides at distinct radii in a random order, with — half the time
/// — a rectangular hole inside the first bar; the axis in a random in-plane
/// direction through a random point, the profile on either side of it,
/// every loop written in either orientation. A revolve of it makes
/// planes and cylinders only, every pair of which the checker's S5 row
/// decides; the angle is a full turn one time in four and otherwise a
/// partial one clear of both ends of `(0, 2π)`, the extrude length random.
pub fn rectilinear() -> impl Strategy<Value = Sweep> {
    let bars = *STAIRCASE_BARS.end();
    (
        (
            frame(),
            finite_f64(0.0..=TAU),
            finite_f64(-5.0..=5.0),
            finite_f64(-5.0..=5.0),
            any::<bool>(),
        ),
        (
            finite_f64(STAIRCASE_CLEARANCE),
            STAIRCASE_BARS,
            proptest::collection::vec(finite_f64(BAR_JITTER), bars),
            proptest::collection::vec(finite_f64(BAR_HEIGHT), bars),
            proptest::collection::vec(finite_f64(0.0..=1.0), bars),
            proptest::collection::vec(any::<bool>(), bars),
            any::<bool>(),
        ),
        (
            any::<bool>(),
            any::<bool>(),
            angle(),
            finite_f64(1.0..=10.0),
        ),
    )
        .prop_map(build_staircase)
}

fn build_staircase(
    (
        (plane, beta, au, av, left),
        (clearance, bars, jitter, heights, keys, on_axis, touch),
        (hole, flipped, angle, length),
    ): StaircaseDraw,
) -> Sweep {
    // The axis in (u, v), and the material side of it.
    let along = Vec2::new(real::cos(beta), real::sin(beta));
    let left_normal = Vec2::new(-along.y, along.x);
    let radial = if left { left_normal } else { -left_normal };
    let origin = Point2::new(au, av);
    let at = |rho: f64, t: f64| origin + t * along + rho * radial;
    // The bars' outer radii: distinct by construction, in a random order.
    let n = bars
        .min(jitter.len())
        .min(heights.len())
        .min(keys.len())
        .min(on_axis.len());
    // Each bar's inner radius: the clearance, or on the axis where the
    // draw touches it — the first bar when the draw names none.
    let named = on_axis.iter().take(n).any(|&b| b);
    let inner: Vec<f64> = (0..n)
        .map(|k| {
            if touch && (on_axis[k] || (!named && k == 0)) {
                0.0
            } else {
                clearance
            }
        })
        .collect();
    let mut levels: Vec<(f64, f64)> = (0..n)
        .map(|k| {
            let share = STAIRCASE_REACH / n as f64;
            (keys[k], clearance + share * (k as f64 + jitter[k]))
        })
        .collect();
    levels.sort_by(|a, b| a.0.total_cmp(&b.0));
    let rho: Vec<f64> = levels.iter().map(|l| l.1).collect();
    let mut t = vec![0.0];
    for k in 0..n {
        t.push(t[k] + heights[k]);
    }
    // The histogram: up the outer sides, then down the inner sides with a
    // step wherever two bars' inner sides differ; down the first bar's
    // inner side is the closing segment.
    let mut points = vec![at(inner[0], t[0])];
    for k in 0..n {
        points.push(at(rho[k], t[k]));
        points.push(at(rho[k], t[k + 1]));
    }
    points.push(at(inner[n - 1], t[n]));
    for k in (1..n).rev() {
        if inner[k] != inner[k - 1] {
            points.push(at(inner[k], t[k]));
            points.push(at(inner[k - 1], t[k]));
        }
    }
    let no_arcs = vec![None; points.len()];
    let outer = path_loop(&points, &no_arcs, flipped);
    let holes = if hole {
        // A rectangle in the middle of the first bar.
        let (ra, rb) = (
            inner[0] + 0.25 * (rho[0] - inner[0]),
            inner[0] + 0.75 * (rho[0] - inner[0]),
        );
        let (ta, tb) = (t[0] + 0.25 * heights[0], t[0] + 0.75 * heights[0]);
        let corners = [at(ra, ta), at(rb, ta), at(rb, tb), at(ra, tb)];
        vec![path_loop(&corners, &[None; 4], !flipped)]
    } else {
        Vec::new()
    };
    let direction = plane.vec_to_world(Vec3::new(along.x, along.y, 0.0));
    Sweep {
        profile: Profile {
            plane,
            outer,
            holes,
        },
        axis: Axis {
            origin: plane.to_world(Point3::new(au, av, 0.0)),
            direction: UnitVec3::new_normalize(direction),
        },
        angle,
        length,
    }
}

/// What one segment of a [`general`] profile sweeps about the axis.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Sweeps {
    /// A line: a cone with its apex on the axis, in either orientation
    /// (a cylinder or a plane only where the draw happens to align it).
    Line,
    /// An arc bulging outward by this fraction of its chord: a torus.
    Torus(f64),
    /// An arc centred on the axis: a sphere (a line where the chord is
    /// too nearly perpendicular to the axis, see [`SPHERE_MIN_SLOPE`]).
    Sphere,
}

/// Vertices of a [`general`] profile's outer loop: with [`JITTER`], at
/// least six keeps every chord under a right angle as seen from the
/// centre, which is what keeps the deep outward arcs inside their chords'
/// sectors (see [`general`]).
pub const GENERAL_VERTICES: RangeInclusive<usize> = 6..=9;

/// The sagitta of a [`general`] profile's torus arc as a fraction of its
/// chord: deep enough that the arc's whole circle stays clear of the
/// axis at [`AXIS_DISTANCE`], at most a semicircle.
pub const GENERAL_SAGITTA_FRACTION: RangeInclusive<f64> = 0.2..=0.5;

/// The distance of a [`general`] profile's centre from the axis. Its
/// vertices lie within [`PROFILE_RADIUS`] of the centre, a torus arc's
/// circle has its centre within `PROFILE_RADIUS + 0.525·chord` of it and
/// a radius at most `0.725·chord` (the ends of [`GENERAL_SAGITTA_FRACTION`]),
/// and a chord under a right angle is at most `√2·PROFILE_RADIUS`, so the
/// least distance here leaves every such circle more than two units clear
/// of the axis.
pub const AXIS_DISTANCE: RangeInclusive<f64> = 30.0..=40.0;

/// The least `|chord · axis|` a sphere arc is drawn on. The arc's centre
/// is where the chord's bisector meets the axis, at most
/// `ρ(midpoint) / slope` along it; below this slope the centre runs away
/// and the arc flattens until three points no longer place its circle
/// to the model's tolerance, so the segment stays a line instead.
pub const SPHERE_MIN_SLOPE: f64 = 0.5;

/// How clear of the axis a [`general`] profile with a side along it keeps
/// a torus arc's circle: a chord whose arc would come nearer stays a line.
pub const TORUS_CLEARANCE: f64 = 1.0;

/// The raw draw of [`general`].
type GeneralDraw = (
    (Frame, f64, f64, bool, bool),
    (f64, Vec<f64>, Vec<Sweeps>),
    (usize, Vec<bool>, f64, f64, usize),
);

/// A profile beside an axis whose segments sweep every surface kind, both
/// in a random plane pose: a convex polygon of [`GENERAL_VERTICES`]
/// vertices on the circle of [`PROFILE_RADIUS`] about the centre, the
/// centre at [`AXIS_DISTANCE`] from the axis on either side of it, each
/// chord kept as a line (a cone, its apex on the axis, widening or
/// narrowing as the draw falls), replaced by an outward arc of
/// [`GENERAL_SAGITTA_FRACTION`] (a torus, its circle clear of the axis by
/// the distance's bound) or by the arc centred on the axis through its
/// ends (a sphere; a line where the chord is within [`SPHERE_MIN_SLOPE`]
/// of perpendicular to the axis), with zero, one or two holes as
/// [`star`] draws them and every loop written in either orientation. One
/// time in three the axis runs instead through one side of the polygon,
/// which then lies along it: the two chords beside that side close at the
/// axis, a line as a cone's apex and an arc centred on the axis as a
/// sphere's pole (a torus arc there is drawn as a line), no other chord
/// takes a sphere arc, and a torus arc whose circle would come within
/// [`TORUS_CLEARANCE`] of the axis is a line. The angle is a full turn one
/// time in four, otherwise partial and clear of both ends of `(0, 2π)`;
/// the extrude length is random.
///
/// The polygon is convex, unlike [`star`]'s, because the torus arcs are
/// deep: two outward arcs meeting at a notch would cross. On a convex
/// polygon an outward arc of at most a semicircle lies in the half-disc
/// on its chord, which lies in the chord's sector from the centre while
/// the chord subtends less than a right angle, so arcs on different
/// chords never meet away from their shared vertex, and the half-discs
/// on adjacent chords meet only there (their circles' other common point
/// is the foot of the vertex on the third side). A sphere arc is shallow
/// — its radius is at least the profile's distance from the axis — and
/// bulges away from the axis, inward on the axis side of the polygon,
/// where the holes' disc is shrunk by its sagitta. With a side along the
/// axis a pole's arc leaves the axis square to it, into the polygon, and
/// turns by less than the convex corner at its other end leaves room for,
/// which is why no other chord may bulge inward too.
pub fn general() -> impl Strategy<Value = Sweep> {
    let vertices = GENERAL_VERTICES;
    let kind = prop_oneof![
        Just(Sweeps::Line),
        finite_f64(GENERAL_SAGITTA_FRACTION).prop_map(Sweeps::Torus),
        Just(Sweeps::Sphere),
    ];
    (
        (
            frame(),
            finite_f64(0.0..=TAU),
            finite_f64(-5.0..=5.0),
            any::<bool>(),
            proptest::bool::weighted(1.0 / 3.0),
        ),
        (
            finite_f64(AXIS_DISTANCE),
            proptest::collection::vec(finite_f64(-JITTER..=JITTER), vertices.clone()),
            proptest::collection::vec(kind, vertices),
        ),
        (
            0usize..=2,
            proptest::collection::vec(any::<bool>(), 2),
            angle(),
            finite_f64(1.0..=10.0),
            0usize..*GENERAL_VERTICES.end(),
        ),
    )
        .prop_map(build_general)
}

fn build_general(
    (
        (plane, beta, slide, left, touch),
        (distance, jitter, kinds),
        (holes, flipped, angle, length, side),
    ): GeneralDraw,
) -> Sweep {
    let n = jitter.len().min(kinds.len());
    let centre = Point2::origin();
    let share = TAU / n as f64;
    let points: Vec<Point2> = (0..n)
        .map(|k| {
            let a = (k as f64 + jitter[k]) * share;
            Point2::new(PROFILE_RADIUS * real::cos(a), PROFILE_RADIUS * real::sin(a))
        })
        .collect();
    // The side along the axis, when the draw touches it.
    let touching = touch.then_some(side % n);
    let (axis_origin, along, radial) = match touching {
        Some(s) => {
            // The polygon runs counter-clockwise, so it lies on the left of
            // its side `s`, whichever way the axis runs along that side.
            let d = (points[(s + 1) % n] - points[s]).normalize();
            let along = if left { d } else { -d };
            (points[s], along, Vec2::new(-d.y, d.x))
        }
        None => {
            let along = Vec2::new(real::cos(beta), real::sin(beta));
            let left_normal = Vec2::new(-along.y, along.x);
            let radial = if left { left_normal } else { -left_normal };
            // The axis `distance` from the centre, the profile on its
            // `radial` side.
            (centre - distance * radial + slide * along, along, radial)
        }
    };
    let rho = |p: Point2| (p - axis_origin).dot(&radial);
    // Whether chord `k` has an end on the side along the axis.
    let beside = |k: usize| touching.is_some_and(|s| k == (s + 1) % n || (k + 1) % n == s);
    let vias: Vec<Option<Point2>> = (0..n)
        .map(|k| {
            let (a, b) = (points[k], points[(k + 1) % n]);
            let chord = b - a;
            let mid = a + chord / 2.0;
            if touching == Some(k) {
                return None;
            }
            match kinds[k] {
                Sweeps::Line => None,
                Sweeps::Torus(fraction) => {
                    let outward = (mid - centre).normalize();
                    let via = mid + fraction * chord.norm() * outward;
                    // The arc's circle, its centre `radius` back from the
                    // via along the chord's bisector.
                    let (half, sagitta) = (chord.norm() / 2.0, fraction * chord.norm());
                    let radius = (half * half + sagitta * sagitta) / (2.0 * sagitta);
                    let clear = rho(via - radius * outward) - radius;
                    let near = touching.is_some() && (beside(k) || clear < TORUS_CLEARANCE);
                    (!near).then_some(via)
                }
                Sweeps::Sphere => {
                    let g = chord.normalize();
                    if g.dot(&along).abs() < SPHERE_MIN_SLOPE || (touching.is_some() && !beside(k))
                    {
                        return None;
                    }
                    // The chord's bisector meets the axis where ρ vanishes.
                    let bisector = Vec2::new(-g.y, g.x);
                    let on_axis = mid - rho(mid) / bisector.dot(&radial) * bisector;
                    let r = (a - on_axis).norm();
                    Some(on_axis + r * (mid - on_axis).normalize())
                }
            }
        })
        .collect();
    // The largest disc about the centre the chords and the inward arcs
    // leave free.
    let free = (0..n)
        .map(|k| {
            let (a, b) = (points[k], points[(k + 1) % n]);
            let mid = a + (b - a) / 2.0;
            let toward_centre = (centre - mid).normalize();
            let inward = vias[k].map_or(0.0, |via| (via - mid).dot(&toward_centre).max(0.0));
            distance_to_segment(centre, a, b) - inward
        })
        .fold(f64::INFINITY, f64::min);
    let direction = plane.vec_to_world(Vec3::new(along.x, along.y, 0.0));
    Sweep {
        profile: Profile {
            plane,
            outer: path_loop(&points, &vias, flipped[0]),
            holes: holes_within(free, holes, flipped[1]),
        },
        axis: Axis {
            origin: plane.to_world(Point3::new(axis_origin.x, axis_origin.y, 0.0)),
            direction: UnitVec3::new_normalize(direction),
        },
        angle,
        length,
    }
}
