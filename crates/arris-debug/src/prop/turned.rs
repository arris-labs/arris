//! Turned parts: a profile in the `xy` plane of lines and one circular arc,
//! revolved a whole turn about the `y` axis, with some of its circular edges
//! blended through one call (ADR-0036). `x` is the radius and `y` the height
//! along the axis, so the part is a stack of cylinders, a coned shoulder, a
//! domed end or a toroidal bead on a disc, the corner of each pair of
//! neighbouring faces one edge of the solid — the pairs of the meridian
//! row, a plane, a cylinder, a cone, a sphere or a torus coaxial with it.
//!
//! The strategy draws the proportions so that no blend leaves its face: the
//! size is a fraction of the shortest meridian and of the smallest radius
//! beside any blended corner, and every corner turns the outline through
//! at most about 60°, so a ball touches within its own radius of the edge.
//! The part's closed forms are the property test's
//! (`crates/arris-ops/tests/blend_prop.rs`).

use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_math::{Frame, Point2, Point3};
use proptest::prelude::*;

use super::finite_f64;

/// One meridian of the outline, from the end of the one before it (the
/// first from [`Turned::start`]).
#[derive(Debug, Clone, PartialEq)]
pub enum Piece {
    /// A straight segment: a plane square to the axis, a cylinder or a
    /// cone.
    Line {
        /// Where it ends.
        to: Point2,
    },
    /// A circular arc: a sphere centred on the axis, or a torus.
    Arc {
        /// Where it ends.
        to: Point2,
        /// A point between its ends.
        via: Point2,
        /// The circle's centre.
        centre: Point2,
        /// The circle's radius.
        radius: f64,
    },
}

impl Piece {
    /// Where the piece ends.
    pub fn end(&self) -> Point2 {
        match self {
            Piece::Line { to } | Piece::Arc { to, .. } => *to,
        }
    }
}

/// A turned part and the blend drawn for it.
#[derive(Debug, Clone, PartialEq)]
pub struct Turned {
    /// Where the outline starts, on the axis.
    pub start: Point2,
    /// The outline's pieces, the last one the closing run along the axis.
    pub pieces: Vec<Piece>,
    /// The corners blended: corner `k` is where piece `k − 1` meets piece
    /// `k`, `k` in `1 .. pieces.len() − 1`.
    pub blended: Vec<usize>,
    /// The radius of a fillet or the distance of a chamfer.
    pub size: f64,
    /// A chamfer rather than a fillet.
    pub chamfer: bool,
}

impl Turned {
    /// Corner `k` of the outline, in (radius, height).
    pub fn corner(&self, k: usize) -> Point2 {
        self.pieces[k - 1].end()
    }

    /// Where piece `k` starts.
    pub fn from(&self, k: usize) -> Point2 {
        if k == 0 {
            self.start
        } else {
            self.pieces[k - 1].end()
        }
    }

    /// The length of piece `k`'s meridian.
    pub fn length(&self, k: usize) -> f64 {
        let from = self.from(k);
        match &self.pieces[k] {
            Piece::Line { to } => (to - from).norm(),
            Piece::Arc {
                to,
                via,
                centre,
                radius,
            } => radius * sweep(from, *via, *to, *centre).abs(),
        }
    }

    /// The edge of corner `k` as a point of the revolved solid, a quarter
    /// turn from the seam in the profile's plane.
    pub fn edge_point(&self, k: usize) -> Point3 {
        let c = self.corner(k);
        Point3::new(0.0, c.y, -c.x)
    }

    /// A point of the solid on its axis.
    pub fn centre(&self) -> Point3 {
        let top = self.pieces[self.pieces.len() - 2].end();
        Point3::new(0.0, top.y / 2.0, 0.0)
    }

    /// The outline as a profile in the world's `xy` plane.
    pub fn profile(&self) -> Profile {
        Profile {
            plane: Frame::world(),
            outer: ProfileLoop::Path {
                start: self.start,
                segments: self
                    .pieces
                    .iter()
                    .map(|p| match p {
                        Piece::Line { to } => ProfileSegment::LineTo(*to),
                        Piece::Arc { to, via, .. } => ProfileSegment::ArcTo { to: *to, via: *via },
                    })
                    .collect(),
            },
            holes: Vec::new(),
        }
    }
}

/// The signed angle swept from `from` to `to` about `centre` through `via`.
pub fn sweep(from: Point2, via: Point2, to: Point2, centre: Point2) -> f64 {
    let angle = |p: Point2| (p.y - centre.y).atan2(p.x - centre.x);
    let wrap = |a: f64| a.rem_euclid(core::f64::consts::TAU);
    let (a, b) = (
        wrap(angle(via) - angle(from)),
        wrap(angle(to) - angle(from)),
    );
    if a <= b {
        b
    } else {
        b - core::f64::consts::TAU
    }
}

fn p(x: f64, y: f64) -> Point2 {
    Point2::new(x, y)
}

fn line(x: f64, y: f64) -> Piece {
    Piece::Line { to: p(x, y) }
}

/// An arc of the circle about `centre` of `radius` from angle `from` to
/// angle `to`, through the angle between.
fn arc(centre: Point2, radius: f64, from: f64, to: f64) -> Piece {
    let at = |a: f64| p(centre.x + radius * a.cos(), centre.y + radius * a.sin());
    Piece::Arc {
        to: at(to),
        via: at((from + to) / 2.0),
        centre,
        radius,
    }
}

/// A cylinder, a coned shoulder narrowing or widening it and a second
/// cylinder: corners 1 to 4, plane–cylinder, cylinder–cone, cone–cylinder
/// and cylinder–plane.
fn shoulder() -> impl Strategy<Value = (Point2, Vec<Piece>)> {
    (
        finite_f64(1.0..=3.0),
        finite_f64(0.8..=3.0),
        finite_f64(0.8..=3.0),
        prop_oneof![finite_f64(0.2..=0.75), finite_f64(-0.6..=-0.2)],
        finite_f64(0.25..=1.0),
    )
        .prop_map(|(r1, h1, h3, t, slope)| {
            let r2 = r1 * (1.0 - t);
            let h2 = (r1 - r2).abs() / slope.tan();
            let top = h1 + h2 + h3;
            (
                p(0.0, 0.0),
                vec![
                    line(r1, 0.0),
                    line(r1, h1),
                    line(r2, h1 + h2),
                    line(r2, top),
                    line(0.0, top),
                    line(0.0, 0.0),
                ],
            )
        })
}

/// A cylinder under a spherical dome centred on its axis that meets it at
/// an angle: corners 1 and 2, plane–cylinder and cylinder–sphere.
fn dome() -> impl Strategy<Value = (Point2, Vec<Piece>)> {
    (
        finite_f64(1.0..=3.0),
        finite_f64(1.0..=4.0),
        finite_f64(0.4..=2.0),
    )
        .prop_map(|(r, w, k)| {
            // The sphere's centre on the axis, `k·r` below the rim.
            let c = w - k * r;
            let s = r * (1.0 + k * k).sqrt();
            // The rim is at angle `atan k` above the sphere's equator, the
            // apex at `π/2`.
            let centre = p(0.0, c);
            let from = k.atan();
            let to = core::f64::consts::FRAC_PI_2;
            (
                p(0.0, 0.0),
                vec![
                    line(r, 0.0),
                    line(r, w),
                    arc(centre, s, from, to),
                    line(0.0, 0.0),
                ],
            )
        })
}

/// A cylinder under a toroidal bead, the arc of a tube about a circle off
/// the axis, meeting the cylinder and the flat above it at an angle: corners
/// 1 to 3, plane–cylinder, cylinder–torus and torus–plane.
fn bead() -> impl Strategy<Value = (Point2, Vec<Piece>)> {
    (
        finite_f64(0.5..=1.5),
        finite_f64(1.5..=3.0),
        finite_f64(1.0..=3.0),
        finite_f64(20.0..=60.0),
        finite_f64(110.0..=150.0),
    )
        .prop_map(|(q, g, w, a1, a2)| {
            let (a1, a2) = (a1.to_radians(), a2.to_radians());
            let cx = g * q;
            let r = cx + q * a1.cos();
            let cz = w - q * a1.sin();
            let end = p(cx + q * a2.cos(), cz + q * a2.sin());
            (
                p(0.0, 0.0),
                vec![
                    line(r, 0.0),
                    line(r, w),
                    arc(p(cx, cz), q, a1, a2),
                    line(0.0, end.y),
                    line(0.0, 0.0),
                ],
            )
        })
}

/// Turned parts with a blend drawn for some of their corners, one call.
pub fn turned() -> impl Strategy<Value = Turned> {
    (
        prop_oneof![shoulder(), dome(), bead()],
        finite_f64(0.0..=1.0),
        proptest::collection::vec(finite_f64(0.0..=1.0), 4),
        finite_f64(0.05..=0.25),
        any::<bool>(),
    )
        .prop_map(|((start, pieces), density, draws, fraction, chamfer)| {
            let mut t = Turned {
                start,
                pieces,
                blended: Vec::new(),
                size: 0.0,
                chamfer,
            };
            // A pick of the corners, never none: the density's own pick,
            // else the first.
            let corners = 1..t.pieces.len() - 1;
            let mut blended: Vec<usize> = corners
                .clone()
                .zip(draws.iter().cycle())
                .filter(|(_, u)| **u < density)
                .map(|(k, _)| k)
                .collect();
            if blended.is_empty() {
                blended.push(corners.start);
            }
            let room = blended
                .iter()
                .map(|&k| t.length(k - 1).min(t.length(k)).min(t.corner(k).x))
                .fold(f64::INFINITY, f64::min);
            t.size = fraction * room;
            t.blended = blended;
            t
        })
}
