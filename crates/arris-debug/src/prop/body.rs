//! Bodies in random poses: the operands of the boolean property tests.
//! Each strategy yields a *description* of a body — its numbers and a
//! rigid motion — that [`Boxed::build`], [`Cylindrical::build`] and
//! [`OverlappingPair::build`] turn into bodies of a model through the
//! primitives and `ops::transform`, so a failing case prints as numbers a
//! fixture can be written from.

use crate::unmetered::{extrude, primitive_box, primitive_cylinder, revolve, transform};
use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{Axis, Frame, Isometry, Point2, Point3, UnitVec3, Vec3};
use arris_ops::OpError;
use arris_topo::{Body, Model};
use proptest::prelude::*;

use core::ops::RangeInclusive;

use super::{DEFAULT_SCALE, finite_f64, pose_in, radius, rotation, unit_vec3};

/// The smallest extent (a box side, a cylinder's diameter or height) the
/// strategies produce.
pub const MIN_EXTENT: f64 = 1.0;
/// The largest.
pub const MAX_EXTENT: f64 = 20.0;

/// An axis-aligned box moved by a rigid motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Boxed {
    /// The corner before the motion.
    pub min: Point3,
    /// The opposite corner.
    pub max: Point3,
    /// The motion.
    pub pose: Isometry,
}

impl Boxed {
    /// The box as a body of `m`: `primitive_box` then `transform`.
    pub fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        let (body, _) = primitive_box(m, self.min, self.max)?;
        Ok(transform(m, body, &self.pose)?.0)
    }
}

/// A cylinder moved by a rigid motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cylindrical {
    /// The axis before the motion, its origin the base cap's centre.
    pub axis: Axis,
    /// The radius.
    pub radius: f64,
    /// The height along the axis.
    pub height: f64,
    /// The motion.
    pub pose: Isometry,
}

impl Cylindrical {
    /// The cylinder as a body of `m`: `primitive_cylinder` then
    /// `transform`.
    pub fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        let (body, _) = primitive_cylinder(m, self.axis, self.radius, self.height)?;
        Ok(transform(m, body, &self.pose)?.0)
    }
}

/// A box and a cylinder whose axis passes through the box's interior
/// and whose length exceeds the box's diagonal, both under one motion:
/// every pair intersects, and a corner of the box lies outside the
/// cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OverlappingPair {
    /// The box.
    pub cuboid: Boxed,
    /// The cylinder.
    pub cylinder: Cylindrical,
}

impl OverlappingPair {
    /// Both bodies in `m`, the box first.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        let a = self.cuboid.build(m)?;
        let b = self.cylinder.build(m)?;
        Ok((a, b))
    }

    /// Whether the cylinder's wall clears every edge of the box: the
    /// distance from each of the twelve edge segments to the axis line
    /// exceeds the radius. Then the cylinder enters through the interior
    /// of one face and leaves through the interior of another (every
    /// ruling meets the box in one segment, so the wall's entry and exit
    /// curves are closed curves that cross no edge), so `box − cylinder`
    /// is one shell and `cylinder − box` exactly two. Decided before the
    /// motion, which both operands share.
    pub fn pierces(&self) -> bool {
        let (lo, hi) = (self.cuboid.min, self.cuboid.max);
        let corner = |i: usize| {
            Point3::new(
                if i & 1 == 0 { lo.x } else { hi.x },
                if i & 2 == 0 { lo.y } else { hi.y },
                if i & 4 == 0 { lo.z } else { hi.z },
            )
        };
        let axis = &self.cylinder.axis;
        let d = axis.direction.into_inner();
        // The component of `v` perpendicular to the axis.
        let across = |v: Vec3| v - d * v.dot(&d);
        (0..8)
            .flat_map(|i| {
                [1usize, 2, 4]
                    .into_iter()
                    .filter(move |b| i & b == 0)
                    .map(move |b| (i, i | b))
            })
            .all(|(i, j)| {
                // The nearest point of the segment to the axis line, by
                // minimising the perpendicular offset over the segment.
                let a = across(corner(i) - axis.origin);
                let b = across(corner(j) - corner(i));
                let bb = b.dot(&b);
                let s = if bb > 0.0 {
                    (-a.dot(&b) / bb).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                (a + b * s).norm() > self.cylinder.radius
            })
    }
}

/// A box cut by one or two **bars**, and the same recipe with every bar
/// moved and resized: the operands of the split-order property
/// (ADR-0009, `docs/DATA-MODEL.md` §Provenance). A bar is a slab thin
/// along one axis of the box's own frame and past the box in the other
/// two, so cutting with it splits the box in two lumps and every face it
/// crosses into two pieces, and no bar ever ends inside a face.
///
/// The two builds' bars are drawn from bands that keep them apart, off
/// the box's own faces and in the same order — bar 0 inside the first
/// half, bar 1 inside the second — and the second build's box is resized
/// along each axis and posed for itself. So **which entities bound which
/// piece is the same in both builds** however far the bars moved and
/// however the box was resized or turned, which is the bound the split
/// order is guaranteed under.
#[derive(Debug, Clone, PartialEq)]
pub struct BarCut {
    /// The box of the first build.
    pub cuboid: Boxed,
    /// The box of the second: the same box resized along each axis and
    /// in a pose of its own, so nothing a world axis decides survives.
    pub rebuilt: Boxed,
    /// The axis the bars are thin along, in the box's own frame.
    pub axis: usize,
    /// Each bar's centre and half-width along that axis as fractions of
    /// the box's extent there, in the first build.
    pub bars: Vec<(f64, f64)>,
    /// The same bars in the second build; the same length as `bars`.
    pub moved: Vec<(f64, f64)>,
}

impl BarCut {
    /// The box of build `which` (`0` or `1`).
    pub fn cuboid_of(&self, which: usize) -> Boxed {
        if which == 0 {
            self.cuboid
        } else {
            self.rebuilt
        }
    }

    /// The bars of build `which` as boxes in that build's own pose, in
    /// the order they are cut with: along the axis they are thin on, so
    /// the first cut splits the box and the second splits one of its
    /// lumps.
    pub fn slabs(&self, which: usize) -> Vec<Boxed> {
        let cuboid = self.cuboid_of(which);
        let e = cuboid.max - cuboid.min;
        let bars = if which == 0 { &self.bars } else { &self.moved };
        bars.iter()
            .map(|&(centre, half)| {
                // Past the box in the other two axes, by a quarter of the
                // extent there: the bar's own faces never touch the box.
                let pad = e / 4.0;
                let mut min = cuboid.min - pad;
                let mut max = cuboid.max + pad;
                min[self.axis] = cuboid.min[self.axis] + (centre - half) * e[self.axis];
                max[self.axis] = cuboid.min[self.axis] + (centre + half) * e[self.axis];
                Boxed {
                    min,
                    max,
                    pose: cuboid.pose,
                }
            })
            .collect()
    }
}

/// A box and a cylinder whose wall touches one face of the box from
/// outside along a ruling through the face's interior, both under one
/// motion: the tangent case with nothing else in contact (plan
/// m4-booleans step 11). Every boolean's outcome is known: `box −
/// cylinder` is the box, `common` selects nothing, and `fuse` would
/// hold the face and the wall touching along a slit, the designed
/// `TangentContact` refusal.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TangentPair {
    /// The box.
    pub cuboid: Boxed,
    /// The cylinder.
    pub cylinder: Cylindrical,
    /// The touched face: the axis it is normal to (`0`, `1`, `2`) and
    /// whether it is the box's `max` side.
    pub face: (usize, bool),
}

impl TangentPair {
    /// Both bodies in `m`, the box first.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        let a = self.cuboid.build(m)?;
        let b = self.cylinder.build(m)?;
        Ok((a, b))
    }

    /// The outward normal of the touched face, before the motion.
    pub fn normal(&self) -> Vec3 {
        let mut n = Vec3::zeros();
        n[self.face.0] = if self.face.1 { 1.0 } else { -1.0 };
        n
    }

    /// Where the ruling crosses the touched face: the foot of the axis's
    /// midpoint on the face's plane, before the motion.
    pub fn foot(&self) -> Point3 {
        let mid = self.cylinder.axis.at(self.cylinder.height / 2.0);
        mid - self.cylinder.radius * self.normal()
    }
}

/// [`TangentPair`]s: a box from [`box_in`], a face of it, a point in the
/// face's interior (a tenth to nine tenths of each extent), an axis
/// direction in the face's plane at a random angle, and a cylinder of
/// random radius and height on that direction at the radius's distance
/// outside the face, its midpoint over the point — so the ruling
/// crosses the face's interior and the two faces touch along a segment
/// interior to both. The cylinder's ends may reach past the box or stop
/// short of it: both kinds of touch — a box edge on the wall and a rim
/// on the face — occur.
pub fn tangent_pair() -> impl Strategy<Value = TangentPair> {
    (
        box_in(DEFAULT_SCALE),
        0usize..3,
        any::<bool>(),
        (finite_f64(0.1..=0.9), finite_f64(0.1..=0.9)),
        finite_f64(0.0..=core::f64::consts::TAU),
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        radius(MIN_EXTENT..=MAX_EXTENT),
    )
        .prop_filter_map(
            "a cylinder tangent to a box face",
            |(cuboid, axis_index, max_side, (fu, fv), angle, r, height)| {
                let e = cuboid.max - cuboid.min;
                let (i, j, k) = (axis_index, (axis_index + 1) % 3, (axis_index + 2) % 3);
                let mut foot = cuboid.min;
                foot[i] = if max_side {
                    cuboid.max[i]
                } else {
                    cuboid.min[i]
                };
                foot[j] += fu * e[j];
                foot[k] += fv * e[k];
                let mut normal = Vec3::zeros();
                normal[i] = if max_side { 1.0 } else { -1.0 };
                let mut d = Vec3::zeros();
                d[j] = angle.cos();
                d[k] = angle.sin();
                let mid = foot + r * normal;
                let axis = Axis::new(mid - (height / 2.0) * d, d).ok()?;
                Some(TangentPair {
                    cuboid,
                    cylinder: Cylindrical {
                        axis,
                        radius: r,
                        height,
                        pose: cuboid.pose,
                    },
                    face: (i, max_side),
                })
            },
        )
}

/// Where a [`ParallelPair`]'s second cylinder stands against the first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParallelKind {
    /// It reaches past both caps of the first, at a random angle about it.
    Clear,
    /// It reaches past both caps, and one ruling of the pair is the
    /// first cylinder's seam.
    SeamOnRuling,
    /// It is as tall as the first and their caps are coincident: the rim
    /// circles cross on each cap plane at the rulings' ends.
    FlushCaps,
}

/// Two cylinders on parallel axes whose walls cross in two rulings, both
/// under one motion: `a` on the
/// `z` axis centred at the origin, `b`'s axis `distance` from it, strictly
/// between the internal and external tangent distances `|R₁ − R₂|` and
/// `R₁ + R₂`, so neither wall holds the other and neither touches it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ParallelPair {
    /// The first cylinder.
    pub a: Cylindrical,
    /// The second.
    pub b: Cylindrical,
    /// The distance between the axes.
    pub distance: f64,
    /// Where `b` stands.
    pub kind: ParallelKind,
}

impl ParallelPair {
    /// Both bodies in `m`, `a` first.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        Ok((self.a.build(m)?, self.b.build(m)?))
    }
}

/// [`ParallelPair`]s: two radii, the axes' distance drawn between the
/// tangent distances a tenth of the gap clear of each, and `b` placed at
/// a random angle about `a` — a quarter [`ParallelKind::SeamOnRuling`],
/// a quarter [`ParallelKind::FlushCaps`], the rest
/// [`ParallelKind::Clear`], reaching a tenth to a half of `a`'s height
/// past each cap.
pub fn parallel_pair() -> impl Strategy<Value = ParallelPair> {
    (
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        finite_f64(0.1..=0.9),
        finite_f64(0.0..=core::f64::consts::TAU),
        radius(MIN_EXTENT..=MAX_EXTENT),
        finite_f64(0.1..=0.5),
        0u8..4,
        pose_in(DEFAULT_SCALE),
    )
        .prop_map(|(r1, r2, gap, angle, height, reach, kind, pose)| {
            let (near, far) = ((r1 - r2).abs(), r1 + r2);
            let distance = near + gap * (far - near);
            let kind = match kind {
                0 => ParallelKind::SeamOnRuling,
                1 => ParallelKind::FlushCaps,
                _ => ParallelKind::Clear,
            };
            // `a`'s seam runs along its frame's `x`, through (r1, 0); the
            // rulings are at `angle ± α` about `a`'s axis, with `cos α` from
            // the triangle of the two radii and the distance.
            let angle = if kind == ParallelKind::SeamOnRuling {
                ((distance * distance + r1 * r1 - r2 * r2) / (2.0 * distance * r1))
                    .clamp(-1.0, 1.0)
                    .acos()
            } else {
                angle
            };
            let (below, tall) = if kind == ParallelKind::FlushCaps {
                (0.0, height)
            } else {
                (reach * height, height * (1.0 + 2.0 * reach))
            };
            let centre = Point3::new(distance * angle.cos(), distance * angle.sin(), 0.0);
            ParallelPair {
                a: Cylindrical {
                    axis: Axis::z_at(Point3::new(0.0, 0.0, -height / 2.0)),
                    radius: r1,
                    height,
                    pose,
                },
                b: Cylindrical {
                    axis: Axis::z_at(centre - Vec3::new(0.0, 0.0, height / 2.0 + below)),
                    radius: r2,
                    height: tall,
                    pose,
                },
                distance,
                kind,
            }
        })
}

/// Two cylinders of one radius whose axes cross at their midpoints at an
/// angle `psi`, each through the other, both under one motion: `a` on the
/// `z` axis centred
/// at the origin, `b` in the `xz` plane, turned about its own axis first.
/// Their walls cross in two ellipses, which cross each other at
/// `(0, ±R, 0)` before the motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CrossingPair {
    /// The first cylinder.
    pub a: Cylindrical,
    /// The second; its pose is a turn about its own axis, then `a`'s.
    pub b: Cylindrical,
    /// The angle between the axes, in radians.
    pub psi: f64,
    /// Whether `b`'s turn puts its seam through the crossing vertex
    /// `(0, R, 0)`, where the seam is tangent to `a`'s wall.
    pub seam_through_crossing: bool,
}

impl CrossingPair {
    /// Both bodies in `m`, `a` first.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        Ok((self.a.build(m)?, self.b.build(m)?))
    }

    /// The volume of their common, the Steinmetz solid at `psi`:
    /// `16R³ / (3 sin ψ)`.
    pub fn common_volume(&self) -> f64 {
        16.0 * self.a.radius.powi(3) / (3.0 * self.psi.sin())
    }
}

/// [`CrossingPair`]s: a radius, `ψ ∈ [30°, 90°]`, both cylinders
/// `4R / sin ψ` to `8R / sin ψ` long — so each cap's disc stands clear of
/// the other wall, its centre more than `2R` from the other axis — and
/// `b` turned about its axis by a random angle, or in a quarter of cases
/// so its seam runs through the crossing vertex `(0, R, 0)`.
pub fn crossing_pair() -> impl Strategy<Value = CrossingPair> {
    (
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        finite_f64(30.0..=90.0),
        finite_f64(1.0..=2.0),
        finite_f64(0.0..=core::f64::consts::TAU),
        0u8..4,
        pose_in(DEFAULT_SCALE),
    )
        .prop_filter_map(
            "two crossing cylinders",
            |(r, psi_deg, stretch, turn, kind, pose)| {
                let psi = psi_deg.to_radians();
                let length = stretch * 4.0 * r / psi.sin();
                let d = Vec3::new(psi.sin(), 0.0, psi.cos());
                let seam_through_crossing = kind == 0;
                let turn = if seam_through_crossing {
                    // The seam runs along the frame's `x`, perpendicular to
                    // the axis like `y`: the rotation taking one to the
                    // other is about the axis.
                    let seam = Frame::from_z(Point3::origin(), d).ok()?.x().into_inner();
                    UnitQuaternion::rotation_between(&seam, &Vec3::y())?
                } else {
                    UnitQuaternion::from_axis_angle(&UnitVec3::try_new(d, 0.0)?, turn)
                };
                Some(CrossingPair {
                    a: Cylindrical {
                        axis: Axis::z_at(Point3::new(0.0, 0.0, -length / 2.0)),
                        radius: r,
                        height: length,
                        pose,
                    },
                    b: Cylindrical {
                        axis: Axis::new(Point3::origin() - (length / 2.0) * d, d).ok()?,
                        radius: r,
                        height: length,
                        pose: Isometry::from_rotation(turn).then(&pose),
                    },
                    psi,
                    seam_through_crossing,
                })
            },
        )
}

/// Two cylinders of unequal radii whose axes cross at an angle `psi`, or
/// pass skew at a distance `offset`, both under one motion: `a` on the `z`
/// axis centred at the origin, radius `R`; `b` narrower, radius `r`, its
/// axis in the direction `(sin ψ, 0, cos ψ)` through `(0, offset, 0)`,
/// turned about its own axis first. Their walls meet in the quartic that
/// is traced and fitted (ADR-0018): two loops round `b` where it passes
/// through `a` (`offset < R − r`), one where it breaks out of `a`'s side
/// (`R − r < offset < R + r`). Each cylinder is long enough that its caps
/// stand clear of the other's wall.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuarticPair {
    /// The wider cylinder.
    pub a: Cylindrical,
    /// The narrower; its pose is a turn about its own axis, then `a`'s.
    pub b: Cylindrical,
    /// The angle between the axes, in radians.
    pub psi: f64,
    /// The distance between the axes, zero where they cross.
    pub offset: f64,
}

impl QuarticPair {
    /// Both bodies in `m`, `a` first.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        Ok((self.a.build(m)?, self.b.build(m)?))
    }
}

/// [`QuarticPair`]s: `R`, `r / R ∈ [0.3, 0.85]`, `ψ ∈ [30°, 90°]`, and the
/// offset in one of three kinds a third each — zero, the axes crossing;
/// in `[0.1, 0.8]` of `R − r`, skew with `b` through `a`; or `R − r` plus
/// `[0.1, 0.9]` of `2r`, skew and breaking out. The section is singular
/// at `R − r` and `R + r`, where the walls touch, and the kinds keep a
/// tenth of the way clear of both. Both cylinders are `1.2` to `2` times
/// the length that puts their caps' discs past the other wall, and `b` is
/// turned about its axis by a random angle, so its seam is generic.
pub fn quartic_pair() -> impl Strategy<Value = QuarticPair> {
    (
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        finite_f64(0.3..=0.85),
        finite_f64(30.0..=90.0),
        (0u8..3, finite_f64(0.0..=1.0)),
        finite_f64(1.2..=2.0),
        finite_f64(0.0..=core::f64::consts::TAU),
        pose_in(DEFAULT_SCALE),
    )
        .prop_filter_map(
            "two cylinders meeting in a quartic",
            |(big, ratio, psi_deg, (kind, gap), stretch, turn, pose)| {
                let (r1, r2) = (big, ratio * big);
                let offset = match kind {
                    0 => 0.0,
                    1 => (0.1 + 0.7 * gap) * (r1 - r2),
                    _ => r1 - r2 + (0.1 + 0.8 * gap) * 2.0 * r2,
                };
                let psi = psi_deg.to_radians();
                let d = Vec3::new(psi.sin(), 0.0, psi.cos());
                // `b`'s cap discs, of radius `r`, stand more than `R + r`
                // from `a`'s axis; `a`'s caps above every point of `b`
                // within `R + r` of that axis.
                let long_b = stretch * 2.0 * (r1 + r2) / psi.sin();
                let long_a = stretch * 2.0 * ((r1 + r2) / psi.tan() + r1 + r2);
                let centre = Point3::new(0.0, offset, 0.0);
                let spin = UnitQuaternion::from_axis_angle(&UnitVec3::try_new(d, 0.0)?, turn);
                // The turn about `b`'s own axis, through `centre`.
                let about = Isometry::new(spin, centre.coords - spin * centre.coords);
                Some(QuarticPair {
                    a: Cylindrical {
                        axis: Axis::z_at(Point3::new(0.0, 0.0, -long_a / 2.0)),
                        radius: r1,
                        height: long_a,
                        pose,
                    },
                    b: Cylindrical {
                        axis: Axis::new(centre - (long_b / 2.0) * d, d).ok()?,
                        radius: r2,
                        height: long_b,
                        pose: about.then(&pose),
                    },
                    psi,
                    offset,
                })
            },
        )
}

/// A solid of revolution whose surface has a singular point on its
/// boundary, before its motion: about `z`, the point on the axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SingularSolid {
    /// A cone on its base disc in `z = 0`, the apex at `(0, 0, height)`.
    Cone {
        /// The base radius.
        radius: f64,
        /// The height of the apex over the base.
        height: f64,
    },
    /// A ball about the origin, the pole at `(0, 0, radius)`.
    Ball {
        /// The radius.
        radius: f64,
    },
}

impl SingularSolid {
    /// The apex, or the ball's north pole.
    pub fn singular_point(&self) -> Point3 {
        match *self {
            SingularSolid::Cone { height, .. } => Point3::new(0.0, 0.0, height),
            SingularSolid::Ball { radius } => Point3::new(0.0, 0.0, radius),
        }
    }

    /// The solid as a body of `m`: a triangle or a half disc in the
    /// `(x, z)` plane revolved a whole turn about `z`, so the singular
    /// point is a vertex holding a degenerate edge, as a revolve makes it.
    pub fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        let p = Point2::new;
        let outer = match *self {
            SingularSolid::Cone { radius, height } => ProfileLoop::Path {
                start: p(0.0, 0.0),
                segments: vec![
                    ProfileSegment::LineTo(p(radius, 0.0)),
                    ProfileSegment::LineTo(p(0.0, height)),
                    ProfileSegment::LineTo(p(0.0, 0.0)),
                ],
            },
            SingularSolid::Ball { radius } => ProfileLoop::Path {
                start: p(0.0, -radius),
                segments: vec![
                    ProfileSegment::ArcTo {
                        to: p(0.0, radius),
                        via: p(radius, 0.0),
                    },
                    ProfileSegment::LineTo(p(0.0, -radius)),
                ],
            },
        };
        // The (x, z) plane: the world frame a quarter turn about `x`.
        let plane = Frame::from_rotation(
            Point3::origin(),
            &UnitQuaternion::from_axis_angle(&Vec3::x_axis(), core::f64::consts::FRAC_PI_2),
        );
        let profile = Profile {
            plane,
            outer,
            holes: Vec::new(),
        };
        Ok(revolve(
            m,
            &profile,
            Axis::z_at(Point3::origin()),
            core::f64::consts::TAU,
        )?
        .0)
    }
}

/// A cone or a ball and a block one of whose faces runs through the apex
/// or the pole, both under one motion: the section is two rulings ending
/// on the apex, or a circle through the pole — through both poles when
/// the face holds the axis — and no edge of the block comes near the
/// solid, so the singular vertex and the seam's crossings are all that
/// pave it (ADR-0021).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SingularSlice {
    /// The solid, before `pose`.
    pub solid: SingularSolid,
    /// The block; its pose takes its `z = 0` face through the singular
    /// point, then `pose`.
    pub block: Boxed,
    /// The angle between the face's normal and the solid's axis, radians.
    pub tilt: f64,
    /// The motion of both.
    pub pose: Isometry,
}

impl SingularSlice {
    /// Both bodies in `m`, the solid first.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        let solid = self.solid.build(m)?;
        let solid = transform(m, solid, &self.pose)?.0;
        Ok((solid, self.block.build(m)?))
    }
}

/// How far from the axis a [`singular_slice`]'s ball face that does not
/// hold it is tilted: three degrees, fifty times the `√(2 tol / R)` of
/// the smallest ball at the default tolerance. A strategy's margin, not a
/// tolerance.
pub const SEAM_CLEARANCE: f64 = 0.05;

/// [`SingularSlice`]s, a cone or a ball a half each. The face's normal is
/// tilted from the axis by `90° ± 0.8 α` for a cone of half-angle `α` —
/// a plane through the apex nearer the axis than the rulings are, so it
/// cuts two of them, and a fifth of `α` clear of the plane that touches
/// along one — and by `15°` to `165°` for a ball, a quarter of those at
/// `90°` exactly: the great circle through both poles. The tilted face is
/// turned about the solid's axis and the block about the face's own
/// normal by random angles, so neither the seam nor the block's edges are
/// anywhere special; the block is four times the solid across.
///
/// The turn about the axis is the `u` a ball's section leaves its pole
/// at, anywhere round the turn, and in a quarter of the cases it is
/// `1e-7` to `1e-1` of a radian beside the seam's, evenly in its
/// logarithm, either side: nearer than `√(2 tol / R)` the seam and the
/// section are one curve within the tolerance over a stretch beside the
/// pole, and the section's block there is the seam's piece
/// (`boolean/pole-slice-beside-seam-cut`). Not for the great circle
/// through both poles: a meridian plane a hair off the seam's lies
/// within the tolerance of the seam over most of its length, and bounds
/// a lune a few tolerances wide beside it — the sliver faces of
/// `docs/BACKLOG.md`, two shrunk poses of which are ignored in
/// `boolean_prop.rs`; it keeps the turn drawn round the whole turn. Along
/// the seam exactly, and in its plane, are the variants of
/// `boolean/ball-pole-slice-cut`. A
/// ball's tilt keeps [`SEAM_CLEARANCE`] from `90°` unless it is `90°`
/// exactly: a circle through one pole at `90° + δ` passes the other
/// `2R sin δ` away, and beside a pole is `Reason::BesideSingularity` by
/// design (ADR-0021), met at `δ = 8e-5` in eight thousand poses.
pub fn singular_slice() -> impl Strategy<Value = SingularSlice> {
    (
        any::<bool>(),
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        finite_f64(MIN_EXTENT..=MAX_EXTENT),
        (0u8..4, finite_f64(-1.0..=1.0)),
        (
            any::<bool>(),
            finite_f64(0.0..=core::f64::consts::PI),
            (0u8..4, finite_f64(-7.0..=-1.0), any::<bool>()),
        ),
        finite_f64(0.0..=core::f64::consts::TAU),
        pose_in(DEFAULT_SCALE),
    )
        .prop_map(
            |(cone, radius, height, (kind, s), (far, spin, beside), turn, pose)| {
                let meridian = !cone && kind == 0;
                let spin = match beside {
                    (0, exponent, below) if !meridian => {
                        let off = 10f64.powf(exponent);
                        if below { -off } else { off }
                    }
                    _ => spin,
                };
                let spin = if far {
                    spin + core::f64::consts::PI
                } else {
                    spin
                };
                let solid = if cone {
                    SingularSolid::Cone { radius, height }
                } else {
                    SingularSolid::Ball { radius }
                };
                let quarter = core::f64::consts::FRAC_PI_2;
                let tilt = match solid {
                    SingularSolid::Cone { radius, height } => {
                        quarter + 0.8 * s * (radius / height).atan()
                    }
                    SingularSolid::Ball { .. } if kind == 0 => quarter,
                    SingularSolid::Ball { .. } => {
                        let most = 75f64.to_radians() - SEAM_CLEARANCE;
                        quarter + s.signum() * (SEAM_CLEARANCE + s.abs() * most)
                    }
                };
                let reach = 4.0 * radius.max(height);
                let at = solid.singular_point();
                let q = UnitQuaternion::from_axis_angle(&Vec3::z_axis(), spin)
                    * UnitQuaternion::from_axis_angle(&Vec3::x_axis(), tilt)
                    * UnitQuaternion::from_axis_angle(&Vec3::z_axis(), turn);
                let through = Isometry::new(q, at.coords);
                SingularSlice {
                    solid,
                    block: Boxed {
                        min: Point3::new(-reach, -reach, 0.0),
                        max: Point3::new(reach, reach, reach),
                        pose: through.then(&pose),
                    },
                    tilt,
                    pose,
                }
            },
        )
}

/// A solid whose curved face is a quadric other than a circular cylinder,
/// or a torus, before its motion: each built as Arris builds one for a
/// caller — a revolve or an extrude — about `z`, so a boolean meets the
/// face kinds a fillet, a chamfer or an extruded ellipse leaves behind.
/// None has a singular point on its boundary but the ball, whose poles
/// the revolve makes vertices.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QuadricSolid {
    /// A cone's frustum on its base disc in `z = 0`: radius `bottom` there
    /// and `top` at `z = height`, the two never equal, the apex off the
    /// body.
    Frustum {
        /// The radius in `z = 0`.
        bottom: f64,
        /// The radius in `z = height`.
        top: f64,
        /// The height.
        height: f64,
    },
    /// A ball about the origin.
    Ball {
        /// The radius.
        radius: f64,
    },
    /// A ring torus about `z` centred at the origin, `minor < major`.
    Ring {
        /// The distance from the axis to the tube's centre circle.
        major: f64,
        /// The tube's radius.
        minor: f64,
    },
    /// An elliptic cylinder's prism on its base in `z = 0`, the ellipse's
    /// semi-axes along `x` and `y`, never equal.
    EllipticPrism {
        /// The semi-axis along `x`.
        a: f64,
        /// The semi-axis along `y`.
        b: f64,
        /// The height.
        height: f64,
    },
}

impl QuadricSolid {
    /// The point `(s, t, w)` of the unit cube names inside the solid, each
    /// coordinate kept a tenth clear of the boundary: `s` across the
    /// section, `t` round the axis, `w` along it (on a ring, round the
    /// tube). Strictly interior for every input in `[0, 1]³`.
    pub fn interior(&self, s: f64, t: f64, w: f64) -> Point3 {
        let turn = core::f64::consts::TAU * t;
        let (sin, cos) = turn.sin_cos();
        let s = 0.9 * s;
        let along = 0.1 + 0.8 * w;
        match *self {
            QuadricSolid::Frustum {
                bottom,
                top,
                height,
            } => {
                let z = along * height;
                let r = s * (bottom + (top - bottom) * along);
                Point3::new(r * cos, r * sin, z)
            }
            QuadricSolid::Ball { radius } => {
                // `w` as the latitude, `s` the fraction of the radius.
                let lat = core::f64::consts::PI * (w - 0.5);
                let r = s * radius;
                Point3::new(r * lat.cos() * cos, r * lat.cos() * sin, r * lat.sin())
            }
            QuadricSolid::Ring { major, minor } => {
                let tube = core::f64::consts::TAU * w;
                let rho = major + s * minor * tube.cos();
                Point3::new(rho * cos, rho * sin, s * minor * tube.sin())
            }
            QuadricSolid::EllipticPrism { a, b, height } => {
                Point3::new(s * a * cos, s * b * sin, along * height)
            }
        }
    }

    /// The radius of the smallest ball about the origin holding the solid.
    pub fn reach(&self) -> f64 {
        match *self {
            QuadricSolid::Frustum {
                bottom,
                top,
                height,
            } => bottom.max(top).hypot(height),
            QuadricSolid::Ball { radius } => radius,
            QuadricSolid::Ring { major, minor } => major + minor,
            QuadricSolid::EllipticPrism { a, b, height } => a.max(b).hypot(height),
        }
    }

    /// The solid's least thickness: what a tool is sized against so it
    /// neither vanishes in the solid nor swallows it.
    pub fn thickness(&self) -> f64 {
        match *self {
            QuadricSolid::Frustum {
                bottom,
                top,
                height,
            } => (2.0 * bottom.min(top)).min(height),
            QuadricSolid::Ball { radius } => 2.0 * radius,
            QuadricSolid::Ring { minor, .. } => 2.0 * minor,
            QuadricSolid::EllipticPrism { a, b, height } => (2.0 * a.min(b)).min(height),
        }
    }

    /// The solid as a body of `m`, before any motion: the frustum, ball
    /// and ring a profile in the `(x, z)` plane revolved a whole turn about
    /// `z`, the prism an ellipse in `z = 0` extruded along `z`.
    pub fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        let p = Point2::new;
        let outer = match *self {
            QuadricSolid::Frustum {
                bottom,
                top,
                height,
            } => ProfileLoop::Path {
                start: p(0.0, 0.0),
                segments: vec![
                    ProfileSegment::LineTo(p(bottom, 0.0)),
                    ProfileSegment::LineTo(p(top, height)),
                    ProfileSegment::LineTo(p(0.0, height)),
                    ProfileSegment::LineTo(p(0.0, 0.0)),
                ],
            },
            QuadricSolid::Ball { radius } => ProfileLoop::Path {
                start: p(0.0, -radius),
                segments: vec![
                    ProfileSegment::ArcTo {
                        to: p(0.0, radius),
                        via: p(radius, 0.0),
                    },
                    ProfileSegment::LineTo(p(0.0, -radius)),
                ],
            },
            QuadricSolid::Ring { major, minor } => ProfileLoop::Circle {
                center: p(major, 0.0),
                radius: minor,
            },
            QuadricSolid::EllipticPrism { a, b, height } => {
                let profile = Profile {
                    plane: Frame::world(),
                    outer: ProfileLoop::Ellipse {
                        center: p(0.0, 0.0),
                        major: arris_math::Vec2::new(a, 0.0),
                        minor_radius: b,
                    },
                    holes: Vec::new(),
                };
                return Ok(extrude(m, &profile, Vec3::z(), height)?.0);
            }
        };
        // The (x, z) plane: the world frame a quarter turn about `x`.
        let plane = Frame::from_rotation(
            Point3::origin(),
            &UnitQuaternion::from_axis_angle(&Vec3::x_axis(), core::f64::consts::FRAC_PI_2),
        );
        let profile = Profile {
            plane,
            outer,
            holes: Vec::new(),
        };
        Ok(revolve(
            m,
            &profile,
            Axis::z_at(Point3::origin()),
            core::f64::consts::TAU,
        )?
        .0)
    }
}

/// The tool a [`QuadricPair`]'s solid is cut with.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum QuadricTool {
    /// A box.
    Box(Boxed),
    /// A cylinder.
    Cylinder(Cylindrical),
}

impl QuadricTool {
    /// The tool as a body of `m`.
    pub fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match self {
            QuadricTool::Box(b) => b.build(m),
            QuadricTool::Cylinder(c) => c.build(m),
        }
    }
}

/// A [`QuadricSolid`] and a box or a cylinder through a point of its
/// interior, both under one motion: the operands of the identities over
/// quadric operands (C3's accept line, docs/ROADMAP.md). The tool's own pose
/// places it in the solid's frame, then `pose`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadricPair {
    /// The solid, before `pose`.
    pub solid: QuadricSolid,
    /// The tool, its pose already `pose` after its own placing.
    pub tool: QuadricTool,
    /// The motion of both.
    pub pose: Isometry,
}

impl QuadricPair {
    /// Both bodies in `m`, the solid first.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        let solid = self.solid.build(m)?;
        let solid = transform(m, solid, &self.pose)?.0;
        Ok((solid, self.tool.build(m)?))
    }
}

/// [`QuadricSolid`]s, a quarter of each kind: a size from `MIN_EXTENT / 2`
/// to `MAX_EXTENT / 2` and the shape's ratios — a frustum's radii apart
/// by a fifth to four fifths, either end the wider, as tall as a fifth to
/// twice its base; a ring's tube a fifth to seven tenths of its major
/// radius; a prism's semi-axes apart by the same fifth to four fifths.
pub fn quadric_solid() -> impl Strategy<Value = QuadricSolid> {
    (
        0u8..4,
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        finite_f64(0.2..=0.8),
        finite_f64(0.2..=2.0),
        any::<bool>(),
    )
        .prop_map(|(kind, size, ratio, tall, flip)| match kind {
            0 => {
                let (bottom, top) = if flip {
                    (size, ratio * size)
                } else {
                    (ratio * size, size)
                };
                QuadricSolid::Frustum {
                    bottom,
                    top,
                    height: tall * 2.0 * size,
                }
            }
            1 => QuadricSolid::Ball { radius: size },
            2 => QuadricSolid::Ring {
                major: size,
                minor: (0.2 + 0.5 * (ratio - 0.2) / 0.6) * size,
            },
            _ => QuadricSolid::EllipticPrism {
                a: size,
                b: ratio * size,
                height: tall * 2.0 * size,
            },
        })
}

/// [`QuadricPair`]s: a solid from [`quadric_solid`], a point of its
/// interior ([`QuadricSolid::interior`]), and a tool through it that
/// neither holds the solid nor is held by it, so every boolean has
/// material. Half the time a box centred there and turned at random: one
/// side a fifth to four fifths of the solid's thickness, which no width of
/// the solid is under, so the box cannot hold it; one two and a half to
/// three times its reach, past its diameter, so the solid cannot hold the
/// box; the third between 1.3 times the thickness and twice the reach — a
/// side of exactly the thickness centred on the solid's middle is tangent
/// to it, and so is one of `1.25` times a frustum's narrow end whose wide
/// end is `1.25` times wider, a pair a shrink runs straight to. Half the
/// time a cylinder whose axis runs through the point in a random
/// direction, of radius 0.15 to 0.4 of the thickness — a tenth, from a
/// point a tenth of the height from a cap, is tangent to it — reaching
/// twice the solid's reach ahead of the point, so past it, and behind it
/// as far again or, a third of the time, ending inside it, so a cap meets
/// the curved face as well. Both under a pose at [`DEFAULT_SCALE`].
pub fn quadric_pair() -> impl Strategy<Value = QuadricPair> {
    (
        quadric_solid(),
        (
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
            finite_f64(0.0..=1.0),
        ),
        any::<bool>(),
        (
            rotation(),
            finite_f64(0.2..=0.8),
            finite_f64(2.5..=3.0),
            finite_f64(0.0..=1.0),
        ),
        (
            unit_vec3(),
            finite_f64(0.15..=0.4),
            0u8..3,
            finite_f64(0.5..=2.0),
        ),
        pose_in(DEFAULT_SCALE),
    )
        .prop_filter_map(
            "a tool through a quadric solid",
            |(solid, (s, t, w), boxed, (turn, ex, ey, ez), (d, rf, long, short), pose)| {
                let at = solid.interior(s, t, w);
                let (thick, reach) = (solid.thickness(), solid.reach());
                let tool = if boxed {
                    let (least, most) = (1.3 * thick, (2.0 * reach).max(1.3 * thick));
                    let half = Vec3::new(ex * thick, ey * reach, least + ez * (most - least)) / 2.0;
                    QuadricTool::Box(Boxed {
                        min: Point3::from(-half),
                        max: Point3::from(half),
                        pose: Isometry::new(turn, at.coords).then(&pose),
                    })
                } else {
                    // Behind the point: past the solid, or a stub ending
                    // inside it. Ahead, always past it — every point of the
                    // solid is within `2 reach` of every other — so the
                    // cylinder is never swallowed whole.
                    let behind = if long == 0 {
                        short * thick / 2.0
                    } else {
                        2.0 * reach
                    };
                    let d = d.into_inner();
                    QuadricTool::Cylinder(Cylindrical {
                        axis: Axis::new(at - behind * d, d).ok()?,
                        radius: rf * thick,
                        height: behind + 2.0 * reach,
                        pose,
                    })
                };
                Some(QuadricPair { solid, tool, pose })
            },
        )
}

/// A designed contact the corpus holds, which a [`BandPair`] moves off
/// by a fraction of the tolerance to a few tolerances (ADR-0022).
/// Each names the fixtures it stands for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum BandContact {
    /// Two boxes sharing a whole face, their side faces coplanar:
    /// `boolean/flush-union`.
    FlushBoxes,
    /// A cylinder standing on a box's face, its cap flush with it:
    /// `boolean/boss-flush`.
    FlushBoss,
    /// A pin inside a bore, the walls touching along a ruling from
    /// inside: `boolean/pin-in-bore-*`.
    PinInBore,
    /// A rod filling a tube's bore, walls coincident and caps flush:
    /// `boolean/coaxial-fuse`.
    CoaxialRod,
    /// A bore through a cylinder on its axis: `boolean/coaxial-cut`.
    CoaxialBore,
    /// Two cylinders on parallel axes whose walls touch from outside:
    /// `boolean/tangent-cylinders-*`.
    TangentCylinders,
    /// A blind hole whose wall touches a side face from inside:
    /// `boolean/tangent-hole`.
    TangentHole,
    /// A cylinder whose wall touches a box face from outside:
    /// `boolean/tangent-outside-cut`.
    TangentOutside,
    /// A quarter bend and the pipe it runs into, caps flush and the tube
    /// circle shared: `boolean/pipe-elbow-fuse`.
    PipeElbow,
    /// Two boxes touching along an edge: `boolean/edge-touching-fuse`.
    EdgeOnEdge,
}

impl BandContact {
    /// Every contact, in the order [`band_pair`] draws them.
    pub const ALL: [BandContact; 10] = [
        BandContact::FlushBoxes,
        BandContact::FlushBoss,
        BandContact::PinInBore,
        BandContact::CoaxialRod,
        BandContact::CoaxialBore,
        BandContact::TangentCylinders,
        BandContact::TangentHole,
        BandContact::TangentOutside,
        BandContact::PipeElbow,
        BandContact::EdgeOnEdge,
    ];
}

/// How a [`BandPair`]'s moving operand leaves the contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum BandMotion {
    /// Moved along the contact's normal: a positive offset into the
    /// fixed operand, a negative one away.
    Offset,
    /// Turned about a line through the contact's middle, so each end of
    /// the contact moves by the offset, one into the fixed operand and
    /// one away.
    Tilt,
    /// Its cylinder's radius grown by the offset, the axis kept.
    Radius,
}

/// A solid of a [`BandPair`], before any motion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BandSolid {
    /// An axis-aligned box.
    Block {
        /// One corner.
        min: Point3,
        /// The opposite one.
        max: Point3,
    },
    /// A cylinder, its axis's origin the base cap's centre.
    Rod {
        /// The axis.
        axis: Axis,
        /// The radius.
        radius: f64,
        /// The height.
        height: f64,
    },
    /// A cylinder less a coaxial bore that runs past both its caps, built
    /// by a `cut`, as `boolean/coaxial-fuse` builds its tube.
    Tube {
        /// The axis.
        axis: Axis,
        /// The outer radius.
        outer: f64,
        /// The bore's radius.
        bore: f64,
        /// The height.
        height: f64,
    },
    /// A disc of radius `minor` about `(major, 0, 0)` in the plane
    /// `y = 0` revolved a quarter turn about `z`, toward `+y`.
    Bend {
        /// The distance from `z` to the tube's centre circle.
        major: f64,
        /// The tube's radius.
        minor: f64,
    },
}

impl BandSolid {
    /// The solid as a body of `m`, moved by `pose`.
    pub fn build(&self, m: &mut Model, pose: &Isometry) -> Result<Body, OpError> {
        let body = match *self {
            BandSolid::Block { min, max } => primitive_box(m, min, max)?.0,
            BandSolid::Rod {
                axis,
                radius,
                height,
            } => primitive_cylinder(m, axis, radius, height)?.0,
            BandSolid::Tube {
                axis,
                outer,
                bore,
                height,
            } => {
                let (wall, _) = primitive_cylinder(m, axis, outer, height)?;
                let past = Axis {
                    origin: axis.at(-1.0),
                    direction: axis.direction,
                };
                let (hole, _) = primitive_cylinder(m, past, bore, height + 2.0)?;
                arris_ops::cut(m, wall, hole, &arris_math::Control::NONE)?.0
            }
            BandSolid::Bend { major, minor } => {
                let plane = Frame::from_rotation(
                    Point3::origin(),
                    &UnitQuaternion::from_axis_angle(&Vec3::x_axis(), core::f64::consts::FRAC_PI_2),
                );
                let profile = Profile {
                    plane,
                    outer: ProfileLoop::Circle {
                        center: Point2::new(major, 0.0),
                        radius: minor,
                    },
                    holes: Vec::new(),
                };
                revolve(
                    m,
                    &profile,
                    Axis::z_at(Point3::origin()),
                    core::f64::consts::FRAC_PI_2,
                )?
                .0
            }
        };
        Ok(transform(m, body, pose)?.0)
    }

    /// The same solid with its cylinder's radius grown by `by`: a rod's
    /// radius, a tube's bore, a bend's tube. A block has none and is
    /// itself.
    pub fn grown(&self, by: f64) -> BandSolid {
        match *self {
            BandSolid::Block { .. } => *self,
            BandSolid::Rod {
                axis,
                radius,
                height,
            } => BandSolid::Rod {
                axis,
                radius: radius + by,
                height,
            },
            BandSolid::Tube {
                axis,
                outer,
                bore,
                height,
            } => BandSolid::Tube {
                axis,
                outer,
                bore: bore + by,
                height,
            },
            BandSolid::Bend { major, minor } => BandSolid::Bend {
                major,
                minor: minor + by,
            },
        }
    }
}

/// A designed contact of the corpus, in its own frame, and a way off it:
/// the operands of the tolerance band (ADR-0022). The
/// `fixed` solid stays where it is; the `moving` one is moved off the
/// contact by [`BandPair::build`]'s offset — a multiple of the tolerance
/// — along `normal`, turned about `hinge`, or grown, as `motion` says.
/// Both are then placed by `pose`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BandPair {
    /// The contact.
    pub contact: BandContact,
    /// How the moving solid leaves it.
    pub motion: BandMotion,
    /// The solid that stays.
    pub fixed: BandSolid,
    /// The solid that moves, at the contact.
    pub moving: BandSolid,
    /// The unit direction a positive offset moves `moving` in: into
    /// `fixed`.
    pub normal: Vec3,
    /// The middle of the contact, which a tilt turns about.
    pub pivot: Point3,
    /// The unit direction a tilt turns about, through `pivot`.
    pub hinge: Vec3,
    /// The contact's extent across the hinge: a tilt by `δ` turns by
    /// `2δ / lever`, so each end of the contact moves by `δ`.
    pub lever: f64,
    /// The area of the contact, zero where it is a line or a point.
    pub area: f64,
    /// Where the contact is a line: its length and the radius of the
    /// relative curvature across it — zero for an edge on an edge.
    pub line: Option<(f64, f64)>,
    /// The motion of both.
    pub pose: Isometry,
}

impl BandPair {
    /// Both solids in `m`, `fixed` first, the moving one off the contact
    /// by `delta` (a length, positive into `fixed`).
    pub fn build(&self, m: &mut Model, delta: f64) -> Result<(Body, Body), OpError> {
        let a = self.fixed.build(m, &self.pose)?;
        let (solid, off) = self.moved(delta);
        let b = solid.build(m, &off.then(&self.pose))?;
        Ok((a, b))
    }

    /// The moving solid and the motion taking it off the contact by
    /// `delta`, before `pose`.
    pub fn moved(&self, delta: f64) -> (BandSolid, Isometry) {
        match self.motion {
            BandMotion::Offset => (
                self.moving,
                Isometry::new(UnitQuaternion::identity(), delta * self.normal),
            ),
            BandMotion::Tilt => {
                let turn = UnitQuaternion::from_axis_angle(
                    &UnitVec3::new_normalize(self.hinge),
                    2.0 * delta / self.lever,
                );
                let p = self.pivot.coords;
                (self.moving, Isometry::new(turn, p - turn * p))
            }
            BandMotion::Radius => (self.moving.grown(delta), Isometry::identity()),
        }
    }

    /// A bound on how far the volume of any boolean of the pair moves
    /// when the moving solid is `delta` off the contact: the contact's
    /// area times the offset, and along a line contact the strip the
    /// offset opens across it, `√(8ρδ)` wide, times the offset — plus,
    /// on a line of no curvature (an edge on an edge), the offset squared
    /// along it.
    pub fn volume_bound(&self, delta: f64) -> f64 {
        let d = delta.abs();
        let line = self.line.map_or(0.0, |(len, rho)| {
            len * d * (8.0 * rho * d).sqrt() + len * d * d
        });
        self.area * d + line
    }
}

/// The multiples of the tolerance a band property moves each
/// [`BandPair`] off its contact by, both signs of each: a fraction of it
/// to several (ADR-0022).
pub const BAND_STEPS: [f64; 7] = [0.25, 0.5, 1.0, 1.5, 2.0, 4.0, 16.0];

/// [`BandPair`]s: a contact from [`BandContact`] a tenth each, its sizes
/// drawn in `[MIN_EXTENT, MAX_EXTENT]` (radii half that) with every
/// feature a tenth of its room clear of the next, a motion among those
/// that move it — a radius only where the moving solid has one that
/// reaches the contact — and a pose at [`DEFAULT_SCALE`]. Each is its
/// fixture's contact exactly at an offset of zero, with the fixture's
/// numbers free.
pub fn band_pair() -> impl Strategy<Value = BandPair> {
    let unit = || finite_f64(0.0..=1.0);
    (
        0u8..10,
        0u8..3,
        [unit(), unit(), unit(), unit(), unit(), unit()],
        pose_in(DEFAULT_SCALE),
    )
        .prop_filter_map("a contact moved off", |(kind, motion, f, pose)| {
            band_pair_from(
                kind,
                |motions| motions.get(usize::from(motion) % motions.len()).copied(),
                f,
                pose,
            )
        })
}

/// [`band_pair`]'s pairs of the given contacts under the given motions
/// only, a cell of `cells` each in turn: the band a property holds where
/// it holds, without drawing and rejecting the rest. A cell whose motion
/// does not move its contact is never drawn.
pub fn band_pair_of(
    cells: &'static [(BandContact, BandMotion)],
) -> impl Strategy<Value = BandPair> {
    let unit = || finite_f64(0.0..=1.0);
    (
        0..cells.len(),
        [unit(), unit(), unit(), unit(), unit(), unit()],
        pose_in(DEFAULT_SCALE),
    )
        .prop_filter_map("a cell moved off", move |(i, f, pose)| {
            let (contact, motion) = *cells.get(i)?;
            let kind = BandContact::ALL.iter().position(|c| *c == contact)?;
            band_pair_from(
                u8::try_from(kind).ok()?,
                |motions| motions.contains(&motion).then_some(motion),
                f,
                pose,
            )
        })
}

/// The pair of contact `kind` (its index in [`BandContact::ALL`]) with
/// the sizes `f` and the pose, under the motion `pick` chooses among the
/// ones that move it.
fn band_pair_from(
    kind: u8,
    pick: impl Fn(&[BandMotion]) -> Option<BandMotion>,
    f: [f64; 6],
    pose: Isometry,
) -> Option<BandPair> {
    let e = |x: f64| MIN_EXTENT + x * (MAX_EXTENT - MIN_EXTENT);
    let r = |x: f64| e(x) / 2.0;
    let within = |x: f64| 0.1 + 0.8 * x;
    let p = Point3::new;
    let x = Vec3::x();
    let y = Vec3::y();
    let z = Vec3::z();
    let pi = core::f64::consts::PI;
    let block = |min: Point3, max: Point3| BandSolid::Block { min, max };
    let rod = |at: Point3, d: Vec3, radius: f64, height: f64| {
        Some(BandSolid::Rod {
            axis: Axis::new(at, d).ok()?,
            radius,
            height,
        })
    };
    let all = [BandMotion::Offset, BandMotion::Tilt, BandMotion::Radius];
    let rigid = [BandMotion::Offset, BandMotion::Tilt];
    let (contact, motions, fixed, moving, normal, pivot, hinge, lever, area, line) = match kind {
        0 => {
            let (ax, ay, az, bx) = (e(f[0]), e(f[1]), e(f[2]), e(f[3]));
            (
                BandContact::FlushBoxes,
                &rigid[..],
                block(p(0.0, 0.0, 0.0), p(ax, ay, az)),
                block(p(ax, 0.0, 0.0), p(ax + bx, ay, az)),
                -x,
                p(ax, ay / 2.0, az / 2.0),
                y,
                az,
                ay * az,
                None,
            )
        }
        1 => {
            let (px, py, pz) = (e(f[0]), e(f[1]), e(f[2]));
            let radius = px.min(py) / 2.0 * within(f[3]);
            let (cx, cy) = (
                radius + (px - 2.0 * radius) * within(f[4]),
                radius + (py - 2.0 * radius) * within(f[5]),
            );
            (
                BandContact::FlushBoss,
                &rigid[..],
                block(p(0.0, 0.0, 0.0), p(px, py, pz)),
                rod(p(cx, cy, pz), z, radius, e(f[3]))?,
                -z,
                p(cx, cy, pz),
                x,
                2.0 * radius,
                pi * radius * radius,
                None,
            )
        }
        2 => {
            let big = r(f[0]);
            let small = big * within(f[1]);
            let tall = e(f[2]);
            let h = tall * within(f[3]);
            let z0 = (tall - h) * within(f[4]);
            (
                BandContact::PinInBore,
                &all[..],
                rod(p(0.0, 0.0, 0.0), z, big, tall)?,
                rod(p(0.0, big - small, z0), z, small, h)?,
                y,
                p(0.0, big, z0 + h / 2.0),
                x,
                h,
                0.0,
                Some((h, small * big / (big - small))),
            )
        }
        3 => {
            let outer = r(f[0]);
            let bore = outer * within(f[1]);
            let tall = e(f[2]);
            let phi = core::f64::consts::TAU * f[3];
            let (s, c) = phi.sin_cos();
            (
                BandContact::CoaxialRod,
                &all[..],
                BandSolid::Tube {
                    axis: Axis::z_at(p(0.0, 0.0, 0.0)),
                    outer,
                    bore,
                    height: tall,
                },
                rod(p(0.0, 0.0, 0.0), z, bore, tall)?,
                Vec3::new(c, s, 0.0),
                p(0.0, 0.0, tall / 2.0),
                Vec3::new(-s, c, 0.0),
                tall,
                2.0 * pi * bore * tall,
                None,
            )
        }
        4 => {
            let outer = r(f[0]);
            let bore = outer * within(f[1]);
            let tall = e(f[2]);
            let phi = core::f64::consts::TAU * f[3];
            let (s, c) = phi.sin_cos();
            (
                BandContact::CoaxialBore,
                &rigid[..],
                rod(p(0.0, 0.0, 0.0), z, outer, tall)?,
                rod(p(0.0, 0.0, -tall / 2.0), z, bore, 2.0 * tall)?,
                Vec3::new(c, s, 0.0),
                p(0.0, 0.0, tall / 2.0),
                Vec3::new(-s, c, 0.0),
                tall,
                0.0,
                None,
            )
        }
        5 => {
            let (big, other, tall) = (r(f[0]), r(f[1]), e(f[2]));
            let past = tall * within(f[3]) / 2.0;
            (
                BandContact::TangentCylinders,
                &all[..],
                rod(p(0.0, 0.0, 0.0), z, big, tall)?,
                rod(p(0.0, big + other, -past), z, other, tall + 2.0 * past)?,
                -y,
                p(0.0, big, tall / 2.0),
                x,
                tall,
                0.0,
                Some((tall, big * other / (big + other))),
            )
        }
        6 => {
            let (px, py, pz) = (e(f[0]), e(f[1]), e(f[2]));
            let radius = (px / 2.0).min(py / 2.0) * within(f[3]);
            let cy = radius + (py - 2.0 * radius) * within(f[4]);
            let z0 = pz * within(f[5]);
            (
                BandContact::TangentHole,
                &all[..],
                block(p(0.0, 0.0, 0.0), p(px, py, pz)),
                rod(p(radius, cy, z0), z, radius, 2.0 * pz - z0)?,
                -x,
                p(0.0, cy, (z0 + pz) / 2.0),
                y,
                pz - z0,
                0.0,
                Some((pz - z0, radius)),
            )
        }
        7 => {
            let (px, py, pz, radius) = (e(f[0]), e(f[1]), e(f[2]), r(f[3]));
            let cy = py * within(f[4]);
            // Past the bottom cap, or ending inside the face;
            // past the top always.
            let z0 = if f[5] < 0.5 {
                -pz * within(2.0 * f[5]) / 2.0
            } else {
                pz * within(2.0 * f[5] - 1.0)
            };
            let low = z0.max(0.0);
            (
                BandContact::TangentOutside,
                &all[..],
                block(p(0.0, 0.0, 0.0), p(px, py, pz)),
                rod(p(px + radius, cy, z0), z, radius, 1.5 * pz - z0)?,
                -x,
                p(px, cy, (low + pz) / 2.0),
                y,
                pz - low,
                0.0,
                Some((pz - low, radius)),
            )
        }
        8 => {
            let major = r(f[0]);
            let minor = major * within(f[1]);
            (
                BandContact::PipeElbow,
                &all[..],
                BandSolid::Bend { major, minor },
                rod(p(major, 0.0, 0.0), -y, minor, e(f[2]))?,
                y,
                p(major, 0.0, 0.0),
                z,
                2.0 * minor,
                pi * minor * minor,
                None,
            )
        }
        _ => {
            let (ax, ay, az) = (e(f[0]), e(f[1]), e(f[2]));
            let (bx, by) = (e(f[3]), e(f[4]));
            let s = core::f64::consts::FRAC_1_SQRT_2;
            (
                BandContact::EdgeOnEdge,
                &rigid[..],
                block(p(0.0, 0.0, 0.0), p(ax, ay, az)),
                block(p(ax, ay, 0.0), p(ax + bx, ay + by, az)),
                Vec3::new(-s, -s, 0.0),
                p(ax, ay, az / 2.0),
                Vec3::new(s, -s, 0.0),
                az,
                0.0,
                Some((az, 0.0)),
            )
        }
    };
    Some(BandPair {
        contact,
        motion: pick(motions)?,
        fixed,
        moving,
        normal,
        pivot,
        hinge,
        lever,
        area,
        line,
        pose,
    })
}

/// Box extents, each in `[MIN_EXTENT, MAX_EXTENT]`.
fn extents() -> impl Strategy<Value = Vec3> {
    (
        finite_f64(MIN_EXTENT..=MAX_EXTENT),
        finite_f64(MIN_EXTENT..=MAX_EXTENT),
        finite_f64(MIN_EXTENT..=MAX_EXTENT),
    )
        .prop_map(|(x, y, z)| Vec3::new(x, y, z))
}

/// Boxes of random extents centred at the origin, under a rigid motion
/// whose translation is uniform in the box of half-width `scale`.
pub fn box_in(scale: f64) -> impl Strategy<Value = Boxed> {
    (extents(), pose_in(scale)).prop_map(|(e, pose)| Boxed {
        min: Point3::from(-e / 2.0),
        max: Point3::from(e / 2.0),
        pose,
    })
}

/// Cylinders of random radius and height on the `z` axis, centred at
/// the origin, under a rigid motion as [`box_in`]'s.
pub fn cylinder_in(scale: f64) -> impl Strategy<Value = Cylindrical> {
    (
        radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
        radius(MIN_EXTENT..=MAX_EXTENT),
        pose_in(scale),
    )
        .prop_map(|(r, h, pose)| Cylindrical {
            axis: Axis::z_at(Point3::new(0.0, 0.0, -h / 2.0)),
            radius: r,
            height: h,
            pose,
        })
}

/// [`BarCut`]s: a box from [`box_in`], one or two bars thin along a
/// random axis of its own frame, and a second set of bars drawn from the
/// same bands — bar 0's centre in `[0.15, 0.40]` of the extent and bar
/// 1's in `[0.60, 0.85]`, each half-width at most `0.10`, so every bar
/// stays clear of the box's ends and of its neighbour in both builds.
pub fn bar_cut() -> impl Strategy<Value = BarCut> {
    let bar = |centre: RangeInclusive<f64>| (finite_f64(centre), finite_f64(0.02..=0.10));
    let build = || (bar(0.15..=0.40), bar(0.60..=0.85));
    let resize = || {
        (
            finite_f64(0.6..=1.6),
            finite_f64(0.6..=1.6),
            finite_f64(0.6..=1.6),
        )
    };
    (
        box_in(DEFAULT_SCALE),
        0usize..3,
        1usize..=2,
        build(),
        build(),
        (resize(), pose_in(DEFAULT_SCALE)),
    )
        .prop_map(
            |(cuboid, axis, count, (a0, a1), (b0, b1), ((sx, sy, sz), pose))| {
                let e = cuboid.max - cuboid.min;
                let scaled = Vec3::new(e.x * sx, e.y * sy, e.z * sz);
                BarCut {
                    cuboid,
                    rebuilt: Boxed {
                        min: Point3::from(-scaled / 2.0),
                        max: Point3::from(scaled / 2.0),
                        pose,
                    },
                    axis,
                    bars: vec![a0, a1][..count].to_vec(),
                    moved: vec![b0, b1][..count].to_vec(),
                }
            },
        )
}

/// [`OverlappingPair`]s: a box from [`box_in`], a cylinder whose axis
/// runs through a random interior point of the box in a random
/// direction, three diagonals long, with a radius between a tenth and
/// six tenths of the box's smallest side, both under the box's motion at
/// [`DEFAULT_SCALE`].
pub fn overlapping_pair() -> impl Strategy<Value = OverlappingPair> {
    (
        box_in(DEFAULT_SCALE),
        (
            finite_f64(0.1..=0.9),
            finite_f64(0.1..=0.9),
            finite_f64(0.1..=0.9),
        ),
        unit_vec3(),
        finite_f64(0.1..=0.6),
    )
        .prop_filter_map("a cylinder axis", |(cuboid, (fx, fy, fz), d, rf)| {
            let e = cuboid.max - cuboid.min;
            let through = cuboid.min + Vec3::new(fx * e.x, fy * e.y, fz * e.z);
            let height = 3.0 * e.norm();
            let base = through - (height / 2.0) * d.into_inner();
            let axis = Axis::new(base, UnitVec3::into_inner(d)).ok()?;
            Some(OverlappingPair {
                cuboid,
                cylinder: Cylindrical {
                    axis,
                    radius: rf * e.x.min(e.y).min(e.z),
                    height,
                    pose: cuboid.pose,
                },
            })
        })
}

/// [`overlapping_pair`]s whose cylinder [`OverlappingPair::pierces`] the
/// box — clears every edge — so the outcome of every boolean is known by
/// construction: `fuse` and `common` are one shell, `box − cylinder` is
/// one shell and `cylinder − box` is two. The pairs that fail the test
/// (the wall crossing an edge, slicing a corner off, or fat enough that
/// the cylinder's two ends join around the box) are a large share of
/// [`overlapping_pair`] and stay in it.
pub fn piercing_pair() -> impl Strategy<Value = OverlappingPair> {
    overlapping_pair().prop_filter(
        "the cylinder clears every edge of the box",
        OverlappingPair::pierces,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prop::check;
    use arris_check::{Level, check as check_body};
    use arris_math::nalgebra::UnitQuaternion;

    /// `pierces` is the wall clearing every edge: the through-hole does,
    /// the same hole widened to reach the long edges does not, and a post
    /// standing across a side face (`boolean/sliver-common`) does not.
    #[test]
    fn pierces_is_the_wall_clearing_every_edge() {
        let still = Isometry::new(UnitQuaternion::identity(), Vec3::zeros());
        let plate = Boxed {
            min: Point3::origin(),
            max: Point3::new(40.0, 30.0, 10.0),
            pose: still,
        };
        let post = |x: f64, y: f64, radius: f64| OverlappingPair {
            cuboid: plate,
            cylinder: Cylindrical {
                axis: Axis::z_at(Point3::new(x, y, -1.0)),
                radius,
                height: 12.0,
                pose: still,
            },
        };
        assert!(post(20.0, 15.0, 4.0).pierces());
        // The edges y = 0 and y = 30 are 15 away from the axis.
        assert!(post(20.0, 15.0, 14.5).pierces());
        assert!(!post(20.0, 15.0, 15.5).pierces());
        assert!(!post(43.8, 15.0, 4.0).pierces());
        // Along an edge: the axis on the edge x = 0, y = 0 itself.
        assert!(!post(0.0, 0.0, 1.0).pierces());
    }

    /// The tangent pairs touch: the axis is one radius outside the face's
    /// plane, the foot of the ruling is inside the face, and the bodies
    /// are clean.
    #[test]
    fn the_tangent_pairs_touch_one_face_from_outside() {
        check(tangent_pair(), |pair| {
            let mut m = Model::default();
            let (a, b) = pair
                .build(&mut m)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(check_body(&m, a, Level::Fast).is_ok());
            prop_assert!(check_body(&m, b, Level::Fast).is_ok());
            let (lo, hi) = (pair.cuboid.min, pair.cuboid.max);
            let (i, max_side) = pair.face;
            let plane = if max_side { hi[i] } else { lo[i] };
            let mid = pair.cylinder.axis.at(pair.cylinder.height / 2.0);
            let n = pair.normal();
            prop_assert!(((mid[i] - plane) * n[i] - pair.cylinder.radius).abs() < 1e-12);
            prop_assert!(pair.cylinder.axis.direction.dot(&n).abs() < 1e-12);
            let foot = pair.foot();
            prop_assert!((foot[i] - plane).abs() < 1e-12);
            prop_assert!(
                (0..3)
                    .filter(|&c| c != i)
                    .all(|c| lo[c] < foot[c] && foot[c] < hi[c])
            );
            Ok(())
        });
    }

    /// The parallel pairs cross: the axes `distance` apart, strictly
    /// between the tangent distances; a seam pair's seam on `b`'s wall; a
    /// flush pair's caps level with each other and every other `b` past
    /// both of `a`'s caps; the bodies are clean.
    #[test]
    fn the_parallel_pairs_cross_in_two_rulings() {
        check(parallel_pair(), |pair| {
            let mut m = Model::default();
            let (a, b) = pair
                .build(&mut m)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(check_body(&m, a, Level::Fast).is_ok());
            prop_assert!(check_body(&m, b, Level::Fast).is_ok());
            let (r1, r2, d) = (pair.a.radius, pair.b.radius, pair.distance);
            prop_assert!((r1 - r2).abs() < d && d < r1 + r2);
            let offset = pair.b.axis.origin - pair.a.axis.origin;
            let across = Vec3::new(offset.x, offset.y, 0.0);
            prop_assert!((across.norm() - d).abs() < 1e-12 * MAX_EXTENT);
            let (za, zb) = (pair.a.axis.origin.z, pair.b.axis.origin.z);
            let (ha, hb) = (pair.a.height, pair.b.height);
            match pair.kind {
                ParallelKind::SeamOnRuling => {
                    let seam = Vec3::new(r1, 0.0, 0.0);
                    prop_assert!(((seam - across).norm() - r2).abs() < 1e-12 * MAX_EXTENT);
                }
                ParallelKind::FlushCaps => {
                    prop_assert!((za - zb).abs() < 1e-12 * MAX_EXTENT && ha == hb);
                }
                ParallelKind::Clear => {}
            }
            if pair.kind != ParallelKind::FlushCaps {
                prop_assert!(zb < za && zb + hb > za + ha);
            }
            Ok(())
        });
    }

    /// The crossing pairs pass through each other: equal radii, the axes
    /// at `psi` through both midpoints, each cap's centre more than two
    /// radii from the other axis, and a seam pair's turned seam along `y`,
    /// through the crossing vertex; the bodies are clean.
    #[test]
    fn the_crossing_pairs_pass_through_each_other() {
        check(crossing_pair(), |pair| {
            let mut m = Model::default();
            let (a, b) = pair
                .build(&mut m)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(check_body(&m, a, Level::Fast).is_ok());
            prop_assert!(check_body(&m, b, Level::Fast).is_ok());
            let r = pair.a.radius;
            prop_assert_eq!(pair.b.radius, r);
            let cos = pair.a.axis.direction.dot(&pair.b.axis.direction);
            prop_assert!((cos - pair.psi.cos()).abs() < 1e-12);
            for c in [pair.a, pair.b] {
                prop_assert!(c.axis.at(c.height / 2.0).coords.norm() < 1e-12 * MAX_EXTENT);
                prop_assert!(c.height / 2.0 * pair.psi.sin() > 2.0 * r);
            }
            if pair.seam_through_crossing {
                let turn = pair.b.pose.then(&pair.a.pose.inverse());
                let frame = Frame::from_z(pair.b.axis.origin, pair.b.axis.direction.into_inner())
                    .map_err(|e| TestCaseError::fail(e.to_string()))?;
                let seam = turn.apply_vec(frame.x().into_inner());
                prop_assert!((seam.y.abs() - 1.0).abs() < 1e-12);
            }
            Ok(())
        });
    }

    /// The quadric pairs build clean bodies, and the tool neither holds
    /// the solid nor is held by it: a box has a side under the solid's
    /// thickness and one over its diameter, a cylinder is thinner than the
    /// solid and reaches past it ahead of the point it runs through.
    #[test]
    fn the_quadric_pairs_build_clean_bodies_neither_holding_the_other() {
        check(quadric_pair(), |pair| {
            let mut m = Model::default();
            let (a, b) = pair
                .build(&mut m)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(check_body(&m, a, Level::Fast).is_ok());
            prop_assert!(check_body(&m, b, Level::Fast).is_ok());
            let (thick, reach) = (pair.solid.thickness(), pair.solid.reach());
            match pair.tool {
                QuadricTool::Box(boxed) => {
                    let e = boxed.max - boxed.min;
                    prop_assert!(e.min() < thick && e.max() > 2.0 * reach);
                }
                QuadricTool::Cylinder(c) => {
                    prop_assert!(2.0 * c.radius < thick);
                    let ahead = c.axis.at(c.height);
                    prop_assert!(ahead.coords.norm() > reach);
                }
            }
            Ok(())
        });
    }

    /// The band pairs build clean operands at the contact and at the
    /// band's widest step either way, and the normal and the hinge are
    /// unit and square to each other.
    #[test]
    fn the_band_pairs_build_clean_bodies_across_the_band() {
        check(band_pair(), |pair| {
            let tol = Model::default().precision().default_tolerance;
            for step in [0.0, -BAND_STEPS[6], BAND_STEPS[6]] {
                let mut m = Model::default();
                let (a, b) = pair
                    .build(&mut m, step * tol)
                    .map_err(|e| TestCaseError::fail(e.to_string()))?;
                prop_assert!(check_body(&m, a, Level::Fast).is_ok());
                prop_assert!(check_body(&m, b, Level::Fast).is_ok());
            }
            prop_assert!((pair.normal.norm() - 1.0).abs() < 1e-12);
            prop_assert!((pair.hinge.norm() - 1.0).abs() < 1e-12);
            prop_assert!(pair.normal.dot(&pair.hinge).abs() < 1e-12);
            prop_assert!(pair.lever > 0.0 && pair.area >= 0.0);
            Ok(())
        });
    }

    #[test]
    fn the_pairs_build_clean_bodies_with_the_axis_through_the_box() {
        check(overlapping_pair(), |pair| {
            let mut m = Model::default();
            let (a, b) = pair
                .build(&mut m)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(check_body(&m, a, Level::Fast).is_ok());
            prop_assert!(check_body(&m, b, Level::Fast).is_ok());
            // The axis's midpoint is the box's interior point, before
            // and after the motion.
            let mid = pair.cylinder.axis.at(pair.cylinder.height / 2.0);
            let (lo, hi) = (pair.cuboid.min, pair.cuboid.max);
            prop_assert!((0..3).all(|i| lo[i] < mid[i] && mid[i] < hi[i]));
            Ok(())
        });
    }
}
