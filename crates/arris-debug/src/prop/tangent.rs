//! Tangent solids in random poses (the retired tangency-faults plan): three families
//! whose walls touch along a line, each with a closed form for every
//! boolean. A strategy yields a description (numbers and one rigid
//! motion) that `build` turns into bodies of a model, so a failing case
//! prints as numbers a fixture can be written from.
//!
//! - [`TangentInside`]: a cylinder inside a larger one, the two walls
//!   touching along one ruling, their heights free (the inner may stand
//!   out of either end of the outer).
//! - [`BarBosses`]: a bar with a vertical boss at each of its ends (one or
//!   two), each boss's wall tangent to the end face from inside and free to
//!   stand out past the bar's long sides.
//! - [`BoreColumn`]: a box bored through along `z` by a cylinder, and a
//!   square column through the same place whose sides touch the bore along
//!   four lines.

use crate::unmetered::{primitive_box, primitive_cylinder, transform};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::real;
use arris_math::{Axis, Isometry, Point3, Vec3};
use arris_ops::OpError;
use arris_topo::{Body, Model};
use proptest::prelude::*;

use super::{finite_f64, pose, radius};

/// A cylinder on the `z` axis shifted to `(x, y)`, moved by `pose`.
fn upright(
    m: &mut Model,
    at: (f64, f64),
    radius: f64,
    z: (f64, f64),
    pose: &Isometry,
) -> Result<Body, OpError> {
    let axis = Axis::z_at(Point3::new(at.0, at.1, z.0));
    let (body, _) = primitive_cylinder(m, axis, radius, z.1 - z.0)?;
    Ok(transform(m, body, pose)?.0)
}

const SQRT_2: f64 = core::f64::consts::SQRT_2;

/// The length two intervals share (negative when they are apart).
pub fn overlap(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.1.min(b.1) - a.0.max(b.0)
}

/// The area of the part of a disc of radius `r` centred on the origin that
/// lies at `y ≤ h`; `h` past either end of the disc gives all of it or
/// nothing.
pub fn disc_below(r: f64, h: f64) -> f64 {
    if h <= -r {
        0.0
    } else if h >= r {
        core::f64::consts::PI * r * r
    } else {
        r * r * real::acos(-h / r) + h * (r * r - h * h).sqrt()
    }
}

/// A cylinder inside a larger one whose wall it touches along one ruling:
/// both on `z`-parallel axes, the inner one `outer.radius − inner.radius`
/// away in the direction `angle` about the axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TangentInside {
    /// The outer cylinder's radius.
    pub outer: f64,
    /// The inner cylinder's radius, less than the outer's.
    pub inner: f64,
    /// Where the inner axis lies about the outer's, in radians.
    pub angle: f64,
    /// The outer cylinder's `z` extent.
    pub outer_z: (f64, f64),
    /// The inner cylinder's `z` extent; it shares at least a quarter unit
    /// with the outer's.
    pub inner_z: (f64, f64),
    /// The motion both share.
    pub pose: Isometry,
}

impl TangentInside {
    /// The outer cylinder, then the inner one, in `m`.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body), OpError> {
        let d = self.outer - self.inner;
        let (x, y) = (d * real::cos(self.angle), d * real::sin(self.angle));
        let outer = upright(m, (0.0, 0.0), self.outer, self.outer_z, &self.pose)?;
        let inner = upright(m, (x, y), self.inner, self.inner_z, &self.pose)?;
        Ok((outer, inner))
    }

    /// The volume of the outer cylinder, of the inner one and of what they
    /// share: the inner's disc lies inside the outer's, so it is the
    /// inner's cross section over the `z` the two have in common.
    pub fn volumes(&self) -> (f64, f64, f64) {
        let pi = core::f64::consts::PI;
        let height = |z: (f64, f64)| z.1 - z.0;
        (
            pi * self.outer * self.outer * height(self.outer_z),
            pi * self.inner * self.inner * height(self.inner_z),
            pi * self.inner * self.inner * overlap(self.outer_z, self.inner_z),
        )
    }
}

/// [`TangentInside`] draws: radii from 1.5 to 8 with the inner a fifth to
/// nine tenths of the outer, a random pose, and heights of two kinds, half
/// each: free ones of 1 to 10 that overlap, and an inner cylinder that
/// reaches past both ends of the outer by 0 (a coplanar cap) or a quarter
/// to four units.
pub fn tangent_inside() -> impl Strategy<Value = TangentInside> {
    let free = (
        (finite_f64(-5.0..=0.0), finite_f64(1.0..=10.0)),
        (finite_f64(-6.0..=4.0), finite_f64(1.0..=10.0)),
    )
        .prop_filter_map(
            "the cylinders share some height",
            |((lo0, h0), (lo1, h1))| {
                let (outer_z, inner_z) = ((lo0, lo0 + h0), (lo1, lo1 + h1));
                (overlap(outer_z, inner_z) >= 0.25).then_some((outer_z, inner_z))
            },
        );
    let reach = || prop_oneof![Just(0.0), finite_f64(0.25..=4.0)];
    let spanning = (
        (finite_f64(-5.0..=0.0), finite_f64(1.0..=10.0)),
        reach(),
        reach(),
    )
        .prop_map(|((lo, h), below, above)| ((lo, lo + h), (lo - below, lo + h + above)));
    (
        radius(1.5..=8.0),
        finite_f64(0.2..=0.9),
        finite_f64(0.0..=core::f64::consts::TAU),
        prop_oneof![free, spanning],
        pose(),
    )
        .prop_map(
            |(outer, ratio, angle, (outer_z, inner_z), pose)| TangentInside {
                outer,
                inner: outer * ratio,
                angle,
                outer_z,
                inner_z,
                pose,
            },
        )
}

/// One boss of a [`BarBosses`]: a vertical cylinder tangent to the bar's
/// end face from inside.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Boss {
    /// The radius.
    pub radius: f64,
    /// Where its axis lies across the bar's width, within `±half_width`.
    pub across: f64,
    /// Its `z` extent; it shares at least a quarter unit with the bar's.
    pub z: (f64, f64),
}

/// A bar `[−half_length, half_length] × [−half_width, half_width] ×
/// [−half_thickness, half_thickness]` with a boss tangent to its end
/// `x = half_length` and, when `second` is set, another tangent to
/// `x = −half_length`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BarBosses {
    /// Half the bar's length along `x`.
    pub half_length: f64,
    /// Half its width along `y`.
    pub half_width: f64,
    /// Half its thickness along `z`.
    pub half_thickness: f64,
    /// The boss at `x = half_length`.
    pub first: Boss,
    /// The boss at `x = −half_length`, if any.
    pub second: Option<Boss>,
    /// The motion all share.
    pub pose: Isometry,
}

impl BarBosses {
    /// The bar, then the bosses, in `m`.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Vec<Body>), OpError> {
        let (a, w, t) = (self.half_length, self.half_width, self.half_thickness);
        let (bar, _) = primitive_box(m, Point3::new(-a, -w, -t), Point3::new(a, w, t))?;
        let bar = transform(m, bar, &self.pose)?.0;
        let mut bosses = Vec::new();
        for (boss, side) in [(Some(self.first), 1.0), (self.second, -1.0)] {
            if let Some(b) = boss {
                let x = side * (a - b.radius);
                bosses.push(upright(m, (x, b.across), b.radius, b.z, &self.pose)?);
            }
        }
        Ok((bar, bosses))
    }

    /// The bar's volume.
    pub fn bar_volume(&self) -> f64 {
        8.0 * self.half_length * self.half_width * self.half_thickness
    }

    /// A boss's volume and the volume it shares with the bar: the part of
    /// its disc inside the bar's width, over the `z` they have in common.
    pub fn boss_volumes(&self, boss: &Boss) -> (f64, f64) {
        let w = self.half_width;
        let area =
            disc_below(boss.radius, w - boss.across) - disc_below(boss.radius, -w - boss.across);
        let z = overlap((-self.half_thickness, self.half_thickness), boss.z);
        let pi = core::f64::consts::PI;
        (
            pi * boss.radius * boss.radius * (boss.z.1 - boss.z.0),
            area * z,
        )
    }

    /// The bosses present, first then second.
    pub fn bosses(&self) -> Vec<Boss> {
        let mut all = vec![self.first];
        all.extend(self.second);
        all
    }
}

/// [`BarBosses`] draws: a bar 14 to 30 long, 3 to 8 wide and 1 to 4 thick,
/// bosses of radius 1 to 3 (long enough bars keep the two apart) at any
/// place across the width, half of them exactly as thick as the bar, and a
/// random pose.
pub fn bar_bosses() -> impl Strategy<Value = BarBosses> {
    let boss = |t: f64| {
        (
            radius(1.0..=3.0),
            finite_f64(-1.0..=1.0),
            any::<bool>(),
            (finite_f64(-t - 1.0..=t - 0.25), finite_f64(0.5..=4.0)),
        )
            .prop_filter_map(
                "a boss shares some thickness",
                move |(r, f, flush, (lo, h))| {
                    let z = if flush { (-t, t) } else { (lo, lo + h) };
                    (overlap((-t, t), z) >= 0.25).then_some((r, f, z))
                },
            )
    };
    (
        (
            finite_f64(7.0..=15.0),
            finite_f64(1.5..=4.0),
            finite_f64(0.5..=2.0),
        ),
        any::<bool>(),
        pose(),
    )
        .prop_flat_map(move |((a, w, t), two, pose)| (Just((a, w, t, two, pose)), boss(t), boss(t)))
        .prop_map(
            |((a, w, t, two, pose), (r1, f1, z1), (r2, f2, z2))| BarBosses {
                half_length: a,
                half_width: w,
                half_thickness: t,
                first: Boss {
                    radius: r1,
                    across: f1 * w,
                    z: z1,
                },
                second: two.then_some(Boss {
                    radius: r2,
                    across: f2 * w,
                    z: z2,
                }),
                pose,
            },
        )
}

/// A box bored through along `z` by a cylinder through its centre, and a
/// square column through the same place whose sides touch the bore along
/// four lines. The column is turned `turn` about `z`, so the lines are
/// anywhere about the box's own axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoreColumn {
    /// The box's half extents; both across extents exceed the half diagonal of the column's
    /// side, so the column stands inside the box's width at any turn.
    pub half: (f64, f64, f64),
    /// The bore's radius, and half the column's side.
    pub radius: f64,
    /// Half the column's length along `z`, more than the box's half height.
    pub column_half: f64,
    /// The column's turn about `z`, in radians.
    pub turn: f64,
    /// The motion all share.
    pub pose: Isometry,
}

impl BoreColumn {
    /// The box, the bore and the column, in `m`. The bore reaches a unit
    /// past the box at each end.
    pub fn build(&self, m: &mut Model) -> Result<(Body, Body, Body), OpError> {
        let (a, b, c) = self.half;
        let (cuboid, _) = primitive_box(m, Point3::new(-a, -b, -c), Point3::new(a, b, c))?;
        let cuboid = transform(m, cuboid, &self.pose)?.0;
        let bore = upright(m, (0.0, 0.0), self.radius, (-c - 1.0, c + 1.0), &self.pose)?;
        let (r, h) = (self.radius, self.column_half);
        let (column, _) = primitive_box(m, Point3::new(-r, -r, -h), Point3::new(r, r, h))?;
        let turn =
            Isometry::from_rotation(UnitQuaternion::from_axis_angle(&Vec3::z_axis(), self.turn));
        let column = transform(m, column, &turn.then(&self.pose))?.0;
        Ok((cuboid, bore, column))
    }
}

/// [`BoreColumn`] draws: a bore of radius 0.4 to 2, a box a half unit to
/// six wider than the column's turned corner on each side and 1 to 6 half high, a column
/// a half to five units longer than it at each end, any turn, a random
/// pose.
pub fn bore_column() -> impl Strategy<Value = BoreColumn> {
    (
        radius(0.4..=2.0),
        (finite_f64(0.5..=6.0), finite_f64(0.5..=6.0)),
        finite_f64(1.0..=6.0),
        finite_f64(0.5..=5.0),
        finite_f64(0.0..=core::f64::consts::TAU),
        pose(),
    )
        .prop_map(|(r, (ea, eb), c, extra, turn, pose)| BoreColumn {
            half: (r * SQRT_2 + ea, r * SQRT_2 + eb, c),
            radius: r,
            column_half: c + extra,
            turn,
            pose,
        })
}
