//! Random *recipes*: the corpus grammar's twelve operations drawn by a
//! seeded strategy, so the same recipe runs through Arris and through the
//! oracle and the two can be compared (ADR-0024 §2). A drawn recipe is a
//! `fixture.json` like any other — it prints as one, and a failure is
//! committed as one.
//!
//! A recipe is two to four **operands** chained by one to three of
//! `fuse`, `common` and `cut`. Each operand is a box, a cylinder, or a
//! profile from [`super::profile`] extruded or revolved; a box or a
//! cylinder may have one edge filleted or chamfered first, where the
//! edge's point is known in closed form (or, of those without a blend, have
//! one face moved by `offset`, ADR-0048, or the whole hollowed by `shell`,
//! open on one face or closed, ADR-0049), and a stadium or a plate with a
//! D-shaped notch has its whole top outline blended through one edge
//! (ADR-0035). [`turned_recipe`] draws a turned part alone, a coned
//! shoulder, a dome or a toroidal bead revolved about `y` with a pick of
//! its circular corners blended in one call (ADR-0036), outside the
//! general draw so that one's seeded stream is unchanged. Every operand is then placed by
//! one `transform`: its own motion within [`OFFSET`] of the others (so
//! the operands overlap), under a motion the recipe shares, which is the
//! identity half the time and a pose at [`super::DEFAULT_SCALE`]
//! otherwise. A quarter of the operands are then mirrored in a plane
//! through a point near the others (`mirror`, ADR-0031), and their probes
//! with them.
//!
//! The probes are every operand's centre and, for a box or a cylinder,
//! points [`PROBE_STEP`] inside and outside each of its faces along its
//! own axes, placed by the operand's motion; a hollowed one's centre is in
//! its cavity, and its probes straddle both skins and its mouth. Tolerances and precision are
//! the fixture defaults.

use core::f64::consts::TAU;
use core::ops::RangeInclusive;

use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{Axis, Frame, Isometry, Point2, Point3, Reflection, Vec3};
use proptest::prelude::*;

use super::body::{MAX_EXTENT, MIN_EXTENT};
use super::turned::Turned;
use super::{finite_f64, point_in_box, pose, radius, rotation, unit_vec3};
use crate::fixtures::{
    self, Analytic, Circle, Ellipse, Loop, MirrorPlane, Num, Plane, PrecisionSpec, Probe, Recipe,
    Rotate, Segment, ShellSide, Step, Tolerances,
};

/// How many booleans a recipe chains; it has one operand more.
pub const BOOLEANS: RangeInclusive<usize> = 1..=3;

/// The half-width of the box an operand's centre is placed in about the
/// others', before the shared motion: under [`MIN_EXTENT`]'s multiple of
/// the extents, so most operands overlap and some only touch or miss.
pub const OFFSET: f64 = 5.0;

/// How far inside and outside a face a probe along an operand's axis
/// lies: far above the fixtures' probe tolerance, so neither kernel calls
/// it "on", and far below the smallest extent.
pub const PROBE_STEP: f64 = 0.01;

/// A blend's size as a fraction of the smaller of the two extents beside
/// its edge: never so large that the blend leaves a face, never within
/// rounding of zero.
pub const BLEND_FRACTION: RangeInclusive<f64> = 0.05..=0.4;

/// How far round a cylinder's cap edge a blend's point lies from the
/// seam's side, in radians: clear of the seam vertex both kernels put
/// at angle zero.
const CAP_ANGLE: RangeInclusive<f64> = 0.5..=(TAU - 0.5);

/// What an operand is before its motion, in its own frame.
#[derive(Debug, Clone, PartialEq)]
enum Shape {
    /// A box centred on the origin.
    Box { extents: Vec3 },
    /// A cylinder on the `z` axis, centred on the origin.
    Cylinder { radius: f64, height: f64 },
    /// A profile in the `xy` plane extruded along `z`.
    Extrude {
        profile: Profile,
        length: f64,
        centre: Point2,
    },
    /// A stadium or a plate with a D-shaped notch, extruded along `z`, its
    /// whole top outline blended through the one edge at `edge`, a point
    /// of the edge in the shape's own frame (ADR-0035): a chain of line and
    /// arc stripes, or an open arc. `centre` is a point of the material in
    /// the profile.
    Rim {
        profile: Profile,
        length: f64,
        centre: Point2,
        edge: Point3,
        size: f64,
        chamfer: bool,
    },
    /// A turned part: lines and an arc revolved a whole turn about `y`,
    /// some of its circular edges blended in one call (ADR-0036).
    Turned(Turned),
    /// A profile in the `xy` plane revolved about an axis in that plane;
    /// `centre` is a point of the material in the profile.
    Revolve {
        profile: Profile,
        axis: Axis,
        angle: f64,
        centre: Point2,
    },
}

/// A fillet or chamfer of one edge of a box or a cylinder.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Blend {
    chamfer: bool,
    /// Which edge: a box's 12 by `edge % 12`, a cylinder's two caps by
    /// `edge % 2`.
    edge: usize,
    /// Round a cylinder's cap from its seam, in radians.
    angle: f64,
    /// The size as a fraction ([`BLEND_FRACTION`]).
    fraction: f64,
}

/// One face of a box or a cylinder moved along its normal by `offset`
/// before the operand is placed: its whole body is the operand's, so no
/// blend has run first and the face's point is known in closed form.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Push {
    /// Which face: a box's six by `face % 6`, a cylinder's side, top and
    /// bottom by `face % 3`.
    face: usize,
    /// Where on a cylinder's side the face's point lies, in radians from
    /// the seam ([`CAP_ANGLE`]).
    angle: f64,
    /// The distance as a signed fraction of the extent behind the face
    /// ([`PUSH_FRACTION`]): never near zero, and pulled in by less than
    /// half the extent.
    fraction: f64,
}

/// How far a face is moved, as a fraction of the extent along its normal:
/// outward up to half again, inward up to two fifths — no face then
/// vanishes — and never within [`PUSH_MIN`] of zero.
const PUSH_FRACTION: RangeInclusive<f64> = -0.4..=0.5;

/// The least fraction of the extent a drawn push moves a face by.
const PUSH_MIN: f64 = 0.05;

/// A box or a cylinder hollowed by `shell` before the operand is placed:
/// its whole body is the operand's, so no blend or push has run first.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Hollow {
    /// The face left open: a box's six by `face % 6`, a cylinder's top
    /// and bottom by `face % 2` (its side is never opened); `None` for a
    /// closed void.
    opening: Option<usize>,
    /// The thickness as a fraction of the operand's half-extent
    /// ([`HOLLOW_FRACTION`]).
    fraction: f64,
    /// The wall grown outside the faces, not inside them.
    outward: bool,
}

/// A hollow's thickness as a fraction of the operand's least half-extent
/// (a cylinder's radius or half its height): well clear of [`PROBE_STEP`]
/// at the smallest extent, and inward, a cavity left of at least four
/// tenths of the extent.
const HOLLOW_FRACTION: RangeInclusive<f64> = 0.1..=0.6;

/// One operand as drawn.
#[derive(Debug, Clone, PartialEq)]
struct Operand {
    shape: Shape,
    blend: Option<Blend>,
    /// A face moved by an offset, on a box or a cylinder with no blend.
    push: Option<Push>,
    /// The whole hollowed, on a box or a cylinder with no blend or push.
    hollow: Option<Hollow>,
    /// Its own motion about the others', its centre at the origin.
    local: Isometry,
    /// The plane it is reflected in after its motions: a point and a
    /// normal, in the world the operands share.
    mirror: Option<(Point3, Vec3)>,
}

/// A boolean of the chain so far with the next operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Op {
    Fuse,
    Common,
    Cut,
}

/// The profile of `sweep`'s own plane moved into the `xy` plane at the
/// origin, and a point of its axis and direction moved with it.
fn flatten(profile: &Profile, axis: Option<&Axis>) -> (Profile, Option<Axis>) {
    let plane = profile.plane;
    let axis = axis.map(|a| Axis {
        origin: plane.to_local(a.origin),
        direction: arris_math::UnitVec3::new_normalize(
            plane.vec_to_local(a.direction.into_inner()),
        ),
    });
    (
        Profile {
            plane: Frame::world(),
            ..profile.clone()
        },
        axis,
    )
}

fn shape() -> impl Strategy<Value = Shape> {
    let extent = || finite_f64(MIN_EXTENT..=MAX_EXTENT);
    let length = || finite_f64(1.0..=10.0);
    prop_oneof![
        3 => (extent(), extent(), extent()).prop_map(|(x, y, z)| Shape::Box {
            extents: Vec3::new(x, y, z)
        }),
        3 => (
            radius(MIN_EXTENT / 2.0..=MAX_EXTENT / 2.0),
            radius(MIN_EXTENT..=MAX_EXTENT)
        )
            .prop_map(|(radius, height)| Shape::Cylinder { radius, height }),
        1 => (super::profile::star(), length()).prop_map(|(p, length)| Shape::Extrude {
            profile: flatten(&p, None).0,
            length,
            centre: Point2::origin(),
        }),
        1 => super::profile::ellipse().prop_map(|(p, length)| {
            let centre = match &p.outer {
                ProfileLoop::Ellipse { center, .. } => *center,
                _ => Point2::origin(),
            };
            Shape::Extrude {
                profile: flatten(&p, None).0,
                length,
                centre,
            }
        }),
        1 => rim(),
        1 => super::profile::general().prop_map(|s| Shape::Extrude {
            profile: flatten(&s.profile, None).0,
            length: s.length,
            centre: Point2::origin(),
        }),
        1 => super::profile::general().prop_filter_map("an axis in the plane", |s| {
            let (profile, axis) = flatten(&s.profile, Some(&s.axis));
            Some(Shape::Revolve {
                profile,
                axis: axis?,
                angle: s.angle,
                centre: Point2::origin(),
            })
        }),
    ]
}

/// A stadium of two lines `l` long and two half circles of radius `r`, or a
/// `w × h` plate with a half disc of radius `r` cut from its `y = 0` side,
/// extruded `length` and blended along its top at a fraction of the
/// smaller of `r` and `length` ([`BLEND_FRACTION`]).
fn rim() -> impl Strategy<Value = Shape> {
    let line = |u: f64, v: f64| ProfileSegment::LineTo(Point2::new(u, v));
    let arc = |u: f64, v: f64, via: (f64, f64)| ProfileSegment::ArcTo {
        to: Point2::new(u, v),
        via: Point2::new(via.0, via.1),
    };
    let path = move |start: (f64, f64), segments: Vec<ProfileSegment>| Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: Point2::new(start.0, start.1),
            segments,
        },
        holes: Vec::new(),
    };
    let common = || {
        (
            finite_f64(1.0..=4.0),
            finite_f64(1.0..=4.0),
            finite_f64(0.5..=3.0),
            finite_f64(BLEND_FRACTION),
            any::<bool>(),
        )
    };
    prop_oneof![
        (common(), 0usize..4).prop_map(move |((l, r, length, fraction, chamfer), pick)| {
            let profile = path(
                (0.0, -r),
                vec![
                    line(l, -r),
                    arc(l, r, (l + r, 0.0)),
                    line(0.0, r),
                    arc(0.0, -r, (-r, 0.0)),
                ],
            );
            let edge = [(l / 2.0, -r), (l + r, 0.0), (l / 2.0, r), (-r, 0.0)][pick];
            Shape::Rim {
                profile,
                length,
                centre: Point2::new(l / 2.0, 0.0),
                edge: Point3::new(edge.0, edge.1, length),
                size: fraction * r.min(length),
                chamfer,
            }
        }),
        (common(), finite_f64(0.2..=0.6)).prop_map(
            move |((w, h, length, fraction, chamfer), f)| {
                let (w, h) = (w + 2.0, h + 2.0);
                let r = f * w / 2.0;
                let profile = path(
                    (0.0, 0.0),
                    vec![
                        line(w / 2.0 - r, 0.0),
                        arc(w / 2.0 + r, 0.0, (w / 2.0, r)),
                        line(w, 0.0),
                        line(w, h),
                        line(0.0, h),
                        line(0.0, 0.0),
                    ],
                );
                Shape::Rim {
                    profile,
                    length,
                    centre: Point2::new(w / 2.0, (r + h) / 2.0),
                    edge: Point3::new(w / 2.0, r, length),
                    size: fraction * r.min(length),
                    chamfer,
                }
            }
        ),
    ]
}

/// Recipes of one turned part each, no boolean: what the differential
/// holds to Open CASCADE for the meridian row's blends alone (ADR-0036).
pub fn turned_recipe() -> impl Strategy<Value = Recipe> {
    (
        super::turned::turned(),
        prop_oneof![Just(Isometry::identity()), pose()],
    )
        .prop_map(|(t, shared)| {
            let operand = Operand {
                shape: Shape::Turned(t),
                blend: None,
                push: None,
                hollow: None,
                local: Isometry::identity(),
                mirror: None,
            };
            write(&[operand], &[], &shared)
        })
}

fn blend() -> impl Strategy<Value = Blend> {
    (
        any::<bool>(),
        0usize..12,
        finite_f64(CAP_ANGLE),
        finite_f64(BLEND_FRACTION),
    )
        .prop_map(|(chamfer, edge, angle, fraction)| Blend {
            chamfer,
            edge,
            angle,
            fraction,
        })
}

fn push() -> impl Strategy<Value = Push> {
    (0usize..6, finite_f64(CAP_ANGLE), finite_f64(PUSH_FRACTION)).prop_map(
        |(face, angle, fraction)| Push {
            face,
            angle,
            fraction: if fraction.abs() < PUSH_MIN {
                PUSH_MIN.copysign(fraction)
            } else {
                fraction
            },
        },
    )
}

fn hollow() -> impl Strategy<Value = Hollow> {
    (
        proptest::option::weighted(0.75, 0usize..6),
        finite_f64(HOLLOW_FRACTION),
        any::<bool>(),
    )
        .prop_map(|(opening, fraction, outward)| Hollow {
            opening,
            fraction,
            outward,
        })
}

fn operand() -> impl Strategy<Value = Operand> {
    let turn = prop_oneof![Just(UnitQuaternion::identity()), rotation()];
    (
        shape(),
        proptest::option::weighted(1.0 / 3.0, blend()),
        turn,
        point_in_box(OFFSET),
        proptest::option::weighted(0.25, (point_in_box(OFFSET), unit_vec3())),
        proptest::option::weighted(0.2, push()),
        proptest::option::weighted(0.2, hollow()),
    )
        .prop_map(|(shape, blend, q, at, mirror, push, hollow)| {
            let simple = matches!(shape, Shape::Box { .. } | Shape::Cylinder { .. });
            let blend = if simple { blend } else { None };
            let push = if simple && blend.is_none() {
                push
            } else {
                None
            };
            Operand {
                mirror: mirror.map(|(o, n)| (o, n.into_inner())),
                hollow: if simple && blend.is_none() && push.is_none() {
                    hollow
                } else {
                    None
                },
                push,
                blend,
                shape,
                local: Isometry::new(q, at.coords),
            }
        })
}

fn op() -> impl Strategy<Value = Op> {
    prop_oneof![Just(Op::Fuse), Just(Op::Common), Just(Op::Cut)]
}

/// Recipes of [`BOOLEANS`] booleans over one operand more, as the module
/// describes. Every recipe is well formed: its steps name only steps
/// before them, its result is its last step, its profiles are valid and
/// every blend's point lies on one edge — so a refusal building it is the
/// kernel's, never the recipe's.
pub fn recipe() -> impl Strategy<Value = Recipe> {
    (
        proptest::collection::vec((operand(), op()), BOOLEANS),
        operand(),
        prop_oneof![Just(Isometry::identity()), pose()],
    )
        .prop_map(|(chained, first, shared)| {
            let mut operands = vec![first];
            let mut ops = Vec::new();
            for (operand, op) in chained {
                operands.push(operand);
                ops.push(op);
            }
            write(&operands, &ops, &shared)
        })
}

fn num3(v: [f64; 3]) -> [Num; 3] {
    v.map(Num::from)
}

fn num2(p: Point2) -> [Num; 2] {
    [p.x.into(), p.y.into()]
}

fn vec3(v: Vec3) -> [Num; 3] {
    num3([v.x, v.y, v.z])
}

fn point3(p: Point3) -> [Num; 3] {
    vec3(p.coords)
}

fn loop_of(l: &ProfileLoop) -> Loop {
    match l {
        ProfileLoop::Circle { center, radius } => Loop::Circle {
            circle: Circle {
                center: num2(*center),
                radius: (*radius).into(),
            },
        },
        ProfileLoop::Ellipse {
            center,
            major,
            minor_radius,
        } => Loop::Ellipse {
            ellipse: Ellipse {
                center: num2(*center),
                major: num2(Point2::from(*major)),
                minor_radius: (*minor_radius).into(),
            },
        },
        ProfileLoop::Path { start, segments } => Loop::Path {
            start: num2(*start),
            segments: segments
                .iter()
                .map(|s| match s {
                    ProfileSegment::LineTo(to) => Segment::Line { line_to: num2(*to) },
                    ProfileSegment::ArcTo { to, via } => Segment::Arc {
                        arc_to: num2(*to),
                        via: num2(*via),
                    },
                    ProfileSegment::EllipseTo {
                        to,
                        center,
                        major,
                        minor_radius,
                        ccw,
                    } => Segment::Ellipse {
                        ellipse_to: num2(*to),
                        center: num2(*center),
                        major: num2(Point2::from(*major)),
                        minor_radius: (*minor_radius).into(),
                        ccw: *ccw,
                    },
                })
                .collect(),
        },
    }
}

/// The step moving a body by `motion`: its rotation about an axis through
/// the origin, then its translation, as the grammar applies them.
fn transform_step(name: String, of: String, motion: &Isometry) -> Step {
    let rotate = motion.rotation().axis_angle().map(|(axis, angle)| Rotate {
        axis: vec3(axis.into_inner()),
        origin: None,
        angle_deg: angle.to_degrees().into(),
    });
    let t = motion.translation();
    Step::Transform {
        name,
        of,
        translate: (t != Vec3::zeros()).then(|| vec3(t)),
        rotate,
    }
}

/// The point of a box's edge `edge % 12` halfway along it, and the two
/// extents beside it: edges along `x`, then `y`, then `z`, four each, by
/// the signs of the other two coordinates.
fn box_edge(extents: Vec3, edge: usize) -> (Point3, f64) {
    let edge = edge % 12;
    let along = edge / 4;
    let (j, k) = ((along + 1) % 3, (along + 2) % 3);
    let sign = |bit: usize| if edge & bit == 0 { -0.5 } else { 0.5 };
    let mut p = Vec3::zeros();
    p[j] = sign(1) * extents[j];
    p[k] = sign(2) * extents[k];
    (Point3::from(p), extents[j].min(extents[k]))
}

/// Writes the steps and probes of `operands` chained by `ops`, every
/// operand placed by `shared` after its own motion.
fn write(operands: &[Operand], ops: &[Op], shared: &Isometry) -> Recipe {
    let mut steps = Vec::new();
    let mut probes = Vec::new();
    let mut placed = Vec::new();
    for (i, o) in operands.iter().enumerate() {
        let base = format!("b{i}");
        // The operand's centre in its own frame, and the probes along its
        // axes before the motion.
        let mut local: Vec<(String, Point3)> = Vec::new();
        let mut rim_body = None;
        let centre = match &o.shape {
            Shape::Box { extents } => {
                let e = *extents;
                steps.push(Step::Box {
                    name: base.clone(),
                    min: vec3(-e / 2.0),
                    max: vec3(e / 2.0),
                });
                for axis in 0..3 {
                    let moved = |side: usize| push_of(o, axis * 2 + side) * e[axis];
                    let half = [e[axis] / 2.0 + moved(0), e[axis] / 2.0 + moved(1)];
                    match o.hollow {
                        Some(h) => {
                            let open =
                                |side: usize| h.opening.map(|f| f % 6) == Some(axis * 2 + side);
                            let t = h.fraction * e.min() / 2.0;
                            along_hollow(&mut local, axis, half, t, h.outward, [open(0), open(1)]);
                        }
                        None => along_axis(&mut local, axis, half),
                    }
                }
                Point3::origin()
            }
            Shape::Cylinder { radius, height } => {
                steps.push(Step::Cylinder {
                    name: base.clone(),
                    base: num3([0.0, 0.0, -height / 2.0]),
                    axis: num3([0.0, 0.0, 1.0]),
                    radius: (*radius).into(),
                    height: (*height).into(),
                });
                let side = radius + push_of(o, 0) * radius;
                // The caps: face 2 is the bottom, 1 the top.
                let caps = [
                    height / 2.0 + push_of(o, 2) * height,
                    height / 2.0 + push_of(o, 1) * height,
                ];
                match o.hollow {
                    Some(h) => {
                        // A hollow's opening: 0 the bottom, 1 the top.
                        let open = |side: usize| h.opening.map(|f| f % 2) == Some(side);
                        let t = h.fraction * radius.min(height / 2.0);
                        along_hollow(&mut local, 0, [side, side], t, h.outward, [false; 2]);
                        along_hollow(&mut local, 1, [side, side], t, h.outward, [false; 2]);
                        along_hollow(&mut local, 2, caps, t, h.outward, [open(0), open(1)]);
                    }
                    None => {
                        along_axis(&mut local, 0, [side, side]);
                        along_axis(&mut local, 1, [side, side]);
                        along_axis(&mut local, 2, caps);
                    }
                }
                Point3::origin()
            }
            Shape::Extrude {
                profile,
                length,
                centre,
            } => {
                let sketch = format!("sk{i}");
                steps.push(profile_step(&sketch, profile));
                steps.push(Step::Extrude {
                    name: base.clone(),
                    profile: sketch,
                    direction: num3([0.0, 0.0, 1.0]),
                    length: (*length).into(),
                });
                Point3::new(centre.x, centre.y, length / 2.0)
            }
            Shape::Rim {
                profile,
                length,
                centre,
                edge,
                size,
                chamfer,
            } => {
                let sketch = format!("sk{i}");
                steps.push(profile_step(&sketch, profile));
                steps.push(Step::Extrude {
                    name: base.clone(),
                    profile: sketch,
                    direction: num3([0.0, 0.0, 1.0]),
                    length: (*length).into(),
                });
                let name = format!("f{i}");
                let (of, edges, size) = (base.clone(), vec![point3(*edge)], (*size).into());
                steps.push(if *chamfer {
                    Step::Chamfer {
                        name: name.clone(),
                        of,
                        edges,
                        distance: size,
                    }
                } else {
                    Step::Fillet {
                        name: name.clone(),
                        of,
                        edges,
                        radius: size,
                    }
                });
                rim_body = Some(name);
                Point3::new(centre.x, centre.y, length / 2.0)
            }
            Shape::Turned(turned) => {
                let sketch = format!("sk{i}");
                steps.push(profile_step(&sketch, &turned.profile()));
                steps.push(Step::Revolve {
                    name: base.clone(),
                    profile: sketch,
                    axis: fixtures::Axis {
                        origin: num3([0.0, 0.0, 0.0]),
                        direction: num3([0.0, 1.0, 0.0]),
                    },
                    angle_deg: 360.0.into(),
                });
                let name = format!("f{i}");
                let (of, size) = (base.clone(), turned.size.into());
                let edges = turned
                    .blended
                    .iter()
                    .map(|&k| point3(turned.edge_point(k)))
                    .collect();
                steps.push(if turned.chamfer {
                    Step::Chamfer {
                        name: name.clone(),
                        of,
                        edges,
                        distance: size,
                    }
                } else {
                    Step::Fillet {
                        name: name.clone(),
                        of,
                        edges,
                        radius: size,
                    }
                });
                rim_body = Some(name);
                turned.centre()
            }
            Shape::Revolve {
                profile,
                axis,
                angle,
                centre,
            } => {
                let sketch = format!("sk{i}");
                steps.push(profile_step(&sketch, profile));
                steps.push(Step::Revolve {
                    name: base.clone(),
                    profile: sketch,
                    axis: fixtures::Axis {
                        origin: point3(axis.origin),
                        direction: vec3(axis.direction.into_inner()),
                    },
                    angle_deg: angle.to_degrees().into(),
                });
                // Halfway round the sweep, right-handed about the axis.
                let half = UnitQuaternion::from_axis_angle(&axis.direction, angle / 2.0);
                let c = Point3::new(centre.x, centre.y, 0.0);
                axis.origin + half * (c - axis.origin)
            }
        };
        let mut body = rim_body.unwrap_or(base);
        if let Some(p) = o.push {
            let (point, extent) = match &o.shape {
                Shape::Box { extents } => {
                    let axis = (p.face % 6) / 2;
                    let side = if p.face % 2 == 0 { -0.5 } else { 0.5 };
                    let mut at = Vec3::zeros();
                    at[axis] = side * extents[axis];
                    (Point3::from(at), extents[axis])
                }
                Shape::Cylinder { radius, height } => match p.face % 3 {
                    0 => (
                        Point3::new(radius * p.angle.cos(), radius * p.angle.sin(), 0.0),
                        *radius,
                    ),
                    1 => (Point3::new(0.0, 0.0, height / 2.0), *height),
                    _ => (Point3::new(0.0, 0.0, -height / 2.0), *height),
                },
                _ => unreachable!("only a box or a cylinder is pushed"),
            };
            let name = format!("p{i}");
            steps.push(Step::Offset {
                name: name.clone(),
                of: body,
                faces: vec![point3(point)],
                distance: (p.fraction * extent).into(),
            });
            body = name;
        }
        if let Some(h) = o.hollow {
            let (openings, thickness) = match &o.shape {
                Shape::Box { extents } => {
                    let opening = h.opening.map(|f| {
                        let face = f % 6;
                        let mut at = Vec3::zeros();
                        at[face / 2] = if face % 2 == 0 { -0.5 } else { 0.5 } * extents[face / 2];
                        point3(Point3::from(at))
                    });
                    (opening, h.fraction * extents.min() / 2.0)
                }
                Shape::Cylinder { radius, height } => {
                    let opening = h.opening.map(|f| {
                        let z = if f % 2 == 0 { -height } else { *height } / 2.0;
                        point3(Point3::new(0.0, 0.0, z))
                    });
                    (opening, h.fraction * radius.min(height / 2.0))
                }
                _ => unreachable!("only a box or a cylinder is hollowed"),
            };
            let name = format!("h{i}");
            steps.push(Step::Shell {
                name: name.clone(),
                of: body,
                openings: openings.into_iter().collect(),
                thickness: thickness.into(),
                side: if h.outward {
                    ShellSide::Outward
                } else {
                    ShellSide::Inward
                },
            });
            body = name;
        }
        if let Some(b) = o.blend {
            let (point, size) = match &o.shape {
                Shape::Box { extents } => box_edge(*extents, b.edge),
                Shape::Cylinder { radius, height } => {
                    let z = if b.edge % 2 == 0 { -height } else { *height } / 2.0;
                    let p = Point3::new(radius * b.angle.cos(), radius * b.angle.sin(), z);
                    (p, radius.min(*height))
                }
                _ => unreachable!("only a box or a cylinder is blended"),
            };
            let name = format!("f{i}");
            let edges = vec![point3(point)];
            let size = (b.fraction * size).into();
            steps.push(if b.chamfer {
                Step::Chamfer {
                    name: name.clone(),
                    of: body,
                    edges,
                    distance: size,
                }
            } else {
                Step::Fillet {
                    name: name.clone(),
                    of: body,
                    edges,
                    radius: size,
                }
            });
            body = name;
        }
        // Centre on the origin, the operand's own motion, the shared one.
        let motion = Isometry::from_translation(-centre.coords)
            .then(&o.local)
            .then(shared);
        let name = format!("o{i}");
        steps.push(transform_step(name.clone(), body, &motion));
        // The mirror, if drawn, after the motions: the probes follow it.
        let reflect = o.mirror.and_then(|(origin, normal)| {
            let plane = Reflection::new(origin, normal).ok()?;
            Some((origin, normal, plane))
        });
        let (name, place) = match reflect {
            Some((origin, normal, plane)) => {
                let image = format!("m{i}");
                steps.push(Step::Mirror {
                    name: image.clone(),
                    of: name,
                    plane: MirrorPlane {
                        origin: point3(origin),
                        normal: vec3(normal),
                    },
                });
                (image, Some(plane))
            }
            None => (name, None),
        };
        let at = |p: Point3| {
            let moved = motion.apply(p);
            place.as_ref().map_or(moved, |plane| plane.apply(moved))
        };
        // A hollow's centre is in its cavity.
        let centre_label = if o.hollow.is_some() {
            "cavity-out"
        } else {
            "centre"
        };
        probes.push(probe(format!("{name}-{centre_label}"), at(centre)));
        for (label, p) in local {
            probes.push(probe(format!("{name}-{label}"), at(p)));
        }
        placed.push(name);
    }
    let mut last = placed[0].clone();
    for (k, op) in ops.iter().enumerate() {
        let name = format!("r{}", k + 1);
        let next = placed[k + 1].clone();
        steps.push(match op {
            Op::Fuse => Step::Fuse {
                name: name.clone(),
                a: Some(last),
                b: Some(next),
                bodies: Vec::new(),
            },
            Op::Common => Step::Common {
                name: name.clone(),
                a: last,
                b: next,
            },
            Op::Cut => Step::Cut {
                name: name.clone(),
                target: last,
                tool: Some(next),
                tools: Vec::new(),
            },
        });
        last = name;
    }
    Recipe {
        description: "A random recipe drawn by arris_debug::prop::recipe.".into(),
        params: Default::default(),
        variants: Default::default(),
        steps,
        result: last,
        probes,
        precision: PrecisionSpec::default(),
        tolerances: Tolerances::default(),
        analytic: Analytic::default(),
    }
}

/// The signed fraction of its extent that the operand's face `face` is
/// moved by: a box's six by `face % 6` (`axis * 2 + side`, the side `0` at
/// the minimum), a cylinder's side, top and bottom by `face % 3`; zero for
/// every other face, and for an operand with no push.
fn push_of(o: &Operand, face: usize) -> f64 {
    let Some(p) = o.push else {
        return 0.0;
    };
    let modulus = match o.shape {
        Shape::Box { .. } => 6,
        _ => 3,
    };
    if p.face % modulus == face {
        p.fraction
    } else {
        0.0
    }
}

/// The four probes along `axis` of an operand whose faces across it are
/// `half` from its centre — the minimum side's, then the maximum side's:
/// just in and just out on either side.
fn along_axis(out: &mut Vec<(String, Point3)>, axis: usize, half: [f64; 2]) {
    let name = ["x", "y", "z"][axis];
    for (sign, side, half) in [(-1.0, "-", half[0]), (1.0, "+", half[1])] {
        for (reach, how) in [(half - PROBE_STEP, "in"), (half + PROBE_STEP, "out")] {
            let mut p = Vec3::zeros();
            p[axis] = sign * reach;
            out.push((format!("{name}{side}-{how}"), Point3::from(p)));
        }
    }
}

/// The probes along `axis` of a hollowed operand whose faces across it
/// are `half` from its centre, its wall `t` thick and grown `outward` or
/// in, the side at either end `open` or not: just in and just out of
/// each skin, the cavity's labelled `cavity-out`, and at an opening, the
/// mouth's either side of its plane, both out.
fn along_hollow(
    out: &mut Vec<(String, Point3)>,
    axis: usize,
    half: [f64; 2],
    t: f64,
    outward: bool,
    open: [bool; 2],
) {
    let name = ["x", "y", "z"][axis];
    for ((sign, side, half), open) in [(-1.0, "-", half[0]), (1.0, "+", half[1])]
        .into_iter()
        .zip(open)
    {
        // Each skin's distance from the centre, the inner then the outer.
        let (inner, outer) = if outward {
            (half, half + t)
        } else {
            (half - t, half)
        };
        let reaches: Vec<(f64, &str)> = if open {
            vec![
                (inner - PROBE_STEP, "cavity-out"),
                (outer + PROBE_STEP, "mouth-out"),
            ]
        } else {
            vec![
                (inner - PROBE_STEP, "cavity-out"),
                (inner + PROBE_STEP, "wall-in"),
                (outer - PROBE_STEP, "in"),
                (outer + PROBE_STEP, "out"),
            ]
        };
        for (reach, how) in reaches {
            let mut p = Vec3::zeros();
            p[axis] = sign * reach;
            out.push((format!("{name}{side}-{how}"), Point3::from(p)));
        }
    }
}

fn probe(label: String, point: Point3) -> Probe {
    Probe {
        label,
        point: point3(point),
        expect: None,
    }
}

fn profile_step(name: &str, profile: &Profile) -> Step {
    let origin = profile.plane.to_world(Point3::origin());
    let x = profile.plane.vec_to_world(Vec3::x());
    let y = profile.plane.vec_to_world(Vec3::y());
    Step::Profile {
        name: name.to_string(),
        plane: Plane {
            origin: point3(origin),
            x: vec3(x),
            y: vec3(y),
        },
        outer: loop_of(&profile.outer),
        holes: profile.holes.iter().map(loop_of).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_edge_point_is_on_the_edge_it_names() {
        let e = Vec3::new(2.0, 4.0, 6.0);
        let (p, beside) = box_edge(e, 0);
        assert_eq!(p, Point3::new(0.0, -2.0, -3.0));
        assert_eq!(beside, 4.0);
        let (p, beside) = box_edge(e, 11);
        assert_eq!(p, Point3::new(1.0, 2.0, 0.0));
        assert_eq!(beside, 2.0);
    }

    #[test]
    fn a_motion_writes_as_the_grammar_applies_it() {
        let q = UnitQuaternion::from_axis_angle(&Vec3::z_axis(), 0.5);
        let motion = Isometry::new(q, Vec3::new(1.0, 2.0, 3.0));
        let Step::Transform {
            translate, rotate, ..
        } = transform_step("t".into(), "b".into(), &motion)
        else {
            unreachable!()
        };
        let rotate = rotate.unwrap();
        let axis = Vec3::from(rotate.axis.map(|n| n.eval(&Default::default()).unwrap()));
        let angle = rotate.angle_deg.eval(&Default::default()).unwrap();
        let back = Isometry::new(
            UnitQuaternion::from_axis_angle(
                &arris_math::UnitVec3::new_normalize(axis),
                angle.to_radians(),
            ),
            Vec3::from(
                translate
                    .unwrap()
                    .map(|n| n.eval(&Default::default()).unwrap()),
            ),
        );
        let p = Point3::new(0.3, -0.7, 1.1);
        assert!((back.apply(p) - motion.apply(p)).norm() < 1e-14);
    }
}
