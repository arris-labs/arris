//! `ops::offset_faces` at random (ADR-0048): a box, a cylinder and a box
//! with all twelve edges rounded, of random proportions, in random poses.
//!
//! The whole body offset by `d` is the closed form — the box grown by `d` on
//! each side with sharp joins, the cylinder one of radius `r + d` and height
//! `h + 2d`, the rounded box (Steiner's formula) one of radius `r + d` — in
//! volume, area and counts; offset by `d` then `−d` returns the body's
//! volume, area and counts; a wall perpendicular to its neighbours (a box's
//! face, a cylinder's cap, an extruded L's wall) pushed by `d` changes the
//! volume by its area times `d`; offsetting then moving is moving then
//! offsetting; and the result is an operand of fillet, chamfer and cut, clean
//! at `Full`. A failure prints the case and the seed, and becomes a fixture
//! under `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).

use core::f64::consts::PI;

use arris_check::classify::{Classification, classify_point};
use arris_check::{Level, Report, check};
use arris_debug::testing::{REL, close_to, fail};
use arris_debug::unmetered::{
    chamfer, cut, extrude, fillet, mass_properties, offset_faces, primitive_box,
    primitive_cylinder, transform,
};
use arris_debug::{prop, prop_shards};
use arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_math::{Axis, Frame, Isometry, Point2, Point3, Vec3};
use arris_ops::OpError;
use arris_ops::measure::MassProperties;
use arris_topo::provenance::audit;
use arris_topo::{Body, Edge, EntityId, Face, Model};
use proptest::prelude::*;

/// What the properties offset, before its pose.
#[derive(Debug, Clone, PartialEq)]
enum Solid {
    /// The box from the origin to `(x, y, z)`.
    Box { x: f64, y: f64, z: f64 },
    /// The cylinder on the `z` axis from the origin, `height` tall.
    Cylinder { radius: f64, height: f64 },
    /// The box `(x, y, z)` with all twelve edges filleted at `radius`.
    Rounded { x: f64, y: f64, z: f64, radius: f64 },
}

impl Solid {
    /// The distance inward at which something collapses or vanishes: a
    /// whole offset by `−limit` or more is refused by design.
    fn limit(&self) -> f64 {
        match *self {
            Solid::Box { x, y, z } => x.min(y).min(z) / 2.0,
            Solid::Cylinder { radius, height } => radius.min(height / 2.0),
            Solid::Rounded { radius, .. } => radius,
        }
    }

    /// The scale an outward offset is drawn against.
    fn scale(&self) -> f64 {
        match *self {
            Solid::Box { x, y, z } | Solid::Rounded { x, y, z, .. } => x.min(y).min(z),
            Solid::Cylinder { radius, height } => radius.min(height),
        }
    }

    fn build(&self, m: &mut Model) -> Result<Body, TestCaseError> {
        match *self {
            Solid::Box { x, y, z } => Ok(primitive_box(m, Point3::origin(), Point3::new(x, y, z))
                .map_err(fail)?
                .0),
            Solid::Cylinder { radius, height } => {
                Ok(
                    primitive_cylinder(m, Axis::z_at(Point3::origin()), radius, height)
                        .map_err(fail)?
                        .0,
                )
            }
            Solid::Rounded { x, y, z, radius } => {
                let (b, _) =
                    primitive_box(m, Point3::origin(), Point3::new(x, y, z)).map_err(fail)?;
                let mids = [
                    [x / 2.0, 0.0, 0.0],
                    [x / 2.0, y, 0.0],
                    [x / 2.0, 0.0, z],
                    [x / 2.0, y, z],
                    [0.0, y / 2.0, 0.0],
                    [x, y / 2.0, 0.0],
                    [0.0, y / 2.0, z],
                    [x, y / 2.0, z],
                    [0.0, 0.0, z / 2.0],
                    [x, 0.0, z / 2.0],
                    [0.0, y, z / 2.0],
                    [x, y, z / 2.0],
                ];
                let edges = mids
                    .iter()
                    .map(|&[a, b2, c]| edge_near(m, b, Point3::new(a, b2, c)))
                    .collect::<Result<Vec<_>, _>>()?;
                Ok(fillet(m, b, &edges, radius).map_err(fail)?.0)
            }
        }
    }

    /// The volume of the whole body offset by `d`.
    fn volume(&self, d: f64) -> f64 {
        match *self {
            Solid::Box { x, y, z } => (x + 2.0 * d) * (y + 2.0 * d) * (z + 2.0 * d),
            Solid::Cylinder { radius, height } => PI * (radius + d).powi(2) * (height + 2.0 * d),
            Solid::Rounded { x, y, z, radius } => {
                let (r, x, y, z) = (
                    radius + d,
                    x - 2.0 * radius,
                    y - 2.0 * radius,
                    z - 2.0 * radius,
                );
                x * y * z
                    + 2.0 * r * (x * y + y * z + z * x)
                    + PI * r * r * (x + y + z)
                    + 4.0 / 3.0 * PI * r.powi(3)
            }
        }
    }

    /// The area of the whole body offset by `d`.
    fn area(&self, d: f64) -> f64 {
        match *self {
            Solid::Box { x, y, z } => {
                let (x, y, z) = (x + 2.0 * d, y + 2.0 * d, z + 2.0 * d);
                2.0 * (x * y + y * z + z * x)
            }
            Solid::Cylinder { radius, height } => {
                let (r, h) = (radius + d, height + 2.0 * d);
                2.0 * PI * r * r + 2.0 * PI * r * h
            }
            Solid::Rounded { x, y, z, radius } => {
                let (r, x, y, z) = (
                    radius + d,
                    x - 2.0 * radius,
                    y - 2.0 * radius,
                    z - 2.0 * radius,
                );
                2.0 * (x * y + y * z + z * x) + 2.0 * PI * r * (x + y + z) + 4.0 * PI * r * r
            }
        }
    }
}

fn solid() -> impl Strategy<Value = Solid> {
    let side = || prop::finite_f64(1.0..=4.0);
    prop_oneof![
        (side(), side(), side()).prop_map(|(x, y, z)| Solid::Box { x, y, z }),
        (prop::finite_f64(0.5..=3.0), side())
            .prop_map(|(radius, height)| Solid::Cylinder { radius, height }),
        (side(), side(), side(), prop::finite_f64(0.1..=0.45)).prop_map(|(x, y, z, f)| {
            Solid::Rounded {
                x,
                y,
                z,
                radius: f * x.min(y).min(z),
            }
        }),
    ]
}

/// A signed distance as a fraction of the solid's limit inward and its
/// scale outward, never within a few percent of zero.
fn distance(solid: &Solid, fraction: f64) -> f64 {
    let f = if fraction.abs() < 0.03 {
        0.03_f64.copysign(fraction)
    } else {
        fraction
    };
    if f < 0.0 {
        f * solid.limit()
    } else {
        f * solid.scale()
    }
}

/// The edge of `body` whose curve's midpoint is nearest `at`, required
/// within `1e-9` of `at`'s distance from the origin (and of 1).
fn edge_near(m: &Model, body: Body, at: Point3) -> Result<Edge, TestCaseError> {
    let mut best: Option<(f64, Edge)> = None;
    for e in m.edges(body).map_err(fail)? {
        let entity = m.edge(e.id).map_err(fail)?;
        let Some((curve, range)) = entity.curve() else {
            continue;
        };
        let d = (m.curve(curve).map_err(fail)?.point(range.midpoint()) - at).norm();
        if best.is_none_or(|(b, _)| d < b) {
            best = Some((d, e));
        }
    }
    match best {
        Some((d, e)) if d <= 1e-9 * at.coords.norm().max(1.0) => Ok(e),
        _ => Err(fail(format!("no edge through {at}"))),
    }
}

/// The face of `body` that `at` lies inside.
fn face_at(m: &Model, body: Body, at: Point3) -> Result<Face, TestCaseError> {
    match classify_point(m, body, at).map_err(fail)? {
        Classification::On(s) => match s.id {
            EntityId::Face(id) => Ok(Face::forward(id)),
            other => Err(fail(format!("{at} is on {other}, not inside a face"))),
        },
        other => Err(fail(format!("{at} is {other:?}, on no face"))),
    }
}

fn assert_checked(report: &Report) -> Result<(), TestCaseError> {
    prop_assert!(report.is_ok(), "{}", report);
    prop_assert!(
        report.unchecked().is_empty(),
        "nothing unchecked\n{}",
        report
    );
    Ok(())
}

fn counts(m: &Model, body: Body) -> Result<[usize; 3], TestCaseError> {
    Ok([
        m.vertices(body).map_err(fail)?.len(),
        m.edges(body).map_err(fail)?.len(),
        m.faces(body).map_err(fail)?.len(),
    ])
}

/// `body` moved to `pose`, in the same model.
fn posed(m: &mut Model, body: Body, pose: &Isometry) -> Result<Body, TestCaseError> {
    Ok(transform(m, body, pose).map_err(fail)?.0)
}

/// `body`'s every face moved by `d`, clean at `Full`, its record audited.
fn offset_whole(m: &mut Model, body: Body, d: f64) -> Result<Body, TestCaseError> {
    let faces = m.faces(body).map_err(fail)?;
    let (out, p) = offset_faces(m, body, &faces, d)
        .map_err(|e| fail(format!("offset of every face by {d}: {e}")))?;
    assert_checked(&check(m, out, Level::Full))?;
    audit(m, &[body], out, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    Ok(out)
}

/// The two measures agree to `rel` with a floor of one.
fn same_measure(what: &str, got: f64, want: f64, rel: f64) -> Result<(), TestCaseError> {
    prop_assert!(
        close_to(got, want, 1.0, rel),
        "{}: {} vs {}",
        what,
        got,
        want
    );
    Ok(())
}

/// Every mass property of `a` equal to `b`'s, `b` having been moved by `by`
/// when given.
fn same_body(
    a: &MassProperties,
    b: &MassProperties,
    by: Option<&Isometry>,
) -> Result<(), TestCaseError> {
    same_measure("volume", a.volume, b.volume, REL)?;
    same_measure("area", a.area, b.area, REL)?;
    let c = by.map_or(b.centroid, |p| p.apply(b.centroid));
    prop_assert!(
        (a.centroid - c).norm() <= 1e-7 * a.centroid.coords.norm().max(1.0),
        "centroid {} vs {}",
        a.centroid,
        c
    );
    Ok(())
}

prop_shards! {
    /// The whole body offset by `d` is the closed form's volume and area,
    /// with the input's counts, in any pose.
    a_whole_offset_is_the_closed_form
        [shard_0 shard_1 shard_2 shard_3]
        ((solid, fraction, pose)) = (solid(), prop::finite_f64(-0.9..=2.0), prop::pose()) => {
            let d = distance(&solid, fraction);
            let mut m = Model::default();
            let rest = solid.build(&mut m)?;
            let body = posed(&mut m, rest, &pose)?;
            let before = counts(&m, body)?;
            let out = offset_whole(&mut m, body, d)?;
            prop_assert_eq!(counts(&m, out)?, before, "topology is kept");
            let props = mass_properties(&m, out).map_err(fail)?;
            same_measure("volume", props.volume, solid.volume(d), REL)?;
            same_measure("area", props.area, solid.area(d), REL)?;
            Ok(())
        }
}

prop_shards! {
    /// Offset by `d` then by `−d` returns the body's volume, area and
    /// counts.
    an_offset_and_its_inverse_return_the_body
        [shard_0 shard_1 shard_2 shard_3]
        ((solid, fraction, pose)) = (solid(), prop::finite_f64(-0.9..=2.0), prop::pose()) => {
            let d = distance(&solid, fraction);
            let mut m = Model::default();
            let rest = solid.build(&mut m)?;
            let body = posed(&mut m, rest, &pose)?;
            let was = mass_properties(&m, body).map_err(fail)?;
            let before = counts(&m, body)?;
            let there = offset_whole(&mut m, body, d)?;
            let back = offset_whole(&mut m, there, -d)?;
            prop_assert_eq!(counts(&m, back)?, before);
            same_body(&mass_properties(&m, back).map_err(fail)?, &was, None)?;
            Ok(())
        }
}

/// One face of a box: its axis and whether it is at the box's maximum.
type Side = (usize, bool);

/// A box and the faces of it moved together by `d`, as a mask over its six
/// sides.
#[derive(Debug, Clone)]
struct BoxCase {
    size: [f64; 3],
    mask: [bool; 6],
    fraction: f64,
    pose: Isometry,
}

impl BoxCase {
    fn sides(&self) -> Vec<Side> {
        (0..6)
            .filter(|&i| self.mask[i])
            .map(|i| (i / 2, i % 2 == 1))
            .collect()
    }

    /// The least distance the box allows: each axis keeps at least
    /// half of its extent.
    fn limit(&self) -> f64 {
        self.size[0].min(self.size[1]).min(self.size[2]) / 2.0
    }

    fn d(&self) -> f64 {
        let f = if self.fraction.abs() < 0.03 {
            0.03_f64.copysign(self.fraction)
        } else {
            self.fraction
        };
        if f < 0.0 {
            f * self.limit()
        } else {
            f * self.size[0].min(self.size[1]).min(self.size[2])
        }
    }

    /// The box after the sides move by `d`: its `(lo, hi)` corners.
    fn moved(&self, d: f64) -> ([f64; 3], [f64; 3]) {
        let (mut lo, mut hi) = ([0.0; 3], self.size);
        for (axis, max) in self.sides() {
            if max {
                hi[axis] += d;
            } else {
                lo[axis] -= d;
            }
        }
        (lo, hi)
    }

    /// The point at the middle of side `(axis, max)` of the box `(lo, hi)`.
    fn face_point(lo: [f64; 3], hi: [f64; 3], (axis, max): Side) -> Point3 {
        let mut p = [0.0; 3];
        for i in 0..3 {
            p[i] = if i == axis {
                if max { hi[i] } else { lo[i] }
            } else {
                (lo[i] + hi[i]) / 2.0
            };
        }
        Point3::new(p[0], p[1], p[2])
    }

    fn volume(lo: [f64; 3], hi: [f64; 3]) -> f64 {
        (0..3).map(|i| hi[i] - lo[i]).product()
    }

    fn area(lo: [f64; 3], hi: [f64; 3]) -> f64 {
        let e: Vec<f64> = (0..3).map(|i| hi[i] - lo[i]).collect();
        2.0 * (e[0] * e[1] + e[1] * e[2] + e[2] * e[0])
    }
}

fn box_case() -> impl Strategy<Value = BoxCase> {
    let side = || prop::finite_f64(1.0..=4.0);
    (
        [side(), side(), side()],
        proptest::collection::vec(any::<bool>(), 6),
        prop::finite_f64(-0.9..=2.0),
        prop::pose(),
    )
        .prop_map(|(size, mask, fraction, pose)| {
            let mut mask: [bool; 6] = mask.try_into().unwrap_or([true; 6]);
            if !mask.iter().any(|&b| b) {
                mask[0] = true;
            }
            BoxCase {
                size,
                mask,
                fraction,
                pose,
            }
        })
}

/// The faces of the box `(lo, hi)` at `sides`, found in its posed body.
fn box_faces(
    m: &Model,
    body: Body,
    pose: &Isometry,
    (lo, hi): ([f64; 3], [f64; 3]),
    sides: &[Side],
) -> Result<Vec<Face>, TestCaseError> {
    sides
        .iter()
        .map(|&s| face_at(m, body, pose.apply(BoxCase::face_point(lo, hi, s))))
        .collect()
}

/// A box with some of its sides moved by `d`.
fn pushed_box(
    m: &mut Model,
    case: &BoxCase,
    pose: &Isometry,
) -> Result<(Body, Body, f64), TestCaseError> {
    let d = case.d();
    let [x, y, z] = case.size;
    let rest = primitive_box(m, Point3::origin(), Point3::new(x, y, z))
        .map_err(fail)?
        .0;
    let body = posed(m, rest, pose)?;
    let faces = box_faces(m, body, pose, ([0.0; 3], case.size), &case.sides())?;
    let (out, p) = offset_faces(m, body, &faces, d)
        .map_err(|e| fail(format!("{:?} moved by {d}: {e}", case.sides())))?;
    assert_checked(&check(m, out, Level::Full))?;
    audit(m, &[body], out, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    Ok((body, out, d))
}

prop_shards! {
    /// Any set of a box's sides moved together is the box with those sides
    /// moved, sharp at every join, and moving them back returns the box.
    a_box_with_sides_moved_is_the_box_with_sides_moved
        [shard_0 shard_1 shard_2 shard_3]
        (case) = box_case() => {
            let mut m = Model::default();
            let (body, out, d) = pushed_box(&mut m, &case, &case.pose)?;
            let (lo, hi) = case.moved(d);
            let props = mass_properties(&m, out).map_err(fail)?;
            same_measure("volume", props.volume, BoxCase::volume(lo, hi), REL)?;
            same_measure("area", props.area, BoxCase::area(lo, hi), REL)?;
            prop_assert_eq!(counts(&m, out)?, counts(&m, body)?);
            // And back: the same sides, at their new places, moved by -d.
            let sides = case.sides();
            let faces = box_faces(&m, out, &case.pose, (lo, hi), &sides)?;
            let (home, _) = offset_faces(&mut m, out, &faces, -d)
                .map_err(|e| fail(format!("moved back by {}: {e}", -d)))?;
            assert_checked(&check(&m, home, Level::Full))?;
            same_body(
                &mass_properties(&m, home).map_err(fail)?,
                &mass_properties(&m, body).map_err(fail)?,
                None,
            )?;
            Ok(())
        }
}

prop_shards! {
    /// Moving then offsetting is offsetting then moving: the same volume,
    /// area, counts and centroid, the pose applied.
    an_offset_commutes_with_a_transform
        [shard_0 shard_1 shard_2 shard_3]
        ((case, pose)) = (box_case(), prop::pose()) => {
            // Offset at rest, then moved.
            let mut m = Model::default();
            let (_, rest, _) = pushed_box(&mut m, &case, &Isometry::identity())?;
            let moved_after = posed(&mut m, rest, &pose)?;
            // Moved, then offset, in its own model.
            let mut n = Model::default();
            let (_, offset_after, _) = pushed_box(&mut n, &case, &pose)?;
            prop_assert_eq!(counts(&m, moved_after)?, counts(&n, offset_after)?);
            let a = mass_properties(&m, moved_after).map_err(fail)?;
            let b = mass_properties(&n, offset_after).map_err(fail)?;
            same_body(&a, &b, None)?;
            Ok(())
        }
}

prop_shards! {
    /// Whole solids offset in a pose and at rest, one moved after the
    /// other: the same measures.
    a_whole_offset_commutes_with_a_transform
        [shard_0 shard_1 shard_2 shard_3]
        ((solid, fraction, pose)) = (solid(), prop::finite_f64(-0.9..=2.0), prop::pose()) => {
            let d = distance(&solid, fraction);
            let mut m = Model::default();
            let rest = solid.build(&mut m)?;
            let grown = offset_whole(&mut m, rest, d)?;
            let moved_after = posed(&mut m, grown, &pose)?;
            let mut n = Model::default();
            let rest = solid.build(&mut n)?;
            let body = posed(&mut n, rest, &pose)?;
            let offset_after = offset_whole(&mut n, body, d)?;
            prop_assert_eq!(counts(&m, moved_after)?, counts(&n, offset_after)?);
            same_body(
                &mass_properties(&m, moved_after).map_err(fail)?,
                &mass_properties(&n, offset_after).map_err(fail)?,
                None,
            )?;
            Ok(())
        }
}

/// A right prism: a box, a cylinder, or the extruded L
/// `(0,0)–(x,0)–(x,arm_y)–(arm_x,arm_y)–(arm_x,y)–(0,y)`, its walls
/// perpendicular to one another and to its caps.
#[derive(Debug, Clone, PartialEq)]
enum Wall {
    Box,
    Cylinder,
    Ell { arm_x: f64, arm_y: f64 },
}

/// A face of a prism to push by `fraction` of what it allows.
#[derive(Debug, Clone)]
struct WallCase {
    wall: Wall,
    size: [f64; 3],
    face: usize,
    fraction: f64,
    pose: Isometry,
}

impl WallCase {
    fn polygon(&self) -> Vec<Point2> {
        let [x, y, _] = self.size;
        let p = Point2::new;
        match self.wall {
            Wall::Ell { arm_x, arm_y } => vec![
                p(0.0, 0.0),
                p(x, 0.0),
                p(x, arm_y),
                p(arm_x, arm_y),
                p(arm_x, y),
                p(0.0, y),
            ],
            Wall::Box | Wall::Cylinder => vec![p(0.0, 0.0), p(x, 0.0), p(x, y), p(0.0, y)],
        }
    }

    fn build(&self, m: &mut Model) -> Result<Body, TestCaseError> {
        let [x, y, z] = self.size;
        match self.wall {
            Wall::Box => Ok(primitive_box(m, Point3::origin(), Point3::new(x, y, z))
                .map_err(fail)?
                .0),
            Wall::Cylinder => {
                let r = x.min(y) / 2.0;
                Ok(primitive_cylinder(m, Axis::z_at(Point3::origin()), r, z)
                    .map_err(fail)?
                    .0)
            }
            Wall::Ell { .. } => {
                let poly = self.polygon();
                let profile = Profile {
                    plane: Frame::world(),
                    outer: ProfileLoop::Path {
                        start: poly[0],
                        segments: poly[1..]
                            .iter()
                            .chain(core::iter::once(&poly[0]))
                            .map(|&q| ProfileSegment::LineTo(q))
                            .collect(),
                    },
                    holes: Vec::new(),
                };
                Ok(extrude(m, &profile, Vec3::z(), z).map_err(fail)?.0)
            }
        }
    }

    /// The faces that can be pushed and, for each, a point inside it, its
    /// area and the shortest edge it has — which `|d|` stays under half of
    /// when pulled.
    fn faces(&self) -> Vec<(Point3, f64, (f64, f64))> {
        let [x, y, z] = self.size;
        match self.wall {
            Wall::Cylinder => {
                let r = x.min(y) / 2.0;
                // The caps; the side's seam and its tangent rim keep it out.
                let cap = PI * r * r;
                vec![
                    (Point3::new(0.3 * r, 0.2 * r, 0.0), cap, (z / 2.0, 3.0 * r)),
                    (Point3::new(0.3 * r, 0.2 * r, z), cap, (z / 2.0, 3.0 * r)),
                ]
            }
            Wall::Box | Wall::Ell { .. } => {
                let poly = self.polygon();
                let n = poly.len();
                let shortest = (0..n)
                    .map(|k| (poly[(k + 1) % n] - poly[k]).norm())
                    .fold(f64::INFINITY, f64::min);
                let mut faces = Vec::new();
                for k in 0..n {
                    let (a, b) = (poly[k], poly[(k + 1) % n]);
                    let mid = Point2::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0);
                    let length = (b - a).norm();
                    // A wall's neighbours are perpendicular: pulled in, it takes
                    // at most half the shortest wall; pushed out, a reflex
                    // neighbour shortens, so the same bound holds.
                    faces.push((
                        Point3::new(mid.x, mid.y, z / 2.0),
                        length * z,
                        (shortest / 2.0, shortest / 2.0),
                    ));
                }
                let area = polygon_area(&poly);
                let inside = Point2::new(
                    poly.iter().map(|p| p.x).sum::<f64>() / n as f64,
                    poly.iter().map(|p| p.y).sum::<f64>() / n as f64,
                );
                // The L's centroid of vertices can fall outside; its first
                // arm's middle is inside.
                let inside = match self.wall {
                    Wall::Ell { arm_y, .. } => Point2::new(x / 2.0, arm_y / 2.0),
                    _ => inside,
                };
                faces.push((
                    Point3::new(inside.x, inside.y, 0.0),
                    area,
                    (z / 2.0, 3.0 * x.min(y)),
                ));
                faces.push((
                    Point3::new(inside.x, inside.y, z),
                    area,
                    (z / 2.0, 3.0 * x.min(y)),
                ));
                faces
            }
        }
    }

    fn volume(&self) -> f64 {
        let [x, y, z] = self.size;
        match self.wall {
            Wall::Cylinder => PI * (x.min(y) / 2.0).powi(2) * z,
            Wall::Box | Wall::Ell { .. } => polygon_area(&self.polygon()) * z,
        }
    }
}

fn polygon_area(poly: &[Point2]) -> f64 {
    let n = poly.len();
    (0..n)
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % n]);
            a.x * b.y - b.x * a.y
        })
        .sum::<f64>()
        .abs()
        / 2.0
}

fn wall_case() -> impl Strategy<Value = WallCase> {
    let side = || prop::finite_f64(1.0..=4.0);
    let arm = || prop::finite_f64(0.3..=0.7);
    (
        prop_oneof![
            Just((Wall::Box, 0.0, 0.0)),
            Just((Wall::Cylinder, 0.0, 0.0)),
            (arm(), arm()).prop_map(|(fx, fy)| (
                Wall::Ell {
                    arm_x: 0.0,
                    arm_y: 0.0
                },
                fx,
                fy
            )),
        ],
        [side(), side(), side()],
        0usize..8,
        prop::finite_f64(-0.9..=2.0),
        prop::pose(),
    )
        .prop_map(|((wall, fx, fy), size, face, fraction, pose)| {
            let wall = match wall {
                Wall::Ell { .. } => Wall::Ell {
                    arm_x: fx * size[0],
                    arm_y: fy * size[1],
                },
                other => other,
            };
            WallCase {
                wall,
                size,
                face,
                fraction,
                pose,
            }
        })
}

prop_shards! {
    /// A face perpendicular to its neighbours pushed by `d` changes the
    /// volume by its area times `d`.
    a_perpendicular_wall_pushed_changes_the_volume_by_area_times_distance
        [shard_0 shard_1 shard_2 shard_3]
        (case) = wall_case() => {
            let faces = case.faces();
            let (at, area, limits) = faces[case.face % faces.len()];
            let f = if case.fraction.abs() < 0.03 { 0.03_f64.copysign(case.fraction) } else { case.fraction };
            let d = f * if f < 0.0 { limits.0 } else { limits.1 };
            let mut m = Model::default();
            let rest = case.build(&mut m)?;
            let body = posed(&mut m, rest, &case.pose)?;
            let face = face_at(&m, body, case.pose.apply(at))?;
            let (out, p) = offset_faces(&mut m, body, &[face], d)
                .map_err(|e| fail(format!("face through {at} by {d}: {e}")))?;
            assert_checked(&check(&m, out, Level::Full))?;
            audit(&m, &[body], out, &p).map_err(|e| fail(format!("provenance: {e}")))?;
            let props = mass_properties(&m, out).map_err(fail)?;
            let want = case.volume() + area * d;
            same_measure("volume", props.volume, want, REL)?;
            Ok(())
        }
}

/// A box with its top pushed by `rise`, a bore of `bore` through its
/// middle, and the size of the blend on the pushed top's front edge.
#[derive(Debug, Clone)]
struct Operand {
    size: [f64; 3],
    rise: f64,
    bore: f64,
    blend: f64,
    pose: Isometry,
}

fn operand() -> impl Strategy<Value = Operand> {
    let side = || prop::finite_f64(2.0..=4.0);
    (
        [side(), side(), side()],
        prop::finite_f64(0.2..=1.5),
        prop::finite_f64(0.1..=0.4),
        prop::finite_f64(0.05..=0.4),
        prop::pose(),
    )
        .prop_map(|(size, rise, bore, blend, pose)| Operand {
            size,
            rise,
            bore: bore * size[0].min(size[1]),
            blend: blend * size[2],
            pose,
        })
}

prop_shards! {
    /// A pushed box is an operand of fillet, chamfer and cut: the result of
    /// each is clean at `Full`, and each changes the volume by its closed
    /// form.
    an_offset_result_is_an_operand_of_fillet_chamfer_and_cut
        [shard_0 shard_1 shard_2 shard_3]
        (case) = operand() => {
            let [x, y, z] = case.size;
            let mut m = Model::default();
            let rest = primitive_box(&mut m, Point3::origin(), Point3::new(x, y, z))
                .map_err(fail)?
                .0;
            let body = posed(&mut m, rest, &case.pose)?;
            let top = face_at(&m, body, case.pose.apply(Point3::new(x / 2.0, y / 2.0, z)))?;
            let (tall, _) = offset_faces(&mut m, body, &[top], case.rise).map_err(fail)?;
            let height = z + case.rise;
            let volume = x * y * height;
            // The pushed top's front rim, along x at y = 0.
            let rim = edge_near(&m, tall, case.pose.apply(Point3::new(x / 2.0, 0.0, height)))?;
            let (rounded, _) = fillet(&mut m, tall, &[rim], case.blend)
                .map_err(|e| fail(format!("fillet of the pushed box: {e}")))?;
            assert_checked(&check(&m, rounded, Level::Full))?;
            let got = mass_properties(&m, rounded).map_err(fail)?.volume;
            same_measure("fillet volume", got, volume - (1.0 - PI / 4.0) * case.blend.powi(2) * x, REL)?;
            let (cut_edge, _) = chamfer(&mut m, tall, &[rim], case.blend)
                .map_err(|e| fail(format!("chamfer of the pushed box: {e}")))?;
            assert_checked(&check(&m, cut_edge, Level::Full))?;
            let got = mass_properties(&m, cut_edge).map_err(fail)?.volume;
            same_measure("chamfer volume", got, volume - 0.5 * case.blend.powi(2) * x, REL)?;
            // A bore through the whole height, in the middle.
            let axis = Axis::z_at(Point3::new(x / 2.0, y / 2.0, -1.0));
            let (tool, _) = primitive_cylinder(&mut m, axis, case.bore, height + 2.0).map_err(fail)?;
            let tool = posed(&mut m, tool, &case.pose)?;
            let (bored, _) = cut(&mut m, tall, tool)
                .map_err(|e: OpError| fail(format!("cut of the pushed box: {e}")))?;
            assert_checked(&check(&m, bored, Level::Full))?;
            let got = mass_properties(&m, bored).map_err(fail)?.volume;
            same_measure("cut volume", got, volume - PI * case.bore.powi(2) * height, REL)?;
            Ok(())
        }
}

prop_shards! {
    /// The whole offset of a rounded box — blends among its faces — is an
    /// operand of a cut: a bore through its middle takes the cylinder's
    /// volume off, clean at `Full`.
    a_rounded_offset_is_an_operand_of_a_cut
        [shard_0 shard_1 shard_2 shard_3]
        ((solid, fraction, pose)) = (
            (
                prop::finite_f64(2.0..=4.0),
                prop::finite_f64(2.0..=4.0),
                prop::finite_f64(2.0..=4.0),
                prop::finite_f64(0.1..=0.3),
            )
                .prop_map(|(x, y, z, f)| Solid::Rounded { x, y, z, radius: f * x.min(y).min(z) }),
            prop::finite_f64(-0.5..=1.0),
            prop::pose(),
        ) => {
            let Solid::Rounded { x, y, z, .. } = solid else {
                return Err(fail("a rounded box"));
            };
            let d = distance(&solid, fraction);
            let mut m = Model::default();
            let rest = solid.build(&mut m)?;
            let grown = offset_whole(&mut m, rest, d)?;
            // A bore through z, narrower than the flat of the top.
            let bore = 0.15 * x.min(y);
            let axis = Axis::z_at(Point3::new(x / 2.0, y / 2.0, -d - 1.0));
            let height = z + 2.0 * d;
            let (tool, _) = primitive_cylinder(&mut m, axis, bore, height + 2.0).map_err(fail)?;
            let (bored, _) = cut(&mut m, grown, tool)
                .map_err(|e| fail(format!("cut of the offset rounded box: {e}")))?;
            assert_checked(&check(&m, bored, Level::Full))?;
            let got = mass_properties(&m, bored).map_err(fail)?.volume;
            same_measure("volume", got, solid.volume(d) - PI * bore * bore * height, REL)?;
            let moved = posed(&mut m, bored, &pose)?;
            assert_checked(&check(&m, moved, Level::Full))?;
            Ok(())
        }
}

/// A cylinder tilted and 22 below the origin, every face offset by 0.03:
/// the rim circles' full-turn range was `t + period` rounded a ulp past
/// one period, which the checker's E1 refuses. Found by
/// `a_whole_offset_is_the_closed_form` at the default seed and case count
/// (shard 2).
#[test]
fn a_posed_cylinder_keeps_its_rim_ranges_within_a_period() {
    use arris_math::nalgebra::{Quaternion, UnitQuaternion};
    let turn = UnitQuaternion::from_quaternion(Quaternion::new(
        -0.03549463608741306,
        0.0,
        0.8318338481779844,
        -0.5538885987582955,
    ));
    let pose = Isometry::new(turn, Vec3::new(0.0, 0.0, -22.005811252262625));
    let solid = Solid::Cylinder {
        radius: 2.2384404102467914,
        height: 1.0,
    };
    let mut m = Model::default();
    let rest = solid.build(&mut m).unwrap();
    let body = posed(&mut m, rest, &pose).unwrap();
    let out = offset_whole(&mut m, body, 0.03).unwrap();
    let props = mass_properties(&m, out).unwrap();
    assert!(close_to(props.volume, solid.volume(0.03), 1.0, REL));
}
