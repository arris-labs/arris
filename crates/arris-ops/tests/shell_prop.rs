//! `ops::shell` at random (ADR-0049): a box, a cylinder and a box with its
//! four vertical edges rounded, of random proportions, in random poses,
//! open on one face or closed, hollowed inward or outward.
//!
//! The volume is the closed form — the body less its inset by the
//! thickness, or its outset less the body, every face but the opening
//! moved — and inward, the shell and `offset_faces` of every face but the
//! openings by the negated thickness add up to the body; a closed void
//! is a second shell; shelling then moving is moving then shelling; every
//! wall's skin copy lies at the thickness from it; and the result is an
//! operand of fillet, chamfer, cut and `offset_faces`, clean at `Full`.
//! A failure prints the case and the seed, and becomes a fixture under
//! `tests/fixtures/regression/` (`tests/fixtures/README.md` §Property-test
//! failures).

use core::f64::consts::PI;

use arris_check::classify::{Classification, classify_point};
use arris_check::{Level, Report, check};
use arris_debug::testing::{REL, close_to, fail};
use arris_debug::unmetered::{
    chamfer, cut, fillet, mass_properties, offset_faces, primitive_box, primitive_cylinder, shell,
    transform,
};
use arris_debug::{prop, prop_shards};
use arris_math::{Axis, Isometry, Point3};
use arris_ops::ShellSide;
use arris_topo::provenance::audit;
use arris_topo::{Body, Edge, EntityId, Face, FaceId, Model};
use proptest::prelude::*;

/// What the properties hollow, before its pose.
#[derive(Debug, Clone, PartialEq)]
enum Solid {
    /// The box from the origin to `(x, y, z)`.
    Box { x: f64, y: f64, z: f64 },
    /// The cylinder on the `z` axis from the origin, `height` tall.
    Cylinder { radius: f64, height: f64 },
    /// The box `(x, y, z)` with its four edges along `z` filleted at
    /// `radius`: its top and bottom meet the blends at a right angle, its
    /// sides are tangent to them.
    Rounded { x: f64, y: f64, z: f64, radius: f64 },
}

/// A face of a solid by where it lies: its axis and whether it is at the
/// maximum. A cylinder's and a rounded box's openings are their caps,
/// along `z`; a rounded box's side is tangent to a wall, which a shell
/// refuses by design.
type Side = (usize, bool);

impl Solid {
    fn extents(&self) -> [f64; 3] {
        match *self {
            Solid::Box { x, y, z } | Solid::Rounded { x, y, z, .. } => [x, y, z],
            Solid::Cylinder { radius, height } => [2.0 * radius, 2.0 * radius, height],
        }
    }

    /// The thickness past which an inward wall collapses or vanishes.
    fn limit(&self) -> f64 {
        match *self {
            Solid::Box { x, y, z } => x.min(y).min(z) / 2.0,
            Solid::Cylinder { radius, height } => radius.min(height / 2.0),
            Solid::Rounded { radius, z, .. } => radius.min(z / 2.0),
        }
    }

    /// The faces that may be opened.
    fn openable(&self) -> Vec<Side> {
        match self {
            Solid::Box { .. } => (0..6).map(|i| (i / 2, i % 2 == 1)).collect(),
            Solid::Cylinder { .. } | Solid::Rounded { .. } => vec![(2, false), (2, true)],
        }
    }

    /// A point inside face `side`, at rest.
    fn face_point(&self, (axis, max): Side) -> Point3 {
        let e = self.extents();
        let mut p = match self {
            Solid::Cylinder { radius, .. } => [0.3 * radius, 0.2 * radius, 0.0],
            Solid::Box { .. } | Solid::Rounded { .. } => [e[0] / 2.0, e[1] / 2.0, e[2] / 2.0],
        };
        p[axis] = if max { e[axis] } else { 0.0 };
        Point3::new(p[0], p[1], p[2])
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
                let edges = [[0.0, 0.0], [x, 0.0], [x, y], [0.0, y]]
                    .iter()
                    .map(|&[a, c]| edge_near(m, b, Point3::new(a, c, z / 2.0)))
                    .collect::<Result<Vec<_>, _>>()?;
                Ok(fillet(m, b, &edges, radius).map_err(fail)?.0)
            }
        }
    }

    /// The volume of the body with every face but `open` moved out by `s`
    /// (in, for a negative `s`).
    fn volume_moved(&self, s: f64, open: Option<Side>) -> f64 {
        // Along an axis with the opening on it, one face moves, not two.
        let along = |axis: usize, e: f64| {
            let faces = if open.is_some_and(|(a, _)| a == axis) {
                1.0
            } else {
                2.0
            };
            e + faces * s
        };
        match *self {
            Solid::Box { x, y, z } => along(0, x) * along(1, y) * along(2, z),
            Solid::Cylinder { radius, height } => PI * (radius + s).powi(2) * along(2, height),
            Solid::Rounded { x, y, z, radius } => {
                let (x, y, r) = (x + 2.0 * s, y + 2.0 * s, radius + s);
                (x * y - (4.0 - PI) * r * r) * along(2, z)
            }
        }
    }

    /// The closed form of the shell's volume.
    fn shell_volume(&self, t: f64, side: ShellSide, open: Option<Side>) -> f64 {
        let body = self.volume_moved(0.0, None);
        match side {
            ShellSide::Inward => body - self.volume_moved(-t, open),
            ShellSide::Outward => self.volume_moved(t, open) - body,
        }
    }
}

/// A shell as drawn: the solid, its opening, the thickness as a fraction
/// of what the side allows, the side and the pose.
#[derive(Debug, Clone)]
struct Case {
    solid: Solid,
    open: Option<Side>,
    fraction: f64,
    side: ShellSide,
    pose: Isometry,
}

impl Case {
    /// Inward, a fraction of the solid's limit; outward, of its least
    /// extent.
    fn thickness(&self) -> f64 {
        match self.side {
            ShellSide::Inward => self.fraction * self.solid.limit(),
            ShellSide::Outward => {
                let e = self.solid.extents();
                self.fraction * e[0].min(e[1]).min(e[2])
            }
        }
    }
}

fn solid() -> impl Strategy<Value = Solid> {
    let side = || prop::finite_f64(1.0..=4.0);
    prop_oneof![
        (side(), side(), side()).prop_map(|(x, y, z)| Solid::Box { x, y, z }),
        (prop::finite_f64(0.5..=2.0), side())
            .prop_map(|(radius, height)| Solid::Cylinder { radius, height }),
        (side(), side(), side(), prop::finite_f64(0.1..=0.45)).prop_map(|(x, y, z, f)| {
            Solid::Rounded {
                x,
                y,
                z,
                radius: f * x.min(y),
            }
        }),
    ]
}

/// A case on `solid`, an opening a quarter of the time none.
fn case_on(solid: impl Strategy<Value = Solid>) -> impl Strategy<Value = Case> {
    (
        solid,
        proptest::option::weighted(0.75, 0usize..6),
        prop::finite_f64(0.05..=0.9),
        any::<bool>(),
        prop::pose(),
    )
        .prop_map(|(solid, open, fraction, outward, pose)| {
            let sides = solid.openable();
            Case {
                open: open.map(|k| sides[k % sides.len()]),
                solid,
                fraction,
                side: if outward {
                    ShellSide::Outward
                } else {
                    ShellSide::Inward
                },
                pose,
            }
        })
}

fn case() -> impl Strategy<Value = Case> {
    case_on(solid())
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

fn counts(m: &Model, body: Body) -> Result<[usize; 4], TestCaseError> {
    Ok([
        m.vertices(body).map_err(fail)?.len(),
        m.edges(body).map_err(fail)?.len(),
        m.faces(body).map_err(fail)?.len(),
        m.shells(body).map_err(fail)?.len(),
    ])
}

/// `body` moved to `pose`, in the same model.
fn posed(m: &mut Model, body: Body, pose: &Isometry) -> Result<Body, TestCaseError> {
    Ok(transform(m, body, pose).map_err(fail)?.0)
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

/// The case's solid built in its pose, and its opening found in it.
fn posed_solid(
    m: &mut Model,
    case: &Case,
    pose: &Isometry,
) -> Result<(Body, Vec<Face>), TestCaseError> {
    let rest = case.solid.build(m)?;
    let body = posed(m, rest, pose)?;
    let openings = case
        .open
        .map(|s| face_at(m, body, pose.apply(case.solid.face_point(s))))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
    Ok((body, openings))
}

/// The case shelled in `pose`, clean at `Full`, its record audited.
fn shelled(
    m: &mut Model,
    case: &Case,
    pose: &Isometry,
) -> Result<(Body, Body, Vec<Face>, arris_topo::Provenance), TestCaseError> {
    let (body, openings) = posed_solid(m, case, pose)?;
    let t = case.thickness();
    let (out, p) = shell(m, body, &openings, t, case.side)
        .map_err(|e| fail(format!("{:?} shelled by {t}: {e}", case.side)))?;
    assert_checked(&check(m, out, Level::Full))?;
    audit(m, &[body], out, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    Ok((body, out, openings, p))
}

/// The corners of `face`: every vertex its loops pass through.
fn face_vertices(m: &Model, face: FaceId) -> Result<Vec<Point3>, TestCaseError> {
    let mut points = Vec::new();
    for l in m.face(face).map_err(fail)?.loops() {
        for c in l.coedges() {
            let e = m.edge(c.edge()).map_err(fail)?;
            points.push(m.vertex(e.start()).map_err(fail)?.point());
        }
    }
    Ok(points)
}

prop_shards! {
    /// The shell's volume is the closed form, on either side, open or
    /// closed, in any pose; a closed one has a second shell.
    a_shell_is_the_closed_form
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let mut m = Model::default();
            let (_, out, _, _) = shelled(&mut m, &case, &case.pose)?;
            let props = mass_properties(&m, out).map_err(fail)?;
            let want = case.solid.shell_volume(case.thickness(), case.side, case.open);
            same_measure("volume", props.volume, want, REL)?;
            let shells = m.shells(out).map_err(fail)?.len();
            prop_assert_eq!(shells, if case.open.is_some() { 1 } else { 2 });
            Ok(())
        }
}

prop_shards! {
    /// Inward, the shell and `offset_faces` of every face but its openings
    /// by the negated thickness add up to the body.
    a_shell_and_its_cavity_add_up_to_the_body
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let case = Case { side: ShellSide::Inward, ..case };
            let mut m = Model::default();
            let (body, out, openings, _) = shelled(&mut m, &case, &case.pose)?;
            let walls: Vec<Face> = m
                .faces(body)
                .map_err(fail)?
                .into_iter()
                .filter(|f| openings.iter().all(|o| o.id != f.id))
                .collect();
            let t = case.thickness();
            let (cavity, _) = offset_faces(&mut m, body, &walls, -t)
                .map_err(|e| fail(format!("the cavity, every wall by {}: {e}", -t)))?;
            let whole = mass_properties(&m, body).map_err(fail)?.volume;
            let sum = mass_properties(&m, out).map_err(fail)?.volume
                + mass_properties(&m, cavity).map_err(fail)?.volume;
            same_measure("shell + cavity", sum, whole, REL)?;
            Ok(())
        }
}

prop_shards! {
    /// Shelling then moving is moving then shelling: the same counts,
    /// volume, area and centroid.
    a_shell_commutes_with_a_transform
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let mut m = Model::default();
            let (_, rest, _, _) = shelled(&mut m, &case, &Isometry::identity())?;
            let moved_after = posed(&mut m, rest, &case.pose)?;
            let mut n = Model::default();
            let (_, shelled_after, _, _) = shelled(&mut n, &case, &case.pose)?;
            prop_assert_eq!(counts(&m, moved_after)?, counts(&n, shelled_after)?);
            let a = mass_properties(&m, moved_after).map_err(fail)?;
            let b = mass_properties(&n, shelled_after).map_err(fail)?;
            same_measure("volume", a.volume, b.volume, REL)?;
            same_measure("area", a.area, b.area, REL)?;
            prop_assert!(
                (a.centroid - b.centroid).norm() <= 1e-7 * a.centroid.coords.norm().max(1.0),
                "centroid {} vs {}",
                a.centroid,
                b.centroid
            );
            Ok(())
        }
}

prop_shards! {
    /// Every wall's skin copy lies at the thickness from it: each corner
    /// of the wall is that far from the copy's surface, and each corner
    /// of the copy that far from the wall's.
    every_wall_is_the_thickness_from_its_skin
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let mut m = Model::default();
            let (body, out, openings, p) = shelled(&mut m, &case, &case.pose)?;
            let t = case.thickness();
            let tol = 1e-9 * case.pose.translation().norm().max(10.0);
            let walls: Vec<Face> = m
                .faces(body)
                .map_err(fail)?
                .into_iter()
                .filter(|f| openings.iter().all(|o| o.id != f.id))
                .collect();
            let out_faces = m.faces(out).map_err(fail)?;
            for wall in walls {
                let copies: Vec<FaceId> = p
                    .generated_from(Face::forward(wall.id).shape())
                    .iter()
                    .filter_map(|s| match s.id {
                        EntityId::Face(id) => Some(id),
                        _ => None,
                    })
                    .collect();
                prop_assert_eq!(copies.len(), 1, "one skin copy of {:?}", wall.id);
                let copy = copies[0];
                prop_assert!(out_faces.iter().any(|f| f.id == copy), "the copy is the result's");
                let surface_of = |f: FaceId| -> Result<arris_geom::Surface, TestCaseError> {
                    Ok(m.surface(m.face(f).map_err(fail)?.surface()).map_err(fail)?.clone())
                };
                let (wall_surface, copy_surface) = (surface_of(wall.id)?, surface_of(copy)?);
                for (from, onto) in [(wall.id, &copy_surface), (copy, &wall_surface)] {
                    for q in face_vertices(&m, from)? {
                        let d = onto.project(q).map_err(fail)?.distance;
                        prop_assert!((d - t).abs() <= tol, "{} from the other skin, not {}", d, t);
                    }
                }
            }
            Ok(())
        }
}

/// A box hollowed inward, open on top, and the size of a blend on one of
/// its outer upright edges, a bore through its floor and a push of its
/// bottom, as fractions.
#[derive(Debug, Clone)]
struct Operand {
    size: [f64; 3],
    wall: f64,
    blend: f64,
    bore: f64,
    push: f64,
    pose: Isometry,
}

fn operand() -> impl Strategy<Value = Operand> {
    let side = || prop::finite_f64(2.0..=4.0);
    (
        [side(), side(), side()],
        prop::finite_f64(0.1..=0.4),
        prop::finite_f64(0.1..=0.8),
        prop::finite_f64(0.1..=0.4),
        prop::finite_f64(0.1..=1.0),
        prop::pose(),
    )
        .prop_map(|(size, wall, blend, bore, push, pose)| Operand {
            wall: wall * size[0].min(size[1]).min(size[2]) / 2.0,
            blend,
            bore,
            push,
            size,
            pose,
        })
}

prop_shards! {
    /// A box hollowed inward and open on top is an operand of fillet,
    /// chamfer, cut and `offset_faces`: the result of each is clean at
    /// `Full`, and each changes the volume by its closed form.
    a_shell_is_an_operand_of_fillet_chamfer_cut_and_offset
        [shard_0 shard_1 shard_2 shard_3]
        (case) = operand() => {
            let [x, y, z] = case.size;
            let t = case.wall;
            let mut m = Model::default();
            let rest = primitive_box(&mut m, Point3::origin(), Point3::new(x, y, z))
                .map_err(fail)?
                .0;
            let body = posed(&mut m, rest, &case.pose)?;
            let top = face_at(&m, body, case.pose.apply(Point3::new(x / 2.0, y / 2.0, z)))?;
            let (cup, _) = shell(&mut m, body, &[top], t, ShellSide::Inward).map_err(fail)?;
            let volume = x * y * z - (x - 2.0 * t) * (y - 2.0 * t) * (z - t);
            // An outer upright edge, its blend within the wall.
            let corner = edge_near(&m, cup, case.pose.apply(Point3::new(0.0, 0.0, z / 2.0)))?;
            let b = case.blend * t;
            let (rounded, _) = fillet(&mut m, cup, &[corner], b)
                .map_err(|e| fail(format!("fillet of the shell: {e}")))?;
            assert_checked(&check(&m, rounded, Level::Full))?;
            let got = mass_properties(&m, rounded).map_err(fail)?.volume;
            same_measure("fillet volume", got, volume - (1.0 - PI / 4.0) * b * b * z, REL)?;
            let (cut_edge, _) = chamfer(&mut m, cup, &[corner], b)
                .map_err(|e| fail(format!("chamfer of the shell: {e}")))?;
            assert_checked(&check(&m, cut_edge, Level::Full))?;
            let got = mass_properties(&m, cut_edge).map_err(fail)?.volume;
            same_measure("chamfer volume", got, volume - 0.5 * b * b * z, REL)?;
            // A bore through the floor, its top end in the cavity.
            let r = case.bore * (x.min(y) / 2.0 - t);
            let axis = Axis::z_at(Point3::new(x / 2.0, y / 2.0, -1.0));
            let (tool, _) = primitive_cylinder(&mut m, axis, r, 1.0 + z / 2.0).map_err(fail)?;
            let tool = posed(&mut m, tool, &case.pose)?;
            let (bored, _) = cut(&mut m, cup, tool)
                .map_err(|e| fail(format!("cut of the shell: {e}")))?;
            assert_checked(&check(&m, bored, Level::Full))?;
            let got = mass_properties(&m, bored).map_err(fail)?.volume;
            same_measure("cut volume", got, volume - PI * r * r * t, REL)?;
            // The bottom pushed down: the floor thickens.
            let bottom = face_at(&m, cup, case.pose.apply(Point3::new(x / 2.0, y / 2.0, 0.0)))?;
            let d = case.push * t;
            let (deeper, _) = offset_faces(&mut m, cup, &[bottom], d)
                .map_err(|e| fail(format!("offset of the shell's bottom: {e}")))?;
            assert_checked(&check(&m, deeper, Level::Full))?;
            let got = mass_properties(&m, deeper).map_err(fail)?.volume;
            same_measure("offset volume", got, volume + x * y * d, REL)?;
            Ok(())
        }
}
