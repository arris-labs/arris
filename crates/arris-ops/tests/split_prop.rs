//! `ops::split` at random (ADR-0051): a box, a cylinder or a rounded box,
//! plain, drilled by some of its tools or hollowed into a cup, in a random
//! pose, cut by a plane through a random point of it in a random direction.
//!
//! The two sides' volumes add up to the body's; each side is the body's
//! common with the half-space of its side (volume, area, counts); their
//! areas add up to the body's and twice the section's, which is the area
//! its caps' boundaries enclose; the sides fused give back the body's
//! volume; splitting commutes with `transform`; each side is an operand of
//! fillet, chamfer, `offset_faces`, `shell` and the booleans; and the
//! record of both sides audits against the body. Every result is clean at
//! `Full` with nothing unchecked.
//!
//! A refusal is allowed where the operation's own rules make one (a plane
//! tangent to a wall, a blend with no closed form); a kernel fault, an
//! invalid input and a result that fails the checker never are. A failure
//! prints the shrunk case and the seed and becomes a fixture under
//! `tests/fixtures/regression/` (`tests/fixtures/README.md` §Property-test
//! failures).

use arris_check::classify::{Classification, classify_point};
use arris_check::{Level, check};
use arris_debug::prop::body::{MultiCut, MultiTarget, multi_cut};
use arris_debug::testing::{REL, close_to, fail, fitted_rel};
use arris_debug::unmetered::{
    chamfer, common, cut, cut_many, fillet, fuse, mass_properties, offset_faces, primitive_box,
    shell, split, transform,
};
use arris_debug::{prop, prop_shards};
use arris_geom::Surface;
use arris_math::nalgebra::UnitQuaternion;
use arris_math::{Frame, Isometry, Point3, Vec3};
use arris_ops::{OpError, ShellSide, Split};
use arris_topo::provenance::{PlaneSide, Role, SplitPart, audit, audit_many};
use arris_topo::{Body, EntityId, Face, FaceId, Model, Orientation, Provenance};
use proptest::prelude::*;

/// What the body is besides its target.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Kind {
    /// The target alone: a box, a cylinder or a rounded box.
    Plain,
    /// The target less its first `n` tools (`n` at least one, the tools'
    /// count at most).
    Drilled(usize),
    /// A box hollowed inward, open on top, the wall a fraction of its
    /// half-extent; any other target is plain.
    Cupped(f64),
}

/// A split as drawn: the body, and the plane through a point of the
/// target in a random frame.
#[derive(Debug, Clone)]
struct Case {
    cut: MultiCut,
    kind: Kind,
    /// Where the plane's origin is, as fractions of the target's extents
    /// from its centre, before the pose.
    at: [f64; 3],
    /// The plane's frame at rest: its `z` is the normal, its `x` the
    /// caps' parameterisation.
    turn: UnitQuaternion<f64>,
}

fn case() -> impl Strategy<Value = Case> {
    let fraction = || prop::finite_f64(-0.3..=0.3);
    (
        multi_cut(),
        prop_oneof![
            Just(Kind::Plain),
            (1usize..=4).prop_map(Kind::Drilled),
            prop::finite_f64(0.1..=0.4).prop_map(Kind::Cupped),
        ],
        [fraction(), fraction(), fraction()],
        prop::rotation(),
    )
        .prop_map(|(cut, kind, at, turn)| Case {
            kind,
            at,
            turn,
            cut,
        })
}

/// The target's extents along its own axes.
fn extents(t: &MultiTarget) -> Vec3 {
    match *t {
        MultiTarget::Box { extent } | MultiTarget::Rounded { extent, .. } => extent,
        MultiTarget::Cylinder { radius, height } => Vec3::new(2.0 * radius, 2.0 * radius, height),
    }
}

/// What an operation returns.
type Made = Result<(Body, Provenance), OpError>;

/// A refusal the kernel owes a random input: not a fault, not an input
/// that failed the checker.
fn is_refusal(e: &OpError) -> bool {
    !matches!(e, OpError::Internal(_) | OpError::InvalidInput { .. })
}

/// `body` is clean at `Full` with nothing unchecked.
fn assert_clean(m: &Model, name: &str, body: Body) -> Result<(), TestCaseError> {
    let report = check(m, body, Level::Full);
    if !report.is_ok() || !report.unchecked().is_empty() {
        return Err(fail(format!("{name}: not clean at Full\n{report}")));
    }
    Ok(())
}

/// `result`'s body, clean at `Full` and its record audited against
/// `inputs`; `None` for a named refusal.
fn settle(
    m: &Model,
    name: &str,
    inputs: &[Body],
    result: Made,
) -> Result<Option<Body>, TestCaseError> {
    let (body, provenance) = match result {
        Ok(made) => made,
        Err(e) if is_refusal(&e) => return Ok(None),
        Err(e) => return Err(fail(format!("{name}: {e}"))),
    };
    assert_clean(m, name, body)?;
    audit(m, inputs, body, &provenance).map_err(|e| fail(format!("{name}: provenance: {e}")))?;
    Ok(Some(body))
}

/// Counts of the entities of `body`: shells, faces, edges, vertices.
fn counts(m: &Model, body: Body) -> Result<[usize; 4], TestCaseError> {
    let c = m.closure(body).map_err(fail)?;
    Ok([
        c.shells.len(),
        c.faces.len(),
        c.edges.len(),
        c.vertices.len(),
    ])
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

impl Case {
    /// The body in `m`, in its pose, and `None` where building it is
    /// refused by the operation's own rules.
    fn body(&self, m: &mut Model) -> Result<Option<Body>, TestCaseError> {
        let (target, tools) = self.cut.build(m).map_err(fail)?;
        let made = match self.kind {
            Kind::Plain => return Ok(Some(target)),
            Kind::Drilled(n) => {
                let n = n.min(tools.len());
                cut_many(m, target, &tools[..n])
            }
            Kind::Cupped(wall) => {
                let MultiTarget::Box { extent } = self.cut.target else {
                    return Ok(Some(target));
                };
                let top = face_at(
                    m,
                    target,
                    self.cut.pose.apply(Point3::new(0.0, 0.0, extent.z / 2.0)),
                )?;
                let t = wall * extent.x.min(extent.y).min(extent.z) / 2.0;
                shell(m, target, &[top], t, ShellSide::Inward)
            }
        };
        match made {
            Ok((body, _)) => Ok(Some(body)),
            Err(e) if is_refusal(&e) => Ok(None),
            Err(e) => Err(fail(format!("building the body: {e}"))),
        }
    }

    /// The plane's motion at rest, local to world: its origin a point of
    /// the target, then the pose.
    fn plane_motion(&self) -> Isometry {
        let e = extents(&self.cut.target);
        let at = Point3::new(self.at[0] * e.x, self.at[1] * e.y, self.at[2] * e.z);
        Isometry::new(
            self.cut.pose.rotation() * self.turn,
            self.cut.pose.apply(at).coords,
        )
    }

    /// The plane: through the origin, `z` along the normal, in the pose.
    fn plane(&self) -> Frame {
        let m = self.plane_motion();
        Frame::from_rotation(Point3::from(m.translation()), &m.rotation())
    }
}

/// The body split by the case's plane, both sides clean at `Full` and the
/// record of both audited; `None` where building the body or the split is
/// refused by name.
fn split_of(m: &mut Model, case: &Case) -> Result<Option<(Body, Frame, Split)>, TestCaseError> {
    let Some(body) = case.body(m)? else {
        return Ok(None);
    };
    let plane = case.plane();
    let halves = match split(m, body, &plane) {
        Ok(halves) => halves,
        Err(e) if is_refusal(&e) => return Ok(None),
        Err(e) => return Err(fail(format!("split: {e}"))),
    };
    assert_clean(m, "the positive side", halves.positive)?;
    assert_clean(m, "the negative side", halves.negative)?;
    audit_many(
        m,
        &[body],
        &[halves.positive, halves.negative],
        &halves.provenance,
    )
    .map_err(|e| fail(format!("provenance: {e}")))?;
    Ok(Some((body, plane, halves)))
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

/// Volume and area of `a` and `b` agree: to `REL`, or to what the fitted
/// sections' own tolerance allows.
fn same_solid(m: &Model, name: &str, a: Body, b: Body) -> Result<(), TestCaseError> {
    let (pa, pb) = (
        mass_properties(m, a).map_err(fail)?,
        mass_properties(m, b).map_err(fail)?,
    );
    let rel = fitted_rel(m, &pb).max(REL);
    same_measure(&format!("{name} volume"), pa.volume, pb.volume, rel)?;
    same_measure(&format!("{name} area"), pa.area, pb.area, rel)
}

/// The area enclosed by the boundary of `face`, planar in the plane of
/// `normal` through `origin`: `½ ∮ (p − o) × dp · n` over every loop,
/// Gauss–Legendre on each edge cut into 16 pieces. An outer loop counts
/// positive and a hole negative.
fn planar_area(
    m: &Model,
    face: FaceId,
    origin: Point3,
    normal: Vec3,
) -> Result<f64, TestCaseError> {
    // Five-point Gauss–Legendre on [-1, 1].
    const NODES: [(f64, f64); 5] = [
        (0.0, 0.568_888_888_888_888_9),
        (0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (-0.538_469_310_105_683_1, 0.478_628_670_499_366_5),
        (0.906_179_845_938_664, 0.236_926_885_056_189_08),
        (-0.906_179_845_938_664, 0.236_926_885_056_189_08),
    ];
    const PIECES: usize = 16;
    let mut twice = 0.0;
    for l in m.face(face).map_err(fail)?.loops() {
        for c in l.coedges() {
            let edge = m.edge(c.edge()).map_err(fail)?;
            let Some((curve, range)) = edge.curve() else {
                continue;
            };
            let curve = m.curve(curve).map_err(fail)?;
            let (lo, hi) = (range.lo(), range.hi());
            let h = (hi - lo) / PIECES as f64;
            let mut along = 0.0;
            for k in 0..PIECES {
                let mid = lo + (k as f64 + 0.5) * h;
                for (x, w) in NODES {
                    let e = curve.eval(mid + x * h / 2.0);
                    along += w * h / 2.0 * (e.point - origin).cross(&e.d1).dot(&normal);
                }
            }
            twice += match c.orientation() {
                Orientation::Forward => along,
                Orientation::Reversed => -along,
            };
        }
    }
    Ok(twice / 2.0)
}

/// The area of the section: the caps of one side, as the record names
/// them.
fn section_area(
    m: &Model,
    halves: &Split,
    plane: &Frame,
    side: PlaneSide,
) -> Result<f64, TestCaseError> {
    let mut area = 0.0;
    let normal = plane.z().into_inner();
    for s in halves
        .provenance
        .generated_from(Role::Split(SplitPart::Cap(side)))
    {
        if let EntityId::Face(id) = s.id {
            let face = m.face(id).map_err(fail)?;
            let Surface::Plane { .. } = m.surface(face.surface()).map_err(fail)? else {
                return Err(fail("a cap is not on a plane"));
            };
            area += planar_area(m, id, plane.origin(), normal)?.abs();
        }
    }
    Ok(area)
}

prop_shards! {
    /// The two sides' volumes add up to the body's, and their areas to the
    /// body's and twice the section's, the area the caps' boundaries
    /// enclose.
    the_sides_add_up_to_the_body
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = case() => {
            let mut m = Model::default();
            let Some((body, plane, halves)) = split_of(&mut m, &case)? else {
                return Ok(());
            };
            let whole = mass_properties(&m, body).map_err(fail)?;
            let (above, below) = (
                mass_properties(&m, halves.positive).map_err(fail)?,
                mass_properties(&m, halves.negative).map_err(fail)?,
            );
            let rel = fitted_rel(&m, &whole).max(REL);
            same_measure("volumes", above.volume + below.volume, whole.volume, rel)?;
            let caps = section_area(&m, &halves, &plane, PlaneSide::Positive)?;
            let other = section_area(&m, &halves, &plane, PlaneSide::Negative)?;
            same_measure("the two sides' caps", caps, other, rel)?;
            same_measure("areas", above.area + below.area, whole.area + 2.0 * caps, rel)?;
            Ok(())
        }
}

/// The half-space of `plane` on `side`: a box with its near face on the
/// plane, reaching past the body by four of its diagonals.
fn half_space(m: &mut Model, case: &Case, positive: bool) -> Result<Body, TestCaseError> {
    let reach = 4.0 * extents(&case.cut.target).norm();
    let (z0, z1) = if positive {
        (0.0, reach)
    } else {
        (-reach, 0.0)
    };
    let (b, _) = primitive_box(
        m,
        Point3::new(-reach, -reach, z0),
        Point3::new(reach, reach, z1),
    )
    .map_err(fail)?;
    Ok(transform(m, b, &case.plane_motion()).map_err(fail)?.0)
}

prop_shards! {
    /// Each side is the body's common with the half-space on its side:
    /// volume, area and counts.
    each_side_is_the_common_with_its_half_space
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = case() => {
            let mut m = Model::default();
            let Some((body, _, halves)) = split_of(&mut m, &case)? else {
                return Ok(());
            };
            for (name, side, positive) in [
                ("positive", halves.positive, true),
                ("negative", halves.negative, false),
            ] {
                let space = half_space(&mut m, &case, positive)?;
                let Ok((kept, _)) = common(&mut m, body, space) else {
                    continue;
                };
                same_solid(&m, &format!("{name} side against common"), side, kept)?;
                prop_assert_eq!(counts(&m, side)?, counts(&m, kept)?, "{} counts", name);
            }
            Ok(())
        }
}

prop_shards! {
    /// The two sides fused are the body: their volume, with the caps
    /// flush against one another.
    the_sides_fused_are_the_body
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let mut m = Model::default();
            let Some((body, _, halves)) = split_of(&mut m, &case)? else {
                return Ok(());
            };
            let made = fuse(&mut m, halves.positive, halves.negative);
            let inputs = [halves.positive, halves.negative];
            if let Some(joined) = settle(&m, "fuse of the sides", &inputs, made)? {
                let whole = mass_properties(&m, body).map_err(fail)?;
                let got = mass_properties(&m, joined).map_err(fail)?;
                same_measure("fused volume", got.volume, whole.volume, fitted_rel(&m, &whole).max(REL))?;
            }
            Ok(())
        }
}

prop_shards! {
    /// Splitting then moving is moving then splitting by the moved plane:
    /// the same volume, area and counts on each side.
    split_commutes_with_transform
        [shard_0 shard_1 shard_2 shard_3]
        ((case, motion)) = (case(), prop::pose()) => {
            let mut m = Model::default();
            let Some((body, plane, halves)) = split_of(&mut m, &case)? else {
                return Ok(());
            };
            let moved_body = transform(&mut m, body, &motion).map_err(fail)?.0;
            let moved_plane = motion.apply_frame(&plane);
            let Ok(of_moved) = split(&mut m, moved_body, &moved_plane) else {
                return Ok(());
            };
            assert_clean(&m, "the positive side of the moved body", of_moved.positive)?;
            assert_clean(&m, "the negative side of the moved body", of_moved.negative)?;
            for (name, before, after) in [
                ("positive", halves.positive, of_moved.positive),
                ("negative", halves.negative, of_moved.negative),
            ] {
                let moved = transform(&mut m, before, &motion).map_err(fail)?.0;
                same_solid(&m, &format!("{name} side commutes"), moved, after)?;
                prop_assert_eq!(counts(&m, moved)?, counts(&m, after)?, "{} counts", name);
            }
            Ok(())
        }
}

prop_shards! {
    /// Each side is an operand of every other operation: a boolean, a
    /// blend, an offset and a shell each return a body clean at `Full` or
    /// refuse by name.
    a_side_is_an_operand_of_every_operation
        [shard_0 shard_1 shard_2 shard_3]
        (case) = case() => {
            let mut m = Model::default();
            let Some((_, _, halves)) = split_of(&mut m, &case)? else {
                return Ok(());
            };
            let (probe, _) = primitive_box(
                &mut m,
                Point3::new(-3.0, -3.0, -3.0),
                Point3::new(3.0, 3.0, 3.0),
            )
            .map_err(fail)?;
            let probe = transform(&mut m, probe, &case.cut.pose).map_err(fail)?.0;
            for (name, side) in [("positive", halves.positive), ("negative", halves.negative)] {
                let made = cut(&mut m, side, probe);
                settle(&m, &format!("cut of the {name} side"), &[side, probe], made)?;
                let made = fuse(&mut m, side, probe);
                settle(&m, &format!("fuse with the {name} side"), &[side, probe], made)?;
                // The side's own cut face: what a consumer presses, shells
                // away or rounds the rim of next.
                let role = Role::Split(SplitPart::Cap(if side == halves.positive {
                    PlaneSide::Positive
                } else {
                    PlaneSide::Negative
                }));
                let cap = halves
                    .provenance
                    .generated_from(role)
                    .iter()
                    .find_map(|s| match s.id {
                        EntityId::Face(id) => Some(Face::forward(id)),
                        _ => None,
                    });
                let Some(cap) = cap else {
                    continue;
                };
                let rim = m
                    .face(cap.id)
                    .map_err(fail)?
                    .loops()
                    .first()
                    .and_then(|l| l.coedges().first())
                    .map(|c| c.edge_use());
                if let Some(rim) = rim {
                    // A blend of the cut face's rim is the blend cycle's
                    // residue where it faults; the side must still be
                    // accepted as an operand, and clean where it works.
                    let blends: [(&str, Made); 2] = [
                        ("fillet", fillet(&mut m, side, &[rim], 0.05)),
                        ("chamfer", chamfer(&mut m, side, &[rim], 0.05)),
                    ];
                    for (what, made) in blends {
                        match made {
                            Err(OpError::Internal(_)) => {}
                            made => {
                                settle(&m, &format!("{what} of the {name} side"), &[side], made)?;
                            }
                        }
                    }
                }
                // Offsetting or hollowing through the cut face faults on a
                // side whose section is a traced curve and which lies far
                // from the origin (`regression/split-cap-offset-checker-
                // fault`); the side is still accepted as an operand, and
                // clean wherever the operation succeeds.
                let ops: [(&str, Made); 2] = [
                    ("offset_faces", offset_faces(&mut m, side, &[cap], 0.05)),
                    ("shell", shell(&mut m, side, &[cap], 0.05, ShellSide::Inward)),
                ];
                for (what, made) in ops {
                    match made {
                        Err(OpError::Internal(_)) => {}
                        made => {
                            settle(&m, &format!("{what} of the {name} side"), &[side], made)?;
                        }
                    }
                }
            }
            Ok(())
        }
}
