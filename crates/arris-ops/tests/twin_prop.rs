//! The twin property (nurbs-analytic-section step 8, ADR-0061): a
//! boolean over a body whose faces are replaced by their exact
//! `to_nurbs` twins ([`arris_debug::prop::twin::nurbs_twin`]) gives the
//! analytic boolean's volume, area and counts, or both refuse. The
//! operands are the ones of the C3 identities — a frustum, a ball, a ring
//! or an elliptic prism against a box or a cylinder, a box against a
//! cylinder, two cylinders on crossing axes — so a plane, a cylinder, a
//! cone, a sphere and a torus each meet each other kind as a NURBS
//! surface against an analytic one, with the NURBS operand first or
//! second, in a random pose — and, since nurbs-nurbs-operands step 2,
//! with both operands twinned, so every face pair across them is two
//! NURBS faces and their section the marched one (ADR-0068).
//!
//! The blends follow (nurbs-face-blend step 8, ADR-0062): a part's edge
//! filleted or chamfered by the closed forms, and the same edge with one of
//! its two faces replaced by its exact NURBS twin, which walks the blend —
//! and, since nurbs-nurbs-operands step 7, with both, the spine the marched
//! section of two offset fits (ADR-0068).
//! The rows are the table's: two planes (a box's edge), a plane and a
//! cylinder along a ruling (a half disc's chord) and the meridian row
//! (a plane, a cylinder, a cone, a sphere or a torus coaxial with the edge's
//! circle: a turned part's corner), in a random pose and at random sizes.
//! The twin agrees in volume, area and counts, or both refuse.
//!
//! A disagreement prints the shrunk pair and the seed and becomes a
//! fixture under `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).

use arris_check::{Level, LumpError, Unchecked, check};
use arris_debug::dump::euler_line;
use arris_debug::prop::turned::{Piece, Turned, turned};
use arris_debug::prop::twin::{nurbs_twin, nurbs_twin_faces};
use arris_debug::testing::{close_to, fail, fitted_rel};
use arris_debug::unmetered::{
    chamfer, common, cut, extrude, fillet, fuse, mass_properties, primitive_box, revolve, transform,
};
use arris_debug::{prop, prop_shards};
use arris_geom::{
    Curve, GeomError, GeomKind, Profile, ProfileLoop, ProfileSegment, SectionFault, SurfaceKind,
};
use arris_math::{Axis, Frame, Isometry, Point2, Point3, Vec3};
use arris_ops::measure::MassProperties;
use arris_ops::{BooleanReason, Fault, OpError, Reason};
use arris_topo::{Body, Edge, Model, Provenance};
use core::f64::consts::TAU;
use proptest::prelude::*;

/// A boolean of two bodies: `fuse`, `common` or `cut`.
type Boolean = fn(&mut Model, Body, Body) -> Result<(Body, Provenance), OpError>;

/// What one boolean came to.
enum Outcome {
    /// A body, clean at `Full`, with its measure and counts.
    Made(MassProperties, String),
    /// A refusal, named.
    Refused(String),
}

/// `op(a, b)`, its body clean at `Full`, or its refusal. An analytic
/// result has nothing unchecked; a `twin` result may leave a pair of its
/// faces undecided (S5: two NURBS faces the checker cannot tell apart),
/// and nothing else. A fault the differential names as an exclusion, and the
/// designed `BesideSingularity`, reject the case: neither is the twin's.
fn outcome(
    m: &mut Model,
    name: &str,
    op: Boolean,
    a: Body,
    b: Body,
    twin: bool,
) -> Result<Outcome, TestCaseError> {
    let r = op(m, a, b);
    match r {
        Ok((body, _)) => {
            let report = check(m, body, Level::Full);
            let undecided = report
                .unchecked()
                .iter()
                .any(|u| !(twin && matches!(u, Unchecked::FacePair { .. })));
            if !report.is_ok() || undecided {
                return Err(fail(format!("{name}: not clean at Full\n{report}")));
            }
            let props = mass_properties(m, body).map_err(|e| fail(format!("{name}: {e}")))?;
            let counts = euler_line(m, body).map_err(fail)?;
            Ok(Outcome::Made(props, counts))
        }
        Err(OpError::Degenerate {
            reason: Reason::Boolean(BooleanReason::BesideSingularity),
            ..
        }) => Err(TestCaseError::reject("BesideSingularity")),
        // A twin result of several lumps whose nesting B1 leaves
        // undecided, every row a pair of two NURBS faces on two shells:
        // a thin slab across a frustum's twin leaves the cone wall's two
        // pieces a gap apart on one surface, and the box proof spends its
        // budget along a tilted strip before the boxes are thinner than
        // the gap (quadric pairs, 2 shards in 8 at 16 cases). Held by
        // `regression/nurbs-frustum-thin-slab-cut-lumps-undecided`
        // (BACKLOG). Shells that do not nest (`LumpError::Nesting`) are
        // not tolerated.
        Err(OpError::Internal(Fault::Lumps(LumpError::Undecided { rows, .. })))
            if twin
                && rows.iter().all(|row| {
                    matches!(
                        row,
                        Unchecked::ShellFacePair {
                            kinds: (SurfaceKind::Nurbs, SurfaceKind::Nurbs),
                            ..
                        }
                    )
                }) =>
        {
            Err(TestCaseError::reject("twin lumps undecided"))
        }
        // The section of two twins refused as crowded where the walls
        // touch (BACKLOG; 3 of 64 quadric pairs both twinned), held by
        // `nurbs_nurbs_prop`'s ignored
        // `the_twins_of_a_cone_and_a_plane_and_of_a_sphere_and_a_cylinder_are_met`
        // and, for two equal cylinders on crossing axes tilted,
        // `the_tilted_twins_of_two_equal_cylinders_on_crossing_axes_meet_in_two_ellipses`.
        // The tangent points of the crossing pairs are accounted for since
        // nurbs-nurbs-operands step 5, which took `TubeCircle` and
        // `UnresolvedTurning` off this arm. Named before the differential's
        // exclusions, which take any `Geometry` fault.
        Err(OpError::Internal(Fault::Geometry(GeomError::DegenerateSection {
            a,
            b,
            fault: fault @ SectionFault::CrowdedSingularity,
        }))) if twin && [a, b].contains(&GeomKind::Surface(SurfaceKind::Nurbs)) => {
            Err(TestCaseError::reject(format!("twin section: {fault}")))
        }
        Err(e) => {
            if let Some(x) = arris_debug::differential::exclusion_of_error(&e) {
                return Err(TestCaseError::reject(x.name));
            }
            Ok(Outcome::Refused(e.to_string()))
        }
    }
}

/// Which operands of a boolean, or faces of a blended edge, are NURBS
/// twins.
#[derive(Debug, Clone, Copy)]
enum Twinned {
    /// The first: a NURBS face against an analytic one (ADR-0061).
    A,
    /// The second.
    B,
    /// Both: every face pair across the operands is two NURBS faces, the
    /// marched section (ADR-0068); a blend's spine is the marched section
    /// of the two offset fits.
    Both,
}

/// Which boolean of a pair is run and which operands are twins.
type Pick = (usize, Twinned);

/// The strategy of a [`Pick`]: a twin boolean costs 5 to 50 times an
/// analytic one and its `Full` check as much again, so a case runs one
/// boolean over one twinning, and the cases cover the rest. Both operands
/// twinned is half the cases.
fn pick() -> impl Strategy<Value = Pick> {
    (
        0usize..3,
        prop_oneof![1 => Just(Twinned::A), 1 => Just(Twinned::B), 2 => Just(Twinned::Both)],
    )
}

/// The boolean `pick` names over the pair `(a, b)` in `m` and over the
/// pair with `a`, `b` or both replaced by their twins: the same volume,
/// area and counts, or both refused.
fn same_as_twin(m: &mut Model, a: Body, b: Body, pick: Pick) -> Result<(), TestCaseError> {
    let (op_name, op) = [
        ("fuse", fuse as Boolean),
        ("common", common as Boolean),
        ("cut", cut as Boolean),
    ][pick.0];
    let mut twin_of = |which: &str, body: Body| {
        nurbs_twin(m, body).map_err(|e| fail(format!("twin of {which}: {e}")))
    };
    let (x, y, which) = match pick.1 {
        Twinned::A => (twin_of("a", a)?, b, "a a twin"),
        Twinned::B => (a, twin_of("b", b)?, "b a twin"),
        Twinned::Both => (twin_of("a", a)?, twin_of("b", b)?, "both twins"),
    };
    let analytic = outcome(m, &format!("{op_name}(a, b)"), op, a, b, false)?;
    let name = format!("{op_name}(a, b) with {which}");
    match (&analytic, outcome(m, &name, op, x, y, true)?) {
        (Outcome::Refused(_), Outcome::Refused(_)) => {}
        (Outcome::Made(p, counts), Outcome::Made(q, twin_counts)) => {
            let rel = fitted_rel(m, p);
            prop_assert!(
                close_to(p.volume, q.volume, p.volume, rel),
                "{}: volume {} against the twin's {}",
                name,
                p.volume,
                q.volume
            );
            prop_assert!(
                close_to(p.area, q.area, p.area, rel),
                "{}: area {} against the twin's {}",
                name,
                p.area,
                q.area
            );
            prop_assert_eq!(counts, &twin_counts, "{}: counts", name);
        }
        (Outcome::Made(..), Outcome::Refused(why)) => {
            return Err(fail(format!("{name}: the twin refuses, {why}")));
        }
        (Outcome::Refused(why), Outcome::Made(..)) => {
            return Err(fail(format!("{name}: the analytic pair refuses, {why}")));
        }
    }
    Ok(())
}

prop_shards! {
    /// A frustum, a ball, a ring or an elliptic prism and a box or a
    /// cylinder through its interior: the cone, the sphere, the torus,
    /// the elliptic cylinder, the cylinder and the plane each against the
    /// others, one operand a NURBS twin.
    quadric_twins_agree_with_the_analytic_boolean
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((pair, pick)) = (prop::body::quadric_pair(), pick()) => {
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            same_as_twin(&mut m, a, b, pick)?;
            Ok(())
        }
}

prop_shards! {
    /// A box and a cylinder through it: planes against a cylinder.
    overlapping_twins_agree_with_the_analytic_boolean
        [shard_0 shard_1 shard_2 shard_3]
        ((pair, pick)) = (prop::body::overlapping_pair(), pick()) => {
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            same_as_twin(&mut m, a, b, pick)?;
            Ok(())
        }
}

prop_shards! {
    /// Two cylinders on crossing axes: a cylinder against a cylinder.
    crossing_twins_agree_with_the_analytic_boolean
        [shard_0 shard_1 shard_2 shard_3]
        ((pair, pick)) = (prop::body::crossing_pair(), pick()) => {
            let mut m = Model::default();
            let (a, b) = pair.build(&mut m).map_err(fail)?;
            same_as_twin(&mut m, a, b, pick)?;
            Ok(())
        }
}

/// The part whose edge is blended.
#[derive(Debug, Clone)]
enum Part {
    /// The box `[0, a] × [0, b] × [0, c]`, the `edge`th of its twelve: four
    /// parallel to each axis in turn.
    Box { dims: [f64; 3], edge: usize },
    /// A half disc of radius `radius` extruded `height` along `z`, its
    /// chord on `y = 0`: the chord edge at `x = ±radius`, a plane against a
    /// cylinder along a ruling. `far` picks the one at `-radius`.
    HalfDisc { radius: f64, height: f64, far: bool },
    /// A turned part and the corner of its outline blended.
    Turned(Turned, usize),
}

/// A part, a blend of one of its edges and the faces of that edge twinned.
#[derive(Debug, Clone)]
struct BlendTwin {
    part: Part,
    chamfer: bool,
    /// The blend's size as a fraction of the part's room.
    fraction: f64,
    /// Which of the edge's two faces are twins: the first, the second or
    /// both.
    twinned: Twinned,
    pose: Isometry,
}

/// How an edge is found in the posed body.
enum Where {
    /// The edge whose curve's middle is at the point.
    Middle(Point3),
    /// The circle of `radius` about the point.
    Circle(f64, Point3),
}

impl Where {
    fn find(&self, m: &Model, body: Body) -> Result<Edge, TestCaseError> {
        for e in m.edges(body).map_err(fail)? {
            let Some((curve, range)) = m.edge(e.id).map_err(fail)?.curve() else {
                continue;
            };
            let curve = m.curve(curve).map_err(fail)?;
            let hit = match self {
                Where::Middle(at) => (curve.point(range.midpoint()) - at).norm() < 1e-9,
                Where::Circle(radius, centre) => matches!(
                    curve,
                    Curve::Circle { frame, radius: r }
                        if (r - radius).abs() < 1e-9 && (frame.origin() - centre).norm() < 1e-9
                ),
            };
            if hit {
                return Ok(e);
            }
        }
        Err(fail("no edge where the part's was"))
    }
}

impl BlendTwin {
    /// The part in `m` at rest, the edge's place at rest, and the size.
    fn build(&self, m: &mut Model) -> Result<(Body, Where, f64), TestCaseError> {
        match &self.part {
            Part::Box {
                dims: [a, b, c],
                edge,
            } => {
                let (body, _) =
                    primitive_box(m, Point3::origin(), Point3::new(*a, *b, *c)).map_err(fail)?;
                let (lo, hi) = (edge % 2 == 0, edge / 2 % 2 == 0);
                let pick = |low: bool, extent: f64| if low { 0.0 } else { extent };
                let at = match edge / 4 {
                    0 => Point3::new(a / 2.0, pick(lo, *b), pick(hi, *c)),
                    1 => Point3::new(pick(lo, *a), b / 2.0, pick(hi, *c)),
                    _ => Point3::new(pick(lo, *a), pick(hi, *b), c / 2.0),
                };
                let room = a.min(*b).min(*c);
                Ok((body, Where::Middle(at), self.fraction * room))
            }
            Part::HalfDisc {
                radius,
                height,
                far,
            } => {
                let profile = Profile {
                    plane: Frame::world(),
                    outer: ProfileLoop::Path {
                        start: Point2::new(-radius, 0.0),
                        segments: vec![
                            ProfileSegment::LineTo(Point2::new(*radius, 0.0)),
                            ProfileSegment::ArcTo {
                                to: Point2::new(-radius, 0.0),
                                via: Point2::new(0.0, *radius),
                            },
                        ],
                    },
                    holes: Vec::new(),
                };
                let (body, _) = extrude(m, &profile, Vec3::z(), *height).map_err(fail)?;
                let x = if *far { -radius } else { *radius };
                let at = Point3::new(x, 0.0, height / 2.0);
                Ok((body, Where::Middle(at), self.fraction * radius))
            }
            Part::Turned(t, k) => {
                let axis = Axis::new(Point3::origin(), Vec3::y()).map_err(fail)?;
                let (body, _) = revolve(m, &t.profile(), axis, TAU).map_err(fail)?;
                let c = t.corner(*k);
                let at = Where::Circle(c.x, Point3::new(0.0, c.y, 0.0));
                Ok((body, at, t.size))
            }
        }
    }
}

/// What one blend came to: its body's measure and counts, clean at
/// `Full` (a twin may leave a pair of its faces undecided), or its
/// refusal.
fn blended(
    m: &mut Model,
    name: &str,
    body: Body,
    place: &Where,
    case: &BlendTwin,
    size: f64,
    twin: bool,
) -> Result<Outcome, TestCaseError> {
    let (chamfered, coaxial) = (case.chamfer, matches!(case.part, Part::Turned(..)));
    let edge = place.find(m, body)?;
    let r = if chamfered {
        chamfer(m, body, &[edge], size)
    } else {
        fillet(m, body, &[edge], size)
    };
    match r {
        Ok((result, _)) => {
            let report = check(m, result, Level::Full);
            let undecided = report
                .unchecked()
                .iter()
                .any(|u| !(twin && matches!(u, Unchecked::FacePair { .. })));
            if !report.is_ok() || undecided {
                return Err(fail(format!("{name}: not clean at Full\n{report}")));
            }
            let props = mass_properties(m, result).map_err(|e| fail(format!("{name}: {e}")))?;
            let counts = euler_line(m, result).map_err(fail)?;
            Ok(Outcome::Made(props, counts))
        }
        // A fillet on a NURBS face against a face coaxial with it: the
        // offsets' section is a circle about the shared axis, whose
        // turning points the tracer cannot tell apart (the cylinder
        // against a cone or a sphere), or no piece of it holds the ends'
        // balls. Held by the ignored `a_twinned_cylinder_fillets_against_a_*`
        // tests below; the fix lifts this arm.
        Err(OpError::Unsupported { a, b })
            if twin
                && coaxial
                && !chamfered
                && [a.0, b.0].contains(&GeomKind::Surface(SurfaceKind::Nurbs)) =>
        {
            Err(TestCaseError::reject("walked-fillet-coaxial-nurbs"))
        }
        Err(e) => Ok(Outcome::Refused(e.to_string())),
    }
}

/// The blend of the part's edge by the closed forms, and of the same edge
/// with one of its two faces twinned: the same volume, area and counts, or
/// both refused.
fn blend_twin_agrees(case: &BlendTwin) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let (rest, place, size) = case.build(&mut m)?;
    let (body, _) = transform(&mut m, rest, &case.pose).map_err(fail)?;
    let place = match place {
        Where::Middle(p) => Where::Middle(case.pose.apply(p)),
        Where::Circle(r, c) => Where::Circle(r, case.pose.apply(c)),
    };
    let edge = place.find(&m, body)?;
    let faces: Vec<_> = {
        let ours = m.faces(body).map_err(fail)?;
        m.edge_uses(edge.id)
            .map_err(fail)?
            .iter()
            .map(|u| u.face)
            .filter(|f| ours.iter().any(|g| g.id == *f))
            .collect()
    };
    let &[first, second] = &faces[..] else {
        return Err(fail(format!("{} faces on the edge", faces.len())));
    };
    let twins = match case.twinned {
        Twinned::A => vec![first],
        Twinned::B => vec![second],
        Twinned::Both => vec![first, second],
    };
    let kind = if case.chamfer { "chamfer" } else { "fillet" };
    let name = format!("{kind} of {:?} at {size}", case.part);
    let twin = nurbs_twin_faces(&mut m, body, &twins).map_err(fail)?;
    let analytic = blended(&mut m, &name, body, &place, case, size, false)?;
    let name = format!("{name} with {twins:?} twins");
    let walked = blended(&mut m, &name, twin, &place, case, size, true)?;
    match (&analytic, walked) {
        (Outcome::Refused(_), Outcome::Refused(_)) => {}
        (Outcome::Made(p, counts), Outcome::Made(q, twin_counts)) => {
            let rel = fitted_rel(&m, p);
            prop_assert!(
                close_to(p.volume, q.volume, p.volume, rel),
                "{}: volume {} against the twin's {}",
                name,
                p.volume,
                q.volume
            );
            prop_assert!(
                close_to(p.area, q.area, p.area, rel),
                "{}: area {} against the twin's {}",
                name,
                p.area,
                q.area
            );
            prop_assert_eq!(counts, &twin_counts, "{}: counts", name);
        }
        (Outcome::Made(..), Outcome::Refused(why)) => {
            return Err(fail(format!("{name}: the twin refuses, {why}")));
        }
        (Outcome::Refused(why), Outcome::Made(..)) => {
            return Err(fail(format!("{name}: the analytic edge refuses, {why}")));
        }
    }
    Ok(())
}

/// A part's blend with `part`'s edge, its two faces both twinned in
/// `both` of every 8 cases and one of them in the rest.
fn blend_twin(part: impl Strategy<Value = Part>, both: u32) -> impl Strategy<Value = BlendTwin> {
    let one = (8 - both) / 2;
    (
        part,
        any::<bool>(),
        prop::finite_f64(0.05..=0.4),
        prop_oneof![one => Just(Twinned::A), one => Just(Twinned::B), both => Just(Twinned::Both)],
        prop::pose(),
    )
        .prop_map(|(part, chamfer, fraction, twinned, pose)| BlendTwin {
            part,
            chamfer,
            fraction,
            twinned,
            pose,
        })
}

prop_shards! {
    /// A box's edge: two planes, one of them twinned.
    box_edges_blend_as_their_twins
        [shard_0 shard_1]
        (case) = blend_twin((
            [prop::finite_f64(1.5..=5.0), prop::finite_f64(1.5..=5.0), prop::finite_f64(1.5..=5.0)],
            0usize..12,
        ).prop_map(|(dims, edge)| Part::Box { dims, edge }), 4) => {
            blend_twin_agrees(&case)
        }
}

prop_shards! {
    /// A half disc's chord edge: a plane against a cylinder along a ruling.
    ruling_edges_blend_as_their_twins
        [shard_0 shard_1]
        (case) = blend_twin((
            prop::finite_f64(1.0..=3.0),
            prop::finite_f64(1.0..=4.0),
            any::<bool>(),
        ).prop_map(|(radius, height, far)| Part::HalfDisc { radius, height, far }), 4) => {
            blend_twin_agrees(&case)
        }
}

prop_shards! {
    /// A turned part's corner: a plane, a cylinder, a cone, a sphere or a
    /// torus against its neighbour, one of the two twinned, or both in one
    /// case of 8: S5 on two long NURBS neighbours costs minutes a case
    /// (BACKLOG), and at one in two the eight shards took 56 to 102 min at
    /// 256 cases, past CI's job limit at 1000. Both twinned is held by the
    /// box and ruling rows at one in two and by the walked fixtures.
    meridian_rows_blend_as_their_twins
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = blend_twin((turned(), 0usize..4).prop_map(|(t, pick)| {
            let k = t.blended[pick % t.blended.len()];
            Part::Turned(t, k)
        }), 1) => {
            blend_twin_agrees(&case)
        }
}

/// A turned part drawn by hand, its corner `k` blended at `size`.
fn turned_case(pieces: Vec<Piece>, k: usize, size: f64, twinned: Twinned) -> BlendTwin {
    BlendTwin {
        part: Part::Turned(
            Turned {
                start: Point2::new(0.0, 0.0),
                pieces,
                blended: vec![k],
                size,
                chamfer: false,
            },
            k,
        ),
        chamfer: false,
        fraction: 0.05,
        twinned,
        pose: Isometry::identity(),
    }
}

/// A shoulder, a cylinder of radius 1 then a cone in to radius 0.8, its
/// corner filleted, either face a twin: the offsets' section is a circle
/// about the shared axis.
#[test]
#[ignore = "a fillet on a twinned cylinder against a coaxial cone is refused: the cylinder offset's section turns where the tracer cannot tell the turning points apart, and no piece holds the ends' balls (nurbs-face-blend step 8 finding, backlog)"]
fn a_twinned_cylinder_fillets_against_a_cone() {
    let line = |x: f64, y: f64| Piece::Line {
        to: Point2::new(x, y),
    };
    for twinned in [Twinned::A, Twinned::B] {
        let pieces = vec![
            line(1.0, 0.0),
            line(1.0, 0.8),
            line(0.8, 1.583263472929188),
            line(0.8, 2.3832634729291877),
            line(0.0, 2.3832634729291877),
            line(0.0, 0.0),
        ];
        blend_twin_agrees(&turned_case(pieces, 2, 0.04, twinned))
            .unwrap_or_else(|e| panic!("{twinned:?}: {e}"));
    }
}

/// A cylinder of radius 1 under a spherical dome, the corner filleted,
/// either face a twin.
#[test]
#[ignore = "a fillet on a twinned cylinder against a coaxial sphere is refused: the section of the offsets has turning points the tracer cannot tell apart (nurbs-face-blend step 8 finding, backlog)"]
fn a_twinned_cylinder_fillets_against_a_sphere() {
    let pieces = vec![
        Piece::Line {
            to: Point2::new(1.0, 0.0),
        },
        Piece::Line {
            to: Point2::new(1.0, 1.0),
        },
        Piece::Arc {
            to: Point2::new(1.157380471778837e-16, 1.2861971982493905),
            via: Point2::new(0.5200742341930144, 1.21323986645996),
            centre: Point2::new(0.0, -0.6039485524840462),
            radius: 1.8901457507334367,
        },
        Piece::Line {
            to: Point2::new(0.0, 0.0),
        },
    ];
    for twinned in [Twinned::A, Twinned::B] {
        blend_twin_agrees(&turned_case(pieces.clone(), 2, 0.05, twinned))
            .unwrap_or_else(|e| panic!("{twinned:?}: {e}"));
    }
}
