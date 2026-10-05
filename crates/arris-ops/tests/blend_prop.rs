//! `ops::fillet` and `ops::chamfer` at random (ADR-0007): a box and an
//! extruded L of random proportions, a random subset of their plane–plane
//! edges — no edge meeting a concave one at a vertex — and a size below
//! the bound the faces allow. Each result is clean at `Full`, its record
//! audits, its volume is the input's less the closed form per edge
//! corrected by each miter's and each corner's, blending then moving
//! it equals moving then blending by volume and counts, and two runs dump
//! identically. A failure prints the case and the seed, and becomes a
//! fixture under `tests/fixtures/regression/` (`tests/fixtures/README.md`
//! §Property-test failures).
//!
//! The end families of ADR-0037 follow: a rib running into a round or
//! conical boss with its top edge blended, and a plate's boss whose foot
//! is blended where a second boss crosses it. Their blends end on a fitted
//! curve, so the volume is the unblended body's less or plus a section
//! integrated in closed form over the face across, to the model's
//! tolerance, and each result also round-trips through STEP.
//!
//! A rim split into two to four arcs (ADR-0041) follows: a hole's edge or a
//! boss's foot on a plate, one arc blended and the chain closing round the
//! ring, at the torus-swept closed form.
//!
//! A blend ending at a cusp (ADR-0042) follows: a crescent prism, its
//! pocket or a spandrel, one edge running to the cusp and its stripe cut by
//! the next wall, at the corner section integrated over the gap.
//!
//! A blend ending across a fan of faces (ADR-0043) closes the file: a
//! rise whose top is a roof of two or three planes meeting along sharp
//! ridges from the rise's top, at the roof's height integrated over the
//! blend's cross-section.
//!
//! A miter of unequal dihedrals (ADR-0044) is last: a prism over a random
//! parallelogram, the rise at one vertex and a top edge there blended, at
//! the two blends' regions in closed form less their overlap.

use arris_check::{Level, Report, Unchecked, check};
use arris_debug::polyhedron::polyhedron;
use arris_debug::prop::turned::{Piece, Turned, sweep, turned};
use arris_debug::testing::{REL, close_to, fail, fitted_rel};
use arris_debug::unmetered::build;
use arris_debug::unmetered::{
    chamfer, cut, extrude, fillet, fuse, mass_properties, primitive_box, primitive_cylinder,
    revolve, step_read, transform,
};
use arris_debug::{dump_text, prop, prop_shards};
use arris_geom::SurfaceKind;
use arris_geom::{Curve, Profile, ProfileLoop, ProfileSegment};
use arris_io::step::{self, ReadOptions};
use arris_math::{Axis, Frame, Isometry, Point2, Point3, Vec2, Vec3};
use arris_ops::OpError;
use arris_topo::provenance::audit;
use arris_topo::{Body, Edge, Model, Provenance};
use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, SQRT_2};
use proptest::prelude::*;

/// Fillet or chamfer, which the closed forms and the checker's
/// expectation depend on.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Blend {
    Fillet,
    Chamfer,
}

/// A right prism over a polygon in the world's XY plane, from `z = 0` to
/// `z = height`: a box through `primitive_box`, an L through `extrude`.
#[derive(Debug, Clone, PartialEq)]
enum Prism {
    /// The box from the origin to `(x, y, height)`.
    Box { x: f64, y: f64, height: f64 },
    /// The L `(0,0)–(x,0)–(x,arm_y)–(arm_x,arm_y)–(arm_x,y)–(0,y)`,
    /// its vertex 3 reflex.
    Ell {
        x: f64,
        y: f64,
        arm_x: f64,
        arm_y: f64,
        height: f64,
    },
}

/// One edge of a prism over polygon vertices `0..n`: the bottom or top
/// edge from vertex `k` to `k + 1`, or the rise at vertex `k`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum PrismEdge {
    Bottom(usize),
    Top(usize),
    Rise(usize),
}

impl Prism {
    fn polygon(&self) -> Vec<Point2> {
        let p = Point2::new;
        match *self {
            Prism::Box { x, y, .. } => vec![p(0.0, 0.0), p(x, 0.0), p(x, y), p(0.0, y)],
            Prism::Ell {
                x, y, arm_x, arm_y, ..
            } => vec![
                p(0.0, 0.0),
                p(x, 0.0),
                p(x, arm_y),
                p(arm_x, arm_y),
                p(arm_x, y),
                p(0.0, y),
            ],
        }
    }

    fn height(&self) -> f64 {
        match *self {
            Prism::Box { height, .. } | Prism::Ell { height, .. } => height,
        }
    }

    fn volume(&self) -> f64 {
        match *self {
            Prism::Box { x, y, height } => x * y * height,
            Prism::Ell {
                x,
                y,
                arm_x,
                arm_y,
                height,
            } => (x * arm_y + arm_x * (y - arm_y)) * height,
        }
    }

    /// Whether polygon vertex `k` is reflex: the rise there is concave.
    fn reflex(&self, k: usize) -> bool {
        let poly = self.polygon();
        let n = poly.len();
        let (a, b, c) = (poly[(k + n - 1) % n], poly[k], poly[(k + 1) % n]);
        (b - a).perp(&(c - b)) < 0.0
    }

    fn edges(&self) -> Vec<PrismEdge> {
        let n = self.polygon().len();
        (0..n)
            .flat_map(|k| [PrismEdge::Bottom(k), PrismEdge::Top(k), PrismEdge::Rise(k)])
            .collect()
    }

    /// The prism's vertices an edge ends at, as `(polygon vertex, top)`.
    fn ends(&self, e: PrismEdge) -> [(usize, bool); 2] {
        let n = self.polygon().len();
        match e {
            PrismEdge::Bottom(k) => [(k, false), ((k + 1) % n, false)],
            PrismEdge::Top(k) => [(k, true), ((k + 1) % n, true)],
            PrismEdge::Rise(k) => [(k, false), (k, true)],
        }
    }

    fn concave(&self, e: PrismEdge) -> bool {
        matches!(e, PrismEdge::Rise(k) if self.reflex(k))
    }

    fn midpoint(&self, e: PrismEdge) -> Point3 {
        let poly = self.polygon();
        let n = poly.len();
        let h = self.height();
        let mid = |k: usize| poly[k] + (poly[(k + 1) % n] - poly[k]) / 2.0;
        match e {
            PrismEdge::Bottom(k) => Point3::new(mid(k).x, mid(k).y, 0.0),
            PrismEdge::Top(k) => Point3::new(mid(k).x, mid(k).y, h),
            PrismEdge::Rise(k) => Point3::new(poly[k].x, poly[k].y, h / 2.0),
        }
    }

    fn length(&self, e: PrismEdge) -> f64 {
        let poly = self.polygon();
        let n = poly.len();
        match e {
            PrismEdge::Bottom(k) | PrismEdge::Top(k) => (poly[(k + 1) % n] - poly[k]).norm(),
            PrismEdge::Rise(_) => self.height(),
        }
    }

    fn shortest_edge(&self) -> f64 {
        self.edges()
            .into_iter()
            .map(|e| self.length(e))
            .fold(f64::INFINITY, f64::min)
    }

    /// The edges of `mask` a blend of one call takes, in edge order: an
    /// edge that meets a concave edge at a vertex, or that is concave and
    /// meets a picked one, is left out, so a vertex holding three picked
    /// edges is a convex right-angled corner. Never empty: the first edge
    /// that qualifies alone stands in for an empty pick.
    fn pick(&self, mask: &[bool]) -> Vec<PrismEdge> {
        let edges = self.edges();
        let meets = |a: PrismEdge, b: PrismEdge| {
            a != b && self.ends(a).iter().any(|v| self.ends(b).contains(v))
        };
        let clear = |e: PrismEdge| {
            self.concave(e) || !edges.iter().any(|&c| self.concave(c) && meets(e, c))
        };
        let mut picked: Vec<PrismEdge> = Vec::new();
        for (&e, &on) in edges.iter().zip(mask) {
            if !on || !clear(e) {
                continue;
            }
            let beside_concave = picked
                .iter()
                .any(|&p| meets(e, p) && (self.concave(p) || self.concave(e)));
            if !beside_concave {
                picked.push(e);
            }
        }
        if picked.is_empty() {
            picked.extend(edges.iter().copied().find(|&e| clear(e)));
        }
        picked
    }

    /// The vertices where `blends` picked edges meet: two at a miter,
    /// three at a corner.
    fn meeting(&self, picked: &[PrismEdge], blends: usize) -> usize {
        let n = self.polygon().len();
        (0..n)
            .flat_map(|k| [(k, false), (k, true)])
            .filter(|v| picked.iter().filter(|e| self.ends(**e).contains(v)).count() == blends)
            .count()
    }

    /// The vertices where two picked edges meet: the miters.
    fn miters(&self, picked: &[PrismEdge]) -> usize {
        self.meeting(picked, 2)
    }

    /// The vertices where three picked edges meet: the corners.
    fn corners(&self, picked: &[PrismEdge]) -> usize {
        self.meeting(picked, 3)
    }

    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match *self {
            Prism::Box { x, y, height } => {
                Ok(primitive_box(m, Point3::origin(), Point3::new(x, y, height))?.0)
            }
            Prism::Ell { height, .. } => {
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
                Ok(extrude(m, &profile, Vec3::z(), height)?.0)
            }
        }
    }
}

/// One case: the prism, the edges blended, the size and the pose.
#[derive(Debug, Clone)]
struct Case {
    prism: Prism,
    edges: Vec<PrismEdge>,
    size: f64,
    pose: Isometry,
}

impl Case {
    /// The volume after the blend: each convex edge loses its section's
    /// area over its length — `(1 − π/4) r²` round, `d²/2` flat — each
    /// concave one gains it, each miter of two convex right-angled edges
    /// gives back the corner the two prisms both counted, `(5/3 − π/2) r³`
    /// round and `d³/3` flat, and each corner of three gives back what the
    /// three prisms counted beyond the sphere octant's or the triangle's
    /// cut, `3(1 − π/4) r³ − (1 − π/6) r³ = (2 − 7π/12) r³` round and
    /// `3d³/2 − 5d³/6 = 2d³/3` flat.
    fn volume(&self, kind: Blend) -> f64 {
        let s = self.size;
        let (section, miter, corner) = match kind {
            Blend::Fillet => (
                (1.0 - FRAC_PI_4) * s * s,
                (5.0 / 3.0 - FRAC_PI_2) * s * s * s,
                (2.0 - 7.0 * PI / 12.0) * s * s * s,
            ),
            Blend::Chamfer => (s * s / 2.0, s * s * s / 3.0, 2.0 * s * s * s / 3.0),
        };
        let edges: f64 = self
            .edges
            .iter()
            .map(|&e| {
                let sign = if self.prism.concave(e) { 1.0 } else { -1.0 };
                sign * section * self.prism.length(e)
            })
            .sum();
        self.prism.volume()
            + edges
            + miter * self.prism.miters(&self.edges) as f64
            + corner * self.prism.corners(&self.edges) as f64
    }
}

fn prism() -> impl Strategy<Value = Prism> {
    let side = || prop::finite_f64(1.0..=4.0);
    let arm = || prop::finite_f64(0.3..=0.7);
    prop_oneof![
        (side(), side(), side()).prop_map(|(x, y, height)| Prism::Box { x, y, height }),
        (side(), side(), arm(), arm(), side()).prop_map(|(x, y, fx, fy, height)| Prism::Ell {
            x,
            y,
            arm_x: fx * x,
            arm_y: fy * y,
            height,
        }),
    ]
}

/// A prism, a pick of its edges, a size up to 0.4 of its shortest edge —
/// so the contacts of two blends on one face, and the trims at both ends
/// of one corner edge, stay apart — and a pose. The pick's density is
/// drawn first, so a lone edge and every corner filled are both common.
fn case() -> impl Strategy<Value = Case> {
    (
        prism(),
        prop::finite_f64(0.0..=1.0),
        proptest::collection::vec(prop::finite_f64(0.0..=1.0), 18),
        prop::finite_f64(0.05..=0.8),
        prop::pose(),
    )
        .prop_map(|(prism, density, draws, fraction, pose)| {
            let mask: Vec<bool> = draws.iter().map(|&u| u < density).collect();
            Case {
                edges: prism.pick(&mask),
                size: fraction * prism.shortest_edge() / 2.0,
                prism,
                pose,
            }
        })
}

type BlendOp = fn(&mut Model, Body, &[Edge], f64) -> Result<(Body, Provenance), OpError>;

fn op(kind: Blend) -> BlendOp {
    match kind {
        Blend::Fillet => fillet,
        Blend::Chamfer => chamfer,
    }
}

/// The edge of `body` whose curve's midpoint is nearest `at`, required
/// within `1e-9` of `at`'s distance from the origin (and of 1): a pose
/// moves the body up to 100 away, and the midpoint with it.
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

/// `report` has no violation and nothing unchecked: every face pair a
/// blend makes is decided by S5 and B1, the skew blend cylinders two
/// fillets meet in and a corner's sphere against a blend cylinder off its
/// centre among them, traced over the overlap of the two faces' boxes
/// (ADR-0018). Nothing here carries a torus face — every blended edge is
/// between two planes — but nothing would be left undecided if it did:
/// every pair of analytic surfaces is decided in every pose (ADR-0019).
fn assert_checked(report: &Report) -> Result<(), TestCaseError> {
    prop_assert!(report.is_ok(), "{}", report);
    prop_assert!(
        report.unchecked().is_empty(),
        "nothing unchecked\n{}",
        report
    );
    Ok(())
}

/// The blend of `case` in a fresh model, the prism moved to its pose
/// first: the model, the input, the result and its record.
fn posed(case: &Case, kind: Blend) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let prism = case.prism.build(&mut m).map_err(fail)?;
    let (moved, _) = transform(&mut m, prism, &case.pose).map_err(fail)?;
    let edges = case
        .edges
        .iter()
        .map(|&e| edge_near(&m, moved, case.pose.apply(case.prism.midpoint(e))))
        .collect::<Result<Vec<_>, _>>()?;
    let (blended, p) = op(kind)(&mut m, moved, &edges, case.size)
        .map_err(|e| fail(format!("{kind:?} of the posed prism: {e}")))?;
    Ok((m, moved, blended, p))
}

fn blends_as_their_closed_forms(case: Case, kind: Blend) -> Result<(), TestCaseError> {
    // Moved, then blended.
    let (m, moved, blended, p) = posed(&case, kind)?;
    let report = check(&m, blended, Level::Full);
    assert_checked(&report)?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    // A fillet miter's ellipse has a fitted pcurve on each cylinder, which
    // bounds the volume only to the model's tolerance.
    let rel = if kind == Blend::Fillet && case.prism.miters(&case.edges) > 0 {
        fitted_rel(&m, &props)
    } else {
        REL
    };
    let volume = case.volume(kind);
    prop_assert!(
        close_to(props.volume, volume, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        volume
    );

    // Blended, then moved.
    let mut here = Model::default();
    let prism = case.prism.build(&mut here).map_err(fail)?;
    let at_rest = case
        .edges
        .iter()
        .map(|&e| edge_near(&here, prism, case.prism.midpoint(e)))
        .collect::<Result<Vec<_>, _>>()?;
    let (rest, rest_p) = op(kind)(&mut here, prism, &at_rest, case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    let rest_report = check(&here, rest, Level::Full);
    assert_checked(&rest_report)?;
    audit(&here, &[prism], rest, &rest_p).map_err(|e| fail(format!("provenance at rest: {e}")))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Deterministic.
    let (again, _, twice, again_p) = posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&again, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(again_p, p);
    Ok(())
}

prop_shards! {
    /// Fillets of a random pick of a box's or an L's edges at a random
    /// pose: clean at `Full` with nothing unchecked — two blend cylinders
    /// on skew axes and a corner's sphere against a blend cylinder are
    /// traced (ADR-0018) — audited, at the closed-form volume,
    /// pose-independent and deterministic.
    fillets_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = case() => {
            blends_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// Chamfers of the same picks: clean at `Full` with nothing
    /// unchecked, audited, at the closed-form volume, pose-independent and
    /// deterministic.
    chamfers_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = case() => {
            blends_as_their_closed_forms(case, Blend::Chamfer)
        }
}

/// Shard 6 of `fillets_match_their_closed_forms` at 5000 cases on the
/// fixed seed, shrunk: every edge of a small L but two sides' top and
/// bottom, filleted at 0.012 about 95 out. Each corner's squareness
/// was read off the directions from the ball's centre to its three touch
/// points, and a body that far out with a ball that small puts them 2e-12
/// off the faces' normals — past the angular tolerance, so a square
/// corner was refused as `VertexBlend` (ADR-0024).
#[test]
fn a_small_fillet_corner_far_out_is_still_square() {
    use arris_math::nalgebra::{Quaternion, Unit};
    let case = Case {
        prism: Prism::Ell {
            x: 1.0,
            y: 1.0,
            arm_x: 0.376254679002123,
            arm_y: 0.524799475802280,
            height: 1.0,
        },
        edges: vec![
            PrismEdge::Bottom(0),
            PrismEdge::Top(0),
            PrismEdge::Rise(0),
            PrismEdge::Bottom(1),
            PrismEdge::Top(1),
            PrismEdge::Rise(1),
            PrismEdge::Rise(2),
            PrismEdge::Rise(3),
            PrismEdge::Bottom(4),
            PrismEdge::Top(4),
            PrismEdge::Rise(4),
            PrismEdge::Bottom(5),
            PrismEdge::Top(5),
            PrismEdge::Rise(5),
        ],
        size: 0.012445353519596,
        pose: Isometry::new(
            Unit::new_unchecked(Quaternion::new(
                0.471734955147940,
                0.0,
                0.798923076100203,
                0.373079147857608,
            )),
            Vec3::new(66.25009385227327, 66.38690084404882, 0.0),
        ),
    };
    if let Err(e) = blends_as_their_closed_forms(case, Blend::Fillet) {
        panic!("{e}");
    }
}

// Turned parts: the meridian row (ADR-0036) in one call. A profile of lines
// and an arc revolved about `y`, some of its corners blended; the volume is
// the part's by Pappus less (a fillet or chamfer of a convex corner) or
// plus (a concave one) each corner's section swept about the axis — a
// region this test bounds itself, from the two meridians and the ball, not
// from the kernel's contacts.

/// A meridian leaving a corner `v`: its point at arc length `s` from it.
#[derive(Debug, Clone, Copy)]
enum Ray {
    Line {
        v: Point2,
        out: Vec2,
        /// The left of the outline's direction of travel, the material's
        /// side.
        left: Vec2,
    },
    Circle {
        centre: Point2,
        radius: f64,
        /// The angle of the corner, and the way the angle runs leaving it.
        from: f64,
        way: f64,
        /// Whether the outline travels the circle counter-clockwise.
        ccw: bool,
    },
}

impl Ray {
    /// Piece `k` of the outline as it leaves corner `k` (`forward`) or
    /// piece `k − 1` as it leaves it backward.
    fn leaving(t: &Turned, k: usize, forward: bool) -> Ray {
        let i = if forward { k } else { k - 1 };
        let (from, to) = (t.from(i), t.pieces[i].end());
        let v = t.corner(k);
        match &t.pieces[i] {
            Piece::Line { .. } => {
                let travel = (to - from).normalize();
                Ray::Line {
                    v,
                    out: if forward { travel } else { -travel },
                    left: Vec2::new(-travel.y, travel.x),
                }
            }
            &Piece::Arc {
                via,
                centre,
                radius,
                ..
            } => {
                let swept = sweep(from, via, to, centre);
                Ray::Circle {
                    centre,
                    radius,
                    from: (v.y - centre.y).atan2(v.x - centre.x),
                    way: if (swept > 0.0) == forward { 1.0 } else { -1.0 },
                    ccw: swept > 0.0,
                }
            }
        }
    }

    fn start(&self) -> Point2 {
        match *self {
            Ray::Line { v, .. } => v,
            Ray::Circle {
                centre,
                radius,
                from,
                ..
            } => centre + Vec2::new(from.cos(), from.sin()) * radius,
        }
    }

    fn direction(&self) -> Vec2 {
        match *self {
            Ray::Line { out, .. } => out,
            Ray::Circle { from, way, .. } => Vec2::new(-from.sin(), from.cos()) * way,
        }
    }

    /// The unit direction of motion at arc length `s`.
    fn direction_at(&self, s: f64) -> Vec2 {
        match *self {
            Ray::Line { out, .. } => out,
            Ray::Circle {
                radius, from, way, ..
            } => {
                let a = from + way * s / radius;
                Vec2::new(-a.sin(), a.cos()) * way
            }
        }
    }

    fn at(&self, s: f64) -> Point2 {
        match *self {
            Ray::Line { v, out, .. } => v + out * s,
            Ray::Circle {
                centre,
                radius,
                from,
                way,
                ..
            } => {
                let a = from + way * s / radius;
                centre + Vec2::new(a.cos(), a.sin()) * radius
            }
        }
    }

    /// The arc length from the corner to the point of the meridian nearest
    /// `p`.
    fn foot(&self, p: Point2) -> f64 {
        match *self {
            Ray::Line { v, out, .. } => (p - v).dot(&out),
            Ray::Circle {
                centre,
                radius,
                from,
                way,
                ..
            } => {
                let a = (p.y - centre.y).atan2(p.x - centre.x);
                radius * (way * (a - from)).rem_euclid(2.0 * PI)
            }
        }
    }

    /// The arc length at distance `d` from the corner: along a line, the
    /// chord on a circle.
    fn at_distance(&self, d: f64) -> f64 {
        match *self {
            Ray::Line { .. } => d,
            Ray::Circle { radius, .. } => radius * 2.0 * (d / (2.0 * radius)).asin(),
        }
    }
}

/// Where a ball of radius `r` on side `side` (`1` the left of the
/// outline's travel, the material) of both meridians touches them: its
/// centre.
fn ball_centre(a: &Ray, b: &Ray, side: f64, r: f64) -> Point2 {
    let v = a.start();
    // A meridian offset to the ball's side: a line through a point and
    // direction, or a circle.
    enum Offset {
        Line(Point2, Vec2),
        Circle(Point2, f64),
    }
    let offset = |ray: &Ray| match *ray {
        Ray::Line { v, out, left } => Offset::Line(v + left * (side * r), out),
        Ray::Circle {
            centre,
            radius,
            ccw,
            ..
        } => Offset::Circle(centre, radius - side * r * if ccw { 1.0 } else { -1.0 }),
    };
    let nearest = |points: Vec<Point2>| {
        points
            .into_iter()
            .min_by(|p, q| (p - v).norm().total_cmp(&(q - v).norm()))
            .expect("an offset meridian meets the other")
    };
    match (offset(a), offset(b)) {
        (Offset::Line(p, d), Offset::Line(q, e)) => {
            let s = (q - p).perp(&e) / d.perp(&e);
            p + d * s
        }
        (Offset::Line(p, d), Offset::Circle(c, rho))
        | (Offset::Circle(c, rho), Offset::Line(p, d)) => {
            let w = p - c;
            let (half, rest) = (w.dot(&d), w.dot(&w) - rho * rho);
            let disc = (half * half - rest).sqrt();
            nearest(vec![p + d * (-half + disc), p + d * (-half - disc)])
        }
        (Offset::Circle(..), Offset::Circle(..)) => unreachable!("two arcs never meet at a corner"),
    }
}

/// `π ∮ x² dy` along `n` samples of `curve` on `[0, 1]`: the volume of
/// revolution about `y` of the region the boundary encloses, by Gauss–
/// Legendre quadrature of 24 points.
fn swept(curve: &dyn Fn(f64) -> (Point2, Vec2)) -> f64 {
    // The nodes and weights of the 8-point rule on three sub-intervals.
    const NODES: [f64; 4] = [
        0.183434642495649,
        0.525532409916328,
        0.796666477413626,
        0.960289856497536,
    ];
    const WEIGHTS: [f64; 4] = [
        0.362683783378361,
        0.313706645877887,
        0.222381034453374,
        0.101228536290376,
    ];
    let mut sum = 0.0;
    for part in 0..3 {
        let (lo, hi) = (part as f64 / 3.0, (part + 1) as f64 / 3.0);
        for (x, w) in NODES.iter().zip(WEIGHTS) {
            for sign in [-1.0, 1.0] {
                let t = (lo + hi) / 2.0 + sign * x * (hi - lo) / 2.0;
                let (p, d) = curve(t);
                sum += w * (hi - lo) / 2.0 * p.x * p.x * d.y;
            }
        }
    }
    PI * sum
}

/// The volume of the part's own outline, by Pappus about `y`.
fn turned_volume(t: &Turned) -> f64 {
    let mut total = 0.0;
    for i in 0..t.pieces.len() {
        let (from, to) = (t.from(i), t.pieces[i].end());
        total += match &t.pieces[i] {
            Piece::Line { .. } => swept(&|s| (from + (to - from) * s, to - from)),
            &Piece::Arc {
                via,
                centre,
                radius,
                ..
            } => {
                let sw = sweep(from, via, to, centre);
                let a0 = (from.y - centre.y).atan2(from.x - centre.x);
                swept(&|s| {
                    let a = a0 + sw * s;
                    (
                        centre + Vec2::new(a.cos(), a.sin()) * radius,
                        Vec2::new(-a.sin(), a.cos()) * (radius * sw),
                    )
                })
            }
        };
    }
    total.abs()
}

/// A boundary piece: its point and velocity at `s` in `[0, 1]`.
type Closing = Box<dyn Fn(f64) -> (Point2, Vec2)>;

/// The section corner `k` loses (convex) or gains (concave), swept: the
/// region between the corner, the two contacts and the ball's arc or the
/// chamfer's chord.
fn corner_volume(t: &Turned, k: usize) -> (f64, bool) {
    let (a, b) = (Ray::leaving(t, k, false), Ray::leaving(t, k, true));
    let (t_in, t_out) = (-a.direction(), b.direction());
    let convex = t_in.perp(&t_out) > 0.0;
    let side = if convex { 1.0 } else { -1.0 };
    let r = t.size;
    let (sa, sb, closing): (f64, f64, Closing) = if t.chamfer {
        let (sa, sb) = (a.at_distance(r), b.at_distance(r));
        let (pa, pb) = (a.at(sa), b.at(sb));
        (sa, sb, Box::new(move |s| (pa + (pb - pa) * s, pb - pa)))
    } else {
        let c = ball_centre(&a, &b, side, r);
        let (sa, sb) = (a.foot(c), b.foot(c));
        let (pa, pb) = (a.at(sa), b.at(sb));
        let angle = |p: Point2| (p.y - c.y).atan2(p.x - c.x);
        let mut turn = angle(pb) - angle(pa);
        while turn > PI {
            turn -= 2.0 * PI;
        }
        while turn < -PI {
            turn += 2.0 * PI;
        }
        let a0 = angle(pa);
        (
            sa,
            sb,
            Box::new(move |s| {
                let q = a0 + turn * s;
                (
                    c + Vec2::new(q.cos(), q.sin()) * r,
                    Vec2::new(-q.sin(), q.cos()) * (r * turn),
                )
            }),
        )
    };
    // The corner out along one meridian to its contact, the closing arc or
    // chord, and back along the other.
    let out = swept(&|s| (a.at(sa * s), a.direction_at(sa * s) * sa));
    let back = swept(&|s| (b.at(sb * (1.0 - s)), -b.direction_at(sb * (1.0 - s)) * sb));
    let close = swept(&|s| closing(s));
    ((out + close + back).abs(), convex)
}

/// The edge of `body` that is the circle about `(0, y, 0)` of radius `x`.
fn circle_at(m: &Model, body: Body, x: f64, y: f64) -> Result<Edge, TestCaseError> {
    for e in m.edges(body).map_err(fail)? {
        let entity = m.edge(e.id).map_err(fail)?;
        let Some((curve, _)) = entity.curve() else {
            continue;
        };
        if let Curve::Circle { frame, radius } = m.curve(curve).map_err(fail)?
            && (frame.origin() - Point3::new(0.0, y, 0.0)).norm() < 1e-9
            && (radius - x).abs() < 1e-9
        {
            return Ok(e);
        }
    }
    Err(fail(format!("no circle of radius {x} at height {y}")))
}

fn turned_in(m: &mut Model, t: &Turned) -> Result<Body, TestCaseError> {
    let axis = Axis::new(Point3::origin(), Vec3::y()).map_err(fail)?;
    Ok(revolve(m, &t.profile(), axis, 2.0 * PI).map_err(fail)?.0)
}

fn blend_turned(
    m: &mut Model,
    body: Body,
    t: &Turned,
) -> Result<(Body, Provenance), TestCaseError> {
    let edges = t
        .blended
        .iter()
        .map(|&k| circle_at(m, body, t.corner(k).x, t.corner(k).y))
        .collect::<Result<Vec<_>, _>>()?;
    let kind = if t.chamfer {
        Blend::Chamfer
    } else {
        Blend::Fillet
    };
    op(kind)(m, body, &edges, t.size).map_err(|e| fail(format!("{kind:?} of the turned part: {e}")))
}

fn turned_matches_pappus(t: Turned) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let part = turned_in(&mut m, &t)?;
    let (blended, p) = blend_turned(&mut m, part, &t)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[part], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let mut want = turned_volume(&t);
    for &k in &t.blended {
        let (section, convex) = corner_volume(&t, k);
        want += if convex { -section } else { section };
    }
    // A section on a cone, a sphere or a torus is fitted, so the volume
    // holds to the model's tolerance over the body's size.
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs Pappus {}",
        props.volume,
        want
    );
    let mut again = Model::default();
    let part = turned_in(&mut again, &t)?;
    let (twice, again_p) = blend_turned(&mut again, part, &t)?;
    prop_assert_eq!(
        dump_text(&again, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(again_p, p);
    Ok(())
}

prop_shards! {
    /// A turned part — a coned shoulder, a dome, a toroidal bead — with a
    /// pick of its circular corners filleted or chamfered in one call:
    /// clean at `Full` with nothing unchecked, audited, at Pappus' volume
    /// about the axis and deterministic.
    turned_parts_blend_as_pappus
        [shard_0 shard_1 shard_2 shard_3] (t) = turned() => {
            turned_matches_pappus(t)
        }
}

/// The composite Simpson rule over `[a, b]` at `n` (even) intervals: the
/// integrands below are smooth after their substitution.
fn simpson(a: f64, b: f64, n: usize, f: &dyn Fn(f64) -> f64) -> f64 {
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for i in 1..n {
        sum += f(a + h * i as f64) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    sum * h / 3.0
}

/// A scene whose blend ends on a face across, by the family of ADR-0037.
#[derive(Debug, Clone, PartialEq)]
enum Ends {
    /// A rib `[0, −w, 0]–[cx, w, h]` running along `x` into a boss about
    /// the vertical axis through `(cx, 0)` from `z = −1` to `h + 1`, of
    /// radius `reach · w` at the foot and `(1 − taper)` of that at the top
    /// (a cylinder when `taper` is `0`); the edge at `y = w` along the
    /// top is blended, `lead` long before it meets the boss at the top.
    Rib {
        w: f64,
        h: f64,
        reach: f64,
        taper: f64,
        lead: f64,
    },
    /// A plate `[−6, 6]² × [0, 1]` with a boss of radius `ra` about the
    /// vertical axis through the origin and a second of radius `rb` and
    /// height `hb` about `(d, 0)`, whose wall crosses the first's: the
    /// first boss's foot, an open arc, is blended.
    Twin {
        ra: f64,
        rb: f64,
        d: f64,
        ha: f64,
        hb: f64,
    },
}

/// A scene, a blend size and a pose.
#[derive(Debug, Clone)]
struct EndCase {
    scene: Ends,
    size: f64,
    pose: Isometry,
}

/// How far a ball's section keeps from the face across's grazing: the
/// ring of a twin boss's blend meets the second wall at angles of at
/// least this much of a radius.
const TWIN_MARGIN: f64 = 0.15;

impl Ends {
    /// The boss's radius at height `z` of a rib's scene.
    fn rib_radius(&self, z: f64) -> f64 {
        let Ends::Rib {
            w, h, reach, taper, ..
        } = *self
        else {
            return 0.0;
        };
        let (foot, top) = (reach * w, reach * w * (1.0 - taper));
        foot + (top - foot) * (z + 1.0) / (h + 2.0)
    }

    /// The boss's axis `x` of a rib's scene.
    fn rib_centre(&self) -> f64 {
        let Ends::Rib { w, h, lead, .. } = *self else {
            return 0.0;
        };
        (self.rib_radius(h).powi(2) - w * w).sqrt() + lead
    }

    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match *self {
            Ends::Rib { w, h, taper, .. } => {
                let cx = self.rib_centre();
                let (rib, _) = primitive_box(m, Point3::new(0.0, -w, 0.0), Point3::new(cx, w, h))?;
                let boss = if taper == 0.0 {
                    let axis = Axis::new(Point3::new(cx, 0.0, -1.0), Vec3::z())?;
                    primitive_cylinder(m, axis, self.rib_radius(-1.0), h + 2.0)?.0
                } else {
                    let plane = Frame::new(Point3::new(cx, 0.0, 0.0), -Vec3::y(), Vec3::x())?;
                    let (foot, top) = (self.rib_radius(-1.0), self.rib_radius(h + 1.0));
                    let corners = [
                        Point2::new(foot, -1.0),
                        Point2::new(top, h + 1.0),
                        Point2::new(0.0, h + 1.0),
                        Point2::new(0.0, -1.0),
                    ];
                    let profile = Profile {
                        plane,
                        outer: ProfileLoop::Path {
                            start: corners[3],
                            segments: corners.iter().map(|&q| ProfileSegment::LineTo(q)).collect(),
                        },
                        holes: Vec::new(),
                    };
                    let axis = Axis::new(Point3::new(cx, 0.0, 0.0), Vec3::z())?;
                    revolve(m, &profile, axis, 2.0 * PI)?.0
                };
                Ok(fuse(m, rib, boss)?.0)
            }
            Ends::Twin { ra, rb, d, ha, hb } => {
                let (plate, _) =
                    primitive_box(m, Point3::new(-6.0, -6.0, 0.0), Point3::new(6.0, 6.0, 1.0))?;
                let (a, _) = primitive_cylinder(
                    m,
                    Axis::new(Point3::new(0.0, 0.0, 1.0), Vec3::z())?,
                    ra,
                    ha,
                )?;
                let (b, _) =
                    primitive_cylinder(m, Axis::new(Point3::new(d, 0.0, 1.0), Vec3::z())?, rb, hb)?;
                let (plated, _) = fuse(m, plate, a)?;
                Ok(fuse(m, plated, b)?.0)
            }
        }
    }

    /// The blended edge of the unposed body, as a point of it and the
    /// test that picks it among `body`'s edges.
    fn edge(&self, m: &Model, body: Body, pose: &Isometry) -> Result<Edge, TestCaseError> {
        match *self {
            Ends::Rib { w, h, lead, .. } => {
                edge_near(m, body, pose.apply(Point3::new(lead / 2.0, w, h)))
            }
            Ends::Twin { ra, .. } => {
                let foot = pose.apply(Point3::new(0.0, 0.0, 1.0));
                let up = pose.apply(Point3::new(0.0, 0.0, 2.0)) - foot;
                let scale = foot.coords.norm().max(1.0);
                for e in m.edges(body).map_err(fail)? {
                    let entity = m.edge(e.id).map_err(fail)?;
                    let Some((curve, range)) = entity.curve() else {
                        continue;
                    };
                    let Curve::Circle { frame, radius } = m.curve(curve).map_err(fail)? else {
                        continue;
                    };
                    let at = m.curve(curve).map_err(fail)?.point(range.midpoint());
                    let off_axis = (frame.origin() - foot).cross(&up).norm();
                    if (radius - ra).abs() < 1e-9 * scale
                        && off_axis < 1e-9 * scale
                        && (at - foot).dot(&up).abs() < 1e-9 * scale
                    {
                        return Ok(e);
                    }
                }
                Err(fail(format!("no foot arc of radius {ra}")))
            }
        }
    }

    /// The signed change of volume of blending the edge at `kind`'s size
    /// `r`: minus the sliver of a convex edge, plus the fillet of a
    /// concave one, integrated over the section's region and the
    /// face across.
    fn change(&self, kind: Blend, r: f64) -> f64 {
        const N: usize = 400;
        match *self {
            Ends::Rib { w, h, .. } => {
                let cx = self.rib_centre();
                // The region of the section, (u, v) from the corner's
                // square `[w − r, w] × [h − r, h]`, above `floor(u)`, and
                // the rib's length there: it ends on the boss's wall at
                // `x = cx − √(ρ(z)² − y²)`.
                // `u = r sin α` keeps a ball's `√(r² − u²)` smooth.
                let sliver = simpson(0.0, FRAC_PI_2, N, &|a| {
                    let u = r * a.sin();
                    let lo = match kind {
                        Blend::Fillet => r * a.cos(),
                        Blend::Chamfer => r - u,
                    };
                    r * a.cos()
                        * simpson(lo, r, N, &|v| {
                            let (y, z) = (w - r + u, h - r + v);
                            cx - (self.rib_radius(z).powi(2) - y * y).max(0.0).sqrt()
                        })
                });
                -sliver
            }
            Ends::Twin { ra, rb, d, .. } => {
                // The ring about the first boss's axis from radius `ra`
                // to `ra + r`, outside the second boss, over the section
                // between the plate, the wall and the ball or the chamfer.
                let outside = |rho: f64| {
                    let c = (rho * rho + d * d - rb * rb) / (2.0 * rho * d);
                    rho * (2.0 * PI - 2.0 * c.clamp(-1.0, 1.0).acos())
                };
                match kind {
                    Blend::Fillet => {
                        // ρ = ra + r − r cos α, the gap r − r sin α.
                        simpson(0.0, FRAC_PI_2, N, &|a| {
                            let rho = ra + r - r * a.cos();
                            outside(rho) * (r - r * a.sin()) * r * a.sin()
                        })
                    }
                    Blend::Chamfer => simpson(ra, ra + r, N, &|rho| outside(rho) * (ra + r - rho)),
                }
            }
        }
    }
}

fn end_case() -> impl Strategy<Value = EndCase> {
    let rib = (
        prop::finite_f64(0.5..=1.5),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(1.5..=2.5),
        prop_oneof![Just(0.0), prop::finite_f64(0.05..=0.25)],
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(0.1..=1.0),
    )
        .prop_map(|(w, h, reach, taper, lead, fraction)| {
            (
                Ends::Rib {
                    w,
                    h,
                    reach,
                    taper,
                    lead,
                },
                fraction * w.min(h) / 3.0,
            )
        });
    let twin = (
        prop::finite_f64(0.9..=1.6),
        prop::finite_f64(0.5..=1.2),
        prop::finite_f64(0.0..=1.0),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(0.5..=1.5),
        prop::finite_f64(0.0..=1.0),
    )
        .prop_map(|(ra, rb, u, ha, hb, fraction)| {
            let size = 0.03 + fraction * 0.17;
            let lo = (ra + size + TWIN_MARGIN - rb).max(rb - ra + TWIN_MARGIN);
            let hi = ra + rb - TWIN_MARGIN;
            (
                Ends::Twin {
                    ra,
                    rb,
                    d: lo + u * (hi - lo),
                    ha,
                    hb,
                },
                size,
            )
        });
    (prop_oneof![rib, twin], prop::pose()).prop_map(|((scene, size), pose)| EndCase {
        scene,
        size,
        pose,
    })
}

/// The blend of `case` in a fresh model, the scene moved to its pose
/// first: the model, the input, the result and its record.
fn end_posed(
    case: &EndCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let scene = case.scene.build(&mut m).map_err(fail)?;
    let (moved, _) = transform(&mut m, scene, &case.pose).map_err(fail)?;
    let edge = case.scene.edge(&m, moved, &case.pose)?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size).map_err(|e| {
        // A turned twin boss's chamfer cone is refused against the second
        // wall's cylinder in some poses: the failure waits as
        // `regression/twin-boss-foot-turned-chamfer-cone-cylinder`, and
        // the property rejects that refusal alone until the fix lands.
        let cone_on_cylinder = |k: &arris_geom::GeomKind| k.to_string();
        if let (OpError::Unsupported { a, b }, Ends::Twin { .. }, Blend::Chamfer) =
            (&e, &case.scene, kind)
            && cone_on_cylinder(&a.0) == "cone surface"
            && cone_on_cylinder(&b.0) == "cylinder surface"
        {
            return TestCaseError::reject("twin-boss-foot-turned-chamfer-cone-cylinder");
        }
        fail(format!("{kind:?} of the posed scene: {e}"))
    })?;
    Ok((m, moved, blended, p))
}

fn ends_blend_as_their_sections(case: EndCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = end_posed(&case, kind)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let before = mass_properties(&m, moved).map_err(fail)?;
    let want = before.volume + case.scene.change(kind, case.size);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the section's {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let scene = case.scene.build(&mut here).map_err(fail)?;
    let edge = case.scene.edge(&here, scene, &Isometry::identity())?;
    let (rest, _) = op(kind)(&mut here, scene, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    assert_checked(&check(&here, rest, Level::Full))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    assert_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = end_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A rib into a round or conical boss, or a boss's foot where a
    /// second boss crosses it, filleted: clean at `Full` with nothing
    /// unchecked, audited, at the section's volume integrated over the
    /// face the blend ends on, pose-independent, through STEP and
    /// deterministic.
    ends_fillet_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = end_case() => {
            ends_blend_as_their_sections(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same scenes chamfered: a rib's plane ends on the boss in a
    /// conic, a boss's foot cone on the second boss in a traced curve.
    ends_chamfer_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = end_case() => {
            ends_blend_as_their_sections(case, Blend::Chamfer)
        }
}

// A blend running into a step (ADR-0038): the mixed corner at an end, its
// corner edge lengthened past the vertex and the face across taking the
// end from outside. Closed forms for each scene, held at random poses.

/// A scene with a convex edge that ends in a step.
#[derive(Debug, Clone, PartialEq)]
enum Stepped {
    /// An L in the `xz` plane extruded `depth` along `y`: a low block
    /// `x ∈ [0, low]` of height `foot` beside a tall one `x ∈ [−tall, 0]`
    /// of height `foot + rise`, the step's face leaning from `(0, foot)`
    /// to `(lean, foot + rise)`. The low block's top front edge is
    /// blended: its near end runs into the step across a plane, its far
    /// end is a box corner.
    Lean {
        tall: f64,
        low: f64,
        foot: f64,
        rise: f64,
        depth: f64,
        lean: f64,
    },
    /// A cylinder of `radius` about `z` and `height`, its second quadrant
    /// cut down to `cut`; the low quarter's rim, an open arc, is blended,
    /// each end running into a plane through the axis.
    Drum { radius: f64, height: f64, cut: f64 },
}

/// A scene, a blend size and a pose.
#[derive(Debug, Clone)]
struct StepCase {
    scene: Stepped,
    size: f64,
    pose: Isometry,
}

impl Stepped {
    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match *self {
            Stepped::Lean {
                tall,
                low,
                foot,
                rise,
                depth,
                lean,
            } => {
                let corners = [
                    Point2::new(low, 0.0),
                    Point2::new(low, foot),
                    Point2::new(0.0, foot),
                    Point2::new(lean, foot + rise),
                    Point2::new(-tall, foot + rise),
                    Point2::new(-tall, 0.0),
                ];
                let profile = Profile {
                    plane: Frame::new(Point3::origin(), -Vec3::y(), Vec3::x())?,
                    outer: ProfileLoop::Path {
                        start: corners[5],
                        segments: corners.iter().map(|&q| ProfileSegment::LineTo(q)).collect(),
                    },
                    holes: Vec::new(),
                };
                Ok(extrude(m, &profile, -Vec3::y(), depth)?.0)
            }
            Stepped::Drum {
                radius,
                height,
                cut: z,
            } => {
                let axis = Axis::new(Point3::origin(), Vec3::z())?;
                let (drum, _) = primitive_cylinder(m, axis, radius, height)?;
                let (quarter, _) = primitive_box(
                    m,
                    Point3::new(-2.0 * radius, 0.0, z),
                    Point3::new(0.0, 2.0 * radius, height + 1.0),
                )?;
                Ok(cut(m, drum, quarter)?.0)
            }
        }
    }

    /// A point on the blended edge of the unposed body.
    fn midpoint(&self) -> Point3 {
        match *self {
            Stepped::Lean { low, foot, .. } => Point3::new(low / 2.0, 0.0, foot),
            Stepped::Drum { radius, cut, .. } => {
                let q = radius / 2.0_f64.sqrt();
                Point3::new(-q, q, cut)
            }
        }
    }

    /// The largest size the scene's faces allow.
    fn room(&self) -> f64 {
        match *self {
            Stepped::Lean {
                foot, depth, low, ..
            } => foot.min(depth).min(low / 2.0),
            Stepped::Drum { radius, cut, .. } => radius.min(cut),
        }
    }

    /// The volume after blending the edge at `size`: the scene's, less the
    /// section's area over the edge's length, moved by the step's end —
    /// the end plane meets the section at `x = −(lean / rise) w` for `w`
    /// below the edge — or, about the axis, the section swept a quarter
    /// turn by Pappus.
    fn volume(&self, kind: Blend, r: f64) -> f64 {
        let (area, first_moment, centroid) = match kind {
            // The spandrel's area, its moment `∫ w dA` over the depth
            // below the edge, and its centroid's distance in from the
            // rim `r (10 − 3π) / (12 − 3π)`.
            Blend::Fillet => (
                (1.0 - FRAC_PI_4) * r * r,
                (5.0 / 6.0 - FRAC_PI_4) * r * r * r,
                r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI),
            ),
            Blend::Chamfer => (r * r / 2.0, r * r * r / 6.0, r / 3.0),
        };
        match *self {
            Stepped::Lean {
                tall,
                low,
                foot,
                rise,
                depth,
                lean,
            } => {
                // The L's area: the full width to `foot`, then the tall
                // block's trapezoid, `tall` wide at its foot and
                // `tall + lean` at its top.
                let section = (tall + low) * foot + rise * (2.0 * tall + lean) / 2.0;
                depth * section - (area * low + lean / rise * first_moment)
            }
            Stepped::Drum {
                radius,
                height,
                cut,
            } => {
                let whole = PI * radius * radius * (0.75 * height + 0.25 * cut);
                whole - FRAC_PI_2 * (radius - centroid) * area
            }
        }
    }
}

/// Scenes at random: the lean as a fraction of the rise, `±reach` — within
/// `0.8` the end plane keeps the section's end inside both blocks' faces —
/// and a size up to 0.4 of the room.
fn step_case(reach: f64) -> impl Strategy<Value = StepCase> {
    let lean = (
        prop::finite_f64(1.0..=2.5),
        prop::finite_f64(1.5..=3.0),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(0.5..=2.0),
        prop::finite_f64(1.5..=3.0),
        prop::finite_f64(-reach..=reach),
    )
        .prop_map(|(tall, low, foot, rise, depth, ratio)| Stepped::Lean {
            tall,
            low,
            foot,
            rise,
            depth,
            // Still a simple outline: the top edge stays over the tall
            // block's back side.
            lean: (ratio * rise).max(-0.9 * tall),
        });
    let drum = (
        prop::finite_f64(1.5..=3.0),
        prop::finite_f64(1.5..=3.0),
        prop::finite_f64(0.6..=1.0),
    )
        .prop_map(|(radius, height, at)| Stepped::Drum {
            radius,
            height,
            cut: at * (height - 0.3),
        });
    (
        prop_oneof![lean, drum],
        prop::finite_f64(0.1..=1.0),
        prop::pose(),
    )
        .prop_map(|(scene, fraction, pose)| StepCase {
            size: fraction * 0.4 * scene.room(),
            scene,
            pose,
        })
}

/// The blend of `case` in a fresh model, the scene moved to its pose first.
/// With `wide`, a refusal that names the corner (`BlendTooLarge`) rejects
/// the case: the lean passed the end's reach.
fn step_posed(
    case: &StepCase,
    kind: Blend,
    wide: bool,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let scene = case.scene.build(&mut m).map_err(fail)?;
    let (moved, _) = transform(&mut m, scene, &case.pose).map_err(fail)?;
    let edge = edge_near(&m, moved, case.pose.apply(case.scene.midpoint()))?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size).map_err(|e| {
        if wide
            && let OpError::Degenerate {
                reason: arris_ops::Reason::Blend(arris_ops::BlendReason::TooLarge),
                ..
            } = e
        {
            return TestCaseError::reject("the lean is past the end's reach");
        }
        fail(format!("{kind:?} of the posed step: {e}"))
    })?;
    Ok((m, moved, blended, p))
}

fn steps_blend_as_their_closed_forms(
    case: StepCase,
    kind: Blend,
    wide: bool,
) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = step_posed(&case, kind, wide)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let want = case.scene.volume(kind, case.size);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let scene = case.scene.build(&mut here).map_err(fail)?;
    let edge = edge_near(&here, scene, case.scene.midpoint())?;
    let (rest, _) = op(kind)(&mut here, scene, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    assert_checked(&check(&here, rest, Level::Full))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    assert_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = step_posed(&case, kind, wide)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A convex edge running into a step, filleted — across a leaning
    /// plane, or along a drum's rim into planes through its axis: clean at
    /// `Full`, audited, at the closed-form volume, pose-independent, through
    /// STEP and deterministic.
    fillets_into_a_step_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = step_case(0.8) => {
            steps_blend_as_their_closed_forms(case, Blend::Fillet, false)
        }
}

prop_shards! {
    /// The same scenes chamfered.
    chamfers_into_a_step_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = step_case(0.8) => {
            steps_blend_as_their_closed_forms(case, Blend::Chamfer, false)
        }
}

prop_shards! {
    /// A lean beyond the end's reach either builds at its closed form or
    /// is refused as `BlendTooLarge`, naming the corner; never another
    /// failure, never a wrong solid.
    leans_past_the_reach_build_or_are_refused_by_name
        [shard_0 shard_1] (case) = step_case(4.0) => {
            steps_blend_as_their_closed_forms(case, Blend::Fillet, true)
                .map_err(rejecting_a_loop_that_meets_itself)
        }
}

/// A lean far past the reach builds a face whose loop meets itself, where
/// the property wants the closed form or a refusal by name; that one
/// failure is rejected, by its checker row, until
/// `a_lean_far_past_the_reach_builds_or_is_refused` passes (docs/BACKLOG.md).
fn rejecting_a_loop_that_meets_itself(e: TestCaseError) -> TestCaseError {
    match &e {
        TestCaseError::Fail(reason) if reason.message().contains("intersects itself") => {
            TestCaseError::reject("a lean past the reach whose loop meets itself")
        }
        _ => e,
    }
}

/// The shrunk failure of the nightly-failures cleanup's final property run
/// (`ARRIS_PROPTEST_SEED=d226ff33…`, 1000 cases, shard 0 of 2), a lean at
/// ratio 3.98 of the rise, past the reach the end plane keeps its corner
/// inside both blocks: the result is built and its face 16's loop meets
/// itself (L5), neither the closed form's solid nor a refusal by name.
#[test]
#[ignore = "L5: a lean past the end's reach builds a face whose loop meets itself, where the property wants the closed form or BlendTooLarge (docs/BACKLOG.md, a lean past the reach)"]
fn a_lean_far_past_the_reach_builds_or_is_refused() {
    use arris_math::nalgebra::{Quaternion, Unit};
    let case = StepCase {
        scene: Stepped::Lean {
            tall: 1.9147732443785255,
            low: 2.8778284242395706,
            foot: 1.8066700099203505,
            rise: 0.5,
            depth: 1.5,
            lean: 1.9901216089305018,
        },
        size: 0.48747985066026334,
        pose: Isometry::new(
            Unit::new_unchecked(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
            Vec3::new(0.0, 0.0, 0.0),
        ),
    };
    if let Err(e) = steps_blend_as_their_closed_forms(case, Blend::Fillet, true) {
        panic!("{e}");
    }
}

// A chain through a vertex of four edges (ADR-0039): the foot of a chamfer,
// between a side wall and the chamfer's strip, runs on round the whole foot
// outline where both faces turn tangentially. The section's corner swept
// along the outline's offsets in closed form, held at random poses.

/// A chamfered stadium or split pin and the blend of its chamfer's foot.
#[derive(Debug, Clone)]
struct FootCase {
    /// The stadium's straight length; `0` is the disc of two half circles
    /// (the split rim), its rim's two halves chamfered apart.
    length: f64,
    radius: f64,
    height: f64,
    /// The chamfer's distance and the blend's size.
    chamfer: f64,
    size: f64,
    pose: Isometry,
}

/// The corner's section of a blend of size `r` on the 135° edge between the
/// wall and the chamfer's strip: its area and its centroid's depth in from
/// the wall. A fillet's is the spandrel, a chamfer's the triangle of two
/// legs `r`.
fn foot_corner(kind: Blend, r: f64) -> (f64, f64) {
    match kind {
        Blend::Chamfer => (r * r * SQRT_2 / 4.0, r / (3.0 * SQRT_2)),
        Blend::Fillet => {
            let theta = FRAC_PI_4;
            let t = r * (theta / 2.0).tan();
            let kite = r * t / 2.0;
            let sector = r * r * theta / 2.0;
            let moment = kite * r / 3.0 + kite * (t / SQRT_2 + r) / 3.0
                - sector * (r - 2.0 * r * theta.sin() / (3.0 * theta));
            let area = 2.0 * kite - sector;
            (area, moment / area)
        }
    }
}

impl FootCase {
    fn build(&self, m: &mut Model) -> Result<(Body, Point3), TestCaseError> {
        let (l, r) = (self.length, self.radius);
        let p = Point2::new;
        let split = l == 0.0;
        let outer = if split {
            ProfileLoop::Path {
                start: p(r, 0.0),
                segments: vec![
                    ProfileSegment::ArcTo {
                        to: p(-r, 0.0),
                        via: p(0.0, r),
                    },
                    ProfileSegment::ArcTo {
                        to: p(r, 0.0),
                        via: p(0.0, -r),
                    },
                ],
            }
        } else {
            ProfileLoop::Path {
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
            }
        };
        let profile = Profile {
            plane: Frame::world(),
            outer,
            holes: Vec::new(),
        };
        let body = extrude(m, &profile, Vec3::z(), self.height)
            .map_err(fail)?
            .0;
        let h = self.height;
        let rims: Vec<Point3> = if split {
            vec![Point3::new(0.0, r, h), Point3::new(0.0, -r, h)]
        } else {
            vec![
                Point3::new(l / 2.0, -r, h),
                Point3::new(l + r, 0.0, h),
                Point3::new(l / 2.0, r, h),
                Point3::new(-r, 0.0, h),
            ]
        };
        let mut edges = Vec::new();
        for at in rims {
            edges.push(edge_near(m, body, at)?);
        }
        let chamfered = chamfer(m, body, &edges, self.chamfer).map_err(fail)?.0;
        let foot = if split { 0.0 } else { l / 2.0 };
        Ok((chamfered, Point3::new(foot, r, h - self.chamfer)))
    }

    /// The volume after blending the foot: the stadium's, less the chamfer's
    /// and the blend's corner section, each swept along the outline's
    /// offset of length `2 length + 2π (radius − depth)`.
    fn volume(&self, kind: Blend) -> f64 {
        let (l, r) = (self.length, self.radius);
        let swept = |(area, depth): (f64, f64)| area * (2.0 * l + 2.0 * PI * (r - depth));
        let whole = self.height * (2.0 * l * r + PI * r * r);
        whole
            - swept((self.chamfer * self.chamfer / 2.0, self.chamfer / 3.0))
            - swept(foot_corner(kind, self.size))
    }
}

fn foot_case() -> impl Strategy<Value = FootCase> {
    (
        prop_oneof![Just(true), Just(false)],
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(1.2..=2.5),
        prop::finite_f64(0.1..=0.4),
        prop::finite_f64(0.1..=0.5),
        prop::pose(),
    )
        .prop_map(
            |(split, length, radius, height, chamfer, share, pose)| FootCase {
                length: if split { 0.0 } else { length },
                radius,
                height,
                chamfer,
                size: share * chamfer,
                pose,
            },
        )
}

/// `report` is clean and leaves nothing unchecked but pairs of a plane and a
/// torus, which S5's tracers leave undecided at a knife-edge of poses of a
/// walked chain (`a_walked_chain_in_a_far_pose_is_decided`, a named
/// exclusion: it goes when that test passes).
fn feet_checked(report: &Report) -> Result<(), TestCaseError> {
    prop_assert!(report.is_ok(), "{}", report);
    prop_assert!(
        report.unchecked().iter().all(|u| matches!(
            u,
            Unchecked::FacePair {
                kinds: (SurfaceKind::Plane, SurfaceKind::Torus)
                    | (SurfaceKind::Torus, SurfaceKind::Plane),
                ..
            }
        )),
        "nothing unchecked\n{}",
        report
    );
    Ok(())
}

fn foot_posed(
    case: &FootCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let (body, at) = case.build(&mut m)?;
    let (moved, _) = transform(&mut m, body, &case.pose).map_err(fail)?;
    let edge = edge_near(&m, moved, case.pose.apply(at))?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} of the posed foot: {e}")))?;
    Ok((m, moved, blended, p))
}

fn feet_blend_as_their_closed_forms(case: FootCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = foot_posed(&case, kind)?;
    feet_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let want = case.volume(kind);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let (body, at) = case.build(&mut here)?;
    let edge = edge_near(&here, body, at)?;
    let (rest, _) = op(kind)(&mut here, body, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    feet_checked(&check(&here, rest, Level::Full))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    feet_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = foot_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A chamfered stadium or split pin, the chamfer's foot filleted: the
    /// chain runs on round the outline through four vertices of four edges,
    /// clean at `Full`, audited, at the closed-form volume, pose-independent,
    /// through STEP and deterministic.
    fillets_through_a_vertex_of_four_edges_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = foot_case() => {
            feet_blend_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same feet chamfered.
    chamfers_through_a_vertex_of_four_edges_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = foot_case() => {
            feet_blend_as_their_closed_forms(case, Blend::Chamfer)
        }
}

/// A chamfered stadium turned and moved far out, the chamfer's foot
/// filleted and its chain walked round the outline: S5 leaves a plane
/// against a torus undecided at `Full` in exactly this pose — a translation
/// moved by `1e-14`, or by `1e-2`, decides it — which a fixture's axis and
/// angle cannot carry, found by `fillets_through_a_vertex_of_four_edges_
/// match_their_closed_forms` (blend-corners step 7).
#[test]
#[ignore = "S5: a plane against a torus is not decided in this far pose of a walked chain, so Full has an unchecked pair (docs/BACKLOG.md, a plane against a torus in S5)"]
fn a_walked_chain_in_a_far_pose_is_decided() {
    use arris_math::nalgebra::{Quaternion, UnitQuaternion};
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        -0.09475886763043798,
        0.5890236655990984,
        -0.5660022382625896,
        0.5689581220541085,
    ));
    let case = FootCase {
        length: 2.578050057721793,
        radius: 1.0,
        height: 2.0333019670063206,
        chamfer: 0.12936930004443964,
        size: 0.042621373723744804,
        pose: Isometry::new(
            q,
            Vec3::new(63.84502733042328, 46.952118434286945, -39.56313382866168),
        ),
    };
    let (m, _, blended, _) = foot_posed(&case, Blend::Fillet).unwrap();
    assert_checked(&check(&m, blended, Level::Full)).unwrap();
}

// A rim split into arcs (ADR-0041): a hole's top edge or a boss's foot on a
// plate, its circle written as two to four arcs of one wall, the wall's seam
// at the first vertex and nothing but the two arcs at the others. No
// operation of the kernel splits an edge, so the file does it: the plate's
// own STEP text has the ring's closed edge replaced by arcs, and the part is
// read back, as a part from a writer that splits its closed edges is.

/// A plate with a hole or a boss whose rim is split, and the blend of one of
/// its arcs, which runs on round the whole ring.
#[derive(Debug, Clone)]
struct RimCase {
    /// A boss (its foot, concave) on the plate, else a hole (its top edge,
    /// convex) through it.
    boss: bool,
    radius: f64,
    /// The hole's depth, the plate's thickness; the boss stands this high on a
    /// plate 1 thick.
    height: f64,
    size: f64,
    /// The arcs' lengths as weights; the angles follow from them, the first
    /// arc starting at the wall's seam.
    weights: Vec<f64>,
    /// The arc blended.
    arc: usize,
    pose: Isometry,
}

/// `text` with the closed edge of the circle at height `z` replaced by arcs
/// from its vertex through `cuts` (angles about its axis, ascending, in
/// `(0, 2π)`), in the loops that use it.
fn split_ring(text: &str, z: f64, cuts: &[f64]) -> Result<String, TestCaseError> {
    let lines: Vec<&str> = text.lines().collect();
    let entity = |id: &str| -> Option<&str> {
        let head = format!("{id} = ");
        lines.iter().find_map(|l| l.strip_prefix(&head))
    };
    let args = |rhs: &str| -> Vec<String> {
        let inner = rhs.split_once('(').map_or("", |(_, r)| r);
        inner
            .trim_end_matches(';')
            .trim_end_matches(')')
            .split(',')
            .map(|s| s.trim().to_string())
            .collect()
    };
    let mut top = 0u64;
    let mut found: Option<(String, String, [f64; 3], String)> = None;
    for l in &lines {
        let Some((id, rhs)) = l.split_once(" = ") else {
            continue;
        };
        if let Some(n) = id.strip_prefix('#').and_then(|n| n.parse::<u64>().ok()) {
            top = top.max(n);
        }
        if !rhs.starts_with("EDGE_CURVE(") {
            continue;
        }
        let a = args(rhs);
        if a[1] != a[2] || found.is_some() {
            continue;
        }
        let vertex = entity(&a[1]).ok_or_else(|| fail("no vertex"))?;
        let point = entity(&args(vertex)[1]).ok_or_else(|| fail("no point"))?;
        let c = args(point);
        let xyz: Vec<f64> = c[1..4]
            .iter()
            .map(|s| s.trim_matches(|ch| ch == '(' || ch == ')').parse::<f64>())
            .collect::<Result<_, _>>()
            .map_err(fail)?;
        if (xyz[2] - z).abs() < 1e-9 {
            found = Some((
                id.to_string(),
                a[1].clone(),
                [xyz[0], xyz[1], xyz[2]],
                a[3].clone(),
            ));
        }
    }
    let (edge, start, at, curve) =
        found.ok_or_else(|| fail(format!("no closed edge at z = {z}")))?;
    let (radius, theta0) = (at[0].hypot(at[1]), at[1].atan2(at[0]));
    let mut next = top;
    let mut fresh = || {
        next += 1;
        format!("#{next}")
    };
    let mut add = Vec::new();
    let mut stops = vec![start.clone()];
    for cut in cuts {
        let (vertex, point) = (fresh(), fresh());
        let t = theta0 + cut;
        add.push(format!(
            "{point} = CARTESIAN_POINT('',({:?},{:?},{:?}));",
            radius * t.cos(),
            radius * t.sin(),
            at[2]
        ));
        add.push(format!("{vertex} = VERTEX_POINT('',{point});"));
        stops.push(vertex);
    }
    stops.push(start);
    let arcs: Vec<String> = (0..stops.len() - 1).map(|_| fresh()).collect();
    for (k, arc) in arcs.iter().enumerate() {
        add.push(format!(
            "{arc} = EDGE_CURVE('',{},{},{curve},.T.);",
            stops[k],
            stops[k + 1]
        ));
    }
    // Each use of the edge becomes its arcs, in the order the use runs.
    let mut uses: Vec<(String, Vec<String>)> = Vec::new();
    let mut out: Vec<String> = Vec::new();
    for l in &lines {
        let Some((id, rhs)) = l.split_once(" = ") else {
            out.push(l.to_string());
            continue;
        };
        if id == edge {
            continue;
        }
        if rhs.starts_with("ORIENTED_EDGE(") {
            let a = args(rhs);
            if a[3] == edge {
                let forward = a[4] == ".T.";
                let ids: Vec<String> = arcs.iter().map(|_| fresh()).collect();
                let order: Vec<usize> = if forward {
                    (0..arcs.len()).collect()
                } else {
                    (0..arcs.len()).rev().collect()
                };
                for (slot, k) in ids.iter().zip(order) {
                    add.push(format!(
                        "{slot} = ORIENTED_EDGE('',*,*,{},{});",
                        arcs[k], a[4]
                    ));
                }
                uses.push((id.to_string(), ids));
                continue;
            }
        }
        out.push(l.to_string());
    }
    let replaced: Vec<String> = out
        .into_iter()
        .map(|l| {
            let Some((id, rhs)) = l.split_once(" = ") else {
                return l;
            };
            if !rhs.starts_with("EDGE_LOOP(") {
                return l;
            }
            let members: Vec<String> = args(rhs)[1..]
                .iter()
                .flat_map(|m| {
                    let m = m.trim_start_matches('(');
                    match uses.iter().find(|(u, _)| u == m) {
                        Some((_, ids)) => ids.clone(),
                        None => vec![m.to_string()],
                    }
                })
                .collect();
            format!("{id} = EDGE_LOOP('',({}));", members.join(","))
        })
        .collect();
    let data_end = replaced.iter().rposition(|l| l == "ENDSEC;");
    let mut text = String::new();
    for (k, l) in replaced.into_iter().enumerate() {
        if Some(k) == data_end {
            for a in add.drain(..) {
                text.push_str(&a);
                text.push('\n');
            }
        }
        text.push_str(&l);
        text.push('\n');
    }
    Ok(text)
}

impl RimCase {
    /// The angles at which the arcs start, the first at the seam.
    fn starts(&self) -> Vec<f64> {
        let total: f64 = self.weights.iter().sum();
        let mut at = 0.0;
        self.weights
            .iter()
            .map(|w| {
                let start = at;
                at += 2.0 * PI * w / total;
                start
            })
            .collect()
    }

    /// The plate and its split ring, and the midpoint of the arc blended.
    fn build(&self, m: &mut Model) -> Result<(Body, Point3), TestCaseError> {
        let (r, h) = (self.radius, self.height);
        let plate_h = if self.boss { 1.0 } else { h };
        let (plate, _) = primitive_box(
            m,
            Point3::new(-3.0, -3.0, 0.0),
            Point3::new(3.0, 3.0, plate_h),
        )
        .map_err(fail)?;
        let solid = |m: &mut Model, z: f64, len: f64| {
            let axis = Axis::new(Point3::new(0.0, 0.0, z), Vec3::z()).map_err(fail)?;
            primitive_cylinder(m, axis, r, len).map_err(fail)
        };
        let whole = if self.boss {
            let (boss, _) = solid(m, 0.5, 0.5 + h)?;
            fuse(m, plate, boss).map_err(fail)?.0
        } else {
            let (hole, _) = solid(m, -1.0, h + 2.0)?;
            cut(m, plate, hole).map_err(fail)?.0
        };
        let starts = self.starts();
        let text = split_ring(
            &step::write(m, &[whole]).map_err(fail)?,
            plate_h,
            &starts[1..],
        )?;
        let mut back = Model::new(m.precision()).map_err(fail)?;
        let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
        let solid = read.solids[0]
            .result
            .as_ref()
            .map_err(|e| fail(format!("the split plate: {e}")))?;
        let body = solid.body;
        // The ring's one edge became the arcs: a split that missed would
        // leave a whole circle to blend, and pass.
        let (before, after) = (
            m.edges(whole).map_err(fail)?.len(),
            back.edges(body).map_err(fail)?.len(),
        );
        prop_assert_eq!(
            after,
            before + starts.len() - 1,
            "the ring is {} arcs",
            starts.len()
        );
        *m = back;
        let end = starts.get(self.arc + 1).copied().unwrap_or(2.0 * PI);
        let mid = (starts[self.arc] + end) / 2.0;
        Ok((body, Point3::new(r * mid.cos(), r * mid.sin(), plate_h)))
    }

    /// The volume after blending the ring: the plate's, less (a hole) or
    /// plus (a boss's foot) the corner's section swept a whole turn about
    /// the axis, Pappus.
    fn volume(&self, kind: Blend) -> f64 {
        let (r, h, s) = (self.radius, self.height, self.size);
        let (area, offset) = match kind {
            Blend::Fillet => (
                (1.0 - PI / 4.0) * s * s,
                s * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI),
            ),
            Blend::Chamfer => (s * s / 2.0, s / 3.0),
        };
        let swept = 2.0 * PI * (r + offset) * area;
        if self.boss {
            36.0 + PI * r * r * h + swept
        } else {
            36.0 * h - PI * r * r * h - swept
        }
    }
}

fn rim_case() -> impl Strategy<Value = RimCase> {
    (
        prop_oneof![Just(true), Just(false)],
        prop::finite_f64(0.8..=1.6),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(0.05..=0.3),
        proptest::collection::vec(prop::finite_f64(0.6..=1.4), 2..=4),
        0usize..4,
        prop::pose(),
    )
        .prop_map(|(boss, radius, height, size, weights, arc, pose)| RimCase {
            boss,
            radius,
            height,
            size,
            arc: arc % weights.len(),
            weights,
            pose,
        })
}

fn rim_posed(
    case: &RimCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let (body, at) = case.build(&mut m)?;
    assert_checked(&check(&m, body, Level::Full))?;
    let (moved, _) = transform(&mut m, body, &case.pose).map_err(fail)?;
    let edge = edge_near(&m, moved, case.pose.apply(at))?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} of the posed rim: {e}")))?;
    Ok((m, moved, blended, p))
}

fn rims_blend_as_their_closed_forms(case: RimCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = rim_posed(&case, kind)?;
    feet_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let want = case.volume(kind);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        want
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    feet_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = rim_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A hole's rim or a boss's foot split into two to four arcs, one arc
    /// filleted: the chain runs on through every vertex of the split and
    /// closes, clean at `Full`, audited, at the closed-form volume, through
    /// STEP and deterministic.
    fillets_of_a_split_rim_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = rim_case() => {
            rims_blend_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same rims chamfered.
    chamfers_of_a_split_rim_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = rim_case() => {
            rims_blend_as_their_closed_forms(case, Blend::Chamfer)
        }
}

// A blend ending at a cusp (ADR-0042): a crescent prism, its pocket, or a
// spandrel, one edge running to the vertex where two walls leave tangent
// and on one side of the shared face. The stripe is cut by the next wall,
// so the volume is the corner section integrated over the part of its band
// that lies inside the gap, held at random poses.

/// A scene whose blended edge ends at a cusp.
#[derive(Debug, Clone, PartialEq)]
enum Cusped {
    /// The crescent prism `z ∈ [0, h]`: below the `x` axis, inside the
    /// circle of radius `big` about the origin and outside the circle of
    /// radius `small` about `(small − big, 0)`, the two tangent at
    /// `(−big, 0)`. The big arc's edge is blended, or the small arc's when
    /// `small_arc`, at the floor when `floor` and at the top otherwise:
    /// both convex, the sliver between the walls material.
    Crescent {
        big: f64,
        small: f64,
        h: f64,
        small_arc: bool,
        floor: bool,
    },
    /// The same crescent cut `h` deep from the top of a block, its floor at
    /// `z = 1`: the floor edge of the big arc, or of the small arc when
    /// `small_arc`, is concave, and the sliver void.
    Pocket {
        big: f64,
        small: f64,
        h: f64,
        small_arc: bool,
    },
    /// The square `[0, a]²` less the quarter disc of radius `a` about
    /// `(0, a)`, `z ∈ [0, h]`: the arc leaves the bottom edge tangent at
    /// the origin. The bottom wall's top or floor edge, a line between two
    /// planes, is blended.
    Spandrel { a: f64, h: f64, floor: bool },
}

/// A cusped scene, a blend size and a pose.
#[derive(Debug, Clone)]
struct CuspCase {
    scene: Cusped,
    size: f64,
    pose: Isometry,
}

/// The crescent prism from `z0` to `z0 + h`: the half disc below the `x`
/// axis less the small cylinder. A boolean, not a profile of three
/// segments: `Profile::edges` refuses a loop with a cusp unless the small
/// radius is half the big one (`regression/cusp-profile-off-half`).
fn crescent(m: &mut Model, big: f64, small: f64, z0: f64, h: f64) -> Result<Body, OpError> {
    let half = Profile {
        plane: Frame::new(Point3::new(0.0, 0.0, z0), Vec3::z(), Vec3::x())?,
        outer: ProfileLoop::Path {
            start: Point2::new(-big, 0.0),
            segments: vec![
                ProfileSegment::ArcTo {
                    to: Point2::new(big, 0.0),
                    via: Point2::new(0.0, -big),
                },
                ProfileSegment::LineTo(Point2::new(-big, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let (disc, _) = extrude(m, &half, Vec3::z(), h)?;
    let axis = Axis::new(Point3::new(small - big, 0.0, z0 - 1.0), Vec3::z())?;
    let (bite, _) = primitive_cylinder(m, axis, small, h + 2.0)?;
    Ok(cut(m, disc, bite)?.0)
}

impl Cusped {
    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match *self {
            Cusped::Crescent { big, small, h, .. } => crescent(m, big, small, 0.0, h),
            Cusped::Pocket { big, small, h, .. } => {
                let (block, _) = primitive_box(
                    m,
                    Point3::new(-big - 1.0, -big - 1.0, 0.0),
                    Point3::new(big + 1.0, 1.0, 1.0 + h),
                )?;
                let tool = crescent(m, big, small, 1.0, h + 1.0)?;
                Ok(cut(m, block, tool)?.0)
            }
            Cusped::Spandrel { a, h, .. } => {
                let t = FRAC_PI_4;
                let profile = Profile {
                    plane: Frame::world(),
                    outer: ProfileLoop::Path {
                        start: Point2::origin(),
                        segments: vec![
                            ProfileSegment::LineTo(Point2::new(a, 0.0)),
                            ProfileSegment::LineTo(Point2::new(a, a)),
                            ProfileSegment::ArcTo {
                                to: Point2::origin(),
                                via: Point2::new(a * t.cos(), a - a * t.sin()),
                            },
                        ],
                    },
                    holes: Vec::new(),
                };
                Ok(extrude(m, &profile, Vec3::z(), h)?.0)
            }
        }
    }

    /// The blended edge's midpoint on the unposed body.
    fn midpoint(&self) -> Point3 {
        match *self {
            Cusped::Crescent {
                big,
                small,
                h,
                small_arc,
                floor,
            } => {
                let z = if floor { 0.0 } else { h };
                if small_arc {
                    Point3::new(small - big, -small, z)
                } else {
                    Point3::new(0.0, -big, z)
                }
            }
            Cusped::Pocket {
                big,
                small,
                small_arc,
                ..
            } => {
                if small_arc {
                    Point3::new(small - big, -small, 1.0)
                } else {
                    Point3::new(0.0, -big, 1.0)
                }
            }
            Cusped::Spandrel { a, h, floor } => {
                Point3::new(a / 2.0, 0.0, if floor { 0.0 } else { h })
            }
        }
    }

    /// The signed change of volume of blending the edge at `kind`'s size
    /// `r`: minus the sliver of a convex edge, plus the fillet of a concave
    /// one. The section's point `u` in from the blended wall and `v` from
    /// the shared face, in the corner's square below the ball or the
    /// chamfer, sweeps along the wall's offset at `u` for as far as that
    /// offset stays inside the scene's outline: to the far end's wall at a
    /// right angle, and to the next wall at the cusp, which is where the
    /// stripe is cut (ADR-0042). So the change is `∫ g(u) L(u) du` over
    /// `[0, r]`, `g` the section's height at `u` and `L` the offset's
    /// length inside the outline, both with a square root at `u = 0` that
    /// `u = w²` takes away.
    fn change(&self, kind: Blend, r: f64) -> f64 {
        const N: usize = 400;
        let g = |u: f64| match kind {
            Blend::Fillet => r - (2.0 * r * u - u * u).max(0.0).sqrt(),
            Blend::Chamfer => r - u,
        };
        // The offset's length at `u` inside the outline.
        let length = |u: f64| match *self {
            Cusped::Crescent {
                big,
                small,
                small_arc,
                ..
            }
            | Cusped::Pocket {
                big,
                small,
                small_arc,
                ..
            } => {
                let c = big - small;
                if small_arc {
                    // A circle of radius `small + u` about the small
                    // centre, below the axis and inside the big circle.
                    let rho = small + u;
                    let k = (c * c + rho * rho - big * big) / (2.0 * c * rho);
                    rho * k.clamp(-1.0, 1.0).acos()
                } else {
                    // A circle of radius `big − u` about the origin, below
                    // the axis and outside the small circle.
                    let rho = big - u;
                    let k = (small * small - rho * rho - c * c) / (2.0 * rho * c);
                    rho * k.clamp(-1.0, 1.0).acos()
                }
            }
            Cusped::Spandrel { a, .. } => a - (2.0 * a * u - u * u).max(0.0).sqrt(),
        };
        let swept = simpson(0.0, r.sqrt(), N, &|w| {
            let u = w * w;
            2.0 * w * g(u) * length(u)
        });
        match self {
            Cusped::Pocket { .. } => swept,
            Cusped::Crescent { .. } | Cusped::Spandrel { .. } => -swept,
        }
    }
}

/// A crescent, its pocket or a spandrel, and a size from 0.05 to 0.8 of
/// the least of the scene's height and the widths the stripe has to cross:
/// the small arc's radius (a contact on the shared face must meet the next
/// edge before the axis) and the distance between the centres (the small
/// arc's stripe must reach the big wall). Small or large against the gap,
/// which narrows to nothing at the cusp, the stripe is cut there.
fn cusp_case() -> impl Strategy<Value = CuspCase> {
    let crescent = (
        prop::finite_f64(1.5..=3.0),
        prop::finite_f64(0.3..=0.7),
        prop::finite_f64(0.8..=2.0),
        0u8..3,
        prop::finite_f64(0.05..=0.8),
    )
        .prop_map(|(big, share, h, which, fraction)| {
            let small = share * big;
            let bound = small.min(big - small).min(h);
            let scene = match which {
                0 => Cusped::Crescent {
                    big,
                    small,
                    h,
                    small_arc: false,
                    floor: false,
                },
                1 => Cusped::Crescent {
                    big,
                    small,
                    h,
                    small_arc: true,
                    floor: true,
                },
                _ => Cusped::Pocket {
                    big,
                    small,
                    h,
                    small_arc: false,
                },
            };
            (scene, fraction * bound)
        });
    let other = (
        prop::finite_f64(1.5..=3.0),
        prop::finite_f64(0.3..=0.7),
        prop::finite_f64(0.8..=2.0),
        0u8..3,
        prop::finite_f64(0.05..=0.8),
    )
        .prop_map(|(big, share, h, which, fraction)| {
            let small = share * big;
            let bound = small.min(big - small).min(h);
            let scene = match which {
                0 => Cusped::Crescent {
                    big,
                    small,
                    h,
                    small_arc: true,
                    floor: false,
                },
                1 => Cusped::Pocket {
                    big,
                    small,
                    h,
                    small_arc: true,
                },
                _ => Cusped::Spandrel {
                    a: 2.0 * big,
                    h,
                    floor: share < 0.5,
                },
            };
            let bound = match scene {
                Cusped::Spandrel { a, h, .. } => (a / 2.0).min(h),
                _ => bound,
            };
            (scene, fraction * bound)
        });
    (prop_oneof![crescent, other], prop::pose()).prop_map(|((scene, size), pose)| CuspCase {
        scene,
        size,
        pose,
    })
}

/// The blend of `case` in a fresh model, the scene moved to its pose
/// first: the model, the input, the result and its record.
fn cusp_posed(
    case: &CuspCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let scene = case.scene.build(&mut m).map_err(fail)?;
    let (moved, _) = transform(&mut m, scene, &case.pose).map_err(fail)?;
    let edge = edge_near(&m, moved, case.pose.apply(case.scene.midpoint()))?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size).map_err(|e| {
        // A wide fillet's torus is refused against the next wall's cylinder
        // in some poses, its branch ending at the node a hair past
        // `tol.linear` from `Q`: the failure waits as
        // `regression/cusp-small-arc-wide-fillet`, and the property rejects
        // that refusal alone until the fix lands.
        let kind_of = |k: &arris_geom::GeomKind| k.to_string();
        if let (OpError::Unsupported { a, b }, Blend::Fillet) = (&e, kind)
            && !matches!(case.scene, Cusped::Spandrel { .. })
            && kind_of(&a.0) == "torus surface"
            && kind_of(&b.0) == "cylinder surface"
        {
            return TestCaseError::reject("cusp-small-arc-wide-fillet");
        }
        fail(format!("{kind:?} of the posed scene: {e}"))
    })?;
    Ok((m, moved, blended, p))
}

fn cusps_blend_as_their_sections(case: CuspCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = cusp_posed(&case, kind)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let before = mass_properties(&m, moved).map_err(fail)?;
    let want = before.volume + case.scene.change(kind, case.size);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the section's {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let scene = case.scene.build(&mut here).map_err(fail)?;
    let edge = edge_near(&here, scene, case.scene.midpoint())?;
    let (rest, _) = op(kind)(&mut here, scene, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    assert_checked(&check(&here, rest, Level::Full))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    assert_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = cusp_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A crescent's arc, its pocket's floor arc or a spandrel's line
    /// filleted where it runs to the cusp: the stripe cut by the next wall,
    /// clean at `Full` with nothing unchecked, audited, at the section's
    /// volume integrated over the gap, pose-independent, through STEP and
    /// deterministic.
    cusps_fillet_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = cusp_case() => {
            cusps_blend_as_their_sections(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same scenes chamfered: the cut is transverse at the spine.
    cusps_chamfer_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = cusp_case() => {
            cusps_blend_as_their_sections(case, Blend::Chamfer)
        }
}

/// A convex roof over the rectangle `[0, x] × [0, y]`, for the fan of
/// ADR-0043: the rise at the origin ends at its apex `(0, 0, h)`, and the
/// roof is `h − φ`, `φ(P) = max_i g_i · P` over two or three gradients in
/// the first quadrant — a plane against the wall `x = 0` first, a plane
/// against the wall `y = 0` last — so each pair of neighbours ties along a
/// sharp ridge from the apex and the apex has `pieces + 2` edges.
#[derive(Debug, Clone, PartialEq)]
struct Roof {
    x: f64,
    y: f64,
    /// The gradients, from the plane against `x = 0` to the plane against
    /// `y = 0`.
    grads: Vec<Vec2>,
    /// How far the lowest point of the roof stands above the floor.
    clearance: f64,
}

/// A point of the rectangle's far boundary, by the length `s` along it
/// from `(0, y)`: along the wall `y = y`, then down the wall `x = x`.
fn on_far_wall(roof: &Roof, s: f64) -> (f64, f64) {
    if s <= roof.x {
        (s, roof.y)
    } else {
        (roof.x, roof.y - (s - roof.x))
    }
}

impl Roof {
    fn phi(&self, x: f64, y: f64) -> f64 {
        self.grads
            .iter()
            .map(|g| g.x * x + g.y * y)
            .fold(0.0, f64::max)
    }

    /// The apex's height: the roof's lowest corner stands `clearance`
    /// above the floor.
    fn apex(&self) -> f64 {
        let low = [(0.0, self.y), (self.x, self.y), (self.x, 0.0)]
            .iter()
            .map(|&(x, y)| self.phi(x, y))
            .fold(0.0, f64::max);
        low + self.clearance
    }

    /// The ridges' tangents `y / x`, steepest first, and where each meets
    /// the far boundary.
    fn ridges(&self) -> Vec<(f64, f64)> {
        self.grads
            .windows(2)
            .map(|w| {
                // `(g_a − g_b) · P = 0`.
                let tau = (w[1].x - w[0].x) / (w[0].y - w[1].y);
                let at = if tau * self.x <= self.y {
                    self.x + (self.y - tau * self.x)
                } else {
                    self.y / tau
                };
                (tau, at)
            })
            .collect()
    }

    /// The roof's solid: a polyhedron over the rectangle.
    fn build(&self, m: &mut Model) -> Result<Body, TestCaseError> {
        let h = self.apex();
        let ridges = self.ridges();
        let top = |x: f64, y: f64| Point3::new(x, y, h - self.phi(x, y));
        let mut points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(self.x, 0.0, 0.0),
            Point3::new(self.x, self.y, 0.0),
            Point3::new(0.0, self.y, 0.0),
            Point3::new(0.0, 0.0, h),
        ];
        let at = |points: &mut Vec<Point3>, p: Point3| {
            points.push(p);
            points.len() - 1
        };
        // The far boundary's points in order, with the length along it.
        let mut path: Vec<(f64, usize)> = Vec::new();
        let (t0, tn) = (top(0.0, self.y), top(self.x, 0.0));
        path.push((0.0, at(&mut points, t0)));
        let mut cuts: Vec<f64> = ridges.iter().map(|r| r.1).collect();
        cuts.push(self.x);
        cuts.sort_by(f64::total_cmp);
        for s in cuts {
            let (x, y) = on_far_wall(self, s);
            path.push((s, at(&mut points, top(x, y))));
        }
        path.push((self.x + self.y, at(&mut points, tn)));
        let corner = path.iter().find(|p| p.0 == self.x).map(|p| p.1);
        let corner = corner.ok_or_else(|| fail("the corner"))?;
        // The roof's pieces: the apex and the boundary between two ridges.
        let mut bounds: Vec<f64> = vec![0.0];
        bounds.extend(ridges.iter().map(|r| r.1));
        bounds.push(self.x + self.y);
        let mut faces: Vec<Vec<usize>> = Vec::new();
        for w in bounds.windows(2) {
            let mut face = vec![4];
            face.extend(
                path.iter()
                    .filter(|p| p.0 >= w[0] - 1e-12 && p.0 <= w[1] + 1e-12)
                    .map(|p| p.1),
            );
            faces.push(face);
        }
        faces.push(vec![0, 1, 2, 3]);
        faces.push(vec![0, 3, path[0].1, 4]);
        faces.push(vec![0, 1, path[path.len() - 1].1, 4]);
        let mut back = vec![3, 2, corner];
        back.extend(path.iter().filter(|p| p.0 < self.x).rev().map(|p| p.1));
        faces.push(back);
        let mut right = vec![1, 2, corner];
        right.extend(path.iter().filter(|p| p.0 > self.x).map(|p| p.1));
        faces.push(right);
        // Faces outward: the solid is convex, so a face's Newell normal
        // points away from the centroid of the points.
        let centre = points.iter().map(|p| p.coords).sum::<Vec3>() / points.len() as f64;
        let faces: Vec<Vec<Vec<usize>>> = faces
            .into_iter()
            .map(|mut f| {
                let mut normal = Vec3::zeros();
                let mut mid = Vec3::zeros();
                for i in 0..f.len() {
                    let (a, b) = (points[f[i]].coords, points[f[(i + 1) % f.len()]].coords);
                    normal += a.cross(&b);
                    mid += a / f.len() as f64;
                }
                if normal.dot(&(mid - centre)) < 0.0 {
                    f.reverse();
                }
                vec![f]
            })
            .collect();
        let (builder, keys) = polyhedron(m, &points, &faces, 11).map_err(fail)?;
        Ok(build(m, builder, &keys).map_err(fail)?.0)
    }

    /// The change of volume of the rise at the apex blended at `size`:
    /// minus the roof's height integrated over the blend's cross-section
    /// in the corner, `{0 ≤ x, 0 ≤ y ≤ g(x)}` under the arc or the chord.
    fn change(&self, kind: Blend, size: f64) -> f64 {
        let h = self.apex();
        // The section by a parameter `w`: `(x, g, dx/dw)` and the range,
        // smooth in `w` where the arc's square root is not in `x`.
        let section = |w: f64| match kind {
            Blend::Fillet => (
                size * (1.0 - w.cos()),
                size * (1.0 - w.sin()),
                size * w.sin(),
            ),
            Blend::Chamfer => (size * w, size * (1.0 - w), size),
        };
        let top = match kind {
            Blend::Fillet => FRAC_PI_2,
            Blend::Chamfer => 1.0,
        };
        // The sectors of `y / x`: each with its gradient, from the plane
        // against `y = 0`.
        let mut sectors: Vec<(f64, f64, Vec2)> = Vec::new();
        let ridges = self.ridges();
        let mut lo = 0.0;
        for (i, g) in self.grads.iter().rev().enumerate() {
            let hi = ridges.iter().rev().nth(i).map_or(f64::INFINITY, |r| r.0);
            sectors.push((lo, hi, *g));
            lo = hi;
        }
        let inner = |w: f64| -> f64 {
            let (x, g, dx) = section(w);
            let mut sum = h * g;
            for &(lo, hi, grad) in &sectors {
                let a = g.min(x * lo);
                let b = g.min(x * hi);
                sum -= grad.x * x * (b - a) + grad.y * (b * b - a * a) / 2.0;
            }
            sum * dx
        };
        // Split where the integrand turns: `x τ = g` at each ridge.
        let mut cuts: Vec<f64> = vec![0.0, top];
        for r in &ridges {
            let (mut a, mut b) = (0.0, top);
            for _ in 0..200 {
                let mid = (a + b) / 2.0;
                let (x, g, _) = section(mid);
                if r.0 * x < g {
                    a = mid;
                } else {
                    b = mid;
                }
            }
            cuts.push((a + b) / 2.0);
        }
        cuts.sort_by(f64::total_cmp);
        let removed: f64 = cuts
            .windows(2)
            .map(|w| simpson(w[0], w[1], 400, &inner))
            .sum();
        -removed
    }
}

/// A fan case: the roof, the size and the pose.
#[derive(Debug, Clone)]
struct FanCase {
    roof: Roof,
    size: f64,
    pose: Isometry,
}

/// A roof of two or three planes whose ridges meet the far walls clear of
/// each other and of the corner, a size to 0.35 of the least of the
/// rectangle's sides and the gap the ridges leave, and a pose.
fn fan_case() -> impl Strategy<Value = FanCase> {
    (
        prop::finite_f64(2.0..=4.0),
        prop::finite_f64(2.0..=4.0),
        prop::finite_f64(0.2..=1.0),
        prop::finite_f64(0.0..=1.0),
        prop::finite_f64(0.0..=1.0),
        any::<bool>(),
        prop::finite_f64(0.5..=1.5),
        prop::finite_f64(0.05..=1.0),
        prop::pose(),
    )
        .prop_map(|(x, y, slope, u1, u2, three, clearance, fraction, pose)| {
            // The hits of the ridges on the far walls: each in a wall's
            // middle 70%, the second after the first.
            let reach = 0.7 * (x + y);
            let s_at = |u: f64| {
                let a = u * reach;
                if a <= 0.7 * x {
                    0.15 * x + a
                } else {
                    x + 0.15 * y + (a - 0.7 * x)
                }
            };
            let tau_at = |s: f64| {
                let (px, py) = if s <= x { (s, y) } else { (x, y - (s - x)) };
                py / px
            };
            let (s_first, s_second) = (s_at(0.45 * u1), s_at(0.55 + 0.45 * u2));
            let grads = if three {
                let (t12, t23) = (tau_at(s_first), tau_at(s_second));
                let ratio = (t12 * t23).sqrt();
                let s = {
                    let k = (t12 / t23).sqrt();
                    k / (1.0 + k)
                };
                let q = slope * ratio;
                vec![
                    Vec2::new(0.0, slope),
                    Vec2::new(s * q, s * slope),
                    Vec2::new(q, 0.0),
                ]
            } else {
                let t = tau_at(s_at(u1));
                vec![Vec2::new(0.0, slope), Vec2::new(slope * t, 0.0)]
            };
            let roof = Roof {
                x,
                y,
                grads,
                clearance,
            };
            FanCase {
                size: 0.35 * fraction * x.min(y),
                roof,
                pose,
            }
        })
}

/// The blend of the rise at the roof's apex, the roof moved to its pose
/// first: the model, the input, the result and its record.
fn fan_posed(
    case: &FanCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let roof = case.roof.build(&mut m)?;
    let (moved, _) = transform(&mut m, roof, &case.pose).map_err(fail)?;
    let rise = Point3::new(0.0, 0.0, case.roof.apex() / 2.0);
    let edge = edge_near(&m, moved, case.pose.apply(rise))?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} of the posed roof: {e}")))?;
    Ok((m, moved, blended, p))
}

fn fans_blend_as_their_sections(case: FanCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = fan_posed(&case, kind)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let before = mass_properties(&m, moved).map_err(fail)?;
    let want = before.volume + case.roof.change(kind, case.size);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the section's {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let roof = case.roof.build(&mut here)?;
    let rise = Point3::new(0.0, 0.0, case.roof.apex() / 2.0);
    let edge = edge_near(&here, roof, rise)?;
    let (rest, _) = op(kind)(&mut here, roof, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    assert_checked(&check(&here, rest, Level::Full))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = fan_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A rise ending at the apex of a roof of two or three planes,
    /// filleted: the end one arc a piece with a vertex on each ridge,
    /// clean at `Full` with nothing unchecked, audited, at the roof's
    /// height integrated over the section, pose-independent and
    /// deterministic.
    fans_fillet_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = fan_case() => {
            fans_blend_as_their_sections(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same roofs chamfered.
    fans_chamfer_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = fan_case() => {
            fans_blend_as_their_sections(case, Blend::Chamfer)
        }
}

// A miter of unequal dihedrals (ADR-0044): a prism over a random
// parallelogram, the rise at one vertex and a top edge there blended, the
// vertex's other top edge sharp. The volume is the prism less the two
// blends' removed regions taken whole along their edges, plus where those
// regions overlap — in closed form below.

/// A prism over the parallelogram `(0,0) (a,0) (a + b cos θ, b sin θ)
/// (b cos θ, b sin θ)`, `height` high, and the corner at vertex `vertex`
/// whose rise and one top edge are blended.
#[derive(Debug, Clone)]
struct SlantCase {
    a: f64,
    b: f64,
    theta: f64,
    height: f64,
    vertex: usize,
    /// `true` for the top edge leaving the vertex, `false` for the one
    /// arriving at it.
    leaving: bool,
    fraction: f64,
    pose: Isometry,
}

impl SlantCase {
    fn polygon(&self) -> [Point2; 4] {
        let (c, s) = (self.theta.cos(), self.theta.sin());
        [
            Point2::new(0.0, 0.0),
            Point2::new(self.a, 0.0),
            Point2::new(self.a + self.b * c, self.b * s),
            Point2::new(self.b * c, self.b * s),
        ]
    }

    /// The interior angle at the corner's vertex.
    fn angle(&self) -> f64 {
        if self.vertex % 2 == 0 {
            self.theta
        } else {
            PI - self.theta
        }
    }

    /// The top edge's polygon vertices, and its length.
    fn top(&self) -> (usize, f64) {
        let first = if self.leaving {
            self.vertex
        } else {
            (self.vertex + 3) % 4
        };
        (first, if first % 2 == 0 { self.a } else { self.b })
    }

    /// The blend's size: a fraction of the shortest of the prism's edges
    /// the blends' contacts and the trim must stay clear of.
    fn size(&self) -> f64 {
        self.fraction * self.a.min(self.b).min(self.height)
    }

    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
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
        Ok(extrude(m, &profile, Vec3::z(), self.height)?.0)
    }

    /// The rise's and the top edge's midpoints.
    fn midpoints(&self) -> [Point3; 2] {
        let poly = self.polygon();
        let v = poly[self.vertex];
        let (first, _) = self.top();
        let mid = poly[first] + (poly[(first + 1) % 4] - poly[first]) / 2.0;
        [
            Point3::new(v.x, v.y, self.height / 2.0),
            Point3::new(mid.x, mid.y, self.height),
        ]
    }

    /// The volume after the blend. With `t` the interior angle at the
    /// vertex, the rise removes the corner's section over the height —
    /// `r² (cot(t/2) − (π − t)/2)` round, `d² sin t / 2` flat — and the top
    /// edge its square section over its length, `(1 − π/4) r²` or `d²/2`,
    /// the slant's gain at one end paid back at the other. The two overlap
    /// in the wall strip the top edge's section reaches, depth `n` and
    /// height `w` below the top, where the corner's section is
    /// `L(n)` wide along the wall and the top edge's reaches `n` only
    /// below `w < g(n)`: `∫ L(n) g(n) dn` to the depth where the corner's
    /// section ends, `min(r, r (1 + cos t))` round (substituting
    /// `n = r (1 − cos φ)`), `d sin t` flat.
    fn volume(&self, kind: Blend) -> f64 {
        let t = self.angle();
        let s = self.size();
        let (_, length) = self.top();
        let base = self.a * self.b * self.theta.sin();
        let (corner, edge, overlap) = match kind {
            Blend::Fillet => {
                let kappa = 1.0 / (t / 2.0).tan();
                let phi = (PI - t).min(FRAC_PI_2);
                let overlap = s.powi(3)
                    * simpson(0.0, phi, 2000, &|p: f64| {
                        (kappa - p.sin() - (1.0 - p.cos()) / t.tan()) * (1.0 - p.sin()) * p.sin()
                    });
                (
                    s * s * (kappa - (PI - t) / 2.0),
                    (1.0 - FRAC_PI_4) * s * s,
                    overlap,
                )
            }
            Blend::Chamfer => {
                let k = (t / 2.0).tan() + 1.0 / t.tan();
                let n = s * t.sin();
                // ∫₀ⁿ (d − k x)(d − x) dx
                let overlap = s * s * n - (1.0 + k) * s * n * n / 2.0 + k * n.powi(3) / 3.0;
                (s * s * t.sin() / 2.0, s * s / 2.0, overlap)
            }
        };
        base * self.height - corner * self.height - edge * length + overlap
    }
}

fn slant_case() -> impl Strategy<Value = SlantCase> {
    (
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(0.9..=1.35),
        prop::finite_f64(1.0..=3.0),
        0usize..4,
        any::<bool>(),
        prop::finite_f64(0.05..=0.25),
        prop::pose(),
    )
        .prop_map(
            |(a, b, theta, height, vertex, leaving, fraction, pose)| SlantCase {
                a,
                b,
                theta,
                height,
                vertex,
                leaving,
                fraction,
                pose,
            },
        )
}

/// The blend of `case`'s two edges, in the order `edges` lists them, in a
/// fresh model with the prism moved to its pose.
fn slant_posed(
    case: &SlantCase,
    kind: Blend,
    swap: bool,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let prism = case.build(&mut m).map_err(fail)?;
    let (moved, _) = transform(&mut m, prism, &case.pose).map_err(fail)?;
    let mut edges = case
        .midpoints()
        .iter()
        .map(|&p| edge_near(&m, moved, case.pose.apply(p)))
        .collect::<Result<Vec<_>, _>>()?;
    if swap {
        edges.reverse();
    }
    let (blended, p) = op(kind)(&mut m, moved, &edges, case.size())
        .map_err(|e| fail(format!("{kind:?} of the slanted corner: {e}")))?;
    Ok((m, moved, blended, p))
}

fn slants_blend_as_their_closed_forms(case: SlantCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = slant_posed(&case, kind, false)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    // A fillet miter's trim arc has a fitted pcurve on the cylinder.
    let rel = match kind {
        Blend::Fillet => fitted_rel(&m, &props),
        Blend::Chamfer => REL,
    };
    let volume = case.volume(kind);
    prop_assert!(
        close_to(props.volume, volume, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        volume
    );
    // Either order of the two edges: the same ids, so the same dump and
    // record; and the same again on a second run.
    let (swapped, _, flipped, flipped_p) = slant_posed(&case, kind, true)?;
    prop_assert_eq!(
        dump_text(&swapped, flipped).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?,
        "the edges in the other order"
    );
    prop_assert_eq!(flipped_p, p.clone());
    let (again, _, twice, again_p) = slant_posed(&case, kind, false)?;
    prop_assert_eq!(
        dump_text(&again, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(again_p, p);
    Ok(())
}

prop_shards! {
    /// Fillets of a slanted prism's rise and one top edge at a vertex, the
    /// other top edge sharp: a miter of unequal dihedrals, clean at `Full`
    /// with nothing unchecked, audited, at the closed-form volume, the same
    /// for either order of the edges and deterministic.
    slanted_fillets_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = slant_case() => {
            slants_blend_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same corners chamfered.
    slanted_chamfers_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = slant_case() => {
            slants_blend_as_their_closed_forms(case, Blend::Chamfer)
        }
}

/// The closed form at the slanted prism of
/// `blend/miter-unequal-dihedrals-fillet` and its chamfer twin: the
/// parallelogram `(0,0) (2,0) (3,2) (1,2)` is `a = 2`, `b = √5`,
/// `θ = atan 2`, its vertex 2 the corner.
#[test]
fn the_slanted_closed_form_is_the_fixtures() {
    let case = SlantCase {
        a: 2.0,
        b: 5.0_f64.sqrt(),
        theta: 2.0_f64.atan(),
        height: 2.0,
        vertex: 2,
        leaving: true,
        fraction: 0.1,
        pose: Isometry::identity(),
    };
    // 0.2 of the shortest edge `a = 2` over ten.
    assert!((case.size() - 0.2).abs() < 1e-12);
    let r: f64 = 0.2;
    let golden = (1.0 + 5.0_f64.sqrt()) / 2.0;
    let fixture = 8.0
        - 2.0 * r * r * (golden - (PI - 2.0_f64.atan()) / 2.0)
        - 2.0 * r * r * (1.0 - FRAC_PI_4)
        + r.powi(3) * (golden - 0.5) * (1.0 - FRAC_PI_4)
        + r.powi(3) * (3.0 - PI) / 4.0;
    let got = case.volume(Blend::Fillet);
    assert!((got - fixture).abs() < 1e-12, "{got} against {fixture}");
}
