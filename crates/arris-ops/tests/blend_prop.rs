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

use arris_debug::testing::{REL, close_to, fail, fitted_rel};
use arris_debug::unmetered::{chamfer, extrude, fillet, mass_properties, primitive_box, transform};
use arris_debug::{dump_text, prop, prop_shards};
use arris_ops::OpError;
use arris_ops::arris_check::arris_topo::arris_geom::{Profile, ProfileLoop, ProfileSegment};
use arris_ops::arris_check::arris_topo::arris_math::{Frame, Isometry, Point2, Point3, Vec3};
use arris_ops::arris_check::arris_topo::provenance::audit;
use arris_ops::arris_check::arris_topo::{Body, Edge, Model, Provenance};
use arris_ops::arris_check::{Level, Report, check};
use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};
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
    use arris_ops::arris_check::arris_topo::arris_math::nalgebra::{Quaternion, Unit};
    let case = Case {
        prism: Prism::Ell {
            x: 1.0,
            y: 1.0,
            arm_x: 0.3762546790021239,
            arm_y: 0.5247994758022808,
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
        size: 0.012445353519596015,
        pose: Isometry::new(
            Unit::new_unchecked(Quaternion::new(
                0.47173495514794034,
                0.0,
                0.7989230761002037,
                0.373079147857608,
            )),
            Vec3::new(66.25009385227327, 66.38690084404882, 0.0),
        ),
    };
    if let Err(e) = blends_as_their_closed_forms(case, Blend::Fillet) {
        panic!("{e}");
    }
}
