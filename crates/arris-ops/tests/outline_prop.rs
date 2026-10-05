//! `ops::fillet` and `ops::chamfer` along an outline at random
//! (ADR-0035): a stadium, a rounded rectangle and a plate with a D-shaped
//! notch, extruded and moved to a random pose, one top edge named — the
//! walk reaches the rest of the chain — at a size below the arcs'. Each
//! result is clean at `Full` with nothing unchecked, its record audits, its
//! volume is the prism's less the corner's section swept along the
//! outline by Pappus (`A` times the length on a line, times the angle at
//! the centroid's radius on an arc), blending then moving equals moving
//! then blending by volume and counts, and two runs dump identically. A
//! failure prints the case and the seed, and becomes a fixture under
//! `tests/fixtures/regression/` (`tests/fixtures/README.md` §Property-test
//! failures).

use arris_check::{Level, Report, Unchecked, check};
use arris_debug::testing::{REL, close_to, fail};
use arris_debug::unmetered::{chamfer, extrude, fillet, mass_properties, transform};
use arris_debug::{dump_text, prop, prop_shards};
use arris_geom::{Profile, ProfileLoop, ProfileSegment, SurfaceKind};
use arris_math::{Frame, Isometry, Point2, Point3, Vec3};
use arris_ops::OpError;
use arris_topo::provenance::audit;
use arris_topo::{Body, Edge, Model, Provenance};
use core::f64::consts::{FRAC_PI_4, PI};
use proptest::prelude::*;

#[derive(Debug, Clone, Copy, PartialEq)]
enum Blend {
    Fillet,
    Chamfer,
}

/// An outline in the world's XY plane, extruded from `z = 0` to `height`.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Outline {
    /// Two lines of length `length` joined by two half circles of radius
    /// `radius`, the left one's centre at the origin: a closed chain of
    /// four, every vertex tangent.
    Stadium { length: f64, radius: f64 },
    /// A `w × h` rectangle with its corners rounded at `corner`: a closed
    /// chain of eight.
    Rounded { w: f64, h: f64, corner: f64 },
    /// A `w × h` plate with a half disc of radius `radius` cut from the
    /// middle of its `y = 0` side: an open arc, its ends on the side face.
    Notch { w: f64, h: f64, radius: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct Case {
    outline: Outline,
    height: f64,
    /// The size as a fraction of the smaller of the arcs' radius and the
    /// height, below the bound the faces allow.
    fraction: f64,
    /// Which top edge is named, modulo the outline's.
    pick: usize,
    pose: Isometry,
}

fn line(u: f64, v: f64) -> ProfileSegment {
    ProfileSegment::LineTo(Point2::new(u, v))
}

fn arc(u: f64, v: f64, via: (f64, f64)) -> ProfileSegment {
    ProfileSegment::ArcTo {
        to: Point2::new(u, v),
        via: Point2::new(via.0, via.1),
    }
}

/// The ridge of the section a fillet removes, `(1 − π/4) r²` with its
/// centroid `k r` from the arc's radius, and a chamfer's `d² / 2` with
/// `d / 3`.
fn section(kind: Blend, size: f64) -> (f64, f64) {
    match kind {
        Blend::Fillet => (
            (1.0 - FRAC_PI_4) * size * size,
            size * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI),
        ),
        Blend::Chamfer => (size * size / 2.0, size / 3.0),
    }
}

impl Outline {
    fn start_and_segments(&self) -> (Point2, Vec<ProfileSegment>) {
        let s = FRAC_PI_4.sin();
        match *self {
            Outline::Stadium {
                length: l,
                radius: r,
            } => (
                Point2::new(0.0, -r),
                vec![
                    line(l, -r),
                    arc(l, r, (l + r, 0.0)),
                    line(0.0, r),
                    arc(0.0, -r, (-r, 0.0)),
                ],
            ),
            Outline::Rounded { w, h, corner: c } => {
                let d = c - c * s;
                (
                    Point2::new(c, 0.0),
                    vec![
                        line(w - c, 0.0),
                        arc(w, c, (w - d, d)),
                        line(w, h - c),
                        arc(w - c, h, (w - d, h - d)),
                        line(c, h),
                        arc(0.0, h - c, (d, h - d)),
                        line(0.0, c),
                        arc(c, 0.0, (d, d)),
                    ],
                )
            }
            Outline::Notch { w, h, radius: r } => (
                Point2::new(0.0, 0.0),
                vec![
                    line(w / 2.0 - r, 0.0),
                    arc(w / 2.0 + r, 0.0, (w / 2.0, r)),
                    line(w, 0.0),
                    line(w, h),
                    line(0.0, h),
                    line(0.0, 0.0),
                ],
            ),
        }
    }

    /// The radius of the smallest arc, the bound a blend's size keeps under.
    fn arc_radius(&self) -> f64 {
        match *self {
            Outline::Stadium { radius, .. } => radius,
            Outline::Rounded { corner, .. } => corner,
            Outline::Notch { radius, .. } => radius,
        }
    }

    /// The midpoints of the top edges a blend can be named by.
    fn top_midpoints(&self, height: f64) -> Vec<Point3> {
        let p = |u: f64, v: f64| Point3::new(u, v, height);
        let s = FRAC_PI_4.sin();
        match *self {
            Outline::Stadium {
                length: l,
                radius: r,
            } => vec![p(l / 2.0, -r), p(l + r, 0.0), p(l / 2.0, r), p(-r, 0.0)],
            Outline::Rounded { w, h, corner: c } => {
                let d = c - c * s;
                vec![
                    p(w / 2.0, 0.0),
                    p(w - d, d),
                    p(w, h / 2.0),
                    p(w - d, h - d),
                    p(w / 2.0, h),
                    p(d, h - d),
                    p(0.0, h / 2.0),
                    p(d, d),
                ]
            }
            Outline::Notch { w, radius, .. } => vec![p(w / 2.0, radius)],
        }
    }

    fn area(&self) -> f64 {
        match *self {
            Outline::Stadium { length, radius } => 2.0 * radius * length + PI * radius * radius,
            Outline::Rounded { w, h, corner } => w * h - (4.0 - PI) * corner * corner,
            Outline::Notch { w, h, radius } => w * h - PI * radius * radius / 2.0,
        }
    }

    /// The volume of the prism with the outline's whole top blended.
    fn volume(&self, height: f64, kind: Blend, size: f64) -> f64 {
        let (a, k) = section(kind, size);
        let swept = match *self {
            Outline::Stadium { length, radius } => 2.0 * length * a + 2.0 * PI * (radius - k) * a,
            Outline::Rounded { w, h, corner } => {
                (2.0 * (w - 2.0 * corner) + 2.0 * (h - 2.0 * corner)) * a
                    + 2.0 * PI * (corner - k) * a
            }
            Outline::Notch { radius, .. } => PI * (radius + k) * a,
        };
        self.area() * height - swept
    }
}

impl Case {
    fn size(&self) -> f64 {
        self.fraction * self.outline.arc_radius().min(self.height)
    }

    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        let (start, segments) = self.outline.start_and_segments();
        let profile = Profile {
            plane: Frame::world(),
            outer: ProfileLoop::Path { start, segments },
            holes: Vec::new(),
        };
        Ok(extrude(m, &profile, Vec3::z(), self.height)?.0)
    }

    fn midpoint(&self) -> Point3 {
        let at = self.outline.top_midpoints(self.height);
        at[self.pick % at.len()]
    }
}

fn outline() -> impl Strategy<Value = Outline> {
    let side = || prop::finite_f64(1.0..=4.0);
    prop_oneof![
        (side(), side()).prop_map(|(length, radius)| Outline::Stadium { length, radius }),
        (side(), side(), prop::finite_f64(0.1..=0.4)).prop_map(|(w, h, f)| Outline::Rounded {
            w: w + 1.0,
            h: h + 1.0,
            corner: f * (w + 1.0).min(h + 1.0),
        }),
        (side(), side(), prop::finite_f64(0.2..=0.6)).prop_map(|(w, h, f)| Outline::Notch {
            w: w + 2.0,
            h: h + 2.0,
            radius: f * (w + 2.0) / 2.0,
        }),
    ]
}

fn case() -> impl Strategy<Value = Case> {
    (
        outline(),
        prop::finite_f64(0.5..=3.0),
        prop::finite_f64(0.05..=0.4),
        0usize..8,
        prop::pose(),
    )
        .prop_map(|(outline, height, fraction, pick, pose)| Case {
            outline,
            height,
            fraction,
            pick,
            pose,
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

/// `report` leaves nothing unchecked but pairs of two tori — the half tori
/// of one torus a chain over a round end makes, which S5 leaves undecided
/// in some poses (`regression/turned-stadium-fillet-torus-pair`, a named
/// exclusion: it goes when that fixture passes).
fn only_torus_pairs(report: &Report) -> bool {
    report.unchecked().iter().all(|u| {
        matches!(
            u,
            Unchecked::FacePair {
                kinds: (SurfaceKind::Torus, SurfaceKind::Torus),
                ..
            }
        )
    })
}

/// The blend of `case` in a fresh model, the prism moved to its pose
/// first (or not): the model, the input, the result and its record.
fn built(
    case: &Case,
    kind: Blend,
    posed: bool,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let prism = case.build(&mut m).map_err(fail)?;
    let (input, at) = if posed {
        (
            transform(&mut m, prism, &case.pose).map_err(fail)?.0,
            case.pose.apply(case.midpoint()),
        )
    } else {
        (prism, case.midpoint())
    };
    let edge = edge_near(&m, input, at)?;
    let (result, p) = op(kind)(&mut m, input, &[edge], case.size())
        .map_err(|e| fail(format!("{kind:?} of {case:?}: {e}")))?;
    Ok((m, input, result, p))
}

fn blends_as_their_closed_forms(case: Case, kind: Blend) -> Result<(), TestCaseError> {
    // Moved, then blended.
    let (m, input, result, p) = built(&case, kind, true)?;
    let report = check(&m, result, Level::Full);
    prop_assert!(report.is_ok(), "{}", report);
    prop_assert!(only_torus_pairs(&report), "nothing unchecked\n{}", report);
    audit(&m, &[input], result, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, result).map_err(fail)?;
    let volume = case.outline.volume(case.height, kind, case.size());
    prop_assert!(
        close_to(props.volume, volume, 1.0, REL),
        "volume {} vs the closed form {}",
        props.volume,
        volume
    );

    // Blended, then moved.
    let (mut here, _, rest, rest_p) = built(&case, kind, false)?;
    let rest_report = check(&here, rest, Level::Full);
    prop_assert!(rest_report.is_ok(), "{}", rest_report);
    prop_assert!(
        only_torus_pairs(&rest_report),
        "nothing unchecked\n{}",
        rest_report
    );
    let _ = rest_p;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, REL),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, result).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Deterministic.
    let (again, _, twice, again_p) = built(&case, kind, true)?;
    prop_assert_eq!(
        dump_text(&again, twice).map_err(fail)?,
        dump_text(&m, result).map_err(fail)?
    );
    prop_assert_eq!(again_p, p);
    Ok(())
}

prop_shards! {
    /// Fillets along a stadium's, a rounded rectangle's or a D-notch's top
    /// outline, one edge named: clean at `Full` with nothing unchecked,
    /// audited, at the Pappus volume, pose-independent and deterministic.
    outline_fillets_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = case() => {
            blends_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// Chamfers along the same outlines.
    outline_chamfers_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (case) = case() => {
            blends_as_their_closed_forms(case, Blend::Chamfer)
        }
}
