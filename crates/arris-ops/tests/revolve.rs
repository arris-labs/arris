//! `ops::revolve` (`docs/ARCHITECTURE.md` §Operations): a thousand rectilinear
//! staircases beside an axis or reaching it in random poses — the checker
//! at `Full` with nothing violated and nothing unchecked, volume and
//! area to Pappus's theorems, the mesh closed and within its chord of
//! the exact volume, one `Generated` per entity and every part of the
//! sketch present, the dump identical on two runs; a thousand general
//! profiles whose segments sweep cones, spheres and tori — the same,
//! nothing unchecked either, a ray through an apex or a pole abandoned on
//! its degenerate edge; the closed forms of a frustum, a spherical zone
//! and a ring, which the corpus reads back through the oracle; the tube's
//! numbers equal to `boolean/coaxial-cut`'s; a profile given clockwise
//! the same body as counter-clockwise; the angle's bounds; and every
//! typed refusal with the model untouched.

use arris_debug::unmetered::{mass_properties, revolve};
use std::cell::RefCell;
use std::collections::BTreeSet;

use arris_debug::fixtures::{Analytic, Loop, Num, Plane, Recipe, Segment, Step};
use arris_debug::prop::profile::Sweep;
use arris_debug::prop::sweep;
use arris_debug::testing::{close, fail};
use arris_debug::unmetered::tessellate;
use arris_debug::{corpus, dump_text, euler_line, fixtures, oracle, prop, prop_shards, sample};
use arris_io::step;
use arris_ops::arris_check::arris_topo::arris_geom::region2::Side;
use arris_ops::arris_check::arris_topo::arris_geom::{
    Curve2, Profile, ProfileEdge, ProfileError, ProfileLoop, ProfileSegment, Surface, SurfaceKind,
};

use arris_ops::arris_check::arris_topo::arris_math::{
    Axis, Frame, Point2, Point3, Tolerance, Vec2, Vec3,
};

use arris_ops::arris_check::arris_topo::provenance::SweepPart;
use arris_ops::arris_check::arris_topo::{Body, EntityId, Model, Provenance, Role, Shape};
use arris_ops::arris_check::classify::{Classification, classify_point};
use arris_ops::arris_check::domain::FaceDomain;
use arris_ops::arris_check::{Level, check, lumps};
use arris_ops::{OpError, Reason};
use core::f64::consts::{PI, TAU};
use proptest::prelude::*;

/// The relative tolerance the Pappus identities hold to.
const REL: f64 = 1e-9;

/// The chord tolerance the property meshes at. The mesh's surface lies
/// within this of the true one, so its volume is within this times the
/// area of the exact volume.
const MESH_CHORD: f64 = 1e-2;

/// [`arris_debug::testing::recorded_parts`] over `Role::Revolve`.
fn recorded_parts(
    m: &Model,
    body: Body,
    p: &Provenance,
) -> Result<BTreeSet<SweepPart>, TestCaseError> {
    arris_debug::testing::recorded_parts(m, body, p, |role| match role {
        Role::Revolve(part) => Some(part),
        _ => None,
    })
}

/// A chain of a loop's segments off the axis, as the sketch alone gives
/// it: the loop, the lowest segment, and where along the axis its first
/// and last vertices lie (`None` for a loop never along the axis).
type ChainOf = (usize, usize, Option<(f64, f64)>);

/// The parts a revolve of `sweep` makes, from the sketch alone: the
/// vertex indices by the segment that starts there; no end parts in a
/// full turn, and no `StartEdge` there for a segment perpendicular to the
/// axis; for a segment along the axis nothing but, in a partial turn, its
/// `StartEdge`; for a vertex on the axis no `EndVertex`, a `Rise` only
/// where a cone or a sphere closes there on a degenerate edge, and in a
/// full turn a `StartVertex` only then too; and in a full turn a `Cavity`
/// for every chain of segments off the axis but the one whose ends span
/// the others, named by its loop and lowest segment.
fn expected_parts(sweep: &Sweep, tol: Tolerance) -> BTreeSet<SweepPart> {
    let full = (sweep.angle - TAU).abs() <= tol.angular;
    let plane = &sweep.profile.plane;
    let a = plane.vec_to_local(sweep.axis.direction.into_inner());
    let along = Vec2::new(a.x, a.y).normalize();
    let o = plane.to_local(sweep.axis.origin);
    let origin = Point2::new(o.x, o.y);
    let rho = |p: Point2| (p - origin).dot(&Vec2::new(-along.y, along.x));
    let t = |p: Point2| (p - origin).dot(&along);
    let on_axis = |p: Point2| rho(p).abs() <= tol.linear;
    let perpendicular =
        |e: &ProfileEdge| (e.end - e.start).normalize().dot(&along).abs() <= tol.angular;
    let is_along = |e: &ProfileEdge| {
        matches!(e.pcurve, Curve2::Line { .. }) && on_axis(e.start) && on_axis(e.end)
    };
    // A segment with an end on the axis closes there on a degenerate edge
    // unless it is a plane: an arc there sweeps a sphere, an oblique line a
    // cone.
    let closes = |e: &ProfileEdge| {
        !is_along(e) && (matches!(e.pcurve, Curve2::Circle { .. }) || !perpendicular(e))
    };

    let mut parts = BTreeSet::from([SweepPart::Body, SweepPart::Shell]);
    if !full {
        parts.insert(SweepPart::StartCap);
        parts.insert(SweepPart::EndCap);
    }
    let mut chains: Vec<ChainOf> = Vec::new();
    for edges in sweep.profile.edges(tol).unwrap() {
        let n = edges.len();
        for (j, e) in edges.iter().enumerate() {
            let (loop_index, segment) = (e.loop_index, e.segment);
            let vertex = if e.reversed {
                (segment + 1) % n
            } else {
                segment
            };
            if !is_along(e) {
                parts.insert(SweepPart::Side {
                    loop_index,
                    segment,
                });
            }
            if !(full && (perpendicular(e) || is_along(e))) {
                parts.insert(SweepPart::StartEdge {
                    loop_index,
                    segment,
                });
            }
            if !(full || is_along(e)) {
                parts.insert(SweepPart::EndEdge {
                    loop_index,
                    segment,
                });
            }
            let before = &edges[(j + n - 1) % n];
            let (rise, start) = if on_axis(e.start) {
                let singular = closes(e) || closes(before);
                (singular, singular || !full)
            } else {
                if !full {
                    parts.insert(SweepPart::EndVertex { loop_index, vertex });
                }
                (true, true)
            };
            if rise {
                parts.insert(SweepPart::Rise { loop_index, vertex });
            }
            if start {
                parts.insert(SweepPart::StartVertex { loop_index, vertex });
            }
        }
        // The loop's chains, walked from just past its first segment along
        // the axis.
        let first = (0..n).find(|&j| is_along(&edges[j]));
        let from = first.map_or(0, |j| j + 1);
        let mut current: Option<usize> = None;
        for e in edges.iter().cycle().skip(from).take(n) {
            if is_along(e) {
                current = None;
                continue;
            }
            let c = *current.get_or_insert_with(|| {
                chains.push((
                    e.loop_index,
                    e.segment,
                    first.map(|_| (t(e.start), t(e.start))),
                ));
                chains.len() - 1
            });
            let chain = &mut chains[c];
            chain.1 = chain.1.min(e.segment);
            if let Some(span) = &mut chain.2 {
                span.1 = t(e.end);
            }
        }
    }
    if full {
        let width = |c: &ChainOf| c.2.map_or(f64::INFINITY, |(a, b)| (b - a).abs());
        let outer = chains
            .iter()
            .enumerate()
            .filter(|(_, c)| c.0 == 0)
            .max_by(|a, b| width(a.1).total_cmp(&width(b.1)))
            .map(|(i, _)| i);
        for (i, &(loop_index, segment, _)) in chains.iter().enumerate() {
            if Some(i) != outer {
                parts.insert(SweepPart::Cavity {
                    loop_index,
                    segment,
                });
            }
        }
    }
    parts
}

/// The property every random sweep is held to: it revolves — a full turn
/// into one lump with a void per hole and per notch cut in from the
/// axis — is clean at `Fast`, has no violation at `Full` and no unchecked
/// row, has Pappus's volume and area, a closed mesh within its chord of
/// the exact volume, one `Generated` per entity and every part of the
/// sketch, and the same dump on a second run.
fn revolves_to_pappus(sweep: &Sweep) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let tol = m.precision().tolerance();
    let (body, p) = revolve(&mut m, &sweep.profile, sweep.axis, sweep.angle)
        .map_err(|e| fail(format!("revolve: {e}")))?;
    let fast = check(&m, body, Level::Fast);
    prop_assert!(fast.is_ok(), "not clean at Fast\n{}", fast);
    let report = check(&m, body, Level::Full);
    prop_assert!(
        report.is_ok() && report.unchecked().is_empty(),
        "not clean at Full\n{}\n{}",
        report,
        dump_text(&m, body).map_err(fail)?
    );
    let props = mass_properties(&m, body).map_err(fail)?;
    let pappus = sweep::revolved(&sweep.profile, &sweep.axis, sweep.angle, tol).map_err(fail)?;
    prop_assert!(
        close(props.volume, pappus.volume),
        "volume {} vs Pappus {}",
        props.volume,
        pappus.volume
    );
    prop_assert!(
        close(props.area, pappus.area),
        "area {} vs Pappus {}",
        props.area,
        pappus.area
    );
    let mesh = tessellate(&m, body, MESH_CHORD).map_err(fail)?;
    let Some(volume) = mesh.signed_volume() else {
        return Err(fail("the mesh is not closed"));
    };
    prop_assert!(
        (volume - props.volume).abs() <= MESH_CHORD * props.area,
        "mesh volume {} vs {} at chord {}",
        volume,
        props.volume,
        MESH_CHORD
    );
    let parts = recorded_parts(&m, body, &p)?;
    prop_assert_eq!(parts, expected_parts(sweep, tol));
    let dump = dump_text(&m, body).map_err(fail)?;
    let mut again = Model::default();
    let (twice, _) = revolve(&mut again, &sweep.profile, sweep.axis, sweep.angle).map_err(fail)?;
    prop_assert_eq!(dump_text(&again, twice).map_err(fail)?, dump);
    Ok(())
}

prop_shards! {
    /// A staircase of segments parallel and perpendicular to the axis,
    /// every face a plane or a cylinder the checker decides every pair
    /// of: clean at `Full` with nothing unchecked, Pappus's volume and
    /// area, a closed mesh, complete provenance, a deterministic dump.
    rectilinear_profiles_revolve_to_pappus [shard_0 shard_1 shard_2 shard_3]
        (sweep) = prop::profile::rectilinear() => {
            revolves_to_pappus(&sweep)
        }
}

prop_shards! {
    /// A profile whose segments sweep cones in both orientations, spheres
    /// and tori beside planes and cylinders: clean at `Fast`, clean at
    /// `Full` with nothing unchecked — S5 and B1 decide every pair on the
    /// three by the coaxial and meridian arms and the line arms
    /// (ADR-0008) — Pappus's volume and area, a closed mesh, complete
    /// provenance, a deterministic dump.
    general_profiles_revolve_to_pappus [shard_0 shard_1 shard_2 shard_3]
        (sweep) = prop::profile::general() => {
            revolves_to_pappus(&sweep)
        }
}

/// The plane `y = 0` with `u` along `x` and `v` along `z`: the plane the
/// `sweep/revolve-*` fixtures draw in.
fn xz_plane() -> Frame {
    Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap()
}

fn rectangle(u0: f64, u1: f64, v0: f64, v1: f64, clockwise: bool) -> ProfileLoop {
    let p = |u, v| Point2::new(u, v);
    let corners = if clockwise {
        [p(u0, v1), p(u1, v1), p(u1, v0), p(u0, v0)]
    } else {
        [p(u1, v0), p(u1, v1), p(u0, v1), p(u0, v0)]
    };
    ProfileLoop::Path {
        start: p(u0, v0),
        segments: corners.into_iter().map(ProfileSegment::LineTo).collect(),
    }
}

fn tube_profile(clockwise: bool) -> Profile {
    Profile {
        plane: xz_plane(),
        outer: rectangle(1.0, 2.0, -1.0, 1.0, clockwise),
        holes: Vec::new(),
    }
}

fn counts(m: &Model, body: Body) -> (usize, usize, usize) {
    let c = m.closure(body).unwrap();
    (c.vertices.len(), c.edges.len(), c.faces.len())
}

/// The full-turn tube has the numbers of the coaxial cut that builds the
/// same solid the other way: the two recipes of the corpus, measured.
#[test]
fn the_tube_has_the_coaxial_cuts_numbers() {
    let tube = corpus::chain(
        &fixtures::corpus_root().join("sweep/revolve-tube"),
        "default",
    )
    .unwrap();
    let cut = corpus::chain(
        &fixtures::corpus_root().join("boolean/coaxial-cut"),
        "default",
    )
    .unwrap();
    let (a, b) = (tube.result().unwrap(), cut.result().unwrap());
    let (pa, pb) = (
        mass_properties(&tube.model, a).unwrap(),
        mass_properties(&cut.model, b).unwrap(),
    );
    assert!(
        close(pa.volume, pb.volume),
        "{} vs {}",
        pa.volume,
        pb.volume
    );
    assert!(close(pa.area, pb.area), "{} vs {}", pa.area, pb.area);
    assert!((pa.centroid - pb.centroid).norm() <= REL);
    assert_eq!(counts(&tube.model, a), counts(&cut.model, b));
    assert_eq!(counts(&tube.model, a), (4, 6, 4));
}

/// A profile carries no orientation: the tube written clockwise is the
/// same body, id for id, as written counter-clockwise.
#[test]
fn a_clockwise_profile_gives_the_counter_clockwise_dump() {
    let axis = Axis::z_at(Point3::origin());
    let mut m = Model::default();
    let (ccw, _) = revolve(&mut m, &tube_profile(false), axis, TAU).unwrap();
    let mut n = Model::default();
    let (cw, _) = revolve(&mut n, &tube_profile(true), axis, TAU).unwrap();
    assert_eq!(dump_text(&m, ccw).unwrap(), dump_text(&n, cw).unwrap());
}

/// An angle within the angular tolerance of `2π` is the full turn, one
/// above it is refused, and so are a zero and a non-finite one.
#[test]
fn the_angle_is_held_to_a_turn() {
    let axis = Axis::z_at(Point3::origin());
    let profile = tube_profile(false);
    let eps = Model::default().precision().angular_tolerance / 2.0;
    for angle in [TAU, TAU + eps, TAU - eps] {
        let mut m = Model::default();
        let (body, _) = revolve(&mut m, &profile, axis, angle).unwrap();
        assert_eq!(counts(&m, body), (4, 6, 4), "a full turn at {angle}");
    }
    let mut m = Model::default();
    let (body, _) = revolve(&mut m, &profile, axis, TAU - 1e-3).unwrap();
    assert_eq!(
        counts(&m, body),
        (8, 12, 6),
        "just short of a turn: two flat ends"
    );
    let refused = |angle: f64| revolve(&mut Model::default(), &profile, axis, angle).unwrap_err();
    assert!(matches!(
        refused(TAU + 1e-3),
        OpError::Degenerate {
            reason: Reason::AngleAboveTurn,
            ..
        }
    ));
    assert!(matches!(
        refused(0.0),
        OpError::Degenerate {
            reason: Reason::NotPositive { what: "angle", .. },
            ..
        }
    ));
    assert!(matches!(
        refused(f64::NAN),
        OpError::Degenerate {
            reason: Reason::NonFinite { what: "angle" },
            ..
        }
    ));
}

/// Every refusal of the profile against the axis, and the model as it
/// was after each: the tube revolved afterwards has the ids of one
/// revolved in a fresh model.
#[test]
fn the_profile_is_held_clear_of_the_axis_and_the_axis_to_the_plane() {
    let z = Axis::z_at(Point3::origin());
    let sketch = |outer: ProfileLoop| Profile {
        plane: xz_plane(),
        outer,
        holes: Vec::new(),
    };
    let p = |u, v| Point2::new(u, v);
    let mut m = Model::default();
    let reason = |m: &mut Model, profile: &Profile, axis: Axis| match revolve(m, profile, axis, 1.0)
    {
        Err(OpError::Degenerate { reason, .. }) => reason,
        other => panic!("{other:?}"),
    };
    // A vertex on the axis with no segment along it: a full turn's surface
    // would touch itself there (a partial turn builds it), NonManifold.
    let apex = sketch(ProfileLoop::Path {
        start: p(0.0, 0.0),
        segments: vec![
            ProfileSegment::LineTo(p(1.0, -1.0)),
            ProfileSegment::LineTo(p(1.0, 1.0)),
            ProfileSegment::LineTo(p(0.0, 0.0)),
        ],
    });
    assert!(matches!(
        revolve(&mut m, &apex, z, TAU),
        Err(OpError::Degenerate {
            reason: Reason::NonManifold,
            ..
        })
    ));
    // Straddling it.
    let across = sketch(rectangle(-1.0, 1.0, -1.0, 1.0, false));
    assert_eq!(reason(&mut m, &across, z), Reason::ProfileCrossesAxis);
    // An axis tilted out of the plane, and one lifted off it.
    let tube = tube_profile(false);
    let tilted = Axis::new(Point3::origin(), Vec3::new(0.0, 0.1, 1.0)).unwrap();
    assert_eq!(reason(&mut m, &tube, tilted), Reason::AxisNotInProfilePlane);
    let lifted = Axis::z_at(Point3::new(0.0, 1.0, 0.0));
    assert_eq!(reason(&mut m, &tube, lifted), Reason::AxisNotInProfilePlane);
    // A full turn of a profile with a hole closes the hole into a cavity:
    // one lump, its void the hole's sides, Generated from the hole's loop.
    // It is no refusal, so it is built in a model of its own.
    let holed = Profile {
        holes: vec![rectangle(1.25, 1.75, -0.5, 0.5, false)],
        ..tube_profile(false)
    };
    let mut own = Model::default();
    let (ring, provenance) = revolve(&mut own, &holed, z, TAU).unwrap();
    let report = check(&own, ring, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let shells = own.shells(ring).unwrap();
    let found = lumps(&own, ring).unwrap();
    assert_eq!((shells.len(), found.len()), (2, 1));
    assert_eq!(found[0].outer, shells[0]);
    assert_eq!(found[0].voids, [shells[1]]);
    let cavity = Role::Revolve(SweepPart::Cavity {
        loop_index: 1,
        segment: 0,
    });
    assert_eq!(provenance.generated_from(cavity), [shells[1].shape()]);
    assert!(
        revolve(&mut m, &holed, z, 1.0).is_ok(),
        "a partial turn's hole opens onto the ends"
    );
    // An invalid sketch reaches the caller as the profile's own error.
    let bowtie = sketch(ProfileLoop::Path {
        start: p(1.0, 0.0),
        segments: vec![
            ProfileSegment::LineTo(p(11.0, 0.0)),
            ProfileSegment::LineTo(p(3.0, 8.0)),
            ProfileSegment::LineTo(p(9.0, 10.0)),
            ProfileSegment::LineTo(p(1.0, 0.0)),
        ],
    });
    assert!(matches!(
        revolve(&mut m, &bowtie, z, 1.0),
        Err(OpError::Profile(ProfileError::SelfIntersecting {
            loop_index: 0,
            ..
        }))
    ));
    // Nothing of any refusal stayed behind — nor of the partial turn
    // that succeeded, which a fresh model is given too.
    let mut fresh = Model::default();
    revolve(&mut fresh, &holed, z, 1.0).unwrap();
    let (body, _) = revolve(&mut m, &tube, z, TAU).unwrap();
    let (again, _) = revolve(&mut fresh, &tube, z, TAU).unwrap();
    assert_eq!(
        dump_text(&m, body).unwrap(),
        dump_text(&fresh, again).unwrap(),
        "the model is as it was"
    );
}

/// The consumer's rectangle `[0, 1] × [−1, 1]` with its side `u = 0` on
/// the axis — segments 0 to 3 the bottom, the wall, the top and the side
/// on the axis; vertices 0 and 3 on the axis. A full turn is a solid
/// cylinder: the side on the axis sweeps no face and no edge, its
/// vertices nothing, the bottom and the top a disc of one rise each. A
/// partial turn — a quarter, and three quarters, reflex at the axis — is
/// a sector whose flat ends share the one edge on the axis, `Generated`
/// from that side's `StartEdge`, its two vertices shared too and neither
/// sweeping a rise.
#[test]
fn a_profile_along_its_axis_sweeps_nothing_there() {
    let z = Axis::z_at(Point3::origin());
    let profile = Profile {
        plane: xz_plane(),
        outer: rectangle(0.0, 1.0, -1.0, 1.0, false),
        holes: Vec::new(),
    };
    let side = |segment| SweepPart::Side {
        loop_index: 0,
        segment,
    };
    let start_edge = |segment| SweepPart::StartEdge {
        loop_index: 0,
        segment,
    };
    let rise = |vertex| SweepPart::Rise {
        loop_index: 0,
        vertex,
    };
    let start_vertex = |vertex| SweepPart::StartVertex {
        loop_index: 0,
        vertex,
    };
    let end_vertex = |vertex| SweepPart::EndVertex {
        loop_index: 0,
        vertex,
    };
    for (angle, line) in [
        (TAU, "2/3/3/3/1 g0 = 0"),
        (PI / 2.0, "6/9/5/5/1 g0 = 0"),
        (3.0 * PI / 2.0, "6/9/5/5/1 g0 = 0"),
    ] {
        let full = angle == TAU;
        let mut m = Model::default();
        let (body, p) = revolve(&mut m, &profile, z, angle).unwrap();
        let report = check(&m, body, Level::Full);
        assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "{angle}\n{report}"
        );
        assert_eq!(report.euler().unwrap().to_string(), line, "{angle}");
        // A cylinder of radius 1 and height 2 over `angle` of a turn.
        let props = mass_properties(&m, body).unwrap();
        assert!(close(props.volume, angle), "{angle}: {}", props.volume);
        let caps = if full { 0.0 } else { 4.0 };
        assert!(
            close(props.area, 3.0 * angle + caps),
            "{angle}: {}",
            props.area
        );

        let parts = recorded_parts(&m, body, &p).unwrap();
        assert!(!parts.contains(&side(3)), "{angle}: {parts:?}");
        for vertex in [0, 3] {
            assert!(!parts.contains(&rise(vertex)), "{angle}: {parts:?}");
            assert!(!parts.contains(&end_vertex(vertex)), "{angle}: {parts:?}");
            assert_eq!(parts.contains(&start_vertex(vertex)), !full, "{angle}");
        }
        assert_eq!(parts.contains(&start_edge(3)), !full, "{angle}");
        assert!(!parts.contains(&SweepPart::EndEdge {
            loop_index: 0,
            segment: 3
        }));
        if full {
            continue;
        }
        // The edge on the axis is used once by each flat end, and nothing
        // else uses it.
        let [
            Shape {
                id: EntityId::Edge(shared),
                ..
            },
        ] = p.generated_from(Role::Revolve(start_edge(3)))
        else {
            panic!("{angle}: {p}");
        };
        let caps: BTreeSet<_> = [SweepPart::StartCap, SweepPart::EndCap]
            .into_iter()
            .map(|c| p.generated_from(Role::Revolve(c))[0].id)
            .collect();
        let users: BTreeSet<_> = m
            .edge_uses(*shared)
            .unwrap()
            .iter()
            .map(|u| EntityId::from(u.face))
            .collect();
        assert_eq!(users, caps, "{angle}");
    }
}

/// Notches cut in from the axis (`docs/ARCHITECTURE.md` §Operations): in
/// a full turn the profile's chains — its runs of segments off
/// the axis — are shells, the one spanning the others the lump's outer
/// shell and each notch a void of it, named by its loop and the lowest
/// segment the consumer wrote in it, so two notches of one loop are two
/// voids with two names; a quarter turn opens them onto the flat ends, one
/// shell. A profile touching the axis at a vertex with no segment along it
/// is refused in a full turn as `NonManifold`, naming nothing, and built
/// in a partial one.
#[test]
fn notches_reaching_the_axis_close_into_voids_and_a_pinch_is_non_manifold() {
    let z = Axis::z_at(Point3::origin());
    let p = |u: f64, v: f64| Point2::new(u, v);
    // x ∈ [0, 2], z ∈ [0, 5], notched to x = 1 at z ∈ [3, 4] (segments 4 to
    // 6) and z ∈ [1, 2] (segments 8 to 10); segments 3, 7 and 11 lie along
    // the axis.
    let corners = [
        (2.0, 0.0),
        (2.0, 5.0),
        (0.0, 5.0),
        (0.0, 4.0),
        (1.0, 4.0),
        (1.0, 3.0),
        (0.0, 3.0),
        (0.0, 2.0),
        (1.0, 2.0),
        (1.0, 1.0),
        (0.0, 1.0),
        (0.0, 0.0),
    ];
    let notched = Profile {
        plane: xz_plane(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: corners
                .iter()
                .map(|&(u, v)| ProfileSegment::LineTo(p(u, v)))
                .collect(),
        },
        holes: Vec::new(),
    };
    let mut m = Model::default();
    let (body, provenance) = revolve(&mut m, &notched, z, TAU).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(report.euler().unwrap().to_string(), "6/9/9/9/3 g0 = 0");
    let volume = mass_properties(&m, body).unwrap().volume;
    assert!(close(volume, 18.0 * PI), "{volume}");
    let shells = m.shells(body).unwrap();
    let found = lumps(&m, body).unwrap();
    assert_eq!((shells.len(), found.len()), (3, 1));
    assert_eq!(found[0].outer, shells[0]);
    let voids: BTreeSet<_> = found[0].voids.iter().copied().collect();
    assert_eq!(voids, BTreeSet::from([shells[1], shells[2]]));
    let role = |part| provenance.generated_from(Role::Revolve(part)).to_vec();
    assert_eq!(role(SweepPart::Shell), [shells[0].shape()]);
    let cavity = |segment| SweepPart::Cavity {
        loop_index: 0,
        segment,
    };
    assert_eq!(role(cavity(4)), [shells[1].shape()], "the upper notch");
    assert_eq!(role(cavity(8)), [shells[2].shape()], "the lower notch");
    recorded_parts(&m, body, &provenance).unwrap();

    let mut m = Model::default();
    let (quarter, _) = revolve(&mut m, &notched, z, PI / 2.0).unwrap();
    let report = check(&m, quarter, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(m.shells(quarter).unwrap().len(), 1);

    let kite = Profile {
        plane: xz_plane(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(1.0, -1.0)),
                ProfileSegment::LineTo(p(2.0, 0.0)),
                ProfileSegment::LineTo(p(1.0, 1.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let mut m = Model::default();
    assert!(matches!(
        revolve(&mut m, &kite, z, TAU),
        Err(OpError::Degenerate {
            reason: Reason::NonManifold,
            entities,
        }) if entities.is_empty()
    ));
    assert!(
        revolve(&mut m, &kite, z, PI / 2.0).is_ok(),
        "a partial turn's flat ends make the vertex manifold"
    );
}

/// The surface kind of every face of `body`, with a cone's `Z` against
/// `axis` — `true` where they agree, the cone widening along the axis.
fn surfaces_of(m: &Model, body: Body, axis: &Axis) -> Vec<(SurfaceKind, Option<bool>)> {
    m.faces(body)
        .unwrap()
        .iter()
        .map(|f| {
            let surface = m.surface(m.face(f.id).unwrap().surface()).unwrap();
            let widening = match surface {
                Surface::Cone { frame, .. } => Some(frame.z().dot(&axis.direction) > 0.0),
                _ => None,
            };
            (surface.kind(), widening)
        })
        .collect()
}

/// `prop::profile::general` earns its name: over the configured cases it
/// sweeps a cone widening along the axis, one narrowing, a sphere and a
/// torus — so the property above has exercised every arm, not merely
/// admitted it.
#[test]
fn the_general_profile_sweeps_every_surface_kind() {
    let seen: RefCell<BTreeSet<(SurfaceKind, Option<bool>)>> = RefCell::new(BTreeSet::new());
    prop::check(prop::profile::general(), |sweep| {
        let mut m = Model::default();
        // A partial turn: a full turn of a profile with holes is refused.
        let (body, _) = revolve(&mut m, &sweep.profile, sweep.axis, 1.0).map_err(fail)?;
        seen.borrow_mut().extend(surfaces_of(&m, body, &sweep.axis));
        Ok(())
    });
    let seen = seen.into_inner();
    for kind in [
        (SurfaceKind::Plane, None),
        (SurfaceKind::Cone, Some(true)),
        (SurfaceKind::Cone, Some(false)),
        (SurfaceKind::Sphere, None),
        (SurfaceKind::Torus, None),
    ] {
        assert!(seen.contains(&kind), "{kind:?} never swept; seen {seen:?}");
    }
}

/// The profile strategies reach the axis, so the properties above hold
/// over every case that touches it rather than merely admit them: over
/// the configured cases a staircase's full turn
/// closes a notch cut in from the axis into a void and its partial turn
/// shares an edge on the axis between the flat ends, and a general
/// profile's revolve closes on a degenerate edge at a cone's apex and at a
/// sphere's pole.
#[test]
fn the_profile_strategies_touch_the_axis() {
    let seen: RefCell<BTreeSet<&'static str>> = RefCell::new(BTreeSet::new());
    prop::check(prop::profile::rectilinear(), |sweep| {
        let mut m = Model::default();
        let (body, p) = revolve(&mut m, &sweep.profile, sweep.axis, sweep.angle).map_err(fail)?;
        let parts = recorded_parts(&m, body, &p)?;
        let mut seen = seen.borrow_mut();
        if parts
            .iter()
            .any(|part| matches!(part, SweepPart::Cavity { loop_index: 0, .. }))
        {
            seen.insert("a notch closed into a void");
        }
        let shared = parts.iter().any(|&part| match part {
            SweepPart::StartEdge {
                loop_index,
                segment,
            } => {
                parts.contains(&SweepPart::StartCap)
                    && !parts.contains(&SweepPart::EndEdge {
                        loop_index,
                        segment,
                    })
            }
            _ => false,
        });
        if shared {
            seen.insert("an edge on the axis shared by the flat ends");
        }
        Ok(())
    });
    prop::check(prop::profile::general(), |sweep| {
        let mut m = Model::default();
        let (body, _) = revolve(&mut m, &sweep.profile, sweep.axis, sweep.angle).map_err(fail)?;
        for f in m.faces(body).map_err(fail)? {
            let face = m.face(f.id).map_err(fail)?;
            let degenerate = face
                .loops()
                .iter()
                .flat_map(|l| l.coedges())
                .any(|c| m.edge(c.edge()).is_ok_and(|e| e.is_degenerate()));
            if degenerate {
                let kind = m.surface(face.surface()).map_err(fail)?.kind();
                seen.borrow_mut().insert(if kind == SurfaceKind::Cone {
                    "a cone's apex"
                } else if kind == SurfaceKind::Sphere {
                    "a sphere's pole"
                } else {
                    "a degenerate edge on another surface"
                });
            }
        }
        Ok(())
    });
    let seen = seen.into_inner();
    assert_eq!(
        seen,
        BTreeSet::from([
            "a notch closed into a void",
            "an edge on the axis shared by the flat ends",
            "a cone's apex",
            "a sphere's pole",
        ])
    );
}

/// The recipe of a profile in the `xz` plane revolved `angle_deg` about
/// `z`, for a scratch fixture the oracle answers.
fn revolved_recipe(
    description: &str,
    angle_deg: f64,
    outer: Loop,
    volume: &str,
    area: &str,
) -> Recipe {
    let n = |v: f64| Num::Literal(v);
    Recipe {
        description: description.to_string(),
        params: Default::default(),
        variants: Default::default(),
        steps: vec![
            Step::Profile {
                name: "sketch".into(),
                plane: Plane {
                    origin: [n(0.0), n(0.0), n(0.0)],
                    x: [n(1.0), n(0.0), n(0.0)],
                    y: [n(0.0), n(0.0), n(1.0)],
                },
                outer,
                holes: Vec::new(),
            },
            Step::Revolve {
                name: "result".into(),
                profile: "sketch".into(),
                axis: fixtures::Axis {
                    origin: [n(0.0), n(0.0), n(0.0)],
                    direction: [n(0.0), n(0.0), n(1.0)],
                },
                angle_deg: n(angle_deg),
            },
        ],
        result: "result".into(),
        probes: Vec::new(),
        precision: Default::default(),
        tolerances: Default::default(),
        analytic: Analytic {
            volume: Some(Num::Expr(volume.into())),
            area: Some(Num::Expr(area.into())),
            ..Default::default()
        },
    }
}

fn line_to(u: f64, v: f64) -> Segment {
    Segment::Line {
        line_to: [Num::Literal(u), Num::Literal(v)],
    }
}

/// The profile of a recipe's `profile` step, as `ops::revolve` takes it.
fn profile_of(recipe: &Recipe) -> Profile {
    let Some(Step::Profile {
        name,
        plane,
        outer,
        holes,
    }) = recipe.steps.first()
    else {
        panic!("no profile step");
    };
    fixtures::geom::build_profile(name, plane, outer, holes, &Default::default()).unwrap()
}

/// The three quadric-faced revolves, held to the checker at `Full` with
/// nothing unchecked and to their closed forms: a trapezoid's frustum less
/// its bore, narrowing and widening, an arc's spherical zone less its
/// bore, and a circle's ring — the last with the counts and the Euler line
/// of `sample::torus`. The oracle reads them in the corpus, as
/// `sweep/revolve-frustum`, `revolve-barrel` and `revolve-ring`.
#[test]
fn the_frustum_the_zone_and_the_ring_have_their_closed_forms() {
    let z = Axis::z_at(Point3::origin());
    // x ∈ [1, 4] at z = −1 narrowing to x ∈ [1, 2] at z = 1: the cone's
    // Z is −z; and the same widening, the cone's Z is +z.
    let frustum = |widening: bool| {
        let (lo, hi) = if widening { (2.0, 4.0) } else { (4.0, 2.0) };
        revolved_recipe(
            "a trapezoid revolved: a frustum less its bore",
            360.0,
            Loop::Path {
                start: [Num::Literal(1.0), Num::Literal(-1.0)],
                segments: vec![
                    line_to(lo, -1.0),
                    line_to(hi, 1.0),
                    line_to(1.0, 1.0),
                    line_to(1.0, -1.0),
                ],
            },
            "2 * pi * (16 + 8 + 4) / 3 - 2 * pi",
            "15 * pi + 3 * pi + 6 * sqrt(8) * pi + 4 * pi",
        )
    };
    let frustum_volume = 2.0 * PI * (16.0 + 8.0 + 4.0) / 3.0 - 2.0 * PI;
    let frustum_area = 15.0 * PI + 3.0 * PI + 6.0 * 8f64.sqrt() * PI + 4.0 * PI;
    // The arc of radius 3 about the origin from z = −1 to z = 1, at
    // x = √8: a zone of height 2 between two discs of radius √8.
    let a = 8f64.sqrt();
    let zone = revolved_recipe(
        "an arc centred on the axis revolved: a spherical zone less its bore",
        360.0,
        Loop::Path {
            start: [Num::Literal(1.0), Num::Literal(-1.0)],
            segments: vec![
                line_to(a, -1.0),
                Segment::Arc {
                    arc_to: [Num::Literal(a), Num::Literal(1.0)],
                    via: [Num::Literal(3.0), Num::Literal(0.0)],
                },
                line_to(1.0, 1.0),
                line_to(1.0, -1.0),
            ],
        },
        "pi * 2 * (3 * 8 + 3 * 8 + 4) / 6 - 2 * pi",
        "12 * pi + 14 * pi + 4 * pi",
    );
    let zone_volume = PI * 2.0 * (3.0 * 8.0 + 3.0 * 8.0 + 4.0) / 6.0 - 2.0 * PI;
    let zone_area = 12.0 * PI + 14.0 * PI + 4.0 * PI;
    // A circle of radius 2 centred 5 from the axis.
    let ring = ring_recipe();
    let ring_volume = 2.0 * PI * PI * 5.0 * 4.0;
    let ring_area = 4.0 * PI * PI * 5.0 * 2.0;

    let cases = [
        (
            "revolve-frustum",
            frustum(false),
            frustum_volume,
            frustum_area,
            (4, 6, 4),
            false,
        ),
        (
            "revolve-frustum-widening",
            frustum(true),
            frustum_volume,
            frustum_area,
            (4, 6, 4),
            true,
        ),
        (
            "revolve-zone",
            zone,
            zone_volume,
            zone_area,
            (4, 6, 4),
            false,
        ),
        (
            "revolve-ring",
            ring,
            ring_volume,
            ring_area,
            (1, 2, 1),
            false,
        ),
    ];
    for (name, recipe, volume, area, expected_counts, widening) in cases {
        let mut m = Model::default();
        let profile = profile_of(&recipe);
        let (body, _) = revolve(&mut m, &profile, z, TAU).unwrap();
        let report = check(&m, body, Level::Full);
        assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "{name}\n{report}"
        );
        let props = mass_properties(&m, body).unwrap();
        assert!(
            close(props.volume, volume),
            "{name}: {} vs {volume}",
            props.volume
        );
        assert!(close(props.area, area), "{name}: {} vs {area}", props.area);
        assert_eq!(counts(&m, body), expected_counts, "{name}");
        let surfaces = surfaces_of(&m, body, &z);
        if name.starts_with("revolve-frustum") {
            assert!(
                surfaces.contains(&(SurfaceKind::Cone, Some(widening))),
                "{name}: {surfaces:?}"
            );
        }
    }

    // The ring is `sample::torus` as a revolve builds it: one face, two
    // seams, one vertex, genus 1.
    let mut m = Model::default();
    let (ring, _) = revolve(&mut m, &profile_of(&ring_recipe()), z, TAU).unwrap();
    let mut n = Model::default();
    let torus = sample::torus(&mut n, Point3::origin(), 5.0, 2.0).unwrap();
    assert_eq!(
        euler_line(&m, ring).unwrap(),
        euler_line(&n, torus).unwrap()
    );
    assert_eq!(counts(&m, ring), counts(&n, torus));
}

/// A face closing at the axis (`docs/DATA-MODEL.md` §Invariants, E6):
/// a right triangle with a leg on the axis turns into a cone, its apex
/// a degenerate edge, in a full turn and a quarter; a half disc on the
/// axis into a ball with a degenerate edge at each pole — `sample::sphere`
/// as a revolve builds it; and a quarter turn of a kite touching the axis
/// at one vertex into two cones closing there, each on a degenerate edge
/// of its own. Each is clean at `Full` with nothing unchecked, closes its
/// Euler line at genus 0 without the degenerate edges, has its
/// closed-form volume and area, meshes closed with every
/// degenerate edge one index, accounts for every entity — a `Rise` naming
/// the degenerate edge of each face closing there — and is read back from
/// Arris's STEP by the oracle, whose reader rebuilds the degenerate edges
/// the writer leaves out.
#[test]
fn a_cone_a_ball_and_a_pinch_close_on_degenerate_edges_at_the_axis() {
    let z = Axis::z_at(Point3::origin());
    let at = |u: f64, v: f64| [Num::Literal(u), Num::Literal(v)];
    let triangle = || Loop::Path {
        start: at(0.0, 0.0),
        segments: vec![line_to(1.0, 0.0), line_to(0.0, 1.0), line_to(0.0, 0.0)],
    };
    let half_disc = Loop::Path {
        start: at(0.0, -1.0),
        segments: vec![
            Segment::Arc {
                arc_to: at(0.0, 1.0),
                via: at(1.0, 0.0),
            },
            line_to(0.0, -1.0),
        ],
    };
    let kite = Loop::Path {
        start: at(0.0, 0.0),
        segments: vec![
            line_to(1.0, -1.0),
            line_to(2.0, 0.0),
            line_to(1.0, 1.0),
            line_to(0.0, 0.0),
        ],
    };
    let cone_area = PI * (1.0 + 2f64.sqrt());
    // (name, angle, profile, closed forms as the recipe states them and as
    // numbers, the Euler line, the degenerate edges)
    let cases = [
        (
            "revolve-apex-cone",
            360.0,
            triangle(),
            ("pi / 3", "pi * (1 + sqrt(2))"),
            (PI / 3.0, cone_area),
            "2/2/2/2/1 g0 = 0",
            1,
        ),
        (
            "revolve-apex-cone-quarter",
            90.0,
            triangle(),
            ("pi / 12", "pi * (1 + sqrt(2)) / 4 + 1"),
            (PI / 12.0, cone_area / 4.0 + 1.0),
            "4/6/4/4/1 g0 = 0",
            1,
        ),
        (
            "revolve-ball",
            360.0,
            half_disc,
            ("4 * pi / 3", "4 * pi"),
            (4.0 * PI / 3.0, 4.0 * PI),
            "2/1/1/1/1 g0 = 0",
            2,
        ),
        (
            "revolve-pinch-quarter",
            90.0,
            kite,
            ("pi", "2 * sqrt(2) * pi + 4"),
            (PI, 2.0 * 2f64.sqrt() * PI + 4.0),
            "7/11/6/6/1 g0 = 0",
            2,
        ),
    ];
    for (name, angle_deg, outer, (volume_expr, area_expr), (volume, area), line, singular) in cases
    {
        let recipe = revolved_recipe(name, angle_deg, outer, volume_expr, area_expr);
        let mut m = Model::default();
        let (body, p) = revolve(&mut m, &profile_of(&recipe), z, angle_deg.to_radians())
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        let report = check(&m, body, Level::Full);
        assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "{name}\n{report}\n{}",
            dump_text(&m, body).unwrap()
        );
        assert_eq!(report.euler().unwrap().to_string(), line, "{name}");
        let degenerate: Vec<_> = m
            .edges(body)
            .unwrap()
            .into_iter()
            .filter(|e| m.edge(e.id).unwrap().is_degenerate())
            .collect();
        assert_eq!(degenerate.len(), singular, "{name}");
        // Each degenerate edge is where its face's surface is singular: a
        // ray through the apex or a pole lands on the face's boundary
        // there, so the classifier abandons that direction rather than
        // count the hit — its `uv`, `u = 0` at the singular `v`, is on the
        // degenerate edge's side of the domain at every `u`.
        for e in &degenerate {
            let edge = m.edge(e.id).unwrap();
            let at = m.vertex(edge.start()).unwrap().point();
            for used in m.edge_uses(e.id).unwrap().iter() {
                let face = m.face(used.face).unwrap();
                let surface = m.surface(face.surface()).unwrap();
                let v = match surface {
                    Surface::Cone {
                        radius, half_angle, ..
                    } => -radius / half_angle.sin(),
                    other => other.project(at).unwrap().uv.y,
                };
                let domain =
                    FaceDomain::of(&m, used.face, m.precision().parametric_tolerance).unwrap();
                let [us, _] = domain.uv_box().unwrap();
                for u in [us.lo(), us.lerp(0.5), us.hi(), 0.0] {
                    assert_eq!(
                        domain.side(Point2::new(u, v)).0,
                        Side::Boundary,
                        "{name}: ({u}, {v}) on {surface:?}"
                    );
                }
            }
        }
        // A full turn classifies the axis by a cast past its apex or pole.
        if angle_deg == 360.0 {
            let on_axis = |z: f64| classify_point(&m, body, Point3::new(0.0, 0.0, z)).unwrap();
            assert_eq!(on_axis(0.5), Classification::Inside, "{name}");
            assert_eq!(on_axis(1.5), Classification::Outside, "{name}");
        }

        let props = mass_properties(&m, body).unwrap();
        assert!(close(props.volume, volume), "{name}: {}", props.volume);
        assert!(close(props.area, area), "{name}: {}", props.area);
        let mesh = tessellate(&m, body, MESH_CHORD).unwrap();
        let meshed = mesh
            .signed_volume()
            .unwrap_or_else(|| panic!("{name}: the mesh is not closed"));
        assert!(
            (meshed - volume).abs() <= MESH_CHORD * area,
            "{name}: mesh volume {meshed} vs {volume}"
        );
        for e in &degenerate {
            assert_eq!(mesh.edge_polyline(e.id).unwrap().len(), 1, "{name}");
        }
        recorded_parts(&m, body, &p).unwrap();

        let dir = oracle::scratch_fixture(name, &recipe).unwrap();
        oracle::compare_dir(&dir, &step::write(&m, &[body]).unwrap(), None, name)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
    }
}

fn ring_recipe() -> Recipe {
    revolved_recipe(
        "a circle revolved: a ring torus",
        360.0,
        Loop::Circle {
            circle: fixtures::Circle {
                center: [Num::Literal(5.0), Num::Literal(0.0)],
                radius: Num::Literal(2.0),
            },
        },
        "2 * pi * pi * 5 * 4",
        "4 * pi * pi * 5 * 2",
    )
}

/// An arc that stays clear of the axis while its circle crosses it would
/// sweep a spindle torus, which the data model does not hold: refused by
/// name, the model untouched. A whole circle crossing the axis is the
/// profile crossing it, refused as that.
#[test]
fn an_arc_whose_circle_crosses_the_axis_is_a_spindle_torus() {
    let z = Axis::z_at(Point3::origin());
    let p = |u, v| Point2::new(u, v);
    // From (2, −1) through (2.2, 0) to (2, 1): radius 2.6 about (−0.4, 0).
    let bulge = Profile {
        plane: xz_plane(),
        outer: ProfileLoop::Path {
            start: p(2.0, -1.0),
            segments: vec![
                ProfileSegment::ArcTo {
                    to: p(2.0, 1.0),
                    via: p(2.2, 0.0),
                },
                ProfileSegment::LineTo(p(1.0, 1.0)),
                ProfileSegment::LineTo(p(1.0, -1.0)),
                ProfileSegment::LineTo(p(2.0, -1.0)),
            ],
        },
        holes: Vec::new(),
    };
    let mut m = Model::default();
    assert!(matches!(
        revolve(&mut m, &bulge, z, TAU),
        Err(OpError::Degenerate {
            reason: Reason::SpindleTorus,
            ..
        })
    ));
    let circle = Profile {
        plane: xz_plane(),
        outer: ProfileLoop::Circle {
            center: p(1.0, 0.0),
            radius: 2.0,
        },
        holes: Vec::new(),
    };
    assert!(matches!(
        revolve(&mut m, &circle, z, TAU),
        Err(OpError::Degenerate {
            reason: Reason::ProfileCrossesAxis,
            ..
        })
    ));
    let tube = tube_profile(false);
    let (body, _) = revolve(&mut m, &tube, z, TAU).unwrap();
    let mut fresh = Model::default();
    let (again, _) = revolve(&mut fresh, &tube, z, TAU).unwrap();
    assert_eq!(
        dump_text(&m, body).unwrap(),
        dump_text(&fresh, again).unwrap(),
        "the model is as it was"
    );
}

/// A revolve refuses an elliptic segment by name (ADR-0014): the surface
/// it would sweep has no variant. The reason names the first such segment
/// in the *consumer's* order whichever way the loop had to be walked, and
/// an ellipse whose radii agree within the tolerance is a circle edge and
/// revolves.
#[test]
fn an_elliptic_segment_is_refused_naming_the_sketch() {
    let z = Axis::z_at(Point3::origin());
    let p = |u, v| Point2::new(u, v);
    let mut m = Model::default();
    let elliptic_revolve = |profile: &Profile, m: &mut Model| match revolve(m, profile, z, TAU) {
        Err(OpError::Degenerate {
            reason:
                Reason::EllipticRevolve {
                    loop_index,
                    segment,
                },
            ..
        }) => (loop_index, segment),
        other => panic!("expected an elliptic refusal, got {other:?}"),
    };

    // A full ellipse 5 from the axis: the one segment of loop 0.
    let ring = Profile {
        plane: xz_plane(),
        outer: ProfileLoop::Ellipse {
            center: p(5.0, 0.0),
            major: Vec2::new(1.6, 1.2),
            minor_radius: 1.0,
        },
        holes: Vec::new(),
    };
    assert_eq!(elliptic_revolve(&ring, &mut m), (0, 0));

    // A bulged rectangle: lines but for segment 2, the ellipse over the
    // top. An elliptic hole is named only if nothing earlier is.
    let bulge = |ccw: bool| ProfileSegment::EllipseTo {
        to: p(2.0, 1.0),
        center: p(3.0, 1.0),
        major: Vec2::new(1.0, 0.0),
        minor_radius: 0.5,
        ccw,
    };
    let hole = ProfileLoop::Ellipse {
        center: p(3.0, -0.5),
        major: Vec2::new(0.5, 0.0),
        minor_radius: 0.25,
    };
    let bulged = Profile {
        plane: xz_plane(),
        outer: ProfileLoop::Path {
            start: p(2.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(4.0, -1.0)),
                ProfileSegment::LineTo(p(4.0, 1.0)),
                bulge(true),
                ProfileSegment::LineTo(p(2.0, -1.0)),
            ],
        },
        holes: vec![hole.clone()],
    };
    assert_eq!(elliptic_revolve(&bulged, &mut m), (0, 2));
    let holed = Profile {
        outer: ProfileLoop::Path {
            start: p(2.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(4.0, -1.0)),
                ProfileSegment::LineTo(p(4.0, 1.0)),
                ProfileSegment::LineTo(p(2.0, 1.0)),
                ProfileSegment::LineTo(p(2.0, -1.0)),
            ],
        },
        holes: vec![hole],
        ..bulged.clone()
    };
    assert_eq!(elliptic_revolve(&holed, &mut m), (1, 0));

    // An oval of two half-ellipses, written clockwise so the walk turns
    // it round: the segment named is the consumer's first, not the
    // walk's, which would be segment 1.
    let half = |to, ccw| ProfileSegment::EllipseTo {
        to,
        center: p(3.0, 0.0),
        major: Vec2::new(1.0, 0.0),
        minor_radius: 0.5,
        ccw,
    };
    let clockwise = Profile {
        outer: ProfileLoop::Path {
            start: p(2.0, 0.0),
            segments: vec![half(p(4.0, 0.0), false), half(p(2.0, 0.0), false)],
        },
        holes: Vec::new(),
        ..bulged
    };
    assert_eq!(elliptic_revolve(&clockwise, &mut m), (0, 0));

    // Nothing of any of the refusals stayed behind: the same revolve in
    // a fresh model dumps the same.
    let mut fresh = Model::default();

    // Radii that agree within the linear tolerance are a circle edge
    // (ADR-0014), which sweeps the ring torus every arm already has.
    let round = Profile {
        plane: xz_plane(),
        outer: ProfileLoop::Ellipse {
            center: p(5.0, 0.0),
            major: Vec2::new(2.0, 0.0),
            minor_radius: 2.0 - 1e-9,
        },
        holes: Vec::new(),
    };
    let (body, _) = revolve(&mut m, &round, z, TAU).unwrap();
    let report = check(&m, body, Level::Full);
    assert!(
        report.is_ok() && report.unchecked().is_empty(),
        "not clean at Full\n{report}"
    );
    assert_eq!(m.faces(body).unwrap().len(), 1, "the torus");
    assert_eq!(m.edges(body).unwrap().len(), 2, "its two seams");
    let (again, _) = revolve(&mut fresh, &round, z, TAU).unwrap();
    assert_eq!(
        dump_text(&m, body).unwrap(),
        dump_text(&fresh, again).unwrap(),
        "the model is as it was"
    );
}
