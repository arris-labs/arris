//! `ops::extrude` (`docs/ARCHITECTURE.md` §Operations): a thousand general
//! profiles — lines, outward and inward arcs, circular and polygonal
//! holes, in random poses — extruded along their plane's normal and
//! against it: the checker at `Full` with no violation and nothing
//! unchecked, volume and
//! area to Pappus's `A·L` and `2A + P·L`, the mesh closed within its
//! chord of the exact volume, one `Generated` per entity and every part
//! of the sketch present, the profile face on the profile's own plane,
//! the dump identical on two runs; the cross-check of a polygon with a
//! round hole against the cut of a cylinder from the polygon's extrusion
//! at two hundred poses; and every refusal with the model untouched.

use arris_debug::unmetered::{extrude, mass_properties, primitive_cylinder};
use std::collections::BTreeSet;

use arris_debug::prop::profile::{PROFILE_RADIUS, Sweep};
use arris_debug::prop::{finite_f64, frame, sweep};
use arris_debug::testing::{close, fail};
use arris_debug::unmetered::cut;
use arris_debug::unmetered::tessellate;
use arris_debug::{dump_text, prop, prop_shards};
use arris_geom::{Profile, ProfileError, ProfileLoop, ProfileSegment, Surface};

use arris_check::{Level, check};
use arris_math::{Axis, Frame, Point2, Tolerance, Vec2, Vec3};
use arris_ops::{InputReason, OpError, Reason, SweepReason};
use arris_topo::provenance::SweepPart;
use arris_topo::{Body, EntityId, Model, Orientation, Provenance, Role};
use core::f64::consts::TAU;
use proptest::prelude::*;
use proptest::test_runner::{Config, RngAlgorithm, TestRng, TestRunner};

/// The relative tolerance the Pappus identities and the cross-check hold
/// to.
const REL: f64 = 1e-9;

/// The chord tolerance the property meshes at. The mesh's surface lies
/// within this of the true one, so its volume is within this times the
/// area of the exact volume.
const MESH_CHORD: f64 = 1e-2;

/// The poses the cut cross-check runs at: a boolean per case, so fewer
/// than the property's thousand.
const CROSS_CHECK_CASES: u32 = 200;

/// [`arris_debug::testing::recorded_parts`] over `Role::Extrude`.
fn recorded_parts(
    m: &Model,
    body: Body,
    p: &Provenance,
) -> Result<BTreeSet<SweepPart>, TestCaseError> {
    arris_debug::testing::recorded_parts(m, body, p, |role| match role {
        Role::Extrude(part) => Some(part),
        _ => None,
    })
}

/// The parts an extrude of `profile` makes, from the sketch alone: two
/// caps, and for every segment a side, a start and an end edge, and for
/// the vertex it starts at a rise and two vertices.
fn expected_parts(profile: &Profile, tol: Tolerance) -> BTreeSet<SweepPart> {
    let mut parts = BTreeSet::from([
        SweepPart::Body,
        SweepPart::Shell,
        SweepPart::StartCap,
        SweepPart::EndCap,
    ]);
    for edges in profile.edges(tol).unwrap() {
        let n = edges.len();
        for e in &edges {
            let (loop_index, segment) = (e.loop_index, e.segment);
            let vertex = if e.reversed {
                (segment + 1) % n
            } else {
                segment
            };
            parts.extend([
                SweepPart::Side {
                    loop_index,
                    segment,
                },
                SweepPart::StartEdge {
                    loop_index,
                    segment,
                },
                SweepPart::EndEdge {
                    loop_index,
                    segment,
                },
                SweepPart::Rise { loop_index, vertex },
                SweepPart::StartVertex { loop_index, vertex },
                SweepPart::EndVertex { loop_index, vertex },
            ]);
        }
    }
    parts
}

/// The property every random profile is held to, extruded both ways.
fn extrudes_to_pappus(sweep: &Sweep) -> Result<(), TestCaseError> {
    let profile = &sweep.profile;
    for sign in [1.0, -1.0] {
        let direction = sign * profile.plane.z().into_inner();
        let mut m = Model::default();
        let tol = m.precision().tolerance();
        let (body, p) = extrude(&mut m, profile, direction, sweep.length)
            .map_err(|e| fail(format!("extrude along {sign}·n: {e}")))?;
        let fast = check(&m, body, Level::Fast);
        prop_assert!(fast.is_ok(), "not clean at Fast\n{}", fast);
        let report = check(&m, body, Level::Full);
        // Every wall of an extrude runs along the one direction, so two of
        // them are parallel cylinders, a pair S5 decides: nothing is
        // unchecked (docs/DATA-MODEL.md §Curves).
        prop_assert!(
            report.is_ok() && report.unchecked().is_empty(),
            "not clean at Full along {}·n\n{}\n{}",
            sign,
            report,
            dump_text(&m, body).map_err(fail)?
        );
        let props = mass_properties(&m, body).map_err(fail)?;
        let pappus = sweep::extruded(profile, sweep.length, tol).map_err(fail)?;
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
        prop_assert_eq!(recorded_parts(&m, body, &p)?, expected_parts(profile, tol));
        // The profile face is on the profile's own plane, frame and all,
        // and faces away from the sweep.
        let cap = p.generated_from(Role::Extrude(SweepPart::StartCap));
        prop_assert_eq!(cap.len(), 1);
        let face = m
            .faces(body)
            .map_err(fail)?
            .into_iter()
            .find(|f| EntityId::from(f.id) == cap[0].id)
            .ok_or_else(|| fail("the start cap is not a face of the body"))?;
        let surface = m
            .surface(m.face(face.id).map_err(fail)?.surface())
            .map_err(fail)?;
        prop_assert_eq!(
            surface,
            &Surface::Plane {
                frame: profile.plane
            }
        );
        let expected_use = if sign > 0.0 {
            Orientation::Reversed
        } else {
            Orientation::Forward
        };
        prop_assert_eq!(face.orientation, expected_use);
        let dump = dump_text(&m, body).map_err(fail)?;
        let mut again = Model::default();
        let (twice, _) = extrude(&mut again, profile, direction, sweep.length).map_err(fail)?;
        prop_assert_eq!(dump_text(&again, twice).map_err(fail)?, dump);
    }
    Ok(())
}

prop_shards! {
    /// A convex polygon with outward arcs (cylinders the material is
    /// inside), arcs bulging inward (cylinders it is outside) and holes,
    /// extruded both ways: clean at `Full` but for non-coaxial cylinder
    /// pairs, Pappus's volume and area, a closed mesh, complete
    /// provenance, the profile face on the profile's plane, a
    /// deterministic dump.
    general_profiles_extrude_to_pappus [shard_0 shard_1 shard_2 shard_3]
        (sweep) = prop::profile::general() => {
            extrudes_to_pappus(&sweep)
        }
}

/// A full ellipse in a random pose, its major axis at a random angle and
/// written either way round, extruded both ways: clean at `Full` with
/// nothing unchecked — its side is an elliptic cylinder against two
/// planes across its axis, every pair a closed form (ADR-0014) — volume
/// `π a b h` and area `2π a b + P h` by Pappus, a closed mesh, complete
/// provenance, a deterministic dump, and the side face's surface an
/// elliptic cylinder with `a ≥ b` whatever the consumer called major.
#[test]
fn ellipses_extrude_to_pi_a_b_h() {
    prop::check(prop::profile::ellipse(), |(profile, length)| {
        let ProfileLoop::Ellipse {
            major,
            minor_radius,
            ..
        } = &profile.outer
        else {
            return Err(fail("the strategy draws an ellipse loop"));
        };
        let (a, b) = (
            major.norm().max(*minor_radius),
            major.norm().min(*minor_radius),
        );
        let sweep = Sweep {
            profile: profile.clone(),
            axis: Axis::z_at(profile.plane.origin()),
            angle: TAU,
            length,
        };
        extrudes_to_pappus(&sweep)?;
        let mut m = Model::default();
        let (body, p) =
            extrude(&mut m, &profile, profile.plane.z().into_inner(), length).map_err(fail)?;
        let props = mass_properties(&m, body).map_err(fail)?;
        let exact = core::f64::consts::PI * a * b * length;
        prop_assert!(
            close(props.volume, exact),
            "volume {} vs πabh {exact}",
            props.volume
        );
        let side = p.generated_from(Role::Extrude(SweepPart::Side {
            loop_index: 0,
            segment: 0,
        }));
        prop_assert_eq!(side.len(), 1);
        let face = m
            .faces(body)
            .map_err(fail)?
            .into_iter()
            .find(|f| EntityId::from(f.id) == side[0].id)
            .ok_or_else(|| fail("the side is not a face of the body"))?;
        let surface = m
            .surface(m.face(face.id).map_err(fail)?.surface())
            .map_err(fail)?;
        let Surface::EllipticCylinder {
            major_radius,
            minor_radius,
            ..
        } = surface
        else {
            return Err(fail(format!(
                "the side is {surface:?}, no elliptic cylinder"
            )));
        };
        prop_assert!((major_radius - a).abs() <= 1e-12 && (minor_radius - b).abs() <= 1e-12);
        Ok(())
    });
}

/// A star polygon of straight segments with one round hole inside the
/// disc its chords leave free, in a random pose, and an extrude length.
fn polygon_with_hole() -> impl Strategy<Value = (Profile, Point2, f64, f64)> {
    (
        frame(),
        proptest::collection::vec((finite_f64(-0.2..=0.2), finite_f64(0.6..=1.0)), 5..=8),
        (
            finite_f64(0.0..=TAU),
            finite_f64(0.0..=0.4),
            finite_f64(0.1..=0.5),
        ),
        finite_f64(1.0..=10.0),
        any::<bool>(),
    )
        .prop_map(
            |(plane, vertices, (phase, offset, radius), length, backwards)| {
                let n = vertices.len();
                let share = TAU / n as f64;
                let mut points: Vec<Point2> = vertices
                    .iter()
                    .enumerate()
                    .map(|(k, &(jitter, fraction))| {
                        let a = (k as f64 + jitter) * share;
                        Point2::new(
                            fraction * PROFILE_RADIUS * a.cos(),
                            fraction * PROFILE_RADIUS * a.sin(),
                        )
                    })
                    .collect();
                let free = (0..n)
                    .map(|k| distance_to_segment(points[k], points[(k + 1) % n]))
                    .fold(f64::INFINITY, f64::min);
                let center = Point2::from(offset * free * Vec2::new(phase.cos(), phase.sin()));
                if backwards {
                    points.reverse();
                }
                let outer = ProfileLoop::Path {
                    start: points[0],
                    segments: points
                        .iter()
                        .skip(1)
                        .chain(core::iter::once(&points[0]))
                        .map(|&q| ProfileSegment::LineTo(q))
                        .collect(),
                };
                let profile = Profile {
                    plane,
                    outer,
                    holes: vec![ProfileLoop::Circle {
                        center,
                        radius: radius * free,
                    }],
                };
                (profile, center, radius * free, length)
            },
        )
}

/// The distance from the origin to the segment `a`–`b`.
fn distance_to_segment(a: Point2, b: Point2) -> f64 {
    let d = b - a;
    let t = (-a.coords.dot(&d) / d.norm_squared()).clamp(0.0, 1.0);
    (a.coords + t * d).norm()
}

fn counts(m: &Model, body: Body) -> (usize, usize, usize) {
    let c = m.closure(body).unwrap();
    (c.vertices.len(), c.edges.len(), c.faces.len())
}

/// A polygon with a round hole extruded is the polygon's extrusion with a
/// cylinder cut from it: the same volume, area and counts by the boolean
/// path, at two hundred poses.
#[test]
fn a_holed_extrusion_is_the_cut_of_its_bore() {
    let mut runner = TestRunner::new_with_rng(
        Config {
            cases: CROSS_CHECK_CASES,
            failure_persistence: None,
            ..Config::default()
        },
        TestRng::from_seed(RngAlgorithm::ChaCha, &prop::seed()),
    );
    let result = runner.run(&polygon_with_hole(), |(profile, center, radius, length)| {
        let n = profile.plane.z().into_inner();
        let mut m = Model::default();
        let (holed, _) = extrude(&mut m, &profile, n, length).map_err(fail)?;
        let solid = Profile {
            holes: Vec::new(),
            ..profile.clone()
        };
        let mut k = Model::default();
        let (plate, _) = extrude(&mut k, &solid, n, length).map_err(fail)?;
        let bore_axis = Axis::new(profile.to_world(center) - n, n).map_err(fail)?;
        let (bore, _) =
            primitive_cylinder(&mut k, bore_axis, radius, length + 2.0).map_err(fail)?;
        let (drilled, _) = cut(&mut k, plate, bore).map_err(|e| fail(format!("cut: {e}")))?;
        let (a, b) = (
            mass_properties(&m, holed).map_err(fail)?,
            mass_properties(&k, drilled).map_err(fail)?,
        );
        prop_assert!(close(a.volume, b.volume), "{} vs {}", a.volume, b.volume);
        prop_assert!(close(a.area, b.area), "{} vs {}", a.area, b.area);
        prop_assert_eq!(counts(&m, holed), counts(&k, drilled));
        Ok(())
    });
    if let Err(e) = result {
        panic!(
            "{e}\nreproduce with {}={}",
            prop::SEED_VAR,
            prop::seed_hex(&prop::seed())
        );
    }
}

fn p(u: f64, v: f64) -> Point2 {
    Point2::new(u, v)
}

fn square(side: f64) -> ProfileLoop {
    ProfileLoop::Path {
        start: p(0.0, 0.0),
        segments: vec![
            ProfileSegment::LineTo(p(side, 0.0)),
            ProfileSegment::LineTo(p(side, side)),
            ProfileSegment::LineTo(p(0.0, side)),
            ProfileSegment::LineTo(p(0.0, 0.0)),
        ],
    }
}

fn plate() -> Profile {
    Profile {
        plane: Frame::world(),
        outer: square(10.0),
        holes: vec![ProfileLoop::Circle {
            center: p(5.0, 5.0),
            radius: 2.0,
        }],
    }
}

/// The direction is held to the normal, the length to a positive
/// thickness, and an invalid sketch reaches the caller as its own error —
/// every refusal with the model as it was.
#[test]
fn every_refusal_is_typed_and_leaves_the_model_untouched() {
    let mut m = Model::default();
    let tol = m.precision().tolerance();
    let z = Vec3::z();
    let reason = |m: &mut Model, profile: &Profile, direction: Vec3, length: f64| match extrude(
        m, profile, direction, length,
    ) {
        Err(OpError::Degenerate { reason, .. }) => reason,
        other => panic!("{other:?}"),
    };
    let one_degree = 1f64.to_radians();
    let tilted = Vec3::new(0.0, one_degree.sin(), one_degree.cos());
    assert_eq!(
        reason(&mut m, &plate(), tilted, 1.0),
        Reason::Sweep(SweepReason::DirectionNotNormal)
    );
    assert_eq!(
        reason(&mut m, &plate(), -tilted, 1.0),
        Reason::Sweep(SweepReason::DirectionNotNormal)
    );
    assert_eq!(
        reason(&mut m, &plate(), Vec3::x(), 1.0),
        Reason::Sweep(SweepReason::DirectionNotNormal)
    );
    assert!(matches!(
        reason(&mut m, &plate(), z, 0.0),
        Reason::Input(InputReason::NotPositive { what: "length", .. })
    ));
    assert!(matches!(
        reason(&mut m, &plate(), z, -1.0),
        Reason::Input(InputReason::NotPositive { what: "length", .. })
    ));
    assert!(matches!(
        reason(&mut m, &plate(), z, f64::NAN),
        Reason::Input(InputReason::NonFinite { what: "length" })
    ));
    assert_eq!(
        reason(&mut m, &plate(), z, tol.linear / 2.0),
        Reason::Input(InputReason::ZeroThickness)
    );
    assert!(matches!(
        reason(&mut m, &plate(), Vec3::zeros(), 1.0),
        Reason::Input(InputReason::NotPositive { .. })
    ));
    assert!(matches!(
        reason(&mut m, &plate(), Vec3::new(0.0, f64::INFINITY, 1.0), 1.0),
        Reason::Input(InputReason::NonFinite { what: "direction" })
    ));

    // Every fault of the sketch, as `Profile::edges` names it.
    let sketch = |outer: ProfileLoop, holes: Vec<ProfileLoop>| Profile {
        plane: Frame::world(),
        outer,
        holes,
    };
    let path = |start: Point2, segments: Vec<ProfileSegment>| ProfileLoop::Path { start, segments };
    let circle = |u, v, radius| ProfileLoop::Circle {
        center: p(u, v),
        radius,
    };
    let line = ProfileSegment::LineTo;
    let faults = [
        (
            sketch(
                path(
                    p(0.0, 0.0),
                    vec![line(p(10.0, 0.0)), line(p(10.0, 10.0)), line(p(0.0, 1.0))],
                ),
                Vec::new(),
            ),
            ProfileError::NotClosed {
                loop_index: 0,
                gap: 1.0,
            },
        ),
        (
            sketch(path(p(0.0, 0.0), vec![line(p(0.0, 0.0))]), Vec::new()),
            ProfileError::TooFewSegments { loop_index: 0 },
        ),
        (
            sketch(
                path(
                    p(0.0, 0.0),
                    vec![line(p(1e-9, 0.0)), line(p(10.0, 10.0)), line(p(0.0, 0.0))],
                ),
                Vec::new(),
            ),
            ProfileError::ShortSegment {
                loop_index: 0,
                segment: 0,
            },
        ),
        (
            sketch(
                path(
                    p(0.0, 0.0),
                    vec![
                        line(p(10.0, 0.0)),
                        ProfileSegment::ArcTo {
                            to: p(10.0, 10.0),
                            via: p(10.0, 5.0),
                        },
                        line(p(0.0, 0.0)),
                    ],
                ),
                Vec::new(),
            ),
            ProfileError::DegenerateArc {
                loop_index: 0,
                segment: 1,
            },
        ),
        (
            sketch(
                path(p(0.0, 0.0), vec![line(p(10.0, 0.0)), line(p(0.0, 0.0))]),
                Vec::new(),
            ),
            ProfileError::ZeroArea { loop_index: 0 },
        ),
        (
            sketch(
                path(
                    p(0.0, 0.0),
                    vec![
                        line(p(10.0, 0.0)),
                        line(p(2.0, 8.0)),
                        line(p(8.0, 10.0)),
                        line(p(0.0, 0.0)),
                    ],
                ),
                Vec::new(),
            ),
            ProfileError::SelfIntersecting {
                loop_index: 0,
                segments: [1, 3],
            },
        ),
        (
            sketch(square(10.0), vec![circle(10.0, 5.0, 2.0)]),
            ProfileError::Crossing { loops: [0, 1] },
        ),
        (
            sketch(square(10.0), vec![circle(20.0, 5.0, 2.0)]),
            ProfileError::HoleOutside { hole: 1 },
        ),
        (
            sketch(
                square(10.0),
                vec![circle(5.0, 5.0, 3.0), circle(5.0, 5.0, 1.0)],
            ),
            ProfileError::NestedHoles { holes: [1, 2] },
        ),
    ];
    for (profile, expected) in faults {
        match extrude(&mut m, &profile, z, 1.0) {
            Err(OpError::Profile(e)) => assert_eq!(e, expected),
            other => panic!("{expected}: {other:?}"),
        }
    }

    // Nothing of any refusal stayed behind.
    let (body, _) = extrude(&mut m, &plate(), z, 3.0).unwrap();
    let mut fresh = Model::default();
    let (again, _) = extrude(&mut fresh, &plate(), z, 3.0).unwrap();
    assert_eq!(
        dump_text(&m, body).unwrap(),
        dump_text(&fresh, again).unwrap(),
        "the model is as it was"
    );
}

/// The same profile extruded up and down is the same solid moved by the
/// length, face for face: the counts and the volume agree, and the dump
/// differs only in where the geometry sits.
#[test]
fn an_extrude_either_way_has_the_same_numbers() {
    let mut m = Model::default();
    let (up, _) = extrude(&mut m, &plate(), Vec3::z(), 3.0).unwrap();
    let (down, _) = extrude(&mut m, &plate(), -Vec3::z(), 3.0).unwrap();
    let (a, b) = (
        mass_properties(&m, up).unwrap(),
        mass_properties(&m, down).unwrap(),
    );
    assert!(close(a.volume, b.volume) && close(a.area, b.area));
    assert!((a.centroid.z - 1.5).abs() <= REL && (b.centroid.z + 1.5).abs() <= REL);
    assert_eq!(counts(&m, up), counts(&m, down));
    assert_eq!(counts(&m, up), (10, 15, 7));
}
