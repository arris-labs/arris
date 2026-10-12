//! `ops::planar_face` (`docs/ARCHITECTURE.md` §Operations): a thousand
//! general profiles — lines, outward and inward arcs, circular and
//! polygonal holes, in random poses — made a one-face sheet: the checker
//! at `Full` with nothing unchecked, the sheet's area and centroid the
//! profile's own in its plane (an independent path from the surface
//! integral over the B-Rep), the face on the profile's plane with its
//! front on the plane's normal, one `Generated` per entity with every part
//! of the sketch present, the dump identical on two runs; and the refusals
//! typed with the model untouched.

use std::collections::BTreeSet;

use arris_check::{Level, check};
use arris_debug::testing::{close, fail};
use arris_debug::unmetered::{planar_face, surface_properties};
use arris_debug::{dump_text, prop, prop_shards};
use arris_geom::{Profile, ProfileError, ProfileLoop, ProfileSegment, Surface};
use arris_math::{Frame, Point2};
use arris_ops::OpError;
use arris_topo::entity::BodyKind;
use arris_topo::provenance::SweepPart;
use arris_topo::{Model, Orientation, Role};
use proptest::prelude::*;

/// The parts a planar face of `profile` makes, from the sketch alone: the
/// face, the shell and the body, and for every segment an edge and the
/// vertex it starts at.
fn expected_parts(profile: &Profile, m: &Model) -> BTreeSet<SweepPart> {
    let tol = m.precision().tolerance();
    let mut parts = BTreeSet::from([SweepPart::StartCap, SweepPart::Shell, SweepPart::Body]);
    let loops = profile.edges(tol).expect("a valid profile");
    for edges in &loops {
        for edge in edges {
            parts.insert(SweepPart::StartEdge {
                loop_index: edge.loop_index,
                segment: edge.segment,
            });
            parts.insert(SweepPart::StartVertex {
                loop_index: edge.loop_index,
                vertex: edge.segment,
            });
        }
    }
    parts
}

fn is_a_planar_face_of(profile: &Profile) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let tol = m.precision().tolerance();
    let (sheet, provenance) = planar_face(&mut m, profile).map_err(fail)?;
    prop_assert_eq!(m.body(sheet.id).map_err(fail)?.kind(), BodyKind::Sheet);
    let report = check(&m, sheet, Level::Full);
    prop_assert!(
        report.is_ok() && report.unchecked().is_empty(),
        "not clean at Full\n{}\n{}",
        report,
        dump_text(&m, sheet).map_err(fail)?
    );

    // Area and centroid: the profile's, in its plane.
    let (area, centroid) = profile.area_and_centroid(tol).map_err(fail)?;
    let props = surface_properties(&m, sheet).map_err(fail)?;
    prop_assert!(
        close(props.area, area),
        "area {} vs the profile's {}",
        props.area,
        area
    );
    let world = profile.to_world(centroid);
    prop_assert!(
        (props.centroid - world).norm() <= 1e-9 * (1.0 + world.coords.norm()),
        "centroid {} vs the profile's {}",
        props.centroid,
        world
    );

    // One face, on the profile's own plane, front along its normal.
    let faces = m.faces(sheet).map_err(fail)?;
    prop_assert_eq!(faces.len(), 1);
    prop_assert_eq!(faces[0].orientation, Orientation::Forward);
    let surface = m
        .surface(m.face(faces[0].id).map_err(fail)?.surface())
        .map_err(fail)?;
    prop_assert_eq!(
        surface,
        &Surface::Plane {
            frame: profile.plane
        }
    );

    // Provenance: one Generated per entity, every part of the sketch.
    let recorded =
        arris_debug::testing::recorded_parts(&m, sheet, &provenance, |role| match role {
            Role::PlanarFace(part) => Some(part),
            _ => None,
        })?;
    prop_assert_eq!(recorded, expected_parts(profile, &m));

    // Deterministic.
    let dump = dump_text(&m, sheet).map_err(fail)?;
    let mut again = Model::default();
    let (twice, _) = planar_face(&mut again, profile).map_err(fail)?;
    prop_assert_eq!(dump_text(&again, twice).map_err(fail)?, dump);
    Ok(())
}

prop_shards! {
    /// A convex polygon with outward arcs, arcs bulging inward and holes,
    /// made a sheet: clean at `Full`, the profile's own area and centroid,
    /// the face on the profile's plane, complete provenance, a
    /// deterministic dump.
    general_profiles_make_a_planar_face [shard_0 shard_1]
        (sweep) = prop::profile::general() => {
            is_a_planar_face_of(&sweep.profile)
        }
}

fn p(u: f64, v: f64) -> Point2 {
    Point2::new(u, v)
}

/// A refused profile is `OpError::Profile` with the model as it was.
#[test]
fn a_refused_profile_is_typed_and_leaves_the_model_untouched() {
    let square = |side: f64| ProfileLoop::Path {
        start: p(0.0, 0.0),
        segments: vec![
            ProfileSegment::LineTo(p(side, 0.0)),
            ProfileSegment::LineTo(p(side, side)),
            ProfileSegment::LineTo(p(0.0, side)),
            ProfileSegment::LineTo(p(0.0, 0.0)),
        ],
    };
    let plate = || Profile {
        plane: Frame::world(),
        outer: square(10.0),
        holes: vec![],
    };
    let outside = Profile {
        holes: vec![ProfileLoop::Circle {
            center: p(20.0, 5.0),
            radius: 2.0,
        }],
        ..plate()
    };
    let mut m = Model::default();
    match planar_face(&mut m, &outside) {
        Err(OpError::Profile(e)) => assert_eq!(e, ProfileError::HoleOutside { hole: 1 }),
        other => panic!("{other:?}"),
    }
    // Nothing of the refusal stayed behind.
    let (sheet, _) = planar_face(&mut m, &plate()).unwrap();
    let mut fresh = Model::default();
    let (again, _) = planar_face(&mut fresh, &plate()).unwrap();
    assert_eq!(
        dump_text(&m, sheet).unwrap(),
        dump_text(&fresh, again).unwrap(),
        "the model is as it was"
    );
}
