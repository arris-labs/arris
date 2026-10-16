//! `ops::loft`, ruled (ADR-0065): a square frustum and a cone frustum have
//! the frustum's closed-form volume, every entity of a loft is `Generated`
//! once from a `Role::Loft` naming its span and the part of the sketch,
//! sections of unequal segment counts loft through the arc-length cut, a
//! closed loft builds no caps, a bad set of sections is refused with the
//! model untouched, and the dump is the same on two runs. The checker runs
//! after every build (debug builds), so each `Ok` here is checker-green.

use std::collections::BTreeSet;

use arris_debug::dump_text;
use arris_debug::testing::entities_of;
use arris_debug::unmetered::mass_properties;
use arris_geom::{LoftError, Profile, ProfileLoop, ProfileSegment};
use arris_math::{Frame, Point2, Point3, Vec3, real};
use arris_ops::{Control, LoftKind, OpError, loft};
use arris_topo::provenance::{Origin, PathPart, Relation, SweepPart};
use arris_topo::{Body, Model, Provenance, Role};
use core::f64::consts::PI;

fn at(z: f64) -> Frame {
    Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).unwrap()
}

fn square(z: f64, h: f64) -> Profile {
    let p = |u, v| Point2::new(u, v);
    Profile {
        plane: at(z),
        outer: ProfileLoop::Path {
            start: p(h, h),
            segments: vec![
                ProfileSegment::LineTo(p(-h, h)),
                ProfileSegment::LineTo(p(-h, -h)),
                ProfileSegment::LineTo(p(h, -h)),
                ProfileSegment::LineTo(p(h, h)),
            ],
        },
        holes: Vec::new(),
    }
}

fn circle(z: f64, radius: f64) -> Profile {
    Profile {
        plane: at(z),
        outer: ProfileLoop::Circle {
            center: Point2::origin(),
            radius,
        },
        holes: Vec::new(),
    }
}

fn ruled(m: &mut Model, sections: &[Profile], closed: bool) -> Result<(Body, Provenance), OpError> {
    loft(m, sections, LoftKind::Ruled, closed, &Control::NONE)
}

/// The loft's parts: every entity of the body `Generated` once from a
/// `Role::Loft`, nothing modified or deleted, every part naming one entity.
fn parts(m: &Model, body: Body, provenance: &Provenance) -> BTreeSet<PathPart> {
    let entities = entities_of(m, body);
    let mut found = BTreeSet::new();
    for &e in &entities {
        let origins = provenance.origins(e);
        assert_eq!(origins.len(), 1, "{e}: {origins:?}");
        let (relation, origin) = origins[0];
        assert_eq!(relation, Relation::Generated, "{e} from {origin}");
        let Origin::Role(Role::Loft(part)) = origin else {
            panic!("{e}: generated from {origin}, not a loft part");
        };
        assert!(found.insert(part), "{part:?} names two entities");
    }
    assert_eq!(provenance.deleted().count(), 0);
    assert_eq!(provenance.outputs().len(), entities.len());
    found
}

#[test]
fn a_square_frustum_has_the_frustums_volume() {
    let mut m = Model::default();
    let (body, _) = ruled(&mut m, &[square(0.0, 2.0), square(3.0, 1.0)], false).unwrap();
    let (a1, a2) = (16.0, 4.0);
    let want = 3.0 / 3.0 * (a1 + a2 + f64::sqrt(a1 * a2));
    let mp = mass_properties(&m, body).unwrap();
    assert!((mp.volume - want).abs() <= 1e-12 * want, "{}", mp.volume);
    assert_eq!(m.faces(body).unwrap().len(), 6);
    assert_eq!(m.edges(body).unwrap().len(), 12);
    assert_eq!(m.vertices(body).unwrap().len(), 8);
}

#[test]
fn coaxial_circles_loft_the_cone_frustum() {
    let mut m = Model::default();
    let (body, _) = ruled(&mut m, &[circle(0.0, 3.0), circle(4.0, 1.0)], false).unwrap();
    let want = PI * 4.0 / 3.0 * (9.0 + 3.0 + 1.0);
    let mp = mass_properties(&m, body).unwrap();
    assert!((mp.volume - want).abs() <= 1e-9 * want, "{}", mp.volume);
}

#[test]
fn every_entity_is_generated_from_its_span_and_part() {
    let mut m = Model::default();
    let sections = [square(0.0, 2.0), square(3.0, 1.0), square(5.0, 2.0)];
    let (body, provenance) = ruled(&mut m, &sections, false).unwrap();
    let found = parts(&m, body, &provenance);
    let role = |span, part| PathPart { span, part };
    for span in 0..2 {
        for segment in 0..4 {
            assert!(found.contains(&role(
                span,
                SweepPart::Side {
                    loop_index: 0,
                    segment
                }
            )));
            assert!(found.contains(&role(
                span,
                SweepPart::Rise {
                    loop_index: 0,
                    vertex: segment
                }
            )));
            assert!(found.contains(&role(
                span,
                SweepPart::StartEdge {
                    loop_index: 0,
                    segment
                }
            )));
        }
    }
    assert!(found.contains(&role(0, SweepPart::StartCap)));
    assert!(found.contains(&role(1, SweepPart::EndCap)));
    assert!(found.contains(&role(
        1,
        SweepPart::EndEdge {
            loop_index: 0,
            segment: 2
        }
    )));
    assert!(!found.contains(&role(0, SweepPart::EndCap)));
    // 3 rings of 4 vertices, 3 rings of 4 edges and 2 spans of 4 rails;
    // the caps and 8 sides, the shell and the body.
    assert_eq!(m.faces(body).unwrap().len(), 10);
    assert_eq!(m.edges(body).unwrap().len(), 20);
    assert_eq!(m.vertices(body).unwrap().len(), 12);
}

#[test]
fn a_circle_against_a_square_is_cut_where_the_squares_corners_are() {
    let mut m = Model::default();
    let (body, provenance) = ruled(&mut m, &[circle(0.0, 4.0), square(6.0, 4.0)], false).unwrap();
    let found = parts(&m, body, &provenance);
    assert_eq!(m.faces(body).unwrap().len(), 6, "four sides and two caps");
    assert!(found.contains(&PathPart {
        span: 0,
        part: SweepPart::Side {
            loop_index: 0,
            segment: 3
        }
    }));
    let mp = mass_properties(&m, body).unwrap();
    // Between the cone of the circle and the pyramid frustum of the
    // square that holds it, 6 high.
    let (lo, hi) = (PI * 16.0 * 6.0 / 3.0 * 3.0, 64.0 * 6.0);
    assert!(mp.volume > lo * 0.99 && mp.volume < hi, "{}", mp.volume);
}

#[test]
fn a_closed_loft_builds_no_caps() {
    let ring = |angle: f64| -> Profile {
        let (s, c) = real::sin_cos(angle);
        // The section's x is radial and its y is z: the meridian disc, its
        // normal against the ring's direction.
        Profile {
            plane: Frame::new(
                Point3::new(8.0 * c, 8.0 * s, 0.0),
                Vec3::new(s, -c, 0.0),
                Vec3::new(c, s, 0.0),
            )
            .unwrap(),
            outer: ProfileLoop::Circle {
                center: Point2::origin(),
                radius: 1.5,
            },
            holes: Vec::new(),
        }
    };
    let sections: Vec<Profile> = (0..4).map(|k| ring(k as f64 * PI / 2.0)).collect();
    let mut m = Model::default();
    let (body, provenance) = ruled(&mut m, &sections, true).unwrap();
    let found = parts(&m, body, &provenance);
    assert_eq!(m.faces(body).unwrap().len(), 4, "a side per span, no caps");
    assert!(
        !found
            .iter()
            .any(|p| matches!(p.part, SweepPart::StartCap | SweepPart::EndCap))
    );
    let mp = mass_properties(&m, body).unwrap();
    assert!(mp.volume > 0.0);
    assert!(mp.centroid.coords.norm() < 1e-9, "{:?}", mp.centroid);
}

#[test]
fn too_few_sections_are_refused_and_the_model_is_untouched() {
    let mut m = Model::default();
    let err = ruled(&mut m, &[square(0.0, 1.0)], false).unwrap_err();
    assert!(matches!(
        err,
        OpError::Loft(LoftError::TooFewSections {
            given: 1,
            needed: 2
        })
    ));
    // The failed call consumed no id: the next loft is the first body.
    let (body, _) = ruled(&mut m, &[square(0.0, 2.0), square(3.0, 1.0)], false).unwrap();
    let mut fresh = Model::default();
    let (same, _) = ruled(&mut fresh, &[square(0.0, 2.0), square(3.0, 1.0)], false).unwrap();
    assert_eq!(
        dump_text(&m, body).unwrap(),
        dump_text(&fresh, same).unwrap()
    );
}

#[test]
fn a_loft_is_the_same_on_two_runs() {
    let build = || {
        let mut m = Model::default();
        let (body, _) = ruled(&mut m, &[circle(0.0, 4.0), square(6.0, 4.0)], false).unwrap();
        dump_text(&m, body).unwrap()
    };
    assert_eq!(build(), build());
}
