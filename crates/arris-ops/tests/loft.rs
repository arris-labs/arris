//! `ops::loft`, ruled and smooth (ADR-0065): a square frustum and a cone frustum have
//! the frustum's closed-form volume, every entity of a loft is `Generated`
//! once from a `Role::Loft` naming its span and the part of the sketch,
//! sections of unequal segment counts loft through the arc-length cut, a
//! closed loft builds no caps, a bad set of sections is refused with the
//! model untouched, and the dump is the same on two runs; a smooth loft is
//! one span through every section, three sections on one frustum loft to
//! that frustum, two sections are the ruled loft, and more than one
//! polynomial takes are refused. The checker runs
//! after every build (debug builds), so each `Ok` here is checker-green.

use std::collections::BTreeSet;

use arris_debug::dump_text;
use arris_debug::testing::entities_of;
use arris_debug::unmetered::mass_properties;
use arris_geom::{LoftError, Profile, ProfileError, ProfileLoop, ProfileSegment, SurfaceKind};
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

fn smooth(
    m: &mut Model,
    sections: &[Profile],
    closed: bool,
) -> Result<(Body, Provenance), OpError> {
    loft(m, sections, LoftKind::Smooth, closed, &Control::NONE)
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
fn a_closed_loft_of_two_sections_is_refused() {
    let mut m = Model::default();
    let err = ruled(&mut m, &[square(0.0, 2.0), square(3.0, 1.0)], true).unwrap_err();
    assert!(matches!(
        err,
        OpError::Loft(LoftError::TooFewSections {
            given: 2,
            needed: 3
        })
    ));
}

#[test]
fn sections_with_unequal_loop_counts_are_refused_naming_the_section() {
    let mut m = Model::default();
    let mut holed = square(3.0, 2.0);
    holed.holes.push(ProfileLoop::Circle {
        center: Point2::origin(),
        radius: 0.5,
    });
    let err = ruled(&mut m, &[square(0.0, 2.0), holed], false).unwrap_err();
    assert!(matches!(
        err,
        OpError::Loft(LoftError::LoopCount {
            section: 1,
            expected: 1,
            found: 2
        })
    ));
}

#[test]
fn consecutive_sections_the_same_are_refused_naming_both() {
    let mut m = Model::default();
    let sections = [square(0.0, 2.0), square(3.0, 1.0), square(3.0, 1.0)];
    let err = ruled(&mut m, &sections, false).unwrap_err();
    assert!(matches!(
        err,
        OpError::Loft(LoftError::Coincident {
            section: 2,
            previous: 1
        })
    ));
}

#[test]
fn a_loop_that_collapses_is_refused_as_the_profile_naming_the_section() {
    let mut m = Model::default();
    let collapsed = Profile {
        plane: at(3.0),
        outer: ProfileLoop::Path {
            start: Point2::new(1.0, 1.0),
            segments: vec![
                ProfileSegment::LineTo(Point2::new(1.0, 1.0)),
                ProfileSegment::LineTo(Point2::new(1.0, 1.0)),
                ProfileSegment::LineTo(Point2::new(1.0, 1.0)),
            ],
        },
        holes: Vec::new(),
    };
    let err = ruled(&mut m, &[square(0.0, 2.0), collapsed], false).unwrap_err();
    let OpError::Loft(LoftError::Profile { section: 1, source }) = err else {
        panic!("expected the profile's refusal naming section 1, found {err:?}");
    };
    assert!(
        matches!(
            source,
            ProfileError::ShortSegment { .. } | ProfileError::TooFewSegments { .. }
        ),
        "{source:?}"
    );
    // A circle of no radius is the same refusal.
    let err = ruled(&mut m, &[square(0.0, 2.0), circle(3.0, 0.0)], false).unwrap_err();
    assert!(matches!(
        err,
        OpError::Loft(LoftError::Profile { section: 1, .. })
    ));
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

/// The kinds of the body's faces, sorted.
fn kinds(m: &Model, body: Body) -> Vec<SurfaceKind> {
    let mut kinds: Vec<SurfaceKind> = m
        .faces(body)
        .unwrap()
        .iter()
        .map(|f| m.surface(m.face(f.id).unwrap().surface()).unwrap().kind())
        .collect();
    kinds.sort();
    kinds
}

#[test]
fn coplanar_lines_give_planes_and_coaxial_circles_a_cone_or_a_cylinder() {
    let mut m = Model::default();
    let (frustum, _) = ruled(&mut m, &[square(0.0, 2.0), square(3.0, 1.0)], false).unwrap();
    assert_eq!(kinds(&m, frustum), vec![SurfaceKind::Plane; 6]);
    let (cone, _) = ruled(&mut m, &[circle(0.0, 3.0), circle(4.0, 1.0)], false).unwrap();
    assert_eq!(
        kinds(&m, cone),
        vec![SurfaceKind::Plane, SurfaceKind::Plane, SurfaceKind::Cone]
    );
    let (tube, _) = ruled(&mut m, &[circle(0.0, 3.0), circle(4.0, 3.0)], false).unwrap();
    assert_eq!(
        kinds(&m, tube),
        vec![
            SurfaceKind::Plane,
            SurfaceKind::Plane,
            SurfaceKind::Cylinder
        ]
    );
    // The cone and the cylinder keep the closed forms.
    let want_tube = PI * 9.0 * 4.0;
    let mp = mass_properties(&m, tube).unwrap();
    assert!(
        (mp.volume - want_tube).abs() <= 1e-12 * want_tube,
        "{}",
        mp.volume
    );
}

#[test]
fn a_circle_against_a_square_and_a_twisted_square_stay_nurbs() {
    let mut m = Model::default();
    let (body, _) = ruled(&mut m, &[circle(0.0, 4.0), square(6.0, 4.0)], false).unwrap();
    assert_eq!(
        kinds(&m, body),
        vec![
            SurfaceKind::Plane,
            SurfaceKind::Plane,
            SurfaceKind::Nurbs,
            SurfaceKind::Nurbs,
            SurfaceKind::Nurbs,
            SurfaceKind::Nurbs
        ]
    );
}

#[test]
fn three_sections_on_one_frustum_loft_smooth_to_that_frustum() {
    // Equal steps of side and height: the chord parameters are 0, 1/2, 1
    // and the quadratic through collinear rows is the straight line.
    let mut m = Model::default();
    let sections = [square(0.0, 2.0), square(2.0, 1.5), square(4.0, 1.0)];
    let (body, provenance) = smooth(&mut m, &sections, false).unwrap();
    let want = 4.0 / 3.0 * (16.0 + 4.0 + 8.0);
    let mp = mass_properties(&m, body).unwrap();
    assert!((mp.volume - want).abs() <= 1e-12 * want, "{}", mp.volume);
    // One span: four sides through all three sections, rings only at the
    // ends, every part named once from span 0.
    let found = parts(&m, body, &provenance);
    assert!(found.iter().all(|p| p.span == 0), "{found:?}");
    assert_eq!(m.faces(body).unwrap().len(), 6);
    assert_eq!(m.edges(body).unwrap().len(), 12);
    assert_eq!(m.vertices(body).unwrap().len(), 8);
    assert_eq!(
        kinds(&m, body),
        [vec![SurfaceKind::Plane; 2], vec![SurfaceKind::Nurbs; 4]].concat()
    );
}

#[test]
fn a_smooth_loft_passes_through_its_middle_section() {
    // A circle between two squares: the side faces meet z = 3 on the
    // circle, wherever the surfaces' own parameters put it.
    let mut m = Model::default();
    let sections = [square(0.0, 2.0), circle(3.0, 1.5), square(7.0, 2.0)];
    let (body, _) = smooth(&mut m, &sections, false).unwrap();
    for f in m.faces(body).unwrap() {
        let surface = m.surface(m.face(f.id).unwrap().surface()).unwrap();
        if surface.kind() != SurfaceKind::Nurbs {
            continue;
        }
        let [du, dv] = surface.domain();
        // The row of the middle section: where z crosses 3 along v.
        for i in 0..=8 {
            let u = du.lerp(i as f64 / 8.0);
            let (mut lo, mut hi) = (dv.lo(), dv.hi());
            for _ in 0..80 {
                let mid = 0.5 * (lo + hi);
                if surface.eval(u, mid).point.z < 3.0 {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let q = surface.eval(u, lo).point;
            assert!(
                (q.coords.xy().norm() - 1.5).abs() <= 1e-9,
                "{:?} at u = {u}",
                q
            );
        }
    }
}

#[test]
fn two_sections_loft_smooth_as_they_loft_ruled() {
    let sections = [circle(0.0, 3.0), circle(4.0, 1.0)];
    let mut a = Model::default();
    let (ruled_body, _) = ruled(&mut a, &sections, false).unwrap();
    let mut b = Model::default();
    let (smooth_body, _) = smooth(&mut b, &sections, false).unwrap();
    assert_eq!(
        dump_text(&a, ruled_body).unwrap(),
        dump_text(&b, smooth_body).unwrap()
    );
}

#[test]
fn a_smooth_loft_through_more_sections_than_one_polynomial_takes_is_refused() {
    let tower: Vec<Profile> = (0..10)
        .map(|k| square(k as f64, 2.0 - 0.1 * k as f64))
        .collect();
    let mut m = Model::default();
    let err = smooth(&mut m, &tower, false).unwrap_err();
    assert!(matches!(
        err,
        OpError::Loft(LoftError::TooManySections { given: 10, max: 9 })
    ));
    // Closed, the first counts twice: nine sections are ten rows.
    let err = smooth(&mut m, &tower[..9], true).unwrap_err();
    assert!(matches!(
        err,
        OpError::Loft(LoftError::TooManySections { given: 10, max: 9 })
    ));
    // Ruled takes any number.
    assert!(ruled(&mut m, &tower, false).is_ok());
}
