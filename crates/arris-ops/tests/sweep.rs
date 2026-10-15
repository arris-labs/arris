//! `ops::sweep` along a path (ADR-0063), every span fitted: a degree-one
//! NURBS line agrees with `extrude` and an exact NURBS arc with
//! `revolve` in volume, area and centroid; a spring's volume is the
//! screw's closed form; two spans meet at the profile carried to their
//! joint, named as the second span's start; a closed path builds no caps,
//! and a hole in its profile closes into a void; and an invalid path is
//! refused by name with the model untouched. The checker runs after
//! every build (debug builds), so each `Ok` here is checker-green.

use std::collections::BTreeSet;

use arris_debug::testing::entities_of;
use arris_debug::unmetered::mass_properties;
use arris_geom::{
    FrameLaw, NurbsCurve, Path, PathError, PathSegment, Profile, ProfileLoop, ProfileSegment,
};
use arris_math::{Axis, Frame, Interval, Point2, Point3, Vec3};
use arris_ops::{Control, OpError, extrude, revolve, sweep};
use arris_topo::provenance::{Origin, PathPart, Relation, SweepPart};
use arris_topo::{Body, Model, Provenance, Role};
use core::f64::consts::{FRAC_PI_2, PI, TAU};

/// How near a fitted sweep's measures come to the exact build's: the
/// skin is within a quarter of the default tolerance, 2.5e-8, of the
/// carried profile, so a measure of a body some ten across moves by
/// about that relative to its size.
const FITTED_REL: f64 = 1e-8;

fn p(u: f64, v: f64) -> Point2 {
    Point2::new(u, v)
}

/// `(volume, area, centroid)` of `body`.
fn measures(m: &Model, body: Body) -> (f64, f64, Point3) {
    let mp = mass_properties(m, body).unwrap();
    (mp.volume, mp.area, mp.centroid)
}

/// `a` and `b` agree within `rel` of `b`'s size in volume, area and
/// centroid.
fn assert_agree(a: (f64, f64, Point3), b: (f64, f64, Point3), rel: f64, what: &str) {
    let size = b.0.cbrt();
    assert!(
        (a.0 - b.0).abs() <= rel * b.0,
        "{what}: volume {} vs {}",
        a.0,
        b.0
    );
    assert!(
        (a.1 - b.1).abs() <= rel * b.1,
        "{what}: area {} vs {}",
        a.1,
        b.1
    );
    assert!(
        (a.2 - b.2).norm() <= rel * size,
        "{what}: centroid {:?} vs {:?}",
        a.2,
        b.2
    );
}

/// The sweep's parts: every entity of the body `Generated` once from a
/// `Role::Sweep`, nothing modified or deleted, and every part naming one
/// entity.
fn parts(m: &Model, body: Body, provenance: &Provenance) -> BTreeSet<PathPart> {
    let entities = entities_of(m, body);
    let mut found = BTreeSet::new();
    for &e in &entities {
        let origins = provenance.origins(e);
        assert_eq!(origins.len(), 1, "{e}: {origins:?}");
        let (relation, origin) = origins[0];
        assert_eq!(relation, Relation::Generated, "{e} from {origin}");
        let Origin::Role(Role::Sweep(part)) = origin else {
            panic!("{e}: generated from {origin}, not a sweep part");
        };
        assert!(found.insert(part), "{part:?} names two entities");
    }
    assert_eq!(provenance.deleted().count(), 0);
    assert_eq!(provenance.outputs().len(), entities.len());
    found
}

/// A plate 40 × 30 with a hole of radius 4 on the xy plane.
fn plate() -> Profile {
    Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(40.0, 0.0)),
                ProfileSegment::LineTo(p(40.0, 30.0)),
                ProfileSegment::LineTo(p(0.0, 30.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        },
        holes: vec![ProfileLoop::Circle {
            center: p(20.0, 15.0),
            radius: 4.0,
        }],
    }
}

/// A slot x ∈ [8, 10] with a round end at x = 10, z ∈ [−1, 1], in the
/// xz plane: beside the z axis, so a turn about z is a revolve.
fn slot_beside_z() -> Profile {
    Profile {
        plane: Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Path {
            start: p(8.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(10.0, -1.0)),
                ProfileSegment::ArcTo {
                    via: p(11.0, 0.0),
                    to: p(10.0, 1.0),
                },
                ProfileSegment::LineTo(p(8.0, 1.0)),
                ProfileSegment::LineTo(p(8.0, -1.0)),
            ],
        },
        holes: Vec::new(),
    }
}

/// The disc of radius `r` (with a hole of radius `hole`, if any) centred
/// at `(R, 0, 0)` in the xz plane.
fn disc_at(big_r: f64, r: f64, hole: Option<f64>) -> Profile {
    Profile {
        plane: Frame::new(Point3::new(big_r, 0.0, 0.0), -Vec3::y(), Vec3::x()).unwrap(),
        outer: ProfileLoop::Circle {
            center: p(0.0, 0.0),
            radius: r,
        },
        holes: hole
            .map(|radius| ProfileLoop::Circle {
                center: p(0.0, 0.0),
                radius,
            })
            .into_iter()
            .collect(),
    }
}

/// The circle of radius `r` about z in the xy plane as four quarter arcs
/// from `(r, 0, 0)`, counter-clockwise: a closed planar path.
fn ring_path(r: f64) -> Path {
    let h = r * 0.5f64.sqrt();
    Path {
        start: Point3::new(r, 0.0, 0.0),
        segments: vec![
            PathSegment::ArcTo {
                via: Point3::new(h, h, 0.0),
                to: Point3::new(0.0, r, 0.0),
            },
            PathSegment::ArcTo {
                via: Point3::new(-h, h, 0.0),
                to: Point3::new(-r, 0.0, 0.0),
            },
            PathSegment::ArcTo {
                via: Point3::new(-h, -h, 0.0),
                to: Point3::new(0.0, -r, 0.0),
            },
            PathSegment::ArcTo {
                via: Point3::new(h, -h, 0.0),
                to: Point3::new(r, 0.0, 0.0),
            },
        ],
    }
}

#[test]
fn a_degree_one_nurbs_line_sweeps_the_extrusion() {
    let line = NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Point3::origin(), Point3::new(0.0, 0.0, 10.0)],
        vec![1.0, 1.0],
    )
    .unwrap();
    let path = Path {
        start: Point3::origin(),
        segments: vec![PathSegment::Nurbs(line)],
    };
    let mut m = Model::default();
    let (swept, _) = sweep(
        &mut m,
        &plate(),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (extruded, _) = extrude(&mut m, &plate(), Vec3::z(), 10.0, &Control::NONE).unwrap();
    assert_agree(
        measures(&m, swept),
        measures(&m, extruded),
        FITTED_REL,
        "line",
    );
    assert_eq!(m.faces(swept).unwrap().len(), 7);
    assert_eq!(m.edges(swept).unwrap().len(), 15);
}

#[test]
fn an_exact_nurbs_arc_sweeps_the_revolve() {
    // A quarter turn about z from (9, 0, 0), leaving along +y: the
    // profile's plane holds the axis, so the sweep is a revolve.
    let arc =
        NurbsCurve::circle(&Frame::world(), 9.0, Interval::new(0.0, FRAC_PI_2).unwrap()).unwrap();
    let path = Path {
        start: Point3::new(9.0, 0.0, 0.0),
        segments: vec![PathSegment::Nurbs(arc)],
    };
    let mut m = Model::default();
    let (swept, _) = sweep(
        &mut m,
        &slot_beside_z(),
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (revolved, _) = revolve(
        &mut m,
        &slot_beside_z(),
        Axis::z_at(Point3::origin()),
        FRAC_PI_2,
        &Control::NONE,
    )
    .unwrap();
    assert_agree(
        measures(&m, swept),
        measures(&m, revolved),
        FITTED_REL,
        "arc",
    );
}

#[test]
fn a_spring_has_the_screws_volume() {
    // A wire of radius 1 five turns round a helix of radius 10 and pitch
    // 6 about z, the profile in the axis' plane: a screw motion, so the
    // volume is the turn angle times the disc's first moment about the
    // axis, 2π · 5 · π · 10.
    let path = Path {
        start: Point3::new(10.0, 0.0, 0.0),
        segments: vec![PathSegment::Helix {
            axis: Axis::z_at(Point3::origin()),
            pitch: 6.0,
            angle: 5.0 * TAU,
        }],
    };
    let mut m = Model::default();
    let (body, provenance) = sweep(
        &mut m,
        &disc_at(10.0, 1.0, None),
        &path,
        FrameLaw::FixedBinormal(Vec3::z_axis()),
        &Control::NONE,
    )
    .unwrap();
    let (volume, _, centroid) = measures(&m, body);
    let exact = TAU * 5.0 * PI * 10.0;
    assert!(
        (volume - exact).abs() <= 1e-9 * exact,
        "{volume} vs {exact}"
    );
    assert!((centroid - Point3::new(0.0, 0.0, 15.0)).norm() <= 1e-8);
    assert_eq!(
        (
            m.faces(body).unwrap().len(),
            m.edges(body).unwrap().len(),
            m.vertices(body).unwrap().len()
        ),
        (3, 3, 2),
        "two caps, the wire's one face, its seam rail"
    );
    let found = parts(&m, body, &provenance);
    let rise = PathPart {
        span: 0,
        part: SweepPart::Rise {
            loop_index: 0,
            vertex: 0,
        },
    };
    assert!(found.contains(&rise), "{found:?}");
}

#[test]
fn two_spans_meet_at_the_profile_carried_to_their_joint() {
    let square = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(-1.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(1.0, -1.0)),
                ProfileSegment::LineTo(p(1.0, 1.0)),
                ProfileSegment::LineTo(p(-1.0, 1.0)),
                ProfileSegment::LineTo(p(-1.0, -1.0)),
            ],
        },
        holes: Vec::new(),
    };
    let path = Path {
        start: Point3::origin(),
        segments: vec![
            PathSegment::LineTo(Point3::new(0.0, 0.0, 4.0)),
            PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0)),
        ],
    };
    let mut m = Model::default();
    let (body, provenance) = sweep(
        &mut m,
        &square,
        &path,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (volume, _, _) = measures(&m, body);
    assert!((volume - 40.0).abs() <= FITTED_REL * 40.0, "{volume}");
    // Three rings of four vertices and four edges, two spans of four
    // rails and four sides, two caps.
    assert_eq!(m.vertices(body).unwrap().len(), 12);
    assert_eq!(m.edges(body).unwrap().len(), 20);
    assert_eq!(m.faces(body).unwrap().len(), 10);
    let found = parts(&m, body, &provenance);
    let at = |span, part| PathPart { span, part };
    let joint = at(
        1,
        SweepPart::StartEdge {
            loop_index: 0,
            segment: 2,
        },
    );
    let end = at(
        1,
        SweepPart::EndEdge {
            loop_index: 0,
            segment: 2,
        },
    );
    for part in [
        joint,
        end,
        at(0, SweepPart::StartCap),
        at(1, SweepPart::EndCap),
        at(
            1,
            SweepPart::Side {
                loop_index: 0,
                segment: 3,
            },
        ),
    ] {
        assert!(found.contains(&part), "{part:?} in {found:?}");
    }
    assert!(!found.contains(&at(0, SweepPart::EndCap)));
    assert!(!found.contains(&at(
        0,
        SweepPart::EndEdge {
            loop_index: 0,
            segment: 2
        }
    )));
}

#[test]
fn a_closed_path_builds_no_caps_and_a_hole_closes_into_a_void() {
    let mut m = Model::default();
    let (body, provenance) = sweep(
        &mut m,
        &disc_at(10.0, 2.0, Some(1.0)),
        &ring_path(10.0),
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap();
    let (volume, _, _) = measures(&m, body);
    // Pappus: the annulus's area times the circle its centre runs.
    let exact = TAU * 10.0 * PI * (4.0 - 1.0);
    assert!(
        (volume - exact).abs() <= FITTED_REL * exact,
        "{volume} vs {exact}"
    );
    assert_eq!(m.shells(body).unwrap().len(), 2, "the tube and its bore");
    assert_eq!(m.faces(body).unwrap().len(), 8, "four spans of two loops");
    let found = parts(&m, body, &provenance);
    assert!(found.iter().all(|p| !matches!(
        p.part,
        SweepPart::StartCap | SweepPart::EndCap | SweepPart::EndEdge { .. }
    )));
    assert!(found.contains(&PathPart {
        span: 0,
        part: SweepPart::Cavity {
            loop_index: 1,
            segment: 0
        }
    }));
}

#[test]
fn an_invalid_path_is_refused_by_name_with_the_model_untouched() {
    let bent = Path {
        start: Point3::origin(),
        segments: vec![
            PathSegment::LineTo(Point3::new(0.0, 0.0, 10.0)),
            PathSegment::LineTo(Point3::new(10.0, 0.0, 10.0)),
        ],
    };
    let mut m = Model::default();
    let before = arris_io::native::to_bytes(&m).unwrap();
    let err = sweep(
        &mut m,
        &plate(),
        &bent,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap_err();
    assert!(
        matches!(err, OpError::Path(PathError::Corner { segment: 1, .. })),
        "{err}"
    );
    assert_eq!(arris_io::native::to_bytes(&m).unwrap(), before);
    let empty = Path {
        start: Point3::origin(),
        segments: Vec::new(),
    };
    let err = sweep(
        &mut m,
        &plate(),
        &empty,
        FrameLaw::RotationMinimizing,
        &Control::NONE,
    )
    .unwrap_err();
    assert_eq!(err, OpError::Path(PathError::Empty));
}
