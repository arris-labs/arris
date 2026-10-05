//! The Euler operators against the checker and the oracle (ADR-0002):
//! the cylinder built by
//! `mvfs → mef → mev → mef` is clean at `Full` and matches
//! `primitive/cylinder`; `sample::frame`, built with `mef`/`mev` for the
//! window's rim and walls and `kfmrh` to open the floor, is clean at
//! `Full` with genus 1 and two two-loop faces and matches
//! `boolean/frame-cut`; both dump identically across two builds.

use core::f64::consts::TAU;

use arris_check::{Level, check};
use arris_debug::{dump_text, oracle, sample};
use arris_geom::{Curve, Curve2, Surface};
use arris_io::step;
use arris_math::{Frame, Frame2, Interval, Point2, Point3, Vec2, Vec3};
use arris_topo::builder::{Builder, Position, Seed, Split, Strut};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::{Body, Model, Orientation};

/// The cylinder of `docs/DATA-MODEL.md` §Seams through the builder.
fn cylinder(m: &mut Model, r: f64, h: f64) -> Body {
    let base = Frame::world();
    let top = base.with_origin(Point3::new(0.0, 0.0, h));
    let wall = m.add_surface(Surface::Cylinder {
        frame: base,
        radius: r,
    });
    let bottom = m.add_surface(Surface::Plane { frame: base });
    let top_plane = m.add_surface(Surface::Plane { frame: top });
    let uv_line = |m: &mut Model, u: f64, v: f64, along_u: bool| {
        m.add_curve2(Curve2::Line {
            origin: Point2::new(u, v),
            direction: if along_u {
                Vec2::x_axis()
            } else {
                Vec2::y_axis()
            },
        })
    };
    let cap = |m: &mut Model| {
        m.add_curve2(Curve2::Circle {
            frame: Frame2::identity(),
            radius: r,
        })
    };
    let mut b = Builder::new(m.precision().default_tolerance);
    let (v0, f_bottom) = b
        .mvfs(Seed {
            point: Point3::new(r, 0.0, 0.0),
            surface: bottom,
            orientation: Orientation::Reversed,
        })
        .unwrap();
    let at = Position::new(f_bottom, 0, 0);
    let circle = m.add_curve(Curve::Circle {
        frame: base,
        radius: r,
    });
    let (pw, pc) = (uv_line(m, 0.0, 0.0, true), cap(m));
    let (_, f_wall) = b
        .mef(
            at,
            at,
            Split {
                geometry: EdgeGeometry::Curve {
                    curve: circle,
                    range: Interval::TURN,
                },
                surface: wall,
                orientation: Orientation::Forward,
                pcurves: [Some(pw), Some(pc)],
            },
        )
        .unwrap();
    let seam = m.add_curve(Curve::Line {
        origin: Point3::new(r, 0.0, 0.0),
        direction: Vec3::z_axis(),
    });
    let (up, down) = (uv_line(m, TAU, 0.0, false), uv_line(m, 0.0, 0.0, false));
    let (v1, _) = b
        .mev(
            b.find_position(f_wall, 0, v0).unwrap(),
            Strut {
                point: Point3::new(r, 0.0, h),
                geometry: EdgeGeometry::Curve {
                    curve: seam,
                    range: Interval::new(0.0, h).unwrap(),
                },
                pcurves: [Some(up), Some(down)],
            },
        )
        .unwrap();
    let top_circle = m.add_curve(Curve::Circle {
        frame: top,
        radius: r,
    });
    let (pw, pc) = (uv_line(m, 0.0, h, true), cap(m));
    let at = b.find_position(f_wall, 0, v1).unwrap();
    b.mef(
        at,
        at,
        Split {
            geometry: EdgeGeometry::Curve {
                curve: top_circle,
                range: Interval::TURN,
            },
            surface: top_plane,
            orientation: Orientation::Forward,
            pcurves: [Some(pc), Some(pw)],
        },
    )
    .unwrap();
    b.finish(m, BodyKind::Solid).unwrap().body
}

fn frame(m: &mut Model) -> Body {
    sample::frame(
        m,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(10.0, 10.0),
        Point2::new(30.0, 20.0),
    )
    .unwrap()
}

#[test]
fn the_built_cylinder_is_clean_at_full_and_matches_the_oracle() {
    let mut m = Model::default();
    let body = cylinder(&mut m, 4.0, 12.0);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}\n{}", dump_text(&m, body).unwrap());
    assert!(report.unchecked().is_empty(), "{report}");
    assert_eq!(report.euler().unwrap().to_string(), "2/3/3/3/1 g0 = 0");
    let text = step::write(&m, &[body]).unwrap();
    oracle::compare("primitive/cylinder", &text, None, "built-cylinder").unwrap();
    let mut other = Model::default();
    let twin = cylinder(&mut other, 4.0, 12.0);
    assert_eq!(
        dump_text(&m, body).unwrap(),
        dump_text(&other, twin).unwrap()
    );
}

#[test]
fn the_frame_is_clean_at_full_with_genus_one_and_matches_the_oracle() {
    let mut m = Model::default();
    let body = frame(&mut m);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}\n{}", dump_text(&m, body).unwrap());
    assert!(report.unchecked().is_empty(), "{report}");
    assert_eq!(report.euler().unwrap().to_string(), "16/24/10/12/1 g1 = 0");
    let two_loop_faces = m
        .faces(body)
        .unwrap()
        .iter()
        .filter(|f| m.face(f.id).unwrap().loops().len() == 2)
        .count();
    assert_eq!(two_loop_faces, 2, "the top and the bottom");
    let text = step::write(&m, &[body]).unwrap();
    oracle::compare("boolean/frame-cut", &text, None, "frame").unwrap();
    let mut other = Model::default();
    let twin = frame(&mut other);
    assert_eq!(
        dump_text(&m, body).unwrap(),
        dump_text(&other, twin).unwrap()
    );
}

#[test]
fn a_window_outside_the_box_is_refused_and_the_model_untouched() {
    let mut m = Model::default();
    let r = sample::frame(
        &mut m,
        Point3::origin(),
        Point3::new(40.0, 30.0, 10.0),
        Point2::new(-1.0, 10.0),
        Point2::new(30.0, 20.0),
    );
    assert!(
        matches!(
            r,
            Err(sample::SampleError::Extent {
                name: "window min x",
                ..
            })
        ),
        "{r:?}"
    );
    assert!(m.surface(arris_topo::SurfaceId::new(0, 0)).is_err());
}

#[test]
fn the_oracle_reports_a_mismatch_as_a_table_not_a_skip() {
    let mut m = Model::default();
    let body = cylinder(&mut m, 4.0, 12.0);
    let text = step::write(&m, &[body]).unwrap();
    let err = oracle::compare("primitive/box", &text, None, "cylinder-as-box").unwrap_err();
    assert!(matches!(err, oracle::OracleError::Mismatch { .. }), "{err}");
    let err = oracle::compare("primitive/box", &text, Some("nope"), "cylinder-as-box").unwrap_err();
    assert!(
        matches!(err, oracle::OracleError::Environment { .. }),
        "{err}"
    );
}
