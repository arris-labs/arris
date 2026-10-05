//! The text dump and the sample bodies (`docs/DATA-MODEL.md` §Native
//! format, last paragraph; §Seams).

use core::f64::consts::{FRAC_PI_4, SQRT_2, TAU};

use arris_check::{Level, check};
use arris_debug::{dump_text, euler_line, sample};
use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Frame2, Interval, Point2, Point3, UnitVec3, Vec2, Vec3};
use arris_topo::entity::{Coedge, Edge, EdgeGeometry, Face, Loop, Vertex};
use arris_topo::{
    BodyId, Curve2Id, CurveId, EdgeId, FaceId, Model, Orientation, ShellId, SurfaceId, VertexId,
};

fn cylinder_dump() -> (Model, String) {
    let mut m = Model::default();
    let b = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = dump_text(&m, b).unwrap();
    (m, text)
}

fn box_dump() -> (Model, String) {
    let mut m = Model::default();
    let b = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let text = dump_text(&m, b).unwrap();
    (m, text)
}

#[test]
fn the_seam_edge_is_listed_twice_with_opposite_signs() {
    let (_, text) = cylinder_dump();
    let coedges: Vec<&str> = text
        .lines()
        .filter(|l| l.trim_start().starts_with("coedge "))
        .collect();
    assert_eq!(coedges.len(), 6, "four on the wall, one per cap:\n{text}");
    let seam_up = coedges.iter().filter(|l| l.contains("coedge +e1 ")).count();
    let seam_down = coedges.iter().filter(|l| l.contains("coedge -e1 ")).count();
    assert_eq!((seam_up, seam_down), (1, 1), "{text}");
    // The two seam pcurves are at u = 2π and u = 0.
    assert!(
        text.contains("coedge +e1 p1 line origin (6.283185307180, 0)")
            || text.contains("coedge +e1 p1 line origin (6.28318530718, 0)"),
        "{text}"
    );
    assert!(
        text.contains("coedge -e1 p3 line origin (0, 0) direction (0, 1)"),
        "{text}"
    );
    assert!(text.ends_with("euler 2/3/3/3/1 g0 = 0\n"), "{text}");
    // The edges section lists the seam once, with its first use's sign.
    let edges: Vec<&str> = text
        .lines()
        .skip_while(|l| *l != "edges")
        .skip(1)
        .take_while(|l| *l != "vertices")
        .filter(|l| l.starts_with("  "))
        .filter(|l| !l.starts_with("    "))
        .collect();
    assert_eq!(edges.len(), 3, "{text}");
    assert!(edges[1].starts_with("  +e1 v0 -> v1 c1 [0, 12]"), "{text}");
    assert!(
        text.contains("face -f1 S1"),
        "the bottom cap is used reversed:\n{text}"
    );
}

#[test]
fn the_box_line_is_8_12_6_6() {
    let (m, text) = box_dump();
    assert!(text.ends_with("euler 8/12/6/6/1 g0 = 0\n"), "{text}");
    assert_eq!(
        euler_line(&m, arris_topo::Body::forward(BodyId::new(0, 0))).unwrap(),
        "euler 8/12/6/6/1 g0 = 0"
    );
    assert!(text.starts_with("precision default 0.0000001 min 0.000000000001 max 0.01 angular 0.000000000001 parametric 0.0000001 samples 23\nbody +b0 solid\n  shell +s0\n"), "{text}");
    assert_eq!(text.lines().filter(|l| l.contains("coedge ")).count(), 24);
    assert!(text.contains("  v7 (40, 30, 10) tol 0.0000001"), "{text}");
}

#[test]
fn two_fresh_builds_dump_byte_identically() {
    assert_eq!(cylinder_dump().1, cylinder_dump().1);
    assert_eq!(box_dump().1, box_dump().1);
    let mut both = Model::default();
    let first = sample::cylinder(&mut both, 4.0, 12.0).unwrap();
    let second = sample::cylinder(&mut both, 4.0, 12.0).unwrap();
    assert_ne!(
        dump_text(&both, first).unwrap(),
        dump_text(&both, second).unwrap(),
        "the ids differ, so the dumps do"
    );
}

/// Every token of the form `[+-]?<kind letter><index>[g<gen>]` in a dump
/// resolves in its model.
fn assert_ids_resolve(m: &Model, text: &str) {
    let mut checked = 0;
    for token in text.split(|c: char| c.is_whitespace() || "()[],".contains(c)) {
        let bare = token.trim_start_matches(['+', '-']);
        let mut chars = bare.chars();
        let Some(kind) = chars.next() else { continue };
        let rest: &str = chars.as_str();
        if !"vefsbcSp".contains(kind) || rest.is_empty() {
            continue;
        }
        let (index, generation) = match rest.split_once('g') {
            Some((i, g)) => (i, g),
            None => (rest, "0"),
        };
        let (Ok(index), Ok(generation)) = (index.parse::<u32>(), generation.parse::<u32>()) else {
            continue;
        };
        let ok = match kind {
            'v' => m.vertex(VertexId::new(index, generation)).is_ok(),
            'e' => m.edge(EdgeId::new(index, generation)).is_ok(),
            'f' => m.face(FaceId::new(index, generation)).is_ok(),
            's' => m.shell(ShellId::new(index, generation)).is_ok(),
            'b' => m.body(BodyId::new(index, generation)).is_ok(),
            'c' => m.curve(CurveId::new(index, generation)).is_ok(),
            'S' => m.surface(SurfaceId::new(index, generation)).is_ok(),
            'p' => m.curve2(Curve2Id::new(index, generation)).is_ok(),
            _ => unreachable!(),
        };
        assert!(ok, "{token} does not resolve:\n{text}");
        checked += 1;
    }
    assert!(
        checked > 20,
        "the tokenizer found only {checked} ids:\n{text}"
    );
}

#[test]
fn every_id_in_a_dump_resolves() {
    let (m, text) = cylinder_dump();
    assert_ids_resolve(&m, &text);
    let (m, text) = box_dump();
    assert_ids_resolve(&m, &text);
    assert!(!text.contains('?'), "nothing dangling:\n{text}");
}

#[test]
fn a_dangling_reference_is_marked_not_hidden() {
    use arris_topo::entity::{Body, Shell};
    let mut m = Model::default();
    let s = m
        .raw()
        .add_shell(Shell::new(vec![arris_topo::Face::forward(FaceId::new(
            9, 0,
        ))]));
    let b = m
        .raw()
        .add_body(Body::solid(vec![arris_topo::Shell::forward(s)]));
    let text = dump_text(&m, arris_topo::Body::forward(b)).unwrap();
    assert!(text.contains("    face +f9 ?\n"), "{text}");
    assert!(text.ends_with("euler 0/0/0/0/1 g1 = 0\n"), "{text}");
    assert!(dump_text(&m, arris_topo::Body::forward(BodyId::new(3, 0))).is_err());
}

/// A cone of radius 1 at `z = 0` narrowing to its apex at `(0, 0, −1)`,
/// built by hand: the wall's loop walks the (u, v) rectangle
/// `[0, 2π] × [−√2, 0]` counter-clockwise — the apex's degenerate edge
/// along `v = −√2`, the seam up at `u = 2π`, the rim back along `v = 0`,
/// the seam down at `u = 0` — and the disc on `z = 0` closes it.
fn apex_cone(m: &mut Model) -> arris_topo::Body {
    use arris_topo::entity::{Body, Shell};
    let tol = m.precision().default_tolerance;
    // Radius `1 + v sin 45°` at height `v cos 45°`: the apex at `v = −√2`.
    let apex_v = -SQRT_2;
    let wall = m.add_surface(Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: FRAC_PI_4,
    });
    let disc = m.add_surface(Surface::Plane {
        frame: Frame::world(),
    });
    let seam = m.add_curve(Curve::Line {
        origin: Point3::new(0.0, 0.0, -1.0),
        direction: UnitVec3::new_normalize(Vec3::new(1.0, 0.0, 1.0)),
    });
    let rim = m.add_curve(Curve::Circle {
        frame: Frame::world(),
        radius: 1.0,
    });
    let apex = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(0.0, 0.0, -1.0), tol));
    let corner = m
        .raw()
        .add_vertex(Vertex::new(Point3::new(1.0, 0.0, 0.0), tol));
    let e_apex = m.raw().add_edge(Edge::new(
        EdgeGeometry::Degenerate {
            range: Interval::TURN,
        },
        apex,
        apex,
        tol,
    ));
    let e_seam = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: seam,
            range: Interval::new(0.0, SQRT_2).unwrap(),
        },
        apex,
        corner,
        tol,
    ));
    let e_rim = m.raw().add_edge(Edge::new(
        EdgeGeometry::Curve {
            curve: rim,
            range: Interval::TURN,
        },
        corner,
        corner,
        tol,
    ));
    let mut line = |u: f64, v: f64, along_u: bool| {
        m.add_curve2(Curve2::Line {
            origin: Point2::new(u, v),
            direction: if along_u {
                Vec2::x_axis()
            } else {
                Vec2::y_axis()
            },
        })
    };
    let wall_loop = Loop::new(vec![
        Coedge::new(e_apex, Orientation::Forward, line(0.0, apex_v, true)),
        Coedge::new(e_seam, Orientation::Forward, line(TAU, apex_v, false)),
        Coedge::new(e_rim, Orientation::Reversed, line(0.0, 0.0, true)),
        Coedge::new(e_seam, Orientation::Reversed, line(0.0, apex_v, false)),
    ]);
    let wall_face = m.raw().add_face(Face::new(wall, vec![wall_loop], tol));
    let p_disc = m.add_curve2(Curve2::Circle {
        frame: Frame2::identity(),
        radius: 1.0,
    });
    let disc_face = m.raw().add_face(Face::new(
        disc,
        vec![Loop::new(vec![Coedge::new(
            e_rim,
            Orientation::Forward,
            p_disc,
        )])],
        tol,
    ));
    let shell = m.raw().add_shell(Shell::new(vec![
        arris_topo::Face::forward(wall_face),
        arris_topo::Face::forward(disc_face),
    ]));
    let body = m
        .raw()
        .add_body(Body::solid(vec![arris_topo::Shell::forward(shell)]));
    arris_topo::Body::forward(body)
}

/// A degenerate edge is a singular point of its surface, not a boundary
/// between faces, so the Euler line leaves it out: a sphere with two pole
/// edges and a cone with an apex edge close at genus 0, in the dump and
/// in the checker's report alike, while the closure still holds every
/// edge (`docs/DATA-MODEL.md` §Euler–Poincaré).
#[test]
fn a_sphere_and_a_cone_close_at_genus_0_without_their_degenerate_edges() {
    let mut m = Model::default();
    let sphere = sample::sphere(&mut m, Point3::origin(), 3.0).unwrap();
    let cone = apex_cone(&mut m);
    for (body, line) in [(sphere, "2/1/1/1/1 g0 = 0"), (cone, "2/2/2/2/1 g0 = 0")] {
        let report = check(&m, body, Level::Fast);
        assert!(report.is_ok(), "{report}\n{}", dump_text(&m, body).unwrap());
        assert_eq!(report.euler().unwrap().to_string(), line);
        assert_eq!(euler_line(&m, body).unwrap(), format!("euler {line}"));
        assert!(
            dump_text(&m, body)
                .unwrap()
                .ends_with(&format!("euler {line}\n"))
        );
        assert_eq!(m.edges(body).unwrap().len(), 3);
    }
}

#[test]
fn samples_refuse_bad_extents_and_leave_the_model_untouched() {
    let mut m = Model::default();
    assert!(sample::cylinder(&mut m, 0.0, 1.0).is_err());
    assert!(sample::cylinder(&mut m, 1.0, f64::NAN).is_err());
    assert!(sample::cuboid(&mut m, Point3::origin(), Point3::new(1.0, -1.0, 1.0)).is_err());
    assert!(m.vertex(VertexId::new(0, 0)).is_err());
    assert!(m.curve(CurveId::new(0, 0)).is_err());
    assert!(sample::unit_box(&mut m).is_ok());
}
