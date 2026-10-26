//! V1–V3.

use arris_check::{Level, Violation, check};
use arris_debug::testing::cylinder;
use arris_geom::Curve2;
use arris_math::{Frame2, Interval, Point2, Precision, Vec2};
use arris_topo::Model;
use arris_topo::entity::{Coedge, EdgeGeometry, Face, Loop, Vertex};

use crate::support::*;

#[test]
fn v1_a_vertex_tolerance_below_the_floor_and_none_above_the_ceiling() {
    let p = Precision::DEFAULT;
    let mut m = Model::default();
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &|i, v| {
            if i == 0 {
                Vertex::new(v.point(), 10.0 * p.max_tolerance)
            } else {
                v
            }
        },
        &keep_edge,
        &keep_face,
    );
    // Above the ceiling is no violation: `max_tolerance` bounds what an
    // operation grows, and a healed read holds more (ADR-0060).
    let report = check(&m, r.body, Level::Fast);
    assert!(report.is_ok(), "{report}");
    // Below the floor: every edge at the vertex is now above it too (E5).
    let b = box40(&mut m);
    let r = rebuild(
        &mut m,
        b,
        &|i, v| {
            if i == 0 {
                Vertex::new(v.point(), p.min_tolerance / 10.0)
            } else {
                v
            }
        },
        &keep_edge,
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let v0 = r.vertices[0].to_string();
    let mut expected = vec![("V1", v0)];
    for e in m.vertex_edges(r.vertices[0]).unwrap() {
        expected.push(("E5", e.to_string()));
    }
    assert_lines(&report, &expected);
}

#[test]
fn v2_a_range_end_off_its_vertex() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let r = rebuild(
        &mut m,
        c,
        &keep_vertex,
        &|i, e| {
            if i == 1 {
                let (curve, _) = e.curve().unwrap();
                edit_edge(
                    &e,
                    EdgeGeometry::Curve {
                        curve,
                        range: Interval::new(0.5, 12.0).unwrap(),
                    },
                )
            } else {
                e
            }
        },
        &keep_face,
    );
    let report = check(&m, r.body, Level::Fast);
    let v0 = r.vertices[0].to_string();
    // The moved range end takes the seam's pcurve end with it, so the
    // wall's loop no longer meets itself in (u, v) at either side of it.
    let f0 = r.faces[0].to_string();
    assert_lines(
        &report,
        &[
            ("V2", v0.clone()),
            ("V3", v0),
            ("L2", f0.clone()),
            ("L2", f0),
        ],
    );
    assert!(matches!(
        report.violations()[0],
        Violation::VertexOffEdge { edge, distance, .. } if edge == r.edges[1] && (distance - 0.5).abs() < 1e-12
    ));
    assert!(matches!(
        report.violations()[1],
        Violation::VertexOffFace { face, .. } if face == r.faces[0]
    ));
}

#[test]
fn v3_a_pcurve_whose_end_misses_the_vertex() {
    let mut m = Model::default();
    let c = cylinder(&mut m);
    let shifted = m.add_curve2(Curve2::Circle {
        frame: Frame2::new(
            Point2::new(1.0, 0.0),
            Vec2::x(),
            arris_math::Handedness::Right,
        )
        .unwrap(),
        radius: 4.0,
    });
    let r = rebuild(&mut m, c, &keep_vertex, &keep_edge, &|i, f| {
        if i == 1 {
            let c = f.loops()[0].coedges()[0];
            Face::new(
                f.surface(),
                vec![Loop::new(vec![Coedge::new(
                    c.edge(),
                    c.orientation(),
                    shifted,
                )])],
                f.tolerance(),
            )
        } else {
            f
        }
    });
    let report = check(&m, r.body, Level::Fast);
    assert_lines(
        &report,
        &[
            ("V3", r.vertices[0].to_string()),
            ("E4", r.edges[0].to_string()),
        ],
    );
    assert!(matches!(
        report.violations()[0],
        Violation::VertexOffFace { face, distance, .. } if face == r.faces[1] && (distance - 1.0).abs() < 1e-12
    ));
}
