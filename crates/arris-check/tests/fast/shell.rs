//! S1–S4, B3, the Euler–Poincaré line and the `Full` rerun of the `Fast` rows.

use arris_check::{EdgeUseFault, Level, Violation, WireFault, check};
use arris_debug::sample;
use arris_geom::Curve;
use arris_math::{Interval, Point3, Vec3};
use arris_topo::entity::{
    Body as BodyEntity, BodyKind, Edge, EdgeGeometry, Face, Loop, Shell, Vertex,
};
use arris_topo::{
    Body, Edge as EdgeHandle, EdgeId, Face as FaceHandle, Model, Orientation, Shell as ShellHandle,
};

use crate::support::*;

#[test]
fn the_euler_line_of_each_sample_closes_and_a_sheets_does_not() {
    let mut m = Model::default();
    let box40 = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let cylinder = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let line = |body| check(&m, body, Level::Full).euler().unwrap();
    assert_eq!(line(box40).to_string(), "8/12/6/6/1 g0 = 0");
    assert_eq!(line(cylinder).to_string(), "2/3/3/3/1 g0 = 0");
    assert_eq!(line(box40).genus, 0);
    assert!(line(box40).closes() && line(cylinder).closes());
    // A sheet is not a closed orientable surface, and the counts say so:
    // no genus takes the residual to zero.
    let (sheet, _, _) = plate(&mut m, &[rect(0.0, 0.0, 10.0, 10.0, true)]);
    let report = check(&m, sheet, Level::Fast);
    let line = report.euler().unwrap();
    assert_eq!(line.to_string(), "4/4/1/1/1 g1 = 1");
    assert!(!line.closes());
    // The line is a line, never a violation.
    assert!(report.is_ok(), "{report}");
    // A body that does not resolve has no counts to take.
    assert_eq!(
        check(
            &m,
            Body::forward(arris_topo::BodyId::new(99, 0)),
            Level::Fast
        )
        .euler(),
        None
    );
}

#[test]
fn s1_a_face_used_twice_by_a_shell() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let face = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(r.coedges.clone())], tol));
    // The second use is reversed, so the uses of every edge still pair
    // up and S1 is the only thing wrong.
    let (body, shell) = body_of(
        &mut m,
        vec![
            FaceHandle::forward(face),
            FaceHandle::new(face, Orientation::Reversed),
        ],
        BodyKind::Sheet,
    );
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("S1", shell.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FaceUsedTwice { shell, face }
    );
}

/// S2: one face of a box used the other way round, so its four edges
/// have two uses that agree about which side the material is on.
#[test]
fn s2_a_solids_face_use_flipped() {
    let mut m = Model::default();
    let b = sample::cuboid(&mut m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap();
    let faces = m.faces(b).unwrap();
    let mut edges: Vec<EdgeId> = m.face(faces[0].id).unwrap().loops()[0]
        .coedges()
        .iter()
        .map(|c| c.edge())
        .collect();
    edges.sort();
    let uses: Vec<FaceHandle> = faces
        .iter()
        .enumerate()
        .map(|(i, f)| {
            if i == 0 {
                FaceHandle::new(f.id, f.orientation.flipped())
            } else {
                *f
            }
        })
        .collect();
    let (body, shell) = body_of(&mut m, uses, BodyKind::Solid);
    let report = check(&m, body, Level::Fast);
    let expected: Vec<(&str, String)> = edges.iter().map(|_| ("S2", shell.to_string())).collect();
    assert_lines(&report, &expected);
    for (v, edge) in report.violations().iter().zip(edges) {
        assert_eq!(
            *v,
            Violation::EdgeUses {
                shell,
                edge,
                fault: EdgeUseFault::Orientation,
            }
        );
    }
}

#[test]
fn s3_two_disconnected_face_sets_in_one_shell() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let near = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let far = ring(&mut m, &rect(20.0, 20.0, 30.0, 30.0, true));
    let uses: Vec<FaceHandle> = [near, far]
        .iter()
        .map(|r| {
            FaceHandle::forward(m.raw().add_face(Face::new(
                surface,
                vec![Loop::new(r.coedges.clone())],
                tol,
            )))
        })
        .collect();
    let (body, shell) = body_of(&mut m, uses, BodyKind::Sheet);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("S3", shell.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::ShellDisconnected {
            shell,
            components: 2,
        }
    );
}

/// S4: the sample cylinder without its top cap. The top circle is left
/// with one coedge, which is both the wrong count for a solid (S2) and
/// the hole that leaves the shell open (S4).
#[test]
fn s4_a_solids_edge_with_one_coedge() {
    let mut m = Model::default();
    let c = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let faces = m.faces(c).unwrap();
    let top_circle = m.edges(c).unwrap()[2].id;
    let (body, shell) = body_of(&mut m, vec![faces[0], faces[1]], BodyKind::Solid);
    let report = check(&m, body, Level::Fast);
    assert_lines(
        &report,
        &[("S2", shell.to_string()), ("S4", shell.to_string())],
    );
    assert_eq!(
        report.violations()[0],
        Violation::EdgeUses {
            shell,
            edge: top_circle,
            fault: EdgeUseFault::Count { coedges: 1 },
        }
    );
    assert_eq!(
        report.violations()[1],
        Violation::ShellOpen {
            shell,
            edge: top_circle,
        }
    );
}

#[test]
fn b3_a_wire_with_a_shell() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let face = m
        .raw()
        .add_face(Face::new(surface, vec![Loop::new(r.coedges.clone())], tol));
    let shell = m
        .raw()
        .add_shell(Shell::new(vec![FaceHandle::forward(face)]));
    let id = m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        vec![ShellHandle::forward(shell)],
        Vec::new(),
        Vec::new(),
    ));
    let report = check(&m, Body::forward(id), Level::Fast);
    assert_lines(&report, &[("B3", id.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::WireMalformed {
            body: id,
            fault: WireFault::HasShell { shell },
        }
    );
}

#[test]
fn b3_a_wire_whose_free_edges_meet_three_at_a_vertex() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let hub = m.raw().add_vertex(Vertex::new(Point3::origin(), tol));
    let mut free = Vec::new();
    for direction in [Vec3::x_axis(), Vec3::y_axis(), Vec3::z_axis()] {
        let tip = m
            .raw()
            .add_vertex(Vertex::new(Point3::origin() + direction.scale(10.0), tol));
        let curve = m.add_curve(Curve::Line {
            origin: Point3::origin(),
            direction,
        });
        free.push(EdgeHandle::forward(m.raw().add_edge(Edge::new(
            EdgeGeometry::Curve {
                curve,
                range: Interval::new(0.0, 10.0).unwrap(),
            },
            hub,
            tip,
            tol,
        ))));
    }
    let id = m.raw().add_body(BodyEntity::new(
        BodyKind::Wire,
        Vec::new(),
        free,
        Vec::new(),
    ));
    let report = check(&m, Body::forward(id), Level::Fast);
    assert_lines(&report, &[("B3", id.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::WireMalformed {
            body: id,
            fault: WireFault::VertexOverused {
                vertex: hub,
                edges: 3,
            },
        }
    );
}

/// The rows of this step run at `Fast`, so `Level::Full` reports them
/// too and no `Full` row of step 8 is claimed yet.
#[test]
fn the_fast_rows_also_run_at_full() {
    let mut m = Model::default();
    let (body, face, _) = plate(&mut m, &[rect(0.0, 0.0, 10.0, 10.0, false)]);
    let report = check(&m, body, Level::Full);
    assert_lines(&report, &[("L4", face.to_string())]);
}
