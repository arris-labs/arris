//! F1–F2.

use arris_check::{FaceFault, Level, ToleranceBound, Violation, check};
use arris_topo::Model;
use arris_topo::entity::Loop;

use crate::support::*;

#[test]
fn f1_a_face_without_loops() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground(&mut m);
    let (body, face, _) = one_face(&mut m, surface, Vec::new(), tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("F1", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FaceMalformed {
            face,
            fault: FaceFault::NoLoops,
        }
    );
}

/// F1: the same plane as a NURBS patch of knot range `[0, 1]²`, with a
/// loop that walks a square of side two. The patch extrapolates to the
/// same points, so E4 and V3 hold; what is wrong is that the face claims
/// (u, v) its surface does not have.
#[test]
fn f1_a_pcurve_outside_the_surfaces_domain() {
    let mut m = Model::default();
    let tol = m.precision().default_tolerance;
    let surface = ground_patch(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 2.0, 2.0, true));
    let (body, face, _) = one_face(&mut m, surface, vec![Loop::new(r.coedges.clone())], tol);
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("F1", face.to_string())]);
    assert_eq!(
        report.violations()[0],
        Violation::FaceMalformed {
            face,
            fault: FaceFault::PcurveOutsideDomain {
                loop_index: 0,
                coedge: 0,
            },
        }
    );
}

#[test]
fn f2_a_face_tolerance_below_the_floor() {
    let mut m = Model::default();
    let precision = m.precision();
    let surface = ground(&mut m);
    let r = ring(&mut m, &rect(0.0, 0.0, 10.0, 10.0, true));
    let (body, face, _) = one_face(
        &mut m,
        surface,
        vec![Loop::new(r.coedges.clone())],
        precision.min_tolerance / 10.0,
    );
    let report = check(&m, body, Level::Fast);
    assert_lines(&report, &[("F2", face.to_string())]);
    assert!(matches!(
        report.violations()[0],
        Violation::FaceTolerance {
            bound: ToleranceBound::BelowMinimum,
            ..
        }
    ));
}

/// F2 and E5 are the same ordering read from the two sides, so a face
/// coarser than its edges is reported on the face and on every edge.
#[test]
fn f2_a_face_tolerance_above_its_edges() {
    let mut m = Model::default();
    let precision = m.precision();
    let surface = ground(&mut m);
    let r = ring(&mut m, &[p2(0.0, 0.0), p2(10.0, 0.0), p2(0.0, 10.0)]);
    let (body, face, _) = one_face(
        &mut m,
        surface,
        vec![Loop::new(r.coedges.clone())],
        precision.default_tolerance * 10.0,
    );
    let report = check(&m, body, Level::Fast);
    let f = face.to_string();
    assert_lines(
        &report,
        &[
            ("E5", r.edges[0].to_string()),
            ("E5", r.edges[1].to_string()),
            ("E5", r.edges[2].to_string()),
            ("F2", f.clone()),
            ("F2", f.clone()),
            ("F2", f),
        ],
    );
    for v in &report.violations()[3..] {
        assert!(matches!(
            v,
            Violation::FaceTolerance {
                bound: ToleranceBound::Neighbour { .. },
                ..
            }
        ));
    }
}
