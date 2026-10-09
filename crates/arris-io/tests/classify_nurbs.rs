//! A real part's B-spline faces classified (ADR-0053):
//! NIST FTC-07's solid #7820, whose faces include B-spline surfaces a ray
//! from a probe has to cross. Before the line-against-NURBS arm,
//! `classify_point` refused such a body; now it gives Open CASCADE's class,
//! and the committed oracle values below are what
//! `BRepClass3d_SolidClassifier` answers for the same points (probes 0.5
//! either side of a B-spline face along its normal, and one on a face).

use arris_check::classify::{Classification, Classifier};
use arris_debug::unmetered::step_read;
use arris_io::step::ReadOptions;
use arris_math::Point3;
use arris_topo::{EntityKind, Model};

const PART: &str = "../../tests/fixtures/real/nist-ftc-07/nist_ftc_07_asme1_rd.stp";

#[test]
fn real_probes_either_side_of_a_real_parts_bspline_faces_get_the_oracles_class() {
    let text = std::fs::read_to_string(PART).expect("the committed NIST part");
    let mut m = Model::default();
    let read = step_read(&mut m, &text, &ReadOptions::default()).expect("the part reads");
    let body = read.solids[0]
        .result
        .as_ref()
        .expect("solid #7820 reads")
        .body;
    let classifier = Classifier::of_body(&m, body).expect("the body resolves");

    let probes: [([f64; 3], bool); 6] = [
        (
            [149.41777601243018, 0.06353341519543688, -107.47685398671177],
            true,
        ),
        (
            [
                149.42617155745853,
                -0.07578688462894186,
                -108.46706576652483,
            ],
            false,
        ),
        (
            [151.9268539867118, 0.06353341519544034, -104.96777601243006],
            true,
        ),
        (
            [
                152.91706576652481,
                -0.07578688462898728,
                -104.97617155745873,
            ],
            false,
        ),
        (
            [-151.92685398671176, 0.0635334151955326, -104.9677760124301],
            true,
        ),
        (
            [
                -152.9170657665248,
                -0.07578688462886388,
                -104.97617155745861,
            ],
            false,
        ),
    ];
    for (p, inside) in probes {
        let at = Point3::new(p[0], p[1], p[2]);
        let want = if inside {
            Classification::Inside
        } else {
            Classification::Outside
        };
        assert_eq!(classifier.classify(at), Ok(want), "{p:?}");
        assert_eq!(
            classifier.classify_linear(at),
            Ok(want),
            "{p:?}, every face"
        );
    }

    let on = Point3::new(
        149.42197378494436,
        -0.006126734716752489,
        -107.9719598766183,
    );
    match classifier.classify(on) {
        Ok(Classification::On(shape)) => assert_eq!(shape.kind(), EntityKind::Face),
        other => panic!("a point on a B-spline face is on it, not {other:?}"),
    }
}
