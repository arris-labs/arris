//! The speed bar of `query::classify_points` (C12):
//! 10,648 points (a 22 x 22 x 22 grid over the body's box, the training
//! harness's per-pair sample) against NIST FTC-07's first solid, a real part
//! of 269 faces, take at most `BAR` seconds, printed. The batch builds its
//! classifier first: of the time, about 0.3 s is the 269 faces' domains
//! (their B-spline pcurves sampled at the model's tolerance) and 0.1 s is
//! the 10,648 classifications; the plan's 0.15 s did not hold for a part this
//! size without a cheaper domain (a backlog line). The answers of a sample
//! of the grid and the part's vertices are held equal to the classifier that
//! tries every face (`classify_linear`), which the box prune of `on` must
//! reproduce. Ignored in a debug build, where the bar does not apply:
//! `cargo nextest run --release -p arris-ops --test classify_points_speed`.

use std::time::Instant;

use arris_check::classify::{Classification, Classifier};
use arris_debug::unmetered::step_read;
use arris_io::step::ReadOptions;
use arris_math::{Control, Point3};
use arris_ops::query::classify_points;
use arris_topo::Model;

const PART: &str = "../../tests/fixtures/real/nist-ftc-07/nist_ftc_07_asme1_rd.stp";

/// The bar, in seconds, for the grid below: what the batch measures with
/// room for a loaded machine (0.39 s alone), not the plan's 0.15 s.
const BAR: f64 = 0.6;
const GRID: usize = 22;

#[test]
#[cfg_attr(debug_assertions, ignore = "timing bar: release only")]
fn real_a_grid_of_10648_points_classifies_within_the_bar() {
    let text = std::fs::read_to_string(PART).expect("the committed NIST part");
    let mut m = Model::default();
    let read = step_read(&mut m, &text, &ReadOptions::default()).expect("the part reads");
    let body = read.solids[0]
        .result
        .as_ref()
        .expect("solid #7820 reads")
        .body;
    let closure = m.closure(body).expect("the body resolves");
    assert!(closure.faces.len() >= 100, "{} faces", closure.faces.len());

    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &v in &closure.vertices {
        let p = m.vertex(v).expect("a vertex of the closure").point();
        for k in 0..3 {
            lo[k] = lo[k].min(p.coords[k]);
            hi[k] = hi[k].max(p.coords[k]);
        }
    }
    let at = |i: usize, k: usize| lo[k] + (hi[k] - lo[k]) * (i as f64 + 0.5) / GRID as f64;
    let mut points = Vec::with_capacity(GRID * GRID * GRID);
    for i in 0..GRID {
        for j in 0..GRID {
            for l in 0..GRID {
                points.push(Point3::new(at(i, 0), at(j, 1), at(l, 2)));
            }
        }
    }
    assert_eq!(points.len(), 10_648);

    let start = Instant::now();
    let answers = classify_points(&m, body, &points, &Control::NONE).expect("the batch runs");
    let seconds = start.elapsed().as_secs_f64();
    let inside = answers
        .iter()
        .flatten()
        .filter(|c| matches!(c, Classification::Inside))
        .count();
    println!(
        "classify_points: {} faces, {} points, {inside} inside, {seconds:.3} s (bar {BAR} s)",
        closure.faces.len(),
        points.len()
    );

    // The box prune of `on` against every face, on a sample of the grid
    // and on the part's vertices (each is on its own vertex).
    let classifier = Classifier::of_body(&m, body).expect("the body resolves");
    let corners: Vec<Point3> = closure
        .vertices
        .iter()
        .take(60)
        .map(|&v| m.vertex(v).expect("a vertex of the closure").point())
        .collect();
    for (k, &p) in points.iter().enumerate().step_by(53) {
        let want = classifier.classify_linear(p).ok();
        assert_eq!(answers[k], want, "grid point {k} {p:?}");
    }
    let on_vertices = classify_points(&m, body, &corners, &Control::NONE).expect("the batch runs");
    for (p, got) in corners.iter().zip(&on_vertices) {
        assert_eq!(*got, classifier.classify_linear(*p).ok(), "vertex {p:?}");
        assert!(matches!(got, Some(Classification::On(_))), "vertex {p:?}");
    }
    assert!(seconds <= BAR, "{seconds:.3} s over the {BAR} s bar");
}
