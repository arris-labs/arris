//! A real part's B-spline face offset (ADR-0062, `nurbs-face-blend` step
//! 2): NIST CTC-05's rational degree-(3, 2) face f109, the one the census
//! of the fillet column found under a fillet against a torus, offset by
//! the battery's radius both ways over its domain carried a sixth of its
//! width past each side — the ball's reach past the edge's ends is a tenth
//! (the plan's census). The span count and the time are printed for the
//! plan.

use arris_debug::unmetered::step_read;
use arris_geom::{Surface, SurfaceKind};
use arris_io::step::ReadOptions;
use arris_math::{Interval, Meter};
use arris_topo::Model;

const PART: &str =
    "../../tests/fixtures/real/nist-ctc-05-ap203-torus-elbow-s5/nist_ctc_05_asme1_ap203-249.stp";

/// The battery's fillet radius for the part.
const RADIUS: f64 = 0.8889999999999987;

/// How far past each side of the domain the offset is asked, as a
/// fraction of the domain's width there.
const PAST: f64 = 1.0 / 6.0;

#[test]
fn real_ctc_05s_bspline_face_offsets_at_the_census_radius() {
    let text = std::fs::read_to_string(PART).expect("the committed NIST part");
    let mut m = Model::default();
    let read = step_read(&mut m, &text, &ReadOptions::default()).expect("the part reads");
    let body = read.solids[0].result.as_ref().expect("#249 reads").body;
    let nurbs: Vec<_> = (m.faces(body).unwrap().iter())
        .filter_map(|f| {
            let face = m.face(f.id).ok()?;
            match m.surface(face.surface()).ok()? {
                Surface::Nurbs(s) => Some((f.id, face.tolerance(), s.clone())),
                _ => None,
            }
        })
        .collect();
    // The face the census names; the part's other three B-spline faces
    // meet no sampled edge.
    let Some((id, face_tolerance, s)) = nurbs.iter().find(|(id, _, _)| id.to_string() == "f109")
    else {
        panic!("no f109 among {} B-spline faces", nurbs.len());
    };
    assert_eq!(s.degree(), [3, 2]);
    assert_eq!(Surface::Nurbs(s.clone()).kind(), SurfaceKind::Nurbs);
    // The offset's share of the stripe's tolerance (ADR-0062 §1).
    let tol = 0.25 * m.precision().default_tolerance.max(*face_tolerance);
    let over = s.domain().map(|d| {
        let w = PAST * d.length();
        Interval::new(d.lo() - w, d.hi() + w).unwrap()
    });
    for d in [-RADIUS, RADIUS] {
        let started = std::time::Instant::now();
        let fit = s
            .offset_fitted(d, over, tol, &mut Meter::default())
            .unwrap_or_else(|e| panic!("{id} by {d}: {e}"));
        let elapsed = started.elapsed();
        let spans = [0, 1].map(|k| {
            let mut k: Vec<f64> = fit.knots()[k].to_vec();
            k.dedup();
            k.len() - 1
        });
        eprintln!(
            "{id} degree {:?}, offset by {d} over {over:?} to {tol:e}: {spans:?} spans of degree {:?} in {elapsed:?}",
            s.degree(),
            fit.degree()
        );
        assert_eq!(fit.domain(), over);
        for i in 0..=20 {
            for j in 0..=20 {
                let (u, v) = (over[0].lerp(i as f64 / 20.0), over[1].lerp(j as f64 / 20.0));
                let target = s.eval(u, v).point + d * s.normal(u, v).unwrap().into_inner();
                let off = (fit.eval(u, v).point - target).norm();
                assert!(off <= tol, "{id} by {d} at ({u}, {v}): {off}");
            }
        }
    }
}
