//! The classifier's ray cast through the face hierarchy against the cast
//! over every face (ADR-0053): wherever the linear cast decides, the tree
//! cast gives the same answer, point for point.

use arris_check::classify::Classifier;
use arris_debug::prop::{self, check as prop_check};
use arris_debug::unmetered::cut;
use arris_math::Point3;
use arris_topo::Model;
use proptest::prelude::*;

#[test]
fn the_tree_cast_agrees_with_the_linear_cast_point_for_point() {
    prop_check(
        (
            prop::body::overlapping_pair(),
            proptest::collection::vec(prop::point_in_box(2.0 * prop::DEFAULT_SCALE), 6),
            0usize..3,
        ),
        |(pair, points, mode)| {
            let mut m = Model::default();
            let (a, b) = pair
                .build(&mut m)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            let body = match mode {
                0 => a,
                1 => b,
                _ => {
                    cut(&mut m, a, b)
                        .map_err(|e| TestCaseError::fail(e.to_string()))?
                        .0
                }
            };
            let classifier =
                Classifier::of_body(&m, body).map_err(|e| TestCaseError::fail(e.to_string()))?;
            // The probes, and the corners of the box they come from, which
            // are on or near the body's boundary.
            let corners = [Point3::origin(), Point3::new(1.0, 1.0, 1.0)];
            for p in points.iter().chain(corners.iter()) {
                let linear = classifier.classify_linear(*p);
                if linear.is_ok() {
                    prop_assert_eq!(classifier.classify(*p), linear, "{:?}", p);
                }
            }
            Ok(())
        },
    );
}
