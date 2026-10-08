//! `FaceTree` against a brute-force box test (ADR-0053): the same faces,
//! in the stated order, on every run.

use arris_check::domain::FaceDomain;
use arris_check::tree::FaceTree;
use arris_debug::prop::{self, check as prop_check};
use arris_debug::unmetered::cut;
use arris_math::{Aabb, Point3, Vec3};
use arris_topo::{Body, FaceId, Model};
use proptest::prelude::*;

/// Every face of the body with its box, in closure order.
fn boxes(m: &Model, body: Body) -> Vec<(FaceId, Option<Aabb>)> {
    let tol = m.precision().parametric_tolerance;
    m.closure(body)
        .unwrap()
        .faces
        .into_iter()
        .map(|id| (id, FaceDomain::of(m, id, tol).ok().and_then(|d| d.bounds())))
        .collect()
}

/// The slab test, written out again.
fn entry(b: &Aabb, o: Point3, d: Vec3, range: (f64, f64)) -> Option<f64> {
    let (mut t0, mut t1) = range;
    for k in 0..3 {
        if d[k] == 0.0 {
            if o[k] < b.min[k] || o[k] > b.max[k] {
                return None;
            }
            continue;
        }
        let (a, c) = ((b.min[k] - o[k]) / d[k], (b.max[k] - o[k]) / d[k]);
        t0 = t0.max(a.min(c));
        t1 = t1.min(a.max(c));
    }
    (t0 <= t1).then_some(t0)
}

#[test]
fn along_and_near_agree_with_a_brute_force_box_test() {
    prop_check(
        (
            prop::body::overlapping_pair(),
            prop::point_in_box(2.0 * prop::DEFAULT_SCALE),
            prop::unit_vec3(),
            prop::radius(0.5..=30.0),
            0usize..3,
        ),
        |(pair, origin, dir, reach, mode)| {
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
            let all = boxes(&m, body);
            let tree =
                FaceTree::of_body(&m, body).map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(tree.len(), all.len());

            // Along: axis-parallel directions too, which hit the zero
            // component path of the slab test.
            let mut dirs = vec![dir.into_inner(), Vec3::new(0.0, 0.0, -1.0)];
            dirs.push(Vec3::new(dir.x, 0.0, 0.0));
            for d in dirs {
                let range = (0.0, reach);
                let mut want: Vec<(Option<f64>, FaceId)> = all
                    .iter()
                    .filter_map(|(id, b)| match b {
                        None => Some((None, *id)),
                        Some(b) => entry(b, origin, d, range).map(|t| (Some(t), *id)),
                    })
                    .collect();
                // Unbounded first, then entry order, ties by id.
                want.sort_by(|x, y| match (x.0, y.0) {
                    (None, None) => x.1.cmp(&y.1),
                    (None, Some(_)) => std::cmp::Ordering::Less,
                    (Some(_), None) => std::cmp::Ordering::Greater,
                    (Some(a), Some(b)) => a.total_cmp(&b).then(x.1.cmp(&y.1)),
                });
                let want: Vec<FaceId> = want.into_iter().map(|w| w.1).collect();
                let got = tree.along(origin, d, range);
                prop_assert_eq!(&got, &want);
                prop_assert_eq!(&tree.along(origin, d, range), &got, "same on every run");
            }

            // Near: a box about the origin.
            let region = Aabb {
                min: [
                    origin.x - reach / 3.0,
                    origin.y - reach / 3.0,
                    origin.z - reach / 3.0,
                ],
                max: [
                    origin.x + reach / 3.0,
                    origin.y + reach / 3.0,
                    origin.z + reach / 3.0,
                ],
            };
            let mut want: Vec<FaceId> = all
                .iter()
                .filter(|(_, b)| b.is_none_or(|b| b.intersects(&region)))
                .map(|(id, _)| *id)
                .collect();
            want.sort_unstable();
            prop_assert_eq!(tree.near(&region), want);
            Ok(())
        },
    );
}

#[test]
fn a_tree_over_no_boxes_answers_nothing_and_an_unbounded_face_always() {
    let empty = FaceTree::of_boxes(vec![]);
    assert!(empty.is_empty());
    let o = Point3::origin();
    assert!(empty.along(o, Vec3::x(), (0.0, 1.0)).is_empty());
    let mut m = Model::default();
    let body = arris_debug::sample::unit_box(&mut m).unwrap();
    let faces = m.closure(body).unwrap().faces;
    let tree = FaceTree::of_boxes(vec![(faces[0], None)]);
    assert_eq!(tree.along(o, Vec3::x(), (0.0, 1.0)), vec![faces[0]]);
    assert_eq!(tree.near(&Aabb::of_point(o)), vec![faces[0]]);
}
