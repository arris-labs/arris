use super::support::*;

// A kink and a vanishing end in a real part (ADR-0074, ADR-0075): the
// vertex's tolerance that covers them is a file's healed one, which no
// operation makes, so the parts are the corpus's own, read, moved to a
// random pose and filleted at a random radius. FTC-11's slot chains through
// `v24` (ADR-0075), CTC-05 AP203's e256 leaves its edge short of `v84`
// (ADR-0074). The result is clean at `Full` where the corpus's is, audits,
//  dumps identically twice.

/// The solid `id` of the corpus fixture `name`'s STEP file, read into `m`
/// and moved by `pose`: the original and the moved body.
fn corpus_part(
    m: &mut Model,
    dir: &str,
    file: &str,
    id: u64,
    pose: &Isometry,
) -> Result<Body, TestCaseError> {
    let path = arris_debug::fixtures::corpus_root().join(dir).join(file);
    let text = std::fs::read_to_string(&path).map_err(fail)?;
    let read = step_read(m, &text, &ReadOptions::default()).map_err(fail)?;
    let solid = read
        .solids
        .iter()
        .find(|s| s.entity.id == id)
        .ok_or_else(|| fail(format!("no solid #{id}")))?;
    let body = solid
        .result
        .as_ref()
        .map_err(|e| fail(format!("{e}")))?
        .body;
    Ok(transform(m, body, pose).map_err(fail)?.0)
}

/// What a pose must not change of a part filleted at `at` (in the file's
/// frame) and `r`: the result's counts of vertices, edges and faces. Its
/// volume is not held: a fitted NURBS part's volume moves 1e-6 of itself
/// from pose to pose, a hundred times the fillet's share (docs/BACKLOG.md).
type Shape = [usize; 3];

/// The part posed and filleted: the result's [`Shape`], its dump, the model
/// and bodies.
fn corpus_fillet(
    dir: &str,
    file: &str,
    id: u64,
    at: Point3,
    r: f64,
    pose: &Isometry,
) -> Result<(Shape, String, Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let part = corpus_part(&mut m, dir, file, id, pose)?;
    let edge = edge_near(&m, part, pose.apply(at))?;
    let (blended, p) = fillet(&mut m, part, &[edge], r).map_err(fail)?;
    let counts = [
        m.vertices(blended).map_err(fail)?.len(),
        m.edges(blended).map_err(fail)?.len(),
        m.faces(blended).map_err(fail)?.len(),
    ];
    let dump = dump_text(&m, blended).map_err(fail)?;
    Ok((counts, dump, m, part, blended, p))
}

/// The part's fillet in `pose` is clean at `Full`, audits, dumps as a second
/// run does, and has `reference`'s counts.
fn corpus_fillet_holds(
    dir: &str,
    file: &str,
    id: u64,
    at: Point3,
    r: f64,
    pose: &Isometry,
    reference: Shape,
) -> Result<(), TestCaseError> {
    let (counts, dump, m, part, blended, p) = corpus_fillet(dir, file, id, at, r, pose)?;
    let report = check(&m, blended, Level::Full);
    prop_assert!(report.is_ok(), "{}", report);
    audit(&m, &[part], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    prop_assert_eq!(counts, reference, "the pose changed the counts");
    let again = corpus_fillet(dir, file, id, at, r, pose)?;
    prop_assert_eq!(dump, again.1, "a second run");
    Ok(())
}

const SLOT_DIR: &str = "blend/nist-ftc-11-ap242-bspline-circle-fillet";
const SLOT_FILE: &str = "nist_ftc_11_asme1_ap242-e2-1299.stp";
const SLOT_EDGE: [f64; 3] = [-23.35841244624011, -4.182968297495704, 0.43];

fn slot_holds(r: f64, pose: &Isometry) -> Result<(), TestCaseError> {
    let at = Point3::new(SLOT_EDGE[0], SLOT_EDGE[1], SLOT_EDGE[2]);
    let reference = corpus_fillet(SLOT_DIR, SLOT_FILE, 1299, at, r, &Isometry::identity())?.0;
    corpus_fillet_holds(SLOT_DIR, SLOT_FILE, 1299, at, r, pose, reference)
}

prop_shards! {
    /// FTC-11's slot chain at a random radius and pose: through the kink at
    /// `v24` it is clean at `Full`, audits, has the counts the pose does not change, and dumps identically twice.
    slot_chain_through_its_kink_is_pose_independent
        [shard_0 shard_1] / 4
        (case) = (prop::finite_f64(0.004..=0.0105), prop::pose()) => {
            slot_holds(case.0, &case.1)
        }
}

const ELBOW_DIR: &str = "real/nist-ctc-05-ap203-torus-elbow-s5";
const ELBOW_FILE: &str = "nist_ctc_05_asme1_ap203-249.stp";
/// The file's radius: the vanishing end is this radius's. Nearby radii are
/// not all built (0.8895 and 0.895 are refused, 0.885 and 0.8899 build).
const ELBOW_RADIUS: f64 = 0.889;
const ELBOW_EDGE: [f64; 3] = [150.98396231882393, 31.26068001247087, 56.54476386520281];

fn elbow_holds(pose: &Isometry) -> Result<(), TestCaseError> {
    let at = Point3::new(ELBOW_EDGE[0], ELBOW_EDGE[1], ELBOW_EDGE[2]);
    let r = ELBOW_RADIUS;
    let reference = corpus_fillet(ELBOW_DIR, ELBOW_FILE, 249, at, r, &Isometry::identity())?.0;
    corpus_fillet_holds(ELBOW_DIR, ELBOW_FILE, 249, at, r, pose, reference)
}

prop_shards! {
    /// CTC-05 AP203's e256 at the file's radius in a random pose: the stripe ends short of `v84` on its narrowest exact arc and
    /// the edge's remnant runs on (ADR-0074), the chain on through `v115`
    /// (ADR-0075). Clean at `Full`, audits, has the counts the pose does not
    /// change and dumps identically twice.
    an_elbows_vanishing_end_is_pose_independent
        [shard_0 shard_1] / 16
        (pose) = prop::pose() => {
            elbow_holds(&pose)
        }
}
/// Two poses CI's 1000 cases drew: the stripe's torus edge `e794` was cut
/// from an edge whose tolerance was measured at the samples of its whole
/// range, and the checker, sampling the piece's range elsewhere, found a
/// gap 1e-13 above it (E4). The piece's tolerance is measured again.
#[test]
fn an_elbows_cut_edge_holds_its_pcurve_gap_in_the_poses_that_found_it() {
    use arris_math::nalgebra::{Quaternion, Unit};
    for (q, t) in [
        (
            [-0.5316200052406779, 0.8469829809552857, 0.0, 0.0],
            [77.72634321343656, 95.82277935844732, 0.0],
        ),
        (
            [
                -0.6480756590218757,
                -0.3440673409759742,
                0.0,
                0.679422994206911,
            ],
            [-49.38188549084462, -97.70284495841538, -73.94413016562885],
        ),
    ] {
        let pose = Isometry::new(
            Unit::new_unchecked(Quaternion::new(q[3], q[0], q[1], q[2])),
            Vec3::new(t[0], t[1], t[2]),
        );
        elbow_holds(&pose).unwrap();
    }
}
