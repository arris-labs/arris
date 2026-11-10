use super::support::*;

// A chain through a vertex of four edges (ADR-0039): the foot of a chamfer,
// between a side wall and the chamfer's strip, runs on round the whole foot
// outline where both faces turn tangentially. The section's corner swept
// along the outline's offsets in closed form, held at random poses.

/// A chamfered stadium or split pin and the blend of its chamfer's foot.
#[derive(Debug, Clone)]
struct FootCase {
    /// The stadium's straight length; `0` is the disc of two half circles
    /// (the split rim), its rim's two halves chamfered apart.
    length: f64,
    radius: f64,
    height: f64,
    /// The chamfer's distance and the blend's size.
    chamfer: f64,
    size: f64,
    pose: Isometry,
}

/// The corner's section of a blend of size `r` on the 135° edge between the
/// wall and the chamfer's strip: its area and its centroid's depth in from
/// the wall. A fillet's is the spandrel, a chamfer's the triangle of two
/// legs `r`.
fn foot_corner(kind: Blend, r: f64) -> (f64, f64) {
    match kind {
        Blend::Chamfer => (r * r * SQRT_2 / 4.0, r / (3.0 * SQRT_2)),
        Blend::Fillet => {
            let theta = FRAC_PI_4;
            let t = r * real::tan(theta / 2.0);
            let kite = r * t / 2.0;
            let sector = r * r * theta / 2.0;
            let moment = kite * r / 3.0 + kite * (t / SQRT_2 + r) / 3.0
                - sector * (r - 2.0 * r * real::sin(theta) / (3.0 * theta));
            let area = 2.0 * kite - sector;
            (area, moment / area)
        }
    }
}

impl FootCase {
    fn build(&self, m: &mut Model) -> Result<(Body, Point3), TestCaseError> {
        let (l, r) = (self.length, self.radius);
        let p = Point2::new;
        let split = l == 0.0;
        let outer = if split {
            ProfileLoop::Path {
                start: p(r, 0.0),
                segments: vec![
                    ProfileSegment::ArcTo {
                        to: p(-r, 0.0),
                        via: p(0.0, r),
                    },
                    ProfileSegment::ArcTo {
                        to: p(r, 0.0),
                        via: p(0.0, -r),
                    },
                ],
            }
        } else {
            ProfileLoop::Path {
                start: p(0.0, -r),
                segments: vec![
                    ProfileSegment::LineTo(p(l, -r)),
                    ProfileSegment::ArcTo {
                        to: p(l, r),
                        via: p(l + r, 0.0),
                    },
                    ProfileSegment::LineTo(p(0.0, r)),
                    ProfileSegment::ArcTo {
                        to: p(0.0, -r),
                        via: p(-r, 0.0),
                    },
                ],
            }
        };
        let profile = Profile {
            plane: Frame::world(),
            outer,
            holes: Vec::new(),
        };
        let body = extrude(m, &profile, Vec3::z(), self.height)
            .map_err(fail)?
            .0;
        let h = self.height;
        let rims: Vec<Point3> = if split {
            vec![Point3::new(0.0, r, h), Point3::new(0.0, -r, h)]
        } else {
            vec![
                Point3::new(l / 2.0, -r, h),
                Point3::new(l + r, 0.0, h),
                Point3::new(l / 2.0, r, h),
                Point3::new(-r, 0.0, h),
            ]
        };
        let mut edges = Vec::new();
        for at in rims {
            edges.push(edge_near(m, body, at)?);
        }
        let chamfered = chamfer(m, body, &edges, self.chamfer).map_err(fail)?.0;
        let foot = if split { 0.0 } else { l / 2.0 };
        Ok((chamfered, Point3::new(foot, r, h - self.chamfer)))
    }

    /// The volume after blending the foot: the stadium's, less the chamfer's
    /// and the blend's corner section, each swept along the outline's
    /// offset of length `2 length + 2π (radius − depth)`.
    fn volume(&self, kind: Blend) -> f64 {
        let (l, r) = (self.length, self.radius);
        let swept = |(area, depth): (f64, f64)| area * (2.0 * l + 2.0 * PI * (r - depth));
        let whole = self.height * (2.0 * l * r + PI * r * r);
        whole
            - swept((self.chamfer * self.chamfer / 2.0, self.chamfer / 3.0))
            - swept(foot_corner(kind, self.size))
    }
}

fn foot_case() -> impl Strategy<Value = FootCase> {
    (
        prop_oneof![Just(true), Just(false)],
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(1.2..=2.5),
        prop::finite_f64(0.1..=0.4),
        prop::finite_f64(0.1..=0.5),
        prop::pose(),
    )
        .prop_map(
            |(split, length, radius, height, chamfer, share, pose)| FootCase {
                length: if split { 0.0 } else { length },
                radius,
                height,
                chamfer,
                size: share * chamfer,
                pose,
            },
        )
}

fn foot_posed(
    case: &FootCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let (body, at) = case.build(&mut m)?;
    let (moved, _) = transform(&mut m, body, &case.pose).map_err(fail)?;
    let edge = edge_near(&m, moved, case.pose.apply(at))?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} of the posed foot: {e}")))?;
    Ok((m, moved, blended, p))
}

fn feet_blend_as_their_closed_forms(case: FootCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = foot_posed(&case, kind)?;
    feet_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let want = case.volume(kind);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let (body, at) = case.build(&mut here)?;
    let edge = edge_near(&here, body, at)?;
    let (rest, _) = op(kind)(&mut here, body, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    feet_checked(&check(&here, rest, Level::Full))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    feet_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = foot_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A chamfered stadium or split pin, the chamfer's foot filleted: the
    /// chain runs on round the outline through four vertices of four edges,
    /// clean at `Full`, audited, at the closed-form volume, pose-independent,
    /// through STEP and deterministic.
    fillets_through_a_vertex_of_four_edges_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = foot_case() => {
            feet_blend_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same feet chamfered.
    chamfers_through_a_vertex_of_four_edges_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = foot_case() => {
            feet_blend_as_their_closed_forms(case, Blend::Chamfer)
        }
}

/// A chamfered stadium turned and moved far out, the chamfer's foot
/// filleted and its chain walked round the outline: S5 leaves a plane
/// against a torus undecided at `Full` in exactly this pose — a translation
/// moved by `1e-14`, or by `1e-2`, decides it — which a fixture's axis and
/// angle cannot carry, found by `fillets_through_a_vertex_of_four_edges_
/// match_their_closed_forms`.
#[test]
#[ignore = "S5: a plane against a torus is not decided in this far pose of a walked chain, so Full has an unchecked pair (docs/BACKLOG.md, a plane against a torus in S5)"]
fn a_walked_chain_in_a_far_pose_is_decided() {
    use arris_math::nalgebra::{Quaternion, UnitQuaternion};
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        -0.09475886763043798,
        0.5890236655990984,
        -0.5660022382625896,
        0.5689581220541085,
    ));
    let case = FootCase {
        length: 2.578050057721793,
        radius: 1.0,
        height: 2.0333019670063206,
        chamfer: 0.12936930004443964,
        size: 0.042621373723744804,
        pose: Isometry::new(
            q,
            Vec3::new(63.84502733042328, 46.952118434286945, -39.56313382866168),
        ),
    };
    let (m, _, blended, _) = foot_posed(&case, Blend::Fillet).unwrap();
    assert_checked(&check(&m, blended, Level::Full)).unwrap();
}
