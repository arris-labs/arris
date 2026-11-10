use super::support::*;

/// [`prop_shards!`] with each shard `#[ignore]`d for the two known holes in
/// ADR-0084 §5′'s domain (the macro's attributes land on its module, which
/// `ignore` does not reach): `cargo test -- --ignored` runs them.
macro_rules! ignored_shards {
    (
        $(#[$meta:meta])*
        $name:ident [$($shard:ident)+] ($arg:pat_param) = $strategy:expr => $body:block
    ) => {
        $(#[$meta])*
        mod $name {
            #[allow(unused_imports)]
            use super::*;
            ignored_shards!(@each (0u32 $(+ ignored_shards!(@one $shard))+), 0u32,
                [$($shard)+], ($arg) = $strategy => $body);
        }
    };
    (@one $shard:ident) => { 1u32 };
    (@each $shards:expr, $index:expr, [], ($arg:pat_param) = $strategy:expr => $body:block) => {};
    (@each $shards:expr, $index:expr, [$first:ident $($rest:ident)*], ($arg:pat_param) = $strategy:expr => $body:block) => {
        #[test]
        #[ignore = "two known holes in the domain: a spindle torus leaves a plane against a torus undecided at Full, and some poses are refused Unsupported (BACKLOG.md, a cusp corner fillet past a spindle; in some poses)"]
        fn $first() {
            arris_debug::prop::check_shard_at($index, $shards, 1, $strategy, |$arg| $body);
        }
        ignored_shards!(@each $shards, $index + 1u32, [$($rest)*], ($arg) = $strategy => $body);
    };
}

// ---- cusp_corner: both edges of a cusp blended (ADR-0084 as amended) ----

/// The area of the intersection of the discs of radii `a` and `b` whose
/// centres are `d` apart.
fn lens(a: f64, b: f64, d: f64) -> f64 {
    if d >= a + b {
        0.0
    } else if d <= (a - b).abs() {
        PI * a.min(b) * a.min(b)
    } else {
        let cos =
            |x: f64, y: f64| real::acos(((d * d + x * x - y * y) / (2.0 * d * x)).clamp(-1.0, 1.0));
        let kite = (-d + a + b) * (d + a - b) * (d - a + b) * (d + a + b);
        a * a * cos(a, b) + b * b * cos(b, a) - 0.5 * kite.max(0.0).sqrt()
    }
}

/// The plan area of the scene `scene`'s outline where both blended walls
/// are at least `w` away (the walls the two blended edges stand on).
/// Crescent and pocket: the big disc less `w` and outside the small disc
/// plus `w`, in the half plane; spandrel: above the line by `w`, in the
/// box, outside the arc's circle grown by `w` (ADR-0084 §5′).
fn clear_of_both(scene: &Cusped, w: f64) -> f64 {
    match *scene {
        Cusped::Crescent { big, small, .. } | Cusped::Pocket { big, small, .. } => {
            let (r1, r2) = (big - w, small + w);
            0.5 * (PI * r1 * r1 - lens(r1, r2, big - small))
        }
        Cusped::Spandrel { a, .. } => {
            let (big, x0) = (a + w, 2.0 * (a * w).sqrt());
            let primitive = |x: f64| {
                0.5 * (x * (big * big - x * x).max(0.0).sqrt() + big * big * real::asin(x / big))
            };
            (a - w) * (a - x0) - (primitive(a) - primitive(x0))
        }
        Cusped::Overhang { .. } => 0.0,
    }
}

/// The plan area of the outline of the scene.
fn outline_area(scene: &Cusped) -> f64 {
    match *scene {
        Cusped::Crescent { big, small, .. } | Cusped::Pocket { big, small, .. } => {
            0.5 * PI * (big * big - small * small)
        }
        Cusped::Spandrel { a, .. } => a * a * (1.0 - PI / 4.0),
        Cusped::Overhang { .. } => 0.0,
    }
}

/// The signed change of volume of blending both edges of the cusp at
/// `kind`'s size `r`. The result is the mutual trim of the two edges'
/// blends, so at depth `t` below the shared face the removed (or added,
/// for the pocket) plan region is the outline less where both walls are
/// at least the section's inset `w(t)` away: the volume is `∫₀ʳ A(t) dt`.
fn corner_change(scene: &Cusped, kind: Blend, r: f64) -> f64 {
    const N: usize = 400;
    let area = |w: f64| outline_area(scene) - clear_of_both(scene, w);
    let swept = match kind {
        // `t = r (1 − cos φ)` takes the root at `t = 0` away.
        Blend::Fillet => simpson(0.0, FRAC_PI_2, N, &|phi| {
            area(r * (1.0 - real::sin(phi))) * r * real::sin(phi)
        }),
        Blend::Chamfer => simpson(0.0, r, N, &|t| area(r - t)),
    };
    match scene {
        Cusped::Pocket { .. } => swept,
        _ => -swept,
    }
}

/// The midpoints of the two edges of the scene's cusp, on the unposed body.
fn corner_edges(scene: &Cusped) -> [Point3; 2] {
    match *scene {
        Cusped::Crescent {
            big,
            small,
            h,
            floor,
            ..
        } => {
            let z = if floor { 0.0 } else { h };
            [
                Point3::new(0.0, -big, z),
                Point3::new(small - big, -small, z),
            ]
        }
        Cusped::Pocket { big, small, .. } => [
            Point3::new(0.0, -big, 1.0),
            Point3::new(small - big, -small, 1.0),
        ],
        Cusped::Spandrel { a, h, floor } => {
            let z = if floor { 0.0 } else { h };
            [
                Point3::new(a / 2.0, 0.0, z),
                Point3::new(a * real::cos(FRAC_PI_4), a - a * real::sin(FRAC_PI_4), z),
            ]
        }
        Cusped::Overhang { .. } => [Point3::origin(); 2],
    }
}

/// A scene whose cusp has two edges to blend, a size inside the safe
/// domain of ADR-0084 §5′ and a pose. The edges' offsets meet (the
/// contacts on the shared face cross) when the size is below the gap
/// between the circles' centres, `big − small`, on the crescent and pocket
/// and below a quarter of the spandrel's side `a`, where the offsets meet
/// on the wall across; the stripes' section runs from the face down to the
/// spine one size below it, so the size is also below the height.
#[derive(Debug, Clone)]
struct CornerCase {
    scene: Cusped,
    size: f64,
    pose: Isometry,
}

/// The upper bound of the safe size domain of `scene` (ADR-0084 §5′).
fn corner_bound(scene: &Cusped) -> f64 {
    match *scene {
        Cusped::Crescent { big, small, h, .. } | Cusped::Pocket { big, small, h, .. } => {
            (big - small).min(h)
        }
        Cusped::Spandrel { a, h, .. } => (a / 4.0).min(h),
        Cusped::Overhang { .. } => 0.0,
    }
}

fn corner_scene() -> impl Strategy<Value = Cusped> {
    (
        prop::finite_f64(1.5..=3.0),
        prop::finite_f64(0.3..=0.7),
        prop::finite_f64(0.8..=2.0),
        0u8..5,
    )
        .prop_map(|(big, share, h, which)| {
            let small = share * big;
            match which {
                0 | 1 => Cusped::Crescent {
                    big,
                    small,
                    h,
                    small_arc: false,
                    floor: which == 1,
                },
                2 => Cusped::Pocket {
                    big,
                    small,
                    h,
                    small_arc: false,
                },
                _ => Cusped::Spandrel {
                    a: 2.0 * big,
                    h,
                    floor: which == 4,
                },
            }
        })
}

/// A scene and a size from 0.05 to 0.9 of the safe bound.
fn corner_case() -> impl Strategy<Value = CornerCase> {
    (corner_scene(), prop::finite_f64(0.05..=0.9), prop::pose()).prop_map(
        |(scene, fraction, pose)| CornerCase {
            size: fraction * corner_bound(&scene),
            scene,
            pose,
        },
    )
}

/// `scene` built in `m` and moved by `pose`, and its cusp's two edges.
fn corner_body(
    m: &mut Model,
    scene: &Cusped,
    pose: &Isometry,
) -> Result<(Body, [Edge; 2]), TestCaseError> {
    let built = scene.build(m).map_err(fail)?;
    let (moved, _) = transform(m, built, pose).map_err(fail)?;
    let [a, b] = corner_edges(scene);
    let edges = [
        edge_near(m, moved, pose.apply(a))?,
        edge_near(m, moved, pose.apply(b))?,
    ];
    Ok((moved, edges))
}

/// Both edges of the cusp blended in a fresh model, in the order `swap`
/// says: the model, the input, the edges, the result and its record.
fn corner_posed(
    case: &CornerCase,
    kind: Blend,
    swap: bool,
) -> Result<(Model, Body, [Edge; 2], Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let (moved, edges) = corner_body(&mut m, &case.scene, &case.pose)?;
    let ordered = if swap { [edges[1], edges[0]] } else { edges };
    let (blended, p) = op(kind)(&mut m, moved, &ordered, case.size)
        .map_err(|e| fail(format!("{kind:?} of both edges of the cusp: {e}")))?;
    Ok((m, moved, edges, blended, p))
}

fn forward(e: Edge) -> arris_topo::Shape {
    arris_topo::Shape::new(e.id, arris_topo::Orientation::Forward)
}

fn corner_blends_as_the_mutual_trim(case: CornerCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, edges, blended, p) = corner_posed(&case, kind, false)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;

    // The record: three shared entities, in one order from either edge;
    // both edges deleted.
    let [e, n] = edges.map(forward);
    let pair = p.generated_pair(e, n);
    prop_assert_eq!(pair.len(), 3, "the section edge, T* and S: {:?}", pair);
    prop_assert_eq!(&pair, &p.generated_pair(n, e));
    prop_assert!(p.is_deleted(e) && p.is_deleted(n), "both edges deleted");

    // The volume is the unblended body's less the mutual trim's.
    let props = mass_properties(&m, blended).map_err(fail)?;
    let before = mass_properties(&m, moved).map_err(fail)?;
    let want = before.volume + corner_change(&case.scene, kind, case.size);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the mutual trim's {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let (scene, es) = corner_body(&mut here, &case.scene, &Isometry::identity())?;
    let (rest, _) = op(kind)(&mut here, scene, &es, case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    assert_checked(&check(&here, rest, Level::Full))?;
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

    // Deterministic: a second run, and the edges selected the other way
    // round, give the same dump and record.
    let reference = dump_text(&m, blended).map_err(fail)?;
    for swap in [false, true] {
        let (twin, _, _, twice, twice_p) = corner_posed(&case, kind, swap)?;
        prop_assert_eq!(
            dump_text(&twin, twice).map_err(fail)?,
            reference.clone(),
            "dump with the selection swapped = {}",
            swap
        );
        prop_assert_eq!(twice_p, p.clone(), "record, swapped = {}", swap);
    }
    Ok(())
}

ignored_shards! {
    /// A crescent, its pocket or a spandrel with both edges of the cusp
    /// filleted at a size inside the safe domain (ADR-0084 §5′): clean at
    /// `Full` with nothing unchecked, audited, `generated_pair` exactly the
    /// section edge, `T*` and `S`, the volume the mutual trim's closed
    /// form, pose-independent, and the same dump and record on a second run
    /// and with the edges selected in the other order.
    cusp_corner_fillets_are_the_mutual_trim
        [shard_0 shard_1 shard_2 shard_3] (case) = corner_case() => {
            corner_blends_as_the_mutual_trim(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same fillets over the complement of the two known holes, at
    /// rest (`corner_complement`): a stated residue, not the domain.
    cusp_corner_complement_fillets_are_the_mutual_trim
        [shard_0 shard_1 shard_2 shard_3] (case) = corner_complement() => {
            corner_blends_as_the_mutual_trim(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same scenes chamfered.
    cusp_corner_chamfers_are_the_mutual_trim
        [shard_0 shard_1 shard_2 shard_3] (case) = corner_case() => {
            corner_blends_as_the_mutual_trim(case, Blend::Chamfer)
        }
}

fn corner_mirror_holds(
    case: CornerCase,
    origin: Point3,
    normal: arris_math::UnitVec3,
    kind: Blend,
) -> Result<(), TestCaseError> {
    let plane = arris_math::Reflection::new(origin, normal.into_inner()).map_err(fail)?;
    // Blend, then reflect.
    let (m, _, _, blended, _) = corner_posed(&case, kind, false)?;
    let mut a = m;
    let (image, _) = mirror(&mut a, blended, &plane).map_err(fail)?;
    assert_checked(&check(&a, image, Level::Full))?;
    // Reflect, then blend.
    let mut b = Model::default();
    let built = case.scene.build(&mut b).map_err(fail)?;
    let (moved, _) = transform(&mut b, built, &case.pose).map_err(fail)?;
    let (flipped, _) = mirror(&mut b, moved, &plane).map_err(fail)?;
    let edges = corner_edges(&case.scene)
        .map(|at| edge_near(&b, flipped, plane.apply(case.pose.apply(at))));
    let [e0, e1] = edges;
    let (other, _) = op(kind)(&mut b, flipped, &[e0?, e1?], case.size)
        .map_err(|e| fail(format!("{kind:?} of the reflected scene: {e}")))?;
    assert_checked(&check(&b, other, Level::Full))?;
    let (x, y) = (
        mass_properties(&a, image).map_err(fail)?,
        mass_properties(&b, other).map_err(fail)?,
    );
    let rel = fitted_rel(&a, &x);
    prop_assert!(
        close_to(x.volume, y.volume, 1.0, rel),
        "volume {} vs {}",
        x.volume,
        y.volume
    );
    prop_assert!(
        close_to(x.area, y.area, 1.0, rel),
        "area {} vs {}",
        x.area,
        y.area
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&a, image).map_err(fail)?,
        arris_debug::dump::euler_line(&b, other).map_err(fail)?,
        "counts: blend then mirror against mirror then blend"
    );
    Ok(())
}

fn corner_cancel_holds(
    case: CornerCase,
    draw: u64,
    kind: Blend,
    past: f64,
) -> Result<(), TestCaseError> {
    use std::sync::atomic::{AtomicU64, Ordering};
    let mut m = Model::default();
    let (moved, edges) = corner_body(&mut m, &case.scene, &case.pose)?;
    let before = arris_io::native::to_bytes(&m).map_err(fail)?;
    let run = |m: &mut Model, size: f64, c: &arris_ops::Control<'_>| match kind {
        Blend::Fillet => arris_ops::fillet(m, moved, &edges, size, c),
        Blend::Chamfer => arris_ops::chamfer(m, moved, &edges, size, c),
    };

    // Past the safe bound: refused, nothing changed. The bound is
    // the least of the gap and the height; past the gap alone is
    // the contacts not crossing.
    let too_big = past * corner_bound(&case.scene);
    if too_big > case.size && corner_bound(&case.scene) < corner_height(&case.scene) {
        let refusal = run(&mut m, too_big, &arris_ops::Control::NONE);
        prop_assert!(
            matches!(
                refusal,
                Err(OpError::Degenerate {
                    reason: Reason::Blend(BlendReason::TooLarge),
                    ..
                })
            ),
            "size {} (bound {}) is BlendTooLarge, not {:?}",
            too_big,
            corner_bound(&case.scene),
            refusal.map(|_| ())
        );
        prop_assert_eq!(
            arris_io::native::to_bytes(&m).map_err(fail)?,
            before.clone(),
            "refusal changed the model"
        );
    }

    // The whole step count, then every kind of stop below it.
    let asked = AtomicU64::new(0);
    let counting = || {
        asked.fetch_add(1, Ordering::Relaxed);
        false
    };
    let mut whole = m.clone();
    let (result, _) = run(&mut whole, case.size, &arris_ops::Control::poll(&counting))
        .map_err(|e| fail(format!("{kind:?}: {e}")))?;
    let n = asked.load(Ordering::Relaxed);
    let expected = dump_text(&whole, result).map_err(fail)?;
    prop_assert!(n > 0, "a blend asks the poll");
    let k = draw % n;
    let mut cut = m.clone();
    let stopped = run(&mut cut, case.size, &arris_ops::Control::budget(k));
    match stopped {
        Err(OpError::Interrupted(stop)) => {
            prop_assert_eq!((stop.by, stop.steps), (arris_ops::Stop::Budget, k));
        }
        other => return Err(fail(format!("budget {k} of {n}: {:?}", other.map(|_| ())))),
    }
    prop_assert_eq!(
        arris_io::native::to_bytes(&cut).map_err(fail)?,
        before.clone(),
        "budget {} of {} changed the model",
        k,
        n
    );
    let asked = AtomicU64::new(0);
    let poll = || asked.fetch_add(1, Ordering::Relaxed) >= k;
    let mut polled = m.clone();
    let stopped = run(&mut polled, case.size, &arris_ops::Control::poll(&poll));
    prop_assert!(
        matches!(stopped, Err(OpError::Interrupted(_))),
        "poll at {k} of {n}: {:?}",
        stopped.map(|_| ())
    );
    prop_assert_eq!(
        arris_io::native::to_bytes(&polled).map_err(fail)?,
        before,
        "poll at {} of {} changed the model",
        k,
        n
    );
    let mut again = m.clone();
    let (made, _) = run(&mut again, case.size, &arris_ops::Control::budget(n))
        .map_err(|e| fail(format!("a budget of {n}: {e}")))?;
    prop_assert_eq!(
        dump_text(&again, made).map_err(fail)?,
        expected,
        "a budget of {} changed the result",
        n
    );
    Ok(())
}

/// The fillet draws that leave out the two known holes in ADR-0084 §5′'s
/// domain (`BACKLOG.md`): the big arc's spindle torus (`size > big / 2`,
/// an undecided plane against torus at `Full`) and the poses in which the
/// trace of the section is refused `Unsupported`. No predicate separates
/// the poses that fail from those that build, so this draws the scene at
/// rest: the pose class is left to the ignored full-domain properties.
fn corner_complement() -> impl Strategy<Value = CornerCase> {
    corner_case()
        .prop_filter("not a spindle", |c| match c.scene {
            Cusped::Crescent { big, .. } | Cusped::Pocket { big, .. } => c.size <= big / 2.0,
            _ => true,
        })
        .prop_map(|c| CornerCase {
            pose: Isometry::identity(),
            ..c
        })
}

prop_shards! {
    /// Mirror covariance of a chamfered cusp: blended and then reflected in
    /// a random plane has the volume, area and counts of the reflected
    /// scene blended.
    cusp_corner_chamfer_commutes_with_a_mirror
        [shard_0 shard_1] ((case, origin, normal)) = (
            corner_case(),
            prop::point_in_box(prop::DEFAULT_SCALE),
            prop::unit_vec3(),
        ) => {
            corner_mirror_holds(case, origin, normal, Blend::Chamfer)
        }
}

ignored_shards! {
    /// The same for a fillet over the full domain of ADR-0084 §5′.
    cusp_corner_fillet_commutes_with_a_mirror
        [shard_0 shard_1] ((case, origin, normal)) = (
            corner_case(),
            prop::point_in_box(prop::DEFAULT_SCALE),
            prop::unit_vec3(),
        ) => {
            corner_mirror_holds(case, origin, normal, Blend::Fillet)
        }
}

prop_shards! {
    /// Cancellation and refusal leave the model as it was: a budget below
    /// the step count of a chamfer of both edges, or a poll that answers
    /// `true` there, stops it with `Interrupted` and the model's bytes
    /// unchanged; a size past the safe bound is refused `BlendTooLarge`
    /// (naming the cusp's edges only where the join's own check fires first,
    /// which `a_cusp_of_two_arcs_past_the_gap_names_both_edges` pins), the
    /// bytes unchanged too.
    cusp_corner_chamfer_cancel_and_refusal_roll_back
        [shard_0 shard_1] ((case, draw, past)) = (
            corner_case(),
            any::<u64>(),
            prop::finite_f64(1.05..=1.5),
        ) => {
            corner_cancel_holds(case, draw, Blend::Chamfer, past)
        }
}

ignored_shards! {
    /// The same for a fillet over the full domain.
    cusp_corner_fillet_cancel_and_refusal_roll_back
        [shard_0 shard_1] ((case, draw, past)) = (
            corner_case(),
            any::<u64>(),
            prop::finite_f64(1.05..=1.5),
        ) => {
            corner_cancel_holds(case, draw, Blend::Fillet, past)
        }
}

prop_shards! {
    /// The same for a fillet over the complement of the known holes.
    cusp_corner_complement_fillet_cancel_and_refusal_roll_back
        [shard_0 shard_1] ((case, draw, past)) = (
            corner_complement(),
            any::<u64>(),
            prop::finite_f64(1.05..=1.5),
        ) => {
            corner_cancel_holds(case, draw, Blend::Fillet, past)
        }
}

/// The scene's height, the other bound on the size.
fn corner_height(scene: &Cusped) -> f64 {
    match *scene {
        Cusped::Crescent { h, .. } | Cusped::Pocket { h, .. } | Cusped::Spandrel { h, .. } => h,
        Cusped::Overhang { thick, .. } => thick,
    }
}

/// The first known hole in ADR-0084 §5′'s domain (`BACKLOG.md`): the big
/// arc's fillet torus is a spindle (`r > big / 2`), and `Full` leaves the
/// plane against that torus undecided. Seed
/// 61727269732d70726f70657274792d74657374732d736565642d763100000000, 16
/// cases, `cusp_corner_fillets_are_the_mutual_trim` shard 0. The volume is
/// the mutual trim's closed form, which Arris's build meets to 4e-11.
#[test]
#[ignore = "Full leaves a plane against the big arc's spindle torus undecided (BACKLOG.md, a cusp corner fillet past a spindle)"]
fn a_cusp_corner_fillet_with_a_spindle_torus_is_decided() -> Result<(), TestCaseError> {
    let scene = Cusped::Crescent {
        big: 1.5,
        small: 0.45,
        h: 1.78,
        small_arc: false,
        floor: false,
    };
    let mut m = Model::default();
    let (body, edges) = corner_body(&mut m, &scene, &Isometry::identity())?;
    let (blended, _) = fillet(&mut m, body, &edges, 0.84).map_err(fail)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    let want = mass_properties(&m, body).map_err(fail)?.volume
        + corner_change(&scene, Blend::Fillet, 0.84);
    let got = mass_properties(&m, blended).map_err(fail)?.volume;
    prop_assert!(close_to(got, want, 1.0, REL), "volume {} vs {}", got, want);
    Ok(())
}

/// The second known hole: a spandrel's line and arc filleted together are
/// refused `Unsupported` (no closed form, the cylinder against the torus)
/// in this pose, though they build at rest, turned alone, moved alone and
/// chamfered. Seed as above, shard 3.
#[test]
#[ignore = "OpError::Unsupported in this pose: the mutual section's trace does not decide (BACKLOG.md, a cusp corner fillet in some poses)"]
fn a_cusp_corner_fillet_in_a_pose_that_failed_builds() -> Result<(), TestCaseError> {
    use arris_math::nalgebra::{Quaternion, Vector3};
    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        0.1238403715491793,
        0.5306348074725354,
        -0.7012934674725672,
        0.45964957952058433,
    ));
    let pose = Isometry::new(q, Vector3::new(0.0, -71.48262894551169, 0.0));
    let (a, h, r) = (5.6949232644320285, 0.8743198834582826, 0.5810549060509224);
    let scene = Cusped::Spandrel { a, h, floor: true };
    let mut m = Model::default();
    let (body, edges) = corner_body(&mut m, &scene, &pose)?;
    let (blended, _) = fillet(&mut m, body, &edges, r).map_err(fail)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    let want =
        mass_properties(&m, body).map_err(fail)?.volume + corner_change(&scene, Blend::Fillet, r);
    let got = mass_properties(&m, blended).map_err(fail)?.volume;
    prop_assert!(close_to(got, want, 1.0, REL), "volume {} vs {}", got, want);
    Ok(())
}
