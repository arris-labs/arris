use super::support::*;

// A fillet whose contact runs over an edge of its face (ADR-0071): a
// boss's root near a plate's edge, and a slab's lip over a cross hole. The
// ball rolls on the edge between the crossings, so the body is clean at
// `Full` and audits where it builds, its volume between the unblended
// body's and the whole ring's or lip's section swept (the run-over adds
// less than the ring and takes less than the lip), and two runs dump
// identically; a case it does not cover is refused by name.

/// The result of `fillet` at `at`: built, or refused as a degenerate
/// blend naming its entities; any other error fails the case.
fn built_or_refused(
    m: &mut Model,
    body: Body,
    at: Point3,
    r: f64,
) -> Result<Option<(Body, Provenance)>, TestCaseError> {
    let edge = edge_near(m, body, at)?;
    match fillet(m, body, &[edge], r) {
        Ok(built) => Ok(Some(built)),
        Err(OpError::Degenerate { .. }) => Ok(None),
        Err(e) => Err(fail(format!("fillet at {at}: {e}"))),
    }
}

/// `body` blended is clean at `Full`, audits against `input`, and dumps
/// as it does in a second run of `again`.
fn run_over_holds(
    m: &Model,
    input: Body,
    blended: Body,
    p: &Provenance,
    again: &(Model, Body),
) -> Result<(), TestCaseError> {
    let report = check(m, blended, Level::Full);
    prop_assert!(report.is_ok(), "{}", report);
    audit(m, &[input], blended, p).map_err(|e| fail(format!("provenance: {e}")))?;
    prop_assert_eq!(
        dump_text(m, blended).map_err(fail)?,
        dump_text(&again.0, again.1).map_err(fail)?,
        "a second run"
    );
    Ok(())
}

/// A 10 × 10 × 2 plate with a boss of `radius` 2 high on its top whose
/// axis is `distance` from the front edge `y = 0`, the root circle
/// filleted at `r`: the contact on the top, of radius `radius + r`, runs
/// off that edge where `distance < radius + r`.
fn standoff(
    radius: f64,
    distance: f64,
    r: f64,
) -> Result<Option<(Model, Body, Body, Provenance)>, TestCaseError> {
    let mut m = Model::default();
    let (plate, _) = primitive_box(
        &mut m,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(10.0, 10.0, 2.0),
    )
    .map_err(fail)?;
    let axis = Axis::new(Point3::new(5.0, distance, 1.0), Vec3::z()).map_err(fail)?;
    let (boss, _) = primitive_cylinder(&mut m, axis, radius, 3.0).map_err(fail)?;
    let (part, _) = fuse(&mut m, plate, boss).map_err(fail)?;
    let at = Point3::new(5.0 - radius, distance, 2.0);
    Ok(built_or_refused(&mut m, part, at, r)?.map(|(b, p)| (m, part, b, p)))
}

fn standoff_holds(radius: f64, share: f64, r: f64) -> Result<(), TestCaseError> {
    let distance = radius + share * r;
    let Some((m, part, blended, p)) = standoff(radius, distance, r)? else {
        return Ok(());
    };
    let again = standoff(radius, distance, r)?
        .map(|(m, _, b, _)| (m, b))
        .ok_or_else(|| fail("built once and refused the next time"))?;
    run_over_holds(&m, part, blended, &p, &again)?;
    let before = mass_properties(&m, part).map_err(fail)?.volume;
    let added = mass_properties(&m, blended).map_err(fail)?.volume - before;
    let ring = (1.0 - PI / 4.0) * r * r * 2.0 * PI * (radius + r);
    prop_assert!(
        added > 0.0 && added <= ring * (1.0 + REL),
        "the fillet adds {added}, at most the whole ring's {ring}"
    );
    Ok(())
}

prop_shards! {
    /// A boss's root at random radius, fillet radius and distance from the
    /// plate's edge, its contact running off the edge by a random share of
    /// the fillet radius.
    standoff_roots_roll_on_the_plates_edge
        [shard_0 shard_1 shard_2 shard_3]
        (case) = (prop::finite_f64(1.0..=2.0), prop::finite_f64(0.05..=0.95), prop::finite_f64(0.3..=0.8)) => {
            standoff_holds(case.0, case.1, case.2)
        }
}

/// A 12 × 20 × 8 slab with a hole of `a` across it along `x`, its axis at
/// height `c`, the top edge at `x = 12` filleted at `r`.
fn lip(a: f64, c: f64, r: f64) -> Result<Option<(Model, Body, Body, Provenance)>, TestCaseError> {
    let mut m = Model::default();
    let (slab, _) = primitive_box(
        &mut m,
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(12.0, 20.0, 8.0),
    )
    .map_err(fail)?;
    let axis = Axis::new(Point3::new(-1.0, 10.0, c), Vec3::x()).map_err(fail)?;
    let (drill, _) = primitive_cylinder(&mut m, axis, a, 14.0).map_err(fail)?;
    let (part, _) = cut(&mut m, slab, drill).map_err(fail)?;
    let at = Point3::new(12.0, 10.0, 8.0);
    Ok(built_or_refused(&mut m, part, at, r)?.map(|(b, p)| (m, part, b, p)))
}

fn lip_holds(a: f64, c: f64, r: f64) -> Result<(), TestCaseError> {
    let Some((m, part, blended, p)) = lip(a, c, r)? else {
        return Ok(());
    };
    let again = lip(a, c, r)?
        .map(|(m, _, b, _)| (m, b))
        .ok_or_else(|| fail("built once and refused the next time"))?;
    run_over_holds(&m, part, blended, &p, &again)?;
    let before = mass_properties(&m, part).map_err(fail)?.volume;
    let taken = before - mass_properties(&m, blended).map_err(fail)?.volume;
    let lip = (1.0 - PI / 4.0) * r * r * 20.0;
    prop_assert!(
        taken > 0.0 && taken <= lip * (1.0 + REL),
        "the fillet takes {taken}, at most the whole lip's {lip}"
    );
    Ok(())
}

prop_shards! {
    /// A slab's lip over a cross hole at random radius and height, the
    /// contact on the end face running through the hole's rim wherever the
    /// hole reaches it.
    slab_lips_roll_on_a_cross_holes_rim
        [shard_0 shard_1 shard_2 shard_3]
        (case) = (prop::finite_f64(2.0..=3.0), prop::finite_f64(4.0..=4.5), prop::finite_f64(1.0..=2.0)) => {
            lip_holds(case.0, case.1, case.2)
        }
}

// A rounded block under an oblique top, or a pocket under an oblique floor,
// one line of the top filleted: the chain runs on round the eight edges of
// the top, and at the vertices where a wall turns into a corner cylinder
// along a ruling that leans out of the ball's normal plane the table stripe
// and the walked stripe meet on one ball off the vertex (ADR-0072). Each
// case builds clean at `Full`, nothing unchecked, or is refused by name,
// and each shard builds at least its floor of cases;
// every arc of radius `r` the blend makes has its centre at `r` from the
// tilted plane and at `r` from the ruling of one of the corner cylinders'
// eight tangents with the walls.

/// The geometry of one block or pocket, in its own frame: a `length` ×
/// `width` rounded rectangle, corners at `corner`, the tilted plane through
/// `(0, width / 2, 2)` at `tilt` about `x`.
#[derive(Debug, Clone, Copy)]
struct Oblique {
    length: f64,
    width: f64,
    corner: f64,
    tilt: f64,
    r: f64,
    concave: bool,
    pose: Isometry,
}

impl Oblique {
    const TOP: f64 = 2.0;

    fn profile(&self, z: f64) -> Result<Profile, TestCaseError> {
        let (l, w, c) = (self.length, self.width, self.corner);
        let k = c * (1.0 - real::cos(FRAC_PI_4));
        let p = Point2::new;
        let frame = Frame::new(Point3::new(0.0, 0.0, z), Vec3::z(), Vec3::x()).map_err(fail)?;
        Ok(Profile {
            plane: frame,
            outer: ProfileLoop::Path {
                start: p(c, 0.0),
                segments: vec![
                    ProfileSegment::LineTo(p(l - c, 0.0)),
                    ProfileSegment::ArcTo {
                        to: p(l, c),
                        via: p(l - k, k),
                    },
                    ProfileSegment::LineTo(p(l, w - c)),
                    ProfileSegment::ArcTo {
                        to: p(l - c, w),
                        via: p(l - k, w - k),
                    },
                    ProfileSegment::LineTo(p(c, w)),
                    ProfileSegment::ArcTo {
                        to: p(0.0, w - c),
                        via: p(k, w - k),
                    },
                    ProfileSegment::LineTo(p(0.0, c)),
                    ProfileSegment::ArcTo {
                        to: p(c, 0.0),
                        via: p(k, k),
                    },
                ],
            },
            holes: Vec::new(),
        })
    }

    /// The motion that tilts the plane `z = 2` about `x` through
    /// `(0, width / 2, 2)`.
    fn tilting(&self) -> Isometry {
        let about = Vec3::new(0.0, self.width / 2.0, Self::TOP);
        let turn = UnitQuaternion::from_axis_angle(&Vec3::x_axis(), self.tilt);
        Isometry::from_translation(-about)
            .then(&Isometry::from_rotation(turn))
            .then(&Isometry::from_translation(about))
    }

    /// The body, the unblended edge's midpoint in the posed frame.
    fn build(&self, m: &mut Model) -> Result<(Body, Point3), TestCaseError> {
        let (l, w) = (self.length, self.width);
        let (slab, _) = if self.concave {
            primitive_box(
                m,
                Point3::new(-1.0, -3.0, -3.0),
                Point3::new(l + 1.0, w + 3.0, Self::TOP),
            )
        } else {
            primitive_box(
                m,
                Point3::new(-1.0, -3.0, Self::TOP),
                Point3::new(l + 1.0, w + 3.0, 8.0),
            )
        }
        .map_err(fail)?;
        let (tilted, _) = transform(m, slab, &self.tilting()).map_err(fail)?;
        let body = if self.concave {
            let (stock, _) = primitive_box(
                m,
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(l + 1.0, w + 1.0, 4.0),
            )
            .map_err(fail)?;
            let (prism, _) = extrude(m, &self.profile(0.5)?, Vec3::z(), 4.5).map_err(fail)?;
            let (pocket, _) = cut(m, prism, tilted).map_err(fail)?;
            cut(m, stock, pocket).map_err(fail)?.0
        } else {
            let (prism, _) = extrude(m, &self.profile(0.0)?, Vec3::z(), 6.0).map_err(fail)?;
            cut(m, prism, tilted).map_err(fail)?.0
        };
        let at = Point3::new(l / 2.0, 0.0, Self::TOP - (w / 2.0) * real::tan(self.tilt));
        let (moved, _) = transform(m, body, &self.pose).map_err(fail)?;
        Ok((moved, self.pose.apply(at)))
    }

    /// The vertical lines where the walls meet the corner cylinders, as
    /// points of `(x, y)`.
    fn rulings(&self) -> [(f64, f64); 8] {
        let (l, w, c) = (self.length, self.width, self.corner);
        [
            (c, 0.0),
            (l - c, 0.0),
            (l, c),
            (l, w - c),
            (l - c, w),
            (c, w),
            (0.0, w - c),
            (0.0, c),
        ]
    }
}

/// Whether `case` built: `false` for a refusal by name, an error for
/// anything else that is not as the property says.
fn oblique_holds(case: Oblique) -> Result<bool, TestCaseError> {
    let mut m = Model::default();
    let (part, at) = case.build(&mut m)?;
    let edge = edge_near(&m, part, at)?;
    let (blended, p) = match fillet(&mut m, part, &[edge], case.r) {
        Ok(built) => built,
        Err(OpError::Degenerate { .. } | OpError::Unsupported { .. }) => return Ok(false),
        Err(e) => return Err(fail(format!("fillet of {case:?}: {e}"))),
    };
    // Clean at `Full`: every pair the walked stripes make, against the
    // walls, the corner cylinders and the tilted plane, decided
    // (ADR-0061).
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[part], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    // The arcs: each circle of radius `r` has its centre at `r` from the
    // tilted plane and from one of the rulings, in the frame of the case.
    let back = case.pose.inverse();
    let (ny, nz) = (-real::sin(case.tilt), real::cos(case.tilt));
    let mut arcs = 0;
    for e in m.edges(blended).map_err(fail)? {
        let Some((curve, _)) = m.edge(e.id).map_err(fail)?.curve() else {
            continue;
        };
        let Curve::Circle { frame, radius } = m.curve(curve).map_err(fail)? else {
            continue;
        };
        if (radius - case.r).abs() > 1e-7 * case.r {
            continue;
        }
        arcs += 1;
        let c = back.apply(frame.origin());
        let to_plane = (ny * (c.y - case.width / 2.0) + nz * (c.z - Oblique::TOP)).abs();
        prop_assert!(
            (to_plane - case.r).abs() <= 1e-6 * case.r,
            "an arc's centre {c} is {to_plane} from the tilted plane, not {}",
            case.r
        );
        let to_ruling = case
            .rulings()
            .iter()
            .map(|&(x, y)| real::hypot(c.x - x, c.y - y))
            .fold(f64::INFINITY, f64::min);
        prop_assert!(
            (to_ruling - case.r).abs() <= 1e-6 * case.r,
            "an arc's centre {c} is {to_ruling} from the nearest ruling, not {}",
            case.r
        );
    }
    prop_assert!(
        arcs == 8,
        "{arcs} arcs of radius {} for 8 junctions",
        case.r
    );
    Ok(true)
}

/// A rounded block or pocket at a random tilt of 5° to 40°, corner and
/// fillet radius and pose: the junctions at leaning rulings meet on one
/// ball off the vertex. A refusal by name passes a case, so each shard
/// also builds at least `BUILT_FLOOR` of the cases it ran (a budgeted
/// shard may stop early): a regression that refused every pose would
/// otherwise pass with nothing built.
mod oblique_chains_meet_on_the_rulings_offset {
    use super::*;
    use std::cell::Cell;

    const SHARDS: u32 = 4;

    /// The share of a shard's cases run that must build: half the least a
    /// shard of a 1000-case run built, 225 of 250 (225 to 234,
    /// walked-saddle-cell, step 9).
    const BUILT_FLOOR: f64 = 0.45;

    thread_local! {
        /// The cases this thread's shard built; the runner calls the
        /// property on the shard's own thread (no fork, no timeout).
        static BUILT: Cell<u32> = const { Cell::new(0) };
        /// The cases this thread's shard ran: under the nightly time
        /// budget a shard may stop before its nominal count.
        static RAN: Cell<u32> = const { Cell::new(0) };
    }

    fn shard(index: u32) {
        BUILT.set(0);
        RAN.set(0);
        prop::check_shard(
            index,
            SHARDS,
            (
                prop::finite_f64(3.0..=5.0),
                prop::finite_f64(1.5..=2.5),
                prop::finite_f64(0.3..=0.6),
                prop::finite_f64(5.0..=40.0),
                prop::finite_f64(0.1..=0.25),
                any::<bool>(),
                prop::pose(),
            ),
            |(length, width, corner, tilt, r, concave, pose)| {
                RAN.set(RAN.get() + 1);
                let built = oblique_holds(Oblique {
                    length,
                    width,
                    corner,
                    tilt: tilt.to_radians(),
                    r,
                    concave,
                    pose,
                })?;
                if built {
                    BUILT.set(BUILT.get() + 1);
                }
                Ok(())
            },
        );
        let ran = RAN.get();
        let built = BUILT.get();
        let floor = (BUILT_FLOOR * f64::from(ran)).floor();
        assert!(
            f64::from(built) >= floor,
            "shard {index} built {built} of {ran} cases run, under its floor of {floor}"
        );
    }

    #[test]
    fn shard_0() {
        shard(0);
    }

    #[test]
    fn shard_1() {
        shard(1);
    }

    #[test]
    fn shard_2() {
        shard(2);
    }

    #[test]
    fn shard_3() {
        shard(3);
    }
}

/// A rounded block under a tilted top at 31°, found by
/// `oblique_chains_meet_on_the_rulings_offset` once its S5 allowance is
/// removed (1000 cases, walked-saddle-cell, step 6): a walked skin
/// tangent to its table cylinder along the junction's arc, whose slope
/// across the isoline, measured as if dropped over the whole domain, was
/// past the tangency's bound, so the line was divided out as a crossing
/// and the tracer refused the pair as `UnresolvedTurning`. The slope is
/// dropped over the patches holding the line alone (ADR-0061, amended
/// 2026-10-29).
#[test]
fn an_oblique_fillet_whose_skin_is_tangent_to_its_table_cylinder_is_checked() {
    let pose = Isometry::new(
        arris_math::nalgebra::Unit::new_unchecked(arris_math::nalgebra::Quaternion::new(
            0.0, 1.0, 0.0, 0.0,
        )),
        Vec3::new(0.0, 0.0, -89.16487392943412),
    );
    let case = Oblique {
        length: 3.0,
        width: 1.5,
        corner: 0.4591144096367237,
        tilt: 31.015754421798317f64.to_radians(),
        r: 0.22468281425389497,
        concave: false,
        pose,
    };
    let mut m = Model::default();
    let (part, at) = case.build(&mut m).unwrap();
    let edge = edge_near(&m, part, at).unwrap();
    let (blended, _) = fillet(&mut m, part, &[edge], case.r).unwrap();
    assert_checked(&check(&m, blended, Level::Full)).unwrap();
}

/// A rounded block under a tilted top at 5.1°, found by
/// `oblique_chains_meet_on_the_rulings_offset` once its S5 allowance is
/// removed (1000 cases, walked-saddle-cell, step 7): two planes of the
/// block against two walked faces of the fillet, whose sections are
/// slivers round the skin's corner touch, one end on its edge `u = 0`,
/// which the tracer refused as `UnresolvedTurning` (ADR-0061, amended
/// 2026-10-29).
#[test]
fn an_oblique_fillet_at_a_shallow_tilt_is_checked() {
    let pose = Isometry::new(
        arris_math::nalgebra::Unit::new_unchecked(arris_math::nalgebra::Quaternion::new(
            0.0, 0.0, 1.0, 0.0,
        )),
        Vec3::zeros(),
    );
    oblique_is_checked(
        0.5147874278629065,
        5.104592103941619,
        0.10587863935816388,
        pose,
    );
}

/// The same block as `an_oblique_fillet_at_a_shallow_tilt_is_checked`
/// posed 90 above the origin (shard 1 of the same 1000-case run): the
/// table stripes' cylinders against two walked skins, `CrowdedSingularity`
/// from `singular_points`: an uncertified box of the quotient's critical
/// points past the skin's edge `v = 2π`, beside the isoline of its
/// tangency to the cylinder, which the rounding floor at the pose leaves
/// uncertified and which was refused, on the edge, as too flat to decide.
/// The box is the isoline's (ADR-0061, amended 2026-10-29).
#[test]
fn an_oblique_fillet_at_a_shallow_tilt_far_from_the_origin_is_checked() {
    let pose = Isometry::new(
        arris_math::nalgebra::Unit::new_unchecked(arris_math::nalgebra::Quaternion::new(
            0.0, 0.0, 1.0, 0.0,
        )),
        Vec3::new(0.0, 0.0, 90.31915304296272),
    );
    oblique_is_checked(
        0.5147874278629065,
        5.104592103941619,
        0.10587863935816388,
        pose,
    );
}

/// A rounded block under a top at 24.9° posed `(0, 77, -69)`, shard 2 of
/// the same run: the same refusal as
/// `an_oblique_fillet_at_a_shallow_tilt_far_from_the_origin_is_checked`.
#[test]
fn an_oblique_fillet_at_a_tilt_of_25_degrees_far_from_the_origin_is_checked() {
    let pose = Isometry::new(
        arris_math::nalgebra::Unit::new_unchecked(arris_math::nalgebra::Quaternion::new(
            0.0, 0.0, 1.0, 0.0,
        )),
        Vec3::new(0.0, 76.99460580419323, -68.83766176376402),
    );
    oblique_is_checked(
        0.3335513775210022,
        24.928118781146715,
        0.19908739964489544,
        pose,
    );
}

/// The fillet of the block's tilted edge, built and checked at `Full`
/// with nothing undecided.
fn oblique_is_checked(corner: f64, tilt: f64, r: f64, pose: Isometry) {
    let case = Oblique {
        length: 3.0,
        width: 1.5,
        corner,
        tilt: tilt.to_radians(),
        r,
        concave: false,
        pose,
    };
    let mut m = Model::default();
    let (part, at) = case.build(&mut m).unwrap();
    let edge = edge_near(&m, part, at).unwrap();
    let (blended, _) = fillet(&mut m, part, &[edge], case.r).unwrap();
    assert_checked(&check(&m, blended, Level::Full)).unwrap();
}
