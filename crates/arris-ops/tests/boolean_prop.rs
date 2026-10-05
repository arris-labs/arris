//! The booleans at random poses (ADR-0004): a
//! box and a cylinder, and two cylinders on parallel or crossing axes —
//! of one radius, or of two meeting in a traced, fitted quartic
//! (ADR-0018) — and a frustum, a ball, a ring or an elliptic prism
//! against a box or a cylinder, from `prop::body`, both operand orders — volume
//! and area additivity, the cut identity, commutativity of `fuse` and
//! `common`, results of several lumps held to the same identities
//! (ADR-0006), two runs dumping identically, a turn of the tool about its
//! own axis — which moves only its seam — changing nothing, and every
//! result clean at `Full` with nothing unchecked. A
//! failure
//! prints the shrunk pair and the seed, and becomes a fixture under
//! `tests/fixtures/boolean/` (`tests/fixtures/README.md` §Property-test
//! failures).

use arris_debug::prop::body::{
    Boxed, CrossingPair, Cylindrical, OverlappingPair, QuadricPair, QuadricSolid, QuadricTool,
    QuarticPair, SingularSlice, SingularSolid, TangentPair,
};
use arris_debug::testing::{REL, close_to, fail, fitted_rel};
use arris_debug::unmetered::{common, cut, fuse};
use arris_debug::unmetered::{mass_properties, transform};
use arris_debug::{dump_text, prop, prop_shards};
use arris_math::nalgebra::{Quaternion, UnitQuaternion};
use arris_math::{Axis, Frame, Isometry, Point3, UnitVec3, Vec3};

use arris_check::{Level, check};
use arris_ops::measure::MassProperties;
use arris_ops::{OpError, Reason};
use arris_topo::provenance::audit;
use arris_topo::{Body, Model, Provenance};
use proptest::prelude::*;

/// A boolean of two bodies: `fuse`, `common` or `cut`.
type Boolean = fn(&mut Model, Body, Body) -> Result<(Body, Provenance), OpError>;

/// [`arris_debug::testing::close_to`] at [`REL`].
fn close(a: f64, b: f64, floor: f64) -> bool {
    close_to(a, b, floor, REL)
}

/// `op(a, b)`, its result clean at `Full` with nothing unchecked, and its
/// mass properties.
fn run(
    m: &mut Model,
    name: &str,
    op: Boolean,
    a: Body,
    b: Body,
) -> Result<(Body, MassProperties), TestCaseError> {
    let result = op(m, a, b);
    let body = clean(m, name, result, a, b)?;
    let props = mass_properties(m, body).map_err(|e| fail(format!("{name}: measure: {e}")))?;
    Ok((body, props))
}

/// The body `result` holds, clean at `Full` with nothing unchecked and its
/// provenance audited against `a` and `b`.
fn clean(
    m: &Model,
    name: &str,
    result: Result<(Body, Provenance), OpError>,
    a: Body,
    b: Body,
) -> Result<Body, TestCaseError> {
    let (body, provenance) = result.map_err(|e| fail(format!("{name}: {e}")))?;
    let report = check(m, body, Level::Full);
    if !report.is_ok() || !report.unchecked().is_empty() {
        return Err(fail(format!("{name}: not clean at Full\n{report}")));
    }
    audit(m, &[a, b], body, &provenance).map_err(|e| fail(format!("{name}: provenance: {e}")))?;
    Ok(body)
}

/// The operands of `pair` in a fresh model, with their mass properties.
fn operands(
    pair: &OverlappingPair,
) -> Result<(Model, Body, Body, MassProperties, MassProperties), TestCaseError> {
    operands_by(|m| pair.build(m))
}

/// The operands `build` makes in a fresh model, with their mass
/// properties.
fn operands_by(
    build: impl FnOnce(&mut Model) -> Result<(Body, Body), OpError>,
) -> Result<(Model, Body, Body, MassProperties, MassProperties), TestCaseError> {
    let mut m = Model::default();
    let (a, b) = build(&mut m).map_err(fail)?;
    let pa = mass_properties(&m, a).map_err(fail)?;
    let pb = mass_properties(&m, b).map_err(fail)?;
    Ok((m, a, b, pa, pb))
}

/// `V(A ∪ B) + V(A ∩ B) = V(A) + V(B)`, and the same for the areas: the
/// boundary of the union and the boundary of the common partition the
/// two operands' boundaries between them.
fn assert_additive(
    union: &MassProperties,
    inter: &MassProperties,
    pa: &MassProperties,
    pb: &MassProperties,
) -> Result<(), TestCaseError> {
    assert_additive_to(union, inter, pa, pb, REL)
}

/// [`assert_additive`] to `rel`: [`fitted_rel`] where the union and the
/// common each fitted their own pcurves of the same sections.
fn assert_additive_to(
    union: &MassProperties,
    inter: &MassProperties,
    pa: &MassProperties,
    pb: &MassProperties,
    rel: f64,
) -> Result<(), TestCaseError> {
    let (v, s) = (pa.volume + pb.volume, pa.area + pb.area);
    prop_assert!(
        close_to(union.volume + inter.volume, v, v, rel),
        "V(A ∪ B) + V(A ∩ B) = {} + {}, V(A) + V(B) = {} + {}",
        union.volume,
        inter.volume,
        pa.volume,
        pb.volume
    );
    prop_assert!(
        close_to(union.area + inter.area, s, s, rel),
        "A(A ∪ B) + A(A ∩ B) = {} + {}, A(A) + A(B) = {} + {}",
        union.area,
        inter.area,
        pa.area,
        pb.area
    );
    Ok(())
}

/// `V(A − B) + V(A ∩ B) = V(A)`.
fn assert_cut_identity(
    diff: &MassProperties,
    inter: &MassProperties,
    pa: &MassProperties,
    pb: &MassProperties,
) -> Result<(), TestCaseError> {
    assert_cut_identity_to(diff, inter, pa, pb, REL)
}

/// [`assert_cut_identity`] to `rel`, as [`assert_additive_to`].
fn assert_cut_identity_to(
    diff: &MassProperties,
    inter: &MassProperties,
    pa: &MassProperties,
    pb: &MassProperties,
    rel: f64,
) -> Result<(), TestCaseError> {
    prop_assert!(
        close_to(
            diff.volume + inter.volume,
            pa.volume,
            pa.volume + pb.volume,
            rel
        ),
        "V(A − B) + V(A ∩ B) = {} + {}, V(A) = {}",
        diff.volume,
        inter.volume,
        pa.volume
    );
    Ok(())
}

prop_shards! {
    /// A cylinder that clears every edge of the box: every outcome is known
    /// by construction. `fuse`, `common` and `box − cylinder` are one clean
    /// shell each, `cylinder − box` two lumps of one solid — the cylinder's
    /// two ends — and all of them obey the identities.
    piercing_pairs_obey_every_identity_whatever_their_lumps
        [shard_0 shard_1 shard_2 shard_3]
        (pair) = prop::body::piercing_pair() => {
            let (mut m, a, b, pa, pb) = operands(&pair)?;
            let (_, union) = run(&mut m, "fuse(a, b)", fuse, a, b)?;
            let (_, inter) = run(&mut m, "common(a, b)", common, a, b)?;
            let (_, diff) = run(&mut m, "cut(a, b)", cut, a, b)?;
            assert_additive(&union, &inter, &pa, &pb)?;
            assert_cut_identity(&diff, &inter, &pa, &pb)?;
            let (ends, ends_props) = run(&mut m, "cut(b, a)", cut, b, a)?;
            prop_assert_eq!(
                m.shells(ends).map_err(fail)?.len(),
                2,
                "cut(b, a): the cylinder's two ends"
            );
            assert_cut_identity(&ends_props, &inter, &pb, &pa)?;
            Ok(())
        }
}

prop_shards! {
    /// Any overlapping pair, the wall free to cross the box's edges: `fuse`
    /// and `common` are clean and additive; `box − cylinder` is clean and
    /// obeys the identity whether it is one lump or several (a corner
    /// sliced off).
    overlapping_pairs_fuse_and_common_additively
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7
         shard_8 shard_9 shard_10 shard_11]
        (pair) = prop::body::overlapping_pair() => {
            let (mut m, a, b, pa, pb) = operands(&pair)?;
            let (_, union) = run(&mut m, "fuse(a, b)", fuse, a, b)?;
            let (_, inter) = run(&mut m, "common(a, b)", common, a, b)?;
            assert_additive(&union, &inter, &pa, &pb)?;
            let (_, diff) = run(&mut m, "cut(a, b)", cut, a, b)?;
            assert_cut_identity(&diff, &inter, &pa, &pb)?;
            Ok(())
        }
}

/// A cylinder touching a random face of the box from outside along a
/// ruling through the face's interior (plan step 11): `box − cylinder`
/// is the box — the same faces, edges and vertices by id, the same mass
/// properties — `common` is the designed `Empty`, and `fuse`, which
/// would keep the face and the wall touching along the contact, is the
/// designed `TangentContact`; the model is as it was after each
/// refusal. The touches land in both places — a box edge on the wall
/// where the ruling crosses the face's rim, a rim circle on the face
/// where the cylinder ends inside it — and neither paves anything.
#[test]
fn a_cylinder_tangent_to_a_box_face_leaves_the_box_and_shares_nothing() {
    prop::check(prop::body::tangent_pair(), |pair: TangentPair| {
        prop_assume!(!touch_at_the_face_rim(&pair));
        prop_assume!(!seam_on_the_touch(&pair));
        a_tangent_cylinder_leaves_the_box(&pair)
    });
}

/// The named exclusion of the property above: a touch within the two
/// bodies' tolerances of the touched face's own rim, where the cut
/// rebuilds the rim's vertex, edges and faces under new ids that it
/// should keep. Two ways in: a corner of the face on the wall — the face's
/// two edges there lie in the tangent plane, so a ruling within
/// `√(4 R tol)` of the corner puts it on the wall
/// (`a_touch_at_a_box_corner_keeps_its_ids`) — and an end of the touching
/// ruling within that reach of the face's edge, where the cap's rim
/// grazes the edge (`a_rim_grazing_the_face_edge_keeps_its_ids`). Lifted
/// by their fix (ADR-0024).
fn touch_at_the_face_rim(pair: &TangentPair) -> bool {
    let tol = arris_math::Precision::DEFAULT.tolerance().linear;
    let (min, max) = (pair.cuboid.min, pair.cuboid.max);
    let (j, k) = ((pair.face.0 + 1) % 3, (pair.face.0 + 2) % 3);
    let axis = &pair.cylinder.axis;
    let r = pair.cylinder.radius;
    let d = axis.direction.into_inner();
    let corner_on_wall = [(min, min), (min, max), (max, min), (max, max)]
        .iter()
        .any(|&(a, b)| {
            let mut corner = if pair.face.1 { max } else { min };
            corner[j] = a[j];
            corner[k] = b[k];
            let w = corner - axis.origin;
            let off_axis = (w - w.dot(&d) * d).norm();
            (off_axis - r).abs() <= 2.0 * tol
        });
    let reach = (4.0 * r * tol).sqrt();
    let end_at_edge = [0.0, pair.cylinder.height].iter().any(|&t| {
        let end = axis.at(t) - r * pair.normal();
        [j, k].iter().any(|&c| {
            let inside = end[c] >= min[c] - reach && end[c] <= max[c] + reach;
            let other = if c == j { k } else { j };
            let within = end[other] >= min[other] - reach && end[other] <= max[other] + reach;
            inside
                && within
                && ((end[c] - min[c]).abs() <= reach || (end[c] - max[c]).abs() <= reach)
        })
    });
    corner_on_wall || end_at_edge
}

/// The property above at 5000 cases on the fixed seed, once the corner
/// was excluded, shrunk: a cylinder of radius 0.5 whose end's rim touches
/// the face 1.25e-4 from its edge, within the `√(2 r tol)` the rim stays
/// on the face for. The cut is the box, with that edge's faces rebuilt
/// under new ids.
#[test]
#[ignore = "a cap's rim grazing a box edge rebuilds it under new ids (ADR-0024)"]
fn a_rim_grazing_the_face_edge_keeps_its_ids() {
    let half_turn = Isometry::new(
        UnitQuaternion::new_unchecked(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
        Vec3::zeros(),
    );
    let pair = TangentPair {
        cuboid: Boxed {
            min: Point3::new(-9.835693963533036, -8.255260674403228, -0.5),
            max: Point3::new(9.835693963533036, 8.255260674403228, 0.5),
            pose: half_turn,
        },
        cylinder: Cylindrical {
            axis: Axis::new(
                Point3::new(-1.1563076784374635, -8.255135824278263, -1.0),
                Vec3::new(-0.9396571492496195, 0.3421175848507037, 0.0),
            )
            .unwrap(),
            radius: 0.5,
            height: 14.286588459948725,
            pose: half_turn,
        },
        face: (2, false),
    };
    assert!(touch_at_the_face_rim(&pair), "the exclusion covers it");
    if let Err(e) = a_tangent_cylinder_leaves_the_box(&pair) {
        panic!("{e}");
    }
}

/// The second named exclusion: the touch along the cylinder's seam,
/// within the two bodies' tolerances — an axis along a box edge, whose
/// seam `Frame::from_z` puts on the side against the face. The cut fails
/// with `Fault::Split`, a section edge ending at a node nothing else
/// reaches (`regression/tangent-seam-on-face-cut`, ADR-0024); the fix lifts it.
fn seam_on_the_touch(pair: &TangentPair) -> bool {
    let tol = arris_math::Precision::DEFAULT.tolerance().linear;
    let axis = &pair.cylinder.axis;
    let Ok(frame) = Frame::from_z(axis.origin, axis.direction.into_inner()) else {
        return false;
    };
    let toward_face = -pair.normal();
    pair.cylinder.radius * (1.0 - frame.x().dot(&toward_face)) <= 2.0 * tol
}

fn a_tangent_cylinder_leaves_the_box(pair: &TangentPair) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let (a, b) = pair.build(&mut m).map_err(fail)?;
    let pa = mass_properties(&m, a).map_err(fail)?;
    let before = dump_text(&m, a).map_err(fail)?;
    let (body, _) = run(&mut m, "cut(box, cylinder)", cut, a, b)?;
    let faces = |body: Body| -> Result<Vec<_>, TestCaseError> {
        Ok(m.faces(body).map_err(fail)?.iter().map(|f| f.id).collect())
    };
    prop_assert_eq!(faces(body)?, faces(a)?, "every face of the box kept by id");
    let after = mass_properties(&m, body).map_err(fail)?;
    prop_assert!(close(after.volume, pa.volume, pa.volume));
    prop_assert!(close(after.area, pa.area, pa.area));
    prop_assert!((after.centroid - pa.centroid).norm() <= REL * pa.area.sqrt());
    match common(&mut m, a, b) {
        Err(OpError::Degenerate {
            reason: Reason::Empty,
            ..
        }) => {}
        Ok(_) => return Err(fail("common(box, cylinder): a touch shares no material")),
        Err(e) => return Err(fail(format!("common(box, cylinder): {e}"))),
    }
    match fuse(&mut m, a, b) {
        Err(OpError::Degenerate {
            reason: Reason::TangentContact,
            ..
        }) => {}
        Ok(_) => {
            return Err(fail(
                "fuse(box, cylinder): the face and the wall would share a slit",
            ));
        }
        Err(e) => return Err(fail(format!("fuse(box, cylinder): {e}"))),
    }
    prop_assert_eq!(
        dump_text(&m, a).map_err(fail)?,
        before,
        "the model is as it was"
    );
    Ok(())
}

/// The property above at 5000 cases on the fixed seed, shrunk: a ruling
/// passing 4.3e-4 from a corner of the touched face, a cylinder of radius
/// 5.05, so the corner is 1.9e-8 off the wall. The cut is the box, but
/// with the corner's vertex, edges and faces rebuilt under new ids.
#[test]
#[ignore = "a touch at a box corner rebuilds it under new ids (ADR-0024)"]
fn a_touch_at_a_box_corner_keeps_its_ids() {
    let half_turn = Isometry::new(
        UnitQuaternion::new_unchecked(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
        Vec3::zeros(),
    );
    let pair = TangentPair {
        cuboid: Boxed {
            min: Point3::new(-0.5, -0.7717329406756528, -1.961474021437454),
            max: Point3::new(0.5, 0.7717329406756528, 1.961474021437454),
            pose: half_turn,
        },
        cylinder: Cylindrical {
            axis: Axis::new(
                Point3::new(-5.549081524292037, -5.182674682392934, 8.433831597890485),
                Vec3::new(0.0, 0.5631140638662752, -0.8263791811729098),
            )
            .unwrap(),
            radius: 5.049081524292037,
            height: 18.8267736124392,
            pose: half_turn,
        },
        face: (0, false),
    };
    assert!(touch_at_the_face_rim(&pair), "the exclusion covers it");
    if let Err(e) = a_tangent_cylinder_leaves_the_box(&pair) {
        panic!("{e}");
    }
}

/// The dump with every number, and the sign in front of every id,
/// replaced by `#`, its lines sorted: two results that differ only in
/// which entity got which id, the order the faces were assembled in and
/// which way a section edge runs have the same text.
fn up_to_ids(dump: &str) -> Vec<String> {
    let mut lines: Vec<String> = dump
        .lines()
        .map(|line| {
            let mut out = String::with_capacity(line.len());
            let c: Vec<char> = line.chars().collect();
            let mut i = 0;
            while i < c.len() {
                let id_sign = (c[i] == '+' || c[i] == '-')
                    && c.get(i + 1).is_some_and(|x| "bsfevp".contains(*x))
                    && c.get(i + 2).is_some_and(char::is_ascii_digit);
                if id_sign {
                    i += 1;
                    continue;
                }
                let number = c[i].is_ascii_digit()
                    || (c[i] == '-' && c.get(i + 1).is_some_and(char::is_ascii_digit));
                if !number {
                    out.push(c[i]);
                    i += 1;
                    continue;
                }
                out.push('#');
                i += usize::from(c[i] == '-');
                while c.get(i).is_some_and(|x| x.is_ascii_digit() || *x == '.') {
                    i += 1;
                }
                if c.get(i) == Some(&'e') {
                    let mut j = i + 1;
                    j += usize::from(c.get(j).is_some_and(|x| *x == '-' || *x == '+'));
                    if c.get(j).is_some_and(char::is_ascii_digit) {
                        i = j;
                        while c.get(i).is_some_and(char::is_ascii_digit) {
                            i += 1;
                        }
                    }
                }
            }
            out
        })
        .collect();
    lines.sort();
    lines
}

/// Volume, area, centroid and inertia equal to `REL`.
fn assert_same_properties(
    x: &MassProperties,
    y: &MassProperties,
    what: &str,
) -> Result<(), TestCaseError> {
    assert_same_properties_to(x, y, what, REL)
}

/// The same to `rel`: [`fitted_rel`] where the two bodies were built
/// from different fittings of the same curves.
fn assert_same_properties_to(
    x: &MassProperties,
    y: &MassProperties,
    what: &str,
    rel: f64,
) -> Result<(), TestCaseError> {
    prop_assert!(
        close_to(x.volume, y.volume, 1.0, rel),
        "{what}: volumes {} and {}",
        x.volume,
        y.volume
    );
    prop_assert!(
        close_to(x.area, y.area, 1.0, rel),
        "{what}: areas {} and {}",
        x.area,
        y.area
    );
    let scale = x.centroid.coords.abs().max().max(1.0);
    prop_assert!(
        (x.centroid - y.centroid).norm() <= rel * scale,
        "{what}: centroids {} and {}",
        x.centroid,
        y.centroid
    );
    let scale = x.inertia.abs().max().max(1.0);
    prop_assert!(
        (x.inertia - y.inertia).abs().max() <= rel * scale,
        "{what}: inertia {} and {}",
        x.inertia,
        y.inertia
    );
    Ok(())
}

prop_shards! {
    /// `fuse(a, b)` and `fuse(b, a)`, `common(a, b)` and `common(b, a)`: the
    /// same mass properties, the same counts, the same dump up to ids.
    fuse_and_common_commute_at_random_poses
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7
         shard_8 shard_9 shard_10 shard_11 shard_12 shard_13 shard_14
         shard_15 shard_16 shard_17]
        (pair) = prop::body::overlapping_pair() => {
            for (name, op) in [("fuse", fuse as Boolean), ("common", common as Boolean)] {
                let (mut m, a, b, _, _) = operands(&pair)?;
                assert_commutes(&mut m, name, op, a, b)?;
            }
            Ok(())
        }
}

/// `op(a, b)` and `op(b, a)`: the same mass properties, the same counts,
/// the same dump up to ids.
fn assert_commutes(
    m: &mut Model,
    name: &str,
    op: Boolean,
    a: Body,
    b: Body,
) -> Result<(), TestCaseError> {
    let first = run(m, &format!("{name}(a, b)"), op, a, b)?;
    assert_commutes_with(m, name, op, first, a, b)
}

/// [`assert_commutes`] against `first`, the result of `op(a, b)` already
/// run and checked: only `op(b, a)` is run, and checked, here.
fn assert_commutes_with(
    m: &mut Model,
    name: &str,
    op: Boolean,
    first: (Body, MassProperties),
    a: Body,
    b: Body,
) -> Result<(), TestCaseError> {
    let (ab, pab) = first;
    let (ba, pba) = run(m, &format!("{name}(b, a)"), op, b, a)?;
    assert_same_properties(&pab, &pba, name)?;
    let (da, db) = (
        dump_text(m, ab).map_err(fail)?,
        dump_text(m, ba).map_err(fail)?,
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(m, ab).map_err(fail)?,
        arris_debug::dump::euler_line(m, ba).map_err(fail)?,
        "{}: counts",
        name
    );
    prop_assert_eq!(
        up_to_ids(&da),
        up_to_ids(&db),
        "{}: dumps\n{}\n{}",
        name,
        da,
        db
    );
    Ok(())
}

// -- coincident faces (plan step 10, `⚠ OPEN` 4) -----------------------

prop_shards! {
    /// `V((A − B) ∪ B) = V(A ∪ B)` and `V((A − B) ∪ (A ∩ B)) = V(A)`: every
    /// face of `A − B` that came from the tool is coincident with a face of
    /// `B` with the normals opposed, every face of `A ∩ B` is coincident
    /// with one of `A − B`, and the section edges of the first cut are
    /// common blocks of the fuse — the flush case at every pose the cut
    /// succeeds at. Both fuses are clean at `Full` and hold the union's
    /// counts where they are known.
    cut_then_fuse_restores_the_union_at_random_poses
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7
         shard_8 shard_9 shard_10 shard_11 shard_12 shard_13 shard_14
         shard_15 shard_16 shard_17 shard_18 shard_19 shard_20 shard_21
         shard_22 shard_23 shard_24 shard_25 shard_26 shard_27 shard_28
         shard_29]
        (pair) = prop::body::overlapping_pair() => {
            let (mut m, a, b, pa, _) = operands(&pair)?;
            let (diff, _) = cut(&mut m, a, b).map_err(|e| fail(format!("cut(a, b): {e}")))?;
            let (union, punion) = run(&mut m, "fuse(a, b)", fuse, a, b)?;
            let (inter, _) = run(&mut m, "common(a, b)", common, a, b)?;
            let (restored, prestored) = run(&mut m, "fuse(a − b, b)", fuse, diff, b)?;
            let rel = fitted_rel(&m, &punion);
            assert_same_properties_to(&prestored, &punion, "(a − b) ∪ b against a ∪ b", rel)?;
            prop_assert_eq!(
                arris_debug::dump::euler_line(&m, restored).map_err(fail)?,
                arris_debug::dump::euler_line(&m, union).map_err(fail)?,
                "(a − b) ∪ b: counts"
            );
            let (_, pwhole) = run(&mut m, "fuse(a − b, a ∩ b)", fuse, diff, inter)?;
            let rel = fitted_rel(&m, &pa);
            prop_assert!(
                close_to(pwhole.volume, pa.volume, pa.volume, rel),
                "V((a − b) ∪ (a ∩ b)) = {}, V(a) = {}",
                pwhole.volume,
                pa.volume
            );
            prop_assert!(
                close_to(pwhole.area, pa.area, pa.area, rel),
                "A((a − b) ∪ (a ∩ b)) = {}, A(a) = {}",
                pwhole.area,
                pa.area
            );
            Ok(())
        }
}

/// The first shrunk failure of the test above (seed and count in the
/// commit body): an oblique cylinder through an axis-aligned box. The
/// section ellipse of the cut is an edge of both `A − B` and `A ∩ B`,
/// and the two faces of `A − B` and `A ∩ B` on the box's face meet along
/// it as a common block; the fitted pcurves of that ellipse on the
/// cylinder differ between the two operands by more than the polygon
/// band, so the coincidence has to be decided by the curves.
#[test]
fn cut_then_fuse_of_an_oblique_cylinder_through_a_box() {
    let pair = OverlappingPair {
        cuboid: Boxed {
            min: Point3::new(-5.111718902382138, -9.737641865616778, -8.27137327424838),
            max: Point3::new(5.111718902382138, 9.737641865616778, 8.27137327424838),
            pose: Isometry::identity(),
        },
        cylinder: Cylindrical {
            axis: Axis::new(
                Point3::new(19.799757631166585, 37.59109796609268, 13.164435039577995),
                Vec3::new(
                    -0.5661663057675169,
                    -0.7883157354119573,
                    -0.2408609879483756,
                ),
            )
            .unwrap(),
            radius: 5.84425050754312,
            height: 82.56639984248808,
            pose: Isometry::identity(),
        },
    };
    let (mut m, a, b, pa, _) = operands(&pair).unwrap();
    let (diff, _) = cut(&mut m, a, b).unwrap();
    let (union, punion) = run(&mut m, "fuse(a, b)", fuse, a, b).unwrap();
    let (inter, _) = run(&mut m, "common(a, b)", common, a, b).unwrap();
    let (restored, prestored) = run(&mut m, "fuse(a − b, b)", fuse, diff, b).unwrap();
    let rel = fitted_rel(&m, &punion);
    assert_same_properties_to(&prestored, &punion, "(a − b) ∪ b against a ∪ b", rel).unwrap();
    assert_eq!(
        arris_debug::dump::euler_line(&m, restored).unwrap(),
        arris_debug::dump::euler_line(&m, union).unwrap()
    );
    let (_, pwhole) = run(&mut m, "fuse(a − b, a ∩ b)", fuse, diff, inter).unwrap();
    let rel = fitted_rel(&m, &pa);
    assert!(
        close_to(pwhole.volume, pa.volume, pa.volume, rel),
        "{} vs {}",
        pwhole.volume,
        pa.volume
    );
    assert!(
        close_to(pwhole.area, pa.area, pa.area, rel),
        "{} vs {}",
        pwhole.area,
        pa.area
    );
}

/// The second shrunk failure of the property above, the one that fixed
/// its bound (seed and count in the commit body): a cylinder across a
/// box a tenth of its length, at a pose 38 units from the origin. Every
/// count matched and the additivity identities held to 1e-15, but
/// `V((A − B) ∪ B)` and `V(A ∪ B)` differed by 1.02e-9 relative. The
/// section's pcurve on the cylinder is fitted once for the union and
/// again for the cut it is restored from, and the gap each fit leaves was
/// integrated about the origin, 38 away. `mass_properties` now takes its
/// first pass about the body's own vertices (ADR-0024), and the two agree to [`REL`].
#[test]
fn cut_then_fuse_of_a_cylinder_across_a_small_box() {
    let pose = Isometry::new(
        UnitQuaternion::from_quaternion(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
        Vec3::new(0.0, 0.0, -38.41144480853464),
    );
    let pair = OverlappingPair {
        cuboid: Boxed {
            min: Point3::new(-0.7606671317062929, -0.5, -0.5),
            max: Point3::new(0.7606671317062929, 0.5, 0.5),
            pose,
        },
        cylinder: Cylindrical {
            axis: Axis::new(
                Point3::new(2.4979061392527404, -1.3037770040355285, 0.959678290952288),
                Vec3::new(
                    -0.7701744682418742,
                    0.46517622691348676,
                    -0.4363970283845648,
                ),
            )
            .unwrap(),
            radius: 0.5775556717543766,
            height: 6.231381987111529,
            pose,
        },
    };
    let (mut m, a, b, _, _) = operands(&pair).unwrap();
    let (diff, _) = cut(&mut m, a, b).unwrap();
    let (union, punion) = run(&mut m, "fuse(a, b)", fuse, a, b).unwrap();
    let (restored, prestored) = run(&mut m, "fuse(a − b, b)", fuse, diff, b).unwrap();
    assert_eq!(
        arris_debug::dump::euler_line(&m, restored).unwrap(),
        arris_debug::dump::euler_line(&m, union).unwrap()
    );
    assert_same_properties_to(&prestored, &punion, "(a − b) ∪ b against a ∪ b", REL).unwrap();
}

// -- two cylinders on parallel or crossing axes ---------------------------

/// `op` over the operands `build` makes, in two fresh models: the same
/// dump, ids and all.
fn assert_deterministic(
    build: impl Fn(&mut Model) -> Result<(Body, Body), OpError>,
    name: &str,
    op: Boolean,
) -> Result<(), TestCaseError> {
    let once = || -> Result<String, TestCaseError> {
        let mut m = Model::default();
        let (a, b) = build(&mut m).map_err(fail)?;
        let (body, _) = op(&mut m, a, b).map_err(|e| fail(format!("{name}: {e}")))?;
        dump_text(&m, body).map_err(fail)
    };
    let (first, second) = (once()?, once()?);
    prop_assert_eq!(first, second, "{}: two runs", name);
    Ok(())
}

prop_shards! {
    /// Two parallel walls crossing in two rulings — the tool clear of both
    /// caps, a ruling on the target's seam, or the caps flush: `fuse`,
    /// `common` and both cuts clean at `Full` with nothing unchecked and
    /// their provenance audited, additive, the cut identity both ways
    /// whatever the lumps, `fuse` and `common` commuting, and every result
    /// dumping identically in a second run.
    parallel_cylinders_obey_every_identity
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (pair) = prop::body::parallel_pair() => {
            let (mut m, a, b, pa, pb) = operands_by(|m| pair.build(m))?;
            let (u, union) = run(&mut m, "fuse(a, b)", fuse, a, b)?;
            let (c, inter) = run(&mut m, "common(a, b)", common, a, b)?;
            let (_, diff) = run(&mut m, "cut(a, b)", cut, a, b)?;
            let (_, back) = run(&mut m, "cut(b, a)", cut, b, a)?;
            assert_additive(&union, &inter, &pa, &pb)?;
            assert_cut_identity(&diff, &inter, &pa, &pb)?;
            assert_cut_identity(&back, &inter, &pb, &pa)?;
            assert_commutes_with(&mut m, "fuse", fuse, (u, union), a, b)?;
            assert_commutes_with(&mut m, "common", common, (c, inter), a, b)?;
            for (name, op) in [
                ("fuse", fuse as Boolean),
                ("common", common as Boolean),
                ("cut", cut as Boolean),
            ] {
                assert_deterministic(|m| pair.build(m), name, op)?;
            }
            Ok(())
        }
}

prop_shards! {
    /// Two cylinders of one radius crossing at `ψ ∈ [30°, 90°]`, each
    /// through the other, a quarter with the tool's seam through a
    /// crossing vertex: `fuse` and `common` clean at `Full` with nothing
    /// unchecked and their provenance audited, additive, commuting and
    /// dumping identically in a second run, and the common the Steinmetz
    /// solid `16R³ / (3 sin ψ)` within `REL + tol·A/V`. Either cut is the designed
    /// `NonManifold`: the tool is as wide as the target, so the two lumps
    /// left meet at the crossing vertices (ADR-0006, `boolean/cross-cylinders-cut`).
    crossing_cylinders_obey_every_identity
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (pair) = prop::body::crossing_pair() => {
            let (mut m, a, b, pa, pb) = operands_by(|m| pair.build(m))?;
            let (u, union) = run(&mut m, "fuse(a, b)", fuse, a, b)?;
            let (c, inter) = run(&mut m, "common(a, b)", common, a, b)?;
            assert_additive(&union, &inter, &pa, &pb)?;
            // The section ellipses' pcurves are fitted within their edges'
            // tolerance, so the common's boundary lies within the largest
            // of those of the exact one, and its volume within that times
            // its area; additivity compares the same fits and holds to
            // `REL`.
            let steinmetz = pair.common_volume();
            let mut tolerance = m.precision().default_tolerance;
            for e in m.edges(c).map_err(fail)? {
                tolerance = tolerance.max(m.edge(e.id).map_err(fail)?.tolerance());
            }
            let rel = REL + tolerance * inter.area / inter.volume;
            prop_assert!(
                close_to(inter.volume, steinmetz, steinmetz, rel),
                "V(A ∩ B) = {}, 16R³ / (3 sin ψ) = {}, to {}",
                inter.volume,
                steinmetz,
                rel
            );
            for (name, target, tool) in [("cut(a, b)", a, b), ("cut(b, a)", b, a)] {
                match cut(&mut m, target, tool) {
                    Err(OpError::Degenerate {
                        reason: Reason::NonManifold,
                        ..
                    }) => {}
                    Ok(_) => {
                        return Err(fail(format!(
                            "{name}: the two lumps meet at the crossing vertices"
                        )));
                    }
                    Err(e) => return Err(fail(format!("{name}: {e}"))),
                }
            }
            assert_commutes_with(&mut m, "fuse", fuse, (u, union), a, b)?;
            assert_commutes_with(&mut m, "common", common, (c, inter), a, b)?;
            for (name, op) in [("fuse", fuse as Boolean), ("common", common as Boolean)] {
                assert_deterministic(|m| pair.build(m), name, op)?;
            }
            Ok(())
        }
}

/// `pair` with its tool turned about its own axis so that the seam lies
/// `theta` radians past the crossing vertex `(0, R, 0)` of `pair.a`'s
/// frame — `π` puts it through the other one, `(0, −R, 0)`. A turn about
/// its own axis is the identity on the tool as a set of points, so every
/// turn of one pair is the same pair of solids.
fn turned(pair: &CrossingPair, theta: f64) -> Result<CrossingPair, TestCaseError> {
    let d = pair.b.axis.direction.into_inner();
    let seam = Frame::from_z(Point3::origin(), d)
        .map_err(fail)?
        .x()
        .into_inner();
    // The seam's angle about `d` from `y`, both perpendicular to `d`.
    let to_y = seam.cross(&Vec3::y()).dot(&d).atan2(seam.dot(&Vec3::y()));
    let about = UnitQuaternion::from_axis_angle(&UnitVec3::new_normalize(d), to_y + theta);
    Ok(CrossingPair {
        b: Cylindrical {
            pose: Isometry::from_rotation(about).then(&pair.a.pose),
            ..pair.b
        },
        ..*pair
    })
}

prop_shards! {
    /// Where the tool's seam sits decides nothing (plans/
    /// seam-parametrisation-faults): two crossing cylinders fused, or
    /// intersected, with the tool turned about its own axis to a generic
    /// turn, with its seam through a crossing vertex, and twice beside
    /// one — once with the seam's touch of the other wall `R sin δ / sin ψ`
    /// from the vertex, `ψ` the angle between the axes, log-uniform from a
    /// tenth of a tolerance to 1.9, where the touch's ball and the
    /// vertex's meet and the touch joins it (`boolean/
    /// seam-a-tolerance-from-crossing-fuse`, ADR-0022), a tenth short of
    /// two so that no pose's rounding decides it; and once with `sin δ`
    /// log-uniform from ten tolerances over `R` to 0.03, which spans the
    /// bands where the seam's chord in the other wall is under the
    /// tolerance (the pave's touch, ADR-0016) and where
    /// the sliver it cuts off lies within it (the transversal rule) — are
    /// the same body at every turn: the same mass properties within what
    /// the fitted ellipses allow, and the counts of the generic turn but
    /// where the seam's touch joins the crossing vertex. There — at the
    /// turn through it and at the first turn beside it — they are the
    /// through turn's, fewer: two vertices and two edges in a `fuse`, and
    /// a face besides in a `common`, the seam's crossings with the
    /// ellipses being the crossing vertices themselves: the seam is
    /// topology, and where it runs is the one thing a turn may change.
    /// Between the two windows the touch stands for two crossings of its
    /// own a few tolerances from the vertex, and at some turns a `common`
    /// keeps the sliver between them, a face the polygons do not resolve:
    /// `regression/seam-two-tolerances-from-crossing-common`'s band, and
    /// not among them. Each result is measured moved back to the
    /// pair's own frame, the crossing of the axes at the origin: the
    /// boundary integral over fitted pcurves carries an error that grows
    /// with the distance from the origin and differs with where the seam
    /// cuts the ellipses — 4.6e-6 in a centroid 108 away where the same
    /// bodies at the origin agree to 1.4e-10 — which is the measurement's,
    /// not the turn's (`docs/BACKLOG.md`). The generic turn stays a tenth
    /// of a radian from both crossing vertices.
    a_turn_of_the_tool_about_its_own_axis_changes_nothing
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((pair, is_fuse, (generic, through), beside)) = (
            prop::body::crossing_pair(),
            any::<bool>(),
            (prop::finite_f64(0.1..=core::f64::consts::PI - 0.1), 0u8..4),
            [
                (prop::finite_f64(0.0..=1.0), 0u8..4),
                (prop::finite_f64(0.0..=1.0), 0u8..4),
            ],
        ) => {
            let (name, op) = if is_fuse {
                ("fuse", fuse as Boolean)
            } else {
                ("common", common as Boolean)
            };
            let tol = Model::default().precision().default_tolerance;
            let r = pair.a.radius;
            // Log-uniform bounds on `sin δ`: the touch joined to the
            // crossing vertex, then clear of it.
            let joined = pair.psi.sin() * tol / r;
            let windows = [
                ((0.1 * joined).log10(), (1.9 * joined).log10()),
                ((10.0 * tol / r).log10(), 0.03_f64.log10()),
            ];
            let pi = core::f64::consts::PI;
            // A quadrant: the side of the crossing vertex, and which one.
            let at = |quadrant: u8, delta: f64| {
                let delta = if quadrant & 1 == 1 { -delta } else { delta };
                if quadrant & 2 == 2 { pi + delta } else { delta }
            };
            // Each turn with the turn whose counts it has: `None` for the
            // generic one, `Some(true)` for the through one's.
            let mut turns = vec![(at(through, generic), None), (at(through, 0.0), None)];
            for (w, (&(x, quadrant), (lo, hi))) in beside.iter().zip(windows).enumerate() {
                let sin = 10f64.powf(lo + x * (hi - lo));
                turns.push((at(quadrant, sin.asin()), Some(w == 0)));
            }
            let mut first: Option<(String, MassProperties, f64)> = None;
            let mut through_counts: Option<String> = None;
            for (i, &(theta, like_through)) in turns.iter().enumerate() {
                let turned = turned(&pair, theta)?;
                let (mut m, a, b, _, _) = operands_by(|m| turned.build(m))?;
                let what = format!("{name} at a turn of {theta} rad");
                let (body, _) = run(&mut m, &what, op, a, b)?;
                let counts = arris_debug::dump::euler_line(&m, body).map_err(fail)?;
                // Measured in the pair's own frame: see the doc above.
                let (home, _) = transform(&mut m, body, &pair.a.pose.inverse()).map_err(fail)?;
                let props = mass_properties(&m, home).map_err(fail)?;
                if i == 1 {
                    through_counts = Some(counts.clone());
                }
                match &first {
                    None => first = Some((counts, props, fitted_rel(&m, &props))),
                    Some((c, p, rel)) => {
                        match like_through {
                            Some(false) => prop_assert_eq!(
                                &counts, c, "{}: counts against the generic turn", what
                            ),
                            Some(true) => prop_assert_eq!(
                                Some(&counts),
                                through_counts.as_ref(),
                                "{}: counts against the turn through the crossing vertex",
                                what
                            ),
                            None => {}
                        }
                        let rel = rel.max(fitted_rel(&m, &props));
                        assert_same_properties_to(&props, p, &what, rel)?;
                    }
                }
            }
            Ok(())
        }
}

// -- two cylinders meeting in a quartic (ADR-0018) ------------------------

/// `body`'s mass properties measured moved by `back`: a pair's results
/// measured in its own frame, near the origin. The boundary integral over
/// fitted pcurves, which meet their neighbours only within the edges'
/// tolerance, grows with the distance from the origin (`docs/BACKLOG.md`,
/// mass properties that do not depend on where the body is) — 1.2e-9
/// relative in a common's volume 25 away where the same bodies at the
/// origin agree to 2.6e-11 — which is the measurement's, not the
/// boolean's.
fn measured_at(
    m: &mut Model,
    body: Body,
    back: &Isometry,
) -> Result<MassProperties, TestCaseError> {
    let (home, _) = transform(m, body, back).map_err(fail)?;
    mass_properties(m, home).map_err(fail)
}

/// The quartic property over one pair.
fn quartic_identities(pair: &QuarticPair) -> Result<(), TestCaseError> {
    let back = pair.a.pose.inverse();
    let (mut m, a, b, _, _) = operands_by(|m| pair.build(m))?;
    let (pa, pb) = (
        measured_at(&mut m, a, &back)?,
        measured_at(&mut m, b, &back)?,
    );
    let (u, _) = run(&mut m, "fuse(a, b)", fuse, a, b)?;
    let (c, _) = run(&mut m, "common(a, b)", common, a, b)?;
    let (diff, _) = run(&mut m, "cut(a, b)", cut, a, b)?;
    let (back_cut, _) = run(&mut m, "cut(b, a)", cut, b, a)?;
    let union = measured_at(&mut m, u, &back)?;
    let inter = measured_at(&mut m, c, &back)?;
    // The union, the common and each cut fit their own pcurves of the
    // section.
    let rel = fitted_rel(&m, &union);
    assert_additive_to(&union, &inter, &pa, &pb, rel)?;
    let (pdiff, pback) = (
        measured_at(&mut m, diff, &back)?,
        measured_at(&mut m, back_cut, &back)?,
    );
    assert_cut_identity_to(&pdiff, &inter, &pa, &pb, rel)?;
    assert_cut_identity_to(&pback, &inter, &pb, &pa, rel)?;
    let default = m.precision().default_tolerance;
    for body in [u, c, diff, back_cut] {
        for e in m.edges(body).map_err(fail)? {
            let tolerance = m.edge(e.id).map_err(fail)?.tolerance();
            prop_assert!(tolerance == default, "{} grew to {}", e.id, tolerance);
        }
    }
    let (restored, _) = run(&mut m, "fuse(a − b, b)", fuse, diff, b)?;
    let prestored = measured_at(&mut m, restored, &back)?;
    assert_same_properties_to(&prestored, &union, "(a − b) ∪ b against a ∪ b", rel)?;
    prop_assert_eq!(
        arris_debug::dump::euler_line(&m, restored).map_err(fail)?,
        arris_debug::dump::euler_line(&m, u).map_err(fail)?,
        "(a − b) ∪ b: counts"
    );
    for (name, op, first) in [
        ("fuse", fuse as Boolean, u),
        ("common", common as Boolean, c),
    ] {
        let (other, _) = run(&mut m, &format!("{name}(b, a)"), op, b, a)?;
        let (pfirst, pother) = (
            measured_at(&mut m, first, &back)?,
            measured_at(&mut m, other, &back)?,
        );
        assert_same_properties(&pfirst, &pother, name)?;
        let (da, db) = (
            dump_text(&m, first).map_err(fail)?,
            dump_text(&m, other).map_err(fail)?,
        );
        prop_assert_eq!(
            up_to_ids(&da),
            up_to_ids(&db),
            "{}: dumps\n{}\n{}",
            name,
            da,
            db
        );
    }
    Ok(())
}

prop_shards! {
    /// Two cylinders of unequal radii on crossing axes, or on skew axes
    /// with the narrower through the wider or breaking out of its side:
    /// their walls meet in traced loops, fitted periodic NURBS
    /// (ADR-0018). `fuse`, `common` and both cuts clean at `Full` with
    /// nothing unchecked and their
    /// provenance audited, additive, the cut identity both ways whatever
    /// the lumps, `fuse` and `common` commuting, `(a − b) ∪ b` the union
    /// within what two fittings of the same loops allow and with its
    /// counts, and no edge of any result above its faces' tolerance —
    /// every body measured in the pair's own frame ([`measured_at`]).
    quartic_cylinders_obey_every_identity
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7
         shard_8 shard_9 shard_10 shard_11 shard_12 shard_13 shard_14
         shard_15]
        (pair) = prop::body::quartic_pair() => { quartic_identities(&pair) }
}

/// The singular property over one slice.
fn singular_identities(slice: &SingularSlice) -> Result<(), TestCaseError> {
    let back = slice.pose.inverse();
    let (mut m, a, b, _, _) = operands_by(|m| slice.build(m))?;
    let (pa, pb) = (
        measured_at(&mut m, a, &back)?,
        measured_at(&mut m, b, &back)?,
    );
    let (c, _) = run(&mut m, "common(a, b)", common, a, b)?;
    let (diff, _) = run(&mut m, "cut(a, b)", cut, a, b)?;
    let inter = measured_at(&mut m, c, &back)?;
    assert_cut_identity(&measured_at(&mut m, diff, &back)?, &inter, &pa, &pb)?;
    // Both keep the singular vertex, and the degenerate edge on it.
    for body in [c, diff] {
        let mut degenerate = 0;
        for e in m.edges(body).map_err(fail)? {
            degenerate += usize::from(m.edge(e.id).map_err(fail)?.is_degenerate());
        }
        prop_assert!(
            degenerate > 0,
            "no degenerate edge left\n{}",
            dump_text(&m, body).map_err(fail)?
        );
    }
    Ok(())
}

prop_shards! {
    /// A plane through a revolved cone's apex or a ball's pole, at a
    /// random pose (ADR-0021): the operand's singular vertex paves the
    /// section — two rulings ending on the apex, a circle through the
    /// pole — and the section's arrivals pave its degenerate edge. `common` and
    /// `cut` clean at `Full` with nothing unchecked and their provenance
    /// audited, together the solid in volume, each still holding a
    /// degenerate edge — measured in the pair's own frame
    /// ([`measured_at`]).
    a_plane_through_an_apex_or_a_pole_cuts_additively
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (slice) = prop::body::singular_slice() => { singular_identities(&slice) }
}

/// Two shrunk failures of the property above once its turn was drawn
/// beside the seam (seed and count in the commit body): a ball of radius
/// 0.5 turned over, sliced through its lower pole with the section
/// leaving it a few 1e-7 of a radian from the seam's meridian, so the
/// circle crosses the seam again 3.5e-7 and 7.7e-8 from the pole — the
/// seam's touch joining the pole's vertex in both. The body is built with
/// the section's arrival at the pole ended on the seam's corner of the
/// sphere's (u, v) box, where it came in past it inside the vertex's ball
/// (`Fault::Seam`, and L2's jump of 1.16e-7 at the corner, before).
#[test]
fn a_circle_through_a_pole_a_hair_off_the_seam_arrives_at_its_corner() {
    let q = |i, j, k, w| UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k));
    let turned_over = Isometry::new(q(0.0, 1.0, 0.0, 0.0), Vec3::zeros());
    for (block, tilt) in [
        (
            q(
                4.5060693312784737e-7,
                0.9836235643885107,
                -0.18023507865959792,
                -8.256733467680473e-8,
            ),
            0.36245087704211887,
        ),
        (
            q(
                0.9611716615602801,
                5.565711370811689e-8,
                -1.5979085870124377e-8,
                -0.2759511496867646,
            ),
            0.5591582703804245,
        ),
    ] {
        let slice = SingularSlice {
            solid: SingularSolid::Ball { radius: 0.5 },
            block: Boxed {
                min: Point3::new(-4.0, -4.0, 0.0),
                max: Point3::new(4.0, 4.0, 4.0),
                pose: Isometry::new(block, Vec3::new(0.0, 0.0, -0.5)),
            },
            tilt,
            pose: turned_over,
        };
        singular_identities(&slice).unwrap();
    }
}

/// Two shrunk failures of the property above with the turn drawn beside
/// the seam at a ball's tilt of 90° exactly (seed and count in the
/// commit body): the face's plane holds the axis 1.3e-7 of a radian off
/// the seam's meridian plane, so the seam lies within the tolerance of
/// the face over most of its length and the section, a great circle
/// through both poles, bounds a lune a few tolerances wide beside it.
/// Posed 93 from the origin, the seam's crossings of the face — at the
/// poles, placed by rounding over the angle to within 1e-6 of them — lie
/// 1.3e-7 from a pole and read as a section passing beside it
/// (`BesideSingularity`); at the origin they pave vertices 3e-7 from the
/// poles and the section leaves one along the seam, 1.3e-7 of `u`
/// beside it (`TangentContact`). The desired outcome is the half ball.
/// The strategy keeps the turn at 90° drawn round the whole turn; the
/// sliver faces are a backlog line (ADR-0022).
#[test]
#[ignore = "BesideSingularity and TangentContact for a meridian plane a hair off the seam's: the lune beside the seam, a sliver face (docs/BACKLOG.md)"]
fn a_meridian_plane_a_hair_off_the_seams_halves_the_ball() {
    let q = |i, j, k, w| UnitQuaternion::new_unchecked(Quaternion::new(w, i, j, k));
    let slice = |radius: f64, reach: f64, block: Isometry, pose: Isometry| SingularSlice {
        solid: SingularSolid::Ball { radius },
        block: Boxed {
            min: Point3::new(-reach, -reach, 0.0),
            max: Point3::new(reach, reach, reach),
            pose: block,
        },
        tilt: core::f64::consts::FRAC_PI_2,
        pose,
    };
    let radius = 5.827313694055293;
    for s in [
        slice(
            radius,
            23.309254776221174,
            Isometry::new(
                q(
                    0.31084389850611865,
                    0.12408923660051907,
                    0.670065071443858,
                    -0.6625637570470906,
                ),
                Vec3::new(5.622888875430533, -1.2288177833473812, 92.08918236910976),
            ),
            Isometry::new(
                q(
                    0.6444801692372493,
                    -0.4035454753701673,
                    0.6113486138805179,
                    -0.21920135281188774,
                ),
                Vec3::new(0.0, 0.0, 93.0006166447507),
            ),
        ),
        slice(
            radius,
            72.65819946186286,
            Isometry::new(
                q(
                    0.8911085450837674,
                    -0.35282950433601523,
                    0.2848169086567667,
                    -0.01778286512058247,
                ),
                Vec3::new(-3.054515193050281, -4.878197349998894, -0.9114342756409481),
            ),
            Isometry::new(
                q(
                    0.6444801692372494,
                    -0.40354547537016733,
                    0.0,
                    0.6494585135081319,
                ),
                Vec3::zeros(),
            ),
        ),
    ] {
        singular_identities(&s).unwrap();
    }
}

/// The first shrunk failure of the property above (seed, count and shard
/// in the commit body): a pipe breaking out of a wider cylinder's side at
/// 30°, posed. In `fuse(a − b, b)` the notch's loop edge crosses the
/// tool's seam 1.04e-7 from its own end vertex, a hair past that
/// vertex's tolerance, and the crossing merged into the section vertex
/// holding that end paved the end's vertex beside it: a sliver block the
/// tool's wall could not be split along (`SplitFault::Turn`).
#[test]
fn cut_then_fuse_of_a_posed_notch_paves_no_sliver() {
    let q = |w, i, j, k| UnitQuaternion::from_quaternion(Quaternion::new(w, i, j, k));
    let pair = QuarticPair {
        a: Cylindrical {
            axis: Axis::new(Point3::new(0.0, 0.0, -18.591497639375465), Vec3::z()).unwrap(),
            radius: 4.362154122057592,
            height: 37.18299527875093,
            pose: Isometry::new(
                q(
                    0.39735241229102636,
                    -0.8049770723501876,
                    0.055914839041806996,
                    0.43703146821705174,
                ),
                Vec3::zeros(),
            ),
        },
        b: Cylindrical {
            axis: Axis::new(
                Point3::new(-6.804960430409843, 4.754293292607629, -11.786537208965626),
                Vec3::new(0.49999999999999994, 0.0, 0.8660254037844387),
            )
            .unwrap(),
            radius: 1.3086462366172775,
            height: 27.219841721639376,
            pose: Isometry::new(
                q(
                    -0.38861564805113025,
                    0.8338430943856225,
                    0.08895622062190912,
                    -0.38179885129198093,
                ),
                Vec3::new(
                    -1.3736883297866176,
                    0.019778598734097874,
                    0.5950931799908585,
                ),
            ),
        },
        psi: 0.5235987755982988,
        offset: 4.754293292607629,
    };
    quartic_identities(&pair).unwrap();
}

/// A pair in the numbers a [`QuarticPair`] prints as: `a` on `z` from
/// `z0`, `b` from `origin` along `direction`, each pose as its rotation's
/// `[i, j, k, w]` and its translation.
fn printed_pair(
    (z0, ra, ha, qa, ta): (f64, f64, f64, [f64; 4], [f64; 3]),
    (origin, direction, rb, hb, qb, tb): ([f64; 3], [f64; 3], f64, f64, [f64; 4], [f64; 3]),
) -> QuarticPair {
    let pose = |q: [f64; 4], t: [f64; 3]| {
        Isometry::new(
            UnitQuaternion::from_quaternion(Quaternion::new(q[3], q[0], q[1], q[2])),
            Vec3::new(t[0], t[1], t[2]),
        )
    };
    let d = Vec3::new(direction[0], direction[1], direction[2]);
    QuarticPair {
        a: Cylindrical {
            axis: Axis::new(Point3::new(0.0, 0.0, z0), Vec3::z()).unwrap(),
            radius: ra,
            height: ha,
            pose: pose(qa, ta),
        },
        b: Cylindrical {
            axis: Axis::new(Point3::new(origin[0], origin[1], origin[2]), d).unwrap(),
            radius: rb,
            height: hb,
            pose: pose(qb, tb),
        },
        psi: d.z.acos(),
        offset: origin[1],
    }
}

/// The shrunk failure of the nightly of 2026-10-03
/// (`ARRIS_PROPTEST_SEED=56c7709c…`, 5000 cases, shard 13 of 16): a pipe
/// through a wider cylinder, the section fitted. The checker's E4 found the
/// pcurve of an edge on `f52` 1.00002e-7 off its curve at a sample the fit
/// had not verified, 2e-5 of the tolerance over it.
#[test]
fn a_fitted_section_pcurve_stays_within_its_edges_tolerance() {
    let pair = printed_pair(
        (
            -32.645288741979265,
            4.585967523682559,
            65.29057748395853,
            [
                0.0,
                0.5272031288669888,
                0.8270659522717962,
                -0.19498402884750365,
            ],
            [0.0; 3],
        ),
        (
            [-13.78745304751915, 3.1830378291452073, -18.857835694460114],
            [0.5902042911287625, 0.0, 0.8072539220921723],
            2.9129217959635363,
            46.720951557809656,
            [
                0.056683684682950986,
                0.6074548131731985,
                0.7276314401215783,
                -0.31358905782899443,
            ],
            [
                -0.6451723780053112,
                -0.9636808512452482,
                0.07514465143248916,
            ],
        ),
    );
    quartic_identities(&pair).unwrap();
}

/// The shrunk failures of the property above at 1000 cases (seed, count
/// and shards in the commit body), each a pipe breaking out of a wider
/// cylinder's side, posed. In `fuse(a − b, b)` the notch's loop edge
/// crosses the tool's seam at a shallow angle, where the two stay within
/// the tolerance of each other over a stretch longer than it: the two
/// planes the NURBS–line arm cuts the seam with each found the crossing,
/// more than the tolerance apart, two hits of one crossing; and the one
/// hit, where it was one, lay a hair past the tolerance of the vertex the
/// cut had made there. Both made a second vertex beside the first — a
/// sliver block (`SplitFault::Turn`), or a sliver face read as a tangent
/// contact.
#[test]
fn cut_then_fuse_across_a_shallow_seam_crossing() {
    for pair in [
        printed_pair(
            (
                -33.271785099980114,
                8.784080989788679,
                66.54357019996023,
                [0.8534056078886708, -0.5212474157481917, 0.0, 0.0],
                [0.0; 3],
            ),
            (
                [-18.73658712072356, 9.917425628099492, -14.535197979256553],
                [0.7901221055007523, 0.0, 0.6129494745891034],
                6.8297416108142865,
                47.42706726031656,
                [
                    0.768996988947286,
                    -0.6180635441690795,
                    0.08507851090243305,
                    -0.13929369455143564,
                ],
                [0.368980530659905, -2.572707340369413, 3.167632839953812],
            ),
        ),
        printed_pair(
            (
                -6.465142997646601,
                2.963504607715283,
                12.930285995293202,
                [0.8215254802359442, 0.5701718033392228, 0.0, 0.0],
                [0.0; 3],
            ),
            (
                [-4.623067188035841, 2.768975511689066, -1.8420758096107592],
                [0.9289713659270369, 0.0, 0.3701515923073348],
                0.8890513823145848,
                9.953088669040731,
                [
                    -0.8092333294483326,
                    -0.5854353545288876,
                    -0.027972515808736387,
                    -0.040303877442897165,
                ],
                [
                    -0.023346537933804104,
                    -0.10667794678265723,
                    -0.27131164504694527,
                ],
            ),
        ),
        printed_pair(
            (
                -24.010644021068433,
                4.490124734597612,
                48.02128804213687,
                [0.0, 1.0, 0.0, 0.0],
                [0.0; 3],
            ),
            (
                [-8.78850567293599, 3.7553340362184136, -15.222138348132445],
                [0.49999999999999994, 0.0, 0.8660254037844387],
                2.8336299928490463,
                35.15402269174397,
                [
                    0.10556022737653042,
                    0.9925435596796849,
                    -0.060945225691557935,
                    0.0,
                ],
                [-0.7869161560813056, 0.11158811459991469, 0.4543262545432073],
            ),
        ),
    ] {
        quartic_identities(&pair).unwrap();
    }
}

// -- a quadric or a torus against a box or a cylinder ---------------------

/// `op(a, b)` checked as [`clean`] does, or `None` where it is the
/// designed `BesideSingularity`: a tool wall passing a ball's pole closer
/// than the polygons of a pcurve can follow and not through it, refused by
/// name before anything is built (ADR-0021).
fn run_unless_beside(
    m: &mut Model,
    name: &str,
    op: Boolean,
    a: Body,
    b: Body,
) -> Result<Option<Body>, TestCaseError> {
    match op(m, a, b) {
        Err(OpError::Degenerate {
            reason: Reason::BesideSingularity,
            ..
        }) => Ok(None),
        // A fault one of the differential's named exclusions covers, as
        // `under_exclusions` treats the checker guard's panic: the
        // `Split` fault an elliptic prism cut and fused back by a pipe
        // reached on a nightly seed
        // (`an_elliptic_prism_cut_then_fused_back_by_a_pipe`).
        Err(e) if arris_debug::differential::exclusion_of_error(&e).is_some() => {
            Err(TestCaseError::reject(
                arris_debug::differential::exclusion_of_error(&e).map_or("", |x| x.name),
            ))
        }
        result => clean(m, name, result, a, b).map(Some),
    }
}

/// The quadric property over one pair; a pair any of whose booleans is
/// the designed `BesideSingularity` holds nothing further.
fn quadric_identities(pair: &QuadricPair) -> Result<(), TestCaseError> {
    let back = pair.pose.inverse();
    let (mut m, a, b, _, _) = operands_by(|m| pair.build(m))?;
    let (pa, pb) = (
        measured_at(&mut m, a, &back)?,
        measured_at(&mut m, b, &back)?,
    );
    let mut bodies = Vec::new();
    for (name, op, x, y) in [
        ("fuse(a, b)", fuse as Boolean, a, b),
        ("common(a, b)", common as Boolean, a, b),
        ("cut(a, b)", cut as Boolean, a, b),
        ("cut(b, a)", cut as Boolean, b, a),
        ("fuse(b, a)", fuse as Boolean, b, a),
        ("common(b, a)", common as Boolean, b, a),
    ] {
        match run_unless_beside(&mut m, name, op, x, y)? {
            Some(body) => bodies.push(body),
            None => return Ok(()),
        }
    }
    let [u, c, diff, back_cut, u_ba, c_ba] = bodies[..] else {
        return Err(fail("six booleans, six bodies"));
    };
    let union = measured_at(&mut m, u, &back)?;
    let inter = measured_at(&mut m, c, &back)?;
    // Every result fits its own pcurves of the sections: see the doc of
    // the property.
    let rel = fitted_rel(&m, &inter);
    assert_additive_to(&union, &inter, &pa, &pb, rel)?;
    let pdiff = measured_at(&mut m, diff, &back)?;
    assert_cut_identity_to(&pdiff, &inter, &pa, &pb, rel)?;
    let pback = measured_at(&mut m, back_cut, &back)?;
    assert_cut_identity_to(&pback, &inter, &pb, &pa, rel)?;
    for (name, first, other, props) in [("fuse", u, u_ba, &union), ("common", c, c_ba, &inter)] {
        let pother = measured_at(&mut m, other, &back)?;
        assert_same_properties_to(props, &pother, name, rel)?;
        let (da, db) = (
            dump_text(&m, first).map_err(fail)?,
            dump_text(&m, other).map_err(fail)?,
        );
        prop_assert_eq!(
            up_to_ids(&da),
            up_to_ids(&db),
            "{}: dumps\n{}\n{}",
            name,
            da,
            db
        );
    }
    let Some(restored) = run_unless_beside(&mut m, "fuse(a − b, b)", fuse, diff, b)? else {
        return Ok(());
    };
    let prestored = measured_at(&mut m, restored, &back)?;
    assert_same_properties_to(&prestored, &union, "(a − b) ∪ b against a ∪ b", rel)?;
    prop_assert_eq!(
        arris_debug::dump::euler_line(&m, restored).map_err(fail)?,
        arris_debug::dump::euler_line(&m, u).map_err(fail)?,
        "(a − b) ∪ b: counts"
    );
    Ok(())
}

prop_shards! {
    /// A frustum, a ball, a ring or an elliptic prism — the faces a
    /// revolve, a fillet, a chamfer or an extruded ellipse leaves — and a
    /// box or a cylinder through its interior, at a random pose
    /// (C3's accept line, docs/ROADMAP.md): `fuse`, `common` and both cuts clean at
    /// `Full` with nothing unchecked and their provenance audited,
    /// additive, the cut identity both ways whatever the lumps, `fuse` and
    /// `common` commuting to the dump, and `(a − b) ∪ b` the union with its
    /// counts — every body measured in the pair's own frame
    /// ([`measured_at`]). A tool wall passing a ball's pole is the designed
    /// `BesideSingularity`, and the pair holds nothing further. No pair
    /// is spared an identity: where the restoring fuse traces a section
    /// over another region than the cut, the cut's section edge is along
    /// the fuse's own by the surfaces, never by two splines compared
    /// (`boolean/frustum-stub-cut-then-fuse`).
    ///
    /// Every identity is held to [`fitted_rel`], not `REL`: a section on a
    /// sphere, a torus, a cone or an elliptic cylinder has a fitted pcurve
    /// there, which each result fits for itself. Measured on the three
    /// shrunk cases `REL` failed — a pipe through an elliptic prism,
    /// obliquely and across it, and a slab through a ring — additivity was
    /// 6.4e-9, 2.3e-9 and 1.4e-9 off at the default tolerance of 1e-7 and
    /// 5e-12, 5e-12 and 5e-11 at 1e-9: the fits', scaling with the
    /// tolerance, and a tenth of the bound.
    quadric_operands_obey_every_identity
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7
         shard_8 shard_9 shard_10 shard_11 shard_12 shard_13 shard_14
         shard_15]
        (pair) = prop::body::quadric_pair() => {
            under_exclusions(|| quadric_identities(&pair))
        }
}

/// `f`, with a panic that one of the differential's named exclusions
/// covers turned into a rejected case: the debug build's checker guard
/// reporting a defect of the list's, which a thin tool through a frustum
/// or an elliptic prism once reached in eleven shards of
/// `quadric_operands_obey_every_identity` at 5000 cases on the fixed seed
/// (`a_thin_slab_through_a_frustum_passes_the_checker`, ADR-0024). One list, `differential::EXCLUSIONS`,
/// so the fix that lifts it there lifts it here; any other panic fails as
/// before.
fn under_exclusions(f: impl FnOnce() -> Result<(), TestCaseError>) -> Result<(), TestCaseError> {
    match std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)) {
        Ok(result) => result,
        Err(payload) => match arris_debug::differential::exclusion_of_panic(&*payload) {
            Some(exclusion) => Err(TestCaseError::reject(exclusion.name)),
            None => std::panic::resume_unwind(payload),
        },
    }
}

/// `quadric_operands_obey_every_identity` at 5000 cases on the fixed seed,
/// shard 12 of 16, shrunk: a frustum of radii 0.1 and 0.5, 0.2 tall and
/// turned half a turn about y, and a slab 0.04 thick through it. One of
/// the six booleans' output failed the checker's L4 — a hole loop outside
/// every outer loop, a circle's coarse polygon leaving out a hole near its
/// rim — which the debug build's guard turned into a panic.
#[test]
fn a_thin_slab_through_a_frustum_passes_the_checker() {
    let half_turn = Isometry::new(
        UnitQuaternion::new_unchecked(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
        Vec3::zeros(),
    );
    let pair = QuadricPair {
        solid: QuadricSolid::Frustum {
            bottom: 0.1,
            top: 0.5,
            height: 0.2,
        },
        tool: QuadricTool::Box(Boxed {
            min: Point3::new(
                -0.020000000000000004,
                -0.673145600891813,
                -0.1710401140284179,
            ),
            max: Point3::new(0.020000000000000004, 0.673145600891813, 0.1710401140284179),
            pose: Isometry::new(
                UnitQuaternion::new_unchecked(Quaternion::new(
                    0.35469347939734847,
                    -0.7371988246533198,
                    -0.46908439236099336,
                    0.33270145993981504,
                )),
                Vec3::new(
                    0.020280696329477547,
                    -0.3195423506272533,
                    -0.1747667029906671,
                ),
            ),
        }),
        pose: half_turn,
    };
    if let Err(e) = quadric_identities(&pair) {
        panic!("{e}");
    }
}

/// `quadric_operands_obey_every_identity` at 5000 cases on the nightly
/// seed `9492b872…05f880c5`, shard 7 of 16, shrunk: an elliptic prism of
/// semi-axes 4.6 and 2.4, 9.1 tall, and a pipe of radius 1.7 across it
/// nearly along y, both turned half a turn about y. The cut and the five
/// other booleans build; fusing the pipe back into the cut fails with
/// `Fault::Split` — a cycle of a face's arrangement that does not turn
/// once — which `differential::EXCLUSIONS` `split-fault` covers.
#[test]
#[ignore = "Fault::Split, a face arrangement that is not a subdivision (docs/BACKLOG.md, the differential's findings; differential::EXCLUSIONS split-fault)"]
fn an_elliptic_prism_cut_then_fused_back_by_a_pipe() {
    let half_turn = Isometry::new(
        UnitQuaternion::new_unchecked(Quaternion::new(0.0, 0.0, 1.0, 0.0)),
        Vec3::zeros(),
    );
    let pair = QuadricPair {
        solid: QuadricSolid::EllipticPrism {
            a: 4.618723547468061,
            b: 2.4184486698938907,
            height: 9.116543533344732,
        },
        tool: QuadricTool::Cylinder(Cylindrical {
            axis: Axis {
                origin: Point3::new(0.9003270109670206, -2.5373774203492996, 0.9200294783571933),
                direction: UnitVec3::new_unchecked(Vec3::new(
                    0.09653684119572423,
                    0.995320198470987,
                    -0.004282616913362748,
                )),
            },
            radius: 1.6849079765193715,
            height: 22.395176041983547,
            pose: half_turn,
        }),
        pose: half_turn,
    };
    if let Err(e) = quadric_identities(&pair) {
        panic!("{e}");
    }
}

/// An isometry in the numbers one prints as: the rotation's `[i, j, k, w]`
/// and the translation.
fn printed_pose(q: [f64; 4], t: [f64; 3]) -> Isometry {
    Isometry::new(
        UnitQuaternion::new_unchecked(Quaternion::new(q[3], q[0], q[1], q[2])),
        Vec3::new(t[0], t[1], t[2]),
    )
}

/// A cylinder tool in the numbers a [`QuadricTool`] prints as.
fn printed_pipe(
    origin: [f64; 3],
    direction: [f64; 3],
    radius: f64,
    height: f64,
    pose: Isometry,
) -> QuadricTool {
    QuadricTool::Cylinder(Cylindrical {
        axis: Axis {
            origin: Point3::new(origin[0], origin[1], origin[2]),
            direction: UnitVec3::new_unchecked(Vec3::new(direction[0], direction[1], direction[2])),
        },
        radius,
        height,
        pose,
    })
}

/// The shrunk failure of the nightly of 2026-10-03
/// (`ARRIS_PROPTEST_SEED=3549c46a…`, 5000 cases, shard 8 of 16): a pipe
/// through a cylinder, both posed 94 from the origin. The checker's E4
/// found a section edge's pcurve 1.0027e-7 off its curve.
#[test]
fn a_pipe_through_a_cylinder_far_from_the_origin_passes_the_checker() {
    let pair = printed_pair(
        (
            -16.553867792102892,
            3.90639191699275,
            33.107735584205784,
            [0.0, 1.0, 0.0, 0.0],
            [94.26147434370276, 3.6420629033563987, 0.0],
        ),
        (
            [-7.26705601343576, 0.8681143753815194, -9.286811778667133],
            [0.6162617888954505, 0.0, 0.7875413687847638],
            2.149488094203717,
            23.584314797322683,
            [
                0.7769368889889022,
                0.16355208099404178,
                -0.6079636398097872,
                0.0,
            ],
            [94.04085237271289, 5.331848779569667, 0.1726396807848398],
        ),
    );
    quartic_identities(&pair).unwrap();
}

/// The shrunk failure of the nightly of 2026-10-02
/// (`ARRIS_PROPTEST_SEED=29a5102c…`, 5000 cases, shard 2 of 16): a frustum
/// and a pipe through it, posed 60 from the origin. The checker's E4 found
/// a section edge's pcurve 1.00036e-7 off its curve (the numbers are the
/// whole shard's final shrink; the first one printed passes).
#[test]
fn a_pipe_through_a_frustum_far_from_the_origin_passes_the_checker() {
    let pose = printed_pose(
        [
            0.0352982588521497,
            0.9275792477249976,
            0.0,
            0.3719553361788676,
        ],
        [59.55015422791381, 0.0, 0.0],
    );
    let pair = QuadricPair {
        solid: QuadricSolid::Frustum {
            bottom: 1.358969708425192,
            top: 3.339156998036633,
            height: 7.596003427461487,
        },
        tool: printed_pipe(
            [-1.9323479265652481, -1.742746103016946, 3.440097701893266],
            [0.6238825555963347, 0.7784403386805128, 0.0692906627010817],
            1.0802719024055645,
            17.688794783712012,
            pose,
        ),
        pose,
    };
    quadric_identities(&pair).unwrap();
}

/// The shrunk failure of the nightly of 2026-10-03
/// (`ARRIS_PROPTEST_SEED=3549c46a…`, 5000 cases, shard 4 of 16): a thin
/// elliptic prism and a pipe across it, posed 86 up. `cut(b, a)` returns a
/// body whose shells do not nest — a void inside no shell.
#[test]
fn a_pipe_across_a_thin_elliptic_prism_cuts_to_nested_shells() {
    let pose = printed_pose(
        [0.0, 0.867861247426458, 0.0, 0.4968066577808638],
        [0.0, 0.0, 86.05926964904897],
    );
    let pair = QuadricPair {
        solid: QuadricSolid::EllipticPrism {
            a: 0.5,
            b: 0.11329688561452053,
            height: 1.494827150200483,
        },
        tool: printed_pipe(
            [0.165636270934527, 0.039286320986688016, 0.29589288919933354],
            [
                -0.2775229599973419,
                -0.5142221887195153,
                -0.8115149704736352,
            ],
            0.07974837592678169,
            3.332880428915773,
            pose,
        ),
        pose,
    };
    quadric_identities(&pair).unwrap();
}

/// The shrunk failure of the nightly of 2026-09-30
/// (`ARRIS_PROPTEST_SEED=9f2abc6e…`, 5000 cases, shard 4 of 16): a ring cut
/// by a long thin box posed through it. `cut(b, a)` left a shell of a
/// hair's volume that read as a void inside no shell.
#[test]
fn a_ring_cut_by_a_posed_box_leaves_no_void_inside_no_shell() {
    let pair = QuadricPair {
        solid: QuadricSolid::Ring {
            major: 9.855105457570687,
            minor: 3.735277781142768,
        },
        tool: QuadricTool::Box(Boxed {
            min: Point3::new(-2.586165151881783, -17.488251608027525, -7.252358415400309),
            max: Point3::new(2.586165151881783, 17.488251608027525, 7.252358415400309),
            pose: printed_pose(
                [
                    0.3894803974308203,
                    0.31833458087937805,
                    -0.3737087044120897,
                    -0.7793009167709984,
                ],
                [17.014790695252074, 48.473340691733384, -7.758557463181772],
            ),
        }),
        pose: printed_pose(
            [
                0.0,
                0.810749343296628,
                -0.5519774068048309,
                0.19495241655619694,
            ],
            [21.24868463442002, 48.68403360657203, 0.0],
        ),
    };
    quadric_identities(&pair).unwrap();
}

/// The shrunk failures of `quartic_cylinders_obey_every_identity` on the
/// nightlies of 2026-10-01 (`ARRIS_PROPTEST_SEED=e016178c…`, shard 13 of
/// 16: volumes additive to 1.1e-9 where `REL` is 1e-9) and 2026-10-02
/// (`29a5102c…`, shard 6: areas additive to 1.5e-9), a pipe a hair
/// narrower than its wall's offset breaking out of a cylinder's side. The
/// union and the common each fit their own pcurves of the one section,
/// each to within the edge's tolerance, so the identity holds to the
/// fit's reach over the body (`fitted_rel`), not to `REL`: the misses are
/// a tenth of what that bound allows.
#[test]
fn a_pipe_breaking_out_of_a_cylinders_side_is_additive_to_the_fit() {
    for pair in [
        printed_pair(
            (
                -1.185754057221079,
                0.5,
                2.371508114442158,
                [0.0, 1.0, 0.0, 0.0],
                [0.0; 3],
            ),
            (
                [
                    -1.0752037844916815,
                    0.3773224747891321,
                    -0.11055027272939752,
                ],
                [0.994755775496313, 0.0, 0.10227877158398482],
                0.39600315374306794,
                2.161744241103261,
                [
                    0.026571389565977554,
                    0.9656641172739785,
                    -0.2584313716753464,
                    0.0,
                ],
                [
                    -0.019363463022661453,
                    0.05093311016856722,
                    0.1883276106771102,
                ],
            ),
        ),
        printed_pair(
            (
                -1.077813894843341,
                0.5,
                2.155627789686682,
                [0.0, 1.0, 0.0, 0.0],
                [0.0; 3],
            ),
            (
                [
                    -0.9600050682715803,
                    0.3013927975008962,
                    -0.11780882657176053,
                ],
                [0.9925542752487733, 0.0, 0.12180316369200966],
                0.30000422355965023,
                1.9344132451214622,
                [
                    0.01342989330706866,
                    0.9939028906438282,
                    -0.1094380278312946,
                    0.0,
                ],
                [
                    -0.00804598801668432,
                    0.007328090982824442,
                    0.06556545464413344,
                ],
            ),
        ),
    ] {
        quartic_identities(&pair).unwrap();
    }
}
