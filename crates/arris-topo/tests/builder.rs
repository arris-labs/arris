//! The builder and the Euler operators (ADR-0002): every operator keeps
//! the Euler line at zero, every operator
//! followed by its inverse restores the builder's dump (a property over
//! random small sequences), an operator that refuses leaves the builder
//! untouched, and `finish` refuses what cannot be stored while the
//! transaction leaves nothing behind. Topology only: the geometry ids are
//! dummies that never resolve, since nothing here reaches `finish`
//! except the tests of `finish` itself.

use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Interval, Point2, Point3, Vec2, Vec3};
use arris_topo::builder::{
    BuildError, Builder, EdgeRef, FaceRef, Join, Position, Seed, Split, Strut, VertexRef,
};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::{Curve2Id, CurveId, Model, Orientation, SurfaceId, VertexId};
use proptest::prelude::*;

fn curve(k: u32) -> EdgeGeometry {
    EdgeGeometry::Curve {
        curve: CurveId::new(k, 0),
        range: Interval::UNIT,
    }
}

fn pcurve(k: u32) -> Option<Curve2Id> {
    Some(Curve2Id::new(k, 0))
}

fn surface(k: u32) -> SurfaceId {
    SurfaceId::new(k, 0)
}

fn point(k: u32) -> Point3 {
    Point3::new(f64::from(k), 0.0, 0.0)
}

fn seed() -> Seed {
    Seed {
        point: point(0),
        surface: surface(0),
        orientation: Orientation::Forward,
    }
}

fn strut(k: u32) -> Strut {
    Strut {
        point: point(k),
        geometry: curve(k),
        pcurves: [pcurve(2 * k), pcurve(2 * k + 1)],
    }
}

fn split(k: u32, s: u32, orientation: Orientation) -> Split {
    Split {
        geometry: curve(k),
        surface: surface(s),
        orientation,
        pcurves: [pcurve(2 * k), pcurve(2 * k + 1)],
    }
}

fn join(k: u32) -> Join {
    Join {
        geometry: curve(k),
        pcurves: [pcurve(2 * k), pcurve(2 * k + 1)],
    }
}

fn assert_euler(b: &Builder, what: &str) {
    let c = b.counts();
    assert_eq!(c.euler(), 0, "after {what}: {c}\n{}", b.dump());
}

/// The cylinder's topology: seed, a closed edge splitting off the wall, a
/// strut for the seam, a closed edge splitting off the top cap.
struct Cylinder {
    v0: VertexRef,
    v1: VertexRef,
    e_seam: EdgeRef,
    e_top: EdgeRef,
    f_bottom: FaceRef,
    f_wall: FaceRef,
    f_top: FaceRef,
}

fn cylinder(b: &mut Builder) -> Cylinder {
    let (v0, f_bottom) = b.mvfs(seed()).unwrap();
    assert_euler(b, "mvfs");
    let at = Position::new(f_bottom, 0, 0);
    let (e_bottom, f_wall) = b.mef(at, at, split(0, 1, Orientation::Forward)).unwrap();
    assert_euler(b, "mef (bottom circle)");
    let at = b.find_position(f_wall, 0, v0).unwrap();
    let (v1, e_seam) = b.mev(at, strut(1)).unwrap();
    assert_euler(b, "mev (seam)");
    let at = b.find_position(f_wall, 0, v1).unwrap();
    let (e_top, f_top) = b.mef(at, at, split(2, 2, Orientation::Forward)).unwrap();
    assert_euler(b, "mef (top circle)");
    assert_eq!(
        (e_bottom, e_seam, e_top),
        (EdgeRef::new(0), EdgeRef::new(1), EdgeRef::new(2))
    );
    Cylinder {
        v0,
        v1,
        e_seam,
        e_top,
        f_bottom,
        f_wall,
        f_top,
    }
}

#[test]
fn the_cylinder_keeps_the_euler_line_at_zero_and_has_the_seam_twice_in_the_wall() {
    let mut b = Builder::new(1e-7);
    let Cylinder {
        v0,
        v1,
        f_bottom,
        f_wall,
        f_top,
        ..
    } = cylinder(&mut b);
    assert_eq!(b.counts().to_string(), "2/3/3/3/1 g0 = 0");
    let wall: Vec<String> = b.face(f_wall).unwrap().loops()[0]
        .uses()
        .iter()
        .map(|u| format!("{}{}", u.orientation, u.edge))
        .collect();
    assert_eq!(
        wall,
        ["+e0", "+e1", "-e2", "-e1"],
        "bottom, seam up, top, seam down"
    );
    assert_eq!(
        b.face(f_bottom).unwrap().loops()[0].uses()[0].orientation,
        Orientation::Reversed
    );
    assert_eq!(
        b.face(f_top).unwrap().loops()[0].uses()[0].orientation,
        Orientation::Forward
    );
    // The seam vertices occur twice in the wall's loop: a position is needed.
    assert!(matches!(
        b.find_position(f_wall, 0, v0),
        Err(BuildError::Ambiguous { positions, .. }) if positions == [0, 1]
    ));
    assert!(matches!(
        b.find_position(f_wall, 0, v1),
        Err(BuildError::Ambiguous { positions, .. }) if positions == [2, 3]
    ));
    assert_eq!(
        b.vertex_at(Position::new(f_wall, 0, 4)).unwrap(),
        v0,
        "index n is index 0"
    );
    assert!(matches!(
        b.vertex_at(Position::new(f_wall, 0, 5)),
        Err(BuildError::BadPosition { len: 4, .. })
    ));
}

/// A box by Mäntylä's recipe: a rectangle of struts closed by `mef`, four
/// struts up from its corners, four `mef`s for the sides; then a frame by
/// a bridge strut into the top, a rectangle of struts, `mef` for the plug,
/// `kemr` on the bridge, four struts down, four `mef`s for the walls and
/// `kfmrh` to open the floor into the bottom.
#[test]
fn a_box_and_then_a_frame_keep_the_euler_line_at_zero() {
    let mut b = Builder::new(1e-7);
    let (v0, f0) = b.mvfs(seed()).unwrap();
    let mut k = 0;
    let mut next = || {
        k += 1;
        k
    };
    // Three struts from the seed loop, then the closing mef.
    let (v1, _) = b.mev(Position::new(f0, 0, 0), strut(next())).unwrap();
    let (v3, _) = b
        .mev(b.find_position(f0, 0, v1).unwrap(), strut(next()))
        .unwrap();
    let (v2, _) = b
        .mev(b.find_position(f0, 0, v3).unwrap(), strut(next()))
        .unwrap();
    assert_euler(&b, "three struts");
    let from = b.find_position(f0, 0, v2).unwrap();
    let to = b.find_position(f0, 0, v0).unwrap();
    let (_, f1) = b
        .mef(from, to, split(next(), 1, Orientation::Forward))
        .unwrap();
    assert_euler(&b, "closing mef");
    assert_eq!(b.counts().to_string(), "4/4/2/2/1 g0 = 0");
    // Four struts up from f1's loop, then four sides.
    let corners = [v0, v1, v3, v2];
    let mut tops = Vec::new();
    for &c in &corners {
        let at = b.find_position(f1, 0, c).unwrap();
        let (t, _) = b.mev(at, strut(next())).unwrap();
        tops.push(t);
        assert_euler(&b, "strut up");
    }
    for i in 0..4 {
        let from = b.find_position(f1, 0, tops[(i + 1) % 4]).unwrap();
        let to = b.find_position(f1, 0, tops[i]).unwrap();
        b.mef(from, to, split(next(), 2 + i as u32, Orientation::Forward))
            .unwrap();
        assert_euler(&b, "side mef");
    }
    assert_eq!(b.counts().to_string(), "8/12/6/6/1 g0 = 0");
    for (_, f) in b.faces() {
        assert_eq!(f.loops().len(), 1);
        assert_eq!(f.loops()[0].uses().len(), 4);
    }
    let box_dump = b.dump();

    // The frame: a bridge from a top corner into the hole's rim.
    let at = b.find_position(f1, 0, tops[0]).unwrap();
    let (h0, bridge) = b.mev(at, strut(next())).unwrap();
    let (h1, _) = b
        .mev(b.find_position(f1, 0, h0).unwrap(), strut(next()))
        .unwrap();
    let (h2, _) = b
        .mev(b.find_position(f1, 0, h1).unwrap(), strut(next()))
        .unwrap();
    let (h3, _) = b
        .mev(b.find_position(f1, 0, h2).unwrap(), strut(next()))
        .unwrap();
    assert_euler(&b, "rim struts");
    // The plug: from h3 to the first occurrence of h0 (before h0→h1).
    let from = b.find_position(f1, 0, h3).unwrap();
    let Err(BuildError::Ambiguous { positions, .. }) = b.find_position(f1, 0, h0) else {
        panic!("h0 is on the loop twice");
    };
    let to = Position::new(f1, 0, positions[0]);
    let (_, plug) = b
        .mef(from, to, split(next(), 0, Orientation::Reversed))
        .unwrap();
    assert_euler(&b, "plug mef");
    assert_eq!(b.face(plug).unwrap().loops()[0].uses().len(), 4);
    let (_, _, _) = b.kemr(bridge).unwrap();
    assert_euler(&b, "kemr bridge");
    assert_eq!(
        b.face(f1).unwrap().loops().len(),
        2,
        "the top has a ring now"
    );
    // Four struts down from the plug, four walls.
    let rim = [h0, h1, h2, h3];
    let mut floor = Vec::new();
    for &h in &rim {
        let at = b.find_position(plug, 0, h).unwrap();
        let (g, _) = b.mev(at, strut(next())).unwrap();
        floor.push(g);
        assert_euler(&b, "strut down");
    }
    for i in 0..4 {
        let from = b.find_position(plug, 0, floor[(i + 1) % 4]).unwrap();
        let to = b.find_position(plug, 0, floor[i]).unwrap();
        b.mef(from, to, split(next(), 6 + i as u32, Orientation::Forward))
            .unwrap();
        assert_euler(&b, "wall mef");
    }
    assert_eq!(b.counts().to_string(), "16/24/11/12/1 g0 = 0");
    // The floor is the plug, on the bottom's surface with the opposite
    // orientation; kfmrh opens it into the bottom face.
    assert_eq!(
        b.face(plug).unwrap().surface(),
        b.face(f0).unwrap().surface()
    );
    assert_ne!(
        b.face(plug).unwrap().orientation(),
        b.face(f0).unwrap().orientation()
    );
    let ring = b.kfmrh(plug, f0).unwrap();
    assert_euler(&b, "kfmrh");
    assert_eq!(b.counts().to_string(), "16/24/10/12/1 g1 = 0");
    assert_eq!(b.face(f0).unwrap().loops().len(), 2);
    let two_loop_faces = b.faces().filter(|(_, f)| f.loops().len() == 2).count();
    assert_eq!(two_loop_faces, 2, "top and bottom");
    // And back: the inverse of every step restores the box.
    let plug = b.mfkrh(f0, ring).unwrap();
    assert_eq!(b.counts().genus, 0);
    for i in (0..4).rev() {
        let e = b.face(plug).unwrap().loops()[0]
            .uses()
            .iter()
            .map(|u| u.edge)
            .find(|&e| {
                let e = b.edge(e).unwrap();
                (e.start() == floor[i] && e.end() == floor[(i + 1) % 4])
                    || (e.start() == floor[(i + 1) % 4] && e.end() == floor[i])
            })
            .unwrap();
        b.kef(e).unwrap();
        assert_euler(&b, "kef wall");
    }
    for &g in floor.iter().rev() {
        let e = b
            .edges()
            .find(|(_, e)| e.end() == g)
            .map(|(e, _)| e)
            .unwrap();
        b.kev(e).unwrap();
        assert_euler(&b, "kev strut down");
    }
    let plug_edge = b.face(plug).unwrap().loops()[0]
        .uses()
        .iter()
        .find(|u| {
            u.orientation == Orientation::Forward && {
                let e = b.edge(u.edge).unwrap();
                e.start() == h3 && e.end() == h0
            }
        })
        .map(|u| u.edge)
        .unwrap();
    // The ring must be joined back to the outer loop before the plug edge
    // can be killed as a face separator: mekr with the bridge.
    let ring_index = b
        .face(f1)
        .unwrap()
        .loops()
        .iter()
        .position(|l| l.uses().iter().any(|u| u.edge == plug_edge))
        .unwrap();
    let outer_index = 1 - ring_index;
    let from = b.find_position(f1, outer_index, tops[0]).unwrap();
    let to = b.find_position(f1, ring_index, h0).unwrap();
    let bridge = b.mekr(from, to, join(next())).unwrap();
    assert_euler(&b, "mekr bridge");
    b.kef(plug_edge).unwrap();
    assert_euler(&b, "kef plug");
    for &h in [h3, h2, h1, h0].iter() {
        let e = b
            .edges()
            .find(|(_, e)| e.end() == h)
            .map(|(e, _)| e)
            .unwrap();
        if e == bridge {
            assert_eq!(h, h0);
        }
        b.kev(e).unwrap();
        assert_euler(&b, "kev rim strut");
    }
    assert_eq!(
        b.dump(),
        box_dump,
        "the frame's construction undone is the box"
    );
}

/// What the property picks, resolved against the builder at that moment;
/// a pick that does not apply is skipped.
#[derive(Debug, Clone)]
enum Pick {
    Mev(u32, u32, u32),
    Mef(u32, u32, u32, u32, u32, bool),
    Mekr(u32, u32, u32, u32, u32),
    Kemr(u32),
    Kev(u32),
    Kef(u32),
    Kfmrh(u32, u32),
    Mfkrh(u32, u32),
}

fn pick() -> impl Strategy<Value = Pick> {
    prop_oneof![
        4 => (any::<u32>(), any::<u32>(), any::<u32>()).prop_map(|(a, b, c)| Pick::Mev(a, b, c)),
        4 => (any::<u32>(), any::<u32>(), any::<u32>(), any::<u32>(), 0..3u32, any::<bool>())
            .prop_map(|(a, b, c, d, s, o)| Pick::Mef(a, b, c, d, s, o)),
        2 => (any::<u32>(), any::<u32>(), any::<u32>(), any::<u32>(), any::<u32>())
            .prop_map(|(a, b, c, d, e)| Pick::Mekr(a, b, c, d, e)),
        2 => any::<u32>().prop_map(Pick::Kemr),
        2 => any::<u32>().prop_map(Pick::Kev),
        2 => any::<u32>().prop_map(Pick::Kef),
        2 => (any::<u32>(), any::<u32>()).prop_map(|(a, b)| Pick::Kfmrh(a, b)),
        1 => (any::<u32>(), any::<u32>()).prop_map(|(a, b)| Pick::Mfkrh(a, b)),
    ]
}

fn choose<T: Copy>(items: &[T], k: u32) -> Option<T> {
    if items.is_empty() {
        None
    } else {
        Some(items[k as usize % items.len()])
    }
}

/// Runs one pick on `b` if it applies: the op, its inverse (which must
/// restore the dump), and the op again from the inverse's record (which
/// must restore the dump after the op). Returns whether it applied.
fn apply(b: &mut Builder, p: &Pick, k: u32) -> Result<bool, TestCaseError> {
    let faces: Vec<FaceRef> = b.faces().map(|(f, _)| f).collect();
    let edges: Vec<EdgeRef> = b.edges().map(|(e, _)| e).collect();
    let before = b.dump();
    prop_assert_eq!(b.counts().euler(), 0);
    let position = |b: &Builder, f: FaceRef, li: u32, ci: u32, extra: usize| {
        let loops = b.face(f).unwrap().loops();
        let li = li as usize % loops.len();
        let n = loops[li].uses().len();
        Position::new(f, li, ci as usize % (n + extra))
    };
    macro_rules! refused {
        ($result:expr) => {
            match $result {
                Ok(v) => v,
                Err(e) => {
                    prop_assert_eq!(b.dump(), before, "a refusal ({}) changed the builder", e);
                    return Ok(false);
                }
            }
        };
    }
    match *p {
        Pick::Mev(fi, li, ci) => {
            let Some(f) = choose(&faces, fi) else {
                return Ok(false);
            };
            let at = position(b, f, li, ci, 1);
            let (_, e) = refused!(b.mev(at, strut(k)));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let (at2, strut2) = b.kev(e).map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "mev then kev");
            b.mev(at2, strut2)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "kev then mev");
        }
        Pick::Mef(fi, li, a, c, s, o) => {
            let Some(f) = choose(&faces, fi) else {
                return Ok(false);
            };
            let from = position(b, f, li, a, 1);
            let to = Position::new(f, from.loop_index, {
                let n = b.face(f).unwrap().loops()[from.loop_index].uses().len();
                c as usize % (n + 1)
            });
            let orientation = if o {
                Orientation::Forward
            } else {
                Orientation::Reversed
            };
            let (e, _) = refused!(b.mef(from, to, split(k, s, orientation)));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let (from2, to2, split2) = b.kef(e).map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "mef then kef");
            b.mef(from2, to2, split2)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "kef then mef");
        }
        Pick::Mekr(fi, la, a, lb, c) => {
            let two_loops: Vec<FaceRef> = faces
                .iter()
                .copied()
                .filter(|&f| b.face(f).unwrap().loops().len() >= 2)
                .collect();
            let Some(f) = choose(&two_loops, fi) else {
                return Ok(false);
            };
            let loops = b.face(f).unwrap().loops().len();
            let la = la as usize % loops;
            let lb = (la + 1 + lb as usize % (loops - 1)) % loops;
            let from = Position::new(
                f,
                la,
                a as usize % (b.face(f).unwrap().loops()[la].uses().len() + 1),
            );
            let to = Position::new(
                f,
                lb,
                c as usize % (b.face(f).unwrap().loops()[lb].uses().len() + 1),
            );
            let e = refused!(b.mekr(from, to, join(k)));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let (from2, to2, join2) = b.kemr(e).map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "mekr then kemr");
            b.mekr(from2, to2, join2)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "kemr then mekr");
        }
        Pick::Kemr(ei) => {
            let Some(e) = choose(&edges, ei) else {
                return Ok(false);
            };
            let (from, to, join) = refused!(b.kemr(e));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let e2 = b
                .mekr(from, to, join)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "kemr then mekr");
            b.kemr(e2).map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "mekr then kemr");
        }
        Pick::Kev(ei) => {
            let Some(e) = choose(&edges, ei) else {
                return Ok(false);
            };
            let (at, strut) = refused!(b.kev(e));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let (_, e2) = b
                .mev(at, strut)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "kev then mev");
            b.kev(e2).map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "mev then kev");
        }
        Pick::Kef(ei) => {
            let Some(e) = choose(&edges, ei) else {
                return Ok(false);
            };
            let (from, to, split) = refused!(b.kef(e));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let (e2, _) = b
                .mef(from, to, split)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "kef then mef");
            b.kef(e2).map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "mef then kef");
        }
        Pick::Kfmrh(a, c) => {
            let Some(kill) = choose(&faces, a) else {
                return Ok(false);
            };
            let Some(into) = choose(&faces, c) else {
                return Ok(false);
            };
            let ring = refused!(b.kfmrh(kill, into));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let f2 = b
                .mfkrh(into, ring)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "kfmrh then mfkrh");
            b.kfmrh(f2, into)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "mfkrh then kfmrh");
        }
        Pick::Mfkrh(fi, r) => {
            let Some(f) = choose(&faces, fi) else {
                return Ok(false);
            };
            let ring = r as usize % b.face(f).unwrap().loops().len().max(1);
            let new = refused!(b.mfkrh(f, ring));
            prop_assert_eq!(b.counts().euler(), 0);
            let after = b.dump();
            let r2 = b
                .kfmrh(new, f)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), before, "mfkrh then kfmrh");
            b.mfkrh(f, r2)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert_eq!(b.dump(), after, "kfmrh then mfkrh");
        }
    }
    Ok(true)
}

/// Random small sequences: every operator keeps the Euler line at zero,
/// every operator followed by its inverse restores the dump, the record
/// the inverse returns re-applies the operator to the same dump, and a
/// refused operator changes nothing.
#[test]
fn every_operator_keeps_euler_zero_and_its_inverse_restores_the_dump() {
    arris_debug::prop::check(proptest::collection::vec(pick(), 1..24), |picks| {
        let mut b = Builder::new(1e-7);
        b.mvfs(seed())
            .map_err(|e| TestCaseError::fail(e.to_string()))?;
        for (k, p) in picks.iter().enumerate() {
            apply(&mut b, p, k as u32 + 1)?;
        }
        Ok(())
    });
}

#[test]
fn mvfs_then_kvfs_is_the_empty_builder_and_kvfs_refuses_anything_more() {
    let mut b = Builder::new(1e-7);
    let empty = b.dump();
    assert!(b.is_empty());
    assert_eq!(b.counts().to_string(), "0/0/0/0/0 g0 = 0");
    let (v, f) = b.mvfs(seed()).unwrap();
    assert_eq!((v, f), (VertexRef::new(0), FaceRef::new(0)));
    assert_eq!(b.mvfs(seed()), Err(BuildError::NotEmpty));
    assert_eq!(b.counts().to_string(), "1/0/1/1/1 g0 = 0");
    let made = b.dump();
    b.mev(Position::new(f, 0, 0), strut(1)).unwrap();
    assert_eq!(b.kvfs(), Err(BuildError::NotASeed));
    b.kev(EdgeRef::new(0)).unwrap();
    assert_eq!(b.dump(), made);
    assert_eq!(b.kvfs(), Ok(seed()));
    assert_eq!(b.dump(), empty);
    assert!(b.is_empty());
    assert_eq!(b.mvfs(seed()), Ok((v, f)), "the slots start over");
}

#[test]
fn refusals_are_typed_and_leave_the_builder_untouched() {
    let mut b = Builder::new(1e-7);
    let Cylinder {
        v0,
        v1,
        e_seam,
        e_top,
        f_bottom,
        f_wall,
        f_top,
    } = cylinder(&mut b);
    let before = b.dump();
    let degenerate = EdgeGeometry::Degenerate {
        range: Interval::UNIT,
    };
    assert_eq!(
        b.mev(
            Position::new(f_wall, 0, 0),
            Strut {
                point: point(9),
                geometry: degenerate,
                pcurves: [None, None]
            }
        ),
        Err(BuildError::DegenerateStrut)
    );
    let from = Position::new(f_wall, 0, 0);
    let to = Position::new(f_wall, 0, 2);
    assert_eq!(
        b.mef(
            from,
            to,
            Split {
                geometry: degenerate,
                surface: surface(4),
                orientation: Orientation::Forward,
                pcurves: [None, None]
            }
        ),
        Err(BuildError::DegenerateEnds { start: v0, end: v1 })
    );
    assert_eq!(
        b.mef(
            from,
            Position::new(f_top, 0, 0),
            split(9, 4, Orientation::Forward)
        ),
        Err(BuildError::NotOneLoop {
            from,
            to: Position::new(f_top, 0, 0)
        })
    );
    assert_eq!(
        b.mekr(from, to, join(9)),
        Err(BuildError::NotTwoLoops { from, to })
    );
    assert_eq!(b.kev(e_seam), Err(BuildError::NotAStrut { edge: e_seam }));
    assert_eq!(b.kemr(e_top), Err(BuildError::NotInOneLoop { edge: e_top }));
    assert_eq!(
        b.kef(e_seam),
        Err(BuildError::NotSeparating { edge: e_seam })
    );
    assert_eq!(
        b.kfmrh(f_bottom, f_top),
        Err(BuildError::SurfaceMismatch {
            kill: f_bottom,
            kill_surface: surface(0),
            into: f_top,
            into_surface: surface(2)
        })
    );
    assert_eq!(
        b.kfmrh(f_top, f_top),
        Err(BuildError::OneFace { face: f_top })
    );
    assert_eq!(
        b.mfkrh(f_wall, 0),
        Err(BuildError::NoRing {
            face: f_wall,
            ring: 0
        })
    );
    assert_eq!(
        b.mfkrh(FaceRef::new(7), 0),
        Err(BuildError::NoFace(FaceRef::new(7)))
    );
    assert_eq!(
        b.kev(EdgeRef::new(7)),
        Err(BuildError::NoEdge(EdgeRef::new(7)))
    );
    assert_eq!(
        b.vertex_at(Position::new(f_wall, 1, 0)),
        Err(BuildError::NoLoop {
            face: f_wall,
            loop_index: 1
        })
    );
    assert_eq!(
        b.find_position(f_top, 0, v0),
        Err(BuildError::NotInLoop {
            vertex: v0,
            face: f_top,
            loop_index: 0
        })
    );
    assert_eq!(b.dump(), before);
    // kfmrh with the same orientation is refused; with the opposite one and
    // the same surface it needs one loop on the killed side.
    let mut c = Builder::new(1e-7);
    let (_, f) = c.mvfs(seed()).unwrap();
    let at = Position::new(f, 0, 0);
    let (_, g) = c.mef(at, at, split(0, 0, Orientation::Forward)).unwrap();
    assert_eq!(
        c.kfmrh(g, f),
        Err(BuildError::SameOrientation { kill: g, into: f })
    );
    assert_eq!(c.mfkrh(f, 0), Err(BuildError::NoRing { face: f, ring: 0 }));
}

/// A valid model for the tests of `finish`: the cylinder of the sample,
/// every geometry id real.
fn cylinder_in(m: &mut Model) -> (Builder, FaceRef) {
    use core::f64::consts::TAU;
    let (r, h) = (4.0, 12.0);
    let base = Frame::world();
    let top = base.with_origin(Point3::new(0.0, 0.0, h));
    let wall = m.add_surface(Surface::Cylinder {
        frame: base,
        radius: r,
    });
    let bottom = m.add_surface(Surface::Plane { frame: base });
    let top_plane = m.add_surface(Surface::Plane { frame: top });
    let uv_line = |m: &mut Model, u: f64, v: f64, along_u: bool| {
        m.add_curve2(Curve2::Line {
            origin: Point2::new(u, v),
            direction: if along_u {
                Vec2::x_axis()
            } else {
                Vec2::y_axis()
            },
        })
    };
    let cap = |m: &mut Model| {
        m.add_curve2(Curve2::Circle {
            frame: arris_math::Frame2::identity(),
            radius: r,
        })
    };
    let mut b = Builder::new(m.precision().default_tolerance);
    let (v0, f_bottom) = b
        .mvfs(Seed {
            point: Point3::new(r, 0.0, 0.0),
            surface: bottom,
            orientation: Orientation::Reversed,
        })
        .unwrap();
    let at = Position::new(f_bottom, 0, 0);
    let circle = m.add_curve(Curve::Circle {
        frame: base,
        radius: r,
    });
    let (pw, pc) = (uv_line(m, 0.0, 0.0, true), cap(m));
    let (_, f_wall) = b
        .mef(
            at,
            at,
            Split {
                geometry: EdgeGeometry::Curve {
                    curve: circle,
                    range: Interval::TURN,
                },
                surface: wall,
                orientation: Orientation::Forward,
                pcurves: [Some(pw), Some(pc)],
            },
        )
        .unwrap();
    let seam = m.add_curve(Curve::Line {
        origin: Point3::new(r, 0.0, 0.0),
        direction: Vec3::z_axis(),
    });
    let (up, down) = (uv_line(m, TAU, 0.0, false), uv_line(m, 0.0, 0.0, false));
    let (v1, _) = b
        .mev(
            b.find_position(f_wall, 0, v0).unwrap(),
            Strut {
                point: Point3::new(r, 0.0, h),
                geometry: EdgeGeometry::Curve {
                    curve: seam,
                    range: Interval::new(0.0, h).unwrap(),
                },
                pcurves: [Some(up), Some(down)],
            },
        )
        .unwrap();
    let top_circle = m.add_curve(Curve::Circle {
        frame: top,
        radius: r,
    });
    let (pw, pc) = (uv_line(m, 0.0, h, true), cap(m));
    let at = b.find_position(f_wall, 0, v1).unwrap();
    let (_, f_top) = b
        .mef(
            at,
            at,
            Split {
                geometry: EdgeGeometry::Curve {
                    curve: top_circle,
                    range: Interval::TURN,
                },
                surface: top_plane,
                orientation: Orientation::Forward,
                pcurves: [Some(pc), Some(pw)],
            },
        )
        .unwrap();
    (b, f_top)
}

#[test]
fn finish_stores_reversed_faces_backwards_and_returns_the_maps() {
    let mut m = Model::default();
    let (b, f_top) = cylinder_in(&mut m);
    let built = b.finish(&mut m, BodyKind::Solid).unwrap();
    assert_eq!(built.vertices.len(), 2);
    assert_eq!(built.edges.len(), 3);
    assert_eq!(built.faces.len(), 3);
    assert_eq!(built.faces[&f_top], arris_topo::FaceId::new(2, 0));
    let text = arris_debug::dump_text(&m, built.body).unwrap();
    // The same topology, effective orientations and loops as the raw-built
    // sample cylinder, with the faces in slot order: bottom cap (used
    // Reversed, its stored coedge Forward so the effective one is `-e0`),
    // wall, top cap.
    let lines: Vec<String> = text
        .lines()
        .filter(|l| l.starts_with("    face") || l.starts_with("        coedge"))
        .map(|l| l.split_whitespace().take(2).collect::<Vec<_>>().join(" "))
        .collect();
    assert_eq!(
        lines,
        [
            "face -f0",
            "coedge -e0",
            "face +f1",
            "coedge +e0",
            "coedge +e1",
            "coedge -e2",
            "coedge -e1",
            "face +f2",
            "coedge +e2",
        ]
    );
    assert!(text.ends_with("euler 2/3/3/3/1 g0 = 0\n"));
    let bottom = m.face(arris_topo::FaceId::new(0, 0)).unwrap();
    assert_eq!(
        bottom.loops()[0].coedges()[0].orientation(),
        Orientation::Forward
    );
    assert_eq!(
        m.shell(built.shells[0]).unwrap().faces()[0].orientation,
        Orientation::Reversed
    );
}

/// `finish` holds a degenerate edge to one use: a closed degenerate edge
/// splitting a cap off the cylinder's top face is used by the cap and by
/// the face it leaves — twice — and is refused as `EdgeUses`, since no
/// surface closes on one singular point twice (`docs/DATA-MODEL.md`
/// §Euler operators). Every pcurve and surface resolves, so the edge rule
/// is what refuses it.
#[test]
fn finish_refuses_a_degenerate_edge_used_twice() {
    let mut m = Model::default();
    let (mut b, f_top) = cylinder_in(&mut m);
    let plane = m.add_surface(Surface::Plane {
        frame: Frame::world(),
    });
    let point = m.add_curve2(Curve2::Line {
        origin: Point2::origin(),
        direction: Vec2::x_axis(),
    });
    let at = Position::new(f_top, 0, 0);
    let (edge, _) = b
        .mef(
            at,
            at,
            Split {
                geometry: EdgeGeometry::Degenerate {
                    range: Interval::UNIT,
                },
                surface: plane,
                orientation: Orientation::Forward,
                pcurves: [Some(point), Some(point)],
            },
        )
        .unwrap();
    assert_eq!(
        b.finish(&mut m, BodyKind::Solid),
        Err(BuildError::EdgeUses { edge, uses: 2 })
    );
}

#[test]
fn finish_refuses_what_cannot_be_stored_and_the_transaction_leaves_nothing() {
    let mut m = Model::default();
    let probe = VertexId::new(0, 0);
    for kind in [BodyKind::Solid, BodyKind::Sheet] {
        assert_eq!(
            Builder::new(1e-7).finish(&mut m, kind),
            Err(BuildError::Empty)
        );
    }
    for kind in [BodyKind::Wire, BodyKind::General] {
        assert_eq!(
            Builder::new(1e-7).finish(&mut m, kind),
            Err(BuildError::Kind(kind))
        );
    }
    let r: Result<(), BuildError> = m.transaction(|m| {
        let plane = m.add_surface(Surface::Plane {
            frame: Frame::world(),
        });
        // The seed alone: a loop without coedges.
        let mut b = Builder::new(1e-7);
        let (_, f) = b
            .mvfs(Seed {
                point: Point3::origin(),
                surface: plane,
                orientation: Orientation::Forward,
            })
            .unwrap();
        assert_eq!(
            b.finish(m, BodyKind::Solid),
            Err(BuildError::EmptyLoop {
                face: f,
                loop_index: 0
            })
        );
        assert!(m.vertex(probe).is_err(), "a refused finish appends nothing");
        // A pcurve id that does not resolve.
        let (b2, _) = cylinder_in(m);
        let mut dangling = b2.clone();
        let at = Position::new(FaceRef::new(1), 0, 2);
        let previous = dangling.set_pcurve(at, Curve2Id::new(99, 0)).unwrap();
        assert!(previous.is_some());
        assert!(matches!(
            dangling.finish(m, BodyKind::Solid),
            Err(BuildError::NotFound(_))
        ));
        assert!(m.vertex(probe).is_err());
        assert_eq!(
            b2.clone()
                .set_pcurve(Position::new(FaceRef::new(1), 0, 4), Curve2Id::new(0, 0)),
            Err(BuildError::BadPosition {
                position: Position::new(FaceRef::new(1), 0, 4),
                len: 4
            })
        );
        // A pcurve never given.
        let mut fresh = Builder::new(1e-7);
        let (_, f) = fresh
            .mvfs(Seed {
                point: Point3::origin(),
                surface: plane,
                orientation: Orientation::Forward,
            })
            .unwrap();
        let at = Position::new(f, 0, 0);
        let (_, g) = fresh
            .mef(
                at,
                at,
                Split {
                    geometry: EdgeGeometry::Curve {
                        curve: CurveId::new(0, 0),
                        range: Interval::UNIT,
                    },
                    surface: plane,
                    orientation: Orientation::Reversed,
                    pcurves: [None, Some(Curve2Id::new(0, 0))],
                },
            )
            .unwrap();
        assert_eq!(
            fresh.finish(m, BodyKind::Solid),
            Err(BuildError::MissingPcurve {
                position: Position::new(g, 0, 0)
            })
        );
        // The valid one finishes, and the enclosing transaction undoes it.
        let built = b2.finish(m, BodyKind::Solid).unwrap();
        assert!(m.vertex(probe).is_ok());
        assert_eq!(m.body(built.body.id).unwrap().kind(), BodyKind::Solid);
        Err(BuildError::Empty)
    });
    assert_eq!(r, Err(BuildError::Empty));
    assert!(
        m.vertex(probe).is_err(),
        "the transaction left nothing behind"
    );
    assert!(m.surface(SurfaceId::new(0, 0)).is_err());
    assert_eq!(
        m.add_surface(Surface::Plane {
            frame: Frame::world()
        }),
        SurfaceId::new(0, 0),
        "not even an id"
    );
}
