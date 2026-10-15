//! The walked fillet (ADR-0062, nurbs-face-blend step 5): an edge
//! with a NURBS face, or a pair with no closed form, blends to a NURBS
//! stripe. The twin of a face is its exact NURBS form, so a fillet of the
//! twinned edge must agree with the closed form on the original: volume
//! and area, the checker clean at `Full`: a walked blend against the
//! analytic faces it touches is decided (step 6), and only a pair of NURBS
//! faces, which the checker has no form for, is left unchecked. The walked
//! chamfer (step 7) is held to the same twin property.

use arris_check::{Level, Unchecked, check};
use arris_debug::prop::twin::nurbs_twin_faces;
use arris_debug::sample;
use arris_debug::unmetered::{chamfer, fillet, mass_properties, primitive_box};
use arris_geom::{Curve, Surface, SurfaceKind};
use arris_math::Point3;
use arris_ops::{BlendReason, OpError, Reason};
use arris_topo::{Body, Edge, Face, Model};

/// The circle edge of `body` whose centre is level with `z`.
fn circle_at(m: &Model, body: Body, z: f64) -> Edge {
    m.edges(body)
        .unwrap()
        .into_iter()
        .find(|e| {
            m.edge(e.id).unwrap().curve().is_some_and(|(curve, _)| {
                matches!(m.curve(curve).unwrap(), Curve::Circle { frame, .. } if (frame.origin().z - z).abs() < 1e-9)
            })
        })
        .unwrap_or_else(|| panic!("no circle at z = {z}"))
}

/// The edge of `body` whose curve's midpoint is `at`.
fn edge_at(m: &Model, body: Body, at: Point3) -> Edge {
    m.edges(body)
        .unwrap()
        .into_iter()
        .find(|e| {
            m.edge(e.id).unwrap().curve().is_some_and(|(curve, range)| {
                (m.curve(curve).unwrap().point(range.midpoint()) - at).norm() < 1e-9
            })
        })
        .unwrap_or_else(|| panic!("no edge through {at}"))
}

/// The face of `body` whose surface satisfies `want`.
fn face_where(m: &Model, body: Body, want: impl Fn(&Surface) -> bool) -> Face {
    m.faces(body)
        .unwrap()
        .into_iter()
        .find(|f| want(m.surface(m.face(f.id).unwrap().surface()).unwrap()))
        .expect("the face")
}

/// `walked` agrees with `exact`: clean at `Full` with nothing unchecked
/// but pairs of NURBS faces, and of a NURBS face with a face of one of the
/// kinds `left` names, the same volume and area to the fit.
fn agrees(m: &Model, exact: Body, walked: Body, left: &[SurfaceKind]) {
    let report = check(m, walked, Level::Full);
    assert!(report.is_ok(), "{report}");
    let undecided = |kinds: &(SurfaceKind, SurfaceKind)| {
        let other = match *kinds {
            (SurfaceKind::Nurbs, other) | (other, SurfaceKind::Nurbs) => other,
            _ => return false,
        };
        other == SurfaceKind::Nurbs || left.contains(&other)
    };
    assert!(
        report
            .unchecked()
            .iter()
            .all(|u| matches!(u, Unchecked::FacePair { kinds, .. } if undecided(kinds))),
        "{report}"
    );
    let (a, b) = (
        mass_properties(m, exact).unwrap(),
        mass_properties(m, walked).unwrap(),
    );
    assert!(
        (a.volume - b.volume).abs() <= 1e-8 * a.volume,
        "volume {} against {}",
        a.volume,
        b.volume
    );
    assert!(
        (a.area - b.area).abs() <= 1e-8 * a.area,
        "area {} against {}",
        a.area,
        b.area
    );
    assert!((a.centroid - b.centroid).norm() <= 1e-7);
}

/// A cylinder's wall or its cap as a NURBS twin, its rims filleted: the
/// walked ring against the closed form's torus on the original, concave
/// and convex, whichever face is the twin.
#[test]
fn a_cylinders_twinned_rim_fillets_as_the_closed_forms_torus() {
    let (radius, height) = (4.0, 12.0);
    let r = 0.5;
    {
        for z in [0.0, height] {
            for twin_wall in [true, false] {
                let mut m = Model::default();
                let cylinder = sample::cylinder(&mut m, radius, height).unwrap();
                let rim = circle_at(&m, cylinder, z);
                let (exact, _) = fillet(&mut m, cylinder, &[rim], r).unwrap();
                let face = if twin_wall {
                    face_where(&m, cylinder, |s| matches!(s, Surface::Cylinder { .. }))
                } else {
                    face_where(
                        &m,
                        cylinder,
                        |s| matches!(s, Surface::Plane { frame } if (frame.origin().z - z).abs() < 1e-9),
                    )
                };
                let twin = nurbs_twin_faces(&mut m, cylinder, &[face.id]).unwrap();
                let rim = circle_at(&m, twin, z);
                let (walked, _) = fillet(&mut m, twin, &[rim], r)
                    .unwrap_or_else(|e| panic!("r {r} z {z} wall {twin_wall}: {e}"));
                agrees(&m, exact, walked, &[]);
            }
        }
    }
}

/// A box's top face as a NURBS twin, one top edge filleted: a walked
/// stripe of a NURBS and a plane face, two planes across its ends.
#[test]
fn a_boxs_twinned_top_edge_fillets_as_the_closed_forms_cylinder() {
    for r in [0.2, 0.5] {
        let mut m = Model::default();
        let (cube, _) =
            primitive_box(&mut m, Point3::origin(), Point3::new(3.0, 2.0, 2.0)).unwrap();
        let edge = edge_at(&m, cube, Point3::new(1.5, 0.0, 2.0));
        let (exact, _) = fillet(&mut m, cube, &[edge], r).unwrap();
        let top = face_where(&m, cube, |s| {
            matches!(s, Surface::Plane { frame } if (frame.origin().z - 2.0).abs() < 1e-9
                && frame.z().z.abs() > 0.5)
        });
        let twin = nurbs_twin_faces(&mut m, cube, &[top.id]).unwrap();
        let edge = edge_at(&m, twin, Point3::new(1.5, 0.0, 2.0));
        let (walked, _) = fillet(&mut m, twin, &[edge], r).unwrap_or_else(|e| panic!("r {r}: {e}"));
        agrees(&m, exact, walked, &[]);
    }
}

/// A cylinder's wall or its cap as a NURBS twin, its rims chamfered: the
/// walked ring against the closed form's cone on the original, concave
/// and convex, whichever face is the twin.
#[test]
fn a_cylinders_twinned_rim_chamfers_as_the_closed_forms_cone() {
    let (radius, height) = (4.0, 12.0);
    for d in [0.5, 1.5] {
        for z in [0.0, height] {
            for twin_wall in [true, false] {
                let mut m = Model::default();
                let cylinder = sample::cylinder(&mut m, radius, height).unwrap();
                let rim = circle_at(&m, cylinder, z);
                let (exact, _) = chamfer(&mut m, cylinder, &[rim], d).unwrap();
                let face = if twin_wall {
                    face_where(&m, cylinder, |s| matches!(s, Surface::Cylinder { .. }))
                } else {
                    face_where(
                        &m,
                        cylinder,
                        |s| matches!(s, Surface::Plane { frame } if (frame.origin().z - z).abs() < 1e-9),
                    )
                };
                let twin = nurbs_twin_faces(&mut m, cylinder, &[face.id]).unwrap();
                let rim = circle_at(&m, twin, z);
                let (walked, _) = chamfer(&mut m, twin, &[rim], d)
                    .unwrap_or_else(|e| panic!("d {d} z {z} wall {twin_wall}: {e}"));
                agrees(&m, exact, walked, &[]);
            }
        }
    }
}

/// A box's top face as a NURBS twin, one top edge chamfered: a walked
/// stripe of a NURBS and a plane face, two planes across its ends, the
/// closed form's plane on the original.
#[test]
fn a_boxs_twinned_top_edge_chamfers_as_the_closed_forms_plane() {
    for d in [0.2, 0.5] {
        let mut m = Model::default();
        let (cube, _) =
            primitive_box(&mut m, Point3::origin(), Point3::new(3.0, 2.0, 2.0)).unwrap();
        let edge = edge_at(&m, cube, Point3::new(1.5, 0.0, 2.0));
        let (exact, _) = chamfer(&mut m, cube, &[edge], d).unwrap();
        let top = face_where(&m, cube, |s| {
            matches!(s, Surface::Plane { frame } if (frame.origin().z - 2.0).abs() < 1e-9
                && frame.z().z.abs() > 0.5)
        });
        let twin = nurbs_twin_faces(&mut m, cube, &[top.id]).unwrap();
        let edge = edge_at(&m, twin, Point3::new(1.5, 0.0, 2.0));
        let (walked, _) =
            chamfer(&mut m, twin, &[edge], d).unwrap_or_else(|e| panic!("d {d}: {e}"));
        agrees(&m, exact, walked, &[]);
    }
}

/// A radius past the cylinder's own radius cannot roll inside its twinned
/// wall: refused as too large, the model untouched.
#[test]
fn a_radius_past_the_walls_curvature_is_too_large() {
    let mut m = Model::default();
    let cylinder = sample::cylinder(&mut m, 2.0, 6.0).unwrap();
    let wall = face_where(&m, cylinder, |s| matches!(s, Surface::Cylinder { .. }));
    let twin = nurbs_twin_faces(&mut m, cylinder, &[wall.id]).unwrap();
    let rim = circle_at(&m, twin, 6.0);
    let err = fillet(&mut m, twin, &[rim], 2.5).unwrap_err();
    assert!(
        matches!(
            err,
            OpError::Degenerate {
                reason: Reason::Blend(BlendReason::TooLarge),
                ..
            }
        ),
        "{err}"
    );
}

/// An edge between two NURBS faces is NURBS against NURBS, the cycle's
/// third item: refused naming both faces.
#[test]
fn two_nurbs_faces_are_unsupported() {
    let mut m = Model::default();
    let (cube, _) = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 2.0)).unwrap();
    let twin = arris_debug::prop::twin::nurbs_twin(&mut m, cube).unwrap();
    let edge = edge_at(&m, twin, Point3::new(1.0, 0.0, 2.0));
    let err = fillet(&mut m, twin, &[edge], 0.2).unwrap_err();
    assert!(matches!(err, OpError::Unsupported { .. }), "{err}");
}

/// A bar whose wall runs tangentially into a quarter cylinder, that
/// cylinder as its NURBS twin, the wall's top edge filleted: the chain
/// runs on from the table's cylinder into a walked stripe on the twin and
/// the two meet on the ball's arc at the tangent vertex (ADR-0035 §3), as
/// the table's cylinder meets its torus on the original.
#[test]
fn a_chain_into_a_twinned_arc_meets_on_the_balls_arc() {
    let recipe: arris_debug::fixtures::Recipe = serde_json::from_str(
        r#"{"steps": [
            {"name": "sketch", "op": "profile", "plane": {"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 1, 0]},
             "outer": {"start": [0, 0], "segments": [{"line_to": [3, 0]}, {"arc_to": [4, 1], "via": [3.7071067811865475, 0.2928932188134524]}, {"line_to": [0, 1]}, {"line_to": [0, 0]}]}},
            {"name": "bar", "op": "extrude", "profile": "sketch", "direction": [0, 0, 1], "length": 1}
        ], "result": "bar"}"#,
    )
    .unwrap();
    for r in [0.1, 0.25] {
        let chain = arris_debug::corpus::build("walked/line-into-arc", &recipe).unwrap();
        let mut m = chain.model;
        let bar = chain.steps["bar"].body;
        let edge = edge_at(&m, bar, Point3::new(1.5, 0.0, 1.0));
        let (exact, _) = fillet(&mut m, bar, &[edge], r).unwrap();
        let wall = face_where(&m, bar, |s| matches!(s, Surface::Cylinder { .. }));
        let twin = nurbs_twin_faces(&mut m, bar, &[wall.id]).unwrap();
        let edge = edge_at(&m, twin, Point3::new(1.5, 0.0, 1.0));
        let (walked, _) = fillet(&mut m, twin, &[edge], r).unwrap_or_else(|e| panic!("r {r}: {e}"));
        // The table's cylinder against the walked blend at their junction
        // is left by name: `the_convex_junction_is_decided` below.
        agrees(&m, exact, walked, &[SurfaceKind::Cylinder]);
    }
}

/// The chain above checked with nothing unchecked but its pair of two
/// NURBS faces: the bar's wall and the quarter cylinder are convex, and the
/// table's cylinder, carried on past the junction, passes inside the walked
/// blend beside its wall contact. That branch enters the blend's domain
/// beside the corner where the arc meets the contact on the top face, and
/// runs within rounding of the contact edge, which ADR-0061 refuses (its
/// 2026-10-14 amendment takes a touch that does not enter). The slot's
/// concave junction is decided (`blend/nurbs-slot-plate-rim-fillet`).
#[test]
#[ignore = "S5 leaves a walked blend against the table's cylinder at a convex junction undecided: a branch entering the domain at the corner, refused (backlog)"]
fn the_convex_junction_is_decided() {
    let recipe: arris_debug::fixtures::Recipe = serde_json::from_str(
        r#"{"steps": [
            {"name": "sketch", "op": "profile", "plane": {"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 1, 0]},
             "outer": {"start": [0, 0], "segments": [{"line_to": [3, 0]}, {"arc_to": [4, 1], "via": [3.7071067811865475, 0.2928932188134524]}, {"line_to": [0, 1]}, {"line_to": [0, 0]}]}},
            {"name": "bar", "op": "extrude", "profile": "sketch", "direction": [0, 0, 1], "length": 1}
        ], "result": "bar"}"#,
    )
    .unwrap();
    for r in [0.1, 0.25] {
        let chain = arris_debug::corpus::build("walked/line-into-arc", &recipe).unwrap();
        let mut m = chain.model;
        let bar = chain.steps["bar"].body;
        let edge = edge_at(&m, bar, Point3::new(1.5, 0.0, 1.0));
        let (exact, _) = fillet(&mut m, bar, &[edge], r).unwrap();
        let wall = face_where(&m, bar, |s| matches!(s, Surface::Cylinder { .. }));
        let twin = nurbs_twin_faces(&mut m, bar, &[wall.id]).unwrap();
        let edge = edge_at(&m, twin, Point3::new(1.5, 0.0, 1.0));
        let (walked, _) = fillet(&mut m, twin, &[edge], r).unwrap_or_else(|e| panic!("r {r}: {e}"));
        agrees(&m, exact, walked, &[]);
    }
}
