//! `ops::fillet` (ADR-0007): one convex box edge end to end — the blend
//! cylinder, its contacts and its end arcs — clean at `Full` with nothing
//! unchecked, the closed-form volume, the provenance rooted at the edge
//! and audited, identical over two runs; two edges meeting in a miter,
//! held to the oracle's numbers while its S5 row waits; a second fillet
//! on a filleted body, its records composed back to the extrude; and
//! every typed refusal.

use arris_debug::fixtures::Class;
use arris_debug::unmetered::cut;
use arris_debug::unmetered::{chamfer, extrude, fillet, mass_properties, primitive_box, revolve};
use arris_debug::{corpus, dump_text, fixtures};
use arris_ops::arris_check::arris_topo::arris_geom::{
    Curve, Profile, ProfileLoop, ProfileSegment, Surface,
};

use arris_ops::arris_check::arris_topo::arris_math::nalgebra::{Unit, UnitQuaternion};
use arris_ops::arris_check::arris_topo::arris_math::{Axis, Frame, Point2, Point3, Vec3};
use arris_ops::arris_check::arris_topo::provenance::{Origin, Relation, Role, SweepPart, audit};
use arris_ops::arris_check::arris_topo::{Body, Edge, EntityId, Model, Orientation, Shape};
use arris_ops::arris_check::classify::{Classification, classify_point};
use arris_ops::arris_check::{Level, check};
use arris_ops::{OpError, Reason};
/// The edge of `body` whose curve's midpoint is `at`.
fn edge_at(m: &Model, body: Body, at: Point3) -> Edge {
    m.edges(body)
        .unwrap()
        .into_iter()
        .find(|e| {
            let entity = m.edge(e.id).unwrap();
            entity.curve().is_some_and(|(curve, range)| {
                (m.curve(curve).unwrap().point(range.midpoint()) - at).norm() < 1e-9
            })
        })
        .unwrap_or_else(|| panic!("no edge through {at}"))
}

fn cube(m: &mut Model, side: f64) -> Body {
    primitive_box(m, Point3::origin(), Point3::new(side, side, side))
        .unwrap()
        .0
}

fn reason(err: &OpError) -> Option<Reason> {
    match err {
        OpError::Degenerate { reason, .. } => Some(*reason),
        _ => None,
    }
}

/// The consumer's number: a 2-cube with one vertical edge filleted at
/// `r = 0.2` loses `(1 − π/4) r² · 2`.
#[test]
fn one_convex_box_edge_end_to_end() {
    let mut m = Model::default();
    let body = cube(&mut m, 2.0);
    let edge = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let (blended, provenance) = fillet(&mut m, body, &[edge], 0.2).unwrap();

    let report = check(&m, blended, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    let line = report.euler().unwrap();
    assert_eq!(
        (
            line.vertices,
            line.edges,
            line.faces,
            line.loops,
            line.genus
        ),
        (10, 15, 7, 7, 0)
    );

    let props = mass_properties(&m, blended).unwrap();
    let r: f64 = 0.2;
    let volume = 8.0 - (1.0 - core::f64::consts::FRAC_PI_4) * r * r * 2.0;
    assert!(
        (props.volume - volume).abs() <= 1e-9 * volume,
        "{}",
        props.volume
    );
    let area = 24.0 - 4.0 * r - 2.0 * (1.0 - core::f64::consts::FRAC_PI_4) * r * r
        + core::f64::consts::PI * r;
    assert!((props.area - area).abs() <= 1e-9 * area, "{}", props.area);

    // Provenance: rooted at the edge, and audited.
    audit(&m, &[body], blended, &provenance).unwrap();
    let generated = provenance.generated_from(edge.shape());
    let faces = generated
        .iter()
        .filter(|s| matches!(s.id, EntityId::Face(_)))
        .count();
    let edges = generated
        .iter()
        .filter(|s| matches!(s.id, EntityId::Edge(_)))
        .count();
    let vertices = generated
        .iter()
        .filter(|s| matches!(s.id, EntityId::Vertex(_)))
        .count();
    assert_eq!((faces, edges, vertices), (1, 4, 4));
    assert!(provenance.is_deleted(edge.shape()));
    let entity = m.edge(edge.id).unwrap();
    for v in [entity.start(), entity.end()] {
        assert!(provenance.is_deleted(Shape::new(v, Orientation::Forward)));
    }
    // The blended edge's two faces and the two caps across its ends are
    // modified; the two faces away from it are kept.
    let modified: Vec<_> = m
        .faces(body)
        .unwrap()
        .into_iter()
        .filter(|f| !provenance.modified_from(f.shape()).is_empty())
        .collect();
    assert_eq!(modified.len(), 4, "{provenance}");
    let kept = m
        .faces(body)
        .unwrap()
        .into_iter()
        .filter(|f| provenance.is_kept(f.shape(), &m, blended))
        .count();
    assert_eq!(kept, 2);

    // Deterministic.
    let mut again = Model::default();
    let body2 = cube(&mut again, 2.0);
    let edge2 = edge_at(&again, body2, Point3::new(2.0, 2.0, 1.0));
    let (blended2, provenance2) = fillet(&mut again, body2, &[edge2], 0.2).unwrap();
    assert_eq!(
        dump_text(&m, blended).unwrap(),
        dump_text(&again, blended2).unwrap()
    );
    assert_eq!(provenance, provenance2);
}

/// An extruded L, (0,0)–(2,0)–(2,1)–(1,1)–(1,2)–(0,2) at height 2: its
/// inner vertical edge at (1, 1) is concave.
fn ell(m: &mut Model) -> Body {
    let p = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(2.0, 0.0)),
                ProfileSegment::LineTo(p(2.0, 1.0)),
                ProfileSegment::LineTo(p(1.0, 1.0)),
                ProfileSegment::LineTo(p(1.0, 2.0)),
                ProfileSegment::LineTo(p(0.0, 2.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    extrude(m, &profile, Vec3::z(), 2.0).unwrap().0
}

/// A concave edge adds material: the L's inner edge gains
/// `(1 − π/4) r² · 2`, its blend face is reversed against its cylinder,
/// and the arcs on the caps lie outside the caps as they were. A radius
/// the notch's walls cannot hold is still refused.
#[test]
fn a_concave_edge_adds_material() {
    let mut m = Model::default();
    let body = ell(&mut m);
    let edge = edge_at(&m, body, Point3::new(1.0, 1.0, 1.0));
    let (blended, provenance) = fillet(&mut m, body, &[edge], 0.2).unwrap();

    let report = check(&m, blended, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    let line = report.euler().unwrap();
    assert_eq!(
        (line.vertices, line.edges, line.faces, line.loops),
        (14, 21, 9, 9)
    );
    let r: f64 = 0.2;
    let props = mass_properties(&m, blended).unwrap();
    let volume = 6.0 + (1.0 - core::f64::consts::FRAC_PI_4) * r * r * 2.0;
    assert!(
        (props.volume - volume).abs() <= 1e-9 * volume,
        "{}",
        props.volume
    );
    audit(&m, &[body], blended, &provenance).unwrap();
    let [face_id] = provenance
        .generated_from(edge.shape())
        .iter()
        .filter_map(|s| match s.id {
            EntityId::Face(id) => Some(id),
            _ => None,
        })
        .collect::<Vec<_>>()[..]
    else {
        panic!("{provenance}");
    };
    let blend = m
        .faces(blended)
        .unwrap()
        .into_iter()
        .find(|f| f.id == face_id)
        .unwrap();
    assert_eq!(blend.orientation, Orientation::Reversed);

    // The walls of the notch are 1 long: a ball of 1.2 does not fit.
    let err = fillet(&mut m, body, &[edge], 1.2).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
}

/// The four vertical edges of the consumer's cube in one call: each cap
/// edge is cut at both its ends by two different blends; the volume is
/// the consumer's number, and the same set listed in reverse is the same
/// result and the same record.
#[test]
fn four_disjoint_edges_in_one_call_in_either_order() {
    let corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)];
    let build = |reversed: bool| {
        let mut m = Model::default();
        let body = cube(&mut m, 2.0);
        let mut edges: Vec<Edge> = corners
            .iter()
            .map(|&(x, y)| edge_at(&m, body, Point3::new(x, y, 1.0)))
            .collect();
        if reversed {
            edges.reverse();
        }
        let (blended, provenance) = fillet(&mut m, body, &edges, 0.2).unwrap();
        (m, body, blended, provenance)
    };
    let (m, body, blended, provenance) = build(false);
    let report = check(&m, blended, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    let line = report.euler().unwrap();
    assert_eq!(
        (line.vertices, line.edges, line.faces, line.loops),
        (16, 24, 10, 10)
    );
    let props = mass_properties(&m, blended).unwrap();
    assert!(
        (props.volume - 7.93132741).abs() <= 1e-8,
        "{}",
        props.volume
    );
    audit(&m, &[body], blended, &provenance).unwrap();
    // Each cap edge is shortened at both ends, into one edge.
    let cap = edge_at(&m, body, Point3::new(1.0, 0.0, 2.0));
    let cap = Shape::new(cap.id, Orientation::Forward);
    assert_eq!(provenance.modified_from(cap).len(), 1, "{provenance}");

    let (again, _, blended2, provenance2) = build(true);
    assert_eq!(
        dump_text(&m, blended).unwrap(),
        dump_text(&again, blended2).unwrap()
    );
    assert_eq!(provenance, provenance2);
}

/// A radius that is not finite or not positive is the parameter's own
/// refusal.
#[test]
fn a_bad_radius_is_refused_by_name() {
    let mut m = Model::default();
    let body = cube(&mut m, 2.0);
    let edge = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let before = dump_text(&m, body).unwrap();
    assert!(matches!(
        reason(&fillet(&mut m, body, &[edge], f64::NAN).unwrap_err()),
        Some(Reason::NonFinite { what: "radius" })
    ));
    assert!(matches!(
        reason(&fillet(&mut m, body, &[edge], 0.0).unwrap_err()),
        Some(Reason::NotPositive { what: "radius", .. })
    ));
    assert_eq!(
        dump_text(&m, body).unwrap(),
        before,
        "the model is as it was"
    );
}

/// An empty list, an edge listed twice, and an edge of another body.
#[test]
fn the_edge_list_is_checked_before_anything_is_built() {
    let mut m = Model::default();
    let body = cube(&mut m, 2.0);
    let other = primitive_box(
        &mut m,
        Point3::new(5.0, 0.0, 0.0),
        Point3::new(7.0, 2.0, 2.0),
    )
    .unwrap()
    .0;
    let edge = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let foreign = edge_at(&m, other, Point3::new(7.0, 2.0, 1.0));
    assert_eq!(
        reason(&fillet(&mut m, body, &[], 0.2).unwrap_err()),
        Some(Reason::NoEdges)
    );
    assert_eq!(
        reason(&fillet(&mut m, body, &[edge, edge], 0.2).unwrap_err()),
        Some(Reason::RepeatedEdge)
    );
    let err = fillet(&mut m, body, &[foreign], 0.2).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::EdgeNotInBody));
    assert!(matches!(&err, OpError::Degenerate { entities, .. } if entities[0] == foreign.shape()));
}

/// A blend larger than its faces hold: a 2-cube at `r = 2.5`, whose
/// contact lines fall outside the faces, and a plate thinner than `r`,
/// whose corner edges are shorter than the trim.
#[test]
fn a_blend_too_large_for_its_faces_is_refused() {
    let mut m = Model::default();
    let body = cube(&mut m, 2.0);
    let edge = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let err = fillet(&mut m, body, &[edge], 2.5).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
    // Exactly the face's width: the contact would lie on the far edge.
    let err = fillet(&mut m, body, &[edge], 2.0).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");

    let plate = primitive_box(&mut m, Point3::origin(), Point3::new(2.0, 2.0, 0.5))
        .unwrap()
        .0;
    let top = edge_at(&m, plate, Point3::new(1.0, 2.0, 0.5));
    let err = fillet(&mut m, plate, &[top], 1.0).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
    // And a radius the plate holds builds.
    fillet(&mut m, plate, &[top], 0.2).unwrap();
}

/// The slot's arc-to-line edge: its two faces meet at a tangent
/// dihedral, so there is no corner.
#[test]
fn a_tangent_dihedral_is_a_tangent_chain() {
    let mut m = Model::default();
    let p = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, -1.0),
            segments: vec![
                ProfileSegment::LineTo(p(4.0, -1.0)),
                ProfileSegment::ArcTo {
                    to: p(4.0, 1.0),
                    via: p(5.0, 0.0),
                },
                ProfileSegment::LineTo(p(0.0, 1.0)),
                ProfileSegment::ArcTo {
                    to: p(0.0, -1.0),
                    via: p(-1.0, 0.0),
                },
            ],
        },
        holes: Vec::new(),
    };
    let slot = extrude(&mut m, &profile, Vec3::z(), 2.0).unwrap().0;
    let seam = edge_at(&m, slot, Point3::new(4.0, -1.0, 1.0));
    let err = fillet(&mut m, slot, &[seam], 0.2).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::TangentChain), "{err}");
}

/// A revolve's cone ruling on its end cap — a plane through the cone's
/// apex, along a ruling — is outside the table and is named as such.
#[test]
fn a_pair_outside_the_table_is_unsupported() {
    let mut m = Model::default();
    let p = |u, v| Point2::new(u, v);
    let plane = Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
    let profile = Profile {
        plane,
        outer: ProfileLoop::Path {
            start: p(1.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(2.0, 0.0)),
                ProfileSegment::LineTo(p(1.0, 2.0)),
                ProfileSegment::LineTo(p(1.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let ring = revolve(
        &mut m,
        &profile,
        Axis::z_at(Point3::origin()),
        core::f64::consts::FRAC_PI_2,
    )
    .unwrap()
    .0;
    // The cone's ruling on the start cap, the plane `y = 0` through the
    // axis.
    let ruling = edge_at(&m, ring, Point3::new(1.5, 0.0, 1.0));
    let err = fillet(&mut m, ring, &[ruling], 0.1).unwrap_err();
    assert!(matches!(err, OpError::Unsupported { .. }), "{err}");
}

/// A loop of lines and three-point arcs from `start` in the world's XY
/// plane, extruded 2 along +Z.
fn extruded(m: &mut Model, start: (f64, f64), segments: Vec<ProfileSegment>) -> Body {
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: Point2::new(start.0, start.1),
            segments,
        },
        holes: Vec::new(),
    };
    extrude(m, &profile, Vec3::z(), 2.0).unwrap().0
}

fn line_to(u: f64, v: f64) -> ProfileSegment {
    ProfileSegment::LineTo(Point2::new(u, v))
}

fn arc_to(u: f64, v: f64, via: (f64, f64)) -> ProfileSegment {
    ProfileSegment::ArcTo {
        to: Point2::new(u, v),
        via: Point2::new(via.0, via.1),
    }
}

/// The signed area Green's theorem gives the segment from `a` to `b`.
fn segment_area(a: Point2, b: Point2) -> f64 {
    (a.x * b.y - a.y * b.x) / 2.0
}

/// The signed area Green's theorem gives the short arc about `c` from `a`
/// to `b`.
fn arc_area(c: Point2, a: Point2, b: Point2) -> f64 {
    let (u, v) = (a - c, b - c);
    let sweep = (u.x * v.y - u.y * v.x).atan2(u.dot(&v));
    let w = b - a;
    (u.norm_squared() * sweep + c.x * w.y - c.y * w.x) / 2.0
}

/// A chain whose ball does not fit the arc it runs into (ADR-0035 §6): a
/// wall running tangentially into a quarter cylinder of radius 1, filleted
/// at a radius past it, where the arc's torus would not be a ring torus
/// and its contact on the top reaches the axis. Open CASCADE builds none
/// of these, so there is no fixture; a radius the arc holds builds.
#[test]
fn a_chain_whose_ball_leaves_the_arc_is_too_large() {
    let mut m = Model::default();
    let bar = extruded(
        &mut m,
        (0.0, 0.0),
        vec![
            line_to(3.0, 0.0),
            arc_to(4.0, 1.0, (3.7071067811865475, 0.2928932188134524)),
            line_to(0.0, 1.0),
            line_to(0.0, 0.0),
        ],
    );
    let wall = edge_at(&m, bar, Point3::new(1.5, 0.0, 2.0));
    for r in [1.0, 1.1] {
        let err = fillet(&mut m, bar, &[wall], r).unwrap_err();
        assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "r {r}: {err}");
    }
    fillet(&mut m, bar, &[wall], 0.25).unwrap();
}

/// A plane against a cylinder along a ruling (ADR-0007): the chord edge
/// of a disc's segment at three chord heights — a right dihedral, an
/// obtuse and an acute one — the rim of a half-round notch in a plate,
/// convex on a concave cylinder, and the root of a half-round rib, concave
/// on a convex one. Each is clean at `Full` with nothing unchecked and
/// audited, and moves the volume by its cross-section's corner times the
/// height: the region between the edge, the two contacts and the ball's
/// arc by Green's theorem, the ball's centre found in the section's own
/// terms — on the chord's offset line at `R ∓ r` from the axis.
#[test]
fn a_plane_against_a_cylinder_along_a_ruling_blends_to_a_cylinder() {
    let r = 0.3;
    let p = Point2::new;
    // A name, the loop, the cylinder's radius, the edge and the ball's
    // centre in the section, and whether the blend removes the corner.
    let mut cases = Vec::new();
    let big: f64 = 1.5;
    for k in [0.0_f64, -0.6, 0.6] {
        let xe = (big * big - k * k).sqrt();
        cases.push((
            format!("segment at y = {k}"),
            (-xe, k),
            vec![line_to(xe, k), arc_to(-xe, k, (0.0, big))],
            big,
            p(xe, k),
            p(((big - r).powi(2) - (k + r).powi(2)).sqrt(), k + r),
            true,
        ));
    }
    let notch = 1.0;
    cases.push((
        "notch rim".into(),
        (-2.0, -2.0),
        vec![
            line_to(2.0, -2.0),
            line_to(2.0, 0.0),
            line_to(notch, 0.0),
            arc_to(-notch, 0.0, (0.0, -notch)),
            line_to(-2.0, 0.0),
            line_to(-2.0, -2.0),
        ],
        notch,
        p(notch, 0.0),
        p(((notch + r).powi(2) - r * r).sqrt(), -r),
        true,
    ));
    let rib = 0.7;
    cases.push((
        "rib root".into(),
        (-2.0, -1.0),
        vec![
            line_to(2.0, -1.0),
            line_to(2.0, 0.0),
            line_to(rib, 0.0),
            arc_to(-rib, 0.0, (0.0, rib)),
            line_to(-2.0, 0.0),
            line_to(-2.0, -1.0),
        ],
        rib,
        p(rib, 0.0),
        p(((rib + r).powi(2) - r * r).sqrt(), r),
        false,
    ));
    for (name, start, segments, big, e, centre, removes) in cases {
        let mut m = Model::default();
        let body = extruded(&mut m, start, segments);
        let before = mass_properties(&m, body).unwrap().volume;
        let edge = edge_at(&m, body, Point3::new(e.x, e.y, 1.0));
        let (blended, provenance) =
            fillet(&mut m, body, &[edge], r).unwrap_or_else(|err| panic!("{name}: {err}"));
        let report = check(&m, blended, Level::Full);
        assert!(report.is_ok(), "{name}: {report}");
        assert!(report.unchecked().is_empty(), "{name}: {report}");
        audit(&m, &[body], blended, &provenance).unwrap();
        let on_plane = p(centre.x, e.y);
        let on_cylinder = Point2::from(centre.coords * (big / centre.coords.norm()));
        let corner = (segment_area(e, on_plane)
            + arc_area(centre, on_plane, on_cylinder)
            + arc_area(Point2::origin(), on_cylinder, e))
        .abs();
        let volume = if removes {
            before - corner * 2.0
        } else {
            before + corner * 2.0
        };
        let got = mass_properties(&m, blended).unwrap().volume;
        assert!(
            (got - volume).abs() <= 1e-9 * volume,
            "{name}: {got} against {volume}"
        );
    }
}

/// The closed edge of `body` that is a circle about `centre` of `radius`.
fn rim_at(m: &Model, body: Body, centre: Point3, radius: f64) -> Edge {
    m.edges(body)
        .unwrap()
        .into_iter()
        .find(|e| {
            let entity = m.edge(e.id).unwrap();
            entity.start() == entity.end()
                && entity.curve().is_some_and(|(curve, _)| {
                    matches!(
                        m.curve(curve).unwrap(),
                        Curve::Circle { frame, radius: r }
                            if (frame.origin() - centre).norm() < 1e-9 && (r - radius).abs() < 1e-9
                    )
                })
        })
        .unwrap_or_else(|| panic!("no rim of radius {radius} about {centre}"))
}

/// A 4 × 4 plate 1 thick with a hole of radius 1 through its middle.
fn holed_plate(m: &mut Model) -> Body {
    let p = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                line_to(4.0, 0.0),
                line_to(4.0, 4.0),
                line_to(0.0, 4.0),
                line_to(0.0, 0.0),
            ],
        },
        holes: vec![ProfileLoop::Circle {
            center: p(2.0, 2.0),
            radius: 1.0,
        }],
    };
    extrude(m, &profile, Vec3::z(), 1.0).unwrap().0
}

/// A disc of radius 3 and height 1 with a boss of radius 1 and height 1
/// on it, revolved whole about `z`.
fn bossed_disc(m: &mut Model) -> Body {
    let p = |u, v| Point2::new(u, v);
    let plane = Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
    let profile = Profile {
        plane,
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                line_to(3.0, 0.0),
                line_to(3.0, 1.0),
                line_to(1.0, 1.0),
                line_to(1.0, 2.0),
                line_to(0.0, 2.0),
                line_to(0.0, 0.0),
            ],
        },
        holes: Vec::new(),
    };
    revolve(
        m,
        &profile,
        Axis::z_at(Point3::origin()),
        core::f64::consts::TAU,
    )
    .unwrap()
    .0
}

/// A plane against a cylinder along a circle (ADR-0007): a torus coaxial
/// with the cylinder, with no ends. A hole's top rim, its bottom rim and
/// both in one call — convex, the material outside the cylinder — a
/// boss's base, concave, and its top rim and the disc's outer rim, convex
/// with the material inside. Each is clean at `Full` with nothing
/// unchecked and audited, generates one face, two contacts, a seam and
/// two vertices from the edge, and moves the volume by Pappus: the
/// corner's section `(1 − π/4) r²` swept about the axis at its centroid,
/// `(10 − 3π) / (12 − 3π) · r` from the edge toward the ball.
#[test]
fn a_plane_against_a_cylinder_along_a_circle_blends_to_a_torus() {
    use core::f64::consts::{PI, TAU};
    let r: f64 = 0.25;
    let section = (1.0 - PI / 4.0) * r * r;
    let reach = r * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI);
    // A name, the body, each rim's centre and radius with the sign the
    // blend moves the volume by and the side of the rim the ball is on.
    type Build = fn(&mut Model) -> Body;
    type Rim = (Point3, f64, f64, f64);
    let cases: Vec<(&str, Build, Vec<Rim>)> = vec![
        (
            "hole top rim",
            holed_plate,
            vec![(Point3::new(2.0, 2.0, 1.0), 1.0, -1.0, 1.0)],
        ),
        (
            "hole bottom rim",
            holed_plate,
            vec![(Point3::new(2.0, 2.0, 0.0), 1.0, -1.0, 1.0)],
        ),
        (
            "both hole rims",
            holed_plate,
            vec![
                (Point3::new(2.0, 2.0, 1.0), 1.0, -1.0, 1.0),
                (Point3::new(2.0, 2.0, 0.0), 1.0, -1.0, 1.0),
            ],
        ),
        (
            "boss base",
            bossed_disc,
            vec![(Point3::new(0.0, 0.0, 1.0), 1.0, 1.0, 1.0)],
        ),
        (
            "boss top rim",
            bossed_disc,
            vec![(Point3::new(0.0, 0.0, 2.0), 1.0, -1.0, -1.0)],
        ),
        (
            "disc outer rim",
            bossed_disc,
            vec![(Point3::new(0.0, 0.0, 1.0), 3.0, -1.0, -1.0)],
        ),
    ];
    for (name, build, rims) in cases {
        let mut m = Model::default();
        let body = build(&mut m);
        let before = mass_properties(&m, body).unwrap().volume;
        let edges: Vec<Edge> = rims
            .iter()
            .map(|&(c, radius, _, _)| rim_at(&m, body, c, radius))
            .collect();
        let (blended, provenance) =
            fillet(&mut m, body, &edges, r).unwrap_or_else(|err| panic!("{name}: {err}"));
        let report = check(&m, blended, Level::Full);
        assert!(report.is_ok(), "{name}: {report}");
        assert!(report.unchecked().is_empty(), "{name}: {report}");
        audit(&m, &[body], blended, &provenance).unwrap();
        for edge in &edges {
            let edge = Shape::new(edge.id, Orientation::Forward);
            let generated = provenance.generated_from(edge);
            let count =
                |pick: fn(&EntityId) -> bool| generated.iter().filter(|s| pick(&s.id)).count();
            assert_eq!(
                (
                    count(|id| matches!(id, EntityId::Face(_))),
                    count(|id| matches!(id, EntityId::Edge(_))),
                    count(|id| matches!(id, EntityId::Vertex(_)))
                ),
                (1, 3, 2),
                "{name}: {generated:?}"
            );
            assert!(provenance.is_deleted(edge), "{name}");
        }
        let volume = before
            + rims
                .iter()
                .map(|&(_, radius, sign, side)| sign * TAU * (radius + side * reach) * section)
                .sum::<f64>();
        let got = mass_properties(&m, blended).unwrap().volume;
        assert!(
            (got - volume).abs() <= 1e-9 * volume,
            "{name}: {got} against {volume}"
        );
    }
}

/// A closed rim's refusals: a ball whose torus would not be a ring — its
/// centre circle no wider than its tube, at the boss's top rim — and a
/// ball wider than the plate the hole goes through.
#[test]
fn a_rim_blend_too_large_is_refused() {
    let mut m = Model::default();
    let disc = bossed_disc(&mut m);
    let top = rim_at(&m, disc, Point3::new(0.0, 0.0, 2.0), 1.0);
    let err = fillet(&mut m, disc, &[top], 0.5).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
    fillet(&mut m, disc, &[top], 0.45).unwrap();
    let plate = holed_plate(&mut m);
    let rim = rim_at(&m, plate, Point3::new(2.0, 2.0, 1.0), 1.0);
    let err = fillet(&mut m, plate, &[rim], 1.2).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
}

/// The cone row's refusals (ADR-0036 §3): a pointed cone boss on a wide
/// disc, radius 2 at its base and its apex 2 above, its base rim at 135°.
/// A ball of radius 1 builds; one of radius 8 touches the cone 8·tan(π/8)
/// ≈ 3.3 up the ruling, past the apex at 2√2 ≈ 2.8, and a chamfer 4 along
/// it likewise: `BlendTooLarge`, naming the edge and the cone.
#[test]
fn a_contact_past_a_cones_apex_is_too_large() {
    let mut m = Model::default();
    let p = |u, v| Point2::new(u, v);
    let plane = Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap();
    let profile = Profile {
        plane,
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                line_to(20.0, 0.0),
                line_to(20.0, 1.0),
                line_to(2.0, 1.0),
                line_to(0.0, 3.0),
                line_to(0.0, 0.0),
            ],
        },
        holes: Vec::new(),
    };
    let part = revolve(
        &mut m,
        &profile,
        Axis::z_at(Point3::origin()),
        core::f64::consts::TAU,
    )
    .unwrap()
    .0;
    let base = rim_at(&m, part, Point3::new(0.0, 0.0, 1.0), 2.0);
    fillet(&mut m, part, &[base], 1.0).unwrap();
    let err = fillet(&mut m, part, &[base], 8.0).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
    let err = chamfer(&mut m, part, &[base], 4.0).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
}

/// The ruling arm's refusals: a ruling blend meeting a plane–plane blend
/// at a corner, whose contact on the cylinder misses the other's on the
/// third edge — two arcs, a vertex blend — and a ball larger than the
/// half disc it rolls in.
#[test]
fn a_ruling_blend_refuses_a_miter_and_a_ball_too_large() {
    let mut m = Model::default();
    let d = extruded(
        &mut m,
        (-1.5, 0.0),
        vec![line_to(1.5, 0.0), arc_to(-1.5, 0.0, (0.0, 1.5))],
    );
    let ruling = edge_at(&m, d, Point3::new(1.5, 0.0, 1.0));
    let cap = edge_at(&m, d, Point3::new(0.0, 0.0, 2.0));
    let err = fillet(&mut m, d, &[ruling, cap], 0.2).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::VertexBlend), "{err}");
    let err = fillet(&mut m, d, &[ruling], 1.6).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::BlendTooLarge), "{err}");
}

/// The miter (ADR-0007): the vertical and the cap edge at one corner of
/// the 2-cube blended in one call, the fixture `blend/fillet-miter`
/// held to the oracle's numbers here as well: `Full` is clean with nothing
/// unchecked, the two blend cylinders with crossing axes of equal radius
/// decided by the cylinder–cylinder arm; counts, volume, area, centroid
/// and every probe are the oracle's; the miter edge and its two vertices
/// are generated from both edges; the record audits; and the two edges in
/// either order build the same result.
#[test]
fn two_edges_at_a_vertex_meet_in_a_miter() {
    let dir = fixtures::corpus_root().join("blend/fillet-miter");
    let fixture = fixtures::load(&dir).unwrap();
    let chain = corpus::chain(&dir, "default").unwrap();
    let (m, blended) = (&chain.model, chain.result().unwrap());
    let result = &chain.steps["result"];

    let report = check(m, blended, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");

    let expected = &fixture.expected.results["default"];
    let line = report.euler().unwrap();
    assert_eq!(
        (
            line.vertices,
            line.edges,
            line.faces,
            line.loops,
            line.shells,
            line.genus
        ),
        (
            expected.counts.vertices,
            expected.counts.edges,
            expected.counts.faces,
            expected.counts.loops,
            expected.counts.shells,
            expected.genus.unwrap()
        )
    );
    let tolerances = fixture.recipe.tolerances;
    let props = mass_properties(m, blended).unwrap();
    let (volume, area) = (expected.volume.unwrap(), expected.area.unwrap());
    assert!(
        (props.volume - volume).abs() <= tolerances.volume_rel * volume,
        "volume {} vs the oracle's {volume}",
        props.volume
    );
    assert!(
        (props.area - area).abs() <= tolerances.area_rel * area,
        "area {} vs the oracle's {area}",
        props.area
    );
    let centroid = expected.centroid.unwrap();
    let centroid = Point3::new(centroid[0], centroid[1], centroid[2]);
    assert!(
        (props.centroid - centroid).norm() <= tolerances.centroid_abs,
        "centroid {} vs the oracle's {centroid}",
        props.centroid
    );
    for probe in &expected.probes {
        let point = Point3::new(probe.point[0], probe.point[1], probe.point[2]);
        let found = match classify_point(m, blended, point).unwrap() {
            Classification::Inside => Class::In,
            Classification::Outside => Class::Out,
            Classification::On(_) => Class::On,
        };
        assert_eq!(found, probe.class, "probe {}", probe.label);
    }

    // Provenance: the miter edge and its two vertices from both edges,
    // the rest of each blend from its own, the third edge shortened.
    audit(m, &result.inputs, blended, &result.provenance).unwrap();
    let cube = result.inputs[0];
    let vertical = edge_at(m, cube, Point3::new(2.0, 2.0, 1.0));
    let cap = edge_at(m, cube, Point3::new(1.0, 2.0, 2.0));
    let third = edge_at(m, cube, Point3::new(2.0, 1.0, 2.0));
    let fwd = |e: Edge| Shape::new(e.id, Orientation::Forward);
    let shared = result.provenance.generated_pair(fwd(vertical), fwd(cap));
    let (edges, vertices): (Vec<Shape>, Vec<Shape>) = shared
        .iter()
        .copied()
        .partition(|s| matches!(s.id, EntityId::Edge(_)));
    assert_eq!((edges.len(), vertices.len()), (1, 2), "{shared:?}");
    for edge in [vertical, cap] {
        let generated = result.provenance.generated_from(fwd(edge));
        assert_eq!(generated.len(), 9, "{generated:?}");
    }
    assert_eq!(result.provenance.modified_from(fwd(third)).len(), 1);
    let corner = m.edge(vertical.id).unwrap().end();
    assert!(
        result
            .provenance
            .is_deleted(Shape::new(corner, Orientation::Forward))
    );

    // The same set in the other order is the same result.
    let mut again = Model::default();
    let cube2 = cube_body(&mut again);
    let vertical2 = edge_at(&again, cube2, Point3::new(2.0, 2.0, 1.0));
    let cap2 = edge_at(&again, cube2, Point3::new(1.0, 2.0, 2.0));
    let (blended2, provenance2) = fillet(&mut again, cube2, &[cap2, vertical2], 0.2).unwrap();
    assert_eq!(
        dump_text(m, blended).unwrap(),
        dump_text(&again, blended2).unwrap()
    );
    assert_eq!(result.provenance, provenance2);
}

/// A second fillet on a filleted body, `blend/second-fillet`: the
/// extruded square's rise at (2, 0) filleted, then the rise at (2, 2) of
/// that result. Each step's record audits, and the three records composed
/// with `Provenance::then` — in either bracketing — name every face of the
/// result from one `SweepPart` of the extrude: the six faces from their
/// caps and sides, the side face both blends trimmed still from its
/// segment, and each blend face with its two contacts, two arcs and four
/// vertices from the rise its edge was.
#[test]
fn a_second_fillet_composes_back_to_the_extrude() {
    let dir = fixtures::corpus_root().join("blend/second-fillet");
    let chain = corpus::chain(&dir, "default").unwrap();
    let m = &chain.model;
    let [cube, first, second] = ["cube", "first", "result"].map(|name| &chain.steps[name]);
    for step in [first, second] {
        audit(m, &step.inputs, step.body, &step.provenance).unwrap();
    }
    let whole = cube
        .provenance
        .then(&first.provenance)
        .then(&second.provenance);
    assert_eq!(
        whole,
        cube.provenance
            .then(&first.provenance.then(&second.provenance))
    );

    let side = |segment| SweepPart::Side {
        loop_index: 0,
        segment,
    };
    let rise = |vertex| SweepPart::Rise {
        loop_index: 0,
        vertex,
    };
    let mut expected: Vec<SweepPart> = vec![SweepPart::StartCap, SweepPart::EndCap];
    expected.extend((0..4).map(side));
    expected.extend([rise(1), rise(2)]);
    expected.sort();
    let mut found: Vec<SweepPart> = Vec::new();
    for face in m.faces(second.body).unwrap() {
        let origins = whole.origins(Shape::new(face.id, Orientation::Forward));
        let [(Relation::Generated, Origin::Role(Role::Extrude(part)))] = origins[..] else {
            panic!("{} from {origins:?}", face.id);
        };
        found.push(part);
    }
    found.sort();
    assert_eq!(found, expected);

    // Each blend whole from its rise; the shared side face one face.
    for vertex in [1, 2] {
        let generated = whole.generated_from(Role::Extrude(rise(vertex)));
        let count = |pick: fn(&EntityId) -> bool| generated.iter().filter(|s| pick(&s.id)).count();
        assert_eq!(
            (
                count(|id| matches!(id, EntityId::Face(_))),
                count(|id| matches!(id, EntityId::Edge(_))),
                count(|id| matches!(id, EntityId::Vertex(_)))
            ),
            (1, 4, 4),
            "rise {vertex}: {generated:?}"
        );
    }
    let shared = whole.generated_from(Role::Extrude(side(1)));
    assert_eq!(shared.len(), 1, "{shared:?}");
}

fn cube_body(m: &mut Model) -> Body {
    cube(m, 2.0)
}

/// Two cap edges at a corner: the same solid rotated, so the same
/// numbers as the vertical-plus-cap miter.
#[test]
fn two_cap_edges_are_the_same_miter_rotated() {
    let mut m = Model::default();
    let body = cube(&mut m, 2.0);
    let along_x = edge_at(&m, body, Point3::new(1.0, 2.0, 2.0));
    let along_y = edge_at(&m, body, Point3::new(2.0, 1.0, 2.0));
    let (blended, provenance) = fillet(&mut m, body, &[along_x, along_y], 0.2).unwrap();
    let report = check(&m, blended, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    let line = report.euler().unwrap();
    assert_eq!(
        (line.vertices, line.edges, line.faces, line.loops),
        (11, 17, 8, 8)
    );
    let r: f64 = 0.2;
    let props = mass_properties(&m, blended).unwrap();
    let volume = 8.0 - 4.0 * (1.0 - core::f64::consts::FRAC_PI_4) * r * r
        + (5.0 / 3.0 - core::f64::consts::FRAC_PI_2) * r * r * r;
    assert!(
        (props.volume - volume).abs() <= 1e-9 * volume,
        "{}",
        props.volume
    );
    let area = 4.0 * (2.0 - r)
        + (2.0 - r) * (2.0 - r)
        + 4.0
        + 2.0 * (4.0 - (1.0 - core::f64::consts::FRAC_PI_4) * r * r)
        + 2.0 * (core::f64::consts::FRAC_PI_2 * r * (2.0 - r) + r * r);
    assert!((props.area - area).abs() <= 1e-9 * area, "{}", props.area);
    audit(&m, &[body], blended, &provenance).unwrap();
    // The third edge, the vertical one, is shortened to z = 2 − r.
    let vertical = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let [shortened] = provenance.modified_from(Shape::new(vertical.id, Orientation::Forward))
    else {
        panic!("{provenance}");
    };
    let EntityId::Edge(id) = shortened.id else {
        panic!("{shortened:?}")
    };
    let entity = m.edge(id).unwrap();
    let (curve, range) = entity.curve().unwrap();
    let top = m.curve(curve).unwrap().point(range.hi());
    assert!((top - Point3::new(2.0, 2.0, 1.8)).norm() < 1e-9, "{top}");
}

/// A corner whose two blended edges have different dihedrals — the
/// slanted vertical edge of an extruded parallelogram and its cap edge
/// — is not one ellipse: the two far contacts meet the third edge at two
/// points. Refused by name, the model untouched (the blend-network
/// cycle's).
#[test]
fn a_miter_of_unequal_dihedrals_is_a_vertex_blend() {
    let mut m = Model::default();
    let p = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, 0.0),
            segments: vec![
                ProfileSegment::LineTo(p(2.0, 0.0)),
                ProfileSegment::LineTo(p(3.0, 2.0)),
                ProfileSegment::LineTo(p(1.0, 2.0)),
                ProfileSegment::LineTo(p(0.0, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let prism = extrude(&mut m, &profile, Vec3::z(), 2.0).unwrap().0;
    let vertical = edge_at(&m, prism, Point3::new(3.0, 2.0, 1.0));
    let cap = edge_at(&m, prism, Point3::new(2.0, 2.0, 2.0));
    let before = dump_text(&m, prism).unwrap();
    let err = fillet(&mut m, prism, &[vertical, cap], 0.2).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::VertexBlend), "{err}");
    assert_eq!(dump_text(&m, prism).unwrap(), before);
    // Each edge alone blends.
    fillet(&mut m, prism, &[vertical], 0.2).unwrap();
    fillet(&mut m, prism, &[cap], 0.2).unwrap();
}

/// A chain runs on through a vertex of four edges where both of its faces
/// turn tangentially (ADR-0039 §1): the foot of a chamfered stadium's
/// chamfer, between the side plane and the chamfer strip, meets the foot
/// of the half cone over the half cylinder at a vertex whose two other
/// edges — side plane to half cylinder, strip to half cone — are tangent
/// dihedrals. A fillet or a chamfer of the foot line walks on into the
/// arc, and the junction, its runs sharing no face, is refused by name
/// with both edges and the vertex until it is built (`plans/blend-corners`
/// step 4), the model untouched. In two poses.
#[test]
fn a_chain_runs_on_where_both_faces_turn() {
    let turn =
        UnitQuaternion::from_axis_angle(&Unit::new_normalize(Vec3::new(0.3, -0.5, 0.8)), 1.1);
    let d = 0.25;
    for pose in [
        Frame::world(),
        Frame::from_rotation(Point3::new(120.0, -75.0, 40.0), &turn),
    ] {
        let mut m = Model::default();
        let at = |x, y, z| pose.to_world(Point3::new(x, y, z));
        let p = |u, v| Point2::new(u, v);
        let profile = Profile {
            plane: pose,
            outer: ProfileLoop::Path {
                start: p(0.0, -1.0),
                segments: vec![
                    ProfileSegment::LineTo(p(2.0, -1.0)),
                    ProfileSegment::ArcTo {
                        to: p(2.0, 1.0),
                        via: p(3.0, 0.0),
                    },
                    ProfileSegment::LineTo(p(0.0, 1.0)),
                    ProfileSegment::ArcTo {
                        to: p(0.0, -1.0),
                        via: p(-1.0, 0.0),
                    },
                ],
            },
            holes: Vec::new(),
        };
        let direction = pose.vec_to_world(Vec3::z());
        let stadium = extrude(&mut m, &profile, direction, 1.0).unwrap().0;
        let top = edge_at(&m, stadium, at(1.0, 1.0, 1.0));
        let chamfered = chamfer(&mut m, stadium, &[top], d).unwrap().0;
        let foot = edge_at(&m, chamfered, at(1.0, 1.0, 1.0 - d));
        let before = dump_text(&m, chamfered).unwrap();
        for blend in [fillet, chamfer] {
            let err = blend(&mut m, chamfered, &[foot], 0.1).unwrap_err();
            let OpError::Degenerate { entities, reason } = &err else {
                panic!("{err}");
            };
            assert_eq!(*reason, Reason::VertexBlend, "{err}");
            let [a, b, v] = entities[..] else {
                panic!("two edges and a vertex: {err}");
            };
            let (EntityId::Edge(ea), EntityId::Edge(eb), EntityId::Vertex(v)) = (a.id, b.id, v.id)
            else {
                panic!("two edges and a vertex: {err}");
            };
            // The closed chain's first junction in vertex order: a foot
            // line and a foot arc, at one of the four corners of the foot.
            let circles = [ea, eb]
                .iter()
                .filter(|&&e| {
                    let (curve, _) = m.edge(e).unwrap().curve().unwrap();
                    matches!(m.curve(curve).unwrap(), Curve::Circle { .. })
                })
                .count();
            assert_eq!(circles, 1, "{err}");
            let corner = m.vertex(v).unwrap().point();
            let corners = [(0.0, 1.0), (2.0, 1.0), (0.0, -1.0), (2.0, -1.0)];
            assert!(
                corners
                    .iter()
                    .any(|&(x, y)| (corner - at(x, y, 1.0 - d)).norm() < 1e-9),
                "{corner:?}"
            );
            assert_eq!(m.vertex_edges(v).unwrap().len(), 4);
        }
        assert_eq!(dump_text(&m, chamfered).unwrap(), before);
    }
}

/// Three fillets at a box corner meet in a sphere octant about the ball's
/// one centre (ADR-0007), tangent to each blend cylinder along a great
/// circle, its pole a degenerate edge where the cap's two contacts cross.
/// Clean at `Full` with nothing unchecked, at the closed-form volume and
/// area, the sphere face and its pole generated from each of the three
/// edges, the record audited, and the same for the edges in either order.
#[test]
fn three_edges_at_a_box_corner_meet_in_a_sphere() {
    let corner = |m: &mut Model| {
        let body = cube(m, 2.0);
        let edges = [
            edge_at(m, body, Point3::new(2.0, 2.0, 1.0)),
            edge_at(m, body, Point3::new(1.0, 2.0, 2.0)),
            edge_at(m, body, Point3::new(2.0, 1.0, 2.0)),
        ];
        (body, edges)
    };
    let mut m = Model::default();
    let (body, edges) = corner(&mut m);
    let (blended, provenance) = fillet(&mut m, body, &edges, 0.2).unwrap();

    let report = check(&m, blended, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    let line = report.euler().unwrap();
    assert_eq!(
        (
            line.vertices,
            line.edges,
            line.faces,
            line.loops,
            line.genus
        ),
        (13, 21, 10, 10, 0)
    );
    audit(&m, &[body], blended, &provenance).unwrap();

    let r: f64 = 0.2;
    let props = mass_properties(&m, blended).unwrap();
    let volume = 8.0
        - 3.0 * (1.0 - core::f64::consts::FRAC_PI_4) * r * r * (2.0 - r)
        - (1.0 - core::f64::consts::PI / 6.0) * r.powi(3);
    assert!(
        (props.volume - volume).abs() <= 1e-9 * volume,
        "{}",
        props.volume
    );
    let area =
        24.0 - 12.0 * r + 3.0 * core::f64::consts::PI * r - core::f64::consts::FRAC_PI_4 * r * r;
    assert!((props.area - area).abs() <= 1e-9 * area, "{}", props.area);

    for edge in edges {
        let generated = provenance.generated_from(Shape::new(edge.id, Orientation::Forward));
        assert_eq!(
            generated.len(),
            11,
            "the blend and the sphere, two contacts, two arcs, the pole and four vertices"
        );
        let spheres = generated
            .iter()
            .filter(|s| match s.id {
                EntityId::Face(f) => matches!(
                    m.surface(m.face(f).unwrap().surface()).unwrap(),
                    Surface::Sphere { .. }
                ),
                _ => false,
            })
            .count();
        let poles = generated
            .iter()
            .filter(|s| match s.id {
                EntityId::Edge(e) => m.edge(e).unwrap().curve().is_none(),
                _ => false,
            })
            .count();
        assert_eq!((spheres, poles), (1, 1));
    }

    let mut again = Model::default();
    let (body, mut edges) = corner(&mut again);
    edges.reverse();
    let (twice, _) = fillet(&mut again, body, &edges, 0.2).unwrap();
    assert_eq!(
        dump_text(&again, twice).unwrap(),
        dump_text(&m, blended).unwrap()
    );
}

/// The cube with its corner at `(2, 2, 2)` cut off by the plane
/// `x + y + z = 5.4`: at the triangle's corner `(2, 2, 1.4)` the vertical
/// edge meets two edges of the cut, and no face there is square to the
/// other two.
fn cut_corner(m: &mut Model) -> Body {
    let body = cube(m, 2.0);
    let n = Vec3::new(1.0, 1.0, 1.0).normalize();
    let p = |u, v| Point2::new(u, v);
    let profile = Profile {
        plane: Frame::new(Point3::new(1.8, 1.8, 1.8), n, Vec3::new(1.0, -1.0, 0.0)).unwrap(),
        outer: ProfileLoop::Path {
            start: p(-3.0, -3.0),
            segments: vec![
                ProfileSegment::LineTo(p(3.0, -3.0)),
                ProfileSegment::LineTo(p(3.0, 3.0)),
                ProfileSegment::LineTo(p(-3.0, 3.0)),
                ProfileSegment::LineTo(p(-3.0, -3.0)),
            ],
        },
        holes: Vec::new(),
    };
    let tool = extrude(m, &profile, n, 2.0).unwrap().0;
    cut(m, body, tool).unwrap().0
}

/// Three fillets at a corner the sphere's exact sides do not cover are
/// refused by name, the model untouched (the blend-network cycle's): the
/// L's reflex top
/// corner, its rise concave and its top edges convex, and the corner of a
/// cut where no face is square to the other two, so no meridian frame
/// makes every side a line in (u, v).
#[test]
fn a_corner_of_mixed_blends_or_no_square_face_is_a_vertex_blend() {
    let mut m = Model::default();
    let ell = ell(&mut m);
    let reflex = [
        edge_at(&m, ell, Point3::new(1.0, 1.0, 1.0)),
        edge_at(&m, ell, Point3::new(1.5, 1.0, 2.0)),
        edge_at(&m, ell, Point3::new(1.0, 1.5, 2.0)),
    ];
    let before = dump_text(&m, ell).unwrap();
    let err = fillet(&mut m, ell, &reflex, 0.1).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::VertexBlend), "{err}");
    assert_eq!(dump_text(&m, ell).unwrap(), before);

    let cornered = cut_corner(&mut m);
    let oblique = [
        edge_at(&m, cornered, Point3::new(2.0, 2.0, 0.7)),
        edge_at(&m, cornered, Point3::new(1.7, 2.0, 1.7)),
        edge_at(&m, cornered, Point3::new(2.0, 1.7, 1.7)),
    ];
    let before = dump_text(&m, cornered).unwrap();
    let err = fillet(&mut m, cornered, &oblique, 0.1).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::VertexBlend), "{err}");
    assert_eq!(dump_text(&m, cornered).unwrap(), before);
}
