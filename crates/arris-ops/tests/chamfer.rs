//! `ops::chamfer` (ADR-0007): one box edge clean at `Full` with nothing
//! unchecked, at its closed-form volume and area, its record rooted at
//! the edge and audited; the miter's chamfer twin, two planes meeting in
//! a line, fully checked; a concave edge; and the refusals a chamfer
//! words differently from a fillet.

use arris_debug::dump_text;
use arris_debug::unmetered::cut;
use arris_debug::unmetered::{chamfer, extrude, mass_properties, primitive_box, revolve};
use arris_ops::arris_check::arris_topo::arris_geom::{Curve, Profile, ProfileLoop, ProfileSegment};
use arris_ops::arris_check::arris_topo::arris_math::{Axis, Frame, Point2, Point3, Vec3};
use arris_ops::arris_check::arris_topo::provenance::audit;
use arris_ops::arris_check::arris_topo::{Body, Edge, EntityId, Model, Orientation, Shape};
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

fn cube(m: &mut Model) -> Body {
    primitive_box(m, Point3::origin(), Point3::new(2.0, 2.0, 2.0))
        .unwrap()
        .0
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

/// A plane against a cylinder along a circle chamfers to a 45° cone
/// (ADR-0007): a hole's top rim, convex with the material outside the
/// cylinder, a boss's base, concave, and its top rim, convex with the
/// material inside. Each is clean at `Full` with nothing unchecked and
/// audited, and moves the volume by Pappus: the triangle `d²/2` swept
/// about the axis at its centroid, `d/3` from the edge toward the cut.
#[test]
fn a_rim_chamfers_to_a_cone() {
    use core::f64::consts::TAU;
    let d: f64 = 0.25;
    let p = |u, v| Point2::new(u, v);
    let mut m = Model::default();
    let plate = extrude(
        &mut m,
        &Profile {
            plane: Frame::world(),
            outer: ProfileLoop::Path {
                start: p(0.0, 0.0),
                segments: [p(4.0, 0.0), p(4.0, 4.0), p(0.0, 4.0), p(0.0, 0.0)]
                    .map(ProfileSegment::LineTo)
                    .to_vec(),
            },
            holes: vec![ProfileLoop::Circle {
                center: p(2.0, 2.0),
                radius: 1.0,
            }],
        },
        Vec3::z(),
        1.0,
    )
    .unwrap()
    .0;
    let disc = revolve(
        &mut m,
        &Profile {
            plane: Frame::new(Point3::origin(), -Vec3::y(), Vec3::x()).unwrap(),
            outer: ProfileLoop::Path {
                start: p(0.0, 0.0),
                segments: [
                    p(3.0, 0.0),
                    p(3.0, 1.0),
                    p(1.0, 1.0),
                    p(1.0, 2.0),
                    p(0.0, 2.0),
                    p(0.0, 0.0),
                ]
                .map(ProfileSegment::LineTo)
                .to_vec(),
            },
            holes: Vec::new(),
        },
        Axis::z_at(Point3::origin()),
        TAU,
    )
    .unwrap()
    .0;
    // A name, the body, the rim, the sign the chamfer moves the volume by
    // and the side of the rim the cut is on.
    let cases = [
        ("hole top rim", plate, Point3::new(2.0, 2.0, 1.0), -1.0, 1.0),
        ("boss base", disc, Point3::new(0.0, 0.0, 1.0), 1.0, 1.0),
        ("boss top rim", disc, Point3::new(0.0, 0.0, 2.0), -1.0, -1.0),
    ];
    for (name, body, centre, sign, side) in cases {
        let before = mass_properties(&m, body).unwrap().volume;
        let rim = rim_at(&m, body, centre, 1.0);
        let (chamfered, provenance) =
            chamfer(&mut m, body, &[rim], d).unwrap_or_else(|err| panic!("{name}: {err}"));
        let report = check(&m, chamfered, Level::Full);
        assert!(report.is_ok(), "{name}: {report}");
        assert!(report.unchecked().is_empty(), "{name}: {report}");
        audit(&m, &[body], chamfered, &provenance).unwrap();
        let generated = provenance.generated_from(Shape::new(rim.id, Orientation::Forward));
        assert_eq!(generated.len(), 6, "{name}: {generated:?}");
        let volume = before + sign * TAU * (1.0 + side * d / 3.0) * d * d / 2.0;
        let got = mass_properties(&m, chamfered).unwrap().volume;
        assert!(
            (got - volume).abs() <= 1e-9 * volume,
            "{name}: {got} against {volume}"
        );
    }
}

/// A plane against a cylinder along a ruling — a half disc's chord edge —
/// has a fillet in the table and no chamfer, and is named as such.
#[test]
fn a_ruling_edge_has_no_chamfer() {
    let mut m = Model::default();
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: Point2::new(-1.5, 0.0),
            segments: vec![
                ProfileSegment::LineTo(Point2::new(1.5, 0.0)),
                ProfileSegment::ArcTo {
                    to: Point2::new(-1.5, 0.0),
                    via: Point2::new(0.0, 1.5),
                },
            ],
        },
        holes: Vec::new(),
    };
    let d = extrude(&mut m, &profile, Vec3::z(), 2.0).unwrap().0;
    let edge = edge_at(&m, d, Point3::new(1.5, 0.0, 1.0));
    let err = chamfer(&mut m, d, &[edge], 0.2).unwrap_err();
    assert!(matches!(err, OpError::Unsupported { .. }), "{err}");
}

/// The polygon `points` in the world's XY plane extruded 2 along +Z.
fn prism(m: &mut Model, points: &[(f64, f64)]) -> Body {
    let p = |(u, v): (f64, f64)| Point2::new(u, v);
    let mut segments: Vec<ProfileSegment> = points[1..]
        .iter()
        .map(|&q| ProfileSegment::LineTo(p(q)))
        .collect();
    segments.push(ProfileSegment::LineTo(p(points[0])));
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(points[0]),
            segments,
        },
        holes: Vec::new(),
    };
    extrude(m, &profile, Vec3::z(), 2.0).unwrap().0
}

fn reason(err: &OpError) -> Option<Reason> {
    match err {
        OpError::Degenerate { reason, .. } => Some(*reason),
        _ => None,
    }
}

fn forward(e: Edge) -> Shape {
    Shape::new(e.id, Orientation::Forward)
}

/// `body` clean at `Full` with nothing unchecked, with the Euler line's
/// `(V, E, F, L)` and `volume` to 1e-9 relative.
fn assert_clean(m: &Model, body: Body, counts: (usize, usize, usize, usize), volume: f64) {
    let report = check(m, body, Level::Full);
    assert!(report.is_ok(), "{report}");
    assert!(report.unchecked().is_empty(), "{report}");
    let line = report.euler().unwrap();
    assert_eq!(
        (line.vertices, line.edges, line.faces, line.loops),
        counts,
        "{report}"
    );
    let found = mass_properties(m, body).unwrap().volume;
    assert!(
        (found - volume).abs() <= 1e-9 * volume,
        "{found} vs {volume}"
    );
}

const D: f64 = 0.2;

/// The consumer's number: a 2-cube with one vertical edge chamfered at
/// `d = 0.2` loses a right prism of `d²/2 · 2`.
#[test]
fn one_box_edge_end_to_end() {
    let mut m = Model::default();
    let body = cube(&mut m);
    let edge = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let (chamfered, provenance) = chamfer(&mut m, body, &[edge], D).unwrap();
    assert_clean(&m, chamfered, (10, 15, 7, 7), 7.96);
    let area = mass_properties(&m, chamfered).unwrap().area;
    let expected = 24.0 - 4.0 * D - D * D + 2.0 * core::f64::consts::SQRT_2 * D;
    assert!((area - expected).abs() <= 1e-9 * expected, "{area}");

    audit(&m, &[body], chamfered, &provenance).unwrap();
    let generated = provenance.generated_from(edge.shape());
    let faces = generated
        .iter()
        .filter(|s| matches!(s.id, EntityId::Face(_)))
        .count();
    assert_eq!((generated.len(), faces), (9, 1), "{provenance}");
    assert!(provenance.is_deleted(edge.shape()));

    let mut again = Model::default();
    let body2 = cube(&mut again);
    let edge2 = edge_at(&again, body2, Point3::new(2.0, 2.0, 1.0));
    let (chamfered2, provenance2) = chamfer(&mut again, body2, &[edge2], D).unwrap();
    assert_eq!(
        dump_text(&m, chamfered).unwrap(),
        dump_text(&again, chamfered2).unwrap()
    );
    assert_eq!(provenance, provenance2);
}

/// The miter's chamfer twin: the vertical and the cap edge at the corner
/// (2, 2, 2). The two planes meet in the line from (2 − d, 2, 2 − d) to
/// (2, 2 − d, 2), `Generated` from both edges; the third edge ends at the
/// second point; the prisms' overlap `d³/3` is counted once; and with no
/// cylinder in it the result is checked at `Full` with nothing unchecked.
/// Either order of the two edges is the same result.
#[test]
fn two_chamfers_at_a_corner_meet_in_a_line() {
    let build = |reversed: bool| {
        let mut m = Model::default();
        let body = cube(&mut m);
        let vertical = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
        let cap = edge_at(&m, body, Point3::new(1.0, 2.0, 2.0));
        let edges = if reversed {
            [cap, vertical]
        } else {
            [vertical, cap]
        };
        let (chamfered, provenance) = chamfer(&mut m, body, &edges, D).unwrap();
        (m, body, chamfered, provenance)
    };
    let (m, body, chamfered, provenance) = build(false);
    assert_clean(
        &m,
        chamfered,
        (11, 17, 8, 8),
        8.0 - 2.0 * D * D + D * D * D / 3.0,
    );
    audit(&m, &[body], chamfered, &provenance).unwrap();

    let vertical = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let cap = edge_at(&m, body, Point3::new(1.0, 2.0, 2.0));
    let shared = provenance.generated_pair(forward(vertical), forward(cap));
    let lines: Vec<_> = shared
        .iter()
        .filter_map(|s| match s.id {
            EntityId::Edge(id) => Some(id),
            _ => None,
        })
        .collect();
    let [line] = lines[..] else {
        panic!("one corner line: {shared:?}");
    };
    assert_eq!(shared.len(), 3, "the line and its two vertices");
    let (curve, range) = m.edge(line).unwrap().curve().unwrap();
    let curve = m.curve(curve).unwrap();
    let mut ends = [curve.point(range.lo()), curve.point(range.hi())];
    ends.sort_by(|a, b| a.x.total_cmp(&b.x));
    assert!((ends[0] - Point3::new(2.0 - D, 2.0, 2.0 - D)).norm() < 1e-9);
    assert!((ends[1] - Point3::new(2.0, 2.0 - D, 2.0)).norm() < 1e-9);

    let third = edge_at(&m, body, Point3::new(2.0, 1.0, 2.0));
    let [shortened] = provenance.modified_from(forward(third)) else {
        panic!("{provenance}");
    };
    let EntityId::Edge(id) = shortened.id else {
        panic!("{shortened:?}");
    };
    let (curve, range) = m.edge(id).unwrap().curve().unwrap();
    let curve = m.curve(curve).unwrap();
    let far_y = curve.point(range.lo()).y.max(curve.point(range.hi()).y);
    assert!((far_y - (2.0 - D)).abs() < 1e-9, "{far_y}");

    let (again, _, chamfered2, provenance2) = build(true);
    assert_eq!(
        dump_text(&m, chamfered).unwrap(),
        dump_text(&again, chamfered2).unwrap()
    );
    assert_eq!(provenance, provenance2);
}

/// A concave edge adds a prism: the inner edge of an extruded L gains
/// `d²/2 · 2`.
#[test]
fn a_concave_edge_adds_a_prism() {
    let mut m = Model::default();
    let body = prism(
        &mut m,
        &[
            (0.0, 0.0),
            (2.0, 0.0),
            (2.0, 1.0),
            (1.0, 1.0),
            (1.0, 2.0),
            (0.0, 2.0),
        ],
    );
    let edge = edge_at(&m, body, Point3::new(1.0, 1.0, 1.0));
    let (chamfered, provenance) = chamfer(&mut m, body, &[edge], D).unwrap();
    assert_clean(&m, chamfered, (14, 21, 9, 9), 6.0 + D * D);
    audit(&m, &[body], chamfered, &provenance).unwrap();
}

/// A distance that is not finite or not positive is named as the
/// distance, the model untouched.
#[test]
fn a_bad_distance_is_refused_by_name() {
    let mut m = Model::default();
    let body = cube(&mut m);
    let edge = edge_at(&m, body, Point3::new(2.0, 2.0, 1.0));
    let before = dump_text(&m, body).unwrap();
    assert!(matches!(
        reason(&chamfer(&mut m, body, &[edge], f64::INFINITY).unwrap_err()),
        Some(Reason::NonFinite { what: "distance" })
    ));
    assert!(matches!(
        reason(&chamfer(&mut m, body, &[edge], -0.2).unwrap_err()),
        Some(Reason::NotPositive {
            what: "distance",
            ..
        })
    ));
    assert_eq!(dump_text(&m, body).unwrap(), before);
}

/// Two chamfers at a corner whose edges make unequal angles with its
/// third edge — the slanted vertical edge of an extruded parallelogram
/// and its cap edge, 90° and 63.4° to the top of the slanted face — have
/// far contacts that meet it at two points: refused by name, the model
/// untouched (the blend-network cycle's). Each edge alone chamfers.
#[test]
fn a_corner_of_unequal_angles_is_a_vertex_blend() {
    let mut m = Model::default();
    let body = prism(&mut m, &[(0.0, 0.0), (2.0, 0.0), (3.0, 2.0), (1.0, 2.0)]);
    let vertical = edge_at(&m, body, Point3::new(3.0, 2.0, 1.0));
    let cap = edge_at(&m, body, Point3::new(2.0, 2.0, 2.0));
    let before = dump_text(&m, body).unwrap();
    let err = chamfer(&mut m, body, &[vertical, cap], D).unwrap_err();
    assert_eq!(reason(&err), Some(Reason::VertexBlend), "{err}");
    assert_eq!(dump_text(&m, body).unwrap(), before);
    chamfer(&mut m, body, &[vertical], D).unwrap();
    chamfer(&mut m, body, &[cap], D).unwrap();
}

/// Three chamfers at a box corner meet in the triangle of the points
/// where each corner face's two contacts cross (ADR-0007): clean at `Full`
/// with nothing unchecked, at `8 − 3d² + 2d³/3` and its closed-form area,
/// the triangle generated from each of the three edges, the record
/// audited.
#[test]
fn three_chamfers_at_a_box_corner_meet_in_a_triangle() {
    let mut m = Model::default();
    let body = cube(&mut m);
    let edges = [
        edge_at(&m, body, Point3::new(2.0, 2.0, 1.0)),
        edge_at(&m, body, Point3::new(1.0, 2.0, 2.0)),
        edge_at(&m, body, Point3::new(2.0, 1.0, 2.0)),
    ];
    let (chamfered, provenance) = chamfer(&mut m, body, &edges, D).unwrap();

    let report = check(&m, chamfered, Level::Full);
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
    audit(&m, &[body], chamfered, &provenance).unwrap();

    let props = mass_properties(&m, chamfered).unwrap();
    let volume = 8.0 - 3.0 * D * D + 2.0 * D.powi(3) / 3.0;
    assert!(
        (props.volume - volume).abs() <= 1e-9 * volume,
        "{}",
        props.volume
    );
    let (r2, r3) = (core::f64::consts::SQRT_2, 3.0_f64.sqrt());
    let area = 24.0 - 12.0 * D + 6.0 * r2 * D + 1.5 * D * D - 3.0 * r2 * D * D + r3 / 2.0 * D * D;
    assert!((props.area - area).abs() <= 1e-9 * area, "{}", props.area);

    for edge in edges {
        let generated = provenance.generated_from(Shape::new(edge.id, Orientation::Forward));
        let faces = generated
            .iter()
            .filter(|s| matches!(s.id, EntityId::Face(_)))
            .count();
        assert_eq!(
            (generated.len(), faces),
            (10, 2),
            "the chamfer and the triangle, two contacts, two segments and four vertices"
        );
    }
}

/// Three chamfers meet in a triangle at any convex corner of three planes:
/// at the corner `(2, 2, 1.4)` of the cube cut by `x + y + z = 5.4`, where
/// no face is square to the other two and three fillets are refused, they
/// build clean at `Full` with nothing unchecked, audited.
#[test]
fn three_chamfers_at_an_oblique_corner_meet_in_a_triangle() {
    let mut m = Model::default();
    let body = cube(&mut m);
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
    let tool = extrude(&mut m, &profile, n, 2.0).unwrap().0;
    let cornered = cut(&mut m, body, tool).unwrap().0;
    let edges = [
        edge_at(&m, cornered, Point3::new(2.0, 2.0, 0.7)),
        edge_at(&m, cornered, Point3::new(1.7, 2.0, 1.7)),
        edge_at(&m, cornered, Point3::new(2.0, 1.7, 1.7)),
    ];
    let (chamfered, provenance) = chamfer(&mut m, cornered, &edges, 0.1).unwrap();
    let report = check(&m, chamfered, Level::Full);
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
        (15, 24, 11, 11, 0)
    );
    audit(&m, &[cornered], chamfered, &provenance).unwrap();
}
