//! The primitives (`docs/ARCHITECTURE.md` §Operations): clean at `Full`,
//! deterministic across runs and models, every entity under exactly one
//! `Role` and nothing `Modified` or `Deleted`, typed errors for bad
//! parameters with the model untouched, an axis normalised by
//! `Axis::new`, and both `primitive/*` fixtures matched by the oracle.

use arris_debug::unmetered::{primitive_box, primitive_cylinder};
use std::collections::BTreeSet;

use arris_debug::{dump_text, oracle, sample};
use arris_io::step;
use arris_math::{Axis, Point3, Vec3};
use arris_topo::provenance::{BoxPart, Coord, CylinderPart, Side};
use arris_topo::{Body, Model, Orientation, Provenance, Relation, Role, Shape, SurfaceId};

use arris_check::{Level, check};
use arris_ops::{OpError, Reason};
fn the_box(m: &mut Model) -> (Body, Provenance) {
    primitive_box(m, Point3::origin(), Point3::new(40.0, 30.0, 10.0)).unwrap()
}

fn the_cylinder(m: &mut Model) -> (Body, Provenance) {
    primitive_cylinder(m, Axis::z_at(Point3::origin()), 4.0, 12.0).unwrap()
}

/// Every entity of the closure and the body itself is generated from
/// exactly one role, nothing is modified or deleted, and every role's
/// output is one of those entities.
fn assert_roles(m: &Model, body: Body, p: &Provenance) {
    let c = m.closure(body).unwrap();
    let mut entities: Vec<Shape> = Vec::new();
    entities.extend(
        c.vertices
            .iter()
            .map(|&v| Shape::new(v, Orientation::Forward)),
    );
    entities.extend(c.edges.iter().map(|&e| Shape::new(e, Orientation::Forward)));
    entities.extend(c.faces.iter().map(|&f| Shape::new(f, Orientation::Forward)));
    entities.extend(
        c.shells
            .iter()
            .map(|&s| Shape::new(s, Orientation::Forward)),
    );
    entities.push(body.into());
    let mut roles = BTreeSet::new();
    for &e in &entities {
        let origins = p.origins(e);
        assert_eq!(origins.len(), 1, "{e}: {origins:?}\n{p}");
        let (relation, origin) = origins[0];
        assert_eq!(relation, Relation::Generated, "{e}");
        assert!(
            matches!(origin, arris_topo::Origin::Role(_)),
            "{e}: {origin}"
        );
        assert!(roles.insert(origin), "{origin} names two entities");
        assert!(p.modified_from(origin).is_empty());
        assert!(!p.is_deleted(e));
    }
    assert_eq!(p.deleted().count(), 0);
    assert_eq!(p.outputs().len(), entities.len(), "{p}");
    assert_eq!(p.origins_recorded().count(), entities.len());
}

#[test]
fn the_box_is_clean_at_full_and_every_entity_has_a_role() {
    let mut m = Model::default();
    let (body, p) = the_box(&mut m);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}\n{}", dump_text(&m, body).unwrap());
    assert!(report.unchecked().is_empty());
    assert_eq!(report.euler().unwrap().to_string(), "8/12/6/6/1 g0 = 0");
    assert_roles(&m, body, &p);
    // The roles say what each entity is: the top face's plane has Z up.
    let top = p.generated_from(Role::Box(BoxPart::Face(Coord::Z, Side::Max)))[0];
    let top_face = m.face(top.id.try_into_face().unwrap()).unwrap();
    let plane = m.surface(top_face.surface()).unwrap();
    assert!((plane.frame().unwrap().z().z - 1.0).abs() < 1e-15);
    let corner = p.generated_from(Role::Box(BoxPart::Vertex([
        Side::Max,
        Side::Max,
        Side::Max,
    ])))[0];
    let point = m
        .vertex(corner.id.try_into_vertex().unwrap())
        .unwrap()
        .point();
    assert_eq!(point, Point3::new(40.0, 30.0, 10.0));
    let along_z = p.generated_from(Role::Box(BoxPart::Edge {
        along: Coord::Z,
        sides: [Side::Min, Side::Max],
    }))[0];
    let edge = m.edge(along_z.id.try_into_edge().unwrap()).unwrap();
    let (a, b) = (
        m.vertex(edge.start()).unwrap().point(),
        m.vertex(edge.end()).unwrap().point(),
    );
    assert_eq!((a.x, a.y, b.x, b.y), (0.0, 30.0, 0.0, 30.0));
    assert_ne!(a.z, b.z);
}

#[test]
fn the_cylinder_is_clean_at_full_and_every_entity_has_a_role() {
    let mut m = Model::default();
    let (body, p) = the_cylinder(&mut m);
    let report = check(&m, body, Level::Full);
    assert!(report.is_ok(), "{report}\n{}", dump_text(&m, body).unwrap());
    assert!(report.unchecked().is_empty());
    assert_eq!(report.euler().unwrap().to_string(), "2/3/3/3/1 g0 = 0");
    assert_roles(&m, body, &p);
    let seam = p.generated_from(Role::Cylinder(CylinderPart::Seam))[0];
    let uses = m.edge_uses(seam.id.try_into_edge().unwrap()).unwrap();
    assert_eq!(uses.len(), 2);
    assert_eq!(uses[0].face, uses[1].face, "both uses in the wall");
    let wall = p.generated_from(Role::Cylinder(CylinderPart::Wall))[0];
    assert_eq!(uses[0].face, wall.id.try_into_face().unwrap());
    // The same topology and orientations as the raw-built sample.
    let text = dump_text(&m, body).unwrap();
    let mut s = Model::default();
    let sample_body = sample::cylinder(&mut s, 4.0, 12.0).unwrap();
    let sample_text = dump_text(&s, sample_body).unwrap();
    let lines = |t: &str| -> Vec<String> {
        t.lines()
            .filter(|l| l.contains("coedge"))
            .map(|l| l.split_whitespace().nth(1).unwrap().to_string())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect()
    };
    assert_eq!(lines(&text), lines(&sample_text));
}

#[test]
fn two_runs_and_two_models_dump_identically() {
    let build = |m: &mut Model| {
        let (b, pb) = the_box(m);
        let (c, pc) = the_cylinder(m);
        (b, pb, c, pc)
    };
    let mut m = Model::default();
    let (b1, pb1, c1, pc1) = build(&mut m);
    let mut n = Model::default();
    let (b2, pb2, c2, pc2) = build(&mut n);
    assert_eq!(dump_text(&m, b1).unwrap(), dump_text(&n, b2).unwrap());
    assert_eq!(dump_text(&m, c1).unwrap(), dump_text(&n, c2).unwrap());
    assert_eq!((pb1, pc1), (pb2, pc2), "the same relations on every run");
    assert_eq!((b1, c1), (b2, c2), "the same ids");
    // And again in the same model: new ids, the same shape.
    let (b3, _) = the_box(&mut m);
    assert_ne!(b1, b3);
    let strip = |t: String| -> String {
        t.lines()
            .filter(|l| !l.starts_with("body"))
            .map(|l| {
                l.chars()
                    .filter(|c| !c.is_ascii_digit())
                    .collect::<String>()
            })
            .collect::<Vec<_>>()
            .join("\n")
    };
    assert_eq!(
        strip(dump_text(&m, b1).unwrap()),
        strip(dump_text(&m, b3).unwrap())
    );
}

#[test]
fn bad_parameters_are_degenerate_with_a_reason_and_the_model_untouched() {
    let mut m = Model::default();
    let probe = SurfaceId::new(0, 0);
    let degenerate = |r: Result<(Body, Provenance), OpError>| match r {
        Err(OpError::Degenerate { entities, reason }) => {
            assert!(entities.is_empty());
            reason
        }
        other => panic!("expected Degenerate, got {other:?}"),
    };
    assert_eq!(
        degenerate(primitive_box(
            &mut m,
            Point3::origin(),
            Point3::new(40.0, 0.0, 10.0)
        )),
        Reason::NotPositive {
            what: "y extent",
            value: 0.0
        }
    );
    assert_eq!(
        degenerate(primitive_box(
            &mut m,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 1.0)
        )),
        Reason::NotPositive {
            what: "x extent",
            value: -1.0
        }
    );
    assert_eq!(
        degenerate(primitive_box(
            &mut m,
            Point3::new(f64::NAN, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0)
        )),
        Reason::NonFinite { what: "min" }
    );
    assert_eq!(
        degenerate(primitive_box(
            &mut m,
            Point3::origin(),
            Point3::new(1.0, f64::INFINITY, 1.0)
        )),
        Reason::NonFinite { what: "max" }
    );
    let z = Axis::z_at(Point3::origin());
    assert_eq!(
        degenerate(primitive_cylinder(&mut m, z, 0.0, 12.0)),
        Reason::NotPositive {
            what: "radius",
            value: 0.0
        }
    );
    assert_eq!(
        degenerate(primitive_cylinder(&mut m, z, 4.0, -12.0)),
        Reason::NotPositive {
            what: "height",
            value: -12.0
        }
    );
    assert_eq!(
        degenerate(primitive_cylinder(&mut m, z, f64::NAN, 12.0)),
        Reason::NonFinite { what: "radius" }
    );
    let bad = Axis {
        origin: Point3::new(f64::INFINITY, 0.0, 0.0),
        ..z
    };
    assert_eq!(
        degenerate(primitive_cylinder(&mut m, bad, 4.0, 12.0)),
        Reason::NonFinite {
            what: "axis origin"
        }
    );
    assert!(m.surface(probe).is_err(), "nothing was appended");
    assert_eq!(
        the_box(&mut m).0.id,
        arris_topo::BodyId::new(0, 0),
        "not even an id was consumed"
    );
    let text = OpError::Degenerate {
        entities: Vec::new(),
        reason: Reason::NotPositive {
            what: "radius",
            value: 0.0,
        },
    }
    .to_string();
    assert_eq!(text, "degenerate result: radius must be positive, not 0");
}

#[test]
fn an_axis_of_any_length_is_normalised_and_a_tilted_one_seams_by_the_gp_ax3_rule() {
    let a = Axis::new(Point3::new(1.0, 2.0, 3.0), Vec3::new(0.0, 0.0, 7.5)).unwrap();
    assert_eq!(a, Axis::z_at(Point3::new(1.0, 2.0, 3.0)));
    let mut m = Model::default();
    let (c1, _) = primitive_cylinder(&mut m, a, 4.0, 12.0).unwrap();
    let mut n = Model::default();
    let (c2, _) =
        primitive_cylinder(&mut n, Axis::z_at(Point3::new(1.0, 2.0, 3.0)), 4.0, 12.0).unwrap();
    assert_eq!(dump_text(&m, c1).unwrap(), dump_text(&n, c2).unwrap());
    assert!(Axis::new(Point3::origin(), Vec3::zeros()).is_err());
    assert!(Axis::new(Point3::origin(), Vec3::new(f64::NAN, 0.0, 0.0)).is_err());
    // An axis along +x seams where gp_Ax3(P, N) puts X: at +z.
    let mut m = Model::default();
    let (c, p) = primitive_cylinder(
        &mut m,
        Axis::new(Point3::origin(), Vec3::x()).unwrap(),
        2.0,
        5.0,
    )
    .unwrap();
    let report = check(&m, c, Level::Full);
    assert!(report.is_ok(), "{report}");
    let v0 = p.generated_from(Role::Cylinder(CylinderPart::BottomVertex))[0];
    let point = m.vertex(v0.id.try_into_vertex().unwrap()).unwrap().point();
    assert_eq!(point, Point3::new(0.0, 0.0, 2.0));
}

#[test]
fn both_primitives_match_their_fixtures_through_the_oracle() {
    let mut m = Model::default();
    let (b, _) = the_box(&mut m);
    let (c, _) = the_cylinder(&mut m);
    oracle::compare(
        "primitive/box",
        &step::write(&m, &[b]).unwrap(),
        None,
        "primitive-box",
    )
    .unwrap();
    oracle::compare(
        "primitive/cylinder",
        &step::write(&m, &[c]).unwrap(),
        None,
        "primitive-cylinder",
    )
    .unwrap();
}

/// A helper the tests read handles with.
trait TryIntoId {
    fn try_into_face(self) -> Option<arris_topo::FaceId>;
    fn try_into_edge(self) -> Option<arris_topo::EdgeId>;
    fn try_into_vertex(self) -> Option<arris_topo::VertexId>;
}

impl TryIntoId for arris_topo::EntityId {
    fn try_into_face(self) -> Option<arris_topo::FaceId> {
        match self {
            Self::Face(f) => Some(f),
            _ => None,
        }
    }
    fn try_into_edge(self) -> Option<arris_topo::EdgeId> {
        match self {
            Self::Edge(e) => Some(e),
            _ => None,
        }
    }
    fn try_into_vertex(self) -> Option<arris_topo::VertexId> {
        match self {
            Self::Vertex(v) => Some(v),
            _ => None,
        }
    }
}
