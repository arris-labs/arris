//! The `distance` form of the `query` fixture kind (ADR-0054): two
//! operands, held to `BRepExtrema_DistShapeShape`.
//!
//! The runner compares the distance within the fixture's `probe`, `contained`
//! exactly, and the feet to *any* oracle solution (it lists the ones it
//! found, and a family of equal ones only by its sampled ends). A foot's
//! support kind is compared exactly and a face by its centroid, never by id,
//! except on a `point` operand, which Open CASCADE sees as a vertex. `unique`
//! must equal "the oracle lists one solution" unless `analytic.query_differs`
//! says why.

use std::collections::BTreeMap;

use arris_math::{Control, Point3, Vec3};
use arris_ops::query::{Foot, Operand, distance};
use arris_topo::{Body, EntityId, Orientation, Shape};

use super::{Answer, DistanceSpec, OperandSpec, Query, QueryError, QueryFixture, face_centroid};
use crate::corpus::Chain;
use crate::fixtures::Recipe;

/// What differs between Arris's answer and the oracle's, first difference.
fn mismatch(f: &QueryFixture, variant: &str, q: &Query, what: String) -> QueryError {
    QueryError::Mismatch {
        fixture: f.name.clone(),
        variant: variant.to_string(),
        label: q.label.clone(),
        what,
    }
}

/// The operand `spec` names, in `chain`'s model.
fn resolve(
    chain: &Chain,
    spec: &OperandSpec,
    params: &BTreeMap<String, f64>,
) -> Result<Operand, String> {
    let point = |p: &[crate::fixtures::Num; 3]| -> Result<Point3, String> {
        let n = |i: usize| p[i].eval(params).map_err(|e| e.to_string());
        Ok(Point3::new(n(0)?, n(1)?, n(2)?))
    };
    if let Some(p) = &spec.point {
        return Ok(Operand::Point(point(p)?));
    }
    let name = spec
        .step
        .as_ref()
        .ok_or("an operand needs a `point` or a `step`")?;
    let body: Body = chain
        .steps
        .get(name)
        .ok_or_else(|| format!("no step {name:?}"))?
        .body;
    let model = &chain.model;
    let picks = [
        spec.face.as_ref().map(|n| ("face", n)),
        spec.edge.as_ref().map(|n| ("edge", n)),
        spec.vertex.as_ref().map(|n| ("vertex", n)),
    ];
    let mut picks = picks.into_iter().flatten();
    let Some((kind, near)) = picks.next() else {
        return Ok(Operand::Shape(Shape::new(body.id, body.orientation)));
    };
    if picks.next().is_some() {
        return Err("an operand narrows to one of `face`, `edge` or `vertex`".into());
    }
    let near = point(&near.near)?;
    let closure = model.closure(body).map_err(|e| e.to_string())?;
    let shapes: Vec<Shape> = match kind {
        "face" => closure
            .faces
            .iter()
            .map(|&f| Shape::new(f, Orientation::Forward))
            .collect(),
        "edge" => closure
            .edges
            .iter()
            .map(|&e| Shape::new(e, Orientation::Forward))
            .collect(),
        _ => closure
            .vertices
            .iter()
            .map(|&v| Shape::new(v, Orientation::Forward))
            .collect(),
    };
    let mut best: Option<(f64, Shape)> = None;
    for shape in shapes {
        let Ok(d) = distance(
            model,
            Operand::Point(near),
            Operand::Shape(shape),
            &Control::NONE,
        ) else {
            continue;
        };
        if best.is_none_or(|(least, _)| d.distance < least) {
            best = Some((d.distance, shape));
        }
    }
    best.map(|(_, s)| Operand::Shape(s))
        .ok_or_else(|| format!("the body of {name:?} has no {kind}"))
}

fn support(foot: &Foot) -> &'static str {
    match foot.shape.map(|s| s.id) {
        Some(EntityId::Vertex(_)) => "vertex",
        Some(EntityId::Edge(_)) => "edge",
        Some(EntityId::Face(_)) => "face",
        Some(EntityId::Shell(_) | EntityId::Body(_)) | None => "none",
    }
}

/// Runs one distance query of `fixture` under `variant` against `answer`.
pub(super) fn run(
    fixture: &QueryFixture,
    variant: &str,
    chain: &Chain,
    query: &Query,
    spec: &DistanceSpec,
    answer: &Answer,
) -> Result<(), QueryError> {
    let wrong = |what: String| mismatch(fixture, variant, query, what);
    let Some(oracle) = &answer.distance else {
        return Err(wrong("the oracle has no distance".into()));
    };
    let expression = |source| QueryError::Expression {
        fixture: fixture.name.clone(),
        label: query.label.clone(),
        source,
    };
    let operand = |s: &OperandSpec| resolve(chain, s, &chain.params).map_err(wrong);
    let (a, b) = (operand(&spec.a)?, operand(&spec.b)?);
    let ours =
        distance(&chain.model, a, b, &Control::NONE).map_err(|source| QueryError::Refused {
            fixture: fixture.name.clone(),
            variant: variant.to_string(),
            label: query.label.clone(),
            source: Box::new(source),
        })?;
    let tolerances = &fixture.recipe.tolerances;
    let probe = tolerances.probe;
    if let Some(expect) = &query.expect_distance {
        let expect = expect.eval(&chain.params).map_err(expression)?;
        if (ours.distance - expect).abs() > probe {
            return Err(wrong(format!(
                "distance {} but the recipe expects {expect}",
                ours.distance
            )));
        }
    }
    if (ours.distance - oracle.distance).abs() > probe {
        return Err(wrong(format!(
            "distance {} but the oracle's is {}",
            ours.distance, oracle.distance
        )));
    }
    if ours.contained != oracle.contained {
        return Err(wrong(format!(
            "contained {} but the oracle's is {}",
            ours.contained, oracle.contained
        )));
    }
    if ours.contained {
        return Ok(());
    }
    let differs = fixture.recipe.analytic.query_differs.is_some();
    if !differs && ours.unique != (oracle.solutions.len() == 1) {
        return Err(wrong(format!(
            "unique {} but the oracle lists {} solution(s)",
            ours.unique,
            oracle.solutions.len()
        )));
    }
    let at = |p: Point3, q: &[f64; 3]| (p.coords - Vec3::from(*q)).norm() <= probe;
    let Some(solution) = oracle
        .solutions
        .iter()
        .find(|s| at(ours.a.point, &s.a) && at(ours.b.point, &s.b))
    else {
        if differs {
            return Ok(());
        }
        return Err(wrong(format!(
            "feet {:?} and {:?} match none of the oracle's {:?}",
            ours.a.point.coords.as_slice(),
            ours.b.point.coords.as_slice(),
            oracle
                .solutions
                .iter()
                .map(|s| (s.a, s.b))
                .collect::<Vec<_>>()
        )));
    };
    for (side, foot, spec, kind, centroid) in [
        (
            "first",
            &ours.a,
            &spec.a,
            &solution.a_support,
            &solution.a_face_centroid,
        ),
        (
            "second",
            &ours.b,
            &spec.b,
            &solution.b_support,
            &solution.b_face_centroid,
        ),
    ] {
        // Open CASCADE sees a bare point as a vertex of its own.
        if spec.point.is_some() {
            continue;
        }
        if support(foot) != kind {
            return Err(wrong(format!(
                "the {side} foot lies on a {} but the oracle's lies on a {kind}",
                support(foot)
            )));
        }
        if let (
            Some(Shape {
                id: EntityId::Face(face),
                ..
            }),
            Some(want),
        ) = (foot.shape, centroid)
        {
            let got = face_centroid(&chain.model, face)
                .map_err(|e| wrong(format!("the centroid of {face}: {e}")))?;
            if (got - Vec3::from(*want)).norm() > tolerances.centroid_abs {
                return Err(wrong(format!(
                    "the {side} foot is on the face centred {:?} but the oracle's is centred {want:?}",
                    got.as_slice()
                )));
            }
        }
    }
    Ok(())
}

/// The lint of one distance query's operands: each evaluates and names
/// a step of the recipe.
pub(super) fn lint_spec(
    r: &Recipe,
    params: &BTreeMap<String, f64>,
    spec: &DistanceSpec,
) -> Vec<String> {
    let mut out = Vec::new();
    for (side, operand) in [("a", &spec.a), ("b", &spec.b)] {
        let narrowed = [&operand.face, &operand.edge, &operand.vertex]
            .iter()
            .filter(|n| n.is_some())
            .count();
        match (&operand.point, &operand.step) {
            (Some(_), Some(_)) => out.push(format!("operand {side}: a `point` and a `step`")),
            (None, None) => out.push(format!("operand {side}: needs a `point` or a `step`")),
            (Some(p), None) => {
                if narrowed > 0 {
                    out.push(format!("operand {side}: a point cannot be narrowed"));
                }
                for n in p {
                    if let Err(e) = n.eval(params) {
                        out.push(format!("operand {side}: {e}"));
                    }
                }
            }
            (None, Some(step)) => {
                if !r.steps.iter().any(|s| s.name() == step) {
                    out.push(format!("operand {side}: {step:?} is not a step"));
                }
                if narrowed > 1 {
                    out.push(format!(
                        "operand {side}: narrow to one of `face`, `edge` or `vertex`"
                    ));
                }
                for n in [&operand.face, &operand.edge, &operand.vertex]
                    .into_iter()
                    .flatten()
                {
                    for c in &n.near {
                        if let Err(e) = c.eval(params) {
                            out.push(format!("operand {side}: {e}"));
                        }
                    }
                }
            }
        }
    }
    out
}
