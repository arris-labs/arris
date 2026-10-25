//! The operations on edges and faces a recipe picks by point: fillet and
//! chamfer, offset, shell.

use super::*;

pub(super) fn blend(
    cx: &mut Cx<'_, '_>,
    of: &str,
    edges: &[[Num; 3]],
    size: &Num,
) -> Result<Option<Made>, CorpusError> {
    let of_body = cx.reference(of)?;
    let probe = cx.fixture.recipe.tolerances.probe;
    let mut selected = Vec::with_capacity(edges.len());
    for p in edges {
        let point = cx.point(p)?;
        let edge = edge_at(cx.m, of_body, point, probe).map_err(|what| CorpusError::EdgePoint {
            fixture: cx.fixture.name.clone(),
            step: cx.step.name().to_string(),
            point: [point.x, point.y, point.z],
            what,
        })?;
        selected.push(edge);
    }
    let size = cx.number(size)?;
    let blended = if matches!(cx.step, Step::Chamfer { .. }) {
        arris_ops::chamfer(cx.m, of_body, &selected, size, cx.control)
    } else {
        arris_ops::fillet(cx.m, of_body, &selected, size, cx.control)
    };
    solid(blended.map_err(|e| cx.op(e))?, vec![of_body])
}

/// The faces of `of` a recipe names by the points on them.
fn faces_at(cx: &Cx<'_, '_>, of_body: Body, points: &[[Num; 3]]) -> Result<Vec<Face>, CorpusError> {
    let mut selected = Vec::with_capacity(points.len());
    for p in points {
        let point = cx.point(p)?;
        let face = face_at(cx.m, of_body, point).map_err(|what| CorpusError::FacePoint {
            fixture: cx.fixture.name.clone(),
            step: cx.step.name().to_string(),
            point: [point.x, point.y, point.z],
            what,
        })?;
        selected.push(face);
    }
    Ok(selected)
}

pub(super) fn offset(
    cx: &mut Cx<'_, '_>,
    of: &str,
    faces: &[[Num; 3]],
    distance: &Num,
) -> Result<Option<Made>, CorpusError> {
    let of_body = cx.reference(of)?;
    let selected = faces_at(cx, of_body, faces)?;
    let distance = cx.number(distance)?;
    solid(
        arris_ops::offset_faces(cx.m, of_body, &selected, distance, cx.control)
            .map_err(|e| cx.op(e))?,
        vec![of_body],
    )
}

pub(super) fn shell(
    cx: &mut Cx<'_, '_>,
    of: &str,
    openings: &[[Num; 3]],
    thickness: &Num,
    side: ShellSide,
) -> Result<Option<Made>, CorpusError> {
    let of_body = cx.reference(of)?;
    let selected = faces_at(cx, of_body, openings)?;
    let thickness = cx.number(thickness)?;
    let side = match side {
        ShellSide::Inward => arris_ops::ShellSide::Inward,
        ShellSide::Outward => arris_ops::ShellSide::Outward,
    };
    solid(
        arris_ops::shell(cx.m, of_body, &selected, thickness, side, cx.control)
            .map_err(|e| cx.op(e))?,
        vec![of_body],
    )
}

/// The edge of `body` a recipe names by `point`: the one
/// `classify_point` answers `On(Edge)` for, when no second edge of the
/// body passes within `probe` of the point — the rule the oracle's
/// nearest-edge search keeps too (`tests/fixtures/README.md`). Errors:
/// what the point names instead.
/// The face of `body` that `point` classifies `On`: a point inside or
/// outside the body, or on an edge or a vertex, names no one face.
pub(super) fn face_at(m: &Model, body: Body, point: Point3) -> Result<Face, String> {
    match classify_point(m, body, point) {
        Ok(Classification::On(Shape {
            id: EntityId::Face(id),
            ..
        })) => Ok(Face::forward(id)),
        Ok(Classification::On(shape)) => Err(format!("is on {shape}, not inside a face")),
        Ok(Classification::Inside) => Err("is inside the body, on no face".into()),
        Ok(Classification::Outside) => Err("is outside the body, on no face".into()),
        Err(e) => Err(format!("could not be classified: {e}")),
    }
}

pub(super) fn edge_at(m: &Model, body: Body, point: Point3, probe: f64) -> Result<Edge, String> {
    let on = match classify_point(m, body, point) {
        Ok(Classification::On(shape)) => shape,
        Ok(Classification::Inside) => return Err("is inside the body, on no edge".into()),
        Ok(Classification::Outside) => return Err("is outside the body, on no edge".into()),
        Err(e) => return Err(format!("could not be classified: {e}")),
    };
    let EntityId::Edge(id) = on.id else {
        return Err(format!("is on {on}, not on an edge"));
    };
    let mut near: Vec<EntityId> = Vec::new();
    for edge in m.edges(body).map_err(|e| e.to_string())? {
        let entity = m.edge(edge.id).map_err(|e| e.to_string())?;
        let Some((curve, range)) = entity.curve() else {
            continue;
        };
        let curve = m.curve(curve).map_err(|e| e.to_string())?;
        let Ok(projection) = curve.project(point) else {
            continue;
        };
        let t = match curve.period() {
            Some(p) => {
                let k = ((range.lo() - projection.t) / p).ceil();
                projection.t + k * p
            }
            None => projection.t,
        };
        if projection.distance <= probe && range.lo() - probe <= t && t <= range.hi() + probe {
            near.push(EntityId::Edge(edge.id));
        }
    }
    if near.len() != 1 {
        return Err(format!(
            "is within the probe tolerance of {} edges ({})",
            near.len(),
            near.iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    Ok(Edge::forward(id))
}
