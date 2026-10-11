//! The `section` form of the `query` fixture kind (ADR-0056): a solid and
//! a plane, held to Open CASCADE's section.
//!
//! The runner builds the section on a clone of the fixture's model, holds
//! its sheet to the checker at `Full`, and compares: `None` against the
//! oracle's `none` exactly, then the face and loop counts exactly, the edge
//! count exactly unless `analytic.query_differs` says why not, the total
//! edge length and the area relative to `area_rel`, and the centroid within
//! `centroid_abs`.
//!
//! Under `analytic.query_differs` the oracle's sheet is not the kernel's —
//! Open CASCADE keeps a region of the plane flush with a face of the
//! solid, which is no section (ADR-0056 §1) — so the none-or-sheet verdict
//! is the recipe's `expect`, and a sheet is held to the recipe's
//! `expect_faces`, `expect_area` and `expect_centroid`, which the lint
//! requires, instead of the oracle's counts and measures.

use std::collections::BTreeMap;

use arris_check::{Level, check};
use arris_geom::integrate::gauss_legendre;
use arris_math::{Control, Frame, Point3, Vec3};
use arris_ops::section;
use arris_topo::{Body, Model};

use super::{Answer, PlaneSpec, Query, QueryError, QueryFixture, SectionSpec, VariantAnswers};
use crate::corpus::Chain;
use crate::fixtures::{Num, Recipe};

/// The values `expect` takes in a section query.
const EXPECTS: [&str; 2] = ["none", "sheet"];
/// Panels the quadrature of an edge's length is cut into: smooth curves
/// to rounding, a section's NURBS curve to its fit.
const LENGTH_PANELS: usize = 64;

/// The solid a section query names: a whole `{"step": …}`.
fn body_of(chain: &Chain, spec: &SectionSpec) -> Result<Body, String> {
    let o = &spec.body;
    if o.point.is_some() || o.face.is_some() || o.edge.is_some() || o.vertex.is_some() {
        return Err("a section body is a whole `step`".into());
    }
    let name = o.step.as_ref().ok_or("the body needs a `step`")?;
    chain
        .steps
        .get(name)
        .map(|s| s.body)
        .ok_or_else(|| format!("no step {name:?}"))
}

/// The plane as a frame under `params`.
fn frame_of(plane: &PlaneSpec, params: &BTreeMap<String, f64>) -> Result<Frame, String> {
    let vector = |v: &[Num; 3]| -> Result<Vec3, String> {
        let n = |i: usize| v[i].eval(params).map_err(|e| e.to_string());
        Ok(Vec3::new(n(0)?, n(1)?, n(2)?))
    };
    let origin = Point3::from(vector(&plane.origin)?);
    let z = vector(&plane.z)?;
    let frame = match &plane.x {
        Some(x) => Frame::new(origin, z, vector(x)?),
        None => Frame::from_z(origin, z),
    };
    frame.map_err(|e| format!("the plane is no frame: {e}"))
}

/// The length of the edge, by Gauss–Legendre over the curve's speed.
fn edge_length(m: &Model, edge: arris_topo::EdgeId) -> Result<f64, String> {
    let e = m.edge(edge).map_err(|e| e.to_string())?;
    let Some((curve, range)) = e.curve() else {
        return Ok(0.0);
    };
    let curve = m.curve(curve).map_err(|e| e.to_string())?;
    let table = gauss_legendre();
    let step = range.length() / LENGTH_PANELS as f64;
    let mut length = 0.0;
    for k in 0..LENGTH_PANELS {
        let (a, half) = (range.lo() + step * k as f64, step / 2.0);
        for &(x, w) in &table {
            length += w * half * curve.eval(a + half * (1.0 + x)).d1.norm();
        }
    }
    Ok(length)
}

/// Runs one section query of `fixture` under `variant` against `answer`.
pub(super) fn run(
    fixture: &QueryFixture,
    variant: &str,
    chain: &Chain,
    query: &Query,
    spec: &SectionSpec,
    answer: &Answer,
) -> Result<(), QueryError> {
    let wrong = |what: String| QueryError::Mismatch {
        fixture: fixture.name.clone(),
        variant: variant.to_string(),
        label: query.label.clone(),
        what,
    };
    let Some(oracle) = &answer.section else {
        return Err(wrong("the oracle has no section".into()));
    };
    let body = body_of(chain, spec).map_err(wrong)?;
    let frame = frame_of(&spec.plane, &chain.params).map_err(wrong)?;
    // The operation adds a body: it is made in a copy of the model.
    let mut m = chain.model.clone();
    let ours =
        section(&mut m, body, &frame, &Control::NONE).map_err(|source| QueryError::Refused {
            fixture: fixture.name.clone(),
            variant: variant.to_string(),
            label: query.label.clone(),
            source: Box::new(source),
        })?;
    let tolerances = &fixture.recipe.tolerances;
    let verdict = if ours.is_some() { "sheet" } else { "none" };
    if let Some(expect) = &query.expect
        && expect != verdict
    {
        return Err(wrong(format!("{verdict} but the recipe expects {expect}")));
    }
    let differs = fixture.recipe.analytic.query_differs.is_some();
    if differs && query.expect.is_none() {
        return Err(wrong("a query_differs fixture states `expect`".into()));
    }
    if !differs && (verdict == "none") != oracle.none {
        return Err(wrong(format!(
            "{verdict} but the oracle's section is {}",
            if oracle.none { "none" } else { "a sheet" }
        )));
    }
    let Some(ours) = ours else { return Ok(()) };
    let report = check(&m, ours.sheet, Level::Full);
    if !report.is_ok() {
        return Err(wrong(format!("the sheet fails the checker: {report}")));
    }
    let faces = m.faces(ours.sheet).map_err(|e| wrong(e.to_string()))?;
    let mut loops = 0;
    for f in &faces {
        loops += m
            .face(f.id)
            .map_err(|e| wrong(e.to_string()))?
            .loops()
            .len();
    }
    let closure = m.closure(ours.sheet).map_err(|e| wrong(e.to_string()))?;
    let mut edges = 0;
    let mut length = 0.0;
    for &e in &closure.edges {
        let l = edge_length(&m, e).map_err(wrong)?;
        if m.edge(e)
            .map_err(|e| wrong(e.to_string()))?
            .curve()
            .is_some()
        {
            edges += 1;
        }
        length += l;
    }
    if let Some(expect) = query.expect_faces.filter(|&n| n != faces.len()) {
        return Err(wrong(format!(
            "{} faces, the recipe expects {expect}",
            faces.len()
        )));
    }
    let relative = |name: &str, got: f64, want: f64, tolerance: f64| {
        let difference = (got - want).abs() / want.abs().max(got.abs()).max(f64::MIN_POSITIVE);
        if difference.is_nan() || difference > tolerance {
            Err(wrong(format!(
                "{name} {got} but {want} expected: {difference:e} relative, above {tolerance:e}"
            )))
        } else {
            Ok(())
        }
    };
    if let Some(expect) = &query.expect_area {
        let expect = expect
            .eval(&chain.params)
            .map_err(|source| QueryError::Expression {
                fixture: fixture.name.clone(),
                label: query.label.clone(),
                source,
            })?;
        relative("area", ours.area, expect, tolerances.area_rel)?;
    }
    for (name, got, want) in [
        ("faces", faces.len(), oracle.faces),
        ("loops", loops, oracle.loops),
    ] {
        if got != want {
            return Err(wrong(format!("{got} {name} but the oracle's are {want}")));
        }
    }
    if edges != oracle.edges {
        return Err(wrong(format!(
            "{edges} edges but the oracle's are {}",
            oracle.edges
        )));
    }
    relative("area", ours.area, oracle.area, tolerances.area_rel)?;
    // A split closed edge or a flush boundary changes the oracle's count
    // and the length with it: the length is held only where counts agree.
    if edges == oracle.edges {
        relative(
            "edge length",
            length,
            oracle.edge_length,
            tolerances.area_rel,
        )?;
    }
    let Some(at) = oracle.centroid else {
        return Err(wrong("the oracle has no centroid".into()));
    };
    if (ours.centroid.coords - Vec3::from(at)).norm() > tolerances.centroid_abs {
        return Err(wrong(format!(
            "centroid {:?} but the oracle's is {at:?}",
            ours.centroid.coords.as_slice()
        )));
    }
    Ok(())
}

/// The lint of one section query: its body a step, its plane evaluating
/// to a frame, its `expect*` known and agreeing with the oracle's answer.
pub(super) fn lint(
    r: &Recipe,
    params: &BTreeMap<String, f64>,
    q: &Query,
    spec: &SectionSpec,
    answers: &VariantAnswers,
) -> Vec<String> {
    let mut out = Vec::new();
    let o = &spec.body;
    match (
        &o.step,
        o.point.is_some() || o.face.is_some() || o.edge.is_some() || o.vertex.is_some(),
    ) {
        (Some(step), false) => {
            if !r.steps.iter().any(|s| s.name() == step) {
                out.push(format!("body: {step:?} is not a step"));
            }
        }
        _ => out.push("the body is a whole `step`".into()),
    }
    if let Err(e) = frame_of(&spec.plane, params) {
        out.push(e);
    }
    let Some(oracle) = answers
        .queries
        .iter()
        .find(|a| a.label == q.label)
        .and_then(|a| a.section.as_ref())
    else {
        out.push("the oracle has no section".into());
        return out;
    };
    if let Some(expect) = &q.expect {
        if !EXPECTS.contains(&expect.as_str()) {
            out.push(format!("expect {expect:?} is not none or sheet"));
        } else if r.analytic.query_differs.is_none() && (expect == "none") != oracle.none {
            out.push(format!(
                "expect {expect} but the oracle says {}",
                if oracle.none { "none" } else { "a sheet" }
            ));
        }
    }
    let held = r.analytic.query_differs.is_none();
    if let Some(expect) = q.expect_faces.filter(|&n| held && n != oracle.faces) {
        out.push(format!(
            "expect_faces {expect} but the oracle says {}",
            oracle.faces
        ));
    }
    if r.analytic.query_differs.is_some() {
        if q.expect.is_none() {
            out.push("a query_differs fixture states `expect`".into());
        }
        if q.expect.as_deref() == Some("sheet") && q.expect_area.is_none() {
            out.push("a query_differs sheet states `expect_area`".into());
        }
    }
    if let Some(at) = &q.expect_centroid {
        for n in at {
            if let Err(e) = n.eval(params) {
                out.push(e.to_string());
            }
        }
    }
    if let Some(expect) = &q.expect_area {
        match expect.eval(params) {
            Ok(e) if held && (e - oracle.area).abs() > 1e-9 * oracle.area.abs().max(e.abs()) => {
                out.push(format!(
                    "expect_area {e} but the oracle says {}",
                    oracle.area
                ));
            }
            Ok(_) => {}
            Err(e) => out.push(e.to_string()),
        }
    }
    out
}
