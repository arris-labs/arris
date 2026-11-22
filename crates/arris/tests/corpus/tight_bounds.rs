//! Extents of the corpus results, checked against the tolerance envelope and mesh.

use arris::check::domain::FaceDomain;
use arris::math::{Aabb, Control};
use arris::mesh::{MeshRequest, tessellate_with};
use arris::ops::query::tight_bounds;
use arris::topo::{Body, Model};
use arris_debug::{corpus, fixtures};

#[test]
fn tight_bounds_boolean_corpus() {
    check_area("boolean");
}

#[test]
fn tight_bounds_blend_corpus() {
    check_area("blend");
}

#[test]
fn tight_bounds_sweep_corpus() {
    check_area("sweep");
}

#[test]
fn tight_bounds_loft_corpus() {
    check_area("loft");
}

#[test]
fn tight_bounds_closed_loft_across_the_knot_seams() {
    let dir = fixtures::corpus_root().join("loft/loft-closed-torus-like");
    let chain = corpus::chain(&dir, "default").unwrap();
    let body = chain.result().unwrap();

    // Clipping a rounding-width patch across the seam used to leave
    // millions of open patches. A generous work budget keeps that case
    // reproducible without relying on elapsed time or memory use.
    let got = tight_bounds(&chain.model, body.id.into(), &Control::budget(4_096))
        .unwrap()
        .unwrap();
    // The loft interpolates radially oriented circles. In s = 2v - 1
    // its x- and y-coordinate multipliers are -1 + (14s² - 8s⁴)/3
    // and 8(s³ - s)/3. The largest section radius about z is 9.5.
    let y = 9.5 * 16.0 / (9.0 * 3.0_f64.sqrt());
    let want_min = [-9.5, -y, -1.5];
    let want_max = [9.5 * 25.0 / 24.0, y, 1.5];
    let eps = chain.model.precision().default_tolerance;
    for k in 0..3 {
        assert!((got.min[k] - want_min[k]).abs() <= eps, "{got:?}");
        assert!((got.max[k] - want_max[k]).abs() <= eps, "{got:?}");
    }
}

#[test]
fn tight_bounds_ruled_thread_chamfer() {
    let dir = fixtures::corpus_root().join("sweep/sweep-thread-chamfer");
    let chain = corpus::chain(&dir, "default").unwrap();
    let body = chain.result().unwrap();

    // The chamfer wall has an open degree-one direction. Around each turn
    // the ruling slopes can change sign, but the trims enclose them all.
    assert!(
        tight_bounds(&chain.model, body.id.into(), &Control::budget(16_384))
            .unwrap()
            .is_some()
    );
}

fn check_area(area: &str) {
    let root = fixtures::corpus_root().join(area);
    let mut failures = Vec::new();
    let mut checked = 0;
    for dir in fixtures::corpus()
        .into_iter()
        .filter(|dir| dir.starts_with(&root))
    {
        let fixture = fixtures::load(&dir).unwrap();
        for variant in fixture.recipe.variant_names() {
            // The individual corpus tests assert the typed refusal for cases
            // whose recipe produces no result.
            if fixture.recipe.analytic.expect_error.is_some()
                || fixture.expected.results[&variant].degenerate
            {
                continue;
            }
            let chord = fixture.recipe.tolerances.mesh_chord;
            let result = (|| -> Result<(), Box<dyn std::error::Error>> {
                let chain = corpus::chain(&dir, &variant)?;
                let made = chain
                    .steps
                    .get(&chain.result)
                    .ok_or("missing result step")?;
                assert!(!made.outputs.is_empty(), "result step has no outputs");
                for &body in &made.outputs {
                    check_body(&chain.model, body, chord)?;
                    checked += 1;
                }
                Ok(())
            })();
            if let Err(error) = result {
                failures.push(format!("{} [{variant}]: {error}", fixture.name));
            }
        }
    }
    assert!(checked > 0, "{area}: no fixture results checked");
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

fn check_body(model: &Model, body: Body, chord: f64) -> Result<(), Box<dyn std::error::Error>> {
    let tight = tight_bounds(model, body.id.into(), &Control::NONE)?.ok_or("no tight box")?;
    let (outer, tolerance) = tolerance_envelope(model, body)?;
    if outer.union(tight) != outer {
        return Err(format!("{body:?}: tight {tight:?} exceeds bounds {outer:?}").into());
    }

    let mesh = tessellate_with(model, body, &MeshRequest::new(chord), &Control::NONE)?;
    let positions = mesh.positions();
    if !positions
        .iter()
        .flatten()
        .all(|coordinate| coordinate.is_finite())
    {
        return Err(format!("{body:?}: nonfinite mesh vertex").into());
    }
    // Checking the mesh box against the grown tight box checks every vertex.
    let sampled = Aabb::of_points(positions).ok_or("empty mesh")?;
    let grown = tight.inflated(chord + tolerance);
    if grown.union(sampled) != grown {
        return Err(
            format!("{body:?}: mesh box {sampled:?} exceeds grown tight box {grown:?}").into(),
        );
    }

    let limit = chord + model.precision().default_tolerance;
    for axis in 0..3 {
        let lower_gap = (tight.min[axis] - sampled.min[axis]).abs();
        let upper_gap = (tight.max[axis] - sampled.max[axis]).abs();
        if lower_gap.max(upper_gap) > limit {
            return Err(format!(
                "{body:?}: tight {tight:?}, mesh {sampled:?}, axis {axis} side gaps \
                 ({lower_gap}, {upper_gap}) exceed chord + eps ({limit})"
            )
            .into());
        }
    }
    Ok(())
}

/// The components of Python's `bounds(body)` and the largest tolerance they carry.
fn tolerance_envelope(
    model: &Model,
    body: Body,
) -> Result<(Aabb, f64), Box<dyn std::error::Error>> {
    let closure = model.closure(body)?;
    let mut boxes = Vec::new();
    let mut tolerance = model.precision().default_tolerance;
    for id in closure.vertices {
        let vertex = model.vertex(id)?;
        tolerance = tolerance.max(vertex.tolerance());
        boxes.push(Aabb::of_point(vertex.point()).inflated(vertex.tolerance()));
    }
    for id in closure.edges {
        let edge = model.edge(id)?;
        tolerance = tolerance.max(edge.tolerance());
        if let Some((curve, range)) = edge.curve()
            && let Some(b) = model.curve(curve)?.bounds(range)
        {
            boxes.push(b.inflated(edge.tolerance()));
        }
    }
    for id in closure.faces {
        let face = model.face(id)?;
        tolerance = tolerance.max(face.tolerance());
        let domain = FaceDomain::of(model, id, model.precision().parametric_tolerance)?;
        let b = domain.bounds().or_else(|| {
            let surface = domain.surface();
            surface
                .bounds(surface.domain())
                .map(|b| b.inflated(face.tolerance()))
        });
        boxes.push(b.ok_or("no finite face bounds")?);
    }
    let bounds = boxes
        .into_iter()
        .reduce(Aabb::union)
        .ok_or("no body bounds")?;
    Ok((bounds, tolerance))
}
