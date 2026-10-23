//! Helpers copied across test files, once (dev-only: not part of a
//! consumer's dependency graph, and not built for wasm32 since it takes
//! `proptest`'s `TestCaseError`): a relative-tolerance and a
//! period-aware numeric comparison, the bound a mass property holds to
//! along a fitted pcurve, the provenance accounting a
//! generated body's record owes (`docs/DATA-MODEL.md` §Provenance), and
//! central differences against a curve's or surface's own analytic
//! derivatives.

use std::collections::BTreeSet;

use arris_geom::{CurveEval, SurfaceEval};
use arris_math::{Point3, Precision, Tolerance, Vec3};
use arris_ops::measure::MassProperties;
use arris_topo::provenance::{Origin, Provenance, Relation, Role, SweepPart};
use arris_topo::{Body, EntityId, Model, Orientation, Shape};
use proptest::test_runner::TestCaseError;

/// The relative tolerance [`close`] and [`close_param`] hold to.
pub const REL: f64 = 1e-9;

/// The bound a closed form holds to against evaluation on the property
/// tests' default scale: `1e-12 · DEFAULT_SCALE`.
pub const EXACT: f64 = 1e-12 * crate::prop::DEFAULT_SCALE;

/// The tolerance of the default precision, as most tests state it.
pub fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// A cube of edge `side` with a corner at the origin, from the unmetered
/// box primitive.
pub fn cube(m: &mut Model, side: f64) -> Body {
    crate::unmetered::primitive_box(m, Point3::origin(), Point3::new(side, side, side))
        .unwrap()
        .0
}

/// The sample cylinder tests build on: radius `4.0`, height `12.0`.
pub fn cylinder(m: &mut Model) -> Body {
    crate::sample::cylinder(m, 4.0, 12.0).unwrap()
}

/// The step [`d1_stencil`] and [`d2_stencil`] sample at.
pub const FD_STEP: f64 = 1e-4;

/// A `proptest` failure carrying `what`'s message: `TestCaseError::fail`
/// over anything `Display`, so a query's own error converts with `?` by
/// way of `.map_err(fail)`.
pub fn fail(what: impl core::fmt::Display) -> TestCaseError {
    TestCaseError::fail(what.to_string())
}

/// `|a − b| ≤ rel · max(|a|, |b|, floor)`: the relative comparison every
/// closed form is stated with, with a floor so a quantity that is
/// exactly zero is not compared relatively.
pub fn close_to(a: f64, b: f64, floor: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * a.abs().max(b.abs()).max(floor)
}

/// The relative bound a mass property holds to when a face's boundary
/// runs along a fitted pcurve: [`REL`] plus what the model's own
/// tolerance permits over the body's size. A pcurve is fitted to within
/// its edge's tolerance, so a face's (u, v) region is bounded to within
/// `tol` in 3D and every mass property — each an integral over that
/// boundary — carries a relative error of order `tol` over a length of
/// the body, taken as `√A`. `REL` alone is a literal, and the kernel's
/// rule is that the tolerance is the model's. The two cases it was first
/// measured on, a union against the cut it is restored from 38 away
/// from the origin and a fillet miter on a box 60 away, were mostly the
/// gaps between fits integrated about the origin, which
/// `mass_properties` no longer does (ADR-0024); a
/// section on a cone, a sphere or a torus fitted twice still needs the
/// bound.
pub fn fitted_rel(m: &Model, p: &MassProperties) -> f64 {
    REL + m.precision().default_tolerance / p.area.sqrt()
}

/// [`close_to`] at [`REL`] and a floor of `1.0`.
pub fn close(a: f64, b: f64) -> bool {
    close_to(a, b, 1.0, REL)
}

/// Equal modulo `period`, when there is one; [`close`] otherwise.
pub fn close_param(a: f64, b: f64, period: Option<f64>) -> bool {
    match period {
        Some(p) => {
            let d = (a - b).rem_euclid(p);
            d.min(p - d) <= REL * a.abs().max(b.abs()).max(1.0)
        }
        None => close(a, b),
    }
}

/// The vertices, edges, faces, shells and the body itself, each once as a
/// `Forward` [`Shape`].
pub fn entities_of(m: &Model, body: Body) -> BTreeSet<Shape> {
    let c = m.closure(body).unwrap();
    let mut set: BTreeSet<Shape> = BTreeSet::new();
    set.extend(
        c.vertices
            .iter()
            .map(|&v| Shape::new(v, Orientation::Forward)),
    );
    set.extend(c.edges.iter().map(|&e| Shape::new(e, Orientation::Forward)));
    set.extend(c.faces.iter().map(|&f| Shape::new(f, Orientation::Forward)));
    set.extend(
        c.shells
            .iter()
            .map(|&s| Shape::new(s, Orientation::Forward)),
    );
    set.insert(Shape::from(body));
    set
}

/// Every entity of the body generated from exactly one role `role_part`
/// recognises as a [`SweepPart`], every part naming one entity but a
/// degenerate `Rise` (a pinch closes two faces on the same axis edge),
/// nothing modified and nothing deleted: the set of parts recorded, for
/// an `extrude` or a `revolve`'s own property test to hold against the
/// parts it expects from the sketch alone.
pub fn recorded_parts(
    m: &Model,
    body: Body,
    p: &Provenance,
    role_part: impl Fn(Role) -> Option<SweepPart>,
) -> Result<BTreeSet<SweepPart>, TestCaseError> {
    let entities = entities_of(m, body);
    let mut parts = BTreeSet::new();
    for &e in &entities {
        let origins = p.origins(e);
        if origins.len() != 1 {
            return Err(fail(format!("{e}: {origins:?}\n{p}")));
        }
        let (relation, origin) = origins[0];
        if relation != Relation::Generated {
            return Err(fail(format!(
                "{e}: {relation} from {origin}, not generated"
            )));
        }
        let Origin::Role(role) = origin else {
            return Err(fail(format!("{e}: generated from {origin}, not a role")));
        };
        let Some(part) = role_part(role) else {
            return Err(fail(format!(
                "{e}: generated from {origin}, not a sweep part"
            )));
        };
        let degenerate =
            matches!(e.id, EntityId::Edge(id) if m.edge(id).is_ok_and(|x| x.is_degenerate()));
        if !(parts.insert(part) || (degenerate && matches!(part, SweepPart::Rise { .. }))) {
            return Err(fail(format!("{part:?} names two entities")));
        }
        if !p.modified_from(origin).is_empty() {
            return Err(fail(format!("{origin} has a modified output too")));
        }
        if p.is_deleted(e) {
            return Err(fail(format!("{e} is deleted")));
        }
    }
    if p.deleted().count() != 0 {
        return Err(fail(format!("{p} deletes something")));
    }
    if p.outputs().len() != entities.len() {
        return Err(fail(format!(
            "{p} outputs {} but there are {} entities",
            p.outputs().len(),
            entities.len()
        )));
    }
    Ok(parts)
}

/// Fourth-order first derivative of `f` at `0` from four samples at
/// [`FD_STEP`].
pub fn d1_stencil(f: impl Fn(f64) -> Vec3) -> Vec3 {
    let h = FD_STEP;
    (-f(2.0 * h) + 8.0 * f(h) - 8.0 * f(-h) + f(-2.0 * h)) / (12.0 * h)
}

/// Fourth-order second derivative of `f` at `0` from five samples at
/// [`FD_STEP`].
pub fn d2_stencil(f: impl Fn(f64) -> Vec3) -> Vec3 {
    let h = FD_STEP;
    (-f(2.0 * h) + 16.0 * f(h) - 30.0 * f(0.0) + 16.0 * f(-h) - f(-2.0 * h)) / (12.0 * h * h)
}

/// [`CurveEval`] by central differences of `point` alone, against
/// [`Curve::eval`](arris_geom::Curve::eval)'s or
/// [`NurbsCurve::eval`](arris_geom::NurbsCurve::eval)'s own derivatives.
pub fn central_differences_curve(point: impl Fn(f64) -> Point3, t: f64) -> CurveEval {
    let p = |dt: f64| point(t + dt).coords;
    CurveEval {
        point: point(t),
        d1: d1_stencil(p),
        d2: d2_stencil(p),
    }
}

/// [`SurfaceEval`] by central differences of `point` alone, against
/// [`Surface::eval`](arris_geom::Surface::eval)'s or
/// [`NurbsSurface::eval`](arris_geom::NurbsSurface::eval)'s own
/// derivatives.
pub fn central_differences_surface(
    point: impl Fn(f64, f64) -> Point3,
    u: f64,
    v: f64,
) -> SurfaceEval {
    let p = |du: f64, dv: f64| point(u + du, v + dv).coords;
    SurfaceEval {
        point: point(u, v),
        du: d1_stencil(|h| p(h, 0.0)),
        dv: d1_stencil(|h| p(0.0, h)),
        duu: d2_stencil(|h| p(h, 0.0)),
        dvv: d2_stencil(|h| p(0.0, h)),
        duv: d1_stencil(|hu| d1_stencil(|hv| p(hu, hv))),
    }
}
