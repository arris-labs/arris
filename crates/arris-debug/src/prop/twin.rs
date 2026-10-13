//! The exact NURBS twin of a body (plans/nurbs-analytic-section step 8): the
//! same topology on the same curves, every analytic surface replaced by
//! [`Surface::to_nurbs`] of it over the parameter box the body needs.
//!
//! `to_nurbs` reparametrises an angle (a rational arc is not the angle),
//! so a pcurve is not reused: every use is drawn again on its twin with
//! [`arris_geom::pcurve_on`], same-parameter with its edge. A boolean over the twin must then agree with the boolean over the
//! original: volume, area and counts, or both refuse. That is the twin
//! property, `arris-ops`'s `twin_prop` test.

use std::collections::BTreeMap;

use arris_geom::{Curve, Curve2, Surface, pcurve_on};
use arris_math::{Interval, Meter, Point2, Point3, Tolerance, Vec2};
use arris_ops::{Fault, OpError};
use arris_topo::builder::{Assembly, Builder, GeometryRemap};
use arris_topo::entity::{BodyKind, EdgeGeometry};
use arris_topo::{Body, Curve2Id, Model, SurfaceId};

/// How many points of every edge bound the twin's box.
const SAMPLES: usize = 24;

/// How much wider than the body's reach a twin's unbounded direction is,
/// as a fraction of the reach: the pcurves between samples may bulge.
const MARGIN: f64 = 0.1;

/// The twin of each surface a remap is asked for, and for each use of an
/// edge on a twinned face the edge's curve and range and the tolerance to
/// draw its pcurve to.
struct Twins {
    surfaces: BTreeMap<SurfaceId, Surface>,
    uses: BTreeMap<Curve2Id, (Curve, Interval, Tolerance)>,
    /// The first pcurve that could not be drawn.
    failed: Option<arris_geom::GeomError>,
}

impl GeometryRemap for Twins {
    fn point(&mut self, _model: &mut Model, p: Point3) -> Point3 {
        p
    }

    fn curve(&mut self, _model: &mut Model, c: arris_topo::CurveId) -> arris_topo::CurveId {
        c
    }

    fn surface(&mut self, model: &mut Model, s: SurfaceId) -> SurfaceId {
        match self.surfaces.get(&s) {
            Some(twin) => model.add_surface(twin.clone()),
            None => s,
        }
    }

    fn pcurve(&mut self, model: &mut Model, p: Curve2Id, surface: SurfaceId) -> Curve2Id {
        let (Some(twin), Some((curve, range, tol))) =
            (self.surfaces.get(&surface), self.uses.get(&p))
        else {
            return p;
        };
        match pcurve_on(curve, *range, twin, *tol, &mut Meter::default()) {
            Ok(drawn) => {
                let analytic = model.surface(surface).ok().map(Surface::period);
                let was = model.curve2(p).map(|c| c.point(range.lo())).ok();
                let placed = match (analytic, was) {
                    (Some(analytic), Some(was)) => {
                        placed_like(drawn, range.lo(), twin, analytic, was)
                    }
                    _ => drawn,
                };
                model.add_curve2(placed)
            }
            Err(e) => {
                self.failed.get_or_insert(e);
                p
            }
        }
    }
}

/// `body` with every analytic surface replaced by its exact NURBS twin,
/// as a new body of `m`: a sphere and a torus over their whole domain, a
/// plane, a cylinder, an elliptic cylinder and a cone over the box the
/// body's edges reach in `(u, v)`, widened by [`MARGIN`] (a cone's never
/// past its apex). A surface that is already NURBS is kept as it is.
///
/// Errors: [`OpError::Internal`] naming the geometry refusal when a twin
/// cannot be made, and the builder's when the copy cannot be assembled.
///
/// ```
/// use arris_check::{Level, check};
/// use arris_debug::prop::twin::nurbs_twin;
/// use arris_debug::sample;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let cylinder = sample::cylinder(&mut m, 4.0, 12.0)?;
/// let twin = nurbs_twin(&mut m, cylinder)?;
/// assert!(check(&m, twin, Level::Full).is_ok());
/// assert_eq!(m.faces(twin)?.len(), m.faces(cylinder)?.len());
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn nurbs_twin(m: &mut Model, body: Body) -> Result<Body, OpError> {
    let kind = m.body(body.id)?.kind();
    let closure = m.closure(body)?;

    let mut points = Vec::new();
    for &e in &closure.edges {
        let edge = *m.edge(e)?;
        if let EdgeGeometry::Curve { curve, range } = edge.geometry() {
            let curve = m.curve(curve)?;
            for i in 0..=SAMPLES {
                points.push(curve.point(range.lerp(i as f64 / SAMPLES as f64)));
            }
        }
    }

    let mut twins = BTreeMap::new();
    for &s in &closure.surfaces {
        let surface = m.surface(s)?.clone();
        let bounds = match &surface {
            Surface::Nurbs(_) => continue,
            Surface::Sphere { .. } | Surface::Torus { .. } => surface.domain(),
            Surface::Plane { .. } => [reach(&surface, &points, 0), reach(&surface, &points, 1)],
            Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
                [Interval::TURN, reach(&surface, &points, 1)]
            }
        };
        let twin = surface
            .to_nurbs(bounds)
            .map_err(|e| OpError::Internal(Fault::Geometry(e)))?;
        twins.insert(s, Surface::Nurbs(twin));
    }

    let tolerance = m.precision().default_tolerance;
    let angular = m.precision().tolerance().angular;
    let mut uses = BTreeMap::new();
    for &f in &closure.faces {
        for l in m.face(f)?.loops() {
            for c in l.coedges() {
                let edge = *m.edge(c.edge())?;
                if let EdgeGeometry::Curve { curve, range } = edge.geometry() {
                    let tol = Tolerance::new(edge.tolerance(), angular);
                    uses.insert(c.pcurve(), (m.curve(curve)?.clone(), range, tol));
                }
            }
        }
    }
    let mut remap = Twins {
        surfaces: twins,
        uses,
        failed: None,
    };
    m.transaction(|m| {
        let (assembly, _) = Assembly::of_body(m, body, &mut remap)?;
        if let Some(e) = remap.failed.take() {
            return Err(OpError::Internal(Fault::Geometry(e)));
        }
        let (b, _) = match kind {
            BodyKind::Sheet => Builder::assemble_sheet(m, tolerance, assembly)?,
            _ => Builder::assemble(m, tolerance, assembly)?,
        };
        Ok(b.finish(m, kind)?.body)
    })
}

/// The interval of parameter `axis` (0 for `u`, 1 for `v`) `points` reach
/// on `surface`, widened by [`MARGIN`] of its length and at least one
/// unit: a point `project` finds ambiguous (on an axis) is skipped. A cone
/// keeps the sign of its `v`, so the widening never reaches the apex.
fn reach(surface: &Surface, points: &[Point3], axis: usize) -> Interval {
    let values: Vec<f64> = points
        .iter()
        .filter_map(|&p| surface.project(p).ok())
        .map(|proj| proj.uv[axis])
        .collect();
    let lo = values.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = values.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if !(lo.is_finite() && hi.is_finite()) {
        return Interval::UNIT;
    }
    let pad = (MARGIN * (hi - lo)).max(1.0);
    let (lo, hi) = match surface {
        Surface::Cone { .. } if lo > 0.0 => ((lo - pad).max(lo / 2.0), hi + pad),
        Surface::Cone { .. } if hi < 0.0 => (lo - pad, (hi + pad).min(hi / 2.0)),
        _ => (lo - pad, hi + pad),
    };
    Interval::new(lo, hi).unwrap_or(Interval::UNIT)
}

/// `drawn` moved by whole periods of the twin so that it starts on the
/// same side of each periodic seam as the analytic pcurve did, `was` at
/// `t`. The twin's parameter is a monotone reparametrisation of the
/// analytic one that agrees at every arc end, so the fraction of the
/// period a point lies at differs by less than half between them, and the
/// whole periods between the two fractions are the seam's side: a seam
/// edge's two uses differ by exactly the period.
fn placed_like(
    drawn: Curve2,
    t: f64,
    twin: &Surface,
    analytic: [Option<f64>; 2],
    was: Point2,
) -> Curve2 {
    let (domain, period) = (twin.domain(), twin.period());
    let at = drawn.point(t);
    let mut by = Vec2::zeros();
    for k in 0..2 {
        if let (Some(p_a), Some(p_t)) = (analytic[k], period[k]) {
            let fraction_a = was[k] / p_a;
            let fraction_t = (at[k] - domain[k].lo()) / p_t;
            by[k] = (fraction_a - fraction_t).round() * p_t;
        }
    }
    if by == Vec2::zeros() {
        drawn
    } else {
        drawn.translated(by)
    }
}
