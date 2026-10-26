//! M3: every coordinate, parameter and tolerance is finite.

use std::collections::BTreeSet;

use arris_geom::{Curve, Curve2, Surface};
use arris_math::{Frame, Frame2};
use arris_topo::EntityId;

use super::{Checker, coedges};
use crate::violation::{Quantity, Violation};

fn all_finite(xs: impl IntoIterator<Item = f64>) -> bool {
    xs.into_iter().all(f64::is_finite)
}

fn frame_finite(f: &Frame) -> bool {
    all_finite(
        f.origin()
            .coords
            .iter()
            .chain(f.x().iter())
            .chain(f.y().iter())
            .chain(f.z().iter())
            .copied(),
    )
}

fn frame2_finite(f: &Frame2) -> bool {
    all_finite(
        f.origin()
            .coords
            .iter()
            .chain(f.x().iter())
            .chain(f.y().iter())
            .copied(),
    )
}

/// Which quantities of a curve are non-finite: `(coordinates, parameters)`.
fn curve_non_finite(c: &Curve) -> (bool, bool) {
    match c {
        Curve::Line { origin, direction } => (
            !all_finite(origin.coords.iter().chain(direction.iter()).copied()),
            false,
        ),
        Curve::Circle { frame, radius } => (!frame_finite(frame), !radius.is_finite()),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => (
            !frame_finite(frame),
            !(major_radius.is_finite() && minor_radius.is_finite()),
        ),
        Curve::Nurbs(n) => (
            !all_finite(
                n.control_points()
                    .iter()
                    .flat_map(|p| p.coords.iter().copied()),
            ),
            !all_finite(n.knots().iter().chain(n.weights()).copied()),
        ),
    }
}

fn surface_non_finite(s: &Surface) -> (bool, bool) {
    match s {
        Surface::Plane { frame } => (!frame_finite(frame), false),
        Surface::Cylinder { frame, radius } | Surface::Sphere { frame, radius } => {
            (!frame_finite(frame), !radius.is_finite())
        }
        Surface::Cone {
            frame,
            radius,
            half_angle,
        } => (
            !frame_finite(frame),
            !(radius.is_finite() && half_angle.is_finite()),
        ),
        Surface::Torus {
            frame,
            major_radius,
            minor_radius,
        }
        | Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        } => (
            !frame_finite(frame),
            !(major_radius.is_finite() && minor_radius.is_finite()),
        ),
        Surface::Nurbs(n) => (
            !all_finite(
                n.control_points()
                    .iter()
                    .flat_map(|p| p.coords.iter().copied()),
            ),
            !all_finite(
                n.knots()
                    .iter()
                    .flat_map(|k| k.iter())
                    .chain(n.weights())
                    .copied(),
            ),
        ),
    }
}

/// A pcurve's numbers are all parameters.
fn pcurve_non_finite(c: &Curve2) -> bool {
    match c {
        Curve2::Line { origin, direction } => {
            !all_finite(origin.coords.iter().chain(direction.iter()).copied())
        }
        Curve2::Circle { frame, radius } => !frame2_finite(frame) || !radius.is_finite(),
        Curve2::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => !frame2_finite(frame) || !(major_radius.is_finite() && minor_radius.is_finite()),
        Curve2::Nurbs(n) => !all_finite(
            n.control_points()
                .iter()
                .flat_map(|p| p.coords.iter().copied())
                .chain(n.knots().iter().copied())
                .chain(n.weights().iter().copied()),
        ),
    }
}

impl Checker<'_> {
    /// M3: every coordinate, parameter and tolerance is finite, reported
    /// against the entity holding or referencing the number.
    pub(crate) fn finiteness(&mut self) {
        let mut found: BTreeSet<(EntityId, Quantity)> = BTreeSet::new();
        for &id in &self.closure.vertices {
            let Ok(v) = self.model.vertex(id) else {
                continue;
            };
            if !all_finite(v.point().coords.iter().copied()) {
                found.insert((id.into(), Quantity::Coordinate));
            }
            if !v.tolerance().is_finite() {
                found.insert((id.into(), Quantity::Tolerance));
            }
        }
        for &id in &self.closure.edges {
            let Ok(e) = self.model.edge(id) else {
                continue;
            };
            let range = e.range();
            if !(range.lo().is_finite() && range.hi().is_finite()) {
                found.insert((id.into(), Quantity::Parameter));
            }
            if !e.tolerance().is_finite() {
                found.insert((id.into(), Quantity::Tolerance));
            }
            if let Some(curve) = e.curve().and_then(|(c, _)| self.model.curve(c).ok()) {
                let (coords, params) = curve_non_finite(curve);
                if coords {
                    found.insert((id.into(), Quantity::Coordinate));
                }
                if params {
                    found.insert((id.into(), Quantity::Parameter));
                }
            }
        }
        for &id in &self.closure.faces {
            let Ok(f) = self.model.face(id) else {
                continue;
            };
            if !f.tolerance().is_finite() {
                found.insert((id.into(), Quantity::Tolerance));
            }
            if let Ok(surface) = self.model.surface(f.surface()) {
                let (coords, params) = surface_non_finite(surface);
                if coords {
                    found.insert((id.into(), Quantity::Coordinate));
                }
                if params {
                    found.insert((id.into(), Quantity::Parameter));
                }
            }
            for (_, _, coedge) in coedges(f) {
                if let Ok(pcurve) = self.model.curve2(coedge.pcurve()) {
                    if pcurve_non_finite(pcurve) {
                        found.insert((id.into(), Quantity::Parameter));
                    }
                }
            }
        }
        for (entity, quantity) in found {
            self.push(Violation::NonFinite { entity, quantity });
        }
    }
}
