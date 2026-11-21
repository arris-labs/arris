//! Axis extrema of sphere and torus faces, classified in their trim charts.

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_geom::{Surface, region2::Piece, region2::Side};
use arris_math::{Aabb, Interrupted, Meter, Point2, Point3, Vec3, real};

use super::domain::TrimDomain;

pub(super) fn face_box(
    surface: &Surface,
    loops: &[Vec<Piece<'_>>],
    boundary: Option<Aabb>,
    chord: f64,
    eps: f64,
    meter: &mut Meter<'_>,
) -> Result<Option<Aabb>, Interrupted> {
    if loops.is_empty() {
        let bounds = surface.bounds(surface.domain());
        return Ok(match (boundary, bounds) {
            (Some(a), Some(b)) => Some(a.union(b)),
            (a, b) => a.or(b),
        });
    }
    let Some(boundary) = boundary else {
        return Ok(None);
    };
    let scale = match surface {
        Surface::Sphere { radius, .. } => radius.abs(),
        Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => major_radius.abs() + minor_radius.abs(),
        Surface::Plane { .. }
        | Surface::Cylinder { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Cone { .. }
        | Surface::Nurbs(_) => return Ok(Some(boundary)),
    };
    // The second partials along any world axis are at most `scale`.
    // Near a stationary point its coordinate changes by at most
    // `scale * |duv|²`. An ambiguous point is within twice the chord's
    // reach of the true trim: every chord location has a pcurve point
    // projecting there, by continuity. Leave room for rounding.
    let chord = chord.min(0.25 * (eps / scale).sqrt());
    let Some(domain) = TrimDomain::of(loops, surface.period(), chord, meter)? else {
        return Ok(None);
    };
    let [u, _] = domain.uv_box().unwrap_or_else(|| surface.domain());
    let mut result = boundary;
    let mut unresolved = boundary;
    for (k, uv, p) in stationary_points(surface, u.midpoint()) {
        let value = p[k];
        if (result.min[k]..=result.max[k]).contains(&value) {
            continue;
        }
        let target = match domain.side(uv, meter)? {
            Side::Outside => continue,
            Side::Inside => &mut result,
            // The lifted trim box already certifies its sides against
            // real trim points. Charge this extra distance to the
            // remaining face allowance and include only the axis whose
            // extremum was being tested.
            Side::Boundary if (boundary.min[k] - eps..=boundary.max[k] + eps).contains(&value) => {
                &mut result
            }
            Side::Boundary => &mut unresolved,
        };
        target.min[k] = target.min[k].min(value);
        target.max[k] = target.max[k].max(value);
    }
    Ok((result.union(unresolved) == result).then_some(result))
}

fn stationary_points(surface: &Surface, u_mid: f64) -> Vec<(usize, Point2, Point3)> {
    let axes = [Vec3::x(), Vec3::y(), Vec3::z()];
    let mut points = Vec::new();
    match surface {
        Surface::Sphere { frame, radius } => {
            for (k, axis) in axes.into_iter().enumerate() {
                for d in [axis, -axis] {
                    let x = d.dot(frame.x().as_ref());
                    let y = d.dot(frame.y().as_ref());
                    let z = d.dot(frame.z().as_ref());
                    let h = real::hypot(x, y);
                    let u = if h == 0.0 {
                        u_mid
                    } else {
                        real::atan2(y, x).rem_euclid(TAU)
                    };
                    let v = real::atan2(z, h);
                    points.push((k, Point2::new(u, v), frame.origin() + *radius * d));
                }
            }
        }
        Surface::Torus { frame, .. } => {
            for (k, axis) in axes.into_iter().enumerate() {
                let x = axis.dot(frame.x().as_ref());
                let y = axis.dot(frame.y().as_ref());
                let z = axis.dot(frame.z().as_ref());
                let h = real::hypot(x, y);
                if h == 0.0 {
                    // The crest has constant height. If part of it is
                    // trimmed away, the trim reaches that same height.
                    for v in [FRAC_PI_2, 3.0 * FRAC_PI_2] {
                        points.push((k, Point2::new(u_mid, v), surface.point(u_mid, v)));
                    }
                } else {
                    // The two meridians have radial directions ±h.
                    // Each has two stationary angles a half turn apart.
                    let azimuth = real::atan2(y, x);
                    for (u, radial) in [(azimuth, h), (azimuth + PI, -h)] {
                        let u = u.rem_euclid(TAU);
                        let meridian = real::atan2(z, radial);
                        for v in [meridian, meridian + PI] {
                            let v = v.rem_euclid(TAU);
                            points.push((k, Point2::new(u, v), surface.point(u, v)));
                        }
                    }
                }
            }
        }
        Surface::Plane { .. }
        | Surface::Cylinder { .. }
        | Surface::EllipticCylinder { .. }
        | Surface::Cone { .. }
        | Surface::Nurbs(_) => {}
    }
    points
}
