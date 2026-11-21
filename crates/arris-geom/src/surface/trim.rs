//! Bounds of a surface restricted to a pcurve.

use arris_math::nalgebra::Vector4;
use arris_math::{Aabb, Frame, Interrupted, Interval, Meter, Point2, Point3, Vec3, real};

use crate::curve::{coords, sinusoid_range};
use crate::curve2::chord::{Span, intervals};
use crate::nurbs::Patch;
use crate::{Curve, Curve2, NurbsCurve, Surface};

impl Surface {
    /// Join an existing outer box with the image of a pcurve over `range`.
    /// Any side added to the box is within `eps` of a point of that image,
    /// to floating-point rounding. The supplied box can prune spans that
    /// cannot extend that side. The meter ticks per subdivision.
    ///
    /// Returns `None` if the range or tolerance is invalid or cannot be
    /// bounded to that tolerance at the available numeric precision.
    pub fn tight_trim_bounds(
        &self,
        curve: &Curve2,
        range: Interval,
        boundary: Aabb,
        eps: f64,
        meter: &mut Meter<'_>,
    ) -> Result<Option<Aabb>, Interrupted> {
        if !(range.is_bounded() && range.length() > 0.0 && eps.is_finite() && eps > 0.0) {
            return Ok(None);
        }
        if let Some(b) = coarse_lift(self, curve, range)
            && within(b, boundary, 0.0)
        {
            return Ok(Some(boundary));
        }
        if let Surface::Plane { frame } = self {
            let exact = plane_curve(frame, curve)
                .map(|curve| curve.tight_bounds_with(range, eps, meter))
                .transpose()?
                .flatten();
            if let Some(b) = exact {
                return Ok(Some(boundary.union(b)));
            }
        }
        if let Curve2::Line { direction, .. } = curve
            && let Some(fixed) = direction.iter().position(|&x| x == 0.0)
        {
            let Some(uv) = intervals(curve.point(range.lo()), curve.point(range.hi())) else {
                return Ok(None);
            };
            let parameter = uv[fixed].lo();
            let varying = uv[1 - fixed];
            if let Surface::Nurbs(surface) = self {
                if let Ok(c) = surface.isocurve(fixed, parameter) {
                    let domain = surface.domain()[1 - fixed];
                    // A closed nonperiodic isocurve wraps only when
                    // evaluated through the surface.
                    if domain.contains(varying.lo()) && domain.contains(varying.hi())
                        || c.period().is_some()
                    {
                        return Curve::Nurbs(c)
                            .tight_bounds_with(varying, eps, meter)
                            .map(|b| b.map(|b| boundary.union(b)));
                    }
                }
            } else {
                return Ok(self
                    .analytic_isoline(fixed, parameter, varying)
                    .map(|b| boundary.union(b)));
            }
        }

        let Some(spans) = curve.chord_spans(range) else {
            return Ok(None);
        };
        let mut inner = boundary;
        let mut outer = boundary;
        for span in spans {
            let Some((i, o)) = self.bound_trim_span(curve, span, inner, eps, meter)? else {
                return Ok(None);
            };
            inner = i;
            outer = outer.union(o);
        }
        Ok(Some(outer.union(inner)))
    }

    fn analytic_isoline(&self, fixed: usize, at: f64, range: Interval) -> Option<Aabb> {
        let e = if fixed == 0 {
            self.eval(at, 0.0)
        } else {
            self.eval(0.0, at)
        };
        let angular = fixed == 1 || matches!(self, Surface::Sphere { .. } | Surface::Torus { .. });
        if !angular {
            return Aabb::of_points(&[
                coords(e.point + range.lo() * e.dv),
                coords(e.point + range.hi() * e.dv),
            ]);
        }
        // An angular isoline has S(t) = centre + a cos(t) + b sin(t).
        let (a, b) = if fixed == 0 {
            (-e.dvv, e.dv)
        } else {
            (-e.duu, e.du)
        };
        let centre = e.point - a;
        Some(super::axis_bounds(|k| {
            sinusoid_range(a[k], b[k], range).map(|x| x + centre[k])
        }))
    }

    fn bound_trim_span(
        &self,
        curve: &Curve2,
        span: Span,
        mut inner: Aabb,
        eps: f64,
        meter: &mut Meter<'_>,
    ) -> Result<Option<(Aabb, Aabb)>, Interrupted> {
        let mut outer = inner;
        let mut pending = vec![(span, None)];
        while let Some((span, envelope)) = pending.pop() {
            let Some(chord) = span.chord(curve) else {
                let Some([a, b]) = span.split() else {
                    return Ok(None);
                };
                meter.tick()?;
                pending.extend([(b, None), (a, None)]);
                continue;
            };
            let Some(surface) =
                envelope.or_else(|| TrimEnvelope::new(self, chord.uv).map(std::rc::Rc::new))
            else {
                let Some([a, b]) = span.split() else {
                    return Ok(None);
                };
                meter.tick()?;
                pending.extend([(b, None), (a, None)]);
                continue;
            };
            let Some((rectangle, derivatives, smooth)) = surface.bounds(chord.uv) else {
                return Ok(None);
            };
            if within(rectangle, inner, 0.0) {
                continue;
            }
            let [a, b] = chord.ends;
            let (a3, b3) = (self.point(a.x, a.y), self.point(b.x, b.y));
            if !a3
                .coords
                .iter()
                .chain(b3.coords.iter())
                .all(|x| x.is_finite())
            {
                return Ok(None);
            }
            let ends = Aabb::of_point(a3).union(Aabb::of_point(b3));
            inner = inner.union(ends);

            // On the lifted straight UV chord, each axis deviates from
            // its linear interpolant by at most max|d²/dt²| / 8. A
            // chord crossing a crease still has bounded one-sided speed.
            let delta = b - a;
            let dd = derivatives.acceleration(delta.abs());
            let mut radius = if smooth {
                dd / 8.0
            } else {
                derivatives.lipschitz(delta.abs()) / 2.0
            };
            let error = derivatives.lipschitz(chord.deviation);
            radius += error;
            let mut hull = intersect_enclosures(ends, rectangle, radius);
            if !within(hull, inner, eps) && smooth {
                // A chord along which a world coordinate is monotone
                // has its extrema at the endpoints. The Lipschitz term
                // for the pcurve's deviation still applies.
                let uv = a + 0.5 * delta;
                let e = self.eval(uv.x, uv.y);
                let slope = delta.x * e.du + delta.y * e.dv;
                for k in 0..3 {
                    if slope[k].abs() > dd[k] / 2.0 {
                        radius[k] = error[k];
                    }
                }
                hull = intersect_enclosures(ends, rectangle, radius);
            }
            if within(hull, inner, eps) {
                outer = outer.union(hull);
                continue;
            }
            let Some([a, b]) = span.split() else {
                return Ok(None);
            };
            meter.tick()?;
            pending.extend([(b, Some(surface.clone())), (a, Some(surface))]);
        }
        Ok(Some((inner, outer)))
    }
}

/// The unsplit control hull of the pcurve, lifted by an affine map or
/// enclosed in a parameter rectangle. Common interior pcurves need no
/// span extraction.
fn coarse_lift(surface: &Surface, curve: &Curve2, range: Interval) -> Option<Aabb> {
    let Curve2::Nurbs(curve) = curve else {
        return None;
    };
    let domain = curve.domain();
    if curve.period().is_none() && !(domain.contains(range.lo()) && domain.contains(range.hi())) {
        return None;
    }
    let mut uv: Option<[Interval; 2]> = None;
    if let Surface::Plane { frame } = surface {
        return curve
            .control_points()
            .iter()
            .fold(None, |acc: Option<Aabb>, p| {
                let b = Aabb::of_point(frame.to_world(Point3::new(p.x, p.y, 0.0)));
                Some(acc.map_or(b, |a| a.union(b)))
            });
    }
    for &p in curve.control_points() {
        let [u, v] = uv.unwrap_or(intervals(p, p)?);
        uv = Some([
            Interval::new(u.lo().min(p.x), u.hi().max(p.x)).ok()?,
            Interval::new(v.lo().min(p.y), v.hi().max(p.y)).ok()?,
        ]);
    }
    let uv = uv?;
    if let Surface::Nurbs(nurbs) = surface {
        let domain = nurbs.domain();
        let period = nurbs.closure();
        if (0..2).any(|k| {
            period[k].is_none()
                && !(domain[k].contains(uv[k].lo()) && domain[k].contains(uv[k].hi()))
        }) {
            return None;
        }
        return nurbs
            .control_points()
            .iter()
            .fold(None, |acc: Option<Aabb>, p| {
                let b = Aabb::of_point(*p);
                Some(acc.map_or(b, |a| a.union(b)))
            });
    }
    surface.bounds(uv)
}

fn within(outer: Aabb, inner: Aabb, eps: f64) -> bool {
    (0..3).all(|k| inner.min[k] - outer.min[k] <= eps && outer.max[k] - inner.max[k] <= eps)
}

fn intersect_enclosures(ends: Aabb, rectangle: Aabb, radius: Vec3) -> Aabb {
    Aabb {
        min: std::array::from_fn(|k| (ends.min[k] - radius[k]).max(rectangle.min[k])),
        max: std::array::from_fn(|k| (ends.max[k] + radius[k]).min(rectangle.max[k])),
    }
}

fn plane_curve(plane: &Frame, curve: &Curve2) -> Option<Curve> {
    let lift = |p: Point2| plane.to_world(Point3::new(p.x, p.y, 0.0));
    let vector = |v: arris_math::Vec2| plane.vec_to_world(Vec3::new(v.x, v.y, 0.0));
    let frame = |local: &arris_math::Frame2| {
        let x = vector(local.x().into_inner());
        let y = vector(local.y().into_inner());
        Frame::from_orthonormal(lift(local.origin()), x, y, x.cross(&y)).ok()
    };
    Some(match curve {
        Curve2::Line { origin, direction } => Curve::Line {
            origin: lift(*origin),
            direction: arris_math::UnitVec3::new_unchecked(vector(direction.into_inner())),
        },
        Curve2::Circle { frame: f, radius } => Curve::Circle {
            frame: frame(f)?,
            radius: *radius,
        },
        Curve2::Ellipse {
            frame: f,
            major_radius,
            minor_radius,
        } => Curve::Ellipse {
            frame: frame(f)?,
            major_radius: *major_radius,
            minor_radius: *minor_radius,
        },
        Curve2::Nurbs(c) => Curve::Nurbs(
            NurbsCurve::new(
                c.degree(),
                c.knots().to_vec(),
                c.control_points().iter().copied().map(lift).collect(),
                c.weights().to_vec(),
            )
            .ok()?,
        ),
    })
}

#[derive(Clone, Copy, Default)]
struct Partials {
    u: Vec3,
    v: Vec3,
    uu: Vec3,
    uv: Vec3,
    vv: Vec3,
}

impl Partials {
    fn acceleration(self, d: arris_math::Vec2) -> Vec3 {
        self.uu * (d.x * d.x) + self.uv * (2.0 * d.x * d.y) + self.vv * (d.y * d.y)
    }

    fn lipschitz(self, d: arris_math::Vec2) -> Vec3 {
        self.u * d.x + self.v * d.y
    }

    fn union(self, other: Self) -> Self {
        let max = |a: Vec3, b: Vec3| a.zip_map(&b, f64::max);
        Self {
            u: max(self.u, other.u),
            v: max(self.v, other.v),
            uu: max(self.uu, other.uu),
            uv: max(self.uv, other.uv),
            vv: max(self.vv, other.vv),
        }
    }
}

enum TrimEnvelope<'a> {
    Analytic(&'a Surface),
    Nurbs(Vec<NurbsEnvelope>),
}

impl<'a> TrimEnvelope<'a> {
    fn new(surface: &'a Surface, uv: [Interval; 2]) -> Option<Self> {
        match surface {
            Surface::Nurbs(n) => Some(Self::Nurbs(
                n.trim_patches(uv)?
                    .into_iter()
                    .map(NurbsEnvelope::new)
                    .collect::<Option<_>>()?,
            )),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. } => Some(Self::Analytic(surface)),
        }
    }

    fn bounds(&self, uv: [Interval; 2]) -> Option<(Aabb, Partials, bool)> {
        match self {
            Self::Analytic(s) => Some((s.bounds(uv)?, analytic_partials(s, uv), true)),
            Self::Nurbs(patches) => {
                let mut bounds: Option<Aabb> = None;
                let mut partials = Partials::default();
                let mut count = 0;
                for patch in patches {
                    // At a span boundary, the adjoining patch gives the
                    // same endpoint by continuity. Include its derivative
                    // only when the curve can lie on both sides of the knot.
                    let overlaps = (0..2).all(|k| {
                        let [lo, hi] = patch.uv[k];
                        if uv[k].lo() == uv[k].hi() {
                            lo <= uv[k].lo() && uv[k].hi() <= hi
                        } else {
                            lo < uv[k].hi() && uv[k].lo() < hi
                        }
                    });
                    if overlaps {
                        bounds = Some(bounds.map_or(patch.hull, |b| b.union(patch.hull)));
                        partials = partials.union(patch.partials);
                        count += 1;
                    }
                }
                Some((bounds?, partials, count == 1))
            }
        }
    }
}

struct NurbsEnvelope {
    uv: [[f64; 2]; 2],
    hull: Aabb,
    partials: Partials,
}

impl NurbsEnvelope {
    fn new(patch: Patch) -> Option<Self> {
        let [p, q] = patch.degree;
        let u_scale = inverse_width(patch.u);
        let v_scale = inverse_width(patch.v);
        let mut hull: Option<Aabb> = None;
        let mut min_w = f64::INFINITY;
        let origin = cartesian(*patch.net.first()?);
        let mut radius = Vec3::zeros();
        let mut relative = Vec::with_capacity(patch.net.len());
        for h in patch.net {
            let w = h.w;
            let pt = cartesian(h);
            if !(w > 0.0 && w.is_finite() && pt.iter().all(|x| x.is_finite())) {
                return None;
            }
            let b = Aabb::of_point(Point3::from(pt));
            hull = Some(hull.map_or(b, |b| b.union(b)));
            min_w = min_w.min(w);
            let v = pt - origin;
            radius = radius.zip_map(&v.abs(), f64::max);
            relative.push(h - w * Vector4::new(origin.x, origin.y, origin.z, 0.0));
        }
        let at = |i, j| relative[i * (q + 1) + j];
        let u = |i, j| (p as f64 * u_scale) * (at(i + 1, j) - at(i, j));
        let v = |i, j| (q as f64 * v_scale) * (at(i, j + 1) - at(i, j));
        let magnitude =
            |rows: usize, columns: usize, difference: &dyn Fn(usize, usize) -> Vector4<f64>| {
                (0..rows)
                    .flat_map(|i| (0..columns).map(move |j| difference(i, j)))
                    .fold(Vector4::zeros(), |a, b| a.zip_map(&b.abs(), f64::max))
            };
        let hu = magnitude(p, q + 1, &u);
        let hv = magnitude(p + 1, q, &v);
        let huu = magnitude(p - 1, q + 1, &|i, j| {
            ((p - 1) as f64 * u_scale) * (u(i + 1, j) - u(i, j))
        });
        let huv = magnitude(p, q, &|i, j| (q as f64 * v_scale) * (u(i, j + 1) - u(i, j)));
        let hvv = magnitude(p + 1, q - 1, &|i, j| {
            ((q - 1) as f64 * v_scale) * (v(i, j + 1) - v(i, j))
        });
        // Bound each quotient-rule term by the convex hull of its
        // Bernstein derivative net. A constant translation of the
        // surface reduces the bound on the undifferentiated term.
        let quotient = |h: Vector4<f64>| Vec3::new(h.x, h.y, h.z) + h.w * radius;
        let u = quotient(hu) / min_w;
        let v = quotient(hv) / min_w;
        let partials = Partials {
            u,
            v,
            uu: (quotient(huu) + 2.0 * hu.w * u) / min_w,
            uv: (quotient(huv) + hu.w * v + hv.w * u) / min_w,
            vv: (quotient(hvv) + 2.0 * hv.w * v) / min_w,
        };
        if ![
            partials.u,
            partials.v,
            partials.uu,
            partials.uv,
            partials.vv,
        ]
        .iter()
        .flat_map(|d| d.iter())
        .all(|x| x.is_finite())
        {
            return None;
        }
        Some(Self {
            uv: [patch.u, patch.v],
            hull: hull?,
            partials,
        })
    }
}

fn inverse_width([a, b]: [f64; 2]) -> f64 {
    if a == b { 0.0 } else { 1.0 / (b - a) }
}

fn cartesian(h: Vector4<f64>) -> Vec3 {
    Vec3::new(h.x, h.y, h.z) / h.w
}

fn analytic_partials(surface: &Surface, uv: [Interval; 2]) -> Partials {
    let Some(frame) = surface.frame() else {
        return Partials::default();
    };
    let x = frame.x();
    let y = frame.y();
    let z = frame.z().abs();
    let radial = |a, b| Vec3::from_fn(|k, _| real::hypot(a * x[k], b * y[k]));
    match *surface {
        Surface::Plane { .. } => Partials {
            u: x.abs(),
            v: y.abs(),
            ..Partials::default()
        },
        Surface::Cylinder { radius, .. } => {
            let r = radial(radius, radius);
            Partials {
                u: r,
                v: z,
                uu: r,
                ..Partials::default()
            }
        }
        Surface::EllipticCylinder {
            major_radius,
            minor_radius,
            ..
        } => {
            let r = radial(major_radius, minor_radius);
            Partials {
                u: r,
                v: z,
                uu: r,
                ..Partials::default()
            }
        }
        Surface::Cone {
            radius, half_angle, ..
        } => {
            let (sin, cos) = real::sin_cos(half_angle);
            let v = uv[1];
            let rho = (radius + v.lo() * sin)
                .abs()
                .max((radius + v.hi() * sin).abs());
            let r = radial(rho, rho);
            let slope = radial(sin, sin);
            Partials {
                u: r,
                v: slope + cos.abs() * z,
                uu: r,
                uv: slope,
                vv: Vec3::zeros(),
            }
        }
        Surface::Sphere { radius, .. } => {
            let r = radial(radius, radius);
            Partials {
                u: r,
                v: Vec3::repeat(radius.abs()),
                uu: r,
                uv: r,
                vv: Vec3::repeat(radius.abs()),
            }
        }
        Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => {
            let r = radial(
                major_radius.abs() + minor_radius.abs(),
                major_radius.abs() + minor_radius.abs(),
            );
            let t = Vec3::repeat(minor_radius.abs());
            Partials {
                u: r,
                v: t,
                uu: r,
                uv: radial(minor_radius, minor_radius),
                vv: t,
            }
        }
        Surface::Nurbs(_) => Partials::default(),
    }
}
