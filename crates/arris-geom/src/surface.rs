//! Surfaces and their evaluation.

use core::f64::consts::{FRAC_PI_2, TAU};
use core::fmt;

use arris_math::{
    Aabb, Frame, Interval, Isometry, Point3, Reflection, Tolerance, UnitVec3, Vec3, is_negligible,
};

use crate::curve::{active_points, coords, linear_range, product_range, sinusoid_range};
use crate::{GeomError, GeomKind, NurbsCurve, NurbsSurface};

/// A surface, placed by its frame, with the parametrisation of
/// `docs/DATA-MODEL.md` §Surfaces (the one Open CASCADE's `Geom`
/// classes use, so STEP round-trips without re-parametrising).
///
/// The fields are plain data: a `Surface` is a value the arena stores once
/// and never modifies, and its validity (positive radii, a torus with
/// `major_radius > minor_radius`) is the checker's to enforce; the NURBS
/// variant is valid by its constructor. Evaluation of any finite value
/// never panics.
///
/// ```
/// use arris_geom::Surface;
/// use arris_math::{Frame, Point3};
/// use core::f64::consts::FRAC_PI_2;
///
/// let cyl = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
/// let e = cyl.eval(FRAC_PI_2, 3.0);
/// assert!((e.point - Point3::new(0.0, 2.0, 3.0)).norm() < 1e-15);
/// let n = cyl.normal(FRAC_PI_2, 3.0).unwrap();
/// assert!((n.y - 1.0).abs() < 1e-15);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Surface {
    /// `P(u, v) = O + u·X + v·Y`; normal `Z`.
    Plane {
        /// Origin and axes.
        frame: Frame,
    },
    /// `P(u, v) = O + R(cos u·X + sin u·Y) + v·Z`; seam at `u = 0`.
    Cylinder {
        /// `Z` is the axis; `X` points at the seam.
        frame: Frame,
        /// `R`.
        radius: f64,
    },
    /// `P(u, v) = O + a cos u·X + b sin u·Y + v·Z` with `a ≥ b`: an
    /// ellipse swept along its normal, what an extruded elliptic profile
    /// segment sweeps (ADR-0014); seam at `u = 0`, the ruling through
    /// `O + a·X`.
    EllipticCylinder {
        /// `Z` is the axis; `X` is the section's major axis and points at
        /// the seam.
        frame: Frame,
        /// `a`, along `X`.
        major_radius: f64,
        /// `b`, along `Y`.
        minor_radius: f64,
    },
    /// `P(u, v) = O + (R + v sin α)(cos u·X + sin u·Y) + v cos α·Z`; the
    /// apex is at `v = −R / sin α`.
    Cone {
        /// `Z` is the axis; `X` points at the seam.
        frame: Frame,
        /// `R`, the radius at `v = 0`.
        radius: f64,
        /// `α ∈ (0, π/2)`, the angle between a ruling and the axis.
        half_angle: f64,
    },
    /// `P(u, v) = O + R cos v (cos u·X + sin u·Y) + R sin v·Z`; poles at
    /// `v = ±π/2`.
    Sphere {
        /// `Z` runs pole to pole; `X` points at the seam.
        frame: Frame,
        /// `R`.
        radius: f64,
    },
    /// `P(u, v) = O + (R + r cos v)(cos u·X + sin u·Y) + r sin v·Z`.
    Torus {
        /// `Z` is the axis; `X` points at the `u` seam.
        frame: Frame,
        /// `R`, from the axis to the tube's centre circle.
        major_radius: f64,
        /// `r`, the tube's radius; `r < R` in cycle 1.
        minor_radius: f64,
    },
    /// A rational B-spline; parametrised by its knots, no frame.
    Nurbs(NurbsSurface),
}

/// The fieldless twin of [`Surface`], for errors and dispatch tables.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum SurfaceKind {
    /// [`Surface::Plane`].
    Plane,
    /// [`Surface::Cylinder`].
    Cylinder,
    /// [`Surface::EllipticCylinder`].
    EllipticCylinder,
    /// [`Surface::Cone`].
    Cone,
    /// [`Surface::Sphere`].
    Sphere,
    /// [`Surface::Torus`].
    Torus,
    /// [`Surface::Nurbs`].
    Nurbs,
}

impl fmt::Display for SurfaceKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            SurfaceKind::Plane => "plane",
            SurfaceKind::Cylinder => "cylinder",
            SurfaceKind::EllipticCylinder => "elliptic cylinder",
            SurfaceKind::Cone => "cone",
            SurfaceKind::Sphere => "sphere",
            SurfaceKind::Torus => "torus",
            SurfaceKind::Nurbs => "NURBS",
        })
    }
}

/// A singular point of a surface's parametrisation, where every value of
/// one parameter names one point: a cone's apex, a sphere's pole, a
/// NURBS surface's collapsed row ([`Surface::singularities`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Singularity {
    /// The point.
    pub point: Point3,
    /// Which parameter is fixed along the row: `1` where every `u` names
    /// the point at one `v`, as on the analytic surfaces.
    pub fixed: usize,
    /// The value it is fixed at.
    pub value: f64,
}

/// A surface evaluated at one `(u, v)`: the point and its derivatives to
/// second order. Derivatives are with respect to the parameters as
/// stored, never normalised.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceEval {
    /// `P(u, v)`.
    pub point: Point3,
    /// `∂P/∂u`.
    pub du: Vec3,
    /// `∂P/∂v`.
    pub dv: Vec3,
    /// `∂²P/∂u²`.
    pub duu: Vec3,
    /// `∂²P/∂u∂v`.
    pub duv: Vec3,
    /// `∂²P/∂v²`.
    pub dvv: Vec3,
}

impl Surface {
    /// Which variant this is.
    pub fn kind(&self) -> SurfaceKind {
        match self {
            Surface::Plane { .. } => SurfaceKind::Plane,
            Surface::Cylinder { .. } => SurfaceKind::Cylinder,
            Surface::EllipticCylinder { .. } => SurfaceKind::EllipticCylinder,
            Surface::Cone { .. } => SurfaceKind::Cone,
            Surface::Sphere { .. } => SurfaceKind::Sphere,
            Surface::Torus { .. } => SurfaceKind::Torus,
            Surface::Nurbs(_) => SurfaceKind::Nurbs,
        }
    }

    /// The placing frame of an analytic surface; `None` for a NURBS, which
    /// is placed by its control points.
    pub fn frame(&self) -> Option<&Frame> {
        match self {
            Surface::Plane { frame }
            | Surface::Cylinder { frame, .. }
            | Surface::EllipticCylinder { frame, .. }
            | Surface::Cone { frame, .. }
            | Surface::Sphere { frame, .. }
            | Surface::Torus { frame, .. } => Some(frame),
            Surface::Nurbs(_) => None,
        }
    }

    /// The point and its derivatives to second order at `(u, v)`. Defined
    /// for every finite parameter, inside the domain or not: a periodic
    /// parameter wraps, an unbounded one extends, a clamped NURBS
    /// extrapolates its end piece.
    pub fn eval(&self, u: f64, v: f64) -> SurfaceEval {
        let zero = Vec3::zeros();
        match self {
            Surface::Nurbs(s) => s.eval(u, v),
            Surface::Plane { frame } => {
                let (o, x, y, _) = axes(frame);
                SurfaceEval {
                    point: o + u * x + v * y,
                    du: x,
                    dv: y,
                    duu: zero,
                    duv: zero,
                    dvv: zero,
                }
            }
            &Surface::Cylinder { ref frame, radius } => {
                let (o, x, y, z) = axes(frame);
                let (su, cu) = u.sin_cos();
                let radial = cu * x + su * y;
                let tangential = -su * x + cu * y;
                SurfaceEval {
                    point: o + radius * radial + v * z,
                    du: radius * tangential,
                    dv: z,
                    duu: -radius * radial,
                    duv: zero,
                    dvv: zero,
                }
            }
            &Surface::EllipticCylinder {
                ref frame,
                major_radius,
                minor_radius,
            } => {
                let (o, x, y, z) = axes(frame);
                let (su, cu) = u.sin_cos();
                let radial = (major_radius * cu) * x + (minor_radius * su) * y;
                SurfaceEval {
                    point: o + radial + v * z,
                    du: (-major_radius * su) * x + (minor_radius * cu) * y,
                    dv: z,
                    duu: -radial,
                    duv: zero,
                    dvv: zero,
                }
            }
            &Surface::Cone {
                ref frame,
                radius,
                half_angle,
            } => {
                let (o, x, y, z) = axes(frame);
                let (su, cu) = u.sin_cos();
                let (sa, ca) = half_angle.sin_cos();
                let rho = radius + v * sa;
                let radial = cu * x + su * y;
                let tangential = -su * x + cu * y;
                SurfaceEval {
                    point: o + rho * radial + (v * ca) * z,
                    du: rho * tangential,
                    dv: sa * radial + ca * z,
                    duu: -rho * radial,
                    duv: sa * tangential,
                    dvv: zero,
                }
            }
            &Surface::Sphere { ref frame, radius } => {
                let (o, x, y, z) = axes(frame);
                let (su, cu) = u.sin_cos();
                let (sv, cv) = v.sin_cos();
                let radial = cu * x + su * y;
                let tangential = -su * x + cu * y;
                SurfaceEval {
                    point: o + (radius * cv) * radial + (radius * sv) * z,
                    du: (radius * cv) * tangential,
                    dv: (-radius * sv) * radial + (radius * cv) * z,
                    duu: (-radius * cv) * radial,
                    duv: (-radius * sv) * tangential,
                    dvv: (-radius * cv) * radial - (radius * sv) * z,
                }
            }
            &Surface::Torus {
                ref frame,
                major_radius,
                minor_radius,
            } => {
                let (o, x, y, z) = axes(frame);
                let (su, cu) = u.sin_cos();
                let (sv, cv) = v.sin_cos();
                let rho = major_radius + minor_radius * cv;
                let radial = cu * x + su * y;
                let tangential = -su * x + cu * y;
                SurfaceEval {
                    point: o + rho * radial + (minor_radius * sv) * z,
                    du: rho * tangential,
                    dv: (-minor_radius * sv) * radial + (minor_radius * cv) * z,
                    duu: -rho * radial,
                    duv: (-minor_radius * sv) * tangential,
                    dvv: (-minor_radius * cv) * radial - (minor_radius * sv) * z,
                }
            }
        }
    }

    /// `P(u, v)` alone.
    pub fn point(&self, u: f64, v: f64) -> Point3 {
        self.eval(u, v).point
    }

    /// The axis-aligned box the surface fills over the parameter
    /// rectangle `uv`, or `None` when either range is not finite — a
    /// box has finite corners, and an unbounded plane has no box.
    ///
    /// Exact for a plane (the rectangle's four corners) and a cylinder or
    /// an elliptic cylinder (a sinusoid in `u` per axis, and `v` along
    /// the axis); an outer
    /// bound for the surfaces whose two parameters multiply — a cone, a
    /// sphere, a torus — where the box is the product of the two
    /// intervals rather than of the pairs that actually occur, and for a
    /// NURBS, whose control hull over the spans the rectangle touches
    /// contains it.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Interval};
    /// use core::f64::consts::TAU;
    ///
    /// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let whole = wall
    ///     .bounds([Interval::TURN, Interval::new(0.0, 5.0).unwrap()])
    ///     .unwrap();
    /// assert_eq!(whole.min, [-2.0, -2.0, 0.0]);
    /// assert_eq!(whole.max, [2.0, 2.0, 5.0]);
    /// assert_eq!(TAU, Interval::TURN.length());
    /// ```
    pub fn bounds(&self, uv: [Interval; 2]) -> Option<Aabb> {
        let [u, v] = uv;
        if ![u.lo(), u.hi(), v.lo(), v.hi()]
            .iter()
            .all(|x| x.is_finite())
        {
            return None;
        }
        match self {
            Surface::Nurbs(s) => {
                let [du, dv] = [s.degree()[0], s.degree()[1]];
                let [ku, kv] = s.knots();
                let rows: Vec<usize> = active_points(ku, du, u).collect();
                let hull: Vec<[f64; 3]> = active_points(kv, dv, v)
                    .flat_map(|j| rows.iter().map(move |&i| (i, j)))
                    .filter_map(|(i, j)| s.control_point(i, j).map(coords))
                    .collect();
                Aabb::of_points(&hull)
            }
            Surface::Plane { .. } => {
                let corners: Vec<[f64; 3]> = [u.lo(), u.hi()]
                    .into_iter()
                    .flat_map(|a| [v.lo(), v.hi()].map(|b| coords(self.point(a, b))))
                    .collect();
                Aabb::of_points(&corners)
            }
            &Surface::Cylinder { ref frame, radius } => Some(axis_bounds(|k| {
                let (o, x, y, z) = axes3(frame, k);
                let radial = sinusoid_range(radius * x, radius * y, u);
                let along = linear_range(z, v);
                [o + radial[0] + along[0], o + radial[1] + along[1]]
            })),
            &Surface::EllipticCylinder {
                ref frame,
                major_radius,
                minor_radius,
            } => Some(axis_bounds(|k| {
                let (o, x, y, z) = axes3(frame, k);
                let radial = sinusoid_range(major_radius * x, minor_radius * y, u);
                let along = linear_range(z, v);
                [o + radial[0] + along[0], o + radial[1] + along[1]]
            })),
            &Surface::Cone {
                ref frame,
                radius,
                half_angle,
            } => {
                let (sa, ca) = half_angle.sin_cos();
                // The radius at `v`, which the sinusoid across the axis
                // is scaled by; both may change sign past the apex.
                let rho = {
                    let span = linear_range(sa, v);
                    [radius + span[0], radius + span[1]]
                };
                Some(axis_bounds(|k| {
                    let (o, x, y, z) = axes3(frame, k);
                    let radial = product_range(rho, sinusoid_range(x, y, u));
                    let along = linear_range(ca * z, v);
                    [o + radial[0] + along[0], o + radial[1] + along[1]]
                }))
            }
            &Surface::Sphere { ref frame, radius } => Some(axis_bounds(|k| {
                let (o, x, y, z) = axes3(frame, k);
                // `R cos v` scales the sinusoid across the axis and
                // `R sin v` runs along it.
                let scale = sinusoid_range(radius, 0.0, v);
                let radial = product_range(scale, sinusoid_range(x, y, u));
                let along = sinusoid_range(0.0, radius * z, v);
                [o + radial[0] + along[0], o + radial[1] + along[1]]
            })),
            &Surface::Torus {
                ref frame,
                major_radius,
                minor_radius,
            } => Some(axis_bounds(|k| {
                let (o, x, y, z) = axes3(frame, k);
                let tube = sinusoid_range(minor_radius, 0.0, v);
                let rho = [major_radius + tube[0], major_radius + tube[1]];
                let radial = product_range(rho, sinusoid_range(x, y, u));
                let along = sinusoid_range(0.0, minor_radius * z, v);
                [o + radial[0] + along[0], o + radial[1] + along[1]]
            })),
        }
    }

    /// The part of the surface over the parameter rectangle `bounds` as
    /// a rational B-spline, exactly: every point of the result is a point
    /// of the surface and every point of the surface over `bounds` is one
    /// of the result, to rounding. A plane is a bilinear patch and a
    /// cylinder, elliptic cylinder, cone, sphere and torus are the
    /// extrusion or revolution of a conic or a line
    /// ([`NurbsSurface::extrusion`], [`NurbsSurface::revolution`]), so
    /// what a reader converts a `SURFACE_OF_REVOLUTION` to and what an
    /// analytic face becomes are one representation. A NURBS is returned
    /// whole, whatever `bounds`.
    ///
    /// The parametrisation matches where the analytic one is linear —
    /// both a plane's parameters, a cylinder's, elliptic cylinder's and
    /// cone's `v` — and is otherwise a monotone reparametrisation that
    /// agrees at every arc end. The domain of the result is not `bounds`
    /// where an angle is involved, though it covers the same points. A
    /// range of a full turn in `u` is closed and not periodic; the sphere's
    /// `v = ±π/2` rows and the cone's apex row are collapsed to a single point exactly
    /// (`docs/DATA-MODEL.md` §Surfaces).
    ///
    /// Errors: [`GeomError::Degenerate`] for a `bounds` that is not
    /// finite, is empty in a direction, or spans more than a turn of an
    /// angle, and for a radius that is not finite and positive.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Interval, Point3};
    ///
    /// let ball = Surface::Sphere { frame: Frame::world(), radius: 2.0 };
    /// let twin = ball.to_nurbs(ball.domain()).unwrap();
    /// // Both poles are single points, and the twin closes in `u`.
    /// let [n, m] = twin.counts();
    /// assert_eq!(twin.control_point(0, 3), twin.control_point(n - 1, 3));
    /// assert_eq!(twin.control_point(0, m - 1), twin.control_point(4, m - 1));
    /// let [du, dv] = twin.domain();
    /// let north = twin.eval(du.lerp(0.3), dv.hi()).point;
    /// assert!((north - Point3::new(0.0, 0.0, 2.0)).norm() < 1e-14);
    /// ```
    pub fn to_nurbs(&self, bounds: [Interval; 2]) -> Result<NurbsSurface, GeomError> {
        let [u, v] = bounds;
        let kind = GeomKind::Surface(self.kind());
        let fault = |reason: String| GeomError::Degenerate { kind, reason };
        if ![u, v].iter().all(|r| r.is_bounded()) {
            return Err(fault("the parameter bounds are not finite".into()));
        }
        let radius = |what: &str, r: f64| {
            if r.is_finite() && r > 0.0 {
                Ok(r)
            } else {
                Err(fault(format!("{what} {r} is not finite and positive")))
            }
        };
        let line = |from: Point3, to: Point3, range: Interval| {
            NurbsCurve::new(
                1,
                vec![range.lo(), range.lo(), range.hi(), range.hi()],
                vec![from, to],
                vec![1.0; 2],
            )
        };
        match self {
            Surface::Nurbs(s) => Ok(s.clone()),
            Surface::Plane { .. } => {
                let end = |a: f64, b: f64| self.point(a, b);
                NurbsSurface::new(
                    [1, 1],
                    [
                        vec![u.lo(), u.lo(), u.hi(), u.hi()],
                        vec![v.lo(), v.lo(), v.hi(), v.hi()],
                    ],
                    vec![
                        end(u.lo(), v.lo()),
                        end(u.lo(), v.hi()),
                        end(u.hi(), v.lo()),
                        end(u.hi(), v.hi()),
                    ],
                    vec![1.0; 4],
                )
            }
            &Surface::Cylinder {
                ref frame,
                radius: r,
            } => {
                let r = radius("radius", r)?;
                let section = NurbsCurve::circle(frame, r, u)?;
                NurbsSurface::extrusion(&section, frame.z().into_inner(), v)
            }
            &Surface::EllipticCylinder {
                ref frame,
                major_radius,
                minor_radius,
            } => {
                let a = radius("major radius", major_radius)?;
                let b = radius("minor radius", minor_radius)?;
                let section = NurbsCurve::ellipse(frame, a, b, u)?;
                NurbsSurface::extrusion(&section, frame.z().into_inner(), v)
            }
            &Surface::Cone {
                ref frame,
                radius: r,
                half_angle,
            } => {
                let r = radius("radius", r)?;
                let (sa, ca) = half_angle.sin_cos();
                let ruling = |t: f64| {
                    frame.origin()
                        + (r + t * sa) * frame.x().into_inner()
                        + (t * ca) * frame.z().into_inner()
                };
                let generatrix = line(ruling(v.lo()), ruling(v.hi()), v)?;
                NurbsSurface::revolution(&generatrix, frame.origin(), frame.z(), u)
            }
            &Surface::Sphere {
                ref frame,
                radius: r,
            } => {
                let r = radius("radius", r)?;
                let meridian = NurbsCurve::ellipse(
                    &Frame::from_orthonormal(
                        frame.origin(),
                        frame.x().into_inner(),
                        frame.z().into_inner(),
                        -frame.y().into_inner(),
                    )
                    .map_err(|e| fault(e.to_string()))?,
                    r,
                    r,
                    v,
                )?;
                NurbsSurface::revolution(&meridian, frame.origin(), frame.z(), u)
            }
            &Surface::Torus {
                ref frame,
                major_radius,
                minor_radius,
            } => {
                let big = radius("major radius", major_radius)?;
                let small = radius("minor radius", minor_radius)?;
                let tube = NurbsCurve::circle(
                    &Frame::from_orthonormal(
                        frame.origin() + big * frame.x().into_inner(),
                        frame.x().into_inner(),
                        frame.z().into_inner(),
                        -frame.y().into_inner(),
                    )
                    .map_err(|e| fault(e.to_string()))?,
                    small,
                    v,
                )?;
                NurbsSurface::revolution(&tube, frame.origin(), frame.z(), u)
            }
        }
    }

    /// The surface normal `∂P/∂u × ∂P/∂v` normalised (plane `Z`; cylinder,
    /// cone and sphere radially outward; an elliptic cylinder outward
    /// along the section's own normal `b cos u·X + a sin u·Y`; torus
    /// outward from the tube), or
    /// `None` where the parametrisation is singular: a cone's apex, a
    /// sphere's poles, any surface whose radius is zero, and a NURBS point
    /// where the two derivatives are parallel or vanish. Singular means
    /// the radial scale factor is zero to rounding
    /// ([`arris_math::is_negligible`]), so `v = π/2` in `f64` is the pole
    /// even though `cos(π/2)` is not exactly `0`. Never a direction made
    /// of rounding noise.
    pub fn normal(&self, u: f64, v: f64) -> Option<UnitVec3> {
        let singular = match *self {
            Surface::Nurbs(ref s) => return s.normal(u, v),
            Surface::Plane { .. } => false,
            Surface::Cylinder { radius, .. } => radius == 0.0,
            Surface::EllipticCylinder {
                major_radius,
                minor_radius,
                ..
            } => major_radius == 0.0 || minor_radius == 0.0,
            Surface::Cone {
                radius, half_angle, ..
            } => {
                let along = v * half_angle.sin();
                is_negligible(radius + along, radius.abs().max(along.abs()))
            }
            Surface::Sphere { radius, .. } => is_negligible(radius * v.cos(), radius),
            Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } => {
                let tube = minor_radius * v.cos();
                minor_radius == 0.0
                    || is_negligible(major_radius + tube, major_radius.abs().max(tube.abs()))
            }
        };
        if singular {
            return None;
        }
        let e = self.eval(u, v);
        UnitVec3::try_new(e.du.cross(&e.dv), 0.0)
    }

    /// The normal curvature at `(u, v)` along the tangent `direction`: the
    /// second fundamental form over the first, `II(w) / I(w)`, from
    /// [`Surface::eval`]'s derivatives with `direction` resolved onto
    /// `∂P/∂u` and `∂P/∂v`. The sign is taken against [`Surface::normal`]:
    /// positive where the surface bends toward it, so a cylinder, a sphere
    /// or a torus, whose normal points outward, is negative across its
    /// curvature — `−1/R` across a cylinder's rulings and `0` along them.
    /// The value is independent of `direction`'s length and sense. `None`
    /// where `normal` is `None`, and where `direction` is zero, not finite
    /// or leaves the tangent plane by more than `tol.angular`.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Precision, Vec3};
    /// use core::f64::consts::FRAC_PI_2;
    ///
    /// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let tol = Precision::DEFAULT.tolerance();
    /// // At u = π/2 the wall's normal is +y: across the ruling the wall
    /// // bends away from it, along the ruling not at all.
    /// let across = wall.normal_curvature(FRAC_PI_2, 0.0, Vec3::x(), tol).unwrap();
    /// assert!((across + 0.5).abs() < 1e-15);
    /// assert_eq!(wall.normal_curvature(FRAC_PI_2, 0.0, Vec3::z(), tol), Some(0.0));
    /// // The normal is no tangent.
    /// assert_eq!(wall.normal_curvature(FRAC_PI_2, 0.0, Vec3::y(), tol), None);
    /// ```
    pub fn normal_curvature(&self, u: f64, v: f64, direction: Vec3, tol: Tolerance) -> Option<f64> {
        let n = self.normal(u, v)?.into_inner();
        let length = direction.norm();
        if length <= 0.0 || !length.is_finite() {
            return None;
        }
        if direction.dot(&n).abs() > tol.angular.sin() * length {
            return None;
        }
        let e = self.eval(u, v);
        let (ee, ff, gg) = (e.du.dot(&e.du), e.du.dot(&e.dv), e.dv.dot(&e.dv));
        let det = ee * gg - ff * ff;
        if det <= 0.0 || !det.is_finite() {
            return None;
        }
        // `direction = a ∂P/∂u + b ∂P/∂v` in the tangent plane.
        let (wu, wv) = (direction.dot(&e.du), direction.dot(&e.dv));
        let a = (gg * wu - ff * wv) / det;
        let b = (ee * wv - ff * wu) / det;
        let first = a * a * ee + 2.0 * a * b * ff + b * b * gg;
        if first <= 0.0 || !first.is_finite() {
            return None;
        }
        let second = a * a * e.duu.dot(&n) + 2.0 * a * b * e.duv.dot(&n) + b * b * e.dvv.dot(&n);
        Some(second / first)
    }

    /// The parametric domain `[u, v]`: the closed fundamental interval
    /// `[0, 2π]` of a periodic direction, `[−π/2, π/2]` for the sphere's
    /// `v`, [`Interval::REAL`] where the table says ℝ, the knot ranges of
    /// a NURBS.
    pub fn domain(&self) -> [Interval; 2] {
        match self {
            Surface::Plane { .. } => [Interval::REAL, Interval::REAL],
            Surface::Cylinder { .. } | Surface::EllipticCylinder { .. } | Surface::Cone { .. } => {
                [Interval::TURN, Interval::REAL]
            }
            Surface::Sphere { .. } => [Interval::TURN, latitude()],
            Surface::Torus { .. } => [Interval::TURN, Interval::TURN],
            Surface::Nurbs(s) => s.domain(),
        }
    }

    /// The period of each parameter, `None` where it is not periodic: the
    /// length after which `eval` repeats. A `Nurbs` direction's is its
    /// [`crate::NurbsSurface::closure`] — its knots' period, or its
    /// domain's length where it is closed without periodic knots, as a
    /// full turn of an exact revolution is.
    pub fn period(&self) -> [Option<f64>; 2] {
        match self {
            Surface::Plane { .. } => [None, None],
            Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. } => [Some(TAU), None],
            Surface::Torus { .. } => [Some(TAU), Some(TAU)],
            Surface::Nurbs(s) => s.closure(),
        }
    }

    /// The singular points of the parametrisation, where a whole row of
    /// one parameter names one point: a cone's apex, a sphere's two
    /// poles, a NURBS surface's collapsed rows (boundary rows of control
    /// points that are one point to rounding), and none on the others.
    /// They are the points [`crate::pcurve_on`] ends a pcurve on, at the
    /// row's own value, and where a loop's walk in (u, v) runs along the
    /// row on a degenerate edge.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Point3};
    /// use core::f64::consts::FRAC_PI_2;
    ///
    /// let sphere = Surface::Sphere { frame: Frame::world(), radius: 2.0 };
    /// let poles = sphere.singularities();
    /// assert_eq!(poles.len(), 2);
    /// assert_eq!((poles[0].fixed, poles[0].value), (1, FRAC_PI_2));
    /// assert!((poles[0].point - Point3::new(0.0, 0.0, 2.0)).norm() < 1e-15);
    /// ```
    pub fn singularities(&self) -> Vec<Singularity> {
        match *self {
            Surface::Cone {
                ref frame,
                radius,
                half_angle,
            } => {
                let (sin, cos) = half_angle.sin_cos();
                let v = -radius / sin;
                vec![Singularity {
                    point: frame.origin() + v * cos * frame.z().into_inner(),
                    fixed: 1,
                    value: v,
                }]
            }
            Surface::Sphere { ref frame, radius } => [1.0, -1.0]
                .into_iter()
                .map(|side| Singularity {
                    point: frame.origin() + side * radius * frame.z().into_inner(),
                    fixed: 1,
                    value: side * FRAC_PI_2,
                })
                .collect(),
            Surface::Nurbs(ref nurbs) => (nurbs.collapsed_rows().into_iter())
                .map(|(fixed, value, point)| Singularity {
                    point,
                    fixed,
                    value,
                })
                .collect(),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Torus { .. } => Vec::new(),
        }
    }

    /// The largest parameter steps `[hu, hv]` for which a triangle whose
    /// corners lie on the surface, at parameters at most `hu` apart in
    /// `u` and `hv` apart in `v` within `bounds`, deviates from the
    /// surface by at most `chord`. The bound is the second fundamental
    /// form: a chord over a parameter step `h` in a direction of normal
    /// curvature `k` leaves the surface by at most `k h² / 8`, and a
    /// triangle by the sum over its two directions, so where both
    /// directions curve each gets half the chord. `f64::INFINITY` along a
    /// direction the surface is flat or ruled in — both on a plane, `v`
    /// on a cylinder, an elliptic cylinder and a cone, whose triangles
    /// are then bounded by the `u` step alone whatever their height; the
    /// cone's `u` curvature is taken at the radius of the region's far
    /// `v` bound, and an elliptic cylinder's `u` step is bounded by its
    /// major radius, the largest its second derivative gets. A torus takes
    /// `R + r` in `u` and `r` in `v`, a sphere its radius in both; a
    /// NURBS samples the form's three coefficients on a grid over
    /// `bounds` clipped to its domain and gives one step to both
    /// directions. A `chord` of zero gives zero steps; a cone whose `v`
    /// bound is unbounded gives a zero `u` step, since no step fits.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Interval};
    ///
    /// let cyl = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let [hu, hv] = cyl.chord_steps(1e-3, cyl.domain());
    /// assert!((hu - (8.0 * 1e-3 / 2.0f64).sqrt()).abs() < 1e-15);
    /// assert_eq!(hv, f64::INFINITY);
    /// ```
    pub fn chord_steps(&self, chord: f64, bounds: [Interval; 2]) -> [f64; 2] {
        // The step for a normal curvature bound `k` and a chord share.
        let step = |k: f64, share: f64| {
            if k == 0.0 {
                f64::INFINITY
            } else if k.is_finite() {
                (8.0 * share / k).sqrt()
            } else {
                0.0
            }
        };
        match *self {
            Surface::Plane { .. } => [f64::INFINITY; 2],
            Surface::Cylinder { radius, .. } => [step(radius.abs(), chord), f64::INFINITY],
            Surface::EllipticCylinder { major_radius, .. } => {
                [step(major_radius.abs(), chord), f64::INFINITY]
            }
            Surface::Cone {
                radius, half_angle, ..
            } => {
                let (sa, ca) = half_angle.sin_cos();
                let v = bounds[1];
                let rho = if v.is_bounded() {
                    (radius + v.lo() * sa)
                        .abs()
                        .max((radius + v.hi() * sa).abs())
                } else {
                    f64::INFINITY
                };
                [step(rho * ca, chord), f64::INFINITY]
            }
            Surface::Sphere { radius, .. } => [step(radius.abs(), chord / 2.0); 2],
            Surface::Torus {
                major_radius,
                minor_radius,
                ..
            } => [
                step((major_radius + minor_radius).abs(), chord / 2.0),
                step(minor_radius.abs(), chord / 2.0),
            ],
            Surface::Nurbs(ref s) => {
                let domain = s.domain();
                let clip = |b: Interval, d: Interval| b.intersection(&d).unwrap_or(d);
                let (bu, bv) = (clip(bounds[0], domain[0]), clip(bounds[1], domain[1]));
                let n = NURBS_FORM_SAMPLES;
                let mut k = [0.0f64; 3];
                for i in 0..=n {
                    for j in 0..=n {
                        let (u, v) = (bu.lerp(i as f64 / n as f64), bv.lerp(j as f64 / n as f64));
                        let Some(normal) = s.normal(u, v) else {
                            continue;
                        };
                        let e = s.eval(u, v);
                        k[0] = k[0].max(e.duu.dot(&normal).abs());
                        k[1] = k[1].max(e.duv.dot(&normal).abs());
                        k[2] = k[2].max(e.dvv.dot(&normal).abs());
                    }
                }
                let h = step(k[0] + 2.0 * k[1] + k[2], chord);
                [h, h]
            }
        }
    }

    /// The parameter steps `[hu, hv]` a triangle standing on a curve whose
    /// (u, v) points all lie inside `band` needs, a function of the
    /// surface and the band alone: the same as [`Surface::chord_steps`]
    /// over `band` for every kind but a NURBS surface, which is read
    /// over its whole domain instead, never finer than any sub-box's
    /// steps. A cone's `u` step is therefore the one at the radii the
    /// band reaches, so an edge's samples do not depend on how far its
    /// face's region extends beyond it (ADR-0052).
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::{Frame, Interval};
    ///
    /// let cone = Surface::Cone {
    ///     frame: Frame::world(),
    ///     radius: 1.0,
    ///     half_angle: 0.5,
    /// };
    /// let near = Interval::new(0.0, 1.0).unwrap();
    /// let far = Interval::new(0.0, 10.0).unwrap();
    /// let u = Interval::new(0.0, 1.0).unwrap();
    /// // The wider the band's radii, the finer the step.
    /// assert!(cone.chord_steps_along(1e-3, [u, far])[0] < cone.chord_steps_along(1e-3, [u, near])[0]);
    /// ```
    pub fn chord_steps_along(&self, chord: f64, band: [Interval; 2]) -> [f64; 2] {
        match self {
            Surface::Nurbs(s) => self.chord_steps(chord, s.domain()),
            _ => self.chord_steps(chord, band),
        }
    }

    /// The same surface with its frame moved by `motion`: the
    /// parametrisation is carried along, so `moved.eval(u, v).point ==
    /// motion.apply(self.eval(u, v).point)` to rounding.
    pub fn transformed(&self, motion: &Isometry) -> Surface {
        match self {
            Surface::Nurbs(s) => Surface::Nurbs(s.transformed(motion)),
            &Surface::Plane { frame } => Surface::Plane {
                frame: frame.transformed(motion),
            },
            &Surface::Cylinder { frame, radius } => Surface::Cylinder {
                frame: frame.transformed(motion),
                radius,
            },
            &Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            } => Surface::EllipticCylinder {
                frame: frame.transformed(motion),
                major_radius,
                minor_radius,
            },
            &Surface::Cone {
                frame,
                radius,
                half_angle,
            } => Surface::Cone {
                frame: frame.transformed(motion),
                radius,
                half_angle,
            },
            &Surface::Sphere { frame, radius } => Surface::Sphere {
                frame: frame.transformed(motion),
                radius,
            },
            &Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => Surface::Torus {
                frame: frame.transformed(motion),
                major_radius,
                minor_radius,
            },
        }
    }

    /// The surface moved by `distance` along its own [`Surface::normal`]
    /// (positive: the way the normal points), as a surface of the same
    /// kind: a plane's parallel plane, a cylinder's coaxial cylinder of
    /// radius `R + d`, a cone's coaxial cone of the same half-angle with
    /// radius `R + d / cos α` at `v = 0`, a sphere's concentric sphere of
    /// radius `R + d` and a torus of the same major radius and minor
    /// radius `r + d`. Every parametrisation keeps its frame, so `u` is
    /// unchanged and a seam stays where it was; a plane's offset keeps
    /// its `(u, v)` too, and a cone's `v` moves by `−d tan α`.
    ///
    /// A cone is offset on the nappe its normal is outward on, where
    /// `R + v sin α > 0`, and the offset's own nappe is the one where its
    /// radius is positive: a point whose move along the normal crosses the
    /// axis (`R + v sin α + d cos α ≤ 0`) is on the offset's other nappe,
    /// and whether a face reaches the axis or the apex it moves is the
    /// face's to decide, not the surface's.
    ///
    /// `None` where no surface of the kind is the offset: a non-finite
    /// `distance`, an elliptic cylinder (the parallel curve of an ellipse
    /// is no ellipse), a NURBS surface (no closed form; the NURBS cycle's),
    /// and a radius driven to or through zero — a cylinder or a sphere
    /// whose new radius is not positive beyond rounding
    /// ([`arris_math::is_negligible`]), a torus whose new minor radius is
    /// not, or reaches its major radius.
    ///
    /// ```
    /// use arris_geom::Surface;
    /// use arris_math::Frame;
    ///
    /// let hole = Surface::Cylinder { frame: Frame::world(), radius: 3.0 };
    /// // The normal points out from the axis: a positive distance widens it.
    /// let wider = hole.offset(1.0).unwrap();
    /// assert!(matches!(wider, Surface::Cylinder { radius, .. } if radius == 4.0));
    /// assert!(hole.offset(-3.0).is_none());
    /// ```
    pub fn offset(&self, distance: f64) -> Option<Surface> {
        if !distance.is_finite() {
            return None;
        }
        // A radius `scale` was driven to `moved`: gone to rounding.
        let collapsed = |moved: f64, scale: f64| {
            moved <= 0.0 || is_negligible(moved, scale.abs().max(distance.abs()))
        };
        match *self {
            Surface::Nurbs(_) | Surface::EllipticCylinder { .. } => None,
            Surface::Plane { frame } => {
                let by = distance * frame.z().into_inner();
                let frame = Frame::from_orthonormal(
                    frame.origin() + by,
                    frame.x().into_inner(),
                    frame.y().into_inner(),
                    frame.z().into_inner(),
                )
                .ok()?;
                Some(Surface::Plane { frame })
            }
            Surface::Cylinder { frame, radius } => {
                let moved = radius + distance;
                (!collapsed(moved, radius)).then_some(Surface::Cylinder {
                    frame,
                    radius: moved,
                })
            }
            Surface::Cone {
                frame,
                radius,
                half_angle,
            } => Some(Surface::Cone {
                frame,
                radius: radius + distance / half_angle.cos(),
                half_angle,
            }),
            Surface::Sphere { frame, radius } => {
                let moved = radius + distance;
                (!collapsed(moved, radius)).then_some(Surface::Sphere {
                    frame,
                    radius: moved,
                })
            }
            Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => {
                let moved = minor_radius + distance;
                let reaches = moved >= major_radius
                    || is_negligible(major_radius - moved, major_radius.abs().max(moved.abs()));
                (!collapsed(moved, minor_radius) && !reaches).then_some(Surface::Torus {
                    frame,
                    major_radius,
                    minor_radius: moved,
                })
            }
        }
    }

    /// The mirror image of the surface and the parameter map that
    /// relates them: `image.eval(m(u, v)).point == plane.apply(self.eval(u,
    /// v).point)` to rounding, with `m` the returned [`ParamMap`]. Frames
    /// stay right-handed; a quadric's `u` is reflected, and a plane or a
    /// NURBS surface keeps its parameters while its normal turns against
    /// the image of the original's (ADR-0031 §2).
    ///
    /// ```
    /// use arris_geom::{ParamMap, Surface};
    /// use arris_math::{Frame, Point3, Reflection, Vec3};
    ///
    /// let s = Surface::Cylinder { frame: Frame::world(), radius: 1.0 };
    /// let r = Reflection::new(Point3::new(2.0, 0.0, 0.0), Vec3::x()).unwrap();
    /// let (image, map) = s.mirrored(&r);
    /// assert_eq!(map, ParamMap::ReflectU);
    /// let (u, v) = map.apply(0.5, 1.0);
    /// assert!((image.eval(u, v).point - r.apply(s.eval(0.5, 1.0).point)).norm() < 1e-14);
    /// ```
    pub fn mirrored(&self, plane: &Reflection) -> (Surface, ParamMap) {
        match self {
            Surface::Nurbs(s) => (Surface::Nurbs(s.mirrored(plane)), ParamMap::Identity),
            &Surface::Plane { frame } => (
                Surface::Plane {
                    frame: plane.apply_frame_reversed(&frame),
                },
                ParamMap::Identity,
            ),
            &Surface::Cylinder { frame, radius } => (
                Surface::Cylinder {
                    frame: plane.apply_frame(&frame),
                    radius,
                },
                ParamMap::ReflectU,
            ),
            &Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            } => (
                Surface::EllipticCylinder {
                    frame: plane.apply_frame(&frame),
                    major_radius,
                    minor_radius,
                },
                ParamMap::ReflectU,
            ),
            &Surface::Cone {
                frame,
                radius,
                half_angle,
            } => (
                Surface::Cone {
                    frame: plane.apply_frame(&frame),
                    radius,
                    half_angle,
                },
                ParamMap::ReflectU,
            ),
            &Surface::Sphere { frame, radius } => (
                Surface::Sphere {
                    frame: plane.apply_frame(&frame),
                    radius,
                },
                ParamMap::ReflectU,
            ),
            &Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => (
                Surface::Torus {
                    frame: plane.apply_frame(&frame),
                    major_radius,
                    minor_radius,
                },
                ParamMap::ReflectU,
            ),
        }
    }
}

/// How a surface's parameters change under a mirror: the image of
/// `S(u, v)` is `R·S(m(u, v))` for this `m` (ADR-0031 §2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParamMap {
    /// `(u, v) ↦ (u, v)`: a plane's or a NURBS surface's. Its normal
    /// turns against the image of the original's.
    Identity,
    /// `(u, v) ↦ (2π − u, v)`: a quadric's. Its normal is the image of
    /// the original's.
    ReflectU,
}

impl ParamMap {
    /// The image of `(u, v)`.
    ///
    /// ```
    /// use arris_geom::ParamMap;
    /// assert_eq!(ParamMap::Identity.apply(1.0, 2.0), (1.0, 2.0));
    /// assert_eq!(ParamMap::ReflectU.apply(1.0, 2.0), (core::f64::consts::TAU - 1.0, 2.0));
    /// ```
    pub fn apply(self, u: f64, v: f64) -> (f64, f64) {
        match self {
            ParamMap::Identity => (u, v),
            ParamMap::ReflectU => (TAU - u, v),
        }
    }
}

/// Grid points per direction at which a NURBS surface's second
/// fundamental form is sampled for [`Surface::chord_steps`]: a sampling
/// density, not a tolerance, chosen so a bicubic patch's curvature cannot
/// hide between samples over one knot span.
const NURBS_FORM_SAMPLES: usize = 32;

/// A frame's origin and axes as plain vectors.
fn axes(f: &Frame) -> (Point3, Vec3, Vec3, Vec3) {
    (
        f.origin(),
        f.x().into_inner(),
        f.y().into_inner(),
        f.z().into_inner(),
    )
}

/// `[−π/2, π/2]`: the sphere's `v`. Constants in order and not NaN, so the
/// fallback is unreachable.
fn latitude() -> Interval {
    Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap_or(Interval::UNIT)
}

/// The frame's origin and axes at coordinate `k`.
fn axes3(frame: &Frame, k: usize) -> (f64, f64, f64, f64) {
    (frame.origin()[k], frame.x()[k], frame.y()[k], frame.z()[k])
}

/// The box whose interval on axis `k` is `span(k)`.
fn axis_bounds(span: impl Fn(usize) -> [f64; 2]) -> Aabb {
    let mut min = [0.0; 3];
    let mut max = [0.0; 3];
    for (k, (lo, hi)) in min.iter_mut().zip(max.iter_mut()).enumerate() {
        let [a, b] = span(k);
        (*lo, *hi) = (a, b);
    }
    Aabb { min, max }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn domains_and_periods_follow_the_table() {
        let f = Frame::world();
        let plane = Surface::Plane { frame: f };
        assert_eq!(plane.domain(), [Interval::REAL, Interval::REAL]);
        assert_eq!(plane.period(), [None, None]);
        let cyl = Surface::Cylinder {
            frame: f,
            radius: 1.0,
        };
        assert_eq!(cyl.domain(), [Interval::TURN, Interval::REAL]);
        assert_eq!(cyl.period(), [Some(TAU), None]);
        let elliptic = Surface::EllipticCylinder {
            frame: f,
            major_radius: 2.0,
            minor_radius: 1.0,
        };
        assert_eq!(elliptic.domain(), [Interval::TURN, Interval::REAL]);
        assert_eq!(elliptic.period(), [Some(TAU), None]);
        assert_eq!(elliptic.kind().to_string(), "elliptic cylinder");
        let sphere = Surface::Sphere {
            frame: f,
            radius: 1.0,
        };
        assert_eq!(sphere.domain()[1].lo(), -FRAC_PI_2);
        assert_eq!(sphere.period(), [Some(TAU), None]);
        let torus = Surface::Torus {
            frame: f,
            major_radius: 2.0,
            minor_radius: 1.0,
        };
        assert_eq!(torus.domain(), [Interval::TURN, Interval::TURN]);
        assert_eq!(torus.period(), [Some(TAU), Some(TAU)]);
        assert_eq!(torus.kind(), SurfaceKind::Torus);
        assert_eq!(torus.kind().to_string(), "torus");
    }

    #[test]
    fn a_zero_radius_has_no_normal() {
        let f = Frame::world();
        assert!(
            Surface::Cylinder {
                frame: f,
                radius: 0.0
            }
            .normal(1.0, 1.0)
            .is_none()
        );
        assert!(
            Surface::Sphere {
                frame: f,
                radius: 0.0
            }
            .normal(1.0, 1.0)
            .is_none()
        );
        assert!(
            Surface::Torus {
                frame: f,
                major_radius: 1.0,
                minor_radius: 0.0
            }
            .normal(1.0, 1.0)
            .is_none()
        );
        assert!(Surface::Plane { frame: f }.normal(1.0, 1.0).is_some());
        assert!(
            Surface::EllipticCylinder {
                frame: f,
                major_radius: 2.0,
                minor_radius: 0.0
            }
            .normal(1.0, 1.0)
            .is_none()
        );
    }

    #[test]
    fn an_elliptic_cylinder_evaluates_its_section_along_its_axis() {
        let s = Surface::EllipticCylinder {
            frame: Frame::world(),
            major_radius: 3.0,
            minor_radius: 2.0,
        };
        let e = s.eval(FRAC_PI_2, 5.0);
        assert!((e.point - Point3::new(0.0, 2.0, 5.0)).norm() < 1e-15);
        assert!((e.du - Vec3::new(-3.0, 0.0, 0.0)).norm() < 1e-15);
        assert_eq!(e.dv, Vec3::z());
        assert!((e.duu - Vec3::new(0.0, -2.0, 0.0)).norm() < 1e-15);
        // The normal is the section's own, not the radial direction: at
        // u = π/4 the point is (3, 2)/√2 and the normal ∝ (2, 3).
        let n = s.normal(core::f64::consts::FRAC_PI_4, 0.0).unwrap();
        let expected = Vec3::new(2.0, 3.0, 0.0).normalize();
        assert!((n.into_inner() - expected).norm() < 1e-15, "{n:?}");
        // Bounds: the box of the section swept along z.
        let b = s
            .bounds([Interval::TURN, Interval::new(-1.0, 4.0).unwrap()])
            .unwrap();
        assert!((b.min[0] + 3.0).abs() < 1e-15 && (b.max[1] - 2.0).abs() < 1e-15);
        assert_eq!((b.min[2], b.max[2]), (-1.0, 4.0));
        // Ruled along v, bounded by the major radius in u.
        let [hu, hv] = s.chord_steps(1e-3, s.domain());
        assert!((hu - (8.0 * 1e-3 / 3.0f64).sqrt()).abs() < 1e-15);
        assert_eq!(hv, f64::INFINITY);
    }
}
