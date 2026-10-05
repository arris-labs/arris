//! The geometry of the subset mapped onto Arris's variants, exactly
//! (ADR-0025 §1), or refused by name (§2).
//!
//! The analytic entities stay themselves. Every B-spline subtype becomes
//! `Nurbs` with its knots expanded. A `SURFACE_OF_LINEAR_EXTRUSION` of a
//! line is a plane, of a circle or an ellipse a cylinder or an elliptic
//! cylinder with its section recomputed perpendicular to the direction,
//! and of anything else the exact NURBS extrusion. A
//! `SURFACE_OF_REVOLUTION` of a line is a plane, a cylinder or a cone
//! where the line meets the axis or runs beside it, of a circle in a plane
//! through the axis a sphere or a torus, and of anything else the exact
//! NURBS revolution. `PARABOLA`, `HYPERBOLA` and `POLYLINE` become exact
//! NURBS. `TRIMMED_CURVE`, `RECTANGULAR_TRIMMED_SURFACE` and the surface
//! curves become their basis.
//!
//! Two things the file's entity carries survive as flags beside the
//! variant, because the variant cannot hold them:
//!
//! - **the sense of a surface's normal.** A variant's normal is fixed by
//!   its form — a cylinder's points away from its axis — where the
//!   file's is `∂σ/∂u × ∂σ/∂v` of its own parametrisation. A face's
//!   `same_sense` is relative to the file's, so the reader turns it where
//!   the two disagree ([`ReadSurface::reversed`]).
//! - **the sense of a curve**, which a `TRIMMED_CURVE` of sense `.F.`
//!   runs against its basis ([`ReadCurve::reversed`]).
//!
//! Some exact forms are bounded and the entity is not: a parabola, a
//! hyperbola, a surface swept from one or a line revolved skew to the
//! axis. Those are kept as what they are and bounded only once the part
//! they belong to is known ([`Ball`]): every parameter where the entity
//! can be within the part is kept, found from closed-form bounds, so the
//! NURBS form is exact over any face or edge the part can have on it.

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_geom::{Curve, MAX_DEGREE, NurbsCurve, NurbsSurface, Surface};
use arris_math::{Frame, Interval, Point3, UnitVec3, Vec3, is_negligible};

use super::Refusal;
use super::entities::{Args, Entities, describe};
use super::units::Units;
use crate::step::part21::{Instance, Param};

/// How deep a curve or surface may refer to others (a trimmed curve of a
/// surface curve of a trimmed curve…) before the reader stops: a real file
/// nests two or three, and a cycle of references must end.
const GEOMETRY_DEPTH: u8 = 16;

/// Where a part lies: every face and edge of it is inside the ball. What
/// an unbounded entity's exact NURBS form is bounded to.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Ball {
    /// The centre.
    pub(crate) centre: Point3,
    /// The radius, positive.
    pub(crate) radius: f64,
}

/// A curve as read: an Arris curve, or a curve whose exact form needs
/// its range, with its sense against the file entity it was read from.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ReadCurve {
    /// The entity it was read from.
    pub(crate) id: u64,
    /// What it is.
    pub(crate) form: CurveForm,
    /// It runs against the file entity's own sense: a `TRIMMED_CURVE` of
    /// sense `.F.` on its basis.
    pub(crate) reversed: bool,
    /// A point of the curve inside its trim, where a `TRIMMED_CURVE` of a
    /// line or a conic gives the trim as parameters: which part of the
    /// basis the file means, where that decides something — the half of
    /// a line or a circle turned about an axis it crosses.
    pub(crate) inside: Option<Point3>,
}

/// What a [`ReadCurve`] is.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum CurveForm {
    /// A curve Arris holds as it is.
    Exact(Curve),
    /// `O + F t²·X + 2F t·Y`, `F > 0`: unbounded.
    Parabola {
        /// `O`, `X`, `Y`.
        frame: Frame,
        /// `F`.
        focal: f64,
    },
    /// `O + a cosh t·X + b sinh t·Y`: unbounded.
    Hyperbola {
        /// `O`, `X`, `Y`.
        frame: Frame,
        /// `a`.
        semi_axis: f64,
        /// `b`.
        semi_imaginary: f64,
    },
}

/// A surface as read, and whether its normal is against the file
/// entity's.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct ReadSurface {
    /// The entity it was read from.
    pub(crate) id: u64,
    /// What it is.
    pub(crate) form: SurfaceForm,
    /// The variant's normal is the opposite of the file entity's, so a
    /// face on it turns its `same_sense`.
    pub(crate) reversed: bool,
}

/// What a [`ReadSurface`] is.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum SurfaceForm {
    /// A surface Arris holds as it is.
    Exact(Surface),
    /// `C(u) + v·d`, bounded once its part is known.
    Extrusion {
        /// `C`, not a line, a circle or an ellipse.
        curve: ReadCurve,
        /// `d`.
        direction: UnitVec3,
    },
    /// `C(v)` turned by `u` about the axis through `origin`, bounded once
    /// its part is known.
    Revolution {
        /// `C`.
        curve: ReadCurve,
        /// A point of the axis.
        origin: Point3,
        /// The axis.
        axis: UnitVec3,
    },
}

/// The geometry of one representation context.
pub(crate) struct Geometry<'a> {
    pub(crate) entities: Entities<'a>,
    pub(crate) units: Units,
}

/// A refusal of `entity` for a value Arris cannot hold.
fn degenerate(entity: u64, what: impl Into<String>) -> Refusal {
    Refusal::Degenerate {
        entity,
        what: what.into(),
    }
}

/// `v` as a unit vector, or a refusal of `entity` naming `what`.
fn unit(entity: u64, v: Vec3, what: &str) -> Result<UnitVec3, Refusal> {
    let n = v.norm();
    if !(n.is_finite() && n > 0.0) {
        return Err(degenerate(entity, format!("{what} has no direction")));
    }
    Ok(UnitVec3::new_unchecked(v / n))
}

/// The refusal of an entity outside the subset, by its name (ADR-0025
/// §2): the named families, or any other as unsupported.
fn refused(id: u64, instance: &Instance) -> Refusal {
    let name = describe(instance);
    let has = |n: &str| instance.record(n).is_some();
    if has("OFFSET_SURFACE") || has("OFFSET_CURVE_3D") || has("OFFSET_CURVE_2D") {
        Refusal::Offset { entity: id, name }
    } else if has("COMPOSITE_CURVE")
        || has("COMPOSITE_CURVE_ON_SURFACE")
        || has("BOUNDARY_CURVE")
        || has("OUTER_BOUNDARY_CURVE")
        || has("RECTANGULAR_COMPOSITE_SURFACE")
    {
        Refusal::Composite { entity: id, name }
    } else if has("CURVE_BOUNDED_SURFACE") {
        Refusal::CurveBounded { entity: id, name }
    } else if has("DEGENERATE_TOROIDAL_SURFACE") {
        Refusal::DegenerateTorus { entity: id }
    } else {
        Refusal::Unsupported { entity: id, name }
    }
}

impl Geometry<'_> {
    fn deeper(&self, id: u64, depth: u8) -> Result<u8, Refusal> {
        if depth >= GEOMETRY_DEPTH {
            return Err(super::entities::malformed(
                id,
                "geometry nested too deep to follow",
            ));
        }
        Ok(depth + 1)
    }

    /// A positive length parameter, converted.
    fn positive(&self, args: &Args<'_>, i: usize, what: &str) -> Result<f64, Refusal> {
        let x = self.units.length(args.real(i)?);
        if !(x.is_finite() && x > 0.0) {
            return Err(degenerate(args.id, format!("{what} of {x}")));
        }
        Ok(x)
    }

    /// The 3D curve `id`, referenced from `from`.
    pub(crate) fn curve(&self, from: u64, id: u64) -> Result<ReadCurve, Refusal> {
        self.curve_at(from, id, 0)
    }

    fn curve_at(&self, from: u64, id: u64, depth: u8) -> Result<ReadCurve, Refusal> {
        let depth = self.deeper(id, depth)?;
        let instance = self.entities.get(from, id)?;
        let exact = |curve: Curve| ReadCurve {
            id,
            form: CurveForm::Exact(curve),
            reversed: false,
            inside: None,
        };
        let args = |name: &str| self.entities.record(from, id, name);
        if instance.record("B_SPLINE_CURVE").is_some()
            || instance.record("B_SPLINE_CURVE_WITH_KNOTS").is_some()
            || instance.record("UNIFORM_CURVE").is_some()
            || instance.record("QUASI_UNIFORM_CURVE").is_some()
            || instance.record("BEZIER_CURVE").is_some()
        {
            return Ok(exact(Curve::Nurbs(self.bspline_curve(id, instance)?)));
        }
        let Instance::Simple(record) = instance else {
            return Err(refused(id, instance));
        };
        match record.name.as_str() {
            "LINE" => {
                let a = args("LINE")?;
                let origin = self.units.point(&self.entities, id, a.reference(1)?)?;
                let direction = self.vector(id, a.reference(2)?)?;
                Ok(exact(Curve::Line { origin, direction }))
            }
            "CIRCLE" => {
                let a = args("CIRCLE")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let radius = self.positive(&a, 2, "a radius")?;
                Ok(exact(Curve::Circle { frame, radius }))
            }
            "ELLIPSE" => {
                let a = args("ELLIPSE")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let first = self.positive(&a, 2, "a semi-axis")?;
                let second = self.positive(&a, 3, "a semi-axis")?;
                Ok(exact(ellipse(frame, first, second)))
            }
            "PARABOLA" => {
                let a = args("PARABOLA")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let focal = self.units.length(a.real(2)?);
                if !(focal.is_finite() && focal != 0.0) {
                    return Err(degenerate(id, format!("a focal distance of {focal}")));
                }
                // A negative focal distance is the parabola turned half a
                // turn about its axis `Z`, at the same parameter.
                let frame = if focal < 0.0 {
                    turned(&frame, PI)
                } else {
                    frame
                };
                Ok(ReadCurve {
                    id,
                    form: CurveForm::Parabola {
                        frame,
                        focal: focal.abs(),
                    },
                    reversed: false,
                    inside: None,
                })
            }
            "HYPERBOLA" => {
                let a = args("HYPERBOLA")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let semi_axis = self.positive(&a, 2, "a semi-axis")?;
                let semi_imaginary = self.positive(&a, 3, "a semi-imaginary axis")?;
                Ok(ReadCurve {
                    id,
                    form: CurveForm::Hyperbola {
                        frame,
                        semi_axis,
                        semi_imaginary,
                    },
                    reversed: false,
                    inside: None,
                })
            }
            "POLYLINE" => {
                let a = args("POLYLINE")?;
                let mut points: Vec<Point3> = Vec::new();
                for p in a.references(1)? {
                    let p = self.units.point(&self.entities, id, p)?;
                    // A repeated point is a span of no length, which no
                    // parameter could be found on.
                    if points.last() != Some(&p) {
                        points.push(p);
                    }
                }
                if points.len() < 2 {
                    return Err(degenerate(id, "a polyline of fewer than two points"));
                }
                let n = points.len();
                let mut knots = vec![0.0];
                knots.extend((0..n).map(|i| i as f64));
                knots.push((n - 1) as f64);
                let curve = NurbsCurve::new(1, knots, points, vec![1.0; n])
                    .map_err(|e| degenerate(id, e.to_string()))?;
                Ok(exact(Curve::Nurbs(curve)))
            }
            "TRIMMED_CURVE" => {
                // The basis: the edge trims it anyway (ADR-0025 §1). A
                // sense of `.F.` runs the trimmed curve against it.
                let a = args("TRIMMED_CURVE")?;
                let basis_id = a.reference(1)?;
                let mut basis = self.curve_at(id, basis_id, depth)?;
                let sense = a.enumeration(4)? == "T";
                if !sense {
                    basis.reversed = !basis.reversed;
                }
                let trims = (trim_parameter(a.list(2)?), trim_parameter(a.list(3)?));
                if let (Some(t1), Some(t2)) = trims {
                    basis.inside = self.inside(id, basis_id, t1, t2, sense)?;
                }
                Ok(basis)
            }
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" | "BOUNDED_SURFACE_CURVE" => {
                // Its 3D curve: pcurves are rebuilt, never read.
                let a = args(record.name.as_str())?;
                self.curve_at(id, a.reference(1)?, depth)
            }
            _ => Err(refused(id, instance)),
        }
    }

    /// A `VECTOR`'s direction; its magnitude is a parametrisation, which
    /// the variant's own replaces.
    fn vector(&self, from: u64, id: u64) -> Result<UnitVec3, Refusal> {
        let a = self.entities.record(from, id, "VECTOR")?;
        let d = self.units.direction(&self.entities, id, a.reference(1)?)?;
        unit(id, d, "a vector")
    }

    /// The point of the line or conic `basis` in the middle of the trim
    /// from `t1` to `t2` in its own parameter — a length along a line's
    /// vector, an angle on a conic, in the file's units — running forward
    /// on `sense` and backward otherwise. `None` for any other basis.
    fn inside(
        &self,
        from: u64,
        basis: u64,
        t1: f64,
        t2: f64,
        sense: bool,
    ) -> Result<Option<Point3>, Refusal> {
        let Instance::Simple(record) = self.entities.get(from, basis)? else {
            return Ok(None);
        };
        let e = &self.entities;
        match record.name.as_str() {
            "LINE" => {
                let a = e.record(from, basis, "LINE")?;
                let origin = self.units.point(e, basis, a.reference(1)?)?;
                let v = e.record(basis, a.reference(2)?, "VECTOR")?;
                let direction = self.units.direction(e, v.id, v.reference(1)?)?;
                let magnitude = self.units.length(v.real(2)?);
                let direction = unit(v.id, direction, "a vector")?;
                Ok(Some(
                    origin + 0.5 * (t1 + t2) * magnitude * direction.into_inner(),
                ))
            }
            "CIRCLE" | "ELLIPSE" => {
                let a = e.record(from, basis, record.name.as_str())?;
                let frame = self.units.placement(e, basis, a.reference(1)?)?;
                let first = self.units.length(a.real(2)?);
                let second = if record.name == "CIRCLE" {
                    first
                } else {
                    self.units.length(a.real(3)?)
                };
                let (t1, t2) = (self.units.angle(t1), self.units.angle(t2));
                // The arc runs counter-clockwise from `t1` to `t2` on
                // `sense`, and from `t2` to `t1` otherwise, a turn at most.
                let (start, end) = if sense { (t1, t2) } else { (t2, t1) };
                let middle = start + 0.5 * (end - start).rem_euclid(TAU);
                // The file's own formula, whichever semi-axis is longer.
                let (s, c) = middle.sin_cos();
                Ok(Some(
                    frame.origin()
                        + first * c * frame.x().into_inner()
                        + second * s * frame.y().into_inner(),
                ))
            }
            _ => Ok(None),
        }
    }

    /// Any B-spline curve instance, simple or complex, as a NURBS curve.
    fn bspline_curve(&self, id: u64, instance: &Instance) -> Result<NurbsCurve, Refusal> {
        let spline = Spline::of(id, instance, 1)?;
        let [degree] = spline.degrees[..] else {
            return Err(super::entities::malformed(id, "a curve of two degrees"));
        };
        let refs = spline.args.references(spline.first + 1)?;
        let mut points = Vec::with_capacity(refs.len());
        for p in refs {
            points.push(self.units.point(&self.entities, id, p)?);
        }
        let knots = spline.knots(0, degree, points.len())?;
        let weights = match spline.weights()? {
            Some(w) => w,
            None => vec![1.0; points.len()],
        };
        NurbsCurve::new(degree, knots, points, weights).map_err(|e| degenerate(id, e.to_string()))
    }

    /// The surface `id`, referenced from `from`.
    pub(crate) fn surface(&self, from: u64, id: u64) -> Result<ReadSurface, Refusal> {
        self.surface_at(from, id, 0)
    }

    fn surface_at(&self, from: u64, id: u64, depth: u8) -> Result<ReadSurface, Refusal> {
        let depth = self.deeper(id, depth)?;
        let instance = self.entities.get(from, id)?;
        let exact = |surface: Surface| ReadSurface {
            id,
            form: SurfaceForm::Exact(surface),
            reversed: false,
        };
        let args = |name: &str| self.entities.record(from, id, name);
        if instance.record("B_SPLINE_SURFACE").is_some()
            || instance.record("B_SPLINE_SURFACE_WITH_KNOTS").is_some()
            || instance.record("UNIFORM_SURFACE").is_some()
            || instance.record("QUASI_UNIFORM_SURFACE").is_some()
            || instance.record("BEZIER_SURFACE").is_some()
        {
            return Ok(exact(Surface::Nurbs(self.bspline_surface(id, instance)?)));
        }
        let Instance::Simple(record) = instance else {
            return Err(refused(id, instance));
        };
        match record.name.as_str() {
            "PLANE" => {
                let a = args("PLANE")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                Ok(exact(Surface::Plane { frame }))
            }
            "CYLINDRICAL_SURFACE" => {
                let a = args("CYLINDRICAL_SURFACE")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let radius = self.positive(&a, 2, "a radius")?;
                Ok(exact(Surface::Cylinder { frame, radius }))
            }
            "CONICAL_SURFACE" => {
                let a = args("CONICAL_SURFACE")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let radius = self.units.length(a.real(2)?);
                let half_angle = self.units.angle(a.real(3)?);
                if !(radius.is_finite() && radius >= 0.0) {
                    return Err(degenerate(id, format!("a radius of {radius}")));
                }
                if !(half_angle > 0.0 && half_angle < FRAC_PI_2) {
                    return Err(degenerate(id, format!("a semi-angle of {half_angle} rad")));
                }
                Ok(exact(Surface::Cone {
                    frame,
                    radius,
                    half_angle,
                }))
            }
            "SPHERICAL_SURFACE" => {
                let a = args("SPHERICAL_SURFACE")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let radius = self.positive(&a, 2, "a radius")?;
                Ok(exact(Surface::Sphere { frame, radius }))
            }
            "TOROIDAL_SURFACE" => {
                let a = args("TOROIDAL_SURFACE")?;
                let frame = self.units.placement(&self.entities, id, a.reference(1)?)?;
                let major = self.positive(&a, 2, "a major radius")?;
                let minor = self.positive(&a, 3, "a minor radius")?;
                torus(id, frame, major, minor).map(exact)
            }
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                let a = args("SURFACE_OF_LINEAR_EXTRUSION")?;
                let curve = self.curve_at(id, a.reference(1)?, depth)?;
                let direction = self.vector(id, a.reference(2)?)?;
                extrusion(id, curve, direction)
            }
            "SURFACE_OF_REVOLUTION" => {
                let a = args("SURFACE_OF_REVOLUTION")?;
                let curve = self.curve_at(id, a.reference(1)?, depth)?;
                let axis = self
                    .entities
                    .record(id, a.reference(2)?, "AXIS1_PLACEMENT")?;
                let origin = self
                    .units
                    .point(&self.entities, axis.id, axis.reference(1)?)?;
                let direction = match axis.optional_reference(2)? {
                    Some(d) => self.units.direction(&self.entities, axis.id, d)?,
                    None => Vec3::z(),
                };
                let direction = unit(axis.id, direction, "an axis")?;
                revolution(id, curve, origin, direction)
            }
            "RECTANGULAR_TRIMMED_SURFACE" => {
                // The basis: the face trims it anyway (ADR-0025 §1). A
                // sense of `.F.` in one direction turns the normal.
                let a = args("RECTANGULAR_TRIMMED_SURFACE")?;
                let mut basis = self.surface_at(id, a.reference(1)?, depth)?;
                let u_sense = a.enumeration(6)? == "T";
                let v_sense = a.enumeration(7)? == "T";
                if u_sense != v_sense {
                    basis.reversed = !basis.reversed;
                }
                Ok(basis)
            }
            _ => Err(refused(id, instance)),
        }
    }

    /// Any B-spline surface instance, simple or complex, as a NURBS
    /// surface: the file's net is a list of `u` rows, which is Arris's
    /// row-major order over `u`.
    fn bspline_surface(&self, id: u64, instance: &Instance) -> Result<NurbsSurface, Refusal> {
        let spline = Spline::of(id, instance, 2)?;
        let [p, q] = spline.degrees[..] else {
            return Err(super::entities::malformed(id, "a surface of one degree"));
        };
        let rows = spline.args.list(spline.first + 2)?;
        let mut points = Vec::new();
        let mut columns = None;
        for row in rows {
            let Param::List(row) = row else {
                return Err(spline
                    .args
                    .malformed("a control net that is not a list of rows"));
            };
            if *columns.get_or_insert(row.len()) != row.len() {
                return Err(spline
                    .args
                    .malformed("a control net whose rows differ in length"));
            }
            for p in row {
                let Param::Ref(p) = p else {
                    return Err(spline
                        .args
                        .malformed("a control net of something but points"));
                };
                points.push(self.units.point(&self.entities, id, *p)?);
            }
        }
        let (n, m) = (rows.len(), columns.unwrap_or(0));
        let knots = [spline.knots(0, p, n)?, spline.knots(1, q, m)?];
        let weights = match spline.weights()? {
            Some(w) if w.len() == n * m => w,
            Some(_) => return Err(spline.args.malformed("weights that do not match the net")),
            None => vec![1.0; n * m],
        };
        NurbsSurface::new([p, q], knots, points, weights).map_err(|e| degenerate(id, e.to_string()))
    }
}

/// The ellipse of semi-axes `first` along `X` and `second` along `Y`,
/// with the major one along `X` as Arris holds it: a frame turned a
/// quarter about `Z` when `second` is the longer, which moves the
/// parameter by a quarter turn and keeps the sense.
fn ellipse(frame: Frame, first: f64, second: f64) -> Curve {
    if first >= second {
        Curve::Ellipse {
            frame,
            major_radius: first,
            minor_radius: second,
        }
    } else {
        Curve::Ellipse {
            frame: turned(&frame, FRAC_PI_2),
            major_radius: second,
            minor_radius: first,
        }
    }
}

/// The parameter a trim select list gives as `PARAMETER_VALUE(t)`, if
/// it gives one.
fn trim_parameter(list: &[Param]) -> Option<f64> {
    list.iter().find_map(|p| match p {
        Param::Typed(name, inner) if name == "PARAMETER_VALUE" => super::entities::number(inner),
        _ => None,
    })
}

/// `frame` turned by `angle` about its `Z`.
fn turned(frame: &Frame, angle: f64) -> Frame {
    let (s, c) = angle.sin_cos();
    let x = c * frame.x().into_inner() + s * frame.y().into_inner();
    let y = frame.z().into_inner().cross(&x);
    // Orthonormal by construction, so the only failure is non-finite
    // input, which a frame never holds.
    Frame::from_orthonormal(frame.origin(), x, y, frame.z().into_inner()).unwrap_or(*frame)
}

/// A torus Arris holds: `R > r` (data-model §Surfaces), else refused.
fn torus(id: u64, frame: Frame, major: f64, minor: f64) -> Result<Surface, Refusal> {
    if major <= minor {
        return Err(Refusal::SelfIntersectingTorus {
            entity: id,
            major,
            minor,
        });
    }
    Ok(Surface::Torus {
        frame,
        major_radius: major,
        minor_radius: minor,
    })
}

/// Whether `surface`'s normal is against `normal`, the file entity's at
/// `point` on it. Errors: the point has no normal on the surface, or the
/// two are perpendicular — neither happens at a point the callers pick,
/// off every singular row.
fn reversed_at(id: u64, surface: &Surface, point: Point3, normal: Vec3) -> Result<bool, Refusal> {
    let fault = |why: String| degenerate(id, format!("its normal cannot be compared: {why}"));
    let at = surface.project(point).map_err(|e| fault(e.to_string()))?;
    let ours = surface
        .normal(at.uv.x, at.uv.y)
        .ok_or_else(|| fault("a singular point".into()))?;
    let dot = ours.dot(&normal);
    if is_negligible(dot, normal.norm()) {
        return Err(fault("the normals are perpendicular".into()));
    }
    Ok(dot < 0.0)
}

/// A `SURFACE_OF_LINEAR_EXTRUSION` of `curve` along `d`: a plane, a
/// cylinder or an elliptic cylinder where the curve is a line or a conic,
/// else the NURBS extrusion, bounded later.
fn extrusion(id: u64, curve: ReadCurve, d: UnitVec3) -> Result<ReadSurface, Refusal> {
    // The file's normal is `C′ × d` with `C` in its own sense.
    let sense = if curve.reversed { -1.0 } else { 1.0 };
    let exact = |surface: Surface, reversed: bool| ReadSurface {
        id,
        form: SurfaceForm::Exact(surface),
        reversed,
    };
    match &curve.form {
        CurveForm::Exact(Curve::Line { origin, direction }) => {
            let z = direction.cross(&d);
            if is_negligible(z.norm(), 1.0) {
                return Err(degenerate(id, "a line extruded along itself"));
            }
            let frame = Frame::new(*origin, sense * z, direction.into_inner())
                .map_err(|e| degenerate(id, e.to_string()))?;
            Ok(exact(Surface::Plane { frame }, false))
        }
        CurveForm::Exact(Curve::Circle { frame, radius }) => {
            section(id, frame, *radius, *radius, d, sense).map(|(s, r)| exact(s, r))
        }
        CurveForm::Exact(Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        }) => section(id, frame, *major_radius, *minor_radius, d, sense).map(|(s, r)| exact(s, r)),
        CurveForm::Exact(Curve::Nurbs(_))
        | CurveForm::Parabola { .. }
        | CurveForm::Hyperbola { .. } => {
            // The NURBS extrusion runs `u` along the curve's own form, so
            // its normal is the file's unless the file runs the curve
            // against it.
            let reversed = curve.reversed;
            Ok(ReadSurface {
                id,
                form: SurfaceForm::Extrusion {
                    curve,
                    direction: d,
                },
                reversed,
            })
        }
    }
}

/// The cylinder swept by the ellipse `O + a cos t·X + b sin t·Y` of
/// `frame` along `d`, its section taken perpendicular to `d`: the
/// ellipse projected along `d`, whose conjugate semi-diameters `a·X′` and
/// `b·Y′` give its principal axes. A circle where they are equal to
/// rounding. Returns the surface and whether its normal, outward, is
/// against the file's `sense·C′ × d`: the section runs clockwise about
/// `d`.
fn section(
    id: u64,
    frame: &Frame,
    a: f64,
    b: f64,
    d: UnitVec3,
    sense: f64,
) -> Result<(Surface, bool), Refusal> {
    let along = |v: Vec3| v - v.dot(&d) * d.into_inner();
    let p = a * along(frame.x().into_inner());
    let q = b * along(frame.y().into_inner());
    let turn = p.cross(&q).dot(&d);
    if is_negligible(turn, a * b) {
        return Err(degenerate(id, "a conic extruded along its own plane"));
    }
    // The parameter of the major axis: `|p cos t + q sin t|` is largest
    // at `tan 2t = 2 p·q / (|p|² − |q|²)`.
    let t = 0.5 * (2.0 * p.dot(&q)).atan2(p.norm_squared() - q.norm_squared());
    let (s, c) = t.sin_cos();
    let major = c * p + s * q;
    let minor = -s * p + c * q;
    let (major_radius, minor_radius) = (major.norm(), minor.norm());
    let axis = Frame::new(frame.origin(), d.into_inner(), major)
        .map_err(|e| degenerate(id, e.to_string()))?;
    let surface = if is_negligible(major_radius - minor_radius, major_radius) {
        Surface::Cylinder {
            frame: axis,
            radius: major_radius,
        }
    } else {
        Surface::EllipticCylinder {
            frame: axis,
            major_radius,
            minor_radius,
        }
    };
    Ok((surface, sense * turn < 0.0))
}

/// A `SURFACE_OF_REVOLUTION` of `curve` about the axis through `origin`
/// along `axis`: a plane, a cylinder or a cone for a line that meets the
/// axis or runs beside it, a sphere or a torus for a circle in a plane
/// through it, else the NURBS revolution, bounded later.
fn revolution(
    id: u64,
    curve: ReadCurve,
    origin: Point3,
    axis: UnitVec3,
) -> Result<ReadSurface, Refusal> {
    let sense = if curve.reversed { -1.0 } else { 1.0 };
    let a = axis.into_inner();
    // The file's normal at a point `P` of the curve at `u = 0`, where the
    // curve's tangent is `T`: `∂σ/∂u × ∂σ/∂v = (a × (P − A)) × T`.
    let normal = |p: Point3, tangent: Vec3| a.cross(&(p - origin)).cross(&(sense * tangent));
    let exact = |surface: Surface, point: Point3, tangent: Vec3| -> Result<ReadSurface, Refusal> {
        let reversed = reversed_at(id, &surface, point, normal(point, tangent))?;
        Ok(ReadSurface {
            id,
            form: SurfaceForm::Exact(surface),
            reversed,
        })
    };
    // The NURBS revolution runs `v` along the curve's own form, so its
    // normal is the file's unless the file runs the curve against it.
    let nurbs = |curve: ReadCurve| ReadSurface {
        id,
        reversed: curve.reversed,
        form: SurfaceForm::Revolution {
            curve,
            origin,
            axis,
        },
    };
    // The foot of `p` on the axis.
    let foot = |p: Point3| origin + (p - origin).dot(&a) * a;
    match &curve.form {
        CurveForm::Exact(Curve::Line {
            origin: p,
            direction,
        }) => {
            let v = direction.into_inner();
            let w = *p - origin;
            let scale = w.norm();
            let cross = v.cross(&a);
            if is_negligible(cross.norm(), 1.0) {
                // Beside the axis: a cylinder, unless it is the axis.
                let radial = *p - foot(*p);
                let radius = radial.norm();
                if is_negligible(radius, scale) {
                    return Err(degenerate(id, "a line revolved about itself"));
                }
                let frame =
                    Frame::new(foot(*p), a, radial).map_err(|e| degenerate(id, e.to_string()))?;
                return exact(Surface::Cylinder { frame, radius }, *p, v);
            }
            if !is_negligible(w.dot(&cross), scale * cross.norm()) {
                return Ok(nurbs(curve));
            }
            // It meets the axis at `Q = P + s·v`, where `(P + s·v − A)`
            // has no component across `a`: `s = −(w × a)·(v × a) / |v × a|²`.
            let s = -w.cross(&a).dot(&cross) / cross.norm_squared();
            let meet = *p + s * v;
            // How far along the line from the axis the normals are
            // compared: as far as its own point, or any distance when
            // that is on the axis — the vector's length is one.
            let reach = match (*p - meet).norm() {
                r if !is_negligible(r, scale) => r,
                _ if scale > 0.0 => scale,
                _ => 1.0,
            };
            let cos = v.dot(&a);
            if is_negligible(cos, 1.0) {
                // Across the axis: a plane, whose file normal is `± a`
                // depending on the side of the axis, since the turned
                // line covers the plane twice. The side is the trim's,
                // where the file gives one, else the line's own point's,
                // or the one it runs to from the axis.
                let from = curve
                    .inside
                    .filter(|q| !is_negligible((q - meet).norm(), scale))
                    .unwrap_or(*p);
                let side = if (from - meet).dot(&v) < 0.0 {
                    -1.0
                } else {
                    1.0
                };
                let frame =
                    Frame::new(foot(meet), a, v).map_err(|e| degenerate(id, e.to_string()))?;
                return exact(Surface::Plane { frame }, meet + side * reach * v, v);
            }
            // Along the axis at an angle: a cone. `Z` is the side of the
            // axis the line runs up, `X` the side it runs out to, and the
            // frame sits where the line is `reach` out along `v`. Both
            // nappes are the one line turned, so the file's normal and
            // the cone's agree or disagree on both alike, and they are
            // compared on the nappe the frame is on.
            let z = if cos > 0.0 { a } else { -a };
            let out = v - v.dot(&z) * z;
            let x = out / out.norm();
            let half_angle = v.dot(&x).atan2(v.dot(&z));
            let sample = meet + reach * v;
            let base = meet + reach * v.dot(&z) * z;
            let radius = reach * half_angle.sin();
            let frame = Frame::new(base, z, x).map_err(|e| degenerate(id, e.to_string()))?;
            exact(
                Surface::Cone {
                    frame,
                    radius,
                    half_angle,
                },
                sample,
                v,
            )
        }
        CurveForm::Exact(Curve::Circle { frame, radius }) => {
            let n = frame.z().into_inner();
            let o = frame.origin();
            let scale = (o - origin).norm() + radius;
            let in_plane =
                is_negligible(n.dot(&a), 1.0) && is_negligible((origin - o).dot(&n), scale);
            if !in_plane {
                return Ok(nurbs(curve));
            }
            let centre = foot(o);
            let offset = o - centre;
            let distance = offset.norm();
            // The point of the circle the normals are compared at. A
            // circle through the axis covers its sphere twice, with
            // opposite normals, so the point is on the half the file
            // means: inside its trim where it gives one, else the first
            // of the quarter points off the axis — where the circle
            // starts.
            let conic = Curve::Circle {
                frame: *frame,
                radius: *radius,
            };
            let off_axis = |q: &Point3| !is_negligible((q - foot(*q)).norm(), scale);
            let starts = [0.0, FRAC_PI_2, PI].map(|t| conic.eval(t).point);
            let point = curve
                .inside
                .filter(off_axis)
                .or_else(|| starts.into_iter().find(off_axis))
                .unwrap_or(o);
            let tangent = conic
                .project(point)
                .map(|at| conic.eval(at.t).d1)
                .map_err(|e| degenerate(id, e.to_string()))?;
            if is_negligible(distance, scale) {
                let frame = Frame::from_z(centre, a).map_err(|e| degenerate(id, e.to_string()))?;
                return exact(
                    Surface::Sphere {
                        frame,
                        radius: *radius,
                    },
                    point,
                    tangent,
                );
            }
            let frame = Frame::new(centre, a, offset).map_err(|e| degenerate(id, e.to_string()))?;
            let surface = torus(id, frame, distance, *radius)?;
            exact(surface, point, tangent)
        }
        CurveForm::Exact(Curve::Ellipse { .. } | Curve::Nurbs(_))
        | CurveForm::Parabola { .. }
        | CurveForm::Hyperbola { .. } => Ok(nurbs(curve)),
    }
}

impl ReadCurve {
    /// The curve, bounded to `within` where its exact form must be: an
    /// Arris curve that holds every point of it inside the ball, in the
    /// file entity's parametrisation direction (its sense is
    /// [`ReadCurve::reversed`]). Errors: the curve's exact form cannot be
    /// built over the range (a hyperbola so long that `eᵗ` overflows).
    pub(crate) fn resolve(&self, within: &Ball) -> Result<Curve, Refusal> {
        match &self.form {
            CurveForm::Exact(c) => Ok(c.clone()),
            CurveForm::Parabola { .. } | CurveForm::Hyperbola { .. } => {
                self.bounded(None, within).map(Curve::Nurbs)
            }
        }
    }

    /// A NURBS curve holding every point `P` of this curve whose
    /// `|M(P − c)| ≤ ρ` for the ball's centre `c` and radius `ρ`, where
    /// `M` removes the component along `across` (none: the identity).
    /// Closed forms bound the parameter: `|M(P(t) − c)|` grows past the
    /// ball's radius at a `t` found from its leading term. Errors: the
    /// curve's projection by `M` is bounded no matter the parameter — a
    /// line along `across`, a conic in a plane holding it — so no range
    /// is the part's.
    pub(crate) fn bounded(
        &self,
        across: Option<UnitVec3>,
        within: &Ball,
    ) -> Result<NurbsCurve, Refusal> {
        let m = |v: Vec3| match across {
            Some(d) => v - v.dot(&d) * d.into_inner(),
            None => v,
        };
        let id = self.id;
        let unbounded = || degenerate(id, "a curve that stays within the part at every parameter");
        let range =
            |lo: f64, hi: f64| Interval::new(lo, hi).map_err(|e| degenerate(id, e.to_string()));
        let rho = within.radius;
        match &self.form {
            CurveForm::Exact(Curve::Line { origin, direction }) => {
                let c = m(*origin - within.centre).norm();
                let slope = m(direction.into_inner()).norm();
                if is_negligible(slope, 1.0) {
                    return Err(unbounded());
                }
                let s = (rho + c) / slope;
                let (a, b) = (
                    *origin - s * direction.into_inner(),
                    *origin + s * direction.into_inner(),
                );
                NurbsCurve::new(1, vec![-s, -s, s, s], vec![a, b], vec![1.0; 2])
                    .map_err(|e| degenerate(id, e.to_string()))
            }
            CurveForm::Exact(Curve::Circle { frame, radius }) => {
                NurbsCurve::circle(frame, *radius, Interval::TURN)
                    .map_err(|e| degenerate(id, e.to_string()))
            }
            CurveForm::Exact(Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            }) => NurbsCurve::ellipse(frame, *major_radius, *minor_radius, Interval::TURN)
                .map_err(|e| degenerate(id, e.to_string())),
            CurveForm::Exact(Curve::Nurbs(n)) => Ok(n.clone()),
            CurveForm::Parabola { frame, focal } => {
                // |A t² + B t + C| ≥ |A| t² − |B| |t| − |C|.
                let a = (*focal * m(frame.x().into_inner())).norm();
                let b = (2.0 * *focal * m(frame.y().into_inner())).norm();
                let c = m(frame.origin() - within.centre).norm();
                let s = if !is_negligible(a, *focal) {
                    (b + (b * b + 4.0 * a * (c + rho)).sqrt()) / (2.0 * a)
                } else if !is_negligible(b, *focal) {
                    (c + rho) / b
                } else {
                    return Err(unbounded());
                };
                NurbsCurve::parabola(frame, *focal, range(-s, s)?)
                    .map_err(|e| degenerate(id, e.to_string()))
            }
            CurveForm::Hyperbola {
                frame,
                semi_axis,
                semi_imaginary,
            } => {
                // a cosh t·X + b sinh t·Y = eᵗ·P + e⁻ᵗ·Q, and
                // |eᵗ P + e⁻ᵗ Q| ≥ |P| eᵗ − |Q| for t ≥ 0.
                let x = *semi_axis * m(frame.x().into_inner());
                let y = *semi_imaginary * m(frame.y().into_inner());
                let (p, q) = (0.5 * (x + y).norm(), 0.5 * (x - y).norm());
                let c = m(frame.origin() - within.centre).norm();
                let scale = semi_axis + semi_imaginary;
                if is_negligible(p, scale) || is_negligible(q, scale) {
                    return Err(unbounded());
                }
                let hi = ((rho + c + q) / p).ln().max(0.0);
                let lo = -((rho + c + p) / q).ln().max(0.0);
                // The range always holds `t = 0`, so it is never empty.
                let (lo, hi) = if hi > lo { (lo, hi) } else { (lo, lo + 1.0) };
                NurbsCurve::hyperbola(frame, *semi_axis, *semi_imaginary, range(lo, hi)?)
                    .map_err(|e| degenerate(id, e.to_string()))
            }
        }
    }
}

impl ReadSurface {
    /// The surface, bounded to `within` where its exact form must be: an
    /// Arris surface holding every point of it inside the ball. Its
    /// normal is against the file entity's where
    /// [`ReadSurface::reversed`] says so. Errors: the exact form cannot
    /// be built over the range.
    pub(crate) fn resolve(&self, within: &Ball) -> Result<Surface, Refusal> {
        let id = self.id;
        let fault = |e: arris_geom::GeomError| degenerate(id, e.to_string());
        match &self.form {
            SurfaceForm::Exact(s) => Ok(s.clone()),
            SurfaceForm::Extrusion { curve, direction } => {
                // A point `X = C(u) + v·d` of the part has `C(u)` on the
                // part's side of the plane across `d`, and `|v| ≤ ρ + h`
                // for `h` the farthest control point from the centre.
                let c = curve.bounded(Some(*direction), within)?;
                let reach = c
                    .control_points()
                    .iter()
                    .map(|p| (p - within.centre).norm())
                    .fold(0.0, f64::max);
                let v = within.radius + reach;
                let range = Interval::new(-v, v).map_err(|e| degenerate(id, e.to_string()))?;
                NurbsSurface::extrusion(&c, direction.into_inner(), range)
                    .map(Surface::Nurbs)
                    .map_err(fault)
            }
            SurfaceForm::Revolution {
                curve,
                origin,
                axis,
            } => {
                // Turning keeps every distance to a point `A` of the axis,
                // so a point of the part is turned from a point of the
                // curve within `ρ + |c − A|` of the centre's foot `A`.
                let a = axis.into_inner();
                let foot = origin + (within.centre - origin).dot(&a) * a;
                let ball = Ball {
                    centre: foot,
                    radius: within.radius + (within.centre - foot).norm(),
                };
                let c = curve.bounded(None, &ball)?;
                NurbsSurface::revolution(&c, *origin, *axis, Interval::TURN)
                    .map(Surface::Nurbs)
                    .map_err(fault)
            }
        }
    }
}

/// The parts of a B-spline instance of any subtype, simple or complex,
/// that the knots and weights are read from.
struct Spline<'a> {
    /// The record holding the degrees and the control points.
    args: Args<'a>,
    /// Where the degrees start in it: after the name in a simple
    /// instance, first in a complex one's `B_SPLINE_*` partial entity.
    first: usize,
    /// The degree per direction.
    degrees: Vec<usize>,
    /// How the knots are given.
    knots: KnotForm<'a>,
    /// The `RATIONAL_B_SPLINE_*` partial entity, if any.
    rational: Option<Args<'a>>,
}

/// `knots` of degree `p` over `n` control points with a padding knot
/// moved onto the domain's end where that end already has multiplicity
/// `p`: the knot outside it enters no basis function on the domain, so the
/// curve or surface is the same there, and it is now clamped. Open
/// CASCADE writes a periodic B-spline so — `(1, 2, …, 2, 1)` of degree
/// two around a closed circle — and clamped, a direction whose ends are
/// one row is closed (`NurbsSurface::closure`), which the pcurves and the
/// seams need.
fn clamped(mut knots: Vec<f64>, p: usize, n: usize) -> Vec<f64> {
    if knots.len() == n + p + 1 && p >= 1 {
        if knots[1..=p].iter().all(|&k| k == knots[p]) {
            knots[0] = knots[p];
        }
        if knots[n..n + p].iter().all(|&k| k == knots[n]) {
            knots[n + p] = knots[n];
        }
    }
    knots
}

/// The knot subtype of a B-spline.
enum KnotForm<'a> {
    /// `_WITH_KNOTS`: multiplicities and values per direction, from
    /// parameter `first` of the record.
    Given(Args<'a>, usize),
    /// `UNIFORM_*`: `−p, …, n` once each.
    Uniform,
    /// `QUASI_UNIFORM_*`: `0` and `n − p` of multiplicity `p + 1`, the
    /// integers between once.
    QuasiUniform,
    /// `BEZIER_*`: pieces of degree `p`, joined at the integers.
    Bezier,
}

impl<'a> Spline<'a> {
    /// The spline parts of `instance`, a curve for `dims` 1 and a surface
    /// for 2.
    fn of(id: u64, instance: &'a Instance, dims: usize) -> Result<Self, Refusal> {
        let (base, subtypes) = if dims == 1 {
            (
                "B_SPLINE_CURVE",
                [
                    "B_SPLINE_CURVE_WITH_KNOTS",
                    "UNIFORM_CURVE",
                    "QUASI_UNIFORM_CURVE",
                    "BEZIER_CURVE",
                ],
            )
        } else {
            (
                "B_SPLINE_SURFACE",
                [
                    "B_SPLINE_SURFACE_WITH_KNOTS",
                    "UNIFORM_SURFACE",
                    "QUASI_UNIFORM_SURFACE",
                    "BEZIER_SURFACE",
                ],
            )
        };
        let rational = format!("RATIONAL_{base}");
        let record = |name: &str| instance.record(name).map(|record| Args { id, record });
        // A simple instance flattens the supertype's attributes after the
        // name; a complex one holds them in the supertype's own record.
        let (args, first, simple) = match instance {
            Instance::Simple(r) => (Args { id, record: r }, 1, true),
            Instance::Complex(_) => {
                let args = record(base).ok_or_else(|| {
                    super::entities::malformed(id, format!("a complex B-spline with no {base}"))
                })?;
                (args, 0, false)
            }
        };
        let knots = if let Some(k) = record(subtypes[0]) {
            // After the supertype's attributes in a simple instance:
            // the degrees, the net, the form, a closure flag per
            // direction and the self-intersection flag.
            let at = if simple { first + 2 * dims + 3 } else { 0 };
            KnotForm::Given(k, at)
        } else if record(subtypes[1]).is_some() {
            KnotForm::Uniform
        } else if record(subtypes[2]).is_some() {
            KnotForm::QuasiUniform
        } else if record(subtypes[3]).is_some() {
            KnotForm::Bezier
        } else {
            return Err(super::entities::malformed(
                id,
                format!("{} gives no knots", describe(instance)),
            ));
        };
        let mut degrees = Vec::with_capacity(dims);
        for i in 0..dims {
            let d = args.integer(first + i)?;
            let d = usize::try_from(d)
                .ok()
                .filter(|&d| (1..=MAX_DEGREE).contains(&d))
                .ok_or_else(|| Refusal::Unsupported {
                    entity: id,
                    name: format!("a B-spline of degree {d}, where Arris holds 1 to {MAX_DEGREE}"),
                })?;
            degrees.push(d);
        }
        Ok(Spline {
            args,
            first,
            degrees,
            knots,
            rational: record(&rational),
        })
    }

    /// The expanded knots of direction `dir`, of degree `p` over `n`
    /// control points.
    fn knots(&self, dir: usize, p: usize, n: usize) -> Result<Vec<f64>, Refusal> {
        let bad = |what: String| self.args.malformed(what);
        if n <= p {
            return Err(bad(format!("{n} control points for degree {p}")));
        }
        let knots = match &self.knots {
            KnotForm::Given(args, at) => {
                let dims = self.degrees.len();
                let mults = args.integers(at + dir)?;
                let values = args.reals(at + dims + dir)?;
                if mults.len() != values.len() {
                    return Err(bad(
                        "knot values and multiplicities of different counts".into()
                    ));
                }
                let mut knots = Vec::new();
                for (&m, &k) in mults.iter().zip(&values) {
                    let m = usize::try_from(m)
                        .ok()
                        .filter(|&m| (1..=p + 1).contains(&m))
                        .ok_or_else(|| bad(format!("a knot multiplicity of {m}")))?;
                    knots.extend(core::iter::repeat_n(k, m));
                }
                knots
            }
            KnotForm::Uniform => (0..n + p + 1).map(|i| i as f64 - p as f64).collect(),
            KnotForm::QuasiUniform => {
                let last = (n - p) as f64;
                let mut knots = vec![0.0; p + 1];
                knots.extend((1..n - p).map(|i| i as f64));
                knots.extend(core::iter::repeat_n(last, p + 1));
                knots
            }
            KnotForm::Bezier => {
                if (n - 1) % p != 0 {
                    return Err(bad(format!(
                        "a Bézier of {n} control points, not whole pieces of degree {p}"
                    )));
                }
                let pieces = (n - 1) / p;
                let mut knots = vec![0.0; p + 1];
                for i in 1..pieces {
                    knots.extend(core::iter::repeat_n(i as f64, p));
                }
                knots.extend(core::iter::repeat_n(pieces as f64, p + 1));
                knots
            }
        };
        if knots.len() != n + p + 1 {
            return Err(bad(format!(
                "{} knots for {n} control points of degree {p}",
                knots.len()
            )));
        }
        Ok(clamped(knots, p, n))
    }

    /// The weights, flattened in the net's order, if the spline is
    /// rational.
    fn weights(&self) -> Result<Option<Vec<f64>>, Refusal> {
        let Some(r) = self.rational else {
            return Ok(None);
        };
        // The weights are the partial entity's only attribute, and the
        // last of a simple instance's.
        let at = r
            .len()
            .checked_sub(1)
            .ok_or_else(|| r.malformed("no weights"))?;
        let list = r.list(at)?;
        let mut weights = Vec::new();
        for w in list {
            match w {
                Param::List(row) => {
                    for x in row {
                        weights.push(
                            super::entities::number(x)
                                .ok_or_else(|| r.malformed("a weight that is not a number"))?,
                        );
                    }
                }
                x => weights.push(
                    super::entities::number(x)
                        .ok_or_else(|| r.malformed("a weight that is not a number"))?,
                ),
            }
        }
        Ok(Some(weights))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::step::part21;
    use arris_geom::CurveKind;
    use arris_math::Precision;
    use std::collections::BTreeMap;

    /// A file of the given data lines, in millimetres and radians.
    fn file(data: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    /// The common placements: `#1` the world frame, `#2` a frame off the
    /// origin with `Z = (0, 0.6, 0.8)`, `#3` a frame whose `Z` is world
    /// `X`; points `#11`–`#13` and directions `#21`–`#26`.
    const PLACEMENTS: &str = "#1=AXIS2_PLACEMENT_3D('',#11,#21,#22);
#2=AXIS2_PLACEMENT_3D('',#12,#23,#22);
#3=AXIS2_PLACEMENT_3D('',#13,#22,#24);
#11=CARTESIAN_POINT('',(0.,0.,0.));
#12=CARTESIAN_POINT('',(1.,-2.,3.));
#13=CARTESIAN_POINT('',(0.,5.,0.));
#21=DIRECTION('',(0.,0.,1.));
#22=DIRECTION('',(1.,0.,0.));
#23=DIRECTION('',(0.,0.6,0.8));
#24=DIRECTION('',(0.,1.,0.));
#25=DIRECTION('',(0.,1.,1.));
#26=DIRECTION('',(0.,0.,-1.));";

    struct Parsed(BTreeMap<u64, Instance>);

    impl Parsed {
        fn new(data: &str) -> Self {
            let text = file(&format!("{PLACEMENTS}\n{data}"));
            Parsed(part21::parse(&text).unwrap().instances)
        }

        fn geometry(&self) -> Geometry<'_> {
            Geometry {
                entities: Entities::new(&self.0),
                units: Units {
                    length: 1.0,
                    angle: 1.0,
                    uncertainty: None,
                    motion: None,
                },
            }
        }

        fn curve(&self, id: u64) -> Result<ReadCurve, Refusal> {
            self.geometry().curve(id, id)
        }

        fn surface(&self, id: u64) -> Result<ReadSurface, Refusal> {
            self.geometry().surface(id, id)
        }
    }

    /// The part every unbounded form is bounded to in these tests.
    const PART: Ball = Ball {
        centre: Point3::new(0.5, 0.5, 0.5),
        radius: 20.0,
    };

    /// Far below the numbers here, far above rounding.
    const CLOSE: f64 = 1e-9;

    /// `read` holds every point of the file's `step(t)` for `t` in
    /// `range`, running the same way as its tangent unless `reversed`.
    fn curve_holds(read: &ReadCurve, range: Interval, step: impl Fn(f64) -> (Point3, Vec3)) {
        let curve = read.resolve(&PART).unwrap();
        let sense = if read.reversed { -1.0 } else { 1.0 };
        for i in 0..=16 {
            let (p, tangent) = step(range.lerp(f64::from(i) / 16.0));
            let (t, distance) = nearest(&curve, p);
            assert!(distance < CLOSE, "{p:?} is {distance} off {curve:?}");
            let ours = curve.eval(t).d1;
            assert!(
                sense * ours.dot(&tangent) > 0.0,
                "{p:?}: {ours:?} against {tangent:?}"
            );
        }
    }

    /// The parameter of `curve` nearest `p` and the distance: the closed
    /// form of an analytic curve; for a NURBS, the nearest of a dense
    /// sampling narrowed by ternary search, since `Curve::project` on a
    /// NURBS is a local search and these tests want the global answer.
    fn nearest(curve: &Curve, p: Point3) -> (f64, f64) {
        let Curve::Nurbs(n) = curve else {
            let at = curve.project(p).unwrap();
            return (at.t, at.distance);
        };
        let domain = n.domain();
        let d = |t: f64| (n.eval(t).point - p).norm();
        const SAMPLES: usize = 4096;
        let step = |i: usize| domain.lerp(i as f64 / SAMPLES as f64);
        let best = (0..=SAMPLES)
            .min_by(|&i, &j| d(step(i)).total_cmp(&d(step(j))))
            .unwrap();
        let (mut lo, mut hi) = (step(best.saturating_sub(1)), step((best + 1).min(SAMPLES)));
        for _ in 0..200 {
            let (a, b) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
            if d(a) < d(b) {
                hi = b;
            } else {
                lo = a;
            }
        }
        let t = 0.5 * (lo + hi);
        (t, d(t))
    }

    /// `read` holds every point of the file's `step(u, v)` over the
    /// ranges, its normal the file's `∂u × ∂v` unless `reversed`.
    fn surface_holds(
        read: &ReadSurface,
        u: Interval,
        v: Interval,
        step: impl Fn(f64, f64) -> (Point3, Vec3),
    ) {
        let surface = read.resolve(&PART).unwrap();
        let sense = if read.reversed { -1.0 } else { 1.0 };
        for i in 0..=8 {
            for j in 0..=8 {
                let (p, normal) = step(u.lerp(f64::from(i) / 8.0), v.lerp(f64::from(j) / 8.0));
                let at = surface.project(p).unwrap();
                assert!(
                    at.distance < CLOSE,
                    "{p:?} is {} off {surface:?}",
                    at.distance
                );
                if normal.norm() < CLOSE {
                    continue;
                }
                let ours = surface.normal(at.uv.x, at.uv.y).unwrap();
                assert!(
                    sense * ours.dot(&normal) > 0.0,
                    "{p:?}: {ours:?} against {normal:?} on {surface:?}"
                );
            }
        }
    }

    fn range(lo: f64, hi: f64) -> Interval {
        Interval::new(lo, hi).unwrap()
    }

    fn exact_curve(read: &ReadCurve) -> &Curve {
        match &read.form {
            CurveForm::Exact(c) => c,
            other => panic!("{other:?}"),
        }
    }

    fn exact_surface(read: &ReadSurface) -> &Surface {
        match &read.form {
            SurfaceForm::Exact(s) => s,
            other => panic!("{other:?}"),
        }
    }

    /// `v` turned by `angle` about the unit `axis` (Rodrigues).
    fn rotate(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
        let (s, c) = angle.sin_cos();
        v * c + axis.cross(&v) * s + axis * axis.dot(&v) * (1.0 - c)
    }

    /// ISO 10303-42's surface of revolution of `c` about the axis through
    /// `a0` along the unit `axis`: its point and `∂u × ∂v`.
    fn revolved(
        c: impl Fn(f64) -> (Point3, Vec3),
        a0: Point3,
        axis: Vec3,
    ) -> impl Fn(f64, f64) -> (Point3, Vec3) {
        move |u, v| {
            let (p, t) = c(v);
            let q = a0 + rotate(p - a0, axis, u);
            let du = axis.cross(&(q - a0));
            (q, du.cross(&rotate(t, axis, u)))
        }
    }

    /// ISO 10303-42's surface of linear extrusion of `c` along `d`.
    fn extruded(c: impl Fn(f64) -> (Point3, Vec3), d: Vec3) -> impl Fn(f64, f64) -> (Point3, Vec3) {
        move |u, v| {
            let (p, t) = c(u);
            (p + v * d, t.cross(&d))
        }
    }

    /// The ellipse `O + a cos t·X + b sin t·Y` of the frame `(o, x, y)`.
    fn conic(o: Point3, x: Vec3, y: Vec3, a: f64, b: f64) -> impl Fn(f64) -> (Point3, Vec3) {
        move |t| {
            let (s, c) = t.sin_cos();
            (o + a * c * x + b * s * y, -a * s * x + b * c * y)
        }
    }

    #[test]
    fn lines_and_conics_map_to_their_variants() {
        let x = Parsed::new(
            "#30=VECTOR('',#25,7.);
#31=LINE('',#12,#30);
#32=CIRCLE('',#2,2.5);
#33=ELLIPSE('',#2,1.5,4.);
#34=TRIMMED_CURVE('',#32,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.F.,.PARAMETER.);
#35=SURFACE_CURVE('',#34,(#90),.PCURVE_S1.);
#90=PLANE('',#1);",
        );
        let s = 0.5f64.sqrt();
        let line = x.curve(31).unwrap();
        assert!(matches!(exact_curve(&line), Curve::Line { .. }));
        curve_holds(&line, range(-3.0, 3.0), |t| {
            (
                Point3::new(1.0, -2.0 + s * t, 3.0 + s * t),
                Vec3::new(0.0, s, s),
            )
        });
        let (o, z) = (Point3::new(1.0, -2.0, 3.0), Vec3::new(0.0, 0.6, 0.8));
        let xa = Vec3::x();
        let ya = z.cross(&xa);
        let circle = x.curve(32).unwrap();
        assert!(matches!(
            exact_curve(&circle),
            Curve::Circle { radius: 2.5, .. }
        ));
        curve_holds(&circle, range(0.0, TAU), conic(o, xa, ya, 2.5, 2.5));
        // The longer semi-axis along `Y`: the frame turns a quarter and
        // the sense stays.
        let ellipse = x.curve(33).unwrap();
        let Curve::Ellipse {
            major_radius,
            minor_radius,
            ..
        } = exact_curve(&ellipse)
        else {
            panic!()
        };
        assert_eq!((*major_radius, *minor_radius), (4.0, 1.5));
        curve_holds(&ellipse, range(0.0, TAU), conic(o, xa, ya, 1.5, 4.0));
        // A trimmed curve of sense `.F.`, through a surface curve, is its
        // basis run backwards.
        let trimmed = x.curve(35).unwrap();
        assert!(trimmed.reversed);
        assert_eq!(exact_curve(&trimmed), exact_curve(&circle));
        let back = conic(o, xa, ya, 2.5, 2.5);
        curve_holds(&trimmed, range(0.0, TAU), |t| {
            let (p, d) = back(-t);
            (p, -d)
        });
    }

    use core::f64::consts::TAU;

    #[test]
    fn parabolas_hyperbolas_and_polylines_become_exact_nurbs() {
        let x = Parsed::new(
            "#31=PARABOLA('',#2,0.75);
#32=PARABOLA('',#2,-0.75);
#33=HYPERBOLA('',#2,2.,1.5);
#34=POLYLINE('',(#11,#12,#12,#13));",
        );
        let (o, z) = (Point3::new(1.0, -2.0, 3.0), Vec3::new(0.0, 0.6, 0.8));
        let (xa, ya) = (Vec3::x(), z.cross(&Vec3::x()));
        for (id, f) in [(31, 0.75), (32, -0.75)] {
            let read = x.curve(id).unwrap();
            assert!(matches!(read.resolve(&PART).unwrap(), Curve::Nurbs(_)));
            curve_holds(&read, range(-3.0, 3.0), |t| {
                (
                    o + f * t * t * xa + 2.0 * f * t * ya,
                    2.0 * f * t * xa + 2.0 * f * ya,
                )
            });
        }
        let hyperbola = x.curve(33).unwrap();
        curve_holds(&hyperbola, range(-1.5, 1.5), |t| {
            (
                o + 2.0 * t.cosh() * xa + 1.5 * t.sinh() * ya,
                2.0 * t.sinh() * xa + 1.5 * t.cosh() * ya,
            )
        });
        // The repeated point is dropped: three points, two spans.
        let polyline = x.curve(34).unwrap();
        let Curve::Nurbs(n) = exact_curve(&polyline) else {
            panic!()
        };
        assert_eq!(n.control_points().len(), 3);
        let (a, b, c) = (
            Point3::origin(),
            Point3::new(1.0, -2.0, 3.0),
            Point3::new(0.0, 5.0, 0.0),
        );
        curve_holds(&polyline, range(0.01, 0.99), |t| (a + t * (b - a), b - a));
        curve_holds(&polyline, range(0.01, 0.99), |t| (b + t * (c - b), c - b));
    }

    /// Control points `#41`–`#46` on a wavy line, for the B-splines.
    const WAVE: &str = "#41=CARTESIAN_POINT('',(0.,0.,0.));
#42=CARTESIAN_POINT('',(1.,2.,0.));
#43=CARTESIAN_POINT('',(2.,-1.,1.));
#44=CARTESIAN_POINT('',(3.,1.,0.));
#45=CARTESIAN_POINT('',(4.,0.,2.));
#46=CARTESIAN_POINT('',(5.,1.,1.));";

    #[test]
    fn every_bspline_curve_subtype_expands_its_knots() {
        let x = Parsed::new(&format!(
            "{WAVE}
#51=B_SPLINE_CURVE_WITH_KNOTS('',2,(#41,#42,#43,#44,#45),.UNSPECIFIED.,.F.,.F.,(3,1,1,3),(0.,0.5,1.5,2.),.UNSPECIFIED.);
#52=UNIFORM_CURVE('',2,(#41,#42,#43,#44,#45),.UNSPECIFIED.,.F.,.F.);
#53=QUASI_UNIFORM_CURVE('',2,(#41,#42,#43,#44,#45),.UNSPECIFIED.,.F.,.F.);
#54=BEZIER_CURVE('',2,(#41,#42,#43,#44,#45),.UNSPECIFIED.,.F.,.F.);
#55=( BOUNDED_CURVE() B_SPLINE_CURVE(2,(#41,#42,#43),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.) CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,0.5,1.)) REPRESENTATION_ITEM('') );
#56=( BOUNDED_CURVE() B_SPLINE_CURVE(2,(#41,#42,#43,#44,#45),.UNSPECIFIED.,.F.,.F.) BEZIER_CURVE() CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE((1.,2.,1.,0.5,1.)) REPRESENTATION_ITEM('') );
#57=UNIFORM_CURVE('',2,(#41,#42,#43,#44,#41,#42),.UNSPECIFIED.,.T.,.F.);"
        ));
        let knots = |id| match exact_curve(&x.curve(id).unwrap()) {
            Curve::Nurbs(n) => (n.knots().to_vec(), n.weights().to_vec()),
            other => panic!("{other:?}"),
        };
        assert_eq!(knots(51).0, [0.0, 0.0, 0.0, 0.5, 1.5, 2.0, 2.0, 2.0]);
        assert_eq!(knots(52).0, [-2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0]);
        assert_eq!(knots(53).0, [0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0]);
        assert_eq!(knots(54).0, [0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0]);
        assert_eq!(
            knots(55),
            (vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![1.0, 0.5, 1.0])
        );
        assert_eq!(knots(56).1, [1.0, 2.0, 1.0, 0.5, 1.0]);
        assert_eq!(knots(56).0, knots(54).0);
        // A uniform curve whose last points repeat its first wraps.
        let Curve::Nurbs(closed) = exact_curve(&x.curve(57).unwrap()).clone() else {
            panic!()
        };
        assert_eq!(closed.period(), Some(4.0));
        // Held pointwise to the curve the knots define: the uniform one
        // on its domain `[0, 3]`.
        let uniform = x.curve(52).unwrap();
        let Curve::Nurbs(n) = exact_curve(&uniform).clone() else {
            panic!()
        };
        assert_eq!(n.domain(), range(0.0, 3.0));
    }

    #[test]
    fn a_bspline_whose_knots_do_not_fit_is_refused() {
        let x = Parsed::new(&format!(
            "{WAVE}
#51=B_SPLINE_CURVE_WITH_KNOTS('',2,(#41,#42,#43,#44,#45),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.);
#52=BEZIER_CURVE('',2,(#41,#42,#43,#44),.UNSPECIFIED.,.F.,.F.);
#53=B_SPLINE_CURVE_WITH_KNOTS('',30,(#41,#42),.UNSPECIFIED.,.F.,.F.,(2,2),(0.,1.),.UNSPECIFIED.);
#54=B_SPLINE_CURVE_WITH_KNOTS('',1,(#41,#42),.UNSPECIFIED.,.F.,.F.,(2,2),(1.,0.),.UNSPECIFIED.);"
        ));
        assert_eq!(
            x.curve(51).unwrap_err().kind(),
            super::super::RefusalKind::Malformed
        );
        assert_eq!(
            x.curve(52).unwrap_err().kind(),
            super::super::RefusalKind::Malformed
        );
        assert_eq!(
            x.curve(53).unwrap_err().kind(),
            super::super::RefusalKind::Unsupported
        );
        assert_eq!(
            x.curve(54).unwrap_err().kind(),
            super::super::RefusalKind::Degenerate
        );
    }

    #[test]
    fn every_bspline_surface_subtype_maps_to_a_nurbs_surface() {
        // A 3 × 3 net over the wave's points and their lifts.
        let x = Parsed::new(&format!(
            "{WAVE}
#47=CARTESIAN_POINT('',(0.,3.,1.));
#48=CARTESIAN_POINT('',(1.,3.,2.));
#49=CARTESIAN_POINT('',(2.,4.,0.));
#61=B_SPLINE_SURFACE_WITH_KNOTS('',2,1,((#41,#42),(#43,#44),(#45,#46)),.UNSPECIFIED.,.F.,.F.,.F.,(3,3),(2,2),(0.,1.),(0.,2.),.UNSPECIFIED.);
#62=( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,1,((#41,#42),(#43,#44),(#45,#46)),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),(0.,1.),(0.,2.),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((1.,1.),(0.5,2.),(1.,1.))) REPRESENTATION_ITEM('') SURFACE() );
#63=QUASI_UNIFORM_SURFACE('',2,1,((#41,#42),(#43,#44),(#45,#46)),.UNSPECIFIED.,.F.,.F.,.F.);
#64=BEZIER_SURFACE('',1,1,((#41,#42),(#43,#44),(#45,#46)),.UNSPECIFIED.,.F.,.F.,.F.);
#65=UNIFORM_SURFACE('',1,1,((#41,#42),(#43,#44),(#45,#46)),.UNSPECIFIED.,.F.,.F.,.F.);
#66=B_SPLINE_SURFACE_WITH_KNOTS('',2,1,((#41,#42),(#43,#44)),.UNSPECIFIED.,.F.,.F.,.F.,(3,3),(2,2),(0.,1.),(0.,2.),.UNSPECIFIED.);
#67=B_SPLINE_SURFACE_WITH_KNOTS('',2,1,((#41,#42),(#43),(#45,#46)),.UNSPECIFIED.,.F.,.F.,.F.,(3,3),(2,2),(0.,1.),(0.,2.),.UNSPECIFIED.);"
        ));
        let nurbs = |id| match exact_surface(&x.surface(id).unwrap()) {
            Surface::Nurbs(n) => n.clone(),
            other => panic!("{other:?}"),
        };
        let a = nurbs(61);
        assert_eq!(a.counts(), [3, 2]);
        assert_eq!(a.control_point(1, 0), Some(Point3::new(2.0, -1.0, 1.0)));
        assert_eq!(a.knots()[1], [0.0, 0.0, 2.0, 2.0]);
        let b = nurbs(62);
        assert_eq!(b.weights(), [1.0, 1.0, 0.5, 2.0, 1.0, 1.0]);
        assert_eq!(nurbs(63).knots()[0], [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
        assert_eq!(nurbs(64).knots()[0], [0.0, 0.0, 1.0, 2.0, 2.0]);
        // `−1, 0, 1, 2`, whose ends pad a degree-one domain and enter no
        // basis function on it: clamped, the same surface.
        assert_eq!(nurbs(65).knots()[1], [0.0, 0.0, 1.0, 1.0]);
        assert_eq!(
            x.surface(66).unwrap_err().kind(),
            super::super::RefusalKind::Malformed
        );
        assert_eq!(
            x.surface(67).unwrap_err().kind(),
            super::super::RefusalKind::Malformed
        );
    }

    #[test]
    fn elementary_surfaces_stay_themselves() {
        let x = Parsed::new(
            "#71=PLANE('',#2);
#72=CYLINDRICAL_SURFACE('',#2,3.);
#73=CONICAL_SURFACE('',#2,1.,0.3);
#74=SPHERICAL_SURFACE('',#2,4.);
#75=TOROIDAL_SURFACE('',#2,5.,2.);
#76=RECTANGULAR_TRIMMED_SURFACE('',#72,0.,1.,0.,1.,.F.,.T.);
#77=RECTANGULAR_TRIMMED_SURFACE('',#72,0.,1.,0.,1.,.F.,.F.);",
        );
        for (id, kind) in [
            (71, "plane"),
            (72, "cylinder"),
            (73, "cone"),
            (74, "sphere"),
            (75, "torus"),
        ] {
            let read = x.surface(id).unwrap();
            assert!(!read.reversed);
            assert_eq!(exact_surface(&read).kind().to_string(), kind);
        }
        assert!(x.surface(76).unwrap().reversed, "one sense turned");
        assert!(!x.surface(77).unwrap().reversed, "both turned");
    }

    #[test]
    fn extrusions_become_planes_cylinders_or_nurbs() {
        let x = Parsed::new(&format!(
            "{WAVE}
#30=VECTOR('',#22,1.);
#31=LINE('',#12,#30);
#32=VECTOR('',#25,3.);
#33=VECTOR('',#26,1.);
#34=CIRCLE('',#1,2.);
#35=ELLIPSE('',#2,3.,1.);
#36=B_SPLINE_CURVE_WITH_KNOTS('',2,(#41,#42,#43,#44,#45),.UNSPECIFIED.,.F.,.F.,(3,1,1,3),(0.,0.5,1.5,2.),.UNSPECIFIED.);
#37=PARABOLA('',#1,0.5);
#38=TRIMMED_CURVE('',#34,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.F.,.PARAMETER.);
#81=SURFACE_OF_LINEAR_EXTRUSION('',#31,#32);
#82=SURFACE_OF_LINEAR_EXTRUSION('',#34,#32);
#83=SURFACE_OF_LINEAR_EXTRUSION('',#34,#33);
#84=SURFACE_OF_LINEAR_EXTRUSION('',#35,#32);
#85=SURFACE_OF_LINEAR_EXTRUSION('',#36,#32);
#86=SURFACE_OF_LINEAR_EXTRUSION('',#37,#32);
#87=SURFACE_OF_LINEAR_EXTRUSION('',#38,#32);
#88=SURFACE_OF_LINEAR_EXTRUSION('',#31,#30);
#89=SURFACE_OF_LINEAR_EXTRUSION('',#34,#30);"
        ));
        let s = 0.5f64.sqrt();
        let d = Vec3::new(0.0, s, s);
        let v = range(-4.0, 4.0);
        // A line: a plane whose normal is the file's.
        let plane = x.surface(81).unwrap();
        assert!(matches!(exact_surface(&plane), Surface::Plane { .. }));
        surface_holds(
            &plane,
            range(-3.0, 3.0),
            v,
            extruded(|t| (Point3::new(1.0 + t, -2.0, 3.0), Vec3::x()), d),
        );
        // A circle in `z = 0` along `(0, 1, 1)`: an elliptic cylinder of
        // semi-axes 2 and 2/√2.
        let world = |a: f64, b: f64| conic(Point3::origin(), Vec3::x(), Vec3::y(), a, b);
        let oblique = x.surface(82).unwrap();
        let Surface::EllipticCylinder {
            major_radius,
            minor_radius,
            ..
        } = exact_surface(&oblique)
        else {
            panic!("{oblique:?}")
        };
        assert!((major_radius - 2.0).abs() < CLOSE && (minor_radius - 2.0 * s).abs() < CLOSE);
        surface_holds(&oblique, range(0.0, TAU), v, extruded(world(2.0, 2.0), d));
        // Along `−Z`: a cylinder turned inside out.
        let inward = x.surface(83).unwrap();
        assert!(matches!(
            exact_surface(&inward),
            Surface::Cylinder { radius: 2.0, .. }
        ));
        assert!(inward.reversed);
        surface_holds(
            &inward,
            range(0.0, TAU),
            v,
            extruded(world(2.0, 2.0), -Vec3::z()),
        );
        // An ellipse off the origin, obliquely.
        let (o, z) = (Point3::new(1.0, -2.0, 3.0), Vec3::new(0.0, 0.6, 0.8));
        let tilted = x.surface(84).unwrap();
        assert!(matches!(
            exact_surface(&tilted),
            Surface::EllipticCylinder { .. }
        ));
        surface_holds(
            &tilted,
            range(0.0, TAU),
            v,
            extruded(conic(o, Vec3::x(), z.cross(&Vec3::x()), 3.0, 1.0), d),
        );
        // A spline and a parabola: exact NURBS, bounded to the part.
        let Curve::Nurbs(wave) = exact_curve(&x.curve(36).unwrap()).clone() else {
            panic!()
        };
        let spline = x.surface(85).unwrap();
        assert!(matches!(spline.resolve(&PART).unwrap(), Surface::Nurbs(_)));
        surface_holds(
            &spline,
            range(0.0, 2.0),
            v,
            extruded(
                |t| {
                    let e = wave.eval(t);
                    (e.point, e.d1)
                },
                d,
            ),
        );
        let parabola = x.surface(86).unwrap();
        surface_holds(
            &parabola,
            range(-3.0, 3.0),
            v,
            extruded(
                |t| (Point3::new(0.5 * t * t, t, 0.0), Vec3::new(t, 1.0, 0.0)),
                d,
            ),
        );
        // A circle run backwards turns the normal.
        let backwards = x.surface(87).unwrap();
        assert!(backwards.reversed);
        surface_holds(
            &backwards,
            range(0.0, TAU),
            v,
            extruded(
                |t| {
                    let (p, d) = world(2.0, 2.0)(-t);
                    (p, -d)
                },
                d,
            ),
        );
        // Along its own line, or along its own plane: nothing.
        assert_eq!(
            x.surface(88).unwrap_err().kind(),
            super::super::RefusalKind::Degenerate
        );
        assert_eq!(
            x.surface(89).unwrap_err().kind(),
            super::super::RefusalKind::Degenerate
        );
    }

    #[test]
    fn revolutions_become_quadrics_tori_or_nurbs() {
        let x = Parsed::new(&format!(
            "{WAVE}
#30=VECTOR('',#21,1.);
#90=AXIS1_PLACEMENT('',#11,#21);
#91=CARTESIAN_POINT('',(2.,0.,0.));
#92=CARTESIAN_POINT('',(0.,0.,-1.));
#93=DIRECTION('',(0.6,0.,0.8));
#94=DIRECTION('',(-0.6,0.,-0.8));
#95=DIRECTION('',(0.,1.,0.5));
#31=LINE('',#91,#30);
#32=LINE('',#92,#101);
#33=LINE('',#92,#102);
#34=LINE('',#91,#103);
#35=LINE('',#92,#104);
#101=VECTOR('',#93,1.);
#102=VECTOR('',#94,1.);
#103=VECTOR('',#95,1.);
#104=VECTOR('',#22,1.);
#110=AXIS2_PLACEMENT_3D('',#11,#24,#22);
#111=AXIS2_PLACEMENT_3D('',#91,#24,#22);
#112=AXIS2_PLACEMENT_3D('',#91,#23,#22);
#36=CIRCLE('',#110,3.);
#37=CIRCLE('',#111,0.5);
#38=CIRCLE('',#111,2.5);
#39=CIRCLE('',#112,0.5);
#40=ELLIPSE('',#111,0.75,0.5);
#51=B_SPLINE_CURVE_WITH_KNOTS('',2,(#42,#43,#44),.UNSPECIFIED.,.F.,.F.,(3,3),(0.,1.),.UNSPECIFIED.);
#52=HYPERBOLA('',#111,0.5,0.25);
#81=SURFACE_OF_REVOLUTION('',#31,#90);
#82=SURFACE_OF_REVOLUTION('',#32,#90);
#83=SURFACE_OF_REVOLUTION('',#33,#90);
#84=SURFACE_OF_REVOLUTION('',#34,#90);
#85=SURFACE_OF_REVOLUTION('',#35,#90);
#86=SURFACE_OF_REVOLUTION('',#36,#90);
#87=SURFACE_OF_REVOLUTION('',#37,#90);
#88=SURFACE_OF_REVOLUTION('',#38,#90);
#89=SURFACE_OF_REVOLUTION('',#39,#90);
#96=SURFACE_OF_REVOLUTION('',#40,#90);
#97=SURFACE_OF_REVOLUTION('',#51,#90);
#98=SURFACE_OF_REVOLUTION('',#52,#90);
#120=TRIMMED_CURVE('',#35,(PARAMETER_VALUE(-3.)),(PARAMETER_VALUE(-1.)),.T.,.PARAMETER.);
#121=SURFACE_OF_REVOLUTION('',#120,#90);
#122=TRIMMED_CURVE('',#36,(PARAMETER_VALUE(1.5707963267948966)),(PARAMETER_VALUE(4.71238898038469)),.T.,.PARAMETER.);
#123=SURFACE_OF_REVOLUTION('',#122,#90);
#124=TRIMMED_CURVE('',#36,(PARAMETER_VALUE(4.71238898038469)),(PARAMETER_VALUE(1.5707963267948966)),.F.,.PARAMETER.);
#125=SURFACE_OF_REVOLUTION('',#124,#90);"
        ));
        let axis = Vec3::z();
        let o = Point3::origin();
        let turn = range(0.0, TAU);
        let line = |p: Point3, d: Vec3| move |t: f64| (p + t * d, d);
        // Beside the axis: a cylinder.
        let cylinder = x.surface(81).unwrap();
        assert!(matches!(
            exact_surface(&cylinder),
            Surface::Cylinder { radius: 2.0, .. }
        ));
        surface_holds(
            &cylinder,
            turn,
            range(-3.0, 3.0),
            revolved(line(Point3::new(2.0, 0.0, 0.0), axis), o, axis),
        );
        // Meeting it at an angle, each way along the line: a cone, both
        // nappes held.
        let up = Vec3::new(0.6, 0.0, 0.8);
        for (id, d) in [(82, up), (83, -up)] {
            let cone = x.surface(id).unwrap();
            assert!(
                matches!(exact_surface(&cone), Surface::Cone { .. }),
                "{cone:?}"
            );
            surface_holds(
                &cone,
                turn,
                range(-4.1, 3.9),
                revolved(line(Point3::new(0.0, 0.0, -1.0), d), o, axis),
            );
        }
        // Skew to it: a hyperboloid, as NURBS.
        let skew = x.surface(84).unwrap();
        assert!(matches!(skew.form, SurfaceForm::Revolution { .. }));
        surface_holds(
            &skew,
            turn,
            range(-3.0, 3.0),
            revolved(
                line(
                    Point3::new(2.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.5).normalize(),
                ),
                o,
                axis,
            ),
        );
        // Across it through the axis: a plane, on the line's side.
        let plane = x.surface(85).unwrap();
        assert!(matches!(exact_surface(&plane), Surface::Plane { .. }));
        surface_holds(
            &plane,
            turn,
            range(0.5, 3.0),
            revolved(line(Point3::new(0.0, 0.0, -1.0), Vec3::x()), o, axis),
        );
        // The line trimmed to the axis's other side: the plane's other
        // sense.
        let other = x.surface(121).unwrap();
        assert_ne!(other.reversed, plane.reversed);
        assert_eq!(other.form, plane.form);
        surface_holds(
            &other,
            turn,
            range(-3.0, -0.5),
            revolved(line(Point3::new(0.0, 0.0, -1.0), Vec3::x()), o, axis),
        );
        // Circles in a plane through the axis: a sphere, a torus, and a
        // torus that would cross its axis.
        // The circles' frames have `Z` along world `Y`, so their `Y` is
        // `Z × X`, world `−Z`: they run clockwise in the half-plane, and
        // the file's normal points in.
        let circle = |p: Point3, r: f64| conic(p, Vec3::x(), -Vec3::z(), r, r);
        let sphere = x.surface(86).unwrap();
        assert!(matches!(
            exact_surface(&sphere),
            Surface::Sphere { radius: 3.0, .. }
        ));
        // A whole circle through the axis covers the sphere twice; the
        // half it starts on, `x > 0`, is the one read.
        assert!(sphere.reversed);
        surface_holds(
            &sphere,
            turn,
            range(-1.4, 1.4),
            revolved(circle(o, 3.0), o, axis),
        );
        // Trimmed to the other half, either way round the trim is
        // written, it is the sphere's other sense; run backwards, the
        // same again.
        let (half, backwards) = (x.surface(123).unwrap(), x.surface(125).unwrap());
        assert_eq!(half.form, sphere.form);
        assert!(!half.reversed);
        surface_holds(
            &half,
            turn,
            range(1.7, 4.6),
            revolved(circle(o, 3.0), o, axis),
        );
        assert!(backwards.reversed, "the other half, run backwards");
        let torus = x.surface(87).unwrap();
        assert!(matches!(
            exact_surface(&torus),
            Surface::Torus {
                major_radius: 2.0,
                minor_radius: 0.5,
                ..
            }
        ));
        // The circle's `Y` is `Z × X = −(world Z)` here: its frame's `Z`
        // is world `Y`.
        let torus_circle = |t: f64| {
            let (s, c) = t.sin_cos();
            let p = Point3::new(2.0, 0.0, 0.0) + 0.5 * c * Vec3::x() - 0.5 * s * Vec3::z();
            (p, -0.5 * s * Vec3::x() - 0.5 * c * Vec3::z())
        };
        assert!(
            torus.reversed,
            "the circle runs clockwise in the half-plane"
        );
        surface_holds(&torus, turn, turn, revolved(torus_circle, o, axis));
        assert_eq!(
            x.surface(88).unwrap_err().kind(),
            super::super::RefusalKind::SelfIntersectingTorus
        );
        // Out of the axis's plane, an ellipse, a spline, a hyperbola: NURBS.
        for id in [89, 96, 97, 98] {
            let read = x.surface(id).unwrap();
            assert!(matches!(read.form, SurfaceForm::Revolution { .. }), "{id}");
            assert!(matches!(read.resolve(&PART).unwrap(), Surface::Nurbs(_)));
        }
        let Curve::Nurbs(arc) = exact_curve(&x.curve(51).unwrap()).clone() else {
            panic!()
        };
        surface_holds(
            &x.surface(97).unwrap(),
            turn,
            range(0.0, 1.0),
            revolved(
                move |t| {
                    let e = arc.eval(t);
                    (e.point, e.d1)
                },
                o,
                axis,
            ),
        );
        let tilted = x.curve(39).unwrap();
        let Curve::Circle { frame, .. } = exact_curve(&tilted).clone() else {
            panic!()
        };
        surface_holds(
            &x.surface(89).unwrap(),
            turn,
            turn,
            revolved(
                conic(
                    frame.origin(),
                    frame.x().into_inner(),
                    frame.y().into_inner(),
                    0.5,
                    0.5,
                ),
                o,
                axis,
            ),
        );
        surface_holds(
            &x.surface(96).unwrap(),
            turn,
            turn,
            revolved(
                |t: f64| {
                    let (s, c) = t.sin_cos();
                    (
                        Point3::new(2.0 + 0.75 * c, 0.0, -0.5 * s),
                        Vec3::new(-0.75 * s, 0.0, -0.5 * c),
                    )
                },
                o,
                axis,
            ),
        );
    }

    #[test]
    fn entities_outside_the_subset_are_refused_by_name() {
        use super::super::RefusalKind as K;
        let x = Parsed::new(
            "#71=CYLINDRICAL_SURFACE('',#2,3.);
#72=TOROIDAL_SURFACE('',#2,2.,2.);
#73=CYLINDRICAL_SURFACE('',#2,0.);
#74=CONICAL_SURFACE('',#2,1.,1.6);
#31=CIRCLE('',#1,2.);
#81=OFFSET_SURFACE('',#71,1.,.F.);
#82=OFFSET_CURVE_3D('',#31,1.,.F.,#21);
#83=COMPOSITE_CURVE('',(#90),.F.);
#84=RECTANGULAR_COMPOSITE_SURFACE('',((#91)));
#85=CURVE_BOUNDED_SURFACE('',#71,(#90),.F.);
#86=DEGENERATE_TOROIDAL_SURFACE('',#2,1.,2.,.T.);
#87=CLOTHOID('',#1,1.);
#88=TRIMMED_CURVE('',#89,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.T.,.PARAMETER.);
#89=TRIMMED_CURVE('',#88,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE(1.)),.T.,.PARAMETER.);
#92=PCURVE('',#71,#93);
#94=CIRCLE('',#1,-1.);
#95=( BOUNDED_CURVE() CURVE() GEOMETRIC_REPRESENTATION_ITEM() REPRESENTATION_ITEM('') );
#96=LINE('',#11,#71);",
        );
        for (id, kind) in [
            (81, K::Offset),
            (84, K::Composite),
            (85, K::CurveBounded),
            (86, K::DegenerateTorus),
        ] {
            let e = x.surface(id).unwrap_err();
            assert_eq!((e.kind(), e.entity()), (kind, id), "{e}");
        }
        for (id, kind) in [
            (82, K::Offset),
            (83, K::Composite),
            (87, K::Unsupported),
            (92, K::Unsupported),
            (95, K::Unsupported),
        ] {
            let e = x.curve(id).unwrap_err();
            assert_eq!((e.kind(), e.entity()), (kind, id), "{e}");
        }
        assert_eq!(x.surface(72).unwrap_err().kind(), K::SelfIntersectingTorus);
        assert_eq!(x.surface(73).unwrap_err().kind(), K::Degenerate);
        assert_eq!(x.surface(74).unwrap_err().kind(), K::Degenerate);
        assert_eq!(x.curve(94).unwrap_err().kind(), K::Degenerate);
        // A cycle of references ends, as malformed.
        assert_eq!(x.curve(88).unwrap_err().kind(), K::Malformed);
        // A reference to the wrong entity type names the one referred to.
        let e = x.curve(96).unwrap_err();
        assert_eq!((e.kind(), e.entity()), (K::Malformed, 71), "{e}");
        // A curve where a surface belongs, and the other way round.
        assert_eq!(x.surface(31).unwrap_err().kind(), K::Unsupported);
        assert_eq!(x.curve(71).unwrap_err().kind(), K::Unsupported);
    }

    /// Two values of one analytic variant equal to rounding, a NURBS
    /// exactly: what a file written and read back holds.
    fn same_frame(a: &Frame, b: &Frame) -> bool {
        let scale = a.origin().coords.norm().max(1.0);
        is_negligible((a.origin() - b.origin()).norm(), scale)
            && is_negligible((a.z().into_inner() - b.z().into_inner()).norm(), 1.0)
            && is_negligible((a.x().into_inner() - b.x().into_inner()).norm(), 1.0)
    }

    fn same(a: f64, b: f64) -> bool {
        is_negligible(a - b, a.abs().max(b.abs()))
    }

    fn same_surface(a: &Surface, b: &Surface) -> bool {
        match (a, b) {
            (Surface::Plane { frame: f }, Surface::Plane { frame: g }) => same_frame(f, g),
            (
                Surface::Cylinder {
                    frame: f,
                    radius: r,
                },
                Surface::Cylinder {
                    frame: g,
                    radius: s,
                },
            )
            | (
                Surface::Sphere {
                    frame: f,
                    radius: r,
                },
                Surface::Sphere {
                    frame: g,
                    radius: s,
                },
            ) => same_frame(f, g) && same(*r, *s),
            (
                Surface::Cone {
                    frame: f,
                    radius: r,
                    half_angle: h,
                },
                Surface::Cone {
                    frame: g,
                    radius: s,
                    half_angle: k,
                },
            ) => same_frame(f, g) && same(*r, *s) && same(*h, *k),
            (
                Surface::Torus {
                    frame: f,
                    major_radius: r,
                    minor_radius: q,
                },
                Surface::Torus {
                    frame: g,
                    major_radius: s,
                    minor_radius: t,
                },
            )
            | (
                Surface::EllipticCylinder {
                    frame: f,
                    major_radius: r,
                    minor_radius: q,
                },
                Surface::EllipticCylinder {
                    frame: g,
                    major_radius: s,
                    minor_radius: t,
                },
            ) => same_frame(f, g) && same(*r, *s) && same(*q, *t),
            (Surface::Nurbs(n), Surface::Nurbs(m)) => n == m,
            _ => false,
        }
    }

    fn same_curve(a: &Curve, b: &Curve) -> bool {
        match (a, b) {
            (
                Curve::Line {
                    origin: o,
                    direction: d,
                },
                Curve::Line {
                    origin: p,
                    direction: e,
                },
            ) => {
                let scale = o.coords.norm().max(1.0);
                is_negligible((o - p).norm(), scale)
                    && is_negligible((d.into_inner() - e.into_inner()).norm(), 1.0)
            }
            (
                Curve::Circle {
                    frame: f,
                    radius: r,
                },
                Curve::Circle {
                    frame: g,
                    radius: s,
                },
            ) => same_frame(f, g) && same(*r, *s),
            (
                Curve::Ellipse {
                    frame: f,
                    major_radius: r,
                    minor_radius: q,
                },
                Curve::Ellipse {
                    frame: g,
                    major_radius: s,
                    minor_radius: t,
                },
            ) => same_frame(f, g) && same(*r, *s) && same(*q, *t),
            (Curve::Nurbs(n), Curve::Nurbs(m)) => n == m,
            _ => false,
        }
    }

    #[test]
    fn the_writers_own_geometry_maps_back_equal() {
        use arris_debug::{corpus, fixtures, sample};
        use arris_topo::Model;
        let mut kinds = (
            std::collections::BTreeSet::new(),
            std::collections::BTreeSet::new(),
        );
        let mut cases: Vec<(Model, arris_topo::Body)> = Vec::new();
        for name in [
            "primitive/box",
            "sweep/revolve-frustum",
            "sweep/revolve-ring",
            "sweep/extrude-ellipse",
            "boolean/ball-ball-common",
            "boolean/cross-cylinders-fuse",
        ] {
            let chain = corpus::chain(&fixtures::corpus_root().join(name), "default").unwrap();
            let body = chain.result().unwrap();
            cases.push((chain.model, body));
        }
        let mut m = Model::new(Precision::DEFAULT).unwrap();
        let body = sample::cuboid_nurbs(
            &mut m,
            Point3::new(-1.0, 0.0, 2.0),
            Point3::new(3.0, 2.0, 5.0),
        )
        .unwrap();
        cases.push((m, body));
        for (m, body) in &cases {
            let text = crate::step::write(m, &[*body]).unwrap();
            let x = part21::parse(&text).unwrap();
            let closure = m.closure(*body).unwrap();
            let surfaces: Vec<Surface> = closure
                .faces
                .iter()
                .map(|&f| m.surface(m.face(f).unwrap().surface()).unwrap().clone())
                .collect();
            // As the writer puts them: a periodic NURBS edge on its own
            // piece over its range.
            let curves: Vec<Curve> = closure
                .edges
                .iter()
                .filter_map(|&e| m.edge(e).unwrap().curve())
                .map(|(c, range)| match m.curve(c).unwrap() {
                    Curve::Nurbs(n) if n.period().is_some() => {
                        Curve::Nurbs(n.segment(range).unwrap())
                    }
                    c => c.clone(),
                })
                .collect();
            let geometry = Geometry {
                entities: Entities::new(&x.instances),
                units: Units::of_context(
                    &Entities::new(&x.instances),
                    super::super::units::context_of(
                        &x.instances,
                        "ADVANCED_BREP_SHAPE_REPRESENTATION",
                    )
                    .unwrap(),
                    super::super::LengthUnit::Millimetre,
                )
                .unwrap(),
            };
            for (&id, instance) in &x.instances {
                if let Some(face) = instance.record("ADVANCED_FACE") {
                    let Param::Ref(s) = face.params[2] else {
                        panic!()
                    };
                    let read = geometry.surface(id, s).unwrap();
                    assert!(!read.reversed);
                    let read = exact_surface(&read);
                    kinds.0.insert(read.kind());
                    assert!(
                        surfaces.iter().any(|s| same_surface(s, read)),
                        "#{s}: {read:?}"
                    );
                }
                if let Some(edge) = instance.record("EDGE_CURVE") {
                    let Param::Ref(c) = edge.params[3] else {
                        panic!()
                    };
                    let read = geometry.curve(id, c).unwrap();
                    assert!(!read.reversed);
                    let read = exact_curve(&read);
                    kinds.1.insert(read.kind());
                    assert!(curves.iter().any(|c| same_curve(c, read)), "#{c}: {read:?}");
                }
            }
        }
        assert_eq!(kinds.0.len(), 7, "every surface kind: {:?}", kinds.0);
        assert_eq!(
            kinds.1,
            [
                CurveKind::Line,
                CurveKind::Circle,
                CurveKind::Ellipse,
                CurveKind::Nurbs
            ]
            .into(),
        );
    }
}
