//! The `geometry` fixture kind: named analytic surfaces and curves with
//! parameters to evaluate, points to project and pairs to intersect, and
//! the oracle's answer for each (`tests/fixtures/README.md`, "kind":
//! "geometry"). [`build_surface`] and [`build_curve`] turn a spec into
//! the `arris-geom` value the fixture describes; [`build_profile`] does
//! the same for the `profile` step of a *solid* recipe, whose loops and
//! segments are one-to-one with `geom::Profile`'s.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use arris_geom::profile::{Profile, ProfileLoop, ProfileSegment};
use arris_geom::{Curve, GeomError, NurbsCurve, NurbsSurface, Surface};
use arris_math::{Frame, FrameError, Point2, Point3, Precision, UnitVec3, Vec2, Vec3};
use serde::{Deserialize, Serialize};

use super::{ExprError, FixtureError, Loop, Num, Plane, Segment, read_json, recipe_hash};

/// A placed surface, as written in `fixture.json`. `origin`, `z` and `x`
/// are the frame as `Frame::new` builds it (`x` made perpendicular to
/// `z`), the oracle's `gp_Ax3(origin, z, x)`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum SurfaceSpec {
    /// [`Surface::Plane`].
    Plane {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`.
        z: [Num; 3],
        /// Frame `x` hint.
        x: [Num; 3],
    },
    /// [`Surface::Cylinder`].
    Cylinder {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`.
        z: [Num; 3],
        /// Frame `x` hint.
        x: [Num; 3],
        /// `R`.
        radius: Num,
    },
    /// [`Surface::EllipticCylinder`], which Open CASCADE has no analytic
    /// surface for: the oracle builds it as the linear extrusion of the
    /// section ellipse along the frame's `z`, which is the same point
    /// set, and meets a curve with it through the general intersector.
    #[serde(rename = "elliptic_cylinder")]
    EllipticCylinder {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`, the axis.
        z: [Num; 3],
        /// Frame `x` hint: the section's major axis.
        x: [Num; 3],
        /// `a`, along `X`.
        major_radius: Num,
        /// `b`, along `Y`.
        minor_radius: Num,
    },
    /// [`Surface::Cone`], with the half-angle in degrees as every angle
    /// in a recipe.
    Cone {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`.
        z: [Num; 3],
        /// Frame `x` hint.
        x: [Num; 3],
        /// `R` at `v = 0`.
        radius: Num,
        /// `α` in degrees.
        half_angle_deg: Num,
    },
    /// [`Surface::Sphere`].
    Sphere {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`.
        z: [Num; 3],
        /// Frame `x` hint.
        x: [Num; 3],
        /// `R`.
        radius: Num,
    },
    /// [`Surface::Torus`].
    Torus {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`.
        z: [Num; 3],
        /// Frame `x` hint.
        x: [Num; 3],
        /// `R`.
        major_radius: Num,
        /// `r`.
        minor_radius: Num,
    },
    /// [`Surface::Nurbs`], as [`NurbsSurface::new`] takes it: the oracle's
    /// `Geom_BSplineSurface` over the same knots, each written as often as
    /// it repeats, so the two share both parameters. It is evaluated and
    /// projected onto, never paired.
    Nurbs {
        /// `[p, q]`.
        degree: [usize; 2],
        /// The knots of `u` and of `v`.
        knots: [Vec<Num>; 2],
        /// The control net, Cartesian: one row per control point of `u`,
        /// each with a point per control point of `v`.
        control_points: Vec<Vec<[Num; 3]>>,
        /// The weights, in the net's shape.
        weights: Vec<Vec<Num>>,
    },
}

/// A placed curve, as written in `fixture.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum CurveSpec {
    /// [`Curve::Line`].
    Line {
        /// A point on the line.
        origin: [Num; 3],
        /// Its direction, normalised by the interpreter.
        direction: [Num; 3],
    },
    /// [`Curve::Circle`].
    Circle {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`.
        z: [Num; 3],
        /// Frame `x` hint.
        x: [Num; 3],
        /// `R`.
        radius: Num,
    },
    /// [`Curve::Ellipse`].
    Ellipse {
        /// Frame origin.
        origin: [Num; 3],
        /// Frame `z`.
        z: [Num; 3],
        /// Frame `x` hint: the major axis.
        x: [Num; 3],
        /// `a`.
        major_radius: Num,
        /// `b`.
        minor_radius: Num,
    },
    /// [`Curve::Nurbs`], as [`NurbsCurve::new`] takes it: the oracle's
    /// `Geom_BSplineCurve` over the same knots, each written as often as
    /// it repeats, so the two share a parameter.
    Nurbs {
        /// `p`.
        degree: usize,
        /// The `n + p + 1` knots.
        knots: Vec<Num>,
        /// The `n` control points, Cartesian.
        control_points: Vec<[Num; 3]>,
        /// The `n` weights.
        weights: Vec<Num>,
    },
}

/// A parameter to evaluate at: `[u, v]` for a surface, `t` for a curve.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ParamSpec {
    /// A surface's `(u, v)`.
    Surface([Num; 2]),
    /// A curve's `t`.
    Curve(Num),
}

/// What to ask of one named surface or curve.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Sample {
    /// The surface or curve.
    pub of: String,
    /// Parameters to evaluate at.
    #[serde(default)]
    pub params: Vec<ParamSpec>,
    /// Points to project.
    #[serde(default)]
    pub points: Vec<[Num; 3]>,
}

/// Two names to intersect: two surfaces, a curve `a` against a surface
/// `b`, or two curves; or, with `extrema`, two curves, or a curve and a
/// surface, whose distance's stationary pairs are asked for instead.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pair {
    /// A surface or a curve.
    pub a: String,
    /// A surface, or a curve when `a` is one.
    pub b: String,
    /// The stationary pairs of the distance between two curves, or a curve
    /// and a surface, not their intersection.
    #[serde(default, skip_serializing_if = "core::ops::Not::not")]
    pub extrema: bool,
}

/// A `fixture.json` of the geometry kind.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeomRecipe {
    /// `"geometry"`.
    pub kind: String,
    /// What the fixture is for.
    #[serde(default)]
    pub description: String,
    /// Named numbers the specs may use in expressions.
    #[serde(default)]
    pub params: BTreeMap<String, f64>,
    /// The surfaces by name.
    #[serde(default)]
    pub surfaces: BTreeMap<String, SurfaceSpec>,
    /// The curves by name.
    #[serde(default)]
    pub curves: BTreeMap<String, CurveSpec>,
    /// Evaluations and projections.
    #[serde(default)]
    pub samples: Vec<Sample>,
    /// Intersections.
    #[serde(default)]
    pub pairs: Vec<Pair>,
}

/// Why a spec did not build.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BuildError {
    /// A number did not evaluate.
    #[error("{name}: {source}")]
    Expr {
        /// The spec.
        name: String,
        /// The cause.
        source: ExprError,
    },
    /// The frame is degenerate.
    #[error("{name}: frame: {source}")]
    Frame {
        /// The spec.
        name: String,
        /// The cause.
        source: FrameError,
    },
    /// A direction is zero.
    #[error("{name}: zero direction")]
    ZeroDirection {
        /// The spec.
        name: String,
    },
    /// A NURBS curve's degree, knots, control points and weights are no
    /// curve.
    #[error("{name}: {source}")]
    Geometry {
        /// The spec.
        name: String,
        /// The cause.
        source: GeomError,
    },
    /// A profile plane's `x` and `y` are not orthogonal, as the oracle's
    /// `recipe.py` also refuses (by the same tolerance,
    /// `Precision::DEFAULT.angular_tolerance`).
    #[error("{name}: plane x and y are not orthogonal")]
    NotOrthogonal {
        /// The spec.
        name: String,
    },
}

fn num(name: &str, n: &Num, params: &BTreeMap<String, f64>) -> Result<f64, BuildError> {
    n.eval(params).map_err(|source| BuildError::Expr {
        name: name.to_string(),
        source,
    })
}

fn vec3(name: &str, v: &[Num; 3], params: &BTreeMap<String, f64>) -> Result<Vec3, BuildError> {
    Ok(Vec3::new(
        num(name, &v[0], params)?,
        num(name, &v[1], params)?,
        num(name, &v[2], params)?,
    ))
}

fn frame(
    name: &str,
    origin: &[Num; 3],
    z: &[Num; 3],
    x: &[Num; 3],
    params: &BTreeMap<String, f64>,
) -> Result<Frame, BuildError> {
    Frame::new(
        Point3::from(vec3(name, origin, params)?),
        vec3(name, z, params)?,
        vec3(name, x, params)?,
    )
    .map_err(|source| BuildError::Frame {
        name: name.to_string(),
        source,
    })
}

fn point2(name: &str, p: &[Num; 2], params: &BTreeMap<String, f64>) -> Result<Point2, BuildError> {
    Ok(Point2::new(
        num(name, &p[0], params)?,
        num(name, &p[1], params)?,
    ))
}

/// The [`ProfileLoop`] a recipe loop describes, under `params`.
fn build_loop(
    name: &str,
    spec: &Loop,
    params: &BTreeMap<String, f64>,
) -> Result<ProfileLoop, BuildError> {
    Ok(match spec {
        Loop::Circle { circle } => ProfileLoop::Circle {
            center: point2(name, &circle.center, params)?,
            radius: num(name, &circle.radius, params)?,
        },
        Loop::Ellipse { ellipse } => ProfileLoop::Ellipse {
            center: point2(name, &ellipse.center, params)?,
            major: point2(name, &ellipse.major, params)?.coords,
            minor_radius: num(name, &ellipse.minor_radius, params)?,
        },
        Loop::Path { start, segments } => ProfileLoop::Path {
            start: point2(name, start, params)?,
            segments: segments
                .iter()
                .map(|s| {
                    Ok(match s {
                        Segment::Line { line_to } => {
                            ProfileSegment::LineTo(point2(name, line_to, params)?)
                        }
                        Segment::Arc { arc_to, via } => ProfileSegment::ArcTo {
                            to: point2(name, arc_to, params)?,
                            via: point2(name, via, params)?,
                        },
                        Segment::Ellipse {
                            ellipse_to,
                            center,
                            major,
                            minor_radius,
                            ccw,
                        } => ProfileSegment::EllipseTo {
                            to: point2(name, ellipse_to, params)?,
                            center: point2(name, center, params)?,
                            major: Vec2::from(point2(name, major, params)?.coords),
                            minor_radius: num(name, minor_radius, params)?,
                            ccw: *ccw,
                        },
                    })
                })
                .collect::<Result<Vec<_>, BuildError>>()?,
        },
    })
}

/// The [`Profile`] a recipe's `profile` step describes, under `params`:
/// the plane by its origin and its two in-plane axes (`z = x × y`, as
/// `Frame::new` and the oracle's `gp_Ax3` build it), and the loops
/// one-to-one with [`ProfileLoop`] and [`ProfileSegment`] — the recipe
/// grammar of `tests/fixtures/README.md` *is* the profile grammar.
/// Validation is `Profile::edges`'s, not this function's.
///
/// ```no_run
/// use arris_debug::fixtures::{self, geom::build_profile};
/// use std::collections::BTreeMap;
///
/// let dir = fixtures::corpus_root().join("sweep/extrude-plate-with-hole");
/// let fixture = fixtures::load(&dir).unwrap();
/// let params = fixture.recipe.params_of("default").unwrap();
/// let fixtures::Step::Profile { plane, outer, holes, .. } = &fixture.recipe.steps[0] else {
///     panic!("the first step is the sketch")
/// };
/// let profile = build_profile("sketch", plane, outer, holes, &params).unwrap();
/// assert_eq!(profile.holes.len(), 1);
/// ```
pub fn build_profile(
    name: &str,
    plane: &Plane,
    outer: &Loop,
    holes: &[Loop],
    params: &BTreeMap<String, f64>,
) -> Result<Profile, BuildError> {
    let (x, y) = (vec3(name, &plane.x, params)?, vec3(name, &plane.y, params)?);
    if let (Some(ux), Some(uy)) = (UnitVec3::try_new(x, 0.0), UnitVec3::try_new(y, 0.0)) {
        if ux.dot(&uy).abs() > Precision::DEFAULT.angular_tolerance {
            return Err(BuildError::NotOrthogonal {
                name: name.to_string(),
            });
        }
    }
    let frame = Frame::new(
        Point3::from(vec3(name, &plane.origin, params)?),
        x.cross(&y),
        x,
    )
    .map_err(|source| BuildError::Frame {
        name: name.to_string(),
        source,
    })?;
    Ok(Profile {
        plane: frame,
        outer: build_loop(name, outer, params)?,
        holes: holes
            .iter()
            .map(|h| build_loop(name, h, params))
            .collect::<Result<Vec<_>, BuildError>>()?,
    })
}

/// The [`Surface`] a spec describes, under `params`.
pub fn build_surface(
    name: &str,
    spec: &SurfaceSpec,
    params: &BTreeMap<String, f64>,
) -> Result<Surface, BuildError> {
    Ok(match spec {
        SurfaceSpec::Plane { origin, z, x } => Surface::Plane {
            frame: frame(name, origin, z, x, params)?,
        },
        SurfaceSpec::Cylinder {
            origin,
            z,
            x,
            radius,
        } => Surface::Cylinder {
            frame: frame(name, origin, z, x, params)?,
            radius: num(name, radius, params)?,
        },
        SurfaceSpec::EllipticCylinder {
            origin,
            z,
            x,
            major_radius,
            minor_radius,
        } => Surface::EllipticCylinder {
            frame: frame(name, origin, z, x, params)?,
            major_radius: num(name, major_radius, params)?,
            minor_radius: num(name, minor_radius, params)?,
        },
        SurfaceSpec::Cone {
            origin,
            z,
            x,
            radius,
            half_angle_deg,
        } => Surface::Cone {
            frame: frame(name, origin, z, x, params)?,
            radius: num(name, radius, params)?,
            half_angle: num(name, half_angle_deg, params)?.to_radians(),
        },
        SurfaceSpec::Sphere {
            origin,
            z,
            x,
            radius,
        } => Surface::Sphere {
            frame: frame(name, origin, z, x, params)?,
            radius: num(name, radius, params)?,
        },
        SurfaceSpec::Torus {
            origin,
            z,
            x,
            major_radius,
            minor_radius,
        } => Surface::Torus {
            frame: frame(name, origin, z, x, params)?,
            major_radius: num(name, major_radius, params)?,
            minor_radius: num(name, minor_radius, params)?,
        },
        SurfaceSpec::Nurbs {
            degree,
            knots,
            control_points,
            weights,
        } => {
            let geometry = |source| BuildError::Geometry {
                name: name.to_string(),
                source,
            };
            let numbers = |of: &[Num]| {
                of.iter()
                    .map(|n| num(name, n, params))
                    .collect::<Result<Vec<f64>, _>>()
            };
            let mut points = Vec::new();
            for row in control_points {
                for p in row {
                    points.push(Point3::from(vec3(name, p, params)?));
                }
            }
            let mut flat_weights = Vec::new();
            for row in weights {
                flat_weights.extend(numbers(row)?);
            }
            Surface::Nurbs(
                NurbsSurface::new(
                    *degree,
                    [numbers(&knots[0])?, numbers(&knots[1])?],
                    points,
                    flat_weights,
                )
                .map_err(geometry)?,
            )
        }
    })
}

/// The [`Curve`] a spec describes, under `params`.
pub fn build_curve(
    name: &str,
    spec: &CurveSpec,
    params: &BTreeMap<String, f64>,
) -> Result<Curve, BuildError> {
    Ok(match spec {
        CurveSpec::Line { origin, direction } => Curve::Line {
            origin: Point3::from(vec3(name, origin, params)?),
            direction: UnitVec3::try_new(vec3(name, direction, params)?, 0.0).ok_or_else(|| {
                BuildError::ZeroDirection {
                    name: name.to_string(),
                }
            })?,
        },
        CurveSpec::Circle {
            origin,
            z,
            x,
            radius,
        } => Curve::Circle {
            frame: frame(name, origin, z, x, params)?,
            radius: num(name, radius, params)?,
        },
        CurveSpec::Ellipse {
            origin,
            z,
            x,
            major_radius,
            minor_radius,
        } => Curve::Ellipse {
            frame: frame(name, origin, z, x, params)?,
            major_radius: num(name, major_radius, params)?,
            minor_radius: num(name, minor_radius, params)?,
        },
        CurveSpec::Nurbs {
            degree,
            knots,
            control_points,
            weights,
        } => {
            let numbers = |of: &[Num]| {
                of.iter()
                    .map(|n| num(name, n, params))
                    .collect::<Result<Vec<f64>, _>>()
            };
            let points = control_points
                .iter()
                .map(|p| vec3(name, p, params).map(Point3::from))
                .collect::<Result<Vec<Point3>, _>>()?;
            Curve::Nurbs(
                NurbsCurve::new(*degree, numbers(knots)?, points, numbers(weights)?).map_err(
                    |source| BuildError::Geometry {
                        name: name.to_string(),
                        source,
                    },
                )?,
            )
        }
    })
}

/// One evaluation by the oracle: `Geom_Surface::D2` or `Geom_Curve::D2`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Evaluation {
    /// A surface at `(u, v)`.
    Surface {
        /// `[u, v]`.
        at: [f64; 2],
        /// `P`.
        point: [f64; 3],
        /// `∂P/∂u`.
        du: [f64; 3],
        /// `∂P/∂v`.
        dv: [f64; 3],
        /// `∂²P/∂u²`.
        duu: [f64; 3],
        /// `∂²P/∂u∂v`.
        duv: [f64; 3],
        /// `∂²P/∂v²`.
        dvv: [f64; 3],
    },
    /// A curve at `t`.
    Curve {
        /// `t`.
        at: f64,
        /// `P`.
        point: [f64; 3],
        /// `dP/dt`.
        d1: [f64; 3],
        /// `d²P/dt²`.
        d2: [f64; 3],
    },
}

/// One projection by the oracle: `GeomAPI_ProjectPointOnSurf` or
/// `OnCurve`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Projection {
    /// The point projected.
    pub point: [f64; 3],
    /// `(u, v)` of the nearest surface point.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uv: Option<[f64; 2]>,
    /// `t` of the nearest curve point.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub t: Option<f64>,
    /// The nearest point.
    pub nearest: [f64; 3],
    /// Its distance from the query.
    pub distance: f64,
}

/// The oracle's answers for one sample.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SampleResult {
    /// The surface or curve.
    pub of: String,
    /// One per `params` entry, in order.
    pub evaluations: Vec<Evaluation>,
    /// One per `points` entry, in order.
    pub projections: Vec<Projection>,
}

/// A result curve of a surface pair, sampled.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CurveSample {
    /// `line`, `circle` or `ellipse`.
    #[serde(rename = "type")]
    pub kind: String,
    /// Points on it.
    pub points: Vec<[f64; 3]>,
}

/// A hit of a curve against a surface, or of two curves.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Hit {
    /// The point: the first curve's, for two curves.
    pub point: [f64; 3],
    /// The (first) curve's parameter.
    pub t: f64,
    /// The second curve's parameter, for two curves.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tb: Option<f64>,
}

/// One stationary pair of the distance between two curves, as
/// `Extrema_ExtCC` reports it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExtremumResult {
    /// The first curve's parameter.
    pub ta: f64,
    /// The second curve's parameter.
    pub tb: f64,
    /// The first curve's point.
    pub a: [f64; 3],
    /// The second curve's point.
    pub b: [f64; 3],
    /// The distance between them.
    pub distance: f64,
}

/// One stationary pair of the distance between a curve and a surface, as
/// `GeomAPI_ExtremaCurveSurface` reports it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CurveSurfaceResult {
    /// The curve's parameter.
    pub t: f64,
    /// The surface's `u`.
    pub u: f64,
    /// The surface's `v`.
    pub v: f64,
    /// The curve's point.
    pub a: [f64; 3],
    /// The surface's point.
    pub b: [f64; 3],
    /// The distance between them.
    pub distance: f64,
}

/// The oracle's answer for one pair: `IntAna_QuadQuadGeo` for two
/// surfaces (`empty`, `coincident`, or the result curves' kind with the
/// curves sampled), `IntAna_IntConicQuad` for a conic against a plane or
/// a quadric and `GeomAPI_IntCS` where it has no form — a NURBS curve,
/// and a conic against a torus or an elliptic cylinder — (`coincident`,
/// or `points` with the hits, duplicates within `Precision::Confusion`
/// reported once and hits off either operand dropped and counted),
/// `GeomAPI_ExtremaCurveCurve` for two curves (`points`, the extrema
/// within `Precision::Confusion`); for an extrema pair `Extrema_ExtCC`
/// (`parallel` with the distance, or `extrema` with the stationary pairs
/// it found, which need not be all of them); for a curve against a surface
/// `GeomAPI_ExtremaCurveSurface` in the same two forms.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PairResult {
    /// The first name.
    pub a: String,
    /// The second name.
    pub b: String,
    /// `empty`, `coincident`, `point`, `line`, `circle`, `ellipse`,
    /// `unsolved` (no conic: `IntAna_NoGeometricSolution`) or `points`;
    /// `extrema` or `parallel` for an extrema pair.
    #[serde(rename = "type")]
    pub kind: String,
    /// The result curves of a surface pair.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub curves: Vec<CurveSample>,
    /// The isolated points of a surface pair that meets in points.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub points: Vec<[f64; 3]>,
    /// The hits of a curve against a surface or of two curves.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hits: Vec<Hit>,
    /// What the oracle reported and then dropped: hits it found off an
    /// operand, or, for an `unsolved` surface pair, the lines it walked
    /// along a tangency, whose samples it has no crossing to polish onto
    /// — a curve the pair is tangent along is then the walk's, not a
    /// missing one of Arris's.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dropped: Option<usize>,
    /// The stationary pairs of an `extrema` result.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub extrema: Vec<ExtremumResult>,
    /// The stationary pairs of a curve against a surface.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub curve_surface: Vec<CurveSurfaceResult>,
    /// The distance of a `parallel` result, the same along the family.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance: Option<f64>,
}

/// An `expected.json` of the geometry kind.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeomExpected {
    /// The `cadquery-ocp` version that wrote it.
    pub occt: String,
    /// The recipe hash it was computed from.
    pub recipe_sha256: String,
    /// `"geometry"`.
    pub kind: String,
    /// One per recipe sample, in order.
    pub samples: Vec<SampleResult>,
    /// One per recipe pair, in order.
    pub pairs: Vec<PairResult>,
}

/// A loaded geometry fixture directory.
#[derive(Debug, Clone, PartialEq)]
pub struct GeomFixture {
    /// The directory.
    pub dir: PathBuf,
    /// `<area>/<slug>`.
    pub name: String,
    /// The recipe.
    pub recipe: GeomRecipe,
    /// The hash of the recipe as loaded.
    pub recipe_sha256: String,
    /// The oracle's answer.
    pub expected: GeomExpected,
}

/// Loads a geometry fixture directory: both files and the recipe's hash.
pub fn load(dir: &Path) -> Result<GeomFixture, FixtureError> {
    let raw: serde_json::Value = read_json(&dir.join("fixture.json"))?;
    let recipe: GeomRecipe =
        serde_json::from_value(raw.clone()).map_err(|source| FixtureError::Json {
            path: dir.join("fixture.json"),
            source,
        })?;
    let expected: GeomExpected = read_json(&dir.join("expected.json"))?;
    let recipe_sha256 = recipe_hash(&raw).map_err(|kind| FixtureError::UnknownKind {
        path: dir.join("fixture.json"),
        kind,
    })?;
    Ok(GeomFixture {
        dir: dir.to_path_buf(),
        name: super::name_of(dir),
        recipe_sha256,
        recipe,
        expected,
    })
}

/// The corpus lint for a geometry directory: both files present and
/// parseable, the hash matches, every sample and pair names a surface or
/// curve of the recipe, every spec builds, and the oracle's answer has
/// one result per sample and pair with the counts the recipe asked for.
/// The values themselves are `crates/arris-geom/tests/oracle.rs`'s to
/// compare.
pub fn lint(dir: &Path) -> Vec<String> {
    let mut problems = Vec::new();
    let fixture = match load(dir) {
        Ok(f) => f,
        Err(e) => return vec![format!("{}: {e}", dir.display())],
    };
    let name = &fixture.name;
    let mut problem = |text: String| problems.push(format!("{name}: {text}"));
    let r = &fixture.recipe;
    let x = &fixture.expected;
    if x.recipe_sha256 != fixture.recipe_sha256 {
        problem(format!(
            "expected.json is stale: recipe hash {} but the recipe hashes to {} — rerun tools/oracle/expected.py",
            x.recipe_sha256, fixture.recipe_sha256
        ));
    }
    if x.kind != "geometry" {
        problem(format!(
            "expected.json kind is {:?}, not \"geometry\"",
            x.kind
        ));
    }
    for (n, spec) in &r.surfaces {
        if let Err(e) = build_surface(n, spec, &r.params) {
            problem(format!("surface {e}"));
        }
        if r.curves.contains_key(n) {
            problem(format!("{n:?} is both a surface and a curve"));
        }
    }
    for (n, spec) in &r.curves {
        if let Err(e) = build_curve(n, spec, &r.params) {
            problem(format!("curve {e}"));
        }
    }
    let known = |n: &str| r.surfaces.contains_key(n) || r.curves.contains_key(n);
    if x.samples.len() != r.samples.len() {
        problem(format!(
            "{} samples in the recipe, {} results",
            r.samples.len(),
            x.samples.len()
        ));
    }
    for (i, s) in r.samples.iter().enumerate() {
        if !known(&s.of) {
            problem(format!("sample {i} names unknown {:?}", s.of));
        }
        if let Some(res) = x.samples.get(i) {
            if res.of != s.of
                || res.evaluations.len() != s.params.len()
                || res.projections.len() != s.points.len()
            {
                problem(format!(
                    "sample {i} ({:?}) asks {} evaluations and {} projections; the result has {} and {} for {:?}",
                    s.of,
                    s.params.len(),
                    s.points.len(),
                    res.evaluations.len(),
                    res.projections.len(),
                    res.of
                ));
            }
        }
    }
    if x.pairs.len() != r.pairs.len() {
        problem(format!(
            "{} pairs in the recipe, {} results",
            r.pairs.len(),
            x.pairs.len()
        ));
    }
    for (i, p) in r.pairs.iter().enumerate() {
        let two_curves = r.curves.contains_key(&p.a) && r.curves.contains_key(&p.b);
        let curve_surface = r.curves.contains_key(&p.a) && r.surfaces.contains_key(&p.b);
        if p.extrema && !(two_curves || curve_surface) {
            problem(format!(
                "pair {i} ({:?}, {:?}) asks for extrema, which are of two curves or a curve and a surface",
                p.a, p.b
            ));
        }
        if !(two_curves || known(&p.a) && r.surfaces.contains_key(&p.b)) {
            problem(format!(
                "pair {i} ({:?}, {:?}) is not a surface or curve against a surface, or two curves",
                p.a, p.b
            ));
        }
        if let Some(res) = x.pairs.get(i) {
            if p.extrema != matches!(res.kind.as_str(), "extrema" | "parallel") {
                problem(format!(
                    "pair {i} ({:?}, {:?}): the recipe {} extrema and the result is {:?}",
                    p.a,
                    p.b,
                    if p.extrema {
                        "asks for"
                    } else {
                        "does not ask for"
                    },
                    res.kind
                ));
            }
            if res.a != p.a || res.b != p.b {
                problem(format!(
                    "pair {i} is ({:?}, {:?}) but the result is ({:?}, {:?})",
                    p.a, p.b, res.a, res.b
                ));
            }
        }
    }
    problems
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle_loop() -> Loop {
        serde_json::from_str(r#"{"circle": {"center": [0, 0], "radius": 1}}"#).unwrap()
    }

    #[test]
    fn a_non_orthogonal_plane_is_refused_as_the_oracle_refuses_it() {
        let plane: Plane =
            serde_json::from_str(r#"{"origin": [0, 0, 0], "x": [1, 0, 0], "y": [1, 1, 0]}"#)
                .unwrap();
        let params = BTreeMap::new();
        assert!(matches!(
            build_profile("p", &plane, &circle_loop(), &[], &params),
            Err(BuildError::NotOrthogonal { .. })
        ));
    }

    #[test]
    fn an_orthogonal_plane_builds() {
        let plane: Plane =
            serde_json::from_str(r#"{"origin": [0, 0, 0], "x": [1, 0, 0], "y": [0, 1, 0]}"#)
                .unwrap();
        let params = BTreeMap::new();
        assert!(build_profile("p", &plane, &circle_loop(), &[], &params).is_ok());
    }
}
