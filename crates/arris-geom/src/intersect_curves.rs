//! Curve–curve intersection: the closed-form table.
//!
//! Two curves meet in points, and every pair a boolean needs is decided
//! through a plane one of them already lies in: a conic's own plane
//! (ADR-0004), or for a NURBS curve against a line two planes through
//! the line (ADR-0018). The one pair with no plane to use — two lines —
//! has a closed form of its own, and the coplanar cases, where the plane
//! says nothing, have theirs.

use arris_math::{Frame, Meter, Point2, Point3, Tolerance, Vec2, Vec3, roots, wrap_angle};

use crate::conic2::{Conic2, ConicMeet, conic_pair};
use crate::{
    Curve, CurveKind, CurveSurfaceHit, CurveSurfaceIntersection, GeomError, GeomKind, NurbsCurve,
    Surface, intersect_curve_surface,
};

/// One point two curves have in common.
///
/// `point` is `a.point(ta)`, and `b.point(tb)` is within `tol.linear` of
/// it. A periodic parameter is in `[0, 2π)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveCurveHit {
    /// The first curve's parameter.
    pub ta: f64,
    /// The second curve's parameter.
    pub tb: f64,
    /// `a.point(ta)`.
    pub point: Point3,
    /// `true` when the curves touch here without crossing: the first
    /// curve stays on one side of the second within `tol.linear`. Two
    /// crossings that close are one touch.
    pub tangent: bool,
}

/// What two curves have in common.
#[derive(Debug, Clone, PartialEq)]
pub enum CurveIntersection {
    /// The hits, ascending by `ta`; empty when the curves miss each
    /// other.
    Points(Vec<CurveCurveHit>),
    /// The curves are the same curve within the tolerance — the same
    /// line, or the same circle — and there is no point to return.
    Coincident,
}

/// The intersection of two curves, by the case table: every
/// pair with a closed form is computed exactly, every other pair is an
/// explicit [`GeomError::Unsupported`] arm — no wildcard, no marcher.
///
/// Guarantees: hits are sorted by `ta`, each parameter is in its curve's
/// domain, each hit lies on both curves within `tol.linear`, and the
/// result is deterministic bit for bit.
///
/// The table. Two lines are the closed form: parallel within
/// `tol.angular` gives `Coincident` or nothing by the distance between
/// them, and otherwise the nearest approach is a hit when it is shorter
/// than `tol.linear`. Every other supported pair has a conic operand, and
/// goes through *that conic's plane*: the other curve meets the plane at
/// points ([`intersect_curve_surface`]), and a point is a hit when the
/// conic's own projection of it is within `tol.linear`, which gives the
/// conic's parameter with it. A curve the plane reports `Coincident` with
/// is the coplanar case, where the plane decides nothing and each pair
/// has its own form: a line against a circle or an ellipse is the conic
/// against the plane through the line perpendicular to the conic's — the
/// same points, and the tangency decided in the linear tolerance by an
/// arm that already exists — and two coplanar circles are the radical
/// line. A coplanar pair with an ellipse in it is the quartic of
/// `crate::conic2`, the second conic's implicit form along the first:
/// `Coincident` when the first lies within `tol.linear` of the second
/// everywhere — which is what two booleans in a row make — and
/// otherwise each extremum of the residual within `tol.linear` a touch
/// and each sign change between two of them a crossing.
///
/// A **NURBS curve** (ADR-0018) goes through the same planes, its hits on
/// them from [`intersect_curve_surface`]'s NURBS arm. Against a circle or
/// an ellipse, through the conic's plane as above; a NURBS curve in that
/// plane meets the conic where it meets the cylinder the conic is the
/// section of — circular or elliptic, the conic's frame and radii — so
/// the touch is decided in `tol.linear` of length by that arm, and a
/// curve on the cylinder too is `Coincident`. Against a line, through two
/// planes that hold the line, square to each other: the hits on either
/// within `tol.linear` of the line, one per point, a crossing of one
/// plane before a touch of the other, since the curve is tangent to the
/// line only where it touches every plane through it; a curve in one of
/// the planes is the coplanar case, and one in both lies along the line,
/// `Coincident`. Two NURBS curves are `Unsupported`: that is a marcher's.
///
/// ```
/// use arris_geom::{Curve, CurveIntersection, intersect_curves};
/// use arris_math::{Frame, Meter, Point3, Precision, Vec3};
///
/// let circle = Curve::Circle { frame: Frame::world(), radius: 2.0 };
/// let axis = Curve::Line { origin: Point3::new(0.0, 0.0, -1.0), direction: Vec3::z_axis() };
/// // The axis pierces the circle's plane at its centre, which is not on it.
/// let CurveIntersection::Points(hits) =
///     intersect_curves(&axis, &circle, Precision::DEFAULT.tolerance(), &mut Meter::default())?
/// else { panic!() };
/// assert!(hits.is_empty());
/// assert_eq!(
///     intersect_curves(&circle, &circle, Precision::DEFAULT.tolerance(), &mut Meter::default())?,
///     CurveIntersection::Coincident
/// );
/// # Ok::<(), arris_geom::GeomError>(())
/// ```
pub fn intersect_curves(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveIntersection, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    match (a, b) {
        (
            &Curve::Line {
                origin: oa,
                direction: da,
            },
            &Curve::Line {
                origin: ob,
                direction: db,
            },
        ) => Ok(line_line(oa, da.into_inner(), ob, db.into_inner(), tol)),
        (
            Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. },
            Curve::Circle { .. } | Curve::Ellipse { .. },
        ) => through_plane(a, b, tol, meter),
        (Curve::Circle { .. } | Curve::Ellipse { .. }, Curve::Line { .. }) => {
            Ok(swapped(through_plane(b, a, tol, meter)?))
        }
        (Curve::Nurbs(_), &Curve::Line { origin, direction }) => {
            nurbs_line(a, origin, direction.into_inner(), tol, meter)
        }
        (&Curve::Line { origin, direction }, Curve::Nurbs(_)) => Ok(swapped(nurbs_line(
            b,
            origin,
            direction.into_inner(),
            tol,
            meter,
        )?)),
        (Curve::Nurbs(_), Curve::Circle { .. } | Curve::Ellipse { .. }) => {
            through_plane(a, b, tol, meter)
        }
        (Curve::Circle { .. } | Curve::Ellipse { .. }, Curve::Nurbs(_)) => {
            Ok(swapped(through_plane(b, a, tol, meter)?))
        }
        // Two fitted curves meet where a marcher finds them, the NURBS
        // cycle's; the
        // pave model reads the crossings of one pair's traced curves from
        // the tracer's points instead (ADR-0018).
        (Curve::Nurbs(_), Curve::Nurbs(_)) => Err(GeomError::Unsupported {
            a: GeomKind::Curve(a.kind()),
            b: GeomKind::Curve(b.kind()),
        }),
    }
}

/// Whether two curves are the same curve within `tol`: the
/// [`CurveIntersection::Coincident`] verdict of [`intersect_curves`],
/// without the common points of a pair that is not. A question such as
/// "does this edge lie along that section curve" needs only this, and it
/// has an answer for every pair of lines and conics.
///
/// Guarantees: wherever [`intersect_curves`] answers, this is `true`
/// exactly when that answer is `Coincident`, by the same arms. Two lines
/// coincide when parallel within `tol.angular` and within `tol.linear` of
/// each other; a line never coincides with a conic; two conics coincide
/// when the first lies in the second's plane and within `tol.linear` of
/// it everywhere, by the radical line for two circles and by the
/// conics' quartic where an ellipse is among them. A NURBS curve
/// coincides with a line or a conic
/// by [`intersect_curves`]' arms — it lies within `tol.linear` of both
/// planes through the line, or of the conic's plane and its cylinder —
/// and with a second NURBS curve when the two are the same spline,
/// every control point within `tol.linear` of its twin over the same
/// degree, knots and weights: the same section made twice. Two NURBS
/// curves are not the same when a point of either at an end, a knot or
/// halfway between two has no hit of the other within `tol.linear` on
/// the plane square to it there; any
/// other pair of them — one curve over two sets of knots, say — is
/// [`GeomError::Unsupported`] naming the pair, which only a marcher
/// could decide.
///
/// ```
/// use arris_geom::{Curve, CurveIntersection, curves_coincide, intersect_curves};
/// use arris_math::{Frame, Meter, Precision};
///
/// let tol = Precision::DEFAULT.tolerance();
/// let meter = &mut Meter::default();
/// let circle = Curve::Circle { frame: Frame::world(), radius: 1.0 };
/// let ellipse = Curve::Ellipse { frame: Frame::world(), major_radius: 3.0, minor_radius: 2.0 };
/// // Coplanar: the quartic says where they meet — nowhere, the circle
/// // lying inside the ellipse — and they are not the same curve.
/// let apart = intersect_curves(&circle, &ellipse, tol, meter)?;
/// assert_eq!(apart, CurveIntersection::Points(Vec::new()));
/// assert!(!curves_coincide(&circle, &ellipse, tol, meter)?);
/// assert!(curves_coincide(&ellipse, &ellipse, tol, meter)?);
/// # Ok::<(), arris_geom::GeomError>(())
/// ```
pub fn curves_coincide(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<bool, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    let unsupported = || GeomError::Unsupported {
        a: GeomKind::Curve(a.kind()),
        b: GeomKind::Curve(b.kind()),
    };
    match (a, b) {
        (
            &Curve::Line {
                origin: oa,
                direction: da,
            },
            &Curve::Line {
                origin: ob,
                direction: db,
            },
        ) => Ok(line_line(oa, da.into_inner(), ob, db.into_inner(), tol)
            == CurveIntersection::Coincident),
        (Curve::Line { .. }, Curve::Circle { .. } | Curve::Ellipse { .. })
        | (Curve::Circle { .. } | Curve::Ellipse { .. }, Curve::Line { .. }) => Ok(false),
        (
            Curve::Circle { .. } | Curve::Ellipse { .. },
            Curve::Circle { .. } | Curve::Ellipse { .. },
        ) => {
            let Some((fb, _)) = conic_frame(b) else {
                return Err(unsupported());
            };
            // Through `b`'s plane, as `through_plane`: a curve that meets
            // it at points is not in it, and is not `b`.
            let plane = Surface::Plane { frame: *fb };
            if !matches!(
                intersect_curve_surface(a, &plane, tol, meter)?,
                CurveSurfaceIntersection::Coincident
            ) {
                return Ok(false);
            }
            // In it, as `coplanar`: two circles by the radical line, a
            // pair with an ellipse by the conics' quartic.
            Ok(coplanar(a, b, tol, meter)? == CurveIntersection::Coincident)
        }
        (Curve::Nurbs(_), &Curve::Line { origin, direction })
        | (&Curve::Line { origin, direction }, Curve::Nurbs(_)) => {
            let spline = if let Curve::Nurbs(_) = a { a } else { b };
            Ok(
                nurbs_line(spline, origin, direction.into_inner(), tol, meter)?
                    == CurveIntersection::Coincident,
            )
        }
        (Curve::Nurbs(_), Curve::Circle { .. } | Curve::Ellipse { .. })
        | (Curve::Circle { .. } | Curve::Ellipse { .. }, Curve::Nurbs(_)) => {
            let (spline, conic) = if let Curve::Nurbs(_) = a {
                (a, b)
            } else {
                (b, a)
            };
            // As `through_plane` and then `coplanar` decide it: in the
            // conic's plane, and on the conic's cylinder there.
            let Some((frame, _)) = conic_frame(conic) else {
                return Err(unsupported());
            };
            let plane = Surface::Plane { frame: *frame };
            Ok(matches!(
                intersect_curve_surface(spline, &plane, tol, meter)?,
                CurveSurfaceIntersection::Coincident
            ) && matches!(
                intersect_curve_surface(
                    spline,
                    &wall_of(conic).ok_or_else(unsupported)?,
                    tol,
                    meter
                )?,
                CurveSurfaceIntersection::Coincident
            ))
        }
        (Curve::Nurbs(na), Curve::Nurbs(nb)) => {
            nurbs_nurbs_coincide(na, nb, tol, meter)?.ok_or_else(unsupported)
        }
    }
}

/// The points two conics in planes that are not parallel have in common,
/// found where the line the two planes meet in crosses the first conic —
/// a quadratic along that line — and kept where they lie on the second
/// within `tol.linear`; `None` when either curve is not a circle or an
/// ellipse, or the planes are parallel within `tol.angular`, where the
/// line does not exist.
///
/// Guarantees: every common point is a root of the one quadratic, so each
/// is placed to rounding whatever the angle the conic makes with the
/// other's plane, and a hit is `tangent` only where its root is double
/// to the polynomial's own rounding
/// ([`arris_math::roots::POLYNOMIAL_ROUNDING`]) — never by a tolerance.
/// That is the difference from [`intersect_curves`], which meets the
/// first conic with the second's plane and calls two crossings less than
/// `tol.linear` deep one touch: a small circle through a sphere's pole
/// 2e-4 of a radian from the seam's meridian crosses it again 1.8e-4 on,
/// within 8e-10 of the circle's plane over the whole stretch, and is one
/// touch there and two crossings here. Hits are sorted by `ta` in
/// `[0, 2π)`, `tb` is the second conic's projection of the point, and
/// the result is deterministic bit for bit.
///
/// ```
/// use arris_geom::{Curve, conic_crossings};
/// use arris_math::{Frame, Point3, Precision, Vec3};
///
/// // Two great circles of the unit sphere, 1e-3 of a radian apart: they
/// // cross at both ends of the line their planes share.
/// let a = Curve::Circle { frame: Frame::world(), radius: 1.0 };
/// let tilted = Frame::from_z(Point3::origin(), Vec3::new(0.0, (1e-3f64).sin(), (1e-3f64).cos()))?;
/// let b = Curve::Circle { frame: tilted, radius: 1.0 };
/// let hits = conic_crossings(&a, &b, Precision::DEFAULT.tolerance())?.unwrap();
/// assert_eq!(hits.len(), 2);
/// assert!(hits.iter().all(|h| !h.tangent && h.point.y.abs() < 1e-15));
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
///
/// Errors: [`GeomError::InvalidTolerance`]; a projection onto the second
/// conic that fails.
pub fn conic_crossings(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
) -> Result<Option<Vec<CurveCurveHit>>, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    let (Some((fa, [ra, rb])), Some((fb, _))) = (conic_frame(a), conic_frame(b)) else {
        return Ok(None);
    };
    let (na, nb) = (fa.z().into_inner(), fb.z().into_inner());
    let along = na.cross(&nb);
    if along.norm().atan2(na.dot(&nb).abs()) <= tol.angular {
        return Ok(None);
    }
    // The line in `a`'s plane, in its frame: square to `b`'s normal
    // there, through the point of `b`'s plane nearest `a`'s centre.
    let normal = fa.vec_to_local(nb);
    let across = Vec2::new(normal.x, normal.y);
    let reach = across.norm();
    let direction = Vec2::new(-across.y, across.x) / reach;
    let height = nb.dot(&(fb.origin() - fa.origin()));
    let foot = across * (height / (reach * reach));
    // The conic `(x / ra)² + (y / rb)² = 1` along `foot + s direction`.
    let scaled = |v: Vec2| Vec2::new(v.x / ra, v.y / rb);
    let (q, d) = (scaled(foot), scaled(direction));
    let Ok(roots) = roots::quadratic(d.norm_squared(), 2.0 * q.dot(&d), q.norm_squared() - 1.0)
    else {
        return Ok(None);
    };
    let mut out = Vec::with_capacity(roots.len());
    for root in roots.iter() {
        let at = Point2::from(foot) + root.value * direction;
        let point = fa.origin() + at.x * fa.x().into_inner() + at.y * fa.y().into_inner();
        let projection = match b.project(point) {
            Ok(p) => p,
            Err(GeomError::Ambiguous { .. }) => continue,
            Err(e) => return Err(e),
        };
        if projection.distance > tol.linear {
            continue;
        }
        out.push(CurveCurveHit {
            ta: wrap_angle((at.y / rb).atan2(at.x / ra)),
            tb: projection.t,
            point,
            tangent: root.multiplicity > 1,
        });
    }
    out.sort_by(|x, y| x.ta.total_cmp(&y.ta));
    Ok(Some(out))
}

/// `hits` sorted by `ta`; a total order, since every parameter is finite.
fn points(mut hits: Vec<CurveCurveHit>) -> CurveIntersection {
    hits.sort_by(|x, y| x.ta.total_cmp(&y.ta));
    CurveIntersection::Points(hits)
}

/// The same answer with the operands the other way round.
fn swapped(found: CurveIntersection) -> CurveIntersection {
    match found {
        CurveIntersection::Coincident => CurveIntersection::Coincident,
        CurveIntersection::Points(hits) => points(
            hits.into_iter()
                .map(|h| CurveCurveHit {
                    ta: h.tb,
                    tb: h.ta,
                    // `point` is the first curve's; the two agree within
                    // `tol.linear` and the swap keeps the one that was
                    // computed, not a re-evaluation.
                    point: h.point,
                    tangent: h.tangent,
                })
                .collect(),
        ),
    }
}

/// The frame of a conic: its centre and its plane.
fn conic_frame(c: &Curve) -> Option<(&Frame, [f64; 2])> {
    match c {
        Curve::Circle { frame, radius } => Some((frame, [*radius, *radius])),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => Some((frame, [*major_radius, *minor_radius])),
        Curve::Line { .. } | Curve::Nurbs(_) => None,
    }
}

fn line_line(oa: Point3, da: Vec3, ob: Point3, db: Vec3, tol: Tolerance) -> CurveIntersection {
    let cross = da.cross(&db);
    let offset = ob - oa;
    if cross.norm().atan2(da.dot(&db).abs()) <= tol.angular {
        // Parallel: the same line, or never meeting.
        let across = offset - offset.dot(&da) * da;
        return if across.norm() <= tol.linear {
            CurveIntersection::Coincident
        } else {
            CurveIntersection::Points(Vec::new())
        };
    }
    // Skew or crossing: the parameters of the nearest approach, and the
    // gap between the two points there.
    let n2 = cross.norm_squared();
    let ta = offset.cross(&db).dot(&cross) / n2;
    let tb = offset.cross(&da).dot(&cross) / n2;
    let (pa, pb) = (oa + ta * da, ob + tb * db);
    if (pa - pb).norm() > tol.linear {
        return CurveIntersection::Points(Vec::new());
    }
    CurveIntersection::Points(vec![CurveCurveHit {
        ta,
        tb,
        point: pa,
        tangent: false,
    }])
}

/// `a` against the conic `b`, through `b`'s plane: every common point is
/// in that plane, so the plane's hits are the candidates and `b`'s own
/// projection keeps the ones that are on it.
fn through_plane(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveIntersection, GeomError> {
    let Some((frame, _)) = conic_frame(b) else {
        return Err(GeomError::Unsupported {
            a: GeomKind::Curve(a.kind()),
            b: GeomKind::Curve(b.kind()),
        });
    };
    let plane = Surface::Plane { frame: *frame };
    let hits = match intersect_curve_surface(a, &plane, tol, meter)? {
        CurveSurfaceIntersection::Points(hits) => hits,
        CurveSurfaceIntersection::Coincident => return coplanar(a, b, tol, meter),
    };
    let mut out = Vec::with_capacity(hits.len());
    for h in hits {
        let projection = match b.project(h.point) {
            Ok(p) => p,
            // The point is the conic's centre, which is equidistant from
            // every point of it: on the conic only for a radius below the
            // tolerance, which no valid conic has.
            Err(GeomError::Ambiguous { .. }) => continue,
            Err(e) => return Err(e),
        };
        if projection.distance <= tol.linear {
            out.push(CurveCurveHit {
                ta: h.t,
                tb: projection.t,
                point: h.point,
                tangent: h.tangent,
            });
        }
    }
    Ok(points(out))
}

/// `a` and the conic `b` in one plane, where the plane decides nothing.
fn coplanar(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveIntersection, GeomError> {
    match (a, b) {
        (&Curve::Line { origin, direction }, _) => {
            let (frame, _) = conic_frame(b).ok_or(GeomError::Unsupported {
                a: GeomKind::Curve(a.kind()),
                b: GeomKind::Curve(b.kind()),
            })?;
            line_conic_coplanar(origin, direction.into_inner(), frame, b, tol, meter)
        }
        (
            &Curve::Circle {
                frame: fa,
                radius: ra,
            },
            &Curve::Circle {
                frame: fb,
                radius: rb,
            },
        ) => circle_circle_coplanar(a, b, &fa, ra, &fb, rb, tol),
        // A coplanar pair with an ellipse in it: the quartic of
        // `crate::conic2`, in the second conic's plane.
        (
            Curve::Circle { .. } | Curve::Ellipse { .. },
            Curve::Circle { .. } | Curve::Ellipse { .. },
        ) => conic_conic_coplanar(a, b, tol),
        (Curve::Nurbs(_), Curve::Circle { .. } | Curve::Ellipse { .. }) => {
            nurbs_conic_coplanar(a, b, tol, meter)
        }
        (Curve::Nurbs(_), Curve::Line { .. } | Curve::Nurbs(_))
        | (Curve::Circle { .. } | Curve::Ellipse { .. }, Curve::Line { .. } | Curve::Nurbs(_)) => {
            Err(GeomError::Unsupported {
                a: GeomKind::Curve(a.kind()),
                b: GeomKind::Curve(b.kind()),
            })
        }
    }
}

/// Two conics in one plane with an ellipse among them, where the plane
/// decides nothing: in the second's plane both are a [`Conic2`] — the
/// first's centre and axes turned into it, which is where they lie
/// within the tolerance — and [`conic_pair`]'s quartic decides them.
/// `Coincident` when the first lies within `tol.linear` of the second
/// everywhere, which is what a boolean's edge and the edge a second
/// boolean meets it with are; otherwise each touch and crossing is a
/// hit at the first conic's own parameter, with `tb` from the second's
/// projection of the point as [`through_plane`] reads it.
fn conic_conic_coplanar(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
) -> Result<CurveIntersection, GeomError> {
    let (Some((fa, ra)), Some((fb, rb))) = (conic_frame(a), conic_frame(b)) else {
        return Err(GeomError::Unsupported {
            a: GeomKind::Curve(a.kind()),
            b: GeomKind::Curve(b.kind()),
        });
    };
    let degenerate = |reason: String| GeomError::Degenerate {
        kind: GeomKind::Curve(a.kind()),
        reason,
    };
    let into_plane = |v: Vec3| {
        let w = fb.vec_to_local(v);
        Vec2::new(w.x, w.y)
            .try_normalize(0.0)
            .ok_or_else(|| degenerate("a conic axis square to the plane it lies in".to_owned()))
    };
    let centre = fb.to_local(fa.origin());
    let first = Conic2 {
        centre: Point2::new(centre.x, centre.y),
        x: into_plane(fa.x().into_inner())?,
        y: into_plane(fa.y().into_inner())?,
        a: ra[0],
        b: ra[1],
    };
    let second = Conic2 {
        centre: Point2::origin(),
        x: Vec2::x(),
        y: Vec2::y(),
        a: rb[0],
        b: rb[1],
    };
    match conic_pair(&first, &second, tol)
        .map_err(|e| degenerate(format!("two conics in one plane: {e}")))?
    {
        ConicMeet::Coincident => Ok(CurveIntersection::Coincident),
        ConicMeet::Empty => Ok(CurveIntersection::Points(Vec::new())),
        ConicMeet::Meets(meets) => {
            let mut out = Vec::with_capacity(meets.len());
            for (ta, tangent) in meets {
                let point = a.point(ta);
                let tb = match b.project(point) {
                    Ok(p) => p.t,
                    // The second conic's centre, which no point of it is.
                    Err(GeomError::Ambiguous { .. }) => continue,
                    Err(e) => return Err(e),
                };
                out.push(CurveCurveHit {
                    ta,
                    tb,
                    point,
                    tangent,
                });
            }
            Ok(points(out))
        }
    }
}

/// The cylinder a conic is the section of across its axis: the conic's
/// frame, its radii. A curve in the conic's plane meets the conic exactly
/// where it meets that cylinder, and its distance from the conic in the
/// plane is its distance from the cylinder.
fn wall_of(conic: &Curve) -> Option<Surface> {
    match *conic {
        Curve::Circle { frame, radius } => Some(Surface::Cylinder { frame, radius }),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => Some(Surface::EllipticCylinder {
            frame,
            major_radius,
            minor_radius,
        }),
        Curve::Line { .. } | Curve::Nurbs(_) => None,
    }
}

/// A NURBS curve in the conic `b`'s plane: the curve against the conic's
/// cylinder ([`wall_of`]), which decides the touches in `tol.linear` of
/// length by the NURBS arm that already exists, and `Coincident` when
/// the curve lies on the cylinder too.
fn nurbs_conic_coplanar(
    a: &Curve,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveIntersection, GeomError> {
    let wall = wall_of(b).ok_or(GeomError::Unsupported {
        a: GeomKind::Curve(a.kind()),
        b: GeomKind::Curve(b.kind()),
    })?;
    let hits = match intersect_curve_surface(a, &wall, tol, meter)? {
        CurveSurfaceIntersection::Coincident => return Ok(CurveIntersection::Coincident),
        CurveSurfaceIntersection::Points(hits) => hits,
    };
    let mut out = Vec::with_capacity(hits.len());
    for h in hits {
        let projection = match b.project(h.point) {
            Ok(p) => p,
            // The centre, which no point of the cylinder is.
            Err(GeomError::Ambiguous { .. }) => continue,
            Err(e) => return Err(e),
        };
        out.push(CurveCurveHit {
            ta: h.t,
            tb: projection.t,
            point: h.point,
            tangent: h.tangent,
        });
    }
    Ok(points(out))
}

/// The points [`nurbs_line`] samples between two candidates to tell one
/// shallow crossing, found by both planes, from two: the stretch between
/// two candidates of one crossing is a few tolerances long and straight
/// to rounding, so every sample of it is on the line, while between two
/// crossings the curve leaves the line and the samples across the gap
/// see it do so.
const LINE_STRETCH_SAMPLES: usize = 8;

/// A NURBS curve against a line, through two planes that hold the line
/// and are square to each other: every common point is on both, so the
/// curve's hits on either within `tol.linear` of the line are the
/// candidates. A crossing of the line is transversal to at least one of
/// the two planes unless the curve runs along the line there, so it is
/// found to rounding by that one; the candidates of one point — within
/// `tol.linear` of each other, or joined by a stretch of the curve that
/// stays within it of the line — are one hit, a crossing of a plane
/// before a touch of the other — the curve is tangent to the line only
/// where it touches every plane through it — then the one nearest the
/// line. A curve in one of the planes is the
/// coplanar case, the other plane's hits being the answer as a line and
/// a coplanar conic are; a curve in both lies along the line,
/// `Coincident`.
fn nurbs_line(
    a: &Curve,
    origin: Point3,
    direction: Vec3,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveIntersection, GeomError> {
    let degenerate = |_| GeomError::Degenerate {
        kind: GeomKind::Curve(CurveKind::Line),
        reason: "a line whose direction is not a direction".to_string(),
    };
    let across = Frame::from_z(origin, direction).map_err(degenerate)?;
    let mut found = Vec::with_capacity(2);
    for normal in [across.x(), across.y()] {
        let frame = Frame::from_z(origin, normal.into_inner()).map_err(degenerate)?;
        found.push(intersect_curve_surface(
            a,
            &Surface::Plane { frame },
            tol,
            meter,
        )?);
    }
    let on_line = |h: &CurveSurfaceHit| CurveCurveHit {
        ta: h.t,
        tb: (h.point - origin).dot(&direction),
        point: h.point,
        tangent: h.tangent,
    };
    let (first, second) = match (&found[0], &found[1]) {
        (CurveSurfaceIntersection::Coincident, CurveSurfaceIntersection::Coincident) => {
            return Ok(CurveIntersection::Coincident);
        }
        (CurveSurfaceIntersection::Coincident, CurveSurfaceIntersection::Points(hits))
        | (CurveSurfaceIntersection::Points(hits), CurveSurfaceIntersection::Coincident) => {
            return Ok(points(hits.iter().map(on_line).collect()));
        }
        (CurveSurfaceIntersection::Points(first), CurveSurfaceIntersection::Points(second)) => {
            (first, second)
        }
    };
    let off_line = |p: Point3| {
        let v = p - origin;
        (v - v.dot(&direction) * direction).norm()
    };
    let mut candidates: Vec<(f64, CurveCurveHit)> = first
        .iter()
        .chain(second)
        .map(|h| (off_line(h.point), on_line(h)))
        .filter(|(off, _)| *off <= tol.linear)
        .collect();
    // Stable, so the first plane's candidate wins a tie.
    candidates.sort_by(|x, y| x.1.tangent.cmp(&y.1.tangent).then(x.0.total_cmp(&y.0)));
    // One crossing when the curve stays within `tol.linear` of the line
    // from one candidate to the other: at a shallow angle each plane
    // finds the crossing where the curve's own offset across it vanishes,
    // and those two lie that offset over the angle's tangent apart, which
    // can be past `tol.linear` though nothing leaves the line between.
    let stays_on_line = |t0: f64, t1: f64| {
        (1..LINE_STRETCH_SAMPLES).all(|k| {
            let s = k as f64 / LINE_STRETCH_SAMPLES as f64;
            off_line(a.point(t0 + s * (t1 - t0))) <= tol.linear
        })
    };
    let mut kept: Vec<CurveCurveHit> = Vec::with_capacity(candidates.len());
    for (_, candidate) in candidates {
        match kept.iter_mut().find(|k| {
            (k.point - candidate.point).norm() <= tol.linear || stays_on_line(k.ta, candidate.ta)
        }) {
            // A crossing of either plane there says the curve crosses.
            Some(k) => k.tangent &= candidate.tangent,
            None => kept.push(candidate),
        }
    }
    Ok(points(kept))
}

/// Whether two NURBS curves are the same curve, where that has an
/// answer: `Some(true)` when they are the same spline — degree, knots
/// and weights equal and every control point within `tol.linear` of its
/// twin, so the two are within it at every parameter, the weights being
/// equal — which is what the same section made twice is; `Some(false)`
/// when a point of either, at an end or a knot or halfway between two,
/// is not within `tol.linear` of the other; `None` otherwise, a curve
/// made again over other knots or weights, which only a curve–curve
/// marcher could tell from a near one.
///
/// "Within `tol.linear` of the other" is decided by the NURBS arm of
/// [`intersect_curve_surface`], not by a projection, whose sampling can
/// settle on a farther local minimum: a curve the same as the first
/// runs along it at the point, so it crosses the plane square to the
/// first curve there within `tol.linear` of the point — the part of
/// their offset across the curve — and a curve with no hit on that plane
/// so near is not the same curve.
fn nurbs_nurbs_coincide(
    a: &NurbsCurve,
    b: &NurbsCurve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Option<bool>, GeomError> {
    let same_spline = a.degree() == b.degree()
        && a.knots() == b.knots()
        && a.weights() == b.weights()
        && a.control_points()
            .iter()
            .zip(b.control_points())
            .all(|(p, q)| (p - q).norm() <= tol.linear);
    if same_spline {
        return Ok(Some(true));
    }
    for (x, y) in [(a, b), (b, a)] {
        let other = Curve::Nurbs(y.clone());
        for t in probes(x) {
            let at = x.eval(t);
            // A stationary point has no plane square to the curve; the
            // other probes decide.
            let Ok(frame) = Frame::from_z(at.point, at.d1) else {
                continue;
            };
            let near = match intersect_curve_surface(&other, &Surface::Plane { frame }, tol, meter)?
            {
                CurveSurfaceIntersection::Coincident => false,
                CurveSurfaceIntersection::Points(hits) => hits
                    .iter()
                    .any(|h| (h.point - at.point).norm() <= tol.linear),
            };
            if !near {
                return Ok(Some(false));
            }
        }
    }
    Ok(None)
}

/// The parameters a curve is probed at: its distinct knots in the domain
/// and the midpoint of every span between them.
fn probes(c: &NurbsCurve) -> Vec<f64> {
    let domain = c.domain();
    let mut knots: Vec<f64> = c
        .knots()
        .iter()
        .copied()
        .filter(|&k| domain.contains(k))
        .collect();
    knots.dedup();
    let mut out = Vec::with_capacity(2 * knots.len());
    for w in knots.windows(2) {
        out.push(w[0]);
        out.push(0.5 * (w[0] + w[1]));
    }
    out.extend(knots.last());
    out
}

/// A line in the conic's own plane: the points the two share are the
/// conic's points on the plane through the line perpendicular to the
/// conic's, which [`intersect_curve_surface`] already decides — including
/// the tangency, in the linear tolerance.
fn line_conic_coplanar(
    origin: Point3,
    direction: Vec3,
    conic: &Frame,
    b: &Curve,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<CurveIntersection, GeomError> {
    let normal = conic.z().cross(&direction);
    let cut = Frame::from_z(origin, normal).map_err(|_| GeomError::Degenerate {
        kind: GeomKind::Curve(CurveKind::Line),
        reason: "a line in the conic's plane whose direction is not a direction".to_string(),
    })?;
    let plane = Surface::Plane { frame: cut };
    let found = match intersect_curve_surface(b, &plane, tol, meter)? {
        CurveSurfaceIntersection::Points(hits) => hits,
        // The conic lies in the cutting plane too, which would make it a
        // line: no valid conic does.
        CurveSurfaceIntersection::Coincident => {
            return Err(GeomError::Degenerate {
                kind: GeomKind::Curve(b.kind()),
                reason: "a conic inside a plane perpendicular to its own".to_string(),
            });
        }
    };
    Ok(points(
        found
            .into_iter()
            .map(|h| CurveCurveHit {
                ta: (h.point - origin).dot(&direction),
                tb: h.t,
                point: h.point,
                tangent: h.tangent,
            })
            .collect(),
    ))
}

/// Two circles in one plane: the radical line. Worked in `b`'s frame,
/// where `b` is the unit of the construction and `a`'s centre is one
/// vector away.
fn circle_circle_coplanar(
    a: &Curve,
    b: &Curve,
    fa: &Frame,
    ra: f64,
    fb: &Frame,
    rb: f64,
    tol: Tolerance,
) -> Result<CurveIntersection, GeomError> {
    let local = fb.to_local(fa.origin());
    let centre = Vec2::new(local.x, local.y);
    let apart = centre.norm();
    if apart <= tol.linear {
        return Ok(if (ra - rb).abs() <= tol.linear {
            CurveIntersection::Coincident
        } else {
            // Concentric and of different radii: never meeting.
            CurveIntersection::Points(Vec::new())
        });
    }
    let along = centre / apart;
    let across = Vec2::new(-along.y, along.x);
    let lift = |p: Vec2| fb.to_world(Point3::new(p.x, p.y, 0.0));
    let touch = |at: Vec2| -> Result<CurveIntersection, GeomError> {
        Ok(points(vec![hit(a, b, lift(at), true)?]))
    };
    // Outer touch, inner touch, and the two ways to miss.
    if (apart - (ra + rb)).abs() <= tol.linear {
        return touch(rb * along);
    }
    if (apart - (ra - rb).abs()).abs() <= tol.linear {
        return touch(if ra > rb { -rb * along } else { rb * along });
    }
    if apart > ra + rb || apart < (ra - rb).abs() {
        return Ok(CurveIntersection::Points(Vec::new()));
    }
    // The chord: at `x` along the line of centres, half-length `h`.
    let x = (apart * apart + rb * rb - ra * ra) / (2.0 * apart);
    let h = (rb - x).sqrt() * (rb + x).sqrt();
    Ok(points(vec![
        hit(a, b, lift(x * along + h * across), false)?,
        hit(a, b, lift(x * along - h * across), false)?,
    ]))
}

/// A hit at `point`, with each curve's parameter from its own projection.
fn hit(a: &Curve, b: &Curve, point: Point3, tangent: bool) -> Result<CurveCurveHit, GeomError> {
    Ok(CurveCurveHit {
        ta: wrap_angle_if_periodic(a, a.project(point)?.t),
        tb: wrap_angle_if_periodic(b, b.project(point)?.t),
        point,
        tangent,
    })
}

/// A periodic curve's parameter in `[0, 2π)`; anything else unchanged.
fn wrap_angle_if_periodic(c: &Curve, t: f64) -> f64 {
    if c.period().is_some() {
        wrap_angle(t)
    } else {
        t
    }
}
