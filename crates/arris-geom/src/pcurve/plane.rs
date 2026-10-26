//! Pcurves in a plane, and the projection of a curve onto one for a consumer's sketch.

use arris_math::real;

use arris_math::{
    Frame, Frame2, Handedness, Interval, Meter, Point2, Tolerance, UnitVec2, Vec2, Vec3,
    is_negligible,
};

use crate::{Curve, Curve2, GeomError, GeomKind, NurbsCurve, NurbsCurve2, PcurveFault, fit_curve2};

use super::*;

/// The (u, v) coordinates of `p` in a plane's frame.
pub(super) fn in_plane(plane: &Frame, p: arris_math::Point3) -> Point2 {
    let q = plane.to_local(p);
    Point2::new(q.x, q.y)
}

/// The (u, v) components of `v` in a plane's frame.
pub(super) fn in_plane_vec(plane: &Frame, v: Vec3) -> Vec2 {
    let q = plane.vec_to_local(v);
    Vec2::new(q.x, q.y)
}

/// The exact pcurve of a curve lying in `plane`, at the same parameter.
pub(super) fn in_plane_curve(curve: &Curve, plane: &Frame) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    match curve {
        &Curve::Line { origin, direction } => {
            let d = in_plane_vec(plane, direction.into_inner());
            let direction = UnitVec2::try_new(d, 0.0)
                .ok_or_else(|| degenerate(kind, PcurveFault::LinePerpendicular))?;
            Ok(Curve2::Line {
                origin: in_plane(plane, origin),
                direction,
            })
        }
        &Curve::Circle { ref frame, radius } => Ok(Curve2::Circle {
            frame: conic_frame(plane, frame, kind)?,
            radius,
        }),
        &Curve::Ellipse {
            ref frame,
            major_radius,
            minor_radius,
        } => Ok(Curve2::Ellipse {
            frame: conic_frame(plane, frame, kind)?,
            major_radius,
            minor_radius,
        }),
        Curve::Nurbs(c) => Ok(Curve2::Nurbs(nurbs_in_plane(c, plane)?)),
    }
}

/// The projection of `curve` over `range` onto `plane`, at its own
/// parameter, as a `Nurbs` fitted to it within `tol.linear` in the plane:
/// a curve off the plane is as near as any pcurve can be to it there, the
/// projection, and holding the fit to the curve itself would ask it to
/// close the curve's own distance from the plane, which it cannot.
pub(super) fn projected_in_plane(
    curve: &Curve,
    range: Interval,
    plane: &Frame,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<Curve2, GeomError> {
    let f = |t: f64| in_plane(plane, curve.point(t));
    let fit = fit_curve2(
        f,
        range,
        PCURVE_FIT_DEGREE,
        |t, q| (q - f(t)).norm(),
        tol.linear,
        meter,
    )?;
    Ok(Curve2::Nurbs(fit))
}

/// The `Frame2` of a conic whose frame lies in `plane`: origin and `X` in
/// plane coordinates, right-handed when the conic's `Z` is along the
/// plane's normal and left-handed when it opposes it.
pub(super) fn conic_frame(
    plane: &Frame,
    frame: &Frame,
    kind: GeomKind,
) -> Result<Frame2, GeomError> {
    let handedness = if frame.z().dot(&plane.z()) >= 0.0 {
        Handedness::Right
    } else {
        Handedness::Left
    };
    Frame2::new(
        in_plane(plane, frame.origin()),
        in_plane_vec(plane, frame.x().into_inner()),
        handedness,
    )
    .map_err(|e| degenerate(kind, PcurveFault::ConicAxes(e)))
}

/// A NURBS with its control points in plane coordinates: the projection
/// is affine, so knots and weights carry over and the parameter is the
/// same.
pub(super) fn nurbs_in_plane(c: &NurbsCurve, plane: &Frame) -> Result<NurbsCurve2, GeomError> {
    NurbsCurve2::new(
        c.degree(),
        c.knots().to_vec(),
        c.control_points()
            .iter()
            .map(|p| in_plane(plane, *p))
            .collect(),
        c.weights().to_vec(),
    )
}

/// The orthogonal projection of `curve` onto `plane` as a `Curve2` in the
/// plane's (u, v), for a consumer's sketch (`docs/ARCHITECTURE.md`
/// §Facade): a line stays a `Line`, a circle becomes a `Circle` when its
/// plane is parallel and an `Ellipse` otherwise, an ellipse an `Ellipse`,
/// a NURBS a `Nurbs` with its control points projected. This is a
/// point-set projection: the parameter is the variant's own — a line's
/// arc length in the plane, a conic's angle about its projected axes,
/// which differs from the 3D parameter by a phase when the conic is
/// oblique — and only a NURBS, or a curve lying in the plane, keeps the
/// 3D parameter. For the same-parameter pcurve of a curve on the plane
/// use [`pcurve_on`].
///
/// Errors: [`GeomError::Degenerate`] when the projection collapses — a
/// line perpendicular to the plane, a conic in a plane perpendicular to it.
///
/// ```
/// use arris_geom::{Curve, Curve2, project_to_plane};
/// use arris_math::{Frame, Point3, Vec3};
///
/// // A unit circle tilted by 60° about x projects to an ellipse 1 × 0.5.
/// let tilted = Frame::new(Point3::origin(), Vec3::new(0.0, -(3f64.sqrt()) / 2.0, 0.5), Vec3::x()).unwrap();
/// let circle = Curve::Circle { frame: tilted, radius: 1.0 };
/// let Curve2::Ellipse { major_radius, minor_radius, .. } = project_to_plane(&circle, &Frame::world()).unwrap() else { panic!() };
/// assert!((major_radius - 1.0).abs() < 1e-15 && (minor_radius - 0.5).abs() < 1e-15);
/// ```
pub fn project_to_plane(curve: &Curve, plane: &Frame) -> Result<Curve2, GeomError> {
    let kind = GeomKind::Curve(curve.kind());
    match curve {
        &Curve::Line { origin, direction } => {
            let d = in_plane_vec(plane, direction.into_inner());
            if is_negligible(d.norm(), 1.0) {
                return Err(degenerate(kind, PcurveFault::LineProjectsToPoint));
            }
            Ok(Curve2::Line {
                origin: in_plane(plane, origin),
                direction: UnitVec2::new_normalize(d),
            })
        }
        &Curve::Circle { ref frame, radius } => {
            if is_negligible(frame.z().cross(&plane.z()).norm(), 1.0) {
                return in_plane_curve(curve, plane);
            }
            projected_conic(plane, frame, radius, radius, kind)
        }
        &Curve::Ellipse {
            ref frame,
            major_radius,
            minor_radius,
        } => {
            if is_negligible(frame.z().cross(&plane.z()).norm(), 1.0) {
                return in_plane_curve(curve, plane);
            }
            projected_conic(plane, frame, major_radius, minor_radius, kind)
        }
        Curve::Nurbs(c) => Ok(Curve2::Nurbs(nurbs_in_plane(c, plane)?)),
    }
}

/// The ellipse `o + a cos t·X + b sin t·Y` projected onto `plane`: the
/// image is `o' + M (cos t, sin t)ᵀ` with `M = [a X' | b Y']` the projected
/// axes, and the singular value decomposition of the 2 × 2 `M` gives the
/// image's semi-axes (the singular values) and their directions (the left
/// singular vectors); a negative second singular value is a reflection,
/// which the frame's handedness records.
pub(super) fn projected_conic(
    plane: &Frame,
    frame: &Frame,
    a: f64,
    b: f64,
    kind: GeomKind,
) -> Result<Curve2, GeomError> {
    let col1 = a * in_plane_vec(plane, frame.x().into_inner());
    let col2 = b * in_plane_vec(plane, frame.y().into_inner());
    let (major, minor_signed, phi) = principal_axes(col1, col2);
    if is_negligible(minor_signed, major) {
        return Err(degenerate(kind, PcurveFault::ConicProjectsToSegment));
    }
    let handedness = if minor_signed > 0.0 {
        Handedness::Right
    } else {
        Handedness::Left
    };
    let frame2 = Frame2::new(
        in_plane(plane, frame.origin()),
        Vec2::new(real::cos(phi), real::sin(phi)),
        handedness,
    )
    .map_err(|e| degenerate(kind, PcurveFault::ProjectedAxes(e)))?;
    Ok(Curve2::Ellipse {
        frame: frame2,
        major_radius: major,
        minor_radius: minor_signed.abs(),
    })
}

/// The principal axes of the ellipse `M (cos t, sin t)ᵀ`, `M = [col1 |
/// col2]` two conjugate semi-diameters in a plane's (u, v): `(major,
/// minor, φ)` with the semi-axes the singular values of `M`, `φ` the
/// angle of the major axis from `u`, and `minor` signed — negative when
/// `M` is a reflection, so the ellipse is traversed clockwise. A
/// projected conic's, and an oblique plane section's of an elliptic
/// cylinder.
pub(crate) fn principal_axes(col1: Vec2, col2: Vec2) -> (f64, f64, f64) {
    // M = [[p, q], [r, s]] by rows.
    let (p, q, r, s) = (col1.x, col2.x, col1.y, col2.y);
    let (e, f, g, h) = (0.5 * (p + s), 0.5 * (p - s), 0.5 * (r + q), 0.5 * (r - q));
    let (big, small) = (real::hypot(e, h), real::hypot(f, g));
    let phi = 0.5 * (real::atan2(h, e) + real::atan2(g, f));
    (big + small, big - small, phi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::{Frame, Point3, Vec3};

    #[test]
    fn projection_degenerates_are_named() {
        let plane = Frame::world();
        let vertical = Curve::Line {
            origin: Point3::origin(),
            direction: Vec3::z_axis(),
        };
        assert!(matches!(
            project_to_plane(&vertical, &plane),
            Err(GeomError::Degenerate { .. })
        ));
        let edge_on = Curve::Circle {
            frame: Frame::from_z(Point3::origin(), Vec3::x()).unwrap(),
            radius: 1.0,
        };
        assert!(matches!(
            project_to_plane(&edge_on, &plane),
            Err(GeomError::Degenerate { .. })
        ));
        let flat = Curve::Circle {
            frame: Frame::from_z(Point3::new(1.0, 2.0, 3.0), -Vec3::z()).unwrap(),
            radius: 1.5,
        };
        let Curve2::Circle { frame, radius } = project_to_plane(&flat, &plane).unwrap() else {
            panic!()
        };
        assert_eq!(radius, 1.5);
        assert!(!frame.is_right_handed());
        assert_eq!(frame.origin(), Point2::new(1.0, 2.0));
    }
}
