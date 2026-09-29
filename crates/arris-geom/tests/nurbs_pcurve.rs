//! `pcurve_on` onto a NURBS surface (`docs/DATA-MODEL.md` §Pcurves,
//! ADR-0025 §1): curves lying on the twins of `Surface::to_nurbs`, whose
//! closed-form points put them on the twin to rounding, each given a
//! pcurve whose image is held to the curve within the linear tolerance.
//!
//! The twins are built over a full turn from a random angle, so the seam
//! of their closed direction is anywhere along the curve, and a sphere's
//! over both poles, so its collapsed rows are where a meridian ends.

mod common;

use core::f64::consts::{FRAC_PI_2, PI, TAU};

use arris_debug::prop::finite_f64;
use arris_debug::prop::geom::{cylinder, sphere, torus};
use arris_geom::{Curve, Curve2, GeomError, Surface, pcurve_on};
use arris_math::{Frame, Interval, Point3, Precision, Tolerance, Vec3};
use proptest::prelude::*;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// Where along a pcurve its image is compared with the curve: far denser
/// than the fit's own check parameters, so what is held is the curve and
/// not the samples the fit saw.
const SAMPLES: usize = 2000;

/// The pcurve of `curve` over `range` on `twin`, and the largest distance
/// between its image and the curve at [`SAMPLES`] parameters.
fn deviation(curve: &Curve, range: Interval, twin: &Surface) -> Result<(Curve2, f64), GeomError> {
    let pc = pcurve_on(curve, range, twin, tol(), &mut arris_math::Meter::default())?;
    let worst = (0..=SAMPLES)
        .map(|i| {
            let t = range.lerp(i as f64 / SAMPLES as f64);
            let uv = pc.point(t);
            (twin.point(uv.x, uv.y) - curve.point(t)).norm()
        })
        .fold(0.0, f64::max);
    Ok((pc, worst))
}

/// A full turn from `start`, in the angle parameter.
fn turn_from(start: f64) -> Interval {
    Interval::new(start, start + TAU).unwrap()
}

/// A range of a circle's angle: any start, a sweep from a tenth of a turn
/// to a whole one, one time in four exactly a turn.
fn sweep() -> impl Strategy<Value = Interval> {
    (
        finite_f64(-7.0..=7.0),
        prop_oneof![3 => finite_f64(0.6..=TAU), 1 => Just(TAU)],
    )
        .prop_map(|(lo, sweep)| Interval::new(lo, lo + sweep).unwrap())
}

/// The circle about `frame`'s axis at height `z` and radius `rho`, its own
/// `X` turned by `phase` from the frame's and its `Z` along or against it.
fn about_axis(frame: &Frame, z: f64, rho: f64, phase: f64, along: bool) -> Curve {
    let x = phase.cos() * frame.x().into_inner() + phase.sin() * frame.y().into_inner();
    let axis = if along { frame.z() } else { -frame.z() };
    Curve::Circle {
        frame: Frame::new(
            frame.origin() + z * frame.z().into_inner(),
            axis.into_inner(),
            x,
        )
        .unwrap(),
        radius: rho,
    }
}

/// The unit direction at angle `u` about `frame`'s axis.
fn radial(frame: &Frame, u: f64) -> Vec3 {
    u.cos() * frame.x().into_inner() + u.sin() * frame.y().into_inner()
}

arris_debug::prop_shards! {
    a_parallel_of_a_nurbs_sphere_crosses_its_seam [shard_0 shard_1 shard_2 shard_3]
        ((surface, seam, latitude, phase, along, range)) =
        (
            sphere(),
            finite_f64(-7.0..=7.0),
            finite_f64(-1.4..=1.4),
            finite_f64(-PI..=PI),
            any::<bool>(),
            sweep(),
        )
    => {
            let Surface::Sphere { frame, radius } = surface else {
                unreachable!()
            };
            let twin = Surface::Nurbs(
                surface
                    .to_nurbs([turn_from(seam), Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap()])
                    .unwrap(),
            );
            let parallel = about_axis(
                &frame,
                radius * latitude.sin(),
                radius * latitude.cos(),
                phase,
                along,
            );
            let (_, worst) = deviation(&parallel, range, &twin)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(worst <= tol().linear, "off the parallel by {worst}");
            Ok(())
    }
}

arris_debug::prop_shards! {
    a_meridian_of_a_nurbs_sphere_runs_to_a_pole [shard_0 shard_1 shard_2 shard_3]
        ((surface, seam, at, from, north)) =
        (
            sphere(),
            finite_f64(-7.0..=7.0),
            // The meridian's angle about the axis: anywhere, or on the seam.
            prop_oneof![3 => finite_f64(-PI..=PI).prop_map(Some), 1 => Just(None)],
            finite_f64(-1.4..=1.3),
            any::<bool>(),
        )
    => {
            let Surface::Sphere { frame, radius } = surface else {
                unreachable!()
            };
            let twin = Surface::Nurbs(
                surface
                    .to_nurbs([turn_from(seam), Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap()])
                    .unwrap(),
            );
            let u = at.unwrap_or(seam);
            // `t` is the latitude: `X` radial at `u`, `Y` the axis.
            let meridian = Curve::Circle {
                frame: Frame::new(
                    frame.origin(),
                    radial(&frame, u).cross(&frame.z().into_inner()),
                    radial(&frame, u),
                )
                .unwrap(),
                radius,
            };
            let range = if north {
                Interval::new(from, FRAC_PI_2).unwrap()
            } else {
                Interval::new(-FRAC_PI_2, -from).unwrap()
            };
            let (_, worst) = deviation(&meridian, range, &twin)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(worst <= tol().linear, "off the meridian by {worst}");
            // Over the pole it runs through a collapsed row, which is
            // refused, never fitted through.
            let over = Interval::new(0.5 * PI - 0.4, 0.5 * PI + 0.3).unwrap();
            prop_assert!(
                matches!(
                    pcurve_on(&meridian, over, &twin, tol(), &mut arris_math::Meter::default()),
                    Err(GeomError::ThroughSingularity { .. })
                ),
                "a meridian over the pole is not refused"
            );
            Ok(())
    }
}

arris_debug::prop_shards! {
    a_nurbs_torus_carries_its_section_circles [shard_0 shard_1 shard_2 shard_3]
        ((surface, seam_u, seam_v, at, phase, tube, along, range)) =
        (
            torus(),
            finite_f64(-7.0..=7.0),
            finite_f64(-7.0..=7.0),
            finite_f64(-PI..=PI),
            finite_f64(-PI..=PI),
            any::<bool>(),
            any::<bool>(),
            sweep(),
        )
    => {
            let Surface::Torus {
                frame,
                major_radius,
                minor_radius,
            } = surface
            else {
                unreachable!()
            };
            let twin = Surface::Nurbs(
                surface
                    .to_nurbs([turn_from(seam_u), turn_from(seam_v)])
                    .unwrap(),
            );
            let circle = if tube {
                // The tube's circle at angle `at` about the axis.
                let out = radial(&frame, at);
                let sense = if along { 1.0 } else { -1.0 };
                Curve::Circle {
                    frame: Frame::new(
                        frame.origin() + major_radius * out,
                        sense * out.cross(&frame.z().into_inner()),
                        phase.cos() * out + phase.sin() * frame.z().into_inner(),
                    )
                    .unwrap(),
                    radius: minor_radius,
                }
            } else {
                // The circle about the axis at the tube's angle `at`.
                about_axis(
                    &frame,
                    minor_radius * at.sin(),
                    major_radius + minor_radius * at.cos(),
                    phase,
                    along,
                )
            };
            let (_, worst) = deviation(&circle, range, &twin)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(worst <= tol().linear, "off the circle by {worst}");
            Ok(())
    }
}

arris_debug::prop_shards! {
    an_oblique_ellipse_lies_on_a_nurbs_cylinder [shard_0 shard_1 shard_2 shard_3]
        ((surface, seam, tilt, heading, height, range)) =
        (
            cylinder(),
            finite_f64(-7.0..=7.0),
            finite_f64(0.05..=1.2),
            finite_f64(-PI..=PI),
            finite_f64(-5.0..=5.0),
            sweep(),
        )
    => {
            let Surface::Cylinder { frame, radius } = surface else {
                unreachable!()
            };
            // The plane through the axis point at `height`, its normal
            // tilted from the axis toward `heading`: the section's minor
            // radius is the cylinder's, its major axis rises along
            // `heading`.
            let along = radial(&frame, heading);
            let normal = tilt.cos() * frame.z().into_inner() - tilt.sin() * along;
            let major = tilt.cos() * along + tilt.sin() * frame.z().into_inner();
            let reach = radius * tilt.tan() + 1.0;
            let twin = Surface::Nurbs(
                surface
                    .to_nurbs([
                        turn_from(seam),
                        Interval::new(height - reach, height + reach).unwrap(),
                    ])
                    .unwrap(),
            );
            let ellipse = Curve::Ellipse {
                frame: Frame::new(
                    frame.origin() + height * frame.z().into_inner(),
                    normal,
                    major,
                )
                .unwrap(),
                major_radius: radius / tilt.cos(),
                minor_radius: radius,
            };
            let (_, worst) = deviation(&ellipse, range, &twin)
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
            prop_assert!(worst <= tol().linear, "off the ellipse by {worst}");
            Ok(())
    }
}

#[test]
fn a_curve_off_a_nurbs_surface_is_named() {
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 1.0,
    };
    let twin = Surface::Nurbs(
        ball.to_nurbs([
            Interval::TURN,
            Interval::new(-FRAC_PI_2, FRAC_PI_2).unwrap(),
        ])
        .unwrap(),
    );
    let lifted = Curve::Circle {
        frame: Frame::from_z(Point3::new(0.0, 0.0, 1e-3), Vec3::z()).unwrap(),
        radius: 1.0,
    };
    assert!(matches!(
        pcurve_on(
            &lifted,
            Interval::TURN,
            &twin,
            tol(),
            &mut arris_math::Meter::default()
        ),
        Err(GeomError::NotOnSurface { .. })
    ));
}
