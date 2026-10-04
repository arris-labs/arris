//! Every curve on every analytic surface has a pcurve whose image under
//! the surface is the curve at the same parameter — exactly on the exact
//! arms, within the tolerance on the fitted one, which is one fallback
//! over the surface's own projection — and a curve projected onto a plane
//! is the expected conic. The six exact arms a revolve makes on a cone, a
//! sphere and a torus are lines in (u, v); a section of any of the five
//! curved surfaces at a random pose has a pcurve on it, a torus's against
//! the tracer's exact (u, v); and by a singular point — an apex, a pole —
//! a range that ends on it fits, one that runs through it is refused
//! naming where to split, and one that passes beside it fits from the
//! band outwards. No surface is `Unsupported`; a NURBS one's arm is
//! `tests/nurbs_pcurve.rs`'s.

use core::f64::consts::{PI, TAU};

use arris_debug::prop::geom::{cylinder, nurbs_curve, plane};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, frame, radius, unit_vec3};
use arris_geom::{
    Curve, Curve2, Curve2Kind, GeomError, MAX_FIT_SPANS, MeetKind, NurbsCurve,
    PCURVE_SINGULAR_BAND, Surface, intersect_surfaces, pcurve_on, project_to_plane, trace_torus,
};
use arris_math::{Frame, Interval, Point2, Point3, Precision, Tolerance, Vec3};
use proptest::prelude::*;

/// Exact arms: the image matches the curve to rounding at the scale.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// Parameters the image is checked at per case, both ends included.
const CHECKS: usize = 64;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// A region every traced section of these tests lies in; the closed
/// forms ignore it.
fn within() -> arris_math::Aabb {
    arris_math::Aabb {
        min: [-100.0; 3],
        max: [100.0; 3],
    }
}

/// Which analytic curve to build in the plane.
#[derive(Debug, Clone, Copy)]
enum Flat {
    Line,
    Circle { flipped: bool },
    Ellipse { flipped: bool },
}

fn flat() -> impl Strategy<Value = Flat> {
    prop_oneof![
        Just(Flat::Line),
        any::<bool>().prop_map(|flipped| Flat::Circle { flipped }),
        any::<bool>().prop_map(|flipped| Flat::Ellipse { flipped }),
    ]
}

/// An analytic curve lying in the plane: origin at plane (u0, v0), in-plane
/// axes turned by `angle`, `Z` along or against the plane's normal.
fn curve_in(plane: &Frame, which: Flat, u0: f64, v0: f64, angle: f64, r: f64, extra: f64) -> Curve {
    let origin = plane.to_world(Point3::new(u0, v0, 0.0));
    let x = plane.vec_to_world(Vec3::new(angle.cos(), angle.sin(), 0.0));
    let z = |flipped: bool| {
        if flipped {
            -plane.z().into_inner()
        } else {
            plane.z().into_inner()
        }
    };
    match which {
        Flat::Line => Curve::Line {
            origin,
            direction: arris_math::UnitVec3::new_normalize(x),
        },
        Flat::Circle { flipped } => Curve::Circle {
            frame: Frame::new(origin, z(flipped), x).unwrap(),
            radius: r,
        },
        Flat::Ellipse { flipped } => Curve::Ellipse {
            frame: Frame::new(origin, z(flipped), x).unwrap(),
            major_radius: r + extra,
            minor_radius: r,
        },
    }
}

/// `surface.point(pcurve(t)) == curve.point(t)` at `CHECKS` parameters
/// over `range`, both ends included, to `within`.
fn image_matches(
    pc: &Curve2,
    curve: &Curve,
    surface: &Surface,
    range: Interval,
    within: f64,
) -> Result<(), TestCaseError> {
    for i in 0..=CHECKS {
        let t = range.lerp(i as f64 / CHECKS as f64);
        let uv = pc.point(t);
        let image = surface.point(uv.x, uv.y);
        let expected = curve.point(t);
        let err = (image - expected).norm();
        prop_assert!(
            err <= within,
            "at t = {t}: image {image} vs curve {expected}, off by {err} ({pc:?})"
        );
    }
    Ok(())
}

#[test]
fn every_analytic_curve_in_a_plane_has_an_exact_pcurve() {
    check(
        (
            plane(),
            flat(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.0..=TAU),
            radius(0.1..=10.0),
            finite_f64(0.0..=10.0),
        ),
        |(s, which, u0, v0, angle, r, extra)| {
            let Surface::Plane { frame } = &s else {
                unreachable!("the plane strategy yields planes")
            };
            let c = curve_in(frame, which, u0, v0, angle, r, extra);
            let range = match which {
                Flat::Line => Interval::new(-DEFAULT_SCALE, DEFAULT_SCALE).unwrap(),
                _ => Interval::TURN,
            };
            let pc = pcurve_on(&c, range, &s, tol(), &mut arris_math::Meter::default()).unwrap();
            image_matches(&pc, &c, &s, range, EXACT)?;
            match (which, &pc) {
                (Flat::Line, Curve2::Line { .. }) => {}
                (
                    Flat::Circle { flipped },
                    Curve2::Circle {
                        frame: f2,
                        radius: r2,
                    },
                ) => {
                    prop_assert_eq!(*r2, r);
                    // A circle whose Z opposes the normal runs clockwise in (u, v).
                    prop_assert_eq!(f2.is_right_handed(), !flipped);
                }
                (Flat::Ellipse { flipped }, Curve2::Ellipse { frame: f2, .. }) => {
                    prop_assert_eq!(f2.is_right_handed(), !flipped);
                }
                _ => prop_assert!(false, "{which:?} gave {:?}", pc.kind()),
            }
            // Lifting the curve off the plane by more than the tolerance is
            // refused, naming the distance.
            let lifted = c.transformed(&arris_math::Isometry::from_translation(
                3.0 * tol().linear * frame.z().into_inner(),
            ));
            let err = pcurve_on(&lifted, range, &s, tol(), &mut arris_math::Meter::default())
                .unwrap_err();
            let off = matches!(err, arris_geom::GeomError::NotOnSurface { .. });
            prop_assert!(off, "{err}");
            Ok(())
        },
    );
}

#[test]
fn a_nurbs_in_a_plane_has_its_control_points_projected() {
    check(
        (plane(), nurbs_curve(), finite_f64(0.0..=1.0)),
        |(s, c, at)| {
            let Surface::Plane { frame } = &s else {
                unreachable!("the plane strategy yields planes")
            };
            // Flatten the control points onto the plane.
            let n = frame.z().into_inner();
            let points: Vec<Point3> = c
                .control_points()
                .iter()
                .map(|p| p - (p - frame.origin()).dot(&n) * n)
                .collect();
            let flat =
                NurbsCurve::new(c.degree(), c.knots().to_vec(), points, c.weights().to_vec())
                    .unwrap();
            let curve = Curve::Nurbs(flat);
            let range = curve.domain();
            let pc =
                pcurve_on(&curve, range, &s, tol(), &mut arris_math::Meter::default()).unwrap();
            prop_assert_eq!(pc.kind(), Curve2Kind::Nurbs);
            image_matches(&pc, &curve, &s, range, EXACT)?;
            let t = range.lerp(at);
            let uv = pc.point(t);
            prop_assert!((s.point(uv.x, uv.y) - curve.point(t)).norm() <= EXACT);
            // The projection agrees, since the curve lies in the plane.
            let projected = project_to_plane(&curve, frame).unwrap();
            prop_assert!((projected.point(t) - uv).norm() <= EXACT);
            Ok(())
        },
    );
}

/// Which plane–cylinder section to cut.
#[derive(Debug, Clone, Copy)]
enum Section {
    Circle,
    Ellipse,
    Rulings,
}

fn section() -> impl Strategy<Value = Section> {
    prop_oneof![
        Just(Section::Circle),
        Just(Section::Ellipse),
        Just(Section::Rulings),
    ]
}

#[test]
fn every_section_of_a_cylinder_has_a_pcurve_on_it() {
    check(
        (
            cylinder(),
            section(),
            unit_vec3(),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-0.9..=0.9),
        ),
        |(s, which, n, height, across)| {
            let Surface::Cylinder {
                frame: cyl,
                radius: r,
            } = &s
            else {
                unreachable!("the cylinder strategy yields cylinders")
            };
            let axis = cyl.z().into_inner();
            let on_axis = cyl.origin() + height * axis;
            let (origin, normal) = match which {
                Section::Circle => (on_axis, if n.z >= 0.0 { axis } else { -axis }),
                Section::Ellipse => {
                    if n.cross(&axis).norm() < 1e-3 || n.dot(&axis).abs() < 1e-3 {
                        return Ok(());
                    }
                    (on_axis, n.into_inner())
                }
                Section::Rulings => {
                    let perp = (n.into_inner() - n.dot(&axis) * axis).normalize();
                    (on_axis + across * r * cyl.x().into_inner(), perp)
                }
            };
            let plane = Surface::Plane {
                frame: Frame::from_z(origin, normal).unwrap(),
            };
            let r = intersect_surfaces(
                &plane,
                &s,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            let crossing = r.points().is_empty()
                && !r.curves().is_empty()
                && r.curves().iter().all(|m| m.kind == MeetKind::Crossing);
            prop_assert!(crossing, "no transversal section for {which:?}: {r:?}");
            for c in r.curves().iter().map(|m| &m.curve) {
                let range = match c {
                    Curve::Line { .. } => Interval::new(-DEFAULT_SCALE, DEFAULT_SCALE).unwrap(),
                    _ => Interval::TURN,
                };
                let pc = pcurve_on(c, range, &s, tol(), &mut arris_math::Meter::default()).unwrap();
                match (c, &pc) {
                    (Curve::Line { .. }, Curve2::Line { direction, .. }) => {
                        // A ruling: constant u.
                        prop_assert_eq!(direction.x, 0.0);
                        image_matches(&pc, c, &s, range, EXACT)?;
                    }
                    (Curve::Circle { .. }, Curve2::Line { direction, origin }) => {
                        // A parallel: constant v, u from the seam offset.
                        prop_assert_eq!(direction.y, 0.0);
                        prop_assert!((0.0..TAU).contains(&origin.x));
                        image_matches(&pc, c, &s, range, EXACT)?;
                    }
                    (Curve::Ellipse { .. }, Curve2::Nurbs(fit)) => {
                        image_matches(&pc, c, &s, range, tol().linear)?;
                        prop_assert_eq!(fit.domain(), range);
                        // Continuous across the seam: u never jumps, and
                        // over the full turn it covers one period.
                        let us: Vec<f64> = (0..=CHECKS)
                            .map(|i| pc.point(range.lerp(i as f64 / CHECKS as f64)).x)
                            .collect();
                        for w in us.windows(2) {
                            prop_assert!((w[1] - w[0]).abs() < PI / 4.0, "u jumps: {us:?}");
                        }
                        let span = us.iter().cloned().fold(f64::MIN, f64::max)
                            - us.iter().cloned().fold(f64::MAX, f64::min);
                        prop_assert!((span - TAU).abs() <= 1e-6, "u covers {span}, not a turn");
                    }
                    _ => prop_assert!(false, "{c:?} gave {:?}", pc.kind()),
                }
            }
            Ok(())
        },
    );
}

/// The property's body: a circle of radius `r` in `circle_frame`
/// projected to the plane of `target` is the ellipse of semi-axes `r` and
/// `r·cos`, through the projection of the circle's point at `t`.
fn circle_projects_to_the_expected_ellipse(
    target: Frame,
    circle_frame: Frame,
    r: f64,
    t: f64,
) -> Result<(), TestCaseError> {
    let circle = Curve::Circle {
        frame: circle_frame,
        radius: r,
    };
    let cos = circle_frame.z().dot(&target.z()).abs();
    let projected = project_to_plane(&circle, &target).unwrap();
    let Curve2::Ellipse {
        frame: f2,
        major_radius,
        minor_radius,
    } = &projected
    else {
        // Only a circle parallel to the target stays a circle.
        let stays_a_circle = matches!(projected, Curve2::Circle { .. });
        prop_assert!(stays_a_circle && cos > 1.0 - 1e-12, "{projected:?}");
        return Ok(());
    };
    prop_assert!(
        (major_radius - r).abs() <= EXACT,
        "major {major_radius} vs {r}"
    );
    prop_assert!(
        (minor_radius - r * cos).abs() <= EXACT,
        "minor {minor_radius} vs {}",
        r * cos
    );
    // The point set: the projection of every circle point lies on
    // the ellipse (its implicit form in the ellipse's own frame).
    let p = target.to_local(circle.point(t));
    let q = f2.to_local(arris_math::Point2::new(p.x, p.y));
    let implicit = (q.x / major_radius).powi(2) + (q.y / minor_radius).powi(2);
    // A rounding of the coordinates moves the implicit form by twice
    // its size over the minor radius, which a circle nearly
    // perpendicular to the plane makes small.
    let rounding = 8.0
        * f64::EPSILON
        * (circle_frame.origin().coords.norm() + target.origin().coords.norm() + r);
    prop_assert!(
        (implicit - 1.0).abs() <= 1e-9 + 2.0 * rounding / minor_radius,
        "implicit {implicit} at t = {t}"
    );
    // The traversal sense is the sign of Z against the normal.
    prop_assert_eq!(
        f2.is_right_handed(),
        circle_frame.z().dot(&target.z()) > 0.0
    );
    // A projected line is the line of projected points.
    let line = Curve::Line {
        origin: circle_frame.origin(),
        direction: circle_frame.x(),
    };
    if let Ok(Curve2::Line { .. }) = project_to_plane(&line, &target) {
        let pl = project_to_plane(&line, &target).unwrap();
        let p3 = target.to_local(line.point(t));
        let proj = pl.project(arris_math::Point2::new(p3.x, p3.y)).unwrap();
        prop_assert!(proj.distance <= EXACT);
    }
    Ok(())
}

#[test]
fn a_circle_projected_to_an_oblique_plane_is_the_expected_ellipse() {
    check(
        (frame(), frame(), radius(0.1..=10.0), finite_f64(0.0..=TAU)),
        |(target, circle_frame, r, t)| {
            circle_projects_to_the_expected_ellipse(target, circle_frame, r, t)
        },
    );
}

/// The direction `(cos θ, sin θ, 0)` of a frame, in the world.
fn radial(frame: &Frame, theta: f64) -> Vec3 {
    frame.vec_to_world(Vec3::new(theta.cos(), theta.sin(), 0.0))
}

/// `a` and `b` are the same angle modulo a turn, to rounding.
fn same_angle(a: f64, b: f64) -> bool {
    let d = (a - b).rem_euclid(TAU);
    d.min(TAU - d) <= 1e-9
}

/// The pcurve is a `Line` with the expected origin and direction, and its
/// image is the curve at the same parameter.
fn line_pcurve(
    pc: &Curve2,
    curve: &Curve,
    surface: &Surface,
    range: Interval,
    origin: arris_math::Point2,
    direction: arris_math::Vec2,
) -> Result<(), TestCaseError> {
    let Curve2::Line {
        origin: o,
        direction: d,
    } = pc
    else {
        prop_assert!(false, "{pc:?} is not a line in (u, v)");
        return Ok(());
    };
    prop_assert_eq!(d.into_inner(), direction, "{:?}", pc);
    if direction.x == 0.0 {
        prop_assert!(same_angle(o.x, origin.x), "u is {} not {}", o.x, origin.x);
        prop_assert!(
            (o.y - origin.y).abs() <= 1e-9,
            "v is {} not {}",
            o.y,
            origin.y
        );
    } else {
        prop_assert!(same_angle(o.x, origin.x), "u is {} not {}", o.x, origin.x);
        prop_assert!(
            (o.y - origin.y).abs() <= 1e-9 || same_angle(o.y, origin.y),
            "v is {} not {}",
            o.y,
            origin.y
        );
    }
    image_matches(pc, curve, surface, range, EXACT)
}

#[test]
fn a_ruling_and_a_parallel_of_a_cone_are_lines_in_uv() {
    check(
        (
            arris_debug::prop::geom::cone(),
            finite_f64(0.0..=TAU),
            finite_f64(0.0..=TAU),
            finite_f64(-5.0..=5.0),
            any::<bool>(),
            any::<bool>(),
        ),
        |(s, u0, beta, v0, up, flip)| {
            let Surface::Cone {
                frame: cone,
                radius,
                half_angle,
            } = &s
            else {
                unreachable!("the cone strategy yields cones")
            };
            let (sin, cos) = (half_angle.sin(), half_angle.cos());
            let axis = cone.z().into_inner();
            // A ruling: the line through the apex in the half-plane u0.
            let sense = if up { 1.0 } else { -1.0 };
            let ruling = Curve::Line {
                origin: s.point(u0, v0),
                direction: arris_math::UnitVec3::new_normalize(
                    sense * (sin * radial(cone, u0) + cos * axis),
                ),
            };
            let range = Interval::new(-4.0, 4.0).unwrap();
            let pc =
                pcurve_on(&ruling, range, &s, tol(), &mut arris_math::Meter::default()).unwrap();
            line_pcurve(
                &pc,
                &ruling,
                &s,
                range,
                arris_math::Point2::new(u0, v0),
                arris_math::Vec2::new(0.0, sense),
            )?;
            // A parallel: the circle at v0, its own X at an arbitrary
            // angle and its Z either way round. Beyond the apex the
            // radial factor `R + v sin α` is negative, and the surface
            // reaches the circle's own `X` at `u + π`.
            let signed = radius + v0 * sin;
            if signed.abs() < 1e-3 {
                return Ok(()); // the apex is not a circle
            }
            let z = if flip { -axis } else { axis };
            let circle = Curve::Circle {
                frame: Frame::new(
                    cone.to_world(Point3::new(0.0, 0.0, v0 * cos)),
                    z,
                    radial(cone, beta),
                )
                .unwrap(),
                radius: signed.abs(),
            };
            let pc = pcurve_on(
                &circle,
                Interval::TURN,
                &s,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            let expected_u = if signed > 0.0 { beta } else { beta + PI };
            line_pcurve(
                &pc,
                &circle,
                &s,
                Interval::TURN,
                arris_math::Point2::new(expected_u.rem_euclid(TAU), v0),
                arris_math::Vec2::new(if flip { -1.0 } else { 1.0 }, 0.0),
            )?;
            Ok(())
        },
    );
}

#[test]
fn a_parallel_and_a_meridian_of_a_sphere_are_lines_in_uv() {
    check(
        (
            arris_debug::prop::geom::sphere(),
            finite_f64(-1.4..=1.4),
            finite_f64(0.0..=TAU),
            finite_f64(0.0..=TAU),
            finite_f64(-0.7..=0.7),
            any::<bool>(),
        ),
        |(s, v0, beta, psi, phi, flip)| {
            let Surface::Sphere {
                frame: sphere,
                radius,
            } = &s
            else {
                unreachable!("the sphere strategy yields spheres")
            };
            let axis = sphere.z().into_inner();
            // A parallel at the latitude v0.
            let z = if flip { -axis } else { axis };
            let parallel = Curve::Circle {
                frame: Frame::new(
                    sphere.to_world(Point3::new(0.0, 0.0, radius * v0.sin())),
                    z,
                    radial(sphere, beta),
                )
                .unwrap(),
                radius: radius * v0.cos(),
            };
            let pc = pcurve_on(
                &parallel,
                Interval::TURN,
                &s,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            line_pcurve(
                &pc,
                &parallel,
                &s,
                Interval::TURN,
                arris_math::Point2::new(beta, v0),
                arris_math::Vec2::new(if flip { -1.0 } else { 1.0 }, 0.0),
            )?;
            // A meridian: the great circle whose plane holds the axis,
            // its own X at the latitude phi of the half-plane psi + π/2.
            let u = psi + PI / 2.0;
            let cz = radial(sphere, psi);
            let cx = phi.cos() * radial(sphere, u) + phi.sin() * axis;
            let meridian = Curve::Circle {
                frame: Frame::new(sphere.origin(), if flip { -cz } else { cz }, cx).unwrap(),
                radius: *radius,
            };
            // A range that stays clear of the poles: v runs phi ± t.
            let range = Interval::new(-0.7, 0.7).unwrap();
            let pc = pcurve_on(
                &meridian,
                range,
                &s,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            let Curve2::Line { origin, direction } = &pc else {
                prop_assert!(false, "{pc:?} is not a line in (u, v)");
                return Ok(());
            };
            prop_assert_eq!(direction.x, 0.0, "{:?}", pc);
            prop_assert_eq!(direction.y.abs(), 1.0, "{:?}", pc);
            prop_assert!(same_angle(origin.x, u), "u is {} not {}", origin.x, u);
            prop_assert!(
                (origin.y - phi).abs() <= 1e-9,
                "v is {} not {}",
                origin.y,
                phi
            );
            image_matches(&pc, &meridian, &s, range, EXACT)?;
            // Over a full turn the same line still names the curve: the
            // great circle runs up one meridian and down the other, which
            // is where v outside [−π/2, π/2] puts it.
            let pc = pcurve_on(
                &meridian,
                Interval::TURN,
                &s,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            image_matches(&pc, &meridian, &s, Interval::TURN, EXACT)?;
            Ok(())
        },
    );
}

#[test]
fn a_parallel_and_a_tube_circle_of_a_torus_are_lines_in_uv() {
    check(
        (
            arris_debug::prop::geom::torus(),
            finite_f64(0.0..=TAU),
            finite_f64(0.0..=TAU),
            finite_f64(0.0..=TAU),
            finite_f64(0.0..=TAU),
            any::<bool>(),
        ),
        |(s, v0, beta, u0, phi, flip)| {
            let Surface::Torus {
                frame: torus,
                major_radius,
                minor_radius,
            } = &s
            else {
                unreachable!("the torus strategy yields tori")
            };
            let axis = torus.z().into_inner();
            // A circle about the axis at v0.
            let z = if flip { -axis } else { axis };
            let parallel = Curve::Circle {
                frame: Frame::new(
                    torus.to_world(Point3::new(0.0, 0.0, minor_radius * v0.sin())),
                    z,
                    radial(torus, beta),
                )
                .unwrap(),
                radius: major_radius + minor_radius * v0.cos(),
            };
            let pc = pcurve_on(
                &parallel,
                Interval::TURN,
                &s,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            line_pcurve(
                &pc,
                &parallel,
                &s,
                Interval::TURN,
                arris_math::Point2::new(beta, v0),
                arris_math::Vec2::new(if flip { -1.0 } else { 1.0 }, 0.0),
            )?;
            // A circle of the tube at u0, its own X at the tube angle phi.
            let r = radial(torus, u0);
            let cz = r.cross(&axis);
            let cx = phi.cos() * r + phi.sin() * axis;
            let tube = Curve::Circle {
                frame: Frame::new(
                    torus.origin() + *major_radius * r,
                    if flip { -cz } else { cz },
                    cx,
                )
                .unwrap(),
                radius: *minor_radius,
            };
            let pc = pcurve_on(
                &tube,
                Interval::TURN,
                &s,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            let Curve2::Line { origin, direction } = &pc else {
                prop_assert!(false, "{pc:?} is not a line in (u, v)");
                return Ok(());
            };
            prop_assert_eq!(direction.x, 0.0, "{:?}", pc);
            prop_assert_eq!(direction.y.abs(), 1.0, "{:?}", pc);
            prop_assert!(same_angle(origin.x, u0), "u is {} not {}", origin.x, u0);
            prop_assert!(same_angle(origin.y, phi), "v is {} not {}", origin.y, phi);
            prop_assert!((0.0..TAU).contains(&origin.y));
            image_matches(&pc, &tube, &s, Interval::TURN, EXACT)?;
            Ok(())
        },
    );
}

/// A line or a conic tilted from a plane by more than the angular
/// tolerance, and within the linear one of it over the range, has the
/// fitted projection for its pcurve there: its image lies within the
/// tolerance of the curve's own projection, so off the curve by no more
/// than the curve's distance from the plane and the tolerance again. The
/// seam's last stretch before a ball's pole on a face turned 25° from its
/// plane (`boolean/pole-slice-beside-seam-cut`), where the exact arm's
/// circle was 0.19 off; a half circle turned 4.9e-8 about its diameter
/// from the plane of a meridian, 9.8e-8 from it at its middle, which a
/// fit held to the curve itself could not reach; and a line turned 5e-8.
#[test]
fn a_curve_tilted_from_a_plane_has_its_projection_for_a_pcurve() {
    let meridian = Frame::new(Point3::origin(), -Vec3::y(), -Vec3::z()).unwrap();
    let seam = Curve::Circle {
        frame: meridian,
        radius: 2.0,
    };
    let turned = |axis: Vec3, at: Point3, angle: f64| {
        let q = arris_math::nalgebra::UnitQuaternion::from_axis_angle(
            &arris_math::nalgebra::Unit::new_normalize(axis),
            angle,
        );
        let normal = q * Vec3::y();
        Surface::Plane {
            frame: Frame::from_z(at, normal).unwrap(),
        }
    };
    let cases = [
        (
            seam.clone(),
            turned(
                Vec3::new(1.0, 2e-4, 0.0),
                Point3::new(0.0, 0.0, 2.0),
                25f64.to_radians(),
            ),
            Interval::new(3.141503975729147, PI).unwrap(),
        ),
        (
            seam,
            turned(Vec3::z(), Point3::origin(), 4.9e-8),
            Interval::new(0.0, PI).unwrap(),
        ),
        (
            Curve::Line {
                origin: Point3::origin(),
                direction: arris_math::UnitVec3::new_normalize(Vec3::new(1.0, 5e-8, 0.0)),
            },
            turned(Vec3::z(), Point3::origin(), 0.0),
            Interval::new(-1.0, 1.0).unwrap(),
        ),
    ];
    for (curve, surface, range) in cases {
        let Surface::Plane { frame } = &surface else {
            unreachable!()
        };
        let pc = pcurve_on(
            &curve,
            range,
            &surface,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(matches!(pc, Curve2::Nurbs(_)), "{pc:?}");
        let off_plane = (0..=DENSE)
            .map(|i| {
                frame
                    .to_local(curve.point(range.lerp(i as f64 / DENSE as f64)))
                    .z
                    .abs()
            })
            .fold(0.0, f64::max);
        let worst = worst_image(&pc, &curve, &surface, range);
        assert!(off_plane <= tol().linear, "{off_plane}");
        assert!(
            worst <= off_plane + tol().linear,
            "{worst} against {off_plane}"
        );
    }
}

/// Parameters a fitted pcurve's image is held to the curve at.
const DENSE: usize = 2000;

/// The farthest the image of `pc` is from `curve` at [`DENSE`] + 1
/// parameters over `range`, both ends included.
fn worst_image(pc: &Curve2, curve: &Curve, surface: &Surface, range: Interval) -> f64 {
    (0..=DENSE)
        .map(|i| {
            let t = range.lerp(i as f64 / DENSE as f64);
            let uv = pc.point(t);
            (surface.point(uv.x, uv.y) - curve.point(t)).norm()
        })
        .fold(0.0, f64::max)
}

/// `surface` with its frame's origin moved to `origin`.
fn placed(surface: &Surface, origin: Point3) -> Surface {
    let mut out = surface.clone();
    match &mut out {
        Surface::Plane { frame }
        | Surface::Cylinder { frame, .. }
        | Surface::EllipticCylinder { frame, .. }
        | Surface::Cone { frame, .. }
        | Surface::Sphere { frame, .. }
        | Surface::Torus { frame, .. } => *frame = frame.with_origin(origin),
        Surface::Nurbs(_) => {}
    }
    out
}

/// A plane, a cylinder or a sphere: what a section is cut with.
fn cutter() -> impl Strategy<Value = Surface> {
    prop_oneof![plane(), cylinder(), arris_debug::prop::geom::sphere(),]
}

/// A surface and a cutter placed where it meets it: the cutter's origin
/// within its own radius of a point of the surface.
fn cut(base: impl Strategy<Value = Surface>) -> impl Strategy<Value = (Surface, Surface)> {
    (
        base,
        cutter(),
        finite_f64(0.0..=TAU),
        finite_f64(0.0..=1.0),
        unit_vec3(),
        finite_f64(0.0..=0.9),
    )
        .prop_map(|(base, cutter, u, s, direction, reach)| {
            let v = match &base {
                Surface::Sphere { .. } => 2.8 * (s - 0.5),
                Surface::Torus { .. } => TAU * s,
                _ => 10.0 * (s - 0.5),
            };
            let size = match cutter {
                Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => radius,
                _ => 0.0,
            };
            let at = base.point(u, v) + direction.into_inner() * (reach * size);
            (base, placed(&cutter, at))
        })
}

/// The pcurve of `curve` over `range` on `surface` holds the curve within
/// the tolerance at [`DENSE`] parameters, and comes back a whole number
/// of turns from where it started when the range is the curve's period.
fn fits(curve: &Curve, range: Interval, surface: &Surface) -> Result<(), TestCaseError> {
    let pc = match pcurve_on(
        curve,
        range,
        surface,
        tol(),
        &mut arris_math::Meter::default(),
    ) {
        Ok(pc) => pc,
        // Through an apex or a pole: both sides of the split fit.
        Err(GeomError::ThroughSingularity { t, .. }) => {
            prop_assert!(
                range.lo() < t && t < range.hi(),
                "split at {t} of {range:?}"
            );
            for half in [(range.lo(), t), (t, range.hi())] {
                let half = Interval::new(half.0, half.1).unwrap();
                let pc = pcurve_on(
                    curve,
                    half,
                    surface,
                    tol(),
                    &mut arris_math::Meter::default(),
                )
                .map_err(|e| TestCaseError::fail(format!("{half:?} of a split: {e}")))?;
                let off = worst_image(&pc, curve, surface, half);
                prop_assert!(off <= tol().linear, "a split half is {off} off");
            }
            return Ok(());
        }
        Err(e) => {
            return Err(TestCaseError::fail(format!(
                "{curve:?} on {surface:?}: {e}"
            )));
        }
    };
    let off = worst_image(&pc, curve, surface, range);
    prop_assert!(off <= tol().linear, "the image is {off} off {curve:?}");
    if curve.period() == Some(range.length()) {
        // Closed in 3D, so closed in (u, v) up to the surface's periods:
        // a seam crossing is continuous, and a whole turn is a whole turn.
        let by = pc.point(range.hi()) - pc.point(range.lo());
        for (k, period) in surface.period().into_iter().enumerate() {
            let turns = period.map_or(0.0, |p| (by[k] / p).round() * p);
            // In length: an angle of 1e-6 is a micrometre on these radii
            // only beside a pole, where `u` is that loose.
            prop_assert!(
                (by[k] - turns).abs() <= 1e-5,
                "parameter {k} comes back {} from a whole turn",
                by[k] - turns
            );
        }
    }
    Ok(())
}

/// Every curve of the section of the pair has a pcurve on the first.
fn section_has_pcurves((base, cutter): (Surface, Surface)) -> Result<(), TestCaseError> {
    let meets = match intersect_surfaces(
        &base,
        &cutter,
        &within(),
        tol(),
        &mut arris_math::Meter::default(),
    ) {
        Ok(meets) => meets,
        // A pose the tracers refuse by name is not this property's.
        Err(GeomError::DegenerateSection { .. }) => return Ok(()),
        Err(e) => return Err(TestCaseError::fail(format!("no section: {e}"))),
    };
    for c in meets.curves().iter().map(|m| &m.curve) {
        let range = match c {
            Curve::Line { .. } => Interval::new(-DEFAULT_SCALE, DEFAULT_SCALE).unwrap(),
            _ => c.domain(),
        };
        fits(c, range, &base)?;
    }
    Ok(())
}

arris_debug::prop_shards! {
    /// An elliptic cylinder's sections: rulings and its own ellipse exact,
    /// every other conic and every quartic fitted.
    every_section_of_an_elliptic_cylinder_has_a_pcurve_on_it [shard_0 shard_1 shard_2 shard_3]
        (pair) = cut(arris_debug::prop::geom::elliptic_cylinder()) => { section_has_pcurves(pair) }
}

arris_debug::prop_shards! {
    /// A cone's sections: rulings and parallels exact, the ellipses, the
    /// hyperbolas and the quartics fitted, past the apex or beside it.
    every_section_of_a_cone_has_a_pcurve_on_it [shard_0 shard_1 shard_2 shard_3]
        (pair) = cut(arris_debug::prop::geom::cone()) => { section_has_pcurves(pair) }
}

arris_debug::prop_shards! {
    /// A sphere's sections: small circles about any axis, and the
    /// quartics a cylinder cuts.
    every_section_of_a_sphere_has_a_pcurve_on_it [shard_0 shard_1 shard_2 shard_3]
        (pair) = cut(arris_debug::prop::geom::sphere()) => { section_has_pcurves(pair) }
}

arris_debug::prop_shards! {
    /// A torus's sections, unwrapped across both seams.
    every_section_of_a_torus_has_a_pcurve_on_it [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        (pair) = cut(arris_debug::prop::geom::torus()) => { section_has_pcurves(pair) }
}

/// The projected pcurve of every traced branch of a torus section against
/// the tracer's exact `(u, v)`, in length on the torus: by how much the
/// pcurve is farther from the branch's (u, v) than the fitted 3D curve it
/// is the pcurve of is from the branch's points, at worst over the
/// branches. The fitted curve is held to its two surfaces, not to the
/// branch, so along a grazing section it sits off the branch by many
/// tolerances — 1.6e-6 at the default 1e-7, over 1000 poses — and the
/// pcurve has to follow it there: what the projection itself adds is the
/// difference.
fn against_the_branch(torus: &Surface, cutter: &Surface) -> Result<f64, TestCaseError> {
    let &Surface::Torus {
        major_radius,
        minor_radius,
        ..
    } = torus
    else {
        unreachable!("the torus strategy yields tori")
    };
    let Ok(trace) = trace_torus(torus, cutter, tol(), &mut arris_math::Meter::default()) else {
        return Ok(0.0);
    };
    let meets = intersect_surfaces(
        torus,
        cutter,
        &within(),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .map_err(|e| TestCaseError::fail(format!("traced, and no section: {e}")))?;
    let mut excess = 0.0f64;
    for branch in trace.branches() {
        let domain = branch.domain();
        // The fitted curve of this branch: same parameter, and the
        // nearest to its points — nearest rather than within the
        // tolerance, since a fit held to its two surfaces slides along a
        // grazing one by more than that.
        let away = |c: &Curve| -> f64 {
            [0.21, 0.67]
                .iter()
                .map(|&s| (c.point(domain.lerp(s)) - branch.point(domain.lerp(s))).norm())
                .sum()
        };
        let fitted = (meets.curves().iter())
            .map(|m| &m.curve)
            .filter(|c| matches!(c, Curve::Nurbs(_)) && c.domain() == domain)
            .min_by(|a, b| away(a).total_cmp(&away(b)));
        let Some(fitted) = fitted else {
            // A pose a closed form owns — a plane across the axis, a
            // sphere on it — is exact circles, with nothing fitted.
            let exact = |m: &arris_geom::MeetCurve| !matches!(m.curve, Curve::Nurbs(_));
            prop_assert!(
                meets.curves().iter().all(exact),
                "a branch with no fitted curve"
            );
            continue;
        };
        let pc = pcurve_on(
            fitted,
            domain,
            torus,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .map_err(|e| TestCaseError::fail(format!("no pcurve: {e}")))?;
        let turns = |t: f64| -> Option<[f64; 2]> {
            let (p, q) = (pc.point(t), branch.uv(t)?);
            Some([((p.x - q.x) / TAU).round(), ((p.y - q.y) / TAU).round()])
        };
        let whole = turns(domain.lo());
        // A closed branch's own (u, v) wraps back at the end of its
        // period, where the pcurve is whole turns on.
        let last = if branch.is_closed() { DENSE - 1 } else { DENSE };
        for i in 0..=last {
            let t = domain.lerp(i as f64 / DENSE as f64);
            let (p, Some(q)) = (pc.point(t), branch.uv(t)) else {
                return Err(TestCaseError::fail("a torus branch without uv"));
            };
            // One unwrapping against another: the same whole turns apart
            // all the way along.
            prop_assert_eq!(turns(t), whole, "the unwrapping slips at t = {}", t);
            let [ku, kv] = whole.unwrap_or([0.0; 2]);
            let apart = |s: f64| {
                let q = branch.uv(s).unwrap_or(q);
                let du = (p.x - q.x - ku * TAU) * (major_radius + minor_radius * q.y.cos());
                let dv = (p.y - q.y - kv * TAU) * minor_radius;
                du.hypot(dv)
            };
            // The nearest of the branch about the same parameter, by
            // golden section over a window that holds it: of its own
            // (u, v) to the pcurve, and of its points to the 3D curve.
            let reach = domain.length() / DENSE as f64;
            let window = ((t - reach).max(domain.lo()), (t + reach).min(domain.hi()));
            let on_curve = fitted.point(t);
            let lifted = |s: f64| (branch.point(s) - on_curve).norm();
            let in_uv = nearest(&apart, t, window);
            let in_space = nearest(&lifted, t, window);
            excess = excess.max(in_uv - in_space);
        }
    }
    Ok(excess)
}

/// The least of `f` over `window`, by golden section.
fn nearest(f: &dyn Fn(f64) -> f64, middle: f64, (mut a, mut b): (f64, f64)) -> f64 {
    let ratio = 0.5 * (5f64.sqrt() - 1.0);
    for _ in 0..40 {
        let (x1, x2) = (b - ratio * (b - a), a + ratio * (b - a));
        if f(x1) <= f(x2) {
            b = x2;
        } else {
            a = x1;
        }
    }
    // The arcs of a branch join within the tracer's tolerance, not to
    // rounding, so `f` may step inside the window: the window's middle
    // is the same parameter, and no worse than what the search found.
    f(0.5 * (a + b)).min(f(middle))
}

arris_debug::prop_shards! {
    /// The torus's pcurve comes from the projection and from nowhere
    /// else: measured against the tracer's exact (u, v), it adds at most
    /// 0.82 of the tolerance (over 1000 poses) to what the fitted 3D
    /// curve is off the exact branch already — and the branch's own
    /// (u, v) would be *that* far off the curve the edge carries, up to
    /// sixteen tolerances, so it is no pcurve of it.
    a_torus_pcurve_by_projection_is_the_tracers_uv [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((torus, cutter)) = cut(arris_debug::prop::geom::torus()) => {
            let added = against_the_branch(&torus, &cutter)?;
            prop_assert!(added <= tol().linear, "{added} beyond the fitted curve's own");
            Ok(())
        }
}

/// The unit sphere at rest, and the small circle of angular radius
/// `0.4` whose nearest point to the north pole misses it by `miss`, at
/// `t = π/2`, the pole outside it.
fn beside_the_pole(miss: f64) -> (Surface, Curve) {
    let sphere = Surface::Sphere {
        frame: Frame::world(),
        radius: 1.0,
    };
    let (across, tilt) = (0.4f64, 0.4 + miss);
    let axis = Vec3::new(tilt.sin(), 0.0, tilt.cos());
    let circle = Curve::Circle {
        frame: Frame::new(Point3::origin() + across.cos() * axis, axis, Vec3::y()).unwrap(),
        radius: across.sin(),
    };
    (sphere, circle)
}

#[test]
fn a_small_circle_through_a_pole_is_split_there_and_both_halves_fit() {
    // Through it exactly, and within the band of it.
    for miss in [0.0, 0.9 * PCURVE_SINGULAR_BAND * tol().linear] {
        let (sphere, circle) = beside_the_pole(miss);
        let err = pcurve_on(
            &circle,
            Interval::TURN,
            &sphere,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_err();
        let GeomError::ThroughSingularity { t, .. } = err else {
            panic!("{err}")
        };
        assert!((t - PI / 2.0).abs() < 1e-6, "split at {t}");
        // The circle passes the pole along −y: it arrives from u = π/2
        // and leaves towards u = 3π/2, the limit of each half's u, and
        // each half ends on the pole's own v.
        let halves = [
            (Interval::new(0.0, t).unwrap(), true, PI / 2.0),
            (Interval::new(t, TAU).unwrap(), false, 3.0 * PI / 2.0),
        ];
        for (range, ends_there, u) in halves {
            let pc = pcurve_on(
                &circle,
                range,
                &sphere,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            let off = worst_image(&pc, &circle, &sphere, range);
            assert!(off <= tol().linear, "{off} off over {range:?}");
            let at = pc.point(if ends_there { range.hi() } else { range.lo() });
            assert_eq!(at.y, PI / 2.0);
            assert!(same_angle(at.x, u), "arrives with u = {}, not {u}", at.x);
        }
    }
}

#[test]
fn a_circle_cut_at_the_pole_it_runs_through_fits_as_one_range() {
    // What a boolean paves it into (ADR-0021): one block from the pole
    // round to the pole, a range that ends on the one singular point
    // twice, half a turn of `u` apart — each end read on its own side.
    let (sphere, circle) = beside_the_pole(0.0);
    let range = Interval::new(PI / 2.0, PI / 2.0 + TAU).unwrap();
    let pc = pcurve_on(
        &circle,
        range,
        &sphere,
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let off = worst_image(&pc, &circle, &sphere, range);
    assert!(off <= tol().linear, "{off} off");
    for (t, u) in [(range.lo(), 3.0 * PI / 2.0), (range.hi(), PI / 2.0)] {
        let at = pc.point(t);
        assert_eq!(at.y, PI / 2.0);
        assert!(same_angle(at.x, u), "u = {} at {t}, not {u}", at.x);
    }
}

#[test]
fn a_half_meridian_is_a_line_in_the_sphere_s_own_latitudes() {
    // From the south pole to the north by way of `t = π`: `v = t − π`,
    // and not the `t + π` the circle's phase alone gives — a sphere's
    // `v` is no periodic parameter, and nothing downstream could put a
    // whole turn of it back.
    let sphere = Surface::Sphere {
        frame: Frame::world(),
        radius: 0.5,
    };
    let circle = Curve::Circle {
        frame: Frame::new(Point3::origin(), Vec3::y(), Vec3::x()).unwrap(),
        radius: 0.5,
    };
    for range in [
        Interval::new(PI / 2.0, 3.0 * PI / 2.0).unwrap(),
        Interval::new(-PI / 2.0, PI / 2.0).unwrap(),
        Interval::new(3.0 * PI / 2.0, 5.0 * PI / 2.0).unwrap(),
    ] {
        let pc = pcurve_on(
            &circle,
            range,
            &sphere,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(matches!(pc, Curve2::Line { .. }), "{pc:?}");
        let off = worst_image(&pc, &circle, &sphere, range);
        assert!(off <= tol().linear, "{off} off over {range:?}");
        for i in 0..=8 {
            let v = pc.point(range.lerp(f64::from(i) / 8.0)).y;
            assert!(v.abs() <= PI / 2.0 + 1e-12, "v = {v} over {range:?}");
        }
    }
}

#[test]
fn a_circle_beside_a_pole_fits_from_the_band_to_a_hundredth_of_the_radius() {
    // `u` swings by nearly π over a stretch as long as the miss, and the
    // fit follows it by halving spans there: measured at the default
    // tolerance, 917 control points just outside the band, 901 at one
    // tolerance, 767 at 1e-5, 479 at 1e-3 and 293 at 1e-2 of the radius
    // — a quarter of `MAX_FIT_SPANS` at the worst.
    let band = PCURVE_SINGULAR_BAND * tol().linear;
    for miss in [1.04 * band, tol().linear, 1e-6, 1e-5, 1e-4, 1e-3, 1e-2] {
        let (sphere, circle) = beside_the_pole(miss);
        let pc = pcurve_on(
            &circle,
            Interval::TURN,
            &sphere,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("a miss of {miss}: {e}"));
        let off = worst_image(&pc, &circle, &sphere, Interval::TURN);
        assert!(off <= tol().linear, "a miss of {miss}: {off} off");
        let Curve2::Nurbs(fit) = &pc else {
            panic!("{pc:?}")
        };
        let points = fit.control_points().len();
        assert!(points <= MAX_FIT_SPANS / 4, "a miss of {miss}: {points}");
        // The pole is outside the circle: `u` comes back to where it
        // started, having swung by the angle the circle fills as seen
        // from the pole, which is all but a half turn.
        let by = pc.point(TAU) - pc.point(0.0);
        assert!(by.norm() <= 1e-9, "{by}");
        let us: Vec<f64> = (0..=DENSE)
            .map(|i| pc.point(TAU * i as f64 / DENSE as f64).x)
            .collect();
        let swing = us.iter().cloned().fold(f64::MIN, f64::max)
            - us.iter().cloned().fold(f64::MAX, f64::min);
        assert!(
            PI / 2.0 < swing && swing < PI,
            "a miss of {miss}: u swings {swing}"
        );
    }
}

/// The cone at rest with its apex at the origin: half-angle `0.5`, the
/// reference circle of radius `2` above it.
fn cone_on_its_apex() -> Surface {
    let (radius, half_angle) = (2.0, 0.5f64);
    Surface::Cone {
        frame: Frame::from_z(Point3::new(0.0, 0.0, radius / half_angle.tan()), Vec3::z()).unwrap(),
        radius,
        half_angle,
    }
}

#[test]
fn a_cone_carries_what_runs_through_its_apex_and_what_passes_beside_it() {
    let cone = cone_on_its_apex();
    let long = Interval::new(-5.0, 5.0).unwrap();
    // A plane through the apex and inside the cone: two rulings, each an
    // exact line in (u, v) from one nappe to the other.
    let through = Surface::Plane {
        frame: Frame::from_z(Point3::origin(), Vec3::new(1.0, 0.0, 0.2)).unwrap(),
    };
    let meets = intersect_surfaces(
        &cone,
        &through,
        &within(),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(meets.curves().len(), 2, "{meets:?}");
    for ruling in meets.curves().iter().map(|m| &m.curve) {
        let pc = pcurve_on(
            ruling,
            long,
            &cone,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(pc.kind(), Curve2Kind::Line);
        assert!(worst_image(&pc, ruling, &cone, long) <= EXACT);
    }
    // The same ruling as a spline has no exact arm: it is through the
    // apex, and either side of it fits at the ruling's own u.
    let tilt = 0.5f64;
    let ruling = |s: f64| Point3::new(s * tilt.sin(), 0.0, s * tilt.cos());
    let spline = Curve::Nurbs(
        NurbsCurve::new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![ruling(-3.0), ruling(2.0)],
            vec![1.0, 1.0],
        )
        .unwrap(),
    );
    let err = pcurve_on(
        &spline,
        Interval::UNIT,
        &cone,
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap_err();
    let GeomError::ThroughSingularity { t, .. } = err else {
        panic!("{err}")
    };
    assert!((t - 0.6).abs() < 1e-9, "split at {t}");
    for (range, u) in [
        (Interval::new(0.0, t).unwrap(), PI),
        (Interval::new(t, 1.0).unwrap(), 0.0),
    ] {
        let pc = pcurve_on(
            &spline,
            range,
            &cone,
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(worst_image(&pc, &spline, &cone, range) <= tol().linear);
        // The lower nappe is reached with a negative radial factor, so
        // the ruling keeps one u across the apex on the surface and the
        // projection names the far side of it on each nappe.
        for s in [0.0, 0.5, 1.0] {
            let at = pc.point(range.lerp(s));
            assert!(same_angle(at.x, u) || same_angle(at.x, u + PI), "{at}");
        }
    }
    // Beside the apex: an oblique plane above it cuts an ellipse that
    // winds once round the axis, and a steep one a hair off it cuts a
    // hyperbola whose near branch swings round the apex.
    let above = Surface::Plane {
        frame: Frame::from_z(Point3::new(0.0, 0.0, 1.0), Vec3::new(0.3, 0.0, 1.0)).unwrap(),
    };
    let beside = Surface::Plane {
        frame: Frame::from_z(Point3::new(1e-4, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.2)).unwrap(),
    };
    for (plane, curves) in [(above, 1), (beside, 2)] {
        let meets = intersect_surfaces(
            &cone,
            &plane,
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert_eq!(meets.curves().len(), curves, "{meets:?}");
        for c in meets.curves().iter().map(|m| &m.curve) {
            let pc = pcurve_on(
                c,
                c.domain(),
                &cone,
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            assert_eq!(pc.kind(), Curve2Kind::Nurbs);
            let off = worst_image(&pc, c, &cone, c.domain());
            assert!(off <= tol().linear, "{off} off {c:?}");
        }
    }
}

#[test]
fn a_villarceau_circle_and_a_spiric_oval_are_unwrapped_across_both_seams() {
    let (major_radius, minor_radius) = (2.0, 0.5f64);
    let ring = Surface::Torus {
        frame: Frame::world(),
        major_radius,
        minor_radius,
    };
    // A Villarceau circle: in the bitangent plane, of the major radius,
    // its centre the minor radius off the axis. Once round the axis and
    // once round the tube.
    let tilt = (minor_radius / major_radius).asin();
    let normal = Vec3::new(-tilt.sin(), 0.0, tilt.cos());
    let villarceau = Curve::Circle {
        frame: Frame::new(Point3::new(0.0, minor_radius, 0.0), normal, Vec3::y()).unwrap(),
        radius: major_radius,
    };
    let pc = pcurve_on(
        &villarceau,
        Interval::TURN,
        &ring,
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert!(worst_image(&pc, &villarceau, &ring, Interval::TURN) <= tol().linear);
    let by = pc.point(TAU) - pc.point(0.0);
    assert!(
        (by.x.abs() - TAU).abs() <= 1e-6 && (by.y.abs() - TAU).abs() <= 1e-6,
        "{by}"
    );
    // A spiric oval about the outer equator's point on the seam of both
    // parameters: it crosses each seam twice and comes back to its start.
    let wall = Surface::Plane {
        frame: Frame::from_z(Point3::new(2.3, 0.0, 0.0), Vec3::x()).unwrap(),
    };
    let meets = intersect_surfaces(
        &ring,
        &wall,
        &within(),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(meets.curves().len(), 1, "{meets:?}");
    let oval = &meets.curves()[0].curve;
    let range = oval.domain();
    let pc = pcurve_on(oval, range, &ring, tol(), &mut arris_math::Meter::default()).unwrap();
    assert!(worst_image(&pc, oval, &ring, range) <= tol().linear);
    let by = pc.point(range.hi()) - pc.point(range.lo());
    assert!(by.norm() <= 1e-6, "{by}");
    let (mut us, mut vs) = (Vec::new(), Vec::new());
    for i in 0..=CHECKS {
        let uv = pc.point(range.lerp(i as f64 / CHECKS as f64));
        us.push(uv.x);
        vs.push(uv.y);
    }
    for k in [&us, &vs] {
        let (lo, hi) = (
            k.iter().cloned().fold(f64::MAX, f64::min),
            k.iter().cloned().fold(f64::MIN, f64::max),
        );
        // Continuous through the seam: a narrow band about a multiple of
        // a turn, never a jump to the other end of [0, 2π).
        assert!(hi - lo < PI, "{lo} to {hi}");
        let seam = (0.5 * (lo + hi) / TAU).round() * TAU;
        assert!(
            lo < seam && seam < hi,
            "[{lo}, {hi}] does not straddle a seam"
        );
    }
}

#[test]
fn every_surface_fits_what_it_has_no_arm_for_and_a_curve_off_it_is_not_on_it() {
    let tol = tol();
    // Every analytic surface fits what it has no exact arm for: a
    // rational quarter of a sphere's equator, as a spline.
    let sphere_frame = Frame::from_z(Point3::new(1.0, 2.0, 3.0), Vec3::z()).unwrap();
    let sphere = Surface::Sphere {
        frame: sphere_frame,
        radius: 5.0,
    };
    let arc = Curve::Nurbs(
        NurbsCurve::new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![
                sphere_frame.to_world(Point3::new(5.0, 0.0, 0.0)),
                sphere_frame.to_world(Point3::new(5.0, 5.0, 0.0)),
                sphere_frame.to_world(Point3::new(0.0, 5.0, 0.0)),
            ],
            vec![1.0, 0.5f64.sqrt(), 1.0],
        )
        .unwrap(),
    );
    let pc = pcurve_on(
        &arc,
        Interval::UNIT,
        &sphere,
        tol,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!(pc.kind(), Curve2Kind::Nurbs);
    assert!(worst_image(&pc, &arc, &sphere, Interval::UNIT) <= tol.linear);
    assert!((pc.point(1.0) - Point2::new(PI / 2.0, 0.0)).norm() <= 1e-9);
    // A curve off a surface is `NotOnSurface`, whatever the surface.
    let cone = cone_on_its_apex();
    let torus = Surface::Torus {
        frame: Frame::world(),
        major_radius: 5.0,
        minor_radius: 1.0,
    };
    let wall = Surface::EllipticCylinder {
        frame: Frame::world(),
        major_radius: 3.0,
        minor_radius: 2.0,
    };
    let lifted = Curve::Circle {
        frame: Frame::from_z(Point3::new(0.0, 0.0, 50.0), Vec3::z()).unwrap(),
        radius: 2.5,
    };
    for surface in [&sphere, &cone, &torus, &wall] {
        let err = pcurve_on(
            &lifted,
            Interval::TURN,
            surface,
            tol,
            &mut arris_math::Meter::default(),
        )
        .unwrap_err();
        assert!(
            matches!(err, GeomError::NotOnSurface { .. }),
            "{surface:?}: {err}"
        );
    }
}

/// No arm of the table is `Unsupported` any more: a NURBS surface answers
/// every curve kind, with a pcurve or a named error — a random curve is
/// off a random surface, and says where (`tests/nurbs_pcurve.rs` holds
/// the curves that are on one).
#[test]
fn a_nurbs_surface_answers_every_curve() {
    check(
        (
            arris_debug::prop::geom::nurbs_surface(),
            arris_debug::prop::geom::curve(),
        ),
        |(s, c)| {
            let range = Interval::new(0.0, 1.0).unwrap();
            match pcurve_on(
                &c,
                range,
                &Surface::Nurbs(s),
                tol(),
                &mut arris_math::Meter::default(),
            ) {
                Ok(_) | Err(GeomError::NotOnSurface { .. }) => Ok(()),
                Err(err) => Err(TestCaseError::fail(format!("{err}"))),
            }
        },
    );
}

/// The nightly of 2026-10-03: a circle of radius 0.1 at a cosine of 1e-4
/// to a plane a hundred away. The minor radius is 1e-5, so the rounding of
/// the coordinates (1e-14) is 2e-9 of the implicit form: the property's
/// conditioning, not an error of the projection.
#[test]
fn a_circle_nearly_perpendicular_to_a_far_plane_is_held_to_its_conditioning() {
    let target = Frame::from_orthonormal(
        Point3::new(47.79953633735795, -87.92253844625674, 27.641203251716362),
        Vec3::new(
            -0.8694537858378548,
            0.4838003234894943,
            -0.09993678643864023,
        ),
        Vec3::new(0.3694873617434822, 0.502564788899189, -0.7816058613332902),
        Vec3::new(
            -0.32791645857453217,
            -0.7164955547316217,
            -0.6157149634739788,
        ),
    )
    .unwrap();
    let circle = Frame::from_orthonormal(
        Point3::new(-70.81861402369576, 0.0, 0.0),
        Vec3::new(-0.776538802537452, -0.5682703804410408, -0.2721328037321022),
        Vec3::new(0.5658260918001677, -0.4389570136662874, -0.6979667427544975),
        Vec3::new(0.27717922359341496, -0.6959780994159008, 0.6624093622085834),
    )
    .unwrap();
    circle_projects_to_the_expected_ellipse(target, circle, 0.1, 2.8077318630220667).unwrap();
}
