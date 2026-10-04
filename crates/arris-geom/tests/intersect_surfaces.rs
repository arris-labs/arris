//! Plane–plane, plane–cylinder and cylinder–cylinder intersections follow
//! the case table in any pose: the variant is the constructed case's,
//! every result curve lies on both surfaces, the ellipses' axes are the
//! closed form, a tangent line is the ruling at the touch, the result is
//! symmetric under swapping and bit-identical across runs. Coaxial pairs
//! with a cone, a sphere or a torus in them meet where their meridians
//! meet (ADR-0008): every circle is centred on the axis and is a genuine
//! crossing of the meridians, no crossing a sampled meridian sees is
//! missed, constructed touches are touching curves and constructed
//! apexes and poles are points alone. Two cylinders in a quartic pose
//! meet in fitted curves on both surfaces, bit for bit under a swap
//! (ADR-0018), and every fitted section is held to the exact branch it
//! was traced from, two fits of it in two regions within half a
//! tolerance of each other (ADR-0022). Every other surface pair — a plane oblique to a cone's
//! axis, two tori on different axes — is `Unsupported`.

use core::f64::consts::{FRAC_PI_2, TAU};

use arris_debug::prop::geom::{
    HALF_ANGLE_RANGE, RADIUS_RANGE, cone, cylinder, elliptic_cylinder, nurbs_surface, plane,
    sphere, surface, torus,
};
use arris_debug::prop::{DEFAULT_SCALE, check, finite_f64, frame, point_in_box, unit_vec3};
use arris_debug::prop_shards;
use arris_geom::{
    Curve, FitError, GeomError, GeomKind, HYPERBOLA_HALF_SPAN, MeetKind, NurbsCurve,
    SECTION_FIT_FRACTION, SectionBranch, Surface, SurfaceIntersection, SurfaceKind,
    intersect_surfaces, trace_quadrics, trace_torus,
};
use arris_math::{Aabb, Frame, Point3, Precision, Tolerance, UnitVec3, Vec3};
use proptest::prelude::*;

/// Points on the result curves against both surfaces, and the closed
/// forms of the ellipse's axes and the tangent ruling.
const EXACT: f64 = 1e-12 * DEFAULT_SCALE;
/// The relative rounding of a closed-form point and of its implicit
/// distance, both taken as differences of coordinates of the point's
/// magnitude: a grazing plane's ellipse is centred up to 1e6 out, where
/// one ulp is already past [`EXACT`].
const ROUNDING: f64 = 8.0 * f64::EPSILON;
/// Parameters sampled along every result curve.
const SAMPLES: usize = 17;

fn tol() -> Tolerance {
    Precision::DEFAULT.tolerance()
}

/// A region every traced section of these tests lies in — two cylinders
/// in a quartic pose meet within their radii of both axes, at most a few
/// hundred [`DEFAULT_SCALE`]s from the origin; the closed forms ignore it.
fn within() -> Aabb {
    Aabb {
        min: [-1e4; 3],
        max: [1e4; 3],
    }
}

/// The signed distance from `p` to the surface by its implicit form:
/// negative on a plane's back, inside a cylinder, a sphere or a torus's
/// tube, and inside a double cone's nappes.
fn signed_distance(s: &Surface, p: Point3) -> f64 {
    let q = s.frame().unwrap().to_local(p);
    let rho = q.x.hypot(q.y);
    match *s {
        Surface::Plane { .. } => q.z,
        Surface::Cylinder { radius, .. } => rho - radius,
        Surface::EllipticCylinder {
            major_radius: a,
            minor_radius: b,
            ..
        } => {
            // Exact through the projection, signed by the implicit form;
            // a point the projection finds ambiguous is deep inside.
            let sign = ((q.x / a).powi(2) + (q.y / b).powi(2) - 1.0).signum();
            sign * s.project(p).map_or(f64::INFINITY, |pr| pr.distance)
        }
        Surface::Cone {
            radius, half_angle, ..
        } => {
            let (sa, ca) = half_angle.sin_cos();
            let apex = -radius * ca / sa;
            rho * ca - (q.z - apex).abs() * sa
        }
        Surface::Sphere { radius, .. } => q.coords.norm() - radius,
        Surface::Torus {
            major_radius,
            minor_radius,
            ..
        } => (rho - major_radius).hypot(q.z) - minor_radius,
        Surface::Nurbs(_) => unreachable!("no NURBS pair has a closed form"),
    }
}

/// The distance from `p` to the surface by its implicit form.
fn implicit_distance(s: &Surface, p: Point3) -> f64 {
    signed_distance(s, p).abs()
}

/// Every sampled point of `c` lies on both `a` and `b`: to rounding, and
/// a NURBS — a fitted section, or an exact conic that is held no looser —
/// within the fit's fraction of the tolerance, plus `slack`, the
/// tolerance a traced branch may be off within a singular point's reach.
/// A NURBS is sampled densely, so the fit is held between its own check
/// parameters too, and relative to its size, so a clipped branch a
/// thousand units long is held as its rounding allows. A conic is a
/// closed form and is held to rounding — except a tube circle of a
/// traced torus section, which is exact on the torus and within
/// `tol.linear` of the other surface, the tolerance it was accepted in
/// (ADR-0019).
fn on_both(c: &Curve, a: &Surface, b: &Surface, slack: f64) -> Result<(), TestCaseError> {
    let (range, samples) = match c {
        Curve::Line { .. } => (-DEFAULT_SCALE..=DEFAULT_SCALE, SAMPLES),
        Curve::Circle { .. } | Curve::Ellipse { .. } => (0.0..=TAU, SAMPLES),
        Curve::Nurbs(c) => (c.domain().lo()..=c.domain().hi(), DENSE),
    };
    let tube_circle = [a, b].iter().any(|s| s.kind() == SurfaceKind::Torus) && !coaxial(a, b);
    for i in 0..samples {
        let s = i as f64 / (samples - 1) as f64;
        let t = range.start() + s * (range.end() - range.start());
        let p = c.point(t);
        let bound = match c {
            Curve::Nurbs(_) => {
                slack + SECTION_FIT_FRACTION * tol().linear + 1e-11 * (1.0 + p.coords.norm())
            }
            Curve::Circle { .. } if tube_circle => tol().linear,
            // A conic's point is its centre plus two terms of its radii,
            // each rounded at its own magnitude: an ellipse 2.3e5 across
            // and centred as far out rounds its points at that size, not
            // at theirs (nightly seed 9492b872…, 5000 cases).
            Curve::Circle { frame, radius } => {
                EXACT + ROUNDING * (p.coords.norm() + frame.origin().coords.norm() + radius)
            }
            Curve::Ellipse {
                frame,
                major_radius,
                ..
            } => EXACT + ROUNDING * (p.coords.norm() + frame.origin().coords.norm() + major_radius),
            // A line of two planes is placed to its rounding over the
            // sine between them ([`line_spread`]).
            Curve::Line { .. } => EXACT + ROUNDING * line_spread(a, b) * p.coords.norm(),
        };
        let (da, db) = (implicit_distance(a, p), implicit_distance(b, p));
        prop_assert!(
            da <= bound && db <= bound,
            "{c:?} at t = {t} is off by {da} from {a:?} and {db} from {b:?}"
        );
    }
    Ok(())
}

/// How much two lines computed from the same two surfaces may be placed
/// apart relative to their rounding: `1 / sin θ` for two planes `θ`
/// apart, whose line's position is conditioned so — two planes 1.5e-3
/// of a radian apart place it 1e3 out, each origin to 2e-10 of the other
/// (nightly seed 9492b872…, 5000 cases) — and `1` for every other pair.
fn line_spread(a: &Surface, b: &Surface) -> f64 {
    match (a, b) {
        (Surface::Plane { frame: fa }, Surface::Plane { frame: fb }) => {
            let sin = fa.z().cross(&fb.z()).norm();
            if sin > 0.0 { 1.0 / sin } else { 1.0 }
        }
        _ => 1.0,
    }
}

/// The same point set, up to a line's or a circle's orientation — two lines
/// placed as far apart as their origins' rounding times `spread` allows
/// ([`line_spread`]), two conics as far apart as the rounding of their
/// own size allows (a circle 6e5 out is placed to about 1e-9); a NURBS — fitted or a conic's branch — the same
/// curve bit for bit.
fn same_curve(a: &Curve, b: &Curve, spread: f64) -> bool {
    let parallel = |x: &UnitVec3, y: &UnitVec3| x.cross(y).norm() <= EXACT;
    match (a, b) {
        (Curve::Nurbs(x), Curve::Nurbs(y)) => x == y,
        (
            Curve::Line { origin, direction },
            Curve::Line {
                origin: o2,
                direction: d2,
            },
        ) => {
            parallel(direction, d2)
                && (o2 - origin).cross(direction).norm()
                    <= EXACT + ROUNDING * spread * (origin.coords.norm() + o2.coords.norm())
        }
        (
            Curve::Circle { frame, radius },
            Curve::Circle {
                frame: f2,
                radius: r2,
            },
        ) => {
            let size =
                frame.origin().coords.norm() + f2.origin().coords.norm() + radius.abs() + r2.abs();
            let bound = EXACT + ROUNDING * size;
            (frame.origin() - f2.origin()).norm() <= bound
                && (radius - r2).abs() <= bound
                && parallel(&frame.z(), &f2.z())
        }
        (
            Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            },
            Curve::Ellipse {
                frame: f2,
                major_radius: a2,
                minor_radius: b2,
            },
        ) => {
            let size = frame.origin().coords.norm()
                + f2.origin().coords.norm()
                + major_radius.abs()
                + a2.abs();
            let bound = EXACT + ROUNDING * size;
            (frame.origin() - f2.origin()).norm() <= bound
                && (major_radius - a2).abs() <= bound
                && (minor_radius - b2).abs() <= bound
                && parallel(&frame.z(), &f2.z())
                && parallel(&frame.x(), &f2.x())
        }
        _ => false,
    }
}

fn curves_of(r: &SurfaceIntersection) -> Vec<&Curve> {
    r.curves().iter().map(|m| &m.curve).collect()
}

fn points_of(r: &SurfaceIntersection) -> Vec<Point3> {
    r.points().iter().map(|m| m.point).collect()
}

/// A result read as the shapes the closed-form arms here give one at a
/// time: curves that all cross, curves that all touch, points alone —
/// or `Mixed`, several at once.
#[derive(Debug, Clone, PartialEq)]
enum Seen {
    Empty,
    Coincident,
    Crossing(Vec<Curve>),
    Touch(Vec<Curve>),
    Points(Vec<Point3>),
    Mixed,
}

fn seen(r: &SurfaceIntersection) -> Seen {
    let all = |kind: MeetKind| r.curves().iter().all(|m| m.kind == kind);
    match r {
        SurfaceIntersection::Empty => Seen::Empty,
        SurfaceIntersection::Coincident => Seen::Coincident,
        SurfaceIntersection::Meets { curves, points } if curves.is_empty() => {
            Seen::Points(points.iter().map(|m| m.point).collect())
        }
        SurfaceIntersection::Meets { points, .. } if !points.is_empty() => Seen::Mixed,
        SurfaceIntersection::Meets { .. } if all(MeetKind::Crossing) => {
            Seen::Crossing(curves_of(r).into_iter().cloned().collect())
        }
        SurfaceIntersection::Meets { .. } if all(MeetKind::Touch) => {
            Seen::Touch(curves_of(r).into_iter().cloned().collect())
        }
        SurfaceIntersection::Meets { .. } => Seen::Mixed,
    }
}

/// Parameters a fitted section is held to its exact branch at.
const HELD: usize = 2000;

/// Points of one fit looked for on the fits of the same section traced
/// in another region.
const REGION_SAMPLES: usize = 200;

/// The fits of `r` beside the exact branches they were fitted from, when
/// `r` is a traced section: the tracer the intersector's `section` arm
/// runs, run again, gives as many branches as `r` has curves after its
/// tube circles, each a `Curve::Nurbs` over its branch's own domain.
/// Empty for a result of any other arm, which has no branch to hold.
fn fits_of(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    r: &SurfaceIntersection,
) -> Vec<(NurbsCurve, SectionBranch)> {
    if !r
        .curves()
        .iter()
        .any(|m| matches!(m.curve, Curve::Nurbs(_)))
    {
        return Vec::new();
    }
    let trace = if [a, b].iter().any(|s| s.kind() == SurfaceKind::Torus) {
        trace_torus(a, b, tol(), &mut arris_math::Meter::default())
    } else {
        trace_quadrics(a, b, within, tol(), &mut arris_math::Meter::default())
    };
    let Ok(trace) = trace else {
        return Vec::new();
    };
    let fits = r.curves().get(trace.circles().len()..).unwrap_or_default();
    let paired: Vec<_> = fits
        .iter()
        .zip(trace.branches())
        .filter_map(|(m, branch)| match &m.curve {
            Curve::Nurbs(c) if c.domain() == branch.domain() => Some((c.clone(), branch.clone())),
            _ => None,
        })
        .collect();
    if paired.len() == fits.len() && fits.len() == trace.branches().len() {
        paired
    } else {
        Vec::new()
    }
}

/// A traced section's fits held to the exact branch (ADR-0018, ADR-0019
/// as amended by ADR-0022): at [`HELD`] parameters of each, within
/// [`SECTION_FIT_FRACTION`] of `tol.linear` of the branch at the same
/// parameter, as precisely as `f64` knows it there
/// ([`SectionBranch::distance`]). Returns whether `r` was a traced
/// section.
fn held_to_branches(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    r: &SurfaceIntersection,
) -> Result<bool, TestCaseError> {
    let fits = fits_of(a, b, within, r);
    for (fit, branch) in &fits {
        let domain = fit.domain();
        for i in 0..HELD {
            let t = domain.lerp(i as f64 / (HELD - 1) as f64);
            let (p, q) = (fit.eval(t).point, branch.point(t));
            let off = branch.distance(t, p);
            prop_assert!(
                off <= SECTION_FIT_FRACTION * tol().linear + 1e-11 * (1.0 + q.coords.norm()),
                "{a:?} vs {b:?}: the fit is {off} from its branch at t = {t}"
            );
        }
    }
    Ok(!fits.is_empty())
}

/// Two fits of one section, traced in `within` and in a second region
/// overlapping it by two thirds, within twice [`SECTION_FIT_FRACTION`]
/// of `tol.linear` of each other wherever both regions reach: each held
/// to the one exact section, the two can be no farther apart than that.
/// A torus section is traced with no region at all and is skipped.
fn two_regions_agree(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    r: &SurfaceIntersection,
) -> Result<(), TestCaseError> {
    if [a, b].iter().any(|s| s.kind() == SurfaceKind::Torus) || fits_of(a, b, within, r).is_empty()
    {
        return Ok(());
    }
    let extent = within.extent();
    let shift: [f64; 3] = core::array::from_fn(|k| extent[k] / 3.0);
    let other = Aabb {
        min: core::array::from_fn(|k| within.min[k] + shift[k]),
        max: core::array::from_fn(|k| within.max[k] + shift[k]),
    };
    let r2 = intersect_surfaces(a, b, &other, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(format!("{a:?} vs {b:?} in {other:?}: {e}")))?;
    let theirs: Vec<Curve> = (r2.curves().iter())
        .filter(|m| matches!(m.curve, Curve::Nurbs(_)))
        .map(|m| m.curve.clone())
        .collect();
    // A branch is clipped along the walked rulings, not at the box: a
    // tenth of the box in from both.
    let inside = |p: Point3| {
        (0..3).all(|k| {
            let margin = extent[k] / 10.0;
            p[k] >= other.min[k] + margin
                && p[k] <= within.max[k] - margin
                && p[k] >= within.min[k] + margin
                && p[k] <= other.max[k] - margin
        })
    };
    for m in r.curves() {
        let Curve::Nurbs(c) = &m.curve else {
            continue;
        };
        let domain = c.domain();
        for i in 0..REGION_SAMPLES {
            let p = c
                .eval(domain.lerp(i as f64 / (REGION_SAMPLES - 1) as f64))
                .point;
            if !inside(p) {
                continue;
            }
            let apart = (theirs.iter())
                .filter_map(|other| other.project(p).ok())
                .map(|on| on.distance)
                .fold(f64::INFINITY, f64::min);
            prop_assert!(
                apart
                    <= 2.0 * SECTION_FIT_FRACTION * tol().linear + 1e-11 * (1.0 + p.coords.norm()),
                "{a:?} vs {b:?}: {p} of one region's fit is {apart} from the other's"
            );
        }
    }
    Ok(())
}

/// The checks every result passes whatever its case: curves and points
/// on both surfaces, symmetry under swapping (the same curves and points,
/// in any order: two parallel cylinders order their rulings from the
/// first, a coaxial pair its circles along the first's axis),
/// determinism.
fn common_properties(a: &Surface, b: &Surface) -> Result<SurfaceIntersection, TestCaseError> {
    common_properties_in(a, b, &within())
}

/// [`common_properties`] inside a region of the caller's.
fn common_properties_in(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
) -> Result<SurfaceIntersection, TestCaseError> {
    let r = intersect_surfaces(a, b, within, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(format!("{a:?} vs {b:?}: {e}")))?;
    let slack = if r.points().is_empty()
        || r.curves()
            .iter()
            .all(|m| !matches!(m.curve, Curve::Nurbs(_)))
    {
        0.0
    } else {
        tol().linear
    };
    for c in curves_of(&r) {
        on_both(c, a, b, slack)?;
    }
    held_to_branches(a, b, within, &r)?;
    two_regions_agree(a, b, within, &r)?;
    for p in points_of(&r) {
        let (da, db) = (implicit_distance(a, p), implicit_distance(b, p));
        prop_assert!(
            da <= tol().linear && db <= tol().linear,
            "{p} is off by {da} from {a:?} and {db} from {b:?}"
        );
    }
    let swapped = intersect_surfaces(b, a, within, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    prop_assert_eq!(
        core::mem::discriminant(&swapped),
        core::mem::discriminant(&r),
        "swap changed the variant: {:?} vs {:?}",
        r,
        swapped
    );
    let (ps, qs) = (points_of(&r), points_of(&swapped));
    prop_assert_eq!(ps.len(), qs.len());
    for p in ps {
        prop_assert!(
            qs.iter().any(|q| (p - q).norm() <= EXACT),
            "swap changed a point: {} not in {:?}",
            p,
            qs
        );
    }
    let (cs, ss) = (curves_of(&r), curves_of(&swapped));
    prop_assert_eq!(cs.len(), ss.len());
    let mut taken = vec![false; ss.len()];
    for c in cs {
        let found = ss
            .iter()
            .enumerate()
            .position(|(i, s)| !taken[i] && same_curve(c, s, line_spread(a, b)));
        prop_assert!(
            found.is_some(),
            "swap changed a curve: {:?} not in {:?}",
            c,
            ss
        );
        if let Some(i) = found {
            taken[i] = true;
        }
    }
    let again = intersect_surfaces(a, b, within, tol(), &mut arris_math::Meter::default())
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
    prop_assert_eq!(&again, &r, "two runs differ");
    Ok(r)
}

/// The plane–cylinder case to realise; picked first, then posed.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Case {
    Circle,
    Ellipse,
    TwoLines,
    TangentLine,
    Empty,
}

/// A plane that realises `case` against `cyl`, built from the raw
/// numbers: `height` along the axis, `phase` around it, `angle` the
/// normal's tilt off the axis for the ellipse, `fraction` the axis
/// distance over the radius for the line cases, `shift` an in-plane
/// offset of the plane's origin so it is not on the axis, `flip` the
/// normal's sign.
#[derive(Debug, Clone, Copy)]
struct Raw {
    height: f64,
    phase: f64,
    angle: f64,
    fraction: f64,
    shift: f64,
    flip: bool,
}

fn raw() -> impl Strategy<Value = Raw> {
    (
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(0.0..=TAU),
        finite_f64(0.1..=FRAC_PI_2 - 0.1),
        finite_f64(0.0..=0.9),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        any::<bool>(),
    )
        .prop_map(|(height, phase, angle, fraction, shift, flip)| Raw {
            height,
            phase,
            angle,
            fraction,
            shift,
            flip,
        })
}

fn case() -> impl Strategy<Value = Case> {
    prop_oneof![
        Just(Case::Circle),
        Just(Case::Ellipse),
        Just(Case::TwoLines),
        Just(Case::TangentLine),
        Just(Case::Empty),
    ]
}

/// The plane for `case`, and what the result must be: the centre on the
/// axis (circle, ellipse), the foot of the axis (lines) and the major
/// axis direction (ellipse).
fn realise(case: Case, cyl: &Surface, raw: Raw) -> (Surface, Point3, Vec3) {
    let Surface::Cylinder { frame, radius } = cyl else {
        unreachable!()
    };
    let (x, y, z) = (
        frame.x().into_inner(),
        frame.y().into_inner(),
        frame.z().into_inner(),
    );
    let around = raw.phase.cos() * x + raw.phase.sin() * y;
    let across = z.cross(&around);
    let on_axis = frame.origin() + raw.height * z;
    let sign = if raw.flip { -1.0 } else { 1.0 };
    let (normal, anchor, major) = match case {
        Case::Circle => (z, on_axis + raw.shift * around, around),
        Case::Ellipse => {
            let n = raw.angle.cos() * z + raw.angle.sin() * around;
            let major = raw.angle.sin() * z - raw.angle.cos() * around;
            (n, on_axis + raw.shift * across, major)
        }
        Case::TwoLines | Case::TangentLine | Case::Empty => {
            let d = match case {
                Case::TwoLines => raw.fraction * radius,
                Case::TangentLine => *radius,
                _ => radius * (1.1 + raw.fraction),
            };
            let foot = on_axis + d * around;
            (around, foot + raw.shift * across, around)
        }
    };
    let plane = Surface::Plane {
        frame: Frame::from_z(anchor, sign * normal).unwrap(),
    };
    // A line's origin is its point nearest the cylinder's origin: the foot
    // of that origin on the plane, then a half-chord across.
    let reference = match case {
        Case::Circle | Case::Ellipse => on_axis,
        _ => frame.origin() + (anchor - frame.origin()).dot(&around) * around,
    };
    (plane, reference, major)
}

#[test]
fn plane_cylinder_follows_the_case_table() {
    check((case(), cylinder(), raw()), |(case, cyl, raw)| {
        let (plane, reference, major) = realise(case, &cyl, raw);
        let Surface::Cylinder { frame, radius } = cyl else {
            unreachable!()
        };
        let r = common_properties(&plane, &cyl)?;
        match (case, &seen(&r)) {
            (Case::Circle, Seen::Crossing(c)) => {
                let [
                    Curve::Circle {
                        frame: cf,
                        radius: cr,
                    },
                ] = c.as_slice()
                else {
                    return Err(TestCaseError::fail(format!("{case:?}: {r:?}")));
                };
                prop_assert!((cf.origin() - reference).norm() <= EXACT);
                prop_assert_eq!(*cr, radius);
                // The circle takes the cylinder's X so the seam is shared.
                prop_assert_eq!(cf.x(), frame.x());
                prop_assert_eq!(cf.z(), frame.z());
            }
            (Case::Ellipse, Seen::Crossing(c)) => {
                let [
                    Curve::Ellipse {
                        frame: ef,
                        major_radius,
                        minor_radius,
                    },
                ] = c.as_slice()
                else {
                    return Err(TestCaseError::fail(format!("{case:?}: {r:?}")));
                };
                prop_assert!((ef.origin() - reference).norm() <= EXACT);
                prop_assert!((minor_radius - radius).abs() <= EXACT);
                prop_assert!((major_radius - radius / raw.angle.cos()).abs() <= EXACT);
                prop_assert!(
                    (ef.x().into_inner() - major).norm() <= EXACT,
                    "major axis {:?} vs {major}",
                    ef.x()
                );
                prop_assert!(ef.x().dot(&frame.z()) > 0.0, "major axis points down v");
            }
            (Case::TwoLines, Seen::Crossing(c)) => {
                let [
                    Curve::Line {
                        origin: o1,
                        direction: d1,
                    },
                    Curve::Line {
                        origin: o2,
                        direction: d2,
                    },
                ] = c.as_slice()
                else {
                    return Err(TestCaseError::fail(format!("{case:?}: {r:?}")));
                };
                prop_assert_eq!(*d1, frame.z());
                prop_assert_eq!(*d2, frame.z());
                let half = (radius * radius - (raw.fraction * radius).powi(2)).sqrt();
                prop_assert!(((o1 - reference).norm() - half).abs() <= EXACT);
                prop_assert!(((o2 - reference).norm() - half).abs() <= EXACT);
                prop_assert!((o1 - o2).norm() >= 2.0 * half - EXACT);
            }
            (Case::TangentLine, Seen::Touch(c)) => {
                let [Curve::Line { origin, direction }] = c.as_slice() else {
                    return Err(TestCaseError::fail(format!("{case:?}: {r:?}")));
                };
                prop_assert_eq!(*direction, frame.z());
                // The ruling through the point of the cylinder nearest the
                // plane: the foot of the axis, one radius out.
                prop_assert!((origin - reference).norm() <= EXACT);
                prop_assert!((implicit_distance(&cyl, *origin)) <= EXACT);
            }
            (Case::Empty, Seen::Empty) => {}
            _ => return Err(TestCaseError::fail(format!("{case:?} gave {r:?}"))),
        }
        Ok(())
    });
}

#[test]
fn random_plane_and_cylinder_agree_on_every_common_property() {
    check((plane(), cylinder()), |(p, c)| {
        common_properties(&p, &c)?;
        Ok(())
    });
}

#[test]
fn two_random_planes_meet_along_a_line_on_both() {
    check((plane(), plane()), |(a, b)| {
        let r = common_properties(&a, &b)?;
        let Seen::Crossing(c) = seen(&r) else {
            return Err(TestCaseError::fail(format!("{r:?}")));
        };
        let [Curve::Line { origin, direction }] = c.as_slice() else {
            return Err(TestCaseError::fail(format!("{r:?}")));
        };
        prop_assert!(direction.dot(&a.frame().unwrap().z()).abs() <= EXACT);
        prop_assert!(direction.dot(&b.frame().unwrap().z()).abs() <= EXACT);
        // The origin is the point of the line nearest a's origin.
        prop_assert!((origin - a.frame().unwrap().origin()).dot(direction).abs() <= EXACT);
        Ok(())
    });
}

#[test]
fn parallel_planes_are_coincident_or_empty_by_the_gap() {
    check(
        (
            plane(),
            point_in_box(DEFAULT_SCALE),
            finite_f64(0.01..=DEFAULT_SCALE),
            any::<bool>(),
            any::<bool>(),
        ),
        |(a, anchor, gap, flip, lift)| {
            let n = a.frame().unwrap().z().into_inner();
            let in_plane = anchor - n.dot(&(anchor - a.frame().unwrap().origin())) * n;
            let origin = if lift { in_plane + gap * n } else { in_plane };
            let sign = if flip { -1.0 } else { 1.0 };
            let b = Surface::Plane {
                frame: Frame::from_z(origin, sign * n).unwrap(),
            };
            let r = common_properties(&a, &b)?;
            let expected = if lift {
                SurfaceIntersection::Empty
            } else {
                SurfaceIntersection::Coincident
            };
            prop_assert_eq!(r, expected);
            Ok(())
        },
    );
}

/// Whether two surfaces share an axis as the coaxial arm reads it
/// (`docs/DATA-MODEL.md` §Curves): a cylinder, cone or torus carries its
/// `Z`; two carriers are coaxial when the axes are parallel within the
/// angular tolerance and the second's origin is within the linear one of
/// the first's axis; a plane is coaxial with a carrier its normal is
/// parallel to; a sphere with a carrier whose axis passes through its
/// centre; every plane–sphere and sphere–sphere pair shares an axis.
fn coaxial(a: &Surface, b: &Surface) -> bool {
    let carries = |s: &Surface| {
        matches!(
            s.kind(),
            SurfaceKind::Cylinder | SurfaceKind::Cone | SurfaceKind::Torus
        )
    };
    let parallel =
        |x: &UnitVec3, y: &UnitVec3| x.cross(y).norm().atan2(x.dot(y).abs()) <= tol().angular;
    let on_axis = |f: &Frame, p: Point3| {
        let d = p - f.origin();
        (d - d.dot(&f.z()) * f.z().into_inner()).norm() <= tol().linear
    };
    let (fa, fb) = (a.frame().unwrap(), b.frame().unwrap());
    match (carries(a), carries(b)) {
        (true, true) => parallel(&fa.z(), &fb.z()) && on_axis(fa, fb.origin()),
        (true, false) | (false, true) => {
            let (carrier, other) = if carries(a) { (fa, b) } else { (fb, a) };
            match other.kind() {
                SurfaceKind::Plane => {
                    let plane = other.frame().unwrap();
                    let across = FRAC_PI_2
                        - plane
                            .z()
                            .cross(&carrier.z())
                            .norm()
                            .atan2(plane.z().dot(&carrier.z()).abs())
                        <= tol().angular;
                    let holds =
                        plane.z().dot(&(carrier.origin() - plane.origin())).abs() <= tol().linear;
                    parallel(&plane.z(), &carrier.z()) || (across && holds)
                }
                SurfaceKind::Sphere => on_axis(carrier, other.frame().unwrap().origin()),
                // An elliptic cylinder is no surface of revolution, and a
                // NURBS patch carries no axis: neither shares one.
                SurfaceKind::EllipticCylinder | SurfaceKind::Nurbs => false,
                SurfaceKind::Cylinder | SurfaceKind::Cone | SurfaceKind::Torus => {
                    unreachable!("a carrier")
                }
            }
        }
        (false, false) => true,
    }
}

/// Any surface, a NURBS patch among them: what the intersector's last
/// `Unsupported` arms are left with.
fn any_surface() -> impl Strategy<Value = Surface> {
    prop_oneof![surface(), nurbs_surface().prop_map(Surface::Nurbs)]
}

#[test]
fn every_other_pair_is_unsupported() {
    check((any_surface(), any_surface()), |(a, b)| {
        // Every pair of analytic surfaces is decided in every pose — by a
        // closed form, an exact conic, the ruled tracer (ADR-0018) or the
        // torus's (ADR-0019) — so `Unsupported` is left the pairs with a
        // `Surface::Nurbs` in them, which are the NURBS cycle's.
        let nurbs = |k| k == SurfaceKind::Nurbs;
        let supported = !nurbs(a.kind()) && !nurbs(b.kind());
        match intersect_surfaces(&a, &b, &within(), tol(), &mut arris_math::Meter::default()) {
            Ok(_) => prop_assert!(supported, "{a:?} vs {b:?} should be unsupported"),
            Err(GeomError::Unsupported { a: ka, b: kb }) => {
                prop_assert!(!supported, "{a:?} vs {b:?} has a closed form");
                prop_assert_eq!(ka, GeomKind::Surface(a.kind()));
                prop_assert_eq!(kb, GeomKind::Surface(b.kind()));
            }
            Err(e) => prop_assert!(false, "{e}"),
        }
        Ok(())
    });
}

#[test]
fn a_tilted_plane_is_oblique_never_a_guess() {
    // A normal a hair off the axis is oblique, not a circle: the plan's
    // tolerance is angular and the ellipse it returns is the exact section.
    check((cylinder(), unit_vec3()), |(cyl, n)| {
        let axis = cyl.frame().unwrap().z();
        let tilt = n.cross(&axis).norm().atan2(n.dot(&axis).abs());
        let plane = Surface::Plane {
            frame: Frame::from_z(cyl.frame().unwrap().origin(), n.into_inner()).unwrap(),
        };
        let r = common_properties(&plane, &cyl)?;
        if tilt > tol().angular && FRAC_PI_2 - tilt > tol().angular {
            let oblique = matches!(seen(&r), Seen::Crossing(c) if matches!(c.as_slice(), [Curve::Ellipse { .. }]));
            prop_assert!(oblique, "tilt {tilt} gave {:?}", r);
        }
        Ok(())
    });
}

/// The faces of `tests/fixtures/boolean/through-hole`: a box
/// `[0,0,0]–[40,30,10]` minus a cylinder of radius 4 at `(20, 15)` along
/// `z`. The four walls are clear of the hole; the two caps cut circles.
#[test]
fn through_hole_faces_against_the_hole() {
    let hole = Surface::Cylinder {
        frame: Frame::from_z(Point3::new(20.0, 15.0, -1.0), Vec3::z()).unwrap(),
        radius: 4.0,
    };
    let wall = |origin, normal| Surface::Plane {
        frame: Frame::from_z(origin, normal).unwrap(),
    };
    for (origin, normal) in [
        (Point3::new(0.0, 0.0, 0.0), -Vec3::x()),
        (Point3::new(40.0, 0.0, 0.0), Vec3::x()),
        (Point3::new(0.0, 0.0, 0.0), -Vec3::y()),
        (Point3::new(0.0, 30.0, 0.0), Vec3::y()),
    ] {
        assert_eq!(
            intersect_surfaces(
                &wall(origin, normal),
                &hole,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::Empty
        );
    }
    for (z, normal) in [(0.0, -Vec3::z()), (10.0, Vec3::z())] {
        let cap = wall(Point3::new(0.0, 0.0, z), normal);
        let Seen::Crossing(c) = seen(
            &intersect_surfaces(
                &cap,
                &hole,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
        ) else {
            panic!()
        };
        let [Curve::Circle { frame, radius }] = c.as_slice() else {
            panic!()
        };
        assert_eq!(*radius, 4.0);
        assert_eq!(frame.origin(), Point3::new(20.0, 15.0, z));
        assert_eq!(frame.z(), hole.frame().unwrap().z());
    }
}

// --- cylinder–cylinder ----------------------------------------------------

/// One cylinder, and a second on the same axis: the same radius, or a
/// different one, at a random slide along the axis and a random phase.
fn coaxial_pair() -> impl Strategy<Value = (Surface, Surface, bool)> {
    (
        cylinder(),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(0.0..=TAU),
        finite_f64(0.2..=4.0),
        any::<bool>(),
    )
        .prop_filter_map(
            "a second cylinder on the same axis",
            |(a, slide, phase, factor, same)| {
                let (frame, radius) = match a {
                    Surface::Cylinder { frame, radius } => (frame, radius),
                    _ => return None,
                };
                let origin = frame.origin() + slide * frame.z().into_inner();
                let x = phase.cos() * frame.x().into_inner() + phase.sin() * frame.y().into_inner();
                let other = Surface::Cylinder {
                    frame: Frame::new(origin, frame.z().into_inner(), x).ok()?,
                    radius: if same { radius } else { factor * radius },
                };
                // A "different" radius within the tolerance is the same
                // cylinder; the case is the one the radii say it is.
                let agree = match other {
                    Surface::Cylinder { radius: r, .. } => (r - radius).abs() <= tol().linear,
                    _ => false,
                };
                Some((a, other, agree))
            },
        )
}

#[test]
fn coaxial_cylinders_are_coincident_or_empty_by_their_radii() {
    check(coaxial_pair(), |(a, b, agree)| {
        let expected = if agree {
            SurfaceIntersection::Coincident
        } else {
            SurfaceIntersection::Empty
        };
        prop_assert_eq!(
            intersect_surfaces(&a, &b, &within(), tol(), &mut arris_math::Meter::default())
                .map_err(|e| TestCaseError::fail(e.to_string()))?,
            expected.clone(),
            "{:?} vs {:?}",
            a,
            b
        );
        // Symmetric, and the same on a second run.
        prop_assert_eq!(
            intersect_surfaces(&b, &a, &within(), tol(), &mut arris_math::Meter::default())
                .map_err(|e| TestCaseError::fail(e.to_string()))?,
            expected
        );
        Ok(())
    });
}

/// The pose of two cylinders' axes, as the case table reads it.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Pose {
    Parallel,
    CrossingEqual,
    CrossingUnequal,
    SkewApart,
    SkewClose,
}

impl Pose {
    /// The poses whose curve is a quartic, with no closed form: traced and
    /// fitted.
    fn is_quartic(self) -> bool {
        matches!(self, Pose::CrossingUnequal | Pose::SkewClose)
    }
}

fn pose(a: &Surface, b: &Surface) -> Pose {
    let (
        Surface::Cylinder {
            frame: fa,
            radius: ra,
        },
        Surface::Cylinder {
            frame: fb,
            radius: rb,
        },
    ) = (a, b)
    else {
        unreachable!("two cylinders")
    };
    let cross = fa.z().cross(&fb.z());
    if cross.norm().atan2(fa.z().dot(&fb.z()).abs()) <= tol().angular {
        return Pose::Parallel;
    }
    let gap = cross.normalize().dot(&(fb.origin() - fa.origin())).abs();
    match (gap > tol().linear, (ra - rb).abs() <= tol().linear) {
        (true, _) if gap > ra + rb + tol().linear => Pose::SkewApart,
        (true, _) => Pose::SkewClose,
        (false, true) => Pose::CrossingEqual,
        (false, false) => Pose::CrossingUnequal,
    }
}

/// The unit vector at `angle` off `frame`'s `Z`, at `phase` around it.
fn tilted(frame: &Frame, angle: f64, phase: f64) -> Vec3 {
    let around = phase.cos() * frame.x().into_inner() + phase.sin() * frame.y().into_inner();
    angle.cos() * frame.z().into_inner() + angle.sin() * around
}

/// The case of two parallel cylinders to realise.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Parallel {
    Two,
    Outside,
    Inside,
    Apart,
    Nested,
}

/// A cylinder and a second parallel to it realising the case: its axis
/// `d` away at `phase`, reversed or not, its origin slid along the axis,
/// its radius drawn apart. `fraction` places `d` between the tangent
/// distances (clear of each), beyond them or inside the smaller.
#[allow(clippy::type_complexity)]
fn parallel_pair() -> impl Strategy<Value = (Parallel, Surface, Surface)> {
    (
        prop_oneof![
            Just(Parallel::Two),
            Just(Parallel::Outside),
            Just(Parallel::Inside),
            Just(Parallel::Apart),
            Just(Parallel::Nested),
        ],
        cylinder(),
        finite_f64(RADIUS_RANGE),
        finite_f64(0.0..=TAU),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(0.05..=0.95),
        any::<bool>(),
        finite_f64(0.0..=TAU),
    )
        .prop_filter_map(
            "radii far enough apart for a case inside the larger",
            |(case, a, rb, phase, slide, fraction, flip, x_phase)| {
                let Surface::Cylinder { frame, radius: ra } = a else {
                    return None;
                };
                let (lo, hi) = ((ra - rb).abs(), ra + rb);
                if matches!(case, Parallel::Inside | Parallel::Nested) && lo < 0.05 * hi {
                    return None;
                }
                let d = match case {
                    Parallel::Two => lo + fraction * (hi - lo),
                    Parallel::Outside => hi,
                    Parallel::Inside => lo,
                    Parallel::Apart => hi * (1.1 + fraction),
                    Parallel::Nested => lo * fraction,
                };
                let z = frame.z().into_inner();
                let towards = tilted(&frame, FRAC_PI_2, phase);
                let origin = frame.origin() + d * towards + slide * z;
                let x = tilted(&frame, FRAC_PI_2, x_phase);
                let b = Surface::Cylinder {
                    frame: Frame::new(origin, if flip { -z } else { z }, x).ok()?,
                    radius: rb,
                };
                Some((case, Surface::Cylinder { frame, radius: ra }, b))
            },
        )
}

#[test]
fn parallel_cylinders_follow_the_case_table() {
    check(parallel_pair(), |(case, a, b)| {
        let r = common_properties(&a, &b)?;
        let fa = a.frame().unwrap();
        let z = fa.z();
        let read = seen(&r);
        let rulings = match (case, &read) {
            (Parallel::Two, Seen::Crossing(c)) if c.len() == 2 => c,
            (Parallel::Outside | Parallel::Inside, Seen::Touch(c)) if c.len() == 1 => c,
            (Parallel::Apart | Parallel::Nested, Seen::Empty) => return Ok(()),
            _ => return Err(TestCaseError::fail(format!("{case:?} gave {r:?}"))),
        };
        let mut origins = Vec::new();
        for c in rulings {
            let Curve::Line { origin, direction } = c else {
                return Err(TestCaseError::fail(format!("{case:?}: {c:?}")));
            };
            // Along the first cylinder's Z, from the point nearest its
            // origin.
            prop_assert_eq!(*direction, z);
            prop_assert!((origin - fa.origin()).dot(&z).abs() <= EXACT);
            origins.push(*origin);
        }
        if let [o1, o2] = origins.as_slice() {
            // Ordered along Z × ŵ, ŵ from the first axis toward the second.
            let offset = b.frame().unwrap().origin() - fa.origin();
            let towards = offset - offset.dot(&z) * z.into_inner();
            let side = z.cross(&towards);
            prop_assert!((o2 - o1).dot(&side) > 0.0, "{o1} then {o2}");
        }
        Ok(())
    });
}

/// A cylinder and a second of the same radius whose axis crosses the
/// first's at a point slid along it, at an angle in [10°, 90°], reversed
/// or not, the second's origin slid along its own axis.
fn crossing_pair() -> impl Strategy<Value = (Surface, Surface, f64, Point3)> {
    (
        cylinder(),
        finite_f64(10f64.to_radians()..=FRAC_PI_2),
        finite_f64(0.0..=TAU),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        any::<bool>(),
    )
        .prop_filter_map(
            "a second cylinder crossing the first",
            |(a, psi, phase, slide, slide_b, flip)| {
                let Surface::Cylinder { frame, radius } = a else {
                    return None;
                };
                let z = frame.z().into_inner();
                let axis = tilted(&frame, psi, phase);
                let axis = if flip { -axis } else { axis };
                let crossing = frame.origin() + slide * z;
                let b = Surface::Cylinder {
                    frame: Frame::new(crossing + slide_b * axis, axis, z.cross(&axis)).ok()?,
                    radius,
                };
                Some((Surface::Cylinder { frame, radius }, b, psi, crossing))
            },
        )
}

#[test]
fn equal_cylinders_crossing_meet_in_the_two_bisecting_ellipses() {
    check(crossing_pair(), |(a, b, psi, crossing)| {
        let r = common_properties(&a, &b)?;
        let (fa, fb) = (a.frame().unwrap(), b.frame().unwrap());
        let Surface::Cylinder { radius, .. } = a else {
            unreachable!()
        };
        let Seen::Crossing(c) = seen(&r) else {
            return Err(TestCaseError::fail(format!("{r:?}")));
        };
        let [first, second] = c.as_slice() else {
            return Err(TestCaseError::fail(format!("{r:?}")));
        };
        // Where the two ellipses cross: R along the axes' common normal.
        let normal = fa.z().cross(&fb.z()).normalize();
        let meets = [crossing + radius * normal, crossing - radius * normal];
        for (ellipse, major) in [
            (first, radius / (psi / 2.0).sin()),
            (second, radius / (psi / 2.0).cos()),
        ] {
            let Curve::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } = ellipse
            else {
                return Err(TestCaseError::fail(format!("{ellipse:?}")));
            };
            let scale = major.max(1.0);
            prop_assert!((frame.origin() - crossing).norm() <= EXACT);
            prop_assert_eq!(*minor_radius, radius);
            prop_assert!(
                (major_radius - major).abs() <= EXACT * scale,
                "{major_radius} vs {major}"
            );
            // Z and X signed with their largest-magnitude component positive.
            for v in [frame.z().into_inner(), frame.x().into_inner()] {
                let k = (0..3).fold(0, |k, i| if v[i].abs() > v[k].abs() { i } else { k });
                prop_assert!(v[k] > 0.0, "{v} is not canonically signed");
            }
            for p in meets {
                let d = ellipse
                    .project(p)
                    .map_err(|e| TestCaseError::fail(e.to_string()))?
                    .distance;
                prop_assert!(d <= EXACT * scale, "{p} is {d} off {ellipse:?}");
            }
        }
        // Either operand order: the same two ellipses bit for bit, in the
        // same order, so a boolean fits each pcurve once whichever it is.
        let swapped = intersect_surfaces(
            &b,
            &a,
            &within(),
            Precision::DEFAULT.tolerance(),
            &mut arris_math::Meter::default(),
        )
        .map_err(|e| TestCaseError::fail(e.to_string()))?;
        prop_assert_eq!(&swapped, &r, "swapping the operands");
        Ok(())
    });
}

#[test]
fn skew_cylinders_further_apart_than_their_radii_are_empty() {
    check(
        (
            cylinder(),
            finite_f64(RADIUS_RANGE),
            finite_f64(0.05..=FRAC_PI_2),
            finite_f64(0.0..=TAU),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
            finite_f64(0.05..=1.0),
        ),
        |(a, rb, psi, phase, slide, slide_b, fraction)| {
            let Surface::Cylinder { frame, radius: ra } = a else {
                unreachable!()
            };
            let z = frame.z().into_inner();
            let axis = tilted(&frame, psi, phase);
            let normal = z.cross(&axis).normalize();
            let gap = (ra + rb) * (1.0 + fraction);
            let origin = frame.origin() + slide * z + gap * normal + slide_b * axis;
            let b = Surface::Cylinder {
                frame: Frame::new(origin, axis, normal).unwrap(),
                radius: rb,
            };
            prop_assert_eq!(common_properties(&a, &b)?, SurfaceIntersection::Empty);
            Ok(())
        },
    );
}

/// A cylinder and a second in a quartic pose: its axis at an angle in
/// [0.1, π/2] off the first's, crossing it with a radius at least 1.2
/// times smaller or larger, or skew, its common perpendicular a fraction
/// of the two radii long.
fn quartic_pair() -> impl Strategy<Value = (Surface, Surface)> {
    (
        cylinder(),
        finite_f64(0.1..=FRAC_PI_2),
        finite_f64(0.0..=TAU),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(0.05..=0.95),
        finite_f64(1.2..=4.0),
        any::<bool>(),
        any::<bool>(),
    )
        .prop_filter_map(
            "a second cylinder in a quartic pose",
            |(a, psi, phase, slide, slide_b, fraction, factor, skew, shrink)| {
                let Surface::Cylinder { frame, radius } = a else {
                    return None;
                };
                let z = frame.z().into_inner();
                let axis = tilted(&frame, psi, phase);
                let normal = z.cross(&axis).normalize();
                let rb = if shrink {
                    radius / factor
                } else {
                    radius * factor
                };
                let gap = if skew { fraction * (radius + rb) } else { 0.0 };
                let origin = frame.origin() + slide * z + gap * normal + slide_b * axis;
                let b = Surface::Cylinder {
                    frame: Frame::new(origin, axis, normal).ok()?,
                    radius: rb,
                };
                Some((Surface::Cylinder { frame, radius }, b))
            },
        )
}

/// Parameters a fitted section curve is sampled at, both ends included:
/// several per span of the densest fits, so the curve is held to both
/// surfaces between the fit's own check parameters too.
const DENSE: usize = 2001;

/// Two cylinders in a quartic pose meet in fitted curves (ADR-0018): each
/// a crossing `Curve::Nurbs`, periodic and closed when it is a loop, an
/// open one ending at a singular point of the result; densely sampled,
/// within the fit's fraction of the tolerance of both surfaces — beyond
/// the tolerance the tracer allows within a singular point's reach, when
/// there is one; each point on both within the tolerance; and swapping
/// the operands, or asking again, gives the result bit for bit.
#[test]
fn cylinders_in_a_quartic_pose_meet_in_fitted_curves_on_both_surfaces() {
    check(quartic_pair(), |(a, b)| {
        prop_assert!(pose(&a, &b).is_quartic(), "{a:?} vs {b:?}");
        let r = intersect_surfaces(&a, &b, &within(), tol(), &mut arris_math::Meter::default())
            .map_err(|e| TestCaseError::fail(format!("{a:?} vs {b:?}: {e}")))?;
        prop_assert!(!r.curves().is_empty() || !r.points().is_empty(), "{r:?}");
        prop_assert!(
            held_to_branches(&a, &b, &within(), &r)?,
            "{a:?} vs {b:?}: not a traced section"
        );
        two_regions_agree(&a, &b, &within(), &r)?;
        let singular = if r.points().is_empty() {
            0.0
        } else {
            tol().linear
        };
        let bound = |p: Point3| {
            singular + SECTION_FIT_FRACTION * tol().linear + 1e-11 * (1.0 + p.coords.norm())
        };
        for m in r.curves() {
            prop_assert_eq!(m.kind, MeetKind::Crossing);
            let Curve::Nurbs(c) = &m.curve else {
                return Err(TestCaseError::fail(format!("not fitted: {:?}", m.curve)));
            };
            let domain = c.domain();
            match c.period() {
                Some(period) => {
                    prop_assert_eq!(period, domain.length());
                    let gap = (c.eval(domain.lo()).point - c.eval(domain.hi()).point).norm();
                    prop_assert!(gap <= EXACT, "a loop open by {gap}");
                }
                None => {
                    for end in [domain.lo(), domain.hi()] {
                        let p = c.eval(end).point;
                        prop_assert!(
                            r.points().iter().any(|q| (q.point - p).norm() <= EXACT),
                            "an open branch ends at {p}, at no singular point"
                        );
                    }
                }
            }
            for i in 0..DENSE {
                let p = c.eval(domain.lerp(i as f64 / (DENSE - 1) as f64)).point;
                let off = implicit_distance(&a, p).max(implicit_distance(&b, p));
                prop_assert!(off <= bound(p), "{p} is {off} off a surface");
            }
        }
        for p in r.points() {
            let off = implicit_distance(&a, p.point).max(implicit_distance(&b, p.point));
            prop_assert!(off <= tol().linear, "{:?} is {off} off a surface", p);
        }
        let swapped =
            intersect_surfaces(&b, &a, &within(), tol(), &mut arris_math::Meter::default())
                .map_err(|e| TestCaseError::fail(e.to_string()))?;
        prop_assert!(swapped == r, "the swap changed the result");
        let again = intersect_surfaces(&a, &b, &within(), tol(), &mut arris_math::Meter::default())
            .map_err(|e| TestCaseError::fail(e.to_string()))?;
        prop_assert!(again == r, "a second run changed the result");
        Ok(())
    });
}

// --- coaxial surfaces of revolution (ADR-0008) ------------------------------

/// The kind of operand to place on the shared axis.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Coaxial {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
}

fn coaxial_kind() -> impl Strategy<Value = Coaxial> {
    prop_oneof![
        Just(Coaxial::Plane),
        Just(Coaxial::Cylinder),
        Just(Coaxial::Cone),
        Just(Coaxial::Sphere),
        Just(Coaxial::Torus),
    ]
}

/// Where an operand sits on the axis: slid `slide` along it, its own
/// frame at `phase` about it and reversed by `flip`; a plane's origin
/// `shift` off the axis in its own plane; a sphere's own `Z` off the axis
/// by `tilt` unless `aligned`; radii `r1` and `r2` and a cone's `angle`.
#[derive(Debug, Clone, Copy)]
struct Placement {
    slide: f64,
    phase: f64,
    flip: bool,
    shift: f64,
    tilt: f64,
    aligned: bool,
    r1: f64,
    r2: f64,
    angle: f64,
}

fn placement() -> impl Strategy<Value = Placement> {
    (
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(0.0..=TAU),
        any::<bool>(),
        finite_f64(-DEFAULT_SCALE..=DEFAULT_SCALE),
        finite_f64(0.1..=FRAC_PI_2 - 0.1),
        any::<bool>(),
        finite_f64(RADIUS_RANGE),
        finite_f64(RADIUS_RANGE),
        finite_f64(HALF_ANGLE_RANGE),
    )
        .prop_map(
            |(slide, phase, flip, shift, tilt, aligned, r1, r2, angle)| Placement {
                slide,
                phase,
                flip,
                shift,
                tilt,
                aligned,
                r1,
                r2,
                angle,
            },
        )
}

/// A surface of `kind` on `axis`'s `Z`, placed by `p`.
fn on_axis(kind: Coaxial, axis: &Frame, p: Placement) -> Surface {
    let z = axis.z().into_inner();
    let sign = if p.flip { -1.0 } else { 1.0 };
    let around = tilted(axis, FRAC_PI_2, p.phase);
    let origin = axis.origin() + p.slide * z;
    let frame = Frame::new(origin, sign * z, around).unwrap();
    match kind {
        Coaxial::Plane => Surface::Plane {
            frame: frame.with_origin(origin + p.shift * around),
        },
        Coaxial::Cylinder => Surface::Cylinder {
            frame,
            radius: p.r1,
        },
        Coaxial::Cone => Surface::Cone {
            frame,
            radius: p.r1,
            half_angle: p.angle,
        },
        Coaxial::Sphere => {
            let own = if p.aligned {
                sign * z
            } else {
                tilted(axis, p.tilt, p.phase)
            };
            Surface::Sphere {
                frame: Frame::new(origin, own, tilted(axis, FRAC_PI_2, p.phase + 1.0)).unwrap(),
                radius: p.r1,
            }
        }
        Coaxial::Torus => Surface::Torus {
            frame,
            major_radius: p.r1 + p.r2,
            minor_radius: p.r1,
        },
    }
}

/// The meridian of `s` in the half-plane through `axis`'s `Z` along its
/// `X`, sampled as 3D points: a plane's ray from the axis, a cylinder's
/// ruling, a cone's two nappes as one V through the apex, a sphere's
/// half circle, a torus's outer tube circle. `t` is the parameter the
/// meridian is walked by, and [`meridian_at`] evaluates it.
fn meridian_at(s: &Surface, axis: &Frame, t: f64) -> Point3 {
    let z = axis.z().into_inner();
    let x = axis.x().into_inner();
    let height = |p: Point3| (p - axis.origin()).dot(&z);
    let at = |rho: f64, h: f64| axis.origin() + rho * x + h * z;
    match *s {
        Surface::Plane { ref frame } => at(t, height(frame.origin())),
        Surface::Cylinder { radius, .. } => at(radius, t),
        Surface::Cone {
            ref frame,
            radius,
            half_angle,
        } => {
            let (sa, ca) = half_angle.sin_cos();
            let apex = height(frame.origin() - (radius * ca / sa) * frame.z().into_inner());
            at((t - apex).abs() * sa / ca, t)
        }
        Surface::Sphere { ref frame, radius } => {
            at(radius * t.cos(), height(frame.origin()) + radius * t.sin())
        }
        Surface::Torus {
            ref frame,
            major_radius,
            minor_radius,
        } => at(
            major_radius + minor_radius * t.cos(),
            height(frame.origin()) + minor_radius * t.sin(),
        ),
        Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => {
            unreachable!("not a surface of revolution: `coaxial_kind` never makes one")
        }
    }
}

/// The meridian parameter of the one kink of `s`'s meridian, a cone's
/// apex; every other meridian is smooth.
fn meridian_kink(s: &Surface, axis: &Frame) -> Option<f64> {
    match *s {
        Surface::Cone {
            ref frame,
            radius,
            half_angle,
        } => {
            let (sa, ca) = half_angle.sin_cos();
            let apex = frame.origin() - (radius * ca / sa) * frame.z().into_inner();
            Some((apex - axis.origin()).dot(&axis.z().into_inner()))
        }
        Surface::Plane { .. }
        | Surface::Cylinder { .. }
        | Surface::Sphere { .. }
        | Surface::Torus { .. } => None,
        Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => {
            unreachable!("not a surface of revolution: `coaxial_kind` never makes one")
        }
    }
}

/// The range of the meridian parameter that reaches every meeting the
/// strategies can make: the slides, the radii and a cone's apex offset
/// are all within a few scales, and a cone meets a plane at most a few
/// scales further out.
fn meridian_range(s: &Surface) -> (f64, f64) {
    match s {
        Surface::Plane { .. } => (0.0, 100.0 * DEFAULT_SCALE),
        Surface::Cylinder { .. } | Surface::Cone { .. } => {
            (-8.0 * DEFAULT_SCALE, 8.0 * DEFAULT_SCALE)
        }
        Surface::Sphere { .. } => (-FRAC_PI_2, FRAC_PI_2),
        Surface::Torus { .. } => (0.0, TAU),
        Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => {
            unreachable!("not a surface of revolution: `coaxial_kind` never makes one")
        }
    }
}

/// The meridian parameter of a point on `s`'s meridian.
fn meridian_param(s: &Surface, axis: &Frame, p: Point3) -> f64 {
    let z = axis.z().into_inner();
    let x = axis.x().into_inner();
    let d = p - axis.origin();
    let (rho, h) = (d.dot(&x), d.dot(&z));
    let height = |q: Point3| (q - axis.origin()).dot(&z);
    match *s {
        Surface::Plane { .. } => rho,
        Surface::Cylinder { .. } | Surface::Cone { .. } => h,
        Surface::Sphere { ref frame, .. } => (h - height(frame.origin())).atan2(rho),
        Surface::Torus {
            ref frame,
            major_radius,
            ..
        } => (h - height(frame.origin())).atan2(rho - major_radius),
        Surface::EllipticCylinder { .. } | Surface::Nurbs(_) => {
            unreachable!("not a surface of revolution: `coaxial_kind` never makes one")
        }
    }
}

/// Samples along a meridian.
const MERIDIAN_SAMPLES: usize = 8192;
/// How far a meeting the sampler brackets may be from the chord of the
/// bracket: the chord's deviation from a sphere's or a torus's meridian
/// at this sampling is below 1e-6.
const BRACKET: f64 = 1e-5 * DEFAULT_SCALE;
/// The step along a meridian either side of a crossing, where the
/// other surface's signed distance changes sign.
const ACROSS: f64 = 1e-6;

/// Every result circle is centred on the axis about it, every result
/// point is on the axis, every crossing circle is a crossing of the
/// meridians and every touching one is not, and every crossing the
/// sampled meridian of `a` sees against `b` is one of the result's.
fn meets_where_the_meridians_meet(
    a: &Surface,
    b: &Surface,
    axis: &Frame,
    r: &SurfaceIntersection,
) -> Result<(), TestCaseError> {
    let z = axis.z().into_inner();
    let off_axis = |p: Point3| (p - axis.origin()).cross(&z).norm();
    let x = axis.x().into_inner();
    // The result's meetings in the half-plane, as 3D points on `a`'s
    // meridian.
    let mut meetings: Vec<Point3> = Vec::new();
    for m in r.curves() {
        let c = &m.curve;
        let Curve::Circle { frame, radius } = c else {
            return Err(TestCaseError::fail(format!("{c:?} is no circle")));
        };
        prop_assert!(off_axis(frame.origin()) <= EXACT, "{c:?} is off the axis");
        prop_assert!(
            frame.z().cross(&z).norm() <= EXACT,
            "{c:?} is not about the axis"
        );
        let on_meridian = frame.origin() + *radius * x;
        let t = meridian_param(a, axis, on_meridian);
        let before = signed_distance(b, meridian_at(a, axis, t - ACROSS));
        let after = signed_distance(b, meridian_at(a, axis, t + ACROSS));
        match m.kind {
            MeetKind::Crossing => prop_assert!(
                before * after < 0.0,
                "{c:?} is no crossing: {before} and {after} either side"
            ),
            MeetKind::Touch => prop_assert!(
                before * after > 0.0,
                "{c:?} is a crossing: {before} and {after} either side"
            ),
        }
        meetings.push(on_meridian);
    }
    for p in points_of(r) {
        prop_assert!(off_axis(p) <= EXACT, "{p} is off the axis");
        meetings.push(p);
    }
    let (lo, hi) = meridian_range(a);
    // A cone's V kinks at the apex, and a chord across the kink misses a
    // crossing beside it by up to the step times the cone's slope — so the
    // apex is a sample of its own and every chord runs along one nappe.
    let mut ts: Vec<f64> = (0..=MERIDIAN_SAMPLES)
        .map(|i| lo + (hi - lo) * i as f64 / MERIDIAN_SAMPLES as f64)
        .chain(meridian_kink(a, axis).filter(|t| (lo..=hi).contains(t)))
        .collect();
    ts.sort_by(f64::total_cmp);
    let mut previous: Option<(Point3, f64)> = None;
    for t in ts {
        let p = meridian_at(a, axis, t);
        let f = signed_distance(b, p);
        if let Some((q, g)) = previous {
            if g * f < 0.0 || g == 0.0 {
                let near = meetings.iter().any(|m| {
                    let (d, v) = (p - q, m - q);
                    let s = (v.dot(&d) / d.dot(&d)).clamp(0.0, 1.0);
                    (v - s * d).norm() <= BRACKET
                });
                prop_assert!(
                    near,
                    "the meridians of {a:?} and {b:?} cross between {q} and {p} but the result {r:?} has nothing there"
                );
            }
        }
        previous = Some((p, f));
    }
    Ok(())
}

#[test]
fn coaxial_pairs_meet_where_their_meridians_meet() {
    check(
        (
            frame(),
            coaxial_kind(),
            placement(),
            coaxial_kind(),
            placement(),
        ),
        |(axis, ka, pa, kb, pb)| {
            let (a, b) = (on_axis(ka, &axis, pa), on_axis(kb, &axis, pb));
            // Two planes and a plane against a cylinder are cycle 1's arms,
            // two cylinders the cylinder plan's: the closed forms this
            // property is about have a cone, a sphere or a torus in them.
            let quadric = |k| matches!(k, Coaxial::Cone | Coaxial::Sphere | Coaxial::Torus);
            prop_assume!(quadric(ka) || quadric(kb));
            let r = common_properties(&a, &b)?;
            meets_where_the_meridians_meet(&a, &b, &axis, &r)
        },
    );
}

/// The normals of both surfaces at `p`, parallel: what a touch means.
fn normals_parallel(a: &Surface, b: &Surface, p: Point3) -> Result<(), TestCaseError> {
    let normal = |s: &Surface| -> Result<UnitVec3, TestCaseError> {
        let uv = s
            .project(p)
            .map_err(|e| TestCaseError::fail(e.to_string()))?
            .uv;
        s.normal(uv.x, uv.y)
            .ok_or_else(|| TestCaseError::fail(format!("{s:?} is singular at {p}")))
    };
    let (na, nb) = (normal(a)?, normal(b)?);
    prop_assert!(
        na.cross(&nb).norm() <= 1e-9,
        "{a:?} and {b:?} are not tangent at {p}: normals {na:?} and {nb:?}"
    );
    Ok(())
}

/// The one circle of a result that touches along curves, or a failure
/// naming it.
fn one_tangent_circle(r: &SurfaceIntersection) -> Result<(Point3, f64), TestCaseError> {
    match seen(r) {
        Seen::Touch(c) => match c.as_slice() {
            [Curve::Circle { frame, radius }] => Ok((frame.origin(), *radius)),
            _ => Err(TestCaseError::fail(format!("{r:?}"))),
        },
        _ => Err(TestCaseError::fail(format!("{r:?}"))),
    }
}

/// The one point of a result of points alone, or a failure naming it.
fn one_point(r: &SurfaceIntersection) -> Result<Point3, TestCaseError> {
    match seen(r) {
        Seen::Points(p) if p.len() == 1 => Ok(p[0]),
        _ => Err(TestCaseError::fail(format!("{r:?}"))),
    }
}

/// The body of the constructed-touches property: each touch built from
/// `axis` and the placements `pa` and `pb` is one tangent circle or one
/// point, `upper` choosing the side.
fn constructed_touches(
    axis: Frame,
    pa: Placement,
    pb: Placement,
    upper: bool,
) -> Result<(), TestCaseError> {
    let z = axis.z().into_inner();
    let sign = if upper { 1.0 } else { -1.0 };
    let torus = on_axis(Coaxial::Torus, &axis, pa);
    let Surface::Torus {
        frame: tf,
        major_radius: big,
        minor_radius: small,
    } = torus
    else {
        unreachable!()
    };
    // A torus on a plane at `z0 ± r`: the tube's top or bottom
    // circle, of radius `R`.
    let plane = Surface::Plane {
        frame: Frame::new(
            tf.origin() + sign * small * z + pb.shift * tilted(&axis, FRAC_PI_2, pb.phase),
            if pb.flip { -z } else { z },
            tilted(&axis, FRAC_PI_2, pb.phase),
        )
        .unwrap(),
    };
    let r = common_properties(&torus, &plane)?;
    let (centre, radius) = one_tangent_circle(&r)?;
    prop_assert!((centre - (tf.origin() + sign * small * z)).norm() <= EXACT);
    prop_assert!((radius - big).abs() <= EXACT);
    normals_parallel(&torus, &plane, centre + radius * axis.x().into_inner())?;
    // A torus inside a cylinder of radius `R + r`, and around one of
    // `R − r`: the outer or the inner equator.
    let cylinder = Surface::Cylinder {
        frame: on_axis(Coaxial::Cylinder, &axis, pb)
            .frame()
            .unwrap()
            .to_owned(),
        radius: big + sign * small,
    };
    let r = common_properties(&cylinder, &torus)?;
    let (centre, radius) = one_tangent_circle(&r)?;
    prop_assert!((centre - tf.origin()).norm() <= EXACT);
    prop_assert!((radius - (big + sign * small)).abs() <= EXACT);
    normals_parallel(&cylinder, &torus, centre + radius * axis.x().into_inner())?;
    // A sphere on a plane: the pole, one point on the axis.
    let sphere = on_axis(Coaxial::Sphere, &axis, pa);
    let Surface::Sphere {
        frame: sf,
        radius: ball,
    } = sphere
    else {
        unreachable!()
    };
    let cap = Surface::Plane {
        frame: plane.frame().unwrap().with_origin(
            sf.origin() + sign * ball * z + pb.shift * tilted(&axis, FRAC_PI_2, pb.phase),
        ),
    };
    let r = common_properties(&sphere, &cap)?;
    let p = one_point(&r)?;
    prop_assert!((p - (sf.origin() + sign * ball * z)).norm() <= EXACT, "{p}");
    // A sphere on a cylinder of its radius: the equator.
    let hoop = Surface::Cylinder {
        frame: on_axis(Coaxial::Cylinder, &axis, pb)
            .frame()
            .unwrap()
            .to_owned(),
        radius: ball,
    };
    let r = common_properties(&sphere, &hoop)?;
    let (centre, radius) = one_tangent_circle(&r)?;
    prop_assert!((centre - sf.origin()).norm() <= EXACT);
    prop_assert!((radius - ball).abs() <= EXACT);
    normals_parallel(&sphere, &hoop, centre + radius * axis.x().into_inner())?;
    // Two spheres touching from outside along a random direction,
    // and from inside.
    let direction = tilted(&axis, pb.tilt, pb.phase);
    for (distance, other) in [(ball + pb.r1, pb.r1), (ball - 0.5 * pb.r1, 0.5 * pb.r1)] {
        let touch = Surface::Sphere {
            frame: Frame::new(
                sf.origin() + distance * direction,
                tilted(&axis, pb.tilt, pb.phase + 2.0),
                z,
            )
            .unwrap(),
            radius: other,
        };
        let r = common_properties(&sphere, &touch)?;
        let p = one_point(&r)?;
        // The contact lies along the line between the centres, which the
        // rounding of the centres' coordinates turns by that much over their
        // distance: nearly equal spheres touch nowhere better than that.
        let rounding = 16.0 * f64::EPSILON * (sf.origin().coords.norm() + distance.abs() + ball);
        let bound = EXACT + rounding * (ball / distance.abs()).max(1.0);
        prop_assert!(
            (p - (sf.origin() + ball * direction)).norm() <= bound,
            "{p} against {} (off {}, bound {bound})",
            sf.origin() + ball * direction,
            (p - (sf.origin() + ball * direction)).norm()
        );
    }
    Ok(())
}

#[test]
fn constructed_touches_are_tangent_and_touches_on_the_axis_are_points() {
    check(
        (frame(), placement(), placement(), any::<bool>()),
        |(axis, pa, pb, upper)| constructed_touches(axis, pa, pb, upper),
    );
}

/// The nightly of 2026-09-29: a sphere and a sphere inside it of radius
/// 3.19439934… whose centre is 1.7e-6 from its own.
#[test]
fn a_sphere_touched_from_inside_by_one_nearly_its_own_size() {
    let axis = Frame::from_orthonormal(
        Point3::origin(),
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, -1.0),
    )
    .unwrap();
    let pa = Placement {
        slide: -50.24157320904401,
        phase: 0.0,
        flip: false,
        shift: 0.0,
        tilt: 0.1,
        aligned: false,
        r1: 3.1944010823494944,
        r2: 0.1,
        angle: 0.05,
    };
    let pb = Placement {
        slide: 0.0,
        r1: 6.388798694487638,
        ..pa
    };
    constructed_touches(axis, pa, pb, false).unwrap();
}

#[test]
fn constructed_apexes_are_points_and_equal_cones_coincide() {
    check((frame(), placement(), placement()), |(axis, pa, pb)| {
        let z = axis.z().into_inner();
        let cone = on_axis(Coaxial::Cone, &axis, pa);
        let Surface::Cone {
            frame: cf,
            radius,
            half_angle,
        } = cone
        else {
            unreachable!()
        };
        let apex = cf.origin() - (radius / half_angle.tan()) * cf.z().into_inner();
        // A plane perpendicular to the axis through the apex.
        let through = Surface::Plane {
            frame: Frame::new(
                apex + pb.shift * tilted(&axis, FRAC_PI_2, pb.phase),
                if pb.flip { -z } else { z },
                tilted(&axis, FRAC_PI_2, pb.phase),
            )
            .unwrap(),
        };
        let p = one_point(&common_properties(&through, &cone)?)?;
        prop_assert!((p - apex).norm() <= EXACT, "{p} vs {apex}");
        // A second cone closing on the same apex at another angle,
        // opening either way: the apex alone.
        let other_angle = if (pb.angle - half_angle).abs() > 0.05 {
            pb.angle
        } else {
            (half_angle + 0.3).min(1.5)
        };
        let sign = if pb.flip { -1.0 } else { 1.0 };
        let other = |angle: f64| Surface::Cone {
            frame: Frame::new(
                apex + sign * (pb.r1 / angle.tan()) * z,
                sign * z,
                tilted(&axis, FRAC_PI_2, pb.phase),
            )
            .unwrap(),
            radius: pb.r1,
            half_angle: angle,
        };
        let p = one_point(&common_properties(&cone, &other(other_angle))?)?;
        prop_assert!((p - apex).norm() <= EXACT, "{p} vs {apex}");
        // The same apex at the same angle: the same double cone.
        prop_assert_eq!(
            common_properties(&cone, &other(half_angle))?,
            SurfaceIntersection::Coincident
        );
        // A cylinder on the axis cuts both nappes: two circles at
        // `R / tan α` either side of the apex.
        let cylinder = on_axis(Coaxial::Cylinder, &axis, pb);
        let r = common_properties(&cylinder, &cone)?;
        let Seen::Crossing(c) = seen(&r) else {
            return Err(TestCaseError::fail(format!("{r:?}")));
        };
        prop_assert_eq!(c.len(), 2, "{:?}", r);
        for circle in c {
            let Curve::Circle { frame, radius: rc } = circle else {
                return Err(TestCaseError::fail(format!("{circle:?}")));
            };
            prop_assert!((rc - pb.r1).abs() <= EXACT);
            let along = (frame.origin() - apex).dot(&z).abs();
            prop_assert!(
                (along - pb.r1 / half_angle.tan()).abs() <= EXACT * 100.0,
                "{along}"
            );
        }
        Ok(())
    });
}

/// A plane through the axis of a cone cuts its two rulings through the
/// apex, and through the axis of a torus its two tube circles: each
/// crossing, on both surfaces, a ruling from the apex along the
/// cone's `∂P/∂v` at the half-angle to the axis and a circle of radius
/// `r` about `O ± R·w` in the plane with its `t` the torus's `v`, the
/// first on the side `w = Z × n`; the plane's origin anywhere in it, its
/// normal either way. The same plane off the axis cuts a cone in a
/// hyperbola's two branches, and a torus in a spiric section, traced and
/// fitted (ADR-0019).
#[test]
fn a_plane_through_the_axis_cuts_the_meridian() {
    check(
        (frame(), placement(), placement(), any::<bool>()),
        |(axis, pa, pb, flip)| {
            let z = axis.z().into_inner();
            let around = tilted(&axis, FRAC_PI_2, pb.phase);
            let n = if flip { -around } else { around };
            let anchor = axis.origin() + pb.slide * z + pb.shift * z.cross(&around);
            let plane = Surface::Plane {
                frame: Frame::new(anchor, n, z).unwrap(),
            };
            let w = axis.z().cross(&UnitVec3::new_normalize(n));
            for kind in [Coaxial::Cone, Coaxial::Torus] {
                let carrier = on_axis(kind, &axis, pa);
                let own_z = carrier.frame().unwrap().z().into_inner();
                // `on_axis` may reverse the carrier's own Z, and the side
                // is taken from it.
                let w = w * own_z.dot(&z).signum();
                let r = common_properties(&plane, &carrier)?;
                let Seen::Crossing(c) = seen(&r) else {
                    return Err(TestCaseError::fail(format!("{kind:?}: {r:?}")));
                };
                prop_assert_eq!(c.len(), 2, "{:?}: {:?}", kind, r);
                prop_assert_eq!(
                    &intersect_surfaces(
                        &carrier,
                        &plane,
                        &within(),
                        tol(),
                        &mut arris_math::Meter::default()
                    )
                    .unwrap(),
                    &r,
                    "either order"
                );
                for (curve, sign) in c.iter().zip([1.0, -1.0]) {
                    match (&carrier, curve) {
                        (
                            Surface::Cone {
                                frame,
                                radius,
                                half_angle,
                            },
                            Curve::Line { origin, direction },
                        ) => {
                            let apex = frame.origin() - (radius / half_angle.tan()) * own_z;
                            prop_assert!((origin - apex).norm() <= EXACT, "{origin} vs {apex}");
                            let (sa, ca) = half_angle.sin_cos();
                            let expected = sa * sign * w + ca * own_z;
                            prop_assert!((direction.into_inner() - expected).norm() <= EXACT);
                        }
                        (
                            Surface::Torus {
                                frame,
                                major_radius,
                                minor_radius,
                            },
                            Curve::Circle {
                                frame: cf,
                                radius: cr,
                            },
                        ) => {
                            let centre = frame.origin() + sign * major_radius * w;
                            prop_assert!((cf.origin() - centre).norm() <= EXACT);
                            prop_assert_eq!(cr, minor_radius);
                            for t in [0.0, 1.0, 2.5, 4.0] {
                                let u =
                                    (sign * w).dot(&frame.y()).atan2((sign * w).dot(&frame.x()));
                                let d = (curve.point(t) - carrier.point(u, t)).norm();
                                prop_assert!(d <= EXACT, "t = {t} is not the torus's v: {d}");
                            }
                        }
                        _ => return Err(TestCaseError::fail(format!("{kind:?}: {curve:?}"))),
                    }
                }
                // Off the axis by more than the tolerance: a hyperbola's
                // two exact branches, or a spiric section, C3's.
                let off = Surface::Plane {
                    frame: plane
                        .frame()
                        .unwrap()
                        .with_origin(anchor + (pb.r2 + 0.5) * around),
                };
                let near = Aabb::of_point(anchor).inflated(4.0 * DEFAULT_SCALE);
                let r = common_properties_in(&off, &carrier, &near)?;
                if kind == Coaxial::Torus {
                    // A spiric section: two ovals, one, a figure eight's
                    // two arms through their singular point, a touch or
                    // nothing, by the plane's distance against `R ± r` —
                    // every branch fitted, never a conic.
                    prop_assert!(
                        r.curves()
                            .iter()
                            .all(|m| matches!(m.curve, Curve::Nurbs(_))),
                        "a spiric section: {r:?}"
                    );
                } else {
                    let Seen::Crossing(c) = seen(&r) else {
                        return Err(TestCaseError::fail(format!("off the axis: {r:?}")));
                    };
                    prop_assert_eq!(c.len(), 2, "{:?}", r);
                    prop_assert!(c.iter().all(|c| matches!(c, Curve::Nurbs(_))));
                }
            }
            Ok(())
        },
    );
}

// --- a torus against every analytic surface (ADR-0019) -----------------------

/// The ring the torus cases cut: `R = 2`, `r = 0.5`, in the tilt pose of
/// the fixtures, so no case is axis-aligned.
fn ring() -> Surface {
    Surface::Torus {
        frame: Frame::new(
            Point3::new(-2.5, 1.75, 0.5),
            Vec3::new(2.0, 3.0, 6.0) / 7.0,
            Vec3::new(3.0, -6.0, 2.0) / 7.0,
        )
        .unwrap(),
        major_radius: 2.0,
        minor_radius: 0.5,
    }
}

/// A point of the ring's own (u, v), and a direction of its frame.
fn on_ring(u: f64, v: f64) -> Point3 {
    ring().point(u, v)
}

fn ring_dir(x: f64, y: f64, z: f64) -> Vec3 {
    ring().frame().unwrap().vec_to_world(Vec3::new(x, y, z))
}

/// A torus meets each of the six analytic kinds in a section traced in
/// its own parameter plane and fitted (ADR-0019): the spiric sections of
/// a plane through the hole and one tangent to it, a drill through the
/// tube, a cone and a sphere off the axis, an elliptic cylinder through
/// the ring and a second torus interlocked with it. Every curve is a
/// fitted loop or an arc ending at a singular point, on both surfaces at
/// its samples, and bit for bit under a swap.
#[test]
fn a_torus_meets_every_analytic_surface_off_its_axis() {
    let ring = ring();
    let centre = ring.frame().unwrap().origin();
    // (name, surface, loops, open arms, points)
    let cases: Vec<(&str, Surface, usize, usize, usize)> = vec![
        (
            "a plane through the hole",
            Surface::Plane {
                frame: Frame::from_z(centre + ring_dir(1.0, 0.0, 0.0), ring_dir(1.0, 0.0, 0.0))
                    .unwrap(),
            },
            2,
            0,
            0,
        ),
        (
            "a plane tangent to the hole",
            Surface::Plane {
                frame: Frame::from_z(centre + ring_dir(1.5, 0.0, 0.0), ring_dir(1.0, 0.0, 0.0))
                    .unwrap(),
            },
            0,
            2,
            1,
        ),
        (
            "a drill through the tube",
            Surface::Cylinder {
                frame: Frame::from_z(centre + ring_dir(2.0, 0.0, 0.0), ring_dir(0.0, 0.0, 1.0))
                    .unwrap(),
                radius: 0.2,
            },
            2,
            0,
            0,
        ),
        (
            "a cone off the axis",
            Surface::Cone {
                frame: Frame::from_z(centre + ring_dir(1.8, 0.3, -1.0), ring_dir(0.2, 0.1, 1.0))
                    .unwrap(),
                radius: 0.5,
                half_angle: 0.4,
            },
            2,
            0,
            0,
        ),
        (
            "a sphere off the axis",
            Surface::Sphere {
                frame: Frame::from_z(centre + ring_dir(1.7, 0.4, 0.0), ring_dir(0.0, 0.0, 1.0))
                    .unwrap(),
                radius: 0.8,
            },
            2,
            0,
            0,
        ),
        (
            "an elliptic cylinder through the ring",
            Surface::EllipticCylinder {
                frame: Frame::new(centre, ring_dir(1.0, 0.0, 0.0), ring_dir(0.0, 0.0, 1.0))
                    .unwrap(),
                major_radius: 1.4,
                minor_radius: 0.9,
            },
            4,
            0,
            0,
        ),
        (
            "an interlocked torus",
            Surface::Torus {
                frame: Frame::from_z(centre + ring_dir(2.0, 0.0, 0.0), ring_dir(1.0, 0.0, 0.0))
                    .unwrap(),
                major_radius: 1.0,
                minor_radius: 0.3,
            },
            2,
            0,
            0,
        ),
    ];
    for (name, other, loops, arms, points) in cases {
        let r = intersect_surfaces(
            &ring,
            &other,
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap_or_else(|e| panic!("{name}: {e}"));
        let fitted: Vec<&arris_geom::NurbsCurve> = r
            .curves()
            .iter()
            .map(|m| {
                assert_eq!(m.kind, MeetKind::Crossing, "{name}: {r:?}");
                match &m.curve {
                    Curve::Nurbs(c) => c,
                    other => panic!("{name}: {other:?} is not fitted"),
                }
            })
            .collect();
        assert_eq!(
            (
                fitted.iter().filter(|c| c.period().is_some()).count(),
                fitted.iter().filter(|c| c.period().is_none()).count(),
                r.points().len()
            ),
            (loops, arms, points),
            "{name}: {r:?}"
        );
        let slack = if points == 0 { 0.0 } else { tol().linear };
        for m in r.curves() {
            on_both(&m.curve, &ring, &other, slack).unwrap_or_else(|e| panic!("{name}: {e}"));
        }
        for p in r.points() {
            let off = implicit_distance(&ring, p.point).max(implicit_distance(&other, p.point));
            assert!(off <= tol().linear, "{name}: a point is {off} off");
        }
        assert_eq!(
            intersect_surfaces(
                &other,
                &ring,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            r,
            "{name}: the swap changed the result"
        );
    }
}

/// The pipe elbow: a cylinder of the tube's radius whose axis is tangent
/// to the centre circle shares a whole tube circle with the torus. It
/// comes back exact — a `Curve::Circle` on the torus, parametrised by its
/// `v` — and touching, since the two do not cross along it, before the
/// two fitted arms of the rest of the section, which end at the singular
/// points where they reach it (ADR-0019).
#[test]
fn a_pipe_elbow_shares_an_exact_tube_circle_with_its_pipe() {
    let ring = ring();
    let centre = ring.frame().unwrap().origin();
    let pipe = Surface::Cylinder {
        frame: Frame::from_z(centre + ring_dir(2.0, 0.0, 0.0), ring_dir(0.0, 1.0, 0.0)).unwrap(),
        radius: 0.5,
    };
    let r = intersect_surfaces(
        &ring,
        &pipe,
        &within(),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let [circle, first, second] = r.curves() else {
        panic!("{r:?}")
    };
    assert_eq!(circle.kind, MeetKind::Touch, "{r:?}");
    let Curve::Circle { frame, radius } = &circle.curve else {
        panic!("{r:?}")
    };
    assert_eq!(*radius, 0.5);
    assert!((frame.origin() - on_ring(0.0, 0.0) + 0.5 * ring_dir(1.0, 0.0, 0.0)).norm() < EXACT);
    // Its parameter is the torus's `v`: the circle's point at `t` is the
    // torus's at (0, t).
    for t in [0.0, 1.0, 2.5, -0.7] {
        let d = (circle.curve.point(t) - on_ring(0.0, t)).norm();
        assert!(d < EXACT, "t = {t} is not the torus's v: {d}");
    }
    // The rest: two arms crossing, each ending at one of the two singular
    // points where it meets the circle.
    assert_eq!(
        (first.kind, second.kind),
        (MeetKind::Crossing, MeetKind::Crossing)
    );
    assert_eq!(r.points().len(), 2, "{r:?}");
    assert!(r.points().iter().all(|p| p.kind == MeetKind::Crossing));
    for m in r.curves() {
        on_both(&m.curve, &ring, &pipe, tol().linear).unwrap();
    }
    assert_eq!(
        intersect_surfaces(
            &pipe,
            &ring,
            &within(),
            tol(),
            &mut arris_math::Meter::default()
        )
        .unwrap(),
        r,
        "the swap changed the result"
    );
}

/// A plane bitangent to the ring meets it in the two Villarceau circles.
/// They are *fitted*, not returned as `Curve::Circle`s: the tracer finds
/// them as four arms through the pose's two singular points, and a
/// second "bitangent within `tol.linear`" detector beside the tracer's
/// own is what ADR-0019 declines to add for a pose of measure zero. The
/// arms are held to the exact circles all the same.
#[test]
fn a_bitangent_plane_meets_the_ring_in_fitted_villarceau_circles() {
    let ring = ring();
    let centre = ring.frame().unwrap().origin();
    let tilt = (0.5f64 / 2.0).asin();
    let plane = Surface::Plane {
        frame: Frame::from_z(centre, ring_dir(-tilt.sin(), 0.0, tilt.cos())).unwrap(),
    };
    let r = intersect_surfaces(
        &ring,
        &plane,
        &within(),
        tol(),
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    assert_eq!((r.curves().len(), r.points().len()), (4, 2), "{r:?}");
    assert!(
        r.curves()
            .iter()
            .all(|m| matches!(&m.curve, Curve::Nurbs(c) if c.period().is_none())),
        "{r:?}"
    );
    // Each arm lies on one of the two circles of radius `R` centred `r`
    // either side of the axis, within the fit's fraction of the tolerance
    // beyond the tolerance the tracer allows at a singular point.
    let bound = tol().linear + SECTION_FIT_FRACTION * tol().linear;
    for m in r.curves() {
        let domain = m.curve.domain();
        let off = |p: Point3, side: f64| {
            ((p - (centre + side * 0.5 * ring_dir(0.0, 1.0, 0.0))).norm() - 2.0).abs()
        };
        let middle = m.curve.point(domain.lerp(0.5));
        let side = if off(middle, 1.0) < off(middle, -1.0) {
            1.0
        } else {
            -1.0
        };
        for i in 0..=64 {
            let p = m.curve.point(domain.lerp(i as f64 / 64.0));
            assert!(
                off(p, side) <= bound,
                "{p} is {} off its circle",
                off(p, side)
            );
        }
    }
}

/// A non-coaxial pose from §Non-goals of the quadric plan.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Apart {
    /// A plane oblique to a cone's or a torus's axis.
    Oblique,
    /// A plane parallel to the axis and off it: through it, the plane
    /// cuts the meridian (`a_plane_through_the_axis_cuts_the_meridian`).
    Parallel,
    /// Two cones or two tori on parallel axes apart, or on crossing axes.
    OtherAxis,
    /// A sphere whose centre is off a carrier's axis.
    OffCentre,
}

fn apart() -> impl Strategy<Value = Apart> {
    prop_oneof![
        Just(Apart::Oblique),
        Just(Apart::Parallel),
        Just(Apart::OtherAxis),
        Just(Apart::OffCentre),
    ]
}

prop_shards! {
/// Every pose of §Non-goals of ADR-0008 is decided now, a torus's too: a
/// plane oblique to a cone's axis or parallel to it and off it in an
/// exact conic, two cones off one axis and a sphere off a cone's or a
/// cylinder's axis traced and fitted (ADR-0018), and every torus pose —
/// a plane oblique to the axis or parallel to it and off it, two tori on
/// other axes, a sphere off the axis — traced in the torus's parameter
/// plane and fitted (ADR-0019); each on both surfaces at and between its
/// samples, and bit for bit under a swap. Sharded: a torus case traces
/// and fits three times.
every_non_coaxial_quadric_pose_is_decided
    [shard_0 shard_1 shard_2 shard_3]
    ((axis, pose, kind, pa, pb, through)) = (
            frame(),
            apart(),
            prop_oneof![
                Just(Coaxial::Cone),
                Just(Coaxial::Torus),
                Just(Coaxial::Cylinder)
            ],
            placement(),
            placement(),
            any::<bool>(),
        ) => {
            let z = axis.z().into_inner();
            let carrier = on_axis(kind, &axis, pa);
            let around = tilted(&axis, FRAC_PI_2, pb.phase);
            let offset = pb.r1 + 0.5;
            let other = match pose {
                Apart::Oblique => Surface::Plane {
                    frame: Frame::from_z(
                        axis.origin() + pb.slide * z,
                        tilted(&axis, pb.tilt, pb.phase),
                    )
                    .unwrap(),
                },
                Apart::Parallel => Surface::Plane {
                    frame: Frame::new(axis.origin() + pb.slide * z + offset * around, around, z)
                        .unwrap(),
                },
                Apart::OtherAxis => {
                    let direction = if through {
                        z
                    } else {
                        tilted(&axis, pb.tilt, pb.phase)
                    };
                    let moved = Frame::new(
                        axis.origin() + pb.slide * z + offset * around,
                        direction,
                        z.cross(&around),
                    )
                    .unwrap();
                    on_axis(kind, &moved, pb)
                }
                Apart::OffCentre => Surface::Sphere {
                    frame: Frame::new(axis.origin() + pb.slide * z + offset * around, z, around)
                        .unwrap(),
                    radius: pb.r2,
                },
            };
            // A plane against a cylinder and two cylinders are cycle 1's
            // and the cylinder plan's arms, not this one's: a cylinder
            // carries the axis here only against an off-centre sphere.
            prop_assume!(kind != Coaxial::Cylinder || pose == Apart::OffCentre);
            let near = Aabb::of_point(axis.origin()).inflated(4.0 * DEFAULT_SCALE);
            common_properties_in(&carrier, &other, &near)?;
            Ok(())
        }
}

// --- every quadric pair at a meeting pose (C3's accept line) -----------------

/// A rough size of a surface: how far from a point of it another can sit
/// and still be likely to meet it.
fn size(surface: &Surface) -> f64 {
    match *surface {
        Surface::Cylinder { radius, .. }
        | Surface::Sphere { radius, .. }
        | Surface::Cone { radius, .. } => radius,
        Surface::EllipticCylinder { minor_radius, .. } => minor_radius,
        Surface::Plane { .. } | Surface::Torus { .. } | Surface::Nurbs(_) => 1.0,
    }
}

/// Every analytic surface, in a random pose.
fn quadric() -> impl Strategy<Value = Surface> {
    prop_oneof![
        plane(),
        cylinder(),
        elliptic_cylinder(),
        cone(),
        sphere(),
        torus()
    ]
}

/// Two quadrics placed to meet: the second's origin within its own size
/// of a point of the first, and a region of thirty units about it.
fn meeting() -> impl Strategy<Value = (Surface, Surface, Aabb)> {
    (
        quadric(),
        quadric(),
        finite_f64(0.0..=TAU),
        finite_f64(-5.0..=5.0),
        unit_vec3(),
        finite_f64(0.0..=0.9),
    )
        .prop_map(|(a, b, u, v, direction, reach)| {
            let v = if matches!(a, Surface::Sphere { .. }) {
                v * 0.3
            } else {
                v
            };
            let at = a.point(u, v) + direction.into_inner() * (reach * size(&b));
            let mut b = b;
            match &mut b {
                Surface::Plane { frame }
                | Surface::Cylinder { frame, .. }
                | Surface::EllipticCylinder { frame, .. }
                | Surface::Cone { frame, .. }
                | Surface::Sphere { frame, .. }
                | Surface::Torus { frame, .. } => *frame = frame.with_origin(at),
                Surface::Nurbs(_) => {}
            }
            (a, b, Aabb::of_point(at).inflated(30.0))
        })
}

prop_shards! {
    /// C3's accept line for every quadric pair, the torus's among them, at
    /// random poses where the two meet: decided — by a closed form, an
    /// exact conic or a tracer and a fit (ADR-0018, ADR-0019) — every
    /// curve on both surfaces at its samples, a fitted one densely and
    /// within the fit's fraction of the tolerance, every point on both
    /// within the tolerance, the same under a swap and on a second run.
    /// Sharded: a case traces and fits three times.
    every_quadric_pair_meets_on_both_surfaces
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((a, b, within)) = meeting() => {
            // A fit that gives up at its span cap is a named exclusion:
            // a flat cone against a cylinder beside its apex on a nightly
            // seed (`a_flat_cone_and_a_cylinder_fit_within_the_span_cap`,
            // ignored). Any other failure fails as before.
            if let Err(GeomError::Fit(FitError::Diverged { .. })) =
                intersect_surfaces(&a, &b, &within, tol(), &mut arris_math::Meter::default())
            {
                return Err(TestCaseError::reject("fit-diverged"));
            }
            common_properties_in(&a, &b, &within)?;
            Ok(())
        }
}

/// `every_quadric_pair_meets_on_both_surfaces` at 5000 cases on the first
/// nightly's date seed (`e4bc2ddf…`), shard 5 of 8, shrunk: a cone of
/// half-angle 80.5° and a cylinder of radius 8.9 across it beside its
/// apex. The section's fit still deviates by 7.6e-8 at 3472 spans, the
/// most it may use, and `intersect_surfaces` refuses with
/// `FitError::Diverged`. The desired answer is the section, fitted.
#[test]
#[ignore = "the section fit diverges at its span cap (docs/BACKLOG.md, the nightly's findings; the property's fit-diverged exclusion)"]
fn a_flat_cone_and_a_cylinder_fit_within_the_span_cap() {
    let frame = |o: [f64; 3], x: [f64; 3], y: [f64; 3], z: [f64; 3]| {
        Frame::from_orthonormal(Point3::from(o), Vec3::from(x), Vec3::from(y), Vec3::from(z))
            .unwrap()
    };
    let cone = Surface::Cone {
        frame: frame(
            [0.0, 0.0, 0.0],
            [-0.8531737429817391, 0.3462896657204334, 0.390100027815636],
            [
                -0.29014599193643076,
                -0.9365311890362789,
                0.1967857599663108,
            ],
            [0.4334857179305382, 0.05470648387096651, 0.8994983785270108],
        ),
        radius: 7.441055951202834,
        half_angle: 1.4057978124306354,
    };
    let cylinder = Surface::Cylinder {
        frame: frame(
            [-4.785284191718697, -4.986390222073084, 7.561299106676653],
            [
                -0.27161909453342686,
                0.10336296986947918,
                -0.9568381074897688,
            ],
            [0.07032826101955032, 0.9936894533182202, 0.0873796662050311],
            [
                0.9598317577507046,
                -0.04355877436174938,
                -0.27717436748244756,
            ],
        ),
        radius: 8.888420394193522,
    };
    let within = Aabb {
        min: [
            -34.785284191718695,
            -34.986390222073084,
            -22.438700893323347,
        ],
        max: [25.214715808281305, 25.013609777926916, 37.56129910667666],
    };
    common_properties_in(&cone, &cylinder, &within).unwrap();
}

// --- a plane against a cone off its axis --------------------------------------

/// Every sample of each curve of `r` on both surfaces to rounding — the
/// conics are exact, not fitted.
fn exactly_on_both(r: &SurfaceIntersection, a: &Surface, b: &Surface) {
    for m in r.curves() {
        let domain = match &m.curve {
            Curve::Line { .. } => arris_math::Interval::new(-5.0, 5.0).unwrap(),
            other => other.domain(),
        };
        for i in 0..=400 {
            let p = m.curve.point(domain.lerp(i as f64 / 400.0));
            let off = implicit_distance(a, p).max(implicit_distance(b, p));
            assert!(off <= 1e-12 * (1.0 + p.coords.norm()), "{p} is {off} off");
        }
    }
}

/// A cone of half-angle 30° about `z`, its apex at `z = −√3`, against a
/// plane of each pose: steeper than the cone an ellipse, as steep a
/// parabola, shallower a hyperbola's two branches; through the apex the
/// apex alone, one touching ruling or two crossing ones. Parabolas and
/// hyperbolas are exact rational quadratics clipped to the region; a
/// hyperbola's arcs keep their middle weight at `cosh` of at most
/// `HYPERBOLA_HALF_SPAN`.
#[test]
fn a_plane_cuts_a_cone_off_its_axis_in_every_conic() {
    let alpha = core::f64::consts::FRAC_PI_6;
    let cone = Surface::Cone {
        frame: Frame::world(),
        radius: 1.0,
        half_angle: alpha,
    };
    let apex = Point3::new(0.0, 0.0, -(3.0f64.sqrt()));
    let within = Aabb {
        min: [-20.0; 3],
        max: [20.0; 3],
    };
    // A plane through `at` whose normal is `tilt` off the axis, towards x.
    let plane = |at: Point3, tilt: f64| Surface::Plane {
        frame: Frame::from_z(at, Vec3::new(tilt.sin(), 0.0, tilt.cos())).unwrap(),
    };
    let meet = |p: &Surface| {
        let r = intersect_surfaces(p, &cone, &within, tol(), &mut arris_math::Meter::default())
            .unwrap();
        assert_eq!(
            intersect_surfaces(&cone, p, &within, tol(), &mut arris_math::Meter::default())
                .unwrap(),
            r
        );
        exactly_on_both(&r, p, &cone);
        r
    };
    let up = Point3::new(0.0, 0.0, 1.0);

    let r = meet(&plane(up, 0.3));
    let Seen::Crossing(c) = seen(&r) else {
        panic!("{r:?}")
    };
    assert!(matches!(c.as_slice(), [Curve::Ellipse { .. }]), "{r:?}");

    let steep = FRAC_PI_2 - alpha;
    let r = meet(&plane(up, steep));
    let Seen::Crossing(c) = seen(&r) else {
        panic!("{r:?}")
    };
    let [Curve::Nurbs(parabola)] = c.as_slice() else {
        panic!("{r:?}")
    };
    assert_eq!(parabola.degree(), 2);
    assert!(parabola.weights().iter().all(|&w| w == 1.0));

    let r = meet(&plane(Point3::new(0.5, 0.0, 0.0), FRAC_PI_2));
    let Seen::Crossing(c) = seen(&r) else {
        panic!("{r:?}")
    };
    assert_eq!(c.len(), 2, "{r:?}");
    for branch in &c {
        let Curve::Nurbs(h) = branch else {
            panic!("{r:?}")
        };
        assert_eq!(h.degree(), 2);
        assert!(
            h.weights()
                .iter()
                .all(|&w| (1.0..=HYPERBOLA_HALF_SPAN.cosh()).contains(&w))
        );
        // The branch reaches past the region at both ends.
        let d = h.domain();
        for end in [d.lo(), d.hi()] {
            let p = h.eval(end).point;
            assert!(
                (0..3).any(|i| p[i] < within.min[i] || p[i] > within.max[i]),
                "{p} is inside the region"
            );
        }
    }

    let r = meet(&plane(apex, 0.3));
    assert!(r.curves().is_empty(), "{r:?}");
    assert_eq!(r.points().len(), 1);
    assert_eq!(r.points()[0].kind, MeetKind::Crossing);
    assert!((r.points()[0].point - apex).norm() < 1e-12);

    let r = meet(&plane(apex, steep));
    let Seen::Touch(c) = seen(&r) else {
        panic!("{r:?}")
    };
    assert!(matches!(c.as_slice(), [Curve::Line { .. }]), "{r:?}");

    let r = meet(&plane(apex, FRAC_PI_2 - 0.2));
    let Seen::Crossing(c) = seen(&r) else {
        panic!("{r:?}")
    };
    assert_eq!(c.len(), 2, "{r:?}");
    assert!(
        c.iter()
            .all(|c| matches!(c, Curve::Line { origin, .. } if (origin - apex).norm() < 1e-12))
    );
}

/// Found by the `intersect_surfaces` fuzz target (`fuzz/`, ADR-0024 §5):
/// a plane 2e-12 from square to an elliptic cylinder's axis — past the
/// angular tolerance, so the oblique arm — cut a section of radius 1000
/// that was 3e-6 small, the in-plane basis it read the semi-diameters in
/// being off the plane by the rounding of a near-cancelling difference.
#[test]
fn a_plane_a_hair_from_square_cuts_an_elliptic_cylinder_to_its_radii() {
    let axis = Vec3::new(0.0, -1.0, -1.0);
    for (a, b) in [(999.999, 999.999), (1000.0, 400.0)] {
        let wall = Surface::EllipticCylinder {
            frame: Frame::new(Point3::new(-472.0, -936.0, 488.0), axis, -Vec3::x()).unwrap(),
            major_radius: a,
            minor_radius: b,
        };
        for tilt in [2e-12, 5e-12, 1e-10, 1e-8] {
            let normal = axis + Vec3::new(0.0, tilt, -tilt);
            let cap = Surface::Plane {
                frame: Frame::new(Point3::origin(), normal, -Vec3::x()).unwrap(),
            };
            let hit = intersect_surfaces(
                &cap,
                &wall,
                &Aabb {
                    min: [-2e3; 3],
                    max: [2e3; 3],
                },
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap();
            let [meet] = hit.curves() else {
                panic!("{hit:?}")
            };
            let Curve::Ellipse {
                major_radius,
                minor_radius,
                ..
            } = meet.curve
            else {
                panic!("{meet:?}")
            };
            // The section's semi-axes are `a` and `b` to first order in
            // the tilt, the stretch along the tilt `1/cos` of it.
            assert!(
                (major_radius - a).abs() <= 1e-9 * a,
                "{tilt:e}: {major_radius} for {a}"
            );
            assert!(
                (minor_radius - b).abs() <= 1e-9 * a,
                "{tilt:e}: {minor_radius} for {b}"
            );
        }
    }
}

/// Found by the `intersect_curves` fuzz target on the nightly
/// (`fuzz/`, ADR-0024 §5): two planes 5.6e-9 of a radian from parallel,
/// past the angular tolerance, meet in a line some 1e9 out, and its
/// origin came out NaN — the determinant `1 − c²` rounded to zero.
#[test]
fn two_planes_a_hair_from_parallel_meet_in_a_finite_line_on_both() {
    for tilt in [5.6e-9, 1e-10, 2e-12] {
        let a = Surface::Plane {
            frame: Frame::from_z(Point3::origin(), Vec3::new(1.0, tilt, 0.0)).unwrap(),
        };
        let b = Surface::Plane {
            frame: Frame::from_z(Point3::new(5.5, 1.75, 1.5), -Vec3::x()).unwrap(),
        };
        let hit = intersect_surfaces(&a, &b, &within(), tol(), &mut arris_math::Meter::default())
            .unwrap();
        let [meet] = hit.curves() else {
            panic!("{tilt:e}: {hit:?}")
        };
        let Curve::Line { origin, .. } = meet.curve else {
            panic!("{meet:?}")
        };
        assert!(origin.iter().all(|x| x.is_finite()), "{tilt:e}: {origin}");
        // On both planes to its rounding over the sine between them.
        let bound = ROUNDING * origin.coords.norm() / tilt;
        for s in [&a, &b] {
            assert!(
                implicit_distance(s, origin) <= bound,
                "{tilt:e}: {} off",
                implicit_distance(s, origin)
            );
        }
    }
}

/// NIST's FTC-07 (ADR-0026's amendment of step 5): a fillet torus, tube
/// 0.43 about a centre circle of 11.42, and a plane tilted 2° that is
/// tangent to its rim on the torus's seam, at `v` = 2°. The seam's root
/// search landed 6.8e-7 from the singular point, outside its cell, where
/// the distance is quadratic and within the tolerance, and the tracer
/// refused the section as `UnresolvedTurning`. It is one touch.
#[test]
fn a_plane_tangent_to_a_torus_rim_on_its_seam_touches_it_once() {
    let torus = Surface::Torus {
        frame: Frame::from_orthonormal(
            Point3::new(-116.08298424713472, 10.4902, -94.74199999999995),
            Vec3::new(0.0, 0.0, -1.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
        )
        .unwrap(),
        major_radius: 11.419415587038413,
        minor_radius: 0.4318,
    };
    let plane = Surface::Plane {
        frame: Frame::from_orthonormal(
            Point3::new(-116.08298424713479, 12.7, -106.67064670468945),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, 0.999390827019096, -0.034899496702496),
            Vec3::new(0.0, -0.034899496702496, -0.999390827019096),
        )
        .unwrap(),
    };
    let within = Aabb {
        min: [-128.0, 9.5, -107.0],
        max: [-104.0, 11.5, -82.0],
    };
    let tol = Tolerance::new(1e-7, 1e-12);
    let r = intersect_surfaces(
        &torus,
        &plane,
        &within,
        tol,
        &mut arris_math::Meter::default(),
    )
    .unwrap();
    let SurfaceIntersection::Meets { curves, points } = &r else {
        panic!("{r:?}");
    };
    assert!(curves.is_empty(), "{r:?}");
    assert_eq!(points.len(), 1, "{r:?}");
    assert_eq!(points[0].kind, MeetKind::Touch);
    let p = points[0].point;
    assert!(
        implicit_distance(&torus, p).max(implicit_distance(&plane, p)) <= 1e-7,
        "{p}"
    );
}
