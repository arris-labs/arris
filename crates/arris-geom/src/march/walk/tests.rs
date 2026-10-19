use std::time::Instant;

use arris_math::{Frame, Interval, Precision};
use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

use super::*;
use crate::Surface;

const TURN: [f64; 2] = [0.0, 2.0 * PI];

fn twin(surface: &Surface, u: [f64; 2], v: [f64; 2]) -> NurbsSurface {
    let range = |[lo, hi]: [f64; 2]| Interval::new(lo, hi).unwrap();
    surface.to_nurbs([range(u), range(v)]).unwrap()
}

/// A pair of analytic surfaces, their twins, and what the closed form
/// says their section is.
#[derive(Clone)]
struct Case {
    label: &'static str,
    surfaces: [(Surface, NurbsSurface); 2],
    /// Loops.
    closed: usize,
    /// The open branches' ends, `None` for an end on an edge and the
    /// singular point's index for one there, each pair sorted.
    open: Vec<[Option<usize>; 2]>,
    /// The singular points and whether each is isolated.
    points: Vec<(Point3, bool)>,
    /// Where the open branches end on an edge.
    clipped: Vec<Point3>,
}

fn cylinder(origin: Point3, axis: Vec3, seam: Vec3, radius: f64) -> (Surface, NurbsSurface) {
    let surface = Surface::Cylinder {
        frame: Frame::new(origin, axis, seam).unwrap(),
        radius,
    };
    let nurbs = twin(&surface, TURN, [-2.0, 2.0]);
    (surface, nurbs)
}

fn cases() -> Vec<Case> {
    let o = Point3::origin();
    // The plane z = 0 over [−2, 2]², and the plane x = z through the
    // origin over [−3, 3]², its `u` along y: the line x = z = 0 from edge
    // to edge of the first, inside the second.
    let floor = Surface::Plane {
        frame: Frame::world(),
    };
    let leaning = Surface::Plane {
        frame: Frame::new(o, Vec3::new(1.0, 0.0, -1.0), Vec3::y()).unwrap(),
    };
    let planes = [
        (floor.clone(), twin(&floor, [-2.0, 2.0], [-2.0, 2.0])),
        (leaning.clone(), twin(&leaning, [-3.0, 3.0], [-3.0, 3.0])),
    ];
    // A cylinder of radius 1 along z, its seam towards +y, and one of
    // radius 0.6 along x, its seam towards +z: through the axis, two
    // loops; 0.5 off it, one loop over the wider one's seam; 0.4 off,
    // tangent inside at (0, 1, 0), a figure of eight crossing there; 1.6
    // off, tangent outside, the one point. The wider's seam is turned to
    // −y where it would be at the tangency.
    let wide = cylinder(o, Vec3::z(), Vec3::y(), 1.0);
    let wide_away = cylinder(o, Vec3::z(), -Vec3::y(), 1.0);
    let narrow = |off: f64| cylinder(Point3::new(0.0, off, 0.0), Vec3::x(), Vec3::z(), 0.6);
    let touch = Point3::new(0.0, 1.0, 0.0);
    // Viviani's figure of eight: the unit sphere and a cylinder of radius
    // 0.5 along z through (0.5, 0, 0), crossing at (1, 0, 0). The
    // sphere's poles are along y and its seam towards −x, off the curve;
    // the cylinder's seam, towards −x as well, is crossed at (0, 0, ±1).
    let ball = Surface::Sphere {
        frame: Frame::new(o, Vec3::y(), -Vec3::x()).unwrap(),
        radius: 1.0,
    };
    let sphere = (ball.clone(), twin(&ball, TURN, [-FRAC_PI_2, FRAC_PI_2]));
    let tube = cylinder(Point3::new(0.5, 0.0, 0.0), Vec3::z(), -Vec3::x(), 0.5);
    let base = vec![
        Case {
            label: "plane × plane",
            surfaces: planes,
            closed: 0,
            open: vec![[None, None]],
            points: vec![],
            clipped: vec![Point3::new(0.0, -2.0, 0.0), Point3::new(0.0, 2.0, 0.0)],
        },
        Case {
            label: "crossing cylinders",
            surfaces: [wide.clone(), narrow(0.0)],
            closed: 2,
            open: vec![],
            points: vec![],
            clipped: vec![],
        },
        Case {
            label: "skew cylinders",
            surfaces: [wide, narrow(0.5)],
            closed: 1,
            open: vec![],
            points: vec![],
            clipped: vec![],
        },
        Case {
            label: "cylinders tangent inside",
            surfaces: [wide_away.clone(), narrow(0.4)],
            closed: 0,
            open: vec![[Some(0), Some(0)], [Some(0), Some(0)]],
            points: vec![(touch, false)],
            clipped: vec![],
        },
        Case {
            label: "cylinders tangent outside",
            surfaces: [wide_away, narrow(1.6)],
            closed: 0,
            open: vec![],
            points: vec![(touch, true)],
            clipped: vec![],
        },
        Case {
            label: "Viviani's sphere × cylinder",
            surfaces: [sphere, tube],
            closed: 0,
            open: vec![[Some(0), Some(0)], [Some(0), Some(0)]],
            points: vec![(Point3::new(1.0, 0.0, 0.0), false)],
            clipped: vec![],
        },
    ];
    let mut out = Vec::new();
    for case in base {
        let mut swapped = case.clone();
        swapped.surfaces.reverse();
        out.push(case);
        out.push(swapped);
    }
    out
}

/// Every twin pair's walked section has the closed form's branches,
/// ends and singular points, in either order of the surfaces, every
/// walked point on both twins and both analytic surfaces.
#[test]
fn march_every_twin_section_has_the_closed_forms_branches() {
    let tol = Precision::DEFAULT.tolerance();
    let near = ON_SHARE * tol.linear;
    for case in cases() {
        let [(sa, a), (sb, b)] = &case.surfaces;
        let start = Instant::now();
        let section = march(a, b, tol, &mut Meter::default())
            .unwrap_or_else(|e| panic!("{}: {e}", case.label));
        let seconds = start.elapsed().as_secs_f64();
        let label = case.label;
        let closed = section.branches.iter().filter(|b| b.ends.is_none()).count();
        assert_eq!(closed, case.closed, "{label}");
        let index = |e: BranchEnd| match e {
            BranchEnd::Singular(k) => Some(k),
            BranchEnd::Clipped => None,
        };
        let mut open: Vec<[Option<usize>; 2]> = section
            .branches
            .iter()
            .filter_map(|b| b.ends)
            .map(|[s, e]| {
                let mut pair = [index(s), index(e)];
                pair.sort();
                pair
            })
            .collect();
        open.sort();
        assert_eq!(open, case.open, "{label}");
        assert_eq!(section.points.len(), case.points.len(), "{label}");
        for (found, (at, isolated)) in section.points.iter().zip(&case.points) {
            assert!(
                (found.point - at).norm() <= tol.linear,
                "{label}: {found:?}"
            );
            assert_eq!(found.isolated, *isolated, "{label}");
            let x = found.at;
            let (p, q) = (a.eval(x[0], x[1]).point, b.eval(x[2], x[3]).point);
            assert!((p - q).norm() <= tol.linear, "{label}: {found:?}");
        }
        // No chord longer than the longest step and its drift.
        let longest = (1.0 + STEP_DRIFT) * Marcher::new(a, b, tol).step_max;
        let mut walked = 0;
        for branch in &section.branches {
            let ends = branch.at.first().into_iter().chain(branch.at.last());
            for (k, at) in ends.enumerate() {
                if branch.ends.map(|e| e[k]) == Some(BranchEnd::Clipped) {
                    let p = a.eval(at[0], at[1]).point;
                    assert!(
                        case.clipped.iter().any(|c| (p - c).norm() <= tol.linear),
                        "{label}: clipped at {p}"
                    );
                }
            }
            for pair in branch.at.windows(2) {
                let [p, q] = [pair[0], pair[1]].map(|x| a.eval(x[0], x[1]).point);
                assert!((p - q).norm() <= longest, "{label}: a jump from {p} to {q}");
            }
            for x in &branch.at {
                walked += 1;
                let (p, q) = (a.eval(x[0], x[1]).point, b.eval(x[2], x[3]).point);
                let singular = section
                    .points
                    .iter()
                    .any(|s| (s.point - p).norm() <= tol.linear);
                // A singular point is where the descent left it, within
                // the tolerance; every walked point is on both to the
                // corrector's share.
                let bound = if singular { tol.linear } else { near };
                assert!(
                    (p - q).norm() <= bound,
                    "{label}: {x:?} apart by {}",
                    (p - q).norm()
                );
                for s in [sa, sb] {
                    let off = s.project(p).unwrap().distance;
                    assert!(
                        off <= bound + near,
                        "{label}: {p} is {off} off {:?}",
                        s.kind()
                    );
                }
            }
        }
        eprintln!(
            "{label}: {} branches, {} points, {walked} walked points; {seconds:.3} s",
            section.branches.len(),
            section.points.len()
        );
    }
}

/// An isocurve of one surface on the other is returned exact (ADR-0068 step 6).
#[test]
fn march_an_isocurve_on_the_other_surface_is_returned_exact() {
    let tol = Precision::DEFAULT.tolerance();
    // A ruled patch: from (1, 0, 0) to (1, 0, 1) along u = 0.
    let ruled = NurbsSurface::new(
        [1, 1],
        [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
        vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 1.0),
            Point3::new(-0.5, 1.5, 0.0),
            Point3::new(-0.5, -0.5, 1.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    // A plane through the ruling x = 1, y = 0.
    let floor = Surface::Plane {
        frame: Frame::new(Point3::new(1.0, 0.0, 0.0), Vec3::x(), Vec3::y()).unwrap(),
    };
    let floor_twin = twin(&floor, [-2.0, 2.0], [-2.0, 2.0]);
    for (a, b) in [(&ruled, &floor_twin), (&floor_twin, &ruled)] {
        let section = march(a, b, tol, &mut Meter::default()).unwrap();
        assert_eq!(section.isocurves.len(), 1);
        let Curve::Nurbs(ref line) = section.isocurves[0] else {
            panic!("expected Nurbs curve");
        };
        let expected = ruled.isocurve(0, 0.0).unwrap();
        assert_eq!(line.control_points().len(), expected.control_points().len());
        for (p, q) in line.control_points().iter().zip(expected.control_points()) {
            assert!((p - q).norm() <= tol.linear);
        }
        assert!(section.branches.is_empty());
        assert!(section.points.is_empty());
    }
}

/// A pole on the other surface as a singular point (ADR-0068 step 6).
#[test]
fn march_a_pole_on_the_other_surface_is_a_singular_point() {
    let tol = Precision::DEFAULT.tolerance();
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 1.0,
    };
    let sphere = twin(&ball, [0.0, 2.0 * PI], [-FRAC_PI_2, FRAC_PI_2]);
    // A cone whose apex is at (0, 0, 1), the north pole of the sphere,
    // opening towards +z so it meets the sphere only at that pole.
    // v_apex = -R / sin(π/4) = -sqrt(2).
    let v_apex = -core::f64::consts::SQRT_2;
    let cone = Surface::Cone {
        frame: Frame::new(Point3::new(0.0, 0.0, 2.0), Vec3::z(), Vec3::x()).unwrap(),
        radius: 1.0,
        half_angle: FRAC_PI_4,
    };
    let cone_twin = twin(&cone, [0.0, 2.0 * PI], [v_apex, v_apex + 1.0]);
    let touch = Point3::new(0.0, 0.0, 1.0);
    for (a, b) in [(&sphere, &cone_twin), (&cone_twin, &sphere)] {
        let section = march(a, b, tol, &mut Meter::default()).unwrap();
        assert_eq!(section.points.len(), 1);
        assert!((section.points[0].point - touch).norm() <= tol.linear);
        assert!(section.points[0].isolated);
        assert!(section.branches.is_empty());
        assert!(section.isocurves.is_empty());
    }
}

/// A patch flush with the other surface is Unsupported (ADR-0068 step 6).
#[test]
fn march_a_patch_flush_with_the_other_surface_is_unsupported() {
    let tol = Precision::DEFAULT.tolerance();
    let square = NurbsSurface::new(
        [1, 1],
        [vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0]],
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ],
        vec![1.0; 4],
    )
    .unwrap();
    let floor = Surface::Plane {
        frame: Frame::world(),
    };
    let floor_twin = twin(&floor, [-2.0, 2.0], [-2.0, 2.0]);
    for (a, b) in [(&square, &floor_twin), (&floor_twin, &square)] {
        let err = march(a, b, tol, &mut Meter::default()).unwrap_err();
        assert!(matches!(err, GeomError::Unsupported { .. }));
    }
}

/// A sphere resting in a torus's tube touches it along a circle: not an
/// isocurve of either twin and not at a pole, so the continuum is a tube
/// circle and is refused (ADR-0061 §4).
#[test]
fn march_a_tube_circle_is_refused() {
    let tol = Precision::DEFAULT.tolerance();
    let torus = Surface::Torus {
        frame: Frame::world(),
        major_radius: 3.0,
        minor_radius: 1.0,
    };
    let ball = Surface::Sphere {
        frame: Frame::world(),
        radius: 4.0,
    };
    let torus_twin = twin(&torus, TURN, TURN);
    let ball_twin = twin(&ball, TURN, [-FRAC_PI_2, FRAC_PI_2]);
    for (a, b) in [(&torus_twin, &ball_twin), (&ball_twin, &torus_twin)] {
        let err = march(a, b, tol, &mut Meter::default()).unwrap_err();
        assert!(
            matches!(
                err,
                GeomError::DegenerateSection {
                    fault: SectionFault::TubeCircle,
                    ..
                }
            ),
            "{err}"
        );
    }
}
