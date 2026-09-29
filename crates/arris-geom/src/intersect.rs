//! Surface–surface intersection: the closed-form table, and the traced
//! and fitted sections of the quadric pairs that have none.

use core::f64::consts::FRAC_PI_2;

use arris_math::{Aabb, Frame, Meter, Point2, Point3, Tolerance, UnitVec3, Vec2, Vec3};

use crate::conic2::{Conic2, ConicMeet, conic_pair};
use crate::pcurve::principal_axes;
use crate::{Curve, GeomError, GeomKind, Surface};

/// What two surfaces have in common.
///
/// Every curve of a closed form is an exact analytic curve lying on both
/// surfaces to rounding, with a frame that is Arris's own deterministic
/// choice (`docs/DATA-MODEL.md` §Curves): a circle on a cylinder takes the
/// cylinder's `X`, a plane's ellipse on a cylinder has its `X` along its
/// major axis in the direction of increasing `v`, two crossing cylinders'
/// ellipses have each `Z` and `X` signed so the largest-magnitude
/// component is positive, a ruling on a cylinder runs along its `Z` from
/// the point nearest the cylinder's origin (the first cylinder's, for two
/// parallel ones), and the line of two planes starts at its point nearest
/// the first plane's origin, a circle about a shared axis takes the
/// frame of the first operand that carries the axis (ADR-0008), and a
/// plane through a cone's or a torus's axis gives a ruling from the apex
/// along `∂P/∂v` or a tube circle whose `t` is the torus's `v`. Swapping
/// the operands gives the same point sets, up to the orientation of a
/// line and the order of two parallel cylinders' rulings — and two
/// crossing cylinders' ellipses bit for bit. A traced section — two
/// quadrics that meet in no conic, or a pair with a torus in it — is a
/// `Curve::Nurbs` fitted within [`crate::SECTION_FIT_FRACTION`] of the
/// tolerance of the exact branch it traces, at the same parameter and to
/// the branch's own precision there ([`crate::SectionBranch::distance`]) — and
/// so of both surfaces — at the tracer's parametrisation and
/// orientation, which depend on the two surfaces and never on their
/// order: swapping the operands gives it bit for bit (ADR-0018,
/// ADR-0019). A tube circle of a torus section is the exception the
/// tracer answers exactly: a `Curve::Circle` on the torus, parametrised
/// by its `v`.
#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceIntersection {
    /// The surfaces do not meet: parallel planes apart by more than the
    /// linear tolerance, a plane clear of a cylinder, two cylinders apart
    /// or one nested in the other, coaxial surfaces of revolution whose
    /// meridians never meet.
    Empty,
    /// The surfaces are the same surface within the tolerance; there is
    /// no curve to return.
    Coincident,
    /// The surfaces meet along these curves and at these isolated points,
    /// each a crossing or a touch (ADR-0018): at least one of the two
    /// lists is non-empty. The curves come in the order the arm that
    /// found them documents — rulings across their plane, circles
    /// ascending along the shared axis, parallel elliptic cylinders'
    /// rulings by the first section's parameter — crossings and touches
    /// interleaved in that one order, and a traced section's tube
    /// circles ([`crate::SectionTrace::circles`]) before its branches,
    /// each in the tracer's ([`crate::SectionTrace::branches`]); the points ascend
    /// along the surfaces' shared axis in the direction the first operand
    /// carrying it points, and a traced section's singular points by the
    /// walked ruling through each. A point is on a curve of the same
    /// result only as the end of traced branches through it: a singular
    /// point where the section crosses itself.
    Meets {
        /// The curves the surfaces share.
        curves: Vec<MeetCurve>,
        /// The isolated points they share.
        points: Vec<MeetPoint>,
    },
}

/// How two surfaces meet along a curve or at a point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum MeetKind {
    /// The surfaces cross: along a curve, each passes from one side of the
    /// other to the other; at a point, the meeting is a crossing through
    /// a singular point — a plane perpendicular to a cone through its
    /// apex, two cones closing on one apex.
    Crossing,
    /// The surfaces touch without crossing: along a curve, tangent all
    /// along it — a plane on a cylinder along a ruling, two cylinders
    /// with parallel axes touching, a sphere on a cylinder of its radius;
    /// at a point, tangent there and apart around it — a plane on a
    /// sphere, two spheres touching.
    Touch,
}

/// A curve two surfaces share, and how they meet along it.
#[derive(Debug, Clone, PartialEq)]
pub struct MeetCurve {
    /// The curve, on both surfaces.
    pub curve: Curve,
    /// Whether the surfaces cross or touch along it.
    pub kind: MeetKind,
}

/// An isolated point two surfaces share, and how they meet there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MeetPoint {
    /// The point, on both surfaces.
    pub point: Point3,
    /// Whether the surfaces cross or touch there.
    pub kind: MeetKind,
}

impl SurfaceIntersection {
    /// `Meets` with every curve of one `kind` and no point; `Empty` when
    /// there is no curve.
    pub(crate) fn curves_of(kind: MeetKind, curves: Vec<Curve>) -> Self {
        if curves.is_empty() {
            return SurfaceIntersection::Empty;
        }
        SurfaceIntersection::Meets {
            curves: curves
                .into_iter()
                .map(|curve| MeetCurve { curve, kind })
                .collect(),
            points: Vec::new(),
        }
    }

    /// The curves of a `Meets`, in its order; none for `Empty` and
    /// `Coincident`.
    ///
    /// ```
    /// use arris_geom::{MeetKind, Surface, intersect_surfaces};
    /// use arris_math::{Aabb, Frame, Point3, Precision, Vec3};
    ///
    /// let side = Surface::Plane { frame: Frame::from_z(Point3::new(2.0, 0.0, 0.0), Vec3::x()).unwrap() };
    /// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
    /// let within = Aabb { min: [-5.0; 3], max: [5.0; 3] };
    /// let hit = intersect_surfaces(&side, &wall, &within, Precision::DEFAULT.tolerance(), &mut arris_math::Meter::default()).unwrap();
    /// assert_eq!(hit.curves().len(), 1);
    /// assert_eq!(hit.curves()[0].kind, MeetKind::Touch);
    /// assert!(hit.points().is_empty());
    /// ```
    pub fn curves(&self) -> &[MeetCurve] {
        match self {
            SurfaceIntersection::Meets { curves, .. } => curves,
            SurfaceIntersection::Empty | SurfaceIntersection::Coincident => &[],
        }
    }

    /// The isolated points of a `Meets`, in its order; none for `Empty`
    /// and `Coincident`.
    pub fn points(&self) -> &[MeetPoint] {
        match self {
            SurfaceIntersection::Meets { points, .. } => points,
            SurfaceIntersection::Empty | SurfaceIntersection::Coincident => &[],
        }
    }
}

/// The intersection of two surfaces, by the case table: every pair with
/// a closed form is computed exactly, a quadric pair that meets in no
/// conic is traced and fitted (ADR-0018), and every other pair is an
/// explicit [`GeomError::Unsupported`] arm — no wildcard, no marcher.
///
/// Guarantees: the result is symmetric under swapping `a` and `b` up to
/// a line's orientation, deterministic bit for bit, and each returned
/// curve lies on both surfaces to rounding — a fitted one within
/// [`crate::SECTION_FIT_FRACTION`] of `tol.linear` of the exact section
/// at its own parameter, so two fits of one section, in two regions,
/// are within twice that of each other where both reach — and a torus's tube
/// circle on the other surface within `tol.linear` of it, exactly as a
/// point of a `Meets` is. `tol.angular` decides parallel and
/// perpendicular; `tol.linear` decides coincident, tangent and empty.
///
/// `within` bounds a section traced on rulings: its branches are clipped
/// where they leave the region's extent along the walked rulings, and a
/// loop inside it is closed ([`crate::trace_quadrics`]). The closed forms
/// ignore it and return their lines unbounded, and so does a torus
/// section, which is bounded already ([`crate::trace_torus`]). A caller
/// that intersects several pairs on the same two surfaces passes one
/// region to all of them, so they get the same curves bit for bit.
///
/// The table, every curve and point of a `Meets` a crossing unless it
/// says touching: plane–plane is `Empty`, `Coincident` or one line;
/// plane–cylinder is a circle when the normal is parallel to the axis,
/// an ellipse when oblique (`b = R`, `a = R / |n · Z|`, centred at the
/// axis's piercing point), and when perpendicular two rulings, one
/// touching ruling or `Empty` by the axis-to-plane distance against `R`.
/// Cylinder–cylinder with parallel axes is `Coincident` or `Empty` when
/// coaxial, one touching ruling when the axes are `ra + rb` or
/// `|ra − rb|` apart, two rulings between those distances and `Empty`
/// beyond them; with crossing axes and equal radii it is two ellipses in
/// the planes bisecting the axes; with skew axes further apart than
/// `ra + rb` it is `Empty`. Crossing axes of unequal radii and skew axes
/// within `ra + rb` meet in a quartic space curve, traced and fitted
/// inside `within`: closed loops as periodic B-splines, a point where the
/// cylinders touch as a point — touching where nothing else meets there,
/// crossing where the branches through it end. A plane against an **elliptic cylinder** (ADR-0014) is decided
/// in every pose: the section ellipse when the normal is parallel to the
/// axis, and when perpendicular two rulings, one touching ruling or
/// `Empty` by the plane's offset against the section's reach along its
/// normal, and oblique an ellipse — the affine image of the section,
/// its axes the singular values of the section's semi-diameters
/// projected onto the plane. An elliptic cylinder against a cylinder or
/// another elliptic cylinder with parallel axes meets where the two
/// sections meet in the plane across the axes, through the quartic of
/// [`arris_math::roots`]: `Coincident`, `Empty`, or a ruling at each
/// meeting of the sections, touching where they touch, ascending by the
/// first operand's section parameter — up to four of them, touches and
/// crossings together (ADR-0018); crossing axes, and the elliptic
/// cylinder against a cone, a sphere or a torus in any pose, meet in a
/// traced and fitted section, and against a NURBS it is `Unsupported`.
/// Every pair with a cone, a sphere or a torus in it is
/// decided when the two share an axis — a plane perpendicular to it, a
/// cylinder, cone or torus on it, a sphere centred on it, and every
/// plane–sphere and sphere–sphere pair — by one arm over the meridian
/// sections in the plane through the axis (ADR-0008, `docs/DATA-MODEL.md`
/// §Curves): circles about the axis and points on it, each crossing or
/// touching, `Coincident` or `Empty`; a plane through a cone's or a
/// torus's axis cuts its meridian, two rulings through the apex or two
/// tube circles. A pair sharing no axis is in general position: a plane
/// against a cone meets it in an exact conic — an ellipse, a parabola or
/// a hyperbola's two branches as rational quadratic NURBS over `within`,
/// or through the apex the apex, one touching ruling or two crossing
/// ones; a cylinder, a cone or a sphere against a cone or a sphere in a
/// traced and fitted section; and a torus against any analytic surface
/// in a section traced in the torus's own parameter plane (ADR-0019,
/// [`crate::trace_torus`]), the spiric sections among them — each tube
/// circle of the torus that lies on the other surface an exact
/// `Curve::Circle`, within `tol.linear` of that surface as the tolerance
/// it was detected in allows, and every other branch fitted. `within` is
/// no part of that one: a torus is compact. Read `IntAna_QuadQuadGeo` in
/// the reference tree for the case analysis, reimplemented on our
/// frames.
///
/// ```
/// use arris_geom::{Curve, MeetKind, Surface, intersect_surfaces};
/// use arris_math::{Aabb, Frame, Meter, Point3, Precision, Vec3};
///
/// let within = Aabb { min: [-10.0; 3], max: [10.0; 3] };
/// let cap = Surface::Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 5.0), Vec3::z()).unwrap() };
/// let wall = Surface::Cylinder { frame: Frame::world(), radius: 2.0 };
/// let hit = intersect_surfaces(&cap, &wall, &within, Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
/// let [meet] = hit.curves() else { panic!() };
/// assert_eq!(meet.kind, MeetKind::Crossing);
/// let Curve::Circle { frame, radius } = &meet.curve else { panic!() };
/// assert_eq!(*radius, 2.0);
/// assert_eq!(frame.origin(), Point3::new(0.0, 0.0, 5.0));
///
/// // A pipe of radius 1 through a pipe of radius 2: two fitted loops.
/// let branch = Surface::Cylinder { frame: Frame::from_z(Point3::origin(), Vec3::x()).unwrap(), radius: 1.0 };
/// let hit = intersect_surfaces(&wall, &branch, &within, Precision::DEFAULT.tolerance(), &mut Meter::default()).unwrap();
/// assert_eq!(hit.curves().len(), 2);
/// assert!(hit.curves().iter().all(|m| matches!(&m.curve, Curve::Nurbs(c) if c.period().is_some())));
/// ```
pub fn intersect_surfaces(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SurfaceIntersection, GeomError> {
    if !tol.is_consistent() {
        return Err(GeomError::InvalidTolerance(tol));
    }
    match (a, b) {
        (Surface::Plane { frame: pa }, Surface::Plane { frame: pb }) => {
            Ok(plane_plane(pa, pb, tol))
        }
        (Surface::Plane { frame: plane }, Surface::Cylinder { frame, radius })
        | (Surface::Cylinder { frame, radius }, Surface::Plane { frame: plane }) => {
            plane_cylinder(plane, frame, *radius, tol)
        }
        (
            Surface::Cylinder {
                frame: ca,
                radius: ra,
            },
            Surface::Cylinder {
                frame: cb,
                radius: rb,
            },
        ) => cylinder_cylinder(a, b, ca, *ra, cb, *rb, within, tol, meter),
        (
            Surface::Plane { frame: plane },
            Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            },
        )
        | (
            Surface::EllipticCylinder {
                frame,
                major_radius,
                minor_radius,
            },
            Surface::Plane { frame: plane },
        ) => plane_elliptic_cylinder(plane, frame, [*major_radius, *minor_radius], tol),
        (
            Surface::Cylinder {
                frame: ca,
                radius: ra,
            },
            Surface::EllipticCylinder {
                frame: cb,
                major_radius,
                minor_radius,
            },
        ) => elliptic_pair(
            a,
            b,
            (ca, [*ra, *ra]),
            (cb, [*major_radius, *minor_radius]),
            within,
            tol,
            meter,
        ),
        (
            Surface::EllipticCylinder {
                frame: ca,
                major_radius,
                minor_radius,
            },
            Surface::Cylinder {
                frame: cb,
                radius: rb,
            },
        ) => elliptic_pair(
            a,
            b,
            (ca, [*major_radius, *minor_radius]),
            (cb, [*rb, *rb]),
            within,
            tol,
            meter,
        ),
        (
            Surface::EllipticCylinder {
                frame: ca,
                major_radius: aa,
                minor_radius: ab,
            },
            Surface::EllipticCylinder {
                frame: cb,
                major_radius: ba,
                minor_radius: bb,
            },
        ) => elliptic_pair(a, b, (ca, [*aa, *ab]), (cb, [*ba, *bb]), within, tol, meter),
        (
            Surface::EllipticCylinder { .. },
            Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. },
        )
        | (
            Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. },
            Surface::EllipticCylinder { .. },
        ) => crate::section::traced(a, b, within, tol, meter),
        (Surface::EllipticCylinder { .. }, Surface::Nurbs(_))
        | (Surface::Nurbs(_), Surface::EllipticCylinder { .. }) => Err(GeomError::Unsupported {
            a: GeomKind::Surface(a.kind()),
            b: GeomKind::Surface(b.kind()),
        }),
        (
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. },
            Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. },
        )
        | (
            Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. },
            Surface::Plane { .. } | Surface::Cylinder { .. },
        ) => crate::meridian::intersect_coaxial(a, b, within, tol, meter),
        (
            Surface::Nurbs(_),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_),
        )
        | (
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. },
            Surface::Nurbs(_),
        ) => Err(GeomError::Unsupported {
            a: GeomKind::Surface(a.kind()),
            b: GeomKind::Surface(b.kind()),
        }),
    }
}

/// Two surfaces that share no axis, at least one of them a cone, a sphere
/// or a torus — what the meridian arm leaves (ADR-0008): a plane against
/// a cone meets it in an exact conic (`crate::cone_section`); a
/// cylinder, a cone or a sphere against a cone or a sphere in a quartic,
/// traced inside `within` and fitted (`crate::section`); and a torus
/// against any of those or another torus in a section traced in the
/// torus's parameter plane, its tube circles on the other surface exact
/// and the rest fitted (ADR-0019), with `within` no part of it — a torus
/// is compact. A plane against a sphere and two spheres always share an
/// axis, and the pairs with neither a cone, a sphere nor a torus in them
/// never reach here; each is listed, `Unsupported`, so the match stays
/// exhaustive without a wildcard.
pub(crate) fn off_axis(
    a: &Surface,
    b: &Surface,
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SurfaceIntersection, GeomError> {
    match (a, b) {
        (
            Surface::Plane { frame: plane },
            Surface::Cone {
                frame,
                radius,
                half_angle,
            },
        )
        | (
            Surface::Cone {
                frame,
                radius,
                half_angle,
            },
            Surface::Plane { frame: plane },
        ) => crate::cone_section::plane_cone(plane, frame, *radius, *half_angle, within, tol),
        (
            Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. },
            Surface::Cone { .. } | Surface::Sphere { .. },
        )
        | (Surface::Cone { .. } | Surface::Sphere { .. }, Surface::Cylinder { .. }) => {
            crate::section::traced(a, b, within, tol, meter)
        }
        (
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. },
            Surface::Torus { .. },
        )
        | (
            Surface::Torus { .. },
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. },
        ) => crate::section::traced(a, b, within, tol, meter),
        (
            Surface::Plane { .. },
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Sphere { .. }
            | Surface::Nurbs(_),
        )
        | (
            Surface::Cylinder { .. },
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Nurbs(_),
        )
        | (
            Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. },
            Surface::EllipticCylinder { .. } | Surface::Nurbs(_),
        )
        | (Surface::Sphere { .. }, Surface::Plane { .. })
        | (
            Surface::EllipticCylinder { .. } | Surface::Nurbs(_),
            Surface::Plane { .. }
            | Surface::Cylinder { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_),
        ) => Err(GeomError::Unsupported {
            a: GeomKind::Surface(a.kind()),
            b: GeomKind::Surface(b.kind()),
        }),
    }
}

/// Two cylinders: parallel axes in [`parallel_cylinders`], crossing axes
/// of equal radii in [`crossing_cylinders`], and skew axes further apart
/// than the two radii `Empty` — every point of a cylinder is within its
/// radius of its axis, so by the triangle inequality the two never meet.
/// Crossing axes of unequal radii and skew axes within the radii meet in
/// a quartic space curve with no conic form, traced inside `within` and
/// fitted (`crate::section`, ADR-0018).
#[allow(clippy::too_many_arguments)]
fn cylinder_cylinder(
    a: &Surface,
    b: &Surface,
    ca: &Frame,
    ra: f64,
    cb: &Frame,
    rb: f64,
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SurfaceIntersection, GeomError> {
    let offset = cb.origin() - ca.origin();
    if line_angle(&ca.z(), &cb.z()) <= tol.angular {
        return Ok(parallel_cylinders(ca, ra, rb, offset, tol));
    }
    // The axes are not parallel, so their cross product has a length of at
    // least sin(tol.angular) and normalises; only a non-finite frame fails.
    let Some(common) = UnitVec3::try_new(ca.z().cross(&cb.z()), 0.0) else {
        return Err(frame_degenerate(GeomKind::Surface(
            crate::SurfaceKind::Cylinder,
        )));
    };
    // The length of the axes' common perpendicular: their nearest approach.
    let gap = common.dot(&offset).abs();
    if gap > tol.linear {
        return if gap > ra + rb + tol.linear {
            Ok(SurfaceIntersection::Empty)
        } else {
            crate::section::traced(a, b, within, tol, meter)
        };
    }
    if (ra - rb).abs() > tol.linear {
        return crate::section::traced(a, b, within, tol, meter);
    }
    crossing_cylinders(ca, cb, offset, 0.5 * (ra + rb))
}

/// Two cylinders whose axes are parallel, `d` the distance between the
/// axes: coaxial (`d` within `tol.linear`) is `Coincident` when the radii
/// agree and `Empty` when they do not; `d` within `tol.linear` of
/// `ra + rb` (outside) or of `|ra − rb|` (inside) is one touching ruling;
/// strictly between the two, the two crossing rulings through the
/// crossing points of the two circles in a plane across the axes;
/// otherwise `Empty`. Every ruling runs along `ca`'s `Z` from its point in
/// the plane across the axis through `ca`'s origin — the point nearest
/// that origin — and the two are ordered by their offset along
/// `Z × ŵ`, `ŵ` the unit vector from `ca`'s axis toward `cb`'s: negative
/// first. The tangent ruling is on `ca` to rounding, `ra` along `±ŵ`.
fn parallel_cylinders(
    ca: &Frame,
    ra: f64,
    rb: f64,
    offset: Vec3,
    tol: Tolerance,
) -> SurfaceIntersection {
    let axis = ca.z();
    let z: Vec3 = axis.into_inner();
    let across = offset - offset.dot(&z) * z;
    let d = across.norm();
    if d <= tol.linear {
        return if (ra - rb).abs() <= tol.linear {
            SurfaceIntersection::Coincident
        } else {
            SurfaceIntersection::Empty
        };
    }
    let towards = across / d;
    // The crossing points' offset along `towards` from the first axis, by
    // the radical line of the two circles.
    let along = (d * d + ra * ra - rb * rb) / (2.0 * d);
    let outside = (d - (ra + rb)).abs() <= tol.linear;
    let inside = (d - (ra - rb).abs()).abs() <= tol.linear;
    if outside || inside {
        // `along` is `ra` at an outside touch and at an inside one around
        // the smaller cylinder, `−ra` at an inside one within the larger:
        // its sign says which side of the first axis the ruling is on.
        return SurfaceIntersection::curves_of(
            MeetKind::Touch,
            vec![Curve::Line {
                origin: ca.origin() + ra.copysign(along) * towards,
                direction: axis,
            }],
        );
    }
    if d > ra + rb || d < (ra - rb).abs() {
        return SurfaceIntersection::Empty;
    }
    // Strictly between the tangent distances `|along| < ra`, clear of it by
    // more than the tolerance; the clamp only absorbs rounding.
    let half = (ra * ra - along * along).max(0.0).sqrt();
    let side = z.cross(&towards);
    let foot = ca.origin() + along * towards;
    SurfaceIntersection::curves_of(
        MeetKind::Crossing,
        vec![
            Curve::Line {
                origin: foot - half * side,
                direction: axis,
            },
            Curve::Line {
                origin: foot + half * side,
                direction: axis,
            },
        ],
    )
}

/// Two cylinders of one `radius` whose axes cross: two crossing
/// ellipses in the planes that bisect the axes, centred at the crossing.
/// With `a` and `b` the axes, `b` flipped so that `a · b ≥ 0` and `ψ` the
/// angle between them, the first ellipse has `Z` along `a − b`, `X` along
/// `a + b` and major radius `R / sin(ψ/2)`; the second has `Z` along
/// `a + b`, `X` along `a − b` and major radius `R / cos(ψ/2)`; both have
/// minor radius `R` along `a × b`. A point equidistant from both axes in
/// a plane through the crossing is on one cylinder exactly when it is on
/// the other, and those planes are the bisectors; each ellipse is then
/// `ca`'s oblique section. Each `Z` and `X` takes the sign that makes its
/// largest-magnitude component positive, the lower index on a tie, and
/// the crossing is the midpoint of the axes' nearest points summed in
/// either order: swapping the operands negates `a − b` exactly and leaves
/// `a + b` and the midpoint as they were, so it gives the same ellipses bit
/// for bit, the same way round — one parametrisation, and one fit of each
/// pcurve. The two ellipses cross each other at `±R` along `a × b`.
fn crossing_cylinders(
    ca: &Frame,
    cb: &Frame,
    offset: Vec3,
    radius: f64,
) -> Result<SurfaceIntersection, GeomError> {
    let a: Vec3 = ca.z().into_inner();
    let b: Vec3 = if ca.z().dot(&cb.z()) < 0.0 {
        -cb.z().into_inner()
    } else {
        cb.z().into_inner()
    };
    // The nearest points of the two axes: `s` along `a` from `ca`'s origin
    // and `t` along `b` from `cb`'s, `1 − c²` being `sin²ψ`, not zero.
    let c = a.dot(&b);
    let denom = 1.0 - c * c;
    let (on_a, on_b) = (offset.dot(&a), offset.dot(&b));
    let s = (on_a - c * on_b) / denom;
    let t = (c * on_a - on_b) / denom;
    let near_a = ca.origin() + s * a;
    let near_b = cb.origin() + t * b;
    let centre = Point3::from(0.5 * (near_a.coords + near_b.coords));
    // Signed so that neither operand order is preferred: see the doc.
    let canonical = |v: Vec3| {
        let k = (0..3).fold(0, |k, i| if v[i].abs() > v[k].abs() { i } else { k });
        if v[k] < 0.0 { -v } else { v }
    };
    let (minus, plus) = (canonical(a - b), canonical(a + b));
    // `|a − b| = 2 sin(ψ/2)` and `|a + b| = 2 cos(ψ/2)`, each measured
    // directly so a small angle keeps its digits.
    let ellipse = |z: Vec3, x: Vec3, half_chord: f64| {
        Frame::new(centre, z, x)
            .map(|frame| Curve::Ellipse {
                frame,
                major_radius: 2.0 * radius / half_chord,
                minor_radius: radius,
            })
            .map_err(|_| frame_degenerate(GeomKind::Surface(crate::SurfaceKind::Cylinder)))
    };
    Ok(SurfaceIntersection::curves_of(
        MeetKind::Crossing,
        vec![
            ellipse(minus, plus, minus.norm())?,
            ellipse(plus, minus, plus.norm())?,
        ],
    ))
}

/// The angle in `[0, π/2]` between the lines carried by two unit vectors:
/// `atan2` of the cross and dot magnitudes, well conditioned at both ends
/// where `acos` is not.
pub(crate) fn line_angle(a: &UnitVec3, b: &UnitVec3) -> f64 {
    a.cross(b).norm().atan2(a.dot(b).abs())
}

fn plane_plane(pa: &Frame, pb: &Frame, tol: Tolerance) -> SurfaceIntersection {
    let (n1, n2) = (pa.z(), pb.z());
    if line_angle(&n1, &n2) <= tol.angular {
        let gap = n1.dot(&(pb.origin() - pa.origin())).abs();
        return if gap <= tol.linear {
            SurfaceIntersection::Coincident
        } else {
            SurfaceIntersection::Empty
        };
    }
    // The normals are not parallel, so the cross product has a length of
    // at least sin(tol.angular) and normalises; a failure here is a
    // non-finite frame, which no constructor produces.
    let across = n1.cross(&n2);
    let Some(direction) = UnitVec3::try_new(across, 0.0) else {
        return SurfaceIntersection::Empty;
    };
    // Of the points on both planes, the one nearest to plane a's origin:
    // solve n1·x = h1, n2·x = h2 in the span of n1 and n2 over a's origin.
    // The system's determinant is `1 − c²`, taken as `|n1 × n2|²`: for
    // planes 5.6e-9 of a radian from parallel `c²` rounds to one and
    // `1 − c²` to zero, which put the line's origin at NaN (the
    // `intersect_curves` fuzz target, ADR-0024 §5), where the cross
    // product keeps sin²θ to its own rounding.
    let c = n1.dot(&n2);
    let h1 = 0.0;
    let h2 = n2.dot(&(pb.origin() - pa.origin()));
    let denom = across.norm_squared();
    let s1 = (h1 - h2 * c) / denom;
    let s2 = (h2 - h1 * c) / denom;
    let origin = pa.origin() + s1 * n1.into_inner() + s2 * n2.into_inner();
    SurfaceIntersection::curves_of(MeetKind::Crossing, vec![Curve::Line { origin, direction }])
}

fn plane_cylinder(
    plane: &Frame,
    cyl: &Frame,
    radius: f64,
    tol: Tolerance,
) -> Result<SurfaceIntersection, GeomError> {
    let (n, axis) = (plane.z(), cyl.z());
    let angle = line_angle(&n, &axis);
    if angle <= tol.angular {
        // Normal along the axis: the plane cuts a circle at the piercing
        // point, with the cylinder's own axes so the seam is shared.
        let t = n.dot(&(plane.origin() - cyl.origin())) / n.dot(&axis);
        let frame = cyl.with_origin(cyl.origin() + t * axis.into_inner());
        return Ok(SurfaceIntersection::curves_of(
            MeetKind::Crossing,
            vec![Curve::Circle { frame, radius }],
        ));
    }
    if FRAC_PI_2 - angle <= tol.angular {
        // Normal across the axis: the axis is parallel to the plane at a
        // signed distance `dist`, and the section is made of rulings.
        let dist = n.dot(&(cyl.origin() - plane.origin()));
        let foot = cyl.origin() - dist * n.into_inner();
        if (dist.abs() - radius).abs() <= tol.linear {
            return Ok(SurfaceIntersection::curves_of(
                MeetKind::Touch,
                vec![Curve::Line {
                    origin: foot,
                    direction: axis,
                }],
            ));
        }
        if dist.abs() < radius {
            let half = (radius * radius - dist * dist).sqrt();
            let Some(across) = UnitVec3::try_new(n.cross(&axis), 0.0) else {
                return Err(frame_degenerate(GeomKind::Surface(
                    crate::SurfaceKind::Cylinder,
                )));
            };
            let across: Vec3 = across.into_inner();
            return Ok(SurfaceIntersection::curves_of(
                MeetKind::Crossing,
                vec![
                    Curve::Line {
                        origin: foot - half * across,
                        direction: axis,
                    },
                    Curve::Line {
                        origin: foot + half * across,
                        direction: axis,
                    },
                ],
            ));
        }
        return Ok(SurfaceIntersection::Empty);
    }
    // Oblique: an ellipse centred at the piercing point, minor axis `R`
    // across the axis, major axis `R / cos` along the axis's projection
    // onto the plane — the direction of increasing `v`.
    let cos = n.dot(&axis).abs();
    let t = n.dot(&(plane.origin() - cyl.origin())) / n.dot(&axis);
    let centre: Point3 = cyl.origin() + t * axis.into_inner();
    let frame = Frame::new(centre, n.into_inner(), axis.into_inner())
        .map_err(|_| frame_degenerate(GeomKind::Surface(crate::SurfaceKind::Plane)))?;
    Ok(SurfaceIntersection::curves_of(
        MeetKind::Crossing,
        vec![Curve::Ellipse {
            frame,
            major_radius: radius / cos,
            minor_radius: radius,
        }],
    ))
}

/// A plane against an elliptic cylinder, `[a, b]` its radii, in every
/// pose (ADR-0014). Normal along the axis: the section ellipse at the
/// piercing point, with the cylinder's own axes so the seam is shared.
/// Normal across the axis: the plane cuts the section in the line at
/// its signed offset `d` from the section's centre along the plane's
/// normal, and the section reaches `±M` along that normal, `M = √((a
/// n·X)² + (b n·Y)²)`; `|d|` within `tol.linear` of `M` is one touching
/// ruling, `|d| < M` two crossing rulings at `φ ± acos(d / M)`, `φ`
/// the parameter of the farthest reach, ordered by their offset along
/// `n × Z` (negative first, as plane–cylinder orders them), and beyond
/// `Empty`. Oblique: an ellipse centred at the axis's piercing point,
/// the affine image of the section — `cos u·A + sin u·B` with `A`, `B`
/// the section's semi-diameters slid along the axis into the plane —
/// whose axes are the singular values of `[A | B]` in the plane's own
/// basis (`e₁` the axis projected onto the plane, `e₂ = n × e₁`); its
/// `Z` is the plane's normal and its `X` the major axis.
fn plane_elliptic_cylinder(
    plane: &Frame,
    cyl: &Frame,
    [a, b]: [f64; 2],
    tol: Tolerance,
) -> Result<SurfaceIntersection, GeomError> {
    let (n, axis) = (plane.z(), cyl.z());
    let angle = line_angle(&n, &axis);
    let kind = GeomKind::Surface(crate::SurfaceKind::EllipticCylinder);
    if angle <= tol.angular {
        let t = n.dot(&(plane.origin() - cyl.origin())) / n.dot(&axis);
        let frame = cyl.with_origin(cyl.origin() + t * axis.into_inner());
        return Ok(SurfaceIntersection::curves_of(
            MeetKind::Crossing,
            vec![Curve::Ellipse {
                frame,
                major_radius: a,
                minor_radius: b,
            }],
        ));
    }
    let ruling_at = |u: f64| Curve::Line {
        origin: Surface::EllipticCylinder {
            frame: *cyl,
            major_radius: a,
            minor_radius: b,
        }
        .point(u, 0.0),
        direction: axis,
    };
    if FRAC_PI_2 - angle <= tol.angular {
        // The plane's normal in the section, and the section line's
        // offset from the centre along it.
        let local = cyl.vec_to_local(n.into_inner());
        let Some(m) = Vec2::new(local.x, local.y).try_normalize(0.0) else {
            return Err(frame_degenerate(kind));
        };
        let o = cyl.to_local(plane.origin());
        let d = m.dot(&Vec2::new(o.x, o.y));
        let reach = (a * m.x).hypot(b * m.y);
        let phase = (b * m.y).atan2(a * m.x);
        if (d.abs() - reach).abs() <= tol.linear {
            let u = if d >= 0.0 {
                phase
            } else {
                phase + core::f64::consts::PI
            };
            return Ok(SurfaceIntersection::curves_of(
                MeetKind::Touch,
                vec![ruling_at(u)],
            ));
        }
        if d.abs() >= reach {
            return Ok(SurfaceIntersection::Empty);
        }
        let half = (d / reach).clamp(-1.0, 1.0).acos();
        let Some(across) = UnitVec3::try_new(n.cross(&axis), 0.0) else {
            return Err(frame_degenerate(kind));
        };
        let mut rulings = [ruling_at(phase - half), ruling_at(phase + half)];
        let offset = |c: &Curve| match c {
            Curve::Line { origin, .. } => (origin - cyl.origin()).dot(&across),
            _ => 0.0,
        };
        if offset(&rulings[0]) > offset(&rulings[1]) {
            rulings.swap(0, 1);
        }
        return Ok(SurfaceIntersection::curves_of(
            MeetKind::Crossing,
            rulings.to_vec(),
        ));
    }
    // Oblique: the section's semi-diameters slid along the axis into the
    // plane are conjugate semi-diameters of the section ellipse.
    let nz = n.dot(&axis);
    let t = n.dot(&(plane.origin() - cyl.origin())) / nz;
    let centre: Point3 = cyl.origin() + t * axis.into_inner();
    let z: Vec3 = axis.into_inner();
    let slide = |w: Vec3| w - (n.dot(&w) / nz) * z;
    let big = a * slide(cyl.x().into_inner());
    let small = b * slide(cyl.y().into_inner());
    let Some(e1) = UnitVec3::try_new(z - nz * n.into_inner(), 0.0) else {
        return Err(frame_degenerate(kind));
    };
    // The axis's part in the plane is short when the plane is a hair
    // past square to it — the difference of two near-equal unit vectors —
    // and normalised it keeps the rounding of that difference along `n`,
    // 8e-5 of it at 2e-12 from square: `e2` came out short by 3e-9 and a
    // section of radius 1000 was 3e-6 small (the `intersect_surfaces`
    // fuzz target, ADR-0024 §5). One pass of Gram–Schmidt puts it back
    // in the plane.
    let Some(e1) = UnitVec3::try_new(e1.into_inner() - e1.dot(&n) * n.into_inner(), 0.0) else {
        return Err(frame_degenerate(kind));
    };
    let e2 = n.cross(&e1);
    let (major, minor, phi) = principal_axes(
        Vec2::new(big.dot(&e1), big.dot(&e2)),
        Vec2::new(small.dot(&e1), small.dot(&e2)),
    );
    let x = phi.cos() * e1.into_inner() + phi.sin() * e2;
    let frame = Frame::new(centre, n.into_inner(), x).map_err(|_| frame_degenerate(kind))?;
    Ok(SurfaceIntersection::curves_of(
        MeetKind::Crossing,
        vec![Curve::Ellipse {
            frame,
            major_radius: major,
            minor_radius: minor.abs(),
        }],
    ))
}

/// An elliptic cylinder against a cylinder or another elliptic cylinder,
/// each given as its frame and `[a, b]` (the radius twice for a
/// cylinder). Axes parallel within `tol.angular`: the pair meets where
/// the two sections meet in the plane across the first's axis through
/// its origin (`crate::conic2`), each meeting a ruling along the first's
/// `Z` from the section point — `Coincident`, `Empty`, or touching
/// rulings at the touches and crossing ones at the crossings, together
/// and ascending by the first section's parameter (ADR-0018); axes that
/// are not parallel meet in a quartic space curve, traced inside
/// `within` and fitted (`crate::section`).
fn elliptic_pair(
    a: &Surface,
    b: &Surface,
    (ca, [aa, ab]): (&Frame, [f64; 2]),
    (cb, [ba, bb]): (&Frame, [f64; 2]),
    within: &Aabb,
    tol: Tolerance,
    meter: &mut Meter,
) -> Result<SurfaceIntersection, GeomError> {
    if line_angle(&ca.z(), &cb.z()) > tol.angular {
        return crate::section::traced(a, b, within, tol, meter);
    }
    let first = Conic2 {
        centre: Point2::origin(),
        x: Vec2::x(),
        y: Vec2::y(),
        a: aa,
        b: ab,
    };
    let centre = ca.to_local(cb.origin());
    let xb = ca.vec_to_local(cb.x().into_inner());
    // The second's major axis in the section; a circle takes the
    // section's own `u` axis, its implicit form being the same either way.
    let x = if (ba - bb).abs() <= tol.linear {
        Vec2::x()
    } else {
        Vec2::new(xb.x, xb.y)
            .try_normalize(0.0)
            .ok_or_else(|| frame_degenerate(GeomKind::Surface(b.kind())))?
    };
    let second = Conic2 {
        centre: Point2::new(centre.x, centre.y),
        x,
        y: Vec2::new(-x.y, x.x),
        a: ba,
        b: bb,
    };
    let meet = conic_pair(&first, &second, tol).map_err(|e| GeomError::Degenerate {
        kind: GeomKind::Surface(a.kind()),
        reason: format!("the sections' meeting: {e}"),
    })?;
    let ruling = |t: f64| {
        let p = first.point(t);
        Curve::Line {
            origin: ca.to_world(Point3::new(p.x, p.y, 0.0)),
            direction: ca.z(),
        }
    };
    Ok(match meet {
        ConicMeet::Coincident => SurfaceIntersection::Coincident,
        ConicMeet::Empty => SurfaceIntersection::Empty,
        ConicMeet::Meets(meets) => SurfaceIntersection::Meets {
            curves: meets
                .iter()
                .map(|&(t, touch)| MeetCurve {
                    curve: ruling(t),
                    kind: if touch {
                        MeetKind::Touch
                    } else {
                        MeetKind::Crossing
                    },
                })
                .collect(),
            points: Vec::new(),
        },
    })
}

/// The error for a frame that could not be built from an operand's axes:
/// only a non-finite frame reaches it, since the angular tests above rule
/// out parallel axes.
fn frame_degenerate(kind: GeomKind) -> GeomError {
    GeomError::Degenerate {
        kind,
        reason: "non-finite frame".to_owned(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_math::Precision;

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

    /// The curves of a `Meets` with no points, every one of `kind`.
    fn only(r: &SurfaceIntersection, kind: MeetKind) -> Vec<Curve> {
        assert!(r.points().is_empty(), "{r:?}");
        assert!(!r.curves().is_empty(), "{r:?}");
        r.curves()
            .iter()
            .map(|m| {
                assert_eq!(m.kind, kind, "{r:?}");
                m.curve.clone()
            })
            .collect()
    }

    #[test]
    fn coordinate_planes_meet_along_an_axis() {
        let xy = Surface::Plane {
            frame: Frame::world(),
        };
        let yz = Surface::Plane {
            frame: Frame::from_z(Point3::new(3.0, 0.0, 0.0), Vec3::x()).unwrap(),
        };
        let curves = only(
            &intersect_surfaces(
                &xy,
                &yz,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        let Curve::Line { origin, direction } = &curves[0] else {
            panic!()
        };
        assert_eq!(*origin, Point3::new(3.0, 0.0, 0.0));
        assert_eq!(direction.into_inner().abs(), Vec3::y());
    }

    #[test]
    fn parallel_planes_are_empty_or_coincident() {
        let a = Surface::Plane {
            frame: Frame::world(),
        };
        let lifted = Surface::Plane {
            frame: Frame::from_z(Point3::new(1.0, 2.0, 0.5), Vec3::z()).unwrap(),
        };
        let flipped = Surface::Plane {
            frame: Frame::from_z(Point3::new(1.0, 2.0, 0.0), -Vec3::z()).unwrap(),
        };
        assert_eq!(
            intersect_surfaces(
                &a,
                &lifted,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::Empty
        );
        assert_eq!(
            intersect_surfaces(
                &a,
                &flipped,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::Coincident
        );
    }

    #[test]
    fn an_inconsistent_tolerance_is_an_error() {
        let a = Surface::Plane {
            frame: Frame::world(),
        };
        assert!(matches!(
            intersect_surfaces(
                &a,
                &a,
                &within(),
                Tolerance::new(0.0, 1e-12),
                &mut arris_math::Meter::default()
            ),
            Err(GeomError::InvalidTolerance(_))
        ));
    }

    fn elliptic(origin: Point3, a: f64, b: f64) -> Surface {
        Surface::EllipticCylinder {
            frame: Frame::from_z(origin, Vec3::z()).unwrap(),
            major_radius: a,
            minor_radius: b,
        }
    }

    /// Every point of `curve` over a turn or a unit of parameter lies on
    /// both surfaces to rounding, by their projections.
    fn on_both(curve: &Curve, a: &Surface, b: &Surface) {
        for i in 0..=32 {
            let t = match curve {
                Curve::Line { .. } => i as f64 / 32.0 * 4.0 - 2.0,
                _ => i as f64 / 32.0 * core::f64::consts::TAU,
            };
            let p = curve.point(t);
            for s in [a, b] {
                let d = s.project(p).unwrap().distance;
                assert!(d < 1e-12, "{p} is {d} off {:?}", s.kind());
            }
        }
    }

    #[test]
    fn a_plane_cuts_an_elliptic_cylinder_in_every_pose() {
        let wall = elliptic(Point3::origin(), 3.0, 2.0);
        // Across the axis: the section.
        let cap = Surface::Plane {
            frame: Frame::from_z(Point3::new(1.0, 1.0, 5.0), Vec3::z()).unwrap(),
        };
        let c = only(
            &intersect_surfaces(
                &cap,
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        let Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } = &c[0]
        else {
            panic!("{c:?}")
        };
        assert_eq!(frame.origin(), Point3::new(0.0, 0.0, 5.0));
        assert_eq!((*major_radius, *minor_radius), (3.0, 2.0));
        assert_eq!(frame.x().into_inner(), Vec3::x());
        // Along the axis: two rulings at x = 1 (y = ±2√(8/9)), a tangent
        // one at x = 3, none at x = 4 — and the same by the reversed order.
        let side = |x: f64| Surface::Plane {
            frame: Frame::from_z(Point3::new(x, 0.0, 0.0), Vec3::x()).unwrap(),
        };
        let c = only(
            &intersect_surfaces(
                &wall,
                &side(1.0),
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        assert_eq!(c.len(), 2);
        let y = 2.0 * (8.0f64 / 9.0).sqrt();
        let (Curve::Line { origin: o0, .. }, Curve::Line { origin: o1, .. }) = (&c[0], &c[1])
        else {
            panic!("{c:?}")
        };
        // Ordered along `n × Z = −y`, negative first: the +y ruling leads.
        assert!((o0 - Point3::new(1.0, y, 0.0)).norm() < 1e-12, "{c:?}");
        assert!((o1 - Point3::new(1.0, -y, 0.0)).norm() < 1e-12, "{c:?}");
        for r in &c {
            on_both(r, &wall, &side(1.0));
        }
        let c = only(
            &intersect_surfaces(
                &side(3.0),
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Touch,
        );
        let Curve::Line { origin, .. } = &c[0] else {
            panic!()
        };
        assert!((origin - Point3::new(3.0, 0.0, 0.0)).norm() < 1e-12);
        assert_eq!(
            intersect_surfaces(
                &side(4.0),
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::Empty
        );
        // Oblique: an ellipse on both surfaces.
        let tilted = Surface::Plane {
            frame: Frame::from_z(Point3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 2.0, 3.0)).unwrap(),
        };
        let c = only(
            &intersect_surfaces(
                &tilted,
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        assert!(matches!(c[0], Curve::Ellipse { .. }), "{c:?}");
        on_both(&c[0], &tilted, &wall);
        assert_eq!(
            intersect_surfaces(
                &wall,
                &tilted,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::curves_of(MeetKind::Crossing, c)
        );
    }

    #[test]
    fn parallel_elliptic_cylinders_meet_along_rulings() {
        let wall = elliptic(Point3::origin(), 3.0, 2.0);
        // A coaxial cylinder between the radii crosses the section at
        // four parameters; of the major radius it touches at two.
        let bore = |r: f64| Surface::Cylinder {
            frame: Frame::world(),
            radius: r,
        };
        let c = only(
            &intersect_surfaces(
                &wall,
                &bore(2.5),
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        assert_eq!(c.len(), 4);
        for r in &c {
            on_both(r, &wall, &bore(2.5));
        }
        let c = only(
            &intersect_surfaces(
                &bore(3.0),
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Touch,
        );
        assert_eq!(c.len(), 2);
        assert_eq!(
            intersect_surfaces(
                &bore(1.0),
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::Empty
        );
        assert_eq!(
            intersect_surfaces(
                &wall,
                &wall,
                &within(),
                tol(),
                &mut arris_math::Meter::default()
            )
            .unwrap(),
            SurfaceIntersection::Coincident
        );
        // Two equal elliptic cylinders offset along the major axis: two
        // rulings at x = 1, y = ±2√(8/9), like the slot's two ends.
        let other = elliptic(Point3::new(2.0, 0.0, 7.0), 3.0, 2.0);
        let c = only(
            &intersect_surfaces(
                &wall,
                &other,
                &within(),
                tol(),
                &mut arris_math::Meter::default(),
            )
            .unwrap(),
            MeetKind::Crossing,
        );
        assert_eq!(c.len(), 2);
        for r in &c {
            on_both(r, &wall, &other);
        }
        // A crossing axis has no closed form: a smaller cylinder through
        // the wall meets it in two traced loops, fitted and periodic.
        let crossing = Surface::Cylinder {
            frame: Frame::from_z(Point3::origin(), Vec3::x()).unwrap(),
            radius: 1.0,
        };
        let r = intersect_surfaces(
            &wall,
            &crossing,
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        let c = only(&r, MeetKind::Crossing);
        assert_eq!(c.len(), 2, "{r:?}");
        assert!(
            c.iter()
                .all(|c| matches!(c, Curve::Nurbs(n) if n.period().is_some())),
            "{r:?}"
        );
    }

    /// A circle of radius 2 about (1, 0) touches the 3 × 2 section at its
    /// vertex (3, 0) from inside and crosses it twice at x = 3/5: one
    /// `Meets` with a touching ruling between two crossing ones, by the
    /// section's parameter (ADR-0018).
    #[test]
    fn parallel_cylinders_that_both_touch_and_cross_meet_in_both_kinds() {
        let wall = elliptic(Point3::origin(), 3.0, 2.0);
        let mixed = Surface::Cylinder {
            frame: Frame::from_z(Point3::new(1.0, 0.0, 0.0), Vec3::z()).unwrap(),
            radius: 2.0,
        };
        let r = intersect_surfaces(
            &wall,
            &mixed,
            &within(),
            tol(),
            &mut arris_math::Meter::default(),
        )
        .unwrap();
        assert!(r.points().is_empty(), "{r:?}");
        let kinds: Vec<MeetKind> = r.curves().iter().map(|m| m.kind).collect();
        assert_eq!(
            kinds,
            [MeetKind::Touch, MeetKind::Crossing, MeetKind::Crossing],
            "{r:?}"
        );
        // The section's `x` at each ruling: the touch at 3, both crossings
        // at 3/5; `y` of the crossings ascending with the parameter.
        let origins: Vec<Point3> = r
            .curves()
            .iter()
            .map(|m| match &m.curve {
                Curve::Line { origin, .. } => *origin,
                other => panic!("{other:?}"),
            })
            .collect();
        assert!(
            (origins[0] - Point3::new(3.0, 0.0, 0.0)).norm() < 1e-7,
            "{origins:?}"
        );
        let y = 2.0 * (1.0f64 - 0.04).sqrt();
        assert!(
            (origins[1] - Point3::new(0.6, y, 0.0)).norm() < 1e-9,
            "{origins:?}"
        );
        assert!(
            (origins[2] - Point3::new(0.6, -y, 0.0)).norm() < 1e-9,
            "{origins:?}"
        );
        for m in r.curves() {
            on_both(&m.curve, &wall, &mixed);
        }
    }
}
