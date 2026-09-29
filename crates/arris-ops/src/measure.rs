//! `measure`: the mass properties of a body, integrated exactly over the
//! B-Rep (`docs/ARCHITECTURE.md` §Operations). A query, not an
//! operation: it makes no body, records no provenance and opens no
//! transaction.
//!
//! Every quantity is a flux integral over the body's faces by Green's
//! theorem, taken in each face's own (u, v) through [`face_flux`] with
//! the face use's sign — the one integral the checker's B2 row computes
//! an enclosed volume by. The
//! divergence of the field is the integrand of the volume integral:
//! `P · (∂P/∂u × ∂P/∂v) / 3` for the volume itself, `x² / 2 · n_x` for
//! `∫ x dV`, `x³ / 3 · n_x` for `∫ x² dV`, `x² y / 2 · n_x` for
//! `∫ x y dV`. Nothing is discretised, so the numbers are the geometry's
//! and not a mesh's: they match Open CASCADE to 1e-9 relative on the
//! fixture corpus.

use arris_check::arris_topo::arris_geom::integrate::{region_integral, surface_grid};
use arris_check::arris_topo::arris_math::{Aabb, Control, Matrix3, Meter, Point3, Vec3};
use arris_check::arris_topo::entity::{Body as BodyEntity, BodyKind};
use arris_check::arris_topo::{Body, FaceId, Model, Shape};
use arris_check::flux::{FluxError, face_flux};
use arris_check::{Level, check};

use crate::error::{OpError, Reason};

/// The mass properties of a body of unit density
/// (`docs/ARCHITECTURE.md` §Operations): what
/// [`mass_properties`] returns.
///
/// The mass is the volume, since the density is one. The inertia tensor
/// is about the centroid, in the physical convention — `∫ (|r|² I − r
/// rᵀ) dV`, so the diagonal holds the moments of inertia and the
/// off-diagonal the *negated* products — and [`Self::inertia_about`]
/// carries it to any other point by the parallel-axis theorem.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MassProperties {
    /// The enclosed volume, positive for a solid.
    pub volume: f64,
    /// The total area of the body's faces.
    pub area: f64,
    /// The centre of mass.
    pub centroid: Point3,
    /// The inertia tensor about the centroid, unit density.
    pub inertia: Matrix3,
}

impl MassProperties {
    /// The inertia tensor about `point` instead of the centroid, by the
    /// parallel-axis theorem: `I + m (|r|² I₃ − r rᵀ)` with `r = point −
    /// centroid` and `m` the volume. `inertia_about(centroid)` is
    /// [`Self::inertia`] to rounding.
    ///
    /// ```
    /// use arris_ops::{measure, primitive_box};
    /// use arris_ops::arris_check::arris_topo::Model;
    /// use arris_ops::arris_check::arris_topo::arris_math::Point3;
    ///
    /// let mut m = Model::default();
    /// let (body, _) = primitive_box(
    ///     &mut m,
    ///     Point3::new(-1.0, -1.0, -1.0),
    ///     Point3::new(1.0, 1.0, 1.0), &arris_ops::Control::NONE,
    /// )
    /// .unwrap();
    /// let p = measure::mass_properties(&m, body, &arris_ops::Control::NONE).unwrap();
    /// // A cube of side 2: m d² / 6 about the centroid, and 8 · (1 + 1)
    /// // more about a corner.
    /// assert!((p.inertia[(0, 0)] - 8.0 * 4.0 / 6.0).abs() < 1e-12);
    /// let corner = p.inertia_about(Point3::new(1.0, 1.0, 1.0));
    /// assert!((corner[(0, 0)] - (8.0 * 4.0 / 6.0 + 8.0 * 2.0)).abs() < 1e-12);
    /// ```
    pub fn inertia_about(&self, point: Point3) -> Matrix3 {
        let r = point - self.centroid;
        self.inertia + self.volume * (r.dot(&r) * Matrix3::identity() - r * r.transpose())
    }
}

/// A flux integrand: the point of the surface, already translated by the
/// pass's offset, and `∂P/∂u × ∂P/∂v`, the unnormalised normal whose
/// length is the area element.
type Integrand = fn(Vec3, Vec3) -> f64;

/// The first pass: the volume and the three first moments `∫ x dV`, from
/// the fields whose divergence each is — `P / 3` and `x² / 2` along each
/// axis.
const FIRST: [Integrand; 4] = [
    |p, n| p.dot(&n) / 3.0,
    |p, n| p.x * p.x / 2.0 * n.x,
    |p, n| p.y * p.y / 2.0 * n.y,
    |p, n| p.z * p.z / 2.0 * n.z,
];

/// The second pass, taken about the centroid: `∫ x² dV` from `x³ / 3`
/// and `∫ x y dV` from `x² y / 2`, along each axis and each pair.
const SECOND: [Integrand; 6] = [
    |p, n| p.x.powi(3) / 3.0 * n.x,
    |p, n| p.y.powi(3) / 3.0 * n.y,
    |p, n| p.z.powi(3) / 3.0 * n.z,
    |p, n| p.x * p.x * p.y / 2.0 * n.x,
    |p, n| p.y * p.y * p.z / 2.0 * n.y,
    |p, n| p.z * p.z * p.x / 2.0 * n.z,
];

/// The mass properties of `body` at unit density: volume, area, centroid
/// and the inertia tensor about the centroid, integrated exactly over
/// the B-Rep — no mesh, no chord tolerance, and no sampling of the
/// surfaces beyond a quadrature the closed forms are exact under.
///
/// The volume and the first moments are integrated about the centre of
/// the body's vertices rather than the origin: faces whose pcurves of one
/// section are fitted apart close only to the fit, and each gap leaks
/// flux in proportion to its distance from the point the integral is
/// taken about — the body's size, not its distance from the origin.
///
/// The second moments are integrated about the centroid itself rather
/// than about the origin and carried there by the parallel-axis theorem:
/// a small body far from the origin has `∫ x² dV ≈ V |c|²`, orders of
/// magnitude above the tensor that survives the subtraction, and the
/// theorem would spend the difference on rounding.
///
/// A query: it takes `&Model`, returns no body and records no
/// provenance. The body must be a `Solid`; a `Sheet` or a wire has no
/// enclosed volume and is [`OpError::Degenerate`] with
/// [`Reason::NotSolid`].
///
/// Errors: [`OpError::InvalidInput`] when the body fails the checker
/// (debug builds, and release with the `paranoid` feature, as every
/// operation checks its input); [`OpError::NotFound`] when the body or
/// an entity it refers to does not resolve; [`OpError::Degenerate`] for
/// a body that is not a solid, or whose faces enclose no positive
/// volume.
///
/// ```
/// use arris_ops::measure::mass_properties;
/// use arris_ops::primitive_cylinder;
/// use arris_ops::arris_check::arris_topo::Model;
/// use arris_ops::arris_check::arris_topo::arris_math::{Axis, Point3};
/// use core::f64::consts::PI;
///
/// let mut m = Model::default();
/// let (body, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 2.0, 5.0, &arris_ops::Control::NONE).unwrap();
/// let p = mass_properties(&m, body, &arris_ops::Control::NONE).unwrap();
/// let volume = PI * 4.0 * 5.0;
/// assert!((p.volume - volume).abs() < 1e-12 * volume);
/// assert!((p.centroid - Point3::new(0.0, 0.0, 2.5)).norm() < 1e-12);
/// // About its own axis: m r² / 2.
/// assert!((p.inertia[(2, 2)] - volume * 2.0).abs() < 1e-12 * volume);
/// ```
pub fn mass_properties(
    m: &Model,
    body: Body,
    control: &Control<'_>,
) -> Result<MassProperties, OpError> {
    crate::verify_input(m, body)?;
    let mut meter = Meter::new(control);
    let entity = m.body(body.id)?;
    if entity.kind() != BodyKind::Solid {
        return Err(OpError::Degenerate {
            entities: vec![Shape::new(body.id, body.orientation)],
            reason: Reason::NotSolid,
        });
    }
    let faces = face_uses(m, body, entity)?;

    let reference = reference_point(m, body)?;
    let first = integrate_faces(m, body, &faces, reference, &FIRST, &mut meter)?;
    let volume = first[0];
    if !(volume.is_finite() && volume > 0.0) {
        return Err(OpError::Degenerate {
            entities: vec![Shape::new(body.id, body.orientation)],
            reason: Reason::NotPositive {
                what: "the enclosed volume",
                value: volume,
            },
        });
    }
    let centroid = Point3::from(reference + Vec3::new(first[1], first[2], first[3]) / volume);
    let second = integrate_faces(m, body, &faces, centroid.coords, &SECOND, &mut meter)?;
    // The physical tensor: the diagonal is `∫ (|r|² − x_i²) dV`, the
    // off-diagonal the negated products, symmetric by construction.
    let inertia = Matrix3::new(
        second[1] + second[2],
        -second[3],
        -second[5],
        -second[3],
        second[0] + second[2],
        -second[4],
        -second[5],
        -second[4],
        second[0] + second[1],
    );
    Ok(MassProperties {
        volume,
        area: face_area(m, &faces, &mut meter)?,
        centroid,
        inertia,
    })
}

/// The point the first pass is taken about: the centre of the box
/// around the body's vertices, or the origin for a body with none.
///
/// A flux over a closed surface does not depend on where it is taken
/// about, but a boundary whose faces carry their own fitted pcurves of
/// one section is closed only to its tolerance, and every gap between
/// them leaks flux of `P / 3` and `x² / 2` in proportion to `|P|` and
/// `|P|²`. About the origin that is the body's distance from the origin;
/// about a point of the body it is the body's own size. A cylinder posed
/// 100 from the origin and cut by an oblique plane measures 1e-9 off its
/// closed form about the origin and 1e-12 about its own vertices.
fn reference_point(m: &Model, body: Body) -> Result<Vec3, OpError> {
    let mut points = Vec::new();
    for v in m.vertices(body)? {
        points.push(m.vertex(v.id)?.point().coords.into());
    }
    Ok(Aabb::of_points(&points).map_or_else(Vec3::zeros, |b| Vec3::from(b.center())))
}

/// The body's face uses with their effective orientation, through the
/// shells as the checker's B2 row walks them: a face used by two shells
/// is integrated once per use.
fn face_uses(m: &Model, body: Body, entity: &BodyEntity) -> Result<Vec<(FaceId, f64)>, OpError> {
    let mut out = Vec::new();
    for shell_use in entity.shells() {
        let orientation = body.orientation.compose(shell_use.orientation);
        let shell = m.shell(shell_use.id)?;
        for face_use in shell.faces() {
            out.push((
                face_use.id,
                orientation.compose(face_use.orientation).sign(),
            ));
        }
    }
    Ok(out)
}

/// `∬ f du dv` for each of `integrands` over every face, summed with the
/// face use's sign: the flux of each field out of the body through
/// [`face_flux`], the checker's own integral, with the surface's point
/// translated by `−offset`.
///
/// Errors: [`OpError::NotFound`] naming a face whose face, surface or
/// loop does not resolve; [`OpError::InvalidInput`] with the checker's
/// report for a face whose loops cannot be integrated (L1, E1).
fn integrate_faces<const N: usize>(
    m: &Model,
    body: Body,
    faces: &[(FaceId, f64)],
    offset: Vec3,
    integrands: &[Integrand; N],
    meter: &mut Meter<'_>,
) -> Result<[f64; N], OpError> {
    let mut totals = [0.0; N];
    for &(id, sign) in faces {
        meter.tick()?;
        for (total, f) in totals.iter_mut().zip(integrands) {
            let flux = face_flux(m, id, |p, n| f(p.coords - offset, n)).map_err(|e| match e {
                FluxError::NotFound(e) => OpError::from(e),
                FluxError::Unintegrable { .. } => OpError::InvalidInput {
                    body,
                    report: Box::new(check(m, body, Level::Fast)),
                },
            })?;
            *total += sign * flux;
        }
    }
    Ok(totals)
}

/// The total area of the faces: `∬ |∂P/∂u × ∂P/∂v| du dv` over each
/// one's region. A surface integral, not a flux, so no orientation
/// enters it.
fn face_area(m: &Model, faces: &[(FaceId, f64)], meter: &mut Meter<'_>) -> Result<f64, OpError> {
    let mut area = 0.0;
    for &(id, _) in faces {
        meter.tick()?;
        let face = m.face(id)?;
        let surface = m.surface(face.surface())?;
        let grid = surface_grid(surface);
        for l in face.loops() {
            let pieces = m.loop_pieces(l)?;
            area += region_integral(&pieces, &grid, |u, v| {
                let e = surface.eval(u, v);
                e.du.cross(&e.dv).norm()
            });
        }
    }
    Ok(area)
}

#[cfg(test)]
mod tests {
    use arris_check::arris_topo::entity::Face;
    use arris_check::arris_topo::{AnyId, SurfaceId};

    use super::*;

    /// A face whose surface id was compacted away: `face_area` names the
    /// surface, not the face that holds it — the id that failed to
    /// resolve, itself.
    #[test]
    fn face_area_names_the_surface_when_it_does_not_resolve() {
        let mut m = Model::default();
        let gone = SurfaceId::new(0, 0);
        let f = m.raw().add_face(Face::new(gone, Vec::new(), 1e-7));
        match face_area(&m, &[(f, 1.0)], &mut Meter::default()).unwrap_err() {
            OpError::NotFound(id) => assert_eq!(id, AnyId::from(gone)),
            other => panic!("{other:?}"),
        }
    }

    /// A tilted cylinder far from the origin, less what lies above an
    /// oblique plane crossing every one of its rulings: `π r² t` exactly,
    /// `t` the axis's length below the plane. Its section is fitted on the
    /// cylinder side, so its faces close only to the fit; taken about the
    /// origin, 110 away, the volume was 1e-9 off (ADR-0024, the
    /// differential's case 172 of the fixed seed).
    #[test]
    fn a_body_far_from_the_origin_measures_to_its_closed_form() {
        use core::f64::consts::PI;

        use arris_check::arris_topo::arris_math::Axis;

        use crate::{Control, common, primitive_box, primitive_cylinder};

        let mut m = Model::default();
        let base = Point3::new(100.0, -80.0, 60.0);
        let axis = Axis::new(base, Vec3::new(0.3, 0.2, 1.0)).unwrap();
        let (radius, t) = (1.0, 3.0);
        let (cylinder, _) = primitive_cylinder(&mut m, axis, radius, 6.0, &Control::NONE).unwrap();
        let top = base.z + t * axis.direction.z;
        let (slab, _) = primitive_box(
            &mut m,
            base - Vec3::new(10.0, 10.0, 10.0),
            Point3::new(base.x + 10.0, base.y + 10.0, top),
            &Control::NONE,
        )
        .unwrap();
        let (below, _) = common(&mut m, cylinder, slab, &Control::NONE).unwrap();
        let p = mass_properties(&m, below, &Control::NONE).unwrap();
        let volume = PI * radius * radius * t;
        assert!(
            (p.volume - volume).abs() < 1e-11 * volume,
            "{} vs {volume}: {:e} relative",
            p.volume,
            (p.volume - volume) / volume
        );
    }
}
