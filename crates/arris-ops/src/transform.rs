//! `transform`: a rigid motion of a body (`docs/ARCHITECTURE.md`
//! §Operations).

use arris_math::{Control, Isometry, Meter, Point3, RELATIVE_ROUNDING};
use arris_topo::builder::{Assembly, Builder, EdgeSpec, GeometryRemap, VertexSpec};
use arris_topo::entity::BodyKind;
use arris_topo::{Body, CurveId, EntityId, Model, Orientation, Provenance, Shape, SurfaceId};

use crate::error::OpError;
use crate::verify;

/// Moves every point, curve and surface [`Assembly::of_body`] asks for by
/// `motion`, adding the result curve or surface: `Curve::transformed` and
/// `Surface::transformed` carry the parametrisation along, so every
/// pcurve id is reused as it stands.
struct Move<'a>(&'a Isometry);

impl GeometryRemap for Move<'_> {
    fn point(&mut self, _model: &mut Model, p: Point3) -> Point3 {
        self.0.apply(p)
    }

    fn curve(&mut self, model: &mut Model, c: CurveId) -> CurveId {
        let moved = model
            .curve(c)
            .expect("of_body's closure names a live curve")
            .transformed(self.0);
        model.add_curve(moved)
    }

    fn surface(&mut self, model: &mut Model, s: SurfaceId) -> SurfaceId {
        let moved = model
            .surface(s)
            .expect("of_body's closure names a live surface")
            .transformed(self.0);
        model.add_surface(moved)
    }
}

/// Moves `body` rigidly by `motion`: every curve and surface appended
/// transformed (`Curve::transformed`, `Surface::transformed` carry the
/// parametrisation along, so every pcurve id is reused as it stands),
/// every vertex, edge, face, shell and the body itself appended new
/// through [`Builder::assemble`] in the body's own iteration order, each
/// recorded `Modified` one-to-one from the entity it moved. The body's
/// kind is kept. Every vertex's and edge's tolerance above the model's
/// default is grown by the rounding the motion leaves, [`RELATIVE_ROUNDING`] of the body's scale
/// before or after it, whichever is larger (`docs/DATA-MODEL.md`
/// §Tolerances): a tolerance measured to the gap it covers, as a read
/// one is, still covers it in the moved body.
///
/// Built over `assemble` (`assemble_sheet` for a `Sheet`), so it reaches
/// exactly as far as those do: a solid or a sheet moves whole, every shell
/// of it carried to a shell of the result in the body's stored order, a
/// sheet's faces keeping their fronts; any other body comes back as
/// [`OpError::Internal`] naming the builder's refusal.
///
/// Errors: [`OpError::InvalidInput`] when `body` fails the checker (debug
/// builds, and release with the `paranoid` feature); [`OpError::NotFound`]
/// when it does not resolve. The model is untouched on error.
///
/// ```
/// use arris_ops::{primitive_cylinder, transform};
/// use arris_topo::{Model, Shape};
/// use arris_math::{Axis, Isometry, Point3, Vec3};
///
/// let mut m = Model::default();
/// let (body, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 4.0, 12.0, &arris_ops::Control::NONE).unwrap();
/// let motion = Isometry::from_translation(Vec3::new(1.0, 2.0, 3.0));
/// let (moved, provenance) = transform(&mut m, body, &motion, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(moved).unwrap().len(), 3);
/// assert_eq!(provenance.modified_from(Shape::from(body)).len(), 1);
/// ```
pub fn transform(
    m: &mut Model,
    body: Body,
    motion: &Isometry,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    copy_body(m, body, &mut Move(motion), control)
}

/// Copies `body` through [`Assembly::of_body`] with `remap` and
/// [`Builder::assemble`], every entity `Modified` one-to-one from the one
/// it copies: what `transform` and `mirror` are each a remap of. Each
/// copied vertex and edge whose tolerance is above the model's default
/// carries it grown by [`rounding`].
pub(crate) fn copy_body(
    m: &mut Model,
    body: Body,
    remap: &mut impl GeometryRemap,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    crate::verify_input(m, body)?;
    let mut meter = Meter::new(control);
    for _ in m.faces(body)? {
        meter.tick()?;
    }
    let entity = m.body(body.id)?.clone();
    let tolerance = m.precision().default_tolerance;
    let old_points = (m.vertices(body)?.into_iter())
        .map(|v| Ok(m.vertex(v.id)?.point()))
        .collect::<Result<Vec<Point3>, OpError>>()?;

    m.transaction(|m| {
        let (mut assembly, index) = Assembly::of_body(m, body, remap)?;
        let grown = rounding(&old_points, &assembly);
        // A tolerance at the floor is not measured to a gap: the rounding
        // it covers is orders below it (`rounding`).
        let floor = tolerance;
        for spec in &mut assembly.vertices {
            if let VertexSpec::New { tolerance, .. } = spec
                && *tolerance > floor
            {
                *tolerance += grown;
            }
        }
        for spec in &mut assembly.edges {
            if let EdgeSpec::New { tolerance, .. } = spec
                && *tolerance > floor
            {
                *tolerance += grown;
            }
        }
        let (b, slots) = match entity.kind() {
            BodyKind::Sheet => Builder::assemble_sheet(m, tolerance, assembly)?,
            _ => Builder::assemble(m, tolerance, assembly)?,
        };
        let built = b.finish(m, entity.kind())?;

        fn forward(id: impl Into<EntityId>) -> Shape {
            Shape::new(id, Orientation::Forward)
        }
        let mut provenance = Provenance::new();
        for (&old, &i) in &index.vertices {
            provenance.add_modified(forward(old), forward(built.vertices[&slots.vertices[i]]));
        }
        for (&old, &i) in &index.edges {
            provenance.add_modified(forward(old), forward(built.edges[&slots.edges[i]]));
        }
        for (&old, &(si, fi)) in &index.faces {
            provenance.add_modified(forward(old), forward(built.faces[&slots.faces[si][fi]]));
        }
        for (&old, &si) in &index.shells {
            provenance.add_modified(forward(old), forward(built.shells[si]));
        }
        provenance.add_modified(forward(body.id), forward(built.body.id));

        verify(m, built.body)?;
        Ok((built.body, provenance))
    })
}

/// The growth a copy gives every vertex's and edge's tolerance above the
/// model's default:
/// [`RELATIVE_ROUNDING`] of the farthest from the origin the body's
/// vertices reach before the copy (`old`) or after it (`assembly`'s points). A copy
/// evaluates every point, curve and surface again at the copy's
/// coordinates, so every gap a tolerance covers — a pcurve's image from
/// its vertex (V3), from its curve (E4), a curve's end from its vertex
/// (V2) — moves by rounding at that scale. A tolerance measured to its gap
/// with only the original's rounding beside it (a read entity's, ADR-0025
/// §4) would otherwise fall short of it by a few ulps once the body is
/// moved away from the origin. One at the default is not measured to its
/// gap, and the rounding is orders below it, so it is kept as it stands:
/// a copy of an exact body carries its tolerances bit for bit. The same
/// growth on vertices and edges keeps the ordering `vertex ≥ edge`.
fn rounding(old: &[Point3], assembly: &Assembly) -> f64 {
    let new = assembly.vertices.iter().filter_map(|v| match v {
        VertexSpec::New { point, .. } => Some(*point),
        VertexSpec::Keep(_) => None,
    });
    let scale = (old.iter().copied().chain(new))
        .map(|p| p.coords.norm())
        .fold(0.0, f64::max);
    RELATIVE_ROUNDING * scale
}
