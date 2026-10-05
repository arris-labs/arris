//! `mirror`: the reflection of a body in a plane (`docs/ARCHITECTURE.md`
//! §Operations, ADR-0031).

use std::collections::BTreeMap;

use arris_geom::ParamMap;
use arris_math::{Control, Point3, Reflection};
use arris_topo::builder::{FaceRemap, GeometryRemap};
use arris_topo::{Body, Curve2Id, CurveId, Model, Provenance, SurfaceId};

use crate::error::OpError;
use crate::transform::copy_body;

/// Reflects every point, curve and surface [`Assembly::of_body`] asks for,
/// and answers for the faces and pcurves what ADR-0031 §4 says: where a
/// surface's parameters are reflected its pcurves are reflected with them
/// and its loops are stored walked the other way, and where its normal
/// turns against the image of the original's the face's use is toggled.
///
/// [`Assembly::of_body`]: arris_topo::builder::Assembly::of_body
struct Reflect<'a> {
    plane: &'a Reflection,
    maps: BTreeMap<SurfaceId, ParamMap>,
}

impl GeometryRemap for Reflect<'_> {
    fn point(&mut self, _model: &mut Model, p: Point3) -> Point3 {
        self.plane.apply(p)
    }

    fn curve(&mut self, model: &mut Model, c: CurveId) -> CurveId {
        let image = model
            .curve(c)
            .expect("of_body's closure names a live curve")
            .mirrored(self.plane);
        model.add_curve(image)
    }

    fn surface(&mut self, model: &mut Model, s: SurfaceId) -> SurfaceId {
        let (image, map) = model
            .surface(s)
            .expect("of_body's closure names a live surface")
            .mirrored(self.plane);
        self.maps.insert(s, map);
        model.add_surface(image)
    }

    fn pcurve(&mut self, model: &mut Model, p: Curve2Id, surface: SurfaceId) -> Curve2Id {
        match self.maps[&surface] {
            ParamMap::Identity => p,
            ParamMap::ReflectU => {
                let image = model
                    .curve2(p)
                    .expect("of_body's closure names a live pcurve")
                    .reflected();
                model.add_curve2(image)
            }
        }
    }

    fn face(&mut self, _model: &mut Model, surface: SurfaceId) -> FaceRemap {
        match self.maps[&surface] {
            ParamMap::Identity => FaceRemap {
                toggle_use: true,
                reverse_loops: false,
            },
            ParamMap::ReflectU => FaceRemap {
                toggle_use: false,
                reverse_loops: true,
            },
        }
    }
}

/// Reflects `body` in `plane`: its mirror image, a new solid with its
/// material inside and every orientation made to say so. Every curve and
/// surface is appended as its `mirrored` image (frames stay right-handed:
/// a quadric's `u` is reflected, a plane's and a NURBS surface's normal
/// turns, a curve keeps its parameter), a pcurve is reused where the
/// surface's parameters are unchanged and reflected where they are not,
/// and every vertex, edge, face, shell and the body itself is appended
/// new through [`Builder::assemble`] in the body's own iteration order,
/// each recorded `Modified` one-to-one from the entity it mirrors — no
/// entity is shared with the input, even with the plane through the body:
/// a mirror copies, it does not cut. The body's kind is kept.
///
/// Built over the seam [`transform`](crate::transform) is, so it reaches as
/// far as that does: a body of shells of faces that share nothing; one
/// that is not comes back as [`OpError::Internal`] naming the builder's
/// refusal.
///
/// Errors: [`OpError::InvalidInput`] when `body` fails the checker (debug
/// builds, and release with the `paranoid` feature); [`OpError::NotFound`]
/// when it does not resolve; [`OpError::Interrupted`] when `control` stops
/// it (one step per face). The model is untouched on error.
///
/// [`Builder::assemble`]: arris_topo::builder::Builder::assemble
///
/// ```
/// use arris_ops::{mirror, primitive_cylinder};
/// use arris_topo::{Model, Shape};
/// use arris_math::{Axis, Point3, Reflection, Vec3};
///
/// let mut m = Model::default();
/// let (body, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 4.0, 12.0, &arris_ops::Control::NONE).unwrap();
/// let plane = Reflection::new(Point3::new(10.0, 0.0, 0.0), Vec3::x()).unwrap();
/// let (image, provenance) = mirror(&mut m, body, &plane, &arris_ops::Control::NONE).unwrap();
/// assert_eq!(m.faces(image).unwrap().len(), 3);
/// assert_eq!(provenance.modified_from(Shape::from(body)).len(), 1);
/// ```
pub fn mirror(
    m: &mut Model,
    body: Body,
    plane: &Reflection,
    control: &Control<'_>,
) -> Result<(Body, Provenance), OpError> {
    copy_body(
        m,
        body,
        &mut Reflect {
            plane,
            maps: BTreeMap::new(),
        },
        control,
    )
}
