//! The flux of a field through a face (`docs/ARCHITECTURE.md` §The
//! checker): `∬ f(P, ∂P/∂u × ∂P/∂v) du dv` over the face's region in its
//! own (u, v), by Green's theorem through its loops.
//!
//! B2's enclosed volume, the lumps query's shell volumes and the mass
//! properties `arris-ops` measures are this one integral with different
//! fields, so the checker and a measurement cannot disagree about how a
//! region is integrated.

use core::fmt;

use arris_geom::integrate::{region_integral, surface_grid};
use arris_math::{Point3, Vec3};
use arris_topo::{FaceId, Model, NotFound};

use crate::domain::bounded_pieces;

/// Why a face's flux has no value. Never a guess: an integral over a
/// region the loops do not close is not taken.
#[derive(Debug, Clone, PartialEq)]
pub enum FluxError {
    /// The face, its surface, or an edge or pcurve of its loops does not
    /// resolve.
    NotFound(NotFound),
    /// A loop of the face has no coedge, or a coedge's range is not
    /// bounded and increasing: the checker's L1 and E1.
    Unintegrable {
        /// The face.
        face: FaceId,
    },
}

impl fmt::Display for FluxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FluxError::NotFound(e) => write!(f, "{e}"),
            FluxError::Unintegrable { face } => {
                write!(f, "{face}: a loop has no coedge or an unbounded range")
            }
        }
    }
}

impl std::error::Error for FluxError {}

impl From<NotFound> for FluxError {
    fn from(e: NotFound) -> Self {
        FluxError::NotFound(e)
    }
}

/// `∬ integrand(P, ∂P/∂u × ∂P/∂v) du dv` over `face`'s region in its own
/// (u, v): the flux through the face, with its surface's normal, of the
/// field whose divergence `integrand` is the matching term of.
///
/// Guarantees: each loop is integrated by `region_integral` on the
/// surface's `surface_grid` and the loops are summed in stored order, so a
/// counter-clockwise outer loop counts positively and a clockwise hole
/// subtracts itself. The sign is the face's; a caller integrating over a
/// shell multiplies by each face use's orientation. `integrand` receives
/// the surface point and the unnormalised normal, whose length is the area
/// element.
///
/// Errors: [`FluxError::NotFound`], [`FluxError::Unintegrable`].
///
/// ```
/// use arris_check::flux::face_flux;
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_math::Point3;
///
/// let mut m = Model::default();
/// let body = sample::cuboid(&mut m, Point3::origin(), Point3::new(4.0, 3.0, 2.0))?;
/// // `P / 3` has divergence one: its flux out of the box is the volume.
/// let mut volume = 0.0;
/// for f in m.faces(body)? {
///     volume += f.orientation.sign() * face_flux(&m, f.id, |p, n| p.coords.dot(&n) / 3.0)?;
/// }
/// assert!((volume - 24.0).abs() < 1e-12);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn face_flux(
    model: &Model,
    face: FaceId,
    integrand: impl Fn(Point3, Vec3) -> f64,
) -> Result<f64, FluxError> {
    let entity = model.face(face)?;
    let surface = model.surface(entity.surface())?;
    let grid = surface_grid(surface);
    let mut total = 0.0;
    for l in entity.loops() {
        let Some(pieces) = bounded_pieces(model, l)? else {
            return Err(FluxError::Unintegrable { face });
        };
        total += region_integral(&pieces, &grid, |u, v| {
            let e = surface.eval(u, v);
            integrand(e.point, e.du.cross(&e.dv))
        });
    }
    Ok(total)
}
