//! Join the 3D closure and the pcurves lifted onto each face.

use arris_geom::region2::Piece;
use arris_geom::{Curve2, Surface};
use arris_math::{Aabb, Interrupted, Interval, Meter, Point2, UnitVec2, shift_nearest_uv};

pub(super) fn bounded_loop(pieces: &[Piece<'_>]) -> bool {
    !pieces.is_empty()
        && pieces
            .iter()
            .all(|piece| piece.range.is_bounded() && piece.range.lo() < piece.range.hi())
}

pub(super) fn join_ends<'a>(
    pieces: &'a [Piece<'a>],
) -> impl Iterator<Item = (Point2, Point2)> + 'a {
    pieces
        .iter()
        .zip(pieces.iter().cycle().skip(1))
        .map(|(piece, next)| (piece.last(), next.first()))
        .filter(|(a, b)| a != b)
}

pub(super) fn bounds(
    surface: &Surface,
    loops: &[Vec<Piece<'_>>],
    boundary: Aabb,
    eps: f64,
    meter: &mut Meter<'_>,
) -> Result<Option<Aabb>, Interrupted> {
    let mut bounds = boundary;
    for piece in loops.iter().flatten() {
        // Seed from the same witnesses every time. Allowing one lifted
        // trim's outer box to seed another would accumulate approximation
        // error from piece to piece.
        let Some(b) = surface.tight_trim_bounds(piece.curve, piece.range, boundary, eps, meter)?
        else {
            return Ok(None);
        };
        bounds = bounds.union(b);
    }
    for (a, b) in loops.iter().flat_map(|pieces| join_ends(pieces)) {
        let d = b - a;
        let d = d - shift_nearest_uv(d, surface.period());
        let length = d.norm();
        if length == 0.0 {
            continue;
        }
        let curve = Curve2::Line {
            origin: a,
            direction: UnitVec2::new_unchecked(d / length),
        };
        let Ok(range) = Interval::new(0.0, length) else {
            return Ok(None);
        };
        let Some(b) = surface.tight_trim_bounds(&curve, range, boundary, eps, meter)? else {
            return Ok(None);
        };
        bounds = bounds.union(b);
    }
    Ok(Some(bounds))
}

/// A whole NURBS control hull bounds the trims when no pcurve requires
/// extrapolating the surface. For the cheap path a conservative proof is
/// enough: a NURBS pcurve's own control hull may extend past the surface.
pub(super) fn within_domain(surface: &Surface, loops: &[Vec<Piece<'_>>]) -> bool {
    let domain = surface.domain();
    let period = surface.period();
    loops.iter().flatten().all(|piece| {
        let contains = |uv: arris_math::Point2| {
            (0..2).all(|k| period[k].is_some() || domain[k].contains(uv[k]))
        };
        match piece.curve {
            Curve2::Line { .. } => {
                contains(piece.curve.point(piece.range.lo()))
                    && contains(piece.curve.point(piece.range.hi()))
            }
            Curve2::Circle { frame, radius } => {
                let radial = arris_math::Vec2::repeat(radius.abs());
                contains(frame.origin() - radial) && contains(frame.origin() + radial)
            }
            Curve2::Ellipse {
                frame,
                major_radius,
                minor_radius,
            } => {
                let radial = arris_math::Vec2::repeat(major_radius.abs().max(minor_radius.abs()));
                contains(frame.origin() - radial) && contains(frame.origin() + radial)
            }
            Curve2::Nurbs(curve) => {
                let range = curve.domain();
                (curve.period().is_some()
                    || (range.contains(piece.range.lo()) && range.contains(piece.range.hi())))
                    && curve.control_points().iter().copied().all(contains)
            }
        }
    })
}
