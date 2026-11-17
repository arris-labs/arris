//! Two rims' blends that crowd one face: the contacts of two closed edges' rings on a shared face
//! cross or swap order, so the face has no band left between them.

use std::collections::BTreeMap;

use arris_geom::{Curve, Surface};
use arris_math::{Point3, Vec3};
use arris_topo::FaceId;

use super::build::Env;
use super::degenerate;
use super::ring::{Ring, RingEnds};
use crate::error::{BlendReason, OpError, Reason};
use crate::rebuild::forward;

/// What a ring's blend takes from one face, along that face's meridian: the
/// span from the edge to the contact, about the ring's axis.
struct Reach {
    ring: usize,
    axis_origin: Point3,
    axis: Vec3,
    /// The span's ends, in the face's meridian coordinate: the radius on a
    /// plane, the height on a cylinder or a cone.
    span: [f64; 2],
}

/// Refuses two closed edges' blends that take overlapping bands of one
/// face: the contacts cross, swap order or meet, so no face is left between
/// them. [`BlendReason::TooLarge`] names both edges and the face. Each
/// ring is read on the face's meridian — the radius from the ring's axis on
/// a plane, the height along it on a cylinder or a cone — from its edge to
/// its contact at the edge's start; a pair on different axes, a ring run
/// over an edge, an open arc or a face of another kind is not compared
/// here.
pub(super) fn crowded_faces(env: &Env<'_>, rings: &[Ring]) -> Result<(), OpError> {
    let Env { m, tol, .. } = *env;
    let mut on_face: BTreeMap<FaceId, Vec<Reach>> = BTreeMap::new();
    for (i, ring) in rings.iter().enumerate() {
        if ring.rolled.is_some() || !matches!(ring.ends, RingEnds::Seam(_)) {
            continue;
        }
        let edge = m.edge(ring.edge)?;
        let Some((curve, range)) = edge.curve() else {
            continue;
        };
        let Curve::Circle { frame, .. } = *m.curve(curve)? else {
            continue;
        };
        let (axis_origin, axis) = (frame.origin(), frame.z().into_inner());
        let at_edge = m.curve(curve)?.point(range.lo());
        for contact in &ring.contacts {
            let surface = m.surface(m.face(contact.face)?.surface())?;
            let coordinate = |p: Point3| match surface {
                Surface::Plane { .. } => {
                    let w = p - axis_origin;
                    Some((w - axis * w.dot(&axis)).norm())
                }
                Surface::Cylinder { .. } | Surface::Cone { .. } => {
                    Some((p - axis_origin).dot(&axis))
                }
                Surface::Sphere { .. }
                | Surface::Torus { .. }
                | Surface::EllipticCylinder { .. }
                | Surface::Nurbs(_) => None,
            };
            let (Some(from), Some(to)) = (coordinate(at_edge), coordinate(contact.points[0]))
            else {
                continue;
            };
            on_face.entry(contact.face).or_default().push(Reach {
                ring: i,
                axis_origin,
                axis,
                span: [from.min(to), from.max(to)],
            });
        }
    }
    for (&face, reaches) in &on_face {
        for (k, a) in reaches.iter().enumerate() {
            for b in &reaches[k + 1..] {
                let w = b.axis_origin - a.axis_origin;
                let coaxial = a.axis.cross(&b.axis).norm() <= tol.angular
                    && w.cross(&a.axis).norm() <= tol.linear;
                if a.ring == b.ring || !coaxial {
                    continue;
                }
                // The coordinate's zero is the ring's own origin on a
                // cylinder or a cone: shift the second by the offset along
                // the axis.
                let shift = match m.surface(m.face(face)?.surface())? {
                    Surface::Plane { .. } => 0.0,
                    _ => w.dot(&a.axis),
                };
                let (lo, hi) = (
                    a.span[0].max(b.span[0] + shift),
                    a.span[1].min(b.span[1] + shift),
                );
                if hi - lo >= -tol.linear {
                    let edges = [rings[a.ring].edge, rings[b.ring].edge];
                    return Err(degenerate(
                        vec![forward(edges[0]), forward(edges[1]), forward(face)],
                        Reason::Blend(BlendReason::TooLarge),
                    ));
                }
            }
        }
    }
    Ok(())
}
