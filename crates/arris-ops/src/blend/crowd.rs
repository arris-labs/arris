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
    /// The span's ends in space: the edge's start and the contact there. The
    /// face's meridian coordinate (the radius on a plane, the height on a
    /// cylinder or a cone) is read from them against one ring's axis.
    points: [Point3; 2],
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
            let kind = m.surface(m.face(contact.face)?.surface())?;
            if matches!(
                kind,
                Surface::Sphere { .. }
                    | Surface::Torus { .. }
                    | Surface::EllipticCylinder { .. }
                    | Surface::Nurbs(_)
            ) {
                continue;
            }
            on_face.entry(contact.face).or_default().push(Reach {
                ring: i,
                axis_origin,
                axis,
                points: [at_edge, contact.points[0]],
            });
        }
    }
    for (&face, reaches) in &on_face {
        let surface = m.surface(m.face(face)?.surface())?;
        // The face's meridian coordinate of a point, against `a`'s axis and
        // origin whichever ring the point is of: the two rings' own frames
        // may point opposite ways, so each is never read by its own.
        let coordinate = |a: &Reach, p: Point3| match surface {
            Surface::Plane { .. } => {
                let w = p - a.axis_origin;
                Some((w - a.axis * w.dot(&a.axis)).norm())
            }
            Surface::Cylinder { .. } | Surface::Cone { .. } => {
                Some((p - a.axis_origin).dot(&a.axis))
            }
            Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Nurbs(_) => None,
        };
        for (k, a) in reaches.iter().enumerate() {
            for b in &reaches[k + 1..] {
                let w = b.axis_origin - a.axis_origin;
                let coaxial = a.axis.cross(&b.axis).norm() <= tol.angular
                    && w.cross(&a.axis).norm() <= tol.linear;
                if a.ring == b.ring || !coaxial {
                    continue;
                }
                let span = |r: &Reach| {
                    let (from, to) = (coordinate(a, r.points[0])?, coordinate(a, r.points[1])?);
                    Some([from.min(to), from.max(to)])
                };
                let (Some(sa), Some(sb)) = (span(a), span(b)) else {
                    continue;
                };
                let (lo, hi) = (sa[0].max(sb[0]), sa[1].min(sb[1]));
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
