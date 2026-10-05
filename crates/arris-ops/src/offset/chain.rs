//! The dragged chain: the move set closed over the edges where two faces
//! meet tangentially, and each such edge carried along the faces' shared
//! normal — the one curve both offsets still share, where their section
//! is only a touch.

use std::collections::{BTreeMap, BTreeSet};

use arris_geom::{Curve, Surface};
use arris_math::{Frame, Point3, Tolerance, Vec3};
use arris_topo::{EdgeId, FaceId, Model};

use super::meet::Constraint;
use crate::body_view::BodyView;
use crate::error::{OffsetReason, OpError, Reason};
use crate::rebuild::forward;

/// The faces an offset moves and the edges it carries.
pub(super) struct Chain {
    /// The chosen faces and every face reached from one of them across a
    /// tangent edge.
    pub(super) moved: BTreeSet<FaceId>,
    /// The faces in `moved` that were not chosen: dragged by a tangent
    /// neighbour.
    pub(super) dragged: BTreeSet<FaceId>,
    /// Every edge between two distinct faces tangent along it; both its
    /// faces are in `moved` or neither is.
    pub(super) tangent: BTreeSet<EdgeId>,
}

/// The chosen faces closed over tangent edges: a face meeting a moved face
/// tangentially along an edge moves with it, and so on along the chain
/// (ADR-0048 §6). Tangency is read at the edge's midpoint as the blend
/// reads it, for a ball of the offset's distance: the offsets of two faces
/// whose normals differ there by no more than that ball can tell apart are
/// one sheet.
pub(super) fn chain(
    m: &Model,
    view: &BodyView,
    chosen: &BTreeSet<FaceId>,
    distance: f64,
    tol: Tolerance,
) -> Result<Chain, OpError> {
    let mut tangent = BTreeSet::new();
    let mut across: BTreeMap<FaceId, Vec<FaceId>> = BTreeMap::new();
    for (&e, uses) in &view.uses {
        let [a, b] = uses.as_slice() else { continue };
        if a.face == b.face {
            continue;
        }
        let Some((_, range)) = m.edge(e)?.curve() else {
            continue;
        };
        if view.tangent_at(m, e, range.midpoint(), distance.abs(), tol)? == Some(true) {
            tangent.insert(e);
            across.entry(a.face).or_default().push(b.face);
            across.entry(b.face).or_default().push(a.face);
        }
    }
    let mut moved = chosen.clone();
    let mut queue: Vec<FaceId> = chosen.iter().copied().collect();
    while let Some(f) = queue.pop() {
        for &g in across.get(&f).into_iter().flatten() {
            if moved.insert(g) {
                queue.push(g);
            }
        }
    }
    let dragged = moved.difference(chosen).copied().collect();
    Ok(Chain {
        moved,
        dragged,
        tangent,
    })
}

/// Each tangent edge between two moved faces carried along their shared
/// outward normal by `distance`: a line whose normal is the same along it
/// shifted, a circle whose normal keeps one angle to its plane and its
/// radius re-radiused and moved along its axis — the curve both offsets
/// pass through, since each point of it moves by `distance` along the
/// normal both faces share there. Any other curve, or a line or circle
/// whose normal turns along it, is [`OffsetReason::NoExactOffset`] naming
/// the edge; a circle driven to or through zero radius
/// [`OffsetReason::Vanishes`].
pub(super) fn carried(
    m: &Model,
    view: &BodyView,
    chain: &Chain,
    distance: f64,
    tol: Tolerance,
) -> Result<BTreeMap<EdgeId, Curve>, OpError> {
    let mut out = BTreeMap::new();
    for &e in &chain.tangent {
        let Some(&[a, _]) = view.uses.get(&e).map(Vec::as_slice) else {
            continue;
        };
        if !chain.moved.contains(&a.face) {
            continue;
        }
        let edge = m.edge(e)?;
        let Some((curve_id, range)) = edge.curve() else {
            continue;
        };
        let refuse = |reason| OpError::Degenerate {
            entities: vec![forward(e)],
            reason: Reason::Offset(reason),
        };
        let pcurve = m.curve2(a.pcurve)?;
        let normals = [range.lo(), range.midpoint(), range.hi()]
            .into_iter()
            .map(|t| Ok((t, view.outward(m, a.face, pcurve.point(t))?)))
            .collect::<Result<Vec<(f64, Vec3)>, OpError>>()?;
        let curve = m.curve(curve_id)?;
        let new = match curve {
            Curve::Line { origin, direction } => {
                let n = normals[0].1;
                if normals
                    .iter()
                    .any(|(_, k)| k.cross(&n).norm() > tol.angular)
                {
                    return Err(refuse(OffsetReason::NoExactOffset));
                }
                Curve::Line {
                    origin: origin + n * distance,
                    direction: *direction,
                }
            }
            Curve::Circle { frame, radius } => {
                let (c, z) = (frame.origin(), frame.z().into_inner());
                // The normal's share along the radius and along the axis,
                // the same at every sample on a circle of a surface of
                // revolution about its axis.
                let split = |&(t, n): &(f64, Vec3)| {
                    let rho = (curve.point(t) - c) / *radius;
                    (n.dot(&rho), n.dot(&z))
                };
                let (radial, axial) = split(&normals[0]);
                if normals.iter().map(split).any(|(r, x)| {
                    (r - radial).abs() > tol.angular || (x - axial).abs() > tol.angular
                }) {
                    return Err(refuse(OffsetReason::NoExactOffset));
                }
                let radius = radius + distance * radial;
                if radius <= edge.tolerance().max(tol.linear) {
                    return Err(refuse(OffsetReason::Vanishes));
                }
                let origin = c + z * (distance * axial);
                let frame = Frame::from_orthonormal(
                    origin,
                    frame.x().into_inner(),
                    frame.y().into_inner(),
                    z,
                )
                .map_err(|_| refuse(OffsetReason::NoExactOffset))?;
                Curve::Circle { frame, radius }
            }
            Curve::Ellipse { .. } | Curve::Nurbs(_) => {
                return Err(refuse(OffsetReason::NoExactOffset));
            }
        };
        out.insert(e, new);
    }
    Ok(out)
}

/// The surfaces whose section is `curve`, as a vertex's constraints: a
/// line the two planes through it square to each other, a circle its
/// plane and the cylinder about its axis through it. `None` for another
/// kind or a frame that cannot be built.
pub(super) fn constraints_of(curve: &Curve, face: FaceId) -> Option<[Constraint; 2]> {
    let plane = |origin: Point3, n: Vec3| -> Option<Constraint> {
        Some(Constraint {
            surface: Surface::Plane {
                frame: Frame::from_z(origin, n).ok()?,
            },
            face,
        })
    };
    match curve {
        Curve::Line { origin, direction } => {
            let across = Frame::from_z(*origin, direction.into_inner()).ok()?;
            Some([
                plane(*origin, across.x().into_inner())?,
                plane(*origin, across.y().into_inner())?,
            ])
        }
        Curve::Circle { frame, radius } => Some([
            plane(frame.origin(), frame.z().into_inner())?,
            Constraint {
                surface: Surface::Cylinder {
                    frame: *frame,
                    radius: *radius,
                },
                face,
            },
        ]),
        Curve::Ellipse { .. } | Curve::Nurbs(_) => None,
    }
}
