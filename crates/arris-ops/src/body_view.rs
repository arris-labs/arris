//! A body read once, as an operation that edits it needs to: each face's
//! effective orientation and shell, every edge's uses and every vertex's
//! edges, all in the body's own order, with the questions asked of them —
//! a face's outward normal, whether an edge is convex, whether two faces
//! are tangent along it. Blend reads it; shell and offset read a body the
//! same way, beside `rebuild::rewrite`.

use std::collections::{BTreeMap, BTreeSet};

use arris_math::{Point2, Tolerance, Vec3};
use arris_topo::{Body, Curve2Id, EdgeId, FaceId, Model, Orientation, VertexId};

use crate::error::{Fault, OpError};

/// One use of an edge by a loop of one of the body's faces, addressed.
#[derive(Debug, Clone, Copy)]
pub(crate) struct UseAt {
    pub(crate) face: FaceId,
    pub(crate) loop_index: usize,
    pub(crate) coedge_index: usize,
    /// The stored orientation of the coedge.
    pub(crate) orientation: Orientation,
    pub(crate) pcurve: Curve2Id,
}

/// The body read once: each face's effective orientation and the shell
/// it belongs to, every edge's uses and every vertex's edges, all in the
/// body's own order.
pub(crate) struct BodyView {
    pub(crate) orientation: BTreeMap<FaceId, Orientation>,
    pub(crate) shell_of: BTreeMap<FaceId, usize>,
    pub(crate) faces: Vec<FaceId>,
    pub(crate) uses: BTreeMap<EdgeId, Vec<UseAt>>,
    pub(crate) vertex_edges: BTreeMap<VertexId, BTreeSet<EdgeId>>,
}

impl BodyView {
    /// Reads `body`: a face in two shells, or twice in one, is an
    /// internal fault.
    pub(crate) fn of(m: &Model, body: Body) -> Result<BodyView, OpError> {
        let mut view = BodyView {
            orientation: BTreeMap::new(),
            shell_of: BTreeMap::new(),
            faces: Vec::new(),
            uses: BTreeMap::new(),
            vertex_edges: BTreeMap::new(),
        };
        for (s, shell) in m.shells(body)?.into_iter().enumerate() {
            for face_use in m.shell(shell.id)?.faces() {
                let face = face_use.oriented_by(shell.orientation);
                if view.orientation.insert(face.id, face.orientation).is_some() {
                    return Err(OpError::Internal(Fault::Invariant {
                        what: "a face used once",
                    }));
                }
                view.shell_of.insert(face.id, s);
                view.faces.push(face.id);
                let entity = m.face(face.id)?;
                for (loop_index, l) in entity.loops().iter().enumerate() {
                    for (coedge_index, c) in l.coedges().iter().enumerate() {
                        view.uses.entry(c.edge()).or_default().push(UseAt {
                            face: face.id,
                            loop_index,
                            coedge_index,
                            orientation: c.orientation(),
                            pcurve: c.pcurve(),
                        });
                        let edge = m.edge(c.edge())?;
                        for v in [edge.start(), edge.end()] {
                            view.vertex_edges.entry(v).or_default().insert(c.edge());
                        }
                    }
                }
            }
        }
        Ok(view)
    }

    /// The face's outward normal at `uv`, the surface's composed with the
    /// body's use of the face.
    pub(crate) fn outward(&self, m: &Model, face: FaceId, uv: Point2) -> Result<Vec3, OpError> {
        let surface = m.surface(m.face(face)?.surface())?;
        let n = surface
            .normal(uv.x, uv.y)
            .ok_or(OpError::Internal(Fault::NoNormal { face }))?;
        Ok(n.into_inner() * self.orientation[&face].sign())
    }

    /// Whether `edge` is convex, read at its midpoint as a stripe or a ring
    /// reads it: the direction into its first face against the second
    /// face's outward normal. `None` for an edge with no curve or not used
    /// by exactly two faces.
    pub(crate) fn convex(&self, m: &Model, edge: EdgeId) -> Result<Option<bool>, OpError> {
        let Some((curve, range)) = m.edge(edge)?.curve() else {
            return Ok(None);
        };
        let Some(&[ua, ub]) = self.uses.get(&edge).map(Vec::as_slice) else {
            return Ok(None);
        };
        let mid = range.midpoint();
        let n1 = self.outward(m, ua.face, m.curve2(ua.pcurve)?.point(mid))?;
        let n2 = self.outward(m, ub.face, m.curve2(ub.pcurve)?.point(mid))?;
        let t1 = m.curve(curve)?.eval(mid).d1
            * self.orientation[&ua.face].compose(ua.orientation).sign();
        Ok(Some(n1.cross(&t1).dot(&n2) < 0.0))
    }

    /// Whether the two faces of `edge` are tangent at its parameter `t`
    /// for a blend of `size`, by [`tangent_normals`] at their own
    /// tolerance; `None` for an edge not used by exactly two faces.
    pub(crate) fn tangent_at(
        &self,
        m: &Model,
        edge: EdgeId,
        t: f64,
        size: f64,
        tol: Tolerance,
    ) -> Result<Option<bool>, OpError> {
        let Some(&[ua, ub]) = self.uses.get(&edge).map(Vec::as_slice) else {
            return Ok(None);
        };
        let n1 = self.outward(m, ua.face, m.curve2(ua.pcurve)?.point(t))?;
        let n2 = self.outward(m, ub.face, m.curve2(ub.pcurve)?.point(t))?;
        let tolerance = faces_tolerance(m, ua.face, ub.face)?;
        Ok(Some(tangent_normals(n1, n2, size, tolerance, tol)))
    }
}

/// The tolerance an operation is built to beside the faces `f1` and `f2`:
/// the model's default, or the larger of the faces' own where that is
/// wider.
pub(crate) fn faces_tolerance(m: &Model, f1: FaceId, f2: FaceId) -> Result<f64, OpError> {
    Ok(m.precision()
        .default_tolerance
        .max(m.face(f1)?.tolerance())
        .max(m.face(f2)?.tolerance()))
}

/// Whether two faces with outward normals `n1` and `n2` at a point of
/// their edge meet tangentially for a blend of `size` (ADR-0040): the
/// normals parallel within `tol.angular`, or the ball — a chamfer's
/// contacts — that touches one face moved by no more than `tolerance`, the
/// faces' own, to touch the other, `size` times the sine between them.
pub(crate) fn tangent_normals(
    n1: Vec3,
    n2: Vec3,
    size: f64,
    tolerance: f64,
    tol: Tolerance,
) -> bool {
    let sine = n1.cross(&n2).norm();
    sine <= tol.angular || size * sine <= tolerance
}

#[cfg(test)]
mod tests {
    use arris_geom::{Profile, ProfileLoop, ProfileSegment};
    use arris_math::{Axis, Control, Frame, Point3};

    use super::*;
    use crate::{cut, extrude, primitive_box, primitive_cylinder};

    fn tol(m: &Model) -> Tolerance {
        m.precision().tolerance()
    }

    /// An L in the xy plane, extruded 1 along z: a 2 × 2 square less its
    /// upper right 1 × 1 corner, whose inner corner is the one concave
    /// edge.
    fn l_prism(m: &mut Model) -> Body {
        let p = |u, v| Point2::new(u, v);
        let profile = Profile {
            plane: Frame::world(),
            outer: ProfileLoop::Path {
                start: p(0.0, 0.0),
                segments: vec![
                    ProfileSegment::LineTo(p(2.0, 0.0)),
                    ProfileSegment::LineTo(p(2.0, 1.0)),
                    ProfileSegment::LineTo(p(1.0, 1.0)),
                    ProfileSegment::LineTo(p(1.0, 2.0)),
                    ProfileSegment::LineTo(p(0.0, 2.0)),
                    ProfileSegment::LineTo(p(0.0, 0.0)),
                ],
            },
            holes: vec![],
        };
        extrude(m, &profile, Vec3::z(), 1.0, &Control::NONE)
            .unwrap()
            .0
    }

    fn convexity(m: &Model, body: Body) -> Vec<bool> {
        let view = BodyView::of(m, body).unwrap();
        (m.edges(body).unwrap().iter())
            .map(|e| view.convex(m, e.id).unwrap().unwrap())
            .collect()
    }

    #[test]
    fn every_edge_of_a_box_is_convex() {
        let mut m = Model::default();
        let (body, _) = primitive_box(
            &mut m,
            Point3::origin(),
            Point3::new(1.0, 2.0, 3.0),
            &Control::NONE,
        )
        .unwrap();
        let convex = convexity(&m, body);
        assert_eq!(convex.len(), 12);
        assert!(convex.iter().all(|&c| c));
    }

    #[test]
    fn the_inner_edge_of_an_l_prism_is_the_one_concave_edge() {
        let mut m = Model::default();
        let body = l_prism(&mut m);
        let convex = convexity(&m, body);
        assert_eq!(convex.len(), 18);
        assert_eq!(convex.iter().filter(|&&c| !c).count(), 1);
    }

    #[test]
    fn a_void_shells_faces_are_reversed_to_point_into_the_void() {
        let mut m = Model::default();
        let c = Control::NONE;
        let (big, _) =
            primitive_box(&mut m, Point3::origin(), Point3::new(10.0, 10.0, 10.0), &c).unwrap();
        let (small, _) = primitive_box(
            &mut m,
            Point3::new(3.0, 3.0, 3.0),
            Point3::new(6.0, 6.0, 6.0),
            &c,
        )
        .unwrap();
        let (body, _) = cut(&mut m, big, small, &c).unwrap();
        let view = BodyView::of(&m, body).unwrap();
        assert_eq!(m.shells(body).unwrap().len(), 2);
        let (outer_centre, void_centre) = (Point3::new(5.0, 5.0, 5.0), Point3::new(4.5, 4.5, 4.5));
        for &face in &view.faces {
            let entity = m.face(face).unwrap();
            let surface = m.surface(entity.surface()).unwrap();
            let at = surface.eval(0.0, 0.0).point;
            let n = view.outward(&m, face, Point2::origin()).unwrap();
            // Out of the material: away from the outer box's centre on
            // the outside, towards the void's centre on the inside.
            let (centre, sign) = match view.shell_of[&face] {
                0 => (outer_centre, 1.0),
                _ => (void_centre, -1.0),
            };
            assert!(
                sign * n.dot(&(at - centre)) > 0.0,
                "face {face:?} of shell {} points the wrong way",
                view.shell_of[&face]
            );
        }
        assert!(tol(&m).linear > 0.0);
    }

    #[test]
    fn edge_uses_come_in_the_bodys_face_order() {
        let mut m = Model::default();
        let body = l_prism(&mut m);
        let view = BodyView::of(&m, body).unwrap();
        let order: BTreeMap<FaceId, usize> = view
            .faces
            .iter()
            .enumerate()
            .map(|(i, &f)| (f, i))
            .collect();
        let shells: Vec<FaceId> = (m.shells(body).unwrap().into_iter())
            .flat_map(|s| {
                m.shell(s.id)
                    .unwrap()
                    .faces()
                    .iter()
                    .map(|f| f.oriented_by(s.orientation).id)
                    .collect::<Vec<_>>()
            })
            .collect();
        assert_eq!(view.faces, shells);
        for (edge, uses) in &view.uses {
            assert_eq!(uses.len(), 2, "{edge:?}");
            assert!(
                order[&uses[0].face] < order[&uses[1].face],
                "{edge:?} is used out of the body's order"
            );
        }
    }

    #[test]
    fn a_seams_two_uses_are_in_one_face_with_opposite_orientations() {
        let mut m = Model::default();
        let axis = Axis::new(Point3::origin(), Vec3::z()).unwrap();
        let (body, _) = primitive_cylinder(&mut m, axis, 1.0, 2.0, &Control::NONE).unwrap();
        let view = BodyView::of(&m, body).unwrap();
        let seams: Vec<&Vec<UseAt>> = view
            .uses
            .values()
            .filter(|u| u.len() == 2 && u[0].face == u[1].face)
            .collect();
        assert_eq!(seams.len(), 1);
        let [a, b] = [seams[0][0], seams[0][1]];
        assert_ne!(a.orientation, b.orientation);
        assert_ne!(a.pcurve, b.pcurve);
        assert_eq!(view.uses.len(), 3);
    }
}
