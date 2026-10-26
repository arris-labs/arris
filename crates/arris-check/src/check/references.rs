//! M1 and M2: every reference resolves and is indexed.

use std::collections::BTreeSet;

use arris_topo::{CoedgeRef, EntityId};

use super::{Checker, coedges};
use crate::violation::{Reference, Violation};

impl Checker<'_> {
    /// M1: every id an entity of the body references resolves.
    pub(crate) fn references(&mut self) {
        let mut unresolved: Vec<(EntityId, Reference)> = Vec::new();
        let body_id: EntityId = self.body.id.into();
        if let Ok(body) = self.model.body(self.body.id) {
            for s in body.shells() {
                if self.model.shell(s.id).is_err() {
                    unresolved.push((body_id, Reference::Entity(s.id.into())));
                }
            }
            for e in body.free_edges() {
                if self.model.edge(e.id).is_err() {
                    unresolved.push((body_id, Reference::Entity(e.id.into())));
                }
            }
            for &v in body.free_vertices() {
                if self.model.vertex(v).is_err() {
                    unresolved.push((body_id, Reference::Entity(v.into())));
                }
            }
        }
        for &shell_id in &self.closure.shells {
            let Ok(shell) = self.model.shell(shell_id) else {
                continue;
            };
            for f in shell.faces() {
                if self.model.face(f.id).is_err() {
                    unresolved.push((shell_id.into(), Reference::Entity(f.id.into())));
                }
            }
        }
        for &face_id in &self.closure.faces {
            let Ok(face) = self.model.face(face_id) else {
                continue;
            };
            if self.model.surface(face.surface()).is_err() {
                unresolved.push((face_id.into(), Reference::Geometry(face.surface().into())));
            }
            for (_, _, coedge) in coedges(face) {
                if self.model.edge(coedge.edge()).is_err() {
                    unresolved.push((face_id.into(), Reference::Entity(coedge.edge().into())));
                }
                if self.model.curve2(coedge.pcurve()).is_err() {
                    unresolved.push((face_id.into(), Reference::Geometry(coedge.pcurve().into())));
                }
            }
        }
        for &edge_id in &self.closure.edges {
            let Ok(edge) = self.model.edge(edge_id) else {
                continue;
            };
            for v in [edge.start(), edge.end()] {
                if self.model.vertex(v).is_err() {
                    unresolved.push((edge_id.into(), Reference::Entity(v.into())));
                }
            }
            if let Some((curve, _)) = edge.curve() {
                if self.model.curve(curve).is_err() {
                    unresolved.push((edge_id.into(), Reference::Geometry(curve.into())));
                }
            }
        }
        unresolved.sort();
        unresolved.dedup();
        for (from, to) in unresolved {
            self.push(Violation::Unresolved { from, to });
        }
    }

    /// M2: every reference a parent makes is in the adjacency index of the
    /// entity it names.
    pub(crate) fn indices(&mut self) {
        let mut missing: BTreeSet<(EntityId, EntityId)> = BTreeSet::new();
        for &shell_id in &self.closure.shells {
            let Ok(shell) = self.model.shell(shell_id) else {
                continue;
            };
            for f in shell.faces() {
                if let Ok(shells) = self.model.face_shells(f.id) {
                    if !shells.contains(&shell_id) {
                        missing.insert((f.id.into(), shell_id.into()));
                    }
                }
            }
        }
        for &face_id in &self.closure.faces {
            let Ok(face) = self.model.face(face_id) else {
                continue;
            };
            for (li, ci, coedge) in coedges(face) {
                if let Ok(uses) = self.model.edge_uses(coedge.edge()) {
                    let expected = CoedgeRef {
                        face: face_id,
                        loop_index: li,
                        coedge_index: ci,
                    };
                    if !uses.contains(&expected) {
                        missing.insert((coedge.edge().into(), face_id.into()));
                    }
                }
            }
        }
        for &edge_id in &self.closure.edges {
            let Ok(edge) = self.model.edge(edge_id) else {
                continue;
            };
            for v in [edge.start(), edge.end()] {
                if let Ok(edges) = self.model.vertex_edges(v) {
                    if !edges.contains(&edge_id) {
                        missing.insert((v.into(), edge_id.into()));
                    }
                }
            }
        }
        for (entity, referenced_by) in missing {
            self.push(Violation::NotIndexed {
                entity,
                referenced_by,
            });
        }
    }
}
