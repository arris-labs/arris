//! The last phases: the body built, what healing grew named, the body
//! checked, the provenance recorded.

use arris_check::{Level, check};
use arris_topo::builder::{Assembly, AssemblySlots, Builder, Built};
use arris_topo::entity::BodyKind;
use arris_topo::provenance::{FileEntity, Role};
use arris_topo::{Orientation, Provenance, Shape};

use super::Build;
use super::assemble::Tolerances;
use crate::step::reader::topology::Solid;
use crate::step::reader::{Grown, Healing, Refusal};

impl Build<'_, '_> {
    /// The body of the assembly, its provenance, and what healing grew.
    /// Errors: the builder's refusal, or the checker's on the body.
    pub(super) fn finish(
        &mut self,
        assembly: Assembly,
        tolerances: Tolerances,
    ) -> Result<Solid, Refusal> {
        let solid = self.solid;
        let topology = |e: arris_topo::builder::BuildError| Refusal::Topology {
            entity: solid,
            what: e.to_string(),
        };
        let (builder, slots) = match self.file.kind {
            BodyKind::Sheet => Builder::assemble_sheet(self.model, self.tolerance, assembly),
            _ => Builder::assemble(self.model, self.tolerance, assembly),
        }
        .map_err(topology)?;
        let built = builder
            .finish(self.model, self.file.kind)
            .map_err(topology)?;
        let grown = self.grown(&tolerances, &slots, &built);
        self.checked(&built)?;
        Ok(Solid {
            body: built.body,
            provenance: self.provenance(&slots, &built),
            healing: Healing {
                grown,
                turned: (self.turn_back.iter())
                    .filter_map(|&(si, fi)| {
                        let slot = slots.faces.get(si)?.get(fi)?;
                        built.faces.get(slot).copied()
                    })
                    .collect(),
            },
        })
    }

    /// Every vertex and edge grown past the read cap, named (ADR-0058
    /// §3): only a healing read gets here with one.
    fn grown(&self, tolerances: &Tolerances, slots: &AssemblySlots, built: &Built) -> Vec<Grown> {
        let mut grown = Vec::new();
        for ((&tolerance, slot), &v) in tolerances
            .vertices
            .iter()
            .zip(&slots.vertices)
            .zip(&self.vertex_ids)
        {
            if let (true, Some(&id)) = (tolerance > self.read_cap, built.vertices.get(slot)) {
                grown.push(Grown {
                    entity: id.into(),
                    file: v,
                    gap: tolerance,
                });
            }
        }
        let edge_entities = self.edge_ids.iter().chain(&self.rebuilt_faces);
        for ((&tolerance, slot), &e) in tolerances.edges.iter().zip(&slots.edges).zip(edge_entities)
        {
            if let (true, Some(&id)) = (tolerance > self.read_cap, built.edges.get(slot)) {
                grown.push(Grown {
                    entity: id.into(),
                    file: e,
                    gap: tolerance,
                });
            }
        }
        grown.sort_by(|a, b| b.gap.total_cmp(&a.gap));
        grown
    }

    /// The body held to the checker. Errors: what the checker found,
    /// blamed on the file's orientation where faces were turned back.
    fn checked(&self, built: &Built) -> Result<(), Refusal> {
        let report = check(self.model, built.body, Level::Fast);
        if !report.is_ok() && !self.turn_back.is_empty() {
            // A face turned back that the checker then rejects was not
            // the file's face reversed but some other inconsistency —
            // a face whose front and bounds are both turned, which the
            // walk turned back leaves bounding no region: the file's
            // orientation, refused as such.
            let names: Vec<String> = (self.turn_back.iter())
                .filter_map(|&(si, fi)| self.file.shells.get(si)?.faces.get(fi))
                .map(|f| format!("#{}", f.id))
                .collect();
            return Err(Refusal::Topology {
                entity: self.solid,
                what: format!(
                    "{} walk every shared edge the way the neighbour across it does, and turned back fail the checker:\n{report}",
                    names.join(", ")
                ),
            });
        }
        if !report.is_ok() {
            return Err(Refusal::Invalid {
                entity: self.solid,
                report: Box::new(report),
            });
        }
        Ok(())
    }

    /// Each file entity as the entities it generated.
    fn provenance(&self, slots: &AssemblySlots, built: &Built) -> Provenance {
        let instance = self.instance;
        let file_entity = |id: u64| Role::File(FileEntity { id, instance });
        let mut provenance = Provenance::new();
        for (&v, slot) in self.vertex_ids.iter().zip(&slots.vertices) {
            if let Some(&id) = built.vertices.get(slot) {
                provenance.add_generated(file_entity(v), Shape::new(id, Orientation::Forward));
            }
        }
        // A rebuilt edge is generated from its face's entity.
        let edge_entities = self.edge_ids.iter().chain(&self.rebuilt_faces);
        for (&e, slot) in edge_entities.zip(&slots.edges) {
            if let Some(&id) = built.edges.get(slot) {
                provenance.add_generated(file_entity(e), Shape::new(id, Orientation::Forward));
            }
        }
        for (shell, shell_slots) in self.file.shells.iter().zip(&slots.faces) {
            for (face, slot) in shell.faces.iter().zip(shell_slots) {
                if let Some(&id) = built.faces.get(slot) {
                    provenance
                        .add_generated(file_entity(face.id), Shape::new(id, Orientation::Forward));
                }
            }
        }
        for (shell, &id) in self.file.shells.iter().zip(&built.shells) {
            provenance.add_generated(file_entity(shell.id), Shape::new(id, Orientation::Forward));
        }
        provenance.add_generated(file_entity(self.solid), built.body);
        provenance
    }
}
