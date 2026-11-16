//! Operands that share entities. Entities are immutable and bodies share
//! them by id (`docs/ARCHITECTURE.md` §Operations): a body split and fused
//! back keeps the faces the split did not touch, so it shares them, their
//! edges and their vertices with the body it was split from. The pave
//! model keys every edge, face and vertex by its id and reads each as one
//! operand's, so an operand that shares one with an earlier operand is
//! copied first, entity for entity on the same geometry, and the boolean
//! runs on the copy. Its record is then written back against the
//! operands' own ids ([`Apart::restore`]).

use std::collections::{BTreeMap, BTreeSet};

use arris_math::Control;
use arris_topo::builder::KeepGeometry;
use arris_topo::provenance::Origin;
use arris_topo::{Body, EntityId, Model, Provenance, Shape};

use crate::error::OpError;
use crate::transform::copy_body;

/// The operands a boolean decomposes, no two sharing an entity, and how
/// the copies among them map back.
pub(super) struct Apart {
    /// The operands, each the caller's or its copy, in the caller's order.
    pub(super) operands: Vec<Body>,
    /// Every entity of a copy, to the operand's entity it copies.
    original: BTreeMap<EntityId, EntityId>,
    /// The entities an operand shares with an earlier one: one input
    /// entity, read by the boolean once per operand.
    shared: BTreeSet<EntityId>,
}

/// Every vertex, edge, face, shell and the body of `body`.
fn entities(m: &Model, body: Body) -> Result<BTreeSet<EntityId>, OpError> {
    let c = m.closure(body)?;
    let mut set: BTreeSet<EntityId> = BTreeSet::new();
    set.extend(c.vertices.iter().map(|&v| EntityId::from(v)));
    set.extend(c.edges.iter().map(|&e| EntityId::from(e)));
    set.extend(c.faces.iter().map(|&f| EntityId::from(f)));
    set.extend(c.shells.iter().map(|&s| EntityId::from(s)));
    set.insert(body.id.into());
    Ok(set)
}

/// `operands` as the pave model reads them: an operand that shares an
/// entity with an earlier one is replaced by its copy
/// (`transform::copy_body` on the same geometry); the others are given
/// back as they are, so operands that share nothing cost one closure each.
/// The copies are appended to `m`: the caller runs this inside the
/// boolean's transaction.
pub(super) fn apart(
    m: &mut Model,
    operands: &[Body],
    control: &Control<'_>,
) -> Result<Apart, OpError> {
    let mut seen: BTreeSet<EntityId> = BTreeSet::new();
    let mut out = Apart {
        operands: Vec::with_capacity(operands.len()),
        original: BTreeMap::new(),
        shared: BTreeSet::new(),
    };
    for &body in operands {
        let own = entities(m, body)?;
        let common: Vec<EntityId> = own.intersection(&seen).copied().collect();
        if common.is_empty() {
            out.operands.push(body);
        } else {
            let (copy, record) = copy_body(m, body, &mut KeepGeometry, control)?;
            for origin in record.origins_recorded() {
                let Origin::Entity(old) = origin else {
                    continue;
                };
                for new in record.modified_from(origin) {
                    out.original.insert(new.id, old.id);
                }
            }
            out.shared.extend(common);
            out.operands.push(Body {
                id: copy.id,
                orientation: body.orientation,
            });
        }
        seen.extend(own);
    }
    Ok(out)
}

impl Apart {
    /// `true` when no operand was copied: the boolean's record is already
    /// written against the operands.
    fn untouched(&self) -> bool {
        self.original.is_empty()
    }

    /// The operand entity `s` stands for: a copy's original, else itself.
    fn back(&self, s: Shape) -> Shape {
        match self.original.get(&s.id) {
            Some(&id) => Shape::new(id, s.orientation),
            None => s,
        }
    }

    /// `e` naming the caller's operands: an entity of a copy in a refusal
    /// is its original, since the copy is rolled back with the boolean.
    /// A kernel fault keeps the ids it was raised on.
    pub(super) fn error(&self, e: OpError) -> OpError {
        match e {
            OpError::Unsupported { a, b } => OpError::Unsupported {
                a: (a.0, self.back(a.1)),
                b: (b.0, self.back(b.1)),
            },
            OpError::Degenerate { entities, reason } => OpError::Degenerate {
                entities: entities.into_iter().map(|s| self.back(s)).collect(),
                reason,
            },
            OpError::Tolerance { entity, wanted } => OpError::Tolerance {
                entity: self.back(entity),
                wanted,
            },
            other => other,
        }
    }

    /// `made`, the record of the boolean over [`Apart::operands`] whose
    /// result is `result`, written against the caller's operands
    /// (`docs/DATA-MODEL.md` §Provenance). Every origin in a copy is its
    /// original. A copy's entity kept in the result is `Modified` from
    /// its original, which the result no longer holds by id. A shared
    /// entity is one input read twice: when the result keeps it by id
    /// (the earlier operand's, untouched), what the boolean made of the
    /// copy is `Generated` from it and it is not deleted; otherwise its
    /// pieces from both readings are its pieces, and it is deleted only
    /// when neither reading left one.
    pub(super) fn restore(
        &self,
        m: &Model,
        result: Body,
        made: Provenance,
    ) -> Result<Provenance, OpError> {
        if self.untouched() {
            return Ok(made);
        }
        let output = entities(m, result)?;
        // A shared entity the result holds by id: the earlier operand's,
        // kept untouched.
        let kept_shared = |id: EntityId| self.shared.contains(&id) && output.contains(&id);
        let mut out = Provenance::new();
        for origin in made.origins_recorded() {
            let (back, from_copy) = match origin {
                Origin::Entity(s) => (
                    Origin::Entity(self.back(s)),
                    self.original.contains_key(&s.id),
                ),
                Origin::Role(r) => (Origin::Role(r), false),
            };
            for &y in made.generated_from(origin) {
                out.add_generated(back, y);
            }
            let generated_instead = match back {
                Origin::Entity(s) => from_copy && kept_shared(s.id),
                Origin::Role(_) => false,
            };
            for &y in made.modified_from(origin) {
                if generated_instead {
                    out.add_generated(back, y);
                } else {
                    out.add_modified(back, y);
                }
            }
        }
        let recorded: BTreeSet<Shape> = made.outputs();
        for &id in &output {
            if let Some(&old) = self.original.get(&id) {
                let s = Shape::new(id, arris_topo::Orientation::Forward);
                if !recorded.contains(&s) {
                    out.add_modified(Shape::new(old, arris_topo::Orientation::Forward), s);
                }
            }
        }
        for s in made.deleted() {
            let back = self.back(s);
            if kept_shared(back.id) || !out.modified_from(back).is_empty() {
                continue;
            }
            out.add_deleted(back);
        }
        Ok(out)
    }
}
