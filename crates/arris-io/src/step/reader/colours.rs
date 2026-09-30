//! Colours (ADR-0033): what a file paints a solid or a face, resolved
//! from its `STYLED_ITEM`s through the style chain
//!
//! `STYLED_ITEM → PRESENTATION_STYLE_ASSIGNMENT → SURFACE_STYLE_USAGE →
//! SURFACE_SIDE_STYLE → SURFACE_STYLE_FILL_AREA → FILL_AREA_STYLE →
//! FILL_AREA_STYLE_COLOUR → COLOUR_RGB`.
//!
//! A colour is never a reason to lose a body: anything in the chain that
//! is not plain RGB — a texture, a `COLOUR_RGB` outside `[0, 1]`, a style
//! with no colour, an override made for one context only — is skipped, and
//! the item has no colour from it. An item styled twice takes the lowest
//! `STYLED_ITEM` id, so the choice is the file's, not the traversal's.

use std::collections::BTreeMap;

use arris_check::arris_topo::EntityId;
use arris_check::arris_topo::provenance::{FileEntity, Role};

use super::ReadSolid;
use super::products::{FaceColour, ProductTree, Rgb};
use crate::step::part21::{Instance, Param};

/// The styled items whose item is a solid or a face: what each is painted.
pub(crate) struct Colours {
    by_item: BTreeMap<u64, Rgb>,
}

/// The entity names of a styled item that paints its item everywhere: an
/// override made for one context (`CONTEXT_DEPENDENT_…`) is not here.
const STYLED: [&str; 2] = ["STYLED_ITEM", "OVER_RIDING_STYLED_ITEM"];

/// The colour names a `DRAUGHTING_PRE_DEFINED_COLOUR` is plain RGB in
/// disguise for.
const NAMED: [(&str, [f64; 3]); 8] = [
    ("black", [0.0, 0.0, 0.0]),
    ("white", [1.0, 1.0, 1.0]),
    ("red", [1.0, 0.0, 0.0]),
    ("green", [0.0, 1.0, 0.0]),
    ("blue", [0.0, 0.0, 1.0]),
    ("yellow", [1.0, 1.0, 0.0]),
    ("cyan", [0.0, 1.0, 1.0]),
    ("magenta", [1.0, 0.0, 1.0]),
];

impl Colours {
    /// Every item of `instances` a styled item paints.
    pub(crate) fn of(instances: &BTreeMap<u64, Instance>) -> Colours {
        let mut by_item = BTreeMap::new();
        for instance in instances.values() {
            let Some(styled) = instance
                .records()
                .iter()
                .find(|r| STYLED.contains(&r.name.as_str()))
            else {
                continue;
            };
            let Some(Param::Ref(item)) = styled.params.get(2) else {
                continue;
            };
            if by_item.contains_key(item) {
                continue;
            }
            if let Some(colour) = styled.params.get(1).and_then(|p| colour_of(instances, p)) {
                by_item.insert(*item, colour);
            }
        }
        Colours { by_item }
    }

    /// Paints `tree`: each occurrence the colour of its first painted
    /// solid, and every face of a solid read that the file paints apart.
    pub(crate) fn apply(&self, tree: &mut ProductTree, solids: &[ReadSolid]) {
        if self.by_item.is_empty() {
            return;
        }
        let mut stack: Vec<&mut super::Occurrence> = tree.roots.iter_mut().collect();
        while let Some(occurrence) = stack.pop() {
            occurrence.colour = occurrence
                .solids
                .iter()
                .find_map(|&i| solids.get(i).and_then(|s| self.by_item.get(&s.entity.id)))
                .copied();
            stack.extend(occurrence.children.iter_mut());
        }
        let faces: Vec<u64> = self.by_item.keys().copied().collect();
        for (k, solid) in solids.iter().enumerate() {
            let Ok(back) = &solid.result else { continue };
            // The faces of the file that carry a colour are few: each is
            // asked of the record by its own entity.
            for &id in &faces {
                let origin = Role::File(FileEntity {
                    id,
                    instance: solid.entity.instance,
                });
                for shape in back.provenance.generated_from(origin) {
                    if let (EntityId::Face(face), Some(colour)) = (shape.id, self.by_item.get(&id))
                    {
                        tree.faces.push(FaceColour {
                            solid: k,
                            face,
                            colour: *colour,
                        });
                    }
                }
            }
        }
    }
}

/// The colour of the style assignments `styles` lists: the first that
/// reaches a plain RGB.
fn colour_of(instances: &BTreeMap<u64, Instance>, styles: &Param) -> Option<Rgb> {
    let record = |id: &u64, name: &str| instances.get(id)?.record(name);
    let refs = |p: Option<&Param>| -> Vec<u64> {
        match p {
            Some(Param::List(items)) => items
                .iter()
                .filter_map(|i| match i {
                    Param::Ref(r) => Some(*r),
                    _ => None,
                })
                .collect(),
            _ => Vec::new(),
        }
    };
    let one = |p: Option<&Param>| match p {
        Some(Param::Ref(r)) => Some(*r),
        _ => None,
    };
    for assignment in refs(Some(styles)) {
        let Some(assignment) = record(&assignment, "PRESENTATION_STYLE_ASSIGNMENT") else {
            continue;
        };
        for usage in refs(assignment.params.first()) {
            let Some(usage) = record(&usage, "SURFACE_STYLE_USAGE") else {
                continue;
            };
            let Some(side) =
                one(usage.params.get(1)).and_then(|s| record(&s, "SURFACE_SIDE_STYLE"))
            else {
                continue;
            };
            for fill in refs(side.params.get(1)) {
                let Some(fill) = record(&fill, "SURFACE_STYLE_FILL_AREA") else {
                    continue;
                };
                let Some(area) =
                    one(fill.params.first()).and_then(|a| record(&a, "FILL_AREA_STYLE"))
                else {
                    continue;
                };
                for colour in refs(area.params.get(1)) {
                    let Some(colour) = record(&colour, "FILL_AREA_STYLE_COLOUR") else {
                        continue;
                    };
                    if let Some(rgb) = one(colour.params.get(1)).and_then(|c| rgb(instances, c)) {
                        return Some(rgb);
                    }
                }
            }
        }
    }
    None
}

/// Colour `id` as red, green and blue in `[0, 1]`, or `None` for any
/// other model of colour.
fn rgb(instances: &BTreeMap<u64, Instance>, id: u64) -> Option<Rgb> {
    let instance = instances.get(&id)?;
    if let Some(r) = instance.record("COLOUR_RGB") {
        let mut c = [0.0; 3];
        for (slot, p) in c.iter_mut().zip(r.params.iter().skip(1)) {
            *slot = super::entities::number(p)?;
        }
        return (r.params.len() == 4 && c.iter().all(|x| (0.0..=1.0).contains(x)))
            .then_some(Rgb(c));
    }
    let named = instance.record("DRAUGHTING_PRE_DEFINED_COLOUR")?;
    let Some(Param::String(name)) = named.params.first() else {
        return None;
    };
    NAMED
        .iter()
        .find(|(n, _)| n.eq_ignore_ascii_case(name))
        .map(|(_, c)| Rgb(*c))
}
