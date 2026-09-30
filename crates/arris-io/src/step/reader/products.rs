//! The product tree (ADR-0033): the file's assemblies as occurrences of
//! products, beside the flattened bodies. The tree is the walk
//! [`super::assembly`] already makes — one occurrence per site of it —
//! so it and the bodies agree on what "instance k" is; what this module
//! adds is each node's product, its name, and the shape of the result.

use std::collections::BTreeMap;

use arris_check::arris_topo::arris_math::{Interrupted, Isometry, Meter};

use super::Refusal;
use super::assembly::Flat;
use crate::step::part21::{Instance, Param};

/// A colour, as red, green and blue in `[0, 1]`: what a `COLOUR_RGB`
/// spells (ADR-0033).
///
/// ```
/// use arris_io::step::Rgb;
///
/// let orange = Rgb([1.0, 0.5, 0.0]);
/// assert_eq!(orange.0[1], 0.5);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rgb(pub [f64; 3]);

/// A face a file colours apart from its solid (ADR-0033).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FaceColour {
    /// The solid, an index into [`Read::solids`](super::Read::solids).
    pub solid: usize,
    /// The face, of that solid's body.
    pub face: arris_check::arris_topo::FaceId,
    /// Its colour.
    pub colour: Rgb,
}

/// One place a product stands in the file's assemblies: the product, the
/// placement that puts it in its parent, what it holds and what stands
/// in it (ADR-0033).
///
/// Guarantees: a child's [`placement`](Occurrence::placement) is in its
/// parent's frame, in [`ReadOptions::length_unit`](super::ReadOptions);
/// [`solids`](Occurrence::solids) are indices into
/// [`Read::solids`](super::Read::solids) of the instances whose paths
/// end here; an occurrence is kept if it has a product, a solid or a
/// child, and so is every placement of a part the file places.
#[derive(Debug, Clone, PartialEq)]
pub struct Occurrence {
    /// The `PRODUCT_DEFINITION` the occurrence is of, or `None` for a
    /// representation the file attaches to no product.
    pub product: Option<u64>,
    /// The product's name: its `PRODUCT` name, or its id where the name
    /// is empty; empty where the file names none.
    pub name: String,
    /// The placement in the parent, or why there is none: a transformation
    /// Arris does not read. The identity at a root.
    pub placement: Result<Isometry, Refusal>,
    /// The colour of the occurrence's solids, where the file gives them
    /// one (ADR-0033).
    pub colour: Option<Rgb>,
    /// The solids the occurrence holds itself, as indices into
    /// [`Read::solids`](super::Read::solids).
    pub solids: Vec<usize>,
    /// The occurrences placed in this one, ascending by the file's
    /// placement.
    pub children: Vec<Occurrence>,
}

/// The file's assemblies: the roots, and the faces coloured apart from
/// their solids (ADR-0033).
///
/// ```
/// use arris_io::step::ProductTree;
///
/// let empty = ProductTree::default();
/// assert!(empty.roots.is_empty());
/// ```
#[derive(Debug, Clone, Default, PartialEq)]
pub struct ProductTree {
    /// The occurrences no placement leads to, ascending by the file
    /// entity of their representation: each root assembly, each lone part.
    pub roots: Vec<Occurrence>,
    /// The faces a file colours apart from their solids.
    pub faces: Vec<FaceColour>,
}

impl ProductTree {
    /// Every occurrence, parents before children, in the tree's order.
    ///
    /// ```
    /// use arris_io::step::ProductTree;
    ///
    /// assert_eq!(ProductTree::default().occurrences().count(), 0);
    /// ```
    pub fn occurrences(&self) -> impl Iterator<Item = &Occurrence> {
        let mut stack: Vec<&Occurrence> = self.roots.iter().rev().collect();
        core::iter::from_fn(move || {
            let next = stack.pop()?;
            stack.extend(next.children.iter().rev());
            Some(next)
        })
    }
}

/// The tree of `flat`'s sites, with `index` giving each solid instance's
/// place in `Read::solids`.
pub(crate) fn build(
    instances: &BTreeMap<u64, Instance>,
    flat: &Flat,
    index: &BTreeMap<(u64, u32), usize>,
    meter: &mut Meter<'_>,
) -> Result<ProductTree, Interrupted> {
    let products = Products::new(instances);
    let mut made: Vec<Option<Occurrence>> = Vec::with_capacity(flat.sites.len());
    for site in &flat.sites {
        meter.tick()?;
        let pd = flat
            .members
            .get(&site.node)
            .into_iter()
            .flatten()
            .find_map(|rep| products.definition_of.get(rep).copied());
        let (product, name) = match pd {
            Some(pd) => (Some(pd), products.name(pd)),
            None => (None, String::new()),
        };
        made.push(Some(Occurrence {
            product,
            name,
            placement: site.placement.clone(),
            colour: None,
            solids: site
                .solids
                .iter()
                .filter_map(|key| index.get(key).copied())
                .collect(),
            children: Vec::new(),
        }));
    }
    // Parents come before their children in the sites' order: take them
    // last to first, each child into its parent.
    let position: BTreeMap<(u64, &[u64]), usize> = flat
        .sites
        .iter()
        .enumerate()
        .map(|(i, s)| ((s.root, s.path.as_slice()), i))
        .collect();
    let mut roots = Vec::new();
    for (i, site) in flat.sites.iter().enumerate().rev() {
        let Some(mut occurrence) = made[i].take() else {
            continue;
        };
        occurrence.children.reverse();
        let kept = occurrence.product.is_some()
            || !occurrence.solids.is_empty()
            || !occurrence.children.is_empty();
        if !kept {
            continue;
        }
        let parent = site
            .path
            .split_last()
            .and_then(|(_, up)| position.get(&(site.root, up)))
            .and_then(|&p| made[p].as_mut());
        match (site.path.is_empty(), parent) {
            (true, _) => roots.push(occurrence),
            (false, Some(parent)) => parent.children.push(occurrence),
            (false, None) => {}
        }
    }
    roots.reverse();
    Ok(ProductTree {
        roots,
        faces: Vec::new(),
    })
}

/// The file's products: which `PRODUCT_DEFINITION` each representation
/// defines, and each definition's name.
struct Products<'a> {
    instances: &'a BTreeMap<u64, Instance>,
    /// `SHAPE_DEFINITION_REPRESENTATION`: representation → definition.
    definition_of: BTreeMap<u64, u64>,
}

impl<'a> Products<'a> {
    fn new(instances: &'a BTreeMap<u64, Instance>) -> Self {
        let mut definition_of = BTreeMap::new();
        for instance in instances.values() {
            let Some(sdr) = instance.record("SHAPE_DEFINITION_REPRESENTATION") else {
                continue;
            };
            let [Param::Ref(shape), Param::Ref(rep), ..] = sdr.params[..] else {
                continue;
            };
            // The shape is that of a product definition; one of a
            // placement (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`) is no product's.
            let Some(pds) = instances
                .get(&shape)
                .and_then(|i| i.record("PRODUCT_DEFINITION_SHAPE"))
            else {
                continue;
            };
            let Some(Param::Ref(definition)) = pds.params.get(2) else {
                continue;
            };
            if instances.get(definition).is_some_and(|i| {
                i.record("PRODUCT_DEFINITION").is_some()
                    || i.record("PRODUCT_DEFINITION_WITH_ASSOCIATED_DOCUMENTS")
                        .is_some()
            }) {
                definition_of.entry(rep).or_insert(*definition);
            }
        }
        Products {
            instances,
            definition_of,
        }
    }

    /// The name of definition `pd`'s product: its name, or its id where
    /// the name is empty, or empty.
    fn name(&self, pd: u64) -> String {
        let text = |p: Option<&Param>| match p {
            Some(Param::String(s)) => s.clone(),
            _ => String::new(),
        };
        let product = (|| {
            let Some(Param::Ref(formation)) = self.instances.get(&pd)?.records()[0].params.get(2)
            else {
                return None;
            };
            let formation = self.instances.get(formation)?;
            let formation = formation
                .records()
                .iter()
                .find(|r| r.name.starts_with("PRODUCT_DEFINITION_FORMATION"))?;
            let Some(Param::Ref(product)) = formation.params.get(2) else {
                return None;
            };
            self.instances.get(product)?.record("PRODUCT")
        })();
        match product {
            Some(p) => {
                let name = text(p.params.get(1));
                if name.is_empty() {
                    text(p.params.first())
                } else {
                    name
                }
            }
            None => String::new(),
        }
    }
}
