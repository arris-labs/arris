//! The product structure flattened (ADR-0025 §5): every solid of the file
//! at every placement an assembly puts it, each an instance of its own.
//!
//! - **Representations** are the instances whose record ends in
//!   `REPRESENTATION` — a `SHAPE_REPRESENTATION`, an
//!   `ADVANCED_BREP_SHAPE_REPRESENTATION` — with their items and their
//!   context. Two joined by a relationship with no transformation (a
//!   `SHAPE_REPRESENTATION_RELATIONSHIP`, the link from a product's shape
//!   to its B-rep) are one node: they share a placement.
//! - **Placements** are the edges from a child node to a parent: a
//!   `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` — what a
//!   `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` names for each
//!   `NEXT_ASSEMBLY_USAGE_OCCURRENCE` — from its `rep_1` to its `rep_2`,
//!   whose `ITEM_DEFINED_TRANSFORMATION` moves `transform_item_1`, read in
//!   `rep_1`'s units, onto `transform_item_2`, read in `rep_2`'s; and a
//!   `MAPPED_ITEM` from the representation its `REPRESENTATION_MAP` maps
//!   to the one holding it, moving the map's origin onto its target.
//! - **Instances** are the paths from a node no placement leads out of —
//!   the root assembly, or a lone part — down to each solid, the motions
//!   along a path composed from the solid up. A solid's instances are
//!   numbered in the order of their paths, each path the ascending list
//!   of the placements' ids from the root, so the numbering is the
//!   file's, not the traversal's. A solid no representation lists is
//!   instance 0 with no context, which the reader refuses.
//!
//! A presentation — a `DRAUGHTING_MODEL`, a saved view of AP242's PMI
//! that maps the part's shape into itself to annotate it, or a
//! `…_PRESENTATION_REPRESENTATION` — is no representation here: what it
//! maps is shown, not placed.
//!
//! A placement given by a `CARTESIAN_TRANSFORMATION_OPERATOR_3D`, or one
//! whose axes do not read, is a refusal of every solid below it; a cycle
//! of placements is not followed round.

use std::collections::{BTreeMap, BTreeSet};

use arris_math::Isometry;

use super::Refusal;
use super::entities::{Args, Entities, describe};
use super::units::Units;
use crate::step::part21::{Instance, Param};

/// How many placements deep a path is followed: deeper than any
/// assembly, and the end of a cycle the per-path check missed.
const PATH_DEPTH: usize = 64;

/// One placement of one solid.
pub(crate) struct Placed {
    /// The solid.
    pub(crate) solid: u64,
    /// Which placement of it: its rank among the solid's paths.
    pub(crate) instance: u32,
    /// The context its representation reads in, if any lists it.
    pub(crate) context: Option<u64>,
    /// The motion from the solid's representation to the root's, `None`
    /// where there is none to make; or why there is none.
    pub(crate) motion: Motion,
}

/// The motion a path composes, or why it has none.
type Motion = Result<Option<Isometry>, Refusal>;

/// A node to visit: the node, the path to it, its motion, the nodes
/// the path passed through, and the edge that placed it.
type Visit<'a> = (u64, Vec<u64>, Motion, Vec<u64>, Option<&'a Edge>);

/// One place the walk stood: a node at the end of a path from a root,
/// which the product tree (`super::products`) reports as an occurrence
/// — the tree and the flattening are one walk, so they agree on what
/// "instance k" is.
pub(crate) struct Site {
    /// The root the path starts from.
    pub(crate) root: u64,
    /// The ids of the placements from the root, ascending in the order
    /// the flattening numbers a solid's instances by.
    pub(crate) path: Vec<u64>,
    /// The node: its least representation.
    pub(crate) node: u64,
    /// The placement in the parent, in the caller's unit, or why there
    /// is none; `Ok(identity)` at a root.
    pub(crate) placement: Result<Isometry, Refusal>,
    /// The solids the node holds, each with which instance of it this
    /// site is.
    pub(crate) solids: Vec<(u64, u32)>,
}

/// What [`placements`] finds: every solid's instances and every place
/// the walk stood.
pub(crate) struct Flat {
    /// Every instance of every solid.
    pub(crate) placed: Vec<Placed>,
    /// The sites, ascending by root and then by path.
    pub(crate) sites: Vec<Site>,
    /// The representations each node joins, by the node.
    pub(crate) members: BTreeMap<u64, Vec<u64>>,
}

/// A representation: its items and its context.
struct Representation {
    items: Vec<u64>,
    context: u64,
}

/// A placement: the child node's representation moved into the
/// parent's.
struct Edge {
    /// The instance that places: the relationship, or the mapped item.
    id: u64,
    child: u64,
    parent: u64,
    /// The motion, or why it has none.
    motion: Result<Isometry, Refusal>,
}

/// Every instance of each of `solids` the file places, ascending by
/// solid and then by instance, and the sites of the walk that found
/// them.
pub(crate) fn placements(
    instances: &BTreeMap<u64, Instance>,
    entities: &Entities<'_>,
    solids: &BTreeSet<u64>,
    units: &mut dyn FnMut(u64) -> Result<Units, Refusal>,
) -> Flat {
    let mut reps: BTreeMap<u64, Representation> = BTreeMap::new();
    for (&id, instance) in instances {
        if is_presentation(instance) {
            continue;
        }
        for record in instance.records() {
            if !record.name.ends_with("REPRESENTATION") {
                continue;
            }
            if let [_, Param::List(items), Param::Ref(context), ..] = &record.params[..] {
                let items = items
                    .iter()
                    .filter_map(|i| match i {
                        Param::Ref(r) => Some(*r),
                        _ => None,
                    })
                    .collect();
                reps.insert(
                    id,
                    Representation {
                        items,
                        context: *context,
                    },
                );
            }
        }
    }

    // Representations joined with no transformation are one node.
    let mut node: BTreeMap<u64, u64> = reps.keys().map(|&r| (r, r)).collect();
    let find = |node: &BTreeMap<u64, u64>, mut r: u64| {
        for _ in 0..reps.len().max(1) {
            match node.get(&r) {
                Some(&up) if up != r => r = up,
                _ => break,
            }
        }
        r
    };
    let mut edges: Vec<Edge> = Vec::new();
    for (&id, instance) in instances {
        let Some(relationship) = instance.record("REPRESENTATION_RELATIONSHIP").or_else(|| {
            instance
                .record("SHAPE_REPRESENTATION_RELATIONSHIP")
                .filter(|r| r.params.len() >= 4)
        }) else {
            continue;
        };
        let (Some(Param::Ref(rep_1)), Some(Param::Ref(rep_2))) =
            (relationship.params.get(2), relationship.params.get(3))
        else {
            continue;
        };
        if !(reps.contains_key(rep_1) && reps.contains_key(rep_2)) {
            continue;
        }
        match instance.record("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") {
            None => {
                // The larger id joins the smaller: the node is named by
                // its least representation.
                let (a, b) = (find(&node, *rep_1), find(&node, *rep_2));
                let (lo, hi) = (a.min(b), a.max(b));
                node.insert(hi, lo);
            }
            Some(with) => {
                let motion = match with.params.first() {
                    Some(Param::Ref(t)) => {
                        transformation(entities, id, *t, &reps, *rep_1, *rep_2, units)
                    }
                    _ => Err(super::entities::malformed(
                        id,
                        "a relationship with a transformation that names none",
                    )),
                };
                edges.push(Edge {
                    id,
                    child: *rep_1,
                    parent: *rep_2,
                    motion,
                });
            }
        }
    }
    for (&rep, r) in &reps {
        for &item in &r.items {
            let Some(mapped) = instances.get(&item).and_then(|i| i.record("MAPPED_ITEM")) else {
                continue;
            };
            let args = Args {
                id: item,
                record: mapped,
            };
            let edge = (|| {
                let map = entities.record(item, args.reference(1)?, "REPRESENTATION_MAP")?;
                let origin = map.reference(0)?;
                let child = map.reference(1)?;
                let target = args.reference(2)?;
                let child_context = reps.get(&child).map(|c| c.context).ok_or_else(|| {
                    super::entities::malformed(
                        map.id,
                        format!("maps #{child}, which is no representation"),
                    )
                })?;
                let motion = displacement(
                    entities,
                    item,
                    origin,
                    child_context,
                    target,
                    r.context,
                    units,
                );
                Ok::<_, Refusal>(Edge {
                    id: item,
                    child,
                    parent: rep,
                    motion,
                })
            })();
            match edge {
                Ok(edge) => edges.push(edge),
                Err(refusal) => edges.push(Edge {
                    id: item,
                    child: item,
                    parent: rep,
                    motion: Err(refusal),
                }),
            }
        }
    }

    // Each node's solids, from the representations it joins, by the
    // least representation that lists each.
    let mut held: BTreeMap<u64, BTreeMap<u64, u64>> = BTreeMap::new();
    for (&rep, r) in &reps {
        for &item in &r.items {
            if solids.contains(&item) {
                held.entry(find(&node, rep))
                    .or_default()
                    .entry(item)
                    .or_insert(r.context);
            }
        }
    }
    let mut below: BTreeMap<u64, Vec<&Edge>> = BTreeMap::new();
    let mut placed_nodes: BTreeSet<u64> = BTreeSet::new();
    for e in &edges {
        let (child, parent) = (find(&node, e.child), find(&node, e.parent));
        below.entry(parent).or_default().push(e);
        placed_nodes.insert(child);
    }
    let roots: Vec<u64> = (reps.keys().map(|&r| find(&node, r)))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .filter(|n| !placed_nodes.contains(n))
        .collect();

    // Every path to every solid, from each root down.
    let mut found: BTreeMap<u64, BTreeMap<Vec<u64>, (u64, Motion)>> = BTreeMap::new();
    let mut stack: Vec<Visit> = roots
        .iter()
        .rev()
        .map(|&r| (r, Vec::new(), Ok(None), vec![r], None))
        .collect();
    let mut visited: Vec<(u64, Vec<u64>, u64, Option<&Edge>)> = Vec::new();
    while let Some((at, path, motion, trail, via)) = stack.pop() {
        visited.push((trail[0], path.clone(), at, via));
        if let Some(solids) = held.get(&at) {
            for (&solid, &context) in solids {
                found
                    .entry(solid)
                    .or_default()
                    .entry(path.clone())
                    .or_insert_with(|| (context, motion.clone()));
            }
        }
        if path.len() >= PATH_DEPTH {
            continue;
        }
        for e in below.get(&at).into_iter().flatten().rev() {
            let child = find(&node, e.child);
            if trail.contains(&child) {
                continue;
            }
            // The child's points into the parent's frame, then on up.
            let composed = match (&e.motion, &motion) {
                (Err(r), _) | (Ok(_), Err(r)) => Err(r.clone()),
                (Ok(m), Ok(None)) => Ok(Some(*m)),
                (Ok(m), Ok(Some(up))) => Ok(Some(m.then(up))),
            };
            let mut path = path.clone();
            path.push(e.id);
            let mut trail = trail.clone();
            trail.push(child);
            stack.push((child, path, composed, trail, Some(e)));
        }
    }

    let mut sites: Vec<Site> = visited
        .into_iter()
        .map(|(root, path, node, via)| {
            let rank = |solid: u64| {
                found
                    .get(&solid)
                    .and_then(|paths| paths.keys().position(|p| *p == path))
                    .and_then(|i| u32::try_from(i).ok())
                    .unwrap_or(0)
            };
            Site {
                root,
                node,
                solids: (held.get(&node).into_iter().flatten())
                    .map(|(&solid, _)| (solid, rank(solid)))
                    .collect(),
                placement: match via {
                    Some(e) => e.motion.clone(),
                    None => Ok(Isometry::identity()),
                },
                path,
            }
        })
        .collect();
    sites.sort_by(|a, b| (a.root, &a.path).cmp(&(b.root, &b.path)));
    let mut members: BTreeMap<u64, Vec<u64>> = BTreeMap::new();
    for &rep in reps.keys() {
        members.entry(find(&node, rep)).or_default().push(rep);
    }

    let mut out = Vec::new();
    for &solid in solids {
        match found.remove(&solid) {
            Some(paths) => {
                for (instance, (_, (context, motion))) in paths.into_iter().enumerate() {
                    out.push(Placed {
                        solid,
                        instance: u32::try_from(instance).unwrap_or(u32::MAX),
                        context: Some(context),
                        motion,
                    });
                }
            }
            None => {
                // Listed by no node a root reaches: by the least
                // representation that lists it, where it stands.
                let context = reps
                    .values()
                    .find(|r| r.items.contains(&solid))
                    .map(|r| r.context);
                out.push(Placed {
                    solid,
                    instance: 0,
                    context,
                    motion: Ok(None),
                });
            }
        }
    }
    Flat {
        placed: out,
        sites,
        members,
    }
}

/// Whether `instance` is a presentation of the product rather than a
/// part of its structure: a `DRAUGHTING_MODEL` — an AP242 saved view,
/// which maps the part's shape into itself, from a camera's placement,
/// to annotate it — or a `…_PRESENTATION_REPRESENTATION`. Its mapped
/// items place nothing, and it is no node of the flattening.
fn is_presentation(instance: &Instance) -> bool {
    instance
        .records()
        .iter()
        .any(|r| r.name == "DRAUGHTING_MODEL" || r.name.ends_with("PRESENTATION_REPRESENTATION"))
}

/// The motion of the relationship `relationship`'s transformation `id`:
/// an `ITEM_DEFINED_TRANSFORMATION` moving its first item, in `rep_1`'s
/// units, onto its second, in `rep_2`'s.
fn transformation(
    entities: &Entities<'_>,
    relationship: u64,
    id: u64,
    reps: &BTreeMap<u64, Representation>,
    rep_1: u64,
    rep_2: u64,
    units: &mut dyn FnMut(u64) -> Result<Units, Refusal>,
) -> Result<Isometry, Refusal> {
    let instance = entities.get(relationship, id)?;
    let Some(record) = instance.record("ITEM_DEFINED_TRANSFORMATION") else {
        return Err(Refusal::Unsupported {
            entity: id,
            name: format!("{} as an assembly's placement", describe(instance)),
        });
    };
    let args = Args { id, record };
    let (first, second) = (args.reference(2)?, args.reference(3)?);
    let context = |rep: u64| {
        reps.get(&rep)
            .map(|r| r.context)
            .ok_or_else(|| super::entities::malformed(relationship, "names no representation"))
    };
    displacement(
        entities,
        id,
        first,
        context(rep_1)?,
        second,
        context(rep_2)?,
        units,
    )
}

/// The motion taking the placement `from`, read in `from_context`'s
/// units, onto `to`, read in `to_context`'s: `to ∘ from⁻¹`.
fn displacement(
    entities: &Entities<'_>,
    at: u64,
    from: u64,
    from_context: u64,
    to: u64,
    to_context: u64,
    units: &mut dyn FnMut(u64) -> Result<Units, Refusal>,
) -> Result<Isometry, Refusal> {
    let a = units(from_context)?.placement(entities, at, from)?;
    let b = units(to_context)?.placement(entities, at, to)?;
    Ok(a.as_isometry().inverse().then(&b.as_isometry()))
}
