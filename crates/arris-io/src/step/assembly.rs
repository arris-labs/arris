//! Writing a product tree (ADR-0033): each product once, each placement a
//! `NEXT_ASSEMBLY_USAGE_OCCURRENCE` with its transformation, names as
//! `PRODUCT` names, colours as `STYLED_ITEM`s.

use std::collections::{BTreeMap, BTreeSet};

use arris_check::arris_topo::arris_math::{Control, Frame, Isometry, Meter};
use arris_check::arris_topo::{Body, FaceId, Model};

use super::{Occurrence, ProductTree, Rgb, StepError, Writer, real, refs, string};

/// Why a [`ProductTree`] cannot be written by [`write_products`]: what is
/// wrong with the tree, named by the occurrence or the body.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum TreeError {
    /// An occurrence holds a solid index past the bodies given.
    #[error("{name:?} holds solid {index}, but only {bodies} bodies were given")]
    SolidOutOfRange {
        /// The occurrence's name.
        name: String,
        /// The index.
        index: usize,
        /// How many bodies there are.
        bodies: usize,
    },
    /// One body is held by two products: it would be written twice, each
    /// time as the same faces and edges. Two occurrences of one product
    /// share it by carrying the same `Occurrence::product`.
    #[error("body {index} is held by two products")]
    SolidShared {
        /// The index.
        index: usize,
    },
    /// A placement that is not a finite rigid motion: one a read refused
    /// (`Err`), or a non-finite number.
    #[error("the placement of {name:?} cannot be written: {what}")]
    Placement {
        /// The occurrence's name.
        name: String,
        /// Why.
        what: String,
    },
    /// A colour outside `[0, 1]` or not a number.
    #[error("{name:?} has a colour outside [0, 1]")]
    Colour {
        /// The occurrence's name.
        name: String,
    },
    /// An occurrence with a colour and no solid: a colour is the colour of
    /// an occurrence's own solids (a read gives one only there), and this
    /// one has none to carry it.
    #[error("{name:?} has a colour and no solid to carry it")]
    ColourWithoutSolids {
        /// The occurrence's name.
        name: String,
    },
    /// Two occurrences carry one `Occurrence::product` but differ in name,
    /// solids, colour or children: they would be one product written once.
    #[error("product {product} is named by two occurrences that differ")]
    ProductMismatch {
        /// The product.
        product: u64,
    },
    /// A product holds itself.
    #[error("product {product} holds itself")]
    Cycle {
        /// The product.
        product: u64,
    },
    /// A face colour names a face the solid's body does not write.
    #[error("{face} is no face of the body {solid} written")]
    Face {
        /// The solid, an index into the bodies.
        solid: usize,
        /// The face.
        face: FaceId,
    },
}

/// The STEP AP214 Part 21 text of the product structure `tree` over
/// `bodies`: each root a product, each occurrence of a product placed in
/// its parent by the occurrence's `placement`, names as `PRODUCT` names,
/// colours as `STYLED_ITEM`s (ADR-0033).
///
/// A body is held in its product's own frame: the geometry a read returns
/// for an occurrence is the product's at its placement — *baked* — so a
/// consumer that writes back what it read moves each body by the inverse of
/// its occurrence's composed placement first. Occurrences that carry the
/// same `Some(product)` are one product, written once and placed many
/// times, and must agree in name, solids, colour and children; `None`
/// makes a product of its own. A root has no parent: its placement is not
/// written.
///
/// Guarantees: what is written reads back (`step::read`) to a tree of the
/// same names, nesting and colours, placements equal to rounding, and for
/// each instance the volume and centroid of the body composed with its
/// path; deterministic, the entities in the order of the tree; a tree of
/// one root holding all the bodies writes what [`write`](super::write)
/// does, plus nothing. Errors: [`StepError::NoBodies`] for a tree with no
/// root, [`StepError::Tree`] for one that cannot be written
/// ([`TreeError`]), every error of [`write`](super::write), and
/// [`StepError::Interrupted`] when `control`'s poll or budget stops the
/// call (a step is an occurrence or a body). The model is read only.
///
/// ```
/// use arris_debug::sample;
/// use arris_io::arris_check::arris_topo::Model;
/// use arris_io::arris_check::arris_topo::arris_math::{Control, Isometry, Vec3};
/// use arris_io::step::{self, Occurrence, ProductTree};
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// let part = Occurrence {
///     product: None,
///     name: "pin".into(),
///     placement: Ok(Isometry::from_translation(Vec3::new(10.0, 0.0, 0.0))),
///     colour: None,
///     solids: vec![0],
///     children: vec![],
/// };
/// let assembly = Occurrence {
///     name: "assembly".into(),
///     placement: Ok(Isometry::identity()),
///     solids: vec![],
///     children: vec![part.clone()],
///     ..part
/// };
/// let tree = ProductTree { roots: vec![assembly], faces: vec![] };
/// let text = step::write_products(&m, &[body], &tree, &Control::NONE).unwrap();
/// assert_eq!(text.matches("NEXT_ASSEMBLY_USAGE_OCCURRENCE(").count(), 1);
/// assert!(text.contains("PRODUCT('pin','pin'"));
/// ```
pub fn write_products(
    model: &Model,
    bodies: &[Body],
    tree: &ProductTree,
    control: &Control<'_>,
) -> Result<String, StepError> {
    let Some(first) = tree.roots.first() else {
        return Err(StepError::NoBodies);
    };
    let used = validate(tree, bodies.len())?;
    let mut faces_written = BTreeSet::new();
    for &i in &used {
        faces_written.extend(model.closure(bodies[i])?.faces);
    }
    let owner = bodies.first().map_or_else(
        || arris_check::arris_topo::AnyId::from(model_anchor()),
        |b| b.id.into(),
    );
    let mut w = Writer::new(model, faces_written, owner, &first.name)?;
    let mut emit = Emit {
        bodies,
        meter: Meter::new(control),
        done: BTreeMap::new(),
        styled: Vec::new(),
        paints: BTreeMap::new(),
        usages: 0,
        solid_entities: BTreeMap::new(),
        owner,
    };
    for (i, root) in tree.roots.iter().enumerate() {
        w.occurrence(&mut emit, root, i == 0)?;
    }
    for colour in &tree.faces {
        // The face is of the body its solid index names, which is written.
        let of_body = match bodies.get(colour.solid) {
            Some(&b) => model.closure(b)?.faces.contains(&colour.face),
            None => false,
        };
        let entity = w.face_entities.get(&colour.face).copied();
        let written = emit.solid_entities.contains_key(&colour.solid);
        let Some(entity) = entity.filter(|_| written && of_body) else {
            return Err(TreeError::Face {
                solid: colour.solid,
                face: colour.face,
            }
            .into());
        };
        let items = w.paint(&mut emit, colour.colour, &[entity])?;
        emit.styled.extend(items);
    }
    if !emit.styled.is_empty() {
        let context = w.context_3d;
        w.push(format!(
            "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',({}),#{context})",
            refs(emit.styled.iter().copied())
        ));
    }
    Ok(w.finish())
}

/// A body id to name a number that is not in a body when there are no
/// bodies at all: the model's first, which never exists to be named.
fn model_anchor() -> arris_check::arris_topo::BodyId {
    arris_check::arris_topo::BodyId::new(0, 0)
}

/// The state one [`write_products`] carries.
struct Emit<'a> {
    bodies: &'a [Body],
    meter: Meter<'a>,
    /// The products written, by the `Occurrence::product` they carry.
    done: BTreeMap<u64, Written>,
    /// The `STYLED_ITEM`s so far, for the presentation representation.
    styled: Vec<usize>,
    /// The `PRESENTATION_STYLE_ASSIGNMENT` of each colour, once.
    paints: BTreeMap<[u64; 3], usize>,
    /// How many usages are written, for their ids.
    usages: usize,
    /// The solid entities of each body written.
    solid_entities: BTreeMap<usize, Vec<usize>>,
    owner: arris_check::arris_topo::AnyId,
}

/// A product written: its `PRODUCT_DEFINITION` and its representation.
#[derive(Clone, Copy)]
struct Written {
    definition: usize,
    representation: usize,
}

/// The body indices the tree holds, each in one product only, or why the
/// tree cannot be written.
fn validate(tree: &ProductTree, bodies: usize) -> Result<BTreeSet<usize>, TreeError> {
    fn walk<'t>(
        o: &'t Occurrence,
        is_root: bool,
        bodies: usize,
        used: &mut BTreeSet<usize>,
        seen: &mut BTreeMap<u64, &'t Occurrence>,
        above: &mut Vec<u64>,
    ) -> Result<(), TreeError> {
        if !is_root {
            match &o.placement {
                Err(refusal) => {
                    return Err(TreeError::Placement {
                        name: o.name.clone(),
                        what: refusal.to_string(),
                    });
                }
                Ok(p) => {
                    let r = p.rotation();
                    let t = p.translation();
                    let finite =
                        t.iter().all(|x| x.is_finite()) && r.coords.iter().all(|x| x.is_finite());
                    if !finite {
                        return Err(TreeError::Placement {
                            name: o.name.clone(),
                            what: "a number is not finite".into(),
                        });
                    }
                }
            }
        }
        if let Some(c) = o.colour {
            if !c.0.iter().all(|x| (0.0..=1.0).contains(x)) {
                return Err(TreeError::Colour {
                    name: o.name.clone(),
                });
            }
            if o.solids.is_empty() {
                return Err(TreeError::ColourWithoutSolids {
                    name: o.name.clone(),
                });
            }
        }
        if let Some(key) = o.product {
            if above.contains(&key) {
                return Err(TreeError::Cycle { product: key });
            }
            if let Some(first) = seen.get(&key) {
                return if same_product(first, o) {
                    Ok(())
                } else {
                    Err(TreeError::ProductMismatch { product: key })
                };
            }
            seen.insert(key, o);
        }
        for &i in &o.solids {
            if i >= bodies {
                return Err(TreeError::SolidOutOfRange {
                    name: o.name.clone(),
                    index: i,
                    bodies,
                });
            }
            if !used.insert(i) {
                return Err(TreeError::SolidShared { index: i });
            }
        }
        above.extend(o.product);
        for child in &o.children {
            walk(child, false, bodies, used, seen, above)?;
        }
        if o.product.is_some() {
            above.pop();
        }
        Ok(())
    }
    let mut used = BTreeSet::new();
    let mut seen = BTreeMap::new();
    for root in &tree.roots {
        walk(root, true, bodies, &mut used, &mut seen, &mut Vec::new())?;
    }
    Ok(used)
}

/// Whether `a` and `b`, two occurrences of one product, agree in
/// everything but where each stands: name, colour, solids, and children
/// with their placements.
fn same_product(a: &Occurrence, b: &Occurrence) -> bool {
    a.name == b.name
        && a.colour == b.colour
        && a.solids == b.solids
        && a.children.len() == b.children.len()
        && a.children
            .iter()
            .zip(&b.children)
            .all(|(x, y)| x.placement == y.placement && same_product(x, y))
}

impl Writer<'_> {
    /// The product of occurrence `o` — written once per `Some(product)` —
    /// with its solids, colours and children placed in it.
    fn occurrence(
        &mut self,
        emit: &mut Emit<'_>,
        o: &Occurrence,
        first: bool,
    ) -> Result<Written, StepError> {
        emit.meter.tick()?;
        if let Some(written) = o.product.and_then(|k| emit.done.get(&k)) {
            return Ok(*written);
        }
        let written = if first {
            Written {
                definition: self.first_definition,
                representation: self.shape_representation,
            }
        } else {
            self.add_product(&o.name)
        };
        if let Some(k) = o.product {
            emit.done.insert(k, written);
        }
        let mut solids = Vec::new();
        for &i in &o.solids {
            emit.meter.tick()?;
            let entities = self.solids(emit.bodies[i])?;
            solids.extend(entities.iter().copied());
            emit.solid_entities.insert(i, entities);
        }
        if let Some(colour) = o.colour {
            let items = self.paint(emit, colour, &solids)?;
            emit.styled.extend(items);
        }
        let text = if solids.is_empty() {
            format!(
                "SHAPE_REPRESENTATION('',(#{}),#{})",
                self.world_placement, self.context_3d
            )
        } else {
            format!(
                "ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#{})",
                refs(std::iter::once(self.world_placement).chain(solids)),
                self.context_3d
            )
        };
        self.set(written.representation, text);
        for child in &o.children {
            let below = self.occurrence(emit, child, false)?;
            let placement = match &child.placement {
                Ok(p) => *p,
                Err(_) => Isometry::identity(),
            };
            self.usage(emit, written, below, &child.name, &placement)?;
        }
        Ok(written)
    }

    /// A product of `name` beside the first: its chain of definitions and
    /// its shape representation, reserved, with the number of each.
    fn add_product(&mut self, name: &str) -> Written {
        let name = string(name);
        let product = self.push(format!(
            "PRODUCT('{name}','{name}','',(#{}))",
            self.product_context
        ));
        let formation = self.push(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
        let definition = self.push(format!(
            "PRODUCT_DEFINITION('design','',#{formation},#{})",
            self.definition_context
        ));
        let shape = self.push(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
        self.push(format!(
            "PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"
        ));
        let representation = self.reserve();
        self.push(format!(
            "SHAPE_DEFINITION_REPRESENTATION(#{shape},#{representation})"
        ));
        Written {
            definition,
            representation,
        }
    }

    /// `child` placed in `parent` by `placement`: the usage occurrence,
    /// its shape, the transformation that moves the child's world axis
    /// onto the placement's, and the relationship that carries it.
    fn usage(
        &mut self,
        emit: &mut Emit<'_>,
        parent: Written,
        child: Written,
        name: &str,
        placement: &Isometry,
    ) -> Result<(), StepError> {
        emit.usages += 1;
        let nauo = self.push(format!(
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE('{}','{}','',#{},#{},$)",
            emit.usages,
            string(name),
            parent.definition,
            child.definition
        ));
        let shape = self.push(format!(
            "PRODUCT_DEFINITION_SHAPE('Placement','Placement of an item',#{nauo})"
        ));
        let axis = self.placement_3d(&placement.apply_frame(&Frame::world()), emit.owner)?;
        let transformation = self.push(format!(
            "ITEM_DEFINED_TRANSFORMATION('','',#{},#{axis})",
            self.world_placement
        ));
        let relation = self.push(format!(
            "( REPRESENTATION_RELATIONSHIP('','',#{},#{}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{transformation}) SHAPE_REPRESENTATION_RELATIONSHIP() )",
            child.representation, parent.representation
        ));
        self.push(format!(
            "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{relation},#{shape})"
        ));
        Ok(())
    }

    /// A `STYLED_ITEM` painting each of `items` with `colour`, through the
    /// style chain the reader follows; the chain is written once per
    /// colour.
    fn paint(
        &mut self,
        emit: &mut Emit<'_>,
        colour: Rgb,
        items: &[usize],
    ) -> Result<Vec<usize>, StepError> {
        let key = colour.0.map(f64::to_bits);
        let assignment = match emit.paints.get(&key) {
            Some(&a) => a,
            None => {
                let [r, g, b] = colour.0;
                let rgb = self.push(format!(
                    "COLOUR_RGB('',{},{},{})",
                    real(r, emit.owner)?,
                    real(g, emit.owner)?,
                    real(b, emit.owner)?
                ));
                let fill = self.push(format!("FILL_AREA_STYLE_COLOUR('',#{rgb})"));
                let area = self.push(format!("FILL_AREA_STYLE('',(#{fill}))"));
                let surface = self.push(format!("SURFACE_STYLE_FILL_AREA(#{area})"));
                let side = self.push(format!("SURFACE_SIDE_STYLE('',(#{surface}))"));
                let usage = self.push(format!("SURFACE_STYLE_USAGE(.BOTH.,#{side})"));
                let a = self.push(format!("PRESENTATION_STYLE_ASSIGNMENT((#{usage}))"));
                emit.paints.insert(key, a);
                a
            }
        };
        Ok(items
            .iter()
            .map(|item| self.push(format!("STYLED_ITEM('color',(#{assignment}),#{item})")))
            .collect())
    }
}
