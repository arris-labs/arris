//! `build`: a solid from topology the consumer filled, every entity
//! recorded under the consumer's own key (ADR-0028).

use core::fmt;
use std::collections::{BTreeMap, BTreeSet};

use arris_check::arris_topo::builder::{BuildError, Builder, Built, EdgeRef, FaceRef, VertexRef};
use arris_check::arris_topo::entity::BodyKind;
use arris_check::arris_topo::provenance::ConsumerKey;
use arris_check::arris_topo::{Body, Model, Orientation, Provenance, Role, Shape};
use arris_check::{Level, Report};

use crate::error::{Fault, OpError};

/// The consumer's key for every slot of a [`Builder`]: what
/// [`build`] records each entity it appends as `Generated` from, as
/// `Role::Consumer(ConsumerKey { namespace, key })`. The kernel never
/// reads a key: keys need not be unique — two slots under one key are two
/// outputs of one role, in slot order — and one key may name entities of
/// different kinds (ADR-0028). A key for a slot the builder does not hold
/// is ignored.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct BuildKeys {
    /// The namespace every key of this build is in.
    pub namespace: u32,
    /// The key of each vertex slot.
    pub vertices: BTreeMap<VertexRef, u64>,
    /// The key of each edge slot.
    pub edges: BTreeMap<EdgeRef, u64>,
    /// The key of each face slot.
    pub faces: BTreeMap<FaceRef, u64>,
    /// The key of each shell, by its index: the shell index its faces
    /// carry ([`StagedFace::shell`]), `0` for a builder of operators.
    ///
    /// [`StagedFace::shell`]: arris_check::arris_topo::builder::StagedFace::shell
    pub shells: Vec<u64>,
    /// The body's key.
    pub body: u64,
}

/// A slot of the builder handed to [`build`], named in its refusals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum BuildSlot {
    /// A vertex slot.
    Vertex(VertexRef),
    /// An edge slot.
    Edge(EdgeRef),
    /// A face slot.
    Face(FaceRef),
    /// A shell, by the index its faces carry.
    Shell(usize),
}

impl fmt::Display for BuildSlot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            BuildSlot::Vertex(v) => write!(f, "{v}"),
            BuildSlot::Edge(e) => write!(f, "{e}"),
            BuildSlot::Face(x) => write!(f, "{x}"),
            BuildSlot::Shell(i) => write!(f, "shell {i}"),
        }
    }
}

/// Why [`build`] refused the consumer's topology as a solid: the input's
/// fault, not the kernel's, so none is [`OpError::Internal`].
#[derive(Debug, Clone, PartialEq)]
pub enum Rejection {
    /// [`Builder::finish`] refused it: a loop with no uses, a missing
    /// pcurve, an edge not used exactly twice.
    Builder(BuildError),
    /// The slot is an entity already in the model, taken whole by a
    /// `Keep` spec of [`Builder::assemble`]: `build` makes a body from
    /// nothing, and a kept entity is another body's.
    Kept(BuildSlot),
    /// The finished body fails the checker at `Level::Full`.
    Checker(Box<Report>),
}

impl fmt::Display for Rejection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Rejection::Builder(e) => write!(f, "the builder refused it: {e}"),
            Rejection::Kept(slot) => write!(f, "{slot} is an entity already in the model"),
            Rejection::Checker(report) => write!(f, "it fails the checker:\n{report}"),
        }
    }
}

/// The provenance of a body every entity of which has a role: each
/// vertex, edge and face by its slot, each shell by its position in
/// `built.shells`, and the body.
pub(crate) fn roles(
    built: &Built,
    vertex: impl Fn(VertexRef) -> Result<Role, OpError>,
    edge: impl Fn(EdgeRef) -> Result<Role, OpError>,
    face: impl Fn(FaceRef) -> Result<Role, OpError>,
    shell: impl Fn(usize) -> Result<Role, OpError>,
    body: Role,
) -> Result<Provenance, OpError> {
    let mut p = Provenance::new();
    for (&r, &id) in &built.vertices {
        p.add_generated(vertex(r)?, Shape::new(id, Orientation::Forward));
    }
    for (&r, &id) in &built.edges {
        p.add_generated(edge(r)?, Shape::new(id, Orientation::Forward));
    }
    for (&r, &id) in &built.faces {
        p.add_generated(face(r)?, Shape::new(id, Orientation::Forward));
    }
    for (i, &id) in built.shells.iter().enumerate() {
        p.add_generated(shell(i)?, Shape::new(id, Orientation::Forward));
    }
    p.add_generated(body, built.body);
    Ok(p)
}

/// The solid `builder` holds, finished into `model`, with every entity
/// recorded as `Generated` from `Role::Consumer` of the key `keys` gives
/// its slot: the operation for a consumer that builds topology itself
/// (`Builder`'s Euler operators or [`Builder::assemble`]) and wants its
/// provenance chains to end at names of its own (ADR-0028). An `Ok`
/// body passes the checker at `Level::Full` in **every** build profile,
/// not only in debug builds as other operations' outputs do: the
/// topology is the consumer's input, and a body that fails is refused
/// rather than treated as a kernel bug. The record passes
/// [`audit`](arris_check::arris_topo::provenance::audit) with no inputs.
///
/// Errors, each leaving the model as it was: [`OpError::Unkeyed`] naming
/// the first live slot, in vertex, edge, face, shell order, that `keys`
/// has no key for; [`OpError::Rejected`] with [`Rejection::Kept`] for a
/// slot `assemble` kept from the model, [`Rejection::Builder`] for
/// `finish`'s refusal and [`Rejection::Checker`] with the report of a
/// body the checker fails.
///
/// ```
/// use arris_debug::polyhedron::polyhedron;
/// use arris_ops::arris_check::arris_topo::arris_math::Point3;
/// use arris_ops::arris_check::arris_topo::provenance::{ConsumerKey, Role};
/// use arris_ops::arris_check::arris_topo::Model;
/// use arris_ops::build;
///
/// // A tetrahedron, its faces counter-clockwise from outside, filled into
/// // a builder by `Builder::assemble` with point `i` keyed `i` and face
/// // `j` keyed `j` in namespace 7.
/// let mut m = Model::default();
/// let points = [
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.0, 1.0, 0.0),
///     Point3::new(0.0, 0.0, 1.0),
/// ];
/// let faces = [vec![vec![0, 2, 1]], vec![vec![0, 1, 3]], vec![vec![0, 3, 2]], vec![vec![1, 2, 3]]];
/// let (builder, keys) = polyhedron(&mut m, &points, &faces, 7)?;
/// let (body, provenance) = build(&mut m, builder, &keys)?;
/// // The slanted face, by the consumer's own name for it.
/// let slanted = Role::Consumer(ConsumerKey { namespace: 7, key: 3 });
/// let generated = provenance.generated_from(slanted);
/// // Keys need not be unique: point 3 and the edge `0 << 32 | 3` share it.
/// assert_eq!(generated.len(), 3);
/// assert!(m.faces(body)?.iter().any(|f| generated.contains(&(*f).into())));
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn build(
    model: &mut Model,
    builder: Builder,
    keys: &BuildKeys,
) -> Result<(Body, Provenance), OpError> {
    let mut shells = BTreeSet::new();
    for (v, staged) in builder.vertices() {
        admit(
            BuildSlot::Vertex(v),
            staged.kept().is_some(),
            keys.vertices.contains_key(&v),
        )?;
    }
    for (e, staged) in builder.edges() {
        admit(
            BuildSlot::Edge(e),
            staged.kept().is_some(),
            keys.edges.contains_key(&e),
        )?;
    }
    for (f, staged) in builder.faces() {
        admit(
            BuildSlot::Face(f),
            staged.kept().is_some(),
            keys.faces.contains_key(&f),
        )?;
        shells.insert(staged.shell());
    }
    // `finish` makes one shell per index the faces carry, in index
    // order: the `i`-th of `built.shells` is the `i`-th index here.
    let shells: Vec<usize> = shells.into_iter().collect();
    for &i in &shells {
        admit(BuildSlot::Shell(i), false, i < keys.shells.len())?;
    }
    let role = |key: u64| {
        Role::Consumer(ConsumerKey {
            namespace: keys.namespace,
            key,
        })
    };
    let invariant = || {
        OpError::Internal(Fault::Invariant {
            what: "a built slot's key",
        })
    };
    model.transaction(|m| {
        let built = builder
            .finish(m, BodyKind::Solid)
            .map_err(|e| OpError::Rejected(Rejection::Builder(e)))?;
        let report = arris_check::check(m, built.body, Level::Full);
        if !report.is_ok() {
            return Err(OpError::Rejected(Rejection::Checker(Box::new(report))));
        }
        let provenance = roles(
            &built,
            |v| {
                keys.vertices
                    .get(&v)
                    .copied()
                    .map(role)
                    .ok_or_else(invariant)
            },
            |e| keys.edges.get(&e).copied().map(role).ok_or_else(invariant),
            |f| keys.faces.get(&f).copied().map(role).ok_or_else(invariant),
            |i| {
                shells
                    .get(i)
                    .and_then(|&s| keys.shells.get(s))
                    .copied()
                    .map(role)
                    .ok_or_else(invariant)
            },
            role(keys.body),
        )?;
        Ok((built.body, provenance))
    })
}

/// A slot `build` can take: not kept from the model, and keyed.
fn admit(slot: BuildSlot, kept: bool, keyed: bool) -> Result<(), OpError> {
    if kept {
        Err(OpError::Rejected(Rejection::Kept(slot)))
    } else if !keyed {
        Err(OpError::Unkeyed { slot })
    } else {
        Ok(())
    }
}
