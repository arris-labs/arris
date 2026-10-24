//! What `extrude`, `revolve`, `sweep` and `loft` build their solids from
//! in common: the spec lists with the roles that go with them, and the
//! assemble → finish → verify → record tail. Each operation keeps what is
//! its own — which vertices exist, what curves its rails are, how its
//! sides close.

use arris_topo::builder::{Assembly, Builder, EdgeSpec, FaceSpec, VertexSpec};
use arris_topo::entity::BodyKind;
use arris_topo::provenance::SweepPart;
use arris_topo::{Body, Model, Provenance, Role};

use crate::error::{Fault, OpError};
use crate::verify;

use super::record;

/// The vertex, edge and face specs of a sweep in the order they are
/// appended, each with the [`Role`] it is `Generated` from. A spec and its
/// role enter together, so the lists cannot fall out of step; the slot a
/// push returns is what edges and faces name their parts by.
#[derive(Default)]
pub(super) struct Specs {
    vertices: Vec<VertexSpec>,
    vertex_roles: Vec<Role>,
    edges: Vec<EdgeSpec>,
    edge_roles: Vec<Role>,
    faces: Vec<FaceSpec>,
    face_roles: Vec<Role>,
}

/// A sweep's faces grouped into shells: the face specs per shell, the
/// role of each shell, and the role of every face in the order the shells
/// list them.
pub(super) struct Shells {
    pub(super) shells: Vec<Vec<FaceSpec>>,
    pub(super) shell_roles: Vec<Role>,
    pub(super) face_roles: Vec<Role>,
}

impl Specs {
    /// Appends a vertex and returns its slot.
    pub(super) fn vertex(&mut self, spec: VertexSpec, role: Role) -> usize {
        self.vertices.push(spec);
        self.vertex_roles.push(role);
        self.vertices.len() - 1
    }

    /// Appends an edge and returns its slot.
    pub(super) fn edge(&mut self, spec: EdgeSpec, role: Role) -> usize {
        self.edges.push(spec);
        self.edge_roles.push(role);
        self.edges.len() - 1
    }

    /// Appends a face and returns its position among the faces.
    pub(super) fn face(&mut self, spec: FaceSpec, role: Role) -> usize {
        self.faces.push(spec);
        self.face_roles.push(role);
        self.faces.len() - 1
    }

    /// Every face so far in one shell with the role `shell`: the shells of
    /// a sweep whose caps join everything.
    pub(super) fn one_shell(&mut self, shell: Role) -> Shells {
        Shells {
            shells: vec![core::mem::take(&mut self.faces)],
            shell_roles: vec![shell],
            face_roles: core::mem::take(&mut self.face_roles),
        }
    }

    /// The faces bucketed into shells by `key`, one entry per face in the
    /// order they were pushed (a closed sweep has no caps, so every face
    /// is a side): the loop for a closed sweep and loft, the chain for a
    /// full revolve. The shells are stored in `order` — a bucket number
    /// each — with the role `role` gives the bucket, and the face roles
    /// follow them, each face keeping the order it was made in. A key of
    /// `None`, one past the buckets, or a missing one is a kernel bug.
    pub(super) fn shells(
        &mut self,
        key: &[Option<usize>],
        order: &[usize],
        role: impl Fn(usize) -> Role,
    ) -> Result<Shells, OpError> {
        let invariant = || {
            OpError::Internal(Fault::Invariant {
                what: "a shell for every side face",
            })
        };
        let faces = core::mem::take(&mut self.faces);
        let roles = core::mem::take(&mut self.face_roles);
        if key.len() != faces.len() {
            return Err(invariant());
        }
        let mut buckets: Vec<Vec<(FaceSpec, Role)>> = order.iter().map(|_| Vec::new()).collect();
        for ((face, face_role), bucket) in faces.into_iter().zip(roles).zip(key) {
            let Some(bucket) = bucket.and_then(|b| buckets.get_mut(b)) else {
                return Err(invariant());
            };
            bucket.push((face, face_role));
        }
        let mut shells = Vec::with_capacity(order.len());
        let mut shell_roles = Vec::with_capacity(order.len());
        let mut face_roles = Vec::new();
        for &b in order {
            let (specs, roles): (Vec<FaceSpec>, Vec<Role>) = buckets
                .get_mut(b)
                .map(core::mem::take)
                .ok_or_else(invariant)?
                .into_iter()
                .unzip();
            shells.push(specs);
            face_roles.extend(roles);
            shell_roles.push(role(b));
        }
        Ok(Shells {
            shells,
            shell_roles,
            face_roles,
        })
    }
}

/// Assembles `specs` and `shells` into a solid in `m`, finishes and
/// verifies it, and records every entity `Generated` from its role;
/// `part` makes the role of the body itself.
pub(super) fn finish_solid(
    m: &mut Model,
    tolerance: f64,
    specs: Specs,
    shells: Shells,
    part: impl Fn(SweepPart) -> Role,
) -> Result<(Body, Provenance), OpError> {
    let assembly = Assembly {
        vertices: specs.vertices,
        edges: specs.edges,
        shells: shells.shells,
    };
    let (b, slots) = Builder::assemble(m, tolerance, assembly)?;
    let built = b.finish(m, BodyKind::Solid)?;
    verify(m, built.body)?;
    let provenance = record(
        &built,
        &slots,
        &specs.vertex_roles,
        &specs.edge_roles,
        &shells.face_roles,
        &shells.shell_roles,
        part,
    );
    Ok((built.body, provenance))
}
