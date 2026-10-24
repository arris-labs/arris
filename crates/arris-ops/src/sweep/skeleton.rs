//! What `extrude`, `revolve`, `sweep` and `loft` build their solids from
//! in common: the spec lists with the roles that go with them, and the
//! assemble → finish → verify → record tail. Each operation keeps what is
//! its own — which vertices exist, what curves its rails are, how its
//! sides close.

use arris_topo::builder::{Assembly, Builder, EdgeSpec, FaceSpec, VertexSpec};
use arris_topo::entity::BodyKind;
use arris_topo::provenance::SweepPart;
use arris_topo::{Body, Model, Provenance, Role};

use crate::error::OpError;
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

    /// Takes the faces and their roles, for a grouping of the operation's
    /// own.
    pub(super) fn take_faces(&mut self) -> (Vec<FaceSpec>, Vec<Role>) {
        (
            core::mem::take(&mut self.faces),
            core::mem::take(&mut self.face_roles),
        )
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
