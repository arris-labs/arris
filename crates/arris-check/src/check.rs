//! `check`: the closure walk and the rows it evaluates.
//!
//! Rows implemented here: M1–M3 (`references`, `finite`), V1–V3 (`vertex`)
//! and E1–E7 (`edge`), one file per family (`docs/DATA-MODEL.md`
//! §Invariants); the loop, face, shell and body rows are in
//! `crate::topology` and the `Full` rows in `crate::full`, which `level`
//! selects. Every row walks the body's closure in sorted id
//! order and reports through `Report::new`, so the report is the same on
//! every run and platform.

use std::collections::BTreeMap;

use arris_math::Precision;
use arris_topo::entity::{Coedge, Face};
use arris_topo::euler::EulerLine;
use arris_topo::{Body, Closure, CoedgeRef, EdgeId, FaceId, Model, NotFound, VertexId};

use crate::domain::FaceDomain;
use crate::report::Report;
use crate::unchecked::Unchecked;
use crate::violation::{Level, Reference, Violation};

mod edge;
mod finite;
mod references;
mod vertex;

/// Every violation of `docs/DATA-MODEL.md` §Invariants that `body`
/// exhibits in `model`, at `level` — `Fast` after every operation, `Full`
/// on demand — as a [`Report`] sorted by entity then row. A body handle
/// that does not resolve is one `M1` line naming the body. Never panics
/// and never repairs: a reference that does not resolve is reported once
/// under M1 and skipped by every other row.
///
/// ```
/// use arris_check::{Level, check};
/// use arris_debug::sample;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0).unwrap();
/// assert!(check(&m, body, Level::Fast).is_ok());
/// ```
pub fn check(model: &Model, body: Body, level: Level) -> Report {
    let Ok(mut c) = Checker::new(model, body) else {
        return Report::new(vec![Violation::Unresolved {
            from: body.id.into(),
            to: Reference::Entity(body.id.into()),
        }]);
    };
    c.references();
    c.indices();
    c.finiteness();
    c.vertex_rows();
    c.edge_rows();
    c.face_rows();
    c.shell_rows();
    c.body_rows();
    if level == Level::Full {
        c.full_rows();
    }
    // Linear in the closure, so it is taken at every level.
    let line = EulerLine::of(model, &c.closure);
    let unchecked = core::mem::take(&mut c.unchecked);
    let violations = c
        .violations
        .into_iter()
        .filter(|v| v.level() <= level)
        .collect();
    Report::new(violations)
        .with_euler(line)
        .with_unchecked(unchecked)
}

pub(crate) struct Checker<'m> {
    pub(crate) model: &'m Model,
    pub(crate) precision: Precision,
    pub(crate) body: Body,
    pub(crate) closure: Closure,
    /// Edge → its uses by the body's faces, in face creation order.
    pub(crate) uses: BTreeMap<EdgeId, Vec<(CoedgeRef, Coedge)>>,
    /// Vertex → the body's edges that end at it, each once.
    pub(crate) vertex_edges: BTreeMap<VertexId, Vec<EdgeId>>,
    pub(crate) violations: Vec<Violation>,
    /// Every face's domain at the model's parametric tolerance, built by
    /// the `Full` rows and empty at `Fast`.
    pub(crate) domains: BTreeMap<FaceId, FaceDomain<'m>>,
    /// The `Full` rows this body could not be decided on.
    pub(crate) unchecked: Vec<Unchecked>,
}

impl<'m> Checker<'m> {
    /// A checker over `body`'s closure with no row run yet: what [`check`]
    /// runs the rows on and `crate::lumps` runs B1's nesting on. Errors:
    /// the body handle does not resolve.
    pub(crate) fn new(model: &'m Model, body: Body) -> Result<Self, NotFound> {
        let closure = model.closure(body)?;
        // Adjacency is read off the closure's own entities, not the arena's
        // indices, so every row stands on its own and M2 is the only row
        // that says anything about the indices.
        let mut uses: BTreeMap<EdgeId, Vec<(CoedgeRef, Coedge)>> = BTreeMap::new();
        for &face_id in &closure.faces {
            let Ok(face) = model.face(face_id) else {
                continue;
            };
            for (loop_index, coedge_index, coedge) in coedges(face) {
                uses.entry(coedge.edge()).or_default().push((
                    CoedgeRef {
                        face: face_id,
                        loop_index,
                        coedge_index,
                    },
                    coedge,
                ));
            }
        }
        let mut vertex_edges: BTreeMap<VertexId, Vec<EdgeId>> = BTreeMap::new();
        for &edge_id in &closure.edges {
            let Ok(edge) = model.edge(edge_id) else {
                continue;
            };
            for v in [edge.start(), edge.end()] {
                let list = vertex_edges.entry(v).or_default();
                if list.last() != Some(&edge_id) {
                    list.push(edge_id);
                }
            }
        }
        Ok(Checker {
            model,
            precision: model.precision(),
            body,
            closure,
            uses,
            vertex_edges,
            violations: Vec::new(),
            domains: BTreeMap::new(),
            unchecked: Vec::new(),
        })
    }
}

/// `(face, loop index, coedge index, coedge)` for every coedge of a face.
pub(crate) fn coedges(face: &Face) -> impl Iterator<Item = (usize, usize, Coedge)> + '_ {
    face.loops().iter().enumerate().flat_map(|(li, l)| {
        l.coedges()
            .iter()
            .enumerate()
            .map(move |(ci, &c)| (li, ci, c))
    })
}

impl Checker<'_> {
    pub(crate) fn push(&mut self, v: Violation) {
        self.violations.push(v);
    }

    /// The uses of `edge` by faces of the body, in face creation order.
    fn uses_in_body(&self, edge: EdgeId) -> Vec<(CoedgeRef, Coedge)> {
        self.uses.get(&edge).cloned().unwrap_or_default()
    }
}
