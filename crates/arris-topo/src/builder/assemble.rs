use std::collections::{BTreeMap, BTreeSet};

use super::validate::{edge_uses, is_degenerate};
use super::{
    Assembly, AssemblySlots, BuildError, Builder, Counts, EdgeKey, EdgeRef, EdgeSpec, FaceRef,
    FaceSpec, StagedEdge, StagedFace, StagedLoop, StagedVertex, Use, VertexKey, VertexRef,
    VertexSpec, effective_uses,
};
#[cfg(doc)]
use crate::entity::BodyKind;
use crate::handle::{self};
use crate::id::{EdgeId, EntityKind, FaceId, VertexId};
use crate::model::Model;
use crate::orientation::Orientation;

/// The tables [`Builder::assemble`] resolves keys through.
#[derive(Default)]
struct Assembled {
    kept_vertices: BTreeMap<VertexId, VertexRef>,
    kept_edges: BTreeMap<EdgeId, EdgeRef>,
    kept_faces: BTreeMap<FaceId, FaceRef>,
    vertex_of: Vec<VertexRef>,
    edge_of: Vec<EdgeRef>,
}

impl Builder {
    /// A builder holding the body `assembly` describes: the builder's
    /// second entry point beside [`Builder::new`] and the operators, and
    /// the one an operation that computes its result's faces outright —
    /// a boolean, a sweep, a transform — uses (`docs/DATA-MODEL.md`
    /// §Euler operators, ADR-0004).
    ///
    /// Every entity is `Keep` or `New`. A `Keep` slot *is* the arena's
    /// entity: its geometry is read from `model`, [`Builder::finish`]
    /// appends nothing for it and returns its id, so an operation that
    /// leaves a face alone shares it with its input and its provenance
    /// records nothing. A kept face is kept whole — its loops, pcurves,
    /// edges and vertices come from the model, and the edges and vertices
    /// are kept with it. Any operator applied to a kept slot drops the
    /// mark, and `finish` appends that slot instead.
    ///
    /// What `assemble` proves, so that the result is a body each of whose
    /// shells the operators could have built: every loop has coedges and
    /// closes through effective vertices; every edge is used exactly twice
    /// and in opposite directions, or once when it is degenerate — a
    /// singular point, which the Euler–Poincaré line below does not count;
    /// no arena entity is kept twice; no shell
    /// is empty; no edge is used by, and no vertex is an end of edges of,
    /// two shells; the faces of each shell are one edge-connected
    /// component; and the Euler–Poincaré line of each shell closes at a
    /// whole genus, their sum becoming the builder's ([`Builder::counts`]).
    /// It does not prove how the shells nest — which one is outer and
    /// which a void inside it is geometry, the checker's B1.
    ///
    /// Errors, leaving nothing behind (the model is only read):
    /// [`BuildError::NoSpec`] for a key past its list;
    /// [`BuildError::NotFound`] for an id that does not resolve;
    /// [`BuildError::Empty`], [`BuildError::EmptyShell`],
    /// [`BuildError::Duplicate`], [`BuildError::EmptyLoop`],
    /// [`BuildError::LoopOpen`], [`BuildError::EdgeUses`],
    /// [`BuildError::SameDirection`], [`BuildError::SharedEdge`],
    /// [`BuildError::SharedVertex`], [`BuildError::Disconnected`],
    /// [`BuildError::NotClosed`].
    ///
    /// ```
    /// use arris_topo::builder::{Assembly, Builder, FaceSpec};
    /// use arris_topo::entity::BodyKind;
    /// use arris_topo::Model;
    /// use arris_debug::{dump_text, sample};
    ///
    /// // Every face of a body kept: the same entities, a new shell.
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
    /// let faces = m.faces(body)?.into_iter().map(FaceSpec::Keep).collect();
    /// let assembly = Assembly { shells: vec![faces], ..Assembly::default() };
    /// let (b, _slots) = Builder::assemble(&m, m.precision().default_tolerance, assembly)?;
    /// assert_eq!(b.counts().to_string(), "2/3/3/3/1 g0 = 0");
    /// let again = b.finish(&mut m, BodyKind::Solid)?;
    /// assert_eq!(m.faces(again.body)?, m.faces(body)?, "the same faces, kept");
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn assemble(
        model: &Model,
        tolerance: f64,
        assembly: Assembly,
    ) -> Result<(Builder, AssemblySlots), BuildError> {
        Builder::assemble_shells(model, tolerance, assembly, false)
    }

    /// [`Builder::assemble`] for a `Sheet`: each shell is an open surface,
    /// what [`Builder::finish`] stores with [`BodyKind::Sheet`]. It proves
    /// what `assemble` proves but that an edge other than a degenerate one
    /// is used once — a boundary edge, with a face on one side only — or
    /// twice in opposite directions, and that each shell's Euler–Poincaré
    /// line closes at a whole genus once every boundary cycle is capped by
    /// a face of one loop: `V − E + 2F − L + B = 2(1 − G)` over its `B`
    /// cycles of boundary edges. A boundary that touches itself at a
    /// vertex is one cycle there, so a shell pinched at a boundary vertex
    /// does not close. A planar section's faces, and any face kept whole
    /// on its own, are such shells.
    ///
    /// Errors: [`Builder::assemble`]'s, [`BuildError::NotClosed`]
    /// reporting the capped counts.
    ///
    /// ```
    /// use arris_topo::builder::{Assembly, Builder, FaceSpec};
    /// use arris_topo::entity::BodyKind;
    /// use arris_topo::Model;
    /// use arris_debug::sample;
    ///
    /// // A cylinder's top cap, kept whole as a sheet of its own.
    /// let mut m = Model::default();
    /// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
    /// let cap = m.faces(body)?[0];
    /// let assembly = Assembly { shells: vec![vec![FaceSpec::Keep(cap)]], ..Assembly::default() };
    /// let (b, _slots) = Builder::assemble_sheet(&m, m.precision().default_tolerance, assembly)?;
    /// let sheet = b.finish(&mut m, BodyKind::Sheet)?;
    /// assert_eq!(m.faces(sheet.body)?, [cap]);
    /// assert!(arris_check::check(&m, sheet.body, arris_check::Level::Full).is_ok());
    /// # Ok::<(), Box<dyn std::error::Error>>(())
    /// ```
    pub fn assemble_sheet(
        model: &Model,
        tolerance: f64,
        assembly: Assembly,
    ) -> Result<(Builder, AssemblySlots), BuildError> {
        Builder::assemble_shells(model, tolerance, assembly, true)
    }

    /// [`Builder::assemble`] of closed shells, or of `open` ones for
    /// [`Builder::assemble_sheet`].
    fn assemble_shells(
        model: &Model,
        tolerance: f64,
        assembly: Assembly,
        open: bool,
    ) -> Result<(Builder, AssemblySlots), BuildError> {
        let mut b = Builder::new(tolerance);
        let mut at = Assembled::default();
        for spec in &assembly.vertices {
            let slot = match *spec {
                VertexSpec::Keep(id) => b.keep_vertex(model, &mut at, id, true)?,
                VertexSpec::New { point, tolerance } => {
                    VertexRef(b.vertices.insert(StagedVertex {
                        point,
                        tolerance,
                        kept: None,
                    }))
                }
            };
            at.vertex_of.push(slot);
        }
        for spec in &assembly.edges {
            let slot = match *spec {
                EdgeSpec::Keep(id) => b.keep_edge(model, &mut at, id, true)?,
                EdgeSpec::New {
                    geometry,
                    start,
                    end,
                    tolerance,
                } => {
                    let start = b.vertex_key(model, &mut at, start)?;
                    let end = b.vertex_key(model, &mut at, end)?;
                    EdgeRef(b.edges.insert(StagedEdge {
                        geometry,
                        start,
                        end,
                        tolerance,
                        kept: None,
                    }))
                }
            };
            at.edge_of.push(slot);
        }
        let mut face_slots: Vec<Vec<FaceRef>> = Vec::with_capacity(assembly.shells.len());
        for (shell, specs) in assembly.shells.iter().enumerate() {
            if specs.is_empty() {
                return Err(BuildError::EmptyShell { shell });
            }
            let mut shell_slots = Vec::with_capacity(specs.len());
            for spec in specs {
                let slot = match spec {
                    FaceSpec::Keep(face) => b.keep_face(model, &mut at, *face, shell)?,
                    FaceSpec::New {
                        surface,
                        orientation,
                        loops,
                        tolerance,
                    } => {
                        model.surface(*surface)?;
                        let mut staged = Vec::with_capacity(loops.len());
                        for uses in loops {
                            let mut walk = Vec::with_capacity(uses.len());
                            for u in uses {
                                model.curve2(u.pcurve)?;
                                walk.push(Use {
                                    edge: b.edge_key(model, &mut at, u.edge)?,
                                    orientation: u.orientation,
                                    pcurve: Some(u.pcurve),
                                });
                            }
                            staged.push(b.staged_loop(walk, staged.len())?);
                        }
                        let mut face = StagedFace {
                            surface: *surface,
                            orientation: *orientation,
                            shell,
                            loops: staged,
                            tolerance: *tolerance,
                            kept: None,
                        };
                        face.canonicalise();
                        FaceRef(b.faces.insert(face))
                    }
                };
                shell_slots.push(slot);
            }
            face_slots.push(shell_slots);
        }
        b.genus = b.assembled_genus(open)?;
        let slots = AssemblySlots {
            vertices: at.vertex_of,
            edges: at.edge_of,
            faces: face_slots,
        };
        Ok((b, slots))
    }

    /// The slot of the kept vertex `id`, made on first mention. `named`
    /// marks the mention that is a [`VertexSpec::Keep`] of its own, which
    /// may not repeat one.
    fn keep_vertex(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        id: VertexId,
        named: bool,
    ) -> Result<VertexRef, BuildError> {
        if let Some(&slot) = at.kept_vertices.get(&id) {
            if named {
                return Err(BuildError::Duplicate(id.into()));
            }
            return Ok(slot);
        }
        let v = model.vertex(id)?;
        let slot = VertexRef(self.vertices.insert(StagedVertex {
            point: v.point(),
            tolerance: v.tolerance(),
            kept: Some(id),
        }));
        at.kept_vertices.insert(id, slot);
        Ok(slot)
    }

    /// The slot of the kept edge `id`, made on first mention together with
    /// its two vertices.
    fn keep_edge(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        id: EdgeId,
        named: bool,
    ) -> Result<EdgeRef, BuildError> {
        if let Some(&slot) = at.kept_edges.get(&id) {
            if named {
                return Err(BuildError::Duplicate(id.into()));
            }
            return Ok(slot);
        }
        let e = *model.edge(id)?;
        let start = self.keep_vertex(model, at, e.start(), false)?;
        let end = self.keep_vertex(model, at, e.end(), false)?;
        let slot = EdgeRef(self.edges.insert(StagedEdge {
            geometry: e.geometry(),
            start,
            end,
            tolerance: e.tolerance(),
            kept: Some(id),
        }));
        at.kept_edges.insert(id, slot);
        Ok(slot)
    }

    /// The slot of the kept face `face` in shell `shell`, with every edge
    /// and vertex it names kept too. A stored loop of a `Reversed` face is
    /// walked backwards with every use flipped — the inverse of what
    /// [`Builder::finish`] stores.
    fn keep_face(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        face: handle::Face,
        shell: usize,
    ) -> Result<FaceRef, BuildError> {
        if at.kept_faces.contains_key(&face.id) {
            return Err(BuildError::Duplicate(face.id.into()));
        }
        let entity = model.face(face.id)?.clone();
        model.surface(entity.surface())?;
        let mut loops = Vec::with_capacity(entity.loops().len());
        for l in entity.loops() {
            let raw = l
                .coedges()
                .iter()
                .map(|c| (c.edge(), c.orientation(), c.pcurve()));
            let mut walk = Vec::with_capacity(l.coedges().len());
            for (edge, orientation, pcurve) in effective_uses(face.orientation, raw) {
                model.curve2(pcurve)?;
                walk.push(Use {
                    edge: self.keep_edge(model, at, edge, false)?,
                    orientation,
                    pcurve: Some(pcurve),
                });
            }
            let index = loops.len();
            loops.push(self.staged_loop(walk, index)?);
        }
        let mut staged = StagedFace {
            surface: entity.surface(),
            orientation: face.orientation,
            shell,
            loops,
            tolerance: entity.tolerance(),
            kept: Some(face.id),
        };
        staged.canonicalise();
        let slot = FaceRef(self.faces.insert(staged));
        at.kept_faces.insert(face.id, slot);
        Ok(slot)
    }

    /// The loop of `walk`, seeded at the effective start of its first use.
    /// `loop_index` is only what an error names, the face being the one
    /// about to be inserted. Errors: the walk is empty — an assembled loop
    /// is a walk, never the bare vertex [`Builder::mvfs`] leaves behind.
    fn staged_loop(&self, walk: Vec<Use>, loop_index: usize) -> Result<StagedLoop, BuildError> {
        let first = walk.first().ok_or(BuildError::EmptyLoop {
            face: FaceRef(self.faces.next_slot()),
            loop_index,
        })?;
        let seed = self.use_start(first)?;
        Ok(StagedLoop { uses: walk, seed })
    }

    fn vertex_key(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        key: VertexKey,
    ) -> Result<VertexRef, BuildError> {
        match key {
            VertexKey::Kept(id) => self.keep_vertex(model, at, id, false),
            VertexKey::New(index) => at.vertex_of.get(index).copied().ok_or(BuildError::NoSpec {
                kind: EntityKind::Vertex,
                index,
            }),
        }
    }

    fn edge_key(
        &mut self,
        model: &Model,
        at: &mut Assembled,
        key: EdgeKey,
    ) -> Result<EdgeRef, BuildError> {
        match key {
            EdgeKey::Kept(id) => self.keep_edge(model, at, id, false),
            EdgeKey::New(index) => at.edge_of.get(index).copied().ok_or(BuildError::NoSpec {
                kind: EntityKind::Edge,
                index,
            }),
        }
    }

    /// The effective end vertex of `u`: the start of the same use walked
    /// the other way.
    fn use_end(&self, u: &Use) -> Result<VertexRef, BuildError> {
        let e = self.edge(u.edge)?;
        Ok(match u.orientation {
            Orientation::Forward => e.end,
            Orientation::Reversed => e.start,
        })
    }

    /// Proves the assembled slots are closed surfaces of whole genus — or,
    /// `open`, surfaces that are once their boundary cycles are capped —
    /// one per shell and sharing nothing, and returns the sum of their
    /// genera; the errors are [`Builder::assemble`]'s.
    fn assembled_genus(&self, open: bool) -> Result<usize, BuildError> {
        let mut uses: BTreeMap<EdgeRef, Vec<(Orientation, FaceRef)>> = BTreeMap::new();
        for (fi, face) in self.faces.iter() {
            let face_ref = FaceRef(fi);
            for (li, lp) in face.loops.iter().enumerate() {
                if lp.uses.is_empty() {
                    return Err(BuildError::EmptyLoop {
                        face: face_ref,
                        loop_index: li,
                    });
                }
                let n = lp.uses.len();
                for (ci, u) in lp.uses.iter().enumerate() {
                    let previous = &lp.uses[(ci + n - 1) % n];
                    let (ended, starts) = (self.use_end(previous)?, self.use_start(u)?);
                    if ended != starts {
                        return Err(BuildError::LoopOpen {
                            face: face_ref,
                            loop_index: li,
                            coedge_index: ci,
                            ended,
                            starts,
                        });
                    }
                    uses.entry(u.edge)
                        .or_default()
                        .push((u.orientation, face_ref));
                }
            }
        }
        for (ei, e) in self.edges.iter() {
            let edge = EdgeRef(ei);
            let list = uses.get(&edge).map_or(&[][..], Vec::as_slice);
            edge_uses(edge, e, list, open, |u| u.0)?;
        }
        if self.faces.len() == 0 {
            return Err(BuildError::Empty);
        }
        let shell_of = |f: FaceRef| self.faces.get(f.0).map(|face| face.shell);
        // Every edge in the shell of its two faces, and every vertex in the
        // shell of its edges: two shells share nothing.
        let mut edge_shell: BTreeMap<EdgeRef, usize> = BTreeMap::new();
        for (&edge, list) in &uses {
            let shells: Vec<usize> = list.iter().filter_map(|&(_, f)| shell_of(f)).collect();
            if let [a, b] = shells[..] {
                if a != b {
                    return Err(BuildError::SharedEdge {
                        edge,
                        shells: [a.min(b), a.max(b)],
                    });
                }
            }
            if let Some(&s) = shells.first() {
                edge_shell.insert(edge, s);
            }
        }
        let mut vertex_shell: BTreeMap<VertexRef, usize> = BTreeMap::new();
        for (ei, e) in self.edges.iter() {
            let Some(&s) = edge_shell.get(&EdgeRef(ei)) else {
                continue;
            };
            for v in [e.start, e.end] {
                match vertex_shell.get(&v) {
                    Some(&t) if t != s => {
                        return Err(BuildError::SharedVertex {
                            vertex: v,
                            shells: [t.min(s), t.max(s)],
                        });
                    }
                    _ => {
                        vertex_shell.insert(v, s);
                    }
                }
            }
        }
        let shells: BTreeSet<usize> = self.faces.iter().map(|(_, f)| f.shell).collect();
        let mut genus = 0;
        for shell in shells {
            let faces: Vec<FaceRef> = self
                .faces
                .iter()
                .filter(|(_, f)| f.shell == shell)
                .map(|(fi, _)| FaceRef(fi))
                .collect();
            let Some(&from) = faces.first() else {
                continue;
            };
            let mut reached: BTreeSet<FaceRef> = BTreeSet::new();
            reached.insert(from);
            let mut front = vec![from];
            while let Some(f) = front.pop() {
                let Some(face) = self.faces.get(f.0) else {
                    continue;
                };
                for u in face.loops.iter().flat_map(|lp| lp.uses.iter()) {
                    for &(_, other) in uses.get(&u.edge).map_or(&[][..], Vec::as_slice) {
                        if reached.insert(other) {
                            front.push(other);
                        }
                    }
                }
            }
            if let Some(&face) = faces.iter().find(|f| !reached.contains(f)) {
                return Err(BuildError::Disconnected { face, from });
            }
            // An open shell's boundary cycles, each capped below by a face
            // of one loop.
            let caps = if open {
                self.boundary_cycles(&uses, &edge_shell, shell)
            } else {
                0
            };
            let counts = Counts {
                vertices: vertex_shell.values().filter(|&&s| s == shell).count(),
                edges: edge_shell
                    .iter()
                    .filter(|&(&e, &s)| {
                        s == shell && !self.edges.get(e.0).is_some_and(is_degenerate)
                    })
                    .count(),
                faces: faces.len() + caps,
                loops: faces
                    .iter()
                    .filter_map(|f| self.faces.get(f.0))
                    .map(|f| f.loops.len())
                    .sum::<usize>()
                    + caps,
                shells: 1,
                genus: 0,
            };
            // The shell's line at S = 1 closes at a whole genus, or the
            // faces are no closed surface.
            let line = counts.line();
            if !line.closes() || line.genus < 0 {
                return Err(BuildError::NotClosed { counts });
            }
            genus += line.genus as usize;
        }
        // A vertex no edge ends at is on no shell, and the body's counts
        // are then not the sum of its closed shells'.
        if vertex_shell.len() != self.vertices.len() {
            return Err(BuildError::NotClosed {
                counts: self.counts(),
            });
        }
        Ok(genus)
    }
}

impl Builder {
    /// The cycles of boundary edges of `shell` — the non-degenerate edges
    /// one coedge uses — counted as the components of the graph they make
    /// over their vertices: one per cycle where each boundary vertex ends
    /// two of them, fewer where a boundary touches itself.
    fn boundary_cycles(
        &self,
        uses: &BTreeMap<EdgeRef, Vec<(Orientation, FaceRef)>>,
        edge_shell: &BTreeMap<EdgeRef, usize>,
        shell: usize,
    ) -> usize {
        // Not `arris_math::DisjointSets`: the slots are sparse (a killed vertex leaves a hole) and the sets span only the boundary's own vertices.
        let mut parent: BTreeMap<VertexRef, VertexRef> = BTreeMap::new();
        fn root(parent: &mut BTreeMap<VertexRef, VertexRef>, v: VertexRef) -> VertexRef {
            let mut r = v;
            while let Some(&p) = parent.get(&r) {
                if p == r {
                    break;
                }
                r = p;
            }
            parent.insert(v, r);
            r
        }
        for (&edge, list) in uses {
            let boundary = list.len() == 1
                && edge_shell.get(&edge) == Some(&shell)
                && self.edges.get(edge.0).is_some_and(|e| !is_degenerate(e));
            let Some(e) = self.edges.get(edge.0).filter(|_| boundary) else {
                continue;
            };
            for v in [e.start, e.end] {
                parent.entry(v).or_insert(v);
            }
            let (a, b) = (root(&mut parent, e.start), root(&mut parent, e.end));
            if a != b {
                parent.insert(a.max(b), a.min(b));
            }
        }
        let vertices: Vec<VertexRef> = parent.keys().copied().collect();
        vertices
            .into_iter()
            .filter(|&v| root(&mut parent, v) == v)
            .count()
    }
}
