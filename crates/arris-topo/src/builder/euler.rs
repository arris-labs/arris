use super::{
    BuildError, Builder, EdgeRef, FaceRef, Join, Position, Seed, Split, StagedEdge, StagedFace,
    StagedLoop, StagedVertex, Strut, Use, VertexRef,
};
use crate::entity::EdgeGeometry;
use crate::id::Curve2Id;
use crate::orientation::Orientation;

/// The uses from `from` up to but not including `to`, around the loop:
/// nothing when they are equal, `uses[from..]` then `uses[..to]` when
/// `from` is past `to`.
pub(super) fn around(uses: &[Use], from: usize, to: usize) -> Vec<Use> {
    let n = uses.len();
    let (from, to) = (from.min(n), to.min(n));
    if from < to {
        uses[from..to].to_vec()
    } else if from > to {
        let mut v = uses[from..].to_vec();
        v.extend_from_slice(&uses[..to]);
        v
    } else {
        Vec::new()
    }
}

impl Builder {
    /// The vertex at a junction: the effective start of the coedge there,
    /// or the seed of a loop without coedges. Errors: the position does
    /// not resolve or is past the loop's last junction.
    pub fn vertex_at(&self, at: Position) -> Result<VertexRef, BuildError> {
        let lp = self.loop_ref(at.face, at.loop_index)?;
        let n = lp.uses.len();
        if at.coedge_index > n {
            return Err(BuildError::BadPosition {
                position: at,
                len: n,
            });
        }
        if n == 0 {
            return Ok(lp.seed);
        }
        self.use_start(&lp.uses[at.coedge_index % n])
    }

    /// The one junction of the loop where `vertex` stands. Errors: it is
    /// at none, or at several (a seam vertex, a closed edge's vertex) —
    /// then the caller says which through [`Builder::vertex_at`].
    pub fn find_position(
        &self,
        face: FaceRef,
        loop_index: usize,
        vertex: VertexRef,
    ) -> Result<Position, BuildError> {
        let lp = self.loop_ref(face, loop_index)?;
        let n = lp.uses.len();
        let mut positions = Vec::new();
        if n == 0 {
            if lp.seed == vertex {
                positions.push(0);
            }
        } else {
            for (i, u) in lp.uses.iter().enumerate() {
                if self.use_start(u)? == vertex {
                    positions.push(i);
                }
            }
        }
        match positions.as_slice() {
            [i] => Ok(Position::new(face, loop_index, *i)),
            [] => Err(BuildError::NotInLoop {
                vertex,
                face,
                loop_index,
            }),
            _ => Err(BuildError::Ambiguous {
                vertex,
                face,
                loop_index,
                positions,
            }),
        }
    }

    /// Every use of `e`: `(face, loop, coedge, orientation)`, in face slot
    /// order.
    fn uses_of(&self, e: EdgeRef) -> Vec<(FaceRef, usize, usize, Orientation)> {
        let mut out = Vec::new();
        for (fi, face) in self.faces.iter() {
            for (li, lp) in face.loops.iter().enumerate() {
                for (ci, u) in lp.uses.iter().enumerate() {
                    if u.edge == e {
                        out.push((FaceRef(fi), li, ci, u.orientation));
                    }
                }
            }
        }
        out
    }

    /// Where the use `(edge, orientation)` is in `f`, after
    /// canonicalisation.
    fn locate(&self, f: FaceRef, key: (EdgeRef, Orientation)) -> Option<(usize, usize)> {
        let face = self.faces.get(f.0)?;
        for (li, lp) in face.loops.iter().enumerate() {
            if let Some(ci) = lp.uses.iter().position(|u| u.key() == key) {
                return Some((li, ci));
            }
        }
        None
    }

    /// The index of the loop of `f` that has no uses and stands at `seed`.
    fn empty_loop(&self, f: FaceRef, seed: VertexRef) -> Option<usize> {
        self.faces
            .get(f.0)?
            .loops
            .iter()
            .position(|lp| lp.uses.is_empty() && lp.seed == seed)
    }

    /// Puts `f`'s loops back in canonical rotation and order after an
    /// operator changed them, and drops the face's `Keep` mark: a face an
    /// operator has touched is no longer the arena's face, so `finish`
    /// appends it (`docs/DATA-MODEL.md` §Euler operators).
    fn canonicalise(&mut self, f: FaceRef) {
        if let Some(face) = self.faces.get_mut(f.0) {
            face.canonicalise();
            face.kept = None;
        }
    }

    /// The position of the use `(edge, orientation)` in `f`, or of the
    /// empty loop at `seed` when there is no such use. Both exist by
    /// construction after the kills that call this.
    fn position_after(
        &self,
        f: FaceRef,
        key: Option<(EdgeRef, Orientation)>,
        seed: VertexRef,
    ) -> Result<Position, BuildError> {
        let (li, ci) = key
            .and_then(|k| self.locate(f, k))
            .or_else(|| self.empty_loop(f, seed).map(|li| (li, 0)))
            .ok_or(BuildError::Invariant { face: f })?;
        Ok(Position::new(f, li, ci))
    }

    /// **mvfs** — make vertex, face, shell: the first vertex of a body, a
    /// face with one loop of no coedges standing at it, and the shell.
    /// Returns the vertex and the face; the loop is `Position::new(face,
    /// 0, 0)`. Errors: the builder already holds a body.
    pub fn mvfs(&mut self, seed: Seed) -> Result<(VertexRef, FaceRef), BuildError> {
        if !self.is_empty() {
            return Err(BuildError::NotEmpty);
        }
        let v = VertexRef(self.vertices.insert(StagedVertex {
            point: seed.point,
            tolerance: self.tolerance,
            kept: None,
        }));
        let f = FaceRef(self.faces.insert(StagedFace {
            surface: seed.surface,
            orientation: seed.orientation,
            shell: 0,
            loops: vec![StagedLoop {
                uses: Vec::new(),
                seed: v,
            }],
            tolerance: self.tolerance,
            kept: None,
        }));
        Ok((v, f))
    }

    /// **kvfs** — kill vertex, face, shell: the inverse of [`Builder::mvfs`],
    /// leaving the builder empty. Errors: the builder holds anything but
    /// one vertex and one face of one loop of no coedges.
    pub fn kvfs(&mut self) -> Result<Seed, BuildError> {
        let seed = match (
            self.vertices.iter().next(),
            self.faces.iter().next(),
            self.vertices.len(),
            self.edges.len(),
            self.faces.len(),
        ) {
            (Some((_, v)), Some((_, f)), 1, 0, 1)
                if f.loops.len() == 1 && f.loops[0].uses.is_empty() =>
            {
                Seed {
                    point: v.point,
                    surface: f.surface,
                    orientation: f.orientation,
                }
            }
            _ => return Err(BuildError::NotASeed),
        };
        self.vertices.clear();
        self.edges.clear();
        self.faces.clear();
        self.genus = 0;
        Ok(seed)
    }

    /// **mev** — make edge, vertex: a new vertex and the edge from the
    /// vertex at `at` to it, inserted at `at` as two consecutive uses —
    /// `Forward` away, `Reversed` back. Returns the vertex and the edge.
    /// Errors: `at` does not resolve; the strut is degenerate.
    pub fn mev(&mut self, at: Position, strut: Strut) -> Result<(VertexRef, EdgeRef), BuildError> {
        if matches!(strut.geometry, EdgeGeometry::Degenerate { .. }) {
            return Err(BuildError::DegenerateStrut);
        }
        let start = self.vertex_at(at)?;
        let v = VertexRef(self.vertices.insert(StagedVertex {
            point: strut.point,
            tolerance: self.tolerance,
            kept: None,
        }));
        let e = EdgeRef(self.edges.insert(StagedEdge {
            geometry: strut.geometry,
            start,
            end: v,
            tolerance: self.tolerance,
            kept: None,
        }));
        let lp = self.loop_mut(at.face, at.loop_index)?;
        let i = at.coedge_index.min(lp.uses.len());
        lp.uses.splice(
            i..i,
            [
                Use {
                    edge: e,
                    orientation: Orientation::Forward,
                    pcurve: strut.pcurves[0],
                },
                Use {
                    edge: e,
                    orientation: Orientation::Reversed,
                    pcurve: strut.pcurves[1],
                },
            ],
        );
        self.canonicalise(at.face);
        Ok((v, e))
    }

    /// **kev** — kill edge, vertex: the inverse of [`Builder::mev`]. The
    /// edge must be a strut — used twice in a row by one loop, `Forward`
    /// then `Reversed` — and its end vertex on nothing else; both are
    /// removed. Returns the position and the strut that remake them.
    /// Errors: not a strut; the end vertex is used by another edge or
    /// seeds an empty loop.
    pub fn kev(&mut self, edge: EdgeRef) -> Result<(Position, Strut), BuildError> {
        let e = *self.edge(edge)?;
        let uses = self.uses_of(edge);
        let (f, li, i) = match uses.as_slice() {
            [
                (fa, la, i, Orientation::Forward),
                (fb, lb, j, Orientation::Reversed),
            ]
            | [
                (fb, lb, j, Orientation::Reversed),
                (fa, la, i, Orientation::Forward),
            ] if fa == fb && la == lb => {
                let n = self.loop_ref(*fa, *la)?.uses.len();
                if (*i + 1) % n != *j || e.start == e.end {
                    return Err(BuildError::NotAStrut { edge });
                }
                (*fa, *la, *i)
            }
            _ => return Err(BuildError::NotAStrut { edge }),
        };
        let tip = e.end;
        for (k, other) in self.edges.iter() {
            if k != edge.0 && (other.start == tip || other.end == tip) {
                return Err(BuildError::VertexInUse {
                    vertex: tip,
                    other: EdgeRef(k),
                });
            }
        }
        for (fi, face) in self.faces.iter() {
            for (l, lp) in face.loops.iter().enumerate() {
                if lp.uses.is_empty() && lp.seed == tip {
                    return Err(BuildError::VertexSeeds {
                        vertex: tip,
                        face: FaceRef(fi),
                        loop_index: l,
                    });
                }
            }
        }
        let point = self.vertex(tip)?.point;
        let lp = self.loop_mut(f, li)?;
        let n = lp.uses.len();
        let j = (i + 1) % n;
        let following = (n > 2).then(|| lp.uses[(i + 2) % n].key());
        let (p0, p1) = (lp.uses[i].pcurve, lp.uses[j].pcurve);
        let (first, second) = if j > i { (j, i) } else { (i, j) };
        lp.uses.remove(first);
        lp.uses.remove(second);
        if lp.uses.is_empty() {
            lp.seed = e.start;
        }
        self.vertices.remove(tip.0);
        self.edges.remove(edge.0);
        self.canonicalise(f);
        let at = self.position_after(f, following, e.start)?;
        Ok((
            at,
            Strut {
                point,
                geometry: e.geometry,
                pcurves: [p0, p1],
            },
        ))
    }

    /// **mef** — make edge, face: an edge from the vertex at `from` to the
    /// vertex at `to`, both junctions of one loop, splitting it. The new
    /// face is on the edge's left: its loop is the edge `Forward` then the
    /// coedges from `to` around to `from`; the old loop keeps the rest
    /// after the edge `Reversed`. With `from == to` the new face is the
    /// edge alone (a closed edge splitting off a cap); with `from = to +
    /// len` it takes every coedge, the old face keeping the edge alone.
    /// Coedges that move keep their pcurve ids, which are on the old
    /// surface — [`Builder::set_pcurve`] is the caller's next call unless
    /// the surfaces are one. Returns the edge and the new face. Errors:
    /// the positions are not in one loop; `from` is past the loop and not
    /// `to + len`; a degenerate edge between two vertices.
    pub fn mef(
        &mut self,
        from: Position,
        to: Position,
        split: Split,
    ) -> Result<(EdgeRef, FaceRef), BuildError> {
        if from.face != to.face || from.loop_index != to.loop_index {
            return Err(BuildError::NotOneLoop { from, to });
        }
        let n = self.loop_ref(from.face, from.loop_index)?.uses.len();
        let (a, b) = (from.coedge_index, to.coedge_index);
        let full_turn = n > 0 && a == b + n;
        if a > n && !full_turn {
            return Err(BuildError::BadPosition {
                position: from,
                len: n,
            });
        }
        let end = self.vertex_at(to)?;
        let start = if full_turn {
            end
        } else {
            self.vertex_at(from)?
        };
        if matches!(split.geometry, EdgeGeometry::Degenerate { .. }) && start != end {
            return Err(BuildError::DegenerateEnds { start, end });
        }
        let shell = self.face(from.face)?.shell;
        let e = EdgeRef(self.edges.insert(StagedEdge {
            geometry: split.geometry,
            start,
            end,
            tolerance: self.tolerance,
            kept: None,
        }));
        let tolerance = self.tolerance;
        let lp = self.loop_mut(from.face, from.loop_index)?;
        let mut new_uses = vec![Use {
            edge: e,
            orientation: Orientation::Forward,
            pcurve: split.pcurves[0],
        }];
        let mut old_uses = vec![Use {
            edge: e,
            orientation: Orientation::Reversed,
            pcurve: split.pcurves[1],
        }];
        if full_turn {
            // Every coedge moves, the edge going in at the junction.
            let b = b.min(n);
            new_uses.extend_from_slice(&lp.uses[b..]);
            new_uses.extend_from_slice(&lp.uses[..b]);
        } else if a == b {
            // Every coedge stays, the edge going in at the junction.
            let a = a.min(n);
            old_uses.extend_from_slice(&lp.uses[a..]);
            old_uses.extend_from_slice(&lp.uses[..a]);
        } else {
            new_uses.extend(around(&lp.uses, b, a));
            old_uses.extend(around(&lp.uses, a, b));
        }
        lp.uses = old_uses;
        let f = FaceRef(self.faces.insert(StagedFace {
            surface: split.surface,
            orientation: split.orientation,
            shell,
            loops: vec![StagedLoop {
                uses: new_uses,
                seed: end,
            }],
            tolerance,
            kept: None,
        }));
        self.canonicalise(from.face);
        self.canonicalise(f);
        Ok((e, f))
    }

    /// **kef** — kill edge, face: the inverse of [`Builder::mef`]. The
    /// edge must be used once by each of two faces; the face on its left
    /// (the `Forward` use), which must have one loop, is removed and its
    /// other coedges take the edge's place in the other face's loop.
    /// Returns the two positions and the split that remake them. Errors:
    /// the edge does not separate two faces; the face to kill has rings.
    pub fn kef(&mut self, edge: EdgeRef) -> Result<(Position, Position, Split), BuildError> {
        let e = *self.edge(edge)?;
        let uses = self.uses_of(edge);
        let ((fa, la, i), (fb, lb, j)) = match uses.as_slice() {
            [
                (fa, la, i, Orientation::Forward),
                (fb, lb, j, Orientation::Reversed),
            ]
            | [
                (fb, lb, j, Orientation::Reversed),
                (fa, la, i, Orientation::Forward),
            ] if fa != fb => ((*fa, *la, *i), (*fb, *lb, *j)),
            _ => return Err(BuildError::NotSeparating { edge }),
        };
        let loops = self.face(fa)?.loops.len();
        if loops != 1 {
            return Err(BuildError::Rings { face: fa, loops });
        }
        let Some(killed) = self.faces.remove(fa.0) else {
            return Err(BuildError::NoFace(fa));
        };
        let mut spliced = killed.loops[la].uses.clone();
        spliced.rotate_left(i);
        let plus = spliced.remove(0);
        let first_spliced = spliced.first().map(Use::key);
        let count = spliced.len();
        let lp = self.loop_mut(fb, lb)?;
        let n = lp.uses.len();
        let minus = lp.uses[j];
        let following = (n > 1).then(|| lp.uses[(j + 1) % n].key());
        lp.uses.splice(j..=j, spliced);
        if lp.uses.is_empty() {
            lp.seed = e.start;
        }
        self.edges.remove(edge.0);
        self.canonicalise(fb);
        let to = self.position_after(fb, first_spliced.or(following), e.start)?;
        let n = self.loop_ref(fb, to.loop_index)?.uses.len();
        let mut from = to;
        if first_spliced.is_some() {
            if count == n {
                // The spliced block is the whole loop: `from = to + n` is
                // the split that moves every coedge back from there.
                from.coedge_index = to.coedge_index + n;
            } else {
                from.coedge_index = (to.coedge_index + count) % n;
            }
        }
        Ok((
            from,
            to,
            Split {
                geometry: e.geometry,
                surface: killed.surface,
                orientation: killed.orientation,
                pcurves: [plus.pcurve, minus.pcurve],
            },
        ))
    }

    /// **mekr** — make edge, kill ring: an edge from the vertex at `from`
    /// to the vertex at `to`, junctions of two loops of one face, joining
    /// them into one loop — `from`'s loop up to `from`, the edge
    /// `Forward`, all of `to`'s loop from `to` around, the edge
    /// `Reversed`, the rest of `from`'s loop. Returns the edge. Errors: the
    /// positions are not two loops of one face; a degenerate edge between
    /// two vertices.
    pub fn mekr(
        &mut self,
        from: Position,
        to: Position,
        join: Join,
    ) -> Result<EdgeRef, BuildError> {
        if from.face != to.face || from.loop_index == to.loop_index {
            return Err(BuildError::NotTwoLoops { from, to });
        }
        let start = self.vertex_at(from)?;
        let end = self.vertex_at(to)?;
        if matches!(join.geometry, EdgeGeometry::Degenerate { .. }) && start != end {
            return Err(BuildError::DegenerateEnds { start, end });
        }
        let e = EdgeRef(self.edges.insert(StagedEdge {
            geometry: join.geometry,
            start,
            end,
            tolerance: self.tolerance,
            kept: None,
        }));
        let face = self.face_mut(from.face)?;
        let ring = face.loops.remove(to.loop_index);
        let la = if to.loop_index < from.loop_index {
            from.loop_index - 1
        } else {
            from.loop_index
        };
        let lp = &mut face.loops[la];
        let a = from.coedge_index.min(lp.uses.len());
        let b = to.coedge_index.min(ring.uses.len());
        let mut merged = lp.uses[..a].to_vec();
        merged.push(Use {
            edge: e,
            orientation: Orientation::Forward,
            pcurve: join.pcurves[0],
        });
        merged.extend_from_slice(&ring.uses[b..]);
        merged.extend_from_slice(&ring.uses[..b]);
        merged.push(Use {
            edge: e,
            orientation: Orientation::Reversed,
            pcurve: join.pcurves[1],
        });
        merged.extend_from_slice(&lp.uses[a..]);
        lp.uses = merged;
        self.canonicalise(from.face);
        Ok(e)
    }

    /// **kemr** — kill edge, make ring: the inverse of [`Builder::mekr`].
    /// The edge must be used twice by one loop; removing it splits the
    /// loop at its two uses, the coedges between the `Forward` and the
    /// `Reversed` use becoming a second loop of the face (a ring, possibly
    /// a lone vertex). Returns the two positions and the join that remake
    /// them. Errors: the edge is not used twice by one loop.
    pub fn kemr(&mut self, edge: EdgeRef) -> Result<(Position, Position, Join), BuildError> {
        let e = *self.edge(edge)?;
        let uses = self.uses_of(edge);
        let (f, li, i, j) = match uses.as_slice() {
            [
                (fa, la, i, Orientation::Forward),
                (fb, lb, j, Orientation::Reversed),
            ]
            | [
                (fb, lb, j, Orientation::Reversed),
                (fa, la, i, Orientation::Forward),
            ] if fa == fb && la == lb => (*fa, *la, *i, *j),
            _ => return Err(BuildError::NotInOneLoop { edge }),
        };
        let lp = self.loop_mut(f, li)?;
        let n = lp.uses.len();
        let (plus, minus) = (lp.uses[i], lp.uses[j]);
        let ring = around(&lp.uses, (i + 1) % n, j);
        let rest = around(&lp.uses, (j + 1) % n, i);
        let (ring_first, rest_first) = (ring.first().map(Use::key), rest.first().map(Use::key));
        lp.uses = rest;
        if lp.uses.is_empty() {
            lp.seed = e.start;
        }
        self.face_mut(f)?.loops.push(StagedLoop {
            uses: ring,
            seed: e.end,
        });
        self.edges.remove(edge.0);
        self.canonicalise(f);
        let from = self.position_after(f, rest_first, e.start)?;
        let to = self.position_after(f, ring_first, e.end)?;
        Ok((
            from,
            to,
            Join {
                geometry: e.geometry,
                pcurves: [plus.pcurve, minus.pcurve],
            },
        ))
    }

    /// **kfmrh** — kill face, make ring, hole: `kill`, a face of one loop
    /// on the same surface as `into` and used with the opposite
    /// orientation (two coplanar faces with opposing outward normals and
    /// nothing between them), is removed and its loop becomes a ring of
    /// `into`, keeping its uses and pcurves; the body gains a handle.
    /// Returns the ring's index in `into`. Errors: the faces are one; the
    /// surfaces or orientations do not match; `kill` has rings.
    pub fn kfmrh(&mut self, kill: FaceRef, into: FaceRef) -> Result<usize, BuildError> {
        let k = self.face(kill)?;
        let t = self.face(into)?;
        if kill == into {
            return Err(BuildError::OneFace { face: kill });
        }
        if k.surface != t.surface {
            return Err(BuildError::SurfaceMismatch {
                kill,
                kill_surface: k.surface,
                into,
                into_surface: t.surface,
            });
        }
        if k.orientation == t.orientation {
            return Err(BuildError::SameOrientation { kill, into });
        }
        if k.loops.len() != 1 {
            return Err(BuildError::Rings {
                face: kill,
                loops: k.loops.len(),
            });
        }
        let Some(killed) = self.faces.remove(kill.0) else {
            return Err(BuildError::NoFace(kill));
        };
        let ring = killed.loops.into_iter().next().ok_or(BuildError::Rings {
            face: kill,
            loops: 0,
        })?;
        let key = ring.uses.first().map(Use::key);
        let seed = ring.seed;
        self.face_mut(into)?.loops.push(ring);
        self.genus += 1;
        self.canonicalise(into);
        Ok(self.position_after(into, key, seed)?.loop_index)
    }

    /// **mfkrh** — make face, kill ring, hole: the inverse of
    /// [`Builder::kfmrh`]. Loop `ring` of `face` is lifted out as the one
    /// loop of a new face on the same surface with the opposite
    /// orientation; the body loses a handle. Returns the new face. Errors:
    /// `face` has no such ring or no other loop; the genus is zero.
    pub fn mfkrh(&mut self, face: FaceRef, ring: usize) -> Result<FaceRef, BuildError> {
        let f = self.face(face)?;
        if f.loops.len() < 2 || ring >= f.loops.len() {
            return Err(BuildError::NoRing { face, ring });
        }
        if self.genus == 0 {
            return Err(BuildError::NoHandle);
        }
        let (surface, orientation, tolerance, shell) =
            (f.surface, f.orientation, f.tolerance, f.shell);
        let lifted = self.face_mut(face)?.loops.remove(ring);
        let new = FaceRef(self.faces.insert(StagedFace {
            surface,
            orientation: orientation.flipped(),
            shell,
            loops: vec![lifted],
            tolerance,
            kept: None,
        }));
        self.genus -= 1;
        self.canonicalise(face);
        self.canonicalise(new);
        Ok(new)
    }

    /// Gives the coedge at `at` its pcurve on the face's surface,
    /// returning the previous one. Errors: `at` does not name a coedge.
    pub fn set_pcurve(
        &mut self,
        at: Position,
        pcurve: Curve2Id,
    ) -> Result<Option<Curve2Id>, BuildError> {
        let lp = self.loop_mut(at.face, at.loop_index)?;
        let n = lp.uses.len();
        let Some(u) = lp.uses.get_mut(at.coedge_index) else {
            return Err(BuildError::BadPosition {
                position: at,
                len: n,
            });
        };
        let previous = u.pcurve.replace(pcurve);
        if let Some(face) = self.faces.get_mut(at.face.0) {
            face.kept = None;
        }
        Ok(previous)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn around_is_the_cyclic_half_open_range() {
        let u = |i: u32| Use {
            edge: EdgeRef(i),
            orientation: Orientation::Forward,
            pcurve: None,
        };
        let uses = [u(0), u(1), u(2), u(3)];
        let edges = |v: Vec<Use>| v.iter().map(|u| u.edge.0).collect::<Vec<_>>();
        assert_eq!(edges(around(&uses, 1, 3)), [1, 2]);
        assert_eq!(edges(around(&uses, 3, 1)), [3, 0]);
        assert_eq!(edges(around(&uses, 2, 2)), Vec::<u32>::new());
        assert_eq!(edges(around(&uses, 0, 4)), [0, 1, 2, 3]);
        assert_eq!(edges(around(&uses, 4, 0)), Vec::<u32>::new());
        assert_eq!(edges(around(&[], 0, 0)), Vec::<u32>::new());
    }
}
