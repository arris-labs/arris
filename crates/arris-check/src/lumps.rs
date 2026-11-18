//! Lumps (`docs/DATA-MODEL.md` §Entities, ADR-0006): the regions of
//! material a solid's shells bound, as B1 proves them — the row's nesting
//! and [`lumps`], the query that returns it, over one piece of code.
//!
//! A shell enclosing positive volume is an *outer* shell and one enclosing
//! negative volume a *void*. No two shells may cross or overlap; they may
//! touch (ADR-0088). A shell lies inside another when a point of it does,
//! by the ray cast `classify` runs over the other's faces alone; two closed
//! surfaces that do not cross are nested or apart, so one point clear of
//! the other decides. Each void's innermost container must be
//! an outer shell and each outer shell's none or a void: a lump is an outer
//! shell with the voids whose innermost container it is, and a lump may sit
//! in another lump's cavity.

use core::cmp::Reverse;
use core::fmt;
use std::collections::BTreeSet;

use arris_math::Meter;
use arris_topo::entity::BodyKind;
use arris_topo::{Body, FaceId, Model, NotFound, Shell, ShellId};

use crate::check::Checker;
use crate::classify::Classifier;
use crate::full::Meeting;
use crate::unchecked::Unchecked;
use crate::violation::ShellNestingFault;

/// How many points of one shell [`Checker::shell_inside`] tries before it
/// gives the nesting up as undecided.
const PLACING_POINTS: usize = 64;

/// One lump of a solid body (ADR-0006): an outer shell and the voids
/// inside it. Every handle carries its effective orientation — the body
/// handle's composed with the shell use's — as `Model::shells` yields it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lump {
    /// The shell enclosing the lump's material: positive volume.
    pub outer: Shell,
    /// The void shells whose innermost containing shell is `outer`, in
    /// the body's stored order: negative volume each.
    pub voids: Vec<Shell>,
}

/// Why [`lumps`] has no answer for a body. Never a guess: each is what the
/// checker's B1 row reports of the same body, or what stops it running.
#[derive(Debug, Clone, PartialEq)]
pub enum LumpError {
    /// The body handle does not resolve.
    NotFound(NotFound),
    /// The body is not a `Solid`: only a solid's shells nest into lumps.
    NotSolid {
        /// The body.
        body: Body,
        /// Its kind.
        kind: BodyKind,
    },
    /// A shell's enclosed volume could not be integrated: a reference of
    /// it does not resolve or a range is not bounded — the checker's M1
    /// and E1.
    Unmeasurable {
        /// The body.
        body: Body,
        /// The shell.
        shell: ShellId,
    },
    /// The shells do not nest into lumps: B1's first fault.
    Nesting {
        /// The body.
        body: Body,
        /// What is wrong.
        fault: ShellNestingFault,
    },
    /// The nesting could not be decided: B1's undecided rows.
    Undecided {
        /// The body.
        body: Body,
        /// The rows, each naming what could not be decided.
        rows: Vec<Unchecked>,
    },
}

impl fmt::Display for LumpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            LumpError::NotFound(e) => write!(f, "{e}"),
            LumpError::NotSolid { body, kind } => {
                write!(f, "{body} is a {kind} body, and only a solid has lumps")
            }
            LumpError::Unmeasurable { body, shell } => {
                write!(
                    f,
                    "{body}: the volume {shell} encloses cannot be integrated"
                )
            }
            LumpError::Nesting { body, fault } => {
                write!(f, "{body}: the shells do not nest: {fault}")
            }
            LumpError::Undecided { body, rows } => {
                write!(f, "{body}: the nesting is undecided:")?;
                for row in rows {
                    write!(f, " {row};")?;
                }
                Ok(())
            }
        }
    }
}

impl std::error::Error for LumpError {}

impl From<NotFound> for LumpError {
    fn from(e: NotFound) -> Self {
        LumpError::NotFound(e)
    }
}

/// The lumps of the solid `body` (ADR-0006): each outer shell with the
/// voids whose innermost container it is, in the order the body stores the
/// outer shells, each lump's voids in stored order. Exactly the nesting
/// the checker's B1 proves, by the same code: for a body B1 passes and
/// decides, this is `Ok`, and for one it does not, it is the reason. A
/// body of one shell is one lump and casts no ray.
///
/// Errors: [`LumpError::NotFound`], [`LumpError::NotSolid`],
/// [`LumpError::Unmeasurable`], [`LumpError::Nesting`] with B1's first
/// fault, [`LumpError::Undecided`] with its undecided rows.
///
/// ```
/// use arris_check::lumps;
/// use arris_debug::sample;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
/// let found = lumps(&m, body)?;
/// assert_eq!(found.len(), 1);
/// assert_eq!(found[0].outer, m.shells(body)?[0]);
/// assert!(found[0].voids.is_empty());
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn lumps(model: &Model, body: Body) -> Result<Vec<Lump>, LumpError> {
    let mut c = Checker::new(model, body)?;
    let kind = model.body(body.id)?.kind();
    if kind != BodyKind::Solid {
        return Err(LumpError::NotSolid { body, kind });
    }
    let shells = c
        .shell_volumes()
        .map_err(|shell| LumpError::Unmeasurable { body, shell })?;
    if shells.len() > 1 {
        // Only a meeting test between shells reads the polygons.
        c.discretise_faces();
    }
    let nesting = c.nesting(&shells);
    if let Some(fault) = nesting.faults.into_iter().next() {
        return Err(LumpError::Nesting { body, fault });
    }
    if !nesting.unchecked.is_empty() {
        return Err(LumpError::Undecided {
            body,
            rows: nesting.unchecked,
        });
    }
    Ok(nesting.lumps)
}

/// What B1 found of a body's shells: its faults, its undecided rows, and
/// the lumps when there are neither.
pub(crate) struct Nesting {
    pub(crate) faults: Vec<ShellNestingFault>,
    pub(crate) unchecked: Vec<Unchecked>,
    pub(crate) lumps: Vec<Lump>,
}

impl Checker<'_> {
    /// Every shell use of the body with its effective orientation and the
    /// signed volume it encloses, in stored order. Errors: the first shell
    /// whose volume could not be integrated.
    pub(crate) fn shell_volumes(&self) -> Result<Vec<(Shell, f64)>, ShellId> {
        let Ok(body) = self.model.body(self.body.id) else {
            return Ok(Vec::new());
        };
        body.shells()
            .iter()
            .map(|s| {
                let s = s.oriented_by(self.body.orientation);
                self.shell_volume(s.id, s.orientation)
                    .map(|v| (s, v))
                    .ok_or(s.id)
            })
            .collect()
    }

    /// Whether `shell` lies inside the closed surface `other` classifies
    /// against, by the first point of it that is clear of `other`'s faces
    /// and edges and that a ray decides: two lumps that touch (ADR-0088)
    /// may have a vertex on the contact, on `other`, where a ray's parity
    /// means nothing, so that vertex is passed over for the next, and a
    /// shell whose every vertex is on the contact for an interior point of
    /// its faces ([`Checker::shell_points`]). `None` when no point of the
    /// first [`PLACING_POINTS`] decides, or there is no classifier.
    fn shell_inside(&self, shell: ShellId, other: Option<&Classifier>) -> Option<bool> {
        let other = other?;
        for point in self.shell_points(shell).take(PLACING_POINTS) {
            if !matches!(other.on(point), Ok(None)) {
                continue;
            }
            if let Ok(Some(is)) = other.contains(point) {
                return Some(is);
            }
        }
        None
    }

    /// B1 over `shells` (from [`Checker::shell_volumes`]). The faces must
    /// have been discretised when there is more than one shell. Stops at
    /// the first stage that fails — roles, then meetings, then containment
    /// — since a later stage means nothing over an earlier failure.
    pub(crate) fn nesting(&self, shells: &[(Shell, f64)]) -> Nesting {
        let mut out = Nesting {
            faults: Vec::new(),
            unchecked: Vec::new(),
            lumps: Vec::new(),
        };
        let body = self.body.id;
        if shells.is_empty() {
            out.faults.push(ShellNestingFault::NoShells);
            return out;
        }
        for &(shell, volume) in shells {
            if !(volume.is_finite() && volume != 0.0) {
                out.faults
                    .push(ShellNestingFault::InsideOut { shell: shell.id });
            }
        }
        if !shells.iter().any(|&(_, v)| v > 0.0) {
            out.faults.insert(0, ShellNestingFault::NoOuter);
        }
        if !out.faults.is_empty() {
            return out;
        }
        if let [(outer, _)] = shells {
            out.lumps.push(Lump {
                outer: *outer,
                voids: Vec::new(),
            });
            return out;
        }

        // No two shells meet.
        let n = shells.len();
        let boxes = self.face_boxes();
        let faces: Vec<Vec<FaceId>> = shells
            .iter()
            .map(|(s, _)| {
                self.model.shell(s.id).map_or_else(
                    |_| Vec::new(),
                    |e| {
                        e.faces()
                            .iter()
                            .map(|f| f.id)
                            .collect::<BTreeSet<_>>()
                            .into_iter()
                            .collect()
                    },
                )
            })
            .collect();
        for i in 0..n {
            for j in i + 1..n {
                let mut meets = false;
                'pairs: for &a in &faces[i] {
                    for &b in &faces[j] {
                        match self.faces_meet(
                            a,
                            b,
                            &boxes,
                            Meeting::Crossing,
                            &mut Meter::default(),
                        ) {
                            Ok(false) => {}
                            Ok(true) => {
                                meets = true;
                                break 'pairs;
                            }
                            Err(kinds) => out.unchecked.push(Unchecked::ShellFacePair {
                                body,
                                face_a: a,
                                face_b: b,
                                kinds,
                            }),
                        }
                    }
                }
                if meets {
                    let (a, b) = (shells[i].0.id, shells[j].0.id);
                    out.faults.push(ShellNestingFault::Overlap {
                        shells: [a.min(b), a.max(b)],
                    });
                }
            }
        }
        if !out.faults.is_empty() || !out.unchecked.is_empty() {
            return out;
        }

        // Which shells each lies inside, by a point of it clear of the
        // other: each shell's classifier is built once and asked for every
        // other's point.
        let classifiers: Vec<Option<Classifier>> = shells
            .iter()
            .map(|(s, _)| self.shell_classifier(s.id))
            .collect();
        let mut inside = vec![vec![false; n]; n];
        for i in 0..n {
            for j in (0..n).filter(|&j| j != i) {
                match self.shell_inside(shells[i].0.id, classifiers[j].as_ref()) {
                    Some(is) => inside[i][j] = is,
                    None => {
                        out.unchecked.push(Unchecked::ShellNesting {
                            body,
                            shell: shells[i].0.id,
                        });
                        break;
                    }
                }
            }
        }
        if !out.unchecked.is_empty() {
            return out;
        }
        let depth = |i: usize| inside[i].iter().filter(|&&is| is).count();
        // The innermost container is the deepest one; containers of one
        // shell that do not meet are nested, so the depths differ.
        let innermost = |i: usize| {
            (0..n)
                .filter(|&j| inside[i][j])
                .max_by_key(|&j| (depth(j), Reverse(j)))
        };
        let is_outer = |i: usize| shells[i].1 > 0.0;
        for i in 0..n {
            let shell = shells[i].0.id;
            match (is_outer(i), innermost(i)) {
                (false, None) => out.faults.push(ShellNestingFault::VoidOutside { shell }),
                (false, Some(j)) if !is_outer(j) => {
                    out.faults.push(ShellNestingFault::VoidInVoid {
                        shell,
                        container: shells[j].0.id,
                    });
                }
                (true, Some(j)) if is_outer(j) => {
                    out.faults.push(ShellNestingFault::OuterInOuter {
                        shell,
                        container: shells[j].0.id,
                    });
                }
                _ => {}
            }
        }
        if !out.faults.is_empty() {
            return out;
        }
        for o in (0..n).filter(|&o| is_outer(o)) {
            out.lumps.push(Lump {
                outer: shells[o].0,
                voids: (0..n)
                    .filter(|&v| !is_outer(v) && innermost(v) == Some(o))
                    .map(|v| shells[v].0)
                    .collect(),
            });
        }
        out
    }
}
