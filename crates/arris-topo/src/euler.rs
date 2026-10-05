//! The Euler–Poincaré line of a body (`docs/DATA-MODEL.md`
//! §Euler–Poincaré): one count of a closure, which the checker's report,
//! the builder's counts, the dump and the fixture lint all take.

use core::fmt;

use crate::Model;
use crate::walk::Closure;

/// The Euler–Poincaré line of a body: the five counts of its closure and
/// the genus they imply through `V − E + F − (L − F) − 2(S − G) = 0`. The
/// genus is *derived*, as the oracle derives it, so the line is not a
/// violation on its own; what it checks is its parity — a count set that
/// leaves a [`EulerLine::residual`] of one cannot come from any closed
/// orientable surface, whatever its genus. A degenerate edge is not
/// counted: it is a singular point of its surface (a cone's apex, a
/// sphere's pole), not a boundary between faces — S2's reading of it —
/// and counting it would give a sphere genus 1 and a cone an odd line.
///
/// ```
/// use arris_debug::sample;
/// use arris_topo::Model;
/// use arris_topo::euler::EulerLine;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
/// let line = EulerLine::of(&m, &m.closure(body)?);
/// assert_eq!(line.to_string(), "2/3/3/3/1 g0 = 0");
/// assert!(line.closes());
/// // The sphere's two pole edges are left out.
/// let sphere = sample::sphere(&mut m, arris_math::Point3::origin(), 3.0)?;
/// assert_eq!(EulerLine::of(&m, &m.closure(sphere)?).to_string(), "2/1/1/1/1 g0 = 0");
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EulerLine {
    /// Vertices in the closure.
    pub vertices: usize,
    /// Edges in the closure that are not degenerate.
    pub edges: usize,
    /// Faces in the closure.
    pub faces: usize,
    /// Loops over those faces.
    pub loops: usize,
    /// Shells in the closure.
    pub shells: usize,
    /// The genus the counts imply, `S − ⌊(V − E + 2F − L) / 2⌋`.
    pub genus: i64,
}

impl EulerLine {
    /// The line of the counts, with the genus derived from them. `edges`
    /// is the count with every degenerate edge already left out.
    pub const fn new(
        vertices: usize,
        edges: usize,
        faces: usize,
        loops: usize,
        shells: usize,
    ) -> Self {
        // V − E + F − (L − F) − 2(S − G) = 0  ⇒  2G = 2S − (V − E + 2F − L).
        let x = vertices as i64 - edges as i64 + 2 * faces as i64 - loops as i64;
        EulerLine {
            vertices,
            edges,
            faces,
            loops,
            shells,
            genus: shells as i64 - x.div_euclid(2),
        }
    }

    /// The line of `closure` in `model`: its vertices, its edges less
    /// every degenerate one, its faces, their loops and its shells. An id
    /// of the closure that does not resolve counts as a non-degenerate
    /// edge or a face of no loops — the checker's M1 reports it.
    pub fn of(model: &Model, closure: &Closure) -> Self {
        let loops = closure
            .faces
            .iter()
            .filter_map(|&f| model.face(f).ok())
            .map(|f| f.loops().len())
            .sum();
        let edges = closure
            .edges
            .iter()
            .filter(|&&e| !model.edge(e).is_ok_and(|e| e.is_degenerate()))
            .count();
        EulerLine::new(
            closure.vertices.len(),
            edges,
            closure.faces.len(),
            loops,
            closure.shells.len(),
        )
    }

    /// `V − E + 2F − L`, which is `2(S − G)` on a line that closes.
    pub const fn characteristic(&self) -> i64 {
        self.vertices as i64 - self.edges as i64 + 2 * self.faces as i64 - self.loops as i64
    }

    /// `V − E + F − (L − F) − 2(S − G)` at the given `genus` rather than
    /// the derived one: zero exactly when the counts close at that genus.
    pub const fn at_genus(&self, genus: i64) -> i64 {
        self.characteristic() - 2 * (self.shells as i64 - genus)
    }

    /// What the counts leave once the genus is taken out: `0` for a line
    /// that closes, `1` for one that cannot come from any genus.
    pub const fn residual(&self) -> i64 {
        self.characteristic().rem_euclid(2)
    }

    /// `true` when [`EulerLine::residual`] is zero.
    pub const fn closes(&self) -> bool {
        self.residual() == 0
    }
}

impl fmt::Display for EulerLine {
    /// `V/E/F/L/S g<genus> = <residual>`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}/{}/{}/{}/{} g{} = {}",
            self.vertices,
            self.edges,
            self.faces,
            self.loops,
            self.shells,
            self.genus,
            self.residual()
        )
    }
}
