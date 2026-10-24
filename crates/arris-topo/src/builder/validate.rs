use super::StagedEdge;
use crate::entity::EdgeGeometry;

/// Whether a staged edge is degenerate: the singular point of the one face
/// that closes on it — a cone's apex, a sphere's pole — which is not a
/// boundary between two faces (`docs/DATA-MODEL.md` §Invariants, S2) and
/// which the Euler–Poincaré line does not count.
pub(super) fn is_degenerate(edge: &StagedEdge) -> bool {
    matches!(edge.geometry, EdgeGeometry::Degenerate { .. })
}

/// Whether `n` coedges may use `edge` in a finished body: one for a
/// degenerate edge, since no surface closes on one singular point twice;
/// two for any other edge of a closed shell, and one or two of an `open`
/// one, whose boundary edges have a face on one side only.
pub(super) fn uses_allowed(edge: &StagedEdge, n: usize, open: bool) -> bool {
    if is_degenerate(edge) {
        n == 1
    } else if open {
        n == 1 || n == 2
    } else {
        n == 2
    }
}
