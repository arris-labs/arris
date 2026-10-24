#[cfg(doc)]
use super::Builder;
use super::{BuildError, EdgeRef, StagedEdge};
use crate::entity::EdgeGeometry;
use crate::orientation::Orientation;

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
fn uses_allowed(edge: &StagedEdge, n: usize, open: bool) -> bool {
    if is_degenerate(edge) {
        n == 1
    } else if open {
        n == 1 || n == 2
    } else {
        n == 2
    }
}

/// The edge-use rule of a finished body, for one edge and the uses `list`
/// gives it (`direction` reads a use's effective orientation): a number of
/// uses [`uses_allowed`] permits ([`BuildError::EdgeUses`]), and two uses
/// in the same direction ([`BuildError::SameDirection`]). What
/// [`Builder::finish`] and [`Builder::assemble`] each prove edge by edge,
/// in slot order, so the first fault reported is the same one.
pub(super) fn edge_uses<T>(
    edge: EdgeRef,
    staged: &StagedEdge,
    list: &[T],
    open: bool,
    direction: impl Fn(&T) -> Orientation,
) -> Result<(), BuildError> {
    if !uses_allowed(staged, list.len(), open) {
        return Err(BuildError::EdgeUses {
            edge,
            uses: list.len(),
        });
    }
    if matches!(list, [a, b] if direction(a) == direction(b)) {
        return Err(BuildError::SameDirection { edge });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::builder::VertexRef;
    use arris_math::Interval;

    fn edge(geometry: EdgeGeometry) -> StagedEdge {
        StagedEdge {
            geometry,
            start: VertexRef::new(0),
            end: VertexRef::new(1),
            tolerance: 1e-7,
            kept: None,
        }
    }

    #[test]
    fn use_counts_and_directions_are_refused_by_the_one_rule() {
        use Orientation::{Forward, Reversed};
        let e = EdgeRef::new(3);
        let curve = edge(EdgeGeometry::Curve {
            curve: crate::id::CurveId::new(0, 0),
            range: Interval::TURN,
        });
        let same = |o: &Orientation| *o;
        assert_eq!(
            edge_uses(e, &curve, &[Forward, Reversed], false, same),
            Ok(())
        );
        assert_eq!(
            edge_uses(e, &curve, &[Forward, Forward], false, same),
            Err(BuildError::SameDirection { edge: e })
        );
        assert_eq!(
            edge_uses(e, &curve, &[Forward], false, same),
            Err(BuildError::EdgeUses { edge: e, uses: 1 })
        );
        assert_eq!(edge_uses(e, &curve, &[Forward], true, same), Ok(()));
        // A list of another type reads its direction through the closure,
        // as `assemble`'s (orientation, face) pairs do.
        assert_eq!(
            edge_uses(e, &curve, &[(Reversed, 7), (Reversed, 9)], false, |u| u.0),
            Err(BuildError::SameDirection { edge: e })
        );
        let apex = edge(EdgeGeometry::Degenerate {
            range: Interval::TURN,
        });
        assert_eq!(edge_uses(e, &apex, &[Forward], false, same), Ok(()));
        assert_eq!(
            edge_uses(e, &apex, &[Forward, Reversed], false, same),
            Err(BuildError::EdgeUses { edge: e, uses: 2 })
        );
    }
}
