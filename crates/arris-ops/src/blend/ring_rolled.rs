//! A ring's run-over (ADR-0071): a closed edge's torus whose contact on `F`
//! leaves it through an edge `E` of `F`. The ball rolls on `E` between the
//! two crossings, so the rolled face (`rolled.rs`) takes the place of the
//! torus there: the torus becomes one face open at it, from the second
//! crossing's section arc round through its own seam to the first's. The
//! seam is then inside the face and no edge of it is made.

use core::f64::consts::TAU;

use arris_geom::{Curve, Curve2, Surface, pcurve_on};
use arris_math::{Frame, Interval, Meter, Point2, Tolerance, Vec2, real};
use arris_topo::{EdgeId, Orientation};

use super::build::{Env, Made};
use super::ring::{Ring, RingContact, placed_uv};
use super::rolled::{Host, Rolled, SectionArc, push_rolled_face, rolled_of};
use super::run_over::{Crossing, RunOver, crossed};
use super::{Kind, invariant};
use crate::body_view::UseAt;
use crate::error::{OpError, fault_of};
use crate::rebuild::{AddedFace, Rewrite, StoredUse};
use arris_topo::Model;
use arris_topo::builder::EdgeKey;

/// What a ring's run-over reads of the ring: the blended edge, its uses,
/// tolerance and convexity, the torus and the contacts, the lower `v` first.
pub(super) struct RingHost<'a> {
    pub(super) edge: EdgeId,
    pub(super) uses: &'a [UseAt],
    pub(super) tolerance: f64,
    pub(super) convex: bool,
    pub(super) surface: &'a Surface,
    pub(super) contacts: &'a [RingContact; 2],
}

/// The rolled face of the closed edge's run-over `run`, its contact `on_f`
/// on `F` (ADR-0071 §1, §2).
///
/// Errors: `BlendTooLarge` naming the blended edge and `E` where the blend
/// is not on a torus, the stretch off `F` runs through the contacts' seam,
/// the two contacts do not run the same way round, or `rolled_of` refuses
/// (`rolled.rs`).
pub(super) fn ring_rolled(
    env: &Env<'_>,
    meter: &mut Meter<'_>,
    ring: &RingHost<'_>,
    on_f: usize,
    run: RunOver,
) -> Result<Rolled, OpError> {
    let crossed_edge = run.edge;
    let refuse = || crossed(ring.edge, crossed_edge);
    let [lower, upper] = ring.contacts;
    let tol = Tolerance::new(ring.tolerance, env.tol.angular);
    let (Surface::Torus { .. }, Kind::Fillet { radius }) = (ring.surface, env.kind) else {
        return Err(refuse());
    };
    // The stretch off `F` lies inside the contacts' range, away from the
    // seam, and both contacts run with `u` or both against it.
    let wraps = run.off.hi() > lower.range.hi() + tol.linear;
    if wraps
        || lower.along_u != upper.along_u
        || lower.range.lo() != upper.range.lo()
        || lower.range.hi() != upper.range.hi()
    {
        return Err(refuse());
    }
    let use_of = |face| ring.uses.iter().find(|u| u.face == face);
    let (Some(lo_use), Some(hi_use)) = (use_of(lower.face), use_of(upper.face)) else {
        return Err(invariant("a ring's uses on its two faces"));
    };
    let host = Host {
        edge: ring.edge,
        tolerance: ring.tolerance,
        convex: ring.convex,
        faces: [lower.face, upper.face],
        curves: [&lower.curve, &upper.curve],
        ranges: [lower.range, upper.range],
        on_face: [&lower.on_face, &upper.on_face],
        loops: [lo_use.loop_index, hi_use.loop_index],
    };
    let arc_at = |c: &Crossing, meter: &mut Meter<'_>| -> Result<SectionArc, OpError> {
        let (a, b) = (lower.curve.point(c.t), upper.curve.point(c.t));
        let (from, to) = (a - c.centre, b - c.centre);
        let turn = from.cross(&to);
        let beta = real::atan2(turn.norm(), from.dot(&to));
        let normal = turn.try_normalize(0.0).ok_or_else(refuse)?;
        let curve = Curve::Circle {
            frame: Frame::new(c.centre, normal, from)?,
            radius,
        };
        let range = Interval::new(0.0, beta).map_err(|_| refuse())?;
        let on_blend = pcurve_on(&curve, range, ring.surface, tol, meter).map_err(fault_of)?;
        let at = lower.on_blend.point(c.t);
        let on_blend = placed_uv(on_blend, range.lo(), Point2::new(at.x, at.y));
        Ok(SectionArc {
            on_stripe: on_blend,
            on_rolled: Curve2::along_u(0.0),
            curve,
            range,
        })
    };
    rolled_of(env, meter, &host, &arc_at, on_f, run)
}

/// The torus face of a ring with a run-over, and the rolled face, appended
/// to `rw.added`; `made.added` is the torus face and the rolled face's
/// index goes in its `rolled.added[1]`. Counter-clockwise in `(u, v)` as
/// the ring's face is: the lower contact along `u` from the second
/// crossing's arc past the seam to the first's (`u` taken a turn on from
/// the seam), up that arc, the upper contact back, down the other.
pub(super) fn add_ring_rolled_faces(
    m: &mut Model,
    rw: &mut Rewrite,
    shell: usize,
    r: &Ring,
    rolled: &Rolled,
    made: &mut Made,
) -> Result<(), OpError> {
    let ring = &r.contacts;
    let Some(mut rolled_made) = made.rolled else {
        return Err(invariant("a ring's run-over entities"));
    };
    let along = ring[0].along_u;
    // Increasing `u`: past the seam the contact's stretch after the second
    // crossing and the one before the first, in `t` when it runs with `u`,
    // against it otherwise; the crossing the face starts at, then the one
    // it ends at.
    let (start, end) = if along { (1, 0) } else { (0, 1) };
    let turn = Vec2::new(TAU, 0.0);
    let chain = |m: &mut Model, j: usize| -> Vec<StoredUse> {
        let pieces = rolled_made.contacts[j];
        let on = &ring[j].on_blend;
        let parts = if along {
            [(pieces[1], false), (pieces[0], true)]
        } else {
            [(pieces[0], false), (pieces[1], true)]
        };
        parts
            .into_iter()
            .map(|(edge, wrapped)| StoredUse {
                edge: EdgeKey::New(edge),
                orientation: if along {
                    Orientation::Forward
                } else {
                    Orientation::Reversed
                },
                pcurve: m.add_curve2(if wrapped {
                    on.translated(turn)
                } else {
                    on.clone()
                }),
            })
            .collect()
    };
    let arc = |m: &mut Model, i: usize, up: bool| StoredUse {
        edge: EdgeKey::New(rolled_made.arcs[i]),
        orientation: if up {
            Orientation::Forward
        } else {
            Orientation::Reversed
        },
        pcurve: m.add_curve2(if i == end {
            rolled.arcs[i].on_stripe.translated(turn)
        } else {
            rolled.arcs[i].on_stripe.clone()
        }),
    };
    let mut loop_uses = chain(m, 0);
    loop_uses.push(arc(m, end, true));
    let mut back = chain(m, 1);
    back.reverse();
    for u in &mut back {
        u.orientation = match u.orientation {
            Orientation::Forward => Orientation::Reversed,
            Orientation::Reversed => Orientation::Forward,
        };
    }
    loop_uses.extend(back);
    loop_uses.push(arc(m, start, false));
    let surface = m.add_surface(r.surface.clone());
    made.added = rw.face(AddedFace {
        shell,
        surface,
        orientation: r.orientation,
        loops: vec![loop_uses],
        tolerance: r.tolerance,
    });
    // The rolled face follows the torus face.
    push_rolled_face(m, rw, shell, rolled, &mut rolled_made)?;
    made.rolled = Some(rolled_made);
    Ok(())
}
