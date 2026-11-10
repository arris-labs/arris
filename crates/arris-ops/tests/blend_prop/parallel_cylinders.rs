use super::support::*;

// Two cylinders with parallel axes meeting along a ruling (ADR-0036 §5): an
// ogive bar, whose tip edge is convex, and two equal discs united, whose
// waist edge is concave. The blended edge is the only one, so the faces
// meet at it and no other blend or step is in play.

/// The loop `a → t → b` of the corner a parallel-cylinder blend changes:
/// the wall arc about `ca` from `a` to `t`, the wall arc about `cb` from
/// `t` to `b`, and the blend back from `b` to `a`, a straight chord or the
/// arc about `ball`. Every arc is the short way round. Returns the
/// enclosed area, by Green's theorem on each piece: a circular arc about
/// `c` of radius `rho` over `dphi` adds `rho² dphi / 2 + c × (q − p) / 2`.
fn corner_area(
    (a, t, b): (Point2, Point2, Point2),
    (ca, cb): (Point2, Point2),
    ball: Option<Point2>,
) -> f64 {
    let cross = |u: Vec2, v: Vec2| u.x * v.y - u.y * v.x;
    let arc = |p: Point2, q: Point2, c: Point2| {
        let (u, v) = (p - c, q - c);
        let dphi = real::atan2(cross(u, v), u.dot(&v));
        0.5 * u.norm_squared() * dphi + 0.5 * cross(c.coords, q - p)
    };
    let line = |p: Point2, q: Point2| 0.5 * cross(p.coords, q.coords);
    let closing = match ball {
        Some(c) => arc(b, a, c),
        None => line(b, a),
    };
    (arc(a, t, ca) + arc(t, b, cb) + closing).abs()
}

/// The point of the circle about `c` of radius `rho` that is `angle` from
/// the direction `at`.
fn on_circle(c: Point2, rho: f64, at: f64) -> Point2 {
    Point2::new(c.x + rho * real::cos(at), c.y + rho * real::sin(at))
}

/// An ogive bar and the closed form of its tip, or the two discs and
/// theirs: the profile to extrude, the edge's midpoint, the unblended
/// volume, the corner's area as `size` blends it and its sign (the corner
/// leaves a convex body and joins a concave one).
struct Pair {
    profile: Profile,
    edge: Point3,
    volume: f64,
    corner: f64,
    convex: bool,
}

const HEIGHT: f64 = 1.5;

/// A bar of half-thickness `w` and length `l` ending in two arcs of radius
/// `big >= w`, each running tangentially out of a side (its centre `big - w`
/// across the bar) and meeting at the tip.
fn ogive(w: f64, l: f64, big: f64, kind: Blend, share: f64) -> Pair {
    let p = Point2::new;
    let s = (w * (2.0 * big - w)).sqrt();
    let tip = p(l + s, 0.0);
    let (cl, cu) = (p(l, big - w), p(l, w - big));
    let a = real::atan2(w - big, s);
    let via = on_circle(cl, big, 0.5 * (a - FRAC_PI_2));
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: p(0.0, -w),
            segments: vec![
                ProfileSegment::LineTo(p(l, -w)),
                ProfileSegment::ArcTo { to: tip, via },
                ProfileSegment::ArcTo {
                    to: p(l, w),
                    via: p(via.x, -via.y),
                },
                ProfileSegment::LineTo(p(0.0, w)),
                ProfileSegment::LineTo(p(0.0, -w)),
            ],
        },
        holes: Vec::new(),
    };
    let rect = 2.0 * w * l;
    // The cap is the loop (l, −w) → tip → (l, w) closed by the line back
    // down the bar's end: a corner whose closing piece is a chord.
    let cap = corner_area((p(l, -w), tip, p(l, w)), (cl, cu), None);
    let corner = match kind {
        Blend::Fillet => {
            let r = share * w;
            let ball = p(
                l + ((big - r) * (big - r) - (big - w) * (big - w)).sqrt(),
                0.0,
            );
            let contact = |c: Point2| {
                let u = ball - c;
                c + u * (big / u.norm())
            };
            corner_area((contact(cl), tip, contact(cu)), (cl, cu), Some(ball))
        }
        Blend::Chamfer => {
            let reach = 2.0 * big * real::sin(0.5 * (FRAC_PI_2 + a));
            let theta = 2.0 * real::asin(share * reach / (2.0 * big));
            let low = on_circle(cl, big, a - theta);
            corner_area((low, tip, p(low.x, -low.y)), (cl, cu), None)
        }
    };
    Pair {
        profile,
        edge: Point3::new(tip.x, 0.0, HEIGHT / 2.0),
        volume: HEIGHT * (rect + cap),
        corner: HEIGHT * corner,
        convex: true,
    }
}

/// Two discs of radius `big` about `(0, 0)` and `(gap, 0)` united.
fn waist(big: f64, gap: f64, kind: Blend, share: f64) -> Pair {
    let p = Point2::new;
    let h = (big * big - gap * gap / 4.0).sqrt();
    let top = p(gap / 2.0, h);
    let (c1, c2) = (p(0.0, 0.0), p(gap, 0.0));
    let profile = Profile {
        plane: Frame::world(),
        outer: ProfileLoop::Path {
            start: top,
            segments: vec![
                ProfileSegment::ArcTo {
                    to: p(gap / 2.0, -h),
                    via: p(-big, 0.0),
                },
                ProfileSegment::ArcTo {
                    to: top,
                    via: p(gap + big, 0.0),
                },
            ],
        },
        holes: Vec::new(),
    };
    let lens_angle = 2.0 * real::acos(gap / (2.0 * big));
    let lens = big * big * (lens_angle - real::sin(lens_angle));
    let corner = match kind {
        Blend::Fillet => {
            let r = share * big;
            let ball = p(gap / 2.0, ((big + r) * (big + r) - gap * gap / 4.0).sqrt());
            let contact = |c: Point2| {
                let u = ball - c;
                c + u * (big / u.norm())
            };
            corner_area((contact(c1), top, contact(c2)), (c1, c2), Some(ball))
        }
        Blend::Chamfer => {
            let a = real::atan2(h, gap / 2.0);
            let reach = 2.0 * big * real::sin(0.5 * (FRAC_PI_2 - a));
            let theta = 2.0 * real::asin(share * reach / (2.0 * big));
            let left = on_circle(c1, big, a + theta);
            corner_area((left, top, p(gap - left.x, left.y)), (c1, c2), None)
        }
    };
    Pair {
        profile,
        edge: Point3::new(top.x, top.y, HEIGHT / 2.0),
        volume: HEIGHT * (2.0 * PI * big * big - lens),
        corner: HEIGHT * corner,
        convex: false,
    }
}

/// `pair` moved by `pose`, its edge blended at `size`: built and held to
/// its closed form, or refused by a named blend reason.
fn parallel_pair_holds(
    pair: Pair,
    kind: Blend,
    size: f64,
    pose: &Isometry,
) -> Result<(), TestCaseError> {
    let run = |pose: &Isometry| -> Run {
        let mut m = Model::default();
        let body = extrude(&mut m, &pair.profile, Vec3::z(), HEIGHT)
            .map_err(fail)?
            .0;
        let (moved, _) = transform(&mut m, body, pose).map_err(fail)?;
        let edge = edge_near(&m, moved, pose.apply(pair.edge))?;
        let out = op(kind)(&mut m, moved, &[edge], size);
        Ok((m, moved, out))
    };
    let (m, moved, out) = run(pose)?;
    let (blended, p) = match out {
        Ok(built) => built,
        Err(OpError::Degenerate {
            reason: Reason::Blend(BlendReason::TooLarge | BlendReason::VertexBlend),
            ..
        }) => return Ok(()),
        Err(e) => return Err(fail(format!("{kind:?} of {size}: {e}"))),
    };
    let report = check(&m, blended, Level::Full);
    assert_checked(&report)?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let before = mass_properties(&m, moved).map_err(fail)?.volume;
    prop_assert!(
        close_to(before, pair.volume, 1.0, REL),
        "the unblended body is {before}, not {}",
        pair.volume
    );
    let want = if pair.convex {
        pair.volume - pair.corner
    } else {
        pair.volume + pair.corner
    };
    let got = mass_properties(&m, blended).map_err(fail)?.volume;
    prop_assert!(
        close_to(got, want, 1.0, REL),
        "{kind:?} of {size}: volume {got} vs the closed form {want}"
    );
    let (again, _, second) = run(pose)?;
    let twice = second.map_err(|e| fail(format!("built once, then {e}")))?.0;
    prop_assert_eq!(
        dump_text(&again, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?,
        "a second run"
    );
    Ok(())
}
prop_shards! {
    /// An ogive bar's convex tip edge, filleted or chamfered, in a random
    /// pose: the faces are two cylinders with parallel axes. Built clean
    /// at `Full`, audited, at the closed-form volume, deterministic.
    an_ogive_tip_blends_as_its_closed_form
        [shard_0 shard_1 shard_2 shard_3]
        ((w, l, ratio, share, kind, pose)) = (
            prop::finite_f64(0.5..=1.2),
            prop::finite_f64(1.0..=3.0),
            prop::finite_f64(1.5..=3.0),
            prop::finite_f64(0.1..=0.8),
            blend_kind(),
            prop::pose(),
        ) => {
            let big = ratio * w;
            let pair = ogive(w, l, big, kind, share);
            let size = match kind {
                Blend::Fillet => share * w,
                Blend::Chamfer => {
                    let a = real::atan2(w - big, (w * (2.0 * big - w)).sqrt());
                    share * 2.0 * big * real::sin(0.5 * (FRAC_PI_2 + a))
                }
            };
            parallel_pair_holds(pair, kind, size, &pose)
        }
}

prop_shards! {
    /// Two equal discs united, their concave waist edge filleted or
    /// chamfered, in a random pose: built clean at `Full`, audited, at the
    /// closed-form volume, deterministic.
    a_waist_blends_as_its_closed_form
        [shard_0 shard_1 shard_2 shard_3]
        ((big, spread, share, kind, pose)) = (
            prop::finite_f64(1.0..=2.0),
            prop::finite_f64(0.8..=1.6),
            prop::finite_f64(0.1..=0.5),
            blend_kind(),
            prop::pose(),
        ) => {
            let gap = spread * big;
            let pair = waist(big, gap, kind, share);
            let size = match kind {
                Blend::Fillet => share * big,
                Blend::Chamfer => {
                    let a = real::atan2((big * big - gap * gap / 4.0).sqrt(), gap / 2.0);
                    share * 2.0 * big * real::sin(0.5 * (FRAC_PI_2 - a))
                }
            };
            parallel_pair_holds(pair, kind, size, &pose)
        }
}
