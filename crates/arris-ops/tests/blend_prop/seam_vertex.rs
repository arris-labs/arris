use super::support::*;

// A walked blend at a vertex on a seam (ADR-0083): the ball's seam is a
// choice of where the revolve's profile plane lies, not a feature of the
// body, so the blend does not depend on it.

/// The truncated ball of `regression/sphere-cylinder-oblique-seam-fillet`
/// revolved from a profile plane at azimuth `phi` (its seam), fused with
/// the pin of radius 0.6 along `x` at `y = off`, its loop edge at
/// `(x, off + 0.6, 0)` blended by `kind` at `r`.
fn seam_ball(phi: f64, off: f64, kind: Blend, r: f64) -> Result<(Model, Body), TestCaseError> {
    let mut m = Model::default();
    let (c, s) = (real::cos(phi), real::sin(phi));
    let foot = 1.3228756555322954;
    let half = Profile {
        plane: Frame::new(
            Point3::origin(),
            Vec3::new(s, -c, 0.0),
            Vec3::new(c, s, 0.0),
        )
        .map_err(fail)?,
        outer: ProfileLoop::Path {
            start: Point2::new(0.0, -1.5),
            segments: vec![
                ProfileSegment::LineTo(Point2::new(foot, -1.5)),
                ProfileSegment::ArcTo {
                    to: Point2::new(foot, 1.5),
                    via: Point2::new(2.0, 0.0),
                },
                ProfileSegment::LineTo(Point2::new(0.0, 1.5)),
                ProfileSegment::LineTo(Point2::new(0.0, -1.5)),
            ],
        },
        holes: Vec::new(),
    };
    let ball = revolve(&mut m, &half, Axis::z_at(Point3::origin()), 2.0 * PI)
        .map_err(fail)?
        .0;
    let pin_axis = Axis::new(Point3::new(0.0, off, 0.0), Vec3::x()).map_err(fail)?;
    let pin = primitive_cylinder(&mut m, pin_axis, 0.6, 3.0)
        .map_err(fail)?
        .0;
    let fused = fuse(&mut m, ball, pin).map_err(fail)?.0;
    let y = off + 0.6;
    let at = Point3::new((4.0 - y * y).sqrt(), y, 0.0);
    // The loop edge nearest `at`: an edge on both the ball and the pin,
    // which the seam may split anywhere.
    let mut best: Option<(f64, Edge)> = None;
    for e in m.edges(fused).map_err(fail)? {
        let Some((curve, range)) = m.edge(e.id).map_err(fail)?.curve() else {
            continue;
        };
        let q = m.curve(curve).map_err(fail)?.point(range.midpoint());
        let on_ball = (q.coords.norm() - 2.0).abs() < 1e-6;
        let on_pin = (real::hypot(q.y - off, q.z) - 0.6).abs() < 1e-6;
        let d = (q - at).norm();
        if on_ball && on_pin && best.is_none_or(|(b, _)| d < b) {
            best = Some((d, e));
        }
    }
    let edge = best
        .map(|(_, e)| e)
        .ok_or_else(|| fail(format!("no loop edge, seam at {phi}, pin at {off}")))?;
    let (out, p) = match kind {
        Blend::Fillet => fillet(&mut m, fused, &[edge], r),
        Blend::Chamfer => chamfer(&mut m, fused, &[edge], r),
    }
    .map_err(|e| fail(format!("seam at {phi}, pin at {off}, r {r}: {e}")))?;
    let report = check(&m, out, Level::Full);
    prop_assert!(
        report.is_ok(),
        "seam at {phi}, pin at {off}, r {r}: {}",
        report
    );
    audit(&m, &[fused], out, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    Ok((m, out))
}

prop_shards! {
    /// The ball's seam at a random azimuth across the pin's loop (a pin at
    /// a random offset, the seam through the loop at `y = off + t * 0.6`),
    /// filleted or chamfered at a random size: clean at `Full`, audited,
    /// and its volume and area equal the same body's with the seam through
    /// the loop a quarter radius above the pin's axis.
    /// A seam that misses the loop leaves it a ring on the pin's seam
    /// vertex alone, which is not built, and neither is one within a few
    /// hundredths of a pin radius of the pin's axis height, where either kind
    /// is refused at isolated heights: `t` skips (-0.1, 0.1) (`BACKLOG.md`).
    a_walked_blend_does_not_depend_on_where_the_seam_lies
        [shard_0 shard_1 shard_2 shard_3 shard_4 shard_5 shard_6 shard_7]
        ((t, off, r, kind)) = (
            prop::finite_f64(-0.8..=0.6),
            prop::finite_f64(0.3..=0.7),
            prop::finite_f64(0.05..=0.15),
            blend_kind(),
        ) => {
            let t = if t < -0.1 { t } else { t + 0.2 };
            let phi = real::asin((off + t * 0.6) / 2.0);
            let (m, body) = seam_ball(phi, off, kind, r)?;
            let (rm, reference) = seam_ball(real::asin((off + 0.25 * 0.6) / 2.0), off, kind, r)?;
            let (a, b) = (
                mass_properties(&m, body).map_err(fail)?,
                mass_properties(&rm, reference).map_err(fail)?,
            );
            prop_assert!(
                close_to(a.volume, b.volume, 1.0, 1e-7),
                "seam {phi}, pin {off}, r {r}: volume {} vs {}", a.volume, b.volume
            );
            prop_assert!(
                close_to(a.area, b.area, 1.0, 1e-7),
                "seam {phi}, pin {off}, r {r}: area {} vs {}", a.area, b.area
            );
            Ok(())
        }
}
