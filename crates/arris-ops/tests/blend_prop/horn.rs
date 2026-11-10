use super::support::*;

// The torus of a rim blend through the horn (ADR-0080): a cylinder's top
// rim filleted at a share of its radius, the ball's centre circle at
// `R = radius - r` above `r` (a ring torus), at it (the horn torus) and
// below it (the spindle torus), short of the contact at the axis.

/// The circle edge of `body` at height `z` on the `z` axis.
fn circle_at_height(m: &Model, body: Body, z: f64) -> Result<Edge, TestCaseError> {
    for e in m.edges(body).map_err(fail)? {
        let entity = m.edge(e.id).map_err(fail)?;
        if let Some((curve, _)) = entity.curve()
            && let Curve::Circle { frame, .. } = m.curve(curve).map_err(fail)?
            && (frame.origin().z - z).abs() < 1e-9
        {
            return Ok(e);
        }
    }
    Err(fail(format!("no circle at z = {z}")))
}

prop_shards! {
    /// A cylinder's top rim filleted at `f` of its radius, `f` from a ring
    /// torus (`f < 1/2`) through the horn (`f = 1/2`, drawn on purpose) to a
    /// spindle torus short of the contact at the axis (the cylinder taller
    /// than the largest ball), in a random pose:
    /// clean at `Full`, audits, and its volume is Pappus's closed form, the
    /// cylinder less the corner revolved about the axis.
    a_rim_blend_from_the_ring_through_the_horn_to_the_spindle_is_pappus
        [shard_0 shard_1 shard_2 shard_3]
        ((radius, height, f, pose)) = (
            prop::finite_f64(1.0..=3.0),
            prop::finite_f64(3.0..=4.5),
            prop_oneof![Just(0.5), prop::finite_f64(0.3..=0.9)],
            prop::pose(),
        ) => {
            let r = f * radius;
            let mut m = Model::default();
            let (rest, _) = primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), radius, height)
                .map_err(fail)?;
            let rim = circle_at_height(&m, rest, height)?;
            let (rounded, p) = fillet(&mut m, rest, &[rim], r)
                .map_err(|e| fail(format!("rim of {radius} filleted at {r}: {e}")))?;
            let (body, _) = transform(&mut m, rounded, &pose).map_err(fail)?;
            let report = check(&m, body, Level::Full);
            prop_assert!(report.is_ok(), "{}", report);
            audit(&m, &[rest], rounded, &p).map_err(|e| fail(format!("provenance: {e}")))?;
            let corner = (1.0 - PI / 4.0) * r * r;
            let want = PI * radius * radius * height
                - 2.0 * PI * ((radius - r) * corner + r * r * r / 6.0);
            let got = mass_properties(&m, body).map_err(fail)?.volume;
            prop_assert!(
                close_to(got, want, 1.0, REL),
                "rim of {radius} at {r}: {got} vs {want}"
            );
            Ok(())
        }
}

prop_shards! {
    /// The cap-edge chain at `r2 / r` in `(0.3, 0.7)` (the horn torus at
    /// `1/2`, drawn on purpose): a cube's vertical edge filleted at `r`, then
    /// the cap edge on top of it at `r2`, the fillet of the first blend's end
    /// arc a torus of major radius `r − r2` and minor `r2`. Every draw is
    /// built clean at `Full` with its record audited: a probe over the whole
    /// range (`r` 0.1 to 0.6 by 0.1, `r2 / r` 0.3 to 0.7 by 0.1) built every
    /// one, so no refusal is accepted, and a draw that stops building (a
    /// refusal, a horn or spindle torus the band declines) fails the
    /// property and is not counted as a pass.
    the_cap_edge_chain_builds_green
        [shard_0 shard_1]
        ((r, share)) = (
            prop::finite_f64(0.1..=0.6),
            prop_oneof![Just(0.5), prop::finite_f64(0.3..=0.7)],
        ) => {
            let r2 = share * r;
            let mut m = Model::default();
            let (cube, _) = primitive_box(
                &mut m,
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 2.0, 2.0),
            )
            .map_err(fail)?;
            let edge = edge_near(&m, cube, Point3::new(2.0, 0.0, 1.0))?;
            let (first, _) = fillet(&mut m, cube, &[edge], r).map_err(fail)?;
            let cap = edge_near(&m, first, Point3::new(1.0 - 0.5 * r, 0.0, 2.0))?;
            let (out, p) = fillet(&mut m, first, &[cap], r2)
                .map_err(|e| fail(format!("r {r}, r2 {r2} is refused: {e}")))?;
            let report = check(&m, out, Level::Full);
            prop_assert!(report.is_ok(), "r {r}, r2 {r2}: {}", report);
            audit(&m, &[first], out, &p).map_err(|e| fail(format!("provenance: {e}")))?;
            Ok(())
        }
}
