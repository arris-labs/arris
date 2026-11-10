use super::support::*;

// A miter of unequal dihedrals (ADR-0044): a prism over a random
// parallelogram, the rise at one vertex and a top edge there blended, the
// vertex's other top edge sharp. The volume is the prism less the two
// blends' removed regions taken whole along their edges, plus where those
// regions overlap — in closed form below.

/// A prism over the parallelogram `(0,0) (a,0) (a + b cos θ, b sin θ)
/// (b cos θ, b sin θ)`, `height` high, and the corner at vertex `vertex`
/// whose rise and one top edge are blended.
#[derive(Debug, Clone)]
struct SlantCase {
    a: f64,
    b: f64,
    theta: f64,
    height: f64,
    vertex: usize,
    /// `true` for the top edge leaving the vertex, `false` for the one
    /// arriving at it.
    leaving: bool,
    fraction: f64,
    pose: Isometry,
}

impl SlantCase {
    fn polygon(&self) -> [Point2; 4] {
        let (c, s) = (real::cos(self.theta), real::sin(self.theta));
        [
            Point2::new(0.0, 0.0),
            Point2::new(self.a, 0.0),
            Point2::new(self.a + self.b * c, self.b * s),
            Point2::new(self.b * c, self.b * s),
        ]
    }

    /// The interior angle at the corner's vertex.
    fn angle(&self) -> f64 {
        if self.vertex % 2 == 0 {
            self.theta
        } else {
            PI - self.theta
        }
    }

    /// The top edge's polygon vertices, and its length.
    fn top(&self) -> (usize, f64) {
        let first = if self.leaving {
            self.vertex
        } else {
            (self.vertex + 3) % 4
        };
        (first, if first % 2 == 0 { self.a } else { self.b })
    }

    /// The blend's size: a fraction of the shortest of the prism's edges
    /// the blends' contacts and the trim must stay clear of.
    fn size(&self) -> f64 {
        self.fraction * self.a.min(self.b).min(self.height)
    }

    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        let poly = self.polygon();
        let profile = Profile {
            plane: Frame::world(),
            outer: ProfileLoop::Path {
                start: poly[0],
                segments: poly[1..]
                    .iter()
                    .chain(core::iter::once(&poly[0]))
                    .map(|&q| ProfileSegment::LineTo(q))
                    .collect(),
            },
            holes: Vec::new(),
        };
        Ok(extrude(m, &profile, Vec3::z(), self.height)?.0)
    }

    /// The rise's and the top edge's midpoints.
    fn midpoints(&self) -> [Point3; 2] {
        let poly = self.polygon();
        let v = poly[self.vertex];
        let (first, _) = self.top();
        let mid = poly[first] + (poly[(first + 1) % 4] - poly[first]) / 2.0;
        [
            Point3::new(v.x, v.y, self.height / 2.0),
            Point3::new(mid.x, mid.y, self.height),
        ]
    }

    /// The volume after the blend. With `t` the interior angle at the
    /// vertex, the rise removes the corner's section over the height —
    /// `r² (cot(t/2) − (π − t)/2)` round, `d² sin t / 2` flat — and the top
    /// edge its square section over its length, `(1 − π/4) r²` or `d²/2`,
    /// the slant's gain at one end paid back at the other. The two overlap
    /// in the wall strip the top edge's section reaches, depth `n` and
    /// height `w` below the top, where the corner's section is
    /// `L(n)` wide along the wall and the top edge's reaches `n` only
    /// below `w < g(n)`: `∫ L(n) g(n) dn` to the depth where the corner's
    /// section ends, `min(r, r (1 + cos t))` round (substituting
    /// `n = r (1 − cos φ)`), `d sin t` flat.
    fn volume(&self, kind: Blend) -> f64 {
        let t = self.angle();
        let s = self.size();
        let (_, length) = self.top();
        let base = self.a * self.b * real::sin(self.theta);
        let (corner, edge, overlap) = match kind {
            Blend::Fillet => {
                let kappa = 1.0 / real::tan(t / 2.0);
                let phi = (PI - t).min(FRAC_PI_2);
                let overlap = real::powi(s, 3)
                    * simpson(0.0, phi, 2000, &|p: f64| {
                        (kappa - real::sin(p) - (1.0 - real::cos(p)) / real::tan(t))
                            * (1.0 - real::sin(p))
                            * real::sin(p)
                    });
                (
                    s * s * (kappa - (PI - t) / 2.0),
                    (1.0 - FRAC_PI_4) * s * s,
                    overlap,
                )
            }
            Blend::Chamfer => {
                let k = real::tan(t / 2.0) + 1.0 / real::tan(t);
                let n = s * real::sin(t);
                // ∫₀ⁿ (d − k x)(d − x) dx
                let overlap = s * s * n - (1.0 + k) * s * n * n / 2.0 + k * real::powi(n, 3) / 3.0;
                (s * s * real::sin(t) / 2.0, s * s / 2.0, overlap)
            }
        };
        base * self.height - corner * self.height - edge * length + overlap
    }
}

fn slant_case() -> impl Strategy<Value = SlantCase> {
    (
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(0.9..=1.35),
        prop::finite_f64(1.0..=3.0),
        0usize..4,
        any::<bool>(),
        prop::finite_f64(0.05..=0.25),
        prop::pose(),
    )
        .prop_map(
            |(a, b, theta, height, vertex, leaving, fraction, pose)| SlantCase {
                a,
                b,
                theta,
                height,
                vertex,
                leaving,
                fraction,
                pose,
            },
        )
}

/// The blend of `case`'s two edges, in the order `edges` lists them, in a
/// fresh model with the prism moved to its pose.
fn slant_posed(
    case: &SlantCase,
    kind: Blend,
    swap: bool,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let prism = case.build(&mut m).map_err(fail)?;
    let (moved, _) = transform(&mut m, prism, &case.pose).map_err(fail)?;
    let mut edges = case
        .midpoints()
        .iter()
        .map(|&p| edge_near(&m, moved, case.pose.apply(p)))
        .collect::<Result<Vec<_>, _>>()?;
    if swap {
        edges.reverse();
    }
    let (blended, p) = op(kind)(&mut m, moved, &edges, case.size())
        .map_err(|e| fail(format!("{kind:?} of the slanted corner: {e}")))?;
    Ok((m, moved, blended, p))
}

fn slants_blend_as_their_closed_forms(case: SlantCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = slant_posed(&case, kind, false)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    // A fillet miter's trim arc has a fitted pcurve on the cylinder.
    let rel = match kind {
        Blend::Fillet => fitted_rel(&m, &props),
        Blend::Chamfer => REL,
    };
    let volume = case.volume(kind);
    prop_assert!(
        close_to(props.volume, volume, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        volume
    );
    // Either order of the two edges: the same ids, so the same dump and
    // record; and the same again on a second run.
    let (swapped, _, flipped, flipped_p) = slant_posed(&case, kind, true)?;
    prop_assert_eq!(
        dump_text(&swapped, flipped).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?,
        "the edges in the other order"
    );
    prop_assert_eq!(flipped_p, p.clone());
    let (again, _, twice, again_p) = slant_posed(&case, kind, false)?;
    prop_assert_eq!(
        dump_text(&again, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(again_p, p);
    Ok(())
}

prop_shards! {
    /// Fillets of a slanted prism's rise and one top edge at a vertex, the
    /// other top edge sharp: a miter of unequal dihedrals, clean at `Full`
    /// with nothing unchecked, audited, at the closed-form volume, the same
    /// for either order of the edges and deterministic.
    slanted_fillets_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = slant_case() => {
            slants_blend_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same corners chamfered.
    slanted_chamfers_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = slant_case() => {
            slants_blend_as_their_closed_forms(case, Blend::Chamfer)
        }
}

/// The closed form at the slanted prism of
/// `blend/miter-unequal-dihedrals-fillet` and its chamfer twin: the
/// parallelogram `(0,0) (2,0) (3,2) (1,2)` is `a = 2`, `b = √5`,
/// `θ = atan 2`, its vertex 2 the corner.
#[test]
fn the_slanted_closed_form_is_the_fixtures() {
    let case = SlantCase {
        a: 2.0,
        b: 5.0_f64.sqrt(),
        theta: real::atan(2.0_f64),
        height: 2.0,
        vertex: 2,
        leaving: true,
        fraction: 0.1,
        pose: Isometry::identity(),
    };
    // 0.2 of the shortest edge `a = 2` over ten.
    assert!((case.size() - 0.2).abs() < 1e-12);
    let r: f64 = 0.2;
    let golden = (1.0 + 5.0_f64.sqrt()) / 2.0;
    let fixture = 8.0
        - 2.0 * r * r * (golden - (PI - real::atan(2.0_f64)) / 2.0)
        - 2.0 * r * r * (1.0 - FRAC_PI_4)
        + real::powi(r, 3) * (golden - 0.5) * (1.0 - FRAC_PI_4)
        + real::powi(r, 3) * (3.0 - PI) / 4.0;
    let got = case.volume(Blend::Fillet);
    assert!((got - fixture).abs() < 1e-12, "{got} against {fixture}");
}
