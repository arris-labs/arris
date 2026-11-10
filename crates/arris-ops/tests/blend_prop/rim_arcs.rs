use super::support::*;

// A rim split into arcs (ADR-0041): a hole's top edge or a boss's foot on a
// plate, its circle written as two to four arcs of one wall, the wall's seam
// at the first vertex and nothing but the two arcs at the others. No
// operation of the kernel splits an edge, so the file does it: the plate's
// own STEP text has the ring's closed edge replaced by arcs, and the part is
// read back, as a part from a writer that splits its closed edges is.

/// A plate with a hole or a boss whose rim is split, and the blend of one of
/// its arcs, which runs on round the whole ring.
#[derive(Debug, Clone)]
struct RimCase {
    /// A boss (its foot, concave) on the plate, else a hole (its top edge,
    /// convex) through it.
    boss: bool,
    radius: f64,
    /// The hole's depth, the plate's thickness; the boss stands this high on a
    /// plate 1 thick.
    height: f64,
    size: f64,
    /// The arcs' lengths as weights; the angles follow from them, the first
    /// arc starting at the wall's seam.
    weights: Vec<f64>,
    /// The arc blended.
    arc: usize,
    pose: Isometry,
}

/// `text` with the closed edge of the circle at height `z` replaced by arcs
/// from its vertex through `cuts` (angles about its axis, ascending, in
/// `(0, 2π)`), in the loops that use it.
fn split_ring(text: &str, z: f64, cuts: &[f64]) -> Result<String, TestCaseError> {
    let lines: Vec<&str> = text.lines().collect();
    let entity = |id: &str| -> Option<&str> {
        let head = format!("{id} = ");
        lines.iter().find_map(|l| l.strip_prefix(&head))
    };
    let args = |rhs: &str| -> Vec<String> {
        let inner = rhs.split_once('(').map_or("", |(_, r)| r);
        inner
            .trim_end_matches(';')
            .trim_end_matches(')')
            .split(',')
            .map(|s| s.trim().to_string())
            .collect()
    };
    let mut top = 0u64;
    let mut found: Option<(String, String, [f64; 3], String)> = None;
    for l in &lines {
        let Some((id, rhs)) = l.split_once(" = ") else {
            continue;
        };
        if let Some(n) = id.strip_prefix('#').and_then(|n| n.parse::<u64>().ok()) {
            top = top.max(n);
        }
        if !rhs.starts_with("EDGE_CURVE(") {
            continue;
        }
        let a = args(rhs);
        if a[1] != a[2] || found.is_some() {
            continue;
        }
        let vertex = entity(&a[1]).ok_or_else(|| fail("no vertex"))?;
        let point = entity(&args(vertex)[1]).ok_or_else(|| fail("no point"))?;
        let c = args(point);
        let xyz: Vec<f64> = c[1..4]
            .iter()
            .map(|s| s.trim_matches(|ch| ch == '(' || ch == ')').parse::<f64>())
            .collect::<Result<_, _>>()
            .map_err(fail)?;
        if (xyz[2] - z).abs() < 1e-9 {
            found = Some((
                id.to_string(),
                a[1].clone(),
                [xyz[0], xyz[1], xyz[2]],
                a[3].clone(),
            ));
        }
    }
    let (edge, start, at, curve) =
        found.ok_or_else(|| fail(format!("no closed edge at z = {z}")))?;
    let (radius, theta0) = (real::hypot(at[0], at[1]), real::atan2(at[1], at[0]));
    let mut next = top;
    let mut fresh = || {
        next += 1;
        format!("#{next}")
    };
    let mut add = Vec::new();
    let mut stops = vec![start.clone()];
    for cut in cuts {
        let (vertex, point) = (fresh(), fresh());
        let t = theta0 + cut;
        add.push(format!(
            "{point} = CARTESIAN_POINT('',({:?},{:?},{:?}));",
            radius * real::cos(t),
            radius * real::sin(t),
            at[2]
        ));
        add.push(format!("{vertex} = VERTEX_POINT('',{point});"));
        stops.push(vertex);
    }
    stops.push(start);
    let arcs: Vec<String> = (0..stops.len() - 1).map(|_| fresh()).collect();
    for (k, arc) in arcs.iter().enumerate() {
        add.push(format!(
            "{arc} = EDGE_CURVE('',{},{},{curve},.T.);",
            stops[k],
            stops[k + 1]
        ));
    }
    // Each use of the edge becomes its arcs, in the order the use runs.
    let mut uses: Vec<(String, Vec<String>)> = Vec::new();
    let mut out: Vec<String> = Vec::new();
    for l in &lines {
        let Some((id, rhs)) = l.split_once(" = ") else {
            out.push(l.to_string());
            continue;
        };
        if id == edge {
            continue;
        }
        if rhs.starts_with("ORIENTED_EDGE(") {
            let a = args(rhs);
            if a[3] == edge {
                let forward = a[4] == ".T.";
                let ids: Vec<String> = arcs.iter().map(|_| fresh()).collect();
                let order: Vec<usize> = if forward {
                    (0..arcs.len()).collect()
                } else {
                    (0..arcs.len()).rev().collect()
                };
                for (slot, k) in ids.iter().zip(order) {
                    add.push(format!(
                        "{slot} = ORIENTED_EDGE('',*,*,{},{});",
                        arcs[k], a[4]
                    ));
                }
                uses.push((id.to_string(), ids));
                continue;
            }
        }
        out.push(l.to_string());
    }
    let replaced: Vec<String> = out
        .into_iter()
        .map(|l| {
            let Some((id, rhs)) = l.split_once(" = ") else {
                return l;
            };
            if !rhs.starts_with("EDGE_LOOP(") {
                return l;
            }
            let members: Vec<String> = args(rhs)[1..]
                .iter()
                .flat_map(|m| {
                    let m = m.trim_start_matches('(');
                    match uses.iter().find(|(u, _)| u == m) {
                        Some((_, ids)) => ids.clone(),
                        None => vec![m.to_string()],
                    }
                })
                .collect();
            format!("{id} = EDGE_LOOP('',({}));", members.join(","))
        })
        .collect();
    let data_end = replaced.iter().rposition(|l| l == "ENDSEC;");
    let mut text = String::new();
    for (k, l) in replaced.into_iter().enumerate() {
        if Some(k) == data_end {
            for a in add.drain(..) {
                text.push_str(&a);
                text.push('\n');
            }
        }
        text.push_str(&l);
        text.push('\n');
    }
    Ok(text)
}

impl RimCase {
    /// The angles at which the arcs start, the first at the seam.
    fn starts(&self) -> Vec<f64> {
        let total: f64 = self.weights.iter().sum();
        let mut at = 0.0;
        self.weights
            .iter()
            .map(|w| {
                let start = at;
                at += 2.0 * PI * w / total;
                start
            })
            .collect()
    }

    /// The plate and its split ring, and the midpoint of the arc blended.
    fn build(&self, m: &mut Model) -> Result<(Body, Point3), TestCaseError> {
        let (r, h) = (self.radius, self.height);
        let plate_h = if self.boss { 1.0 } else { h };
        let (plate, _) = primitive_box(
            m,
            Point3::new(-3.0, -3.0, 0.0),
            Point3::new(3.0, 3.0, plate_h),
        )
        .map_err(fail)?;
        let solid = |m: &mut Model, z: f64, len: f64| {
            let axis = Axis::new(Point3::new(0.0, 0.0, z), Vec3::z()).map_err(fail)?;
            primitive_cylinder(m, axis, r, len).map_err(fail)
        };
        let whole = if self.boss {
            let (boss, _) = solid(m, 0.5, 0.5 + h)?;
            fuse(m, plate, boss).map_err(fail)?.0
        } else {
            let (hole, _) = solid(m, -1.0, h + 2.0)?;
            cut(m, plate, hole).map_err(fail)?.0
        };
        let starts = self.starts();
        let text = split_ring(
            &step::write(m, &[whole]).map_err(fail)?,
            plate_h,
            &starts[1..],
        )?;
        let mut back = Model::new(m.precision()).map_err(fail)?;
        let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
        let solid = read.solids[0]
            .result
            .as_ref()
            .map_err(|e| fail(format!("the split plate: {e}")))?;
        let body = solid.body;
        // The ring's one edge became the arcs: a split that missed would
        // leave a whole circle to blend, and pass.
        let (before, after) = (
            m.edges(whole).map_err(fail)?.len(),
            back.edges(body).map_err(fail)?.len(),
        );
        prop_assert_eq!(
            after,
            before + starts.len() - 1,
            "the ring is {} arcs",
            starts.len()
        );
        *m = back;
        let end = starts.get(self.arc + 1).copied().unwrap_or(2.0 * PI);
        let mid = (starts[self.arc] + end) / 2.0;
        Ok((
            body,
            Point3::new(r * real::cos(mid), r * real::sin(mid), plate_h),
        ))
    }

    /// The volume after blending the ring: the plate's, less (a hole) or
    /// plus (a boss's foot) the corner's section swept a whole turn about
    /// the axis, Pappus.
    fn volume(&self, kind: Blend) -> f64 {
        let (r, h, s) = (self.radius, self.height, self.size);
        let (area, offset) = match kind {
            Blend::Fillet => (
                (1.0 - PI / 4.0) * s * s,
                s * (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI),
            ),
            Blend::Chamfer => (s * s / 2.0, s / 3.0),
        };
        let swept = 2.0 * PI * (r + offset) * area;
        if self.boss {
            36.0 + PI * r * r * h + swept
        } else {
            36.0 * h - PI * r * r * h - swept
        }
    }
}

fn rim_case() -> impl Strategy<Value = RimCase> {
    (
        prop_oneof![Just(true), Just(false)],
        prop::finite_f64(0.8..=1.6),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(0.05..=0.3),
        proptest::collection::vec(prop::finite_f64(0.6..=1.4), 2..=4),
        0usize..4,
        prop::pose(),
    )
        .prop_map(|(boss, radius, height, size, weights, arc, pose)| RimCase {
            boss,
            radius,
            height,
            size,
            arc: arc % weights.len(),
            weights,
            pose,
        })
}

fn rim_posed(
    case: &RimCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let (body, at) = case.build(&mut m)?;
    assert_checked(&check(&m, body, Level::Full))?;
    let (moved, _) = transform(&mut m, body, &case.pose).map_err(fail)?;
    let edge = edge_near(&m, moved, case.pose.apply(at))?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} of the posed rim: {e}")))?;
    Ok((m, moved, blended, p))
}

fn rims_blend_as_their_closed_forms(case: RimCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = rim_posed(&case, kind)?;
    feet_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let want = case.volume(kind);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the closed form {}",
        props.volume,
        want
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    feet_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = rim_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A hole's rim or a boss's foot split into two to four arcs, one arc
    /// filleted: the chain runs on through every vertex of the split and
    /// closes, clean at `Full`, audited, at the closed-form volume, through
    /// STEP and deterministic.
    fillets_of_a_split_rim_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = rim_case() => {
            rims_blend_as_their_closed_forms(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same rims chamfered.
    chamfers_of_a_split_rim_match_their_closed_forms
        [shard_0 shard_1 shard_2 shard_3] (case) = rim_case() => {
            rims_blend_as_their_closed_forms(case, Blend::Chamfer)
        }
}
