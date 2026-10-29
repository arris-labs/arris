use super::*;

/// How a result step is expected to fail.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Refusal {
    /// `OpError::Degenerate`, any reason: the oracle recorded no solid.
    Degenerate,
    /// The recipe's `analytic.expect_error`.
    Error(ExpectError),
}

impl Refusal {
    fn expected(self) -> String {
        match self {
            Refusal::Degenerate => "OpError::Degenerate".into(),
            Refusal::Error(ExpectError::TangentContact) => {
                "OpError::Degenerate with Reason::TangentContact".into()
            }
            Refusal::Error(ExpectError::NonManifold) => {
                "OpError::Degenerate with Reason::NonManifold".into()
            }
            Refusal::Error(ExpectError::BlendTooLarge) => {
                "OpError::Degenerate with Reason::BlendTooLarge".into()
            }
            Refusal::Error(ExpectError::TangentChain) => {
                "OpError::Degenerate with Reason::TangentChain".into()
            }
            Refusal::Error(ExpectError::VertexBlend) => {
                "OpError::Degenerate with Reason::VertexBlend".into()
            }
            Refusal::Error(ExpectError::EllipticRevolve) => {
                "OpError::Degenerate with Reason::EllipticRevolve".into()
            }
            Refusal::Error(ExpectError::PathCorner) => {
                "OpError::Path with PathError::Corner".into()
            }
            Refusal::Error(ExpectError::ProfileAlongPath) => {
                "OpError::Degenerate with Reason::ProfileAlongPath".into()
            }
            Refusal::Error(ExpectError::SweepFolds) => {
                "OpError::Degenerate with Reason::SweepFolds".into()
            }
            Refusal::Error(ExpectError::FrameDoesNotClose) => {
                "OpError::Degenerate with Reason::FrameDoesNotClose".into()
            }
            Refusal::Error(ExpectError::LoftTooFewSections) => {
                "OpError::Loft with LoftError::TooFewSections".into()
            }
            Refusal::Error(ExpectError::LoftLoopCount) => {
                "OpError::Loft with LoftError::LoopCount".into()
            }
            Refusal::Error(ExpectError::LoftCoincident) => {
                "OpError::Loft with LoftError::Coincident".into()
            }
            Refusal::Error(ExpectError::Vanishes) => {
                "OpError::Degenerate with Reason::Vanishes".into()
            }
            Refusal::Error(ExpectError::VertexSplits) => {
                "OpError::Degenerate with Reason::VertexSplits".into()
            }
            Refusal::Error(ExpectError::NoExactOffset) => {
                "OpError::Degenerate with Reason::NoExactOffset".into()
            }
            Refusal::Error(ExpectError::SurfaceCollapses) => {
                "OpError::Degenerate with Reason::SurfaceCollapses".into()
            }
            Refusal::Error(ExpectError::Gap) => "OpError::Degenerate with Reason::Gap".into(),
            Refusal::Error(ExpectError::SelfIntersects) => {
                "OpError::Degenerate with Reason::SelfIntersects".into()
            }
            Refusal::Error(ExpectError::OpeningDragged) => {
                "OpError::Degenerate with Reason::OpeningDragged".into()
            }
            Refusal::Error(ExpectError::NoCrossing) => {
                "OpError::Degenerate with Reason::NoCrossing".into()
            }
            Refusal::Error(ExpectError::Nurbs) => {
                "OpError::Unsupported with a NURBS surface or curve in the pair".into()
            }
            Refusal::Error(ExpectError::Unsupported) => "OpError::Unsupported".into(),
        }
    }

    /// `Ok` when `built` is the failure expected.
    pub(super) fn assert(
        self,
        fixture: &str,
        step: &str,
        built: Result<(), CorpusError>,
    ) -> Result<(), CorpusError> {
        let found = match built {
            Ok(()) => "a body".to_string(),
            Err(CorpusError::Op {
                source: OpError::Degenerate { reason, .. },
                ..
            }) => {
                let matches = match self {
                    Refusal::Degenerate => true,
                    Refusal::Error(ExpectError::TangentContact) => {
                        reason == Reason::Boolean(BooleanReason::TangentContact)
                    }
                    Refusal::Error(ExpectError::NonManifold) => {
                        reason == Reason::Input(InputReason::NonManifold)
                    }
                    Refusal::Error(ExpectError::BlendTooLarge) => {
                        reason == Reason::Blend(BlendReason::TooLarge)
                    }
                    Refusal::Error(ExpectError::TangentChain) => {
                        reason == Reason::Blend(BlendReason::TangentChain)
                    }
                    Refusal::Error(ExpectError::VertexBlend) => {
                        reason == Reason::Blend(BlendReason::VertexBlend)
                    }
                    Refusal::Error(ExpectError::EllipticRevolve) => {
                        matches!(reason, Reason::Sweep(SweepReason::EllipticRevolve { .. }))
                    }
                    Refusal::Error(ExpectError::ProfileAlongPath) => {
                        reason == Reason::Sweep(SweepReason::ProfileAlongPath)
                    }
                    Refusal::Error(ExpectError::SweepFolds) => {
                        matches!(reason, Reason::Sweep(SweepReason::SweepFolds { .. }))
                    }
                    Refusal::Error(ExpectError::FrameDoesNotClose) => {
                        matches!(reason, Reason::Sweep(SweepReason::FrameDoesNotClose { .. }))
                    }
                    Refusal::Error(ExpectError::Vanishes) => {
                        reason == Reason::Offset(OffsetReason::Vanishes)
                    }
                    Refusal::Error(ExpectError::VertexSplits) => {
                        reason == Reason::Offset(OffsetReason::VertexSplits)
                    }
                    Refusal::Error(ExpectError::NoExactOffset) => {
                        reason == Reason::Offset(OffsetReason::NoExactOffset)
                    }
                    Refusal::Error(ExpectError::SurfaceCollapses) => {
                        reason == Reason::Offset(OffsetReason::SurfaceCollapses)
                    }
                    Refusal::Error(ExpectError::Gap) => reason == Reason::Offset(OffsetReason::Gap),
                    Refusal::Error(ExpectError::SelfIntersects) => {
                        reason == Reason::Offset(OffsetReason::SelfIntersects)
                    }
                    Refusal::Error(ExpectError::OpeningDragged) => {
                        reason == Reason::Shell(ShellReason::OpeningDragged)
                    }
                    Refusal::Error(ExpectError::NoCrossing) => {
                        reason == Reason::Split(SplitReason::NoCrossing)
                    }
                    Refusal::Error(
                        ExpectError::Nurbs
                        | ExpectError::Unsupported
                        | ExpectError::PathCorner
                        | ExpectError::LoftTooFewSections
                        | ExpectError::LoftLoopCount
                        | ExpectError::LoftCoincident,
                    ) => false,
                };
                if matches {
                    return Ok(());
                }
                format!("OpError::Degenerate with {reason}")
            }
            Err(CorpusError::Op {
                source: OpError::Unsupported { a, b },
                ..
            }) => {
                let nurbs = |k: GeomKind| {
                    matches!(
                        k,
                        GeomKind::Surface(SurfaceKind::Nurbs) | GeomKind::Curve(CurveKind::Nurbs)
                    )
                };
                if self == Refusal::Error(ExpectError::Nurbs) && (nurbs(a.0) || nurbs(b.0)) {
                    return Ok(());
                }
                if self == Refusal::Error(ExpectError::Unsupported) {
                    return Ok(());
                }
                format!(
                    "OpError::Unsupported: no closed form for {} ({}) against {} ({})",
                    a.1, a.0, b.1, b.0
                )
            }
            Err(CorpusError::Op {
                source: OpError::Loft(e),
                ..
            }) if matches!(
                (self, &e),
                (
                    Refusal::Error(ExpectError::LoftTooFewSections),
                    LoftError::TooFewSections { .. },
                ) | (
                    Refusal::Error(ExpectError::LoftLoopCount),
                    LoftError::LoopCount { .. }
                ) | (
                    Refusal::Error(ExpectError::LoftCoincident),
                    LoftError::Coincident { .. }
                )
            ) =>
            {
                return Ok(());
            }
            Err(CorpusError::Op {
                source: OpError::Path(PathError::Corner { .. }),
                ..
            }) if self == Refusal::Error(ExpectError::PathCorner) => return Ok(()),
            Err(e) => e.to_string(),
        };
        Err(CorpusError::Expectation {
            fixture: fixture.to_string(),
            step: step.to_string(),
            expected: self.expected(),
            found,
        })
    }
}

/// Every stage of [`run`] that reads no file and starts no process, in
/// its order — [`check_stage`], [`counts_stage`], [`measure_stage`],
/// [`mesh_stage`], [`probe_stage`] and [`provenance_stage`] — on a
/// recipe already built, against the oracle's `expected` result of it.
/// What the differential holds a generated recipe to (ADR-0024 §2).
/// Returns the result body. Errors: the first stage that fails.
///
/// ```no_run
/// use arris_debug::{corpus, fixtures};
///
/// let dir = fixtures::corpus_root().join("boolean/through-hole");
/// let fixture = fixtures::load(&dir).unwrap();
/// let chain = corpus::chain(&dir, "default").unwrap();
/// corpus::stages(&fixture, &chain, &fixture.expected.results["default"]).unwrap();
/// ```
pub fn stages(fixture: &Fixture, chain: &Chain, expected: &Measured) -> Result<Body, CorpusError> {
    let (body, report) = check_stage(fixture, chain)?;
    counts_stage(fixture, chain, &report, expected)?;
    let target = measure_stage(fixture, chain, expected)?;
    // A sheet bounds no volume: no closed mesh to weigh and no inside to
    // probe (the mesh of a sheet is `tests/sheet_operands.rs`'s).
    if !expected.sheet {
        mesh_stage(fixture, chain, &target)?;
        probe_stage(fixture, chain, expected)?;
    }
    provenance_stage(fixture, chain)?;
    Ok(body)
}

/// The result body of `chain`. Errors: [`CorpusError::Reference`] when
/// the recipe's result names no body step.
pub(super) fn result_of(fixture: &Fixture, chain: &Chain) -> Result<Body, CorpusError> {
    chain.result().ok_or_else(|| CorpusError::Reference {
        fixture: fixture.name.clone(),
        step: "result".into(),
        name: chain.result.clone(),
    })
}

/// The checker at `Full` on the result, with nothing left undecided but
/// what the checker cannot decide on NURBS faces: a result with no NURBS
/// face is held to every row. A blend's leaves only its pairs of two NURBS
/// faces (`check_leaving_nurbs_blended`): a walked blend against the
/// analytic faces beside it is decided, but a NURBS face the blend did not
/// make, its operand's, keeps the read operand's rule. Any other result with them — an
/// operand read from a file of B-splines — leaves its pairs with one in
/// them that a tracer refuses (`check_leaving_nurbs`), as the read-back
/// and the battery do. Returns the result body and the report. Errors:
/// [`CorpusError::Check`].
pub fn check_stage(fixture: &Fixture, chain: &Chain) -> Result<(Body, Report), CorpusError> {
    let body = result_of(fixture, chain)?;
    // A chain the read-back made of the fixture's file names another
    // result: its provenance is the reader's, whose faces are all `Generated`.
    let blended = chain.result == fixture.recipe.result
        && (fixture.recipe.steps.iter()).any(|step| {
            step.name() == fixture.recipe.result
                && matches!(step, Step::Fillet { .. } | Step::Chamfer { .. })
        });
    let made = chain.steps.get(&chain.result).filter(|_| blended);
    let report = if let Some(made) = made {
        check_leaving_nurbs_blended(&fixture.name, &chain.model, body, &made.provenance)?
    } else {
        check_leaving_nurbs(&fixture.name, &chain.model, body)?
    };
    Ok((body, report))
}

/// Counts and genus of the result, from [`check_stage`]'s `report`,
/// against the oracle's — a solid per lump, as the oracle counts them —
/// or the recipe's own under `analytic.counts_differ`. Errors:
/// [`CorpusError::Counts`], [`CorpusError::Genus`], and
/// [`CorpusError::Lumps`] for a result whose lumps cannot be read.
pub fn counts_stage(
    fixture: &Fixture,
    chain: &Chain,
    report: &Report,
    expected: &Measured,
) -> Result<(), CorpusError> {
    let name = fixture.name.clone();
    let body = result_of(fixture, chain)?;
    let line = report.euler().ok_or_else(|| CorpusError::Check {
        fixture: name.clone(),
        report: Box::new(report.clone()),
    })?;
    let solids = if expected.sheet {
        0
    } else {
        lumps(&chain.model, body)
            .map_err(|source| CorpusError::Lumps {
                fixture: name.clone(),
                source,
            })?
            .len()
    };
    let found = Counts {
        vertices: line.vertices,
        edges: line.edges,
        faces: line.faces,
        loops: line.loops,
        shells: line.shells,
        solids,
    };
    // The oracle's counts, unless the recipe states a convention Arris
    // does not follow and gives its own.
    let analytic = &fixture.recipe.analytic;
    let counts = match (&analytic.counts_differ, analytic.counts) {
        (Some(_), Some(own)) => own,
        _ => expected.counts,
    };
    if found != counts {
        return Err(CorpusError::Counts {
            fixture: name,
            expected: counts,
            found,
        });
    }
    // The oracle's genus, unless the recipe's own counts change it.
    let genus = match (&analytic.counts_differ, analytic.genus) {
        (Some(_), Some(own)) => Some(own),
        _ => expected.genus,
    };
    if let Some(genus) = genus {
        if line.genus != genus {
            return Err(CorpusError::Genus {
                fixture: name,
                expected: genus,
                found: line.genus,
            });
        }
    }
    Ok(())
}

/// What [`measure_stage`] held the result to: the oracle's measurements,
/// or the recipe's closed forms under `analytic.measure_differs`.
#[derive(Debug, Clone, PartialEq)]
pub struct Target {
    /// The numbers.
    pub measured: Measured,
    /// Whose they are, as the errors say it: "the oracle's" or "the
    /// closed form's".
    pub by: &'static str,
}

/// `true` when the recipe starts from a file's solid, a `step` operand.
pub(super) fn reads_a_file(fixture: &Fixture) -> bool {
    (fixture.recipe.steps.iter()).any(|s| matches!(s, Step::Read { .. }))
}

/// `measure` over the B-Rep against what the oracle measured of the same
/// recipe — or, where the recipe says the oracle is wrong, its closed
/// forms (ADR-0015): volume, area, centroid and the inertia tensor, each
/// within the fixture's tolerance for it — widened to the result's own
/// ([`within_own_tolerance`]) where the recipe reads a file's solid, a
/// `step` operand. Returns the [`Target`] held to.
/// Errors: [`CorpusError::Measure`] naming the quantity.
pub fn measure_stage(
    fixture: &Fixture,
    chain: &Chain,
    expected: &Measured,
) -> Result<Target, CorpusError> {
    let body = result_of(fixture, chain)?;
    let (measured, by) = measure_target(fixture, chain, expected)?;
    let fail = |what| CorpusError::Measure {
        fixture: fixture.name.clone(),
        what,
    };
    // A recipe on a file's solid holds its result as the part's own
    // reading and battery hold it: within the tolerances the file gave
    // the body, which a kernel-built primitive's defaults do not size
    // (ADR-0023, ADR-0026 §6).
    let tolerances = if reads_a_file(fixture) {
        within_own_tolerance(&fixture.recipe.tolerances, &chain.model, body).map_err(fail)?
    } else {
        fixture.recipe.tolerances
    };
    compare_mass(&chain.model, body, &measured, by, &tolerances).map_err(fail)?;
    Ok(Target { measured, by })
}

/// How many chords [`within_own_tolerance`] measures an edge's length
/// over: a first-order bound needs the length to a few per cent, which a
/// polyline of this many chords gives on any curve the corpus has.
pub(super) const OWN_TOLERANCE_CHORDS: usize = 64;

/// `tolerances` widened to what the body itself declares (ADR-0023): a
/// reader may move any point of the boundary by up to its largest vertex
/// tolerance `t` and hand back the same shape, so a round trip of it is
/// held to the first-order change such a move makes and no closer — over
/// a boundary of area `A` and volume `V`, edges of total length `L`, and
/// the corners of its box within `R` of the centroid, `A·t` of volume,
/// `L·t` of area, `A·t·R / V` of centroid and `A·t·R²` of each inertia
/// component, as the oracle's `within_own_tolerance` widens its own. A
/// tolerance of the fixture's that is wider stays. Errors: what the
/// model or `mass_properties` refuses, said in words.
///
/// ```
/// use arris_debug::corpus::within_own_tolerance;
/// use arris_debug::fixtures::Tolerances;
/// use arris_debug::sample;
/// use arris_topo::Model;
///
/// let mut m = Model::default();
/// let body = sample::cylinder(&mut m, 4.0, 12.0)?;
/// let own = within_own_tolerance(&Tolerances::default(), &m, body)?;
/// assert!(own.volume_rel >= Tolerances::default().volume_rel);
/// # Ok::<(), Box<dyn std::error::Error>>(())
/// ```
pub fn within_own_tolerance(
    tolerances: &Tolerances,
    m: &Model,
    body: Body,
) -> Result<Tolerances, String> {
    let closure = m.closure(body).map_err(|e| e.to_string())?;
    let mut t: f64 = 0.0;
    let mut corners: Vec<Point3> = Vec::new();
    for &v in &closure.vertices {
        let v = m.vertex(v).map_err(|e| e.to_string())?;
        t = t.max(v.tolerance());
        corners.push(v.point());
    }
    let mut length = 0.0;
    for &e in &closure.edges {
        let e = m.edge(e).map_err(|e| e.to_string())?;
        let Some((curve, range)) = e.curve() else {
            continue;
        };
        let curve = m.curve(curve).map_err(|e| e.to_string())?;
        let points: Vec<Point3> = (0..=OWN_TOLERANCE_CHORDS)
            .map(|i| curve.point(range.lerp(i as f64 / OWN_TOLERANCE_CHORDS as f64)))
            .collect();
        length += points.windows(2).map(|w| (w[1] - w[0]).norm()).sum::<f64>();
        if let Some(b) = curve.bounds(range) {
            corners.push(Point3::new(b.min[0], b.min[1], b.min[2]));
            corners.push(Point3::new(b.max[0], b.max[1], b.max[2]));
        }
    }
    // A sheet bounds no volume: its properties are the surface's, and
    // moving its boundary by `t` changes the area by `L·t`, the centroid
    // by `L·t·R / A` and each inertia component by `L·t·R²`.
    let sheet = m.body(body.id).map_err(|e| e.to_string())?.kind() == BodyKind::Sheet;
    let (volume, area, centroid, inertia) = if sheet {
        let p = surface_properties(m, body).map_err(|e| e.to_string())?;
        (None, p.area, p.centroid, p.inertia)
    } else {
        let p = mass_properties(m, body).map_err(|e| e.to_string())?;
        (Some(p.volume.abs()), p.area, p.centroid, p.inertia)
    };
    let (lo, hi) = corners.iter().fold(
        (Vec3::repeat(f64::INFINITY), Vec3::repeat(f64::NEG_INFINITY)),
        |(lo, hi), p| (lo.inf(&p.coords), hi.sup(&p.coords)),
    );
    let mut reach: f64 = 0.0;
    for x in [lo.x, hi.x] {
        for y in [lo.y, hi.y] {
            for z in [lo.z, hi.z] {
                reach = reach.max((Point3::new(x, y, z) - centroid).norm());
            }
        }
    }
    let scale = inertia.iter().fold(0.0, |a: f64, x| a.max(x.abs()));
    let scale = if scale > 0.0 { scale } else { 1.0 };
    Ok(match volume {
        Some(volume) => Tolerances {
            volume_rel: tolerances.volume_rel.max(area * t / volume),
            area_rel: tolerances.area_rel.max(length * t / area),
            centroid_abs: tolerances.centroid_abs.max(area * t * reach / volume),
            inertia_rel: tolerances.inertia_rel.max(area * t * reach * reach / scale),
            ..*tolerances
        },
        None => Tolerances {
            area_rel: tolerances.area_rel.max(length * t / area),
            centroid_abs: tolerances.centroid_abs.max(length * t * reach / area),
            inertia_rel: tolerances
                .inertia_rel
                .max(length * t * reach * reach / scale),
            ..*tolerances
        },
    })
}

/// The mesh at the fixture's `mesh_chord`: closed, positive, with the
/// corner block's ADR-0012 invariants on every face-local vertex (so the
/// whole corpus covers them rather than one focused test), and its
/// signed volume within `mesh_volume_rel` of `target`'s. Errors:
/// [`CorpusError::Mesh`].
pub fn mesh_stage(fixture: &Fixture, chain: &Chain, target: &Target) -> Result<(), CorpusError> {
    let body = result_of(fixture, chain)?;
    mesh_check(
        &fixture.name,
        &chain.model,
        body,
        &fixture.recipe.tolerances,
        target,
    )?;
    mesh_keeps_stage(fixture, chain, body)
}

/// Every edge the result shares by id with a step of the recipe's
/// `mesh_keeps` has the same polyline, bit for bit, in the mesh of each at
/// `mesh_chord`. Errors: [`CorpusError::Mesh`], naming the edge and the
/// two counts; [`CorpusError::Reference`] for a step that is no body.
pub(super) fn mesh_keeps_stage(
    fixture: &Fixture,
    chain: &Chain,
    result: Body,
) -> Result<(), CorpusError> {
    if fixture.recipe.mesh_keeps.is_empty() {
        return Ok(());
    }
    let chord = fixture.recipe.tolerances.mesh_chord;
    let mesh_failure = |what: String| CorpusError::Mesh {
        fixture: fixture.name.clone(),
        chord,
        what,
    };
    let request = MeshRequest::new(chord);
    let mesh_of = |body: Body| {
        arris_mesh::tessellate_with(&chain.model, body, &request, &Control::NONE)
            .map_err(|e| mesh_failure(e.to_string()))
    };
    let after = mesh_of(result)?;
    let polyline = |mesh: &arris_mesh::TriMesh, e: arris_topo::EdgeId| -> Vec<[f64; 3]> {
        mesh.edge_polyline(e)
            .unwrap_or(&[])
            .iter()
            .filter_map(|&i| mesh.positions().get(i as usize).copied())
            .collect()
    };
    for name in &fixture.recipe.mesh_keeps {
        let made = chain
            .steps
            .get(name)
            .ok_or_else(|| CorpusError::Reference {
                fixture: fixture.name.clone(),
                step: "mesh_keeps".into(),
                name: name.clone(),
            })?;
        let before = mesh_of(made.body)?;
        let kept = chain
            .model
            .closure(made.body)
            .map_err(|e| mesh_failure(e.to_string()))?
            .edges;
        for e in kept {
            let (then, now) = (polyline(&before, e), polyline(&after, e));
            if now.is_empty() {
                continue;
            }
            if then != now {
                return Err(mesh_failure(format!(
                    "edge {e} is kept from {name:?} and meshed as {} points there, {} in the result",
                    then.len(),
                    now.len()
                )));
            }
        }
    }
    Ok(())
}

/// Every face of `body` meshed alone by [`arris_mesh::tessellate_faces`]
/// and the parts welded, in reverse order, equals `whole`: the same edge
/// polylines and the same triangles per face, as positions, bit for bit,
/// the same face-local normals and (u, v)s, and a closed mesh.
pub(super) fn weld_stage(
    m: &Model,
    body: Body,
    request: &MeshRequest,
    whole: &arris_mesh::TriMesh,
) -> Result<(), String> {
    let faces = m.faces(body).map_err(|e| e.to_string())?;
    let mut parts = Vec::with_capacity(faces.len());
    for f in &faces {
        parts.push(
            arris_mesh::tessellate_faces(m, body, &[f.id], request, &Control::NONE)
                .map_err(|e| format!("{} meshed alone: {e}", f.id))?,
        );
    }
    parts.reverse();
    let welded = arris_mesh::TriMesh::weld(&parts).map_err(|e| format!("weld: {e}"))?;
    if !welded.is_closed() {
        return Err("the faces meshed one by one and welded are not closed".into());
    }
    let triples = |mesh: &arris_mesh::TriMesh, face| -> Option<Vec<[[f64; 3]; 3]>> {
        let range = mesh.faces().iter().find(|r| r.face == face)?;
        Some(
            mesh.triangles()[range.triangles.clone()]
                .iter()
                .map(|t| t.map(|i| mesh.positions()[i as usize]))
                .collect(),
        )
    };
    let polyline = |mesh: &arris_mesh::TriMesh, e| -> Option<Vec<[f64; 3]>> {
        Some(
            mesh.edge_polyline(e)?
                .iter()
                .map(|&i| mesh.positions()[i as usize])
                .collect(),
        )
    };
    for f in &faces {
        if triples(&welded, f.id) != triples(whole, f.id) {
            return Err(format!("{}'s triangles differ face by face", f.id));
        }
        let (a, b) = (
            welded.corners().and_then(|c| c.face(f.id)),
            whole.corners().and_then(|c| c.face(f.id)),
        );
        if let (Some(a), Some(b)) = (a, b) {
            let (wc, oc) = (welded.corners().ok_or("")?, whole.corners().ok_or("")?);
            if wc.normals()[a.vertices.clone()] != oc.normals()[b.vertices.clone()]
                || wc.uvs()[a.vertices.clone()] != oc.uvs()[b.vertices.clone()]
            {
                return Err(format!(
                    "{}'s corner normals or (u, v)s differ face by face",
                    f.id
                ));
            }
        } else if a.is_some() != b.is_some() {
            return Err(format!("{}'s corner block differs face by face", f.id));
        }
    }
    if welded.edges().len() != whole.edges().len() {
        return Err(format!(
            "{} edges face by face, {} in one call",
            welded.edges().len(),
            whole.edges().len()
        ));
    }
    for range in whole.edges() {
        if polyline(&welded, range.edge) != polyline(whole, range.edge) {
            return Err(format!("{}'s polyline differs face by face", range.edge));
        }
    }
    Ok(())
}

/// [`mesh_stage`] of `body` in `m`, at `tolerances`; `name` is what the
/// error calls it.
pub(crate) fn mesh_check(
    name: &str,
    m: &Model,
    body: Body,
    tolerances: &Tolerances,
    target: &Target,
) -> Result<(), CorpusError> {
    let mesh_failure = |what: String| CorpusError::Mesh {
        fixture: name.to_string(),
        chord: tolerances.mesh_chord,
        what,
    };
    let request = MeshRequest::new(tolerances.mesh_chord).with_corners();
    let mesh = arris_mesh::tessellate_with(m, body, &request, &Control::NONE)
        .map_err(|e| mesh_failure(e.to_string()))?;
    corners_stage(m, body, &mesh).map_err(mesh_failure)?;
    let Some(mesh_volume) = mesh.signed_volume() else {
        return Err(mesh_failure("the mesh is not closed".into()));
    };
    if !mesh_volume.is_finite() || mesh_volume <= 0.0 {
        return Err(mesh_failure(format!(
            "the mesh's signed volume is {mesh_volume}, not positive"
        )));
    }
    weld_stage(m, body, &request, &mesh).map_err(mesh_failure)?;
    if let Some(volume) = target.measured.volume {
        let relative = (mesh_volume - volume).abs() / volume.abs();
        if relative.is_nan() || relative > tolerances.mesh_volume_rel {
            return Err(mesh_failure(format!(
                "mesh volume {mesh_volume} vs {} {volume}: {relative:e} relative, above mesh_volume_rel {:e}",
                target.by, tolerances.mesh_volume_rel
            )));
        }
    }
    Ok(())
}

/// Arris's classification of each of the oracle's probe points against
/// the oracle's, exactly. Both sides have their own tolerance for "on" —
/// the fixture's `probe` for the oracle, the entities' own for Arris —
/// and a probe is placed so that the two agree; a disagreement is a
/// finding, never something a band is widened to cover. Errors:
/// [`CorpusError::Probe`].
pub fn probe_stage(
    fixture: &Fixture,
    chain: &Chain,
    expected: &Measured,
) -> Result<(), CorpusError> {
    let body = result_of(fixture, chain)?;
    for probe in &expected.probes {
        let point = Point3::new(probe.point[0], probe.point[1], probe.point[2]);
        let found = match classify_point(&chain.model, body, point) {
            Ok(Classification::Inside) => Ok(Class::In),
            Ok(Classification::Outside) => Ok(Class::Out),
            Ok(Classification::On(_)) => Ok(Class::On),
            Err(e) => Err(e.to_string()),
        };
        let matches = match &found {
            Ok(class) => *class == probe.class,
            Err(_) => false,
        };
        if !matches {
            return Err(CorpusError::Probe {
                fixture: fixture.name.clone(),
                label: probe.label.clone(),
                point: probe.point,
                expected: probe.class,
                found: match found {
                    Ok(class) => format!("{class:?}"),
                    Err(e) => e,
                },
            });
        }
    }
    Ok(())
}

/// The provenance accounting of every body step, in recipe order (a
/// profile step makes none). Errors: [`CorpusError::Provenance`] naming
/// the step.
pub fn provenance_stage(fixture: &Fixture, chain: &Chain) -> Result<(), CorpusError> {
    for step in &fixture.recipe.steps {
        let Some(out) = chain.steps.get(step.name()) else {
            continue;
        };
        arris_topo::provenance::audit_many(
            &chain.model,
            &out.inputs,
            &out.outputs,
            &out.provenance,
        )
        .map_err(|e| CorpusError::Provenance {
            fixture: fixture.name.clone(),
            step: step.name().to_string(),
            what: e.to_string(),
        })?;
    }
    Ok(())
}

/// ADR-0012's invariants on every face-local vertex of the mesh: the
/// face's surface at the corner's own (u, v) is the shared position it
/// stands on, within the largest tolerance of the face and the edges and
/// vertices bounding it — a corner on an edge stands on the edge's curve
/// and takes its (u, v) from the pcurve, which agree to the edge's
/// tolerance and not the face's (docs/DATA-MODEL.md §Tolerances,
/// ADR-0026's amendment of step 5) — and its normal is the
/// surface's own in the face use's sense — outward — wherever the
/// parametrisation is not singular, and a unit vector in the tangent
/// plane where it is.
pub(super) fn corners_stage(m: &Model, body: Body, mesh: &TriMesh) -> Result<(), String> {
    let corners = mesh
        .corners()
        .ok_or_else(|| "the corner block was asked for and is missing".to_string())?;
    let faces = m.faces(body).map_err(|e| e.to_string())?;
    if corners.faces().len() != faces.len() {
        return Err(format!(
            "{} corner faces for {} faces",
            corners.faces().len(),
            faces.len()
        ));
    }
    for (used, cf) in faces.iter().zip(corners.faces()) {
        if cf.face != used.id {
            return Err(format!(
                "corner face {} where the body has {}",
                cf.face, used.id
            ));
        }
        let face = m.face(used.id).map_err(|e| e.to_string())?;
        let surface = m.surface(face.surface()).map_err(|e| e.to_string())?;
        let reversed = used.orientation == Orientation::Reversed;
        let mut within = face.tolerance();
        for coedge in face.loops().iter().flat_map(|l| l.coedges()) {
            let edge = m.edge(coedge.edge()).map_err(|e| e.to_string())?;
            within = within.max(edge.tolerance());
            for v in [edge.start(), edge.end()] {
                within = within.max(m.vertex(v).map_err(|e| e.to_string())?.tolerance());
            }
        }
        for i in cf.vertices.clone() {
            let ([u, v], normal, shared) = (
                corners.uvs()[i],
                corners.normals()[i],
                corners.positions()[i] as usize,
            );
            let at = Point3::from(mesh.positions()[shared]);
            let on_surface = surface.point(u, v);
            let off = (on_surface - at).norm();
            if off.is_nan() || off > within {
                return Err(format!(
                    "{}: ({u}, {v}) evaluates to {on_surface}, {off:e} from the {at} it stands on, \
                     above the face's and its boundary's tolerance {within:e}",
                    cf.face
                ));
            }
            let n = Vec3::from(normal);
            if let Some(own) = surface.normal(u, v) {
                let outward = if reversed {
                    -own.into_inner()
                } else {
                    own.into_inner()
                };
                if (n - outward).norm() > CORNER_NORMAL_SLACK {
                    return Err(format!(
                        "{}: the normal at ({u}, {v}) is {n}, not the outward {outward}",
                        cf.face
                    ));
                }
            } else {
                let e = surface.eval(u, v);
                for d in [e.du, e.dv] {
                    if n.dot(&d).abs() > CORNER_NORMAL_SLACK * d.norm().max(1.0) {
                        return Err(format!(
                            "{}: the normal at the singular ({u}, {v}) leaves the tangent plane",
                            cf.face
                        ));
                    }
                }
            }
        }
    }
    Ok(())
}

/// How far a corner normal may lie from the direction it should be: the
/// two are the same computation, so this is rounding on a unit vector,
/// not a geometric tolerance.
pub(super) const CORNER_NORMAL_SLACK: f64 = 1e-12;

/// What the measure stage holds a result to, and whose it is: the
/// oracle's measurements, or under `analytic.measure_differs` the
/// recipe's closed forms of the volume, area, centroid and inertia, each
/// required (ADR-0015), or a second oracle's record of them for the
/// chain's variant where the fixture carries one (ADR-0073).
pub(super) fn measure_target(
    fixture: &Fixture,
    chain: &Chain,
    expected: &Measured,
) -> Result<(Measured, &'static str), CorpusError> {
    let analytic = &fixture.recipe.analytic;
    if analytic.measure_differs.is_none() {
        return Ok((expected.clone(), "the oracle's"));
    }
    if let Some(second) = &fixture.second_oracle {
        let Some(held) = second.results.get(&chain.variant) else {
            return Err(CorpusError::Measure {
                fixture: fixture.name.clone(),
                what: format!(
                    "{SECOND_ORACLE} has no result for variant {:?}",
                    chain.variant
                ),
            });
        };
        let mut target = expected.clone();
        target.volume = Some(held.volume);
        target.area = Some(held.area);
        target.centroid = Some(held.centroid);
        target.inertia = Some(held.inertia);
        return Ok((target, "the second oracle's"));
    }
    let params = &chain.params;
    let missing = |field: &str| CorpusError::Measure {
        fixture: fixture.name.clone(),
        what: format!("analytic.measure_differs needs analytic.{field}"),
    };
    let eval = |n: &Num| {
        n.eval(params).map_err(|source| CorpusError::Expression {
            fixture: fixture.name.clone(),
            step: "analytic".into(),
            source,
        })
    };
    let mut target = expected.clone();
    target.volume = Some(eval(
        analytic.volume.as_ref().ok_or_else(|| missing("volume"))?,
    )?);
    target.area = Some(eval(
        analytic.area.as_ref().ok_or_else(|| missing("area"))?,
    )?);
    let centroid = analytic
        .centroid
        .as_ref()
        .ok_or_else(|| missing("centroid"))?;
    target.centroid = Some([
        eval(&centroid[0])?,
        eval(&centroid[1])?,
        eval(&centroid[2])?,
    ]);
    let inertia = analytic
        .inertia
        .as_ref()
        .ok_or_else(|| missing("inertia"))?;
    let mut rows = [[0.0; 3]; 3];
    for (row, forms) in rows.iter_mut().zip(inertia) {
        for (x, form) in row.iter_mut().zip(forms) {
            *x = eval(form)?;
        }
    }
    target.inertia = Some(rows);
    Ok((target, "the closed form's"))
}

/// Compares Arris's mass properties against the oracle's, quantity by
/// quantity: volume and area relative, the centroid's distance
/// absolute, each component of the inertia tensor relative to the
/// tensor's largest one, so a product of inertia that cancels to zero is
/// not compared against itself. A quantity the oracle did not record is
/// skipped. Errors: the first quantity that differs, named with both
/// values.
pub(crate) fn compare_mass(
    m: &Model,
    body: Body,
    expected: &Measured,
    by: &str,
    tolerances: &Tolerances,
) -> Result<(), String> {
    // A sheet has no volume: its area, centroid and inertia are the
    // surface properties at unit area density.
    let (volume, area, found_centroid, tensor) = if expected.sheet {
        let found = surface_properties(m, body).map_err(|e| e.to_string())?;
        (None, found.area, found.centroid, found.inertia)
    } else {
        let found = mass_properties(m, body).map_err(|e| e.to_string())?;
        (
            Some(found.volume),
            found.area,
            found.centroid,
            found.inertia,
        )
    };
    let relative = |name: &str, a: f64, e: f64, tolerance: f64| -> Result<(), String> {
        let difference = (a - e).abs() / e.abs().max(a.abs()).max(f64::MIN_POSITIVE);
        if difference.is_nan() || difference > tolerance {
            return Err(format!(
                "{name} {a} vs {by} {e}: {difference:e} relative, above {tolerance:e}"
            ));
        }
        Ok(())
    };
    if let (Some(found), Some(volume)) = (volume, expected.volume) {
        relative("volume", found, volume, tolerances.volume_rel)?;
    }
    if let Some(expected_area) = expected.area {
        relative("area", area, expected_area, tolerances.area_rel)?;
    }
    if let Some(centroid) = expected.centroid {
        let oracle = Point3::new(centroid[0], centroid[1], centroid[2]);
        let distance = (found_centroid - oracle).norm();
        if distance.is_nan() || distance > tolerances.centroid_abs {
            return Err(format!(
                "centroid {} vs {by} {oracle}: {distance:e} apart, above centroid_abs {:e}",
                found_centroid, tolerances.centroid_abs
            ));
        }
    }
    if let Some(inertia) = expected.inertia {
        let scale = inertia
            .iter()
            .flatten()
            .fold(0.0f64, |m, x| m.max(x.abs()))
            .max(f64::MIN_POSITIVE);
        for (i, row) in inertia.iter().enumerate() {
            for (j, &e) in row.iter().enumerate() {
                let a = tensor[(i, j)];
                let difference = (a - e).abs() / scale;
                if difference.is_nan() || difference > tolerances.inertia_rel {
                    return Err(format!(
                        "inertia[{i}][{j}] {a} vs {by} {e}: {difference:e} of the tensor, above inertia_rel {:e}",
                        tolerances.inertia_rel
                    ));
                }
            }
        }
    }
    Ok(())
}
