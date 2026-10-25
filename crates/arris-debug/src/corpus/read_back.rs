use super::*;

/// A sheet result's STEP, read back by Arris: one `Sheet` body that passes
/// the checker at `Full` with the written body's face count and surface
/// properties within the fixture's tolerances (the oracle has read the
/// file already, in `compare_dir`). Errors: [`CorpusError::ReadBack`].
pub(super) fn sheet_round_trip(
    fixture: &Fixture,
    chain: &Chain,
    body: Body,
    text: &str,
    expected: &Measured,
) -> Result<(), CorpusError> {
    let fail = |what: String| CorpusError::ReadBack {
        fixture: fixture.name.clone(),
        file: "sheet",
        what,
    };
    let mut model = model_for(fixture)?;
    let read = step_read(&mut model, text, &io_step::ReadOptions::default())
        .map_err(|e| fail(e.to_string()))?;
    let [only] = &read.solids[..] else {
        return Err(fail(format!("{} bodies read, not one", read.solids.len())));
    };
    let back = only
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?
        .body;
    if model.body(back.id).map_err(|e| fail(e.to_string()))?.kind() != BodyKind::Sheet {
        return Err(fail("not a sheet".into()));
    }
    let report = check_leaving_nurbs(&fixture.name, &model, back)?;
    if !report.is_ok() {
        return Err(fail(format!("fails the checker\n{report}")));
    }
    let faces = |m: &Model, b: Body| m.faces(b).map(|f| f.len()).map_err(|e| fail(e.to_string()));
    let (written, read_faces) = (faces(&chain.model, body)?, faces(&model, back)?);
    if written != read_faces {
        return Err(fail(format!(
            "{read_faces} faces read of {written} written"
        )));
    }
    let target = Measured {
        sheet: true,
        ..expected.clone()
    };
    let found = surface_properties(&chain.model, body).map_err(|e| fail(e.to_string()))?;
    let want = Measured {
        area: Some(found.area),
        centroid: Some([found.centroid.x, found.centroid.y, found.centroid.z]),
        inertia: Some(core::array::from_fn(|i| {
            core::array::from_fn(|j| found.inertia[(i, j)])
        })),
        ..target
    };
    compare_mass(
        &model,
        back,
        &want,
        "the written sheet's",
        &fixture.recipe.tolerances,
    )
    .map_err(fail)
}

/// Open CASCADE's own STEP of the fixture's result under `variant`,
/// `text`, read by `arris_io::step::read` into a model of the fixture's
/// precision (ADR-0025): every solid read — none refused — and together,
/// a lump each as the oracle counts them, held to the checker at `Full`
/// with nothing unchecked, to the oracle's own counts and genus (the file
/// is Open CASCADE's topology, so `analytic.counts_differ` does not
/// apply), and to the oracle's own measurements — the file carries Open
/// CASCADE's shape, so `analytic.measure_differs`, which blames its
/// boolean, does not apply either — at the fixture's tolerances, widened
/// to the read body's own where those are wider (ADR-0023). A fixture
/// whose `analytic.occt_step_refused` names a refusal passes on that
/// refusal and fails on a read. Errors:
/// [`CorpusError::ReadBack`] for a file that does not read, and the
/// stage errors of [`check_stage`], [`counts_stage`] and
/// [`measure_stage`].
pub fn read_back_stage(
    fixture: &Fixture,
    variant: &str,
    text: &str,
    expected: &Measured,
) -> Result<(), CorpusError> {
    read_back(fixture, variant, text, expected, "plain")
}

/// [`read_back_stage`] of Open CASCADE's STEP of the result converted to
/// B-splines by `BRepBuilderAPI_NurbsConvert` (`occt_step.py --nurbs`):
/// every face a NURBS surface, every edge a NURBS curve — the proof on
/// files of the projection, the pcurves and the rebuilt poles onto NURBS
/// (ADR-0025). Held as the plain file is, but to the
/// converted shape's own counts (`nurbs_counts` in `expected.json`, since
/// conversion can add seams), and with the rows the checker cannot
/// decide on a NURBS face left unchecked: S5's and B1's face pairs with
/// a NURBS face in them, and B1's nesting of a shell no ray is cast from,
/// which a NURBS face never answers. `analytic.occt_step_refused` passes
/// this file too where it refuses the same way, and
/// `analytic.occt_nurbs_refused` names a refusal of this file alone, which
/// passes on it and fails on a read. Errors: as [`read_back_stage`], and
/// [`CorpusError::ReadBack`] where `expected.json` records no
/// `nurbs_counts`.
pub fn read_back_nurbs_stage(
    fixture: &Fixture,
    variant: &str,
    text: &str,
    expected: &Measured,
) -> Result<(), CorpusError> {
    let counts = expected.nurbs_counts.ok_or_else(|| CorpusError::ReadBack {
        fixture: fixture.name.clone(),
        file: "NURBS",
        what: "against no nurbs_counts in expected.json: rerun expected.py".into(),
    })?;
    let converted = Measured {
        counts,
        ..expected.clone()
    };
    read_back(fixture, variant, text, &converted, "NURBS")
}

/// [`read_back_stage`] of the file `which` names.
pub(super) fn read_back(
    fixture: &Fixture,
    variant: &str,
    text: &str,
    expected: &Measured,
    which: &'static str,
) -> Result<(), CorpusError> {
    let fail = |what: String| CorpusError::ReadBack {
        fixture: fixture.name.clone(),
        file: which,
        what,
    };
    let Some(params) = fixture.recipe.params_of(variant) else {
        return Err(CorpusError::Variant {
            fixture: fixture.name.clone(),
            variant: variant.to_string(),
        });
    };
    let mut model = model_for(fixture)?;
    let read = step_read(&mut model, text, &io_step::ReadOptions::default())
        .map_err(|e| fail(e.to_string()))?;
    let nurbs = which == "NURBS";
    if let (true, Some(refused)) = (nurbs, &fixture.recipe.analytic.occt_nurbs_refused) {
        // The converted file alone is refused, for the reader's own gap:
        // the refusal it names is the pass, and a read lifts it.
        let kinds: Vec<String> = (read.solids.iter())
            .filter_map(|s| s.result.as_ref().err())
            .map(|r| r.kind().to_string())
            .collect();
        return if kinds.contains(&refused.kind) {
            Ok(())
        } else {
            Err(fail(format!(
                "with {kinds:?}, not the refusal analytic.occt_nurbs_refused names ({}): lift it if it reads",
                refused.kind
            )))
        };
    }
    if let Some(refused) = &fixture.recipe.analytic.occt_step_refused {
        // The file describes no solid by the standard: the refusal it
        // names is the pass, and a read is the failure that lifts it. The
        // converted file is passed on the same refusal where it has it,
        // and read as any other where it does not: a conversion of a
        // walked face can refuse for the reader's own reason (ADR-0025).
        let kinds: Vec<String> = (read.solids.iter())
            .filter_map(|s| s.result.as_ref().err())
            .map(|r| r.kind().to_string())
            .collect();
        if kinds.contains(&refused.kind) {
            return Ok(());
        }
        if !nurbs {
            return Err(fail(format!(
                "with {kinds:?}, not the refusal analytic.occt_step_refused names ({}): lift it if it reads",
                refused.kind
            )));
        }
    }
    let mut bodies = Vec::with_capacity(read.solids.len());
    for solid in &read.solids {
        let back = solid
            .result
            .as_ref()
            .map_err(|r| fail(format!("refused: {r}")))?;
        bodies.push(back.body);
    }
    let body = match bodies[..] {
        [] => return Err(fail("no solid".into())),
        [one] => one,
        _ => {
            // The lumps as one body, each read body's faces kept in a
            // shell of it.
            let mut shells = Vec::new();
            for &b in &bodies {
                for shell in model.shells(b).map_err(|e| fail(e.to_string()))? {
                    let faces = model.shell(shell.id).map_err(|e| fail(e.to_string()))?;
                    shells.push(faces.faces().iter().copied().map(FaceSpec::Keep).collect());
                }
            }
            let assembly = Assembly {
                shells,
                ..Assembly::default()
            };
            let tolerance = model.precision().default_tolerance;
            let (builder, _) =
                Builder::assemble(&model, tolerance, assembly).map_err(|e| fail(e.to_string()))?;
            builder
                .finish(&mut model, BodyKind::Solid)
                .map_err(|e| fail(e.to_string()))?
                .body
        }
    };
    let chain = Chain {
        model,
        steps: BTreeMap::from([(
            "read".to_string(),
            Made {
                body,
                provenance: Provenance::new(),
                inputs: Vec::new(),
                outputs: vec![body],
            },
        )]),
        profiles: BTreeMap::new(),
        result: "read".into(),
        params,
    };
    let mut held = fixture.clone();
    held.name = format!("{} [{which} OCCT STEP]", fixture.name);
    held.recipe.analytic.counts_differ = None;
    // Every `measure_differs` but one with `occt_file_measure_differs`
    // blames Open CASCADE's boolean, not its measure: its file carries its
    // own shape, which its own measurements are of.
    held.recipe.analytic.measure_differs = None;
    held.recipe.tolerances =
        within_own_tolerance(&fixture.recipe.tolerances, &chain.model, body).map_err(fail)?;
    // Open CASCADE built it: its walked blends miss their ball by more
    // than the tolerance, so a pair of one against an analytic face may be
    // a tracer's refusal (`check_leaving_nurbs`), where an Arris blend's is
    // decided.
    let report = check_leaving_nurbs(&held.name, &chain.model, body)?;
    // Counts as the oracle's, a solid per solid read — each is one of
    // the file's, as Open CASCADE counts them, and no lump query is asked
    // of shells no ray may be cast against (a NURBS face's).
    let line = report.euler().ok_or_else(|| CorpusError::Check {
        fixture: held.name.clone(),
        report: Box::new(report.clone()),
    })?;
    let found = Counts {
        vertices: line.vertices,
        edges: line.edges,
        faces: line.faces,
        loops: line.loops,
        shells: line.shells,
        solids: bodies.len(),
    };
    if found != expected.counts {
        return Err(CorpusError::Counts {
            fixture: held.name.clone(),
            expected: expected.counts,
            found,
        });
    }
    if let Some(genus) = expected.genus.filter(|&g| g != line.genus) {
        return Err(CorpusError::Genus {
            fixture: held.name.clone(),
            expected: genus,
            found: line.genus,
        });
    }
    // The closed forms hold Open CASCADE's file where the fixture blames
    // its measurement of its own shape, not the shape.
    if fixture.recipe.analytic.occt_file_measure_differs.is_some() {
        let (target, _) = measure_target(fixture, &chain.params, expected)?;
        measure_stage(&held, &chain, &target)?;
    } else {
        measure_stage(&held, &chain, expected)?;
    }
    Ok(())
}

/// The checker at `Full` on a body another kernel built — Open CASCADE's
/// file read back, a real part — with nothing violated and nothing left
/// undecided but what the checker cannot decide on a NURBS face: S5's and
/// B1's face pairs with one in them, and B1's nesting of a shell of a
/// body with one, whose rays may all be abandoned on it (ADR-0025). A
/// NURBS face of its own may be a tracer's refusal against an analytic
/// one: a kink, or a walked blend that misses its ball by more than the
/// tolerance. `name` is what the error calls the body. Errors:
/// [`CorpusError::Check`].
pub(crate) fn check_leaving_nurbs(
    name: &str,
    m: &Model,
    body: Body,
) -> Result<Report, CorpusError> {
    check_leaving(name, m, body, false)
}

/// [`check_leaving_nurbs`] on a body Arris built: a pair of a NURBS face
/// and an analytic one is decided (ADR-0061), a walked blend against its
/// neighbours among them, so only pairs of two NURBS faces are left
/// undecided. Errors: [`CorpusError::Check`].
pub(crate) fn check_leaving_nurbs_pairs(
    name: &str,
    m: &Model,
    body: Body,
) -> Result<Report, CorpusError> {
    check_leaving(name, m, body, true)
}

/// The checker at `Full` with S5's and B1's face pairs left undecided
/// where both faces are NURBS, or with `pairs` false either is.
pub(super) fn check_leaving(
    name: &str,
    m: &Model,
    body: Body,
    pairs: bool,
) -> Result<Report, CorpusError> {
    let report = check(m, body, Level::Full);
    let has_nurbs = |f: FaceId| {
        (m.face(f))
            .and_then(|f| m.surface(f.surface()))
            .is_ok_and(|s| matches!(s, Surface::Nurbs(_)))
    };
    let any_nurbs = m
        .closure(body)
        .is_ok_and(|closure| closure.faces.into_iter().any(has_nurbs));
    let undecidable = |u: &Unchecked| match u {
        Unchecked::FacePair { kinds, .. } | Unchecked::ShellFacePair { kinds, .. } => {
            let nurbs = [kinds.0, kinds.1].map(|k| k == SurfaceKind::Nurbs);
            if pairs {
                nurbs[0] && nurbs[1]
            } else {
                nurbs[0] || nurbs[1]
            }
        }
        Unchecked::ShellNesting { .. } => any_nurbs,
    };
    if !report.is_ok() || !report.unchecked().iter().all(undecidable) {
        return Err(CorpusError::Check {
            fixture: name.to_string(),
            report: Box::new(report),
        });
    }
    Ok(report)
}
