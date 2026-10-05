//! The STEP reader on Arris's own files (ADR-0025): the STEP of every corpus fixture reads back to one
//! solid with the fixture's counts, its degenerate edges — which the
//! writer leaves out — rebuilt, and a volume, area and centroid within
//! its tolerances, its provenance naming the file entity of every entity.

use arris_debug::unmetered::step_read;
use std::collections::BTreeMap;

use arris_check::{Level, check};
use arris_debug::corpus::{self, Chain, Made};
use arris_debug::fixtures::{self, DUMPED_AREAS, Fixture};
use arris_io::step::{self, ReadBody, ReadOptions};
use arris_topo::builder::{Assembly, Builder, FaceSpec};
use arris_topo::entity::BodyKind;
use arris_topo::provenance::{Origin, Relation, Role};
use arris_topo::{Body, Edge, Face, FileEntity, Model, Shape, Shell, Vertex};

/// The number of degenerate edges of the body.
fn degenerate_edges(m: &Model, body: Body) -> Result<usize, String> {
    let closure = m.closure(body).map_err(|e| e.to_string())?;
    let mut n = 0;
    for &e in &closure.edges {
        if m.edge(e).map_err(|e| e.to_string())?.is_degenerate() {
            n += 1;
        }
    }
    Ok(n)
}

/// Writes the fixture's result under `variant`, reads it back into a
/// model of the fixture's precision, and holds it to the fixture's
/// checker, counts and measure stages, and to as many degenerate edges
/// as the result has, which the writer leaves out and the reader
/// rebuilds. `Ok(None)` for a variant with no solid to write, else the
/// number of degenerate edges read back.
fn read_back(fixture: &Fixture, variant: &str) -> Result<Option<usize>, String> {
    let Some(expected) = fixture.expected.results.get(variant) else {
        return Err("no expected result".into());
    };
    if expected.degenerate || fixture.recipe.analytic.expect_error.is_some() {
        return Ok(None);
    }
    let chain = corpus::chain(&fixture.dir, variant).map_err(|e| e.to_string())?;
    let body = chain.result().ok_or("no result")?;
    let original = body;
    let degenerate = degenerate_edges(&chain.model, body)?;
    let text = step::write(&chain.model, &[body]).map_err(|e| e.to_string())?;
    let mut model = Model::new(chain.model.precision()).map_err(|e| e.to_string())?;
    let read = step_read(&mut model, &text, &ReadOptions::default()).map_err(|e| e.to_string())?;
    // The writer writes a solid per lump (ADR-0006), and the oracle
    // counts them so.
    let lumps = arris_check::lumps(&chain.model, body)
        .map_err(|e| e.to_string())?
        .len();
    if read.solids.len() != lumps {
        return Err(format!(
            "{} solids read of {lumps} lumps",
            read.solids.len()
        ));
    }
    let mut bodies = Vec::new();
    for solid in &read.solids {
        let back = solid.result.as_ref().map_err(|r| format!("refused: {r}"))?;
        provenance_names_the_file(&model, back, solid.entity)?;
        bodies.push(back.clone());
    }
    // The lumps as the one body the fixture's result is, each read body's
    // faces kept in a shell of it.
    let body = if bodies.len() == 1 {
        bodies[0].body
    } else {
        let mut shells = Vec::new();
        for b in &bodies {
            for shell in model.shells(b.body).map_err(|e| e.to_string())? {
                let faces = model.shell(shell.id).map_err(|e| e.to_string())?.faces();
                shells.push(faces.iter().copied().map(FaceSpec::Keep).collect());
            }
        }
        let assembly = Assembly {
            shells,
            ..Assembly::default()
        };
        let tolerance = model.precision().default_tolerance;
        let (builder, _) =
            Builder::assemble(&model, tolerance, assembly).map_err(|e| e.to_string())?;
        builder
            .finish(&mut model, BodyKind::Solid)
            .map_err(|e| e.to_string())?
            .body
    };

    let read_chain = Chain {
        model,
        steps: BTreeMap::from([(
            "read".to_string(),
            Made {
                body,
                provenance: bodies[0].provenance.clone(),
                inputs: Vec::new(),
            },
        )]),
        profiles: BTreeMap::new(),
        result: "read".into(),
        params: chain.params.clone(),
    };
    let fast = check(&read_chain.model, body, Level::Fast);
    if !fast.is_ok() {
        return Err(format!("the checker at Fast:\n{fast}"));
    }
    let (_, report) = corpus::check_stage(fixture, &read_chain).map_err(|e| e.to_string())?;
    corpus::counts_stage(fixture, &read_chain, &report, expected).map_err(|e| e.to_string())?;
    // A result an operation left with tolerances raised past the model's
    // default is read back to them (ADR-0025 §4), and a round trip of it
    // is held to its own tolerance (ADR-0023); every other to the
    // fixture's.
    let default = chain.model.precision().default_tolerance;
    let raised = (chain
        .model
        .closure(original)
        .map_err(|e| e.to_string())?
        .vertices)
        .iter()
        .any(|&v| chain.model.vertex(v).is_ok_and(|v| v.tolerance() > default));
    let mut held = fixture.clone();
    if raised {
        held.recipe.tolerances =
            corpus::within_own_tolerance(&fixture.recipe.tolerances, &chain.model, original)?;
    }
    corpus::measure_stage(&held, &read_chain, expected).map_err(|e| e.to_string())?;
    let rebuilt = degenerate_edges(&read_chain.model, body)?;
    if rebuilt != degenerate {
        return Err(format!(
            "{rebuilt} degenerate edges read back of {degenerate}"
        ));
    }
    Ok(Some(degenerate))
}

/// Every entity of the read body is generated from a file entity of the
/// solid's placement, and the body from the solid itself.
fn provenance_names_the_file(
    model: &Model,
    back: &ReadBody,
    solid: FileEntity,
) -> Result<(), String> {
    let closure = model.closure(back.body).map_err(|e| e.to_string())?;
    let shapes: Vec<Shape> = closure
        .vertices
        .iter()
        .map(|&v| Shape::from(Vertex::forward(v)))
        .chain(closure.edges.iter().map(|&e| Edge::forward(e).into()))
        .chain(closure.faces.iter().map(|&f| Face::forward(f).into()))
        .chain(closure.shells.iter().map(|&s| Shell::forward(s).into()))
        .collect();
    for s in shapes {
        match back.provenance.origins(s)[..] {
            [(Relation::Generated, Origin::Role(Role::File(FileEntity { instance: 0, .. })))] => {}
            ref other => return Err(format!("{s} comes from {other:?}")),
        }
    }
    let body_origin = back.provenance.origins(back.body);
    if body_origin != [(Relation::Generated, Origin::Role(Role::File(solid)))] {
        return Err(format!("the body comes from {body_origin:?}"));
    }

    Ok(())
}

/// Every variant of every fixture in `area` read back; failures listed
/// together.
fn area(area: &str) {
    let mut failures = Vec::new();
    let mut degenerate = 0;
    let mut read = 0;
    let root = fixtures::corpus_root().join(area);
    for dir in fixtures::corpus() {
        if !dir.starts_with(&root) {
            continue;
        }
        let fixture = fixtures::load(&dir).unwrap();
        for variant in fixture.recipe.variant_names() {
            match read_back(&fixture, &variant) {
                Ok(None) => {}
                Ok(Some(n)) => {
                    read += 1;
                    degenerate += usize::from(n > 0);
                }
                Err(e) => failures.push(format!("{} [{variant}]: {e}", fixture.name)),
            }
        }
    }
    assert!(read > 0, "nothing read in {area}");
    eprintln!("{area}: {read} read back, {degenerate} of them with a degenerate edge");
    assert!(
        failures.is_empty(),
        "{} of {} read back wrong:\n{}",
        failures.len(),
        failures.len() + read,
        failures.join("\n")
    );
}

#[test]
fn every_area_is_known() {
    for a in DUMPED_AREAS {
        assert!(
            [
                "primitive",
                "build",
                "transform",
                "boolean",
                "sweep",
                "provenance",
                "blend"
            ]
            .contains(&a),
            "{a} has no read-back test"
        );
    }
}

#[test]
fn primitive_fixtures_read_back() {
    area("primitive");
}

#[test]
fn build_fixtures_read_back() {
    area("build");
}

#[test]
fn transform_fixtures_read_back() {
    area("transform");
}

#[test]
fn boolean_fixtures_read_back() {
    area("boolean");
}

#[test]
fn sweep_fixtures_read_back() {
    area("sweep");
}

#[test]
fn provenance_fixtures_read_back() {
    area("provenance");
}

#[test]
fn blend_fixtures_read_back() {
    area("blend");
}

/// A cylinder's STEP, as Arris writes it.
fn cylinder_step() -> String {
    let mut m = Model::default();
    let body = arris_debug::sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    step::write(&m, &[body]).unwrap()
}

/// Two reads of one file are the same entities with the same ids.
#[test]
fn a_read_is_deterministic() {
    let text = cylinder_step();
    let dumps: Vec<String> = (0..2)
        .map(|_| {
            let mut m = Model::default();
            let read = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
            let body = read.solids[0].result.as_ref().unwrap().body;
            arris_debug::dump_text(&m, body).unwrap()
        })
        .collect();
    assert_eq!(dumps[0], dumps[1]);
}

/// A bound whose edges do not close is the solid's topology refused,
/// named by the solid, and leaves nothing in the model.
#[test]
fn a_loop_that_does_not_close_is_refused() {
    let text = cylinder_step();
    // Turn the first oriented edge of the file.
    let at = text.find("ORIENTED_EDGE('',*,*,").unwrap();
    let flag = at + text[at..].find(".T.").unwrap();
    let broken = format!("{}.F.{}", &text[..flag], &text[flag + 3..]);
    let mut m = Model::default();
    let read = step_read(&mut m, &broken, &ReadOptions::default()).unwrap();
    let refusal = read.solids[0].result.as_ref().unwrap_err();
    assert_eq!(refusal.kind(), step::RefusalKind::Topology, "{refusal}");
    assert_eq!(refusal.entity(), read.solids[0].entity.id);
    // Nothing was left behind: the next read gets the ids a fresh model
    // gives.
    let again = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
    let mut fresh = Model::default();
    let first = step_read(&mut fresh, &text, &ReadOptions::default()).unwrap();
    assert_eq!(
        again.solids[0].result.as_ref().unwrap().body,
        first.solids[0].result.as_ref().unwrap().body
    );
}

/// A faceted B-rep stands where a solid would: counted and refused by
/// name, beside the solid the file also holds.
#[test]
fn a_faceted_brep_is_refused_beside_the_solid() {
    let text = cylinder_step().replace(
        "ENDSEC;\nEND-ISO-10303-21;",
        "#90000 = FACETED_BREP('',#90001);\n#90001 = CLOSED_SHELL('',());\nENDSEC;\nEND-ISO-10303-21;",
    );
    let mut m = Model::default();
    let read = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), 2);
    assert!(
        read.solids[0].result.is_ok(),
        "{:?}",
        read.solids[0].result.as_ref().err()
    );
    let refusal = read.solids[1].result.as_ref().unwrap_err();
    assert_eq!(refusal.kind(), step::RefusalKind::Unsupported);
    assert_eq!(refusal.entity(), 90000);
    assert_eq!(
        read.solids[0].uncertainty,
        Some(m.precision().default_tolerance)
    );
}

/// An AP242 `TESSELLATED_SOLID`, as NIST's FTC-08 `-tg` edition writes
/// its part, is a faceted relative refused where it stands (ADR-0025 §2),
/// never a file of no solid.
#[test]
fn a_tessellated_solid_is_refused_beside_the_solid() {
    let text = cylinder_step().replace(
        "ENDSEC;\nEND-ISO-10303-21;",
        "#90000 = TESSELLATED_SOLID('',(#90001),$);\n#90001 = COMPLEX_TRIANGULATED_FACE('',#90002,3,(),$,(1,2,3),(),((1,2,3)));\n#90002 = COORDINATES_LIST('',3,((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)));\nENDSEC;\nEND-ISO-10303-21;",
    );
    let mut m = Model::default();
    let read = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), 2);
    let refusal = read.solids[1].result.as_ref().unwrap_err();
    assert_eq!(refusal.kind(), step::RefusalKind::Unsupported);
    assert_eq!(refusal.entity(), 90000);
    assert!(
        refusal.to_string().contains("TESSELLATED_SOLID"),
        "{refusal}"
    );
}

/// A file that does not parse fails whole.
#[test]
fn a_parse_error_fails_the_file() {
    let mut m = Model::default();
    let err = step_read(&mut m, "ISO-10303-21;\nHEADER;", &ReadOptions::default()).unwrap_err();
    assert!(matches!(err, step::ReadError::Parse(_)), "{err}");
}

/// A bound whose edges end apart in 3D — the cylinder's seam line
/// moved to the far side of the wall, so the circles reach their vertex
/// at `u = 0` and the seam, 8 away, leaves from `u = π` — is a gap past
/// the cap (ADR-0025 §4), named by the vertex where the walk finds it.
#[test]
fn a_seam_moved_off_its_vertices_is_a_gap() {
    let text = cylinder_step();
    let seam_point = "#62 = CARTESIAN_POINT('',(4.,0.,0.));";
    assert!(text.contains(seam_point));
    let broken = text.replace(seam_point, "#62 = CARTESIAN_POINT('',(-4.,0.,0.));");
    let mut m = Model::default();
    let read = step_read(&mut m, &broken, &ReadOptions::default()).unwrap();
    let refusal = read.solids[0].result.as_ref().unwrap_err();
    assert_eq!(
        refusal,
        &step::Refusal::Gap {
            entity: 32,
            gap: 8.0,
            cap: cylinder_cap()
        },
        "{refusal}"
    );
    assert_eq!(refusal.kind(), step::RefusalKind::Gap);
}

/// The header, the context and the circle of radius 4 about z at z = 0
/// (`#31`, on its vertex `#32`) of the cylinder's file, the data section
/// left open for the rest.
fn circle_and_context() -> String {
    let text = cylinder_step();
    let keep = |line: &str| {
        let id = line
            .strip_prefix('#')
            .and_then(|l| l.split(' ').next())
            .and_then(|n| n.parse::<u64>().ok());
        id.is_none_or(|id| id <= 21 || (26..=29).contains(&id) || (31..=56).contains(&id))
    };
    let data = text.strip_suffix("ENDSEC;\nEND-ISO-10303-21;\n").unwrap();
    let body: String = data
        .lines()
        .filter(|l| keep(l))
        .map(|l| format!("{l}\n"))
        .collect();
    body
}

/// A cone whose apex is a `VERTEX_LOOP`, as some writers bound it: the
/// cone face has the base circle for one bound and the apex for the
/// other, and no seam. The face is joined into one loop by the seam the
/// file left out, the ruling from the circle's vertex to the apex, and
/// the apex reads as its degenerate edge; the solid has the cone's
/// volume.
#[test]
fn a_vertex_loop_at_an_apex_is_a_degenerate_edge() {
    // A cone of radius 4 at z = 0 narrowing to its apex at z = −12, and
    // the disc on top.
    let body = circle_and_context();
    let cone = format!(
        "{body}#22 = MANIFOLD_SOLID_BREP('',#23);
#23 = CLOSED_SHELL('',(#24,#106));
#24 = ADVANCED_FACE('',(#30,#200),#25,.T.);
#25 = CONICAL_SURFACE('',#26,4.,{});
#30 = FACE_OUTER_BOUND('',#105,.T.);
#57 = ORIENTED_EDGE('',*,*,#31,.F.);
#105 = EDGE_LOOP('',(#57));
#200 = FACE_BOUND('',#201,.T.);
#201 = VERTEX_LOOP('',#202);
#202 = VERTEX_POINT('',#203);
#203 = CARTESIAN_POINT('',(0.,0.,-12.));
#106 = ADVANCED_FACE('',(#107),#45,.T.);
#107 = FACE_OUTER_BOUND('',#109,.T.);
#108 = ORIENTED_EDGE('',*,*,#31,.T.);
#109 = EDGE_LOOP('',(#108));
ENDSEC;
END-ISO-10303-21;
",
        (1.0f64 / 3.0).atan()
    );
    let mut m = Model::default();
    let read = step_read(&mut m, &cone, &ReadOptions::default()).unwrap();
    let back = read.solids[0].result.as_ref().unwrap();
    let report = check(&m, back.body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(degenerate_edges(&m, back.body), Ok(1));
    let closure = m.closure(back.body).unwrap();
    // The base circle, the seam the face needs from its vertex to the
    // apex, and the apex's degenerate edge.
    assert_eq!((closure.faces.len(), closure.edges.len()), (2, 3));
    let props = arris_debug::unmetered::mass_properties(&m, back.body).unwrap();
    let volume = std::f64::consts::PI * 16.0 * 12.0 / 3.0;
    assert!((props.volume - volume).abs() < 1e-9 * volume, "{props:?}");
    assert!((props.centroid.z + 3.0).abs() < 1e-9, "{props:?}");
    // The rebuilt edges are generated from the face they close.
    let circle = m.edge(closure.edges[0]).unwrap();
    assert!(circle.curve().is_some() && circle.is_closed());
    for &e in &closure.edges[1..] {
        assert_eq!(
            back.provenance.origins(Shape::from(Edge::forward(e))),
            [(
                Relation::Generated,
                Origin::Role(Role::File(FileEntity {
                    id: 24,
                    instance: 0
                }))
            )]
        );
    }
}

/// A hemisphere whose pole is a `VERTEX_LOOP`: the seam the face needs
/// is the meridian from the equator's vertex to the pole, a circle, and
/// the pole reads as the face's degenerate edge.
#[test]
fn a_vertex_loop_at_a_pole_is_a_degenerate_edge() {
    let body = circle_and_context();
    let dome = format!(
        "{body}#22 = MANIFOLD_SOLID_BREP('',#23);
#23 = CLOSED_SHELL('',(#24,#106));
#24 = ADVANCED_FACE('',(#30,#200),#25,.T.);
#25 = SPHERICAL_SURFACE('',#26,4.);
#30 = FACE_OUTER_BOUND('',#105,.T.);
#57 = ORIENTED_EDGE('',*,*,#31,.T.);
#105 = EDGE_LOOP('',(#57));
#200 = FACE_BOUND('',#201,.T.);
#201 = VERTEX_LOOP('',#202);
#202 = VERTEX_POINT('',#203);
#203 = CARTESIAN_POINT('',(0.,0.,4.));
#106 = ADVANCED_FACE('',(#107),#45,.F.);
#107 = FACE_OUTER_BOUND('',#109,.F.);
#108 = ORIENTED_EDGE('',*,*,#31,.T.);
#109 = EDGE_LOOP('',(#108));
ENDSEC;
END-ISO-10303-21;
"
    );
    let mut m = Model::default();
    let read = step_read(&mut m, &dome, &ReadOptions::default()).unwrap();
    let back = read.solids[0].result.as_ref().unwrap();
    let report = check(&m, back.body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    assert_eq!(degenerate_edges(&m, back.body), Ok(1));
    let props = arris_debug::unmetered::mass_properties(&m, back.body).unwrap();
    let volume = 2.0 / 3.0 * std::f64::consts::PI * 64.0;
    assert!((props.volume - volume).abs() < 1e-9 * volume, "{props:?}");
    assert!((props.centroid.z - 1.5).abs() < 1e-9, "{props:?}");
}

/// The cylinder's file with `from` replaced by `to`, read: the solid's
/// result and the model.
fn perturbed(from: &str, to: &str) -> (Model, Result<ReadBody, step::Refusal>) {
    let text = cylinder_step();
    assert!(text.contains(from), "{from}");
    let mut m = Model::default();
    let read = step_read(&mut m, &text.replace(from, to), &ReadOptions::default()).unwrap();
    let result = read.solids[0].result.clone();
    (m, result)
}

/// The largest vertex and edge tolerance of the body.
fn largest_tolerances(m: &Model, body: Body) -> (f64, f64) {
    let closure = m.closure(body).unwrap();
    let v = (closure.vertices.iter())
        .map(|&v| m.vertex(v).unwrap().tolerance())
        .fold(0.0, f64::max);
    let e = (closure.edges.iter())
        .map(|&e| m.edge(e).unwrap().tolerance())
        .fold(0.0, f64::max);
    (v, e)
}

/// The cylinder's gap cap: `READ_GAP_FRACTION` of the diameter of the
/// ball holding its box, `[−4, 4]² × [0, 12]`.
fn cylinder_cap() -> f64 {
    let diameter = (8.0f64 * 8.0 + 8.0 * 8.0 + 12.0 * 12.0).sqrt();
    arris_math::READ_GAP_FRACTION * diameter
}

/// A vertex moved by δ off the curves that meet it (ADR-0025 §4): below
/// the cap the solid reads to a body the checker passes at `Full`, the
/// vertex and the edges it bounds carrying at least δ; above it, the gap
/// is refused by name.
#[test]
fn a_vertex_moved_off_its_edges_carries_the_gap() {
    let from = "#60 = CARTESIAN_POINT('',(4.,0.,12.));";
    let delta = 0.1 * cylinder_cap();
    let (m, result) = perturbed(
        from,
        &format!("#60 = CARTESIAN_POINT('',(4.,0.,{}));", 12.0 + delta),
    );
    let back = result.unwrap_or_else(|r| panic!("{r}"));
    let report = check(&m, back.body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let (vertex, edge) = largest_tolerances(&m, back.body);
    assert!(vertex >= delta && edge <= vertex, "{vertex} {edge} {delta}");
    assert!(vertex <= 2.0 * delta, "{vertex} for {delta}");

    let delta = 10.0 * cylinder_cap();
    let (_, result) = perturbed(
        from,
        &format!("#60 = CARTESIAN_POINT('',(4.,0.,{}));", 12.0 + delta),
    );
    let refusal = result.unwrap_err();
    assert_eq!(refusal.kind(), step::RefusalKind::Gap, "{refusal}");
    let step::Refusal::Gap { gap, cap, .. } = refusal else {
        unreachable!()
    };
    assert!(
        gap > cap && gap >= delta * (1.0 - 1e-9),
        "{gap} {cap} {delta}"
    );
}

/// An edge curve lifted by δ off one of its faces — the top circle moved
/// up its axis, off the top plane and its vertex — is fitted on the face
/// at the gap: below the cap a checker-green body whose edge carries at
/// least δ, above it the edge refused as a gap.
#[test]
fn an_edge_lifted_off_its_face_carries_the_gap() {
    let from = "#82 = CARTESIAN_POINT('',(0.,0.,12.));";
    let delta = 0.1 * cylinder_cap();
    let (m, result) = perturbed(
        from,
        &format!("#82 = CARTESIAN_POINT('',(0.,0.,{}));", 12.0 + delta),
    );
    let back = result.unwrap_or_else(|r| panic!("{r}"));
    let report = check(&m, back.body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    let (vertex, edge) = largest_tolerances(&m, back.body);
    assert!(edge >= delta && vertex >= edge, "{vertex} {edge} {delta}");
    assert!(edge <= 2.0 * delta, "{edge} for {delta}");

    let delta = 10.0 * cylinder_cap();
    let (_, result) = perturbed(
        from,
        &format!("#82 = CARTESIAN_POINT('',(0.,0.,{}));", 12.0 + delta),
    );
    let refusal = result.unwrap_err();
    // The circle's vertex is where its uses meet the seam, and the walk
    // finds the gap there first.
    let step::Refusal::Gap { entity, gap, cap } = refusal else {
        panic!("{refusal}")
    };
    assert_eq!(entity, 59);
    assert!(
        gap > cap && gap >= delta * (1.0 - 1e-9),
        "{gap} {cap} {delta}"
    );
}

/// An Open CASCADE XCAF assembly of a box and a through-hole block, the
/// block placed twice (ADR-0025 §5): three bodies, each checker-clean at
/// `Full`, at the oracle's volume and centroid of its placed shape, and
/// named by its solid and which placement of it — the block's two in the
/// order of their paths.
#[test]
fn an_assembly_reads_to_a_body_per_placed_solid() {
    let root = fixtures::corpus_root();
    let (text, placed) = arris_debug::oracle::occt_assembly(
        &root.join("primitive/box"),
        &root.join("boolean/through-hole"),
        "occt-assembly-box-through-hole",
    )
    .unwrap();
    let mut m = Model::default();
    let read = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), 3);
    let instances: Vec<(u64, u32)> = read
        .solids
        .iter()
        .map(|s| (s.entity.id, s.entity.instance))
        .collect();
    assert_eq!(instances[0].1, 0);
    assert_eq!(instances[1], (instances[2].0, 0));
    assert_eq!(instances[2].1, 1);
    // Placement order is the box, then the block twice: the file's
    // solids ascend the same way, the block's paths in NAUO order.
    for (solid, oracle) in read.solids.iter().zip(&placed) {
        let back = solid.result.as_ref().unwrap_or_else(|r| panic!("{r}"));
        let report = check(&m, back.body, Level::Full);
        assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
        let mass = arris_debug::unmetered::mass_properties(&m, back.body).unwrap();
        assert!(
            (mass.volume - oracle.volume).abs() <= 1e-9 * oracle.volume,
            "{:?}: volume {} vs {}",
            solid.entity,
            mass.volume,
            oracle.volume
        );
        let c = oracle.centroid;
        let apart = (mass.centroid - arris_math::Point3::new(c[0], c[1], c[2])).norm();
        assert!(
            apart <= 1e-7,
            "{:?}: centroid {} vs {c:?}",
            solid.entity,
            mass.centroid
        );
        provenance_names_the_placement(&m, back, solid.entity).unwrap();
    }
    // Two reads of one file are the same entities with the same ids.
    let dumps: Vec<String> = (0..2)
        .map(|_| {
            let mut m = Model::default();
            let read = step_read(&mut m, &text, &ReadOptions::default()).unwrap();
            read.solids
                .iter()
                .map(|s| arris_debug::dump_text(&m, s.result.as_ref().unwrap().body).unwrap())
                .collect()
        })
        .collect();
    assert_eq!(dumps[0], dumps[1]);
}

/// Every entity of a body read at a placement is generated from a file
/// entity at that placement.
fn provenance_names_the_placement(
    model: &Model,
    back: &ReadBody,
    solid: FileEntity,
) -> Result<(), String> {
    let closure = model.closure(back.body).map_err(|e| e.to_string())?;
    let shapes: Vec<Shape> = (closure.vertices.iter())
        .map(|&v| Shape::from(Vertex::forward(v)))
        .chain(closure.faces.iter().map(|&f| Face::forward(f).into()))
        .collect();
    for s in shapes {
        match back.provenance.origins(s)[..] {
            [(Relation::Generated, Origin::Role(Role::File(FileEntity { instance, .. })))]
                if instance == solid.instance => {}
            ref other => return Err(format!("{s} comes from {other:?}")),
        }
    }
    Ok(())
}

/// A file of two solids, one spoiled by an `OFFSET_SURFACE` put under
/// one of its faces: the other reads, and the spoiled one is refused by
/// name — one refused solid never hides another (ADR-0025 §3).
#[test]
fn a_refused_solid_does_not_hide_the_others() {
    let mut m = Model::default();
    let block = arris_debug::sample::cuboid(
        &mut m,
        arris_math::Point3::new(10.0, 0.0, 0.0),
        arris_math::Point3::new(12.0, 3.0, 4.0),
    )
    .unwrap();
    let cylinder = arris_debug::sample::cylinder(&mut m, 4.0, 12.0).unwrap();
    let text = step::write(&m, &[block, cylinder]).unwrap();
    let wall = text
        .lines()
        .find(|l| l.contains("CYLINDRICAL_SURFACE("))
        .and_then(|l| l.split(" = ").next())
        .unwrap()
        .to_string();
    let face = text
        .lines()
        .find(|l| l.contains("ADVANCED_FACE(") && l.contains(&format!(",{wall},")))
        .unwrap();
    let spoiled = text
        .replace(face, &face.replace(&format!(",{wall},"), ",#999999,"))
        .replace(
            "ENDSEC;\nEND-ISO-10303-21;",
            &format!("#999999 = OFFSET_SURFACE('',{wall},0.5,.F.);\nENDSEC;\nEND-ISO-10303-21;"),
        );
    let mut back = Model::default();
    let read = step_read(&mut back, &spoiled, &ReadOptions::default()).unwrap();
    assert_eq!(read.solids.len(), 2);
    let results: Vec<_> = read.solids.iter().map(|s| s.result.as_ref()).collect();
    let refused: Vec<_> = results.iter().filter_map(|r| r.err()).collect();
    assert_eq!(refused.len(), 1, "{refused:?}");
    assert_eq!(
        refused[0].kind(),
        step::RefusalKind::Offset,
        "{}",
        refused[0]
    );
    assert_eq!(refused[0].entity(), 999999);
    let read_ok = results.iter().find_map(|r| r.ok()).unwrap();
    assert_eq!(back.faces(read_ok.body).unwrap().len(), 6);
}
