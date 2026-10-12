//! Gap healing in the STEP reader (ADR-0058): a vertex or an edge past
//! the read cap is grown to its gap up to the healing ceiling and named
//! in the body's record; past the ceiling, and with `Heal::Off` past the
//! read cap, it is refused as before; a file that needs no healing reads
//! alike either way.

use arris_check::{Level, check};
use arris_debug::unmetered::{mass_properties, step_read};
use arris_io::step::{self, Heal, ReadBody, ReadOptions, Refusal};
use arris_math::{HEAL_GAP_FRACTION, READ_GAP_FRACTION};
use arris_topo::{EntityId, Model};

/// The STEP of a cylinder of `radius` and `height`, as Arris writes it.
fn cylinder_step(radius: f64, height: f64) -> String {
    let mut m = Model::default();
    let body = arris_debug::sample::cylinder(&mut m, radius, height).unwrap();
    step::write(&m, &[body]).unwrap()
}

/// The diameter of the ball holding the cylinder's box: the size the
/// caps are fractions of.
fn size(radius: f64, height: f64) -> f64 {
    (8.0 * radius * radius + height * height).sqrt()
}

/// The cylinder's file with its top vertex (`#59`, at `#60`) moved `delta`
/// up its axis, off the circles and the seam that meet it.
fn moved(radius: f64, height: f64, delta: f64) -> String {
    let text = cylinder_step(radius, height);
    let from = (text.lines())
        .find(|l| l.starts_with("#60 = CARTESIAN_POINT("))
        .unwrap()
        .to_string();
    let to = format!(
        "#60 = CARTESIAN_POINT('',({radius},0.,{}));",
        height + delta
    );
    assert!(from.contains(&format!(",{height}.)")), "{from}");
    text.replace(&from, &to)
}

/// `text` read with `heal`: the model and its one solid's result.
fn read(text: &str, heal: Heal) -> (Model, Result<ReadBody, Refusal>) {
    let mut m = Model::default();
    let options = ReadOptions {
        heal,
        ..ReadOptions::default()
    };
    let read = step_read(&mut m, text, &options).unwrap();
    assert_eq!(read.solids.len(), 1);
    let result = read.solids[0].result.clone();
    (m, result)
}

/// Every vertex and edge of the body whose tolerance is past `cap`.
fn past(m: &Model, back: &ReadBody, cap: f64) -> Vec<(EntityId, f64)> {
    let closure = m.closure(back.body).unwrap();
    let mut out: Vec<(EntityId, f64)> = (closure.vertices.iter())
        .map(|&v| (v.into(), m.vertex(v).unwrap().tolerance()))
        .chain((closure.edges.iter()).map(|&e| (e.into(), m.edge(e).unwrap().tolerance())))
        .filter(|&(_, t)| t > cap)
        .collect();
    out.sort_by_key(|a| a.0);
    out
}

#[test]
fn a_gap_past_the_read_cap_is_grown_to_and_named() {
    let read_cap = READ_GAP_FRACTION * size(4.0, 12.0);
    let delta = 2.0 * read_cap;
    let text = moved(4.0, 12.0, delta);

    let (_, off) = read(&text, Heal::Off);
    // Refused at the first entity past the read cap: an edge at the
    // vertex. The part's box holds the moved vertex.
    let Err(Refusal::Gap { gap, cap, .. }) = off else {
        panic!("{off:?}")
    };
    assert_eq!(cap, READ_GAP_FRACTION * size(4.0, 12.0 + delta));
    assert!(gap >= delta * (1.0 - 1e-9), "{gap}");

    let (m, healed) = read(&text, Heal::GapsAndOrientation);
    let back = healed.unwrap_or_else(|r| panic!("{r}"));
    let report = check(&m, back.body, Level::Full);
    assert!(report.is_ok() && report.unchecked().is_empty(), "{report}");
    // Exactly the entities past the read cap are named, at the tolerance
    // each was grown to, largest first; the moved vertex among them.
    let grown = &back.healing.grown;
    assert!(grown.windows(2).all(|w| w[0].gap >= w[1].gap), "{grown:?}");
    let mut named: Vec<(EntityId, f64)> = grown.iter().map(|g| (g.entity, g.gap)).collect();
    named.sort_by_key(|a| a.0);
    assert_eq!(
        named,
        past(&m, &back, READ_GAP_FRACTION * size(4.0, 12.0 + delta))
    );
    let heal_cap = HEAL_GAP_FRACTION * size(4.0, 12.0 + delta);
    assert!(grown.iter().all(|g| g.gap <= heal_cap), "{grown:?}");
    assert!(
        (grown.iter()).any(|g| matches!(g.entity, EntityId::Vertex(_))
            && g.file == 59
            && g.gap >= delta * (1.0 - 1e-9)),
        "{grown:?}"
    );
    assert!(back.healing.turned.is_empty());
}

#[test]
fn a_gap_past_the_healing_ceiling_is_refused_naming_the_ceiling() {
    let heal_cap = HEAL_GAP_FRACTION * size(4.0, 12.0);
    let delta = 2.0 * heal_cap;
    let (_, result) = read(&moved(4.0, 12.0, delta), Heal::GapsAndOrientation);
    let Err(Refusal::Gap { gap, cap, .. }) = result else {
        panic!("{result:?}")
    };
    assert_eq!(cap, HEAL_GAP_FRACTION * size(4.0, 12.0 + delta));
    assert!(gap >= delta * (1.0 - 1e-9), "{gap}");
}

/// A file whose gaps are within the read cap reads to the same body
/// healing or not, with an empty record.
#[test]
fn a_file_that_needs_no_healing_reads_alike_either_way() {
    let read_cap = READ_GAP_FRACTION * size(4.0, 12.0);
    for text in [cylinder_step(4.0, 12.0), moved(4.0, 12.0, 0.1 * read_cap)] {
        let dumps: Vec<String> = [Heal::Off, Heal::GapsAndOrientation]
            .into_iter()
            .map(|heal| {
                let (m, result) = read(&text, heal);
                let back = result.unwrap_or_else(|r| panic!("{r}"));
                assert!(back.healing.is_empty(), "{:?}", back.healing);
                arris_debug::dump_text(&m, back.body).unwrap()
            })
            .collect();
        assert_eq!(dumps[0], dumps[1]);
    }
}

/// A part 1200 across whose vertex lies 0.03 off — the size of gap the
/// corpus's parts hold — heals past the model's `max_tolerance` of
/// 0.01, which bounds what an operation grows and not what the read
/// holds (ADR-0058 §2, ADR-0060), and the body passes the checker.
#[test]
fn healing_grows_past_the_models_max_tolerance() {
    let delta = 0.03;
    assert!(delta > Model::default().precision().max_tolerance);
    let text = moved(400.0, 1200.0, delta);
    let (_, off) = read(&text, Heal::Off);
    assert!(
        matches!(off, Err(Refusal::Gap { cap, .. }) if cap == Model::default().precision().max_tolerance),
        "{off:?}"
    );
    let (m, healed) = read(&text, Heal::GapsAndOrientation);
    let back = healed.unwrap_or_else(|r| panic!("{r}"));
    let report = check(&m, back.body, Level::Full);
    assert!(report.is_ok(), "{report}");
    let largest = back.healing.grown.first().map_or(0.0, |g| g.gap);
    assert!(largest >= delta * (1.0 - 1e-9), "{:?}", back.healing);
}

/// A healed body written and read back under the default options has
/// the same counts and volume, and a record no larger than the first.
#[test]
fn a_healed_body_reads_back_from_its_own_step() {
    let text = moved(400.0, 1200.0, 0.03);
    let (m, healed) = read(&text, Heal::GapsAndOrientation);
    let first = healed.unwrap_or_else(|r| panic!("{r}"));
    assert!(!first.healing.is_empty());
    let again = step::write(&m, &[first.body]).unwrap();
    let (back_model, back) = read(&again, Heal::default());
    let back = back.unwrap_or_else(|r| panic!("{r}"));
    let counts = |m: &Model, b| {
        let c = m.closure(b).unwrap();
        (c.vertices.len(), c.edges.len(), c.faces.len())
    };
    assert_eq!(counts(&m, first.body), counts(&back_model, back.body));
    let (a, b) = (
        mass_properties(&m, first.body).unwrap().volume,
        mass_properties(&back_model, back.body).unwrap().volume,
    );
    assert!((a - b).abs() <= 1e-9 * a, "{a} {b}");
    assert!(
        back.healing.grown.len() <= first.healing.grown.len(),
        "{:?} after {:?}",
        back.healing,
        first.healing
    );
}

/// Every committed part read with `Heal::Off` and healing: a solid whose
/// healing record is empty reads alike either way — the whole file's
/// dumps equal where no solid of it heals — a healed one is `Gap` (a
/// grown gap) or `Topology` (a turned face) with healing off, and a
/// refusal is the same refusal. `Heal::Off` is the
/// reader before healing, bit for bit.
#[test]
fn real_heal_off_reads_every_committed_part_as_before() {
    use arris_debug::fixtures::{Kind, corpus_root, kind_of};
    let root = corpus_root().join("real");
    let mut dirs: Vec<_> = (std::fs::read_dir(&root).unwrap())
        .map(|e| e.unwrap().path())
        .filter(|d| kind_of(d).is_ok_and(|k| k == Kind::Part))
        .collect();
    dirs.sort();
    assert!(dirs.len() > 20, "{}", root.display());
    for dir in dirs {
        let part = arris_debug::part::load(&dir).unwrap();
        let precision = part.part.precision.precision();
        let bytes = std::fs::read(dir.join(&part.part.file)).unwrap();
        let text = String::from_utf8_lossy(&bytes);
        let reads: Vec<(Model, step::Read)> = [Heal::Off, Heal::GapsAndOrientation]
            .into_iter()
            .map(|heal| {
                let mut m = Model::new(precision).unwrap();
                let options = ReadOptions {
                    heal,
                    ..ReadOptions::default()
                };
                let read = step_read(&mut m, &text, &options).unwrap();
                (m, read)
            })
            .collect();
        let [(off_model, off), (on_model, on)] = &reads[..] else {
            unreachable!()
        };
        let name = &part.name;
        assert_eq!(off.solids.len(), on.solids.len(), "{name}");
        let healed =
            (on.solids.iter()).any(|s| s.result.as_ref().is_ok_and(|b| !b.healing.is_empty()));
        for (a, b) in off.solids.iter().zip(&on.solids) {
            let at = format!("{name} #{}[{}]", a.entity.id, a.entity.instance);
            match (&a.result, &b.result) {
                (Ok(x), Ok(y)) => {
                    assert!(y.healing.is_empty(), "{at} heals and reads unhealed");
                    if !healed {
                        assert_eq!(
                            arris_debug::dump_text(off_model, x.body).unwrap(),
                            arris_debug::dump_text(on_model, y.body).unwrap(),
                            "{at}"
                        );
                    }
                }
                (Err(r), Ok(y)) => {
                    assert!(
                        !y.healing.is_empty(),
                        "{at}: refused unhealed ({r}) with nothing healed"
                    );
                    // A grown gap is `Gap` unhealed, a turned face
                    // `Topology`; with both, whichever the read meets first.
                    let kinds: &[step::RefusalKind] =
                        match (y.healing.grown.is_empty(), y.healing.turned.is_empty()) {
                            (false, true) => &[step::RefusalKind::Gap],
                            (true, false) => &[step::RefusalKind::Topology],
                            _ => &[step::RefusalKind::Gap, step::RefusalKind::Topology],
                        };
                    assert!(kinds.contains(&r.kind()), "{at}: {r}");
                }
                (Err(r), Err(s)) => assert_eq!(r.kind(), s.kind(), "{at}: {r} vs {s}"),
                (Ok(_), Err(s)) => panic!("{at} reads unhealed and is refused healing: {s}"),
            }
        }
    }
}
