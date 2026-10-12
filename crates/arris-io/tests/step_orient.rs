//! Orientation healing in the STEP reader (ADR-0058 §4): a face whose
//! bounds the file walks the same way as its neighbours on every shared
//! edge has its walk turned back and is named in the body's record; a
//! face that disagrees on some shared edges and not all, and two faces
//! each disagreeing only with the other, are still `Refusal::Topology`.

use arris_check::{Level, check};
use arris_debug::testing::fail;
use arris_debug::unmetered::{mass_properties, step_read, transform};
use arris_debug::{corpus, fixtures, prop, prop_shards};
use arris_io::step::{self, Heal, ReadBody, ReadOptions, Refusal, RefusalKind};
use arris_math::Isometry;
use arris_topo::provenance::{Origin, Relation, Role};
use arris_topo::{Face, FileEntity, Model, Shape};
use proptest::prelude::*;

/// The `#id` and the bound `#id`s of every `ADVANCED_FACE` of `text`, in
/// file order.
fn faces(text: &str) -> Vec<(u64, Vec<u64>)> {
    let ids = |s: &str| -> Vec<u64> {
        s.split('#')
            .skip(1)
            .filter_map(|t| {
                let digits: String = t.chars().take_while(char::is_ascii_digit).collect();
                digits.parse().ok()
            })
            .collect()
    };
    text.lines()
        .filter(|l| l.contains("= ADVANCED_FACE("))
        .map(|l| {
            let (head, rest) = l.split_once(" = ").unwrap();
            let bounds = rest.split_once("),").unwrap().0;
            (ids(head)[0], ids(bounds))
        })
        .collect()
}

/// `text` with each of `bounds`' sense flag turned: the face's walk
/// reversed, its surface and normal kept — what Open CASCADE's writer
/// does to a face reversed on a left-handed surface frame.
fn turned_bounds(text: &str, bounds: &[u64]) -> String {
    text.lines()
        .map(|l| {
            let turn = bounds.iter().any(|b| l.starts_with(&format!("#{b} = ")));
            if turn {
                let flipped = l.replacen(".T.);", ".F.);", 1);
                assert_ne!(flipped, l, "{l}");
                flipped
            } else {
                l.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n"
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

/// The corpus box in `pose`, its face `k` written with its bounds turned:
/// it reads back to the box with that face turned back and named, and is
/// `Topology` with healing off.
fn turned_face_reads_back(pose: Isometry, k: usize) -> Result<(), TestCaseError> {
    let chain =
        corpus::chain(&fixtures::corpus_root().join("primitive/box"), "default").map_err(fail)?;
    let body = chain.result().ok_or_else(|| fail("no box"))?;
    let mut m = chain.model;
    let body = transform(&mut m, body, &pose).map_err(fail)?.0;
    let volume = mass_properties(&m, body).map_err(fail)?.volume;
    let text = step::write(&m, &[body]).map_err(fail)?;
    let all = faces(&text);
    prop_assert_eq!(all.len(), 6);
    let (face, bounds) = &all[k % all.len()];
    let text = turned_bounds(&text, bounds);

    let (_, off) = read(&text, Heal::Off);
    prop_assert!(
        matches!(&off, Err(r) if r.kind() == RefusalKind::Topology),
        "{off:?}"
    );
    let (back, healed) = read(&text, Heal::GapsAndOrientation);
    let healed = healed.map_err(fail)?;
    let report = check(&back, healed.body, Level::Full);
    prop_assert!(report.is_ok(), "{report}");
    let read_volume = mass_properties(&back, healed.body).map_err(fail)?.volume;
    prop_assert!(
        (read_volume - volume).abs() <= 1e-9 * volume,
        "{read_volume} {volume}"
    );
    prop_assert_eq!(healed.healing.turned.len(), 1);
    prop_assert!(healed.healing.grown.is_empty());
    let origins = healed
        .provenance
        .origins(Shape::from(Face::forward(healed.healing.turned[0])));
    prop_assert_eq!(
        origins,
        vec![(
            Relation::Generated,
            Origin::Role(Role::File(FileEntity {
                id: *face,
                instance: 0
            }))
        )]
    );
    Ok(())
}

prop_shards! {
    /// A box face turned in the file, in any pose, is turned back and
    /// named.
    a_turned_face_reads_back_turned_back
        [shard_0 shard_1 shard_2 shard_3]
        ((pose, k)) = (prop::pose(), 0usize..6) => {
            turned_face_reads_back(pose, k)
        }
}

/// The through-hole block's top face has an outer and an inner bound:
/// turning only the inner one leaves the face agreeing with its
/// neighbours on its outer edges and disagreeing on its hole's, which is
/// an inconsistent file, not a reversed face.
#[test]
fn a_face_that_disagrees_on_some_edges_is_still_topology() {
    let chain = corpus::chain(
        &fixtures::corpus_root().join("boolean/through-hole"),
        "default",
    )
    .unwrap();
    let body = chain.result().unwrap();
    let text = step::write(&chain.model, &[body]).unwrap();
    let (_, bounds) = faces(&text)
        .into_iter()
        .find(|(_, b)| b.len() == 2)
        .expect("a face with a hole");
    let text = turned_bounds(&text, &bounds[1..]);
    let (_, result) = read(&text, Heal::GapsAndOrientation);
    let Err(refusal) = result else {
        panic!("reads")
    };
    assert_eq!(refusal.kind(), RefusalKind::Topology, "{refusal}");
}
