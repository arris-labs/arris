//! The recipe strategy (`prop::recipe`, ADR-0024 §2): every drawn recipe
//! is a fixture both kernels can read. It survives the serde round trip,
//! builds in Arris to a body or a typed refusal and never to a malformed
//! recipe's error, and its probes say what the generator meant — every
//! operand's centre inside it (a hollowed one's in its cavity, outside),
//! a box's or a cylinder's probes inside and outside it as labelled. One drawn recipe goes through the oracle and
//! the fixture loader, which hash it alike.
//!
//! A kernel panic is not the draw's fault, so it is let pass here: the
//! differential classes it and fails on it (`regression/tilted-cylinder-
//! slot-cut` and `pin-at-disc-rim-common-fuse` are the two this draw
//! found first).

use arris_check::classify::{Classification, classify_point};
use arris_debug::corpus::{self, CorpusError};
use arris_debug::fixtures::{self, Recipe};
use arris_debug::oracle;
use arris_debug::prop::recipe::recipe;
use arris_debug::prop::{DEFAULT_SEED, runner_with_seed};
use arris_debug::testing::fail;
use arris_math::Point3;
use proptest::prelude::*;
use proptest::strategy::ValueTree;

fn point(p: &[fixtures::Num; 3]) -> Point3 {
    let v = p.clone().map(|n| n.eval(&Default::default()).unwrap());
    Point3::new(v[0], v[1], v[2])
}

arris_debug::prop_shards! {
    /// The draw at the hook's count, split so its builds run at once.
    every_drawn_recipe_is_a_fixture_arris_builds_or_refuses
        [s0 s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 s14 s15] (r) = recipe() => {
        let text = serde_json::to_string_pretty(&r).map_err(fail)?;
        let back: Recipe = serde_json::from_str(&text).map_err(fail)?;
        prop_assert_eq!(&back, &r);
        // A kernel panic — the debug build's checker guard, say — is the
        // differential's `Panic` or `CheckerViolation`, held and shrunk
        // there (ADR-0024 §2), not a fault of the draw.
        let Ok(built) = std::panic::catch_unwind(|| corpus::build("generated/recipe", &r)) else {
            return Ok(());
        };
        let chain = match built {
            Ok(chain) => chain,
            // A typed refusal is the kernel's answer, not the recipe's.
            Err(CorpusError::Op { .. }) => return Ok(()),
            Err(e) => return Err(fail(format!("a malformed recipe: {e}\n{text}"))),
        };
        prop_assert!(chain.result().is_some());
        // Every operand's own probes against the operand alone.
        for made in chain.steps.keys().filter(|k| k.starts_with('o') || k.starts_with('m')) {
            let body = chain.steps[made].body;
            for p in r.probes.iter().filter(|p| p.label.starts_with(&format!("{made}-"))) {
                let want = if p.label.ends_with("-out") {
                    Classification::Outside
                } else {
                    Classification::Inside
                };
                let found = classify_point(&chain.model, body, point(&p.point)).map_err(fail)?;
                prop_assert_eq!(found, want, "probe {} of\n{}", p.label, text);
            }
        }
        Ok(())
    }
}

/// The first of the fixed seed's draws the oracle builds: an oracle
/// refusal (a curved trim on an elliptic cylinder's wall, say) is the
/// differential's `OracleRefuses`, not a question about the hash.
#[test]
fn a_drawn_recipe_hashes_alike_through_the_oracle_and_the_loader() {
    let mut runner = runner_with_seed(&DEFAULT_SEED);
    let mut refused = Vec::new();
    for _ in 0..8 {
        let r = recipe().new_tree(&mut runner).unwrap().current();
        let dir = match oracle::scratch_fixture("drawn-recipe", &r) {
            Ok(dir) => dir,
            Err(e) => {
                refused.push(e.to_string());
                continue;
            }
        };
        let loaded = fixtures::load(&dir).unwrap();
        assert_eq!(loaded.recipe, r);
        assert_eq!(loaded.recipe_sha256, loaded.expected.recipe_sha256);
        assert!(loaded.expected.results.contains_key("default"));
        return;
    }
    panic!(
        "the oracle refused eight draws in a row:\n{}",
        refused.join("\n")
    );
}
