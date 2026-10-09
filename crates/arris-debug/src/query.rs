//! The `query` fixture kind (ADR-0053 §8): a solid recipe and the
//! questions a consumer asks of its result. The format is
//! `tests/fixtures/README.md` §Query fixtures; this module is its Rust
//! reading, its runner ([`run`]) and its lint ([`lint`]).
//!
//! `fixture.json` is the solid kind's recipe (`params`, `variants`,
//! `steps`, `result`) with `"kind": "query"` and a `queries` list. A
//! query is one ray, `{"label", "ray": {origin, direction}, "hits":
//! "first" | "all", "expect_hits"}`; the distance, interference and
//! section plans add their own forms to the same list. `expected.json`
//! is Open CASCADE's answer, per variant and query: every hit of
//! `IntCurvesFace_ShapeIntersector` on the half-line, sorted by `t`.
//!
//! The runner builds the result under each variant and holds it to the
//! checker at `Full`; `expect_hits`, the closed-form count the author
//! states, is what the lint holds to the oracle's count.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::corpus::{self, CorpusError};
use crate::fixtures::{
    self, Expected, Fixture, FixtureError, Num, Recipe, expand_patterns, name_of, recipe_hash,
};

/// How many of a ray's hits the query asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum HitsWanted {
    /// The hit with the smallest `t`.
    First,
    /// Every hit, by ascending `t`.
    All,
}

/// A half-line: `origin` and the direction it runs in.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RaySpec {
    /// Where it starts.
    pub origin: [Num; 3],
    /// Which way it runs; not necessarily a unit vector.
    pub direction: [Num; 3],
}

/// One ray question of a fixture.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Query {
    /// A name for the report; unique within the fixture.
    pub label: String,
    /// The ray.
    pub ray: RaySpec,
    /// How many hits are asked for.
    pub hits: HitsWanted,
    /// The number of hits the author works out by hand, for the answer
    /// to `hits`: what the lint holds the oracle's count to. `None`
    /// leaves the count to the oracle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expect_hits: Option<usize>,
}

/// A hit as Open CASCADE reports it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OracleHit {
    /// The distance along the ray's unit direction.
    pub t: f64,
    /// Where it lands.
    pub point: [f64; 3],
    /// The face's parameters there.
    pub uv: [f64; 2],
    /// The face's outward normal there; zero at a pole, which is not
    /// compared.
    pub normal: [f64; 3],
    /// The centroid of the face it lands on: the key a hit is matched by
    /// across kernels, since ids differ.
    pub face_centroid: [f64; 3],
    /// `in`, `out` or `tangent`.
    pub transition: String,
    /// `in` the face, or `on` its boundary.
    pub state: String,
}

/// The oracle's answer to one query.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Answer {
    /// The query's label.
    pub label: String,
    /// Every hit on the half-line, by ascending `t`.
    pub hits: Vec<OracleHit>,
}

impl Answer {
    /// The hits the query `wanted` asks for: all of them, or the first.
    pub fn wanted(&self, wanted: HitsWanted) -> &[OracleHit] {
        match wanted {
            HitsWanted::All => &self.hits,
            HitsWanted::First => &self.hits[..self.hits.len().min(1)],
        }
    }
}

/// The oracle's answers under one variant, in the order of the queries.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct VariantAnswers {
    /// One per query.
    pub queries: Vec<Answer>,
}

/// An `expected.json` of the query kind.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct QueryExpected {
    /// The `cadquery-ocp` version that wrote it.
    pub occt: String,
    /// The hash of the recipe's [`QUERY_KEYS`].
    pub recipe_sha256: String,
    /// `"query"`.
    pub kind: String,
    /// Per variant.
    pub results: BTreeMap<String, VariantAnswers>,
}

/// A loaded query fixture.
#[derive(Debug, Clone, PartialEq)]
pub struct QueryFixture {
    /// The directory.
    pub dir: PathBuf,
    /// `<area>/<slug>`.
    pub name: String,
    /// The solid recipe the questions are asked of.
    pub recipe: Recipe,
    /// The questions.
    pub queries: Vec<Query>,
    /// The hash of the recipe as loaded.
    pub recipe_sha256: String,
    /// The oracle's answers.
    pub expected: QueryExpected,
}

impl QueryFixture {
    /// The recipe as a solid [`Fixture`] with no oracle measurements,
    /// which is what the corpus builder takes.
    fn as_solid(&self) -> Fixture {
        Fixture {
            dir: self.dir.clone(),
            name: self.name.clone(),
            recipe: self.recipe.clone(),
            recipe_sha256: self.recipe_sha256.clone(),
            expected: Expected {
                occt: self.expected.occt.clone(),
                recipe_sha256: self.expected.recipe_sha256.clone(),
                results: BTreeMap::new(),
            },
        }
    }
}

/// Why a query fixture failed.
#[derive(Debug, thiserror::Error)]
pub enum QueryError {
    /// A file is missing, malformed or of another kind.
    #[error(transparent)]
    Fixture(#[from] FixtureError),
    /// Building the recipe, or the checker on its result, failed.
    #[error(transparent)]
    Corpus(#[from] CorpusError),
    /// `expected.json` holds no answers for a variant or a query.
    #[error("{fixture}: [{variant}] the oracle has no answer for query {label:?}")]
    NoAnswer {
        /// The fixture.
        fixture: String,
        /// The variant.
        variant: String,
        /// The query's label.
        label: String,
    },
    /// A number of a query does not evaluate.
    #[error("{fixture}: query {label:?}: {source}")]
    Expression {
        /// The fixture.
        fixture: String,
        /// The query's label.
        label: String,
        /// The cause.
        source: fixtures::ExprError,
    },
    /// The hits do not match.
    #[error("{fixture}: [{variant}] query {label:?}: {what}")]
    Mismatch {
        /// The fixture.
        fixture: String,
        /// The variant.
        variant: String,
        /// The query's label.
        label: String,
        /// What differs.
        what: String,
    },
}

/// Loads a query fixture's directory: both files, and the recipe's hash.
/// Errors: [`FixtureError`] for a missing or malformed file.
///
/// ```no_run
/// use arris_debug::{fixtures, query};
///
/// let dir = fixtures::corpus_root().join("query/ray-box-through");
/// let fixture = query::load(&dir).unwrap();
/// assert_eq!(fixture.queries.len(), fixture.expected.results["default"].queries.len());
/// ```
pub fn load(dir: &Path) -> Result<QueryFixture, FixtureError> {
    let path = dir.join("fixture.json");
    let raw: serde_json::Value = read_json(&path)?;
    let mut expanded = raw.clone();
    expand_patterns(&mut expanded).map_err(|(step, why)| FixtureError::Pattern {
        path: path.clone(),
        step,
        why,
    })?;
    let json_err = |source| FixtureError::Json {
        path: path.clone(),
        source,
    };
    let queries: Vec<Query> =
        serde_json::from_value(expanded.get("queries").cloned().unwrap_or_default())
            .map_err(json_err)?;
    let recipe: Recipe = serde_json::from_value(expanded).map_err(json_err)?;
    let expected: QueryExpected = read_json(&dir.join("expected.json"))?;
    let recipe_sha256 = recipe_hash(&raw).map_err(|kind| FixtureError::UnknownKind {
        path: path.clone(),
        kind,
    })?;
    Ok(QueryFixture {
        dir: dir.to_path_buf(),
        name: name_of(dir),
        recipe,
        queries,
        recipe_sha256,
        expected,
    })
}

fn read_json<T: serde::de::DeserializeOwned>(path: &Path) -> Result<T, FixtureError> {
    let text = std::fs::read_to_string(path).map_err(|source| FixtureError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    serde_json::from_str(&text).map_err(|source| FixtureError::Json {
        path: path.to_path_buf(),
        source,
    })
}

/// Runs a query fixture: for every variant, builds the result, holds it
/// to the checker at `Full`, and holds each query's answer to the
/// closed-form `expect_hits`. Errors: [`QueryError`], naming the variant
/// and the query.
///
/// ```no_run
/// use arris_debug::{fixtures, query};
///
/// query::run(&fixtures::corpus_root().join("query/ray-box-through")).unwrap();
/// ```
pub fn run(dir: &Path) -> Result<(), QueryError> {
    let fixture = load(dir)?;
    let solid = fixture.as_solid();
    for variant in fixture.recipe.variant_names() {
        let Some(answers) = fixture.expected.results.get(&variant) else {
            return Err(QueryError::Corpus(CorpusError::Variant {
                fixture: fixture.name.clone(),
                variant,
            }));
        };
        let chain = corpus::chain_of(&solid, &variant)?;
        corpus::check_stage(&solid, &chain)?;
        for query in &fixture.queries {
            let Some(answer) = answers.queries.iter().find(|a| a.label == query.label) else {
                return Err(QueryError::NoAnswer {
                    fixture: fixture.name.clone(),
                    variant,
                    label: query.label.clone(),
                });
            };
            let count = answer.wanted(query.hits).len();
            if let Some(expect) = query.expect_hits.filter(|&n| n != count) {
                return Err(QueryError::Mismatch {
                    fixture: fixture.name.clone(),
                    variant,
                    label: query.label.clone(),
                    what: format!("{count} hits, the recipe expects {expect}"),
                });
            }
        }
    }
    Ok(())
}

/// The corpus lint of a query fixture: presence, the recipe's hash, the
/// kind recorded, the result being a step, every variant answering every
/// query by label, unique labels, the numbers of every ray evaluating
/// to a ray with a direction, and `expect_hits` matching the oracle's
/// count. Returns every problem found, empty when clean.
pub fn lint(dir: &Path) -> Vec<String> {
    let fixture = match load(dir) {
        Ok(f) => f,
        Err(e) => return vec![format!("{}: {e}", dir.display())],
    };
    let mut problems = Vec::new();
    let name = &fixture.name;
    let mut problem = |text: String| problems.push(format!("{name}: {text}"));
    let (r, x) = (&fixture.recipe, &fixture.expected);
    if x.recipe_sha256 != fixture.recipe_sha256 {
        problem(format!(
            "expected.json is stale: recipe hash {} but the recipe hashes to {} — rerun tools/oracle/expected.py",
            x.recipe_sha256, fixture.recipe_sha256
        ));
    }
    if x.kind != "query" {
        problem(format!("expected.json kind is {:?}, not \"query\"", x.kind));
    }
    if !r.steps.iter().any(|s| s.name() == r.result) {
        problem(format!("result {:?} is not a step", r.result));
    }
    if fixture.queries.is_empty() {
        problem("no queries".into());
    }
    let mut labels = std::collections::BTreeSet::new();
    for q in &fixture.queries {
        if !labels.insert(q.label.as_str()) {
            problem(format!("duplicate query label {:?}", q.label));
        }
    }
    for variant in r.variant_names() {
        let (Some(params), Some(answers)) = (r.params_of(&variant), x.results.get(&variant)) else {
            problem(format!(
                "variant {variant:?} has no answers in expected.json"
            ));
            continue;
        };
        for q in &fixture.queries {
            let ray = [&q.ray.origin, &q.ray.direction].map(|v| {
                let n = |i: usize| v[i].eval(&params);
                Ok::<_, fixtures::ExprError>([n(0)?, n(1)?, n(2)?])
            });
            match &ray {
                [Ok(_), Ok(d)] if d.iter().all(|c| *c == 0.0) => {
                    problem(format!(
                        "[{variant}] query {:?}: the direction is zero",
                        q.label
                    ));
                }
                [Ok(_), Ok(_)] => {}
                [Err(e), _] | [_, Err(e)] => {
                    problem(format!("[{variant}] query {:?}: {e}", q.label));
                }
            }
            let Some(answer) = answers.queries.iter().find(|a| a.label == q.label) else {
                problem(format!(
                    "[{variant}] query {:?} has no answer in expected.json",
                    q.label
                ));
                continue;
            };
            let count = answer.wanted(q.hits).len();
            if let Some(expect) = q.expect_hits.filter(|&n| n != count) {
                problem(format!(
                    "[{variant}] query {:?}: expect_hits {expect} but the oracle says {count}",
                    q.label
                ));
            }
        }
        if answers.queries.len() != fixture.queries.len() {
            problem(format!(
                "[{variant}] {} answers for {} queries",
                answers.queries.len(),
                fixture.queries.len()
            ));
        }
    }
    problems
}
