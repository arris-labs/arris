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
//! The runner builds the result under each variant, holds it to the
//! checker at `Full`, fires every ray with `arris_ops::query::ray_fire`
//! and compares the hits to the oracle's: the count, then for each hit `t`
//! and the point within the fixture's `probe` tolerance, the normal, and
//! the face a hit on a face lands on by the face's centroid within
//! `centroid_abs` (ids differ between kernels, ADR-0053 §8).
//! `expect_hits`, the closed-form count the author states, is what the
//! lint holds to the oracle's count and the runner to Arris's.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use arris_check::flux::face_flux;
use arris_math::{Point3, Vec3};
use arris_ops::query::{Ray, RayHit, RayHits, ray_fire};
use arris_topo::{EntityId, FaceId, Model};
use serde::{Deserialize, Serialize};

mod distance;

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

/// One question of a fixture: a ray or a distance.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Query {
    /// A name for the report; unique within the fixture.
    pub label: String,
    /// The ray, for a ray query.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ray: Option<RaySpec>,
    /// How many hits the ray query asks for.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub hits: Option<HitsWanted>,
    /// The number of hits the author works out by hand, for the answer
    /// to `hits`: what the lint holds the oracle's count to. `None`
    /// leaves the count to the oracle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expect_hits: Option<usize>,
    /// The two operands, for a distance query.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance: Option<DistanceSpec>,
    /// The distance the author works out by hand: what the lint holds the
    /// oracle's to and the runner Arris's. `None` leaves it to the oracle.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub expect_distance: Option<Num>,
}

/// The two operands of a distance query.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DistanceSpec {
    /// The first operand.
    pub a: OperandSpec,
    /// The second.
    pub b: OperandSpec,
}

/// A point of the pick: the entity nearest it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Near {
    /// The point.
    pub near: [Num; 3],
}

/// One operand of a distance query: a `point`, or a recipe `step`'s body
/// whole or narrowed to its `face`, `edge` or `vertex` nearest a point
/// (ids differ between kernels, so an entity is picked by place).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OperandSpec {
    /// A bare point.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<[Num; 3]>,
    /// The name of a recipe step: its body.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub step: Option<String>,
    /// The face of the step's body nearest the point.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face: Option<Near>,
    /// The edge of the step's body nearest the point.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edge: Option<Near>,
    /// The vertex of the step's body nearest the point.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub vertex: Option<Near>,
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
    /// Every hit on the half-line, by ascending `t`; none for a distance
    /// query.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub hits: Vec<OracleHit>,
    /// The oracle's distance, for a distance query.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance: Option<OracleDistance>,
}

/// One closest pair as `BRepExtrema_DistShapeShape` reports it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OracleSolution {
    /// The point on the first operand.
    pub a: [f64; 3],
    /// The point on the second.
    pub b: [f64; 3],
    /// What the first point lies on: `vertex`, `edge` or `face`.
    pub a_support: String,
    /// What the second point lies on.
    pub b_support: String,
    /// The centroid of the face the first point lies on, for a face.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub a_face_centroid: Option<[f64; 3]>,
    /// The centroid of the face the second point lies on, for a face.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub b_face_centroid: Option<[f64; 3]>,
}

/// The oracle's answer to a distance query.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OracleDistance {
    /// The minimum distance.
    pub distance: f64,
    /// Every closest pair it lists.
    pub solutions: Vec<OracleSolution>,
    /// `InnerSolution`: one operand lies in the other solid.
    pub contained: bool,
}

/// What the oracle reports for one place a ray meets the body: the first
/// of its hits there, and the normal of each face it reports there.
#[derive(Debug, Clone, PartialEq)]
pub struct OracleGroup {
    /// The first hit of the place.
    pub hit: OracleHit,
    /// The normals of all the hits at the place, the first's included.
    pub normals: Vec<[f64; 3]>,
}

impl Answer {
    /// The places the query `wanted` asks for, as groups: with `merge`,
    /// hits within it of one `t` are one place (a ray through an edge or a
    /// vertex, which Open CASCADE reports once per face and Arris once —
    /// `analytic.query_differs`); without, each hit is its own. `First`
    /// is the first place.
    pub fn groups(&self, wanted: HitsWanted, merge: Option<f64>) -> Vec<OracleGroup> {
        let mut groups: Vec<(f64, OracleGroup)> = Vec::new();
        for hit in &self.hits {
            match groups.last_mut() {
                Some((t, group)) if merge.is_some_and(|within| hit.t - *t <= within) => {
                    group.normals.push(hit.normal);
                }
                _ => groups.push((
                    hit.t,
                    OracleGroup {
                        hit: hit.clone(),
                        normals: vec![hit.normal],
                    },
                )),
            }
        }
        let groups = groups.into_iter().map(|(_, g)| g);
        match wanted {
            HitsWanted::All => groups.collect(),
            HitsWanted::First => groups.take(1).collect(),
        }
    }

    /// Whether two hits share a `t` within `within`.
    pub fn has_shared_hits(&self, within: f64) -> bool {
        self.hits.windows(2).any(|w| w[1].t - w[0].t <= within)
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
    /// The hash of the recipe's [`crate::fixtures::QUERY_KEYS`].
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
    /// A ray's fire was refused.
    #[error("{fixture}: [{variant}] query {label:?}: {source}")]
    Refused {
        /// The fixture.
        fixture: String,
        /// The variant.
        variant: String,
        /// The query's label.
        label: String,
        /// The refusal.
        source: Box<arris_ops::OpError>,
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
/// to the checker at `Full`, fires each query's ray at it and holds the
/// hits to the oracle's, and to the closed-form `expect_hits`. Errors:
/// [`QueryError`], naming the variant and the query.
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
        let params = fixture.recipe.params_of(&variant).unwrap_or_default();
        let chain = corpus::chain_of(&solid, &variant)?;
        let (body, _) = corpus::check_stage(&solid, &chain)?;
        for query in &fixture.queries {
            let mismatch = |what: String| QueryError::Mismatch {
                fixture: fixture.name.clone(),
                variant: variant.clone(),
                label: query.label.clone(),
                what,
            };
            let Some(answer) = answers.queries.iter().find(|a| a.label == query.label) else {
                return Err(QueryError::NoAnswer {
                    fixture: fixture.name.clone(),
                    variant,
                    label: query.label.clone(),
                });
            };
            if let Some(spec) = &query.distance {
                if query.ray.is_some() {
                    return Err(mismatch("both a ray and a distance".into()));
                }
                distance::run(&fixture, &variant, &chain, query, spec, answer)?;
                continue;
            }
            let (Some(_), Some(wanted)) = (&query.ray, query.hits) else {
                return Err(mismatch("neither a distance nor a ray with `hits`".into()));
            };
            let merge = fixture
                .recipe
                .analytic
                .query_differs
                .is_some()
                .then_some(fixture.recipe.tolerances.probe);
            let groups = answer.groups(wanted, merge);
            let ray = ray_of(&fixture.name, query, &params)?;
            let which = match wanted {
                HitsWanted::First => RayHits::First,
                HitsWanted::All => RayHits::All,
            };
            let hits = ray_fire(&chain.model, body, &ray, which).map_err(|source| {
                QueryError::Refused {
                    fixture: fixture.name.clone(),
                    variant: variant.clone(),
                    label: query.label.clone(),
                    source: Box::new(source),
                }
            })?;
            if let Some(expect) = query.expect_hits.filter(|&n| n != hits.len()) {
                return Err(mismatch(format!(
                    "{} hits, the recipe expects {expect}",
                    hits.len()
                )));
            }
            compare(&chain.model, &hits, &groups, &fixture.recipe.tolerances).map_err(mismatch)?;
        }
    }
    Ok(())
}

/// The query's ray under `params`.
fn ray_of(fixture: &str, query: &Query, params: &BTreeMap<String, f64>) -> Result<Ray, QueryError> {
    let ray = query.ray.as_ref().expect("a ray query");
    let vector = |v: &[Num; 3]| -> Result<Vec3, QueryError> {
        let n = |i: usize| {
            v[i].eval(params).map_err(|source| QueryError::Expression {
                fixture: fixture.to_string(),
                label: query.label.clone(),
                source,
            })
        };
        Ok(Vec3::new(n(0)?, n(1)?, n(2)?))
    };
    Ok(Ray {
        origin: Point3::from(vector(&ray.origin)?),
        direction: vector(&ray.direction)?,
    })
}

/// Arris's `hits` against the oracle's, hit for hit: `t`, the point and
/// the normal within `tolerances.probe`, and a hit on a face by that
/// face's centroid within `tolerances.centroid_abs`. A hit that names an
/// edge or a vertex has no one face to match: the oracle reports it once
/// per face. `Err` is what differs, first difference only.
fn compare(
    model: &Model,
    hits: &[RayHit],
    oracle: &[OracleGroup],
    tolerances: &fixtures::Tolerances,
) -> Result<(), String> {
    if hits.len() != oracle.len() {
        return Err(format!(
            "{} hits, the oracle has {}",
            hits.len(),
            oracle.len()
        ));
    }
    let probe = tolerances.probe;
    for (i, (hit, group)) in hits.iter().zip(oracle).enumerate() {
        let o = &group.hit;
        if (hit.t - o.t).abs() > probe {
            return Err(format!("hit {i}: t {} but the oracle's is {}", hit.t, o.t));
        }
        let at = Vec3::from(o.point);
        if (hit.point.coords - at).norm() > probe {
            return Err(format!(
                "hit {i}: point {:?} but the oracle's is {:?}",
                hit.point.coords.as_slice(),
                o.point
            ));
        }
        // A normal the oracle leaves zero (a pole) is not compared; one
        // of a place on an edge or a vertex is any of its faces'.
        let agrees = |n: &[f64; 3]| {
            let n = Vec3::from(*n);
            n.norm() < 0.5 || (hit.normal.into_inner() - n).norm() <= probe
        };
        if !group.normals.iter().any(agrees) {
            return Err(format!(
                "hit {i}: normal {:?} but the oracle's {} {:?}",
                hit.normal.as_slice(),
                if group.normals.len() == 1 {
                    "is"
                } else {
                    "are"
                },
                group.normals
            ));
        }
        if let EntityId::Face(face) = hit.shape.id {
            let centroid = face_centroid(model, face)
                .map_err(|e| format!("hit {i}: the centroid of {face}: {e}"))?;
            if (centroid - Vec3::from(o.face_centroid)).norm() > tolerances.centroid_abs {
                return Err(format!(
                    "hit {i}: lands on the face centred {:?} but the oracle's is centred {:?}",
                    centroid.as_slice(),
                    o.face_centroid
                ));
            }
        }
    }
    Ok(())
}

/// The area centroid of `face`: `∬ P dA / ∬ dA` over its region.
fn face_centroid(model: &Model, face: FaceId) -> Result<Vec3, arris_check::flux::FluxError> {
    let area = face_flux(model, face, |_, n| n.norm())?;
    let mut moment = [0.0; 3];
    for (k, m) in moment.iter_mut().enumerate() {
        *m = face_flux(model, face, |p, n| p[k] * n.norm())?;
    }
    Ok(Vec3::new(moment[0], moment[1], moment[2]) / area)
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
            match (&q.ray, &q.distance) {
                (Some(ray), None) => {
                    let ray = [&ray.origin, &ray.direction].map(|v| {
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
                    if q.hits.is_none() {
                        problem(format!(
                            "[{variant}] query {:?}: a ray needs `hits`",
                            q.label
                        ));
                    }
                }
                (None, Some(spec)) => {
                    for text in distance::lint_spec(r, &params, spec) {
                        problem(format!("[{variant}] query {:?}: {text}", q.label));
                    }
                }
                _ => problem(format!(
                    "[{variant}] query {:?}: needs exactly one of `ray` and `distance`",
                    q.label
                )),
            }
            let Some(answer) = answers.queries.iter().find(|a| a.label == q.label) else {
                problem(format!(
                    "[{variant}] query {:?} has no answer in expected.json",
                    q.label
                ));
                continue;
            };
            let merge = r
                .analytic
                .query_differs
                .is_some()
                .then_some(r.tolerances.probe);
            if let Some(wanted) = q.hits {
                let count = answer.groups(wanted, merge).len();
                if let Some(expect) = q.expect_hits.filter(|&n| n != count) {
                    problem(format!(
                        "[{variant}] query {:?}: expect_hits {expect} but the oracle says {count}",
                        q.label
                    ));
                }
            }
            if q.distance.is_some() {
                match &answer.distance {
                    None => problem(format!(
                        "[{variant}] query {:?}: the oracle has no distance",
                        q.label
                    )),
                    Some(d) => {
                        if let Some(expect) = &q.expect_distance
                            && let Ok(expect) = expect.eval(&params)
                            && (expect - d.distance).abs() > r.tolerances.probe
                        {
                            problem(format!(
                                "[{variant}] query {:?}: expect_distance {expect} but the oracle says {}",
                                q.label, d.distance
                            ));
                        }
                    }
                }
            }
        }
        if let Some(why) = &r.analytic.query_differs {
            if why.trim().is_empty() {
                problem("analytic.query_differs needs its evidence".into());
            }
            if !answers.queries.iter().any(|a| {
                a.has_shared_hits(r.tolerances.probe)
                    || a.distance.as_ref().is_some_and(|d| d.solutions.len() > 1)
            }) {
                problem(format!(
                    "[{variant}] analytic.query_differs is set but no answer of the oracle reports a hit twice or lists several solutions"
                ));
            }
        } else if let Some(a) = answers
            .queries
            .iter()
            .find(|a| a.has_shared_hits(r.tolerances.probe))
        {
            problem(format!(
                "[{variant}] query {:?}: the oracle reports a hit twice, which needs analytic.query_differs",
                a.label
            ));
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
