//! Body bytes (ADR-0029): one body and its [`Provenance`] as bytes that
//! leave one model and enter another, in another process or under a later
//! release.
//!
//! [`write()`] imports the body into a fresh model under the writer's
//! [`Precision`](arris_check::arris_topo::arris_math::Precision), so its
//! geometry and topology are dense from zero whatever holes the writer's
//! model had, and encodes that model with the record as the writer holds
//! it and the map from the writer's ids to the dense ones. The record
//! stays in the writer's ids because it may name entities outside the
//! body (a boolean's inputs), and an id there could equal a dense id of
//! the body's own; the map says exactly which are the body's. The bytes
//! are [`BODY_MAGIC`], then `postcard` of the version and the body;
//! [`to_json`] is the same as one line of JSON, for diffs. Both are
//! deterministic byte for byte.
//!
//! [`read`] imports the body into the caller's model inside a
//! transaction and returns it with its record and the map from the
//! writer's ids to the caller's, or a typed [`BodyError`] with the
//! caller's model left as it was. [`Imported::translated`] puts the record
//! in the caller's ids, the foreign entities through a map the caller
//! gives.
//!
//! The wire is the model's own types: [`BODY_VERSION`] is bumped by the
//! commit that changes one of them, which also freezes the old shape and
//! writes the migration from it.

use std::collections::{BTreeMap, BTreeSet};

use arris_check::arris_topo::{
    Body, Closure, EntityId, IdMap, Model, Origin, Provenance, Shape, TopoError,
};
use arris_check::{Level, Report, check};
use serde::{Deserialize, Serialize};

/// The version this crate writes: the newest it reads.
pub const BODY_VERSION: u32 = 1;

/// The first eight bytes of every body's bytes, so they are never
/// mistaken for the native format's model bytes, which begin with the
/// same version varint.
pub const BODY_MAGIC: [u8; 8] = *b"ARRISBDY";

/// The magic as the JSON form spells it.
const JSON_MAGIC: &str = "ARRISBDY";

/// Why a body could not be written or read.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum BodyError {
    /// The data does not begin with [`BODY_MAGIC`]: it is not body bytes
    /// (a native model, another format, or nothing).
    #[error("not body bytes: the magic is missing")]
    Magic,
    /// The data carries a version newer than this build reads.
    #[error("body version {found} is newer than {newest}, the newest this build reads")]
    Version {
        /// The version in the data.
        found: u32,
        /// [`BODY_VERSION`].
        newest: u32,
    },
    /// The body could not be encoded.
    #[error("could not encode the body: {0}")]
    Encode(String),
    /// The data is not a body of its version: truncated, malformed, or
    /// holding a value that fails its type's validation.
    #[error("could not decode the body: {0}")]
    Decode(String),
    /// The body could not be copied: a handle did not resolve, in the
    /// writer's model on [`write()`] or in the decoded one on [`read`]
    /// (a reference the bytes hold to nothing).
    #[error(transparent)]
    Topo(#[from] TopoError),
    /// An entity's tolerance lies outside the reading model's
    /// `[min_tolerance, max_tolerance]`: the body was written under a
    /// precision this model says cannot exist. Never rescaled (ADR-0029).
    #[error("{entity} has tolerance {tolerance:e}, outside this model's [{min:e}, {max:e}]")]
    Precision {
        /// The entity, in the writer's ids.
        entity: EntityId,
        /// Its tolerance.
        tolerance: f64,
        /// The reading model's `min_tolerance`.
        min: f64,
        /// The reading model's `max_tolerance`.
        max: f64,
    },
    /// The body decoded and copied, but the checker at `Level::Full`
    /// rejects it in the reading model: faces that cross, a body inside
    /// out, a vertex off its edges. The report names every violation.
    #[error("the body read is not valid:\n{0}")]
    Rejected(Box<Report>),
}

/// What [`read`] returns: the body in the caller's model, the record
/// written with it, and the way from the writer's ids to the caller's.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Imported {
    /// The body, now in the caller's model.
    pub body: Body,
    /// The record as the writer held it, in the writer's ids;
    /// [`Imported::translated`] is it in the caller's.
    pub provenance: Provenance,
    /// The writer's id of every entity and geometry value of the body →
    /// its id in the caller's model. An entity of the record this map
    /// does not hold is [foreign](Imported::foreign).
    pub map: IdMap,
    /// The version the data carried, before any migration.
    pub version: u32,
}

impl Imported {
    /// The entities the record names that are not the body's, in the
    /// writer's ids, ascending: a boolean's inputs, the faces it deleted.
    /// A consumer that sent those to the writer (by `Model::import` into
    /// the writer's model) holds their way back: the inverse of the map
    /// that import returned.
    pub fn foreign(&self) -> Vec<Shape> {
        let p = &self.provenance;
        let mut all: BTreeSet<Shape> = p.outputs();
        all.extend(p.deleted());
        all.extend(p.origins_recorded().filter_map(|o| match o {
            Origin::Entity(s) => Some(s),
            Origin::Role(_) => None,
        }));
        all.into_iter()
            .filter(|&s| self.map.map(s).is_none())
            .collect()
    }

    /// The record in the caller's ids: the body's entities through
    /// [`Imported::map`], every other entity through `foreign` (a foreign
    /// entity `foreign` does not hold stays in the writer's id). One pass
    /// over one map, so an id the writer's model and the caller's both
    /// use is never translated twice.
    ///
    /// ```
    /// use arris_io::arris_check::arris_topo::{IdMap, Model};
    /// use arris_io::body;
    /// use arris_ops::primitive_box;
    ///
    /// let mut a = Model::default();
    /// arris_debug::sample::unit_box(&mut a).unwrap(); // so the next ids are not dense
    /// let (cube, record) = primitive_box(&mut a, [0.0; 3], [1.0; 3]).unwrap();
    /// let bytes = body::write(&a, cube, &record).unwrap();
    ///
    /// let mut b = Model::default();
    /// let read = body::read(&mut b, &bytes).unwrap();
    /// assert!(read.foreign().is_empty(), "a primitive names only its own");
    /// let mine = read.translated(&IdMap::default());
    /// assert_eq!(mine, record.mapped(&read.map));
    /// ```
    pub fn translated(&self, foreign: &IdMap) -> Provenance {
        let mut both = foreign.clone();
        both.vertices.extend(&self.map.vertices);
        both.edges.extend(&self.map.edges);
        both.faces.extend(&self.map.faces);
        both.shells.extend(&self.map.shells);
        both.bodies.extend(&self.map.bodies);
        self.provenance.mapped(&both)
    }
}

/// What is written after the magic and the version: the body alone in a
/// dense model, the record in the writer's ids, and the writer's ids →
/// the dense ones.
#[derive(Serialize)]
struct BodyOut<'m> {
    model: &'m Model,
    body: Body,
    provenance: &'m Provenance,
    map: &'m IdMap,
}

/// What is read at [`BODY_VERSION`], once the header has passed.
#[derive(Deserialize)]
struct BodyIn {
    model: Model,
    body: Body,
    provenance: Provenance,
    map: IdMap,
}

/// The JSON form: the magic and the version as fields ahead of the body.
#[derive(Serialize)]
struct JsonOut<'m> {
    magic: &'static str,
    version: u32,
    #[serde(flatten)]
    body: BodyOut<'m>,
}

/// The body alone in a fresh model under `model`'s precision, and the
/// map from `model`'s ids to the copy's.
fn dense(model: &Model, body: Body) -> Result<(Model, Body, IdMap), BodyError> {
    let mut fresh = Model::new(model.precision())?;
    let (copy, map) = fresh.import(model, body)?;
    Ok((fresh, copy, map))
}

/// `body` and its `record` as body bytes: [`BODY_MAGIC`], then `postcard`
/// of the version, the body with its ids made dense, `record` as it is
/// and the map from `model`'s ids to the dense ones. For storage.
/// `record` may name entities outside the body (a boolean's inputs).
/// Errors: [`BodyError::Topo`] when `body` does not resolve in `model`.
///
/// ```
/// use arris_debug::sample;
/// use arris_io::arris_check::arris_topo::{Model, Provenance};
/// use arris_io::body;
///
/// let mut a = Model::default();
/// let cube = sample::unit_box(&mut a).unwrap();
/// let bytes = body::write(&a, cube, &Provenance::default()).unwrap();
/// assert_eq!(bytes[..8], body::BODY_MAGIC);
///
/// let mut b = Model::default();
/// let read = body::read(&mut b, &bytes).unwrap();
/// assert_eq!(b.closure(read.body).unwrap().faces.len(), 6);
/// ```
pub fn write(model: &Model, body: Body, record: &Provenance) -> Result<Vec<u8>, BodyError> {
    let (fresh, copy, map) = dense(model, body)?;
    let out = BodyOut {
        model: &fresh,
        body: copy,
        provenance: record,
        map: &map,
    };
    let encoded = postcard::to_allocvec(&(BODY_VERSION, out))
        .map_err(|e| BodyError::Encode(e.to_string()))?;
    let mut bytes = BODY_MAGIC.to_vec();
    bytes.extend(encoded);
    Ok(bytes)
}

/// The same body as [`write()`]'s, as one line of JSON text: keys in
/// declaration order, every real in the shortest form that round-trips.
/// For diffs and tests.
///
/// ```
/// use arris_debug::sample;
/// use arris_io::arris_check::arris_topo::{Model, Provenance};
/// use arris_io::body;
///
/// let mut a = Model::default();
/// let cube = sample::unit_box(&mut a).unwrap();
/// let text = body::to_json(&a, cube, &Provenance::default()).unwrap();
/// assert!(text.starts_with(r#"{"magic":"ARRISBDY","version":1,"#));
/// ```
pub fn to_json(model: &Model, body: Body, record: &Provenance) -> Result<String, BodyError> {
    let (fresh, copy, map) = dense(model, body)?;
    serde_json::to_string(&JsonOut {
        magic: JSON_MAGIC,
        version: BODY_VERSION,
        body: BodyOut {
            model: &fresh,
            body: copy,
            provenance: record,
            map: &map,
        },
    })
    .map_err(|e| BodyError::Encode(e.to_string()))
}

fn check_version(found: u32) -> Result<(), BodyError> {
    if found == BODY_VERSION {
        Ok(())
    } else {
        Err(BodyError::Version {
            found,
            newest: BODY_VERSION,
        })
    }
}

/// The body in `bytes` ([`write()`]'s) imported into `model`, with its
/// record and the map from the writer's ids to `model`'s, checked at
/// `Level::Full` in every build profile. On any error `model` is left as it was. Errors:
/// [`BodyError::Magic`] for data that is not body bytes;
/// [`BodyError::Version`] for a newer version; [`BodyError::Decode`] for
/// a truncated or malformed stream, or a map that is not a one-to-one map
/// onto the body; [`BodyError::Topo`] for a body whose
/// references do not resolve; [`BodyError::Precision`] for a tolerance
/// `model` cannot hold; [`BodyError::Rejected`] for a body the checker
/// rejects.
///
/// ```
/// use arris_debug::sample;
/// use arris_io::arris_check::arris_topo::{Model, Provenance};
/// use arris_io::{body, native};
///
/// let mut a = Model::default();
/// sample::unit_box(&mut a).unwrap();
/// let model_bytes = native::to_bytes(&a).unwrap();
/// let mut b = Model::default();
/// assert_eq!(body::read(&mut b, &model_bytes), Err(body::BodyError::Magic));
/// ```
pub fn read(model: &mut Model, bytes: &[u8]) -> Result<Imported, BodyError> {
    let rest = bytes.strip_prefix(&BODY_MAGIC).ok_or(BodyError::Magic)?;
    let (version, _): (u32, &[u8]) =
        postcard::take_from_bytes(rest).map_err(|e| BodyError::Decode(e.to_string()))?;
    check_version(version)?;
    let (_, decoded): (u32, BodyIn) =
        postcard::from_bytes(rest).map_err(|e| BodyError::Decode(e.to_string()))?;
    import(model, decoded, version)
}

/// The body in `text` ([`to_json()`]'s) imported into `model`, as [`read`]
/// does, with the same errors; text that is not JSON at all is
/// [`BodyError::Decode`], JSON without the magic [`BodyError::Magic`].
///
/// ```
/// use arris_debug::sample;
/// use arris_io::arris_check::arris_topo::{Model, Provenance};
/// use arris_io::body;
///
/// let mut a = Model::default();
/// let cube = sample::unit_box(&mut a).unwrap();
/// let text = body::to_json(&a, cube, &Provenance::default()).unwrap();
/// let mut b = Model::default();
/// assert_eq!(body::from_json(&mut b, &text).unwrap().version, body::BODY_VERSION);
/// ```
pub fn from_json(model: &mut Model, text: &str) -> Result<Imported, BodyError> {
    // A tree first, so the magic and the version are read before the
    // body and a mismatch never fails as a decode error deep inside the
    // arenas; every real was parsed to its exact `f64` already.
    let tree: serde_json::Value =
        serde_json::from_str(text).map_err(|e| BodyError::Decode(e.to_string()))?;
    if tree.get("magic").and_then(serde_json::Value::as_str) != Some(JSON_MAGIC) {
        return Err(BodyError::Magic);
    }
    let version = tree
        .get("version")
        .and_then(serde_json::Value::as_u64)
        .and_then(|v| u32::try_from(v).ok())
        .ok_or_else(|| BodyError::Decode("no version a u32 holds".into()))?;
    check_version(version)?;
    let decoded = BodyIn::deserialize(tree).map_err(|e| BodyError::Decode(e.to_string()))?;
    import(model, decoded, version)
}

/// `true` when `written` sends distinct writer ids onto exactly `closure`
/// (ascending, as a [`Closure`] lists it).
fn onto<K: Ord + Copy>(written: &BTreeMap<K, K>, closure: &[K]) -> bool {
    let mut values: Vec<K> = written.values().copied().collect();
    values.sort();
    values == closure
}

/// Refuses a written map that is not one to one onto the body's closure:
/// then an entity of the record could not be told to be the body's.
fn check_map(map: &IdMap, closure: &Closure, body: Body) -> Result<(), BodyError> {
    let covered = onto(&map.vertices, &closure.vertices)
        && onto(&map.edges, &closure.edges)
        && onto(&map.faces, &closure.faces)
        && onto(&map.shells, &closure.shells)
        && onto(&map.bodies, &[body.id])
        && onto(&map.curves, &closure.curves)
        && onto(&map.surfaces, &closure.surfaces)
        && onto(&map.curve2s, &closure.curve2s);
    if covered {
        Ok(())
    } else {
        Err(BodyError::Decode(
            "the id map is not one to one onto the body".into(),
        ))
    }
}

/// Refuses the first vertex, edge or face of the body (in that order,
/// each kind ascending) whose tolerance lies outside `model`'s range,
/// naming it in the writer's ids through `back`.
fn check_tolerances(
    model: &Model,
    scratch: &Model,
    closure: &Closure,
    back: &IdMap,
) -> Result<(), BodyError> {
    let p = model.precision();
    let within = |entity: EntityId, tolerance: f64| {
        if p.min_tolerance <= tolerance && tolerance <= p.max_tolerance {
            Ok(())
        } else {
            Err(BodyError::Precision {
                entity: back.map_entity(entity).unwrap_or(entity),
                tolerance,
                min: p.min_tolerance,
                max: p.max_tolerance,
            })
        }
    };
    for &v in &closure.vertices {
        within(
            v.into(),
            scratch.vertex(v).map_err(TopoError::from)?.tolerance(),
        )?;
    }
    for &e in &closure.edges {
        within(
            e.into(),
            scratch.edge(e).map_err(TopoError::from)?.tolerance(),
        )?;
    }
    for &f in &closure.faces {
        within(
            f.into(),
            scratch.face(f).map_err(TopoError::from)?.tolerance(),
        )?;
    }
    Ok(())
}

/// `first` then `second`: every pair `a → b` of `first` whose `b` the
/// second holds, as `a → second(b)`.
fn compose(first: &IdMap, second: &IdMap) -> IdMap {
    fn then<K: Ord + Copy>(a: &BTreeMap<K, K>, b: &BTreeMap<K, K>) -> BTreeMap<K, K> {
        a.iter()
            .filter_map(|(&k, v)| b.get(v).map(|&w| (k, w)))
            .collect()
    }
    IdMap {
        vertices: then(&first.vertices, &second.vertices),
        edges: then(&first.edges, &second.edges),
        faces: then(&first.faces, &second.faces),
        shells: then(&first.shells, &second.shells),
        bodies: then(&first.bodies, &second.bodies),
        curves: then(&first.curves, &second.curves),
        surfaces: then(&first.surfaces, &second.surfaces),
        curve2s: then(&first.curve2s, &second.curve2s),
    }
}

/// The decoded body copied into `model` inside a transaction and checked
/// there at `Level::Full`, in every build profile: the bytes are input
/// from outside the kernel (ADR-0029 §6).
fn import(model: &mut Model, decoded: BodyIn, version: u32) -> Result<Imported, BodyError> {
    let BodyIn {
        model: scratch,
        body,
        provenance,
        map: written,
    } = decoded;
    model.transaction(|m| {
        // The import first: it resolves every reference the bytes hold,
        // geometry included, and the transaction undoes it on any refusal
        // after.
        let (copy, into) = m.import(&scratch, body)?;
        let closure = scratch.closure(body).map_err(TopoError::from)?;
        check_map(&written, &closure, body)?;
        check_tolerances(m, &scratch, &closure, &written.inverse())?;
        let report = check(m, copy, Level::Full);
        if !report.is_ok() {
            return Err(BodyError::Rejected(Box::new(report)));
        }
        Ok(Imported {
            body: copy,
            provenance,
            map: compose(&written, &into),
            version,
        })
    })
}
