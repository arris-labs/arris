//! The native format (`docs/DATA-MODEL.md` §Native format): `serde` of
//! the model under a version header, as JSON for diffs or as `postcard`
//! bytes for size. Both are deterministic byte for byte for the same
//! model — the encoders visit the arenas in slot order and every map in
//! the model is ordered — and a model that round-trips through either
//! dumps identically before and after, with the same ids, freed slots
//! included.
//!
//! The schema is the model; [`NATIVE_VERSION`] is bumped when the model's
//! encoding changes, and a file of another version is
//! [`NativeError::Version`] — a refusal, since no migration exists yet
//! (a bump is a design delta that comes with one or with this refusal).

use arris_topo::Model;
use serde::{Deserialize, Serialize};

/// The version this crate writes and the only one it reads.
pub const NATIVE_VERSION: u32 = 1;

/// Why a model could not be written or read.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum NativeError {
    /// The data carries a version this crate does not read.
    #[error("native format version {found} is not {supported}, the one this build reads")]
    Version {
        /// The version in the data.
        found: u32,
        /// [`NATIVE_VERSION`].
        supported: u32,
    },
    /// The model could not be encoded.
    #[error("could not encode the model: {0}")]
    Encode(String),
    /// The data is not a model of this version: truncated, malformed, or
    /// holding a value that fails its type's validation (an inconsistent
    /// precision, a frame that is not orthonormal, a NURBS with bad
    /// knots).
    #[error("could not decode the model: {0}")]
    Decode(String),
}

/// What is written: the version first, so a reader of another version
/// stops at the header.
#[derive(Serialize)]
struct NativeOut<'m> {
    version: u32,
    model: &'m Model,
}

/// What is read, once the header has passed.
#[derive(Deserialize)]
struct NativeIn {
    #[allow(dead_code)]
    version: u32,
    model: Model,
}

/// The header alone, read before the model so a mismatch never fails as
/// a decode error deep inside the arenas.
#[derive(Deserialize)]
struct Header {
    version: u32,
}

fn check_version(found: u32) -> Result<(), NativeError> {
    if found == NATIVE_VERSION {
        Ok(())
    } else {
        Err(NativeError::Version {
            found,
            supported: NATIVE_VERSION,
        })
    }
}

/// The model as JSON text: one line, keys in declaration order, every
/// real in the shortest form that round-trips. For diffs and tests.
///
/// ```
/// use arris_topo::Model;
/// use arris_io::native;
///
/// let m = Model::default();
/// let text = native::to_json(&m).unwrap();
/// assert!(text.starts_with("{\"version\":1,"));
/// assert_eq!(native::from_json(&text).unwrap().precision(), m.precision());
/// ```
pub fn to_json(model: &Model) -> Result<String, NativeError> {
    serde_json::to_string(&NativeOut {
        version: NATIVE_VERSION,
        model,
    })
    .map_err(|e| NativeError::Encode(e.to_string()))
}

/// The model read back from [`to_json`]'s text. Errors:
/// [`NativeError::Version`] for another version;
/// [`NativeError::Decode`] for anything that is not a model.
pub fn from_json(text: &str) -> Result<Model, NativeError> {
    let header: Header =
        serde_json::from_str(text).map_err(|e| NativeError::Decode(e.to_string()))?;
    check_version(header.version)?;
    let native: NativeIn =
        serde_json::from_str(text).map_err(|e| NativeError::Decode(e.to_string()))?;
    Ok(native.model)
}

/// The model as `postcard` bytes: compact, deterministic, no schema in
/// the stream. For storage.
///
/// ```
/// use arris_topo::Model;
/// use arris_io::native;
///
/// let m = Model::default();
/// let bytes = native::to_bytes(&m).unwrap();
/// assert_eq!(bytes[0], 1, "the version, as a varint");
/// assert_eq!(native::from_bytes(&bytes).unwrap().precision(), m.precision());
/// ```
pub fn to_bytes(model: &Model) -> Result<Vec<u8>, NativeError> {
    postcard::to_allocvec(&NativeOut {
        version: NATIVE_VERSION,
        model,
    })
    .map_err(|e| NativeError::Encode(e.to_string()))
}

/// The model read back from [`to_bytes`]'s bytes. Errors:
/// [`NativeError::Version`] for another version;
/// [`NativeError::Decode`] for a truncated or malformed stream.
pub fn from_bytes(bytes: &[u8]) -> Result<Model, NativeError> {
    let (header, _): (Header, &[u8]) =
        postcard::take_from_bytes(bytes).map_err(|e| NativeError::Decode(e.to_string()))?;
    check_version(header.version)?;
    let native: NativeIn =
        postcard::from_bytes(bytes).map_err(|e| NativeError::Decode(e.to_string()))?;
    Ok(native.model)
}
