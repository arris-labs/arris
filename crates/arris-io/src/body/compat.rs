//! Every earlier version of body bytes, read by migrating it one version
//! at a time to [`BODY_VERSION`] (ADR-0029 §3). No release has written a
//! version before 1, so this module holds no version yet; the chain is
//! proven by a version 0 that exists only in this crate's tests.
//!
//! **Bumping the version.** The commit that changes a type body bytes
//! carry (the model, its entities and geometry, `Provenance`, `IdMap`,
//! the body's own fields):
//!
//! 1. copies the old shape of only the types it touched into a module
//!    `compat::vN`, `N` the version being left, with the old `BodyIn`
//!    built from them and a `vN::BodyIn::up` giving the `vN+1` one;
//! 2. bumps [`BODY_VERSION`] to `N + 1`;
//! 3. adds `N`'s arm to the dispatch, decoding `vN::BodyIn` and calling
//!    `up` once per version to the newest, so every older arm gains one
//!    `up` as well;
//! 4. adds the new version's set to the guard and blesses it
//!    (`ARRIS_BLESS=1`) beside the old version's files, which stay as
//!    they are and keep reading to their committed dumps.
//!
//! Dropping an old version is a `Breaking` changelog line and an
//! amendment of ADR-0029, never a cleanup.

use serde::de::DeserializeOwned;

use super::{BODY_VERSION, BodyError, BodyIn};

/// Where a body is decoded from: the `postcard` bytes after the magic,
/// or the JSON tree.
pub(super) trait Source {
    /// The body as `T`, one version's shape.
    fn decode<T: DeserializeOwned>(self) -> Result<T, BodyError>;
}

/// The `postcard` bytes after [`super::BODY_MAGIC`], version included.
pub(super) struct Postcard<'b>(pub(super) &'b [u8]);

impl Source for Postcard<'_> {
    fn decode<T: DeserializeOwned>(self) -> Result<T, BodyError> {
        postcard::from_bytes::<(u32, T)>(self.0)
            .map(|(_, body)| body)
            .map_err(|e| BodyError::Decode(e.to_string()))
    }
}

impl Source for serde_json::Value {
    fn decode<T: DeserializeOwned>(self) -> Result<T, BodyError> {
        T::deserialize(self).map_err(|e| BodyError::Decode(e.to_string()))
    }
}

/// The body in `source`, written at `version`, migrated to
/// [`BODY_VERSION`]. Errors: [`BodyError::Version`] for a newer version;
/// [`BodyError::Decode`] for an older one no release wrote, or data that
/// is not a body of its version.
pub(super) fn decode(version: u32, source: impl Source) -> Result<BodyIn, BodyError> {
    match version {
        BODY_VERSION => source.decode(),
        #[cfg(test)]
        0 => source.decode::<v0::BodyIn>().map(v0::BodyIn::up),
        found if found > BODY_VERSION => Err(BodyError::Version {
            found,
            newest: BODY_VERSION,
        }),
        found => Err(BodyError::Decode(format!(
            "no release wrote body version {found}"
        ))),
    }
}

/// A version no release wrote, for the chain's test: version 1 with the
/// record optional, `None` where a body was written with none.
#[cfg(test)]
mod v0 {
    use arris_check::arris_topo::{Body, IdMap, Model, Provenance};
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize)]
    pub(super) struct BodyIn {
        pub(super) model: Model,
        pub(super) body: Body,
        pub(super) provenance: Option<Provenance>,
        pub(super) map: IdMap,
    }

    impl BodyIn {
        /// Version 1's shape: no record is the empty one.
        pub(super) fn up(self) -> super::BodyIn {
            super::BodyIn {
                model: self.model,
                body: self.body,
                provenance: self.provenance.unwrap_or_default(),
                map: self.map,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::{self as body, BODY_MAGIC, Imported, dense};
    use super::v0;
    use crate::native;
    use arris_check::arris_topo::{Model, Provenance};
    use arris_debug::unmetered::primitive_box;

    /// A box and its primitive's record, in a model that holds another
    /// box first so its ids are not dense.
    fn written() -> (Model, arris_check::arris_topo::Body, Provenance) {
        let mut m = Model::default();
        primitive_box(&mut m, [5.0; 3], [6.0; 3]).unwrap();
        let (b, record) = primitive_box(&mut m, [0.0; 3], [4.0, 3.0, 2.0]).unwrap();
        (m, b, record)
    }

    /// The box at version 0 with `record`, as bytes and as JSON.
    fn version_0(record: Option<Provenance>) -> (Vec<u8>, String) {
        let (m, b, _) = written();
        let (model, body, map) = dense(&m, b).unwrap();
        let old = v0::BodyIn {
            model,
            body,
            provenance: record,
            map,
        };
        let mut bytes = BODY_MAGIC.to_vec();
        bytes.extend(postcard::to_allocvec(&(0u32, &old)).unwrap());
        let mut tree = serde_json::to_value(&old).unwrap();
        tree["magic"] = "ARRISBDY".into();
        tree["version"] = 0.into();
        (bytes, tree.to_string())
    }

    /// `read` and the model it read into, asserting the read passed.
    fn into_fresh(
        read: impl FnOnce(&mut Model) -> Result<Imported, body::BodyError>,
    ) -> (Model, Imported) {
        let mut m = Model::default();
        let imported = read(&mut m).unwrap();
        (m, imported)
    }

    #[test]
    fn version_0_reads_through_read_as_its_version_1_twin() {
        let (m, b, record) = written();
        for (old, new) in [
            (Some(record.clone()), record),
            (None, Provenance::default()),
        ] {
            let (old_bytes, old_text) = version_0(old);
            let new_bytes = body::write(&m, b, &new).unwrap();
            let new_text = body::to_json(&m, b, &new).unwrap();
            let (m1, v1) = into_fresh(|t| body::read(t, &new_bytes));
            assert_eq!(v1.version, 1);
            let expected = native::to_bytes(&m1).unwrap();
            for (m0, v0) in [
                into_fresh(|t| body::read(t, &old_bytes)),
                into_fresh(|t| body::from_json(t, &old_text)),
                into_fresh(|t| body::from_json(t, &new_text)),
            ] {
                assert_eq!(native::to_bytes(&m0).unwrap(), expected);
                assert_eq!(Imported { version: 1, ..v0 }, v1);
            }
            let (_, from_old) = into_fresh(|t| body::read(t, &old_bytes));
            assert_eq!(from_old.version, 0, "the version the data carried");
        }
    }
}
