//! An ordered map on the wire as a sequence of `(key, value)` pairs in
//! key order: JSON has no map keyed by anything but text, and `postcard`
//! writes a map entry and a pair alike, so its bytes are the same as the
//! map's. Read back, the keys must be strictly ascending — what a
//! `BTreeMap` could have written — or the value is refused.

use core::fmt::Display;
use std::collections::BTreeMap;

use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

pub(crate) fn serialize<K: Serialize, V: Serialize, S: Serializer>(
    map: &BTreeMap<K, V>,
    serializer: S,
) -> Result<S::Ok, S::Error> {
    serializer.collect_seq(map)
}

pub(crate) fn deserialize<'de, K, V, D>(deserializer: D) -> Result<BTreeMap<K, V>, D::Error>
where
    K: Deserialize<'de> + Ord + Display,
    V: Deserialize<'de>,
    D: Deserializer<'de>,
{
    let pairs = Vec::<(K, V)>::deserialize(deserializer)?;
    let mut map = BTreeMap::new();
    for (k, v) in pairs {
        if map.last_key_value().is_some_and(|(last, _)| *last >= k) {
            return Err(D::Error::custom(format!("keys out of order at {k}")));
        }
        map.insert(k, v);
    }
    Ok(map)
}
