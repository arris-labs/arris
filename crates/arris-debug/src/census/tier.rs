//! The committed tier in fixture order.

use super::*;

// Keep the committed tier's part order and share its recorded fillet filter.
// A census of all solids can ignore the predicate by passing `false`.
pub(super) fn over_committed_tier<T>(
    only_refused: bool,
    per_file: impl Fn(&Path, &dyn Fn(&str) -> bool) -> Result<T, String>,
) -> Result<Vec<(String, T)>, String> {
    let mut out = Vec::new();
    for name in COMMITTED_TIER {
        let fixture = part::load(&corpus_root().join(name)).map_err(|e| e.to_string())?;
        let refused: Vec<String> = (fixture.part.solids.iter())
            .filter(|s| matches!(s.battery.get("fillet"), Some(Class::ArrisRefuses(_))))
            .map(|s| crate::battery::key(s.id, s.instance))
            .collect();
        let file = fixture.dir.join(&fixture.part.file);
        let part = name.rsplit('/').next().unwrap_or(name).to_string();
        out.push((
            part,
            per_file(&file, &|k| !only_refused || refused.iter().any(|r| r == k))?,
        ));
    }
    Ok(out)
}
