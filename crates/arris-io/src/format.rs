//! The spelling the text writers share.

/// A finite real in the mesh writers' and STEP's spelling: Rust's shortest
/// round-trip decimal with a decimal point always present and no negative
/// zero, so a reader parses back the same `f64`. `x` must be finite.
pub(crate) fn real(x: f64) -> String {
    let x = if x == 0.0 { 0.0 } else { x };
    let s = format!("{x}");
    if s.contains('.') { s } else { format!("{s}.") }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reals_are_shortest_round_trip_with_a_point() {
        assert_eq!(real(40.0), "40.");
        assert_eq!(real(0.5), "0.5");
        assert_eq!(real(-0.0), "0.");
    }
}
