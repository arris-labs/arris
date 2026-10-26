//! Checker `Fast` (`docs/DATA-MODEL.md` §Invariants), one binary filed by
//! entity row: for each row a sample body broken through the raw insert API
//! reports that row on that entity, together with exactly the rows its
//! definition implies and nothing else; the sample bodies are clean and the
//! report is the same on two fresh builds.
//!
//! Three rows measure the same triangle from different corners — V2 (vertex
//! against the curve's end), V3 (vertex against the surface at the
//! pcurve's end), E4 (curve against the surface along the pcurve) — so a
//! moved range end or a shifted pcurve is reported under two of them; each
//! test names its full set.
//!
//! Most loop and face rows are about a face's own (u, v), so those tests
//! start from a *plate* (`support::plate`) — a sheet body of one planar face
//! whose loops are rings of line edges, its (u, v) the world's `x` and `y`.
//! A plate is clean at `Fast`, so what a test breaks is the only thing the
//! report holds, except where one row's definition implies another's.

mod edge;
mod face;
mod loops;
mod model;
mod shell;
mod support;
mod vertex;
