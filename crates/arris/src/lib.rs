//! `arris` — a B-Rep geometric kernel written from scratch in Rust.
//!
//! This crate is the facade: it re-exports the workspace crates as modules
//! (`arris::math` through `arris::io`) so a consumer depends on one name
//! and reaches everything through it. Nothing lives here that does not
//! live in a lower crate; the guarantees are each crate's
//! (`docs/ARCHITECTURE.md`).
//!
//! ```
//! use arris::math::{Axis, Control, Point3};
//! use arris::topo::Model;
//! use arris::check::{Level, check};
//!
//! let mut m = Model::default();
//! let (cylinder, provenance) =
//!     arris::ops::primitive_cylinder(&mut m, Axis::z_at(Point3::origin()), 4.0, 12.0, &Control::NONE)
//!         .unwrap();
//! assert!(check(&m, cylinder, Level::Full).is_ok());
//! assert_eq!(provenance.outputs().len(), 10);
//! let text = arris::io::step::write(&m, &[cylinder]).unwrap();
//! assert!(text.starts_with("ISO-10303-21;"));
//!
//! // And back: every solid of a file, each its own result.
//! let mut back = Model::default();
//! let read = arris::io::step::read(&mut back, &text, &Default::default()).unwrap();
//! let body = read.solids[0].result.as_ref().unwrap().body;
//! assert!(check(&back, body, Level::Full).is_ok());
//! ```
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub use arris_check as check;
pub use arris_geom as geom;
pub use arris_io as io;
pub use arris_math as math;
pub use arris_mesh as mesh;
pub use arris_ops as ops;
pub use arris_topo as topo;
