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
//! let read = arris::io::step::read(&mut back, &text, &Default::default(), &Control::NONE).unwrap();
//! let body = read.solids[0].result.as_ref().unwrap().body;
//! assert!(check(&back, body, Level::Full).is_ok());
//! ```
//!
//! Every operation on a model takes a [`Control`] last, so its caller can
//! stop it: a poll it answers from whatever its platform has (here an
//! `AtomicBool` another thread sets), a budget of steps, both or neither.
//! A stop is `Interrupted` in the operation's own error type, and the
//! model is as it was before the call.
//!
//! ```
//! use arris::{Control, Stop};
//! use arris::ops::{OpError, primitive_box};
//! use arris::topo::Model;
//! use core::sync::atomic::{AtomicBool, Ordering};
//!
//! let mut m = Model::default();
//!
//! // A poll: the consumer's flag, asked at every step.
//! let cancel = AtomicBool::new(true);
//! let poll = || cancel.load(Ordering::Relaxed);
//! let stopped = primitive_box(&mut m, [0.0; 3], [1.0; 3], &Control::poll(&poll));
//! let Err(OpError::Interrupted(stop)) = stopped else { panic!("not stopped") };
//! assert_eq!(stop.by, Stop::Poll);
//! assert!(m.body(arris::topo::BodyId::new(0, 0)).is_err(), "nothing was left in the model");
//!
//! // A budget: the same input and budget stop at the same step everywhere.
//! let spent = primitive_box(&mut m, [0.0; 3], [1.0; 3], &Control::budget(0));
//! assert!(matches!(spent, Err(OpError::Interrupted(s)) if s.by == Stop::Budget && s.steps == 0));
//!
//! // Neither: run to the end. A budget that is enough changes nothing.
//! cancel.store(false, Ordering::Relaxed);
//! assert!(primitive_box(&mut m, [0.0; 3], [1.0; 3], &Control::poll(&poll).with_budget(100)).is_ok());
//! ```
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub use arris_math::{Control, Interrupted, Stop};

pub use arris_check as check;
pub use arris_geom as geom;
pub use arris_io as io;
pub use arris_math as math;
pub use arris_mesh as mesh;
pub use arris_ops as ops;
pub use arris_topo as topo;
