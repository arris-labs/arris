//! `arris-py` — the Python binding: the package `arris` on PyPI.
//!
//! A thin 1:1 layer over the `arris` facade, one layer above it
//! (ADR-0034). Nothing here is geometry; the guarantees are the kernel's.
//! The crate is `publish = false` on crates.io and empty on
//! `wasm32-unknown-unknown`, where pyo3 does not build.
#![forbid(unsafe_code)]
#![warn(missing_docs)]
#![cfg(not(target_arch = "wasm32"))]

mod control;
pub mod error;
mod handle;
pub mod kernel_error;
mod model;
mod profile;
mod provenance;
mod query;
mod role;

pub use control::Cancel;
pub use error::{ArrisError, BindError, Class};
pub use handle::{Body, Edge, Face, Shell, Vertex};
pub use model::Model;
pub use profile::{Loop, Profile, Segment};
pub use provenance::Provenance;
pub use query::{EulerLine, Frame, MassProperties, Report, UncheckedRow, Violation};
pub use role::Role;

use pyo3::prelude::*;

/// The package version in PEP 440 form: the workspace's `0.5.0-dev` is
/// `0.5.0.dev0` to Python, and a release is the same string with no suffix.
pub fn python_version(cargo: &str) -> String {
    match cargo.split_once("-dev") {
        Some((release, "")) => format!("{release}.dev0"),
        _ => cargo.to_owned(),
    }
}

/// The extension module, `arris._arris`; the package's `__init__` re-exports it.
#[pymodule]
#[pyo3(name = "_arris")]
fn arris_py(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add("__version__", python_version(env!("CARGO_PKG_VERSION")))?;
    error::register(module)?;
    module.add_class::<Model>()?;
    module.add_class::<Body>()?;
    module.add_class::<Shell>()?;
    module.add_class::<Face>()?;
    module.add_class::<Edge>()?;
    module.add_class::<Vertex>()?;
    module.add_class::<Cancel>()?;
    module.add_class::<Role>()?;
    module.add_class::<Segment>()?;
    module.add_class::<Loop>()?;
    module.add_class::<Profile>()?;
    module.add_class::<Provenance>()?;
    module.add_class::<Frame>()?;
    module.add_class::<MassProperties>()?;
    module.add_class::<EulerLine>()?;
    module.add_class::<Violation>()?;
    module.add_class::<UncheckedRow>()?;
    module.add_class::<Report>()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::python_version;

    #[test]
    fn a_dev_version_is_a_pep_440_dev_release() {
        assert_eq!(python_version("0.5.0-dev"), "0.5.0.dev0");
        assert_eq!(python_version("0.5.0"), "0.5.0");
    }

    #[test]
    fn the_facade_is_reachable() {
        let mut model = arris::topo::Model::default();
        let (body, _) =
            arris::ops::primitive_box(&mut model, [0.0; 3], [1.0; 3], &arris::Control::NONE)
                .unwrap();
        assert!(arris::check::check(&model, body, arris::check::Level::Full).is_ok());
    }
}
