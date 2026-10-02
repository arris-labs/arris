//! What a binding call can refuse before the kernel is reached, and its
//! Python classes. The kernel's own errors map here in `plans/python-binding`
//! step 4.

use arris::topo::{EntityId, NotFound};
use pyo3::PyErr;
use pyo3::create_exception;
use pyo3::exceptions::PyException;

create_exception!(
    _arris,
    ArrisError,
    PyException,
    "Every error the kernel or the binding returns is an instance of this class."
);
create_exception!(
    _arris,
    ForeignHandleError,
    ArrisError,
    "A handle from one `Model` was given to another."
);
create_exception!(
    _arris,
    StaleHandleError,
    ArrisError,
    "A handle no longer resolves in its model: its entity was freed by `Model.retain`."
);
create_exception!(
    _arris,
    ModelPoisonedError,
    ArrisError,
    "A call panicked inside this model's lock, so its contents cannot be trusted."
);

/// Why the binding refused a call before, or while, reaching the kernel.
#[derive(Debug, Clone, PartialEq)]
pub enum BindError {
    /// A handle minted by model `owner` was given to model `this`.
    Foreign {
        /// The entity the handle names.
        entity: EntityId,
        /// The serial of the model that minted it.
        owner: u64,
        /// The serial of the model it was given to.
        this: u64,
    },
    /// The kernel does not resolve the handle: its slot was freed or reused.
    Stale(NotFound),
    /// An earlier call panicked while holding the model's lock.
    Poisoned {
        /// The serial of the model.
        model: u64,
    },
}

impl From<NotFound> for BindError {
    fn from(not_found: NotFound) -> Self {
        BindError::Stale(not_found)
    }
}

impl core::fmt::Display for BindError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BindError::Foreign {
                entity,
                owner,
                this,
            } => write!(
                f,
                "handle {entity} belongs to model {owner}, not to model {this}"
            ),
            BindError::Stale(not_found) => not_found.fmt(f),
            BindError::Poisoned { model } => write!(
                f,
                "model {model} is poisoned: an earlier call panicked inside it"
            ),
        }
    }
}

impl std::error::Error for BindError {}

impl From<BindError> for PyErr {
    fn from(error: BindError) -> PyErr {
        let message = error.to_string();
        match error {
            BindError::Foreign { .. } => ForeignHandleError::new_err(message),
            BindError::Stale(_) => StaleHandleError::new_err(message),
            BindError::Poisoned { .. } => ModelPoisonedError::new_err(message),
        }
    }
}
