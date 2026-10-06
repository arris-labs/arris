//! The exceptions: one class per kernel error variant a binding call can
//! raise, under `ArrisError` (ADR-0034 §8), and what the binding refuses
//! before it reaches the kernel.
//!
//! [`Class`] lists every class once. The maps in [`crate::kernel_error`]
//! pick one with an exhaustive `match` per kernel enum, so a variant added
//! in the kernel stops this crate compiling until it has a class.

use arris::topo::{EntityId, NotFound};
use pyo3::PyErr;
use pyo3::create_exception;
use pyo3::exceptions::PyException;
use pyo3::prelude::*;

create_exception!(
    arris,
    ArrisError,
    PyException,
    "Every error the kernel or the binding returns is an instance of this class."
);

/// `Interrupted` as callers meet it: the macro's class under `ArrisError`
/// and the builtin `InterruptedError`, so `except InterruptedError` and
/// `except ArrisError` both catch it. `create_exception!` takes one base,
/// so the two-base class is made once, with `type(...)`.
fn interrupted_type(py: Python<'_>) -> PyResult<Bound<'_, pyo3::types::PyType>> {
    use pyo3::sync::PyOnceLock;
    use pyo3::types::{PyDict, PyTuple, PyType};
    static TYPE: PyOnceLock<Py<PyType>> = PyOnceLock::new();
    TYPE.get_or_try_init(py, || {
        let bases = PyTuple::new(
            py,
            [
                py.get_type::<Interrupted>().into_any(),
                py.get_type::<pyo3::exceptions::PyInterruptedError>()
                    .into_any(),
            ],
        )?;
        let namespace = PyDict::new(py);
        namespace.set_item("__module__", "arris")?;
        namespace.set_item("__doc__", Class::Interrupted.doc())?;
        let made = py
            .get_type::<PyType>()
            .call1(("Interrupted", bases, namespace))?;
        Ok::<_, PyErr>(made.cast_into::<PyType>()?.unbind())
    })
    .map(|t| t.bind(py).clone())
}

fn interrupted_error(message: String) -> PyErr {
    Python::attach(|py| match interrupted_type(py) {
        Ok(class) => PyErr::from_type(class, message),
        Err(failure) => failure,
    })
}

/// Declares each exception class once: its Python type, its place in the
/// [`Class`] enum, its name, and its registration in the module.
macro_rules! classes {
    ($( $name:ident($base:ident) = $doc:literal; )*) => {
        $( create_exception!(arris, $name, $base, $doc); )*

        /// An exception class of the module, by name: what a kernel error
        /// maps to before it is raised.
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub enum Class {
            /// [`ArrisError`] itself: the common base, never raised bare.
            Arris,
            $( #[doc = $doc] $name, )*
        }

        impl Class {
            /// Every class, in declaration order.
            pub const ALL: &'static [Class] = &[Class::Arris, $( Class::$name, )*];

            /// The class's docstring.
            pub const fn doc(self) -> &'static str {
                match self {
                    Class::Arris => "Every error the kernel or the binding returns is an instance of this class.",
                    $( Class::$name => $doc, )*
                }
            }

            /// The class's name in Python.
            pub const fn name(self) -> &'static str {
                match self {
                    Class::Arris => "ArrisError",
                    $( Class::$name => stringify!($name), )*
                }
            }

            /// The name of the class this one derives from.
            pub const fn base(self) -> &'static str {
                match self {
                    Class::Arris => "Exception",
                    $( Class::$name => stringify!($base), )*
                }
            }

            /// An error of this class with `message`.
            pub fn new_err(self, message: String) -> PyErr {
                if self == Class::Interrupted {
                    return interrupted_error(message);
                }
                match self {
                    Class::Arris => ArrisError::new_err(message),
                    $( Class::$name => $name::new_err(message), )*
                }
            }
        }

        /// Adds every exception class to the extension module.
        pub(crate) fn register(module: &Bound<'_, PyModule>) -> PyResult<()> {
            let py = module.py();
            module.add("ArrisError", py.get_type::<ArrisError>())?;
            $( module.add(stringify!($name), py.get_type::<$name>())?; )*
            module.add("Interrupted", interrupted_type(py)?)?;
            Ok(())
        }
    };
}

classes! {
    ForeignHandleError(ArrisError) = "A handle from one `Model` was given to another.";
    StaleHandleError(ArrisError) = "An id no longer resolves in its model: its entity was freed by `Model.retain`, or the kernel was handed an id that never named one.";
    ModelPoisonedError(ArrisError) = "A call panicked inside this model's lock, so its contents cannot be trusted.";
    Interrupted(ArrisError) = "The caller stopped the call, by its poll or its budget of steps; the model is as it was. Also an `InterruptedError`.";

    OpError(ArrisError) = "Why an operation (`ops`) failed.";
    OpInvalidInputError(OpError) = "An input body fails the checker.";
    OpUnsupportedError(OpError) = "No closed form for this pair of surfaces or curves yet.";
    OpDegenerateError(OpError) = "The requested result has no valid representation.";
    OpProfileError(OpError) = "The profile of a sweep is not a valid sketch.";
    OpToleranceError(OpError) = "The result would need an entity tolerance above the model's maximum.";
    OpInternalError(OpError) = "A kernel bug, caught by the operation's own checks.";
    OpUnkeyedError(OpError) = "A consumer-built slot has no key.";
    OpRejectedError(OpError) = "The consumer's topology was refused as a solid.";

    GeomError(ArrisError) = "Why a geometric query has no answer.";
    GeomUnsupportedError(GeomError) = "No closed form for this pair of kinds yet.";
    GeomDegenerateError(GeomError) = "An operand has no valid representation for the query.";
    GeomInvalidToleranceError(GeomError) = "The tolerance given is not finite and positive.";
    GeomAmbiguousError(GeomError) = "The point has no unique nearest point.";
    GeomAmbiguousUvError(GeomError) = "The (u, v) point has no unique nearest point on the pcurve.";
    GeomNotOnSurfaceError(GeomError) = "The curve does not lie on the surface within the tolerance.";
    GeomThroughSingularityError(GeomError) = "The curve runs through a singular point of the surface.";
    GeomDegenerateSectionError(GeomError) = "The section of two quadrics is degenerate.";

    FitError(ArrisError) = "Why a NURBS fit did not produce a curve.";
    FitDegenerateError(FitError) = "The fit request cannot be fitted.";
    FitInvalidToleranceError(FitError) = "The fit tolerance is not finite and positive.";
    FitNonFiniteError(FitError) = "The curve or its deviation is not finite.";
    FitDivergedError(FitError) = "The fit still deviates beyond the tolerance with the most spans it may use.";

    TopoError(ArrisError) = "Why a topological call on a model failed.";
    TopoPrecisionError(TopoError) = "The model's precision is not consistent, or a tolerance is outside it.";

    StepError(ArrisError) = "Why a STEP file could not be written or read.";
    StepParseError(StepError) = "The text is not a Part 21 exchange structure; `line`, `column` and `instance` say where.";
    StepUnsupportedError(StepError) = "The body has a structure the STEP writer has no form for.";
    StepLumpsError(StepError) = "The body's shells could not be read as lumps.";
    StepNonFiniteError(StepError) = "A number is not finite, and STEP has no spelling for it.";
    StepNoBodiesError(StepError) = "Nothing to write.";
    StepTreeError(StepError) = "The product tree cannot be written.";

    BodyError(ArrisError) = "Why a body's bytes could not be written or read.";
    BodyMagicError(BodyError) = "The data does not begin with the body magic.";
    BodyVersionError(BodyError) = "The data is of a newer version than this build reads.";
    BodyEncodeError(BodyError) = "The body could not be encoded.";
    BodyDecodeError(BodyError) = "The data is not a body of its version.";
    BodyPrecisionError(BodyError) = "An entity's tolerance is outside the reading model's.";
    BodyRejectedError(BodyError) = "The body decoded but fails the checker in the reading model.";

    NativeError(ArrisError) = "Why a model could not be written to or read from the native format.";
    NativeVersionError(NativeError) = "The data is of a native-format version this build does not read.";
    NativeEncodeError(NativeError) = "The model could not be encoded.";
    NativeDecodeError(NativeError) = "The data is not a model of this version.";

    MeshError(ArrisError) = "Why a mesh could not be built or a body tessellated.";
    MeshTooManyTrianglesError(MeshError) = "Binary STL's triangle count is a u32 and the meshes hold more triangles than that.";
    MeshIndexOutOfRangeError(MeshError) = "A triangle or edge index names no position.";
    MeshRangeOutOfBoundsError(MeshError) = "A range does not fit the list it indexes.";
    MeshNonFinitePositionError(MeshError) = "A position has a non-finite coordinate.";
    MeshNotInBodyError(MeshError) = "A face to mesh is not a face of the body.";
    MeshInvalidInputError(MeshError) = "The body fails the checker before tessellation.";
    MeshChordError(MeshError) = "The chord tolerance is not finite and positive.";
    MeshFaceError(MeshError) = "A face's domain could not be triangulated.";
    MeshGridTooLargeError(MeshError) = "A face's interior lattice would need too many points.";
    MeshCornersError(MeshError) = "A corner block does not fit together.";
    MeshInternalError(MeshError) = "Tessellation's own bookkeeping broke on validated input.";
}

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
    /// A provenance record of model `owner` was combined with one of model
    /// `this`: its origins and outputs name entities of different arenas.
    ForeignRecord {
        /// The serial of the model the record belongs to.
        owner: u64,
        /// The serial of the model it was combined with.
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
            BindError::ForeignRecord { owner, this } => write!(
                f,
                "a record of model {owner} cannot be combined with one of model {this}"
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

impl BindError {
    /// The class this error is raised as.
    pub fn class(&self) -> Class {
        match self {
            BindError::Foreign { .. } => Class::ForeignHandleError,
            BindError::ForeignRecord { .. } => Class::ForeignHandleError,
            BindError::Stale(_) => Class::StaleHandleError,
            BindError::Poisoned { .. } => Class::ModelPoisonedError,
        }
    }
}

impl From<BindError> for PyErr {
    fn from(error: BindError) -> PyErr {
        error.class().new_err(error.to_string())
    }
}
