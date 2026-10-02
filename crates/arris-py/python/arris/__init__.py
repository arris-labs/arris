"""Arris: a B-Rep geometric kernel. A thin layer over the Rust crate."""

from ._arris import (
    ArrisError,
    Body,
    Edge,
    Face,
    ForeignHandleError,
    Model,
    ModelPoisonedError,
    Shell,
    StaleHandleError,
    Vertex,
    __version__,
)

__all__ = [
    "ArrisError",
    "Body",
    "Edge",
    "Face",
    "ForeignHandleError",
    "Model",
    "ModelPoisonedError",
    "Shell",
    "StaleHandleError",
    "Vertex",
    "__version__",
]
