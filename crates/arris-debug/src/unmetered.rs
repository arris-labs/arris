//! The operations that take a [`Control`], called with [`Control::NONE`]
//! (ADR-0030): what a test that does not care about cancellation imports
//! in place of `arris_ops`'s, so its calls read as the operation and its
//! operands. A test that does care calls the operation itself.

use arris_geom::Profile;
use arris_math::{Axis, Control, Isometry, Point3, Reflection, Vec3};
use arris_ops::boolean::Interferences;
use arris_ops::measure::MassProperties;
use arris_ops::{self, BuildKeys, OpError};
use arris_topo::builder::Builder;
use arris_topo::{Body, Edge, Face, Model, Provenance};
/// [`arris_ops::cut`] to its end.
pub fn cut(m: &mut Model, target: Body, tool: Body) -> Result<(Body, Provenance), OpError> {
    arris_ops::cut(m, target, tool, &Control::NONE)
}

/// [`arris_ops::fuse`] to its end.
pub fn fuse(m: &mut Model, a: Body, b: Body) -> Result<(Body, Provenance), OpError> {
    arris_ops::fuse(m, a, b, &Control::NONE)
}

/// [`arris_ops::common`] to its end.
pub fn common(m: &mut Model, a: Body, b: Body) -> Result<(Body, Provenance), OpError> {
    arris_ops::common(m, a, b, &Control::NONE)
}

/// [`arris_ops::boolean::interferences`] to its end.
pub fn interferences(m: &Model, a: Body, b: Body) -> Result<Interferences, OpError> {
    arris_ops::boolean::interferences(m, a, b, &Control::NONE)
}

/// [`arris_ops::primitive_box`] to its end.
pub fn primitive_box(
    m: &mut Model,
    min: impl Into<Point3>,
    max: impl Into<Point3>,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::primitive_box(m, min, max, &Control::NONE)
}

/// [`arris_ops::primitive_cylinder`] to its end.
pub fn primitive_cylinder(
    m: &mut Model,
    axis: Axis,
    radius: f64,
    height: f64,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::primitive_cylinder(m, axis, radius, height, &Control::NONE)
}

/// [`arris_ops::extrude`] to its end.
pub fn extrude(
    m: &mut Model,
    profile: &Profile,
    direction: Vec3,
    length: f64,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::extrude(m, profile, direction, length, &Control::NONE)
}

/// [`arris_ops::revolve`] to its end.
pub fn revolve(
    m: &mut Model,
    profile: &Profile,
    axis: Axis,
    angle: f64,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::revolve(m, profile, axis, angle, &Control::NONE)
}

/// [`arris_ops::fillet`] to its end.
pub fn fillet(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    radius: f64,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::fillet(m, body, edges, radius, &Control::NONE)
}

/// [`arris_ops::chamfer`] to its end.
pub fn chamfer(
    m: &mut Model,
    body: Body,
    edges: &[Edge],
    distance: f64,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::chamfer(m, body, edges, distance, &Control::NONE)
}

/// [`arris_ops::offset_faces`] to its end.
pub fn offset_faces(
    m: &mut Model,
    body: Body,
    faces: &[Face],
    distance: f64,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::offset_faces(m, body, faces, distance, &Control::NONE)
}

/// [`arris_ops::transform`] to its end.
pub fn transform(
    m: &mut Model,
    body: Body,
    motion: &Isometry,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::transform(m, body, motion, &Control::NONE)
}

/// [`arris_ops::mirror`] to its end.
pub fn mirror(
    m: &mut Model,
    body: Body,
    plane: &Reflection,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::mirror(m, body, plane, &Control::NONE)
}

/// [`arris_ops::build`] to its end.
pub fn build(
    m: &mut Model,
    builder: Builder,
    keys: &BuildKeys,
) -> Result<(Body, Provenance), OpError> {
    arris_ops::build(m, builder, keys, &Control::NONE)
}

/// [`arris_ops::measure::mass_properties`] to its end.
pub fn mass_properties(m: &Model, body: Body) -> Result<MassProperties, OpError> {
    arris_ops::measure::mass_properties(m, body, &Control::NONE)
}

/// [`arris_mesh::tessellate`] to its end.
pub fn tessellate(
    m: &Model,
    body: Body,
    chord: f64,
) -> Result<arris_mesh::TriMesh, arris_mesh::MeshError> {
    arris_mesh::tessellate(m, body, chord, &Control::NONE)
}

/// [`arris_mesh::tessellate_with`] to its end.
pub fn tessellate_with(
    m: &Model,
    body: Body,
    request: &arris_mesh::MeshRequest,
) -> Result<arris_mesh::TriMesh, arris_mesh::MeshError> {
    arris_mesh::tessellate_with(m, body, request, &Control::NONE)
}

/// [`arris_io::step::read`] to its end.
pub fn step_read(
    m: &mut Model,
    text: &str,
    options: &arris_io::step::ReadOptions,
) -> Result<arris_io::step::Read, arris_io::step::ReadError> {
    arris_io::step::read(m, text, options, &Control::NONE)
}

/// [`arris_io::body::read`] to its end.
pub fn body_read(
    m: &mut Model,
    bytes: &[u8],
) -> Result<arris_io::body::Imported, arris_io::body::BodyError> {
    arris_io::body::read(m, bytes, &Control::NONE)
}

/// [`arris_io::body::from_json`] to its end.
pub fn body_from_json(
    m: &mut Model,
    text: &str,
) -> Result<arris_io::body::Imported, arris_io::body::BodyError> {
    arris_io::body::from_json(m, text, &Control::NONE)
}
