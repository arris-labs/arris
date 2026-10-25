//! The steps that sketch or sweep a profile: profile, planar face,
//! extrude, revolve, sweep and loft.

use super::*;
use crate::fixtures::{Loop, Plane};

pub(super) fn profile(
    cx: &mut Cx<'_, '_>,
    name: &str,
    plane: &Plane,
    outer: &Loop,
    holes: &[Loop],
) -> Result<Option<Made>, CorpusError> {
    let profile = build_profile(name, plane, outer, holes, cx.params).map_err(|source| {
        CorpusError::Profile {
            fixture: cx.fixture.name.clone(),
            step: name.to_string(),
            source: Box::new(source),
        }
    })?;
    cx.profiles.insert(name.to_string(), profile);
    Ok(None)
}

pub(super) fn planar_face(cx: &mut Cx<'_, '_>, profile: &str) -> Result<Option<Made>, CorpusError> {
    let profile = profile_reference(cx.fixture, cx.step, profile, cx.profiles)?;
    solid(
        arris_ops::planar_face(cx.m, profile, cx.control).map_err(|e| cx.op(e))?,
        Vec::new(),
    )
}

pub(super) fn extrude(
    cx: &mut Cx<'_, '_>,
    profile: &str,
    direction: &[Num; 3],
    length: &Num,
) -> Result<Option<Made>, CorpusError> {
    let profile = profile_reference(cx.fixture, cx.step, profile, cx.profiles)?;
    let direction = cx.vector(direction)?;
    let length = cx.number(length)?;
    solid(
        arris_ops::extrude(cx.m, profile, direction, length, cx.control).map_err(|e| cx.op(e))?,
        Vec::new(),
    )
}

pub(super) fn revolve(
    cx: &mut Cx<'_, '_>,
    profile: &str,
    axis: &fixtures::Axis,
    angle_deg: &Num,
) -> Result<Option<Made>, CorpusError> {
    let profile = profile_reference(cx.fixture, cx.step, profile, cx.profiles)?;
    let origin = cx.point(&axis.origin)?;
    let direction = cx.vector(&axis.direction)?;
    let axis = cx.axis(origin, direction)?;
    let angle = cx.number(angle_deg)?.to_radians();
    solid(
        arris_ops::revolve(cx.m, profile, axis, angle, cx.control).map_err(|e| cx.op(e))?,
        Vec::new(),
    )
}

pub(super) fn sweep(
    cx: &mut Cx<'_, '_>,
    profile: &str,
    path: &SweepPath,
    law: &SweepLaw,
) -> Result<Option<Made>, CorpusError> {
    let profile = profile_reference(cx.fixture, cx.step, profile, cx.profiles)?;
    let path = sweep_path(cx.fixture, cx.step, path, cx.params)?;
    let law = match law {
        SweepLaw::RotationMinimizing => FrameLaw::RotationMinimizing,
        SweepLaw::Binormal(b) => {
            // Validated as an axis's direction is: a recipe with a
            // zero binormal is the recipe's error.
            let b = cx.vector(b)?;
            FrameLaw::FixedBinormal(cx.axis(Point3::origin(), b)?.direction)
        }
    };
    solid(
        arris_ops::sweep(cx.m, profile, &path, law, cx.control).map_err(|e| cx.op(e))?,
        Vec::new(),
    )
}

pub(super) fn loft(
    cx: &mut Cx<'_, '_>,
    sections: &[String],
    kind: &crate::fixtures::LoftKind,
    closed: bool,
) -> Result<Option<Made>, CorpusError> {
    let sections = sections
        .iter()
        .map(|section| profile_reference(cx.fixture, cx.step, section, cx.profiles).cloned())
        .collect::<Result<Vec<_>, _>>()?;
    let kind = match kind {
        crate::fixtures::LoftKind::Ruled => arris_ops::LoftKind::Ruled,
        crate::fixtures::LoftKind::Smooth => arris_ops::LoftKind::Smooth,
    };
    solid(
        arris_ops::loft(cx.m, &sections, kind, closed, cx.control).map_err(|e| cx.op(e))?,
        Vec::new(),
    )
}
