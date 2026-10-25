//! The steps that move or cut one body: transform, mirror and split.

use super::*;
use crate::fixtures::{MirrorPlane, SplitPlane};

pub(super) fn transform(
    cx: &mut Cx<'_, '_>,
    of: &str,
    translate: &Option<[Num; 3]>,
    rotate: &Option<Rotate>,
) -> Result<Option<Made>, CorpusError> {
    let of_body = cx.reference(of)?;
    let motion = motion(cx.fixture, cx.step, rotate, translate, cx.params)?;
    solid(
        arris_ops::transform(cx.m, of_body, &motion, cx.control).map_err(|e| cx.op(e))?,
        vec![of_body],
    )
}

pub(super) fn mirror(
    cx: &mut Cx<'_, '_>,
    of: &str,
    plane: &MirrorPlane,
) -> Result<Option<Made>, CorpusError> {
    let of_body = cx.reference(of)?;
    let reflection =
        Reflection::new(cx.point(&plane.origin)?, cx.vector(&plane.normal)?).map_err(|source| {
            CorpusError::Plane {
                fixture: cx.fixture.name.clone(),
                step: cx.step.name().to_string(),
                source,
            }
        })?;
    solid(
        arris_ops::mirror(cx.m, of_body, &reflection, cx.control).map_err(|e| cx.op(e))?,
        vec![of_body],
    )
}

pub(super) fn split(
    cx: &mut Cx<'_, '_>,
    of: &str,
    plane: &SplitPlane,
    side: &Option<Num>,
) -> Result<Option<Made>, CorpusError> {
    let of_body = cx.reference(of)?;
    let origin = cx.point(&plane.origin)?;
    let normal = cx.vector(&plane.normal)?;
    let frame = match &plane.x {
        Some(x) => Frame::new(origin, normal, cx.vector(x)?),
        None => Frame::from_z(origin, normal),
    }
    .map_err(|source| CorpusError::Axis {
        fixture: cx.fixture.name.clone(),
        step: cx.step.name().to_string(),
        source,
    })?;
    let positive = match side {
        Some(side) => cx.number(side)? > 0.0,
        None => true,
    };
    let made = arris_ops::split(cx.m, of_body, &frame, cx.control).map_err(|e| cx.op(e))?;
    Ok(Some(Made {
        body: if positive {
            made.positive
        } else {
            made.negative
        },
        provenance: made.provenance,
        inputs: vec![of_body],
        outputs: vec![made.positive, made.negative],
    }))
}
