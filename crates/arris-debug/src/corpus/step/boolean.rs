//! The booleans of a recipe: fuse, common and cut.

use super::*;

pub(super) fn fuse(
    cx: &mut Cx<'_, '_>,
    a: &Option<String>,
    b: &Option<String>,
    bodies: &[String],
) -> Result<Option<Made>, CorpusError> {
    let names = operand_names(cx.fixture, cx.step, [a, b], bodies, "fuse")?;
    let operands = names
        .into_iter()
        .map(|n| cx.reference(n))
        .collect::<Result<Vec<Body>, CorpusError>>()?;
    solid(
        arris_ops::fuse_many(cx.m, &operands, cx.control).map_err(|e| cx.op(e))?,
        operands,
    )
}

pub(super) fn common(cx: &mut Cx<'_, '_>, a: &str, b: &str) -> Result<Option<Made>, CorpusError> {
    let a = cx.reference(a)?;
    let b = cx.reference(b)?;
    solid(
        arris_ops::common(cx.m, a, b, cx.control).map_err(|e| cx.op(e))?,
        vec![a, b],
    )
}

pub(super) fn cut(
    cx: &mut Cx<'_, '_>,
    target: &str,
    tool: &Option<String>,
    tools: &[String],
) -> Result<Option<Made>, CorpusError> {
    let names = operand_names(cx.fixture, cx.step, [tool, &None], tools, "cut")?;
    let target = cx.reference(target)?;
    let tools = names
        .into_iter()
        .map(|n| cx.reference(n))
        .collect::<Result<Vec<Body>, CorpusError>>()?;
    let operands: Vec<Body> = core::iter::once(target)
        .chain(tools.iter().copied())
        .collect();
    solid(
        arris_ops::cut_many(cx.m, target, &tools, cx.control).map_err(|e| cx.op(e))?,
        operands,
    )
}
