//! One recipe step built in Arris: [`build_step`] dispatches on the step's
//! op to one function per op, grouped by family — [`solids`] (box,
//! cylinder, polyhedron, read), [`sketch`] (profile, planar face, extrude,
//! revolve, sweep, loft), [`motion`] (transform, mirror, split),
//! [`boolean`] (fuse, common, cut) and [`features`] (fillet, chamfer,
//! offset, shell). The dispatch names every `Step` variant: a new op fails
//! to compile here.

use super::*;

mod boolean;
mod features;
mod motion;
mod sketch;
mod solids;

#[cfg(test)]
pub(in crate::corpus) use solids::{ReadFault, read_solid};

/// What a step's function reads and writes: the model, the fixture and
/// the step it is building, the parameters, the bodies made so far, the
/// profiles sketched so far, and the cancellation control.
pub(super) struct Cx<'a, 'c> {
    m: &'a mut Model,
    fixture: &'a Fixture,
    step: &'a Step,
    params: &'a BTreeMap<String, f64>,
    made: &'a BTreeMap<String, Made>,
    profiles: &'a mut BTreeMap<String, Profile>,
    control: &'a Control<'c>,
}

impl Cx<'_, '_> {
    /// The operation's refusal, as the step's failure.
    fn op(&self, source: OpError) -> CorpusError {
        CorpusError::Op {
            fixture: self.fixture.name.clone(),
            step: self.step.name().to_string(),
            source,
        }
    }

    fn number(&self, n: &Num) -> Result<f64, CorpusError> {
        number(self.fixture, self.step, n, self.params)
    }

    fn point(&self, p: &[Num; 3]) -> Result<Point3, CorpusError> {
        point(self.fixture, self.step, p, self.params)
    }

    fn vector(&self, v: &[Num; 3]) -> Result<Vec3, CorpusError> {
        vector(self.fixture, self.step, v, self.params)
    }

    fn reference(&self, name: &str) -> Result<Body, CorpusError> {
        reference(self.fixture, self.step, name, self.made)
    }

    /// The axis of `origin` and `direction`, or the recipe's error.
    fn axis(&self, origin: Point3, direction: Vec3) -> Result<Axis, CorpusError> {
        Axis::new(origin, direction).map_err(|source| CorpusError::Axis {
            fixture: self.fixture.name.clone(),
            step: self.step.name().to_string(),
            source,
        })
    }
}

/// The step's result: `built` made from `inputs`, its one output the body.
fn solid(
    (body, provenance): (Body, Provenance),
    inputs: Vec<Body>,
) -> Result<Option<Made>, CorpusError> {
    Ok(Some(Made {
        body,
        provenance,
        inputs,
        outputs: vec![body],
    }))
}

/// Builds one step: a body step into the [`Made`] it returns, a `profile`
/// step into `profiles` and `None`, since a sketch is a value and makes no
/// body.
pub(super) fn build_step(
    m: &mut Model,
    fixture: &Fixture,
    step: &Step,
    params: &BTreeMap<String, f64>,
    made: &BTreeMap<String, Made>,
    profiles: &mut BTreeMap<String, Profile>,
    control: &Control<'_>,
) -> Result<Option<Made>, CorpusError> {
    let cx = &mut Cx {
        m,
        fixture,
        step,
        params,
        made,
        profiles,
        control,
    };
    match step {
        Step::Box { min, max, .. } => solids::cuboid(cx, min, max),
        Step::Cylinder {
            base,
            axis,
            radius,
            height,
            ..
        } => solids::cylinder(cx, base, axis, radius, height),
        Step::Polyhedron {
            points,
            faces,
            namespace,
            ..
        } => solids::polyhedron(cx, points, faces, *namespace),
        Step::Read {
            file,
            sha256,
            id,
            near,
            ..
        } => solids::read(cx, file, sha256, *id, near),
        Step::Profile {
            name,
            plane,
            outer,
            holes,
        } => sketch::profile(cx, name, plane, outer, holes),
        Step::PlanarFace { profile, .. } => sketch::planar_face(cx, profile),
        Step::Extrude {
            profile,
            direction,
            length,
            ..
        } => sketch::extrude(cx, profile, direction, length),
        Step::Revolve {
            profile,
            axis,
            angle_deg,
            ..
        } => sketch::revolve(cx, profile, axis, angle_deg),
        Step::Sweep {
            profile, path, law, ..
        } => sketch::sweep(cx, profile, path, law),
        Step::Loft {
            sections,
            kind,
            closed,
            ..
        } => sketch::loft(cx, sections, kind, *closed),
        Step::Transform {
            of,
            translate,
            rotate,
            ..
        } => motion::transform(cx, of, translate, rotate),
        Step::Mirror { of, plane, .. } => motion::mirror(cx, of, plane),
        Step::Split {
            of, plane, side, ..
        } => motion::split(cx, of, plane, side),
        Step::Fuse { a, b, bodies, .. } => boolean::fuse(cx, a, b, bodies),
        Step::Common { a, b, .. } => boolean::common(cx, a, b),
        Step::Cut {
            target,
            tool,
            tools,
            ..
        } => boolean::cut(cx, target, tool, tools),
        Step::Fillet {
            of,
            edges,
            radius: size,
            ..
        }
        | Step::Chamfer {
            of,
            edges,
            distance: size,
            ..
        } => features::blend(cx, of, edges, size),
        Step::Offset {
            of,
            faces,
            distance,
            ..
        } => features::offset(cx, of, faces, distance),
        Step::Shell {
            of,
            openings,
            thickness,
            side,
            ..
        } => features::shell(cx, of, openings, thickness, *side),
    }
}

/// The operand names of a `fuse` or `cut` step, from its two single-name
/// fields or its list, never both.
pub(super) fn operand_names<'a>(
    fixture: &Fixture,
    step: &Step,
    single: [&'a Option<String>; 2],
    list: &'a [String],
    what: &str,
) -> Result<Vec<&'a str>, CorpusError> {
    let named: Vec<&str> = single.iter().filter_map(|n| n.as_deref()).collect();
    let wrong = |what: String| CorpusError::Operands {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        what,
    };
    match (named.is_empty(), list.is_empty()) {
        (false, false) => Err(wrong(format!(
            "a {what} names its operands singly or as a list, not both"
        ))),
        (true, true) => Err(wrong(format!("a {what} names no operands"))),
        (false, true) => Ok(named),
        (true, false) => Ok(list.iter().map(String::as_str).collect()),
    }
}

pub(super) fn reference(
    fixture: &Fixture,
    step: &Step,
    name: &str,
    made: &BTreeMap<String, Made>,
) -> Result<Body, CorpusError> {
    let missing = || CorpusError::Reference {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        name: name.to_string(),
    };
    if let Some(m) = made.get(name) {
        return Ok(m.body);
    }
    // `<split>.positive` and `<split>.negative`: a side of a split step.
    let (base, side) = name.rsplit_once('.').ok_or_else(missing)?;
    let sides = made.get(base).map(|m| m.outputs.as_slice());
    match (side, sides) {
        ("positive", Some([positive, _])) => Ok(*positive),
        ("negative", Some([_, negative])) => Ok(*negative),
        _ => Err(missing()),
    }
}
