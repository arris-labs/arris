//! The solids a recipe starts from: box, cylinder, polyhedron and a solid
//! read from a STEP file.

use super::*;

pub(super) fn cuboid(
    cx: &mut Cx<'_, '_>,
    min: &[Num; 3],
    max: &[Num; 3],
) -> Result<Option<Made>, CorpusError> {
    let (min, max) = (cx.point(min)?, cx.point(max)?);
    solid(
        arris_ops::primitive_box(cx.m, min, max, cx.control).map_err(|e| cx.op(e))?,
        Vec::new(),
    )
}

pub(super) fn cylinder(
    cx: &mut Cx<'_, '_>,
    base: &[Num; 3],
    axis: &[Num; 3],
    radius: &Num,
    height: &Num,
) -> Result<Option<Made>, CorpusError> {
    let base = cx.point(base)?;
    let direction = cx.point(axis)?;
    let axis = cx.axis(base, Vec3::from(direction.coords))?;
    let radius = cx.number(radius)?;
    let height = cx.number(height)?;
    solid(
        arris_ops::primitive_cylinder(cx.m, axis, radius, height, cx.control)
            .map_err(|e| cx.op(e))?,
        Vec::new(),
    )
}

pub(super) fn polyhedron(
    cx: &mut Cx<'_, '_>,
    points: &[[Num; 3]],
    faces: &[Vec<Vec<usize>>],
    namespace: u32,
) -> Result<Option<Made>, CorpusError> {
    let points = points
        .iter()
        .map(|p| cx.point(p))
        .collect::<Result<Vec<_>, _>>()?;
    // The staging and the build in one transaction: a refusal or
    // an interrupt leaves no geometry of the staged solid behind.
    let (fixture, step, control) = (cx.fixture, cx.step, cx.control);
    let made =
        cx.m.transaction(|m| {
            let (builder, keys) = crate::polyhedron::polyhedron(m, &points, faces, namespace)
                .map_err(|source| CorpusError::Polyhedron {
                    fixture: fixture.name.clone(),
                    step: step.name().to_string(),
                    source,
                })?;
            arris_ops::build(m, builder, &keys, control).map_err(|source| CorpusError::Op {
                fixture: fixture.name.clone(),
                step: step.name().to_string(),
                source,
            })
        })?;
    solid(made, Vec::new())
}

pub(super) fn read(
    cx: &mut Cx<'_, '_>,
    file: &str,
    sha256: &str,
    id: u64,
    near: &Option<[Num; 3]>,
) -> Result<Option<Made>, CorpusError> {
    let near = match near {
        Some(p) => Some(cx.point(p)?),
        None => None,
    };
    let probe = cx.fixture.recipe.tolerances.probe;
    let (fixture, step) = (cx.fixture, cx.step);
    let read =
        read_solid(cx.m, &fixture.dir, file, sha256, id, near, probe).map_err(
            |fault| match fault {
                ReadFault::Refused(refusal) => CorpusError::Refused {
                    fixture: fixture.name.clone(),
                    step: step.name().to_string(),
                    refusal: Box::new(refusal),
                },
                ReadFault::File(what) => CorpusError::StepFile {
                    fixture: fixture.name.clone(),
                    step: step.name().to_string(),
                    what,
                },
            },
        )?;
    solid((read.body, read.provenance), Vec::new())
}

/// Why a `step` operand gave no solid.
pub(in crate::corpus) enum ReadFault {
    /// The reader refused the solid named.
    Refused(io_step::Refusal),
    /// The file is missing, is not the one the recipe hashed, does not
    /// parse, or names no single solid.
    File(String),
}

/// The solid a `step` operand names ([`Step::Read`]): the file `file`
/// beside the fixture in `dir`, held to `sha256`, read into `m` by
/// `arris_io::step::read` in millimetres; of its instances of `#id`, the
/// only one, or the one whose centroid is nearest `near`. Every instance
/// of `#id` must read: a refusal of any is the operand's. A tie for
/// nearest — two centroids whose distances from `near` differ by no more
/// than `probe` — names no solid.
pub(in crate::corpus) fn read_solid(
    m: &mut Model,
    dir: &Path,
    file: &str,
    sha256: &str,
    id: u64,
    near: Option<Point3>,
    probe: f64,
) -> Result<io_step::ReadBody, ReadFault> {
    let path = dir.join(file);
    let bytes =
        std::fs::read(&path).map_err(|e| ReadFault::File(format!("{}: {e}", path.display())))?;
    let digest: String = Sha256::digest(&bytes)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    if digest != sha256 {
        return Err(ReadFault::File(format!(
            "{} hashes to {digest}, not the recipe's {sha256}",
            path.display()
        )));
    }
    // Part 21 is ISO 8859-1 outside its escapes; a byte that is not UTF-8
    // can only sit in a string, whose text the reader does not use.
    let text = String::from_utf8_lossy(&bytes);
    let read = step_read(m, &text, &io_step::ReadOptions::default())
        .map_err(|e| ReadFault::File(format!("{}: {e}", path.display())))?;
    let mut candidates = Vec::new();
    for solid in read.solids.into_iter().filter(|s| s.entity.id == id) {
        candidates.push(solid.result.map_err(ReadFault::Refused)?);
    }
    match (candidates.len(), near) {
        (0, _) => Err(ReadFault::File(format!("{file} has no solid #{id}"))),
        (1, _) => Ok(candidates.remove(0)),
        (n, None) => Err(ReadFault::File(format!(
            "{file} places #{id} {n} times: name one by `near`"
        ))),
        (_, Some(near)) => {
            let mut by_distance = Vec::with_capacity(candidates.len());
            for (k, c) in candidates.iter().enumerate() {
                let props = mass_properties(m, c.body)
                    .map_err(|e| ReadFault::File(format!("#{id}, placement {k}: {e}")))?;
                by_distance.push(((props.centroid - near).norm(), k));
            }
            by_distance.sort_by(|a, b| a.0.total_cmp(&b.0));
            if by_distance[1].0 - by_distance[0].0 <= probe {
                return Err(ReadFault::File(format!(
                    "two placements of #{id} are as near {near:?}"
                )));
            }
            Ok(candidates.swap_remove(by_distance[0].1))
        }
    }
}
