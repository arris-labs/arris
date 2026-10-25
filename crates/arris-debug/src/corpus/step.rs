use super::*;

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
    let name = fixture.name.clone();
    let op = |source: OpError| CorpusError::Op {
        fixture: name.clone(),
        step: step.name().to_string(),
        source,
    };
    let body = |(body, provenance): (Body, Provenance), inputs: Vec<Body>| {
        Ok(Some(Made {
            body,
            provenance,
            inputs,
            outputs: vec![body],
        }))
    };
    match step {
        Step::Box { min, max, .. } => {
            let (min, max) = (
                point(fixture, step, min, params)?,
                point(fixture, step, max, params)?,
            );
            body(
                arris_ops::primitive_box(m, min, max, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Cylinder {
            base,
            axis,
            radius,
            height,
            ..
        } => {
            let base = point(fixture, step, base, params)?;
            let direction = point(fixture, step, axis, params)?;
            let axis = Axis::new(base, Vec3::from(direction.coords)).map_err(|source| {
                CorpusError::Axis {
                    fixture: name.clone(),
                    step: step.name().to_string(),
                    source,
                }
            })?;
            let radius = number(fixture, step, radius, params)?;
            let height = number(fixture, step, height, params)?;
            body(
                arris_ops::primitive_cylinder(m, axis, radius, height, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Profile {
            name,
            plane,
            outer,
            holes,
        } => {
            let profile = build_profile(name, plane, outer, holes, params).map_err(|source| {
                CorpusError::Profile {
                    fixture: fixture.name.clone(),
                    step: name.clone(),
                    source: Box::new(source),
                }
            })?;
            profiles.insert(name.clone(), profile);
            Ok(None)
        }
        Step::PlanarFace { profile, .. } => {
            let profile = profile_reference(fixture, step, profile, profiles)?;
            body(
                arris_ops::planar_face(m, profile, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Extrude {
            profile,
            direction,
            length,
            ..
        } => {
            let profile = profile_reference(fixture, step, profile, profiles)?;
            let direction = vector(fixture, step, direction, params)?;
            let length = number(fixture, step, length, params)?;
            body(
                arris_ops::extrude(m, profile, direction, length, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Revolve {
            profile,
            axis,
            angle_deg,
            ..
        } => {
            let profile = profile_reference(fixture, step, profile, profiles)?;
            let origin = point(fixture, step, &axis.origin, params)?;
            let direction = vector(fixture, step, &axis.direction, params)?;
            let axis = Axis::new(origin, direction).map_err(|source| CorpusError::Axis {
                fixture: name.clone(),
                step: step.name().to_string(),
                source,
            })?;
            let angle = number(fixture, step, angle_deg, params)?.to_radians();
            body(
                arris_ops::revolve(m, profile, axis, angle, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Sweep {
            profile, path, law, ..
        } => {
            let profile = profile_reference(fixture, step, profile, profiles)?;
            let path = sweep_path(fixture, step, path, params)?;
            let law = match law {
                SweepLaw::RotationMinimizing => FrameLaw::RotationMinimizing,
                SweepLaw::Binormal(b) => {
                    // Validated as an axis's direction is: a recipe with a
                    // zero binormal is the recipe's error.
                    let b = vector(fixture, step, b, params)?;
                    let axis =
                        Axis::new(Point3::origin(), b).map_err(|source| CorpusError::Axis {
                            fixture: name.clone(),
                            step: step.name().to_string(),
                            source,
                        })?;
                    FrameLaw::FixedBinormal(axis.direction)
                }
            };
            body(
                arris_ops::sweep(m, profile, &path, law, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Loft {
            sections,
            kind,
            closed,
            ..
        } => {
            let profile_of =
                |section: &String| profile_reference(fixture, step, section, profiles).cloned();
            let sections = sections
                .iter()
                .map(profile_of)
                .collect::<Result<Vec<_>, _>>()?;
            let kind = match kind {
                crate::fixtures::LoftKind::Ruled => arris_ops::LoftKind::Ruled,
                crate::fixtures::LoftKind::Smooth => arris_ops::LoftKind::Smooth,
            };
            body(
                arris_ops::loft(m, &sections, kind, *closed, control).map_err(op)?,
                Vec::new(),
            )
        }
        Step::Transform {
            of,
            translate,
            rotate,
            ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let motion = motion(fixture, step, rotate, translate, params)?;
            body(
                arris_ops::transform(m, of_body, &motion, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Mirror { of, plane, .. } => {
            let of_body = reference(fixture, step, of, made)?;
            let reflection = Reflection::new(
                point(fixture, step, &plane.origin, params)?,
                vector(fixture, step, &plane.normal, params)?,
            )
            .map_err(|source| CorpusError::Plane {
                fixture: fixture.name.clone(),
                step: step.name().to_string(),
                source,
            })?;
            body(
                arris_ops::mirror(m, of_body, &reflection, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Split {
            of, plane, side, ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let origin = point(fixture, step, &plane.origin, params)?;
            let normal = vector(fixture, step, &plane.normal, params)?;
            let frame = match &plane.x {
                Some(x) => Frame::new(origin, normal, vector(fixture, step, x, params)?),
                None => Frame::from_z(origin, normal),
            }
            .map_err(|source| CorpusError::Axis {
                fixture: fixture.name.clone(),
                step: step.name().to_string(),
                source,
            })?;
            let positive = match side {
                Some(side) => number(fixture, step, side, params)? > 0.0,
                None => true,
            };
            let made = arris_ops::split(m, of_body, &frame, control).map_err(op)?;
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
        Step::Fuse { a, b, bodies, .. } => {
            let names = operand_names(fixture, step, [a, b], bodies, "fuse")?;
            let operands = names
                .into_iter()
                .map(|n| reference(fixture, step, n, made))
                .collect::<Result<Vec<Body>, CorpusError>>()?;
            body(
                arris_ops::fuse_many(m, &operands, control).map_err(op)?,
                operands,
            )
        }
        Step::Common { a, b, .. } => {
            let a = reference(fixture, step, a, made)?;
            let b = reference(fixture, step, b, made)?;
            body(common(m, a, b, control).map_err(op)?, vec![a, b])
        }
        Step::Cut {
            target,
            tool,
            tools,
            ..
        } => {
            let names = operand_names(fixture, step, [tool, &None], tools, "cut")?;
            let target = reference(fixture, step, target, made)?;
            let tools = names
                .into_iter()
                .map(|n| reference(fixture, step, n, made))
                .collect::<Result<Vec<Body>, CorpusError>>()?;
            let operands: Vec<Body> = core::iter::once(target)
                .chain(tools.iter().copied())
                .collect();
            body(
                arris_ops::cut_many(m, target, &tools, control).map_err(op)?,
                operands,
            )
        }
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
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let probe = fixture.recipe.tolerances.probe;
            let mut selected = Vec::with_capacity(edges.len());
            for p in edges {
                let point = point(fixture, step, p, params)?;
                let edge =
                    edge_at(m, of_body, point, probe).map_err(|what| CorpusError::EdgePoint {
                        fixture: name.clone(),
                        step: step.name().to_string(),
                        point: [point.x, point.y, point.z],
                        what,
                    })?;
                selected.push(edge);
            }
            let size = number(fixture, step, size, params)?;
            let blended = if matches!(step, Step::Chamfer { .. }) {
                arris_ops::chamfer(m, of_body, &selected, size, control)
            } else {
                arris_ops::fillet(m, of_body, &selected, size, control)
            };
            body(blended.map_err(op)?, vec![of_body])
        }
        Step::Offset {
            of,
            faces,
            distance,
            ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let mut selected = Vec::with_capacity(faces.len());
            for p in faces {
                let point = point(fixture, step, p, params)?;
                let face = face_at(m, of_body, point).map_err(|what| CorpusError::FacePoint {
                    fixture: name.clone(),
                    step: step.name().to_string(),
                    point: [point.x, point.y, point.z],
                    what,
                })?;
                selected.push(face);
            }
            let distance = number(fixture, step, distance, params)?;
            body(
                arris_ops::offset_faces(m, of_body, &selected, distance, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Shell {
            of,
            openings,
            thickness,
            side,
            ..
        } => {
            let of_body = reference(fixture, step, of, made)?;
            let mut selected = Vec::with_capacity(openings.len());
            for p in openings {
                let point = point(fixture, step, p, params)?;
                let face = face_at(m, of_body, point).map_err(|what| CorpusError::FacePoint {
                    fixture: name.clone(),
                    step: step.name().to_string(),
                    point: [point.x, point.y, point.z],
                    what,
                })?;
                selected.push(face);
            }
            let thickness = number(fixture, step, thickness, params)?;
            let side = match side {
                ShellSide::Inward => arris_ops::ShellSide::Inward,
                ShellSide::Outward => arris_ops::ShellSide::Outward,
            };
            body(
                arris_ops::shell(m, of_body, &selected, thickness, side, control).map_err(op)?,
                vec![of_body],
            )
        }
        Step::Polyhedron {
            points,
            faces,
            namespace,
            ..
        } => {
            let points = points
                .iter()
                .map(|p| point(fixture, step, p, params))
                .collect::<Result<Vec<_>, _>>()?;
            // The staging and the build in one transaction: a refusal or
            // an interrupt leaves no geometry of the staged solid behind.
            let made = m.transaction(|m| {
                let (builder, keys) =
                    polyhedron(m, &points, faces, *namespace).map_err(|source| {
                        CorpusError::Polyhedron {
                            fixture: name.clone(),
                            step: step.name().to_string(),
                            source,
                        }
                    })?;
                arris_ops::build(m, builder, &keys, control).map_err(op)
            })?;
            body(made, Vec::new())
        }
        Step::Read {
            file,
            sha256,
            id,
            near,
            ..
        } => {
            let near = match near {
                Some(p) => Some(point(fixture, step, p, params)?),
                None => None,
            };
            let probe = fixture.recipe.tolerances.probe;
            let read =
                read_solid(m, &fixture.dir, file, sha256, *id, near, probe).map_err(|fault| {
                    match fault {
                        ReadFault::Refused(refusal) => CorpusError::Refused {
                            fixture: name.clone(),
                            step: step.name().to_string(),
                            refusal: Box::new(refusal),
                        },
                        ReadFault::File(what) => CorpusError::StepFile {
                            fixture: name.clone(),
                            step: step.name().to_string(),
                            what,
                        },
                    }
                })?;
            body((read.body, read.provenance), Vec::new())
        }
    }
}

/// Why a `step` operand gave no solid.
pub(super) enum ReadFault {
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
pub(super) fn read_solid(
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

/// The edge of `body` a recipe names by `point`: the one
/// `classify_point` answers `On(Edge)` for, when no second edge of the
/// body passes within `probe` of the point — the rule the oracle's
/// nearest-edge search keeps too (`tests/fixtures/README.md`). Errors:
/// what the point names instead.
/// The face of `body` that `point` classifies `On`: a point inside or
/// outside the body, or on an edge or a vertex, names no one face.
pub(super) fn face_at(m: &Model, body: Body, point: Point3) -> Result<Face, String> {
    match classify_point(m, body, point) {
        Ok(Classification::On(Shape {
            id: EntityId::Face(id),
            ..
        })) => Ok(Face::forward(id)),
        Ok(Classification::On(shape)) => Err(format!("is on {shape}, not inside a face")),
        Ok(Classification::Inside) => Err("is inside the body, on no face".into()),
        Ok(Classification::Outside) => Err("is outside the body, on no face".into()),
        Err(e) => Err(format!("could not be classified: {e}")),
    }
}

pub(super) fn edge_at(m: &Model, body: Body, point: Point3, probe: f64) -> Result<Edge, String> {
    let on = match classify_point(m, body, point) {
        Ok(Classification::On(shape)) => shape,
        Ok(Classification::Inside) => return Err("is inside the body, on no edge".into()),
        Ok(Classification::Outside) => return Err("is outside the body, on no edge".into()),
        Err(e) => return Err(format!("could not be classified: {e}")),
    };
    let EntityId::Edge(id) = on.id else {
        return Err(format!("is on {on}, not on an edge"));
    };
    let mut near: Vec<EntityId> = Vec::new();
    for edge in m.edges(body).map_err(|e| e.to_string())? {
        let entity = m.edge(edge.id).map_err(|e| e.to_string())?;
        let Some((curve, range)) = entity.curve() else {
            continue;
        };
        let curve = m.curve(curve).map_err(|e| e.to_string())?;
        let Ok(projection) = curve.project(point) else {
            continue;
        };
        let t = match curve.period() {
            Some(p) => {
                let k = ((range.lo() - projection.t) / p).ceil();
                projection.t + k * p
            }
            None => projection.t,
        };
        if projection.distance <= probe && range.lo() - probe <= t && t <= range.hi() + probe {
            near.push(EntityId::Edge(edge.id));
        }
    }
    if near.len() != 1 {
        return Err(format!(
            "is within the probe tolerance of {} edges ({})",
            near.len(),
            near.iter()
                .map(|e| e.to_string())
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    Ok(Edge::forward(id))
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
