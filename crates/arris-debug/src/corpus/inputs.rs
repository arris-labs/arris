use super::*;

/// A fixture's recipe built up to, but not including, its result step:
/// what a test of the boolean decomposition reads a `boolean/*` fixture
/// through before the boolean exists to run.
#[derive(Debug)]
pub struct Inputs {
    /// The model the steps were built in.
    pub model: Model,
    /// Every body step before the result, by name.
    pub bodies: BTreeMap<String, Body>,
    /// Every `profile` step before the result, by name.
    pub profiles: BTreeMap<String, Profile>,
    /// The result step, unbuilt.
    pub result: Step,
    fixture: Fixture,
    params: BTreeMap<String, f64>,
    made: BTreeMap<String, Made>,
}

impl Inputs {
    /// The result step run in `model` under `control`: what a test that
    /// interrupts an operation calls on a clone of [`Inputs::model`].
    /// Errors: [`CorpusError::Op`] with [`OpError::Interrupted`] when
    /// `control` stops it, and as [`run`]'s build stage otherwise.
    ///
    /// ```no_run
    /// use arris_debug::{corpus, fixtures};
    /// use arris_ops::Control;
    ///
    /// let dir = fixtures::corpus_root().join("boolean/through-hole");
    /// let inputs = corpus::inputs(&dir, "default").unwrap();
    /// let mut model = inputs.model.clone();
    /// let made = inputs.run_result(&mut model, &Control::NONE).unwrap();
    /// assert!(!made.provenance.is_empty());
    /// ```
    pub fn run_result(
        &self,
        model: &mut Model,
        control: &Control<'_>,
    ) -> Result<Made, CorpusError> {
        let mut profiles = self.profiles.clone();
        build_step(
            model,
            &self.fixture,
            &self.result,
            &self.params,
            &self.made,
            &mut profiles,
            control,
        )?
        .ok_or_else(|| CorpusError::Reference {
            fixture: self.fixture.name.clone(),
            step: "result".into(),
            name: self.fixture.recipe.result.clone(),
        })
    }

    /// The target and the tools of a result step that is a `cut`, in the
    /// recipe's order — what the one call and the chain of cuts the
    /// benchmark times against each other both take — or `None` when the
    /// result is not a cut or a name does not resolve.
    ///
    /// ```no_run
    /// use arris_debug::{corpus, fixtures};
    ///
    /// let dir = fixtures::corpus_root().join("boolean/plate-2x2-holes-cut-many");
    /// let inputs = corpus::inputs(&dir, "default").unwrap();
    /// let (_plate, holes) = inputs.cut_operands().unwrap();
    /// assert_eq!(holes.len(), 4);
    /// ```
    pub fn cut_operands(&self) -> Option<(Body, Vec<Body>)> {
        let Step::Cut {
            target,
            tool,
            tools,
            ..
        } = &self.result
        else {
            return None;
        };
        let names: Vec<&String> = tool.iter().chain(tools.iter()).collect();
        let tools = names
            .into_iter()
            .map(|n| self.bodies.get(n).copied())
            .collect::<Option<Vec<_>>>()?;
        Some((*self.bodies.get(target)?, tools))
    }

    /// The two bodies the result step combines — `a` and `b` of a `fuse`
    /// or `common`, the target and the tool of a `cut` — or `None` when
    /// the result is not a boolean.
    pub fn operands(&self) -> Option<(Body, Body)> {
        let (x, y) = match &self.result {
            Step::Fuse {
                a: Some(a),
                b: Some(b),
                ..
            }
            | Step::Common { a, b, .. } => (a, b),
            Step::Cut {
                target,
                tool: Some(tool),
                ..
            } => (target, tool),
            Step::Fuse { .. } | Step::Cut { .. } => return None,
            Step::Box { .. }
            | Step::Cylinder { .. }
            | Step::Profile { .. }
            | Step::PlanarFace { .. }
            | Step::Extrude { .. }
            | Step::Revolve { .. }
            | Step::Sweep { .. }
            | Step::Loft { .. }
            | Step::Transform { .. }
            | Step::Mirror { .. }
            | Step::Split { .. }
            | Step::Fillet { .. }
            | Step::Chamfer { .. }
            | Step::Offset { .. }
            | Step::Shell { .. }
            | Step::Polyhedron { .. }
            | Step::Read { .. } => return None,
        };
        Some((*self.bodies.get(x)?, *self.bodies.get(y)?))
    }
}

/// Builds every step of `dir`'s recipe under `variant` before the one
/// named as the result, and returns them with the result step itself.
/// Errors: as [`run`]'s build stage, and [`CorpusError::Reference`] when
/// no step is named as the result.
///
/// ```no_run
/// use arris_debug::{corpus, fixtures};
///
/// let dir = fixtures::corpus_root().join("boolean/through-hole");
/// let inputs = corpus::inputs(&dir, "default").unwrap();
/// let (plate, hole) = inputs.operands().unwrap();
/// assert_ne!(plate, hole);
/// ```
pub fn inputs(dir: &Path, variant: &str) -> Result<Inputs, CorpusError> {
    inputs_of(fixtures::load(dir)?, variant)
}

/// [`inputs`] for the step `step` of a recipe with no directory — a
/// generated one (`prop::recipe`) — under its `default` variant: every
/// step before it is built (unmetered) and `step` is left as the result
/// step [`Inputs::run_result`] runs. Errors: as [`inputs`]'s, and
/// [`CorpusError::Reference`] when `step` names no step of the recipe.
///
/// ```
/// use arris_debug::corpus;
/// use arris_debug::fixtures::Recipe;
/// use arris_ops::Control;
///
/// let recipe: Recipe = serde_json::from_str(
///     r#"{"steps": [{"op": "box", "name": "b", "min": [0, 0, 0], "max": [1, 2, 3]}], "result": "b"}"#,
/// )
/// .unwrap();
/// let inputs = corpus::inputs_for_step("generated/box", &recipe, "b").unwrap();
/// let mut model = inputs.model.clone();
/// assert!(inputs.run_result(&mut model, &Control::budget(0)).is_err());
/// ```
pub fn inputs_for_step(name: &str, recipe: &Recipe, step: &str) -> Result<Inputs, CorpusError> {
    let mut recipe = recipe.clone();
    recipe.result = step.to_string();
    inputs_of(
        Fixture {
            dir: PathBuf::new(),
            name: name.to_string(),
            recipe,
            recipe_sha256: String::new(),
            expected: Expected {
                occt: String::new(),
                recipe_sha256: String::new(),
                results: BTreeMap::new(),
            },
            second_oracle: None,
        },
        "default",
    )
}

pub(super) fn inputs_of(fixture: Fixture, variant: &str) -> Result<Inputs, CorpusError> {
    let name = fixture.name.clone();
    let Some(params) = fixture.recipe.params_of(variant) else {
        return Err(CorpusError::Variant {
            fixture: name,
            variant: variant.to_string(),
        });
    };
    let mut model = model_for(&fixture)?;
    let mut made: BTreeMap<String, Made> = BTreeMap::new();
    let mut profiles: BTreeMap<String, Profile> = BTreeMap::new();
    for step in &fixture.recipe.steps {
        if step.name() == fixture.recipe.result {
            let bodies = made.iter().map(|(k, v)| (k.clone(), v.body)).collect();
            return Ok(Inputs {
                model,
                bodies,
                profiles,
                result: step.clone(),
                fixture: fixture.clone(),
                params,
                made,
            });
        }
        if let Some(out) = build_step(
            &mut model,
            &fixture,
            step,
            &params,
            &made,
            &mut profiles,
            &Control::NONE,
        )? {
            made.insert(step.name().to_string(), out);
        }
    }
    Err(CorpusError::Reference {
        fixture: name,
        step: "result".into(),
        name: fixture.recipe.result.clone(),
    })
}

pub(super) fn number(
    fixture: &Fixture,
    step: &Step,
    n: &Num,
    params: &BTreeMap<String, f64>,
) -> Result<f64, CorpusError> {
    n.eval(params).map_err(|source| CorpusError::Expression {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        source,
    })
}

pub(super) fn point(
    fixture: &Fixture,
    step: &Step,
    p: &[Num; 3],
    params: &BTreeMap<String, f64>,
) -> Result<Point3, CorpusError> {
    Ok(Point3::new(
        number(fixture, step, &p[0], params)?,
        number(fixture, step, &p[1], params)?,
        number(fixture, step, &p[2], params)?,
    ))
}

/// The kernel's path from a recipe's: each number evaluated, a helix's
/// angle from degrees, a NURBS segment validated.
pub(super) fn sweep_path(
    fixture: &Fixture,
    step: &Step,
    path: &SweepPath,
    params: &BTreeMap<String, f64>,
) -> Result<GeomPath, CorpusError> {
    let mut segments = Vec::with_capacity(path.segments.len());
    for (k, segment) in path.segments.iter().enumerate() {
        segments.push(match segment {
            RecipeSegment::Line { line_to } => {
                GeomSegment::LineTo(point(fixture, step, line_to, params)?)
            }
            RecipeSegment::Arc { arc_to, via } => GeomSegment::ArcTo {
                via: point(fixture, step, via, params)?,
                to: point(fixture, step, arc_to, params)?,
            },
            RecipeSegment::Helix { helix } => {
                let origin = point(fixture, step, &helix.axis.origin, params)?;
                let direction = vector(fixture, step, &helix.axis.direction, params)?;
                let axis = Axis::new(origin, direction).map_err(|source| CorpusError::Axis {
                    fixture: fixture.name.clone(),
                    step: step.name().to_string(),
                    source,
                })?;
                GeomSegment::Helix {
                    axis,
                    pitch: number(fixture, step, &helix.pitch, params)?,
                    angle: number(fixture, step, &helix.angle_deg, params)?.to_radians(),
                }
            }
            RecipeSegment::Nurbs { nurbs } => {
                let numbers = |of: &[Num]| {
                    of.iter()
                        .map(|n| number(fixture, step, n, params))
                        .collect::<Result<Vec<f64>, _>>()
                };
                let points = nurbs
                    .control_points
                    .iter()
                    .map(|p| point(fixture, step, p, params))
                    .collect::<Result<Vec<_>, _>>()?;
                let curve = NurbsCurve::new(
                    nurbs.degree,
                    numbers(&nurbs.knots)?,
                    points,
                    numbers(&nurbs.weights)?,
                )
                .map_err(|source| CorpusError::Path {
                    fixture: fixture.name.clone(),
                    step: step.name().to_string(),
                    segment: k,
                    source,
                })?;
                GeomSegment::Nurbs(curve)
            }
        });
    }
    Ok(GeomPath {
        start: point(fixture, step, &path.start, params)?,
        segments,
    })
}

pub(super) fn vector(
    fixture: &Fixture,
    step: &Step,
    v: &[Num; 3],
    params: &BTreeMap<String, f64>,
) -> Result<Vec3, CorpusError> {
    Ok(Vec3::new(
        number(fixture, step, &v[0], params)?,
        number(fixture, step, &v[1], params)?,
        number(fixture, step, &v[2], params)?,
    ))
}

/// The rigid motion of a `Step::Transform`: the rotation about `origin`
/// (the world origin when absent) applied first, then the translation —
/// `tools/oracle/oracle/recipe.py`'s `gp_Trsf` composition.
pub(super) fn motion(
    fixture: &Fixture,
    step: &Step,
    rotate: &Option<Rotate>,
    translate: &Option<[Num; 3]>,
    params: &BTreeMap<String, f64>,
) -> Result<Isometry, CorpusError> {
    let mut m = Isometry::identity();
    if let Some(rot) = rotate {
        let axis = vector(fixture, step, &rot.axis, params)?;
        let origin = match &rot.origin {
            Some(o) => point(fixture, step, o, params)?,
            None => Point3::origin(),
        };
        let angle = number(fixture, step, &rot.angle_deg, params)?.to_radians();
        let rotation = UnitQuaternion::from_axis_angle(&UnitVec3::new_normalize(axis), angle);
        let about_origin = Isometry::from_rotation(rotation);
        m = Isometry::new(
            rotation,
            origin.coords - about_origin.apply_vec(origin.coords),
        );
    }
    if let Some(t) = translate {
        m = m.then(&Isometry::from_translation(vector(
            fixture, step, t, params,
        )?));
    }
    Ok(m)
}

/// The profile a sweep step names, built by an earlier `profile` step.
pub(super) fn profile_reference<'a>(
    fixture: &Fixture,
    step: &Step,
    name: &str,
    profiles: &'a BTreeMap<String, Profile>,
) -> Result<&'a Profile, CorpusError> {
    profiles.get(name).ok_or_else(|| CorpusError::Reference {
        fixture: fixture.name.clone(),
        step: step.name().to_string(),
        name: name.to_string(),
    })
}
