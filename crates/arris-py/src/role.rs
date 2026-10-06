//! `Role`: what an entity is to the operation that made it from nothing,
//! as a Python value.
//!
//! The kernel's [`topo::provenance::Role`] is a nest of enums. Python reads
//! it as three plain parts: a `kind` (the operation), a `part` (the
//! variant's own name) and `fields` (its payload, in declaration order).
//! [`view`] and [`from_parts`] are the two directions, pure so a test can
//! walk every variant without an interpreter; `view` is an exhaustive
//! `match`, so a role variant added in the kernel stops this crate
//! compiling until Python can read it.

use arris::topo::provenance::{
    BoxPart, ConsumerKey, Coord, CylinderPart, FileEntity, PlaneSide, Role as Kernel, Side,
    SplitPart, SweepPart,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyTuple;

/// One value of a role's payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Field {
    /// A coordinate or a side, by its variant name (`"Z"`, `"Max"`).
    Name(&'static str),
    /// An index, an id or a key.
    Uint(u64),
}

/// A role as `(kind, part, fields)`.
pub type View = (&'static str, &'static str, Vec<Field>);

const COORDS: [(&str, Coord); 3] = [("X", Coord::X), ("Y", Coord::Y), ("Z", Coord::Z)];
const SIDES: [(&str, Side); 2] = [("Min", Side::Min), ("Max", Side::Max)];
const PLANE_SIDES: [(&str, PlaneSide); 2] = [
    ("Positive", PlaneSide::Positive),
    ("Negative", PlaneSide::Negative),
];

fn coord(c: Coord) -> Field {
    Field::Name(match c {
        Coord::X => "X",
        Coord::Y => "Y",
        Coord::Z => "Z",
    })
}

fn side(s: Side) -> Field {
    Field::Name(match s {
        Side::Min => "Min",
        Side::Max => "Max",
    })
}

fn plane_side(s: PlaneSide) -> Field {
    Field::Name(match s {
        PlaneSide::Positive => "Positive",
        PlaneSide::Negative => "Negative",
    })
}

fn index(n: usize) -> Field {
    Field::Uint(n as u64)
}

/// The three parts of `role`.
pub fn view(role: Kernel) -> View {
    match role {
        Kernel::Box(part) => {
            let (name, fields) = match part {
                BoxPart::Body => ("Body", vec![]),
                BoxPart::Shell => ("Shell", vec![]),
                BoxPart::Face(c, s) => ("Face", vec![coord(c), side(s)]),
                BoxPart::Edge { along, sides } => {
                    ("Edge", vec![coord(along), side(sides[0]), side(sides[1])])
                }
                BoxPart::Vertex(sides) => ("Vertex", sides.into_iter().map(side).collect()),
            };
            ("box", name, fields)
        }
        Kernel::Cylinder(part) => (
            "cylinder",
            match part {
                CylinderPart::Body => "Body",
                CylinderPart::Shell => "Shell",
                CylinderPart::Wall => "Wall",
                CylinderPart::BottomCap => "BottomCap",
                CylinderPart::TopCap => "TopCap",
                CylinderPart::BottomRim => "BottomRim",
                CylinderPart::TopRim => "TopRim",
                CylinderPart::Seam => "Seam",
                CylinderPart::BottomVertex => "BottomVertex",
                CylinderPart::TopVertex => "TopVertex",
            },
            vec![],
        ),
        Kernel::Extrude(part) => {
            let (name, fields) = sweep_view(part);
            ("extrude", name, fields)
        }
        Kernel::Revolve(part) => {
            let (name, fields) = sweep_view(part);
            ("revolve", name, fields)
        }
        Kernel::File(FileEntity { id, instance }) => (
            "file",
            "Entity",
            vec![Field::Uint(id), Field::Uint(u64::from(instance))],
        ),
        Kernel::Consumer(ConsumerKey { namespace, key }) => (
            "consumer",
            "Key",
            vec![Field::Uint(u64::from(namespace)), Field::Uint(key)],
        ),
        Kernel::Split(SplitPart::Cap(s)) => ("split", "Cap", vec![plane_side(s)]),
    }
}

fn sweep_view(part: SweepPart) -> (&'static str, Vec<Field>) {
    match part {
        SweepPart::Body => ("Body", vec![]),
        SweepPart::Shell => ("Shell", vec![]),
        SweepPart::Cavity {
            loop_index,
            segment,
        } => ("Cavity", vec![index(loop_index), index(segment)]),
        SweepPart::StartCap => ("StartCap", vec![]),
        SweepPart::EndCap => ("EndCap", vec![]),
        SweepPart::Side {
            loop_index,
            segment,
        } => ("Side", vec![index(loop_index), index(segment)]),
        SweepPart::StartEdge {
            loop_index,
            segment,
        } => ("StartEdge", vec![index(loop_index), index(segment)]),
        SweepPart::EndEdge {
            loop_index,
            segment,
        } => ("EndEdge", vec![index(loop_index), index(segment)]),
        SweepPart::Rise { loop_index, vertex } => ("Rise", vec![index(loop_index), index(vertex)]),
        SweepPart::StartVertex { loop_index, vertex } => {
            ("StartVertex", vec![index(loop_index), index(vertex)])
        }
        SweepPart::EndVertex { loop_index, vertex } => {
            ("EndVertex", vec![index(loop_index), index(vertex)])
        }
    }
}

/// A cursor over a role's fields that names what it was reading when it
/// fails.
struct Fields<'a> {
    what: String,
    rest: &'a [Field],
}

impl Fields<'_> {
    fn pick<T: Copy>(&mut self, table: &[(&str, T)], label: &str) -> Result<T, String> {
        let (first, rest) = self
            .rest
            .split_first()
            .ok_or_else(|| format!("{} needs a {label}", self.what))?;
        self.rest = rest;
        match first {
            Field::Name(name) => table
                .iter()
                .find(|(n, _)| n == name)
                .map(|(_, v)| *v)
                .ok_or_else(|| format!("{} is not a {label}", name)),
            Field::Uint(n) => Err(format!("{n} is not a {label}")),
        }
    }

    fn uint(&mut self, label: &str) -> Result<u64, String> {
        match self.rest.split_first() {
            Some((Field::Uint(n), rest)) => {
                self.rest = rest;
                Ok(*n)
            }
            _ => Err(format!("{} needs {label} as a non-negative int", self.what)),
        }
    }

    fn done(self) -> Result<(), String> {
        if self.rest.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "{} takes {} fewer fields",
                self.what,
                self.rest.len()
            ))
        }
    }
}

fn narrow<T: TryFrom<u64>>(n: u64, label: &str) -> Result<T, String> {
    T::try_from(n).map_err(|_| format!("{label} {n} is too large"))
}

/// The role named by `(kind, part, fields)`: the inverse of [`view`].
/// Errors name the part of the request that is not a role.
pub fn from_parts(kind: &str, part: &str, fields: &[Field]) -> Result<Kernel, String> {
    let mut f = Fields {
        what: format!("{kind} part {part}"),
        rest: fields,
    };
    let role = match kind {
        "box" => Kernel::Box(match part {
            "Body" => BoxPart::Body,
            "Shell" => BoxPart::Shell,
            "Face" => BoxPart::Face(f.pick(&COORDS, "coordinate")?, f.pick(&SIDES, "side")?),
            "Edge" => BoxPart::Edge {
                along: f.pick(&COORDS, "coordinate")?,
                sides: [f.pick(&SIDES, "side")?, f.pick(&SIDES, "side")?],
            },
            "Vertex" => BoxPart::Vertex([
                f.pick(&SIDES, "side")?,
                f.pick(&SIDES, "side")?,
                f.pick(&SIDES, "side")?,
            ]),
            _ => return Err(format!("{part} is not a part of a box")),
        }),
        "cylinder" => Kernel::Cylinder(match part {
            "Body" => CylinderPart::Body,
            "Shell" => CylinderPart::Shell,
            "Wall" => CylinderPart::Wall,
            "BottomCap" => CylinderPart::BottomCap,
            "TopCap" => CylinderPart::TopCap,
            "BottomRim" => CylinderPart::BottomRim,
            "TopRim" => CylinderPart::TopRim,
            "Seam" => CylinderPart::Seam,
            "BottomVertex" => CylinderPart::BottomVertex,
            "TopVertex" => CylinderPart::TopVertex,
            _ => return Err(format!("{part} is not a part of a cylinder")),
        }),
        "extrude" => Kernel::Extrude(sweep_from(part, &mut f)?),
        "revolve" => Kernel::Revolve(sweep_from(part, &mut f)?),
        "file" => match part {
            "Entity" => Kernel::File(FileEntity {
                id: f.uint("an id")?,
                instance: narrow(f.uint("an instance")?, "instance")?,
            }),
            _ => return Err(format!("{part} is not a part of a file")),
        },
        "consumer" => match part {
            "Key" => Kernel::Consumer(ConsumerKey {
                namespace: narrow(f.uint("a namespace")?, "namespace")?,
                key: f.uint("a key")?,
            }),
            _ => return Err(format!("{part} is not a part of a consumer role")),
        },
        "split" => Kernel::Split(match part {
            "Cap" => SplitPart::Cap(f.pick(&PLANE_SIDES, "plane side")?),
            _ => return Err(format!("{part} is not a part of a split")),
        }),
        _ => {
            return Err(format!(
                "{kind} is not a kind of role (box, cylinder, extrude, revolve, file, consumer, split)"
            ));
        }
    };
    f.done()?;
    Ok(role)
}

fn sweep_from(part: &str, f: &mut Fields<'_>) -> Result<SweepPart, String> {
    let mut two = |a: &str, b: &str| -> Result<(usize, usize), String> {
        Ok((narrow(f.uint(a)?, a)?, narrow(f.uint(b)?, b)?))
    };
    Ok(match part {
        "Body" => SweepPart::Body,
        "Shell" => SweepPart::Shell,
        "StartCap" => SweepPart::StartCap,
        "EndCap" => SweepPart::EndCap,
        "Cavity" => {
            let (loop_index, segment) = two("a loop", "a segment")?;
            SweepPart::Cavity {
                loop_index,
                segment,
            }
        }
        "Side" => {
            let (loop_index, segment) = two("a loop", "a segment")?;
            SweepPart::Side {
                loop_index,
                segment,
            }
        }
        "StartEdge" => {
            let (loop_index, segment) = two("a loop", "a segment")?;
            SweepPart::StartEdge {
                loop_index,
                segment,
            }
        }
        "EndEdge" => {
            let (loop_index, segment) = two("a loop", "a segment")?;
            SweepPart::EndEdge {
                loop_index,
                segment,
            }
        }
        "Rise" => {
            let (loop_index, vertex) = two("a loop", "a vertex")?;
            SweepPart::Rise { loop_index, vertex }
        }
        "StartVertex" => {
            let (loop_index, vertex) = two("a loop", "a vertex")?;
            SweepPart::StartVertex { loop_index, vertex }
        }
        "EndVertex" => {
            let (loop_index, vertex) = two("a loop", "a vertex")?;
            SweepPart::EndVertex { loop_index, vertex }
        }
        _ => return Err(format!("{part} is not a part of a sweep")),
    })
}

/// What an entity is to the operation that made it from nothing: a box's
/// face, a cylinder's wall, a sweep's side, an entity of a file, a key
/// of the consumer's own, or a split's cap.
///
/// Frozen and hashable. `kind` is the operation (`"box"`, `"cylinder"`,
/// `"extrude"`, `"revolve"`, `"file"`, `"consumer"`, `"split"`), `part` the kernel's
/// own name for the variant and `fields` its payload, in declaration order;
/// `str(role)` is the kernel's text form (`box:Face(Z, Max)`). `Role(kind, part, *fields)` is the
/// inverse and raises `ValueError` naming what is not a role.
///
/// ```python
/// import arris
///
/// top = arris.Role("box", "Face", "Z", "Max")
/// assert (top.kind, top.part, top.fields) == ("box", "Face", ("Z", "Max"))
/// assert str(top) == "box:Face(Z, Max)"
/// assert arris.Role.consumer(7, 42).key == 42
/// ```
#[pyclass(frozen, eq, hash, module = "arris")]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Role {
    pub(crate) kernel: Kernel,
}

impl Role {
    /// The Python value of `kernel`.
    pub fn new(kernel: Kernel) -> Role {
        Role { kernel }
    }
}

fn to_py<'py>(py: Python<'py>, field: Field) -> PyResult<Bound<'py, PyAny>> {
    use pyo3::IntoPyObjectExt;
    match field {
        Field::Name(name) => name.into_bound_py_any(py),
        Field::Uint(n) => n.into_bound_py_any(py),
    }
}

#[pymethods]
impl Role {
    #[new]
    #[pyo3(signature = (kind, part, *fields))]
    fn py_new(kind: &str, part: &str, fields: &Bound<'_, PyTuple>) -> PyResult<Role> {
        let fields = fields
            .iter()
            .map(|item| {
                if let Ok(n) = item.extract::<u64>() {
                    Ok(Field::Uint(n))
                } else {
                    // A name is only meaningful as one of the kernel's own.
                    let name: String = item.extract().map_err(|_| {
                        PyValueError::new_err(format!("{item} is neither a name nor an index"))
                    })?;
                    COORDS
                        .iter()
                        .map(|(n, _)| *n)
                        .chain(SIDES.iter().map(|(n, _)| *n))
                        .chain(PLANE_SIDES.iter().map(|(n, _)| *n))
                        .find(|n| *n == name)
                        .map(Field::Name)
                        .ok_or_else(|| PyValueError::new_err(format!("{name} is not a name")))
                }
            })
            .collect::<PyResult<Vec<_>>>()?;
        from_parts(kind, part, &fields)
            .map(Role::new)
            .map_err(PyValueError::new_err)
    }

    /// The consumer's own role: `key` in `namespace`. The kernel never
    /// reads either; it carries the pair.
    #[staticmethod]
    fn consumer(namespace: u32, key: u64) -> Role {
        Role::new(Kernel::Consumer(ConsumerKey { namespace, key }))
    }

    /// The operation: `"box"`, `"cylinder"`, `"extrude"`, `"revolve"`,
    /// `"file"`, `"consumer"` or `"split"`.
    #[getter]
    fn kind(&self) -> &'static str {
        view(self.kernel).0
    }

    /// The kernel's name for the variant (`"Face"`, `"Wall"`, `"Side"`).
    #[getter]
    fn part(&self) -> &'static str {
        view(self.kernel).1
    }

    /// The variant's payload, in declaration order: coordinates and sides —
    /// a box's and a split plane's — as names, indices, ids and keys as ints.
    #[getter]
    fn fields<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyTuple>> {
        let items = view(self.kernel)
            .2
            .into_iter()
            .map(|f| to_py(py, f))
            .collect::<PyResult<Vec<_>>>()?;
        PyTuple::new(py, items)
    }

    /// A consumer role's namespace; `None` for any other.
    #[getter]
    fn namespace(&self) -> Option<u32> {
        match self.kernel {
            Kernel::Consumer(k) => Some(k.namespace),
            _ => None,
        }
    }

    /// A consumer role's key; `None` for any other.
    #[getter]
    fn key(&self) -> Option<u64> {
        match self.kernel {
            Kernel::Consumer(k) => Some(k.key),
            _ => None,
        }
    }

    fn __str__(&self) -> String {
        self.kernel.to_string()
    }

    fn __repr__(&self) -> String {
        format!("Role({})", self.__str__())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One value of every variant of every part enum.
    fn every_role() -> Vec<Kernel> {
        let mut roles = vec![
            Kernel::Box(BoxPart::Body),
            Kernel::Box(BoxPart::Shell),
            Kernel::Box(BoxPart::Face(Coord::Z, Side::Max)),
            Kernel::Box(BoxPart::Edge {
                along: Coord::Y,
                sides: [Side::Max, Side::Min],
            }),
            Kernel::Box(BoxPart::Vertex([Side::Min, Side::Max, Side::Min])),
            Kernel::File(FileEntity {
                id: 98,
                instance: 3,
            }),
            Kernel::Consumer(ConsumerKey {
                namespace: 7,
                key: u64::MAX,
            }),
            Kernel::Split(SplitPart::Cap(PlaneSide::Positive)),
            Kernel::Split(SplitPart::Cap(PlaneSide::Negative)),
        ];
        roles.extend(
            [
                CylinderPart::Body,
                CylinderPart::Shell,
                CylinderPart::Wall,
                CylinderPart::BottomCap,
                CylinderPart::TopCap,
                CylinderPart::BottomRim,
                CylinderPart::TopRim,
                CylinderPart::Seam,
                CylinderPart::BottomVertex,
                CylinderPart::TopVertex,
            ]
            .map(Kernel::Cylinder),
        );
        let (l, n) = (2, 5);
        let sweeps = [
            SweepPart::Body,
            SweepPart::Shell,
            SweepPart::Cavity {
                loop_index: l,
                segment: n,
            },
            SweepPart::StartCap,
            SweepPart::EndCap,
            SweepPart::Side {
                loop_index: l,
                segment: n,
            },
            SweepPart::StartEdge {
                loop_index: l,
                segment: n,
            },
            SweepPart::EndEdge {
                loop_index: l,
                segment: n,
            },
            SweepPart::Rise {
                loop_index: l,
                vertex: n,
            },
            SweepPart::StartVertex {
                loop_index: l,
                vertex: n,
            },
            SweepPart::EndVertex {
                loop_index: l,
                vertex: n,
            },
        ];
        roles.extend(sweeps.map(Kernel::Extrude));
        roles.extend(sweeps.map(Kernel::Revolve));
        roles
    }

    #[test]
    fn every_role_reads_back_as_itself() {
        let roles = every_role();
        let mut seen = std::collections::BTreeSet::new();
        for role in roles {
            let (kind, part, fields) = view(role);
            assert_eq!(
                from_parts(kind, part, &fields),
                Ok(role),
                "{kind}/{part}/{fields:?}"
            );
            assert!(
                seen.insert((kind, part, fields)),
                "{role} reads the same as another role"
            );
        }
    }

    #[test]
    fn a_request_that_is_no_role_names_what_is_wrong() {
        let name = |n| Field::Name(n);
        for (kind, part, fields, mentions) in [
            ("sphere", "Wall", vec![], "kind of role"),
            ("box", "Wall", vec![], "part of a box"),
            ("box", "Face", vec![name("Z")], "needs a side"),
            (
                "box",
                "Face",
                vec![name("W"), name("Max")],
                "not a coordinate",
            ),
            ("box", "Face", vec![name("Z"), name("Up")], "not a side"),
            (
                "box",
                "Face",
                vec![Field::Uint(1), name("Max")],
                "not a coordinate",
            ),
            ("box", "Body", vec![name("Z")], "fewer fields"),
            ("extrude", "Side", vec![Field::Uint(0)], "needs a segment"),
            ("extrude", "Side", vec![name("Z"), Field::Uint(0)], "a loop"),
            (
                "consumer",
                "Key",
                vec![Field::Uint(u64::MAX), Field::Uint(1)],
                "too large",
            ),
        ] {
            let err = from_parts(kind, part, &fields).unwrap_err();
            assert!(err.contains(mentions), "{kind}/{part}: {err}");
        }
    }
}
