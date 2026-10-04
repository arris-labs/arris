//! The refusal histogram (ADR-0026 §5): every refusal over the real-part
//! corpus mapped to the named cycle it blocks, counted as *cycle → parts
//! blocked*.
//!
//! The table is [`blocks_refusal`] for the reader's [`Refusal`],
//! [`blocks_parse`] for a file that does not parse, and [`blocks_reason`]
//! for an operation's [`OpError`] at a battery [`Stage`]. Each is an
//! exhaustive match with no wildcard arm, so a new `RefusalKind`, a new
//! `OpError` or a new [`Reason`] fails to compile here until the table has
//! its row — which only an ADR adds.
//!
//! A part fixture records the cycle of each refusal it holds (the
//! `blocks` of its solids), and the part runner holds that record to the
//! table on every run. [`Histogram::add_part`] therefore counts the
//! committed tier from the fixtures alone, and [`Histogram::add_refusal`]
//! counts a part run live, as the fetched tier is.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use arris_io::arris_check::arris_topo::arris_geom::{CurveKind, GeomKind, Surface, SurfaceKind};
use arris_io::arris_check::arris_topo::{EntityId, Model};
use arris_io::step::{ReadError, Refusal};
use arris_ops::{OpError, Reason};

use crate::battery::Class;
use crate::part::{Outcome as ReadOutcome, PartFixture};

/// The committed tier (ADR-0026 §1): NIST's eleven AP203 geometry-only
/// files, one `real/` fixture each. The other `real/` fixtures are shrunk
/// from these or spoiled by hand, and are no parts of their own.
pub const COMMITTED_TIER: [&str; 11] = [
    "real/nist-ctc-01",
    "real/nist-ctc-02",
    "real/nist-ctc-03",
    "real/nist-ctc-04",
    "real/nist-ctc-05",
    "real/nist-ftc-06",
    "real/nist-ftc-07",
    "real/nist-ftc-08",
    "real/nist-ftc-09",
    "real/nist-ftc-10",
    "real/nist-ftc-11",
];

/// A cycle a refusal blocks: one of `docs/ROADMAP.md`'s named cycles, or
/// the refusal counted as itself where no named cycle removes it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Cycle {
    /// The healing cycle: sewing, repair, sheet and wire bodies.
    Healing,
    /// The NURBS cycle: NURBS–NURBS intersection, NURBS operands in
    /// booleans.
    Nurbs,
    /// The blend-network cycle: chains, vertex blends, the pairs outside
    /// ADR-0007's table.
    BlendNetwork,
    /// The sweep row: STEP offsets and composites, oblique and elliptic
    /// sweeps. ADR-0047 split the sweep cycle (shell and offset are the
    /// prismatic-features cycle's, sweep and loft the NURBS cycle's); the
    /// row splits with them when a plan next touches this attribution.
    Sweep,
    /// Counted as itself, under this name.
    Itself(&'static str),
}

impl Cycle {
    /// The named cycles, in the order a histogram prints them at equal
    /// counts; every one has its row, blocking a part or not.
    pub const NAMED: [Cycle; 4] = [
        Cycle::Healing,
        Cycle::Nurbs,
        Cycle::BlendNetwork,
        Cycle::Sweep,
    ];
}

impl core::fmt::Display for Cycle {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Cycle::Healing => f.write_str("healing"),
            Cycle::Nurbs => f.write_str("NURBS"),
            Cycle::BlendNetwork => f.write_str("blend network"),
            Cycle::Sweep => f.write_str("sweep"),
            Cycle::Itself(name) => write!(f, "itself: {name}"),
        }
    }
}

/// Where a part is refused: the reader, or a stage of the battery
/// (ADR-0026 §5), in the order they run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Stage {
    /// The file read.
    Read,
    /// A solid read, tessellated and measured.
    Measure,
    /// Written and read back.
    WriteRead,
    /// Cut by the box.
    BoxCut,
    /// Drilled along the first principal axis.
    DrillX,
    /// Along the second.
    DrillY,
    /// Along the third.
    DrillZ,
    /// A sample of its edges filleted.
    Fillet,
}

impl Stage {
    /// Every stage, in order.
    pub const ALL: [Stage; 8] = [
        Stage::Read,
        Stage::Measure,
        Stage::WriteRead,
        Stage::BoxCut,
        Stage::DrillX,
        Stage::DrillY,
        Stage::DrillZ,
        Stage::Fillet,
    ];

    /// Its name, as a fixture records it and a histogram prints it.
    pub fn name(self) -> &'static str {
        match self {
            Stage::Read => "read",
            Stage::Measure => "measure",
            Stage::WriteRead => "write_read",
            Stage::BoxCut => "box_cut",
            Stage::DrillX => "drill_x",
            Stage::DrillY => "drill_y",
            Stage::DrillZ => "drill_z",
            Stage::Fillet => "fillet",
        }
    }

    /// The stage of a name, if it is one.
    pub fn of_name(name: &str) -> Option<Stage> {
        Stage::ALL.into_iter().find(|s| s.name() == name)
    }
}

/// The cycle a file that does not parse blocks: counted as itself, once
/// per file (ADR-0026 §5).
pub fn blocks_parse(error: &ReadError) -> Cycle {
    match error {
        ReadError::Parse(_) => Cycle::Itself("unparsed"),
        // The caller's stop, not the file's: the corpus runs to its end.
        ReadError::Interrupted(_) => Cycle::Itself("interrupted"),
    }
}

/// The cycle a reader's refusal blocks (ADR-0026 §5). It takes the
/// refusal whole: an `Unsupported` surface model is the healing cycle's,
/// a faceted B-rep counts as itself, and construction geometry — a
/// surface model only a `CONSTRUCTIVE_GEOMETRY_REPRESENTATION` holds — is
/// no body of the part and counts as itself (amendment of step 8).
///
/// ```
/// use arris_debug::histogram::{Cycle, blocks_refusal};
/// use arris_io::step::Refusal;
///
/// let gap = Refusal::Gap { entity: 7, gap: 0.02, cap: 0.01 };
/// assert_eq!(blocks_refusal(&gap), Cycle::Healing);
/// let sheet = Refusal::Unsupported { entity: 9, name: "SHELL_BASED_SURFACE_MODEL".into() };
/// assert_eq!(blocks_refusal(&sheet), Cycle::Healing);
/// let facets = Refusal::Unsupported { entity: 9, name: "FACETED_BREP".into() };
/// assert_eq!(blocks_refusal(&facets), Cycle::Itself("faceted"));
/// ```
pub fn blocks_refusal(refusal: &Refusal) -> Cycle {
    match refusal {
        Refusal::NoLengthUnit { .. } => Cycle::Itself("no length unit"),
        Refusal::Malformed { .. } => Cycle::Itself("malformed"),
        Refusal::Offset { .. } | Refusal::Composite { .. } => Cycle::Sweep,
        Refusal::CurveBounded { .. } => Cycle::Itself("curve-bounded surface"),
        Refusal::DegenerateTorus { .. } | Refusal::SelfIntersectingTorus { .. } => {
            Cycle::Itself("torus crossing its axis")
        }
        Refusal::Unsupported { name, .. } => {
            let sheet = ["SHELL_BASED_SURFACE_MODEL", "FACE_BASED_SURFACE_MODEL"];
            let faceted = ["FACETED_BREP", "TESSELLATED"];
            if name.contains("CONSTRUCTIVE_GEOMETRY_REPRESENTATION") {
                Cycle::Itself("supplemental geometry")
            } else if sheet.iter().any(|s| name.contains(s)) {
                Cycle::Healing
            } else if faceted.iter().any(|s| name.contains(s)) {
                Cycle::Itself("faceted")
            } else {
                Cycle::Itself("outside the subset")
            }
        }
        Refusal::Degenerate { .. } => Cycle::Itself("degenerate geometry"),
        Refusal::Topology { .. }
        | Refusal::OpenLoop { .. }
        | Refusal::Gap { .. }
        | Refusal::Invalid { .. } => Cycle::Healing,
        Refusal::Pcurve { .. } => Cycle::Nurbs,
    }
}

/// How the axes of two cylinders a blend is refused for lie
///: the pair is the blend network's only where they
/// are parallel, which ADR-0036 §5 closes with a cylinder; any other pair
/// meets in a curve that is a quartic or an ellipse, and its blend is a
/// surface with no exact kind, which is the NURBS cycle's.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CylinderPair {
    /// The axes are parallel.
    Parallel,
    /// The axes meet: a crease of two cylinders, whose radii are equal
    /// where the crease is a planar ellipse.
    Crossing {
        /// Whether the two radii are equal.
        equal_radii: bool,
    },
    /// The axes are skew.
    Skew,
}

impl core::fmt::Display for CylinderPair {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            CylinderPair::Parallel => "parallel axes",
            CylinderPair::Crossing { equal_radii: true } => "crossing axes, equal radii",
            CylinderPair::Crossing { equal_radii: false } => "crossing axes",
            CylinderPair::Skew => "skew axes",
        })
    }
}

/// The axes of the two cylinders an `Unsupported` pair names, in `m`'s
/// tolerances: `None` unless both are faces of `m` on cylinders.
pub fn cylinder_pair(m: &Model, error: &OpError) -> Option<CylinderPair> {
    let OpError::Unsupported { a, b } = error else {
        return None;
    };
    let cylinder = |id: EntityId| -> Option<(
        arris_io::arris_check::arris_topo::arris_math::Point3,
        arris_io::arris_check::arris_topo::arris_math::Vec3,
        f64,
    )> {
        let EntityId::Face(face) = id else {
            return None;
        };
        match m.surface(m.face(face).ok()?.surface()).ok()? {
            Surface::Cylinder { frame, radius } => {
                Some((frame.origin(), frame.z().into_inner(), *radius))
            }
            Surface::Plane { .. }
            | Surface::EllipticCylinder { .. }
            | Surface::Cone { .. }
            | Surface::Sphere { .. }
            | Surface::Torus { .. }
            | Surface::Nurbs(_) => None,
        }
    };
    let ((oa, da, ra), (ob, db, rb)) = (cylinder(a.1.id)?, cylinder(b.1.id)?);
    let precision = m.precision();
    let across = da.cross(&db);
    if across.norm() <= precision.angular_tolerance {
        return Some(CylinderPair::Parallel);
    }
    let gap = (ob - oa).dot(&across).abs() / across.norm();
    Some(if gap <= precision.default_tolerance {
        CylinderPair::Crossing {
            equal_radii: (ra - rb).abs() <= precision.default_tolerance,
        }
    } else {
        CylinderPair::Skew
    })
}

/// [`blocks_reason`] for a refusal in the model `m` it was raised on: a
/// fillet's cylinder pair whose axes are not parallel is the NURBS cycle's,
/// its blend having no exact surface; every other refusal is
/// [`blocks_reason`]'s.
pub fn blocks_reason_in(m: &Model, stage: Stage, error: &OpError) -> Option<Cycle> {
    match cylinder_pair(m, error) {
        Some(CylinderPair::Crossing { .. } | CylinderPair::Skew) if stage == Stage::Fillet => {
            Some(Cycle::Nurbs)
        }
        Some(CylinderPair::Parallel | CylinderPair::Crossing { .. } | CylinderPair::Skew)
        | None => blocks_reason(stage, error),
    }
}

/// The cycle an operation's refusal at a battery stage blocks (ADR-0026
/// §5), or `None` for [`OpError::Internal`]: a kernel bug, never a count.
///
/// ```
/// use arris_debug::histogram::{Cycle, Stage, blocks_reason};
/// use arris_ops::{OpError, Reason};
///
/// let chain = OpError::Degenerate { entities: Vec::new(), reason: Reason::TangentChain };
/// assert_eq!(blocks_reason(Stage::Fillet, &chain), Some(Cycle::BlendNetwork));
/// let empty = OpError::Degenerate { entities: Vec::new(), reason: Reason::Empty };
/// assert_eq!(blocks_reason(Stage::BoxCut, &empty), Some(Cycle::Itself("no material")));
/// ```
pub fn blocks_reason(stage: Stage, error: &OpError) -> Option<Cycle> {
    Some(match error {
        OpError::Unsupported { a, b } => {
            let nurbs = |k: GeomKind| match k {
                GeomKind::Surface(s) => s == SurfaceKind::Nurbs,
                GeomKind::Curve(c) => c == CurveKind::Nurbs,
                GeomKind::Curve2(_) | GeomKind::Point => false,
            };
            if nurbs(a.0) || nurbs(b.0) {
                Cycle::Nurbs
            } else if stage == Stage::Fillet {
                Cycle::BlendNetwork
            } else {
                Cycle::Itself("unsupported pair")
            }
        }
        OpError::Degenerate { reason, .. } => match reason {
            Reason::BlendTooLarge | Reason::TangentChain | Reason::VertexBlend => {
                Cycle::BlendNetwork
            }
            Reason::DirectionNotNormal | Reason::EllipticRevolve { .. } => Cycle::Sweep,
            Reason::NotSolid => Cycle::Healing,
            Reason::TangentContact | Reason::NonManifold => Cycle::Itself("non-manifold"),
            Reason::BesideSingularity => Cycle::Itself("beside a singularity"),
            Reason::Empty | Reason::ZeroThickness => Cycle::Itself("no material"),
            // Argument and sweep-profile errors the battery should never
            // raise: a count here is a battery bug to read.
            Reason::NonFinite { .. } => Cycle::Itself("NonFinite"),
            Reason::NotPositive { .. } => Cycle::Itself("NotPositive"),
            Reason::NoEdges => Cycle::Itself("NoEdges"),
            Reason::RepeatedEdge => Cycle::Itself("RepeatedEdge"),
            Reason::EdgeNotInBody => Cycle::Itself("EdgeNotInBody"),
            Reason::NotProjectable => Cycle::Itself("NotProjectable"),
            Reason::DegenerateEdge => Cycle::Itself("DegenerateEdge"),
            Reason::ProjectionCollapses => Cycle::Itself("ProjectionCollapses"),
            Reason::NotPlanar => Cycle::Itself("NotPlanar"),
            Reason::OutOfDomain => Cycle::Itself("OutOfDomain"),
            Reason::Singular => Cycle::Itself("Singular"),
            Reason::ProfileCrossesAxis => Cycle::Itself("ProfileCrossesAxis"),
            Reason::AxisNotInProfilePlane => Cycle::Itself("AxisNotInProfilePlane"),
            Reason::AngleAboveTurn => Cycle::Itself("AngleAboveTurn"),
            Reason::SpindleTorus => Cycle::Itself("SpindleTorus"),
        },
        OpError::InvalidInput { .. } => Cycle::Itself("InvalidInput"),
        OpError::Profile(_) => Cycle::Itself("Profile"),
        OpError::Tolerance { .. } => Cycle::Itself("Tolerance"),
        OpError::NotFound(_) => Cycle::Itself("NotFound"),
        // `build`'s refusals: the battery never builds a consumer's
        // topology, so a count here is a battery bug to read.
        OpError::Unkeyed { .. } => Cycle::Itself("Unkeyed"),
        OpError::Rejected(_) => Cycle::Itself("Rejected"),
        // The battery passes `Control::NONE`: a count here is a battery
        // bug to read.
        OpError::Interrupted(_) => Cycle::Itself("Interrupted"),
        OpError::Internal(_) => return None,
    })
}

/// The refusals over a set of parts, counted (ADR-0026 §5).
///
/// Guarantees: deterministic — the same parts added in any order print
/// the same text, byte for byte.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Histogram {
    /// Every part added.
    parts: BTreeSet<String>,
    /// Solid instances read and refused.
    read: usize,
    refused: usize,
    /// Battery stages by class: `agree`, `both-refuse`, …
    classes: BTreeMap<String, usize>,
    /// Per cycle, each part it blocks and the first stage it does.
    blocked: BTreeMap<String, BTreeMap<String, Stage>>,
    /// Each refusal, by cycle, stage and its name: how many times.
    refusals: BTreeMap<(String, Stage, String), usize>,
}

/// Where a histogram cannot count a part: a refusal with no cycle
/// recorded, a stage that waits, or a part that waits.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("{part}: {what}")]
pub struct Uncounted {
    /// The part.
    pub part: String,
    /// What is missing.
    pub what: String,
}

impl Histogram {
    /// An empty histogram.
    pub fn new() -> Histogram {
        Histogram::default()
    }

    /// Counts `part`, whether or not anything refuses it.
    pub fn add(&mut self, part: &str) {
        self.parts.insert(part.to_string());
    }

    /// Counts a solid instance of `part` read.
    pub fn add_read(&mut self, part: &str) {
        self.add(part);
        self.read += 1;
    }

    /// Counts a battery stage of `part` at `class`: `agree`,
    /// `both-refuse`, `oracle-refuses` or `arris-refuses`.
    pub fn add_class(&mut self, part: &str, class: &str) {
        self.add(part);
        *self.classes.entry(class.to_string()).or_default() += 1;
    }

    /// Counts one refusal of `part` at `stage`, named `name` — the
    /// reader's refusal kind, or the operation's error as the
    /// differential names it — as blocking `cycle`. A refusal at
    /// [`Stage::Read`] is a solid refused.
    pub fn add_refusal(&mut self, part: &str, stage: Stage, name: &str, cycle: &str) {
        self.add(part);
        if stage == Stage::Read {
            self.refused += 1;
        }
        let first = (self.blocked.entry(cycle.to_string()).or_default())
            .entry(part.to_string())
            .or_insert(stage);
        *first = (*first).min(stage);
        *(self.refusals)
            .entry((cycle.to_string(), stage, name.to_string()))
            .or_default() += 1;
    }

    /// Counts a part fixture from what it records: each solid read or
    /// refused, each battery stage's class, and each refusal under the
    /// cycle its `blocks` names, which the part runner holds to the table.
    /// Errors: a part or a stage that waits on a fix, and a refusal with
    /// no cycle recorded.
    pub fn add_part(&mut self, fixture: &PartFixture) -> Result<(), Uncounted> {
        let part = fixture.name.as_str();
        let uncounted = |what: String| Uncounted {
            part: part.to_string(),
            what,
        };
        if let Some(slug) = fixture.part.waits_on.first() {
            return Err(uncounted(format!("waits on {slug}")));
        }
        self.add(part);
        for solid in &fixture.part.solids {
            let at = crate::battery::key(solid.id, solid.instance);
            let cycle = |stage: &str| {
                solid
                    .blocks
                    .get(stage)
                    .ok_or_else(|| uncounted(format!("{at} {stage} records no cycle")))
            };
            match &solid.outcome {
                ReadOutcome::Refused(r) => {
                    self.add_refusal(part, Stage::Read, &r.kind, cycle("read")?);
                }
                ReadOutcome::Read => self.add_read(part),
            }
            for (name, class) in &solid.battery {
                let stage = Stage::of_name(name)
                    .ok_or_else(|| uncounted(format!("{at}: no stage {name}")))?;
                match class {
                    Class::Agree => self.add_class(part, "agree"),
                    Class::BothRefuse => self.add_class(part, "both-refuse"),
                    Class::OracleRefuses => self.add_class(part, "oracle-refuses"),
                    Class::ArrisRefuses(why) => {
                        self.add_class(part, "arris-refuses");
                        self.add_refusal(part, stage, why, cycle(name)?);
                    }
                    Class::WaitsOn(slug) => {
                        return Err(uncounted(format!("{at} {name} waits on {slug}")));
                    }
                }
            }
        }
        Ok(())
    }

    /// Counts a part surveyed live (`crate::survey`): its solids read and
    /// refused, its battery's classes, and each refusal under the cycle
    /// the table gave it.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn add_report(&mut self, report: &crate::survey::Report) {
        let part = report.part.as_str();
        self.add(part);
        self.read += report.read;
        for (_, _, class) in &report.classes {
            *self.classes.entry(class.clone()).or_default() += 1;
        }
        for r in &report.refusals {
            let stage = Stage::of_name(&r.stage).unwrap_or(Stage::Read);
            self.add_refusal(part, stage, &r.name, &r.cycle);
        }
    }

    /// How many parts `cycle` blocks.
    pub fn parts_blocked(&self, cycle: &str) -> usize {
        self.blocked.get(cycle).map_or(0, BTreeMap::len)
    }

    /// The histogram as markdown: a summary line; *cycle → parts blocked*,
    /// each part under the first stage the cycle blocks it at, every named
    /// cycle with its row, most parts first; and every refusal by name,
    /// most frequent first.
    ///
    /// ```
    /// use arris_debug::histogram::{Histogram, Stage};
    ///
    /// let mut h = Histogram::new();
    /// h.add_read("real/a");
    /// h.add_refusal("real/b", Stage::Read, "gap past the cap", "healing");
    /// let text = h.markdown();
    /// assert!(text.contains("| healing | 1 | 1 |"));
    /// assert!(text.contains("| NURBS | 0 |"));
    /// ```
    pub fn markdown(&self) -> String {
        let mut out = String::new();
        let class = |c: &str| self.classes.get(c).copied().unwrap_or(0);
        let stages: usize = self.classes.values().sum();
        let _ = writeln!(
            out,
            "{} parts, {} solids: {} read, {} refused. {stages} battery stages: {} agree, {} both refuse, {} Open CASCADE refuses, {} Arris refuses.",
            self.parts.len(),
            self.read + self.refused,
            self.read,
            self.refused,
            class("agree"),
            class("both-refuse"),
            class("oracle-refuses"),
            class("arris-refuses"),
        );
        out.push('\n');

        let named: Vec<String> = Cycle::NAMED.iter().map(Cycle::to_string).collect();
        let mut cycles: Vec<&str> = named.iter().map(String::as_str).collect();
        for c in self.blocked.keys() {
            if !cycles.contains(&c.as_str()) {
                cycles.push(c);
            }
        }
        let rank = |c: &str| named.iter().position(|n| n == c).unwrap_or(named.len());
        cycles.sort_by(|a, b| {
            (self.parts_blocked(b).cmp(&self.parts_blocked(a)))
                .then(rank(a).cmp(&rank(b)))
                .then(a.cmp(b))
        });
        out.push_str("| Cycle | Parts blocked |");
        for s in Stage::ALL {
            let _ = write!(out, " {} |", s.name());
        }
        out.push_str("\n|---|---:|");
        out.push_str(&"---:|".repeat(Stage::ALL.len()));
        out.push('\n');
        for c in &cycles {
            let _ = write!(out, "| {c} | {} |", self.parts_blocked(c));
            for s in Stage::ALL {
                let n = (self.blocked.get(*c))
                    .map_or(0, |parts| parts.values().filter(|&&f| f == s).count());
                let _ = write!(out, " {n} |");
            }
            out.push('\n');
        }
        out.push('\n');

        let mut refusals: Vec<(&(String, Stage, String), &usize)> = self.refusals.iter().collect();
        refusals.sort_by(|a, b| {
            b.1.cmp(a.1)
                .then(rank(&a.0.0).cmp(&rank(&b.0.0)))
                .then(a.0.cmp(b.0))
        });
        out.push_str("| Refusal | Stage | Count | Blocks |\n|---|---|---:|---|\n");
        for ((cycle, stage, name), n) in refusals {
            let _ = writeln!(out, "| {name} | {} | {n} | {cycle} |", stage.name());
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_io::step::RefusalKind;

    /// A refusal of each kind, as the reader could return it.
    fn of_kind(kind: RefusalKind) -> Refusal {
        let (entity, what, name) = (1, String::new(), String::new());
        match kind {
            RefusalKind::NoLengthUnit => Refusal::NoLengthUnit { context: entity },
            RefusalKind::Malformed => Refusal::Malformed { entity, what },
            RefusalKind::Offset => Refusal::Offset { entity, name },
            RefusalKind::Composite => Refusal::Composite { entity, name },
            RefusalKind::CurveBounded => Refusal::CurveBounded { entity, name },
            RefusalKind::DegenerateTorus => Refusal::DegenerateTorus { entity },
            RefusalKind::SelfIntersectingTorus => Refusal::SelfIntersectingTorus {
                entity,
                major: 1.0,
                minor: 2.0,
            },
            RefusalKind::Unsupported => Refusal::Unsupported { entity, name },
            RefusalKind::Degenerate => Refusal::Degenerate { entity, what },
            RefusalKind::Topology => Refusal::Topology { entity, what },
            RefusalKind::Pcurve => Refusal::Pcurve {
                edge: entity,
                face: 2,
                what,
            },
            RefusalKind::OpenLoop => Refusal::OpenLoop {
                face: entity,
                bound: 2,
                vertex: 3,
            },
            RefusalKind::Gap => Refusal::Gap {
                entity,
                gap: 1.0,
                cap: 0.5,
            },
            RefusalKind::Invalid => Refusal::Invalid {
                entity,
                report: Box::default(),
            },
        }
    }

    /// Every kind has its row, and the rows ADR-0026 §5 names are the
    /// ones the table gives.
    #[test]
    fn every_refusal_kind_maps_to_its_cycle() {
        let expected = |kind: RefusalKind| match kind {
            RefusalKind::Offset | RefusalKind::Composite => Cycle::Sweep,
            RefusalKind::Topology
            | RefusalKind::OpenLoop
            | RefusalKind::Gap
            | RefusalKind::Invalid => Cycle::Healing,
            RefusalKind::Pcurve => Cycle::Nurbs,
            _ => Cycle::Itself(""),
        };
        for kind in RefusalKind::ALL {
            let refusal = of_kind(kind);
            assert_eq!(refusal.kind(), kind);
            let cycle = blocks_refusal(&refusal);
            match expected(kind) {
                Cycle::Itself(_) => assert!(matches!(cycle, Cycle::Itself(_)), "{kind}: {cycle}"),
                named => assert_eq!(cycle, named, "{kind}"),
            }
        }
        let unsupported = |name: &str| {
            blocks_refusal(&Refusal::Unsupported {
                entity: 1,
                name: name.into(),
            })
        };
        assert_eq!(unsupported("FACE_BASED_SURFACE_MODEL"), Cycle::Healing);
        assert_eq!(unsupported("TESSELLATED_SOLID"), Cycle::Itself("faceted"));
        assert_eq!(
            unsupported("SHELL_BASED_SURFACE_MODEL in a CONSTRUCTIVE_GEOMETRY_REPRESENTATION"),
            Cycle::Itself("supplemental geometry")
        );
        assert_eq!(
            unsupported("VERTEX_LOOP"),
            Cycle::Itself("outside the subset")
        );
    }

    /// The operations' rows: NURBS wherever a NURBS entity is in the
    /// pair, the blend network for another pair at the fillet, and a
    /// fault never counted.
    #[test]
    fn an_unsupported_pair_splits_by_nurbs_and_stage() {
        use arris_io::arris_check::arris_topo::{FaceId, Orientation, Shape};
        let shape = Shape::new(FaceId::new(0, 0), Orientation::Forward);
        let pair = |a: GeomKind, b: GeomKind| OpError::Unsupported {
            a: (a, shape),
            b: (b, shape),
        };
        let plane = GeomKind::Surface(SurfaceKind::Plane);
        let nurbs = GeomKind::Surface(SurfaceKind::Nurbs);
        let circle = GeomKind::Curve(CurveKind::Circle);
        assert_eq!(
            blocks_reason(Stage::BoxCut, &pair(plane, nurbs)),
            Some(Cycle::Nurbs)
        );
        assert_eq!(
            blocks_reason(Stage::Fillet, &pair(circle, plane)),
            Some(Cycle::BlendNetwork)
        );
        assert_eq!(
            blocks_reason(Stage::DrillX, &pair(circle, plane)),
            Some(Cycle::Itself("unsupported pair"))
        );
        let fault = OpError::Internal(arris_ops::Fault::Invariant { what: "" });
        assert_eq!(blocks_reason(Stage::BoxCut, &fault), None);
    }

    /// The histogram prints the same whatever order its parts come in,
    /// and counts a part once per cycle at its first stage.
    #[test]
    fn the_print_is_independent_of_order() {
        let adds: [(&str, Stage, &str, &str); 4] = [
            (
                "real/a",
                Stage::Fillet,
                "Degenerate(TangentChain)",
                "blend network",
            ),
            ("real/b", Stage::Read, "gap past the cap", "healing"),
            (
                "real/a",
                Stage::BoxCut,
                "Unsupported(plane × NURBS)",
                "NURBS",
            ),
            (
                "real/a",
                Stage::DrillX,
                "Unsupported(plane × NURBS)",
                "NURBS",
            ),
        ];
        let mut forward = Histogram::new();
        let mut backward = Histogram::new();
        for a in adds {
            forward.add_refusal(a.0, a.1, a.2, a.3);
        }
        for a in adds.iter().rev() {
            backward.add_refusal(a.0, a.1, a.2, a.3);
        }
        assert_eq!(forward.markdown(), backward.markdown());
        assert_eq!(forward.parts_blocked("NURBS"), 1);
        assert!(
            forward
                .markdown()
                .contains("| NURBS | 1 | 0 | 0 | 0 | 1 | 0 |")
        );
    }

    /// The committed tier counts from its fixtures, every refusal with its
    /// cycle, and prints byte-identically twice.
    #[test]
    fn the_committed_tier_prints_the_same_twice() {
        let count = || {
            let mut h = Histogram::new();
            for name in COMMITTED_TIER {
                let dir = crate::fixtures::corpus_root().join(name);
                let fixture = crate::part::load(&dir).expect("the fixture");
                h.add_part(&fixture).expect("counted");
            }
            h.markdown()
        };
        let first = count();
        assert_eq!(first, count());
        assert!(first.starts_with("11 parts"), "{first}");
    }
}
