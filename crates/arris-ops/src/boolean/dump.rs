//! The text dump of the interference model: [`Interferences`]'s `Display`,
//! one section per kind of entity, in the order the model is built.

use super::*;

/// A number as the dump writes it: the shortest decimal that round-trips.
fn num(x: f64) -> String {
    format!("{x}")
}

fn point3(p: Point3) -> String {
    format!("({}, {}, {})", num(p.x), num(p.y), num(p.z))
}

fn point2(p: Point2) -> String {
    format!("({}, {})", num(p.x), num(p.y))
}

/// A curve in one line: its kind and the numbers that place it.
fn curve(c: &Curve) -> String {
    match c {
        Curve::Line { origin, direction } => format!(
            "line origin {} direction ({}, {}, {})",
            point3(*origin),
            num(direction.x),
            num(direction.y),
            num(direction.z)
        ),
        Curve::Circle { frame, radius } => format!(
            "circle centre {} radius {}",
            point3(frame.origin()),
            num(*radius)
        ),
        Curve::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => format!(
            "ellipse centre {} major {} minor {}",
            point3(frame.origin()),
            num(*major_radius),
            num(*minor_radius)
        ),
        Curve::Nurbs(n) => format!(
            "nurbs degree {} points {}",
            n.degree(),
            n.control_points().len()
        ),
    }
}

/// A pcurve in one line.
fn curve2(c: &Curve2) -> String {
    match c {
        Curve2::Line { origin, direction } => format!(
            "line origin {} direction ({}, {})",
            point2(*origin),
            num(direction.x),
            num(direction.y)
        ),
        Curve2::Circle { frame, radius } => format!(
            "circle centre {} radius {}",
            point2(frame.origin()),
            num(*radius)
        ),
        Curve2::Ellipse {
            frame,
            major_radius,
            minor_radius,
        } => format!(
            "ellipse centre {} major {} minor {}",
            point2(frame.origin()),
            num(*major_radius),
            num(*minor_radius)
        ),
        Curve2::Nurbs(n) => format!(
            "nurbs degree {} points {} u [{}, {}]",
            n.degree(),
            n.control_points().len(),
            num(n
                .control_points()
                .iter()
                .map(|p| p.x)
                .fold(f64::INFINITY, f64::min)),
            num(n
                .control_points()
                .iter()
                .map(|p| p.x)
                .fold(f64::NEG_INFINITY, f64::max))
        ),
    }
}

impl fmt::Display for Interferences {
    /// The whole model, one entity per line: the pairs with their
    /// intersection kind and curves, the hits, the vertices, the paves
    /// per edge, the section curves with their paves, the section edges
    /// with their pcurves, the contacts of the tangent pairs, the
    /// coincident edges, the crossings, images and common blocks of the
    /// coincident pairs. Deterministic, so two runs are compared by their
    /// text.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.write_pairs(f)?;
        self.write_hits(f)?;
        self.write_section_crossings(f)?;
        self.write_triple_points(f)?;
        self.write_vertices(f)?;
        self.write_paves(f)?;
        self.write_curves(f)?;
        self.write_section_edges(f)?;
        self.write_contacts(f)?;
        self.write_coincident(f)?;
        self.write_crossings(f)?;
        self.write_images(f)?;
        self.write_blocks(f)?;
        Ok(())
    }
}

impl Interferences {
    /// The `pairs` section of the dump.
    fn write_pairs(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let names: Vec<String> = self.operands.iter().map(|b| b.to_string()).collect();
        writeln!(f, "interferences {}", names.join(" vs "))?;
        writeln!(f, "pairs {}", self.pairs.len())?;
        for (i, p) in self.pairs.iter().enumerate() {
            match &p.intersection {
                SurfaceIntersection::Empty => writeln!(f, "  p{i} {} x {}: empty", p.a, p.b)?,
                SurfaceIntersection::Coincident => {
                    writeln!(f, "  p{i} {} x {}: coincident", p.a, p.b)?;
                }
                SurfaceIntersection::Meets { curves, points } => {
                    writeln!(f, "  p{i} {} x {}: meets", p.a, p.b)?;
                    let kind = |k: MeetKind| match k {
                        MeetKind::Crossing => "crossing",
                        MeetKind::Touch => "touch",
                    };
                    for c in curves {
                        writeln!(f, "    {} {}", kind(c.kind), curve(&c.curve))?;
                    }
                    for q in points {
                        writeln!(f, "    {} point {}", kind(q.kind), point3(q.point))?;
                    }
                }
            }
        }
        Ok(())
    }

    /// The `hits` section of the dump.
    fn write_hits(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "hits {}", self.hits.len())?;
        for (i, h) in self.hits.iter().enumerate() {
            let landing = match h.landing {
                Landing::Interior => "interior".to_string(),
                Landing::Boundary(s) => format!("boundary {s}"),
            };
            let vertex = match h.vertex {
                Some(v) => format!("v{v}"),
                None => "-".to_string(),
            };
            writeln!(
                f,
                "  h{i} {} t {} on {} uv {} at {} {}{}{} -> {vertex}",
                h.edge,
                num(h.t),
                h.face,
                point2(h.uv),
                point3(h.point),
                landing,
                if h.tangent { " tangent" } else { "" },
                match h.at_vertex {
                    Some(v) => format!(" at {v}"),
                    None => String::new(),
                }
            )?;
        }
        Ok(())
    }

    /// The `section crossings` section of the dump.
    fn write_section_crossings(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "section crossings {}", self.section_crossings.len())?;
        for (i, x) in self.section_crossings.iter().enumerate() {
            let vertex = match x.vertex {
                Some(v) => format!("v{v}"),
                None => "-".to_string(),
            };
            writeln!(
                f,
                "  k{i} p{} curve {} t {} x curve {} t {} at {}{} -> {vertex}",
                x.pair,
                x.curves[0],
                num(x.t[0]),
                x.curves[1],
                num(x.t[1]),
                point3(x.point),
                if x.tangent { " tangent" } else { "" }
            )?;
        }
        Ok(())
    }

    /// The `triple points` section of the dump.
    fn write_triple_points(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "triple points {}", self.triple_points.len())?;
        for (i, x) in self.triple_points.iter().enumerate() {
            let vertex = match x.vertex {
                Some(v) => format!("v{v}"),
                None => "-".to_string(),
            };
            writeln!(
                f,
                "  q{i} p{} curve {} t {} x {} at {} -> {vertex}",
                x.pair,
                x.curve,
                num(x.t),
                x.face,
                point3(x.point),
            )?;
        }
        Ok(())
    }

    /// The `vertices` section of the dump.
    fn write_vertices(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "vertices {}", self.vertices.len())?;
        for (i, v) in self.vertices.iter().enumerate() {
            let hits: Vec<String> = v.hits.iter().map(|h| format!("h{h}")).collect();
            let existing: Vec<String> = v.existing.iter().map(|e| e.to_string()).collect();
            let source = match v.source {
                VertexSource::Hits => String::new(),
                VertexSource::SectionCrossing => {
                    let list: Vec<String> = v
                        .section_crossings
                        .iter()
                        .map(|k| format!("k{k}"))
                        .collect();
                    format!(" section crossing [{}]", list.join(" "))
                }
                VertexSource::CurveStart { pair, curve } => {
                    format!(" start of curve {curve} of p{pair}")
                }
                VertexSource::Singular => " singular".to_string(),
                VertexSource::TriplePoint => {
                    let list: Vec<String> =
                        v.triple_points.iter().map(|k| format!("q{k}")).collect();
                    format!(" triple point [{}]", list.join(" "))
                }
            };
            writeln!(
                f,
                "  v{i} {} tol {} hits [{}] existing [{}]{source}",
                point3(v.point),
                num(v.tolerance),
                hits.join(" "),
                existing.join(" ")
            )?;
        }
        Ok(())
    }

    /// The `paves` section of the dump.
    fn write_paves(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "paves")?;
        for (edge, paves) in &self.paves {
            let list: Vec<String> = paves
                .iter()
                .map(|p| format!("t {} v{}", num(p.t), p.vertex))
                .collect();
            writeln!(f, "  {edge}: {}", list.join(", "))?;
        }
        Ok(())
    }

    /// The `section curves` section of the dump.
    fn write_curves(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "section curves {}", self.curves.len())?;
        for (i, c) in self.curves.iter().enumerate() {
            let paves: Vec<String> = c
                .paves
                .iter()
                .map(|p| format!("t {} v{}", num(p.t), p.vertex))
                .collect();
            let edges: Vec<String> = c.edges.iter().map(|e| format!("s{e}")).collect();
            writeln!(
                f,
                "  c{i} p{} {}: paves [{}] edges [{}]",
                c.pair,
                curve(&c.curve),
                paves.join(", "),
                edges.join(" ")
            )?;
        }
        Ok(())
    }

    /// The `section edges` section of the dump.
    fn write_section_edges(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "section edges {}", self.sections.len())?;
        for (i, s) in self.sections.iter().enumerate() {
            writeln!(
                f,
                "  s{i} c{} [{}, {}] v{} -> v{} tol {}",
                s.curve,
                num(s.range.lo()),
                num(s.range.hi()),
                s.start,
                s.end,
                num(s.tolerance)
            )?;
            writeln!(f, "    on a: {}", curve2(&s.pcurves[0]))?;
            writeln!(f, "    on b: {}", curve2(&s.pcurves[1]))?;
            for u in &s.shared {
                writeln!(f, "    p{} on {}: {}", u.pair, u.face, curve2(&u.pcurve))?;
            }
        }
        Ok(())
    }

    /// The `contacts` section of the dump.
    fn write_contacts(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "contacts {}", self.contacts.len())?;
        for (i, c) in self.contacts.iter().enumerate() {
            writeln!(
                f,
                "  t{i} p{} curve {} [{}, {}] at {} uv {} / {}",
                c.pair,
                c.curve,
                num(c.range.lo()),
                num(c.range.hi()),
                point3(c.point),
                point2(c.uv[0]),
                point2(c.uv[1])
            )?;
        }
        Ok(())
    }

    /// The `coincident` section of the dump.
    fn write_coincident(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "coincident {}", self.coincident.len())?;
        for (e, face) in &self.coincident {
            writeln!(f, "  {e} in {face}")?;
        }
        Ok(())
    }

    /// The `crossings` section of the dump.
    fn write_crossings(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "crossings {}", self.crossings.len())?;
        for (i, x) in self.crossings.iter().enumerate() {
            let vertex = match x.vertex {
                Some(v) => format!("v{v}"),
                None => "-".to_string(),
            };
            writeln!(
                f,
                "  x{i} p{} {} t {} x {} t {} at {}{} -> {vertex}",
                x.pair,
                x.a,
                num(x.ta),
                x.b,
                num(x.tb),
                point3(x.point),
                if x.tangent { " tangent" } else { "" }
            )?;
        }
        Ok(())
    }

    /// The `images` section of the dump.
    fn write_images(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "images {}", self.images.len())?;
        for (i, im) in self.images.iter().enumerate() {
            let pair = &self.pairs[im.pair];
            let on = if im.on == pair.operands[1] {
                pair.b
            } else {
                pair.a
            };
            writeln!(
                f,
                "  i{i} p{} {}#{} [{}, {}] on {} tol {}: {}",
                im.pair,
                im.edge,
                im.index,
                num(im.range.lo()),
                num(im.range.hi()),
                on,
                num(im.tolerance),
                curve2(&im.pcurve)
            )?;
        }
        Ok(())
    }

    /// The `common blocks` section of the dump.
    fn write_blocks(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "common blocks {}", self.blocks.len())?;
        for (i, b) in self.blocks.iter().enumerate() {
            writeln!(
                f,
                "  b{i} p{} {}#{} = {}#{}{} tol {}",
                b.pair,
                b.b.0,
                b.b.1,
                b.a.0,
                b.a.1,
                if b.reversed { " reversed" } else { "" },
                num(b.tolerance)
            )?;
            for (id, pc) in &b.pcurves {
                writeln!(f, "    for {id}: {}", curve2(pc))?;
            }
        }
        Ok(())
    }
}
