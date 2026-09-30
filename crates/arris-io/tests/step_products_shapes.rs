//! The product tree on the shapes real files have that Open CASCADE's own
//! assembly does not (ADR-0033): a lone part, two roots, a `MAPPED_ITEM`
//! assembly, a placement Arris refuses, an empty name, a cycle of usages.
//! Each file is Part 21 text built here; its solids are dangling
//! `MANIFOLD_SOLID_BREP`s, which the reader refuses as a solid and keeps
//! as one, because the tree is about structure.

use arris_debug::unmetered::step_read;
use arris_io::arris_check::arris_topo::Model;
use arris_io::step::{Occurrence, Read, ReadOptions, RefusalKind};

/// A file under construction: the units and the world axis are `#1`–`#9`.
struct File {
    lines: Vec<String>,
    next: u64,
}

impl File {
    fn new() -> Self {
        let lines = [
            "#1=( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );",
            "#2=( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );",
            "#3=( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );",
            "#4=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'d','c');",
            "#5=( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#4)) GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3)) REPRESENTATION_CONTEXT('c','3D') );",
            "#6=CARTESIAN_POINT('',(0.,0.,0.));",
            "#7=AXIS2_PLACEMENT_3D('',#6,$,$);",
            "#8=APPLICATION_CONTEXT('c');",
            "#9=PRODUCT_CONTEXT('',#8,'mechanical');",
            "#10=PRODUCT_DEFINITION_CONTEXT('part definition',#8,'design');",
        ];
        File {
            lines: lines.iter().map(|l| l.to_string()).collect(),
            next: 11,
        }
    }

    fn push(&mut self, text: String) -> u64 {
        let id = self.next;
        self.lines.push(format!("#{id}={text};"));
        self.next += 1;
        id
    }

    /// A translation, as an `AXIS2_PLACEMENT_3D` at `(x, 0, 0)`.
    fn at(&mut self, x: f64) -> u64 {
        let p = self.push(format!("CARTESIAN_POINT('',({x:?},0.,0.))"));
        self.push(format!("AXIS2_PLACEMENT_3D('',#{p},$,$)"))
    }

    /// A product named `name` (id `id`) with a shape representation of a
    /// solid: its definition and its representation.
    fn part(&mut self, id: &str, name: &str) -> (u64, u64) {
        let product = self.push(format!("PRODUCT('{id}','{name}','',(#9))"));
        let formation = self.push(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
        let definition = self.push(format!("PRODUCT_DEFINITION('design','',#{formation},#10)"));
        let shape = self.push(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
        let solid = self.push("MANIFOLD_SOLID_BREP('',#99999)".to_string());
        let rep = self.push(format!("SHAPE_REPRESENTATION('',(#7,#{solid}),#5)"));
        self.push(format!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));
        (definition, rep)
    }

    /// An assembly: a product whose representation holds only `items`.
    fn assembly(&mut self, name: &str, items: &[u64]) -> (u64, u64) {
        let (definition, rep) = self.part(name, name);
        if !items.is_empty() {
            let list: Vec<String> = items.iter().map(|i| format!("#{i}")).collect();
            let at = self
                .lines
                .iter()
                .position(|l| l.starts_with(&format!("#{rep}=")))
                .unwrap();
            self.lines[at] = format!(
                "#{rep}=SHAPE_REPRESENTATION('',(#7,{}),#5);",
                list.join(",")
            );
        }
        (definition, rep)
    }

    /// `child` placed in `parent` by `transformation`, a written
    /// `ITEM_DEFINED_TRANSFORMATION`'s or other entity's id.
    fn usage(&mut self, parent: (u64, u64), child: (u64, u64), transformation: &str) {
        let nauo = self.push(format!(
            "NEXT_ASSEMBLY_USAGE_OCCURRENCE('u','','',#{},#{},$)",
            parent.0, child.0
        ));
        let shape = self.push(format!("PRODUCT_DEFINITION_SHAPE('','',#{nauo})"));
        let relation = self.push(format!(
            "( REPRESENTATION_RELATIONSHIP('','',#{},#{}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION({transformation}) SHAPE_REPRESENTATION_RELATIONSHIP() )",
            child.1, parent.1
        ));
        self.push(format!(
            "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{relation},#{shape})"
        ));
    }

    /// A rigid motion taking a child's world axis to `(x, 0, 0)`.
    fn shift(&mut self, x: f64) -> String {
        let to = self.at(x);
        format!(
            "#{}",
            self.push(format!("ITEM_DEFINED_TRANSFORMATION('','',#7,#{to})"))
        )
    }

    fn text(&self) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION((''),'2;1');\nFILE_NAME('','',(''),(''),'','','');\nFILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n{}\nENDSEC;\nEND-ISO-10303-21;\n",
            self.lines.join("\n")
        )
    }

    fn read(&self) -> Read {
        let mut m = Model::default();
        step_read(&mut m, &self.text(), &ReadOptions::default()).unwrap()
    }
}

fn names(o: &Occurrence) -> String {
    if o.children.is_empty() {
        o.name.clone()
    } else {
        let inner: Vec<String> = o.children.iter().map(names).collect();
        format!("{}({})", o.name, inner.join(","))
    }
}

fn shoved(o: &Occurrence) -> f64 {
    o.placement.as_ref().unwrap().translation().x
}

/// A part no usage places is one root holding its solid.
#[test]
fn a_lone_part_is_one_root() {
    let mut f = File::new();
    f.part("p1", "lone");
    let read = f.read();
    assert_eq!(read.products.roots.len(), 1);
    let root = &read.products.roots[0];
    assert_eq!((root.name.as_str(), root.solids.clone()), ("lone", vec![0]));
    assert!(root.children.is_empty());
}

/// Two products, neither placed in the other, are two roots, ascending by
/// the file's representation.
#[test]
fn two_roots_are_two_trees() {
    let mut f = File::new();
    f.part("a", "first");
    let second = f.part("b", "second");
    let third = f.part("c", "third");
    let t = f.shift(3.0);
    f.usage(second, third, &t);
    let read = f.read();
    let roots: Vec<String> = read.products.roots.iter().map(names).collect();
    assert_eq!(roots, ["first", "second(third)"]);
    assert_eq!(read.solids.len(), 3);
    assert_eq!(read.products.roots[1].children[0].solids, vec![2]);
    assert_eq!(shoved(&read.products.roots[1].children[0]), 3.0);
}

/// An assembly by `MAPPED_ITEM`: no usage occurrence, the mapped item
/// places the part at its target.
#[test]
fn a_mapped_item_assembly_has_its_child() {
    let mut f = File::new();
    let part = f.part("p", "part");
    let target = f.at(5.0);
    let map = f.push(format!("REPRESENTATION_MAP(#7,#{})", part.1));
    let item = f.push(format!("MAPPED_ITEM('',#{map},#{target})"));
    f.assembly("asm", &[item]);
    let read = f.read();
    let roots: Vec<String> = read.products.roots.iter().map(names).collect();
    // The part is a root of its own as well: nothing *usage* places it,
    // and the flattening agrees, numbering its solid twice.
    assert_eq!(roots.last().unwrap(), "asm(part)", "{roots:?}");
    let asm = read.products.roots.last().unwrap();
    assert_eq!(shoved(&asm.children[0]), 5.0);
    assert_eq!(asm.children[0].solids.len(), 1);
}

/// A placement by an operator Arris does not read keeps its occurrence,
/// with the refusal for a placement and the solid refused as before.
#[test]
fn a_refused_placement_is_kept_with_its_refusal() {
    let mut f = File::new();
    let part = f.part("p", "part");
    let asm = f.assembly("asm", &[]);
    let op = f.push("CARTESIAN_TRANSFORMATION_OPERATOR_3D('','',$,$,$,$,$)".to_string());
    f.usage(asm, part, &format!("#{op}"));
    let read = f.read();
    let root = read.products.roots.last().unwrap();
    assert_eq!(names(root), "asm(part)");
    let child = &root.children[0];
    let refusal = child.placement.as_ref().unwrap_err();
    assert_eq!(refusal.kind(), RefusalKind::Unsupported);
    let solid = &read.solids[child.solids[0]];
    assert!(solid.result.is_err());
}

/// An empty product name is the id; both empty, empty.
#[test]
fn an_empty_name_is_the_id() {
    let mut f = File::new();
    f.part("the-id", "");
    f.part("", "");
    let read = f.read();
    let roots: Vec<&str> = read
        .products
        .roots
        .iter()
        .map(|r| r.name.as_str())
        .collect();
    assert_eq!(roots, ["the-id", ""]);
}

/// A cycle of usages is cut where the flattening cuts it: a root that
/// holds `a`, which holds `b`, which holds `a` again, is `r(a(b))`.
#[test]
fn a_cycle_of_usages_is_not_followed_round() {
    let mut f = File::new();
    let r = f.assembly("r", &[]);
    let a = f.assembly("a", &[]);
    let b = f.assembly("b", &[]);
    let t = f.shift(1.0);
    f.usage(r, a, &t);
    let t = f.shift(2.0);
    f.usage(a, b, &t);
    let t = f.shift(3.0);
    f.usage(b, a, &t);
    let read = f.read();
    let roots: Vec<String> = read.products.roots.iter().map(names).collect();
    assert_eq!(roots, ["r(a(b))"]);
}
