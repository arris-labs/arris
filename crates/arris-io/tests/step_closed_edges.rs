//! Two things Autodesk exporters write that the reader once refused or
//! measured wrong, each reduced to a synthetic part in `tests/data/closed-edges`
//! (generated with Open CASCADE, then edited as the exporters write them):
//!
//! - a closed edge on a closed B-spline whose one vertex is not at the
//!   curve's seam (cope curves on pipes). STEP allows it: the edge runs once
//!   round from its vertex. A cylinder of radius 10 and height 20 whose top
//!   circle is an exact rational B-spline starting 37° and 90° from the
//!   edge's vertex. It was refused as a gap of the chord between the two.
//! - a void shell written inside out: its faces already facing into the
//!   void and its `ORIENTED_CLOSED_SHELL` turning them back out. A 10 cube
//!   with a 4 cube void. It read with the void's volume added, not taken.

use arris_check::{Level, check};
use arris_debug::unmetered::{mass_properties, step_read};
use arris_io::step::{Heal, ReadOptions, Refusal, RefusalKind};
use arris_topo::Model;
use core::f64::consts::PI;

const DATA: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/data/closed-edges");

fn text(name: &str) -> String {
    std::fs::read_to_string(format!("{DATA}/{name}")).unwrap()
}

/// The one solid of `name` read with `heal`, or its refusal.
fn read_one(name: &str, heal: Heal) -> (Model, Result<arris_io::step::ReadBody, Refusal>) {
    let mut m = Model::default();
    let options = ReadOptions {
        heal,
        ..ReadOptions::default()
    };
    let read = step_read(&mut m, &text(name), &options).unwrap();
    assert_eq!(read.solids.len(), 1, "{name}");
    let result = read.solids.into_iter().next().unwrap().result;
    (m, result)
}

#[test]
fn a_closed_bspline_edge_whose_vertex_is_off_the_seam_reads_as_the_whole_loop() {
    let exact = PI * 10.0 * 10.0 * 20.0;
    for name in ["closed_bspline_37.step", "closed_bspline_90.step"] {
        for heal in [Heal::Off, Heal::GapsAndOrientation] {
            let (m, result) = read_one(name, heal);
            let body = result.unwrap_or_else(|r| panic!("{name} {heal:?}: refused: {r}")).body;
            assert!(check(&m, body, Level::Full).is_ok(), "{name}");
            assert_eq!(m.faces(body).unwrap().len(), 3, "{name}");
            assert_eq!(m.edges(body).unwrap().len(), 3, "{name}");
            let v = mass_properties(&m, body).unwrap().volume;
            assert!((v - exact).abs() < 1e-12 * exact, "{name}: {v} against {exact}");
        }
    }
}

#[test]
fn a_void_written_inside_out_is_turned_and_named_when_healing() {
    let (m, result) = read_one("void_box_inside_out.step", Heal::GapsAndOrientation);
    let read = result.unwrap();
    assert!(check(&m, read.body, Level::Full).is_ok());
    let v = mass_properties(&m, read.body).unwrap().volume;
    assert!((v - 936.0).abs() < 1e-9, "{v}: the void's 64 taken from 1000");
    assert_eq!(read.healing.turned.len(), 6, "the void's six faces named");
}

#[test]
fn a_void_written_inside_out_is_refused_without_healing() {
    let (_, result) = read_one("void_box_inside_out.step", Heal::Off);
    let refusal = result.unwrap_err();
    assert_eq!(refusal.kind(), RefusalKind::Topology, "{refusal}");
}

#[test]
fn a_void_written_as_the_standard_says_is_left_alone() {
    for heal in [Heal::Off, Heal::GapsAndOrientation] {
        let (m, result) = read_one("void_box.step", heal);
        let read = result.unwrap();
        let v = mass_properties(&m, read.body).unwrap().volume;
        assert!((v - 936.0).abs() < 1e-9, "{v}");
        assert!(read.healing.turned.is_empty());
    }
}
