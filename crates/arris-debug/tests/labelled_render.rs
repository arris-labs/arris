//! `render_labelled` (ADR-0067): a body's faces
//! coloured by a label the caller chose, shaded and supersampled. Each
//! label's colour is present in the pixels, the buffer is the same on two
//! runs, and a badly sized label list is refused by name.

use arris_debug::{LabelError, Raster, View, body, render_labelled, sample};
use arris_topo::Model;

const PALETTE: [[u8; 3]; 3] = [[200, 40, 40], [40, 200, 40], [40, 40, 200]];

/// A box's triangles labelled by the axis their normal runs along, so the
/// three faces an iso view shows carry three labels.
fn labelled_box() -> (arris_mesh::TriMesh, Vec<usize>) {
    let mut m = Model::default();
    let b = sample::unit_box(&mut m).unwrap();
    let mesh = body::mesh_of(&m, b).unwrap();
    let labels = (0..mesh.triangles().len())
        .map(|t| {
            let [a, b, c] = mesh.triangle_positions(t).unwrap();
            let (u, v) = (
                [b[0] - a[0], b[1] - a[1], b[2] - a[2]],
                [c[0] - a[0], c[1] - a[1], c[2] - a[2]],
            );
            let n = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            (0..3)
                .max_by(|&i, &j| n[i].abs().total_cmp(&n[j].abs()))
                .unwrap()
        })
        .collect();
    (mesh, labels)
}

/// Pixels bright enough to carry a hue, whose colour points the way of
/// `base` (shading scales a colour, it does not turn it).
fn pixels_of(raster: &Raster, base: [u8; 3]) -> usize {
    let unit = |c: [f64; 3]| {
        let n = c.iter().map(|x| x * x).sum::<f64>().sqrt();
        c.map(|x| x / n)
    };
    let want = unit(base.map(f64::from));
    raster
        .pixels()
        .iter()
        .filter(|p| p.iter().map(|&c| u32::from(c)).sum::<u32>() > 90)
        .filter(|p| {
            let got = unit(p.map(f64::from));
            got.iter().zip(want).all(|(g, w)| (g - w).abs() < 0.03)
        })
        .count()
}

#[test]
fn each_labels_colour_is_in_the_pixels() {
    let (mesh, labels) = labelled_box();
    let raster = render_labelled(&mesh, &labels, &PALETTE, View::Iso).unwrap();
    for colour in PALETTE {
        assert!(pixels_of(&raster, colour) > 2000, "{colour:?}");
    }
}

#[test]
fn the_shading_gives_one_label_more_than_one_tone() {
    let (mesh, labels) = labelled_box();
    let raster = render_labelled(&mesh, &labels, &PALETTE, View::Iso).unwrap();
    let tones: std::collections::BTreeSet<[u8; 3]> = raster
        .histogram()
        .into_iter()
        .map(|(c, _)| c)
        .filter(|&c| c != arris_debug::render::colors::BACKGROUND)
        .collect();
    assert!(tones.len() >= 3, "{tones:?}");
}

#[test]
fn two_runs_give_one_buffer() {
    let (mesh, labels) = labelled_box();
    let a = render_labelled(&mesh, &labels, &PALETTE, View::Iso).unwrap();
    let b = render_labelled(&mesh, &labels, &PALETTE, View::Iso).unwrap();
    assert_eq!(a, b);
}

#[test]
fn supersampling_softens_an_edge() {
    let (mesh, labels) = labelled_box();
    let raster = render_labelled(&mesh, &labels, &PALETTE, View::Iso).unwrap();
    // A silhouette pixel is a mean of colour and background: lighter than
    // any shaded interior colour of its label would be, and not the pure
    // background either.
    let blended = raster
        .histogram()
        .into_iter()
        .filter(|&(c, _)| c != arris_debug::render::colors::BACKGROUND)
        .count();
    let flat = {
        let mut m = Model::default();
        let b = sample::unit_box(&mut m).unwrap();
        let mesh = body::mesh_of(&m, b).unwrap();
        arris_debug::render(&mesh, &[], View::Iso, None)
            .histogram()
            .len()
    };
    assert!(
        blended > flat,
        "{blended} tones against the flat render's {flat}"
    );
}

#[test]
fn labels_that_do_not_fit_are_refused_by_name() {
    let (mesh, labels) = labelled_box();
    let short = render_labelled(&mesh, &labels[1..], &PALETTE, View::Iso);
    assert!(matches!(short, Err(LabelError::Count { .. })));
    let past = render_labelled(&mesh, &labels, &PALETTE[..2], View::Iso);
    assert!(matches!(past, Err(LabelError::Palette { palette: 2, .. })));
}
