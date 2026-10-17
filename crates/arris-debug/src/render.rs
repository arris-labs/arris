//! A software rasteriser: a [`TriMesh`] and some [`Polyline`]s to a PNG the
//! agent can read. Orthographic, z-buffered, flat-shaded, one deterministic
//! colour per face id, no anti-aliasing — a picture to read, not to admire.

use std::path::{Path, PathBuf};

use arris_mesh::{Aabb, Polyline, TriMesh};
use arris_topo::{EdgeId, FaceId};

/// Image width in pixels. Fixed so two renders of the same scene are
/// comparable pixel for pixel.
pub const WIDTH: u32 = 800;
/// Image height in pixels.
pub const HEIGHT: u32 = 600;

/// The direction a render looks from. All views are orthographic; the
/// scene is scaled to fit the image with a margin, so absolute size is not
/// readable from a picture — the bounding box in the text dump is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum View {
    /// From `(+1, −1, +1)`: the +x, −y and +z sides are visible; world +x
    /// runs down-right, +y up-right, +z up.
    Iso,
    /// From `+z` down: +x right, +y up.
    Top,
    /// From `−y`: +x right, +z up.
    Front,
    /// From `+x`: +y right, +z up.
    Right,
}

/// One thing drawn in red on top of everything else.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Highlight {
    /// A face's triangles, by the mesh's [`arris_mesh::FaceRange`].
    Face(FaceId),
    /// An edge's polyline, by the mesh's [`arris_mesh::EdgeRange`].
    Edge(EdgeId),
    /// A point, as a large dot (a probe point, a suspicious vertex).
    Point([f64; 3]),
}

/// Why a render could not be written.
#[derive(Debug, thiserror::Error)]
pub enum RenderError {
    /// The output directory could not be created.
    #[error("cannot create {dir}: {source}")]
    CreateDir {
        /// The directory.
        dir: PathBuf,
        /// The cause.
        source: std::io::Error,
    },
    /// The PNG could not be encoded or written.
    #[error("cannot write {path}: {source}")]
    Write {
        /// The file.
        path: PathBuf,
        /// The cause.
        source: image::ImageError,
    },
}

/// An RGB pixel buffer of [`WIDTH`] × [`HEIGHT`], row-major from the top
/// left: what [`render`] returns and [`render_png`] writes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Raster {
    pixels: Vec<[u8; 3]>,
}

impl Raster {
    /// The pixel at column `x`, row `y` (row 0 at the top).
    pub fn pixel(&self, x: u32, y: u32) -> Option<[u8; 3]> {
        if x >= WIDTH || y >= HEIGHT {
            return None;
        }
        self.pixels.get((y * WIDTH + x) as usize).copied()
    }

    /// All pixels, row-major from the top left.
    pub fn pixels(&self) -> &[[u8; 3]] {
        &self.pixels
    }

    /// The distinct colours present, sorted, with their pixel counts.
    pub fn histogram(&self) -> Vec<([u8; 3], usize)> {
        let mut sorted = self.pixels.clone();
        sorted.sort_unstable();
        let mut out: Vec<([u8; 3], usize)> = Vec::new();
        for c in sorted {
            match out.last_mut() {
                Some((last, n)) if *last == c => *n += 1,
                _ => out.push((c, 1)),
            }
        }
        out
    }

    /// Encodes the buffer as a PNG at `path`.
    pub fn save(&self, path: &Path) -> Result<(), RenderError> {
        let mut img = image::RgbImage::new(WIDTH, HEIGHT);
        for (i, p) in img.pixels_mut().enumerate() {
            *p = image::Rgb(self.pixels[i]);
        }
        img.save(path).map_err(|source| RenderError::Write {
            path: path.to_path_buf(),
            source,
        })
    }
}

/// The directory renders go to: `target/inspect/` at the workspace root
/// (gitignored). Created on first write.
pub fn inspect_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/inspect")
}

/// Renders `mesh` and `polylines` from `view`, with `highlight` in red, and
/// writes the PNG. A relative `path` goes under [`inspect_dir`] and gets a
/// `.png` extension if it has none; an absolute one is used as is. Returns
/// the file written, for the agent to read.
///
/// What the picture shows: faces flat-shaded in one colour per face id (grey
/// for triangles outside any face range), the mesh's edge polylines and the
/// given `polylines` in black with hidden parts hidden, a dot at each
/// polyline end, and the highlight in red on top.
///
/// ```no_run
/// use arris_debug::{render_png, View};
/// use arris_mesh::TriMesh;
///
/// let mesh = TriMesh::new();
/// let path = render_png(&mesh, &[], View::Iso, None, "empty-iso").unwrap();
/// assert!(path.ends_with("target/inspect/empty-iso.png") || path.ends_with("empty-iso.png"));
/// ```
pub fn render_png(
    mesh: &TriMesh,
    polylines: &[Polyline],
    view: View,
    highlight: Option<Highlight>,
    path: impl AsRef<Path>,
) -> Result<PathBuf, RenderError> {
    let path = path.as_ref();
    let mut full = if path.is_absolute() {
        path.to_path_buf()
    } else {
        inspect_dir().join(path)
    };
    if full.extension().is_none() {
        full.set_extension("png");
    }
    if let Some(dir) = full.parent() {
        std::fs::create_dir_all(dir).map_err(|source| RenderError::CreateDir {
            dir: dir.to_path_buf(),
            source,
        })?;
    }
    render(mesh, polylines, view, highlight).save(&full)?;
    Ok(full)
}

/// The colours that are not face colours, so a reader of the buffer can
/// tell decoration from geometry.
pub mod colors {
    /// The empty image.
    pub const BACKGROUND: [u8; 3] = [255, 255, 255];
    /// Polylines and mesh edges.
    pub const LINE: [u8; 3] = [0, 0, 0];
    /// Polyline end points.
    pub const DOT: [u8; 3] = [40, 40, 40];
    /// The highlight.
    pub const HIGHLIGHT: [u8; 3] = [230, 30, 30];
    /// Triangles outside every face range, before shading.
    pub const UNRANGED: [u8; 3] = [160, 160, 160];
}

/// Renders to a pixel buffer without writing anything. Same picture as
/// [`render_png`].
pub fn render(
    mesh: &TriMesh,
    polylines: &[Polyline],
    view: View,
    highlight: Option<Highlight>,
) -> Raster {
    let cam = Camera::fit(view, scene_bounds(mesh, polylines, highlight));
    let mut fb = FrameBuffer::new();

    // Faces.
    let mut face_of_triangle: Vec<Option<usize>> = vec![None; mesh.triangles().len()];
    for (fi, fr) in mesh.faces().iter().enumerate() {
        for t in fr.triangles.clone() {
            if let Some(slot) = face_of_triangle.get_mut(t) {
                *slot = Some(fi);
            }
        }
    }
    let light = cam.light();
    for (ti, _) in mesh.triangles().iter().enumerate() {
        let Some([a, b, c]) = mesh.triangle_positions(ti) else {
            continue;
        };
        let n = normalize(cross(sub(b, a), sub(c, a)));
        let shade = 0.35 + 0.65 * dot(n, light).max(0.0);
        // Quantised so triangles of one planar face with normals equal up to
        // rounding get one colour.
        let shade = (shade * 32.0).round() / 32.0;
        let face = face_of_triangle[ti].map(|fi| mesh.faces()[fi].face);
        let base = match (highlight, face) {
            (Some(Highlight::Face(h)), Some(f)) if h == f => colors::HIGHLIGHT,
            (_, Some(f)) => face_color(f),
            (_, None) => colors::UNRANGED,
        };
        let color = base.map(|ch| (f64::from(ch) * shade).round().clamp(0.0, 255.0) as u8);
        fb.triangle([cam.project(a), cam.project(b), cam.project(c)], color);
    }

    // Edges and polylines, with a depth bias toward the viewer so lines on a
    // face's surface win the z-test against it.
    let bias = cam.depth_bias();
    let positions = mesh.positions();
    for er in mesh.edges() {
        let Some(idx) = mesh.edge_polyline(er.edge) else {
            continue;
        };
        let pts: Vec<[f64; 3]> = idx.iter().map(|&i| positions[i as usize]).collect();
        let (color, width) = match highlight {
            Some(Highlight::Edge(e)) if e == er.edge => (colors::HIGHLIGHT, 3),
            _ => (colors::LINE, 2),
        };
        fb.polyline(&cam, &pts, bias, color, width);
        fb.end_dots(&cam, &pts, bias, colors::DOT, 2);
    }
    for pl in polylines {
        fb.polyline(&cam, &pl.points, bias, colors::LINE, 2);
        fb.end_dots(&cam, &pl.points, bias, colors::DOT, 2);
    }
    if let Some(Highlight::Point(p)) = highlight {
        let s = cam.project(p);
        fb.dot(s, f64::INFINITY, colors::HIGHLIGHT, 5);
    }

    Raster { pixels: fb.color }
}

/// The supersampling factor of [`render_labelled`]: each pixel is the mean
/// of this many samples along each axis.
const SAMPLES: u32 = 2;

/// Why [`render_labelled`] was given labels it cannot draw.
#[derive(Debug, thiserror::Error)]
pub enum LabelError {
    /// `labels` is not one entry per triangle.
    #[error("{labels} labels for {triangles} triangles")]
    Count {
        /// The labels given.
        labels: usize,
        /// The mesh's triangles.
        triangles: usize,
    },
    /// A label names no palette entry.
    #[error("label {label} on triangle {triangle}, but the palette has {palette} colours")]
    Palette {
        /// The label.
        label: usize,
        /// The first triangle carrying it.
        triangle: usize,
        /// The palette's length.
        palette: usize,
    },
}

/// Renders `mesh` with each triangle in the `palette` colour of its label
/// (`labels[t]` for triangle `t`), Lambert-shaded and supersampled
/// 2 × 2 times, so a reader sees form and the operation that made a
/// face at once. The mesh's edges are drawn thin in black. Unlike
/// [`render`] it colours by what the caller says, not by face id, and does
/// not quantise the shading. Deterministic: two calls give equal buffers.
///
/// ```
/// use arris_debug::{View, render_labelled};
/// use arris_mesh::TriMesh;
///
/// let raster = render_labelled(&TriMesh::new(), &[], &[], View::Iso).unwrap();
/// assert_eq!(raster.histogram().len(), 1, "only the background");
/// ```
pub fn render_labelled(
    mesh: &TriMesh,
    labels: &[usize],
    palette: &[[u8; 3]],
    view: View,
) -> Result<Raster, LabelError> {
    let triangles = mesh.triangles().len();
    if labels.len() != triangles {
        return Err(LabelError::Count {
            labels: labels.len(),
            triangles,
        });
    }
    if let Some((triangle, &label)) = labels
        .iter()
        .enumerate()
        .find(|(_, label)| **label >= palette.len())
    {
        return Err(LabelError::Palette {
            label,
            triangle,
            palette: palette.len(),
        });
    }
    let mut cam = Camera::fit(view, mesh.aabb());
    cam.samples = f64::from(SAMPLES);
    let mut fb = FrameBuffer::sized(WIDTH * SAMPLES, HEIGHT * SAMPLES);
    let light = cam.light();
    for (ti, &label) in labels.iter().enumerate() {
        let Some([a, b, c]) = mesh.triangle_positions(ti) else {
            continue;
        };
        let n = normalize(cross(sub(b, a), sub(c, a)));
        let shade = 0.3 + 0.7 * dot(n, light).max(0.0);
        let color =
            palette[label].map(|ch| (f64::from(ch) * shade).round().clamp(0.0, 255.0) as u8);
        fb.triangle([cam.project(a), cam.project(b), cam.project(c)], color);
    }
    let bias = cam.depth_bias();
    let positions = mesh.positions();
    for er in mesh.edges() {
        let Some(idx) = mesh.edge_polyline(er.edge) else {
            continue;
        };
        let pts: Vec<[f64; 3]> = idx.iter().map(|&i| positions[i as usize]).collect();
        fb.polyline(&cam, &pts, bias, colors::LINE, i64::from(SAMPLES));
    }
    Ok(fb.downsample())
}

/// A deterministic, well-separated colour for a face id: hue steps by the
/// golden ratio with the slot index, so neighbouring ids differ clearly.
pub fn face_color(face: FaceId) -> [u8; 3] {
    let i = f64::from(face.index()) + 0.5 * f64::from(face.generation());
    let hue = (i * 0.618_033_988_749_895).fract();
    hsv_to_rgb(hue, 0.55, 0.9)
}

fn hsv_to_rgb(h: f64, s: f64, v: f64) -> [u8; 3] {
    let h6 = h * 6.0;
    let i = h6.floor() as i32;
    let f = h6 - h6.floor();
    let p = v * (1.0 - s);
    let q = v * (1.0 - s * f);
    let t = v * (1.0 - s * (1.0 - f));
    let (r, g, b) = match i.rem_euclid(6) {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    };
    [
        (r * 255.0).round() as u8,
        (g * 255.0).round() as u8,
        (b * 255.0).round() as u8,
    ]
}

fn scene_bounds(
    mesh: &TriMesh,
    polylines: &[Polyline],
    highlight: Option<Highlight>,
) -> Option<Aabb> {
    let mut b = mesh.aabb();
    for pl in polylines {
        b = match (b, pl.aabb()) {
            (Some(x), Some(y)) => Some(x.union(y)),
            (x, y) => x.or(y),
        };
    }
    if let Some(Highlight::Point(p)) = highlight {
        let pb = Aabb { min: p, max: p };
        b = Some(b.map_or(pb, |x| x.union(pb)));
    }
    b
}

/// Screen-space point: pixel x (right), pixel y (down), depth (larger is
/// nearer the viewer).
#[derive(Debug, Clone, Copy)]
struct Screen {
    x: f64,
    y: f64,
    depth: f64,
}

struct Camera {
    right: [f64; 3],
    up: [f64; 3],
    toward_viewer: [f64; 3],
    scale: f64,
    center: [f64; 3],
    diagonal: f64,
    /// Pixels per output pixel along each axis: 1, or 2 when supersampling.
    samples: f64,
}

impl Camera {
    fn basis(view: View) -> ([f64; 3], [f64; 3], [f64; 3]) {
        match view {
            View::Top => ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
            View::Front => ([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]),
            View::Right => ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
            View::Iso => {
                let e = normalize([1.0, -1.0, 1.0]);
                let right = normalize([1.0, 1.0, 0.0]);
                let up = cross(e, right);
                (right, up, e)
            }
        }
    }

    /// A camera that fits `bounds` into the image with a 10 % margin,
    /// preserving aspect; an empty scene gets unit scale about the origin.
    fn fit(view: View, bounds: Option<Aabb>) -> Camera {
        let (right, up, toward_viewer) = Self::basis(view);
        let Some(b) = bounds else {
            return Camera {
                right,
                up,
                toward_viewer,
                scale: 1.0,
                center: [0.0; 3],
                diagonal: 1.0,
                samples: 1.0,
            };
        };
        // Project the eight corners to find the screen extent.
        let mut xs = (f64::INFINITY, f64::NEG_INFINITY);
        let mut ys = (f64::INFINITY, f64::NEG_INFINITY);
        for corner in 0..8 {
            let p = [
                if corner & 1 == 0 { b.min[0] } else { b.max[0] },
                if corner & 2 == 0 { b.min[1] } else { b.max[1] },
                if corner & 4 == 0 { b.min[2] } else { b.max[2] },
            ];
            let x = dot(p, right);
            let y = dot(p, up);
            xs = (xs.0.min(x), xs.1.max(x));
            ys = (ys.0.min(y), ys.1.max(y));
        }
        let w = (xs.1 - xs.0).max(f64::MIN_POSITIVE);
        let h = (ys.1 - ys.0).max(f64::MIN_POSITIVE);
        let scale = (f64::from(WIDTH) * 0.8 / w).min(f64::from(HEIGHT) * 0.8 / h);
        let scale = if scale.is_finite() && scale > 0.0 {
            scale
        } else {
            1.0
        };
        Camera {
            right,
            up,
            toward_viewer,
            scale,
            center: b.center(),
            diagonal: b.diagonal().max(1e-300),
            samples: 1.0,
        }
    }

    fn project(&self, p: [f64; 3]) -> Screen {
        let d = sub(p, self.center);
        Screen {
            x: self.samples * (f64::from(WIDTH) / 2.0 + self.scale * dot(d, self.right)),
            y: self.samples * (f64::from(HEIGHT) / 2.0 - self.scale * dot(d, self.up)),
            depth: dot(d, self.toward_viewer),
        }
    }

    /// Light from over the viewer's right shoulder, so the three faces of
    /// an iso view shade differently even when they share a colour.
    fn light(&self) -> [f64; 3] {
        normalize(add(
            add(self.toward_viewer, scale(self.up, 0.6)),
            scale(self.right, 0.3),
        ))
    }

    fn depth_bias(&self) -> f64 {
        self.diagonal * 1e-3
    }
}

struct FrameBuffer {
    color: Vec<[u8; 3]>,
    depth: Vec<f64>,
    width: u32,
    height: u32,
}

impl FrameBuffer {
    fn new() -> Self {
        Self::sized(WIDTH, HEIGHT)
    }

    fn sized(width: u32, height: u32) -> Self {
        let n = (width * height) as usize;
        FrameBuffer {
            color: vec![colors::BACKGROUND; n],
            depth: vec![f64::NEG_INFINITY; n],
            width,
            height,
        }
    }

    /// The mean of each `SAMPLES`² block, as a [`WIDTH`] × [`HEIGHT`] raster.
    fn downsample(&self) -> Raster {
        let k = SAMPLES as usize;
        let mut pixels = Vec::with_capacity((WIDTH * HEIGHT) as usize);
        for y in 0..HEIGHT as usize {
            for x in 0..WIDTH as usize {
                let mut sum = [0usize; 3];
                for dy in 0..k {
                    for dx in 0..k {
                        let c = self.color[(y * k + dy) * self.width as usize + x * k + dx];
                        for (s, ch) in sum.iter_mut().zip(c) {
                            *s += usize::from(ch);
                        }
                    }
                }
                pixels.push(sum.map(|s| ((s + k * k / 2) / (k * k)) as u8));
            }
        }
        Raster { pixels }
    }

    fn plot(&mut self, x: i64, y: i64, depth: f64, color: [u8; 3]) {
        if x < 0 || y < 0 || x >= i64::from(self.width) || y >= i64::from(self.height) {
            return;
        }
        let i = (y as usize) * self.width as usize + x as usize;
        if depth >= self.depth[i] {
            self.depth[i] = depth;
            self.color[i] = color;
        }
    }

    fn triangle(&mut self, s: [Screen; 3], color: [u8; 3]) {
        let edge = |a: Screen, b: Screen, px: f64, py: f64| {
            (b.x - a.x) * (py - a.y) - (b.y - a.y) * (px - a.x)
        };
        let area = edge(s[0], s[1], s[2].x, s[2].y);
        if area.abs() < 1e-12 || !area.is_finite() {
            return;
        }
        let x0 = s
            .iter()
            .map(|p| p.x)
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.0) as i64;
        let x1 = s
            .iter()
            .map(|p| p.x)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil()
            .min(f64::from(self.width) - 1.0) as i64;
        let y0 = s
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min)
            .floor()
            .max(0.0) as i64;
        let y1 = s
            .iter()
            .map(|p| p.y)
            .fold(f64::NEG_INFINITY, f64::max)
            .ceil()
            .min(f64::from(self.height) - 1.0) as i64;
        for y in y0..=y1 {
            for x in x0..=x1 {
                let (px, py) = (x as f64 + 0.5, y as f64 + 0.5);
                let w0 = edge(s[1], s[2], px, py) / area;
                let w1 = edge(s[2], s[0], px, py) / area;
                let w2 = edge(s[0], s[1], px, py) / area;
                if w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0 {
                    let depth = w0 * s[0].depth + w1 * s[1].depth + w2 * s[2].depth;
                    self.plot(x, y, depth, color);
                }
            }
        }
    }

    fn line(&mut self, a: Screen, b: Screen, color: [u8; 3], width: i64) {
        let steps = (b.x - a.x).abs().max((b.y - a.y).abs()).ceil().max(1.0);
        if !steps.is_finite() {
            return;
        }
        let n = steps as i64;
        for i in 0..=n {
            let t = i as f64 / steps;
            let x = a.x + t * (b.x - a.x);
            let y = a.y + t * (b.y - a.y);
            let depth = a.depth + t * (b.depth - a.depth);
            for dy in 0..width {
                for dx in 0..width {
                    self.plot(
                        x.floor() as i64 + dx - width / 2,
                        y.floor() as i64 + dy - width / 2,
                        depth,
                        color,
                    );
                }
            }
        }
    }

    fn polyline(&mut self, cam: &Camera, pts: &[[f64; 3]], bias: f64, color: [u8; 3], width: i64) {
        for w in pts.windows(2) {
            let mut a = cam.project(w[0]);
            let mut b = cam.project(w[1]);
            a.depth += bias;
            b.depth += bias;
            self.line(a, b, color, width);
        }
    }

    fn dot(&mut self, s: Screen, depth: f64, color: [u8; 3], radius: i64) {
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx * dx + dy * dy <= radius * radius {
                    self.plot(
                        s.x.floor() as i64 + dx,
                        s.y.floor() as i64 + dy,
                        depth,
                        color,
                    );
                }
            }
        }
    }

    fn end_dots(&mut self, cam: &Camera, pts: &[[f64; 3]], bias: f64, color: [u8; 3], radius: i64) {
        for p in [pts.first(), pts.last()].into_iter().flatten() {
            let s = cam.project(*p);
            self.dot(s, s.depth + 2.0 * bias, color, radius);
        }
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(a: [f64; 3]) -> [f64; 3] {
    let n = dot(a, a).sqrt();
    if n > 0.0 && n.is_finite() {
        scale(a, 1.0 / n)
    } else {
        [0.0; 3]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use arris_mesh::TriMesh;

    /// The cube [-1, 1]³: 8 positions, 12 outward triangles, one face range
    /// per side (-z, +z, -y, +y, -x, +x) and its 12 edges as polylines.
    fn cube() -> TriMesh {
        let mut m = TriMesh::new();
        for z in [-1.0, 1.0] {
            for y in [-1.0, 1.0] {
                for x in [-1.0, 1.0] {
                    m.push_position([x, y, z]).unwrap();
                }
            }
        }
        let quads: [[u32; 4]; 6] = [
            [0, 2, 3, 1],
            [4, 5, 7, 6],
            [0, 1, 5, 4],
            [2, 6, 7, 3],
            [0, 4, 6, 2],
            [1, 3, 7, 5],
        ];
        for (i, q) in quads.iter().enumerate() {
            m.push_face(
                FaceId::new(i as u32, 0),
                [[q[0], q[1], q[2]], [q[0], q[2], q[3]]],
            )
            .unwrap();
        }
        let edges: [[u32; 2]; 12] = [
            [0, 1],
            [2, 3],
            [4, 5],
            [6, 7],
            [0, 2],
            [1, 3],
            [4, 6],
            [5, 7],
            [0, 4],
            [1, 5],
            [2, 6],
            [3, 7],
        ];
        for (i, e) in edges.iter().enumerate() {
            m.push_edge(EdgeId::new(i as u32, 0), e).unwrap();
        }
        m
    }

    /// Distinct colours that are neither background nor decoration.
    fn face_colors(r: &Raster) -> Vec<[u8; 3]> {
        r.histogram()
            .into_iter()
            .map(|(c, _)| c)
            .filter(|c| ![colors::BACKGROUND, colors::LINE, colors::DOT].contains(c))
            .collect()
    }

    #[test]
    fn iso_shows_three_faces_and_top_shows_one() {
        let m = cube();
        let iso = render(&m, &[], View::Iso, None);
        assert_eq!(face_colors(&iso).len(), 3, "{:?}", face_colors(&iso));
        let top = render(&m, &[], View::Top, None);
        assert_eq!(face_colors(&top).len(), 1);
        let front = render(&m, &[], View::Front, None);
        assert_eq!(face_colors(&front).len(), 1);
        let right = render(&m, &[], View::Right, None);
        assert_eq!(face_colors(&right).len(), 1);
        assert_ne!(
            face_colors(&top),
            face_colors(&front),
            "different faces, different colours"
        );
    }

    #[test]
    fn hidden_edges_stay_hidden_and_visible_ones_are_drawn() {
        let m = cube();
        let top = render(&m, &[], View::Top, None);
        let black = top.pixels().iter().filter(|&&c| c == colors::LINE).count();
        assert!(black > 0, "the top face's four edges are drawn");
        // From above, the top face covers everything: no pixel inside the
        // square is background.
        assert_eq!(top.pixel(WIDTH / 2, HEIGHT / 2), Some(face_colors(&top)[0]));
    }

    #[test]
    fn highlight_paints_only_its_target_red() {
        let m = cube();
        let plain = render(&m, &[], View::Iso, None);
        let hi = render(&m, &[], View::Iso, Some(Highlight::Face(FaceId::new(1, 0))));
        let red = |r: &Raster| {
            r.pixels()
                .iter()
                .filter(|c| c[0] > 150 && c[1] < 60 && c[2] < 60)
                .count()
        };
        assert_eq!(red(&plain), 0);
        assert!(red(&hi) > 1000, "the +z face is a large red area");
        assert_eq!(
            face_colors(&hi).len(),
            3,
            "still three faces, one of them red"
        );
        let hidden = render(&m, &[], View::Top, Some(Highlight::Face(FaceId::new(0, 0))));
        assert_eq!(
            red(&hidden),
            0,
            "the -z face is hidden from the top and stays hidden"
        );
        let point = render(&m, &[], View::Top, Some(Highlight::Point([0.0, 0.0, 5.0])));
        assert!(
            red(&point) > 0 && red(&point) < 200,
            "a point is a small dot"
        );
    }

    #[test]
    fn writes_the_png_the_agent_reads() {
        let m = cube();
        let path = render_png(&m, &[], View::Iso, None, "cube-iso").unwrap();
        assert!(path.exists());
        assert_eq!(path.extension().and_then(|e| e.to_str()), Some("png"));
        let back = image::open(&path).unwrap().to_rgb8();
        assert_eq!((back.width(), back.height()), (WIDTH, HEIGHT));
        let raster = render(&m, &[], View::Iso, None);
        assert_eq!(
            back.pixels().map(|p| p.0).collect::<Vec<_>>(),
            raster.pixels()
        );
        render_png(
            &m,
            &[],
            View::Top,
            Some(Highlight::Edge(EdgeId::new(3, 0))),
            "cube-top",
        )
        .unwrap();
    }

    #[test]
    fn empty_scene_is_a_blank_image() {
        let r = render(&TriMesh::new(), &[], View::Iso, None);
        assert_eq!(
            r.histogram(),
            vec![(colors::BACKGROUND, (WIDTH * HEIGHT) as usize)]
        );
    }

    #[test]
    fn face_colors_are_deterministic_and_distinct_for_small_ids() {
        let cs: Vec<_> = (0..12).map(|i| face_color(FaceId::new(i, 0))).collect();
        let mut sorted = cs.clone();
        sorted.sort();
        sorted.dedup();
        assert_eq!(sorted.len(), cs.len());
        assert_eq!(face_color(FaceId::new(3, 0)), face_color(FaceId::new(3, 0)));
    }
}
