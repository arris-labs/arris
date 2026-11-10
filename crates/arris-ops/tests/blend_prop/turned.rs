use super::support::*;

// Turned parts: the meridian row (ADR-0036) in one call. A profile of lines
// and an arc revolved about `y`, some of its corners blended; the volume is
// the part's by Pappus less (a fillet or chamfer of a convex corner) or
// plus (a concave one) each corner's section swept about the axis — a
// region this test bounds itself, from the two meridians and the ball, not
// from the kernel's contacts.

/// A meridian leaving a corner `v`: its point at arc length `s` from it.
#[derive(Debug, Clone, Copy)]
enum Ray {
    Line {
        v: Point2,
        out: Vec2,
        /// The left of the outline's direction of travel, the material's
        /// side.
        left: Vec2,
    },
    Circle {
        centre: Point2,
        radius: f64,
        /// The angle of the corner, and the way the angle runs leaving it.
        from: f64,
        way: f64,
        /// Whether the outline travels the circle counter-clockwise.
        ccw: bool,
    },
}

impl Ray {
    /// Piece `k` of the outline as it leaves corner `k` (`forward`) or
    /// piece `k − 1` as it leaves it backward.
    fn leaving(t: &Turned, k: usize, forward: bool) -> Ray {
        let i = if forward { k } else { k - 1 };
        let (from, to) = (t.from(i), t.pieces[i].end());
        let v = t.corner(k);
        match &t.pieces[i] {
            Piece::Line { .. } => {
                let travel = (to - from).normalize();
                Ray::Line {
                    v,
                    out: if forward { travel } else { -travel },
                    left: Vec2::new(-travel.y, travel.x),
                }
            }
            &Piece::Arc {
                via,
                centre,
                radius,
                ..
            } => {
                let swept = sweep(from, via, to, centre);
                Ray::Circle {
                    centre,
                    radius,
                    from: real::atan2(v.y - centre.y, v.x - centre.x),
                    way: if (swept > 0.0) == forward { 1.0 } else { -1.0 },
                    ccw: swept > 0.0,
                }
            }
        }
    }

    fn start(&self) -> Point2 {
        match *self {
            Ray::Line { v, .. } => v,
            Ray::Circle {
                centre,
                radius,
                from,
                ..
            } => centre + Vec2::new(real::cos(from), real::sin(from)) * radius,
        }
    }

    fn direction(&self) -> Vec2 {
        match *self {
            Ray::Line { out, .. } => out,
            Ray::Circle { from, way, .. } => Vec2::new(-real::sin(from), real::cos(from)) * way,
        }
    }

    /// The unit direction of motion at arc length `s`.
    fn direction_at(&self, s: f64) -> Vec2 {
        match *self {
            Ray::Line { out, .. } => out,
            Ray::Circle {
                radius, from, way, ..
            } => {
                let a = from + way * s / radius;
                Vec2::new(-real::sin(a), real::cos(a)) * way
            }
        }
    }

    fn at(&self, s: f64) -> Point2 {
        match *self {
            Ray::Line { v, out, .. } => v + out * s,
            Ray::Circle {
                centre,
                radius,
                from,
                way,
                ..
            } => {
                let a = from + way * s / radius;
                centre + Vec2::new(real::cos(a), real::sin(a)) * radius
            }
        }
    }

    /// The arc length from the corner to the point of the meridian nearest
    /// `p`.
    fn foot(&self, p: Point2) -> f64 {
        match *self {
            Ray::Line { v, out, .. } => (p - v).dot(&out),
            Ray::Circle {
                centre,
                radius,
                from,
                way,
                ..
            } => {
                let a = real::atan2(p.y - centre.y, p.x - centre.x);
                radius * (way * (a - from)).rem_euclid(2.0 * PI)
            }
        }
    }

    /// The arc length at distance `d` from the corner: along a line, the
    /// chord on a circle.
    fn at_distance(&self, d: f64) -> f64 {
        match *self {
            Ray::Line { .. } => d,
            Ray::Circle { radius, .. } => radius * 2.0 * real::asin(d / (2.0 * radius)),
        }
    }
}

/// Where a ball of radius `r` on side `side` (`1` the left of the
/// outline's travel, the material) of both meridians touches them: its
/// centre.
fn ball_centre(a: &Ray, b: &Ray, side: f64, r: f64) -> Point2 {
    let v = a.start();
    // A meridian offset to the ball's side: a line through a point and
    // direction, or a circle.
    enum Offset {
        Line(Point2, Vec2),
        Circle(Point2, f64),
    }
    let offset = |ray: &Ray| match *ray {
        Ray::Line { v, out, left } => Offset::Line(v + left * (side * r), out),
        Ray::Circle {
            centre,
            radius,
            ccw,
            ..
        } => Offset::Circle(centre, radius - side * r * if ccw { 1.0 } else { -1.0 }),
    };
    let nearest = |points: Vec<Point2>| {
        points
            .into_iter()
            .min_by(|p, q| (p - v).norm().total_cmp(&(q - v).norm()))
            .expect("an offset meridian meets the other")
    };
    match (offset(a), offset(b)) {
        (Offset::Line(p, d), Offset::Line(q, e)) => {
            let s = (q - p).perp(&e) / d.perp(&e);
            p + d * s
        }
        (Offset::Line(p, d), Offset::Circle(c, rho))
        | (Offset::Circle(c, rho), Offset::Line(p, d)) => {
            let w = p - c;
            let (half, rest) = (w.dot(&d), w.dot(&w) - rho * rho);
            let disc = (half * half - rest).sqrt();
            nearest(vec![p + d * (-half + disc), p + d * (-half - disc)])
        }
        (Offset::Circle(..), Offset::Circle(..)) => unreachable!("two arcs never meet at a corner"),
    }
}

/// `π ∮ x² dy` along `n` samples of `curve` on `[0, 1]`: the volume of
/// revolution about `y` of the region the boundary encloses, by Gauss–
/// Legendre quadrature of 24 points.
fn swept(curve: &dyn Fn(f64) -> (Point2, Vec2)) -> f64 {
    // The nodes and weights of the 8-point rule on three sub-intervals.
    const NODES: [f64; 4] = [
        0.183434642495649,
        0.525532409916328,
        0.796666477413626,
        0.960289856497536,
    ];
    const WEIGHTS: [f64; 4] = [
        0.362683783378361,
        0.313706645877887,
        0.222381034453374,
        0.101228536290376,
    ];
    let mut sum = 0.0;
    for part in 0..3 {
        let (lo, hi) = (part as f64 / 3.0, (part + 1) as f64 / 3.0);
        for (x, w) in NODES.iter().zip(WEIGHTS) {
            for sign in [-1.0, 1.0] {
                let t = (lo + hi) / 2.0 + sign * x * (hi - lo) / 2.0;
                let (p, d) = curve(t);
                sum += w * (hi - lo) / 2.0 * p.x * p.x * d.y;
            }
        }
    }
    PI * sum
}

/// The volume of the part's own outline, by Pappus about `y`.
fn turned_volume(t: &Turned) -> f64 {
    let mut total = 0.0;
    for i in 0..t.pieces.len() {
        let (from, to) = (t.from(i), t.pieces[i].end());
        total += match &t.pieces[i] {
            Piece::Line { .. } => swept(&|s| (from + (to - from) * s, to - from)),
            &Piece::Arc {
                via,
                centre,
                radius,
                ..
            } => {
                let sw = sweep(from, via, to, centre);
                let a0 = real::atan2(from.y - centre.y, from.x - centre.x);
                swept(&|s| {
                    let a = a0 + sw * s;
                    (
                        centre + Vec2::new(real::cos(a), real::sin(a)) * radius,
                        Vec2::new(-real::sin(a), real::cos(a)) * (radius * sw),
                    )
                })
            }
        };
    }
    total.abs()
}

/// A boundary piece: its point and velocity at `s` in `[0, 1]`.
type Closing = Box<dyn Fn(f64) -> (Point2, Vec2)>;

/// The section corner `k` loses (convex) or gains (concave), swept: the
/// region between the corner, the two contacts and the ball's arc or the
/// chamfer's chord.
fn corner_volume(t: &Turned, k: usize) -> (f64, bool) {
    let (a, b) = (Ray::leaving(t, k, false), Ray::leaving(t, k, true));
    let (t_in, t_out) = (-a.direction(), b.direction());
    let convex = t_in.perp(&t_out) > 0.0;
    let side = if convex { 1.0 } else { -1.0 };
    let r = t.size;
    let (sa, sb, closing): (f64, f64, Closing) = if t.chamfer {
        let (sa, sb) = (a.at_distance(r), b.at_distance(r));
        let (pa, pb) = (a.at(sa), b.at(sb));
        (sa, sb, Box::new(move |s| (pa + (pb - pa) * s, pb - pa)))
    } else {
        let c = ball_centre(&a, &b, side, r);
        let (sa, sb) = (a.foot(c), b.foot(c));
        let (pa, pb) = (a.at(sa), b.at(sb));
        let angle = |p: Point2| real::atan2(p.y - c.y, p.x - c.x);
        let mut turn = angle(pb) - angle(pa);
        while turn > PI {
            turn -= 2.0 * PI;
        }
        while turn < -PI {
            turn += 2.0 * PI;
        }
        let a0 = angle(pa);
        (
            sa,
            sb,
            Box::new(move |s| {
                let q = a0 + turn * s;
                (
                    c + Vec2::new(real::cos(q), real::sin(q)) * r,
                    Vec2::new(-real::sin(q), real::cos(q)) * (r * turn),
                )
            }),
        )
    };
    // The corner out along one meridian to its contact, the closing arc or
    // chord, and back along the other.
    let out = swept(&|s| (a.at(sa * s), a.direction_at(sa * s) * sa));
    let back = swept(&|s| (b.at(sb * (1.0 - s)), -b.direction_at(sb * (1.0 - s)) * sb));
    let close = swept(&|s| closing(s));
    ((out + close + back).abs(), convex)
}

/// The edge of `body` that is the circle about `(0, y, 0)` of radius `x`.
fn circle_at(m: &Model, body: Body, x: f64, y: f64) -> Result<Edge, TestCaseError> {
    for e in m.edges(body).map_err(fail)? {
        let entity = m.edge(e.id).map_err(fail)?;
        let Some((curve, _)) = entity.curve() else {
            continue;
        };
        if let Curve::Circle { frame, radius } = m.curve(curve).map_err(fail)?
            && (frame.origin() - Point3::new(0.0, y, 0.0)).norm() < 1e-9
            && (radius - x).abs() < 1e-9
        {
            return Ok(e);
        }
    }
    Err(fail(format!("no circle of radius {x} at height {y}")))
}

fn turned_in(m: &mut Model, t: &Turned) -> Result<Body, TestCaseError> {
    let axis = Axis::new(Point3::origin(), Vec3::y()).map_err(fail)?;
    Ok(revolve(m, &t.profile(), axis, 2.0 * PI).map_err(fail)?.0)
}

fn blend_turned(
    m: &mut Model,
    body: Body,
    t: &Turned,
) -> Result<(Body, Provenance), TestCaseError> {
    let edges = t
        .blended
        .iter()
        .map(|&k| circle_at(m, body, t.corner(k).x, t.corner(k).y))
        .collect::<Result<Vec<_>, _>>()?;
    let kind = if t.chamfer {
        Blend::Chamfer
    } else {
        Blend::Fillet
    };
    op(kind)(m, body, &edges, t.size).map_err(|e| fail(format!("{kind:?} of the turned part: {e}")))
}

fn turned_matches_pappus(t: Turned) -> Result<(), TestCaseError> {
    let mut m = Model::default();
    let part = turned_in(&mut m, &t)?;
    let (blended, p) = blend_turned(&mut m, part, &t)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[part], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let mut want = turned_volume(&t);
    for &k in &t.blended {
        let (section, convex) = corner_volume(&t, k);
        want += if convex { -section } else { section };
    }
    // A section on a cone, a sphere or a torus is fitted, so the volume
    // holds to the model's tolerance over the body's size.
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs Pappus {}",
        props.volume,
        want
    );
    let mut again = Model::default();
    let part = turned_in(&mut again, &t)?;
    let (twice, again_p) = blend_turned(&mut again, part, &t)?;
    prop_assert_eq!(
        dump_text(&again, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(again_p, p);
    Ok(())
}

prop_shards! {
    /// A turned part — a coned shoulder, a dome, a toroidal bead — with a
    /// pick of its circular corners filleted or chamfered in one call:
    /// clean at `Full` with nothing unchecked, audited, at Pappus' volume
    /// about the axis and deterministic.
    turned_parts_blend_as_pappus
        [shard_0 shard_1 shard_2 shard_3] (t) = turned() => {
            turned_matches_pappus(t)
        }
}
/// A scene whose blend ends on a face across, by the family of ADR-0037.
#[derive(Debug, Clone, PartialEq)]
enum Ends {
    /// A rib `[0, −w, 0]–[cx, w, h]` running along `x` into a boss about
    /// the vertical axis through `(cx, 0)` from `z = −1` to `h + 1`, of
    /// radius `reach · w` at the foot and `(1 − taper)` of that at the top
    /// (a cylinder when `taper` is `0`); the edge at `y = w` along the
    /// top is blended, `lead` long before it meets the boss at the top.
    Rib {
        w: f64,
        h: f64,
        reach: f64,
        taper: f64,
        lead: f64,
    },
    /// A plate `[−6, 6]² × [0, 1]` with a boss of radius `ra` about the
    /// vertical axis through the origin and a second of radius `rb` and
    /// height `hb` about `(d, 0)`, whose wall crosses the first's: the
    /// first boss's foot, an open arc, is blended.
    Twin {
        ra: f64,
        rb: f64,
        d: f64,
        ha: f64,
        hb: f64,
    },
}

/// A scene, a blend size and a pose.
#[derive(Debug, Clone)]
struct EndCase {
    scene: Ends,
    size: f64,
    pose: Isometry,
}

/// How far a ball's section keeps from the face across's grazing: the
/// ring of a twin boss's blend meets the second wall at angles of at
/// least this much of a radius.
const TWIN_MARGIN: f64 = 0.15;

impl Ends {
    /// The boss's radius at height `z` of a rib's scene.
    fn rib_radius(&self, z: f64) -> f64 {
        let Ends::Rib {
            w, h, reach, taper, ..
        } = *self
        else {
            return 0.0;
        };
        let (foot, top) = (reach * w, reach * w * (1.0 - taper));
        foot + (top - foot) * (z + 1.0) / (h + 2.0)
    }

    /// The boss's axis `x` of a rib's scene.
    fn rib_centre(&self) -> f64 {
        let Ends::Rib { w, h, lead, .. } = *self else {
            return 0.0;
        };
        (real::powi(self.rib_radius(h), 2) - w * w).sqrt() + lead
    }

    fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match *self {
            Ends::Rib { w, h, taper, .. } => {
                let cx = self.rib_centre();
                let (rib, _) = primitive_box(m, Point3::new(0.0, -w, 0.0), Point3::new(cx, w, h))?;
                let boss = if taper == 0.0 {
                    let axis = Axis::new(Point3::new(cx, 0.0, -1.0), Vec3::z())?;
                    primitive_cylinder(m, axis, self.rib_radius(-1.0), h + 2.0)?.0
                } else {
                    let plane = Frame::new(Point3::new(cx, 0.0, 0.0), -Vec3::y(), Vec3::x())?;
                    let (foot, top) = (self.rib_radius(-1.0), self.rib_radius(h + 1.0));
                    let corners = [
                        Point2::new(foot, -1.0),
                        Point2::new(top, h + 1.0),
                        Point2::new(0.0, h + 1.0),
                        Point2::new(0.0, -1.0),
                    ];
                    let profile = Profile {
                        plane,
                        outer: ProfileLoop::Path {
                            start: corners[3],
                            segments: corners.iter().map(|&q| ProfileSegment::LineTo(q)).collect(),
                        },
                        holes: Vec::new(),
                    };
                    let axis = Axis::new(Point3::new(cx, 0.0, 0.0), Vec3::z())?;
                    revolve(m, &profile, axis, 2.0 * PI)?.0
                };
                Ok(fuse(m, rib, boss)?.0)
            }
            Ends::Twin { ra, rb, d, ha, hb } => {
                let (plate, _) =
                    primitive_box(m, Point3::new(-6.0, -6.0, 0.0), Point3::new(6.0, 6.0, 1.0))?;
                let (a, _) = primitive_cylinder(
                    m,
                    Axis::new(Point3::new(0.0, 0.0, 1.0), Vec3::z())?,
                    ra,
                    ha,
                )?;
                let (b, _) =
                    primitive_cylinder(m, Axis::new(Point3::new(d, 0.0, 1.0), Vec3::z())?, rb, hb)?;
                let (plated, _) = fuse(m, plate, a)?;
                Ok(fuse(m, plated, b)?.0)
            }
        }
    }

    /// The blended edge of the unposed body, as a point of it and the
    /// test that picks it among `body`'s edges.
    fn edge(&self, m: &Model, body: Body, pose: &Isometry) -> Result<Edge, TestCaseError> {
        match *self {
            Ends::Rib { w, h, lead, .. } => {
                edge_near(m, body, pose.apply(Point3::new(lead / 2.0, w, h)))
            }
            Ends::Twin { ra, .. } => {
                let foot = pose.apply(Point3::new(0.0, 0.0, 1.0));
                let up = pose.apply(Point3::new(0.0, 0.0, 2.0)) - foot;
                let scale = foot.coords.norm().max(1.0);
                for e in m.edges(body).map_err(fail)? {
                    let entity = m.edge(e.id).map_err(fail)?;
                    let Some((curve, range)) = entity.curve() else {
                        continue;
                    };
                    let Curve::Circle { frame, radius } = m.curve(curve).map_err(fail)? else {
                        continue;
                    };
                    let at = m.curve(curve).map_err(fail)?.point(range.midpoint());
                    let off_axis = (frame.origin() - foot).cross(&up).norm();
                    if (radius - ra).abs() < 1e-9 * scale
                        && off_axis < 1e-9 * scale
                        && (at - foot).dot(&up).abs() < 1e-9 * scale
                    {
                        return Ok(e);
                    }
                }
                Err(fail(format!("no foot arc of radius {ra}")))
            }
        }
    }

    /// The signed change of volume of blending the edge at `kind`'s size
    /// `r`: minus the sliver of a convex edge, plus the fillet of a
    /// concave one, integrated over the section's region and the
    /// face across.
    fn change(&self, kind: Blend, r: f64) -> f64 {
        const N: usize = 400;
        match *self {
            Ends::Rib { w, h, .. } => {
                let cx = self.rib_centre();
                // The region of the section, (u, v) from the corner's
                // square `[w − r, w] × [h − r, h]`, above `floor(u)`, and
                // the rib's length there: it ends on the boss's wall at
                // `x = cx − √(ρ(z)² − y²)`.
                // `u = r sin α` keeps a ball's `√(r² − u²)` smooth.
                let sliver = simpson(0.0, FRAC_PI_2, N, &|a| {
                    let u = r * real::sin(a);
                    let lo = match kind {
                        Blend::Fillet => r * real::cos(a),
                        Blend::Chamfer => r - u,
                    };
                    r * real::cos(a)
                        * simpson(lo, r, N, &|v| {
                            let (y, z) = (w - r + u, h - r + v);
                            cx - (real::powi(self.rib_radius(z), 2) - y * y).max(0.0).sqrt()
                        })
                });
                -sliver
            }
            Ends::Twin { ra, rb, d, .. } => {
                // The ring about the first boss's axis from radius `ra`
                // to `ra + r`, outside the second boss, over the section
                // between the plate, the wall and the ball or the chamfer.
                let outside = |rho: f64| {
                    let c = (rho * rho + d * d - rb * rb) / (2.0 * rho * d);
                    rho * (2.0 * PI - 2.0 * real::acos(c.clamp(-1.0, 1.0)))
                };
                match kind {
                    Blend::Fillet => {
                        // ρ = ra + r − r cos α, the gap r − r sin α.
                        simpson(0.0, FRAC_PI_2, N, &|a| {
                            let rho = ra + r - r * real::cos(a);
                            outside(rho) * (r - r * real::sin(a)) * r * real::sin(a)
                        })
                    }
                    Blend::Chamfer => simpson(ra, ra + r, N, &|rho| outside(rho) * (ra + r - rho)),
                }
            }
        }
    }
}

fn end_case() -> impl Strategy<Value = EndCase> {
    let rib = (
        prop::finite_f64(0.5..=1.5),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(1.5..=2.5),
        prop_oneof![Just(0.0), prop::finite_f64(0.05..=0.25)],
        prop::finite_f64(1.0..=3.0),
        prop::finite_f64(0.1..=1.0),
    )
        .prop_map(|(w, h, reach, taper, lead, fraction)| {
            (
                Ends::Rib {
                    w,
                    h,
                    reach,
                    taper,
                    lead,
                },
                fraction * w.min(h) / 3.0,
            )
        });
    let twin = (
        prop::finite_f64(0.9..=1.6),
        prop::finite_f64(0.5..=1.2),
        prop::finite_f64(0.0..=1.0),
        prop::finite_f64(1.0..=2.0),
        prop::finite_f64(0.5..=1.5),
        prop::finite_f64(0.0..=1.0),
    )
        .prop_map(|(ra, rb, u, ha, hb, fraction)| {
            let size = 0.03 + fraction * 0.17;
            let lo = (ra + size + TWIN_MARGIN - rb).max(rb - ra + TWIN_MARGIN);
            let hi = ra + rb - TWIN_MARGIN;
            (
                Ends::Twin {
                    ra,
                    rb,
                    d: lo + u * (hi - lo),
                    ha,
                    hb,
                },
                size,
            )
        });
    (prop_oneof![rib, twin], prop::pose()).prop_map(|((scene, size), pose)| EndCase {
        scene,
        size,
        pose,
    })
}

/// The blend of `case` in a fresh model, the scene moved to its pose
/// first: the model, the input, the result and its record.
fn end_posed(
    case: &EndCase,
    kind: Blend,
) -> Result<(Model, Body, Body, Provenance), TestCaseError> {
    let mut m = Model::default();
    let scene = case.scene.build(&mut m).map_err(fail)?;
    let (moved, _) = transform(&mut m, scene, &case.pose).map_err(fail)?;
    let edge = case.scene.edge(&m, moved, &case.pose)?;
    let (blended, p) = op(kind)(&mut m, moved, &[edge], case.size).map_err(|e| {
        // A turned twin boss's chamfer cone is refused against the second
        // wall's cylinder in some poses: the failure waits as
        // `regression/twin-boss-foot-turned-chamfer-cone-cylinder`, and
        // the property rejects that refusal alone until the fix lands.
        let cone_on_cylinder = |k: &arris_geom::GeomKind| k.to_string();
        if let (OpError::Unsupported { a, b }, Ends::Twin { .. }, Blend::Chamfer) =
            (&e, &case.scene, kind)
            && cone_on_cylinder(&a.0) == "cone surface"
            && cone_on_cylinder(&b.0) == "cylinder surface"
        {
            return TestCaseError::reject("twin-boss-foot-turned-chamfer-cone-cylinder");
        }
        fail(format!("{kind:?} of the posed scene: {e}"))
    })?;
    Ok((m, moved, blended, p))
}

fn ends_blend_as_their_sections(case: EndCase, kind: Blend) -> Result<(), TestCaseError> {
    let (m, moved, blended, p) = end_posed(&case, kind)?;
    assert_checked(&check(&m, blended, Level::Full))?;
    audit(&m, &[moved], blended, &p).map_err(|e| fail(format!("provenance: {e}")))?;
    let props = mass_properties(&m, blended).map_err(fail)?;
    let before = mass_properties(&m, moved).map_err(fail)?;
    let want = before.volume + case.scene.change(kind, case.size);
    let rel = fitted_rel(&m, &props);
    prop_assert!(
        close_to(props.volume, want, 1.0, rel),
        "volume {} vs the section's {}",
        props.volume,
        want
    );

    // Blended, then moved.
    let mut here = Model::default();
    let scene = case.scene.build(&mut here).map_err(fail)?;
    let edge = case.scene.edge(&here, scene, &Isometry::identity())?;
    let (rest, _) = op(kind)(&mut here, scene, &[edge], case.size)
        .map_err(|e| fail(format!("{kind:?} at rest: {e}")))?;
    assert_checked(&check(&here, rest, Level::Full))?;
    let (then_moved, _) = transform(&mut here, rest, &case.pose).map_err(fail)?;
    let other = mass_properties(&here, then_moved).map_err(fail)?;
    prop_assert!(
        close_to(other.volume, props.volume, 1.0, rel),
        "blend then move {} vs move then blend {}",
        other.volume,
        props.volume
    );
    prop_assert_eq!(
        arris_debug::dump::euler_line(&here, then_moved).map_err(fail)?,
        arris_debug::dump::euler_line(&m, blended).map_err(fail)?,
        "counts: blend then move against move then blend"
    );

    // Through STEP and back.
    let text = step::write(&m, &[blended]).map_err(fail)?;
    let mut back = Model::new(m.precision()).map_err(fail)?;
    let read = step_read(&mut back, &text, &ReadOptions::default()).map_err(fail)?;
    prop_assert_eq!(read.solids.len(), 1, "one solid read back");
    let solid = read.solids[0]
        .result
        .as_ref()
        .map_err(|r| fail(format!("refused: {r}")))?;
    assert_checked(&check(&back, solid.body, Level::Full))?;
    let again = mass_properties(&back, solid.body).map_err(fail)?;
    prop_assert!(
        close_to(again.volume, props.volume, 1.0, rel),
        "STEP read back {} of {}",
        again.volume,
        props.volume
    );

    // Deterministic.
    let (twin, _, twice, twice_p) = end_posed(&case, kind)?;
    prop_assert_eq!(
        dump_text(&twin, twice).map_err(fail)?,
        dump_text(&m, blended).map_err(fail)?
    );
    prop_assert_eq!(twice_p, p);
    Ok(())
}

prop_shards! {
    /// A rib into a round or conical boss, or a boss's foot where a
    /// second boss crosses it, filleted: clean at `Full` with nothing
    /// unchecked, audited, at the section's volume integrated over the
    /// face the blend ends on, pose-independent, through STEP and
    /// deterministic.
    ends_fillet_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = end_case() => {
            ends_blend_as_their_sections(case, Blend::Fillet)
        }
}

prop_shards! {
    /// The same scenes chamfered: a rib's plane ends on the boss in a
    /// conic, a boss's foot cone on the second boss in a traced curve.
    ends_chamfer_as_their_sections
        [shard_0 shard_1 shard_2 shard_3] (case) = end_case() => {
            ends_blend_as_their_sections(case, Blend::Chamfer)
        }
}
