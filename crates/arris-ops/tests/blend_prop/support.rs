pub(super) use arris_check::{Level, Report, Unchecked, check};
pub(super) use arris_debug::polyhedron::polyhedron;
pub(super) use arris_debug::prop::turned::{Piece, Turned, sweep, turned};
pub(super) use arris_debug::testing::{REL, close_to, fail, fitted_rel};
pub(super) use arris_debug::unmetered::build;
pub(super) use arris_debug::unmetered::{
    chamfer, cut, extrude, fillet, fuse, mass_properties, mirror, primitive_box,
    primitive_cylinder, revolve, step_read, transform,
};
pub(super) use arris_debug::{dump_text, prop, prop_shards};
pub(super) use arris_geom::SurfaceKind;
pub(super) use arris_geom::{Curve, Profile, ProfileLoop, ProfileSegment};
pub(super) use arris_io::step::{self, ReadOptions};
pub(super) use arris_math::nalgebra::UnitQuaternion;
pub(super) use arris_math::real;
pub(super) use arris_math::{Axis, Frame, Isometry, Point2, Point3, Vec2, Vec3};
pub(super) use arris_ops::{BlendReason, OpError, Reason};
pub(super) use arris_topo::provenance::audit;
pub(super) use arris_topo::{Body, Edge, Model, Provenance};
pub(super) use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, SQRT_2};
pub(super) use proptest::prelude::*;

/// Fillet or chamfer, which the closed forms and the checker's
/// expectation depend on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Blend {
    Fillet,
    Chamfer,
}

pub(super) type BlendOp = fn(&mut Model, Body, &[Edge], f64) -> Result<(Body, Provenance), OpError>;

pub(super) fn op(kind: Blend) -> BlendOp {
    match kind {
        Blend::Fillet => fillet,
        Blend::Chamfer => chamfer,
    }
}

/// The edge of `body` whose curve's midpoint is nearest `at`, required
/// within `1e-9` of `at`'s distance from the origin (and of 1): a pose
/// moves the body up to 100 away, and the midpoint with it.
pub(super) fn edge_near(m: &Model, body: Body, at: Point3) -> Result<Edge, TestCaseError> {
    let mut best: Option<(f64, Edge)> = None;
    for e in m.edges(body).map_err(fail)? {
        let entity = m.edge(e.id).map_err(fail)?;
        let Some((curve, range)) = entity.curve() else {
            continue;
        };
        let d = (m.curve(curve).map_err(fail)?.point(range.midpoint()) - at).norm();
        if best.is_none_or(|(b, _)| d < b) {
            best = Some((d, e));
        }
    }
    match best {
        Some((d, e)) if d <= 1e-9 * at.coords.norm().max(1.0) => Ok(e),
        _ => Err(fail(format!("no edge through {at}"))),
    }
}

/// `report` has no violation and nothing unchecked: every face pair a
/// blend makes is decided by S5 and B1, the skew blend cylinders two
/// fillets meet in and a corner's sphere against a blend cylinder off its
/// centre among them, traced over the overlap of the two faces' boxes
/// (ADR-0018). Nothing here carries a torus face — every blended edge is
/// between two planes — but nothing would be left undecided if it did:
/// every pair of analytic surfaces is decided in every pose (ADR-0019).
pub(super) fn assert_checked(report: &Report) -> Result<(), TestCaseError> {
    prop_assert!(report.is_ok(), "{}", report);
    prop_assert!(
        report.unchecked().is_empty(),
        "nothing unchecked\n{}",
        report
    );
    Ok(())
}

/// The composite Simpson rule over `[a, b]` at `n` (even) intervals: the
/// integrands below are smooth after their substitution.
pub(super) fn simpson(a: f64, b: f64, n: usize, f: &dyn Fn(f64) -> f64) -> f64 {
    let h = (b - a) / n as f64;
    let mut sum = f(a) + f(b);
    for i in 1..n {
        sum += f(a + h * i as f64) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    sum * h / 3.0
}

/// A scene whose blended edge ends at a cusp.
#[derive(Debug, Clone, PartialEq)]
pub(super) enum Cusped {
    /// The crescent prism `z ∈ [0, h]`: below the `x` axis, inside the
    /// circle of radius `big` about the origin and outside the circle of
    /// radius `small` about `(small − big, 0)`, the two tangent at
    /// `(−big, 0)`. The big arc's edge is blended, or the small arc's when
    /// `small_arc`, at the floor when `floor` and at the top otherwise:
    /// both convex, the sliver between the walls material.
    Crescent {
        big: f64,
        small: f64,
        h: f64,
        small_arc: bool,
        floor: bool,
    },
    /// The same crescent cut `h` deep from the top of a block, its floor at
    /// `z = 1`: the floor edge of the big arc, or of the small arc when
    /// `small_arc`, is concave, and the sliver void.
    Pocket {
        big: f64,
        small: f64,
        h: f64,
        small_arc: bool,
    },
    /// The square `[0, a]²` less the quarter disc of radius `a` about
    /// `(0, a)`, `z ∈ [0, h]`: the arc leaves the bottom edge tangent at
    /// the origin. The bottom wall's top or floor edge, a line between two
    /// planes, is blended.
    Spandrel { a: f64, h: f64, floor: bool },
    /// A lip over a pillar (ADR-0079): the pillar of radius `pillar` about
    /// the z axis, and a lip `thick` thick on the pillar from `z = 1`, the
    /// rectangle below the `x` axis less the pillar's disc and less the
    /// bite's disc of radius `bite` about `(−pillar − bite, 0)`, tangent to
    /// the pillar's at `(−pillar, 0)`. The bite's underside arc, convex, is
    /// blended: the walls leave the cusp one on each side of the underside,
    /// an overhang tip. `mirrored` reflects the lip's profile frame.
    Overhang {
        pillar: f64,
        bite: f64,
        thick: f64,
        mirrored: bool,
    },
}

/// The crescent prism from `z0` to `z0 + h`: the half disc below the `x`
/// axis less the small cylinder. A boolean, not a profile of three
/// segments: `Profile::edges` refuses a loop with a cusp unless the small
/// radius is half the big one (`regression/cusp-profile-off-half`).
pub(super) fn crescent(
    m: &mut Model,
    big: f64,
    small: f64,
    z0: f64,
    h: f64,
) -> Result<Body, OpError> {
    let half = Profile {
        plane: Frame::new(Point3::new(0.0, 0.0, z0), Vec3::z(), Vec3::x())?,
        outer: ProfileLoop::Path {
            start: Point2::new(-big, 0.0),
            segments: vec![
                ProfileSegment::ArcTo {
                    to: Point2::new(big, 0.0),
                    via: Point2::new(0.0, -big),
                },
                ProfileSegment::LineTo(Point2::new(-big, 0.0)),
            ],
        },
        holes: Vec::new(),
    };
    let (disc, _) = extrude(m, &half, Vec3::z(), h)?;
    let axis = Axis::new(Point3::new(small - big, 0.0, z0 - 1.0), Vec3::z())?;
    let (bite, _) = primitive_cylinder(m, axis, small, h + 2.0)?;
    Ok(cut(m, disc, bite)?.0)
}

impl Cusped {
    pub(super) fn build(&self, m: &mut Model) -> Result<Body, OpError> {
        match *self {
            Cusped::Crescent { big, small, h, .. } => crescent(m, big, small, 0.0, h),
            Cusped::Overhang {
                pillar,
                bite,
                thick,
                mirrored,
            } => {
                // The profile frame's normal flips with the hand, so its y
                // axis does; the extrusion still runs up the z axis.
                let hand = if mirrored { -1.0 } else { 1.0 };
                let (x, y) = (pillar + 2.0 * bite + 1.0, pillar + 1.0);
                let profile = Profile {
                    plane: Frame::new(Point3::new(0.0, 0.0, 1.0), hand * Vec3::z(), Vec3::x())?,
                    outer: ProfileLoop::Path {
                        start: Point2::new(-pillar, 0.0),
                        segments: vec![
                            ProfileSegment::ArcTo {
                                to: Point2::new(-pillar - 2.0 * bite, 0.0),
                                via: Point2::new(-pillar - bite, -bite),
                            },
                            ProfileSegment::LineTo(Point2::new(-x, 0.0)),
                            ProfileSegment::LineTo(Point2::new(-x, -y)),
                            ProfileSegment::LineTo(Point2::new(x, -y)),
                            ProfileSegment::LineTo(Point2::new(x, 0.0)),
                            ProfileSegment::LineTo(Point2::new(pillar, 0.0)),
                            ProfileSegment::ArcTo {
                                to: Point2::new(-pillar, 0.0),
                                via: Point2::new(0.0, -pillar),
                            },
                        ],
                    },
                    holes: Vec::new(),
                };
                let axis = Axis::new(Point3::origin(), Vec3::z())?;
                let (post, _) = primitive_cylinder(m, axis, pillar, 1.0 + thick + 1.0)?;
                let (lip, _) = extrude(m, &profile, Vec3::z(), thick)?;
                Ok(fuse(m, post, lip)?.0)
            }
            Cusped::Pocket { big, small, h, .. } => {
                let (block, _) = primitive_box(
                    m,
                    Point3::new(-big - 1.0, -big - 1.0, 0.0),
                    Point3::new(big + 1.0, 1.0, 1.0 + h),
                )?;
                let tool = crescent(m, big, small, 1.0, h + 1.0)?;
                Ok(cut(m, block, tool)?.0)
            }
            Cusped::Spandrel { a, h, .. } => {
                let t = FRAC_PI_4;
                let profile = Profile {
                    plane: Frame::world(),
                    outer: ProfileLoop::Path {
                        start: Point2::origin(),
                        segments: vec![
                            ProfileSegment::LineTo(Point2::new(a, 0.0)),
                            ProfileSegment::LineTo(Point2::new(a, a)),
                            ProfileSegment::ArcTo {
                                to: Point2::origin(),
                                via: Point2::new(a * real::cos(t), a - a * real::sin(t)),
                            },
                        ],
                    },
                    holes: Vec::new(),
                };
                Ok(extrude(m, &profile, Vec3::z(), h)?.0)
            }
        }
    }

    /// The blended edge's midpoint on the unposed body.
    pub(super) fn midpoint(&self) -> Point3 {
        match *self {
            Cusped::Crescent {
                big,
                small,
                h,
                small_arc,
                floor,
            } => {
                let z = if floor { 0.0 } else { h };
                if small_arc {
                    Point3::new(small - big, -small, z)
                } else {
                    Point3::new(0.0, -big, z)
                }
            }
            Cusped::Pocket {
                big,
                small,
                small_arc,
                ..
            } => {
                if small_arc {
                    Point3::new(small - big, -small, 1.0)
                } else {
                    Point3::new(0.0, -big, 1.0)
                }
            }
            Cusped::Spandrel { a, h, floor } => {
                Point3::new(a / 2.0, 0.0, if floor { 0.0 } else { h })
            }
            Cusped::Overhang {
                pillar,
                bite,
                mirrored,
                ..
            } => {
                let hand = if mirrored { -1.0 } else { 1.0 };
                Point3::new(-pillar - bite, -hand * bite, 1.0)
            }
        }
    }

    /// The signed change of volume of blending the edge at `kind`'s size
    /// `r`: minus the sliver of a convex edge, plus the fillet of a concave
    /// one. The section's point `u` in from the blended wall and `v` from
    /// the shared face, in the corner's square below the ball or the
    /// chamfer, sweeps along the wall's offset at `u` for as far as that
    /// offset stays inside the scene's outline: to the far end's wall at a
    /// right angle, and to the next wall at the cusp, which is where the
    /// stripe is cut (ADR-0042). So the change is `∫ g(u) L(u) du` over
    /// `[0, r]`, `g` the section's height at `u` and `L` the offset's
    /// length inside the outline, both with a square root at `u = 0` that
    /// `u = w²` takes away.
    pub(super) fn change(&self, kind: Blend, r: f64) -> f64 {
        const N: usize = 400;
        let g = |u: f64| match kind {
            Blend::Fillet => r - (2.0 * r * u - u * u).max(0.0).sqrt(),
            Blend::Chamfer => r - u,
        };
        // The offset's length at `u` inside the outline.
        let length = |u: f64| match *self {
            Cusped::Crescent {
                big,
                small,
                small_arc,
                ..
            }
            | Cusped::Pocket {
                big,
                small,
                small_arc,
                ..
            } => {
                let c = big - small;
                if small_arc {
                    // A circle of radius `small + u` about the small
                    // centre, below the axis and inside the big circle.
                    let rho = small + u;
                    let k = (c * c + rho * rho - big * big) / (2.0 * c * rho);
                    rho * real::acos(k.clamp(-1.0, 1.0))
                } else {
                    // A circle of radius `big − u` about the origin, below
                    // the axis and outside the small circle.
                    let rho = big - u;
                    let k = (small * small - rho * rho - c * c) / (2.0 * rho * c);
                    rho * real::acos(k.clamp(-1.0, 1.0))
                }
            }
            Cusped::Spandrel { a, .. } => a - (2.0 * a * u - u * u).max(0.0).sqrt(),
            Cusped::Overhang { pillar, bite, .. } => {
                // A circle of radius `bite + u` about the bite's centre, on
                // the lip's side of the axis and outside the pillar's disc,
                // which is where the stripe is cut (ADR-0079): the half
                // circle less the arc inside the disc.
                let rho = bite + u;
                let c = pillar + bite;
                let k = (c * c + rho * rho - pillar * pillar) / (2.0 * c * rho);
                rho * (PI - real::acos(k.clamp(-1.0, 1.0)))
            }
        };
        let swept = simpson(0.0, r.sqrt(), N, &|w| {
            let u = w * w;
            2.0 * w * g(u) * length(u)
        });
        match self {
            Cusped::Pocket { .. } => swept,
            Cusped::Crescent { .. } | Cusped::Spandrel { .. } | Cusped::Overhang { .. } => -swept,
        }
    }
}

pub(super) fn blend_kind() -> impl Strategy<Value = Blend> {
    prop_oneof![Just(Blend::Fillet), Just(Blend::Chamfer)]
}

/// `report` is clean and leaves nothing unchecked but pairs of a plane and a
/// torus, which S5's tracers leave undecided at a knife-edge of poses of a
/// walked chain (`a_walked_chain_in_a_far_pose_is_decided`, a named
/// exclusion: it goes when that test passes).
pub(super) fn feet_checked(report: &Report) -> Result<(), TestCaseError> {
    prop_assert!(report.is_ok(), "{}", report);
    prop_assert!(
        report.unchecked().iter().all(|u| matches!(
            u,
            Unchecked::FacePair {
                kinds: (SurfaceKind::Plane, SurfaceKind::Torus)
                    | (SurfaceKind::Torus, SurfaceKind::Plane),
                ..
            }
        )),
        "nothing unchecked\n{}",
        report
    );
    Ok(())
}

/// A model, its input body and the blend of it, built or refused.
pub(super) type Run = Result<(Model, Body, Result<(Body, Provenance), OpError>), TestCaseError>;
