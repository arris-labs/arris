# 02 — Data Model

What lives in a `Model`: the geometry enums and their parametrisations, the
topology entities and how orientation composes over them, what a pcurve and
a tolerance mean, the invariants the checker enforces, the provenance record
an operation returns, and the native format. The crate boundaries and the
operation contract are in [architecture](ARCHITECTURE.md).

## Conventions

- **Units** are the consumer's. Arris carries no unit; `Precision` (below)
  is what makes a model's numbers meaningful.
- **`f64`** everywhere. Angles in radians. Parameters are `f64`; parameter
  ranges are `Interval` from `arris-math`.
- **Points and vectors** are `nalgebra`'s by alias — `Point3`, `Vec3`,
  `UnitVec3`, `Point2`, `Vec2`, `UnitVec2` — with `nalgebra` re-exported
  from `arris-math` (ADR-0001). A `UnitVec3` is unit by construction and
  that is the only invariant it carries.
- **Frames** are right-handed: `Frame { origin, x, y, z }` with `x`, `y`, `z`
  orthonormal `UnitVec3` and `z = x × y`, built only through validating
  constructors (`Frame::new(origin, z, x_hint)`, `Frame::from_z`, which
  picks `x` by the rule of Open CASCADE's `gp_Ax3(P, N)` so an axis-built
  cylinder seams where the oracle's does, `Frame::from_rotation`, and
  `Frame::from_orthonormal`, which the native format reads a frame back
  through). The first two scale a direction by its largest coordinate before normalising it, so
  an axis given at `1e-154` is as unit as one given at `1`, and
  `Frame::new` refuses a hint that is along `z` to rounding, whose
  residue has no direction. Every
  analytic surface and curve is placed by a frame, so a transform is a
  frame change and nothing else: an **`Isometry`** (a rotation then a
  translation) moves geometry through `Frame::transformed`. A **`Frame2`**
  is the (u, v)-plane analogue and may be of either handedness, which is
  how a pcurve records its direction of traversal.
- **Tolerances** reach an algorithm as `Tolerance { linear, angular }`,
  derived from the model by `Precision::tolerance()` or from an entity's
  own tolerance by the operation that owns it (§Tolerances). Exact
  predicates (`arris_math::predicates::{orient2d, incircle}`) return a
  `Sign` and take no tolerance.
- **Root finding** is `arris_math::roots`: `quadratic`, `cubic` and
  `quartic` return the real roots ascending with multiplicity, a multiple
  root reported once and decided to rounding (`POLYNOMIAL_ROUNDING`, a
  statement about `f64`, never a tolerance); `newton_in_interval` is the
  Newton that never leaves its bracket, for every iteration in the kernel
  that has one.
- **Orientation** is the two-valued `Orientation::{Forward, Reversed}` and
  composes by XOR: `Forward ∘ o = o`, `Reversed ∘ o = !o`.
- **Parametrisations match Open CASCADE's `Geom` classes** for the analytic
  types (read from the reference tree, `SEED.md` §8), so STEP round-trips
  without re-parametrising and the oracle's pcurves agree with ours.
- **Ids are typed and generational**: `VertexId`, `EdgeId`, `FaceId`,
  `ShellId`, `BodyId` for topology; `CurveId`, `SurfaceId`, `Curve2Id` for
  geometry; each `{ index: u32, generation: u32 }`, ordered by `(index,
  generation)`. `EntityId` is the enum over the five topological ids (what
  a `Shape` wraps) and `GeometryId` over the three geometric ones (what an
  entity references); each orders by kind, then id. In text dumps an id is
  its kind letter and index, `f3`, with `g<n>` appended for a generation
  above zero. Loops and coedges are not entities (§Topology).

## Geometry

Geometry is stored in the arena once and referenced by id; two faces may
share a `SurfaceId` (the two halves of a split face do). A geometry value is
never modified.

### Surfaces

```rust
pub enum Surface {
    Plane            { frame: Frame },
    Cylinder         { frame: Frame, radius: f64 },
    EllipticCylinder { frame: Frame, major_radius: f64, minor_radius: f64 },
    Cone             { frame: Frame, radius: f64, half_angle: f64 },
    Sphere           { frame: Frame, radius: f64 },
    Torus            { frame: Frame, major_radius: f64, minor_radius: f64 },
    Nurbs            (NurbsSurface),
}
```

With `O, X, Y, Z` the frame and `c = cos`, `s = sin`:

| Variant | `P(u, v)` | Domain | Periodic | Seam / singularity |
|---|---|---|---|---|
| Plane | `O + u·X + v·Y` | ℝ² | — | none. Normal `Z` |
| Cylinder | `O + R(c u·X + s u·Y) + v·Z` | u ∈ [0, 2π), v ∈ ℝ | u, period 2π | seam at u = 0, the line through `O + R·X` along `Z` |
| EllipticCylinder | `O + a c u·X + b s u·Y + v·Z`, `a ≥ b > 0` | u ∈ [0, 2π), v ∈ ℝ | u, period 2π | seam at u = 0, the ruling through `O + a·X`; `X` is the section's major axis. What an extruded elliptic profile segment sweeps (ADR-0014) |
| Cone | `O + (R + v·s α)(c u·X + s u·Y) + v·c α·Z` | u ∈ [0, 2π), v ∈ ℝ | u | seam at u = 0; apex at v = −R / s α, a degenerate edge. `α` ∈ (0, π/2) is the half-angle; `R` the radius at v = 0 |
| Sphere | `O + R c v (c u·X + s u·Y) + R s v·Z` | u ∈ [0, 2π), v ∈ [−π/2, π/2] | u | seam at u = 0; poles at v = ±π/2, degenerate edges |
| Torus | `O + (R + r c v)(c u·X + s u·Y) + r s v·Z` | u, v ∈ [0, 2π) | u and v | seams at u = 0 and v = 0; `R > r` (no self-intersecting tori until an operation needs them: a revolve refuses one as `Reason::SpindleTorus`) |
| Nurbs | Piegl & Tiller, rational; clamped or not (§NURBS) | knot range | either, where the knots and net wrap or a clamped direction's end rows are one row (its closure) | as the knots say; a clamped end row that is one point is a collapsed row, a singularity |

`Surface::singularities()` lists a surface's singular points as
`Singularity { point, fixed, value }` — the cone's apex, the sphere's two
poles, a NURBS surface's collapsed rows — each the row of `fixed` at
`value` that is one point. `pcurve_on` ends a pcurve on one at the row's
own value, and a loop runs along the row there on a degenerate edge;
the STEP reader rebuilds that edge where the file left it out (ADR-0025
§1) by the same list.

The surface normal is `∂P/∂u × ∂P/∂v`, normalised. For the analytic types
that is: plane `Z`; cylinder, cone and sphere radially outward; an
elliptic cylinder outward along its section's own normal `b c u·X + a s
u·Y`, which is never singular; torus outward from the tube. It is the
*surface's* normal; a face's normal is the surface's composed with the
face use's orientation (§Orientation).

The cone's radius grows along `+Z` and nowhere else — `α` is never
obtuse — so a sweep that needs a cone narrowing along its axis places
the cone with `Z` against the axis (`ops::revolve`, 01 §Operations): the
same surface, its `u` running the other way about the axis, which the
pcurves of the rises that cross it carry (§Pcurves). Every surface a
revolve makes shares that one frame's origin on the axis and its `X`
into the profile's plane, so `u = 0` is the profile plane and every seam
lies in it.

A surface's parametric domain is unbounded where the table says ℝ; a face
trims it with loops. Periodic directions are stored as a period, and a
pcurve on a periodic surface may run outside `[0, 2π)` — a loop that crosses
the seam is written with a seam edge (§Seams), not by unwrapping.

`Surface::eval(u, v)` returns `SurfaceEval { point, du, dv, duu, duv, dvv }`
for every finite parameter, inside the domain or not (a periodic parameter
wraps, and so does a NURBS direction closed without being periodic, by
its domain's length); `normal(u, v)` is `None` where the parametrisation is singular —
the apex, the poles, a zero radius, a NURBS point whose two derivatives
are parallel or vanish — decided to rounding (`arris_math::is_negligible`),
never a direction made of noise. `domain()`
gives the closed fundamental interval of a periodic direction, `[0, 2π]`,
and `Interval::REAL` where the table says ℝ; `period()` the period per
direction — for a `Nurbs`, its `closure()`: the knots' period, or the
domain's length where the direction is closed without it.

`Surface::project(p)` returns the nearest point of the whole parametric
surface (both nappes of a cone) as `SurfaceProjection { uv, point,
distance }` by the variant's closed form, with a periodic `u` in `[0, 2π)`
and the sphere's `v` in `[−π/2, π/2]`. Where the nearest point or its
parameter is not unique — the axis of a cylinder, cone or torus, the plane
through a cone's apex, a sphere's centre, a torus's centre circle, an
elliptic cylinder's axis or the strip over the segment of its section's
major axis inside the evolute — the
result is `GeomError::Ambiguous` naming the locus, decided to rounding and
never resolved by a silent choice of parameter. A point on a sphere's axis
off its centre projects to the pole with `u = 0`: the point is unique,
only the degenerate parameter is not. An elliptic cylinder projects
through its section's quartic (`Curve::project`'s), `v` being the height.
A **`Nurbs`** projects by search, not by closed form (ADR-0025 §1;
`NurbsSurface::project`), and the search is *global*: the surface is cut
into its Bézier patches, each bounded from below by its control hull
(the larger of the distance to the hull's box and two support bounds), and
a best-first search halves the patches whose bound is within the best
distance found and hands each small survivor to a projected Newton
iteration confined to its knot span, so a minimum on a kink between spans
is a minimum of each. What comes out is every local minimum as near as the
nearest to rounding, and two *distinct* points among them are
`GeomError::Ambiguous` at `AmbiguousLocus::MedialAxis` — the surface's
medial axis, the analogue of a quadric's axis — as is a query with more
patches as near as the best one than the search's budget allows (a
continuum: a sphere's centre). Two parameters that name one point are
reported as the lower: a point on the seam of a closed direction at the
start of its knots, one on a collapsed row (a pole) at the row's own `v`
with `u` at the start of its knots, as the sphere's closed form does, and
a periodic parameter inside its domain. A minimum is located to the square
root of rounding in its parameters and to rounding in its distance. A
parameter within the rounding of the point's coordinates of an end of the
domain is that end.

`Surface::chord_steps(chord, bounds)` gives the largest parameter steps
`[hu, hv]` for which a triangle whose corners lie on the surface within
`bounds` deviates from it by at most `chord`, by the second fundamental
form: `INFINITY` along a flat or ruled direction (both on a plane, `v` on
a cylinder, an elliptic cylinder and a cone), the cone's `u` curvature
read at the radius of the region's far `v` bound, an elliptic cylinder's
`u` step bounded by its major radius, a sphere its radius in both directions and a
torus `R + r` in `u` and `r` in `v`, each with the chord shared between
the two directions, a NURBS the form's three coefficients sampled over
`bounds` and one step for both. What tessellation sizes an edge's samples
and a face's interior grid by.

`SurfaceKind` is the fieldless twin of the enum, used in errors and
dispatch tables. `Surface::frame()` is the placing frame of an analytic
variant and `None` for `Nurbs`, which is placed by its control points;
`project` onto a `Nurbs` is a search (above). `Surface::to_nurbs(bounds)`
is the part of any analytic surface over a parameter rectangle as an exact
`NurbsSurface` (§NURBS, exact forms). `Surface` and `Curve`
are `Clone`, not `Copy`: the NURBS variants own their knots and control
points.

### Curves

```rust
pub enum Curve {
    Line    { origin: Point3, direction: UnitVec3 },
    Circle  { frame: Frame, radius: f64 },
    Ellipse { frame: Frame, major_radius: f64, minor_radius: f64 },
    Nurbs   (NurbsCurve),
}
```

| Variant | `P(t)` | Domain | Periodic |
|---|---|---|---|
| Line | `O + t·D` | ℝ | — |
| Circle | `O + R(c t·X + s t·Y)` | [0, 2π) | 2π |
| Ellipse | `O + a c t·X + b s t·Y`, `a ≥ b` | [0, 2π) | 2π |
| Nurbs | rational; clamped or not (§NURBS) | knot range | where the knots and control points wrap |

The tangent is `dP/dt`, never normalised in the enum's own evaluation; a
line's parameter is arc length because `D` is unit. `Curve::eval(t)` returns
`CurveEval { point, d1, d2 }`; `domain()`, `period()` and `kind()`
(`CurveKind`) follow the table as for surfaces. `Curve::project(p)` returns
the nearest point as `CurveProjection { t, point, distance }`, a periodic
`t` in `[0, 2π)`. A line and a circle project by closed form; an ellipse
through the quartic in `tan(t/2)` of `arris_math::roots`, every candidate
polished so the residual `(p − C(t)) · C′(t)` is zero to rounding. A point
on a circle's axis, at an ellipse's centre, or on the open segment of an
ellipse's major axis inside its evolute (two mirror-image nearest points)
is `GeomError::Ambiguous` naming the locus. A NURBS curve projects by
sampling every span (`2p + 2` parameters each) and bracketed Newton on
the derivative of the squared distance around the best sample, across
the seam of a closed or periodic curve, each bracket read on the one
polynomial piece it lies in — at a knot where the curve is only C⁰ that
derivative jumps, and a bracket across it is left to its halves: the
nearest *local* minimum from that sample, never `Ambiguous` and never a
guarantee against a nearer point the sampling missed.

`Curve::chord_segments(range, chord)` is the 3D twin of
`Piece::segment_count` (§Pcurves): how many straight segments approximate
the curve over the range within the chord, by the same `|d2| h² / 8`
bound with the same floors and ceiling.

`intersect_surfaces(a, b, within, tol)` returns
`SurfaceIntersection::{Empty, Coincident, Meets { curves: Vec<MeetCurve>,
points: Vec<MeetPoint> }}` for the pairs it decides and
`GeomError::Unsupported` naming the pair for every other — plane–plane (a
line), plane–cylinder (a circle, an ellipse, two rulings, one tangent
ruling, or nothing), cylinder–cylinder in every pose by the first table
below — lines and conics by closed form, the quartics traced and fitted
inside the region `within` (below the tracer) — every pair with a cone, a
sphere or a torus in it wherever the two share an axis, by the meridian
arm of the third table (ADR-0008), and elsewhere a plane against a cone
in an exact conic and a cylinder, a cone or a sphere against a cone or a
sphere traced and fitted, and the elliptic cylinder's pairs by the second
table (ADR-0014), and a torus against any of them in a section traced in
the torus's own parameter plane (below the torus tracer, ADR-0019); every
pair with a `Nurbs` operand is an explicit `Unsupported` arm, and no
other pair of kinds is (a partial coincidence on a shared axis, below, is
refused by pose, not by kind). Every curve and every point
of a `Meets` carries its `MeetKind` (ADR-0018): a curve is a `Crossing`
when the surfaces cross along it and a `Touch` when they are tangent all
along it; a point is a `Touch` when the surfaces are tangent there and
apart around it (a plane tangent to a sphere, two spheres touching) and
a `Crossing` through a singular point (a plane perpendicular to a cone
through its apex, two cones closing on one apex). A `Meets` holds at
least one entry and mixes kinds freely — a circle beside a point, a
touching ruling between two crossing ones; its curves come in the order
the arm documents, the kinds interleaved in it, and its points lie on
the shared axis and ascend along it — a traced section's come in the
tracer's order instead, its exact tube circles before its fitted
branches. The tables name a curve's kind
where it is a touch; every other curve crosses. `tol.angular` decides parallel and
perpendicular, `tol.linear` decides coincident, tangent and empty.

| Two cylinders, radii `R₁`, `R₂` | Result |
|---|---|
| Parallel axes, `d` apart, `d ≤ tol.linear` (coaxial) | `Coincident` when the radii agree within `tol.linear`, `Empty` when they do not |
| Parallel, `d` within `tol.linear` of `R₁ + R₂` or of `\|R₁ − R₂\|` | one touching ruling |
| Parallel, `\|R₁ − R₂\| < d < R₁ + R₂` | two rulings |
| Parallel, otherwise (apart, nested) | `Empty` |
| Crossing axes (nearest approach within `tol.linear`), radii equal within `tol.linear` | two ellipses |
| Crossing axes, unequal radii | the fitted branches of the traced quartic: two loops |
| Skew axes, nearest approach over `R₁ + R₂ + tol.linear` | `Empty` (triangle inequality) |
| Skew axes, nearest approach within that | the fitted branches of the traced quartic — one loop or two, a figure eight's two lobes ending at its singular point, or a touching point alone |

**The elliptic cylinder's arms** (ADR-0014). A plane is decided in every
pose; a cylinder or another elliptic cylinder where the axes are
parallel within `tol.angular`, by the two *sections* in the plane across
the first operand's axis through its origin — two conics, `geom`'s
`conic2` — where `F₂(E₁(t))`, the second's implicit form along the
first's parametrisation, is a trigonometric polynomial of degree two:
its extrema come from the quartic in `tan(t/2)` of `arris_math::roots`,
an extremum where the first section is within `tol.linear` of the
second is a touch, every extremum a touch is `Coincident`, and each arc
between two extrema that are not touches whose ends differ in sign holds
one crossing by bracketed Newton. Each meeting is a ruling along the
first operand's `Z` from its section point, ascending by the first
section's parameter — up to four.

| Elliptic cylinder, radii `a ≥ b`, against | Result |
|---|---|
| A plane, normal parallel to the axis within `tol.angular` | the section ellipse at the piercing point, the cylinder's own axes |
| A plane parallel to the axis, offset `d` from the section's centre along its normal `n` against the section's reach `M = √((a n·X)² + (b n·Y)²)` | `\|d\|` within `tol.linear` of `M`: one touching ruling; `\|d\| < M`: two rulings ordered along `n × Z`, negative first; beyond: `Empty` |
| A plane oblique to the axis | one ellipse: the affine image of the section, its semi-axes the singular values of the section's semi-diameters slid along the axis into the plane, `Z` the plane's normal, `X` the major axis |
| A cylinder or an elliptic cylinder with parallel axes | `Coincident`, `Empty`, or a ruling at each meeting of the sections, touching where they touch — both kinds in one `Meets` when the sections both touch and cross |
| A cylinder or an elliptic cylinder with other axes; a cone, a sphere, in any pose | the fitted branches of the traced section |
| A torus, in any pose | the fitted branches of the section traced in the torus's parameter plane (ADR-0019); the two share no axis, an elliptic cylinder being no surface of revolution |
| A NURBS | `Unsupported` |

**The meridian arm.** Two surfaces of revolution about one axis meet
where their meridians meet in a plane through the axis. In that plane,
with `ρ` signed across the axis and `z` along it, each surface is a set
of lines and circles symmetric under `ρ ↦ −ρ`, its *meridian sections*:

| Surface | Meridian sections |
|---|---|
| Plane perpendicular to the axis, at height `h` | the line `z = h` |
| Cylinder, radius `R` | the lines `ρ = ±R` |
| Cone, apex at `z_a`, half-angle `α` | the two lines through `(0, z_a)` at `±α` from the axis, both nappes |
| Sphere, centre at `z_c`, radius `R` | the circle about `(0, z_c)` of radius `R` |
| Torus, centre at `z_c`, radii `R > r` | the circles about `(±R, z_c)` of radius `r` |

A sphere is a surface of revolution about any line through its centre,
so the arm covers every plane–sphere and sphere–sphere pair too.
*Coaxial* means: the axes of two cylinders, cones or tori parallel within
`tol.angular` and the second's origin within `tol.linear` of the first's
axis; a plane whose normal is parallel to the axis within `tol.angular`;
a sphere whose centre is within `tol.linear` of the axis; two spheres,
whose axis is the line through their centres, or, when the centres are
within `tol.linear`, no axis at all — `Coincident` when the radii agree
within `tol.linear`, `Empty` when they do not. Each pair of sections
meets by its closed form — two lines at one point or, parallel within
`tol.angular`, coincident or apart within `tol.linear`; a line and a
circle at two points, or at the foot of the centre when its distance to
the line is within `tol.linear` of the radius (a touch); two circles at
the two points of their radical line, or at one on the line of centres
when the centres are within `tol.linear` of the sum or the difference of
the radii (a touch) — and the mirror pair of sections meets in the
mirror points bit for bit, so only the half-plane `ρ ≥ 0` is read. A
meeting within `tol.linear` of the axis is a point on it; one beyond is
a circle about the axis of radius `ρ`, crossing or touching as the
sections meet; a point on the axis is a crossing when any two sections
cross there. Two sections that coincide make the pair
`Coincident` when every other meeting lies on a coincident section, as
the apex of two equal coaxial cones does, and `Unsupported` when one
does not — a partial coincidence, not a meeting. `tol.linear` carries over
exactly: a distance in the plane through the axis is the 3D distance
between points at one angle, and a point's distance to a surface of
revolution is its distance to the full symmetric section. Kinds mix in
one `Meets` (ADR-0018): a sphere centred on a cone's axis through its
apex meets it in a circle and the apex. A pair sharing no axis is in
general position: a plane oblique to a cone's axis, or parallel to it
and further than `tol.linear` from it, meets it in a conic (below); two
cones on different axes, a cylinder against a cone off its axis and a
sphere off a cylinder's or a cone's axis meet in a quartic, traced and
fitted (below the tracer); a torus in any of these — a plane oblique to
its axis or parallel to it and off it, a cylinder, a cone or a sphere
off its axis, a second torus on another axis — meets it in a section
traced in the torus's parameter plane (below the torus tracer,
ADR-0019).

**A plane against a cone off its axis** is exact. In the plane's own
coordinates — `x` along `e₁`, the cone's axis projected onto the plane,
`y` along `e₂ = n × e₁`, from the apex's foot `A − h·n` — the cone is
`(x cos φ − h n·Z)² = cos²α (x² + y² + h²)` for `φ` the angle between
the axis and the plane, so with `k = sin(α − φ) sin(α + φ)` the section
is `k (x − x₀)² − cos²α y² = h² cos²α sin²α / k`. Steeper than the cone
(`φ > α`) it is an ellipse, `Curve::Ellipse` with `Z` the plane's normal
and `X` along `e₁`, the direction of increasing `v`; as steep within
`tol.angular` a parabola, one quadratic Bézier over `y`; shallower a
hyperbola's two branches, the `+e₁` one first, each running with `y` as
rational quadratic arcs of at most `HYPERBOLA_HALF_SPAN` (1) in its own
`u` of `(a cosh u, b sinh u)`, joined at double knots so no arc's middle
weight passes `cosh 1`. A parabola or a branch covers every point of it
in the sphere about `within`, and one that sphere misses is left out: no
`Curve` variant carries them, and they are exact, never fitted. Through
the apex within `tol.linear` the plane meets it at the apex alone, a
crossing, where it is steeper; along one touching ruling `e₁` from the
apex where as steep; and in two crossing rulings from the apex, ordered
along `e₂`, negative first, where shallower.

**A plane through the axis** — its normal perpendicular to a cone's or a
torus's axis within `tol.angular`, the carrier's origin within
`tol.linear` of it — cuts the meridian itself, both halves, each
crossing: a cone in the two rulings through its apex, a torus in its
two tube circles. These are a partial revolve's flat ends against its
cone and torus faces. A sphere's is the meridian arm's (a plane through
its centre), a cylinder's the plane–cylinder rulings.

An intersection curve's frame is Arris's own deterministic choice,
matching Open CASCADE only where the *surface's* parametrisation is
concerned: a circle on a cylinder takes the cylinder's `X` so the seam is
shared; a plane–cylinder ellipse's `Z` is the plane's normal and its `X`
the major axis in the direction of increasing `v`; a line's origin is its
point nearest the cylinder's origin (plane–cylinder), the first
cylinder's origin (cylinder–cylinder) or the first plane's origin
(plane–plane), and a ruling's direction is the (first) cylinder's `Z`.
Two parallel cylinders' rulings are ordered by their offset along
`Z × ŵ`, `ŵ` the unit vector from the first axis toward the second,
negative first; a tangent ruling is the first cylinder's, `R₁` along `ŵ`
(along `−ŵ` for an inside touch within the larger second). Two crossing
cylinders of radius `R`, axes `a` and `b` with `b` flipped so that
`a · b ≥ 0` and `ψ` the angle between them, meet in ellipses centred at
the midpoint of the axes' nearest points: the first with `Z` along
`a − b`, `X` along `a + b` and major radius `R / sin(ψ/2)`, the second
with `Z` along `a + b`, `X` along `a − b` and major radius `R / cos(ψ/2)`,
both with minor radius `R` along `a × b`, each `Z` and `X` signed so its
largest-magnitude component is positive (the lower index on a tie) —
and the two ellipses cross each other at `±R` along `a × b`, off the
plane of the axes. Swapping two crossing operands gives the same
ellipses bit for bit, so a boolean's pcurve fits do not depend on the
order; swapping two parallel ones gives the same point sets, up to a
line's orientation and the order of two rulings. A circle of the
meridian arm is centred on the axis with the frame of the first operand
whose frame carries the axis — a cylinder, a cone or a torus always, a
sphere when its own `Z` is parallel to the axis — so its seam shares a
plane with that surface's; when neither carries it, a plane against a
sphere takes the plane's frame moved to the sphere's centre, and two
spheres take `Z` from the first centre toward the second with `X` the
first sphere's `X` or `Y`, whichever has the larger component across
that line. The circles come ascending along the frame's `Z`, and so do
the points of a `Meets`; swapping the operands gives the same
point sets, and reverses the order when the second operand's axis points
the other way. A plane through the axis, with normal `n`, orders its two
curves by the side `w = Z × n` of the carrier's own `Z`, `+w` first: a
cone's ruling starts at the apex and runs along the cone's `∂P/∂v` on its
side, so its `t` is the cone's `v` less the apex's; a torus's tube circle
is centred at `O + R·w` (then `O − R·w`) with `X` the side and `Y` the
torus's `Z`, so its `t` is the torus's `v`. Neither depends on the
operand order.

`intersect_curve_surface(c, s, tol)` returns `CurveSurfaceIntersection::{
Points(Vec<CurveSurfaceHit>), Coincident}` for the pairs with a closed form
— line–plane, line–cylinder, line–elliptic cylinder, conic–plane and
conic–cylinder, *conic* being a circle or an ellipse, a line against a
cone, a sphere or a torus, and a conic in a plane across an elliptic
cylinder's axis — for a conic against a cone, a sphere or an elliptic
cylinder in any other plane and against a torus (below), for a `Nurbs` curve against every
analytic surface (below), and `GeomError::Unsupported` naming the pair
for the one arm left without a form: any curve against a `Nurbs`
surface. A hit is
`CurveSurfaceHit { t, uv, point, tangent }`: `point` is the curve's point
at `t`, `uv` the surface's own projection of it — except at a cone's
apex, whose projection is `Ambiguous`, where it is `u = 0` and the apex's
`v = −R / sin α`, as a sphere's pole takes `u = 0` from the projection —
hits ascending by `t` with a periodic `t` in `[0, 2π)`, a periodic
`Nurbs`'s in `[knots[p], knots[n])`. A line is parallel to a plane or to a
cylinder's axis within `tol.angular`, and then coincident or clear within
`tol.linear`; a circle is `Coincident` when it lies within `tol.linear`
of the surface everywhere, which the extrema of its distance decide. A
hit is `tangent` where the distance along the curve has an extremum
within `tol.linear` of zero — the two crossings such an extremum would
split into are one touch — so a transversal hit is on both operands to
rounding and a tangent one within `tol.linear`. Conic–cylinder finds the
extrema of the radial distance through the quartic in `tan(t/2)` and the
crossings between them by bracketed Newton. Line–sphere is the closed
form of the nearest approach to the centre: two hits, one tangent where
that approach is within `tol.linear` of the radius, or none. Line–cone
and line–torus walk the exact signed distance along the line — to the
cone `ρ cos α − |h| sin α`, `h` the height above the apex; to the torus
`√((ρ − R)² + z²) − r` — which is monotone between its extrema and
kinks: for the cone the point nearest the axis, the crossing of the
plane through the apex and the two stationary points of its smooth
pieces, in closed form; for the torus the roots of the quartic its
squared stationary condition is, in `arris_math::roots`. An extremum
within `tol.linear` of zero is a touch (a run of them with nothing
between, one touch), and each other stretch or tail whose ends differ in
sign holds one crossing, by bracketed Newton on the distance. A line
through the apex at the half-angle within `tol.linear` and `tol.angular`
is a ruling, `Coincident`; any other line through the apex touches the
cone there. The rest are closed forms.
An ellipse is not a separate case anywhere here: a conic reaches `a`
along its frame's `X` and `b` along its `Y`, which is the radius twice
for a circle, and neither closed form assumes the two are equal — so the
oblique section edge a boolean puts on a cylinder wall is tested against
a third face by the same arms. A line against an **elliptic cylinder**
(ADR-0014) is the classifier's ray: a line along the axis within
`tol.angular` is `Coincident` when its section point is within
`tol.linear` of the section and clear otherwise; the section reaches `M
= √((a n·X)² + (b n·Y)²)` along the line's normal `n` in the section, a
line whose offset is within `tol.linear` of `M` touches there, one
farther misses, and one nearer crosses twice at the roots of the
quadratic in the frame that scales the section to the unit circle. A
conic in a plane across the axis within `tol.angular` meets the surface
where it meets the section — the two-conic form above, the conic's own
parameter kept — as `Coincident` or the touches and crossings as hits.

**A conic against a cone, a sphere, or an elliptic cylinder in any other
plane** is what a boolean asks of an operand face's own circle, or of
the section edge a fillet or a chamfer left, against the face it is cut
by (ADR-0020). A quadric's polynomial `F` along `c + cos t·u + sin t·v`
is the trigonometric polynomial `a₁ cos t + b₁ sin t + a₂ cos 2t + b₂
sin 2t + c₀`, its linear part the first harmonic and its quadratic part
the second with half the constant; between two extrema of it — the roots
of its derivative, the same quartic in `tan(t/2)` — `F` is monotone, so
the signed distance, whose sign it carries, crosses zero at most once
there. Beside those go the distance's kinks: for a cone the crossings of
the plane through the apex, and the axis, which is a minimum of the
distance from it. Where `F` is *constant* along the conic — a conic
concentric with and similar to the surface's section, the one shape
whose derivative says nothing — the extrema of the distance from the
axis are looked at instead, which are where such a conic is nearest the
surface and farthest from it; there is no crossing to find, the sign
never changing. What is decided there is decided on the distance, by the
stops the NURBS arm below has: `Coincident` for a parallel of a cone or
a sphere and for an oblique section lying on an elliptic cylinder, one
`tangent` hit at a stop within `tol.linear` absorbing the crossings
beside it, and one crossing on each other stretch whose ends differ in
sign. A conic through a cone's **apex** touches it there — the distance
has its extremum, of value zero, at a point the conic does not cross —
and takes the apex's `uv`.

**A conic against a torus** has no such trigonometric polynomial to
solve, the torus's `F` being quartic: the conic goes in as the four
rational quadratic quarter arcs a turn is exactly made of (`geom`'s
`arc`), and each one put into `F` is a polynomial of degree eight in
Bernstein form with the torus's sign along it — the substitution and the
isolation the NURBS arm below makes of a span, over a conic's exact
quarters, with the quarters' own ends beside the sign changes of the
derivative, since an extremum at a join is a change of sign neither side
sees. The verdict is the same one, on the same distance: `Coincident`
for a conic lying on the torus — a parallel, a tube circle, a Villarceau
circle — one `tangent` hit at a stop within `tol.linear`, and up to
eight crossings, each hit's `t` the conic's own angle.

**A `Curve::Nurbs` against an analytic surface** (ADR-0018) is what the
next boolean asks of a fitted section edge. Every analytic surface is
the zero set of a polynomial `F` in its own frame — `z`; `x² + y² − R²`;
`(x/a)² + (y/b)² − 1`; `cos²α (x² + y²) − sin²α h²` with `h` the height
above the apex, both nappes; `x² + y² + z² − R²`; `(x² + y² + z² + R² −
r²)² − 4R²(x² + y²)` — and a rational span `A(s) / w(s)` of degree `p`
put into it and cleared of its denominator, `g = wᵈ F(A / w)`, is a
polynomial of degree `d·p` with `F`'s sign along the span. `g` is built
in **Bernstein form** from the span's homogeneous Bézier points (the
spline's blossom at the span's ends, which reads a periodic curve's
unclamped knots as any others) by products in that basis, never through
power coefficients. It decides only where to look: the sign changes of
`g′`, isolated by halving on the variation of the coefficients' signs
and polished in the bracket, are the extrema of `g`, and between two of
them `g` crosses zero at most once. A coefficient within `64ε` of the
magnitude of the terms it was summed from is zero, and a stretch whose
coefficients all are is looked at in its middle. What is decided there
is decided in length, as the arms above decide it, on the exact signed
distance `δ(t)` the line arms walk: of those parameters and the curve's
distinct knots, the **stops** are where `δ` is an extremum among its
neighbours, and the two ends of a curve that is not periodic. Every stop
within `tol.linear` is `Coincident` — a fitted section curve is, with
both of its surfaces; a stop within `tol.linear` is a hit that absorbs
the crossings beside it, a run of such stops one hit at the one nearest
the surface; two consecutive other stops of opposite sign hold one
crossing, by bracketed Newton on `δ`, so a transversal hit is on the
surface to rounding. A hit at a stop is `tangent` when its run holds an
extremum inside the curve: a curve that only **ends** within
`tol.linear` of the surface meets it there and is no touch — that is a
section edge ending on a face, which makes a vertex. A closed curve that
is not periodic — a clamped B-spline whose two ends are one point to
rounding, as a file's closed curve is and no curve the kernel makes is —
goes round like a periodic one over its domain's length: its end is its
start, read past it by wrapping, so a surface met at the join is one hit
there, at the start parameter, not one at each end. A span of degree 25
against a torus is a polynomial of degree 100, and is held to the line
arm's hits by the property tests.

`intersect_curves(a, b, tol)` returns `CurveIntersection::{
Points(Vec<CurveCurveHit>), Coincident}`, a hit being `CurveCurveHit {
ta, tb, point, tangent }` with `point` the *first* curve's point at `ta`
and the second's within `tol.linear` of it, hits ascending by `ta`, a
periodic parameter in `[0, 2π)`. Two lines are the closed form —
parallel within `tol.angular` gives `Coincident` or nothing by the
distance between them, and otherwise the nearest approach is a hit when
it is shorter than `tol.linear`. Every other pair of a line or a conic has a
conic operand and goes through *that conic's plane*: the other curve's hits on
the plane are the candidates, and a candidate is a hit when the conic's
own projection of it is within `tol.linear`, which gives `tb` with it. A
curve the plane reports `Coincident` with is the coplanar case, where the
plane decides nothing: a line against a coplanar conic is that conic
against the plane through the line perpendicular to the conic's — the
same points, the tangency decided in `tol.linear` by an arm that already
exists — and two coplanar circles are the radical line. A coplanar pair
with an ellipse in it is the quartic of `geom`'s `conic2`, in the second
conic's plane: the residual `F₂(E₁(t))`, the second's implicit form
along the first, is a trigonometric polynomial of degree two again, so
each extremum of it where the first's point is within `tol.linear` of
the second is a touch, every extremum a touch is `Coincident` — the edge
a boolean made and the edge a second boolean meets it with — and each
arc between two extrema that are not touches whose ends differ in sign
holds one crossing. A residual with no extremum is constant, a conic
concentric with and similar to the other, and is `Coincident` or empty
by the distance.
A **`Nurbs` curve** (ADR-0018) — the fitted section edge the next
boolean meets — goes through the same planes, its hits on them from the
NURBS arm of `intersect_curve_surface` above. Against a circle or an
ellipse, through the conic's plane; one in that plane meets the conic
where it meets the cylinder the conic is the section of, circular or
elliptic, in the conic's frame and of its radii, so a touch is decided
in `tol.linear` of length by that arm, and a curve on the cylinder too is
`Coincident`. Against a line, through the two planes that hold the line
and are square to each other (the line's `Frame::from_z` gives their
normals): the hits on either within `tol.linear` of the line are the
candidates, those within `tol.linear` of each other, or joined by a
stretch of the curve that stays within it of the line, one hit — at a
shallow angle each plane finds the crossing where the curve's own
offset across it vanishes, the two that offset over the angle's tangent
apart, which can be past `tol.linear` — a crossing
of either plane before a touch, since a curve is tangent to a line only
where it touches every plane through it, then the one nearest the line —
a curve in one plane is the coplanar case, the other plane's hits the
answer, and a curve in both is `Coincident`. A crossing of the line is
transversal to one of the two planes unless the curve runs along the
line there, so it is found to rounding. `Nurbs`–`Nurbs` is `Unsupported`:
where two fitted curves of one pair meet is the tracer's `points`
(§Curves, below), and anything else a marcher's (the NURBS cycle's).
`curves_coincide(a, b, tol) -> Result<bool, GeomError>` is that
`Coincident` verdict alone, by the same arms, so it answers every pair
the intersector answers. Two `Nurbs` curves coincide when they are
the same spline, every control point within `tol.linear` of its twin
over the same degree, knots and weights — the same section made twice —
and do not when a point of either at an end, a knot or halfway between
two has no hit of the other on the plane square to it there within
`tol.linear` (a projection's sampling could settle on a farther local
minimum; the plane's NURBS arm cannot). Any other two — one curve over
two knot vectors — are `Unsupported` naming the pair.

`trace_quadrics(a, b, within, tol) -> Result<SectionTrace, GeomError>`
is the exact section of two quadrics, one of them ruled — a cylinder, an
elliptic cylinder or a cone against any of those or a sphere, in any
pose — before any fit; the intersector calls it for every such pair
that meets in no conic (the fit below). The ruled operand is
walked as a family of lines by its own angle `s`; a ruling meets the
other quadric where `a(s)w² + b(s)w + c(s) = 0`, and with a cone's
rulings taken from its apex the discriminant is a trigonometric
polynomial of degree two for every ruled kind. Where the number of hits
changes is therefore the roots of one quartic in `tan(s/2)`: the branch
structure is algebraic, nothing is sampled, and no loop can fall between
two samples. A `SectionBranch` is one smooth callable over its own
`domain()`, closed or open — two roots over the same interval are joined
where the section turns back, through a parameter that removes the
square root — lying on the walked surface to rounding and on the other
within `tol.linear`. A critical point of the discriminant where the other
surface is within `tol.linear` of touching the ruling, the discriminant's
value read as a distance (`|D| / 4|a||∇q|`), is a **singular point**
(`SectionPoint`): the value is taken out by a bump reaching twice the
angle at which it would have vanished, so a near miss, a gap and a loop
the tolerance cannot tell from a point are all the one point they are at
that tolerance, branches through it end at it *exactly*
(`BranchEnd::Singular`), and one that no branch reaches is `isolated`, a
touch. That decision is a closed bound, so there is no pose the tracer
guesses at; the poses it refuses are named
(`GeomError::DegenerateSection` with a `SectionFault`): surfaces tangent
along a curve and a ruling lying on the other surface, both a closed
form's, and three turning points within one tolerance, a cusp. Because a
section may run to infinity — two cones, a cylinder along a cone's
ruling — every branch is clipped to the extent of `within` along the
walked rulings (`BranchEnd::Clipped`), again at the roots of a quartic;
everything inside `within` is returned and a loop inside it stays
closed. Which operand is walked is a rule on the two surfaces — parallel
rulings before a cone's, the smaller radius or narrower cone first, then
a circular section before an elliptic one, then the frames coordinate
by coordinate — so swapping the arguments changes
nothing, bit for bit.

`trace_torus(a, b, tol) -> Result<SectionTrace, GeomError>` is the exact
section of a torus with a plane, a cylinder, an elliptic cylinder, a
cone, a sphere or another torus, in any pose, before any fit (ADR-0019).
There is no ruling to walk, so the torus's own parametrisation goes into
the other surface's implicit polynomial: `f(u, v) = 0` over sixteen
quarter-turn patches, each a tensor Bernstein polynomial in the
half-angle chart of its quarter, of bidegree (2, 2) against a plane,
(4, 4) against a quadric and (8, 8) against a torus. Every component of
the section either turns in `u` or winds round the torus and crosses
`u = 0`, so the turning points — the common zeros of `(f, f_v)`, isolated
by subdivision and each certified alone in its box — and the isolated
roots of `f(0, ·)` seed all of them: the branch structure is proven, not
sampled, as the ruled tracer's is. A branch is graphs `v(u)` joined
through their turning points by the same parameter, each stretch proven
in a cell of its own on the polynomial's coefficients before its root is
followed. Within a thousandth of a radian of a turning point a graph is
walked in `v`, not in `u` — `v` from the square root of the offset, `u`
the root along that `v`, well conditioned where the root along `u` moves
by `√ε` for a rounding `ε` — so two arms meet at the turn in one point
and the branch is continuous through it, as the ruled tracer's anchored
discriminant makes its own; every point of it is on the torus **exactly** — `uv(t)` is its
(u, v) there, unwrapped across both seams and `None` for a ruled branch —
and on the other surface to rounding. A critical point of the other
surface's distance over the torus that is within `tol.linear` of zero is
a **singular point**, taken out by a bump in the Hessian's own metric
(ADR-0019) so that the branches through it end at it exactly; an
isolated one is a touch. A tube circle of the torus that lies on the
other surface within `tol.linear` — a pipe elbow against its pipe, a
sphere or a cone about the tangent to the centre circle — is a factor of
the polynomial, and comes back as one of `SectionTrace::circles`: an
exact `Curve::Circle` on the torus whose `t` is the torus's `v`,
`tangent` where the two do not cross along it, with the rest of the
section traced on the quotient and cut at the circle. A torus is
compact, so `within` is no part of this and every branch is returned
whole. Which of two tori is walked is a rule on the two surfaces — the
smaller over all (`R + r`), then the smaller tube, then the frames
coordinate by coordinate — so swapping the arguments changes nothing,
bit for bit. The poses it refuses are named (`SectionFault`, ADR-0019)
and of measure zero: two tube circles with more of the section besides,
a circle not held to the tolerance wherever the rest is traced, turning
points `f64` does not tell apart, a continuum of them, and a singular
point crowded by another, by a turning point or by a tube circle.

**A section that is no conic is a fitted `Curve::Nurbs`** (ADR-0018);
the exact form is never stored. Each branch of the trace is fitted at
its own parameter by `fit_curve`, or `fit_curve_periodic` when it is a
loop — so a closed section is one periodic B-spline with no joint, which
the pave model cuts at its paves only — of degree `SECTION_FIT_DEGREE`
(5, measured: the fewest control points where two cylinders meet at a
small angle, the longest fits there are, and the pcurves' degree),
until the fitted point is nowhere farther than `SECTION_FIT_FRACTION` (a
quarter) of `tol.linear` from the exact branch at the same parameter —
from the stretch of the line the branch's root was found along, a
ruling or a tube circle, on which the other surface's value vanishes in
`f64` (`SectionBranch::distance`): the branch's own precision, rounding
wherever that line crosses the other surface at an angle, and up to
`3·10⁻⁶` along the section where it runs a hair from tangent, which the
walked angle's last digit then moves the root by.
That bounds each surface's distance too — a projection's distance is
1-Lipschitz, so the fit is no farther from either surface than the
branch is, which is rounding away from a singular point's reach, plus
that quarter — and it holds what the two surfaces alone do not: where
they meet at an angle `θ`, a point `ε` off both can be `ε / sin(θ/2)`
across from the section. So the edge's tube holds the true section at
every meeting angle, and two fits of one section — traced in two
regions, or by two operations — are within half of `tol.linear` of
each other (ADR-0022, amending ADR-0018 and ADR-0019). The fraction is a
quarter because each face's pcurve is fitted to the 3D curve
afterwards, can come no nearer it than it is to the face, and is
accepted at half of `tol.linear` (§Tolerances). The
branches keep the tracer's order, orientation and parameter, which
depend on the two surfaces alone, so swapping the operands gives the
result bit for bit; each is a `Crossing`, and each singular point is a
point of the result — a `Touch` where it is isolated, a `Crossing` where
branches end at it, the one case where a point of a `Meets` lies on its
curves. The region bounds only what runs to infinity: the closed forms
ignore it, and a caller intersecting several pairs on the same two
surfaces passes one region to all, so they get one curve — a boolean
the overlap of its operands' boxes grown by its own diagonal, S5 the
overlap of the two faces' boxes; a torus section ignores it. Every pair
of analytic surfaces is decided in every pose, the spiric sections and
the interlocked tori among them; the tracers' refusals (`SectionFault`)
remain, poses of measure zero — two cones sharing an apex or a ruling, a
cusp of the section, a torus pose of ADR-0019's list — and S5 lists
those pairs as unchecked. A tube circle of a torus section is the one
curve of a `Meets` that is exact and still only within `tol.linear` of
the other surface, the tolerance it was accepted in, as a point of a
`Meets` is; every other conic is a closed form and lies on both surfaces
to rounding. The Villarceau circles of a bitangent plane are not one of
them: they come back as the fitted arms the tracer finds through that
pose's two singular points (ADR-0019).

### Pcurves (`Curve2`)

```rust
pub enum Curve2 {
    Line    { origin: Point2, direction: UnitVec2 },
    Circle  { frame: Frame2, radius: f64 },
    Ellipse { frame: Frame2, major_radius: f64, minor_radius: f64 },
    Nurbs   (NurbsCurve2),
}
```

A pcurve is a curve in a surface's (u, v) plane, with the parametrisations
of the 3D table: `O + t·D`, `O + R(c t·X + s t·Y)`, `O + a c t·X + b s
t·Y`. A circle or an ellipse is placed by a `Frame2`, whose handedness is
the direction of traversal: right-handed is counter-clockwise in (u, v),
left-handed clockwise. That is what a `center`-and-radius circle could not
say, and a 3D circle shared by a cap and a wall *is* clockwise on the one
of the two planes whose normal opposes the circle's `Z` — the pcurve keeps
the edge's parameter and the frame records the turn, so the curve is never
reversed (§Orientation). `Curve2::eval(t)` returns `Curve2Eval { point,
d1, d2 }`; `domain()`, `period()` and `kind()` (`Curve2Kind`) follow the
table; `project(p)` returns `Curve2Projection { t, point, distance }` by
the closed forms of the 3D variants in the plane, `GeomError::AmbiguousUv`
at a circle's centre and on an ellipse's ambiguous loci, and by sampling
and bracketed Newton for a NURBS.

`pcurve_on(curve, range, surface, tol)` builds the pcurve, exhaustively
over (curve, surface), and its image under the surface is the curve *at
the same parameter*. On a plane every 3D curve lying in it has an exact
pcurve: a line is a `Line`, a circle a `Circle` and an ellipse an
`Ellipse` whose `Frame2` is right-handed when the curve's `Z` is along
the plane's normal and left-handed when it opposes it, a NURBS a `Nurbs`
with its control points projected (an affine map, so knots and weights
carry over). A line or a conic tilted from the plane by more than
`tol.angular` lies within `tol.linear` of it over the range asked alone
— a piece of an operand edge along a section block on the other face —
and its projection there is not its own shape at its own parameter (a
tilted circle projects to an ellipse), so the projection is fitted, held
to it within `tol.linear` in the plane: off the curve by the curve's own
distance from the plane and no more than that again. On a
cylinder, a circle around the axis is a `Line` at constant v whose `u`
starts at the offset of the circle's `X` from the cylinder's and runs in
the sense of the circle's `Z` against the cylinder's, a line along the
axis is a `Line` at constant u, and an oblique plane section (a 3D
ellipse), or a NURBS, is a sinusoid in (u, v) — not a `Curve2` variant, so
it is fitted (below). The rule: exact where a variant exists, fitted
otherwise, and in both cases the checker verifies the pcurve against the
3D curve (§Invariants E4). A curve farther than `tol.linear` from the
surface at any of `PCURVE_SAMPLES + 1` parameters over the range is
`GeomError::NotOnSurface` naming the parameter and the distance.

On the surfaces of revolution the exact arms are the six a revolve makes,
each a `Line` in (u, v): on a **cone**, a ruling — the line through the
apex — at constant `u`, and a circle about the axis at constant `v`; on a
**sphere**, a circle about the axis at constant `v` (a parallel) and the
great circle through both poles at constant `u` (a meridian); on a
**torus**, a circle about the axis at constant `v` and a circle of the
tube at constant `u`. A `u` origin is the offset of the circle's `X` from
the surface's, in `[0, 2π)`, running in the sense of the circle's `Z`
against the surface's, as on the cylinder — and `u + π` for a cone's
circle beyond the apex, whose radial factor `R + v sin α` is negative. A
constant-`u` arm's `v` runs with `t` or against it by the turn of the
circle's own axes in the plane of the axis, and a meridian's `v` leaves
`[−π/2, π/2]` where the great circle passes a pole onto the opposite
meridian, which is where the sphere's parametrisation puts it. On an
**elliptic cylinder** (ADR-0014) the exact arms are the two an extrude
makes, each a `Line`: a ruling at constant `u`, the parameter of its
section point, `v` running with `t` or against it by the line's
direction against `Z`; and the section ellipse — centred on the axis,
its `Z` along the axis, its major axis along the surface's `X` either
way, the radii agreeing within `tol.linear` — at constant `v`, `u`
starting at `0` or `π` by its `X` against the surface's and running in
the sense of its `Z` against the surface's, as a parallel does on a
cylinder. On a **NURBS surface** every curve is fitted (below).

**The fitted fallback is one, for every surface.** Every other
curve on a cylinder, an elliptic cylinder, a cone, a sphere or a torus —
an oblique section, a small circle about no axis of the sphere, a
Villarceau circle, a traced quartic, any NURBS — and every curve on a
NURBS surface is a `Nurbs` fitted by
`fit_curve2` (below) over the surface's own projection of the curve
(`Surface::project`), held to the curve in 3D. Each periodic parameter —
`u`, and on a torus `v` as well, and on a NURBS surface each direction
with a closure, periodic or closed — is unwrapped along `t`, so a seam
crossing stays continuous and the parameter may leave `[0, 2π)` (the
domain, on a NURBS surface, whose evaluation wraps it back). On a NURBS
surface the curve is on the surface by the global projection's
distance, a start on a seam reads at the knots' start, and a seam's
second use is placed a period along by the caller, as on the analytic
surfaces; the fit's projections start from the unwrapping table's
neighbour by Newton's iteration and fall back to the global search where
that lands farther than the singular band from the curve — the search is
global per call, and a fit asks thousands. The
unwrapping reads a table of `PCURVE_SAMPLES` parameters and halves an
interval over which a parameter swings by a quarter turn, up to
thirty-two times, before it refuses the curve as winding faster than it
resolves: that is what `u` does beside a pole or an apex — by `π` over a
stretch as long as the miss — and the fit follows by halving its spans
there. Measured on a unit sphere at the default tolerance, a small circle
passing a pole fits at every miss from the band below to a hundredth of
the radius: 917 control points just outside the band, 901 at one
tolerance, 767 at `1e-5`, 479 at `1e-3`, 293 at `1e-2`, a quarter of
`MAX_FIT_SPANS` at the worst and 10 to 110 ms. The projection is the only
source of a torus's pcurve, too: the tracer's exact (u, v)
(`SectionBranch::uv`) is the branch's, and the curve an edge carries is
the 3D fit of it, within `SECTION_FIT_FRACTION` of a tolerance of the
branch at the same parameter; the projected pcurve follows the fitted
curve, where the branch's own (u, v) would be off the edge's curve by
the fit's whole deviation.

**A fitted pcurve never runs through a singular point** of the surface —
a cone's apex, a sphere's pole, a NURBS surface's collapsed row — where
every `u` names one point (on a NURBS surface, every value of whichever
parameter runs along the row). The
decision is a distance: a curve within `PCURVE_SINGULAR_BAND` (a quarter)
of `tol.linear` of the point is on it. A range with that inside it is
`GeomError::ThroughSingularity { curve, surface, t }`, `t` the parameter
of the nearest approach, and the caller splits there; a range that *ends*
there is fitted, and its pcurve ends on the point's own `v` with the `u`
the curve arrives with, the limit along it read from its tangent — on
a surface of revolution as far along it as the curve's middle, where
every distance reads the same, and on a NURBS surface just clear of the
fade below, before the surface bends away from the tangent. The
band is a quarter because a pcurve that ends on the point is off the
curve there by the curve's own miss, which no refinement removes, and the
fit accepts half the tolerance: a quarter leaves the fit the other
quarter, so both sides of a split always fit — and it is how near its
branch, and so its surfaces, a fitted section is held
(`SECTION_FIT_FRACTION`). Within one
tolerance of such an end the curve is carried onto the point, fading out
by four, so that `u` is read as the curve's own end sees it and not as
the point does, to which the miss, however small, is a right angle. A
range may end on the one point twice — a circle through a pole, cut
there, which is what a boolean makes of it — and each end is read on its
own side of the range, half a turn of `u` apart. The exact arms are as
they were: a ruling keeps its one `u` through the apex onto the other
nappe, a meridian its one `u` over a pole — at the sphere's own latitudes
for the range asked, `v = t − π` for the half circle by way of `t = π`
and not the `t + π` the circle's phase alone gives, since a sphere's `v`
is no period of anything and no caller could put a whole turn of it back.
A curve that changes a cone's nappe *beside* the apex — farther
than the band from it and within the tolerance of the surface, which
only a very flat cone has room for — jumps by `π` in `u` and is refused
as winding.

**What a boolean does there** (ADR-0021). A section within the band of
a face's singular vertex is paved by that operand vertex, so no block has
the point inside it and `ThroughSingularity` is never met from a boolean;
the vertex's degenerate edge is paved in turn at each `u` a section edge
arrives with — the node the face's arrangement needs, one vertex standing
for a whole line of (u, v) — and cut there into degenerate pieces on the
same vertex. On a cone *through* also means straight through: a curve
that only comes within the band of an apex turns back there, its tangent
perpendicular to the axis, where one through it leaves along a ruling.
A section that passes the point outside the band and within four polygon
segments of the face's box diagonal (`diagonal × 4 /
MAX_SEGMENTS_PER_PIECE`) is `Reason::BesideSingularity`: the fit above
follows it, and the face's polygons — cut evenly in the parameter, at most
`MAX_SEGMENTS_PER_PIECE` to a piece — do not, built from a miss of `1e-4`
on a ball of radius 2 and not at `1e-5`, for a circle and a traced loop
alike.

**The (u, v) toolkit** is what every algorithm that reasons about a
face's domain shares — the checker's loop, face and body rows,
tessellation, mass properties and classification — and it lives in
`arris-geom` below all of them (decided in M2). `region2`: a loop is a
sequence of `Piece { curve: &Curve2, range, reversed }` walked in order
— `Model::loop_pieces(&Loop)` is the loop's, so the checker's rows,
tessellation and `measure` all ask once —
and `discretise(pieces, chord_tolerance)` is its `Polygon2` — each piece
sampled at the segment count its second derivative bounds the chord
deviation by (`|d2| h² / 8`; a line is one segment, a conic never fewer
than `MIN_SEGMENTS_PER_TURN` per turn and never fewer than
`MIN_SEGMENTS_PER_ARC` — two — however short the arc, so a loop of one
arc and one line keeps the area of its bulge, a NURBS never fewer than
`MIN_SEGMENTS_PER_SPAN` per knot span, and never more than
`MAX_SEGMENTS_PER_PIECE`, the deviation achieved reported by
`chord_deviation()`; `f64::INFINITY` asks for the minimum counts, enough
for a sign) with the pieces taken as written, so a seam-crossing loop's
`u` runs past the period and is never wrapped; `Polygon2::from_points`
is the same ring from points a caller sampled itself, as tessellation
does at the parameters its 3D edges were discretised at. Over the polygon:
`signed_area()` (shoelace), `winding_number(p)` by `orient2d` crossings
(zero outside, `±1` inside by the turn), `contains(p)` exactly on a
segment, `gaps()` between consecutive pieces (L2), and
`self_intersections()` / `intersections(&other)` over
`segments_intersect`, exact through `orient2d` with touching counted
(L5, S5). `Curve2::speed_bounds(range)` bounds `|du/dt|` and `|dv/dt|`
over a range, exact for a line, the radius (the major radius) for a
conic, and sampled for a NURBS.

`region2::point_side(polygons, p, boundary_tolerance) -> Side::{Inside,
Outside, Boundary}` is where a (u, v) point lies with respect to the
polygons of a face's loops: within the tolerance of any segment is
`Boundary`, and otherwise the sum of the winding numbers decides. The
tolerance is a distance in the parameter plane and is the caller's — the
checker passes the model's parametric tolerance scaled to the surface, a
boolean the face's tolerance converted the same way; nothing in `region2`
knows the model. `region2::SideIndex::new(polygons)` reads a region once
so that `side(p, boundary_tolerance)` is `point_side`'s answer for every
finite point, taken from the segments filed under strips of `v`: the
winding from `p`'s own strip, which holds every segment that could cross
the ray, and the boundary test over the strips the band meets and one
more either side, which absorbs the distance's rounding; `FaceDomain`
holds one per face. `region2::interior_point(polygons, clearance)` is a
point strictly inside the region and further than `clearance` from every
segment: a horizontal is cut by the segments into spans, the spans whose
midpoint has a non-zero winding number are the inside ones, and the
widest of those that clears the segments gives its middle. The height
is the midpoint between two consecutive distinct vertex heights — never
a vertex's own, so the line runs along no segment and through no vertex
— the one nearest the polygons' mid-height first and the next ones
outward when that holds no span at the clearance: deterministic, and
`None` (never a guess) only for a region too thin to hold a point at
that clearance at any of them. A
caller passes the polygons' `chord_deviation` so the point is inside the
*curved* region and not merely inside its polygon; a boolean classifies a
piece there rather than at its centroid, which for a sliver rounds onto
its own boundary (ADR-0004).

`Curve::bounds(range)` and `Surface::bounds([u, v])` are the axis-aligned
box each fills over a parameter range, `None` when a range is not finite.
Exact where the geometry is affine or separable — a line's two endpoints,
a conic's extrema per axis, a plane rectangle's four corners, a
cylinder's sinusoid in `u` and travel in `v` — and an outer bound where
the two parameters multiply (a cone, a sphere, a torus) or where the
geometry is a NURBS, whose control hull over the spans the range touches
contains it. A face's box is the union of its edges' boxes and its
surface's over the loops' (u, v) bounds, inflated by the face's
tolerance; `Aabb::intersects` on two of them is the cheap reject a
boolean's face pairs go through before any intersection is computed.
`Curve2::translated(by)` (and `Frame2::translated`) is the same pcurve
moved in (u, v) with its parameter carried along: how a boolean puts a
section edge's pcurve on a periodic surface into the copy of the domain
the face's loops are written in, a whole number of periods along `u`.
`integrate::region_integral(pieces, grid, f)` is `∬ f du dv` over
the region by Green's theorem — `∮ G dv` with `G = ∫_{u₀}^{u} f ds` —
with Gauss–Legendre quadrature of `GAUSS_ORDER` points per interval, a
conic piece split at quarter turns and a NURBS at its knots — a
periodic one's repeated by whole periods, since a block of a closed
section wraps past the knots' end (`NurbsCurve2::breaks_within`) — signed by
the loop's turn so holes subtract themselves: `f = |∂P/∂u × ∂P/∂v|` is
an area, `f = P · (∂P/∂u × ∂P/∂v) / 3` summed over a solid's faces with
their use orientation is Gauss's volume (B2, `measure`). The *inner*
integral is split at every break of a `Grid`'s `inner` it crosses — the
surface's own quarter turns or knots, repeated by its period, so a
feature of the integrand there is an interval's end and never its
middle — at most `MAX_INNER_INTERVALS` intervals, beyond which it falls
back to that many equal ones: a strip that crosses a whole turn of
`cos u` is not one interval's work, so a caller passes
`integrate::surface_grid(surface)` — quarter turns on the quadrics,
sixteenths on an elliptic cylinder, whose area element `√(a² sin²u + b²
cos²u)` is no trigonometric polynomial and has complex singularities
`atanh(b / a)` off `u = 0` and `π` that a quarter turn resolves to
`1e-7` at an aspect of eighteen and a sixteenth to `1e-11`, a NURBS's
own `u` knots, repeated by its closure — where its derivatives jump at a
knot of multiplicity its degree, which Open CASCADE's converted
quadrics have at every arc's end, and which a grid of its smallest span
anchored at zero put inside an interval, off by `1e-5` of a volume — and
`Grid::NONE` (one interval, exact for a polynomial `f`) on a plane. On
a NURBS the grid's `outer` breaks, its knots in `u` and `v`, also split
each boundary piece where it crosses one, where the boundary integrand
kinks for the same reason.

`project_to_plane(curve, plane)` is the orthogonal projection onto a plane
for a consumer's sketch (architecture §How a consumer's kernel facade maps on): a point-set projection
whose parameter is the variant's own — a line stays a `Line`, a circle
becomes a `Circle` when parallel and an `Ellipse` otherwise (its
semi-axes the singular values of the projected axes, its parameter the
circle's shifted by a phase), an ellipse an `Ellipse`, a NURBS a `Nurbs`
with its control points projected. A projection that collapses to a
point or a segment is `Degenerate`. Every arm is affine in the
parameter, which is how `ops::query::project_to_plane` carries an edge's
range into the projected curve's: a line's `s = |D'| t` with `D'` the
direction's in-plane part, a NURBS's `s = t`, and a conic's `s = t + φ`,
the phase `φ` read off the image at `t = 0` in the image ellipse's own
frame from the projected point's and tangent's local `x`
(`a cos φ` and `−a sin φ`), which involve the major radius alone and so
stay exact for a conic seen nearly edge-on. The carried conic range
starts in `[0, 2π)` and keeps its length. A range copied unshifted would
draw another arc of the same ellipse.

**Fitting** is `fit_curve2(f, range, degree, deviation, tol)`: a global
least-squares B-spline approximation of `f: t ↦ (u, v)` at the *given*
parameter (*The NURBS Book* §9.4.1) — so the result is same-parameter by
construction and interpolates both ends — with the knot vector refined
where the caller's `deviation(t, fitted point)` exceeds `tol` (checked at
`4p + 4` parameters per span, accepted at half the tolerance so the
result meets it between the checks too). The deviation is the caller's
measure in the caller's units: for a pcurve, the 3D distance between the
surface at the fitted point and the true curve. Refinement is bounded by
`MAX_FIT_SPANS` (4096, a budget and not a tolerance: the loop two
metre-scale tori of nearly equal radii share runs 100 to 170 units and
takes 1000 to 1300 spans at degree 5, where the cylinder pairs take
under 200): beyond it the result is `FitError::Diverged`
(`GeomError::Fit`), never a loop. `fit_curve` is the same fit of a 3D
curve `t ↦ P` into a `NurbsCurve`, for the section curves no exact
`Curve` variant carries. `fit_curve_periodic` fits a closed one over
one period `range` with a *periodic* knot vector instead of a clamped
one: `m` spans, `m` free control points, control point `i` being free
point `i mod m`, so the result's `period()` is `range.length()`, its
image is closed and `C^(p−1)` across the seam by construction, and no
point of the loop is singled out as a joint — nothing is interpolated.
Its normal equations are a band with corners (each free point is
coupled to its `p` neighbours cyclically), solved by the same Cholesky
factorisation kept to the matrix's envelope, whose fill stays inside
it. A curve whose start does not reach its end within the fit's margin
is refused as `FitError::Degenerate` before anything is fitted.

### Profiles

A consumer's sketch is a value, not a shape: `geom::profile` holds it and
the sweeps of `arris-ops` read it (`docs/ARCHITECTURE.md` §Operations).

```rust
pub struct Profile { plane: Frame, outer: ProfileLoop, holes: Vec<ProfileLoop> }

pub enum ProfileLoop {
    Circle  { center: Point2, radius: f64 },
    Ellipse { center: Point2, major: Vec2, minor_radius: f64 },
    Path    { start: Point2, segments: Vec<ProfileSegment> },
}

pub enum ProfileSegment {
    LineTo(Point2),
    ArcTo     { to: Point2, via: Point2 },
    EllipseTo { to: Point2, center: Point2, major: Vec2, minor_radius: f64, ccw: bool },
}
```

The loops are drawn in the plane's own (u, v) — `origin + u·X + v·Y` — and
carry no orientation: the grammar is one-to-one with a recipe's `profile`
step (`tests/fixtures/README.md`), the consumer's sketch as it is drawn.
An arc is three points: `via` decides its centre, its radius and which way
round it goes. An elliptic arc (ADR-0014) is given its ellipse — `major`
runs from the centre to a major vertex, so its length is the major radius
and its direction the axis, and `minor_radius` the other — and its turn,
`ccw` counter-clockwise about the plane's normal, since with the centre
and axes given only the direction is left to state; a full ellipse is
the loop variant, never a segment.

`Profile::edges(tol)` is the validation and the orientation in one, and
returns one `Vec<ProfileEdge>` per loop — index `0` the outer, the holes
from `1` — in walking order:

| Check | Error |
|---|---|
| a path loop has at least two segments | `TooFewSegments` |
| a path loop's last segment ends where the loop started, within `tol.linear` | `NotClosed { loop_index, gap }` |
| a segment is longer than `tol.linear` — its length for a line, the distance between its ends for an arc, so an arc back to its own start is refused rather than taken for a full circle | `ShortSegment { loop_index, segment }` |
| an arc's `via` is off its chord by more than `tol.linear` | `DegenerateArc` |
| an ellipse's `major` and `minor_radius` are finite and each above `tol.linear` | `DegenerateEllipse { loop_index, segment }` |
| an elliptic segment's start and `to` each lie within `tol.linear` of its ellipse | `OffEllipse { loop_index, segment, distance }` |
| a loop's mean width — twice its area over its perimeter: the width of a long thin rectangle, the radius of a disc — is above `tol.linear` | `ZeroArea { loop_index }` |
| a loop does not meet itself | `SelfIntersecting { loop_index, segments }` |
| no two loops meet | `Crossing { loops }` |
| every hole is inside the outer loop | `HoleOutside { hole }` |
| no hole is inside another | `NestedHoles { holes }` |

Each loop's structural checks — the first six rows — run before the next
loop's, then each loop's area and self-intersection, then the checks
between loops, so the error reported is the first fault in that order;
each names its loop or loops, and the segment where one is at fault, by
the indices the consumer wrote. The area, self-intersection and containment
checks are made on each loop's polygon at `region2`'s *minimum* segment
counts, whose arcs are their chords. `GeomError` reaches the caller as
`ProfileError::Geometry` — an inconsistent tolerance, and the curve-in-its-
own-plane case that cannot happen, carried rather than unwrapped.

The outer loop comes back counter-clockwise about the plane's normal and
every hole clockwise, reversed from the consumer's order where needed. A
`ProfileEdge` is one segment's 3D `Curve` — a `Line`, a `Circle` whose
`Z` is `±` the plane's normal so the parameter runs from the segment's
start through its `via`, or an `Ellipse` whose `Z` is `±` the normal so
the parameter runs in the segment's turn and whose `X` is the major
axis — its `range`, its exact in-plane `Curve2` (by
`pcurve_on`, so same-parameter and checked), its two endpoints in (u, v),
the `(loop_index, segment)` indices *as the consumer wrote them*, and
`reversed`, which says whether orienting the loop turned it round. A
circle loop is one closed edge over `[0, 2π]` whose one vertex sits at
`center + radius · plane.x`, where the oracle's `gp_Circ` on the plane's
`Ax2` puts it; an ellipse loop's sits at `center + a·X`, the end of the
major axis, where `gp_Elips` puts it. An ellipse is normalised before it
becomes an edge: a `minor_radius` longer than `major` names the same
point set, and the edge's axes are swapped with the frame turned a
quarter turn so `a ≥ b`; radii that agree within `tol.linear` give a
`Circle` edge of their mean radius through the same ends, which deviates
from the ellipse by at most half their difference, so a near-circular
section is always a cylinder with every arm a cylinder has. An elliptic
arc's ends are the ellipse's nearest points to the consumer's, within
`tol.linear`, and a reversed walk turns its `ccw`.

`Profile::area_and_centroid(tol)` is the region's area and (u, v) centroid
by `integrate::region_integral` over those oriented edges, so the holes
subtract themselves.

### NURBS

`NurbsCurve`, `NurbsCurve2` and `NurbsSurface` follow *The NURBS Book*:
degree `p` in `1..=MAX_DEGREE` (25, Open CASCADE's bound, which also
sizes the evaluator's stack buffers so evaluation never allocates), a
non-decreasing knot vector of `n + p + 1` knots, `n` Cartesian control
points with their positive weights stored separately, de Boor evaluation
with derivatives to second order (the quotient rule over the homogeneous
sums), and knot insertion as the primitive edit (degree elevation is in
the backlog). The constructors validate and return
`GeomError::Degenerate` naming the fault: a knot value's multiplicity is
at most `p + 1`, and at most `p` strictly inside the domain
`[knots[p], knots[n]]`, which is non-empty and whose last span is not,
and a span that is not empty is at least `f64::MIN_POSITIVE`, since every
derivative divides by it. Two knots a rounding apart at an ordinary scale
are valid; the kernel's own splits and fits make them. A knot vector need not be clamped. A curve or a surface direction is
**periodic** exactly when its structure wraps: the knots repeat `n − p`
places on shifted by the domain's length and the last `p` control points
(rows, for a surface) repeat the first `p`, both to rounding; `period()`
is then the domain's length and evaluation wraps the parameter into the
domain first. Any other parameter outside the domain evaluates the
nearest polynomial piece. Knot insertion (`insert_knot(t, times)`)
leaves the image over the domain unchanged; on a periodic curve it
breaks the wrap the knots implied, so the result's `period()` is `None`.
`NurbsCurve::segment(range)` is the curve over exactly `range`, clamped
there, in the same parameter — the range's ends raised to multiplicity
`p` by insertion and the control points between kept; a periodic curve
takes any range of at most one period wherever it starts, read on the
curve unrolled over two periods, so an edge's block past the knots' end
is a piece like any other (the STEP writer's, architecture §Formats and
tools).

**Exact forms.** What a file calls a conic, an extrusion or a revolution,
and what an analytic surface is when it must be a `Nurbs`, has an exact
rational form — the same point set to rounding, never a fit
(ADR-0025 §1). `NurbsCurve::circle` and `ellipse` are rational quadratic
arcs of at most a quarter turn each, with weights `1, cos(h), 1` for the
arc's half-angle `h`; the parameter is the angle at every arc end and a
monotone reparametrisation between (a rational arc has no angle
parametrisation). `parabola` is the quadratic polynomial it is, in the
parameter of ISO 10303-42, so `eval(t)` is the parabola's own point;
`hyperbola` is one rational Bézier arc, the homogeneous form
`(a(q² + 1)/2, b(q² − 1)/2, q)` in `q = eᵗ` being quadratic. Every angle
range is **clamped**, and a full turn is therefore *closed* — its first
and last control points coincide — and not periodic: a periodic rational
circle needs its knots doubled at each arc end, and a domain that starts
on a doubled knot ends on one, which the constructor refuses. That is
also what a closed B-spline in a file is, and why the seam handling of
`pcurve_on` onto a NURBS surface treats a
closed direction that is not periodic: `NurbsSurface::closure()` is the
domain's length there, evaluation wraps a parameter outside the domain
by it, and `Surface::period()` reports it, so a seam on a closed
direction is a seam like a periodic one's. `NurbsSurface::extrusion(curve, d,
range)` is `C(u) + v·d`, degree one in `v`, whose parameter is the
distance along `d`. `NurbsSurface::revolution(curve, origin, axis,
angle)` is the tensor product of the curve with the angle's arcs: `u` is
the angle (counter-clockwise about the axis, from the curve as placed) and
`v` the curve's own parameter, the order of every quadric. A control
point on the axis makes a **collapsed row** exactly — a pole or an apex —
decided by `is_negligible` against the coordinates that formed it, never
by a tolerance. `Surface::to_nurbs(bounds)` gives the part of any
analytic surface over a parameter rectangle in these forms (a plane a
bilinear patch; a cylinder and an elliptic cylinder the extrusion of a
conic; a cone, a sphere and a torus the revolution of a line or a conic),
with the parameter linear exactly where the analytic one is linear and
the angle at every arc end otherwise.

## Topology

### Entities

Five arena entity kinds. Loops and coedges live inside the face that owns
them, because nothing outside a face refers to them: provenance, iteration
and a consumer's topological references name vertices, edges and faces.
The entity structs below live in `arris_topo::entity`; the crate root
holds the *handles* of the same five names (`Body`, `Shell`, `Face`,
`Edge`, `Vertex`: id plus orientation, architecture §The model), since
a consumer holds handles far more often than it reads an entity.

```rust
pub struct Vertex { point: Point3, tolerance: f64 }

pub struct Edge {
    geometry: EdgeGeometry,                 // Curve { curve: CurveId, range: Interval } | Degenerate { range: Interval }
    start: VertexId, end: VertexId,         // equal on a closed or degenerate edge
    tolerance: f64,
}

pub struct Face {
    surface: SurfaceId,
    loops: Vec<Loop>,
    tolerance: f64,
}
pub struct Loop   { coedges: Vec<Coedge> }  // ordered, closed
pub struct Coedge { edge: EdgeId, orientation: Orientation, pcurve: Curve2Id }

pub struct Shell { faces: Vec<Face> }        // the handle: FaceId + Orientation

pub struct Body {
    kind: BodyKind,                          // Solid | Sheet | Wire | General
    shells: Vec<Shell>,                      // handles
    free_edges: Vec<Edge>,                   // handles; wire and general bodies
    free_vertices: Vec<VertexId>,            // general bodies
}
```

A reference that carries an orientation — a shell's face use, a body's
shell or free-edge use — is stored as the handle of that kind, since a
handle *is* an id and an orientation. The fields are private: an entity is
built by its constructor (`Vertex::new`, `Edge::new`, …, which check
nothing) and read through getters, and once in the arena it is never
written. The arena appends it through the raw insert (`Model::raw()`,
test scaffolding that stores a dangling reference as given), the builder
(§Euler operators) or `import`.

- A **vertex** is a point and a tolerance.
- An **edge** is a bounded piece of a 3D curve between two vertices, oriented
  by its curve's parameter direction. `range` is a sub-interval of the
  curve's domain; on a periodic curve it may cross the period (`[3π/2,
  5π/2]`). A **degenerate edge** has no 3D curve: both vertices are the same
  vertex at a surface singularity (a sphere's pole, a cone's apex) and it
  exists only to give the face's loop a pcurve across the singularity; it
  carries the parameter range of its pcurves itself, since there is no
  curve to take one from (`Edge::range()` is the range of either kind).
  It is used by the one face that closes on it, never as a boundary
  between two (S2), and a revolve makes one per face at a cone's apex or
  a sphere's pole on its axis.
- A **face** is a surface trimmed by one or more loops. The face's natural
  normal is its surface's normal.
- A **loop** is a closed ring of coedges. A **coedge** is one use of an edge
  by one loop: the edge, the direction it is traversed in relative to the
  edge's own, and the pcurve for that use. A loop keeps the face's material
  on the *left* when walked in coedge order with the face's natural normal
  up — outer loops counter-clockwise in (u, v), holes clockwise. There is no
  outer/inner flag; that orientation rule and the winding it implies are the
  whole distinction, and on a periodic surface it is the winding number
  through the period that decides.
- A **shell** is a set of face uses. A shell of a solid body is closed and
  its effective face normals point out of the material.
- A **body** is what operations take and return. `Solid`: every shell
  closed, every edge used by exactly two coedges, no edge or vertex used by
  two shells, and the shells nesting into *lumps* — a lump an outer shell,
  enclosing positive volume, with the void shells whose innermost
  container it is (B1) — so one solid holds a cavity, the two halves of a
  split, disjoint pieces, and a piece inside another's cavity (ADR-0006).
  A lump is derived, never stored: `arris_check::lumps(&Model, Body)`
  returns them, and an operation stores a body's shells lump by lump, the
  outer shell first. `Sheet`: open shells
  allowed, every edge used by one or two coedges, a face's effective normal
  is the sheet's front. `Wire`: no faces, only free edges. `General`: any
  mix, including a face used by two shells (a face separating two regions
  of one body) and an edge used by more than two coedges.
  Non-manifold structure is thus representable from day one (`SEED.md`
  §9); every operation today produces and accepts `Solid` only: the builder's
  `finish` builds no other kind (`BuildError::Kind`), and `measure`
  refuses one as `OpError::Degenerate` with `Reason::NotSolid`.

### Orientation

A handle is an id and an orientation, and every reference from an entity to
a sub-entity carries one (a body's shell uses, a shell's face uses, a
coedge's edge use). Orientation composes by XOR along the path from the
handle down to the entity, and every rule in this document is stated in
terms of the *effective* orientation at the end of that path:

- effective face normal = surface normal, flipped if the composed
  orientation down to the face use is `Reversed`;
- effective edge direction = curve direction, flipped by the composed
  orientation down to the coedge;
- a loop of a `Reversed` face is walked backwards, which keeps the material
  on the left of the flipped normal — the convention survives composition.

Entities themselves are never oriented: a surface is never flipped to make
a face's normal point outward, a curve is never reversed to make a coedge
forward. A face used `Reversed` by a shell is the same face, in the same
arena slot, that another shell may use `Forward` from the other side.

### Seams and closed faces

A face on a periodic surface whose loop crosses the seam contains the seam
as an edge used twice by the same loop, once `Forward` and once `Reversed`,
with two pcurves that differ by the period in the periodic parameter (`u =
0` and `u = 2π` on a cylinder). A full cylinder wall is one face, one loop of
four coedges: bottom circle, seam up, top circle, seam down. The seam edge is
an ordinary edge with an ordinary 3D curve; only its two pcurves know it is
a seam. This is the representation truck lacks and every seam-crossing
algorithm quietly needs: tessellation samples the seam once and the wall's
loop polygon carries the two copies a period apart, so the wall's
triangles use the one run of indices twice and the mesh closes across the
seam by construction (architecture §Tessellation).

### Euler operators

Entities are immutable and Euler operators mutate; the two meet in
`arris_topo::Builder`. A builder holds one body under construction as a
staging area of vertices, edges and faces in tombstoned slots, edited by
Mäntylä's ten operators (*An Introduction to Solid Modeling*, ch. 9,
adapted to coedges and seams; ADR-0002) and frozen into the arena by
`finish(&mut Model, BodyKind) -> Result<Built, BuildError>`, which
appends every live slot in order inside a transaction and returns the
body with the slot → id maps an operation's provenance is built from. The
builder is the only way an operation makes topology; the raw insert and
`import` are the other two paths into the arena, and neither is an
operation's.

| Operator | Makes / kills | Inverse |
|---|---|---|
| `mvfs(Seed)` | the first vertex, a face with one loop of no coedges at it, the shell | `kvfs()` |
| `mev(at, Strut)` | a vertex and the edge to it, used twice in a row by the loop at `at` — `Forward` away, `Reversed` back | `kev(edge)` |
| `mef(from, to, Split)` | an edge between two junctions of one loop and the face on its left: the edge `Forward` then the coedges from `to` around to `from`; the old loop keeps the rest after the edge `Reversed` | `kef(edge)` |
| `mekr(from, to, Join)` | an edge between two loops of one face, joining them | `kemr(edge)` |
| `kfmrh(kill, into)` | removes a one-loop face on `into`'s surface with the opposite orientation; its loop becomes a ring of `into`; genus + 1 | `mfkrh(face, ring)` |

Every operator keeps `V − E + F − (L − F) − 2(S − G) = 0` (`counts()`;
`G` is the builder's own count of handles), and every operator followed by
its inverse restores the builder byte for byte (`dump()`): a kill leaves a
tombstone the next make of that kind fills, most recently freed first,
and loops are kept canonical — rotated to start at their lowest `(edge,
orientation)` use, ordered within a face by that key, a loop without
coedges first by its vertex — so the state is a function of the content
alone. A kill returns the record its make takes, so undoing is a call.

Positions, not vertices: an operator's place in a loop is `Position {
face, loop_index, coedge_index }`, the junction before that coedge and
the effective start vertex of it, because a vertex may stand at several
junctions of one loop (a seam's, a closed edge's) and only the junction
says which. `coedge_index` runs `0..=len`, `len` being the junction after
the last coedge — the same vertex as `0`, the same insertion point for
`mev`, and for `mef` the split that moves every coedge (`from = to +
len`, from any junction) as opposed to none (`from = to`), which is how a
closed edge splits off a cap on either side. `find_position(face, loop,
vertex)` answers the unambiguous case and names every junction otherwise.

Orientations inside the builder are *effective*: a use is walked as seen
from outside the material with the face's outward normal up, and each
face carries the orientation the shell will use it with (`Seed`,
`Split`). `finish` stores a `Reversed` face's loop backwards with every
use flipped, so every stored loop is counter-clockwise about its
surface's normal (§Orientation) — the cylinder's bottom cap, used
`Reversed`, stores its circle `Forward`. `kfmrh` needs the two faces on
one `SurfaceId` with opposite orientations: two coplanar faces with
opposing outward normals and nothing between them, which is what the
floor of a pocket reaching the bottom face is, and the loop moves with
its pcurves.

Geometry is explicit and the builder never computes any of it: `mev`
takes the new vertex's point, the edge's `CurveId` and range and the
*two* pcurves of its two uses on the current face — a seam is exactly a
strut whose two pcurves differ by the period, and only the caller knows
which use it is drawing; `mef` takes the curve, the range, the new face's
`SurfaceId` and one pcurve per side; every pcurve is `Option` and
`set_pcurve(position, id)` gives or replaces one, since a coedge `mef`
moves to a face on another surface keeps a pcurve id that is no longer
its own, and a strut in a face it will leave has none worth giving.
`finish` refuses a coedge without a pcurve, a loop without coedges, an
edge not used exactly twice (a degenerate edge: exactly once, since no
surface closes on one singular point twice) or twice the same way, a geometry id that
does not resolve, and any kind but `Solid` — each a typed `BuildError`,
and the model exactly as it was — and never evaluates geometry: the
checker, above this crate, is where the finished body is proven.

**Assembly, and kept ids.** An operation that computes its result's faces
outright rather than reaching them by a sequence of edits — a boolean, a
sweep, a transform — enters the builder through `assemble(&Model, tolerance, Assembly) ->
Result<(Builder, AssemblySlots), BuildError>` instead (ADR-0004). An
`Assembly` is a list of `VertexSpec`s, a list of `EdgeSpec`s and the
body's shells, each a list of `FaceSpec`s, every spec `Keep` (an entity
the model already holds) or `New`, with
`VertexKey`/`EdgeKey` naming either an arena id or a position in the
list; a `New` face's loops are `UseSpec`s in effective orientation, as
the operators take them. A `Keep` face is kept whole — its loops,
pcurves, edges and vertices are the model's, and the edges and vertices
are kept with it. `AssemblySlots { vertices, edges, faces }` gives back
the slot `assemble` assigned to each spec, in spec order — `faces` shell
by shell — so a caller that holds a slot per entity of its own operands
looks an output up as `built.vertices[&slots.vertices[i]]` rather than
zipping the builder's slot order against its own list, which an
`Assembly` interleaving `Keep` and `New` specs need not agree with.

`builder::effective_uses(face_orientation, uses)` walks a loop's coedges
between *stored* order (the loop's own, on its own surface) and
*effective* order (as seen from outside the material, each orientation
composed with `face_orientation`, the whole walk reversed when it is
`Reversed`) — composing an orientation with itself and reversing a list
are each their own inverse, so one function walks both ways.
`FaceSpec::from_face(model, face_use, edge_key)` reads a face whole into
a `FaceSpec::New` through it — the surface, the tolerance, every loop's
uses in effective order, each edge named through the caller's own
`edge_key`. `finish`, `keep_face` and a boolean's own assembly walk
their loops through `effective_uses` directly. `Assembly::of_body(model, body,
remap: &mut impl GeometryRemap) -> Result<(Assembly, BodyIndex),
NotFound>` builds on it to describe a whole body shell by shell with
every entity `New`, its curves, surfaces and vertex points named through
`remap` — the identity, `KeepGeometry`, for a caller that wants the same
geometry; `transform`'s own remap moves each by its motion. `BodyIndex`
maps every vertex, edge, shell and face (its shell and spec index) of the
body read to its spec index in the `Assembly` — what provenance is built
from, once `assemble` returns the matching `AssemblySlots`; `transform`
is `of_body` plus this remap, its own work reduced to the geometry move
alone.

A kept slot *is* the arena's entity: `finish` appends nothing for it and
returns its id, so a face an operation did not touch keeps its `FaceId`
and its provenance records nothing (§Provenance). Structural sharing and
the kept/modified distinction are that one rule. Any operator applied to
a kept slot drops the mark — `canonicalise` and `set_pcurve` clear it —
and `finish` appends that slot instead, so the two entry points compose;
`Builder::dump` writes ` kept f3` on a slot that still carries a mark.
`assemble` proves what the operators would have kept true of each shell:
every loop has coedges and closes through effective vertices, every edge
is used exactly twice and in opposite directions — a degenerate edge exactly
once, the singular point of its face, which no Euler line counts — no arena entity is kept
twice, no shell is empty, no edge is used by and no vertex is an end of
edges of two shells, the faces of each shell are one edge-connected
component, and each shell's Euler–Poincaré line closes at a whole genus,
their sum becoming the builder's — each failure a typed `BuildError`
(`LoopOpen`, `EdgeUses`, `SameDirection`, `Duplicate`, `Empty`,
`EmptyShell`, `SharedEdge`, `SharedVertex`, `Disconnected`, `NotClosed`,
`EmptyLoop`, `NoSpec`, `NotFound`), and the model is only read. How the shells nest is
not the builder's to prove: it is B1's. `finish` appends one shell per
shell of the assembly, in order, and `Built::shells` lists them; a face an
operator makes out of another belongs to that face's shell, so a builder
of operators makes one.

### Adjacency and iteration

The arena keeps derived indices, maintained on every append because
entities are immutable: edge → coedges (`CoedgeRef { face, loop_index,
coedge_index }`, the address of a coedge, since loops and coedges are not
entities), vertex → edges (each edge once, a closed edge included), face →
shells. The queries — `Model::edge_uses(edge)`, `vertex_edges(vertex)`,
`face_shells(face)` — are model-wide, in creation order of the referencing
entity, and `NotFound` for an id that does not resolve; an entity shared
by two bodies lists both bodies' uses, and a per-body question filters
through the closure. A reference that does not resolve when its entity is
appended (a raw insert with a dangling id) is not indexed: M1 is where it
is reported. The indices are one value shared by every clone of a model
and copied whole on the first append after a clone, so `Model::clone`
stays O(chunks) and the copy is paid once, by the clone that diverges.
After `retain`, and when a model is read from the native format, they
are rebuilt from the entities in slot order — the same lists, in creation
order for a model that never freed a slot.

`Model::shells(body)`, `faces(body)`, `edges(body)`, `vertices(body)`
iterate in a deterministic order — depth-first over the body's shells,
faces, loops and coedges in stored order, then the free edges, then the
free vertices; each entity once at first visit, so a seam edge appears
once and a face used by two shells appears under the first. Every handle
yielded carries its *effective* orientation, composed by XOR from the
body handle down the path it was first reached by (§Orientation); a
vertex's is the orientation of the edge use that reached it — its
effective start first, then its end — which means nothing geometrically
and is there so every handle composes alike. A reference that does not
resolve is skipped; only the body handle itself is `NotFound`.
`closure(body)` is the same reach as sorted, duplicate-free id lists per
kind, geometry included: what the checker, `import`, `retain` and the
text dump walk. That order is the order tessellation numbers `FaceRange`s
in. Provenance does not follow it: an origin's outputs come in **split
order** (§Provenance), the order the operation added them.

## Tolerances

An entity's tolerance `t` says: the true geometry this entity stands for lies
within distance `t` of the stored geometry — a ball around a vertex's point,
a tube around an edge's curve, a slab around a face's surface. It is a
statement about *this* entity, not about the model, so two edges of one face
may carry different tolerances.

- **Ordering.** For every incidence, `vertex.tolerance ≥ edge.tolerance ≥
  face.tolerance`: an edge's tube contains its vertices' balls in the sense
  that the edge's ends are within the vertices' tolerances, and a face's slab
  is at least as tight as any edge in it. This is the Open CASCADE ordering
  and it is what makes "is this point on this edge" answerable with the
  edge's tolerance alone.
- **Growth.** An operation never emits an entity with a tolerance smaller
  than that of the input entity it was `Modified` from, and it raises a
  tolerance only for a reason it can name: an intersection whose curves
  agree only to `t`, a vertex merged from two points `t` apart — the
  section vertices of a boolean being the connected components of its
  candidate points, two the same when their balls meet, so a vertex's
  spread is the component's and not an accident of the order its points
  were found in (`docs/ARCHITECTURE.md` §Operations) — and a section
  edge whose pcurve is moved to end on such a vertex's own (u, v) on a
  face, where its members lie further apart than the face's tolerance
  and no exact curve meets them all, by the move. The record of
  why lives in the operation's tests, not in the entity. A fitted section
  curve is *not* such a reason (ADR-0018, ADR-0022): it lies within
  `SECTION_FIT_FRACTION` of the faces' tolerance of the exact section at
  its own parameter — and so of both surfaces, and of any other fit of
  the same section within twice that — and each pcurve fitted to it
  within that tolerance, so its edge's tube holds the true section and
  the edge carries its faces' tolerance as an edge on a closed-form curve
  does.
- **Read from a file**, tolerances are measured, never taken from the
  file's global uncertainty (ADR-0025 §4). The STEP reader rebuilds every
  pcurve, fitting one at the gap where an edge curve lies off its face's
  surface — the fit's tolerance a search step that may pass the cap, the
  gap its pcurve leaves what the cap judges — and where two pcurves of a loop end apart in (u, v) past L2's
  band it ends them on one point (`pcurve_ending_on`) — the seam's end
  where one is a seam, else the vertex's own (u, v), as a boolean ends a
  section edge. What the file leaves implicit and Arris's face needs is
  rebuilt: an edge running through a pole or an apex is split there, two
  loops each wrapping a period of the surface are joined by a seam along
  an isocurve (an edge split where the seam must meet it), and one such
  loop is joined to the singular point on its face's side, as a
  `VERTEX_LOOP` is. Each edge then carries the largest distance of its
  pcurves' images from its curve — sampled at the checker's samples and
  at the `PCURVE_SAMPLES` a fit is held to, each peak past the default
  climbed to its top between them, since a fitted pcurve strays furthest
  between samples and a finer look, the mesh's, finds it there (and a
  closed edge its curve's own gap), each vertex the largest distance from its
  point to the curves' ends and the pcurves' images there and the span of
  a degenerate edge's image, each face the default; every value is
  floored at `default_tolerance` and raised to keep the ordering. A gap
  past `READ_GAP_FRACTION` of the part's size, or past `max_tolerance`,
  is refused (`Refusal::Gap`): closing it is sewing.
- **`Precision`** is the model-wide configuration set at `Model::new`:
  `default_tolerance` (what primitives get), `min_tolerance` (the floor no
  entity goes below), `max_tolerance` (an operation that would exceed it
  returns `OpError::Tolerance`), `angular_tolerance` (radians, for
  parallel/tangent decisions), `parametric_tolerance` (how far a pcurve
  may deviate in (u, v) at unit parametric speed; the checker scales it by
  the surface's parametric derivative, so on a cylinder of radius `r` the
  bound in `u` is it over `r`), and `check_samples` (how many parameters the checker samples
  along an edge). Default values are chosen for a model whose features are
  of order 1–1000 units.
- **No literals.** A tolerance in an algorithm is the entity's, or a field
  of `Precision`, or a named constant in `arris-math` with a comment. `1e-6`
  in an algorithm is a bug (`.agents/rules/kernel.md`).

Exact predicates (`robust`) decide combinatorial questions — which side of a
2D segment a point lies on, whether a triangle is oriented — on the stored
coordinates; tolerances decide whether two things are *the same*. The two
never mix: a predicate is never softened by a tolerance, and a tolerance
comparison never pretends to be exact.

## Invariants

The list `arris-check` enforces. Each item is a `Violation` variant carrying
the entity (and, where relevant, the parameter or the second entity) and has
a test that constructs the violation through the raw insert API and sees it
reported. The level says when it runs (architecture §The checker). The
list at least covers Open CASCADE's `BRepCheck` statuses (read in the
reference tree) mapped onto this representation.

**Model and references**

| # | Invariant | Level |
|---|---|---|
| M1 | Every id referenced by an entity of the body resolves in this model, with the stored generation | Fast |
| M2 | Every reference a parent in the body makes is in the adjacency index of the entity it names — a coedge in `edge_uses` of its edge, an edge in `vertex_edges` of its vertices, a shell in `face_shells` of its faces. A reference that did not resolve when its parent was appended (a raw insert naming a later id) is never indexed; every other row reads adjacency off the closure itself, so this is the only row about the indices | Fast |
| M3 | Every coordinate, parameter and tolerance is finite | Fast |

**Vertex**

| # | Invariant | Level |
|---|---|---|
| V1 | `Precision::min_tolerance ≤ tolerance ≤ max_tolerance` | Fast |
| V2 | For every incident edge, the edge's curve at the end of its range is within the vertex's tolerance of the vertex's point | Fast |
| V3 | For every face the vertex lies on (through any coedge), the surface at the pcurve's end is within the vertex's tolerance of the point | Fast |

**Edge**

| # | Invariant | Level |
|---|---|---|
| E1 | A non-degenerate edge has a curve and a non-empty range inside the curve's domain (crossing the period at most once) | Fast |
| E2 | `start == end` exactly when the curve returns to its start over the range within the edge's tolerance: a closed curve names one vertex, an open one two. The geometric match of each end to its vertex is V2's | Fast |
| E3 | Every edge in a body is used by at least one coedge, or is a free edge of a wire/general body | Fast |
| E4 | For every coedge, the surface evaluated along the pcurve is within the edge's tolerance of the 3D curve at the same parameter, at `Precision::check_samples` parameters including both ends — the pcurve and the curve share the edge's parameter (same-parameter, same-range, always) | Fast |
| E5 | `edge.tolerance ≥ face.tolerance` for every face it bounds; `≤ vertex.tolerance` of both vertices | Fast |
| E6 | A degenerate edge has `start == end` and lies on faces whose surface is singular along its pcurve over its range (the image at `check_samples` parameters spans at most the vertex's tolerance) | Fast |
| E7 | A seam edge (used twice by one loop) has its two coedges in opposite orientation and pcurves that differ by exactly the surface's period in the periodic parameter | Fast |
| E8 | The edge does not self-intersect within its range. An analytic curve over a range E1 accepted cannot; a NURBS is tested as a polyline of `check_samples` points per knot span, two non-adjacent segments closer than the edge's tolerance being the crossing when more than that tolerance of polyline runs between them (around a closed edge, either way) — nearer, they are one stretch of the curve, as a periodic section edge's few units in the last place past its knots' end are | Full |

**Loop and face**

| # | Invariant | Level |
|---|---|---|
| L1 | A loop has at least one coedge and is closed: coedge *i*'s effective end vertex is coedge *i+1*'s effective start vertex, cyclically. Reported once per loop, at the first junction that breaks | Fast |
| L2 | The pcurves are continuous in (u, v) at every coedge junction within `parametric_tolerance` scaled to the surface's speed, or jump by exactly one period in a periodic parameter — across a seam edge, and where a closed edge's pcurve wraps the parameter once. Every junction that is neither is reported | Fast |
| L3 | No edge is used twice in one loop except as a seam (E7); no edge is used by two loops of the same face except as a seam | Fast |
| L4 | Each loop's signed area in (u, v) is non-zero — its mean width, the area over half its perimeter, is above `parametric_tolerance` — and the loops of a face have exactly one outer loop (positive winding) per connected component of the domain, holes with negative winding inside one of them. Two outer loops are one component exactly when one contains the other; disjoint ones are two | Fast |
| L5 | The loops of a face do not intersect each other or themselves in (u, v), as polygons within `parametric_tolerance` of the pcurves | Full |
| F1 | The face has a surface and at least one loop; every pcurve lies within the surface's non-periodic domain bounds, to `parametric_tolerance`. Reported once per face, at the first coedge that leaves them | Fast |
| F2 | `face.tolerance ≥ Precision::min_tolerance` and ≤ every incident edge's | Fast |

**Shell and body**

| # | Invariant | Level |
|---|---|---|
| S1 | Every face use in a shell resolves and no face is used twice by one shell | Fast |
| S2 | In a `Solid` body every non-degenerate edge of the shell is used by exactly two coedges, with opposite effective orientation (the two faces agree on which side the material is); in a `Sheet` by one or two; in `General` by any number. A degenerate edge is a singular point of a surface, not a boundary between two faces — a sphere's pole is used once by the one face that closes on it — so it is not counted here. The orientations pair up in every kind: as many forward uses as reversed, but for an odd count, where exactly one is left over. A `Wire` body's shell is not judged here — B3 says it should have none | Fast |
| S3 | A shell is connected through its edges | Fast |
| S4 | A shell of a `Solid` is closed: no non-degenerate edge with one coedge (S2's exemption) | Fast |
| S5 | The faces of a shell intersect only along their shared edges and vertices: their surfaces' intersection is empty, or every point of it that is interior to both faces is within the tolerance of an edge or vertex they share; two coincident surfaces must not carry faces whose interiors overlap. Surfaces meeting in isolated points (the points of a `Meets`: a touch, or a crossing through an apex) are held to the same rule point by point, and every curve is held to it whether it crosses or touches — a point inside both faces that is not within tolerance of a vertex both faces reach, or of an edge they share, is a violation; the vertex clause covers two cones closing on one apex, or a blend sphere touching a plane at the corner of its contact lines, which share a vertex but no edge. A pair whose boxes — each face's edges' curve boxes and its surface's box over its loops, grown by the tolerances — are apart shares no point and is decided without an intersector; every pair of analytic surfaces is decided in every pose, over its exact conics, over the curves a ruled pair's tracer fits in the overlap of the two faces' boxes (ADR-0018) — two blend cylinders on skew axes, a corner's sphere against a blend cylinder off its centre, two cylinders a boolean left meeting in a loop — or over the section a torus's tracer walks in its own parameter plane and fits whole, that overlap ignored (ADR-0019): a pin through the tube of a ring, a bend of pipe tangent to the straight one it runs into along a tube circle, a torus and a torus interlocked. What the intersector does not decide — a `Nurbs` in the pair, and the two tracers' refusals, poses of measure zero (`SectionFault`) — is **unchecked**: listed by `Report::unchecked`, never passed and never a violation | Full |
| B1 | A `Solid` body has at least one shell, and its shells nest into lumps (ADR-0006): every shell enclosing positive volume is an outer shell and every one enclosing negative volume — its effective normals turned into the void — a void, a shell enclosing none being neither; no face of one shell meets a face of another, by S5's test with nothing shared; and, one shell lying inside another when a vertex of it does by the parity of a ray cast against the other's faces alone, each void's innermost container is an outer shell and each outer shell's is none or a void. `arris_check::lumps` returns the lumps this proves — each outer shell with the voids whose innermost container it is. A face pair of two shells S5's test leaves undecided — a `Nurbs` in the pair, a tracer's refusal — or a shell no ray could be classified against, is **unchecked**; every pair of analytic surfaces is decided as S5 decides it, a ruled pair's traced section over the overlap of the two faces' boxes and a torus's over the whole torus. A ray meets every analytic surface by closed form, and one whose hit lands at a cone's apex or a sphere's pole lands on that face's degenerate edge, a boundary, and is abandoned for the next direction — so a solid of analytic faces — plane, circular or elliptic cylinder, cone, sphere and torus — is decided in every pose (ADR-0008, ADR-0019) | Full |
| B2 | A `Solid` body encloses positive volume: `∬ p · (r_u × r_v) / 3` over each face's region in (u, v), summed with the sign of each face use. The value is reported with the violation | Full |
| B3 | A `Wire` body has no shells; `free_edges` form chains (each vertex used by at most two free edges) — `General` bodies exempt | Fast |

**Euler–Poincaré** (every level, reported as one line, never a violation on
its own): `V − E + F − (L − F) − 2(S − G) = 0` with `L` the number of loops
and `S` the number of shells. `E` leaves degenerate edges out: a cone's apex
or a sphere's pole is a singular point of the surface, not a boundary
between faces (S2's exemption), and counting it would give a sphere genus 1
and a cone an odd line; the oracle leaves out the edges Open CASCADE marks
degenerate, so both sides count alike. The line is one type,
`arris_topo::euler::EulerLine`: the checker's report and the dump take it
with `EulerLine::of` over a closure, and `Builder::counts`, `assemble`'s
per-shell test and the fixture lint build it with `EulerLine::new` from
their own counts; a builder counts no degenerate edge either, and `finish` refuses one not used exactly
once. The genus `G` is *derived* from the counts,
as the oracle derives it, so the line cannot fail on its genus; what it
checks is its parity — a count set that leaves a residual of one cannot
come from any closed orientable surface, whatever its genus. `Report::euler`
carries the line, `arris_debug::dump_text` prints it, and every fixture
asserts it. The line is linear in the closure, so it is taken at `Fast` too.

## Provenance

Every operation returns a `Provenance`: which output entities came from
which origins, and how. Three relations, in Open CASCADE's
`BRepTools_History` vocabulary (read in the reference tree), because they
are the three a parametric history needs; and an origin is an input
entity *or a role* — what an entity is to the operation that made it from
nothing — so that every chain has a root (ADR-0002):

```rust
pub enum Relation { Generated, Modified, Deleted }
pub enum Origin   { Entity(Shape), Role(Role) }
pub enum Role     { Box(BoxPart), Cylinder(CylinderPart), Extrude(SweepPart), Revolve(SweepPart), File(FileEntity), Consumer(ConsumerKey) }   // exhaustive
pub struct FileEntity { id: u64, instance: u32 }   // a file's #id, and which placement of it
pub struct ConsumerKey { namespace: u32, key: u64 }   // the consumer's own name, opaque (ADR-0028)

pub struct Provenance {
    generated: BTreeMap<Origin, Vec<Shape>>,   // origin → outputs generated from it, in split order
    modified:  BTreeMap<Origin, Vec<Shape>>,   // origin → outputs that are pieces of it, in split order
    deleted:   BTreeSet<Shape>,
}
```

- **Generated**: the output is a new entity of a *different* kind or role
  built from the origin — the wall of a hole from the tool's cylindrical
  face, an intersection edge from a pair of faces (one record per face;
  `generated_pair(a, b)` is their intersection), the side faces of a
  sweep from the profile's segments, every entity of a primitive from its
  role (`Role::Box(BoxPart::Face(Coord::Z, Side::Max))` is a box's top;
  `BoxPart::Edge { along, sides }` and `BoxPart::Vertex([Side; 3])` name
  the rest; `CylinderPart::{Wall, BottomCap, TopCap, BottomRim, TopRim,
  Seam, BottomVertex, TopVertex}` a cylinder's; both have `Shell` and
  `Body`). A sweep's entities are `Generated` from a `SweepPart` naming
  the part of the consumer's sketch (§Profiles) each came from, with the
  consumer's own indices: `StartCap` and `EndCap` from the profile face;
  `Side`, `StartEdge` and `EndEdge { loop_index, segment }` from one
  segment; `Rise`, `StartVertex` and `EndVertex { loop_index, vertex }`
  from one vertex, `vertex` the index of the segment that starts there
  (a circle loop has segment `0` and vertex `0`); `Shell` (the outer
  shell) and `Body`; and `Cavity { loop_index, segment }`, a void shell
  of a full revolve (ADR-0006) — a hole's (`segment` `0`), or a notch's
  closed from a chain of segments running off the axis and back, named
  by the lowest index the consumer wrote among its segments, so two
  notches of one loop are two names. A
  full revolve has no `EndCap`, `EndEdge` or `EndVertex` — its start
  edges are the seams — and in one a segment perpendicular to the axis,
  which sweeps an annulus of two closed rises, has no `StartEdge` at all.
  A revolve's vertex on the axis sweeps no rise and has no
  `EndVertex`; its `Rise` names instead the degenerate edge of each face
  closing there at a cone's apex or a sphere's pole, and in a full turn
  it has a `StartVertex` only where such a face keeps it; a line
  segment along the axis sweeps no `Side` and has no `EndEdge`, its
  `StartEdge` being in a partial turn the edge both flat ends share and
  nothing in a full turn. A solid a reader reads (ADR-0025) has every
  entity `Generated` from its own file entity, `Role::File(FileEntity {
  id, instance })` — the `#id` and which placement of it an assembly
  makes: the body from its `MANIFOLD_SOLID_BREP` or `BREP_WITH_VOIDS`,
  each shell from its `CLOSED_SHELL`, each face from its
  `ADVANCED_FACE`, each edge from its `EDGE_CURVE` and each vertex from
  its `VERTEX_POINT`. A vertex the reader makes is `Generated` from what
  made it: one where an edge is split — at a pole, an apex or a seam's
  end — from that edge's `EDGE_CURVE`, and the singular point a wrapping
  loop is joined to from its face's `ADVANCED_FACE`. An edge the reader
  rebuilds, which the file has no entity for — a degenerate edge the
  writer left out, or the seam joining a `VERTEX_LOOP` to its face's
  other bound — is `Generated` from its face's entity, and an edge split
  at a pole, an apex or a seam from its `EDGE_CURVE`. A solid a consumer
  builds itself — its own Euler operators or `Builder::assemble` —
  and finishes through `ops::build` has every entity `Generated` from
  `Role::Consumer(ConsumerKey { namespace, key })`, the key the
  consumer's `BuildKeys` gave that entity's slot (ADR-0028). The kernel
  never reads a key: keys need not be unique, so two slots under one key
  are two outputs of one role in slot order, and one key may name
  entities of different kinds.
- **Modified**: the output is a trimmed, split or re-tolerated piece of the
  input, same kind — the box's top face with a circle cut out of it, each
  half of a face split by an intersection curve (one input, several
  outputs), a transformed face.
- **Deleted**: the input has no image of its own kind in the output — the
  part of the tool inside the target, a face swallowed by a fuse.
- **Kept** is not recorded: an entity untouched by the operation keeps its
  id and is simply present in the output body. `Provenance::is_kept(input,
  &model, output_body)` is a query, not a relation.

Every entity of every input body is accounted for: it is kept, or it is
recorded — `Modified` into pieces, `Generated` from, `Deleted`, or both
`Deleted` and `Generated` from (the tool face that is gone and whose
image is the hole's wall); never both `Deleted` and `Modified`, since a
piece is an image; and every entity of the output is a kept input or has
an origin. `arris_topo::provenance::audit(model, inputs, output,
&provenance) -> Result<(), AuditError>` is that rule, once: the corpus
runner and the ops property tests call it on every fixture and every
random case, and that the relations are the same on every run.

A boolean writes its record from the pieces as it makes them
(ADR-0004). An entity of an operand whose ids are reused is kept when
nothing at it changed; a face whose loops changed at all — a split edge,
a section edge, a re-tolerated vertex — is `Modified` into its
surviving pieces, a split edge `Modified` into its surviving sub-edges,
a vertex a section vertex re-tolerated `Modified` into the new one, and
whatever has no piece left is `Deleted`. Where the two operands share
an entity within tolerance the result holds it once, from the first
operand: a vertex of B merged into a section vertex that a vertex of A
stands for is `Modified` into A's, and a piece of an edge of B that is
a piece of an edge of A (a common block of a coincident face pair) is
`Modified` into A's piece; a piece of a face of A lying on a coincident
face of B and kept by the normals is `Modified` from A's face and
`Generated` from B's, B's face `Deleted`. A section vertex that is no
operand's vertex is `Generated` from the edge and the face of every hit
it merges, and from both edges of every crossing (from both faces of
the pair for a section crossing — two section curves of one pair
crossing each other — and for a closed section curve no hit paves); a section edge is
`Generated` from both faces of its pair, so `generated_pair(wall, cap)`
is the hole's rim. The tool of a `cut` keeps nothing: every entity of it
is `Deleted`, and a piece of it that survives — the hole's wall from the
tool's wall, the floor of a blind hole from the tool's cap, whole or not
— is a new entity `Generated` from the tool entity it is a piece of, so
no entity is shared between the tool body and the result. A result
shell — the surviving pieces that share edges, several of them being the
lumps and voids of ADR-0006 — is `Modified` from every shell of a
kept-by-id operand (the target of a `cut`, either operand of a `fuse` or
a `common`) a piece of it came from, and one made of a cut tool's pieces
alone, a cavity, is `Generated` from the tool's shell; a shell of a
kept-by-id operand no result shell came from is `Deleted`. The result's
body is `Modified` from the target's in `cut`, from both operands' in
`fuse` and `common`.

A blend writes its record against the blended edge, with no `Role` of
its own (ADR-0007): the blend face, its two contact edges, its two end
arcs and the four vertices where the arcs meet the contacts are
`Generated` from the edge; each of the edge's two faces, each face
across an end and each corner edge the trim shortens is `Modified` into
its new self; the edge and the two corner vertices it consumes are
`Deleted`; the shell and the body are `Modified` one-to-one, and every
other entity of the body is kept by id. Two blends that share a face
modify it once, into the face rewritten by both. At a miter the two
blends' ends are one edge and two vertices, `Generated` from both edges
they join, so `generated_pair` finds them; the corner's third edge,
shortened, is `Modified`; no face across takes an arc. A chamfer's record
is a fillet's with its end segments in place of the arcs, and two
chamfers at a corner meet in a line recorded as a miter's ellipse is. A
closed edge's blend has no ends: its torus or cone face, its two contact
circles, its seam and the seam's two vertices are `Generated` from the
edge; the edge's two faces and the cylinder's seam, shortened to the
contact, are `Modified`; the edge and its one vertex are `Deleted`. At a
corner of three blends each side of the corner face — a great circle of
the sphere, a side of the triangle — is its blend's end arc, `Generated`
from that blend's edge, and each corner point from the two edges whose
contacts cross there; the sphere or the triangle face, and a sphere's
pole, a degenerate edge, are `Generated` from all three edges; no corner
edge is cut and the vertex is `Deleted`. `audit` holds on every
result. A second blend on a blended body is rooted at an edge the first
kept or modified, so the records composed with `then` name each blend
face from the role its edge came from (`blend/second-fillet`: both blend
faces from the extrude's `Rise`s, the side face both trimmed still from
its `Side`).

Queries: `generated_from(origin) -> &[Shape]`, `modified_from(origin)`,
`is_deleted(input)`, `origins(output) -> Vec<(Relation, Origin)>` (the
inverse), `outputs()`, `origins_recorded()`, and `Provenance::then(&self,
&next) -> Provenance`, which composes two records so that a chain of
operations (eight cuts of a bolt pattern) reports against the original
inputs: an output of the first that the second modifies is replaced by
its pieces and one it deletes is dropped, with the relations chained
(`Modified` then `Modified` is `Modified`; anything through `Generated`
is `Generated`); one the second generates from stays and gains the
children; an input modified into pieces that are all gone is deleted;
intermediate entities appear nowhere. Composition is associative over
well-formed chains — an output is a new entity, and a record names only
what exists when it runs — up to one order: what later records generate
from an earlier record's pieces comes out as the same sets in a
different order under each bracketing (`docs/BACKLOG.md`). The property
tests check associativity on random chains with that one difference
excluded by name. `Provenance::mapped(&IdMap)` translates a record through the id
map `import` returns, leaving ids the map does not hold (origins in
bodies that were not imported) as they are. `Provenance::rerooted(f)` replaces every role
origin `r` by `f(r)` and touches nothing else: how a consumer roots a
primitive's, sweep's or file's record at its own keys, typically a
`Role::Consumer` that prefixes its key to the operation's part
(ADR-0028). Two roles `f` sends to one concatenate their outputs in the
old roles' ascending order, deduplicated, so the result is deterministic
for any `f`; only an injective `f` keeps `Split(k)` meaning one piece.
Re-rooting commutes with `then` over records whose later steps carry no
role of their own.

**A record crossing models** is `mapped`, and what it cannot say is the
point: an id the map does not hold (an origin in a body that was not
imported) is left as it is, so once the body's entities carry the mapped
ids the writer's foreign ids and the body's own overlap — a writer's
operand face `f2` and the body's own dense `f2` become one value in the
record, and neither a stored flag nor a computed one tells them apart
(ADR-0029's amendment). Body bytes therefore carry the record in the
writer's ids beside the writer → dense map, and `body::read` decides
membership in the writer's ids, where no two entities share one. An
entity the returned `Imported::map` does not hold is foreign
(`Imported::foreign`, ascending, in the writer's ids); the consumer that
sent those entities holds their way back, the `IdMap::inverse` of the map
its own import returned. `Imported::translated(foreign)` puts the whole
record into the caller's ids in one pass over one map — the body's
entities through `Imported::map`, the foreign ones through the map given
— so an id both models use is translated once and once only. The same
overlap is inside one model: importing a body into a model that already
holds entities can send an origin onto an id the body also uses, and
`mapped` alone does not separate them.

**Stability** is what the record is for. Rebuilding the same feature tree
with a changed parameter produces, for each output entity, the same
`origins` chain in terms of the *inputs' roles* (the third hole's tool
face, the top face of the base plate) — because the record is built inside
the algorithm from the entity ids it actually split, not recovered afterwards
by geometric matching, and because the chain ends at a `Role` — the
kernel's name for a part of a primitive, sweep or file, or the
consumer's own key where it built the body or re-rooted the record
(`provenance/consumer-rebuild` holds a consumer's body to that). A consumer's
persistent name is therefore a function of the origins chain, and the
roadmap's acceptance corpus asserts that function is constant across
parameter changes.

**Split order** (ADR-0009). Arris ships no name grammar: a consumer
names from the record, and what only the kernel can give it is the order
of one origin's outputs, which is what `Split(k)` in such a name means.
`generated_from` and `modified_from` list an origin's outputs in the
order the operation added them, deduplicated, and every operation adds
pieces in split order; `PartialEq` compares that order; `mapped` keeps
it; `then` nests what it modifies — piece `i`'s pieces before piece
`i + 1`'s — but the outputs later records generate from a piece lose
the piece they came through, so their order depends on the bracketing
(`then_nests_what_later_records_generate_from_pieces`, ignored,
`docs/BACKLOG.md`). **A face's pieces ascend by
their boundary key**: the sorted, deduplicated origins of the piece's
boundary edges as the record names them — an operand edge for a piece of
one, both faces of the pair for a section edge — compared
lexicographically. Every such origin is an input of the operation, whose
ids a rebuild of the same upstream recipe repeats, so the key reads no
output id and no geometry, and the order holds under every parameter
edit that keeps which entities bound which piece. Two pieces with equal
keys, a tie, are ordered by a point strictly inside each in the origin
face's own (u, v), `u` first, coordinates within the model's parametric
tolerance counting as equal. A tie needs a closed face: the pieces on
either side of a seam are bounded by the same origins, since a piece
never contains a seam — the split keeps a periodic face's seam as a
boundary, so a wall cut with the seam inside one region is three faces,
as Open CASCADE builds it too. A `Generated` list of pieces (a cut tool's
face surviving in several) follows the same order. **An edge's pieces
ascend along its curve**, a closed edge's from its range's start — the
boolean cuts an operand edge at its paves, which come ascending by
parameter, and a closed edge's range is one interval across the seam —
and the section edges one face pair generates are ordered along their
own curve. Edges of one origin on *different* curves are not compared:
a face origin pairs with several faces of the other operand, and the
pairs' own order separates them. The `provenance/split-*` fixtures hold
piece `k` of every split origin to the same neighbours, by role, in
every variant of their recipes — a face's by the faces it shares an edge
with, an edge's by its two end vertices.

## Native format

`arris-io::native` is `serde` of the `Model` under a version header
(`NATIVE_VERSION`): the `Precision`, then every arena's slots in index
order — each slot its generation and, when live, its entity with its ids
as integer pairs — freed slots included, so the model read back has the
same ids and mints the same next one; the adjacency indices are derived
and rebuilt on the way in. Every value with an invariant is validated as
it is read (`Frame::from_orthonormal`, `NurbsCurve::new`, `Interval::new`,
`Precision::is_consistent`), so a stored model is never less of a model
than a built one; a dangling reference is stored as it is and is the
checker's M1 to report. Deterministic byte-for-byte for the same model
(`BTreeMap`s, the shortest round-trip decimal in JSON); a model that
round-trips through it dumps identically before and after, with the
same ids (`arris-io`'s native tests). Two encodings, a `serde` choice per call:
`to_bytes`/`from_bytes` over `postcard` for storage, `to_json`/`from_json`
for diffs. The schema is the model; for the whole model a file of another
version is `NativeError::Version`, a refusal, since a version bump there
is a design delta that comes with a migration or with exactly this
refusal. Body bytes take the other road: every earlier version reads.

**Body bytes** (`arris_io::body`, ADR-0029) are the format a body
outlives the release that wrote it in. `body::write` imports one body
into a fresh model under the writer's `Precision` — so its geometry and
topology are dense from zero whatever holes the writer's model had — and
encodes that model with the record as the writer holds it and the map
from the writer's ids to the dense ones, under a `BODY_MAGIC` of eight
bytes and `BODY_VERSION`; `body::to_json` is the same body as one line of
JSON, for diffs. Both are deterministic byte for byte. `body::read`
decodes, migrates an earlier version to `BODY_VERSION` one version at a
time through `body::compat`, refuses a newer one (`BodyError::Version`),
refuses a tolerance outside the target's `[min_tolerance, max_tolerance]`
rather than rescaling it (`BodyError::Precision`, naming the entity in
the writer's ids), imports into the caller's model inside a transaction
and runs the checker at `Level::Full` in every build profile. It returns
the body, the record and the map from the writer's ids to the caller's,
or a typed `BodyError` with the caller's model left as it was. A
version's bytes are frozen: one blessed `.bin`, `.json` and dump per
guard body under `crates/arris-io/tests/body/v<N>/` — together covering
every curve, surface and pcurve kind, a consumer-keyed record and a
boolean's record with foreign origins — is read in the suite forever, so
a change to a type that alters how an old file decodes fails a test
instead of shipping. An entity of the record outside the body (a
boolean's operands) is foreign and stays in the writer's ids
(§Provenance).

The text dump (`arris_debug::dump_text(&model, body)`) is a different
thing: a human-readable, deterministic listing that fixtures store as
`dump.txt` and tests diff. It has no reader and is never a format. Its
lines are: the model's `Precision`; the body with its kind; its shells,
faces, loops and coedges depth-first in iteration order (§Adjacency and
iteration), each handle with its effective orientation (`+f0`, `-e1` —
a seam edge appears twice in its loop, once with each sign), each face
with its surface written out and each coedge with its pcurve; the free
edges and vertices; then `edges` (each once, with its vertices, curve,
range, tolerance and the curve written out) and `vertices` (point,
tolerance) in iteration order; then the Euler line `euler V/E/F/L/S
g<G> = <r>` where `G` is the genus the counts imply and `r` the residual
of §Euler–Poincaré's identity once `G` is rounded down, `0` for a line
that closes. Every number is rounded to `DUMP_DECIMALS` (12) places with
trailing zeros trimmed, so an ulp never shows and a real change does; a
reference that does not resolve is written as its id and `?`, never
skipped, so the dump of an invalid body says where.

## Open questions

None: the last, the quadric intersection curves, is decided by ADR-0018
(§Curves).

