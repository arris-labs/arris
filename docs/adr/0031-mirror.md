# ADR-0031 — Mirror: a `Reflection` of its own, frames kept right-handed, the surface's `u` reflected, the loops reversed

- Status: accepted (2026-09-30)
- Plan: `mirror` steps 1–6
- Follows: ADR-0002 (orientation and the coedge loop), ADR-0009
  (provenance, the transaction an operation runs in),
  `docs/DATA-MODEL.md` §Conventions (frames are right-handed),
  §Orientation, §Pcurves

## Context

A mirrored component is a core feature of a parametric CAD, and
`ops::transform` cannot make one: it takes an `Isometry`, a rotation and a
translation, whose whole point is that handedness is preserved. Every
analytic surface and curve is placed by a right-handed `Frame`, so a rigid
motion is a frame change and nothing else, and a face's pcurves stay valid
as they stand.

A reflection (`det = −1`) breaks each of those at once:

- the image of a right-handed frame is left-handed, and DATA-MODEL says
  frames are right-handed by construction;
- the image of a surface, in the same parameters, has its normal
  `∂u × ∂v` turned against the image of the original normal, so a face's
  use orientation is wrong for it;
- the image of a pcurve in the same parameters winds the other way about
  the loop it bounds: a loop stored counter-clockwise in `(u, v)`, the
  condition every face's loop satisfies whatever its use orientation, is
  clockwise in the image.

The design has to say where each of the three is absorbed.

## Decision

1. **A reflection is its own type.** `arris_math::Reflection { origin,
   normal }` (a plane through `origin` with unit `normal`), with `apply`
   for points and `apply_vec` and `apply_unit` for directions. An
   `Isometry` stays proper by construction, so every `transformed` keeps
   the invariant it relies on; a `Reflection` of a zero, non-finite or
   subnormal normal is refused when it is made.
2. **Frames stay right-handed; the parametrisation absorbs the
   reflection.** The mirror image of `S(u, v)` is `R·S(m(u, v))` for a
   *parameter map* `m`, chosen per surface kind so that the image is the
   same kind of surface over a right-handed frame:

   | Surface | Frame of the image | Map `m` | Surface normal against `R n` |
   |---|---|---|---|
   | Plane | `O′ = R O`, `X′ = R X`, `Y′ = R Y`, `Z′ = X′ × Y′ = −R Z` | identity | opposite |
   | Cylinder, elliptic cylinder, cone, sphere, torus | `O′ = R O`, `X′ = R X`, `Y′ = −R Y`, `Z′ = X′ × Y′ = R Z` | `u ↦ 2π − u`, `v` unchanged | same |
   | NURBS | control net reflected, weights, knots and degrees kept | identity | opposite |

   For a quadric the reflected direction is `u`: `cos(2π − u) = cos u` and
   `sin(2π − u) = −sin u`, and `Y′ = −R Y` restores the sign, so
   `P′(2π − u, v) = R·P(u, v)`. Choosing `u` rather than `v` keeps the
   axis direction `Z′ = R Z` — which a cone needs, since its radius grows
   along `+Z` and nowhere else (DATA-MODEL §Surfaces), and a sphere's
   poles and a torus's tube angle keep their parameters. The seam stays at
   `u = 0` and `2π` and stays on the image of the seam; `2π − u`, not
   `−u`, keeps a pcurve that lay in `[0, 2π]` in `[0, 2π]`. An elliptic
   cylinder's `X′ = R X` is still its section's major axis.

   The plane and the NURBS surface take the identity map because nothing
   forces another: their image is `R·S(u, v)` and only the normal turns.

   A **curve** never needs a map. A circle or ellipse of frame `(O, X, Y,
   Z)` is `O + ρ (c t·X + s t·Y)`, and the image `O′ + ρ (c t·X′ + s
   t·Y′)` over `X′ = R X`, `Y′ = R Y`, `Z′ = X′ × Y′ = −R Z` is the same
   curve in the same parameter, its frame right-handed by construction
   and its axis normal pointing the other way, which a curve's frame does
   not care about. A line's direction is `R D`; a NURBS curve's control
   points are reflected. So `Curve::mirrored` returns a `Curve` and every
   edge keeps its vertex order and its parameter range.
3. **Pcurves are mapped, and the map is one reflection of `(u, v)`.**
   `transform` reuses every pcurve id; a mirror reuses a pcurve exactly
   where the map is the identity (a plane's or a NURBS surface's). Where
   `u ↦ 2π − u`, `Curve2::reflected` builds the image: a line's origin
   and direction, a circle's or ellipse's `Frame2` (whose handedness is
   the direction of traversal and is free to be left, DATA-MODEL
   §Pcurves), a NURBS pcurve's control points; the parameter `t` is
   untouched, so a pcurve still runs along its edge's curve in the
   edge's parameter. A periodic `u` shifted by a period is the same
   pcurve; `2π − u` is the shift that keeps the seam pair at `0` and
   `2π`.
4. **The image of a loop is walked the other way.** A mirror reverses the
   winding of every loop about the outward normal, so the *effective*
   loop of a mirrored face is the reverse of the mirror image of the
   original's effective loop, on every face. That fixes the two things a
   face copy has to say:
   - the **use orientation** of the mirrored face is the original's,
     toggled exactly when the surface's normal is opposite `R n`
     (the last column of decision 2's table): planes and NURBS faces
     toggle; quadric faces keep it. The effective normal of the image is
     then `R` of the original's, on every face.
   - the **stored loop** is the original's, copied — coedges in their
     order, orientations as they are — when the map is the identity (the
     image is counter-clockwise in `(u, v)` about the image surface's own
     normal exactly as the original was), and *reversed* when the map
     reflects `(u, v)`: the coedge sequence walked backwards and each
     coedge use's orientation toggled, which restores the counter-
     clockwise winding a stored loop must have. A plane's face and a
     cylinder's face sharing a circle then meet it with opposite
     effective directions, as two faces of a manifold shell must.
   Edges themselves, and each edge's two uses' pairing, are unchanged.
   Shell uses of faces follow the face's toggled orientation. The
   checker at `Full` decides whether the result is right; the plan's
   fixtures hold it to the oracle.
5. **Provenance is one to one.** Every vertex, edge, face, shell and the
   body is `Modified` from the entity it mirrors, as `transform`. No
   `Generated`, no `Deleted`, and the result shares no entity with its
   input, even with a plane through the body: a mirror copies, it does
   not cut.
6. **The op is `transform`'s sibling over the same seam.** `mirror`
   builds through `Assembly::of_body` and `Builder::assemble`, so it
   reaches exactly as far as `transform` does: a `Solid`, and anything
   else is `OpError::Internal` naming the builder's refusal. Two hooks
   are added to `GeometryRemap`, both with a default that is the identity
   so `transform`, `KeepGeometry` and any consumer's own implementation
   compile unchanged:
   - `pcurve(&mut self, model, p: Curve2Id, surface: SurfaceId) ->
     Curve2Id` — the pcurve to give a copied use, `surface` the original
     surface the pcurve lay on;
   - `face(&mut self, model, surface: SurfaceId) -> FaceRemap`, with
     `FaceRemap { toggle_use: bool, reverse_loops: bool }` — decision 4's
     two facts about the face over `surface`.

   `Assembly::of_body` applies them: it toggles the face use, and where
   `reverse_loops` it copies each loop backwards with each coedge use
   toggled. `Surface::mirrored(&Reflection) -> (Surface, ParamMap)` with
   `ParamMap::{Identity, ReflectU}` is what `mirror`'s remap reads to
   answer both.

## Consequences

- One `### Breaking` bullet: `GeometryRemap` gains two provided methods
  and `Assembly::of_body` calls them. A consumer with its own
  implementation compiles as it is; the entry says so.
- A mirror allocates a new pcurve for every use on a quadric face and
  none on a plane or NURBS face. The arena's ids stay deterministic:
  faces, edges and uses are copied in the body's own iteration order and
  a pcurve is added at the use that needs it.
- A mirrored quadric wall carries its seam pair as `2π` and `0` where the
  original had `0` and `2π`. The checker and the tessellator take the
  pair by Forward and Reversed use and the period, not by which is the
  larger (§Seams); step 3 runs the mirrored fixtures at `Full` to prove
  it and records a fixture if a reader assumes otherwise.
- `mirror ∘ mirror` in one plane returns the original's geometry: a
  quadric's `Y″ = −R(−R Y) = Y` and `u ↦ 2π − (2π − u)` is the identity,
  a plane's `Z″ = −R(−R Z) = Z`. The dump equals the original's up to id
  numbering, and the property holds it there.

## Alternatives considered

- **A left-handed `Frame`.** The simplest map (`Y′ = R Y` and `Z′ = R
  Z`), and the image of every quadric would keep its parametrisation.
  It weakens the invariant every `transformed`, every normal formula and
  every pcurve rule stands on, and breaks the "a frame's `Z` is `X × Y`"
  that `Frame::from_orthonormal` checks, for one operation.
- **An `Isometry` widened to improper maps.** The same weakening one level
  down: `Isometry::apply_frame` would have to return something that is not
  a `Frame`.
- **A reflection stored on the face** (a flag the tessellator, the
  measurer and the checker each read). Every consumer of a face's
  geometry would have to know; a body that has been mirrored would be
  different from one that was built that way.
- **A mirrored `Surface` variant.** Doubles every exhaustive dispatch
  (`intersect_surfaces` is `n²`) for what a parameter map does.
- **Mirror as an option of `transform`.** The Rust type says what the
  motion preserves; `Isometry` and `Reflection` cannot be confused, and
  the two ops' contracts differ (one reuses pcurves, one does not).
- **Reflecting `v` instead of `u` for a quadric.** A cone's `Z′` would be
  `−R Z`, which a cone's half-angle cannot express (it grows along `+Z`
  only), and the sphere's poles and the seam's period would move for no
  gain.
- **Reflecting `u ↦ −u`.** The same map without the period shift; every
  pcurve of a wall would leave `[0, 2π]`, allowed but a needless change
  to the range every reader of a seam has met.
