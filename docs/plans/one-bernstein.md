# Plan: one-bernstein

- Started: 2026-10-25
- Start commit: `b99e7236` (the acceptance's `<start>`)
- Milestone: the refactor side slot beside C11 (cycle C11, docs/ROADMAP.md)
- Idea: docs/ideas/refactor-after-c10.md, its `one-bernstein` part absorbed
  (decision 2026-10-23: option B, the side slot; Q3: 2-D isolation
  bit-identical, the joint exclusion off for N = 2; no ADR)

## Goal
`arris-geom` has one tensor Bernstein module, `bernstein_n`: one `PolyN<N>`,
one `common_zeros::<N>`, one `Zero`, `Gate`, `Wrap`, `merge`. `Poly2` is
`PolyN<2>`, its 2-D extras in `impl PolyN<2>`, and `bernstein2.rs` is gone;
the one-variable de Casteljau leaves live in `bernstein`. Every 2-D isolation
(torus tracer, NURBS–line, NURBS critical points, curve extrema) is
bit-identical to the plan's start, held by isolation goldens, and every
blessed `dump.txt`, `fixture.json`, id and refusal string is byte-identical.
`torus_walk.rs` (3472 lines) is a `torus_walk/` module split by phase, no
file past 1000 lines.

## Non-goals
- No behaviour change; a defect found while moving code becomes a regression
  fixture and a backlog line. Turning the joint exclusion on at N = 2 is a
  measured change of its own (backlog line, Q3).
- The three section tracers stay separate algorithms; `trace_torus.rs`,
  `march.rs` and `chained`'s body are not restructured, only moved.
- No public type or signature changes (all crate-private); no `CHANGELOG.md`.
- `long-functions` is the next side-slot plan; the `arris-debug` split is its
  own idea and ADR.

## Design deltas
- **The idea under-counted the differences.** Read at `b99e7236`: beyond
  `excluded_together`, the two isolations differ in
  1. the product: `Poly2::mul` sums `(i₀, j₀, i₁, j₁)` as
     `((r₀·r₁)·a)·b`, `PolyN::mul` sums `(i₀, i₁, j₀, j₁)` as `((a·b)·r₀)·r₁`
     — and `march` multiplies `PolyN<2>`s, so both orders run at N = 2;
  2. elevation to one degree, which only the joint exclusion needs;
  3. the 2×2 inverse and Newton step (closed form) against Gauss–Jordan;
  4. the certificate score's mixed partial: 2-D takes `∂ₜ(∂ₛp)` for both
     axes, N takes `∂ₗ(∂ₖp)` from each axis's own first;
  5. `Halving::Scored` (the torus tracer's) and two-sided gates, which N lacks;
  6. NaN in the look-ahead tie and a gate's drop (finite coefficients make
     none; the 2-D form is kept).
  `Zero::holds`/`volume`, `merge`'s shift order, `partial`, `split`, `eval`,
  `outer`, `combine` and `grown` already agree bit for bit.
- **Rule for a formula one side must give up:** the 2-D side is kept (the
  older callers with the most fixtures). N ≥ 3's goldens may be re-recorded
  in that commit, named in its body, only while every blessed dump stays
  byte-identical. A dump that moves is a stop: the formula stays per N
  (`if N == 2`, folded at compile time; for the product a second method) and
  a backlog line records it.
- **`bernstein_n` after the merge:**
  - `common_zeros(system, floors, gates, halving, meter)`, one function;
    `common_zeros_with` goes. `Halving::{Scored, LookAhead}` moves here; the
    N ≥ 3 callers (`march`, `march/walk`, `spline_nurbs`) pass `LookAhead`.
  - `Gate { poly, floor, one_sided }`; `spline_nurbs` passes `one_sided: true`.
  - Joint exclusion and elevation run for `N > 2` only (Q3).
  - `solve(j, f) -> [f64; N]`: Cramer at N = 2 as `bernstein2` writes it,
    Gauss–Jordan above; `preconditioned` and `polish` both go through it.
    No N = 2 isolation runs through `bernstein_n` today, so this moves nothing.
  - `impl PolyN<2>`: `across`, `along`, `over_linear`, `transposed`,
    `restricted`; `sign` becomes generic. Callers of `du`/`dv`, `split_u`/
    `split_v`, `eval(s, t)`, `outer(a, b)` move to `partial(k)`, `split(k, …)`,
    `eval([s, t])`, `outer([a, b])`.
  - `Range`, `MARGIN`, `MAX_FRONT`, `FLAT`, `POLISH_STEPS` move in;
    `casteljau` and `split` (one variable) move to `bernstein`.
  - The module doc absorbs `bernstein2`'s: the three tests, the margin's
    reason, the references (Krawczyk, Moore, Sherbrooke–Patrikalakis).
- **Isolation goldens** (test-only, `bernstein_n`'s tests): seeded random
  systems — circles, products, gated, wrapped — isolated at N = 2, 3, 4 under
  both halvings; a digest of every zero's `at`/`lo`/`hi` bits, `certified`,
  `depth`, `boxes` and any `Continuum` front, and of the 2-D arithmetic
  (`mul`, `partial`, `split`, `eval`), against committed values.
- **`torus_walk/`**: `mod.rs` (doc, `TorusWalk`, `TorusArc`, entry points,
  `Tracer::run`), `walker.rs` (the `Walker` and its evaluations), and by the
  idea's phases: cells and seeds, singular points, arcs, assembly. Moves only.
- `docs/DATA-MODEL.md` (the NURBS–line and the curve-distance rows) names
  `bernstein_n::common_zeros` instead of `bernstein2::common_zeros_with`.

## Steps
Complexity grades: **[1]** routine — the design says exactly what to write
and the tests are mechanical; **[2]** careful — a geometric or numeric case
to get right within a given design; **[3]** unproven — an algorithm whose
robustness or bound has to be established here.

- [x] Step 1 **[2]** — Record the start commit here. The isolation goldens,
  over `bernstein2` at N = 2 and `bernstein_n` at 3 and 4, values committed.
  *Settled:* N ≥ 3 has one halving until step 3 gives it `Halving`; its
  `scored` goldens are recorded there.
- [x] Step 2 **[3]** — One product: `PolyN::mul` in `bernstein2`'s order
  generalised (per axis, own index then the other's; ratios, then `a·b`),
  `Poly2::mul` calling the same arithmetic. Measured: the goldens and the
  `march`/`spline_nurbs`/`nurbs_nurbs_prop` dumps; the Rule decides.
- [x] Step 3 **[2]** — `bernstein_n`'s isolation takes `Halving`,
  two-sided gates, `solve`, the 2-D mixed partial; joint exclusion and
  elevation for `N > 2`. A golden test asserts `bernstein_n::common_zeros::<2>`
  equals `bernstein2::common_zeros_with` bit for bit on every golden system.
  *Settled:* the mixed partial is `∂max(∂min p)` for every N (the 2-D form
  at N = 2); N ≥ 3's existing goldens did not move, `scored` ones recorded.
- [x] Step 4 **[2]** — `Poly2` becomes `PolyN<2>` in every caller and in
  `bernstein2`'s isolation; the 2-D extras as `impl PolyN<2>`. Goldens unchanged.
- [x] Step 5 **[2]** — The 2-D callers (`trace_torus`, `torus_walk`,
  `nurbs/line`, `nurbs/critical`, `extrema`) isolate through
  `bernstein_n::common_zeros`; `Zero2`, `merge_wrapped` and `bernstein2`'s
  `Cell`/`Gate`/`krawczyk`/`merge` deleted, its unit tests moved to N = 2.
  *Settled:* the unit tests live in `bernstein_n/two.rs`; the bit-for-bit test
  of step 3 went with `bernstein2`'s isolation, the 2-D goldens (unchanged)
  now run through `bernstein_n`. The DATA-MODEL rows were renamed here too.
- [x] Step 6 **[1]** — `bernstein2.rs` deleted: the leaves and constants
  moved, the module doc merged. `rg bernstein2 crates` is empty.
- [ ] Step 7 **[1]** — `torus_walk/` with `mod.rs`, `walker.rs` and the
  cells-and-seeds and singular-points files; dumps of `torus/`, `nurbs/` unchanged.
- [ ] Step 8 **[1]** — The arcs and assembly files; `sizes.sh` shows no
  `torus_walk/` file past 1000 lines.
- [ ] Step 9 **[1]** — `docs/DATA-MODEL.md` rows and crate docs naming the
  merged module; `fn_lengths.py` shows no new function past 150.

## Acceptance
- `git diff <start> -- tests/fixtures` is empty.
- The 2-D goldens equal step 1's at every step; N ≥ 3's equal or
  re-recorded only by a commit the Rule allowed, named in its body.
- `cargo nextest run -p arris-geom` and `-p arris-ops` (the torus, NURBS,
  blend and real-part corpora) green; the full gate (`tools/full-gate.sh`)
  green at `/retire-plan`.
- One `common_zeros` and one `PolyN` in `arris-geom`; no `bernstein2` left.

## Docs to update on completion
- `docs/DATA-MODEL.md` — the two `bernstein2::common_zeros_with` mentions (step 9).
- `docs/ARCHITECTURE.md` — only if a module list names `torus_walk.rs` or
  `bernstein2`; ADRs are append-only and keep the old names.
- `docs/ideas/refactor-after-c10.md` — `Status` names `one-bernstein` done and
  retired; the idea then holds `long-functions` alone.
- `AGENTS.md` current state — the refactor sequence line, `long-functions` next.
- `docs/BACKLOG.md` — add: the joint exclusion at N = 2, measured (Q3); a
  per-N formula the Rule kept, if any. Delete the stale `bernstein2` line
  (the marcher it anticipated is `bernstein_n`, C10).
- `GLOSSARY.md` — no new term expected.

## Open questions
- Settled (step 2): `bernstein2`'s product order survives, one formula for
  every N; the 2-D goldens and every blessed dump held, N ≥ 3's goldens
  re-recorded.
- Settled (step 3): the mixed partial at N ≥ 3 is `∂max(∂min p)`, the 2-D
  form; no N ≥ 3 golden moved.
- Settled: the merged module keeps the name `bernstein_n`; the ADRs that
  cite `bernstein2` are not edited.
