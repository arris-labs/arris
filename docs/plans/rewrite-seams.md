# Plan: rewrite-seams

- Started: 2026-11-04
- Milestone: side slot beside C12 (docs/ROADMAP.md §C12); runs after the nightly's two panics, before C12's first code step
- Idea: docs/ideas/refactor-after-c11.md, its `rewrite-seams` part absorbed (decision B, 2026-11-04, round 1; Q2: the parser fix is step 1)

Under ~150 lines and ~10 steps; larger work is milestone plans.

## Goal
The review scripts count every library function, including those with an
array in their signature. A `Rewrite` grows through methods that return
the new element's index. An edge use is built by one `StoredUse`
constructor, so C12's draft and thicken can use both from their first
commit. The trace and the march share one function that moves a point by
whole closures. Behaviour does not change: every blessed dump,
`fixture.json`, `cancel_counts.txt` and `tolerance_band.txt` is
byte-identical to the plan's start, and ids and refusal strings are
unchanged.

## Non-goals
- No public type or signature change. Everything here is
  `pub(crate)`, so the binding, `_arris.pyi` and `CHANGELOG.md` are
  untouched.
- `add_ring_faces`' two four-use loops written as one, and every split of
  `blend/build.rs` and its long functions: `blend-phases`, the idea's
  next plan.
- `census.rs`, `fixtures::lint`, `blend_prop` and the loft section types:
  `census-and-tests`, the plan after that.
- A defect found while moving code becomes a regression fixture and a
  backlog line. It is never fixed here.

## Design deltas
- **`.agents/skills/refactor-review/scripts/_rust.py`.** `functions`
  takes a `;` as ending a declaration only at bracket depth 0 before the
  body's `{`, so the `;` inside `[T; N]` no longer drops a function.
  `fn_lengths.py` then counts 3627 functions at `1a7bc1e6` (3207 before),
  and `junction` is first.
- **`rebuild::Rewrite` gains `pub(crate)` methods** that push and return
  the index: `vertex(VertexSpec) -> usize`,
  `edge(EdgeSpec, Option<EdgeId>) -> usize` and
  `face(AddedFace) -> usize`. The fields stay public within the crate,
  since the face maps and `vertex_parents` are still written directly.
- **`rebuild::StoredUse` gains constructors**, both crate-private:
  - `new(m: &mut Model, edge: EdgeKey, forward: bool, pcurve: &Curve2)`,
    which adds the pcurve;
  - `of(edge: EdgeKey, c: &Coedge)`, an operand's use carried over with
    its orientation and pcurve.

  A crate-private `rebuild::along(forward: bool) -> Orientation` replaces
  the nine written-out `if … { Forward } else { Reversed }`.
  `Orientation::flipped` replaces `ring_rolled.rs`'s hand-written flip.
- **`arris-geom`**: one crate-private `near_by_closures(at, to, closure)`,
  in `march/walk.rs`'s parent or a shared module the step picks. It is
  called by `trace.rs` (now `near_closure`) and `march/walk.rs` (now
  `near`).
- No ADR: structure under ADR-0013 (layers) and the `ops::rebuild` design
  of the kernel-seams work.

## Steps
Complexity grades; a step's grade is its tier, the model that executes it
(`.agents/tiers.md`), and `**[1, tier 3]**` overrides the tier: **[1]**
routine — the design says exactly what to write and the tests are
mechanical; **[2]** careful — a geometric or numeric case to get right
within a given design; **[3]** unproven — an algorithm whose robustness or
bound has to be established here.

- [x] Step 1 **[1]** — `_rust.py`'s depth-aware `;`. Run the four
  scripts. The commit body records the counts before and after
  (functions, past 100 and past 150 lines) and the new duplicate rows.
- [x] Step 2 **[1]** — Add `Rewrite::vertex`, `edge` and `face`, each with
  a unit test in `rebuild.rs` that the index returned addresses the
  element. Move `offset/faces.rs` and `blend/rolled.rs` onto them.
- [x] Step 3 **[1]** — Move `blend/build.rs`'s remaining `len()` + `push`
  pairs onto the methods. After it,
  `grep -rE 'rw\.(vertices|edges|added)\.len\(\)' crates/arris-ops/src` is
  empty outside `rebuild.rs`, except where a length is read without a
  push.
- [x] Step 4 **[1]** — Add `StoredUse::new`, `StoredUse::of` and `along`,
  with unit tests. Move `rebuild.rs`, `shell/rim.rs`, `offset/faces.rs`,
  `blend/rolled.rs` (its two copies of `along` and `add` go) and
  `blend/ring_rolled.rs` (`flipped`) onto them.
- [x] Step 5 **[1]** — Move `blend/build.rs`'s 18 `StoredUse` literals and
  its `along` closures onto them. A literal whose pcurve is an existing
  `Curve2Id`, and not a new curve, stays a literal; the commit says how
  many.
- [ ] Step 6 **[1]** — Replace `near_closure` and `near` with one function
  in `arris-geom`. The trace, march and torus-walk tests pass unchanged.

## Acceptance
- `tools/full-gate.sh` is green on the plan's last commit.
- `git diff <plan start>.. -- 'tests/fixtures/**' 'crates/**/cancel_counts.txt'
  'crates/**/tolerance_band.txt'` is empty.
- Within `crates/arris-ops/src`, `StoredUse {` appears only in
  `rebuild.rs` and at step 5's counted literals.
- `if … { Orientation::Forward } else { Orientation::Reversed }` appears
  nowhere outside `rebuild::along`.
- `python3 .agents/skills/refactor-review/scripts/duplicates.py` no longer
  lists `near_closure`/`near`.

## Docs to update on completion
- `docs/ideas/refactor-after-c11.md`: drop this plan's section, already
  cut at planning, and confirm the status names the next plan
  (`blend-phases`).
- `docs/ARCHITECTURE.md` §Operations, the `rebuild` paragraph
  (`rebuild::rewrite`'s callers): one sentence saying a `Rewrite` is
  filled through its index-returning methods and `StoredUse::new`/`of`.
- `CHANGELOG.md`: nothing, since no consumer can see the change.
- `GLOSSARY.md`: no new term.

## Open questions
- None for the human. The agent decides the shared module for the
  closure helper at step 6 (the lowest module both `trace` and `march`
  already import) and records the choice in the commit.
