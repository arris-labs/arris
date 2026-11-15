---
name: map
description: See arris's architecture as one page derived from the code and drilled into — the crates and the layer rule, then each crate's modules, each module's sub-modules, what any module names and what names it, a hand-drawn pipeline whose every box is checked against the source — with the checks the same graph decides (forbidden or unused edges, module cycles, design docs naming paths that no longer exist). Use when asked to show, draw, map or visualise the architecture, a crate or an operation's pipeline; before a refactor review or a plan that moves modules; and when a design doc's paths may have drifted from the code.
---

# Seeing the architecture

`docs/ARCHITECTURE.md` says what the system is meant to be; this skill draws
what the code *is*, and says where the two disagree. Every box is a crate or
module that exists; every arrow is a declared Cargo edge or a path the
source names (`use`, or an inline `crate::`/`super::`/`arris_*::` path),
followed through re-exports to the module that defines it. Nothing in a
chart is drawn from memory: when the picture and the docs disagree, the
picture is the code, and the finding is the docs' or the code's to fix.

## Run

```sh
S=.agents/skills/map/scripts/archmap.py
python3 $S                             # the whole map: one page, drilled into
python3 $S ops::boolean                # the same page opened there; report.md on it
python3 $S --flow FLOW.json            # add a pipeline whose boxes are bound to code
python3 $S --min-weight 3              # thinner charts: arrows under 3 paths go
```

Stdlib Python, about two seconds over the workspace, nothing to install and
nothing fetched; it reads `cargo metadata --no-deps` and the library
sources (not `tests/`, `examples/`, or `#[cfg(test)]` modules), and lays
every chart out itself (`scripts/draw.py`: layered, the foundation at the
bottom, cycles broken for the layout only). Output goes to `target/archmap/`
(or `--out DIR`): `index.html` (one self-contained page, light and dark),
`report.md` (findings, then charts as text: nodes and `a → b (weight)`
lines) and `graph.json` (every module-to-module edge with its weight, for a
question no chart answers — read it with `jq`).

**Read `report.md` yourself**; it is the chart as text, and the arrows in it
are what you reason from. Without a target it holds the layer chart and
each crate's; with one, that module's chart and its side panel (who names
it, what it names, its most-named items). The page is for the human:
`xdg-open target/archmap/index.html#ops::boolean`. Before you hand a page
over, **look at it**:

```sh
cd target/archmap && chromium --headless --disable-gpu --hide-scrollbars \
  --window-size=1500,1300 --screenshot=$PWD/shot.png "file://$PWD/index.html#ops::boolean"
```

then read `shot.png` with the Read tool (`#<module>` picks the chart, or
selects a leaf in its parent's; `--force-dark-mode` checks the dark theme).
The snap Chromium reads and writes only under the home directory, so keep
the files in `target/`, not in `/tmp`.

## What the page shows

One map, entered at the top and drilled into. A trail (`workspace › ops ›
boolean`) goes back up; the URL hash names where you are, so back, forward
and a pasted link work. Scroll zooms about the cursor, a drag pans, a
double-click on the empty canvas (or ⤢) fits the chart again.

- **workspace** — one box per crate with its layer from
  `tools/check-layers.sh` (the table there is the source; a crate missing
  from it is a finding), module count and non-blank lines. An arrow is a
  normal dependency, labelled with how many paths the source writes into
  that crate. Dev-dependencies are listed under the chart, not drawn: they
  cross layers by design.
- **a crate or a module that holds modules** — one box per child (its
  sub-modules folded in), its lines, `pub` when it is a public path; a
  `mod.rs`/`lib.rs` box for the container's own file when it names or is
  named. An arrow `a → b (n)`: `a` names `n` paths that resolve into `b`.
  Around them, dashed **pills** are the outside, one step out: another
  crate, or the sibling module the path enters — users on top, what is used
  at the bottom, the seven heaviest each side, each with its heaviest
  arrows. A **stacked** box holds modules: click it to open it. Every level
  has one, 38 charts in all.
- **the side panel** — click any other box: its file, size, who names it
  and what it names (all of them, one step out, each a link), and its
  most-named items with their line.
- **flow** — the one chart the code cannot draw: an operation's stages in
  order. You write it (below); the script checks each box.

Three reductions keep a chart readable, each stated under it: a `pub use`
is the re-export list and draws no arrow; an arrow implied by a longer
path is dropped (transitive reduction over the cycle-condensed graph); and,
unless `--min-weight N` is given, the lightest arrows go until the chart
fits 40, each module keeping its heaviest. A chart still too busy is read
one level down, not merged by hand.

Marks: a red box or arrow is a **cycle** of up to four modules (usually one
type that belongs a layer down — read both ends); an orange box is a
**tangle**, a larger strongly connected set (in `geom` the curve, surface,
error and intersection modules naming each other's types — idiomatic in
one crate, but where a split would have to cut). A small hollow dot means
no design doc names the module by a path.

## Findings and what to do with them

`report.md` splits them. **Findings** break a rule:

- a forbidden edge — the layer rule (ADR-0013); the hook should already
  have refused it, so a finding here means the table and the hook disagree;
- a declared dependency no source path names — remove it, or say why it
  is there;
- a design-doc path (`` `a::b` `` in `docs/ARCHITECTURE.md` or
  `DATA-MODEL.md`) that resolves to no module or item — the doc names
  something that was renamed or never existed. Fix it as `docs(sync): …`
  in its own commit, or in the step's commit when a plan is moving it;
- a flow box bound to nothing.

**Observations** are not rules: cycles and tangles at every level,
undocumented modules, busy charts. They are input to `/refactor-review`
and to a plan's *Design deltas*, never a fix of their own. Name boxes in
the project's words (`GLOSSARY.md`) when you write about them.

## A pipeline: the flow file

For "how does `fuse` work" or "where does a blend refuse", draw the stages.
Each box is **bound** to a module or item by `at` (`ops::fuse`,
`ops::boolean::pave::build`, `check::classify_point`, `topo::Builder`); the
script resolves every binding through re-exports, prints its `file:line`,
and marks an unbound box red. A worked example is
`references/fuse.flow.json`:

```json
{"title": "…", "direction": "TB",
 "nodes": [{"id": "fuse", "label": "fuse(a, b)", "at": "ops::fuse"}, …],
 "edges": [["fuse", "inter"], ["inter", "pave", "per face pair"], …]}
```

The binding proves the box exists, not that the arrow is right. Before
handing a flow over, read the body at each binding's `file:line` and
confirm the next box is what it calls or returns into. Keep a flow file the
docs would want under `docs/` only when a design doc links it; otherwise it
is scratch.

## Hand-off

Give the human: the path to `index.html` with the `#module` they should
start at (or the published page, when they ask for one to share), the
findings with their `file:line`, one sentence per chart you looked at on
what it shows that the docs do not, and the command that redraws it. For a small chart, the `a → b` lines from
`report.md` in the reply let the human read it without opening anything.

## Limits

Paths are resolved from the source text, not by the compiler: a name
brought in by a glob from a crate outside the workspace, a macro-generated
path, or a method call does not draw an arrow — so a child file that only
adds an `impl` block to its parent's type (`boolean::pave::hits`) reads as
named by nothing. A weight
counts paths written, not calls made; `use crate::{a, b}` is two. These
undercount, never invent: an arrow drawn is a path that is there.
