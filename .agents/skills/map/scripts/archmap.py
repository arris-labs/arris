#!/usr/bin/env python3
"""Architecture maps of the arris workspace, derived from the code.

Every box is a crate or a module that exists; every arrow is a declared
Cargo edge or a path the source names (`use`, or an inline `crate::`/
`super::`/`arris_*::` path), resolved through re-exports to the module that
defines it. The checks run on the same graph: the layer rule, declared
edges no source uses, module cycles, docs naming paths that do not resolve,
and modules no design doc names.

    archmap.py                     the whole map: one page, drilled into by clicking
    archmap.py ops::boolean        the same page opened there; report.md on it
    archmap.py --flow FLOW.json    a hand-drawn pipeline, each box bound to code

Output in --out DIR (default target/archmap): index.html, report.md, graph.json.
Stdlib only; reads `cargo metadata --no-deps` and the sources.
"""

from __future__ import annotations

import argparse
import html
import json
import re
import subprocess
import sys

import draw  # beside this script; its directory is on sys.path
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

EDGE_BUDGET = 40
SMALL_CYCLE = 4
STD = {"std", "core", "alloc", "f64", "f32", "usize", "isize", "u8", "u16", "u32", "u64", "i8", "i16", "i32",
       "i64", "Vec", "Option", "Result", "Box", "String", "nalgebra", "serde", "rayon", "pyo3", "robust", "image"}
DOCS = ("docs/ARCHITECTURE.md", "docs/DATA-MODEL.md")

# ---------------------------------------------------------------- source text


def strip(src: str) -> str:
    """Blank comments, strings and char literals; keep newlines and offsets."""
    out: list[str] = []
    i, n = 0, len(src)
    raw = re.compile(r'b?r(#*)"')
    char = re.compile(r"'(?:\\(?:x[0-9a-fA-F]{2}|u\{[0-9a-fA-F]*\}|.)|[^\\'\n])'")
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif src.startswith("/*", i):
            depth, j = 1, i + 2
            while j < n and depth:
                if src.startswith("/*", j):
                    depth, j = depth + 1, j + 2
                elif src.startswith("*/", j):
                    depth, j = depth - 1, j + 2
                else:
                    j += 1
            out.append(re.sub(r"[^\n]", " ", src[i:j]))
            i = j
        elif (c in "br") and (i == 0 or not (src[i - 1].isalnum() or src[i - 1] == "_")) and raw.match(src, i):
            m = raw.match(src, i)
            end = src.find('"' + m.group(1), m.end())
            end = n if end < 0 else end + 1 + len(m.group(1))
            out.append('""' + re.sub(r"[^\n]", " ", src[i + 2 : end]))
            i = end
        elif c == '"':
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == "\\" else 1
            out.append('"' + re.sub(r"[^\n]", " ", src[i + 1 : j]) + '"')
            i = j + 1
        elif c == "'" and (m := char.match(src, i)):
            out.append("' '" + " " * (m.end() - i - 3))
            i = m.end()
        else:
            out.append(c)
            i += 1
    return "".join(out)


def block_end(text: str, open_brace: int) -> int:
    depth = 0
    for j in range(open_brace, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                return j + 1
    return len(text)


def drop_test_blocks(text: str) -> str:
    """Blank `#[cfg(test)] mod x { … }` so test-only imports draw no arrow."""
    pat = re.compile(r"#\[cfg\(test\)\]\s*(?:#\[[^\]]*\]\s*)*(?:pub(?:\([^)]*\))?\s+)?mod\s+\w+\s*\{")
    while m := pat.search(text):
        end = block_end(text, m.end() - 1)
        text = text[: m.start()] + re.sub(r"[^\n]", " ", text[m.start() : end]) + text[end:]
    return text


def split_top(s: str) -> list[str]:
    parts, depth, cur = [], 0, []
    for ch in s:
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
        if ch == "," and depth == 0:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(ch)
    if "".join(cur):
        parts.append("".join(cur))
    return [p for p in parts if p]


def expand_use(tree: str) -> list[tuple[list[str], str]]:
    """`a::{b::{c, d as e}, f::*}` → [(['a','b','c'],'c'), (['a','b','d'],'e'), (['a','f','*'],'*')]."""
    tree = re.sub(r"\s+", " ", tree).strip()
    out: list[tuple[list[str], str]] = []

    def rec(prefix: list[str], t: str) -> None:
        t = t.strip().removeprefix("::")
        if not t:
            return
        if t.startswith("{"):
            for part in split_top(t[1 : t.rfind("}")]):
                rec(prefix, part)
            return
        brace = t.find("{")
        if brace >= 0:
            head = [s for s in t[:brace].split("::") if s.strip()]
            rec(prefix + [s.strip() for s in head], t[brace:])
            return
        alias = None
        if " as " in t:
            t, alias = (x.strip() for x in t.split(" as ", 1))
        segs = [s.strip() for s in t.split("::") if s.strip()]
        if segs == ["self"]:
            out.append((prefix, alias or (prefix[-1] if prefix else "self")))
            return
        if segs and segs[-1] == "self":
            segs = segs[:-1]
        path = prefix + segs
        if path:
            out.append((path, alias or path[-1]))

    rec([], tree)
    return out


USE = re.compile(r"\b((?:pub(?:\([^)]*\))?\s+)?)use\s+([^;]+);")
BARE = re.compile(r"(?<![:\w])([a-z_]\w*)(?:\s*::\s*[A-Za-z_]\w*)+")  # `pave::build(…)`, head a child or a used name
INLINE = re.compile(r"\b(?:crate|super|self|arris(?:_[a-z]+)?)(?:\s*::\s*[A-Za-z_]\w*)+")
MODDECL = re.compile(r"((?:#\[[^\]]*\]\s*)*)(pub(?:\([^)]*\))?\s+)?mod\s+([A-Za-z_]\w*)\s*;")
ITEM = re.compile(r"\b(?:fn|struct|enum|trait|type|const|static|union)\s+([A-Za-z_]\w*)|macro_rules!\s*([A-Za-z_]\w*)")

# ------------------------------------------------------------------ the model


@dataclass
class Module:
    crate: str
    path: tuple[str, ...]
    file: Path
    public: bool
    loc: int = 0
    text: str = ""
    uses: list[tuple[list[str], str]] = field(default_factory=list)  # (segs, bound name)
    reexport: set[int] = field(default_factory=set)  # indices into uses written as `pub use`
    doc_only: set[int] = field(default_factory=set)  # indices into uses only a doc link reads
    inline: list[tuple[list[str], bool]] = field(default_factory=list)  # (segs, resolved like a use)
    items: dict[str, int] = field(default_factory=dict)  # name → line


@dataclass
class Crate:
    name: str
    lib: str  # the name sources use: arris_geom
    root: Path
    layer: int | None
    deps: list[tuple[str, str]]  # (dep crate, kind)
    modules: dict[tuple[str, ...], Module] = field(default_factory=dict)

    @property
    def short(self) -> str:
        return self.name.removeprefix("arris-") if self.name != "arris" else "arris"


class Workspace:
    def __init__(self, root: Path):
        self.root = root
        meta = json.loads(
            subprocess.run(
                ["cargo", "metadata", "--format-version", "1", "--no-deps"],
                cwd=root, check=True, capture_output=True, text=True,
            ).stdout
        )
        layers = read_layers(root / "tools/check-layers.sh")
        self.crates: dict[str, Crate] = {}
        names = {p["name"] for p in meta["packages"]}
        for p in meta["packages"]:
            libs = [t for t in p["targets"] if set(t["kind"]) & {"lib", "rlib", "cdylib"}]
            if not libs:
                continue
            deps = [(d["name"], d["kind"] or "normal") for d in p["dependencies"] if d["name"] in names]
            self.crates[p["name"]] = Crate(
                p["name"], libs[0]["name"], Path(libs[0]["src_path"]), layers.get(p["name"]), deps
            )
        self.by_lib = {c.lib: c.name for c in self.crates.values()}
        self.by_short = {c.short: c.name for c in self.crates.values()}
        for c in self.crates.values():
            self._load(c, (), c.root, True)
        self.named: dict[tuple[str, str], int] = defaultdict(int)  # crate → crate its source names
        self.item_refs: dict[tuple[str, tuple, str], int] = defaultdict(int)  # (crate, module, item) → paths
        self.edges = self._edges()

    # -- module tree
    def _load(self, crate: Crate, path: tuple[str, ...], file: Path, public: bool) -> None:
        raw = file.read_text(encoding="utf-8")
        text = drop_test_blocks(strip(raw))
        m = Module(crate.name, path, file, public, loc=sum(1 for l in raw.splitlines() if l.strip()))
        m.text = text
        crate.modules[path] = m
        is_dir_owner = file.name in ("lib.rs", "mod.rs", "main.rs")
        child_dir = file.parent if is_dir_owner else file.parent / file.stem
        for d in MODDECL.finditer(text):
            attrs, vis, name = d.group(1), d.group(2), d.group(3)
            if "cfg(test)" in attrs:
                continue
            pm = re.search(r'#\[path\s*=\s*"([^"]+)"', raw[d.start() : d.end()])
            cands = [file.parent / pm.group(1)] if pm else [child_dir / f"{name}.rs", child_dir / name / "mod.rs"]
            f = next((x for x in cands if x.exists()), None)
            if f:
                self._load(crate, path + (name,), f, public and bool(vis and vis.strip() == "pub"))
        for u in USE.finditer(text):
            new = expand_use(u.group(2))
            if u.group(1).strip():
                m.reexport.update(range(len(m.uses), len(m.uses) + len(new)))
            m.uses.extend(new)
        no_use = USE.sub(lambda u: re.sub(r"[^\n]", " ", u.group(0)), text)
        # A private import whose name the code never writes but a doc comment does is
        # there for a doc link (rustc counts those as uses): it couples nothing, so it
        # draws no arrow. A trait imported for its methods is never written either, but
        # no doc names it, so it stays.
        idents = set(re.findall(r"[A-Za-z_]\w*", no_use))
        docs = set(re.findall(r"[A-Za-z_]\w*", " ".join(re.findall(r"^\s*//[/!](.*)$", raw, re.M))))
        m.doc_only = {i for i, (segs, name) in enumerate(m.uses)
                      if i not in m.reexport and name not in idents and name in docs}
        for p in INLINE.finditer(no_use):
            m.inline.append(([s.strip() for s in p.group(0).split("::")], False))
        heads = {k[-1] for k in crate.modules if k[:-1] == path and len(k) == len(path) + 1} | {n for _, n in m.uses}
        for p in BARE.finditer(no_use):
            if p.group(1) in heads and p.group(1) not in ("crate", "self", "super"):
                m.inline.append(([s.strip() for s in p.group(0).split("::")], True))
        for it in ITEM.finditer(text):
            m.items.setdefault(it.group(1) or it.group(2), text.count("\n", 0, it.start()) + 1)

    # -- path resolution
    def anchor(self, crate: str, mod: tuple[str, ...], segs: list[str], from_use: bool):
        """Where a path starts: (crate, module, remaining segments) or None for std & co."""
        if not segs:
            return None
        s0 = segs[0]
        if s0 == "crate":
            return crate, (), segs[1:]
        if s0 == "self":
            return crate, mod, segs[1:]
        if s0 == "super":
            k = 0
            while k < len(segs) and segs[k] == "super":
                k += 1
            return crate, mod[: max(0, len(mod) - k)], segs[k:]
        if s0 in self.by_lib:
            return self.by_lib[s0], (), segs[1:]
        if from_use and mod + (s0,) in self.crates[crate].modules:
            return crate, mod, segs
        if from_use and s0 in self._names(crate, mod):
            return crate, mod, segs
        return None

    def _names(self, crate: str, mod: tuple[str, ...]) -> dict[str, list[str]]:
        m = self.crates[crate].modules.get(mod)
        return {} if m is None else {name: segs for segs, name in m.uses}

    def walk(self, crate: str, mod: tuple[str, ...], segs: list[str], hops: int = 0):
        """Follow a path through modules and imports to the deepest module."""
        mods = self.crates[crate].modules
        i = 0
        while i < len(segs):
            s = segs[i]
            if mod + (s,) in mods:
                mod, i = mod + (s,), i + 1
                continue
            if hops < 12:
                m = mods.get(mod)
                target = None
                if m is not None:
                    for useg, name in m.uses:
                        if name == s and useg and useg[-1] != "*":
                            target = useg
                            break
                    if target is None:  # a glob that brings the name
                        for useg, name in m.uses:
                            if name == "*":
                                a = self.anchor(crate, mod, useg[:-1], True)
                                if a and self._defines(*self.walk(*a, hops + 1)[:2], s):
                                    target = useg[:-1] + [s]
                                    break
                if target is not None:
                    a = self.anchor(crate, mod, target, True)
                    if a and (a[0], a[1], a[2]) != (crate, mod, segs[i:]):
                        return self.walk(a[0], a[1], a[2] + segs[i + 1 :], hops + 1)
            break
        return crate, mod, segs[i:]

    def _defines(self, crate: str, mod: tuple[str, ...], name: str) -> bool:
        m = self.crates[crate].modules.get(mod)
        return m is not None and (name in m.items or any(n == name for _, n in m.uses))

    def resolve(self, crate: str, mod: tuple[str, ...], segs: list[str], from_use: bool):
        a = self.anchor(crate, mod, segs, from_use)
        if a is None:
            return None
        return self.walk(*a)

    def _edges(self) -> dict[tuple[str, tuple, str, tuple], int]:
        """(crate, module) → (crate, module): number of paths naming it.

        A `pub use` is the re-export list, not a use: it counts toward the
        crate-level `named` (a facade crate is only that) but draws no arrow.
        """
        w: dict = defaultdict(int)
        for c in self.crates.values():
            for path, m in c.modules.items():
                refs = [(segs, True, i in m.reexport) for i, (segs, _) in enumerate(m.uses) if i not in m.doc_only] + \
                       [(s, u, False) for s, u in m.inline]
                for segs, from_use, reexport in refs:
                    if segs and segs[-1] == "*":
                        segs = segs[:-1]
                    a = self.anchor(c.name, path, segs, from_use)
                    if a is None:
                        continue
                    if a[0] != c.name:
                        self.named[(c.name, a[0])] += 1
                    if reexport:
                        continue
                    tc, tm, left = self.walk(*a)
                    if (tc, tm) == (c.name, path):
                        continue
                    if tc == c.name and path[: len(tm)] == tm:
                        continue  # a module naming its own ancestor's item — the item is the ancestor's
                    w[(c.name, path, tc, tm)] += 1
                    if left and left[0] in self.crates[tc].modules[tm].items:
                        self.item_refs[(tc, tm, left[0])] += 1
        return dict(w)

    def short(self, crate: str) -> str:
        return self.crates[crate].short


def read_layers(script: Path) -> dict[str, int]:
    out = {}
    if script.exists():
        for m in re.finditer(r"^\s*([\w| -]+)\)\s*echo\s+(\d+)", script.read_text(), re.M):
            for name in m.group(1).split("|"):
                out[name.strip()] = int(m.group(2))
    return out


def label_of(path: tuple[str, ...]) -> str:
    return "::".join(path) or "(root)"


def tarjan(nodes: list, adj: dict) -> list[list]:
    index, low, on, stack, sccs, n = {}, {}, set(), [], [], [0]

    def go(v):
        index[v] = low[v] = n[0]
        n[0] += 1
        stack.append(v)
        on.add(v)
        for w in sorted(adj.get(v, ())):
            if w not in index:
                go(w)
                low[v] = min(low[v], low[w])
            elif w in on:
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            comp = []
            while True:
                w = stack.pop()
                on.discard(w)
                comp.append(w)
                if w == v:
                    break
            if len(comp) > 1:
                sccs.append(sorted(comp))

    sys.setrecursionlimit(10000)
    for v in nodes:
        if v not in index:
            go(v)
    return sccs


# ------------------------------------------------------------------- charts

LAYER_COLOR = ["blue", "teal", "purple", "orange", "yellow", "pink", "gray", "tan", "plain"]
PILLS_PER_SIDE = 7  # outside neighbours drawn around a container; the rest are in the side panel


def layer_color(layer: int | None) -> str:
    return "plain" if layer is None else LAYER_COLOR[min(layer, len(LAYER_COLOR) - 1)]


class Chart:
    """Nodes and arrows a view decided on; draw.py lays them out."""

    def __init__(self, title: str, subtitle: str = ""):
        self.title, self.subtitle = title, subtitle
        self.nodes: dict[str, dict] = {}  # key → title, meta, color, marks, tip, pill, mono
        self.edges: list[dict] = []  # a, b, w, label, red, dash, show_label, weighted
        self.notes: list[str] = []
        self.legend: list[tuple[str, str]] = []  # (css class of a swatch, text)
        self.ranks: dict | None = None
        self.pin: dict[str, str] = {}  # key → "top" | "bottom"
        self.max_row = 8

    def node(self, key, title, **kw):
        self.nodes[key] = {"title": title, **kw}

    def render(self) -> str:
        nodes = [draw.Node(k, n["title"], n.get("meta", ""), n.get("color", "plain"), n.get("marks", ()),
                           n.get("tip", ""), n.get("pill", False), n.get("mono", "")) for k, n in self.nodes.items()]
        edges = [e for e in self.edges if e["a"] in self.nodes and e["b"] in self.nodes]
        return draw.svg(draw.Layout(nodes, edges, self.ranks, self.max_row, pin=self.pin))

    def text(self) -> str:
        out = [f"## {self.title}", ""]
        if self.subtitle:
            out += [re.sub(r"<[^>]+>", "", self.subtitle), ""]
        out += [f"- {n}" for n in self.notes]
        if self.notes:
            out.append("")
        name = lambda k: self.nodes[k].get("name") or k
        out.append("Nodes: " + "; ".join(
            name(k) + (f" ({n['meta']})" if n.get("meta") else "") + (f" [{', '.join(sorted(n['marks']))}]" if n.get("marks") else "")
            for k, n in self.nodes.items()))
        out.append("")
        for e in self.edges:
            lab = e.get("label") if e.get("label") is not None else e.get("w")
            out.append(f"    {name(e['a'])} → {name(e['b'])}" + (f" ({lab})" if lab not in (None, "") else "")
                       + (" ✗" if e.get("red") else "") + (" ┄ outside" if e.get("dash") else ""))
        return "\n".join(out) + "\n"


def paths(n: int) -> str:
    return f"{n} path" if n == 1 else f"{n} paths"


def kloc(n: int) -> str:
    return f"{n / 1000:.1f}k" if n >= 1000 else str(n)


# --------------------------------------------------------------- the hierarchy


def key_of(ws: Workspace, crate: str, mod: tuple) -> str:
    """The page's name for a crate or module: `ops`, `ops::boolean::pave`."""
    return "::".join((ws.short(crate),) + tuple(mod))


def parse_target(ws: Workspace, spec: str) -> tuple[str, tuple]:
    segs = spec.replace("-", "_").split("::")
    s0 = segs[0]
    crate = ws.by_lib.get(s0) or ws.by_short.get(s0) or ws.by_short.get(s0.removeprefix("arris_"))
    if crate is None:
        sys.exit(f"archmap: no crate '{segs[0]}'")
    mod = tuple(segs[1:])
    if mod not in ws.crates[crate].modules:
        c, m, left = ws.walk(crate, (), list(mod))
        if left:
            sys.exit(f"archmap: {spec}: no module {'::'.join(left)} under {c}::{label_of(m)}")
        crate, mod = c, m
    return crate, mod


def children(crate: Crate, mod: tuple) -> list[tuple]:
    return sorted(p for p in crate.modules if len(p) == len(mod) + 1 and p[: len(mod)] == mod)


def subtree_loc(crate: Crate, mod: tuple) -> int:
    return sum(m.loc for p, m in crate.modules.items() if p[: len(mod)] == mod)


def one_step_out(ws: Workspace, crate: str, mod: tuple, oc: str, om: tuple, crate_deep: int = 1) -> str:
    """The other end of an arrow leaving (crate, mod), named one step out from it:
    the child of the deepest module both share; in another crate, its first
    `crate_deep` levels (0: the crate itself)."""
    if oc != crate:
        return key_of(ws, oc, om[:crate_deep])
    k = 0
    while k < min(len(om), len(mod)) and om[k] == mod[k]:
        k += 1
    return key_of(ws, oc, om[: k + 1])


def documented(ws: Workspace) -> set[tuple[str, tuple]]:
    """Modules a design doc names (by a path that resolves to them or below)."""
    seen = set()
    for _, _, hit in doc_paths(ws):
        if hit and hit[0] in ws.crates:
            c, mod = hit
            for k in range(1, len(mod) + 1):
                seen.add((c, mod[:k]))
    return seen


def doc_paths(ws: Workspace):
    """(doc:line, `span`, (crate, module) or None) for each `a::b` span in the design docs."""
    for rel in DOCS:
        f = ws.root / rel
        if not f.exists():
            continue
        for ln, line in enumerate(f.read_text().splitlines(), 1):
            for m in re.finditer(r"`((?:[a-z_][a-z0-9_]*::)+[A-Za-z_][A-Za-z0-9_]*)(?:\([^`]*\))?`", line):
                segs = m.group(1).split("::")
                yield f"{rel}:{ln}", m.group(1), resolve_doc(ws, segs)


def resolve_doc(ws: Workspace, segs: list[str]):
    """A doc path: `crate::…`, `short::…` (ops::measure), or a module path in some crate."""
    tries = []
    s0 = segs[0].replace("-", "_")
    if s0 in STD:
        return ("std", ())
    if s0 in ws.by_lib:
        tries.append((ws.by_lib[s0], segs[1:]))
    if s0 in ws.by_short and ws.by_short[s0] != "arris":
        tries.append((ws.by_short[s0], segs[1:]))
    tries += [(c, segs) for c in sorted(ws.crates)]
    for c in sorted(ws.crates):  # a nested module named by its tail: pave::build is boolean::pave::build
        for mod in sorted(ws.crates[c].modules):
            if len(mod) > 1 and mod[-1] == segs[0]:
                tries.append((c, list(mod[:-1]) + segs))
    for crate, rest in tries:
        if rest and (rest[0],) not in ws.crates[crate].modules and crate not in (ws.by_lib.get(s0), ws.by_short.get(s0)):
            continue
        c, mod, left = ws.walk(crate, (), rest)
        m = ws.crates[c].modules[mod]
        if not left:
            return (c, mod)
        if len(left) == 1 and (left[0] in m.items or ws._defines(c, mod, left[0])):
            return (c, mod)
        if len(left) == 2 and left[0][:1].isupper():  # Type::method
            return (c, mod)
    return None


# --------------------------------------------------------------------- views


def view_layers(ws: Workspace) -> tuple[Chart, list[str]]:
    ch = Chart("Crates and the layer rule",
               "Each crate depends only on crates below it. An arrow is a normal dependency; "
               "its width, how many paths the crate's source writes into the one below. Click a crate to open it.")
    findings = []
    for c in sorted(ws.crates.values(), key=lambda c: (-(c.layer or 0), c.name)):
        loc = sum(m.loc for m in c.modules.values())
        marks = []
        if c.layer is None:
            marks.append("breaks")
            findings.append(f"{c.name} has no layer in tools/check-layers.sh")
        if len(c.modules) > 1:
            marks.append("drill")
        ch.node(c.short, c.name, meta=f"layer {'?' if c.layer is None else c.layer} · {len(c.modules) - 1} modules · {kloc(loc)} lines",
                color=layer_color(c.layer), marks=marks, tip=f"{c.name}: {loc} non-blank lines in {len(c.modules) - 1} modules and lib.rs")
    for c in ws.crates.values():
        for dep, kind in sorted(c.deps):
            if dep not in ws.crates or kind == "build":
                continue
            r = ws.named.get((c.name, dep), 0)
            red = False
            if kind == "normal" and c.layer is not None and ws.crates[dep].layer is not None \
                    and ws.crates[dep].layer >= c.layer:
                red = True
                findings.append(f"forbidden edge {c.name} (layer {c.layer}) → {dep} (layer {ws.crates[dep].layer})")
            if kind == "normal" and r == 0:
                findings.append(f"{c.name} declares {dep} but no source path names it (unused dependency?)")
            if kind == "dev":
                continue
            ch.edges.append({"a": c.short, "b": ws.short(dep), "w": max(r, 1), "label": paths(r), "red": red,
                             "tip": f"{c.name} → {dep}: {paths(r)}"})
    normal = defaultdict(set)
    for e in ch.edges:
        normal[e["a"]].add(e["b"])

    def via(a, b):  # b reached from a through another dependency
        seen, todo = set(), [x for x in normal[a] if x != b]
        while todo:
            x = todo.pop()
            if x == b:
                return True
            if x not in seen:
                seen.add(x)
                todo += normal[x]
        return False

    implied = [e for e in ch.edges if not e["red"] and via(e["a"], e["b"])]
    ch.edges = [e for e in ch.edges if e not in implied]
    ch.notes.append("Not drawn, implied by a longer path: " + ", ".join(
        f"{e['a']} → {e['b']} ({e['label']})" for e in implied) + ".")
    ch.ranks = {c.short: (c.layer if c.layer is not None else 99) for c in ws.crates.values()}
    dev = sorted(f"{c.short} → {ws.short(d)}" for c in ws.crates.values() for d, k in c.deps if k == "dev" and d in ws.crates)
    ch.notes.append("Dev-dependencies cross layers by design and are not drawn: " + ", ".join(dev) + ".")
    ch.legend = [("sw-red", "forbidden edge"), ("sw-drill", "holds modules: click to open"), ("sw-w", "width: paths written")]
    return ch, findings


def view_container(ws: Workspace, crate: str, mod: tuple, docs: set, min_weight: int | None) -> tuple[Chart, list[str]]:
    """The children of a crate or module, and its neighbours one step out as pills."""
    c = ws.crates[crate]
    me = key_of(ws, crate, mod)
    own = "lib.rs" if not mod else c.modules[mod].file.name
    kids = children(c, mod)
    inside = lambda bc, bm: bc == crate and bm[: len(mod)] == mod
    node = lambda m: key_of(ws, crate, m[: len(mod) + 1])  # the child holding m, or me for my own file
    w: dict = defaultdict(int)
    ext_in: dict = defaultdict(int)
    ext_out: dict = defaultdict(int)
    for (ac, am, bc, bm), n in ws.edges.items():
        ai, bi = inside(ac, am), inside(bc, bm)
        if ai and bi:
            if node(am) != node(bm):
                w[(node(am), node(bm))] += n
        elif ai:
            ext_out[(node(am), one_step_out(ws, crate, mod, bc, bm, 0))] += n
        elif bi:
            ext_in[(one_step_out(ws, crate, mod, ac, am, 0), node(bm))] += n
    ch = Chart(f"{c.name}{'::' + label_of(mod) if mod else ''}",
               f"{len(kids)} child modules ({sum(1 for p in c.modules if p[: len(mod)] == mod and p != mod)} in all)"
               f"{f' in layer {c.layer}' if not mod else ''}, {kloc(subtree_loc(c, mod))} lines. "
               "A → B: A names paths defined in B; users above, what they use below. "
               "Pills are the neighbours outside, one step out. A stacked box holds modules: click to open it.")
    findings = []
    members = [key_of(ws, crate, k) for k in kids]
    if any(me in pair for t in (w, ext_in, ext_out) for pair in t):  # its own file, when it names or is named
        members.append(me)
    adj = defaultdict(set)
    for (a, b) in w:
        adj[a].add(b)
    sccs = tarjan(sorted(members), adj)
    comp_of = {v: i for i, comp in enumerate(sccs) for v in comp}
    for i, v in enumerate(sorted(members)):
        comp_of.setdefault(v, len(sccs) + i)
    for comp in sccs:
        kind = "cycle" if len(comp) <= SMALL_CYCLE else "tangle"
        names = [x.split("::")[-1] if x != me else own for x in comp]
        shown = names if len(names) <= 6 else names[:6] + [f"… {len(names) - 6} more"]
        findings.append(f"observe: {kind} in {me} ({len(comp)} modules): " + " ⇄ ".join(shown))
    # Transitive reduction over the cycle-condensed graph, then the edge budget.
    cadj = defaultdict(set)
    for (a, b) in w:
        if comp_of[a] != comp_of[b]:
            cadj[comp_of[a]].add(comp_of[b])
    reach: dict = {}

    def reachable(x):
        if x not in reach:
            reach[x] = set()
            for d in cadj[x]:
                reach[x] |= {d} | reachable(d)
        return reach[x]

    keep, implied = {}, 0
    for (a, b), n in w.items():
        ca, cb = comp_of[a], comp_of[b]
        if ca != cb and any(cb in reachable(x) for x in cadj[ca] if x != cb):
            implied += 1
            continue
        keep[(a, b)] = n
    mw = min_weight or 1
    while min_weight is None and sum(1 for n in keep.values() if n >= mw) > EDGE_BUDGET:
        mw += 1
    heaviest = {}
    for (a, b), n in keep.items():
        if n > heaviest.get(a, (0, None))[0]:
            heaviest[a] = (n, b)
    for k in kids:
        v = key_of(ws, crate, k)
        marks = []
        if comp_of[v] < len(sccs):
            marks.append("cycle" if len(sccs[comp_of[v]]) <= SMALL_CYCLE else "tangle")
        if (crate, k) not in docs:
            marks.append("undoc")
        inner = len([p for p in c.modules if len(p) > len(k) and p[: len(k)] == k])
        if inner:
            marks.append("drill")
        loc = subtree_loc(c, k)
        ch.node(v, k[-1], meta=f"{kloc(loc)} lines" + (" · pub" if c.modules[k].public else "")
                + (f" · {inner} inside" if inner else ""), marks=marks, name=v,
                tip=f"{v} — {loc} non-blank lines" + (f", {inner} modules inside" if inner else ""))
    if me in members:
        ch.node(me, own, meta=f"{kloc(c.modules[mod].loc)} lines · own file", color="tan", pill=True, name=me,
                marks=["cycle"] if comp_of[me] < len(sccs) and len(sccs[comp_of[me]]) <= SMALL_CYCLE else [],
                tip=f"{me}'s own file, {c.modules[mod].file.relative_to(ws.root)}")
    hidden = 0
    for (a, b), n in sorted(keep.items(), key=lambda x: (-x[1], x[0])):
        if n < mw and heaviest[a][1] != b:
            hidden += 1
            continue
        same = comp_of[a] == comp_of[b] and comp_of[a] < len(sccs)
        ch.edges.append({"a": a, "b": b, "w": n, "label": str(n), "red": same and len(sccs[comp_of[a]]) <= SMALL_CYCLE,
                         "tip": f"{a} → {b}: {paths(n)}"})
    # Outside neighbours: users on top, what is used at the bottom, the heaviest few each side.
    tot_in, tot_out = defaultdict(int), defaultdict(int)
    for (o, _), n in ext_in.items():
        tot_in[o] += n
    for (_, o), n in ext_out.items():
        tot_out[o] += n
    top = sorted(tot_in, key=lambda o: (-tot_in[o], o))[:PILLS_PER_SIDE]
    bottom = [o for o in sorted(tot_out, key=lambda o: (-tot_out[o], o)) if o not in top][:PILLS_PER_SIDE]
    for o in top + bottom:
        oc, om = parse_target(ws, o)
        ch.node(o, o if not om else om[-1], meta="crate" if not om else ("::".join(om[:-1]) or "module"),
                color=layer_color(ws.crates[oc].layer), pill=True, name=o, marks=["outside"],
                tip=f"{o}: names {me} {tot_in.get(o, 0)}×, named by it {tot_out.get(o, 0)}×")
        ch.pin[o] = "top" if o in top else "bottom"
    # A pill keeps its heaviest arrows only (up to 3, each a quarter of its paths): it says
    # where the outside comes in, the side panel says all of it.
    pill_cut = 0
    for side, table in (("in", ext_in), ("out", ext_out)):
        per = defaultdict(list)
        for (a, b), n in table.items():
            o, v = (a, b) if side == "in" else (b, a)
            if o in ch.nodes and v in ch.nodes:
                per[o].append((n, v))
        for o, lst in per.items():
            lst.sort(key=lambda x: (-x[0], x[1]))
            total = sum(n for n, _ in lst)
            kept = [x for i, x in enumerate(lst) if i == 0 or (i < 3 and x[0] * 4 >= total)]
            pill_cut += len(lst) - len(kept)
            for n, v in kept:
                a, b = (o, v) if side == "in" else (v, o)
                ch.edges.append({"a": a, "b": b, "w": n, "label": str(n), "dash": True, "tip": f"{a} → {b}: {paths(n)}"})
    rest_in, rest_out = len(tot_in) - len(top), len(set(tot_out) - set(top) - set(bottom))
    ch.legend = [("sw-drill", "holds modules: click to open"), ("sw-cycle", f"cycle (≤{SMALL_CYCLE} modules)"),
                 ("sw-tangle", "tangle (larger cycle)"), ("sw-undoc", "no design doc names it"),
                 ("sw-out", "outside, one step out"), ("sw-w", "width: paths written")]
    ch.notes.append(f"Not drawn: {implied} arrows implied by a longer path"
                    + (f", {hidden} arrows under {mw} paths (each module keeps its heaviest)" if hidden else "")
                    + (f", {pill_cut} light arrows to the pills" if pill_cut else "")
                    + (f", {rest_in} more users and {rest_out} more used outside (the side panel lists them)"
                       if rest_in or rest_out else "") + ".")
    if len(ch.edges) > EDGE_BUDGET + 2 * PILLS_PER_SIDE:
        ch.notes.append(f"Busy: {len(ch.edges)} arrows. Open a module to see less at once, or pass --min-weight.")
    return ch, findings


def details(ws: Workspace, docs: set) -> dict[str, dict]:
    """For every crate and module: size, file, who names it and what it names
    (one step out from it), and its most-named items — the side panel."""
    ins: dict = defaultdict(lambda: defaultdict(int))
    outs: dict = defaultdict(lambda: defaultdict(int))
    for (ac, am, bc, bm), n in ws.edges.items():
        for k in range(len(am) + 1):
            if not (bc == ac and bm[:k] == am[:k]):
                outs[(ac, am[:k])][one_step_out(ws, ac, am[:k], bc, bm)] += n
        for k in range(len(bm) + 1):
            if not (ac == bc and am[:k] == bm[:k]):
                ins[(bc, bm[:k])][one_step_out(ws, bc, bm[:k], ac, am)] += n
    items: dict = defaultdict(list)
    for (c, m, it), n in ws.item_refs.items():
        items[(c, m)].append((it, ws.crates[c].modules[m].items[it], n))
    top = lambda d, k=14: sorted(d.items(), key=lambda x: (-x[1], x[0]))[:k]
    out = {}
    for c in ws.crates.values():
        for mod, m in c.modules.items():
            inner = [p for p in c.modules if len(p) > len(mod) and p[: len(mod)] == mod]
            i, o = ins[(c.name, mod)], outs[(c.name, mod)]
            out[key_of(ws, c.name, mod)] = {
                "kind": "crate" if not mod else "module",
                "loc": subtree_loc(c, mod), "own": m.loc, "inside": len(inner),
                "file": str(m.file.relative_to(ws.root)), "pub": m.public,
                "layer": c.layer, "doc": bool(not mod or (c.name, mod) in docs),
                "ins": top(i), "ins_n": len(i), "outs": top(o), "outs_n": len(o),
                "items": [[a, ln, n] for a, ln, n in sorted(items[(c.name, mod)], key=lambda x: (-x[2], x[0]))[:12]],
                "items_n": len(m.items),
            }
    return out


def details_text(key: str, d: dict) -> str:
    out = [f"## {key}", "", f"{d['file']} · {d['loc']} lines ({d['own']} in its own file)"
           + (f" · {d['inside']} modules inside" if d["inside"] else ""), ""]
    out += [f"Named by ({d['ins_n']}):"] + [f"    {k} ({n})" for k, n in d["ins"]] + [""]
    out += [f"Names ({d['outs_n']}):"] + [f"    {k} ({n})" for k, n in d["outs"]] + [""]
    out += [f"Most-named items here ({d['items_n']} defined):"] + [f"    {a} :{ln} ({n})" for a, ln, n in d["items"]]
    return "\n".join(out) + "\n"


def view_drift(ws: Workspace, docs: set) -> tuple[dict, list[str]]:
    bad, ok = [], 0
    for where, span, hit in doc_paths(ws):
        if hit is None:
            bad.append((where, span))
        else:
            ok += 1
    undoc = []
    for c in sorted(ws.crates.values(), key=lambda c: c.layer or 0):
        top = sorted(p for p in c.modules if len(p) == 1)
        missing = [p[0] for p in top if (c.name, p) not in docs]
        undoc.append((c.name, missing, len(top)))
    findings = [f"{w}: `{s}` names no module or item in the workspace" for w, s in bad]
    return {"ok": ok, "bad": bad, "undoc": undoc}, findings


def drift_md(d: dict) -> str:
    lines = ["## Design docs against the module tree", "",
             f"{d['ok']} `a::b` paths in {', '.join(DOCS)} resolve; {len(d['bad'])} do not.", ""]
    lines += [f"- {w}: `{s}`" for w, s in d["bad"]] or ["- none"]
    lines += ["", "Top-level modules no design doc names by a path:", ""]
    lines += [f"- {c}: {', '.join(m)}" for c, m, _ in d["undoc"] if m] or ["- none"]
    return "\n".join(lines) + "\n"


def view_flow(ws: Workspace, spec_file: Path) -> tuple[Chart, list[str]]:
    spec = json.loads(spec_file.read_text())
    ch = Chart(spec.get("title", spec_file.stem), spec.get("subtitle", "Each box is bound to a module or item "
               "in the code; hover for its file:line. A red box does not resolve."))
    findings = []
    for n in spec["nodes"]:
        at, marks, where = n.get("at"), [], ""
        if at:
            segs = at.replace("-", "_").split("::")
            hit = resolve_doc(ws, segs)
            if hit is None or hit[0] not in ws.crates:
                marks.append("breaks")
                findings.append(f"flow node '{n['id']}' is bound to `{at}`, which does not resolve")
            else:
                c, mod = hit
                m = ws.crates[c].modules[mod]
                line = m.items.get(segs[-1])
                where = f"{m.file.relative_to(ws.root)}" + (f":{line}" if line else "")
        else:
            marks.append("breaks")
            findings.append(f"flow node '{n['id']}' is bound to nothing")
        ch.node(n["id"], n.get("label", n["id"]), mono=at or "unbound", marks=marks, color=n.get("color", "plain"),
                tip=f"{at} — {where}" if where else (at or ""), name=n["id"], where=where)
    for e in spec.get("edges", []):
        a, b, *rest = e
        ch.edges.append({"a": a, "b": b, "label": rest[0] if rest else "", "show_label": True, "weighted": False})
    ch.max_row = 4
    ch.notes += [f"{k} → {n['where']}" for k, n in ch.nodes.items() if n.get("where")]
    return ch, findings


# --------------------------------------------------------------------- write

PAGE_CSS = """
:root{--bg:#f6f7f9;--card:#ffffff;--ink:#1f2933;--muted:#66707d;--line:#e3e6ea;--accent:#2563eb;--bad:#c8323c;--warn:#c2620c;--chip:#eef1f5}
@media (prefers-color-scheme:dark){:root:not([data-theme=light]){--bg:#121418;--card:#1a1d22;--ink:#e6e8eb;--muted:#9aa3ae;--line:#2b3038;--accent:#7aa7ff;--bad:#ff6b73;--warn:#f59e4a;--chip:#252a31}}
:root[data-theme=dark]{--bg:#121418;--card:#1a1d22;--ink:#e6e8eb;--muted:#9aa3ae;--line:#2b3038;--accent:#7aa7ff;--bad:#ff6b73;--warn:#f59e4a;--chip:#252a31}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
.wrap{max-width:1500px;margin:0 auto;padding:28px 16px 64px}
header h1{font-size:26px;letter-spacing:-.01em;margin:0 0 4px}
header p{margin:0;color:var(--muted)}
a{color:var(--accent);text-decoration:none}a:hover{text-decoration:underline}
code{font:12.5px ui-monospace,SFMono-Regular,Menlo,monospace;background:var(--chip);padding:1px 5px;border-radius:4px}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px;margin:22px 0}
.stat{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.stat b{display:block;font-size:22px;font-variant-numeric:tabular-nums}
.stat span{color:var(--muted);font-size:13px}
.stat.bad b{color:var(--bad)}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px 20px;margin:16px 0}
.card h2{font-size:17px;margin:0 0 4px}
.card .sub{color:var(--muted);margin:0 0 12px;font-size:14px}
.findings{border-left:4px solid var(--bad)}
.findings li{margin:4px 0}
.findings.none{border-left-color:#2e9e62}
ul{padding-left:20px;margin:6px 0}
details summary{cursor:pointer;color:var(--muted);font-size:14px;margin-top:8px}
.crumbs{display:flex;flex-wrap:wrap;align-items:center;gap:4px;font-size:15px;margin:0 0 10px;position:sticky;top:0;background:var(--card);padding:6px 0;z-index:2}
.crumbs a{padding:2px 8px;border-radius:6px;background:var(--chip);color:var(--ink)}
.crumbs a:last-child{background:var(--ink);color:var(--card)}
.crumbs span{color:var(--muted)}
.mgrid{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:16px;align-items:start}
@media (max-width:960px){.mgrid{grid-template-columns:1fr}}
.canvas{position:relative;overflow:hidden;border:1px solid var(--line);border-radius:10px;background:var(--bg);height:74vh;touch-action:none;cursor:grab;user-select:none}
.canvas.drag{cursor:grabbing}
.canvas svg{display:block;width:100%;height:100%}
.zoom{position:absolute;right:8px;top:8px;display:flex;gap:4px;z-index:1}
.zoom button{font:600 15px/1 ui-sans-serif,system-ui,sans-serif;width:30px;height:30px;border:1px solid var(--line);border-radius:7px;background:var(--card);color:var(--ink);cursor:pointer}
.zoom button:hover{border-color:var(--accent)}
.canvas .n{cursor:pointer}
.side{border:1px solid var(--line);border-radius:10px;padding:12px 14px;font-size:13.5px;position:sticky;top:48px;max-height:80vh;overflow:auto}
.side h3{font:600 14px ui-monospace,SFMono-Regular,Menlo,monospace;margin:0 0 4px;word-break:break-all}
.side h4{font-size:12px;text-transform:uppercase;letter-spacing:.04em;color:var(--muted);margin:14px 0 4px}
.side .meta{color:var(--muted);margin:0}
.side ol{list-style:none;padding:0;margin:0}
.side li{display:flex;justify-content:space-between;gap:8px;padding:1px 0;font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}
.side li span{color:var(--muted);font-variant-numeric:tabular-nums}
.side li.more{color:var(--muted);font-family:inherit}
.legend{display:flex;flex-wrap:wrap;gap:14px;font-size:13px;color:var(--muted);margin:0 0 10px}
.legend i{display:inline-block;width:22px;height:12px;border-radius:3px;margin-right:6px;vertical-align:-1px;border:1.5px solid var(--muted)}
.sw-red{border:0!important;height:3px!important;background:var(--bad)}
.sw-cycle{border:2.5px solid var(--bad)!important}
.sw-tangle{border:2.5px solid var(--warn)!important}
.sw-undoc{width:9px!important;height:9px!important;border-radius:50%!important;border:1.5px solid var(--muted)!important}
.sw-drill{box-shadow:3px 3px 0 -1px var(--card),3px 3px 0 0 var(--muted)}
.sw-out{border-radius:8px!important;border-style:dashed!important}
.notes{color:var(--muted);font-size:13px;margin-top:10px}
.hint{color:var(--muted);font-size:12.5px;margin:8px 0 0}
pre.text{white-space:pre-wrap;font:12px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace;background:var(--chip);padding:12px;border-radius:8px;max-height:420px;overflow:auto}
table{border-collapse:collapse;width:100%;font-size:14px}
td,th{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line);vertical-align:top}
th{color:var(--muted);font-weight:600;font-size:13px}
.mods{color:var(--muted);font-size:13px}
"""

MAP_JS = r"""
const M=JSON.parse(document.getElementById('map-data').textContent);
const $=s=>document.querySelector(s);
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const up=k=>{const i=k.lastIndexOf('::');return k===''?null:(i<0?'':k.slice(0,i))};
function where(k){ // the chart that shows k, and k to select in it
  if(k in M.charts)return[k,k];
  let p=up(k);while(p!==null&&!(p in M.charts))p=up(p);
  return[p===null?'':p,k];
}
function crumbs(k){
  const parts=k?k.split('::'):[],out=['<a href="#">workspace</a>'];
  parts.forEach((p,i)=>out.push('<span>›</span><a href="#'+esc(parts.slice(0,i+1).join('::'))+'">'+esc(p)+'</a>'));
  $('#m-crumbs').innerHTML=out.join('');
}
function rows(list,n){
  return '<ol>'+list.map(([k,w])=>'<li><a href="#'+esc(k)+'">'+esc(k)+'</a><span>'+w+'</span></li>').join('')
   +(n>list.length?'<li class=more>+'+(n-list.length)+' more</li>':'')+'</ol>';
}
function panel(k){
  const d=M.nodes[k],el=$('#m-side');
  if(!d){el.innerHTML=M.help;return}
  el.innerHTML='<h3>'+esc(k)+'</h3><p class=meta>'+d.kind+(d.layer!=null?' · layer '+d.layer:'')+' · '+d.loc+' lines'
   +(d.inside?' ('+d.own+' in its own file) · '+d.inside+' modules inside':'')+(d.pub?' · pub':'')+(d.doc?'':' · no design doc names it')+'</p>'
   +'<p class=meta><code>'+esc(d.file)+'</code></p>'
   +'<h4>Named by · '+d.ins_n+'</h4>'+(d.ins.length?rows(d.ins,d.ins_n):'<p class=meta>nothing outside it</p>')
   +'<h4>Names · '+d.outs_n+'</h4>'+(d.outs.length?rows(d.outs,d.outs_n):'<p class=meta>nothing outside it</p>')
   +'<h4>Most-named items · '+d.items_n+' defined here</h4>'+(d.items.length?'<ol>'+d.items.map(([a,ln,n])=>
     '<li><code>'+esc(a)+'</code><span>:'+ln+' · '+n+'×</span></li>').join('')+'</ol>':'<p class=meta>none named from outside its file</p>');
}
function bind(svg){
  const nodes=svg.querySelectorAll('.n'),edges=svg.querySelectorAll('.e');
  nodes.forEach(n=>{
    n.addEventListener('mouseenter',()=>{const k=n.dataset.k;svg.classList.add('dim');n.classList.add('on');
      edges.forEach(e=>{if(e.dataset.a===k||e.dataset.b===k){e.classList.add('on');
        const o=e.dataset.a===k?e.dataset.b:e.dataset.a;nodes.forEach(m=>{if(m.dataset.k===o)m.classList.add('on')})}})});
    n.addEventListener('mouseleave',()=>{svg.classList.remove('dim');svg.querySelectorAll('.on').forEach(x=>x.classList.remove('on'))});
  });
}
function zoomable(cv){ // wheel zooms about the cursor, drag pans, double-click or ⤢ fits
  const svg=cv.querySelector('svg');if(!svg)return;
  const home=svg.getAttribute('viewBox').split(' ').map(Number);let vb=home.slice();
  cv.style.height=Math.max(320,Math.min(innerHeight*0.74,cv.clientWidth*home[3]/home[2]+16))+'px';
  const set=()=>svg.setAttribute('viewBox',vb.join(' '));
  const at=(cx,cy)=>new DOMPoint(cx,cy).matrixTransform(svg.getScreenCTM().inverse());
  const zoom=(k,p)=>{const w=Math.min(home[2]*3,Math.max(home[2]/30,vb[2]*k)),s=w/vb[2];
    p=p||{x:vb[0]+vb[2]/2,y:vb[1]+vb[3]/2};vb=[p.x-(p.x-vb[0])*s,p.y-(p.y-vb[1])*s,vb[2]*s,vb[3]*s];set()};
  const fit=()=>{vb=home.slice();set()};
  svg.addEventListener('wheel',e=>{e.preventDefault();zoom(Math.exp(e.deltaY*(e.deltaMode?0.05:0.0015)),at(e.clientX,e.clientY))},{passive:false});
  let d=null;
  svg.addEventListener('pointerdown',e=>{if(e.button!==0)return;d={x:e.clientX,y:e.clientY,vb:vb.slice(),m:svg.getScreenCTM().a,moved:false}});
  addEventListener('pointermove',e=>{if(!d)return;const dx=e.clientX-d.x,dy=e.clientY-d.y;
    if(!d.moved&&Math.hypot(dx,dy)<4)return;d.moved=true;cv.classList.add('drag');
    vb=[d.vb[0]-dx/d.m,d.vb[1]-dy/d.m,vb[2],vb[3]];set()});
  addEventListener('pointerup',()=>{if(!d)return;cv.dataset.dragged=d.moved?'1':'';d=null;cv.classList.remove('drag')});
  svg.addEventListener('dblclick',e=>{if(!e.target.closest('.n'))fit()});
  const bar=document.createElement('div');bar.className='zoom';
  [['+','zoom in',()=>zoom(1/1.4)],['−','zoom out',()=>zoom(1.4)],['⤢','fit',fit]].forEach(([t,l,f])=>{
    const b=document.createElement('button');b.textContent=t;b.title=l;b.addEventListener('click',e=>{e.stopPropagation();f()});bar.appendChild(b)});
  cv.appendChild(bar);
}
function show(k){
  const[ck,sel]=where(k),c=M.charts[ck],cv=$('#m-canvas');
  crumbs(ck);
  if(cv.dataset.k!==ck){cv.dataset.k=ck;cv.innerHTML=c.svg;bind(cv.querySelector('svg'));zoomable(cv);
    $('#m-title').textContent=c.title;$('#m-sub').innerHTML=c.sub;$('#m-legend').innerHTML=c.legend;
    $('#m-notes').innerHTML=c.notes;$('#m-text').textContent=c.text}
  cv.querySelectorAll('.n').forEach(n=>n.classList.toggle('sel',n.dataset.k===sel&&sel!==''));
  panel(sel);
}
$('#m-canvas').addEventListener('click',e=>{const n=e.target.closest('.n');
  if(n&&!e.currentTarget.dataset.dragged)location.hash=n.dataset.k});
document.querySelectorAll('svg.am.flow').forEach(s=>{bind(s);zoomable(s.parentNode)});
addEventListener('hashchange',()=>show(decodeURIComponent(location.hash.slice(1))));
show(decodeURIComponent(location.hash.slice(1))||M.start);
"""


def fmt(s: str) -> str:
    """Escape, then show `code` spans as code."""
    return re.sub(r"`([^`]+)`", r"<code>\1</code>", html.escape(s))


def legend_html(ch: Chart) -> str:
    width = ('<svg width="26" height="12" style="vertical-align:-1px;margin-right:6px"><path d="M1 3H25" stroke="#8a94a3" '
             'stroke-width="1.2"/><path d="M1 9H25" stroke="#8a94a3" stroke-width="4"/></svg>')
    return "".join(f'<span>{width if c == "sw-w" else f"<i class={c!r}></i>"}{html.escape(t)}</span>' for c, t in ch.legend)


def notes_html(ch: Chart) -> str:
    return f'<ul class="notes">{"".join(f"<li>{fmt(n)}</li>" for n in ch.notes)}</ul>' if ch.notes else ""


def flow_card(ch: Chart) -> str:
    sv = ch.render().replace('<svg class="am"', '<svg class="am flow"', 1)
    return (f'<section class="card"><h2>{html.escape(ch.title)}</h2><p class="sub">{fmt(ch.subtitle)}</p>'
            f'<div class="canvas">{sv}</div>{notes_html(ch)}</section>')


def drift_card(d: dict) -> str:
    rows = "".join(f"<tr><td><code>{html.escape(s)}</code></td><td>{html.escape(w)}</td></tr>" for w, s in d["bad"])
    bad = (f"<table><tr><th>Path in the doc</th><th>Where</th></tr>{rows}</table>" if rows
           else "<p>Every path resolves.</p>")
    und = "".join(f"<tr><td>{html.escape(c)}</td><td>{len(m)} of {n}</td><td class=mods>{html.escape(', '.join(m))}</td></tr>"
                  for c, m, n in d["undoc"])
    return (f'<section class="card"><h2>Design docs against the code</h2>'
            f'<p class="sub">{d["ok"]} <code>a::b</code> paths in {", ".join(DOCS)} resolve to a module or item; '
            f'{len(d["bad"])} do not.</p>{bad}'
            f'<details><summary>Top-level modules no design doc names by a path</summary>'
            f"<table><tr><th>Crate</th><th>Unnamed</th><th>Modules</th></tr>{und}</table></details></section>")


def write(ws: Workspace, out: Path, charts: dict[str, Chart], nodes: dict, start: str, flows: list[Chart],
          drift: dict, findings: list[str], cmd: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    for old in [*out.glob("*.svg"), *out.glob("*.mmd")]:
        old.unlink()  # a chart per file was the old layout; the page holds them all
    observe = [f.removeprefix("observe: ") for f in findings if f.startswith("observe: ")]
    findings = [f for f in findings if not f.startswith("observe: ")]
    report = [f"# Architecture map\n\n`{cmd}`\n", "## Findings (a rule breaks)\n"]
    report += [f"- {f}" for f in findings] or ["- none"]
    report += ["", "## Observations\n"] + ([f"- {f}" for f in observe] or ["- none"]) + [""]
    shown = [""] + ([start] if start else [k for k in charts if k and "::" not in k])
    report += [charts[k].text() for k in shown if k in charts]
    if start:
        report.append(details_text(start, nodes[start]))
    report += [ch.text() for ch in flows] + [drift_md(drift)]
    (out / "report.md").write_text("\n".join(report))
    data = {"start": start, "nodes": nodes,
            "help": ('<p class=meta>Click a box: a stacked one opens, any other is described here — '
                     'who names it, what it names, its most-named items. Pills are outside neighbours; '
                     'click one to go there. The trail above goes back up.</p>'),
            "charts": {k: {"title": ch.title, "sub": fmt(ch.subtitle), "svg": ch.render(), "legend": legend_html(ch),
                           "notes": notes_html(ch), "text": ch.text()} for k, ch in charts.items()}}
    blob = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    loc = sum(m.loc for c in ws.crates.values() for m in c.modules.values())
    stats = [("crates", len(ws.crates), False), ("modules", sum(len(c.modules) for c in ws.crates.values()), False),
             ("lines of library code", f"{loc / 1000:.0f}k", False),
             ("module → module paths", sum(ws.edges.values()), False),
             ("findings", len(findings), bool(findings)), ("observations", len(observe), False)]
    e = html.escape
    st = "".join(f'<div class="stat{" bad" if bad else ""}"><b>{e(str(v))}</b><span>{e(k)}</span></div>' for k, v, bad in stats)
    if findings:
        fl = ('<section class="card findings"><h2>Findings — a rule breaks</h2><ul>'
              + "".join(f"<li>{fmt(f)}</li>" for f in findings) + "</ul></section>")
    else:
        fl = '<section class="card findings none"><h2>No findings</h2><p class="sub">Every rule this map checks holds.</p></section>'
    if observe:
        fl += ('<section class="card"><details><summary><b>Observations</b> — not rules: input for a refactor review '
               f'({len(observe)})</summary><ul>' + "".join(f"<li>{fmt(o)}</li>" for o in observe) + "</ul></details></section>")
    the_map = ('<section class="card"><nav class="crumbs" id="m-crumbs"></nav><h2 id="m-title"></h2>'
               '<p class="sub" id="m-sub"></p><div class="legend" id="m-legend"></div>'
               '<div class="mgrid"><div><div class="canvas" id="m-canvas"></div>'
               '<p class="hint">Hover a box to follow its arrows, an arrow for its weight. Click a box to open or describe it; '
               'scroll to zoom, drag to pan, double-click the empty canvas to fit. Back and forward work.</p><div id="m-notes"></div>'
               '<details><summary>As text</summary><pre class="text" id="m-text"></pre></details></div>'
               '<aside class="side" id="m-side"></aside></div></section>')
    page = (f'<!doctype html><html lang="en"><head><meta charset="utf-8">'
            f'<meta name="viewport" content="width=device-width,initial-scale=1"><title>Arris architecture map</title>'
            f"<style>{PAGE_CSS}{draw.chart_css()}</style></head><body><div class=wrap>"
            f"<header><h1>Arris architecture map</h1><p>Drawn from the code, not the docs: every box is a crate or "
            f"module that exists, every arrow a path the source names. <code>{e(cmd)}</code></p></header>"
            f"<div class=stats>{st}</div>{fl}{the_map}{''.join(flow_card(f) for f in flows)}{drift_card(drift)}</div>"
            f'<script type="application/json" id="map-data">{blob}</script>'
            f"<script>{MAP_JS}</script></body></html>")
    (out / "index.html").write_text(page)
    graph = [{"from": key_of(ws, a, am), "to": key_of(ws, b, bm), "refs": n} for (a, am, b, bm), n in sorted(ws.edges.items())]
    (out / "graph.json").write_text(json.dumps(graph, indent=1))
    print(f"archmap: {len(charts)} charts, {len(findings)} finding(s), {len(observe)} observation(s) → "
          f"{out}/index.html{'#' + start if start else ''}, {out}/report.md")
    for f in findings:
        print(f"  finding: {f}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="?", default="", help="open the page here and write report.md on it: ops, ops::boolean")
    ap.add_argument("--flow", type=Path, action="append", default=[], help="FLOW.json: a pipeline bound to code (repeatable)")
    ap.add_argument("--min-weight", type=int, default=None, help="hide arrows lighter than N (default: fit the budget)")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--root-dir", type=Path, default=Path.cwd())
    a = ap.parse_args()
    root = a.root_dir.resolve()
    while not (root / "Cargo.toml").exists() or "[workspace]" not in (root / "Cargo.toml").read_text():
        if root.parent == root:
            sys.exit("archmap: run inside the arris workspace")
        root = root.parent
    ws = Workspace(root)
    start = key_of(ws, *parse_target(ws, a.target)) if a.target else ""
    docs = documented(ws)
    charts: dict[str, Chart] = {}
    ch, findings = view_layers(ws)
    charts[""] = ch
    for c in sorted(ws.crates.values(), key=lambda c: (c.layer or 0, c.name)):
        for mod in sorted(c.modules, key=lambda p: (len(p), p)):
            if children(c, mod):
                ch, f = view_container(ws, c.name, mod, docs, a.min_weight)
                charts[key_of(ws, c.name, mod)] = ch
                findings += f
    flows = []
    for spec in a.flow:
        ch, f = view_flow(ws, spec)
        flows.append(ch)
        findings += f
    drift, f = view_drift(ws, docs)
    findings += f
    write(ws, a.out or root / "target/archmap", charts, details(ws, docs), start, flows, drift, findings,
          " ".join(["archmap.py", *sys.argv[1:]]))


if __name__ == "__main__":
    main()
