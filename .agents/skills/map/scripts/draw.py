"""Layered layout and SVG for archmap charts. Stdlib only, deterministic.

A chart is nodes and arrows; `a → b` means `a` uses `b`, so `b` sits lower:
the foundation at the bottom, as in the layer rule. Cycles are broken for the
layout only (Eades' greedy order, lightest arrows first), and the arrows
that point back up are drawn red or dashed. Long arrows run through dummy
points; rows are ordered by barycentre sweeps; wide rows wrap.
"""

from __future__ import annotations

import html
import math
from collections import defaultdict

FONT = 13
CHAR = 7.3  # average advance of the 13px UI font, bold a little wider
ROW_GAP = 64
COL_GAP = 22
MARGIN = 24

# fill, ink — light and dark
PALETTE = {
    "plain": (("#ffffff", "#1f2933"), ("#1d2128", "#e6e8eb")),
    "blue": (("#e3edfd", "#1d3f86"), ("#1b2a44", "#b9cdf5")),
    "teal": (("#dcf3ef", "#11554c"), ("#15332f", "#a6e3d8")),
    "orange": (("#fdebdc", "#7a3410"), ("#3a2416", "#f5c3a0")),
    "purple": (("#ece6fb", "#4a2a8c"), ("#2a2240", "#cfc0f5")),
    "yellow": (("#fbf3cf", "#6a4c0c"), ("#352d14", "#ecd88f")),
    "pink": (("#fbe3ef", "#7d1f4c"), ("#3a1c2b", "#f2b6d2")),
    "tan": (("#f2ece2", "#57432a"), ("#2f2a22", "#dccbb0")),
    "gray": (("#eef0f3", "#2c3540"), ("#252a31", "#c9ced6")),
}


def text_w(s: str, bold: bool = False, size: int = FONT) -> float:
    return len(s) * CHAR * size / FONT * (1.06 if bold else 1.0)


class Node:
    def __init__(self, key, title, meta="", color="plain", marks=(), tip="", pill=False, mono=""):
        self.key, self.title, self.meta, self.color = key, title, meta, color
        self.marks, self.tip, self.pill, self.mono = set(marks), tip, pill, mono
        lines = [text_w(title, True)] + ([text_w(meta, size=11)] if meta else []) + \
                ([text_w(mono, size=11) * 1.08] if mono else [])
        self.w = max(84.0, max(lines) + 26)
        self.h = 22 + 16 * (len(lines) - 1) + 10
        self.x = self.y = 0.0  # centre
        self.dummy = False


class Dummy:
    def __init__(self, key):
        self.key, self.w, self.h, self.x, self.y, self.dummy = key, 0.0, 0.0, 0.0, 0.0, True


def eades(keys, edges):
    """An order with few arrows pointing backwards; heavy arrows kept forward."""
    out, inn = defaultdict(dict), defaultdict(dict)
    for a, b, w in edges:
        out[a][b] = out[a].get(b, 0) + w
        inn[b][a] = inn[b].get(a, 0) + w
    alive = set(keys)
    s1, s2 = [], []
    while alive:
        changed = True
        while changed:
            changed = False
            for v in sorted(alive):
                if not any(x in alive for x in out[v]):
                    s2.insert(0, v)
                    alive.discard(v)
                    changed = True
            for v in sorted(alive):
                if not any(x in alive for x in inn[v]):
                    s1.append(v)
                    alive.discard(v)
                    changed = True
        if alive:
            v = max(sorted(alive), key=lambda v: sum(w for x, w in out[v].items() if x in alive)
                    - sum(w for x, w in inn[v].items() if x in alive))
            s1.append(v)
            alive.discard(v)
    return s1 + s2


class Layout:
    def __init__(self, nodes: list[Node], edges: list[dict], ranks: dict | None = None,
                 max_row: int = 8, pin: dict | None = None):
        """`ranks` fixes every row (the layer chart); otherwise rows come from the
        arrows, and `pin` puts some boxes in a row above ("top") or below ("bottom") all others."""
        self.nodes = {n.key: n for n in nodes}
        self.edges = edges
        pin = pin or {}
        keys = [n.key for n in nodes if n.key not in pin]
        pairs = [(e["a"], e["b"], e.get("w", 1)) for e in edges if e["a"] != e["b"] and e["a"] not in pin and e["b"] not in pin]
        if ranks is None:
            pos = {v: i for i, v in enumerate(eades(keys, pairs))}
            down = defaultdict(list)
            for a, b, _ in pairs:
                if pos[a] < pos[b]:
                    down[a].append(b)
            ranks = {}

            def rank(v):
                if v not in ranks:
                    ranks[v] = 0
                    ranks[v] = 1 + max((rank(w) for w in down[v]), default=-1)
                return ranks[v]

            for v in keys:
                rank(v)
            hi = max(ranks.values(), default=0) + 1
            for v, side in pin.items():
                ranks[v] = hi if side == "top" else -1
            keys = [n.key for n in nodes]
        # Rows, top = highest rank; wide rows wrap into several.
        by_rank = defaultdict(list)
        for v in keys:
            by_rank[ranks[v]].append(v)
        self.levels: list[list] = []
        level_of = {}
        for r in sorted(by_rank, reverse=True):
            row = sorted(by_rank[r])
            chunks = [row[i:i + max_row] for i in range(0, len(row), max_row)] or [[]]
            for ch in chunks:
                for v in ch:
                    level_of[v] = len(self.levels)
                self.levels.append(list(ch))
        # Segments through dummies.
        self.routes = []
        self.items = dict(self.nodes)
        nd = 0
        for e in edges:
            a, b = e["a"], e["b"]
            la, lb = level_of[a], level_of[b]
            if la == lb:
                self.routes.append((e, [a, b], "flat"))
                continue
            up = la > lb  # an arrow pointing upwards
            top, bot = (b, a) if up else (a, b)
            lt, lbt = min(la, lb), max(la, lb)
            path = [top]
            for L in range(lt + 1, lbt):
                d = Dummy(f"~{nd}")
                nd += 1
                self.items[d.key] = d
                self.levels[L].append(d.key)
                level_of[d.key] = L
                path.append(d.key)
            path.append(bot)
            self.routes.append((e, path, "up" if up else "down"))
        self.level_of = level_of
        self._order()
        self._place()

    def _neighbours(self):
        above, below = defaultdict(list), defaultdict(list)
        for _, path, kind in self.routes:
            if kind == "flat":
                continue
            for u, v in zip(path, path[1:]):
                below[u].append(v)
                above[v].append(u)
        return above, below

    def _order(self):
        above, below = self._neighbours()
        for sweep in range(12):
            rng = range(1, len(self.levels)) if sweep % 2 == 0 else range(len(self.levels) - 2, -1, -1)
            for L in rng:
                ref = self.levels[L - 1] if sweep % 2 == 0 else self.levels[L + 1]
                idx = {v: i for i, v in enumerate(ref)}
                nb = above if sweep % 2 == 0 else below
                cur = {v: i for i, v in enumerate(self.levels[L])}

                def bary(v):
                    xs = [idx[u] for u in nb[v] if u in idx]
                    return sum(xs) / len(xs) if xs else cur[v]

                self.levels[L].sort(key=lambda v: (bary(v), cur[v]))

    def _place(self):
        it = self.items
        y = MARGIN
        for row in self.levels:
            h = max((it[v].h for v in row), default=0)
            h = max(h, 30)
            for v in row:
                it[v].y = y + h / 2
            y += h + ROW_GAP
        self.height = y - ROW_GAP + MARGIN

        def sep(u, v):
            return (it[u].w + it[v].w) / 2 + (COL_GAP if not (it[u].dummy and it[v].dummy) else 8)

        for row in self.levels:
            x = MARGIN
            for i, v in enumerate(row):
                if i == 0:
                    x = MARGIN + it[v].w / 2
                else:
                    x += sep(row[i - 1], v)
                it[v].x = x
        above, below = self._neighbours()
        for sweep in range(16):
            order = range(len(self.levels)) if sweep % 2 == 0 else range(len(self.levels) - 1, -1, -1)
            for L in order:
                row = self.levels[L]
                if not row:
                    continue
                want = []
                for v in row:
                    nb = above[v] + below[v]
                    want.append(sum(it[u].x for u in nb) / len(nb) if nb else it[v].x)
                xs = list(want)
                for i in range(1, len(row)):  # keep order and spacing, then centre the shift
                    xs[i] = max(xs[i], xs[i - 1] + sep(row[i - 1], row[i]))
                for i in range(len(row) - 2, -1, -1):
                    xs[i] = min(xs[i], xs[i + 1] - sep(row[i], row[i + 1]))
                for i in range(1, len(row)):
                    xs[i] = max(xs[i], xs[i - 1] + sep(row[i - 1], row[i]))
                shift = sum(w - x for w, x in zip(want, xs)) / len(row)
                for v, x in zip(row, xs):
                    it[v].x = x + shift * 0.5
        lo = min((it[v].x - it[v].w / 2 for row in self.levels for v in row), default=0)
        for v in it.values():
            v.x += MARGIN - lo
        self.width = max((v.x + v.w / 2 for v in it.values()), default=0) + MARGIN


def _ports(lay: Layout):
    """Spread the arrows leaving and entering a box over its edge, ordered by where they go."""
    it = lay.items
    outs, ins = defaultdict(list), defaultdict(list)
    for k, (e, path, kind) in enumerate(lay.routes):
        if kind == "flat":
            continue
        first, second, last, before = path[0], path[1], path[-1], path[-2]
        outs[first].append((it[second].x, k))
        ins[last].append((it[before].x, k))
    port = {}
    for side, table in (("out", outs), ("in", ins)):
        for v, lst in table.items():
            lst.sort()
            n = it[v]
            span = min(n.w * 0.6, 14 * (len(lst) - 1))
            for i, (_, k) in enumerate(lst):
                off = 0 if len(lst) == 1 else -span / 2 + span * i / (len(lst) - 1)
                port[(side, k)] = n.x + off
    return port


def _curve(pts):
    d = f"M{pts[0][0]:.1f},{pts[0][1]:.1f}"
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
        dy = (y2 - y1) / 2
        d += f" C{x1:.1f},{y1 + dy:.1f} {x2:.1f},{y2 - dy:.1f} {x2:.1f},{y2:.1f}"
    return d


STYLE = """
.am{--ink:#1f2933;--muted:#66707d;--edge:#8a94a3;--bad:#d12f3a;--warn:#d9730d;--hi:#2563eb;--bg:transparent;
 font-family:ui-sans-serif,system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
@media (prefers-color-scheme:dark){.am{--ink:#e6e8eb;--muted:#9aa3ae;--edge:#6b7685;--bad:#ff6b73;--warn:#f59e4a;--hi:#7aa7ff}}
.am .e path{fill:none;stroke:var(--edge);opacity:.55;transition:opacity .12s}
.am .e.red path{stroke:var(--bad);opacity:.9}
.am .e.dash path{stroke-dasharray:5 4}
.am .e text,.am .el{font-size:11px;fill:var(--muted)}
.am .e text{opacity:0;paint-order:stroke;stroke:var(--card,#fff);stroke-width:3px}
.am .n rect{stroke-width:1.2}
.am .n .t{font-weight:650;font-size:13px}
.am .n .m{font-size:11px;fill-opacity:.75}
.am .n .mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:10.5px;fill-opacity:.8}
.am .n{cursor:default}
.am.dim .e path{opacity:.08}.am.dim .n{opacity:.3}
.am.dim .e.on path{opacity:1;stroke:var(--hi)}.am.dim .e.on.red path{stroke:var(--bad)}
.am.dim .e.on text{opacity:1}.am.dim .n.on{opacity:1}
"""


def svg(lay: Layout) -> str:
    it = lay.items
    port = _ports(lay)
    W, H = math.ceil(lay.width), math.ceil(lay.height)
    o = [f'<svg class="am" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
         f'role="img">']
    o.append('<defs>'
             '<marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="9" markerHeight="9" markerUnits="userSpaceOnUse" orient="auto-start-reverse">'
             '<path d="M0,1 L10,5 L0,9 z" fill="context-stroke"/></marker></defs>')
    o.append("<g>")
    for k, (e, path, kind) in enumerate(lay.routes):
        a, b = it[e["a"]], it[e["b"]]
        w = e.get("w", 1)
        sw = min(5.0, 1.1 + 0.9 * math.log2(1 + w)) if e.get("weighted", True) else 1.6
        cls = "e" + (" red" if e.get("red") else "") + (" dash" if e.get("dash") else "")
        if kind == "flat":
            x1, x2 = a.x, b.x
            yt = a.y - a.h / 2
            mid = (x1 + x2) / 2
            lift = 26 + abs(x2 - x1) * 0.12
            d = f"M{x1:.1f},{yt:.1f} C{x1:.1f},{yt - lift:.1f} {x2:.1f},{yt - lift:.1f} {x2:.1f},{yt - 2:.1f}"
            lx, ly = mid, yt - lift * 0.75
        else:
            pts = [(port[("out", k)], it[path[0]].y + it[path[0]].h / 2)]
            pts += [(it[v].x, it[v].y) for v in path[1:-1]]
            pts.append((port[("in", k)], it[path[-1]].y - it[path[-1]].h / 2 - 2))
            if kind == "up":  # drawn from the lower box up to the upper one
                pts = list(reversed(pts))
            d = _curve(pts)
            m = pts[len(pts) // 2] if len(pts) > 2 else ((pts[0][0] + pts[1][0]) / 2, (pts[0][1] + pts[1][1]) / 2)
            lx, ly = m
        tip = html.escape(e.get("tip") or f"{e['a']} → {e['b']}" + (f"  ({w})" if e.get("weighted", True) else ""))
        label = e.get("label")
        always = e.get("show_label")
        o.append(f'<g class="{cls}" data-a="{html.escape(e["a"])}" data-b="{html.escape(e["b"])}">'
                 f'<title>{tip}</title><path d="{d}" stroke-width="{sw:.2f}" marker-end="url(#ah)"/>'
                 + (f'<text x="{lx:.1f}" y="{ly + 4:.1f}" text-anchor="middle"'
                    f'{" style=\"opacity:1\"" if always else ""}>{html.escape(str(label))}</text>' if label else "")
                 + "</g>")
    o.append("</g><g>")
    for n in lay.nodes.values():
        cls = " ".join(["n", f"c-{n.color}"] + sorted(n.marks))
        x, y = n.x - n.w / 2, n.y - n.h / 2
        rx = n.h / 2 if n.pill else 7
        if "drill" in n.marks:  # a card behind the box: it holds modules
            o.append(f'<rect class="stack" x="{x + 4:.1f}" y="{y + 4:.1f}" width="{n.w:.1f}" height="{n.h:.1f}" rx="{rx:.1f}"/>')
        o.append(f'<g class="{cls}" data-k="{html.escape(n.key)}"><title>{html.escape(n.tip or n.key)}</title>'
                 f'<rect class="box" x="{x:.1f}" y="{y:.1f}" width="{n.w:.1f}" height="{n.h:.1f}" rx="{rx:.1f}"/>')
        if "undoc" in n.marks:  # a hollow dot: no design doc names it
            o.append(f'<circle class="ud" cx="{x + n.w - 8:.1f}" cy="{y + 8:.1f}" r="3"/>')
        ty = y + 19
        o.append(f'<text class="t" x="{n.x:.1f}" y="{ty:.1f}" text-anchor="middle">{html.escape(n.title)}</text>')
        if n.meta:
            ty += 16
            o.append(f'<text class="m" x="{n.x:.1f}" y="{ty:.1f}" text-anchor="middle">{html.escape(n.meta)}</text>')
        if n.mono:
            ty += 16
            o.append(f'<text class="mono" x="{n.x:.1f}" y="{ty:.1f}" text-anchor="middle">{html.escape(n.mono)}</text>')
        o.append("</g>")
    o.append("</g></svg>")
    return "".join(o)


def chart_css() -> str:
    css = STYLE
    for name, ((fl, ik), (fd, idk)) in PALETTE.items():
        light = f".am .n.c-{name} rect{{fill:{fl};stroke:{ik}55}}.am .n.c-{name} text{{fill:{ik}}}"
        dark = f".am .n.c-{name} rect{{fill:{fd};stroke:{idk}66}}.am .n.c-{name} text{{fill:{idk}}}"
        css += light + "\n"
        css += "@media (prefers-color-scheme:dark){" + dark.replace(".am", ":root:not([data-theme=light]) .am") + "}\n"
        css += dark.replace(".am", ":root[data-theme=dark] .am") + "\n"
    return css + MARKS


MARKS = """
.am .n.cycle rect.box{stroke:var(--bad);stroke-width:2.6}
.am .n.tangle rect.box{stroke:var(--warn);stroke-width:2.2}
.am .n.breaks rect.box{stroke:var(--bad);stroke-width:2.6;fill:#ffecee}
.am .n.breaks text{fill:#a1121d}
.am .n.drill rect.box{stroke-width:1.8}
.am .n.outside rect.box{stroke-dasharray:4 3;stroke-width:1.3}
.am .n.sel rect.box{stroke:var(--hi);stroke-width:3}
.am .n:hover rect.box{stroke:var(--hi)}
.am .n .ud{fill:none;stroke:var(--muted);stroke-width:1.3}
"""
