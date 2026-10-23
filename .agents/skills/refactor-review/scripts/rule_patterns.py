"""Patterns `.agents/rules/kernel.md` speaks to, counted in library code
outside `arris-debug` (dev tooling), with the files that hold the most.
A count is a lead: an `expect` on a kernel invariant is allowed, and a
`_ =>` is a rule break only on a dispatch over a geometry enum.

Usage: python3 rule_patterns.py   (from the repository root)
"""

import collections
import os
import re
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _rust import library_files  # noqa: E402

PATTERNS = {
    "unwrap()": r"\.unwrap\(\)",
    "expect(": r"\.expect\(",
    "HashMap/HashSet": r"\bHash(?:Map|Set)\b",
    "_ => arm": r"^\s*_\s*(?:if\b[^=]*)?=>",
    "unwrap_or(0)": r"unwrap_or\((?:0|0\.0)\)",
    "panic!/unreachable!/todo!": r"\b(?:panic|unreachable|todo|unimplemented)!",
    "allow(too_many_arguments)": r"allow\(clippy::too_many_arguments",
}
GEOM_ENUM = re.compile(r"\b(?:Surface|Curve|Curve2|SurfaceKind|CurveKind|GeomKind)::[A-Z]")

counts = {k: collections.Counter() for k in PATTERNS}
geom_wild = []
for path, src in library_files(skip=("arris-debug",)):
    lines = src.split("\n")
    for i, line in enumerate(lines):
        for key, pat in PATTERNS.items():
            if re.search(pat, line):
                counts[key][path] += 1
        if re.search(PATTERNS["_ => arm"], line):
            if GEOM_ENUM.search("\n".join(lines[max(0, i - 12) : i])):
                geom_wild.append(f"{path}:{i + 1}")

for key, by_file in counts.items():
    total = sum(by_file.values())
    worst = ", ".join(f"{p} {n}" for p, n in by_file.most_common(4))
    print(f"{key:28} {total:4}   {worst}")
print("== _ => arms within 12 lines of a geometry enum variant (read each)")
for at in geom_wild:
    print(at)
