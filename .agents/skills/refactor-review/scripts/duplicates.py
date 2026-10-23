"""Copies across library files: function bodies identical up to whitespace
and the function's own name, and constants of one name and value defined
in several files. A hit is a lead to read, not a finding.

Usage: python3 duplicates.py [min_lines=4]   (from the repository root)
"""

import collections
import hashlib
import os
import re
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _rust import functions, library_files  # noqa: E402

min_lines = int(sys.argv[1]) if len(sys.argv) > 1 else 4
CONST = re.compile(r"^\s*(?:pub(?:\([a-z]+\))?\s+)?const\s+([A-Z0-9_]+)\s*:\s*([^=]+)=\s*([^;]+);", re.M)

bodies = collections.defaultdict(list)
consts = collections.defaultdict(list)
for path, src in library_files():
    for name, first, last, body in functions(src):
        if last - first + 1 < min_lines:
            continue
        key = re.sub(r"\s+", "", re.sub(r"^fn\s+\w+", "fn", body))
        digest = hashlib.sha1(key.encode()).hexdigest()
        bodies[digest].append((last - first + 1, f"{path}:{first} {name}"))
    for m in CONST.finditer(src):
        line = src.count("\n", 0, m.start()) + 1
        sig = f"{m.group(1)}: {m.group(2).strip()} = {m.group(3).strip()}"
        consts[sig].append(f"{path}:{line}")

print("== identical function bodies in more than one file")
for hits in sorted(bodies.values(), key=lambda h: -h[0][0]):
    files = {h[1].split(":")[0] for h in hits}
    if len(files) > 1:
        print(f"{hits[0][0]:4} lines: " + ", ".join(h[1] for h in hits))

print("== constants of one name and value in more than one file")
for sig, where in sorted(consts.items(), key=lambda kv: -len(kv[1])):
    if len({w.split(":")[0] for w in where}) > 1:
        print(f"{sig}  <- " + ", ".join(where))
