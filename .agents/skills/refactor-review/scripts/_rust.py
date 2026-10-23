"""Shared reading of the workspace's Rust sources for the review scripts.

Library code is `crates/*/src/**/*.rs` cut at its `#[cfg(test)] mod`, with
`src/**/tests.rs` (out-of-line unit tests) left out. Comments and string
literals are blanked before anything is matched, so a pattern in a doc
comment does not count. Line numbers stay those of the file.
"""

import os
import re

TEST_MOD = re.compile(r"#\[cfg\(test\)\]\s*mod\s+\w+\s*\{")
FN = re.compile(r"\bfn\s+(\w+)")


def blank(src):
    """Comments and string literals replaced by spaces, newlines kept."""
    out = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            j = src.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif src.startswith("/*", i):
            j = src.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append(re.sub(r"[^\n]", " ", src[i:j]))
            i = j
        elif c == '"':
            j = i + 1
            while j < n and src[j] != '"':
                j += 2 if src[j] == "\\" else 1
            j = min(j + 1, n)
            out.append('"' + re.sub(r"[^\n]", " ", src[i + 1 : j - 1]) + '"')
            i = j
        else:
            out.append(c)
            i += 1
    return "".join(out)


def library_files(root="crates", skip=()):
    """(path, blanked library source) for every library file under `root`."""
    for d, _, files in sorted(os.walk(root)):
        if "/src" not in d + "/" or "/target" in d:
            continue
        if any(f"/{s}/" in d + "/" for s in skip):
            continue
        for f in sorted(files):
            if not f.endswith(".rs") or f == "tests.rs":
                continue
            p = os.path.join(d, f)
            with open(p, encoding="utf-8") as h:
                src = h.read()
            m = TEST_MOD.search(src)
            if m:
                src = src[: m.start()]
            yield p, blank(src)


def functions(src):
    """(name, first line, last line, body text) of every fn with a body."""
    for m in FN.finditer(src):
        brace = src.find("{", m.end())
        semi = src.find(";", m.end())
        if brace < 0 or 0 <= semi < brace:
            continue
        depth, k = 0, brace
        while k < len(src):
            if src[k] == "{":
                depth += 1
            elif src[k] == "}":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        first = src.count("\n", 0, m.start()) + 1
        last = src.count("\n", 0, k) + 1
        yield m.group(1), first, last, src[m.start() : k + 1]
