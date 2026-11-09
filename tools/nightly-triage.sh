#!/usr/bin/env bash
# Sorts a nightly job's failures into new and known by signature (ADR-0086
# §3-5, GLOSSARY.md "Signature"), writes the job summary and sets the exit
# code: 1 for a new signature, a missing or unreadable JUnit file or a
# missing known list; 0 otherwise. A passing test that a known entry lists is
# quiet.
#
#   tools/nightly-triage.sh <junit.xml> <known.txt>
#
# A known list line is `<slug> <source> <class> <key...>`: the source is the
# nextest classname and the test's name with its shard suffix dropped
# (`arris-ops::blend_prop::a_property`); the key is `-` for a `timeout` or an
# `abort`. `#` lines and blanks are comments. The summary goes to
# $GITHUB_STEP_SUMMARY when it is set, and always to stdout.
set -euo pipefail
if [ $# -ne 2 ]; then
  echo "usage: tools/nightly-triage.sh <junit.xml> <known.txt>" >&2
  exit 2
fi
exec python3 -I - "$1" "$2" <<'PY'
import os, re, sys
import xml.etree.ElementTree as ET

junit, known_path = sys.argv[1], sys.argv[2]
out = []

def finish(code):
    text = "\n".join(out) + "\n"
    sys.stdout.write(text)
    path = os.environ.get("GITHUB_STEP_SUMMARY")
    if path:
        with open(path, "a") as f:
            f.write(text)
    sys.exit(code)

if not os.path.isfile(junit):
    out.append(f"## Nightly triage\n\nRED: no JUnit file at `{junit}`; the build or the run failed before it wrote one.")
    finish(1)
try:
    root = ET.parse(junit).getroot()
except ET.ParseError as e:
    out.append(f"## Nightly triage\n\nRED: `{junit}` does not parse ({e}).")
    finish(1)
if not os.path.isfile(known_path):
    out.append(f"## Nightly triage\n\nRED: no known list at `{known_path}`.")
    finish(1)

known = {}  # (source, class, key) -> slug
for line in open(known_path):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    parts = line.split(None, 3)
    if len(parts) < 4:
        out.append(f"## Nightly triage\n\nRED: malformed known line `{line}` (want `<slug> <source> <class> <key>`).")
        finish(1)
    slug, source, cls, key = parts
    known[(source, cls, key.strip())] = slug

HARNESS = ("prop.rs", "differential.rs")
SHARD = re.compile(r"::(?:shard_|[a-z])\d+$")
BUDGET = re.compile(r"prop-budget: ran (\d+) of (\d+) cases in ([0-9.]+) s \(stop: (\w+)\)")
PANIC = re.compile(r"panicked at ([^\s:]+):(\d+)")
FAILED = re.compile(r"property (failed|aborted)(?: in shard \d+ of \d+)?(?:, chunk \d+)?: ?")
CASE = re.compile(r"^case \d+: (Disagree\([^)\s]*\)|Internal|CheckerViolation|Panic)(?=[:\s]|$)(.*)$", re.M)
CHECKER = re.compile(r"checker report:(.{0,400})", re.S)
ROW = re.compile(r"(?:^|\s)([A-Z]\d+)\s")
TOKEN = re.compile(r"\b(?:[A-Z]\w*::[A-Z]\w*|(?!Some\b|Ok\b|Err\b|Box\b|Vec\b)[A-Z]\w*\([A-Z]\w*\))")
REPLAY = re.compile(r"ARRIS_PROPTEST_SEED=\S+ ARRIS_PROPTEST_CHUNK=\d+\S*(?: \S+=\S+)*")

def message(text):
    """The failure's own message: after `property failed ...:` and before the
    printed input, else the text after the panic line."""
    m = FAILED.search(text)
    if m:
        body = text[m.end():]
        return body.split("\nminimal failing input", 1)[0].strip()
    lines = text.strip().splitlines()
    if lines and "panicked at" in lines[0]:
        lines = lines[1:]
    return "\n".join(lines).strip()

def stem(msg):
    m = re.search(r"\d", msg)
    s = msg[: m.start()] if m else msg
    s = re.sub(r"\s+", " ", s)
    return re.sub(r"[\s\W_]+$", "", s) or "-"

def classify(kind, text):
    """The signatures of one failing test, first match wins; a differential
    run's case lines each give one."""
    if kind == "timeout":
        return [("timeout", "-")]
    if "property aborted" in text:
        return [("abort", "-")]
    for m in PANIC.finditer(text):
        if os.path.basename(m.group(1)) not in HARNESS:
            return [("panic", f"{m.group(1)}:{m.group(2)}")]
    m = CHECKER.search(text)
    if m:
        r = ROW.search(m.group(1))
        return [("checker", r.group(1) if r else "-")]
    cases = []
    for m in CASE.finditer(text):
        name = m.group(1)
        if name == "Internal":
            t = re.search(r"Internal\(\w+\)", m.group(2))
            name = t.group(0) if t else "Internal"
        elif name == "CheckerViolation":
            r = ROW.search(" " + m.group(2).lstrip(": ") + " ")
            name = f"CheckerViolation({r.group(1)})" if r else name
        cases.append(("differential", name))
    if cases:
        return sorted(set(cases))
    msg = message(text)
    first = msg.split("\n", 1)[0]
    t = TOKEN.search(first)
    if t:
        return [("variant", t.group(0))]
    return [("assert", stem(msg))]

ran = cap = shards = 0
secs = 0.0
stops = {}
tests = fails = 0
counts = {}   # slug -> count
new = {}      # sig -> [count, message, replay]
for tc in root.iter("testcase"):
    tests += 1
    name = tc.get("name", "")
    source = SHARD.sub("", f"{tc.get('classname', '')}::{name}")
    err = "\n".join((tc.findtext(t) or "") for t in ("system-err", "system-out"))
    for m in BUDGET.finditer(err):
        shards += 1
        ran += int(m.group(1)); cap += int(m.group(2)); secs += float(m.group(3))
        stops[m.group(4)] = stops.get(m.group(4), 0) + 1
    for el in list(tc.findall("failure")) + list(tc.findall("error")):
        fails += 1
        kind = "timeout" if "timeout" in (el.get("type") or "") else ""
        text = (el.text or "") + "\n" + (el.get("message") or "") + "\n" + err
        replay = REPLAY.search(text)
        for cls, key in classify(kind, (el.text or "") + "\n" + err):
            slug = known.get((source, cls, key))
            if slug:
                counts[slug] = counts.get(slug, 0) + 1
            else:
                msg = message(el.text or "").split("\n", 1)[0][:200]
                ent = new.setdefault((source, cls, key), [0, msg, replay.group(0) if replay else None])
                ent[0] += 1

out.append("## Nightly triage\n")
out.append(f"{tests} tests, {fails} failures.")
if shards:
    stop = ", ".join(f"{n} on {k}" for k, n in sorted(stops.items()))
    out.append(f"prop-budget: {shards} shards ran {ran} of {cap} cases in {secs:.0f} s ({stop}).")
out.append("")
if counts:
    out.append("Known (accounted for by an open finding, job stays green):")
    for slug in sorted(counts):
        out.append(f"- `{slug}`: {counts[slug]}")
    out.append("")
if new:
    out.append(f"NEW signatures ({len(new)}), job red:")
    for (source, cls, key), (n, msg, replay) in sorted(new.items()):
        out.append(f"- `{source}` {cls} `{key}` x{n}: {msg}")
        if replay:
            out.append(f"  replay: `{replay}`")
else:
    out.append("No new signature.")
finish(1 if new else 0)
PY
