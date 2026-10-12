#!/usr/bin/env python3
"""excerpt.py <file.stp> <solid-id> <out.stp> [description] [--faces ID,ID,...]
— one solid of a STEP file, alone: the entity `#<solid-id>` and everything it refers
to, in a new ADVANCED_BREP_SHAPE_REPRESENTATION over the representation
context of the first representation that lists it, that context's
closure, and a new product the representation is the shape of. The file's
product structure, placements, presentation and PMI are left out, so the
solid stands where its representation puts it. With `--faces`, the
named `ADVANCED_FACE`s of the solid instead, as one `OPEN_SHELL` of a
`SHELL_BASED_SURFACE_MODEL` (a sheet, ADR-0057): a failure between faces
shrunk to them and their shared edges. What a
fetched part's failure is committed as, where its licence allows
(ADR-0026 §2). Run as
`uv run --project tools/oracle tools/oracle/excerpt.py`.
"""

import re
import sys
from pathlib import Path

from oracle.step import instances

_STRING = re.compile(r"'(?:[^']|'')*'")
_REF = re.compile(r"#(\d+)")


def refs(body: str) -> list[int]:
    """The `#id`s a body refers to, strings skipped."""
    return [int(r) for r in _REF.findall(_STRING.sub("''", body))]


def closure(bodies: dict[int, str], roots: list[int]) -> set[int]:
    seen: set[int] = set()
    stack = list(roots)
    while stack:
        at = stack.pop()
        if at in seen or at not in bodies:
            continue
        seen.add(at)
        stack.extend(refs(bodies[at]))
    return seen


def main(argv: list[str]) -> int:
    faces: list[int] = []
    if "--faces" in argv:
        at = argv.index("--faces")
        if at + 1 >= len(argv):
            print(__doc__)
            return 2
        faces = [int(f.lstrip("#")) for f in argv[at + 1].split(",")]
        argv = argv[:at] + argv[at + 2 :]
    if len(argv) < 3:
        print(__doc__)
        return 2
    source, solid, out = Path(argv[0]), int(argv[1]), Path(argv[2])
    description = argv[3] if len(argv) > 3 else f"#{solid} of {source.name}, alone"
    text = source.read_text(errors="replace")
    bodies = instances(text)
    if solid not in bodies:
        print(f"{source}: no #{solid}", file=sys.stderr)
        return 1
    context = None
    for id, body in sorted(bodies.items()):
        if "REPRESENTATION" in body.split("(", 1)[0] and solid in refs(body):
            m = re.search(r"\)\s*,\s*#(\d+)\s*\)\s*$", _STRING.sub("''", body).strip())
            if m:
                context = int(m.group(1))
                break
    if context is None:
        print(f"{source}: no representation lists #{solid}", file=sys.stderr)
        return 1
    missing = [f for f in faces if f not in bodies or not bodies[f].startswith("ADVANCED_FACE")]
    if missing:
        print(f"{source}: no ADVANCED_FACE #{missing[0]}", file=sys.stderr)
        return 1
    keep = closure(bodies, (faces or [solid]) + [context])
    rep = max(bodies) + 1
    schema = re.search(r"FILE_SCHEMA\s*\(.*?\)\s*;", text, re.S)
    lines = [
        "ISO-10303-21;",
        "HEADER;",
        f"FILE_DESCRIPTION(('{description.replace(chr(39), chr(39) * 2)}'),'2;1');",
        f"FILE_NAME('{out.name}','',(''),(''),'','','');",
        schema.group(0) if schema else "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));",
        "ENDSEC;",
        "DATA;",
    ]
    lines += [f"#{id}={bodies[id].strip()};" for id in sorted(keep)]
    if faces:
        listed = ",".join(f"#{f}" for f in faces)
        lines.append(f"#{rep}=OPEN_SHELL('',({listed}));")
        lines.append(f"#{rep + 1}=SHELL_BASED_SURFACE_MODEL('',(#{rep}));")
        lines.append(
            f"#{rep + 2}=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(#{rep + 1}),#{context});"
        )
        rep += 2
    else:
        lines.append(f"#{rep}=ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{solid}),#{context});")
    # A product for the representation to be the shape of: what Open
    # CASCADE's reader transfers from, as a part's own file has it.
    n = rep + 1
    lines += [
        f"#{n}=APPLICATION_CONTEXT('core data for automotive mechanical design processes');",
        f"#{n + 1}=APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{n});",
        f"#{n + 2}=PRODUCT_CONTEXT('',#{n},'mechanical');",
        f"#{n + 3}=PRODUCT('part','part','',(#{n + 2}));",
        f"#{n + 4}=PRODUCT_DEFINITION_FORMATION('','',#{n + 3});",
        f"#{n + 5}=PRODUCT_DEFINITION_CONTEXT('part definition',#{n},'design');",
        f"#{n + 6}=PRODUCT_DEFINITION('design','',#{n + 4},#{n + 5});",
        f"#{n + 7}=PRODUCT_DEFINITION_SHAPE('','',#{n + 6});",
        f"#{n + 8}=SHAPE_DEFINITION_REPRESENTATION(#{n + 7},#{rep});",
    ]
    lines += ["ENDSEC;", "END-ISO-10303-21;", ""]
    out.write_text("\n".join(lines))
    print(f"{out}: {len(keep) + 10 + 2 * bool(faces)} of {len(bodies)} instances, {out.stat().st_size} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
