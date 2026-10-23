"""The longest library functions, and how many run past 100 and 150 lines.

Usage: python3 fn_lengths.py [top=60]   (from the repository root)
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from _rust import functions, library_files  # noqa: E402

top = int(sys.argv[1]) if len(sys.argv) > 1 else 60
rows = []
for path, src in library_files():
    for name, first, last, _ in functions(src):
        rows.append((last - first + 1, f"{path}:{first}", name))
rows.sort(reverse=True)
for length, at, name in rows[:top]:
    print(f"{length:5} {at} {name}")
print(
    f"functions: {len(rows)}, past 100 lines: {sum(r[0] > 100 for r in rows)},"
    f" past 150: {sum(r[0] > 150 for r in rows)}"
)
