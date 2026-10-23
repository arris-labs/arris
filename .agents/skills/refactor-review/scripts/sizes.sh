#!/bin/sh
# The workspace's scale, and how much changed since a base commit.
# Usage: sizes.sh [<base-ref>]   (from the repository root)
set -eu

echo "== lines by crate (src includes inline unit tests)"
for c in crates/*/; do
    s=$(find "$c" -path '*/src/*' -name '*.rs' -exec cat {} + 2>/dev/null | wc -l)
    t=$(find "$c" -path '*/tests/*' -name '*.rs' -exec cat {} + 2>/dev/null | wc -l)
    printf '%-24s src %6s  tests %6s\n' "$(basename "$c")" "$s" "$t"
done

echo "== src files over 900 lines"
find crates -path '*/src/*' -name '*.rs' -exec wc -l {} + |
    grep -v ' total$' | sort -rn | awk '$1 > 900'

echo "== test files over 1000 lines"
find crates -path '*/tests/*' -name '*.rs' -exec wc -l {} + |
    grep -v ' total$' | sort -rn | awk '$1 > 1000'

if [ $# -ge 1 ]; then
    echo "== since $1"
    echo "commits: $(git rev-list --count "$1"..HEAD)"
    echo "refactor commits: $(git log --oneline "$1"..HEAD | grep -c 'refactor(' || true)"
    git diff --shortstat "$1" HEAD -- crates
fi
