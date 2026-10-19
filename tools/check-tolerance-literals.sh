#!/usr/bin/env bash
# A tolerance literal added to kernel source fails the commit
# (`.agents/rules/kernel.md`: "Tolerances are the model's, never a literal").
#
# Scoped to the staged diff: the tree holds literals from before this check,
# and only new ones are refused. An added line under `crates/*/src/**.rs`
# holding a literal like `1e-6` or `2.5e-3` passes when it
#   - declares a `const` or `static` (the named, commented constant),
#   - is a comment,
#   - sits after the file's first `#[cfg(test)]` (the test module, which
#     this repo keeps at the end of a file),
#   - or carries `// tol-literal: <reason>` for a deliberate one.
# `arris-debug` and `arris-py` are tooling and a binding, not algorithms.
#
# `tools/check-tolerance-literals.sh` checks what is staged; with `--diff
# <rev>` it checks the working tree against <rev> instead.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

if [ "${1:-}" = --diff ]; then
  diff_args=("${2:?--diff needs a revision}")
  show() { cat "$1"; }
else
  diff_args=(--cached)
  show() { git show ":$1"; }
fi

bad=0
while IFS= read -r f; do
  case "$f" in
    crates/arris-debug/*|crates/arris-py/*) continue ;;
    crates/*/src/*.rs) ;;
    *) continue ;;
  esac
  # The line where the file's test module starts; 0 when it has none.
  test_from=$(show "$f" | grep -n -m1 '^[[:space:]]*#\[cfg(test)\]' | cut -d: -f1 || true)
  test_from=${test_from:-0}
  hits=$(git diff "${diff_args[@]}" -U0 -- "$f" | awk -v from="$test_from" '
    /^@@/ { split($3, a, ","); n = substr(a[1], 2) + 0; next }
    /^\+\+\+/ { next }
    /^\+/ {
      line = substr($0, 2)
      if (from > 0 && n >= from) { n++; next }
      if (line ~ /^[[:space:]]*\/\//) { n++; next }
      if (line ~ /(^|[[:space:]])(pub(\([a-z]+\))?[[:space:]]+)?(const|static)[[:space:]]/) { n++; next }
      if (line ~ /tol-literal:/) { n++; next }
      code = line; sub(/\/\/.*/, "", code)
      if (code ~ /[0-9]e-[0-9]/) printf "%d: %s\n", n, line
      n++
    }')
  if [ -n "$hits" ]; then
    bad=1
    while IFS= read -r h; do echo "  $f:$h" >&2; done <<<"$hits"
  fi
done < <(git diff "${diff_args[@]}" --name-only --diff-filter=AM)

if [ $bad = 1 ]; then
  cat >&2 <<'EOF'
check-tolerance-literals: a tolerance literal in kernel source
(.agents/rules/kernel.md). Use the entity's tolerance, or a named,
commented `const` (in arris-math when more than one crate needs it). A
deliberate literal says why on its line: `// tol-literal: <reason>`.
EOF
  exit 1
fi
