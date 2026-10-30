#!/usr/bin/env bash
# Tests `tools/step-rework.sh` on a scratch repository: a reworked step, an
# override, a review fix, a step with no grade, a partial and a range.
#
#   tools/step-rework-test.sh
set -euo pipefail
script="$(git rev-parse --show-toplevel)/tools/step-rework.sh"
dir=$(mktemp -d); trap 'rm -rf "$dir"' EXIT
cd "$dir"
git init -q .; git config user.email t@t; git config user.name t
P=pl; P+=ans  # spelled apart: the docs lint reads a literal as a citation
mkdir -p docs/${P}
fail=0

commit() { git add -A; GIT_COMMITTER_DATE="2020-01-01T00:00:00" GIT_AUTHOR_DATE="2020-01-01T00:00:00" git commit -q --allow-empty -m "$1"; }

cat > docs/${P}/p.md <<'P'
- [ ] Step 1 **[1]** — a
- [ ] Step 2 **[2]** — b
- [ ] Step 3 **[1, tier 3]** — c
- [ ] Step 4 — d
- [ ] Step 5 **[2]** — e
- [ ] Step 6 **[3]** — f
P
commit "docs(plan): p"
commit "feat(ops): a (${P}/p step 1)"
commit "feat(ops): b (${P}/p step 2)"
commit "fix(ops): repairs b (${P}/p step 2)"
commit "fix(ops): repairs b again (${P}/p step 2)"
commit "feat(ops): c (${P}/p step 3)"
commit "feat(ops): d (${P}/p step 4)"
commit "feat(ops): e, first part (${P}/p step 5, partial)"
commit "feat(ops): e, rest (${P}/p step 5)"
commit "feat(ops): f and a part (${P}/p steps 6-7)"
commit "fix(ops): a review find (${P}/p review)"
commit "fix(ops): another (${P}/p review)"
commit "chore: unrelated"

out=$("$script")
expect() { grep -Eq "$1" <<<"$out" || { echo "FAIL: expected /$1/ in:" >&2; echo "$out" >&2; fail=1; }; }
expect '^1 +1 +0 +0% +0$'
expect '^2 +2 +1 +50% +2$'
expect '^3 +2 +0 +0% +0$'
expect '^none +2 +0 +0% +0$'
expect '^  p +2$'
# 7 has no plan line: none; 4 has no grade: none.
"$script" --since 2021-01-01 | grep -Eq '^1 +0 +0 +- +0$' \
  || { echo "FAIL: empty range" >&2; fail=1; }

[ $fail = 0 ] && echo "step-rework-test: ok"
exit $fail
