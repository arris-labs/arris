#!/usr/bin/env bash
# Exercise the PR policy on scratch commits. Stub the build commands so
# each case tests the real Git checks without compiling a workspace.
set -euo pipefail
src=$(git rev-parse --show-toplevel)
scratch=$(mktemp -d)
trap 'rm -rf "$scratch"' EXIT

# A hook can pass its own index or work tree through the environment.
mapfile -t git_vars < <(git rev-parse --local-env-vars)
unset "${git_vars[@]}"
git init -q -b main "$scratch/repo"
cd "$scratch/repo"
git config user.name Contributor
git config user.email contributor@users.noreply.github.com
git config core.hooksPath .githooks
git config commit.gpgSign false
mkdir -p tools "$scratch/bin" crates/arris-io/tests/data
for command in check-layers.sh check-tolerance-literals.sh test-half.sh; do
  cat >"tools/$command" <<'STUB'
#!/usr/bin/env bash
printf '%s %s\n' "${0##*/}" "$*"
STUB
  chmod +x "tools/$command"
done
cp tools/check-layers.sh "$scratch/bin/cargo"
export PATH="$scratch/bin:$PATH"

commit() { git commit -q --allow-empty -m "$1" "${@:2}"; }
printf 'legacy fixture\n' >crates/arris-io/tests/data/legacy.step
git add tools crates
commit 'test(io): set up the base'
base=$(git rev-parse HEAD)
check() { "$src/tools/pr-check.sh" --base "$base" "$@" 2>&1; }
has() {
  if [[ "$1" != *"$2"* ]]; then
    printf 'pr-check-test: expected %q in:\n%s\n' "$2" "$1" >&2
    exit 1
  fi
}
pass() {
  local output
  output=$(check "$@") || { printf '%s\n' "$output" >&2; exit 1; }
  has "$output" 'pr-check: ok against'
  printf '%s\n' "$output"
}
fail() {
  local expected=$1 output status=0
  shift
  output=$(check "$@") || status=$?
  if [ "$status" != 1 ]; then
    printf 'pr-check-test: expected exit 1, got %s:\n%s\n' "$status" "$output" >&2
    exit 1
  fi
  has "$output" "$expected"
}
at_base() { git switch -q --detach "$base"; }

# A normal branch can change old data, adds new data in the corpus and
# allows contributors' own signoffs and the names of technical terms.
git switch -q -c contribution
mkdir -p 'tests/fixtures/step with spaces'
touch 'tests/fixtures/step with spaces/shape.step' 'tests/fixtures/part.STP'
printf 'corrected fixture\n' >crates/arris-io/tests/data/legacy.step
git add tests crates
commit 'test(geom,io): cover a STEP read' -m 'Signed-off-by: Contributor <contributor@users.noreply.github.com>'
out=$(pass)
has "$out" 'check-layers.sh '
has "$out" "check-tolerance-literals.sh --diff $base"
has "$out" 'cargo fmt --all -- --check'
has "$out" 'cargo clippy --workspace --all-targets -- -D warnings'
has "$out" "test-half.sh $base HEAD"

# The default fetch reads canonical main. Redirect that URL to this
# scratch repo to exercise the default without contacting GitHub.
git config "url.file://$scratch/repo.insteadOf" https://github.com/arris-labs/arris.git
out=$("$src/tools/pr-check.sh" 2>&1) || { printf '%s\n' "$out" >&2; exit 1; }
has "$out" "test-half.sh $base HEAD"

git config core.hooksPath .git/hooks
fail 'core.hooksPath must be .githooks'
out=$(pass --ci)
if [[ "$out" == *clippy* ]]; then
  printf 'pr-check-test: CI must leave clippy to its own job\n' >&2
  exit 1
fi
git config core.hooksPath .githooks

# Upstream moved while the contributor kept working. CI can check an explicit
# base without enforcing the local rebase requirement.
good=$(git rev-parse HEAD)
git switch -q main
commit 'docs(docs): advance the base'
upstream=$(git rev-parse HEAD)
git switch -q contribution
out=$("$src/tools/pr-check.sh" 2>&1) && status=0 || status=$?
[ "$status" = 1 ] || { printf '%s\n' "$out" >&2; exit 1; }
has "$out" "branch does not contain base ${upstream:0:12}"
has "$out" "test-half.sh $base HEAD"
base=$upstream pass --ci >/dev/null

for subject in 'fix: add scope' 'fix(io): Capital summary' 'fix(io): remove period.'; do
  at_base
  commit "$subject"
  fail 'subject must be type(scope)'
done

for email in contributor@localhost contributor@intranet 'first.last@localhost'; do
  at_base
  GIT_AUTHOR_EMAIL=$email commit 'test(io): check the author'
  fail "author email needs a dotted domain: $email"
done

for trailer in 'Co-Authored-By: Other <other@example.com>' \
  'Claude-Session: abc' '🤖 Generated with Claude Code'; do
  at_base
  commit 'test(io): check the trailers' -m "$trailer"
  fail "disallowed trailer: $trailer"
done

at_base
mkdir -p docs/adr
touch CHANGELOG.md docs/adr/0001-example.md crates/arris-io/tests/data/new.step tools/part.STP
git add CHANGELOG.md docs crates tools/part.STP
commit 'test(io): check file placement'
fail 'CHANGELOG.md is edited by the maintainer at landing'
fail 'docs/adr/0001-example.md is edited by the maintainer at landing'
fail 'place added STEP data under tests/fixtures/: crates/arris-io/tests/data/new.step'
fail 'place added STEP data under tests/fixtures/: tools/part.STP'

at_base
commit 'test(io): add a parallel commit'
git merge -q --no-ff -m 'test(io): combine the commits' "$good"
fail 'is a merge commit'

printf 'uncommitted fixture edit\n' >>crates/arris-io/tests/data/legacy.step
fail 'commit tracked edits before checking the PR head'
echo 'pr-check-test: ok'
