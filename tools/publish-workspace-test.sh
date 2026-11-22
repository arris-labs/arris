#!/usr/bin/env bash
# Offline acceptance for publish-workspace.sh. All network, uploads and waits are stubbed.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
export TEST_DIR="$test_dir"
export PATH="$test_dir/bin:$PATH"
mkdir "$test_dir/bin"

cat > "$test_dir/metadata" <<'JSON'
{
  "workspace_members": ["a", "xy", "Bee", "arris", "private", "other"],
  "packages": [
    {"id":"arris","name":"arris","version":"1.2.3","publish":null},
    {"id":"a","name":"a","version":"1.2.3","publish":["crates-io"]},
    {"id":"xy","name":"xy","version":"1.2.3","publish":null},
    {"id":"Bee","name":"Bee","version":"1.2.3","publish":null},
    {"id":"private","name":"private","version":"1.2.3","publish":[]},
    {"id":"other","name":"other","version":"1.2.3","publish":["another-registry"]},
    {"id":"external","name":"external","version":"1.2.3","publish":null}
  ]
}
JSON

cat > "$test_dir/bin/cargo" <<'STUB'
#!/usr/bin/env bash
set -euo pipefail
if [[ $* == 'metadata --no-deps --format-version 1 --locked' ]]; then
  cat "$TEST_DIR/metadata"
  exit
fi
[[ $1 == publish ]]
printf '%s\n' "$*" >> "$TEST_DIR/uploads"
if [[ $CASE == final && $(wc -l < "$TEST_DIR/uploads") == 3 ]]; then
  touch "$TEST_DIR/final"
fi
case $CASE in
  delayed) [[ -e "$TEST_DIR/visible" ]] ;;
  persistent|final) exit 101 ;;
esac
STUB

cat > "$test_dir/bin/curl" <<'STUB'
#!/usr/bin/env bash
set -euo pipefail
# Check the explicit time bound and that curl config loading is disabled.
[[ $1 == -q && $* == *'--max-time 30'* && $* != *Authorization* ]]
out=
for arg in "$@"; do
  if [[ ${previous:-} == --output ]]; then out=$arg; fi
  previous=$arg
done
case ${!#} in
  https://index.crates.io/1/a) name=a ;;
  https://index.crates.io/2/xy) name=xy ;;
  https://index.crates.io/3/b/bee) name=Bee ;;
  https://index.crates.io/ar/ri/arris) name=arris ;;
  *) printf 'Unexpected index request: %s\n' "${!#}" >&2; exit 1 ;;
esac
echo "$name" >> "$TEST_DIR/lookups"

record() {
  printf '{"name":"%s","vers":"%s","deps":[],"cksum":"%064d","yanked":%s}\n' \
    "$name" "$1" 0 "${2:-false}" > "$out"
}
case $CASE:$name in
  unreadable:arris) exit 28 ;;
  unavailable:arris) printf 503; exit ;;
  malformed:arris) printf '{\n' > "$out" ;;
  invalid:arris) printf '{"name":"arris","vers":"1.2.3"}\n' > "$out" ;;
  yanked:arris) record 1.2.3 true ;;
  complete:*|final:*)
    if [[ $CASE == complete || -e "$TEST_DIR/final" ]]; then
      record 1.2.3
    else
      record 1.2.2
    fi ;;
  resumed:a|resumed:Bee) record 1.2.3 ;;
  delayed:a)
    if [[ -e "$TEST_DIR/visible" ]]; then record 1.2.3; else record 1.2.2; fi ;;
  *:xy) printf 404; exit ;;
  *) record 1.2.2 ;;
esac
printf 200
STUB

cat > "$test_dir/bin/sleep" <<'STUB'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >> "$TEST_DIR/waits"
touch "$TEST_DIR/visible"
STUB
chmod +x "$test_dir"/bin/*

run_case() {
  local expected=$1
  export CASE=$2
  shift 2
  rm -f "$test_dir/visible" "$test_dir/final"
  : > "$test_dir/uploads"
  : > "$test_dir/waits"
  : > "$test_dir/lookups"
  local status=0
  tools/publish-workspace.sh "$@" > "$test_dir/output" 2>&1 || status=$?
  if [[ $status != "$expected" ]]; then
    cat "$test_dir/output" >&2
    echo "$CASE: expected status $expected, got $status" >&2
    exit 1
  fi
}

assert_file() {
  diff -u --label "$CASE expected $1" --label "actual $1" \
    <(if (( $# > 1 )); then printf '%s\n' "${@:2}"; fi) "$test_dir/$1"
}

base='publish --workspace --locked --exclude other'

run_case 0 fresh
assert_file uploads "$base"
assert_file waits
assert_file lookups Bee a arris xy

run_case 0 delayed
assert_file uploads "$base" 'publish --workspace --locked --exclude a --exclude other'
assert_file waits 120

run_case 0 resumed
assert_file uploads 'publish --workspace --locked --exclude Bee --exclude a --exclude other'
assert_file waits

run_case 0 complete
assert_file uploads
assert_file waits
grep -Fq 'All crates.io workspace versions are indexed.' "$test_dir/output"

run_case 0 resumed --dry-run
assert_file uploads
assert_file waits
grep -Fxq 'Would run: cargo publish --workspace --locked --exclude Bee --exclude a --exclude other' "$test_dir/output"

for scenario in unreadable unavailable malformed invalid yanked; do
  run_case 1 "$scenario"
  assert_file uploads
  assert_file waits
  grep -Fq 'index for arris 1.2.3' "$test_dir/output"
done

run_case 1 persistent
assert_file uploads "$base" "$base" "$base"
assert_file waits 120 120
assert_file lookups Bee a arris xy Bee a arris xy Bee a arris xy Bee a arris xy
grep -Fq 'Release remains incomplete after 3 Cargo publish attempts.' "$test_dir/output"

run_case 0 final
assert_file uploads "$base" "$base" "$base"
assert_file waits 120 120
grep -Fq 'All crates.io workspace versions are indexed.' "$test_dir/output"

echo 'publish-workspace-test: passed'
