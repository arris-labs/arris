#!/usr/bin/env bash
# The fetched tier of the real-part corpus (ADR-0026 §2 and its amendment
# of step 9): what the nightly's `real-parts` job runs, and
# the same thing locally.
#
#   tools/real-parts.sh [--jobs N] [--timeout SECONDS]
#
# Fetches NIST's two archives into target/real-parts/archives/ (kept, and
# checked against their pinned SHA-256 before any use), unpacks them into
# target/real-parts/files/, checks every file tools/real-parts.sha256
# lists, and surveys each in its own process of the `real_parts` example
# (arris_debug::survey) under a timeout: Arris's read, Open CASCADE's
# through the oracle's cache, every solid held to it, and the battery.
# Writes target/real-parts/histogram.md, failures.md and
# fillet-by-part.md (the fillet column by part, from the survey). Exits 1 if a
# failure is not excluded by tools/real-parts.waits.
set -euo pipefail

jobs=4
timeout=1200
while [ $# -gt 0 ]; do
  case "$1" in
    --jobs) jobs="$2"; shift 2 ;;
    --timeout) timeout="$2"; shift 2 ;;
    *) echo "usage: tools/real-parts.sh [--jobs N] [--timeout SECONDS]" >&2; exit 2 ;;
  esac
done

cd "$(dirname "$0")/.."
root=target/real-parts
manifest=tools/real-parts.sha256

# name, URL, SHA-256 of the archive (ADR-0026 §1 and §2).
archives=(
  "pmi https://www.nist.gov/system/files/documents/noindex/2024/06/19/NIST-PMI-STEP-Files.zip 8fa78429e6d8d9b0d7681d223b6aa9ec98c3772185c55b1a0e3679b21c181911"
  "d2mi https://www.nist.gov/system/files/documents/el/msid/infotest/NIST-D2MI-Models.zip f20e36fb68633129dfedadf209ba4836bb0e42e18f12e7a2c28140ecdfb26cf3"
)

mkdir -p "$root/archives" "$root/files"
for entry in "${archives[@]}"; do
  read -r name url sha <<<"$entry"
  zip="$root/archives/$name.zip"
  if ! echo "$sha  $zip" | sha256sum --quiet -c - 2>/dev/null; then
    echo "fetching $url"
    curl -fsSL --retry 3 -o "$zip.part" "$url"
    mv "$zip.part" "$zip"
    echo "$sha  $zip" | sha256sum --quiet -c -
  fi
  rm -rf "${root:?}/files/$name"
  unzip -q -o "$zip" -d "$root/files/$name"
done
(cd "$root/files" && grep -v '^#' "../../../$manifest" | sha256sum --quiet -c -)

cargo build --release -p arris-debug --example real_parts
rm -rf "$root/reports" "$root/work"
mkdir -p "$root/reports" "$root/work"

# One process per file: a read that never ends is its timeout.
survey() {
  path="$1"
  stem=$(basename "$path" .stp)
  if ! timeout "$timeout" target/release/examples/real_parts --part \
    "$root/files/$path" "$root/work/$stem" "$root/reports/$stem.json" \
    "NIST, $path" >"$root/work/$stem.log" 2>&1; then
    echo "$stem: no report (timed out or failed; $root/work/$stem.log)"
    rm -f "$root/reports/$stem.json"
  else
    cat "$root/work/$stem.log"
  fi
}
export -f survey
export root timeout
grep -v '^#' "$manifest" | cut -c67- | tr '\n' '\0' |
  xargs -0 -P "$jobs" -I{} bash -c 'survey "$1"' _ {}

target/release/examples/real_parts --summary "$manifest" "$root/reports" \
  tools/real-parts.waits "$root"
