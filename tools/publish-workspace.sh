#!/usr/bin/env bash
# Resume a crates.io workspace release using the public sparse index.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

if (( $# > 1 )) || [[ ${1:-} != "" && ${1:-} != --dry-run ]]; then
  echo "usage: $0 [--dry-run]" >&2
  exit 2
fi

readonly max_attempts=3 retry_delay=120
metadata=$(cargo metadata --no-deps --format-version 1 --locked)
members=$(jq -er '
  .workspace_members as $members
  | [.packages[] | select(.id as $id | $members | index($id))]
  | sort_by(.name)[]
  | select(.publish != [])
  | [.name, .version, (if .publish == null or (.publish | index("crates-io"))
                       then "crates-io" else "other" end)] | @tsv
' <<< "$metadata")

work_dir=$(mktemp -d)
trap 'rm -rf "$work_dir"' EXIT

index_status() {
  local name=$1 version=$2 lower=${1,,} path http
  case ${#lower} in
    1) path="1/$lower" ;;
    2) path="2/$lower" ;;
    3) path="3/${lower:0:1}/$lower" ;;
    *) path="${lower:0:2}/${lower:2:2}/$lower" ;;
  esac

  if ! http=$(curl -q --silent --show-error --connect-timeout 10 --max-time 30 \
      --header 'Cache-Control: no-cache' --output "$work_dir/index" \
      --write-out '%{http_code}' "https://index.crates.io/$path"); then
    echo "Cannot fetch sparse index for $name $version" >&2
    return 1
  fi
  case $http in
    404) echo missing ;;
    200)
      python3 - "$work_dir/index" "$name" "$version" <<'PY'
import json
import re
import sys

path, name, version = sys.argv[1:]
version_pattern = re.compile(
    r"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)"
    r"(?:-[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
    r"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
)
try:
    versions = {}
    with open(path, encoding="utf-8") as records:
        for line in records:
            record = json.loads(line)
            if (
                not isinstance(record, dict)
                or record.get("name") != name
                or not isinstance(record.get("vers"), str)
                or not version_pattern.fullmatch(record["vers"])
                or type(record.get("yanked")) is not bool
                or not isinstance(record.get("deps"), list)
                or not isinstance(record.get("cksum"), str)
                or not re.fullmatch(r"[0-9a-f]{64}", record["cksum"])
                or record["vers"] in versions
            ):
                raise ValueError("invalid or duplicate version record")
            versions[record["vers"]] = record["yanked"]
    if not versions:
        raise ValueError("empty index file")
    if versions.get(version):
        raise ValueError("target version is yanked")
except (OSError, ValueError) as error:
    sys.exit(f"Sparse index for {name} {version}: {error}")

print("indexed" if version in versions else "missing")
PY
      ;;
    *) echo "Sparse index for $name $version returned HTTP $http" >&2; return 1 ;;
  esac
}

for (( attempt=1; attempt<=max_attempts+1; attempt++ )); do
  publish=(cargo publish --workspace --locked)
  missing=()
  while IFS=$'\t' read -r name version registry; do
    if [[ $registry == other ]]; then
      publish+=(--exclude "$name")
      continue
    fi
    status=$(index_status "$name" "$version")
    if [[ $status == indexed ]]; then
      publish+=(--exclude "$name")
    else
      missing+=("$name $version")
    fi
  done <<< "$members"

  if (( ${#missing[@]} == 0 )); then
    echo "All crates.io workspace versions are indexed."
    exit 0
  fi
  printf 'Missing from crates.io: %s\n' "${missing[@]}"
  if [[ ${1:-} == --dry-run ]]; then
    printf '%s' "Would run:"
    printf ' %q' "${publish[@]}"
    printf '\n'
    exit 0
  fi
  # The last pass only reconciles a final Cargo failure with the index.
  if (( attempt > max_attempts )); then
    echo "Release remains incomplete after $max_attempts Cargo publish attempts." >&2
    exit 1
  fi
  echo "Cargo publish attempt $attempt of $max_attempts"
  if "${publish[@]}"; then
    exit 0
  fi
  if (( attempt < max_attempts )); then
    echo "Cargo publish failed. Checking the index again in $retry_delay seconds."
    sleep "$retry_delay"
  fi
done
