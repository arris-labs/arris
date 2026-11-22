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
    r"(?:-([0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*))?"
    r"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
)


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def reject_constant(value):
    raise ValueError(f"invalid JSON constant {value}")


def valid_version(value):
    match = version_pattern.fullmatch(value) if isinstance(value, str) else None
    if match is None:
        return False
    prerelease = match[1] or ""
    return not any(
        len(part) > 1 and part.startswith("0") and part.isdigit()
        for part in prerelease.split(".")
    )


def string_list(value):
    return isinstance(value, list) and all(isinstance(item, str) for item in value)


def valid_dependency(dep):
    return (
        isinstance(dep, dict)
        and isinstance(dep.get("name"), str)
        and isinstance(dep.get("req"), str)
        and string_list(dep.get("features", []))
        and type(dep.get("optional", False)) is bool
        and type(dep.get("default_features", True)) is bool
        and dep.get("kind") in (None, "normal", "dev", "build")
        and all(
            dep.get(field) is None or isinstance(dep[field], str)
            for field in ("target", "registry", "package")
        )
    )


def valid_record(record):
    if not isinstance(record, dict):
        return False
    schema = record.get("v", 1)
    return (
        record.get("name") == name
        and valid_version(record.get("vers"))
        and type(record.get("yanked")) is bool
        and isinstance(record.get("deps"), list)
        and all(valid_dependency(dep) for dep in record["deps"])
        and isinstance(record.get("cksum"), str)
        and re.fullmatch(r"[0-9a-f]{64}", record["cksum"]) is not None
        and type(schema) is int
        and schema in (1, 2)
        and ("features2" not in record or schema == 2)
        and all(
            isinstance(record.get(field, {}), dict)
            and all(string_list(items) for items in record.get(field, {}).values())
            for field in ("features", "features2")
        )
        and all(
            record.get(field) is None or isinstance(record[field], str)
            for field in ("links", "rust_version", "pubtime")
        )
    )


try:
    versions = {}
    unique_versions = set()
    with open(path, encoding="utf-8") as records:
        for line_number, line in enumerate(records, 1):
            try:
                record = json.loads(
                    line, object_pairs_hook=unique_object, parse_constant=reject_constant
                )
                if not valid_record(record):
                    raise ValueError("invalid version record")
                indexed_version = record["vers"]
                # The registry's uniqueness rule ignores build metadata.
                unique_version = indexed_version.partition("+")[0]
                if unique_version in unique_versions:
                    raise ValueError(f"duplicate version {indexed_version!r}")
                unique_versions.add(unique_version)
                versions[indexed_version] = record["yanked"]
            except (ValueError, RecursionError) as error:
                raise ValueError(f"line {line_number}: {error}") from error
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
