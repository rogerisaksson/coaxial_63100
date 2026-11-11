#!/usr/bin/env bash
# The tools a job takes from outside, as a table pins them (.github/inputs.txt unless told): a tool kept
# already (the Actions cache restored it) is taken as it is; else its sources are tried in the table's
# order, each retried, and the first whose sha256 checks is unpacked under $INPUTS/<tool>. Each tool's
# program then goes on the job's PATH and into its variable (NINJA, RENODE). --every fetches and checks
# every source of every tool instead, the mirrors a run would fall back to among them, and keeps none.
#
#     bash .github/inputs.sh [table]
#     bash .github/inputs.sh --every [table]
set -euo pipefail
every=false
if [ "${1:-}" = --every ]; then every=true; shift; fi
table="${1:-$(dirname "$0")/inputs.txt}"
INPUTS="${INPUTS:-$HOME/.ci-inputs}"
GITHUB_PATH="${GITHUB_PATH:-/dev/stdout}"
GITHUB_ENV="${GITHUB_ENV:-/dev/stdout}"

rows() {  # the table's rows, its comments and blank lines left out
  grep -v -e '^#' -e '^[[:space:]]*$' "$table"
}

unpack() {  # archive folder
  mkdir -p "$2"
  case "$1" in
    *.zip) unzip -q -o "$1" -d "$2" ;;
    *.tar.gz) tar -xzf "$1" -C "$2" ;;
    *.deb) dpkg-deb -x "$1" "$2" ;;
    *) echo "::error::$1: no way to unpack it" >&2; return 1 ;;
  esac
}

get() {  # sha256 program source folder: the source fetched (retried), its bytes checked, unpacked into
         # folder - true where its program is there
  local archive="$4.download/${3##*/}" found
  mkdir -p "$4.download"
  curl -sSfL --retry 4 --retry-all-errors -o "$archive" "$3" \
    && echo "$1  $archive" | sha256sum -c --quiet - \
    && unpack "$archive" "$4" \
    && found=( "$4"/$2 ) && [ -f "${found[0]}" ] \
    && rm -rf "$4.download" && return 0
  rm -rf "$4.download" "$4"
  return 1
}

if $every; then
  failed=0
  while read -r tool sum program source; do
    scratch=$(mktemp -d)
    if get "$sum" "$program" "$source" "$scratch/tree"; then echo "ok    $tool  $source"
    else echo "::error::$tool: $source did not serve its program whole"; failed=1; fi
    rm -rf "$scratch"
  done < <(rows)
  exit "$failed"
fi

fetch() {  # tool: its sources in turn until one serves its program whole; the program's path kept
  local tool="$1" name sum program source
  while read -r name sum program source; do
    [ "$name" = "$tool" ] || continue
    if get "$sum" "$program" "$source" "$INPUTS/$tool"; then
      echo "$program" > "$INPUTS/$tool/.program"
      echo "$tool: $source"
      return 0
    fi
    echo "::warning::$tool: $source did not serve its program whole; the next source" >&2
  done < <(rows)
  echo "::error::$tool: none of its sources served its program whole" >&2
  return 1
}

mkdir -p "$INPUTS"
for tool in $(rows | awk '{print $1}' | sort -u); do
  [ -f "$INPUTS/$tool/.program" ] || fetch "$tool"
  found=( "$INPUTS/$tool"/$(cat "$INPUTS/$tool/.program") )
  program="${found[0]}"
  echo "$(dirname "$program")" >> "$GITHUB_PATH"
  echo "${tool^^}=$program" >> "$GITHUB_ENV"
done
