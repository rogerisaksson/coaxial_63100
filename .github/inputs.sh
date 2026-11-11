#!/usr/bin/env bash
# The firmware job's tools from outside, as a table pins them (.github/inputs.txt unless told): a
# tool kept already (the Actions cache restored it) is taken as it is; else its sources are tried in
# the table's order, each retried, and the first whose sha256 checks is unpacked under $INPUTS/<tool>.
# Each tool's program then goes on the job's PATH and into its variable (NINJA, RENODE).
#
#     bash .github/inputs.sh [table]
set -euo pipefail
table="${1:-$(dirname "$0")/inputs.txt}"
INPUTS="${INPUTS:-$HOME/.ci-inputs}"
GITHUB_PATH="${GITHUB_PATH:-/dev/stdout}"
GITHUB_ENV="${GITHUB_ENV:-/dev/stdout}"
mkdir -p "$INPUTS"

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

fetch() {  # tool: its sources in turn until one's bytes check; unpacked, its program's path kept
  local tool="$1" name sum program source archive
  while read -r name sum program source; do
    [ "$name" = "$tool" ] || continue
    archive="$INPUTS/${source##*/}"
    if curl -sSfL --retry 4 --retry-all-errors -o "$archive" "$source" \
        && echo "$sum  $archive" | sha256sum -c --quiet - && unpack "$archive" "$INPUTS/$tool"; then
      rm -f "$archive"
      echo "$program" > "$INPUTS/$tool/.program"
      echo "$tool: $source"
      return 0
    fi
    echo "::warning::$tool: $source did not serve its bytes; the next source" >&2
    rm -rf "$archive" "$INPUTS/$tool"
  done < <(rows)
  echo "::error::$tool: none of its sources served its bytes" >&2
  return 1
}

for tool in $(rows | awk '{print $1}' | sort -u); do
  [ -f "$INPUTS/$tool/.program" ] || fetch "$tool"
  found=( "$INPUTS/$tool"/$(cat "$INPUTS/$tool/.program") )
  program="${found[0]}"
  [ -f "$program" ] || { echo "::error::$tool: no program at $program" >&2; exit 1; }
  echo "$(dirname "$program")" >> "$GITHUB_PATH"
  echo "${tool^^}=$program" >> "$GITHUB_ENV"
done
