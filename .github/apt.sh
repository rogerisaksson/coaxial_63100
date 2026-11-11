#!/usr/bin/env bash
# A job's apt packages, kept: installed by dpkg from the packages kept under $APT_KEPT (the Actions
# cache restored them, keyed by the runner's image) - no mirror reached; where none are kept yet they
# are fetched first, an independent mirror added to the runner's list and every fetch retried. What
# the image has already is never fetched; the packages asked are written down, so a kept nothing is
# told from a fetch not made.
#
#     bash .github/apt.sh package...
set -euo pipefail
KEPT="${APT_KEPT:-$HOME/.ci-apt}"
#: A mirror beside the runner's own: its names are not the ones that failed to resolve together.
MIRROR=https://mirrors.edge.kernel.org/ubuntu/
MIRRORS=/etc/apt/apt-mirrors.txt

if [ "$(cat "$KEPT/.asked" 2>/dev/null)" != "$*" ]; then
  rm -rf "$KEPT" && mkdir -p "$KEPT"
  if [ -f "$MIRRORS" ] && ! grep -qxF "$MIRROR" "$MIRRORS"; then
    echo "$MIRROR" | sudo tee -a "$MIRRORS" > /dev/null
  fi
  sudo apt-get -o Acquire::Retries=5 update
  mapfile -t fetched < <(sudo apt-get install -y -qq --print-uris "$@" | awk '{print $2}')
  sudo apt-get -o Acquire::Retries=5 install -y --download-only "$@"
  for deb in "${fetched[@]}"; do cp "/var/cache/apt/archives/$deb" "$KEPT"/; done
  echo "$*" > "$KEPT/.asked"
fi
shopt -s nullglob
kept=( "$KEPT"/*.deb )
if [ ${#kept[@]} -gt 0 ]; then
  sudo dpkg -i "${kept[@]}"
fi
echo "apt: $* (${#kept[@]} packages kept)"
