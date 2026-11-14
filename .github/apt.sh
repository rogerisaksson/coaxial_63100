#!/usr/bin/env bash
# A job's apt packages, kept: installed by dpkg from the packages kept under $APT_KEPT (the Actions
# cache restored them, keyed by the runner's image) - no mirror reached. Where none are kept yet they
# are fetched first, from the Ubuntu mirrors that serve the runner's release now - the runner's own
# and independent ones, all asked at once, each bounded, before apt is: a name that does not resolve
# costs one bounded probe, not apt's resolver timeout on every file and retry. apt is shown the Ubuntu
# archive alone, through the mirrors that answered, and fails on any fetch that fails. What the image
# has already is never fetched; the packages asked are written down, so a kept nothing is told from a
# fetch not made. --every updates from each independent mirror alone instead and installs nothing: a
# fallback no run exercises would rot unseen.
#
#     bash .github/apt.sh package...
#     bash .github/apt.sh --every
set -euo pipefail
KEPT="${APT_KEPT:-$HOME/.ci-apt}"
ARCH=$(dpkg --print-architecture)
#: Mirrors beside the runner's own, other hosts under other names: the runner's three, all under
#: ubuntu.com, have failed to resolve together. An arm64 runner's packages are Ubuntu's ports, which
#: the archive and kernel.org's mirror of it do not hold (binary-arm64 404 there): MIT's and
#: Berkeley's ports, near GitHub's runners, each serving the release and its security pocket.
if [ "$ARCH" = amd64 ]; then
  INDEPENDENT=(https://mirrors.edge.kernel.org/ubuntu/ https://mirrors.mit.edu/ubuntu/)
else
  INDEPENDENT=(https://mirrors.mit.edu/ubuntu-ports/ https://mirrors.ocf.berkeley.edu/ubuntu-ports/)
fi
LIST=/etc/apt/ci-mirrors.txt
SOURCES=/etc/apt/ci-ubuntu.sources
APT=(-o "Dir::Etc::SourceList=$SOURCES" -o Dir::Etc::SourceParts=/dev/null
     -o APT::Get::List-Cleanup=false -o Acquire::Retries=5)
. /etc/os-release
RELEASE="$VERSION_CODENAME"
keyring=( /usr/share/keyrings/ubuntu-archive-keyring.* )

serves() {  # mirror: true where it serves the release's index and this machine's packages now (each asked twice
            # at most, 10 s each): every index names every architecture, a mirror holds only some
  local index
  index=$(curl -fsS --retry 1 --retry-all-errors --max-time 10 "${1%/}/dists/$RELEASE/InRelease") \
    && [[ $index == *$'\n'"Codename: $RELEASE"$'\n'* ]] \
    && curl -fsI --retry 1 --retry-all-errors --max-time 10 \
      "${1%/}/dists/$RELEASE/main/binary-$ARCH/Packages.gz" > /dev/null
}

use() {  # mirror...: apt shown the Ubuntu archive alone, through these mirrors in this order, updated
  if [ ! -e "${keyring[0]}" ]; then
    echo "::error::no Ubuntu archive keyring under /usr/share/keyrings" >&2
    return 1
  fi
  local mirror
  for mirror in "$@"; do echo "${mirror%/}/"; done | sudo tee "$LIST" > /dev/null
  printf 'Types: deb\nURIs: mirror+file:%s\nSuites: %s\nComponents: %s\nSigned-By: %s\n' "$LIST" \
    "$RELEASE $RELEASE-updates $RELEASE-backports $RELEASE-security" \
    'main restricted universe multiverse' "${keyring[0]}" | sudo tee "$SOURCES" > /dev/null
  sudo apt-get "${APT[@]}" -o APT::Update::Error-Mode=any update
}

if [ "${1:-}" = --every ]; then
  failed=0
  for mirror in "${INDEPENDENT[@]}"; do
    if use "$mirror" > /dev/null; then echo "ok    $mirror  $RELEASE"
    else echo "::error::$mirror: apt could not update $RELEASE from it alone"; failed=1; fi
  done
  exit $failed
fi

if [ "$(cat "$KEPT/.asked" 2>/dev/null)" != "$*" ]; then
  rm -rf "$KEPT" && mkdir -p "$KEPT"
  own=()
  if [ -f /etc/apt/apt-mirrors.txt ]; then
    mapfile -t own < <(awk '/^[a-z]/ {print $1}' /etc/apt/apt-mirrors.txt)
  fi
  mirrors=( "${own[@]}" "${INDEPENDENT[@]}" )
  answered=$(mktemp -d)
  for i in "${!mirrors[@]}"; do (serves "${mirrors[$i]}" && touch "$answered/$i") & done
  wait
  live=()
  for i in "${!mirrors[@]}"; do
    if [ -e "$answered/$i" ]; then live+=( "${mirrors[$i]}" )
    else echo "apt: ${mirrors[$i]} does not serve $RELEASE now"; fi
  done
  rm -rf "${answered:?}"
  if [ ${#live[@]} -eq 0 ]; then
    echo "::error::no mirror serves $RELEASE now: ${mirrors[*]}"
    exit 1
  fi
  use "${live[@]}"
  mapfile -t fetched < <(sudo apt-get "${APT[@]}" install -y -qq --print-uris "$@" | awk '{print $2}')
  sudo apt-get "${APT[@]}" install -y -qq --download-only "$@"
  for deb in "${fetched[@]}"; do cp "/var/cache/apt/archives/$deb" "$KEPT"/; done
  echo "$*" > "$KEPT/.asked"
fi
shopt -s nullglob
kept=( "$KEPT"/*.deb )
if [ ${#kept[@]} -gt 0 ]; then
  sudo dpkg -i "${kept[@]}"
fi
echo "apt: $* (${#kept[@]} packages kept)"
