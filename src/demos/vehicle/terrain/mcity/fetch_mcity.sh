#!/bin/bash
# Fetch the Mcity digital twin assets needed to drive Mcity in Chrono.
#
# Two sources, both from github.com/mcity/mcity-digital-twin (MIT licence):
#   McityMap_Main.xodr   the ASAM OpenDRIVE road network -- lanes, junctions, markings, elevation
#   Omniverse/*.usd(c)   the 3D environment, as a root stage referencing per-asset meshes
#
# Only what is useful for driving is pulled. The SubUSDs folder is 154 MB of vegetation and the
# Foliage_Instanced stage another 21 MB; both are skipped by default since they dominate the
# download while contributing least to a drivable scene. Pass --foliage to include them.
#
# Nothing fetched here is committed -- see .gitignore. Re-run to restore.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
OUT="${MCITY_DIR:-$ROOT/data/mcity}"
REPO="mcity/mcity-digital-twin"
RAW="https://raw.githubusercontent.com/$REPO/main"
USD_ROOT="Omniverse/Collected_McityMap_NSR_v4_1_6"
WANT_FOLIAGE=0
[ "${1:-}" = "--foliage" ] && WANT_FOLIAGE=1

mkdir -p "$OUT/usd"
echo "fetching into $OUT"

# The road network.
curl -sfL -o "$OUT/McityMap_Main.xodr" "$RAW/CARLA/source_version/McityMap/OpenDrive/McityMap_Main.xodr"
echo "  road network: $(du -h "$OUT/McityMap_Main.xodr" | cut -f1)"

# The USD root stage plus the per-asset meshes it references.
curl -sfL -o "$OUT/usd/McityMap_Main.usdc" "$RAW/$USD_ROOT/McityMap_Main.usdc"

# The tree listing, cached on disk. GitHub rate-limits this endpoint per IP, and an unauthenticated
# run that trips the limit gets an HTML error page rather than JSON. Without the cache that
# produced an empty download list and a run that looked like it had succeeded.
TREE="$OUT/repo_tree.json"
if [ ! -s "$TREE" ]; then
  echo "  listing repository tree"
  curl -sfL -o "$TREE.tmp" "https://api.github.com/repos/$REPO/git/trees/main?recursive=1" || true
  if python3 -c "import json,sys; json.load(open('$TREE.tmp'))['tree']" 2>/dev/null; then
    mv "$TREE.tmp" "$TREE"
  else
    rm -f "$TREE.tmp"
    echo "  could not list the repository (GitHub rate limit?); retry in a few minutes" >&2
    exit 1
  fi
else
  echo "  using cached repository tree"
fi

PATHS=$(python3 -c "
import json
keep=('$USD_ROOT/Props/','$USD_ROOT/FinalTrafficLights/')
if $WANT_FOLIAGE: keep += ('$USD_ROOT/SubUSDs/','$USD_ROOT/Foliage_Instanced.usdc')
for t in json.load(open('$TREE'))['tree']:
    p=t['path']
    if p.endswith(('.usd','.usdc','.usda')) and p.startswith(keep): print(p)")

n=$(echo "$PATHS" | grep -c . || true)
echo "  downloading $n asset meshes"
echo "$PATHS" | while read -r p; do
  [ -z "$p" ] && continue
  rel="${p#$USD_ROOT/}"
  mkdir -p "$OUT/usd/$(dirname "$rel")"
  echo "$RAW/$p" "$OUT/usd/$rel"
done | xargs -n2 -P8 sh -c 'curl -sfL -o "$1" "$0"' || true

echo "  done: $(find "$OUT/usd" -name '*.usd*' | wc -l | tr -d ' ') USD files, $(du -sh "$OUT/usd" | cut -f1)"
