#!/bin/bash
# Build the four Mcity vegetation configurations.
#
# The source assets are Omniverse scan-grade: 80k-620k triangles per plant, and Mcity places 2009
# of them for 264M triangles a frame. A rasterizer here sustains roughly 220M triangles a second,
# so the full set is unusable and every configuration below is a different answer to what to give
# up. Each writes its meshes to its own directory, because sharing one silently rewrites the
# meshes another manifest still points at.
#
# Run after usd_to_chrono.py --exclude-groups "" has produced mcity_scene_foliage.json.
#
# Usage:  ./build_configs.sh
set -e
source /opt/homebrew/Caskroom/miniconda/base/etc/profile.d/conda.sh
conda activate chronopc
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
cd "$ROOT"
DATA=data/mcity
DEC="python3 $DIR/decimate_foliage.py"

if [ ! -f "$DATA/mcity_scene_foliage.json" ]; then
  echo "missing $DATA/mcity_scene_foliage.json -- run:" >&2
  echo "  python3 $DIR/usd_to_chrono.py --exclude-groups \"\"" >&2
  exit 1
fi

echo "== 1/4  trees only, bare branches =="
$DEC --leaf-fraction 0 --drop-shrubs \
     --lod-dir lod_trees_bare --out $DATA/mcity_scene_trees_bare.json | tail -2

echo "== 2/4  trees and shrubs, bare branches (shrub twigs thinned) =="
$DEC --leaf-fraction 0 --branch-fraction 0.25 \
     --lod-dir lod_all_bare --out $DATA/mcity_scene_all_bare.json | tail -2

echo "== 3/4  trees only, with leaves =="
$DEC --leaf-fraction 0.4 --drop-shrubs \
     --lod-dir lod_trees_leaf --out $DATA/mcity_scene_trees_leaf.json | tail -2

echo "== 4/4  everything: trees and shrubs, with leaves =="
$DEC --leaf-fraction 0.4 --branch-fraction 0.25 \
     --lod-dir lod_full --out $DATA/mcity_scene_full.json | tail -2

echo
echo "configurations (all take STREAM_RADIUS=90 TILE_SIZE=40):"
for m in mcity_scene mcity_scene_trees_bare mcity_scene_all_bare mcity_scene_trees_leaf mcity_scene_full; do
  [ -f "$DATA/$m.json" ] || continue
  printf "  %-28s %s\n" "$m.json" \
    "$("$DIR/probe_scenery.sh" "$DATA/$m.json" 0 6 2>/dev/null | grep -oE 'triangles, as drawn: +[0-9]+' | tr -s ' ')"
done
