#!/bin/bash
# Build the four Mcity vegetation levels.
#
# The source plants are film-grade models, and ten of the species place every branch through
# point instancers. Expanded, the 2009 plants are about 1.85 billion triangles. Each level below
# is a different answer to what to give up, and decimate_foliage.py does the giving up: a
# triangle budget per plant, spent on welded wood and on enlarged leaves.
#
# Each level writes its meshes to its own directory, because sharing one silently rewrites the
# meshes another manifest still points at.
#
# Run after usd_to_chrono.py --exclude-groups "" has produced mcity_scene_foliage.json.
#
# Usage:  ./build_configs.sh [DATA_DIR]      default <chrono>/data/mcity
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
# Locate the Chrono root by walking up to the marker directory, rather than counting "..".
# These scripts have moved once already and the relative depth silently went wrong.
ROOT="$DIR"
while [ "$ROOT" != "/" ] && [ ! -d "$ROOT/src/chrono" ]; do ROOT="$(dirname "$ROOT")"; done
if [ ! -d "$ROOT/src/chrono" ]; then
  echo "could not locate the Chrono source root above $DIR" >&2
  exit 1
fi
DATA="${1:-$ROOT/data/mcity}"
DEC="python3 $DIR/decimate_foliage.py --dir $DATA"

if [ ! -f "$DATA/mcity_scene_foliage.json" ]; then
  echo "missing $DATA/mcity_scene_foliage.json -- run:" >&2
  echo "  python3 $DIR/usd_to_chrono.py --in $DATA --out $DATA --exclude-groups \"\"" >&2
  exit 1
fi

# The foliage manifest names meshes under assets/. A previous non-foliage conversion can leave
# that directory without them, and the decimator would then quietly skip every tree -- producing
# manifests that reference meshes which do not exist.
MISSING=$(python3 - "$DATA" <<'PY2'
import json, os, sys
data = sys.argv[1]
doc = json.load(open(os.path.join(data, "mcity_scene_foliage.json")))
parts = [p for a in doc["assets"] for p in a["parts"]]
parts += [p for a in doc["assets"] for c in a.get("clumps", []) for p in c["parts"]]
print(sum(1 for p in parts if not os.path.exists(os.path.join(data, p["mesh"]))))
PY2
)
if [ "$MISSING" -gt 0 ]; then
  echo "  $MISSING source meshes named by mcity_scene_foliage.json are missing from $DATA/assets" >&2
  echo "  re-run:  ./setup_mcity.sh --convert --foliage --skip-fetch" >&2
  exit 1
fi

# Bare levels spend on wood what the leafy ones spend on leaves, so the branches read.
echo "== 1/4  trees only, bare branches =="
$DEC --no-leaves --drop-shrubs --tree-wood 6000 \
     --lod-dir lod_trees_bare --out "$DATA/mcity_scene_trees_bare.json" | tail -2

echo "== 2/4  trees and shrubs, bare branches =="
$DEC --no-leaves --tree-wood 6000 --shrub-wood 1500 \
     --lod-dir lod_all_bare --out "$DATA/mcity_scene_all_bare.json" | tail -2

# With the shrubs gone the trees can afford a little more of both.
echo "== 3/4  trees only, with leaves =="
$DEC --drop-shrubs --tree-wood 3000 --tree-leaves 6000 \
     --lod-dir lod_trees_leaf --out "$DATA/mcity_scene_trees_leaf.json" | tail -2

echo "== 4/4  everything: trees and shrubs, with leaves =="
$DEC \
     --lod-dir lod_full --out "$DATA/mcity_scene_full.json" | tail -2
