#!/bin/bash
# Convert esmini's pedestrian and cyclist models to Wavefront OBJ for Chrono.
#
# Chrono ships no humanoid mesh of any kind, so scenario pedestrians and cyclists would otherwise
# be drawn as stacked primitives. esmini has both, but as OpenSceneGraph .osgb binaries that
# nothing in this build can read -- esmini itself is built here with USE_OSG=OFF.
#
# The converted meshes are deliberately NOT committed. esmini's code is MPL-2.0, but its 3D models
# carry no explicit licence and at least one of them (box_cc_by.osgb) is named for CC-BY
# attribution terms. Rather than redistribute assets whose terms are unclear, this regenerates
# them locally from whatever esmini checkout you already have.
#
# Requires osgconv:  brew install open-scene-graph   (or your platform's OpenSceneGraph package)
#
# Usage:  ./demos_live/convert_esmini_models.sh [esmini_root]
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
ESMINI="${1:-${ESMINI_ROOT:-$ROOT/../esmini}}"
OUT="$ROOT/data/vehicle/pedestrian"

if ! command -v osgconv >/dev/null 2>&1; then
  echo "osgconv not found. Install OpenSceneGraph (brew install open-scene-graph) and retry." >&2
  echo "Without it, pedestrians and cyclists fall back to primitive figures." >&2
  exit 1
fi

mkdir -p "$OUT"
for model in walkman cyclist; do
  src="$ESMINI/resources/models/$model.osgb"
  if [ ! -f "$src" ]; then
    echo "missing $src -- skipping" >&2
    continue
  fi
  echo "converting $model"
  osgconv "$src" "$OUT/$model.obj"

  # osgconv drops the source textures -- the OBJ it writes has no texture coordinates at all --
  # and emits a near-black flat material (Kd 0.073) in their place, which renders as a dark blob.
  # Replace it with a visible diffuse so the silhouette reads. The geometry is the point here;
  # the texture is lost in conversion and is a known limitation.
  if [ -f "$OUT/$model.mtl" ]; then
    cat > "$OUT/$model.mtl" <<MTL
newmtl material_1
       Ka 0.20 0.21 0.24 1
       Kd 0.42 0.45 0.52 1
       Ks 0.10 0.10 0.10 1
       Ns 20.0
MTL
  fi
done

echo "done -> $OUT"
