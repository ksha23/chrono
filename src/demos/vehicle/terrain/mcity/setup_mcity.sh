#!/bin/bash
# Turn the Mcity digital twin into a Chrono scene, in one command.
#
# Chrono ships this pipeline, not the scene. The Mcity assets are a third-party dataset of a few
# hundred megabytes under its own licence, and pinning a copy inside Chrono would be both large
# and immediately stale. What is version-controlled here is the conversion: the scripts, the
# surface catalogue, and the material rules. Everything under data/mcity/ is generated and can be
# deleted and rebuilt.
#
#   ./setup_mcity.sh --repo /path/to/mcity-digital-twin    use a clone you already have
#   ./setup_mcity.sh                                       fetch the pieces over HTTPS
#
# Options:
#   --bundle SRC   install a pre-converted scene (URL or local .tar.gz) and stop. This is the
#                  easy path: no USD toolchain, no 3.2 GB clone, no conversion -- roughly 190 MB
#                  instead. Use it unless you are changing the conversion itself.
#   --repo DIR     convert from a local clone instead of downloading
#   --out DIR      where to write the converted scene (default: <chrono>/data/mcity)
#   --foliage      include vegetation (adds ~200 MB and the LOD configurations)
#   --skip-fetch   reuse whatever is already in --out
#
# The clone is large; --repo is much the faster route if you have one:
#   git clone https://github.com/mcity/mcity-digital-twin
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
OUT="$ROOT/data/mcity"
REPO_DIR=""
BUNDLE=""
FOLIAGE=0
SKIP_FETCH=0

while [ $# -gt 0 ]; do
  case "$1" in
    --bundle) BUNDLE="$2"; shift 2 ;;
    --repo) REPO_DIR="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --foliage) FOLIAGE=1; shift ;;
    --skip-fetch) SKIP_FETCH=1; shift ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "$OUT"

# The pre-converted path. Nothing below this point runs: no Python, no USD, no conversion.
if [ -n "$BUNDLE" ]; then
  echo "== installing pre-converted scene =="
  case "$BUNDLE" in
    http://*|https://*)
      echo "  downloading $BUNDLE"
      curl -fL --progress-bar "$BUNDLE" -o "$OUT/.bundle.tar.gz" || {
        echo "  download failed" >&2; exit 1; }
      SRC="$OUT/.bundle.tar.gz" ;;
    *)
      [ -f "$BUNDLE" ] || { echo "  no such file: $BUNDLE" >&2; exit 1; }
      SRC="$BUNDLE" ;;
  esac
  echo "  extracting into $OUT"
  tar -xzf "$SRC" -C "$OUT"
  rm -f "$OUT/.bundle.tar.gz"
  [ -f "$OUT/mcity_scene.json" ] || { echo "  archive did not contain a scene manifest" >&2; exit 1; }
  echo
  echo "done -- scene installed to $OUT"
  echo "  cd bin && ./demo_VEH_McityDrive"
  exit 0
fi

if ! python3 -c "import pxr" 2>/dev/null; then
  echo "usd-core is required:  python3 -m pip install usd-core" >&2
  exit 1
fi

USD_ROOT="Omniverse/Collected_McityMap_NSR_v4_1_6"

if [ "$SKIP_FETCH" = 0 ]; then
  if [ -n "$REPO_DIR" ]; then
    echo "== copying from $REPO_DIR =="
    [ -d "$REPO_DIR/$USD_ROOT" ] || { echo "  not an mcity-digital-twin clone: $USD_ROOT missing" >&2; exit 1; }
    mkdir -p "$OUT/usd"
    cp "$REPO_DIR/CARLA/source_version/McityMap/OpenDrive/McityMap_Main.xodr" "$OUT/"
    cp "$REPO_DIR/$USD_ROOT/McityMap_Main.usdc" "$OUT/usd/"
    for sub in Props FinalTrafficLights; do
      cp -R "$REPO_DIR/$USD_ROOT/$sub" "$OUT/usd/" 2>/dev/null || true
    done
    if [ "$FOLIAGE" = 1 ]; then
      cp -R "$REPO_DIR/$USD_ROOT/SubUSDs" "$OUT/usd/" 2>/dev/null || true
      cp "$REPO_DIR/$USD_ROOT/Foliage_Instanced.usdc" "$OUT/usd/" 2>/dev/null || true
    fi
    # Textures and MDLs are resolved by name later, so index the clone rather than copy it.
    mkdir -p "$OUT/mdl"
    find "$REPO_DIR/$USD_ROOT" -name '*.mdl' -exec cp {} "$OUT/mdl/" \; 2>/dev/null || true
    export MCITY_LOCAL_REPO="$REPO_DIR"
  else
    echo "== fetching over HTTPS =="
    if [ "$FOLIAGE" = 1 ]; then "$DIR/fetch_mcity.sh" --foliage; else "$DIR/fetch_mcity.sh"; fi
  fi
fi

echo "== resolving materials and textures =="
python3 "$DIR/resolve_textures.py" --dir "$OUT"

echo "== converting geometry =="
if [ "$FOLIAGE" = 1 ]; then
  python3 "$DIR/usd_to_chrono.py" --in "$OUT" --out "$OUT" --exclude-groups ""
  cp "$OUT/mcity_scene.json" "$OUT/mcity_scene_foliage.json"
  python3 "$DIR/usd_to_chrono.py" --in "$OUT" --out "$OUT"
  echo "== building vegetation configurations =="
  "$DIR/build_configs.sh"
else
  python3 "$DIR/usd_to_chrono.py" --in "$OUT" --out "$OUT"
fi

echo
echo "done -- scene written to $OUT"
echo
echo "drive it from a build tree:"
echo "  cd bin && ./demo_VEH_McityDrive"
if [ "$FOLIAGE" = 1 ]; then
  echo "  cd bin && ./demo_VEH_McityDrive --foliage trees"
  echo
  echo "vegetation levels: none | trees | trees-leaf | shrubs | full"
fi
echo "  ./demo_VEH_McityDrive --help    for all options"
