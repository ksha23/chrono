#!/bin/bash
# Get the Mcity digital twin as a Chrono scene, in one command.
#
# Chrono ships this pipeline, not the scene. The Mcity assets are a third-party dataset of a few
# hundred megabytes under its own licence, and pinning a copy inside Chrono would be both large
# and immediately stale. What is version-controlled here is the conversion: the scripts, the
# surface catalogue, and the material rules. Everything under data/mcity/ is generated and can be
# deleted and rebuilt.
#
# The conversion gives everybody the same output, so a converted copy is published and that is
# what this installs by default: one 200 MB download, with no USD toolchain and no 3.2 GB clone.
#
#   ./setup_mcity.sh                                       download the converted scene
#   ./setup_mcity.sh --foliage                             and the vegetation levels, 158 MB more
#   ./setup_mcity.sh --convert                             rebuild it from the upstream USD
#   ./setup_mcity.sh --repo /path/to/mcity-digital-twin    rebuild it from a clone you have
#
# Options:
#   --convert      run the conversion instead of downloading its result. Needs usd-core. Use it
#                  when changing the conversion itself.
#   --repo DIR     convert from a local clone instead of fetching the sources over HTTPS.
#                  Implies --convert. The clone is large:
#                    git clone https://github.com/mcity/mcity-digital-twin
#   --foliage      include vegetation and its LOD configurations. Downloads the published
#                  add-on, or with --convert fetches ~200 MB of sources and builds it.
#   --skip-fetch   convert whatever sources are already in --out. Implies --convert.
#   --bundle SRC   install some other pre-converted archive (URL or local .tar.gz) and stop.
#   --out DIR      where to write the scene (default: <chrono>/data/mcity)
#
# The published scene comes from package_mcity.sh. To publish a new one, upload the archives it
# writes and update the URL and hashes below.
set -e

# The published scene. Its hash is checked before anything is extracted, so a changed or truncated
# download stops here instead of turning up later as a half-loaded scene.
SCENE_URL="https://github.com/ksha23/chrono-mcity/releases/download/v1"
SCENE_BASE="mcity_scene_base.tar.gz"
SCENE_BASE_SHA256="41b0e14eb0a10609fde95621a2085ab194d8aa4de45054bb8f09a76a766a41f7"
# Vegetation, as an add-on that extracts over the base scene.
SCENE_FOLIAGE="mcity_scene_foliage.tar.gz"
SCENE_FOLIAGE_SHA256="246434ba4e3249fd50b08cf50411b38f48bd6d451575a3731401139c35995c87"

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
CONVERT=0
FOLIAGE=0
SKIP_FETCH=0

while [ $# -gt 0 ]; do
  case "$1" in
    --convert) CONVERT=1; shift ;;
    --bundle) BUNDLE="$2"; shift 2 ;;
    --repo) REPO_DIR="$2"; CONVERT=1; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --foliage) FOLIAGE=1; shift ;;
    --skip-fetch) SKIP_FETCH=1; CONVERT=1; shift ;;
    -h|--help) sed -n '2,31p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 1 ;;
  esac
done

mkdir -p "$OUT"

sha256_of() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d' ' -f1
  else shasum -a 256 "$1" | cut -d' ' -f1; fi
}

# install_archive SRC [SHA256] -- SRC is a URL or a local .tar.gz.
install_archive() {
  local src="$1" want="${2:-}" file="" tmp=""
  case "$src" in
    http://*|https://*)
      tmp="$OUT/.download.tar.gz"
      echo "  downloading $src"
      curl -fL --progress-bar "$src" -o "$tmp" || {
        rm -f "$tmp"; echo "  download failed" >&2; exit 1; }
      file="$tmp" ;;
    *)
      [ -f "$src" ] || { echo "  no such file: $src" >&2; exit 1; }
      file="$src" ;;
  esac
  if [ -n "$want" ]; then
    local got
    got="$(sha256_of "$file")"
    if [ "$got" != "$want" ]; then
      echo "  checksum mismatch for $src" >&2
      echo "    expected $want" >&2
      echo "    got      $got" >&2
      if [ -n "$tmp" ]; then rm -f "$tmp"; fi
      exit 1
    fi
  fi
  echo "  extracting into $OUT"
  tar -xzf "$file" -C "$OUT"
  if [ -n "$tmp" ]; then rm -f "$tmp"; fi
}

installed() {
  [ -f "$OUT/mcity_scene.json" ] || { echo "  archive did not contain a scene manifest" >&2; exit 1; }
  echo
  echo "done -- scene installed to $OUT"
  if [ "$OUT" = "$ROOT/data/mcity" ]; then
    echo "  cd bin && ./demo_VEH_McityDrive"
  else
    echo "  cd bin && ./demo_VEH_McityDrive --data $OUT"
  fi
  if [ "$FOLIAGE" = 1 ]; then
    echo "  vegetation levels:  --foliage none | trees | trees-leaf | shrubs | full"
  fi
  exit 0
}

# The pre-converted paths. Nothing below them runs: no Python, no USD, no conversion.
if [ -n "$BUNDLE" ]; then
  echo "== installing pre-converted scene =="
  install_archive "$BUNDLE"
  installed
fi

if [ "$CONVERT" = 0 ]; then
  echo "== installing the published scene =="
  install_archive "$SCENE_URL/$SCENE_BASE" "$SCENE_BASE_SHA256"
  if [ "$FOLIAGE" = 1 ]; then
    install_archive "$SCENE_URL/$SCENE_FOLIAGE" "$SCENE_FOLIAGE_SHA256"
  fi
  installed
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
