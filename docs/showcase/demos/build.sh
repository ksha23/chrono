#!/bin/bash
# Build every showcase demo against an existing Chrono build tree.
#
#   CHRONO_ROOT   repo root (default: four levels up from this script)
#   CHRONO_BUILD  build dir containing lib/ (default: $CHRONO_ROOT/build)
#   OUT_DIR       where the binaries go   (default: alongside this script, ./bin)
#
# With no arguments every showcase_*.cpp and verify_*.cpp is built. Pass one or more demo
# names (with or without the .cpp suffix) to build just those, e.g.
#
#   ./build.sh verify_render_math
#
# The Chrono libs must already be built (e.g. `ninja Chrono_sensor` in $CHRONO_BUILD).
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
: "${CHRONO_ROOT:=$(cd "$DIR/../../.." && pwd)}"
: "${CHRONO_BUILD:=$CHRONO_ROOT/build}"
: "${OUT_DIR:=$DIR/bin}"
mkdir -p "$OUT_DIR"
cd "$CHRONO_ROOT"

EIGEN="${EIGEN_INCLUDE:-${CONDA_PREFIX:-/usr/local}/include/eigen3}"
INC="-I src -I $CHRONO_BUILD -isystem $EIGEN -I src/chrono/collision/bullet -I src/chrono_thirdparty/HACDv2 -I src/chrono_thirdparty/yaml-cpp/include"
LIBS="-L $CHRONO_BUILD/lib -lChrono_core -lChrono_vehicle -lChronoModels_vehicle -lChrono_sensor -Wl,-rpath,$CHRONO_BUILD/lib"

if [ "$#" -gt 0 ]; then
  SOURCES=()
  for arg in "$@"; do
    SOURCES+=("$DIR/$(basename "${arg%.cpp}").cpp")
  done
else
  SOURCES=("$DIR"/showcase_*.cpp "$DIR"/verify_*.cpp)
fi

for src in "${SOURCES[@]}"; do
  name="$(basename "${src%.cpp}")"
  echo "building $name"
  c++ -std=c++17 -O2 -DCHRONO_SHOWCASE_ROOT="\"$CHRONO_ROOT\"" "$src" $INC $LIBS -o "$OUT_DIR/$name"
done
echo "done -> $OUT_DIR"
