#!/bin/bash
# Measure what a scenery manifest costs, without risking the machine.
#
# Two safeguards, because a manifest that is too big to render is also big enough to make a
# desktop unusable while you find that out:
#   - the probe links no visual system, so nothing reaches the GPU;
#   - a watchdog samples RSS and kills the probe if it crosses a ceiling.
#
# Usage:  ./probe_scenery.sh <manifest.json> [max_tris_per_asset] [rss_limit_gb]
set -u
source /opt/homebrew/Caskroom/miniconda/base/etc/profile.d/conda.sh
conda activate chronopc

DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/../.." && pwd)"
cd "$ROOT"

MANIFEST="${1:?manifest required}"
MAXTRI="${2:-0}"
LIMIT_GB="${3:-6}"
BIN="$DIR/probe_scenery"

if [ ! -x "$BIN" ] || [ "$DIR/probe_scenery.cpp" -nt "$BIN" ]; then
  echo "building probe"
  c++ -std=c++17 -O2 "$DIR/probe_scenery.cpp" \
      -I src -I build -isystem "$CONDA_PREFIX/include/eigen3" -isystem "$CONDA_PREFIX/include" \
      -I src/chrono/collision/bullet -I src/chrono_thirdparty/HACDv2 \
      -I src/chrono_thirdparty/yaml-cpp/include \
      -L build/lib -lChrono_core -lChrono_scenario -Wl,-rpath,build/lib -o "$BIN" || exit 1
fi

echo "probing $(basename "$MANIFEST")  (kill above ${LIMIT_GB} GB)"
"$BIN" "$MANIFEST" "$MAXTRI" &
PID=$!

PEAK=0
while kill -0 "$PID" 2>/dev/null; do
  # ps reports RSS in KB.
  RSS=$(ps -o rss= -p "$PID" 2>/dev/null | tr -d ' ')
  [ -z "$RSS" ] && break
  [ "$RSS" -gt "$PEAK" ] && PEAK=$RSS
  if [ "$RSS" -gt $((LIMIT_GB * 1024 * 1024)) ]; then
    echo "  !! RSS $((RSS / 1048576)) GB exceeded ${LIMIT_GB} GB -- killing probe"
    kill -9 "$PID" 2>/dev/null
    wait "$PID" 2>/dev/null
    echo "  peak RSS before kill: $((PEAK / 1048576)) GB"
    exit 2
  fi
  sleep 0.25
done
wait "$PID" 2>/dev/null
echo "  peak RSS: $(echo "$PEAK" | awk '{printf "%.2f", $1/1048576}') GB"
