#!/bin/bash
# Build the VSG-rendered scenario demo.
#
# Separate from build_opendrive.sh because this one links Chrono::VSG rather than Chrono::Sensor,
# and needs the VSG and vsgImGui headers. Use this demo to watch a scenario; use scenario_drive
# when a simulated camera is actually the point.
set -e
source /opt/homebrew/Caskroom/miniconda/base/etc/profile.d/conda.sh
conda activate chronopc
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
cd "$ROOT"
VSGIMGUI="${VSGIMGUI_ROOT:-$HOME/opt/vsgImGui}"
INC="-I src -I build -isystem $CONDA_PREFIX/include/eigen3 -isystem $CONDA_PREFIX/include -isystem $VSGIMGUI/include -I src/chrono/collision/bullet -I src/chrono_thirdparty/HACDv2 -I src/chrono_thirdparty/yaml-cpp/include"
LIBS="-L build/lib -lChrono_core -lChrono_vehicle -lChronoModels_vehicle -lChrono_vsg -lChrono_vehicle_vsg -lChrono_scenario -Wl,-rpath,build/lib"
for demo in scenario_vsg intersection_vsg map_viewer_vsg mcity_drive_vsg; do
  echo "building $demo"
  c++ -std=c++17 -O2 "$DIR/$demo.cpp" $INC $LIBS -o "$DIR/$demo"
done
echo "done.  run:  ./demos_live/scenario_vsg [scenario.xosc] [max_seconds] [ego_speed_mps]"
echo "        ./demos_live/intersection_vsg [max_seconds] [ego_speed_mps]"
echo "        KEEP_OPEN=1 ...                        # run until the window is closed"
