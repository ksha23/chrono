#!/bin/bash
# Build the OpenDRIVE and OpenSCENARIO demos. Same flags as build.sh plus Chrono_scenario, which
# needs the tree configured with -DCH_ENABLE_MODULE_SCENARIO=ON -DEsmini_ROOT=<esmini>.
#
# esmini's shared libraries are resolved through the rpaths CMake bakes into Chrono_scenario, and
# are also copied into build/lib by the module's POST_BUILD step, so nothing needs to be on
# LD_LIBRARY_PATH / DYLD_LIBRARY_PATH.
set -e
source /opt/homebrew/Caskroom/miniconda/base/etc/profile.d/conda.sh
conda activate chronopc
DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$DIR/.." && pwd)"
cd "$ROOT"
INC="-I src -I build -isystem $CONDA_PREFIX/include/eigen3 -I src/chrono/collision/bullet -I src/chrono_thirdparty/HACDv2 -I src/chrono_thirdparty/yaml-cpp/include"
LIBS="-L build/lib -lChrono_core -lChrono_vehicle -lChronoModels_vehicle -lChrono_sensor -lChrono_scenario -Wl,-rpath,build/lib"
for demo in opendrive_drive scenario_drive; do
  echo "building $demo"
  c++ -std=c++17 -O2 "$DIR/$demo.cpp" $INC $LIBS -o "$DIR/$demo"
done
echo "done.  run:  ./demos_live/opendrive_drive [road.xodr] [spawn_s]"
echo "          ./demos_live/scenario_drive  [scenario.xosc] [max_seconds] [ego_speed_mps]"
