#!/bin/bash
# Build the OpenDRIVE-in-Chrono demo. Same flags as build.sh plus Chrono_scenario, which needs
# the tree configured with -DCH_ENABLE_MODULE_SCENARIO=ON -DEsmini_ROOT=<esmini>.
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
echo "building opendrive_drive"
c++ -std=c++17 -O2 "$DIR/opendrive_drive.cpp" $INC $LIBS -o "$DIR/opendrive_drive"
echo "done.  run:  ./demos_live/opendrive_drive [road.xodr] [spawn_s]"
