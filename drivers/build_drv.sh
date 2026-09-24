#!/bin/bash
# usage: build_drv.sh <base|fix> files...   Compile campaign drivers against one arm (source headers + libs of that arm).
set -e
W=/home/kyle/chrono-audit/swarm/multicore-addcontact-no-sharedptr
A=$1; shift
B=$W/build-$A; S=$W/$A/src; D=$W/drv
CXXFLAGS="-DCH_IGNORE_DEPRECATED -DEIGEN_MAX_ALIGN_BYTES=16 -DEIGEN_MAX_STATIC_ALIGN_BYTES=16 -DNDEBUG -DTHRUST_DEVICE_SYSTEM=THRUST_DEVICE_SYSTEM_OMP -DTHRUST_HOST_SYSTEM=THRUST_HOST_SYSTEM_OMP -D_ENABLE_EXTENDED_ALIGNED_STORAGE -I$S -I$B -I$S/chrono_thirdparty/yaml-cpp/include -isystem $S/chrono/collision/bullet -isystem $S/chrono_thirdparty -isystem $S/chrono_thirdparty/HACDv2 -isystem /usr/include/eigen3 -isystem /home/kyle/chrono-audit/dem/cccl-inc -isystem /home/kyle/chrono-audit/campaign/deps/spectra/include -g -fno-omit-frame-pointer -O3 -std=gnu++17 -march=native -Wno-deprecated -fopenmp -I$D"
LIBS="-Wl,-rpath,$B/lib $B/lib/libChrono_multicore.so $B/lib/libChrono_core.so /usr/lib/gcc/x86_64-linux-gnu/13/libgomp.so -lpthread $B/lib/libyaml-cpp.so.0.8.0"
mkdir -p $W/bin/$A
for f in "$@"; do
  n=$(basename $f .cpp)
  /usr/bin/c++ $CXXFLAGS -DPROF_MULTICORE ${EXTRA_DEFS} -o $W/bin/$A/$n $f $LIBS && echo "built $A/$n" || { echo "FAILED $A/$n"; exit 1; }
done
