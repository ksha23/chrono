#!/bin/bash
# Base vs fix timing sweep, arms interleaved per run, each run under flock. One line per run.
W=/home/kyle/chrono-audit/swarm/multicore-addcontact-no-sharedptr
OUT=$W/logs/sweep_mixer_$(date +%Y%m%d_%H%M%S).txt
cd $W/run
THREADS="${THREADS:-1 4 8 16}"; REPS=${REPS:-5}
run1() { # arm scen T rep bin envs...
  local arm=$1 scen=$2 T=$3 rep=$4 bin=$5; shift 5
  local ld=$(cut -d" " -f1 /proc/loadavg)
  local line=$(env "$@" PROF_THREADS=$T PROF_TAG=$scen flock ~/chrono-audit/timing.lock timeout 1200 $W/bin/$arm/$bin 2>&1 | grep -E "PROBE_RC|PROF_RESULT" | tr "\n" " ")
  echo "arm=$arm scen=$scen T=$T rep=$rep load1=$ld $line" >> $OUT
}
scen() { # name bin envs...
  local name=$1 bin=$2; shift 2
  for T in $THREADS; do
    if (( (rep + T) % 2 )); then A="base fix"; else A="fix base"; fi
    for arm in $A; do run1 $arm $name $T $rep $bin "$@"; done
  done
}
for rep in $(seq 1 $REPS); do
  scen mixerSMC demo_MCORE_mixerSMC PROF_WARM=100 PROF_STEPS=2000
  scen mixerNSC demo_MCORE_mixerNSC PROF_WARM=100 PROF_STEPS=2000
done
echo "wrote $OUT"; touch $W/logs/sweep_mixer_done
