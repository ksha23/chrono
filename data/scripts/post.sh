#!/bin/bash
# after timing: memory run, nsys traces, ctest -L sensor on both arms (all under the lock)
W=$HOME/chrono-audit/swarm/sensor-physcam-params; R=$W/runs; mkdir -p $R/mem $R/nsys $R/ctest
unset DISPLAY WAYLAND_DISPLAY
while ! grep -q DONE $R/timing/timing.csv 2>/dev/null; do sleep 20; done
for a in base fix; do
  cd $W/build-$a/bin
  flock $HOME/chrono-audit/timing.lock timeout 1200 $W/drv/bin-$a/physcam_drv 640 360 10010 10 100 0 0 500 0 > $R/mem/mem_$a.log 2>&1; echo "mem $a rc=$?" >> $R/post.status
done
for a in base fix; do
  cd $W/build-$a/bin
  flock $HOME/chrono-audit/timing.lock nsys profile --trace=cuda,nvtx,osrt --sample=none --cpuctxsw=none --force-overwrite=true \
    -o $R/nsys/ns_$a $W/drv/bin-$a/physcam_drv 1280 720 70 10 100 0 1 0 1 > $R/nsys/ns_$a.log 2>&1; echo "nsys $a rc=$?" >> $R/post.status
  nsys export --type sqlite --force-overwrite=true -o $R/nsys/ns_$a.sqlite $R/nsys/ns_$a.nsys-rep > /dev/null 2>&1
done
for a in base fix; do
  echo "### $a load $(cut -d" " -f1-3 /proc/loadavg)" >> $R/ctest/summary.txt
  flock $HOME/chrono-audit/timing.lock ctest --test-dir $W/build-$a -L sensor --timeout 1200 > $R/ctest/ctest_$a.log 2>&1
  echo "ctest $a rc=$?" >> $R/post.status
  grep -E "tests passed|Total Test time|\*\*\*|Failed|Not Run" $R/ctest/ctest_$a.log >> $R/ctest/summary.txt
done
echo POSTDONE >> $R/post.status
