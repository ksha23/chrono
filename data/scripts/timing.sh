#!/bin/bash
# 5 scenarios x 5 reps x 2 arms, interleaved, each run under the shared timing lock.
W=$HOME/chrono-audit/swarm/sensor-physcam-params; R=$W/runs/timing; mkdir -p $R
unset DISPLAY WAYLAND_DISPLAY
declare -A SC=( [S1_720p_cam_phys]="1280 720 330 30 100 0 1 0 1" [S2_1080p_cam_phys]="1920 1080 330 30 100 0 1 0 1"
               [S3_720p_cam_lidar_phys]="1280 720 330 30 100 1 1 0 1" [S4_720p_phys]="1280 720 330 30 100 0 0 0 1"
               [S5_360p_phys]="640 360 330 30 100 0 0 0 1" )
echo "scenario,rep,arm,per_step_ms,upd_med_ms,load1,hash" > $R/timing.csv
for rep in 1 2 3 4 5; do
 for s in S1_720p_cam_phys S2_1080p_cam_phys S3_720p_cam_lidar_phys S4_720p_phys S5_360p_phys; do
  arms="base fix"; [ $((rep % 2)) -eq 0 ] && arms="fix base"
  for a in $arms; do
   cd $W/build-$a/bin
   log=$R/${s}_${a}_$rep.log
   flock $HOME/chrono-audit/timing.lock bash -c "echo LOAD \$(cut -d\" \" -f1 /proc/loadavg) > $log; timeout 600 $W/drv/bin-$a/physcam_drv ${SC[$s]} >> $log 2>&1; echo RC \$? >> $log"
   ps=$(grep -oP "per_step_ms=\K[\d.]+" $log); um=$(grep -oP "SENPROF update\s+n=\d+ mean=[\d.]+ med=\K[\d.]+" $log)
   ld=$(grep -oP "^LOAD \K\S+" $log); h=$(grep -oP "HASH \K.*" $log | tr " " "_")
   echo "$s,$rep,$a,$ps,$um,$ld,$h" >> $R/timing.csv
  done
 done
done
echo DONE >> $R/timing.csv
