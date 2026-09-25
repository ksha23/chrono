#!/bin/bash
# configure (shared configure.sh) + build both arms sequentially at -j16
W=~/chrono-audit/swarm/sensor-physcam-params
T="Chrono_sensor demo_SEN_phys_cam demo_SEN_buildtest utest_SEN_analytic_render utest_SEN_camera_convergence utest_SEN_data_access utest_SEN_dynamic_sensors utest_SEN_gps utest_SEN_interface utest_SEN_optixengine utest_SEN_optixgeometry utest_SEN_optixpipeline utest_SEN_radar utest_SEN_rng_streams utest_SEN_scene_lights utest_SEN_shader_staging utest_SEN_threadsafety utest_SEN_physcam_ops"
for a in base fix; do
  ~/chrono-audit/campaign/configure.sh $W/$a $W/build-$a > $W/configure-$a.out 2>&1
  echo "CONFIGURE $a rc=$?" >> $W/build-status.txt
  nice -n 5 ninja -C $W/build-$a -j16 $T > $W/build-$a.out 2>&1
  echo "BUILD $a rc=$? $(date -Is)" >> $W/build-status.txt
done
