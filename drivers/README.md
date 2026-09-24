# Benchmark drivers for perf/multicore-addcontact-no-sharedptr (ksha23/chrono#50)

These files are not part of the PR. They are the headless drivers and scripts used to produce the timing table.
Paths inside the scripts point at the author's workspace on d33-ubuntu; adjust `W` before use.

- `prof.h`: per-step wall-clock timing hooks. Env: `PROF_THREADS`, `PROF_WARM` (untimed steps), `PROF_STEPS`
  (timed steps, then exit), `PROF_TAG`. Prints one `PROF_RESULT` line with median/min/max step time and a
  position/velocity checksum over all bodies after the last step.
- `mcore_granular.cpp`: column of spheres in a box, SMC or NSC (`PROF_METHOD`, `PROF_N`). `PROF_PROBE_RC=<step>`
  times `ReportContacts(container)` 20 times in isolation at that step and prints the median (`PROBE_RC`).
- `demo_MCORE_*.cpp`: copies of `src/demos/multicore` demos with `prof.h` included, `DoStepDynamics` replaced by
  `prof::step`, and the thread count taken from `PROF_THREADS`. `headless_vis.h` stubs out the VSG/Irrlicht calls.
- `build_drv.sh <base|fix> files...`: compiles drivers against one arm's source headers and shared libraries.
- `sweep.sh`: the full base vs fix sweep (5 repetitions, arms alternated, each run under a machine-wide `flock`).
  `sweep_mixer.sh`: the rerun of the two mixer demos after the checksum fix in `prof.h` (see the PR comments).
