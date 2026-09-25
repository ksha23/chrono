# Raw data for ksha23/chrono PR #33 (fix/sensor-physcam-params)

Machine: north-ubuntu, RTX 5070 Ti (sm_120), driver 580.173.02, CUDA 13.0, OptiX 9.0, GCC 13.3.

- `timing/timing.csv`: one row per run (5 scenarios x 5 reps x 2 arms, arms interleaved).
  `per_step_ms` is the mean wall time per step over steps 31 to 330 (one phys-cam frame per step),
  `upd_med_ms` the median `ChSensorManager::Update` time, `load1` the 1-minute load average at start,
  `hash` the FNV-1a hash of every host buffer per sensor. The PR table is the median, min and max
  of `per_step_ms` per scenario and arm.
- `mem/memory.csv` (from `mem/mem_*.log`): 360p phys-cam only, 100 Hz, 10000 frames, sampled every
  500 steps. `proc_MiB` is the per-process used memory from `nvidia-smi`.
- `nsys/api_per_frame.csv` (from `nsys/summ_*.txt`): CUDA runtime API calls in 60 steady-state
  frames of S1, split into the phys-cam render thread and all threads. `*_per_frame` columns are
  already divided by the 60 frames. Produced by `scripts/nsys_summ.py` from the nsys sqlite export.
- `ctest/`: `ctest -L sensor` logs of both arms (round 1).
- `det_*.log`: short determinism run (per-frame phys-cam hashes) of both arms.
- `round2/`: unit test and `ctest -L sensor` logs after the review response (non-blocking test stream),
  plus the wrong-stream mutation check.
- `driver/`: the headless timing driver (a copy of `demo_SEN_phys_cam` with fixed seed and hashing).
  Usage: `physcam_drv W H steps warmup rate_hz lidar(0/1) cam(0/1) mem_every hash(0/1)`.
- `scripts/`: build, timing and post-processing scripts as run.
