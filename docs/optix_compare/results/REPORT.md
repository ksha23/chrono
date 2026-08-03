# OptiX quarter-panel comparison — results (issue #31)

Machine: RTX 5090 laptop, driver 595.84, CUDA 13.1, OptiX SDK 7.7, Ubuntu 24.04.
Branch: `feature/sensor_metal_rt` @ `c265b5c38`, fresh clone in `~/chrono_optix`.
Built with `CH_ENABLE_MODULE_SENSOR/VEHICLE/VEHICLE_MODELS=ON`, `CH_USE_SENSOR_OPTIX=ON`,
`CH_USE_SENSOR_VULKAN_RT=OFF`, NVRTC on, `CHRONO_CUDA_ARCHITECTURES=120`.

All frames here are `frame_88` (≈2.9 s, car fully settled). `metal_ss1_reference_flipped.png` is the
shipped Metal reference flipped vertically so it's orientation-matched to the OptiX saves.

## Answers to the doc's questions

**1. Does OptiX at ss=1 show the wavy/mottled reflection on the left rear quarter panel?**
Yes. OptiX at ss=1 (LEGACY, no GI, no denoiser) shows the same *class* of artifact: sharp,
high-contrast mirror reflections that break into wavy/lumpy bands on the curved panels — most
visibly the lumpy waviness along the sail panel / rear-window frame / shoulder line (same places as
Metal), plus strong mottled dark shapes on the lower quarter and rear fascia. OptiX is **not**
clean at matched settings. If anything the OptiX mottling is *stronger*, because its reflected
environment has more dark content (see caveat below).

**2. Does ss=2 / ss=3 remove it?**
No. ss=3 is nearly indistinguishable from ss=1 (mean |ss1−ss3| = 2.5/255 over the full frame,
mostly geometric edge AA). The wavy bands are actual single-mirror-ray reflection *content*, not
pixel-grid aliasing, so supersampling only softens their edges slightly.

**3. How does OptiX look with the denoiser?**
Essentially identical. The denoiser was confirmed active (console: "Sensor: quarter_panel requested
OptiX denoiser"; `GetUseDenoiser()` is the only gate in `ChOptixEngine`). Mean |ss1−ss1_denoised| =
0.4/255. On the deterministic LEGACY integrator there is no stochastic noise to remove, so the
denoiser passes the sharp reflection bands through. OptiX's clean look in other demos does **not**
come from the denoiser rescuing LEGACY reflections.

**4. Settings actually used**
1280×720, 60° HFOV, PINHOLE, LEGACY, use_diffuse_reflect=false, gamma 2.2; denoiser off except in
the `_denoise` run. Supersample per filename.

## Caveat — the two references don't see the same environment

The reflection *content* differs because the scenes don't fully match: the OptiX render shows the
tiled terrain texture and the HDR's tree line, while the Metal reference's world is mostly white
(untextured-looking ground, washed-out sky, only a thin tree strip). So Metal's mirror rays mostly
reflect white sky (→ smoother-looking panels), while OptiX reflects trees/tiles/its own body
(→ more visible mottle). The Metal paint also reads lighter/brighter overall. Worth checking why the
Metal side's terrain texture / env-map contribution differs before comparing tone or shininess 1:1.
The *self-reflection waviness* along the shoulder/sail/window-frame region, however, is present in
both and looks structurally the same.

## Verdict

The wavy self-reflection at ss=1 is **expected OptiX-parity behavior** of the LEGACY single sharp
mirror ray — option (a) in the doc. Neither supersampling nor the denoiser removes it on OptiX. A
glossy/blurred reflection (or a real GI integrator) would be an *enhancement over* OptiX, not a
parity fix.

## Local-only changes made on this machine (not committed)

- `src/chrono_sensor/optix/ChFilterOptixRender.cpp`: the branch sets
  `OptixDenoiserOptions::denoiseAlpha`, which only exists in OptiX ≥ 8.0; with the installed
  OptiX 7.7 SDK this doesn't compile. Guarded with `#if OPTIX_VERSION >= 80000` and set
  `OptixDenoiserParams::denoiseAlpha = OPTIX_DENOISER_ALPHA_MODE_COPY` for < 8.0. Worth
  cherry-picking into the branch for OptiX 7.x users.
- `src/demos/sensor/demo_SEN_metal_quarterpanel.cpp`: added optional argv[2] to enable the
  denoiser, and per-run output dirs (`quarterpanel_out/optix_ss<N>[_denoise]/`) so runs don't
  overwrite each other.
