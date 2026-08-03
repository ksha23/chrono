# OptiX vs Metal — car-paint reflection comparison (issue #31)

**Audience:** an agent/engineer on a machine with an NVIDIA GPU (CUDA + OptiX) who can build and run
Chrono::Sensor with the **OptiX** backend. **Goal:** render one specific scene on OptiX and report whether
the OptiX backend shows the same car-paint reflection artifact we see on the new **Metal** backend, so we can
decide whether the Metal look is a bug or just expected behavior.

You do **not** need to know anything about the Metal backend to do this. Just build Chrono normally with the
sensor module + OptiX, run the demo, and send back the images + observations.

---

## 1. Background — what we're chasing

This branch adds a native **Metal (Apple GPU) ray-tracing backend** to Chrono::Sensor, meant to match the
OptiX backend. One artifact remains on the Metal side (issue **#31**):

> On the car's **curved body panels** (most visibly the **left rear quarter panel / sail panel**), the paint
> shows a **wavy / mottled dark band**. We traced it (on Metal) to the **car reflecting its own body against
> the bright sky**: a single sharp mirror reflection ray sweeps across the car's own silhouette, so the
> reflected color flips hard between the dark body and the bright sky, and that boundary aliases into a wavy
> band on the curved surface. It is camera-stable and present even with no trees in the scene.

Our Metal reflection code is a faithful port of Chrono's OptiX **legacy** camera shader
(`CalculateContributionToPixel` in `src/chrono_sensor/optix/shaders/camera_legacy_shader.cuh`): a **single
sharp mirror ray**, weighted additively by `(1-rough)^2 * metallic^2 * F*D*G*NdL/(4π)` and clamped. Since
OptiX's legacy path also traces a single sharp reflection ray, **we expect OptiX to show the same wavy
self-reflection at the same settings** — but we want to confirm it, and see how it differs (if at all).

**The key questions for you:**
1. At matched settings (supersample = 1, LEGACY integrator, no GI, no denoiser), does the **OptiX** render
   show the same wavy/mottled reflection on the left rear quarter panel?
2. Does raising the supersample factor (2, 3) clean it up on OptiX?
3. How does OptiX look with its **denoiser** enabled?

---

## 2. The scene

A stationary Audi on a flat terrain patch, under the shipped `sensor/textures/sky_2_4k.hdr` environment map,
with the camera parked looking at the **left rear quarter panel**. It uses **only data that ships with
Chrono** (no external data directory needed).

- **Metal demo:** `demos_live/quarterpanel.cpp` (already run on our side; reference image below).
- **OptiX demo (yours):** `src/demos/sensor/demo_SEN_metal_quarterpanel.cpp` — the exact twin, using the
  standard `ChScene` API + `ChFilterVisualize`/`ChFilterSave`.

Both use identical: Audi JSON model, flat `RigidTerrain` patch, `sky_2_4k.hdr` background, ambient
`(0.35,0.35,0.37)`, a directional "sun", and the **same camera** — 1280×720, 60° HFOV, attached to the
chassis at offset `pos=(-3.0, 3.0, 1.7)` looking at `(-1.4, 0.7, 0.75)` (chassis frame; +x forward, +y left),
**supersample=1, PINHOLE, LEGACY integrator, no diffuse/GI, no denoiser, gamma 2.2**.

### Our Metal reference (supersample = 1)
`docs/optix_compare/metal_quarterpanel_ss1.png` — note the mottled/wavy reflection on the rear quarter panel.
(Depending on your image viewer the saved frame may appear vertically flipped vs. the live window — that's a
`ChFilterSave` orientation detail, not part of the artifact.)

---

## 3. Build (OptiX backend)

From a normal Chrono checkout of **this branch**, configure with the sensor module and OptiX enabled, e.g.:

```bash
mkdir build && cd build
cmake .. \
  -DCH_ENABLE_MODULE_SENSOR=ON \
  -DCH_ENABLE_MODULE_VEHICLE=ON \
  -DCH_ENABLE_MODULE_VEHICLE_MODELS=ON \
  -DCH_USE_CUDA_NVRTC=ON \
  -DOptiX_INSTALL_DIR=/path/to/NVIDIA-OptiX-SDK \
  -DCMAKE_BUILD_TYPE=Release
cmake --build . --target demo_SEN_metal_quarterpanel -j
```

(Exact OptiX/CUDA cmake variable names may differ with your Chrono version — configure OptiX the same way you
would for `demo_SEN_camera`, which must also build. If `demo_SEN_camera`/`demo_SEN_Gator` build and run on
OptiX on your machine, this demo will too — it uses the same API.)

The Metal backend is Apple-only and compiles out on your platform; ignore it. `ChCameraSensor` here routes to
OptiX automatically.

---

## 4. Run

```bash
cd build
./bin/demo_SEN_metal_quarterpanel        # supersample = 1  (matches our Metal reference)
./bin/demo_SEN_metal_quarterpanel 2      # supersample = 2
./bin/demo_SEN_metal_quarterpanel 3      # supersample = 3
```

Each run opens a live window and writes PNGs to `quarterpanel_out/optix/` (it runs ~3 s of settling and
exits). Grab a late frame (the car is settled by then).

> If the vehicle Audi data isn't found, set the data paths the way the other `demo_SEN_*` vehicle demos do on
> your setup (`SetChronoDataPath(...)` / `vehicle::SetDataPath(...)`), then rerun. No *external* (non-Chrono)
> data is used.

---

## 5. What to look at and report back

Please send back:

1. **A late frame from `quarterpanel_out/optix/` at supersample = 1** — the direct comparison to our
   `metal_quarterpanel_ss1.png`.
2. **Frames at supersample = 2 and 3.**
3. Your read on these specific questions:
   - On the **left rear quarter panel / sail panel**, does OptiX at **ss=1** show a **wavy/mottled dark
     reflection band** like the Metal reference, or a **clean, smooth glossy sheen**?
   - Does **ss=2 / ss=3** remove it on OptiX (as we'd expect if it's just reflection aliasing)?
   - Overall, is the OptiX paint **shinier / less shiny** than the Metal reference?
4. **The camera settings OptiX actually used** — in particular whether a **denoiser** was active. Chrono's
   `ChCameraSensor` defaults `use_denoiser = false`, and `ChOptixEngine` only attaches the denoiser when the
   camera requests it (`GetUseDenoiser()`), so it should be **off** here. If you want, also try a run with the
   denoiser on (construct the camera with `use_denoiser = true`) and include that frame — that tells us
   whether OptiX's clean look in other demos comes from the denoiser.

That's it — the images + those notes let us decide whether to (a) accept the reflection as expected OptiX
parity, or (b) add a glossy/blurred reflection or denoiser on the Metal side.

---

## 6. Optional: exact settings reference

The Metal camera for this scene is constructed as (see `demos_live/quarterpanel.cpp`):

```
ChCameraSensor(chassis, 30 Hz, cam_pose, 1280, 720, PI/3,
               ss /*=1*/, PINHOLE, /*use_gi=*/false, /*use_denoiser=*/false)
```

The OptiX twin (`demo_SEN_metal_quarterpanel.cpp`) constructs the same camera with `supersample_factor = ss`
and defaults (`LEGACY` integrator, `use_diffuse_reflect=false`, `use_denoiser=false`, `gamma=2.2`).
