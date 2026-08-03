# Cross-backend parity: Metal RT vs OptiX

**Audience:** someone with an NVIDIA GPU (CUDA + OptiX) who can build and run Chrono::Sensor with the
**OptiX** backend. **Goal:** produce a numeric report of how far the Metal RT backend sits from the
OptiX reference on the subset of outputs where a comparison is meaningful at all.

**This produces a report to read, not a pass/fail gate.** See "Why this is not a gate" below before
using any number here to accept or reject a change.

The machine this tooling was written on has no NVIDIA GPU, so **the OptiX half has never been
executed**. Step 2 is the part that needs you.

---

## 1. What gets compared, and what deliberately does not

Two GPU ray tracers cannot agree bit for bit. They draw from different RNG streams (curand vs the
PCG hash in `ChMetalRTShaderMSL.h`), contract float expressions differently, disagree in the last
ulp of every transcendental, make different watertightness and tie-breaking choices at triangle
edges in the BVH, filter textures differently, and ship completely different denoisers. So the
outputs are partitioned by how deterministic they are:

| group | channels | why it is comparable | what to expect |
|---|---|---|---|
| **GEOMETRY** | depth, normals, segmentation class/instance ids, lidar range, radar range | pure ray/scene intersection, no stochastic shading anywhere | very tight. Class IoU > 0.99, lidar/radar scalars equal to several decimals. **Disagreement here is a real bug in one backend.** |
| **LENS** | FOV_LENS, RADIAL | ray *generation* is pure geometry; the image is still shaded | silhouettes in the same place (edge IoU > 0.95), pixel values at SHADING level |
| **SHADING** | pinhole RGB (ss 1 and 2), fog, HDR environment map | direct lighting only, **denoiser off**, no GI, no area lights, no depth of field, no sensor noise | float ulps amplified through gamma and 8-bit quantisation. PSNR in the 30s–40s dB, SSIM > 0.95 |

**Never compared:** denoised output, global illumination, area-light soft shadows, depth of field,
sensor noise. Denoisers are different algorithms, and the rest are Monte Carlo estimators whose
per-pixel values are a function of the RNG stream. Comparing them produces a number that moves when
nothing is wrong and stays put when something is. Those features are covered instead by the
statistical tier, `src/tests/unit_tests/sensor/utest_SEN_metal_stochastic.cpp`, which asserts
properties of the *estimator* (variance falls as samples rise, the mean does not move, an area light
casts a wider penumbra than a point light) rather than pixel values.

`verify_golden` never turns any of those features on, so as long as both sides come from
`verify_golden` this rule is enforced by construction and not merely by convention.

---

## 2. Producing the OptiX side

`docs/showcase/demos/verify_golden.cpp` is written entirely against the **shared public sensor API**
— `ChSensorManager`, `ChCameraSensor`, `ChDepthCamera`, `ChNormalCamera`, `ChSegmentationCamera`,
`ChLidarSensor`, `ChRadarSensor`, `ChFilterSave`, and the `ChScene` methods that
`ChMetalRTScene` mirrors exactly (commit `5a2542b49`). It therefore compiles and runs unchanged on an
OptiX build. Nothing Metal-specific is referenced, and no external data is used: the scene is built
from primitives plus two files that ship with Chrono
(`data/sensor/textures/checkerboard.png` and `data/sensor/textures/sky_2_4k.hdr`).

Build Chrono with the sensor module and OptiX enabled the same way you would for `demo_SEN_camera`
(see `docs/optix_compare/OPTIX_COMPARISON.md` §3 for a worked cmake invocation), then:

```bash
# from the repo root, with the Chrono libs already built in $CHRONO_BUILD
CHRONO_BUILD=/path/to/build bash docs/showcase/demos/build.sh verify_golden

mkdir -p /tmp/optix_golden
./docs/showcase/demos/bin/verify_golden /tmp/optix_golden base
./docs/showcase/demos/bin/verify_golden /tmp/optix_golden env
```

That writes ten PNGs (`<name>/frame_0.png`) plus `signature.txt`. Copy the whole directory back.

> `build.sh` is a plain `c++` invocation against an existing build tree; it honours `CHRONO_ROOT`,
> `CHRONO_BUILD`, `OUT_DIR` and `EIGEN_INCLUDE`. If your Chrono is configured differently, compiling
> `verify_golden.cpp` as an ordinary Chrono demo works just as well — it has no special requirements.

**Sanity check before you copy anything back:** every camera must report exactly one launch, and the
program exits non-zero if not. If `signature.txt` is missing or a `frame_0.png` is absent, the run
did not complete and the comparison would be meaningless.

---

## 3. Producing the Metal side

Either render it fresh:

```bash
mkdir -p /tmp/metal_golden
./docs/showcase/demos/bin/verify_golden /tmp/metal_golden base
./docs/showcase/demos/bin/verify_golden /tmp/metal_golden env
```

or just use the blessed references already committed at `docs/showcase/golden/`, which are the same
frames stored upright. `parity.py` accepts either layout and flips as needed.

---

## 4. Running the comparison

```bash
python3 docs/showcase/tools/parity.py docs/showcase/golden /tmp/optix_golden --out /tmp/parity_report
```

This prints a Markdown report and writes it to `/tmp/parity_report/REPORT.md`, along with
`<channel>_side_by_side.png` and `<channel>_diff.png` for each channel (the diff is amplified 8x so a
1–2 LSB change is actually visible). Requires Pillow and NumPy; both are in the `chronopc` conda env.

Metrics per channel:

- **MAE / RMSE / PSNR** — standard image differences, in 8-bit units.
- **SSIM** — Gaussian 11×11, σ = 1.5. SSIM cares about structure rather than level, so a uniform
  brightness offset between backends barely moves SSIM while wrecking PSNR. Knowing which of the two
  you are looking at is most of the diagnosis.
- **edge IoU** — intersection-over-union of the gradient-magnitude masks, dilated by one pixel to
  absorb half-pixel sample-placement differences. This separates *where things are* from *how bright
  they are*: high edge IoU with low PSNR means the geometry agrees and the shading does not.
  Reported as `n/a` on a frame with no real structure, where the measure would be pure noise.
- **class IoU** — per-class IoU of the decoded segmentation ids. Exact integers on both backends, so
  this is the strictest geometric measure available and the first number to read.
- **lidar/radar scalars** — return count, min, max and mean of range and intensity/amplitude, from
  `signature.txt`.

---

## 5. How to read the result

1. **Segmentation class IoU and the lidar/radar scalars first.** These are pure geometry. If they
   disagree, one backend's scene translation or ray model is wrong and every shading number below is
   meaningless until that is fixed.
2. **Then edge IoU across the board.** High edge IoU with low PSNR means the backends agree about
   where everything is and disagree about how bright it is — a shading-model difference, not a
   geometry bug.
3. **Then PSNR/SSIM**, as a lead rather than a verdict.
4. If a channel looks wrong, open its `_side_by_side.png` and `_diff.png`. A structured diff (one
   object, one silhouette, one material) is a specific bug; uniform low-level speckle is float noise.

### Why this is not a gate

Any threshold on the SHADING group would either be so loose it catches nothing or so tight it fires
on a driver update. The GEOMETRY group *is* tight enough to gate on, and `parity.py` has an opt-in
`--gate-geometry IOU` flag for exactly that — but it is off by default, and even then it only gates
the segmentation class IoU. Day-to-day regression protection comes from the tiers that do not need a
second GPU at all:

- `docs/showcase/tools/golden.py` — pixel-exact self-comparison on one machine (tier 0)
- `docs/showcase/demos/verify_render_math.cpp` — analytic ground truth, no second renderer involved
  (tier 1)
- `src/tests/unit_tests/sensor/utest_SEN_metal_stochastic.cpp` — statistical properties (tier 2)

See `src/tests/unit_tests/sensor/README.md` for the whole picture.

### Known divergences you will see, and should not chase

`verify_render_math` already identifies these; they are backend semantics differences, not noise.

| divergence | effect on this report |
|---|---|
| Metal ignores `ChDepthCamera` / `ChNormalCamera` / `ChSegmentationCamera` `GetHFOV()` and renders those three at its hard-coded 1.408 rad fallback | **the depth, normal and segmentation frames will not line up at all** until this is fixed — different field of view, so class IoU and edge IoU collapse. Fix this before reading the GEOMETRY group. |
| Metal writes `0.0` for a sky/miss depth ray; OptiX writes `max_depth` | inverts the sky region of the depth frame after `ChFilterDepthToRGBA8`'s contrast stretch |
| Metal does not clamp depth to `maxDepth`; OptiX does | changes the depth frame's contrast-stretch range, so the whole frame shifts |

---

## 6. Relationship to `docs/optix_compare/`

`docs/optix_compare/` is the earlier, targeted version of this: one scene, one artifact (the car-paint
reflection of issue #31), rendered on an OptiX box and reported by eye. This directory generalises it
— same idea of shipping the exact scene to the other machine and getting images back, but across ten
channels with numbers attached instead of one channel judged visually. The conventions are shared:
the OptiX-side instructions live with the tool, the produced images and a `REPORT.md` land together in
one output directory, and the report says plainly what it can and cannot conclude.
