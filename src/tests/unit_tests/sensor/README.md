# Testing Chrono::Sensor's Metal RT backend

```bash
source /opt/homebrew/Caskroom/miniconda/base/etc/profile.d/conda.sh && conda activate chronopc
cmake -S . -B build && ninja -C build && ctest --test-dir build -L sensor
```

That is the whole suite. It takes about five seconds once things are built, prints a per-tier
pass/fail summary, and exits non-zero on any failure. Everything below is the reasoning behind it.

---

## Why there is no "just diff it against OptiX" test

The obvious way to test a new ray-tracing backend is to render the same scene on the reference
backend and compare. It does not work, and it is worth being precise about why, because the shape of
this entire suite follows from it.

Two GPU ray tracers cannot agree bit for bit:

- **Different RNG streams.** OptiX seeds curand per pixel; the Metal shader uses its own PCG hash
  (`ChMetalRTShaderMSL.h`). Every Monte Carlo estimator — supersampling jitter, GI bounce directions,
  area-light shadow rays, lens sampling — therefore takes a different path through sample space.
  Same expected value, different realisation, per pixel, forever.
- **Different float behaviour.** FMA contraction, instruction selection and the last ulp of every
  `sin`, `cos`, `pow` and `rsqrt` differ between NVCC/PTX on an NVIDIA SM and the Metal compiler on
  Apple silicon. A ray direction that differs in the last bit can land on the other side of a
  triangle edge.
- **Different BVH behaviour.** Watertightness guarantees and tie-breaking at shared edges are
  implementation choices. Silhouette pixels legitimately resolve differently.
- **Different texture samplers.** Filtering, rounding and mip selection differ.
- **Completely different denoisers.** OptiX ships an AI denoiser; there is no such thing on the Metal
  side. Comparing denoised output compares two unrelated algorithms.

So "differs from OptiX" carries almost no information. A test built on it fails when nothing is wrong
and passes when something is.

**The way out is to test against analytic ground truth instead.** For the geometric channels, the
correct answer is computable in closed form from the scene geometry and the documented ray model. An
assertion derived that way is backend-independent: it is equally valid on Metal, OptiX and Vulkan,
and a failure means the backend is *wrong*, not merely *different*. That is tier 1, and it is the
strongest thing in this suite.

---

## The tiers

Outputs are partitioned by how deterministic they are, and each partition gets the strongest test
that is actually valid for it.

| tier | what | where | asserts |
|---|---|---|---|
| **0** | golden images | `docs/chrono_sensor/tools/golden.py`, `src/demos/sensor/verify_golden.cpp` | pixel-exact self-comparison on one machine |
| **1** | analytic ground truth | `src/demos/sensor/verify_render_math.cpp` | closed-form geometry, no renderer involved |
| **2** | statistical / convergence | `utest_SEN_metal_stochastic.cpp` | properties of the estimator, not pixel values |
| **3** | backend-agnostic unit tests | `utest_SEN_{gps,data_access,interface,threadsafety,radar}.cpp` | public sensor API behaviour |
| **4** | cross-backend parity | `docs/chrono_sensor/tools/parity.py`, `PARITY.md` | a report to read; needs an NVIDIA GPU |
| — | dynamic sensors | `src/demos/sensor/verify_dynamic_sensors.cpp` | IMU / GPS / magnetometer / tachometer liveness |

### Tier 0 — golden images (the cheapest useful net)

`verify_golden` renders a small deterministic scene — primitives and shipped data only, no vehicle
model, well under a second — across ten modalities: pinhole colour with shadows, a textured ground
plane, glossy and semi-transparent surfaces, the supersampled path, `FOV_LENS`, `RADIAL` distortion,
fog, depth, normals, segmentation, and two HDR environment-map frames. It also emits sixteen scalar
statistics for lidar and radar, which have no image form; a few hundred bytes instead of a binary
blob, and still enough to catch a regression in the beam models.

`golden.py` renders and pixel-diffs against blessed references, reporting max and mean per-channel
difference, how many pixels moved, and whether the run was bit-identical.

This works **only** because the renderer is genuinely deterministic for the non-stochastic feature
set: the Metal shader seeds its RNG from the pixel index rather than from time, so a static scene
renders identically on every frame and in every process. That premise is not assumed — it is asserted
by `MetalStochastic.deterministic_when_all_stochastic_features_are_off`, which currently reports zero
differing pixels across three frames, and confirmed in practice by all ten golden images coming back
bit-identical across separate processes.

The default tolerance is nevertheless 2 LSB rather than 0, because the same source can legitimately
differ by a hair across GPU models, macOS releases and compiler versions. Whether a run was
bit-identical or merely within tolerance is reported per image, so you never have to guess.

**Reference budget: ten 160×120 PNGs plus a manifest, about 100 KB.** Keep it that way — these bytes
live in git history forever. If you need more coverage, prefer another small camera over a bigger
one, and never add multi-frame sequences.

### Tier 1 — analytic ground truth (the strongest tier)

`verify_render_math` builds a scene of four axis-aligned boxes and then re-derives, in
double precision with an independent ray/AABB intersector, exactly what every sensor must report:

- **depth** over the whole frame, on two different sensor poses, plus closed-form spot checks
  (`face / cos θ`, the `1/cos` law off-axis, a floor-plane hit);
- **normals** over the whole frame, exact on axis-aligned faces;
- **segmentation** class *and* instance ids, exact integer equality, plus sky sentinel;
- **projection**: that the pinhole projection of a known cube's centre lands on that cube, and that
  its rendered silhouette's centroid and area match the analytic silhouette;
- **lidar** range and `|N·V|` intensity at known azimuth/elevation on two poses, plus `max_distance`
  suppression;
- **radar** range, reported azimuth/elevation, and zero Doppler in a static scene.

Silhouette and face-crease pixels are excluded from the whole-frame comparisons: sub-pixel sample
placement is a legitimate implementation choice, so those pixels carry no backend-independent
expectation. Everything else is compared, and currently matches to better than 7 × 10⁻⁵ m (normals
exactly).

Assertions come in two flavours. **FAIL** means the backend violates the contract and is always
fatal. **GAP** means a divergence from the OptiX reference semantics that has already been
diagnosed, with the exact source lines on both sides quoted at the call site; gaps are printed
loudly, counted separately, and are fatal only under `--strict`, so a pre-existing bug does not make
the day-to-day run permanently red.

### Tier 2 — statistical / convergence

Everything stochastic lives here: GI, area-light soft shadows, depth of field, sensor noise. No
per-pixel value is asserted. What *is* asserted is a property any correct implementation must have
regardless of its RNG stream:

- more samples must reduce Monte Carlo variance, and must **not** move the mean (an estimator may be
  noisy; it may not be biased) — currently a variance ratio of 0.2499 against the ideal 0.25 for 16
  samples, with a mean shift of 0.05 LSB against an allowed 1.93;
- a finite-extent area light must produce a **wider** shadow transition than a point light in the
  same place — that is similar triangles, not sampling — currently 41 px vs 0 px;
- a finite aperture must soften an out-of-focus silhouette, and its residual sampling noise must fall
  as samples are added — currently 6 px vs 0 px, and a roughness of 9.07 → 2.96;
- additive sensor noise must scale with the requested sigma and leave the mean where it was —
  currently within 1.2% of the derived σ·255/√3.

This follows the pattern set by `utest_SEN_camera_convergence.cpp`, which stays OptiX-gated because
it needs a `ChFilterRGBA16Access` buffer and `Integrator::PATH`, neither of which the Metal path
produces. Writing a separate Metal suite was preferred to weakening that one.

**Tolerances here are derived, not chosen.** The mean-shift bound is four standard errors of the
difference of two sample means, so it rescales automatically with brightness, bit depth, resolution
and sample count; the sensor-noise band predicts σ·255/√3 (three independent channels averaged) and
allows 0.8×–1.25× of it. A fixed absolute or relative tolerance cannot do that — one loose enough for
a bright scene is meaningless on a dark one.

> Two of these tests initially failed, and both times the *measurement* was wrong, not the backend.
> The GI mean appeared to shift by 26 LSB until it was read from a linear rather than a gamma-encoded
> buffer (`pow(x, 1/2.2)` is concave, so by Jensen's inequality a noisy estimate encodes darker than a
> converged one even when the estimator is unbiased). Depth of field appeared to *sharpen* the edge
> until the metric stopped being a gradient magnitude, which lens speckle dominates. If a statistical
> test fails, suspect the statistic first.

### Tier 3 — backend-agnostic unit tests

Historically only `utest_SEN_gps` was unconditional and everything else was gated behind
`CH_USE_SENSOR_OPTIX`, so a Metal build ran almost no sensor tests. Four more depend on nothing but
the public sensor API and are now unconditional: `utest_SEN_data_access`, `utest_SEN_interface`,
`utest_SEN_threadsafety`, `utest_SEN_radar`.

`utest_SEN_optix{engine,geometry,pipeline}` remain gated — they drive OptiX pipeline internals — and
so does `utest_SEN_camera_convergence`, for the reasons above.

The runner enumerates each GoogleTest binary and runs **every case in its own process**. That costs a
few hundred milliseconds and buys crash isolation: one segfaulting case cannot hide the results of
everything scheduled after it. There is currently exactly one such case, which is why it matters.

### Tier 4 — cross-backend parity (report, not gate)

`parity.py` compares two `verify_golden` output trees and emits a Markdown report with MAE, RMSE,
PSNR, SSIM, edge IoU and per-class segmentation IoU, plus the lidar/radar scalars — grouped by how
close each channel has any right to be. Denoised, GI, area-light, depth-of-field and sensor-noise
output is never compared; since `verify_golden` never enables any of it, that rule is enforced by
construction rather than by convention.

It is explicitly a report to read, not a pass/fail gate. The full procedure for someone with an
NVIDIA GPU is in `docs/chrono_sensor/tools/PARITY.md`. **The OptiX half has never been executed** — there
is no NVIDIA GPU on the development machine. The script itself is exercised on two Metal renders,
identical and perturbed.

---

## Running things

```bash
# everything
cmake -S . -B build && ninja -C build && ctest --test-dir build -L sensor

# one tier, with each test's own output
ctest --test-dir build -R verify_render_math --output-on-failure

# what would run
ctest --test-dir build -L sensor -N

# exit 0 while the already-diagnosed backend bugs are still open
ctest --test-dir build -L sensor --output-on-failure
```

Individual pieces:

```bash
ninja -C build Chrono_sensor utest_SEN_metal_stochastic       # library + tier 2
ninja -C build verify_render_math                          # one program
./build/bin/verify_render_math --strict         # tier 1, gaps fatal
./build/bin/utest_SEN_metal_stochastic                        # tier 2
python3 docs/chrono_sensor/tools/golden.py                         # tier 0
```

Unit tests need `BUILD_TESTING=ON` and the googletest submodule:

```bash
git submodule update --init --depth 1 src/chrono_thirdparty/googletest
cmake -S . -B build -DBUILD_TESTING=ON
```

### Known failures

Known-bug context for individual checks is documented in the test sources themselves.
Failures are reported in two separate blocks — *already-diagnosed* and *new* — so a regression is
never buried under expected red. **Delete an entry the moment its bug is fixed**, otherwise the
suite will quietly stop noticing when it comes back.

### Blessing new references

When a rendering change is intentional:

```bash
bash src/demos/sensor/build.sh verify_golden       # rebuild against the new library
python3 docs/chrono_sensor/tools/golden.py                 # LOOK at what changed first
python3 docs/chrono_sensor/tools/golden.py --keep-out /tmp/g   # inspect diff_*.png if unsure
python3 docs/chrono_sensor/tools/golden.py --bless         # then, and only then, re-bless
```

Re-blessing is how a rendering bug becomes permanent, so treat a diff as guilty until proven
innocent. `golden.py` writes an 8×-amplified `diff_<channel>.png` on every failure precisely so that
"what actually changed" is a picture rather than a number. Commit the updated `docs/chrono_sensor/golden/`
in the same commit as the change that caused it, and say in the message why the pixels moved.

---

## Adding a test — which tier?

1. **Is the correct answer computable from geometry and the documented ray model?** Then it belongs
   in tier 1, asserted in closed form. This is always the best option; reach for it first.
2. **Is the output deterministic but not analytically predictable** (a shaded image)? Tier 0.
   Keep the reference small.
3. **Does it involve randomness** — GI, area lights, depth of field, sensor noise? Tier 2, asserting
   a property of the estimator. Derive the tolerance from the sampling statistics rather than
   picking a number that happens to pass.
4. **Is it about API behaviour rather than pixels** — filter graphs, buffer lifetimes, scene
   bookkeeping? Tier 3, as a GoogleTest case, unconditional unless it genuinely needs OptiX.
5. **Does it need a second backend?** Tier 4, as a reported number, never as a gate.

And the rule that outranks all of them: **never widen a tolerance to get green.** If a test fails,
either the backend is wrong (report it, add it to `KNOWN`) or the measurement is wrong (fix the
measurement, and write down why it was wrong). A tolerance loosened to silence a failure destroys the
only thing the test was for.
