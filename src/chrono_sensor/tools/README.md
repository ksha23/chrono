> **Looking for the test suite?** These scripts are the animation pipeline only. The automated
> sensor tests are CTest targets under `src/tests/unit_tests/sensor/` (run them with
> `ctest -L sensor`); see
> [`src/tests/unit_tests/sensor/README.md`](../../../src/tests/unit_tests/sensor/README.md) for
> what they cover and why bit-exact cross-backend comparison is not possible.

# Reproducing the showcase animations

Every `.webp` in [`../../../src/chrono_sensor/webp/`](../../../src/chrono_sensor/webp/) is produced by a two-step pipeline: a **simulation demo** writes raw
sensor frames, then a **Python post-processing script** turns that frame sequence into an animated WebP.

## 1. Build the demos

The Chrono libraries must already be built (e.g. `ninja Chrono_sensor` in your build dir), then:

```bash
cmake -S . -B build && ninja -C build      # demos build with the rest of Chrono
```

The showcase demos are ordinary Chrono demos (`src/demos/sensor/demo_SEN_showcase_*.cpp`) and land in
`build/bin`.

## 2. Run a demo

Demos are headless and finite: they render 150 frames (about 25 s each), write them under
`SENSOR_OUTPUT/SHOWCASE_<NAME>/`, and exit. **Run them from `build/bin`**: Chrono resolves its
data directory relative to the working directory, so launching from anywhere else silently drops
every texture and HDR map.

```bash
cd build/bin && ./demo_SEN_showcase_camera_rgb
```

## 3. Convert frames to an animated WebP

| Script | Purpose |
|---|---|
| `mkwebp.py` | Standard PNG sequence → animated WebP. `--seg` colorizes a segmentation sequence. |
| `mkcomposite.py` | Tiles the multi-sensor demo's four modalities into one 2×2 panel. |
| `mklidar.py` | Decodes dumped lidar range buffers into a bird's-eye point cloud, paired with the companion camera view. |
| `mkradar.py` | Decodes dumped radar returns into a top-down Doppler plot, paired with the companion camera view. |
| `segutil.py` | Shared helpers (frame loading, segmentation palette, depth contrast stretch). |

Paths below are relative to the repo root; `$OUT` is `build/bin/SENSOR_OUTPUT`. Every animation in
`webp/` is produced with the scripts' **default** settings (see below) plus the per-demo flags shown
here; nothing else is passed.

```bash
T=src/chrono_sensor/tools; W=src/chrono_sensor/webp; OUT=build/bin/SENSOR_OUTPUT

python3 $T/mkwebp.py      $OUT/SHOWCASE_CAMERA_RGB          $W/camera_rgb.webp
python3 $T/mkwebp.py      $OUT/SHOWCASE_CAMERA_DEPTH        $W/camera_depth.webp --synthetic
python3 $T/mkwebp.py      $OUT/SHOWCASE_CAMERA_NORMAL       $W/camera_normal.webp --synthetic
python3 $T/mkwebp.py      $OUT/SHOWCASE_CAMERA_SEGMENTATION $W/camera_segmentation.webp --seg
python3 $T/mkcomposite.py $OUT/SHOWCASE_MULTISENSOR         $W/multisensor.webp
python3 $T/mklidar.py     $OUT/SHOWCASE_LIDAR               $W/lidar.webp
python3 $T/mkradar.py     $OUT/SHOWCASE_RADAR               $W/radar.webp
```

The remaining nine (`arealights`, `envmap`, `fog`, `gi`, `lens_fisheye`, `lens_radial`,
`night_headlights`, `physcam_dof`, `physcam_grain`) are plain `mkwebp.py` runs with no flags, from
their matching `$OUT/SHOWCASE_<NAME>` directory.

Requires **Pillow** and **NumPy** (both already in the `chronopc` conda env). Pillow's WebP encoder is used
directly, so the `cwebp` and `img2webp` CLI tools are not needed.

### Encoding / file size

All scripts share `segutil.save_webp()`, which encodes with `method=6`, `minimize_size`, and `quality=58`.
Measured on a 120-frame 820 px clip that started at 4160 KB:

| change | size | note |
|---|---:|---|
| baseline (`quality=70, method=4`) | 4160 KB | |
| `method=6` + `minimize_size` | 3927 KB | only ~5% for ~14× the encode time |
| `quality=55` | 3219 KB | |
| 90 frames instead of 120 | 2578 KB | |
| 700 px wide instead of 820 | 2007 KB | |

**Frame count and pixel width are the real levers**; quality and encoder effort barely move the needle. The
defaults therefore decimate to 90 frames and downscale (700 px for single-view demos, 1000–1100 px for the
multi-panel ones, which need the extra width to stay legible). Override per run with `--width`, `--max`, and
`--quality`. Encoding is a one-off, but the bytes live in git history forever, hence the slow-but-smaller
settings.

### Ghosting on synthetic imagery

Animated WebP delta-codes each frame inside a bounding rectangle and only emits a keyframe every `kmax`
frames. On photographic content the residue hides in the texture, but on **synthetic imagery with hard edges
over large flat areas** (depth maps, label masks, point-cloud plots) the rectangle's stale contents stay
visible, so a moving object drags a visible outline behind it for up to `kmax` frames (over a second at
24 fps).

Pass `--synthetic` to force every frame to be a keyframe, removing inter-frame prediction entirely. On the
depth demo that cut mean error from **2.67 to 0.51** (5×) for +34% size, and removed a hard-edged rectangular
block that was clearly visible around the car. `--seg` implies **lossless** (a smeared class boundary is not
just ugly but semantically wrong); `--lossless` forces it for anything else. Lossless is exact but ~11× larger
on gradient content, so it is not the default.

Currently `--synthetic`: `camera_depth`, `camera_normal`, `multisensor`, `lidar`, `radar`.
Lossless: `camera_segmentation`.

## Notes

- Frames come out of `ChFilterSave` **upright**: the renderers produce bottom-up buffers
  (row 0 = bottom, matching OptiX's raygen) and `ChFilterSave` calls
  `stbi_flip_vertically_on_write(1)`. The scripts load PNGs as-is -- no flip.
- All *parked* demos share one canonical camera orbit (radius 6.0 m, height 1.8 m, looking at
  `(0, 0, 0.8)`, a full 360° over 150 frames) so their animations stay frame-synchronized with each other.
- Lidar and radar dump raw sensor buffers as `.bin`, which is why they have dedicated decode
  scripts: a point cloud has no natural image form, so the decode step is where the projection
  and the colour mapping are chosen.
