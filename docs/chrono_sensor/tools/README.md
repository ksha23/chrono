> **Looking for the test suite?** The scripts in this directory serve two purposes. The animation
> pipeline is documented below; the automated tests live alongside it:
>
> | script | purpose |
> |---|---|
> | `golden.py` | tier 0, golden-image regression against `../golden/` (`--bless` to update) |
> | `parity.py` + `PARITY.md` | tier 4, cross-backend Metal-vs-OptiX parity **report** (needs an NVIDIA GPU) |
>
> The tiers, why bit-exact cross-backend comparison is impossible, and how to add a test are all
> documented in [`src/tests/unit_tests/sensor/README.md`](../../../src/tests/unit_tests/sensor/README.md).

# Reproducing the showcase animations

Every `.webp` in [`../../../docs/chrono_sensor/webp/`](../../../docs/chrono_sensor/webp/) is produced by a two-step pipeline: a **simulation demo** writes raw
sensor frames, then a **Python post-processing script** turns that frame sequence into an animated WebP.

## 1. Build the demos

The Chrono libraries must already be built (e.g. `ninja Chrono_sensor` in your build dir), then:

```bash
cmake -S . -B build && ninja -C build      # demos build with the rest of Chrono
```

The showcase demos are ordinary Chrono demos (`src/demos/sensor/demo_SEN_showcase_*.cpp`) and land in
`build/bin`. Run them from there -- Chrono resolves its data directory relative to the working directory.
it was compiled against.

## 2. Run a demo

Demos are headless and finite — they render a fixed number of frames, write them under
`demos_live/showcase_out/<name>/`, and exit. Run them from the repo root:

```bash
./build/bin/showcase_camera_rgb
```

## 3. Convert frames to an animated WebP

| Script | Purpose |
|---|---|
| `mkwebp.py` | Standard PNG sequence → animated WebP. `--seg` colorizes a segmentation sequence. |
| `mkcomposite.py` | Tiles the multi-sensor demo's four modalities into one 2×2 panel. |
| `mklidar.py` | Decodes dumped lidar range buffers into a bird's-eye point cloud, paired with the companion camera view. |
| `mkradar.py` | Decodes dumped radar returns into a top-down Doppler plot, paired with the companion camera view. |
| `segutil.py` | Shared helpers (frame loading, segmentation palette, depth contrast stretch). |

```bash
python3 mkwebp.py     demos_live/showcase_out/camera_rgb  docs/chrono_sensor/webp/camera_rgb.webp --fps 24
python3 mkwebp.py     demos_live/showcase_out/camera_segmentation \
                      docs/chrono_sensor/webp/camera_segmentation.webp --fps 24 --seg
python3 mkcomposite.py demos_live/showcase_out/multisensor docs/chrono_sensor/webp/multisensor.webp --fps 24
python3 mklidar.py    demos_live/showcase_out/lidar       docs/chrono_sensor/webp/lidar.webp --fps 18
python3 mkradar.py    demos_live/showcase_out/radar       docs/chrono_sensor/webp/radar.webp --fps 18
```

Requires **Pillow** and **NumPy** (both already in the `chronopc` conda env). Pillow's WebP encoder is used
directly — the `cwebp` / `img2webp` CLI tools are not needed.

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
`--quality`. Encoding is a one-off, but the bytes live in git history forever — hence the slow-but-smaller
settings.

### Ghosting on synthetic imagery

Animated WebP delta-codes each frame inside a bounding rectangle and only emits a keyframe every `kmax`
frames. On photographic content the residue hides in the texture, but on **synthetic imagery with hard edges
over large flat areas** — depth maps, label masks, point-cloud plots — the rectangle's stale contents stay
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

- `ChFilterSave` writes frames **vertically flipped**; every script flips them upright.
- All *parked* demos share one canonical camera orbit (radius 6.0 m, height 1.8 m, looking at
  `(0, 0, 0.8)`, a full 360° over 150 frames) so their animations stay frame-synchronized with each other.
- Lidar/radar dump raw sensor buffers as `.bin` (their point-cloud export filters are CUDA-only and are not
  part of the Metal build), which is why they have dedicated decode scripts.
