#!/usr/bin/env python3
"""TIER 4 -- cross-backend PARITY REPORT (Metal RT vs OptiX).

    python3 src/tests/unit_tests/sensor/tools/parity.py METAL_DIR OPTIX_DIR [--out DIR]

This is a REPORT TO READ, NOT A PASS/FAIL GATE. It has no opinion about whether the two
backends "match"; it measures how far apart they are, channel by channel, and groups the
numbers by how close they have any right to be. Read the output, decide yourself.

Why it cannot be a gate
-----------------------
Two GPU ray tracers can never agree bit for bit. They draw from different RNG streams
(curand vs the PCG hash in ChMetalRTShaderMSL.h), they contract float expressions
differently, their transcendental functions differ in the last ulp, their BVH builders
make different watertightness and tie-breaking choices at triangle edges, their texture
samplers round and filter differently, and their denoisers are entirely different
algorithms. A per-pixel equality test would fail forever and tell you nothing.

So the outputs are partitioned by determinism, and only the first two groups are worth
looking at closely:

  GEOMETRY   depth, normals, segmentation ids, lidar and radar range. No stochastic
             shading is involved, so these should agree very tightly -- disagreement here
             means one backend's ray model or scene translation is actually wrong.
             This is the group to take seriously.
  LENS       FOV_LENS and RADIAL ray generation. Also pure geometry, but shaded, so
             expect shading-level noise on top of a geometrically identical image.
  SHADING    direct lighting with the denoiser OFF, no GI, no area lights, no depth of
             field, no sensor noise, supersample 1. Differences here are float ulps
             amplified through gamma and 8-bit quantisation. Small is expected; large is
             a lead, not a verdict.

  NEVER COMPARED: denoised output, global illumination, area-light soft shadows, depth
  of field, sensor noise. Those are stochastic or algorithmically different by
  construction. They are covered instead by the statistical tier
  (src/tests/unit_tests/sensor/utest_SEN_metal_stochastic.cpp), which asserts properties
  of the estimator rather than pixel values. verify_golden never enables any of them, so
  as long as both sides were produced by verify_golden this rule is enforced by
  construction rather than by convention.

Metrics
-------
  MAE / RMSE / PSNR   standard image-difference measures, in 8-bit units.
  SSIM                structural similarity (Gaussian 11x11, sigma 1.5), which cares
                      about structure rather than absolute level -- a uniform brightness
                      offset between backends barely moves SSIM but wrecks PSNR, and
                      knowing which of the two you have is the whole point.
  edge IoU            intersection-over-union of the gradient-magnitude masks, dilated by
                      one pixel. This isolates GEOMETRY from SHADING: if silhouettes land
                      in the same place, edge IoU stays near 1 no matter how different the
                      shading is. Reported as n/a for a frame with no real structure (a
                      heavy-fog frame, say), where the measure would be pure noise.
  class IoU           per-class intersection-over-union of the segmentation ids. These are
                      exact integers on both backends, so this is the single strictest
                      geometric agreement measure available and the first number to read.

Getting the two inputs
----------------------
See PARITY.md next to this file. In short: the same ``verify_golden`` program builds and
runs on both backends because it only uses the shared public sensor API, so you render it
once on the Mac and once on an NVIDIA box and point this script at the two output
directories. There is no NVIDIA GPU on the machine this was developed on, so the OptiX
half has never been executed here -- the script is written to be run by someone who has one.

Requires Pillow and NumPy.
"""

import argparse
import math
import os
import sys

import numpy as np
from PIL import Image, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))

# name -> (group, note). Mirrors the camera list in verify_golden.cpp.
CHANNELS = [
    ("depth",        "GEOMETRY", "ChDepthCamera, contrast-stretched to 8 bit"),
    ("normal",       "GEOMETRY", "ChNormalCamera, world-space normal packed to RGB"),
    ("segmentation", "GEOMETRY", "raw class / instance ids, exact integers"),
    ("fisheye",      "LENS",     "FOV_LENS ray generation"),
    ("radial",       "LENS",     "RADIAL lens distortion"),
    ("rgb",          "SHADING",  "pinhole, denoiser off, no GI, supersample 1"),
    ("rgb_ss2",      "SHADING",  "same, supersample 2"),
    ("fog",          "SHADING",  "per-camera fog"),
    ("env_rgb",      "SHADING",  "HDR environment map, supersample 1"),
    ("env_rgb_ss2",  "SHADING",  "HDR environment map, supersample 2"),
]
GROUP_ORDER = ["GEOMETRY", "LENS", "SHADING"]

# Rough expectations, printed alongside the numbers so the reader has a yardstick. These
# are guidance for a human, NOT thresholds the script enforces.
GUIDANCE = {
    "GEOMETRY": "expect class IoU > 0.99 and edge IoU > 0.95; anything lower is a real "
                "ray-model or scene-translation difference and is worth chasing",
    "LENS":     "expect edge IoU > 0.95 (the rays should land in the same place); PSNR will "
                "sit at SHADING levels because the image is still shaded",
    "SHADING":  "PSNR in the 30s-40s dB and SSIM > 0.95 is normal for two independent "
                "implementations of the same BRDF; treat a low number as a lead to "
                "investigate, never as a failure",
}


def load_frame(path, flip):
    im = Image.open(path).convert("RGBA")
    return ImageOps.flip(im) if flip else im


def find_frame(root, name):
    """Accept either a verify_golden output tree (<root>/<name>/frame_0.png, saved flipped)
    or a flat blessed-reference directory (<root>/<name>.png, already upright)."""
    nested = os.path.join(root, name, "frame_0.png")
    if os.path.exists(nested):
        return nested, True
    flat = os.path.join(root, name + ".png")
    if os.path.exists(flat):
        return flat, False
    return None, False


def gaussian_kernel(size=11, sigma=1.5):
    x = np.arange(size) - (size - 1) / 2.0
    k = np.exp(-(x ** 2) / (2 * sigma ** 2))
    return k / k.sum()


def sep_filter(img, k):
    """Separable convolution with edge replication; numpy only, no scipy."""
    pad = len(k) // 2
    a = np.pad(img, ((pad, pad), (pad, pad)), mode="edge")
    out = np.zeros_like(img, dtype=np.float64)
    for i, w in enumerate(k):
        out += w * a[i:i + img.shape[0], pad:pad + img.shape[1]]
    b = np.pad(out, ((0, 0), (pad, pad)), mode="edge")
    res = np.zeros_like(img, dtype=np.float64)
    for i, w in enumerate(k):
        res += w * b[:, i:i + img.shape[1]]
    return res


def ssim(a, b):
    """Global mean SSIM over the luminance channel, Wang et al. defaults."""
    k = gaussian_kernel()
    C1, C2 = (0.01 * 255) ** 2, (0.03 * 255) ** 2
    mu_a, mu_b = sep_filter(a, k), sep_filter(b, k)
    saa = sep_filter(a * a, k) - mu_a * mu_a
    sbb = sep_filter(b * b, k) - mu_b * mu_b
    sab = sep_filter(a * b, k) - mu_a * mu_b
    num = (2 * mu_a * mu_b + C1) * (2 * sab + C2)
    den = (mu_a ** 2 + mu_b ** 2 + C1) * (saa + sbb + C2)
    return float(np.mean(num / den))


def dilate1(mask):
    if mask is None:
        return None
    out = mask.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out |= np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    return out


def edge_mask(lum, rel_thresh=0.10, abs_floor=6.0):
    """Gradient-magnitude mask. The threshold is relative to the image's own peak gradient AND
    floored in absolute 8-bit units: without the floor, a nearly flat image (a heavy-fog frame,
    say) has its threshold scaled down onto its own quantisation noise, and the resulting mask
    is speckle rather than silhouette."""
    gx = np.zeros_like(lum)
    gy = np.zeros_like(lum)
    gx[:, 1:-1] = lum[:, 2:] - lum[:, :-2]
    gy[1:-1, :] = lum[2:, :] - lum[:-2, :]
    mag = np.hypot(gx, gy)
    peak = mag.max()
    if peak < abs_floor:
        return None  # no real structure to compare
    return mag > max(rel_thresh * peak, abs_floor)


def iou(m1, m2):
    if m1 is None or m2 is None:
        return None
    inter = np.count_nonzero(m1 & m2)
    union = np.count_nonzero(m1 | m2)
    return float(inter) / union if union else 1.0


def decode_class_ids(rgba):
    """The segmentation frame is PixelSemantic{uint16 class; uint16 instance} written raw as
    RGBA8, so class = R + 256*G on a little-endian host."""
    a = np.asarray(rgba, dtype=np.uint32)
    return a[:, :, 0] + (a[:, :, 1] << 8)


def compare(pa, fa, pb, fb, name):
    A = load_frame(pa, fa)
    B = load_frame(pb, fb)
    if A.size != B.size:
        return {"error": f"size {A.size} vs {B.size}"}
    a = np.asarray(A, dtype=np.float64)[:, :, :3]
    b = np.asarray(B, dtype=np.float64)[:, :, :3]
    d = np.abs(a - b)
    mae = float(d.mean())
    rmse = float(np.sqrt(((a - b) ** 2).mean()))
    psnr = float("inf") if rmse == 0 else 20.0 * math.log10(255.0 / rmse)
    la = a.mean(axis=2)
    lb = b.mean(axis=2)
    res = {
        "mae": mae,
        "rmse": rmse,
        "psnr": psnr,
        "ssim": ssim(la, lb),
        "max": float(d.max()),
        "edge_iou": iou(dilate1(edge_mask(la)), dilate1(edge_mask(lb))),
    }
    if name == "segmentation":
        ca, cb = decode_class_ids(A), decode_class_ids(B)
        classes = sorted(set(np.unique(ca)).union(np.unique(cb)))
        per = {int(c): iou(ca == c, cb == c) for c in classes}
        res["class_iou"] = per
        res["class_iou_mean"] = float(np.mean(list(per.values()))) if per else 1.0
        res["class_exact"] = float(np.count_nonzero(ca == cb)) / ca.size
    return res


def read_signature(path):
    vals = {}
    if not os.path.exists(path):
        return vals
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) == 2:
                vals[p[0]] = float(p[1])
    return vals


def write_visuals(pa, fa, pb, fb, dst_prefix):
    """Side-by-side plus an amplified difference map, matching the src/tests/unit_tests/sensor/tools
    convention of shipping a readable picture next to the numbers."""
    A = load_frame(pa, fa).convert("RGB")
    B = load_frame(pb, fb).convert("RGB")
    w, h = A.size
    sbs = Image.new("RGB", (w * 2 + 4, h), (20, 20, 20))
    sbs.paste(A, (0, 0))
    sbs.paste(B, (w + 4, 0))
    sbs.save(dst_prefix + "_side_by_side.png")
    d = np.abs(np.asarray(A, dtype=np.int16) - np.asarray(B, dtype=np.int16)).max(axis=2)
    Image.fromarray(np.clip(d * 8, 0, 255).astype(np.uint8), mode="L").save(dst_prefix + "_diff.png")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("metal_dir", help="verify_golden output (or blessed refs) from the Metal backend")
    ap.add_argument("optix_dir", help="verify_golden output from the OptiX backend")
    ap.add_argument("--out", default=None, help="write REPORT.md and the comparison images here")
    ap.add_argument("--sig-tol", type=float, default=1e-3,
                    help="relative difference above which a lidar/radar scalar is flagged (default 1e-3)")
    ap.add_argument("--gate-geometry", type=float, default=None, metavar="IOU",
                    help="OPT-IN only: exit non-zero if the segmentation class IoU falls below this. "
                         "Off by default -- this tool is a report, not a gate.")
    args = ap.parse_args()

    for d in (args.metal_dir, args.optix_dir):
        if not os.path.isdir(d):
            raise SystemExit(f"parity: {d} is not a directory")
    if args.out:
        os.makedirs(args.out, exist_ok=True)

    lines = []

    def emit(s=""):
        print(s)
        lines.append(s)

    emit("# Metal RT vs OptiX -- parity report")
    emit()
    emit(f"- Metal frames: `{args.metal_dir}`")
    emit(f"- OptiX frames: `{args.optix_dir}`")
    emit()
    emit("This is a report to read, not a pass/fail gate. Two GPU ray tracers cannot agree")
    emit("bit for bit -- different RNG streams, float contraction, BVH tie-breaking and texture")
    emit("samplers guarantee divergence. Denoised, GI, area-light, depth-of-field and sensor-noise")
    emit("output is deliberately absent: none of it is comparable, and all of it is covered")
    emit("instead by the statistical tier (`utest_SEN_metal_stochastic`).")
    emit()

    results = {}
    missing = []
    for name, group, note in CHANNELS:
        pa, fa = find_frame(args.metal_dir, name)
        pb, fb = find_frame(args.optix_dir, name)
        if not pa or not pb:
            missing.append(name)
            continue
        r = compare(pa, fa, pb, fb, name)
        r["group"], r["note"] = group, note
        results[name] = r
        if args.out and "error" not in r:
            write_visuals(pa, fa, pb, fb, os.path.join(args.out, name))

    for group in GROUP_ORDER:
        rows = [(n, results[n]) for n, g, _ in CHANNELS if n in results and results[n]["group"] == group]
        if not rows:
            continue
        emit(f"## {group}")
        emit()
        emit(f"_{GUIDANCE[group]}_")
        emit()
        emit("| channel | MAE | RMSE | PSNR (dB) | SSIM | max | edge IoU | note |")
        emit("|---|---:|---:|---:|---:|---:|---:|---|")
        for name, r in rows:
            if "error" in r:
                emit(f"| {name} | - | - | - | - | - | - | **{r['error']}** |")
                continue
            psnr = "inf" if math.isinf(r["psnr"]) else f"{r['psnr']:.2f}"
            eio = "n/a" if r["edge_iou"] is None else f"{r['edge_iou']:.4f}"
            emit(f"| {name} | {r['mae']:.3f} | {r['rmse']:.3f} | {psnr} | {r['ssim']:.4f} | "
                 f"{r['max']:.0f} | {eio} | {r['note']} |")
        emit()

    seg = results.get("segmentation")
    if seg and "class_iou" in seg:
        emit("### Segmentation class IoU (the strictest geometric measure)")
        emit()
        emit("Class ids are exact integers on both backends, so this compares scene translation and")
        emit("primary-ray geometry with no shading in the way at all.")
        emit()
        emit("| class id | IoU |")
        emit("|---:|---:|")
        for c, v in sorted(seg["class_iou"].items()):
            emit(f"| {c} | {v:.4f} |")
        emit()
        emit(f"- mean class IoU: **{seg['class_iou_mean']:.4f}**")
        emit(f"- pixels with an identical class id: **{100.0 * seg['class_exact']:.2f}%**")
        emit()

    sa = read_signature(os.path.join(args.metal_dir, "signature.txt"))
    sb = read_signature(os.path.join(args.optix_dir, "signature.txt"))
    if sa and sb:
        emit("## GEOMETRY -- lidar / radar scalars")
        emit()
        emit("Beam ranges carry no shading whatsoever, so these should agree to several decimal")
        emit("places. A relative difference above the tolerance is flagged.")
        emit()
        emit("| statistic | Metal | OptiX | relative diff | |")
        emit("|---|---:|---:|---:|---|")
        for k in sorted(set(sa) | set(sb)):
            if k not in sa or k not in sb:
                emit(f"| {k} | {sa.get(k, float('nan')):.5f} | {sb.get(k, float('nan')):.5f} | - | missing |")
                continue
            rel = abs(sb[k] - sa[k]) / max(abs(sa[k]), 1e-6)
            flag = "" if rel <= args.sig_tol else "**differs**"
            emit(f"| {k} | {sa[k]:.5f} | {sb[k]:.5f} | {rel:.2e} | {flag} |")
        emit()
    else:
        emit("_No signature.txt on one or both sides, so the lidar/radar scalars were not compared._")
        emit()

    if missing:
        emit(f"_Channels absent from one or both directories, and therefore skipped: {', '.join(missing)}._")
        emit()

    emit("## How to read this")
    emit()
    emit("1. Start with the segmentation class IoU and the lidar/radar scalars. They are pure")
    emit("   geometry; if they disagree, one backend's scene translation or ray model is wrong,")
    emit("   and every shading number below is meaningless until that is fixed.")
    emit("2. Then look at edge IoU across the board. High edge IoU with low PSNR means the two")
    emit("   backends agree about WHERE everything is and disagree about how bright it is --")
    emit("   a shading-model difference, not a geometry bug.")
    emit("3. Only then look at PSNR/SSIM, and treat them as a lead rather than a verdict.")
    emit("4. If a channel looks wrong, open `<channel>_side_by_side.png` and `<channel>_diff.png`")
    emit("   (written when --out is given; the diff is amplified 8x so a 1-2 LSB change is visible).")

    if args.out:
        with open(os.path.join(args.out, "REPORT.md"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print(f"\nparity: wrote {os.path.join(args.out, 'REPORT.md')}")

    if args.gate_geometry is not None and seg and "class_iou_mean" in seg:
        if seg["class_iou_mean"] < args.gate_geometry:
            print(f"parity: OPT-IN GATE failed -- mean class IoU {seg['class_iou_mean']:.4f} "
                  f"< {args.gate_geometry}")
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
