#!/usr/bin/env python3
"""TIER 0 -- golden-image regression harness for the Chrono::Sensor Metal RT backend.

Renders the small deterministic scene in ``docs/showcase/demos/verify_golden.cpp`` and
pixel-diffs the result against blessed reference frames, failing on any regression.

Why this is the cheapest useful net
-----------------------------------
The renderer is bit-exact frame to frame for the deterministic feature subset -- no GI,
no area lights, no depth of field, no sensor noise, no denoiser -- because the Metal
shader seeds its RNG from the pixel index rather than from time. That property is
asserted directly by ``utest_SEN_metal_stochastic`` and it is what makes a pixel diff
meaningful at all. On one machine the expected difference is therefore exactly zero, and
any non-zero diff is a change in behaviour worth looking at.

The default tolerance is nevertheless a couple of LSB rather than zero, because the same
source can legitimately differ by a hair across GPU models, macOS releases and compiler
versions (different FMA contraction, different transcendental implementations). Whether a
run was bit-identical or merely within tolerance is reported per image, so you never have
to guess which of the two you got.

What is NOT here
----------------
Nothing stochastic, and nothing denoised. Those belong to tier 2
(``utest_SEN_metal_stochastic``), which asserts statistical properties instead of pixels.
Cross-backend comparison belongs to ``parity.py``, which is a report, not a gate.

Reference budget
----------------
Ten 160x120 PNGs plus a manifest: about 140 KB total. Keep it that way -- these bytes
live in git history forever. If you need more coverage, prefer another small camera over
a bigger one, and never add multi-frame sequences here.

Usage
-----
    python3 docs/showcase/tools/golden.py                 # check against references
    python3 docs/showcase/tools/golden.py --bless         # update the references
    python3 docs/showcase/tools/golden.py --keep-out DIR  # keep the rendered frames

Exit code is 0 only if every image and every scalar signature is within tolerance.
Requires Pillow and NumPy (both already in the ``chronopc`` conda env).
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageOps

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_BIN = os.path.join(REPO, "docs", "showcase", "demos", "bin", "verify_golden")
DEFAULT_REFS = os.path.join(REPO, "docs", "showcase", "golden")

# Rendered by verify_golden; "base" uses a procedural sky, "env" the shipped HDR map.
MODES = ("base", "env")
IMAGES = [
    "rgb",           # pinhole colour: shadows, textured ground, glossy + transparent surfaces
    "rgb_ss2",       # the supersampled (antialiasing) path
    "fisheye",       # FOV_LENS ray generation
    "radial",        # RADIAL lens distortion
    "fog",           # per-camera fog
    "depth",         # ChDepthCamera -> ChFilterDepthToRGBA8
    "normal",        # ChNormalCamera -> ChFilterNormalToRGBA8
    "segmentation",  # raw class/instance ids
    "env_rgb",       # HDR environment map: texture sampler + image-based reflections
    "env_rgb_ss2",
]


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def load_upright(path):
    """Load a freshly rendered frame. ChFilterSave writes frames vertically flipped, so flip
    them back: references are stored upright, so a human opening one sees what the camera saw."""
    return ImageOps.flip(Image.open(path).convert("RGBA"))


def load_ref(path):
    """Load a stored reference. Already upright -- do NOT flip it again."""
    return Image.open(path).convert("RGBA")


def render(binary, out_dir, env=None):
    os.makedirs(out_dir, exist_ok=True)
    for mode in MODES:
        proc = subprocess.run([binary, out_dir, mode], cwd=REPO, capture_output=True, text=True, env=env)
        if proc.returncode != 0:
            sys.stderr.write(proc.stdout + proc.stderr)
            raise SystemExit(f"golden: verify_golden {mode} failed with code {proc.returncode}")


def read_signature(path):
    vals = {}
    if not os.path.exists(path):
        return vals
    with open(path) as f:
        for line in f:
            parts = line.split()
            if len(parts) == 2:
                vals[parts[0]] = float(parts[1])
    return vals


def compare_images(ref_png, new_png, max_tol, mean_tol):
    """Per-channel absolute difference between two RGBA frames."""
    a = np.asarray(load_ref(ref_png), dtype=np.int16)
    b = np.asarray(load_upright(new_png), dtype=np.int16)
    if a.shape != b.shape:
        return {"ok": False, "note": f"size {b.shape[1]}x{b.shape[0]} vs reference {a.shape[1]}x{a.shape[0]}"}
    d = np.abs(a - b)
    px_changed = int(np.count_nonzero(d.max(axis=2)))
    res = {
        "max": int(d.max()),
        "mean": float(d.mean()),
        "px_changed": px_changed,
        "px_total": int(a.shape[0] * a.shape[1]),
        "identical": bool(d.max() == 0),
    }
    res["ok"] = res["max"] <= max_tol and res["mean"] <= mean_tol
    return res


def compare_signature(ref, new, rel_tol):
    rows = []
    for key in sorted(set(ref) | set(new)):
        if key not in ref:
            rows.append((key, None, new[key], False, "not in reference"))
            continue
        if key not in new:
            rows.append((key, ref[key], None, False, "missing from run"))
            continue
        r, n = ref[key], new[key]
        scale = max(abs(r), 1e-6)
        rel = abs(n - r) / scale
        rows.append((key, r, n, rel <= rel_tol, f"rel {rel:.2e}"))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", default=DEFAULT_BIN, help="path to the verify_golden binary")
    ap.add_argument("--refs", default=DEFAULT_REFS, help="directory holding the blessed references")
    ap.add_argument("--bless", action="store_true", help="overwrite the references with this run")
    ap.add_argument("--keep-out", default=None, help="write the rendered frames here instead of a temp dir")
    ap.add_argument("--max-diff", type=int, default=2, help="allowed max per-channel difference (LSB, default 2)")
    ap.add_argument("--mean-diff", type=float, default=0.05,
                    help="allowed mean per-channel difference (LSB, default 0.05)")
    ap.add_argument("--sig-tol", type=float, default=1e-3,
                    help="allowed relative difference for the lidar/radar scalars (default 1e-3)")
    ap.add_argument("--quiet", action="store_true", help="only print failures and the summary")
    args = ap.parse_args()

    if not os.path.exists(args.bin):
        raise SystemExit(f"golden: {args.bin} not found -- build it with\n"
                         f"    bash docs/showcase/demos/build.sh verify_golden")

    tmp = args.keep_out or tempfile.mkdtemp(prefix="chrono_golden_")
    try:
        render(args.bin, tmp)

        if args.bless:
            os.makedirs(args.refs, exist_ok=True)
            manifest = {"images": {}, "signature": {}}
            for name in IMAGES:
                src = os.path.join(tmp, name, "frame_0.png")
                if not os.path.exists(src):
                    raise SystemExit(f"golden: expected {src}; the renderer did not produce it")
                dst = os.path.join(args.refs, name + ".png")
                load_upright(src).save(dst, optimize=True)
                manifest["images"][name] = {"sha256": sha256(dst), "bytes": os.path.getsize(dst)}
            manifest["signature"] = read_signature(os.path.join(tmp, "signature.txt"))
            with open(os.path.join(args.refs, "manifest.json"), "w") as f:
                json.dump(manifest, f, indent=2, sort_keys=True)
                f.write("\n")
            total = sum(v["bytes"] for v in manifest["images"].values())
            print(f"golden: blessed {len(manifest['images'])} references "
                  f"({total/1024:.1f} KiB) + {len(manifest['signature'])} scalars -> {args.refs}")
            return 0

        man_path = os.path.join(args.refs, "manifest.json")
        if not os.path.exists(man_path):
            raise SystemExit(f"golden: no references in {args.refs}; create them with --bless")
        with open(man_path) as f:
            manifest = json.load(f)

        failures, identical = [], 0
        print(f"golden-image regression  (tolerance: max {args.max_diff} LSB, mean {args.mean_diff} LSB)")
        print(f"  references: {args.refs}")
        for name in IMAGES:
            ref = os.path.join(args.refs, name + ".png")
            new = os.path.join(tmp, name, "frame_0.png")
            if not os.path.exists(ref):
                failures.append(name)
                print(f"  [FAIL] {name:<14} no reference image (bless it)")
                continue
            if not os.path.exists(new):
                failures.append(name)
                print(f"  [FAIL] {name:<14} the renderer produced no frame")
                continue
            r = compare_images(ref, new, args.max_diff, args.mean_diff)
            if "note" in r:
                failures.append(name)
                print(f"  [FAIL] {name:<14} {r['note']}")
                continue
            if r["identical"]:
                identical += 1
            tag = "PASS" if r["ok"] else "FAIL"
            if not r["ok"]:
                failures.append(name)
                # a visual diff is far more useful than the numbers when something moves
                dump_diff(ref, new, os.path.join(tmp, f"diff_{name}.png"))
            if not args.quiet or not r["ok"]:
                print(f"  [{tag}] {name:<14} max {r['max']:>3} LSB, mean {r['mean']:.4f}, "
                      f"{r['px_changed']}/{r['px_total']} px changed"
                      f"{'  (bit-identical)' if r['identical'] else ''}"
                      f"{'' if r['ok'] else '  -> diff_' + name + '.png in ' + tmp}")

        sig_rows = compare_signature(manifest.get("signature", {}),
                                     read_signature(os.path.join(tmp, "signature.txt")), args.sig_tol)
        sig_bad = [r for r in sig_rows if not r[3]]
        if sig_rows and (not args.quiet or sig_bad):
            print(f"  lidar/radar scalars ({len(sig_rows)} values, relative tolerance {args.sig_tol:g})")
            for key, r, n, ok, note in sig_rows:
                if ok and args.quiet:
                    continue
                rs = "----" if r is None else f"{r:.5f}"
                ns = "----" if n is None else f"{n:.5f}"
                print(f"    [{'PASS' if ok else 'FAIL'}] {key:<24} ref {rs:>12}  now {ns:>12}   {note}")
        if sig_bad:
            failures.append("signature")

        print(f"\ngolden: {len(IMAGES) - len([f for f in failures if f != 'signature'])}/{len(IMAGES)} images "
              f"within tolerance ({identical} bit-identical), "
              f"{len(sig_rows) - len(sig_bad)}/{len(sig_rows)} scalars match")
        if failures:
            print(f"golden: REGRESSION in {', '.join(failures)}")
            print(f"golden: rendered frames kept in {tmp}" if args.keep_out else
                  "golden: rerun with --keep-out DIR to inspect the rendered frames")
            return 1
        print("golden: OK")
        return 0
    finally:
        if not args.keep_out and os.path.isdir(tmp):
            shutil.rmtree(tmp, ignore_errors=True)


def dump_diff(ref_png, new_png, dst):
    a = np.asarray(load_ref(ref_png), dtype=np.int16)
    b = np.asarray(load_upright(new_png), dtype=np.int16)
    d = np.abs(a - b)[:, :, :3].max(axis=2)
    # amplify so a 1-2 LSB change is actually visible
    vis = np.clip(d.astype(np.float32) * 16.0, 0, 255).astype(np.uint8)
    Image.fromarray(vis, mode="L").save(dst)


if __name__ == "__main__":
    sys.exit(main())
