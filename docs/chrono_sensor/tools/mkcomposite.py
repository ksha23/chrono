#!/usr/bin/env python3
# Tile the four multisensor modalities (rgb, depth, normal, seg) into a 2x2 animated webp with labels.
# Usage: mkcomposite.py <multisensor_out_dir> <out.webp> [--fps F] [--max M]
import sys, os, glob, re
from PIL import Image, ImageDraw
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from segutil import load_rgb, load_seg, load_depth, save_webp

def nums(d):
    fs = glob.glob(os.path.join(d, "frame_*.png"))
    return sorted(int(re.search(r"frame_(\d+)", p).group(1)) for p in fs)

def arg(flag, d):
    return type(d)(sys.argv[sys.argv.index(flag)+1]) if flag in sys.argv else d

def label(img, text):
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, 8+7*len(text), 20], fill=(0, 0, 0))
    dr.text((5, 5), text, fill=(255, 255, 255))
    return img

def main():
    base, out = sys.argv[1], sys.argv[2]
    fps = arg("--fps", 24.0); mx = arg("--max", 90); outw = arg("--width", 1000)
    rgb_d = os.path.join(base, "rgb"); dep_d = os.path.join(base, "depth")
    nrm_d = os.path.join(base, "normal"); seg_d = os.path.join(base, "seg")
    idx = nums(rgb_d)
    if not idx:
        print("NO FRAMES in", rgb_d); return 1
    idx = idx[4:]  # drop settling frames
    if len(idx) > mx:
        step = len(idx)/float(mx); idx = [idx[int(i*step)] for i in range(mx)]
    frames = []
    for i in idx:
        f = lambda d: os.path.join(d, f"frame_{i}.png")
        try:
            tl = label(load_rgb(f(rgb_d)),   "RGB camera")
            tr = label(load_depth(f(dep_d)), "Depth")
            bl = label(load_seg(f(seg_d)),   "Segmentation")
            br = label(load_rgb(f(nrm_d)),   "Surface normals")
        except FileNotFoundError:
            continue
        w, h = tl.size
        canvas = Image.new("RGB", (2*w, 2*h))
        canvas.paste(tl, (0, 0)); canvas.paste(tr, (w, 0))
        canvas.paste(bl, (0, h)); canvas.paste(br, (w, h))
        frames.append(canvas)
    if not frames:
        print("NO COMPOSITE FRAMES"); return 1
    if frames[0].width > outw:   # downscale the 2x2 panel for a smaller repo footprint
        frames = [f.resize((outw, round(f.height*outw/f.width)), Image.LANCZOS) for f in frames]
    save_webp(frames, out, fps, quality=arg("--quality", 58), synthetic=True)
    return 0

if __name__ == "__main__":
    sys.exit(main())
