#!/usr/bin/env python3
# ChFilterSave PNG sequence -> animated .webp (via Pillow). Frames are saved vertically
# flipped by ChFilterSave, so we flip them upright.
# Usage: mkwebp.py <in_dir with frame_*.png> <out.webp> [--skip N] [--fps F] [--width W] [--max M] [--seg]
#   --seg : colorize a segmentation sequence (raw class-id bytes -> palette)
import sys, os, glob, re
from PIL import Image, ImageOps
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from segutil import load_seg

def frames(d):
    fs = glob.glob(os.path.join(d, "frame_*.png"))
    return sorted(fs, key=lambda p: int(re.search(r"frame_(\d+)", p).group(1)))

def arg(flag, d):
    return type(d)(sys.argv[sys.argv.index(flag)+1]) if flag in sys.argv else d

def main():
    ind, out = sys.argv[1], sys.argv[2]
    seg = "--seg" in sys.argv
    skip = arg("--skip", 6); fps = arg("--fps", 25.0); width = arg("--width", 820); mx = arg("--max", 120)
    fs = frames(ind)[skip:]
    if not fs:
        print("NO FRAMES:", ind); return 1
    if len(fs) > mx:
        step = len(fs)/float(mx); fs = [fs[int(i*step)] for i in range(mx)]
    imgs = []
    for f in fs:
        im = load_seg(f) if seg else ImageOps.flip(Image.open(f).convert("RGB"))
        if im.width > width:
            im = im.resize((width, round(im.height*width/im.width)), Image.LANCZOS)
        imgs.append(im)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    imgs[0].save(out, format="WEBP", save_all=True, append_images=imgs[1:],
                 duration=int(1000/fps), loop=0, quality=70, method=4)
    print("wrote %s  (%d frames, %d KB)" % (out, len(imgs), os.path.getsize(out)//1024))
    return 0

if __name__ == "__main__":
    sys.exit(main())
