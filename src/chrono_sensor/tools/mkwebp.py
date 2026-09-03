#!/usr/bin/env python3
# ChFilterSave PNG sequence -> animated .webp (via Pillow). Frames are already upright:
# the renderers produce bottom-up buffers and ChFilterSave flips on write.
# Usage: mkwebp.py <in_dir with frame_*.png> <out.webp> [--skip N] [--fps F] [--width W] [--max M] [--seg]
#   --seg : colorize a segmentation sequence (raw class-id bytes -> palette)
import sys, os, glob, re
from PIL import Image
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from segutil import load_seg, save_webp

def frames(d):
    fs = glob.glob(os.path.join(d, "frame_*.png"))
    return sorted(fs, key=lambda p: int(re.search(r"frame_(\d+)", p).group(1)))

def arg(flag, d):
    return type(d)(sys.argv[sys.argv.index(flag)+1]) if flag in sys.argv else d

def main():
    ind, out = sys.argv[1], sys.argv[2]
    seg = "--seg" in sys.argv
    skip = arg("--skip", 6); fps = arg("--fps", 25.0); width = arg("--width", 700); mx = arg("--max", 90)
    fs = frames(ind)[skip:]
    if not fs:
        print("NO FRAMES:", ind); return 1
    if len(fs) > mx:
        step = len(fs)/float(mx); fs = [fs[int(i*step)] for i in range(mx)]
    imgs = []
    for f in fs:
        im = load_seg(f) if seg else Image.open(f).convert("RGB")
        if im.width > width:
            im = im.resize((width, round(im.height*width/im.width)), Image.LANCZOS)
        imgs.append(im)
    # Label masks are encoded losslessly (a smeared class boundary is semantically wrong);
    # other synthetic modalities force all-keyframe to kill inter-frame ghosting.
    save_webp(imgs, out, fps, quality=arg("--quality", 58),
              synthetic=("--synthetic" in sys.argv), lossless=(seg or "--lossless" in sys.argv))
    return 0

if __name__ == "__main__":
    sys.exit(main())
