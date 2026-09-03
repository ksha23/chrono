#!/usr/bin/env python3
# Render dumped 360-lidar range buffers (frame_*.bin) as a car-centric bird's-eye point cloud webp.
# Points coloured by height. Usage: mklidar.py <in_dir> <out.webp> [--fps F] [--max M] [--view R]
import sys, os, glob, re, struct
import numpy as np
from PIL import Image, ImageDraw
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from segutil import save_webp

def frames(d):
    fs = glob.glob(os.path.join(d, "scan", "frame_*.bin"))
    return sorted(fs, key=lambda p: int(re.search(r"frame_(\d+)", p).group(1)))

def arg(flag, d):
    return type(d)(sys.argv[sys.argv.index(flag)+1]) if flag in sys.argv else d

def colormap(t):  # blue -> cyan -> green -> yellow -> red  (t in [0,1])
    t = np.clip(t, 0, 1)
    r = np.clip(1.5 - abs(4*t - 3), 0, 1)
    g = np.clip(1.5 - abs(4*t - 2), 0, 1)
    b = np.clip(1.5 - abs(4*t - 1), 0, 1)
    return np.stack([r, g, b], -1)

def render(path, S, view):
    with open(path, "rb") as f:
        w, h = struct.unpack("ii", f.read(8))
        hfov, vmax, vmin, maxd = struct.unpack("ffff", f.read(16))
        rng = np.frombuffer(f.read(4*w*h), np.float32).reshape(h, w).astype(np.float64)
    az = (-hfov/2 + (np.arange(w)+0.5)/w*hfov)[None, :]
    el = (vmin + (np.arange(h)/(h-1))*(vmax-vmin))[:, None]
    valid = (rng > 0.2) & (rng < maxd*0.98)
    ce = np.cos(el); x = rng*ce*np.cos(az); y = rng*ce*np.sin(az); z = rng*np.sin(el)
    x, y, z = x[valid], y[valid], z[valid]
    img = np.zeros((S, S, 3), np.float64)
    # forward (x) -> up, left (y) -> left ; sensor at centre
    px = (S/2 - y/view*(S/2)).astype(int)   # left(+y) -> left
    py = (S/2 - x/view*(S/2)).astype(int)   # forward(+x) -> up
    m = (px >= 0) & (px < S) & (py >= 0) & (py < S)
    px, py = px[m], py[m]
    # Colour by height. Almost every return is ground/near-ground, so a wide [-2,2] window collapsed the
    # whole cloud into one dark blue. Map the actual sensor-relative band (~ -2.1 m ground .. +0.6 m tops)
    # and lift the floor so ground points read as a bright cyan-green sheet rather than near-black.
    t = np.clip((z[m] + 2.2) / 2.8, 0, 1)
    col = colormap(t) * 0.75 + 0.25          # floor the brightness -> every point is clearly visible
    # 2x2 point splat so single returns don't vanish at this resolution
    for dx, dy in ((0, 0), (1, 0), (0, 1), (1, 1)):
        qx = np.clip(px + dx, 0, S-1); qy = np.clip(py + dy, 0, S-1)
        img[qy, qx] = np.maximum(img[qy, qx], col)
    im = Image.fromarray((np.clip(img, 0, 1)*255).astype(np.uint8), "RGB")
    dr = ImageDraw.Draw(im)
    for rr in (10, 20, 30, 40):              # faint range rings
        rad = rr/view*(S/2)
        dr.ellipse([S/2-rad, S/2-rad, S/2+rad, S/2+rad], outline=(55, 55, 60))
    dr.polygon([(S/2, S/2-9), (S/2-5, S/2+6), (S/2+5, S/2+6)], fill=(240, 240, 255))  # car, pointing up
    dr.rectangle([0, 0, 168, 20], fill=(0, 0, 0)); dr.text((5, 5), "360 lidar - BEV point cloud", fill=(255, 255, 255))
    return im

def main():
    ind, out = sys.argv[1], sys.argv[2]
    fps = arg("--fps", 20.0); mx = arg("--max", 90); outw = arg("--width", 1100); view = arg("--view", 45.0); S = 720
    fs = frames(ind)[arg("--skip", 8):]   # drop the suspension-settling frames at the start
    if not fs:
        print("NO FRAMES:", ind); return 1
    if len(fs) > mx:
        step = len(fs)/float(mx); fs = [fs[int(i*step)] for i in range(mx)]
    # Pair each sensor plot with the companion camera frame from the same scan index, side by side.
    imgs = []
    for f in fs:
        idx = int(re.search(r"frame_(\d+)", f).group(1))
        plot = render(f, S, view)
        campath = os.path.join(ind, "cam", f"frame_{idx}.png")
        if os.path.exists(campath):
            cam = Image.open(campath).convert("RGB")   # ChFilterSave already writes upright
            if cam.size != (S, S):
                cam = cam.resize((S, S), Image.LANCZOS)
            d2 = ImageDraw.Draw(cam)
            d2.rectangle([0, 0, 118, 20], fill=(0, 0, 0))
            d2.text((5, 5), "scene (RGB camera)", fill=(255, 255, 255))
            pair = Image.new("RGB", (2*S, S)); pair.paste(cam, (0, 0)); pair.paste(plot, (S, 0))
            imgs.append(pair)
        else:
            imgs.append(plot)
    if imgs[0].width > outw:   # downscale the camera|plot pair for a smaller repo footprint
        imgs = [f.resize((outw, round(f.height*outw/f.width)), Image.LANCZOS) for f in imgs]
    save_webp(imgs, out, fps, quality=arg("--quality", 58), synthetic=True)
    return 0

if __name__ == "__main__":
    sys.exit(main())
