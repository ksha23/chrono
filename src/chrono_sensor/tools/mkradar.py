#!/usr/bin/env python3
# Render dumped forward-radar returns (frame_*.bin) as a top-down plot: returns in the FOV wedge, coloured
# by Doppler closing speed, sized by amplitude. Usage: mkradar.py <in_dir> <out.webp> [--fps F] [--max M] [--view R]
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

def dopcolor(t):  # closing speed: 0 = cyan (crossing/static) -> 0.5 = yellow -> 1 = red (closing fast)
    t = float(np.clip(t, 0, 1))
    if t < 0.5:                       # cyan -> yellow
        k = t/0.5
        return (int(40 + 215*k), int(220), int(230*(1-k) + 40))
    k = (t-0.5)/0.5                   # yellow -> red
    return (255, int(220*(1-k) + 45), 40)

def render(path, S, view):
    with open(path, "rb") as f:
        w, h = struct.unpack("ii", f.read(8))
        hfov, vfov, maxd = struct.unpack("fff", f.read(12))
        R = np.frombuffer(f.read(36), np.float32).reshape(3, 3).astype(np.float64)  # sensor->world
        rec = np.frombuffer(f.read(4*7*w*h), np.float32).reshape(-1, 7)
    rng, az, el = rec[:, 0], rec[:, 1], rec[:, 2]
    dv = rec[:, 3:6].astype(np.float64); amp = rec[:, 6]
    # Drop GROUND returns: the road lies at a near-constant height below the sensor, so ground hits share a
    # sensor-relative height (z = range*sin(elevation)) regardless of range. Without this the plot is just
    # concentric arcs of pavement and the real obstacles are invisible.
    # The cut is ADAPTIVE (relative to this frame's own ground plane, taken as the median z, since ground
    # dominates the returns): a fixed threshold leaves arc fragments whenever the chassis pitches or rides
    # at a different height -- which is exactly what happens while the suspension settles at the start.
    z_rel = rng * np.sin(el)
    base = (rng > 0.2) & (rng < maxd) & (amp > 0)
    z_ground = np.median(z_rel[base]) if base.any() else -0.9
    valid = base & (z_rel > z_ground + 0.35)
    rng, az, el, dv, amp = rng[valid], az[valid], el[valid], dv[valid], amp[valid]
    # True RADIAL Doppler: project the world-frame relative velocity onto each beam's world direction.
    # |dv| alone is the same for every static object (it is just the car's own speed) -> a flat, uniform plot.
    ce = np.cos(el)
    u_s = np.stack([ce*np.cos(az), ce*np.sin(az), np.sin(el)], -1)   # sensor frame: x fwd, y left, z up
    u_w = u_s @ R.T
    speed = np.abs(-np.einsum("ij,ij->i", dv, u_w))                  # closing speed along the beam
    x = rng*np.cos(el)*np.cos(az); y = rng*np.cos(el)*np.sin(az)  # forward, left
    im = Image.new("RGB", (S, S), (12, 14, 18)); dr = ImageDraw.Draw(im)
    cx, cy = S/2, S - 40                          # sensor near bottom centre, looking up
    for rr in (15, 30, 45, 60):                   # range rings
        rad = rr/view*(S-60)
        dr.arc([cx-rad, cy-rad, cx+rad, cy+rad], 180+45, 360-45, fill=(45, 48, 55))
    for ang in (-hfov/2, hfov/2):                 # FOV wedge edges
        dr.line([cx, cy, cx+np.sin(ang)*(S-60), cy-np.cos(ang)*(S-60)], fill=(45, 48, 55))
    SMAX = 6.0                                     # fixed Doppler scale (m/s); the car closes at ~5.3 m/s
    order = np.argsort(amp)                        # draw strong returns last
    for k in order:
        # +y is LEFT in the sensor frame, so it must map to the LEFT (smaller px) of a forward-up
        # top-down plot -- otherwise the plot is mirrored relative to the companion camera view.
        px = cx - y[k]/view*(S-60); py = cy - x[k]/view*(S-60)
        if 0 <= px < S and 0 <= py < S:
            rad = 3 + 5*min(1.0, amp[k])
            dr.ellipse([px-rad, py-rad, px+rad, py+rad], fill=dopcolor(speed[k]/SMAX))
    dr.polygon([(cx, cy-8), (cx-6, cy+7), (cx+6, cy+7)], fill=(240, 240, 255))  # sensor
    dr.rectangle([0, 0, 210, 20], fill=(0, 0, 0))
    dr.text((5, 5), "forward radar - Doppler returns", fill=(255, 255, 255))
    return im

def main():
    ind, out = sys.argv[1], sys.argv[2]
    fps = arg("--fps", 20.0); mx = arg("--max", 90); outw = arg("--width", 1100); view = arg("--view", 65.0); S = 720
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
