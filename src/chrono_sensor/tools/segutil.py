"""Shared helpers for showcase webp/composite generation."""
import os
import numpy as np
from PIL import Image


def save_webp(imgs, out, fps, quality=58, synthetic=False, lossless=False):
    """Encode a frame list as an animated WebP.

    Size tuning: measured on a 120-frame 820px clip (4160 KB at the old quality=70/method=4
    defaults), `method=6` and `minimize_size` alone only recover ~5% for ~14x the encode time
    -- the real levers are frame count and pixel width, which together roughly halve the file.
    So callers downscale and decimate, and we still pay for method=6 since encoding is a
    one-off while the bytes live in git forever.

    GHOSTING: animated WebP delta-codes each frame inside a bounding rectangle and only emits
    a keyframe every `kmax` frames. On photographic content the residue is masked by texture,
    but on synthetic imagery with hard edges over large flat areas -- depth maps, label masks,
    point-cloud plots -- the rectangle's stale contents stay visible, so a moving object leaves
    an outline hanging behind it for up to kmax frames (over a second at 24 fps). Passing
    `synthetic=True` forces every frame to be a keyframe, which removes the inter-frame
    prediction entirely: on the depth demo that cut mean error from 2.67 to 0.51 for +34% size.
    `lossless=True` additionally guarantees exact pixels -- worth it for label data, where a
    smeared class boundary is not just ugly but semantically wrong.
    """
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    kw = dict(method=6)
    if lossless:
        kw.update(lossless=True, kmax=1)
    elif synthetic:
        kw.update(quality=quality, kmax=1)          # every frame a keyframe -> no lingering ghost
    else:
        kw.update(quality=quality, minimize_size=True, kmax=30, kmin=9)
    imgs[0].save(out, format="WEBP", save_all=True, append_images=imgs[1:],
                 duration=int(1000 / fps), loop=0, **kw)
    print("wrote %s (%d frames, %d KB)" % (out, len(imgs), os.path.getsize(out) // 1024))

# distinct, index-stable palette; index 0 = unlabeled/background (dark)
PALETTE = [(28, 28, 32), (222, 70, 70), (70, 200, 95), (72, 120, 240), (240, 200, 55),
           (210, 95, 220), (55, 205, 210), (240, 145, 45), (150, 220, 60), (205, 205, 205),
           (125, 85, 205), (85, 185, 145), (235, 110, 150), (120, 175, 235)]


def load_rgb(path):
    """ChFilterSave already writes upright (it calls stbi_flip_vertically_on_write)."""
    return Image.open(path).convert("RGB")


def load_seg(path):
    """Segmentation PNG stores raw (class_id, instance_id) as uint16 pairs in RGBA bytes.
    class_id = R + 256*G. Map each class to a stable palette color."""
    im = Image.open(path).convert("RGBA")
    a = np.asarray(im)
    cls = a[:, :, 0].astype(np.int32) + 256 * a[:, :, 1].astype(np.int32)
    out = np.zeros((*cls.shape, 3), np.uint8)
    for c in np.unique(cls):
        out[cls == c] = PALETTE[int(c) % len(PALETTE)]
    return Image.fromarray(out, "RGB")


def load_depth(path):
    """Depth PNG -> grayscale visualization, near surfaces bright and far ones dark.

    A ray that hits nothing now returns max_depth (matching OptiX), so sky/no-return sits at
    the DARK end of the range, not the bright end. An earlier revision of the backend returned
    0.0 for a miss, which made the sky read as the closest possible surface and inverted the
    whole map -- hence the background mask keys on the dark end here. Contrast is stretched
    over the real (non-sky) samples so the foreground gradient is legible.
    """
    im = Image.open(path).convert("L")
    a = np.asarray(im).astype(np.float32)
    sky = a <= 3                          # miss / max_depth -> background
    fg = a[~sky]
    if fg.size > 100:
        lo, hi = np.percentile(fg, 2), np.percentile(fg, 98)
        if hi - lo > 1:
            a = np.clip((a - lo) / (hi - lo), 0, 1) * 255.0
    a[sky] = 0                            # keep the background black
    return Image.fromarray(a.astype(np.uint8), "L").convert("RGB")
