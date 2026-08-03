"""Shared helpers for showcase webp/composite generation."""
import numpy as np
from PIL import Image, ImageOps

# distinct, index-stable palette; index 0 = unlabeled/background (dark)
PALETTE = [(28, 28, 32), (222, 70, 70), (70, 200, 95), (72, 120, 240), (240, 200, 55),
           (210, 95, 220), (55, 205, 210), (240, 145, 45), (150, 220, 60), (205, 205, 205),
           (125, 85, 205), (85, 185, 145), (235, 110, 150), (120, 175, 235)]


def load_rgb(path):
    """ChFilterSave writes vertically flipped -> flip upright."""
    return ImageOps.flip(Image.open(path).convert("RGB"))


def load_seg(path):
    """Segmentation PNG stores raw (class_id, instance_id) as uint16 pairs in RGBA bytes.
    class_id = R + 256*G. Map each class to a stable palette color."""
    im = ImageOps.flip(Image.open(path).convert("RGBA"))
    a = np.asarray(im)
    cls = a[:, :, 0].astype(np.int32) + 256 * a[:, :, 1].astype(np.int32)
    out = np.zeros((*cls.shape, 3), np.uint8)
    for c in np.unique(cls):
        out[cls == c] = PALETTE[int(c) % len(PALETTE)]
    return Image.fromarray(out, "RGB")


def load_depth(path):
    """Depth PNG is a low-contrast grayscale visualization. Stretch contrast over the foreground
    (non-background) pixels so near/far read clearly; keep it grayscale (honest depth look)."""
    im = ImageOps.flip(Image.open(path).convert("L"))
    a = np.asarray(im).astype(np.float32)
    sky = a >= 252                       # sky/no-return encodes as ~white -> treat as background
    fg = a[(a > 4) & ~sky]
    if fg.size > 100:
        lo, hi = np.percentile(fg, 2), np.percentile(fg, 98)
        if hi - lo > 1:
            a = np.clip((a - lo) / (hi - lo), 0, 1) * 255.0
    a[sky] = 0                            # black sky -> the foreground depth gradient reads clearly
    return Image.fromarray(a.astype(np.uint8), "L").convert("RGB")
