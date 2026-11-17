#!/usr/bin/env python3
"""
Compare two maps that share a map frame: an extended map against the map it came from.

    ./compare_maps.py BASE EXTENDED                 # overlay in ~/jupyter_ws/tmp
    ./compare_maps.py BASE EXTENDED --out FILE.png

Measures how far the walls of EXTENDED sit from BASE's walls over the area
BASE observed, so new area does not count, and draws both on one image:
BASE's walls blue, EXTENDED's red, dark gray where they coincide. A start
pose that was off shows as a second, shifted copy of the walls in red.
Without ROS.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys

import numpy as np
import scan_fit as sf
import yaml

PGM_FREE = 254
PGM_UNKNOWN = 205
BASE_COLOR = (40, 80, 255)
EXTENDED_COLOR = (255, 40, 40)
FREE_COLOR = (235, 235, 235)
SCALE = 4


@dataclass
class Grid:
    pixels: np.ndarray  # map_saver pixels, row 0 at the top
    origin: tuple[float, float]
    resolution: float


@dataclass
class Agreement:
    cells: int
    median_m: float
    within: float  # share within FIT_INLIER_M
    beyond: float  # share beyond twice that


def load(stem: Path) -> Grid:
    from PIL import Image

    meta = yaml.safe_load(Path(f'{stem}.yaml').read_text())
    pixels = np.asarray(Image.open(stem.parent / meta['image']).convert('L'))
    return Grid(
        pixels, (float(meta['origin'][0]), float(meta['origin'][1])), float(meta['resolution'])
    )


def wall_agreement(base: Grid, extended: Grid) -> Agreement:
    """Distances from EXTENDED's occupied cells to BASE's walls, over BASE's observed area."""
    rows, cols = np.nonzero(extended.pixels == sf.PGM_OCCUPIED)
    h = extended.pixels.shape[0]
    res = extended.resolution
    x = extended.origin[0] + (cols + 0.5) * res
    y = extended.origin[1] + (h - 1 - rows + 0.5) * res
    bh, bw = base.pixels.shape
    bc = np.floor((x - base.origin[0]) / base.resolution).astype(int)
    br = bh - 1 - np.floor((y - base.origin[1]) / base.resolution).astype(int)
    inside = (bc >= 0) & (bc < bw) & (br >= 0) & (br < bh)
    known = np.zeros_like(inside)
    known[inside] = base.pixels[br[inside], bc[inside]] != PGM_UNKNOWN
    if not known.any():
        return Agreement(0, float('nan'), 0.0, 0.0)
    grid = sf.grid_from_pixels(base.pixels, base.origin, base.resolution)
    d = grid.lookup(x[known], y[known])
    return Agreement(
        int(known.sum()),
        float(np.median(d)),
        float(np.mean(d <= sf.FIT_INLIER_M)),
        float(np.mean(d > 2 * sf.FIT_INLIER_M)),
    )


def overlay(base: Grid, extended: Grid) -> np.ndarray:
    """Return an RGB image of both maps on their combined extent (same resolution assumed)."""
    res = base.resolution
    x0, y0 = min(base.origin[0], extended.origin[0]), min(base.origin[1], extended.origin[1])
    x1 = max(g.origin[0] + g.pixels.shape[1] * res for g in (base, extended))
    y1 = max(g.origin[1] + g.pixels.shape[0] * res for g in (base, extended))
    width, height = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    img = np.full((height, width, 3), 255, np.uint8)
    walls = np.full((height, width, 3), 255, np.uint8)
    any_wall = np.zeros((height, width), bool)
    for g, color in ((base, BASE_COLOR), (extended, EXTENDED_COLOR)):
        gh, gw = g.pixels.shape
        c0 = int(round((g.origin[0] - x0) / res))
        r0 = height - int(round((g.origin[1] - y0) / res)) - gh
        window = (slice(r0, r0 + gh), slice(c0, c0 + gw))
        img[window][g.pixels == PGM_FREE] = FREE_COLOR
        occupied = g.pixels == sf.PGM_OCCUPIED
        # The channel minimum turns walls drawn by both maps dark gray.
        layer = walls[window]
        layer[occupied] = np.minimum(layer[occupied], color)
        any_wall[window] |= occupied
    img[any_wall] = walls[any_wall]
    return img


def main(argv: list[str] | None = None) -> int:
    from PIL import Image
    from neoracer_ros2_driver.maps import map_stem

    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('base', help='the map extended from')
    ap.add_argument('extended', help='the extended map')
    ap.add_argument('--out', type=Path, help='overlay image (default ~/jupyter_ws/tmp/...)')
    args = ap.parse_args(argv)
    base, extended = load(map_stem(args.base)), load(map_stem(args.extended))
    a = wall_agreement(base, extended)
    print(
        f"{args.extended} walls over {args.base}'s observed area: {a.cells} cells; "
        f'median {a.median_m * 1e3:.0f} mm, {a.within:.1%} within '
        f'{sf.FIT_INLIER_M * 1e3:.0f} mm, {a.beyond:.1%} beyond {2 * sf.FIT_INLIER_M * 1e3:.0f} mm'
    )
    out = args.out or Path.home() / 'jupyter_ws' / 'tmp' / f'{args.extended}_over_{args.base}.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    img = overlay(base, extended)
    Image.fromarray(img).resize((img.shape[1] * SCALE, img.shape[0] * SCALE), Image.NEAREST).save(
        out
    )
    print(f'Overlay: {out} (blue {args.base}, red {args.extended}, dark gray both)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
