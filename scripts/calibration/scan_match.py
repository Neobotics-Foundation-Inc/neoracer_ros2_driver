"""
2D lidar scan helpers and scan matching for the calibration tools.

match_scans() aligns two scans of a static scene with point-to-line ICP:
each point of the second scan is paired with its nearest neighbor in the
first, the pair's error is measured along the first scan's local surface
normal, and the worst pairs are trimmed so a few moved objects do not pull
the fit. A pose is (x, y, theta): meters, meters, radians.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import warnings

import numpy as np
from scipy.spatial import cKDTree

Pose = tuple[float, float, float]

NORMAL_NEIGHBORS = 6
# Correspondence gates, coarse to fine (m): the initial guess may be tens of
# cm off after a drive, the converged fit is within millimeters.
GATES = (0.5, 0.2, 0.08)
TRIM_FRACTION = 0.2
MAX_ITERATIONS = 30
CONVERGED_STEP = 1e-5


def scan_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
) -> np.ndarray:
    """Return (N, 2) points in the laser frame for the valid returns."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(ranges.size)
    ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    r, a = ranges[ok], angles[ok]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def median_ranges(scans: list[list[float]]) -> np.ndarray | None:
    """Per-beam median over scans of the most common length; inf counts as missing."""
    if not scans:
        return None
    lengths = [len(s) for s in scans]
    n = max(set(lengths), key=lengths.count)
    stack = np.array([s for s in scans if len(s) == n], dtype=float)
    stack[~np.isfinite(stack)] = np.nan
    # Beams with no return in any scan stay NaN, which scan_points drops.
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        return np.nanmedian(stack, axis=0)


def compose(a: Pose, b: Pose) -> Pose:
    """Return a then b: b expressed in a's frame, mapped to a's parent."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def invert(a: Pose) -> Pose:
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def transform(points: np.ndarray, pose: Pose) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + np.array(pose[:2])


def base_motion(laser_motion: Pose, laser_in_base: Pose) -> Pose:
    """Return base_link's motion for a laser motion, both in their own start frames."""
    return compose(compose(laser_in_base, laser_motion), invert(laser_in_base))


def laser_motion(base: Pose, laser_in_base: Pose) -> Pose:
    """Inverse of base_motion: the laser's motion for a base_link motion."""
    return compose(compose(invert(laser_in_base), base), laser_in_base)


def surface_normals(points: np.ndarray, tree: cKDTree) -> np.ndarray:
    """Return a unit normal per point from the principal axes of its neighbors."""
    k = min(NORMAL_NEIGHBORS, len(points))
    _, idx = tree.query(points, k=k)
    nbrs = points[idx] - points[idx].mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', nbrs, nbrs)
    _, vecs = np.linalg.eigh(cov)
    return vecs[:, :, 0]


@dataclass(frozen=True)
class MatchResult:
    """Pose of the second scan's frame in the first's, and the fit quality."""

    pose: Pose
    rms_m: float  # point-to-line RMS over the kept pairs
    inliers: float  # fraction of the second scan's points within the final gate
    iterations: int


def match_scans(reference: np.ndarray, scan: np.ndarray, guess: Pose) -> MatchResult:
    """Align `scan` to `reference` starting from `guess` (pose of scan in reference)."""
    tree = cKDTree(reference)
    normals = surface_normals(reference, tree)
    pose = guess
    iterations = 0
    for gate in GATES:
        for _ in range(MAX_ITERATIONS):
            iterations += 1
            moved = transform(scan, pose)
            dist, idx = tree.query(moved, distance_upper_bound=gate)
            ok = np.isfinite(dist)
            if ok.sum() < 3:
                return MatchResult(pose, math.inf, 0.0, iterations)
            p, r, n = moved[ok], reference[idx[ok]], normals[idx[ok]]
            err = np.einsum('ij,ij->i', p - r, n)
            keep = np.abs(err) <= np.quantile(np.abs(err), 1.0 - TRIM_FRACTION)
            p, r, n, err = p[keep], r[keep], n[keep], err[keep]
            # Linearized about the current pose: rotation about the origin of
            # the reference frame, applied after the current transform.
            jac = np.column_stack((n[:, 0], n[:, 1], n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1]))
            step, *_ = np.linalg.lstsq(jac, -err, rcond=None)
            dx, dy, dth = (float(v) for v in step)
            pose = compose((dx, dy, dth), pose)
            if max(abs(dx), abs(dy), abs(dth)) < CONVERGED_STEP:
                break
    moved = transform(scan, pose)
    dist, idx = tree.query(moved, distance_upper_bound=GATES[-1])
    ok = np.isfinite(dist)
    if not ok.any():
        return MatchResult(pose, math.inf, 0.0, iterations)
    err = np.einsum('ij,ij->i', moved[ok] - reference[idx[ok]], normals[idx[ok]])
    return MatchResult(
        pose=pose,
        rms_m=float(np.sqrt(np.mean(err**2))),
        inliers=float(ok.mean()),
        iterations=iterations,
    )
