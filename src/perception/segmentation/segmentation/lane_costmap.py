#!/usr/bin/env python3
"""
lane_costmap.py — turns a raw per-frame confirmed-lane BEV mask into a
robust distance-based cost grid, low at the lane center and rising
toward the edges, with no explicit centerline ever extracted or tracked.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

Why no centerline tracking: an earlier lane-line grid in this project
spent a long time fighting exactly the failure mode a tracked,
1-cell-wide feature has -- small per-frame noise makes a discrete
tracked point or line jitter hard. A distance transform sidesteps that:
the "center" is just wherever a cell is farthest from any edge of the
region, which falls out of the whole mask's shape as a smooth field,
not a fragile single measurement. A stray pixel or two at the boundary
barely moves it.

Two robustness problems specific to this mask source (confirmed live,
not hypothetical) still needed solving on top of that:

1. DISCONNECTED FRAGMENTS. The raw per-frame confirmed mask isn't
always one clean blob -- noise islands elsewhere in the grid, or even a
separate lane picked up disconnected from ours. seeded_connected_mask
keeps ONLY the connected component that contains (or is nearest to) the
vehicle's own cell, and drops everything else outright -- since the
vehicle is by definition sitting in its own lane, seeding there is a
reliable, cheap way to throw out anything not actually part of it,
without needing any shape/size heuristic.

Seeding needs a real search radius, not just the vehicle's exact cell:
confirmed live, the front camera has a genuine blind spot in roughly the
first 4m / 20 grid cells directly ahead of the vehicle (that range gets
zero projected pixels at all, camera geometry/mounting height, not a
bug), so the vehicle's own cell is essentially never marked drivable.
seed_search_radius must comfortably exceed that blind spot's depth or
seeding finds nothing every single frame -- confirmed live as the actual
cause of an early version of this module returning an empty mask on
every frame. 25 cells (5m) clears that with margin.

2. TRANSIENT MIS-SEGMENTATION. For a few frames at a time, the model
can accidentally classify an adjacent lane as part of ours too. A
single frame can't tell this apart from a real, correct detection -- it
needs cross-frame evidence. RollingMajorityFilter keeps a short,
FIXED-length window of recent per-cell masks and requires a majority of
them to agree. This is deliberately NOT the same as two approaches this
project already tried and confirmed broken for an earlier lane-line
grid: not an OR-forever history (that let a stale blob persist
indefinitely once marked, since nothing ever un-marks a cell); not a
plain temporal blend/EMA either (a single bad frame still nudges the
result, it just takes longer to fade). A fixed window means a frame's
influence disappears completely once it ages out no matter what; a
majority vote means one bad frame, or even several, can never tip a
cell on their own -- they have to be the majority of the whole window
to be trusted.

Output convention matches the rest of this project's occupancy grids:
int8, 0-100. distance_cost_grid gives 0 at the deepest point of the
robust region (>= max_dist_cells from every edge -- effectively "lane
center or better"), ramping linearly up to 100 at the region's own
boundary, and 100 (fully non-drivable) everywhere outside the region.
"""

import cv2
import numpy as np


def seeded_connected_mask(mask, seed_row, seed_col, seed_search_radius=25):
    """mask: (H, W) bool. Returns mask restricted to the single connected
    component containing the vehicle's own cell (seed_row, seed_col). If
    that exact cell isn't marked drivable this frame (the common case --
    see module docstring on the camera's own blind spot at the vehicle),
    searches seed_search_radius cells around it for the nearest drivable
    cell to seed from instead. Returns an all-False mask if no drivable
    cell exists anywhere within the search radius -- deliberately not
    falling back to any other component; a region unconnected to ego is
    not "our lane" by definition here.
    """
    h, w = mask.shape
    out = np.zeros_like(mask)
    if not mask.any():
        return out

    num_labels, labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)

    seed_label = 0
    if 0 <= seed_row < h and 0 <= seed_col < w and mask[seed_row, seed_col]:
        seed_label = int(labels[seed_row, seed_col])
    else:
        best_d2 = None
        r0, r1 = max(0, seed_row - seed_search_radius), min(h, seed_row + seed_search_radius + 1)
        c0, c1 = max(0, seed_col - seed_search_radius), min(w, seed_col + seed_search_radius + 1)
        for r in range(r0, r1):
            for c in range(c0, c1):
                if mask[r, c]:
                    d2 = (r - seed_row) ** 2 + (c - seed_col) ** 2
                    if best_d2 is None or d2 < best_d2:
                        best_d2 = d2
                        seed_label = int(labels[r, c])

    if seed_label == 0:
        return out
    return labels == seed_label


class RollingMajorityFilter:
    """Keeps the last `window` per-cell boolean masks; update() returns,
    per cell, whether at least `min_votes` of the last `window` frames
    (including this one) marked it True. See module docstring for why
    this specific shape (fixed window + majority, not OR-forever, not a
    blend) was chosen.
    """

    def __init__(self, window=5, min_votes=3):
        if min_votes > window:
            raise ValueError('min_votes cannot exceed window')
        self.window = window
        self.min_votes = min_votes
        self._history = []

    def update(self, mask):
        self._history.append(mask.astype(np.uint8))
        if len(self._history) > self.window:
            self._history.pop(0)
        votes = np.sum(self._history, axis=0)
        return votes >= self.min_votes

    def reset(self):
        self._history = []


def distance_cost_grid(mask, max_dist_cells):
    """mask: (H, W) bool, the robust (seeded + temporally filtered)
    drivable region. Returns an (H, W) int8 cost grid -- see module
    docstring for the 0-100 convention. max_dist_cells is the distance
    (in grid cells) from an edge at which cost bottoms out at 0; roughly
    half a real lane's width is a reasonable starting point, not a
    finalized value.
    """
    if not mask.any():
        return np.full(mask.shape, 100, dtype=np.int8)

    dist = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)
    normalized = np.clip(dist / float(max_dist_cells), 0.0, 1.0)
    cost = np.where(mask, np.round(100 * (1.0 - normalized)), 100).astype(np.int8)
    return cost
