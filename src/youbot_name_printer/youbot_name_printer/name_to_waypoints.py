import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
from matplotlib.textpath import TextPath
from matplotlib.font_manager import FontProperties

def name_to_waypoints(
    name: str,
    z_write: float,        # Z height when writing (pen down)
    z_travel: float,       # Z height when moving between letters (pen up)
    x_offset: float,       # where to start writing in robot space (mm)
    y_offset: float,
    scale: float,          # scale factor — tune until letters are the right size
    n_points: int = 200,   # total points to sample along the path
) -> list[list[float]]:
    """
    Convert a name string into a list of [x, y, z] waypoints in robot space.
    Returns pen-up travel moves at z_travel and pen-down write moves at z_write.
    """
    fp = FontProperties(family="DejaVu Sans", style="normal")
    tp = TextPath((0, 0), name, size=1, prop=fp)
    
    waypoints = []

    for path in tp.to_polygons():
        if len(path) < 2:
            continue

        # Resample the path to n_points evenly spaced points
        # (raw font paths have uneven point density)
        dists = np.cumsum(np.r_[0, np.hypot(np.diff(path[:,0]), np.diff(path[:,1]))])
        total = dists[-1]
        sample_dists = np.linspace(0, total, max(2, int(n_points * total)))
        xs = np.interp(sample_dists, dists, path[:,0])
        ys = np.interp(sample_dists, dists, path[:,1])

        # Pen up — move to start of this stroke at travel height
        waypoints.append([
            xs[0] * scale + x_offset,
            ys[0] * scale + y_offset,
            z_travel,
        ])

        # Pen down — trace the stroke at write height
        for x, y in zip(xs, ys):
            waypoints.append([
                x * scale + x_offset,
                y * scale + y_offset,
                z_write,
            ])

    return waypoints