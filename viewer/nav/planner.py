"""A* on the inflated occupancy grid + line-of-sight waypoint simplification."""

import heapq
import math

import numpy as np

SQRT2 = math.sqrt(2.0)


def octile(a, b):
    """Admissible 8-connected heuristic."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


def astar(passable, start, goal):
    """8-connected A* over a bool passability mask.

    Diagonal moves cost sqrt(2) and are forbidden when either adjacent
    cardinal cell is blocked (no corner cutting -- the robot has a body).
    Returns a list of (row, col) from start to goal, or None.
    """
    rows, cols = passable.shape
    if not passable[start] or not passable[goal]:
        return None
    g = {start: 0.0}
    came = {}
    pq = [(octile(start, goal), start)]
    closed = set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        r, c = cur
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols) or not passable[nr, nc]:
                    continue
                if dr != 0 and dc != 0 and not (passable[r, nc] and passable[nr, c]):
                    continue
                step = SQRT2 if dr != 0 and dc != 0 else 1.0
                ng = g[cur] + step
                nxt = (nr, nc)
                if ng < g.get(nxt, math.inf):
                    g[nxt] = ng
                    came[nxt] = cur
                    heapq.heappush(pq, (ng + octile(nxt, goal), nxt))
    return None


def line_clear(passable, a, b):
    """True if the straight segment a->b stays on passable cells (dense sampling
    at half-cell resolution, so it cannot skip over a blocked cell)."""
    steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) * 2 + 1
    for t in np.linspace(0.0, 1.0, steps + 1):
        r = int(round(a[0] + (b[0] - a[0]) * t))
        c = int(round(a[1] + (b[1] - a[1]) * t))
        if not passable[r, c]:
            return False
    return True


def simplify_path(path, passable):
    """Greedy shortcutting: from each kept waypoint jump to the farthest path
    cell still in line of sight. Turns a cell-by-cell path into a few legs."""
    if not path:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_clear(passable, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out
