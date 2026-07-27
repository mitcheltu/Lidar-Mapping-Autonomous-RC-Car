"""A* on the inflated occupancy grid + line-of-sight waypoint simplification."""

import heapq
import math

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
    """True if a straight center-to-center run from cell a to cell b is legal:
    every cell the segment crosses is passable, and passing exactly through a
    cell corner obeys the same no-corner-cutting rule as astar (both flanking
    cells must be passable)."""
    (r0, c0), (r1, c1) = a, b
    adr, adc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r1 >= r0 else -1
    sc = 1 if c1 >= c0 else -1
    r, c = r0, c0
    if not passable[r, c]:
        return False
    n_r = n_c = 0                     # cell boundaries crossed so far, per axis
    while n_r < adr or n_c < adc:
        # parametric position of the next boundary crossing on each axis
        t_r = (n_r + 0.5) / adr if adr else 2.0
        t_c = (n_c + 0.5) / adc if adc else 2.0
        if abs(t_r - t_c) < 1e-12:    # exact corner: diagonal transition
            if not (passable[r + sr, c] and passable[r, c + sc]):
                return False          # no corner cutting
            r += sr; c += sc; n_r += 1; n_c += 1
        elif t_r < t_c:
            r += sr; n_r += 1
        else:
            c += sc; n_c += 1
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
