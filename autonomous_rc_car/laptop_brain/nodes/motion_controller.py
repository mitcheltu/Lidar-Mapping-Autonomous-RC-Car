#!/usr/bin/env python3
"""A small pure-pursuit controller for differential-drive steering."""

import math


class PurePursuitController:
    def __init__(self, lookahead=0.35, max_steer=0.8):
        self.lookahead = lookahead
        self.max_steer = max_steer

    def steer(self, pose, path):
        if not path:
            return 0.0, 0.0
        x, y, yaw = pose
        target = None
        best_d = float("inf")
        for point in path:
            px, py = point
            dx = px - x
            dy = py - y
            d = math.hypot(dx, dy)
            if d < self.lookahead:
                continue
            if d < best_d:
                best_d = d
                target = (px, py)
        if target is None:
            target = path[-1]
        dx = target[0] - x
        dy = target[1] - y
        heading = math.atan2(dy, dx)
        steer = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
        return max(min(steer, self.max_steer), -self.max_steer), 0.0


if __name__ == "__main__":
    controller = PurePursuitController()
    pose = (0.0, 0.0, 0.0)
    path = [(1.0, 0.0), (2.0, 0.0)]
    print(controller.steer(pose, path))
