import math

from nav.controller import WaypointFollower


def test_done_when_no_waypoints():
    f = WaypointFollower(waypoints=[])
    assert f.done
    assert f.update(0.0, 0.0, 0.0) == (0, 0)


def test_turns_in_place_when_facing_wrong_way():
    # waypoint straight "ahead" in +x; car faces +z (90 deg off)
    f = WaypointFollower(waypoints=[(1.0, 0.0)])
    left, right = f.update(0.0, 0.0, math.pi / 2)
    assert left == -right and left != 0          # pure rotation


def test_drives_forward_when_aligned():
    f = WaypointFollower(waypoints=[(1.0, 0.0)])
    left, right = f.update(0.0, 0.0, 0.0)        # facing +x, waypoint at +x
    assert left > 0 and right > 0
    assert abs(left - right) <= 6                # near-straight


def test_correction_steers_toward_small_heading_error():
    f = WaypointFollower(waypoints=[(1.0, 0.0)])
    l_straight, r_straight = f.update(0.0, 0.0, 0.0)
    l_err, r_err = f.update(0.0, 0.0, 0.15)      # slightly rotated +theta
    assert (l_err - r_err) != (l_straight - r_straight)  # correction applied


def test_advances_through_waypoints_and_finishes():
    f = WaypointFollower(waypoints=[(0.5, 0.0), (0.5, 0.5)], arrive_dist=0.1)
    f.update(0.45, 0.0, 0.0)                     # within arrive_dist of wp0
    assert f.current_waypoint == (0.5, 0.5)
    assert f.update(0.5, 0.45, math.pi / 2) == (0, 0)  # reached the last one
    assert f.done
