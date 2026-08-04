import math

import pytest

from nav.scan import RotationTracker


def feed(tracker, headings):
    for h in headings:
        tracker.update(h)
    return tracker


def test_a_fresh_tracker_has_turned_nothing():
    t = RotationTracker(start=0.0)
    assert t.turned == 0.0
    assert not t.done


def test_small_steps_accumulate():
    t = feed(RotationTracker(start=0.0), [0.1, 0.2, 0.3])
    assert t.turned == pytest.approx(0.3)


def test_rotation_survives_the_pi_wrap():
    # Crossing +pi to -pi is a small step forward, not a huge step back.
    t = feed(RotationTracker(start=3.0), [3.1, -3.1, -3.0])
    assert t.turned == pytest.approx(0.2831853, abs=1e-5)


def test_a_full_turn_counts_as_a_full_turn_either_way():
    steps = [i * math.tau / 36 for i in range(1, 37)]
    forward = feed(RotationTracker(start=0.0), steps)
    backward = feed(RotationTracker(start=0.0), [-s for s in steps])
    assert abs(forward.turned) == pytest.approx(math.tau, abs=1e-6)
    assert abs(backward.turned) == pytest.approx(math.tau, abs=1e-6)


def test_done_once_the_target_is_reached():
    t = RotationTracker(start=0.0, target=math.pi)
    feed(t, [i * math.pi / 8 for i in range(1, 8)])
    assert not t.done
    t.update(math.pi + 0.01)
    assert t.done


def test_spinning_backwards_also_finishes():
    t = RotationTracker(start=0.0, target=math.pi)
    feed(t, [-i * math.pi / 8 for i in range(1, 10)])
    assert t.done


def test_wobbling_on_the_spot_never_completes_a_turn():
    # A car rocking back and forth must not accumulate a phantom rotation.
    t = RotationTracker(start=0.0, target=math.tau)
    for _ in range(50):
        t.update(0.2)
        t.update(-0.2)
    assert not t.done
    assert abs(t.turned) < 0.5


def test_progress_reports_a_fraction_of_the_target():
    t = RotationTracker(start=0.0, target=math.tau)
    t.update(math.pi)
    assert t.progress == pytest.approx(0.5)


def test_progress_is_clamped_once_past_the_target():
    # Stepped, not jumped: a single >180 deg step is genuinely ambiguous after
    # wrapping, which is why the tracker is fed continuously.
    t = RotationTracker(start=0.0, target=math.pi)
    feed(t, [i * math.pi / 8 for i in range(1, 13)])   # 1.5x the target
    assert t.progress == pytest.approx(1.0)


def test_a_single_step_larger_than_half_a_turn_is_ambiguous():
    # Documents a real limitation: sample fast enough that this cannot happen.
    t = RotationTracker(start=0.0, target=math.pi)
    t.update(math.pi * 1.5)          # 270 deg forward is read as 90 deg back
    assert t.turned == pytest.approx(-math.pi / 2)


def test_degrees_turned_is_reported_for_the_console():
    t = RotationTracker(start=0.0, target=math.tau)
    t.update(math.pi)
    assert t.degrees == pytest.approx(180.0)
