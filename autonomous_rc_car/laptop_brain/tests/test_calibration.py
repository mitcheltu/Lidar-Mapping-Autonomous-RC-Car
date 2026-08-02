import math

import pytest

from nav.calibration import (
    Calibration,
    detect_deadband,
    fit_gain,
    latency_from_step,
    trim_from_drift,
)


# --- applying a calibration to a motor command ----------------------------

def test_zero_command_stays_zero():
    # The car must never creep while stopped, whatever the deadband is.
    c = Calibration(drive_deadband=30)
    assert c.apply(0, 0) == (0, 0)


def test_small_command_is_lifted_over_the_deadband():
    c = Calibration(drive_deadband=30)
    left, right = c.apply(1, 1)
    assert left >= 30 and right >= 30


def test_full_scale_command_is_unchanged():
    c = Calibration(drive_deadband=30)
    assert c.apply(100, 100) == (100, 100)


def test_negative_command_keeps_its_sign():
    c = Calibration(drive_deadband=30)
    left, right = c.apply(-5, -5)
    assert left <= -30 and right <= -30


def test_trim_biases_the_two_sides_apart():
    c = Calibration(straightness_trim=0.1)
    left, right = c.apply(50, 50)
    assert left > right


def test_output_is_clamped_to_the_motor_range():
    c = Calibration(drive_deadband=30, straightness_trim=0.5)
    left, right = c.apply(100, 100)
    assert -100 <= left <= 100 and -100 <= right <= 100


# --- deadband detection ---------------------------------------------------

def test_deadband_is_the_first_unit_that_actually_moves():
    samples = [(10, 0.001), (20, 0.002), (30, 0.05), (40, 0.09)]
    assert detect_deadband(samples, threshold=0.03) == 30


def test_deadband_is_none_when_the_car_never_moved():
    samples = [(10, 0.001), (20, 0.002)]
    assert detect_deadband(samples, threshold=0.03) is None


# --- gain fitting ---------------------------------------------------------

def test_gain_fit_recovers_slope_and_implied_deadband():
    # rate = 0.01 * (unit - 20)
    samples = [(30, 0.10), (50, 0.30), (70, 0.50)]
    gain, deadband = fit_gain(samples)
    assert gain == pytest.approx(0.01, abs=1e-6)
    assert deadband == pytest.approx(20.0, abs=1e-6)


def test_gain_fit_needs_at_least_two_points():
    with pytest.raises(ValueError):
        fit_gain([(30, 0.1)])


def test_gain_fit_rejects_a_flat_response():
    # Every speed produced the same rate -> no usable slope.
    with pytest.raises(ValueError):
        fit_gain([(30, 0.2), (50, 0.2), (70, 0.2)])


# --- straightness ---------------------------------------------------------

def test_no_drift_means_no_trim():
    assert trim_from_drift(0.0, distance=1.5, turn_sign=1) == 0.0


def test_drift_produces_a_trim_that_opposes_it():
    left_drift = trim_from_drift(0.2, distance=1.5, turn_sign=1)
    right_drift = trim_from_drift(-0.2, distance=1.5, turn_sign=1)
    assert left_drift > 0 > right_drift
    assert left_drift == pytest.approx(-right_drift)


def test_trim_follows_the_turn_sign_convention():
    positive = trim_from_drift(0.2, distance=1.5, turn_sign=1)
    negative = trim_from_drift(0.2, distance=1.5, turn_sign=-1)
    assert positive == pytest.approx(-negative)


def test_trim_is_bounded_even_for_a_violent_curve():
    assert abs(trim_from_drift(math.pi, distance=0.1, turn_sign=1)) <= 0.5


# --- latency --------------------------------------------------------------

def test_latency_is_the_delay_before_the_car_reaches_a_fifth_of_full_speed():
    # Commanded at t=0; still at rest until 0.2 s, then ramps to 0.5 m/s.
    trace = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0), (0.3, 0.25), (0.4, 0.5), (0.5, 0.5)]
    assert latency_from_step(trace, command_time=0.0) == pytest.approx(0.3)


def test_latency_is_none_when_the_car_never_moved():
    trace = [(0.0, 0.0), (0.1, 0.0), (0.2, 0.0)]
    assert latency_from_step(trace, command_time=0.0) is None


# --- persistence ----------------------------------------------------------

def test_calibration_survives_a_save_load_roundtrip(tmp_path):
    path = tmp_path / "calibration.yaml"
    original = Calibration(
        turn_sign=-1,
        drive_deadband=28,
        turn_deadband=24,
        linear_gain=0.0042,
        angular_gain=0.0135,
        straightness_trim=-0.03,
        command_latency=0.18,
    )
    original.save(path)
    assert Calibration.load(path) == original


def test_loading_a_missing_file_gives_usable_defaults(tmp_path):
    c = Calibration.load(tmp_path / "nope.yaml")
    assert c.turn_sign == 1
    assert c.drive_deadband == 0
    assert c.apply(50, 50) == (50, 50)      # defaults must not distort commands


def test_saving_records_when_the_calibration_was_taken(tmp_path):
    path = tmp_path / "calibration.yaml"
    Calibration().save(path)
    assert Calibration.load(path).calibrated_at != ""
