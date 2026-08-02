import pytest

from nav import config
from nav.calibration import Calibration
from nav.controller import WaypointFollower


@pytest.fixture(autouse=True)
def restore_config():
    """Each test mutates module state; put it back afterwards."""
    yield
    config.reload_calibration()


def test_reload_picks_up_a_saved_calibration(tmp_path, monkeypatch):
    path = tmp_path / "calibration.yaml"
    Calibration(turn_sign=-1, drive_deadband=25).save(path)
    monkeypatch.setenv("RC_CAR_CALIBRATION", str(path))

    config.reload_calibration()

    assert config.TURN_SIGN == -1
    assert config.CALIBRATION.drive_deadband == 25


def test_missing_calibration_file_leaves_driving_untouched(tmp_path, monkeypatch):
    monkeypatch.setenv("RC_CAR_CALIBRATION", str(tmp_path / "absent.yaml"))

    config.reload_calibration()

    assert config.TURN_SIGN == 1
    assert config.CALIBRATION.apply(50, 50) == (50, 50)


def test_uncalibrated_controller_stops_at_the_plain_arrive_distance():
    f = WaypointFollower(waypoints=[(1.0, 0.0)], arrive_dist=0.1,
                         calibration=Calibration())
    assert f.stop_distance == pytest.approx(0.1)


def test_latency_pushes_the_stop_distance_out_by_one_command_of_travel():
    # 0.005 m/s per unit * 40 units * 0.2 s = 0.04 m of travel before it reacts.
    f = WaypointFollower(
        waypoints=[(1.0, 0.0)], arrive_dist=0.1, drive_speed=40,
        calibration=Calibration(linear_gain=0.005, command_latency=0.2),
    )
    assert f.stop_distance == pytest.approx(0.14)


def test_controller_finishes_early_by_its_stop_distance():
    f = WaypointFollower(
        waypoints=[(1.0, 0.0)], arrive_dist=0.1, drive_speed=40,
        calibration=Calibration(linear_gain=0.005, command_latency=0.2),
    )
    # 0.87 m along: 0.13 m short, inside the 0.14 m stop distance.
    assert f.update(0.87, 0.0, 0.0) == (0, 0)
    assert f.done
