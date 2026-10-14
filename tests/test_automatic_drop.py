"""Automatic pickup detection and per-step drop probability."""

from __future__ import annotations

import pytest
from gangelia.fault_injector.core import PANDA_PROFILE, AutomaticDrop, AutomaticDropConfig, CarrySample


class ScriptedRandom:
    """Returns queued values from ``random()``."""

    def __init__(self, values: tuple[float, ...]) -> None:
        self._values = iter(values)

    def random(self) -> float:
        return next(self._values)


def _config(**overrides: float | int) -> AutomaticDropConfig:
    values: dict[str, float | int] = {
        "min_lift_m": 0.03,
        "together_tolerance_m": 0.01,
        "confirm_steps": 3,
        "probability_per_step": 1.0,
        "latch_steps": 2,
    }
    values.update(overrides)
    return AutomaticDropConfig(**values)


def _sample(
    object_z: float,
    gripper_z: float,
    finger: float,
    *,
    object_xy: tuple[float, float] = (0.0, 0.0),
    gripper_xy: tuple[float, float] = (0.0, 0.0),
) -> CarrySample:
    return CarrySample(
        object_position=(object_xy[0], object_xy[1], object_z),
        gripper_position=(gripper_xy[0], gripper_xy[1], gripper_z),
        gripper_joint_positions=(finger, finger),
    )


def _carry_until_armed(drop: AutomaticDrop) -> list[bool]:
    """Table sample, approach, then stable frames until pickup is confirmed."""
    opened: list[bool] = []
    opened.append(drop.observe(_sample(0.03, 0.20, 0.04)))
    opened.append(drop.observe(_sample(0.08, 0.08, 0.0)))
    for _ in range(3):
        opened.append(drop.observe(_sample(0.10, 0.10, 0.0)))
    return opened


def test_distance_without_a_closed_lifted_grasp_does_not_open() -> None:
    drop = AutomaticDrop(_config(), PANDA_PROFILE, ScriptedRandom((0.0,)))

    assert drop.observe(_sample(0.03, 0.03, 0.04)) is False
    assert drop.observe(_sample(0.03, 0.03, 0.0)) is False


def test_changing_gap_does_not_count_as_moving_together() -> None:
    drop = AutomaticDrop(_config(), PANDA_PROFILE, ScriptedRandom((0.0,)))
    drop.observe(_sample(0.03, 0.20, 0.04))

    for x in (0.05, 0.03, 0.01):
        assert drop.observe(_sample(0.10, 0.10, 0.0, gripper_xy=(x, 0.0))) is False

    assert drop.phase == "watching"


def test_stable_carry_then_certain_probability_holds_gripper_open() -> None:
    drop = AutomaticDrop(_config(), PANDA_PROFILE, ScriptedRandom((0.0,)))

    opened = _carry_until_armed(drop)

    assert opened[:-1] == [False, False, False, False]
    assert opened[-1] is True
    assert drop.observe(_sample(0.10, 0.10, 0.0)) is True
    assert drop.observe(_sample(0.10, 0.10, 0.0)) is False
    assert drop.phase == "done"


def test_without_rearm_the_detector_stays_done_after_the_object_falls() -> None:
    drop = AutomaticDrop(_config(), PANDA_PROFILE, ScriptedRandom((0.0, 0.0)))
    _carry_until_armed(drop)
    drop.observe(_sample(0.10, 0.10, 0.0))

    assert drop.observe(_sample(0.03, 0.40, 0.04)) is False
    assert drop.phase == "done"


def test_rearm_watches_again_once_the_object_leaves_the_jaws() -> None:
    drop = AutomaticDrop(_config(rearm_after_drop=True), PANDA_PROFILE, ScriptedRandom((0.0, 0.0)))
    _carry_until_armed(drop)
    drop.observe(_sample(0.10, 0.10, 0.0))
    assert drop.phase == "done"

    assert drop.observe(_sample(0.10, 0.12, 0.0)) is False
    assert drop.phase == "done"
    assert drop.observe(_sample(0.03, 0.40, 0.04)) is False
    assert drop.phase == "watching"

    assert _carry_until_armed(drop)[-1] is True
    assert drop.phase == "latching"


def test_rest_height_is_the_lowest_height_seen_while_watching() -> None:
    drop = AutomaticDrop(_config(), PANDA_PROFILE, ScriptedRandom((0.0,)))
    drop.observe(_sample(0.20, 0.60, 0.04))
    drop.observe(_sample(0.03, 0.60, 0.04))

    opened = [drop.observe(_sample(0.10, 0.10, 0.0)) for _ in range(4)]

    assert opened[-1] is True


def test_zero_probability_never_opens_after_pickup() -> None:
    drop = AutomaticDrop(
        _config(probability_per_step=0.0),
        PANDA_PROFILE,
        ScriptedRandom((0.0, 0.0, 0.0)),
    )

    assert all(command is False for command in _carry_until_armed(drop))
    assert drop.observe(_sample(0.10, 0.10, 0.0)) is False
    assert drop.phase == "armed"


def test_probability_is_rolled_once_per_armed_step() -> None:
    drop = AutomaticDrop(
        _config(probability_per_step=0.25),
        PANDA_PROFILE,
        ScriptedRandom((0.5, 0.1)),
    )

    assert _carry_until_armed(drop)[-1] is False
    assert drop.observe(_sample(0.10, 0.10, 0.0)) is True


def test_sample_with_the_wrong_finger_count_is_rejected() -> None:
    drop = AutomaticDrop(_config(), PANDA_PROFILE, ScriptedRandom((0.0,)))
    sample = CarrySample(
        object_position=(0.0, 0.0, 0.03),
        gripper_position=(0.0, 0.0, 0.03),
        gripper_joint_positions=(0.0,),
    )

    with pytest.raises(ValueError, match="adapter joints"):
        drop.observe(sample)


def test_probability_outside_zero_to_one_is_rejected() -> None:
    with pytest.raises(ValueError, match="probability_per_step"):
        _config(probability_per_step=1.1)
