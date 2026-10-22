# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Regrasp decisions that do not need a GPU or Isaac Sim."""

from __future__ import annotations

import math

from gangelia.fault_injector.regrasp import (
    PANDA_HAND_TO_PINCH_M,
    RegraspConfig,
    RegraspSequence,
    grasp_pinch_orientation,
    object_clear_of_hand,
    pinch_from_panda_hand,
    resume_drop_watch,
)


def _config() -> RegraspConfig:
    return RegraspConfig(still_s=0.3, max_move_s=0.4, close_s=0.2)


def _arm_after_drop() -> RegraspSequence:
    sequence = RegraspSequence(_config())
    sequence.note_drop_released()
    assert sequence.arm() is None
    return sequence


def test_pinch_is_ten_centimetres_along_the_hand_z_axis() -> None:
    """right_gripper sits 0.10 m along panda_link8 +Z, and panda_hand shares that origin."""
    position, orientation = pinch_from_panda_hand((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0))

    assert PANDA_HAND_TO_PINCH_M == 0.10
    assert position == (1.0, 2.0, 3.10)
    # 180 degrees about Z. A parallel gripper is the same grasp with the fingers swapped.
    assert orientation == (0.0, 0.0, 0.0, 1.0)


def test_pinch_follows_a_rotated_hand_axis() -> None:
    half = math.sqrt(0.5)
    position, _orientation = pinch_from_panda_hand((0.0, 0.0, 0.0), (half, half, 0.0, 0.0))

    assert math.isclose(position[0], 0.0, abs_tol=1e-12)
    assert math.isclose(position[1], -0.10, abs_tol=1e-12)
    assert math.isclose(position[2], 0.0, abs_tol=1e-12)


def test_press_before_the_drop_finishes_does_not_move() -> None:
    sequence = RegraspSequence(_config())

    message = sequence.arm()
    command = sequence.step(dt_s=0.1, 
        object_position=(0.0, 0.0, 0.02),
        pinch_position=(0.0, 0.0, 0.20),
        hand_position=(0.0, 0.0, 0.10),
        distance_threshold_m=0.15,
    )

    assert message == "Regrasp waits until the object has left the gripper."
    assert command.phase == "idle"
    assert command.target_position is None
    assert command.fingers_closed is None


def _rotx(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) / 2.0
    return (math.cos(half), math.sin(half), 0.0, 0.0)


def _z_axis(quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = quat
    return (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


def _x_axis(quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = quat
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + z * w),
        2.0 * (x * z - y * w),
    )


def test_cube_already_clear_of_the_hand_can_start() -> None:
    sequence = RegraspSequence(_config())
    assert object_clear_of_hand((0.4, 0.2, 0.02), (0.0, 0.0, 0.4), 0.15) is True
    assert object_clear_of_hand((0.0, 0.0, 0.2), (0.0, 0.0, 0.2), 0.15) is False

    assert sequence.arm(object_clear=True) is None
    command = sequence.step(dt_s=0.1, 
        object_position=(0.4, 0.2, 0.4),
        pinch_position=(0.0, 0.0, 0.5),
        hand_position=(0.0, 0.0, 0.4),
        distance_threshold_m=0.15,
    )
    assert command.phase == "settle"
    assert command.fingers_closed is False


def test_downward_wrist_is_kept_and_a_sideways_wrist_points_down() -> None:
    down = _rotx(180.0)
    kept = grasp_pinch_orientation(down, (0.0, 0.0), (1.0, 0.0))
    assert abs(sum(a * b for a, b in zip(kept, down))) > 0.999

    tilted = grasp_pinch_orientation(_rotx(135.0), (0.0, 0.0), (0.0, 1.0))
    zx, zy, zz = _z_axis(tilted)
    assert math.isclose(zx, 0.0, abs_tol=1e-9)
    assert math.isclose(zy, 0.0, abs_tol=1e-9)
    assert math.isclose(zz, -1.0, abs_tol=1e-9)
    xx, xy, xz = _x_axis(tilted)
    assert math.isclose(xx, 0.0, abs_tol=1e-9)
    assert math.isclose(xy, 1.0, abs_tol=1e-9)
    assert math.isclose(xz, 0.0, abs_tol=1e-9)


def test_object_above_the_base_aims_along_world_x() -> None:
    quat = grasp_pinch_orientation(_rotx(135.0), (1.0, 2.0), (1.0, 2.0))
    zx, zy, zz = _z_axis(quat)
    assert math.isclose(zx, 0.0, abs_tol=1e-9)
    assert math.isclose(zy, 0.0, abs_tol=1e-9)
    assert math.isclose(zz, -1.0, abs_tol=1e-9)
    xx, xy, xz = _x_axis(quat)
    assert math.isclose(xx, 1.0, abs_tol=1e-9)
    assert math.isclose(xy, 0.0, abs_tol=1e-9)
    assert math.isclose(xz, 0.0, abs_tol=1e-9)


def test_automatic_regrasp_does_not_resume_the_drop_watch() -> None:
    assert resume_drop_watch(automatic_regrasp=True, drop_enabled=True) is False
    assert resume_drop_watch(automatic_regrasp=False, drop_enabled=True) is True
    assert resume_drop_watch(automatic_regrasp=False, drop_enabled=False) is False


def test_press_after_release_waits_for_the_object_to_leave_and_stop() -> None:
    sequence = _arm_after_drop()
    falling = (0.2, 0.0, 0.40)
    hand = (0.2, 0.0, 0.50)

    command = sequence.step(dt_s=0.1, 
        object_position=falling,
        pinch_position=(0.2, 0.0, 0.60),
        hand_position=hand,
        distance_threshold_m=0.15,
    )

    assert command.phase == "settle"
    assert command.fingers_closed is False
    assert command.target_position is None
    assert sequence.active is True


def test_settled_object_outside_the_hand_asks_for_a_hover() -> None:
    sequence = _arm_after_drop()
    landed = (0.2, 0.1, 0.02)
    hand = (0.2, 0.1, 0.40)
    for _ in range(2):
        command = sequence.step(dt_s=0.1, 
            object_position=landed,
            pinch_position=(0.0, 0.0, 1.0),
            hand_position=hand,
            distance_threshold_m=0.15,
        )
        assert command.phase == "settle"

    command = sequence.step(dt_s=0.1, 
        object_position=landed,
        pinch_position=(0.0, 0.0, 1.0),
        hand_position=hand,
        distance_threshold_m=0.15,
    )

    assert command.phase == "hover"
    assert command.fingers_closed is False
    assert command.target_position == (landed[0], landed[1], landed[2] + 0.10)


def test_object_that_never_leaves_the_hand_stops() -> None:
    sequence = _arm_after_drop()
    held = (0.0, 0.0, 0.20)
    hand = (0.0, 0.0, 0.20)
    command = None
    for _ in range(5):
        command = sequence.step(
            dt_s=0.1,
            object_position=held,
            pinch_position=(0.0, 0.0, 0.30),
            hand_position=hand,
            distance_threshold_m=0.15,
        )

    assert command is not None
    assert command.phase == "failed"
    assert command.fingers_closed is None
    assert command.reason == "Regrasp stopped: the object did not leave the gripper."
    assert sequence.active is False


def test_object_still_in_the_hand_does_not_hover() -> None:
    sequence = _arm_after_drop()
    held = (0.0, 0.0, 0.20)
    hand = (0.0, 0.0, 0.20)
    command = None
    for _ in range(3):
        command = sequence.step(dt_s=0.1, 
            object_position=held,
            pinch_position=(0.0, 0.0, 0.30),
            hand_position=hand,
            distance_threshold_m=0.15,
        )

    assert command is not None
    assert command.phase == "settle"


def test_hover_then_descend_then_close_then_lift() -> None:
    sequence = _arm_after_drop()
    landed = (0.2, 0.0, 0.02)
    hand = (0.2, 0.0, 0.40)
    for _ in range(3):
        sequence.step(dt_s=0.1, 
            object_position=landed,
            pinch_position=(0.0, 0.0, 1.0),
            hand_position=hand,
            distance_threshold_m=0.15,
        )

    hover = sequence.step(dt_s=0.1, 
        object_position=landed,
        pinch_position=(0.2, 0.0, 0.12),
        hand_position=hand,
        distance_threshold_m=0.15,
    )
    assert hover.phase == "descend"
    assert hover.target_position == landed
    assert hover.fingers_closed is False

    closing = sequence.step(dt_s=0.1, 
        object_position=landed,
        pinch_position=landed,
        hand_position=hand,
        distance_threshold_m=0.15,
    )
    assert closing.phase == "close"
    assert closing.fingers_closed is True
    assert closing.target_position == landed

    jumped = (5.0, 5.0, 5.0)
    held_shut = sequence.step(dt_s=0.1, 
        object_position=jumped,
        pinch_position=landed,
        hand_position=hand,
        distance_threshold_m=0.15,
    )
    assert held_shut.phase == "close"
    assert held_shut.target_position == landed

    lifting = sequence.step(dt_s=0.1, 
        object_position=jumped,
        pinch_position=landed,
        hand_position=hand,
        distance_threshold_m=0.15,
    )
    assert lifting.phase == "lift"
    assert lifting.fingers_closed is True
    assert lifting.target_position == (landed[0], landed[1], landed[2] + 0.10)

    finished = sequence.step(dt_s=0.1, 
        object_position=jumped,
        pinch_position=(0.2, 0.0, 0.12),
        hand_position=(0.2, 0.0, 0.02),
        distance_threshold_m=0.15,
    )
    assert finished.phase == "done"
    assert finished.fingers_closed is None
    assert sequence.active is False


def test_a_second_press_does_not_restart_the_motion() -> None:
    sequence = _arm_after_drop()
    sequence.step(dt_s=0.1, 
        object_position=(0.0, 0.0, 0.4),
        pinch_position=(0.0, 0.0, 0.5),
        hand_position=(0.0, 0.0, 0.4),
        distance_threshold_m=0.15,
    )

    assert sequence.arm() is None
    command = sequence.step(dt_s=0.1, 
        object_position=(0.0, 0.0, 0.2),
        pinch_position=(0.0, 0.0, 0.5),
        hand_position=(0.0, 0.0, 0.4),
        distance_threshold_m=0.15,
    )
    assert command.phase == "settle"


def test_missing_the_target_stops_the_regrasp() -> None:
    sequence = _arm_after_drop()
    landed = (0.2, 0.0, 0.02)
    for _ in range(3):
        sequence.step(dt_s=0.1, 
            object_position=landed,
            pinch_position=(0.0, 0.0, 1.0),
            hand_position=(0.2, 0.0, 0.40),
            distance_threshold_m=0.15,
        )

    command = None
    for _ in range(4):
        command = sequence.step(dt_s=0.1, 
            object_position=landed,
            pinch_position=(9.0, 9.0, 9.0),
            hand_position=(0.2, 0.0, 0.40),
            distance_threshold_m=0.15,
        )

    assert command is not None
    assert command.phase == "failed"
    assert command.reason == "Regrasp stopped: the gripper did not reach the object."
    assert sequence.active is False
