"""Parallel-gripper presets and custom profile parsing."""

from __future__ import annotations

import math

import pytest
from gangelia.fault_injector.core import (
    PANDA_PROFILE,
    ROBOTIQ_2F85_PROFILE,
    ParallelGripperProfile,
    parse_parallel_gripper_profile,
    profile_for_selection,
)


def test_panda_cube_grasp_is_closed_enough_before_the_stop() -> None:
    assert PANDA_PROFILE.closure((0.035, 0.035)) >= PANDA_PROFILE.min_closure
    assert PANDA_PROFILE.closure((0.036, 0.036)) < PANDA_PROFILE.min_closure


def test_robotiq_cube_grasp_is_closed_enough() -> None:
    assert ROBOTIQ_2F85_PROFILE.joint_names == ("finger_joint",)
    assert ROBOTIQ_2F85_PROFILE.closure((0.174,)) >= ROBOTIQ_2F85_PROFILE.min_closure
    assert ROBOTIQ_2F85_PROFILE.closure((0.0,)) < ROBOTIQ_2F85_PROFILE.min_closure
    assert ROBOTIQ_2F85_PROFILE.closed_positions[0] == pytest.approx(math.radians(75))


def test_grasp_frame_is_a_suffix_unless_it_is_absolute() -> None:
    assert PANDA_PROFILE.grasp_frame_path("/World/Franka") == "/World/Franka/panda_hand"
    assert ROBOTIQ_2F85_PROFILE.grasp_frame_path("/World/Robotiq") == "/World/Robotiq/base_link"
    custom = ParallelGripperProfile(
        name="Custom",
        grasp_frame="/World/Arm/tool0",
        joint_names=("jaw",),
        open_positions=(0.0,),
        closed_positions=(1.0,),
        min_closure=0.1,
        distance_threshold_m=0.2,
    )
    assert custom.grasp_frame_path("/World/Arm") == "/World/Arm/tool0"


def test_profile_rejects_a_joint_that_does_not_move() -> None:
    with pytest.raises(ValueError, match="open and closed"):
        ParallelGripperProfile(
            name="Stuck",
            grasp_frame="tool0",
            joint_names=("jaw",),
            open_positions=(0.0,),
            closed_positions=(0.0,),
            min_closure=0.1,
            distance_threshold_m=0.2,
        )


def test_custom_text_becomes_a_profile() -> None:
    profile = parse_parallel_gripper_profile(
        name="Custom",
        grasp_frame="tool0",
        joint_names="left, right",
        open_positions="0, 0",
        closed_positions="1, -1",
        min_closure="0.2",
        distance_threshold_m="0.25",
    )

    assert profile.closure((0.0, 0.0)) == pytest.approx(0.0)
    assert profile.closure((1.0, -1.0)) == pytest.approx(1.0)
    assert profile.distance_threshold_m == 0.25


def test_custom_text_rejects_mismatched_counts() -> None:
    with pytest.raises(ValueError, match="one open position per joint"):
        parse_parallel_gripper_profile(
            name="Custom",
            grasp_frame="tool0",
            joint_names="left, right",
            open_positions="0",
            closed_positions="1, 1",
            min_closure="0.2",
            distance_threshold_m="0.25",
        )


def test_selection_uses_the_preset_unless_custom_is_chosen() -> None:
    custom = PANDA_PROFILE
    assert profile_for_selection(0, None) is PANDA_PROFILE
    assert profile_for_selection(1, None) is ROBOTIQ_2F85_PROFILE
    assert profile_for_selection(2, custom) is custom
