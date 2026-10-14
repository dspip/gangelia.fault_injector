"""Behavior of the planner-independent gripper drop."""

from __future__ import annotations

import pytest
from gangelia.fault_injector.core import PANDA_ADAPTER, GripperAdapter, inject_drop, resolve_joint_indices


class RecordingDriver:
    """Records joint commands instead of talking to Isaac Sim."""

    def __init__(self) -> None:
        self.commands: list[tuple[tuple[str, ...], tuple[float, ...]]] = []

    def set_joint_positions(self, joint_names: tuple[str, ...], positions: tuple[float, ...]) -> None:
        self.commands.append((joint_names, positions))


def test_inject_drop_commands_panda_fingers_open() -> None:
    driver = RecordingDriver()

    result = inject_drop(driver, PANDA_ADAPTER)

    assert driver.commands == [(("panda_finger_joint1", "panda_finger_joint2"), (0.04, 0.04))]
    assert result.adapter_name == "Panda Hand"
    assert result.commanded is True


def test_adapter_rejects_mismatched_joint_and_position_counts() -> None:
    with pytest.raises(ValueError, match="one open position per joint"):
        GripperAdapter(name="Broken", joint_names=("left", "right"), open_positions=(0.04,))


def test_resolve_joint_indices_uses_articulation_dof_order() -> None:
    assert resolve_joint_indices(
        ("panda_joint1", "panda_finger_joint1", "panda_finger_joint2"),
        ("panda_finger_joint1", "panda_finger_joint2"),
    ) == (1, 2)


def test_resolve_joint_indices_reports_missing_joint() -> None:
    with pytest.raises(ValueError, match="missing gripper joint.*right_finger"):
        resolve_joint_indices(("left_finger",), ("left_finger", "right_finger"))
