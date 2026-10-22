# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Isaac-independent drop injection behavior."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

# camera_noise.py is Pro only. A Community package is complete without it.
_BUNDLE_FILES = ("__init__.py", "catalog.py", "drop_rule.py")


def shared_faults_import_root(core_file: Path) -> Path:
    """Return the directory that must be on sys.path to import shared_faults.

    A release tree carries generated copies in ``<extension>/shared_faults``.
    Those copies win so a Community install does not need the Gangelia checkout.
    A checkout with no copies uses ``shared_faults/`` at the repository root.
    """
    extension_root = core_file.resolve().parents[2]
    bundled = extension_root / "shared_faults"
    if bundled.exists():
        missing = [name for name in _BUNDLE_FILES if not (bundled / name).is_file()]
        if missing:
            raise ImportError(
                "The shared_faults copy inside the extension is incomplete: "
                + ", ".join(missing)
                + f". Looked at {bundled}."
            )
        return extension_root

    repo_root = core_file.resolve().parents[4]
    catalog = repo_root / "shared_faults" / "catalog.py"
    if not catalog.is_file():
        raise ImportError(
            "Could not find shared_faults next to the isaac folder. "
            f"Looked at {catalog}."
        )
    return repo_root


def _load_shared_faults() -> None:
    """Put the shared_faults location for this install on sys.path."""
    root = str(shared_faults_import_root(Path(__file__)))
    if root not in sys.path:
        sys.path.insert(0, root)


_load_shared_faults()

from shared_faults.catalog import (  # noqa: E402
    DEMO_FAULT_NOTICE,
    FAULT_CHOICES,
    FaultChoice,
    fault_choices,
    fault_notice,
    fault_notice_for,
    fault_panel,
)
from shared_faults.drop_rule import AutomaticDrop, AutomaticDropConfig, CarrySample  # noqa: E402


class JointPositionDriver(Protocol):
    """Something that can command named articulation joints."""

    def set_joint_positions(self, joint_names: tuple[str, ...], positions: tuple[float, ...]) -> None:
        """Set the named joints to the given positions."""


@dataclass(frozen=True)
class GripperAdapter:
    """How one gripper is opened, without knowing which planner is moving the arm."""

    name: str
    joint_names: tuple[str, ...]
    open_positions: tuple[float, ...]

    def __post_init__(self) -> None:
        """Reject an adapter whose joints and open positions do not line up."""
        if len(self.joint_names) != len(self.open_positions):
            raise ValueError("gripper adapter requires one open position per joint")
        if not self.joint_names:
            raise ValueError("gripper adapter requires at least one joint")


@dataclass(frozen=True)
class ParallelGripperProfile:
    """One parallel gripper, in its own joint units.

    Closure runs from 0 at the open positions to 1 at the closed positions.
    A grasp can count before the jaws reach the closed stop, because the object
    blocks them. ``grasp_frame`` is a prim path when it starts with ``/``.
    Otherwise it is appended to the articulation path.
    """

    name: str
    grasp_frame: str
    joint_names: tuple[str, ...]
    open_positions: tuple[float, ...]
    closed_positions: tuple[float, ...]
    min_closure: float
    distance_threshold_m: float

    def __post_init__(self) -> None:
        """Reject a profile that cannot score a grasp or command an open."""
        if not self.grasp_frame.strip():
            raise ValueError("gripper profile requires a grasp frame")
        if not self.joint_names:
            raise ValueError("gripper profile requires at least one joint")
        if len(self.joint_names) != len(self.open_positions):
            raise ValueError("gripper profile requires one open position per joint")
        if len(self.joint_names) != len(self.closed_positions):
            raise ValueError("gripper profile requires one closed position per joint")
        if any(
            open_position == closed_position
            for open_position, closed_position in zip(self.open_positions, self.closed_positions, strict=True)
        ):
            raise ValueError("gripper profile requires different open and closed positions")
        if not 0.0 <= self.min_closure <= 1.0:
            raise ValueError("min_closure must be between 0 and 1")
        if self.distance_threshold_m <= 0:
            raise ValueError("distance_threshold_m must be positive")

    def closure(self, positions: tuple[float, ...]) -> float:
        """Return the mean travel from open toward closed for these joint positions."""
        if len(positions) != len(self.joint_names):
            raise ValueError("gripper sample does not match the adapter joints")
        scores = [
            (open_position - position) / (open_position - closed_position)
            for position, open_position, closed_position in zip(
                positions, self.open_positions, self.closed_positions, strict=True
            )
        ]
        return sum(scores) / len(scores)

    def grasp_frame_path(self, articulation_path: str) -> str:
        """Return the world prim used to measure distance to the object."""
        frame = self.grasp_frame.strip()
        if frame.startswith("/"):
            return frame
        return f"{articulation_path.rstrip('/')}/{frame}"

    def as_adapter(self) -> GripperAdapter:
        """Return the open command for this profile."""
        return GripperAdapter(
            name=self.name, joint_names=self.joint_names, open_positions=self.open_positions
        )


PANDA_PROFILE = ParallelGripperProfile(
    name="Panda Hand",
    grasp_frame="panda_hand",
    joint_names=("panda_finger_joint1", "panda_finger_joint2"),
    open_positions=(0.04, 0.04),
    closed_positions=(0.0, 0.0),
    min_closure=0.12,
    distance_threshold_m=0.15,
)
ROBOTIQ_2F85_PROFILE = ParallelGripperProfile(
    name="Robotiq 2F-85",
    grasp_frame="base_link",
    joint_names=("finger_joint",),
    open_positions=(0.0,),
    closed_positions=(math.radians(75),),
    min_closure=0.10,
    distance_threshold_m=0.20,
)
PRESETS: tuple[ParallelGripperProfile, ...] = (PANDA_PROFILE, ROBOTIQ_2F85_PROFILE)
CUSTOM_PROFILE_INDEX = len(PRESETS)
PANDA_ADAPTER = PANDA_PROFILE.as_adapter()


def parse_parallel_gripper_profile(
    name: str,
    grasp_frame: str,
    joint_names: str,
    open_positions: str,
    closed_positions: str,
    min_closure: str,
    distance_threshold_m: str,
) -> ParallelGripperProfile:
    """Build a profile from the custom-gripper text fields."""
    return ParallelGripperProfile(
        name=name,
        grasp_frame=grasp_frame,
        joint_names=_comma_strings(joint_names),
        open_positions=_comma_floats(open_positions),
        closed_positions=_comma_floats(closed_positions),
        min_closure=float(min_closure),
        distance_threshold_m=float(distance_threshold_m),
    )


def profile_for_selection(index: int, custom: ParallelGripperProfile | None) -> ParallelGripperProfile:
    """Return a preset, or the custom profile when that row is selected."""
    if index == CUSTOM_PROFILE_INDEX:
        if custom is None:
            raise ValueError("custom gripper profile is incomplete")
        return custom
    try:
        return PRESETS[index]
    except IndexError:
        raise ValueError("unknown gripper profile") from None


def _comma_strings(text: str) -> tuple[str, ...]:
    values = tuple(part.strip() for part in text.split(",") if part.strip())
    if not values:
        raise ValueError("gripper profile requires at least one joint")
    return values


def _comma_floats(text: str) -> tuple[float, ...]:
    try:
        return tuple(float(part.strip()) for part in text.split(",") if part.strip())
    except ValueError as exc:
        raise ValueError("joint positions must be numbers") from exc


@dataclass(frozen=True)
class DropInjectionResult:
    """Record of one drop command."""

    adapter_name: str
    commanded: bool


def resolve_joint_indices(
    articulation_dof_names: tuple[str, ...],
    gripper_joint_names: tuple[str, ...],
) -> tuple[int, ...]:
    """Map adapter joint names onto an articulation's degree-of-freedom order.

    Args:
        articulation_dof_names: Joint names in the articulation's control order.
        gripper_joint_names: Joint names from a gripper adapter.

    Returns:
        Indices of the gripper joints inside ``articulation_dof_names``.
    """
    available = {name: index for index, name in enumerate(articulation_dof_names)}
    missing = [name for name in gripper_joint_names if name not in available]
    if missing:
        raise ValueError(f"articulation is missing gripper joint(s): {', '.join(missing)}")
    return tuple(available[name] for name in gripper_joint_names)



def inject_drop(driver: JointPositionDriver, adapter: GripperAdapter) -> DropInjectionResult:
    """Command a configured gripper to its open position.

    Args:
        driver: Live or test joint-position driver.
        adapter: Gripper whose fingers should open.

    Returns:
        A result naming the adapter that was commanded.
    """
    driver.set_joint_positions(adapter.joint_names, adapter.open_positions)
    return DropInjectionResult(adapter_name=adapter.name, commanded=True)
