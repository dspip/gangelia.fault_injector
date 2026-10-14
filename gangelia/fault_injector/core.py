# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Isaac-independent drop injection behavior."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol


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
DEMO_FAULT_NOTICE = "Demo Version - Contact Gangelia"


@dataclass(frozen=True)
class FaultChoice:
    """One row in the fault list. Unavailable rows show the demo notice."""

    name: str
    available: bool


FAULT_CHOICES: tuple[FaultChoice, ...] = (
    FaultChoice("Drop", True),
    FaultChoice("Object Over Object", False),
    FaultChoice("Camera Noise", False),
    FaultChoice("Object Collision", False),
)


def fault_notice(index: int) -> str | None:
    """Return the demo notice for a locked fault, or None when that fault runs."""
    choice = FAULT_CHOICES[index]
    if choice.available:
        return None
    return DEMO_FAULT_NOTICE


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


@dataclass(frozen=True)
class AutomaticDropConfig:
    """Pickup detector and per-step drop probability.

    Probability is rolled once per physics step after pickup is confirmed.
    It is not a rate per second, so a faster physics rate makes drops more frequent.
    With ``rearm_after_drop``, a finished drop watches for the next pickup once the
    object is farther from the grasp frame than the profile distance.
    """

    min_lift_m: float = 0.03
    together_tolerance_m: float = 0.01
    confirm_steps: int = 6
    probability_per_step: float = 0.0001
    latch_steps: int = 30
    rearm_after_drop: bool = False

    def __post_init__(self) -> None:
        """Reject a configuration that cannot describe a pickup or a drop."""
        if self.min_lift_m <= 0:
            raise ValueError("min_lift_m must be positive")
        if self.together_tolerance_m < 0:
            raise ValueError("together_tolerance_m must be non-negative")
        if self.confirm_steps < 1:
            raise ValueError("confirm_steps must be at least 1")
        if not 0.0 <= self.probability_per_step <= 1.0:
            raise ValueError("probability_per_step must be between 0 and 1")
        if self.latch_steps < 1:
            raise ValueError("latch_steps must be at least 1")


@dataclass(frozen=True)
class CarrySample:
    """One physics-step view of the target object and the gripper."""

    object_position: tuple[float, float, float]
    gripper_position: tuple[float, float, float]
    gripper_joint_positions: tuple[float, ...]


class AutomaticDrop:
    """Confirm a geometric pickup, then roll a drop on every later physics step.

    Pickup requires the named object to be near the grasp frame, the jaws past the
    profile's closure threshold, the object lifted from its first observed height,
    and the gap stable.
    After confirmation, each step draws one random number. A hit commands the
    gripper open for ``latch_steps`` consecutive steps, then stops.
    """

    def __init__(self, config: AutomaticDropConfig, profile: ParallelGripperProfile, rng: object) -> None:
        """Store the detector settings and the random source used for each step."""
        self._profile = profile
        self._config = config
        self._rng = rng
        self._phase = "watching"
        self._confirm = 0
        self._latch_left = 0
        self._rest_z: float | None = None
        self._previous_gap: float | None = None

    @property
    def phase(self) -> str:
        """Current detector phase: watching, armed, latching, or done."""
        return self._phase

    def observe(self, sample: CarrySample) -> bool:
        """Return whether this step should command the gripper open.

        Args:
            sample: Object position, gripper position, and finger joint positions.

        Returns:
            True while the open command must be reasserted.
        """
        if self._phase == "done":
            if self._config.rearm_after_drop and (
                _distance(sample.object_position, sample.gripper_position)
                > self._profile.distance_threshold_m
            ):
                self._phase = "watching"
                self._confirm = 0
                self._rest_z = None
                self._previous_gap = None
            return False
        if self._phase == "latching":
            self._latch_left -= 1
            if self._latch_left <= 0:
                self._phase = "done"
            return True

        gap = _distance(sample.object_position, sample.gripper_position)
        held = self._is_held(sample, gap)
        self._previous_gap = gap
        if self._phase == "watching":
            self._confirm = self._confirm + 1 if held else 0
            if self._confirm < self._config.confirm_steps:
                return False
            self._phase = "armed"
        elif not held and gap > self._profile.distance_threshold_m:
            self._phase = "watching"
            self._confirm = 0
            return False

        if float(self._rng.random()) < self._config.probability_per_step:
            self._phase = "latching"
            self._latch_left = self._config.latch_steps - 1
            if self._latch_left <= 0:
                self._phase = "done"
            return True
        return False

    def _is_held(self, sample: CarrySample, gap: float) -> bool:
        if self._rest_z is None:
            self._rest_z = sample.object_position[2]
        elif self._phase == "watching":
            self._rest_z = min(self._rest_z, sample.object_position[2])
        if len(sample.gripper_joint_positions) != len(self._profile.joint_names):
            raise ValueError("gripper sample does not match the adapter joints")
        if self._profile.closure(sample.gripper_joint_positions) < self._profile.min_closure:
            return False
        if gap > self._profile.distance_threshold_m:
            return False
        if sample.object_position[2] < self._rest_z + self._config.min_lift_m:
            return False
        if self._previous_gap is None:
            return False
        return abs(gap - self._previous_gap) <= self._config.together_tolerance_m


def _distance(start: tuple[float, float, float], end: tuple[float, float, float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(start, end, strict=True)))


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
