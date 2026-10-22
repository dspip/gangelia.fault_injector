# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file in isaac/gangelia.fault_injector.

"""Decide when a carried object should be dropped.

This file does not open a gripper. Isaac and LIBERO each do that themselves.
The Isaac window is the caller today. LIBERO datagen keeps its own basket rule.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


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


class GripperClosure:
    """The few gripper facts the drop rule needs. Isaac profiles provide these."""

    def __init__(
        self,
        joint_names: tuple[str, ...],
        open_positions: tuple[float, ...],
        closed_positions: tuple[float, ...],
        min_closure: float,
        distance_threshold_m: float,
    ) -> None:
        self.joint_names = joint_names
        self.open_positions = open_positions
        self.closed_positions = closed_positions
        self.min_closure = min_closure
        self.distance_threshold_m = distance_threshold_m

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


class AutomaticDrop:
    """Confirm a geometric pickup, then roll a drop on every later physics step.

    Pickup requires the named object to be near the grasp frame, the jaws past the
    profile's closure threshold, the object lifted from its first observed height,
    and the gap stable.
    After confirmation, each step draws one random number. A hit commands the
    gripper open for ``latch_steps`` consecutive steps, then stops.
    """

    def __init__(self, config: AutomaticDropConfig, profile: GripperClosure, rng: object) -> None:
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
