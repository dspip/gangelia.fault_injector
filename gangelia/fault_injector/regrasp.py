# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Decide the Panda regrasp after a drop. This file does not move a robot.

The pinch frame is the Lula ``right_gripper`` link. In
``lula_franka_gen.urdf`` that link is 0.10 m along ``panda_link8`` +Z, and
``panda_hand`` is fixed at the ``panda_link8`` origin, so the offset is along
the hand's local Z. A parallel gripper is unchanged by the extra half-turn
about that axis.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

PANDA_HAND_TO_PINCH_M = 0.10
_HAND_TO_PINCH_QUAT = (0.0, 0.0, 0.0, 1.0)
# Finger approach is the pinch frame +Z. Above this alignment with world down, keep the live wrist.
_DOWN_DOT_MIN = 0.85
_WORLD_DOWN = (0.0, 0.0, -1.0)


@dataclass(frozen=True)
class RegraspConfig:
    """Step budgets for one regrasp."""

    hover_above_m: float = 0.10
    lift_above_m: float = 0.10
    arrive_tolerance_m: float = 0.02
    max_move_s: float = 8.0
    close_s: float = 0.5
    still_s: float = 0.3
    still_band_m: float = 0.005

    def __post_init__(self) -> None:
        """Reject a budget that cannot describe a wait, a move, or a close."""
        if self.hover_above_m <= 0:
            raise ValueError("hover_above_m must be positive")
        if self.lift_above_m <= 0:
            raise ValueError("lift_above_m must be positive")
        if self.arrive_tolerance_m <= 0:
            raise ValueError("arrive_tolerance_m must be positive")
        if self.max_move_s <= 0:
            raise ValueError("max_move_s must be positive")
        if self.close_s <= 0:
            raise ValueError("close_s must be positive")
        if self.still_s <= 0:
            raise ValueError("still_s must be positive")
        if self.still_band_m < 0:
            raise ValueError("still_band_m must be non-negative")


@dataclass(frozen=True)
class RegraspCommand:
    """What the extension should command on this physics step."""

    phase: str
    target_position: tuple[float, float, float] | None
    fingers_closed: bool | None
    reason: str = ""


class RegraspSequence:
    """Wait for a settled drop, then hover, descend, close, and lift.

    ``arm`` starts after a drop release, or when the caller says the object is
    already clear of the hand. A second press while a regrasp is running leaves
    the current step alone.
    """

    def __init__(self, config: RegraspConfig | None = None) -> None:
        self._config = config or RegraspConfig()
        self.reset()

    def reset(self) -> None:
        """Forget the release and any motion that was in progress."""
        self._phase = "idle"
        self._released = False
        self._move_elapsed = 0.0
        self._close_elapsed = 0.0
        self._dt = 0.0
        self._samples: list[tuple[float, float]] = []
        self._anchor: tuple[float, float, float] | None = None
        self._reason = ""

    @property
    def active(self) -> bool:
        """True while the extension must keep stepping this regrasp."""
        return self._phase in {"settle", "hover", "descend", "close", "lift"}

    @property
    def phase(self) -> str:
        """Current phase name."""
        return self._phase

    @property
    def released(self) -> bool:
        """True after Inject Drop or an automatic drop has opened the gripper."""
        return self._released

    def note_drop_released(self) -> None:
        """Record that a drop command opened the gripper."""
        self._released = True

    def arm(self, *, object_clear: bool = False) -> str | None:
        """Start a regrasp.

        ``object_clear`` means the object is already farther from the hand than
        the pickup distance, so a manual open or a finished pick can regrasp.
        Return an error message when the object is still in the hand and no
        drop has released it.
        """
        if self.active:
            return None
        if object_clear:
            self._released = True
        if not self._released:
            return "Regrasp waits until the object has left the gripper."
        self._phase = "settle"
        self._move_elapsed = 0.0
        self._close_elapsed = 0.0
        self._samples = []
        self._anchor = None
        self._reason = ""
        return None

    def step(
        self,
        *,
        dt_s: float,
        object_position: tuple[float, float, float],
        pinch_position: tuple[float, float, float],
        hand_position: tuple[float, float, float],
        distance_threshold_m: float,
    ) -> RegraspCommand:
        """Advance one physics step and return the command for that step."""
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        self._dt = dt_s
        if self._phase == "settle":
            return self._settle(object_position, pinch_position, hand_position, distance_threshold_m)
        if self._phase == "hover":
            return self._hover(object_position, pinch_position)
        if self._phase == "descend":
            return self._descend(object_position, pinch_position)
        if self._phase == "close":
            return self._close()
        if self._phase == "lift":
            return self._lift(pinch_position)
        return RegraspCommand(self._phase, None, None, self._reason)

    def _settle(
        self,
        object_position: tuple[float, float, float],
        pinch_position: tuple[float, float, float],
        hand_position: tuple[float, float, float],
        distance_threshold_m: float,
    ) -> RegraspCommand:
        self._samples.append((self._dt, object_position[2]))
        while (
            len(self._samples) > 1
            and sum(sample_dt for sample_dt, _height in self._samples) - self._samples[0][0]
            >= self._config.still_s
        ):
            self._samples.pop(0)
        duration = sum(sample_dt for sample_dt, _height in self._samples)
        heights = [height for _sample_dt, height in self._samples]
        settled = duration >= self._config.still_s and max(heights) - min(heights) <= self._config.still_band_m
        separated = _distance(object_position, hand_position) > distance_threshold_m
        if settled and separated:
            self._phase = "hover"
            self._move_elapsed = 0.0
            return self._hover(object_position, pinch_position)
        self._move_elapsed += self._dt
        if self._move_elapsed > self._config.max_move_s:
            self._phase = "failed"
            self._reason = "Regrasp stopped: the object did not leave the gripper."
            return RegraspCommand("failed", None, None, self._reason)
        return RegraspCommand("settle", None, False)

    def _descend(
        self,
        object_position: tuple[float, float, float],
        pinch_position: tuple[float, float, float],
    ) -> RegraspCommand:
        target = (object_position[0], object_position[1], object_position[2])
        if _distance(pinch_position, target) <= self._config.arrive_tolerance_m:
            self._anchor = target
            self._phase = "close"
            self._close_elapsed = 0.0
            self._move_elapsed = 0.0
            return self._close()
        return self._count_move("descend", target, fingers_closed=False)

    def _close(self) -> RegraspCommand:
        self._close_elapsed += self._dt
        command = RegraspCommand("close", self._anchor, True)
        if self._close_elapsed >= self._config.close_s:
            self._phase = "lift"
            self._move_elapsed = 0.0
        return command

    def _lift(self, pinch_position: tuple[float, float, float]) -> RegraspCommand:
        assert self._anchor is not None
        target = self._above(self._anchor, self._config.lift_above_m)
        if _distance(pinch_position, target) <= self._config.arrive_tolerance_m:
            self._phase = "done"
            return RegraspCommand("done", None, None)
        return self._count_move("lift", target, fingers_closed=True)

    def _hover(
        self,
        object_position: tuple[float, float, float],
        pinch_position: tuple[float, float, float],
    ) -> RegraspCommand:
        target = self._above(object_position, self._config.hover_above_m)
        if _distance(pinch_position, target) <= self._config.arrive_tolerance_m:
            self._phase = "descend"
            self._move_elapsed = 0.0
            return self._descend(object_position, pinch_position)
        return self._count_move("hover", target, fingers_closed=False)

    def _count_move(
        self,
        phase: str,
        target: tuple[float, float, float],
        *,
        fingers_closed: bool,
    ) -> RegraspCommand:
        self._move_elapsed += self._dt
        if self._move_elapsed > self._config.max_move_s:
            self._phase = "failed"
            self._reason = "Regrasp stopped: the gripper did not reach the object."
            return RegraspCommand("failed", None, None, self._reason)
        return RegraspCommand(phase, target, fingers_closed)

    def _above(
        self, position: tuple[float, float, float], height_m: float
    ) -> tuple[float, float, float]:
        return (position[0], position[1], position[2] + height_m)


def object_clear_of_hand(
    object_position: tuple[float, float, float],
    hand_position: tuple[float, float, float],
    distance_threshold_m: float,
) -> bool:
    """Return whether the object is farther from the hand than a pickup."""
    return _distance(object_position, hand_position) > distance_threshold_m


def resume_drop_watch(*, automatic_regrasp: bool, drop_enabled: bool) -> bool:
    """Automatic regrasp runs once. The drop watcher stays off until checked again."""
    if automatic_regrasp:
        return False
    return drop_enabled


def grasp_pinch_orientation(
    current_quat: tuple[float, float, float, float],
    base_xy: tuple[float, float],
    object_xy: tuple[float, float],
) -> tuple[float, float, float, float]:
    """Return the pinch orientation to hold for the approach.

    A wrist that already points down is kept. Any other wrist is replaced by
    fingers straight down, with local +X aimed from the robot base toward the
    object in the horizontal plane.
    """
    current = _normalize_quat(current_quat)
    if _dot(_quat_z_axis(current), _WORLD_DOWN) >= _DOWN_DOT_MIN:
        return current
    heading = (object_xy[0] - base_xy[0], object_xy[1] - base_xy[1])
    length = math.hypot(heading[0], heading[1])
    if length < 1e-6:
        x_axis = (1.0, 0.0, 0.0)
    else:
        x_axis = (heading[0] / length, heading[1] / length, 0.0)
    y_axis = _cross(_WORLD_DOWN, x_axis)
    return _quat_from_axes(x_axis, y_axis, _WORLD_DOWN)


def pinch_from_panda_hand(
    position: tuple[float, float, float],
    quat_wxyz: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Return the ``right_gripper`` position and orientation for this hand pose."""
    px, py, pz = (float(position[0]), float(position[1]), float(position[2]))
    w, x, y, z = (
        float(quat_wxyz[0]),
        float(quat_wxyz[1]),
        float(quat_wxyz[2]),
        float(quat_wxyz[3]),
    )
    axis_z = (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )
    pinch = (
        px + PANDA_HAND_TO_PINCH_M * axis_z[0],
        py + PANDA_HAND_TO_PINCH_M * axis_z[1],
        pz + PANDA_HAND_TO_PINCH_M * axis_z[2],
    )
    return pinch, _multiply_quat((w, x, y, z), _HAND_TO_PINCH_QUAT)


def _normalize_quat(
    quat: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
    length = math.sqrt(w * w + x * x + y * y + z * z)
    if length < 1e-12:
        raise ValueError("pinch orientation is zero")
    return (w / length, x / length, y / length, z / length)


def _quat_z_axis(quat: tuple[float, float, float, float]) -> tuple[float, float, float]:
    w, x, y, z = quat
    return (
        2.0 * (x * z + y * w),
        2.0 * (y * z - x * w),
        1.0 - 2.0 * (x * x + y * y),
    )


def _dot(left: tuple[float, float, float], right: tuple[float, float, float]) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def _cross(
    left: tuple[float, float, float], right: tuple[float, float, float]
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _quat_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    """Scalar-first quaternion for a rotation whose columns are these axes."""
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return (
            0.25 * scale,
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
        )
    if m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (
            (m21 - m12) / scale,
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
        )
    if m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return (
            (m02 - m20) / scale,
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
        )
    scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return (
        (m10 - m01) / scale,
        (m02 + m20) / scale,
        (m12 + m21) / scale,
        0.25 * scale,
    )


def _multiply_quat(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _distance(start: tuple[float, float, float], end: tuple[float, float, float]) -> float:
    return math.sqrt(sum((left - right) ** 2 for left, right in zip(start, end, strict=True)))
