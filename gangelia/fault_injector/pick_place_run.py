# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file in isaac/gangelia.fault_injector.

"""Run pick and place on the Franka and cube already in the open stage.

The stage is never replaced. The camera prim is never moved. Stepping starts
only while the timeline is playing.
"""

from __future__ import annotations

from collections.abc import Callable

import numpy as np

from gangelia.fault_injector.scene_paths import FaultScene, ScenePrim, discover_fault_scene


def pick_place_step(scene: FaultScene, timeline_playing: bool) -> str:
    """Return ``missing``, ``wait``, or ``start`` for one request."""
    if not scene.robot_path or not scene.cube_path:
        return "missing"
    if not timeline_playing:
        return "wait"
    return "start"


def choose_place_position(
    base_position: np.ndarray,
    cube_position: np.ndarray,
    place_position: np.ndarray | None,
) -> np.ndarray:
    """Return an end-effector place pose beside the base.

    A cube against the pedestal must not be carried back through the base.
    A place prim that sits inside that pedestal is ignored.
    """
    base = np.asarray(base_position, dtype=np.float64).reshape(-1)
    cube = np.asarray(cube_position, dtype=np.float64).reshape(-1)
    beside = _place_beside_base(base, cube)
    if place_position is None:
        return beside
    place = np.asarray(place_position, dtype=np.float64).reshape(-1)
    if float(np.hypot(place[0] - base[0], place[1] - base[1])) < _PLACE_RADIUS_M:
        return beside
    return np.array(
        [place[0], place[1], max(float(place[2]), float(cube[2]) + _PLACE_CLEARANCE_M)],
        dtype=np.float64,
    )


def _place_beside_base(base: np.ndarray, cube: np.ndarray) -> np.ndarray:
    away = cube[:2] - base[:2]
    if float(np.hypot(away[0], away[1])) < 1e-6:
        away = np.array([1.0, 0.0])
    side = np.array([-away[1], away[0]], dtype=np.float64)
    side = side / np.hypot(side[0], side[1])
    place_xy = base[:2] + side * _PLACE_RADIUS_M
    return np.array([place_xy[0], place_xy[1], float(cube[2]) + _PLACE_CLEARANCE_M], dtype=np.float64)


_PLACE_RADIUS_M = 0.45
_PLACE_CLEARANCE_M = 0.10


def scene_from_stage(stage) -> FaultScene:
    """Read prim paths from the stage the user already has open."""
    prims = [ScenePrim(str(prim.GetPath()), prim.GetTypeName(), prim.GetName()) for prim in stage.Traverse()]
    return discover_fault_scene(prims)


class PickPlaceRun:
    """Step one pick-and-place controller while the timeline is playing."""

    def __init__(self, on_status: Callable[[str], None]) -> None:
        self._on_status = on_status
        self._scene: FaultScene | None = None
        self._controller = None
        self._physics_subscription = None
        self._play_subscription = None
        self._stop_subscription = None
        self._waiting_for_play = False
        self._needs_reset = False
        self._hold_gripper_open = False

    @property
    def scene(self) -> FaultScene | None:
        """Return the scene from the last request, if any."""
        return self._scene

    def request(self, stage, timeline_playing: bool) -> str:
        """Arm pick and place. Start immediately only when the timeline is playing."""
        self.stop()
        scene = scene_from_stage(stage)
        self._scene = scene
        step = pick_place_step(scene, timeline_playing)
        if step == "missing":
            return "This stage needs a Franka and a Cube before pick and place can run."
        if step == "wait":
            self._waiting_for_play = True
            self._listen_for_timeline()
            return "Press Play to start pick and place."
        self._begin()
        return f"Pick and place is running on {scene.robot_path}."

    def arm_on_play(self, stage) -> str:
        """Run pick and place when the user presses Play. Do not start now."""
        self.stop()
        scene = scene_from_stage(stage)
        self._scene = scene
        if pick_place_step(scene, timeline_playing=False) == "missing":
            return "This stage needs a Franka and a Cube before pick and place can run."
        self._waiting_for_play = True
        self._listen_for_timeline()
        return "Press Play to start pick and place."

    def hold_gripper_open(self, hold: bool) -> None:
        """Keep the fingers open after each pick-and-place step while a drop is latched."""
        self._hold_gripper_open = hold

    @property
    def driving(self) -> bool:
        """True while pick and place owns the physics step."""
        return self._controller is not None or self._physics_subscription is not None

    def live_carry(self, joint_names: tuple[str, ...]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, ...]] | None:
        """Return the cube, the hand, and the finger joints from the running controller.

        The saved USD pose does not move while the timeline is playing. This
        read is the simulated cube the arm is actually carrying.
        """
        controller = self._controller
        robot = getattr(controller, "robot", None) if controller is not None else None
        cube = getattr(controller, "cube", None) if controller is not None else None
        if robot is None or cube is None:
            return None
        poses = cube.get_world_poses()
        if not poses or poses[0] is None:
            raise ValueError("cube pose is not ready")
        state = robot.get_current_state()
        cube_position = flat_floats(poses[0])
        hand_position = flat_floats(state[1])
        fingers = finger_positions_from_dofs(tuple(robot.dof_names), flat_floats(robot.get_dof_positions()), joint_names)
        return (
            (float(cube_position[0]), float(cube_position[1]), float(cube_position[2])),
            (float(hand_position[0]), float(hand_position[1]), float(hand_position[2])),
            fingers,
        )

    def stop(self) -> None:
        """Drop the motion and the timeline listener."""
        self._halt_motion()
        self._waiting_for_play = False
        self._play_subscription = None
        self._stop_subscription = None

    def _halt_motion(self) -> None:
        self._needs_reset = False
        self._physics_subscription = None
        self._controller = None

    def _listen_for_timeline(self) -> None:
        import omni.timeline

        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._play_subscription = stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY),
            self._on_timeline_play,
            name="gangelia.fault_injector.pick_place_play",
        )
        self._stop_subscription = stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP),
            self._on_timeline_stop,
            name="gangelia.fault_injector.pick_place_stop",
        )

    def _on_timeline_play(self, _event) -> None:
        if not self._waiting_for_play or self._controller is not None or self._physics_subscription is not None:
            return
        self._waiting_for_play = False
        try:
            self._begin()
        except Exception as exc:  # noqa: BLE001 — show the failure in the panel
            self._halt_motion()
            self._waiting_for_play = True
            self._on_status(f"Pick and place stopped: {exc}")
            return
        scene = self._scene
        robot = scene.robot_path if scene is not None else "the Franka"
        self._on_status(f"Pick and place is running on {robot}.")

    def _on_timeline_stop(self, _event) -> None:
        moving = self._controller is not None or self._physics_subscription is not None
        self._halt_motion()
        self._waiting_for_play = True
        if moving:
            self._on_status("Pick and place stopped. Press Play to run it again.")

    def _begin(self) -> None:
        import omni.physx

        self._needs_reset = True
        self._physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_physics)

    def _on_physics(self, _step) -> None:
        import omni.timeline
        import omni.usd

        if not omni.timeline.get_timeline_interface().is_playing():
            return
        try:
            if self._controller is None:
                stage = omni.usd.get_context().get_stage()
                if stage is None or self._scene is None:
                    raise RuntimeError("no stage is open")
                self._controller = _attach(stage, self._scene)
            controller = self._controller
            if self._needs_reset:
                controller.reset()
                self._needs_reset = False
            if controller.is_done():
                self._halt_motion()
                self._waiting_for_play = True
                self._on_status("Pick and place finished.")
                return
            advance_pick_place(controller, hold_gripper_open=self._hold_gripper_open)
        except Exception as exc:  # noqa: BLE001 — show the failure in the panel
            self._halt_motion()
            self._waiting_for_play = True
            self._on_status(f"Pick and place stopped: {exc}")


def flat_floats(value) -> np.ndarray:
    """Return a 1-D float vector from a NumPy array or a tensor that exposes ``numpy()``."""
    if value is None:
        raise ValueError("missing simulation value")
    if not isinstance(value, np.ndarray):
        converter = getattr(value, "numpy", None)
        if callable(converter):
            value = converter()
    return np.asarray(value, dtype=np.float64).reshape(-1)


def finger_positions_from_dofs(names, positions, wanted: tuple[str, ...]) -> tuple[float, ...]:
    """Read the named finger joints out of one articulation position vector."""
    flat = flat_floats(positions)
    index = {str(name): i for i, name in enumerate(names)}
    missing = [name for name in wanted if name not in index]
    if missing:
        raise ValueError("gripper joints not on the robot: " + ", ".join(missing))
    return tuple(float(flat[index[name]]) for name in wanted)


def advance_pick_place(controller, *, hold_gripper_open: bool) -> None:
    """Step the arm, then open the fingers when a drop is holding them open.

    The pick-and-place close runs in the same physics step. Opening afterwards
    is what lets the cube leave the hand.
    """
    controller.forward()
    if hold_gripper_open:
        controller.robot.open_gripper()


def _attach(stage, scene: FaultScene):
    from isaacsim.core.experimental.prims import RigidPrim
    from isaacsim.robot.manipulators.examples.franka.franka_experimental import FrankaExperimental
    from isaacsim.robot.manipulators.examples.franka.pick_place.pick_place import FrankaPickPlace

    controller = FrankaPickPlace()
    controller.robot = FrankaExperimental(robot_path=scene.robot_path, create_robot=False)
    controller.end_effector_link = controller.robot.end_effector_link
    controller.cube = RigidPrim(paths=scene.cube_path)
    positions, orientations = controller.cube.get_world_poses()
    controller.cube_initial_position = np.array(positions.numpy()[0], dtype=np.float64)
    controller.cube_initial_orientation = np.array(orientations.numpy()[0], dtype=np.float64)
    controller.cube_size = np.array([0.0515, 0.0515, 0.0515])
    controller.offset = np.array([0.0, 0.0, 0.0])
    base = _world_translation(stage, scene.robot_path)
    place = _world_translation(stage, scene.place_path) if scene.place_path else None
    controller.target_position = choose_place_position(base, controller.cube_initial_position, place)
    return controller


def _world_translation(stage, prim_path: str) -> np.ndarray:
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    translation = UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation()
    return np.array([translation[0], translation[1], translation[2]], dtype=np.float64)
