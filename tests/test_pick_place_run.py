"""Pick and place waits for Play and refuses a stage with no Franka."""

from __future__ import annotations

import numpy as np

from gangelia.fault_injector.pick_place_run import choose_place_position, pick_place_step
from gangelia.fault_injector.scene_paths import FaultScene


def test_pick_place_waits_until_the_timeline_is_playing() -> None:
    scene = FaultScene("/World/Franka", "/World/Cube", "/World/Camera", None)
    assert pick_place_step(scene, timeline_playing=False) == "wait"
    assert pick_place_step(scene, timeline_playing=True) == "start"


def test_pick_place_refuses_a_stage_without_a_robot_or_cube() -> None:
    assert pick_place_step(FaultScene(None, "/World/Cube", None, None), True) == "missing"
    assert pick_place_step(FaultScene("/World/Franka", None, None, None), True) == "missing"


def test_close_cube_is_placed_beside_the_base_not_behind_it() -> None:
    base = np.array([0.0, 0.0, 0.0])
    cube = np.array([0.2, 0.0, 0.025])
    target = choose_place_position(base, cube, None)
    assert abs(float(target[1])) == 0.45
    assert abs(float(target[0])) < 1e-6
    assert float(target[2]) == 0.125
    assert float(np.hypot(target[0], target[1])) >= 0.4


def test_a_place_prim_inside_the_base_is_ignored() -> None:
    base = np.array([0.0, 0.0, 0.0])
    cube = np.array([0.5, 0.0, 0.025])
    inside = np.array([-0.1, 0.0, 0.02])
    target = choose_place_position(base, cube, inside)
    assert float(np.hypot(target[0] - base[0], target[1] - base[1])) >= 0.4


def test_a_latched_drop_opens_the_gripper_after_the_arm_step() -> None:
    from gangelia.fault_injector.pick_place_run import advance_pick_place

    class _Robot:
        def __init__(self) -> None:
            self.opened = 0

        def open_gripper(self) -> None:
            self.opened += 1

    class _Controller:
        def __init__(self) -> None:
            self.robot = _Robot()
            self.steps = 0

        def forward(self) -> None:
            self.steps += 1

    controller = _Controller()
    advance_pick_place(controller, hold_gripper_open=True)
    assert controller.steps == 1
    assert controller.robot.opened == 1
    advance_pick_place(controller, hold_gripper_open=False)
    assert controller.steps == 2
    assert controller.robot.opened == 1


def test_finger_positions_follow_the_named_joints() -> None:
    from gangelia.fault_injector.pick_place_run import finger_positions_from_dofs

    positions = finger_positions_from_dofs(
        ("panda_joint1", "panda_finger_joint1", "panda_finger_joint2"),
        [[0.1, 0.02, 0.03]],
        ("panda_finger_joint1", "panda_finger_joint2"),
    )
    assert positions == (0.02, 0.03)


def test_live_carry_accepts_numpy_arrays_from_the_controller() -> None:
    from gangelia.fault_injector.pick_place_run import PickPlaceRun

    class _Cube:
        def get_world_poses(self):
            return (np.array([[0.2, 0.0, 0.3]]), np.array([[1.0, 0.0, 0.0, 0.0]]))

    class _Robot:
        dof_names = ("panda_finger_joint1", "panda_finger_joint2")

        def get_current_state(self):
            return (np.zeros((1, 2)), np.array([[0.2, 0.0, 0.4]]), np.array([[1.0, 0.0, 0.0, 0.0]]))

        def get_dof_positions(self):
            return np.array([[0.0, 0.01]])

    class _Controller:
        robot = _Robot()
        cube = _Cube()

    run = PickPlaceRun(lambda _text: None)
    run._controller = _Controller()
    cube, hand, fingers = run.live_carry(("panda_finger_joint1", "panda_finger_joint2"))
    assert cube == (0.2, 0.0, 0.3)
    assert hand == (0.2, 0.0, 0.4)
    assert fingers == (0.0, 0.01)
