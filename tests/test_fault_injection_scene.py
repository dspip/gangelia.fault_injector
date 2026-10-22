"""Pick the Franka, cube, and camera already stored in a stage."""

from __future__ import annotations

from gangelia.fault_injector.scene_paths import ScenePrim, discover_fault_scene


def test_discover_prefers_the_named_franka_cube_and_wrist_camera() -> None:
    scene = discover_fault_scene(
        (
            ScenePrim("/World/ground", "Xform", "ground"),
            ScenePrim("/World/Franka", "Xform", "Franka"),
            ScenePrim("/World/Franka/panda_hand/wrist_camera", "Camera", "wrist_camera"),
            ScenePrim("/World/Cube", "Cube", "Cube"),
            ScenePrim("/World/Bin", "Xform", "Bin"),
            ScenePrim("/OmniverseKit_Persp", "Camera", "Persp"),
        )
    )
    assert scene.robot_path == "/World/Franka"
    assert scene.cube_path == "/World/Cube"
    assert scene.camera_path == "/World/Franka/panda_hand/wrist_camera"
    assert scene.place_path == "/World/Bin"


def test_discover_uses_a_world_camera_when_no_wrist_camera_exists() -> None:
    scene = discover_fault_scene(
        (
            ScenePrim("/World/robot", "Xform", "robot"),
            ScenePrim("/World/Camera", "Camera", "Camera"),
            ScenePrim("/World/Cube", "Cube", "Cube"),
        )
    )
    assert scene.robot_path == "/World/robot"
    assert scene.camera_path == "/World/Camera"
    assert scene.place_path is None


def test_discover_reports_missing_pieces() -> None:
    scene = discover_fault_scene((ScenePrim("/OmniverseKit_Persp", "Camera", "Persp"),))
    assert scene.robot_path is None
    assert scene.cube_path is None
    assert scene.camera_path is None
    assert scene.place_path is None
