# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file in isaac/gangelia.fault_injector.

"""Choose the robot, cube, camera, and place target already in a stage."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ScenePrim:
    """One prim the stage walker saw."""

    path: str
    type_name: str
    name: str


@dataclass(frozen=True)
class FaultScene:
    """Prim paths the fault-injection starter should drive."""

    robot_path: str | None
    cube_path: str | None
    camera_path: str | None
    place_path: str | None


def discover_fault_scene(prims: tuple[ScenePrim, ...] | list[ScenePrim]) -> FaultScene:
    """Return the Franka, cube, camera, and optional place prim in ``prims``."""
    robots: list[str] = []
    cubes: list[str] = []
    cameras: list[str] = []
    places: list[str] = []
    for prim in prims:
        if prim.name.lower() in {"franka", "robot"}:
            robots.append(prim.path)
        if prim.name.lower() == "cube" or prim.path.rstrip("/").lower().endswith("/cube"):
            cubes.append(prim.path)
        if prim.type_name == "Camera" and not _is_viewport_camera(prim):
            cameras.append(prim.path)
        if prim.name.lower() in {"bin", "target", "place"}:
            places.append(prim.path)
    return FaultScene(
        robot_path=_prefer_franka(robots),
        cube_path=cubes[0] if cubes else None,
        camera_path=_prefer_camera(cameras),
        place_path=places[0] if places else None,
    )


def _prefer_franka(paths: list[str]) -> str | None:
    for path in paths:
        if path.rstrip("/").endswith("/Franka"):
            return path
    if paths:
        return paths[0]
    return None


def _prefer_camera(paths: list[str]) -> str | None:
    for path in paths:
        if "wrist" in path.lower():
            return path
    for path in paths:
        if path.rstrip("/").endswith("/Camera"):
            return path
    if paths:
        return paths[0]
    return None


def _is_viewport_camera(prim: ScenePrim) -> bool:
    text = f"{prim.path} {prim.name}".lower()
    return "omniversekit" in text or prim.name.lower() in {"persp", "perspective"}
