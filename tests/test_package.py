"""Checks that the add-on folder can be sent without the rest of this repository."""

from __future__ import annotations

import ast
from pathlib import Path

EXTENSION_ROOT = Path(__file__).parents[1]
FORBIDDEN_IMPORTS = ("lerobot", "lerobot_faults", "fault_system", "libero", "robosuite")


def test_manifest_registers_the_extension() -> None:
    manifest = (EXTENSION_ROOT / "config" / "extension.toml").read_text()

    assert 'name = "gangelia.fault_injector"' in manifest
    assert 'title = "Gangelia Fault Injector"' in manifest
    assert 'category = "Simulation"' in manifest
    assert 'name = "gangelia.fault_injector.extension"' in manifest
    assert '"isaacsim.core.api" = {}' in manifest
    assert '"omni.kit.uiapp" = {}' in manifest


def test_extension_does_not_import_camera_noise_while_loading() -> None:
    path = EXTENSION_ROOT / "gangelia" / "fault_injector" / "extension.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    banned = {
        "gangelia.fault_injector.camera_motion",
        "gangelia.fault_injector.camera_noise_session",
        "gangelia.fault_injector.camera_viewport",
        "gangelia.fault_injector.isaac_camera",
        "shared_faults.camera_noise",
    }
    class _Walker(ast.NodeVisitor):
        def __init__(self) -> None:
            self.function = ""
            self.found: list[str] = []

        def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
            previous = self.function
            self.function = node.name
            self.generic_visit(node)
            self.function = previous

        visit_AsyncFunctionDef = visit_FunctionDef

        def visit_ImportFrom(self, node: ast.ImportFrom) -> None:
            if node.module in banned and self.function != "_start_camera_noise":
                self.found.append(node.module or "")
            self.generic_visit(node)

        def visit_Import(self, node: ast.Import) -> None:
            for alias in node.names:
                if alias.name in banned and self.function != "_start_camera_noise":
                    self.found.append(alias.name)
            self.generic_visit(node)

    walker = _Walker()
    walker.visit(tree)
    assert walker.found == []


def test_extension_sources_do_not_import_this_repository() -> None:
    offenders: list[str] = []
    for path in EXTENSION_ROOT.rglob("*.py"):
        if "tests" in path.parts:
            continue
        text = path.read_text()
        for name in FORBIDDEN_IMPORTS:
            if f"import {name}" in text or f"from {name}" in text:
                offenders.append(f"{path.relative_to(EXTENSION_ROOT)} imports {name}")
    assert offenders == []
