"""Checks that the add-on folder can be sent without the rest of this repository."""

from __future__ import annotations

from pathlib import Path

EXTENSION_ROOT = Path(__file__).parents[1]
FORBIDDEN_IMPORTS = ("lerobot", "fault_system", "libero", "robosuite")


def test_manifest_registers_the_extension() -> None:
    manifest = (EXTENSION_ROOT / "config" / "extension.toml").read_text()

    assert 'name = "gangelia.fault_injector"' in manifest
    assert 'title = "Gangelia Fault Injector"' in manifest
    assert 'category = "Simulation"' in manifest
    assert 'name = "gangelia.fault_injector.extension"' in manifest
    assert '"isaacsim.core.api" = {}' in manifest
    assert '"omni.kit.uiapp" = {}' in manifest


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
