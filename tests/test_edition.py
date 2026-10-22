"""Edition selection from the environment."""

from __future__ import annotations

from gangelia.fault_injector.edition import (
    camera_noise_installed,
    edition_name,
    pro_camera_available,
    pro_enabled,
)


def test_unset_or_regular_is_not_pro(monkeypatch) -> None:
    monkeypatch.delenv("GANGELIA_FAULT_INJECTOR_EDITION", raising=False)
    assert edition_name() == "regular"
    assert pro_enabled() is False
    monkeypatch.setenv("GANGELIA_FAULT_INJECTOR_EDITION", " regular ")
    assert edition_name() == "regular"
    assert pro_enabled() is False


def test_pro_values_enable_pro(monkeypatch) -> None:
    monkeypatch.setenv("GANGELIA_FAULT_INJECTOR_EDITION", "pro")
    assert pro_enabled() is True
    monkeypatch.setenv("GANGELIA_FAULT_INJECTOR_EDITION", "PRO")
    assert pro_enabled() is True


def test_camera_noise_requires_both_the_pro_edition_and_the_files(monkeypatch) -> None:
    monkeypatch.delenv("GANGELIA_FAULT_INJECTOR_EDITION", raising=False)
    assert pro_camera_available() is False
    monkeypatch.setenv("GANGELIA_FAULT_INJECTOR_EDITION", "pro")
    assert pro_camera_available() is camera_noise_installed()
