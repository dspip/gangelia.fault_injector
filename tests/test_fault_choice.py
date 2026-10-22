"""Fault catalog for the panel: Drop runs. Camera Noise is Pro."""

from __future__ import annotations

import pytest
from gangelia.fault_injector.core import (
    DEMO_FAULT_NOTICE,
    FAULT_CHOICES,
    fault_choices,
    fault_notice,
    fault_notice_for,
    fault_panel,
)
from gangelia.fault_injector.theme import BUTTON_LABEL, INK, SURFACE, abgr


def test_drop_runs_and_the_other_faults_show_the_demo_notice() -> None:
    assert [choice.name for choice in FAULT_CHOICES] == [
        "Drop",
        "Object Over Object",
        "Camera Noise",
        "Object Collision",
    ]
    assert fault_notice(0) is None
    assert fault_notice(2) == "Demo Version - Contact Gangelia"
    assert fault_notice(1) == "Demo Version - Contact Gangelia"
    assert fault_notice(3) == "Demo Version - Contact Gangelia"
    assert DEMO_FAULT_NOTICE == "Demo Version - Contact Gangelia"


def test_an_unknown_fault_index_is_rejected() -> None:
    with pytest.raises(IndexError):
        fault_notice(len(FAULT_CHOICES))


def test_camera_noise_is_pro_only() -> None:
    regular = fault_choices(False)
    pro = fault_choices(True)
    assert fault_notice_for(2, False) == DEMO_FAULT_NOTICE
    assert fault_notice_for(2, True) is None
    assert fault_notice_for(1, False) == DEMO_FAULT_NOTICE
    assert fault_notice_for(3, False) == DEMO_FAULT_NOTICE
    assert fault_panel("Drop", True) == "drop"
    assert fault_panel("Camera Noise", True) == "camera"
    assert fault_panel("Camera Noise", False) == "demo"
    assert fault_panel("Object Over Object", False) == "demo"
    assert [choice.name for choice in regular if choice.available] == ["Drop"]
    assert [choice.name for choice in pro if choice.available] == ["Drop", "Camera Noise"]


def test_panel_text_is_light_on_a_dark_surface() -> None:
    assert abgr(0x12, 0x17, 0x1E) == SURFACE
    assert abgr(0xF4, 0xF6, 0xF8) == INK
    assert BUTTON_LABEL == SURFACE
