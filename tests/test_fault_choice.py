"""Fault catalog for the panel: Drop runs, the other three are a demo notice."""

from __future__ import annotations

import pytest
from gangelia.fault_injector.core import DEMO_FAULT_NOTICE, FAULT_CHOICES, fault_notice
from gangelia.fault_injector.theme import BUTTON_LABEL, INK, SURFACE, abgr


def test_drop_runs_and_the_other_faults_show_the_demo_notice() -> None:
    assert [choice.name for choice in FAULT_CHOICES] == [
        "Drop",
        "Object Over Object",
        "Camera Noise",
        "Object Collision",
    ]
    assert fault_notice(0) is None
    for index in range(1, len(FAULT_CHOICES)):
        assert fault_notice(index) == "Demo Version - Contact Gangelia"
    assert DEMO_FAULT_NOTICE == "Demo Version - Contact Gangelia"


def test_an_unknown_fault_index_is_rejected() -> None:
    with pytest.raises(IndexError):
        fault_notice(len(FAULT_CHOICES))


def test_panel_text_is_light_on_a_dark_surface() -> None:
    assert abgr(0x12, 0x17, 0x1E) == SURFACE
    assert abgr(0xF4, 0xF6, 0xF8) == INK
    assert BUTTON_LABEL == SURFACE
