# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file in isaac/gangelia.fault_injector.

"""The list of faults the Isaac window shows.

Add a row here when you want a new failure to appear. A row does not make the
failure run. Each simulator still needs its own action. See README.md.
"""

from __future__ import annotations

from dataclasses import dataclass


DEMO_FAULT_NOTICE = "Demo Version - Contact Gangelia"


@dataclass(frozen=True)
class FaultRecord:
    """One failure, and where its action lives today."""

    name: str
    isaac_ready: bool
    libero_type: str | None
    what_to_edit: str


@dataclass(frozen=True)
class FaultChoice:
    """One row in the Isaac fault list. Unavailable rows show the demo notice."""

    name: str
    available: bool


FAULTS: tuple[FaultRecord, ...] = (
    FaultRecord(
        name="Drop",
        isaac_ready=True,
        libero_type="midair_drop",
        what_to_edit=(
            "Isaac decides with shared_faults/drop_rule.py and opens the gripper in "
            "isaac/gangelia.fault_injector. LIBERO decides with shared_faults/basket_drop.py "
            "and opens the gripper in lerobot_faults/src/lerobot_faults/sim/libero.py. "
            "Those two decisions are not the same."
        ),
    ),
    FaultRecord(
        name="Object Over Object",
        isaac_ready=False,
        libero_type=None,
        what_to_edit="No action exists yet. Add the Isaac action and the LIBERO action, then set isaac_ready.",
    ),
    FaultRecord(
        name="Camera Noise",
        isaac_ready=False,
        libero_type=None,
        what_to_edit=(
            "No Isaac camera action exists yet. LIBERO observation faults "
            "(visual_blur, visual_occlusion, brightness_drop, sensor_dropout, obs_latency) "
            "change the image dict sent to a policy. They do not change an Isaac viewport."
        ),
    ),
    FaultRecord(
        name="Object Collision",
        isaac_ready=False,
        libero_type=None,
        what_to_edit="No action exists yet. LIBERO object_slip and eef_bump write MuJoCo velocities, not PhysX contacts.",
    ),
)

FAULT_CHOICES: tuple[FaultChoice, ...] = tuple(
    FaultChoice(record.name, record.isaac_ready) for record in FAULTS
)


def fault_notice(index: int) -> str | None:
    """Return the demo notice for a locked fault, or None when that fault runs."""
    choice = FAULT_CHOICES[index]
    if choice.available:
        return None
    return DEMO_FAULT_NOTICE
