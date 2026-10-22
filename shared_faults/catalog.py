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

# One branch in this repository. Two packages, built by
# isaac/gangelia.fault_injector/release/assemble.py.
#
# Ask the user "Is this fault Pro or Community?" before adding a name here.
# Do not infer the answer. Camera Noise is listed because the user said Pro.
#
# Community (the default, and the only package that may be copied into
# dspip/gangelia.fault_injector and uploaded to NVIDIA) omits every file
# named here. Pro (assemble.py DESTINATION pro) includes them. Do not put
# a Pro package in the NVIDIA repo. After the user says a fault is Pro, add
# its files here and set pro_only=True. After the user says Community, leave
# its files out of these tuples.
PRO_SHARED_FILES = ("camera_noise.py",)
PRO_EXTENSION_FILES = (
    "gangelia/fault_injector/camera_motion.py",
    "gangelia/fault_injector/camera_noise_session.py",
    "gangelia/fault_injector/camera_viewport.py",
    "gangelia/fault_injector/isaac_camera.py",
)
PRO_EXTENSION_TESTS = (
    "tests/test_camera_motion.py",
    "tests/test_camera_noise_session.py",
    "tests/test_camera_viewport.py",
)
PRO_SCRIPT_NAMES = ("camera_noise_proof.py",)


@dataclass(frozen=True)
class FaultRecord:
    """One failure, and where its action lives today."""

    name: str
    isaac_ready: bool
    libero_type: str | None
    what_to_edit: str
    pro_only: bool = False


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
        isaac_ready=True,
        libero_type=None,
        pro_only=True,
        what_to_edit=(
            "Pro only. The math is shared_faults/camera_noise.py. The Isaac files are listed in "
            "PRO_EXTENSION_FILES above. Community assemble omits them and this row shows the demo "
            "notice. The Pro package includes them. LIBERO is unchanged."
        ),
    ),
    FaultRecord(
        name="Object Collision",
        isaac_ready=False,
        libero_type=None,
        what_to_edit="No action exists yet. LIBERO object_slip and eef_bump write MuJoCo velocities, not PhysX contacts.",
    ),
)

def fault_choices(pro: bool) -> tuple[FaultChoice, ...]:
    """Build the fault list for one edition."""
    return tuple(
        FaultChoice(
            record.name,
            record.isaac_ready and (pro or not record.pro_only),
        )
        for record in FAULTS
    )


FAULT_CHOICES: tuple[FaultChoice, ...] = fault_choices(False)


def fault_notice_for(index: int, pro: bool) -> str | None:
    """Return the demo notice for a locked fault, or None when that fault runs."""
    choice = fault_choices(pro)[index]
    if choice.available:
        return None
    return DEMO_FAULT_NOTICE


def fault_notice(index: int) -> str | None:
    """Return the demo notice for the regular edition."""
    return fault_notice_for(index, False)


def fault_panel(name: str, available: bool) -> str:
    """Return which Isaac window panel should show for one fault row."""
    if not available:
        return "demo"
    if name == "Drop":
        return "drop"
    if name == "Camera Noise":
        return "camera"
    return "demo"
