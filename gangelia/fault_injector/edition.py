# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Which Gangelia Fault Injector edition is active.

The environment variable is not enough. Camera Noise also requires the Pro
files. A Community package does not have those files, so setting the variable
there does not unlock the row.
"""

from __future__ import annotations

import os
from pathlib import Path

_ENV_NAME = "GANGELIA_FAULT_INJECTOR_EDITION"


def edition_name() -> str:
    """Return the configured edition name, defaulting to regular."""
    raw = os.environ.get(_ENV_NAME, "regular")
    return raw.strip().lower() or "regular"


def pro_enabled() -> bool:
    """Return True when the pro edition is active."""
    return edition_name() == "pro"


def camera_noise_installed() -> bool:
    """Return True when this package contains the Camera Noise files."""
    from gangelia.fault_injector.core import shared_faults_import_root
    from shared_faults.catalog import PRO_EXTENSION_FILES, PRO_SHARED_FILES

    extension_root = Path(__file__).resolve().parents[2]
    shared_root = shared_faults_import_root(Path(__file__)) / "shared_faults"
    shared_ready = all((shared_root / name).is_file() for name in PRO_SHARED_FILES)
    extension_ready = all((extension_root / relative).is_file() for relative in PRO_EXTENSION_FILES)
    return shared_ready and extension_ready


def pro_camera_available() -> bool:
    """Return True when Camera Noise is allowed to run in this package."""
    return pro_enabled() and camera_noise_installed()
