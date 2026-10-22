# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Extension Manager images. No Isaac Sim required."""

from __future__ import annotations

import struct
from pathlib import Path

EXTENSION_ROOT = Path(__file__).resolve().parents[1]
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
RGBA = 6


def _png_header(path: Path) -> tuple[int, int, int]:
    data = path.read_bytes()
    assert data.startswith(PNG_SIGNATURE)
    assert data[12:16] == b"IHDR"
    width, height, _bit_depth, color_type = struct.unpack(">IIBB", data[16:26])
    return width, height, color_type


def test_extension_manager_uses_transparent_logo() -> None:
    config = (EXTENSION_ROOT / "config" / "extension.toml").read_text(encoding="utf-8")
    assert 'icon = "data/icon.png"' in config
    assert 'preview_image = "data/preview.png"' in config

    icon_width, icon_height, icon_color = _png_header(EXTENSION_ROOT / "data" / "icon.png")
    assert (icon_width, icon_height, icon_color) == (256, 256, RGBA)

    _preview_width, _preview_height, preview_color = _png_header(EXTENSION_ROOT / "data" / "preview.png")
    assert preview_color == RGBA
