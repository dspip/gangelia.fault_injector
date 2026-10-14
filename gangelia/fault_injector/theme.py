# Copyright 2026 Gangelia. All rights reserved.
#
# Permission to use this file is granted only while the copyright remains
# Gangelia's. See the LICENSE file.

"""Panel colors in omni.ui's ABGR channel order."""

from __future__ import annotations


def abgr(red: int, green: int, blue: int, alpha: int = 0xFF) -> int:
    """Pack an RGB color into the 0xAABBGGRR integer omni.ui stores."""
    return (alpha << 24) | (blue << 16) | (green << 8) | red


SURFACE = abgr(0x12, 0x17, 0x1E)
CARD = abgr(0x1C, 0x24, 0x2E)
INK = abgr(0xF4, 0xF6, 0xF8)
MUTED = abgr(0x8E, 0x9B, 0xAB)
HAIRLINE = abgr(0x32, 0x3C, 0x48)
STATUS = abgr(0x3E, 0xAF, 0x78)
BUTTON = abgr(0xF4, 0xF6, 0xF8)
BUTTON_LABEL = SURFACE
