from __future__ import annotations

from typing import Any

from ..interfaces import ICameraSource


class HikvisionCamera(ICameraSource):
    """Future Hikvision adapter skeleton; depth must remain None."""

    def __init__(self, **_: object) -> None:
        raise NotImplementedError("Hikvision camera driver is reserved but not implemented")

    def get_jpeg_frame(self) -> bytes:
        raise NotImplementedError

    def get_depth_frame(self) -> Any | None:
        return None

