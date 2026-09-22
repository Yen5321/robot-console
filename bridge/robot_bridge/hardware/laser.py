from __future__ import annotations

import logging

from ..interfaces import ILaserController


class NoOpLaserController(ILaserController):
    """Safe placeholder: accepts/logs requests but never actuates hardware."""

    def __init__(self) -> None:
        self._last_request: tuple[bool, int, float, bool] | None = None
        self._log = logging.getLogger(__name__)

    def set_request(self, enabled: bool, power: int, speed: float, safe: bool) -> None:
        request = (enabled, power, speed, safe)
        if request != self._last_request:
            self._log.info(
                "laser placeholder request enabled=%s power=%d speed=%.1f safe=%s (not actuated)",
                enabled, power, speed, safe,
            )
            self._last_request = request

    def force_off(self) -> None:
        self._last_request = (False, 0, 0.0, False)

    def is_active(self) -> bool:
        return False

