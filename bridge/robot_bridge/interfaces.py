from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import numpy as np


class IArmController(ABC):
    @abstractmethod
    def jog_cartesian(
        self, dx: float, dy: float, dz: float, dpitch: float, dyaw: float
    ) -> None:
        """Consume normalized -1..1 velocity inputs and move the arm."""

    def hold_position(self) -> None:
        self.jog_cartesian(0, 0, 0, 0, 0)

    def check_feedback(self) -> None:
        """Poll feedback and stop on loss while motion is active."""

    def fault_stop(self, reason: str) -> None:
        self.estop()

    @abstractmethod
    def set_height_delta(self, delta_mm: float) -> None:
        """Move the current Cartesian target by delta_mm on Z."""

    @abstractmethod
    def get_joint_margins(self) -> list[float]:
        """Return six joint-limit margins in percent (0..100)."""

    @abstractmethod
    def get_pose_safety_ok(self) -> bool:
        """Return whether current pose/arm status is inside the safe domain."""

    @abstractmethod
    def home(self) -> None:
        """Command the configured home pose."""

    @abstractmethod
    def estop(self) -> None:
        """Immediately stop motion and latch the controller."""

    def close(self) -> None:
        """Release hardware resources, if any."""


class ICameraSource(ABC):
    @abstractmethod
    def get_jpeg_frame(self) -> bytes:
        """Return one JPEG-encoded color image."""

    @abstractmethod
    def get_depth_frame(self) -> "np.ndarray | None":
        """Return a depth image, or None when the camera has no depth sensor."""

    def close(self) -> None:
        """Release hardware resources, if any."""


class ILaserController(ABC):
    @abstractmethod
    def set_request(self, enabled: bool, power: int, speed: float, safe: bool) -> None:
        """Apply a laser request; implementations must enforce safe=False as off."""

    @abstractmethod
    def force_off(self) -> None:
        """Immediately deactivate the laser."""

    @abstractmethod
    def is_active(self) -> bool:
        """Return physical laser activity, not merely the requested state."""

    def close(self) -> None:
        """Release hardware resources, if any."""

