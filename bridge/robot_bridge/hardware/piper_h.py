from __future__ import annotations

from ..interfaces import IArmController


class PiperHArmController(IArmController):
    """Future PiperH adapter skeleton; supply SDK/firmware details to implement."""

    def __init__(self, **_: object) -> None:
        raise NotImplementedError("PiperH driver is reserved but not implemented")

    def jog_cartesian(self, dx: float, dy: float, dz: float, dpitch: float, dyaw: float) -> None:
        raise NotImplementedError

    def set_height_delta(self, delta_mm: float) -> None:
        raise NotImplementedError

    def get_joint_margins(self) -> list[float]:
        raise NotImplementedError

    def get_pose_safety_ok(self) -> bool:
        raise NotImplementedError

    def home(self) -> None:
        raise NotImplementedError

    def estop(self) -> None:
        raise NotImplementedError

