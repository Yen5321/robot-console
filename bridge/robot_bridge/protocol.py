from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import ClassVar


CONTROL_FRAME_FMT = "<HIB3h5hBHHBB"
TELEMETRY_FRAME_FMT = "<HBHff6BBB"
CONTROL_FRAME_SIZE = struct.calcsize(CONTROL_FRAME_FMT)
TELEMETRY_FRAME_SIZE = struct.calcsize(TELEMETRY_FRAME_FMT)


class ProtocolError(ValueError):
    pass


def _normalized(raw: int, field: str) -> float:
    if not -1000 <= raw <= 1000:
        raise ProtocolError(f"{field}={raw} is outside -1000..1000")
    return raw / 1000.0


@dataclass(frozen=True, slots=True)
class ControlFrame:
    seq: int
    timestamp: int
    mode: int
    drive_x: int
    drive_y: int
    drive_rotate: int
    arm_dx: int
    arm_dy: int
    arm_dz: int
    arm_pitch: int
    arm_yaw: int
    laser_enable: int
    laser_power: int
    laser_speed: int
    estop: int
    heartbeat: int

    FORMAT: ClassVar[str] = CONTROL_FRAME_FMT
    SIZE: ClassVar[int] = CONTROL_FRAME_SIZE

    @classmethod
    def unpack(cls, data: bytes) -> "ControlFrame":
        if len(data) != cls.SIZE:
            raise ProtocolError(f"control frame must be {cls.SIZE} bytes, got {len(data)}")
        frame = cls(*struct.unpack(cls.FORMAT, data))
        frame.validate()
        return frame

    def validate(self) -> None:
        if self.mode not in (0, 1, 2):
            raise ProtocolError(f"invalid mode {self.mode}")
        for name in (
            "drive_x", "drive_y", "drive_rotate", "arm_dx", "arm_dy",
            "arm_dz", "arm_pitch", "arm_yaw",
        ):
            _normalized(getattr(self, name), name)
        if self.laser_enable not in (0, 1):
            raise ProtocolError("laser_enable must be 0 or 1")
        if not 0 <= self.laser_power <= 100:
            raise ProtocolError("laser_power must be 0..100")
        if self.estop not in (0, 1):
            raise ProtocolError("estop must be 0 or 1")
        if self.heartbeat not in (0, 1):
            raise ProtocolError("heartbeat must be 0 or 1")

    def pack(self) -> bytes:
        self.validate()
        return struct.pack(self.FORMAT, *(
            self.seq, self.timestamp, self.mode,
            self.drive_x, self.drive_y, self.drive_rotate,
            self.arm_dx, self.arm_dy, self.arm_dz, self.arm_pitch, self.arm_yaw,
            self.laser_enable, self.laser_power, self.laser_speed,
            self.estop, self.heartbeat,
        ))

    @property
    def arm_normalized(self) -> tuple[float, float, float, float, float]:
        return tuple(
            _normalized(getattr(self, name), name)
            for name in ("arm_dx", "arm_dy", "arm_dz", "arm_pitch", "arm_yaw")
        )  # type: ignore[return-value]

    @property
    def drive_normalized(self) -> tuple[float, float, float]:
        return tuple(
            _normalized(getattr(self, name), name)
            for name in ("drive_x", "drive_y", "drive_rotate")
        )  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class TelemetryFrame:
    seq: int
    battery_percent: int
    latency_ms: int
    roll_deg: float
    pitch_deg: float
    joint_margins: tuple[int, int, int, int, int, int]
    interlock_bits: int
    laser_active: int

    FORMAT: ClassVar[str] = TELEMETRY_FRAME_FMT
    SIZE: ClassVar[int] = TELEMETRY_FRAME_SIZE

    def pack(self) -> bytes:
        if len(self.joint_margins) != 6:
            raise ProtocolError("joint_margins must contain six values")
        values = (self.battery_percent, *self.joint_margins)
        if not all(0 <= value <= 100 for value in values):
            raise ProtocolError("percentage values are outside 0..100")
        if self.laser_active not in (0, 1):
            raise ProtocolError("laser_active must be 0 or 1")
        if not 0 <= self.latency_ms <= 0xFFFF:
            raise ProtocolError("latency_ms must fit uint16")
        if not 0 <= self.interlock_bits <= 0xFF:
            raise ProtocolError("interlock_bits must fit uint8")
        return struct.pack(
            self.FORMAT,
            self.seq,
            self.battery_percent,
            self.latency_ms,
            self.roll_deg,
            self.pitch_deg,
            *self.joint_margins,
            self.interlock_bits,
            self.laser_active,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "TelemetryFrame":
        if len(data) != cls.SIZE:
            raise ProtocolError(f"telemetry frame must be {cls.SIZE} bytes, got {len(data)}")
        fields = struct.unpack(cls.FORMAT, data)
        return cls(
            seq=fields[0], battery_percent=fields[1], latency_ms=fields[2],
            roll_deg=fields[3], pitch_deg=fields[4],
            joint_margins=tuple(fields[5:11]),  # type: ignore[arg-type]
            interlock_bits=fields[11], laser_active=fields[12],
        )


def is_newer_sequence(new: int, old: int) -> bool:
    """RFC-1982-style comparison for uint16 sequence numbers."""
    delta = (new - old) & 0xFFFF
    return 0 < delta < 0x8000
