from __future__ import annotations

import argparse
from pathlib import Path
import socket
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_bridge.protocol import ControlFrame, TelemetryFrame  # noqa: E402


def scaled(value: str | float) -> int:
    value = float(value)
    if not -1.0 <= value <= 1.0:
        raise argparse.ArgumentTypeError("normalized values must be within -1..1")
    return round(value * 1000)


def main() -> None:
    parser = argparse.ArgumentParser(description="Send protocol-compatible UDP control frames")
    parser.add_argument("host")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--mode", type=int, choices=(0, 1, 2), default=0)
    for option in ("drive-x", "drive-y", "drive-rotate", "arm-dx", "arm-dy", "arm-dz", "arm-pitch", "arm-yaw"):
        parser.add_argument(f"--{option}", type=scaled, default=0)
    parser.add_argument("--laser-enable", action="store_true")
    parser.add_argument("--laser-power", type=int, default=0)
    parser.add_argument("--laser-speed", type=float, default=0.0)
    parser.add_argument("--estop", action="store_true")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.05)
    parser.add_argument("--wait-telemetry", action="store_true")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.wait_telemetry:
        sock.settimeout(1.0)
    timestamp = int(time.time() * 1000) & 0xFFFFFFFF
    for index in range(max(1, args.repeat)):
        frame = ControlFrame(
            seq=index & 0xFFFF, timestamp=(timestamp + round(index * args.interval * 1000)) & 0xFFFFFFFF,
            mode=args.mode, drive_x=args.drive_x, drive_y=args.drive_y,
            drive_rotate=args.drive_rotate, arm_dx=args.arm_dx, arm_dy=args.arm_dy,
            arm_dz=args.arm_dz, arm_pitch=args.arm_pitch, arm_yaw=args.arm_yaw,
            laser_enable=int(args.laser_enable), laser_power=args.laser_power,
            laser_speed=round(args.laser_speed * 10), estop=int(args.estop),
            heartbeat=index & 1,
        )
        sock.sendto(frame.pack(), (args.host, args.port))
        print(f"sent seq={frame.seq} bytes={frame.SIZE} estop={frame.estop}")
        if index + 1 < args.repeat:
            time.sleep(max(0.0, args.interval))
    if args.wait_telemetry:
        data, address = sock.recvfrom(2048)
        print(f"telemetry from {address}: {TelemetryFrame.unpack(data)}")


if __name__ == "__main__":
    main()
