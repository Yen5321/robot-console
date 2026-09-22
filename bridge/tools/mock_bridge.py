"""Software-only endpoint for cross-language tablet/bridge commissioning.

This tool uses the production BridgeService and wire protocol but no robot hardware.
It is intentionally outside robot_bridge/hardware so it cannot be selected by the
production config accidentally.
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_bridge.interfaces import IArmController, ICameraSource, ILaserController
from robot_bridge.service import BridgeService


class CommissioningArm(IArmController):
    def __init__(self) -> None:
        self._estopped = False
        self._home_pose = None
        self._last_jog = (0.0,) * 5

    def jog_cartesian(self, dx, dy, dz, dpitch, dyaw) -> None:
        if self._estopped:
            raise RuntimeError("software commissioning arm is estopped")
        self._last_jog = (dx, dy, dz, dpitch, dyaw)
        logging.getLogger(__name__).info("arm jog %s", self._last_jog)

    def set_height_delta(self, delta_mm: float) -> None: pass
    def get_joint_margins(self) -> list[float]: return [78, 72, 66, 59, 31, 64]
    def get_pose_safety_ok(self) -> bool: return not self._estopped
    def get_diagnostics(self) -> dict:
        return {"pose": [100, 0, 200, 0, 10, 0], "feedback_present": True,
                "enabled": [True] * 6, "status": int(self._estopped), "error": "", "ready": not self._estopped, "target": None, "feedback_age_ms": 0, "home_pose": self._home_pose}
    def enable(self): pass
    def recover(self): self._estopped = False
    def set_home(self): self._home_pose = [100, 0, 200, 0, 10, 0]
    def home(self):
        if self._home_pose is None: raise RuntimeError("No recorded home")
        return self._home_pose[:]
    def advance_home(self, goal, dt): return True
    def estop(self) -> None: self._estopped = True


class CommissioningCamera(ICameraSource):
    def __init__(self) -> None:
        self._jpeg = (Path(__file__).with_name("mock-frame.jpg")).read_bytes()

    def get_jpeg_frame(self) -> bytes:
        return self._jpeg

    def get_depth_frame(self): return None


class CommissioningLaser(ILaserController):
    def __init__(self) -> None: self._active = False
    def set_request(self, enabled: bool, power: int, speed: float, safe: bool) -> None:
        self._active = bool(enabled and safe)
    def force_off(self) -> None: self._active = False
    def is_active(self) -> bool: return self._active


def main() -> None:
    parser = argparse.ArgumentParser(description="software-only WPF/Track-B endpoint")
    parser.add_argument("--udp-port", type=int, default=19000)
    parser.add_argument("--http-port", type=int, default=18080)
    parser.add_argument("--session-delay-ms", type=int, default=0, help="Fake slow initial handshake for regression tests")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    class DelayedSessionService(BridgeService):
        def arm_action(self,payload,peer):
            if payload.get('action')=='session' and args.session_delay_ms and not getattr(self,'_delayed_once',False):
                self._delayed_once=True
                time.sleep(args.session_delay_ms/1000.)
            return super().arm_action(payload,peer)
    service = DelayedSessionService(
        CommissioningArm(), CommissioningCamera(), CommissioningLaser(),
        {
            "bind_host": "127.0.0.1", "udp_port": args.udp_port,
            "http_port": args.http_port, "telemetry_hz": 15,
            "command_timeout_s": 0.25, "sequence_reset_after_s": 2.0,
        },
        {
            "battery_percent": 88, "roll_deg": 1.25, "pitch_deg": -2.5,
            "end_distance_ok": True, "shield_in_position": True,
            "work_area_clear": True,
        },
    )
    service.start()
    print(f"READY udp={args.udp_port} mjpeg=http://127.0.0.1:{args.http_port}/stream.mjpg", flush=True)
    try:
        service.wait()
    except KeyboardInterrupt:
        pass
    finally:
        service.close()


if __name__ == "__main__":
    main()
