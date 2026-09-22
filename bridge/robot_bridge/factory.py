from __future__ import annotations

import importlib
from typing import Any, TypeVar

from .interfaces import IArmController, ICameraSource, ILaserController

T = TypeVar("T")

# Vendor modules stay outside service.py. Adding hardware means adding one class
# and one registry entry; BridgeService itself remains unchanged.
ARM_DRIVERS = {
    "ros_servo": "robot_bridge.hardware.ros_servo:RosServoArm",
    "piper6": "robot_bridge.hardware.piper6:PiperArmController",
    "piperH": "robot_bridge.hardware.piper_h:PiperHArmController",
}
CAMERA_DRIVERS = {
    "d435_process": "robot_bridge.hardware.camera_process:ProcessCamera",
    "none": "robot_bridge.hardware.camera_process:NoCamera",
    "d435": "robot_bridge.hardware.d435:D435Camera",
    "hikvision": "robot_bridge.hardware.hikvision:HikvisionCamera",
}
LASER_DRIVERS = {
    "noop": "robot_bridge.hardware.laser:NoOpLaserController",
}


def _construct(registry: dict[str, str], settings: dict[str, Any], kind: str) -> Any:
    options = dict(settings)
    name = options.pop("driver", None)
    if name not in registry:
        raise ValueError(f"unknown {kind} driver {name!r}; choose from {sorted(registry)}")
    module_name, class_name = registry[name].split(":", 1)
    cls = getattr(importlib.import_module(module_name), class_name)
    return cls(**options)


def create_arm(settings: dict[str, Any]) -> IArmController:
    instance = _construct(ARM_DRIVERS, settings, "arm")
    if not isinstance(instance, IArmController):
        raise TypeError("arm driver does not implement IArmController")
    return instance


def create_camera(settings: dict[str, Any]) -> ICameraSource:
    instance = _construct(CAMERA_DRIVERS, settings, "camera")
    if not isinstance(instance, ICameraSource):
        raise TypeError("camera driver does not implement ICameraSource")
    return instance


def create_laser(settings: dict[str, Any] | None) -> ILaserController:
    instance = _construct(LASER_DRIVERS, settings or {"driver": "noop"}, "laser")
    if not isinstance(instance, ILaserController):
        raise TypeError("laser driver does not implement ILaserController")
    return instance

