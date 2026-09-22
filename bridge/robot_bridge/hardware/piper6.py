from __future__ import annotations

import logging
import math
import json
from pathlib import Path
import threading
import time
from typing import Any, Callable

from ..interfaces import IArmController
from ..parameters import resolve_safety
from ..errors import ArmInputRejected, ArmNotReady


class PiperArmController(IArmController):
    """Piper6 adapter using AgileX piper_sdk C_PiperInterface_V2."""


    # Official SDK defaults, degrees. Keep telemetry meaningful even before the
    # optional on-device limit query has been integrated.
    JOINT_LIMITS_DEG = (
        (-150.0, 150.0), (0.0, 180.0), (-170.0, 0.0),
        (-100.0, 100.0), (-70.0, 70.0), (-120.0, 120.0),
    )

    def __init__(
        self,
        can_name: str = "can0",
        judge_flag: bool = False,
        dh_is_offset: int = 1,
        enable_on_start: bool = False,
        enable_timeout_s: float = 5.0,
        feedback_timeout_s: float = 2.0,
        nominal_command_period_s: float = 0.05,
        max_command_interval_s: float = 0.10,
        max_linear_speed_mm_s: float = 50.0,
        max_angular_speed_deg_s: float = 10.0,
        move_speed_percent: int = 10,
        deadband: float = 0.02,
        workspace_mm: dict[str, list[float]] | None = None,
        orientation_deg: dict[str, list[float]] | None = None,
        home_path: str = "recorded-home.json",
        safety_checks: dict | None = None,
        _sdk_interface: Any | None = None,
        _time_fn: Callable[[], float] = time.monotonic,
    ) -> None:
        self._log = logging.getLogger(__name__)
        self._lock = threading.RLock()
        self._time = _time_fn
        self._feedback_timeout_s = float(feedback_timeout_s)
        self._nominal_period = float(nominal_command_period_s)
        self._max_interval = float(max_command_interval_s)
        # v8 target-rate ceilings, negotiated before any client motion.
        self._linear_speed = min(50.0, max(2.0, float(max_linear_speed_mm_s)))
        self._angular_speed = min(10.0, max(1.0, float(max_angular_speed_deg_s)))
        self._send_pending = False
        self._max_interval = min(self._max_interval, 0.05)
        self._safety = resolve_safety(safety_checks)
        self._freshness_s = self._safety["feedback_max_age_s"]
        self._observations = {}
        self._fault_reason = ""
        self._moving = False
        self._monitor_until = 0.0
        self._last_submitted = None
        self._last_submission_at = None
        self._last_hold = None
        self._home_path = Path(home_path)
        self._home_pose = None
        if self._home_path.is_file():
            saved = json.loads(self._home_path.read_text(encoding="utf-8"))
            candidate = saved.get("pose")
            if isinstance(candidate, list) and len(candidate) == 6 and all(isinstance(v, (int,float)) and math.isfinite(v) for v in candidate):
                self._home_pose = list(candidate)
        self._speed_percent = max(1, min(100, int(move_speed_percent)))
        self._deadband = max(0.0, min(0.99, float(deadband)))
        self._workspace = workspace_mm or {
            "x": [-500.0, 500.0], "y": [-500.0, 500.0], "z": [0.0, 600.0]
        }
        self._orientation = orientation_deg or {
            "roll": [-180.0, 180.0], "pitch": [-90.0, 90.0],
            "yaw": [-180.0, 180.0],
        }
        self._target: list[float] | None = None
        self._last_jog_time: float | None = None
        self._priority_stop = threading.Event()
        self._estop_latched = False

        if _sdk_interface is None:
            try:
                from piper_sdk import C_PiperInterface_V2
            except ImportError as exc:
                raise RuntimeError(
                    "piper_sdk is required for the piper6 driver; "
                    "run: pip install -r requirements.txt"
                ) from exc
            self._piper = C_PiperInterface_V2(
                can_name=can_name,
                judge_flag=judge_flag,
                can_auto_init=True,
                dh_is_offset=dh_is_offset,
                start_sdk_joint_limit=True,
            )
        else:
            self._piper = _sdk_interface

        try:
            self._piper.ConnectPort()
            if enable_on_start:
                self._enable_arm(float(enable_timeout_s))
        except Exception:
            self.close()
            raise

    def _enable_arm(self, timeout_s: float) -> None:
        deadline = self._time() + timeout_s
        while self._time() < deadline:
            if self._priority_stop.is_set(): raise RuntimeError("Priority stop interrupted enable")
            if hasattr(self._piper, "EnablePiper"):
                if self._piper.EnablePiper():
                    return
            else:
                self._piper.EnableArm(7, 0x02)
                status = getattr(self._piper, "GetArmEnableStatus", lambda: [])()
                if status and all(status):
                    return
            time.sleep(0.01)
        raise RuntimeError("Piper6 did not report all joints enabled before timeout")

    @staticmethod
    def _clamp_normalized(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    def _apply_deadband(self, value: float) -> float:
        value = self._clamp_normalized(value)
        return 0.0 if abs(value) < self._deadband else value

    def _fresh(self, key, feedback):
        """Require an observed SDK timestamp change; positive Hz alone is insufficient.

        SDK timestamps use a different clock. Measure age of their last observed
        change using this process's monotonic clock; never compare clock domains.
        """
        now = self._time()
        stamp = float(getattr(feedback, "time_stamp", 0) or 0)
        previous = self._observations.get(key)
        if not math.isfinite(stamp) or stamp <= 0:
            self._observations.pop(key, None)
            return False
        if previous is None:
            self._observations[key] = (stamp, now, False)
        elif stamp != previous[0]:
            self._observations[key] = (stamp, now, True)
        _, changed_at, advanced = self._observations[key]
        return advanced and now - changed_at < self._freshness_s

    def _age_ms(self, key):
        value = self._observations.get(key)
        return round((self._time() - value[1]) * 1000, 1) if value and value[2] else None

    def _read_pose(self) -> tuple[list[float], bool]:
        with self._lock:
            feedback = self._piper.GetArmEndPoseMsgs()
            p = feedback.end_pose
            values = [getattr(p, k) / 1000.0 for k in
                      ("X_axis", "Y_axis", "Z_axis", "RX_axis", "RY_axis", "RZ_axis")]
            live = self._fresh("pose", feedback) and all(math.isfinite(v) for v in values)
            return values, live

    def _snapshot(self):
        pose, live = self._read_pose()
        status = self._piper.GetArmStatus()
        status_live = self._fresh("status", status)
        motors = self._piper.GetArmLowSpdInfoMsgs()
        motors_live = self._fresh("motors", motors)
        flags = [bool(getattr(motors, f"motor_{i}").foc_status.driver_enable_status) for i in range(1, 7)]
        code = int(status.arm_status.arm_status)
        mode = int(getattr(status.arm_status, "ctrl_mode", -1))
        error = ""
        if self._fault_reason:
            error = self._fault_reason
        elif not live or not status_live or not motors_live:
            missing = [name for name, ok in (("pose", live), ("status", status_live), ("motors", motors_live)) if not ok]
            error = f"反馈超时或尚未更新 / Feedback not fresh (<{self._freshness_s*1000:g} ms required): " + ", ".join(missing)
        elif code != 0:
            names = {1:"急停状态",2:"逆运动学无解",3:"奇异点",4:"目标关节角超限",5:"关节通信异常",6:"关节制动未释放",7:"碰撞",8:"示教超速",9:"关节状态异常"}
            error = f"控制器故障：{names.get(code, '未知故障')}（状态码 {code}）"
        elif mode != 1:
            error = f"控制模式 {mode}（2=示教）：请点击使能并切换 CAN 控制"
        elif not all(flags):
            error = "未使能：" + ", ".join(f"J{i}" for i,flag in enumerate(flags,1) if not flag)
        return pose, live, code, mode, flags if motors_live else None, error

    @property
    def motion_may_be_active(self):
        return self._moving or self._send_pending or self._time() < self._monitor_until

    def _require_ready(self):
        pose, _, _, _, _, error = self._snapshot()
        if error:
            raise ArmNotReady(error)
        return pose

    def _sync_target_from_feedback(self):
        # Never block command processing while waiting for feedback.
        self._target = self._require_ready()

    def _send_target(self) -> None:
        if self._priority_stop.is_set(): raise RuntimeError("Priority stop blocks target")
        assert self._target is not None
        self._validate_target(self._target)
        # Cartesian straight-line interpolation, keeping untouched pose axes fixed.
        self._send_pending = True
        self._piper.ModeCtrl(0x01, 0x02, self._speed_percent, 0x00)
        wire = [round(value * 1000.0) for value in self._target]
        self._piper.EndPoseCtrl(*wire)
        self._send_pending = False
        self._monitor_until = self._time() + self._safety["hold_monitor_s"]
        self._last_submitted = self._target.copy()
        self._last_submission_at = self._time()
        self._log.info("ARM_TARGET t=%.6f pose=%s", self._time(), self._target)

    def _validate_target(self, target: list[float]) -> None:
        checks = (
            ("X", target[0], self._workspace["x"]),
            ("Y", target[1], self._workspace["y"]),
            ("Z", target[2], self._workspace["z"]),
            ("roll", target[3], self._orientation["roll"]),
            ("pitch", target[4], self._orientation["pitch"]),
            ("yaw", target[5], self._orientation["yaw"]),
        )
        for name, value, bounds in checks:
            if not self._inside(value, bounds):
                raise ArmInputRejected(
                    f"refusing Piper6 target: {name}={value:.3f} outside "
                    f"{float(bounds[0]):.3f}..{float(bounds[1]):.3f}"
                )

    def jog_cartesian(self, dx, dy, dz, dpitch, dyaw) -> None:
        with self._lock:
            if self._estop_latched:
                raise RuntimeError(self._fault_reason or "Emergency stop latched")
            # v8 applies input deadband on the tablet BEFORE speed scaling.
            controls = [self._clamp_normalized(v) for v in (dx, dy, dz, dpitch, dyaw)]
            if not any(controls):
                self.hold_position()
                return
            pose = self._require_ready()
            if not self._moving:
                self._target = pose.copy()
                self._last_jog_time = None
            now = self._time()
            dt = self._nominal_period if self._last_jog_time is None else now - self._last_jog_time
            dt = max(0.0, min(self._max_interval, dt))
            self._last_jog_time = now
            norm = math.sqrt(sum(v*v for v in controls[:3]))
            if norm > 1:
                controls[:3] = [v/norm for v in controls[:3]]
            anorm = math.hypot(controls[3], controls[4])
            if anorm > 1:
                controls[3:] = [v/anorm for v in controls[3:]]
            previous = self._target.copy()
            for index in range(3):
                self._target[index] += controls[index] * self._linear_speed * dt
            self._target[4] += controls[3] * self._angular_speed * dt
            self._target[5] += controls[4] * self._angular_speed * dt
            # Tracking errors are always recorded; configured optional guards may reject them.
            self._last_jog_tracking = {"time": self._time(), "actual": pose.copy(),
                "attempted_target": self._target.copy(), "input": controls.copy(),
                "rotation_error_deg": self.rotation_distance_deg(self._target[3:], pose[3:]),
                "translation_error_mm": math.dist(self._target[:3], pose[:3]),
                "feedback_age_ms": self._age_ms("pose")}
            exceeded = []
            if self._safety["jog_translation_enabled"] and self._last_jog_tracking["translation_error_mm"] > self._safety["jog_translation_mm"]:
                exceeded.append("translation")
            if self._safety["jog_rotation_enabled"] and self._last_jog_tracking["rotation_error_deg"] > self._safety["jog_rotation_deg"]:
                exceeded.append("rotation")
            if exceeded:
                self._last_rejection = dict(self._last_jog_tracking, checks=exceeded)
                self._target = previous
                raise RuntimeError(f"Configured jog tracking check exceeded ({', '.join(exceeded)}): {self._last_rejection}")
            try:
                self._send_target()
                self._moving = True
            except Exception:
                self._target = previous
                raise

    def hold_position(self):
        with self._lock:
            if not self._moving:
                return
            try:
                pose = self._require_ready()
                self._target = pose.copy()
                self._send_target()
                self._last_hold = {"time": self._time(), "pose": pose.copy(),
                                   "max_translation_mm": 0.0, "max_rotation_deg": 0.0}
                self._log.info("ARM_HOLD %s", self._last_hold)
                self._moving = False
                self._monitor_until = self._time() + self._safety["hold_monitor_s"]
                self._last_jog_time = None
            except Exception as exc:
                self.fault_stop("Hold failed: " + str(exc))
                raise

    def check_feedback(self):
        with self._lock:
            pose, live, _, _, _, error = self._snapshot()
            monitored = self._moving or self._time() < self._monitor_until
            if monitored and error and not self._estop_latched:
                self.fault_stop(error)
                raise RuntimeError(error)
            if self._last_hold and not self._moving and live:
                hold = self._last_hold
                distance = math.dist(pose[:3], hold["pose"][:3])
                rotation = self.rotation_distance_deg(pose[3:], hold["pose"][3:])
                hold["max_translation_mm"] = max(hold["max_translation_mm"], distance)
                hold["max_rotation_deg"] = max(hold["max_rotation_deg"], rotation)
                if monitored and ((self._safety["hold_translation_enabled"] and distance > self._safety["hold_translation_mm"]) or (self._safety["hold_rotation_enabled"] and rotation > self._safety["hold_rotation_deg"])) and not self._estop_latched:
                    self.fault_stop(f"Hold exceeded commissioning tolerance: translation={distance:.3f} mm (limit {self._safety['hold_translation_mm']:g}), rotation={rotation:.3f} deg (limit {self._safety['hold_rotation_deg']:g})")
                    raise RuntimeError(self._fault_reason)

    def request_priority_stop(self,reason):
        # Deliberately bypass the pose/enable lock. Software best effort, not an MCU watchdog.
        self._priority_stop.set()
        if not self._estop_latched:
            self._fault_reason=str(reason)
        self._estop_latched=True
        self._moving=False
        self._target=None
        self._piper.EmergencyStop(0x01)

    def fault_stop(self, reason):
        with self._lock:
            if self._estop_latched:
                return
            self._fault_reason = str(reason)
            self._estop_latched = True
            self._moving = False
            self._send_pending = False
            self._target = None
            self._log.error("ARM_FAULT t=%.6f reason=%s", self._time(), reason)
            self._piper.EmergencyStop(0x01)

    def set_height_delta(self, delta_mm: float) -> None:
        raise RuntimeError("Discrete moves disabled; use continuous jog")

    def get_joint_margins(self) -> list[float]:
        feedback = self._piper.GetArmJointMsgs()
        if not self._fresh("joints", feedback):
            return [0.0] * 6
        state = feedback.joint_state
        angles = [getattr(state, f"joint_{index}") / 1000.0 for index in range(1, 7)]
        margins: list[float] = []
        for angle, (lower, upper) in zip(angles, self.JOINT_LIMITS_DEG):
            margin = 200.0 * min(angle - lower, upper - angle) / (upper - lower)
            margins.append(max(0.0, min(100.0, margin)))
        return margins

    @staticmethod
    def rotation_distance_deg(first, second):
        """Shortest SO(3) angle for degree RPY, R = Rz(yaw) Ry(pitch) Rx(roll).

        Euler component differences are not physical rotation distances near
        pitch +/-90 degrees. Absolute quaternion dot also handles wrap/sign.
        """
        def quaternion(rpy):
            r, p, y = (math.radians(v) / 2 for v in rpy)
            cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
            return (cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy,
                    cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy)
        a, b = quaternion(first), quaternion(second)
        dot = min(1.0, max(0.0, abs(sum(x*y for x,y in zip(a,b)))))
        return math.degrees(2 * math.acos(dot))

    @staticmethod
    def _inside(value: float, bounds: list[float]) -> bool:
        return len(bounds) == 2 and float(bounds[0]) <= value <= float(bounds[1])

    def get_pose_safety_ok(self) -> bool:
        return self.get_diagnostics()["ready"]

    def get_diagnostics(self) -> dict:
        with self._lock:
            result = {"pose": None, "feedback_present": False, "target": self._last_submitted,
                      "feedback_age_ms": None, "enabled": None, "status": None,
                      "ctrl_mode": None, "error": "", "ready": False,
                      "fault_latched": self._estop_latched, "moving_input": self._moving,
                      "last_hold": self._last_hold, "linear_speed_mm_s": self._linear_speed,
                      "angular_speed_deg_s": self._angular_speed,
                      "target_sent_at": self._last_submission_at, "home_pose": self._home_pose,
                      "last_rejection": getattr(self, "_last_rejection", None),
                      "jog_translation_lead_limit_mm": self._safety["jog_translation_mm"] if self._safety["jog_translation_enabled"] else None,
                      "jog_translation_lead_check_enabled": self._safety["jog_translation_enabled"],
                      "jog_angular_lead_check_enabled": self._safety["jog_rotation_enabled"],
                      "effective_safety_checks": self._safety.copy(),
                      "input_deadband": self._deadband,
                      "motion_state": "fault" if self._estop_latched else "moving" if self._moving else "holding" if self.motion_may_be_active else "idle",
                      "motion_may_be_active": self.motion_may_be_active,
                      "last_jog_tracking": getattr(self, "_last_jog_tracking", None),
                      "stop_behavior": "SDK emergency stop; position retention NOT verified"}
            try:
                pose, live, code, mode, flags, error = self._snapshot()
                result.update(pose=pose if live else None, feedback_present=live,
                              feedback_age_ms=self._age_ms("pose"), status=code,
                              ctrl_mode=mode, enabled=flags, error=error)
                result["blocking_code"] = ("fault_latched" if self._estop_latched else
                    "feedback_stale" if "not fresh" in error else "controller_fault" if code!=0 else
                    "teach_mode" if mode==2 else "control_mode" if mode!=1 else
                    "not_enabled" if flags is not None and not all(flags) else "")
                if not error:
                    self._validate_target(pose)
                    result["ready"] = True
            except Exception as exc:
                result["error"] = str(exc)
                result["blocking_code"] = "input_bounds" if isinstance(exc,ArmInputRejected) else "feedback_error"
            return result

    def get_sensor_snapshot(self):
        with self._lock:
            read=getattr(self._piper,"GetArmHighSpdInfoMsgs",None)
            if read is None: return {}
            feedback=read()
            if not self._fresh("motor_currents",feedback): return {}
            values=[getattr(feedback,f"motor_{i}").current/1000.0 for i in range(1,7)]
            return {"motor_current_a":values} if all(math.isfinite(v) for v in values) else {}

    def set_home(self):
        with self._lock:
            self.hold_position()
            pose = self._require_ready()
            self._validate_target(pose)
            stage = self._home_path.with_suffix(".json.tmp")
            try:
                stage.write_text(json.dumps({"pose":pose,"recorded_at":time.time()},indent=2),encoding="utf-8")
                stage.replace(self._home_path)
            except OSError as exc:
                raise ArmInputRejected("零位文件未保存，请检查路径/写权限："+str(exc)) from exc
            self._home_pose = pose.copy()
            return pose

    def home(self):
        with self._lock:
            self._require_ready()
            if self._home_pose is None:
                raise ArmInputRejected("No recorded home; use Set home first")
            self._validate_target(self._home_pose)
            self.hold_position()
            return self._home_pose.copy()

    def advance_home(self, goal, dt):
        with self._lock:
            pose = self._require_ready()
            if not self._moving: self._target = pose.copy()
            translation = [goal[i]-self._target[i] for i in range(3)]
            rotation = [goal[i]-self._target[i] for i in range(3,6)]
            # One interpolation fraction keeps the XYZ/RPY target path synchronized.
            fraction = min(1.0, 4.*dt/max(math.sqrt(sum(v*v for v in translation)),1e-9),
                           2.*dt/max(math.sqrt(sum(v*v for v in rotation)),1e-9))
            target = [v+fraction*(g-v) for v,g in zip(self._target,goal)]
            if math.dist(target[:3],pose[:3]) > self._safety["home_translation_lead_mm"] or self.rotation_distance_deg(target[3:],pose[3:]) > self._safety["home_rotation_lead_deg"]:
                raise RuntimeError("Return target lead exceeded; arm did not follow")
            self._target = target
            self._send_target()
            self._moving = True
            if all(abs(a-b)<1e-6 for a,b in zip(target,goal)) and math.dist(pose[:3],goal[:3])<=self._safety["home_position_tolerance_mm"] and self.rotation_distance_deg(pose[3:],goal[3:])<=self._safety["home_rotation_tolerance_deg"]:
                self.hold_position()
                return True
            return False

    def enable(self):
        with self._lock:
            if self._estop_latched:
                raise RuntimeError("Emergency/fault latched; use explicit recovery first")
            # Motor enable alone does not leave the rear-button teaching mode.
            # Explicit operator action takes CAN ownership at the actual pose.
            self._wait_normal(require_enabled=False)
            pose, live = self._read_pose()
            if not live: raise RuntimeError("No fresh pose for CAN takeover")
            self._target = pose.copy()
            self._send_target()
            self._enable_arm(3.0)
            self._wait_normal(require_enabled=True, require_can=True)
            self._target = None
            self._moving = False
            self._last_jog_time = None

    def _wait_normal(self, require_enabled, require_can=False):
        deadline = time.monotonic()+3.0
        last_error = "waiting for fresh feedback"
        while time.monotonic() < deadline:
            if self._priority_stop.is_set(): raise RuntimeError("Priority stop interrupted recovery")
            status=self._piper.GetArmStatus()
            motors=self._piper.GetArmLowSpdInfoMsgs()
            _, pose_live=self._read_pose()
            status_live=self._fresh("status",status)
            motors_live=self._fresh("motors",motors)
            enabled=all(getattr(motors,f"motor_{i}").foc_status.driver_enable_status for i in range(1,7))
            code=status.arm_status.arm_status
            mode = int(getattr(status.arm_status, "ctrl_mode", -1))
            if status_live and motors_live and pose_live and code==0 and (enabled or not require_enabled) and (mode==1 or not require_can): return
            last_error=f"status={code}, control_mode={mode} (CAN=1), enabled={enabled}, feedback_fresh={pose_live and status_live and motors_live}"
            time.sleep(.02)
        raise RuntimeError("Controller recovery/enable not confirmed: "+last_error)

    def recover(self):
        with self._lock:
            self._priority_stop.clear()
            try:
                self._piper.EmergencyStop(0x02)
                self._wait_normal(require_enabled=False)
                pose,live=self._read_pose()
                if not live: raise RuntimeError("No fresh pose for recovery hold")
                self._target=pose.copy()
                self._send_target()
                self._enable_arm(3.0)
                self._wait_normal(require_enabled=True, require_can=True)
                if self._priority_stop.is_set(): raise RuntimeError("Recovery cancelled by priority stop")
                self._estop_latched = False
                self._fault_reason = ""
                self._moving = False
                self._target = None
                self._last_hold = None
                self._monitor_until = 0
                self._last_jog_time = None
            except Exception as exc:
                self._estop_latched=True
                self._fault_reason="Recovery failed: "+str(exc)
                self._piper.EmergencyStop(0x01)
                raise

    def estop(self) -> None:
        self.fault_stop("Emergency stop requested; explicit local recovery required")

    def close(self) -> None:
        try:
            if self._moving:
                self.fault_stop("Controller closing during motion")
        finally:
            disconnect = getattr(self._piper, "DisconnectPort", None)
            if disconnect:
                disconnect()
