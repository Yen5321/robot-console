from __future__ import annotations

import logging
import socket
import struct
import threading
import time
from typing import Any
from pathlib import Path
from logging.handlers import RotatingFileHandler, QueueHandler, QueueListener
import queue
from .wire_v6 import SessionGuard, decode_control, ack, telemetry as pack_telemetry
import json
import secrets
from collections import deque
from dataclasses import replace

from .interfaces import IArmController, ICameraSource, ILaserController
from .protocol import ControlFrame, ProtocolError, TelemetryFrame, is_newer_sequence
from .video import MjpegServer
from .parameters import effective_config, safety_schema
from .errors import ArmInputRejected, ArmNotReady

class DroppingQueueHandler(QueueHandler):
    """Never fall back to synchronous stderr if the audit disk falls behind."""
    dropped=0
    def enqueue(self,record):
        try: self.queue.put_nowait(record)
        except queue.Full: self.dropped+=1

class BridgeService:
    def __init__(
        self,
        arm: IArmController,
        camera: ICameraSource,
        laser: ILaserController,
        network: dict[str, Any],
        telemetry: dict[str, Any] | None = None,
        *,
        udp_socket: socket.socket | None = None,
        start_video: bool = True,
        configuration: dict | None = None,
        _time_fn=time.monotonic,
        _legacy_test_transport=False,
    ) -> None:
        self.arm, self.camera, self.laser = arm, camera, laser
        self._configuration = effective_config(configuration) if configuration is not None else None
        self._legacy_test_transport=_legacy_test_transport
        self._cached_telemetry=None
        self._wire_at=None;self._action_busy=False;self._urgent_epoch=0
        self._ingress_lock=threading.Lock()
        self._session=None;self._session_request=None;self._wire_rejected=0
        self._audit_listener=None

        self._time = _time_fn
        self._control_lock = threading.RLock()
        self._fault_latched = False
        self._fault_reason = ""
        self._fault_source = ""
        self._input_rearm_required = False
        self._action_token = secrets.token_urlsafe(24)
        self._action_results = {}
        self._home_goal = None
        self._home_tick_at = 0.0
        self._audit = None
        self._recent_records = deque(maxlen=300)
        self._recent_lock = threading.Lock()
        if network.get("audit_path"):
            audit_path = Path(network["audit_path"])
            audit_path.parent.mkdir(parents=True, exist_ok=True)
            self._audit = logging.getLogger(f"robot_bridge.audit.{id(self)}")
            self._audit.setLevel(logging.INFO)
            self._audit.propagate = False
            self._audit_queue=queue.Queue(maxsize=4096)
            self._audit_file=RotatingFileHandler(audit_path,maxBytes=10000000,backupCount=5,encoding="utf-8")
            self._audit.addHandler(DroppingQueueHandler(self._audit_queue))
            self._audit_listener=QueueListener(self._audit_queue,self._audit_file)
            self._audit_listener.start()
        self._host = str(network.get("bind_host", "0.0.0.0"))
        self._udp_port = int(network.get("udp_port", 9000))
        self._http_port = int(network.get("http_port", 8080))
        self._telemetry_hz = float(network.get("telemetry_hz",10.0))
        self._command_timeout_s = min(0.25, float(network.get("command_timeout_s", 0.25)))
        self._sequence_reset_after_s = float(network.get("sequence_reset_after_s", 2.0))
        if not 10.0 <= self._telemetry_hz <= 20.0:
            raise ValueError("telemetry_hz must be within protocol range 10..20")
        if self._command_timeout_s <= 0 or self._sequence_reset_after_s <= 0:
            raise ValueError("command/sequence timeouts must be positive")
        self._telemetry = telemetry or {}
        self._socket = udp_socket or socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._owns_socket = udp_socket is None
        if self._owns_socket:
            self._socket.bind((self._host, self._udp_port))
        self._socket.settimeout(0.25)
        try: self._socket.setsockopt(socket.IPPROTO_IP,socket.IP_TOS,0xb8) # DSCP EF, if network honors it
        except OSError: pass
        self._video = (
            MjpegServer(camera, self._host, self._http_port, max_fps=30.0, diagnostics=self.get_diagnostics, actions=self.arm_action, configuration=self.get_configuration, recent_logs=self.get_recent_logs)
            if start_video else None
        )
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._last_client: tuple[str, int] | None = None
        self._sequence_client: tuple[str, int] | None = None
        self._last_seq: int | None = None
        self._last_heartbeat: int | None = None
        self._last_received_at: float | None = None
        self._latest_control: ControlFrame | None = None
        self._watchdog_tripped = False
        self._telemetry_seq = 0
        self._arm_error = ""
        self._arm_command_state = "idle"
        self._threads: list[threading.Thread] = []
        self._log = logging.getLogger(__name__)

    def start(self) -> None:
        if self._video:
            self._video.start()
        self._threads = [
            threading.Thread(target=self._udp_loop, name="udp-control", daemon=True),
            threading.Thread(target=self._telemetry_loop, name="udp-telemetry", daemon=True),
            threading.Thread(target=self._sample_telemetry_loop,name="sensor-sampling",daemon=True),
            threading.Thread(target=self._safety_loop, name="arm-safety", daemon=True),
            threading.Thread(target=self._wire_watchdog_loop,name="wire-watchdog",daemon=True),
        ]
        for thread in self._threads:
            thread.start()
        self._log.info(
            "bridge listening UDP %s:%d, MJPEG http://%s:%d/stream.mjpg",
            self._host, self._udp_port, self._host, self._http_port,
        )

    def wait(self) -> None:
        while not self._stop.wait(1.0):
            pass

    def _udp_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, address = self._socket.recvfrom(2048)
                batch=[(data,address)]
                self._socket.setblocking(False)
                try:
                    for _ in range(63):
                        try: batch.append(self._socket.recvfrom(2048))
                        except BlockingIOError: break
                finally: self._socket.settimeout(.25)
                candidates=[]
                for packet,peer in batch:
                    try:
                        sid,alive,f=decode_control(packet)
                        if self._session and sid==self._session.sid and peer==self._session.peer:
                            if f.estop: self.process_datagram(packet,peer)
                            else: candidates.append((packet,peer,f.seq))
                    except ProtocolError: self._wire_rejected+=1
                if candidates:
                    latest=candidates[0]
                    for candidate in candidates[1:]:
                        if is_newer_sequence(candidate[2],latest[2]): latest=candidate
                    self.process_datagram(latest[0],latest[1])
            except socket.timeout:
                continue
            except ConnectionResetError:
                # Windows reports an ICMP port-unreachable from a departed UDP
                # client as WSAECONNRESET. It is not a server socket failure.
                self._log.debug("UDP peer disappeared; keep listening for the next tablet")
                continue
            except OSError:
                if not self._stop.is_set():
                    self._log.exception("UDP receive failed")
                return
            except Exception as exc:
                self._log.exception("unexpected control processing failure")
                self._handle_arm_rejection(exc)

    def get_configuration(self):
        if (self._configuration or {}).get("arm",{}).get("driver")=="ros_servo":
            return {"bridge_build":"8.1-piper-l-cpv","read_only":True,"restart_required":True,
                    "effective_config":self._configuration,"motion":self.get_diagnostics()["arm"].get("effective_motion_config",{}),
                    "fixed_protections":["manual_stop","control_timeout","original_input_age","feedback_age","servo_output_age","joint_limits","workspace_bounds","home_tracking"],
                    "real_motion_available":False}
        return {"bridge_build": "8.1-piper-l-cpv", "read_only": True, "restart_required": True,
                "effective_config": self._configuration, "safety_parameter_schema": safety_schema(),
                "fixed_protections": ["manual_emergency_stop", "control_timeout", "feedback_freshness", "controller_status", "hold_feedback_monitor", "home_tracking", "workspace_bounds"]}

    def get_recent_logs(self):
        with self._recent_lock:
            return {"bridge_build":"8.1-piper-l-cpv", "records": list(self._recent_records)}

    def _urgent_fault(self,reason):
        self._urgent_epoch+=1
        first = not self._fault_latched
        if not self._fault_latched:
            self._fault_reason=self._arm_error=reason
            self._fault_source="watchdog" if "timeout" in reason or "heartbeat" in reason else "remote_stop"
            self._fault_latched=True
            self._record("fault", reason=reason)
        self._home_goal=None;self._arm_command_state="fault"
        try:
            self.laser.force_off()
            stop=getattr(self.arm,"request_priority_stop",self.arm.fault_stop)
            if first or reason == "Remote emergency stop": stop(self._fault_reason)
        except Exception: self._log.exception("Priority stop send failed")

    def _wire_watchdog_loop(self):
        while not self._stop.wait(.01):
                if self._wire_at is not None and self._time()-self._wire_at>=self._command_timeout_s and not self._fault_latched:
                    self._urgent_fault(f"Control timeout >={self._command_timeout_s*1000:g} ms (independent software watchdog)")

    def process_datagram(self,data,address):
        if self._legacy_test_transport:
            with self._control_lock: return self._process_datagram(data,address)
        try:
            with self._ingress_lock:
                sid,alive,frame=decode_control(data)
                if self._session is None: raise ProtocolError("session handshake required")
                self._session.accept(sid,address,frame,int(self._time()*1000)&0xffffffff)
                self._wire_at=self._time()
            if frame.estop or not alive:
                self._urgent_fault("Remote emergency stop" if frame.estop else "Tablet UI heartbeat expired")
                self._socket.sendto(ack(sid,frame,2),address)
                return False
            if self._action_busy:
                if frame.mode!=0 or any(frame.arm_normalized+frame.drive_normalized) or frame.laser_enable:
                    self._urgent_fault("Non-neutral input during management action")
                    return False
                self._last_client=address;self._latest_control=frame;self._last_received_at=self._time()
                self._socket.sendto(ack(sid,frame,1),address)
                return True
            with self._control_lock:
                self._last_seq=None;self._sequence_client=None
                accepted=self._process_datagram(frame.pack(),address)
            self._socket.sendto(ack(sid,frame,1 if accepted else 2),address)
            return accepted
        except ProtocolError:
            self._wire_rejected+=1
            return False

    def _process_datagram(self, data: bytes, address: tuple[str, int]) -> bool:
        """Validate/process one datagram. Returns whether an ordinary frame was accepted."""
        try:
            frame = ControlFrame.unpack(data)
        except ProtocolError as exc:
            self._log.warning("discard malformed control packet from %s: %s", address, exc)
            return False

        # Safety takes precedence over ordering: a duplicated emergency packet
        # must still stop both actuators immediately.
        if frame.estop:
            self._latch_fault("Remote emergency stop from " + str(address))
            with self._lock:
                self._last_client = address
                self._last_received_at = self._time()
                self._latest_control = None
                self._watchdog_tripped = True
            self._log.error("EMERGENCY STOP requested by %s", address)
            return False

        now = self._time()
        self._check_timeout(now)
        if self._fault_latched:
            with self._lock:
                self._last_client = address
                # Zero standby packets allow an explicit action to prove neutral input.
                if frame.mode == 0 and not any(frame.arm_normalized+frame.drive_normalized) and not frame.laser_enable:
                    self._last_received_at = now
                    self._latest_control = frame
                    note_control=getattr(self.arm,"note_control",None)
                    if note_control is not None:
                        try: note_control(now)
                        except Exception: self._log.debug("Fault-state heartbeat unavailable",exc_info=True)
            return False
        with self._lock:
            sequence_session_active = (
                self._sequence_client == address
                and self._last_received_at is not None
                and now - self._last_received_at <= self._sequence_reset_after_s
            )
            if sequence_session_active and self._last_seq is not None and not is_newer_sequence(frame.seq, self._last_seq):
                self._log.debug("discard old/duplicate sequence %d (last %d)", frame.seq, self._last_seq)
                return False
            self._last_seq = frame.seq
            self._sequence_client = address
            self._last_heartbeat = frame.heartbeat
            self._last_client = address
            self._last_received_at = now
            self._latest_control = frame
            self._watchdog_tripped = False

        note_control=getattr(self.arm,"note_control",None)
        if note_control is not None:
            try: note_control(now)
            except Exception as exc:
                self._handle_arm_rejection(exc)
                return False
        if self._home_goal is not None and any(frame.arm_normalized):
            self._home_goal = None
            self.arm.hold_position()
        if self._home_goal is not None:
            self._arm_command_state = "returning_home"
        elif frame.mode == 2:
            try:
                if self._input_rearm_required and any(frame.arm_normalized):
                    return False
                if not any(frame.arm_normalized): self._input_rearm_required = False
                self.arm.jog_cartesian(*frame.arm_normalized)
                if any(frame.arm_normalized):
                    self._arm_error = ""
                    self._arm_command_state = "submitted"
                else:
                    self._arm_command_state = "idle"
            except Exception as exc:
                self._arm_error = str(exc)
                self._arm_command_state = "rejected"
                self._log.error("arm command rejected: %s", exc)
                self._handle_arm_rejection(exc)
        else:
            self._input_rearm_required = False
            try:
                self.arm.hold_position()
                self._arm_command_state = "holding"
            except Exception as exc:
                self._latch_fault("Mode-change hold failed: " + str(exc))
        self._record("control", sequence=frame.seq, mode=frame.mode, input=frame.arm_normalized)
        if self._fault_latched:
            return False

        bits = self._interlock_bits(frame)
        self.laser.set_request(
            bool(frame.laser_enable), frame.laser_power, frame.laser_speed / 10.0,
            safe=(bits & 0x1F) == 0x1F,
        )
        return True

    def _handle_arm_rejection(self, exc):
        active = bool(getattr(self.arm, "motion_may_be_active", False))
        self._input_rearm_required = True
        self._arm_error = str(exc)
        self._arm_command_state = "blocked"
        if isinstance(exc, ArmInputRejected):
            if active and isinstance(exc, ArmNotReady):
                self._latch_fault("Motion feedback/state failure: " + str(exc), "arm_state")
            elif active:
                try: self.arm.hold_position()
                except Exception as hold_error: self._latch_fault("Rejected-target hold failed: " + str(hold_error), "hold")
            self._record("rejected", reason=str(exc), motion_was_active=active)
        elif active or getattr(self.arm, "_estop_latched", False):
            self._latch_fault("Arm command failed: " + str(exc), "arm_command")
        else:
            self._record("rejected", reason=str(exc), motion_was_active=False)

    def _interlock_bits(self, control: ControlFrame | None = None) -> int:
        if control is None:
            with self._lock:
                control = self._latest_control
                last_received = self._last_received_at
        else:
            with self._lock:
                last_received = self._last_received_at
        control_is_fresh = (
            last_received is not None
            and self._time() - last_received <= self._command_timeout_s
        )
        vehicle_stopped = (
            control_is_fresh and control is not None
            and control.mode != 1 and not any(control.drive_normalized)
        )
        bits = int(vehicle_stopped) << 0
        bits |= int(self.arm.get_pose_safety_ok()) << 1
        bits |= int(bool(self._telemetry.get("end_distance_ok", False))) << 2
        bits |= int(bool(self._telemetry.get("shield_in_position", False))) << 3
        bits |= int(bool(self._telemetry.get("work_area_clear", False))) << 4
        return bits

    def get_diagnostics(self) -> dict:
        read = getattr(self.arm, "get_diagnostics", None)
        arm = read() if read else {"error": "arm diagnostics unsupported"}
        return {"bridge_version": "8.1-piper-l-cpv", "bridge_build": "8.1-piper-l-cpv", "arm": arm,
                "control_profile": "piper-l-cpv-v8.1",
                "motion_backend": "ros2_moveit_servo", "real_executor_abi": "piper_l:V189:rad_s:sdk_signs",
                "motion_capabilities": {"linear_speed_mm_s": arm.get("linear_speed_mm_s",50.), "angular_speed_deg_s": arm.get("angular_speed_deg_s",10.), "input_deadband": arm.get("input_deadband",.02)},
                "blocking_reason": arm.get("error","") or (self._arm_error if self._input_rearm_required else ""),
                "blocking_code": "fault_latched" if self._fault_latched else arm.get("blocking_code","") or ("rearm_required" if self._input_rearm_required else ""),
                "input_rearm_required": self._input_rearm_required, "fault_source": self._fault_source,
                "laser_hardware_connected": False,
                "effective_network": {"telemetry_hz": self._telemetry_hz, "command_timeout_s": self._command_timeout_s},
                "last_arm_error": self._arm_error, "fault_latched": self._fault_latched,
                "fault_reason": self._fault_reason, "command_state": self._arm_command_state,
                "client": self._last_client, "monotonic_time": self._time(),
                "action_token": self._action_token, "protocol":6, "wire_rejected":self._wire_rejected, "session": self._session.sid if self._session else None, "returning_home": self._home_goal is not None}

    def build_telemetry(self) -> TelemetryFrame:
        with self._lock:
            last_received = self._last_received_at
            seq = self._telemetry_seq
            self._telemetry_seq = (self._telemetry_seq + 1) & 0xFFFF
        age_ms = 0xFFFF if last_received is None else min(
            0xFFFF, max(0, round((self._time() - last_received) * 1000.0))
        )
        margins = self.arm.get_joint_margins()
        if len(margins) != 6:
            self._log.error("arm returned %d joint margins; reporting all zero", len(margins))
            margins = [0.0] * 6
        wire_margins = tuple(max(0, min(100, round(value))) for value in margins)
        return TelemetryFrame(
            seq=seq,
            battery_percent=max(0, min(100, int(self._telemetry.get("battery_percent", 0)))),
            latency_ms=age_ms,
            roll_deg=float(self._telemetry.get("roll_deg", 0.0)),
            pitch_deg=float(self._telemetry.get("pitch_deg", 0.0)),
            joint_margins=wire_margins,  # type: ignore[arg-type]
            interlock_bits=self._interlock_bits(),
            laser_active=int(self.laser.is_active()),
        )

    def arm_action(self, payload, peer):
        with self._control_lock:
            if not isinstance(payload,dict) or not secrets.compare_digest(str(payload.get("token","")),self._action_token):
                raise ValueError("Invalid action session; reconnect")
            if payload.get("action")=="session":
                if payload.get("control_profile") != "piper-l-cpv-v8.1":
                    raise ValueError("v8 control_profile required; legacy client cannot control this bridge")
                port=payload.get("udp_port")
                if not isinstance(port,int) or not 1<=port<=65535: raise ValueError("Invalid UDP port")
                request=payload.get("request_id")
                if not isinstance(request,str) or not 1<=len(request)<=80: raise ValueError("Invalid session request id")
                identity=(peer,port,request)
                if identity!=self._session_request:
                    if self._session is not None:
                        if self._last_received_at is not None and self._time()-self._last_received_at<.5:
                            raise ValueError("Current controller still active; stop it before takeover")
                        self._latch_fault("New controller/path session; explicit recovery required")
                    self._session=SessionGuard(secrets.randbelow(0xffffffff)+1,(peer,port))
                    self._session_request=identity
                    self._last_client=None;self._last_seq=None;self._sequence_client=None
                    self._latest_control=None
                return {"ok":True,"session":self._session.sid,"server_ms":int(self._time()*1000)&0xffffffff,"protocol":6,"control_profile":"piper-l-cpv-v8.1"}
            if self._last_client != (peer,payload.get("udp_port")):
                raise ValueError("Action must come from the current UDP controller")
            action=payload.get("action")
            request_id=payload.get("request_id")
            if not isinstance(request_id,str) or not 1<=len(request_id)<=80:
                raise ValueError("Missing request id")
            if request_id in self._action_results:
                previous_action,result=self._action_results[request_id]
                if previous_action!=action: raise ValueError("Request id reused for a different action")
                return result
            frame=self._latest_control
            if frame is None or frame.estop or frame.mode!=0 or any(frame.arm_normalized+frame.drive_normalized) or frame.laser_enable:
                raise ValueError("Release controls and send neutral standby before action")
            if self._last_received_at is None or self._time()-self._last_received_at>.25:
                raise ValueError("Control heartbeat stale")
            if action not in ("enable","recover","set_home","home","stop"):
                raise ValueError("Unknown arm action")
            self._action_busy=True
            epoch=self._urgent_epoch
            try:
                self._home_goal=None
                if action=="recover":
                    self.arm.recover()
                    if epoch!=self._urgent_epoch: raise RuntimeError("Recovery interrupted by newer stop")
                    self._fault_latched=False
                    self._fault_reason=self._arm_error=""
                    self._fault_source=""
                    self._input_rearm_required=False
                elif self._fault_latched:
                    raise RuntimeError("Fault latched; use recovery first")
                elif action=="enable": self.arm.enable()
                elif action=="set_home": self.arm.set_home()
                elif action=="home":
                    self._home_goal=self.arm.home()
                    self._home_tick_at=self._time()
                else: self.arm.hold_position()
                if epoch!=self._urgent_epoch: raise RuntimeError("Action interrupted by newer stop")
                self._arm_command_state="returning_home" if self._home_goal is not None else "idle"
                result={"ok":True,"action":action}
            except Exception as exc:
                if action=="recover":
                    self._latch_fault("Action failed: "+str(exc))
                else:
                    self._handle_arm_rejection(exc)
                result={"ok":False,"error":str(exc)}
            finally:
                self._action_busy=False
            # SDK enable/recovery may wait for feedback. Resume timeout tracking now,
            # while still requiring a continuing control heartbeat afterwards.
            self._last_received_at=self._time()
            self._action_results[request_id]=(action,result)
            if len(self._action_results)>128:
                self._action_results.pop(next(iter(self._action_results)))
            self._record("action",action=action,result=result)
            return result

    def _record(self, event, **values):
        values.pop("action_token",None)
        record={"time":self._time(),"event":event,**values}
        encoded=json.dumps(record,ensure_ascii=False,allow_nan=False)
        with self._recent_lock: self._recent_records.append(json.loads(encoded))
        if self._audit:
            self._audit.info(encoded)

    def _latch_fault(self, reason, source="bridge"):
        if self._fault_latched:
            return
        self._home_goal = None
        self._fault_latched = True
        self._fault_reason = self._arm_error = reason
        self._fault_source = source
        self._arm_command_state = "fault"
        self.laser.force_off()
        self._record("fault", reason=reason)
        try:
            self.arm.fault_stop(reason)
        except Exception as exc:
            self._arm_error += "; stop send failed: " + str(exc)
            self._log.exception("Fault stop failed")

    def _check_timeout(self, now):
        if self._last_received_at is not None and now - self._last_received_at >= self._command_timeout_s:
            self._latch_fault(f"Control timeout >={self._command_timeout_s*1000:g} ms; local recovery required", "watchdog")

    def safety_tick(self):
        with self._control_lock:
            self._check_timeout(self._time())
            try:
                self.arm.check_feedback()
                now=self._time()
                if self._home_goal is not None and now-self._home_tick_at>=.05:
                    dt=min(.05,now-self._home_tick_at)
                    self._home_tick_at=now
                    if self.arm.advance_home(self._home_goal,dt):
                        self._home_goal=None
                        self._arm_command_state="home_complete"
            except Exception as exc:
                if self._home_goal is not None or getattr(self.arm,"motion_may_be_active",False) or getattr(self.arm,"_estop_latched",False):
                    self._latch_fault("Feedback safety stop: " + str(exc), "arm_feedback")
                else:
                    self._handle_arm_rejection(exc)

    def _safety_loop(self):
        count = 0
        while not self._stop.wait(0.01):
            try:
                self.safety_tick()
                count += 1
                if count % 5 == 0:
                    self._record("sample", **self.get_diagnostics())
            except Exception as exc:
                with self._control_lock:
                    self._handle_arm_rejection(exc)

    def _sample_telemetry_loop(self):
        while not self._stop.is_set():
            try:
                base=self.build_telemetry()
                report=self.get_diagnostics()["arm"]
                sensors=getattr(self.arm,"get_sensor_snapshot",lambda:{})()
                self._cached_telemetry=(self._time(),base,report,sensors)
            except Exception: self._log.exception("Sensor sample unavailable")
            self._stop.wait(1.0/self._telemetry_hz)

    def _telemetry_loop(self):
        while not self._stop.wait(1.0/self._telemetry_hz):
            client=self._last_client
            if client is None: continue
            try:
                with self._lock:
                    seq=self._telemetry_seq;self._telemetry_seq=(seq+1)&0xffff
                cached=self._cached_telemetry
                sample_age=self._time()-cached[0] if cached else float("inf")
                if sample_age<.25:
                    _,base,report,sensors=cached
                    base=replace(base,seq=seq)
                    report=dict(report)
                    if report.get("feedback_age_ms") is not None: report["feedback_age_ms"]+=sample_age*1000
                else:
                    base=TelemetryFrame(seq,0,0xffff,0.0,0.0,(0,0,0,0,0,0),0,0)
                    report={};sensors={}
                if self._legacy_test_transport: packet=base.pack()
                elif self._session: packet=pack_telemetry(self._session.sid,base,int(self._time()*1000)&0xffffffff,report,sensors)
                else: continue
                self._socket.sendto(packet,client)
            except OSError:
                if not self._stop.is_set(): self._log.exception("Telemetry send failed")
            except Exception: self._log.exception("Telemetry packing failed")

    def close(self) -> None:
        if self._stop.is_set():
            return
        with self._control_lock:
            if self._last_received_at is not None:
                try:
                    self.arm.hold_position()
                except Exception as exc:
                    self._latch_fault("Shutdown hold failed: " + str(exc))
        self._stop.set()
        self.laser.force_off()
        if self._video:
            self._video.close()
        self._socket.close()
        for thread in self._threads:
            if thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=1.0)
        for device in (self.laser, self.camera, self.arm):
            try:
                device.close()
            except Exception:
                self._log.exception("device cleanup failed")
        if self._audit_listener:
            # After workers stop, drain queued records outside control paths.
            self._audit_queue.join()
            self._audit_listener.stop()
            self._audit_file.close()
        if self._audit:
            for handler in self._audit.handlers[:]:
                handler.close()
                self._audit.removeHandler(handler)
