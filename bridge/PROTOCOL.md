# v8 速度语义补充（8.0-ros-servo）

生产控制先通过 HTTP session 握手，请求/响应含 `control_profile: ros-servo-v8`。
诊断须为 `bridge_version=8.0-ros-servo`，并返回 `motion_capabilities`：linear_speed_mm_s（2–50）、angular_speed_deg_s（1–10）、input_deadband（0–<1）。
不匹配时 Windows 禁止点动；缺少 profile 的旧客户端 session 请求在改变会话前拒绝。

五轴 32 字节 CRC16 控制帧不变：dx/dy/dz 为线速度比例，pitch/yaw 为角速度比例，均 -1000..1000。
客户端先对物理输入应用死区及合成归一化，再分别乘所选平移/姿态速度除以对应协商上限。
机器人按对应有效上限积分。低速不能再乘旧版的 4/2 上限；旧新程序不能混用。
轴顺序仍为 X,Y,Z,RY,RZ。SDK 原生轴定义保持不变。

诊断新增 blocking_code、blocking_reason、fault_source、fault_latched、input_rearm_required；
arm.motion_state、arm.motion_may_be_active、arm.effective_safety_checks。
arm.pose=null 表示反馈不可用；arm.target 只表示已提交目标，不能证明实际移动。
/configuration 只读，/logs/recent 最多 300 条，完整历史位于机器人日志。

以下为已有底层帧及历史说明；速度语义以上述 v8 规则为准。

---

# Robot Bridge Wire Protocol

This file is the only contract shared with the Windows tablet implementation.
All multi-byte fields are little-endian. UDP datagrams must contain exactly one
complete frame; there is no header, checksum, padding, or trailing data.

## Control frame (tablet -> robot)

- Transport: UDP to robot port `9000` (configurable on the robot)
- Recommended rate: one datagram every 50-100 ms
- Python `struct` format: `<HIB3h5hBHHBB`
- Exact size: **30 bytes**

| Offset | Field | Wire type | Meaning |
|---:|---|---|---|
| 0 | `seq` | `uint16` | Sequence number; wraps from 65535 to 0 |
| 2 | `timestamp` | `uint32` | Sender timestamp; currently retained for diagnostics |
| 6 | `mode` | `uint8` | 0 standby, 1 driving, 2 working |
| 7 | `drive_x` | `int16` | Normalized value multiplied by 1000 |
| 9 | `drive_y` | `int16` | Normalized value multiplied by 1000 |
| 11 | `drive_rotate` | `int16` | Normalized value multiplied by 1000 |
| 13 | `arm_dx` | `int16` | Normalized value multiplied by 1000 |
| 15 | `arm_dy` | `int16` | Normalized value multiplied by 1000 |
| 17 | `arm_dz` | `int16` | Normalized value multiplied by 1000 |
| 19 | `arm_pitch` | `int16` | Normalized value multiplied by 1000 |
| 21 | `arm_yaw` | `int16` | Normalized value multiplied by 1000 |
| 23 | `laser_enable` | `uint8` | 0 off, 1 requested on |
| 24 | `laser_power` | `uint16` | 0-100 |
| 26 | `laser_speed` | `uint16` | Physical value multiplied by 10 (3.2 -> 32) |
| 28 | `estop` | `uint8` | 0 normal, 1 emergency stop |
| 29 | `heartbeat` | `uint8` | Toggle 0/1 for each newly generated frame |

Receivers reject values outside the documented normalized ranges. An emergency
stop is acted on immediately even if its sequence number is duplicated or old.
For ordinary commands, a sequence is newer when
`0 < ((new - old) & 0xffff) < 0x8000`; this supports uint16 wrap-around.
The robot establishes a fresh sequence baseline when the UDP source endpoint
changes or after two seconds without an accepted ordinary command.

## Telemetry frame (robot -> tablet)

- Transport: UDP to the source IP **and source port** of the most recently
  accepted, valid control datagram
- Rate: 10-20 Hz (default 15 Hz)
- Python `struct` format: `<HBHff6BBB`
- Exact size: **21 bytes**

| Offset | Field | Wire type | Meaning |
|---:|---|---|---|
| 0 | `seq` | `uint16` | Robot telemetry sequence number |
| 2 | `battery_percent` | `uint8` | 0-100 |
| 3 | `latency_ms` | `uint16` | Age of the latest accepted control packet at send time, capped at 65535 ms |
| 5 | `roll_deg` | `float32` | Vehicle roll in degrees |
| 9 | `pitch_deg` | `float32` | Vehicle pitch in degrees |
| 13 | `joint_margins[0..5]` | six `uint8` | Joint-limit margin, 0-100; 0 is at/outside a limit |
| 19 | `interlock_bits` | `uint8` | Interlocks described below |
| 20 | `laser_active` | `uint8` | 0 inactive, 1 physically active |

`interlock_bits` low five bits:

- bit 0: vehicle stably stopped
- bit 1: arm pose is inside the configured safe domain
- bit 2: end-effector distance is compliant
- bit 3: protective shield is in position
- bit 4: no person is present in the work area

Unknown sensor state must be reported as false (bit cleared), never assumed safe.
If no new accepted control frame arrives for the configured watchdog interval
(default 250 ms), the robot force-disables the laser and clears the derived
"vehicle stably stopped" interlock.

## Video

HTTP GET `http://<robot-ip>:8080/stream.mjpg` returns
`multipart/x-mixed-replace; boundary=frame`. Video transport is intentionally
independent from both UDP frame formats.
