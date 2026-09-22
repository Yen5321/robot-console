# 架构与接口

## 数据与控制路径

```text
WPF RealRobotLink (20 Hz，原 32 字节控制帧)
  → BridgeService (会话/CRC16/序号/映射时间/250 ms 看门狗)
  → RosServoArm (localhost:9088，原始接收单调时间)
  → Motion (100 Hz，输入年龄、姿态参考、工作空间、状态机)
  → TwistStamped /servo_node/delta_twist_cmds
  → Humble MoveIt Servo (逆雅可比、自碰撞、奇异区、关节限位、平滑)
  → JointTrajectory /console/servo_output
  → console_servo_stamp (保留 DDS source_timestamp)
  → JointTrajectory /console/servo_output_stamped
  → JointGuard → 模拟关节执行 → /joint_states
```

当前 MoveIt 使用其自带逆雅可比路径，没有另写 DLS 求解器，也没有用自定义 IK 代替 Servo。
当前 `sim` 执行器只积分关节速度，末端反馈由固定 URDF 做 FK；不是按输入伪造末端直线位移。
真实执行器未接通。`readonly` 不启动 Servo，只发布接收到的关节状态和独立诊断。

## 接口约定

| 接口 | 类型/单位 | 有效性与所有者 |
|---|---|---|
| Windows→桥接 | 既有五轴 `[X,Y,Z,pitch,yaw]` 归一化；v8 速度能力握手 | 收包线程验证 CRC/会话/序号/时间；仅一个控制端 |
| IPC `/input` | JSON `input` 长度5，`received_at` 同机 monotonic 秒 | 原始控制包年龄<250ms；重复/倒序拒绝；心跳不能延长旧非零输入 |
| IPC `/heartbeat` | JSON `received_at` | 仅有效包更新租期，支持回零；不意味着仍在点动 |
| IPC `/action` | enable/recover/set_home/home/stop | 只对 sim 开放；readonly 均拒绝 |
| IPC `/stop` | reason | 首因锁存；sim 停止，不是实机 SDK 急停 |
| TwistStamped | 基座 `base_link`，m/s、rad/s | 每周期新生成；上游过期先停止；Servo 入口超时100ms |
| Servo JointTrajectory | joint1..joint6，rad、rad/s | 校验顺序、有限数、源年龄<100ms、点时间≤250ms、位置界限 |
| `/joint_states` | 模拟反馈或物理反馈，rad/rad/s | 只读必须三组关节 CAN 都新鲜；重复读取不刷新物理年龄 |
| `/console/diagnostics` | JSON String，10Hz | 无令牌；状态、周期统计、来源计数 |
| HTTP 8080 | `/diagnostics` `/configuration` `/logs/recent` `/arm/action` | 保留客户端接口；令牌仅用于动作会话，导出前脱敏 |

Humble Servo 的轨迹头时间为0，意为立即执行。C++ 中继使用 DDS 原始发布时间填入输出头，
不是用接收时间给旧消息续期。Python Humble 订阅回调不直接提供 MessageInfo，故中继独立为小包。
重复/乱序输出丢弃且不刷新执行租期；持续缺失会触发执行看门狗。

## 坐标、速度与保持

- SDK 原始 XYZ 不改名。WPF 负责摇杆Y/Z和按钮X映射；ROS 内部只接收基座坐标速度。
- 平移保持按下时的姿态参考。姿态按钮按 XYZ 欧拉角速率更新参考，转换为 SO(3) 旋转误差，不能把欧拉角速率当作空间角速度。
- 姿态目标领先限制1°用于防积分积累，不是旧版“误差1°就急停”；纯平移修正速度≤1°/s。
- 平移2–50mm/s、姿态1–10°/s是目标上限；Servo/限位/碰撞可进一步减速，诊断必须说明。
- 末端速度整形后仍限制最终关节速度和加速度；关节制动距离与工作空间预测保留。
- 松手先清输入并减速，sim 到零后保持模拟关节位置；真实重力、制动、负载未模拟。
- 回零独立4mm/s、2°/s，记录本模型的位姿；参考领先>2mm/1°故障锁存；不承诺自动绕障。

## 状态与故障

| 状态 | 进入 | 退出 |
|---|---|---|
| not_ready | 未使能/静态无反馈/只读 | sim 新鲜反馈+显式使能 |
| ready / holding | 新鲜且使能、零输入 | 重新按下或回零 |
| jogging | 有效非零输入/回零 | 松手、受限或故障 |
| decelerating | 松手、失焦、切模式、调速 | 关节与末端速度归零 |
| fault | 有效控制租期/反馈/执行异常 | 显式恢复→未使能，再使能并释放重新按下 |
| estop | 手动停止或桥接优先停止 | 显式恢复，通信恢复不解除 |

普通 Servo 限制2/4/5阻止运动并要求重新按下，不发送 SDK 急停；第一故障原因不被重复诊断覆盖。
框架可在状态机处扩展未来硬件接口，但不能假设受控减速在反馈失效时仍可靠。

## 进程与边界

N100：桥接、D435 子进程、ROS运动节点、Servo、时间中继、robot_state_publisher。
全部 DDS 为 localhost；Windows 只需要 UDP/HTTP/WebRTC。RViz 和 Gazebo 不默认运行。
模型的自碰撞不涵盖塔架、除锈头、线缆和人员；本版也不构成高处运行验收。
