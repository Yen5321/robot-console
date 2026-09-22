> **v8 开发提示：** 以下为原模块参考文档，部分直接 SDK 启动说明已不适用。请以工程根目录 README.md、docs/STATUS.md 和 docs/DEPLOYMENT.md 为准。v8 只开放 ROS 模拟与 CAN 只读模式。

# 地面调试 v8 / 8.0-ros-servo

配套 Windows `RobotConsole-Ground-v8`。使用官方 `piper_sdk.C_PiperInterface_V2`、SDK 原生末端基座 XYZ/RPY 和 MOVE L。不加入工具头偏移，不修改 SDK 轴定义。

## 操作

- 左右摇杆 → Y，上下 → Z，伸出/收回 → X，俯仰 → RY，偏航 → RZ。
- 平移目标速度 2–50 mm/s，启动默认 2；姿态目标速度 1–10°/s，启动默认 1。每次启动恢复低速，不保存运动输入。目标速度不等于已实测速度。
- 合成平移速度不超过所选值。客户端在缩放前应用死区，桥接按协商上限积分。SDK 的 `move_speed_percent` 另有含义，不是 mm/s。
- 按住持续点动；松开、触摸释放、捕获丢失、失焦、模式切换、调速均清零并要求重新按下。平移不主动改变 RPY 目标。
- 回零返回手动“设为零位”的位姿，不是关节机械零点；独立限制 4 mm/s、2°/s，保留跟随和工作空间检查。
- 方向符号在 Windows `jog-directions.json`；交付默认未校准。已确认过的方向文件可复制到新目录；没有实测确认不要将 `Calibrated` 改成 true。

## 升级

关闭旧控制端，在机器人原 SSH 终端 Ctrl+C 停止桥接。Windows 运行 `Bridge-Ground-v8/deploy-bridge.ps1`，按提示输入 SSH 密码。

安装器不会启动、使能或移动机器人；检测到仍运行时报告 PID 并退出。若由 systemd 管理，先停止对应服务以免自动拉起。

安装器校验 ZIP 哈希、Python 语法和新配置后，备份到机器人项目 `backup-v8-时间/`。原 YAML 原样备份；工作 YAML 会规范化缩进/注释。只迁移以下七项，逐项打印旧值与新值：

| 字段 | 新值 |
|---|---|
| arm.max_linear_speed_mm_s | 50.0 |
| arm.max_angular_speed_deg_s | 10.0 |
| arm.enable_on_start | false |
| arm.safety_checks.jog_translation_enabled | false |
| arm.safety_checks.jog_rotation_enabled | false |
| arm.safety_checks.hold_translation_enabled | false |
| arm.safety_checks.hold_rotation_enabled | false |

CAN、网络、相机、工作空间、其余阈值、传感器占位值、home_path 保留；零位文件和日志不覆盖。写入失败回滚已写文件，备份保留。

机器人 SSH 中离线校验（不连接 CAN）：

```bash
cd /home/tieta/robot-bridge-track-b
.venv/bin/python -m robot_bridge --config config.yaml --check-config
```

应显示 `bridge_build: 8.0-ros-servo`。之后手动启动：

```bash
.venv/bin/python -m robot_bridge --config config.yaml
```

启动不自动使能/恢复。v8 客户端不能控制旧桥接，旧客户端不能取得 v8 会话。既有 32 字节帧保留，速度解释须通过 `ros-servo-v8` 握手。

## 状态与停止

静止时未使能、示教/非 CAN 模式、反馈超时、控制器异常：显示原因并禁用点动，不额外发送 SDK 急停。反馈有效要求 SDK 时间戳持续更新。

运动或保持监测期内丢使能、改变模式、反馈失效、控制器异常、SDK 发送异常：停止增加目标并锁存故障，走现有 `EmergencyStop(1)`。普通越界输入拒绝时，若正在点动则尝试新鲜反馈保持；保持失败按故障停止。

重复诊断不反复触发普通故障停止；手动急停始终走优先路径。控制包超时（默认 250 ms）、UI 心跳保持独立。通信恢复不自动恢复运动，必须显式恢复并重新按下。这是进程内软件看门狗，不是已接入的独立 MCU 硬件刹车。

四个可选跟随/松手超差开关默认 false。松手仍从新鲜反馈提交当前位置保持目标并记录 `last_hold` 最大位移/旋转；开启对应 `hold_*_enabled` 后才因超差停止。它们不关闭反馈失效、控制器异常、回零检查、工作空间、断链或手动急停。没有关闭全部保护的总开关。

SDK 急停未验证能原地抱住。此前姿态偏离、急停后下移问题未宣称解决。

## 诊断和配置

- 主区仅当前异常、关节使能、控制模式、实际基座 XYZ/RPY；过期位姿显示不可用。
- “详细诊断”：版本、目标、反馈年龄、阻止码、故障来源/锁存、运动状态、有效上限、最近 300 条日志，可导出。
- 完整日志：机器人 `logs/jog-v5.jsonl` 及轮转文件，历史名称保留。
- `GET /diagnostics`：结构化实时状态。`blocking_code` 包括 `not_enabled`、`teach_mode`、`control_mode`、`feedback_stale`、`controller_fault`、`input_bounds`、`rearm_required`、`fault_latched`。
- `GET /configuration`：只读有效配置、检查参数说明；`GET /logs/recent`：最近事件。
- `python -m robot_bridge --parameter-schema`：离线查看检查参数说明。
- `config.annotated.yaml`：完整中文参考，不能直接覆盖现场 YAML。编辑后先 `--check-config`，停机重启生效。
- 激光为 noop，占位参数可查看，界面禁止启光。去掉人员/挡板两行但不伪造后台传感器满足；锈斑列表保留。

## 地面验收（尚未完成）

从 2 mm/s、1°/s 开始逐轴确认方向和松手效果，再按平移 4、10、20、50 mm/s 与姿态 2、5、10°/s 逐级测试。每档比较目标与反馈、平移 RPY 目标是否不变、停止时间和额外位移。所有档位目前均未完成实机验收。

```bash
.venv/bin/python tools/analyze_jog_log.py logs/jog-v5.jsonl --output ground-stop-report.json
```

约 20 Hz SDK 反馈只能估算稳定样本确认时间，不是精确物理刹停时间。旋转误差使用空间旋转角。应结合独立测量/录像确认松手、断链后额外位移 ≤2 mm、姿态变化 ≤1°；不达标或数据不足不判通过。配套提供 `地面验证记录.csv`。

本轮仅机械臂/相机地面调试，不构成 50 米塔上验收。无线距离、独立硬件制动/看门狗和底盘另行验证。

## 源码软件验证

```bash
python -m unittest discover -s tests -t .
python tools/mock_bridge.py --udp-port 19700 --http-port 18700
```

模拟服务仅绑定本机，不连接 CAN。Windows 源码包含 SmokeTests、UiTests、LinkIntegrationTests；联调测试参数为 `19700 18700`。软件测试不替代实机验收。
