# Robot Console v8 · ROS Servo 地面预览版

Windows WPF → UDP 桥接 → ROS 2 Humble / MoveIt Servo → 模拟关节执行。
**当前交付可运行的模拟链路与只读 CAN 反馈。实机 CPV 运动尚未接通，也没有已验证的停止保持接口。**
界面不能把该版本用于真实机械臂点动。没有一行配置可绕过这个限制。

## 从这里开始

1. 阅读 [当前状态](docs/STATUS.md)，不要把模拟测试当作 N100/机械臂验收。
2. 机器人端 Ubuntu 22.04：按 [部署说明](docs/DEPLOYMENT.md) 安装到独立目录。
3. Windows 解压整个 `RobotConsole-v8-Preview.zip`，运行 `RustRemoval.RobotConsole.exe`。
4. 后续交给 AI：首先让它读本文件、[AGENTS.md](AGENTS.md)、[架构接口](docs/ARCHITECTURE.md)、[故障排查](docs/DEBUGGING.md)。

## 模拟运行（不接 CAN）

在已安装 ROS 2 Humble 的 Ubuntu 22.04 中：

```bash
bash tools/install-deps.sh
bash tools/build-ros.sh
bash tools/start-ros.sh sim
```

另一个终端：

```bash
bash tools/start-bridge.sh sim
```

Windows 连接设置填写这台 Ubuntu 的 IP、UDP 9000、HTTP 8080；视频可保留原配置。
`sim` 没有相机视频。启动后需显式“恢复急停”（若已锁存）、“使能”，再进入作业模式。
界面会标明“ROS 模拟（不驱动实机）”。没有自动恢复运动。

## 只读机械臂

先退出模拟的两个终端，再启动：

```bash
bash tools/start-ros.sh readonly can_name:=can0
bash tools/start-bridge.sh readonly config/bridge.readonly.local.yaml
```

第二条在另一个终端执行。迁移安装器生成 `.local.yaml`；未迁移时使用 `config/bridge.readonly.yaml`。
CAN 接口必须已由现场人员配置为正确波特率并 UP。本版只检查状态，不修改接口或使能电机。
只读端使用官方 SDK 的解码器解析已收到的数据，不调用 `ConnectPort`、模式切换或运动发送函数。
CAN 断开时显示过期；使能/关节/末端反馈各自检查新鲜度。

## 工程目录

| 目录 | 职责 |
|---|---|
| `windows-client` | 界面、触摸/鼠标释放、UDP 编码、连接设置 |
| `bridge` | 会话、CRC/序号/时间、遥测、HTTP 管理与独立相机进程 |
| `ros2_ws/src/robot_console_servo` | 运动状态机、Servo 接入、模拟执行、只读 CAN |
| `ros2_ws/src/console_servo_stamp` | 保留 Servo DDS 原始发布时间 |
| `config` | 桥接示例、依赖锁定；运动配置位于 ROS 包 `config/` |
| `tests` | 硬件无关控制测试、ROS 模拟冒烟测试 |
| `tools` | 环境检查、构建、安装、诊断、打包 |
| `docs` | 当前状态、接口、部署、测试结果与后续验收 |
| `releases` | 自动生成 EXE 目录和 ZIP；禁止在这里改源码 |

## 测试、诊断与停止

```bash
python3 tools/test-offline.py
# 单独运行 ROS 模拟，未连接桥接客户端时：
python3 tests/ros_smoke.py
# 当前 ROS 环境已 source，采集 10 秒诊断包：
python3 tools/collect-diagnostics.py --seconds 10
```

正常退出先松开输入，等待模拟关节停止，再关闭桥接和 ROS 的终端（Ctrl+C）。
回退时先退出 v8，再按原 v7 启动方式启动旧目录；不能让两版争用控制端口。

本次没有在远程 N100 上安装、升级固件、使能或移动机械臂。所有实机速度档位均为未验证。
