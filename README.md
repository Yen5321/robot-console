# RobotConsole v8.1 · PIPER-L CPV

唯一开发源码根目录；不要修改 releases 中的生成文件。

Windows WPF → UDP 桥接 → ROS 2 Humble / MoveIt Servo → 独立 CPV 执行器 → 锁定 pyAgxArm → CAN。
型号 PIPER-L，用户提供完整固件 S-V1.9-0260716，运行时重新查询核对。原装夹爪只建模，不发开合命令。

## 从这里开始

1. **[逐步部署与实机验证](docs/V8.1-DEPLOYMENT.md)**：新目录安装、依赖检查、模拟、只读、受限逐轴验证、Windows连接。
2. [当前完成/待验证状态](docs/STATUS.md)；[软件测试记录](docs/TEST_RESULTS-V8.1.md)。
3. [接口与状态机](docs/CPV-ARCHITECTURE.md)，AI 修改前阅读 [AGENTS.md](AGENTS.md)。

## 入口

- `bash tools/start-ros.sh sim`：真实 Servo 算法 + 模拟执行，无CAN。
- `bash tools/start-ros.sh readonly`：纯接收，不发CAN查询/使能。
- `bash tools/start-ros.sh real`：启动独立实机执行器，**仍不自动使能**；先完成现场验证。
- `python3 tools/commission-cpv.py status`：只读状态。显式 enable/hold-test/pulse 才可能发实机控制。
- `python3 tools/test-offline.py`：软件回归；`bash tools/build-ros.sh`：ROS构建。
- `powershell -File tools/build-windows.ps1`：Windows完整运行目录；`python tools/package.py`：发布ZIP。

## 本轮边界

已经实现实机CPV发送代码、准入检查和现场测试工具。2026-09-17 已远程部署，三个 FK 样本通过；首次实机使能后出现零速下反馈异常变化及电子急停，**实机运动验证失败，当前保持急停**。
执行器 8.1.1 修复接收短帧、首次漂移监测与参数准入；零速运动根因仍待确认，见 [事件记录](docs/CPV-INCIDENT-20260917.md)。
实机先验证FK、五秒零速保持和十二次方向测试，才可开放2mm/s、1°/s摇杆。更高速度未验证。
保持/掉落、CAN断开与进程退出行为、N100十分钟负载及原先抖动问题仍待现场验收；软件看门狗不是硬件失联保护。
