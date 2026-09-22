# 铁塔除锈车 Windows 平板遥控端

WPF / .NET 8 / MVVM 平板端。`MockRobotLink` 用于演示，`RealRobotLink`
按 `PROTOCOL.md` 连接机器人侧 Track B 服务；两端不共享实现代码。

## 运行

```powershell
dotnet run --project .\RobotConsole.csproj
```

按 `Ctrl+Shift+D` 打开隐藏调试面板，可模拟视频丢失、控制断连、重连及任一安全联锁失败。

## 切换到真实 Piper6 / D435

编辑输出目录中的 `robotlink.json`：

```json
{
  "Mode": "Real",
  "RobotHost": "192.168.1.50",
  "UdpPort": 9000,
  "MjpegUrl": "http://192.168.1.50:8080/stream.mjpg",
  "CommandIntervalMs": 50,
  "ControlTimeoutMs": 1000,
  "VideoTimeoutMs": 2500
}
```

默认仍为 `Mock`，避免仅启动 UI 就意外驱动实机。实机测试必须由现场人员清空工作区、
握住物理急停并确认 Piper 坐标系后，再把 `Mode` 改为 `Real`。

## 架构边界

- `Views`：纯 UI 与触摸输入适配，不包含网络、Socket 或串口逻辑。
- `ViewModels`：操作状态机、安全联锁、命令启用条件；只依赖 `IRobotLink`。
- `Models`：协议数据类型。
- `Services`：`IRobotLink` 抽象和 Mock 实现。未来的 `RealRobotLink` 应只在此层实现。

协议 v1 能表达五轴连续点动和激光参数，但没有精确高度/角度步进、机械臂回零、拍照、录像、
急停复位字段。当前 UI 的离散 Z/姿态按钮在真实模式下会明确禁用，首轮实机联调使用 XY 摇杆；
急停后必须现场复位并重启桥接服务，不能从平板远程解除。
