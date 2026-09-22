# AI 开发入口

先读 README、docs/STATUS.md、docs/ARCHITECTURE.md。**此目录是唯一开发源码根目录。**

## 修改位置

- 界面/输入释放：`windows-client/RobotConsole`。保留已要求的功能，不按截图猜测并删除其他功能。
- UDP/会话/诊断：`bridge/robot_bridge`。ROS 后端在 `hardware/ros_servo.py`；旧 `piper6.py` 仅供回归参考，v8 主入口拒绝直接启动它。
- 控制数学/状态机：`ros2_ws/src/robot_console_servo/robot_console_servo/core.py`。
- ROS/模拟：同包 `node.py`；只读 CAN：`passive_can.py`；参数与范围：`settings.py`。
- 不修改 `releases/`、`bin/`、`obj/`、ROS `build/install` 里的生成文件。用工具重新生成。

## 必须保持的约束

- 没有硬件验证证据时，不添加可通过 true/false 开启的实机运动开关。
- 不自动使能、解除急停、更新固件或恢复运动；不把 MOVE L 偷换成 CPV 的回退路径。
- 只允许一个实机 CAN 指令发送者；未来接通 CPV 前核验旧桥接/官方驱动已退出。
- 不用反复更新时间戳维持旧输入；输入时间、Servo 来源时间、物理反馈时间分开。
- 基座 Y/Z 为摇杆，X 为伸缩；ROS 内 m/rad，UI mm/deg。不能交换 SDK 轴定义。
- 保持断链、反馈过期、控制器异常处理；普通拒绝与故障停止明确区分。
- 不把 SDK 调用成功、发送成功或模拟位移写成真实运动成功。
- 不宣称无传感器时已有环境避碰；碰撞矩阵有上游来源，修改需说明被排除的链接对。
- 网络/诊断/录包不能在控制循环执行阻塞写盘；故障保留首个原因。

## 运行检查

```text
python tools/test-offline.py
dotnet run --project windows-client/RobotConsole.SmokeTests -c Release
dotnet run --project windows-client/RobotConsole.UiTests -c Release -- tests/ui-v8.png
bash tools/build-ros.sh
# start-ros.sh sim 后：python3 tests/ros_smoke.py
# sim + bridge 启动后 Windows：
dotnet run --project windows-client/RobotConsole.RosIntegrationTests -c Release -- <Ubuntu-IP>
```

只读解码测试需 `config/requirements-readonly.lock.txt`；Windows 缺 SDK 时会明确跳过，必须在 Linux 再跑。
结果写入 docs/STATUS.md 与 docs/TEST_RESULTS.md，附实际命令、软件版本、模拟/实机类别和未测项。
遇到 SDK/ROS API 差异先查锁定版本，不能通过放宽看门狗或删除校验让测试通过。

## 给下一位 AI 的任务

1. 收集型号铭牌/固件与 `preflight.py` 输出。
2. 用只读反馈核验 URDF 关节方向和 SDK 末端坐标，不默认两者一致。
3. 检查 `cpv_sdk.py` / `cpv_policy.py` / `cpv_node.py` 的独立执行链路；v8.1 已实现独立 CPV 执行器，真实验收仍待现场。
4. 完成停止、承重保持、反馈失效与断电行为的地面验收后，才发布实机运动版本。

## v8.1 修改入口

先读 `docs/CPV-ARCHITECTURE.md` 和 `docs/V8.1-DEPLOYMENT.md`。
- CPV SDK边界：`cpv_sdk.py`；状态/期限/限速/测量证据：`cpv_policy.py`；唯一CAN发送进程：`cpv_node.py`。
- `tests/test_cpv.py`、`test_cpv_sdk.py`：假反馈与官方SDK虚拟CAN；`tests/cpv_executor_smoke.py`：真实执行进程+假SDK，不触碰硬件。
- `tools/commission-cpv.py` 是操作者逐次触发的有界实机测试入口；不得加入循环自动点动或跳过测量证据的开关。
- v8.1 真实关节上限0.05rad/s、加减速0.1rad/s²；Windows真实2mm/s、1°/s。更改需重新验收。
- 依赖锁：`config/dependencies.v8.1.lock.json`；模型来源和生成方法：`tools/vendor-piper-l.py`。
- 未知电机速度换算不填成“实际关节速度”；SDK reset 不属于软件恢复。
