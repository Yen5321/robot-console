# 给现场与下一位 AI 的排查顺序

先确认界面运行方式和 `/diagnostics` 的 `bridge_build=8.0-ros-servo`、
`control_profile=ros-servo-v8`、`arm.backend_mode`。不能通过文件夹名推断实际运行版本。

## 一次性采集

```bash
cd ~/robot-console-v8
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_LOCALHOST_ONLY=1
python3 tools/preflight.py --can-name can0
python3 tools/collect-diagnostics.py --seconds 10
```

把生成的诊断 ZIP 和当前源码 ZIP 一起交给 AI，并说明“操作前状态→按下方向→松手→异常时刻”。
导出不包含 action_token，地址会脱敏；rosbag仅记录限定运动话题，不采相机视频。

## 分层判断

| 观察 | 检查位置 | 不应采取的捷径 |
|---|---|---|
| 版本不匹配 | Windows RealRobotLink 握手和服务版本 | 取消版本验证 |
| received_input 不增加 | UDP ACK、会话、桥接 note_control、IPC | 把旧输入重新加时间戳 |
| control_input_expired | 原始输入年龄、UI心跳、UDP回执、桥接日志 | 关闭看门狗 |
| servo_generated 不增加 | Servo启动服务、Twist、状态、DDS来源时间 | 把“已发送Twist”写成“已运动” |
| Servo状态1/2 | 接近/进入奇异区，查看模型关节姿态 | 提高阈值直到不报错 |
| Servo状态3/4 | 自碰撞距离、排除表、碰撞模型 | 全局关闭碰撞检查 |
| Servo状态5 | 关节限位、方向、模型偏置 | 放宽软件关节限位 |
| servo_output_timeout | Servo/中继进程、DDS、控制周期 | 保持上一条非零速度 |
| feedback_expired | 三组关节CAN年龄，控制器/使能/末端各组状态 | 曾收到一次就一直算有效 |
| readonly 显示有反馈但按钮不能动 | 这是预期的只读模式 | 修改 ready=true |
| 只有模型末端与SDK末端不一致 | 坐标、偏置、单位、真实型号 | 直接把模型反馈标成SDK反馈 |

## 文件定位

- 输入释放问题：WPF `MainViewModel` 和控件捕获事件；先跑 UiTests。
- 会话/超时问题：`bridge/robot_bridge/service.py`，完整链路跑 RosIntegrationTests。
- 运动参数/参考变化：ROS包的 `core.py`、`settings.py`、`config/motion.yaml`。
- 源时间/输出拒绝：`console_servo_stamp`、ROS `node.py` 的 trajectory。
- 真实CAN反馈：`passive_can.py`，使用真实SDK解码器的离线测试。

## 注意保留首次原因

bridge和ROS均有自身看门狗。一次失联可能依次出现多个日志；先按单调时间排序找第一次异常。
原有 HTTP `GET /diagnostics 200` 只说明诊断可访问，不能证明 UDP、UI心跳或运动控制正常。
`sdk_calls=0` 是本预览版预期结果；模拟动作不调用CAN发送。
# 一次性环境检查（先做）

N100 上运行 `python3 tools/doctor-v8.py`，不需要 sudo，不启动 ROS 节点、不打开 CAN、不安装软件。
它检查 ROS 包/候选版本、APT 完整性、启动与 Python 导入、所有 MoveIt 动态库、项目配置/模型、CAN 状态及端口。
结果在 `diagnostics/environment-audit-latest.zip`，详细报告目录含 SUMMARY.txt。
APT 预演成功且候选完整时生成 `repair-ros.sh`，固定本次候选版本但**不会自动执行**；先审核 `apt-repair-preview.txt` 再修复。
只读静态检查不能替代模拟运行、实时性能和物理机械臂验收。缺失库不要用旧版库软链接冒充新版 ABI。
