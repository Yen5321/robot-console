# v8.1 接口、状态和排错入口

## 数据流与所有者

| 接口 | 单位/频率 | 有效期与所有者 |
|---|---|---|
| WPF→UDP桥接 | 原五轴32字节帧；20Hz | CRC/序号/会话/客户端时间校验；原接收时间不得被转发改写 |
| 桥接→网关HTTP9088 | 五轴归一化；X/Y/Z/俯仰/偏航 | 原UDP接收时间，250ms；仅本机 |
| 网关→TwistStamped | 基座m/s、rad/s；100Hz | YZ摇杆/X伸缩；RPY速率经SO(3)转换，平移保持姿态参考 |
| Servo→stamped JointTrajectory | 六轴rad/rad/s；100Hz | 中继保留DDS源时间，非接收时间；100ms |
| 网关→cpv_authority | 原控制时间、原非零输入时间、会话epoch、故障 | 刷新转发不能延长旧输入；重启epoch改变锁存 |
| 执行器→官方SDK | move_cpv_vel(1..6, rad/s) | 唯一CAN发送者；六轴同一比例限速、加减速 |
| CAN→执行器反馈 | 关节位置、SDK法兰、使能/状态、电机速度 | SocketCAN内核接收时间；每个物理报文组分别250ms |
| 执行器→cpv_feedback | 100Hz本机ROS | 发布原时间；诊断读取不能刷新年龄 |

ROS_LOCALHOST_ONLY=1。网络桥接不导入pyAgxArm；独立执行器持有工程CAN写锁。
该锁不能排斥无关厂商程序，部署前还需停掉旧驱动。相机沿用独立进程，不在CAN循环采集。

## SDK边界

`cpv_sdk.py` 使用锁定提交 e7aef17d54cac80cbaeb1b4110ab3d8f1337a95b，`piper_l / PiperFW.V189`。
启动 `connect(start_read_thread=False)`，关闭自动模式切换；只读身份/参数查询，不写Flash、不调增益。
SDK默认CAN传输会吞掉部分错误，适配器只替换传输send为有界python-can发送，编码和J2～J5速度符号仍由SDK负责。
单帧send超时1ms，失败传播到执行器。SDK解析器仍处理接收数据，但年龄按物理CAN分组记录。
不用SDK聚合时间戳冒充六轴都更新，不开启SDK后台读线程抢走发送错误。
零速保持连续发送六轴零值，绝不调用MOVE L/MIT/CPV位置作为后备。

## 状态

| 状态 | 进入 | 退出/操作者动作 |
|---|---|---|
| not_ready | 启动/软件恢复；无运动发送 | 当前型号/完整固件、FK样本和新鲜反馈通过后显式使能 |
| enabling | 显式操作，最多3秒 | 等六轴使能、CAN=1、CPV=5；超时故障 |
| zero_hold | 已使能、输出零速 | 五秒保持和逐轴方向通过后才允许摇杆 |
| jogging | 有效输入+新鲜Servo+已完成准入 | 释放/Servo限制→减速；故障按严重程度停止 |
| decelerating | 松手、清输入或普通超时 | 0.1rad/s²约束降到零；进入零速保持 |
| fault_decelerating / fault_zero_hold | 输入250ms/Servo100ms超时等 | 原因锁存，仍发零速；显式恢复、重新使能、归零重按 |
| electronic_stop | 反馈失效/状态改变/SDK异常/严重故障/手动 | 一次独立SDK电子停止；不承诺承重保持；现场检查 |

首次故障保留。手动急停可把普通故障升级为电子停止，但不覆盖首因。
软件恢复要求新鲜正常状态和500ms静止反馈；电子停止后还须观察到真实急停再恢复的状态转换。
不调用reset。拒绝会提示需要现场支撑/控制器检查，而不是无限重复使能或急停。

## 模型审计

官方模型提交 f6642ce0d7872c686f29c99e9e10cd23d1d49313，`piper_l`，控制链base_link→link6。
`Model` 沿父子链解析固定变换/原点/轴向；不按XML文件顺序猜测链，不额外取反或重复加零偏。
夹爪底座/法兰保留官方网格；两指用STL顶点在整个prismatic范围的扫掠包围盒，再外扩2mm。
六轴运动组不含夹爪。夹爪位置未知时保守包络永远启用。

没有复制通用Piper的ACM。排除直接相邻部件、刚性夹爪组件内接触，以及link5对固定在link6上的法兰/夹爪壳安装邻接。
FCL在模拟参考姿态实测link5↔法兰1.0mm、link5↔夹爪壳5.5mm，属于安装邻近；手指对机械臂仍检查。
其余最近对base_link↔link2约6.13mm。低速自碰撞接近阈值为5mm，实际碰撞仍停止；这些设置仍需现场模型核验。
`model_audit URDF SRDF q1...q6` 打印FCL近距离链接对，避免看到“很慢”就盲目关闭检测。

## 准入证据

- 当前会话三个不同姿态的FK/SDK比对：≤5mm/2°。
- 五秒零速度漂移：≤2mm/1°。
- 十二次正反短脉冲：每次≤0.01rad/s，含减速≤0.5秒。主轴实际变化方向与指令一致、其他轴漂移受限。
- 真实回零独立验证：新零位绑定模型，首次从≤10mm/3°处测试，终点≤0.5mm/0.2°。

结果是测量日志而不是可编辑“关闭保护”开关。会话重启不恢复授权。记录不代表全工作空间、额定负载或高处验收。

## 故障定位顺序

1. `/diagnostics` 查版本/profile/backend_mode，确认不是旧EXE/旧桥接。
2. 查看 executor 的首个fault、phase；不要仅看HTTP200。
3. `control_age_ms` / 原始input_origin：区分UI包断档和转发。
4. `servo_age_ms` / Servo状态：区分算法限制与Servo节点停止。
5. `feedback_ages_ms`：位置、法兰、六轴使能、状态逐项定位。
6. CAN send异常看 `sdk_or_runtime_error` / `electronic_stop_send_failed`；发送成功计数不等于运动。
7. 实际SDK法兰与目标、零速漂移分开；电机速度不能当作已验证关节速度。

日志由独立线程写入，20MB×6轮转；队列满时记录log_dropped，不阻塞控制。周期p99/max每0.5秒统计。
100Hz是目标，不是硬实时保证。进程死亡/CAN断开之后是否硬件自行刹车必须另做现场试验。
