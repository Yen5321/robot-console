# 部署、平板连接与回滚

## 交付物

- `RobotConsole-v8-Preview/` / `.zip`：完整 Windows x64 自包含运行目录；不要只复制 EXE。
- `RobotConsole-v8-RobotUpdate.zip`：Ubuntu源码、固定模型、配置、工具与文档，不含Windows二进制。
- `RobotConsole-v8-Source.zip`：完整未编译源码（含WPF和全部测试）。
- `SHA256SUMS.txt` 与各包内部清单：对照版本，避免混用旧程序。

## 第一步：检查 N100（无运动）

系统为 Ubuntu 22.04。先停止旧桥接的终端，避免两个程序争用9000/8080。
保留 `/home/tieta/robot-bridge-track-b` 和其 `.venv`，不要删除原 YAML。
将 RobotUpdate ZIP 和 `install_v8.py` 上传到 N100 的临时目录。

```bash
/home/tieta/robot-bridge-track-b/.venv/bin/python install_v8.py \
  --archive RobotConsole-v8-RobotUpdate.zip \
  --legacy /home/tieta/robot-bridge-track-b \
  --target /home/tieta/robot-console-v8
cd /home/tieta/robot-console-v8
python3 tools/preflight.py --can-name can0
```

安装器不覆盖旧目录；如果新目标已存在会退出，不反复覆盖。需要重装时选择新的版本目录。
安装器需要 PyYAML；上述命令复用旧桥接虚拟环境。Windows发布目录中的 `deploy-v8.ps1` 可执行同样的上传与独立目录安装，不会启动任何服务。
`config/imported-legacy` 保存原 YAML 与零位文件，`migration.json` 列出变更。
CAN 名称、网络、相机和未接入传感器的值都保留；零位不会未经模型对齐就自动启用。

## 第二步：准备 ROS

若尚无 ROS：按官方 [Humble Ubuntu安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)安装。
本工程不代替系统安装器，不升级 Ubuntu、不改路由器或机械臂固件。

```bash
bash tools/install-deps.sh
source /opt/ros/humble/setup.bash
python3 tools/preflight.py --can-name can0
bash tools/build-ros.sh
```

依赖脚本使用锁定的 ROS 包版本。软件源没有对应版本时会失败，不能自动换成“最新”。
把失败输出和依赖清单交给 AI 检查。Python解码依赖装在 `.deps-readonly`，不覆盖旧虚拟环境。
相机依赖可复用原 v7 虚拟环境，见只读启动命令。

### N100 中科大镜像 2026-09-17 版本

现场已确认镜像不再提供测试清单中的 rclcpp/rclpy 构建。新增的现场清单保存用户提供的7个候选版本，原 `dependencies.lock.json` 仍保留开发环境测试证据。
**现场清单尚未通过 N100 编译和模拟验证，不代表已验证兼容。** 显式选择如下：

```bash
bash tools/install-deps.sh config/dependencies.n100-20260917.lock.json --dry-run
bash tools/install-deps.sh config/dependencies.n100-20260917.lock.json
bash tools/build-ros.sh
```

第一条只预演 APT，不安装；第二条实际安装并可能升级 ROS 依赖，会询问是否继续。
脚本禁止删除已有软件包，不执行系统全量升级、不改固件、不启动服务。
安装后先运行本文模拟步骤；只有模拟验证通过才进行只读反馈。将安装/编译结果和 `preflight.py` 输出保存为现场证据。

## 第三步：先试模拟

两个终端分别运行：

```bash
bash tools/start-ros.sh sim
bash tools/start-bridge.sh sim
```

Windows打开v8 EXE，连接设置填写N100的IP（默认仍为10.126.122.113），UDP9000、HTTP8080。
先检查“ROS模拟”标识，再使能模拟机械臂，进入作业模式操作。模拟没有相机画面，视频缺失不等于控制断链。
测试自动运行只对sim开放：`python3 tests/ros_smoke.py`；运行时先退出Windows控制端并停桥接，避免两套测试操作互相干扰。

## 第四步：只读真实反馈

退出模拟两个终端，确保 CAN 已按现场硬件配置并UP。

终端1：

```bash
bash tools/start-ros.sh readonly
```

终端2，复用已有相机依赖：

```bash
BRIDGE_PYTHON=/home/tieta/robot-bridge-track-b/.venv/bin/python \
  bash tools/start-bridge.sh readonly config/bridge.readonly.local.yaml
```

该模式拒绝所有运动/使能/恢复/回零动作；显示真实反馈，不发送CAN指令。
如果仅检查机械臂且没有相机，复制本地配置，把 `camera` 整段替换为 `camera: {driver: none}`。
不要给 `none` 保留相机宽高等构造参数。

查看SDK反馈和模型FK差异，记录铭牌与固件，采集诊断包后进入实机适配阶段。
**此版本不能通过加一个参数进入实机速度控制。**

## Windows平板与工业路由

- 电脑和平板运行同一个EXE包；现场只启动一个控制端。
- 先让平板与N100在路由器网络下互相可达，再填实际IP；不要照搬原办公网络地址。
- 修改连接设置前程序会断开并清空输入；新连接不自动恢复运动。
- 主/备链路沿用现有连接设置。备用路径切换要重新建立会话，不自动继续旧点动。
- 本轮不改变已有WebRTC推流服务；保留视频地址。D435→桥接MJPEG路径为独立采集进程。

| 端口 | 用途 | 配置要求 |
|---|---|---|
| N100 UDP9000 | 控制/ACK/遥测 | 路由可达，N100允许该UDP服务 |
| N100 TCP8080 | 诊断/管理/MJPEG | 客户端地址一致 |
| localhost TCP9088 | 桥接→ROS IPC | 只绑定127.0.0.1，不对路由开放 |
| WebRTC HTTP/媒体端口 | 视频 | 按原MediaMTX实际配置；不要把HTTP端口当媒体端口 |
| ROS DDS | 本地节点 | ROS_LOCALHOST_ONLY=1，不在路由器上开放DDS |

Windows如被防火墙拦截，为该EXE创建仅限现场机器人地址/专用网络的入站UDP规则。
客户端UDP本地端口由系统分配，不应误以为Windows也固定监听9000。
地址固定策略由现场网络配置决定；本工具不会更改网卡IP、DHCP或路由器。

## 回滚

先关闭Windows v8及N100的v8桥接、ROS终端，再启动旧目录中的原桥接与配套v7客户端。
旧目录未被改写，不需要从v8反向迁移配置。v8记录零位与旧零位分别保存，不能混用。
