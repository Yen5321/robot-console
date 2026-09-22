"""ROS 2 运动网关的本机 IPC 客户端。

该模块位于网络桥接层，只负责把 ``IArmController`` 调用转换成发往
``motion_node`` 的 HTTP 请求。这里不导入厂商 SDK，也不直接读写 CAN；
实机 CAN 的唯一写入者是独立的 CPV 执行进程。
"""
import json
import threading
import time
import urllib.request
import urllib.error
from ..interfaces import IArmController
from ..errors import ArmInputRejected, ArmNotReady


class RosServoArm(IArmController):
    """通过 localhost HTTP 接口控制 ROS 2 Servo 网关。"""

    def __init__(self, endpoint='http://127.0.0.1:9088', **settings):
        # 控制接口必须限制在本机，避免绕过 UDP 桥的会话和安全检查。
        if endpoint != 'http://127.0.0.1:9088':
            raise ValueError('v8 motion IPC must use http://127.0.0.1:9088')
        self.endpoint = endpoint

        # 最近一帧有效 UDP 控制包在桥接进程中的单调时钟时间
        # jog_cartesian 会原样转发它，不能用转发时刻给旧输入续期
        self._stamp = time.monotonic()

        # 诊断接口最多约 25 Hz 刷新，避免多个调用者重复请求 ROS 网关
        self._cache = {}
        self._cache_at = 0.0
        self._home_active = False
        self._lock = threading.RLock()

    def _request(self, path, payload=None):
        """调用 ROS 网关，并统一转换通信错误和业务拒绝。"""
        data = None if payload is None else json.dumps(payload, allow_nan=False).encode()
        req = urllib.request.Request(
            self.endpoint + path,
            data=data,
            headers={'Content-Type': 'application/json'},
        )
        try:
            # 150 ms 超时短于控制租期，后端卡住时让上层尽快进入安全路径。
            with urllib.request.urlopen(req, timeout=.15) as response:
                result = json.load(response)
        except (OSError, urllib.error.URLError, ValueError) as exc:
            raise ArmNotReady('ROS backend unavailable: ' + str(exc)) from exc
        if not result.get('ok', True):
            raise ArmInputRejected(result.get('error', 'ROS request rejected'))
        return result

    def note_control(self, stamp):
        """记录有效控制包的原始接收时间，并刷新会话心跳。"""
        self._stamp = stamp
        self._request('/heartbeat', {'received_at': stamp})

    def jog_cartesian(self, dx, dy, dz, dpitch, dyaw):
        """转发五轴归一化点动输入及其原始时间戳。"""
        self._request(
            '/input',
            {'input': [dx, dy, dz, dpitch, dyaw], 'received_at': self._stamp},
        )

    def hold_position(self):
        """停止当前点动或回零；该动作不等同于电子急停。"""
        self._home_active = False
        self._request('/action', {'action': 'stop'})

    def _action(self, action):
        """发送离散动作，并使本地诊断缓存立即失效。"""
        result = self._request('/action', {'action': action})
        with self._lock:
            self._cache_at = 0.0
        return result

    # 以下短方法用于完整实现 IArmController，让桥接服务不必了解 HTTP 协议。
    def enable(self):
        self._action('enable')

    def recover(self):
        self._action('recover')

    def set_home(self):
        self._action('set_home')

    def home(self):
        self._action('home')
        self._home_active = True
        return True

    def advance_home(self, goal, dt):
        """查询异步回零是否完成；路径规划由 ROS 网关负责。"""
        done = not self.get_diagnostics().get('returning_home', False)
        if done:
            self._home_active = False
        return done

    def estop(self):
        """请求 ROS/CPV 链路执行手动电子急停。"""
        self._request('/stop', {'reason': 'manual_emergency_stop', 'emergency': True})

    def fault_stop(self, reason):
        """报告普通故障并请求停止，不主动升级为电子急停。"""
        self._request('/stop', {'reason': reason, 'emergency': False})

    def request_priority_stop(self, reason):
        """根据上层原因标记决定是否请求电子急停。"""
        self._request(
            '/stop',
            {'reason': reason, 'emergency': 'emergency' in reason.lower()},
        )

    def set_height_delta(self, delta_mm):
        # v8 只允许连续五轴输入，不保留旧版离散高度控制旁路。
        raise ArmInputRejected('Use five-axis continuous input')

    def get_joint_margins(self):
        return self.get_diagnostics().get('joint_margins', [0.0] * 6)

    def get_pose_safety_ok(self):
        return self.get_diagnostics().get('ready', False)

    def get_diagnostics(self):
        """返回诊断快照，并在本地继续计算反馈年龄。"""
        with self._lock:
            now = time.monotonic()
            if now - self._cache_at > .04:
                try:
                    self._cache = self._request('/diagnostics')
                    self._cache_at = now
                except ArmNotReady as exc:
                    # 返回结构化的“不就绪”状态，供遥测继续工作；不要伪造反馈。
                    return {
                        'ready': False,
                        'error': str(exc),
                        'blocking_code': 'ros_unavailable',
                        'enabled': [False] * 6,
                        'feedback_present': False,
                        'pose': None,
                        'backend': 'ros2_moveit_servo',
                    }

            # 复制后再补偿年龄，避免修改共享缓存本身。
            result = dict(self._cache)
            age = result.get('feedback_age_ms')
            if age is not None:
                result['feedback_age_ms'] = age + (now - self._cache_at) * 1000
                if result['feedback_age_ms'] >= 250:
                    result.update(
                        ready=False,
                        feedback_present=False,
                        error='feedback_expired',
                    )
            return result

    @property
    def motion_may_be_active(self):
        """最近诊断是否表明机械臂可能仍处于运动状态。"""
        return self._cache.get('motion_may_be_active', False)

    @property
    def _estop_latched(self):
        """兼容上层接口：返回最近诊断中的故障锁存状态。"""
        return self._cache.get('fault_latched', False)

    def check_feedback(self):
        """验证反馈可用性；故障锁存与反馈过期使用不同异常。"""
        report = self.get_diagnostics()
        if report.get('fault_latched'):
            raise RuntimeError(report.get('error', 'ROS fault'))
        if not report.get('feedback_present'):
            raise ArmNotReady(report.get('error', 'feedback_expired'))

    def close(self):
        """关闭适配器前尽力发送停止请求。"""
        try:
            self.hold_position()
        except ArmInputRejected:
            # 后端已经拒绝时不能在清理阶段掩盖原始退出原因。
            pass
