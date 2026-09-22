"""Validated commissioning parameters. No hardware imports or side effects."""
from copy import deepcopy
import math

# default, minimum, maximum, Chinese explanation. Boolean entries have no range.
SAFETY_PARAMETERS = {
    "hold_translation_enabled": (False, None, None, "松手后额外位移超差检查；false 仍发送保持目标并记录偏移"),
    "hold_rotation_enabled": (False, None, None, "松手后姿态超差检查；不影响反馈失效与控制器异常处理"),
    "jog_translation_enabled": (False, None, None, "点动平移跟随误差检查；false 不因该误差拒绝点动"),
    "jog_translation_mm": (20.0, .1, 100., "仅在对应检查开启时生效，单位 mm"),
    "jog_rotation_enabled": (False, None, None, "点动姿态跟随误差检查；false 不因该误差拒绝点动"),
    "jog_rotation_deg": (1.0, .1, 30., "空间旋转角阈值，单位度；不是各欧拉角分量差"),
    "hold_translation_mm": (2.0, .1, 2., "仅 hold_translation_enabled=true 时触发停止；阈值只允许收紧"),
    "hold_rotation_deg": (1.0, .1, 1., "仅 hold_rotation_enabled=true 时触发停止；阈值只允许收紧"),
    "hold_monitor_s": (1.0, 1., 10., "松手后监测时长，单位秒"),
    "home_translation_lead_mm": (2.0, .1, 2., "回零跟随误差阈值，只允许收紧"),
    "home_rotation_lead_deg": (1.0, .1, 1., "回零旋转误差阈值，只允许收紧"),
    "home_position_tolerance_mm": (.5, .01, .5, "回零到位位置容差"),
    "home_rotation_tolerance_deg": (.2, .01, .2, "回零到位姿态容差"),
    "feedback_max_age_s": (.25, .05, .25, "反馈更新最大间隔；上限 250 ms，不允许禁用"),
}


def number(name, value, low, high):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be a finite number in {low}..{high}, got {value!r}")
    return float(value)


def resolve_safety(settings=None):
    if settings is None: settings = {}
    if not isinstance(settings, dict): raise ValueError("arm.safety_checks must be a mapping")
    unknown = set(settings) - set(SAFETY_PARAMETERS)
    if unknown: raise ValueError(f"Unknown arm.safety_checks keys: {sorted(unknown)}")
    result = {}
    for key, (default, low, high, _) in SAFETY_PARAMETERS.items():
        value = settings.get(key, default)
        if isinstance(default, bool):
            if type(value) is not bool: raise ValueError(f"arm.safety_checks.{key} must be true or false, without quotes")
            result[key] = value
        else: result[key] = number(f"arm.safety_checks.{key}", value, low, high)
    return result


def safety_schema():
    return {k: {"default": v[0], "minimum": v[1], "maximum": v[2], "comment": v[3]}
            for k, v in SAFETY_PARAMETERS.items()}


def validate_config(data):
    unknown = set(data)-{"arm","camera","network","telemetry","laser"}
    if unknown: raise ValueError(f"Unknown config sections: {sorted(unknown)}")
    arm = data["arm"]
    if arm.get("driver") == "ros_servo":
        if set(arm)-{"driver","endpoint"}: raise ValueError("ROS arm accepts only driver and endpoint; configure motion in ROS")
        if arm.get("endpoint","http://127.0.0.1:9088")!="http://127.0.0.1:9088": raise ValueError("ROS IPC must remain localhost:9088")
    if arm.get("driver") == "piper6":
        allowed = {"driver", "can_name", "judge_flag", "dh_is_offset", "enable_on_start", "enable_timeout_s",
                   "feedback_timeout_s", "nominal_command_period_s", "max_command_interval_s", "max_linear_speed_mm_s",
                   "max_angular_speed_deg_s", "move_speed_percent", "deadband", "workspace_mm", "orientation_deg", "home_path", "safety_checks"}
        if set(arm)-allowed: raise ValueError(f"Unknown arm keys: {sorted(set(arm)-allowed)}")
        resolve_safety(arm.get("safety_checks"))
        for name in ("judge_flag", "enable_on_start"):
            if name in arm and type(arm[name]) is not bool: raise ValueError(f"arm.{name} must be a boolean")
        if arm.get("dh_is_offset", 1) not in (0, 1): raise ValueError("arm.dh_is_offset must be 0 or 1")
        ranges = {"enable_timeout_s": (.1,30), "feedback_timeout_s": (.01,30),
                  "nominal_command_period_s": (.001,.1), "max_command_interval_s": (.001,.1),
                  "max_linear_speed_mm_s": (2,50), "max_angular_speed_deg_s": (1,10),
                  "move_speed_percent": (1,100), "deadband": (0,.99)}
        for name, bounds in ranges.items():
            if name in arm: number("arm."+name, arm[name], *bounds)
        for name, axes in (("workspace_mm",("x","y","z")), ("orientation_deg",("roll","pitch","yaw"))):
            if name not in arm: continue
            bounds = arm[name]
            if not isinstance(bounds,dict) or set(bounds)!=set(axes): raise ValueError(f"arm.{name} requires {axes}")
            for axis, pair in bounds.items():
                if not isinstance(pair,list) or len(pair)!=2: raise ValueError(f"arm.{name}.{axis} requires [min,max]")
                values=[number(f"arm.{name}.{axis}",v,-100000,100000) for v in pair]
                if values[0]>=values[1]: raise ValueError(f"arm.{name}.{axis}: min must be below max")
    network=data["network"]
    unknown=set(network)-{"bind_host","udp_port","http_port","telemetry_hz","command_timeout_s","sequence_reset_after_s","audit_path"}
    if unknown: raise ValueError(f"Unknown network keys: {sorted(unknown)}")
    for key, low, high in (("udp_port",1,65535),("http_port",1,65535),("telemetry_hz",10,20),
                           ("command_timeout_s",.05,.25),("sequence_reset_after_s",.1,30)):
        if key in network: number("network."+key,network[key],low,high)
    camera=data["camera"]
    if camera.get("driver")=="d435":
        unknown=set(camera)-{"driver","width","height","fps","jpeg_quality","serial","startup_timeout_s"}
        if unknown: raise ValueError(f"Unknown camera keys: {sorted(unknown)}")
    for key,low,high in (("width",1,4096),("height",1,2160),("fps",1,90),("jpeg_quality",1,100),("startup_timeout_s",.1,60)):
        if key in camera: number("camera."+key,camera[key],low,high)
    for section, keys in ((network,("udp_port","http_port")),(camera,("width","height","fps")),(arm,("move_speed_percent","dh_is_offset"))):
        for key in keys:
            if key in section and type(section[key]) is not int: raise ValueError(f"{key} must be an integer")


def effective_config(data):
    validate_config(data)
    result=deepcopy(data)
    arm=result["arm"]
    if arm.get("driver")=="piper6":
        defaults={"can_name":"can0","judge_flag":False,"dh_is_offset":1,"enable_on_start":False,
                  "enable_timeout_s":5.,"feedback_timeout_s":2.,"nominal_command_period_s":.05,
                  "max_command_interval_s":.1,"max_linear_speed_mm_s":50.,"max_angular_speed_deg_s":10.,
                  "move_speed_percent":10,"deadband":.02,"home_path":"recorded-home.json",
                  "workspace_mm":{"x":[-500.,500.],"y":[-500.,500.],"z":[0.,600.]},
                  "orientation_deg":{"roll":[-180.,180.],"pitch":[-90.,90.],"yaw":[-180.,180.]}}
        for key,value in defaults.items(): arm.setdefault(key,value)
        arm["safety_checks"]=resolve_safety(arm.get("safety_checks"))
        arm["max_linear_speed_mm_s"]=min(50.,float(arm.get("max_linear_speed_mm_s",50)))
        arm["max_angular_speed_deg_s"]=min(10.,float(arm.get("max_angular_speed_deg_s",10)))
        arm["max_command_interval_s"]=min(.05,float(arm.get("max_command_interval_s",.1)))
    for key,value in {"bind_host":"0.0.0.0","udp_port":9000,"http_port":8080,"telemetry_hz":10.,
                      "command_timeout_s":.25,"sequence_reset_after_s":2.,"audit_path":"logs/jog-v5.jsonl"}.items():
        result["network"].setdefault(key,value)
    if result["camera"].get("driver")=="d435":
        for key,value in {"width":640,"height":480,"fps":30,"jpeg_quality":80,"serial":None,"startup_timeout_s":8.}.items():
            result["camera"].setdefault(key,value)
    return result
