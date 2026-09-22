from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    mode=LaunchConfiguration('mode').perform(context)
    if mode not in ('sim','readonly','real'):
        raise RuntimeError('Allowed: sim, readonly, real (commissioning gates enforced)')
    home_path=LaunchConfiguration('home_path').perform(context) or f'recorded-home-v8.1-{mode}.json'
    share=Path(get_package_share_directory('robot_console_servo'))
    from robot_console_servo.settings import load
    motion=load(share/'config/motion.yaml')
    root=ET.fromstring((share/'model/piper_l/piper_l_stock_gripper.urdf').read_text())
    for limit in root.iter('limit'):limit.set('velocity',str(.05))
    urdf=ET.tostring(root,encoding='unicode')
    semantic=(share/'model/piper_l/piper_l.srdf').read_text()
    description={'robot_description':urdf,'robot_description_semantic':semantic}
    limits={'robot_description_planning':{'joint_limits':{f'joint{i}':{'has_velocity_limits':True,'max_velocity':motion['joint_velocity_rad_s'],'has_acceleration_limits':True,'max_acceleration':motion['joint_acceleration_rad_s2']} for i in range(1,7)}}}
    result=[Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf}]),
            Node(package='robot_console_servo',executable='motion_node',output='screen',parameters=[{'robot_description':urdf,'mode':mode,'motion_config':str(share/'config/motion.yaml'),'can_name':LaunchConfiguration('can_name').perform(context),'home_path':home_path}])]
    if mode in ('sim','real'):
        result.append(Node(package='console_servo_stamp',executable='stamp_servo',output='screen'))
        servo=yaml.safe_load((share/'config/servo.yaml').read_text())
        servo['publish_period']=1./motion['cycle_hz']
        result.append(Node(package='moveit_servo',executable='servo_node_main',name='servo_node',output='screen',parameters=[description,limits,{'moveit_servo':servo}]))
    if mode=='real':result.append(Node(package='robot_console_servo',executable='cpv_executor',output='screen',parameters=[{'robot_description':urdf,'can_name':LaunchConfiguration('can_name').perform(context)}]))
    processes=list(result)
    for process in processes:
        result.append(RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='A v8 ROS process exited; restart requires explicit enable'))])))
    return result


def generate_launch_description():
    return LaunchDescription([SetEnvironmentVariable('ROS_LOCALHOST_ONLY','1'),
        DeclareLaunchArgument('mode',default_value='sim'),
        DeclareLaunchArgument('can_name',default_value='can0'),
        DeclareLaunchArgument('home_path',default_value=''),
        OpaqueFunction(function=nodes)])
