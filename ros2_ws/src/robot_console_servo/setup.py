from setuptools import setup
from pathlib import Path

data=[('share/ament_index/resource_index/packages',['resource/robot_console_servo']),('share/robot_console_servo',['package.xml'])]
for folder in ('launch','config','model'):
    for p in Path(folder).rglob('*'):
        if p.is_file():data.append(('share/robot_console_servo/'+str(p.parent),[str(p)]))
setup(name='robot_console_servo',version='8.1.0',packages=['robot_console_servo'],data_files=data,
      install_requires=['setuptools','numpy','scipy'],zip_safe=False,
      maintainer='Robot Console',maintainer_email='maintainer@example.invalid',
      description='Ground-only MoveIt Servo commissioning gateway',license='Proprietary',
      entry_points={'console_scripts':['motion_node=robot_console_servo.node:main','cpv_executor=robot_console_servo.cpv_node:main']})
