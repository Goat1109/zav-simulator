from glob import glob
from setuptools import setup

setup(name='zav_real_control', version='0.1.0', packages=['zav_real_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/zav_real_control']),
                  ('share/zav_real_control', ['package.xml', 'README.md', 'requirements.txt']),
                  ('share/zav_real_control/config', glob('config/*')),
                  ('share/zav_real_control/launch', glob('launch/*.launch.py')),
                  ('share/zav_real_control/firmware', glob('firmware/*')),
                  ('share/zav_real_control/docs', glob('docs/*'))],
      install_requires=['setuptools', 'PyYAML', 'pymavlink==2.4.50', 'pyserial>=3.5,<4'], zip_safe=False,
      maintainer='ZAV', maintainer_email='zav@example.invalid', license='Apache-2.0',
      description='Hardware-only FAST-LIVO2 external odometry and latched RC-flight landing failsafe',
      entry_points={'console_scripts': [
          'go_front = zav_real_control.offboard_node:go_front',
          'go_right = zav_real_control.offboard_node:go_right',
          'go_back = zav_real_control.offboard_node:go_back',
          'go_left = zav_real_control.offboard_node:go_left',
          'waypoint_square = zav_real_control.offboard_node:waypoint_square',
          'waypoint_circle = zav_real_control.offboard_node:waypoint_circle',
          'epic_frame_adapter = zav_real_control.epic_frame:main',
          'super_adapter = zav_real_control.super_adapter:main',
          'super_lio_adapter = zav_real_control.super_lio_adapter:main',
          'mavlink_bridge = zav_real_control.mavlink_node:main',
          'lio_bridge = zav_real_control.bridge:main',
          'baro_link = zav_real_control.parameter_node:main',
          'inspect_status = zav_real_control.inspect:main']})
