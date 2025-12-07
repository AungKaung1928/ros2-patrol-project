from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'patrol_navigation_project'

setup(
  name=package_name,
  version='1.0.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/ament_index/resource_index/packages',
      ['resource/patrol_navigation_project']),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
      glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    (os.path.join('share', package_name, 'config'),
      glob('config/*.yaml')),
    (os.path.join('share', package_name, 'rviz'),
      glob('rviz/*.rviz')),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='allkg',
  maintainer_email='allkg@example.com',
  description='ROS2 autonomous patrol navigation system with Nav2',
  license='Apache-2.0',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'patrol_controller = patrol_navigation_project.patrol_controller:main',
    ],
  },
)
