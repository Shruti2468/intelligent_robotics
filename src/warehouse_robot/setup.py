from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shruti',
    maintainer_email='shrutishalom@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
       'console_scripts': [
            'odom_tf_publisher       = warehouse_robot.odom_tf_publisher:main',
            'static_map_publisher    = warehouse_robot.static_map_publisher:main',
            'path_planner            = warehouse_robot.path_planner:main',
            'robot_controller        = warehouse_robot.robot_controller:main',
            'mission_manager         = warehouse_robot.mission_manager:main',
        ],
    },
)