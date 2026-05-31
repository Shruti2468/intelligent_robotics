from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/warehouse_robot'],
    ),

    (
        'share/warehouse_robot',
        ['package.xml'],
    ),

    (
        os.path.join('share', 'warehouse_robot', 'launch'),
        glob('launch/*.py'),
    ),

    (
        os.path.join('share', 'warehouse_robot', 'urdf'),
        glob('urdf/*'),
    ),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shruti',
    maintainer_email='shrutishalom@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_controller = warehouse_robot.robot_controller:main',
            'path_planner     = warehouse_robot.path_planner:main',
            'task_manager     = warehouse_robot.task_manager:main',   # ← NEW
            'dwa_robot_controller = warehouse_robot.dwa_controller:main',

        ],

    },
)
