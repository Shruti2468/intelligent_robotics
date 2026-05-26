from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'warehouse_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/warehouse_world'],
        ),

        (
            'share/warehouse_world',
            ['package.xml'],
        ),

        (
            os.path.join('share', 'warehouse_world', 'launch'),
            glob('launch/*.py'),
        ),

        (
            os.path.join('share', 'warehouse_world', 'worlds'),
            glob('worlds/*.world'),
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
        ],
    },
)
