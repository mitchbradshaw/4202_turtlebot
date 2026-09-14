import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mapping_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files so `ros2 launch mapping_pkg ...` can find them.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mitch',
    maintainer_email='mitch@todo.todo',
    description='METR4202 ArUco marker detection for TurtleBot3 Waffle Pi',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = mapping_pkg.camera_node:main',
        ],
    },
)
