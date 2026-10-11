from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Einar Augestad, Ingrid Skålnes Strømme',
    maintainer_email='einaraug@stud.ntnu.no, ingskastro@gmail.com',
    description='Exploration package manages the movement of the turtlebot, using \
        algorithms to decide what path the bot takes in mapping towards frontier cells',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frontier_explorer = exploration.explorer_node:main',
        ],
    },
)
