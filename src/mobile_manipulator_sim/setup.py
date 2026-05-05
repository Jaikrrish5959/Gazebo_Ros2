from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mobile_manipulator_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaikrrishs',
    maintainer_email='jaikrrish5959@gmail.com',
    description='Simulation of an autonomous mobile manipulator',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perception = mobile_manipulator_sim.perception_node:main',
            'navigation = mobile_manipulator_sim.navigation_node:main',
            'manipulation = mobile_manipulator_sim.manipulation_node:main',
            'goal_sender = mobile_manipulator_sim.goal_sender_node:main',
        ],
    },
)
