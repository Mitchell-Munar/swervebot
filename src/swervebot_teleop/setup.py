import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swervebot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Mitchell Munar',
    maintainer_email='munar.mitchell@gmail.com',
    description='Joystick teleoperation, dynamic button remapping, drive mode state machine, and manual/autonomous control arbitration for swervebot.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_node = swervebot_teleop.teleop_node:main'
        ],
    },
)
