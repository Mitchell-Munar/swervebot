from glob import glob

from setuptools import setup

setup(
    name='web_bridge_demo',
    version='0.0.0',
    packages=['web_bridge_demo'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/web_bridge_demo']),
        ('share/web_bridge_demo', ['package.xml']),
        ('share/web_bridge_demo/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Mitchell Munar',
    maintainer_email='munar.mitchell@gmail.com',
    description='WebSocket connectivity demo with a hostname ping service.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'ping_service_node = web_bridge_demo.ping_service_node:main',
    ]},
)
