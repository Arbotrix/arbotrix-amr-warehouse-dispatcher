from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_dispatcher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arbotrix Robotics',
    maintainer_email='robotics@arbotrix.local',
    description='Autonomous warehouse AMR dispatcher using Nav2 NavigateToPose.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dispatcher = warehouse_dispatcher.dispatcher_node:main',
            'particlecloud_bridge = warehouse_dispatcher.particlecloud_bridge:main',
            'web_dispatcher = warehouse_dispatcher.web_dispatcher_node:main',
        ],
    },
)
