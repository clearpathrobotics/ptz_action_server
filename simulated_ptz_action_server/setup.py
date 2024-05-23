from glob import glob
import os

from setuptools import setup

package_name = 'simulated_ptz_action_server'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Iverach-Brereton',
    maintainer_email='civerachb@clearpathrobotics.com',
    description='Controller node for a simulated PTZ camera',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ptz_camera_sim_node = simulated_ptz_action_server.ptz_camera_sim_node:main'
        ],
    },
)
