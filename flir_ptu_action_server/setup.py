import os
from glob import glob
from setuptools import setup

package_name = 'flir_ptu_action_server'

setup(
    name=package_name,
    version='0.2.0',
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
    description='PTZ control services for the Flir D46 and E46 PTU devices',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flir_ptu_action_server_node = flir_ptu_action_server.flir_ptu_action_server_node:main'
        ],
    },
)
