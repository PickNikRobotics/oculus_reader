from glob import glob

from setuptools import setup

package_name = 'data_collection'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # ament_index marker so this package is discoverable.
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='Apache-2.0',
    description='Quest-driven VR teleop and data collection tools for MoveIt Pro',
    entry_points={
        'console_scripts': [
            'teleoperate = data_collection.teleoperate:main',
        ],
    },
)
