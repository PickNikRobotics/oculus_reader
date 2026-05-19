from glob import glob

from setuptools import setup

package_name = 'oculus_reader'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # ament_index marker so this package is discoverable
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    include_package_data=True,
    package_data={'': ['APK/teleop-debug.apk']},
    install_requires=[
        'setuptools',
        'numpy',
        'pure-python-adb',
        'pyyaml',
    ],
    zip_safe=True,
    license='Apache-2.0',
    description='Oculus Quest controller reader and Quest-to-robot teleop bridge',
    entry_points={
        'console_scripts': [
            'teleoperate = oculus_reader.teleoperate:main',
            'reader = oculus_reader.reader:main',
        ],
    },
)
