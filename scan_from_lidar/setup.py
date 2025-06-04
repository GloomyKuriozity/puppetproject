from setuptools import setup
import os
from glob import glob

package_name = 'scan_from_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_from_lidar = scan_from_lidar.scan_from_lidar:main',
            'static_tf2_broadcaster = scan_from_lidar.static_tf2_broadcaster:main',
        ],
    },
)
