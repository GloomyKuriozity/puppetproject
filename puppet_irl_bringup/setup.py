from setuptools import find_packages, setup

package_name = 'puppet_irl_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/puppet_irl_bringup/launch', ['launch/slam_toolbox.launch.py']),
        ('share/puppet_irl_bringup/launch', ['launch/nav2_launch.py']),
        ('share/puppet_irl_bringup/launch', ['launch/mapping.launch.py']),
        ('share/puppet_irl_bringup/launch', ['launch/localization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mélanie Geulin',
    maintainer_email='melanie.geulin@gmail.com',
    description="Bring_up all necessary Puppet topics and systems",
    license='MIT Licence',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
