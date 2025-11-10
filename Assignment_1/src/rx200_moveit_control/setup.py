

from glob import glob
from setuptools import find_packages, setup

package_name = 'rx200_moveit_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('rx200_moveit_control/config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master26',
    maintainer_email='ozcascante@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'rx200_moveit_client=rx200_moveit_control.rx200_moveit_action_client:main',
        'point_safety_checker=rx200_moveit_control.point_safety_checker:main',
        ],
    },
)
