from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James',
    maintainer_email='james.petri@mu.ie',
    description='Example application using a service',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'loc_status=motor_control.localization_ready:main',
            'rbt_status=motor_control.system_ready:main',
            'status_manager=motor_control.status_manager:main',
            'motor_service=motor_control.motor_relay_service:main'
        ],
    },
)
