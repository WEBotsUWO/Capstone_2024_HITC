from setuptools import find_packages, setup

package_name = 'laser_pointer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chris',
    maintainer_email='christopher_obieke@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "laser_pointer = laser_pointer.laser_end_effector:main",
            "servo_position = laser_pointer.servo_position:main",
            "position_publisher = laser_pointer.position_publisher:main"
        ],
    },
)
