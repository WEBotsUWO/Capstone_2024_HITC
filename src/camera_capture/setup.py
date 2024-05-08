from setuptools import find_packages, setup

package_name = 'camera_capture'

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
            "camera_capture = camera_capture.camera_capture:main",
            "image_proccessing = camera_capture.image_processing:main",
            "safety_system = camera_capture.safety_system:main",
            "motor_controller = camera_capture.motor_controller:main"
        ],
    },
)
