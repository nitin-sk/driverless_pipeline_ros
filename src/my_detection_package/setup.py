from setuptools import find_packages, setup

package_name = 'my_detection_package'

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
    maintainer='teamojasdv',
    maintainer_email='nitins.ojasracing@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = my_detection_package.camera_node:main',
            'yolo_node = my_detection_package.yolo_node:main',
            'display_node = my_detection_package.display_node:main',
        ],
    },
)

