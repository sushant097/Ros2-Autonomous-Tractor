from setuptools import find_packages, setup

package_name = 'depthai_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'depthai',
        'opencv-python-headless',
        'numpy',
        'cv_bridge',  # Required for ROS 2 image message conversions
    ],
    zip_safe=True,
    maintainer='Sushant',
    maintainer_email='sg2223@msstate.edu',
    description='A ROS 2 package for publishing RGB and depth images from a Luxonis camera.',
    license='Apache License 2.0',  # Specify the license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depthai_publisher = depthai_publisher.depthai_publisher_node:main',
        ],
    },
)
 