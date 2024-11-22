from setuptools import find_packages, setup

package_name = 'serial_cmd_vel_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'serial'],
    zip_safe=True,
    maintainer='charles',
    maintainer_email='cmr750@msstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_cmd_vel_pub_node = serial_cmd_vel_publisher.serial_cmd_vel_publisher:main'
        ],
    },
)
