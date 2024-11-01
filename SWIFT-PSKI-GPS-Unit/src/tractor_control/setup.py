from setuptools import find_packages, setup

package_name = 'tractor_control'

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
    maintainer='Charles M. Raines',
    maintainer_email='cmr750@msstate.edu',
    description='Control pkg for ABE Tractor.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_py_pkg.my_node:main',
            'tractor_teleop_node = tractor_control.final_teleop:main',
            'tractor_motor_controller_node = tractor_control.final_controller:main'
        ],
    },
)
