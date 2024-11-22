from setuptools import find_packages, setup

package_name = 'tractor_gps_driver'

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
    description='GPS Driver for AAI Tractor.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tractor_gps_node = tractor_gps_driver.gps_driver:main'
        ],
    },
)
