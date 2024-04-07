from setuptools import find_packages, setup

package_name = 'sensor_srvcli'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_srvcli.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='aartran@umich.edu',
    description='This package is designed to expose sensor data from a local server',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['server = sensor_srvcli.MinimalSensorServer:main',
                            'client = sensor_srvcli.MinimalSensorClient:main',
                            'visualizer = sensor_srvcli.DataViz:main',
        ],
    },
)
