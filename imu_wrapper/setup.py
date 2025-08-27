import os
from glob import glob
from setuptools import setup

package_name = 'imu_wrapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian Stelter',
    maintainer_email='sebastian.stelter@cranfield.ac.uk',
    description='Wrapper for the Sparkfun ICM20948 IMU',
    license='TODO',
    entry_points={
        'console_scripts': [
            'imu_wrapper = imu_wrapper.imu_wrapper:main',
        ],
    },
)