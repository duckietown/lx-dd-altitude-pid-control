from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'solution'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    package_data={'solution': ['z_pid.yaml']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duckietown',
    maintainer_email='info@duckietown.com',
    description='DD24 PID Altitude Tuning — student solution package',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
