from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pid_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Duckietown',
    maintainer_email='info@duckietown.com',
    description='DD24 altitude PID controller node',
    license='None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_controller_node = pid_controller.pid_controller_node:main',
        ],
    },
)
