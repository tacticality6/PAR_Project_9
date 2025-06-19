from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'par_project_9'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmitaiil',
    maintainer_email='s39477643@student.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"aruco_detector = {package_name}.aruco_detector:main",
            f"task_controller = {package_name}.task_controller:main",
            f"delivery_tracker = {package_name}.delivery_tracker_node:main",
            # f"visual_servoing = {package_name}.visual_servoing:main",
            f"visual_servoing = {package_name}.test_vs:main",
            # f"pointer_detector = {package_name}.pointer_detector:main",
        ],
    },
)
