from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_gym_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py') + glob('launch/*.xml')),
        (os.path.join('share', package_name, 'launch', 'slam'), glob('launch/slam/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        # foxglove_bridge and RViz resolve the URDF's package:// mesh URIs out of
        # the installed share directory, so the meshes have to land there too.
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'rviz'), glob('config/rviz/*.rviz')),
        (os.path.join('share', package_name, 'config', 'foxglove'), glob('config/foxglove/*.json')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Billy Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Bridge for using f1tenth_gym in ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_bridge = f1tenth_gym_ros.gym_bridge:main'
        ],
    },
)
