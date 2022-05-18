import os
from setuptools import setup
from glob import glob

package_name = 'practice'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', f'{package_name}/launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joe Lin',
    maintainer_email='joe@icshop.com.tw',
    description='An example for ROS2: How to use ROSKY2 to learn ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f"hello_ros2 = {package_name}.hello_ros2:main",
            f"subscriber_test = {package_name}.subscriber_test:main",
            f"add_two_ints_server = {package_name}.add_two_ints_server:main",
            f"add_two_ints_client = {package_name}.add_two_ints_client:main",
            f"turtle = {package_name}.turtle:main",
            f"turtle_spawner = {package_name}.turtle_spawner:main",
            f"turtle_move = {package_name}.turtle_move:main",
            f"rosky2_where_to_go = {package_name}.rosky2_where_to_go:main",
            f"distance = {package_name}.distance:main",
            f"go_along = {package_name}.go_along:main",
        ],
    },
)
