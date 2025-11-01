from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranav',
    maintainer_email='pranav@todo.todo',
    description='Trajectory tracking node for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory = bot_trajectory.trajectory:main',
            'diagnol = bot_trajectory.diagnol:main',
            'path_marker = bot_trajectory.path_marker:main',
            'bot_pid = bot_trajectory.bot_pid:main',
            'obstacle = bot_trajectory.obstacle:main'
        ],
    },
)
