from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'voice_kick_ball'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 注册 launch 文件夹
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='originman',
    maintainer_email='originman@todo.todo',
    description='OriginMan Kick Ball Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 格式：'你在launch里写名字 = 包名(内部文件夹名).文件名(不带.py):main'
            'voice_kick_node = voice_kick_ball.voice_kick_node:main',
            'kick_ball = voice_kick_ball.kick_ball:main'
        ],
    },
)