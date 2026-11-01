from setuptools import setup
import os
from glob import glob

package_name = 'ros2_term_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/**')),
        # 모델은 디렉터리가 아닌 파일 단위로 나열해야 설치 가능
        (os.path.join('share', package_name, 'models', 'pavement_hill'),
            glob('models/pavement_hill/model.*')),
        (os.path.join('share', package_name, 'models', 'pavement_hill', 'meshes'),
            glob('models/pavement_hill/meshes/*')),
        (os.path.join('share', package_name, 'models', 'pavement_hill', 'textures'),
            glob('models/pavement_hill/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dong Hyeon Kwak',
    maintainer_email='asdqwe9501@gmail.com',
    description='Camera-based autonomous driving nodes for a Gazebo track (lane following, stop/end lines, obstacle detection)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_cube = ros2_term_project.obstacle_cube:main',  # obstacle_cube.py의 main 함수 호출
            'starter=ros2_term_project.starter:main',
            'controller=ros2_term_project.controller:main',
            'line_follower=ros2_term_project.line_follower:main',
            'line_detector=ros2_term_project.line_detector:main',
            'state_check=ros2_term_project.state_check:main',
        ],
    },
)
