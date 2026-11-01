#!/usr/bin/env python3
#
# Copyright 2023. Prof. Jong Min Lee @ Dong-eui University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
import json


print(os.path.realpath(__file__))

ld = LaunchDescription()
# 차량 지정
car = 'PR001'

def generate_launch_description():
    # configuration
    # 월드 파일 설정
    world_file_name = 'car_track.world'
    world = os.path.join(get_package_share_directory('ros2_term_project'),
                         'worlds', world_file_name)
    print('world file name = %s' % world)

    # 월드에서 model://pavement_hill 을 찾을 수 있도록 모델 경로 등록
    model_path = os.path.join(get_package_share_directory('ros2_term_project'), 'models')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.environ.get('GAZEBO_MODEL_PATH', '') + ':' + model_path)

    # 가제보 시간 사용
    declare_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # 가제보 실행
    gazebo_run = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    obstacle_cube_node = Node(
        package='ros2_term_project',
        executable='obstacle_cube',
        name='obstacle_cube_node',
        output='screen',
    )

    # 지정된 차량 정보를 전달하는 노드
    starter = Node(
        package='ros2_term_project',
        executable='starter',
        name='starter',
        arguments=[car],
        output='screen'
    )

    # 차량의 주행을 관리하는 노드
    controller = Node(
        package='ros2_term_project',
        executable='controller',
        name='controller',
        output='screen'
    )

    # 차선 정보를 처리하는 노드
    line_follower = Node(
        package='ros2_term_project',
        executable='line_follower',
        name='line_follower',
        output='screen',
    )
    # 차선, 종료선, 정지선을 감지하는 노드
    line_detector = Node(
        package='ros2_term_project',
        executable='line_detector',  # 새로 추가된 LineDetector 노드 실행
        name='line_detector',
        output='screen',
    )

    # 차량 속도, 차선 침범 횟수를 확인하는 노드
    state_check = Node(
        package='ros2_term_project',
        executable='state_check',
        name='state_check',
        output='screen'
    )

    # 실행 목록에 추가
    ld.add_action(set_model_path)
    ld.add_action(obstacle_cube_node)
    ld.add_action(declare_argument)
    ld.add_action(gazebo_run)
    ld.add_action(line_follower)
    ld.add_action(line_detector)
    ld.add_action(state_check)
    ld.add_action(controller)
    # ld.add_action(starter)

    return ld
