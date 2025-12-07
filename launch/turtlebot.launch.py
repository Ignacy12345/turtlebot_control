#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ustawienie modelu TurtleBota
        ExecuteProcess(
            cmd=['bash', '-c', 'export TURTLEBOT3_MODEL=burger && exec "$SHELL"'],
            shell=True,
            output='screen'
        ),

        # Uruchomienie Gazebo z robotem
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen'
        ),

        # TimerAction dodaje krótkie opóźnienie, żeby Gazebo zdążyło się załadować
        TimerAction(
            period=5.0,  # opóźnienie w sekundach
            actions=[
                # Węzeł kamery
                Node(
                    package='camera_subscriber',
                    executable='camera_node',
                    name='camera_node',
                    output='screen',
                    parameters=[{'square_size': 200}]
                ),

                # Węzeł sterowania robotem
                Node(
                    package='camera_subscriber',
                    executable='point_follower',
                    name='point_follower',
                    output='screen'
                ),
            ]
        )
    ])
