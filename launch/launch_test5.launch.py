from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
import numpy as np

def generate_launch_description():
    # Launch the wall_follower node
    wall_follower = Node(
        package="wall_follower",
        executable="wall_follower",
        namespace='wall_follower_ns',
        parameters=[{
            "scan_topic": "/scan",
            "drive_topic": "/drive",
            "velocity": 1.,
            "desired_distance": 1.,
            "side": -1  # initial value; will be updated below
        }],
        name='wall_follower',
        remappings=[
            ('/wall_follower_ns/pose', '/pose'),
            ('/wall_follower_ns/map', '/map'),
            ('/wall_follower_ns/base_link', '/base_link'),
            ('/wall_follower_ns/tf', '/tf'),
            ('/wall_follower_ns/tf_static', '/tf_static'),
        ]
    )

    # Define test5 node
    test5 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test5',
        parameters=[{
            "scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 2.,
            "desired_distance": 1.,
            "side": -1,
            "start_x": -4.,
            "start_y": -5.4,
            "start_z": -np.pi/6.,
            "end_x": -3.5,
            "end_y": 17.6,
            "name": "long_right"
        }],
        name='test_wall_follower',
        remappings=[
            ('/test5/pose', '/pose'),
            ('/test5/map', '/map'),
            ('/test5/base_link', '/base_link'),
            ('/test5/tf', '/tf'),
            ('/test5/tf_static', '/tf_static'),
        ]
    )

    # Parameter-setting commands for test5 (set side to -1)
    setup_side5_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_side5_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '-1'
        ]],
        shell=True
    )
    setup_side5_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '-1'
        ]],
        shell=True
    )

    return LaunchDescription([
        LogInfo(msg='Launching wall_follower node'),
        wall_follower,
        # Delay to ensure the wall_follower node is up before test5 starts
        TimerAction(
            period=2.0,
            actions=[LogInfo(msg='Starting Test 5')]
        ),
        TimerAction(
            period=2.5,
            actions=[test5]
        ),
        # Delay the parameter commands so the wall_follower node is available
        TimerAction(
            period=3.0,
            actions=[setup_side5_1]
        ),
        TimerAction(
            period=4.0,
            actions=[setup_side5_2]
        ),
        TimerAction(
            period=5.0,
            actions=[setup_side5_3]
        ),
    ])
