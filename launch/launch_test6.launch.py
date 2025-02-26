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
            "velocity": 1.,            # initial value, to be updated
            "desired_distance": 1.,      # initial value, to be updated
            "side": -1                 # initial value, to be updated
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

    # Define test6 node
    test6 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test6',
        parameters=[{
            "scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 3.,
            "desired_distance": 0.72,
            "side": 1,
            "start_x": -7.,
            "start_y": 10.6,
            "start_z": 0.,
            "end_x": -4.,
            "end_y": -5.,
            "name": "long_left"
        }],
        name='test_wall_follower',
        remappings=[
            ('/test6/pose', '/pose'),
            ('/test6/map', '/map'),
            ('/test6/base_link', '/base_link'),
            ('/test6/tf', '/tf'),
            ('/test6/tf_static', '/tf_static'),
        ]
    )

    # Parameter-setting commands for test6

    # Setup 'side' to 1
    setup_side6_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side6_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side6_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '1'
        ]],
        shell=True
    )

    # Setup 'velocity' to 3.0
    setup_v6_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'velocity ',
            '3.'
        ]],
        shell=True
    )
    setup_v6_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'velocity ',
            '3.'
        ]],
        shell=True
    )
    setup_v6_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'velocity ',
            '3.'
        ]],
        shell=True
    )

    # Setup 'desired_distance' to 0.72
    setup_d6_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'desired_distance ',
            '0.72'
        ]],
        shell=True
    )
    setup_d6_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'desired_distance ',
            '0.72'
        ]],
        shell=True
    )
    setup_d6_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'desired_distance ',
            '0.72'
        ]],
        shell=True
    )

    return LaunchDescription([
        LogInfo(msg='Launching wall_follower node'),
        wall_follower,
        # Delay to ensure the wall_follower node is up
        TimerAction(
            period=2.0,
            actions=[LogInfo(msg='Starting Test 6')]
        ),
        TimerAction(
            period=2.5,
            actions=[test6]
        ),
        # Delay the parameter commands to update wall_follower for test6
        TimerAction(
            period=3.0,
            actions=[setup_side6_1]
        ),
        TimerAction(
            period=3.5,
            actions=[setup_v6_1]
        ),
        TimerAction(
            period=4.0,
            actions=[setup_d6_1]
        ),
        TimerAction(
            period=4.5,
            actions=[setup_side6_2]
        ),
        TimerAction(
            period=5.0,
            actions=[setup_v6_2]
        ),
        TimerAction(
            period=5.5,
            actions=[setup_d6_2]
        ),
        TimerAction(
            period=6.0,
            actions=[setup_side6_3]
        ),
        TimerAction(
            period=6.5,
            actions=[setup_v6_3]
        ),
        TimerAction(
            period=7.0,
            actions=[setup_d6_3]
        ),
    ])
