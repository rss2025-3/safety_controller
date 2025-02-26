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
            "side": -1  # initial value, will be updated later
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

    # Define test4 node
    test4 = Node(
        package="wall_follower",
        executable="test_wall_follower",
        namespace='test4',
        parameters=[{
            "scan_topic": "/scan",
            "drive_topic": "/drive",
            "pose_topic": "/pose",
            "velocity": 2.,
            "desired_distance": 1.,
            "side": 1,
            "start_x": 5.,
            "start_y": -4.,
            "start_z": 3 * np.pi / 4.,
            "end_x": -4.,
            "end_y": -5.,
            "name": "short_left_far_angled"
        }],
        name='test_wall_follower',
        remappings=[
            ('/test4/pose', '/pose'),
            ('/test4/map', '/map'),
            ('/test4/base_link', '/base_link'),
            ('/test4/tf', '/tf'),
            ('/test4/tf_static', '/tf_static'),
        ]
    )

    # Parameter-setting commands for test4
    setup_side4_1 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side4_2 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '1'
        ]],
        shell=True
    )
    setup_side4_3 = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' param set ',
            '/wall_follower_ns/wall_follower ',
            'side ',
            '1'
        ]],
        shell=True
    )

    return LaunchDescription([
        LogInfo(msg='Launching wall_follower node'),
        wall_follower,
        # Give the wall_follower node time to start up before proceeding.
        TimerAction(
            period=2.0,
            actions=[LogInfo(msg='Starting Test 4')]
        ),
        TimerAction(
            period=2.5,
            actions=[test4]
        ),
        # Delay the parameter commands to ensure the wall_follower node is running.
        TimerAction(
            period=3.0,
            actions=[setup_side4_1]
        ),
        TimerAction(
            period=4.0,
            actions=[setup_side4_2]
        ),
        TimerAction(
            period=5.0,
            actions=[setup_side4_3]
        ),
    ])
