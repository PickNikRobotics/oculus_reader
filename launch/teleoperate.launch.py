"""
Launch the Quest teleop node and a static TF that aligns the Quest tracking
frame to the robot's world frame.

The static TF rotation should match the *relative orientation* between the
human teleoperator and the robot at recenter time. Translation does not
matter for the clutch math (only frame deltas are used), so it is fixed at
zero -- the rotation is the only thing you typically need to adjust.

Default orientation
-------------------
Assumes the operator is behind (or to the side of) the robot, facing in the
*same direction* as the robot (i.e. the operator's "forward" is aligned with
the robot's "forward" in world frame). This is the most common stance for
desktop teleop: you stand next to or behind the robot and reach into its
workspace.

If you want to face the robot instead (the operator's "forward" pointing
toward the robot, opposite to the robot's "forward"), you need to override
the quaternion -- otherwise left/right and forward/back will feel inverted.

Calibration recipe
------------------
  1. Stand on a fixed, marked spot near the robot.
  2. Stand still for a couple seconds, then long-press the Quest's Oculus
     button to recenter.
  3. Run this launch file. Wave a controller in RViz to verify "hand left"
     moves the reference frame to your left and "hand forward" moves it
     away from you.

Override the orientation at launch time with qx/qy/qz/qw arguments:

    ros2 launch /path/to/teleoperate.launch.py qx:=... qy:=... qz:=... qw:=...
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.realpath(__file__))
    teleop_script = os.path.realpath(
        os.path.join(here, '..', 'oculus_reader', 'teleoperate.py')
    )

    # Only the orientation matters for the clutch math; translation is fixed
    # at the world origin since deltas are unaffected by it.
    args = [
        DeclareLaunchArgument('qx', default_value='0.5',
            description='Quest->world rotation, x component (xyzw)'),
        DeclareLaunchArgument('qy', default_value='0.5',
            description='Quest->world rotation, y component (xyzw)'),
        DeclareLaunchArgument('qz', default_value='0.5',
            description='Quest->world rotation, z component (xyzw)'),
        DeclareLaunchArgument('qw', default_value='0.5',
            description='Quest->world rotation, w component (xyzw)'),
        DeclareLaunchArgument('world_frame', default_value='world',
            description='Robot world frame (TF parent of quest_origin)'),
        DeclareLaunchArgument('quest_frame', default_value='quest_origin',
            description='Quest tracking frame (TF parent of controller poses)'),
        DeclareLaunchArgument('linear_gain', default_value='1.0',
            description='m/s of EE velocity per metre of position error'),
        DeclareLaunchArgument('angular_gain', default_value='1.0',
            description='rad/s of EE angular velocity per rad of orientation error'),
    ]

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_quest_origin',
        arguments=[
            '--x',  '0', '--y',  '0', '--z',  '0',
            '--qx', LaunchConfiguration('qx'),
            '--qy', LaunchConfiguration('qy'),
            '--qz', LaunchConfiguration('qz'),
            '--qw', LaunchConfiguration('qw'),
            '--frame-id',       LaunchConfiguration('world_frame'),
            '--child-frame-id', LaunchConfiguration('quest_frame'),
        ],
        output='screen',
    )

    teleop = ExecuteProcess(
        cmd=[
            'python3', teleop_script,
            '--ros-args',
            '-p', ['parent_frame_id:=', LaunchConfiguration('quest_frame')],
            '-p', ['linear_gain:=',     LaunchConfiguration('linear_gain')],
            '-p', ['angular_gain:=',    LaunchConfiguration('angular_gain')],
        ],
        output='screen',
    )

    return LaunchDescription([*args, static_tf, teleop])
