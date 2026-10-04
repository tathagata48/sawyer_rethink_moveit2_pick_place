"""Sawyer pick and place.

Terminal 1:  ros2 launch rethink_moveit_config demo.launch.py
Terminal 2:  ros2 launch rethink_pick_place pick_place.launch.py
             ros2 launch rethink_pick_place pick_place.launch.py cycles:=4
             ros2 launch rethink_pick_place pick_place.launch.py add_obstacle:=false velocity_scaling:=0.8
             ros2 launch rethink_pick_place pick_place.launch.py pick_y:=0.30 place_y:=-0.30
List all options:  ros2 launch rethink_pick_place pick_place.launch.py --show-args
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

# name: (default, type, description)
ARGS = {
    'arm_group': ('arm', str, 'planning group of the arm'),
    'tcp_link': ('right_gripper_tip', str, 'link placed at the grasp point (z = approach axis)'),
    'pick_x': ('0.65', float, 'can start position x [m]'),
    'pick_y': ('0.20', float, 'can start position y [m]'),
    'place_x': ('0.65', float, 'place position x [m]'),
    'place_y': ('-0.20', float, 'place position y [m]'),
    'table_top_z': ('-0.15', float, 'table surface height [m]'),
    'table_x': ('0.75', float, 'table centre x [m]'),
    'table_size_x': ('0.70', float, 'table depth [m]'),
    'table_size_y': ('1.20', float, 'table width [m]'),
    'can_radius': ('0.012', float, 'can radius [m]; must fit the open gripper'),
    'can_height': ('0.08', float, 'can height [m]'),
    'approach_distance': ('0.12', float, 'straight-line approach/retreat length [m]'),
    'add_obstacle': ('true', bool, 'put an obstacle between pick and place'),
    'obstacle_height': ('0.20', float, 'obstacle height [m]'),
    'cycles': ('2', int, 'number of moves; the can goes back and forth'),
    'velocity_scaling': ('0.4', float, 'speed of free-space moves (0-1]'),
    'line_speed': ('0.2', float, 'speed of the straight approach/retreat (0-1]'),
}


def convert(value, typ):
    """Launch arguments are strings; parameters need real types."""
    if typ is bool:
        return value.lower() in ('true', '1', 'yes')
    return typ(value)


def launch_setup(context):
    # runs at launch time, when the actual argument values are known
    params = {k: convert(LaunchConfiguration(k).perform(context), typ)
              for k, (_, typ, _) in ARGS.items()}
    moveit_config = MoveItConfigsBuilder(
        'sawyer', package_name='rethink_moveit_config').to_moveit_configs()
    return [Node(
        package='rethink_pick_place', executable='pick_place', output='screen',
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    params])]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=d, description=desc)
         for k, (d, _, desc) in ARGS.items()]
        + [OpaqueFunction(function=launch_setup)])