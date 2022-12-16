from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationNotEquals
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from pathlib import Path

def generate_launch_description():
    package_dir = Path(get_package_share_directory('ros2_behavior_tree_example'))

    primary_ping_pong_node = Node(
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name='primary_ping_pong',
        output='screen',
        remappings=[('incoming_ping', 'secondary_to_primary'),('outgoing_pong', 'primary_to_secondary')],
        parameters=[{
            "rate_hz" : 1.0,
            "num_republish": 5,
            "ping_starter" : True,
            "behaviortree_file" : str(package_dir / "behavior_trees"/'reactive_sequence'/'ping_pong_no_decorator.xml')
        }]
    )

    secondary_ping_pong_node = Node(
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name='secondary_ping_pong',
        output='screen',
        remappings=[('incoming_ping', 'primary_to_secondary'),('outgoing_pong', 'secondary_to_primary')],
        parameters=[{            
            "rate_hz" : 0.75,
            "num_republish": 4,
            "ping_starter" : False,
            "behaviortree_file" : str(package_dir / "behavior_trees"/'reactive_sequence'/'ping_pong.xml')
            }]
    )

    return LaunchDescription([
        primary_ping_pong_node,
        secondary_ping_pong_node
    ])