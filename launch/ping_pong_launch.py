from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path

import pdb

def generate_launch_description():
    package_dir = Path(get_package_share_directory('ros2_behavior_tree_example'))
    behavior_tree_dir = package_dir / 'behavior_trees'

    # limit choices so we can only have available files
    mode_choices = ["sequence", "reactive_sequence"]
    tree_choices = ["ping_pong.xml", "ping_pong_no_decorator.xml", "ping_pong_executor.xml"]
    enable_choices = ["True", "False"]

    node1_enable_arg = DeclareLaunchArgument("node1_enable", 
                                            default_value="True",
                                            choices=enable_choices,
                                            description="Enable Primary Node in case you want to launch separately")
    node2_enable_arg = DeclareLaunchArgument("node2_enable", 
                                            default_value="True",
                                            choices=enable_choices,
                                            description="Enable Secondary Node in case you want to launch separately")

    node1_mode_arg = DeclareLaunchArgument("node1_mode", 
                                            default_value="reactive_sequence", 
                                            choices=mode_choices, 
                                            description="Set trees to reactive sequence")
    node2_mode_arg = DeclareLaunchArgument("node2_mode", 
                                            default_value="sequence", 
                                            choices=mode_choices, 
                                            description="Set trees to standard sequence")

    node1_behaviortree_arg = DeclareLaunchArgument("node1_behaviortree", 
                                                    default_value="ping_pong_executor.xml", 
                                                    choices=tree_choices, 
                                                    description="Set behevior tree file to use desired nodes")
    node2_behaviortree_arg = DeclareLaunchArgument("node2_behaviortree", 
                                                    default_value="ping_pong_no_decorator.xml", 
                                                    choices=tree_choices, 
                                                    description="Set behavior tree file to use desired nodes")

    primary_ping_pong_node = Node(
        condition=IfCondition(LaunchConfiguration("node1_enable")),
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name='primary_ping_pong',
        output='screen',
        remappings=[('incoming_ping', 'secondary_to_primary'),('outgoing_pong', 'primary_to_secondary')],
        parameters=[{
            "rate_hz" : 1.0,
            "num_republish": 5,
            "ping_starter" : True,
            "behaviortree_file" : PathJoinSubstitution([str(behavior_tree_dir),
                                                        LaunchConfiguration("node1_mode"),
                                                        LaunchConfiguration("node1_behaviortree")])
        }]
    )

    secondary_ping_pong_node = Node(
        condition=IfCondition(LaunchConfiguration("node2_enable")),
        package='ros2_behavior_tree_example',
        executable='behavior_tree_example',
        name='secondary_ping_pong',
        output='screen',
        remappings=[('incoming_ping', 'primary_to_secondary'),('outgoing_pong', 'secondary_to_primary')],
        parameters=[{            
            "rate_hz" : 0.75,
            "num_republish": 4,
            "ping_starter" : False,
            "behaviortree_file" : PathJoinSubstitution([str(behavior_tree_dir),
                                                        LaunchConfiguration("node2_mode"),
                                                        LaunchConfiguration("node2_behaviortree")])
            }]
    )

    return LaunchDescription([
        node1_enable_arg,
        node2_enable_arg,
        node1_mode_arg,
        node2_mode_arg,
        node1_behaviortree_arg,
        node2_behaviortree_arg,
        primary_ping_pong_node,
        secondary_ping_pong_node
    ])