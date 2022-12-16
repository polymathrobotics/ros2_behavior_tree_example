# ros2_behavior_tree_example
Toy example for running a simple behavior tree with ROS2

# Launch Arguments to Launch the Test Nodes
```bash
> ros2 launch ros2_behavior_tree_example ping_pong_launch.py -s

Arguments (pass arguments as '<name>:=<value>'):

    'node1_enable':
        Enable Secondary Node in case you want to launch separately
        (default: 'True')

    'node2_enable':
        Enable Secondary Node in case you want to launch separately
        (default: 'True')

    'node1_mode':
        Set trees to reactive sequence. Valid choices are: ['sequence', 'reactive_sequence']
        (default: 'reactive_sequence')

    'node2_mode':
        Set trees to standard sequence. Valid choices are: ['sequence', 'reactive_sequence']
        (default: 'sequence')

    'node1_behaviortree':
        Set behevior tree file to use desired nodes. Valid choices are: ['ping_pong.xml', 'ping_pong_no_decorator.xml', 'ping_pong_executor.xml']
        (default: 'ping_pong_executor.xml')

    'node2_behaviortree':
        Set behavior tree file to use desired nodes. Valid choices are: ['ping_pong.xml', 'ping_pong_no_decorator.xml', 'ping_pong_executor.xml']
        (default: 'ping_pong_no_decorator.xml')
```