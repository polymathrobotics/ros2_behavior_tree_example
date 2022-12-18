# ros2_behavior_tree_example
Toy example for running a simple behavior tree with ROS2

# Launch Arguments to Launch the Test Nodes
```bash
ros@docker-desktop:/workspaces/ros2_ws$ ros2 launch ros2_behavior_tree_example ping_pong_launch.py -s

    Arguments (pass arguments as '<name>:=<value>'):

        'node1_enable':
            Enable Primary Node in case you want to launch separately. Valid choices are: ['True', 'False']
            (default: 'True')

        'node2_enable':
            Enable Secondary Node in case you want to launch separately. Valid choices are: ['True', 'False']
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

# Activate the behavior tree nodes
## Primary Node
```bash
> ros2 lifecycle set /primary_ping_pong configure
Transitioning successful

> ros2 lifecycle set /primary_ping_pong activate
Transitioning successful
```

### Secondary Node
```bash
> ros2 lifecycle set /primary_ping_pong configure
Transitioning successful

> ros2 lifecycle set /primary_ping_pong activate
Transitioning successful
```

# General Lifecycle Node Commands
```bash
ros2 lifecycle get /NODE_NAME # returns lifecycle node status
ros2 lifecycle list /NODE_NAME # returns all lifecycles we can transition to
ros2 lifecycle set /NODE_NAME # allows you to set a valid lifecycle from available transitions
```