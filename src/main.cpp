#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_behavior_tree_example/bt_ros_node.hpp"
#include "std_msgs/msg/string.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node = std::make_shared<polymath::bt_ros_example::BtRosNode>();

  // // This is the same as rclcpp::spin(node);
  rclcpp::executors::SingleThreadedExecutor executor;
  // // executor requires the base interface to run. Doesn't need anything else.
  executor.add_node(node->get_node_base_interface());
  // // Spin away, we don't need to spin once, or anything else as this is handled by a wall timer within the node
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
