#include <bt_ros_node.h>

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.h"

namespace bt_ros_example
{
    BtRosNode::BtRosNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("bt_ros_node", "", options)
    {
        // Get all necesssary params for ros2 to utilize
        
        // Rate at which to run our system
        // Set up parameter descriptor to set up our range and settings
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor();
        param_desc.floating_point_range.resize(1);
        param_desc.floating_point_range[0].from_value = 0.5; 
        param_desc.floating_point_range[0].to_value = 100;
        param_desc.floating_point_range[0].step = 10;
        param_desc.description = "Rate in Hz to run behavior tree at";

        this->declare_parameter("rate_hz", float_t(30), param_desc);

        // Declare the behavior tree default file
        this->declare_parameter("behaviortree_file", "");

        // Set up the blackboard for the behavior tree
        this->blackboard_ = BT::Blackboard::create();

        auto pub = this->create_publisher<std_msgs::msg::String>("abc", 10);
    }
}