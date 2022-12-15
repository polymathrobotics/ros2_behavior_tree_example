#include <chrono>

#include "ros2_behavior_tree_example/bt_ros_node.h"
#include "ros2_behavior_tree_example/plugins/ping_received_bt_node.h"
#include "ros2_behavior_tree_example/plugins/pong_bt_node.h"
#include "ros2_behavior_tree_example/plugins/publish_status_bt_node.h"

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/publisher.hpp"

#include "std_msgs/msg/string.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.h"

using namespace std::chrono_literals;

namespace bt_ros_example
{
    BtRosNode::BtRosNode(const rclcpp::NodeOptions &options)
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
        this->declare_parameter("num_republish", int32_t(3));
        this->declare_parameter("ping_starter", true);

        // Declare the behavior tree default file
        this->declare_parameter("behaviortree_file", "behavior_trees/");

        // Register Nodes into the Factory to generate a tree later
        factory_.registerNodeType<PingReceivedNode>("PingReceivedNode");
        factory_.registerNodeType<PongNode>("PongNode");
        factory_.registerNodeType<PublishStatusNode>("PublishStatusNode");
    }

    BtRosNode::~BtRosNode()
    {}

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_configure(const rclcpp_lifecycle::State &)
    {
        // Set up the blackboard for the behavior tree
        blackboard_ = BT::Blackboard::create();
        blackboard_->set<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node", this->shared_from_this());
        blackboard_->set<int64_t>("num_publish", this->get_parameter("num_republish").as_int());
        blackboard_->set<bool>("ping_start", this->get_parameter("ping_starter").as_bool());
        blackboard_->set<int64_t>("ping_id", 0);
        blackboard_->set<int64_t>("pong_id", 0);

        tree_ = factory_.createTreeFromFile(this->get_parameter("behaviortree_file").as_string());
        
        // Running a timer to run this at a stable rate
        // This enables us to run the executor with just a spin at the upper level
        std::chrono::milliseconds rate(int64_t(1000.0 / this->get_parameter("rate_hz").as_double()));
        timer_ = this->create_wall_timer(rate, 
                                        std::bind(&BtRosNode::timer_callback, this));

        // start with the timer cancelled
        RCLCPP_INFO(this->get_logger(), "Stopping Timer from running");
        timer_->cancel();

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(this->get_logger(), "Starting the Timer and running Ticks");
        timer_->reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(this->get_logger(), "Stopping Timer");
        timer_->cancel();

        // We can wait until Cancel as well by doing timer_->is_cancelled()

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
    {
        blackboard_.reset();
        timer_.reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    BtRosNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
    {
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void
    BtRosNode::timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Ticking the tree");
        tree_.tickOnce();
        return;
    }

}
