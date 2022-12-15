#include "ros2_behavior_tree_example/plugins/pong_bt_node.h"
#include "std_msgs/msg/int32.hpp"

namespace bt_ros_example
{

    PongNode::PongNode(const std::string &action_name, const BT::NodeConfig &conf)
    : BT::StatefulActionNode(action_name, conf),
    pub_topic_("outgoing_pong")
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
        pub_ = node_->create_publisher<std_msgs::msg::Int32>(pub_topic_,
                                                        rclcpp::SystemDefaultsQoS());

        pong_msg_.data = 0;
        return;
    }

    PongNode::~PongNode()
    {
        RCLCPP_INFO(node_->get_logger(),"SHUTTING DOWN PONG NODE");
    }

    BT::NodeStatus PongNode::onStart()
    {
        getInput<int32_t>("num_pongs", num_pongs_ );
        curr_pong_in_burst_ = 0;

        RCLCPP_INFO(node_->get_logger(), "STARTING PONG");

        publish();

        
        if(num_pongs_ <= 1)
        {
            return BT::NodeStatus::SUCCESS;
        }

        curr_pong_in_burst_++;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus PongNode::onRunning()
    {
        RCLCPP_INFO(node_->get_logger(), "RUNNING PONG");
        
        // always start by publishing
        publish();
        curr_pong_in_burst_++;

        if(curr_pong_in_burst_ == num_pongs_)
        {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void PongNode::onHalted()
    {
        // Do any necessary cleanup for the running nodes
        RCLCPP_INFO(node_->get_logger(), "HALTING RUN");
        return;
    }

    void PongNode::publish()
    {
        pong_msg_.data++;
        setOutput("last_pong_id", pong_msg_.data);
        pub_->publish(pong_msg_);
    }


}
