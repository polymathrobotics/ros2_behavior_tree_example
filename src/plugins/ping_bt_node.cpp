#include "ros2_behavior_tree_example/plugins/ping_bt_node.hpp"
#include "std_msgs/msg/int32.hpp"

namespace polymath
{
namespace bt_ros_example
{

    PingNode::PingNode(const std::string &action_name, const BT::NodeConfig &conf)
    : BT::StatefulActionNode(action_name, conf),
    pub_topic_("outgoing_ping")
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
        pub_ = node_->create_publisher<std_msgs::msg::Int32>(pub_topic_,
                                                        rclcpp::SystemDefaultsQoS());

        ping_msg_.data = 0;
        return;
    }

    PingNode::~PingNode()
    {
        RCLCPP_INFO(node_->get_logger(),"SHUTTING DOWN PING NODE");
    }

    BT::NodeStatus PingNode::onStart()
    {
        getInput<int32_t>("num_pings", num_pings_ );
        curr_ping_in_burst_ = 0;

        RCLCPP_INFO(node_->get_logger(), "STARTING PING");

        publish();

        
        if(num_pings_ <= 1)
        {
            return BT::NodeStatus::SUCCESS;
        }

        curr_ping_in_burst_++;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus PingNode::onRunning()
    {
        RCLCPP_INFO(node_->get_logger(), "RUNNING PING");
        
        // always start by publishing
        publish();
        curr_ping_in_burst_++;

        if(curr_ping_in_burst_ == num_pings_)
        {
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;
    }

    void PingNode::onHalted()
    {
        // Do any necessary cleanup for the running nodes
        RCLCPP_INFO(node_->get_logger(), "HALTING RUN");
        return;
    }

    void PingNode::publish()
    {
        ping_msg_.data++;
        setOutput("last_ping_id", ping_msg_.data);
        pub_->publish(ping_msg_);
    }


}
}
