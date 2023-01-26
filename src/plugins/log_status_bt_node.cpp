#include "ros2_behavior_tree_example/plugins/log_status_bt_node.hpp"

namespace polymath
{
namespace bt_ros_example
{
    LogStatusNode::LogStatusNode(const std::string & action_name, const BT::NodeConfig & conf)
    : BT::SyncActionNode(action_name, conf),
      print_ping_pong_(false)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
        auto msg = getInput<std::string>("message");
        if(!msg)
        {
           throw BT::RuntimeError( "missing required input [message]: ", msg.error() );
        }

        auto print_ping_pong = getInput<bool>("print_ping_pong");
        if(print_ping_pong)
        {
            print_ping_pong_ = print_ping_pong.value(); 
        }

        message_ = msg.value();
    }

    LogStatusNode::~LogStatusNode()
    {
        RCLCPP_INFO(node_->get_logger(),"SHUTTING DOWN PUBLISH STATUS NODE");
    }

    BT::NodeStatus LogStatusNode::tick()
    {
        RCLCPP_INFO(node_->get_logger(), message_.c_str());

        if(print_ping_pong_)
        {
            int32_t ping_id = -1;
            int32_t pong_id = -1;
            getInput<int32_t>("ping_id", ping_id);
            getInput<int32_t>("pong_id", pong_id);
            RCLCPP_INFO(node_->get_logger(), "sent ping: %d, received pong: %d", ping_id, pong_id);
        }

        return BT::NodeStatus::SUCCESS;
    }
}
}
