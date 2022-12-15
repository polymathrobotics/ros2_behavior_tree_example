#include "plugins/publish_status_bt_node.h"

namespace bt_ros_example
{
    PublishStatusNode::PublishStatusNode(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::SyncActionNode(condition_name, conf),
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

    BT::NodeStatus PublishStatusNode::tick()
    {
        RCLCPP_INFO(node_->get_logger(), message_.c_str());

        if(print_ping_pong_)
        {
            int32_t ping_id = -1;
            int32_t pong_id = -1;
            getInput<int32_t>("ping_id", ping_id);
            getInput<int32_t>("pong_id", pong_id);
            RCLCPP_INFO(node_->get_logger(), "received ping: %d, sent pong: %d", ping_id, pong_id);
        }

        return BT::NodeStatus::SUCCESS;
    }
}