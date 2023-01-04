#include "ros2_behavior_tree_example/plugins/pong_received_bt_node.hpp"

namespace polymath
{
namespace bt_ros_example
{
    PongReceivedNode::PongReceivedNode(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    sub_topic_("incoming_pong"),
    pong_id_received_(-1)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

        // Setup subscription on running this node
        // Subscription will run when lifecycle node executor is called 
        sub_ = node_->create_subscription<std_msgs::msg::Int32>(sub_topic_,
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&PongReceivedNode::pong_callback, this, std::placeholders::_1));

        bool force_pong;
        getInput("force_pong", force_pong);

        if(force_pong)
        {
            // assume we're transmitting the first pong
            pong_id_received_=0;
        }
        return;
    }

    PongReceivedNode::~PongReceivedNode()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN PONG RECEIVED NODE");
        sub_.reset();
    }

    BT::NodeStatus PongReceivedNode::tick()
    {
        if(pong_id_received_ <= -1)
        {
            // Return failiure if we haven't received any pongs
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(node_->get_logger(), "TICK::PONG_NODE");

        // record last pong id received so that other nodes could use it
        setOutput("last_pong_id", pong_id_received_);

        // reset pong id since we've received it
        pong_id_received_ = -1;
        return BT::NodeStatus::SUCCESS;
    }

    void PongReceivedNode::pong_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "CALLBACK::RECEIVED PONG WITH ID %d", msg->data);
        pong_id_received_ = msg->data;
        return;
    }
}
}
