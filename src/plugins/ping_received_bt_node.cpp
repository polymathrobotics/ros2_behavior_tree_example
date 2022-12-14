#include <plugins/ping_received_bt_node.h>

namespace bt_ros_example
{
    PingReceivedNode::PingReceivedNode(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    sub_topic_("incoming_ping"),
    ping_id_received_(-1)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

        // Setup subscription on running this node
        // Subscription will run when lifecycle node executor is called 
        node_->create_subscription<std_msgs::msg::Int32>(sub_topic_,
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&PingReceivedNode::ping_callback, this, std::placeholders::_1));

        bool force_ping;
        getInput("force_ping", force_ping);

        if(force_ping)
        {
            // assume we're transmitting the first ping
            ping_id_received_=0;
        }
        return;
    }

    PingReceivedNode::~PingReceivedNode()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN PING RECEIVED NODE");
        sub_.reset();
    }

    BT::NodeStatus PingReceivedNode::tick()
    {
        if(ping_id_received_ <= -1)
        {
            // Return failiure if we haven't received any pings
            return BT::NodeStatus::FAILURE;
        }

        // record last ping id received so that other nodes could use it
        setOutput("last_ping_id", ping_id_received_);

        // reset ping id since we've received it
        ping_id_received_ = -1;
        return BT::NodeStatus::SUCCESS;
    }

    void PingReceivedNode::ping_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "RECEIVED PING WITH ID %d", msg->data);
        ping_id_received_ = msg->data;
        return;
    }
}