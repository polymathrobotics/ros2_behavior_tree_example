#include "ros2_behavior_tree_example/plugins/ping_received_executor_bt_node.h"

namespace bt_ros_example
{
    PingReceivedExecutorNode::PingReceivedExecutorNode(const std::string & condition_name, const BT::NodeConfig & conf)
    : BT::ConditionNode(condition_name, conf),
    sub_topic_("incoming_ping"),
    ping_id_received_(-1)
    {
        node_ = conf.blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");

        // Create a callback group and executor for this node specifically
        callback_group_ = node_->create_callback_group(
                                rclcpp::CallbackGroupType::MutuallyExclusive,
                                false);
        executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_;

        // Setup subscription on running this node
        // Subscription will run when lifecycle node executor is called 
        sub_ = node_->create_subscription<std_msgs::msg::Int32>(sub_topic_,
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&PingReceivedExecutorNode::ping_callback, this, std::placeholders::_1),
                                                        options);

        bool force_ping;
        getInput("force_ping", force_ping);

        if(force_ping)
        {
            // assume we're transmitting the first ping
            ping_id_received_=0;
        }
        return;
    }

    PingReceivedExecutorNode::~PingReceivedExecutorNode()
    {
        RCLCPP_INFO(node_->get_logger(), "SHUTTING DOWN PING RECEIVED NODE");

        // cancel stops the executor if it's spinning
        executor_.cancel();

        callback_group_.reset();
        sub_.reset();
    }

    BT::NodeStatus PingReceivedExecutorNode::tick()
    {
        // Spin the executor for all work available on the tick
        // After a pause this means that this node will take slightly longer to process all messages
        // Depending on the QOS settings for the subscription

        // However if this is an older version, it will keep doing work and can hang so we may want to add a timeout.
        executor_.spin_some();

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

    void PingReceivedExecutorNode::ping_callback(std_msgs::msg::Int32::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "RECEIVED PING WITH ID %d", msg->data);
        ping_id_received_ = msg->data;
        return;
    }
}
