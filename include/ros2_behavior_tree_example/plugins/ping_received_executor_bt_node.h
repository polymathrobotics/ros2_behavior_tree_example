#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"

#ifndef PING_RECEIVED_EXECUTOR_NODE_
#define PING_RECEIVED_EXECUTOR_NODE_


namespace bt_ros_example
{
    /**
     * @brief Condition Node that tells us whether a ping has been received
     * 
    */
    class PingReceivedExecutorNode : public BT::ConditionNode
    {
    public:
        /**
         * @brief A constructor for a basic node that checks if a ping message has been received
         * @param condition_name Name for the XML tag for this node
         * @param conf BT Node Configuration
        */
        PingReceivedExecutorNode(const std::string & condition_name, const BT::NodeConfig & conf);
        PingReceivedExecutorNode() = delete;
        ~PingReceivedExecutorNode();

        /**
        * @brief Creates list of BT ports
        * @return BT::PortsList Containing node-specific ports
        */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("force_ping"),
                BT::OutputPort<int32_t>("last_ping_id"),
            };
        }

        /**
         * @brief The main behavior call when this node is run
         * @return NodeStatus Success or Failiure
        */
        BT::NodeStatus tick() override;

        /**
         * @brief Record the last received ping
        */
       void ping_callback(std_msgs::msg::Int32::SharedPtr msg);

    private:
        /**
         * @brief Subscription to the Ping message from a tertiary node
         * */
        std::string                                             sub_topic_;
        rclcpp::executors::SingleThreadedExecutor               executor_;
        rclcpp::CallbackGroup::SharedPtr                        callback_group_;
        int32_t                                                 ping_id_received_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   sub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr              node_;
    };
}

#endif // PING_RECEIVED_EXECUTOR_NODE_