#ifndef PONG_RECEIVED_NODE_H_
#define PONG_RECEIVED_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"

namespace polymath
{
namespace bt_ros_example
{
    ///
    /// @brief Condition Node that tells us whether a pong has been received
    ///
    ///
    class PongReceivedNode : public BT::ConditionNode
    {
    public:
        ///
        /// @brief A constructor for a basic node that checks if a pong message has been received
        /// @param condition_name Name for the XML tag for this node
        /// @param conf BT Node Configuration
        ///
        PongReceivedNode(const std::string & condition_name, const BT::NodeConfig & conf);
        PongReceivedNode() = delete;
        ~PongReceivedNode();

        ///
        ///@brief Creates list of BT ports
        ///@return BT::PortsList Containing node-specific ports
        ///
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("force_pong"),
                BT::OutputPort<int32_t>("last_pong_id"),
            };
        }

        ///
        /// @brief The main behavior call when this node is run
        /// @return NodeStatus Success or Failiure
        ///
        BT::NodeStatus tick() override;

        ///
        /// @brief Record the last received pong
        ///
       void pong_callback(std_msgs::msg::Int32::SharedPtr msg);

    private:
        ///
        /// @brief Subscription to the pong message from a tertiary node
        ///
        std::string sub_topic_;
        int32_t pong_id_received_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };
}
}

#endif // PONG_RECEIVED_NODE_H_
