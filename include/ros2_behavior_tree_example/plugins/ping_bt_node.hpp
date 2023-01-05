#ifndef PING_BT_NODE_H_
#define PING_BT_NODE_H_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"


namespace polymath
{
namespace bt_ros_example
{
    ///
    /// @brief Async Action Node that shoots out a ping
    /// 
    ///
    class PingNode : public BT::StatefulActionNode
    {
    public:
        ///
        /// @brief A constructor for a basic node that checks if a ping message has been received
        /// @param action_name Name for the XML tag for this node
        /// @param conf BT Node Configuration
        ///
        PingNode(const std::string & action_name, const BT::NodeConfig & conf);
        ~PingNode();

        ///
        /// @brief Creates list of BT ports
        /// @return BT::PortsList Containing node-specific ports
        ///
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int32_t>("num_pings"),
                BT::OutputPort<int32_t>("last_ping_id")
            };
        }

        ///
        /// @brief When we first enter this node we "start" the node
        /// @return NodeStatus RUNNING or FAILIURE
        ///
        BT::NodeStatus onStart() override;

        ///
        /// @brief The main behavior call when this node is run
        /// @return NodeStatus RUNNING or SUCCESS 
        ///
        BT::NodeStatus onRunning() override;

        ///
        /// @brief Halts whatever is running next tick
        ///
        void onHalted() override;

        ///
        /// @brief Publishes the current ping
        ///
        void publish();

    private:
        ///
        /// @brief Subscription to the Ping message from a tertiary node
        ///*/
        std::string pub_topic_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        int32_t curr_ping_in_burst_;
        int32_t num_pings_;
        std_msgs::msg::Int32 ping_msg_;
    };
}
}

#endif // PING_BT_NODE_H_
