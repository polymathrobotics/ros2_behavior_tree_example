#ifndef LOG_STATUS_BT_NODE_H_
#define LOG_STATUS_BT_NODE_H_

#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"

namespace polymath
{
namespace bt_ros_example
{
    ///
    ///@brief Async Action Node that shoots out a pong
    ///
    ///
    class LogStatusNode : public BT::SyncActionNode
    {
    public:
        ///
        /// @brief A constructor for a basic node that checks if a ping message has been received
        /// @param action_name Name for the XML tag for this node
        /// @param conf BT Node Configuration
        ///
        LogStatusNode(const std::string & action_name, const BT::NodeConfig & conf);
        
        ~LogStatusNode();

        ///
        /// @brief Creates list of BT ports
        /// @return BT::PortsList Containing node-specific ports
        ///
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("message"),
                BT::InputPort<bool>("print_ping_pong"),
                BT::InputPort<int32_t>("ping_id"),
                BT::InputPort<int32_t>("pong_id")
            };
        }

        ///
        /// @brief Ticking will publish on each tick where this node is called
        /// @return NodeStatus RUNNING or FAILIURE
        ///
        BT::NodeStatus tick() override;

    private:
        ///
        ///@brief Subscription to the Ping message from a tertiary node
        ///
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::string message_;
        bool print_ping_pong_;
    };
}
}

#endif // LOG_STATUS_BT_NODE_H_
