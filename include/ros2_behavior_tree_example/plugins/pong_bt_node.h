#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/int32.hpp"

#ifndef PONG_BT_NODE_H_
#define PONG_BT_NODE_H_


namespace bt_ros_example
{
    /**
     * @brief Async Action Node that shoots out a pong
     * 
    */
    class PongNode : public BT::StatefulActionNode
    {
    public:
        /**
         * @brief A constructor for a basic node that checks if a ping message has been received
         * @param condition_name Name for the XML tag for this node
         * @param conf BT Node Configuration
        */
        PongNode(const std::string & condition_name, const BT::NodeConfig & conf);
        ~PongNode();

        /**
        * @brief Creates list of BT ports
        * @return BT::PortsList Containing node-specific ports
        */
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int32_t>("num_pongs"),
                BT::OutputPort<int32_t>("last_pong_id")
            };
        }

        /**
         * @brief When we first enter this node we "start" the node
         * @return NodeStatus RUNNING or FAILIURE
        */
        BT::NodeStatus onStart() override;

        /**
         * @brief The main behavior call when this node is run
         * @return NodeStatus RUNNING or SUCCESS 
        */
        BT::NodeStatus onRunning() override;

        /**
         * @brief Halts whatever is running next tick
        */
        void onHalted() override;

        /**
         * @brief Publishes the current pong
        */
        void publish();

    private:
        /**
         * @brief Subscription to the Ping message from a tertiary node
         * */
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr      pub_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr              node_;
        int32_t                                                 curr_pong_in_burst_;
        int32_t                                                 num_pongs_;
        std_msgs::msg::Int32                                    pong_msg_;
        std::string                                             pub_topic_;
    };
}

#endif // PONG_BT_NODE_H_
