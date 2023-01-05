#ifndef BT_ROS_NODE_H_
#define BT_ROS_NODE_H_

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace polymath
{
namespace bt_ros_example
{

///
/// @class bt_ros_example::BtRosNode
/// @brief a ros node that implements a behavior tree based to publish and subscribe 
///
class BtRosNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    ///
    ///@brief A constructor for bt_ros_example::BtRosNode
    ///@param opotions Additional options to control the creation of the node
    ///
    explicit BtRosNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ///
    /// @brief a destructor for bt_ros_example::BtRosNode class
    ///
   ~BtRosNode();

protected:
    ///
    /// @brief Configures BtRosNode
    ///
    /// Initializes blackboard and builds behavior tree
    /// Depending on the return value of this function, the LifeCycle state machine
    /// either invokes a transition to the "inactive" state or stays unconfigured
    ///
    /// @param state reference to LifeCycle node State 
    ///
    /// @return SUCCESS or FAILIURE
    ///
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &);

    ///
    /// @brief Activate BtRosNode and start Ticking Behavior Tree
    /// 
    /// wall timer is triggered to trigger behavior tree ticks at
    /// a regular interval
    /// 
    /// @param state reference to LifeCycle node State 
    /// 
    /// @return SUCCESS or FAILIURE
    ///
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state);

    ///
    /// @brief Deactivate BtRosNode and stop Ticking Behavior Tree
    /// 
    /// wall timer is triggered to trigger behavior tree ticks at
    /// a regular interval
    /// 
    /// @param state reference to LifeCycle node State 
    /// 
    /// @return SUCCESS or FAILIURE
    ///
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state);

    ///
    /// @brief Cleans up and resets member variables
    /// 
    /// Cleanup is done to transition the node back to unconfigured state
    /// All configurations are removed and publishers and subscribers should be "released"
    /// 
    /// @param state reference to LifeCycle node State 
    /// 
    /// @return SUCCESS or FAILIURE
    ///
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state);

    ///
    /// @brief Called to finalize node shutdown  
    /// 
    /// @param state reference to LifeCycle node State 
    /// 
    /// @return SUCCESS or FAILIURE
    ///
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state);

    ///
    /// @brief timer callback, ticks timer based on pre set rate
    ///
    void
    timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory_;
};
}
}
#endif //BT_ROS_NODE_H_
