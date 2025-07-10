#pragma once
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include "stack_msgs/srv/move_arm.hpp"
#include "softenable_bt/helpers/tf_listener_wrapper.hpp"


class MoveEEF : public BT::SyncActionNode {
public:
    MoveEEF(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Client<stack_msgs::srv::MoveArm>::SharedPtr client_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<softenable_bt::TFListenerWrapper> tf_wrapper_;
};
