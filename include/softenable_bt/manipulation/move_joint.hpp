#pragma once
#include <behaviortree_cpp/action_node.h>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "stack_msgs/srv/move_arm.hpp"
#include <rclcpp/client.hpp>


class MoveJoint : public BT::SyncActionNode {
public:
    MoveJoint(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();

private:
    rclcpp::Client<stack_msgs::srv::MoveArm>::SharedPtr client_;
    rclcpp::Node::SharedPtr node_;
};
