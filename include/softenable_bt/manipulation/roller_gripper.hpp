#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <stack_msgs/srv/roller_gripper.hpp>

class RollerGripper : public BT::SyncActionNode
{
public:
    RollerGripper(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<stack_msgs::srv::RollerGripper>::SharedPtr client_;
};
