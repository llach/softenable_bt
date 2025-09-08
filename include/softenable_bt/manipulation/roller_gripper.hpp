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
    const std::string LEFT_PREFIX_ = "left";
    const std::string RIGHT_PREFIX_ = "right";
    rclcpp::Node::SharedPtr node_;
    std::vector<std::string> valid_prefixes_;
    rclcpp::Client<stack_msgs::srv::RollerGripper>::SharedPtr left_client_;
    rclcpp::Client<stack_msgs::srv::RollerGripper>::SharedPtr right_client_;
};
