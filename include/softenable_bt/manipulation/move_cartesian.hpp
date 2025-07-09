#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stack_msgs/srv/move_arm.hpp>

class MoveCartesian : public BT::SyncActionNode
{
public:
  MoveCartesian(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<stack_msgs::srv::MoveArm>::SharedPtr client_;
};
