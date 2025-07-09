#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <stack_msgs/srv/stack_detect.hpp>

class StackDetection : public BT::SyncActionNode
{
public:
  StackDetection(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Client<stack_msgs::srv::StackDetect>> client_;
};
