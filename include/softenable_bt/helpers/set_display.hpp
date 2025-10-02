#pragma once

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <softenable_display_msgs/srv/set_display.hpp>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>



/**
 * SetDisplaySkill
 * Input ports:
 *  - name (std::string)    : preset name to apply (required)
 *  - service (std::string) : service name to call (optional, default "/set_display")
 */
class SetDisplaySkill : public BT::SyncActionNode
{
public:
  SetDisplaySkill(const std::string& instance_name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void ensureClient(const std::string& service_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<softenable_display_msgs::srv::SetDisplay>::SharedPtr client_;
  std::string current_service_name_;
};
