#pragma once
#include <behaviortree_cpp/action_node.h>
#include "softenable_bt/types/cartesian_target.hpp"

class MoveCartesian : public BT::SyncActionNode {
public:
  MoveCartesian(const std::string& name, const BT::NodeConfiguration& config);
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();
};
