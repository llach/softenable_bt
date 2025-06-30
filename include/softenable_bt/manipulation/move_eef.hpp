#pragma once
#include <behaviortree_cpp/action_node.h>

class MoveEEF : public BT::SyncActionNode {
public:
    MoveEEF(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};
