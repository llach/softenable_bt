#pragma once
#include <behaviortree_cpp/action_node.h>
#include <array>

class MoveJoint : public BT::SyncActionNode {
public:
    MoveJoint(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};
