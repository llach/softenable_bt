#pragma once
#include <behaviortree_cpp/action_node.h>
#include <array>

class StoreCurrentJointPos : public BT::SyncActionNode {
public:
    using JointArray = std::array<double, 6>;
    StoreCurrentJointPos(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};
