#pragma once
#include <behaviortree_cpp/action_node.h>

class DINODetection : public BT::SyncActionNode {
public:
    DINODetection(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};
