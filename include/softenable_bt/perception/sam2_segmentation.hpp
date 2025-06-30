#pragma once
#include <behaviortree_cpp/action_node.h>

class SAM2Segmentation : public BT::SyncActionNode {
public:
    SAM2Segmentation(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};